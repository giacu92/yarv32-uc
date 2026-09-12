`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * AXI4-Lite I2C master peripheral.
 *
 * Byte-oriented I2C master with small TX/RX FIFOs, software-controlled
 * START / repeated-START / STOP, programmable SCL divisor, level interrupts,
 * and a 4-phase-per-bit SCL generator with slave clock-stretching tolerance.
 *
 * Protocol follows msip_peri / axi4_lite_uart / axi4_lite_spi: registered
 * BVALID held until B handshake, RVALID held until RREADY, single-outstanding.
 *
 * Register map (word offsets from the peripheral base):
 *
 *   0x00 STATUS  (R)  : bit0 BUSY
 *                        bit1 TX_FULL
 *                        bit2 RX_EMPTY
 *                        bit3 TX_EMPTY
 *                        bit4 RX_FULL
 *                        bit5 NACK      (sticky; slave NACKed a write byte)
 *                        bit6 ARBLOST   (sticky; lost arbitration as master)
 *                        bit7 RX_OVERRUN(sticky; read byte dropped, RX FIFO
 *                              was full; cleared via IRQ bit5)
 *   0x04 CTRL    (RW) : bit0 ENABLE
 *                        bit1 IE_TX
 *                        bit2 IE_RX
 *                        bit3 IE_DONE
 *                        bit4 IE_NACK
 *                        bit5 IE_ARB
 *                        bit6 IE_OVR
 *   0x08 CLKDIV  (RW) : SCL frequency ≈ clk_i / (4*(CLKDIV+1))
 *                        Example @50 MHz: 124 → ~100 kHz, 30 → ~400 kHz
 *   0x0C CMD     (W)  : bit0 START   (generate START or repeated-START)
 *                        bit1 STOP    (generate STOP after the byte/burst)
 *                        bit2 READ    (1 = master read, 0 = master write)
 *                        bit3 NACK    (read: master NACKs this byte; forces
 *                              STOP regardless of the STOP bit)
 *   0x10 TXDATA  (W)  : push byte[7:0] into TX FIFO (ignored if full)
 *   0x14 RXDATA  (R)  : pop byte from RX FIFO (0 if empty)
 *   0x18 IRQ     (R/W1C): bit0 TX_EMPTY, bit1 RX_NOT_EMPTY,
 *                          bit2 DONE, bit3 NACK, bit4 ARBLOST, bit5 RX_OVERRUN
 *                          (bits 0,1 are edge-triggered sticky so a W1C clear
 *                          sticks while the condition holds)
 *
 * Transaction model (one CMD starts one operation):
 *   * WRITE (READ=0): [START if set] then auto-drain TX FIFO one byte at a
 *     time (MSB first, slave-ACK checked per byte).  Stops early on a slave
 *     NACK or when TX empties.  [STOP if set].  DONE.
 *   * READ  (READ=1): [START if set] then ONE byte sampled into RX FIFO;
 *     master drives ACK (NACK bit clear) or NACK (NACK bit set).  A master
 *     NACK always forces STOP.  [STOP if set].  DONE.  For a multi-byte read,
 *     re-issue CMD=READ per byte (NACK+STOP on the last).
 *
 * The address byte is always master-WRITTEN, so a register read is two
 * operations: (1) CMD=START write with TX=[addr+W, reg...] (no STOP, bus left
 * active); (2) CMD=START write with TX=[addr+R] (repeated-START); (3) per-byte
 * CMD=READ|NACK|STOP for the data.  Leaving the bus active (no STOP) between
 * steps holds SCL low so no spurious STOP appears.
 *
 * Pins are driven open-drain style:
 *   scl_oe_o / sda_oe_o = 1 → drive low
 *   scl_oe_o / sda_oe_o = 0 → release (external pull-up)
 * The top must implement:
 *   assign scl_io = scl_oe_o ? 1'b0 : 1'bz;
 *   assign sda_io = sda_oe_o ? 1'b0 : 1'bz;
 * and feed the pin values back as scl_i / sda_i (with synchronizers).
 *
 * Reset is synchronous, active-low.
 */
module axi4_lite_i2c #(
    parameter int unsigned TX_FIFO_DEPTH = 16,
    parameter int unsigned RX_FIFO_DEPTH = 16
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    // I2C pins (open-drain style)
    input  wire scl_i,     // synchronized
    input  wire sda_i,     // synchronized
    output wire scl_oe_o,  // 1 = drive low
    output wire sda_oe_o,  // 1 = drive low

    output wire i2c_irq_o
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;

    // -----------------------------------------------------------------
    // Register offsets (addr[4:2])
    // -----------------------------------------------------------------
    localparam logic [2:0] REG_STATUS = 3'd0;
    localparam logic [2:0] REG_CTRL   = 3'd1;
    localparam logic [2:0] REG_CLKDIV = 3'd2;
    localparam logic [2:0] REG_CMD    = 3'd3;
    localparam logic [2:0] REG_TXDATA = 3'd4;
    localparam logic [2:0] REG_RXDATA = 3'd5;
    localparam logic [2:0] REG_IRQ    = 3'd6;

    // -----------------------------------------------------------------
    // Control / status registers
    // -----------------------------------------------------------------
    logic enable_q;
    logic ie_tx_q, ie_rx_q, ie_done_q, ie_nack_q, ie_arb_q, ie_ovr_q;
    logic [15:0] clkdiv_q;

    // Sticky IRQ flags (single-driver: set by engine-event wires, cleared by
    // REG_IRQ W1C — both in the AXI write block).
    logic irq_tx_q, irq_rx_q, irq_done_q, irq_nack_q, irq_arb_q, irq_ovr_q;

    // -----------------------------------------------------------------
    // FIFOs (power-of-two depths)
    // -----------------------------------------------------------------
    localparam int TX_PTR_W = $clog2(TX_FIFO_DEPTH);
    localparam int RX_PTR_W = $clog2(RX_FIFO_DEPTH);

    logic [7:0] tx_fifo_q[TX_FIFO_DEPTH];
    logic [7:0] rx_fifo_q[RX_FIFO_DEPTH];

    logic [TX_PTR_W:0] tx_wptr_q, tx_rptr_q;
    logic [RX_PTR_W:0] rx_wptr_q, rx_rptr_q;

    wire tx_empty = (tx_wptr_q == tx_rptr_q);
    wire tx_full = (tx_wptr_q[TX_PTR_W] != tx_rptr_q[TX_PTR_W]) &&
        (tx_wptr_q[TX_PTR_W-1:0] == tx_rptr_q[TX_PTR_W-1:0]);
    wire rx_empty = (rx_wptr_q == rx_rptr_q);
    wire rx_full = (rx_wptr_q[RX_PTR_W] != rx_rptr_q[RX_PTR_W]) &&
        (rx_wptr_q[RX_PTR_W-1:0] == rx_rptr_q[RX_PTR_W-1:0]);

    wire [7:0] tx_head = tx_fifo_q[tx_rptr_q[TX_PTR_W-1:0]];

    // -----------------------------------------------------------------
    // I2C engine
    // -----------------------------------------------------------------
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_START,
        ST_SHIFT,
        ST_ACK,
        ST_STOP,
        ST_DONE
    } state_e;

    state_e       state_q;
    logic   [7:0] shift_q;
    logic   [2:0] bit_cnt_q;
    logic   [1:0] phase_q;  // 0=SCL low setup, 1=SCL high (stretch wait),
                            // 2=SCL high sample, 3=SCL low shift
    logic         read_q;  // current byte is a master read
    logic         nack_cmd_q;  // master NACKs this read byte
    logic         stop_after_q;  // generate STOP after this byte/burst
    logic         bus_active_q;  // START issued, no STOP yet (holds SCL low)
    logic         byte_nack_q;  // slave NACKed the current write byte
    logic         busy;

    assign busy = (state_q != ST_IDLE);

    // Command latch from software
    logic cmd_start_q, cmd_stop_q, cmd_read_q, cmd_nack_q;
    logic cmd_valid_q;

    // -----------------------------------------------------------------
    // SCL clock generator (4 phases per SCL bit).
    //
    // phase_q is owned by the engine and reset to 0 on every state entry, so
    // each state begins at phase 0 (a free-running counter entered states at
    // arbitrary phases and lost the first bit's MSB).  div_cnt_q is held at
    // clkdiv_q while IDLE so phase_tick stays low.
    // -----------------------------------------------------------------
    logic [15:0] div_cnt_q;
    wire phase_tick = enable_q && (state_q != ST_IDLE) && (div_cnt_q == 16'd0);

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            div_cnt_q <= '0;
        end else if (!enable_q || state_q == ST_IDLE) begin
            div_cnt_q <= clkdiv_q;
        end else if (phase_tick) begin
            div_cnt_q <= clkdiv_q;
        end else begin
            div_cnt_q <= div_cnt_q - 16'd1;
        end
    end

    // -----------------------------------------------------------------
    // Combinational open-drain outputs.
    //
    // scl_oe/sda_oe are pure functions of (state, phase, shift, ...), all flop
    // outputs, so they are glitch-free within a phase.  Data changes only while
    // SCL is low (phase 0/3), matching I2C.  always_comb + assign avoids the
    // output-wire multi-driver a procedural assign caused.
    // -----------------------------------------------------------------
    logic scl_oe_d, sda_oe_d;

    always_comb begin
        scl_oe_d = 1'b0;  // default: release (idle high)
        sda_oe_d = 1'b0;
        unique case (state_q)
            ST_IDLE: begin
                // Between bytes (bus active, no STOP yet) hold SCL low so a
                // later SCL rise cannot look like a STOP; SDA released.
                if (bus_active_q) scl_oe_d = 1'b1;
            end
            ST_START: begin
                unique case (phase_q)
                    2'd0: begin
                        scl_oe_d = 1'b1;
                        sda_oe_d = 1'b0;
                    end  // SCL low, SDA high
                    2'd1: begin
                        scl_oe_d = 1'b0;
                        sda_oe_d = 1'b0;
                    end  // SCL high, SDA high
                    2'd2: begin
                        scl_oe_d = 1'b0;
                        sda_oe_d = 1'b1;
                    end  // SDA falls while SCL high = START
                    2'd3: begin
                        scl_oe_d = 1'b1;
                        sda_oe_d = 1'b1;
                    end  // SCL low, SDA low (prep)
                    default: ;
                endcase
            end
            ST_SHIFT: begin
                // Write: drive ~shift[7] (MSB first); Read: release for slave.
                // shift_q updates at phase 3 (SCL low), so SDA is set up while
                // SCL is low (phase 0/3) and held stable across the SCL-high
                // sample window (phases 1,2) -- data may only change while SCL
                // is low, else a slave sees a spurious START/STOP.
                unique case (phase_q)
                    2'd0: begin
                        scl_oe_d = 1'b1;  // SCL low, set up bit
                        sda_oe_d = read_q ? 1'b0 : ~shift_q[7];
                    end
                    2'd1, 2'd2: begin
                        scl_oe_d = 1'b0;  // SCL high, SDA stable
                        sda_oe_d = read_q ? 1'b0 : ~shift_q[7];
                    end
                    2'd3: begin
                        scl_oe_d = 1'b1;  // SCL low, shift (next bit set up)
                        sda_oe_d = read_q ? 1'b0 : ~shift_q[7];
                    end
                    default: ;
                endcase
            end
            ST_ACK: begin
                // Write: release SDA so the slave drives ACK/NACK.
                // Read:  master drives ACK (low) or NACK (release high).
                // SCL is low during phase 0/3 (SDA set up / released) and high
                // during phase 1/2 (the slave's ACK is sampled at phase 2).
                unique case (phase_q)
                    2'd0: begin
                        scl_oe_d = 1'b1;  // SCL low
                        sda_oe_d = read_q ? (nack_cmd_q ? 1'b0 : 1'b1) : 1'b0;
                    end
                    2'd1, 2'd2: begin
                        scl_oe_d = 1'b0;  // SCL high
                        sda_oe_d = read_q ? (nack_cmd_q ? 1'b0 : 1'b1) : 1'b0;
                    end
                    2'd3: begin
                        scl_oe_d = 1'b1;  // SCL low, release SDA
                        sda_oe_d = 1'b0;
                    end
                    default: ;
                endcase
            end
            ST_STOP: begin
                unique case (phase_q)
                    2'd0: begin
                        scl_oe_d = 1'b1;
                        sda_oe_d = 1'b1;
                    end  // SCL low, SDA low
                    2'd1: begin
                        scl_oe_d = 1'b0;
                        sda_oe_d = 1'b1;
                    end  // SCL high, SDA low
                    2'd2: begin
                        scl_oe_d = 1'b0;
                        sda_oe_d = 1'b0;
                    end  // SDA rises while SCL high = STOP
                    2'd3: begin
                        scl_oe_d = 1'b0;
                        sda_oe_d = 1'b0;
                    end  // released
                    default: ;
                endcase
            end
            ST_DONE: begin
                // If a STOP was issued bus_active_q is already 0 -> release.
                // Otherwise hold SCL low (no spurious STOP) until the next CMD.
                scl_oe_d = bus_active_q ? 1'b1 : 1'b0;
            end
            default: ;
        endcase
    end

    assign scl_oe_o = scl_oe_d;
    assign sda_oe_o = sda_oe_d;

    // Engine event wires -> the AXI block sets the sticky IRQ flops from these
    // so every IRQ flop has exactly one driver (set here, cleared by W1C).
    wire ev_done = (state_q == ST_DONE);
    wire ev_nack = (state_q == ST_ACK) && (phase_q == 2'd2) && phase_tick && !read_q &&
        sda_i;  // slave NACK on a write byte
    wire ev_arb = (state_q == ST_SHIFT) && (phase_q == 2'd2) && phase_tick && !read_q &&
        !sda_oe_d && !sda_i;  // sent 1 (released) but bus is 0
    wire ev_ovr = (state_q == ST_ACK) && (phase_q == 2'd2) && phase_tick && read_q &&
        rx_full;  // read byte dropped, RX FIFO full

    // Slave clock-stretching: at phase 1 (SCL released) wait for scl_i high
    // before sampling.  Only when a slave is actually addressed.
    wire stretch_hold = (phase_q == 2'd1) && !scl_i && (state_q == ST_SHIFT || state_q == ST_ACK);

    // Command handshake.  cmd_valid_q is SET by the AXI register block (a
    // REG_CMD write) and CLEARED when the engine consumes the command, which
    // would give the flop two driving always_ff blocks -- Gowin rejects that
    // (EX2000 "constantly driven from multiple places").  The engine instead
    // exports the consume condition combinationally and the register block
    // owns the flop; the set in the REG_CMD case comes after the clear below
    // it, so a write landing in the same cycle as a consume is not lost.
    //   idle: a fresh transaction is latched out of ST_IDLE.
    //   ack : a read continues into another byte at ST_ACK phase 3, which is
    //         inside the `phase_tick && !stretch_hold` guard of the engine.
    wire cmd_consume_idle = (state_q == ST_IDLE) && enable_q && cmd_valid_q;
    wire cmd_consume_ack = phase_tick && !stretch_hold && (state_q == ST_ACK) &&
        (phase_q == 2'd3) && read_q && !(stop_after_q || nack_cmd_q) && cmd_valid_q;
    wire cmd_consume = cmd_consume_idle || cmd_consume_ack;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            state_q      <= ST_IDLE;
            phase_q      <= 2'd0;
            shift_q      <= '0;
            bit_cnt_q    <= '0;
            read_q       <= 1'b0;
            nack_cmd_q   <= 1'b0;
            stop_after_q <= 1'b0;
            bus_active_q <= 1'b0;
            byte_nack_q  <= 1'b0;
            // cmd_valid_q / cmd_start_q / cmd_stop_q / cmd_read_q /
            // cmd_nack_q are driven and reset by the AXI register block.
            tx_rptr_q    <= '0;
            rx_wptr_q    <= '0;
        end else begin
            // Command latch: consume in IDLE to start a transaction.  A read
            // continuation CMD is consumed at ST_ACK phase 3 (see below).
            if (cmd_consume_idle) begin
                read_q       <= cmd_read_q;
                nack_cmd_q   <= cmd_nack_q;
                stop_after_q <= cmd_stop_q;
                byte_nack_q  <= 1'b0;
                if (cmd_start_q) begin
                    state_q      <= ST_START;
                    bus_active_q <= 1'b1;
                end else if (cmd_read_q || !tx_empty) begin
                    // Continuation byte (bus already active): load now so
                    // ST_SHIFT enters with the first bit ready.
                    if (!cmd_read_q && !tx_empty) begin
                        shift_q   <= tx_head;
                        tx_rptr_q <= tx_rptr_q + 1'b1;
                    end else begin
                        shift_q <= '0;
                    end
                    bit_cnt_q <= 3'd7;
                    state_q   <= ST_SHIFT;
                end
            end

            if (phase_tick) begin
                if (stretch_hold) begin
                    // Slave holding SCL low: hold phase, divider reloads.
                end else begin
                    unique case (state_q)
                        ST_START: begin
                            unique case (phase_q)
                                2'd3: begin
                                    if (read_q) begin
                                        shift_q     <= '0;
                                        bit_cnt_q   <= 3'd7;
                                        byte_nack_q <= 1'b0;
                                        phase_q     <= 2'd0;
                                        state_q     <= ST_SHIFT;
                                    end else if (!tx_empty) begin
                                        shift_q     <= tx_head;
                                        tx_rptr_q   <= tx_rptr_q + 1'b1;
                                        bit_cnt_q   <= 3'd7;
                                        byte_nack_q <= 1'b0;
                                        phase_q     <= 2'd0;
                                        state_q     <= ST_SHIFT;
                                    end else begin
                                        // Write with no data queued: START then
                                        // STOP, no byte transmitted.
                                        phase_q <= 2'd0;
                                        state_q <= ST_STOP;
                                    end
                                end
                                default: phase_q <= phase_q + 2'd1;
                            endcase
                        end

                        ST_SHIFT: begin
                            unique case (phase_q)
                                2'd2: begin
                                    if (read_q) shift_q <= {shift_q[6:0], sda_i};
                                    // write: arbitration signalled via ev_arb;
                                    // on loss abort to ST_DONE (release bus).
                                    if (!read_q && !sda_oe_d && !sda_i) begin
                                        bus_active_q <= 1'b0;
                                        phase_q      <= 2'd0;
                                        state_q      <= ST_DONE;
                                    end else begin
                                        phase_q <= phase_q + 2'd1;
                                    end
                                end
                                2'd3: begin
                                    if (bit_cnt_q == 3'd0) begin
                                        phase_q <= 2'd0;
                                        state_q <= ST_ACK;
                                    end else begin
                                        if (!read_q) shift_q <= {shift_q[6:0], 1'b0};
                                        bit_cnt_q <= bit_cnt_q - 3'd1;
                                        phase_q   <= 2'd0;
                                    end
                                end
                                default: phase_q <= phase_q + 2'd1;
                            endcase
                        end

                        ST_ACK: begin
                            unique case (phase_q)
                                2'd2: begin
                                    if (!read_q && sda_i) byte_nack_q <= 1'b1;
                                    if (read_q && !rx_full) begin
                                        rx_fifo_q[rx_wptr_q[RX_PTR_W-1:0]] <= shift_q;
                                        rx_wptr_q                          <= rx_wptr_q + 1'b1;
                                    end
                                    phase_q <= phase_q + 2'd1;
                                end
                                2'd3: begin
                                    if (!read_q) begin
                                        // Write: auto-drain until TX empty or
                                        // slave NACK.  STOP fires on a NACK, or
                                        // after the last byte when requested;
                                        // stop_after_q must NOT cut the burst
                                        // short while TX still has bytes.
                                        if (byte_nack_q) begin
                                            // Slave NACKed: abandon the rest of
                                            // the queued write bytes so the next
                                            // transaction starts with an empty
                                            // TX FIFO (the simple push/CMD/wait
                                            // library has no flush register).
                                            tx_rptr_q <= tx_wptr_q;
                                            phase_q   <= 2'd0;
                                            state_q   <= ST_STOP;
                                        end else if (!tx_empty) begin
                                            shift_q     <= tx_head;
                                            tx_rptr_q   <= tx_rptr_q + 1'b1;
                                            bit_cnt_q   <= 3'd7;
                                            byte_nack_q <= 1'b0;
                                            phase_q     <= 2'd0;
                                            state_q     <= ST_SHIFT;
                                        end else if (stop_after_q) begin
                                            phase_q <= 2'd0;
                                            state_q <= ST_STOP;
                                        end else begin
                                            phase_q <= 2'd0;
                                            state_q <= ST_DONE;
                                        end
                                    end else begin
                                        // Read: a master NACK always ends the
                                        // read (I2C).  Otherwise continue only
                                        // if a fresh CMD was queued.
                                        if (stop_after_q || nack_cmd_q) begin
                                            phase_q <= 2'd0;
                                            state_q <= ST_STOP;
                                        end else if (cmd_consume_ack) begin
                                            nack_cmd_q   <= cmd_nack_q;
                                            stop_after_q <= cmd_stop_q;
                                            bit_cnt_q    <= 3'd7;
                                            phase_q      <= 2'd0;
                                            state_q      <= ST_SHIFT;
                                        end else begin
                                            phase_q <= 2'd0;
                                            state_q <= ST_DONE;
                                        end
                                    end
                                end
                                default: phase_q <= phase_q + 2'd1;
                            endcase
                        end

                        ST_STOP: begin
                            unique case (phase_q)
                                2'd3: begin
                                    bus_active_q <= 1'b0;
                                    phase_q      <= 2'd0;
                                    state_q      <= ST_DONE;
                                end
                                default: phase_q <= phase_q + 2'd1;
                            endcase
                        end

                        ST_DONE: begin
                            phase_q <= 2'd0;
                            state_q <= ST_IDLE;
                        end

                        default: state_q <= ST_IDLE;
                    endcase
                end
            end
        end
    end

    // -----------------------------------------------------------------
    // Interrupt
    // -----------------------------------------------------------------
    wire tx_empty_irq = tx_empty;
    wire rx_not_empty = !rx_empty;

    assign
        i2c_irq_o = (tx_empty_irq & ie_tx_q) | (rx_not_empty & ie_rx_q) | (irq_done_q & ie_done_q) |
        (irq_nack_q & ie_nack_q) | (irq_arb_q & ie_arb_q) | (irq_ovr_q & ie_ovr_q);

    // -----------------------------------------------------------------
    // AXI write path
    // -----------------------------------------------------------------
    logic aw_seen_q, w_seen_q, bvalid_q;
    logic [DATA_W-1:0] wdata_q;
    logic [STRB_W-1:0] wstrb_q;
    logic [       2:0] awaddr_word_q;
    logic tx_empty_d_q, rx_ne_d_q;  // edge detect for tx/rx IRQ

    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;

    wire              aw_hs = axi.awvalid && axi.awready;
    wire              w_hs = axi.wvalid && axi.wready;
    wire              do_write = (aw_seen_q || aw_hs) && (w_seen_q || w_hs) && !bvalid_q;

    wire [       2:0] waddr_eff = aw_hs ? axi.awaddr[4:2] : awaddr_word_q;
    wire [DATA_W-1:0] wdata_eff = w_hs ? axi.wdata : wdata_q;
    wire [STRB_W-1:0] wstrb_eff = w_hs ? axi.wstrb : wstrb_q;
    wire              wr_low_byte = wstrb_eff[0];

    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;
    wire b_hs = axi.bvalid && axi.bready;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q     <= 1'b0;
            w_seen_q      <= 1'b0;
            bvalid_q      <= 1'b0;
            wdata_q       <= '0;
            wstrb_q       <= '0;
            awaddr_word_q <= '0;

            enable_q      <= 1'b0;
            ie_tx_q       <= 1'b0;
            ie_rx_q       <= 1'b0;
            ie_done_q     <= 1'b0;
            ie_nack_q     <= 1'b0;
            ie_arb_q      <= 1'b0;
            ie_ovr_q      <= 1'b0;
            clkdiv_q      <= 16'd124;  // ~100 kHz @ 50 MHz

            tx_wptr_q     <= '0;
            // tx_rptr_q / rx_wptr_q reset in the engine; rx_rptr_q in the read
            // path — each pointer has exactly one driver.

            cmd_valid_q   <= 1'b0;
            cmd_start_q   <= 1'b0;
            cmd_stop_q    <= 1'b0;
            cmd_read_q    <= 1'b0;
            cmd_nack_q    <= 1'b0;

            irq_tx_q      <= 1'b0;
            irq_rx_q      <= 1'b0;
            irq_done_q    <= 1'b0;
            irq_nack_q    <= 1'b0;
            irq_arb_q     <= 1'b0;
            irq_ovr_q     <= 1'b0;
            tx_empty_d_q  <= 1'b0;
            rx_ne_d_q     <= 1'b0;
        end else begin
            if (aw_hs) begin
                aw_seen_q     <= 1'b1;
                awaddr_word_q <= axi.awaddr[4:2];
            end
            if (w_hs) begin
                w_seen_q <= 1'b1;
                wdata_q  <= axi.wdata;
                wstrb_q  <= axi.wstrb;
            end

            // Engine consumed the queued command.  Placed before the write
            // decode so a REG_CMD write in the same cycle still wins.
            if (cmd_consume) cmd_valid_q <= 1'b0;

            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;

                if (wr_low_byte) begin
                    unique case (waddr_eff)
                        REG_CTRL: begin
                            enable_q  <= wdata_eff[0];
                            ie_tx_q   <= wdata_eff[1];
                            ie_rx_q   <= wdata_eff[2];
                            ie_done_q <= wdata_eff[3];
                            ie_nack_q <= wdata_eff[4];
                            ie_arb_q  <= wdata_eff[5];
                            ie_ovr_q  <= wdata_eff[6];
                        end
                        REG_CLKDIV: clkdiv_q <= wdata_eff[15:0];
                        REG_CMD: begin
                            cmd_start_q <= wdata_eff[0];
                            cmd_stop_q  <= wdata_eff[1];
                            cmd_read_q  <= wdata_eff[2];
                            cmd_nack_q  <= wdata_eff[3];
                            cmd_valid_q <= 1'b1;
                        end
                        REG_TXDATA: begin
                            if (!tx_full) begin
                                tx_fifo_q[tx_wptr_q[TX_PTR_W-1:0]] <= wdata_eff[7:0];
                                tx_wptr_q                          <= tx_wptr_q + 1'b1;
                            end
                        end
                        REG_IRQ: begin
                            if (wdata_eff[0]) irq_tx_q <= 1'b0;
                            if (wdata_eff[1]) irq_rx_q <= 1'b0;
                            if (wdata_eff[2]) irq_done_q <= 1'b0;
                            if (wdata_eff[3]) irq_nack_q <= 1'b0;
                            if (wdata_eff[4]) irq_arb_q <= 1'b0;
                            if (wdata_eff[5]) irq_ovr_q <= 1'b0;
                        end
                        default:    ;
                    endcase
                end
            end

            if (b_hs) bvalid_q <= 1'b0;

            // Sticky IRQ flags — single driver.  tx/rx edge-triggered so a W1C
            // clear sticks while the condition holds; done/nack/arb/ovr fire
            // once per event off the engine wires above.
            tx_empty_d_q <= tx_empty;
            rx_ne_d_q    <= rx_not_empty;
            if (ev_done) irq_done_q <= 1'b1;
            if (ev_nack) irq_nack_q <= 1'b1;
            if (ev_arb) irq_arb_q <= 1'b1;
            if (ev_ovr) irq_ovr_q <= 1'b1;
            if (tx_empty && !tx_empty_d_q) irq_tx_q <= 1'b1;
            if (rx_not_empty && !rx_ne_d_q) irq_rx_q <= 1'b1;
        end
    end

    // -----------------------------------------------------------------
    // AXI read path
    // -----------------------------------------------------------------
    logic              rvalid_q;
    logic [DATA_W-1:0] rdata_q;

    assign axi.arready = !rvalid_q || (rvalid_q && axi.rready);
    wire ar_hs = axi.arvalid && axi.arready;

    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q  <= 1'b0;
            rdata_q   <= '0;
            rx_rptr_q <= '0;
        end else begin
            if (ar_hs) begin
                rvalid_q <= 1'b1;
                unique case (axi.araddr[4:2])
                    REG_STATUS:
                    rdata_q <= {
                        24'b0,
                        irq_ovr_q,
                        irq_arb_q,
                        irq_nack_q,
                        rx_full,
                        tx_empty,
                        rx_empty,
                        tx_full,
                        busy
                    };
                    REG_CTRL:
                    rdata_q <= {
                        25'b0, ie_ovr_q, ie_arb_q, ie_nack_q, ie_done_q, ie_rx_q, ie_tx_q, enable_q
                    };
                    REG_CLKDIV: rdata_q <= {16'b0, clkdiv_q};
                    REG_RXDATA: begin
                        if (!rx_empty) begin
                            rdata_q   <= {24'b0, rx_fifo_q[rx_rptr_q[RX_PTR_W-1:0]]};
                            rx_rptr_q <= rx_rptr_q + 1'b1;
                        end else rdata_q <= '0;
                    end
                    REG_IRQ:
                    rdata_q <= {
                        26'b0, irq_ovr_q, irq_arb_q, irq_nack_q, irq_done_q, irq_rx_q, irq_tx_q
                    };
                    default: rdata_q <= '0;
                endcase
            end else if (rvalid_q && axi.rready) begin
                rvalid_q <= 1'b0;
            end
        end
    end

endmodule

`resetall

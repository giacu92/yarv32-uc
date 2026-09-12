`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * AXI4-Lite SPI master peripheral.
 *
 * Simple byte-oriented SPI master with small TX/RX FIFOs, software-controlled
 * chip-select, and programmable clock divisor + mode (CPOL/CPHA).
 *
 * Protocol follows msip_peri / axi4_lite_uart: registered BVALID held until
 * the B handshake, RVALID held until RREADY, single-outstanding
 * (AWREADY/WREADY/ARREADY low while a response is pending).
 *
 * Register map (word offsets from the peripheral base):
 *
 *   0x00 STATUS  (R)  : bit0 BUSY
 *                        bit1 TX_FULL
 *                        bit2 RX_EMPTY
 *                        bit3 TX_EMPTY
 *                        bit4 RX_FULL
 *                        bit5 RX_OVERRUN (sticky; a byte was dropped because
 *                              the RX FIFO was full at ST_DONE; cleared via
 *                              IRQ bit3 write-1-to-clear)
 *   0x04 CTRL    (RW) : bit0 ENABLE
 *                        bit1 CPOL
 *                        bit2 CPHA
 *                        bit3 IE_TX   (interrupt on TX empty)
 *                        bit4 IE_RX   (interrupt on RX not empty)
 *                        bit5 IE_DONE (interrupt when a byte finishes)
 *                        bit6 IE_OVR  (interrupt on RX overrun)
 *   0x08 CLKDIV  (RW) : SPI clock = clk_i / (2*(CLKDIV+1)). 0 = clk/2.
 *   0x0C CS      (RW) : bit0 = chip-select (0 = active/low)
 *   0x10 TXDATA  (W)  : push byte[7:0] into TX FIFO (ignored if full)
 *   0x14 RXDATA  (R)  : pop byte from RX FIFO (returns 0 if empty)
 *   0x18 IRQ     (R/W1C): bit0 TX_EMPTY  bit1 RX_NOT_EMPTY
 *                        bit2 DONE       bit3 RX_OVERRUN
 *                        (write 1 to clear; tx/rx bits are edge-triggered so a
 *                        clear sticks while the condition holds)
 *
 * Interrupt is level-sensitive:
 *   spi_irq_o = (TX_EMPTY & IE_TX) | (RX_NOT_EMPTY & IE_RX) | (DONE & IE_DONE)
 *             | (RX_OVERRUN & IE_OVR)
 *
 * Reset is synchronous, active-low.
 *
 * Naming: ports *_i/_o; internals no prefix; flops _q.
 */
module axi4_lite_spi #(
    parameter int unsigned TX_FIFO_DEPTH = 16,
    parameter int unsigned RX_FIFO_DEPTH = 16
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    // SPI pins
    output wire spi_sck_o,
    output wire spi_mosi_o,
    input  wire spi_miso_i,
    output wire spi_cs_n_o,

    // Level-sensitive interrupt
    output wire spi_irq_o
);

    localparam int         DATA_W     = axi.DATA_WIDTH;
    localparam int         STRB_W     = DATA_W / 8;

    // -----------------------------------------------------------------
    // Register offsets (addr[4:2])
    // -----------------------------------------------------------------
    localparam logic [2:0] REG_STATUS = 3'd0;
    localparam logic [2:0] REG_CTRL   = 3'd1;
    localparam logic [2:0] REG_CLKDIV = 3'd2;
    localparam logic [2:0] REG_CS     = 3'd3;
    localparam logic [2:0] REG_TXDATA = 3'd4;
    localparam logic [2:0] REG_RXDATA = 3'd5;
    localparam logic [2:0] REG_IRQ    = 3'd6;

    // -----------------------------------------------------------------
    // Control / status registers
    // -----------------------------------------------------------------
    logic        enable_q;
    logic        cpol_q;
    logic        cpha_q;
    logic        ie_tx_q;
    logic        ie_rx_q;
    logic        ie_done_q;
    logic [15:0] clkdiv_q;
    logic        cs_n_q;

    logic        irq_tx_q;
    logic        irq_rx_q;
    logic        irq_done_q;
    logic        irq_ovr_q;
    logic        ie_ovr_q;
    logic        tx_empty_d_q;
    logic        rx_ne_d_q;

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
    // SPI engine
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE,
        ST_SHIFT,
        ST_DONE
    } state_e;

    state_e        state_q;
    logic   [ 7:0] shift_q;   // TX shift register; MOSI = shift_q[7] (MSB first)
    logic   [ 7:0] rx_q;      // RX capture register (separate from TX)
    logic   [ 2:0] bit_cnt_q;
    logic          first_edge_q;
    logic   [15:0] div_cnt_q;
    logic          sck_q;
    logic          busy;

    assign busy = (state_q != ST_IDLE);

    // Combinational engine events sampled by the AXI/IRQ block so the sticky
    // IRQ flops have a single driver (set here, cleared by W1C in the AXI block).
    wire byte_done = (state_q == ST_DONE);
    wire rx_overrun = (state_q == ST_DONE) && rx_full;

    // Clock generation
    wire div_tick = (div_cnt_q == 16'd0);

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            div_cnt_q <= '0;
            sck_q     <= 1'b0;
        end else if (!enable_q || state_q == ST_IDLE) begin
            div_cnt_q <= clkdiv_q;
            sck_q     <= cpol_q;  // idle level
        end else if (div_tick) begin
            div_cnt_q <= clkdiv_q;
            sck_q     <= ~sck_q;
        end else begin
            div_cnt_q <= div_cnt_q - 16'd1;
        end
    end

    // Main SPI state machine
    //
    // One shift register per direction: shift_q drives MOSI (combinational
    // MSB) and shifts left once per bit on the CHANGE edge; rx_q captures
    // MISO on the SAMPLE edge.  Decoupling them avoids the double-shift that
    // corrupted MOSI when a single register shifted on both edges.
    //
    // Edge convention (matches the RTL's registered-sck_q view, where the
    // "leading edge" label is the div_tick with sck_q != cpol, physically
    // falling for CPOL=0 / rising for CPOL=1):
    //   CPHA=0 -> sample on leading, change on trailing
    //   CPHA=1 -> change on leading, sample on trailing
    // MOSI is preset with bit 7 at IDLE, so for CPHA=0 the first CHANGE edge
    // (the very first div_tick, a trailing edge) is a phantom that must not
    // shift; first_edge_q suppresses exactly that one edge.  For CPHA=1 the
    // first div_tick is a SAMPLE edge (real -- the slave presets MISO), so
    // the phantom guard only applies to the change branch.
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            state_q      <= ST_IDLE;
            shift_q      <= '0;
            rx_q         <= '0;
            bit_cnt_q    <= '0;
            first_edge_q <= 1'b0;
            // FIFO pointers written by this block (TX pop on launch, RX push
            // on byte-done) reset here too: a flop must have exactly one
            // driving always_ff, and Gowin errors (EX2000) on a split where
            // the reset lives in one block and the update in another.
            tx_rptr_q    <= '0;
            rx_wptr_q    <= '0;
        end else begin
            case (state_q)
                ST_IDLE: begin
                    if (enable_q && !tx_empty) begin
                        shift_q      <= tx_head;
                        bit_cnt_q    <= 3'd7;
                        first_edge_q <= 1'b1;  // suppress phantom first change edge
                        state_q      <= ST_SHIFT;
                        // pop TX
                        tx_rptr_q    <= tx_rptr_q + 1'b1;
                    end
                end

                ST_SHIFT: begin
                    if (div_tick) first_edge_q <= 1'b0;

                    // CHANGE edge: shift MOSI left, advance bit counter.
                    // CPHA=0 -> trailing (sck_q == cpol); CPHA=1 -> leading.
                    // The first change edge for CPHA=0 is phantom (MOSI preset).
                    if (div_tick && (cpha_q ? (sck_q != cpol_q) : (sck_q == cpol_q)) &&
                        bit_cnt_q != 3'd0 && !(first_edge_q && !cpha_q)) begin
                        shift_q   <= {shift_q[6:0], 1'b0};
                        bit_cnt_q <= bit_cnt_q - 3'd1;
                    end

                    // SAMPLE edge: capture MISO.  CPHA=0 -> leading; CPHA=1 ->
                    // trailing.  The 8th sample (bit_cnt_q == 0) finishes the byte.
                    if (div_tick && (cpha_q ? (sck_q == cpol_q) : (sck_q != cpol_q))) begin
                        rx_q <= {rx_q[6:0], spi_miso_i};
                        if (bit_cnt_q == 3'd0) state_q <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    // Push received byte into RX FIFO if space
                    if (!rx_full) begin
                        rx_fifo_q[rx_wptr_q[RX_PTR_W-1:0]] <= rx_q;
                        rx_wptr_q                          <= rx_wptr_q + 1'b1;
                    end
                    // irq_done_q / irq_ovr_q are set in the AXI block from
                    // byte_done / rx_overrun (single driver).
                    state_q <= ST_IDLE;
                end

                default: state_q <= ST_IDLE;
            endcase

            // Clear DONE flag on write-1
            // (handled in the AXI write path below)
        end
    end

    assign spi_sck_o  = sck_q;
    assign spi_mosi_o = shift_q[7];
    assign spi_cs_n_o = cs_n_q;

    // -----------------------------------------------------------------
    // Interrupt
    // -----------------------------------------------------------------
    wire tx_empty_irq = tx_empty;
    wire rx_not_empty = !rx_empty;

    assign spi_irq_o = (tx_empty_irq & ie_tx_q) | (rx_not_empty & ie_rx_q) |
        (irq_done_q & ie_done_q) | (irq_ovr_q & ie_ovr_q);

    // -----------------------------------------------------------------
    // AXI write path (same pattern as uart / msip)
    // -----------------------------------------------------------------
    logic              aw_seen_q;
    logic              w_seen_q;
    logic [DATA_W-1:0] wdata_q;
    logic [STRB_W-1:0] wstrb_q;
    logic [       2:0] awaddr_word_q;
    logic              bvalid_q;

    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;

    wire              aw_hs = axi.awvalid && axi.awready;
    wire              w_hs = axi.wvalid && axi.wready;

    wire              aw_present = aw_seen_q || aw_hs;
    wire              w_present = w_seen_q || w_hs;
    wire              do_write = aw_present && w_present && !bvalid_q;

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
            wdata_q       <= '0;
            wstrb_q       <= '0;
            awaddr_word_q <= '0;
            bvalid_q      <= 1'b0;

            enable_q      <= 1'b0;
            cpol_q        <= 1'b0;
            cpha_q        <= 1'b0;
            ie_tx_q       <= 1'b0;
            ie_rx_q       <= 1'b0;
            ie_done_q     <= 1'b0;
            clkdiv_q      <= 16'd1;  // safe default
            cs_n_q        <= 1'b1;  // deselected

            tx_wptr_q     <= '0;
            // tx_rptr_q / rx_wptr_q reset in the shift engine; rx_rptr_q in
            // the read path -- each pointer is reset by the block that writes
            // it, so every flop has a single driver.

            irq_tx_q      <= 1'b0;
            irq_rx_q      <= 1'b0;
            irq_done_q    <= 1'b0;
            irq_ovr_q     <= 1'b0;
            ie_ovr_q      <= 1'b0;
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

            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;

                if (wr_low_byte) begin
                    unique case (waddr_eff)
                        REG_CTRL: begin
                            enable_q  <= wdata_eff[0];
                            cpol_q    <= wdata_eff[1];
                            cpha_q    <= wdata_eff[2];
                            ie_tx_q   <= wdata_eff[3];
                            ie_rx_q   <= wdata_eff[4];
                            ie_done_q <= wdata_eff[5];
                            ie_ovr_q  <= wdata_eff[6];
                        end
                        REG_CLKDIV: clkdiv_q <= wdata_eff[15:0];
                        REG_CS:     cs_n_q <= wdata_eff[0];
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
                            if (wdata_eff[3]) irq_ovr_q <= 1'b0;
                        end
                        default:    ;
                    endcase
                end
            end

            if (b_hs) bvalid_q <= 1'b0;

            // Sticky IRQ flags.  tx/rx are edge-triggered so a W1C clear sticks
            // while the condition holds (the previous code re-asserted every
            // cycle, making the bits read-only).  done/overrun fire once per
            // byte off the engine events above; this is the single driver for
            // all four flops (set here, cleared by the REG_IRQ W1C above).
            tx_empty_d_q <= tx_empty;
            rx_ne_d_q    <= rx_not_empty;
            if (byte_done) irq_done_q <= 1'b1;
            if (rx_overrun) irq_ovr_q <= 1'b1;
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
            rx_rptr_q <= '0;  // RX pop lives here, so its reset does too
        end else begin
            if (ar_hs) begin
                rvalid_q <= 1'b1;
                unique case (axi.araddr[4:2])
                    REG_STATUS:
                    rdata_q <= {26'b0, irq_ovr_q, rx_full, tx_empty, rx_empty, tx_full, busy};
                    REG_CTRL:
                    rdata_q <= {
                        25'b0, ie_ovr_q, ie_done_q, ie_rx_q, ie_tx_q, cpha_q, cpol_q, enable_q
                    };
                    REG_CLKDIV: rdata_q <= {16'b0, clkdiv_q};
                    REG_CS: rdata_q <= {31'b0, cs_n_q};
                    REG_RXDATA: begin
                        if (!rx_empty) begin
                            rdata_q   <= {24'b0, rx_fifo_q[rx_rptr_q[RX_PTR_W-1:0]]};
                            rx_rptr_q <= rx_rptr_q + 1'b1;
                        end else begin
                            rdata_q <= '0;
                        end
                    end
                    REG_IRQ: rdata_q <= {28'b0, irq_ovr_q, irq_done_q, irq_rx_q, irq_tx_q};
                    default: rdata_q <= '0;
                endcase
            end else if (rvalid_q && axi.rready) begin
                rvalid_q <= 1'b0;
            end
        end
    end

endmodule

`resetall

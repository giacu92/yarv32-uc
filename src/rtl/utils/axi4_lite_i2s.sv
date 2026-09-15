`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * I2S receiver, bus SLAVE (AXI4-Lite MMIO slave, I2S clock slave).
 *
 * On the CPU side it is an AXI4-Lite slave, like every other peripheral in
 * this tree. On the audio side it does either role, selected by
 * CTRL.MASTER:
 *   - MASTER=0 (reset): I2S clock SLAVE. The external device supplies BCLK
 *     and LRCK/WS and this block only samples them; the two clock pads stay
 *     released.
 *   - MASTER=1: this block GENERATES BCLK and LRCK from clk_i and drives
 *     them out (i2s_clk_oe_o), which is what an INMP441 -- or any other mic
 *     that is itself a clock slave -- needs. Two slaves have nobody to
 *     clock them, and that is not a problem firmware can solve.
 *
 * SD is an input in both modes: this is a receiver, there is no transmit
 * path.
 *
 * Receive only: there is no SD output and no TX FIFO. The intended chain
 * is mic -> I2S -> RX FIFO -> CPU -> FFT coprocessor.
 *
 * ================================================================
 * MASTER MODE
 * ================================================================
 * BCLK = clk_i / (2 * (CLKDIV.DIV + 1)), LRCK toggles every CLKDIV.SLOT
 * BCLK periods, so a frame is 2 * SLOT bit clocks and
 * fs = BCLK / (2 * SLOT). LRCK changes on the FALLING BCLK edge, the
 * transmitter convention every I2S device expects.
 *
 * THE RECEIVE PATH IS UNCHANGED IN MASTER MODE, deliberately: the sampler
 * still reads the (synchronized) INPUT pins, so in master mode it sees its
 * own clocks coming back off the pads, delayed by the pad and the
 * synchronizer exactly as the device's SD is. Relative timing is preserved
 * because both legs take the same path, and there is one sampler to reason
 * about instead of two.
 *
 * On a 50 MHz core the useful setting is DIV=24, SLOT=32: BCLK = 1.000 MHz
 * exactly and fs = 15625 Hz exactly. That is worth not losing -- 50e6 =
 * 2^7 x 5^8, so an integer fs out of a 64-BCLK frame needs (DIV+1) to be a
 * power of 5, and 25 is the only one that lands in the audio band (5 gives
 * 78125 Hz, 125 gives 3125 Hz). 16 kHz exactly is NOT reachable from this
 * clock with a 64-BCLK frame.
 *
 * ================================================================
 * CLOCK DOMAINS -- the one thing to get right
 * ================================================================
 * BCLK is NOT this design's clock. The core runs at CLK_CORE_HZ (50 MHz)
 * and BCLK arrives asynchronously from the outside (3.072 MHz for
 * 48 kHz x 32 bits x 2 channels, 1.024 MHz for a 16 kHz 32-bit frame).
 * Rather than introduce a second clock domain into a design that has none
 * (see top_module: single domain, no CDC), the receiver OVERSAMPLES the
 * audio bus in the core domain: it edge-detects the synchronized BCLK and
 * samples SD on each detected rising edge, which is where an I2S
 * transmitter guarantees its data valid (data changes on the falling
 * edge).
 *
 * That imposes a frequency requirement, and it is the thing that breaks
 * first if someone runs this at an unusual rate:
 *
 *      f(clk_i) >= 4 x f(BCLK)        (2 core cycles per BCLK half-period)
 *
 * At 50 MHz that allows BCLK up to 12.5 MHz, well above anything a
 * 48 kHz / 32-bit stereo frame needs. A `ifdef VERILATOR assertion below
 * fails the simulation if a BCLK half-period is shorter than 2 core
 * cycles, so a too-fast master is caught in a testbench instead of on the
 * board.
 *
 * In MASTER mode the ratio is met by construction (the generator divides
 * clk_i by at least 2 per half period), so it only binds on an external
 * master.
 *
 * i2s_bclk_i / i2s_lrck_i / i2s_sd_i must arrive ALREADY SYNCHRONIZED --
 * the caller owns the 2-flop synchronizer, same contract as gpio_i in
 * axi4_lite_gpio (top_module double-flops each pad; sim_top does the same
 * so the board's chain is exercised). All three lines go through the same
 * depth of synchronizer, so their relative skew is preserved and SD is
 * sampled with the edge it belongs to.
 *
 * ================================================================
 * FRAME FORMAT
 * ================================================================
 * Two formats, selected by CTRL.FORMAT:
 *
 *   FORMAT=0, I2S (Philips): WS changes one BCLK BEFORE the MSB of the
 *             word it announces. So the edge at which WS changes carries
 *             no bit of the new word; collection starts on the NEXT edge.
 *   FORMAT=1, left-justified: the MSB is valid ON the edge where WS
 *             changes.
 *
 * In both cases WS low = left channel, WS high = right (Philips
 * convention), data is MSB first, and any bits between the WLEN-th bit
 * and the next WS transition are padding and are discarded. The receiver
 * therefore does not care about the frame length: a 32-BCLK slot carrying
 * a 24-bit word works with WLEN=24, and so does a 64-BCLK slot.
 *
 * Resynchronization is automatic and free: the bit counter is reloaded by
 * every WS transition, so a receiver that is enabled mid-frame (or that
 * lost lock) is aligned again by the next channel boundary, and at most
 * one word is discarded.
 *
 * ================================================================
 * REGISTER MAP (word offsets from the peripheral base)
 * ================================================================
 *   0x00 STATUS (R):
 *          bit0     RX_EMPTY
 *          bit1     RX_FULL
 *          bit2     RX_OVERRUN (sticky; a word was dropped because the
 *                   FIFO was full. Cleared by W1C on IRQ bit1 -- it is
 *                   the same flop seen from two registers)
 *          bit3     RX_CH      channel of the word at the FIFO head
 *                              (0 = left, 1 = right; reads 0 when empty)
 *          bit4     WS         live LRCK level (synchronized)
 *          bit5     BCLK       live BCLK level (synchronized). With bit4
 *                              this answers "is the master clocking at
 *                              all", which is the first bring-up question
 *                              on a silent I2S link.
 *          bit12:8  RX_LEVEL   words currently in the FIFO (0..DEPTH)
 *   0x04 CTRL   (RW):
 *          bit0     ENABLE     0 = receiver idle, FIFO not written
 *          bit2:1   CHAN       0 = stereo (both channels queued, left
 *                              first within a frame), 1 = left only,
 *                              2 = right only, 3 = stereo
 *          bit3     FORMAT     0 = I2S (1-BCLK delay), 1 = left-justified
 *          bit4     IE_RX      interrupt when RX_LEVEL >= WATER
 *          bit5     IE_OVR     interrupt on RX_OVERRUN
 *          bit6     MASTER     1 = generate BCLK/LRCK and drive them out
 *                              (i2s_clk_oe_o); 0 = listen only.
 *          bit9:8   WLEN       0 = 16 bits, 1 = 24, 2 = 32, 3 = 8.
 *                              Reset 0. Not a free-form bit count: the
 *                              four encodings are what sign extension
 *                              costs a 4-way mux instead of two 32-bit
 *                              barrel shifters, and 16/24/32 is what real
 *                              I2S devices emit.
 *   0x08 WATER  (RW): RX interrupt watermark in words. Reset 1. A WATER
 *                     of 0 behaves as 1 (a watermark of "zero or more"
 *                     samples would assert forever).
 *   0x0C RXDATA (R)  : pop one word, SIGN-EXTENDED from WLEN to 32 bits.
 *                      Reads 0 when empty (no underflow flag, same as the
 *                      SPI RX FIFO). THE READ POPS -- read STATUS.RX_CH
 *                      first if the channel matters.
 *   0x14 CLKDIV (RW) : master-mode clock generation. Ignored when
 *                      MASTER=0.
 *          bit15:0  DIV        BCLK = clk_i / (2*(DIV+1)). Reset 24, i.e.
 *                              1 MHz from a 50 MHz core.
 *          bit23:16 SLOT       BCLK periods per channel slot, 8..64. Reset
 *                              32 (a 64-BCLK frame); 0 reads as 32. Only
 *                              the generator uses it -- the RECEIVER never
 *                              needs a slot length, because a word ends at
 *                              WLEN bits and the rest is padding.
 *   0x10 IRQ    (R/W1C):
 *          bit0     RX        LEVEL, not latched: RX_LEVEL >= WATER.
 *                             Writing it does nothing.
 *          bit1     OVERRUN   sticky, write 1 to clear.
 *
 *   i2s_irq_o = (RX_LEVEL >= WATER & IE_RX) | (OVERRUN & IE_OVR)
 *
 * WHY THE RX BIT IS A LEVEL AND NOT AN EDGE-LATCHED FLAG (axi4_lite_spi
 * latches its equivalent): for a receiver the FIFO IS the event record.
 * A level cannot be lost and needs no clear, so an ISR that drains fewer
 * words than arrived is re-entered instead of hanging with a full FIFO
 * and a cleared flag. Only the overrun, which has no other record, is
 * sticky.
 *
 * Protocol follows msip_peri / axi4_lite_uart / axi4_lite_gpio:
 * registered BVALID held until the B handshake, RVALID held until RREADY,
 * single-outstanding (AWREADY/WREADY/ARREADY low while a response is
 * pending). Sub-word writes merge by strobe lane (the axi4_lite_gpio
 * rule: the LSU leaves shifted rs2 bits on unstrobed lanes, so those
 * lanes must never be written).
 *
 * Reset is synchronous, active-low.
 *
 * Naming: ports *_i/_o; internals no prefix; flops _q, next-state _d.
 */
module axi4_lite_i2s #(
    parameter int unsigned RX_FIFO_DEPTH = 16
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    // I2S bus. All three lines are SAMPLED as inputs in both modes and must
    // be pre-synchronized by the caller (see header); in master mode the
    // two clock lines are also DRIVEN, and the caller ties each pad
    // together (drive when i2s_clk_oe_o, always sample the pad).
    input  wire i2s_bclk_i,
    input  wire i2s_lrck_i,
    input  wire i2s_sd_i,
    output wire i2s_bclk_o,
    output wire i2s_lrck_o,
    output wire i2s_clk_oe_o,

    // Level-sensitive interrupt (see header comment).
    output wire i2s_irq_o
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;

    localparam int RX_PTR_W = $clog2(RX_FIFO_DEPTH);
    localparam int LVL_W = RX_PTR_W + 1;

    // Word offsets (byte addr[4:2], word-aligned).
    localparam logic [2:0] REG_STATUS = 3'd0;
    localparam logic [2:0] REG_CTRL = 3'd1;
    localparam logic [2:0] REG_WATER = 3'd2;
    localparam logic [2:0] REG_RXDATA = 3'd3;
    localparam logic [2:0] REG_IRQ = 3'd4;
    localparam logic [2:0] REG_CLKDIV = 3'd5;

    // Channel select encodings (CTRL bit2:1).
    localparam logic [1:0] CHAN_STEREO = 2'd0;
    localparam logic [1:0] CHAN_LEFT = 2'd1;
    localparam logic [1:0] CHAN_RIGHT = 2'd2;

    // =================================================================
    // Control registers
    // =================================================================
    logic             enable_q;
    logic [      1:0] chan_q;
    logic             format_lj_q;
    logic             ie_rx_q;
    logic             ie_ovr_q;
    logic [      1:0] wlen_sel_q;
    logic             master_q;
    logic [     15:0] clkdiv_q;
    logic [      7:0] slot_q;
    logic [LVL_W-1:0] water_q;
    logic             irq_ovr_q;

    // Word length in bits, from the 2-bit encoding (see the register map:
    // an encoding, not a free-form count, so the sign extension below is a
    // 4-way mux).
    logic [      5:0] wlen;
    always_comb begin
        unique case (wlen_sel_q)
            2'd0:    wlen = 6'd16;
            2'd1:    wlen = 6'd24;
            2'd2:    wlen = 6'd32;
            default: wlen = 6'd8;
        endcase
    end

    // A watermark of 0 would assert forever; treat it as 1.
    wire [LVL_W-1:0] water_eff = (water_q == '0) ? {{(LVL_W - 1) {1'b0}}, 1'b1} : water_q;

    // =================================================================
    // RX FIFO. One entry per received word: the sign-extended sample plus
    // the channel it came from, so a stereo stream can be de-interleaved
    // after the fact (and a dropped word cannot silently swap the
    // channels for everything that follows).
    // =================================================================
    logic [31:0] rx_data_q[RX_FIFO_DEPTH];
    logic rx_ch_q[RX_FIFO_DEPTH];

    logic [RX_PTR_W:0] rx_wptr_q, rx_rptr_q;

    wire rx_empty = (rx_wptr_q == rx_rptr_q);
    wire rx_full = (rx_wptr_q[RX_PTR_W] != rx_rptr_q[RX_PTR_W]) &&
        (rx_wptr_q[RX_PTR_W-1:0] == rx_rptr_q[RX_PTR_W-1:0]);

    wire [LVL_W-1:0] rx_level = rx_wptr_q - rx_rptr_q;

    wire [31:0] rx_head_data = rx_data_q[rx_rptr_q[RX_PTR_W-1:0]];
    wire rx_head_ch = rx_ch_q[rx_rptr_q[RX_PTR_W-1:0]];

    // =================================================================
    // Master-mode clock generator
    //
    // Half-period counter on BCLK, slot counter on the FALLING edge (so
    // LRCK changes there, which is where a transmitter is required to move
    // it). Parked low/low while MASTER=0 so enabling the mode always starts
    // a frame at the beginning of a left slot.
    //
    // SLOT of 0 reads as 32: a zero there would toggle LRCK on every BCLK
    // period and produce a frame no device can decode, so the register
    // cannot express it.
    // =================================================================
    wire [7:0] slot_eff = (slot_q == 8'd0) ? 8'd32 : slot_q;

    logic [15:0] gen_div_q;
    logic gen_bclk_q;
    logic [7:0] gen_slot_q;
    logic gen_ws_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i || !master_q) begin
            gen_div_q  <= clkdiv_q;
            gen_bclk_q <= 1'b0;
            gen_slot_q <= '0;
            gen_ws_q   <= 1'b0;
        end else if (gen_div_q == 16'd0) begin
            gen_div_q  <= clkdiv_q;
            gen_bclk_q <= ~gen_bclk_q;
            // gen_bclk_q still holds the OLD level here, so this is the
            // high-to-low (falling) transition.
            if (gen_bclk_q) begin
                if (gen_slot_q == slot_eff - 8'd1) begin
                    gen_slot_q <= '0;
                    gen_ws_q   <= ~gen_ws_q;
                end else begin
                    gen_slot_q <= gen_slot_q + 8'd1;
                end
            end
        end else begin
            gen_div_q <= gen_div_q - 16'd1;
        end
    end

    assign i2s_bclk_o   = gen_bclk_q;
    assign i2s_lrck_o   = gen_ws_q;
    assign i2s_clk_oe_o = master_q;

    // =================================================================
    // Receive engine (core clock, oversampling the audio bus)
    // =================================================================
    logic bclk_q;  // synchronized BCLK, previous core cycle
    logic ws_prev_q;  // WS as seen at the previous BCLK rising edge
    logic [31:0] shift_q;  // MSB-first accumulator
    logic [5:0] bit_cnt_q;  // bits captured into the current word
    logic collecting_q;  // a word is in progress
    logic cur_ch_q;  // channel of the word in progress

    wire bclk_rise = ~bclk_q & i2s_bclk_i;

    // A WS transition seen at a rising BCLK edge starts a new word. In I2S
    // format that edge carries no data (the MSB is one BCLK later); in
    // left-justified format it carries the MSB.
    wire ws_change = bclk_rise && (i2s_lrck_i != ws_prev_q);

    // The shift register as it would be after capturing this edge's bit.
    wire [31:0] shift_new = {shift_q[30:0], i2s_sd_i};

    // Sign extension from WLEN to 32 bits (4-way mux, see the register map).
    logic [31:0] sample_ext;
    always_comb begin
        unique case (wlen_sel_q)
            2'd0:    sample_ext = {{16{shift_new[15]}}, shift_new[15:0]};
            2'd1:    sample_ext = {{8{shift_new[23]}}, shift_new[23:0]};
            2'd2:    sample_ext = shift_new;
            default: sample_ext = {{24{shift_new[7]}}, shift_new[7:0]};
        endcase
    end

    // Channel filter. CHAN=3 is not a fourth mode -- it reads as stereo.
    wire ch_accept = (chan_q == CHAN_LEFT) ?
        (cur_ch_q == 1'b0) : (chan_q == CHAN_RIGHT) ? (cur_ch_q == 1'b1) : 1'b1;

    // The last bit of a word lands on a capture edge that is NOT a WS
    // transition (WLEN >= 8 > 1, so a word can never complete on the edge
    // that starts it, in either format).
    wire word_done = bclk_rise && enable_q && collecting_q && !ws_change &&
        (bit_cnt_q == wlen - 6'd1);

    // Split request / accept so the overrun flag has a single driver: the
    // engine only reports the event, the AXI block owns the sticky flop
    // (same split as axi4_lite_spi's rx_overrun).
    wire rx_push_req = word_done && ch_accept;
    wire rx_push = rx_push_req && !rx_full;
    wire rx_overrun_ev = rx_push_req && rx_full;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            bclk_q       <= 1'b0;
            ws_prev_q    <= 1'b0;
            shift_q      <= '0;
            bit_cnt_q    <= '0;
            collecting_q <= 1'b0;
            cur_ch_q     <= 1'b0;
            // Written by this block (pushed on word completion), so its
            // reset lives here too: a flop needs exactly one driving
            // always_ff or GowinSynthesis errors (EX2000).
            rx_wptr_q    <= '0;
        end else begin
            bclk_q <= i2s_bclk_i;

            if (!enable_q) begin
                // Idle: track WS so that enabling mid-frame does not fake a
                // transition. Collection then starts at the next real
                // channel boundary, costing at most one discarded word.
                ws_prev_q    <= i2s_lrck_i;
                collecting_q <= 1'b0;
                bit_cnt_q    <= '0;
            end else if (bclk_rise) begin
                ws_prev_q <= i2s_lrck_i;

                if (ws_change) begin
                    cur_ch_q     <= i2s_lrck_i;  // WS low = left, high = right
                    collecting_q <= 1'b1;
                    if (format_lj_q) begin
                        // Left-justified: the MSB is valid on this edge.
                        shift_q   <= shift_new;
                        bit_cnt_q <= 6'd1;
                    end else begin
                        // I2S: this edge announces the word, the MSB is on
                        // the next one.
                        shift_q   <= '0;
                        bit_cnt_q <= '0;
                    end
                end else if (collecting_q) begin
                    shift_q <= shift_new;
                    if (bit_cnt_q == wlen - 6'd1) begin
                        // Word complete. Padding bits up to the next WS
                        // transition are ignored.
                        collecting_q <= 1'b0;
                        bit_cnt_q    <= '0;
                    end else begin
                        bit_cnt_q <= bit_cnt_q + 6'd1;
                    end
                end
            end

            if (rx_push) begin
                rx_data_q[rx_wptr_q[RX_PTR_W-1:0]] <= sample_ext;
                rx_ch_q[rx_wptr_q[RX_PTR_W-1:0]]   <= cur_ch_q;
                rx_wptr_q                          <= rx_wptr_q + 1'b1;
            end
        end
    end

`ifdef VERILATOR
    // The oversampling premise: at least 2 core cycles per BCLK half
    // period (f(clk) >= 4 x f(BCLK)). Below that an edge can be missed
    // entirely and the word silently loses bits, which no functional check
    // in a testbench would attribute to the clock ratio. Counted on both
    // edges, saturating.
    logic [7:0] bclk_gap_q;
    wire bclk_edge = (bclk_q != i2s_bclk_i);

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            bclk_gap_q <= 8'hFF;
        end else if (bclk_edge) begin
            if (bclk_gap_q < 8'd2) begin
                $error("axi4_lite_i2s: BCLK half-period %0d core cycles, need >= 2", bclk_gap_q);
            end
            bclk_gap_q <= 8'd1;
        end else if (bclk_gap_q != 8'hFF) begin
            bclk_gap_q <= bclk_gap_q + 8'd1;
        end
    end
`endif

    // =================================================================
    // AXI write path (same accept pattern as axi4_lite_gpio / _uart).
    // =================================================================
    logic              aw_seen_q;
    logic              w_seen_q;
    logic [DATA_W-1:0] wdata_q;
    logic [STRB_W-1:0] wstrb_q;
    logic [       2:0] awaddr_word_q;
    logic              bvalid_q;

    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;

    wire               aw_hs = axi.awvalid && axi.awready;
    wire               w_hs = axi.wvalid && axi.wready;

    wire               aw_present = aw_seen_q || aw_hs;
    wire               w_present = w_seen_q || w_hs;

    wire  [       2:0] waddr_eff = aw_hs ? axi.awaddr[4:2] : awaddr_word_q;
    wire  [DATA_W-1:0] wdata_eff = w_hs ? axi.wdata : wdata_q;
    wire  [STRB_W-1:0] wstrb_eff = w_hs ? axi.wstrb : wstrb_q;

    // Byte-lane write mask (axi4_lite_gpio rule: unstrobed lanes carry
    // shifted store data, so they must never reach a register).
    logic [DATA_W-1:0] wr_mask;
    always_comb begin
        for (int b = 0; b < STRB_W; b++) begin
            wr_mask[8*b+:8] = {8{wstrb_eff[b]}};
        end
    end

    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;  // OKAY
    wire b_hs = axi.bvalid && axi.bready;

    wire do_write = aw_present && w_present && !bvalid_q;

    // W1C on the overrun flag. Set wins over clear (the axi4_lite_i2c /
    // _gpio rule): an overrun in the clear cycle is not lost.
    wire ovr_clr = do_write && (waddr_eff == REG_IRQ) && wdata_eff[1] && wr_mask[1];

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q     <= 1'b0;
            w_seen_q      <= 1'b0;
            wdata_q       <= '0;
            wstrb_q       <= '0;
            awaddr_word_q <= '0;
            bvalid_q      <= 1'b0;
            enable_q      <= 1'b0;
            chan_q        <= CHAN_STEREO;
            format_lj_q   <= 1'b0;
            ie_rx_q       <= 1'b0;
            ie_ovr_q      <= 1'b0;
            wlen_sel_q    <= 2'd0;  // 16 bits
            master_q      <= 1'b0;  // listen only
            clkdiv_q      <= 16'd24;  // 1 MHz BCLK from a 50 MHz core
            slot_q        <= 8'd32;  // 64-BCLK frame
            water_q       <= {{(LVL_W - 1) {1'b0}}, 1'b1};
            irq_ovr_q     <= 1'b0;
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

            // Sticky overrun: engine sets, W1C clears, set wins.
            if (rx_overrun_ev) begin
                irq_ovr_q <= 1'b1;
            end else if (ovr_clr) begin
                irq_ovr_q <= 1'b0;
            end

            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;

                unique case (waddr_eff)
                    REG_CTRL: begin
                        if (wr_mask[0]) begin
                            enable_q    <= wdata_eff[0];
                            chan_q      <= wdata_eff[2:1];
                            format_lj_q <= wdata_eff[3];
                            ie_rx_q     <= wdata_eff[4];
                            ie_ovr_q    <= wdata_eff[5];
                            master_q    <= wdata_eff[6];
                        end
                        if (wr_mask[8]) begin
                            wlen_sel_q <= wdata_eff[9:8];
                        end
                    end
                    REG_WATER: begin
                        if (wr_mask[0]) begin
                            water_q <= wdata_eff[LVL_W-1:0];
                        end
                    end
                    REG_CLKDIV: begin
                        if (wr_mask[0]) begin
                            clkdiv_q <= wdata_eff[15:0];
                        end
                        if (wr_mask[16]) begin
                            slot_q <= wdata_eff[23:16];
                        end
                    end
                    default: ;  // STATUS / RXDATA read-only, IRQ W1C above
                endcase
            end

            if (b_hs) begin
                bvalid_q <= 1'b0;
            end
        end
    end

    // =================================================================
    // AXI read path (1-cycle registered latency). RXDATA is the only
    // read with a side effect: it pops the FIFO.
    // =================================================================
    logic              rvalid_q;
    logic [DATA_W-1:0] rdata_q;

    assign axi.arready = !rvalid_q || (rvalid_q && axi.rready);
    wire ar_hs = axi.arvalid && axi.arready;

    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;  // OKAY

    wire rx_water_hit = (rx_level >= water_eff);

    logic [DATA_W-1:0] status_word;
    always_comb begin
        status_word           = '0;
        status_word[0]        = rx_empty;
        status_word[1]        = rx_full;
        status_word[2]        = irq_ovr_q;
        status_word[3]        = rx_empty ? 1'b0 : rx_head_ch;
        status_word[4]        = i2s_lrck_i;
        status_word[5]        = i2s_bclk_i;
        status_word[8+:LVL_W] = rx_level;
    end

    logic [DATA_W-1:0] ctrl_word;
    always_comb begin
        ctrl_word      = '0;
        ctrl_word[0]   = enable_q;
        ctrl_word[2:1] = chan_q;
        ctrl_word[3]   = format_lj_q;
        ctrl_word[4]   = ie_rx_q;
        ctrl_word[5]   = ie_ovr_q;
        ctrl_word[6]   = master_q;
        ctrl_word[9:8] = wlen_sel_q;
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q  <= 1'b0;
            rdata_q   <= '0;
            rx_rptr_q <= '0;
        end else begin
            if (ar_hs) begin
                rvalid_q <= 1'b1;
                unique case (axi.araddr[4:2])
                    REG_STATUS: rdata_q <= status_word;
                    REG_CTRL: rdata_q <= ctrl_word;
                    REG_WATER: rdata_q <= {{(DATA_W - LVL_W) {1'b0}}, water_q};
                    REG_RXDATA: begin
                        // Pop. An empty read returns 0 and moves nothing
                        // (same as the SPI RX FIFO: no underflow flag).
                        if (!rx_empty) begin
                            rdata_q   <= rx_head_data;
                            rx_rptr_q <= rx_rptr_q + 1'b1;
                        end else begin
                            rdata_q <= '0;
                        end
                    end
                    REG_IRQ: rdata_q <= {{(DATA_W - 2) {1'b0}}, irq_ovr_q, rx_water_hit};
                    REG_CLKDIV: rdata_q <= {{(DATA_W - 24) {1'b0}}, slot_q, clkdiv_q};
                    default: rdata_q <= '0;
                endcase
            end else if (rvalid_q && axi.rready) begin
                rvalid_q <= 1'b0;
            end
        end
    end

    // =================================================================
    // Interrupt (level-sensitive, see header comment).
    // =================================================================
    assign i2s_irq_o = (rx_water_hit & ie_rx_q) | (irq_ovr_q & ie_ovr_q);

endmodule

`resetall

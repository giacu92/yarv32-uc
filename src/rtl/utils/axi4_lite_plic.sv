`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* Minimal PLIC-style interrupt controller (AXI4-Lite slave).
*
* Aggregates the peripheral level IRQs into the machine external
* interrupt (meip_o -> csr_regfile.meip_i -> mip.MEIP), giving software a
* CAUSE ID where the bare OR it replaces gave none: an ISR reads CLAIM,
* dispatches on the returned ID, and only then services the source.
*
* Source IDs live in rv32_pkg (PLIC_SRC_*): 0 is reserved "no source",
* so the first real source is 1. Fixed hardware priority: LOWER ID WINS
* (matching the PLIC convention that source 1 is the highest-priority
* interrupt). No per-source priority registers and no threshold -- that
* is the deliberate scope cut; this is a cause/claim core, not a full
* PLIC. MSIP and MTIP do NOT pass through here: they are CLINT-style
* direct bits into csr_regfile, and the machine's own MEI>MSI>MTI
* arbitration is unchanged.
*
* Register map (word-addressed, byte offsets from PLIC_BASE):
*
*   0x00 PENDING (R) : bit i = source i pending. RO.
*   0x04 ENABLE  (RW): bit i = source i contributes to meip_o and is
*                      claimable. RESET ALL-ONES: with every source
*                      enabled, meip degenerates to the OR of the IRQ
*                      lines -- the exact behaviour of the bare OR this
*                      module replaced -- so firmware written before the
*                      PLIC (uart_echo, YarvMon) runs unchanged without
*                      touching it. Software that wants masking writes 0s.
*   0x08 CLAIM   (R) : returns the lowest-ID ENABLED PENDING source ID.
*                      Side-effect-FREE: the read clears nothing and holds
*                      no state -- completing the claim is the source's
*                      own service mechanism (see "PENDING IS THE LINES"
*                      below). There is deliberately no separate COMPLETE
*                      register. Returns 0 when nothing is claimable.
*
* PENDING IS THE LINES. Every source here is a level whose edge-ness is
* latched where it belongs -- the GPIO holds INT_STATUS sticky until its
* W1C, the UART/I2C/SPI hold their condition until serviced -- so the
* pending vector is exactly the irq_i lines, taken combinationally:
*
*   pend = irq_i
*
* No claim-side register to get wrong, and meip follows a line the same
* cycle it moves (the bare OR this module replaced had the same shape; a
* registered pend would only add a cycle of latency, and its sample could
* miss a 1-cycle pulse entirely -- no edge source can ever be wired here
* as long as this stays combinational). The CLAIM read is then a pure
* priority encode of the enabled pending lines; "claim = complete" is
* realized by the source's own service mechanism (the ISR W1Cs the GPIO,
* drains the UART), which drops the line, which drops the pending.
* CONSEQUENCE, correct but sharp: a CLAIM whose source is still high
* returns that same ID on every read, meip stays asserted, and the core
* takes the interrupt again after mret. An ISR that claims without
* servicing its source therefore livelocks -- the same contract the bare
* OR already imposed on uart_echo's handler ("drain the RX FIFO before
* returning"). plic_tb pins this behaviour.
*
* Protocol follows msip_peri.sv / axi4_lite_uart.sv: registered BVALID
* held until the B handshake, RVALID held until RREADY, single-outstanding.
*
* Reset is synchronous, active-low.
*
* Naming: ports *_i/_o; internals no prefix; flops _q, next-state _d.
*/

module axi4_lite_plic #(
    parameter int unsigned SRC_N = rv32_pkg::PLIC_SRC_N
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    // IRQ lines in, bit i = source ID i. Bit 0 is tied off by the caller
    // (source 0 is "no source" and can never claim).
    input wire [SRC_N-1:0] irq_i,

    // Machine external interrupt: any enabled pending source.
    output wire meip_o
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;
    localparam int ID_W = $clog2(SRC_N);

    // Word offsets (byte addr[4:2], word-aligned).
    localparam logic [2:0] REG_PENDING = 3'd0;
    localparam logic [2:0] REG_ENABLE = 3'd1;
    localparam logic [2:0] REG_CLAIM = 3'd2;

    // =================================================================
    // Source state: pending (the lines themselves -- see header, no
    // register) and enable (reset all-ones -- see header for why).
    // =================================================================
    wire  [SRC_N-1:0] pend;
    logic [SRC_N-1:0] enable_q;

    // Source 0 is reserved "no source" and is masked out of meip: a
    // caller that forgets the bit-0 tie-off would otherwise create an
    // unclaimable MEI livelock (meip high while CLAIM returns 0 -- the
    // encoder below starts at ID 1 and can never service bit 0).
    localparam logic [SRC_N-1:0] SRC_MASK = {{(SRC_N - 1) {1'b1}}, 1'b0};

    assign pend   = irq_i;
    assign meip_o = |(pend & enable_q & SRC_MASK);

    // =================================================================
    // AXI write path (same accept pattern as axi4_lite_uart). Only
    // ENABLE is writable; PENDING and CLAIM are read-only.
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

    // Byte-lane write mask, replicated from the strobe: a sub-word store
    // MERGES into the register -- strobed lanes take the new data,
    // unstrobed lanes keep their value. The LSU shifts store data into
    // the strobed lanes and leaves unstrobed lanes carrying shifted rs2
    // bits (not zeros), so gating on "byte 0 strobed" would both drop a
    // store to ENABLE[15:8] (answered OKAY) and ghost-write the upper
    // half from unstrobed data lanes.
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

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q     <= 1'b0;
            w_seen_q      <= 1'b0;
            wdata_q       <= '0;
            wstrb_q       <= '0;
            bvalid_q      <= 1'b0;
            enable_q      <= '1;  // reset: every source enabled (see header)
            awaddr_word_q <= '0;
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

                unique case (waddr_eff)
                    // Strobed merge (see wr_mask): a byte store to the
                    // ENABLE[15:8] half now lands, and an unstrobed lane
                    // can never reach the register.
                    REG_ENABLE:
                    enable_q <= (enable_q & ~wr_mask[SRC_N-1:0]) |
                        (wdata_eff[SRC_N-1:0] & wr_mask[SRC_N-1:0]);
                    default: ;  // PENDING/CLAIM read-only
                endcase
            end

            if (b_hs) begin
                bvalid_q <= 1'b0;
            end
        end
    end

    // =================================================================
    // AXI read path (1-cycle registered latency). CLAIM is a pure
    // priority read of the enabled pending lines -- side-effect-free
    // (see header: completing a claim is the source's own job).
    // =================================================================
    logic              rvalid_q;
    logic [DATA_W-1:0] rdata_q;

    assign axi.arready = !rvalid_q || (rvalid_q && axi.rready);
    wire ar_hs = axi.arvalid && axi.arready;

    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;  // OKAY

    // Lowest-ID enabled-pending priority encode. Index 0 never claims
    // (source 0 = "no source"), which also makes 0 the natural "nothing
    // claimable" encoding -- the scan initialises sel_id to 0 and only
    // ever assigns it under the same condition that would set a valid
    // bit, so a separate valid flag would be redundant.
    logic [ID_W-1:0] sel_id;

    always_comb begin
        // Descending scan, last assignment wins -> lowest ID selected.
        sel_id = '0;
        for (int i = SRC_N - 1; i >= 1; i = i - 1) begin
            if (pend[i] && enable_q[i]) begin
                sel_id = ID_W'(i);
            end
        end
    end

    logic [DATA_W-1:0] rdata_mux;
    always_comb begin
        unique case (axi.araddr[4:2])
            // Raw lines, bit 0 included: it records like any other bit
            // (plic_tb pins this) but is masked out of meip and of CLAIM.
            REG_PENDING: rdata_mux = {{(DATA_W - SRC_N) {1'b0}}, pend};
            REG_ENABLE:  rdata_mux = {{(DATA_W - SRC_N) {1'b0}}, enable_q};
            REG_CLAIM:   rdata_mux = {{(DATA_W - ID_W) {1'b0}}, sel_id};
            default:     rdata_mux = '0;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q <= 1'b0;
            rdata_q  <= '0;
        end else begin
            if (ar_hs) begin
                rvalid_q <= 1'b1;
                rdata_q  <= rdata_mux;
            end else if (rvalid_q && axi.rready) begin
                rvalid_q <= 1'b0;
            end
        end
    end

endmodule

`resetall

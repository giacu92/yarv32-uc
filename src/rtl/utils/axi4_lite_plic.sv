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
*   0x08 CLAIM   (R) : returns the lowest-ID ENABLED PENDING source ID
*                      and completes the claim (clears that source's
*                      pending bit) in the same read -- claim = complete,
*                      one transaction, no separate COMPLETE register.
*                      Returns 0 when nothing is claimable.
*
* PENDING IS THE LINES. Every source here is a level whose edge-ness is
* latched where it belongs -- the GPIO holds INT_STATUS sticky until its
* W1C, the UART/I2C/SPI hold their condition until serviced -- so
*
*   pend_q <= irq_i
*
* is the whole model: a source's pending is exactly its line, and there
* is no claim-side state to get wrong. The CLAIM read is then a pure
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
    // Source state: pending (level model, see header) and enable
    // (reset all-ones -- see header for why).
    // =================================================================
    logic [SRC_N-1:0] pend_q;
    logic [SRC_N-1:0] enable_q;

    assign meip_o = |(pend_q & enable_q);

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

    wire              aw_hs = axi.awvalid && axi.awready;
    wire              w_hs = axi.wvalid && axi.wready;

    wire              aw_present = aw_seen_q || aw_hs;
    wire              w_present = w_seen_q || w_hs;

    wire [       2:0] waddr_eff = aw_hs ? axi.awaddr[4:2] : awaddr_word_q;
    wire [DATA_W-1:0] wdata_eff = w_hs ? axi.wdata : wdata_q;
    wire [STRB_W-1:0] wstrb_eff = w_hs ? axi.wstrb : wstrb_q;

    // Byte-0 strobe required (same rationale as axi4_lite_uart).
    wire              wr_low_byte = wstrb_eff[0];

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
            pend_q        <= '0;
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

            // Pending is the lines (see header).
            pend_q <= irq_i;

            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;

                unique case (waddr_eff)
                    REG_ENABLE:
                    if (wr_low_byte) begin
                        enable_q <= wdata_eff[SRC_N-1:0];
                    end
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
    // (source 0 = "no source").
    logic [ID_W-1:0] sel_id;
    logic            sel_valid;

    always_comb begin
        // Declared at block top: Gowin EX3990 rejects in-block automatic
        // declarations, and Verilator will not catch that (see CLAUDE.md).
        // Descending scan, last assignment wins -> lowest ID selected.
        sel_id    = '0;
        sel_valid = 1'b0;
        for (int i = SRC_N - 1; i >= 1; i = i - 1) begin
            if (pend_q[i] && enable_q[i]) begin
                sel_id    = ID_W'(i);
                sel_valid = 1'b1;
            end
        end
    end

    logic [DATA_W-1:0] rdata_mux;
    always_comb begin
        unique case (axi.araddr[4:2])
            REG_PENDING: rdata_mux = {{(DATA_W - SRC_N) {1'b0}}, pend_q};
            REG_ENABLE:  rdata_mux = {{(DATA_W - SRC_N) {1'b0}}, enable_q};
            // Plain ID: 0 means "nothing claimable" (source 0 is the
            // reserved no-source ID and never claims), so a separate valid
            // bit would be redundant.
            REG_CLAIM:   rdata_mux = {{(DATA_W - ID_W) {1'b0}}, sel_valid ? sel_id : '0};
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

`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* PLIC compatible with `sifive,plic-1.0.0` / `riscv,plic0` (AXI4-Lite slave),
* for the Linux branch (M8). Two contexts: 0 = hart 0 M-mode (meip_o),
* 1 = hart 0 S-mode (seip_o, unused until S-mode exists -- M11).
*
* Layout, as mainline drivers/irqchip/irq-sifive-plic.c and OpenSBI's
* lib/utils/irqchip/plic.c address it (window 4 MiB, decoded on addr[21:0]):
*
*   +0x000000 + 4*id            priority[id], 3 bits, RW (id 0 reads 0)
*   +0x001000                   pending[31:0], RO (bit id)
*   +0x002000 + 0x80*ctx        enable[31:0] for context ctx, RW
*   +0x200000 + 0x1000*ctx      threshold, 3 bits, RW
*   +0x200004 + 0x1000*ctx      claim (read) / complete (write)
*
* Gateway (level-triggered sources, which every source here is): a source's
* pending bit is set while its line is high and it is not in service; a
* CLAIM read returns the highest-priority eligible source of that context
* (eligible = pending & enabled & priority > threshold; ties go to the lower
* ID; 0 = none), clears its pending bit and marks it in service, so it is not
* presented again until its COMPLETE write -- even if the line stays high.
* A complete is honoured only if the ID is enabled for that context (spec;
* Linux's plic_irq_eoi() temporarily re-enables a masked source for exactly
* this reason). After the complete, a line still high re-pends next cycle.
*
* Linux masks / unmasks a source by writing priority 0 / 1 (priority 0 is
* "never interrupts" because the eligibility test is priority > threshold
* and thresholds are >= 0), and sets threshold 0 for an active context.
*
* Differences from the MCU's axi4_lite_plic (0x1000_8000, three registers):
* that one had no priority, no threshold, a side-effect-free CLAIM and no
* COMPLETE -- correct for the MCU's firmware, unknown to every driver.
*
* Reset: priorities 0, enables 0, thresholds 0, nothing pending -- the spec's
* reset and what Linux expects. (The MCU PLIC reset ENABLE to all-ones so
* pre-PLIC firmware kept working; firmware on this branch must enable.)
*
* Reset is synchronous, active-low. Naming: ports *_i/_o; flops _q.
*/

module axi4_lite_plic_sifive #(
    parameter int unsigned SRC_N = PLIC_SRC_N  // sources incl. reserved 0, <= 32
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    input  wire [SRC_N-1:0] irq_i,   // level lines; bit 0 ignored (ID 0 = none)
    output wire             meip_o,  // context 0
    output wire             seip_o   // context 1
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int ID_W = $clog2(SRC_N);
    localparam int NCTX = 2;

    logic [SRC_N-1:0][2:0] prio_q;
    logic [SRC_N-1:0] pend_q;
    logic [SRC_N-1:0] insvc_q;
    logic [NCTX-1:0][SRC_N-1:0] en_q;
    logic [NCTX-1:0][2:0] thr_q;

    // ---------------------------------------------------------------
    // Arbitration per context: highest priority, lowest ID on a tie.
    // ---------------------------------------------------------------
    logic [NCTX-1:0][ID_W-1:0] best_id;
    logic [NCTX-1:0] any;
    always_comb begin
        for (int c = 0; c < NCTX; c++) begin
            logic [2:0] bp;
            best_id[c] = '0;
            bp         = '0;
            any[c]     = 1'b0;
            for (int i = SRC_N - 1; i >= 1; i--) begin
                if (pend_q[i] && en_q[c][i] && (prio_q[i] > thr_q[c]) && (prio_q[i] >= bp)) begin
                    best_id[c] = ID_W'(i);
                    bp         = prio_q[i];
                    any[c]     = 1'b1;
                end
            end
        end
    end
    assign meip_o = any[0];
    assign seip_o = any[1];

    // ---------------------------------------------------------------
    // AXI plumbing (single outstanding, BVALID / RVALID held)
    // ---------------------------------------------------------------
    logic aw_seen_q, w_seen_q, bvalid_q;
    logic [DATA_W-1:0] wdata_q;
    logic [       3:0] wstrb_q;
    logic [      21:0] awaddr_q;
    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;
    wire aw_hs = axi.awvalid && axi.awready;
    wire w_hs = axi.wvalid && axi.wready;
    wire do_write = (aw_seen_q || aw_hs) && (w_seen_q || w_hs) && !bvalid_q;
    wire [21:0] waddr = aw_seen_q ? awaddr_q : {axi.awaddr[21:2], 2'b00};
    wire [DATA_W-1:0] wdata = w_seen_q ? wdata_q : axi.wdata;
    wire [3:0] wstrb = w_seen_q ? wstrb_q : axi.wstrb;
    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;

    logic              rvalid_q;
    logic [DATA_W-1:0] rdata_q;
    assign axi.arready = !rvalid_q || axi.rready;
    wire ar_hs = axi.arvalid && axi.arready;
    wire [21:0] raddr = {axi.araddr[21:2], 2'b00};
    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;

    // Address classifiers. A register exists only if the offset matches
    // exactly; everything else reads 0 / ignores writes.
    function automatic logic is_prio(input logic [21:0] a);
        return (a[21:12] == '0) && (a[11:2] < SRC_N) && (a[11:2] != 0);
    endfunction
    function automatic logic is_pend(input logic [21:0] a);
        return a == 22'h001000;
    endfunction
    // enable word 0 of context c: 0x2000 + 0x80*c
    function automatic int en_ctx(input logic [21:0] a);
        for (int c = 0; c < NCTX; c++) if (a == 22'h002000 + 22'(c * 'h80)) return c;
        return -1;
    endfunction
    function automatic int thr_ctx(input logic [21:0] a);
        for (int c = 0; c < NCTX; c++) if (a == 22'h200000 + 22'(c * 'h1000)) return c;
        return -1;
    endfunction
    function automatic int clm_ctx(input logic [21:0] a);
        for (int c = 0; c < NCTX; c++) if (a == 22'h200004 + 22'(c * 'h1000)) return c;
        return -1;
    endfunction

    // Read mux and claim side effect.
    logic [DATA_W-1:0] rmux;
    logic claim_rd;
    logic [ID_W-1:0] claim_id;
    always_comb begin
        rmux     = '0;
        claim_rd = 1'b0;
        claim_id = '0;
        if (is_prio(raddr)) rmux = {29'b0, prio_q[raddr[ID_W+1:2]]};
        else if (is_pend(raddr)) rmux = 32'(pend_q);
        else if (en_ctx(raddr) >= 0) rmux = 32'(en_q[en_ctx(raddr)]);
        else if (thr_ctx(raddr) >= 0) rmux = {29'b0, thr_q[thr_ctx(raddr)]};
        else if (clm_ctx(raddr) >= 0) begin
            rmux     = 32'(best_id[clm_ctx(raddr)]);
            claim_rd = ar_hs && any[clm_ctx(raddr)];
            claim_id = best_id[clm_ctx(raddr)];
        end
    end

    // Complete (write to a claim/complete register).
    logic cmpl_wr;
    logic [ID_W-1:0] cmpl_id;
    always_comb begin
        cmpl_wr = 1'b0;
        cmpl_id = wdata[ID_W-1:0];
        if (do_write && (clm_ctx(waddr) >= 0) && (wdata < SRC_N) && (wdata != 0))
            cmpl_wr = en_q[clm_ctx(waddr)][wdata[ID_W-1:0]];
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q <= 1'b0;
            w_seen_q  <= 1'b0;
            bvalid_q  <= 1'b0;
            wdata_q   <= '0;
            wstrb_q   <= '0;
            awaddr_q  <= '0;
            rvalid_q  <= 1'b0;
            rdata_q   <= '0;
            prio_q    <= '0;
            en_q      <= '0;
            thr_q     <= '0;
            pend_q    <= '0;
            insvc_q   <= '0;
        end else begin
            if (aw_hs) begin
                aw_seen_q <= 1'b1;
                awaddr_q  <= {axi.awaddr[21:2], 2'b00};
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
                if (is_prio(waddr) && wstrb[0]) prio_q[waddr[ID_W+1:2]] <= wdata[2:0];
                else if (en_ctx(waddr) >= 0) begin
                    for (int b = 0; b < 4; b++)
                    if (wstrb[b])
                        for (int k = 8 * b; k < 8 * b + 8; k++)
                        if (k < SRC_N) en_q[en_ctx(waddr)][k] <= wdata[k];
                end else if ((thr_ctx(waddr) >= 0) && wstrb[0]) thr_q[thr_ctx(waddr)] <= wdata[2:0];
            end
            if (axi.bvalid && axi.bready) bvalid_q <= 1'b0;

            if (ar_hs) begin
                rvalid_q <= 1'b1;
                rdata_q  <= rmux;
            end else if (rvalid_q && axi.rready) rvalid_q <= 1'b0;

            // Gateway. Order: complete, then claim, then the lines -- so a
            // line still high after its complete re-pends next cycle, and a
            // claim of a source does not race its own re-pend.
            for (int i = 1; i < SRC_N; i++) begin
                if (cmpl_wr && (cmpl_id == ID_W'(i))) insvc_q[i] <= 1'b0;
                if (claim_rd && (claim_id == ID_W'(i))) begin
                    pend_q[i]  <= 1'b0;
                    insvc_q[i] <= 1'b1;
                end else if (irq_i[i] && !insvc_q[i]) pend_q[i] <= 1'b1;
            end
        end
    end

endmodule

`resetall

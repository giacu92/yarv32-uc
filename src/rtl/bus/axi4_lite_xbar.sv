`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * AXI4-Lite 1->N parametric crossbar: one slave port (from the CPU's
 * single master bridge) routed to one of N targets by base+size address
 * windows. Replaces the per-count copies (the old 1->2/1->3/1->4 files):
 * adding a peripheral is now a parameter change, not a new file.
 *
 *   window i matches when  BASES[i] <= addr < BASES[i] + SIZES[i]
 *   lowest-numbered matching window wins; no match -> local DECERR
 *
 * Why the master side is FLAT VECTORS, not N axi4_lite_if ports:
 * SystemVerilog cannot express a parametric port COUNT as individual
 * interface ports, and the two workarounds are both closed here --
 * Gowin's elaborator does not accept interface-array ports or dynamic
 * interface-array indexing reliably (measured on this project), and a
 * top cannot connect individual elements of an interface array to the
 * separate axi4_lite_if instances the peripherals use anyway. So the
 * xbar speaks plain vectors at the target boundary:
 *
 *   - payload (awaddr/wdata/wstrb/araddr) is BROADCAST to every target
 *     (one driver, like the per-count xbars that drove the same nets
 *     into all master ports); only the valid/ready handshakes are
 *     per-target bit i.
 *   - the board/sim top glues each target's axi4_lite_if instance to
 *     bit i with plain assigns (see top_module / sim_top).
 *
 * BASES/SIZES are packed parameter vectors, not unpacked parameter
 * arrays (same Gowin-elaborator caveat as above). Index i lives at
 * bits [32*i +: 32], so the concatenation reads {window N-1, ..., 1, 0}
 * -- highest index leftmost. N, BASES and SIZES MUST be overridden
 * together: changing N alone re-widths the vectors and silently
 * zero-extends the defaults (a zero size window never matches).
 *
 * Single-outstanding pass-through, exactly like the per-count xbars it
 * replaces (the CPU bridge is single-outstanding overall): no buffering,
 * no arbitration. The target is latched per direction on the AW / AR
 * handshake as a one-hot (wr_match_q / rd_match_q) plus a no-match flag
 * (wr_err_q / rd_err_q); the follow-on W/B and R beats route to that
 * latched one-hot until the transaction retires.
 *
 * Write channel ordering: AXI4-Lite lets AW and W arrive in either
 * order. The bridge asserts both together on a write launch, but either
 * may handshake first. While a write is in flight, W and B follow the
 * latched target; on the launch cycle (!wr_busy_q) W routes by the live
 * awaddr decode, which matches the AW target the same cycle. W valid
 * with AW not valid and not busy does not occur from this bridge.
 *
 * An address that matches NO window is not left to stall: it is
 * completed locally by the decode-error terminator inherited verbatim
 * from the 1->3/1->4 xbars. The write is accepted and answered with
 * BRESP = DECERR, the read answered with RRESP = DECERR / RDATA = 0.
 * Without it the slave-side ready stays low forever and the CPU's LSU
 * parks in EX_MEM_WAIT with no way out but reset -- which is easy to
 * hit, because the LSU routes the WHOLE 0x1000_0000..0x1FFF_FFFF
 * region here on addr[PERI_ADDR_BIT] while only the mapped windows
 * exist.
 *
 * No combinational loop: the slave's ready outputs depend on the
 * targets' ready outputs (register-based downstream) and on this xbar's
 * registered busy flags, never on the slave's own valid outputs.
 */
module axi4_lite_xbar #(
    // Number of targets. Override together with BASES/SIZES.
    parameter int unsigned N = 4,

    // Window bases, bits [32*i +: 32] = base of target i.
    // Default = the current board peri map (m0..m3).
    parameter logic [N*32-1:0] BASES = {SDIO_BASE, MSIP_PERI_ADDR, MTIMER_BASE, UART_BASE},

    // Window sizes, bits [32*i +: 32] = size of target i.
    parameter logic [N*32-1:0] SIZES = {SDIO_SIZE, MSIP_PERI_SIZE, MTIMER_SIZE, UART_SIZE}
) (
    input wire clk_i,
    input wire rstn_i,

    // Slave side: the CPU's single AXI4-Lite master bridge.
    axi4_lite_if.slave s_axi,

    // -----------------------------------------------------------------
    // Target side, flat. Bit i (slice i) of every signal belongs to
    // target i. Payload is broadcast; handshakes are per-target.
    // -----------------------------------------------------------------
    // Broadcast payload (same value toward every target).
    output wire [31:0] m_awaddr_o,
    output wire [31:0] m_wdata_o,
    output wire [ 3:0] m_wstrb_o,
    output wire [31:0] m_araddr_o,

    // xbar -> target handshake bits.
    output wire [N-1:0] m_awvalid_o,
    output wire [N-1:0] m_wvalid_o,
    output wire [N-1:0] m_bready_o,
    output wire [N-1:0] m_arvalid_o,
    output wire [N-1:0] m_rready_o,

    // target -> xbar handshake bits and response payload.
    input wire [   N-1:0] m_awready_i,
    input wire [   N-1:0] m_wready_i,
    input wire [   N-1:0] m_bvalid_i,
    input wire [   N-1:0] m_arready_i,
    input wire [   N-1:0] m_rvalid_i,
    input wire [ 2*N-1:0] m_bresp_i,    // bits [2*i +: 2] = target i BRESP
    input wire [ 2*N-1:0] m_rresp_i,    // bits [2*i +: 2] = target i RRESP
    input wire [32*N-1:0] m_rdata_i     // bits [32*i +: 32] = target i RDATA
);

    // -----------------------------------------------------------------
    // Address decode: one comparator per window, generate-unrolled so
    // every BASES/SIZES slice is an elaboration-time constant part
    // select (genvar index), not a runtime index into a parameter.
    // wr_match/rd_match are one-hot, '0 = no match.
    // -----------------------------------------------------------------
    logic [N-1:0] wr_match;
    logic [N-1:0] rd_match;

    genvar gi;
    generate
        for (gi = 0; gi < N; gi = gi + 1) begin : g_win
            localparam logic [31:0] win_base = BASES[gi*32+:32];
            localparam logic [31:0] win_size = SIZES[gi*32+:32];

            assign
                wr_match[gi] = (s_axi.awaddr >= win_base) && (s_axi.awaddr < win_base + win_size);
            assign
                rd_match[gi] = (s_axi.araddr >= win_base) && (s_axi.araddr < win_base + win_size);
        end
    endgenerate

    wire          wr_live_err = (wr_match == '0);  // no window matches awaddr
    wire          rd_live_err = (rd_match == '0);  // no window matches araddr

    // -----------------------------------------------------------------
    // Per-direction latched target: one-hot + no-match flag, set on the
    // AW / AR handshake, held until B / R retires the transaction.
    // -----------------------------------------------------------------
    logic         wr_busy_q;  // a write is in flight (AW handshaked, B not yet)
    logic [N-1:0] wr_match_q;  // latched AW target one-hot (while wr_busy_q)
    logic         wr_err_q;  // latched AW target = no match (DECERR)

    logic         rd_busy_q;  // a read is in flight (AR handshaked, R not yet)
    logic [N-1:0] rd_match_q;  // latched AR target one-hot (while rd_busy_q)
    logic         rd_err_q;  // latched AR target = no match (DECERR)

    // Effective W target: latched while a write is in flight, live decode
    // on the launch cycle (W only accompanies AW there).
    wire  [N-1:0] wr_eff_match = wr_busy_q ? wr_match_q : wr_match;
    wire          wr_eff_err = wr_busy_q ? wr_err_q : wr_live_err;

    // -----------------------------------------------------------------
    // Decode-error terminator state (no-match). Mirrors the minimal slave
    // handshake the real peripherals use: registered BVALID held until
    // the B handshake, RVALID held until RREADY, single-outstanding.
    // -----------------------------------------------------------------
    logic         err_aw_seen_q;
    logic         err_w_seen_q;
    logic         err_bvalid_q;
    logic         err_rvalid_q;

    wire          err_awready = !err_aw_seen_q && !err_bvalid_q;
    wire          err_wready = !err_w_seen_q && !err_bvalid_q;
    wire          err_arready = !err_rvalid_q;

    localparam logic [1:0] RESP_DECERR = 2'b11;

    // -----------------------------------------------------------------
    // Handshake predicates
    // -----------------------------------------------------------------
    wire aw_go = !wr_busy_q && s_axi.awvalid;  // AW may forward this cycle
    wire ar_go = !rd_busy_q && s_axi.arvalid;  // AR may forward this cycle

    wire aw_hs = s_axi.awvalid && s_axi.awready;
    wire b_hs = s_axi.bvalid && s_axi.bready;
    wire ar_hs = s_axi.arvalid && s_axi.arready;
    wire r_hs = s_axi.rvalid && s_axi.rready;

    // Terminator handshakes (only when the live/latched target is the
    // no-match code).
    wire err_aw_hs = aw_hs && wr_live_err;  // aw_hs implies !wr_busy_q
    wire err_w_hs = s_axi.wvalid && s_axi.wready && wr_eff_err;
    wire err_b_hs = b_hs && wr_busy_q && wr_err_q;
    wire err_ar_hs = ar_hs && rd_live_err;  // ar_hs implies !rd_busy_q
    wire err_r_hs = r_hs && rd_busy_q && rd_err_q;

    wire err_do_write = (err_aw_seen_q || err_aw_hs) && (err_w_seen_q || err_w_hs) && !err_bvalid_q;

    // =================================================================
    // Slave -> target routing. Payload always broadcast (the unselected
    // targets see valid low and ignore it); per-target valid/ready are
    // one-hot gated exactly like the per-count case statements were.
    // =================================================================
    assign m_awaddr_o  = s_axi.awaddr;
    assign m_wdata_o   = s_axi.wdata;
    assign m_wstrb_o   = s_axi.wstrb;
    assign m_araddr_o  = s_axi.araddr;

    assign m_awvalid_o = aw_go ? wr_match : '0;
    assign m_arvalid_o = ar_go ? rd_match : '0;
    assign m_wvalid_o  = s_axi.wvalid ? wr_eff_match : '0;
    assign m_bready_o  = (wr_busy_q && !wr_err_q) ? (wr_match_q & {N{s_axi.bready}}) : '0;
    assign m_rready_o  = (rd_busy_q && !rd_err_q) ? (rd_match_q & {N{s_axi.rready}}) : '0;

    // =================================================================
    // Target -> slave response routing
    // =================================================================
    always_comb begin
        // AW / AR: route the selected target's ready back (or the
        // terminator's on no-match).
        s_axi.awready = aw_go ? (wr_live_err ? err_awready : |(wr_match & m_awready_i)) : 1'b0;
        s_axi.wready = s_axi.wvalid ? (wr_eff_err ? err_wready : |(wr_eff_match & m_wready_i)) :
            1'b0;
        s_axi.arready = ar_go ? (rd_live_err ? err_arready : |(rd_match & m_arready_i)) : 1'b0;

        // Defaults: nothing in flight -> no response, DECERR payload.
        s_axi.bvalid = 1'b0;
        s_axi.bresp = RESP_DECERR;
        s_axi.rvalid = 1'b0;
        s_axi.rdata = '0;
        s_axi.rresp = RESP_DECERR;

        // B: the in-flight write's target (or the terminator).
        if (wr_busy_q && !wr_err_q) begin
            s_axi.bvalid = |(wr_match_q & m_bvalid_i);
            s_axi.bresp  = 2'b00;
            for (int i = 0; i < N; i++) begin
                if (wr_match_q[i]) s_axi.bresp = m_bresp_i[2*i+:2];
            end
        end else if (wr_busy_q) begin
            s_axi.bvalid = err_bvalid_q;  // no match: DECERR (default bresp)
        end

        // R: the in-flight read's target (or the terminator, rdata = 0).
        if (rd_busy_q && !rd_err_q) begin
            s_axi.rvalid = |(rd_match_q & m_rvalid_i);
            s_axi.rresp  = 2'b00;
            s_axi.rdata  = '0;
            for (int i = 0; i < N; i++) begin
                if (rd_match_q[i]) begin
                    s_axi.rresp = m_rresp_i[2*i+:2];
                    s_axi.rdata = m_rdata_i[32*i+:32];
                end
            end
        end else if (rd_busy_q) begin
            s_axi.rvalid = err_rvalid_q;  // no match: DECERR, rdata = 0
        end
    end

    // =================================================================
    // State: latch target on launch, clear on retire; terminator FSM
    // inherited from the 1->3/1->4 xbars.
    // =================================================================
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            wr_busy_q     <= 1'b0;
            wr_match_q    <= '0;
            wr_err_q      <= 1'b0;
            rd_busy_q     <= 1'b0;
            rd_match_q    <= '0;
            rd_err_q      <= 1'b0;
            err_aw_seen_q <= 1'b0;
            err_w_seen_q  <= 1'b0;
            err_bvalid_q  <= 1'b0;
            err_rvalid_q  <= 1'b0;
        end else begin
            // ---- Decode-error terminator ----
            if (err_aw_hs) err_aw_seen_q <= 1'b1;
            if (err_w_hs) err_w_seen_q <= 1'b1;
            if (err_do_write) begin
                err_aw_seen_q <= 1'b0;
                err_w_seen_q  <= 1'b0;
                err_bvalid_q  <= 1'b1;
            end
            if (err_b_hs) err_bvalid_q <= 1'b0;

            if (err_ar_hs) err_rvalid_q <= 1'b1;
            else if (err_r_hs) err_rvalid_q <= 1'b0;

            // ---- Write target latched on AW handshake; cleared on B ----
            // (aw_hs and b_hs are mutually exclusive: b_hs needs
            // wr_busy_q, aw_hs needs !wr_busy_q.)
            if (aw_hs) begin
                wr_busy_q  <= 1'b1;
                wr_match_q <= wr_match;
                wr_err_q   <= wr_live_err;
            end else if (b_hs) begin
                wr_busy_q <= 1'b0;
            end

            // ---- Read target latched on AR handshake; cleared on R ----
            if (ar_hs) begin
                rd_busy_q  <= 1'b1;
                rd_match_q <= rd_match;
                rd_err_q   <= rd_live_err;
            end else if (r_hs) begin
                rd_busy_q <= 1'b0;
            end
        end
    end

endmodule

`resetall

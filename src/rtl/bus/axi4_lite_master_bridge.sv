`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Generic AXI4-Lite master bridge.
 *
 * Sits between the CPU's native cpu_mem_req_t / cpu_mem_rsp_t interface and a
 * master AXI4-Lite port. The native interface is split per direction
 * (stile AXI): req_i carries all master->bridge signals (valid/we/
 * addr/wdata/wstrb/rready), rsp_o carries all bridge->master signals
 * (wready/rvalid/rdata).
 *
 * Behaviour:
 *   - Request launch handshake: req_i.valid && rsp_o.wready, where
 *     rsp_o.wready is high only in S_IDLE. On acceptance the request
 *     (addr/wdata/wstrb) is latched into addr_q/wdata_q/wstrb_q so the
 *     bridge no longer depends on req_i staying stable once the native
 *     master sees wready and may drop its request the next cycle. All
 *     wait/retry states drive AXI from the latched registers, never
 *     from req_i directly. A single shared FSM tracks the pending read
 *     or write beat.
 *   - For reads: on launch, AR is asserted. If not accepted the same
 *     cycle, the bridge moves to S_RD_ADDR and keeps driving AR from
 *     addr_q until ar_hs. In S_RD_WAIT the master's rready is forwarded
 *     to axi.rready, so the AXI slave holds rvalid/rdata until the
 *     master is ready. rsp_o.rvalid is high the cycle the read data is
 *     consumed (rvalid && rready); rsp_o.rdata carries it. The bridge
 *     then returns to S_IDLE.
 *   - For writes: on launch, AW + W are launched in lock-step. Any
 *     combination of accept/no-accept on the launch cycle is handled:
 *       - both accepted            -> S_WR_WAIT
 *       - only AW accepted         -> S_WR_ADDR (retry W from wdata_q/
 *                                     wstrb_q until w_hs)
 *       - only W accepted          -> S_WR_DATA (retry AW from addr_q
 *                                     until aw_hs)
 *       - neither accepted         -> S_WR_BOTH (retry both from the
 *                                     latch until at least one hs, then
 *                                     fall into S_WR_ADDR/S_WR_DATA/
 *                                     S_WR_WAIT as above)
 *     A channel is only ever re-issued if it hasn't already handshaked
 *     (no double AW/W). On B (b_hs in S_WR_WAIT) the bridge raises
 *     rsp_o.bvalid so the LSU can retire the store, then returns to
 *     S_IDLE.
 *   - Read and write share one FSM, so the bridge is single-outstanding
 *     overall: at most one transaction (read OR write) in flight at a
 *     time.
 *
 * The CPU does not need to know whether it is talking to memory or
 * peripherals — the board top picks the topology and instantiates one
 * of these per master port.
 *
 * Naming: ports use *_i/_o; internal signals have no prefix. The FSM
 * state register is state_q, its next-state is state_d. AXI interface
 * member names follow the AXI spec (awvalid, arready, ...).
 */

module axi4_lite_master_bridge (
    input wire clk_i,
    input wire rstn_i,

    // Native CPU interface
    input  cpu_mem_req_t req_i,
    output cpu_mem_rsp_t rsp_o,

    // Master AXI4-Lite port
    axi4_lite_if.master axi
);

    // -----------------------------------------------------------------
    // Per-channel state
    // -----------------------------------------------------------------
    // 7 states -> need 3 bits.
    typedef enum logic [2:0] {
        S_IDLE,
        S_RD_ADDR,
        S_RD_WAIT,
        S_WR_BOTH,
        S_WR_ADDR,
        S_WR_DATA,
        S_WR_WAIT
    } state_t;

    state_t state_q, state_d;

    // -----------------------------------------------------------------
    // Request latch: captured on acceptance (state_q==S_IDLE &&
    // req_i.valid), so every downstream state drives AXI from these
    // registers instead of from req_i, which the native master is free
    // to change/drop the cycle after wready is seen high. we/direction
    // doesn't need latching: state_q alone distinguishes the read side
    // (S_RD_*) from the write side (S_WR_*).
    // -----------------------------------------------------------------
    logic [$bits(axi.awaddr)-1:0] addr_q;
    logic [$bits(axi.wdata)-1:0] wdata_q;
    logic [$bits(axi.wstrb)-1:0] wstrb_q;

    wire accept = (state_q == S_IDLE) && req_i.valid;

    always_ff @(posedge clk_i) begin
        if (accept) begin
            // The native port is 64-bit (CPU_MEM_WIDTH) on the cache build; the
            // AXI4-Lite peri fabric is 32-bit and the LSU drives the low
            // lanes (a word/strb byte b of the request is field bit b), so
            // slice the low half explicitly. The truncation is intentional
            // and would otherwise warn (EX3791) on every field.
            addr_q  <= req_i.addr[$bits(axi.awaddr)-1:0];
            wdata_q <= req_i.wdata[$bits(axi.wdata)-1:0];
            wstrb_q <= req_i.wstrb[$bits(axi.wstrb)-1:0];
        end
    end

    wire ar_hs = axi.arvalid && axi.arready;
    wire aw_hs = axi.awvalid && axi.awready;
    wire w_hs = axi.wvalid && axi.wready;
    wire r_hs = axi.rvalid && axi.rready;
    wire b_hs = axi.bvalid && axi.bready;

    always_comb begin
        // Defaults
        axi.awaddr   = '0;
        axi.awvalid  = 1'b0;
        axi.wdata    = '0;
        axi.wstrb    = '0;
        axi.wvalid   = 1'b0;
        axi.bready   = 1'b1;

        axi.araddr   = '0;
        axi.arvalid  = 1'b0;
        axi.rready   = 1'b0;

        // Accepting is a one-cycle capture (addr/wdata/wstrb latched
        // above the same cycle), so wready can stay unconditional in
        // S_IDLE regardless of how long the downstream AXI target
        // takes to accept AW/AR/W.
        rsp_o.wready = (state_q == S_IDLE);
        rsp_o.rvalid = 1'b0;
        rsp_o.rdata  = '0;
        rsp_o.bvalid = 1'b0;

        state_d      = state_q;

        unique case (state_q)
            S_IDLE: begin
                if (req_i.valid) begin
                    if (req_i.we) begin
                        // Write: launch AW + W in lock-step from the
                        // live request (still valid this cycle; the
                        // latch above captures it in parallel). Cover
                        // all four accept combinations so neither
                        // channel is ever orphaned.
                        // Low lanes of the 64-bit native fields (see the
                        // latch above): the peri AXI fabric is 32-bit.
                        axi.awaddr  = req_i.addr[$bits(axi.awaddr)-1:0];
                        axi.awvalid = 1'b1;
                        axi.wdata   = req_i.wdata[$bits(axi.wdata)-1:0];
                        axi.wstrb   = req_i.wstrb[$bits(axi.wstrb)-1:0];
                        axi.wvalid  = 1'b1;
                        if (aw_hs && w_hs) begin
                            state_d = S_WR_WAIT;
                        end else if (aw_hs) begin
                            // AW done, W outstanding: retry W.
                            state_d = S_WR_ADDR;
                        end else if (w_hs) begin
                            // W done, AW outstanding: retry AW.
                            state_d = S_WR_DATA;
                        end else begin
                            // Neither accepted this cycle: retry both.
                            state_d = S_WR_BOTH;
                        end
                    end else begin
                        // Read: launch AR.
                        axi.araddr  = req_i.addr[$bits(axi.araddr)-1:0];
                        axi.arvalid = 1'b1;
                        state_d     = ar_hs ? S_RD_WAIT : S_RD_ADDR;
                    end
                end
            end

            S_RD_ADDR: begin
                // AR not accepted on the launch cycle: retry from the
                // latched address until the slave accepts it.
                axi.araddr  = addr_q;
                axi.arvalid = 1'b1;
                if (ar_hs) begin
                    state_d = S_RD_WAIT;
                end
            end

            S_RD_WAIT: begin
                // Forward the master's rready to the AXI R channel so the
                // slave holds rvalid/rdata until the master can accept.
                axi.rready   = req_i.rready;
                rsp_o.rvalid = r_hs;
                rsp_o.rdata  = axi.rdata;
                if (r_hs) begin
                    state_d = S_IDLE;
                end
            end

            S_WR_BOTH: begin
                // Neither AW nor W accepted on the launch cycle: retry
                // both from the latch until at least one handshakes,
                // same branch logic as the S_IDLE launch.
                axi.awaddr  = addr_q;
                axi.awvalid = 1'b1;
                axi.wdata   = wdata_q;
                axi.wstrb   = wstrb_q;
                axi.wvalid  = 1'b1;
                if (aw_hs && w_hs) begin
                    state_d = S_WR_WAIT;
                end else if (aw_hs) begin
                    state_d = S_WR_ADDR;
                end else if (w_hs) begin
                    state_d = S_WR_DATA;
                end
                // else: stay in S_WR_BOTH.
            end

            S_WR_ADDR: begin
                // AW hand-shaken, W still outstanding: retry W from the
                // latch. Do not re-drive AW (already accepted).
                axi.wdata  = wdata_q;
                axi.wstrb  = wstrb_q;
                axi.wvalid = 1'b1;
                if (w_hs) begin
                    state_d = S_WR_WAIT;
                end
            end

            S_WR_DATA: begin
                // W hand-shaken, AW still outstanding: retry AW from the
                // latch. Do not re-drive W (already accepted).
                axi.awaddr  = addr_q;
                axi.awvalid = 1'b1;
                if (aw_hs) begin
                    state_d = S_WR_WAIT;
                end
            end

            S_WR_WAIT: begin
                // Write retired: expose bvalid to the native side so the
                // LSU can retire the store. Symmetric with the read path's
                // rsp_o.rvalid = r_hs in S_RD_WAIT (bready is held high, so
                // b_hs fires the cycle the slave raises bvalid).
                rsp_o.bvalid = b_hs;
                if (b_hs) begin
                    state_d = S_IDLE;
                end
            end

            // The 3-bit state_t enum has 1 unused encoding; cover it so
            // the unique case is full (silences EX3005) and recovers to
            // idle.
            default: state_d = S_IDLE;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            state_q <= S_IDLE;
        end else begin
            state_q <= state_d;
        end
    end

endmodule

`resetall

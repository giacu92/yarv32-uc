`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* Machine timer peripheral (CLINT-style, AXI4-Lite slave).
*
* Two 64-bit registers, exposed as four 32-bit MMIO words at MTIMER_BASE
* (peri window 0x1000_1000..0x1000_2FFF, selected by the peri xbar's
* base+size decode):
*
*   +0  mtime_lo     (RO, free-running 64-bit counter, low half)
*   +4  mtime_hi     (RO, free-running 64-bit counter, high half)
*   +8  mtimecmp_lo  (RW, 64-bit compare, low half)
* +0xC  mtimecmp_hi  (RW, 64-bit compare, high half)
*
* mtip_o = (mtime >= mtimecmp) — a level bit that feeds
* csr_regfile.mtip_i (mip.MTIP). A machine timer interrupt fires when
* mstatus.MIE & mip.MTIP & mie.MTIE. SW clears it by writing mtimecmp >
* mtime (mip.MTIP is read-only from CSR write, like MSIP).
*
* mtime counts every cycle from reset (free-running). mtimecmp resets to
* all-ones so mtip_o=0 at boot (no spurious interrupt).
*
* WRITE ORDER for mtimecmp: the 64-bit compare is reachable only as two
* 32-bit words, so an unlucky order transiently arms a value neither the old
* nor the new one. Use the standard RISC-V sequence -- write mtimecmp_hi =
* 0xFFFF_FFFF first, then mtimecmp_lo, then the real mtimecmp_hi. Writing lo
* before hi can fire a spurious timer interrupt in the gap. The 64-bit compare
* is TWO-STAGE PIPELINED (two registered 32-bit compares + a combine flop)
* to cut the carry chain off the timing path — see the compare block. The
* 2-cycle mtip latency is harmless for a level interrupt.
*
* Protocol follows msip_peri / axi4_lite_ram: registered BVALID held until
* the B handshake, RVALID held until RREADY, single-outstanding (AWREADY
* / WREADY / ARREADY low while a response is pending). AW and W may arrive
* in either order; the write commits when both are seen.
*
* Reset is synchronous, active-low.
*
* Naming: ports *_i/_o; internals no prefix; flops _q, next-state _d.
*/

module clint_timer (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    output wire mtip_o,

    // The free-running counter itself, for the CPU's time / timeh CSRs
    // (Zicntr). A wire off mtime_q, so CSR and MMIO read one clock.
    output wire [63:0] mtime_o
);

    localparam int DATA_W = axi.DATA_WIDTH;

    // Register select: addr[3:2] over the 16-byte window.
    //   2'b00 = mtime_lo, 01 = mtime_hi, 10 = mtimecmp_lo, 11 = mtimecmp_hi.
    localparam logic [1:0] REG_MTIME_LO = 2'b00;
    localparam logic [1:0] REG_MTIME_HI = 2'b01;
    localparam logic [1:0] REG_MTIMECMP_LO = 2'b10;
    localparam logic [1:0] REG_MTIMECMP_HI = 2'b11;

    // =================================================================
    // 64-bit counter + compare
    // =================================================================
    logic [63:0] mtime_q;
    logic [63:0] mtimecmp_q;

    assign mtime_o = mtime_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            mtime_q <= '0;
        end else begin
            mtime_q <= mtime_q + 1;
        end
    end

    // Two-stage pipelined 64-bit compare. The full `mtime >= mtimecmp`
    // carry chain is ~26 logic levels — too long to close 40 MHz even into a
    // single flop (the comb compare -> mtip_q.D path was the PnR critical
    // path, ~34.7 MHz Fmax). Split it: stage 1 registers two parallel 32-bit
    // compares (high half gt / eq, low half ge), each ~13 levels into a flop;
    // stage 2 combines them (ge_hi | (eq_hi & ge_lo), a few gates) into mtip_q.
    //
    // Both halves sample the SAME cycle-T mtime_q/mtimecmp_q into stage-1
    // flops, so the combined result is a consistent 64-bit `>=` of cycle T —
    // no high/low time-skew (mtime increments +1/cycle). mtip_o is delayed 2
    // cycles vs mtime. A level interrupt is unaffected: it stays asserted
    // until SW writes mtimecmp > mtime, and is taken at a retire boundary /
    // WFI wake, so 2 extra cycles of latency are invisible.
    logic ge_hi_q;  // stage-1: mtime_hi >  mtimecmp_hi
    logic eq_hi_q;  // stage-1: mtime_hi == mtimecmp_hi
    logic ge_lo_q;  // stage-1: mtime_lo >= mtimecmp_lo

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            ge_hi_q <= 1'b0;
            eq_hi_q <= 1'b0;
            ge_lo_q <= 1'b0;
        end else begin
            ge_hi_q <= (mtime_q[63:32] > mtimecmp_q[63:32]);
            eq_hi_q <= (mtime_q[63:32] == mtimecmp_q[63:32]);
            ge_lo_q <= (mtime_q[31:0] >= mtimecmp_q[31:0]);
        end
    end

    logic mtip_q;
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            mtip_q <= 1'b0;
        end else begin
            mtip_q <= ge_hi_q | (eq_hi_q & ge_lo_q);
        end
    end

    assign mtip_o = mtip_q;

    // =================================================================
    // Write path (AW + W independent; complete when both seen).
    // =================================================================
    logic aw_seen_q, w_seen_q;
    logic [DATA_W-1:0] wdata_q;
    logic [       1:0] awaddr_q;  // latched addr[3:2]
    logic              bvalid_q;

    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;

    wire aw_hs = axi.awvalid && axi.awready;
    wire w_hs = axi.wvalid && axi.wready;

    wire aw_present = aw_seen_q || aw_hs;
    wire w_present = w_seen_q || w_hs;
    wire do_write = aw_present && w_present && !bvalid_q;

    // Effective address decode: live on the launch cycle, latched once
    // AW is seen.
    wire [1:0] aw_sel_live = axi.awaddr[3:2];
    wire [1:0] aw_sel_eff = aw_seen_q ? awaddr_q : aw_sel_live;

    wire [DATA_W-1:0] wdata_eff = w_hs ? axi.wdata : wdata_q;

    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;  // OKAY
    wire b_hs = axi.bvalid && axi.bready;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q  <= 1'b0;
            w_seen_q   <= 1'b0;
            wdata_q    <= '0;
            awaddr_q   <= 2'b00;
            bvalid_q   <= 1'b0;
            mtimecmp_q <= '1;  // mtip_o = (0 >= ~0) = 0 at boot
        end else begin
            if (aw_hs) begin
                aw_seen_q <= 1'b1;
                awaddr_q  <= aw_sel_live;
            end
            if (w_hs) begin
                w_seen_q <= 1'b1;
                wdata_q  <= axi.wdata;
            end

            // Both phases present: commit the word and raise BVALID.
            // mtime is RO — writes to it are silently ignored (no error).
            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;
                case (aw_sel_eff)
                    REG_MTIMECMP_LO: mtimecmp_q[31:0] <= wdata_eff;
                    REG_MTIMECMP_HI: mtimecmp_q[63:32] <= wdata_eff;
                    default: ;  // mtime_lo / mtime_hi: RO, ignore
                endcase
            end

            if (b_hs) begin
                bvalid_q <= 1'b0;
            end
        end
    end

    // =================================================================
    // Read path (1-cycle registered latency). Snapshot the selected word
    // at the AR handshake so a mtime read captures a stable count.
    // =================================================================
    logic              rvalid_q;
    logic [DATA_W-1:0] rdata_q;

    assign axi.arready = !rvalid_q || (rvalid_q && axi.rready);
    wire ar_hs = axi.arvalid && axi.arready;

    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;  // OKAY

    // Read data at the launch cycle (combinational off the live address).
    logic [DATA_W-1:0] rdata_launch;
    always_comb begin
        case (axi.araddr[3:2])
            REG_MTIME_LO:    rdata_launch = mtime_q[31:0];
            REG_MTIME_HI:    rdata_launch = mtime_q[63:32];
            REG_MTIMECMP_LO: rdata_launch = mtimecmp_q[31:0];
            REG_MTIMECMP_HI: rdata_launch = mtimecmp_q[63:32];
            default:         rdata_launch = '0;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q <= 1'b0;
            rdata_q  <= '0;
        end else begin
            if (ar_hs) begin
                rvalid_q <= 1'b1;
                rdata_q  <= rdata_launch;
            end else if (rvalid_q && axi.rready) begin
                rvalid_q <= 1'b0;
            end
        end
    end

`ifdef VERILATOR
    initial begin
        mtime_q    = '0;
        mtimecmp_q = '1;
        ge_hi_q    = 1'b0;
        eq_hi_q    = 1'b0;
        ge_lo_q    = 1'b0;
        mtip_q     = 1'b0;
    end
`endif

endmodule

`resetall

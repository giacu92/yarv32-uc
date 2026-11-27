`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* SiFive-layout CLINT (AXI4-Lite slave), one hart, for the Linux branch (M8).
*
* One 64 KiB window, the layout the mainline drivers and OpenSBI hard-code
* for `compatible = "sifive,clint0", "riscv,clint0"`:
*
*   +0x0000  msip[0]        bit 0 = mip.MSIP of hart 0, RW
*   +0x4000  mtimecmp[0]    low  word, RW
*   +0x4004  mtimecmp[0]    high word, RW
*   +0xBFF8  mtime          low  word, RW
*   +0xBFFC  mtime          high word, RW
*
* Everything else in the window reads 0 and ignores writes (OKAY response).
*
* Replaces the MCU's split pair: msip_peri (0x1000_3000) + clint_timer
* (0x1000_1000, four consecutive words), whose offsets no driver knows.
* Kept from clint_timer: mtimecmp resets to all-ones (no MTIP at boot), the
* two-stage pipelined 64-bit compare (the full carry chain into one flop was
* the 40 MHz limiter once), and mtime_o, which feeds the time / timeh CSRs.
*
* mtime is WRITABLE here (it was read-only in clint_timer): the privileged
* spec makes it a read/write register and OpenSBI's ACLINT/CLINT code may
* write it; read-only was a simplification nothing relied on. A write to one
* half replaces that half and suppresses the increment for that cycle (the
* other half holds), so software can set mtime exactly.
*
* Byte strobes are honoured per lane on every register.
*
* Reset is synchronous, active-low. Naming: ports *_i/_o; flops _q.
*/

module clint (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    output wire        msip_o,
    output wire        mtip_o,
    output wire [63:0] mtime_o
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;

    // Register select over the 64 KiB window (byte offset [15:2]).
    localparam logic [15:0] OFF_MSIP = 16'h0000;
    localparam logic [15:0] OFF_MTIMECMP_LO = 16'h4000;
    localparam logic [15:0] OFF_MTIMECMP_HI = 16'h4004;
    localparam logic [15:0] OFF_MTIME_LO = 16'hBFF8;
    localparam logic [15:0] OFF_MTIME_HI = 16'hBFFC;

    logic        msip_q;
    logic [63:0] mtime_q;
    logic [63:0] mtimecmp_q;

    assign msip_o  = msip_q;
    assign mtime_o = mtime_q;

    // Two-stage pipelined compare, as in clint_timer: mtip_o lags mtime by
    // two cycles, invisible for a level interrupt.
    logic ge_hi_q, eq_hi_q, ge_lo_q, mtip_q;
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            ge_hi_q <= 1'b0;
            eq_hi_q <= 1'b0;
            ge_lo_q <= 1'b0;
            mtip_q  <= 1'b0;
        end else begin
            ge_hi_q <= (mtime_q[63:32] > mtimecmp_q[63:32]);
            eq_hi_q <= (mtime_q[63:32] == mtimecmp_q[63:32]);
            ge_lo_q <= (mtime_q[31:0] >= mtimecmp_q[31:0]);
            mtip_q  <= ge_hi_q | (eq_hi_q & ge_lo_q);
        end
    end
    assign mtip_o = mtip_q;

    // ---------------------------------------------------------------
    // AXI write side (single outstanding, BVALID held)
    // ---------------------------------------------------------------
    logic aw_seen_q, w_seen_q, bvalid_q;
    logic [DATA_W-1:0] wdata_q;
    logic [STRB_W-1:0] wstrb_q;
    logic [      15:0] awaddr_q;

    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;
    wire aw_hs = axi.awvalid && axi.awready;
    wire w_hs = axi.wvalid && axi.wready;
    wire do_write = (aw_seen_q || aw_hs) && (w_seen_q || w_hs) && !bvalid_q;
    wire [15:0] waddr = aw_seen_q ? awaddr_q : {axi.awaddr[15:2], 2'b00};
    wire [DATA_W-1:0] wdata = w_seen_q ? wdata_q : axi.wdata;
    wire [STRB_W-1:0] wstrb = w_seen_q ? wstrb_q : axi.wstrb;
    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;

    function automatic logic [31:0] merge(input logic [31:0] old, input logic [31:0] d,
                                          input logic [3:0] s);
        for (int i = 0; i < 4; i++) if (s[i]) old[8*i+:8] = d[8*i+:8];
        return old;
    endfunction

    wire wr_mtime_lo = do_write && (waddr == OFF_MTIME_LO);
    wire wr_mtime_hi = do_write && (waddr == OFF_MTIME_HI);

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q  <= 1'b0;
            w_seen_q   <= 1'b0;
            bvalid_q   <= 1'b0;
            wdata_q    <= '0;
            wstrb_q    <= '0;
            awaddr_q   <= '0;
            msip_q     <= 1'b0;
            mtimecmp_q <= '1;
            mtime_q    <= '0;
        end else begin
            if (aw_hs) begin
                aw_seen_q <= 1'b1;
                awaddr_q  <= {axi.awaddr[15:2], 2'b00};
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
                unique case (waddr)
                    OFF_MSIP: if (wstrb[0]) msip_q <= wdata[0];
                    OFF_MTIMECMP_LO: mtimecmp_q[31:0] <= merge(mtimecmp_q[31:0], wdata, wstrb);
                    OFF_MTIMECMP_HI: mtimecmp_q[63:32] <= merge(mtimecmp_q[63:32], wdata, wstrb);
                    default: ;
                endcase
            end
            if (axi.bvalid && axi.bready) bvalid_q <= 1'b0;

            // mtime: a write to a half replaces it; otherwise count.
            if (wr_mtime_lo) mtime_q[31:0] <= merge(mtime_q[31:0], wdata, wstrb);
            else if (wr_mtime_hi) mtime_q[63:32] <= merge(mtime_q[63:32], wdata, wstrb);
            else mtime_q <= mtime_q + 64'd1;
        end
    end

    // ---------------------------------------------------------------
    // AXI read side
    // ---------------------------------------------------------------
    logic              rvalid_q;
    logic [DATA_W-1:0] rdata_q;
    assign axi.arready = !rvalid_q || axi.rready;
    wire ar_hs = axi.arvalid && axi.arready;
    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;

    logic [DATA_W-1:0] rmux;
    always_comb begin
        unique case ({
            axi.araddr[15:2], 2'b00
        })
            OFF_MSIP:        rmux = {31'b0, msip_q};
            OFF_MTIMECMP_LO: rmux = mtimecmp_q[31:0];
            OFF_MTIMECMP_HI: rmux = mtimecmp_q[63:32];
            OFF_MTIME_LO:    rmux = mtime_q[31:0];
            OFF_MTIME_HI:    rmux = mtime_q[63:32];
            default:         rmux = '0;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q <= 1'b0;
            rdata_q  <= '0;
        end else if (ar_hs) begin
            rvalid_q <= 1'b1;
            rdata_q  <= rmux;
        end else if (rvalid_q && axi.rready) begin
            rvalid_q <= 1'b0;
        end
    end

endmodule

`resetall

`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * AXI4-Lite 1->4 crossbar: one slave port routed to one of four master
 * ports by base+size address rule. Same single-outstanding pass-through
 * style as the 1->2 (axi4_lite_xbar.sv) and 1->3 (axi4_lite_xbar_3.sv)
 * xbars, extended by one target. Base/size are compile-time localparams
 * (not array ports / dynamic interface-array indexing -- Gowin's
 * elaborator does not support either reliably), matching the working
 * 1->2/1->3 xbars' pattern exactly.
 *
 *   sel 0 -> m0_axi   sel 1 -> m1_axi   sel 2 -> m2_axi   sel 3 -> m3_axi
 *   4 -> no match
 *
 * An address that matches none of the four windows is NOT left to stall.
 * It is completed locally by a decode-error terminator: the write is
 * accepted and answered with BRESP = DECERR, the read is answered with
 * RRESP = DECERR and RDATA = 0. Without it the slave-side ready stays low
 * forever and the CPU's LSU parks in EX_MEM_WAIT with no way out but reset
 * -- which is easy to hit, because the LSU routes the WHOLE
 * 0x1000_0000..0x1FFF_FFFF region here on addr[PERI_ADDR_BIT] while only the
 * four windows are mapped.
 */
module axi4_lite_xbar_4 #(
    parameter logic [31:0] BASE0 = 32'h1000_0000,
    parameter logic [31:0] SIZE0 = 32'h0000_1000,
    parameter logic [31:0] BASE1 = 32'h1000_1000,
    parameter logic [31:0] SIZE1 = 32'h0000_2000,
    parameter logic [31:0] BASE2 = 32'h1000_3000,
    parameter logic [31:0] SIZE2 = 32'h0000_1000,
    parameter logic [31:0] BASE3 = 32'h1000_4000,
    parameter logic [31:0] SIZE3 = 32'h0000_1000
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave  s_axi,
    axi4_lite_if.master m0_axi,
    axi4_lite_if.master m1_axi,
    axi4_lite_if.master m2_axi,
    axi4_lite_if.master m3_axi
);

    // -----------------------------------------------------------------
    // Address decode (combinational function -> 3-bit select, 4 = no
    // match). Function of a constant-width scalar addr, not an array
    // index, so it elaborates like any other combinational helper.
    // -----------------------------------------------------------------
    function automatic logic [2:0] decode(input logic [31:0] addr);
        if (addr >= BASE0 && addr < BASE0 + SIZE0) return 3'd0;
        if (addr >= BASE1 && addr < BASE1 + SIZE1) return 3'd1;
        if (addr >= BASE2 && addr < BASE2 + SIZE2) return 3'd2;
        if (addr >= BASE3 && addr < BASE3 + SIZE3) return 3'd3;
        return 3'd4;  // no match
    endfunction

    // Decode-error terminator state (sel == 3'd4). Mirrors the minimal
    // slave handshake the real peripherals use: registered BVALID held until
    // the B handshake, RVALID held until RREADY, single-outstanding.
    logic err_aw_seen_q;
    logic err_w_seen_q;
    logic err_bvalid_q;
    logic err_rvalid_q;

    wire  err_awready = !err_aw_seen_q && !err_bvalid_q;
    wire  err_wready = !err_w_seen_q && !err_bvalid_q;
    wire  err_arready = !err_rvalid_q;

    localparam logic [1:0] RESP_DECERR = 2'b11;

    logic       wr_busy_q;
    logic [2:0] wr_sel_q;
    logic       rd_busy_q;
    logic [2:0] rd_sel_q;

    wire  [2:0] wr_sel_live = decode(s_axi.awaddr);
    wire  [2:0] rd_sel_live = decode(s_axi.araddr);
    wire  [2:0] wr_sel_eff = wr_busy_q ? wr_sel_q : wr_sel_live;

    wire        aw_hs = s_axi.awvalid && s_axi.awready;
    wire        b_hs = s_axi.bvalid && s_axi.bready;
    wire        ar_hs = s_axi.arvalid && s_axi.arready;
    wire        r_hs = s_axi.rvalid && s_axi.rready;

    always_comb begin
        // ---- Defaults: drive nothing on any master ----
        m0_axi.awaddr  = s_axi.awaddr;
        m0_axi.awvalid = 1'b0;
        m0_axi.wdata   = s_axi.wdata;
        m0_axi.wstrb   = s_axi.wstrb;
        m0_axi.wvalid  = 1'b0;
        m0_axi.bready  = 1'b0;
        m0_axi.araddr  = s_axi.araddr;
        m0_axi.arvalid = 1'b0;
        m0_axi.rready  = 1'b0;

        m1_axi.awaddr  = s_axi.awaddr;
        m1_axi.awvalid = 1'b0;
        m1_axi.wdata   = s_axi.wdata;
        m1_axi.wstrb   = s_axi.wstrb;
        m1_axi.wvalid  = 1'b0;
        m1_axi.bready  = 1'b0;
        m1_axi.araddr  = s_axi.araddr;
        m1_axi.arvalid = 1'b0;
        m1_axi.rready  = 1'b0;

        m2_axi.awaddr  = s_axi.awaddr;
        m2_axi.awvalid = 1'b0;
        m2_axi.wdata   = s_axi.wdata;
        m2_axi.wstrb   = s_axi.wstrb;
        m2_axi.wvalid  = 1'b0;
        m2_axi.bready  = 1'b0;
        m2_axi.araddr  = s_axi.araddr;
        m2_axi.arvalid = 1'b0;
        m2_axi.rready  = 1'b0;

        m3_axi.awaddr  = s_axi.awaddr;
        m3_axi.awvalid = 1'b0;
        m3_axi.wdata   = s_axi.wdata;
        m3_axi.wstrb   = s_axi.wstrb;
        m3_axi.wvalid  = 1'b0;
        m3_axi.bready  = 1'b0;
        m3_axi.araddr  = s_axi.araddr;
        m3_axi.arvalid = 1'b0;
        m3_axi.rready  = 1'b0;

        s_axi.awready  = 1'b0;
        s_axi.wready   = 1'b0;
        s_axi.bvalid   = 1'b0;
        s_axi.bresp    = 2'b00;
        s_axi.arready  = 1'b0;
        s_axi.rvalid   = 1'b0;
        s_axi.rdata    = '0;
        s_axi.rresp    = 2'b00;

        // ---- AW ----
        if (!wr_busy_q && s_axi.awvalid) begin
            unique case (wr_sel_live)
                3'd0: begin
                    m0_axi.awvalid = 1'b1;
                    s_axi.awready  = m0_axi.awready;
                end
                3'd1: begin
                    m1_axi.awvalid = 1'b1;
                    s_axi.awready  = m1_axi.awready;
                end
                3'd2: begin
                    m2_axi.awvalid = 1'b1;
                    s_axi.awready  = m2_axi.awready;
                end
                3'd3: begin
                    m3_axi.awvalid = 1'b1;
                    s_axi.awready  = m3_axi.awready;
                end
                default: s_axi.awready = err_awready;  // no match: DECERR
            endcase
        end

        // ---- W ----
        if (s_axi.wvalid) begin
            unique case (wr_sel_eff)
                3'd0: begin
                    m0_axi.wvalid = 1'b1;
                    s_axi.wready  = m0_axi.wready;
                end
                3'd1: begin
                    m1_axi.wvalid = 1'b1;
                    s_axi.wready  = m1_axi.wready;
                end
                3'd2: begin
                    m2_axi.wvalid = 1'b1;
                    s_axi.wready  = m2_axi.wready;
                end
                3'd3: begin
                    m3_axi.wvalid = 1'b1;
                    s_axi.wready  = m3_axi.wready;
                end
                default: s_axi.wready = err_wready;  // no match: DECERR
            endcase
        end

        // ---- B ----
        if (wr_busy_q) begin
            unique case (wr_sel_q)
                3'd0: begin
                    s_axi.bvalid  = m0_axi.bvalid;
                    s_axi.bresp   = m0_axi.bresp;
                    m0_axi.bready = s_axi.bready;
                end
                3'd1: begin
                    s_axi.bvalid  = m1_axi.bvalid;
                    s_axi.bresp   = m1_axi.bresp;
                    m1_axi.bready = s_axi.bready;
                end
                3'd2: begin
                    s_axi.bvalid  = m2_axi.bvalid;
                    s_axi.bresp   = m2_axi.bresp;
                    m2_axi.bready = s_axi.bready;
                end
                3'd3: begin
                    s_axi.bvalid  = m3_axi.bvalid;
                    s_axi.bresp   = m3_axi.bresp;
                    m3_axi.bready = s_axi.bready;
                end
                default: begin  // no match: DECERR
                    s_axi.bvalid = err_bvalid_q;
                    s_axi.bresp  = RESP_DECERR;
                end
            endcase
        end

        // ---- AR ----
        if (!rd_busy_q && s_axi.arvalid) begin
            unique case (rd_sel_live)
                3'd0: begin
                    m0_axi.arvalid = 1'b1;
                    s_axi.arready  = m0_axi.arready;
                end
                3'd1: begin
                    m1_axi.arvalid = 1'b1;
                    s_axi.arready  = m1_axi.arready;
                end
                3'd2: begin
                    m2_axi.arvalid = 1'b1;
                    s_axi.arready  = m2_axi.arready;
                end
                3'd3: begin
                    m3_axi.arvalid = 1'b1;
                    s_axi.arready  = m3_axi.arready;
                end
                default: s_axi.arready = err_arready;  // no match: DECERR
            endcase
        end

        // ---- R ----
        if (rd_busy_q) begin
            unique case (rd_sel_q)
                3'd0: begin
                    s_axi.rvalid  = m0_axi.rvalid;
                    s_axi.rdata   = m0_axi.rdata;
                    s_axi.rresp   = m0_axi.rresp;
                    m0_axi.rready = s_axi.rready;
                end
                3'd1: begin
                    s_axi.rvalid  = m1_axi.rvalid;
                    s_axi.rdata   = m1_axi.rdata;
                    s_axi.rresp   = m1_axi.rresp;
                    m1_axi.rready = s_axi.rready;
                end
                3'd2: begin
                    s_axi.rvalid  = m2_axi.rvalid;
                    s_axi.rdata   = m2_axi.rdata;
                    s_axi.rresp   = m2_axi.rresp;
                    m2_axi.rready = s_axi.rready;
                end
                3'd3: begin
                    s_axi.rvalid  = m3_axi.rvalid;
                    s_axi.rdata   = m3_axi.rdata;
                    s_axi.rresp   = m3_axi.rresp;
                    m3_axi.rready = s_axi.rready;
                end
                default: begin  // no match: DECERR, rdata = 0
                    s_axi.rvalid = err_rvalid_q;
                    s_axi.rdata  = '0;
                    s_axi.rresp  = RESP_DECERR;
                end
            endcase
        end
    end

    // Decode-error terminator handshakes (only when the live/latched select
    // is the no-match code).
    wire err_aw_hs = aw_hs && (wr_sel_live == 3'd4);
    wire err_w_hs = s_axi.wvalid && s_axi.wready && (wr_sel_eff == 3'd4);
    wire err_do_write = (err_aw_seen_q || err_aw_hs) && (err_w_seen_q || err_w_hs) && !err_bvalid_q;
    wire err_b_hs = b_hs && (wr_sel_q == 3'd4);
    wire err_ar_hs = ar_hs && (rd_sel_live == 3'd4);
    wire err_r_hs = r_hs && (rd_sel_q == 3'd4);

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            wr_busy_q     <= 1'b0;
            wr_sel_q      <= 3'd0;
            rd_busy_q     <= 1'b0;
            rd_sel_q      <= 3'd0;
            err_aw_seen_q <= 1'b0;
            err_w_seen_q  <= 1'b0;
            err_bvalid_q  <= 1'b0;
            err_rvalid_q  <= 1'b0;
        end else begin
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

            if (aw_hs) begin
                wr_busy_q <= 1'b1;
                wr_sel_q  <= wr_sel_live;
            end else if (b_hs) begin
                wr_busy_q <= 1'b0;
            end

            if (ar_hs) begin
                rd_busy_q <= 1'b1;
                rd_sel_q  <= rd_sel_live;
            end else if (r_hs) begin
                rd_busy_q <= 1'b0;
            end
        end
    end

endmodule

`resetall

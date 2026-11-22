`resetall
`timescale 1ns / 1ps
`default_nettype none

// linux-soc M8 testbench wrapper: axi4_lite_plic_sifive behind flat AXI4-Lite ports, driven
// by the C++ BFM in plic_sifive_tb.cpp (shared helpers in ../axi_bfm.h).
module plic_sifive_tb (
    input  wire        clk_i,
    input  wire        rstn_i,
    input  wire [31:0] awaddr,
    input  wire        awvalid,
    input  wire [31:0] wdata,
    input  wire [ 3:0] wstrb,
    input  wire        wvalid,
    input  wire        bready,
    input  wire [31:0] araddr,
    input  wire        arvalid,
    input  wire        rready,
    output wire        awready,
    output wire        wready,
    output wire        bvalid,
    output wire [ 1:0] bresp,
    output wire        arready,
    output wire [31:0] rdata,
    output wire [ 1:0] rresp,
    output wire        rvalid,
    input  wire [15:0] irq_i,
    output wire        meip_o,
    output wire        seip_o
);
    axi4_lite_if axi ();
    assign axi.aclk    = clk_i;
    assign axi.aresetn = rstn_i;
    assign axi.awaddr  = awaddr;
    assign axi.awvalid = awvalid;
    assign axi.wdata   = wdata;
    assign axi.wstrb   = wstrb;
    assign axi.wvalid  = wvalid;
    assign axi.bready  = bready;
    assign axi.araddr  = araddr;
    assign axi.arvalid = arvalid;
    assign axi.rready  = rready;
    assign awready     = axi.awready;
    assign wready      = axi.wready;
    assign bvalid      = axi.bvalid;
    assign bresp       = axi.bresp;
    assign arready     = axi.arready;
    assign rdata       = axi.rdata;
    assign rresp       = axi.rresp;
    assign rvalid      = axi.rvalid;

    axi4_lite_plic_sifive #(
        .SRC_N(16)
    ) u_dut (
        .clk_i (clk_i),
        .rstn_i(rstn_i),
        .axi   (axi.slave),
        .irq_i (irq_i),
        .meip_o(meip_o),
        .seip_o(seip_o)
    );
endmodule

`resetall

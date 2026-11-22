`resetall
`timescale 1ns / 1ps
`default_nettype none

// linux-soc M8 testbench wrapper: axi4_lite_uart16550 behind flat AXI4-Lite ports, driven
// by the C++ BFM in uart16550_tb.cpp (shared helpers in ../axi_bfm.h).
module uart16550_tb (
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
    output wire        txd_o,
    input  wire        rxd_i,
    output wire        irq_o
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

    // CLK/BAUD chosen so DL resets to 2: one bit = 32 clocks.
    axi4_lite_uart16550 #(
        .CLK_FREQ_HZ(3_200_000),
        .BAUD_RATE  (100_000)
    ) u_dut (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi.slave),
        .txd_o     (txd_o),
        .rxd_i     (rxd_i),
        .uart_irq_o(irq_o)
    );
endmodule

`resetall

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Focused axi4_lite_fft testbench wrapper.
 *
 * Exposes the peripheral's AXI4-Lite slave port and its interrupt line as
 * flat ports so the C++ BFM (fft_tb.cpp) can drive them directly. The FFT
 * has no pins at all -- its sample buffers are private BSRAM -- so the AXI
 * port and fft_irq_o are the entire interface.
 *
 * NMAX is cut to 64 here, not the board's 1024. The BFM checks every
 * transform against a bit-exact model of the datapath, and a 64-point
 * engine exercises the same stage/copy/bit-reverse structure at a
 * fraction of the runtime; the full 1024-point geometry is covered by the
 * LOG2N = 10 case, which the peripheral reaches by clamping (CONF is
 * clamped to LOG2NMAX on write, so a LOG2N = 6 build reports 6).
 *
 * WHY ALSO A SEPARATE 1024-POINT BUILD: the twiddle ROM is generated for
 * NMAX = 1024 and indexed in NMAX units, so a smaller NMAX here would use
 * a different table. The wrapper therefore instantiates the peripheral at
 * its shipping size and the BFM simply runs short transforms in it --
 * LOG2N is a runtime register, so N = 4, 8, 64 and 1024 are all reachable
 * from one instance.
 */

module fft_tb (
    input wire clk_i,
    input wire rstn_i,

    // Master -> slave (driven by the C++ BFM)
    input wire [31:0] awaddr,
    input wire        awvalid,
    input wire [31:0] wdata,
    input wire [ 3:0] wstrb,
    input wire        wvalid,
    input wire        bready,
    input wire [31:0] araddr,
    input wire        arvalid,
    input wire        rready,

    // Slave -> master (sampled by the C++ BFM)
    output wire        awready,
    output wire        wready,
    output wire        bvalid,
    output wire [ 1:0] bresp,
    output wire        arready,
    output wire [31:0] rdata,
    output wire [ 1:0] rresp,
    output wire        rvalid,

    output wire fft_irq_o
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

    axi4_lite_fft #(
        .NMAX    (1024),
        .LOG2NMAX(10)
    ) u_fft (
        .clk_i    (clk_i),
        .rstn_i   (rstn_i),
        .axi      (axi.slave),
        .fft_irq_o(fft_irq_o)
    );

endmodule

`resetall

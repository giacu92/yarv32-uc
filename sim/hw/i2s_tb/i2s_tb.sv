`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Focused axi4_lite_i2s testbench wrapper.
 *
 * Instantiates the I2S receiver as an AXI4-Lite slave and exposes the
 * master-side AXI signals, the three audio inputs and the IRQ line as flat
 * ports, so the C++ BFM (i2s_tb.cpp) bit-bangs the audio bus itself
 * instead of going through the i2s_master_model that sim_top uses. Driving
 * the bus by hand is the point: it is the only way to test a frame the
 * model cannot produce -- a truncated word, a slot longer than WLEN, a
 * receiver enabled in the middle of a word, a BCLK that stops.
 *
 * What this test exists to cover:
 *   - reset values and CTRL/WATER readback;
 *   - I2S (1-BCLK delay) and left-justified framing;
 *   - every WLEN encoding (8/16/24/32) and the sign extension of each,
 *     including the most negative word;
 *   - the channel tag carried per FIFO entry, and the CHAN filter;
 *   - padding bits between the end of a word and the next WS transition;
 *   - resynchronization: enabling mid-word costs one word, not lock;
 *   - FIFO fill to exactly DEPTH, overrun sticky + W1C, set-wins-over-clear;
 *   - watermark IRQ as a LEVEL (it must reassert without a clear) and
 *     IE_RX / IE_OVR gating;
 *   - ENABLE=0 capturing nothing;
 *   - master mode: the generated BCLK period and frame length against
 *     CLKDIV, the SLOT=0 reads-as-32 rule, and a full loopback where the
 *     BFM plays an INMP441-style 24-bit slave against the generated clocks
 *     and the receiver has to decode what it sent.
 *
 * The audio inputs are driven synchronously with the clock: the module's
 * contract is that they arrive already synchronized (the tops own the
 * 2-flop chain), so the synchronizer is exercised at the tops, not here.
 */

module i2s_tb (
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

    // I2S bus. The three sampled lines are inputs in both modes; the two
    // generated clocks and their output enable come back out, and the BFM
    // closes the loop through them in master mode exactly as a pad does.
    input  wire i2s_bclk_i,
    input  wire i2s_lrck_i,
    input  wire i2s_sd_i,
    output wire i2s_bclk_o,
    output wire i2s_lrck_o,
    output wire i2s_clk_oe_o,
    output wire i2s_irq_o
);

    axi4_lite_if axi ();

    assign axi.aclk    = clk_i;
    assign axi.aresetn = rstn_i;

    // Master side, driven from the BFM inputs.
    assign axi.awaddr  = awaddr;
    assign axi.awvalid = awvalid;
    assign axi.wdata   = wdata;
    assign axi.wstrb   = wstrb;
    assign axi.wvalid  = wvalid;
    assign axi.bready  = bready;
    assign axi.araddr  = araddr;
    assign axi.arvalid = arvalid;
    assign axi.rready  = rready;

    // Slave side, exposed to the BFM.
    assign awready     = axi.awready;
    assign wready      = axi.wready;
    assign bvalid      = axi.bvalid;
    assign bresp       = axi.bresp;
    assign arready     = axi.arready;
    assign rdata       = axi.rdata;
    assign rresp       = axi.rresp;
    assign rvalid      = axi.rvalid;

    axi4_lite_i2s #(
        .RX_FIFO_DEPTH(16)
    ) u_i2s (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
        .axi         (axi.slave),
        .i2s_bclk_i  (i2s_bclk_i),
        .i2s_lrck_i  (i2s_lrck_i),
        .i2s_sd_i    (i2s_sd_i),
        .i2s_bclk_o  (i2s_bclk_o),
        .i2s_lrck_o  (i2s_lrck_o),
        .i2s_clk_oe_o(i2s_clk_oe_o),
        .i2s_irq_o   (i2s_irq_o)
    );

endmodule

`resetall

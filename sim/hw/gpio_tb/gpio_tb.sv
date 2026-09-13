`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Focused axi4_lite_gpio testbench wrapper.
 *
 * Instantiates the GPIO as an AXI4-Lite slave and exposes the master-side
 * AXI signals, the pin outputs and the IRQ line as flat ports so the C++
 * BFM (gpio_tb.cpp) can drive/observe them directly. gpio_i is driven by
 * the BFM: this is the only place in the tree where an externally-driven
 * pin input is tested (sim_top loops the pins back onto themselves).
 *
 * What this test exists to cover:
 *   - Reset values: OUT/DIR/INT_EN/INT_STATUS 0, pins released.
 *   - OUT/DIR readback; gpio_o follows OUT; gpio_oe_o follows DIR.
 *   - Rise/fall/both edge detect pends the pin; W1C clears an edge pin
 *     and it STAYS clear (an edge is an event, not a level).
 *   - Level type pends while the pin is high; W1C does not stick.
 *   - Pending latches with INT_EN=0 (event record, PLIC-style) and the IRQ
 *     raises the moment INT_EN is set.
 *   - A W1C write with strobe 0x0 (no byte-0) touches nothing.
 */

module gpio_tb (
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

    // GPIO pins + level IRQ
    output wire [3:0] gpio_o,
    output wire [3:0] gpio_oe_o,
    input  wire [3:0] gpio_i,
    output wire       gpio_irq_o
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

    axi4_lite_gpio #(
        .WIDTH(4)
    ) u_gpio (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi.slave),
        .gpio_i    (gpio_i),
        .gpio_o    (gpio_o),
        .gpio_oe_o (gpio_oe_o),
        .gpio_irq_o(gpio_irq_o)
    );

endmodule

`resetall

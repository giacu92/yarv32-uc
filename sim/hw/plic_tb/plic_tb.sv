`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Focused axi4_lite_plic testbench wrapper.
 *
 * Instantiates the PLIC as an AXI4-Lite slave and exposes the master-side
 * AXI signals, the IRQ input vector and the meip output as flat ports so
 * the C++ BFM (plic_tb.cpp) can drive/observe them directly.
 *
 * What this test exists to cover:
 *   - Reset values: PENDING=0, ENABLE=all-ones (the drop-in-for-the-OR
 *     contract: meip degenerates to the OR of the lines), CLAIM reads 0.
 *   - Pending tracks the lines; meip = any enabled pending.
 *   - Disabling a source masks meip and makes it unclaimable.
 *   - CLAIM returns the lowest-ID enabled pending source; other sources
 *     are untouched.
 *   - A CLAIM of a still-high level source does not clear anything
 *     (pending is the line) -- the ISR must service the source.
 *   - Simultaneous sources claim in ID order.
 *   - Source 0 can never claim (it is "no source").
 */

module plic_tb (
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

    // IRQ lines in (bit i = source ID i), meip out
    input  wire [15:0] irq_i,
    output wire        meip_o
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

    axi4_lite_plic #(
        .SRC_N(16)
    ) u_plic (
        .clk_i (clk_i),
        .rstn_i(rstn_i),
        .axi   (axi.slave),
        .irq_i (irq_i),
        .meip_o(meip_o)
    );

endmodule

`resetall

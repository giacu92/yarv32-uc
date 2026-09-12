`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Focused axi4_lite_i2c testbench wrapper.
 *
 * Instantiates the I2C master as an AXI4-Lite slave and exposes the
 * master-side AXI signals, the open-drain bus lines and the IRQ line as
 * flat ports so the C++ BFM (i2c_tb.cpp) can drive/observe them directly.
 * NOT part of synthesis.
 *
 * The open-drain bus is modelled in 2-state as a wired-AND with a pull-up:
 * any driver pulling low (master scl_oe_o/sda_oe_o OR the C++ slave's
 * slave_scl_oe/slave_sda_oe) makes the line low, otherwise the pull-up
 * holds it high. The line value feeds back into scl_i/sda_i. This matches
 * the real `assign scl_io = scl_oe_o ? 1'b0 : 1'bz;` topology without
 * needing Verilator tri-state resolution.
 *
 * What this test exists to cover:
 *   - write_reg  (addr+W, reg, val) -- the example library in the header.
 *   - read_reg   (addr+W reg, repeated-START, addr+R, data byte NACK+STOP).
 *   - multi-byte write and read (register pointer auto-increment).
 *   - slave NACK on a write byte is reported in STATUS / IRQ.
 *   - RX overrun: read bytes beyond the FIFO depth are dropped and flagged.
 *   - BUSY asserts during a transaction; IRQ is level + W1C sticky.
 *
 * CLKDIV is tiny (SCL = clk / (4*(CLKDIV+1)) = clk/12) so a bit takes 12
 * cycles and a 9-bit byte ~110; the divisor logic is ratio-independent.
 */

module i2c_tb #(
    parameter int unsigned TX_FIFO_DEPTH = 16,
    parameter int unsigned RX_FIFO_DEPTH = 16
) (
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

    // Slave-side open-drain drives (C++ slave model)
    input wire slave_scl_oe,
    input wire slave_sda_oe,

    // Bus lines + level IRQ (sampled by the C++ BFM)
    output wire scl_line,
    output wire sda_line,
    output wire i2c_irq_o
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

    // Open-drain wired-AND with pull-up (2-state model).
    wire scl_oe_o, sda_oe_o;
    assign scl_line = (scl_oe_o | slave_scl_oe) ? 1'b0 : 1'b1;
    assign sda_line = (sda_oe_o | slave_sda_oe) ? 1'b0 : 1'b1;

    axi4_lite_i2c #(
        .TX_FIFO_DEPTH(TX_FIFO_DEPTH),
        .RX_FIFO_DEPTH(RX_FIFO_DEPTH)
    ) u_i2c (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi.slave),
        .scl_i     (scl_line),
        .sda_i     (sda_line),
        .scl_oe_o  (scl_oe_o),
        .sda_oe_o  (sda_oe_o),
        .i2c_irq_o (i2c_irq_o)
    );

endmodule

`resetall
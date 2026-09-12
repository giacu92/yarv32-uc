`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Focused axi4_lite_spi testbench wrapper.
 *
 * Instantiates the SPI master as an AXI4-Lite slave and exposes the
 * master-side AXI signals, the SPI pins and the IRQ line as flat ports so
 * the C++ BFM (spi_tb.cpp) can drive/observe them directly. NOT part of
 * synthesis.
 *
 * What this test exists to cover:
 *   - All four CPOL/CPHA modes complete a byte (the CPHA=0 deadlock was the
 *     original bug) and round-trip a known value: the C++ BFM models a real
 *     SPI slave shift register that presents a response byte on MISO and
 *     captures MOSI, so a wrong sample edge garbles the byte and the check
 *     fails -- not a combinational MISO=MOSI loopback, which would hide it.
 *   - TX FIFO burst drains in order with CS held (auto-drain).
 *   - RX FIFO fills in order; RX overrun is latched and the queued bytes
 *     survive.
 *   - BUSY asserts during a transfer and clears after; CS follows REG_CS.
 *   - IRQ is level-sensitive and gated by CTRL; DONE / OVR are W1C sticky.
 *
 * CLKDIV is tiny (the divider logic is ratio-independent): SCL = clk /
 * (2*(CLKDIV+1)) = clk/6, so a bit takes 12 cycles and a byte ~100.
 */

module spi_tb #(
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

    // SPI pins + level IRQ
    output wire spi_sck_o,
    output wire spi_mosi_o,
    input  wire spi_miso_i,
    output wire spi_cs_n_o,
    output wire spi_irq_o
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

    axi4_lite_spi #(
        .TX_FIFO_DEPTH(TX_FIFO_DEPTH),
        .RX_FIFO_DEPTH(RX_FIFO_DEPTH)
    ) u_spi (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi.slave),
        .spi_sck_o (spi_sck_o),
        .spi_mosi_o(spi_mosi_o),
        .spi_miso_i(spi_miso_i),
        .spi_cs_n_o(spi_cs_n_o),
        .spi_irq_o (spi_irq_o)
    );

endmodule

`resetall

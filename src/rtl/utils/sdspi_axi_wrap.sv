`resetall
`timescale 1ns / 1ps
`default_nettype none

// Belt-and-suspenders: also define SDIO_AXI here in case a .gprj reordering
// ever lists this wrapper before sdspi_defs.sv. See sdspi_defs.sv for why
// the macro is needed (sdio_top's AXI-Lite control port is ifdef-gated).
`ifndef SDIO_AXI
`define SDIO_AXI
`endif

import rv32_pkg::*;

/**
 * sdspi_axi_wrap: board-side wrapper around the ZipCPU sdspi SDIO
 * controller (sdio_top), presenting an AXI4-Lite slave on the core's peri
 * bus and the microSD card pins to the Tang Nano 20k SD slot.
 *
 * Configuration: SD card (not eMMC), 4-bit data, AXI-Lite control, NO DMA,
 * no serdes/DDR (the no-serdes front end runs the SD clock at clk_core/4 or
 * slower -- plenty for standard SD). With OPT_SERDES=0 && OPT_DDR=0 the
 * Xilinx-primitive delay/serdes files (xsddelay/xsdddr/xsdserdes8x) are in
 * dead generate branches and are not synthesised.
 *
 * Everything the controller exposes but we do not use is tied off here:
 *   - DMA master port (OPT_DMA=0): M_AXI inputs tied to 0; M_AXI outputs
 *     are driven 0 internally by sdaxil and left unconnected.
 *   - AXI-Stream DMA taps (OPT_ISTREAM/OSTREAM=0): input stream held idle,
 *     output stream never stalled (m_ready=1); outputs unconnected.
 *   - 1.8V signalling / eMMC hardware reset / data strobe: disabled.
 *   - Card detect: no FPGA pin wired to the mechanical switch, so the
 *     controller is told a card is always present (i_card_detect=1) and
 *     OPT_CARD_DETECT=0 keeps it out of the control path.
 *
 * Reset polarity: sdio_top.i_reset is ACTIVE-HIGH (sdaxil uses
 * `if (i_reset)`), so this wrapper inverts the core's active-low rstn_i.
 *
 * sd_int_o carries the controller's o_int (level IRQ). It is not yet ORed
 * into meip -- wire it in top_module when an SD interrupt is wanted.
 */
module sdspi_axi_wrap (
    input wire clk_i,  // core clock (50 MHz)
    input wire rstn_i, // core reset, active-low

    axi4_lite_if.slave s_axi,  // control, from peri xbar m3

    // microSD card slot pins (Tang Nano 20k: PIN80..85, bank 2, 3.3V)
    output wire       sd_clk_o,
    inout  wire       sd_cmd_io,
    inout  wire [3:0] sd_dat_io,

    output wire sd_int_o
);

    // -----------------------------------------------------------------
    // AXI-Lite control (sdio_top's S_AXIL_* uses a 6-bit register
    // address; the peri bus is 32-bit byte-addressed, so take the low 6
    // bits). AWPROT/ARPROT are not used by the controller -> tied 0.
    // -----------------------------------------------------------------
    wire        s_axil_awready;
    wire        s_axil_wready;
    wire        s_axil_bvalid;
    wire [ 1:0] s_axil_bresp;
    wire        s_axil_arready;
    wire        s_axil_rvalid;
    wire [31:0] s_axil_rdata;
    wire [ 1:0] s_axil_rresp;

    // -----------------------------------------------------------------
    // DMA master tie-offs (OPT_DMA=0): drive all M_AXI inputs to 0 so the
    // dead DMA engine never sees a handshake. M_AXI outputs are driven 0
    // by sdaxil when OPT_DMA=0 and are intentionally left unconnected
    // (omitted from the named port map below).
    // -----------------------------------------------------------------
    wire [ 0:0] dma_axi_id_dummy = 1'b0;

    // -----------------------------------------------------------------
    // SDIO controller. NUMIO=4 matches the microSD DAT0..DAT3 pins.
    // -----------------------------------------------------------------
    sdio_top #(
        .LGFIFO           (12),    // 4 KiB internal FIFO
        .NUMIO            (4),
        .ADDRESS_WIDTH    (30),    // DMA byte-address width (unused, no DMA)
        .DW               (32),    // DMA bus width (unused, no DMA)
        .SW               (32),    // stream width (unused)
        .OPT_DMA          (1'b0),  // no DMA master
        .OPT_EMMC         (1'b0),  // SD card, not eMMC
        .OPT_SERDES       (1'b0),  // no serdes -> no IDELAYE2/OSERDES
        .OPT_DDR          (1'b0),  // SDR front end
        .OPT_DS           (1'b0),  // no data strobe (eMMC HS400 only)
        .OPT_1P8V         (1'b0),  // no 1.8V signalling
        .OPT_HWRESET      (1'b0),  // SD cards have no hw reset
        .OPT_CARD_DETECT  (1'b0),  // no card-detect pin in the control path
        .OPT_ISTREAM      (1'b0),  // no input AXI-Stream DMA
        .OPT_OSTREAM      (1'b0),  // no output AXI-Stream DMA
        .OPT_CRCTOKEN     (1'b1),  // expect CRC token after block writes
        .OPT_LITTLE_ENDIAN(1'b1)   // RISC-V is little-endian
    ) u_sdio (
        .i_clk  (clk_i),
        .i_reset(~rstn_i),  // active-high
        .i_hsclk(1'b0),     // unused when OPT_SERDES=0

        // AXI-Lite control
        .S_AXIL_AWVALID(s_axi.awvalid),
        .S_AXIL_AWREADY(s_axil_awready),
        .S_AXIL_AWADDR (s_axi.awaddr[5:0]),
        .S_AXIL_AWPROT (3'b000),
        .S_AXIL_WVALID (s_axi.wvalid),
        .S_AXIL_WREADY (s_axil_wready),
        .S_AXIL_WDATA  (s_axi.wdata),
        .S_AXIL_WSTRB  (s_axi.wstrb),
        .S_AXIL_BVALID (s_axil_bvalid),
        .S_AXIL_BREADY (s_axi.bready),
        .S_AXIL_BRESP  (s_axil_bresp),
        .S_AXIL_ARVALID(s_axi.arvalid),
        .S_AXIL_ARREADY(s_axil_arready),
        .S_AXIL_ARADDR (s_axi.araddr[5:0]),
        .S_AXIL_ARPROT (3'b000),
        .S_AXIL_RVALID (s_axil_rvalid),
        .S_AXIL_RREADY (s_axi.rready),
        .S_AXIL_RDATA  (s_axil_rdata),
        .S_AXIL_RRESP  (s_axil_rresp),

        // DMA master (disabled): inputs tied 0, outputs omitted.
        .M_AXI_AWREADY(1'b0),
        .M_AXI_WREADY (1'b0),
        .M_AXI_BVALID (1'b0),
        .M_AXI_BID    (dma_axi_id_dummy),
        .M_AXI_BRESP  (2'b00),
        .M_AXI_ARREADY(1'b0),
        .M_AXI_RVALID (1'b0),
        .M_AXI_RID    (dma_axi_id_dummy),
        .M_AXI_RDATA  ({32{1'b0}}),
        .M_AXI_RLAST  (1'b0),
        .M_AXI_RRESP  (2'b00),

        // AXI-Stream DMA taps (disabled): idle input, never-stalled output.
        .s_valid(1'b0),
        .s_data ({32{1'b0}}),
        .m_ready(1'b1),

        // SD card slot
        .o_ck  (sd_clk_o),
        .io_cmd(sd_cmd_io),
        .io_dat(sd_dat_io),

        // Misc
        .i_card_detect(1'b1),     // card always present (no detect pin)
        .i_ds         (1'b0),     // no data strobe
        .i_1p8v       (1'b0),     // no 1.8V feedback
        .o_int        (sd_int_o)
        // o_hwreset_n, o_1p8v, o_debug: unconnected (unused outputs)
    );

    // -----------------------------------------------------------------
    // Drive the AXI-Lite slave-side ready/valid/response back to the bus
    // from the controller's outputs.
    // -----------------------------------------------------------------
    assign s_axi.awready = s_axil_awready;
    assign s_axi.wready  = s_axil_wready;
    assign s_axi.bvalid  = s_axil_bvalid;
    assign s_axi.bresp   = s_axil_bresp;
    assign s_axi.arready = s_axil_arready;
    assign s_axi.rvalid  = s_axil_rvalid;
    assign s_axi.rdata   = s_axil_rdata;
    assign s_axi.rresp   = s_axil_rresp;

endmodule

`resetall

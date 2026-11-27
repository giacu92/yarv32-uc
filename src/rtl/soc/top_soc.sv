`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Board-level top of the LINUX SoC for the Tang Nano 20k (GW2AR-18C).
 *
 * Same core and Linux-branch peripherals as top_module, but the two BSRAMs
 * are replaced by the cache subproject (impl/soc_rtl/, a pinned and
 * package-qualified copy of src/sub/yarv32-native-caches) in front of the
 * GW2AR-18's in-package 8 MiB SDRAM. This file is sim/soc/sim_soc.sv minus
 * the simulation models (SDRAM model, SPI-flash model, UART TX monitor,
 * probe shadow, +ITRACE / +SINIT / +BINIT preload), plus the board clocking
 * and pad logic of top_module.
 *
 *   rv32imac_zicsr_zifencei
 *      | imem (64-bit RO)  dmem (32-bit)            axi_peri
 *      v                   v                         v
 *      |              dport_32to64             axi4_lite_xbar (1->5)
 *      v                   v                         |-> UART 16550 0x1000_0000
 *   cache_cntrl  I-port / D-port                     |-> SPI        0x1000_6000
 *      | 2 KiB boot ROM @ 0x80_0000 (M9 loader)      |-> GPIO       0x1000_7000
 *      | sdram_controller                            |-> CLINT      0x1001_0000
 *      v                                             |-> PLIC       0x1040_0000
 *   O_sdram_* / IO_sdram_dq  (in-package SDRAM die)
 *
 * SDRAM PORT NAMES. The embedded SDRAM is a SIP die, not board wiring: it
 * has NO .cst entries, and the Gowin toolchain binds it by MATCHING THE
 * TOP-LEVEL PORT NAMES. The O_sdram_* / IO_sdram_dq ports below are copied
 * verbatim (names, widths, directions) from the cache repo's board top,
 * src/sub/yarv32-native-caches/src/rtl/fpga_top.sv lines 72-82. They are
 * the documented exception to the *_i/_o convention; renaming one silently
 * disconnects the SDRAM.
 *
 * SPI FLASH (NG-4, open). The M9 loader reads its frame from SPI flash at
 * 2 MiB, then falls back to the UART. Whether the on-board configuration
 * flash can be reached from fabric (MSPI pins as user IO) is not settled by
 * any documentation in this repo, so the SPI master stays on the same free
 * header pins as top_module (27-30, see linux_soc.cst). Boot over UART works
 * either way; flash boot needs an external SPI NOR on those pins until NG-4
 * is closed.
 *
 * Naming: ports *_i/_o (except the SDRAM ports above); internals no prefix;
 * instances u_*.
 */

module top_soc (
    // 25 MHz reference clock from the MS5351M (CLK0, PIN10, LVCMOS33).
    input wire clk_i,
    // Board reset button S1 (PIN88), ACTIVE HIGH on this board.
    input wire rst_i,

    // UART (onboard BL616 USB-UART bridge).
    input  wire uart_rxd_i,
    output wire uart_txd_o,

    // SPI master -> onboard microSD slot in SPI mode (M9b, D42): CLK 83,
    // CMD/MOSI 82, DAT0/MISO 84, DAT3/CS 81 (linux_soc.cst). MISO is
    // double-flopped below, so the loader's fast SD clock must leave >= 4
    // core cycles per SCK half period.
    output wire spi_sck_o,
    output wire spi_mosi_o,
    input  wire spi_miso_i,
    output wire spi_cs_n_o,

    // GPIO (4 pins, push-pull, per-bit tri-state; pulled up in the .cst).
    inout wire [3:0] gpio_io,

    // Debug LEDs: led_o[3:1] alive counter, led_o[0] = GPIO0 when driven.
    output wire [3:0] led_o,

    // GW2AR-18 embedded SDRAM (8 MiB). FIXED NAMES -- see the header.
    output wire        O_sdram_clk,
    output wire        O_sdram_cke,
    output wire        O_sdram_cs_n,
    output wire        O_sdram_cas_n,
    output wire        O_sdram_ras_n,
    output wire        O_sdram_wen_n,
    output wire [ 3:0] O_sdram_dqm,
    output wire [10:0] O_sdram_addr,
    output wire [ 1:0] O_sdram_ba,
    inout  wire [31:0] IO_sdram_dq
);

    // Reset vector: the cache's boot ROM (yarv32_cache_pkg::BOOTROM_BASE).
    localparam logic [31:0] BOOT_ADDR = 32'h0080_0000;
    // Core clock in MHz for the cache's SDRAM refresh spacing and power-up
    // wait. MUST track the rPLL below, the SDC generated clock, global_freq
    // in the PnR wrapper and rv32_pkg::UART_CLK_HZ: a stale value here
    // under-refreshes the SDRAM rather than mis-reporting timing.
    localparam int CLK_FREQ_MHZ = 50;
    // SDRAM power-up hold, as the sim (sim_soc) and the cache default.
    localparam int SDRAM_INIT_US = 200;
    // M9 loader image (sim/sw/bootrom, `make -C sim/sw/bootrom`). Path is
    // relative to the Gowin project dir (repo root), like top_module's
    // INIT_FILE. It is a BUILD PRODUCT (gitignored): impl/remote_pnr_soc.sh
    // ships it next to the git archive.
    localparam string BOOTROM_FILE = "sim/sw/bootrom/build/bootrom.hex";

    // -----------------------------------------------------------------
    // Clock generation -- the rPLL of top_module (25 MHz in, 50 MHz
    // clk_core on CLKOUT: IDIV_SEL=4 / FBDIV_SEL=9 / ODIV_SEL=16, VCO
    // 800 MHz), plus the SDRAM clock on CLKOUTP as the cache repo's
    // fpga_top does it: the same 50 MHz shifted by PSDA_SEL in 1/16-period
    // steps, "1000" = 180 degrees, static (DYN_DA_EN="false"). 180 degrees
    // is the bring-up starting point, not a measured optimum; if SDRAM reads
    // are flaky on the board, sweep SDRAM_PSDA_SEL ("1100" = 270, "0100" =
    // 90) and keep the widest working window.
    // -----------------------------------------------------------------
    localparam string SDRAM_PSDA_SEL = "1000";

    wire clk_core;
    wire sdram_clk;
    wire pll_lock;

`ifdef GOWIN
    rPLL #(  // For GW2AR-LV18QN88C8/I7 (Tang Nano 20K)
        .FCLKIN   ("25"),
        .IDIV_SEL (4),               // -> IDIV = 5,  PFD    =   5 MHz (range 3-400)
        .FBDIV_SEL(9),               // -> FBDIV = 10, CLKOUT = 50 MHz (range 3.125-600)
        .ODIV_SEL (16),              // ->            VCO    = 800 MHz (range 500-1250)
        .PSDA_SEL (SDRAM_PSDA_SEL),  // CLKOUTP phase (SDRAM clock), see above
        .DYN_DA_EN("false")          // static phase shift: PSDA_SEL is the value
    ) pll (
        .CLKOUTP (sdram_clk),  // 50 MHz, phase shifted -> O_sdram_clk
        .CLKOUTD (),
        .CLKOUTD3(),
        .RESET   (1'b0),
        .RESET_P (1'b0),
        .CLKFB   (1'b0),
        .FBDSEL  (6'b0),
        .IDSEL   (6'b0),
        .ODSEL   (6'b0),
        .PSDA    (4'b0),
        .DUTYDA  (4'b0),
        .FDLY    (4'b0),
        .CLKIN   (clk_i),      // 25 MHz reference
        .CLKOUT  (clk_core),   // 50 MHz core clock
        .LOCK    (pll_lock)
    );
`else
    assign clk_core  = clk_i;
    assign sdram_clk = clk_i;
    assign pll_lock  = 1'b1;
`endif

    // -----------------------------------------------------------------
    // Reset synchronization, as top_module: asserted while S1 is held or
    // the PLL is not locked, deassertion synchronized to clk_core.
    //
    // The power-up value is given explicitly (in reset), because the cache
    // repo learned on the board that a reset chain whose own flops wake up
    // released never produces a reset at all (src/sub/yarv32-native-caches
    // CLAUDE.md, "The reset chain needs a defined power-up state"). The
    // PnR wrapper sets -init_primitives 1 so the device honours it.
    // -----------------------------------------------------------------
    logic [1:0] rst_sync = 2'b00;

    always_ff @(posedge clk_core) begin
        if (rst_i) begin
            rst_sync <= 2'b00;
        end else if (!pll_lock) begin
            rst_sync <= 2'b00;
        end else begin
            rst_sync <= {rst_sync[0], 1'b1};
        end
    end

    wire rstn_core = rst_sync[1];

    // -----------------------------------------------------------------
    // AXI4-Lite peripheral buses (Linux-branch map, see rv32_pkg).
    // -----------------------------------------------------------------
    axi4_lite_if axi_bus_peri ();
    axi4_lite_if axi_bus_uart ();
    axi4_lite_if axi_bus_spi ();
    axi4_lite_if axi_bus_gpio ();
    axi4_lite_if axi_bus_clint ();
    axi4_lite_if axi_bus_plic ();

    assign axi_bus_peri.aclk     = clk_core;
    assign axi_bus_peri.aresetn  = rstn_core;
    assign axi_bus_uart.aclk     = clk_core;
    assign axi_bus_uart.aresetn  = rstn_core;
    assign axi_bus_spi.aclk      = clk_core;
    assign axi_bus_spi.aresetn   = rstn_core;
    assign axi_bus_gpio.aclk     = clk_core;
    assign axi_bus_gpio.aresetn  = rstn_core;
    assign axi_bus_clint.aclk    = clk_core;
    assign axi_bus_clint.aresetn = rstn_core;
    assign axi_bus_plic.aclk     = clk_core;
    assign axi_bus_plic.aresetn  = rstn_core;

    // -----------------------------------------------------------------
    // CPU. IMEM_ADDR_W = 32: no fetch fault, because the cache decodes only
    // bits [23:0] and 0x8000_0000 (the RV32 nommu kernel load address) must
    // alias SDRAM 0 -- same as sim_soc.
    // -----------------------------------------------------------------
    ifetch_req_t        imem_req;
    ifetch_rsp_t        imem_rsp;
    mem_req_t           dmem_req;
    mem_rsp_t           dmem_rsp;

    wire                msip;
    wire                mtip;
    wire         [63:0] mtime;
    wire                uart_irq;
    wire                spi_irq;
    wire                gpio_irq;
    wire                meip;
    wire                seip_unused;  // S-mode context, no consumer until M11

    // fence.i: the CPU holds the level request, the cache writes back dirty
    // D-lines, invalidates the I-cache and pulses the ack.
    wire                fencei_req;
    wire                fencei_ack;

    rv32imac_zicsr_zifencei #(
        .IMEM_ADDR_W       (32),
        .BP_EN             (rv32_pkg::BP_EN),
        .MUL_SHARED_DSP    (rv32_pkg::MUL_SHARED_DSP),
        .BP_PUSH_LOOKUP    (rv32_pkg::BP_PUSH_LOOKUP),
        .EXEC_REDIR_INCYCLE(rv32_pkg::EXEC_REDIR_INCYCLE),
        .LSU_LIVE_LOAD     (rv32_pkg::LSU_LIVE_LOAD)
    ) u_cpu (
        .clk_i       (clk_core),
        .rstn_i      (rstn_core),
        .boot_addr_i (BOOT_ADDR),
        .axi_peri    (axi_bus_peri.master),
        .imem_req_o  (imem_req),
        .imem_rsp_i  (imem_rsp),
        .dmem_req_o  (dmem_req),
        .dmem_rsp_i  (dmem_rsp),
        .msip_i      (msip),
        .mtip_i      (mtip),
        .mtime_i     (mtime),
        .meip_i      (meip),
        .fencei_req_o(fencei_req),
        .fencei_ack_i(fencei_ack)
    );

    // -----------------------------------------------------------------
    // Memory system: fetch -> cache I-port; LSU -> dport_32to64 -> cache
    // D-port; cache -> sdram_controller -> in-package SDRAM.
    // -----------------------------------------------------------------
    yarv32_cache_pkg::mem_req_t ic_req, dc_req;
    yarv32_cache_pkg::mem_rsp_t ic_rsp, dc_rsp;

    always_comb begin
        ic_req        = '0;
        ic_req.valid  = imem_req.valid;
        ic_req.addr   = {32'd0, imem_req.addr};
        ic_req.rready = imem_req.rready;
    end
    assign imem_rsp.ready  = ic_rsp.wready;
    assign imem_rsp.rvalid = ic_rsp.rvalid;
    assign imem_rsp.rdata  = ic_rsp.rdata;

    wire        dc_valid;
    wire        dc_we;
    wire        dc_rready;
    wire [31:0] dc_addr;
    wire [63:0] dc_wdata;
    wire [ 7:0] dc_wstrb;

    dport_32to64 u_dport (
        .clk_i     (clk_core),
        .rstn_i    (rstn_core),
        .cpu_req_i (dmem_req),
        .cpu_rsp_o (dmem_rsp),
        .m_valid_o (dc_valid),
        .m_we_o    (dc_we),
        .m_addr_o  (dc_addr),
        .m_wdata_o (dc_wdata),
        .m_wstrb_o (dc_wstrb),
        .m_rready_o(dc_rready),
        .m_wready_i(dc_rsp.wready),
        .m_rvalid_i(dc_rsp.rvalid),
        .m_rdata_i (dc_rsp.rdata),
        .m_bvalid_i(dc_rsp.bvalid)
    );

    always_comb begin
        dc_req        = '0;
        dc_req.valid  = dc_valid;
        dc_req.we     = dc_we;
        dc_req.addr   = {32'd0, dc_addr};
        dc_req.wdata  = dc_wdata;
        dc_req.wstrb  = dc_wstrb;
        dc_req.rready = dc_rready;
    end

    cache_cntrl #(
        .CLK_FREQ_MHZ (CLK_FREQ_MHZ),
        .SDRAM_INIT_US(SDRAM_INIT_US),
        .BOOTROM_FILE (BOOTROM_FILE)
    ) u_cache (
        .clk_i        (clk_core),
        .rstn_i       (rstn_core),
        .sdram_clk_i  (sdram_clk),
        .icache_req_i (ic_req),
        .icache_rsp_o (ic_rsp),
        .dcache_req_i (dc_req),
        .dcache_rsp_o (dc_rsp),
        .fencei_req_i (fencei_req),
        .fencei_ack_o (fencei_ack),
        .sdram_clk_o  (O_sdram_clk),
        .sdram_cke_o  (O_sdram_cke),
        .sdram_cs_n_o (O_sdram_cs_n),
        .sdram_cas_n_o(O_sdram_cas_n),
        .sdram_ras_n_o(O_sdram_ras_n),
        .sdram_wen_n_o(O_sdram_wen_n),
        .sdram_dqm_o  (O_sdram_dqm),
        .sdram_addr_o (O_sdram_addr),
        .sdram_ba_o   (O_sdram_ba),
        .sdram_dq_io  (IO_sdram_dq),
        .dbg_state_o  (),
        .dbg_dport_o  (),
        .dbg_cnt_o    (),
        .dbg_acc_o    (),
        .dbg_go_o     (),
        .dbg_rsp_o    (),
        .dbg_tick_o   (),
        .dbg_hb_o     ()
    );

    // -----------------------------------------------------------------
    // Peripheral crossbar, Linux-branch map (as top_module / sim_soc).
    // -----------------------------------------------------------------
    localparam int unsigned PERI_N = 5;
    localparam int unsigned W_UART = 0;
    localparam int unsigned W_SPI = 1;
    localparam int unsigned W_GPIO = 2;
    localparam int unsigned W_CLINT = 3;
    localparam int unsigned W_PLIC = 4;

    logic [         31:0] peri_awaddr;
    logic [         31:0] peri_wdata;
    logic [          3:0] peri_wstrb;
    logic [         31:0] peri_araddr;
    logic [   PERI_N-1:0] peri_awvalid;
    logic [   PERI_N-1:0] peri_wvalid;
    logic [   PERI_N-1:0] peri_bready;
    logic [   PERI_N-1:0] peri_arvalid;
    logic [   PERI_N-1:0] peri_rready;
    logic [   PERI_N-1:0] peri_awready;
    logic [   PERI_N-1:0] peri_wready;
    logic [   PERI_N-1:0] peri_bvalid;
    logic [   PERI_N-1:0] peri_arready;
    logic [   PERI_N-1:0] peri_rvalid;
    logic [ 2*PERI_N-1:0] peri_bresp;
    logic [ 2*PERI_N-1:0] peri_rresp;
    logic [32*PERI_N-1:0] peri_rdata;

    axi4_lite_xbar #(
        .N(PERI_N),
        .BASES({
            rv32_pkg::PLIC_BASE,
            rv32_pkg::CLINT_BASE,
            rv32_pkg::GPIO_BASE,
            rv32_pkg::SPI_BASE,
            rv32_pkg::UART_BASE
        }),
        .SIZES({
            rv32_pkg::PLIC_SIZE,
            rv32_pkg::CLINT_SIZE,
            rv32_pkg::GPIO_SIZE,
            rv32_pkg::SPI_SIZE,
            rv32_pkg::UART_SIZE
        })
    ) u_peri_xbar (
        .clk_i      (clk_core),
        .rstn_i     (rstn_core),
        .s_axi      (axi_bus_peri.slave),
        .m_awaddr_o (peri_awaddr),
        .m_wdata_o  (peri_wdata),
        .m_wstrb_o  (peri_wstrb),
        .m_araddr_o (peri_araddr),
        .m_awvalid_o(peri_awvalid),
        .m_wvalid_o (peri_wvalid),
        .m_bready_o (peri_bready),
        .m_arvalid_o(peri_arvalid),
        .m_rready_o (peri_rready),
        .m_awready_i(peri_awready),
        .m_wready_i (peri_wready),
        .m_bvalid_i (peri_bvalid),
        .m_arready_i(peri_arready),
        .m_rvalid_i (peri_rvalid),
        .m_bresp_i  (peri_bresp),
        .m_rresp_i  (peri_rresp),
        .m_rdata_i  (peri_rdata)
    );

    assign axi_bus_uart.awaddr        = peri_awaddr;
    assign axi_bus_uart.wdata         = peri_wdata;
    assign axi_bus_uart.wstrb         = peri_wstrb;
    assign axi_bus_uart.araddr        = peri_araddr;
    assign axi_bus_uart.awvalid       = peri_awvalid[W_UART];
    assign axi_bus_uart.wvalid        = peri_wvalid[W_UART];
    assign axi_bus_uart.bready        = peri_bready[W_UART];
    assign axi_bus_uart.arvalid       = peri_arvalid[W_UART];
    assign axi_bus_uart.rready        = peri_rready[W_UART];
    assign peri_awready[W_UART]       = axi_bus_uart.awready;
    assign peri_wready[W_UART]        = axi_bus_uart.wready;
    assign peri_bvalid[W_UART]        = axi_bus_uart.bvalid;
    assign peri_arready[W_UART]       = axi_bus_uart.arready;
    assign peri_rvalid[W_UART]        = axi_bus_uart.rvalid;
    assign peri_bresp[2*W_UART+:2]    = axi_bus_uart.bresp;
    assign peri_rresp[2*W_UART+:2]    = axi_bus_uart.rresp;
    assign peri_rdata[32*W_UART+:32]  = axi_bus_uart.rdata;

    assign axi_bus_spi.awaddr         = peri_awaddr;
    assign axi_bus_spi.wdata          = peri_wdata;
    assign axi_bus_spi.wstrb          = peri_wstrb;
    assign axi_bus_spi.araddr         = peri_araddr;
    assign axi_bus_spi.awvalid        = peri_awvalid[W_SPI];
    assign axi_bus_spi.wvalid         = peri_wvalid[W_SPI];
    assign axi_bus_spi.bready         = peri_bready[W_SPI];
    assign axi_bus_spi.arvalid        = peri_arvalid[W_SPI];
    assign axi_bus_spi.rready         = peri_rready[W_SPI];
    assign peri_awready[W_SPI]        = axi_bus_spi.awready;
    assign peri_wready[W_SPI]         = axi_bus_spi.wready;
    assign peri_bvalid[W_SPI]         = axi_bus_spi.bvalid;
    assign peri_arready[W_SPI]        = axi_bus_spi.arready;
    assign peri_rvalid[W_SPI]         = axi_bus_spi.rvalid;
    assign peri_bresp[2*W_SPI+:2]     = axi_bus_spi.bresp;
    assign peri_rresp[2*W_SPI+:2]     = axi_bus_spi.rresp;
    assign peri_rdata[32*W_SPI+:32]   = axi_bus_spi.rdata;

    assign axi_bus_gpio.awaddr        = peri_awaddr;
    assign axi_bus_gpio.wdata         = peri_wdata;
    assign axi_bus_gpio.wstrb         = peri_wstrb;
    assign axi_bus_gpio.araddr        = peri_araddr;
    assign axi_bus_gpio.awvalid       = peri_awvalid[W_GPIO];
    assign axi_bus_gpio.wvalid        = peri_wvalid[W_GPIO];
    assign axi_bus_gpio.bready        = peri_bready[W_GPIO];
    assign axi_bus_gpio.arvalid       = peri_arvalid[W_GPIO];
    assign axi_bus_gpio.rready        = peri_rready[W_GPIO];
    assign peri_awready[W_GPIO]       = axi_bus_gpio.awready;
    assign peri_wready[W_GPIO]        = axi_bus_gpio.wready;
    assign peri_bvalid[W_GPIO]        = axi_bus_gpio.bvalid;
    assign peri_arready[W_GPIO]       = axi_bus_gpio.arready;
    assign peri_rvalid[W_GPIO]        = axi_bus_gpio.rvalid;
    assign peri_bresp[2*W_GPIO+:2]    = axi_bus_gpio.bresp;
    assign peri_rresp[2*W_GPIO+:2]    = axi_bus_gpio.rresp;
    assign peri_rdata[32*W_GPIO+:32]  = axi_bus_gpio.rdata;

    assign axi_bus_clint.awaddr       = peri_awaddr;
    assign axi_bus_clint.wdata        = peri_wdata;
    assign axi_bus_clint.wstrb        = peri_wstrb;
    assign axi_bus_clint.araddr       = peri_araddr;
    assign axi_bus_clint.awvalid      = peri_awvalid[W_CLINT];
    assign axi_bus_clint.wvalid       = peri_wvalid[W_CLINT];
    assign axi_bus_clint.bready       = peri_bready[W_CLINT];
    assign axi_bus_clint.arvalid      = peri_arvalid[W_CLINT];
    assign axi_bus_clint.rready       = peri_rready[W_CLINT];
    assign peri_awready[W_CLINT]      = axi_bus_clint.awready;
    assign peri_wready[W_CLINT]       = axi_bus_clint.wready;
    assign peri_bvalid[W_CLINT]       = axi_bus_clint.bvalid;
    assign peri_arready[W_CLINT]      = axi_bus_clint.arready;
    assign peri_rvalid[W_CLINT]       = axi_bus_clint.rvalid;
    assign peri_bresp[2*W_CLINT+:2]   = axi_bus_clint.bresp;
    assign peri_rresp[2*W_CLINT+:2]   = axi_bus_clint.rresp;
    assign peri_rdata[32*W_CLINT+:32] = axi_bus_clint.rdata;

    assign axi_bus_plic.awaddr        = peri_awaddr;
    assign axi_bus_plic.wdata         = peri_wdata;
    assign axi_bus_plic.wstrb         = peri_wstrb;
    assign axi_bus_plic.araddr        = peri_araddr;
    assign axi_bus_plic.awvalid       = peri_awvalid[W_PLIC];
    assign axi_bus_plic.wvalid        = peri_wvalid[W_PLIC];
    assign axi_bus_plic.bready        = peri_bready[W_PLIC];
    assign axi_bus_plic.arvalid       = peri_arvalid[W_PLIC];
    assign axi_bus_plic.rready        = peri_rready[W_PLIC];
    assign peri_awready[W_PLIC]       = axi_bus_plic.awready;
    assign peri_wready[W_PLIC]        = axi_bus_plic.wready;
    assign peri_bvalid[W_PLIC]        = axi_bus_plic.bvalid;
    assign peri_arready[W_PLIC]       = axi_bus_plic.arready;
    assign peri_rvalid[W_PLIC]        = axi_bus_plic.rvalid;
    assign peri_bresp[2*W_PLIC+:2]    = axi_bus_plic.bresp;
    assign peri_rresp[2*W_PLIC+:2]    = axi_bus_plic.rresp;
    assign peri_rdata[32*W_PLIC+:32]  = axi_bus_plic.rdata;

    // -----------------------------------------------------------------
    // SPI master. MISO double-flopped (asynchronous pin).
    // -----------------------------------------------------------------
    logic [1:0] spi_miso_sync_q;

    always_ff @(posedge clk_core) begin
        if (!rstn_core) begin
            spi_miso_sync_q <= 2'b00;
        end else begin
            spi_miso_sync_q <= {spi_miso_sync_q[0], spi_miso_i};
        end
    end

    axi4_lite_spi u_spi (
        .clk_i     (clk_core),
        .rstn_i    (rstn_core),
        .axi       (axi_bus_spi.slave),
        .spi_sck_o (spi_sck_o),
        .spi_mosi_o(spi_mosi_o),
        .spi_miso_i(spi_miso_sync_q[1]),
        .spi_cs_n_o(spi_cs_n_o),
        .spi_irq_o (spi_irq)
    );

    // -----------------------------------------------------------------
    // GPIO, 4 push-pull pins. Per-bit tri-state (a vector `oe ? out : 'bz`
    // reduces the oe and one enabled pin would drive all four); inputs
    // double-flopped, sync chain resets high (pulled-up pads).
    // -----------------------------------------------------------------
    wire [3:0] gpio_out;
    wire [3:0] gpio_oe;

    assign gpio_io[0] = gpio_oe[0] ? gpio_out[0] : 1'bz;
    assign gpio_io[1] = gpio_oe[1] ? gpio_out[1] : 1'bz;
    assign gpio_io[2] = gpio_oe[2] ? gpio_out[2] : 1'bz;
    assign gpio_io[3] = gpio_oe[3] ? gpio_out[3] : 1'bz;

    logic [3:0] gpio_sync0_q;
    logic [3:0] gpio_sync1_q;

    always_ff @(posedge clk_core) begin
        if (!rstn_core) begin
            gpio_sync0_q <= 4'hF;
            gpio_sync1_q <= 4'hF;
        end else begin
            gpio_sync0_q <= gpio_io;
            gpio_sync1_q <= gpio_sync0_q;
        end
    end

    axi4_lite_gpio #(
        .WIDTH(4)
    ) u_gpio (
        .clk_i     (clk_core),
        .rstn_i    (rstn_core),
        .axi       (axi_bus_gpio.slave),
        .gpio_i    (gpio_sync1_q),
        .gpio_o    (gpio_out),
        .gpio_oe_o (gpio_oe),
        .gpio_irq_o(gpio_irq)
    );

    // -----------------------------------------------------------------
    // CLINT (SiFive layout).
    // -----------------------------------------------------------------
    clint u_clint (
        .clk_i  (clk_core),
        .rstn_i (rstn_core),
        .axi    (axi_bus_clint.slave),
        .msip_o (msip),
        .mtip_o (mtip),
        .mtime_o(mtime)
    );

    // -----------------------------------------------------------------
    // PLIC (sifive,plic-1.0.0). Source IDs by NAME from rv32_pkg.
    // -----------------------------------------------------------------
    logic [PLIC_SRC_N-1:0] plic_irq;
    always_comb begin
        plic_irq                = '0;
        plic_irq[PLIC_SRC_UART] = uart_irq;
        plic_irq[PLIC_SRC_SPI]  = spi_irq;
        plic_irq[PLIC_SRC_GPIO] = gpio_irq;
    end

    axi4_lite_plic_sifive u_plic (
        .clk_i (clk_core),
        .rstn_i(rstn_core),
        .axi   (axi_bus_plic.slave),
        .irq_i (plic_irq),
        .meip_o(meip),
        .seip_o(seip_unused)
    );

    // -----------------------------------------------------------------
    // UART 16550. uart_rxd_i double-flopped (asynchronous far end).
    // -----------------------------------------------------------------
    logic [1:0] uart_rxd_sync_q;

    always_ff @(posedge clk_core) begin
        if (!rstn_core) begin
            uart_rxd_sync_q <= 2'b11;  // idle line is high
        end else begin
            uart_rxd_sync_q <= {uart_rxd_sync_q[0], uart_rxd_i};
        end
    end

    axi4_lite_uart16550 #(
        .CLK_FREQ_HZ(rv32_pkg::UART_CLK_HZ),
        .BAUD_RATE  (rv32_pkg::UART_BAUD)
    ) u_uart (
        .clk_i     (clk_core),
        .rstn_i    (rstn_core),
        .axi       (axi_bus_uart.slave),
        .txd_o     (uart_txd_o),
        .rxd_i     (uart_rxd_sync_q[1]),
        .uart_irq_o(uart_irq)
    );

    // -----------------------------------------------------------------
    // Debug LEDs, as top_module: [3:1] alive counter, [0] GPIO0 when it
    // is driven (dark otherwise).
    // -----------------------------------------------------------------
    logic [27:0] led_cnt_q;

    always_ff @(posedge clk_core) begin
        if (!rstn_core) begin
            led_cnt_q <= '0;
        end else begin
            led_cnt_q <= led_cnt_q + 1'b1;
        end
    end

    assign led_o[3:1] = led_cnt_q[27:25];
    assign led_o[0]   = gpio_oe[0] ? gpio_out[0] : 1'b0;

endmodule

`resetall

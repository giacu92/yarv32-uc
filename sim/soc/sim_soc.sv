`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Simulation top (Verilator). Mirrors top_module's peri bus wiring:
 *
 *   CPU.axi_peri -> axi_bus_peri -> axi4_lite_xbar (1->5, base+size)
 *                    |-> u_uart  (0x1000_0000, 16550)
 *                    |-> u_spi   (0x1000_6000)
 *                    |-> u_gpio  (0x1000_7000)
 *                    |-> u_clint (0x1001_0000, 64 KiB, SiFive layout)
 *                    |-> u_plic  (0x1040_0000, 4 MiB, sifive,plic-1.0.0)
 * Linux-branch map (linux-soc, M8). Fetch and the LSU each have a dedicated
 * BSRAM (Harvard).
 *
 * No debug signals cross the CPU boundary (the CPU exports only its
 * functional ports). The C++ harness observes the per-stage taps
 * (fe_* / de_* / ex_* / writeback) by probing the Verilator hierarchy
 * directly — the sim is built with --public-flat, so the CPU-internal
 * nets (fe_pc_w, de_pc_w, ex_pc_w, wb_en, wb_addr, wb_data, ...) are
 * reachable as flat C++ members of the sim_top model. sim_top itself
 * therefore needs no debug output ports.
 *
 * This module is simulation-only; it is not part of the synthesis
 * file list.
 *
 * Naming follows the project convention: ports *_i/_o, internal
 * signals (incl. interface instances) have no prefix.
 */
module sim_top #(
    // Linux SoC (M5): reset vector = bootrom, SDRAM power-up wait.
    parameter logic        [31:0] BOOT_ADDR          = 32'h0080_0000,
    parameter int                 SDRAM_INIT_US      = 200,
    // UART clock/baud, overridable from the Verilator command line
    // (-GUART_CLK_HZ=... -GUART_BAUD=...). Deliberately NOT taken from
    // rv32_pkg, unlike the four structural knobs below: the package carries
    // the BOARD baud (115200) and the sim wants a fast one.
    // The defaults keep the sim fast
    // (5 clocks per bit); pass the board's real numbers
    // (25_000_000 / 115_200 -> 217 clocks per bit) to check the RX
    // sampling phase at the divisor the hardware actually uses.
    parameter int unsigned        UART_CLK_HZ        = 50_000_000,
    // 16550 semantics (axi4_lite_uart16550): one bit = 16 x DL clocks, so
    // the fastest the sim can run is DL = 1 = 16 clocks per bit, i.e.
    // 50 MHz / 16 = 3.125 MBaud. sim_main.cpp's UART_BIT_CYCLES default
    // matches (16).
    parameter int unsigned        UART_BAUD          = 3_125_000,
    // Branch-predictor enable A/B knob: -GBP_EN=0 disables prediction and
    // reproduces the pre-predictor core exactly (the baseline). Default 1.
    parameter int                 BP_EN              = rv32_pkg::BP_EN,
    // MUL structure A/B knob: -GMUL_SHARED_DSP=0 restores the three-product
    // form (see alu.sv). Functionally identical, so this is a timing knob
    // only -- the retire stream must not move.
    parameter int                 MUL_SHARED_DSP     = rv32_pkg::MUL_SHARED_DSP,
    // PHT lookup placement A/B knob: -GBP_PUSH_LOOKUP=0 reads the PHT at
    // decode with the live GHR (the original form); 1 reads it at
    // instruction-buffer push time and carries the bit in the entry. Unlike
    // MUL_SHARED_DSP this one DOES move the retire stream -- the push-time
    // read sees a slightly older GHR, so predictions differ.
    parameter int                 BP_PUSH_LOOKUP     = rv32_pkg::BP_PUSH_LOOKUP,
    // -GEXEC_REDIR_INCYCLE=1 restores the 2026-08-31 form where an execute
    // redirect also launches its read in the redirect cycle. Default 0 keeps
    // the register file off the I-mem address pins (see fetch_stage.sv) and
    // costs 1 cycle per mispredict / trap / mret.
    parameter int                 EXEC_REDIR_INCYCLE = rv32_pkg::EXEC_REDIR_INCYCLE,
    // -GLSU_LIVE_LOAD=0 captures every bus op (the pre-2026-09-01 LSU).
    parameter int                 LSU_LIVE_LOAD      = rv32_pkg::LSU_LIVE_LOAD
) (
    input  wire       clk_i,
    input  wire       rstn_i,
    // UART RX serial line, driven by the C++ harness (idle high). Lets a
    // serial-console program (YarvMon) be fed real bit-level input; the
    // harness paces one frame at a time off u_uart's rx_ready_q so it
    // cannot overrun the single-byte RX buffer. Tie high for no input.
    input  wire       uart_rxd_i,
    output wire [3:0] led_o
);

    // -----------------------------------------------------------------
    // AXI4-Lite peripheral bus (trunk modport) + peri 1->7 split. Window
    // bases/sizes come from rv32_pkg, same as the board top.
    // -----------------------------------------------------------------
    axi4_lite_if axi_bus_peri ();
    axi4_lite_if axi_bus_uart ();
    axi4_lite_if axi_bus_spi ();
    axi4_lite_if axi_bus_gpio ();
    axi4_lite_if axi_bus_clint ();
    axi4_lite_if axi_bus_plic ();

    assign axi_bus_peri.aclk     = clk_i;
    assign axi_bus_peri.aresetn  = rstn_i;
    assign axi_bus_uart.aclk     = clk_i;
    assign axi_bus_uart.aresetn  = rstn_i;
    assign axi_bus_spi.aclk      = clk_i;
    assign axi_bus_spi.aresetn   = rstn_i;
    assign axi_bus_gpio.aclk     = clk_i;
    assign axi_bus_gpio.aresetn  = rstn_i;
    assign axi_bus_clint.aclk    = clk_i;
    assign axi_bus_clint.aresetn = rstn_i;
    assign axi_bus_plic.aclk     = clk_i;
    assign axi_bus_plic.aresetn  = rstn_i;

    // -----------------------------------------------------------------
    // LINUX-SoC MEMORY SYSTEM (M5). Unified address space, no BSRAM I/D-mem:
    //   fetch (64-bit, 2 outstanding) -> cache_cntrl I-port
    //   LSU   (32-bit)  -> dport_32to64 -> cache_cntrl D-port
    //   cache_cntrl -> sdram_controller -> sdram_model (8 MiB, 32-bit)
    // Map (yarv32_cache_pkg, 24 bits decoded): 0x00_0000-0x7F_FFFF SDRAM
    // (cached), 0x80_0000 2 KiB bootrom, 0x80_1000 cache control register.
    // Peripherals stay on addr[28] and never reach the cache.
    // -----------------------------------------------------------------
    ifetch_req_t        imem_req;
    ifetch_rsp_t        imem_rsp;
    mem_req_t           dmem_req;
    mem_rsp_t           dmem_rsp;

    // Interrupt pending bits (mip.MSIP / mip.MTIP sources).
    wire                msip;
    wire                mtip;
    wire         [63:0] mtime;
    // Machine external interrupt: the PLIC aggregates the peripheral
    // level IRQs (UART / I2C / SPI / GPIO), same as the board top. The
    // source IRQs reset deasserted, so they never raise meip unless a
    // test enables them through MMIO.
    wire                uart_irq;
    wire                spi_irq;
    wire                gpio_irq;
    wire                meip;

    // -----------------------------------------------------------------
    // CPU. Boots from the bootrom. IMEM_ADDR_W = 32: no fetch fault at all,
    // because the cache decodes only bits [23:0] and 0x8000_0000 -- the
    // standard RV32 nommu kernel load address -- must alias SDRAM 0.
    // -----------------------------------------------------------------
    // fence.i: CPU holds the level request, the cache writes back dirty
    // D-lines, invalidates the I-cache and pulses the ack.
    wire fencei_req, fencei_ack;

    rv32imac_zicsr_zifencei #(
        .IMEM_ADDR_W(32),  // no fetch fault: 0x8000_0000 aliases SDRAM (Linux load address)
        .BP_EN(BP_EN),
        .MUL_SHARED_DSP(MUL_SHARED_DSP),
        .BP_PUSH_LOOKUP(BP_PUSH_LOOKUP),
        .EXEC_REDIR_INCYCLE(EXEC_REDIR_INCYCLE),
        .LSU_LIVE_LOAD(LSU_LIVE_LOAD)
    ) u_cpu (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
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

    wire dc_valid, dc_we, dc_rready;
    wire [31:0] dc_addr;
    wire [63:0] dc_wdata;
    wire [ 7:0] dc_wstrb;

    dport_32to64 u_dport (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
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

    wire sd_clk, sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n;
    wire [ 3:0] sd_dqm;
    wire [10:0] sd_addr;
    wire [ 1:0] sd_ba;
    wire [31:0] sd_dq;

    cache_cntrl #(
        .CLK_FREQ_MHZ (50),
        .SDRAM_INIT_US(SDRAM_INIT_US),
        .BOOTROM_FILE ("")
    ) u_cache (
        .clk_i        (clk_i),
        .rstn_i       (rstn_i),
        .sdram_clk_i  (clk_i),
        .icache_req_i (ic_req),
        .icache_rsp_o (ic_rsp),
        .dcache_req_i (dc_req),
        .dcache_rsp_o (dc_rsp),
        .fencei_req_i (fencei_req),
        .fencei_ack_o (fencei_ack),
        .sdram_clk_o  (sd_clk),
        .sdram_cke_o  (sd_cke),
        .sdram_cs_n_o (sd_cs_n),
        .sdram_cas_n_o(sd_cas_n),
        .sdram_ras_n_o(sd_ras_n),
        .sdram_wen_n_o(sd_we_n),
        .sdram_dqm_o  (sd_dqm),
        .sdram_addr_o (sd_addr),
        .sdram_ba_o   (sd_ba),
        .sdram_dq_io  (sd_dq),
        .dbg_state_o  (),
        .dbg_dport_o  (),
        .dbg_cnt_o    (),
        .dbg_acc_o    (),
        .dbg_go_o     (),
        .dbg_rsp_o    (),
        .dbg_tick_o   (),
        .dbg_hb_o     ()
    );

    // +SDRAM_CLK180: clock the SDRAM model on the INVERTED clock, which is
    // what the board does (top_soc.sv: CLKOUTP, PSDA_SEL "1000" = 180 deg).
    // The device then samples commands and drives read data half a cycle
    // earlier than in the default in-phase sim. The default sim cannot see
    // a read-capture problem that depends on that phase; this switch can
    // (2026-11-27: the board read every word one burst slot late).
    logic sdram_clk180;
    initial sdram_clk180 = $test$plusargs("SDRAM_CLK180");
    wire sd_clk_dev = sdram_clk180 ? ~sd_clk : sd_clk;

    sdram_model u_sdram (
        .clk  (sd_clk_dev),
        .cke  (sd_cke),
        .cs_n (sd_cs_n),
        .ras_n(sd_ras_n),
        .cas_n(sd_cas_n),
        .we_n (sd_we_n),
        .dqm  (sd_dqm),
        .addr (sd_addr),
        .ba   (sd_ba),
        .dq   (sd_dq)
    );

    logic [31:0] probe_shadow[4096];  // see the D-side shadow below

    // Preload: +SINIT=<sdram.hex> (32-bit words, word index = byte/4, made
    // by soc/mkimage.py from a program's imem.hex + dmem.hex) and
    // +BINIT=<bootrom.hex> (64-bit words). Loaded after the model's own
    // time-0 pattern fill.
    // Done on the first clock edge rather than in an initial block: the
    // model's own initial fill would otherwise race this one, and this build
    // (--cc, no --timing) has no #delay to order them.
    string sinit_file, binit_file;
    logic preloaded = 1'b0;
    always @(posedge clk_i) begin
        if (!preloaded) begin
            if ($value$plusargs("SINIT=%s", sinit_file)) begin
                $readmemh(sinit_file, u_sdram.mem);
                // +NOPROBE: an image larger than the 16 KiB shadow (a kernel)
                // would abort $readmemh; PROBE is meaningless for it anyway.
                if (!$test$plusargs("NOPROBE")) $readmemh(sinit_file, probe_shadow);
            end
            if ($value$plusargs("BINIT=%s", binit_file))
                $readmemh(binit_file, u_cache.u_bootrom.mem);
            preloaded <= 1'b1;
        end
    end

    // +ITRACE: log every I-port and D-port accept / response (debug aid).
    bit itrace;
    initial itrace = $test$plusargs("ITRACE");
    always @(posedge clk_i) begin
        if (itrace) begin
            if (imem_req.valid && imem_rsp.ready) $display("%0t I-acc  %08x", $time, imem_req.addr);
            if (imem_rsp.rvalid && imem_req.rready)
                $display("%0t I-rsp  %016x", $time, imem_rsp.rdata);
            if (dmem_req.wvalid && dmem_rsp.wready)
                $display(
                    "%0t D-acc  %08x we=%0d wd=%08x st=%x",
                    $time,
                    dmem_req.addr,
                    dmem_req.we,
                    dmem_req.wdata,
                    dmem_req.wstrb
                );
            if (dmem_rsp.rvalid && dmem_req.rready)
                $display("%0t D-rsp  %08x", $time, dmem_rsp.rdata);
        end
    end

    // -----------------------------------------------------------------
    // Architectural D-side shadow, for PROBE (sim_main): every store the
    // CPU's LSU launches to the low 16 KiB is mirrored here, so a marker
    // can be read without caring whether it is still dirty in the D-cache.
    // Initialised from the same image as the SDRAM (preload block above).
    // -----------------------------------------------------------------
    always_ff @(posedge clk_i) begin
        if (dmem_req.wvalid && dmem_rsp.wready && dmem_req.we && !dmem_req.addr[PERI_ADDR_BIT] &&
            (dmem_req.addr[31:14] == '0)) begin
            for (int b = 0; b < 4; b++)
            if (dmem_req.wstrb[b])
                probe_shadow[dmem_req.addr[13:2]][8*b+:8] <= dmem_req.wdata[8*b+:8];
        end
    end

    // -----------------------------------------------------------------
    // Peripheral bus, LINUX-BRANCH MAP (linux-soc M8, see rv32_pkg): five
    // windows, each at the offsets a mainline driver expects. Mirrors
    // top_module. The xbar's target side is flat vectors (see
    // axi4_lite_xbar.sv), glued to each slave's interface with assigns.
    //   window 0 -> axi_bus_uart  (UART_BASE  0x1000_0000,  4 KiB, ns16550a)
    //   window 1 -> axi_bus_spi   (SPI_BASE   0x1000_6000,  4 KiB)
    //   window 2 -> axi_bus_gpio  (GPIO_BASE  0x1000_7000,  4 KiB)
    //   window 3 -> axi_bus_clint (CLINT_BASE 0x1001_0000, 64 KiB, sifive,clint0)
    //   window 4 -> axi_bus_plic  (PLIC_BASE  0x1040_0000,  4 MiB, sifive,plic-1.0.0)
    // FFT, I2S and I2C are main-only and not instantiated here.
    // -----------------------------------------------------------------
    localparam int unsigned PERI_N = 5;

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
        .N    (PERI_N),
        .BASES({PLIC_BASE, CLINT_BASE, GPIO_BASE, SPI_BASE, UART_BASE}),
        .SIZES({PLIC_SIZE, CLINT_SIZE, GPIO_SIZE, SPI_SIZE, UART_SIZE})
    ) u_peri_xbar (
        .clk_i      (clk_i),
        .rstn_i     (rstn_i),
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

    // Window 0: UART (16550).
    assign axi_bus_uart.awaddr  = peri_awaddr;
    assign axi_bus_uart.wdata   = peri_wdata;
    assign axi_bus_uart.wstrb   = peri_wstrb;
    assign axi_bus_uart.araddr  = peri_araddr;
    assign axi_bus_uart.awvalid = peri_awvalid[0];
    assign axi_bus_uart.wvalid  = peri_wvalid[0];
    assign axi_bus_uart.bready  = peri_bready[0];
    assign axi_bus_uart.arvalid = peri_arvalid[0];
    assign axi_bus_uart.rready  = peri_rready[0];
    assign peri_awready[0]      = axi_bus_uart.awready;
    assign peri_wready[0]       = axi_bus_uart.wready;
    assign peri_bvalid[0]       = axi_bus_uart.bvalid;
    assign peri_arready[0]      = axi_bus_uart.arready;
    assign peri_rvalid[0]       = axi_bus_uart.rvalid;
    assign peri_bresp[0+:2]     = axi_bus_uart.bresp;
    assign peri_rresp[0+:2]     = axi_bus_uart.rresp;
    assign peri_rdata[0+:32]    = axi_bus_uart.rdata;

    // Window 1: SPI. Full-duplex loopback (MISO = MOSI).
    assign axi_bus_spi.awaddr   = peri_awaddr;
    assign axi_bus_spi.wdata    = peri_wdata;
    assign axi_bus_spi.wstrb    = peri_wstrb;
    assign axi_bus_spi.araddr   = peri_araddr;
    assign axi_bus_spi.awvalid  = peri_awvalid[1];
    assign axi_bus_spi.wvalid   = peri_wvalid[1];
    assign axi_bus_spi.bready   = peri_bready[1];
    assign axi_bus_spi.arvalid  = peri_arvalid[1];
    assign axi_bus_spi.rready   = peri_rready[1];
    assign peri_awready[1]      = axi_bus_spi.awready;
    assign peri_wready[1]       = axi_bus_spi.wready;
    assign peri_bvalid[1]       = axi_bus_spi.bvalid;
    assign peri_arready[1]      = axi_bus_spi.arready;
    assign peri_rvalid[1]       = axi_bus_spi.rvalid;
    assign peri_bresp[2+:2]     = axi_bus_spi.bresp;
    assign peri_rresp[2+:2]     = axi_bus_spi.rresp;
    assign peri_rdata[32+:32]   = axi_bus_spi.rdata;

    wire spi_sck, spi_mosi, spi_cs_n, spi_miso, flash_miso, sd_miso;
    // MISO source: +SDCARD -> SD card model in SPI mode (M9b, +SDINIT=<hex>);
    // +SPIFLASH -> serial NOR flash model (M9, +FINIT=<hex>); otherwise the
    // loopback spi_loop expects (MISO = MOSI).
    bit spi_flash_en, sd_en;
    initial begin
        spi_flash_en = $test$plusargs("SPIFLASH");
        sd_en        = $test$plusargs("SDCARD");
    end
    wire spi_miso_pin = sd_en ? sd_miso : spi_flash_en ? flash_miso : spi_mosi;
    // Board parity: top_soc double-flops the asynchronous MISO pin before the
    // SPI master, so every source here sees the same 2-cycle latency.
    logic [1:0] spi_miso_sync_q = 2'b11;
    always_ff @(posedge clk_i) begin
        if (!rstn_i) spi_miso_sync_q <= 2'b11;
        else spi_miso_sync_q <= {spi_miso_sync_q[0], spi_miso_pin};
    end
    assign spi_miso = spi_miso_sync_q[1];
    spi_flash_model u_flash (
        .sck (spi_sck),
        .cs_n(spi_cs_n),
        .mosi(spi_mosi),
        .miso(flash_miso)
    );
    sd_spi_model u_sd (
        .clk (clk_i),
        .sck (spi_sck),
        .cs_n(spi_cs_n),
        .mosi(spi_mosi),
        .miso(sd_miso)
    );

    axi4_lite_spi u_spi (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi_bus_spi.slave),
        .spi_sck_o (spi_sck),
        .spi_mosi_o(spi_mosi),
        .spi_miso_i(spi_miso),           // synchronised: loopback, flash or SD model
        .spi_cs_n_o(spi_cs_n),
        .spi_irq_o (spi_irq)
    );

    // Window 2: GPIO, pins looped back with pull-up semantics (board parity).
    assign axi_bus_gpio.awaddr  = peri_awaddr;
    assign axi_bus_gpio.wdata   = peri_wdata;
    assign axi_bus_gpio.wstrb   = peri_wstrb;
    assign axi_bus_gpio.araddr  = peri_araddr;
    assign axi_bus_gpio.awvalid = peri_awvalid[2];
    assign axi_bus_gpio.wvalid  = peri_wvalid[2];
    assign axi_bus_gpio.bready  = peri_bready[2];
    assign axi_bus_gpio.arvalid = peri_arvalid[2];
    assign axi_bus_gpio.rready  = peri_rready[2];
    assign peri_awready[2]      = axi_bus_gpio.awready;
    assign peri_wready[2]       = axi_bus_gpio.wready;
    assign peri_bvalid[2]       = axi_bus_gpio.bvalid;
    assign peri_arready[2]      = axi_bus_gpio.arready;
    assign peri_rvalid[2]       = axi_bus_gpio.rvalid;
    assign peri_bresp[4+:2]     = axi_bus_gpio.bresp;
    assign peri_rresp[4+:2]     = axi_bus_gpio.rresp;
    assign peri_rdata[64+:32]   = axi_bus_gpio.rdata;

    logic [3:0] gpio_out, gpio_oe;
    wire [3:0] gpio_pin = (gpio_oe & gpio_out) | ~gpio_oe;
    logic [3:0] gpio_sync0_q, gpio_sync1_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            gpio_sync0_q <= 4'hF;
            gpio_sync1_q <= 4'hF;
        end else begin
            gpio_sync0_q <= gpio_pin;
            gpio_sync1_q <= gpio_sync0_q;
        end
    end

    axi4_lite_gpio #(
        .WIDTH(4)
    ) u_gpio (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi_bus_gpio.slave),
        .gpio_i    (gpio_sync1_q),
        .gpio_o    (gpio_out),
        .gpio_oe_o (gpio_oe),
        .gpio_irq_o(gpio_irq)
    );

    // Window 3: CLINT (SiFive layout). msip / mtip straight into the CPU,
    // mtime into the time CSR.
    assign axi_bus_clint.awaddr  = peri_awaddr;
    assign axi_bus_clint.wdata   = peri_wdata;
    assign axi_bus_clint.wstrb   = peri_wstrb;
    assign axi_bus_clint.araddr  = peri_araddr;
    assign axi_bus_clint.awvalid = peri_awvalid[3];
    assign axi_bus_clint.wvalid  = peri_wvalid[3];
    assign axi_bus_clint.bready  = peri_bready[3];
    assign axi_bus_clint.arvalid = peri_arvalid[3];
    assign axi_bus_clint.rready  = peri_rready[3];
    assign peri_awready[3]       = axi_bus_clint.awready;
    assign peri_wready[3]        = axi_bus_clint.wready;
    assign peri_bvalid[3]        = axi_bus_clint.bvalid;
    assign peri_arready[3]       = axi_bus_clint.arready;
    assign peri_rvalid[3]        = axi_bus_clint.rvalid;
    assign peri_bresp[6+:2]      = axi_bus_clint.bresp;
    assign peri_rresp[6+:2]      = axi_bus_clint.rresp;
    assign peri_rdata[96+:32]    = axi_bus_clint.rdata;

    clint u_clint (
        .clk_i  (clk_i),
        .rstn_i (rstn_i),
        .axi    (axi_bus_clint.slave),
        .msip_o (msip),
        .mtip_o (mtip),
        .mtime_o(mtime)
    );

    // Window 4: PLIC (sifive,plic-1.0.0). Source IDs by NAME (rv32_pkg).
    assign axi_bus_plic.awaddr  = peri_awaddr;
    assign axi_bus_plic.wdata   = peri_wdata;
    assign axi_bus_plic.wstrb   = peri_wstrb;
    assign axi_bus_plic.araddr  = peri_araddr;
    assign axi_bus_plic.awvalid = peri_awvalid[4];
    assign axi_bus_plic.wvalid  = peri_wvalid[4];
    assign axi_bus_plic.bready  = peri_bready[4];
    assign axi_bus_plic.arvalid = peri_arvalid[4];
    assign axi_bus_plic.rready  = peri_rready[4];
    assign peri_awready[4]      = axi_bus_plic.awready;
    assign peri_wready[4]       = axi_bus_plic.wready;
    assign peri_bvalid[4]       = axi_bus_plic.bvalid;
    assign peri_arready[4]      = axi_bus_plic.arready;
    assign peri_rvalid[4]       = axi_bus_plic.rvalid;
    assign peri_bresp[8+:2]     = axi_bus_plic.bresp;
    assign peri_rresp[8+:2]     = axi_bus_plic.rresp;
    assign peri_rdata[128+:32]  = axi_bus_plic.rdata;

    logic [PLIC_SRC_N-1:0] plic_irq;
    always_comb begin
        plic_irq                = '0;
        plic_irq[PLIC_SRC_UART] = uart_irq;
        plic_irq[PLIC_SRC_SPI]  = spi_irq;
        plic_irq[PLIC_SRC_GPIO] = gpio_irq;
    end

    wire seip_unused;  // S-mode context, no consumer until M11

    axi4_lite_plic_sifive u_plic (
        .clk_i (clk_i),
        .rstn_i(rstn_i),
        .axi   (axi_bus_plic.slave),
        .irq_i (plic_irq),
        .meip_o(meip),
        .seip_o(seip_unused)
    );

    // Double-flop the harness RX line before the UART, exactly as
    // top_module does for the board pin.
    logic [1:0] uart_rxd_sync_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            uart_rxd_sync_q <= 2'b11;  // idle line is high
        end else begin
            uart_rxd_sync_q <= {uart_rxd_sync_q[0], uart_rxd_i};
        end
    end

    wire uart_txd;

    axi4_lite_uart16550 #(
        .CLK_FREQ_HZ(UART_CLK_HZ),
        .BAUD_RATE  (UART_BAUD)
    ) u_uart (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi_bus_uart.slave),
        .txd_o     (uart_txd),
        .rxd_i     (uart_rxd_sync_q[1]),
        .uart_irq_o(uart_irq)
    );

    // -----------------------------------------------------------------
    // UART TX character monitor (sim only). Logs every byte the CPU
    // actually pushes into the TX shift buffer (u_uart.tx_push), so a
    // serial-console program such as YarvMon is observable without
    // decoding txd_o at the bit level. Dropped pushes (write while TX
    // busy) do not appear here -- by construction, since tx_push is
    // already gated on TX_READY, which is exactly the byte stream the
    // pin will carry.
    // -----------------------------------------------------------------
`ifdef VERILATOR
    int     uart_log_fd;
    // Event log with cycle stamps, for `make linux` (linux.py): one line per
    // TX byte ("U <cycle> <hex byte>") and one per misaligned-access trap
    // ("M <cycle> <mtval>"). The misaligned line answers whether the kernel's
    // misaligned-access emulation ever ran, which QEMU never exercised.
    int     evt_log_fd;
    longint sim_cyc_q;
    logic   mis_q;

    initial begin
        uart_log_fd = $fopen("sim_uart_tx.txt", "w");
        evt_log_fd  = $fopen("sim_events.txt", "w");
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            sim_cyc_q <= 0;
            mis_q     <= 1'b0;
        end else begin
            sim_cyc_q <= sim_cyc_q + 1;
            mis_q     <= u_cpu.u_execute.misaligned_trap;
            if (u_cpu.u_execute.misaligned_trap && !mis_q) begin
                $fwrite(evt_log_fd, "M %0d %08x\n", sim_cyc_q, u_cpu.u_execute.mem_addr_q);
                $fflush(evt_log_fd);
            end
        end
        if (rstn_i && u_uart.tx_push) begin
            $fwrite(uart_log_fd, "%c", u_uart.wdata_eff[7:0]);
            $fflush(uart_log_fd);
            $fwrite(evt_log_fd, "U %0d %02x\n", sim_cyc_q, u_uart.wdata_eff[7:0]);
            $fflush(evt_log_fd);
        end
    end
`endif

    // -----------------------------------------------------------------
    // LED: free-running counter, parity with the board top.
    // -----------------------------------------------------------------
    logic [27:0] led_cnt_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            led_cnt_q <= '0;
        end else begin
            led_cnt_q <= led_cnt_q + 1'b1;
        end
    end

    assign led_o = led_cnt_q[27:24];

endmodule

`resetall

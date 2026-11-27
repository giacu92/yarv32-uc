`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Board-level top for the Tang Nano 20k (Gowin GW2AR-18C).
 *
 * Instantiates the RV32IMAC + Zicsr + Zifencei CPU and wires its memory
 * ports to on-die BSRAM. Harvard topology:
 *
 *   rv32imac_zicsr_zifencei
 *      |  imem (native, RO)   dmem (native, byte-strobed)   axi_peri
 *      v                      v                              v
 *   native_ram (u_imem)   native_ram (u_dmem)         axi_bus_peri
 *   (instr)                (data + .rodata + stack)      |
 *                                                        +-- axi4_lite_xbar
 *                                                        |     (peri 1->9,
 *                                                        |      base+size)
 *                                                        |
 *                            0x1000_0000..0FFF ----------+--> uart_i  (UART)
 *                            0x1000_1000..2FFF ----------+--> u_timer (CLINT)
 *                            0x1000_3000..3FFF ----------+--> u_msip  (MSIP)
 *                            0x1000_5000..5FFF ----------+--> u_i2c   (I2C)
 *                            0x1000_6000..6FFF ----------+--> u_spi   (SPI)
 *                            0x1000_7000..7FFF ----------+--> u_gpio  (GPIO)
 *                            0x1000_8000..8FFF ----------+--> u_plic  (PLIC)
 *                            0x1000_A000..BFFF ----------+--> u_fft   (FFT)
 *                            0x1000_C000..CFFF ----------+--> u_i2s   (I2S RX)
 *
 *   Fetch and the LSU no longer contend: each has a dedicated native
 *   BSRAM port. AXI survives only for peripherals (the peri bridge is
 *   inside the CPU). The board top is pure point-to-point wires — the
 *   LSU steers addr[PERI_ADDR_BIT] internally, and the peri xbar here
 *   splits the peri bus into UART / CLINT timer / MSIP / I2C / SPI /
 *   GPIO / PLIC / FFT / I2S by base+size. The window bases come from rv32_pkg (UART_BASE /
 *   MTIMER_BASE / MSIP_PERI_ADDR / I2C_BASE / SPI_BASE / GPIO_BASE /
 *   PLIC_BASE) so the map is defined in exactly one place.
 *
 * Pin assignments are in impl/pnr/rv32imac_Zicsr_Zifencei.cst.
 *
 * Naming: ports use *_i/_o; internal signals (including interface
 * instances) have no prefix. Module instance names keep u_*.
 */

module top_module (
    // 25 MHz reference clock from the MS5351M clock generator (crystal-fed).
    // The MS5351M drives independent single-ended CMOS clocks; CLK0 is on
    // PIN10. Used as a plain LVCMOS33 input (no differential / LVDS).
    input wire clk_i,
    // Board reset button S1 on PIN88. This board's button is ACTIVE-HIGH:
    // pressed = PIN88 HIGH = reset; released = LOW = run (the board pulls
    // the pin low when released). So the reset here is active-high — the
    // system runs with the button untouched and resets only while S1 is
    // held. (An earlier build treated this pin as active-low rstn_i, which
    // inverted the polarity and forced the button to be held to run.)
    input wire rst_i,

    // UART
    input  wire uart_rxd_i,
    output wire uart_txd_o,

    // I2C master (open-drain: the pin is driven low or released; external
    // pull-ups hold the lines high). Pin assignments in the .cst.
    inout wire i2c_scl_io,
    inout wire i2c_sda_io,

    // SPI master (mode 0-3, software chip-select).
    output wire spi_sck_o,
    output wire spi_mosi_o,
    input  wire spi_miso_i,
    output wire spi_cs_n_o,

    // GPIO (4 pins, push-pull; DIR=0 releases the pad, and the .cst's
    // PULL_MODE=UP holds a released pad high). Pin assignments in the .cst.
    inout wire [3:0] gpio_io,

    // I2S receiver. BCLK and LRCK are BIDIRECTIONAL pads: the peripheral
    // drives them in master mode (which is what an INMP441 needs -- the
    // mic is itself a clock slave) and releases them when an external
    // device is the master. SD is always an input; there is no transmit
    // path. Pin assignments in the .cst.
    inout wire i2s_bclk_io,
    inout wire i2s_lrck_io,
    input wire i2s_sd_i,

    // Debug LEDs: led_o[0] = stall indicator, led_o[3:1] = alive counter.
    output wire [3:0] led_o
);

    // -----------------------------------------------------------------
    // Clock generation
    //
    // Reference clock:
    //   clk_i = 25 MHz (MS5351M clock generator, crystal-fed; CLK0 on
    //   PIN10, single-ended LVCMOS33)
    //
    // Internal CPU clock (rPLL CLKOUT) — target 50 MHz:
    //   clk_core = FCLKIN * FBDIV / IDIV = 25 * 10 / 5 = 50 MHz
    //   (IDIV_SEL=4 -> IDIV=5, FBDIV_SEL=9 -> FBDIV=10; ODIV_SEL=16 sets
    //   the VCO = 25*10*16/5 = 800 MHz and does NOT divide CLKOUT).
    //   Period = 20 ns. Constrained in the SDC as a generated clock.
    //
    // Constraint check for this device: PFD = FCLKIN/IDIV = 5 MHz (range
    // 3-400 MHz), CLKOUT = 50 MHz (range 3.125-600), VCO = 800 MHz (range
    // 500-1250, and ODIV_SEL maxes out at 16 on this primitive — larger
    // values are silently replaced by the default 8, which would drop the
    // VCO to 400 and trip EX0311).
    //
    // 50 MHz is a target, not a verified closure. The pre-fetch-rewrite
    // design closed at 40.281 MHz actual via the LSU + CSR register stages;
    // the 64-bit fetch rewrite adds a buffer + room comparator, so whether
    // the remaining paths make 20 ns is what the next PnR run answers. The
    // fallback recipe is below the rPLL.
    // clk_core frequency, in Hz. MUST track the clock source below: it is
    // what the UART divides down to hit BAUD_RATE, so editing the clock
    // source without editing this too leaves the UART running at the wrong
    // baud (garbage on the wire).
    //
    // *** 50 MHz rPLL MODE (active). ***
    // MUST match the rPLL settings below: this is what the UART divides
    // down to hit BAUD_RATE, so changing one without the other puts the
    // serial line at the wrong baud, which on a board looks exactly like a
    // dead core. At 50 MHz BAUDDIV resets to CLK_FREQ_HZ/BAUD_RATE-1 = 433,
    // giving 50e6/434 = 115 207 Hz against a nominal 115 200 (+0.006%, well
    // inside RS-232 tolerance).
    //
    // The number itself lives in rv32_pkg as UART_CLK_HZ, which is what the
    // UART instance below reads. It is deliberately NOT duplicated here: a
    // second copy is the exact failure this comment block is about.

    wire clk_core;
    wire pll_lock;

`ifdef GOWIN
    rPLL #(  // For GW2AR-LV18QN88C8/I7 (Tang Nano 20K)
        .FCLKIN   ("25"),
        .IDIV_SEL (4),     // -> IDIV = 5,  PFD    =   5 MHz (range 3-400)
        .FBDIV_SEL(9),     // -> FBDIV = 10, CLKOUT = 50 MHz (range 3.125-600)
        .ODIV_SEL (16)     // ->            VCO    = 800 MHz (range 500-1250)
    ) pll (
        .CLKOUTP (),
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
        .CLKIN   (clk_i),     // 25 MHz reference
        .CLKOUT  (clk_core),  // 50 MHz core clock
        .LOCK    (pll_lock)
    );

`else
    assign clk_core = clk_i;
    assign pll_lock = 1'b1;
`endif

    // Falling back to the 25 MHz PLL-bypass build is three lines: comment
    // the rPLL out, set rv32_pkg::UART_CLK_HZ to 25_000_000, and restore
    //   wire clk_core = clk_i;
    //   wire pll_lock = 1'b1;
    // then re-comment the SDC's create_generated_clock and set
    // pnr_check.tcl / cmd.do global_freq back to 25.000. The bypass is also
    // the diagnostic configuration: it takes the rPLL out of the picture, so
    // a board that still does not come alive points at the clock reference
    // (clk_i / MS5351M) rather than at the PLL.

    // -----------------------------------------------------------------
    // Reset synchronization
    //
    // Reset is asserted while:
    //   - the external reset button S1 is pressed (rst_i HIGH — active-high
    //     on this board, see the port comment), OR
    //   - PLL has not locked yet.
    //
    // Deassertion is synchronized to clk_core.
    // -----------------------------------------------------------------

    logic [1:0] rst_sync;

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
    // AXI4-Lite buses (trunk modport). LINUX-BRANCH MAP (linux-soc M8,
    // see rv32_pkg): five windows, each at the offsets a mainline driver
    // expects.
    //   axi_bus_peri  : CPU peri master -> peri xbar (1->5, base+size).
    //   axi_bus_uart  : UART  16550   (UART_BASE  0x1000_0000,  4 KiB)
    //   axi_bus_spi   : SPI           (SPI_BASE   0x1000_6000,  4 KiB)
    //   axi_bus_gpio  : GPIO          (GPIO_BASE  0x1000_7000,  4 KiB)
    //   axi_bus_clint : CLINT, SiFive (CLINT_BASE 0x1001_0000, 64 KiB)
    //   axi_bus_plic  : PLIC, SiFive  (PLIC_BASE  0x1040_0000,  4 MiB)
    // FFT, I2S and I2C are main-only; their board pins are left released.
    // -----------------------------------------------------------------
    axi4_lite_if axi_bus_peri ();
    axi4_lite_if axi_bus_uart ();
    axi4_lite_if axi_bus_spi ();
    axi4_lite_if axi_bus_gpio ();
    axi4_lite_if axi_bus_clint ();
    axi4_lite_if axi_bus_plic ();

    // Single clock domain: the whole fabric runs on clk_core / rstn_core.
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

    // Debug tap: decode or execute stage stall.
    //wire         dbg_stall;

    // -----------------------------------------------------------------
    // Native memory ports. Fetch and the LSU each get a dedicated BSRAM;
    // the LSU steers RAM vs peri on addr[PERI_ADDR_BIT] itself, so the
    // board top needs no crossbar for memory.
    // -----------------------------------------------------------------
    // Fetch I-mem port is 64-bit read-only (ifetch); the LSU D-mem port stays
    // on the 32-bit byte-strobed mem_req_t / mem_rsp_t.
    ifetch_req_t        imem_req;
    ifetch_rsp_t        imem_rsp;
    mem_req_t           dmem_req;
    mem_rsp_t           dmem_rsp;
    // The read-only I-mem holds BVALID low (no write-ack); sink it so the
    // port is connected (a native_ram write-ack only exists for the D-mem).
    wire                imem_bvalid_unused;

    // Interrupt pending bits from the peri MMIO slaves.
    wire                msip;
    wire                mtip;
    wire         [63:0] mtime;

    // Machine external interrupt from the sifive PLIC's context 0; the S-mode
    // context (seip) has no consumer until S-mode exists (M11).
    wire                uart_irq;
    wire                spi_irq;
    wire                gpio_irq;
    wire                meip;
    wire                seip_unused;

    // -----------------------------------------------------------------
    // CPU. Functional ports only — no debug crosses the CPU boundary
    // (the per-stage taps are internal; the simulation probes them via
    // the Verilator hierarchy).
    // -----------------------------------------------------------------
    // IMEM_ADDR_W must match u_imem below: fetch uses it to tell a PC inside
    // the implemented I-mem from one outside it, which is the difference
    // between fetching an instruction and taking an access fault.
    rv32imac_zicsr_zifencei #(
        .IMEM_ADDR_W       (14),
        .BP_EN             (rv32_pkg::BP_EN),
        .MUL_SHARED_DSP    (rv32_pkg::MUL_SHARED_DSP),
        .BP_PUSH_LOOKUP    (rv32_pkg::BP_PUSH_LOOKUP),
        .EXEC_REDIR_INCYCLE(rv32_pkg::EXEC_REDIR_INCYCLE),
        .LSU_LIVE_LOAD     (rv32_pkg::LSU_LIVE_LOAD)
    ) u_cpu (
        .clk_i       (clk_core),
        .rstn_i      (rstn_core),
        .boot_addr_i (32'h0000_0000),
        //.dbg_stall_o(dbg_stall),
        .axi_peri    (axi_bus_peri.master),
        .imem_req_o  (imem_req),
        .imem_rsp_i  (imem_rsp),
        .dmem_req_o  (dmem_req),
        .dmem_rsp_i  (dmem_rsp),
        .msip_i      (msip),
        .mtip_i      (mtip),
        .mtime_i     (mtime),
        .meip_i      (meip),
        // BSRAM memories are always coherent: fence.i is a flush + refetch.
        .fencei_req_o(),
        .fencei_ack_i(1'b1)
    );

    // -----------------------------------------------------------------
    // Instruction memory (read-only). Fetch's dedicated port — no
    // contention with the LSU. Preloaded with firmware via INIT_FILE
    // (see the parameter comment below); simulation preloads via
    // $readmemh in sim_top.
    // -----------------------------------------------------------------
    native_ram #(
        .ADDR_W     (14),                                   // 16 KiB (see note below)
        .DATA_WIDTH (64),                                   // one access -> two 32-bit words
        .READ_ONLY  (1),
        .OUTSTANDING(2),                                    // 2 fetch reads in flight
        // A read-only I-mem with no init and no write port is a zero-ROM:
        // Gowin folds every read to constant 0, the fetch stream becomes
        // all-illegal, and the whole pipeline (regfile/csr/alu/execute)
        // gets swept as dead code -- only fetch+decode survive because
        // they feed dbg_stall_o. So the I-mem MUST carry firmware for any
        // meaningful (or even timing-representative) synthesis. The
        // default product firmware is CoreMark (sim/sw/coremark), the EEMBC
        // benchmark vendored verbatim in eembc/. Its image is built by `make`
        // in sim/sw/coremark/ (imem.hex = .text/.text.init -> I-mem 0x0,
        // dmem.hex = .rodata/.data -> D-mem 0x2000). Path is relative to the
        // Gowin project dir (repo root). The imem.hex is 64-bit-wide
        // ($readmemh words = 8 bytes each): the low 32 bits are the first
        // instruction at a word address, the high 32 bits the next (+4).
        // CoreMark .text is 11.1 KiB (of 16) -> 1423 64-bit words, under the
        // 2048-deep I-mem; it exercises the fetch/buffer path heavily, which
        // is what a timing-closure build must stress.
        .INIT_FILE  ("sim/sw/guitar_tuner/build/imem.hex")
    ) u_imem (
        .clk_i       (clk_core),
        .rstn_i      (rstn_core),
        .req_valid_i (imem_req.valid),
        .req_we_i    (1'b0),               // read-only
        .req_addr_i  (imem_req.addr),
        .req_wdata_i ({64{1'b0}}),
        .req_wstrb_i ({8{1'b0}}),
        .req_rready_i(imem_req.rready),
        .rsp_wready_o(imem_rsp.ready),
        .rsp_rvalid_o(imem_rsp.rvalid),
        .rsp_rdata_o (imem_rsp.rdata),
        .rsp_bvalid_o(imem_bvalid_unused)
    );

    // -----------------------------------------------------------------
    // Data memory (byte-strobed read/write). Holds .rodata/.data/.bss
    // and the stack. LSU's dedicated port. Preloaded with the firmware's
    // .rodata/.data image (dmem.hex, @ word 0x800 = byte 0x2000, matching
    // the link VMA) so the monitor's strings/literals are present at
    // power-up; .bss and the stack are zeroed by start.S / runtime use.
    // -----------------------------------------------------------------
    native_ram #(
        .ADDR_W     (14),                                   // 16 KiB (see note below)
        .DATA_WIDTH (32),
        .READ_ONLY  (0),
        .OUTSTANDING(1),                                    // LSU single-outstanding
        .INIT_FILE  ("sim/sw/guitar_tuner/build/dmem.hex")
    ) u_dmem (
        .clk_i       (clk_core),
        .rstn_i      (rstn_core),
        .req_valid_i (dmem_req.wvalid),
        .req_we_i    (dmem_req.we),
        .req_addr_i  (dmem_req.addr),
        .req_wdata_i (dmem_req.wdata),
        .req_wstrb_i (dmem_req.wstrb),
        .req_rready_i(dmem_req.rready),
        .rsp_wready_o(dmem_rsp.wready),
        .rsp_rvalid_o(dmem_rsp.rvalid),
        .rsp_rdata_o (dmem_rsp.rdata),
        .rsp_bvalid_o(dmem_rsp.bvalid)
    );

    // -----------------------------------------------------------------
    // Peripheral crossbar, Linux-branch map (see the bus list above). An
    // address in the peri region matching no window gets a DECERR from the
    // xbar's terminator. Target side is flat vectors glued to each slave's
    // interface (see axi4_lite_xbar.sv for why).
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
    // Main-only peripherals' pads (I2C, I2S): released, inputs unused. The
    // ports stay so the board .cst needs no edit between branches.
    // -----------------------------------------------------------------
    assign i2c_scl_io                 = 1'bz;
    assign i2c_sda_io                 = 1'bz;
    assign i2s_bclk_io                = 1'bz;
    assign i2s_lrck_io                = 1'bz;

    // -----------------------------------------------------------------
    // SPI master (axi4_lite_spi). miso double-flopped (asynchronous pin).
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
    // GPIO (axi4_lite_gpio), 4 push-pull header pins, per-bit tri-state,
    // inputs double-flopped, sync chain resets high (pulled-up pads). See
    // `main`'s top_module for the full reset-pend discussion.
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
    // CLINT, SiFive layout (clint.sv): msip / mtip straight into the CPU,
    // mtime into the time CSR.
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
    // PLIC, sifive,plic-1.0.0 (axi4_lite_plic_sifive). Source IDs by NAME
    // from rv32_pkg; IDs 2 (I2C), 5 (FFT), 6 (I2S) are main-only and tied
    // off here.
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
    // UART, 16550-compatible (axi4_lite_uart16550). DL resets to
    // UART_CLK_HZ / (16 * UART_BAUD) = 27 at 50 MHz / 115200 (115 741 baud,
    // +0.47%); Linux reprograms it from the DT clock-frequency (50 MHz).
    // uart_rxd_i double-flopped: asynchronous far-end transmitter.
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
    ) uart_i (
        .clk_i     (clk_core),
        .rstn_i    (rstn_core),
        .axi       (axi_bus_uart.slave),
        .txd_o     (uart_txd_o),
        .rxd_i     (uart_rxd_sync_q[1]),
        .uart_irq_o(uart_irq)
    );

    // -----------------------------------------------------------------
    // Debug LEDs:
    // led_o[0]: stall indicator (high when the CPU is stalled, low when it is running)
    // led_o[3:1]: free-running counter on clk_core (alive indicator).
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
    //assign led_o[0]   = dbg_stall;
    assign led_o[0]   = gpio_oe[0] ? gpio_out[0] : 1'b0;  // GPIO0 overrides the stall LED

endmodule

`resetall

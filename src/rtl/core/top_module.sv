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
 *                                                        |     (peri 1->6,
 *                                                        |      base+size)
 *                                                        |
 *                            0x1000_0000..0FFF ----------+--> uart_i  (UART)
 *                            0x1000_1000..2FFF ----------+--> u_timer (CLINT)
 *                            0x1000_3000..3FFF ----------+--> u_msip  (MSIP)
 *                            0x1000_5000..5FFF ----------+--> u_i2c   (I2C)
 *                            0x1000_6000..6FFF ----------+--> u_spi   (SPI)
 *
 *   Fetch and the LSU no longer contend: each has a dedicated native
 *   BSRAM port. AXI survives only for peripherals (the peri bridge is
 *   inside the CPU). The board top is pure point-to-point wires — the
 *   LSU steers addr[PERI_ADDR_BIT] internally, and the peri xbar here
 *   splits the peri bus into UART / CLINT timer / MSIP / I2C / SPI
 *   by base+size. The window bases come from rv32_pkg (UART_BASE /
 *   MTIMER_BASE / MSIP_PERI_ADDR / I2C_BASE / SPI_BASE) so the
 *   map is defined in exactly one place.
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
    // AXI4-Lite buses (trunk modport).
    //
    //   axi_bus_peri  : CPU peri master -> peri xbar (1->5, base+size decode).
    //   axi_bus_uart  : xbar m0 -> UART  slave (UART_BASE      0x1000_0000).
    //   axi_bus_timer : xbar m1 -> timer slave (MTIMER_BASE    0x1000_1000).
    //   axi_bus_msip  : xbar m2 -> MSIP  slave (MSIP_PERI_ADDR 0x1000_3000).
    //   axi_bus_i2c   : xbar m3 -> I2C   slave (I2C_BASE       0x1000_5000).
    //   axi_bus_spi   : xbar m4 -> SPI   slave (SPI_BASE       0x1000_6000).
    // -----------------------------------------------------------------
    axi4_lite_if axi_bus_peri ();
    axi4_lite_if axi_bus_msip ();
    axi4_lite_if axi_bus_timer ();
    axi4_lite_if axi_bus_uart ();
    axi4_lite_if axi_bus_i2c ();
    axi4_lite_if axi_bus_spi ();

    // Single clock domain: the whole fabric (CPU bridge, memories, the
    // buses) runs on clk_core / rstn_core. There is NO clock-domain
    // crossing — clk_i (25 MHz) only feeds the rPLL, and rstn_i is the
    // async board reset that feeds the synchronizer.
    assign axi_bus_peri.aclk     = clk_core;
    assign axi_bus_peri.aresetn  = rstn_core;
    assign axi_bus_msip.aclk     = clk_core;
    assign axi_bus_msip.aresetn  = rstn_core;
    assign axi_bus_timer.aclk    = clk_core;
    assign axi_bus_timer.aresetn = rstn_core;
    assign axi_bus_uart.aclk     = clk_core;
    assign axi_bus_uart.aresetn  = rstn_core;
    assign axi_bus_i2c.aclk      = clk_core;
    assign axi_bus_i2c.aresetn   = rstn_core;
    assign axi_bus_spi.aclk      = clk_core;
    assign axi_bus_spi.aresetn   = rstn_core;

    // Debug tap: decode or execute stage stall.
    wire         dbg_stall;

    // -----------------------------------------------------------------
    // Native memory ports. Fetch and the LSU each get a dedicated BSRAM;
    // the LSU steers RAM vs peri on addr[PERI_ADDR_BIT] itself, so the
    // board top needs no crossbar for memory.
    // -----------------------------------------------------------------
    // Fetch I-mem port is 64-bit read-only (ifetch); the LSU D-mem port stays
    // on the 32-bit byte-strobed mem_req_t / mem_rsp_t.
    ifetch_req_t imem_req;
    ifetch_rsp_t imem_rsp;
    mem_req_t    dmem_req;
    mem_rsp_t    dmem_rsp;
    // The read-only I-mem holds BVALID low (no write-ack); sink it so the
    // port is connected (a native_ram write-ack only exists for the D-mem).
    wire         imem_bvalid_unused;

    // Interrupt pending bits from the peri MMIO slaves.
    wire         msip;
    wire         mtip;

    // Machine external interrupt: OR of the peripheral level IRQs (UART,
    // I2C, SPI). The IRQs reset
    // deasserted (peripheral IE registers reset 0), so an unconfigured
    // peripheral never raises meip. Swap in a PLIC when cause IDs matter.
    wire         uart_irq;
    wire         i2c_irq;
    wire         spi_irq;
    wire         meip = uart_irq | i2c_irq | spi_irq;

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
        .clk_i      (clk_core),
        .rstn_i     (rstn_core),
        .boot_addr_i(32'h0000_0000),
        .dbg_stall_o(dbg_stall),
        .axi_peri   (axi_bus_peri.master),
        .imem_req_o (imem_req),
        .imem_rsp_i (imem_rsp),
        .dmem_req_o (dmem_req),
        .dmem_rsp_i (dmem_rsp),
        .msip_i     (msip),
        .mtip_i     (mtip),
        .meip_i     (meip)
    );

    // -----------------------------------------------------------------
    // Instruction memory (read-only). Fetch's dedicated port — no
    // contention with the LSU. Preloaded with firmware via INIT_FILE
    // (see the parameter comment below); simulation preloads via
    // $readmemh in sim_top.
    // -----------------------------------------------------------------
    native_ram #(
        .ADDR_W     (14),                               // 16 KiB (see note below)
        .DATA_WIDTH (64),                               // one access -> two 32-bit words
        .READ_ONLY  (1),
        .OUTSTANDING(2),                                // 2 fetch reads in flight
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
        .INIT_FILE  ("sim/sw/coremark/build/imem.hex")
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
        .ADDR_W     (14),                               // 16 KiB (see note below)
        .DATA_WIDTH (32),
        .READ_ONLY  (0),
        .OUTSTANDING(1),                                // LSU single-outstanding
        .INIT_FILE  ("sim/sw/coremark/build/dmem.hex")
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
    // Peripheral bus: parametric address-decode xbar (1->N) splitting the
    // peri bus into the UART, the CLINT timer, the MSIP slave, the I2C
    // master and the SPI master by base+size. Single-outstanding pass-through (the
    // CPU bridge is single-outstanding overall). An address in the peri
    // region matching no window is completed with a DECERR by the xbar's
    // terminator, not left to hang.
    // The xbar's target side is flat vectors (a parametric port count
    // cannot be N interface ports, and Gowin rejects interface-array
    // ports -- see axi4_lite_xbar.sv), so each slave's axi4_lite_if
    // instance is glued to its window with plain assigns: payload
    // broadcast, handshake bit i = window i.
    //   window 0 -> axi_bus_uart  (UART_BASE      0x1000_0000, 4 KiB)
    //   window 1 -> axi_bus_timer (MTIMER_BASE    0x1000_1000, 8 KiB)
    //   window 2 -> axi_bus_msip  (MSIP_PERI_ADDR 0x1000_3000, 4 KiB)
    //   window 3 -> axi_bus_i2c   (I2C_BASE       0x1000_5000, 4 KiB)
    //   window 4 -> axi_bus_spi   (SPI_BASE       0x1000_6000, 4 KiB)
    // 0x1000_4000 is unmapped (it held the SDIO controller until it was
    // dropped from this branch); an access there gets a DECERR.
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
        .N(PERI_N),
        .BASES({
            rv32_pkg::SPI_BASE,
            rv32_pkg::I2C_BASE,
            rv32_pkg::MSIP_PERI_ADDR,
            rv32_pkg::MTIMER_BASE,
            rv32_pkg::UART_BASE
        }),
        .SIZES({
            rv32_pkg::SPI_SIZE,
            rv32_pkg::I2C_SIZE,
            rv32_pkg::MSIP_PERI_SIZE,
            rv32_pkg::MTIMER_SIZE,
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

    // Window 0: UART. Payload is broadcast; glue the per-target bits.
    assign axi_bus_uart.awaddr   = peri_awaddr;
    assign axi_bus_uart.wdata    = peri_wdata;
    assign axi_bus_uart.wstrb    = peri_wstrb;
    assign axi_bus_uart.araddr   = peri_araddr;
    assign axi_bus_uart.awvalid  = peri_awvalid[0];
    assign axi_bus_uart.wvalid   = peri_wvalid[0];
    assign axi_bus_uart.bready   = peri_bready[0];
    assign axi_bus_uart.arvalid  = peri_arvalid[0];
    assign axi_bus_uart.rready   = peri_rready[0];
    assign peri_awready[0]       = axi_bus_uart.awready;
    assign peri_wready[0]        = axi_bus_uart.wready;
    assign peri_bvalid[0]        = axi_bus_uart.bvalid;
    assign peri_arready[0]       = axi_bus_uart.arready;
    assign peri_rvalid[0]        = axi_bus_uart.rvalid;
    assign peri_bresp[0+:2]      = axi_bus_uart.bresp;
    assign peri_rresp[0+:2]      = axi_bus_uart.rresp;
    assign peri_rdata[0+:32]     = axi_bus_uart.rdata;

    // Window 1: CLINT timer.
    assign axi_bus_timer.awaddr  = peri_awaddr;
    assign axi_bus_timer.wdata   = peri_wdata;
    assign axi_bus_timer.wstrb   = peri_wstrb;
    assign axi_bus_timer.araddr  = peri_araddr;
    assign axi_bus_timer.awvalid = peri_awvalid[1];
    assign axi_bus_timer.wvalid  = peri_wvalid[1];
    assign axi_bus_timer.bready  = peri_bready[1];
    assign axi_bus_timer.arvalid = peri_arvalid[1];
    assign axi_bus_timer.rready  = peri_rready[1];
    assign peri_awready[1]       = axi_bus_timer.awready;
    assign peri_wready[1]        = axi_bus_timer.wready;
    assign peri_bvalid[1]        = axi_bus_timer.bvalid;
    assign peri_arready[1]       = axi_bus_timer.arready;
    assign peri_rvalid[1]        = axi_bus_timer.rvalid;
    assign peri_bresp[2+:2]      = axi_bus_timer.bresp;
    assign peri_rresp[2+:2]      = axi_bus_timer.rresp;
    assign peri_rdata[32+:32]    = axi_bus_timer.rdata;

    // Window 2: MSIP.
    assign axi_bus_msip.awaddr   = peri_awaddr;
    assign axi_bus_msip.wdata    = peri_wdata;
    assign axi_bus_msip.wstrb    = peri_wstrb;
    assign axi_bus_msip.araddr   = peri_araddr;
    assign axi_bus_msip.awvalid  = peri_awvalid[2];
    assign axi_bus_msip.wvalid   = peri_wvalid[2];
    assign axi_bus_msip.bready   = peri_bready[2];
    assign axi_bus_msip.arvalid  = peri_arvalid[2];
    assign axi_bus_msip.rready   = peri_rready[2];
    assign peri_awready[2]       = axi_bus_msip.awready;
    assign peri_wready[2]        = axi_bus_msip.wready;
    assign peri_bvalid[2]        = axi_bus_msip.bvalid;
    assign peri_arready[2]       = axi_bus_msip.arready;
    assign peri_rvalid[2]        = axi_bus_msip.rvalid;
    assign peri_bresp[4+:2]      = axi_bus_msip.bresp;
    assign peri_rresp[4+:2]      = axi_bus_msip.rresp;
    assign peri_rdata[64+:32]    = axi_bus_msip.rdata;

    // Window 3: I2C.
    assign axi_bus_i2c.awaddr    = peri_awaddr;
    assign axi_bus_i2c.wdata     = peri_wdata;
    assign axi_bus_i2c.wstrb     = peri_wstrb;
    assign axi_bus_i2c.araddr    = peri_araddr;
    assign axi_bus_i2c.awvalid   = peri_awvalid[3];
    assign axi_bus_i2c.wvalid    = peri_wvalid[3];
    assign axi_bus_i2c.bready    = peri_bready[3];
    assign axi_bus_i2c.arvalid   = peri_arvalid[3];
    assign axi_bus_i2c.rready    = peri_rready[3];
    assign peri_awready[3]       = axi_bus_i2c.awready;
    assign peri_wready[3]        = axi_bus_i2c.wready;
    assign peri_bvalid[3]        = axi_bus_i2c.bvalid;
    assign peri_arready[3]       = axi_bus_i2c.arready;
    assign peri_rvalid[3]        = axi_bus_i2c.rvalid;
    assign peri_bresp[6+:2]      = axi_bus_i2c.bresp;
    assign peri_rresp[6+:2]      = axi_bus_i2c.rresp;
    assign peri_rdata[96+:32]    = axi_bus_i2c.rdata;

    // Window 4: SPI.
    assign axi_bus_spi.awaddr    = peri_awaddr;
    assign axi_bus_spi.wdata     = peri_wdata;
    assign axi_bus_spi.wstrb     = peri_wstrb;
    assign axi_bus_spi.araddr    = peri_araddr;
    assign axi_bus_spi.awvalid   = peri_awvalid[4];
    assign axi_bus_spi.wvalid    = peri_wvalid[4];
    assign axi_bus_spi.bready    = peri_bready[4];
    assign axi_bus_spi.arvalid   = peri_arvalid[4];
    assign axi_bus_spi.rready    = peri_rready[4];
    assign peri_awready[4]       = axi_bus_spi.awready;
    assign peri_wready[4]        = axi_bus_spi.wready;
    assign peri_bvalid[4]        = axi_bus_spi.bvalid;
    assign peri_arready[4]       = axi_bus_spi.arready;
    assign peri_rvalid[4]        = axi_bus_spi.rvalid;
    assign peri_bresp[8+:2]      = axi_bus_spi.bresp;
    assign peri_rresp[8+:2]      = axi_bus_spi.rresp;
    assign peri_rdata[128+:32]   = axi_bus_spi.rdata;

    // -----------------------------------------------------------------
    // I2C master (axi4_lite_i2c). Open-drain pins: the peripheral outputs
    // only an oe (drive low when set); the tri-state here releases the pin
    // otherwise and the external pull-up (plus the weak internal one in the
    // .cst) holds the line high. The line value is fed back as the input —
    // double-flopped, same as uart_rxd_i: the far-end slave is asynchronous
    // to clk_core, and the engine samples sda_i at phase boundaries, so a
    // metastable sample would silently corrupt a byte or a false-START
    // detect. 2 flops per line; released idle is high.
    // -----------------------------------------------------------------
    wire i2c_scl_oe, i2c_sda_oe;

    assign i2c_scl_io = i2c_scl_oe ? 1'b0 : 1'bz;
    assign i2c_sda_io = i2c_sda_oe ? 1'b0 : 1'bz;

    logic [1:0] i2c_scl_sync_q;
    logic [1:0] i2c_sda_sync_q;

    always_ff @(posedge clk_core) begin
        if (!rstn_core) begin
            i2c_scl_sync_q <= 2'b11;  // released line idles high
            i2c_sda_sync_q <= 2'b11;
        end else begin
            i2c_scl_sync_q <= {i2c_scl_sync_q[0], i2c_scl_io};
            i2c_sda_sync_q <= {i2c_sda_sync_q[0], i2c_sda_io};
        end
    end

    axi4_lite_i2c u_i2c (
        .clk_i    (clk_core),
        .rstn_i   (rstn_core),
        .axi      (axi_bus_i2c.slave),
        .scl_i    (i2c_scl_sync_q[1]),
        .sda_i    (i2c_sda_sync_q[1]),
        .scl_oe_o (i2c_scl_oe),
        .sda_oe_o (i2c_sda_oe),
        .i2c_irq_o(i2c_irq)
    );

    // -----------------------------------------------------------------
    // SPI master (axi4_lite_spi). miso comes off a pin driven by an
    // external device with its own clock: double-flop it like uart_rxd_i
    // so a metastable sample never lands in the shift register.
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
    // MSIP MMIO slave (machine software interrupt). A write of bit[0]
    // to MSIP_PERI_ADDR (0x1000_3000) sets/clears mip.MSIP; msip_o feeds
    // u_cpu.msip_i.
    // -----------------------------------------------------------------
    msip_peri u_msip (
        .clk_i (clk_core),
        .rstn_i(rstn_core),
        .axi   (axi_bus_msip.slave),
        .msip_o(msip)
    );

    // -----------------------------------------------------------------
    // CLINT timer MMIO slave (machine timer interrupt). mtip_o = (mtime
    // >= mtimecmp); feeds u_cpu.mtip_i (mip.MTIP). SW clears it by
    // writing mtimecmp > mtime.
    // -----------------------------------------------------------------
    clint_timer u_timer (
        .clk_i (clk_core),
        .rstn_i(rstn_core),
        .axi   (axi_bus_timer.slave),
        .mtip_o(mtip)
    );

    // -----------------------------------------------------------------
    // UART Controller.
    //
    // uart_rxd_i comes off a board pin driven by a far-end transmitter with
    // its own oscillator: asynchronous to clk_core no matter that the fabric
    // is single-domain. Double-flop it before it reaches the RX sampler --
    // axi4_lite_uart samples rxd_i straight into its shift register and has
    // no framing-error flag, so a metastable sample would silently corrupt a
    // received byte.
    // -----------------------------------------------------------------
    logic [1:0] uart_rxd_sync_q;

    always_ff @(posedge clk_core) begin
        if (!rstn_core) begin
            uart_rxd_sync_q <= 2'b11;  // idle line is high
        end else begin
            uart_rxd_sync_q <= {uart_rxd_sync_q[0], uart_rxd_i};
        end
    end

    axi4_lite_uart #(
        .CLK_FREQ_HZ(rv32_pkg::UART_CLK_HZ),
        .BAUD_RATE  (rv32_pkg::UART_BAUD)
    ) uart_i (
        .clk_i (clk_core),
        .rstn_i(rstn_core),
        .axi   (axi_bus_uart.slave),
        .txd_o (uart_txd_o),
        .rxd_i (uart_rxd_sync_q[1]),

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
    assign led_o[0]   = dbg_stall;

endmodule

`resetall

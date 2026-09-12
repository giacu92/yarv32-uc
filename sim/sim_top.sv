`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Simulation top (Verilator). Mirrors top_module's peri bus wiring:
 *
 *   CPU.axi_peri -> axi_bus_peri -> axi4_lite_xbar (1->5, base+size)
 *                    |-> u_uart  (0x1000_0000)
 *                    |-> u_timer (0x1000_1000+)
 *                    |-> u_msip  (0x1000_3000)
 *                    |-> u_i2c   (0x1000_5000)
 *                    |-> u_spi   (0x1000_6000)
 * Fetch and the LSU each have a dedicated BSRAM (Harvard). The peri bus
 * carries the MSIP + CLINT timer MMIO slaves behind the peri xbar.
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
    // UART clock/baud, overridable from the Verilator command line
    // (-GUART_CLK_HZ=... -GUART_BAUD=...). Deliberately NOT taken from
    // rv32_pkg, unlike the four structural knobs below: the package carries
    // the BOARD baud (115200) and the sim wants a fast one.
    // The defaults keep the sim fast
    // (5 clocks per bit); pass the board's real numbers
    // (25_000_000 / 115_200 -> 217 clocks per bit) to check the RX
    // sampling phase at the divisor the hardware actually uses.
    parameter int unsigned UART_CLK_HZ        = 50_000_000,
    parameter int unsigned UART_BAUD          = 10_000_000,
    // Branch-predictor enable A/B knob: -GBP_EN=0 disables prediction and
    // reproduces the pre-predictor core exactly (the baseline). Default 1.
    parameter int          BP_EN              = rv32_pkg::BP_EN,
    // MUL structure A/B knob: -GMUL_SHARED_DSP=0 restores the three-product
    // form (see alu.sv). Functionally identical, so this is a timing knob
    // only -- the retire stream must not move.
    parameter int          MUL_SHARED_DSP     = rv32_pkg::MUL_SHARED_DSP,
    // PHT lookup placement A/B knob: -GBP_PUSH_LOOKUP=0 reads the PHT at
    // decode with the live GHR (the original form); 1 reads it at
    // instruction-buffer push time and carries the bit in the entry. Unlike
    // MUL_SHARED_DSP this one DOES move the retire stream -- the push-time
    // read sees a slightly older GHR, so predictions differ.
    parameter int          BP_PUSH_LOOKUP     = rv32_pkg::BP_PUSH_LOOKUP,
    // -GEXEC_REDIR_INCYCLE=1 restores the 2026-08-31 form where an execute
    // redirect also launches its read in the redirect cycle. Default 0 keeps
    // the register file off the I-mem address pins (see fetch_stage.sv) and
    // costs 1 cycle per mispredict / trap / mret.
    parameter int          EXEC_REDIR_INCYCLE = rv32_pkg::EXEC_REDIR_INCYCLE,
    // -GLSU_LIVE_LOAD=0 captures every bus op (the pre-2026-09-01 LSU).
    parameter int          LSU_LIVE_LOAD      = rv32_pkg::LSU_LIVE_LOAD
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
    // AXI4-Lite peripheral bus (trunk modport) + peri 1->3 split. Window
    // bases/sizes come from rv32_pkg, same as the board top.
    // -----------------------------------------------------------------
    axi4_lite_if axi_bus_peri ();
    axi4_lite_if axi_bus_msip ();
    axi4_lite_if axi_bus_timer ();
    axi4_lite_if axi_bus_uart ();
    axi4_lite_if axi_bus_i2c ();
    axi4_lite_if axi_bus_spi ();

    assign axi_bus_peri.aclk     = clk_i;
    assign axi_bus_peri.aresetn  = rstn_i;
    assign axi_bus_msip.aclk     = clk_i;
    assign axi_bus_msip.aresetn  = rstn_i;
    assign axi_bus_timer.aclk    = clk_i;
    assign axi_bus_timer.aresetn = rstn_i;
    assign axi_bus_uart.aclk     = clk_i;
    assign axi_bus_uart.aresetn  = rstn_i;
    assign axi_bus_i2c.aclk      = clk_i;
    assign axi_bus_i2c.aresetn   = rstn_i;
    assign axi_bus_spi.aclk      = clk_i;
    assign axi_bus_spi.aresetn   = rstn_i;

    // -----------------------------------------------------------------
    // Native memory ports. Fetch and the LSU each get a dedicated
    // native_ram.
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

    // Interrupt pending bits (mip.MSIP / mip.MTIP sources).
    wire         msip;
    wire         mtip;
    // Machine external interrupt: OR of the peripheral level IRQs, same
    // term as the board top (UART | I2C | SPI; the board's SDIO int is
    // dangling and the sim has no SD target at all). The I2C/SPI IRQs
    // reset deasserted, so they never raise meip unless a test enables
    // them through MMIO.
    wire         uart_irq;
    wire         i2c_irq;
    wire         spi_irq;
    wire         meip = uart_irq | i2c_irq | spi_irq;

    // -----------------------------------------------------------------
    // CPU. Functional ports only; debug is observed via the Verilator
    // hierarchy (--public-flat) from sim_main, not through ports here.
    // The aggregate stall tap (dbg_stall_o) is sunk to an unused wire —
    // it carries no per-stage debug, just a "pipe stalled" status bit.
    // -----------------------------------------------------------------
    wire         unused_dbg_stall;
    // IMEM_ADDR_W must match u_imem below: fetch uses it to tell a PC inside
    // the implemented I-mem from one outside it, which is the difference
    // between fetching an instruction and taking an access fault.
    rv32imac_zicsr_zifencei #(
        .IMEM_ADDR_W       (14),
        .BP_EN             (BP_EN),
        .MUL_SHARED_DSP    (MUL_SHARED_DSP),
        .BP_PUSH_LOOKUP    (BP_PUSH_LOOKUP),
        .EXEC_REDIR_INCYCLE(EXEC_REDIR_INCYCLE),
        .LSU_LIVE_LOAD     (LSU_LIVE_LOAD)
    ) u_cpu (
        .clk_i      (clk_i),
        .rstn_i     (rstn_i),
        .boot_addr_i(32'h0000_0000),
        .dbg_stall_o(unused_dbg_stall),
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
    // Instruction memory (read-only). Fetch's dedicated port. Preloaded
    // via $readmemh with the +IINIT=<path> plusarg (default "imem.hex").
    // -----------------------------------------------------------------
    native_ram #(
        // 16 KiB, same depth as top_module: the GW2AR-18C has 46 BSRAM
        // blocks (828 Kb), so two 64 KiB memories cannot both exist -- and a
        // simulation with different memories stops modelling the board
        // exactly where it matters. 64-bit wide: one access delivers two
        // 32-bit words; 2 outstanding keeps the BSRAM issuing through
        // decode stalls.
        .ADDR_W     (14),
        .DATA_WIDTH (64),
        .READ_ONLY  (1),
        .OUTSTANDING(2),
        .INIT_FILE  ("")
    ) u_imem (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
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
    // and the stack. Preloaded via $readmemh with the +DINIT=<path>
    // plusarg (default "dmem.hex").
    // -----------------------------------------------------------------
    native_ram #(
        // 16 KiB, same depth as top_module: the GW2AR-18C has 46 BSRAM
        // blocks (828 Kb), so two 64 KiB memories cannot both exist -- and a
        // simulation with different memories stops modelling the board
        // exactly where it matters.
        .ADDR_W     (14),
        .DATA_WIDTH (32),
        .READ_ONLY  (0),
        .OUTSTANDING(1),   // LSU single-outstanding
        .INIT_FILE  ("")
    ) u_dmem (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
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

    string iinit_file;
    string dinit_file;
    initial begin
        if (!$value$plusargs("IINIT=%s", iinit_file))
            iinit_file = "imem.hex";  // default: oracle code image
        if (!$value$plusargs("DINIT=%s", dinit_file))
            dinit_file = "dmem.hex";  // default: oracle data image
        $readmemh(iinit_file, u_imem.mem);
        $readmemh(dinit_file, u_dmem.mem);
    end

    // -----------------------------------------------------------------
    // RAM probe: expose a window of the data RAM as scalar wires so they
    // land in the VCD and can be watched in GTKWave (Verilator does not
    // trace unpacked arrays past --trace-max-array, so the full mem[] is
    // not in the VCD). The window is pointed at the program's data array
    // so you can watch it change as the program runs.
    //
    // Data lives in u_dmem (D-mem, 0-based). PROBE_BASE_WORD is the word
    // index of the data array in the data image — re-point it if the
    // link layout changes. word index = byte_addr / 4.
    // Default points at the C quicksort .data array (link.ld places .data
    // at DMEM 0x2000 = word 0x800, N=32 ints). The hand-crafted oracle
    // writes its data at 0x100 (word 64) at runtime — re-point there
    // (PROBE_BASE_WORD=64) to watch the oracle. The trap/timer tests
    // write their pass marker at 0x2000 (word 0x800) too.
    // In GTKWave the signals appear as:
    //   sim_top.g_mem_probe[<i>].mem_probe_w
    // -----------------------------------------------------------------
    localparam int unsigned PROBE_BASE_WORD = 64'h800;  // DMEM 0x2000 (.data / pass marker)
    localparam int unsigned PROBE_LEN = 64'd32;

    genvar gi;
    generate
        for (gi = 0; gi < PROBE_LEN; gi = gi + 1) begin : g_mem_probe
            wire [31:0] mem_probe_w = u_dmem.mem[PROBE_BASE_WORD+gi];
        end
    endgenerate

    // -----------------------------------------------------------------
    // Peripheral bus: parametric peri xbar (base+size decode) feeding the
    // UART, CLINT timer, MSIP, I2C and SPI MMIO slaves, mirroring the board
    // top (the board adds the SDIO target as window 3; the sim has no SD
    // card, so its windows are shifted down by one above MSIP).
    // The xbar's target side is flat vectors (see axi4_lite_xbar.sv for
    // why), so each slave's axi4_lite_if instance is glued to its bit
    // with plain assigns: payload broadcast, handshakes bit i = target i.
    //   window 0 -> axi_bus_uart  (UART_BASE      0x1000_0000)
    //   window 1 -> axi_bus_timer (MTIMER_BASE    0x1000_1000+)
    //   window 2 -> axi_bus_msip  (MSIP_PERI_ADDR 0x1000_3000)
    //   window 3 -> axi_bus_i2c   (I2C_BASE       0x1000_5000)
    //   window 4 -> axi_bus_spi   (SPI_BASE       0x1000_6000)
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
        .BASES({SPI_BASE, I2C_BASE, MSIP_PERI_ADDR, MTIMER_BASE, UART_BASE}),
        .SIZES({SPI_SIZE, I2C_SIZE, MSIP_PERI_SIZE, MTIMER_SIZE, UART_SIZE})
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

    // Window 3: I2C. Pins are tied off — no I2C slave model, so the lines
    // read released (high): an enabled transfer sees all-1s and times out
    // per the engine, which is exactly how an open bus behaves.
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

    wire unused_i2c_scl_oe, unused_i2c_sda_oe;

    axi4_lite_i2c u_i2c (
        .clk_i    (clk_i),
        .rstn_i   (rstn_i),
        .axi      (axi_bus_i2c.slave),
        .scl_i    (1'b1),               // released line (pulled high)
        .sda_i    (1'b1),
        .scl_oe_o (unused_i2c_scl_oe),
        .sda_oe_o (unused_i2c_sda_oe),
        .i2c_irq_o(i2c_irq)
    );

    // Window 4: SPI. MISO tied low (no slave model); the pin outputs sink
    // to unused wires.
    assign axi_bus_spi.awaddr  = peri_awaddr;
    assign axi_bus_spi.wdata   = peri_wdata;
    assign axi_bus_spi.wstrb   = peri_wstrb;
    assign axi_bus_spi.araddr  = peri_araddr;
    assign axi_bus_spi.awvalid = peri_awvalid[4];
    assign axi_bus_spi.wvalid  = peri_wvalid[4];
    assign axi_bus_spi.bready  = peri_bready[4];
    assign axi_bus_spi.arvalid = peri_arvalid[4];
    assign axi_bus_spi.rready  = peri_rready[4];
    assign peri_awready[4]     = axi_bus_spi.awready;
    assign peri_wready[4]      = axi_bus_spi.wready;
    assign peri_bvalid[4]      = axi_bus_spi.bvalid;
    assign peri_arready[4]     = axi_bus_spi.arready;
    assign peri_rvalid[4]      = axi_bus_spi.rvalid;
    assign peri_bresp[8+:2]    = axi_bus_spi.bresp;
    assign peri_rresp[8+:2]    = axi_bus_spi.rresp;
    assign peri_rdata[128+:32] = axi_bus_spi.rdata;

    wire unused_spi_sck, unused_spi_mosi, unused_spi_cs_n;

    axi4_lite_spi u_spi (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi_bus_spi.slave),
        .spi_sck_o (unused_spi_sck),
        .spi_mosi_o(unused_spi_mosi),
        .spi_miso_i(1'b0),
        .spi_cs_n_o(unused_spi_cs_n),
        .spi_irq_o (spi_irq)
    );

    msip_peri u_msip (
        .clk_i (clk_i),
        .rstn_i(rstn_i),
        .axi   (axi_bus_msip.slave),
        .msip_o(msip)
    );

    clint_timer u_timer (
        .clk_i (clk_i),
        .rstn_i(rstn_i),
        .axi   (axi_bus_timer.slave),
        .mtip_o(mtip)
    );

    // Double-flop the harness RX line before the UART, exactly as
    // top_module does for the board pin. Parity matters: without it the
    // synchronizer that only exists in the board top is never exercised
    // in simulation, so a bug in it could not be reproduced here.
    logic [1:0] uart_rxd_sync_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            uart_rxd_sync_q <= 2'b11;  // idle line is high
        end else begin
            uart_rxd_sync_q <= {uart_rxd_sync_q[0], uart_rxd_i};
        end
    end

    wire uart_txd;

    axi4_lite_uart #(
        .CLK_FREQ_HZ(UART_CLK_HZ),  // sim clock is a free-running C++ tick, not board-accurate
        .BAUD_RATE  (UART_BAUD)
    ) u_uart (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .axi       (axi_bus_uart.slave),
        .txd_o     (uart_txd),
        .rxd_i     (uart_rxd_sync_q[1]),  // harness line, double-flopped (board parity)
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
    int uart_log_fd;

    initial begin
        uart_log_fd = $fopen("sim_uart_tx.txt", "w");
    end

    always_ff @(posedge clk_i) begin
        if (rstn_i && u_uart.tx_push) begin
            $fwrite(uart_log_fd, "%c", u_uart.wdata_eff[7:0]);
            $fflush(uart_log_fd);
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

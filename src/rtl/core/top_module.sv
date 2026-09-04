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
 *                                                        +-- axi4_lite_xbar_3
 *                                                        |     (peri 1->3,
 *                                                        |      base+size)
 *                                                        |
 *                            0x1000_0000..0FFF ----------+--> uart_i  (UART)
 *                            0x1000_1000..2FFF ----------+--> u_timer (CLINT)
 *                            0x1000_3000..3FFF ----------+--> u_msip  (MSIP)
 *
 *   Fetch and the LSU no longer contend: each has a dedicated native
 *   BSRAM port. AXI survives only for peripherals (the peri bridge is
 *   inside the CPU). The board top is pure point-to-point wires — the
 *   LSU steers addr[PERI_ADDR_BIT] internally, and the peri xbar here
 *   splits the peri bus into UART / CLINT timer / MSIP by base+size.
 *   The window bases come from rv32_pkg (UART_BASE / MTIMER_BASE /
 *   MSIP_PERI_ADDR) so the map is defined in exactly one place.
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

    // Debug LEDs: led_o[0] = stall indicator, led_o[3:1] = alive counter.
    output wire [3:0] led_o,

    // GW2AR-18 embedded SDRAM (8 MiB). Fixed names — see header.
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
    wire sdram_clk;

    rPLL #(  // For GW2AR-LV18QN88C8/I7 (Tang Nano 20K)
        .FCLKIN   ("25"),
        .IDIV_SEL (4),     // -> IDIV = 5,  PFD    =   5 MHz (range 3-400)
        .FBDIV_SEL(9),     // -> FBDIV = 10, CLKOUT = 50 MHz (range 3.125-600)
        .ODIV_SEL (16)     // ->            VCO    = 800 MHz (range 500-1250)
    ) pll (
        .CLKOUTP (sdram_clk),
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
    //   axi_bus_peri  : CPU peri master -> peri xbar (1->3, base+size decode).
    //   axi_bus_uart  : xbar m0 -> UART  slave (UART_BASE      0x1000_0000).
    //   axi_bus_timer : xbar m1 -> timer slave (MTIMER_BASE    0x1000_1000).
    //   axi_bus_msip  : xbar m2 -> MSIP  slave (MSIP_PERI_ADDR 0x1000_3000).
    // -----------------------------------------------------------------
    axi4_lite_if axi_bus_peri ();
    axi4_lite_if axi_bus_msip ();
    axi4_lite_if axi_bus_timer ();
    axi4_lite_if axi_bus_uart ();

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

    // Debug tap: decode or execute stage stall.
    wire         dbg_stall;

    // -----------------------------------------------------------------
    // Native memory ports. Fetch and the LSU each get a dedicated BSRAM;
    // the LSU steers RAM vs peri on addr[PERI_ADDR_BIT] itself, so the
    // board top needs no crossbar for memory.
    // -----------------------------------------------------------------
    // Fetch I-mem port is 64-bit read-only (ifetch); the LSU D-mem port stays
    // on the 32-bit byte-strobed cpu_mem_req_t / cpu_mem_rsp_t.
    ifetch_req_t imem_req;
    ifetch_rsp_t imem_rsp;
    cpu_mem_req_t    dmem_req;
    cpu_mem_rsp_t    dmem_rsp;
    // The read-only I-mem holds BVALID low (no write-ack); sink it so the
    // port is connected (a native_ram write-ack only exists for the D-mem).
    wire         imem_bvalid_unused;

    // Interrupt pending bits from the peri MMIO slaves.
    wire         msip;
    wire         mtip;

    // Machine external interrupt: OR of the peripheral level IRQs. Only the
    // UART raises one today; add further peripherals to this term (or swap in
    // a PLIC) as they arrive.
    wire         uart_irq;
    wire         meip = uart_irq;

    // -----------------------------------------------------------------
    // CPU. Functional ports only — no debug crosses the CPU boundary
    // (the per-stage taps are internal; the simulation probes them via
    // the Verilator hierarchy).
    // -----------------------------------------------------------------
    // IMEM_ADDR_W must match the address space behind the fetch port: the
    // cache controller's system map is 24 bits (yarv32_cache_pkg::
    // SYS_ADDR_W -- 23 bits of SDRAM plus bit 23 selecting bootrom/CSR),
    // so a PC outside 24 bits faults instead of aliasing. 24, not the
    // SDRAM's 23: the bootrom lives at 0x80_0000 (bit 23) and fetch must
    // be able to execute from it -- at 23 the bootrom PCs would all be
    // instruction access faults and the CPU would have no boot path. The
    // BSRAM build used 14 (separate 16 KiB I/D BSRAMs, Harvard at the
    // backing level; the cache build is unified behind the caches).
    // Linker consequence: .text and .data/.bss/stack must now be laid out
    // DISJOINT in one shared space -- the split images (.text@0,
    // .data@0x2000) would collide.
    rv32imac_zicsr_zifencei #(
        .IMEM_ADDR_W       (24),
        .BP_EN             (rv32_pkg::BP_EN),
        .MUL_SHARED_DSP    (rv32_pkg::MUL_SHARED_DSP),
        .BP_PUSH_LOOKUP    (rv32_pkg::BP_PUSH_LOOKUP),
        .EXEC_REDIR_INCYCLE(rv32_pkg::EXEC_REDIR_INCYCLE),
        .LSU_LIVE_LOAD     (rv32_pkg::LSU_LIVE_LOAD)
    ) u_cpu (
        .clk_i      (clk_core),
        .rstn_i     (rstn_core),
        // Boot vector: the bootrom at 0x80_0000. SDRAM is empty at power-up
        // (volatile, nothing preloads it), so the reset PC must point at the
        // ROM's boot code; the loader copies the payload into SDRAM and jumps
        // there. Requires IMEM_ADDR_W=24 above -- the ROM region has bit 23
        // set, which fetch would fault on at 23.
        .boot_addr_i(32'h0080_0000),
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

    /*
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
*/

    // -----------------------------------------------------------------
    // Cache controller (I-cache + D-cache over the embedded 8 MiB SDRAM),
    // replacing the two BSRAM native_ram instances above.
    //
    // Its ports speak the 64-bit cpu_mem_req_t / cpu_mem_rsp_t, but the CPU is a
    // 32-bit master on that bus, and fetch speaks ifetch_req_t /
    // ifetch_rsp_t -- so two adapters sit here:
    //
    //   icache: ifetch <-> mem types. Pure field wiring -- fetch is
    //   read-only (we/wdata/wstrb stay zero) and the cache never raises
    //   bvalid (posted stores), so the only dropped field is that one.
    //   Fetch issues 8-byte-aligned addresses, and the cache returns the
    //   addressed doubleword whole, so no lane steering is needed.
    //
    //   dcache: the LSU is a 32-bit master on that 64-bit bus. The steering
    //   lives in mem_width_adapter (src/rtl/utils), not here: it replicates
    //   the CPU word into both halves and shifts the strobes into the one
    //   addr[2] selects (the cache's store path is byte-strobed, so the
    //   unstrobed half can never commit), and selects the addressed half
    //   back out of the load response, with the half-select registered at
    //   accept -- the master may change its request the cycle after wready,
    //   and the response can land many cycles later. All per-request state
    //   is the adapter's; what is left here is a pure slice between the
    //   64-bit low-lane view the CPU speaks and the 32-bit view the adapter
    //   speaks. Zero added latency: the request path is wires, the response
    //   path one mux.
    // -----------------------------------------------------------------
    cpu_mem_req_t  icache_req;
    cpu_mem_rsp_t  icache_rsp;
    cpu_mem_req_t  dcache_req;
    cpu_mem_rsp_t  dcache_rsp;
    cpu_mem32_req_t dmem32_req;
    cpu_mem32_rsp_t dmem32_rsp;

    always_comb begin
        icache_req        = '0;
        icache_req.valid  = imem_req.valid;
        icache_req.addr   = imem_req.addr;
        icache_req.rready = imem_req.rready;
    end
    assign imem_rsp.ready  = icache_rsp.wready;
    assign imem_rsp.rvalid = icache_rsp.rvalid;
    assign imem_rsp.rdata  = icache_rsp.rdata;

    // 64-bit low-lane view -> 32-bit view. The LSU only ever drives the low
    // lanes of the 64-bit bus, so this narrows without losing anything.
    always_comb begin
        dmem32_req        = '0;
        dmem32_req.valid  = dmem_req.valid;
        dmem32_req.we     = dmem_req.we;
        dmem32_req.addr   = dmem_req.addr[XLEN-1:0];
        dmem32_req.wdata  = dmem_req.wdata[XLEN-1:0];
        dmem32_req.wstrb  = dmem_req.wstrb[XLEN_STRB_W-1:0];
        dmem32_req.rready = dmem_req.rready;
    end

    // 32-bit selected word back into the low lane of the 64-bit response
    // (the LSU reads rdata[XLEN-1:0]).
    always_comb begin
        dmem_rsp        = '0;
        dmem_rsp.wready = dmem32_rsp.wready;
        dmem_rsp.rvalid = dmem32_rsp.rvalid;
        dmem_rsp.rdata  = dmem32_rsp.rdata;
        dmem_rsp.bvalid = dmem32_rsp.bvalid;
    end

    mem_width_adapter u_dmem_width (
        .clk_i     (clk_core),
        .rstn_i    (rstn_core),
        .cpu_req_i (dmem32_req),
        .cpu_rsp_o (dmem32_rsp),
        .mem_req_o (dcache_req),
        .mem_rsp_i (dcache_rsp)
    );

    cache_cntrl #(
        // Main memory: the GW2AR's embedded 8 MiB SDRAM, 23-bit byte
        // address (yarv32_cache_pkg::SYS_ADDR_W adds one bit on top for
        // the bootrom/CSR window). Must match the real die -- a smaller
        // value aliases the top of memory onto the bottom.
        .MEM_SIZE(23),
        // 32-byte line: one refill = 8 SDRAM beats at 32 bit, and the data
        // macros are one line wide (DATA_WIDTH = 2**(5+3) = 256 bit). The
        // 1 KiB full-page-burst figure was rejected in the plan -- ~260
        // core cyc/miss and a bank held 5.12 us against a 7.8 us tREFI.
        .CL_SIZE(4),
        // 2-way: the tag compare->hit->way-mux path is post-flop, so this
        // competes with -- but does not extend -- the CPU's critical path.
        // Go 4-way only if PnR slack allows.
        .N_WAY(2),
        // 8 KiB per cache: 128 sets x 2 ways x 32 B, the same BSRAM budget
        // as the two 16 KiB BSRAMs this replaces (16 of 46 blocks).
        .CACHE_SIZE(14),
        // MHZ, not Hz: this parameter spaces the SDRAM refresh interval
        // (sdram_controller's CYCLES_BETWEEN_REFRESH) and sizes the
        // power-up wait counter. rv32_pkg::UART_CLK_HZ is 50_000_000 --
        // passing it here made the refresh constant ~46x too large for
        // its 14-bit counter, silently truncating it to a wrong value
        // (EX3791 at sdram_controller.v:79) and overflowing the init-wait
        // localparam. Must stay the clk_core rate in MHz.
        .CLK_FREQ_MHZ(50),
        // JEDEC power-up wait before the SDRAM accepts PRECHARGE/MRS
        // (stable clock, NOP-only for >=100 us). Invisible in sim, fatal
        // on a real die -- the controller itself only waits 15 cycles.
        .SDRAM_INIT_US(200),
        // 2 KiB bootrom image, 64-bit $readmemh words (low 32 bits = the
        // instruction at the word's byte address). Path relative to the
        // Gowin project dir (repo root); a file that is not found is
        // silently an all-zero ROM, so check the NL0002 sweep line in the
        // synth log after a path change.
        .BOOTROM_FILE("sim/sw/bootrom_2k/build/imem.hex"),
        // Control register power-up value: caches ON, bypass OFF (bit 0
        // is CSR_BIT_BYPASS). Firmware can still flip it at runtime
        // through the CSR window at 0x80_1000.
        .CSR_RST_VAL('0)
    ) u_cache (
        .clk_i        (clk_core),
        .rstn_i       (rstn_core),
        .sdram_clk_i  (sdram_clk),
        .icache_req_i (icache_req),
        .icache_rsp_o (icache_rsp),
        .dcache_req_i (dcache_req),
        .dcache_rsp_o (dcache_rsp),
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
    // Peripheral bus: 1->3 address-decode mux splitting the peri bus into
    // the UART, the CLINT timer and the MSIP slave by base+size.
    // Single-outstanding pass-through (the CPU bridge is single-outstanding
    // overall). An address in the peri region matching no window is completed
    // with a DECERR by the xbar's terminator, not left to hang.
    //   m0 UART_BASE      (0x1000_0000, 4 KiB)
    //   m1 MTIMER_BASE    (0x1000_1000, 8 KiB)
    //   m2 MSIP_PERI_ADDR (0x1000_3000, 4 KiB)
    // -----------------------------------------------------------------
    axi4_lite_xbar_3 #(
        .BASE0(rv32_pkg::UART_BASE),
        .SIZE0(rv32_pkg::UART_SIZE),
        .BASE1(rv32_pkg::MTIMER_BASE),
        .SIZE1(rv32_pkg::MTIMER_SIZE),
        .BASE2(rv32_pkg::MSIP_PERI_ADDR),
        .SIZE2(rv32_pkg::MSIP_PERI_SIZE)
    ) u_peri_xbar (
        .clk_i (clk_core),
        .rstn_i(rstn_core),
        .s_axi (axi_bus_peri.slave),
        .m0_axi(axi_bus_uart.master),
        .m1_axi(axi_bus_timer.master),
        .m2_axi(axi_bus_msip.master)
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

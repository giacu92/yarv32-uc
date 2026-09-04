`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Execute stage — consumes the D/E control word (de_i) from decode and:
 *
 *   - selects the ALU operands (rs1 / rs2 / pc / imm / pc4 / rs1-shifted)
 *     per de_i.alu_src_a / alu_src_b;
 *   - drives the ALU (base RV32I + M + Zilx EA), launching the
 *     multi-cycle DIV/REM with start_i and stalling the pipe until the
 *     ALU asserts result_valid_o;
 *   - writes back ALU / PC4 / load (MEM) / old-CSR results to the
 *     register file (stores carry no reg writeback); loads retire via
 *     WB_MEM once the read response arrives;
 *   - exports ex_wb_en_o / ex_wb_addr_o / ex_wb_data_o to decode the
 *     same cycle a writeback retires, so decode's fwd_rs1/fwd_rs2
 *     forward the fresh value directly into the D/E register instead
 *     of re-reading the regfile — RAW hazards at distance-1 (including
 *     load-use and DIV/REM) cost zero bubbles;
 *   - resolves branches and redirects fetch (branch_valid_o /
 *     branch_addr_o), flushing decode's D/E register on a taken branch;
 *   - resolves sync traps / mret / interrupts at the commit point and
 *     drives the trap unit interface;
 *   - drives the native mem_req_o / mem_rsp_i port (the LSU) toward the
 *     peri bridge: loads/stores/Zilx indexed loads launch, stall the
 *     pipe until the response (loads) or retire on launch-accept
 *     (posted stores), and retire.
 *
 * fe_* (fetch) / de_* (decode) / ex_* (execute) taps follow the
 * stage-sigil convention: ex_* is the E/M register — pc / instr / valid
 * of the retired operation.
 *
 * Memory (LSU): loads / stores / Zilx indexed loads launch on the native
 * mem_req_o / mem_rsp_i (or peri_req_o / peri_rsp_i) port. A unified FSM
 * extends the DIV/REM busy state with EX_MEM_WAIT: a mem op launches the
 * cycle it is valid in EX_IDLE (wvalid=1, bridge wready=1 in its idle),
 * then the pipe stalls in EX_MEM_WAIT until the read response (rvalid,
 * load) retires it. The D-mem path has no capture cycle: its request is
 * driven combinationally from the ALU's EA tap in EX_IDLE, and the address is
 * latched in parallel with the launch for the consumers that read it in a
 * LATER cycle (the load byte select and a misaligned trap's mtval). The
 * PERI path keeps a capture cycle, because behind it sit the bridge, the
 * crossbar and a slave's own address decode, all combinational in the
 * address phase (see EX_MEM_LAUNCH). Stores are posted: they retire the same cycle as
 * launch-accept, not on bvalid — the bridge owns the write->B round
 * trip and naturally stalls any following mem op via wready until the
 * store actually completes (RAW-through-memory ordering preserved
 * there). Loads write back via WB_MEM with sub-word extract +
 * sign/zero extension.
 *
 * Current limitations:
 *
 *   - No trap delegation / PMP / S-U mode (machine mode only).
 *   - A misaligned access (LH/LHU at addr[0]=1, LW/SW at addr[1:0]!=0)
 *     raises a load/store-address-misaligned sync trap; a sub-word
 *     access crossing a word boundary is not handled.
 *   - DIV/REM overflow (INT_MIN / -1) is not handled (see alu.sv).
 *
 * Naming: ports *_i/_o; internals no prefix; flops _q/_d; instances u_*.
 */

module execute_stage #(
    // Forwarded to the ALU: MUL structure A/B knob (see alu.sv). Default 0 --
    // the shared-DSP form measured -2.52 ns on the 2026-09-01 PnR run, and
    // rv32_pkg::MUL_SHARED_DSP is 0. The default used to be 1, i.e. the
    // measured-worse form, which only stayed harmless because every real
    // instantiation overrides it from the package.
    parameter int unsigned MUL_SHARED_DSP = 0,
    // Whether an aligned D-mem LOAD launches its bus request live from the
    // effective address in EX_IDLE.
    //   1 = yes (default). Saves one cycle on every D-mem load; the address
    //       and the launch enable are combinational out of
    //       regfile -> forward -> ALU.
    //   0 = no. EVERY bus op captures address / data / strobes into flops in
    //       EX_IDLE and is driven from them in EX_MEM_LAUNCH, and the
    //       misaligned check reads the REGISTERED address. That is the
    //       pre-2026-09-01 LSU exactly -- the shape that closed 50 MHz at
    //       +0.093 ns on 2026-08-31 and ran CoreMark on silicon. Nothing
    //       derived from the live EA reaches the D-mem address pins, stall_o,
    //       or the trap path. The safe fallback.
    // Stores capture either way: a posted store retires on its own launch, so
    // "did it launch" would otherwise reach stall_o (see the fork below).
    parameter int unsigned LSU_LIVE_LOAD  = 1
) (
    input wire clk_i,
    input wire rstn_i,

    // D/E control word from decode.
    input de_t de_i,

    // CSR Regfile
    input  wire [XLEN-1:0] csr_rdata_i,  // read CSR value (old)
    output wire [XLEN-1:0] csr_wdata_o,  // write CSR value (new, RMW result)
    output wire            csr_wren_o,   // CSR write enable (execute-qualified)

    // Trap unit interface (trap unit lives at the CPU top, a peer of the
    // execute stage). Execute = "exception logic": it detects the triggers
    // and resolves cause/tval/pc from de_i + the EA, and exports them; the
    // trap unit consumes them, drives the CSR trap-write bundle + the fetch
    // redirect, and reports the pending+enabled interrupt back here.
    //
    // A pending+enabled machine interrupt (from the trap unit's read of
    // mstatus.MIE / mip / mie): MSIP and/or MTIP. Read to decide
    // take_interrupt and to wake WFI halt.
    input wire            int_pending_i,
    // The trap unit's resolved interrupt cause (MSI > MTI priority), used
    // as the mcause when an interrupt is taken.
    input wire [XLEN-1:0] int_cause_i,
    // The trap unit's resolved fetch target (mtvec vector for traps /
    // interrupts, mepc for mret). Merged with the normal branch redirect.
    input wire [XLEN-1:0] trap_redirect_addr_i,

    // Triggers + payload to the trap unit.
    output wire            sync_trap_req_o,   // sync exception entry
    output wire            mret_req_o,        // mret return
    output wire            take_interrupt_o,  // async interrupt entry
    output wire [XLEN-1:0] trap_cause_o,      // mcause value
    output wire [XLEN-1:0] trap_tval_o,       // mtval value
    output wire [XLEN-1:0] trap_pc_o,         // mepc value

    // Back-pressure from a future downstream stage (tied 0 now).
    input wire stall_i,

    // Back-pressure to decode: stall the D/E register while a DIV/REM
    // is running (and the launch cycle).
    output wire stall_o,

    // Flush decode's D/E register on a taken branch.
    output wire flush_o,

    // Writeback to the register file. wb_data_o is driven procedurally in
    // the writeback always_comb below, so it must be a variable (logic),
    // not a wire — a wire/net cannot take a procedural assignment
    // (GowinSynthesis EX3900). The assign-driven wb_addr_o / wb_en_o stay
    // wire.
    output wire  [     4:0] wb_addr_o,
    output logic [XLEN-1:0] wb_data_o,
    output wire             wb_en_o,

    // Branch redirect to fetch.
    output wire            branch_valid_o,
    output wire [XLEN-1:0] branch_addr_o,

    // Branch-predictor training (at resolve). Execute is the golden resolver:
    // it drives the PHT/GHR/RAS update for the one control-flow instruction
    // resolving this cycle. Kind is re-derived from branch_type + rd + rs1
    // (call = JAL/JALR writing x1/x5; return = JALR reading x1/x5 with rd=x0;
    // JALR with rs1 in {x1,x5} and rd=x0). pred_pht_index is the de_t snapshot so the
    // PHT update uses the history the branch was predicted with.
    output wire bp_train_t bp_train_o,

    // Native memory interface (LSU): loads/stores/Zilx launch here.
    output cpu_mem_req_t mem_req_o,
    input  cpu_mem_rsp_t mem_rsp_i,
    // Native memory interface for peripherals
    output cpu_mem_req_t peri_req_o,
    input  cpu_mem_rsp_t peri_rsp_i,

    // ex_* per-stage taps (E/M register): pc / instr / valid of the retired
    // op. Named like fetch's fe_*_o (no _dbg suffix) so every pipeline stage
    // exposes a uniform pc / instr / valid output.
    output wire [XLEN-1:0] ex_pc_o,
    output wire [XLEN-1:0] ex_instr_o,
    output wire            ex_valid_o
);

    // =================================================================
    // Operand selection + PC-link (PC+2 compressed / PC+4)
    // =================================================================
    logic [XLEN-1:0] pc_link;
    assign pc_link = de_i.pc + (de_i.is_compressed ? 32'd2 : 32'd4);

    logic [XLEN-1:0] operand_a, operand_b;

    // Both muxes are on the design's worst path (regfile -> forward ->
    // operand -> DSP -> alu_result -> wb_data -> forward), so their WIDTH is a
    // timing parameter, not just tidiness. Every input below is a flop field
    // of de_i: nothing that has to be computed this cycle -- no adder output,
    // no CSR read -- reaches an operand any more. See alu_src_a_t /
    // alu_src_b_t in rv32_pkg for what was removed and why.
    always_comb begin
        unique case (de_i.alu_src_a)
            ALU_A_RS1: operand_a = de_i.rs1_data;
            ALU_A_PC:  operand_a = de_i.pc;
            ALU_A_RS2: operand_a = de_i.rs2_data;
            default:   operand_a = '0;
        endcase
        unique case (de_i.alu_src_b)
            ALU_B_RS1: operand_b = de_i.rs1_data;
            ALU_B_IMM: operand_b = de_i.imm;
            ALU_B_RS2: operand_b = de_i.rs2_data;
            default:   operand_b = '0;
        endcase
    end

    // =================================================================
    // Multi-cycle op control: DIV/REM and memory (LSU) share one FSM.
    //
    // EX_IDLE      : ready. A DIV/REM launches (alu_start) -> EX_DIV_BUSY.
    //                An aligned D-MEM LOAD drives the bus this cycle,
    //                combinationally from the EA tap (dmem_load_drive), and
    //                stays here re-driving until the slave accepts, then goes
    //                to EX_MEM_WAIT. Every other bus op -- a peri access in
    //                either direction, or a D-mem STORE -- captures its
    //                address / data / strobes into flops and moves to
    //                EX_MEM_LAUNCH; nothing is driven this cycle. A
    //                misaligned access does not launch at all -> EX_MEM_TRAP.
    //                Single-cycle ALU/branch ops retire here.
    // EX_MEM_LAUNCH: drive the captured request, on the port the registered
    //                target bit selects, until the slave accepts. A store
    //                retires on the accept (posted); a load goes on to
    //                EX_MEM_WAIT.
    // EX_MEM_TRAP  : a misaligned access was detected in EX_IDLE and never
    //                launched. This cycle raises its sync trap off the
    //                REGISTERED address, so the trap -> mtvec -> pc_q
    //                redirect stays flop-launched. Aligned accesses never
    //                enter it, so it costs nothing on the common path.
    // EX_DIV_BUSY  : hold the pipe until the ALU asserts result_valid_o.
    // EX_MEM_WAIT  : hold the pipe until the load read response (rvalid)
    //                retires the op.
    //
    // Why the capture cycle is peri-only (2026-09-01, measured).
    //
    // A capture cycle used to sit in front of BOTH launches (2026-08-25 ->
    // 2026-09-01). It ended the
    //   regfile -> forward mux -> adder -> byte-strobe decode -> memory port
    // chain at a flop and started the bus from one, worth +4.8 ns when the
    // design was closing 40 MHz. It also cost one cycle on EVERY memory
    // access: 0.236 CPI on CoreMark, 0.283 on quicksort - the largest single
    // item left in the no-retire budget after the branch predictor and the
    // in-cycle redirect launch.
    //
    // Removing it from both paths at once did not close: PnR reported
    // -5.533 ns, 39.164 MHz, and all 25 worst paths shared one startpoint
    // (the regfile read output) and ended almost entirely inside the UART --
    // div_pending_q, the TX FIFO write enable, tx_ie/rx_ie, tx_wptr -- with
    // the CSR file behind them. No endpoint was in the D-mem. The reason is
    // that "the memory port" means two very different things here: the D-mem
    // is a slave that registers its inputs, whereas the peri port feeds
    // axi4_lite_master_bridge, which in S_IDLE drives axi.awaddr / wdata /
    // wstrb straight through from req_i, then axi4_lite_xbar_3, then the
    // slave's own address decode -- all combinational in the address phase.
    // So the peri launch had the ALU driving three levels of fabric plus a
    // slave's register decode in one cycle. The CSR endpoints were the same
    // root cause one step removed: result_ready fed off a wready that was
    // selected by a live alu_result bit, so the bridge's wready reached
    // csr_we / wb_en / op_commits.
    //
    // Splitting the two restores exactly the condition that closed 50 MHz on
    // the peri side (flop -> fabric) and keeps the whole cycle win on the
    // D-mem side, where every load and store in a benchmark's timed loop
    // actually goes. MMIO in a timed region is zero in CoreMark (it times
    // with mcycle), Dhrystone and quicksort, so the peri capture cycle is
    // free where it is measured and costs one cycle per MMIO access in
    // YarvMon / the UART oracles.
    //
    // What did NOT come back on the D-mem side is the silicon bug that
    // motivated the flop.
    // Driving the bus from a live value is sound because de_i is HELD by
    // stall_o for as long as the request is outstanding, so the EA is a
    // stable function of flop outputs -- it only has to settle within the
    // cycle, which is a timing requirement, not a correctness one. What is
    // NOT sound is reading that live value in a LATER cycle, and two
    // consumers do exactly that: the load byte select (response cycle) and
    // a misaligned trap's mtval. Both keep reading mem_addr_q, which is
    // still latched -- in parallel with the launch instead of ahead of it.
    //
    // de_i is held stable across the busy/wait states by decode's stall
    // (stall_o below holds decode's D/E register), so the ALU result (EA)
    // and the mem op fields stay valid through EX_MEM_WAIT. The done cycle
    // drops the stall so decode advances and the op is not relaunched
    // (ex_state_q is still BUSY/WAIT at done, so alu_start / mem_req_pending
    // are 0; it clears to IDLE for the next cycle's new de_i).
    // =================================================================
    logic is_div_op;
    assign is_div_op = de_i.alu_op inside {ALU_DIV, ALU_DIVU, ALU_REM, ALU_REMU};

    logic is_mem_op;
    assign is_mem_op = de_i.mem_read | de_i.mem_write;

    // A CSR op now takes two cycles: one to present the address to the
    // registered CSR read, one to retire with the data it returned.
    // de_i.csr_wren is decode's "CSR op present" flag and decode already
    // squashes it (with reg_write / mem_read / mem_write) when the encoding is
    // illegal, so no ~illegal term is needed on top of it here.
    logic is_csr_op;
    assign is_csr_op = de_i.csr_wren;

    logic csr_start;
    logic csr_ready;

    // Misaligned access: raises a precise sync trap (LAD_MIS / SAD_MIS); the
    // access never launches. LH/LHU needs addr[0]=0; LW/SW needs addr[1:0]=00.
    // Because every misaligned case traps, no access that reaches the LSU can
    // cross a word boundary, so the single-beat slaves are always sufficient.
    // Byte accesses (LB/LBU/SB) are always within a word. Only mem ops can be
    // misaligned — non-mem ops reuse mem_size as a don't-care decode default
    // (MS_W) and must NOT be suppressed, so the detector gates on is_mem_op.
    //
    // The alignment check is SPLIT, and the split is what keeps the trap off
    // the critical path now that the capture flop is gone:
    //
    //   - the LAUNCH GATE reads the live EA's low two bits in EX_IDLE. It is a
    //     two-bit test whose only consumer is wvalid, so it adds a couple of
    //     LUT levels to the launch path and nothing to the redirect path. It
    //     has to be live: a misaligned store must never reach the bus, and a
    //     misaligned peri load must never reach a slave with side effects
    //     (reading UART RXDATA pops a byte).
    //   - the TRAP fires one cycle later, from EX_MEM_TRAP, off the
    //     registered mem_addr_q. Raising it in EX_IDLE would put
    //     regfile -> ALU -> misaligned -> trap-vectored-entry -> pc_q on one
    //     cycle, which was the 40 MHz limiter (-0.975 ns). (The EA tap has
    //     since taken the MUL result mux out of that chain, but the trap entry
    //     still ends at pc_q and still does not belong in one cycle.)
    //
    // Cost: a misaligned access spends one extra cycle before trapping.
    // Invisible to the retire trace (a trapping op never retires) and free on
    // aligned accesses, which never enter EX_MEM_TRAP.
    typedef enum logic [2:0] {
        EX_IDLE,
        EX_MEM_LAUNCH,
        EX_MEM_TRAP,
        EX_DIV_BUSY,
        EX_MEM_WAIT,
        EX_CSR_WAIT
    } ex_state_e;
    ex_state_e ex_state_q, ex_state_d;

    logic alu_start;
    assign
        alu_start = de_i.valid & is_div_op & (ex_state_q == EX_IDLE) & ~freeze & ~trap_redirect_req;

    logic            alu_result_valid;
    logic [XLEN-1:0] alu_result;

    // Effective address, tapped off the ALU's adder ahead of the result mux
    // (alu.sv ea_o). Identical to alu_result for every memory op -- asserted
    // below -- but two mux levels earlier, and its low two bits come straight
    // off the bottom of the carry chain. EVERY address-derived LSU signal
    // reads this, not alu_result: the D-mem address pins, the misaligned
    // launch gate, the D-mem-vs-peri fork, the store strobes and the store
    // lane shift.
    logic [XLEN-1:0] lsu_ea;

    // A mem op is asking for the bus this cycle. Held high across a stalled
    // launch (peri bridge busy -> wready low) because de_i is held by stall_o,
    // so the same request is simply re-driven until the slave accepts.
    //
    // Gating: NOT by trap_redirect_req, which would be a combinational loop
    // now that the launch lives in EX_IDLE (take_interrupt reads
    // normal_int_eligible, which would read this). The two terms that matter
    // are broken out instead, both flop-derived:
    //   ~de_i.exception : a decode exception (illegal / ecall / ebreak /
    //                     fetch fault) never carries mem_read/mem_write, so
    //                     this is belt-and-braces, but it is free.
    //   ~wfi_halt_q     : on a WFI wake the interrupt is taken while de_i
    //                     already holds the instruction AFTER the wfi, which
    //                     may well be a load. It must be squashed, not
    //                     launched.
    // mret is never a mem op. Everything else that would have been covered by
    // trap_redirect_req is a normal-path interrupt, and normal_int_eligible
    // now excludes mem ops outright (see there) so it cannot fire against a
    // launch in flight -- which also means wvalid never drops before its
    // accept.
    logic            mem_req_pending;
    assign mem_req_pending = de_i.valid & is_mem_op &
        (ex_state_q == EX_IDLE) & ~freeze & ~de_i.exception & ~wfi_halt_q;

    // Registered request. The address and target bit are latched on every
    // pending request, in parallel with a D-mem launch rather than ahead of
    // it, for the consumers that read the address in a LATER cycle than the
    // one that computed it: the load byte select on the response, mtval on a
    // misaligned trap, and the response-side target select. wdata/wstrb are
    // latched for the PERI launch, which drives the fabric from these flops a
    // cycle later; the D-mem launch drives its own live.
    logic     [      XLEN-1:0] mem_addr_q;
    logic     [      XLEN-1:0] mem_wdata_q;
    logic     [XLEN_STRB_W-1:0] mem_wstrb_q;
    logic                      mem_is_peri_q;

    // Target select, split by which cycle reads it.
    //   _live : the launch decision in EX_IDLE. mem_is_peri_q is still the
    //           PREVIOUS op's bit there, so the D-mem-vs-peri fork has to be
    //           taken on the live address. It only reaches flops and the
    //           D-mem's own wvalid -- never the peri fabric.
    //   _rsp  : EX_MEM_LAUNCH, EX_MEM_WAIT and the response. The address
    //           that produced the outstanding access was latched a cycle or
    //           more ago; the flop is the only sound source (see the load
    //           byte select below).
    wire                       is_peri_live = lsu_ea[PERI_ADDR_BIT];
    wire                       is_peri_rsp = mem_is_peri_q;

    // Effective LSU response for the wait / response phase. Selected from the
    // flop, so a peri op cannot falsely retire on the D-mem's rvalid (the bug
    // from splitting only the request side). The launch phase needs no mux:
    // each launch state talks to exactly one port.
    cpu_mem_rsp_t                  lsu_rsp_wait;
    assign lsu_rsp_wait = is_peri_rsp ? peri_rsp_i : mem_rsp_i;

    // Misaligned-access LAUNCH GATE, off the LIVE effective address. Two bits
    // of the EA into wvalid; the trap it schedules is raised a cycle later
    // from the registered address (see EX_MEM_TRAP).
    // Two forms, because the two builds read the address at different times.
    //   _live : off the live EA. Only LSU_LIVE_LOAD=1 uses it, and only to
    //           filter the live load launch -- a misaligned access must never
    //           reach a slave. The TRAP is still raised a cycle later off the
    //           REGISTERED address, from EX_MEM_TRAP.
    //   _q    : off mem_addr_q. LSU_LIVE_LOAD=0 uses this and raises the trap
    //           from EX_MEM_LAUNCH, exactly as the design did before
    //           2026-09-01; alignment then touches the live EA nowhere.
    //
    // NOT gated by is_mem_op: every consumer below already ANDs with
    // mem_req_pending (which carries is_mem_op) or with a state that only a
    // mem op can reach, so the guard was a redundant AND on the launch gate --
    // the one place in this file where the live EA feeds a bus enable.
    function automatic logic misaligned_of(input logic [XLEN-1:0] a);
        unique case (de_i.mem_size)
            MS_B:    return 1'b0;
            MS_H:    return a[0];
            MS_W:    return |a[1:0];
            default: return 1'b1;
        endcase
    endfunction

    wire mem_misaligned_live = misaligned_of(lsu_ea);
    wire mem_misaligned_q = misaligned_of(mem_addr_q);

    wire mem_misaligned_launch = (LSU_LIVE_LOAD != 0) & mem_req_pending & mem_misaligned_live;

    // The sync trap for a misaligned access. de_i is held through whichever
    // state it fires from by stall_o, so mem_write (load vs store cause) and
    // the retire gating still read the right op.
    wire misaligned_trap = de_i.valid & ((LSU_LIVE_LOAD != 0) ? (ex_state_q == EX_MEM_TRAP) :
                                         ((ex_state_q == EX_MEM_LAUNCH) & mem_misaligned_q));

    // An aligned pending request forks, and the fork is chosen so that only
    // ONE case drives the bus live: an aligned D-mem LOAD.
    //
    //   D-mem load  : driven from EX_IDLE, live from the EA tap.
    //   everything   : captured this cycle, driven from EX_MEM_LAUNCH next
    //   else          cycle (peri load, peri store, D-mem store).
    //
    // Why a load and not a store, which is what the third PnR run of the day
    // settled (2026-09-01, -4.956 ns / 40.1 MHz). A load NEVER retires on its
    // launch -- it goes to EX_MEM_WAIT and retires on rvalid -- so nothing
    // about whether its launch happened has to reach stall_o: mem_req_pending,
    // which is flop-only, already holds decode. A STORE is posted and retires
    // on its accept, so "did this store launch" is exactly the question
    // stall_o has to answer in the same cycle, and answering it needs the
    // effective address: is it misaligned (EA[1:0]), is it peri
    // (EA[PERI_ADDR_BIT]), did the slave accept. That put the deepest
    // combinational value in the machine on the pipeline's CONTROL network:
    //
    //   regfile DO -> forward -> operand_a -> DSP (3.76 ns) -> alu_result
    //     -> mem_misaligned / is_peri_live -> the store's launch handshake
    //     -> store_done
    //     -> stall_o -> decode dec_stall -> fetch buf_pop_cnt -> head_q
    //
    // 27.192 ns end to end, 8.8 ns of it after alu_result, and every one of
    // the 25 worst paths in that run was a leaf of it (fetch head_q/count_q,
    // decode hold_pc_q/hold_word_q clock enables -- all gated by stall_i).
    //
    // THAT is what the capture stage was really buying, and it was never the
    // address path: the D-mem's own address pins have not appeared in a
    // timing report through any of these three runs. It was keeping
    // alu_result out of stall_o / wb_en_o / op_commits. Capturing stores puts
    // it back there and keeps the load's cycle, which is 78% of the win
    // (CoreMark: 62778 loads vs 17815 stores).
    // LSU_LIVE_LOAD=0 collapses the fork: nothing launches live, everything
    // captures, and this is the pre-2026-09-01 LSU.
    wire dmem_load_drive = (LSU_LIVE_LOAD != 0) &
        mem_req_pending & ~mem_misaligned_live & ~is_peri_live & de_i.mem_read;
    wire mem_capture = mem_req_pending & ~dmem_load_drive & ~mem_misaligned_launch;
    wire mem_launch_state = (ex_state_q == EX_MEM_LAUNCH);

    // Captured launch. Gated on de_i.valid as well as on the state: the state
    // is a flop and de_i is held by stall_o, but "held" is a property of other
    // logic, and what this drives is a WRITE to memory. If de_i were ever
    // cleared under this state (decode zeroes de_next on flush_i regardless of
    // stall_o), an ungated version would keep we/wvalid high while the EA
    // collapsed to 0 -- a write of stale data to address 0. No flush can
    // currently reach EX_MEM_LAUNCH (a mem op is not control flow, so
    // mispredict is 0, and every trap/mret/interrupt trigger requires
    // EX_IDLE), so the term is unreachable today; it is here because the cost
    // is one AND and the failure it prevents is silent memory corruption.
    wire captured_drive = mem_launch_state & de_i.valid & ~misaligned_trap;
    // The captured D-mem launch carries STORES only when LSU_LIVE_LOAD=1 (an
    // aligned D-mem load goes live and never reaches this state); it carries
    // BOTH directions when LSU_LIVE_LOAD=0. So the write enable must come from
    // de_i.mem_write, not from "this is the captured D-mem launch". Deriving we
    // from the drive alone made a captured LOAD assert we=1: the D-mem wrote
    // mem_wstrb_q into it instead of reading, never returned rvalid, and the
    // pipe sat in EX_MEM_WAIT forever. The sim caught it by hanging on the
    // Harvard oracle; had the access completed it would have been silent D-mem
    // corruption instead.
    wire dmem_captured_drive = captured_drive & ~is_peri_rsp;
    wire dmem_store_we = dmem_captured_drive & de_i.mem_write;

    // Any launch is driving a bus this cycle. No RTL consumer -- this is the
    // tap the sim's stall histogram reads (sim_main.cpp, `lsu-launch`).
    wire mem_bus_drive = dmem_load_drive | captured_drive;

    // A live D-mem load only ever talks to the D-mem, so it reads mem_rsp_i
    // directly. A captured launch reads the port the REGISTERED target bit
    // selects, which is the same select lsu_rsp_wait already carries -- so it
    // reuses that mux rather than building a second copy of it.
    wire dmem_load_hs = dmem_load_drive & mem_rsp_i.wready;
    wire captured_hs = captured_drive & lsu_rsp_wait.wready;

    // Posted store: once the bridge accepts AW+W (mem_launch_hs on a
    // write), it owns the write->B round trip on its own; the LSU
    // retires the same cycle instead of waiting for bvalid. Correctness:
    // the bridge is single-outstanding and only re-asserts wready once B
    // completes, so any *following* mem op still naturally stalls
    // (wready low) until the store has actually finished at the bridge -
    // RAW-through-memory ordering is preserved there, not by the LSU
    // waiting here. Loads still need EX_MEM_WAIT: data isn't available
    // until rvalid.
    logic mem_done;
    assign mem_done = (ex_state_q == EX_MEM_WAIT) & lsu_rsp_wait.rvalid;

    // A store only ever launches from EX_MEM_LAUNCH, so its retire pulse is
    // flop-derived: the state, the registered target bit, and the slave's
    // wready. No term of it descends from the live EA, which is the whole
    // point (see the fork above).
    logic store_done;
    assign store_done = captured_hs & de_i.mem_write;

    // Unified mem-op retire pulse: rvalid cycle for loads, launch-accept
    // cycle for (posted) stores.
    logic mem_op_done;
    assign mem_op_done = de_i.mem_read ? mem_done : store_done;

    logic div_running;
    assign div_running = (ex_state_q == EX_DIV_BUSY) & ~alu_result_valid;
    // Stall the launch + busy/wait cycles; the done cycle drops the stall
    // so decode advances (mem_running falls away the cycle mem_done=1).
    logic mem_running;
    assign mem_running = (ex_state_q == EX_MEM_WAIT) & ~mem_done;
    // A pending mem op stalls decode unless it is a store that got its accept
    // this cycle (posted -> retires now). EX_MEM_TRAP holds de_i for the
    // misaligned trap that fires from it.
    // Every term here is flop-derived. mem_req_pending is de_i + state +
    // freeze; mem_launch_state and EX_MEM_TRAP are states; store_done is the
    // captured handshake. Keeping the live EA out of this expression is what
    // the store capture exists for.
    assign
        stall_o = alu_start | div_running | ((mem_req_pending | mem_launch_state) & ~store_done) |
        (ex_state_q == EX_MEM_TRAP) | mem_running | csr_start | stall_i | wfi_stall;

    assign
        csr_start = de_i.valid & is_csr_op & (ex_state_q == EX_IDLE) & ~freeze & ~trap_redirect_req;
    assign csr_ready = (ex_state_q == EX_CSR_WAIT);

    always_comb begin
        ex_state_d = ex_state_q;
        unique case (ex_state_q)
            EX_IDLE: begin
                if (alu_start) ex_state_d = EX_DIV_BUSY;
                // Misaligned: no launch, trap from EX_MEM_TRAP next cycle.
                else if (mem_misaligned_launch) ex_state_d = EX_MEM_TRAP;
                // Peri (either direction) or a D-mem store: captured this
                // cycle, driven from the flops next.
                else if (mem_capture) ex_state_d = EX_MEM_LAUNCH;
                // Aligned D-mem load, accepted: wait for its data. Not
                // accepted -> stay here and re-drive (de_i is held).
                else if (dmem_load_hs) ex_state_d = EX_MEM_WAIT;
                else if (csr_start) ex_state_d = EX_CSR_WAIT;
            end
            EX_CSR_WAIT: ex_state_d = EX_IDLE;
            EX_MEM_LAUNCH:
            // Hold the request until the slave accepts it; then a load waits
            // for its data and a (posted) store has already retired. With
            // LSU_LIVE_LOAD=0 a misaligned op lands here too and leaves on its
            // trap without ever driving.
            if (misaligned_trap)
                ex_state_d = EX_IDLE;
            else if (captured_hs) ex_state_d = de_i.mem_read ? EX_MEM_WAIT : EX_IDLE;
            EX_MEM_TRAP: ex_state_d = EX_IDLE;  // trap raised this cycle
            EX_DIV_BUSY: if (alu_result_valid) ex_state_d = EX_IDLE;
            EX_MEM_WAIT: if (mem_done) ex_state_d = EX_IDLE;
            default: ex_state_d = EX_IDLE;
        endcase
    end

    // =================================================================
    // ALU instance
    // =================================================================
    alu #(
        .MUL_SHARED_DSP(MUL_SHARED_DSP)
    ) u_alu (
        .clk_i         (clk_i),
        .rst_ni        (rstn_i),
        .operand_a_i   (operand_a),
        .operand_b_i   (operand_b),
        .alu_op_i      (de_i.alu_op),
        .shamt_i       (de_i.mem_shamt),
        .start_i       (alu_start),
        .result_valid_o(alu_result_valid),
        .result_o      (alu_result),
        .ea_o          (lsu_ea)
    );

    // =================================================================
    // Trap / interrupt / mret (precise, at the commit point).
    //
    // Sync traps (illegal / ecall / ebreak / misaligned) fire in EX_IDLE the
    // cycle the faulting op would retire; the op commits as a trap (ex_valid
    // pulses, counts to minstret). mret commits and redirects to mepc. A
    // pending machine software interrupt is taken instead of retiring the
    // next normal instruction (suppressed, re-runs after mret) OR on a WFI
    // wake; it does NOT count as a retire. All three flush decode (kill
    // younger in-flight) and redirect fetch via branch_valid_o / branch_addr_o
    // (the trap unit's redirect merges with the normal branch redirect).
    //
    // Priority: sync trap > interrupt > mret > normal branch. Traps/mret/
    // interrupt never coincide with a normal branch (a branch is squashed by
    // take_interrupt; an illegal branch already trapped at decode).
    //
    // `freeze` extends the downstream stall with the WFI-halt freeze so a
    // held instruction does not retire/launch while waiting for a pending
    // interrupt. take_interrupt clears wfi_stall the cycle it fires, so
    // freeze drops out and the interrupt proceeds.
    // =================================================================
    wire is_wfi_op = (de_i.sys_op == SYS_WFI);
    wire is_mret_op = (de_i.sys_op == SYS_MRET);

    // WFI halt state. Declared early: wfi_stall / freeze gate the trap
    // predicates below, and freeze extends the downstream stall.
    // int_pending_i comes from the trap unit at the CPU top (combinational
    // from the CSR taps); trap_redirect_addr_i is the trap unit's resolved
    // fetch target (mtvec vector / mepc), merged with the normal branch
    // redirect in branch_valid_o / branch_addr_o below.
    logic wfi_halt_q;
    logic [XLEN-1:0] wfi_next_pc_q;
    wire wfi_stall = wfi_halt_q & ~int_pending_i;
    wire freeze = stall_i | wfi_stall;

    // Sync trap request: a decode exception (illegal/ecall/ebreak) fires in
    // EX_IDLE the cycle the faulting op would retire; a misaligned load/store
    // fires from EX_MEM_TRAP via misaligned_trap (one cycle after the
    // suppressed launch, off the registered EA — see the detector above). The
    // two never coincide: a decode exception is never a mem op, and
    // mem_req_pending is gated on ~de_i.exception.
    wire sync_trap_req = (de_i.valid & ~freeze & (ex_state_q == EX_IDLE) & de_i.exception) |
        misaligned_trap;

    wire mret_req = de_i.valid & ~freeze & (ex_state_q == EX_IDLE) & is_mret_op;

    // A normal instruction eligible to be squashed by an interrupt (any
    // non-trap, non-mret, non-wfi, non-mem valid op in EX_IDLE). WFI is
    // excluded: it retires (commit) then halts; the interrupt is taken on the
    // WFI wake path (wfi_halt_q) with mepc = wfi.pc + size.
    //
    // MEM OPS ARE EXCLUDED, and that is load-bearing in two ways now that the
    // launch lives in EX_IDLE. Correctness: mem_req_pending cannot be gated on
    // trap_redirect_req without a combinational loop, so the interrupt is what
    // gives way instead — it is taken after the access completes, one or two
    // cycles later, which is exactly as legal (RISC-V takes an interrupt
    // between instructions; this one retires first). Protocol: a launch held
    // against a busy peri bridge would otherwise see wvalid drop before its
    // accept. A misaligned op is no longer preempted either — it traps from
    // EX_MEM_TRAP and the interrupt is taken after the handler's mret, rather
    // than before the faulting access. Both orderings are architecturally
    // legal; this one falls out of the launch gate.
    wire normal_int_eligible = de_i.valid & ~is_wfi_op & ~de_i.exception & ~is_mem_op &
        (ex_state_q == EX_IDLE) & ~freeze;

    wire take_interrupt = int_pending_i & ~sync_trap_req & ~mret_req &
        (normal_int_eligible | wfi_halt_q);

    wire trap_redirect_req = sync_trap_req | mret_req | take_interrupt;

    // Payload resolved for the trap unit (at the CPU top). A misaligned
    // access overrides the decode cause/tval (bad EA, from the registered
    // mem_addr_q); an interrupt forces the MSI cause / tval=0. The trap unit
    // consumes these as cause_i / tval_i / pc_i and produces the redirect +
    // CSR trap-write bundle. misaligned_trap (the EX_MEM_TRAP state, not the
    // live mem_misaligned) selects, so an exception in EX_IDLE — where
    // mem_addr_q is stale — reads its own cause/tval.
    wire [XLEN-1:0] sync_cause = misaligned_trap ?
        (de_i.mem_write ? MCAUSE_SAD_MIS : MCAUSE_LAD_MIS) : de_i.exception_cause;
    wire [XLEN-1:0] sync_tval = misaligned_trap ? mem_addr_q : de_i.exception_tval;
    wire [XLEN-1:0] trap_cause = take_interrupt ? int_cause_i : sync_cause;
    wire [XLEN-1:0] trap_tval = take_interrupt ? 32'd0 : sync_tval;
    // mepc: faulting instr pc (sync trap), or the suppressed instr pc
    // (interrupt), or the WFI-wake pc (interrupt taken after WFI). The wake
    // pc is pc + (2|4), which is pc_link -- the same adder, not a second one.
    wire [XLEN-1:0] trap_mepc = wfi_halt_q ? wfi_next_pc_q : de_i.pc;

    // Export the triggers + payload to the trap unit (CPU top).
    assign sync_trap_req_o  = sync_trap_req;
    assign mret_req_o       = mret_req;
    assign take_interrupt_o = take_interrupt;
    assign trap_cause_o     = trap_cause;
    assign trap_tval_o      = trap_tval;
    assign trap_pc_o        = trap_mepc;

    // -----------------------------------------------------------------
    // WFI halt. WFI retires (commit) once, then freezes the pipe until a
    // pending+enabled interrupt wakes it. The interrupt is taken on the
    // wake via the wfi_halt_q branch of take_interrupt, with mepc =
    // wfi.pc + size (the instruction that will execute after the handler).
    // freeze = the downstream stall OR the WFI wait; take_interrupt clears
    // wfi_stall the cycle it fires (int_pending high -> wfi_stall low), so
    // freeze drops out and the interrupt proceeds.
    // -----------------------------------------------------------------
    wire wfi_retire = de_i.valid & (ex_state_q == EX_IDLE) &
        is_wfi_op & ~trap_redirect_req & ~stall_i;

    logic wfi_halt_d;
    logic [XLEN-1:0] wfi_next_pc_d;

    // =================================================================
    // Writeback (ALU / PC4 / MEM). A load retires via WB_MEM with sub-word
    // extract + sign/zero extension; stores have reg_write=0 (no wb).
    // =================================================================
    // Load alignment: the memory returns the containing word; shift the
    // addressed bytes down to bit 0, then sign/zero-extend per mem_size
    // and mem_unsigned. The result is sampled the cycle mem_done (rvalid)
    // pulses, when rdata is valid.
    //
    // The byte offset comes from a copy of the effective address latched in
    // the launch cycle, NOT from the live EA. The EA is combinational out
    // of the D/E operands, and those operands can be fed by the
    // execute->decode forward path, so reading it a cycle (or several)
    // later means the byte select depends on the whole
    // regfile -> forward mux -> adder chain still being settled and
    // unchanged at response time. That is the design's critical path, and
    // on hardware it was not settled: a load whose address came from a
    // distance-1 forward (an `add` immediately followed by `lbu`, which is
    // what indexing a table with a computed index compiles to) selected
    // byte 0 instead of the addressed byte, while the same load with the
    // address already committed in the regfile was correct. It read the
    // right word every time -- only the byte select was wrong -- so
    // hex[(v >> i) & 0xF] returned hex[i & ~3] and every hex value printed
    // on the board came out with the low two bits of each nibble cleared.
    // Simulation could not show it: there the chain settles inside the
    // cycle regardless of its depth.
    //
    // Latching also shortens the response-cycle path to a flop output.
    logic [XLEN-1:0] load_shifted;
    logic [XLEN-1:0] load_data;
    // The native rsp carries a 64-bit doubleword (CPU_MEM_WIDTH) on the cache
    // build; the LSU's 32-bit word sits in the LOW half by convention --
    // the cache build lane-steers the response at the cache boundary
    // (top_module), and the 32-bit sim D-mem drives only the low half --
    // so slice it explicitly instead of truncating (EX3791) per field.
    wire [XLEN-1:0] load_word = lsu_rsp_wait.rdata[XLEN-1:0];
    assign load_shifted = load_word >> {mem_addr_q[1:0], 3'b000};
    always_comb begin
        unique case (de_i.mem_size)
            MS_B:
            load_data = de_i.mem_unsigned ?
                {24'b0, load_shifted[7:0]} : {{24{load_shifted[7]}}, load_shifted[7:0]};
            MS_H:
            load_data = de_i.mem_unsigned ?
                {16'b0, load_shifted[15:0]} : {{16{load_shifted[15]}}, load_shifted[15:0]};
            MS_W: load_data = load_word;
            default: load_data = load_word;
        endcase
    end

    // Writeback mux, ORDERED BY ARRIVAL TIME rather than by wb_src encoding --
    // the same restructure alu.sv already applies to result_o, for the same
    // reason and against the same measurement.
    //
    // alu_result is by far the latest of the four: it comes out of the DSP /
    // adder at ~18.6 ns on the 2026-09-01 48.965 MHz run, while pc_link and
    // csr_rdata_i are flop-derived and load_data comes off a BSRAM output that
    // settled at the start of the cycle. A flat `unique case (de_i.wb_src)`
    // builds a 4-way tree in which alu_result may sit behind TWO levels, and
    // that tree is on the design's worst path:
    //   regfile -> forward -> operand -> DSP -> alu_result -> wb_data ->
    //   forward back into de_bus / the regfile write port     (-0.423 ns)
    // Giving alu_result the final 2:1 and folding the three early candidates
    // behind it costs nothing and takes a level off the end of that path.
    logic [XLEN-1:0] wb_early;
    always_comb begin
        unique case (de_i.wb_src)
            WB_PC4:  wb_early = pc_link;
            WB_MEM:  wb_early = load_data;
            WB_CSR:  wb_early = csr_rdata_i;  // rd <- old CSR value
            default: wb_early = '0;  // includes WB_ALU (unused here)
        endcase
    end

    always_comb begin
        if (de_i.wb_src == WB_ALU) wb_data_o = alu_result;
        else wb_data_o = wb_early;
    end
    assign wb_addr_o = de_i.rd;

    // Result ready: single-cycle ALU/branch ops have alu_result_valid=1
    // (combinational ALU); DIV/REM on alu_result_valid; a load on mem_done
    // (rvalid); a posted store on its launch accept, which is now the same
    // cycle it was issued. Stores have reg_write=0 so their result_ready is
    // don't-care for the regfile write.
    logic result_ready;
    assign result_ready = is_mem_op ? mem_op_done : (is_csr_op ? csr_ready : alu_result_valid);

    // Two terms this used to carry are provably redundant, and both sat on the
    // path that ends in decode's forward mux:
    //   ~de_i.illegal    : decode zeroes reg_write when the encoding is
    //                      illegal, so de_i.reg_write already covers it.
    //   ~misaligned_trap : misaligned_trap is a term OF sync_trap_req, which
    //                      is a term OF trap_redirect_req, so
    //                      ~trap_redirect_req implies it.
    assign wb_en_o      = de_i.valid & de_i.reg_write & ~freeze & result_ready & ~trap_redirect_req;

    // =================================================================
    // CSR read-modify-write (Zicsr). The old CSR value is csr_rdata_i
    // (async read of de_i.csr_addr in the CSR file); rd <- old via
    // WB_CSR (above). The new value (csr_wdata_o) is the RMW result,
    // written to the CSR file when csr_wren_o pulses. The ALU result is
    // unused for CSR ops. CSRRS/CSRRC (and their immediate forms) do not
    // write the CSR when the source is zero; CSRRW always writes. A CSR
    // op is single-cycle (combinational ALU => result_ready=1).
    // =================================================================
    logic [XLEN-1:0] csr_src;
    logic [XLEN-1:0] csr_new;
    logic            csr_op_valid;
    logic            csr_we;

    // Source: rs1 for the register forms, zero-extended 5-bit zimm
    // (carried in de_i.imm by decode) for the immediate forms.
    always_comb begin
        if (de_i.csr_op inside {CSR_RWI, CSR_RSI, CSR_RCI}) csr_src = de_i.imm;
        else csr_src = de_i.rs1_data;
    end

    // Same retire predicate as the regfile writeback (result_ready=1 for
    // a CSR op). de_i.csr_wren is decode's "CSR op present" flag, already
    // squashed by illegal in decode.
    // Same two redundancies as wb_en_o above: decode squashes csr_wren on an
    // illegal encoding, and ~trap_redirect_req implies ~misaligned_trap.
    assign csr_op_valid = de_i.valid & de_i.csr_wren & ~freeze & result_ready & ~trap_redirect_req;

    always_comb begin
        unique case (de_i.csr_op)
            CSR_RW, CSR_RWI: begin
                csr_new = csr_src;
                csr_we  = csr_op_valid;  // CSRRW always writes
            end
            CSR_RS, CSR_RSI: begin
                csr_new = csr_rdata_i | csr_src;
                csr_we  = csr_op_valid & (csr_src != '0);  // no write if src==0
            end
            CSR_RC, CSR_RCI: begin
                csr_new = csr_rdata_i & ~csr_src;
                csr_we  = csr_op_valid & (csr_src != '0);  // no write if src==0
            end
            default: begin
                csr_new = '0;
                csr_we  = 1'b0;
            end
        endcase
    end

    assign csr_wdata_o = csr_new;
    assign csr_wren_o  = csr_we;

    // =================================================================
    // Memory interface (LSU). The effective address (Zilx indexed or
    // base+offset) is the ALU result. A store's data is pre-shifted into
    // the byte lanes selected by wstrb (the slave writes wdata[byte] when
    // wstrb[byte]). rready is asserted in EX_MEM_WAIT on a load so the
    // bridge forwards it to the AXI R channel and the read completes.
    // =================================================================
    // Byte strobes and the lane-aligned write data, computed from the live EA
    // and consumed by the bus in the SAME cycle they are produced — the rule
    // the forward path imposes on anything downstream of the EA. Both depend
    // on EA[1:0] only, which the EA tap delivers off the bottom of the adder's
    // carry chain rather than out of the ALU result mux.
    logic [XLEN_STRB_W-1:0] store_wstrb;
    always_comb begin
        // Byte strobes for a store of de_i.mem_size at the EA.
        unique case (de_i.mem_size)
            MS_B: store_wstrb = 4'b0001 << lsu_ea[1:0];
            MS_H: store_wstrb = 4'b0011 << {lsu_ea[1], 1'b0};
            MS_W: store_wstrb = 4'b1111;
            default: store_wstrb = 4'b1111;
        endcase
    end

    logic [XLEN-1:0] store_wdata;
    assign store_wdata = de_i.rs2_data << {lsu_ea[1:0], 3'b000};

    // Latch the request alongside the launch decision. For a D-mem op this is
    // only for the later-cycle consumers (load byte select, misaligned mtval,
    // response-side target select), which is why it sits in parallel with the
    // launch rather than ahead of it. For a peri op it IS the launch stage:
    // EX_MEM_LAUNCH drives the bus from these flops. The enable is
    // mem_req_pending, not an aligned term: a misaligned access does not
    // launch but still needs its EA in mtval. Re-latching the same value on
    // each cycle of a held D-mem launch is harmless.
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            mem_addr_q    <= '0;
            mem_wdata_q   <= '0;
            mem_wstrb_q   <= '0;
            mem_is_peri_q <= 1'b0;
        end else if (mem_req_pending) begin
            mem_addr_q    <= lsu_ea;
            mem_wdata_q   <= store_wdata;
            mem_wstrb_q   <= store_wstrb;
            mem_is_peri_q <= lsu_ea[PERI_ADDR_BIT];
        end
    end

`ifdef VERILATOR
    // The load byte select reads mem_addr_q when the response arrives, which
    // is only sound because no slave answers a read in the accept cycle:
    // every one of them registers rdata. Check it rather than trust it.
    always_ff @(posedge clk_i) begin
        if (rstn_i && dmem_load_hs && (LSU_LIVE_LOAD != 0)) begin
            assert (!mem_rsp_i.rvalid)
            else $fatal(1, "load response in the accept cycle: registered request is stale");
        end
        if (rstn_i && captured_hs && de_i.mem_read) begin
            assert (!lsu_rsp_wait.rvalid)
            else $fatal(1, "captured load response in the accept cycle: request is stale");
        end
    end

    // The redundancies the retire / writeback / redirect predicates above rely
    // on, checked rather than argued:
    //   1. an illegal instruction never commits and never writes back -- so
    //      dropping ~de_i.illegal from op_commits / wb_en_o / csr_op_valid is
    //      free (decode zeroes reg_write / csr_wren, and illegal implies
    //      exception implies sync_trap_req);
    //   2. misaligned_trap implies trap_redirect_req -- so ~misaligned_trap is
    //      free wherever ~trap_redirect_req is already a term;
    //   3. alu_result_valid is a constant 1 for a control-flow instruction --
    //      so dropping it from cf_resolving does not change when a branch
    //      resolves. Every control-flow opcode decodes to ALU_ADD; only a
    //      DIV/REM can be un-ready.
    always_ff @(posedge clk_i) begin
        if (rstn_i) begin
            assert (!(op_commits && de_i.illegal))
            else $fatal(1, "illegal instruction committed (pc=%08x)", de_i.pc);
            assert (!(wb_en_o && de_i.illegal))
            else $fatal(1, "illegal instruction wrote back (pc=%08x)", de_i.pc);
            assert (!(csr_op_valid && de_i.illegal))
            else $fatal(1, "illegal instruction wrote a CSR (pc=%08x)", de_i.pc);
            assert (!(misaligned_trap && !trap_redirect_req))
            else $fatal(1, "misaligned_trap without trap_redirect_req (pc=%08x)", de_i.pc);
            assert (!((de_i.branch_type != BR_NONE) && !alu_result_valid))
            else
                $fatal(
                    1,
                    "control-flow op with alu_result_valid=0 (pc=%08x alu_op=%0d)",
                    de_i.pc,
                    de_i.alu_op
                );
        end
    end

    // misaligned_trap is the ONE trap source not gated by ~freeze, and it
    // cannot be: with LSU_LIVE_LOAD=0 the same signal suppresses the captured
    // launch (captured_drive), so gating it on ~freeze would let a misaligned
    // store reach the bus while the pipe is frozen. Instead the trap relies on
    // freeze never being high in EX_MEM_TRAP, which holds for two independent
    // reasons: stall_i is tied 0 at the CPU top, and wfi_stall requires
    // wfi_halt_q, which mem_req_pending is gated against -- so a mem op
    // cannot even launch, let alone reach EX_MEM_TRAP, while the WFI halt is
    // active.
    //
    // If either premise ever changes -- someone drives stall_i -- the trap
    // would be raised during a freeze, and the EX_MEM_TRAP -> EX_IDLE
    // transition would then drop the state while sync_trap_req was still
    // suppressed, LOSING the exception. That is a silent wrong-execution bug,
    // so it is checked here rather than left to be discovered.
    always_ff @(posedge clk_i) begin
        if (rstn_i && (ex_state_q == EX_MEM_TRAP)) begin
            assert (!freeze)
            else
                $fatal(
                    1,
                    "EX_MEM_TRAP under freeze (stall_i=%0b wfi_stall=%0b): the misaligned trap at pc=%08x would be lost"
                        ,
                    stall_i,
                    wfi_stall,
                    de_i.pc
                );
        end
    end

    // EX_MEM_LAUNCH has no exit that does not go through de_i.valid: the
    // captured drive is gated on it, so if de_i were ever cleared under that
    // state the handshake could never complete and stall_o would hold the pipe
    // there forever. The argument that it cannot happen is that every flush
    // source (mispredict, sync trap, mret, interrupt) requires either a
    // control-flow op or EX_IDLE, and a captured mem op is neither -- an
    // argument about OTHER logic, so check it here.
    always_ff @(posedge clk_i) begin
        if (rstn_i && (ex_state_q inside {EX_MEM_LAUNCH, EX_MEM_WAIT, EX_MEM_TRAP})) begin
            assert (de_i.valid && is_mem_op)
            else
                $fatal(
                    1,
                    "%s with de_i.valid=%0b is_mem_op=%0b: LSU state has no owner",
                    ex_state_q.name(),
                    de_i.valid,
                    is_mem_op
                );
        end
    end

    // The premise that makes a live launch sound: de_i is held for as long as
    // the request is outstanding, so the latched address still equals the one
    // the EA computes. If this ever fails, the load byte select and the
    // request have parted company — the 2026-08-25 silicon bug, in reverse.
    always_ff @(posedge clk_i) begin
        if (rstn_i && (ex_state_q inside {EX_MEM_WAIT, EX_MEM_LAUNCH}) && de_i.valid) begin
            assert (mem_addr_q == lsu_ea)
            else
                $fatal(
                    1,
                    "%s: mem_addr_q %08x != lsu_ea %08x (de_i moved)",
                    ex_state_q.name(),
                    mem_addr_q,
                    lsu_ea
                );
        end
    end

    // The EA tap is only a shortcut if it agrees with the ALU result mux on
    // every op that uses it. It does iff a memory op's alu_op is always
    // ALU_ADD or ALU_LX (sel_adder=1, is_mul_op=0 -- see alu.sv ea_o). Check
    // that rather than trust the decoder to keep it true.
    always_ff @(posedge clk_i) begin
        if (rstn_i && de_i.valid && is_mem_op) begin
            assert (lsu_ea == alu_result)
            else
                $fatal(
                    1,
                    "mem op with lsu_ea %08x != alu_result %08x (pc=%08x alu_op=%0d)",
                    lsu_ea,
                    alu_result,
                    de_i.pc,
                    de_i.alu_op
                );
        end
    end
`endif

    // Harvard: steer the request to D-mem or the peri bridge. Both ports get
    // FULL ZERO DEFAULTS and exactly one branch drives, so no field of a
    // request ever comes from a different cycle than the rest of it.
    //
    // Note what that does and does not say. An UNSELECTED port is all-zero;
    // the SELECTED one keeps driving its registered payload with wvalid=0
    // between accesses, because mem_addr_q / mem_wdata_q / mem_wstrb_q only
    // change on mem_req_pending. That is the quiet state, not a leak: the
    // bus holds one op's values until the next op captures, so it neither
    // toggles per cycle nor ever mixes two ops' fields. The property that
    // matters is single-cycle-lifetime, not literal zero.
    //
    // This shape is not a style preference, it is the fix for a 2026-09-01 FPGA
    // failure. An intermediate version dropped the defaults and drove the
    // payload fields unconditionally -- addr live from the EA, wdata/wstrb
    // from the capture flops -- on the argument that a slave only acts on
    // wvalid, which is true of native_ram and of the AXI bridge in isolation.
    // The result passed every simulation (all oracles, both cosims, CoreMark to
    // 2000 iterations with the published CRC) and reboot-looped on silicon,
    // while origin/main with this structure ran. Two things it did that this
    // does not: it presented a request bundle whose address belonged to THIS
    // cycle and whose write data belonged to the PREVIOUS captured op, and it
    // toggled 32-bit address and data buses into a BSRAM and an AXI fabric on
    // every cycle of the program instead of only during an access.
    //
    // The rule: a bus is either fully registered or fully driven from one
    // cycle's values, and an idle bus is zero. "The receiver ignores that
    // field" is a property of today's receiver, not of the bus.
    always_comb begin
        mem_req_o.valid  = 1'b0;
        mem_req_o.we      = 1'b0;
        mem_req_o.addr    = '0;
        mem_req_o.wdata   = '0;
        mem_req_o.wstrb   = '0;
        mem_req_o.rready  = 1'b0;
        peri_req_o.valid = 1'b0;
        peri_req_o.we     = 1'b0;
        peri_req_o.addr   = '0;
        peri_req_o.wdata  = '0;
        peri_req_o.wstrb  = '0;
        peri_req_o.rready = 1'b0;

        // Exactly one launch branch, and the three states they belong to
        // (EX_IDLE / EX_MEM_LAUNCH) are mutually exclusive by construction.
        if ((LSU_LIVE_LOAD != 0) && dmem_load_drive) begin
            // Live D-mem LOAD launch. A load carries no write data and no
            // strobes, so those stay at their zero defaults rather than
            // exposing the previous captured op's flops.
            mem_req_o.valid = 1'b1;
            mem_req_o.addr   = lsu_ea;
        end else if (is_peri_rsp) begin
            // we is gated by the drive, matching mem_req_o.we below. The
            // bridge only reads we under wvalid, so an ungated version was
            // functionally safe -- but it left we=1 alongside wvalid=0 and a
            // registered address belonging to an older op, which is the exact
            // shape of mixed-lifetime bundle that cost a board bring-up here
            // (see the note above). One AND buys the invariant back.
            peri_req_o.valid = captured_drive;
            peri_req_o.we     = captured_drive & de_i.mem_write;
            peri_req_o.addr   = mem_addr_q;  // registered EA
            peri_req_o.wdata  = mem_wdata_q;
            peri_req_o.wstrb  = mem_wstrb_q;
        end else begin
            mem_req_o.valid = dmem_captured_drive;
            mem_req_o.we     = dmem_store_we;
            mem_req_o.addr   = mem_addr_q;  // registered EA
            mem_req_o.wdata  = mem_wdata_q;
            mem_req_o.wstrb  = mem_wstrb_q;
        end

        // Read-data phase: steered by the registered target bit, as the
        // outstanding read's address was latched a cycle or more ago.
        if ((ex_state_q == EX_MEM_WAIT) & de_i.mem_read) begin
            if (is_peri_rsp) peri_req_o.rready = 1'b1;
            else mem_req_o.rready = 1'b1;
        end
    end

    // =================================================================
    // Branch resolve + redirect (with prediction check)
    //
    // Execute remains the golden resolver: branch_taken / branch_target are
    // computed exactly as before. What changes is the redirect condition: a
    // control-flow instruction that was correctly predicted taken at decode
    // has already steered fetch, so execute does NOT redirect. execute
    // redirects only on MISPREDICT — the predicted direction/target disagrees
    // with the resolved one — or when there was no taken prediction (the legacy
    // taken-redirect path, which is the BP_EN=0 behaviour). A trap / mret /
    // interrupt always wins (trap_redirect_req) and suppresses both training
    // and the branch redirect.
    // =================================================================
    logic branch_taken;
    always_comb begin
        unique case (de_i.branch_type)
            BR_BEQ:  branch_taken = (de_i.rs1_data == de_i.rs2_data);
            BR_BNE:  branch_taken = (de_i.rs1_data != de_i.rs2_data);
            BR_BLT:  branch_taken = ($signed(de_i.rs1_data) < $signed(de_i.rs2_data));
            BR_BGE:  branch_taken = ($signed(de_i.rs1_data) >= $signed(de_i.rs2_data));
            BR_BLTU: branch_taken = (de_i.rs1_data < de_i.rs2_data);
            BR_BGEU: branch_taken = (de_i.rs1_data >= de_i.rs2_data);
            BR_JAL:  branch_taken = 1'b1;
            BR_JALR: branch_taken = 1'b1;
            default: branch_taken = 1'b0;  // BR_NONE
        endcase
    end

    // Resolved target (unchanged): pc+imm for JAL/branch, (rs1+imm)&~1 for JALR.
    wire [XLEN-1:0] branch_target = (de_i.branch_type == BR_JALR) ?
        (de_i.rs1_data + de_i.imm) & 32'hFFFF_FFFE : (de_i.pc + de_i.imm);

    // A control-flow instruction is resolving this cycle (eligible to train
    // and to mispredict-check). ~trap_redirect_req: a branch squashed by an
    // interrupt re-runs after mret instead of resolving now.
    // alu_result_valid is NOT a term here. It can only be 0 for a DIV/REM, and
    // every control-flow opcode decodes to ALU_ADD, so it is a constant 1
    // whenever branch_type != BR_NONE -- an extra input on the redirect path
    // (branch_valid_o gates fetch's pc_q / count_q / head_q and decode's hold
    // buffer) for no information. Asserted below.
    wire cf_resolving = de_i.valid & ~de_i.illegal &
        (de_i.branch_type != BR_NONE) & ~freeze & ~trap_redirect_req;

    // Predicted-taken flag carried from decode. pred_t=0 covers both "predicted
    // not-taken" and "no prediction" (BP_EN=0 / unpredicted JALR) — both reduce
    // to the legacy taken-redirect when the branch is actually taken.
    wire pred_t = de_i.pred_valid & de_i.pred_taken;

    // Mispredict: predicted taken but actually not-taken (redirect to pc_link,
    // the sequential fall-through); predicted taken with the wrong target
    // (redirect to the resolved target — e.g. a wrong RAS return); or not
    // predicted taken but actually taken (redirect to the resolved target —
    // the legacy path, including every JALR call/indirect which is never
    // target-predicted). A correct taken prediction produces no redirect.
    // The first and third disjuncts are (pred_t & ~taken) | (~pred_t & taken),
    // i.e. one XOR; the target check only adds anything when both are 1, where
    // the XOR is 0. So: direction disagrees, or direction agrees on TAKEN and
    // the target does not.
    wire dir_mispredict = pred_t ^ branch_taken;
    wire tgt_mispredict = pred_t & branch_taken & (branch_target != de_i.pred_target);
    wire mispredict = cf_resolving & (dir_mispredict | tgt_mispredict);

    // Combined fetch redirect: a mispredict OR a trap / mret / interrupt. The
    // trap unit's redirect_addr wins when a trap/mret/interrupt fires; else the
    // resolved target (taken) or the sequential pc_link (a predicted-taken
    // branch that turned out not-taken).
    assign branch_valid_o = mispredict | trap_redirect_req;
    assign branch_addr_o = trap_redirect_req ?
        trap_redirect_addr_i : (branch_taken ? branch_target : pc_link);
    assign flush_o = branch_valid_o;

    // =================================================================
    // Predictor training (at resolve). Drives the PHT sat-update + GHR shift
    // (conditional), the RAS push (call) / pop (return). Kind re-derived from
    // branch_type + rd + rs1. Trains on every resolved control-flow instr,
    // mispredicted or not — a misprediction is exactly the outcome to learn
    // from. Gated by cf_resolving (so a trap-squashed branch does not train).
    // =================================================================
    wire ex_is_cond = (de_i.branch_type inside {BR_BEQ, BR_BNE, BR_BLT, BR_BGE, BR_BLTU, BR_BGEU});
    wire ex_is_jalr = (de_i.branch_type == BR_JALR);
    wire ex_is_call = ((de_i.branch_type == BR_JAL) | ex_is_jalr) &
        (de_i.rd == 5'd1 || de_i.rd == 5'd5);
    wire ex_is_return = ex_is_jalr & (de_i.rs1_addr == 5'd1 || de_i.rs1_addr == 5'd5) &
        (de_i.rd == 5'd0);

    assign bp_train_o.valid     = cf_resolving;
    assign bp_train_o.cond      = cf_resolving & ex_is_cond;
    assign bp_train_o.call      = cf_resolving & ex_is_call;
    assign bp_train_o.ret       = cf_resolving & ex_is_return;
    assign bp_train_o.taken     = branch_taken;
    assign bp_train_o.pht_index = de_i.pred_pht_index;
    assign bp_train_o.push_pc   = pc_link;  // return address for a RAS push

    // =================================================================
    // E/M debug taps (retired instruction)
    // =================================================================
    logic [XLEN-1:0] ex_pc_q, ex_pc_d;
    logic [XLEN-1:0] ex_instr_q, ex_instr_d;
    logic ex_valid_q, ex_valid_d;

    // An op commits (retires -> ex_valid pulses, counts to minstret, logged
    // by the sim retire trace) when it retires normally, OR when an mret
    // retires. A sync trap does NOT commit: the faulting instruction is not
    // retired (it raised an exception before committing -- per the RISC-V
    // spec and Spike's --log-commits, which does not emit a commit line for
    // a trapping instruction). An interrupt does NOT commit either: the
    // squashed instruction re-runs after mret. WFI retires once (normal
    // path, result_ready=1 for the ALU-hint op) then halts. A misaligned
    // mem op is NOT in the normal path -- it raises a sync trap and does
    // not commit. The trap machinery itself (CSR trap-write bundle, fetch
    // redirect) is independent of ex_valid, so traps still fire correctly;
    // only the retire count is suppressed.
    //
    // Written as one OR of two terms rather than a 4-deep priority chain.
    // The chain's three suppressors are exactly the three terms of
    // trap_redirect_req, and mret is the one of them that DOES commit, so
    // "mret, or a normal retire with no redirect pending" is the same
    // function. ~de_i.illegal and ~misaligned_trap drop out with it: an
    // illegal instruction always raises sync_trap_req (decode sets exception
    // whenever it sets illegal, and an illegal op never leaves EX_IDLE), and
    // misaligned_trap is itself a term of sync_trap_req. Both are asserted
    // below under `ifdef VERILATOR rather than assumed.
    logic op_commits;
    assign op_commits    = mret_req | (de_i.valid & ~freeze & result_ready & ~trap_redirect_req);

    assign ex_pc_d       = op_commits ? de_i.pc : ex_pc_q;
    assign ex_instr_d    = op_commits ? de_i.instr : ex_instr_q;
    assign ex_valid_d    = op_commits;

    assign ex_pc_o       = ex_pc_q;
    assign ex_instr_o    = ex_instr_q;
    assign ex_valid_o    = ex_valid_q;

    // WFI halt next-state: set when WFI retires, cleared as soon as an
    // interrupt is pending+enabled. wfi_next_pc holds the return PC for the
    // wake.
    //
    // The clear condition is int_pending_i, NOT take_interrupt: take_interrupt
    // loses priority to sync_trap_req / mret_req (see the take_interrupt term
    // above), so clearing on it would leave wfi_halt_q stuck when the
    // instruction behind the WFI faults. A stuck wfi_halt_q re-raises
    // wfi_stall the moment trap entry clears mstatus.MIE (int_pending falls),
    // freezing the pipe inside the handler with no way to retire the CSR write
    // that would re-enable interrupts -- an unrecoverable deadlock. Clearing
    // on int_pending_i releases the halt whichever event actually wins.
    assign wfi_halt_d    = wfi_retire | (wfi_halt_q & ~int_pending_i);
    assign wfi_next_pc_d = wfi_retire ? pc_link : wfi_next_pc_q;

    // =================================================================
    // Sequential
    // =================================================================
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            ex_state_q    <= EX_IDLE;
            ex_pc_q       <= '0;
            ex_instr_q    <= '0;
            ex_valid_q    <= 1'b0;
            wfi_halt_q    <= 1'b0;
            wfi_next_pc_q <= '0;
        end else begin
            ex_state_q    <= ex_state_d;
            ex_pc_q       <= ex_pc_d;
            ex_instr_q    <= ex_instr_d;
            ex_valid_q    <= ex_valid_d;
            wfi_halt_q    <= wfi_halt_d;
            wfi_next_pc_q <= wfi_next_pc_d;
        end
    end

endmodule

`resetall

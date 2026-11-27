`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* Trap unit — exception / interrupt entry + mret return.
*
* Combinational. Lives in execute; the execute stage holds the triggers
* stable for the one cycle a trap / mret / interrupt is taken (sync traps
* only fire in EX_IDLE, never mid-mem), so no flops are needed here. It
* consumes CSR taps (mtvec / mepc / mstatus / mip / mie) and the trap
* trigger / cause / tval / pc the execute stage resolved, and produces:
*
*   - redirect_valid_o / redirect_addr_o : the fetch target (mtvec vector
*     for traps/interrupts, mepc for mret). Execute merges this with the
*     normal branch redirect (trap/mret win; mutually exclusive in time).
*   - the CSR trap-write bundle (mepc / mcause / mtval / mstatus), driven
*     into the CSR file's dedicated trap-write ports (separate from the
*     csrrw RMW port). Each writes a different register, so all four can
*     fire in one cycle.
*   - int_pending_o : a machine interrupt is pending and enabled
*     (mstatus.MIE & (MEIP&MEIE | MSIP&MSIE | MTIP&MTIE)). Read by execute
*     to decide take_interrupt and to wake WFI halt.
*   - int_cause_o  : the resolved interrupt cause. Priority MEI > MSI > MTI
*     (RISC-V privileged spec order). Execute uses it as the mcause for the
*     interrupt entry.
*
* Priority (resolved by the execute stage before calling in):
*   sync trap > interrupt > mret  (a faulting instruction traps; an
*   interrupt is taken instead of retiring the next normal instruction;
*   mret returns only when neither traps nor interrupts fire).
*
* mcause: synchronous exceptions carry mcause[31]=0 (code in [30:0]);
* interrupts carry mcause[31]=1 (MSI code=3, MTI code=7, MEI code=11). The
* execute stage
* passes the fully-resolved cause_i (MCAUSE_* from the package) for sync
* traps, and takes int_cause_o for interrupts; this module forwards
* cause_i unchanged.
*
* mtvec MODE (mtvec[1:0]):
*   DIRECT   (00) -> all traps/interrupts to BASE = {mtvec[31:2],2'b00}.
*   VECTORED (01) -> interrupts to BASE + 4*code; sync exceptions to BASE.
*
* mstatus field semantics (RV32, linux-soc: M and U implemented, no S):
*   trap entry (sync OR interrupt): MPIE<=MIE, MIE<=0, MPP<=current mode,
*     mode<=M.
*   mret: MIE<=MPIE, MPIE<=1, mode<=MPP, MPP<=00 (U).
*
* MPP encoding is 2'b00=U, 2'b01=S, 2'b11=M; csr_regfile keeps it WARL
* {00, 11}. The current mode is one flop in csr_regfile (priv_m).
*
* Naming: ports *_i/_o; internals no prefix.
*/

module trap_unit (
    // CSR taps (combinational reads from csr_regfile).
    input wire [XLEN-1:0] mtvec_i,
    input wire [XLEN-1:0] mepc_i,
    input wire [XLEN-1:0] mstatus_i,
    input wire [XLEN-1:0] mip_i,
    input wire [XLEN-1:0] mie_i,

    // Current privilege, 1 = M, 0 = U (linux-soc has no S).
    input wire priv_m_i,

    // Triggers, resolved by execute (mutually exclusive this cycle).
    input wire take_trap_i,       // sync exception entry
    input wire take_interrupt_i,  // async interrupt entry
    input wire mret_i,            // mret return

    // Trap payload (execute-resolved). cause_i is the mcause value;
    // tval_i the mtval value; pc_i the mepc value (faulting instr for a
    // sync trap, the suppressed instr's pc for an interrupt, wfi.pc+4 for
    // a WFI-wake interrupt).
    input wire [XLEN-1:0] cause_i,
    input wire [XLEN-1:0] tval_i,
    input wire [XLEN-1:0] pc_i,

    // Redirect to fetch.
    output wire            redirect_valid_o,
    output wire [XLEN-1:0] redirect_addr_o,

    // CSR trap-write bundle (one cycle, each writes a distinct CSR).
    output wire            csr_we_mepc_o,
    output wire [XLEN-1:0] csr_d_mepc_o,
    output wire            csr_we_mcause_o,
    output wire [XLEN-1:0] csr_d_mcause_o,
    output wire            csr_we_mtval_o,
    output wire [XLEN-1:0] csr_d_mtval_o,
    output wire            csr_we_mstatus_o,
    output wire [XLEN-1:0] csr_d_mstatus_o,

    // Pending+enabled machine interrupt of any kind — external, software or
    // timer (for execute / WFI wake).
    output wire int_pending_o,

    // Resolved interrupt cause (MEI > MSI > MTI priority, spec order — all
    // three are wired). Execute uses this as the mcause for the interrupt
    // entry.
    output wire [XLEN-1:0] int_cause_o,

    // WFI wake: some interrupt is pending AND locally enabled (mip & mie),
    // whatever mstatus.MIE says. The priv. spec resumes a WFI on exactly
    // this; the interrupt itself is only TAKEN when int_pending_o is also
    // set. Linux's idle loop executes wfi with MIE = 0 and re-enables
    // interrupts afterwards, so gating the wake on MIE hangs it for good.
    output wire wfi_wake_o,

    // Privilege update: trap entry -> M; mret -> the mode in mstatus.MPP.
    output wire priv_we_o,
    output wire priv_d_o
);

    // -----------------------------------------------------------------
    // Interrupt pending: MIE globally enabled AND at least one source is
    // pending+locally-enabled. Priority MEI > MSI > MTI (spec order).
    // -----------------------------------------------------------------
    wire mie_m = mstatus_i[MSTATUS_MIE_BIT];
    wire msip_p = mip_i[3];  // MSIP
    wire msie_e = mie_i[3];  // MSIE
    wire mtip_p = mip_i[7];  // MTIP
    wire mtie_e = mie_i[7];  // MTIE
    wire meip_p = mip_i[11];  // MEIP
    wire meie_e = mie_i[11];  // MEIE

    wire msi_pending = msip_p & msie_e;
    wire mti_pending = mtip_p & mtie_e;
    wire mei_pending = meip_p & meie_e;

    // In U-mode, M-level interrupts are taken whatever mstatus.MIE says
    // (priv. spec: interrupts for a higher privilege are always globally
    // enabled while running at a lower one).
    assign int_pending_o = (mie_m | ~priv_m_i) & (mei_pending | msi_pending | mti_pending);
    assign wfi_wake_o    = mei_pending | msi_pending | mti_pending;
    assign int_cause_o   = mei_pending ? MCAUSE_MEI : msi_pending ? MCAUSE_MSI : MCAUSE_MTI;

    // -----------------------------------------------------------------
    // Redirect address.
    //   mret      -> mepc
    //   trap/int  -> BASE = {mtvec[31:2],2'b00}; VECTORED interrupts add
    //                4*code (sync exceptions always go to BASE).
    // -----------------------------------------------------------------
    wire [XLEN-1:0] mtvec_base = {mtvec_i[31:2], 2'b00};
    wire            is_vectored = (mtvec_i[1:0] == MTVEC_VECTORED);

    // 4 * interrupt code (cause_i[5:0] for an interrupt; the code lives in
    // mcause[30:0], MS interrupt code = 3). Only added for an interrupt in
    // vectored mode.
    wire [XLEN-1:0] int_offset = {26'b0, cause_i[5:0], 2'b00};
    wire [XLEN-1:0] vec_target = mtvec_base + int_offset;

    wire [XLEN-1:0] trap_target = (take_interrupt_i & is_vectored) ? vec_target : mtvec_base;

    assign redirect_valid_o = take_trap_i | take_interrupt_i | mret_i;
    assign redirect_addr_o  = mret_i ? mepc_i : trap_target;

    // -----------------------------------------------------------------
    // CSR trap-write bundle. mepc / mcause / mtval are written on trap
    // entry AND interrupt entry (not on mret — mret only restores mstatus
    // and redirects to mepc). mstatus is written on all three.
    // -----------------------------------------------------------------
    wire entry = take_trap_i | take_interrupt_i;

    assign csr_we_mepc_o    = entry;
    assign csr_d_mepc_o     = pc_i;

    assign csr_we_mcause_o  = entry;
    assign csr_d_mcause_o   = cause_i;

    assign csr_we_mtval_o   = entry;
    assign csr_d_mtval_o    = tval_i;

    assign csr_we_mstatus_o = take_trap_i | take_interrupt_i | mret_i;

    // -----------------------------------------------------------------
    // mstatus field update.
    //   entry: MPIE<=MIE, MIE<=0, MPP<=11 (M).
    //   mret : MIE<=MPIE, MPIE<=1, MPP<=11 (M).
    // -----------------------------------------------------------------
    logic [XLEN-1:0] mstatus_new;

    always_comb begin
        mstatus_new = mstatus_i;
        if (entry) begin
            mstatus_new[MSTATUS_MPIE_BIT]              = mstatus_i[MSTATUS_MIE_BIT];
            mstatus_new[MSTATUS_MIE_BIT]               = 1'b0;
            mstatus_new[MSTATUS_MPP_HI:MSTATUS_MPP_LO] = priv_m_i ? MSTATUS_MPP_M : MSTATUS_MPP_U;
        end else if (mret_i) begin
            mstatus_new[MSTATUS_MIE_BIT]               = mstatus_i[MSTATUS_MPIE_BIT];
            mstatus_new[MSTATUS_MPIE_BIT]              = 1'b1;
            // MPP <- least-privileged supported mode (U).
            mstatus_new[MSTATUS_MPP_HI:MSTATUS_MPP_LO] = MSTATUS_MPP_U;
        end
    end

    assign csr_d_mstatus_o = mstatus_new;

    assign priv_we_o       = entry | mret_i;
    assign priv_d_o        = entry | (mstatus_i[MSTATUS_MPP_HI:MSTATUS_MPP_LO] == MSTATUS_MPP_M);

endmodule

`resetall

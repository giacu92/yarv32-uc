/*
 * Trap / exception / interrupt oracle (rv32imac_zicsr_zifencei, M-mode).
 *
 * Exercises the trap machinery end-to-end and self-checks via D-mem
 * markers, then writes a pass/fail sentinel:
 *
 *   1. ecall        -> sync trap, mcause=11 (ECALL_M). Handler advances
 *                     mepc +4 and returns to ecall+4.
 *   2. lw at EA=1   -> load-address-misaligned, mcause=4. Handler skips
 *                     the faulting lw (mepc+4) and returns past it.
 *   3. .word 0      -> illegal instruction, mcause=2. Handler skips it.
 *   4. MMIO write   -> sets MSIP (mip.MSIP) via the peri slave at
 *      + wfi          0x1001_0000 (CLINT msip, linux-soc), then WFI halts. The pending+enabled
 *                     machine software interrupt wakes WFI (mepc = wfi+4),
 *                     handler clears MSIP, mret returns past WFI.
 *
 * mtvec is direct mode (MODE=0): every trap / interrupt lands at
 * trap_handler, which branches on mcause[31] (interrupt vs sync) and on
 * the cause code. Sync traps advance mepc by the faulting instr size (4
 * here; all faulting instrs are 4-byte) so they don't re-trap on return.
 *
 * Markers (D-mem): the handler stores each cause code at 0x2000+cause*4
 * (sync) and 0x44 at 0x2040 (interrupt), so each trap leaves a distinct
 * surviving witness. main() self-checks all markers and writes 0x600D at
 * D-mem 0x2000 on pass, 0xBAD on any mismatch (probe word 0x800).
 *
 * Built with -march=rv32imac_zicsr_zifencei so the toolchain emits CSR /
 * mret / wfi / fence.i. The core implements all of these.
 */

    .section .text.init, "ax"
    .global _start
_start:
    li      sp, 0x4000           /* top of the 16 KiB D-mem; stack grows down */

    /* mtvec = trap_handler, direct mode (MODE[1:0] = 00). */
    la      t0, trap_handler
    csrw    mtvec, t0

    /* Enable the machine software interrupt and global MIE.
     * Sync traps are not masked by MIE, so tests 1-3 trap regardless;
     * MIE is needed for the test-4 interrupt. */
    li      t1, 0x8
    csrs    mie, t1             /* MSIE */
    csrs    mstatus, t1          /* MIE */

    /* ---- test 1: ecall ---- */
    ecall
    /* handler returns here (mepc advanced +4) */

    /* ---- test 2: misaligned load (EA = 1, lw needs 4-byte align) ---- */
    lw      t0, 1(x0)           /* traps; skipped by handler */
    /* handler returns here */

    /* ---- test 3: illegal instruction (opcode 0) ---- */
    .word   0x00000000          /* traps; skipped by handler */
    /* handler returns here */

    /* ---- test 4: MSIP self-write + WFI wake ---- */
    li      t0, 0x10010000      /* CLINT msip[0] (CLINT_BASE + 0), linux-soc */
    li      t1, 1
    sw      t1, 0(t0)           /* set MSIP -> mip.MSIP */
    wfi                        /* halt; interrupt wakes -> handler -> mret -> here
                                 * (handler's int_path already clears MSIP; no
                                 * redundant clear here -- the handler's
                                 * csrr t0,mcause clobbers t0, so a store
                                 * through t0 here would target a bad addr) */

    /* ---- self-check the markers ---- */
    li      t0, 0x2008
    lw      t1, 0(t0)
    li      t2, 2               /* illegal cause */
    bne     t1, t2, bad

    li      t0, 0x2010
    lw      t1, 0(t0)
    li      t2, 4               /* load-misaligned cause */
    bne     t1, t2, bad

    li      t0, 0x202C
    lw      t1, 0(t0)
    li      t2, 11              /* ecall cause */
    bne     t1, t2, bad

    li      t0, 0x2040
    lw      t1, 0(t0)
    li      t2, 0x44            /* interrupt marker */
    bne     t1, t2, bad

    li      t0, 0x2000
    li      t1, 0x600D
    sw      t1, 0(t0)          /* PASS sentinel */
    j       park
bad:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
park:
    j       park

    /* ----------------------------------------------------------------- */
    .align 2
trap_handler:
    csrr    t0, mcause
    li      t2, 0x80000000
    and     t1, t0, t2
    bnez    t1, int_path

    /* sync trap: skip the faulting instr (mepc += 4), store cause. */
    csrr    t1, mepc
    addi    t1, t1, 4
    csrw    mepc, t1
    andi    t0, t0, 0x1ff       /* cause code */
    slli    t2, t0, 2          /* *4 */
    li      t3, 0x2000
    add     t3, t3, t2
    sw      t0, 0(t3)          /* D-mem[0x2000 + cause*4] = cause */
    mret

int_path:
    /* machine software interrupt: clear MSIP, mark, return. mepc is
     * already wfi+4 (set by the trap unit's WFI-wake path) -- do NOT
     * advance it. */
    li      t4, 0x10010000
    sw      x0, 0(t4)          /* clear MSIP */
    li      t3, 0x2000
    li      t5, 0x44
    sw      t5, 0x40(t3)       /* D-mem[0x2040] = 0x44 */
    mret
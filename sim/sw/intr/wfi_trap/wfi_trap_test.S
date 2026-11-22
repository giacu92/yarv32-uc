/*
 * WFI-wake arbitration oracle (rv32imac_zicsr_zifencei, M-mode).
 *
 * Regression test for the WFI-halt deadlock: wfi_halt_q used to clear only
 * on take_interrupt, but take_interrupt loses priority to a synchronous
 * trap. Put a faulting instruction directly behind the wfi and the old RTL
 * wedged permanently:
 *
 *   wfi halts (wfi_halt_q=1, wfi_stall freezes the pipe)
 *     -> timer interrupt goes pending, int_pending=1, freeze drops
 *     -> the illegal word behind the wfi raises a sync trap, which WINS
 *        arbitration, so take_interrupt=0 and wfi_halt_q stays set
 *     -> trap entry clears mstatus.MIE, so int_pending falls back to 0
 *     -> wfi_stall re-asserts and freezes the pipe INSIDE the handler,
 *        which can therefore never retire the CSR write that would
 *        re-enable interrupts. Unrecoverable without a reset.
 *
 * The fix clears wfi_halt_q on int_pending rather than on take_interrupt,
 * so the halt is released whichever event actually wins. sim_main.cpp also
 * watches for the stuck state directly (WFI-HALT FAIL / WFI-halt check: OK).
 *
 * Sequence:
 *   1. Set sp / mtvec (direct mode), enable mie.MTIE + mstatus.MIE.
 *   2. Arm the timer: mtimecmp = 400 (mtime counts from 0 every cycle;
 *      400 is comfortably past this setup code, so MTIP stays low until
 *      the wfi is actually reached). mtimecmp_lo @ 0x1001_4000 (SiFive CLINT, linux-soc),
 *      mtimecmp_hi @ 0x1001_4004.
 *   3. wfi -- halts until MTIP wakes it.
 *   4. The word at wfi+4 is an illegal encoding (0x0000007f). On the wake
 *      it takes an illegal-instruction trap (mcause=2) with mepc = wfi+4
 *      (the trap unit's WFI-wake mepc, which is exactly this word's PC).
 *      The handler records marker 2 at D-mem 0x2040 and returns to wfi+8.
 *   5. MTIP is still pending and mret restored mstatus.MIE, so the timer
 *      interrupt is then taken at the next retire boundary (mcause=
 *      0x8000_0007). The handler records marker 7 at D-mem 0x2044 and
 *      disarms the timer (mtimecmp = all ones).
 *
 * main() self-checks BOTH markers and writes 0x600D at D-mem 0x2000 (pass,
 * probe word 0x800) or 0xBAD. Against the buggy RTL it never gets there:
 * the run freezes in the handler and sim_main reports WFI-HALT FAIL.
 *
 * Built with -march=rv32imac_zicsr_zifencei so the toolchain emits CSR /
 * mret / wfi.
 */

    .section .text.init, "ax"
    .global _start
_start:
    li      sp, 0x4000           /* top of the 16 KiB D-mem; stack grows down */

    /* mtvec = trap_handler, direct mode (MODE[1:0] = 00). */
    la      t0, trap_handler
    csrw    mtvec, t0

    /* Clear both markers so a skipped trap cannot pass by accident. */
    li      t0, 0x2040
    sw      x0, 0(t0)
    sw      x0, 4(t0)

    /* Enable the machine timer interrupt (mie.MTIE, bit 7) and global
     * MIE (mstatus bit 3). */
    li      t1, 0x80
    csrs    mie, t1              /* MTIE */
    li      t1, 0x8
    csrs    mstatus, t1          /* MIE */

    /* ---- arm the timer: mtimecmp = 400 ----
     * Write the high half first (0) so mtimecmp stays huge until the low
     * half lands -- no transient MTIP. mtimecmp_hi @ +0xC, lo @ +0x8. */
    li      t0, 0x10014000       /* mtimecmp (CLINT_BASE + 0x4000) */
    sw      x0, 4(t0)            /* mtimecmp_hi = 0 */
    li      t1, 400
    sw      t1, 0(t0)            /* mtimecmp_lo = 400 -> mtimecmp = 400 */

    /* A long multi-cycle op before the wfi, so fetch runs ahead and fills
     * F/D + the skid buffer. That matters: the cycle the wfi retires,
     * decode must latch the illegal word behind it into D/E. Without the
     * run-ahead, D/E holds a bubble for the whole halt, the sync trap
     * cannot race the interrupt at the wake, and the arbitration case this
     * oracle exists to test is never reached. */
    li      t1, 3
    li      t2, 77
    div     t2, t2, t1           /* 32-cycle restoring divide */

    /* ---- wfi: halt until the timer interrupt wakes ---- */
    wfi
    /* The wake lands on THIS word, not on a benign instruction: an illegal
     * encoding, so the sync trap races the interrupt for arbitration. */
    .word   0x0000007f           /* illegal instruction (mcause = 2) */
    /* the illegal handler advances mepc by 4, so it returns here */

    /* ---- self-check both markers ---- */
    li      t0, 0x2040
    lw      t1, 0(t0)
    li      t2, 2                /* illegal-instruction cause code */
    bne     t1, t2, bad

    lw      t1, 4(t0)
    li      t2, 7                /* timer-interrupt cause code */
    bne     t1, t2, bad

    li      t0, 0x2000
    li      t1, 0x600D
    sw      t1, 0(t0)            /* PASS sentinel */
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

    /* ---- synchronous trap: the illegal word behind the wfi ----
     * mepc = wfi+4 = the faulting word's PC. Record the cause code and
     * step over it (the illegal encoding is a full 4-byte word). */
    andi    t0, t0, 0x1ff        /* cause code = 2 */
    li      t3, 0x2040
    sw      t0, 0(t3)            /* D-mem[0x2040] = 2 */
    csrr    t1, mepc
    addi    t1, t1, 4
    csrw    mepc, t1
    mret

int_path:
    /* ---- machine timer interrupt ----
     * Taken at a retire boundary after the illegal trap returned, so mepc
     * is the suppressed instruction's PC -- do NOT advance it. */
    andi    t0, t0, 0x1ff        /* cause code = 7 */
    li      t3, 0x2040
    sw      t0, 4(t3)            /* D-mem[0x2044] = 7 */

    /* Clear MTIP: mtimecmp = 0xFFFFFFFF_FFFFFFFF (mtime stays small, so
     * mtime < mtimecmp -> MTIP=0). Write hi first, then lo. */
    li      t4, 0x10014000
    li      t5, -1
    sw      t5, 4(t4)            /* mtimecmp_hi = 0xFFFFFFFF */
    sw      t5, 0(t4)            /* mtimecmp_lo = 0xFFFFFFFF */
    mret

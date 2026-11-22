/*
 * Timer-interrupt oracle (rv32imac_zicsr_zifencei, M-mode).
 *
 * Exercises the machine timer interrupt (mcause=0x8000_0007, mip.MTIP)
 * end-to-end, sourced by the CLINT timer MMIO slave (clint_timer at peri
 * 0x1000_1000), and self-checks via a D-mem marker, then writes a
 * pass/fail sentinel:
 *
 *   1. Set sp / mtvec (direct mode), enable mie.MTIE + mstatus.MIE.
 *   2. Arm the timer: write mtimecmp = 200 (mtime counts from 0 every
 *      cycle, so mtime >= 200 fires ~cycle 200; margin past the ~30-cycle
 *      setup so MTIP stays low until armed). mtimecmp_lo @ 0x1001_4000 (SiFive CLINT, linux-soc),
 *      mtimecmp_hi @ 0x1001_4004.
 *   3. wfi -- halts. The pending+enabled machine timer interrupt wakes
 *      WFI (mepc = wfi+4, set by the trap unit's WFI-wake path).
 *   4. Handler: stores the masked mcause code (7) as a marker at D-mem
 *      0x2040, clears MTIP by writing mtimecmp = 0xFFFFFFFF_FFFFFFFF,
 *      mret (does NOT advance mepc -- already wfi+4).
 *
 * main() self-checks the marker == 7 and writes 0x600D at D-mem 0x2000
 * (pass, probe word 0x800) or 0xBAD.
 *
 * Built with -march=rv32imac_zicsr_zifencei so the toolchain emits CSR /
 * mret / wfi. The core implements all of these. The CLINT timer + peri
 * 1->2 xbar are wired in top_module / sim_top.
 */

    .section .text.init, "ax"
    .global _start
_start:
    li      sp, 0x4000           /* top of the 16 KiB D-mem; stack grows down */

    /* mtvec = trap_handler, direct mode (MODE[1:0] = 00). */
    la      t0, trap_handler
    csrw    mtvec, t0

    /* Enable the machine timer interrupt (mie.MTIE, bit 7) and global
     * MIE (mstatus bit 3). */
    li      t1, 0x80
    csrs    mie, t1             /* MTIE */
    li      t1, 0x8
    csrs    mstatus, t1          /* MIE */

    /* ---- arm the timer: mtimecmp = 200 ---- */
    /* Write the high half first (0), so mtimecmp is huge until the low
     * half lands -- no transient MTIP. mtimecmp_hi @ +0xC, lo @ +0x8. */
    li      t0, 0x10014000      /* mtimecmp (CLINT_BASE + 0x4000) */
    sw      x0, 4(t0)           /* mtimecmp_hi = 0 */
    li      t1, 200
    sw      t1, 0(t0)           /* mtimecmp_lo = 200  -> mtimecmp = 200 */

    /* ---- wfi: halt until the timer interrupt wakes ---- */
    wfi
    /* handler returns here (mepc = wfi+4) */

    /* ---- self-check the marker ---- */
    li      t0, 0x2040
    lw      t1, 0(t0)
    li      t2, 7               /* timer interrupt cause code */
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

    /* Sync trap (none expected here): skip and return. */
    csrr    t1, mepc
    addi    t1, t1, 4
    csrw    mepc, t1
    mret

int_path:
    /* Machine timer interrupt. mepc is already wfi+4 (trap unit's WFI-wake
     * path) -- do NOT advance it. Store the masked cause code as a marker. */
    andi    t0, t0, 0x1ff       /* cause code = 7 */
    li      t3, 0x2040
    sw      t0, 0(t3)          /* D-mem[0x2040] = 7 */

    /* Clear MTIP: mtimecmp = 0xFFFFFFFF_FFFFFFFF (mtime stays tiny, so
     * mtime < mtimecmp -> MTIP=0). Write hi first then lo. */
    li      t4, 0x10014000
    li      t5, -1
    sw      t5, 4(t4)          /* mtimecmp_hi = 0xFFFFFFFF */
    sw      t5, 0(t4)          /* mtimecmp_lo = 0xFFFFFFFF */
    mret
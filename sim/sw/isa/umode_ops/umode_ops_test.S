/* U-mode oracle -- M/U privilege transitions (linux-soc, M + U, no S).
 *
 * Each check loads its number into s11 first, so a failure leaves it at
 * D-mem 0x2004 next to the 0xBAD marker.
 *
 * Entering U: MPP cleared, mepc = U code, mret. Every trap goes to
 * trap_handler, which checks mcause against s10 (the cause the test is
 * waiting for), saves mstatus/mepc/mtval in s7/s6/s9 and jumps to s8 IN
 * M-MODE (no mret), so each U fragment is one trap long.
 *
 * Checks:
 *   1  mstatus.MPP reads 11 out of reset.
 *   2  csrw mstatus with MPP = 01 (S, unsupported) reads back 11 (WARL).
 *   3  csrw mstatus with MPP = 00 reads back 00.
 *   4  ecall in U -> mcause 8; trap entry recorded MPP = 00 (came from U);
 *      mepc = the ecall.
 *   5  an M-to-M mret leaves MPP = 00 (U), the least-privileged mode.
 *   6  csrr mscratch in U -> mcause 2, mtval = the instruction word.
 *   7  csrr cycle in U -> mcause 2 (mcounteren is read-only zero).
 *   8  mret in U -> mcause 2.
 *   9  wfi in U -> mcause 2.
 *  10  ecall in M -> mcause 11, MPP = 11.
 *  11  M-mode with mstatus.MIE = 0 and MSIP pending + MSIE: no interrupt.
 *  12  same state, entering U: the interrupt is taken at once,
 *      mcause 0x8000_0003, MPP = 00.
 *  13  U code runs ordinary instructions and loads/stores (the result of
 *      the U fragment is checked back in M).
 *  14  wfi in M with mstatus.MIE = 0 and MSIP pending + MSIE resumes and
 *      takes no trap (WFI wakes on mip & mie, whatever MIE says). Linux's
 *      idle loop depends on it; the pre-fix core halted forever.
 */

    .section .text.init, "ax"

    .equ    MSIP_ADDR, 0x10010000

    .macro  CHECK n, reg, expect
    li      s11, \n
    li      t1, \expect
    bne     \reg, t1, bad
    .endm

    /* enter U at label \ucode expecting cause \cause, continue at \cont */
    .macro  TO_U ucode, cause, cont
    la      s8, \cont
    li      s10, \cause
    li      t0, 0x1800
    csrc    mstatus, t0
    la      t0, \ucode
    csrw    mepc, t0
    mret
    .endm

    .globl _start
_start:
    la      t0, trap_handler
    csrw    mtvec, t0
    li      s10, -1                 /* no trap expected */

    /* 1 */
    csrr    a0, mstatus
    li      t0, 0x1800
    and     a0, a0, t0
    CHECK   1, a0, 0x1800
    /* 2 */
    li      t0, 0x1800
    csrc    mstatus, t0
    li      t0, 0x0800
    csrs    mstatus, t0
    csrr    a0, mstatus
    li      t0, 0x1800
    and     a0, a0, t0
    CHECK   2, a0, 0x1800
    /* 3 */
    li      t0, 0x1800
    csrc    mstatus, t0
    csrr    a0, mstatus
    and     a0, a0, t0
    CHECK   3, a0, 0

    /* 4 + 13: ecall from U, after some U-mode work */
    li      s5, 0
    TO_U    u_ecall, 8, c4
c4:
    li      t0, 0x1800
    and     a0, s7, t0
    CHECK   4, a0, 0
    la      t0, u_ecall_insn
    mv      a0, s6
    bne     a0, t0, bad
    /* 13: U fragment computed 3*7 and stored it */
    li      t0, 0x2010
    lw      a0, 0(t0)
    CHECK   13, a0, 21
    CHECK   13, s5, 21

    /* 5: mret from M to M sets MPP to U afterwards */
    li      t0, 0x1800
    csrs    mstatus, t0             /* MPP = M: mret stays in M */
    la      t0, c5
    csrw    mepc, t0
    mret
c5:
    csrr    a0, mstatus
    li      t0, 0x1800
    and     a0, a0, t0
    CHECK   5, a0, 0

    /* 6 */
    TO_U    u_csr, 2, c6
c6:
    mv      a0, s9                  /* mtval = the instruction word */
    CHECK   6, a0, 0x34002573       /* csrr a0, mscratch (a data load
                                       cannot read .text on Harvard) */
    /* 7 */
    TO_U    u_cycle, 2, c7
c7:
    /* 8 */
    TO_U    u_mret, 2, c8
c8:
    /* 9 */
    TO_U    u_wfi, 2, c9
c9:
    /* 10: ecall from M */
    la      s8, c10
    li      s10, 11
    ecall
c10:
    li      t0, 0x1800
    and     a0, s7, t0
    CHECK   10, a0, 0x1800

    /* 11: MSIP pending + MSIE, MIE = 0, in M: nothing happens */
    li      s10, -1
    csrci   mstatus, 8
    li      t0, 8
    csrw    mie, t0
    li      t0, MSIP_ADDR
    li      t1, 1
    sw      t1, 0(t0)
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    li      s11, 11                 /* reaching here without a trap = pass */
    /* 12: enter U, interrupt taken at once */
    TO_U    u_spin, 0x80000003, c12
c12:
    li      t0, MSIP_ADDR
    sw      zero, 0(t0)
    csrw    mie, zero
    li      t0, 0x1800
    and     a0, s7, t0
    CHECK   12, a0, 0

    /* 14: wfi with mstatus.MIE = 0, MSIP pending + MSIE: wakes, no trap */
    li      s10, -1
    csrci   mstatus, 8
    li      t0, 8
    csrw    mie, t0
    li      t0, MSIP_ADDR
    li      t1, 1
    sw      t1, 0(t0)
    wfi
    li      s11, 14                 /* reached = woke without a trap */
    li      t0, MSIP_ADDR
    sw      zero, 0(t0)
    csrw    mie, zero

    li      t0, 0x2000
    li      t1, 0x600D
    sw      t1, 0(t0)
pass:
    j       pass

bad:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
    sw      s11, 4(t0)
    sw      a0, 8(t0)
    sw      a1, 12(t0)
badpark:
    j       badpark

    /* ---- U-mode fragments ---- */
u_ecall:
    li      a2, 3
    li      a3, 7
    mul     s5, a2, a3
    li      t0, 0x2010
    sw      s5, 0(t0)
u_ecall_insn:
    ecall
1:  j       1b
u_csr:
u_csr_insn:
    csrr    a0, mscratch
1:  j       1b
u_cycle:
    csrr    a0, cycle
1:  j       1b
u_mret:
    mret
1:  j       1b
u_wfi:
    wfi
1:  j       1b
u_spin:
    j       u_spin

    .align  2
trap_handler:
    csrr    t1, mcause
    li      s11, 0xEEEE
    bne     t1, s10, unexpected
    csrr    s7, mstatus
    csrr    s6, mepc
    csrr    s9, mtval
    li      s10, -1
    jr      s8
unexpected:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
    sw      s11, 4(t0)
    sw      t1, 8(t0)
    csrr    t1, mcause
    sw      t1, 12(t0)
th_park:
    j       th_park

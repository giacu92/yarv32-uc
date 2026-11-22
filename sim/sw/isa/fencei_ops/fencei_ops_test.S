/* fence.i oracle -- the core-side flush + refetch.
 *
 * 1  fence.i retires exactly once: instret across it advances by 2 (the
 *    fence.i and the csrr), and the instruction after it executes once.
 * 2  fence.i at a halfword-aligned pc (after a compressed instruction), with
 *    compressed instructions after it: pc+4 is refetched correctly.
 * 3  fence.i inside a loop with a predicted-taken backward branch, 64
 *    iterations: the redirect from fence.i and the predictor coexist.
 * 4  fence.i immediately after a taken jal (predicted redirect in flight).
 * 5  a machine software interrupt pending and enabled across a fence.i is
 *    taken exactly once, and execution resumes after it.
 */
    .section .text.init, "ax"

    .macro  CHECK n, reg, expect
    li      s11, \n
    li      t1, \expect
    bne     \reg, t1, bad
    .endm

    .globl _start
_start:
    li      s9, 0                  /* interrupt count */
    la      t0, trap_handler
    csrw    mtvec, t0

    /* ---- 1: retires once ---- */
    li      a1, 0
    csrr    a0, instret
    fence.i
    addi    a1, a1, 1
    csrr    a2, instret
    sub     a2, a2, a0
    CHECK   1, a2, 3              /* csrr(first) + fence.i + addi */
    CHECK   1, a1, 1

    /* ---- 2: halfword-aligned fence.i, compressed neighbours ---- */
    .option push
    .option rvc
    li      a1, 0
    c.nop                          /* with c.li before it: fence.i lands */
    c.nop                          /* at pc % 4 == 2 (checked in objdump) */
    fence.i
    c.addi  a1, 1
    c.addi  a1, 2
    .option pop
    CHECK   2, a1, 3

    /* ---- 3: loop with a predicted branch ---- */
    li      a1, 0
    li      a2, 64
loop3:
    fence.i
    addi    a1, a1, 1
    blt     a1, a2, loop3
    CHECK   3, a1, 64

    /* ---- 4: right after a taken jal ---- */
    li      a1, 0
    jal     zero, tgt4
    li      a1, 99                 /* must be skipped */
tgt4:
    fence.i
    addi    a1, a1, 5
    CHECK   4, a1, 5

    /* ---- 5: pending software interrupt across fence.i ---- */
    li      t0, 0x8                /* mie.MSIE */
    csrw    mie, t0
    li      t2, 0x10003000         /* MSIP */
    li      t0, 1
    sw      t0, 0(t2)
    csrsi   mstatus, 0x8           /* MIE: the interrupt is taken around here */
    fence.i
    li      a1, 7
    csrci   mstatus, 0x8
    CHECK   5, s9, 1
    CHECK   5, a1, 7

    li      t0, 0x2000
    li      t1, 0x600D
    sw      t1, 0(t0)
park:
    j       park

bad:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
    sw      s11, 4(t0)
badpark:
    j       badpark

    .align  2
trap_handler:
    csrr    t3, mcause
    li      t4, 0x80000003
    bne     t3, t4, unexpected
    addi    s9, s9, 1
    li      t4, 0x10003000
    sw      zero, 0(t4)            /* clear MSIP */
    mret
unexpected:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
    li      t1, 0xEEEE
    sw      t1, 4(t0)
th_park:
    j       th_park

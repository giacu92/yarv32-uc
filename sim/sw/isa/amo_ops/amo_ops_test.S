/* A-extension oracle -- LR.W / SC.W / AMO*.W on the D-mem.
 *
 * s11 = number of the running check (stored @0x2004 on failure).
 * a0/a1 = last two values compared (stored @0x2008/@0x200C on failure).
 * s10 = number of traps the test is waiting for; the handler records
 *       mcause in s9 and mtval in s8, steps mepc over the 4-byte
 *       instruction and returns. A trap with s10 == 0 is a FAIL.
 * Scratch words: W0 = 0x2100, W1 = 0x2104.
 */
    .option norvc
    .section .text.init, "ax"

    .equ    W0, 0x2100
    .equ    W1, 0x2104
    .equ    PERI_HOLE, 0x10004000      /* unmapped peri window (DECERR) */

    .macro  EXPECT n, reg, val
    li      s11, \n
    mv      a0, \reg
    li      a1, \val
    bne     a0, a1, bad
    .endm

    /* AMOCASE n, op, old, src, new : W0 = old; op rd, src, (W0);
     * rd must be old and W0 must be new. */
    .macro  AMOCASE n, op, old, src, new
    li      t0, W0
    li      t1, \old
    sw      t1, 0(t0)
    li      t2, \src
    \op     t3, t2, (t0)
    EXPECT  \n, t3, \old
    lw      t4, 0(t0)
    EXPECT  \n, t4, \new
    .endm

    .globl _start
_start:
    li      s10, 0
    la      t0, trap_handler
    csrw    mtvec, t0

    /* ---- 1-9: every AMO, rd = old, mem = new ---- */
    AMOCASE 1, amoswap.w, 0x11111111, 0x22222222, 0x22222222
    AMOCASE 2, amoadd.w,  0xFFFFFFFF, 0x00000002, 0x00000001
    AMOCASE 3, amoxor.w,  0xF0F0F0F0, 0xFF00FF00, 0x0FF00FF0
    AMOCASE 4, amoand.w,  0xF0F0F0F0, 0xFF00FF00, 0xF000F000
    AMOCASE 5, amoor.w,   0xF0F0F0F0, 0xFF00FF00, 0xFFF0FFF0
    /* MIN/MAX: -1 vs 1 orders differently signed and unsigned */
    AMOCASE 6, amomin.w,  0xFFFFFFFF, 0x00000001, 0xFFFFFFFF
    AMOCASE 7, amomax.w,  0xFFFFFFFF, 0x00000001, 0x00000001
    AMOCASE 8, amominu.w, 0xFFFFFFFF, 0x00000001, 0x00000001
    AMOCASE 9, amomaxu.w, 0xFFFFFFFF, 0x00000001, 0xFFFFFFFF
    /* INT_MIN vs INT_MAX, both orders of operand */
    AMOCASE 10, amomin.w,  0x80000000, 0x7FFFFFFF, 0x80000000
    AMOCASE 11, amomax.w,  0x7FFFFFFF, 0x80000000, 0x7FFFFFFF
    AMOCASE 12, amominu.w, 0x7FFFFFFF, 0x80000000, 0x7FFFFFFF
    AMOCASE 13, amomaxu.w, 0x80000000, 0x7FFFFFFF, 0x80000000
    /* equal operands */
    AMOCASE 14, amomin.w,  0x00000005, 0x00000005, 0x00000005

    /* ---- 15: rd == rs2 (the operand register is overwritten by old) ---- */
    li      t0, W0
    li      t1, 40
    sw      t1, 0(t0)
    li      t2, 2
    amoadd.w t2, t2, (t0)
    EXPECT  15, t2, 40
    lw      t4, 0(t0)
    EXPECT  15, t4, 42

    /* ---- 16: distance-1 forward INTO the AMO (address and operand) ---- */
    li      t1, 7
    sw      t1, 0(t0)
    addi    t5, t0, 0              /* address produced the cycle before */
    addi    t6, zero, 3            /* operand produced the cycle before */
    amoadd.w t3, t6, (t5)
    EXPECT  16, t3, 7
    /* ---- 17: distance-1 forward OUT of the AMO ---- */
    li      t1, 100
    sw      t1, 0(t0)
    amoswap.w t3, zero, (t0)
    addi    t4, t3, 1              /* consumes rd immediately */
    EXPECT  17, t4, 101
    lw      t4, 0(t0)
    EXPECT  17, t4, 0

    /* ---- 18: back-to-back AMOs on one word ---- */
    sw      zero, 0(t0)
    li      t2, 1
    amoadd.w zero, t2, (t0)
    amoadd.w zero, t2, (t0)
    amoadd.w t3, t2, (t0)
    EXPECT  18, t3, 2
    lw      t4, 0(t0)
    EXPECT  18, t4, 3

    /* ---- 19: LR/SC success ---- */
    li      t1, 0x1234
    sw      t1, 0(t0)
    lr.w    t3, (t0)
    EXPECT  19, t3, 0x1234
    li      t2, 0x5678
    sc.w    t4, t2, (t0)
    EXPECT  19, t4, 0
    lw      t4, 0(t0)
    EXPECT  19, t4, 0x5678

    /* ---- 20: a second SC fails (the first one cleared the reservation) ---- */
    li      t2, 0x9999
    sc.w    t4, t2, (t0)
    EXPECT  20, t4, 1
    lw      t4, 0(t0)
    EXPECT  20, t4, 0x5678

    /* ---- 21: SC to a different word than the LR fails ---- */
    li      t1, W1
    sw      zero, 0(t1)
    lr.w    t3, (t0)
    sc.w    t4, t2, (t1)
    EXPECT  21, t4, 1
    lw      t4, 0(t1)
    EXPECT  21, t4, 0

    /* ---- 22: a trap between LR and SC kills the reservation ---- */
    lr.w    t3, (t0)
    li      s10, 1
    ecall
    sc.w    t4, t2, (t0)
    EXPECT  22, t4, 1
    EXPECT  22, s9, 11             /* the ecall itself */

    /* ---- 23: LR, plain store elsewhere, SC still succeeds ---- */
    lr.w    t3, (t0)
    sw      zero, 0(t1)
    li      t2, 0xABCD
    sc.w    t4, t2, (t0)
    EXPECT  23, t4, 0
    lw      t4, 0(t0)
    EXPECT  23, t4, 0xABCD

    /* ---- 24-26: misaligned -> 4 (LR) / 6 (SC, AMO), mtval = address ---- */
    addi    t5, t0, 2
    li      s10, 1
    lr.w    t3, (t5)
    EXPECT  24, s9, 4
    EXPECT  24, s8, 0x2102
    li      s10, 1
    sc.w    t3, t2, (t5)
    EXPECT  25, s9, 6
    li      s10, 1
    amoadd.w t3, t2, (t5)
    EXPECT  26, s9, 6
    EXPECT  26, s8, 0x2102
    lw      t4, 0(t0)
    EXPECT  26, t4, 0xABCD         /* nothing was written */

    /* ---- 27-28: atomic to the peri port -> access fault 5 / 7 ---- */
    li      t5, PERI_HOLE
    li      s10, 1
    lr.w    t3, (t5)
    EXPECT  27, s9, 5
    EXPECT  27, s8, PERI_HOLE
    li      s10, 1
    amoswap.w t3, t2, (t5)
    EXPECT  28, s9, 7
    li      s10, 1
    sc.w    t3, t2, (t5)
    EXPECT  28, s9, 7

    /* ---- 29: misa now advertises A ---- */
    csrr    t3, misa
    EXPECT  29, t3, 0x40002105

    /* ===== all checks passed ===== */
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
    sw      a0, 8(t0)
    sw      a1, 12(t0)
badpark:
    j       badpark

    .align  2
trap_handler:
    beqz    s10, unexpected
    csrr    s9, mcause
    csrr    s8, mtval
    addi    s10, s10, -1
    csrr    t6, mepc
    addi    t6, t6, 4
    csrw    mepc, t6
    mret
unexpected:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
    li      t1, 0xEEEE
    sw      t1, 4(t0)
    csrr    t1, mcause
    sw      t1, 8(t0)
    csrr    t1, mepc
    sw      t1, 12(t0)
th_park:
    j       th_park

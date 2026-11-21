/* CSR oracle -- machine CSR values and side effects.
 *
 * Each check loads its number into s11 first, so a failure leaves the
 * failing check number at D-mem 0x2004 next to the 0xBAD marker.
 *
 * Checks:
 *   1  misa == 0x4000_2104 : RV32, I + M + C, and NOT A (bit 0). The core
 *      decodes every standard AMO/LR/SC as illegal, so misa must not
 *      advertise A.
 *   2  mcycle advances between two reads.
 *   3  cycle (0xC00) tracks mcycle: read back-to-back, within 16 cycles.
 *   4  instret counts exactly: two reads 10 nops apart differ by 11 (the
 *      first csrr + 10 nops retired in between).
 *   5  instret == minstret shadow, same exactness.
 *   6  minstret/minstreth write suppresses the increment of the writing
 *      instruction (rv32mi-instret_overflow case 3): write -1 to both
 *      halves, one nop wraps the 64-bit counter to exactly 0.
 *   7  ... and the wrap carried into minstreth (case 4): reads 0.
 *   8  mcycleh is writable and reads back (and cycleh mirrors it).
 *   9  instreth mirrors minstreth.
 *  10  time advances, and tracks the CLINT mtime MMIO register at
 *      0x1000_1000 within 64 cycles (time is mtime, not a counter of its
 *      own).
 *  11  timeh == the CLINT mtime_hi MMIO register.
 *  12  csrr of an unimplemented CSR (satp, 0x180) traps: mcause 2, mtval =
 *      the instruction word.
 *  13  csrw to a read-only CSR (cycle) traps, mcause 2.
 *  14  csrrs/csrrsi/csrrci with x0 / zimm 0 on a read-only CSR do NOT trap
 *      (they do not write).
 *  15  mhartid and the other machine information CSRs, and mstatush, read
 *      0 without trapping (mandatory; Linux and OpenSBI read mhartid).
 *
 * Expected traps: s10 holds the count of traps the test is waiting for. The
 * handler accepts a trap only while s10 != 0, checks mcause == 2, stores
 * mtval in s9, steps mepc over the 4-byte instruction and returns. A trap
 * with s10 == 0 is a FAIL (marker 0xBAD, 0x2004 = 0xEEEE).
 *
 * Any trap the test does not expect lands in trap_handler, which writes
 * 0xBAD, so a missing CSR that traps is a FAIL rather than a hang.
 * Pass marker: 0x600D at D-mem 0x2000.
 */

    .section .text.init, "ax"

    .macro  CHECK n, reg, expect
    li      s11, \n
    li      t1, \expect
    bne     \reg, t1, bad
    .endm

    .globl _start
_start:
    li      s10, 0
    la      t0, trap_handler
    csrw    mtvec, t0

    /* ---- 1: misa ---- */
    csrr    a0, misa
    CHECK   1, a0, 0x40002104

    /* ---- 2: mcycle advances ---- */
    csrr    a0, mcycle
    nop
    csrr    a1, mcycle
    li      s11, 2
    bgeu    a0, a1, bad

    /* ---- 3: cycle tracks mcycle ---- */
    csrr    a0, mcycle
    csrr    a1, cycle
    sub     a2, a1, a0
    li      s11, 3
    li      t1, 16
    bgeu    a2, t1, bad

    /* ---- 4: instret exact ---- */
    csrr    a0, instret
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    csrr    a1, instret
    sub     a2, a1, a0
    CHECK   4, a2, 11

    /* ---- 5: minstret exact ---- */
    csrr    a0, minstret
    nop
    nop
    nop
    csrr    a1, minstret
    sub     a2, a1, a0
    CHECK   5, a2, 4

    /* ---- 6/7: write suppresses the increment ---- */
    li      t0, 0xffffffff
    csrw    minstret, t0
    csrw    minstreth, t0
    nop
    csrr    a0, minstret
    CHECK   6, a0, 0
    csrr    a0, minstreth
    CHECK   7, a0, 0

    /* ---- 8: mcycleh writable, cycleh mirrors ---- */
    li      t0, 5
    csrw    mcycleh, t0
    csrw    mcycle, zero
    csrr    a0, mcycleh
    CHECK   8, a0, 5
    csrr    a0, cycleh
    CHECK   8, a0, 5

    /* ---- 9: instreth mirrors minstreth ---- */
    li      t0, 7
    csrw    minstreth, t0
    csrr    a0, instreth
    CHECK   9, a0, 7

    /* ---- 10: time advances and tracks mtime ---- */
    csrr    a0, time
    nop
    csrr    a1, time
    li      s11, 10
    bgeu    a0, a1, bad
    li      t2, 0x10001000
    lw      a0, 0(t2)              /* mtime_lo */
    csrr    a1, time
    sub     a2, a1, a0
    li      t1, 64
    bgeu    a2, t1, bad

    /* ---- 11: timeh == mtime_hi ---- */
    lw      a0, 4(t2)              /* mtime_hi */
    csrr    a1, timeh
    CHECK   11, a1, 0
    bne     a0, a1, bad

    /* ---- 12: unimplemented CSR traps, mtval = instruction ---- */
    li      s10, 1
    li      s9, 0
trap12:
    csrr    a0, satp
    li      s11, 12
    bnez    s10, bad               /* no trap taken */
    /* .text is I-mem only (Harvard), so the word cannot be loaded; compare
     * against its encoding: csrr a0, satp = csrrs a0, 0x180, x0 */
    li      t1, 0x18002573
    bne     s9, t1, bad

    /* ---- 13: write to a read-only CSR traps ---- */
    li      s10, 1
    csrw    cycle, zero
    li      s11, 13
    bnez    s10, bad

    /* ---- 14: non-writing forms on a read-only CSR do not trap ---- */
    csrrs   a0, cycle, x0
    csrrsi  a0, instret, 0
    csrrci  a0, time, 0

    /* ---- 15: machine information CSRs and mstatush read 0 ---- */
    csrr    a0, mhartid
    CHECK   15, a0, 0
    csrr    a0, mvendorid
    CHECK   15, a0, 0
    csrr    a0, marchid
    CHECK   15, a0, 0
    csrr    a0, mimpid
    CHECK   15, a0, 0
    csrr    a0, 0xF15              /* mconfigptr */
    CHECK   15, a0, 0
    csrr    a0, 0x310              /* mstatush */
    CHECK   15, a0, 0

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
    sw      a0, 8(t0)              /* last value read, for post-mortem */
    sw      a1, 12(t0)
badpark:
    j       badpark

    .align  2
trap_handler:
    beqz    s10, unexpected
    csrr    t1, mcause
    li      t0, 2
    bne     t1, t0, unexpected
    csrr    s9, mtval
    addi    s10, s10, -1
    csrr    t1, mepc
    addi    t1, t1, 4
    csrw    mepc, t1
    mret
unexpected:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
    li      t1, 0xEEEE
    sw      t1, 4(t0)
th_park:
    j       th_park

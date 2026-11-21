/* CSR oracle -- machine CSR values and side effects.
 *
 * Each check loads its number into s11 first, so a failure leaves the
 * failing check number at D-mem 0x2004 next to the 0xBAD marker.
 *
 * Checks:
 *   1  misa == 0x4000_2104 : RV32, I + M + C, and NOT A (bit 0). The core
 *      decodes every standard AMO/LR/SC as illegal, so misa must not
 *      advertise A.
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
    la      t0, trap_handler
    csrw    mtvec, t0

    /* ---- 1: misa ---- */
    csrr    a0, misa
    CHECK   1, a0, 0x40002104

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
badpark:
    j       badpark

    .align  2
trap_handler:
    li      t0, 0x2000
    li      t1, 0xBAD
    sw      t1, 0(t0)
    li      t1, 0xEEEE
    sw      t1, 4(t0)
th_park:
    j       th_park

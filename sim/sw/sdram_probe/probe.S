/* sdram_probe -- board diagnostic boot ROM for the Linux SoC.
 *
 * Replaces yload in the boot ROM for one bitstream. Runs only from the ROM
 * and from registers (no RAM, no stack), so it works even when the SDRAM
 * does not, and prints over the 16550 (115200 8N1) what the SDRAM path does:
 *
 *   T1  bypass write/read of one word at 0x0
 *   T2  address-bit walk: one word at 0 and at 1<<k (k = 2..22), all written,
 *       then all read back (an aliased or stuck address line shows here)
 *   T3  data-bit walk at 0x40: 1<<k and ~(1<<k), k = 0..31
 *   T4  cached store at 0x100000, fence.i, bypass read of the device
 *   T5  cached load of a word written under bypass at 0x100040
 *   T6  a 3-instruction routine written under bypass at 0x200000, fence.i,
 *       called: it must return 0x5A in a0 (instruction fetch from SDRAM)
 *
 * Each line: "Tn ok" or "Tn FAIL a=<addr> got=<x> want=<y>". Then it repeats
 * every ~2 s, so a terminal opened late still sees a full report.
 */
    .option norvc
    .section .text.init, "ax"
    .equ    UART, 0x10000000
    .equ    LSR, 0x14
    .equ    CCSR, 0x00801000

    .macro  PUTS label
    la      a1, \label
    jal     ra, puts
    .endm

    .globl _start
_start:
    li      s0, UART
    li      s1, CCSR
again:
    PUTS    s_banner
    li      s11, 0                 /* failure count */
    li      s8, 0                  /* failures in the current quiet test */

    /* T1 */
    li      t0, 1
    sb      t0, 0(s1)              /* bypass on */
    li      a2, 0
    li      a4, 0x12345678
    sw      a4, 0(a2)
    lw      a3, 0(a2)
    li      a5, '1'
    jal     ra, report

    /* T2: write all, then read all */
    sw      zero, 0(zero)
    li      t3, 2
1:  li      a2, 1
    sll     a2, a2, t3
    addi    a4, t3, 0x100          /* pattern = 0x100 + k */
    sw      a4, 0(a2)
    addi    t3, t3, 1
    li      t0, 23
    blt     t3, t0, 1b
    li      a2, 0
    li      a4, 0
    lw      a3, 0(a2)
    li      a5, '2'
    jal     ra, report_quiet
    li      t3, 2
2:  li      a2, 1
    sll     a2, a2, t3
    addi    a4, t3, 0x100
    lw      a3, 0(a2)
    li      a5, '2'
    jal     ra, report_quiet
    addi    t3, t3, 1
    li      t0, 23
    blt     t3, t0, 2b
    li      a5, '2'
    jal     ra, report_done

    /* T3: data bits */
    li      t3, 0
3:  li      a2, 0x40
    li      a4, 1
    sll     a4, a4, t3
    sw      a4, 0(a2)
    lw      a3, 0(a2)
    li      a5, '3'
    jal     ra, report_quiet
    not     a4, a4
    sw      a4, 0(a2)
    lw      a3, 0(a2)
    jal     ra, report_quiet
    addi    t3, t3, 1
    li      t0, 32
    blt     t3, t0, 3b
    li      a5, '3'
    jal     ra, report_done

    /* T4: cached store, fence.i, device read */
    sb      zero, 0(s1)            /* bypass off */
    li      a2, 0x100000
    li      a4, 0xC0DE0004
    sw      a4, 0(a2)
    fence.i
    li      t0, 1
    sb      t0, 0(s1)
    lw      a3, 0(a2)
    li      a5, '4'
    jal     ra, report

    /* T5: bypass store, cached load */
    li      a2, 0x100040
    li      a4, 0xC0DE0005
    sw      a4, 0(a2)
    sb      zero, 0(s1)
    lw      a3, 0(a2)
    li      a5, '5'
    jal     ra, report

    /* T6: run code from SDRAM */
    li      t0, 1
    sb      t0, 0(s1)
    li      a2, 0x200000
    li      t0, 0x05a00513         /* addi a0, x0, 0x5a */
    sw      t0, 0(a2)
    li      t0, 0x00000013         /* nop */
    sw      t0, 4(a2)
    li      t0, 0x00008067         /* jalr x0, 0(ra) */
    sw      t0, 8(a2)
    sb      zero, 0(s1)
    fence.i
    li      a0, 0
    mv      s10, ra
    jalr    ra, 0(a2)
    mv      a3, a0
    li      a4, 0x5a
    li      a5, '6'
    jal     ra, report

    bnez    s11, 8f
    PUTS    s_pass
    j       9f
8:  PUTS    s_fail
9:  li      t0, 25000000           /* ~2 s at ~2 instr/iteration */
7:  addi    t0, t0, -1
    bnez    t0, 7b
    j       again

/* report: a2 addr, a3 got, a4 want, a5 test char. Prints a line. */
report:
    mv      s9, ra
    bne     a3, a4, rfail
    li      a0, 'T'
    jal     ra, putc
    mv      a0, a5
    jal     ra, putc
    PUTS    s_ok
    jr      s9
rfail:
    addi    s11, s11, 1
    jal     ra, failline
    jr      s9
/* report_quiet: print only on failure (at most 4 lines per test via s8) */
report_quiet:
    mv      s9, ra
    beq     a3, a4, 1f
    addi    s11, s11, 1
    addi    s8, s8, 1
    li      t0, 5
    bge     s8, t0, 1f
    jal     ra, failline
1:  jr      s9
/* report_done: closes a quiet test: "Tn ok" if it had no failure */
report_done:
    mv      s9, ra
    bnez    s8, 1f
    li      a0, 'T'
    jal     ra, putc
    mv      a0, a5
    jal     ra, putc
    PUTS    s_ok
1:  li      s8, 0
    jr      s9

failline:
    mv      s7, ra
    li      a0, 'T'
    jal     ra, putc
    mv      a0, a5
    jal     ra, putc
    PUTS    s_fa
    mv      a0, a2
    jal     ra, hex
    PUTS    s_got
    mv      a0, a3
    jal     ra, hex
    PUTS    s_want
    mv      a0, a4
    jal     ra, hex
    PUTS    s_nl
    jr      s7

/* hex: a0 -> 8 hex digits. Uses t4-t6, a6. */
hex:
    mv      a6, ra
    mv      t4, a0
    li      t5, 28
1:  srl     a0, t4, t5
    andi    a0, a0, 15
    li      t6, 10
    blt     a0, t6, 2f
    addi    a0, a0, 'a' - '0' - 10
2:  addi    a0, a0, '0'
    jal     ra, putc
    addi    t5, t5, -4
    bgez    t5, 1b
    jr      a6

/* puts: a1 -> zero-terminated string in ROM. Uses t1, a7. */
puts:
    mv      a7, ra
1:  lbu     a0, 0(a1)
    beqz    a0, 2f
    jal     ra, putc
    addi    a1, a1, 1
    j       1b
2:  jr      a7

putc:
1:  lw      t2, LSR(s0)
    andi    t2, t2, 0x20
    beqz    t2, 1b
    sw      a0, 0(s0)
    ret

s_banner: .asciz "\r\nsdram_probe\r\n"
s_ok:     .asciz " ok\r\n"
s_fa:     .asciz " FAIL a="
s_got:    .asciz " got="
s_want:   .asciz " want="
s_nl:     .asciz "\r\n"
s_pass:   .asciz "PROBE PASS\r\n"
s_fail:   .asciz "PROBE FAIL\r\n"

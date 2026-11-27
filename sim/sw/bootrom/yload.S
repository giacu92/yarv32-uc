/* yload -- UART boot loader for the Linux SoC boot ROM (M9).
 *
 * Runs from the 2 KiB boot ROM at 0x0080_0000 (the reset vector). Speaks a
 * framed binary protocol over the 16550 at 0x1000_0000 (115200 8N1 on the
 * board, whatever the DLL reset value gives in sim):
 *
 *   loader -> host : 'Y'                          ready
 *   host -> loader : 'Y' 'L'                      magic
 *                    u32 load address            little-endian
 *                    u32 length in bytes
 *                    u32 entry point
 *                    payload[length]
 *                    u32 CRC-32 (IEEE 802.3, reflected, init/xorout ~0)
 *   loader -> host : 'K' and jumps to entry,  or  'E' and starts over
 *
 * Boot sources, in order (2026-11-27):
 *   1. the SPI flash at FLASH_OFF (2 MiB, past the FPGA bitstream on the
 *      board's 8 MiB configuration flash): the same frame, read with the
 *      0x03 READ command. Used only if the first two bytes are 'Y' 'L', and
 *      only if its CRC checks; any failure falls back to
 *   2. the UART, as above ('Y' prompt first).
 *
 * The payload is stored with the cache control register's CACHE_BYPASS bit
 * set (0x0080_1000 bit 0), so every byte lands in the SDRAM device itself and
 * not in a dirty D-cache line the instruction side would never see. The bit
 * is cleared and a fence.i issued before the jump. No stack and no RAM are
 * used: everything lives in registers.
 */
    .option norvc
    .section .text.init, "ax"
    .equ    UART, 0x10000000
    .equ    LSR, 0x14
    .equ    CCSR, 0x00801000
    .equ    SPI, 0x10006000        /* STATUS 0, CTRL 4, CLKDIV 8, CS c, TX 10, RX 14 */
    .equ    FLASH_OFF, 0x00200000

    .globl _start
_start:
    li      s0, UART
    li      s1, CCSR
    li      a6, SPI
    /* ---- try the SPI flash first ---- */
    li      t0, 1
    sw      t0, 8(a6)              /* CLKDIV 1: SPI clock = clk/4 */
    sw      t0, 4(a6)              /* CTRL: ENABLE, mode 0 */
    sw      zero, 0xc(a6)          /* CS low */
    li      a0, 0x03
    jal     ra, spi_xfer
    li      a0, (FLASH_OFF >> 16) & 0xff
    jal     ra, spi_xfer
    li      a0, (FLASH_OFF >> 8) & 0xff
    jal     ra, spi_xfer
    li      a0, FLASH_OFF & 0xff
    jal     ra, spi_xfer
    li      a7, 1                  /* source = SPI */
    jal     ra, getc
    li      t0, 'Y'
    bne     a0, t0, use_uart
    jal     ra, getc
    li      t0, 'L'
    beq     a0, t0, got_magic
use_uart:
    li      t0, 1
    sw      t0, 0xc(a6)            /* CS high */
    li      a7, 0                  /* source = UART */
restart:
    li      a0, 'Y'
    jal     ra, putc
    /* magic */
1:  jal     ra, getc
    li      t0, 'Y'
    bne     a0, t0, 1b
    jal     ra, getc
    li      t0, 'L'
    bne     a0, t0, 1b
got_magic:
    jal     ra, get32
    mv      s2, a0                 /* load address */
    jal     ra, get32
    mv      s3, a0                 /* length */
    jal     ra, get32
    mv      s4, a0                 /* entry */
    li      t0, 1
    sb      t0, 0(s1)              /* CACHE_BYPASS on */
    li      s5, -1                 /* crc */
    li      s7, 0xEDB88320
    mv      s6, s2                 /* write pointer */
    add     s8, s2, s3             /* end */
2:  beq     s6, s8, 5f
    jal     ra, getc
    sb      a0, 0(s6)
    addi    s6, s6, 1
    xor     s5, s5, a0
    li      t1, 8
3:  andi    t2, s5, 1
    srli    s5, s5, 1
    beqz    t2, 4f
    xor     s5, s5, s7
4:  addi    t1, t1, -1
    bnez    t1, 3b
    j       2b
5:  not     s5, s5
    jal     ra, get32
    sb      zero, 0(s1)            /* CACHE_BYPASS off */
    bne     a0, s5, bad
    li      t0, 1
    sw      t0, 0xc(a6)            /* CS high (harmless when booting from UART) */
    li      a0, 'K'
    jal     ra, putc
    fence.i
    jr      s4
bad:
    li      a0, 'E'
    jal     ra, putc
    bnez    a7, use_uart           /* a bad flash image falls back to the UART */
    j       restart

/* a0 <- next byte from the current source (a7: 0 = UART, 1 = SPI flash) */
getc:
    beqz    a7, getc_uart
    li      a0, 0xff
    j       spi_xfer               /* tail call: returns the byte read */
getc_uart:
    lw      t0, LSR(s0)
    andi    t0, t0, 1              /* LSR.DR */
    beqz    t0, getc
    lw      a0, 0(s0)
    andi    a0, a0, 0xff
    ret

/* a0 <- u32, little-endian. Saves ra in s9; uses s10/s11. */
get32:
    mv      s9, ra
    li      s10, 0
    li      s11, 0
6:  jal     ra, getc
    sll     a0, a0, s11
    or      s10, s10, a0
    addi    s11, s11, 8
    li      t0, 32
    bne     s11, t0, 6b
    mv      a0, s10
    mv      ra, s9
    ret

/* SPI: send a0, return the byte received in a0 */
spi_xfer:
    sw      a0, 0x10(a6)
7:  lw      t0, 0(a6)
    andi    t0, t0, 4              /* STATUS.RX_EMPTY */
    bnez    t0, 7b
    lw      a0, 0x14(a6)
    andi    a0, a0, 0xff
    ret

/* transmit a0 */
putc:
    lw      t0, LSR(s0)
    andi    t0, t0, 0x20           /* LSR.THRE */
    beqz    t0, putc
    sw      a0, 0(s0)
    ret

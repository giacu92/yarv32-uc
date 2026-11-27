/* yload -- boot loader for the Linux SoC boot ROM (M9, M9b).
 *
 * Runs from the 2 KiB boot ROM at 0x0080_0000 (the reset vector). Loads one
 * frame, the same bytes from either source:
 *
 *   'Y' 'L'                       magic
 *   u32 load address              little-endian
 *   u32 length in bytes
 *   u32 entry point
 *   payload[length]
 *   u32 CRC-32 (IEEE 802.3, reflected, init/xorout ~0) of the payload
 *
 * Boot sources, in order (D42, 2026-11-27):
 *   1. the microSD card in SPI mode, on the axi4_lite_spi controller at
 *      0x1000_6000 (its pins are the card slot's): the frame starts at
 *      SECTOR 2048 (byte offset 1 MiB), read with CMD18 (multiple block)
 *      and stopped with CMD12. Card prep:
 *          dd if=linux.yl of=/dev/sdX bs=512 seek=2048 conv=fsync
 *      Any failure -- no card, init timeout, no 'Y' 'L' at sector 2048, a
 *      data error token, a token timeout, a CRC mismatch -- falls back to
 *   2. the UART (16550 at 0x1000_0000, 115200 8N1 on the board):
 *        loader -> host : 'Y'                    ready
 *        host -> loader : the frame
 *        loader -> host : 'K' and jumps to entry,  or  'E' and starts over
 *      A bad SD image prints 'E', then 'Y', then waits for the UART.
 *
 * The payload is stored with the cache control register's CACHE_BYPASS bit
 * set (0x0080_1000 bit 0), so every byte lands in the SDRAM device itself and
 * not in a dirty D-cache line the instruction side would never see. The bit
 * is cleared and a fence.i issued before the jump. No stack and no RAM are
 * used: everything lives in registers.
 *
 * Register map: s0 UART, s1 CCSR, a6 SPI, a7 source (0 UART, 1 SD), s7 CRC
 * table; a5 bytes left in the current SD block, a4 card capacity (CCS).
 *
 * SD clocking. The SPI clock is clk / (2 * (CLKDIV + 1)), i.e. one SCK
 * half period = CLKDIV + 1 core clocks.
 *   - identification: CLKDIV 63 -> 50 MHz / 128 = 390.6 kHz (spec: <= 400).
 *   - data:           CLKDIV 3  -> 50 MHz / 8   = 6.25 MHz.
 * Why not clk/4 (CLKDIV 1): in mode 0 the card changes MISO after the SCK
 * falling edge and the master samples it at the next rising edge, half an
 * SCK period later. On the board MISO goes through a 2-flop synchroniser
 * plus ~1 core clock of pad + card output delay = 2-3 core clocks. A half
 * period of 2 clocks (clk/4) would sample the previous bit; 4 clocks
 * (clk/8) leaves 1-2 clocks of margin.
 */
    .option norvc
    .section .text.init, "ax"
    .equ    UART, 0x10000000
    .equ    LSR, 0x14
    .equ    CCSR, 0x00801000
    .equ    SPI, 0x10006000        /* STATUS 0, CTRL 4, CLKDIV 8, CS c, TX 10, RX 14 */
    .equ    SD_SECTOR, 2048        /* frame start: 1 MiB into the card */
    .equ    DIV_INIT, 63           /* 390.6 kHz */
    .equ    DIV_DATA, 3            /* 6.25 MHz, see the header */

    .globl _start
_start:
    li      s0, UART
    li      s1, CCSR
    li      a6, SPI
    la      s7, crc_tab
    li      a7, 0
    li      a5, 0
    /* ---- SD card, SPI mode ---- */
    li      t0, DIV_INIT
    sw      t0, 8(a6)
    li      t0, 1
    sw      t0, 4(a6)              /* CTRL: ENABLE, mode 0 */
    sw      t0, 0xc(a6)            /* CS high */
    li      t2, 10                 /* >= 74 clocks with CS high */
1:  li      a0, 0xff
    jal     ra, spi_xfer
    addi    t2, t2, -1
    bnez    t2, 1b
    sw      zero, 0xc(a6)          /* CS low */
    li      a3, 8                  /* CMD0 -> idle (R1 = 0x01) */
2:  li      a0, 0x40
    li      a1, 0
    li      a2, 0x95
    jal     ra, sd_cmd
    li      t0, 1
    beq     a0, t0, 3f
    addi    a3, a3, -1
    bnez    a3, 2b
    j       use_uart
3:  li      a0, 0x48               /* CMD8, 2.7-3.6 V, check pattern 0xAA */
    li      a1, 0x1AA
    li      a2, 0x87
    jal     ra, sd_cmd
    li      a4, 0                  /* ACMD41 HCS: 0 for a v1 card */
    andi    t0, a0, 4              /* illegal command: SD v1 */
    bnez    t0, 4f
    li      t0, 1
    bne     a0, t0, use_uart
    jal     ra, sd_rd32            /* R7 */
    slli    a1, a1, 20             /* voltage nibble + check pattern */
    li      t0, 0x1AA << 20
    bne     a1, t0, use_uart
    li      a4, 0x40000000
4:  li      a3, 4096               /* ACMD41 until ready, ~1.5 s bound */
5:  li      a0, 0x77               /* CMD55 */
    li      a1, 0
    li      a2, 1
    jal     ra, sd_cmd
    li      a0, 0x69               /* ACMD41 */
    mv      a1, a4
    jal     ra, sd_cmd
    beqz    a0, 6f
    addi    a3, a3, -1
    bnez    a3, 5b
    j       use_uart
6:  beqz    a4, 7f                 /* v1: byte addressing, no CMD58 */
    li      a0, 0x7A               /* CMD58: OCR */
    li      a1, 0
    jal     ra, sd_cmd
    bnez    a0, use_uart
    jal     ra, sd_rd32
    srli    a4, a1, 30             /* CCS: 1 = block addressing */
    andi    a4, a4, 1
7:  li      a1, SD_SECTOR
    bnez    a4, 8f
    li      a0, 0x50               /* CMD16: 512-byte blocks */
    li      a1, 512
    jal     ra, sd_cmd
    bnez    a0, use_uart
    li      a1, SD_SECTOR * 512
8:  li      t0, DIV_DATA
    sw      t0, 8(a6)
    li      a0, 0x52               /* CMD18: read multiple blocks */
    jal     ra, sd_cmd
    bnez    a0, use_uart
    jal     ra, sd_token
    li      a7, 1                  /* source = SD */
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
    li      a5, 0                  /* keeps the load loop off the SD fast path */
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
    mv      s6, s2                 /* write pointer */
    add     s8, s2, s3             /* end */
2:  beq     s6, s8, 5f
    beqz    a5, 3f                 /* UART, or an SD block boundary */
    /* SD fast path, sd_getc inlined (a5 != 0 only while streaming) */
    addi    a5, a5, -1
    li      a0, 0xff
    sw      a0, 0x10(a6)           /* queue the next byte */
4:  lw      t0, 0(a6)
    andi    t0, t0, 4              /* STATUS.RX_EMPTY */
    bnez    t0, 4b
    lw      a0, 0x14(a6)
    andi    a0, a0, 0xff
    j       6f
3:  jal     ra, getc
6:  sb      a0, 0(s6)
    addi    s6, s6, 1
    /* CRC-32, a nibble at a time from a 16-word table (two lookups/byte) */
    xor     s5, s5, a0
    andi    t1, s5, 15
    slli    t1, t1, 2
    add     t1, t1, s7
    lw      t1, 0(t1)
    srli    s5, s5, 4
    xor     s5, s5, t1
    andi    t1, s5, 15
    slli    t1, t1, 2
    add     t1, t1, s7
    lw      t1, 0(t1)
    srli    s5, s5, 4
    xor     s5, s5, t1
    j       2b
5:  not     s5, s5
    jal     ra, get32
    sb      zero, 0(s1)            /* CACHE_BYPASS off */
    bne     a0, s5, bad
    beqz    a7, 9f
    jal     ra, spi_rx             /* drain the byte sd_getc queued */
    li      a0, 0x14C              /* CMD12 (bit 8: skip the stuff byte) */
    li      a1, 0
    jal     ra, sd_cmd
9:  li      t0, 1
    sw      t0, 0xc(a6)            /* CS high (harmless when booting from UART) */
    li      a0, 'K'
    jal     ra, putc
    fence.i
    /* Linux boot convention (RISC-V boot protocol): a0 = hart ID, a1 = DTB
     * pointer, 0 = use the kernel's built-in DTB. Without this a0 still held
     * the 'K' just transmitted, which the kernel would take as its hart ID. */
    csrr    a0, mhartid
    li      a1, 0
    jr      s4
bad:
    li      a0, 'E'
    jal     ra, putc
    bnez    a7, use_uart           /* a bad SD image falls back to the UART */
    j       restart

/* a0 <- next byte from the current source (a7: 0 = UART, 1 = SD card) */
getc:
    bnez    a7, sd_getc
getc_uart:
    lw      t0, LSR(s0)
    andi    t0, t0, 1              /* LSR.DR */
    beqz    t0, getc_uart
    lw      a0, 0(s0)
    andi    a0, a0, 0xff
    ret

/* Next payload byte of the CMD18 stream, pipelined: between calls one 0xFF
 * is always queued in the SPI controller, so the card clocks the next byte
 * out while the caller stores this one and folds it into the CRC. At a block
 * boundary the queued byte is CRC-16 byte 1: drop it and byte 2, wait for
 * the next start token (which queues a new byte). Saves ra in t4. */
sd_getc:
    mv      t4, ra
    bnez    a5, 1f
    jal     ra, spi_rx             /* CRC-16 byte 1 (the queued one) */
    li      a0, 0xff
    jal     ra, spi_xfer           /* CRC-16 byte 2 */
    jal     ra, sd_token
1:  addi    a5, a5, -1
    li      a0, 0xff
    sw      a0, 0x10(a6)           /* queue the next byte ... */
    mv      ra, t4
    j       spi_rx                 /* ... and return the current one */

/* Wait for the 0xFE data start token (bounded), then a5 = 512. A data error
 * token (anything but 0xFF / 0xFE) or a timeout abandons the SD boot. Saves
 * ra in t3; uses a3. */
sd_token:
    mv      t3, ra
    li      a3, 0x40000
1:  li      a0, 0xff
    jal     ra, spi_xfer
    li      t0, 0xfe
    beq     a0, t0, 2f
    li      t0, 0xff
    bne     a0, t0, use_uart
    addi    a3, a3, -1
    bnez    a3, 1b
    j       use_uart
2:  li      a5, 512
    li      a0, 0xff               /* MOSI stays high while data flows */
    sw      a0, 0x10(a6)           /* queue the first data byte */
    mv      ra, t3
    ret

/* Send a command: a0 = 0x40 | index (bit 8: skip one stuff byte before the
 * response, for CMD12), a1 = argument, a2 = CRC byte with the end bit.
 * A 0xFF precedes the command (Nrc). Returns R1 in a0, 0xFF = no response
 * within 16 bytes. Saves ra in t6; uses t1, t2, t5. */
sd_cmd:
    mv      t6, ra
    mv      t5, a0
    li      a0, 0xff
    jal     ra, spi_xfer
    mv      a0, t5
    jal     ra, spi_xfer           /* only byte[7:0] reaches TXDATA */
    li      t2, 24
1:  srl     a0, a1, t2
    jal     ra, spi_xfer
    addi    t2, t2, -8
    bgez    t2, 1b
    mv      a0, a2
    jal     ra, spi_xfer
    srli    t5, t5, 8
    beqz    t5, 2f
    li      a0, 0xff
    jal     ra, spi_xfer
2:  li      t2, 16
3:  li      a0, 0xff
    jal     ra, spi_xfer
    andi    t1, a0, 0x80
    beqz    t1, 4f
    addi    t2, t2, -1
    bnez    t2, 3b
4:  mv      ra, t6
    ret

/* a1 <- next 4 bytes, big-endian (R7 / OCR). Saves ra in t5; uses t2. */
sd_rd32:
    mv      t5, ra
    li      t2, 4
1:  li      a0, 0xff
    jal     ra, spi_xfer
    slli    a1, a1, 8
    or      a1, a1, a0
    addi    t2, t2, -1
    bnez    t2, 1b
    mv      ra, t5
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

/* SPI: send a0, return the byte received in a0. spi_rx: only receive (the
 * byte was queued earlier). */
spi_xfer:
    sw      a0, 0x10(a6)
spi_rx:
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

/* CRC-32 (reflected, poly 0xEDB88320) of each 4-bit value, read by the
 * D side out of this ROM. */
    .balign 4
crc_tab:
    .word   0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac
    .word   0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c
    .word   0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c
    .word   0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c

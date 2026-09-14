#ifndef W25Q_H
#define W25Q_H

/*
 * Driver for Winbond W25Qxx SPI NOR flash (W25Q16/32/64/128 family) on the
 * axi4_lite_spi master, on top of common/spi.h. Mode 0, CS framing owned by
 * the driver (one command = one CS frame).
 *
 * Memory model: 24-bit byte addresses. A page is 256 bytes -- a page
 * program can only clear bits inside one page, so w25q_write() splits
 * crossing writes at page boundaries. Erase granularity is a 4 KiB sector
 * (or 32/64 KiB blocks); erase sets bits to 1, programming clears them.
 *
 * Commands used (first byte on the wire):
 *   0x06 WRITE ENABLE      latches WEL, required before any program/erase
 *   0x04 WRITE DISABLE     drops WEL
 *   0x05 READ STATUS       1 byte, loop until WIP (bit 0) clears
 *   0x03 READ DATA         24-bit addr, then data out for as long as clocked
 *   0x02 PAGE PROGRAM      24-bit addr, then up to 256 bytes in
 *   0x20 SECTOR ERASE      4 KiB
 *   0x52 BLOCK ERASE 32K / 0xD8 BLOCK ERASE 64K
 *   0xC7 CHIP ERASE        whole device
 *   0x9F JEDEC ID          3 bytes: manufacturer 0xEF, type, capacity
 *   0xAB RELEASE POWER-DOWN / device ID (also wakes the chip)
 *   0xB9 POWER-DOWN
 *
 * SPI clock: the datasheet allows 50 MHz for read and ~33 MHz (or less on
 * the smaller parts) for program/erase; the driver takes one clock for
 * everything, so pass the program-safe figure (or use w25q_begin_hz and
 * accept the conservative single clock).
 */

#include "spi.h"

#define W25Q_PAGE_SIZE 256u

/* Command bytes. */
#define W25Q_CMD_WREN       0x06u
#define W25Q_CMD_WRDI       0x04u
#define W25Q_CMD_RDSR       0x05u
#define W25Q_CMD_READ       0x03u
#define W25Q_CMD_PP         0x02u
#define W25Q_CMD_SE         0x20u
#define W25Q_CMD_BE32K      0x52u
#define W25Q_CMD_BE64K      0xD8u
#define W25Q_CMD_CE         0xC7u
#define W25Q_CMD_JEDEC_ID   0x9Fu
#define W25Q_CMD_PWRDN      0xB9u
#define W25Q_CMD_RELEASE    0xABu

/* Status-1 bits. */
#define W25Q_SR_WIP         0x01u  /* busy: program/erase in progress */
#define W25Q_SR_WEL         0x02u  /* write enable latch */

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

/* Enable the SPI engine for the flash. hz should respect the slowest
 * command you use (see the header comment); mode is fixed at 0. */
static void w25q_begin_hz(unsigned int hz)
{
    spi_begin_hz(hz, 0);
}

/* ------------------------------------------------------------------ */
/* Status / identification                                             */
/* ------------------------------------------------------------------ */

/* One status-register read (the standard way to poll). */
static unsigned char w25q_status(void)
{
    unsigned char st;

    spi_select();
    spi_transfer(W25Q_CMD_RDSR);
    st = spi_transfer(0xFFu);
    spi_deselect();
    return st;
}

/* True while a program or erase is running. */
static int w25q_busy(void)
{
    return (w25q_status() & W25Q_SR_WIP) != 0u;
}

/* Block until the chip finishes the current program/erase. There is no
 * timeout: the datasheet worst case (chip erase on the big parts) is a
 * couple of minutes, a real failure loops forever. */
static void w25q_wait_ready(void)
{
    while (w25q_busy())
        ;
}

/* JEDEC ID: 0xEF for Winbond, then memory type and capacity. Returns 0 if
 * the answer does not even look like Winbond (nothing on the bus answers
 * 0xFF back). */
static unsigned int w25q_jedec_id(void)
{
    unsigned char id[3];

    spi_select();
    spi_transfer(W25Q_CMD_JEDEC_ID);
    spi_read(id, 3);
    spi_deselect();
    return ((unsigned int)id[0] << 16) | ((unsigned int)id[1] << 8) | id[2];
}

/* True if a Winbond flash answers. Cheap board bring-up check. */
static int w25q_present(void)
{
    return (w25q_jedec_id() >> 16) == 0xEFu;
}

/* ------------------------------------------------------------------ */
/* Read                                                                */
/* ------------------------------------------------------------------ */

/* Read len bytes from a 24-bit address into buf. Sequential read: keep CS
 * low and the address auto-increments across page and sector boundaries.
 * The 4-byte command+address header is exchanged with the in-flight window,
 * so long reads run at wire speed. */
static void w25q_read(unsigned int addr, unsigned char *buf, unsigned int len)
{
    unsigned char hdr[4];

    hdr[0] = W25Q_CMD_READ;
    hdr[1] = (unsigned char)(addr >> 16);
    hdr[2] = (unsigned char)(addr >> 8);
    hdr[3] = (unsigned char)addr;

    spi_select();
    spi_send(hdr, 4);
    spi_read(buf, len);
    spi_deselect();
}

/* ------------------------------------------------------------------ */
/* Write (program)                                                     */
/* ------------------------------------------------------------------ */

/* Write ENABLE: must precede every program/erase command. */
static void w25q_wren(void)
{
    spi_select();
    spi_transfer(W25Q_CMD_WREN);
    spi_deselect();
}

/* Program a range of bytes, splitting at page boundaries so no page
 * program wraps around inside a page. buf must be erased (0xFF) where it
 * writes: programming can only clear bits. Blocks until done. */
static void w25q_write(unsigned int addr, const unsigned char *buf, unsigned int len)
{
    while (len) {
        unsigned int page_off = addr % W25Q_PAGE_SIZE;
        unsigned int chunk = W25Q_PAGE_SIZE - page_off;

        if (chunk > len)
            chunk = len;

        w25q_wren();
        spi_select();
        spi_transfer(W25Q_CMD_PP);
        spi_transfer((unsigned char)(addr >> 16));
        spi_transfer((unsigned char)(addr >> 8));
        spi_transfer((unsigned char)addr);
        spi_send(buf, chunk);
        spi_deselect();
        w25q_wait_ready();

        addr += chunk;
        buf += chunk;
        len -= chunk;
    }
}

/* Convenience: program up to one page (caller guarantees no wrap). */
static void w25q_write_page(unsigned int page, const unsigned char *buf, unsigned int len)
{
    w25q_write(page * W25Q_PAGE_SIZE, buf, len);
}

/* ------------------------------------------------------------------ */
/* Erase                                                               */
/* ------------------------------------------------------------------ */

static void w25q_erase_cmd(unsigned char cmd, unsigned int addr)
{
    w25q_wren();
    spi_select();
    spi_transfer(cmd);
    spi_transfer((unsigned char)(addr >> 16));
    spi_transfer((unsigned char)(addr >> 8));
    spi_transfer((unsigned char)addr);
    spi_deselect();
    w25q_wait_ready();
}

/* Erase one 4 KiB sector (addr = any address inside it). */
static void w25q_erase_sector(unsigned int addr)
{
    w25q_erase_cmd(W25Q_CMD_SE, addr);
}

/* Erase a 32 KiB block (addr = any address inside it). */
static void w25q_erase_block32k(unsigned int addr)
{
    w25q_erase_cmd(W25Q_CMD_BE32K, addr);
}

/* Erase a 64 KiB block (addr = any address inside it). */
static void w25q_erase_block64k(unsigned int addr)
{
    w25q_erase_cmd(W25Q_CMD_BE64K, addr);
}

/* Erase the whole chip. Slow (up to minutes on the big parts). */
static void w25q_erase_chip(void)
{
    w25q_wren();
    spi_select();
    spi_transfer(W25Q_CMD_CE);
    spi_deselect();
    w25q_wait_ready();
}

/* ------------------------------------------------------------------ */
/* Power                                                               */
/* ------------------------------------------------------------------ */

/* Drop into the ~1 uA power-down state (READ and ID commands stop working). */
static void w25q_power_down(void)
{
    spi_select();
    spi_transfer(W25Q_CMD_PWRDN);
    spi_deselect();
}

/* Wake from power-down (RELEASE POWER-DOWN also returns the legacy device
 * ID byte, which is discarded here). The chip needs ~3 us before the first
 * command; at 50 MHz that is 150 cycles, and the transaction teardown is
 * already slower than that. */
static void w25q_wake(void)
{
    unsigned char id;

    spi_select();
    id = spi_transfer(W25Q_CMD_RELEASE);
    spi_transfer(0xFFu);  /* the ID itself comes on the second byte */
    spi_deselect();
    (void)id;
}

#endif /* W25Q_H */
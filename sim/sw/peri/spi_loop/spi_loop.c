#include <stdint.h>
#include "uart.h"
#include "spi.h"

/*
 * End-to-end SPI oracle: the common/spi.h library against the sim's SPI
 * loopback (MISO = MOSI), which makes every received byte exactly the byte
 * sent and is protocol-agnostic across all four CPOL/CPHA modes.
 *
 * Covers, in order:
 *   1. single-byte full-duplex transfer with CS framing
 *   2. a 16-byte buffered exchange (exactly the in-flight window / FIFO
 *      depth -- the boundary case of spi_transfer_buf's window bound)
 *   3. a 40-byte exchange (window must refill while bytes complete)
 *   4. send-only and read-only forms (filler 0xFF comes back on read)
 *   5. 16/32-bit helpers, MSB first
 *   6. all four SPI modes (the engine's CPHA/CPOL paths; loopback data is
 *      mode-independent, so any mismatch is the engine, not the model)
 *
 * Result marker: 0x600D / 0xBAD at D-mem 0x3000.
 *
 * SCLK: 1 MHz (CLKDIV=24 at 50 MHz) for a fast sim; the board top wires
 * the SPI pins directly, so slower clocks work identically.
 */

static volatile uint32_t *const result = (volatile uint32_t *)0x3000;

static uint32_t fails;

static void check(int ok, const char *what)
{
    uart_puts(ok ? "OK " : "FAIL ");
    uart_puts(what);
    uart_puts("\r\n");
    if (!ok)
        fails++;
}

int main(void)
{
    unsigned char tb[40];
    unsigned char rb[40];

    uart_puts("\r\nSPI LOOP\r\n");

    spi_begin_hz(1000000, 0);

    /* 1. Single byte with CS framing. */
    spi_select();
    unsigned char echo = spi_transfer(0xA5);
    spi_deselect();
    check(echo == 0xA5, "transfer 0xA5 loopback");

    /* 2. Exactly the window depth (16). */
    for (int i = 0; i < 16; ++i)
        tb[i] = (unsigned char)(0x60 + i);
    spi_select();
    spi_transfer_buf(tb, rb, 16);
    spi_deselect();
    {
        int ok = 1;
        for (int i = 0; i < 16; ++i)
            if (rb[i] != tb[i]) ok = 0;
        check(ok, "transfer_buf 16 (window boundary)");
    }

    /* 3. Past the window: refill while bytes complete. */
    for (int i = 0; i < 40; ++i)
        tb[i] = (unsigned char)(0x80 + i);
    spi_select();
    spi_transfer_buf(tb, rb, 40);
    spi_deselect();
    {
        int ok = 1;
        for (int i = 0; i < 40; ++i)
            if (rb[i] != tb[i]) ok = 0;
        check(ok, "transfer_buf 40 (past window)");
    }

    /* 4. Send-only / read-only. Read clocks 0xFF fillers, so with the
     * loopback the fillers come back: rx must be all-FF. */
    spi_select();
    spi_send(tb, 8);
    spi_deselect();
    check(!spi_busy(), "not busy after send");
    spi_select();
    spi_read(rb, 8);
    spi_deselect();
    {
        int ok = 1;
        for (int i = 0; i < 8; ++i)
            if (rb[i] != 0xFF) ok = 0;
        check(ok, "read returns the 0xFF fillers");
    }

    /* 5. Wide helpers, MSB first. */
    spi_select();
    check(spi_transfer16(0x1234) == 0x1234, "transfer16");
    check(spi_transfer32(0x89ABCDEF) == 0x89ABCDEFu, "transfer32");
    spi_send16(0xBEEF);
    spi_send32(0xDEADBEEFu);
    check(spi_read16() == 0xFFFF, "read16");
    check(spi_read32() == 0xFFFFFFFFu, "read32");
    spi_deselect();

    /* 6. All four modes. */
    for (unsigned int mode = 0; mode < 4; ++mode) {
        spi_set_mode(mode);
        spi_select();
        unsigned char a = spi_transfer(0x5A);
        unsigned char b = spi_transfer(0x3C);
        spi_deselect();
        check(a == 0x5A && b == 0x3C, "mode round trip");
    }
    spi_set_mode(0);

    check(!spi_busy(), "engine idle at the end");

    uart_puts(fails ? "BAD\r\n" : "GOOD\r\n");
    *result = fails ? 0xBAD : 0x600D;

    /* Park in a self-loop: the sim harness stops on 8 identical retires. */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}
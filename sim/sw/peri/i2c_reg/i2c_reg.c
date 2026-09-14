#include <stdint.h>
#include "uart.h"
#include "i2c.h"

/*
 * End-to-end I2C oracle: the common/i2c.h library against the sim's
 * i2c_slave_model (address 0x50, 16 byte registers, reset contents 0x10+i,
 * auto-incrementing pointer, first write byte sets the pointer).
 *
 * Covers, in order:
 *   1. read of a reset-content register (0x13 = 0x10+3)
 *   2. write_reg / read_reg round trip
 *   3. write_regs / read_regs burst with pointer auto-increment
 *   4. a 20-byte write (22 queued bytes total) -- longer than the 16-deep
 *      TX FIFO, so it exercises the feed-while-draining path of i2c_push
 *      and the pointer wrap at reg 15 -> 0
 *   5. probe of the present slave (ACK) and of an absent one (NACK), and
 *      the NACK error return of i2c_write
 *
 * Result marker: 0x600D / 0xBAD at D-mem 0x3000.
 *
 * SCL: the oracle sets CLKDIV=24 (~500 kHz at 50 MHz) purely so the sim
 * runs fast; on the board use i2c_begin_hz(100000) for a standard bus.
 */

static volatile uint32_t *const result = (volatile uint32_t *)0x3000;

#define SLAVE 0x50u
#define OTHER 0x51u  /* no device here in the sim -> must NACK */

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
    unsigned char rb[16];
    unsigned char wb[20];
    unsigned char v;
    int rc;

    uart_puts("\r\nI2C REG\r\n");

    /* ~500 kHz for sim speed (see header comment); engine on, IRQs off. */
    i2c_begin_hz(100000);
    i2c_set_clkdiv(24);

    /* 1. Reset contents. */
    rc = i2c_read_reg(SLAVE, 0x03, &v);
    check(rc == I2C_OK && v == 0x13, "reset content reg3=0x13");

    /* 2. write/read round trip. */
    rc = i2c_write_reg(SLAVE, 0x03, 0xAB);
    check(rc == I2C_OK, "write_reg reg3=0xAB");
    rc = i2c_read_reg(SLAVE, 0x03, &v);
    check(rc == I2C_OK && v == 0xAB, "read_reg reg3==0xAB");

    /* 3. Burst with auto-increment. */
    for (int i = 0; i < 4; ++i)
        wb[i] = (unsigned char)(0xC0 + i);
    rc = i2c_write_regs(SLAVE, 0x00, wb, 4);
    check(rc == I2C_OK, "write_regs 0..3");
    rc = i2c_read_regs(SLAVE, 0x00, rb, 4);
    check(rc == I2C_OK && rb[0] == 0xC0 && rb[1] == 0xC1 && rb[2] == 0xC2 &&
                   rb[3] == 0xC3,
          "read_regs 0..3 back in order");

    /* 4. Longer than the TX FIFO: 20 data bytes = 22 queued bytes. The
     * pointer wraps at 15 -> 0, so data bytes 17..20 (0x30..0x33) wrap into
     * regs 0..3 and regs 4..15 keep bytes 5..16 (0x24..0x2F). */
    for (int i = 0; i < 20; ++i)
        wb[i] = (unsigned char)(0x20 + i);
    rc = i2c_write_regs(SLAVE, 0x00, wb, 20);
    check(rc == I2C_OK, "write_regs 20 bytes (past FIFO depth)");
    rc = i2c_read_regs(SLAVE, 0x00, rb, 16);
    if (rc == I2C_OK) {
        int ok = 1;
        for (int i = 0; i < 4; ++i)
            if (rb[i] != (unsigned char)(0x30 + i)) ok = 0;
        for (int i = 4; i < 16; ++i)
            if (rb[i] != (unsigned char)(0x20 + i)) ok = 0;
        check(ok, "wrapped 20-byte write reads back");
    } else {
        check(0, "read_regs 16 after wrap");
    }

    /* 5. Presence and error returns. */
    rc = i2c_probe(SLAVE);
    check(rc == I2C_OK, "probe 0x50 ACK");
    rc = i2c_probe(OTHER);
    check(rc == I2C_ERR_NACK, "probe 0x51 NACK");
    rc = i2c_write(OTHER, wb, 2);
    check(rc == I2C_ERR_NACK, "write to 0x51 reports NACK");
    /* The NACKed write must not leave bytes queued: a following op to the
     * real slave starts clean. */
    rc = i2c_write_reg(SLAVE, 0x0F, 0x5A);
    check(rc == I2C_OK, "clean op after a NACKed one");
    rc = i2c_read_reg(SLAVE, 0x0F, &v);
    check(rc == I2C_OK && v == 0x5A, "read back after NACK recovery");

    uart_puts(fails ? "BAD\r\n" : "GOOD\r\n");
    *result = fails ? 0xBAD : 0x600D;

    /* Park in a self-loop: the sim harness stops on 8 identical retires. */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}
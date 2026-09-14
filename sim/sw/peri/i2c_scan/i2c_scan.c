#include <stdint.h>
#include "uart.h"
#include "i2c.h"

/*
 * I2C bus scanner, the Arduino-classic diagnostic: probes every address
 * 0x08..0x77 (the valid 7-bit range outside the reserved blocks) and lists
 * the ones that ACK.
 *
 * In the sim the only device is i2c_slave_model at 0x50, so the expected
 * result is exactly one ACK at 0x50 -- the oracle asserts that (marker
 * 0x600D / 0xBAD at D-mem 0x3000). On the board the same binary is a plain
 * diagnostic: it prints whatever lives on the bus, so a wiring or
 * pull-up fault shows up as an empty or wrong list.
 *
 * The scan result (ACK count + first ACK address) also lands in .data at
 * 0x3004/0x3008 for a board post-mortem without the serial line.
 */

static volatile uint32_t *const result  = (volatile uint32_t *)0x3000;
static volatile uint32_t *const n_found = (volatile uint32_t *)0x3004;
static volatile uint32_t *const first   = (volatile uint32_t *)0x3008;

#define SCAN_LO 0x08u
#define SCAN_HI 0x77u

int main(void)
{
    unsigned int found = 0;
    unsigned int first_addr = 0xFF;

    uart_puts("\r\nI2C SCAN\r\n");

    /* 100 kHz for the board; the sim tolerates it (a full scan is ~1.5M
     * cycles at CLKDIV=124). */
    i2c_begin_hz(100000);

    for (unsigned int a = SCAN_LO; a <= SCAN_HI; ++a) {
        if (i2c_probe((unsigned char)a) == I2C_OK) {
            uart_puts("ACK ");
            uart_put_hex32(a);
            uart_puts("\r\n");
            if (!found)
                first_addr = a;
            ++found;
        }
    }

    uart_puts("FOUND ");
    uart_put_hex32(found);
    uart_puts("\r\n");

    *n_found = found;
    *first   = first_addr;

    /* Sim oracle: exactly one device, at 0x50. On the board this marker is
     * meaningless (any device set is legitimate) -- read the list. */
    *result = (found == 1 && first_addr == 0x50) ? 0x600D : 0xBAD;
    uart_puts((found == 1 && first_addr == 0x50) ? "GOOD\r\n" : "BAD\r\n");

    /* Park in a self-loop: the sim harness stops on 8 identical retires. */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}
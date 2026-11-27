/*
 * SDRAM test for the Linux SoC on the board: writes the 8 MiB SDRAM from
 * 64 KiB up through the cache, fence.i (every dirty line written back), then
 * reads it back twice -- straight from the device with CACHE_BYPASS set, and
 * through the cache -- and prints the result on the 16550 (115200 8N1).
 * Two patterns: the address itself and its complement, so every data bit is
 * seen at 0 and at 1 and an aliased (stuck) address line shows up as a
 * mismatch. Load with: yboot.py <frame.yl>  (entry 0).
 * Result also at 0x2000: 0x600D / 0xBAD, error count at 0x2004.
 */
#include <stdint.h>
#include "../common/uart.h"

#define LO     0x00010000u
#define HI     0x00800000u
#define CCSR   ((volatile uint8_t *)0x00801000u)
#define RESULT ((volatile uint32_t *)0x00002000u)

static void putc_(char c)
{
    while (!(UART_LSR & UART_LSR_THRE))
        ;
    UART_THR = (uint8_t)c;
}
static void puts_(const char *s)
{
    while (*s) putc_(*s++);
}
static void hex(uint32_t v)
{
    for (int i = 28; i >= 0; i -= 4) putc_("0123456789abcdef"[(v >> i) & 15]);
}

static uint32_t errors;

static void check(const char *what, uint32_t inv)
{
    uint32_t bad = 0;
    for (uint32_t a = LO; a < HI; a += 4) {
        uint32_t want = a ^ inv;
        uint32_t got  = *(volatile uint32_t *)a;
        if (got != want) {
            if (bad < 8) {
                puts_("  MISMATCH ");
                puts_(what);
                puts_(" @");
                hex(a);
                puts_(" got ");
                hex(got);
                puts_(" want ");
                hex(want);
                puts_("\r\n");
            }
            bad++;
        }
        if ((a & 0xFFFFF) == 0xFFFFC) putc_('.');
    }
    puts_(" ");
    puts_(what);
    puts_(bad ? ": FAIL, errors=0x" : ": ok\r\n");
    if (bad) {
        hex(bad);
        puts_("\r\n");
    }
    errors += bad;
}

int main(void)
{
    puts_("\r\nsoc_memtest: SDRAM 0x");
    hex(LO);
    puts_("..0x");
    hex(HI);
    puts_("\r\n");
    for (uint32_t pass = 0; pass < 2; pass++) {
        uint32_t inv = pass ? 0xFFFFFFFFu : 0;
        puts_(pass ? "pattern ~addr\r\n" : "pattern addr\r\n");
        for (uint32_t a = LO; a < HI; a += 4) *(volatile uint32_t *)a = a ^ inv;
        __asm__ volatile("fence.i" ::: "memory");
        *CCSR = 1;
        check("device (bypass)", inv);
        *CCSR = 0;
        check("cached", inv);
    }
    RESULT[1] = errors;
    RESULT[0] = errors ? 0xBAD : 0x600D;
    puts_(errors ? "soc_memtest: FAIL\r\n" : "soc_memtest: PASS\r\n");
    for (;;)
        ;
    return 0;
}

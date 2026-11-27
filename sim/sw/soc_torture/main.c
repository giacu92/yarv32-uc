/*
 * Memory torture test for the Linux SoC: core + cache_cntrl + SDRAM.
 *
 * Phase A  30 000 random aligned stores and loads (byte / halfword / word,
 *          random address, random data) over a 64 KiB region R, mirrored into
 *          a shadow region S 64 KiB away. R is 8x the size of each cache, so
 *          the stream evicts through every set and way, dirty and clean.
 *          Every load from R is checked against the same load from S, and
 *          every 4096 operations the whole of R is compared word by word.
 * Phase B  fence.i (every dirty D line written back), then CACHE_BYPASS on:
 *          R and S are read straight from the SDRAM device and compared. A
 *          lost or misplaced writeback shows here and nowhere else.
 * Phase C  CACHE_BYPASS on, a fresh region F is written straight into the
 *          device (F was never cached); bypass off, F read back through the
 *          cache with byte / halfword / word loads.
 *
 * R and S both go through the cache in phase A, so a fault would have to
 * corrupt both identically to escape A -- and B reads the device directly.
 */
#include <stdint.h>

#define R_BASE 0x00100000u
#define S_BASE 0x00110000u
#define F_BASE 0x00140000u
#define SPAN   0x00010000u /* 64 KiB */
#define NOPS   30000
#define CCSR   ((volatile uint8_t *)0x00801000u)
#define RESULT ((volatile uint32_t *)0x00002000u)

static uint32_t lcg_state = 0x1234567u;
static uint32_t lcg(void)
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return lcg_state;
}

static void fail(uint32_t phase, uint32_t addr, uint32_t got, uint32_t want)
{
    RESULT[1] = phase;
    RESULT[2] = addr;
    RESULT[3] = got;
    RESULT[4] = want;
    RESULT[0] = 0xBAD;
    for (;;)
        ;
}

static int compare_all(uint32_t phase)
{
    volatile uint32_t *r = (volatile uint32_t *)R_BASE;
    volatile uint32_t *s = (volatile uint32_t *)S_BASE;
    for (uint32_t i = 0; i < SPAN / 4; i++) {
        uint32_t a = r[i], b = s[i];
        if (a != b) fail(phase, R_BASE + 4 * i, a, b);
    }
    return 0;
}

int main(void)
{
    /* Same initial content in R and S. */
    for (uint32_t i = 0; i < SPAN / 4; i++) {
        uint32_t v = lcg();
        ((volatile uint32_t *)R_BASE)[i] = v;
        ((volatile uint32_t *)S_BASE)[i] = v;
    }

    /* Phase A */
    for (uint32_t n = 0; n < NOPS; n++) {
        uint32_t x   = lcg();
        uint32_t off = (x >> 8) & (SPAN - 1);
        uint32_t d   = lcg();
        switch (x & 7) {
        case 0:
            *(volatile uint8_t *)(R_BASE + off) = (uint8_t)d;
            *(volatile uint8_t *)(S_BASE + off) = (uint8_t)d;
            break;
        case 1:
            off &= ~1u;
            *(volatile uint16_t *)(R_BASE + off) = (uint16_t)d;
            *(volatile uint16_t *)(S_BASE + off) = (uint16_t)d;
            break;
        case 2:
        case 3:
            off &= ~3u;
            *(volatile uint32_t *)(R_BASE + off) = d;
            *(volatile uint32_t *)(S_BASE + off) = d;
            break;
        case 4: {
            uint32_t a = *(volatile int8_t *)(R_BASE + off);
            uint32_t b = *(volatile int8_t *)(S_BASE + off);
            if (a != b) fail(0xA1, R_BASE + off, a, b);
            break;
        }
        case 5: {
            off &= ~1u;
            uint32_t a = *(volatile uint16_t *)(R_BASE + off);
            uint32_t b = *(volatile uint16_t *)(S_BASE + off);
            if (a != b) fail(0xA2, R_BASE + off, a, b);
            break;
        }
        default: {
            off &= ~3u;
            uint32_t a = *(volatile uint32_t *)(R_BASE + off);
            uint32_t b = *(volatile uint32_t *)(S_BASE + off);
            if (a != b) fail(0xA4, R_BASE + off, a, b);
            break;
        }
        }
        if ((n & 4095) == 4095) compare_all(0xA0);
    }
    compare_all(0xA0);

    /* Phase B: write back, then read the device directly. */
    __asm__ volatile("fence.i" ::: "memory");
    *CCSR = 1;
    compare_all(0xB0);
    *CCSR = 0;

    /* Phase C: device writes under bypass, cached reads after. */
    *CCSR = 1;
    lcg_state = 0xC0FFEEu;
    for (uint32_t i = 0; i < SPAN / 4; i++) ((volatile uint32_t *)F_BASE)[i] = lcg();
    *CCSR = 0;
    lcg_state = 0xC0FFEEu;
    for (uint32_t i = 0; i < SPAN / 4; i++) {
        uint32_t want = lcg();
        uint32_t addr = F_BASE + 4 * i;
        uint32_t w    = *(volatile uint32_t *)addr;
        if (w != want) fail(0xC4, addr, w, want);
        uint32_t h = *(volatile uint16_t *)(addr + 2);
        if (h != (want >> 16)) fail(0xC2, addr + 2, h, want >> 16);
        uint32_t b = *(volatile uint8_t *)(addr + 1);
        if (b != ((want >> 8) & 0xFF)) fail(0xC1, addr + 1, b, (want >> 8) & 0xFF);
    }

    RESULT[0] = 0x600D;
    for (;;)
        ;
    return 0;
}

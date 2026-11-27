/*
 * Memory torture test for the Linux SoC: core + cache_cntrl + SDRAM.
 *
 * Phase A  30 000 random aligned stores and loads (byte / halfword / word,
 *          random address, random data) over a 64 KiB region R, mirrored into
 *          a shadow region S 64 KiB away. R is 8x the size of each cache, so
 *          the stream evicts through every set and way, dirty and clean.
 *          Every load from R is checked against the same load from S, and
 *          every 4096 operations the whole of R is compared word by word.
 * Phase B  checksum of R through the cache, 4 KiB of R and S rewritten so
 *          the cache holds dirty lines, fence.i (every dirty D line written
 *          back), then CACHE_BYPASS on: R and S are read straight from the
 *          SDRAM device and each checksum must equal the cached one. A
 *          writeback lost by the fence.i flush shows here and nowhere else
 *          (mutation-checked; see the comment in main()).
 * Phase C  CACHE_BYPASS on, a fresh region F is written straight into the
 *          device (F was never cached); bypass off, F read back through the
 *          cache with byte / halfword / word loads.
 * Phase D  code writes: hand-encoded routines are stored into SDRAM through
 *          the D-cache with ordinary (halfword) stores, fence.i, then called.
 *          Several rounds, different code at the SAME addresses each round,
 *          so a stale I-cache line or an un-written-back D line executes the
 *          previous round's code. See the comment above phase_d().
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
#define W_OFF   0x00008000u /* phase B dirty window */
#define W_BYTES 0x00001000u
#define CCSR   ((volatile uint8_t *)0x00801000u)
#define RESULT ((volatile uint32_t *)0x00002000u)
#define D_BASE  0x00150000u /* phase D code region, 8 KiB, never cached before */
#define D_BYTES 0x00002000u
#define D_ROUNDS 4

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

/* Order-independent checksum of one region: sum of a per-word mix, so the
 * expected value can be updated word by word when words change. */
static uint32_t mix(uint32_t v, uint32_t i)
{
    return (v ^ (i * 0x9E3779B9u)) * 0x85EBCA6Bu;
}
static uint32_t csum(uint32_t base)
{
    volatile uint32_t *p = (volatile uint32_t *)base;
    uint32_t h = 0;
    for (uint32_t i = 0; i < SPAN / 4; i++) h += mix(p[i], i);
    return h;
}

/* ---- Phase D: code written through the D-cache, fence.i, executed ----
 *
 * The device is first prefilled under CACHE_BYPASS with 0x8082557D at every
 * word -- `c.li a0,-1; c.jr ra` -- so a routine whose new code never reached
 * the device (writeback lost) runs the prefill and returns -1 instead of
 * trapping. A trap anyway (half-written code) is reported as phase 0xDF.
 *
 * Every round rewrites, then one fence.i, then calls:
 *   A  0x8F8 (I set 0x47/0x48): c.li a0 | lui a1 | addi a1 STRADDLING the
 *      32-byte line boundary at 0x900 | c.add a0,a1 | c.jr ra. Same address
 *      every round: from round 1 on its lines sit in the I-cache. -> 0xD1
 *   B  0xC40 (set 0x62): lui a0 | addi a0 | jalr x0,0(ra), all 32-bit, same
 *      address every round.                                         -> 0xD2
 *   C  a fresh line per round (sets 0x50/0x55/0x6A/0x70, second 4 KiB),
 *      never fetched before: lui | addi | c.jr ra | c.nop.          -> 0xD3
 *   S  0xE80 (set 0x74), self-modifying: `sw a1,0(a2); fence.i;
 *      addi a0,x0,OLD; ret`, called with a1 = `addi a0,x0,NEW` and a2 = the
 *      address of the OLD instruction. By the time its fence.i retires the
 *      core has already fetched the OLD word, so only a fence.i that kills
 *      and refetches returns NEW.                                   -> 0xD4
 * .text sits in I sets 0x00..0x3C and the stack in D sets near
 * 0x7F, so nothing else evicts these lines between the stores and fence.i:
 * the flush, not a victim writeback, has to make them visible.
 * Mutation-checked (phase D run alone): flush not invalidating the I ways
 * -> 0xD4 round 0 (0xD1 round 1 with S removed); flush skipping the D
 * writeback -> 0xD1 round 0; execute not redirecting after fence.i -> 0xD4. */
typedef uint32_t (*code_fn)(uint32_t, uint32_t, uint32_t);

static void put16(uint32_t a, uint32_t h) { *(volatile uint16_t *)a = (uint16_t)h; }
static void put32(uint32_t a, uint32_t w) /* halfword stores: a may be 2 mod 4 */
{
    put16(a, w);
    put16(a + 2, w >> 16);
}
static uint32_t enc_lui(uint32_t rd, uint32_t v) { return (v & 0xFFFFF000u) | (rd << 7) | 0x37; }
static uint32_t enc_addi(uint32_t rd, uint32_t rs1, int32_t imm)
{
    return ((uint32_t)imm << 20) | (rs1 << 15) | (rd << 7) | 0x13;
}
/* lui/addi pair materialising v: hi part rounded so the signed lo fits */
static uint32_t hi20(uint32_t v) { return (v + 0x800u) & 0xFFFFF000u; }
static int32_t lo12(uint32_t v) { return (int32_t)(v - hi20(v)); }

#define C_LI(rd, imm) (0x4001u | ((((uint32_t)(imm)) >> 5 & 1) << 12) | ((rd) << 7) | \
                       (((uint32_t)(imm)&0x1F) << 2))
#define C_ADD_A0_A1 0x952Eu
#define C_JR_RA     0x8082u
#define C_NOP       0x0001u
#define RET32       0x00008067u /* jalr x0, 0(ra) */
#define SW_A1_A2    0x00B62023u /* sw a1, 0(a2) */
#define FENCE_I     0x0000100Fu
#define A0 10
#define A1 11

static void __attribute__((aligned(4), noreturn)) trap_d(void)
{
    uint32_t c, e, t;
    __asm__ volatile("csrr %0, mcause" : "=r"(c));
    __asm__ volatile("csrr %0, mepc" : "=r"(e));
    __asm__ volatile("csrr %0, mtval" : "=r"(t));
    fail(0xDF, e, c, t);
    for (;;)
        ;
}

static void phase_d(void)
{
    static const uint32_t fresh[D_ROUNDS] = {0x1A00, 0x1AA0, 0x1D40, 0x1E00};
    const uint32_t sa = D_BASE + 0x8F8, sb = D_BASE + 0xC40, ss = D_BASE + 0xE80;

    __asm__ volatile("csrw mtvec, %0" ::"r"(trap_d));
    *CCSR = 1;
    for (uint32_t a = D_BASE; a < D_BASE + D_BYTES; a += 4) *(volatile uint32_t *)a = 0x8082557Du;
    *CCSR = 0;

    for (uint32_t r = 0; r < D_ROUNDS; r++) {
        /* A */
        int32_t  ci = (int32_t)(r * 13) - 20;
        uint32_t x  = (0x12345u + r * 0x111u) << 12;
        int32_t  y  = (int32_t)(r * 0x255) - 0x400;
        uint32_t wa = (uint32_t)ci + x + (uint32_t)y;
        put16(sa + 0, C_LI(A0, ci));
        put32(sa + 2, enc_lui(A1, x));
        put32(sa + 6, enc_addi(A1, A1, y)); /* bytes 0x8FE..0x901 */
        put16(sa + 10, C_ADD_A0_A1);
        put16(sa + 12, C_JR_RA);
        /* B */
        uint32_t wb = 0xB0000000u ^ (r * 0x01030507u);
        put32(sb + 0, enc_lui(A0, hi20(wb)));
        put32(sb + 4, enc_addi(A0, A0, lo12(wb)));
        put32(sb + 8, RET32);
        /* C */
        uint32_t sc = D_BASE + fresh[r];
        uint32_t wc = 0xC0DE0000u + r * 0x1357u;
        put32(sc + 0, enc_lui(A0, hi20(wc)));
        put32(sc + 4, enc_addi(A0, A0, lo12(wc)));
        put16(sc + 8, C_JR_RA);
        put16(sc + 10, C_NOP);
        /* S */
        put32(ss + 0, SW_A1_A2);
        put32(ss + 4, FENCE_I);
        put32(ss + 8, enc_addi(A0, 0, 0x123));
        put32(ss + 12, RET32);

        __asm__ volatile("fence.i" ::: "memory");

        uint32_t g;
        g = ((code_fn)sa)(0, 0, 0);
        if (g != wa) fail(0xD1, sa, g, wa);
        g = ((code_fn)sb)(0, 0, 0);
        if (g != wb) fail(0xD2, sb, g, wb);
        g = ((code_fn)sc)(0, 0, 0);
        if (g != wc) fail(0xD3, sc, g, wc);
        g = ((code_fn)ss)(0, enc_addi(A0, 0, 0x300 + r), ss + 8);
        if (g != 0x300 + r) fail(0xD4, ss, g, 0x300 + r);
    }
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

    /* Phase B: fence.i must write back every dirty line, then the device is
     * read directly. Two traps make the obvious version vacuous, both found
     * by mutating the flush to skip its writeback:
     *  - compare_all just swept 128 KiB through an 8 KiB cache, which evicts
     *    every dirty line through the ordinary victim writeback, so at the
     *    fence.i nothing is dirty. W_WORDS words of R and S are therefore
     *    rewritten right before it (4 KiB each: R in one way, S in the other
     *    of the same sets) and nothing cached touches R or S after that.
     *  - R and S share cache sets, so a lost writeback leaves both stale at
     *    the same offset: device R vs device S cannot see it. Each is checked
     *    against a checksum taken through the cache instead, updated for the
     *    rewritten words. */
    uint32_t want = csum(R_BASE);
    for (uint32_t i = W_OFF / 4; i < (W_OFF + W_BYTES) / 4; i++) {
        uint32_t old = ((volatile uint32_t *)R_BASE)[i];
        uint32_t v   = lcg();
        ((volatile uint32_t *)R_BASE)[i] = v;
        ((volatile uint32_t *)S_BASE)[i] = v;
        want += mix(v, i) - mix(old, i);
    }
    __asm__ volatile("fence.i" ::: "memory");
    *CCSR = 1;
    uint32_t got_r = csum(R_BASE);
    uint32_t got_s = csum(S_BASE);
    compare_all(0xB0);
    *CCSR = 0;
    if (got_r != want) fail(0xB1, R_BASE, got_r, want);
    if (got_s != want) fail(0xB2, S_BASE, got_s, want);

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

    phase_d();

    RESULT[0] = 0x600D;
    for (;;)
        ;
    return 0;
}

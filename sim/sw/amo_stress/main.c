/*
 * A-extension stress program, written for the Spike cosim.
 *
 * A pseudo-random stream of atomic operations (every AMO*.W, plus a
 * compare-and-swap built from LR.W / SC.W) runs over an 8-word array and an
 * identical plain-C shadow. Every atomic's returned old value and the array
 * afterwards are compared with the shadow. main() returns 0x600D in a0 on
 * success, 0xBAD on the first mismatch -- the cosim compares every retire
 * against Spike anyway, so the return value is a smoke check.
 *
 * No MMIO: the whole run is inside the ISA model, so the cosim does not stop
 * early. Same LCG as quicksort, so Spike and RTL see the same sequence.
 */
#include <stdint.h>

#define N 8
#define ROUNDS 600

static volatile uint32_t arr[N];
static uint32_t shadow[N];

static uint32_t lcg_state = 12345;
static uint32_t lcg(void)
{
    lcg_state = lcg_state * 1103515245u + 12345u;
    return lcg_state >> 1;
}

#define AMO(op, ptr, val)                                                  \
    ({                                                                     \
        uint32_t old_;                                                     \
        __asm__ volatile(op " %0, %2, (%1)"                                \
                         : "=r"(old_)                                      \
                         : "r"(ptr), "r"(val)                              \
                         : "memory");                                      \
        old_;                                                              \
    })

/* Compare-and-swap via LR/SC; returns the value seen. Retries only on a
 * spurious SC failure (none should happen: one hart, nothing intervening). */
static uint32_t cas(volatile uint32_t *p, uint32_t expect, uint32_t desired,
                    uint32_t *sc_fails)
{
    uint32_t seen, fail;
    for (;;) {
        __asm__ volatile("lr.w %0, (%1)" : "=r"(seen) : "r"(p) : "memory");
        if (seen != expect)
            return seen;
        __asm__ volatile("sc.w %0, %2, (%1)"
                         : "=r"(fail)
                         : "r"(p), "r"(desired)
                         : "memory");
        if (!fail)
            return seen;
        ++*sc_fails;
    }
}

int main(void)
{
    uint32_t sc_fails = 0;
    for (int i = 0; i < N; i++) {
        arr[i] = lcg();
        shadow[i] = arr[i];
    }
    for (int r = 0; r < ROUNDS; r++) {
        uint32_t k = lcg();
        int idx = k % N;
        uint32_t v = lcg();
        if ((k >> 8) & 1)
            v |= 0x80000000u; /* exercise the signed corners of MIN/MAX */
        volatile uint32_t *p = &arr[idx];
        uint32_t s = shadow[idx], got, want = s, next;
        switch ((k >> 4) % 10) {
        case 0: got = AMO("amoswap.w", p, v); next = v; break;
        case 1: got = AMO("amoadd.w", p, v); next = s + v; break;
        case 2: got = AMO("amoxor.w", p, v); next = s ^ v; break;
        case 3: got = AMO("amoand.w", p, v); next = s & v; break;
        case 4: got = AMO("amoor.w", p, v); next = s | v; break;
        case 5: got = AMO("amomin.w", p, v);
                next = ((int32_t)s < (int32_t)v) ? s : v; break;
        case 6: got = AMO("amomax.w", p, v);
                next = ((int32_t)s > (int32_t)v) ? s : v; break;
        case 7: got = AMO("amominu.w", p, v); next = s < v ? s : v; break;
        case 8: got = AMO("amomaxu.w", p, v); next = s > v ? s : v; break;
        default: {
            /* CAS: half the time with the right expected value */
            uint32_t e = ((k >> 9) & 1) ? s : s ^ 1u;
            got = cas(p, e, v, &sc_fails);
            next = (s == e) ? v : s;
            break;
        }
        }
        if (got != want)
            return 0xBAD;
        shadow[idx] = next;
        for (int i = 0; i < N; i++)
            if (arr[i] != shadow[i])
                return 0xBAD;
    }
    return sc_fails ? 0xBAD : 0x600D;
}

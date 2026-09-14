#include "fft.h"
#include "plic.h"

/*
 * FFT coprocessor end-to-end oracle.
 *
 * Covers the three things the standalone sim/hw/fft_tb cannot: that the
 * peripheral is reachable at FFT_BASE through the real LSU + bridge +
 * crossbar path, that the 8 KiB two-page window decodes (CSR at +0x0000,
 * sample DATA at +0x1000 -- the only window in the map wider than 4 KiB),
 * and that the DONE interrupt reaches mip.MEIP through the PLIC as source
 * 5.
 *
 * It also exercises the sim/sw/common/fft.h and plic.h libraries as the
 * interface under test: the register pokes below all go through them, so
 * a library that drifts from the RTL fails here rather than in whatever
 * application first trips over it.
 *
 * The transforms here are deliberately the two whose answers are exact in
 * fixed point, so the oracle needs no reference data and no tolerance:
 *
 *   impulse  x[0] = 0x4000, rest 0, N = 8
 *            -> every bin = 0x4000/8 = 0x0800. Each stage halves an
 *               exact power of two, so no rounding happens anywhere.
 *   DC       x[i] = c for all i, N = 16
 *            -> bin 0 = c, every other bin exactly 0.
 *
 * Numeric accuracy is sim/hw/fft_tb's job (bit-exact model + double-DFT
 * cross-check over random vectors); this file is about the plumbing.
 *
 * Streaming is exercised as the peripheral is meant to be used: frame B
 * is written into the CPU-visible buffer WHILE the engine is still
 * transforming frame A, which is the whole reason for the ping-pong.
 *
 * Result marker: 0x600D (pass) / 0xBAD (fail) written to D-mem 0x3000.
 * Debug fields stay in .data so a board run can be read out of D-mem.
 */

/* start.S does not zero .bss and .bss is NOBITS, so an initialised
 * counter must live in .data or it is simply not in the image. */
#define IN_DATA __attribute__((section(".data")))

static volatile unsigned int IN_DATA fail_step = 0; /* first failing step */
static volatile unsigned int IN_DATA irq_seen = 0;
static volatile unsigned int IN_DATA claim_id = 0;
static volatile unsigned int IN_DATA bad_cause = 0;
static volatile unsigned int IN_DATA last_cycles = 0;

static volatile unsigned int *const result = (volatile unsigned int *)0x3000;

/* PLIC dispatcher handler for source 5. Servicing = clearing the sticky
 * DONE flag: the PLIC's CLAIM read is side-effect-free, so a handler that
 * does not do this re-takes forever (the livelock contract in plic.h). */
static void fft_isr(unsigned int src)
{
    claim_id = src;
    fft_irq_clear();
    irq_seen = irq_seen + 1;
}

/* Any interrupt or trap that is not the machine-external one is a
 * failure; record the cause and mask so the parked loop below is quiet. */
static void unexpected_isr(unsigned int mcause)
{
    bad_cause = mcause;
    sys_irq_global_disable();
}

/* Records the first failing step and returns 0, so run() can bail out
 * with the step number left in D-mem for a board post-mortem. */
static int fail(unsigned int step)
{
    if (fail_step == 0)
        fail_step = step;
    return 0;
}

static int run(void)
{
    unsigned int i;

    /* ---- 1. Register plumbing --------------------------------------- */
    /* Reset values prove the CSR page decodes, and that the 8 KiB window
     * did not land on top of a neighbour. */
    if (fft_log2n() != 10u)
        return fail(1);
    if (FFT_SCALE != FFT_SCALE_FULL)
        return fail(2);
    if (fft_busy() || fft_done())
        return fail(3);

    /* CONF is clamped on write, so a readback reports what is in force.
     * 15, not a bigger number: CONF is a 4-bit field and the hardware
     * clamps wdata[3:0], so writing 99 would land on 3 (in range) and
     * test nothing. */
    if (fft_begin(15u) != FFT_LOG2N_MAX)
        return fail(4);
    if (fft_begin(0u) != FFT_LOG2N_MIN)
        return fail(5);

    /* The DATA page is a different 4 KiB page of the same window: a store
     * there must not alias onto a CSR. */
    fft_begin(10u);
    fft_write(0, 0x5678, 0x1234);
    if (fft_read_re(0) != 0x5678 || fft_read_im(0) != 0x1234)
        return fail(6);
    if (fft_log2n() != 10u)
        return fail(7);
    /* Top of the 1024-word page, i.e. the far end of the DATA half. */
    fft_write(FFT_NMAX - 1u, -1, -1);
    if (fft_read_re(FFT_NMAX - 1u) != -1)
        return fail(8);

    /* ---- 2. Impulse, N = 8 ------------------------------------------ */
    /* log2n = 3 is odd, so this also runs the engine's copy pass. */
    fft_begin(3u);
    fft_clear(8u);
    fft_write_real(0, 0x4000);
    fft_start();
    if (!fft_wait())
        return fail(9);
    last_cycles = fft_cycles();
    fft_swap(); /* hand the result back without starting again */
    for (i = 0; i < 8u; i++) {
        if (fft_read_re(i) != 0x0800 || fft_read_im(i) != 0)
            return fail(10);
    }

    /* ---- 3. DC, N = 16 ---------------------------------------------- */
    fft_begin(4u);
    for (i = 0; i < 16u; i++)
        fft_write_real(i, 0x0100);
    fft_start();
    if (!fft_wait())
        return fail(11);
    fft_swap();
    if (fft_read_re(0) != 0x0100)
        return fail(12);
    for (i = 1; i < 16u; i++) {
        if (fft_read_re(i) != 0 || fft_read_im(i) != 0)
            return fail(13);
    }

    /* ---- 4. Streaming ping-pong ------------------------------------- */
    /* Frame A is an impulse (flat spectrum), frame B is DC (bin 0 only),
     * and B is written while A is still being transformed. If the two
     * buffers were not independent, B's samples would corrupt A's result
     * or vice versa, and the two spectra are chosen so either failure is
     * unmistakable. */
    fft_begin(3u);

    fft_clear(8u); /* frame A: impulse */
    fft_write_real(0, 0x4000);
    fft_start(); /* engine takes A; the CPU now sees the other buffer */

    if (!fft_busy())
        return fail(14); /* an 8-point transform must still be running */
    for (i = 0; i < 8u; i++) /* frame B: DC, written DURING frame A */
        fft_write_real(i, 0x0200);

    if (!fft_wait())
        return fail(15);
    fft_start(); /* engine takes B; the CPU now sees A's result */
    for (i = 0; i < 8u; i++) {
        if (fft_read_re(i) != 0x0800 || fft_read_im(i) != 0)
            return fail(16);
    }

    if (!fft_wait())
        return fail(17);
    fft_swap();
    if (fft_read_re(0) != 0x0200)
        return fail(18);
    for (i = 1; i < 8u; i++) {
        if (fft_read_re(i) != 0 || fft_read_im(i) != 0)
            return fail(19);
    }

    /* ---- 5. DONE interrupt through the PLIC -------------------------- */
    fft_begin(5u); /* N = 32, odd log2n -> copy pass under interrupt too */
    if (fft_irq_pending())
        return fail(20);

    sys_isr_install();
    sys_isr_set_unhandled(unexpected_isr);
    plic_attach(PLIC_SRC_FFT, fft_isr);
    sys_irq_source_enable(SYS_MIE_MEIE);
    sys_irq_global_enable();

    for (i = 0; i < 32u; i++)
        fft_write_real(i, 0x0080);
    fft_irq_enable();
    fft_start();

    {
        unsigned int spin = 0;
        while (irq_seen == 0u) {
            sys_wfi();
            if (++spin >= FFT_SPIN_LIMIT)
                return fail(21);
        }
    }

    sys_irq_global_disable();

    if (claim_id != PLIC_SRC_FFT)
        return fail(22);
    if (bad_cause != 0u)
        return fail(23);
    /* The handler's W1C must have cleared DONE and dropped the line. */
    if (fft_done())
        return fail(24);

    /* The interrupted transform still has to be correct: DC at 0x80 into
     * bin 0, nothing anywhere else. */
    fft_swap();
    if (fft_read_re(0) != 0x0080)
        return fail(25);
    for (i = 1; i < 32u; i++) {
        if (fft_read_re(i) != 0 || fft_read_im(i) != 0)
            return fail(26);
    }

    /* ---- 6. Spectrum helpers ---------------------------------------- */
    /* A real cosine at bin 3 must peak at bin 3 (and mirror at N-3), and
     * fft_peak_bin must find it over the non-mirrored half. The samples
     * are a hand-written 16-point cosine in Q15 at amplitude 0.5, so no
     * math library is needed. */
    {
        static const short cos3_16[16] = {16384, 6270,  -11585, -15137, 0,     15137, 11585, -6270,
                                          -16384, -6270, 11585,  15137,  0,     -15137, -11585, 6270};
        short out_re[16];
        short out_im[16];

        fft_begin(4u);
        if (!fft_transform_real(cos3_16, out_re, out_im))
            return fail(27);
        if (fft_peak_bin(1u, 8u) != 3u)
            return fail(28);
        if (fft_power(3u) == 0u || fft_mag(3u) == 0u)
            return fail(29);
        /* A real input of amplitude A splits into A/2 at k and A/2 at
         * N-k with conjugate symmetry, so bin 13 mirrors bin 3: equal
         * real parts, opposite imaginary parts. Compared with a +/-2 LSB
         * slack, not for equality -- the two halves reach the output
         * through different butterflies and therefore different
         * round-half-up points, so they agree in exact arithmetic but not
         * necessarily in the last bit. Exactness is checked where it
         * actually holds (the impulse and DC cases above) and against the
         * bit-exact model in sim/hw/fft_tb. */
        {
            int d_re = fft_read_re(13u) - fft_read_re(3u);
            int d_im = fft_read_im(13u) + fft_read_im(3u);
            if (d_re < 0)
                d_re = -d_re;
            if (d_im < 0)
                d_im = -d_im;
            if (d_re > 2 || d_im > 2)
                return fail(30);
        }
        /* And the bulk read must agree with the word-at-a-time read. */
        if (out_re[3] != (short)fft_read_re(3u))
            return fail(31);
        /* Bin spacing at a nominal 16 kHz rate: bin 3 of 16 is 3 kHz. */
        if (fft_bin_hz(3u, 16000u, 16u) != 3000u)
            return fail(32);
    }

    return 1;
}

int main(void)
{
    int ok = run();

    /* Leave interrupts masked whatever happened, so a stuck source cannot
     * storm the parked loop below. */
    sys_irq_global_disable();
    fft_irq_disable();

    *result = (ok && fail_step == 0u) ? 0x600Du : 0xBADu;

    /* Park in a self-loop, not in wfi: the sim harness stops on 8
     * identical retires, and a wfi with every interrupt masked would trip
     * its "halt with no wake" liveness check. */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}

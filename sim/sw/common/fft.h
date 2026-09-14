#ifndef FFT_H
#define FFT_H

/*
 * FFT coprocessor library (src/rtl/utils/axi4_lite_fft.sv) at
 * rv32_pkg::FFT_BASE = 0x1000_A000. Radix-2 decimation-in-frequency,
 * signed Q15 complex, one butterfly per cycle, 4 to 1024 points.
 *
 * The window is 8 KiB -- the only one in the map wider than 4 KiB --
 * split on bit 12:
 *
 *   +0x0000 CSR page
 *     0x00 CTRL   (RW) bit0 START (W1), bit1 IRQ_EN, bit2 INVERSE,
 *                      bit3 SWAP (W1). START and SWAP are ignored while
 *                      BUSY, and so are CONF and SCALE writes.
 *     0x04 STATUS (R)  bit0 BUSY, bit1 DONE (sticky), bit2 CPU_BUF,
 *                      bit3 IRQ (= DONE & IRQ_EN)
 *     0x08 IRQ    (R/W1C) bit0 DONE
 *     0x0C CONF   (RW) [3:0] LOG2N, CLAMPED TO [2,10] ON WRITE, so a
 *                      readback always reports the size in force.
 *     0x10 SCALE  (RW) bit s = divide by 2 after stage s. Reset all-ones.
 *     0x14 CYCLES (R)  clk cycles the last transform took.
 *   +0x1000 DATA page: the CPU-owned sample buffer, one 32-bit word per
 *           complex sample, {imag[31:16], real[15:0]}, both signed Q15.
 *
 * DATA-page writes have HALFWORD granularity: a word store writes both
 * halves, a halfword store writes the real or the imaginary part alone,
 * and a BYTE store writes NOTHING and is still acknowledged. The storage
 * is Gowin block RAM, which has no per-byte write enable. Every accessor
 * below uses word or halfword stores, so this only bites hand-written
 * pokes -- never build a sample with two `sb`s.
 *
 * ================= THE ONE THING TO GET RIGHT ==================
 *
 * The engine has TWO sample buffers and you only ever see one of them.
 * fft_start() swaps which: it hands the buffer you just filled to the
 * engine AND hands you back the buffer the engine used last, which holds
 * the PREVIOUS transform's result. So results are one frame behind, and
 * the order inside a streaming loop is fixed:
 *
 *     wait -> start -> read the previous result -> fill the next frame
 *
 * Reading after filling loses the result; filling before starting
 * overwrites it. fft_swap() drains the last frame without launching
 * another transform.
 *
 * If you do not care about streaming, use fft_transform_real() /
 * fft_transform(): they do fill/start/wait/swap/read for you and hide the
 * ping-pong completely.
 *
 * ====================== NUMERIC BEHAVIOUR ======================
 *
 * Samples and results are signed Q15: -1.0 is -32768, +1.0 - 2^-15 is
 * 32767. SCALE divides both butterfly outputs by 2 after the stages whose
 * bit is set; with the all-ones reset value the total gain is 1/N and no
 * stage can overflow, which is why that is the default. Clearing bits
 * keeps more low-order bits for small signals, at the cost of having to
 * know your headroom -- every write saturates rather than wrapping, so an
 * overflow clips instead of inverting the sign.
 *
 * A real input of amplitude A at bin k comes back as A/2 in bin k and A/2
 * in bin N-k (the conjugate half), because the default scaling is 1/N and
 * a real cosine splits its energy between the two.
 *
 * Output order is natural: bin i is at index i, no bit-reversal to undo.
 *
 * Timing: (LOG2N + (LOG2N odd ? 1 : 0)) * (N/2 + 4) cycles. 1024 points
 * is about 5160 cycles = 103 us at 50 MHz; 256 points about 1060 = 21 us.
 * Filling the buffer costs more than that for large N -- every sample is
 * one MMIO store -- so for a streaming application the fill, not the
 * transform, is the budget.
 *
 * Interrupts: DONE raises PLIC source 5. Nothing here arms anything;
 * call fft_irq_enable() and wire the source with plic.h + sys.h (see
 * LIBRARIES.md, and sim/sw/peri/fft for a worked example).
 */

#define FFT_BASE 0x1000A000u

#define FFT_CTRL   (*(volatile unsigned int *)(FFT_BASE + 0x00))
#define FFT_STATUS (*(volatile unsigned int *)(FFT_BASE + 0x04))
#define FFT_IRQ    (*(volatile unsigned int *)(FFT_BASE + 0x08))
#define FFT_CONF   (*(volatile unsigned int *)(FFT_BASE + 0x0C))
#define FFT_SCALE  (*(volatile unsigned int *)(FFT_BASE + 0x10))
#define FFT_CYCLES (*(volatile unsigned int *)(FFT_BASE + 0x14))
#define FFT_DATA   ((volatile unsigned int *)(FFT_BASE + 0x1000))

#define FFT_CTRL_START   (1u << 0)
#define FFT_CTRL_IRQ_EN  (1u << 1)
#define FFT_CTRL_INVERSE (1u << 2)
#define FFT_CTRL_SWAP    (1u << 3)

#define FFT_ST_BUSY    (1u << 0)
#define FFT_ST_DONE    (1u << 1)
#define FFT_ST_CPU_BUF (1u << 2)
#define FFT_ST_IRQ     (1u << 3)

/* Size limits. LOG2N below 2 or above 10 is clamped by the hardware. */
#define FFT_LOG2N_MIN 2u
#define FFT_LOG2N_MAX 10u
#define FFT_NMAX      1024u

/* Full 1/N scaling: the overflow-proof default. */
#define FFT_SCALE_FULL 0x3FFu
#define FFT_SCALE_NONE 0x000u

/* Bounded spin for fft_wait(). A 1024-point transform is ~5160 cycles and
 * the polling loop around it is a few tens of cycles per iteration, so
 * this is a diagnosis bound, not a tolerance. */
#define FFT_SPIN_LIMIT 2000000u

/* CTRL has write-only bits (START/SWAP), so the sticky ones are shadowed
 * here rather than read back -- a read-modify-write of CTRL would be a
 * spurious START if bit 0 ever read as 1. */
static unsigned int fft_ctrl_shadow = 0u;

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

/* Set the transform size (log2 of the point count) and reset the engine
 * to the default configuration: full 1/N scaling, forward transform,
 * interrupt disabled, DONE cleared. Ignored by the hardware while BUSY.
 * Returns the LOG2N actually in force (the hardware clamps to [2,10]). */
static unsigned int fft_begin(unsigned int log2n)
{
    fft_ctrl_shadow = 0u;
    FFT_CTRL = 0u;
    FFT_IRQ = 1u;
    FFT_CONF = log2n;
    FFT_SCALE = FFT_SCALE_FULL;
    return FFT_CONF;
}

static unsigned int fft_log2n(void)
{
    return FFT_CONF;
}

static unsigned int fft_points(void)
{
    return 1u << FFT_CONF;
}

/* Per-stage divide-by-2 mask; bit s applies after stage s. FFT_SCALE_FULL
 * (the reset value) is 1/N overall and cannot overflow. Ignored while
 * BUSY. */
static void fft_set_scale(unsigned int mask)
{
    FFT_SCALE = mask;
}

/* Forward (0) or inverse (1). Latched at START, so changing it mid-flight
 * does not disturb a running transform. */
static void fft_set_inverse(int on)
{
    if (on)
        fft_ctrl_shadow |= FFT_CTRL_INVERSE;
    else
        fft_ctrl_shadow &= ~FFT_CTRL_INVERSE;
    FFT_CTRL = fft_ctrl_shadow;
}

/* ------------------------------------------------------------------ */
/* Status                                                              */
/* ------------------------------------------------------------------ */

static unsigned int fft_busy(void)
{
    return (FFT_STATUS & FFT_ST_BUSY) != 0u;
}

static unsigned int fft_done(void)
{
    return (FFT_STATUS & FFT_ST_DONE) != 0u;
}

/* Which physical buffer the DATA page currently shows. Diagnostic only --
 * correct code tracks the sequence, not the bit. */
static unsigned int fft_cpu_buf(void)
{
    return (FFT_STATUS & FFT_ST_CPU_BUF) != 0u;
}

/* Cycles the last completed transform took, START to DONE. */
static unsigned int fft_cycles(void)
{
    return FFT_CYCLES;
}

/* ------------------------------------------------------------------ */
/* Sample access (the CPU-owned buffer)                                */
/* ------------------------------------------------------------------ */

static void fft_write(unsigned int i, int re, int im)
{
    FFT_DATA[i] = (((unsigned int)im & 0xFFFFu) << 16) | ((unsigned int)re & 0xFFFFu);
}

static void fft_write_real(unsigned int i, int re)
{
    FFT_DATA[i] = (unsigned int)re & 0xFFFFu;
}

static void fft_read(unsigned int i, int *re, int *im)
{
    unsigned int w = FFT_DATA[i];
    *re = (int)(short)(w & 0xFFFFu);
    *im = (int)(short)(w >> 16);
}

static int fft_read_re(unsigned int i)
{
    return (int)(short)(FFT_DATA[i] & 0xFFFFu);
}

static int fft_read_im(unsigned int i)
{
    return (int)(short)(FFT_DATA[i] >> 16);
}

/* Bulk fill of n samples. im may be 0 for a real-valued signal. */
static void fft_fill(const short *re, const short *im, unsigned int n)
{
    unsigned int i;
    if (im) {
        for (i = 0; i < n; i++)
            fft_write(i, re[i], im[i]);
    } else {
        for (i = 0; i < n; i++)
            fft_write_real(i, re[i]);
    }
}

static void fft_fill_real(const short *x, unsigned int n)
{
    fft_fill(x, 0, n);
}

/* Bulk read of n bins. Either pointer may be 0 if that half is not
 * wanted. */
static void fft_read_all(short *re, short *im, unsigned int n)
{
    unsigned int i;
    for (i = 0; i < n; i++) {
        unsigned int w = FFT_DATA[i];
        if (re)
            re[i] = (short)(w & 0xFFFFu);
        if (im)
            im[i] = (short)(w >> 16);
    }
}

/* Zero the whole CPU-owned buffer (n samples). Useful when the frame is
 * shorter than the transform, i.e. zero padding. */
static void fft_clear(unsigned int n)
{
    unsigned int i;
    for (i = 0; i < n; i++)
        FFT_DATA[i] = 0u;
}

/* ------------------------------------------------------------------ */
/* Running a transform                                                 */
/* ------------------------------------------------------------------ */

/* Hand the filled buffer to the engine and launch. The DATA page then
 * shows the OTHER buffer, holding the previous transform's result --
 * read it before filling the next frame. Ignored while BUSY. */
static void fft_start(void)
{
    FFT_IRQ = 1u;
    FFT_CTRL = fft_ctrl_shadow | FFT_CTRL_START;
}

/* Swap buffers WITHOUT launching: the way to see the last result at the
 * end of a stream. Ignored while BUSY. */
static void fft_swap(void)
{
    FFT_CTRL = fft_ctrl_shadow | FFT_CTRL_SWAP;
}

/* Block until DONE. Returns 1 on success, 0 if the spin bound expired
 * (which means the engine never finished -- a hardware or wiring fault,
 * not a slow transform). */
static int fft_wait(void)
{
    unsigned int spin = 0u;
    while ((FFT_STATUS & FFT_ST_DONE) == 0u) {
        if (++spin >= FFT_SPIN_LIMIT)
            return 0;
    }
    return 1;
}

/* One-shot blocking transform, ping-pong hidden. re/im are the input and
 * also receive the output; im may be 0 for a real input, in which case
 * out_im may still be requested separately via fft_read_all(). Uses the
 * size already set by fft_begin(). Returns 1 on success. */
static int fft_transform(const short *in_re, const short *in_im, short *out_re, short *out_im)
{
    unsigned int n = fft_points();
    fft_fill(in_re, in_im, n);
    fft_start();
    if (!fft_wait())
        return 0;
    fft_swap();
    fft_read_all(out_re, out_im, n);
    return 1;
}

static int fft_transform_real(const short *in, short *out_re, short *out_im)
{
    return fft_transform(in, 0, out_re, out_im);
}

/* ------------------------------------------------------------------ */
/* Spectrum helpers                                                    */
/* ------------------------------------------------------------------ */

/* |X[k]|^2, exact. Fits in 32 bits unsigned for any Q15 pair. Prefer this
 * to fft_mag() when you only need to compare or threshold bins. */
static unsigned int fft_power(unsigned int k)
{
    int re = fft_read_re(k);
    int im = fft_read_im(k);
    return (unsigned int)(re * re) + (unsigned int)(im * im);
}

/* Magnitude, alpha-max-plus-beta-min approximation: max + 3*min/8. No
 * sqrt (there is no libm and no FPU here). Error is at most 6.8% and
 * always an under- or over-estimate of at most that -- fine for peak
 * picking, not for calibrated measurement. */
static unsigned int fft_mag(unsigned int k)
{
    int re = fft_read_re(k);
    int im = fft_read_im(k);
    unsigned int a = (unsigned int)(re < 0 ? -re : re);
    unsigned int b = (unsigned int)(im < 0 ? -im : im);
    unsigned int hi = a > b ? a : b;
    unsigned int lo = a > b ? b : a;
    return hi + (3u * lo) / 8u;
}

/* Index of the largest-magnitude bin in [first, last]. Start at 1 to skip
 * DC, and stop at N/2 for a real input (bins above N/2 are the conjugate
 * mirror of those below). */
static unsigned int fft_peak_bin(unsigned int first, unsigned int last)
{
    unsigned int best = first;
    unsigned int best_p = fft_power(first);
    unsigned int k;
    for (k = first + 1u; k <= last; k++) {
        unsigned int p = fft_power(k);
        if (p > best_p) {
            best_p = p;
            best = k;
        }
    }
    return best;
}

/* Centre frequency of bin k, in Hz, for a given sample rate. Bins are
 * sample_hz/N apart; the result is truncated.
 *
 * Split into quotient and remainder rather than computing k*sample_hz/n
 * directly, for two reasons: k*sample_hz overflows 32 bits above a ~4 MHz
 * sample rate, and widening it to `unsigned long long` would pull
 * __udivdi3 out of libgcc -- which is NOT linked in this freestanding
 * build (only dhrystone adds -lgcc), so it would fail at link time with an
 * undefined symbol rather than anywhere near this line. Both terms here
 * stay well inside 32 bits: (n-1)*k < n*n <= 2^20. */
static unsigned int fft_bin_hz(unsigned int k, unsigned int sample_hz, unsigned int n)
{
    return (sample_hz / n) * k + ((sample_hz % n) * k) / n;
}

/* ------------------------------------------------------------------ */
/* Interrupt plumbing (nothing is armed unless you call these)         */
/* ------------------------------------------------------------------ */

/* Raise PLIC source 5 while DONE is set. Attach a handler with
 * plic_attach(PLIC_SRC_FFT, fn) and enable MEIE + global (sys.h). */
static void fft_irq_enable(void)
{
    fft_ctrl_shadow |= FFT_CTRL_IRQ_EN;
    FFT_CTRL = fft_ctrl_shadow;
}

static void fft_irq_disable(void)
{
    fft_ctrl_shadow &= ~FFT_CTRL_IRQ_EN;
    FFT_CTRL = fft_ctrl_shadow;
}

/* W1C the sticky DONE flag. THIS IS WHAT COMPLETES THE INTERRUPT: the
 * PLIC's claim read is side-effect-free, so a handler that does not clear
 * DONE here re-takes forever (the livelock contract in plic.h). */
static void fft_irq_clear(void)
{
    FFT_IRQ = 1u;
}

static unsigned int fft_irq_pending(void)
{
    return FFT_IRQ & 1u;
}

#endif /* FFT_H */

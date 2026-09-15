#include "sys.h"
#include "uart.h"
#include "fft.h"
#include "i2s.h"

#if !TUNER_SIM
#include "ssd1306_text.h"
#endif

/*
 * Guitar tuner: INMP441 I2S microphone in, FFT coprocessor for the pitch,
 * SSD1306 OLED out with a needle gauge.
 *
 * Signal chain
 * ------------
 *   INMP441 (24-bit I2S MEMS microphone, left slot)
 *     -> axi4_lite_i2s in MASTER mode, which is what paces the whole
 *        program: the FPGA generates BCLK/LRCK, the mic answers, and a
 *        sample arrives every 1/TUNER_FS_IN_HZ whether the CPU is ready or
 *        not
 *     -> 24-bit sample >> 8 into Q15
 *     -> 2nd-order CIC decimation by TUNER_DECIM
 *     -> DC removal, triangular window
 *     -> axi4_lite_fft, 1/N scaling, natural output order
 *     -> magnitude spectrum, fundamental pick, parabolic interpolation
 *     -> nearest chromatic note + cents
 *     -> note name, frequency, needle gauge, verdict
 *
 * Why an FFT and not autocorrelation: the coprocessor is there. A
 * 1024-point transform costs ~103 us at 50 MHz against ~512 ms to
 * acquire the frame, so the pitch maths is free and the ADC is the whole
 * budget.
 *
 * RESOLUTION, which is the number that decides whether a tuner is
 * usable. Bin spacing is fs/N = 1.953 Hz, and a cent at the low E (82.41
 * Hz) is 0.048 Hz -- so raw bin picking is 40 cents coarse and useless.
 * Parabolic interpolation over the peak and its two neighbours recovers
 * roughly a tenth of a bin, ~0.2 Hz, which is ~4 cents at E2 and ~1 cent
 * at E4. Frame time and resolution trade off exactly (frame = N/fs, bin =
 * fs/N, product 1), so the only real lever is N.
 *
 * Interpolating on LINEAR magnitudes with a triangular window is the
 * cheap choice; a Hann window with log magnitudes is the textbook one and
 * is measurably better, but Hann needs a 1024-entry table or a libm this
 * build does not have, and a log needs the same. The measured error of
 * what is here is in the self-test below.
 *
 * Picking the fundamental
 * -----------------------
 * NOT the largest bin: a bright pickup routinely makes the 2nd harmonic
 * louder than the fundamental, and a tuner that reports the octave is
 * worse than no tuner. Instead: find the largest bin in the guitar band,
 * then rescan from the bottom and take the FIRST local maximum that
 * reaches a quarter of it. The fundamental is always the lowest partial,
 * so the lowest strong peak is it. (Harmonic product spectrum is the
 * other standard answer and is better at low SNR; it needs bins up to 3k,
 * which runs off the top of the band here.)
 *
 * Board wiring
 * ------------
 *   MIC   INMP441 on the I2S pins: SCK -> PIN80 (BCLK), WS -> PIN81
 *         (LRCK), SD -> PIN82, L/R -> GND (left slot), VDD 3.3 V.
 *         No bias network and no anti-alias op-amp: the part is digital
 *         and outputs 24-bit two's complement, which is the main reason
 *         this replaced the MCP3208 front end.
 *   OLED  SSD1306 128x64 at 0x3C on the I2C master.
 *
 * WHY THE PERIPHERAL RUNS IN MASTER MODE, which is not a detail:
 * an INMP441 is itself an I2S clock slave -- it has no oscillator and
 * generates nothing. With the FPGA also a slave, nobody clocks the bus and
 * no amount of firmware fixes it. So axi4_lite_i2s generates BCLK and
 * LRCK here (i2s_master_hz), and the mic answers.
 *
 * Simulation
 * ----------
 * Neither device is modelled in the sim, so -DTUNER_SIM=1 replaces the
 * ADC with a synthetic oscillator and the display with UART, and runs a
 * self-check over known frequencies: exact notes and deliberate +/-20
 * cent offsets, verifying the note choice and the cents error. That path
 * exercises the whole real chain except the two peripherals -- the
 * window, the FFT hardware, the peak pick, the interpolation and the note
 * maths. It writes 0x600D / 0xBAD to D-mem 0x3000 like the other oracles.
 */

/* ---------------------------------------------------------------- */
/* Configuration                                                     */
/* ---------------------------------------------------------------- */

#define TUNER_CORE_HZ 50000000u
#define TUNER_LOG2N   10u
#define TUNER_N       (1u << TUNER_LOG2N)

/*
 * SAMPLE RATES, and why the analysis rate is ~2 kHz and not 20.
 *
 * The transform's rate is NOT an audio-quality choice, it is a
 * time-frequency trade: bin spacing is fs/N and frame time is N/fs, so
 * their product is 1 no matter what. At fs = 2 kHz with N = 1024 the bins
 * are 1.95 Hz and a frame is 0.512 s. Raise fs to 20 kHz and keep N and
 * the bins become 19.5 Hz -- 41 cents at the low E, which is not a tuner.
 * Keeping the bins AND the rate would need N = 16384, i.e. three buffers
 * of 64 KiB against a device with 103 KiB of BSRAM in total. It does not
 * fit, and it would not help: a guitar's highest fundamental is ~1.2 kHz
 * at the top fret, so everything above 2.5 kHz of Nyquist is spent on
 * harmonics the pitch estimator throws away.
 *
 * What a fast rate IS worth is ANTI-ALIASING. A plucked string has strong
 * partials well past 1 kHz, and at a 2 kHz sample rate every one of them
 * folds straight into the band the fundamental lives in. So the ADC runs
 * at TUNER_FS_IN_HZ and a CIC decimator drops it to the analysis rate,
 * which puts the filter's nulls exactly on the frequencies that fold to
 * DC and does it with two adders instead of an analog brick wall.
 *
 * The mic's own decimation filter handles everything above ITS Nyquist
 * (7.8 kHz), so unlike the old analog front end there is no RC network to
 * get right -- but the fold from 7.8 kHz down to 977 Hz is still ours, and
 * that is what the CIC below is for.
 *
 * WHY 15625 Hz AND NOT 16000. The input rate is now set by the FPGA, since
 * the peripheral is the I2S master: fs = clk / (4 * (DIV+1) * SLOT). With
 * clk = 50 MHz = 2^7 x 5^8 and a standard 64-BCLK frame, an integer rate
 * needs (DIV+1) to be a power of 5, and 25 is the only one in the audio
 * band -- 15625 Hz, BCLK exactly 1.000 MHz. 16 kHz is NOT reachable from
 * this clock, and picking a nearby non-integer rate would make every
 * frequency constant below an approximation. So the whole chain is built
 * on 15625 Hz, and the analysis rate is 15625/8 = 1953.125 Hz.
 *
 * That last number is not an integer, which is why the constants below
 * carry the rate in MILLI-Hz: 1953125 mHz is exact, and every conversion
 * stays in integer arithmetic.
 */
#define TUNER_FS_IN_HZ 15625u
#define TUNER_DECIM    8u
#define TUNER_I2S_SLOT 32u                             /* BCLK per channel slot */
#define TUNER_FS_MHZ   ((TUNER_FS_IN_HZ * 1000u) / TUNER_DECIM) /* 1953125 mHz */

/* Search band. Wide enough for D2 (73.4 Hz) to G#4 (415.3 Hz), the
 * chromatic table below, with a little margin either side. */
#define TUNER_FMIN_HZ 70u
#define TUNER_FMAX_HZ 420u
#define TUNER_KMIN    ((TUNER_FMIN_HZ * TUNER_N * 1000u) / TUNER_FS_MHZ)
#define TUNER_KMAX \
    (((TUNER_FMAX_HZ * TUNER_N * 1000u) + TUNER_FS_MHZ - 1u) / TUNER_FS_MHZ)

/*
 * Bin -> centi-Hz: f_cHz = kq8 * fs_mHz / (10 * N * 256), with kq8 the
 * peak position in 1/256 of a bin. At fs = 1953.125 Hz and N = 1024 that
 * reduces to kq8 * 390625 / 2^19 -- exact, no rounding of the constant
 * itself.
 *
 * The two assertions are the reduction, split so neither overflows 32
 * bits: 2^SHIFT = 512*N pins the denominator to N, and NUM*5 = fs_mHz
 * pins the numerator to the sample rate. Change either the rate or the
 * transform size and the build stops here instead of reporting wrong
 * frequencies.
 */
#define TUNER_CHZ_NUM   390625u
#define TUNER_CHZ_SHIFT 19u
_Static_assert((1u << TUNER_CHZ_SHIFT) == 512u * TUNER_N,
               "TUNER_CHZ_SHIFT must be log2(512*N)");
_Static_assert(TUNER_CHZ_NUM * 5u == TUNER_FS_MHZ,
               "TUNER_CHZ_NUM must be fs_mHz/5");

#define TUNER_I2C_HZ 400000u

/* The INMP441 sends 24 bits, MSB first, in the left slot (L/R tied low).
 * Shift the top 16 into the Q15 the rest of the chain works in: the
 * bottom 8 bits of a MEMS mic are its own noise floor, and keeping them
 * would only make the CIC accumulators wider. */
#define TUNER_IN_SHIFT 8u

/* Below this peak magnitude there is nothing to tune. With 1/N scaling a
 * sinusoid of Q15 amplitude A lands near A/4 after the triangular window,
 * so 200 is about A = 800, i.e. -32 dBFS. An INMP441 is -26 dBFS at 94
 * dBSPL, so a guitar played near the mic clears this comfortably; drop it
 * if the mic sits across the room, raise it if a noisy room reports
 * phantom notes. */
#define TUNER_SIGNAL_FLOOR 200u

/* A quarter of the strongest bin is "a real partial". */
#define TUNER_PEAK_FRACTION 4u

/* |cents| at or below this lights the in-tune indicator. */
#define TUNER_IN_TUNE_CENTS 5

/* ---------------------------------------------------------------- */
/* Chromatic note table, D2 .. G#4, frequencies in centi-Hz          */
/* (A4 = 440). Names are 3 chars so the display field never resizes. */
/* ---------------------------------------------------------------- */

typedef struct {
    unsigned int cHz;
    const char  *name;
} tuner_note_t;

static const tuner_note_t tuner_notes[] = {
    {   7342, "D2 " },  /* MIDI 38,   73.416 Hz */
    {   7778, "D#2" },  /* MIDI 39,   77.782 Hz */
    {   8241, "E2 " },  /* MIDI 40,   82.407 Hz */
    {   8731, "F2 " },  /* MIDI 41,   87.307 Hz */
    {   9250, "F#2" },  /* MIDI 42,   92.499 Hz */
    {   9800, "G2 " },  /* MIDI 43,   97.999 Hz */
    {  10383, "G#2" },  /* MIDI 44,  103.826 Hz */
    {  11000, "A2 " },  /* MIDI 45,  110.000 Hz */
    {  11654, "A#2" },  /* MIDI 46,  116.541 Hz */
    {  12347, "B2 " },  /* MIDI 47,  123.471 Hz */
    {  13081, "C3 " },  /* MIDI 48,  130.813 Hz */
    {  13859, "C#3" },  /* MIDI 49,  138.591 Hz */
    {  14683, "D3 " },  /* MIDI 50,  146.832 Hz */
    {  15556, "D#3" },  /* MIDI 51,  155.563 Hz */
    {  16481, "E3 " },  /* MIDI 52,  164.814 Hz */
    {  17461, "F3 " },  /* MIDI 53,  174.614 Hz */
    {  18500, "F#3" },  /* MIDI 54,  184.997 Hz */
    {  19600, "G3 " },  /* MIDI 55,  195.998 Hz */
    {  20765, "G#3" },  /* MIDI 56,  207.652 Hz */
    {  22000, "A3 " },  /* MIDI 57,  220.000 Hz */
    {  23308, "A#3" },  /* MIDI 58,  233.082 Hz */
    {  24694, "B3 " },  /* MIDI 59,  246.942 Hz */
    {  26163, "C4 " },  /* MIDI 60,  261.626 Hz */
    {  27718, "C#4" },  /* MIDI 61,  277.183 Hz */
    {  29366, "D4 " },  /* MIDI 62,  293.665 Hz */
    {  31113, "D#4" },  /* MIDI 63,  311.127 Hz */
    {  32963, "E4 " },  /* MIDI 64,  329.628 Hz */
    {  34923, "F4 " },  /* MIDI 65,  349.228 Hz */
    {  36999, "F#4" },  /* MIDI 66,  369.994 Hz */
    {  39200, "G4 " },  /* MIDI 67,  391.995 Hz */
    {  41530, "G#4" },  /* MIDI 68,  415.305 Hz */
};
#define TUNER_NOTE_N (sizeof(tuner_notes) / sizeof(tuner_notes[0]))

/*
 * Decimation factor. Constant on the board; the self-check makes it a
 * variable so it can run the pitch cases at the analysis rate (where the
 * decimator is a pass-through and each frame costs an eighth as much) and
 * the anti-aliasing cases at the real input rate. gcc folds the board
 * form away entirely.
 */
/* The CIC's own gain is R^order, so the output shift is 2*log2(R) and
 * the two MUST move together -- changing the rate without the shift
 * scales every sample by 64 and the peak vanishes under the noise floor.
 * tuner_set_decim() is what makes that impossible to get wrong. */
#define TUNER_CIC_SHIFT 6u /* order 2, R = 8 */

#if TUNER_SIM
static unsigned int tuner_decim = TUNER_DECIM;
static unsigned int tuner_cic_shift = TUNER_CIC_SHIFT;

static void tuner_set_decim(unsigned int r)
{
    unsigned int sh = 0u;

    tuner_decim = r;
    while (r > 1u) {
        r >>= 1;
        sh += 2u; /* two integrator/comb stages */
    }
    tuner_cic_shift = sh;
}
#else
#define tuner_decim     TUNER_DECIM
#define tuner_cic_shift TUNER_CIC_SHIFT
#endif

/* Result of one frame. */
typedef struct {
    int          valid;  /* a pitch was found at all */
    unsigned int f_cHz;  /* estimated fundamental, centi-Hz */
    unsigned int peak;   /* magnitude of the chosen bin */
    int          note;   /* index into tuner_notes */
    int          cents;  /* deviation from that note, signed */
} tuner_result_t;

/* Magnitude of every usable bin. Read once per frame so the peak search
 * and the interpolation work on memory instead of re-reading MMIO. */
static unsigned short tuner_mag[TUNER_N / 2u];

/* ---------------------------------------------------------------- */
/* Acquisition                                                       */
/* ---------------------------------------------------------------- */

#if TUNER_SIM
/* Synthetic source for the self-check: a phase accumulator into a
 * quarter-sine table, plus a second and third harmonic so the peak pick
 * has to do its job rather than seeing a bare sinusoid. The third
 * harmonic is the LOUDEST partial on purpose -- that is the case where
 * picking the largest bin reports the wrong octave. */
static const short tuner_sinq[257] = {
         0,    201,    402,    603,    804,   1005,   1206,   1407,
      1608,   1809,   2009,   2210,   2410,   2611,   2811,   3012,
      3212,   3412,   3612,   3811,   4011,   4210,   4410,   4609,
      4808,   5007,   5205,   5404,   5602,   5800,   5998,   6195,
      6393,   6590,   6786,   6983,   7179,   7375,   7571,   7767,
      7962,   8157,   8351,   8545,   8739,   8933,   9126,   9319,
      9512,   9704,   9896,  10087,  10278,  10469,  10659,  10849,
     11039,  11228,  11417,  11605,  11793,  11980,  12167,  12353,
     12539,  12725,  12910,  13094,  13279,  13462,  13645,  13828,
     14010,  14191,  14372,  14553,  14732,  14912,  15090,  15269,
     15446,  15623,  15800,  15976,  16151,  16325,  16499,  16673,
     16846,  17018,  17189,  17360,  17530,  17700,  17869,  18037,
     18204,  18371,  18537,  18703,  18868,  19032,  19195,  19357,
     19519,  19680,  19841,  20000,  20159,  20317,  20475,  20631,
     20787,  20942,  21096,  21250,  21403,  21554,  21705,  21856,
     22005,  22154,  22301,  22448,  22594,  22739,  22884,  23027,
     23170,  23311,  23452,  23592,  23731,  23870,  24007,  24143,
     24279,  24413,  24547,  24680,  24811,  24942,  25072,  25201,
     25329,  25456,  25582,  25708,  25832,  25955,  26077,  26198,
     26319,  26438,  26556,  26674,  26790,  26905,  27019,  27133,
     27245,  27356,  27466,  27575,  27683,  27790,  27896,  28001,
     28105,  28208,  28310,  28411,  28510,  28609,  28706,  28803,
     28898,  28992,  29085,  29177,  29268,  29358,  29447,  29534,
     29621,  29706,  29791,  29874,  29956,  30037,  30117,  30195,
     30273,  30349,  30424,  30498,  30571,  30643,  30714,  30783,
     30852,  30919,  30985,  31050,  31113,  31176,  31237,  31297,
     31356,  31414,  31470,  31526,  31580,  31633,  31685,  31736,
     31785,  31833,  31880,  31926,  31971,  32014,  32057,  32098,
     32137,  32176,  32213,  32250,  32285,  32318,  32351,  32382,
     32412,  32441,  32469,  32495,  32521,  32545,  32567,  32589,
     32609,  32628,  32646,  32663,  32678,  32692,  32705,  32717,
     32728,  32737,  32745,  32752,  32757,  32761,  32765,  32766,
     32767,
};

static unsigned int tuner_sim_phase;
static unsigned int tuner_sim_inc;

/*
 * Phase increment per INPUT sample: 2^32 / (fs * 100) per centi-Hz, which
 * is 2748.779 at the 15625 Hz input rate and 21990.233 at the 1953.125 Hz
 * analysis rate. Split into an integer and a thousandths term rather than
 * rounded, which would be ~1e-4 off and cost ~0.2 cents -- visible against
 * a 3-cent budget. The split form is ~1e-7 off, which is not. A 64-bit
 * intermediate would be exact and would also pull __udivdi3, which is not
 * linked.
 */
static unsigned int tuner_sim_pure;

static void tuner_sim_set(unsigned int f_cHz, unsigned int decim, int pure)
{
    tuner_sim_phase = 0u;
    tuner_sim_pure  = (unsigned int)pure;

    /* Only the two rates the self-check uses. 2^32/(1953.125*100) =
     * 21990.233 and 2^32/(15625*100) = 2748.779. */
    if (decim == 1u) {
        tuner_sim_inc = f_cHz * 21990u + (f_cHz * 233u) / 1000u;
    } else {
        tuner_sim_inc = f_cHz * 2748u + (f_cHz * 779u) / 1000u;
    }
}

/* sin(phase) in Q15, phase Q32, quarter table with quadrant folding and
 * linear interpolation between entries. */
static int tuner_sin(unsigned int phase)
{
    unsigned int idx  = (phase >> 22) & 0x3FFu; /* 0..1023 */
    unsigned int frac = (phase >> 6) & 0xFFFFu; /* within one step */
    unsigned int quad = idx >> 8;
    unsigned int i    = idx & 0xFFu;
    int a, b, v;

    if (quad & 1u) {
        a = tuner_sinq[256u - i];
        b = tuner_sinq[255u - i];
    } else {
        a = tuner_sinq[i];
        b = tuner_sinq[i + 1u];
    }
    v = a + (int)(((b - a) * (int)frac) >> 16);
    return (quad & 2u) ? -v : v;
}

static int tuner_mic_sample(void)
{
    int s;

    /* Fundamental at -12 dBFS, 2nd at -14, 3rd at -10: the third is the
     * strongest partial, which is exactly the trap. The anti-aliasing
     * cases want one clean tone instead, so that the magnitude ratio they
     * measure is the filter's and not a harmonic's. */
    if (tuner_sim_pure) {
        s = tuner_sin(tuner_sim_phase) >> 1;
    } else {
        s = (tuner_sin(tuner_sim_phase) >> 2) + (tuner_sin(tuner_sim_phase * 2u) >> 2) +
            (tuner_sin(tuner_sim_phase * 3u) >> 1);
    }
    tuner_sim_phase += tuner_sim_inc;

    /* Q15, the same scale the real path produces after its shift. */
    return s;
}
#else
/*
 * One microphone sample, blocking. The I2S stream is the program's clock:
 * the mic answers the BCLK this peripheral generates, so a sample appears
 * every 1/TUNER_FS_IN_HZ regardless of what the CPU is doing, and reading
 * the FIFO is what paces the acquisition loop. No mcycle busy-wait, which
 * is what the ADC front end needed.
 *
 * The RX FIFO is 16 words -- about 1 ms at this rate -- so anything that
 * blocks the loop for longer loses samples. The display push is the only
 * such thing and it happens between frames, where a discontinuity costs
 * nothing: tuner_acquire() flushes first, so each frame is contiguous.
 */
static int tuner_mic_sample(void)
{
    int s;

    (void)i2s_read_wait(&s);
    return s >> TUNER_IN_SHIFT;
}
#endif

/*
 * Fill the CPU-visible FFT buffer with one windowed frame.
 *
 * Two passes over the buffer, not one, because the DC offset is only
 * known once every sample has been taken: pass one stores raw codes and
 * accumulates the sum, pass two reads each back, removes the mean,
 * windows it and writes it again. A one-pole high-pass would be single
 * pass but would tilt the low end of the band, which is precisely where
 * the low E sits.
 *
 * Window is triangular. It costs ~26 dB of sidelobe against a rectangle
 * and needs no table -- Hann would need 1024 Q15 entries or a cosine this
 * build has no way to compute.
 *
 * DECIMATION. The mic runs at TUNER_FS_IN_HZ and a second-order CIC
 * drops it by TUNER_DECIM to the analysis rate. A CIC of order N and
 * rate R is N integrators at the fast rate, a downsample, and N combs at
 * the slow rate -- no multipliers, no coefficients, and its nulls land
 * exactly on the multiples of the output rate, which is precisely where
 * everything that aliases to DC comes from. Order 2 buys about 30 dB at
 * the worst fold against 15 dB for a plain boxcar, for one more adder.
 *
 * The integrators are allowed to wrap. That is not sloppiness: a CIC is
 * correct under two's-complement wrapping as long as the accumulator is
 * wide enough for the filter's own gain (R^order = 64 here against
 * 12-bit inputs, so 19 bits -- a 32-bit int has room to spare), because
 * the comb stage subtracts the wrap back out. The transient at the start
 * of each frame is two output samples long and lands where the window is
 * zero anyway.
 *
 * On the board the sample clock is the I2S stream itself -- the mic is
 * clocked by this peripheral and answers on its own schedule, so reading
 * the FIFO paces the loop and there is no mcycle busy-wait left. The FIFO
 * is flushed first, so the frame is contiguous even though the previous
 * frame's display push (1 KiB over I2C at 400 kHz, ~25 ms) overran it.
 * The display push is deliberately NOT done here for that reason.
 */
static void tuner_acquire(void)
{
    unsigned int i, d;
    int          mean;
    int          sum = 0;
    /* CIC state: two integrators, two comb delays. */
    int          i1 = 0, i2 = 0, d1 = 0, d2 = 0;

#if !TUNER_SIM
    /* Start the frame on a fresh sample: whatever queued up during the
     * previous frame's display push is stale by up to 25 ms, and a frame
     * stitched across that gap is a frame with a step in it. */
    i2s_flush();
#endif

    for (i = 0; i < TUNER_N; i++) {
        int s, c1, c2;

        for (d = 0; d < tuner_decim; d++) {
            i1 += tuner_mic_sample();
            i2 += i1;
        }

        c1 = i2 - d1;
        d1 = i2;
        c2 = c1 - d2;
        d2 = c1;

        /* Divide out the CIC's gain, R^order, back to the input's own
         * scale. An arithmetic shift, so it rounds toward -inf; the bias
         * is half an LSB of a 12-bit code and the DC removal below eats
         * it whole. At R = 1 the shift is 0 and the whole structure
         * collapses to an identity, which is what lets the pitch cases
         * bypass it. */
        s = c2 >> tuner_cic_shift;

        sum += s;
        fft_write_real(i, s);
    }

    mean = sum / (int)TUNER_N;

    for (i = 0; i < TUNER_N; i++) {
        int          s = fft_read_re(i) - mean;
        unsigned int t = (i < TUNER_N / 2u) ? i : (TUNER_N - 1u - i);
        /* Triangular window in Q15: 0 at the ends, ~1 in the middle. */
        int          w = (int)((t * 2u * 32768u) / TUNER_N);
        if (w > 32767) {
            w = 32767;
        }
        fft_write_real(i, (s * w) >> 15);
    }
}

/* ---------------------------------------------------------------- */
/* Analysis                                                          */
/* ---------------------------------------------------------------- */

static void tuner_read_spectrum(void)
{
    unsigned int k;

    for (k = 0; k < TUNER_N / 2u; k++) {
        tuner_mag[k] = (unsigned short)fft_mag(k);
    }
}

/*
 * Index of the fundamental, or -1 if the band is quiet.
 *
 * See the header: the loudest bin is often a harmonic, so the loudest is
 * only used to set a threshold, and the answer is the LOWEST local
 * maximum that reaches a quarter of it.
 */
static int tuner_find_fundamental(unsigned int *peak_out)
{
    unsigned int k, best = TUNER_KMIN, bestm = 0u, thr;

    for (k = TUNER_KMIN; k <= TUNER_KMAX; k++) {
        if (tuner_mag[k] > bestm) {
            bestm = tuner_mag[k];
            best  = k;
        }
    }

    *peak_out = bestm;
    if (bestm < TUNER_SIGNAL_FLOOR) {
        return -1;
    }

    thr = bestm / TUNER_PEAK_FRACTION;
    for (k = TUNER_KMIN + 1u; k < TUNER_KMAX; k++) {
        if (tuner_mag[k] >= thr && tuner_mag[k] > tuner_mag[k - 1u] &&
            tuner_mag[k] >= tuner_mag[k + 1u]) {
            *peak_out = tuner_mag[k];
            return (int)k;
        }
    }

    return (int)best;
}

/*
 * Sub-bin peak position by parabolic interpolation, in 1/256 of a bin.
 *
 *   delta = 0.5 * (a - c) / (a - 2b + c),   a,b,c = mag[k-1], mag[k], mag[k+1]
 *
 * b is the local maximum so the denominator is negative; a flat top gives
 * zero and is handled. The result is clamped to +/-0.5 bin, which is all
 * the estimator can mean.
 */
static int tuner_interp_q8(unsigned int k)
{
    int a = (int)tuner_mag[k - 1u];
    int b = (int)tuner_mag[k];
    int c = (int)tuner_mag[k + 1u];
    int den = a - 2 * b + c;
    int d;

    if (den == 0) {
        return 0;
    }
    d = ((a - c) * 128) / den;
    if (d > 128) {
        d = 128;
    } else if (d < -128) {
        d = -128;
    }
    return d;
}

/*
 * Nearest chromatic note and the deviation in cents.
 *
 * cents = 1200 * log2(f/f0) is linearised to 1731 * (f - f0) / f0. Over
 * the half semitone that matters the error is under 0.7 cents at the very
 * edge and essentially zero near the middle -- and the middle is the only
 * place a tuner has to be right. No log, no libm, no FPU.
 */
static void tuner_note_of(unsigned int f_cHz, int *note, int *cents)
{
    int i, best = 0, best_c = 0, best_abs = 1 << 30;

    for (i = 0; i < (int)TUNER_NOTE_N; i++) {
        int f0 = (int)tuner_notes[i].cHz;
        int c  = (1731 * ((int)f_cHz - f0)) / f0;
        int ac = (c < 0) ? -c : c;

        if (ac < best_abs) {
            best_abs = ac;
            best_c   = c;
            best     = i;
        }
    }

    *note  = best;
    *cents = best_c;
}

/*
 * Peak position (1/256 of a bin) -> centi-Hz.
 *
 * The exact expression is kq8 * 390625 >> 19, and 390625 needs 19 bits
 * while kq8 reaches ~57000, so the product does NOT fit in 32 bits. Split
 * the multiplicand instead of the constant: six bits of kq8 go into the
 * second term, and each half is rounded rather than truncated, which
 * keeps the whole conversion within 1 centi-Hz -- 0.2 cents at the low E,
 * against a 3-cent estimator. A 64-bit intermediate would be exact and
 * would pull __udivdi3, which is not linked.
 */
static unsigned int tuner_kq8_to_cHz(unsigned int kq8)
{
    unsigned int hi = kq8 >> 6;
    unsigned int lo = kq8 & 63u;
    unsigned int sh = TUNER_CHZ_SHIFT - 6u;

    return ((hi * TUNER_CHZ_NUM + (1u << (sh - 1u))) >> sh)
         + ((lo * TUNER_CHZ_NUM + (1u << (TUNER_CHZ_SHIFT - 1u))) >> TUNER_CHZ_SHIFT);
}

/* One frame, from the transformed spectrum already sitting in the FFT's
 * CPU-visible buffer. */
static void tuner_analyse(tuner_result_t *r)
{
    unsigned int peak = 0u, kq8;
    int k;

    tuner_read_spectrum();
    k = tuner_find_fundamental(&peak);

    r->peak = peak;
    if (k < 0) {
        r->valid = 0;
        r->f_cHz = 0u;
        r->note  = 0;
        r->cents = 0;
        return;
    }

    kq8      = (unsigned int)(((int)k << 8) + tuner_interp_q8((unsigned int)k));
    r->f_cHz = tuner_kq8_to_cHz(kq8);
    r->valid = 1;
    tuner_note_of(r->f_cHz, &r->note, &r->cents);
}

/* ---------------------------------------------------------------- */
/* Display (board build)                                             */
/* ---------------------------------------------------------------- */
#if !TUNER_SIM

/* Gauge geometry. The needle spans +/-TUNER_GAUGE_SPAN cents across
 * +/-TUNER_GAUGE_HALF pixels either side of centre. */
#define TUNER_GAUGE_CX    64
#define TUNER_GAUGE_HALF  56
#define TUNER_GAUGE_SPAN  50
#define TUNER_GAUGE_AXIS  39
#define TUNER_GAUGE_TOP   30
#define TUNER_GAUGE_BOT   48

static int tuner_gauge_x(int cents)
{
    int x;

    if (cents > TUNER_GAUGE_SPAN) {
        cents = TUNER_GAUGE_SPAN;
    } else if (cents < -TUNER_GAUGE_SPAN) {
        cents = -TUNER_GAUGE_SPAN;
    }
    x = TUNER_GAUGE_CX + (cents * TUNER_GAUGE_HALF) / TUNER_GAUGE_SPAN;
    if (x < 1) {
        x = 1;
    } else if (x > (int)SSD1306_WIDTH - 2) {
        x = (int)SSD1306_WIDTH - 2;
    }
    return x;
}

/* Axis, the five reference ticks, and the centre mark. Drawn every frame
 * because the framebuffer is cleared every frame -- 1 KiB of RAM is
 * cheaper than tracking dirty rectangles. */
static void tuner_draw_scale(void)
{
    static const int ticks[] = {-50, -25, 0, 25, 50};
    unsigned int i;

    ssd1306_hline(6u, SSD1306_WIDTH - 7u, TUNER_GAUGE_AXIS, 1);

    for (i = 0; i < sizeof(ticks) / sizeof(ticks[0]); i++) {
        unsigned int x = (unsigned int)tuner_gauge_x(ticks[i]);
        if (ticks[i] == 0) {
            ssd1306_vline(x, TUNER_GAUGE_TOP, TUNER_GAUGE_BOT, 1);
        } else {
            ssd1306_vline(x, TUNER_GAUGE_AXIS - 4u, TUNER_GAUGE_AXIS + 4u, 1);
        }
    }
}

static void tuner_draw_needle(int cents)
{
    unsigned int x = (unsigned int)tuner_gauge_x(cents);

    ssd1306_fill_rect(x - 1u, TUNER_GAUGE_TOP, 3u, TUNER_GAUGE_BOT - TUNER_GAUGE_TOP + 1u, 1);
}

/* Centred string, optionally on an inverted bar (which is how "IN TUNE"
 * announces itself without needing a second font). */
static void tuner_banner(unsigned int y, const char *s, int inverted)
{
    unsigned int w = ssd1306_text_width(s, 1u);
    unsigned int x = (SSD1306_WIDTH - w) / 2u;

    if (inverted) {
        ssd1306_fill_rect(x - 2u, y - 1u, w + 3u, SSD1306_CHAR_H + 1u, 1);
        ssd1306_text(x, y, s, 0);
    } else {
        ssd1306_text(x, y, s, 1);
    }
}

static void tuner_render(const tuner_result_t *r)
{
    ssd1306_clear();

    if (!r->valid) {
        tuner_banner(12u, "NO SIGNAL", 0);
        tuner_banner(32u, "PLUCK A STRING", 0);
        ssd1306_update();
        return;
    }

    /* Note name, double height, left. The table's names are padded to 3
     * characters so a sharp appearing or disappearing does not shift the
     * frequency field next to it. */
    ssd1306_text_scaled(2u, 0u, tuner_notes[r->note].name, 2u, 1);

    /* Measured frequency, two decimals, right of the note name. */
    {
        unsigned int x = ssd1306_fixed(62u, 0u, (int)r->f_cHz, 2u, 1u, 1);
        ssd1306_text(x, 0u, "HZ", 1);
    }

    /* Signed cents, and the target the needle is measured against. */
    {
        unsigned int x = ssd1306_int(62u, 10u, r->cents, 4u, 1u, 1);
        ssd1306_text(x, 10u, "CT", 1);
    }

    tuner_draw_scale();
    tuner_draw_needle(r->cents);

    if (r->cents > TUNER_IN_TUNE_CENTS) {
        tuner_banner(54u, "SHARP", 0);
    } else if (r->cents < -TUNER_IN_TUNE_CENTS) {
        tuner_banner(54u, "FLAT", 0);
    } else {
        tuner_banner(54u, "IN TUNE", 1);
    }

    ssd1306_update();
}
#endif /* !TUNER_SIM */

/* ---------------------------------------------------------------- */
/* UART reporting (both builds -- the board one is a debug console)   */
/* ---------------------------------------------------------------- */

static void tuner_put_uint(unsigned int v)
{
    char buf[12];
    unsigned int n = 0;

    do {
        buf[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v && n < sizeof(buf));
    while (n--) {
        uart_putc(buf[n]);
    }
}

static void tuner_put_int(int v)
{
    if (v < 0) {
        uart_putc('-');
        v = -v;
    }
    tuner_put_uint((unsigned int)v);
}

/* centi-Hz as "82.41" */
static void tuner_put_chz(unsigned int v)
{
    tuner_put_uint(v / 100u);
    uart_putc('.');
    uart_putc((char)('0' + (v / 10u) % 10u));
    uart_putc((char)('0' + v % 10u));
}

static void tuner_report(const tuner_result_t *r)
{
    if (!r->valid) {
        uart_puts("-- no signal (peak ");
        tuner_put_uint(r->peak);
        uart_puts(")\r\n");
        return;
    }
    uart_puts(tuner_notes[r->note].name);
    uart_puts("  ");
    tuner_put_chz(r->f_cHz);
    uart_puts(" Hz  ");
    if (r->cents >= 0) {
        uart_putc('+');
    }
    tuner_put_int(r->cents);
    uart_puts(" cents  peak ");
    tuner_put_uint(r->peak);
    uart_puts("\r\n");
}

/* ---------------------------------------------------------------- */
/* Entry points                                                      */
/* ---------------------------------------------------------------- */

#if TUNER_SIM

/*
 * Self-check. Each case drives the synthetic oscillator at a known
 * frequency and asserts the reported note and cents. The oscillator's
 * third harmonic is the loudest partial, so a build that picked the
 * largest bin would report the octave-and-a-fifth above and fail every
 * case -- that is the property this test exists to pin.
 */
typedef struct {
    unsigned int f_cHz;
    int          note;
    int          cents;
} tuner_case_t;

static const tuner_case_t tuner_cases[] = {
    {   8241,  2,    0 },  /* E2    +0 cents ->   82.407 Hz */
    {   8146,  2,  -20 },  /* E2   -20 cents ->   81.460 Hz */
    {  11000,  7,    0 },  /* A2    +0 cents ->  110.000 Hz */
    {  14854, 12,   20 },  /* D3   +20 cents ->  148.538 Hz */
    {  19600, 17,    0 },  /* G3    +0 cents ->  195.998 Hz */
    {  24552, 21,  -10 },  /* B3   -10 cents ->  245.519 Hz */
    {  32963, 26,    0 },  /* E4    +0 cents ->  329.628 Hz */
    {  33539, 26,   30 },  /* E4   +30 cents ->  335.389 Hz */
    {  38415, 29,  -35 },  /* G4   -35 cents ->  384.150 Hz */
};
#define TUNER_CASE_N (sizeof(tuner_cases) / sizeof(tuner_cases[0]))

/* Budget for the whole estimator: window, interpolation, the cents
 * linearisation and the oscillator's own phase quantisation. Not a
 * tolerance chosen to make the test pass -- the measured worst error is
 * printed on every run, so a regression shows up as the number moving
 * even while the test still passes. */
#define TUNER_CENTS_TOL 8

static volatile unsigned int *const tuner_result = (volatile unsigned int *)0x3000;

/* One frame end to end: drive the oscillator, acquire, transform,
 * analyse. Returns 0 if the engine never finished. */
static int tuner_run_frame(unsigned int f_cHz, int pure, tuner_result_t *r)
{
    tuner_sim_set(f_cHz, tuner_decim, pure);
    tuner_acquire();
    fft_start();
    if (!fft_wait()) {
        uart_puts("FFT never finished\r\n");
        return 0;
    }
    fft_swap();
    tuner_analyse(r);
    return 1;
}

/*
 * Anti-aliasing check, and the only part that runs at the full input
 * rate. Two pure tones of the same amplitude: D4 at 293.66 Hz, which the
 * decimator must pass, and 1706.34 Hz, which is 2000 - 293.66 and folds
 * onto exactly the same bin. A second-order CIC at R = 8 puts about 30 dB
 * between them; the test asks for 18, so it fails a missing or first-order
 * filter without being sensitive to the exact shape.
 */
#define TUNER_ALIAS_IN_CHZ   29366u /* D4 */
#define TUNER_ALIAS_OUT_CHZ 170634u /* 2000 Hz - D4: folds back onto D4 */
#define TUNER_ALIAS_REJECT  8u      /* required magnitude ratio */

static unsigned int tuner_check_alias(void)
{
    tuner_result_t in, out;
    unsigned int   fails = 0u;

    tuner_set_decim(TUNER_DECIM);

    if (!tuner_run_frame(TUNER_ALIAS_IN_CHZ, 1, &in)) {
        return 1u;
    }
    uart_puts("in-band   ");
    tuner_report(&in);

    if (!tuner_run_frame(TUNER_ALIAS_OUT_CHZ, 1, &out)) {
        return 1u;
    }
    uart_puts("aliasing  ");
    tuner_report(&out);

    /* The in-band tone must survive the decimator and still read as D4. */
    if (!in.valid) {
        uart_puts("   ^ decimator killed the passband\r\n");
        fails++;
    }

    /* And the out-of-band one must not come back as a pitch. */
    if (out.peak * TUNER_ALIAS_REJECT > in.peak) {
        uart_puts("   ^ alias rejection only ");
        tuner_put_uint(in.peak / (out.peak ? out.peak : 1u));
        uart_puts("x\r\n");
        fails++;
    } else {
        uart_puts("alias rejection ");
        tuner_put_uint(in.peak / (out.peak ? out.peak : 1u));
        uart_puts("x\r\n");
    }

    tuner_set_decim(1u);
    return fails;
}

/*
 * The one part of the board path the synthetic oscillator cannot cover:
 * the I2S front end itself. sim_top answers master-mode clocks with an
 * INMP441-shaped slave (24-bit, left slot, a ramp for data), so this
 * checks what the tuner depends on and the oscillator bypasses -- that
 * the peripheral produces the designed sample rate, that the mic's words
 * arrive in order, and that the rate is what the frequency constants
 * assume. It does NOT check pitch: the model plays a ramp, not a note.
 */
static unsigned int tuner_check_i2s(void)
{
    unsigned int fails = 0u, fs, i;
    int          prev, cur;
    unsigned int t0, dt;

    fs = i2s_master_hz(TUNER_FS_IN_HZ, TUNER_I2S_SLOT);
    uart_puts("I2S master fs ");
    tuner_put_uint(fs);
    uart_puts(" Hz\r\n");
    if (fs != TUNER_FS_IN_HZ) {
        uart_puts("   ^ rate is not the one the constants assume\r\n");
        fails++;
    }

    i2s_begin(I2S_WLEN_24, I2S_CHAN_LEFT, I2S_FMT_I2S);
    i2s_flush();

    /* The model's word for frame m is m << 4, left slot only. */
    (void)i2s_read_wait(&prev);
    t0 = sys_cycle();
    for (i = 0; i < 4u; i++) {
        if (i2s_read_wait(&cur) != I2S_CH_LEFT) {
            uart_puts("   ^ word came back on the wrong channel\r\n");
            fails++;
            break;
        }
        if (cur != prev + 16) {
            uart_puts("   ^ mic stream lost a frame\r\n");
            fails++;
            break;
        }
        prev = cur;
    }
    dt = (sys_cycle() - t0) / 4u;

    /* One frame is TUNER_CORE_HZ / fs core cycles: 3200 at 50 MHz and
     * 15625 Hz. A wrong divider shows up here as a wrong period long
     * before it shows up as a wrong pitch. */
    uart_puts("frame period ");
    tuner_put_uint(dt);
    uart_puts(" cycles\r\n");
    {
        unsigned int want = TUNER_CORE_HZ / TUNER_FS_IN_HZ;
        unsigned int lo = want - want / 16u, hi = want + want / 16u;
        if (dt < lo || dt > hi) {
            uart_puts("   ^ not the designed frame period\r\n");
            fails++;
        }
    }

    /* Hand the bus back: the pitch cases below feed the estimator from
     * the synthetic oscillator, not from the mic. */
    i2s_end();
    i2s_master_off();
    return fails;
}

int main(void)
{
    tuner_result_t r;
    unsigned int   i, fails = 0u, worst = 0u;

    uart_puts("\r\nguitar tuner self-check\r\n");
    fft_begin(TUNER_LOG2N);

    fails += tuner_check_i2s();

    /* Pitch cases run with the decimator as a pass-through and the
     * oscillator at the analysis rate: same estimator, an eighth of the
     * simulated cycles. The decimator gets its own test below, where the
     * input rate is what is under test. */
    tuner_set_decim(1u);

    for (i = 0; i < TUNER_CASE_N; i++) {
        int err;

        if (!tuner_run_frame(tuner_cases[i].f_cHz, 0, &r)) {
            fails++;
            break;
        }
        tuner_report(&r);

        if (!r.valid || r.note != tuner_cases[i].note) {
            uart_puts("   ^ wrong note, wanted ");
            uart_puts(tuner_notes[tuner_cases[i].note].name);
            uart_puts("\r\n");
            fails++;
            continue;
        }

        err = r.cents - tuner_cases[i].cents;
        if (err < 0) {
            err = -err;
        }
        if ((unsigned int)err > worst) {
            worst = (unsigned int)err;
        }
        if (err > TUNER_CENTS_TOL) {
            uart_puts("   ^ cents error ");
            tuner_put_uint((unsigned int)err);
            uart_puts("\r\n");
            fails++;
        }
    }

    /* Silence must read as "no signal", not as a note picked out of the
     * rounding noise: inc = 0 holds the oscillator at zero, so after DC
     * removal the frame is all zeros. */
    if (tuner_run_frame(0u, 0, &r)) {
        tuner_report(&r);
        if (r.valid) {
            uart_puts("   ^ silence reported as a pitch\r\n");
            fails++;
        }
    } else {
        fails++;
    }

    fails += tuner_check_alias();

    uart_puts("worst cents error ");
    tuner_put_uint(worst);
    uart_puts(", failures ");
    tuner_put_uint(fails);
    uart_puts("\r\n");

    *tuner_result = fails ? 0xBADu : 0x600Du;

    /* Park in a self-loop: the sim harness stops on 8 identical retires. */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}

#else /* board */

int main(void)
{
    tuner_result_t r;

    uart_puts("\r\nYARV guitar tuner\r\n");

    /* The peripheral clocks the mic: an INMP441 generates nothing on its
     * own. The rate it reports back is the one the chain is built on --
     * 15625 Hz is exact from a 50 MHz core, 16000 Hz is not reachable at
     * all (see the SAMPLE RATES note above), so a mismatch here means the
     * core clock moved and every frequency constant below is now wrong. */
    {
        unsigned int fs = i2s_master_hz(TUNER_FS_IN_HZ, TUNER_I2S_SLOT);

        uart_puts("I2S master, fs = ");
        tuner_put_uint(fs);
        uart_puts(" Hz\r\n");
        if (fs != TUNER_FS_IN_HZ) {
            uart_puts("!! not the designed rate -- pitches will be off by "
                      "that ratio\r\n");
        }
    }
    i2s_begin(I2S_WLEN_24, I2S_CHAN_LEFT, I2S_FMT_I2S);

    i2c_begin_hz(TUNER_I2C_HZ);

    if (ssd1306_begin(SSD1306_ADDR) != I2C_OK) {
        uart_puts("SSD1306 did not ACK at 0x3C -- check wiring/address\r\n");
    }
    ssd1306_clear();
    tuner_banner(20u, "YARV TUNER", 0);
    tuner_banner(36u, "E A D G B E", 0);
    ssd1306_update();

    fft_begin(TUNER_LOG2N);

    /* Prime the ping-pong: one frame goes to the engine before the loop,
     * so that from then on every acquisition overlaps a transform. Here
     * that overlap is nearly free -- acquiring 1024 samples at 2 kHz is
     * 512 ms against ~103 us of FFT -- but the structure is the one the
     * peripheral is built for and costs nothing to keep. */
    tuner_acquire();
    fft_start();

    for (;;) {
        tuner_acquire();
        if (!fft_wait()) {
            uart_puts("FFT stalled\r\n");
            continue;
        }
        fft_start();

        tuner_analyse(&r);
        tuner_render(&r);
        tuner_report(&r);

        /* An overrun INSIDE a frame means the acquisition loop could not
         * keep up with the mic, which would corrupt the spectrum rather
         * than merely interrupt it -- worth saying out loud. (The gap
         * around the display push does not show up here: tuner_acquire()
         * flushes, and flushing clears the flag.) */
        if (i2s_overrun()) {
            uart_puts("!! I2S overrun during acquisition\r\n");
            i2s_overrun_clear();
        }
    }
}

#endif /* TUNER_SIM */

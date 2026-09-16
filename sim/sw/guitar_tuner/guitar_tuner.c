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
 *   MIC   INMP441 on the I2S pins: SCK -> PIN73 (BCLK), WS -> PIN74
 *         (LRCK), SD -> PIN75, L/R -> GND (left slot), VDD 3.3 V.
 *         Those three are free BANK1 header pins at 3.3 V; the board's
 *         own I2S_* pins are its audio OUTPUT path and are not usable
 *         for a microphone.
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
/*
 * HOP -- how often a new transform is launched, in decimated samples.
 *
 * The window stays TUNER_N long, so the RESOLUTION does not change: bins
 * are fs/N = 1.907 Hz whatever the hop is. What the hop changes is how
 * often that window is re-examined. At 256 (75% overlap) a new reading
 * lands every 256/1953.125 = 131 ms instead of every 524 ms, which is the
 * difference between a needle that steps and a needle that moves.
 *
 * What it does NOT buy, and this is the usual misunderstanding: each
 * reading still averages the last 0.52 s of signal. Overlap re-samples the
 * same sliding window more often; it does not shorten the window.
 *
 * Cost is negligible -- the transform is 103 us against a 131 ms hop --
 * but note that the whole window has to be rewritten into the coprocessor
 * every time, not just the new samples: after START the CPU gets the OTHER
 * ping-pong buffer, which holds the previous RESULT, not the previous
 * input.
 */
#define TUNER_HOP 256u

/*
 * DC blocker, replacing the per-frame mean.
 *
 *   y[n] = x[n] - x[n-1] + a*y[n-1],   a = 1 - 2^-TUNER_DC_SHIFT
 *
 * A mean needs the whole frame before it can be subtracted, which was fine
 * when frames were independent and is not when samples stream into a ring:
 * it would mean a second pass over 1024 samples at every hop. This runs
 * once per sample and keeps no history.
 *
 * THE POLE IS THE THING TO GET RIGHT. The corner is (1-a)*fs/(2*pi): at
 * shift 8 and fs = 1953.125 that is 1.2 Hz, invisible under an 82 Hz low
 * E. An aggressive a -- 0.9, say -- puts the corner at 31 Hz and tilts
 * exactly the bottom of the guitar's range. Bigger shift = gentler.
 */
#define TUNER_DC_SHIFT 8u

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

/*
 * Signal detection: a Schmitt trigger plus a hold, not one threshold.
 *
 * With 1/N scaling a sinusoid of Q15 amplitude A lands near A/4 after the
 * triangular window, so full scale is about 8000 and these are roughly
 * -42 dBFS to catch a note and -52 dBFS to keep one. Two thresholds
 * because a plucked string DECAYS: a single level either sits high enough
 * to reject room noise and then drops the reading a second after the
 * pluck, or sits low enough to hold the note and then reports phantom
 * pitches out of the noise between plucks. Catching hard and releasing
 * soft does both.
 *
 * The hold is the other half. Even the release threshold loses a decaying
 * string well before the player has finished turning the peg;
 * TUNER_HOLD_FRAMES keeps the last reading on screen after the signal is
 * gone, and the display marks it as held rather than pretending it is
 * live. It is counted in HOPS, so at TUNER_HOP = 256 (131 ms) 24 of them
 * is about 3 seconds -- change the hop and this moves with it.
 *
 * Both numbers are room-dependent and meant to be adjusted: raise
 * ATTACK if a noisy room reports notes with nothing played, lower it if a
 * mic across the room never triggers.
 */
#define TUNER_FLOOR_ATTACK 60u
#define TUNER_FLOOR_HOLD   20u
#define TUNER_HOLD_FRAMES  24u

/*
 * Console reporting rate, in hops.
 *
 * At one line per hop the console is 7.6 lines/s, which is unreadable and
 * was not the intent: the report exists to diagnose a board, and a board
 * is diagnosed from what CHANGES. So a line goes out when the note or the
 * valid/held state changes, and otherwise once every TUNER_REPORT_HOPS
 * (about a second) so the cents are still visible while a peg is turning.
 *
 * Not removed, and worth saying why: this print is what identified both
 * faults of the first board session -- an overrun that turned out to be a
 * false positive, and a threshold too high for a mic at arm's length.
 * Set to 1 for the old line-per-hop behaviour.
 */
#define TUNER_REPORT_HOPS 8u

/*
 * Needle animation.
 *
 * The estimate only changes once per hop, but two consecutive estimates
 * share 75% of their window, so the needle's TARGET moves in small steps
 * and a display that jumps straight to each one reads as stuttering. The
 * needle is therefore redrawn TUNER_ANIM_STEPS times per hop, each time a
 * fraction of the way toward the current target -- the same trick every
 * commercial tuner uses, and it changes nothing about the measurement.
 *
 * At 4 steps a redraw lands every 33 ms. Cost is bounded by the page hash:
 * only rows 32..47 change while the needle moves, that is 2 pages = 256
 * bytes = ~6 ms of I2C, and when the needle has converged nothing is sent
 * at all.
 */
#define TUNER_ANIM_STEPS 4u
_Static_assert(TUNER_HOP % TUNER_ANIM_STEPS == 0u,
               "TUNER_HOP must divide evenly into TUNER_ANIM_STEPS");

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

/*
 * Per-frame diagnostics from the acquisition itself (board build only).
 *
 * tuner_raw_min/max are the mic's own numbers before the CIC, the window
 * and the transform, which is the one measurement that separates "the bus
 * is dead" from "the signal is too quiet": a dead SD line pins both at 0,
 * a live but quiet mic gives a small non-zero swing, and a clipping one
 * hits +/-32767.
 *
 * tuner_frame_ovr is the overrun flag AS OF THE END OF THE FRAME, which is
 * not the same thing as the flag at any other moment -- see the loop in
 * main().
 */
#if !TUNER_SIM
static int tuner_raw_min;
static int tuner_raw_max;
static int tuner_frame_ovr;
/* Running total, printed with every report line. An overrun costs a whole
 * window -- TUNER_N clean samples have to replace the gap before anything
 * is analysed again, i.e. 4 hops = 524 ms of frozen needle -- so a counter
 * that climbs is what separates "the estimator is jittery" from "something
 * in the loop is starving the input". */
static unsigned int tuner_ovr_count;
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

/*
 * The sample ring: TUNER_N decimated, DC-blocked samples, written
 * continuously and read as a window at every hop. Power-of-two size, so
 * the wrap is a mask.
 */
static short        tuner_ring[TUNER_N];
static unsigned int tuner_ring_pos;    /* next write slot = oldest sample */
static unsigned int tuner_ring_count;  /* total samples ever written */

/* Decimator and DC blocker state, persistent across calls: the input is
 * drained in whatever size chunks happen to be available, so neither the
 * CIC phase nor the filter history can live in a local. */
static int          tuner_cic_i1, tuner_cic_i2, tuner_cic_d1, tuner_cic_d2;
static unsigned int tuner_cic_phase;
static int          tuner_dc_x1, tuner_dc_y;

/*
 * Samples still needed before the window can be trusted again.
 *
 * A gap in the stream (an overrun) is not a missing sample, it is a STEP
 * in the middle of the window, which smears the whole spectrum. Rather
 * than display a reading computed across one, the window is declared dirty
 * and nothing is analysed until TUNER_N clean samples have replaced it.
 * Set to TUNER_N at start-up too, since an empty ring is the same problem.
 */
static unsigned int tuner_dirty = TUNER_N;

/* The floor in force for the current frame: ATTACK while nothing is
 * being tracked, HOLD once a note has been caught (see the note above).
 * The self-check leaves it at ATTACK throughout. */
static unsigned int tuner_floor = TUNER_FLOOR_ATTACK;

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
/*
 * Source shape:
 *   TUNER_SRC_RICH  fundamental plus 2nd and 3rd, the THIRD loudest -- the
 *                   trap a largest-bin picker falls into.
 *   TUNER_SRC_PURE  one tone, for the anti-aliasing measurement, where a
 *                   harmonic would contaminate the magnitude ratio.
 *   TUNER_SRC_WEAK  fundamental a fifth of the 2nd harmonic. This is not a
 *                   hypothetical: playing an 82 Hz E2 through a small
 *                   speaker loses ~18 dB of low end, which is exactly this
 *                   ratio, and it made the board report E3 (2026-09-16).
 */
#define TUNER_SRC_RICH 0
#define TUNER_SRC_PURE 1
#define TUNER_SRC_WEAK 2

static unsigned int tuner_sim_src;

static void tuner_sim_set(unsigned int f_cHz, unsigned int decim, int src)
{
    tuner_sim_phase = 0u;
    tuner_sim_src   = (unsigned int)src;

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
    if (tuner_sim_src == TUNER_SRC_PURE) {
        s = tuner_sin(tuner_sim_phase) >> 1;
    } else if (tuner_sim_src == TUNER_SRC_WEAK) {
        /* Fundamental at 1/5 of the 2nd harmonic, nothing else. */
        s = (tuner_sin(tuner_sim_phase) >> 4) + (tuner_sin(tuner_sim_phase * 2u) >> 1);
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
 * Unused on the board: tuner_pump() reads the FIFO directly so that it can
 * check for an overrun on the same pass, and tuner_drain() takes the
 * non-blocking path. Kept as the sim build's hook only.
 */
#endif

/*
 * One input sample -> decimator -> DC blocker -> ring.
 *
 * DECIMATION. The mic runs at TUNER_FS_IN_HZ and a second-order CIC drops
 * it by TUNER_DECIM to the analysis rate. A CIC of order N and rate R is N
 * integrators at the fast rate, a downsample, and N combs at the slow rate
 * -- no multipliers, no coefficients, and its nulls land exactly on the
 * multiples of the output rate, which is precisely where everything that
 * aliases to DC comes from. Order 2 buys about 30 dB at the worst fold
 * against 15 dB for a plain boxcar, for one more adder.
 *
 * The integrators are allowed to wrap. That is not sloppiness: a CIC is
 * correct under two's-complement wrapping as long as the accumulator is
 * wide enough for the filter's own gain (R^order = 64 here), because the
 * comb stage subtracts the wrap back out.
 *
 * All the state is static because this is called from two places with
 * different chunk sizes -- the blocking pump and the opportunistic drain
 * that runs between display chunks -- and a CIC phase in a local would
 * restart the decimation pattern at every call.
 */
static void tuner_push_input(int x)
{
    int c1, c2, s, y;

#if !TUNER_SIM
    /* Raw swing since the last hop, for the no-signal report: it is the
     * mic's own number, before the decimator and the window, and it is
     * what separates a dead bus from a quiet room. */
    if (x < tuner_raw_min) {
        tuner_raw_min = x;
    }
    if (x > tuner_raw_max) {
        tuner_raw_max = x;
    }
#endif

    tuner_cic_i1 += x;
    tuner_cic_i2 += tuner_cic_i1;

    if (++tuner_cic_phase < tuner_decim)
        return;
    tuner_cic_phase = 0u;

    c1           = tuner_cic_i2 - tuner_cic_d1;
    tuner_cic_d1 = tuner_cic_i2;
    c2           = c1 - tuner_cic_d2;
    tuner_cic_d2 = c1;

    /* Divide out the CIC's gain, R^order, back to the input's own scale.
     * An arithmetic shift, so it rounds toward -inf; the bias is half an
     * LSB and the DC blocker below eats it whole. At R = 1 the shift is 0
     * and the whole structure collapses to an identity, which is what lets
     * the self-check bypass it. */
    s = c2 >> tuner_cic_shift;

    /* DC blocker (see TUNER_DC_SHIFT): y += (x - x1) - y/2^k. */
    y            = tuner_dc_y + (s - tuner_dc_x1) - (tuner_dc_y >> TUNER_DC_SHIFT);
    tuner_dc_x1  = s;
    tuner_dc_y   = y;

    if (y > 32767) {
        y = 32767;
    } else if (y < -32768) {
        y = -32768;
    }

    tuner_ring[tuner_ring_pos] = (short)y;
    tuner_ring_pos             = (tuner_ring_pos + 1u) & (TUNER_N - 1u);
    tuner_ring_count++;
    if (tuner_dirty)
        tuner_dirty--;
}

#if !TUNER_SIM
/*
 * Opportunistic drain: take whatever the I2S FIFO holds right now and put
 * it through the chain, without blocking.
 *
 * This is the yield hook the display push calls between I2C chunks, and it
 * is what makes a continuous ring possible at all: the FIFO is 16 words,
 * about 1 ms at 15625 Hz, against a ~25 ms framebuffer push. Without
 * someone draining it mid-transfer the ring would take a 390-sample hole
 * every time the screen is redrawn.
 *
 * An overrun here means the gap already happened, so the window is marked
 * dirty rather than analysed.
 */
static void tuner_drain(void)
{
    int s;

    while (i2s_available()) {
        (void)i2s_read(&s);
        tuner_push_input(s >> TUNER_IN_SHIFT);
    }
    if (i2s_overrun()) {
        i2s_overrun_clear();
        tuner_dirty     = TUNER_N;
        tuner_frame_ovr = 1;
        tuner_ovr_count++;
    }
}
#endif

/* Block until `n` more samples have reached the ring. */
static void tuner_pump(unsigned int n)
{
    unsigned int target = tuner_ring_count + n;

    while (tuner_ring_count < target) {
#if TUNER_SIM
        tuner_push_input(tuner_mic_sample());
#else
        int s;

        /* i2s_read_wait blocks, so this paces itself off the mic. The
         * overrun check is per pass, not per sample: at one MMIO read per
         * 3200 core cycles the loop cannot fall behind on its own, so a
         * flag here means something else stole the CPU. */
        (void)i2s_read_wait(&s);
        tuner_push_input(s >> TUNER_IN_SHIFT);
        if (i2s_overrun()) {
            i2s_overrun_clear();
            tuner_dirty     = TUNER_N;
            tuner_frame_ovr = 1;
            tuner_ovr_count++;
        }
#endif
    }
}

/*
 * Copy the ring into the coprocessor's buffer, oldest sample first, with
 * the analysis window applied.
 *
 * Window is triangular. It costs ~26 dB of sidelobe against a rectangle
 * and needs no table -- Hann would need 1024 Q15 entries or a cosine this
 * build has no way to compute.
 *
 * All TUNER_N samples go out every hop; see the note at TUNER_HOP for why
 * the untouched 768 cannot simply be left in place.
 */
static void tuner_fill_fft(void)
{
    unsigned int i;
    unsigned int r = tuner_ring_pos;
    int          w;

    /*
     * The window is walked incrementally instead of being computed per
     * sample: a triangle over TUNER_N points steps by 2*32768/TUNER_N = 64
     * in Q15, so the ternary, the multiply and the divide that used to sit
     * in the loop body are all one addition. Two loops, one per slope,
     * which also removes the per-sample branch.
     *
     * The ring index is carried and masked rather than recomputed from i.
     */
    w = 0;
    for (i = 0; i < TUNER_N / 2u; i++) {
        fft_write_real(i, (tuner_ring[r] * w) >> 15);
        r = (r + 1u) & (TUNER_N - 1u);
        w += 64;
    }
    w = 32768 - 64;
    for (; i < TUNER_N; i++) {
        fft_write_real(i, (tuner_ring[r] * (w > 32767 ? 32767 : w)) >> 15);
        r = (r + 1u) & (TUNER_N - 1u);
        w -= 64;
    }
}

/* ---------------------------------------------------------------- */
/* Analysis                                                          */
/* ---------------------------------------------------------------- */

/*
 * Magnitudes of the bins the estimator actually looks at.
 *
 * Two things this does NOT do, and both were costing real cycles:
 *
 *   - it does not read all 512 bins. The search runs over KMIN..KMAX (70
 *     to 420 Hz, 186 bins) and the parabolic interpolation needs one
 *     neighbour either side, so everything outside 35..222 is read, packed
 *     into a magnitude and then never looked at. That was 63% of the loop.
 *   - it does not call fft_mag(), which reads the SAME MMIO word twice
 *     (once through fft_read_re and once through fft_read_im). One load
 *     carries both halves; splitting it doubled the bus traffic of the
 *     whole pass.
 *
 * Together: 1024 MMIO loads down to 188. The bins outside the range keep
 * whatever was in the array; nothing reads them.
 */
#define TUNER_KLO (TUNER_KMIN - 1u)
#define TUNER_KHI (TUNER_KMAX + 1u)

static void tuner_read_spectrum(void)
{
    unsigned int k;

    for (k = TUNER_KLO; k <= TUNER_KHI; k++) {
        int re, im;
        unsigned int a, b, hi, lo;

        fft_read(k, &re, &im); /* one word: {imag, real} */
        a  = (unsigned int)(re < 0 ? -re : re);
        b  = (unsigned int)(im < 0 ? -im : im);
        hi = a > b ? a : b;
        lo = a > b ? b : a;

        /* Same sqrt-free approximation as fft_mag(): max + 3*min/8. */
        tuner_mag[k] = (unsigned short)(hi + (3u * lo) / 8u);
    }
}

/*
 * Index of the fundamental, or -1 if the band is quiet.
 *
 * See the header: the loudest bin is often a harmonic, so the loudest is
 * only used to set a threshold, and the answer is the LOWEST local
 * maximum that reaches a quarter of it.
 *
 * TWO gates decide "is there a signal", and they fail differently on
 * purpose. The absolute floor (tuner_floor) rejects a quiet room. The
 * SNR gate rejects a LOUD one: the peak must also stand
 * TUNER_SNR_RATIO times above the mean of the whole search band, which a
 * real note does easily -- one bin out of ~190 barely moves that mean --
 * and broadband noise never does, however loud it gets. The second gate
 * is what makes the low absolute floor safe: without it, turning the
 * sensitivity up far enough to catch a decaying string also turns it up
 * far enough to report pitches out of room rumble.
 */
#define TUNER_SNR_RATIO 4u

/*
 * Sub-harmonic rescue, for when the fundamental is weak rather than absent.
 *
 * The scan above accepts the lowest partial that reaches 1/TUNER_PEAK_
 * FRACTION of the strongest one. A fundamental quieter than that is
 * skipped and the answer comes out an octave high -- which is exactly what
 * happens when the signal arrives through something that cannot reproduce
 * the low end. A small speaker playing an 82 Hz E2 loses about 18 dB down
 * there, which puts the fundamental at a fifth of the 2nd harmonic: right
 * on the threshold, so the reading flips between E2 and E3 frame by frame.
 * (Measured on the board, 2026-09-16.)
 *
 * So after choosing a bin, look once at half of it. Taking it requires
 * a local maximum, 1/TUNER_SUB_FRACTION of the strongest bin -- a much
 * lower bar -- AND the same signal-to-noise margin the detection itself
 * uses, which is what stops a noise bump an octave down from being
 * promoted to the answer. One level only: two would be inviting the
 * octave-down error this is meant to avoid.
 */
#define TUNER_SUB_FRACTION 10u

static int tuner_find_fundamental(unsigned int *peak_out)
{
    unsigned int k, best = TUNER_KMIN, bestm = 0u, thr;
    unsigned int sum = 0u, mean;

    for (k = TUNER_KMIN; k <= TUNER_KMAX; k++) {
        sum += tuner_mag[k];
        if (tuner_mag[k] > bestm) {
            bestm = tuner_mag[k];
            best  = k;
        }
    }

    mean = sum / (TUNER_KMAX - TUNER_KMIN + 1u);

    *peak_out = bestm;
    if (bestm < tuner_floor || bestm < TUNER_SNR_RATIO * mean) {
        return -1;
    }

    thr  = bestm / TUNER_PEAK_FRACTION;
    best = TUNER_KMAX + 1u; /* "nothing found by the scan" */
    for (k = TUNER_KMIN + 1u; k < TUNER_KMAX; k++) {
        if (tuner_mag[k] >= thr && tuner_mag[k] > tuner_mag[k - 1u] &&
            tuner_mag[k] >= tuner_mag[k + 1u]) {
            best = k;
            break;
        }
    }
    if (best > TUNER_KMAX) {
        /* No local maximum cleared the bar: fall back to the strongest
         * bin, which is what the search started from. */
        for (k = TUNER_KMIN, bestm = 0u; k <= TUNER_KMAX; k++) {
            if (tuner_mag[k] > bestm) {
                bestm = tuner_mag[k];
                best  = k;
            }
        }
        *peak_out = bestm;
        return (int)best;
    }

    /* Sub-harmonic rescue (see TUNER_SUB_FRACTION). The harmonic's bin is
     * not exactly twice the fundamental's once rounding is involved, so
     * look at the neighbours of best/2 as well. */
    if (best / 2u > TUNER_KMIN) {
        unsigned int h   = best / 2u;
        unsigned int sub = bestm / TUNER_SUB_FRACTION;

        for (k = h - 1u; k <= h + 1u; k++) {
            if (k <= TUNER_KMIN || k >= TUNER_KMAX) {
                continue;
            }
            if (tuner_mag[k] >= sub && tuner_mag[k] > TUNER_SNR_RATIO * mean &&
                tuner_mag[k] > tuner_mag[k - 1u] && tuner_mag[k] >= tuner_mag[k + 1u]) {
                *peak_out = tuner_mag[k];
                return (int)k;
            }
        }
    }

    *peak_out = tuner_mag[best];
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

/*
 * Gauge geometry. The needle spans +/-TUNER_GAUGE_SPAN cents across
 * +/-TUNER_GAUGE_HALF pixels either side of centre.
 *
 * TOP and BOT are ALIGNED TO PAGE BOUNDARIES on purpose: rows 32..47 are
 * exactly pages 4 and 5, so a needle that moves dirties two pages and the
 * animation below pushes 256 bytes instead of 512. The SSD1306 addresses
 * memory in 8-row pages whatever the drawing code thinks, so a gauge that
 * straddles a page edge costs an extra page per redraw for nothing. Rows
 * 30..48 -- the obvious choice, and what this was -- touched four.
 */
#define TUNER_GAUGE_CX    64
#define TUNER_GAUGE_HALF  56
#define TUNER_GAUGE_SPAN  50
#define TUNER_GAUGE_AXIS  39
#define TUNER_GAUGE_TOP   32
#define TUNER_GAUGE_BOT   47

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

/*
 * One screen. `held` means the reading is the last good one and the
 * signal has since fallen below the release threshold -- still the best
 * estimate there is, so it stays on the gauge, but marked.
 *
 * THE SCALE IS DRAWN IN EVERY STATE, including no-signal. A tuner whose
 * layout jumps between two different screens is hard to read while you
 * are turning a peg; what changes is the needle and the bottom line, not
 * the furniture. The needle is the one thing NOT drawn when there is no
 * reading -- parking it at centre would be indistinguishable from a
 * perfectly tuned string.
 */
static void tuner_render(const tuner_result_t *r, int held, int needle_cents)
{
    /* Drawing a full framebuffer is ~0.6 ms of memset and blitting with no
     * I2C in it, so nothing else would service the mic during it. The FIFO
     * holds 1.02 ms: the margin is real but thin, and one drain here costs
     * nothing. */
    tuner_drain();

    ssd1306_clear();

    if (!r->valid) {
        tuner_banner(2u, "PLUCK A STRING", 0);
        tuner_draw_scale();
        tuner_banner(54u, "NO SIGNAL", 0);
        ssd1306_update();
        return;
    }

    /* Note name, double height, left. The table's names are padded to 3
     * characters so a sharp appearing or disappearing does not shift the
     * frequency field next to it. */
    ssd1306_text_scaled(2u, 0u, tuner_notes[r->note].name, 2u, 1);

    /* Held readings are marked, so a stale note is never mistaken for a
     * live one. Top right corner, out of the way of the numbers. */
    if (held) {
        ssd1306_text(SSD1306_WIDTH - 6u, 0u, "H", 1);
    }

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
    /* The NUMBERS are the reading; only the NEEDLE is animated. Animating
     * the printed cents too would be inventing measurements that were
     * never taken. */
    tuner_draw_needle(needle_cents);

    if (r->cents > TUNER_IN_TUNE_CENTS) {
        tuner_banner(54u, "SHARP", 0);
    } else if (r->cents < -TUNER_IN_TUNE_CENTS) {
        tuner_banner(54u, "FLAT", 0);
    } else {
        tuner_banner(54u, "IN TUNE", 1);
    }
}

/*
 * Push what actually changed, one page at a time, yielding to the mic.
 *
 * Two mechanisms, and they solve different halves of the same problem.
 * The PAGE HASH stops unchanged rows from being sent at all -- the scale,
 * the tick marks and a note name that has not moved are pushed once and
 * then never again, so a typical hop sends one or two pages instead of
 * eight. The CHUNKING inside ssd1306_update_page() bounds how long any
 * single transfer holds the CPU, which is what keeps the sample ring
 * contiguous: 24 bytes at 400 kHz is ~600 us against a FIFO that fills in
 * 1.02 ms.
 *
 * A 16-bit hash can in principle collide and leave a page stale. It is
 * worth the 16 bytes of state rather than a 1 KiB shadow framebuffer in a
 * 8 KiB D-mem, and any stale page is corrected by the next change to it.
 */
/*
 * One animation step: move a third of the way to the target, but never
 * less than one pixel-worth of cents, so it always converges instead of
 * creeping asymptotically forever.
 */
static int tuner_anim_step(int cur, int target)
{
    int d = target - cur;
    int s;

    if (d == 0) {
        return cur;
    }
    s = d / 3;
    if (s == 0) {
        s = (d > 0) ? 1 : -1;
    }
    return cur + s;
}

#define TUNER_PAGES (SSD1306_HEIGHT / 8u)

static unsigned short tuner_page_hash[TUNER_PAGES];
static int            tuner_page_primed;

static void tuner_flush_display(void)
{
    unsigned int p, i;

    /* Same reason as in tuner_render(): the hash pass walks 1 KiB before
     * the first I2C chunk gets a chance to yield. */
    tuner_drain();

    for (p = 0; p < TUNER_PAGES; p++) {
        const unsigned char *row = &ssd1306_fb[p * SSD1306_WIDTH];
        unsigned short       h   = 0u;

        for (i = 0; i < SSD1306_WIDTH; i++) {
            h = (unsigned short)(h * 31u + row[i]);
        }
        if (!tuner_page_primed || h != tuner_page_hash[p]) {
            ssd1306_update_page(p);
            tuner_page_hash[p] = h;
        }
    }
    tuner_page_primed = 1;
}
#endif /* !TUNER_SIM */

/* ---------------------------------------------------------------- */
/* UART reporting (both builds -- the board one is a debug console)   */
/* ---------------------------------------------------------------- */

/*
 * Console output, draining the mic FIFO as it goes.
 *
 * A report line is ~40 characters, which at 115200 baud is 3.5 ms -- three
 * times the 16-word FIFO. On the board that is as damaging to a continuous
 * ring as the display push is, so the same yield applies: one drain per
 * character costs nothing and bounds the gap at one character time.
 */
#if !TUNER_SIM
static void tuner_putc(char c)
{
    tuner_drain();
    uart_putc(c);
}

static void tuner_puts(const char *str)
{
    while (*str) {
        tuner_putc(*str++);
    }
}
#else
#define tuner_putc uart_putc
#define tuner_puts uart_puts
#endif

static void tuner_put_uint(unsigned int v)
{
    char buf[12];
    unsigned int n = 0;

    do {
        buf[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v && n < sizeof(buf));
    while (n--) {
        tuner_putc(buf[n]);
    }
}

static void tuner_put_int(int v)
{
    if (v < 0) {
        tuner_putc('-');
        v = -v;
    }
    tuner_put_uint((unsigned int)v);
}

/* centi-Hz as "82.41" */
static void tuner_put_chz(unsigned int v)
{
    tuner_put_uint(v / 100u);
    tuner_putc('.');
    tuner_putc((char)('0' + (v / 10u) % 10u));
    tuner_putc((char)('0' + v % 10u));
}

static void tuner_report(const tuner_result_t *r, int held)
{
    if (held) {
        tuner_puts("(held) ");
    }
    if (!r->valid) {
        tuner_puts("-- no signal (peak ");
        tuner_put_uint(r->peak);
#if !TUNER_SIM
        /* The raw mic swing is what says WHICH kind of no-signal this is
         * (see tuner_raw_min/max above). */
        tuner_puts(", raw ");
        tuner_put_int(tuner_raw_min);
        tuner_puts("..");
        tuner_put_int(tuner_raw_max);
        tuner_puts(", floor ");
        tuner_put_uint(tuner_floor);
#endif
        tuner_puts(")\r\n");
        return;
    }
    tuner_puts(tuner_notes[r->note].name);
    tuner_puts("  ");
    tuner_put_chz(r->f_cHz);
    tuner_puts(" Hz  ");
    if (r->cents >= 0) {
        tuner_putc('+');
    }
    tuner_put_int(r->cents);
    tuner_puts(" cents  peak ");
    tuner_put_uint(r->peak);
#if !TUNER_SIM
    if (tuner_ovr_count) {
        tuner_puts("  OVR ");
        tuner_put_uint(tuner_ovr_count);
    }
#endif
    tuner_puts("\r\n");
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
static int tuner_run_frame(unsigned int f_cHz, int src, tuner_result_t *r)
{
    tuner_sim_set(f_cHz, tuner_decim, src);
    /* A whole window per case: the oscillator changes frequency between
     * cases, so overlapping them would mix two tones in one window. The
     * board path hops by TUNER_HOP instead. */
    tuner_pump(TUNER_N);
    tuner_fill_fft();
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

    if (!tuner_run_frame(TUNER_ALIAS_IN_CHZ, TUNER_SRC_PURE, &in)) {
        return 1u;
    }
    uart_puts("in-band   ");
    tuner_report(&in, 0);

    if (!tuner_run_frame(TUNER_ALIAS_OUT_CHZ, TUNER_SRC_PURE, &out)) {
        return 1u;
    }
    /* The note printed for this one is meaningless -- with the floor set
     * low enough to catch a decaying string, a 1/49 residue can still
     * clear it. The CRITERION is the peak ratio below, not whether the
     * alias was called a pitch. */
    uart_puts("aliasing  ");
    tuner_report(&out, 0);

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

    /*
     * CPU budget per hop, measured rather than assumed.
     *
     * The overlapped loop has to rewrite the whole window into the
     * coprocessor and read the spectrum back once every TUNER_HOP samples.
     * Both are MMIO loops over a 4 KiB window, so they are the biggest
     * fixed cost in the loop and the one that would quietly cap the hop
     * rate if it grew. One decimated sample is
     * TUNER_CORE_HZ*TUNER_DECIM/TUNER_FS_IN_HZ = 25600 core cycles, so a
     * 256-sample hop is 6.55 M cycles and this should not be close.
     */
    {
        unsigned int t0, fill_cyc, read_cyc, budget;

        t0 = sys_cycle();
        tuner_fill_fft();
        fill_cyc = sys_cycle() - t0;

        t0 = sys_cycle();
        tuner_read_spectrum();
        read_cyc = sys_cycle() - t0;

        budget = TUNER_HOP * (TUNER_CORE_HZ * TUNER_DECIM / TUNER_FS_IN_HZ);

        uart_puts("hop cost ");
        tuner_put_uint(fill_cyc + read_cyc);
        uart_puts(" cycles (fill ");
        tuner_put_uint(fill_cyc);
        uart_puts(" + read ");
        tuner_put_uint(read_cyc);
        uart_puts("), budget ");
        tuner_put_uint(budget);
        uart_puts("\r\n");

        /* A quarter of the budget is already a design smell, not a limit
         * anyone should be near. */
        if (fill_cyc + read_cyc > budget / 4u) {
            uart_puts("   ^ MMIO cost is a quarter of the hop\r\n");
            fails++;
        }
    }

    /*
     * Where the transfer cycles actually go: the peripheral bus, or the
     * loop around it?
     *
     * The same loop shape is run against the coprocessor's DATA page and
     * against D-mem, so the DIFFERENCE is the cost of the peri path --
     * LSU launch, bridge, crossbar, slave, response -- and everything else
     * cancels. Without this split, "moving the data costs more than
     * transforming it" is a true statement that does not say what to fix.
     *
     * It clobbers the ring, which is why it runs before the pitch cases
     * refill it.
     */
    {
        volatile unsigned int *dmem = (volatile unsigned int *)tuner_ring;
        unsigned int           t0, st_p, ld_p, st_d, ld_d, i;
        unsigned int           sink = 0u;

        t0 = sys_cycle();
        for (i = 0; i < 256u; i++) {
            FFT_DATA[i] = i;
        }
        st_p = sys_cycle() - t0;

        t0 = sys_cycle();
        for (i = 0; i < 256u; i++) {
            sink += FFT_DATA[i];
        }
        ld_p = sys_cycle() - t0;

        t0 = sys_cycle();
        for (i = 0; i < 256u; i++) {
            dmem[i] = i;
        }
        st_d = sys_cycle() - t0;

        t0 = sys_cycle();
        for (i = 0; i < 256u; i++) {
            sink += dmem[i];
        }
        ld_d = sys_cycle() - t0;

        uart_puts("256 accesses: peri store ");
        tuner_put_uint(st_p);
        uart_puts(", peri load ");
        tuner_put_uint(ld_p);
        uart_puts(", dmem store ");
        tuner_put_uint(st_d);
        uart_puts(", dmem load ");
        tuner_put_uint(ld_d);
        uart_puts(" cycles\r\n");
        (void)sink;
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

        if (!tuner_run_frame(tuner_cases[i].f_cHz, TUNER_SRC_RICH, &r)) {
            fails++;
            break;
        }
        tuner_report(&r, 0);

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
    if (tuner_run_frame(0u, TUNER_SRC_RICH, &r)) {
        tuner_report(&r, 0);
        if (r.valid) {
            uart_puts("   ^ silence reported as a pitch\r\n");
            fails++;
        }
    } else {
        fails++;
    }

    /*
     * Weak fundamental: the board case. With the fundamental at a fifth of
     * the 2nd harmonic, the lowest-strong-partial scan skips it (its bar is
     * a quarter of the strongest bin) and the answer comes out an octave
     * high unless the sub-harmonic rescue catches it. Two frequencies, one
     * at each end of the range, because the rescue looks at best/2 and that
     * has to stay inside the search band.
     */
    {
        static const unsigned int weak_cHz[] = {8241u, 19600u}; /* E2, G3 */
        static const int          weak_note[] = {2, 17};
        unsigned int              w;

        for (w = 0; w < 2u; w++) {
            if (!tuner_run_frame(weak_cHz[w], TUNER_SRC_WEAK, &r)) {
                fails++;
                break;
            }
            uart_puts("weak fund ");
            tuner_report(&r, 0);
            if (!r.valid || r.note != weak_note[w]) {
                uart_puts("   ^ reported the octave, not the fundamental\r\n");
                fails++;
            }
        }
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

/*
 * Bring-up probe, run once before the tuning loop.
 *
 * It answers the only question worth asking first on a silent link -- is
 * ANYTHING arriving -- and it distinguishes the three failure shapes that
 * all look identical from the tuner's output:
 *
 *   nothing queued at all   the peripheral never captured a word: no BCLK
 *                           coming back, or the mic is in the other slot
 *                           (L/R tied high instead of to GND).
 *   words, all zero         the bus is clocking and framing but SD is not
 *                           connected, or the mic is unpowered -- a
 *                           pulled-down pad reads a clean 0.
 *   words with a swing      the front end works; anything else is a level
 *                           question, and the numbers here say by how much.
 */
static void tuner_mic_probe(void)
{
    int          lo = 32767, hi = -32768;
    unsigned int got = 0u, spin = 0u, nonzero = 0u;
    int          s;

    uart_puts("mic probe: ");
    i2s_flush();

    /* Bounded: roughly 64 frames' worth of core cycles, so a dead bus
     * reports instead of hanging. */
    while (got < 64u && spin < 64u * 4u * (TUNER_CORE_HZ / TUNER_FS_IN_HZ)) {
        if (i2s_available()) {
            (void)i2s_read(&s);
            s >>= TUNER_IN_SHIFT;
            if (s < lo) {
                lo = s;
            }
            if (s > hi) {
                hi = s;
            }
            if (s != 0) {
                nonzero++;
            }
            got++;
        }
        spin++;
    }

    tuner_put_uint(got);
    uart_puts(" words, ");
    tuner_put_uint(nonzero);
    uart_puts(" non-zero, range ");
    if (got) {
        tuner_put_int(lo);
        uart_puts("..");
        tuner_put_int(hi);
    } else {
        uart_puts("n/a");
    }
    uart_puts("\r\n");

    if (got == 0u) {
        uart_puts("   no words at all -- check BCLK/WS reach the mic "
                  "(pins 73/74) and that L/R is tied to GND\r\n");
    } else if (nonzero == 0u) {
        uart_puts("   words arrive but all zero -- check SD (pin 75) and "
                  "the mic's 3V3\r\n");
    }
    if (i2s_overrun()) {
        /* Expected here: this probe reads far slower than the stream. */
        i2s_overrun_clear();
    }
}

int main(void)
{
    tuner_result_t r;
    tuner_result_t last;             /* last reading with a real signal */
    unsigned int   hold = 0u;        /* hops left to keep showing it */
    unsigned int   since_report = 0u;
    int            last_state = -1;  /* valid/held combination last printed */
    int            last_note = -1;
    int            anim_cents = 0;   /* where the needle is drawn now */
    int            target_cents = 0; /* where the last reading says it goes */
    int            showing = 0;      /* a needle is on screen at all */

    last.valid = 0;

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
    tuner_mic_probe();

    /* Which build is on the board, in one line. "Is the display slower
     * than I think" is otherwise unanswerable from the outside: window and
     * hop are the two numbers that decide it, and they are not visible in
     * the picture -- consecutive readings share 75% of their window, so a
     * faster update looks a lot like no change at all. */
    uart_puts("window ");
    tuner_put_uint(TUNER_N);
    uart_puts(" samples (");
    tuner_put_uint((TUNER_N * 1000u) / (TUNER_FS_IN_HZ / TUNER_DECIM));
    uart_puts(" ms), hop ");
    tuner_put_uint(TUNER_HOP);
    uart_puts(" (");
    tuner_put_uint((TUNER_HOP * 1000u) / (TUNER_FS_IN_HZ / TUNER_DECIM));
    uart_puts(" ms per update)\r\n");

    i2c_begin_hz(TUNER_I2C_HZ);

    if (ssd1306_begin(SSD1306_ADDR) != I2C_OK) {
        uart_puts("SSD1306 did not ACK at 0x3C -- check wiring/address\r\n");
    }
    ssd1306_clear();
    tuner_banner(20u, "YARV TUNER", 0);
    tuner_banner(36u, "E A D G B E", 0);
    ssd1306_update();

    /* From here on every display push is paged, chunked and yields to the
     * mic: the ring must not take a 25 ms hole every time the screen
     * changes. The splash above is pushed before the stream starts, so it
     * can still go out in one transaction. */
    ssd1306_yield_fn = tuner_drain;

    fft_begin(TUNER_LOG2N);

    /*
     * Overlapped analysis. The window is TUNER_N long and slides by
     * TUNER_HOP, so a new reading lands every 131 ms while each one still
     * sees 524 ms of signal -- see the note at TUNER_HOP for what that
     * does and does not buy.
     *
     * Order inside the loop is fixed by the ping-pong: fft_start() hands
     * the engine the buffer just filled AND hands back the one holding the
     * previous result, so it must come between filling and reading. The
     * displayed reading is therefore one hop (131 ms) old.
     */
    /*
     * Start the count from a clean slate. Everything between i2s_begin()
     * and here -- the mic probe, the I2C init, the 25 ms splash push that
     * happens before the yield hook is armed -- overruns the FIFO
     * repeatedly and legitimately, and those events would otherwise land
     * in the counter as if they were the steady-state problem it exists to
     * measure.
     */
    i2s_flush();
    tuner_ovr_count = 0u;
    tuner_frame_ovr = 0;
    tuner_dirty     = TUNER_N;

    tuner_pump(TUNER_N); /* prime: one full window before the first launch */
    tuner_fill_fft();
    fft_start();

    for (;;) {
        unsigned int step;

        tuner_raw_min = 32767;
        tuner_raw_max = -32768;

        /*
         * One hop, in TUNER_ANIM_STEPS slices, with the needle stepping
         * toward its target between them. The samples keep arriving
         * through tuner_pump() exactly as before -- the animation rides on
         * the waiting time that already existed, it does not add any.
         */
        for (step = 0; step < TUNER_ANIM_STEPS; step++) {
            tuner_pump(TUNER_HOP / TUNER_ANIM_STEPS);

            if (showing && anim_cents != target_cents) {
                anim_cents = tuner_anim_step(anim_cents, target_cents);
                tuner_render(showing == 2 ? &last : &r, showing == 2, anim_cents);
                tuner_flush_display();
            }
        }

        if (!fft_wait()) {
            tuner_puts("FFT stalled\r\n");
            continue;
        }
        tuner_fill_fft();
        fft_start();

        /*
         * An overrun is now always a fault, which it was not when frames
         * were independent: a gap is a step in the MIDDLE of the sliding
         * window, so it smears the spectrum instead of merely separating
         * two clean frames. tuner_drain()/tuner_pump() mark the window
         * dirty when they see one, and nothing is analysed until a whole
         * window of clean samples has replaced it.
         */
        if (tuner_frame_ovr) {
            tuner_frame_ovr = 0;
            tuner_puts("!! I2S overrun -- window discarded\r\n");
        }
        if (tuner_dirty) {
            continue;
        }

        tuner_analyse(&r);

        /*
         * Catch hard, release soft, then hold. A live reading refills the
         * hold counter and lowers the floor, so a decaying string keeps
         * its note while the peg is being turned; once even the release
         * threshold is gone the last reading stays on the gauge for
         * TUNER_HOLD_FRAMES hops (about 3 seconds) before the display
         * admits there is nothing there.
         */
        if (r.valid) {
            /*
             * SNAP, DO NOT SWEEP, when the reference note changes.
             *
             * Cents are relative to the nearest note, so crossing a
             * semitone takes the reading from about +50 to about -50. That
             * is the same pitch, described against a new reference -- but
             * an animation reads it as a target on the far side of the
             * dial and walks the needle backwards across the whole scale.
             * On a slowly rising input the needle would sweep up, then
             * visibly slide back, once per semitone.
             *
             * The same applies to the first reading after silence: there
             * is nothing to sweep from.
             */
            if (!showing || r.note != last.note) {
                anim_cents = r.cents;
            }
            last         = r;
            hold         = TUNER_HOLD_FRAMES;
            tuner_floor  = TUNER_FLOOR_HOLD;
            target_cents = r.cents;
            showing      = 1;
            tuner_render(&r, 0, anim_cents);
        } else if (hold > 0u) {
            hold--;
            target_cents = last.cents;
            showing      = 2;
            tuner_render(&last, 1, anim_cents);
        } else {
            tuner_floor = TUNER_FLOOR_ATTACK;
            showing     = 0;
            tuner_render(&r, 0, anim_cents);
        }

        /* Report on change, then on a timer (see TUNER_REPORT_HOPS). The
         * display is the live instrument; the console is the log. */
        {
            const tuner_result_t *shown = r.valid ? &r : (hold > 0u ? &last : &r);
            int                   held  = (!r.valid && hold > 0u);
            int                   state = (shown->valid ? 1 : 0) | (held ? 2 : 0);

            if (state != last_state || shown->note != last_note ||
                ++since_report >= TUNER_REPORT_HOPS) {
                tuner_report(shown, held);
                since_report = 0u;
                last_state   = state;
                last_note    = shown->note;
            }
        }

        tuner_flush_display();
    }
}

#endif /* TUNER_SIM */

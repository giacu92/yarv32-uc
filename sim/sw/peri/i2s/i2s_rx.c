#include "i2s.h"
#include "plic.h"

/*
 * I2S receiver end-to-end oracle.
 *
 * Covers what the standalone sim/hw/i2s_tb cannot: that the peripheral is
 * reachable at I2S_BASE through the real LSU + bridge + crossbar path,
 * that its window decodes without colliding with the FFT's 8 KiB window
 * just below it, and that the RX interrupt reaches mip.MEIP through the
 * PLIC as source 6 with a wfi wake. The framing, word lengths, sign
 * extension and FIFO corner cases are i2s_tb's job.
 *
 * It also exercises sim/sw/common/i2s.h as the interface under test: every
 * register access below goes through the library, so a header that drifts
 * from the RTL fails here rather than in the first application to use it.
 *
 * STIMULUS: sim_top models BOTH clock roles, switched by CTRL.MASTER the
 * same way the pads switch on the board. In slave mode an
 * i2s_master_model drives the whole bus with a self-describing stream -- the left word of frame k is +k and the right
 * word is -(k+1), 16-bit, I2S framing, a 32-BCLK slot per channel at
 * BCLK = core/8. Self-describing is what makes the test independent of
 * WHEN the receiver was enabled: reading one left word gives k, and every
 * other value follows from it. It also puts a negative word in every
 * frame, so a broken sign extension cannot pass.
 *
 * In master mode the peripheral generates the clocks and an
 * i2s_mic_model -- an INMP441 shape: 24 bits, left slot only, the other
 * slot silent -- answers on them, which is the configuration the guitar
 * tuner runs on the board.
 *
 * On the board the same firmware needs a real I2S device on PIN80/81/82
 * (BCLK/LRCK/SD), and the value checks will fail against a live
 * microphone -- this is a simulation oracle, not board bring-up.
 *
 * Result marker: 0x600D (pass) / 0xBAD (fail) written to D-mem 0x3000.
 * The first failing step number and the interrupt debug fields stay in
 * .data so a board run can be read out of D-mem.
 */

/* start.S does not zero .bss and .bss is NOBITS, so an initialised
 * counter must live in .data or it is simply not in the image. */
#define IN_DATA __attribute__((section(".data")))

static volatile unsigned int IN_DATA fail_step = 0;  /* first failing step */
static volatile unsigned int IN_DATA irq_seen = 0;
static volatile unsigned int IN_DATA claim_id = 0;
static volatile unsigned int IN_DATA bad_cause = 0;

static volatile unsigned int *const result = (volatile unsigned int *)0x3000;

/* One frame is 2 slots x 32 BCLK x 8 core cycles = 512 cycles, so every
 * bounded wait here is in units of "frames, generously". */
#define SPIN_LIMIT 200000u

/* The model's right word for a left word of k. Computed in 16-bit two's
 * complement and then sign-extended, which is exactly what the hardware
 * hands back. */
static int expect_right(int left)
{
    return -(left + 1);
}

static int fail(unsigned int step)
{
    if (fail_step == 0)
        fail_step = step;
    return 0;
}

/* PLIC dispatcher handler for source 6. Servicing = draining the FIFO
 * below the watermark, which drops i2s_irq_o: the CLAIM read is
 * side-effect-free, so a handler that does not do this re-takes forever
 * (the livelock contract in plic.h). */
static void i2s_isr(unsigned int src)
{
    claim_id = src;
    i2s_flush();
    irq_seen = irq_seen + 1;
}

static void unexpected_isr(unsigned int mcause)
{
    bad_cause = mcause;
    sys_irq_global_disable();
}

static int run(void)
{
    int l, r, prev;
    unsigned int i;

    /* ---- 1. Reset values -------------------------------------------- */
    /* These prove the window decodes and did not land on a neighbour:
     * the FFT's window ends at 0x1000_BFFF, one page below this one. */
    if (I2S_CTRL != 0u)
        return fail(1);
    if (I2S_WATER != 1u)
        return fail(2);
    if (i2s_available() || i2s_level() != 0u)
        return fail(3);
    if (i2s_overrun() || I2S_IRQ != 0u)
        return fail(4);

    /* ---- 2. Disabled receiver captures nothing ---------------------- */
    /* The master is clocking the whole time (the model is free-running),
     * so an enable bit that did nothing would show up right here. */
    for (i = 0; i < 4000u; ++i) {
        if (i2s_available())
            return fail(5);
    }

    /* The live pin levels must actually move: a stopped master is the
     * first thing to suspect on a silent link, and this is how firmware
     * tells the difference. */
    {
        unsigned int seen = 0, spin = 0;
        unsigned int first = i2s_pin_levels();
        while (spin++ < SPIN_LIMIT) {
            if (i2s_pin_levels() != first) {
                seen = 1;
                break;
            }
        }
        if (!seen)
            return fail(6);
    }

    /* ---- 3. Stereo stream, values and channel tags ------------------ */
    i2s_begin(I2S_WLEN_16, I2S_CHAN_STEREO, I2S_FMT_I2S);
    i2s_flush();

    /* A left/right pair: the pairing plus both values, against a sequence
     * the test never had to be told the phase of. */
    i2s_read_frame(&l, &r);
    if (r != expect_right(l))
        return fail(7);
    if (r >= 0) /* the right word is negative by construction */
        return fail(8);

    /* 16 consecutive frames: left increments by exactly one per frame and
     * the pairing never slips. A dropped or duplicated word (a FIFO
     * pointer bug, a lost WS transition) breaks the run immediately. */
    prev = l;
    for (i = 0; i < 16u; ++i) {
        i2s_read_frame(&l, &r);
        if (l != prev + 1)
            return fail(9);
        if (r != expect_right(l))
            return fail(10);
        prev = l;
    }
    if (i2s_overrun()) /* reading in step, so nothing may have been dropped */
        return fail(11);

    /* ---- 4. Channel filter ------------------------------------------ */
    /* Left only: one word per frame, tagged left, still incrementing by
     * one -- the right words are dropped at the receiver, not queued. */
    i2s_begin(I2S_WLEN_16, I2S_CHAN_LEFT, I2S_FMT_I2S);
    i2s_flush();
    if (i2s_read_wait(&prev) != I2S_CH_LEFT)
        return fail(12);
    for (i = 0; i < 8u; ++i) {
        if (i2s_read_wait(&l) != I2S_CH_LEFT)
            return fail(13);
        if (l != prev + 1)
            return fail(14);
        prev = l;
    }

    /* Right only: every word negative, decrementing by one. */
    i2s_begin(I2S_WLEN_16, I2S_CHAN_RIGHT, I2S_FMT_I2S);
    i2s_flush();
    if (i2s_read_wait(&prev) != I2S_CH_RIGHT)
        return fail(15);
    for (i = 0; i < 8u; ++i) {
        if (i2s_read_wait(&r) != I2S_CH_RIGHT)
            return fail(16);
        if (r >= 0 || r != prev - 1)
            return fail(17);
        prev = r;
    }

    /* ---- 5. Watermark and overrun ----------------------------------- */
    i2s_begin(I2S_WLEN_16, I2S_CHAN_STEREO, I2S_FMT_I2S);
    i2s_set_watermark(4);
    i2s_flush();

    /* Below the watermark the IRQ status bit stays low. Two words arrive
     * per frame, so wait for exactly one frame's worth. */
    {
        unsigned int spin = 0;
        while (i2s_level() < 2u && ++spin < SPIN_LIMIT) {
        }
        if (i2s_level() < 2u)
            return fail(18);
        if (I2S_IRQ & I2S_IRQ_RX)
            return fail(19);
    }
    {
        unsigned int spin = 0;
        while (i2s_level() < 4u && ++spin < SPIN_LIMIT) {
        }
        if (!(I2S_IRQ & I2S_IRQ_RX))
            return fail(20);
    }

    /* Stop reading entirely: the FIFO fills to 16 and the receiver starts
     * dropping words, which is the only thing that sets the sticky
     * overrun. The wait is bounded by frames, not by the flag, so a
     * missing overrun fails instead of hanging. */
    {
        unsigned int spin = 0;
        while (!i2s_overrun() && ++spin < SPIN_LIMIT) {
        }
        if (!i2s_overrun())
            return fail(21);
        if (i2s_level() != 16u || !(I2S_STATUS & I2S_ST_RX_FULL))
            return fail(22);
    }

    /* The flag is sticky across draining, and W1C clears it. */
    i2s_flush();
    if (i2s_overrun())
        return fail(23);

    /* ---- 6. RX interrupt through the PLIC --------------------------- */
    sys_isr_install();
    sys_isr_set_unhandled(unexpected_isr);
    plic_attach(PLIC_SRC_I2S, i2s_isr);
    sys_irq_source_enable(SYS_MIE_MEIE);
    sys_irq_global_enable();

    i2s_set_watermark(8);
    i2s_flush();
    i2s_irq_enable(1, 0);

    {
        unsigned int spin = 0;
        while (irq_seen == 0u) {
            sys_wfi();
            if (++spin >= SPIN_LIMIT)
                return fail(24);
        }
    }
    if (claim_id != PLIC_SRC_I2S)
        return fail(25);
    if (bad_cause != 0u)
        return fail(26);

    /* The handler drained the FIFO, so the line is low again -- and with
     * IE_RX still set, the next batch of samples must raise it once more
     * with no clear in between. That is the level semantics the RX bit
     * exists to provide. */
    {
        unsigned int spin = 0;
        while (irq_seen < 2u) {
            sys_wfi();
            if (++spin >= SPIN_LIMIT)
                return fail(27);
        }
    }

    i2s_irq_enable(0, 0);
    sys_irq_global_disable();
    i2s_end();

    /* ---- 7. Master mode ---------------------------------------------- */
    /* The mode the board runs: the peripheral clocks the bus and an
     * INMP441-shaped slave answers. Nothing external is driving anything,
     * so a broken generator shows up as silence rather than as wrong
     * data. */
    if (i2s_master_hz(15625u, 32u) != 15625u)
        return fail(28); /* 15625 Hz must be exact from a 50 MHz core */
    if (I2S_CLKDIV != ((32u << 16) | 24u))
        return fail(29); /* BCLK = 1 MHz, 64-BCLK frame */

    i2s_begin(I2S_WLEN_24, I2S_CHAN_LEFT, I2S_FMT_I2S);
    if (!(I2S_CTRL & I2S_CTRL_MASTER))
        return fail(30); /* i2s_begin must preserve the MASTER bit */
    i2s_flush();

    /* The mic's word for frame m is m << 4, so consecutive frames differ
     * by exactly 16 and a dropped or repeated frame is visible. Only the
     * left slot carries data; the right one is silent, so CHAN_LEFT must
     * see one word per frame and never a zero from the other slot. */
    if (i2s_read_wait(&prev) != I2S_CH_LEFT)
        return fail(31);
    for (i = 0; i < 6u; ++i) {
        if (i2s_read_wait(&l) != I2S_CH_LEFT)
            return fail(32);
        if (l != prev + 16)
            return fail(33);
        prev = l;
    }
    if (i2s_overrun())
        return fail(34);

    /* Dropping master mode releases the pads, and the external master
     * model owns the bus again -- 16-bit stereo, so both the framing and
     * the word length change back under the same receiver. */
    i2s_master_off();
    if (I2S_CTRL & I2S_CTRL_MASTER)
        return fail(35);
    i2s_begin(I2S_WLEN_16, I2S_CHAN_STEREO, I2S_FMT_I2S);
    i2s_flush();
    i2s_read_frame(&l, &r);
    if (r != expect_right(l))
        return fail(36);

    i2s_end();
    return 1;
}

int main(void)
{
    int ok = run();

    /* Leave interrupts masked whatever happened, so a stuck source cannot
     * storm the parked loop below. */
    sys_irq_global_disable();
    i2s_irq_enable(0, 0);
    i2s_end();

    *result = (ok && fail_step == 0u) ? 0x600Du : 0xBADu;

    /* Park in a self-loop, not in wfi: the sim harness stops on 8
     * identical retires, and a wfi with every interrupt masked would trip
     * its "halt with no wake" liveness check. */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}

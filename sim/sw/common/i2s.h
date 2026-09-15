#ifndef I2S_H
#define I2S_H

/*
 * Driver for the axi4_lite_i2s receiver (src/rtl/utils/axi4_lite_i2s.sv)
 * at rv32_pkg::I2S_BASE = 0x1000_C000.
 *
 * Receive only, in either clock role:
 *   - SLAVE (reset): an external device supplies BCLK and LRCK and the two
 *     clock pads are released.
 *   - MASTER: the peripheral generates BCLK and LRCK and drives them out.
 *     This is the mode an INMP441 needs -- that mic is itself a clock
 *     slave, so with both sides listening nothing ever clocks the bus.
 *     i2s_master_hz() sets it up.
 *
 * Register map (word offsets from the peripheral base):
 *   0x00 STATUS (R)    : RX_EMPTY / RX_FULL / RX_OVERRUN(sticky) /
 *                        RX_CH (channel of the FIFO head) / live WS and
 *                        BCLK levels / RX_LEVEL (words in the FIFO)
 *   0x04 CTRL   (RW)   : ENABLE, CHAN, FORMAT, IE_RX, IE_OVR, WLEN
 *   0x08 WATER  (RW)   : RX interrupt watermark in words (reset 1; 0 reads
 *                        as 1)
 *   0x0C RXDATA (R)    : pop one word, SIGN-EXTENDED to 32 bits. THE READ
 *                        POPS, so read STATUS first if the channel matters
 *                        -- which is exactly what i2s_read() does.
 *   0x10 IRQ    (R/W1C): bit0 RX (a LEVEL: RX_LEVEL >= WATER, writing it
 *                        does nothing), bit1 OVERRUN (sticky, W1C)
 *   0x14 CLKDIV (RW)   : master-mode clocks. bits[15:0] DIV
 *                        (BCLK = core / (2*(DIV+1))), bits[23:16] SLOT
 *                        (BCLK per channel slot, so fs = BCLK/(2*SLOT)).
 *                        Reset 24 / 32 = 1 MHz BCLK, 15625 Hz frames from
 *                        a 50 MHz core.
 *
 * The RX interrupt is a level and not a latched flag because the FIFO is
 * the event record: an ISR that drains fewer words than arrived is
 * re-entered instead of leaving a full FIFO behind a cleared flag. Only
 * the overrun, which has no other record, is sticky and needs a W1C --
 * i2s_overrun_clear().
 *
 * CLOCK RATIO, the one thing that breaks first if the audio side is
 * unusual: the receiver oversamples BCLK in the core clock domain (this
 * design has a single clock domain and no CDC), so it needs
 *
 *     f(core) >= 4 x f(BCLK)
 *
 * 12.5 MHz of BCLK at the board's 50 MHz, against the 3.072 MHz that a
 * 48 kHz / 32-bit / stereo frame uses. Simulation fails the run if a
 * testbench violates it; hardware just loses bits.
 *
 * Frame format: FORMAT selects I2S (Philips -- WS changes one BCLK before
 * the MSB) or left-justified (MSB on the WS edge). WS low = left. Bits
 * past WLEN and up to the next WS transition are padding and are
 * discarded, so the slot length does not have to be configured at all: a
 * 32-BCLK slot carrying a 24-bit word works with I2S_WLEN_24, and so does
 * a 64-BCLK slot.
 */

#define I2S_BASE 0x1000C000u

#define I2S_STATUS (*(volatile unsigned int *)(I2S_BASE + 0x00))
#define I2S_CTRL   (*(volatile unsigned int *)(I2S_BASE + 0x04))
#define I2S_WATER  (*(volatile unsigned int *)(I2S_BASE + 0x08))
#define I2S_RXDATA (*(volatile unsigned int *)(I2S_BASE + 0x0C))
#define I2S_IRQ    (*(volatile unsigned int *)(I2S_BASE + 0x10))
#define I2S_CLKDIV (*(volatile unsigned int *)(I2S_BASE + 0x14))

/* Core clock, for i2s_master_hz(). Override with -DI2S_CORE_HZ=... */
#ifndef I2S_CORE_HZ
#define I2S_CORE_HZ 50000000u
#endif

/* STATUS bits. */
#define I2S_ST_RX_EMPTY  (1u << 0)
#define I2S_ST_RX_FULL   (1u << 1)
#define I2S_ST_OVERRUN   (1u << 2)
#define I2S_ST_RX_CH     (1u << 3)
#define I2S_ST_WS        (1u << 4)
#define I2S_ST_BCLK      (1u << 5)
#define I2S_ST_LEVEL_SH  8
#define I2S_ST_LEVEL_MSK 0x1Fu

/* CTRL bits. */
#define I2S_CTRL_ENABLE  (1u << 0)
#define I2S_CTRL_CHAN_SH 1
#define I2S_CTRL_FMT_LJ  (1u << 3)
#define I2S_CTRL_IE_RX   (1u << 4)
#define I2S_CTRL_IE_OVR  (1u << 5)
#define I2S_CTRL_MASTER  (1u << 6)
#define I2S_CTRL_WLEN_SH 8

/* IRQ bits. */
#define I2S_IRQ_RX  (1u << 0)
#define I2S_IRQ_OVR (1u << 1)

/* Channel select (CTRL.CHAN). Stereo queues both words per frame, left
 * first; the other two drop one channel at the receiver so the FIFO holds
 * twice as many usable frames. */
#define I2S_CHAN_STEREO 0u
#define I2S_CHAN_LEFT   1u
#define I2S_CHAN_RIGHT  2u

/* Word length (CTRL.WLEN). An encoding, not a bit count. */
#define I2S_WLEN_16 0u
#define I2S_WLEN_24 1u
#define I2S_WLEN_32 2u
#define I2S_WLEN_8  3u

/* Frame format (CTRL.FORMAT). */
#define I2S_FMT_I2S 0u
#define I2S_FMT_LJ  1u

/* Channel of a word, as returned by i2s_read(). */
#define I2S_CH_LEFT  0
#define I2S_CH_RIGHT 1

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

/* Start receiving. wlen is an I2S_WLEN_* encoding, chan an I2S_CHAN_*,
 * fmt an I2S_FMT_*. Interrupts are left disabled -- add them with
 * i2s_irq_enable() after setting a watermark. The MASTER bit is PRESERVED,
 * so i2s_master_hz() and i2s_begin() can be called in either order. */
static void i2s_begin(unsigned int wlen, unsigned int chan, unsigned int fmt)
{
    unsigned int master = I2S_CTRL & I2S_CTRL_MASTER;

    I2S_CTRL = I2S_CTRL_ENABLE | (chan << I2S_CTRL_CHAN_SH)
             | (fmt ? I2S_CTRL_FMT_LJ : 0u) | (wlen << I2S_CTRL_WLEN_SH) | master;
}

/*
 * Become the I2S clock master: BCLK = core/(2*(div+1)), LRCK toggles every
 * `slot` BCLK periods, so fs = core / (4 * (div+1) * slot). Call before or
 * after i2s_begin() -- this one only touches CLKDIV and the MASTER bit.
 *
 * Returns the sample rate actually produced, which is NOT always the one
 * asked for: only rates that divide the core clock exactly are reachable.
 * From 50 MHz with a 64-BCLK frame the whole set is 390625/(div+1) Hz with
 * (div+1) a power of 5 for an integer result, so 15625 Hz (div = 24) is
 * the one usable audio rate -- 16000 Hz is NOT reachable. Check the return
 * value and derive the rest of the signal chain from it rather than from
 * what was requested.
 */
static unsigned int i2s_master_hz(unsigned int fs_hz, unsigned int slot)
{
    unsigned int per = 4u * slot * fs_hz; /* core cycles per frame */
    unsigned int div;

    if (slot == 0u)
        slot = 32u;
    if (per == 0u || I2S_CORE_HZ / per == 0u)
        div = 0u;
    else
        div = (I2S_CORE_HZ / per) - 1u;

    I2S_CLKDIV = ((slot & 0xFFu) << 16) | (div & 0xFFFFu);
    I2S_CTRL |= I2S_CTRL_MASTER;

    return I2S_CORE_HZ / (4u * slot * (div + 1u));
}

/* Release the clock pads and go back to listening to an external master. */
static void i2s_master_off(void)
{
    I2S_CTRL &= ~I2S_CTRL_MASTER;
}

/* Stop receiving. The FIFO keeps whatever it already holds. */
static void i2s_end(void)
{
    I2S_CTRL &= ~I2S_CTRL_ENABLE;
}

/* Watermark in words, for the RX interrupt (0 behaves as 1). */
static void i2s_set_watermark(unsigned int words)
{
    I2S_WATER = words;
}

/* Interrupt enables. Both are level conditions; see the header for why
 * the RX one is not latched. */
static void i2s_irq_enable(int rx, int overrun)
{
    unsigned int c = I2S_CTRL & ~(I2S_CTRL_IE_RX | I2S_CTRL_IE_OVR);
    if (rx)
        c |= I2S_CTRL_IE_RX;
    if (overrun)
        c |= I2S_CTRL_IE_OVR;
    I2S_CTRL = c;
}

/* ------------------------------------------------------------------ */
/* Status                                                              */
/* ------------------------------------------------------------------ */

static unsigned int i2s_level(void)
{
    return (I2S_STATUS >> I2S_ST_LEVEL_SH) & I2S_ST_LEVEL_MSK;
}

static int i2s_available(void)
{
    return (I2S_STATUS & I2S_ST_RX_EMPTY) ? 0 : 1;
}

/* Did the receiver drop a word because the FIFO was full? Sticky until
 * cleared, so a polling loop can check it once per batch rather than per
 * sample. */
static int i2s_overrun(void)
{
    return (I2S_STATUS & I2S_ST_OVERRUN) ? 1 : 0;
}

static void i2s_overrun_clear(void)
{
    I2S_IRQ = I2S_IRQ_OVR;
}

/* Is the master clocking at all? The first question to ask of a silent
 * I2S link, and the reason STATUS carries the live pin levels: a stopped
 * BCLK reads as a constant here, a running one flickers. Call it twice
 * and compare. */
static unsigned int i2s_pin_levels(void)
{
    return I2S_STATUS & (I2S_ST_WS | I2S_ST_BCLK);
}

/* ------------------------------------------------------------------ */
/* Receiving                                                           */
/* ------------------------------------------------------------------ */

/* Pop one word. Returns its channel (I2S_CH_LEFT / I2S_CH_RIGHT) and
 * stores the sign-extended sample through *sample, or returns -1 and
 * touches nothing if the FIFO is empty.
 *
 * The STATUS read has to come first: the RXDATA read is what pops, so the
 * channel tag of the word being popped is only visible before it. */
static int i2s_read(int *sample)
{
    unsigned int st = I2S_STATUS;
    if (st & I2S_ST_RX_EMPTY)
        return -1;
    *sample = (int)I2S_RXDATA;
    return (st & I2S_ST_RX_CH) ? I2S_CH_RIGHT : I2S_CH_LEFT;
}

/* Pop one word, spinning until one arrives. Only safe while the master is
 * actually clocking -- there is no timeout, by design: a bounded version
 * would need a timebase this header does not want to depend on. Use
 * i2s_read() plus your own loop if you need to give up. */
static int i2s_read_wait(int *sample)
{
    int ch;
    while ((ch = i2s_read(sample)) < 0) {
        /* spin */
    }
    return ch;
}

/* Pop a frame-aligned left/right pair, discarding words until a left one
 * turns up. Requires CHAN = stereo; with a single-channel filter there is
 * no right word to pair and this spins forever. */
static void i2s_read_frame(int *left, int *right)
{
    int s;
    for (;;) {
        if (i2s_read_wait(&s) == I2S_CH_LEFT) {
            *left = s;
            break;
        }
    }
    (void)i2s_read_wait(&s);
    *right = s;
}

/* Read n words into buf, spinning for each. Channel tags are dropped: in
 * stereo the words interleave left, right, left, ... from wherever the
 * FIFO happened to be, so call i2s_flush() and start on a left word if
 * the pairing matters. */
static void i2s_read_buf(int *buf, unsigned int n)
{
    for (unsigned int i = 0; i < n; ++i)
        (void)i2s_read_wait(&buf[i]);
}

/* Drop everything queued, and the overrun flag with it: the state after
 * this call is "whatever arrives from now on". */
static void i2s_flush(void)
{
    while (!(I2S_STATUS & I2S_ST_RX_EMPTY))
        (void)I2S_RXDATA;
    i2s_overrun_clear();
}

#endif /* I2S_H */

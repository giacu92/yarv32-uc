// Verilator C++ BFM for the axi4_lite_i2s unit test.
//
// Bit-bangs the audio bus by hand (rather than instantiating the
// i2s_master_model that sim_top uses) so the frames a real master cannot
// or would not produce are testable too: a truncated word, a slot longer
// or shorter than the default, a receiver enabled mid-word, a stopped
// BCLK.
//
// Transmitter convention, the same one the model implements: WS and SD
// change on the FALLING BCLK edge, so the receiver's rising edge always
// lands in the middle of a stable bit.
//
// Build: make   (in sim/hw/i2s_tb/)
// Run:   make run

#include "Vi2s_tb.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>

// Register offsets (byte addresses within the peripheral window).
static const uint32_t REG_STATUS = 0x00;
static const uint32_t REG_CTRL   = 0x04;
static const uint32_t REG_WATER  = 0x08;
static const uint32_t REG_RXDATA = 0x0C;
static const uint32_t REG_IRQ    = 0x10;
static const uint32_t REG_CLKDIV = 0x14;

// STATUS bits.
static const uint32_t ST_RX_EMPTY = 1u << 0;
static const uint32_t ST_RX_FULL  = 1u << 1;
static const uint32_t ST_OVERRUN  = 1u << 2;
static const uint32_t ST_RX_CH    = 1u << 3;
static const uint32_t ST_WS       = 1u << 4;
static const uint32_t ST_BCLK     = 1u << 5;
static const uint32_t ST_LEVEL_SH = 8;
static const uint32_t ST_LEVEL_MSK= 0x1Fu;

// CTRL fields.
static const uint32_t CTRL_ENABLE = 1u << 0;
static const uint32_t CTRL_CHAN_SH= 1;
static const uint32_t CTRL_LJ     = 1u << 3;
static const uint32_t CTRL_IE_RX  = 1u << 4;
static const uint32_t CTRL_IE_OVR = 1u << 5;
static const uint32_t CTRL_MASTER = 1u << 6;
static const uint32_t CTRL_WLEN_SH= 8;

// CHAN encodings.
static const uint32_t CHAN_STEREO = 0;
static const uint32_t CHAN_LEFT   = 1;
static const uint32_t CHAN_RIGHT  = 2;

// WLEN encodings -> bits.
static const uint32_t WL16 = 0, WL24 = 1, WL32 = 2, WL8 = 3;

static const uint32_t IRQ_RX  = 1u << 0;
static const uint32_t IRQ_OVR = 1u << 1;

static const int FIFO_DEPTH = 16;

// Core clock cycles per BCLK half period. The receiver requires >= 2.
static const int BCLK_HALF = 3;

static Vi2s_tb* top;
static int fails = 0;
static int checks = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        ++checks;                                                         \
        if (!(cond)) {                                                    \
            printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg);        \
            ++fails;                                                      \
        }                                                                 \
    } while (0)

#define CHECK_EQ(got, want, msg)                                          \
    do {                                                                  \
        ++checks;                                                         \
        if ((uint32_t)(got) != (uint32_t)(want)) {                        \
            printf("FAIL [%s:%d]: %s (got 0x%x, want 0x%x)\n",            \
                   __FILE__, __LINE__, msg, (unsigned)(got),              \
                   (unsigned)(want));                                     \
            ++fails;                                                      \
        }                                                                 \
    } while (0)

// ---------------------------------------------------------------------
// Master-mode loopback: a pad plus an INMP441-shaped slave device.
//
// With the loop enabled, tick() feeds the generated clocks straight back
// into the sampled inputs (the peripheral's contract is pre-synchronized
// inputs, so a delay-free pad is the right model here -- the 2-flop chain
// lives in the tops) and drives SD from the little slave below.
//
// The slave is what an INMP441 does: 24 bits, MSB first, in the LEFT slot
// only (L/R tied low), I2S framing, so slot bit 0 is the WS-change edge
// and carries nothing and the MSB lands on slot bit 1. Data changes on the
// FALLING BCLK edge. The right slot is silent.
// ---------------------------------------------------------------------
static bool mic_loop = false;
static uint32_t mic_word = 0;   // 24-bit sample the device shifts out
static int mic_sd = 0;
static int mic_prev_bclk = 0;
static int mic_ws = 0;          // WS as seen at the last falling edge
static int mic_idx = 0;         // bit index within the current slot

static void mic_step() {
    int b = top->i2s_bclk_o;
    int w = top->i2s_lrck_o;

    if (mic_prev_bclk && !b) {   // falling edge: present the next bit
        if (w != mic_ws) {
            mic_ws = w;
            mic_idx = 0;
        } else {
            ++mic_idx;
        }
        if (mic_ws == 0 && mic_idx >= 1 && mic_idx <= 24)
            mic_sd = (int)((mic_word >> (24 - mic_idx)) & 1u);
        else
            mic_sd = 0;
    }
    mic_prev_bclk = b;
}

static void tick() {
    if (mic_loop) {
        top->i2s_bclk_i = top->i2s_bclk_o;
        top->i2s_lrck_i = top->i2s_lrck_o;
        top->i2s_sd_i = mic_sd;
        top->eval();
    }
    top->clk_i = 0;
    top->eval();
    top->clk_i = 1;
    top->eval();
    if (mic_loop)
        mic_step();
}

static void idle() {
    top->awvalid = 0;
    top->wvalid = 0;
    top->arvalid = 0;
    top->bready = 1;
    top->rready = 1;
    top->eval();
}

static void axi_write_strb(uint32_t addr, uint32_t data, uint8_t strb) {
    top->awaddr = addr;
    top->awvalid = 1;
    top->wdata = data;
    top->wstrb = strb;
    top->wvalid = 1;
    top->bready = 1;
    bool aw = false, w = false, b = false;
    for (int guard = 0; guard < 64 && !(aw && w && b); ++guard) {
        top->eval();
        if (top->awvalid && top->awready) aw = true;
        if (top->wvalid && top->wready) w = true;
        if (top->bvalid && top->bready) b = true;
        tick();
        if (aw) top->awvalid = 0;
        if (w) top->wvalid = 0;
    }
    CHECK(aw && w && b, "axi_write did not complete");
    idle();
}

static void axi_write(uint32_t addr, uint32_t data) {
    axi_write_strb(addr, data, 0xF);
}

static uint32_t axi_read(uint32_t addr) {
    top->araddr = addr;
    top->arvalid = 1;
    top->rready = 1;
    uint32_t val = 0;
    bool ar = false, r = false;
    for (int guard = 0; guard < 64 && !(ar && r); ++guard) {
        top->eval();
        if (top->arvalid && top->arready) ar = true;
        if (top->rvalid && top->rready) {
            val = top->rdata;
            r = true;
        }
        tick();
        if (ar) top->arvalid = 0;
    }
    CHECK(ar && r, "axi_read did not complete");
    idle();
    return val;
}

// ---------------------------------------------------------------------
// Audio bus driver
// ---------------------------------------------------------------------

// Drive one BCLK period: the falling edge presents ws/sd, the rising edge
// is where the receiver samples.
static void i2s_bit(int ws, int sd) {
    top->i2s_lrck_i = ws;
    top->i2s_sd_i = sd;
    for (int i = 0; i < BCLK_HALF; ++i) {
        top->i2s_bclk_i = 0;
        tick();
    }
    for (int i = 0; i < BCLK_HALF; ++i) {
        top->i2s_bclk_i = 1;
        tick();
    }
}

// One channel slot. In I2S format slot bit 0 is the WS-change edge and
// carries no data (the MSB is bit 1); in left-justified format the MSB is
// bit 0. Everything past the word is padding.
static void send_slot(int ws, uint32_t word, int wlen, bool lj, int slot_bits) {
    for (int i = 0; i < slot_bits; ++i) {
        int b;
        if (lj)
            b = (i < wlen) ? (int)((word >> (wlen - 1 - i)) & 1u) : 0;
        else
            b = (i >= 1 && i <= wlen) ? (int)((word >> (wlen - i)) & 1u) : 0;
        i2s_bit(ws, b);
    }
}

static void send_frame(uint32_t l, uint32_t r, int wlen, bool lj, int slot_bits) {
    send_slot(0, l, wlen, lj, slot_bits);
    send_slot(1, r, wlen, lj, slot_bits);
}

// The bus parks with WS high (right slot), so the next left slot always
// starts with a real WS transition and the first word of a burst is
// captured instead of discarded.
static void bus_park() {
    top->i2s_bclk_i = 0;
    top->i2s_lrck_i = 1;
    top->i2s_sd_i = 0;
    for (int i = 0; i < 4; ++i) tick();
}

static uint32_t ctrl_word(bool enable, uint32_t chan, bool lj, bool ie_rx,
                          bool ie_ovr, uint32_t wlen_sel) {
    uint32_t v = 0;
    if (enable) v |= CTRL_ENABLE;
    v |= chan << CTRL_CHAN_SH;
    if (lj) v |= CTRL_LJ;
    if (ie_rx) v |= CTRL_IE_RX;
    if (ie_ovr) v |= CTRL_IE_OVR;
    v |= wlen_sel << CTRL_WLEN_SH;
    return v;
}

static uint32_t rx_level() {
    return (axi_read(REG_STATUS) >> ST_LEVEL_SH) & ST_LEVEL_MSK;
}

// Pop one word, returning its channel tag alongside (STATUS first: the
// RXDATA read is what pops).
static uint32_t pop(int* ch) {
    uint32_t st = axi_read(REG_STATUS);
    if (ch) *ch = (st & ST_RX_CH) ? 1 : 0;
    return axi_read(REG_RXDATA);
}

static void drain() {
    while (!(axi_read(REG_STATUS) & ST_RX_EMPTY)) axi_read(REG_RXDATA);
}

static void reset_dut() {
    top->rstn_i = 0;
    bus_park();
    for (int i = 0; i < 8; ++i) tick();
    top->rstn_i = 1;
    for (int i = 0; i < 4; ++i) tick();
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

// One word length end to end, both framings: send a pair of words,
// check value, sign extension and channel tag.
static void test_wordlen(uint32_t wlen_sel, int wlen, bool lj, uint32_t lw,
                         uint32_t rw, int32_t lexp, int32_t rexp, const char* what) {
    char msg[128];
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, lj, false, false, wlen_sel));
    drain();
    send_frame(lw, rw, wlen, lj, wlen + 4);  // 4 padding bits past the word

    int ch = 9;
    uint32_t got = pop(&ch);
    snprintf(msg, sizeof(msg), "%s: left value", what);
    CHECK_EQ(got, (uint32_t)lexp, msg);
    snprintf(msg, sizeof(msg), "%s: left channel tag", what);
    CHECK_EQ(ch, 0, msg);

    got = pop(&ch);
    snprintf(msg, sizeof(msg), "%s: right value", what);
    CHECK_EQ(got, (uint32_t)rexp, msg);
    snprintf(msg, sizeof(msg), "%s: right channel tag", what);
    CHECK_EQ(ch, 1, msg);

    snprintf(msg, sizeof(msg), "%s: FIFO empty after both words", what);
    CHECK(axi_read(REG_STATUS) & ST_RX_EMPTY, msg);
}

// Sample the generated clocks for `n` ticks and check the shape against
// the programmed divider: every BCLK half period is DIV+1 core cycles,
// every LRCK half period is SLOT BCLK periods, and LRCK only ever moves on
// a falling BCLK edge (the transmitter convention -- moving it on a rising
// edge would put the receiver's sampling instant on top of the transition).
static void check_master_timing(int div, int slot, const char* what) {
    static unsigned char bclk[20000];
    static unsigned char ws[20000];
    const int n = 20000;
    char msg[128];
    int i;

    for (i = 0; i < n; ++i) {
        tick();
        bclk[i] = (unsigned char)top->i2s_bclk_o;
        ws[i] = (unsigned char)top->i2s_lrck_o;
    }

    int last_edge = -1, bad_half = 0, halves = 0;
    int last_ws_edge = -1, bad_slot = 0, ws_halves = 0, bad_ws_phase = 0;
    int rises_since_ws = 0, rises = 0;

    for (i = 1; i < n; ++i) {
        int edge = (bclk[i] != bclk[i - 1]);
        if (edge) {
            if (last_edge >= 0 && (i - last_edge) != div + 1)
                ++bad_half;
            if (last_edge >= 0)
                ++halves;
            last_edge = i;
            if (bclk[i]) {
                ++rises;
                ++rises_since_ws;
            }
        }
        if (ws[i] != ws[i - 1]) {
            /* LRCK must move on a falling BCLK edge. */
            if (!(edge && !bclk[i]))
                ++bad_ws_phase;
            if (last_ws_edge >= 0) {
                if (rises_since_ws != slot)
                    ++bad_slot;
                ++ws_halves;
            }
            last_ws_edge = i;
            rises_since_ws = 0;
        }
    }

    snprintf(msg, sizeof(msg), "%s: BCLK toggles at all", what);
    CHECK(halves > 4, msg);
    snprintf(msg, sizeof(msg), "%s: every BCLK half period is DIV+1 cycles", what);
    CHECK_EQ(bad_half, 0, msg);
    snprintf(msg, sizeof(msg), "%s: LRCK toggles at all", what);
    CHECK(ws_halves > 1, msg);
    snprintf(msg, sizeof(msg), "%s: every LRCK half period is SLOT BCLK periods", what);
    CHECK_EQ(bad_slot, 0, msg);
    snprintf(msg, sizeof(msg), "%s: LRCK only moves on a falling BCLK edge", what);
    CHECK_EQ(bad_ws_phase, 0, msg);
    (void)rises;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vi2s_tb;

    top->clk_i = 0;
    top->rstn_i = 0;
    idle();
    reset_dut();

    // ---------------- reset values ----------------
    uint32_t st = axi_read(REG_STATUS);
    CHECK(st & ST_RX_EMPTY, "reset: RX_EMPTY");
    CHECK(!(st & ST_RX_FULL), "reset: not RX_FULL");
    CHECK(!(st & ST_OVERRUN), "reset: no overrun");
    CHECK(!(st & ST_RX_CH), "reset: channel tag 0 when empty");
    CHECK_EQ((st >> ST_LEVEL_SH) & ST_LEVEL_MSK, 0, "reset: level 0");
    CHECK_EQ(axi_read(REG_CTRL), 0, "reset: CTRL 0 (disabled, 16-bit, I2S)");
    CHECK_EQ(axi_read(REG_WATER), 1, "reset: WATER 1");
    CHECK_EQ(axi_read(REG_IRQ), 0, "reset: IRQ 0");
    CHECK_EQ(axi_read(REG_RXDATA), 0, "reset: empty RXDATA reads 0");
    CHECK_EQ(top->i2s_irq_o, 0, "reset: IRQ line low");

    // STATUS mirrors the live lines (the bring-up question "is the master
    // clocking at all").
    top->i2s_bclk_i = 1;
    top->i2s_lrck_i = 1;
    tick();
    st = axi_read(REG_STATUS);
    CHECK(st & ST_WS, "STATUS.WS follows LRCK high");
    CHECK(st & ST_BCLK, "STATUS.BCLK follows BCLK high");
    bus_park();
    st = axi_read(REG_STATUS);
    CHECK(!(st & ST_BCLK), "STATUS.BCLK follows BCLK low");

    // ---------------- disabled receiver captures nothing ----------------
    send_frame(0x1234, 0x5678, 16, false, 32);
    CHECK(axi_read(REG_STATUS) & ST_RX_EMPTY, "ENABLE=0: nothing captured");

    // ---------------- CTRL readback ----------------
    uint32_t cw = ctrl_word(true, CHAN_RIGHT, true, true, true, WL24);
    axi_write(REG_CTRL, cw);
    CHECK_EQ(axi_read(REG_CTRL), cw, "CTRL readback");
    axi_write(REG_WATER, 7);
    CHECK_EQ(axi_read(REG_WATER), 7, "WATER readback");
    axi_write(REG_WATER, 1);

    // ---------------- framing x word length ----------------
    // I2S (1-BCLK delay), every WLEN encoding, positive and negative.
    test_wordlen(WL16, 16, false, 0x1234, 0xFFFF, 0x00001234, -1, "I2S 16-bit");
    test_wordlen(WL16, 16, false, 0x8000, 0x7FFF, -32768, 32767, "I2S 16-bit extremes");
    test_wordlen(WL8, 8, false, 0x7F, 0x80, 127, -128, "I2S 8-bit");
    test_wordlen(WL24, 24, false, 0x7FFFFF, 0x800000, 8388607, -8388608, "I2S 24-bit");
    test_wordlen(WL32, 32, false, 0x12345678, 0x80000000, 0x12345678,
                 (int32_t)0x80000000, "I2S 32-bit");

    // Left-justified: the MSB rides the WS-change edge.
    test_wordlen(WL16, 16, true, 0x1234, 0xFFFF, 0x00001234, -1, "LJ 16-bit");
    test_wordlen(WL24, 24, true, 0xABCDEF, 0x123456, (int32_t)0xFFABCDEF, 0x123456,
                 "LJ 24-bit");

    // A slot exactly as long as the word (no padding at all) still works:
    // the WS transition is what ends a word, and the word ended first.
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, false, false, WL16));
    drain();
    send_frame(0x0F0F, 0xF0F0, 16, false, 17);  // I2S needs WLEN+1 slot bits
    CHECK_EQ(pop(nullptr), 0x0F0F, "minimal slot: left");
    CHECK_EQ(pop(nullptr), (uint32_t)(int32_t)(int16_t)0xF0F0, "minimal slot: right");

    // A very long slot (64 BCLK) is all padding after the word.
    send_frame(0x0102, 0x0304, 16, false, 64);
    CHECK_EQ(pop(nullptr), 0x0102, "64-bit slot: left");
    CHECK_EQ(pop(nullptr), 0x0304, "64-bit slot: right");
    CHECK(axi_read(REG_STATUS) & ST_RX_EMPTY, "64-bit slot: exactly 2 words");

    // ---------------- resync ----------------
    // A word cut short by an early WS transition is discarded, and the
    // next complete word is captured: losing lock costs one word.
    drain();
    for (int i = 0; i < 9; ++i) i2s_bit(0, 1);  // 8 bits of a 16-bit left word
    send_slot(1, 0x2222, 16, false, 32);        // WS flips early
    CHECK_EQ(rx_level(), 1, "truncated word dropped, next one kept");
    CHECK_EQ(pop(nullptr), 0x2222, "resync: the complete word is the right one");

    // ---------------- channel filter ----------------
    axi_write(REG_CTRL, ctrl_word(true, CHAN_LEFT, false, false, false, WL16));
    drain();
    send_frame(0x0AAA, 0x0BBB, 16, false, 32);
    CHECK_EQ(rx_level(), 1, "CHAN=left: one word per frame");
    int ch = 9;
    CHECK_EQ(pop(&ch), 0x0AAA, "CHAN=left: the left word");
    CHECK_EQ(ch, 0, "CHAN=left: channel tag 0");

    axi_write(REG_CTRL, ctrl_word(true, CHAN_RIGHT, false, false, false, WL16));
    drain();
    send_frame(0x0AAA, 0x0BBB, 16, false, 32);
    CHECK_EQ(rx_level(), 1, "CHAN=right: one word per frame");
    CHECK_EQ(pop(&ch), 0x0BBB, "CHAN=right: the right word");
    CHECK_EQ(ch, 1, "CHAN=right: channel tag 1");

    // ---------------- FIFO fill, full, overrun ----------------
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, false, false, WL16));
    drain();
    for (int i = 0; i < FIFO_DEPTH / 2; ++i)
        send_frame(0x100 + 2 * i, 0x100 + 2 * i + 1, 16, false, 32);
    st = axi_read(REG_STATUS);
    CHECK_EQ((st >> ST_LEVEL_SH) & ST_LEVEL_MSK, FIFO_DEPTH, "FIFO filled to depth");
    CHECK(st & ST_RX_FULL, "RX_FULL at depth");
    CHECK(!(st & ST_OVERRUN), "no overrun at exactly depth");

    // One more frame: both words are dropped and the flag sticks.
    send_frame(0xDEAD, 0xBEEF, 16, false, 32);
    st = axi_read(REG_STATUS);
    CHECK(st & ST_OVERRUN, "overrun set when full");
    CHECK_EQ((st >> ST_LEVEL_SH) & ST_LEVEL_MSK, FIFO_DEPTH, "overrun does not grow the FIFO");
    CHECK(axi_read(REG_IRQ) & IRQ_OVR, "IRQ.OVERRUN mirrors STATUS.RX_OVERRUN");

    // The FIFO still holds the first DEPTH words, in order: an overrun
    // drops the NEW word, it does not corrupt the queue.
    for (int i = 0; i < FIFO_DEPTH; ++i) {
        uint32_t got = pop(&ch);
        CHECK_EQ(got, 0x100 + i, "FIFO order preserved across an overrun");
        CHECK_EQ(ch, i & 1, "FIFO channel tags preserved across an overrun");
    }
    CHECK(axi_read(REG_STATUS) & ST_RX_EMPTY, "FIFO drained");

    // Sticky until cleared, and the W1C only touches the bit written.
    CHECK(axi_read(REG_STATUS) & ST_OVERRUN, "overrun still sticky after draining");
    axi_write(REG_IRQ, IRQ_RX);  // writing the level bit clears nothing
    CHECK(axi_read(REG_STATUS) & ST_OVERRUN, "W1C of bit0 does not clear the overrun");
    axi_write(REG_IRQ, IRQ_OVR);
    CHECK(!(axi_read(REG_STATUS) & ST_OVERRUN), "W1C clears the overrun");
    axi_write_strb(REG_IRQ, IRQ_OVR, 0x0);  // zero-strobe write must do nothing

    // ---------------- watermark IRQ (a level, not a latch) ----------------
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, true, false, WL16));
    axi_write(REG_WATER, 4);
    drain();
    send_frame(0x11, 0x22, 16, false, 32);  // level 2
    CHECK_EQ(top->i2s_irq_o, 0, "watermark 4: no IRQ at level 2");
    CHECK(!(axi_read(REG_IRQ) & IRQ_RX), "watermark 4: IRQ.RX low at level 2");
    send_frame(0x33, 0x44, 16, false, 32);  // level 4
    CHECK_EQ(top->i2s_irq_o, 1, "watermark 4: IRQ at level 4");
    CHECK(axi_read(REG_IRQ) & IRQ_RX, "watermark 4: IRQ.RX high at level 4");

    // Writing the RX bit must not clear it -- it is a level.
    axi_write(REG_IRQ, IRQ_RX);
    CHECK_EQ(top->i2s_irq_o, 1, "IRQ.RX is a level: W1C does not clear it");

    // Popping below the watermark drops it, and it comes back with no
    // clear in between. That is the property an edge-latched flag would
    // not have, and why this one is a level.
    axi_read(REG_RXDATA);
    CHECK_EQ(top->i2s_irq_o, 0, "IRQ drops below the watermark");
    send_frame(0x55, 0x66, 16, false, 32);
    CHECK_EQ(top->i2s_irq_o, 1, "IRQ reasserts without a clear");

    // IE_RX gates the line but not the status bit.
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, false, false, WL16));
    CHECK_EQ(top->i2s_irq_o, 0, "IE_RX=0 masks the line");
    CHECK(axi_read(REG_IRQ) & IRQ_RX, "IE_RX=0 leaves IRQ.RX visible");

    // WATER=0 behaves as 1 (a watermark of "zero or more" would never drop).
    drain();
    axi_write(REG_WATER, 0);
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, true, false, WL16));
    CHECK_EQ(top->i2s_irq_o, 0, "WATER=0 reads as 1: no IRQ on an empty FIFO");
    send_frame(0x77, 0x88, 16, false, 32);
    CHECK_EQ(top->i2s_irq_o, 1, "WATER=0 reads as 1: IRQ on the first word");
    axi_write(REG_WATER, 1);

    // ---------------- IE_OVR gating ----------------
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, false, true, WL16));
    drain();
    CHECK_EQ(top->i2s_irq_o, 0, "IE_OVR=1 with no overrun: line low");
    for (int i = 0; i < FIFO_DEPTH / 2 + 1; ++i)
        send_frame(0x200 + 2 * i, 0x200 + 2 * i + 1, 16, false, 32);
    CHECK_EQ(top->i2s_irq_o, 1, "IE_OVR=1: overrun raises the line");
    axi_write(REG_IRQ, IRQ_OVR);
    drain();
    CHECK_EQ(top->i2s_irq_o, 0, "overrun cleared: line low");

    // ---------------- disable mid-stream ----------------
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, false, false, WL16));
    drain();
    send_frame(0x0301, 0x0302, 16, false, 32);
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16));
    send_frame(0x0401, 0x0402, 16, false, 32);
    CHECK_EQ(rx_level(), 2, "disable stops capture, FIFO content kept");
    CHECK_EQ(pop(nullptr), 0x0301, "kept word 0");
    CHECK_EQ(pop(nullptr), 0x0302, "kept word 1");

    // Re-enabling mid-word costs at most one word: the first WS transition
    // after the enable restarts collection.
    axi_write(REG_CTRL, ctrl_word(true, CHAN_STEREO, false, false, false, WL16));
    send_frame(0x0501, 0x0502, 16, false, 32);
    CHECK_EQ(rx_level(), 2, "re-enabled at a slot boundary: both words");
    CHECK_EQ(pop(nullptr), 0x0501, "re-enabled: left");
    CHECK_EQ(pop(nullptr), 0x0502, "re-enabled: right");

    // ---------------- stopped clock ----------------
    // A master that stops clocking must leave the receiver quiet, not
    // shift in the idle level.
    drain();
    for (int i = 0; i < 200; ++i) tick();
    CHECK(axi_read(REG_STATUS) & ST_RX_EMPTY, "no BCLK: nothing captured");

    // ---------------- master mode: clock generation ----------------
    // The reset value is the useful one on this board: DIV=24 and SLOT=32
    // give BCLK = 1.000 MHz and fs = 15625 Hz exactly from a 50 MHz core.
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16));
    CHECK_EQ(axi_read(REG_CLKDIV), (32u << 16) | 24u, "CLKDIV reset = SLOT 32, DIV 24");
    CHECK_EQ(top->i2s_clk_oe_o, 0, "MASTER=0: clock output enable low");

    // A small divider and a short slot, so a whole frame fits in the trace
    // the timing check records.
    axi_write(REG_CLKDIV, (8u << 16) | 1u);   // BCLK = clk/4, 8 BCLK per slot
    CHECK_EQ(axi_read(REG_CLKDIV), (8u << 16) | 1u, "CLKDIV readback");
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16) | CTRL_MASTER);
    CHECK(axi_read(REG_CTRL) & CTRL_MASTER, "CTRL.MASTER readback");
    CHECK_EQ(top->i2s_clk_oe_o, 1, "MASTER=1: clock output enable high");
    check_master_timing(1, 8, "master DIV=1 SLOT=8");

    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16));
    axi_write(REG_CLKDIV, (16u << 16) | 3u);  // BCLK = clk/8, 16 BCLK per slot
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16) | CTRL_MASTER);
    check_master_timing(3, 16, "master DIV=3 SLOT=16");

    // SLOT=0 is not expressible as a frame (LRCK would toggle every BCLK),
    // so the generator reads it as 32 -- the register keeps the 0.
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16));
    axi_write(REG_CLKDIV, (0u << 16) | 1u);
    CHECK_EQ(axi_read(REG_CLKDIV) >> 16, 0u, "SLOT=0 is stored as written");
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16) | CTRL_MASTER);
    check_master_timing(1, 32, "master SLOT=0 behaves as 32");

    // ---------------- master mode: INMP441-shaped loopback ----------
    // The clocks come back through the pad and a 24-bit left-channel slave
    // answers on them, which is the board configuration: nothing external
    // clocks anything, the receiver decodes what its own clock produced.
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16));
    axi_write(REG_CLKDIV, (32u << 16) | 1u);  // BCLK = clk/4, 64-BCLK frame
    mic_loop = true;
    mic_word = 0x123456u;
    axi_write(REG_CTRL,
              ctrl_word(true, CHAN_LEFT, false, false, false, WL24) | CTRL_MASTER);
    drain();
    {
        int got_ch = 9;
        uint32_t got;
        int spin = 0;
        while ((axi_read(REG_STATUS) & ST_RX_EMPTY) && ++spin < 40000) tick();
        got = pop(&got_ch);
        CHECK_EQ(got, 0x123456u, "master loopback: positive 24-bit word");
        CHECK_EQ(got_ch, 0, "master loopback: left channel");
    }

    // A negative word, to prove the sign extension survives the loop, and
    // a second value to prove the stream keeps running.
    mic_word = 0xF00000u;   /* -1048576 in 24-bit two's complement */
    drain();
    {
        int spin = 0;
        uint32_t got;
        while ((axi_read(REG_STATUS) & ST_RX_EMPTY) && ++spin < 40000) tick();
        got = pop(nullptr);
        CHECK_EQ(got, (uint32_t)(int32_t)-1048576, "master loopback: negative word");
    }

    // CHAN=left with a silent right slot: one word per frame, never a
    // zero from the right slot sneaking in.
    {
        int spin = 0, words = 0;
        mic_word = 0x00ABCDu;
        drain();
        while (words < 4 && ++spin < 200000) {
            if (!(axi_read(REG_STATUS) & ST_RX_EMPTY)) {
                int ch = 9;
                uint32_t got = pop(&ch);
                CHECK_EQ(got, 0x00ABCDu, "master loopback: repeated word");
                CHECK_EQ(ch, 0, "master loopback: left only");
                ++words;
            }
            tick();
        }
        CHECK_EQ(words, 4, "master loopback: frames keep arriving");
    }

    mic_loop = false;
    axi_write(REG_CTRL, ctrl_word(false, CHAN_STEREO, false, false, false, WL16));

    printf("\n%d checks, %d failures\n", checks, fails);
    top->final();
    delete top;
    return fails ? 1 : 0;
}

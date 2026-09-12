// Verilator C++ BFM for the axi4_lite_spi compliance test.
//
// Models a real SPI slave (an 8-bit shift register clocked by SCK edges,
// per CPOL/CPHA) -- NOT a combinational MISO=MOSI loopback, which would
// hide a wrong sample edge. The slave presents a queued response byte on
// MISO and captures MOSI into its own shift register, so a wrong edge
// garbles both and the check fails.
//
// Edge convention matches the master's own (the "leading edge" in the RTL
// is the tick where the OLD sck differs from cpol, i.e. physically falling
// for CPOL=0 / rising for CPOL=1; CPHA selects whether that edge samples
// or changes):
//   sample_edge = cpha ? trailing : leading
//   change_edge = cpha ? leading  : trailing
// where leading = (cpol ? rising : falling), trailing = (cpol ? falling : rising).
//
// The master arms a first_edge_q one-shot per byte that suppresses the
// phantom first change edge (MOSI is preset at IDLE). The slave mirrors
// this with s_first_change so it does not shift on that edge either.
//
// Build: make   (in sim/hw/spi_tb/)
// Run:   make run

#include "Vspi_tb.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>
#include <queue>
#include <vector>

// Register offsets (byte addresses within the peripheral window).
static const uint32_t REG_STATUS = 0x00;
static const uint32_t REG_CTRL   = 0x04;
static const uint32_t REG_CLKDIV = 0x08;
static const uint32_t REG_CS     = 0x0C;
static const uint32_t REG_TXDATA = 0x10;
static const uint32_t REG_RXDATA = 0x14;
static const uint32_t REG_IRQ    = 0x18;

static const uint32_t ST_BUSY       = 0x01;
static const uint32_t ST_TX_FULL    = 0x02;
static const uint32_t ST_RX_EMPTY   = 0x04;
static const uint32_t ST_TX_EMPTY   = 0x08;
static const uint32_t ST_RX_FULL    = 0x10;
static const uint32_t ST_RX_OVERRUN = 0x20;

static const uint32_t CTRL_ENABLE  = 0x01;
static const uint32_t CTRL_CPOL    = 0x02;
static const uint32_t CTRL_CPHA    = 0x04;
static const uint32_t CTRL_IE_TX   = 0x08;
static const uint32_t CTRL_IE_RX   = 0x10;
static const uint32_t CTRL_IE_DONE = 0x20;
static const uint32_t CTRL_IE_OVR  = 0x40;

static const uint32_t IRQ_TX_EMPTY = 0x01;
static const uint32_t IRQ_RX_NE    = 0x02;
static const uint32_t IRQ_DONE     = 0x04;
static const uint32_t IRQ_OVR      = 0x08;

// CLKDIV programmed in every test; SCL = clk / (2*(CLKDIV+1)).
static const uint32_t CLKDIV = 2;
static const int FIFO_DEPTH = 16;

static Vspi_tb* top;
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
// SPI slave model
// ---------------------------------------------------------------------
struct SpiSlave {
    int cpol = 0, cpha = 0;
    int sck_prev = 1, cs_prev = 1;
    bool cs_active = false;
    uint8_t sreg = 0;          // MISO shift register (MSB out)
    int bit_cnt = 0;
    uint8_t captured = 0;      // MOSI captured this byte
    bool byte_done = false;    // 8 sample edges seen
    bool first_change = true;  // suppress phantom first change edge (CPHA=0)
    int miso = 1;              // driven MISO level
    std::queue<uint8_t> responses;        // queued response bytes (test fills)
    std::vector<uint8_t> captured_bytes;  // bytes captured from master

    void start_byte(uint8_t resp) {
        sreg = resp;
        bit_cnt = 0;
        captured = 0;
        byte_done = false;
        // CPHA=0: the master's first change edge is phantom (MOSI preset), so
        // suppress the slave's first MISO shift to hold the response MSB for
        // the first sample.  CPHA=1: the first edge is a sample, the first
        // change is real -- shift normally.
        first_change = !cpha;
        miso = (sreg >> 7) & 1;  // present response MSB before first sample
    }

    void step() {
        int sck = top->spi_sck_o;
        int cs = top->spi_cs_n_o;
        int mosi = top->spi_mosi_o & 1;

        // CS edges
        if (cs_prev == 1 && cs == 0) {
            cs_active = true;
            uint8_t resp = responses.empty() ? 0xFF : responses.front();
            if (!responses.empty()) responses.pop();
            start_byte(resp);
        } else if (cs_prev == 0 && cs == 1) {
            cs_active = false;
        }
        cs_prev = cs;

        if (!cs_active) {
            sck_prev = sck;
            top->spi_miso_i = 1;
            return;
        }

        bool rising = (sck == 1 && sck_prev == 0);
        bool falling = (sck == 0 && sck_prev == 1);
        sck_prev = sck;

        bool leading = cpol ? rising : falling;
        bool trailing = cpol ? falling : rising;
        bool sample_edge = cpha ? trailing : leading;
        bool change_edge = cpha ? leading : trailing;

        if (change_edge) {
            if (byte_done && !responses.empty()) {
                // Byte boundary while CS held: the master's phantom first
                // change edge of the new byte is THIS edge (it does not shift
                // MOSI).  Load the next response and present its MSB; the next
                // change edge is real, so do NOT re-arm first_change.
                uint8_t resp = responses.front();
                responses.pop();
                start_byte(resp);
                first_change = false;
            } else if (first_change) {
                // Phantom first change edge: master holds the preset MOSI,
                // slave must not advance its MISO shift register.
                first_change = false;
            } else if (!byte_done) {
                sreg = (uint8_t)((sreg << 1) & 0xFF);
                miso = (sreg >> 7) & 1;
            }
        }

        if (sample_edge && !byte_done) {
            captured = (uint8_t)((captured << 1) | mosi);
            bit_cnt++;
            if (bit_cnt == 8) {
                captured_bytes.push_back(captured);
                byte_done = true;
            }
        }

        top->spi_miso_i = miso;
    }

    void reset_lines() {
        sck_prev = 1;
        cs_prev = 1;
        cs_active = false;
        miso = 1;
        top->spi_miso_i = 1;
    }
};

static SpiSlave slave;

static void tick() {
    top->clk_i = 0;
    top->eval();
    top->clk_i = 1;
    top->eval();
    slave.step();
}

static void idle() {
    top->awvalid = 0;
    top->wvalid = 0;
    top->arvalid = 0;
    top->bready = 1;
    top->rready = 1;
    top->eval();
}

static int axi_write_timed(uint32_t addr, uint32_t data, int guard_cycles) {
    top->awaddr = addr;
    top->awvalid = 1;
    top->wdata = data;
    top->wstrb = 0xF;
    top->wvalid = 1;
    top->bready = 1;
    bool aw = false, w = false, b = false;
    int cycles = 0;
    for (int guard = 0; guard < guard_cycles && !(aw && w && b); ++guard) {
        top->eval();
        if (top->awvalid && top->awready) aw = true;
        if (top->wvalid && top->wready) w = true;
        if (top->bvalid && top->bready) b = true;
        tick();
        ++cycles;
        if (aw) top->awvalid = 0;
        if (w) top->wvalid = 0;
    }
    CHECK(aw && w && b, "axi_write did not complete");
    idle();
    return cycles;
}

static void axi_write(uint32_t addr, uint32_t data) {
    axi_write_timed(addr, data, 64);
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

// Wait for BUSY to clear (transfer complete). Returns false on timeout.
static bool wait_idle(int timeout = 4000) {
    for (int i = 0; i < timeout; ++i) {
        if (!(axi_read(REG_STATUS) & ST_BUSY)) return true;
    }
    return false;
}

// Program mode + divisor and assert CS.
static void spi_setup(int cpol, int cpha) {
    slave.cpol = cpol;
    slave.cpha = cpha;
    axi_write(REG_CTRL, CTRL_ENABLE | (cpol ? CTRL_CPOL : 0) |
                             (cpha ? CTRL_CPHA : 0));
    axi_write(REG_CLKDIV, CLKDIV);
}

// One byte transfer with CS toggled around it. Queues response = ~tx so the
// round trip is trivially distinguishable from a combinational loopback.
static uint8_t spi_xfer_one(uint8_t tx) {
    slave.responses.push((uint8_t)~tx);
    axi_write(REG_CS, 0);  // CS low
    axi_write(REG_TXDATA, tx);
    CHECK(wait_idle(), "transfer did not complete (BUSY stuck)");
    axi_write(REG_CS, 1);  // CS high
    // The slave captured one byte; verify it matches what the master sent.
    if (!slave.captured_bytes.empty()) {
        uint8_t cap = slave.captured_bytes.back();
        slave.captured_bytes.pop_back();
        CHECK_EQ(cap, tx, "slave captured wrong byte (MOSI path)");
    } else {
        CHECK(false, "slave captured no byte");
    }
    uint32_t rx = axi_read(REG_RXDATA) & 0xFF;
    return (uint8_t)rx;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vspi_tb;

    top->clk_i = 0;
    top->rstn_i = 0;
    top->spi_miso_i = 1;
    idle();
    slave.reset_lines();
    for (int i = 0; i < 4; ++i) tick();
    top->rstn_i = 1;
    tick();

    printf("=== axi4_lite_spi mode + FIFO + IRQ compliance ===\n");

    // ---- 1. Reset state ------------------------------------------------
    uint32_t st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_BUSY, 0, "BUSY clear out of reset");
    CHECK_EQ(st & ST_RX_EMPTY, ST_RX_EMPTY, "RX_EMPTY set out of reset");
    CHECK_EQ(st & ST_TX_EMPTY, ST_TX_EMPTY, "TX_EMPTY set out of reset");
    CHECK_EQ(top->spi_irq_o, 0, "IRQ masked out of reset (IEs are 0)");
    CHECK_EQ(top->spi_cs_n_o, 1, "CS deasserted out of reset");

    // ---- 2. All four CPOL/CPHA modes round-trip a byte -----------------
    // The original bug: CPHA=0 deadlocked. Each mode must complete and
    // return the slave's response (~tx) intact -- a wrong sample edge
    // shifts the wrong bit and the check fails.
    const uint8_t modes_tx[4] = {0xA5, 0x3C, 0x81, 0x5A};
    for (int m = 0; m < 4; ++m) {
        int cpol = (m >> 1) & 1;
        int cpha = m & 1;
        spi_setup(cpol, cpha);
        uint8_t tx = modes_tx[m];
        uint8_t rx = spi_xfer_one(tx);
        CHECK_EQ(rx, (uint8_t)~tx, "mode round-trip returned wrong byte");
    }

    // ---- 3. TX FIFO burst drains in order with CS held (auto-drain) ----
    // The slave loads a response off its queue on CS fall, so all response
    // bytes must be queued BEFORE CS is asserted (else byte 0 pops an empty
    // queue and every byte shifts by one).  CS must still be low before the
    // first TXDATA push: the engine starts shifting the moment TX is non-empty
    // (CS is just an output pin, it does not gate the shift engine), so if CS
    // were high the first byte would shift with CS high and the slave would
    // ignore it.  Push responses, assert CS, then queue the 4 TX bytes; the
    // remaining bytes land in the FIFO while byte 0 is on the wire, so the
    // engine pops and shifts all four back-to-back without software
    // intervention.
    spi_setup(0, 0);
    const uint8_t burst_tx[4] = {0x10, 0x20, 0x30, 0x40};
    for (uint8_t b : burst_tx) slave.responses.push((uint8_t)~b);
    axi_write(REG_CS, 0);
    for (uint8_t b : burst_tx) axi_write(REG_TXDATA, b);
    CHECK(wait_idle(), "burst did not complete (BUSY stuck)");
    axi_write(REG_CS, 1);
    CHECK_EQ(slave.captured_bytes.size(), (size_t)4,
             "burst: slave captured 4 bytes");
    for (int i = 0; i < 4; ++i) {
        uint8_t cap = slave.captured_bytes[i];
        CHECK_EQ(cap, burst_tx[i], "burst: MOSI byte in order");
    }
    for (int i = 0; i < 4; ++i) {
        uint32_t rx = axi_read(REG_RXDATA) & 0xFF;
        CHECK_EQ(rx, (uint8_t)~burst_tx[i], "burst: RX byte in order");
    }
    slave.captured_bytes.clear();
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_RX_EMPTY, ST_RX_EMPTY, "RX empty after burst drain");

    // ---- 4. RX overrun -------------------------------------------------
    // Fill the RX FIFO past its depth without reading: the overflow byte is
    // dropped and RX_OVERRUN latches; the queued bytes survive. Single-byte
    // transfers (CS toggled) keep the slave resynced per byte.
    spi_setup(0, 0);
    for (int i = 0; i < FIFO_DEPTH + 1; ++i) {
        uint8_t tx = (uint8_t)(0x80 + i);
        slave.responses.push((uint8_t)~tx);
        axi_write(REG_CS, 0);
        axi_write(REG_TXDATA, tx);
        CHECK(wait_idle(), "overrun fill: transfer did not complete");
        axi_write(REG_CS, 1);
        if (!slave.captured_bytes.empty()) slave.captured_bytes.clear();
    }
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_RX_OVERRUN, ST_RX_OVERRUN, "RX_OVERRUN latched on drop");
    for (int i = 0; i < FIFO_DEPTH; ++i) {
        uint32_t rx = axi_read(REG_RXDATA) & 0xFF;
        CHECK_EQ(rx, (uint8_t)~(0x80 + i), "overrun: queued byte survived");
    }
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_RX_EMPTY, ST_RX_EMPTY, "RX empty after overrun drain");
    CHECK_EQ(st & ST_RX_OVERRUN, ST_RX_OVERRUN,
             "RX_OVERRUN still set until W1C");
    // W1C clears the sticky overrun flag.
    axi_write(REG_IRQ, IRQ_OVR);
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_RX_OVERRUN, 0, "RX_OVERRUN cleared by IRQ W1C");

    // ---- 5. IRQ: level-sensitive, gated by CTRL; DONE is W1C sticky ----
    spi_setup(0, 0);
    // DONE: fires once when a byte finishes, with IE_DONE set.
    axi_write(REG_CTRL, CTRL_ENABLE | CTRL_IE_DONE);
    slave.responses.push(0x00);
    axi_write(REG_CS, 0);
    axi_write(REG_TXDATA, 0x77);
    CHECK(wait_idle(), "IRQ test transfer did not complete");
    axi_write(REG_CS, 1);
    CHECK_EQ(top->spi_irq_o, 1, "DONE IRQ asserts after a byte with IE_DONE");
    uint32_t irq = axi_read(REG_IRQ);
    CHECK_EQ(irq & IRQ_DONE, IRQ_DONE, "IRQ register exposes DONE bit");
    axi_write(REG_IRQ, IRQ_DONE);  // W1C
    CHECK_EQ(top->spi_irq_o, 0, "DONE IRQ clears on W1C while IE stays on");
    // Drain the RX byte so the next test starts clean.
    (void)axi_read(REG_RXDATA);

    // IE_RX: level IRQ while RX has data.
    axi_write(REG_CTRL, CTRL_ENABLE | CTRL_IE_RX);
    CHECK_EQ(top->spi_irq_o, 0, "no RX IRQ with RX empty");
    slave.responses.push(0x00);
    axi_write(REG_CS, 0);
    axi_write(REG_TXDATA, 0x99);
    CHECK(wait_idle(), "IE_RX transfer did not complete");
    axi_write(REG_CS, 1);
    CHECK_EQ(top->spi_irq_o, 1, "RX IRQ asserts with RX not empty + IE_RX");
    (void)axi_read(REG_RXDATA);
    CHECK_EQ(top->spi_irq_o, 0, "RX IRQ deasserts once RX is drained");
    axi_write(REG_CTRL, 0);
    CHECK_EQ(top->spi_irq_o, 0, "IRQ masked with all IEs clear");

    // ---- 6. CS follows REG_CS, BUSY reflects the engine ----------------
    axi_write(REG_CS, 0);
    CHECK_EQ(top->spi_cs_n_o, 0, "CS low after REG_CS write 0");
    axi_write(REG_CS, 1);
    CHECK_EQ(top->spi_cs_n_o, 1, "CS high after REG_CS write 1");

    printf("%d checks, %d failures\n", checks, fails);
    delete top;
    return fails ? 1 : 0;
}
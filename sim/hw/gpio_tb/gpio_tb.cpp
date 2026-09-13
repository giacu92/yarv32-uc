// Verilator C++ BFM for the axi4_lite_gpio unit test.
//
// Drives gpio_i directly (the external-pin stimulus that sim_top cannot
// provide -- the sim top loops the pins back onto themselves) and checks
// the register/IRQ contract of the peripheral: OUT/DIR, edge detect
// (rise/fall/both), level pending, W1C semantics (set wins over clear),
// INT_EN gating, strobe discipline.
//
// The BFM drives gpio_i SYNCHRONOUSLY with tick() -- the module contract
// says gpio_i must already be synchronized, so the sync chain itself is
// exercised at the tops, not here.
//
// Build: make   (in sim/hw/gpio_tb/)
// Run:   make run

#include "Vgpio_tb.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>

// Register offsets (byte addresses within the peripheral window).
static const uint32_t REG_VALUE     = 0x00;
static const uint32_t REG_OUT       = 0x04;
static const uint32_t REG_DIR       = 0x08;
static const uint32_t REG_INT_TYPE  = 0x0C;
static const uint32_t REG_INT_EN    = 0x10;
static const uint32_t REG_INT_STATUS= 0x14;

// INT_TYPE encodings, 2 bits per pin: level-high, rise, fall, both.
static const uint32_t TYPE_LEVEL = 0;
static const uint32_t TYPE_RISE  = 1;
static const uint32_t TYPE_FALL  = 2;
static const uint32_t TYPE_BOTH  = 3;

static Vgpio_tb* top;
static int fails = 0;
static int checks = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        ++checks;                                                         \
        if (!(cond)) {                                                     \
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

static void tick() {
    top->clk_i = 0;
    top->eval();
    top->clk_i = 1;
    top->eval();
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

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vgpio_tb;

    top->clk_i = 0;
    top->rstn_i = 0;
    top->gpio_i = 0;
    idle();
    for (int i = 0; i < 4; ++i) tick();
    top->rstn_i = 1;
    tick();

    printf("=== axi4_lite_gpio reg / edge / level / IRQ compliance ===\n");

    // ---- 1. Reset state -----------------------------------------------
    CHECK_EQ(axi_read(REG_OUT), 0, "OUT resets to 0");
    CHECK_EQ(axi_read(REG_DIR), 0, "DIR resets to 0 (pads released)");
    CHECK_EQ(top->gpio_oe_o, 0, "oe released out of reset");
    CHECK_EQ(axi_read(REG_INT_EN), 0, "INT_EN resets to 0");
    CHECK_EQ(axi_read(REG_INT_STATUS), 0, "INT_STATUS resets to 0");
    CHECK_EQ(top->gpio_irq_o, 0, "IRQ masked out of reset");
    // gpio_i is 0 here, so VALUE reads 0 and level pins are not pending.
    CHECK_EQ(axi_read(REG_VALUE), 0, "VALUE reads the pin inputs");

    // ---- 2. OUT / DIR write + readback, pins follow -------------------
    axi_write(REG_OUT, 0xA);
    CHECK_EQ(axi_read(REG_OUT), 0xA, "OUT readback");
    CHECK_EQ(top->gpio_o, 0xA, "gpio_o follows OUT");
    axi_write(REG_DIR, 0x5);
    CHECK_EQ(axi_read(REG_DIR), 0x5, "DIR readback");
    CHECK_EQ(top->gpio_oe_o, 0x5, "gpio_oe_o follows DIR");

    // ---- 3. INT_TYPE readback (2 bits per pin) ------------------------
    uint32_t types = (TYPE_RISE << 0) | (TYPE_FALL << 2) | (TYPE_BOTH << 4) |
                     (TYPE_LEVEL << 6);
    axi_write(REG_INT_TYPE, types);
    CHECK_EQ(axi_read(REG_INT_TYPE), types, "INT_TYPE readback");
    axi_write(REG_INT_TYPE, 0);  // back to all-level for the level tests

    // ---- 4. Rise edge pends, W1C on an edge pin STICKS ----------------
    axi_write(REG_INT_TYPE, TYPE_RISE);  // pin 0 rise
    top->gpio_i = 0x1;
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0x1, "rise on pin0 pends");
    CHECK_EQ(top->gpio_irq_o, 0, "pending masked with INT_EN=0 (latched)");
    axi_write(REG_INT_STATUS, 0x1);  // W1C
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0, "W1C clears an edge pin");
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0, "edge pin stays cleared");

    // Fall on pin0 must NOT pend a rise-only pin.
    top->gpio_i = 0x0;
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0, "fall does not pend a rise pin");

    // ---- 5. Fall / both edges -----------------------------------------
    axi_write(REG_INT_TYPE, TYPE_FALL);  // pin 0 fall
    top->gpio_i = 0x0;
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0, "no fall yet (line already low)");
    top->gpio_i = 0x1;
    tick();  // rise, no pend
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0, "rise does not pend a fall pin");
    top->gpio_i = 0x0;
    tick();  // fall
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0x1, "fall on pin0 pends");
    axi_write(REG_INT_STATUS, 0x1);

    axi_write(REG_INT_TYPE, TYPE_BOTH);  // pin 0 both
    top->gpio_i = 0x1;
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0x1, "both: rise pends");
    axi_write(REG_INT_STATUS, 0x1);
    top->gpio_i = 0x0;
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0x1, "both: fall pends");
    axi_write(REG_INT_STATUS, 0x1);

    // ---- 6. Level: pends while high, W1C does not stick ----------------
    axi_write(REG_INT_TYPE, TYPE_LEVEL);  // pin 0 level
    top->gpio_i = 0x1;
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0x1,
             "level pin high pends continuously");
    axi_write(REG_INT_STATUS, 0x1);  // W1C while still high
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0x1,
             "W1C on a high level pin does not stick");
    top->gpio_i = 0x0;
    tick();
    axi_write(REG_INT_STATUS, 0x1);  // now the line is low
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0,
             "W1C sticks once the level drops");

    // ---- 7. INT_EN gates the IRQ --------------------------------------
    axi_write(REG_INT_TYPE, TYPE_RISE);  // pin 0 rise
    axi_write(REG_INT_EN, 0x1);
    top->gpio_i = 0x0;
    tick();
    CHECK_EQ(top->gpio_irq_o, 0, "no IRQ with nothing pending");
    top->gpio_i = 0x1;
    tick();
    CHECK_EQ(top->gpio_irq_o, 1, "IRQ raises on pend with INT_EN set");
    axi_write(REG_INT_STATUS, 0x1);
    CHECK_EQ(top->gpio_irq_o, 0, "IRQ drops when the last pending clears");
    axi_write(REG_INT_EN, 0x0);
    top->gpio_i = 0x0;
    tick();
    top->gpio_i = 0x1;
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS) & 0x1, 0x1,
             "pending still latches with INT_EN=0");
    CHECK_EQ(top->gpio_irq_o, 0, "IRQ masked again when INT_EN clears");
    axi_write(REG_INT_EN, 0x1);
    CHECK_EQ(top->gpio_irq_o, 1, "latched pend raises IRQ on re-enable");
    axi_write(REG_INT_STATUS, 0x1);
    axi_write(REG_INT_EN, 0x0);

    // ---- 8. Strobe discipline ------------------------------------------
    // A write with no byte-0 strobe must touch nothing.
    top->gpio_i = 0x0;
    tick();
    uint32_t out_before = axi_read(REG_OUT);
    axi_write_strb(REG_OUT, 0xF, 0x2);  // strobe byte 1 only
    CHECK_EQ(axi_read(REG_OUT), out_before, "OUT untouched by a no-byte0 write");
    axi_write_strb(REG_INT_STATUS, 0xF, 0x0);  // zero strobe
    CHECK_EQ(axi_read(REG_INT_STATUS), 0, "INT_STATUS untouched by a zero strobe");

    // ---- 9. Per-pin independence --------------------------------------
    // Pin 1 rise, pin 2 level: only the pin whose event fires pends.
    axi_write(REG_INT_TYPE, (TYPE_RISE << 2) | (TYPE_LEVEL << 4));
    top->gpio_i = 0x4;  // pin 2 high (level), pin 1 low
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS), 0x4, "only the level pin pends");
    top->gpio_i = 0x6;  // pin 1 rises too
    tick();
    CHECK_EQ(axi_read(REG_INT_STATUS), 0x6, "rise pin pends independently");
    axi_write(REG_INT_STATUS, 0x2);  // clear only the edge pin
    CHECK_EQ(axi_read(REG_INT_STATUS), 0x4, "edge pin cleared, level pin stays");

    printf("%d checks, %d failures\n", checks, fails);
    delete top;
    return fails ? 1 : 0;
}
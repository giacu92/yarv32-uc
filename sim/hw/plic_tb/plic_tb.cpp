// Verilator C++ BFM for the axi4_lite_plic unit test.
//
// Drives the irq_i line vector directly and checks the cause/claim
// contract: pending/enable, lowest-ID priority, claim-clears-pending,
// and the re-pend of a still-asserted level source (the sharp edge of
// the claim-by-read model -- an ISR that claims without servicing
// livelocks, same contract the bare OR imposed).
//
// Build: make   (in sim/hw/plic_tb/)
// Run:   make run

#include "Vplic_tb.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>

// Register offsets (byte addresses within the peripheral window).
static const uint32_t REG_PENDING = 0x00;
static const uint32_t REG_ENABLE  = 0x04;
static const uint32_t REG_CLAIM   = 0x08;

static Vplic_tb* top;
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
                   __FILE__, __LINE__, msg, (unsigned)(got),             \
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

static void axi_write(uint32_t addr, uint32_t data) {
    top->awaddr = addr;
    top->awvalid = 1;
    top->wdata = data;
    top->wstrb = 0xF;
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
    top = new Vplic_tb;

    top->clk_i = 0;
    top->rstn_i = 0;
    top->irq_i = 0;
    idle();
    for (int i = 0; i < 4; ++i) tick();
    top->rstn_i = 1;
    tick();

    printf("=== axi4_lite_plic pending / enable / claim compliance ===\n");

    // ---- 1. Reset state -----------------------------------------------
    CHECK_EQ(axi_read(REG_PENDING), 0, "PENDING resets to 0");
    CHECK_EQ(axi_read(REG_ENABLE), 0xFFFF, "ENABLE resets to all-ones");
    CHECK_EQ(axi_read(REG_CLAIM), 0, "CLAIM reads 0 with nothing pending");
    CHECK_EQ(top->meip_o, 0, "meip low out of reset");

    // ---- 2. Pending tracks the lines; meip follows ---------------------
    top->irq_i = (1u << 1);  // source 1
    tick();
    CHECK_EQ(axi_read(REG_PENDING), (1u << 1), "pending tracks a set line");
    CHECK_EQ(top->meip_o, 1, "meip = any enabled pending");
    top->irq_i = 0;
    tick();
    CHECK_EQ(axi_read(REG_PENDING), 0, "pending clears when the line drops");

    // ---- 3. Enable masks meip and claimability -------------------------
    top->irq_i = (1u << 1) | (1u << 3);  // sources 1 and 3
    tick();
    axi_write(REG_ENABLE, ~(1u << 3));  // disable source 3 only
    CHECK_EQ(axi_read(REG_ENABLE), ~(1u << 3) & 0xFFFFu, "ENABLE readback");
    CHECK_EQ(axi_read(REG_PENDING), (1u << 1) | (1u << 3),
             "pending records a disabled source");
    CHECK_EQ(top->meip_o, 1, "meip still high: source 1 enabled");
    // Claim while source 1's line is high, as a real ISR would (it has
    // not serviced the source yet). The claim must skip disabled source 3
    // even though its line is high too.
    CHECK_EQ(axi_read(REG_CLAIM), 1, "claim skips the disabled source");
    top->irq_i = (1u << 3);  // service source 1: drop its line
    tick();
    CHECK_EQ(axi_read(REG_PENDING), (1u << 3), "source 1 gone once serviced");
    axi_write(REG_ENABLE, 0);
    CHECK_EQ(top->meip_o, 0, "meip low with everything disabled");
    CHECK_EQ(axi_read(REG_CLAIM), 0, "disabled source is unclaimable");
    CHECK_EQ(axi_read(REG_PENDING), (1u << 3),
             "unclaimable source still shows pending (line high)");
    axi_write(REG_ENABLE, 0xFFFF);

    // ---- 4. Claim of a still-high level source --------------------------
    // The documented sharp edge: source 3's line is still high, so the
    // claim does not clear anything -- pending IS the line. The ISR must
    // service the source before mret or the interrupt re-takes.
    CHECK_EQ(axi_read(REG_CLAIM), 3, "claim returns source 3");
    CHECK_EQ(axi_read(REG_PENDING), (1u << 3), "pending stays (line still high)");
    CHECK_EQ(top->meip_o, 1, "meip stays asserted (line still high)");
    top->irq_i = 0;
    tick();
    CHECK_EQ(axi_read(REG_PENDING), 0, "pending drops once the line drops");
    CHECK_EQ(axi_read(REG_CLAIM), 0, "claim reads 0 once the line drops");

    // ---- 5. Simultaneous sources claim in ID order ----------------------
    top->irq_i = (1u << 1) | (1u << 2) | (1u << 3);
    tick();
    CHECK_EQ(axi_read(REG_PENDING), (1u << 1) | (1u << 2) | (1u << 3),
             "all three pending");
    // Claim-then-service per source, ISR order: the claim runs while the
    // line is high, the line drops after.
    CHECK_EQ(axi_read(REG_CLAIM), 1, "lowest ID (1) claims first");
    top->irq_i = (1u << 2) | (1u << 3);  // service source 1
    tick();
    CHECK_EQ(axi_read(REG_PENDING), (1u << 2) | (1u << 3),
             "source 1 gone once serviced");
    CHECK_EQ(axi_read(REG_CLAIM), 2, "next lowest ID (2) claims");
    top->irq_i = (1u << 3);  // service source 2
    tick();
    CHECK_EQ(axi_read(REG_PENDING), (1u << 3), "source 2 gone once serviced");
    top->irq_i = 0;  // service source 3
    tick();
    CHECK_EQ(axi_read(REG_PENDING), 0, "all drained");
    CHECK_EQ(top->meip_o, 0, "meip low with everything drained");
    CHECK_EQ(axi_read(REG_CLAIM), 0, "claim reads 0 once empty");

    // ---- 6. Source 0 can never claim ------------------------------------
    top->irq_i = 1u;  // bit 0: the caller ties it off in a real system
    tick();
    CHECK_EQ(axi_read(REG_PENDING), 1u, "bit 0 records like any other");
    CHECK_EQ(axi_read(REG_CLAIM), 0, "source 0 never claims");
    CHECK_EQ(axi_read(REG_PENDING), 1u, "unclaimed source 0 pending survives");
    top->irq_i = 0;
    tick();

    printf("%d checks, %d failures\n", checks, fails);
    delete top;
    return fails ? 1 : 0;
}
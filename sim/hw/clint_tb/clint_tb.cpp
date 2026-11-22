// clint (SiFive layout) register/behaviour test, linux-soc M8.
//
// Offsets are the ones the mainline Linux driver hard-codes for
// "sifive,clint0" / "riscv,clint0" -- drivers/clocksource/timer-clint.c:
// CLINT_IPI_OFF 0x0, CLINT_TIMER_CMP_OFF 0x4000, CLINT_TIMER_VAL_OFF 0xbff8
// (from the driver source as known to the author, not re-read against a
// pinned kernel tree here -- M10 pins 6.18 and re-checks). OpenSBI's
// lib/utils/timer/aclint_mtimer.c / ipi/aclint_mswi.c use the same offsets
// for a CLINT-compatible node.
#include "Vclint_tb.h"
#include "axi_bfm.h"
#include "verilated.h"

static const uint32_t MSIP = 0x0000;
static const uint32_t CMP_LO = 0x4000, CMP_HI = 0x4004;
static const uint32_t TIME_LO = 0xBFF8, TIME_HI = 0xBFFC;

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto* top = new Vclint_tb;
    Bfm<Vclint_tb> b(top);
    b.reset();

    // ---- reset values ------------------------------------------------
    CHECK_EQ(b.read(MSIP), 0, "msip resets 0");
    CHECK_EQ(top->msip_o, 0, "msip_o low at reset");
    CHECK_EQ(b.read(CMP_LO), 0xFFFFFFFF, "mtimecmp lo resets all-ones");
    CHECK_EQ(b.read(CMP_HI), 0xFFFFFFFF, "mtimecmp hi resets all-ones");
    CHECK_EQ(top->mtip_o, 0, "no MTIP at reset");

    // ---- mtime counts, every clock -------------------------------------
    uint32_t t0 = b.read(TIME_LO);
    b.ticks(100);
    uint32_t t1 = b.read(TIME_LO);
    CHECK(t1 - t0 >= 100 && t1 - t0 < 120, "mtime advances one per clock");
    CHECK_EQ(b.read(TIME_HI), 0, "mtime hi 0 early");
    CHECK((uint32_t)top->mtime_o - b.read(TIME_LO) < 8 ||
              b.read(TIME_LO) - (uint32_t)top->mtime_o < 8,
          "mtime_o tracks the register");

    // ---- msip -----------------------------------------------------------
    b.write(MSIP, 1);
    CHECK_EQ(b.read(MSIP), 1, "msip set");
    CHECK_EQ(top->msip_o, 1, "msip_o follows");
    b.write(MSIP, 0xFFFFFFFE);
    CHECK_EQ(b.read(MSIP), 0, "msip is bit 0 only");
    CHECK_EQ(top->msip_o, 0, "msip_o clears");

    // ---- mtime is writable (both halves), and keeps counting ------------
    b.write(TIME_HI, 0x12345678);
    b.write(TIME_LO, 0xFFFFFF00);
    uint32_t lo = b.read(TIME_LO);
    CHECK(lo - 0xFFFFFF00u < 16, "mtime lo written");
    CHECK_EQ(b.read(TIME_HI), 0x12345678, "mtime hi written");
    b.ticks(300);  // carries into hi
    CHECK_EQ(b.read(TIME_HI), 0x12345679, "low half carries into high half");
    CHECK_EQ((uint32_t)(top->mtime_o >> 32), 0x12345679, "mtime_o high half");

    // ---- byte strobes ---------------------------------------------------
    b.write(CMP_LO, 0xAABBCCDD);
    b.write(CMP_LO, 0x00000011, 0x1);  // byte 0 only
    CHECK_EQ(b.read(CMP_LO), 0xAABBCC11, "byte-lane write merges");

    // ---- MTIP: mtime >= mtimecmp, two-cycle pipelined compare -----------
    b.write(TIME_HI, 0);
    b.write(TIME_LO, 0);
    b.write(CMP_HI, 0xFFFFFFFF);  // safe arming order, as the firmware does
    b.write(CMP_LO, 400);
    b.write(CMP_HI, 0);
    CHECK_EQ(top->mtip_o, 0, "not yet");
    bool seen = false;
    for (int i = 0; i < 600 && !seen; ++i) {
        b.tick();
        seen = top->mtip_o;
    }
    CHECK(seen, "MTIP asserts once mtime reaches mtimecmp");
    uint64_t at = top->mtime_o;
    CHECK(at >= 400 && at <= 404, "MTIP within the documented 2-cycle lag");
    b.write(CMP_HI, 0xFFFFFFFF);
    b.ticks(4);
    CHECK_EQ(top->mtip_o, 0, "MTIP clears when mtimecmp moves past mtime");

    // high-half compare: mtime_hi > cmp_hi with lo smaller
    b.write(CMP_LO, 0xFFFFFFFF);
    b.write(CMP_HI, 0);
    b.write(TIME_HI, 1);
    b.ticks(4);
    CHECK_EQ(top->mtip_o, 1, "MTIP on the high half alone");
    b.write(CMP_HI, 0xFFFFFFFF);

    // ---- unmapped offsets inside the window read 0, writes harmless -----
    CHECK_EQ(b.read(0x0004), 0, "msip[1] (no hart 1) reads 0");
    CHECK_EQ(b.read(0x4008), 0, "mtimecmp[1] reads 0");
    CHECK_EQ(b.read(0x8000), 0, "hole reads 0");
    b.write(0x4008, 0x5A5A5A5A);
    CHECK_EQ(b.read(CMP_LO), 0xFFFFFFFF, "write to a hole touches nothing");

    int rc = bfm_report("clint_tb");
    delete top;
    return rc;
}

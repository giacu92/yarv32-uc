// Verilator C++ BFM for the native cpu_mem_req_t / cpu_mem_rsp_t RAM compliance
// test. Drives native_mem_tb as two native masters (a RW D-mem and a
// read-only I-mem) and verifies:
//   - RVALID is registered and held until RREADY (the key fix vs a naive
//     one-cycle pulse): the master keeps RREADY low for several cycles
//     after RVALID rises; RVALID must not drop.
//   - byte-strobed writes read back correctly.
//   - back-to-back writes to distinct addresses.
//   - single-outstanding: WREADY is low while an unread read response
//     is held (RVALID=1, RREADY=0) — a new request is not accepted.
//   - posted store: a store commits at the launch handshake (wvalid &&
//     wready && we) and is readable immediately.
//   - read-only: a write to the READ_ONLY I-mem is ignored (the preloaded
//     value survives).
//
// Build: make   (in sim/native_mem_tb/)
// Run:   make run   (or ./obj_dir/Vnative_mem_tb)

#include "Vnative_mem_tb.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>

static vluint64_t sim_time = 0;
static Vnative_mem_tb* top;

static int fails = 0;
static int checks = 0;

#define CHECK(cond, msg)                               \
    do {                                               \
        ++checks;                                      \
        if (!(cond)) {                                \
            printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); \
            ++fails;                                  \
        }                                             \
    } while (0)

// One clock edge: low half, eval; high half, eval. Combinational outputs
// (wready, and the registered rvalid reflecting pre-edge state) settle on
// the low eval; the rising edge updates the RAM's registers.
static void tick() {
    top->clk_i = 0;
    top->eval();
    sim_time++;
    top->clk_i = 1;
    top->eval();
    sim_time++;
}

// Quiesce the D-mem master (rready high so a stray response drains).
static void d_idle() {
    top->d_wvalid = 0;
    top->d_we = 0;
    top->d_rready = 1;
    top->eval();
}
static void i_idle() {
    top->i_wvalid = 0;
    top->i_we = 0;
    top->i_rready = 1;
    top->eval();
}

// Posted store to the D-mem. The write commits at the launch handshake
// (wvalid && wready && we) on the rising edge.
static void d_write(uint32_t addr, uint32_t data, uint8_t strb) {
    top->d_wvalid = 1;
    top->d_we = 1;
    top->d_addr = addr;
    top->d_wdata = data;
    top->d_wstrb = strb;
    top->d_rready = 0;
    int guard = 0;
    while (1) {
        top->eval();
        if (top->d_wvalid && top->d_wready) {  // launch_hs this cycle
            tick();  // rising edge commits the byte-strobed write
            break;
        }
        tick();
        if (++guard > 20) {
            printf("TIMEOUT in d_write @0x%08x\n", addr);
            ++fails;
            return;
        }
    }
    top->d_wvalid = 0;
    top->d_we = 0;
    top->eval();
}

// Read from the D-mem. Returns rdata. Optionally delays RREADY by r_hold
// cycles after RVALID rises to verify RVALID is held (not a 1-cycle pulse).
static uint32_t d_read(uint32_t addr, int r_hold) {
    top->d_wvalid = 1;
    top->d_we = 0;
    top->d_addr = addr;
    top->d_rready = 0;
    int guard = 0;
    while (1) {
        top->eval();
        if (top->d_wvalid && top->d_wready) {  // launch_hs this cycle
            tick();  // rising edge launches the read (rvalid_q<=1)
            break;
        }
        tick();
        if (++guard > 20) {
            printf("TIMEOUT in d_read launch @0x%08x\n", addr);
            ++fails;
            return 0xDEAD;
        }
    }
    top->d_wvalid = 0;
    top->eval();
    // RVALID should be high the cycle after launch.
    guard = 0;
    while (!top->d_rvalid) {
        tick();
        top->eval();
        if (++guard > 20) {
            printf("TIMEOUT waiting for d_rvalid @0x%08x\n", addr);
            ++fails;
            return 0xDEAD;
        }
    }
    // Hold RREADY low; RVALID must stay asserted (the compliance point).
    for (int i = 0; i < r_hold; ++i) {
        top->d_rready = 0;
        top->eval();
        CHECK(top->d_rvalid, "D RVALID dropped while RREADY=0 (must be held)");
        tick();
        top->eval();
    }
    // Accept the response.
    top->d_rready = 1;
    top->eval();
    CHECK(top->d_rvalid, "D RVALID dropped before R handshake");
    uint32_t rd = top->d_rdata;
    tick();  // r_hs edge
    top->d_rready = 0;
    top->eval();
    CHECK(!top->d_rvalid, "D RVALID stayed high after R handshake");
    d_idle();
    return rd;
}

// Read from the read-only I-mem (same flow as d_read; used to verify RO
// preload survives a write and that RVALID holds on the RO path too).
static uint32_t i_read(uint32_t addr, int r_hold) {
    top->i_wvalid = 1;
    top->i_we = 0;
    top->i_addr = addr;
    top->i_rready = 0;
    int guard = 0;
    while (1) {
        top->eval();
        if (top->i_wvalid && top->i_wready) {
            tick();
            break;
        }
        tick();
        if (++guard > 20) {
            printf("TIMEOUT in i_read launch @0x%08x\n", addr);
            ++fails;
            return 0xDEAD;
        }
    }
    top->i_wvalid = 0;
    top->eval();
    guard = 0;
    while (!top->i_rvalid) {
        tick();
        top->eval();
        if (++guard > 20) {
            printf("TIMEOUT waiting for i_rvalid @0x%08x\n", addr);
            ++fails;
            return 0xDEAD;
        }
    }
    for (int i = 0; i < r_hold; ++i) {
        top->i_rready = 0;
        top->eval();
        CHECK(top->i_rvalid, "I RVALID dropped while RREADY=0 (must be held)");
        tick();
        top->eval();
    }
    top->i_rready = 1;
    top->eval();
    CHECK(top->i_rvalid, "I RVALID dropped before R handshake");
    uint32_t rd = top->i_rdata;
    tick();
    top->i_rready = 0;
    top->eval();
    CHECK(!top->i_rvalid, "I RVALID stayed high after R handshake");
    i_idle();
    return rd;
}

// Attempt a write to the read-only I-mem (should be ignored).
static void i_write_attempt(uint32_t addr, uint32_t data, uint8_t strb) {
    top->i_wvalid = 1;
    top->i_we = 1;
    top->i_addr = addr;
    top->i_wdata = data;
    top->i_wstrb = strb;
    top->i_rready = 0;
    int guard = 0;
    while (1) {
        top->eval();
        if (top->i_wvalid && top->i_wready) {
            tick();
            break;
        }
        tick();
        if (++guard > 20) {
            printf("TIMEOUT in i_write_attempt @0x%08x\n", addr);
            ++fails;
            return;
        }
    }
    top->i_wvalid = 0;
    top->i_we = 0;
    top->eval();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vnative_mem_tb;

    // Reset (synchronous): hold rstn=0 a few cycles, then release.
    top->clk_i = 0;
    top->rstn_i = 0;
    d_idle();
    i_idle();
    top->eval();
    for (int i = 0; i < 4; ++i) tick();
    top->rstn_i = 1;
    top->eval();
    tick();

    printf("=== native RAM (D-mem RW) compliance test ===\n");

    // --- 1. Posted store + readback, with delayed RREADY (RVALID held) ---
    d_write(0x0000, 0xDEADBEEF, 0xF);
    uint32_t r = d_read(0x0000, /*r_hold=*/3);
    CHECK(r == 0xDEADBEEF, "posted store + readback (delayed RREADY)");
    printf("posted store + readback: 0x%08x\n", r);

    // --- 2. Back-to-back writes to distinct addresses ---
    d_write(0x0010, 0x11111111, 0xF);
    d_write(0x0014, 0x22222222, 0xF);
    CHECK(d_read(0x0010, 0) == 0x11111111, "back-to-back addr 0x10");
    CHECK(d_read(0x0014, 0) == 0x22222222, "back-to-back addr 0x14");
    printf("back-to-back writes: ok\n");

    // --- 3. Byte strobes: write only the low 2 bytes of a known word ---
    d_write(0x0020, 0xFFFFFFFF, 0xF);
    d_write(0x0020, 0x00000000, 0x3);  // strb=0b0011 -> low 2 bytes
    r = d_read(0x0020, 0);
    CHECK(r == 0xFFFF0000, "byte strobe partial write (low 2 bytes)");
    printf("byte-strobe low-2 + readback: 0x%08x (exp 0xFFFF0000)\n", r);

    // --- 4. Byte strobes: high byte only ---
    d_write(0x0024, 0x00000000, 0xF);
    d_write(0x0024, 0xAB000000, 0x8);  // strb=0b1000 -> byte 3
    r = d_read(0x0024, 0);
    CHECK(r == 0xAB000000, "byte strobe partial write (high byte)");
    printf("byte-strobe high-byte + readback: 0x%08x (exp 0xAB000000)\n", r);

    // --- 5. Single-outstanding: while a read response is held
    //     (RVALID=1, RREADY=0), WREADY must be low (no new accept). ---
    // Launch a read, hold RREADY low, then assert a new write and check
    // it is NOT accepted (wready=0).
    top->d_wvalid = 1;
    top->d_we = 0;
    top->d_addr = 0x0010;
    top->d_rready = 0;
    int guard = 0;
    while (1) {
        top->eval();
        if (top->d_wvalid && top->d_wready) {
            tick();  // launch the read
            break;
        }
        tick();
        if (++guard > 20) { printf("TIMEOUT launching read for single-outstanding\n"); ++fails; break; }
    }
    top->d_wvalid = 0;
    top->eval();
    // Wait for RVALID.
    guard = 0;
    while (!top->d_rvalid) { tick(); top->eval(); if (++guard > 20) break; }
    CHECK(top->d_rvalid, "RVALID up for single-outstanding check");
    // Now try to launch a NEW write while the read response is held.
    top->d_wvalid = 1;
    top->d_we = 1;
    top->d_addr = 0x0028;
    top->d_wdata = 0x33333333;
    top->d_wstrb = 0xF;
    top->d_rready = 0;  // do not drain the pending read
    top->eval();
    CHECK(top->d_wready == 0, "WREADY high while read response pending (single outstanding)");
    tick();
    top->eval();
    CHECK(top->d_wready == 0, "WREADY high 2nd cycle while read pending");
    // Drain the pending read, then the write can go.
    top->d_wvalid = 0;
    top->d_rready = 1;
    top->eval();
    tick();  // r_hs
    top->d_rready = 0;
    top->eval();
    d_idle();
    // The earlier write attempt must NOT have committed (wready was 0).
    CHECK(d_read(0x0028, 0) != 0x33333333, "write committed while WREADY=0 (must not)");
    printf("single-outstanding (WREADY low while read pending): ok\n");

    printf("\n=== native RAM (I-mem read-only) compliance test ===\n");

    // --- 6. RO read of preloaded words, RVALID held ---
    r = i_read(0x0000, /*r_hold=*/2);
    CHECK(r == 0xCAFEBABE, "RO read word 0 (preloaded)");
    printf("RO read word0: 0x%08x (exp 0xCAFEBABE)\n", r);
    r = i_read(0x0004, /*r_hold=*/2);
    CHECK(r == 0xDEADBEEF, "RO read word 1 (preloaded)");
    printf("RO read word1: 0x%08x (exp 0xDEADBEEF)\n", r);

    // --- 7. RO ignores a write: preload survives ---
    i_write_attempt(0x0000, 0x0BADF00D, 0xF);
    r = i_read(0x0000, 0);
    CHECK(r == 0xCAFEBABE, "RO write ignored (preload survives)");
    printf("RO write-ignored readback: 0x%08x (exp 0xCAFEBABE)\n", r);

    printf("\n=== results: %d checks, %d failures ===\n", checks, fails);

    delete top;
    return fails ? 1 : 0;
}
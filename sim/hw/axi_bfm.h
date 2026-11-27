// Minimal AXI4-Lite master BFM + check counters, shared by the linux-soc
// (M8) peripheral TBs: plic_sifive_tb, clint_tb, uart16550_tb. Same
// handshake discipline as the older per-TB copies (plic_tb.cpp & co): one
// access at a time, AW and W presented together, B / R accepted at once.
//
// The TB top must expose flat AXI ports named awaddr/awvalid/wdata/wstrb/
// wvalid/bready/araddr/arvalid/rready (in) and awready/wready/bvalid/
// arready/rdata/rvalid (out), plus clk_i / rstn_i.
#pragma once
#include <cstdint>
#include <cstdio>

static int g_fails = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        ++g_checks;                                                       \
        if (!(cond)) {                                                    \
            printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg);        \
            ++g_fails;                                                    \
        }                                                                 \
    } while (0)

#define CHECK_EQ(got, want, msg)                                          \
    do {                                                                  \
        ++g_checks;                                                       \
        uint32_t g_ = (uint32_t)(got), w_ = (uint32_t)(want);             \
        if (g_ != w_) {                                                   \
            printf("FAIL [%s:%d]: %s (got 0x%x, want 0x%x)\n", __FILE__,  \
                   __LINE__, msg, (unsigned)g_, (unsigned)w_);            \
            ++g_fails;                                                    \
        }                                                                 \
    } while (0)

template <class T> struct Bfm {
    T* top;
    uint64_t cycles = 0;
    void (*on_tick)(T*) = nullptr;  // per-cycle hook (e.g. a line model)

    explicit Bfm(T* t) : top(t) {}

    void tick() {
        if (on_tick) on_tick(top);
        top->clk_i = 0;
        top->eval();
        top->clk_i = 1;
        top->eval();
        ++cycles;
    }
    void ticks(int n) {
        for (int i = 0; i < n; ++i) tick();
    }
    void idle() {
        top->awvalid = 0;
        top->wvalid = 0;
        top->arvalid = 0;
        top->bready = 1;
        top->rready = 1;
        top->eval();
    }
    void reset() {
        top->clk_i = 0;
        top->rstn_i = 0;
        idle();
        ticks(4);
        top->rstn_i = 1;
        tick();
    }
    void write(uint32_t addr, uint32_t data, uint32_t strb = 0xF) {
        top->awaddr = addr;
        top->awvalid = 1;
        top->wdata = data;
        top->wstrb = strb;
        top->wvalid = 1;
        top->bready = 1;
        bool aw = false, w = false, b = false;
        for (int g = 0; g < 4096 && !(aw && w && b); ++g) {
            top->eval();
            if (top->awvalid && top->awready) aw = true;
            if (top->wvalid && top->wready) w = true;
            if (top->bvalid && top->bready) b = true;
            tick();
            if (aw) top->awvalid = 0;
            if (w) top->wvalid = 0;
        }
        CHECK(aw && w && b, "axi write did not complete");
        idle();
    }
    uint32_t read(uint32_t addr) {
        top->araddr = addr;
        top->arvalid = 1;
        top->rready = 1;
        uint32_t v = 0;
        bool ar = false, r = false;
        for (int g = 0; g < 4096 && !(ar && r); ++g) {
            top->eval();
            if (top->arvalid && top->arready) ar = true;
            if (top->rvalid && top->rready) {
                v = top->rdata;
                r = true;
            }
            tick();
            if (ar) top->arvalid = 0;
        }
        CHECK(ar && r, "axi read did not complete");
        idle();
        return v;
    }
};

static int bfm_report(const char* name) {
    printf("%s: %d checks, %d failed -> %s\n", name, g_checks, g_fails,
           g_fails ? "FAIL" : "PASS");
    return g_fails ? 1 : 0;
}

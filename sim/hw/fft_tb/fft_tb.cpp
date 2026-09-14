// Verilator C++ BFM for the axi4_lite_fft unit test.
//
// Two independent references check every transform, and they catch
// different classes of bug:
//
//   * A BIT-EXACT MODEL of the datapath (model_fft below) -- same Q15
//     twiddle table, same round-half-up points, same saturation, same
//     stage/copy/bit-reverse structure. The RTL must match it to the
//     digit. This catches a wrong register, a dropped rounding term, an
//     off-by-one address.
//
//   * A DOUBLE-PRECISION DFT (ref_dft) with a tolerance. This catches
//     what the bit-exact model cannot: a model that was written from the
//     same wrong understanding as the RTL. A wrong twiddle exponent or a
//     wrong butterfly pairing moves bins by thousands of LSB; fixed-point
//     rounding moves them by a handful.
//
// That pairing is the method the divider bug was found with (see the
// CLAUDE.md note on brute-force arithmetic verification): a model alone
// proves consistency, not correctness.
//
// Build: make   (in sim/hw/fft_tb/)
// Run:   make run

#include "Vfft_tb.h"
#include "verilated.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

// ---------------------------------------------------------------- map
static const uint32_t CSR_BASE = 0x0000;
static const uint32_t DATA_BASE = 0x1000;

static const uint32_t REG_CTRL = CSR_BASE + 0x00;
static const uint32_t REG_STATUS = CSR_BASE + 0x04;
static const uint32_t REG_IRQ = CSR_BASE + 0x08;
static const uint32_t REG_CONF = CSR_BASE + 0x0C;
static const uint32_t REG_SCALE = CSR_BASE + 0x10;
static const uint32_t REG_CYCLES = CSR_BASE + 0x14;

static const uint32_t CTRL_START = 1u << 0;
static const uint32_t CTRL_IRQ_EN = 1u << 1;
static const uint32_t CTRL_INVERSE = 1u << 2;
static const uint32_t CTRL_SWAP = 1u << 3;

static const uint32_t ST_BUSY = 1u << 0;
static const uint32_t ST_DONE = 1u << 1;
static const uint32_t ST_CPU_BUF = 1u << 2;
static const uint32_t ST_IRQ = 1u << 3;

static const int NMAX = 1024;
static const int LOG2NMAX = 10;

static Vfft_tb* top;
static int fails = 0;
static int checks = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        ++checks;                                                     \
        if (!(cond)) {                                                \
            printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg);    \
            ++fails;                                                  \
        }                                                             \
    } while (0)

#define CHECK_EQ(got, want, msg)                                      \
    do {                                                              \
        ++checks;                                                     \
        if ((uint32_t)(got) != (uint32_t)(want)) {                    \
            printf("FAIL [%s:%d]: %s (got 0x%x, want 0x%x)\n",        \
                   __FILE__, __LINE__, msg, (unsigned)(got),          \
                   (unsigned)(want));                                 \
            ++fails;                                                  \
        }                                                             \
    } while (0)

// ------------------------------------------------------------- AXI BFM
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

// ------------------------------------------------- sample helpers
struct C16 {
    int16_t re;
    int16_t im;
};

static uint32_t pack(C16 c) {
    return ((uint32_t)(uint16_t)c.im << 16) | (uint16_t)c.re;
}

static C16 unpack(uint32_t w) {
    C16 c;
    c.re = (int16_t)(w & 0xFFFF);
    c.im = (int16_t)(w >> 16);
    return c;
}

static void write_frame(const std::vector<C16>& x) {
    for (size_t i = 0; i < x.size(); ++i) {
        axi_write(DATA_BASE + 4 * (uint32_t)i, pack(x[i]));
    }
}

static void read_frame(std::vector<C16>& y, int n) {
    y.resize(n);
    for (int i = 0; i < n; ++i) y[i] = unpack(axi_read(DATA_BASE + 4 * i));
}

// ------------------------------------------------- bit-exact model
//
// Mirrors axi4_lite_fft.sv exactly: same Q15 table (the generator's
// floor(x*32768+0.5) with clamping), same +16384 before the >>15, same
// +1 before each scaling >>1, same saturation, same DIF pairing, same
// bit-reversed final write.

static int16_t tw_re_tab[NMAX / 2];
static int16_t tw_im_tab[NMAX / 2];

static int16_t q15(double x) {
    long v = (long)std::floor(x * 32768.0 + 0.5);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

static void build_twiddle() {
    for (int k = 0; k < NMAX / 2; ++k) {
        double ang = -2.0 * M_PI * k / NMAX;
        tw_re_tab[k] = q15(std::cos(ang));
        tw_im_tab[k] = q15(std::sin(ang));
    }
}

static int16_t sat16(long v) {
    if (v > 32767) return (int16_t)32767;
    if (v < -32768) return (int16_t)-32768;
    return (int16_t)v;
}

static long scale_rnd(long v, bool en) { return en ? ((v + 1) >> 1) : v; }

static int brev(int x, int n) {
    int r = 0;
    for (int i = 0; i < n; ++i) {
        if (x & (1 << i)) r |= 1 << (n - 1 - i);
    }
    return r;
}

static void model_fft(std::vector<C16>& buf, int log2n, uint32_t scale, bool inverse) {
    const int n = 1 << log2n;
    std::vector<C16> scr(n);
    const int total = log2n + (log2n & 1);

    for (int st = 0; st < total; ++st) {
        const bool from_scr = (st & 1) != 0;
        std::vector<C16>& src = from_scr ? scr : buf;
        std::vector<C16>& dst = from_scr ? buf : scr;

        const bool copy = (st >= log2n);
        const int half = copy ? 1 : (n >> (st + 1));
        const int step = copy ? 0 : ((1 << st) * (NMAX / n));
        const bool sc = copy ? false : (((scale >> st) & 1u) != 0);
        const bool last = (st == total - 1);

        for (int bf = 0; bf < n / 2; ++bf) {
            const int low = half - 1;
            const int j = bf & low;
            const int hi = bf & ~low;
            const int ia = (hi << 1) | j;
            const int ib = ia | half;

            const C16 A = src[ia];
            const C16 B = src[ib];

            int16_t wr = tw_re_tab[j * step];
            int16_t wi = tw_im_tab[j * step];
            if (inverse) wi = (wi == (int16_t)-32768) ? (int16_t)32767 : (int16_t)(-wi);

            C16 oa, ob;
            if (copy) {
                oa = A;
                ob = B;
            } else {
                const long sum_r = (long)A.re + B.re;
                const long sum_i = (long)A.im + B.im;
                const long dif_r = (long)A.re - B.re;
                const long dif_i = (long)A.im - B.im;

                const long mul_r = dif_r * wr - dif_i * wi;
                const long mul_i = dif_r * wi + dif_i * wr;
                const long norm_r = (mul_r + 16384) >> 15;
                const long norm_i = (mul_i + 16384) >> 15;

                oa.re = sat16(scale_rnd(sum_r, sc));
                oa.im = sat16(scale_rnd(sum_i, sc));
                ob.re = sat16(scale_rnd(norm_r, sc));
                ob.im = sat16(scale_rnd(norm_i, sc));
            }

            const int wa = last ? brev(ia, log2n) : ia;
            const int wb = last ? brev(ib, log2n) : ib;
            dst[wa] = oa;
            dst[wb] = ob;
        }
    }
    // After `total` passes the result is always back in buf: with log2n
    // even the last stage index is odd, and the copy pass added for odd
    // log2n makes it odd again.
}

// ---------------------------------------------- double-precision DFT
//
// Deliberately a direct O(N^2) DFT, not another FFT: an independent
// formulation, so a wrong butterfly structure cannot be reproduced here
// by accident.
static void ref_dft(const std::vector<C16>& x, std::vector<double>& re,
                    std::vector<double>& im, int n, bool inverse) {
    re.assign(n, 0.0);
    im.assign(n, 0.0);
    const double sgn = inverse ? 1.0 : -1.0;
    for (int k = 0; k < n; ++k) {
        double sr = 0.0, si = 0.0;
        for (int t = 0; t < n; ++t) {
            const double ang = sgn * 2.0 * M_PI * k * t / n;
            const double c = std::cos(ang), s = std::sin(ang);
            const double xr = x[t].re / 32768.0, xi = x[t].im / 32768.0;
            sr += xr * c - xi * s;
            si += xr * s + xi * c;
        }
        // The engine's default SCALE divides by 2 every stage, i.e. 1/N.
        re[k] = sr / n;
        im[k] = si / n;
    }
}

// --------------------------------------------------------- run helper
static void set_conf(int log2n) { axi_write(REG_CONF, (uint32_t)log2n); }

static void start_and_wait(int max_cycles) {
    axi_write(REG_CTRL, axi_read(REG_CTRL) | CTRL_START);
    int c = 0;
    while (c < max_cycles) {
        if (axi_read(REG_STATUS) & ST_DONE) break;
        ++c;
    }
    CHECK(c < max_cycles, "transform finished before the cycle bound");
}

// Runs one transform end to end and checks it against both references.
// Returns the largest absolute deviation from the double DFT, in LSB.
static double run_case(const char* name, std::vector<C16> x, int log2n, uint32_t scale,
                       bool inverse, bool check_float) {
    const int n = 1 << log2n;

    set_conf(log2n);
    axi_write(REG_SCALE, scale);
    axi_write(REG_CTRL, inverse ? CTRL_INVERSE : 0);
    write_frame(x);
    start_and_wait(200000);

    // START hands the just-computed buffer back to the CPU; the results
    // of THIS transform become visible on the next swap.
    axi_write(REG_CTRL, (inverse ? CTRL_INVERSE : 0) | CTRL_SWAP);

    std::vector<C16> got;
    read_frame(got, n);

    std::vector<C16> want = x;
    model_fft(want, log2n, scale, inverse);

    int bad = 0;
    for (int i = 0; i < n; ++i) {
        if (got[i].re != want[i].re || got[i].im != want[i].im) {
            if (bad < 4) {
                printf("FAIL: %s bin %d: got (%d,%d) want (%d,%d)\n", name, i, got[i].re,
                       got[i].im, want[i].re, want[i].im);
            }
            ++bad;
        }
    }
    ++checks;
    if (bad) {
        printf("FAIL: %s -- %d/%d bins differ from the bit-exact model\n", name, bad, n);
        ++fails;
    }

    double worst = 0.0;
    if (check_float) {
        std::vector<double> rr, ri;
        ref_dft(x, rr, ri, n, inverse);
        for (int i = 0; i < n; ++i) {
            const double er = std::fabs(got[i].re / 32768.0 - rr[i]) * 32768.0;
            const double ei = std::fabs(got[i].im / 32768.0 - ri[i]) * 32768.0;
            if (er > worst) worst = er;
            if (ei > worst) worst = ei;
        }
        // Rounding is round-half-up at two points per stage plus the Q15
        // twiddle quantisation; a structural error is orders of magnitude
        // above this.
        const double tol = 4.0 + 2.0 * log2n;
        ++checks;
        if (worst > tol) {
            printf("FAIL: %s -- worst deviation from the double DFT %.1f LSB > %.1f\n", name,
                   worst, tol);
            ++fails;
        }
    }
    return worst;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build_twiddle();
    top = new Vfft_tb;

    top->clk_i = 0;
    top->rstn_i = 0;
    idle();
    for (int i = 0; i < 4; ++i) tick();
    top->rstn_i = 1;
    tick();

    printf("=== axi4_lite_fft register / ping-pong / transform compliance ===\n");

    // ---- 1. Reset state ------------------------------------------------
    CHECK_EQ(axi_read(REG_CONF), LOG2NMAX, "CONF resets to LOG2NMAX");
    CHECK_EQ(axi_read(REG_SCALE), (1u << LOG2NMAX) - 1, "SCALE resets to all-ones (1/N mode)");
    CHECK_EQ(axi_read(REG_STATUS), 0, "STATUS resets idle, not done, buffer 0");
    CHECK_EQ(axi_read(REG_IRQ), 0, "IRQ resets clear");
    CHECK_EQ(axi_read(REG_CTRL), 0, "CTRL resets clear");
    CHECK_EQ(top->fft_irq_o, 0, "IRQ line low out of reset");

    // ---- 2. CSR contract -----------------------------------------------
    axi_write(REG_CTRL, CTRL_IRQ_EN | CTRL_INVERSE);
    CHECK_EQ(axi_read(REG_CTRL), CTRL_IRQ_EN | CTRL_INVERSE, "CTRL sticky bits read back");
    CHECK_EQ(axi_read(REG_CTRL) & CTRL_START, 0, "START does not read back (self-clearing)");
    axi_write(REG_CTRL, 0);

    // CONF is clamped ON WRITE so a readback reports what is in force.
    axi_write(REG_CONF, 0);
    CHECK_EQ(axi_read(REG_CONF), 2, "CONF clamps up to the 4-point minimum");
    axi_write(REG_CONF, 15);
    CHECK_EQ(axi_read(REG_CONF), LOG2NMAX, "CONF clamps down to LOG2NMAX");

    // SCALE merges by byte strobe, like every other register in the tree.
    axi_write(REG_SCALE, 0x000);
    axi_write_strb(REG_SCALE, 0x3FF, 0x1);
    CHECK_EQ(axi_read(REG_SCALE), 0x0FF, "SCALE byte-0 store touches only byte 0");
    axi_write(REG_SCALE, (1u << LOG2NMAX) - 1);

    // ---- 3. DATA page + ping-pong ownership ----------------------------
    axi_write(DATA_BASE + 0, 0x11112222);
    axi_write(DATA_BASE + 4, 0x33334444);
    CHECK_EQ(axi_read(DATA_BASE + 0), 0x11112222, "DATA word 0 reads back");
    CHECK_EQ(axi_read(DATA_BASE + 4), 0x33334444, "DATA word 1 reads back");
    axi_write(DATA_BASE + 4 * (NMAX - 1), 0xDEADBEEF);
    CHECK_EQ(axi_read(DATA_BASE + 4 * (NMAX - 1)), 0xDEADBEEF, "DATA top word reads back");

    // Write granularity is HALFWORD, not byte: a Gowin DPB has no
    // per-byte write enable, so fft_dpram honours a strobe pair per
    // 16-bit slice (see its header -- a byte-granular write is what sent
    // the whole array into flip-flops). sh writes one half of a complex
    // sample, sb writes nothing and is still answered OKAY.
    axi_write(DATA_BASE + 8, 0xAAAABBBB);
    axi_write_strb(DATA_BASE + 8, 0x1111CCCC, 0x3);  // low half only
    CHECK_EQ(axi_read(DATA_BASE + 8), 0xAAAACCCC, "strobe 0x3 writes the real half only");
    axi_write_strb(DATA_BASE + 8, 0x2222DDDD, 0xC);  // high half only
    CHECK_EQ(axi_read(DATA_BASE + 8), 0x2222CCCC, "strobe 0xC writes the imaginary half only");
    axi_write_strb(DATA_BASE + 8, 0xFFFFFFFF, 0x1);  // single byte
    CHECK_EQ(axi_read(DATA_BASE + 8), 0x2222CCCC, "a single-byte strobe writes nothing");
    axi_write_strb(DATA_BASE + 8, 0xFFFFFFFF, 0x4);
    CHECK_EQ(axi_read(DATA_BASE + 8), 0x2222CCCC, "a single-byte strobe in the high half too");
    axi_write_strb(DATA_BASE + 8, 0xFFFFFFFF, 0x0);  // zero strobe
    CHECK_EQ(axi_read(DATA_BASE + 8), 0x2222CCCC, "a zero strobe writes nothing");

    // SWAP hands over the other buffer without starting anything.
    CHECK_EQ(axi_read(REG_STATUS) & ST_CPU_BUF, 0, "CPU owns buffer 0 initially");
    axi_write(REG_CTRL, CTRL_SWAP);
    CHECK(axi_read(REG_STATUS) & ST_CPU_BUF, "SWAP flips buffer ownership");
    CHECK_EQ(axi_read(REG_STATUS) & ST_BUSY, 0, "SWAP does not start a transform");
    axi_write(DATA_BASE + 0, 0x55556666);
    CHECK_EQ(axi_read(DATA_BASE + 0), 0x55556666, "the other buffer is a separate memory");
    axi_write(REG_CTRL, CTRL_SWAP);
    CHECK_EQ(axi_read(DATA_BASE + 0), 0x11112222, "swapping back restores the first buffer");
    axi_write(REG_CTRL, CTRL_SWAP);  // leave ownership where the run cases expect

    // ---- 4. Impulse, N=8 -- odd LOG2N, so the copy pass runs -----------
    // x[0] = 0.5, rest 0. With 1/N scaling every bin is 0.5/8 = 0x0800,
    // and every intermediate stays a power of two, so this is exact with
    // no rounding anywhere -- a hand-checkable anchor for the model.
    {
        std::vector<C16> x(8, C16{0, 0});
        x[0] = C16{0x4000, 0};
        run_case("impulse N=8", x, 3, 0x3FF, false, true);

        set_conf(3);
        axi_write(REG_SCALE, 0x3FF);
        axi_write(REG_CTRL, 0);
        write_frame(x);
        start_and_wait(20000);
        axi_write(REG_CTRL, CTRL_SWAP);
        std::vector<C16> y;
        read_frame(y, 8);
        bool flat = true;
        for (int i = 0; i < 8; ++i) {
            if (y[i].re != 0x0800 || y[i].im != 0) flat = false;
        }
        CHECK(flat, "impulse N=8 gives a flat 0x0800 spectrum");
    }

    // ---- 5. DC, N=16 ---------------------------------------------------
    // A constant input puts all the energy in bin 0 and exactly nothing
    // anywhere else. Catches a wrong twiddle sign or exponent instantly.
    {
        std::vector<C16> x(16, C16{0x0100, 0});
        set_conf(4);
        axi_write(REG_SCALE, 0x3FF);
        axi_write(REG_CTRL, 0);
        write_frame(x);
        start_and_wait(20000);
        axi_write(REG_CTRL, CTRL_SWAP);
        std::vector<C16> y;
        read_frame(y, 16);
        CHECK_EQ((uint16_t)y[0].re, 0x0100, "DC N=16 lands entirely in bin 0");
        bool clean = (y[0].im == 0);
        for (int i = 1; i < 16; ++i) {
            if (y[i].re != 0 || y[i].im != 0) clean = false;
        }
        CHECK(clean, "DC N=16 leaves every other bin at zero");
    }

    // ---- 6. Single tone, N=64 ------------------------------------------
    // A real cosine at bin 5 must show up at bins 5 and 59 and nowhere
    // else. This is the test a wrong bit-reversal fails.
    {
        const int n = 64;
        std::vector<C16> x(n);
        for (int i = 0; i < n; ++i) {
            x[i] = C16{q15(0.5 * std::cos(2.0 * M_PI * 5.0 * i / n)), 0};
        }
        const double worst = run_case("tone N=64 bin5", x, 6, 0x3FF, false, true);
        printf("  tone N=64: worst deviation from the double DFT %.1f LSB\n", worst);

        set_conf(6);
        axi_write(REG_SCALE, 0x3FF);
        axi_write(REG_CTRL, 0);
        write_frame(x);
        start_and_wait(20000);
        axi_write(REG_CTRL, CTRL_SWAP);
        std::vector<C16> y;
        read_frame(y, n);
        const int peak = std::abs(y[5].re) + std::abs(y[5].im);
        int floor_max = 0;
        for (int i = 0; i < n; ++i) {
            if (i == 5 || i == 59) continue;
            const int m = std::abs(y[i].re) + std::abs(y[i].im);
            if (m > floor_max) floor_max = m;
        }
        CHECK(peak > 200, "tone N=64 has a real peak at bin 5");
        CHECK(floor_max * 20 < peak, "tone N=64 leaks less than 5% into other bins");
    }

    // ---- 7. Random vectors at four sizes -------------------------------
    // 4 and 8 are the smallest legal transforms; 32 is odd-LOG2N (copy
    // pass); 1024 is the shipping geometry.
    {
        unsigned seed = 12345;
        auto rnd16 = [&seed]() {
            seed = seed * 1103515245u + 12345u;
            return (int16_t)((int32_t)((seed >> 12) & 0xFFFF) - 32768);
        };
        const int sizes[] = {2, 3, 5, 6, 10};
        for (int si = 0; si < 5; ++si) {
            const int log2n = sizes[si];
            const int n = 1 << log2n;
            std::vector<C16> x(n);
            for (int i = 0; i < n; ++i) x[i] = C16{rnd16(), rnd16()};
            char name[64];
            snprintf(name, sizeof(name), "random N=%d", n);
            // The double check is skipped at N=1024 only because the O(N^2)
            // reference would dominate the test's runtime; the bit-exact
            // model still runs, and N=64 already pins the structure.
            const double worst = run_case(name, x, log2n, 0x3FF, false, log2n <= 6);
            if (log2n <= 6) printf("  %s: worst deviation %.1f LSB\n", name, worst);
        }
    }

    // ---- 8. SCALE = 0: no per-stage divide -----------------------------
    // Small input, so nothing saturates; checks that the scaling shift is
    // a real per-stage control and not baked in.
    {
        const int n = 16;
        std::vector<C16> x(n);
        for (int i = 0; i < n; ++i) x[i] = C16{(int16_t)(100 + 7 * i), (int16_t)(-50 + 3 * i)};
        run_case("unscaled N=16", x, 4, 0x000, false, false);
    }

    // ---- 9. Saturation, not wraparound ---------------------------------
    // Full-scale input with the scaling off must clip, and clip exactly
    // where the model clips. A wrapping adder would invert signs instead,
    // which the bit-exact comparison catches as a sign flip.
    {
        const int n = 16;
        std::vector<C16> x(n, C16{0x7FFF, 0x7FFF});
        run_case("saturating N=16", x, 4, 0x000, false, false);
    }

    // ---- 10. Inverse transform -----------------------------------------
    {
        const int n = 64;
        std::vector<C16> x(n);
        for (int i = 0; i < n; ++i) {
            x[i] = C16{q15(0.4 * std::cos(2.0 * M_PI * 3.0 * i / n)),
                       q15(0.4 * std::sin(2.0 * M_PI * 3.0 * i / n))};
        }
        const double worst = run_case("inverse N=64", x, 6, 0x3FF, true, true);
        printf("  inverse N=64: worst deviation %.1f LSB\n", worst);
    }

    // ---- 11. CYCLES counter --------------------------------------------
    // (LOG2N + odd) passes of (N/2 + PIPE) cycles, plus the START and
    // DONE edges. Checked as a band, not a constant: the point is that it
    // reports the transform, not the polling loop around it.
    {
        set_conf(6);
        axi_write(REG_SCALE, 0x3FF);
        axi_write(REG_CTRL, 0);
        start_and_wait(20000);
        const uint32_t cyc = axi_read(REG_CYCLES);
        const uint32_t lo = 6 * 32;
        const uint32_t hi = 6 * (32 + 8) + 16;
        ++checks;
        if (cyc < lo || cyc > hi) {
            printf("FAIL: CYCLES=%u outside the expected [%u,%u] for N=64\n", cyc, lo, hi);
            ++fails;
        } else {
            printf("  N=64 transform took %u cycles (bound [%u,%u])\n", cyc, lo, hi);
        }
        axi_write(REG_CTRL, CTRL_SWAP);
    }

    // ---- 12. Interrupt + DONE stickiness -------------------------------
    {
        axi_write(REG_IRQ, 1);  // clear any leftover
        CHECK_EQ(axi_read(REG_STATUS) & ST_DONE, 0, "DONE clears on W1C");
        CHECK_EQ(top->fft_irq_o, 0, "IRQ line low with DONE clear");

        set_conf(4);
        axi_write(REG_CTRL, CTRL_IRQ_EN);
        start_and_wait(20000);
        CHECK(axi_read(REG_STATUS) & ST_DONE, "DONE sets at the end of a transform");
        CHECK(axi_read(REG_STATUS) & ST_IRQ, "STATUS mirrors the IRQ line");
        CHECK(top->fft_irq_o, "IRQ line rises with DONE & IRQ_EN");

        axi_write(REG_CTRL, 0);  // drop IRQ_EN
        CHECK_EQ(top->fft_irq_o, 0, "IRQ line drops when IRQ_EN clears");
        CHECK(axi_read(REG_STATUS) & ST_DONE, "DONE survives IRQ_EN clearing");
        axi_write(REG_IRQ, 1);
        CHECK_EQ(axi_read(REG_STATUS) & ST_DONE, 0, "DONE clears on W1C");
    }

    // ---- 13. START ignored while BUSY ----------------------------------
    // The engine must not restart mid-flight: a second START would swap
    // ownership under a running transform and hand the CPU the buffer the
    // engine is writing.
    {
        set_conf(10);  // long enough to still be running a few reads later
        axi_write(REG_SCALE, 0x3FF);
        axi_write(REG_CTRL, 0);
        axi_write(REG_CTRL, CTRL_START);
        const uint32_t buf_during = axi_read(REG_STATUS) & ST_CPU_BUF;
        CHECK(axi_read(REG_STATUS) & ST_BUSY, "a 1024-point transform is still BUSY");
        axi_write(REG_CTRL, CTRL_START);  // must be ignored
        CHECK_EQ(axi_read(REG_STATUS) & ST_CPU_BUF, buf_during,
                 "START while BUSY does not flip ownership");
        axi_write(REG_CTRL, CTRL_SWAP);  // must be ignored too
        CHECK_EQ(axi_read(REG_STATUS) & ST_CPU_BUF, buf_during,
                 "SWAP while BUSY does not flip ownership");
        // CONF/SCALE are latched for the duration as well.
        axi_write(REG_CONF, 4);
        CHECK_EQ(axi_read(REG_CONF), 10, "CONF is write-protected while BUSY");
        axi_write(REG_SCALE, 0);
        CHECK_EQ(axi_read(REG_SCALE), 0x3FF, "SCALE is write-protected while BUSY");
        int guard = 0;
        while (!(axi_read(REG_STATUS) & ST_DONE) && guard < 200000) ++guard;
        CHECK(guard < 200000, "the 1024-point transform completes");
        axi_write(REG_IRQ, 1);
    }

    // ---- 14. Streaming: fill the next frame while this one computes ----
    // The whole point of the ping-pong. Frame A is transformed while the
    // CPU writes frame B into the other buffer; both results must come
    // out right, and B's samples must not have disturbed A.
    {
        const int log2n = 6;
        const int n = 64;
        std::vector<C16> a(n), b(n);
        for (int i = 0; i < n; ++i) {
            a[i] = C16{q15(0.3 * std::cos(2.0 * M_PI * 7.0 * i / n)), 0};
            b[i] = C16{q15(0.3 * std::cos(2.0 * M_PI * 11.0 * i / n)), 0};
        }

        axi_write(REG_IRQ, 1);
        set_conf(log2n);
        axi_write(REG_SCALE, 0x3FF);
        axi_write(REG_CTRL, 0);

        write_frame(a);
        axi_write(REG_CTRL, CTRL_START);  // engine takes A, CPU gets the other buffer

        // Fill B WHILE the engine is running on A -- this is the access
        // pattern the two-port/ownership split exists for.
        CHECK(axi_read(REG_STATUS) & ST_BUSY, "engine is busy while the CPU fills frame B");
        write_frame(b);

        int guard = 0;
        while (!(axi_read(REG_STATUS) & ST_DONE) && guard < 200000) ++guard;
        CHECK(guard < 200000, "frame A completes");

        axi_write(REG_CTRL, CTRL_START);  // engine takes B, CPU gets A's result
        std::vector<C16> ya;
        read_frame(ya, n);

        guard = 0;
        while (!(axi_read(REG_STATUS) & ST_DONE) && guard < 200000) ++guard;
        CHECK(guard < 200000, "frame B completes");
        axi_write(REG_CTRL, CTRL_SWAP);
        std::vector<C16> yb;
        read_frame(yb, n);

        std::vector<C16> wa = a, wb = b;
        model_fft(wa, log2n, 0x3FF, false);
        model_fft(wb, log2n, 0x3FF, false);

        int bada = 0, badb = 0;
        for (int i = 0; i < n; ++i) {
            if (ya[i].re != wa[i].re || ya[i].im != wa[i].im) ++bada;
            if (yb[i].re != wb[i].re || yb[i].im != wb[i].im) ++badb;
        }
        CHECK_EQ(bada, 0, "frame A result survives the CPU filling frame B");
        CHECK_EQ(badb, 0, "frame B result is correct");

        // And the peaks really are at different bins, i.e. the two frames
        // did not collapse onto the same buffer.
        CHECK(std::abs(ya[7].re) > 200, "frame A peaks at bin 7");
        CHECK(std::abs(yb[11].re) > 200, "frame B peaks at bin 11");
    }

    printf("%d checks, %d failures\n", checks, fails);
    delete top;
    return fails ? 1 : 0;
}

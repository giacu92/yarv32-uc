// Verilator C++ testbench for sim_top.
//
// Drives clk/rst, holds reset for a few cycles, then clocks the design
// and logs every stage in ONE pass:
//   - every instruction word the fetch stage delivers to the F/D
//     register (fetch log),
//   - every decoded instruction the decode stage latches into the D/E
//     register (decode log), and
//   - every operation the execute stage retires into the E/M register
//     (execute log), WITH the writeback value (wb_en / wb_addr / wb_data).
//
// The three logs are recorded in a single run, so they show the honest
// pipeline view (each stage lags the previous by one cycle -- a fetch
// word at cycle N retires a few cycles later in the execute log). The
// previous harness reset+reran the program three times to force the
// three logs cycle-aligned; that was gratuitous and made the core appear
// to "keep rerunning" instead of parking.
//
// The run STOPS EARLY once the core parks: when the retire stream sees
// the same pc+instr for PARK_N consecutive retires (a single-instruction
// self-loop, e.g. start.S `1: j 1b`). This avoids churning ~900 cycles of
// the parked `j` after the program finishes. A max_cyc safety bound
// covers programs that never park (long computes, infinite loops of
// changing state). Park detection on RETIRES (not cycles) means a
// multi-cycle DIV/LSU stall (no retires) never false-triggers, and a
// two-instruction tight loop (alternating instrs) never matches either.
//
// The CPU exports no debug ports: the per-stage taps are internal nets
// inside the CPU (fe_pc_w / de_pc_w / ex_pc_w / wb_en / wb_addr /
// wb_data, ...). The sim is built with --public-flat-rw, so this harness
// reaches them as flat C++ members of the sim_top model
// (TAP(fe_pc), ...). sim_top itself carries only clk / rst / led.
//
// Writeback is sampled on the clock low half (before the edge that
// commits it) so the value lines up with the retiring op; ex_valid
// (registered) is sampled after the edge. A VCD waveform (sim_top.vcd)
// is written for GTKWave.
//
// Build: make   (in sim/)
// Run:   make run   (or ./obj_dir/Vsim_top from sim/)

#include "Vsim_top.h"
#include "Vsim_top___024root.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// CPU-internal debug taps are reached through the Verilator root object.
// The sim is built with --public-flat-rw, which exposes every net as a
// flat public member of the root class, named with the full hierarchy:
// sim_top__DOT__u_cpu__DOT__<sig>. The CPU itself exports no debug ports.
#define TAP(field) (top->rootp->sim_top__DOT__u_cpu__DOT__##field)
// sim_top-level net (not inside u_cpu), e.g. the dbg_stall_o sink.
#define STAP(field) (top->rootp->sim_top__DOT__##field)

// Taps inside the execute stage (one level below the CPU top).
#define XTAP(field) (top->rootp->sim_top__DOT__u_cpu__DOT__u_execute__DOT__##field)

// The D-mem storage array. Every self-checking oracle in sw/ ends by storing
// a pass/fail marker to a fixed D-mem word, so reading that word out is the
// exact result -- as opposed to inferring it from the retire trace, where the
// `li` that materialises the marker shows up whether or not the store that
// follows it ever ran.
// Must match sim_top's u_dmem ADDR_W (14 -> 16 KiB -> 4096 words).
#define DMEM_WORDS 4096u
#define DMEM_WORD(w) (top->rootp->sim_top__DOT__u_dmem__DOT__mem[(w)])

// Taps inside the decode and fetch stages. Note the instance names differ:
// the CPU top instantiates decode as u_decode but fetch as fetch_stage_i.
#define DTAP(field) (top->rootp->sim_top__DOT__u_cpu__DOT__u_decode__DOT__##field)
#define FTAP(field) (top->rootp->sim_top__DOT__u_cpu__DOT__fetch_stage_i__DOT__##field)

// Tap inside the branch predictor (u_bp).
#define BTAP(field) (top->rootp->sim_top__DOT__u_cpu__DOT__u_bp__DOT__##field)

// Taps inside the UART peripheral (sim_top level).
#define UTAP(field) (top->rootp->sim_top__DOT__u_uart__DOT__##field)

// ---------------------------------------------------------------------
// UART RX stimulus driver.
//
// Feeds a byte string into the UART's RX pin as real 8N1 frames (start
// bit, 8 data bits LSB first, stop bit), so a serial-console program
// (YarvMon) can be driven from the harness instead of parking forever in
// its uart_getc() poll.
//
// BIT_CYCLES must match the UART instance in sim_top: the divisor resets
// to CLK_FREQ_HZ/BAUD_RATE - 1 (50 MHz / 10 MHz - 1 = 4), and the RX
// engine samples one bit every div+1 = 5 clocks. Sending faster would
// smear frames; slower would sample the wrong bit.
//
// Pacing: by default a new frame only starts while the UART's RX FIFO has
// room, so the stimulus can never provoke RX_OVERRUN. UART_RX_PACED=0
// removes that interlock and ships frames back-to-back — what a
// line-buffered terminal does with a pasted line, and the case the RX FIFO
// exists to survive.
// ---------------------------------------------------------------------
class UartRxDriver {
public:
    // Clocks per bit. Must match the UART instance: default 5 for the fast
    // sim divisor, or 217 for the board-accurate 25 MHz / 115200 build
    // (UART_BIT_CYCLES env, paired with -GUART_CLK_HZ / -GUART_BAUD).
    static int bit_cycles() {
        const char *e = getenv("UART_BIT_CYCLES");
        return e ? atoi(e) : 5;
    }
    static const int GAP_CYCLES = 10;  // idle-high clocks between frames

    static bool paced() {
        const char *e = getenv("UART_RX_PACED");
        return e ? atoi(e) != 0 : true;
    }

    explicit UartRxDriver(std::string bytes)
        : bytes_(std::move(bytes)), bit_cycles_(bit_cycles()), paced_(paced()) {}

    bool done() const { return idx_ >= (int)bytes_.size() && !active_; }
    int  sent() const { return idx_ - (active_ ? 1 : 0); }

    // Returns the line level to drive for the cycle about to be clocked.
    // blocked = the UART cannot take another byte (RX FIFO full).
    //
    // Pacing is on by default (see class comment). UART_RX_PACED=0 sends
    // frames back-to-back regardless of RX_READY -- what a line-buffered
    // terminal actually does when it ships a whole typed line at once.
    // That is the case where a single-byte RX buffer with no FIFO drops
    // input, so it must be reproducible here.
    int step(bool blocked) {
        if (!active_) {
            if (gap_ > 0) { --gap_; return 1; }
            if (idx_ >= (int)bytes_.size() || (paced_ && blocked)) return 1;
            cur_    = (uint8_t)bytes_[idx_++];
            active_ = true;
            bit_    = 0;  // start bit
            cnt_    = 0;
        }
        int level = (bit_ == 0) ? 0 : (bit_ <= 8 ? ((cur_ >> (bit_ - 1)) & 1) : 1);
        if (++cnt_ == bit_cycles_) {
            cnt_ = 0;
            if (++bit_ > 9) {  // stop bit shipped -> frame complete
                active_ = false;
                gap_    = GAP_CYCLES;
            }
        }
        return level;
    }

private:
    std::string bytes_;
    int         bit_cycles_;
    bool        paced_;
    int         idx_    = 0;
    bool        active_ = false;
    uint8_t     cur_    = 0;
    int         bit_    = 0;
    int         cnt_    = 0;
    int         gap_    = 0;
};

// Decode C-style escapes in the UART_RX env string so a shell can pass
// control characters (\r is the Enter key YarvMon's get_line waits for).
static std::string unescape(const char* s) {
    std::string out;
    for (const char* p = s; *p; ++p) {
        if (*p != '\\' || !p[1]) { out.push_back(*p); continue; }
        switch (*++p) {
            case 'r':  out.push_back('\r'); break;
            case 'n':  out.push_back('\n'); break;
            case 't':  out.push_back('\t'); break;
            case '0':  out.push_back('\0'); break;
            case '\\': out.push_back('\\'); break;
            case 'x': {
                int v = 0, n = 0;
                while (n < 2 && isxdigit((unsigned char)p[1])) {
                    char c = *++p;
                    v = v * 16 + (isdigit((unsigned char)c) ? c - '0'
                                                            : (tolower(c) - 'a' + 10));
                    ++n;
                }
                out.push_back((char)v);
                break;
            }
            default: out.push_back(*p); break;
        }
    }
    return out;
}

static vluint64_t sim_time = 0;

// Waveform dumping is skipped entirely when NO_VCD=1 (see main()).
static bool g_no_vcd = false;

static inline void vdump(VerilatedVcdC* tfp, vluint64_t t) {
    if (!g_no_vcd) tfp->dump(t);
}

// One full clock period: low half then high half, dumping the waveform
// at each step. Outputs are sampled after the rising edge.
static void tick(Vsim_top* top, VerilatedVcdC* tfp) {
    top->clk_i = 0;
    top->eval();
    vdump(tfp, sim_time++);
    top->clk_i = 1;
    top->eval();
    vdump(tfp, sim_time++);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    // Per-instruction fe/de/ex+wb logging is for the hand-crafted Harvard
    // oracle (sim/imem.hex + sim/dmem.hex), which is 36 instructions long and
    // exists precisely so one can read every cycle of it. A firmware run under
    // sim/sw/ is six orders of magnitude longer: CoreMark retires ~330k
    // instructions, so the logs cost a snprintf and a heap string per stage
    // per cycle, hundreds of MB of retained std::string, and a final print
    // nobody scrolls through. The counters, the histogram and the CPI
    // decomposition below are built either way -- only the transcript goes.
    //
    // The oracle is the run with no +IINIT plusarg (sim_top defaults to
    // imem.hex/dmem.hex), so absence of +IINIT is the default-on condition.
    // INSTR_LOG=1 forces the transcript back on for a firmware run when
    // something needs reading instruction by instruction; INSTR_LOG=0 turns
    // it off for the oracle.
    bool instr_log = true;
    for (int i = 1; i < argc; ++i) {
        if (strncmp(argv[i], "+IINIT=", 7) == 0) { instr_log = false; break; }
    }
    if (const char *e = getenv("INSTR_LOG")) instr_log = atoi(e) != 0;


    Vsim_top* top = new Vsim_top;
    // NO_VCD=1 skips the waveform dump. A board-accurate UART run (217
    // clocks per bit) needs millions of cycles, which would produce a
    // multi-gigabyte VCD nobody can open.
    const bool no_vcd = getenv("NO_VCD") && atoi(getenv("NO_VCD"));
    g_no_vcd = no_vcd;
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (!no_vcd) {
        Verilated::traceEverOn(true);
        top->trace(tfp, 99);
        tfp->open("sim_top.vcd");
    }

    printf("=== RV32 fetch + decode + execute (LSU) sim ===\n");

    // Reset (async, active-low): hold rstn_i=0 for a few cycles, then
    // release. We set rstn_i (and the initial clk_i) WITHOUT a standalone
    // eval+dump: dumping with clk_i unchanged repeats the last tick's clock
    // level and stretches the clock to two half-cycles at one level (a
    // "plateau"). Instead the next tick's low-half eval carries the reset
    // change, keeping the clock a clean 0/1/0/1 throughout.
    top->clk_i  = 0;
    top->rstn_i = 0;

    // UART RX stimulus: UART_RX="2000\r" types that into the console
    // program (escapes decoded above). Unset = idle line, i.e. the old
    // behaviour (a uart_getc() poll never completes).
    UartRxDriver rx_drv(getenv("UART_RX") ? unescape(getenv("UART_RX")) : std::string());
    top->uart_rxd_i = 1;  // idle high

    for (int i = 0; i < 4; ++i) tick(top, tfp);

    // Release reset: carried by the run loop's first tick below.
    top->rstn_i = 1;

    // -----------------------------------------------------------------
    // Single pass: record the three stage logs, stop once parked.
    // -----------------------------------------------------------------
    const int PARK_N  = 8;     // consecutive identical retires => parked
    // Safety bound for programs that don't park. Override with the MAX_CYC
    // env var (e.g. MAX_CYC=20000 make run) for longer-running programs.
    const int max_cyc = [] {
        const char *e = getenv("MAX_CYC");
        return e ? atoi(e) : 80000;
    }();

    // Optional machine-readable commit log for co-sim (diff vs Spike).
    // Set RTL_TRACE=<path> to emit one line per retire:
    //   0x<pc> x<rd> 0x<rd_value>
    // (rd=0 / value=0 when the retire writes no register, matching Spike's
    //  "no register delta" convention so retire counts align 1:1). Unset =
    // no trace file, behaviour byte-identical to the plain run. Sampled with
    // the same pre-edge-wb / post-edge-valid split as the human ex log.
    FILE* trace_fp = nullptr;
    if (const char *t = getenv("RTL_TRACE")) {
        trace_fp = fopen(t, "w");
        if (!trace_fp)
            fprintf(stderr, "RTL_TRACE: cannot open '%s' for write\n", t);
    }

    std::vector<std::string> fe_log, de_log, ex_log;
    int fetched = 0, decoded = 0, retired = 0;
    int stalled = 0;  // cycles the pipe was stalled (dbg_stall_o=1)

    // ---- Stall-cause accounting ---------------------------------------
    // `stalled` above answers "was dbg_stall_o high", which is only part of
    // the story: dbg_stall_o is dec_stall | ex_stall, and a fetch bubble --
    // decode holding nothing because the instruction buffer is empty -- is
    // neither. It shows up as a non-stall cycle that retires nothing, so
    // removing fetch bubbles (as the 64-bit/2-outstanding rewrite did)
    // RAISES the reported stall percentage while lowering the cycle count.
    // Comparing that percentage across two designs, or across two
    // programs, therefore says very little.
    //
    // This histogram answers the question that does compare: for every
    // cycle in which no instruction retired, what was holding the pipe.
    // Retire cycles plus these buckets are exactly the run, so the columns
    // sum and IPC falls straight out of them.
    //
    // The order below is a priority, and it is the order in which a cycle
    // is actually blocked: execute owns the retire slot, so if execute is
    // busy nothing else matters; decode's own bubbles come next; and only
    // if neither is holding anything is an empty buffer the fetch path's
    // fault.
    enum {
        SC_LOAD_WAIT,  // EX_MEM_WAIT: load issued, waiting for rvalid
        SC_LSU_LAUNCH, // LSU issuing: D-mem drive, peri capture, or peri drive
        SC_LSU_MISTRAP,// EX_MEM_TRAP: misaligned access, raising its sync trap
        SC_DIV,        // DIV/REM multi-cycle
        SC_CSR,        // EX_CSR_WAIT: the registered CSR read
        SC_WFI,        // halted in wfi until an interrupt is pending
        SC_RVC_HOLD,   // compressed upper half held while a fresh word arrived
        SC_RVC_SPAN,   // 32-bit instruction straddling a word boundary
        SC_REDIRECT,   // between a branch/trap redirect and the next retire
        SC_IMEM,       // buffer empty with no redirect: fetch could not keep up
        SC_DEC_BUBBLE, // words available but decode produced no instruction
        SC_OTHER,      // residual: decode produced one, execute retired nothing
        SC_N
    };
    static const char *const sc_name[SC_N] = {
        "load-wait     (EX_MEM_WAIT)", "lsu-launch    (bus accept)",
        "lsu-mistrap   (misaligned)",  "div/rem       (multi-cycle)",
        "csr-read      (EX_CSR_WAIT)", "wfi-halt",
        "rvc-hold      (upper half)",  "rvc-span      (word straddle)",
        "redirect      (flush + refill)", "imem-starve   (fetch behind)",
        "decode-bubble (no instr out)",  "other"
    };
    long sc_count[SC_N] = {0};
    // Cycles actually executed. NOT the same as the loop's `cyc`: park
    // detection breaks out mid-body, so the run is cyc+1 iterations long
    // and a histogram totalled from `cyc` would be one short of the sum of
    // its own buckets.
    long counted_cyc = 0;
    int  sc_this = SC_OTHER;    // cause classified for the current cycle
    bool post_redirect = false; // a redirect fired and nothing has retired since

    // Event counters, so a bucket can be read per operation rather than
    // only as a total (e.g. load-wait cycles divided by loads).
    long n_loads = 0, n_stores = 0, n_divs = 0, n_csrs = 0, n_redirects = 0;

    bool pv_div = false;
    bool pv_csr = false, pv_redir = false;
    // Branch-predictor event counters. n_bp_resolved = every resolved
    // control-flow instr; n_bp_predt = those predicted taken at decode;
    // n_bp_mispred = mispredict redirects. Counted per cycle the wire is
    // high, NOT on its rising edge: cf_resolving is a level that is high for
    // exactly the one cycle each control-flow instr resolves, so two of them
    // resolving in consecutive cycles (a not-taken branch followed by another
    // branch -- what `if (a && b)` compiles to) is two events with no falling
    // edge in between. Edge-counting merged those pairs, under-reporting
    // CoreMark's control-flow count by 2917 of 68234 and silently dropping
    // any mispredict landing on the second cycle of a pair.
    // RAS hit/miss come from the predictor's own free-running counters
    // (sampled once at the end).
    long n_bp_resolved = 0, n_bp_predt = 0, n_bp_mispred = 0;
    uint32_t prev_fe_pc = 0;
    bool have_prev_fe = false;
    int fe_pc_checked = 0, fe_pc_ok = 0, fe_pc_bad = 0;

    // Park detection on the retire stream.
    uint32_t last_ex_pc = 0, last_ex_instr = 0;
    int same_ret = 0;
    int park_cyc = -1;  // cycle the core was first detected parked

    // ---- WFI-halt liveness checks -------------------------------------
    // WFI freezes the whole pipe until a pending+enabled interrupt wakes it
    // (execute_stage: wfi_stall = wfi_halt_q & ~int_pending). Two ways that
    // can go wrong, both of which brick the core with no recovery but reset:
    //
    //  1. STUCK-AFTER-WAKE. wfi_halt_q must drop as soon as int_pending goes
    //     high, whatever event actually wins arbitration that cycle. If it
    //     only cleared on take_interrupt, a sync trap (illegal instruction /
    //     misaligned access) sitting right behind the WFI would win the
    //     priority contest, leave wfi_halt_q set, and then trap entry clears
    //     mstatus.MIE -> int_pending falls -> wfi_stall re-asserts and freezes
    //     the pipe INSIDE the handler, which can never retire the CSR write
    //     that would re-enable interrupts. Post-fix, (wfi_halt_q &&
    //     int_pending) lasts a single cycle; a halt still held long after a
    //     wake was observed is that regression.
    //
    //  2. NEVER-WOKEN. A halt held for an implausibly long time with no
    //     retires at all. Legal per spec (a WFI with no interrupt source
    //     armed waits forever), but no oracle in this tree does that, so it
    //     means the wake path broke.
    //
    // Both bounds are generous and env-overridable; they exist to turn a
    // silent "ran to MAX_CYC" into a named failure.
    const int wfi_stuck_n = []() {
        const char *e = getenv("WFI_STUCK_N");
        return e ? atoi(e) : 64;
    }();
    const int wfi_halt_n = []() {
        const char *e = getenv("WFI_HALT_N");
        return e ? atoi(e) : 4000;
    }();
    bool wfi_wake_seen  = false;  // (wfi_halt_q && int_pending) observed
    int  wfi_since_wake = 0;      // cycles wfi_halt_q held since that wake
    int  wfi_held       = 0;      // cycles wfi_halt_q held continuously
    int  wfi_fail_cyc   = -1;
    const char *wfi_fail_why = nullptr;

    char line[96];
    int cyc;
    for (cyc = 0; cyc < max_cyc; ++cyc) {
        // UART RX line level for this clock period, paced off the RX FIFO.
        top->uart_rxd_i = rx_drv.step(UTAP(rx_fifo_full) != 0);

        // Low half: de_q holds the op about to retire at this edge; sample
        // its writeback before the edge commits it (wb is combinational
        // from de_q, so the pre-edge value lines up with this retire).
        top->clk_i = 0;
        top->eval();
        vdump(tfp, sim_time++);
        bool     wb_en  = TAP(wb_en);
        uint32_t wb_addr = TAP(wb_addr);
        uint32_t wb_data = TAP(wb_data);

        // ---- classify this cycle's blocking cause ----
        // Sampled here, in the low half, on purpose: ex_state_q / count_q
        // and the combinational stall terms derived from them hold THIS
        // cycle's values before the edge. Post-edge they would already be
        // the next cycle's, which is fine for a yes/no aggregate but wrong
        // for attributing a cycle to a cause. Whether the cycle counts at
        // all is decided after the edge, from ex_valid.
        bool ev_redir_now = false;
        {
            // One pulse per completed mem op: a load on its rvalid, a
            // (posted) store on its launch accept. Counting completions
            // rather than issues is edge-safe now that a store both issues
            // and retires in EX_IDLE — two back-to-back stores would leave
            // the issue term high across the boundary and lose a count.
            const bool ev_mem_done = XTAP(mem_done) != 0;
            const bool ev_store_done = XTAP(store_done) != 0;
            const bool ev_div      = XTAP(alu_start) != 0;
            const bool ev_csr      = XTAP(csr_start) != 0;
            const bool ev_redir    = XTAP(branch_valid_o) != 0;
            // Branch predictor: cf_resolving is high for exactly one cycle
            // per resolved control-flow instr (the denominator); pred_t is the prediction
            // decode carried (valid & taken); mispredict is the execute-
            // internal redirect-that-was-a-mispred wire. All three are flat
            // internal wires (tapping the packed bp_train_o struct port would
            // mean slicing a QData by hand).
            const bool ev_bp_res   = XTAP(cf_resolving) != 0;
            const bool ev_bp_predt = XTAP(pred_t) != 0;
            const bool ev_bp_mis   = XTAP(mispredict) != 0;

            // Rising edges only for the FSM-event counters below: these are
            // one-cycle pulses by construction (all are gated on
            // ex_state_q == EX_IDLE), but counting edges keeps the totals
            // right if that ever stops being true. The bp_* counters must NOT
            // use an edge gate -- see their declaration.
            if (ev_mem_done) ++n_loads;    // only loads reach EX_MEM_WAIT
            if (ev_store_done) ++n_stores;  // posted store, at its accept
            if (ev_div && !pv_div) ++n_divs;
            if (ev_csr && !pv_csr) ++n_csrs;
            if (ev_redir && !pv_redir) ++n_redirects;
            // Every high cycle, no edge gate (see the declaration): one
            // control-flow instr resolves per high cycle of cf_resolving.
            if (ev_bp_res) ++n_bp_resolved;
            if (ev_bp_predt && ev_bp_res) ++n_bp_predt;  // predicted-taken, at resolve
            if (ev_bp_mis) ++n_bp_mispred;  // mispredict implies cf_resolving
            pv_div = ev_div;
            pv_csr = ev_csr; pv_redir = ev_redir;

            // NOT applied here: the redirect cycle is normally the branch's
            // own retire, and the post-edge retire test below would clear
            // the flag again in the same cycle. It is set after that test.
            ev_redir_now = ev_redir;

            const bool buf_empty = (FTAP(count_q) == 0);

            if (XTAP(mem_running))                       sc_this = SC_LOAD_WAIT;
            // A store retires on the launch accept, so that cycle is a
            // retire and never reaches the histogram; a load's accept
            // cycle does, and belongs to the launch bucket.
            // Covers every issue cycle: the live D-mem load drive, the
            // capture cycle for a peri access or a D-mem store, and the
            // captured drive from those flops. A store's accept cycle is a
            // retire and never reaches the histogram.
            else if ((XTAP(mem_bus_drive) || XTAP(mem_capture)) && !XTAP(store_done))
                sc_this = SC_LSU_LAUNCH;
            else if (XTAP(misaligned_trap))              sc_this = SC_LSU_MISTRAP;
            else if (XTAP(div_running) || ev_div)        sc_this = SC_DIV;
            else if (ev_csr)                             sc_this = SC_CSR;
            else if (XTAP(wfi_stall))                    sc_this = SC_WFI;
            else if (DTAP(resource_stall))               sc_this = SC_RVC_HOLD;
            else if (DTAP(span_wait))                    sc_this = SC_RVC_SPAN;
            // An empty buffer is charged to the redirect that emptied it
            // until something retires again -- the drain AND the refill are
            // both the branch's cost, not the I-mem's.
            // Everything between a redirect and the next retire is the
            // redirect's cost, whatever it looks like locally: the flushed
            // D/E slot, the killed buffer, and the refill. Splitting those
            // hides the number that matters, which is cycles per taken
            // branch / trap / mret.
            else if (post_redirect)                      sc_this = SC_REDIRECT;
            else if (buf_empty)                          sc_this = SC_IMEM;
            // Words are in the buffer but decode emitted nothing: odd-half
            // realignment, or a spanning stitch waiting on its second word.
            else if (!DTAP(decoded_valid))               sc_this = SC_DEC_BUBBLE;
            else                                         sc_this = SC_OTHER;
        }

        // High half: the posedge commits the writeback and updates the
        // stage registers. fe/de/ex valid are sampled post-edge.
        top->clk_i = 1;
        top->eval();
        vdump(tfp, sim_time++);

        // Aggregate pipe-stall status for this cycle (the old dbg_stall_o
        // tap, = dec_stall | ex_stall; the CPU port was removed, so the
        // nets are tapped through the hierarchy instead).
        // Sampled post-edge; counts RAW-hazard / DIV / LSU stalls.
        if (TAP(dec_stall) || TAP(ex_stall)) ++stalled;

        // ---- WFI-halt liveness (see the declarations above) ----
        {
            const bool halt = XTAP(wfi_halt_q) != 0;
            const bool ipend = XTAP(int_pending_i) != 0;
            if (!halt) {
                wfi_held       = 0;
                wfi_since_wake = 0;
                wfi_wake_seen  = false;
            } else {
                ++wfi_held;
                if (ipend) wfi_wake_seen = true;
                if (wfi_wake_seen) ++wfi_since_wake;

                if (wfi_fail_cyc < 0 && wfi_wake_seen && wfi_since_wake > wfi_stuck_n) {
                    wfi_fail_cyc = cyc;
                    wfi_fail_why =
                        "wfi_halt_q still set >WFI_STUCK_N cycles after an interrupt "
                        "went pending -- the WFI wake path lost arbitration (sync trap / "
                        "mret) and never released the halt";
                } else if (wfi_fail_cyc < 0 && wfi_held > wfi_halt_n) {
                    wfi_fail_cyc = cyc;
                    wfi_fail_why =
                        "wfi_halt_q held >WFI_HALT_N cycles with no wake -- no enabled "
                        "interrupt ever reached the core";
                }
            }
        }

        // ---- fetch log (one line per cycle fe_valid is high) ----
        // Fetch is WORD-granular: it always advances by 4, never by 2 (the
        // +2 upper-half / spanning case is owned by decode's hold buffer).
        // is-compressed (c) is derived from instr[1:0] -- fetch no longer
        // exports it. fe_valid is a held level, so a stall (DIV/LSU busy)
        // repeats the same word for several cycles -- that is the stall,
        // not a bug.
        if (TAP(fe_valid)) {
            uint32_t pc    = TAP(fe_pc);
            uint32_t instr = TAP(fe_instr);
            int      c     = (instr & 3u) != 3u;
            if (instr_log) {
                snprintf(line, sizeof(line), "%5d  0x%08x  0x%08x  %c",
                         cyc, pc, instr, c ? 'C' : '.');
                fe_log.push_back(line);
            }
            if (have_prev_fe) {
                ++fe_pc_checked;
                if (pc == prev_fe_pc + 4u) ++fe_pc_ok; else ++fe_pc_bad;
            }
            prev_fe_pc = pc;
            have_prev_fe = true;
            ++fetched;
        }

        // ---- decode log (one line per cycle de_valid is high) ----
        // de_instr is the 32-bit word decode TREATED (native or
        // RVC-expanded), so is-compressed cannot be recovered from it.
        if (TAP(de_valid)) {
            uint32_t pc    = TAP(de_pc);
            uint32_t instr = TAP(de_instr);
            if (instr_log) {
                snprintf(line, sizeof(line), "%3d  0x%08x  0x%08x", cyc, pc, instr);
                de_log.push_back(line);
            }
            ++decoded;
        }

        // ---- execute retire + writeback log ----
        // ex_* is the E/M register: it latches the PC / instr / valid of
        // the operation that retired this cycle (single-cycle ALU ops
        // retire the cycle they are valid; DIV/REM retire when the ALU
        // asserts result_valid_o; loads/stores retire when the mem
        // response arrives). Illegal ops do not retire (ex_valid stays 0).
        if (wb_en && instr_log) {
            snprintf(line, sizeof(line), "%3d  wb    x%-2u = 0x%08x", cyc, wb_addr, wb_data);
            ex_log.push_back(line);
        }
        // A cycle either retires an instruction or it does not; the
        // no-retire ones go to the cause classified above. The two
        // together are the whole run.
        ++counted_cyc;
        if (TAP(ex_valid)) {
            post_redirect = false;
        } else {
            ++sc_count[sc_this];
        }
        // Set after the retire test, not before it: a taken branch retires
        // in the very cycle it asserts the redirect, so setting the flag
        // earlier would have it cleared again immediately and the refill
        // bubble charged to the I-mem instead of to the branch.
        if (ev_redir_now) post_redirect = true;

        if (TAP(ex_valid)) {
            uint32_t pc    = TAP(ex_pc);
            uint32_t instr = TAP(ex_instr);
            if (instr_log) {
                snprintf(line, sizeof(line), "%3d  ex    0x%08x  0x%08x", cyc, pc, instr);
                ex_log.push_back(line);
            }
            ++retired;

            // Co-sim commit log: one line per retire, pc + reg effect.
            // wb_* were sampled pre-edge (above) and line up with this retire.
            if (trace_fp) {
                // x0 is hardwired zero: an architectural write to x0 is a
                // NOP (the regfile discards it), and Spike's commit log
                // emits no register delta for it. Mask wb_addr==0 so the
                // trace matches Spike (x0 0x0) instead of leaking the
                // discarded writeback value (e.g. a JAL x0 link address).
                uint32_t rd  = (wb_en && wb_addr != 0) ? wb_addr : 0u;
                uint32_t val = (wb_en && wb_addr != 0) ? wb_data : 0u;
                fprintf(trace_fp, "0x%08x x%u 0x%08x\n", pc, rd, val);
            }

            // Park detection: N consecutive IDENTICAL retires => the
            // core is spinning on a single-instruction self-loop.
            if (pc == last_ex_pc && instr == last_ex_instr) {
                ++same_ret;
            } else {
                same_ret = 1;
            }
            last_ex_pc    = pc;
            last_ex_instr = instr;
            if (same_ret >= PARK_N && park_cyc < 0) {
                park_cyc = cyc;
                break;
            }
        }
    }

    // -----------------------------------------------------------------
    // Print the three log sections. The per-cycle transcripts appear only
    // when instr_log is set (the Harvard oracle, or INSTR_LOG=1); the
    // per-stage summary line under each is always printed, so a firmware run
    // still reports fetched / decoded / retired and the +4 word-advance
    // check.
    // -----------------------------------------------------------------
    if (instr_log) {
        printf("--- fetch (fe) ---\n");
        printf("cycle  pc          instr       c\n");
        printf("-----  ----------  ----------  -\n");
        for (const std::string& s : fe_log) printf("%s\n", s.c_str());
        printf("-----  ----------  ----------  -\n");
    }
    printf("fetched %d words in %d cycles | word-advance +4: %d ok / %d bad\n",
           fetched, cyc, fe_pc_ok, fe_pc_bad);

    if (instr_log) {
        printf("--- decode (de) ---\n");
        printf("cyc  pc          instr\n");
        printf("---  ----------  ----------\n");
        for (const std::string& s : de_log) printf("%s\n", s.c_str());
        printf("---  ----------  ----------\n");
    }
    printf("decoded %d instructions in %d cycles\n", decoded, cyc);

    if (instr_log) {
        printf("--- execute (ex) + writeback (wb) ---\n");
        printf("cyc  kind  pc / value\n");
        printf("---  ----  -----------------------\n");
        for (const std::string& s : ex_log) printf("%s\n", s.c_str());
        printf("---  ----  -----------------------\n");
    }
    if (park_cyc >= 0) {
        printf("retired %d instructions in %d cycles (parked at cyc %d: "
               "self-loop detected after %d identical retires)\n",
               retired, cyc, park_cyc, PARK_N);
    } else {
        printf("retired %d instructions in %d cycles (no park: hit %d-cycle safety bound)\n",
               retired, cyc, max_cyc);
    }
    printf("IPC = %.3f (retired / cycles)\n", 1.0 * retired / cyc);

    // Pipe-stall breakdown: stalled cycles vs total run cycles. A stall
    // cycle is one where dbg_stall_o (= dec_stall | ex_stall) was high --
    // the RAW-hazard bubble, the DIV/REM multi-cycle hold, or the LSU
    // EX_MEM_WAIT. The parked tail (after the program parks) is included
    // but contributes only a few non-stall cycles.
    if (cyc > 0) {
        printf("stalled %d/%d cycles (%.1f%%)\n",
               stalled, cyc, 100.0 * stalled / cyc);
    }

    // Cause histogram over the cycles that retired nothing. Read this and
    // not the percentage above when comparing designs: dbg_stall_o does not
    // see a fetch bubble, so a change that removes fetch bubbles lowers the
    // cycle count and raises that percentage at the same time.
    if (counted_cyc > 0) {
        const long nonretire = counted_cyc - retired;
        printf("\nno-retire cycles %ld/%ld (%.1f%%)  --  retired %d (%.1f%%)\n",
               nonretire, counted_cyc, 100.0 * nonretire / counted_cyc,
               retired, 100.0 * retired / counted_cyc);
        for (int i = 0; i < SC_N; ++i) {
            if (!sc_count[i]) continue;
            printf("  %-30s %8ld  %5.1f%% of run  %5.1f%% of no-retire\n",
                   sc_name[i], sc_count[i], 100.0 * sc_count[i] / counted_cyc,
                   nonretire ? 100.0 * sc_count[i] / nonretire : 0.0);
        }
        printf("  %-30s %8ld\n", "TOTAL", nonretire);

        // Per-operation cost, which is what a bucket total does not say:
        // "load-wait is 20% of the run" could be many cheap loads or few
        // expensive ones, and only the second is a memory-system problem.
        printf("events: mem-ops %ld (loads %ld, stores %ld), div/rem %ld, "
               "csr %ld, redirects %ld\n",
               n_loads + n_stores, n_loads, n_stores, n_divs, n_csrs, n_redirects);
        if (n_loads + n_stores)
            printf("        %.2f no-retire cyc per mem-op, %.2f per load "
                   "(wait only)\n",
                   1.0 * (sc_count[SC_LOAD_WAIT] + sc_count[SC_LSU_LAUNCH]
                          + sc_count[SC_LSU_MISTRAP]) / (n_loads + n_stores),
                   n_loads ? 1.0 * sc_count[SC_LOAD_WAIT] / n_loads : 0.0);
        if (n_redirects)
            printf("        %.2f no-retire cyc per redirect\n",
                   1.0 * sc_count[SC_REDIRECT] / n_redirects);

        // The buckets are a partition of the no-retire cycles by
        // construction (one increment per non-retiring cycle), so this can
        // only fire if a cycle was classified twice or the run length and
        // the classification loop disagree. It is a self-check on the
        // instrumentation, not on the core.
        long sum = 0;
        for (int i = 0; i < SC_N; ++i) sum += sc_count[i];
        if (sum != nonretire)
            printf("  (INTERNAL: buckets sum to %ld, expected %ld)\n", sum, nonretire);
    }

    // CPI decomposition. Same data as the histogram, divided by retires
    // instead of by cycles, which is the form that compares: a percentage
    // of the run moves when any *other* bucket changes, while cycles per
    // retired instruction is a per-instruction cost that stands on its own.
    // The terms are additive by construction -- every cycle is either a
    // retire or exactly one bucket -- so 1.0 plus the buckets IS the CPI,
    // and the floor of 1.0 is the retire slot itself (one instruction per
    // cycle, in order, is all this pipeline can do).
    if (retired > 0) {
        printf("\nCPI decomposition (cycles per retired instruction):\n");
        printf("    %-31s %7.3f\n", "retire (floor)", 1.0);
        for (int i = 0; i < SC_N; ++i) {
            if (!sc_count[i]) continue;
            const double cpi = 1.0 * sc_count[i] / retired;
            // A non-empty bucket printed as 0.000 reads as "this costs
            // nothing", which is a different claim from "this is present
            // and below the resolution shown".
            if (cpi < 0.0005)
                printf("  + %-31s  <0.001\n", sc_name[i]);
            else
                printf("  + %-31s %7.3f\n", sc_name[i], cpi);
        }
        printf("  = %-31s %7.3f   (IPC %.3f)\n", "CPI",
               1.0 * counted_cyc / retired, 1.0 * retired / counted_cyc);

        // What generates those costs: a per-operation cost times how often
        // the operation occurs. Both halves are needed to read a term --
        // 3 cycles per redirect is cheap if branches are rare and is the
        // whole problem if one instruction in seven is a taken branch.
        printf("density: ");
        if (n_redirects)
            printf("1 redirect per %.2f instr", 1.0 * retired / n_redirects);
        if (n_loads + n_stores)
            printf(", mem-ops %.1f%% of instr", 100.0 * (n_loads + n_stores) / retired);
        if (n_divs)
            printf(", div/rem %.2f%%", 100.0 * n_divs / retired);
        printf("\n");

        // Branch-predictor stats. n_bp_resolved = every control-flow instr
        // that reached execute's resolve (the denominator for accuracy and
        // MPKI); n_bp_predt = those decode predicted taken; n_bp_mispred =
        // mispredict redirects. RAS hit/miss are the predictor's own
        // free-running counters (returns consumed at decode vs RAS
        // non-empty). With BP_EN=0 these are all zero (no prediction).
        if (n_bp_resolved > 0) {
            const long correct = n_bp_resolved - n_bp_mispred;
            printf("branch predictor: %ld resolved, %ld predicted-taken, "
                   "%ld mispredicts (%.2f%% accuracy, %.2f MPKI)\n",
                   n_bp_resolved, n_bp_predt, n_bp_mispred,
                   100.0 * correct / n_bp_resolved,
                   1000.0 * n_bp_mispred / retired);
            const long ras_hits   = (long)BTAP(ras_hit_q);
            const long ras_misses = (long)BTAP(ras_miss_q);
            const long ras_tot    = ras_hits + ras_misses;
            if (ras_tot > 0)
                printf("  RAS: %ld returns, %ld hits / %ld misses (%.1f%% hit)\n",
                       ras_tot, ras_hits, ras_misses,
                       100.0 * ras_hits / ras_tot);
        }
    }

    if (wfi_fail_cyc >= 0) {
        printf("WFI-HALT FAIL at cycle %d: %s\n", wfi_fail_cyc, wfi_fail_why);
    } else {
        printf("WFI-halt check: OK\n");
    }

    // PROBE=<byte addr>[,<byte addr>...] prints those D-mem words, which is
    // how `make regress` reads an oracle's pass marker. Word index = addr/4;
    // out-of-range addresses print as ---- rather than reading past the array.
    if (const char *pl = getenv("PROBE")) {
        std::string spec(pl);
        size_t pos = 0;
        while (pos < spec.size()) {
            size_t comma = spec.find(',', pos);
            std::string one = spec.substr(pos, comma == std::string::npos
                                                  ? std::string::npos
                                                  : comma - pos);
            pos = (comma == std::string::npos) ? spec.size() : comma + 1;
            if (one.empty()) continue;
            const unsigned long addr = strtoul(one.c_str(), nullptr, 0);
            const unsigned long word = addr / 4;
            if (word < DMEM_WORDS)
                printf("probe[0x%08lx] = 0x%08x\n", addr, DMEM_WORD(word));
            else
                printf("probe[0x%08lx] = ---- (outside the %lu-word D-mem)\n",
                       addr, (unsigned long)DMEM_WORDS);
        }
    }

    // Let a few final cycles ripple for the waveform tail.
    for (int i = 0; i < 4; ++i) tick(top, tfp);

    if (trace_fp) fclose(trace_fp);

    if (!no_vcd) tfp->close();
    delete top;
    delete tfp;
    return wfi_fail_cyc >= 0 ? 1 : 0;
}
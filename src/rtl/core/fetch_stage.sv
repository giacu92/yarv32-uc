`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Fetch stage of the pipeline — 64-bit, 2-outstanding, with a 32-bit
 * instruction buffer.
 *
 * Owns the PC and drives a 64-bit read-only native I-mem interface
 * (ifetch_req_t / ifetch_rsp_t). One 8-byte access delivers two 32-bit
 * words (3-4 with RVC); two reads may be outstanding, so the BSRAM keeps
 * issuing through the decode stalls (DIV/REM, mem-wait) that would idle a
 * single-outstanding port. A depth-8 32-bit-word instruction buffer decouples
 * the 64-bit fetch rate from the 32-bit decode rate: each 64-bit response is
 * split into one or two 32-bit entries (PC-stamped) and pushed at the tail;
 * decode pops the head one word per cycle.
 *
 * Native interface (valid/ready on both sides, see rv32_pkg):
 *   - Request launch : imem_req_o.valid && imem_rsp_i.ready
 *   - Read response  : imem_rsp_i.rvalid && imem_req_o.rready
 *
 * Decode contract: the buffer head is exported as fe_instr /
 * fe_pc / fe_valid / fe_fault, 32-bit / 32-bit / 1-bit / 1-bit, exactly as
 * the old 2-deep F/D+skid FIFO exported them. Decode is a pure consumer of
 * the head register and cannot tell a 32-bit RAM from a split 64-bit RAM.
 * A second read port exports head+1 (fe_next_instr / fe_next_pc /
 * fe_next_valid = count>=2 / fe_next_fault) so decode can same-cycle stitch
 * a 32-bit instr at a 2-byte-aligned branch target (offset 2/6) without a
 * bubble; fe_pop2_i tells fetch to drop both entries on that stitch. The
 * sequential RVC spanning stitch (span_wait, a 32-bit instr at offset 2
 * reached by fall-through) still lives in decode and costs 1 bubble --
 * removing it is dual-issue, a separate change.
 *
 * PC + split:
 *   - pc_q advances by 8 in steady state; pc_d = (pc_q & ~7) + 8 after a
 *     launch/fault. On a redirect pc_q = branch_addr_i (possibly 2-aligned
 *     for an RVC odd-half target, exactly as before).
 *   - req_pc_q holds the real (possibly unaligned) PC of the in-flight
 *     read. The I-mem aligns the address down to 8; fetch splits the
 *     64-bit word back into 32-bit halves at the right PCs:
 *       base = req_pc_q & ~7; low word @base, high word @base+4.
 *       req_pc_q[2]==0 -> push low (pc=req_pc_q, carries unaligned [1:0])
 *                         then high (pc=base+4).           [2 words]
 *       req_pc_q[2]==1 -> skip low (before target), push high
 *                         (pc=req_pc_q, carries unaligned [1]). [1 word]
 *     This reproduces the old "RAM aligns down, fe_pc carries the unaligned
 *     req_pc, decode uses fe_pc[1]" contract — traced for redirect->0x2 and
 *     redirect->0x6.
 *
 * 2-outstanding + redirect drain (generation-tagged, non-blocking):
 *   - inflight_q (0..2): outstanding reads (inc on launch, dec on rsp_cap).
 *   - stale_q (0..2): how many of those outstanding reads were launched
 *     before the last redirect. Responses come back strictly in order (the
 *     I-mem is a fixed-latency BSRAM behind an in-order skid FIFO), so the
 *     next stale_q responses belong to the killed path and the ones after
 *     them belong to the new path. A redirect therefore sets
 *     stale_q = inflight, and each accepted response decrements it; a
 *     response arriving while stale_q != 0 is accepted (rready=1) and
 *     discarded instead of pushed.
 *
 *     This is the whole point of the counter: fetching down the new path
 *     does NOT wait for the old path to drain. The former draining_q flag
 *     blocked bus_issue until inflight reached 0, so every redirect that
 *     caught a read in flight paid an extra cycle (or two) before the first
 *     request at the target could even be launched. With the counter the
 *     redirect cycle itself issues at the target (bus_issue drops the
 *     pc_fault term on a redirect and issue_addr supplies the address)
 *     while the stale response is still in flight behind it.
 *
 *     req_pc_q (single register, see below) stays safe under this: a launch
 *     may only overwrite it when no *good* response is pending. Two facts
 *     make that hold, and neither is local to this file:
 *       (a) rready is low only when count_q >= 7, and count_q >= 7 forces
 *           inflight == 0 (the reserved <= 8 invariant), so a response is
 *           never left waiting while a read is outstanding; and
 *       (b) the I-mem answers in exactly 1 cycle, so the only cycle on which
 *           inflight_q is non-zero at a launch is the cycle that response
 *           lands -- it is consumed with the old req_pc_q before the
 *           overwrite takes effect at the clock edge.
 *     A response consumed under stale_q != 0 is discarded and never reads
 *     req_pc_q at all.
 *
 *     Consequence worth knowing before this port is re-pointed: (b) is an
 *     assumption about the SLAVE, not something this module enforces. Under
 *     a variable-latency I-mem (a cache over the in-package SDRAM, say)
 *     inflight_q really would reach 2, and the older response would then be
 *     split with the younger read's PC stamp -- silently wrong fe_pc, hence
 *     wrong branch targets and link addresses. Measured on the BSRAM the
 *     counter never leaves {0,1}, so the second slot is headroom, not a
 *     path in use. The VERILATOR-only assertion at the bottom of this file
 *     is what fails if that ever stops being true; a depth-2 shadow FIFO for
 *     req_pc is the fix if a slower I-mem lands here.
 *
 * Buffer-room gating (overflow-safe): each outstanding read can push up to
 * 2 words, so issue reserves 2 slots per in-flight read.
 *   reserved     = count + 2*inflight  (always <= 8 by invariant)
 *   available    = 8 - reserved
 *   bus_issue    = inflight<2 && available_issue>=2 && (redirect || !pc_fault)
 *   fault_push   = pc_fault && available>=1 && !rsp_cap && inflight==0
 * A fault pushes exactly 1 word and launches no read -> its own >=1 gate,
 * and it is deferred a cycle when a response is landing the same cycle so
 * the buffer never pushes more than 2 words in one cycle.
 *
 * Timing rule -- `redirect` stays OFF the buffer write path: with a decode
 * predictor, `redirect` is no longer a pure flop output. pred_valid_i is
 * combinational in the buffer head (head_q -> read mux -> c_expand -> decode
 * -> PHT -> pred_target), so every signal `redirect` feeds inherits that
 * whole path. It is therefore kept out of `rready`, `do_rsp`, `fault_push`,
 * `push_cnt` (the buf_*_q write enables) and `buf_pop_cnt`, and used only on
 * `bus_issue` (one module output + the 2-bit inflight counter) and in the
 * next-state `if (redirect)` branch that already muxed pc_d. That is not a
 * relaxation of correctness -- see "Redirect" below for why each term was
 * redundant. Putting it back cost ~1.5 ns of setup slack at 50 MHz: 17 of
 * the 25 worst PnR paths ran head_q -> buf_fault_q/D through the decoder.
 *
 * Registers:
 *   - pc_q / req_pc_q : next fetch address / in-flight read address.
 *   - inflight_q      : outstanding reads (0..2).
 *   - stale_q         : in-flight reads still owed by the killed path.
 *   - redir_launched_q: a redirect cycle issued at the target; pc_q still
 *                       points at that word, so the next issue goes one
 *                       8-byte block on and pc steps past both.
 *   - buf_*_q[8]      : depth-8 32-bit-word instruction buffer.
 *   - head_q/tail_q/count_q : buffer FIFO pointers + occupancy.
 *
 * Redirect (branch_valid_i, highest priority): kills the buffer (count=0,
 * head=tail=0), points pc_q at the target, marks every read already in
 * flight stale, AND launches the first request at the target on the redirect
 * cycle itself (issue_addr = redirect_addr). The read launched on that cycle
 * belongs to the new path, so stale_q is armed from inflight_q - rsp_cap
 * rather than from inflight_d.
 *
 * pc_d on a redirect stays the plain `pc_d = redirect_addr` mux it has always
 * been: neither `launch` nor the target's 8-byte advance may sit on it.
 * redirect is combinational in the decode predictor (head_q -> c_expand ->
 * decode -> PHT -> pred_target), so anything AND-ed onto it lands at the very
 * end of the longest path in the design. Measured: putting `launch` and
 * `(redirect_addr & ~7) + 8` there cost ~1 ns of setup slack at 50 MHz on two
 * separate paths (predicted redirect, and execute's mispredict resolve).
 * redir_launched_q defers the advance one cycle so every 8-byte adder runs
 * off pc_q. The sequence of issued addresses is unchanged -- only where the
 * arithmetic sits.
 *
 * A push or a pop may still *happen* on the redirect cycle, and both are
 * harmless -- which is what lets `redirect` stay off those paths:
 *   - Push: the redirect branch forces head_d = tail_d = count_d = 0, so a
 *     stale word written at the old tail_q lands in a slot that is
 *     unreachable. Refilling restarts at slot 0 and the FIFO invariant is
 *     "a slot is written before it becomes readable", so the stale word is
 *     always overwritten before head can reach it.
 *   - Pop: buf_pop_cnt feeds head_d/count_d only inside the non-redirect
 *     else branch, which the redirect branch overrides wholesale.
 *   - Held response: rready no longer forces high on a redirect, so in
 *     principle a response landing while count_q >= 7 is left in the I-mem
 *     skid instead of being discarded -- inflight stays non-zero, stale_q is
 *     armed to match, and it is consumed and dropped one cycle later, by
 *     which time count_q is 0 and rready high again. Same net effect, one
 *     cycle later. In practice the case cannot arise: count_q >= 7 forces
 *     inflight == 0, so nothing is landing. Left described because the
 *     harmlessness, not the unreachability, is what licenses dropping the
 *     redirect term from rready.
 *
 * stall_i: decode back-pressure. While high the head is held (no pop); fetch
 * keeps running ahead into the buffer (up to 8 words), then stops issuing
 * once full. Redirects discard the buffered words.
 *
 * Naming: ports *_i/_o; internals no prefix; flops _q, next-state _d.
 */
module fetch_stage #(
    // Implemented I-mem size, in address bits (2**IMEM_ADDR_W bytes). A PC
    // outside that range cannot be fetched: the memory decodes only these
    // bits, so the access would alias back into real instructions and
    // execute them. Fetch raises an instruction access fault instead. Must
    // match the I-mem actually instantiated at the top level.
    parameter int IMEM_ADDR_W = 14,
    // Whether the EXECUTE redirect (mispredict / trap / mret / interrupt)
    // launches its read in the redirect cycle itself.
    //   0 = no (default). The read waits for pc_q, one cycle later, exactly as
    //       before 2026-08-31. Costs 1 cycle per execute redirect.
    //   1 = yes, the 2026-08-31 behaviour for both redirect sources.
    // The DECODE prediction always launches in-cycle regardless: its target is
    // src_pc + imm or the RAS top, i.e. PC- and flop-derived, so it does not
    // drag the register file onto the I-mem address pins.
    //
    // Why the split exists (2026-09-01 PnR, -3.812 ns / 42.0 MHz): with
    // branch_addr_i on the issue path, the eight worst paths in the design ran
    //   regfile DO -> forward -> operand -> JALR adder (rs1+imm) ->
    //   branch_target mux -> branch_addr_o -> issue_addr -> u_imem AD[*]
    // i.e. the register file's read output reaching the instruction memory's
    // address pins inside one cycle. Nothing else in the report ended at an
    // I-mem address pin, and nothing else can: issue_addr is the only path
    // there, and pc_q / pc_next_block are flops. Dropping branch_addr_i from
    // that mux removes the whole leg and leaves the predicted redirect -- the
    // common case at 88-99% accuracy -- still launching in-cycle.
    parameter int EXEC_REDIR_INCYCLE = 1
) (
    input wire clk_i,
    input wire rstn_i,

    // Reset vector boot address
    input wire [XLEN-1:0] boot_addr_i,

    // Downstream back-pressure and the execute-resolved redirect
    // (branch / jump / trap entry / mret), both wired at the CPU top.
    input wire            stall_i,
    input wire            branch_valid_i,
    input wire [XLEN-1:0] branch_addr_i,

    // Decode-time predicted redirect (branch predictor). Lower priority than
    // the execute redirect: a mispredict / trap / mret / interrupt resolved in
    // execute overrides whatever decode speculated. Routed through the same
    // kill path as branch_valid_i — kills the buffer, points pc_q at the
    // predicted target, marks any in-flight read stale. The predicted
    // branch itself sits at the buffer head this cycle and is consumed into
    // decode's de_q combinationally, so killing the buffer only discards the
    // younger wrong-path entries behind it (same net effect as an execute
    // redirect, one cycle earlier).
    input wire            pred_valid_i,
    input wire [XLEN-1:0] pred_addr_i,

    // Native 64-bit instruction-memory interface (read-only).
    output ifetch_req_t imem_req_o,
    input  ifetch_rsp_t imem_rsp_i,

    // Push-time branch-predictor lookup. Fetch presents the PC stamp of each
    // word it is about to push; the predictor answers with a direction bit for
    // each halfword slot of each word plus the GHR snapshot they were read
    // with, and fetch stores all of it in the buffer entry. That takes the PHT
    // array read off decode's redirect path -- see BP_PUSH_LOOKUP.
    output wire bp_push_req_t bp_push_o,
    input  wire bp_push_rsp_t bp_push_i,

    // F/D pipeline register outputs (buffer head): each pipeline stage
    // exposes the PC it is treating, the instruction word, and a valid
    // (stage sigil `fe_`).
    output wire [XLEN-1:0] fe_instr_o,  // F/D instruction word (32-bit)
    output wire [XLEN-1:0] fe_pc_o,     // F/D instruction PC (exact)
    output wire            fe_valid_o,  // F/D valid (held level)
    output wire            fe_fault_o,  // F/D entry is an access fault, not an instruction

    // Prediction carried in the head entry (BP_PUSH_LOOKUP=1). One direction
    // bit per halfword slot -- decode selects by src_pc[1] -- plus the GHR
    // snapshot they were read with, from which decode recomputes the gshare
    // index for the D/E snapshot.
    output wire                fe_pred_lo_o,  // head, pc[1]=0
    output wire                fe_pred_hi_o,  // head, pc[1]=1
    output wire [BP_GHR_W-1:0] fe_ghr_o,      // head, GHR at push

    // Buffer head+1 read port (same-cycle RVC spanning stitch). Exposes the
    // word right behind the head so decode can stitch a 32-bit instr sitting
    // at a 2-byte-aligned branch target (offset 2 / 6) without a bubble: the
    // stitch consumes head[31:16] + head+1[15:0] in the cycle the target word
    // is seen. Gated by fe_next_valid_o (count>=2); decode drives fe_pop2_i
    // only then, so pop-2 <= count (no underflow).
    output wire [    XLEN-1:0] fe_next_instr_o,    // buffer[head+1] word
    output wire [    XLEN-1:0] fe_next_pc_o,       // buffer[head+1] PC
    output wire                fe_next_valid_o,    // count >= 2
    output wire                fe_next_fault_o,    // buffer[head+1] fault flag
    // head+1's high-half prediction + GHR snapshot. The same-cycle target
    // stitch re-stashes head+1's upper half into decode's hold buffer, so the
    // prediction for that halfword has to travel with it.
    output wire                fe_next_pred_hi_o,
    output wire [BP_GHR_W-1:0] fe_next_ghr_o,
    input  wire                fe_pop2_i           // decode: pop 2 this cycle (target stitch)
);

    // -----------------------------------------------------------------
    // State
    // -----------------------------------------------------------------
    localparam int BUF_DEPTH = 8;

    logic [XLEN-1:0] pc_q, pc_d;  // next fetch address
    logic [XLEN-1:0] req_pc_q, req_pc_d;  // address of the in-flight read
    logic [1:0] inflight_q, inflight_d;  // outstanding reads (0..2)
    logic [1:0] stale_q, stale_d;  // in-flight reads owed by the killed path
    logic redir_launched_q, redir_launched_d;  // redirect cycle already issued at the target

    // Depth-8 32-bit-word instruction buffer (FIFO).
    logic [    XLEN-1:0] buf_instr_q  [BUF_DEPTH];
    logic [    XLEN-1:0] buf_pc_q     [BUF_DEPTH];
    logic                buf_fault_q  [BUF_DEPTH];
    // Push-time prediction, carried per entry: one direction bit per halfword
    // slot + the GHR snapshot they were read with. 9 bits x 8 entries = 72
    // flops. Unused (and pruned, along with the predictor's push read port)
    // when the CPU is built with BP_PUSH_LOOKUP=0.
    logic                buf_pred_lo_q[BUF_DEPTH];
    logic                buf_pred_hi_q[BUF_DEPTH];
    logic [BP_GHR_W-1:0] buf_ghr_q    [BUF_DEPTH];
    logic [2:0] head_q, head_d;
    logic [2:0] tail_q, tail_d;
    logic [3:0] count_q, count_d;  // 0..BUF_DEPTH

    // -----------------------------------------------------------------
    // Native interface + control wires
    // -----------------------------------------------------------------
    // PC outside the implemented I-mem: no bus request, synthesise a fault
    // FIFO entry instead (the memory would alias an out-of-range read back
    // into the image and execute it).
    wire pc_fault = |pc_q[XLEN-1:IMEM_ADDR_W];

    // Read response accepted this cycle.
    wire rsp_cap = imem_rsp_i.rvalid && imem_req_o.rready;

    // Redirect priority: execute (trap/mret/mispredict, branch_valid_i) is
    // highest; the decode prediction is subordinate and suppressed when an
    // execute redirect fires the same cycle. Both share one kill path.
    wire pred_redirect = pred_valid_i & ~branch_valid_i;
    wire redirect = branch_valid_i | pred_redirect;
    wire [XLEN-1:0] redirect_addr = branch_valid_i ? branch_addr_i : pred_addr_i;

    // Buffer-room accounting. reserved = count + 2*inflight <= 8 (invariant
    // maintained by the issue gate), so available is non-negative.
    wire [2:0] twice_inflight = {inflight_q, 1'b0};  // inflight * 2
    wire [4:0] reserved = {1'b0, count_q} + {2'b0, twice_inflight};
    wire [4:0] available = 5'd8 - reserved;  // >= 0 by invariant

    // Same-cycle redirect issue: the request launched on a redirect cycle
    // goes to the redirect target, not to pc_q. Without this the redirect
    // cycle launches nothing and the first request at the target waits for
    // the next cycle -- one dead cycle on every taken branch, predicted or
    // mispredicted. issue_addr therefore replaces pc_q on the issue path.
    //
    // issue_addr feeds ONLY imem_req_o.addr -- never bus_issue. An
    // |issue_addr[31:IMEM_ADDR_W]| range check here would be an 18-input
    // OR-reduce hanging off the decode predictor's src_pc+imm adder, and it
    // would then reach pc_q through launch: measured -2.292 ns at 50 MHz
    // (44.9 MHz), 7.05 ns of tail on the last-arriving decode signal. The
    // check moved to the response side instead (rsp_fault), where the
    // address is req_pc_q -- a flop, and therefore free.
    // Both derived from pc_q, a flop -- deliberately NOT from redirect_addr.
    wire [XLEN-1:0] pc_next_block = (pc_q & ~32'h7) + 32'd8;
    wire [XLEN-1:0] pc_next_block2 = (pc_q & ~32'h7) + 32'd16;

    // redir_launched_q: the redirect cycle already issued the read at the
    // target and left pc_q pointing AT it, so this cycle issues one block on.
    // This is what keeps the 8-byte advance off the predictor's target: the
    // adder runs on pc_q instead of on redirect_addr. The stream of issued
    // addresses is identical to advancing pc_q on the redirect cycle itself.
    // In-cycle launch address. Only a source whose target is PC-derived may
    // appear here: pred_addr_i is src_pc+imm or the RAS top, both off the
    // register file. branch_addr_i is NOT, and putting it here is what took
    // the 2026-09-01 run to -3.812 ns (see EXEC_REDIR_INCYCLE).
    wire incycle_launch = (EXEC_REDIR_INCYCLE != 0) ? redirect : pred_redirect;
    wire [XLEN-1:0] incycle_addr = (EXEC_REDIR_INCYCLE != 0) ? redirect_addr : pred_addr_i;
    wire [XLEN-1:0]
        issue_addr = incycle_launch ? incycle_addr : redir_launched_q ? pc_next_block : pc_q;

    // Room on a redirect cycle is measured against the buffer the redirect
    // is about to zero, not the one still holding wrong-path words: count
    // becomes 0 this cycle, and reads already in flight are marked stale and
    // will push nothing.
    wire [4:0] available_issue = redirect ? (5'd8 - {2'b0, twice_inflight}) : available;

    // Issue a fetch when idle (inflight<2) and the buffer has room for the
    // response (2 words reserved per in-flight read). Deliberately NOT gated
    // on stale_q: issuing at the new target while the killed path's responses
    // are still in flight is what removes the drain bubble.
    //
    // The in-range term is pc_fault, i.e. pc_q -- a flop. On a redirect cycle
    // it is dropped entirely: a redirect to an unfetchable target DOES launch
    // a bus read, whose aliased data is then discarded and turned into a
    // fault entry when it lands (rsp_fault). Speculating a read that is
    // thrown away costs nothing (the I-mem is read-only and one cycle deep)
    // and keeps the whole range check off the predictor's adder.
    // An execute redirect that does not launch in-cycle must not launch at
    // pc_q either: pc_q still holds the wrong-path address until the next
    // edge. So the issue is suppressed for that one cycle and resumes from
    // pc_q (= the target) next cycle. branch_valid_i is a module input driven
    // by a flop-fed comparison in execute, so this term costs nothing on the
    // address path -- it gates the enable, not the address.
    wire exec_redir_hold = (EXEC_REDIR_INCYCLE == 0) && branch_valid_i;
    wire bus_issue = (inflight_q < 2'd2) && (available_issue >= 5'd2) &&
        (incycle_launch || !pc_fault) && !exec_redir_hold;
    // A fault pushes 1 word and launches no read -> its own >=1 gate. Hold it
    // off a cycle when a response is landing the same cycle so the buffer
    // never pushes more than 2 words in one cycle, and while any read is
    // still in flight so a fault entry can never overtake an older response
    // (an out-of-range redirect launches one, and its rsp_fault entry carries
    // the earlier PC -- it has to reach decode first).
    wire fault_push = pc_fault && (available >= 5'd1) && !rsp_cap && (inflight_q == 2'd0);

    // Read launch accepted this cycle.
    wire launch = bus_issue && imem_rsp_i.ready;

    always_comb begin
        // Accept read data when a stale response is owed (discard it) or
        // whenever the buffer has room for the two words a response can
        // push. Depends only on registers (stale_q, count_q) + nothing
        // from the slave's rvalid -> no combinational loop through the I-mem,
        // and deliberately NOT on redirect (see the timing rule in the header:
        // pred_valid_i drags the whole decoder onto anything redirect feeds).
        imem_req_o.rready = (stale_q != 2'd0) || (count_q <= 4'd6);

        // Issue a fetch when idle (inflight<2) and the buffer has room for
        // the response (2 words reserved per in-flight read). On a redirect
        // cycle the address is the redirect target (see issue_addr) and may
        // be out of range; rsp_fault converts the response.
        imem_req_o.valid  = bus_issue;
        imem_req_o.addr   = issue_addr;
    end

    // -----------------------------------------------------------------
    // 64-bit response split -> 32-bit buffer pushes (0, 1, or 2 words/cycle)
    // -----------------------------------------------------------------
    // A response owed to the killed path (stale_q != 0) is accepted and
    // dropped: no buffer push, no PC stamp read.
    wire do_rsp = rsp_cap && (stale_q == 2'd0);
    // The in-flight read was launched at an address outside the implemented
    // I-mem (only a redirect cycle can do that -- see bus_issue). The memory
    // decodes IMEM_ADDR_W bits, so its answer is an alias of real
    // instructions: drop it and push one fault entry stamped with the exact
    // faulting PC instead. req_pc_q is a flop, so this check costs nothing.
    wire rsp_fault = |req_pc_q[XLEN-1:IMEM_ADDR_W];
    wire do_rsp_data = do_rsp && !rsp_fault;  // push instruction words
    wire do_rsp_fault = do_rsp && rsp_fault;  // push one fault entry
    wire rsp_two = ~req_pc_q[2];  // target in low half -> push both words

    wire [XLEN-1:0] rsp_low_pc = req_pc_q;
    wire [XLEN-1:0] rsp_high_pc = (req_pc_q & ~32'h7) + 32'd4;
    wire [31:0] rsp_low_word = imem_rsp_i.rdata[31:0];
    wire [31:0] rsp_high_word = imem_rsp_i.rdata[63:32];

    // First push: the half containing the fetch PC (low if [2]==0, high if
    // [2]==1). The PC stamp is ALWAYS req_pc_q (rsp_low_pc): it carries the
    // unaligned target bits ([1:0] for the low half, [1] for the high half)
    // so decode's fe_pc[1] selects the right halfword. Using rsp_high_pc
    // (= base+4) for the [2]==1 case would clear bit[1] and make decode pick
    // the wrong halfword on a 2-aligned redirect into the high half
    // (e.g. jalr to 0xee stamped 0xec). A fault fills this slot instead of an
    // instruction in two cases: an out-of-range response (pc=req_pc_q, the
    // exact faulting target) and the no-launch fault push (pc=pc_q).
    wire [XLEN-1:0] push0_pc = do_rsp ? rsp_low_pc : pc_q;
    wire [31:0] push0_word = do_rsp_data ? (rsp_two ? rsp_low_word : rsp_high_word) : 32'd0;
    wire push0_fault = !do_rsp_data;

    // Second push: the high half, only when pushing both words (data
    // response only -- a fault response pushes exactly one entry).
    wire [XLEN-1:0] push1_pc = rsp_high_pc;
    wire [31:0] push1_word = rsp_high_word;
    wire push1_fault = 1'b0;

    wire [1:0] push_cnt = do_rsp_data ?
        (rsp_two ? 2'd2 : 2'd1) : (do_rsp_fault || fault_push) ? 2'd1 : 2'd0;

    // Push-time predictor lookup. Both addresses are combinational off flops
    // (req_pc_q via rsp_low_pc / rsp_high_pc, or pc_q on a fault push), so the
    // PHT read they drive has the whole cycle and lands in a flop -- which is
    // the entire point of doing it here rather than at decode. The request is
    // unconditional: a lookup for a word that is not pushed is simply
    // discarded, and gating it would only add logic to the address path.
    assign bp_push_o.pc0 = push0_pc;
    assign bp_push_o.pc1 = push1_pc;

    // -----------------------------------------------------------------
    // Buffer pop: decode consumes the head when it is not back-pressuring.
    // A same-cycle target-span stitch pops 2 (head + head+1, the stitch word
    // + its consumed low half); fe_pop2_i is only asserted when
    // fe_next_valid_o (count>=2), so pop-2 <= count — no underflow.
    // No !redirect term: buf_pop_cnt is read only inside the non-redirect
    // else branch below, which the redirect branch overrides wholesale, so
    // the term was dead logic on the pred_valid_i path.
    // -----------------------------------------------------------------
    wire [1:0] buf_pop_cnt = (count_q != 4'd0 && !stall_i) ? (fe_pop2_i ? 2'd2 : 2'd1) : 2'd0;

    // -----------------------------------------------------------------
    // Next-state
    // -----------------------------------------------------------------
    always_comb begin
        // Defaults: hold.
        pc_d       = pc_q;
        req_pc_d   = req_pc_q;
        inflight_d = inflight_q;
        stale_d    = stale_q;
        head_d     = head_q;
        tail_d     = tail_q;
        count_d    = count_q;

        if (redirect) begin
            // ---- Redirect (highest priority) ----
            // Execute (trap/mret/mispredict) wins over a decode prediction;
            // redirect_addr selects accordingly. Kills the buffer, points pc_q
            // at the target, marks every in-flight read stale. The
            // predicted branch at the head is consumed into decode's de_q this
            // cycle (combinational read), so zeroing the buffer only discards
            // the younger wrong-path entries behind it.
            pc_d    = redirect_addr;
            head_d  = 3'd0;  // kill the buffer
            tail_d  = 3'd0;
            count_d = 4'd0;
            // A push may still land at the old tail_q and a pop may still be
            // computed: both are harmless because head/tail/count are zeroed
            // here (see the header). inflight and stale are updated below.
        end else begin
            // ---- Normal: issue + split-push + buffer drain ----
            // The word at pc_q is already in flight when redir_launched_q,
            // so pc steps past it -- twice if this cycle launched the block
            // after it as well. Every one of these adders runs on pc_q.
            if (redir_launched_q) pc_d = launch ? pc_next_block2 : pc_next_block;
            else if (launch) pc_d = pc_next_block;
            if (fault_push) begin
                // A fault PC advances past the unfetchable 8-byte word.
                pc_d = pc_next_block;
            end
            // Buffer FIFO advance. buf_pop_cnt is 0/1/2 (3-bit wrap on head).
            head_d  = head_q + buf_pop_cnt;
            tail_d  = tail_q + push_cnt;  // 0/1/2, 3-bit wrap
            count_d = count_q + push_cnt - buf_pop_cnt;
        end

        // inflight tracks outstanding reads: +1 on launch, -1 on rsp_cap.
        // Both may happen the same cycle (a response retires while a new
        // read issues) -> net unchanged, so apply incrementally.
        if (launch) inflight_d = inflight_d + 2'd1;
        if (rsp_cap) inflight_d = inflight_d - 2'd1;

        // The in-flight read address, wherever the launch came from. On a
        // redirect cycle issue_addr is the target; otherwise it is pc_q.
        if (launch) req_pc_d = issue_addr;

        // A redirect-cycle launch defers the pc advance to the next cycle.
        // Only an in-cycle launch does that: when the execute redirect is held
        // (EXEC_REDIR_INCYCLE=0) pc_q becomes the target and the next cycle
        // issues AT it, not one block past it.
        redir_launched_d = incycle_launch && launch;

        // stale tracks how many outstanding responses still belong to the
        // killed path. An accepted response retires the oldest one; a
        // redirect re-marks everything still outstanding. The redirect
        // assignment comes last so it wins over the decrement.
        if (rsp_cap && (stale_q != 2'd0)) stale_d = stale_q - 2'd1;
        // NOTE: inflight_q - rsp_cap, NOT inflight_d: the read launched on
        // this very cycle belongs to the NEW path and must not be discarded.
        if (redirect) stale_d = inflight_q - {1'b0, rsp_cap};
    end

    // -----------------------------------------------------------------
    // Sequential
    // -----------------------------------------------------------------
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            pc_q             <= boot_addr_i;
            req_pc_q         <= '0;
            inflight_q       <= 2'd0;
            stale_q          <= 2'd0;
            redir_launched_q <= 1'b0;
            head_q           <= 3'd0;
            tail_q           <= 3'd0;
            count_q          <= 4'd0;
        end else begin
            pc_q             <= pc_d;
            req_pc_q         <= req_pc_d;
            inflight_q       <= inflight_d;
            stale_q          <= stale_d;
            redir_launched_q <= redir_launched_d;
            head_q           <= head_d;
            tail_q           <= tail_d;
            count_q          <= count_d;
            // Buffer writes: push0 at tail, push1 at tail+1 (3-bit wrap).
            if (push_cnt >= 2'd1) begin
                buf_instr_q[tail_q]   <= push0_word;
                buf_pc_q[tail_q]      <= push0_pc;
                buf_fault_q[tail_q]   <= push0_fault;
                buf_pred_lo_q[tail_q] <= bp_push_i.pred0_lo;
                buf_pred_hi_q[tail_q] <= bp_push_i.pred0_hi;
                buf_ghr_q[tail_q]     <= bp_push_i.ghr;
            end
            if (push_cnt >= 2'd2) begin
                buf_instr_q[tail_q+3'd1]   <= push1_word;
                buf_pc_q[tail_q+3'd1]      <= push1_pc;
                buf_fault_q[tail_q+3'd1]   <= push1_fault;
                buf_pred_lo_q[tail_q+3'd1] <= bp_push_i.pred1_lo;
                buf_pred_hi_q[tail_q+3'd1] <= bp_push_i.pred1_hi;
                buf_ghr_q[tail_q+3'd1]     <= bp_push_i.ghr;
            end
        end
    end

    // -----------------------------------------------------------------
    // F/D outputs (buffer head). fe_valid is a held level (head stays until
    // decode consumes it via !stall_i, or a redirect kills the buffer).
    // -----------------------------------------------------------------
    assign fe_instr_o   = buf_instr_q[head_q];
    assign fe_pc_o      = buf_pc_q[head_q];
    assign fe_valid_o   = (count_q != 4'd0);
    assign fe_fault_o   = buf_fault_q[head_q];
    assign fe_pred_lo_o = buf_pred_lo_q[head_q];
    assign fe_pred_hi_o = buf_pred_hi_q[head_q];
    assign fe_ghr_o     = buf_ghr_q[head_q];

    // Buffer head+1 read port (same-cycle target-span stitch). head_next is a
    // 3-bit wrap, always a valid 0..7 index even when count<2; decode gates on
    // fe_next_valid_o. A redirect zeros count -> both fe_valid_o and
    // fe_next_valid_o drop, so no extra kill logic.
    wire [2:0] head_next = head_q + 3'd1;
    assign fe_next_instr_o   = buf_instr_q[head_next];
    assign fe_next_pc_o      = buf_pc_q[head_next];
    assign fe_next_fault_o   = buf_fault_q[head_next];
    assign fe_next_valid_o   = (count_q >= 4'd2);
    assign fe_next_pred_hi_o = buf_pred_hi_q[head_next];
    assign fe_next_ghr_o     = buf_ghr_q[head_next];

`ifdef VERILATOR
    // The split assumes the I-mem aligns the read address down to 8 bytes
    // (it decodes addr[ADDR_W-1:3]); req_pc_q[2] then selects which 32-bit
    // half the fetch PC falls in. The buffer invariant count + 2*inflight
    // <= 8 is what makes the issue gate overflow-safe.
    initial
        assert (BUF_DEPTH == 8)
        else $fatal(1, "BUF_DEPTH width assumptions broken");

    // req_pc_q is ONE register while inflight_q is allowed to reach 2, so
    // the split PC stamp is only correct while a launch never overwrites the
    // address of a read whose response has not landed yet. That holds on a
    // 1-cycle BSRAM (see the req_pc_q paragraph in the header) and it is an
    // assumption about the slave, not something this module enforces -- a
    // variable-latency I-mem breaks it, and the failure is silent: the older
    // response gets split at the younger read's PC, so fe_pc, branch targets
    // and link addresses go wrong with no error signal anywhere. Measured on
    // the BSRAM this never fires (inflight_q stays in {0,1} on the Harvard
    // oracle, quicksort, bp_pred and CoreMark). Fix if it ever does: a
    // depth-2 shadow FIFO for req_pc, not a wider register.
    always_ff @(posedge clk_i) begin
        if (rstn_i) begin
            assert (!(launch && (inflight_q != 2'd0) && !rsp_cap))
            else
                $fatal(
                    1,
                    "fetch: req_pc_q overwritten with a response still owed (inflight=%0d, issue=%08h, req_pc_q=%08h)"
                        ,
                    inflight_q,
                    issue_addr,
                    req_pc_q
                );

            // Buffer-room invariant: reserved = count + 2*inflight <= 8.
            // `available` is unsigned, so a breach would wrap it to a large
            // value and let the issue gate overflow the buffer instead of
            // blocking.
            assert (({1'b0, count_q} + {2'b0, twice_inflight}) <= 5'd8)
            else $fatal(1, "fetch: reserved > 8 (count=%0d inflight=%0d)", count_q, inflight_q);
        end
    end
`endif

endmodule

`resetall

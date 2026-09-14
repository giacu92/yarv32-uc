`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Branch predictor — gshare 2-bit PHT + 7-bit GHR + 8-entry RAS.
 *
 * Phase 1 of the branch-prediction add: direction prediction for conditional
 * branches (gshare PHT) + return-address prediction for JALR returns (RAS).
 * Unconditional JAL/c.j/c.jal and conditional-branch *targets* are computed
 * directly in decode (pc+imm, off the regfile path), so there is no BTB here;
 * a BTB/indirect-target cache for non-return JALR is a deferred Phase 2.
 *
 * Prediction happens at DECODE (the buffer head), one stage earlier than
 * execute's resolve, so a correct prediction lets execute skip its redirect
 * entirely. Execute remains the golden resolver: it compares the carried
 * prediction against the resolved outcome and redirects only on mispredict.
 *
 * Timing rule: the lookup depends ONLY on the PC and the GHR — never on
 * register data — so it sits off the `regfile -> forward mux -> branch
 * compare -> PC redirect` critical path. The PHT (128x2b), the GHR (7b) and
 * the RAS (8x32b) are all flops read combinationally, so no cycle is added to
 * the frontend (note §11).
 *
 * Training happens ONLY at resolve (the execute training port), never on
 * speculative outcomes. In-order single-issue means a squashed instruction
 * never resolves, so there is no wrong-path contamination of the PHT/GHR/RAS.
 * The PHT update is indexed by the gshare snapshot carried in de_t
 * (pred_pht_index = pc[IDX_W:1]^ghr at lookup time), not the live GHR — an older
 * branch may have shifted the GHR between this branch's decode and its
 * resolve, and the update must use the history the branch was predicted with.
 *
 * GHR (global history register): 7 bits, shifted left on each resolved
 * conditional branch (newest outcome in bit 0). Used to xor the PHT index
 * (gshare) so correlated branches share state usefully. Updated only for
 * conditional branches (note Appendix A).
 *
 * RAS (return address stack): 8 entries, circular with a saturating count.
 * Pushed at the resolve of a CALL (JAL/JALR with rd in {x1,x5}), popped at the
 * resolve of a RETURN (JALR with rs1 in {x1,x5}, rd=x0). The top is read at
 * decode to predict a return's target. Trained at resolve, not at predict, so
 * a wrong-path call never pushes — the RAS is not corrupted by speculation.
 * The decode read is of ras_q, a flop output, so a resolve writing the array
 * in the same cycle cannot disturb it -- the read simply sees the pre-write
 * state. The one visible consequence is that a return decoded in the very
 * cycle its own call resolves would read the top from before that push; a
 * redirect costs at least two cycles of refill, so no call and its matching
 * return are ever that close.
 *
 * Naming: ports *_i/_o; internals no prefix; flops _q/_d.
 */
module branch_predictor (
    input wire clk_i,
    input wire rstn_i,

    // -------------------------------------------------------------------
    // Lookup port (decode, combinational). Decode presents the PC of the
    // control-flow instruction at the buffer head; the predictor returns the
    // gshare PHT direction bit, the RAS top + valid, and the gshare index
    // snapshot to carry in de_t. All four are returned unconditionally --
    // decode knows the instruction kind and selects -- so the request needs
    // no kind bits (the one extra field is the sim-only RAS-counter event).
    // -------------------------------------------------------------------
    input  wire bp_lookup_req_t lookup_req_i,
    output wire bp_lookup_rsp_t lookup_rsp_o,

    // -------------------------------------------------------------------
    // Push-time lookup port (fetch, combinational). Used when the CPU is
    // built with BP_PUSH_LOOKUP=1: fetch reads the direction bits as it
    // pushes words into the instruction buffer and carries them in the
    // entry, so the array read is a flop-to-flop path with a whole cycle
    // instead of a term on decode's redirect path. See the lookup below for
    // what that costs in accuracy (a slightly stale GHR).
    // -------------------------------------------------------------------
    input  wire bp_push_req_t push_req_i,
    output wire bp_push_rsp_t push_rsp_o,

    // -------------------------------------------------------------------
    // Training port (execute, at resolve). One resolved control-flow
    // instruction per cycle (in-order single-issue). Mutually exclusive
    // kind bits; exactly the PHT/GHR/RAS state for that kind updates.
    // -------------------------------------------------------------------
    input wire bp_train_t train_i
);

    // Field aliases — keep the body reading the same names as the old per-
    // wire port list, so the lookup/train logic below is unchanged.
    wire [        XLEN-1:0] lookup_pc_i = lookup_req_i.pc;
    wire                    train_valid_i = train_i.valid;
    wire                    train_cond_i = train_i.cond;
    wire                    train_call_i = train_i.call;
    wire                    train_return_i = train_i.ret;
    wire                    train_taken_i = train_i.taken;
    wire [BP_PHT_IDX_W-1:0] train_pht_index_i = train_i.pht_index;
    wire [        XLEN-1:0] train_push_pc_i = train_i.push_pc;

    // -------------------------------------------------------------------
    // Storage
    // -------------------------------------------------------------------
    // Geometry lives in rv32_pkg because the de_t / bp_train_t index fields
    // must be the same width as this table's index (see BP_PHT_DEPTH there
    // for the trace-driven sizing measurements).
    localparam int unsigned PHT_DEPTH = BP_PHT_DEPTH;
    localparam int unsigned IDX_W = BP_PHT_IDX_W;
    localparam int unsigned GHR_W = BP_GHR_W;
    localparam int unsigned RAS_DEPTH = BP_RAS_DEPTH;
    localparam int unsigned RAS_PTR_W = $clog2(RAS_DEPTH);

    // STORAGE. This note used to state flatly that the table is necessarily a
    // flop array behind a LUT mux -- 128x2 = 256 FF -- because both reads are
    // ASYNCHRONOUS (the decode lookup is combinational, and the train-side
    // read-modify-write reads a second address in the cycle it writes) while
    // Gowin's LUT-RAM/SSRAM is synchronous-read only (SUG949E 8.2, the same
    // limit that keeps the regfile on BSRAM -- see reg_file.sv).
    //
    // AT BP_PHT_DEPTH=512 THAT IS NO LONGER WHAT GETS BUILT. The 2026-09-14
    // PnR report places u_cpu/u_bp/pht_q_pht_q_0_0_s on a BSRAM site, driven
    // by ADA[1:9] address pins from the registered de_q.pred_pht_index, while
    // bp_train.pht_index[*] still fans out to 514 loads and u_bp/n1326_3 to
    // 513. So synthesis replicated the array and put at least one copy in
    // block RAM. The plausible reading: the train RMW -- a registered index,
    // read-then-write at one address -- maps onto a read-before-write BSRAM
    // port, and the four asynchronous push reads stay distributed.
    //
    // THAT READING IS INFERENCE FROM A TIMING REPORT, NOT VERIFIED. It
    // matters because a BSRAM read output is registered, whereas the RTL
    // below needs pht_train_ctr combinationally in the same cycle it writes.
    // If synthesis got that wrong, training reads a stale counter: prediction
    // accuracy degrades, nothing traps, and SIMULATION CANNOT SEE IT, because
    // sim elaborates this RTL and not the netlist. That is the same class as
    // the 2026-09-01 board reboot loop. Confirm against the .vg netlist and
    // the synthesis resource table before trusting a board run.
    //
    // Consequences if it IS block RAM: depth is no longer priced like a wide
    // mux, so 1024 becomes worth a run; and the 6.1 ns read measured at 512
    // entries on 2026-08-31, which forced the resize to 128, was a
    // measurement of the FLOP form and does not describe this build.
    //
    // This array carried `(* ram_style = "distributed" *)` and
    // `(* syn_ramstyle = "distributed" *)` until 2026-08-31. Both were dead:
    // ram_style is the Vivado spelling GowinSynthesis ignores outright, and
    // "distributed" is not one of its accepted values either (they carry the
    // _ram suffix -- block_ram / distributed_ram), so it answered
    // "EX0200: Property syn_ramstyle set invalid for pht_q" and inferred
    // flops regardless. Removing both attributes changed neither the resource
    // report nor PnR, which is the expected result of deleting a rejected
    // property -- and is NOT evidence about what it infers at depth 512.
    //
    // No reset port: the weak-not-taken initialisation is an `initial` block,
    // which an FPGA flop honours as its power-up value. Harmless by
    // construction either way -- the PHT is a hint and execute is the golden
    // resolver, so even an uninitialised table costs at most a few cold-start
    // mispredicts and can never produce a wrong retire.
    logic   [          1:0] pht_q                                              [PHT_DEPTH];
    logic   [    GHR_W-1:0] ghr_q;
    logic   [     XLEN-1:0] ras_q                                              [RAS_DEPTH];
    logic   [RAS_PTR_W-1:0] ras_ptr_q;  // next push slot (wraps mod RAS_DEPTH)
    logic   [  RAS_PTR_W:0] ras_cnt_q;  // 0..RAS_DEPTH, saturating

    // Weak not-taken power-up value for every entry. The loop variable is
    // declared at module scope on purpose: GowinSynthesis does not elaborate a
    // SystemVerilog-style `for (int i = ...)` declared inside an initial block
    // and warns "EX3780: Using initial value of 'i' since it is never
    // assigned", which leaves the table with no initial value at all. A
    // Verilog-2001 loop variable elaborates there and in Verilator alike.
    integer                 pht_init_idx;
    initial begin
        for (pht_init_idx = 0; pht_init_idx < PHT_DEPTH; pht_init_idx = pht_init_idx + 1) begin
            pht_q[pht_init_idx] = 2'b01;  // weak not-taken
        end
    end

    // -------------------------------------------------------------------
    // Lookup (combinational, PC + GHR only)
    // -------------------------------------------------------------------
    // gshare index. pc[IDX_W:1], not pc[IDX_W+1:2]: IALIGN is 16 with the C
    // extension, so pc[1] is a real address bit — dropping it folds two
    // compressed branches 2 bytes apart onto one PHT entry. pc[IDX_W:1]
    // covers the whole table without that aliasing.
    wire [    IDX_W-1:0] lookup_index = lookup_pc_i[IDX_W:1] ^ ghr_q;
    // Top = most-recently-pushed entry = slot (ptr - 1). When empty the top is
    // a don't-care (ras_valid=0 gates its use in decode).
    wire [RAS_PTR_W-1:0] ras_top_idx = ras_ptr_q - RAS_PTR_W'(1);
    wire                 ras_valid = (ras_cnt_q != '0);

    assign lookup_rsp_o.pht_index = lookup_index;
    assign lookup_rsp_o.pht_taken = pht_q[lookup_index][1];
    assign lookup_rsp_o.ras_valid = ras_valid;
    assign lookup_rsp_o.ras_top   = ras_q[ras_top_idx];

    // -------------------------------------------------------------------
    // Push-time lookup (combinational, PC + GHR only)
    // -------------------------------------------------------------------
    // Same gshare function as above, evaluated one or two pipeline stages
    // earlier, for both halfword slots of each pushed word.
    //
    // The two indices for a word are an ALIGNED PAIR, which is why reading
    // both costs one extra entry and not one extra port. With
    //   index = pc[IDX_W:1] ^ ghr
    // the only difference between the two slots is pc[1], the index's LSB
    // source, so
    //   index(pc[1]=0) = {u,  ghr[0]}
    //   index(pc[1]=1) = {u, ~ghr[0]}   where u = pc[IDX_W:2] ^ ghr[IDX_W-1:1]
    // One aligned pair {u,0} / {u,1} covers both, and ghr[0] says which of
    // the two is the pc[1]=0 slot.
    //
    // ACCURACY NOTE, and it is the whole trade of this option: the bits are
    // read with the GHR as it stands at PUSH time, while the decode-time
    // lookup reads it as it stands at DECODE. A conditional branch resolving
    // in between shifts the GHR, so a prediction read at push can be indexed
    // with history one or two outcomes out of date. Nothing is incorrect --
    // de_t carries the index the bits were read at, so training updates the
    // same entry the lookup used, and execute is still the golden resolver --
    // but gshare's whole point is indexing with current history, so expect
    // some accuracy loss. The measured numbers are in sim/bench_ipc_ab.md.
    localparam int unsigned PAIR_W = IDX_W - 1;

    wire [PAIR_W-1:0] push_u0 = push_req_i.pc0[IDX_W:2] ^ ghr_q[IDX_W-1:1];
    wire [PAIR_W-1:0] push_u1 = push_req_i.pc1[IDX_W:2] ^ ghr_q[IDX_W-1:1];

    wire [       1:0] push0_ctr0 = pht_q[{push_u0, 1'b0}];
    wire [       1:0] push0_ctr1 = pht_q[{push_u0, 1'b1}];
    wire [       1:0] push1_ctr0 = pht_q[{push_u1, 1'b0}];
    wire [       1:0] push1_ctr1 = pht_q[{push_u1, 1'b1}];

    assign push_rsp_o.ghr      = ghr_q;
    assign push_rsp_o.pred0_lo = ghr_q[0] ? push0_ctr1[1] : push0_ctr0[1];
    assign push_rsp_o.pred0_hi = ghr_q[0] ? push0_ctr0[1] : push0_ctr1[1];
    assign push_rsp_o.pred1_lo = ghr_q[0] ? push1_ctr1[1] : push1_ctr0[1];
    assign push_rsp_o.pred1_hi = ghr_q[0] ? push1_ctr0[1] : push1_ctr1[1];

    // -------------------------------------------------------------------
    // 2-bit saturating counter update
    // -------------------------------------------------------------------
    function automatic logic [1:0] sat_update(input logic [1:0] c, input logic t);
        logic [1:0] r;
        if (t) r = (c == 2'b11) ? 2'b11 : (c + 2'b01);
        else r = (c == 2'b00) ? 2'b00 : (c - 2'b01);
        return r;
    endfunction

    // -------------------------------------------------------------------
    // Next-state
    // -------------------------------------------------------------------
    logic [    GHR_W-1:0] ghr_d;
    logic [     XLEN-1:0] ras_d     [RAS_DEPTH];
    logic [RAS_PTR_W-1:0] ras_ptr_d;
    logic [  RAS_PTR_W:0] ras_cnt_d;

    always_comb begin
        // Default: hold.
        for (int i = 0; i < RAS_DEPTH; i++) ras_d[i] = ras_q[i];
        ghr_d     = ghr_q;
        ras_ptr_d = ras_ptr_q;
        ras_cnt_d = ras_cnt_q;

        if (train_valid_i) begin
            // GHR: conditional only, shifted with the resolved outcome. The
            // PHT write that goes with it is in its own always_ff below.
            if (train_cond_i) ghr_d = {ghr_q[GHR_W-2:0], train_taken_i};
            // RAS push (call). Circular: write at ptr, advance ptr (mod
            // RAS_DEPTH), saturate count at RAS_DEPTH (a push when full
            // overwrites the oldest entry — standard circular-RAS behaviour).
            if (train_call_i) begin
                ras_d[ras_ptr_q] = train_push_pc_i;
                ras_ptr_d        = ras_ptr_q + RAS_PTR_W'(1);
                ras_cnt_d        = (ras_cnt_q == RAS_DEPTH) ? ras_cnt_q : (ras_cnt_q + 1'b1);
            end
            // RAS pop (return). Decrement ptr (mod RAS_DEPTH); count floors at
            // 0 (a pop when empty just walks the pointer, harmless:
            // ras_valid=0).
            if (train_return_i) begin
                ras_ptr_d = ras_ptr_q - RAS_PTR_W'(1);
                ras_cnt_d = (ras_cnt_q == '0) ? '0 : (ras_cnt_q - 1'b1);
            end
        end
    end

    // -------------------------------------------------------------------
    // PHT write port
    // -------------------------------------------------------------------
    // Deliberately its own always_ff with a single conditional element
    // assignment: one indexed write port with a decoder, rather than the
    // whole-array form it replaced (`for (i) pht_q[i] <= pht_d[i]`), which
    // spells out PHT_DEPTH individually enabled registers each fed by its own
    // next-state mux. Index comes from the de_t
    // snapshot so the update uses the history the branch was predicted with,
    // not the (possibly since-shifted) live GHR.
    //
    // pht_train_ctr is the read-modify-write's read: a second asynchronous
    // read address alongside the decode lookup, which synthesis serves by
    // duplicating the array. It reads the pre-write value, exactly as the old
    // flop array did.
    wire       pht_we = train_valid_i & train_cond_i;
    wire [1:0] pht_train_ctr = pht_q[train_pht_index_i];

    always_ff @(posedge clk_i) begin
        if (pht_we) pht_q[train_pht_index_i] <= sat_update(pht_train_ctr, train_taken_i);
    end

    // -------------------------------------------------------------------
    // Sequential
    // -------------------------------------------------------------------
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            ghr_q     <= '0;
            ras_ptr_q <= '0;
            ras_cnt_q <= '0;
            // ras_q unset on reset (top is gated by ras_valid); the sim
            // zero-inits unsynthesised flops, and silicon never reads a slot
            // before a push writes it (count floors the valid window).
        end else begin
            for (int i = 0; i < RAS_DEPTH; i++) ras_q[i] <= ras_d[i];
            ghr_q     <= ghr_d;
            ras_ptr_q <= ras_ptr_d;
            ras_cnt_q <= ras_cnt_d;
        end
    end

    // -------------------------------------------------------------------
    // Debug counters (sim only). Fed by decode/execute event inputs so all
    // bp_* stats are tapped from one hierarchy in sim_main. No architectural
    // effect; the inputs are don't-cares when the predictor is disabled.
    // -------------------------------------------------------------------
`ifdef VERILATOR
    // Event inputs declared as ports would clutter the architectural list, so
    // they are probed directly by sim_main off the decode/execute hierarchy
    // instead. Keep only RAS-internal stats here.
    //
    // Counted off lookup_req_i.ret_consume, NOT a bare "a return is at the
    // head": the lookup port is a held level, so a return waiting out an
    // execute stall (a preceding load's EX_MEM_WAIT, a div, EX_CSR_WAIT)
    // would be re-counted every cycle it waits — that inflated the reported
    // return count ~1.7x. ret_consume carries decode's ~stall & ~flush, so it
    // is one count per return actually consumed into de_d.
    longint unsigned ras_hit_q;
    longint unsigned ras_miss_q;
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            ras_hit_q  <= 64'd0;
            ras_miss_q <= 64'd0;
        end else begin
            if (lookup_req_i.ret_consume) begin
                if (ras_valid) ras_hit_q <= ras_hit_q + 64'd1;
                else ras_miss_q <= ras_miss_q + 64'd1;
            end
        end
    end
`endif

endmodule

`resetall

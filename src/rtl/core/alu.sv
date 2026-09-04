`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

// ---------------------------------------------------------------
// Combinational ALU (RV32I) + M extension + Zilx EA.
//
// Base RV32I: result ready in the same cycle (result_valid=1 always for
// these ops). MUL is single-cycle (GowinSynthesis infers a DSP from the
// '*' operator). DIV/REM are multi-cycle (restoring division, 32
// iterations) behind a start/done handshake, in the same style as
// cpu_mem_req_t/cpu_mem_rsp_t: the master launches with start_i, the unit answers
// with result_valid_o.
//
// Zilx: the ALU only computes the indexed load's effective address:
//   EA = base + (index << shamt)   (ALU_LX)
// operand_a = rs2_data (base), operand_b = rs1_data (index); shamt comes
// from shamt_i (de_t.mem_shamt: 0 unscaled, log2(size) scaled).
// La load vera (mem_read + sign/zero-extend) è lavoro del LSU, non
// dell'ALU; wb_src=WB_MEM seleziona il dato caricato a writeback.
// ---------------------------------------------------------------


module alu #(
    // MUL structure A/B knob. 0 = three separate 32x32 products with a 4-way
    // mux on their RESULTS (default, and what ships); 1 = one shared signed
    // 33x33 product with the signedness selected on the OPERANDS.
    // Functionally identical -- this exists so a PnR run can move one
    // variable at a time (see the note above the MUL block).
    //
    // The default is 0 because 1 MEASURED WORSE: -2.52 ns, 49.6 -> 44.1 MHz
    // on the 2026-09-01 PnR run (the MUL block records why). It used to
    // default to 1 here, which was harmless only because rv32_pkg's
    // MUL_SHARED_DSP = 0 overrides it on every real instantiation -- a
    // standalone instance of this module, or a new one that forgot to thread
    // the knob, silently got the slower form.
    parameter int unsigned MUL_SHARED_DSP = 0
) (
    input wire clk_i,
    input wire rst_ni,

    input wire [XLEN-1:0] operand_a_i,
    input wire [XLEN-1:0] operand_b_i,
    input wire alu_op_t alu_op_i,  // enum port: needs explicit net type under `default_nettype none` (EX3094); struct ports (de_t/cpu_mem_req_t) auto-resolve to var, enum does not
    input wire [1:0] shamt_i,  // Zilx index scale (log2 size, 0 unscaled)

    // result_valid_o / result_o are driven procedurally in the final
    // always_comb below, so they are variables (logic) — a wire/net cannot
    // take a procedural assignment (GowinSynthesis EX3900). Inputs and the
    // assign-driven helpers stay wire (EX3094: under `default_nettype none`
    // every net needs an explicit net type, and `logic` is a variable kind,
    // not a net — so ports must be `wire`, matching the rest of the project).
    input  wire             start_i,         // pulse: launch the op (DIV/REM only)
    output logic            result_valid_o,
    output logic [XLEN-1:0] result_o,

    // Effective-address tap: the adder's own output, taken BEFORE the result
    // mux. For ALU_ADD / ALU_LX -- the only two alu_op values a load, a store
    // or a Zilx indexed load ever carries -- this is bit-identical to
    // result_o, because sel_adder is 1 and is_mul_op is 0 for both, so the two
    // mux levels in front of result_o are pass-throughs on that path.
    //
    // It exists because those two levels are NOT free for the LSU. Everything
    // the memory port derives from the address -- the D-mem address pins, the
    // misaligned launch gate, the D-mem-vs-peri fork, the store byte strobes
    // and the store lane shift -- sat behind the full ALU result mux, which is
    // shaped for the MUL DSP (the latest signal in the block) and therefore
    // puts the adder two levels deep. Reading adder_sum directly costs no
    // logic at all: the net already exists and the mux stays for every other
    // consumer. The low bits matter most -- the store strobes and lane shift
    // depend only on EA[1:0], which now come off the bottom of the carry chain
    // instead of the top of a mux tree.
    //
    // execute_stage asserts ea_o == result_o whenever a memory request is
    // pending, so the equivalence is checked in every simulation rather than
    // rested on.
    output wire [XLEN-1:0] ea_o
);

    // -----------------------------------------------------------
    // Base RV32I — combinational
    // -----------------------------------------------------------
    // Organised by ARRIVAL TIME, not by opcode. The ALU sits in the middle
    // of the machine's critical path
    //   regfile -> forward mux -> operand mux -> ALU -> wb mux -> forward mux
    // and the adder is the last thing on it to settle, so the adder's output
    // gets its own final 2:1 mux and everything that settles earlier is
    // folded into the other input of that mux.
    //
    // The previous form -- an 11-way `unique case` building base_result, with
    // a second if/else selecting div/mul on top -- put the adder deep inside
    // the first mux and cost 5.50 ns of mux depth after the carry chain,
    // MORE than the 4.76 ns adder feeding it (measured on the 2026-08-30
    // 49.359 MHz PnR run). Selecting on alu_op_i is free by comparison:
    // alu_op_i is a flopped de_q field, stable long before the operands
    // finish forwarding, so every select below is ready early.
    //
    // Ordering of the two late candidates is measured, not assumed. On the
    // 2026-08-31 PnR run the DSP's output (`mul_ss`) arrives 3.76 ns after
    // its operands and is the LATEST signal in the block -- later than the
    // adder's carry chain. An earlier version of this file folded MUL in
    // with the early candidates on the assumption it had slack; it did not,
    // and MUL immediately became the critical path at -1.041 ns with five
    // mux levels behind the DSP. So: MUL takes the final mux (1 level), the
    // adder the one behind it (2 levels), and genuinely early candidates
    // (div / compare / shift / logic) sit deepest (3 levels).

    logic [4:0] shamt_rb;  // shift amount from operand_b (SLL/SRL/SRA)
    assign shamt_rb = operand_b_i[4:0];

    // ---- the one adder ----------------------------------------
    // ADD/LX add; SUB/SLT/SLTU subtract as a + ~b + 1. One carry chain
    // serves all five, so SLT/SLTU no longer instantiate comparators of
    // their own -- that removes two more wide candidates from the mux and
    // one more 32-bit carry chain from the fabric.
    logic            op_is_sub;  // SUB / SLT / SLTU: a + ~b + 1
    logic            op_is_lx;  // Zilx EA: a + (b << shamt)
    logic [XLEN-1:0] addend_b;
    logic [  XLEN:0] adder_sum;  // MSB = carry-out, used by SLTU

    assign op_is_sub = (alu_op_i == ALU_SUB) || (alu_op_i == ALU_SLT) || (alu_op_i == ALU_SLTU);
    assign op_is_lx  = (alu_op_i == ALU_LX);

    always_comb begin
        if (op_is_sub) addend_b = ~operand_b_i;
        else if (op_is_lx) addend_b = operand_b_i << shamt_i;
        else addend_b = operand_b_i;
    end

    assign adder_sum = {1'b0, operand_a_i} + {1'b0, addend_b} + {{XLEN{1'b0}}, op_is_sub};

    // The EA tap (see the port comment). Pre-mux, so the LSU does not pay for
    // the MUL-shaped result mux in front of result_o.
    assign ea_o      = adder_sum[XLEN-1:0];

    // ---- compares, read off that same adder --------------------
    // Unsigned: a + ~b + 1 carries out exactly when a >= b, so a < b is the
    // inverted carry-out. Signed: differing sign bits decide on their own
    // (the negative operand is the smaller), otherwise the difference's sign
    // bit decides. Both are one LUT on top of the adder.
    logic cmp_lt;
    assign cmp_lt = (alu_op_i == ALU_SLTU) ? ~adder_sum[XLEN] :
        ((operand_a_i[XLEN-1] ^ operand_b_i[XLEN-1]) ? operand_a_i[XLEN-1] : adder_sum[XLEN-1]);

    // ---- everything that settles early -------------------------
    // Shifts and bitwise logic: no carry chain, so these can afford to sit
    // behind the deeper half of the mux tree.
    logic [XLEN-1:0] logic_shift_result;
    always_comb begin
        unique case (alu_op_i)
            ALU_SLL: logic_shift_result = operand_a_i << shamt_rb;
            ALU_SRL: logic_shift_result = operand_a_i >> shamt_rb;
            ALU_SRA: logic_shift_result = $signed(operand_a_i) >>> shamt_rb;
            ALU_XOR: logic_shift_result = operand_a_i ^ operand_b_i;
            ALU_OR:  logic_shift_result = operand_a_i | operand_b_i;
            ALU_AND: logic_shift_result = operand_a_i & operand_b_i;
            default: logic_shift_result = '0;
        endcase
    end

    // Selects for the final mux. sel_adder is the only one the adder's own
    // output has to wait on, and it depends on alu_op_i alone.
    logic sel_adder, sel_cmp;
    assign sel_adder = (alu_op_i == ALU_ADD) || (alu_op_i == ALU_SUB) || op_is_lx;
    assign sel_cmp   = (alu_op_i == ALU_SLT) || (alu_op_i == ALU_SLTU);

    // -----------------------------------------------------------
    // MUL — single-cycle (DSP inference)
    // -----------------------------------------------------------
    // The DSP output is the LATEST signal in this module (3.78 ns after its
    // operands on the 2026-08-31 PnR run), so what matters is how many mux
    // levels sit BEHIND it, not how many sit in front.
    //
    // The historical form (MUL_SHARED_DSP=0) computed three products --
    // signed*signed, unsigned*unsigned, signed*unsigned -- and picked between
    // their results with a 4-way mux, which is ~2 LUT levels on top of the
    // 2-3 the final result mux already costs. That whole stack is behind the
    // DSP. CLAUDE.md measured it at ~6 mux levels off alu_result and put it
    // in reserve as worth 1.5-2 ns.
    //
    // MEASURED RESULT (2026-09-01 PnR): the shared form is WORSE by 2.52 ns
    // (49.6 MHz -> 44.1 MHz), so MUL_SHARED_DSP defaults to 0 and the
    // three-product form below is the one that ships. It lost on both halves
    // of the prediction:
    //
    //   - The operand-side sign extension was predicted free because only bit
    //     XLEN depends on the select. It is not: the operand path settles at
    //     8.366 ns and the DSP input is not reached until 10.759, against
    //     8.564 -> 10.249 for the three-product form. **+0.7 ns**, and it lands
    //     on the LATE path, which is the one that could not afford it.
    //   - The post-DSP mux did not shrink. 15.414 -> 20.232 through SEVEN
    //     alu_result hops, against 14.009 -> 18.372 through six. **+0.46 ns.**
    //     The intended structure -- one low/high select behind the DSP, then
    //     the existing final mux -- is not what GowinSynthesis built: it folds
    //     the 66-bit product's two candidate slices into the same mux tree as
    //     every other ALU candidate, so writing the select as one level in RTL
    //     does not make it one level in fabric.
    //
    // The resource goal WAS met -- it maps to a single MULT36X36 instead of
    // three 32x32 -- which is the part worth remembering: on this device the
    // DSP count and the DSP's contribution to the critical path are close to
    // independent, and folding multipliers together buys area, not slack.
    //
    // The shared form (kept for the A/B) uses the fact that signedness is a property
    // of the OPERANDS, not of the product: extend both to XLEN+1 bits with
    // the sign bit each op wants (0 for an unsigned operand) and one signed
    // multiply serves all four. The extension is a single 2:1 on bit XLEN
    // whose select comes from alu_op_i -- a flopped de_q field, ready long
    // before the operands finish forwarding -- so it is free, and bits
    // [XLEN-1:0] pass through untouched. Behind the DSP only the low/high
    // word select remains (1 level).
    //
    //   MUL     : low word,  signedness irrelevant (a*b mod 2^XLEN)
    //   MULH    : high word, signed   x signed
    //   MULHSU  : high word, signed   x unsigned
    //   MULHU   : high word, unsigned x unsigned
    //
    // It also folds three 32x32 multipliers into one 33x33, which should cut
    // DSP occupancy and the routing between the blocks. Check the resource
    // table after synthesis: a 33-bit operand pads to 36 in the MULT18X18
    // tiling, so the single product is not necessarily a third of the area.
    wire [XLEN-1:0] mul_result;
    wire is_mul_op = (alu_op_i == ALU_MUL) || (alu_op_i == ALU_MULH) || (alu_op_i == ALU_MULHSU) ||
        (alu_op_i == ALU_MULHU);

    if (MUL_SHARED_DSP != 0) begin : g_mul_shared
        // Operand signedness, decoded from alu_op_i alone (early). MUL takes
        // the low word, where signedness cannot matter, so it rides with the
        // signed ops rather than getting a case of its own.
        wire                     a_signed = (alu_op_i != ALU_MULHU);
        wire                     b_signed = (alu_op_i == ALU_MUL) || (alu_op_i == ALU_MULH);

        // Only bit XLEN depends on the select; [XLEN-1:0] pass straight to the
        // DSP. Both extended operands are correct 33-bit signed values, so the
        // 66-bit signed product holds the exact 64-bit result in [2*XLEN-1:0]
        // for all four ops (signed x signed fits in +-2^62, signed x unsigned
        // in +-2^63, unsigned x unsigned in 2^64).
        wire signed [    XLEN:0] a_ext = {a_signed & operand_a_i[XLEN-1], operand_a_i};
        wire signed [    XLEN:0] b_ext = {b_signed & operand_b_i[XLEN-1], operand_b_i};
        wire signed [2*XLEN+1:0] prod = a_ext * b_ext;

        // The only mux level behind the DSP.
        wire                     take_hi = (alu_op_i != ALU_MUL);
        assign mul_result = take_hi ? prod[2*XLEN-1:XLEN] : prod[XLEN-1:0];
    end else begin : g_mul_three
        wire signed [2*XLEN-1:0] mul_ss = $signed(operand_a_i) * $signed(operand_b_i);
        wire [2*XLEN-1:0] mul_uu = operand_a_i * operand_b_i;
        wire signed [2*XLEN-1:0] mul_su = $signed(operand_a_i) * $signed({1'b0, operand_b_i});

        // The 4-way mux on the three products' RESULTS. This is the ~2 LUT
        // levels that sit BEHIND the DSP and that MUL_SHARED_DSP=1 set out to
        // remove -- and did not (see the measurement above).
        logic [XLEN-1:0] mul_sel;
        always_comb begin
            unique case (alu_op_i)
                ALU_MUL:    mul_sel = mul_ss[XLEN-1:0];
                ALU_MULH:   mul_sel = mul_ss[2*XLEN-1:XLEN];
                ALU_MULHSU: mul_sel = mul_su[2*XLEN-1:XLEN];
                ALU_MULHU:  mul_sel = mul_uu[2*XLEN-1:XLEN];
                default:    mul_sel = '0;
            endcase
        end
        assign mul_result = mul_sel;
    end

    // -----------------------------------------------------------
    // DIV/REM — multi-cycle, restoring division, 32 iterations.
    // Div-by-zero is detected on the stored divisor (div_divisor_q); the
    // RISC-V results (all-ones for DIV/DIVU, the dividend for REM/REMU) are
    // encoded in the div_result mux below.
    //
    // Signed overflow (INT_MIN / -1) needs no special case: abs(INT_MIN)
    // is INT_MIN again in two's complement, so the unsigned core computes
    // 2^31 / 1 = 0x8000_0000 and 2^31 % 1 = 0, and the sign re-application
    // is a no-op for DIV (sign_a ^ sign_b = 0) and negates 0 for REM. That
    // is exactly the spec's required DIV = INT_MIN, REM = 0.
    // -----------------------------------------------------------
    logic is_div_op;
    assign is_div_op = (alu_op_i == ALU_DIV) || (alu_op_i == ALU_DIVU) || (alu_op_i == ALU_REM) ||
        (alu_op_i == ALU_REMU);

    typedef enum logic [1:0] {
        DIV_IDLE,
        DIV_RUN,
        DIV_DONE
    } div_state_e;
    div_state_e div_state_q, div_state_d;

    logic [4:0] div_cnt_q, div_cnt_d;
    // abs(dividend), kept ONLY to answer the divide-by-zero case (REM/REMU
    // return the dividend). The iteration itself carries the dividend in the
    // low half of div_rem_q.
    logic [XLEN-1:0] div_a_q;
    // Sign to re-apply to the magnitude, decided at launch. ONE flop, not the
    // two it replaced: DIV negates iff the operand signs differ, REM iff the
    // dividend is negative, and no op is both -- so they were always mutually
    // exclusive and a single bit carries either. Folding them also puts the
    // divide-by-zero exception in the flop instead of in the result mux: DIV
    // by zero is -1 regardless of the dividend's sign, so the DIV term is
    // gated on a non-zero divisor here rather than bypassed downstream.
    logic div_neg_q;
    // Divisor was zero. Latched at launch instead of re-deriving
    // (div_divisor_q == 0) in the result path, which was a 32-bit OR-reduce
    // (~3 LUT levels) sitting in front of the result mux for a value that is
    // constant for the whole 34-cycle operation.
    logic div_bz_q;
    logic [2*XLEN-1:0] div_rem_q;  // {remainder, quotient} shift register
    logic [XLEN-1:0] div_divisor_q;

    // Unsigned operands for the division core; the sign is re-applied on
    // the output for DIV/REM (not for DIVU/REMU).
    logic [XLEN-1:0] div_a_abs, div_b_abs;
    assign div_a_abs = (alu_op_i inside {ALU_DIV, ALU_REM}) && operand_a_i[XLEN-1] ? -operand_a_i :
        operand_a_i;
    assign div_b_abs = (alu_op_i inside {ALU_DIV, ALU_REM}) && operand_b_i[XLEN-1] ? -operand_b_i :
        operand_b_i;

    // Shift-subtract helper nets. SUG949E §3 follows Verilog-2001 style:
    // no variable declarations nested inside an always block, so these
    // cannot be `automatic` locals declared in the DIV_RUN branch
    // (GowinSynthesis rejects that, the module is ignored, and the
    // u_alu instantiation cascades to EX3990). Declare them at module
    // scope and drive them with continuous assigns instead.
    // Both are pure bit selects off div_rem_q -- no logic.
    //
    // rem_part is 32 bits, so bit 32 of (2*rem + next_dividend_bit) is
    // DROPPED by the shift. That is sound, but the reason is not the one it
    // looks like. The restoring invariant only gives rem < divisor, and a
    // divisor above 2^31 admits rem >= 2^31, which would overflow. What
    // actually bounds it is the DIVIDEND: after k iterations the partial
    // remainder is floor(a / 2^(32-k)) mod divisor, hence also below 2^k, so
    // entering any of the 32 iterations it is below 2^31 and 2*rem+bit fits
    // in 32 bits. Verified by instrumenting the loop over 200k operand pairs
    // with divisor > 2^31: the partial remainder peaks at exactly 0x7FFFFFFF
    // and bit 32 is never needed.
    //
    // So the bound comes from the 32-bit dividend AND the 32-iteration count
    // together. Widen the dividend, change the iteration count, or reuse this
    // divider at another width and rem_part must grow to XLEN+1 bits first.
    // sw/isa/div_ops covers the divisor > 2^31 shapes that would expose it.
    logic [2*XLEN-1:0] shifted;
    logic [  XLEN-1:0] rem_part;
    assign shifted  = div_rem_q << 1;
    assign rem_part = shifted[2*XLEN-1:XLEN];

    // Single clock, synchronous reset, consistent with the rest of the
    // pipeline. SUG949E section 3.2 rule 1: every register needs a reset or
    // initial value -- div_a_q / div_divisor_q / div_rem_q and the two sign
    // flops originally had none.
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            div_state_q   <= DIV_IDLE;
            div_cnt_q     <= '0;
            div_a_q       <= '0;
            div_divisor_q <= '0;
            div_neg_q     <= 1'b0;
            div_bz_q      <= 1'b0;
            div_rem_q     <= '0;
        end else begin
            div_state_q <= div_state_d;
            div_cnt_q   <= div_cnt_d;
            if (div_state_q == DIV_IDLE && start_i && is_div_op) begin
                div_a_q <= div_a_abs;
                div_divisor_q <= div_b_abs;
                // Sign of the result, as one bit. The DIV term carries the
                // `divisor != 0` guard: with a zero divisor DIV must answer
                // -1, and -1 is the all-ones magnitude with NO negate --
                // negating it would give +1. REM needs no such guard,
                // because its zero-divisor answer IS the signed dividend and
                // therefore does want the sign back.
                div_neg_q <= ((alu_op_i == ALU_DIV) &&
                              (operand_a_i[XLEN-1] ^ operand_b_i[XLEN-1]) && (operand_b_i != '0)) ||
                    ((alu_op_i == ALU_REM) && operand_a_i[XLEN-1]);
                div_bz_q <= (operand_b_i == '0);
                div_rem_q <= {{XLEN{1'b0}}, div_a_abs};
            end else if (div_state_q == DIV_RUN) begin
                // one shift-subtract per cycle
                if (rem_part >= div_divisor_q) begin
                    // Quotient bit 1 into the LSB; keep shifted[31] (the next
                    // dividend bit now at the top of the quotient half) — the
                    // no-subtract branch keeps it via `shifted`, so the
                    // subtract branch must too (shifted[31:1], not [30:0],
                    // else every subtract drops one dividend bit).
                    div_rem_q <= {rem_part - div_divisor_q, shifted[XLEN-1:1], 1'b1};
                end else begin
                    div_rem_q <= shifted;
                end
            end
        end
    end

    always_comb begin
        div_state_d = div_state_q;
        div_cnt_d   = div_cnt_q;
        unique case (div_state_q)
            DIV_IDLE:
            if (start_i && is_div_op) begin
                div_state_d = DIV_RUN;
                div_cnt_d   = '0;
            end
            DIV_RUN: begin
                if (div_cnt_q == XLEN - 1) div_state_d = DIV_DONE;
                else div_cnt_d = div_cnt_q + 5'd1;  // sized: avoid 6->5 truncation (EX3791)
            end
            DIV_DONE: div_state_d = DIV_IDLE;
            default:  div_state_d = DIV_IDLE;
        endcase
    end

    logic [XLEN-1:0] div_quot_raw, div_rem_raw;
    assign div_quot_raw = div_rem_q[XLEN-1:0];
    assign div_rem_raw  = div_rem_q[2*XLEN-1:XLEN];

    // Quotient vs remainder, off alu_op_i -- a flopped de_q field, and held
    // for the whole operation by execute's stall, exactly as the four-way
    // case this replaced relied on.
    logic div_take_rem;
    assign div_take_rem = (alu_op_i == ALU_REM) || (alu_op_i == ALU_REMU);

    // MAGNITUDE first, sign second. This ordering is what fixes a real bug
    // (2026-09-02, caught by the new sw/isa/div_ops oracle): the divider
    // stores abs(dividend) and re-applied the sign only on the NORMAL path,
    // so the divide-by-zero path returned the magnitude. RISC-V requires
    // REM rd, rs1, 0 == rs1, sign included, so `rem -5, 0` answered +5.
    // Applying one negate to whichever magnitude was selected makes the two
    // paths share the sign by construction rather than by duplication.
    //
    // Signed overflow (INT_MIN / -1) still needs no special case -- see the
    // note on the DIV/REM block above.
    //
    // It is also the cheaper shape: ONE 32-bit negate where there were two,
    // and the 32-bit (div_divisor_q == 0) reduce is gone from this path (it
    // is div_bz_q, latched at launch). Depth is unchanged -- the negate moved
    // from in front of the mux to behind it -- so this trades a carry chain
    // and an OR-reduce for nothing.
    logic [XLEN-1:0] div_mag;
    always_comb begin
        if (div_bz_q) div_mag = div_take_rem ? div_a_q : {XLEN{1'b1}};
        else div_mag = div_take_rem ? div_rem_raw : div_quot_raw;
    end

    logic [XLEN-1:0] div_result;
    assign div_result = div_neg_q ? -div_mag : div_mag;

    // -----------------------------------------------------------
    // Mux finale + valid
    // -----------------------------------------------------------
    // Ordered by measured arrival: MUL (DSP, latest) gets the final mux,
    // the adder the one behind it, everything genuinely early sits deepest.
    // See the arrival-time note at the top of the base-RV32I block, and the
    // MUL block for what was done to shorten what sits behind the DSP.
    logic [XLEN-1:0] early_result;

    always_comb begin
        if (is_div_op) early_result = div_result;
        else if (sel_cmp) early_result = {{(XLEN - 1) {1'b0}}, cmp_lt};
        else early_result = logic_shift_result;
    end

    always_comb begin
        if (is_mul_op) result_o = mul_result;
        else if (sel_adder) result_o = adder_sum[XLEN-1:0];
        else result_o = early_result;
    end

    // Only DIV/REM can be un-ready; everything else answers in-cycle.
    always_comb begin
        if (is_div_op) result_valid_o = (div_state_q == DIV_DONE);
        else result_valid_o = 1'b1;
    end

endmodule

`resetall

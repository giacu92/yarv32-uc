`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Decode stage — RV32I + M + C + Zilx indexed loads + Zicsr CSR ops.
 *
 * Consumes fetch's F/D outputs (fe_instr / fe_pc / fe_valid — `fe_` is
 * the fetch stage's sigil; the F/D register is fetch's output, so these
 * inputs take the producer's sigil) and produces a D/E control word
 * (de_o) latched into a D/E register. The register file is read
 * asynchronously from this stage so operands are captured at decode.
 * de_o feeds the execute stage, which drives the reg-file write port
 * (ALU/PC4/load/old-CSR writeback), the CSR file RMW, the fetch
 * redirect, and decode's stall/flush.
 *
 * Each pipeline stage exposes the PC it is treating, the instruction
 * word, and a valid as outputs (prefixed by its stage sigil: fe = fetch,
 * de = decode, ex = execute, ...). Further debug signals are added on
 * demand. Decode's output is the D/E register (de_o); the CPU top
 * exposes de_pc / de_instr / de_valid as taps.
 *
 * Strategy: **expand-then-decode-uniformly**. A 16-bit RVC instruction is
 * first turned into its 32-bit RV32I equivalent by c_expand(); the same
 * uniform decoder then decodes the 32-bit word whether it came native
 * (fe_instr) or from expansion. The M extension has no compressed
 * forms, so only RV32I compressed instructions are expanded.
 *
 * RVC spanning: fetch always delivers a 32-bit word; is-compressed =
 * (word[1:0]!=2'b11) is derived here from fe_instr_i. A compressed
 * instruction in the LOW half of a word is decoded this cycle and the
 * UPPER half is latched into a hold buffer to be decoded next cycle
 * (its PC = word_pc + 2). A 32-bit instruction at a 2-byte-aligned but
 * not 4-byte-aligned address (offset 2 of a fetch word) SPANS two fetch
 * words: its low halfword is bytes 2-3 of word W (stashed in the hold
 * buffer, recognized by [1:0]==2'b11), its upper halfword is bytes 0-1
 * of word W+1. When W+1 arrives, the two are STITCHED into the full
 * 32-bit instruction ({fe_instr[15:0], hold_word[15:0]}) and decoded by
 * the uniform decoder. The stitch costs one bubble (span_wait: the
 * cycle the low half is detected but W+1 is not yet in F/D). Consecutive
 * spanning instructions are handled: a stitch stashes W+1's upper half,
 * which may itself be another spanning low half.
 *
 * Branch target in the UPPER half: a redirect may land on an odd-half
 * address (fe_pc_i[1]=1, a 16-bit compressed target). The low half was
 * branched over and is discarded; the upper half is decoded directly
 * (no hold buffer). If the target is a 32-bit instr (upper half's
 * [1:0]==2'b11), its low halfword is head[31:16] and its upper halfword is
 * head+1's low half (fe_next_instr_i[15:0]) -- both buffered, so the stitch
 * completes in the SAME cycle (target_span_complete, 0 bubbles, pop 2) and
 * head+1's upper half is re-stashed as the next instr. Offset-6 targets
 * (where the next fetch has not landed yet, fe_next_valid_i=0) fall back to
 * target_span_wait: stash the low half, stitch next cycle (1 bubble).
 * fetch realigns pc_q after such a redirect so the following fetch is the
 * next word (low half), not another odd-half address.
 *
 * stall_o feeds fetch's stall_i, back-pressuring the pipe only on
 * execute's stall_i (DIV/REM busy / mem-wait) or a compressed-upper
 * hold colliding with a fresh F/D word (resource_stall). RAW hazards
 * are NOT a source of back-pressure: execute-to-decode forwarding
 * (fwd_rs1/fwd_rs2, see below) resolves same-cycle RAW at decode
 * without stalling fetch or bubbling D/E.
 *
 * Decoded opcodes that ride the D/E register to execute:
 *   OPC_AMO with Zilx funct5 (10010 unscaled / 11010 scaled) -> indexed-load
 *     control (mem_read, ALU_LX effective address, WB_MEM). rs1=index,
 *     rs2=base (roles swapped vs base loads). Real AMOs (other funct5) and
 *     RV64-only encodings (funct5=11110, funct3=011/110) decode to illegal.
 *   OPC_SYSTEM funct3!=0 -> Zicsr CSR op (CSRRW/S/C + immediate variants):
 *     csr_op / csr_wren / csr_addr, reg_write=1, wb_src=WB_CSR (rd<-old
 *     CSR). The ALU result is unused for CSR ops; execute does the RMW.
 *     Immediate variants carry zimm in imm ({27'b0,rs1_field}, uses_rs1=0);
 *     register variants uses_rs1=1. csr_wren is decode's op-present flag
 *     (squashed by dec_illegal); execute qualifies the actual write.
 *
 * Opcodes that decode to illegal=1 (execute raises the illegal-instruction
 * trap): real atomics and RV64-only Zilx encodings under OPC_AMO, reserved
 * funct3 values, sfence.vma (no VM), and any unknown opcode. OPC_MISC_MEM
 * (fence / fence.i) and OPC_SYSTEM funct3=0 (ecall / ebreak / mret / wfi) are
 * decoded and retired -- they ride sys_op to execute.
 *
 * Naming: ports *_i/_o; internal signals no prefix; flops _q, next-state
 * _d. Module instances keep u_*.
 */

module decode_stage #(
    // Branch-prediction enable. 1 = predict at decode (gshare PHT direction +
    // direct pc+imm target + RAS returns); 0 = emit no predictions (de_d.pred_*
    // zeroed, no predicted redirect) so execute resolves every control-flow
    // instruction exactly as before the predictor existed — the A/B baseline
    // and a safety fallback. The predictor block may stay instantiated.
    parameter int BP_EN = 1,
    // Where the PHT direction bit comes from.
    //   1 = the buffer entry, read at PUSH time by fetch and carried in the
    //       entry. Takes the PHT array read off this stage's redirect path,
    //       which makes PHT depth timing-neutral; costs a slightly stale GHR
    //       (see branch_predictor.sv).
    //   0 = the decode-time lookup port, read with the live GHR (default, and
    //       what rv32_pkg ships -- 1 has never been through PnR).
    // Functionally safe either way -- de_t carries the index the bits were
    // read at, so training always updates the entry the lookup used.
    parameter int BP_PUSH_LOOKUP = 1
) (
    input wire clk_i,
    input wire rstn_i,

    // F/D register inputs (from fetch_stage). Named with the fetch
    // stage's `fe_` sigil: these are fetch's output, consumed here.
    // is-compressed is derived from fe_instr_i[1:0] (not a separate
    // port) so fetch exports only pc / instr / valid.
    input wire [XLEN-1:0] fe_instr_i,
    input wire [XLEN-1:0] fe_pc_i,
    input wire            fe_valid_i,
    // The F/D entry is an instruction access fault (PC outside the
    // implemented I-mem), not an instruction. There is no word to decode:
    // it becomes a precise trap with this PC as mtval.
    input wire            fe_fault_i,

    // Prediction carried in the head entry (BP_PUSH_LOOKUP=1): one direction
    // bit per halfword slot, selected by the PC being decoded, plus the GHR
    // snapshot the bits were read with. Ignored when BP_PUSH_LOOKUP=0.
    input wire                fe_pred_lo_i,
    input wire                fe_pred_hi_i,
    input wire [BP_GHR_W-1:0] fe_ghr_i,

    // Buffer head+1 (same-cycle RVC spanning stitch). Exposes the word right
    // behind the head so a 32-bit instr at a 2-byte-aligned branch target
    // (offset 2 / 6) stitches in the cycle the target word is seen, no bubble.
    // fe_next_valid_i = count>=2; fe_pop2_o tells fetch to drop both entries.
    input wire [    XLEN-1:0] fe_next_instr_i,
    input wire [    XLEN-1:0] fe_next_pc_i,
    input wire                fe_next_valid_i,
    input wire                fe_next_fault_i,
    // head+1's high-half prediction + GHR snapshot: the same-cycle target
    // stitch re-stashes head+1's upper half, so its prediction travels with it
    // into the hold buffer.
    input wire                fe_next_pred_hi_i,
    input wire [BP_GHR_W-1:0] fe_next_ghr_i,

    // Register-file read port (decode drives addresses; data returns
    // combinationally the same cycle).
    output wire [     4:0] rs1_addr_o,
    output wire [     4:0] rs2_addr_o,
    input  wire [XLEN-1:0] rs1_data_i,
    input  wire [XLEN-1:0] rs2_data_i,

    // Back-pressure from execute (DIV/REM stall) and a branch redirect.
    input wire stall_i,
    input wire flush_i,

    // Execute writeback observation for the forward path: when execute
    // retires a register write this cycle (ex_wb_en_i) to a destination
    // (ex_wb_addr_i, ex_wb_data_i) that the instruction being decoded
    // reads, decode forwards the fresh value straight into the D/E
    // operands instead of the stale async regfile read (see fwd_rs1 /
    // fwd_rs2 below) — zero bubble, no D/E bubble and no F/D hold.
    input wire            ex_wb_en_i,
    input wire [     4:0] ex_wb_addr_i,
    // Execute writeback data: same cycle as ex_wb_en_i/ex_wb_addr_i,
    // qualified by result_ready (valid on ALU, DIV-done, or load-done).
    input wire [XLEN-1:0] ex_wb_data_i,

    // Back-pressure to fetch (so the pipe can stall on DIV/REM or a RAW
    // hazard).
    output wire stall_o,

    // Same-cycle target-span stitch: tell fetch to pop 2 (head + head+1) this
    // cycle. Asserted only on target_span_complete, which requires
    // fe_next_valid_i (count>=2), so pop-2 <= count.
    output wire fe_pop2_o,

    // Branch-predictor lookup. Decode queries the predictor for the
    // control-flow instruction at the buffer head: the PC (gshare index base
    // and RAS-key identity), whether it is a conditional branch (consult the
    // PHT for direction), and whether it is a JALR return (consult the RAS for
    // the target). The predictor answers combinationally off its flops (PC +
    // GHR only — no register data, so this stays off the regfile -> forward ->
    // compare critical path). Decode builds the prediction from the answer +
    // the direct pc+imm target it computes itself.
    output wire bp_lookup_req_t bp_lookup_o,
    input  wire bp_lookup_rsp_t bp_lookup_i,

    // Predicted redirect to fetch. Asserted the cycle a control-flow
    // instruction at the buffer head is consumed into de_d AND predicted
    // taken — fetch kills the younger (wrong-path) buffer entries and steers
    // to pred_target, one stage earlier than execute's resolve. Lower priority
    // than execute's redirect (trap / mret / interrupt / mispredict), which
    // flushes decode (flush_i) and overrides this.
    output wire            pred_redirect_valid_o,
    output wire [XLEN-1:0] pred_redirect_addr_o,

    // D/E pipeline register output: the full decoded control word, consumed
    // by the execute stage.
    output de_t de_o,

    // de_* per-stage taps (D/E register): the PC / instruction word / valid
    // decode is treating. Exposed like fetch's fe_*_o so every pipeline stage
    // has a uniform pc / instr / valid output (these are fields of de_o,
    // broken out as named ports so the stage boundary shows them even when
    // nothing downstream reads the full struct).
    output wire [XLEN-1:0] de_pc_o,     // D/E instruction PC
    output wire [XLEN-1:0] de_instr_o,  // 32-bit word decode treated
    output wire            de_valid_o   // D/E valid
);

    // =================================================================
    // 32-bit instruction builders (used only by c_expand).
    // Each takes a sign-extended 32-bit offset / value and places the
    // immediate bits per the RV32I encoding. Keeping these separate from
    // the main decoder means the decoder is a pure consumer of a 32-bit
    // word, native or expanded.
    // =================================================================
    function automatic logic [31:0] mk_r(input logic [6:0] op, input logic [4:0] rd,
                                         input logic [4:0] rs1, input logic [4:0] rs2,
                                         input logic [2:0] f3, input logic [6:0] f7);
        return {f7, rs2, rs1, f3, rd, op};
    endfunction

    // I-type: imm[11:0] = off[11:0].
    function automatic logic [31:0] mk_i(input logic [6:0] op, input logic [4:0] rd,
                                         input logic [4:0] rs1, input logic [2:0] f3,
                                         input logic [31:0] off);
        return {off[11:0], rs1, f3, rd, op};
    endfunction

    // S-type: imm[11:5]=off[11:5], imm[4:0]=off[4:0].
    function automatic logic [31:0] mk_s(input logic [6:0] op, input logic [4:0] rs1,
                                         input logic [4:0] rs2, input logic [2:0] f3,
                                         input logic [31:0] off);
        return {off[11:5], rs2, rs1, f3, off[4:0], op};
    endfunction

    // B-type: imm[12]=off[12], imm[11]=off[11], imm[10:5]=off[10:5],
    // imm[4:1]=off[4:1], imm[0]=0. off[0] must be 0.
    function automatic logic [31:0] mk_b(input logic [6:0] op, input logic [4:0] rs1,
                                         input logic [4:0] rs2, input logic [2:0] f3,
                                         input logic [31:0] off);
        return {off[12], off[10:5], rs2, rs1, f3, off[4:1], off[11], op};
    endfunction

    // U-type: imm[31:12]=off[31:12].
    function automatic logic [31:0] mk_u(input logic [6:0] op, input logic [4:0] rd,
                                         input logic [31:0] off);
        return {off[31:12], rd, op};
    endfunction

    // J-type: imm[20]=off[20], imm[10:1]=off[10:1], imm[11]=off[11],
    // imm[19:12]=off[19:12], imm[0]=0. off[0] must be 0.
    function automatic logic [31:0] mk_j(input logic [6:0] op, input logic [4:0] rd,
                                         input logic [31:0] off);
        return {off[20], off[10:1], off[11], off[19:12], rd, op};
    endfunction

    // =================================================================
    // c_expand: 16-bit RVC -> 32-bit RV32I equivalent.
    //
    // Returns 32'h0000_0000 for illegal / unsupported compressed
    // encodings. The main decoder treats opcode 0000000 as illegal (no
    // major opcode matches), so a zero word decodes to illegal=1. Legal
    // expansions always produce a recognized 32-bit instruction.
    //
    // Compressed register mapping: crd/crs1/crs2 = {2'b01, 3-bit} -> x8..x15.
    //
    // Verified-by-hand encodings (see sim/program.hex):
    //   c.li   x1,5     -> 0x4095  -> addi x1,x0,5
    //   c.addi x1,1     -> 0x0085  -> addi x1,x1,1
    //   c.j    16        -> 0xA801  -> jal   x0,16
    //   c.lw   x8,4(x9)  -> 0x4180  -> lw    x8,4(x9)
    // The scrambled-immediate forms (c.lwsp/c.swsp/c.addi4spn/
    // c.addi16sp/c.beqz/bnez/c.srli/srai/andi) are implemented per the
    // RISC-V C spec table but pending exhaustive sim verification.
    // =================================================================
    function automatic logic [31:0] c_expand(input logic [15:0] c);
        logic [ 2:0] f3;  // c[15:13]
        logic [ 4:0] rd5;  // c[11:7]  (5-bit register, quad1/quad2)
        logic [ 4:0] crd;  // compressed rd   = {2'b01,c[4:2]}  (x8..x15)
        logic [ 4:0] crs1;  // compressed rs1  = {2'b01,c[9:7]}
        logic [ 4:0] crs2;  // compressed rs2  = {2'b01,c[4:2]}
        logic [ 4:0] rs2_5;  // c[6:2] (5-bit rs2 / shamt, quad2)
        logic [31:0] off;  // sign-extended offset / immediate
        logic [31:0] res;

        res   = 32'h0000_0000;  // default: illegal (decodes to illegal=1)
        f3    = c[15:13];
        rd5   = c[11:7];
        crd   = {2'b01, c[4:2]};
        crs1  = {2'b01, c[9:7]};
        crs2  = {2'b01, c[4:2]};
        rs2_5 = c[6:2];

        unique case (c[1:0])

            // ---------------------------------------------------
            // Quadrant 0 (c[1:0] = 00)
            // ---------------------------------------------------
            2'b00: begin
                unique case (f3)
                    3'b000: begin  // c.addi4spn  -> addi rd', x2, nzuimm
                        // nzuimm[9:6]=c[10:7], [5:4]=c[12:11], [3]=c[5],
                        // [2]=c[6], [1:0]=00. Illegal if nzuimm==0.
                        // (spec nzuimm[5:4|9:6|2|3]=instr[12:11|10:7|6|5];
                        //  bit[2] comes from c[6], bit[3] from c[5].)
                        logic [9:0] nz;
                        nz = {c[10:7], c[12:11], c[5], c[6], 2'b00};
                        if (nz != 10'd0) begin
                            off = {22'b0, nz};
                            res = mk_i(OPC_OP_IMM, crd, 5'd2, 3'b000, off);
                        end
                    end
                    3'b010: begin  // c.lw  -> lw rd', imm(rs1')
                        // offset[6]=c[5], [5:3]=c[12:10], [2]=c[6], [1:0]=00.
                        logic [6:0] lwoff;
                        lwoff = {c[5], c[12:10], c[6], 2'b00};
                        off   = {25'b0, lwoff};
                        res   = mk_i(OPC_LOAD, crd, crs1, 3'b010, off);
                    end
                    3'b110: begin  // c.sw  -> sw rs2', imm(rs1')
                        logic [6:0] swoff;
                        swoff = {c[5], c[12:10], c[6], 2'b00};
                        off   = {25'b0, swoff};
                        res   = mk_s(OPC_STORE, crs1, crs2, 3'b010, off);
                    end
                    // c.fld / c.flw / c.lq / c.fsd ... : not in RV32IMAC
                    default: res = 32'h0000_0000;
                endcase
            end

            // ---------------------------------------------------
            // Quadrant 1 (c[1:0] = 01)
            // ---------------------------------------------------
            2'b01: begin
                unique case (f3)
                    3'b000: begin  // c.addi / c.nop -> addi rd, rd, nzimm6
                        // imm6 = {c[12], c[6:2]}. c.nop = rd==0 && imm==0;
                        // rd==0 && imm!=0 is reserved -> illegal.
                        logic [5:0] imm6;
                        imm6 = {c[12], c[6:2]};
                        if (rd5 == 5'd0 && imm6 == 6'd0) begin
                            res = mk_i(OPC_OP_IMM, 5'd0, 5'd0, 3'b000, 32'd0);
                        end else if (rd5 != 5'd0) begin
                            off = {{26{imm6[5]}}, imm6};  // sext to 32
                            res = mk_i(OPC_OP_IMM, rd5, rd5, 3'b000, off);  // rs1=rd
                        end
                    end
                    3'b001: begin  // c.jal (RV32) -> jal x1, offset
                        // CJ offset scramble (verified vs c.j 16 = 0xA801):
                        // imm[11]=c[12] imm[10]=c[8] imm[9]=c[10] imm[8]=c[9]
                        // imm[7]=c[6]  imm[6]=c[7] imm[5]=c[2] imm[4]=c[11]
                        // imm[3]=c[5]  imm[2]=c[4] imm[1]=c[3]  imm[0]=0.
                        logic [11:0] joff;
                        joff = {
                            c[12],
                            c[8],
                            c[10],
                            c[9],
                            c[6],
                            c[7],
                            c[2],
                            c[11],
                            c[5],
                            c[4],
                            c[3],
                            1'b0
                        };
                        off = {{20{joff[11]}}, joff};
                        res = mk_j(OPC_JAL, 5'd1, off);  // rd = x1
                    end
                    3'b010: begin  // c.li -> addi rd, x0, imm6
                        logic [5:0] imm6;
                        imm6 = {c[12], c[6:2]};
                        if (rd5 != 5'd0) begin  // rd==0 is HINT -> illegal
                            off = {{26{imm6[5]}}, imm6};
                            res = mk_i(OPC_OP_IMM, rd5, 5'd0, 3'b000, off);  // rs1=x0
                        end
                    end
                    3'b011: begin  // c.addi16sp (rd==x2) / c.lui (else)
                        if (rd5 == 5'd2) begin
                            // nzimm[9]=c[12], [8:7]=c[4:3], [6]=c[5],
                            // [5]=c[2], [4]=c[6], [3:0]=0 (mult of 16).
                            // Illegal if 0.
                            logic [9:0] nz;
                            nz = {c[12], c[4:3], c[5], c[2], c[6], 4'b0};
                            if (nz != 10'd0) begin
                                off = {{22{nz[9]}}, nz};
                                res = mk_i(OPC_OP_IMM, 5'd2, 5'd2, 3'b000, off);
                            end
                        end else begin
                            // c.lui -> lui rd, nzimm6<<12.
                            // Illegal if rd==0 or imm6==0.
                            logic [5:0] imm6;
                            imm6 = {c[12], c[6:2]};
                            if (rd5 != 5'd0 && imm6 != 6'd0) begin
                                off = {{14{imm6[5]}}, imm6, 12'b0};
                                res = mk_u(OPC_LUI, rd5, off);
                            end
                        end
                    end
                    3'b100: begin  // c.srli / c.srai / c.andi (CB) +
                        //               c.sub / c.xor / c.or / c.and (CA)
                        // All share rd' = c[9:7] (x8..x15) as BOTH rd and
                        // rs1 (srli/srai/andi: rd=rs1=rd'; sub/xor/or/and:
                        // rd=rs1=rd', rs2=rs2'=c[4:2]). c[12] is shamt[5]
                        // (srli/srai) or imm[5] (andi); RV32 requires
                        // shamt[5]=0 for srli/srai. Top selector = c[11:10]:
                        //   00 srli  01 srai  10 andi
                        //   11 arith by c[6:5]: 00 sub 01 xor 10 or 11 and
                        unique case (c[11:10])
                            2'b00: begin  // c.srli  rd', shamt (RV32 shamt=c[6:2])
                                off = {25'b0, 2'b00, c[6:2]};
                                res = mk_i(OPC_OP_IMM, crs1, crs1, 3'b101, off);
                            end
                            2'b01: begin  // c.srai  rd', shamt
                                off = {20'b0, 7'b0100000, c[6:2]};
                                res = mk_i(OPC_OP_IMM, crs1, crs1, 3'b101, off);
                            end
                            2'b10: begin  // c.andi  rd', imm
                                logic [5:0] imm6;
                                imm6 = {c[12], c[6:2]};
                                off  = {{26{imm6[5]}}, imm6};
                                res  = mk_i(OPC_OP_IMM, crs1, crs1, 3'b111, off);
                            end
                            2'b11: begin  // c.sub / c.xor / c.or / c.and (CA)
                                unique case (c[6:5])
                                    2'b00: begin  // c.sub  rd', rd' - rs2'
                                        res = mk_r(OPC_OP, crs1, crs1, crs2, 3'b000, 7'b0100000);
                                    end
                                    2'b01: begin  // c.xor  rd', rd' ^ rs2'
                                        res = mk_r(OPC_OP, crs1, crs1, crs2, 3'b100, 7'b0000000);
                                    end
                                    2'b10: begin  // c.or   rd', rd' | rs2'
                                        res = mk_r(OPC_OP, crs1, crs1, crs2, 3'b110, 7'b0000000);
                                    end
                                    2'b11: begin  // c.and  rd', rd' & rs2'
                                        res = mk_r(OPC_OP, crs1, crs1, crs2, 3'b111, 7'b0000000);
                                    end
                                    default: res = 32'h0000_0000;
                                endcase
                            end
                            default: res = 32'h0000_0000;
                        endcase
                    end
                    3'b101: begin  // c.j -> jal x0, offset
                        // CJ offset scramble (see c.jal above).
                        logic [11:0] joff;
                        joff = {
                            c[12],
                            c[8],
                            c[10],
                            c[9],
                            c[6],
                            c[7],
                            c[2],
                            c[11],
                            c[5],
                            c[4],
                            c[3],
                            1'b0
                        };
                        off = {{20{joff[11]}}, joff};
                        res = mk_j(OPC_JAL, 5'd0, off);  // rd = x0
                    end
                    3'b110: begin  // c.beqz rs1', offset  -> beq rs1', x0
                        // CB branch offset: imm[8]=c[12], [7:6]=c[6:5],
                        // [5]=c[2], [4:3]=c[11:10], [2:1]=c[4:3], [0]=0.
                        logic [8:0] boff;
                        boff = {c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0};
                        off  = {{23{boff[8]}}, boff};
                        res  = mk_b(OPC_BRANCH, crs1, 5'd0, 3'b000, off);  // beq
                    end
                    3'b111: begin  // c.bnez rs1', offset  -> bne rs1', x0
                        logic [8:0] boff;
                        boff = {c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0};
                        off  = {{23{boff[8]}}, boff};
                        res  = mk_b(OPC_BRANCH, crs1, 5'd0, 3'b001, off);  // bne
                    end
                endcase
            end

            // ---------------------------------------------------
            // Quadrant 2 (c[1:0] = 10)
            // ---------------------------------------------------
            2'b10: begin
                unique case (f3)
                    3'b000: begin  // c.slli rd, shamt -> slli rd, rd, shamt
                        // shamt = {c[12], c[6:2]} (RV32 uses [4:0]).
                        // rd==0 is HINT -> illegal.
                        if (rd5 != 5'd0) begin
                            off = {25'b0, 2'b00, c[6:2]};
                            res = mk_i(OPC_OP_IMM, rd5, rd5, 3'b001, off);
                        end
                    end
                    3'b010: begin  // c.lwsp rd, imm(x2) -> lw rd, imm(x2)
                        // uimm[5]=c[12], [4:2]=c[6:4], [7:6]=c[3:2], [1:0]=00.
                        // rd==0 reserved -> illegal.
                        logic [7:0] uimm;
                        uimm = {c[3:2], c[12], c[6:4], 2'b00};
                        off  = {24'b0, uimm};
                        if (rd5 != 5'd0) begin
                            res = mk_i(OPC_LOAD, rd5, 5'd2, 3'b010, off);
                        end
                    end
                    3'b100: begin  // c.jr / c.jalr / c.mv / c.add / c.ebreak
                        // rs2==0: c.jr/c.jalr/c.ebreak. rs2!=0: c.mv/c.add
                        // (selected by c[12]: 0=mv, 1=add).
                        if (rd5 == 5'd0 && rs2_5 == 5'd0) begin
                            // c.ebreak -> ebreak (0x0010_0073). The uniform
                            // decoder maps that to SYS_EBREAK, so the trap
                            // reports mcause=3 (breakpoint). Expanding it to
                            // the zero word instead reported mcause=2
                            // (illegal instruction) for a legal instruction.
                            res = mk_i(OPC_SYSTEM, 5'd0, 5'd0, 3'b000, 32'd1);
                        end else if (rs2_5 == 5'd0) begin
                            // c.jr (c[12]=0) / c.jalr (c[12]=1): jalr rd, 0(rs1).
                            // rs1 = c[11:7] (= rd5); rd = x0 for c.jr (discard
                            // link), x1 (ra) for c.jalr. c.ebreak (rs1==0) was
                            // caught above. Using rd5 as rd would write the link
                            // back INTO the jump register, corrupting it.
                            res = mk_i(OPC_JALR, c[12] ? 5'd1 : 5'd0, rd5, 3'b000, 32'd0);
                        end else begin
                            // c.mv / c.add — rd==0 is HINT -> illegal
                            if (rd5 != 5'd0) begin
                                if (c[12])  // c.add  -> add rd, rd, rs2
                                    res = mk_r(OPC_OP, rd5, rd5, rs2_5, 3'b000, 7'b0000000);
                                else  // c.mv   -> add rd, x0, rs2
                                    res = mk_r(OPC_OP, rd5, 5'd0, rs2_5, 3'b000, 7'b0000000);
                            end
                        end
                    end
                    3'b110: begin  // c.swsp rs2, imm(x2) -> sw rs2, imm(x2)
                        // uimm[5:2]=c[12:9], [7:6]=c[8:7], [1:0]=00.
                        logic [7:0] uimm;
                        uimm = {c[8:7], c[12:9], 2'b00};
                        off  = {24'b0, uimm};
                        res  = mk_s(OPC_STORE, 5'd2, rs2_5, 3'b010, off);
                    end
                    // c.fldsp / c.flwsp / c.fswsp : not in RV32IMAC
                    default: res = 32'h0000_0000;
                endcase
            end

            default: res = 32'h0000_0000;
        endcase

        return res;
    endfunction

    // =================================================================
    // Hold buffer: when a 32-bit word's low half is compressed, decode it
    // this cycle and stash the upper half for next cycle.
    // =================================================================
    logic                hold_q;
    logic [        31:0] hold_word_q;
    logic [        31:0] hold_pc_q;
    // Prediction that travels with a stashed halfword (BP_PUSH_LOOKUP=1).
    // Every stashed PC has bit[1]=1 by construction -- buffer_upper stashes
    // fe_pc+2, span_complete fe_pc+2, target_span_complete fe_next_pc+2, and
    // target_span_wait stashes an odd-half target PC that already has bit[1]
    // set -- so the bit stashed is always the entry's HIGH-half prediction and
    // one bit is enough.
    logic                hold_pred_q;
    logic [BP_GHR_W-1:0] hold_ghr_q;

    logic                hold_d;
    logic [        31:0] hold_word_d;
    logic [        31:0] hold_pc_d;
    logic                hold_pred_d;
    logic [BP_GHR_W-1:0] hold_ghr_d;

    // =================================================================
    // D/E pipeline register
    // =================================================================
    de_t de_q, de_d, de_next;

    // =================================================================
    // RAW hazard resolution: execute -> decode forwarding
    //
    // There is no stall-on-RAW interlock. Decode's live combinational source
    // addresses (rs1_addr_dec / rs2_addr_dec, forced to x0 when unused, so an
    // unused source can never match) are compared against execute's retiring
    // writeback destination; on a match the fresh value is injected instead of
    // the stale async regfile read — zero bubble, no D/E bubble, no F/D hold.
    // Covers DIV/REM and load results too: wb_en_o / wb_data_o are qualified
    // by result_ready in execute, so they are valid exactly the cycle the
    // result is ready, and decode is parked (stall_i) through the whole
    // busy/wait window.
    // =================================================================
    wire fwd_rs1 = ex_wb_en_i & (ex_wb_addr_i != 5'd0) & (rs1_addr_dec == ex_wb_addr_i);
    wire fwd_rs2 = ex_wb_en_i & (ex_wb_addr_i != 5'd0) & (rs2_addr_dec == ex_wb_addr_i);

    wire [XLEN-1:0] rs1_fwd = fwd_rs1 ? ex_wb_data_i : rs1_data_i;
    wire [XLEN-1:0] rs2_fwd = fwd_rs2 ? ex_wb_data_i : rs2_data_i;

    // =================================================================
    // Source selection + uniform 32-bit decode (one always_comb).
    // =================================================================
    // is-compressed of the fetched word, derived from the instruction
    // word itself (fetch no longer exports it as a separate port).
    wire fe_is_compressed = (fe_instr_i[1:0] != 2'b11);

    logic [31:0] src_instr32;  // the 32-bit word the uniform decoder consumes
    logic [31:0] src_word32;  // 32-bit source BEFORE expansion (native / stitched)
    logic [15:0] src_half16;  // 16-bit compressed source halfword
    logic [31:0] src_pc;
    logic src_is_compressed;
    logic buffer_upper;
    logic decoded_valid;

    // -----------------------------------------------------------------
    // Spanning-stitch predicates.
    //
    // A 32-bit instruction at a 2-byte-aligned-but-not-4-byte-aligned
    // address (offset 2 of a fetch word) SPANS two fetch words: its low
    // halfword is bytes 2-3 of word W, its upper halfword is bytes 0-1 of
    // word W+1. The hold buffer stashes that low halfword; whether the
    // stash is a compressed upper half or a spanning 32-bit low half is
    // derivable from the stashed halfword's [1:0] (==2'b11 -> 32-bit
    // low half). So the stitch needs NO new flop: these are all wires
    // over flops + inputs.
    // -----------------------------------------------------------------
    wire hold_is_span = (hold_word_q[1:0] == 2'b11);  // stashed low half of a 32-bit instr
    wire hold_plain_pending = hold_q && !hold_is_span;  // stashed compressed upper half
    wire span_pending = hold_q && hold_is_span;  // spanning low half held

    // A fault entry at the head carries no instruction word. It outranks every
    // other source EXCEPT a stashed compressed upper half: that half is a
    // COMPLETE instruction older than the faulting PC, so taking the fault
    // first would drop it and make the trap imprecise (one instruction missing
    // from the retire stream). stall_o already holds F/D while a plain stash is
    // consumed, so the fault entry is still at the head next cycle and traps
    // then. A SPANNING stash is the opposite case: the upper half it is waiting
    // for is exactly what could not be fetched, so the fault belongs to it and
    // the stash is dropped rather than stitched against a synthesised word.
    wire fetch_fault = fe_valid_i && fe_fault_i;
    wire take_fault = fetch_fault && !hold_plain_pending;

    wire is_hold_plain = hold_plain_pending;  // upper half ready
    wire span_complete = !take_fault && span_pending && fe_valid_i;  // stitch
    wire span_wait = !take_fault && span_pending && !fe_valid_i;  // waiting for the word
    wire target_upper = !take_fault && !hold_q && fe_valid_i && fe_pc_i[1];  // odd half
    wire target_span = target_upper && (fe_instr_i[17:16] == 2'b11);  // 32-bit at target
    // Same-cycle stitch: the target word's low half (head[31:16]) is the
    // 32-bit instr's low half, and head+1's low half (fe_next_instr_i[15:0])
    // is its upper half — both buffered, so stitch now (no bubble). A fault
    // at head+1 falls back to wait (stall-and-wait the old way) so the fault
    // routes through with the faulting word's PC next cycle.
    wire target_span_complete = target_span && fe_next_valid_i && !fe_next_fault_i;
    wire target_span_wait = target_span && !target_span_complete;

    logic [4:0] rs1_addr_dec;
    logic [4:0] rs2_addr_dec;
    logic [11:0] csr_addr_dec;

    // ---- Field extraction ----
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [4:0] funct5;  // AMO/Zilx mode (instr[31:27])
    logic [6:0] funct7;
    logic funct7b5;  // funct7[5] — SUB/SRA, M-ext
    logic [11:0] funct12;
    logic aq, rl;  // AMO ordering bits (instr[26:25]) — must be 0 for Zilx
    logic [4:0] rd_field, rs1_field, rs2_field;
    logic is_m_ext;

    // ---- Immediates (sign-extended) ----
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;

    // ---- Decoded control (defaults; recognised opcodes override) ----
    logic [31:0] imm;
    logic uses_rs1, uses_rs2;
    logic       reg_write;
    logic       csr_wren;
    alu_op_t    alu_op;
    alu_src_a_t alu_src_a;
    alu_src_b_t alu_src_b;
    logic mem_read, mem_write;
    mem_size_t            mem_size;
    logic                 mem_unsigned;
    logic      [     1:0] mem_shamt;  // Zilx index scale (0 unscaled, log2 size scaled)
    wb_src_t              wb_src;
    branch_t              branch_type;
    logic                 dec_illegal;
    logic                 is_csr;
    csr_op_t              csr_op;
    sys_op_t              sys_op;
    logic                 exc_req;  // decode requests a sync trap
    logic      [XLEN-1:0] exc_cause;
    logic      [XLEN-1:0] exc_tval;

    // ---- Branch-prediction (prediction-at-decode) ----
    // Control-flow classification and the PC-relative immediate are decoded
    // straight off the SOURCE HALVES (src_word32 / src_half16), in parallel
    // with c_expand and with the opcode case tree rather than behind them --
    // see the block that drives them for why, and the `ifdef VERILATOR` check
    // at the end of the file for the proof that the two decoders agree.
    // is_return = JALR reading a link register (x1/x5) with rd=x0. The direct
    // pc+imm target is computed for JAL and conditional branches (both
    // PC-relative); JALR targets are rs1+imm (data-dependent) so they are not
    // predicted here -- returns use the RAS, calls/indirect fall to execute (a
    // call still pushes the RAS at resolve, where execute re-derives the kind
    // from branch_type/rd).
    logic is_cf, is_return_cf;
    // Shallow classification: kind of control flow, per source form.
    logic cf_c_jal, cf_c_cond, cf_c_jalr, cf_c_ret;  // compressed halfword
    logic cf_n_jal, cf_n_cond, cf_n_jalr, cf_n_ret;  // native 32-bit word
    logic cf_jal, cf_cond, cf_jalr, cf_ret;  // selected by src_is_compressed
    // The two signals fetch actually sees, in their flattest form: an OR of
    // three ANDs for the direction and a single 2:1 for the address.
    logic                            cf_taken;
    logic         [        XLEN-1:0] cf_target;
    logic                            pred_valid;
    logic                            pred_taken;
    logic         [        XLEN-1:0] pred_target;
    pred_source_t                    pred_source;
    logic         [BP_PHT_IDX_W-1:0] pred_pht_index;
    // PHT direction / index / GHR for the instruction being decoded, resolved
    // from either the carried-in-entry source or the decode-time lookup port.
    logic                            pht_taken_src;
    logic         [    BP_GHR_W-1:0] pht_ghr_src;
    logic         [BP_PHT_IDX_W-1:0] pht_index_src;
    logic         [        XLEN-1:0] pred_dir_target;  // src_pc + pred_imm (PC-relative)
    // Branch/JAL immediate for the predictor's adder only. Four candidates,
    // all pure bit permutations of a source half plus a sign extension.
    logic [XLEN-1:0] pred_cj_imm, pred_cb_imm;  // compressed CJ / CB
    logic [XLEN-1:0] pred_n_imm_j, pred_n_imm_b;  // native J / B
    logic [XLEN-1:0] pred_imm;

    // Zilx indexed-load decode helpers (OPC_AMO). Hoisted to module scope
    // and driven with a default every cycle in the decode always_comb below:
    // declaring them inside the OPC_AMO case branch and assigning them only
    // there made GowinSynthesis infer a latch (EX2420/EX3101) — they "held"
    // when any other opcode ran. Defaults at the top of the always_comb kill
    // the latch; the OPC_AMO branch overrides zilx_ok to 1 when the encoding
    // is a valid RV32 Zilx indexed load.
    logic is_unscaled, is_scaled;
    logic size_ok_rv32;
    logic zilx_ok;

    always_comb begin
        // ---- Source selection ----
        // Priority: fault (unless a plain stash outranks it -- see take_fault)
        // > spanning stitch > compressed hold > odd-half branch target > fresh
        // F/D low half. A spanning 32-bit instr's low halfword is stashed in
        // the hold buffer (or sits at an odd-half branch target); its upper
        // halfword is the next fetch word's low half, stitched in here when
        // that word arrives (span_complete).
        //
        // The chain picks a SOURCE, not a decoded word: src_word32 for a
        // native or stitched 32-bit instruction, src_half16 for a compressed
        // one, src_is_compressed says which, and c_expand runs once after.
        // Defaults, so every branch drives all four (no latch) and the
        // don't-care branches sit at a defined 0 instead of holding.
        src_word32        = 32'd0;
        src_half16        = 16'd0;
        src_pc            = fe_pc_i;
        src_is_compressed = 1'b0;

        if (take_fault) begin
            // No instruction word exists. The word is a don't-care; the
            // exception below carries the cause and the faulting PC.
        end else if (span_complete) begin
            // Stitch: low half = hold_word_q[15:0] (bytes 2-3 of word W),
            // upper half = fe_instr_i[15:0] (bytes 0-1 of word W+1). PC is the
            // spanning instr's own PC (hold_pc_q). A real 32-bit instr, so it
            // bypasses c_expand.
            src_word32 = {fe_instr_i[15:0], hold_word_q[15:0]};
            src_pc     = hold_pc_q;
        end else if (span_wait) begin
            // Spanning low half held, upper-half fetch word not here yet: no
            // complete instruction this cycle (bubble). The word is a
            // don't-care (decoded_valid=0 gates de_d.valid / illegal).
            src_pc = hold_pc_q;
        end else if (is_hold_plain) begin
            // Compressed upper half stashed last cycle: decode it. It is
            // compressed by construction -- a stash with [1:0]==2'b11 is
            // hold_is_span and routes to the span cases above.
            src_half16        = hold_word_q[15:0];
            src_pc            = hold_pc_q;
            src_is_compressed = 1'b1;
        end else if (target_span_complete) begin
            // Same-cycle stitch: the target word's upper half (head[31:16]) is
            // the 32-bit instr's low half, head+1's low half is its upper half.
            // PC is the target PC. head+1's upper half is re-stashed (hold
            // next-state) as the next instruction.
            src_word32 = {fe_next_instr_i[15:0], fe_instr_i[31:16]};
        end else if (target_span_wait) begin
            // Branch target at offset 2/6 is a 32-bit instr's low half but
            // head+1 is absent or faults: no emit this cycle (bubble), stash
            // the low half, stitch next cycle (the old span path). A fault at
            // head+1 lands here -> trap next cycle.
        end else if (target_upper) begin
            // Compressed target in the upper half (fe_pc_i[1]=1). The low half
            // was branched over and is discarded; do NOT stash it. target_span
            // was taken above, so [17:16] != 2'b11 here.
            src_half16        = fe_instr_i[31:16];
            src_is_compressed = 1'b1;
        end else if (fe_is_compressed) begin
            // Fresh F/D low half (fe_pc_i[1]=0), compressed.
            src_half16        = fe_instr_i[15:0];
            src_is_compressed = 1'b1;
        end else begin
            // Fresh F/D low half, native 32-bit.
            src_word32 = fe_instr_i;
        end

        // ONE c_expand instance. It used to be called from three separate
        // branches of the chain above, which inlines the whole RVC case tree
        // three times and then muxes 32 bits of tree output; selecting the
        // halfword first costs a 16-bit mux and leaves a single tree.
        src_instr32 = src_is_compressed ? c_expand(src_half16) : src_word32;

        // Stash the upper half of a fresh word whose low half was
        // consumed this cycle (a fresh compressed instr at offset 0).
        // Spanning low halves are stashed separately (span_complete /
        // target_span in the hold next-state). The compressed-vs-spanning
        // distinction is derived from the stashed halfword's [1:0], so
        // buffer_upper needs no span flag of its own.
        buffer_upper = (!hold_q) && !target_upper && fe_valid_i && fe_is_compressed;

        // A COMPLETE instruction is available this cycle: every case EXCEPT
        // span_wait (upper-half word not here yet) and target_span_wait (just
        // stashed the low half, waiting for the stitch). This feeds de_d.valid,
        // the illegal-trap gate and the predictor's is_cf, so neither wait
        // state emits a spurious valid, trap or redirect.
        //
        // The !hold_q term collapses the three head-of-buffer cases the old
        // form spelled out separately: with no stash, a valid head yields an
        // instruction unless it is a target span still waiting for head+1.
        decoded_valid = take_fault | span_complete | is_hold_plain |
            (fe_valid_i && !hold_q && !take_fault && !target_span_wait);

        // ---- Field extraction ----
        opcode = src_instr32[6:0];
        funct3 = src_instr32[14:12];
        funct7 = src_instr32[31:25];
        funct7b5 = src_instr32[30];
        funct5 = src_instr32[31:27];  // AMO/Zilx mode
        funct12 = src_instr32[31:20];
        aq = src_instr32[26];  // AMO aq (must be 0 for Zilx)
        rl = src_instr32[25];  // AMO rl (must be 0 for Zilx)
        rd_field = src_instr32[11:7];
        rs1_field = src_instr32[19:15];
        rs2_field = src_instr32[24:20];
        is_csr = (opcode == OPC_SYSTEM) && (funct3 != 3'b000);
        is_m_ext = (opcode == OPC_OP) && (funct7 == 7'b0000001);

        // ---- Immediates (sign-extended) ----
        imm_i = {{20{src_instr32[31]}}, src_instr32[31:20]};
        imm_s = {{20{src_instr32[31]}}, src_instr32[31:25], src_instr32[11:7]};
        imm_b = {
            {19{src_instr32[31]}},
            src_instr32[31],
            src_instr32[7],
            src_instr32[30:25],
            src_instr32[11:8],
            1'b0
        };
        imm_u = {src_instr32[31:12], 12'b0};
        imm_j = {
            {11{src_instr32[31]}},
            src_instr32[31],
            src_instr32[19:12],
            src_instr32[20],
            src_instr32[30:21],
            1'b0
        };

        // ---- Decoded control defaults (recognised opcodes override) ----
        imm = 32'd0;
        uses_rs1 = 1'b0;
        uses_rs2 = 1'b0;
        reg_write = 1'b0;
        csr_wren = 1'b0;
        csr_op = CSR_NONE;
        alu_op = ALU_ADD;
        alu_src_a = ALU_A_RS1;
        alu_src_b = ALU_B_IMM;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_size = MS_W;
        mem_unsigned = 1'b0;
        mem_shamt = 2'd0;
        wb_src = WB_ALU;
        branch_type = BR_NONE;
        dec_illegal = 1'b1;  // default: anything not matched is illegal
        sys_op = SYS_NONE;
        exc_req = 1'b0;
        exc_cause = '0;
        exc_tval = '0;
        // Zilx helpers: assigned here so every opcode path drives them (no
        // latch). Only OPC_AMO below reads/overrides zilx_ok.
        is_unscaled = (funct5 == 5'b10010);  // lx  (unscaled)
        is_scaled = (funct5 == 5'b11010);  // lxs (scaled)
        size_ok_rv32 = (funct3 inside {3'b000, 3'b001, 3'b010, 3'b100, 3'b101});
        zilx_ok = 1'b0;

        unique case (opcode)
            OPC_LUI: begin
                reg_write   = 1'b1;
                alu_op      = ALU_ADD;
                alu_src_a   = ALU_A_RS1;  // rs1 forced to x0 -> 0
                alu_src_b   = ALU_B_IMM;
                wb_src      = WB_ALU;
                imm         = imm_u;
                dec_illegal = 1'b0;
            end
            OPC_AUIPC: begin
                reg_write   = 1'b1;
                alu_op      = ALU_ADD;
                alu_src_a   = ALU_A_PC;
                alu_src_b   = ALU_B_IMM;
                wb_src      = WB_ALU;
                imm         = imm_u;
                dec_illegal = 1'b0;
            end
            OPC_JAL: begin
                reg_write   = 1'b1;
                alu_op      = ALU_ADD;
                alu_src_a   = ALU_A_PC;
                // ALU_B_IMM, not the pc_link this used to select: JAL's
                // alu_result is never read (rd gets pc_link via WB_PC4, the
                // redirect target comes from execute's own branch_target
                // adder), so the operand is a don't-care -- and selecting
                // pc_link put an adder output into the operand-B mux, which
                // feeds the MUL DSP. See alu_src_b_t in rv32_pkg.
                alu_src_b   = ALU_B_IMM;
                wb_src      = WB_PC4;
                branch_type = BR_JAL;
                imm         = imm_j;
                dec_illegal = 1'b0;
            end
            OPC_JALR: begin
                if (funct3 == 3'b000) begin
                    reg_write   = 1'b1;
                    alu_op      = ALU_ADD;
                    alu_src_a   = ALU_A_RS1;
                    alu_src_b   = ALU_B_IMM;
                    wb_src      = WB_PC4;
                    branch_type = BR_JALR;
                    uses_rs1    = 1'b1;
                    imm         = imm_i;
                    dec_illegal = 1'b0;
                end
            end
            OPC_BRANCH: begin
                alu_op    = ALU_SUB;  // placeholder; branch resolves in execute
                alu_src_a = ALU_A_RS1;
                alu_src_b = ALU_B_RS2;
                uses_rs1  = 1'b1;
                uses_rs2  = 1'b1;
                imm       = imm_b;
                unique case (funct3)
                    3'b000: begin
                        branch_type = BR_BEQ;
                        dec_illegal = 1'b0;
                    end
                    3'b001: begin
                        branch_type = BR_BNE;
                        dec_illegal = 1'b0;
                    end
                    3'b100: begin
                        branch_type = BR_BLT;
                        dec_illegal = 1'b0;
                    end
                    3'b101: begin
                        branch_type = BR_BGE;
                        dec_illegal = 1'b0;
                    end
                    3'b110: begin
                        branch_type = BR_BLTU;
                        dec_illegal = 1'b0;
                    end
                    3'b111: begin
                        branch_type = BR_BGEU;
                        dec_illegal = 1'b0;
                    end
                    default: ;  // funct3 010/011 reserved -> illegal
                endcase
            end
            OPC_LOAD: begin
                reg_write = 1'b1;
                alu_op    = ALU_ADD;  // base + offset
                alu_src_a = ALU_A_RS1;
                alu_src_b = ALU_B_IMM;
                wb_src    = WB_MEM;
                mem_read  = 1'b1;
                uses_rs1  = 1'b1;
                imm       = imm_i;
                unique case (funct3)
                    3'b000: begin
                        mem_size     = MS_B;
                        mem_unsigned = 1'b0;
                        dec_illegal  = 1'b0;
                    end  // lb
                    3'b001: begin
                        mem_size     = MS_H;
                        mem_unsigned = 1'b0;
                        dec_illegal  = 1'b0;
                    end  // lh
                    3'b010: begin
                        mem_size     = MS_W;
                        mem_unsigned = 1'b0;
                        dec_illegal  = 1'b0;
                    end  // lw
                    3'b100: begin
                        mem_size     = MS_B;
                        mem_unsigned = 1'b1;
                        dec_illegal  = 1'b0;
                    end  // lbu
                    3'b101: begin
                        mem_size     = MS_H;
                        mem_unsigned = 1'b1;
                        dec_illegal  = 1'b0;
                    end  // lhu
                    default: ;  // 011/110/111 reserved (RV32) -> illegal
                endcase
            end
            OPC_STORE: begin
                alu_op    = ALU_ADD;  // base + offset
                alu_src_a = ALU_A_RS1;
                alu_src_b = ALU_B_IMM;
                mem_write = 1'b1;
                uses_rs1  = 1'b1;
                uses_rs2  = 1'b1;
                imm       = imm_s;
                unique case (funct3)
                    3'b000: begin
                        mem_size    = MS_B;
                        dec_illegal = 1'b0;
                    end  // sb
                    3'b001: begin
                        mem_size    = MS_H;
                        dec_illegal = 1'b0;
                    end  // sh
                    3'b010: begin
                        mem_size    = MS_W;
                        dec_illegal = 1'b0;
                    end  // sw
                    default: ;  // RV32 has no sd -> illegal
                endcase
            end
            OPC_OP_IMM: begin
                reg_write   = 1'b1;
                alu_src_a   = ALU_A_RS1;
                alu_src_b   = ALU_B_IMM;
                uses_rs1    = 1'b1;
                imm         = imm_i;
                dec_illegal = 1'b0;
                unique case (funct3)
                    3'b000:  alu_op = ALU_ADD;  // addi
                    3'b010:  alu_op = ALU_SLT;  // slti
                    3'b011:  alu_op = ALU_SLTU;  // sltiu
                    3'b100:  alu_op = ALU_XOR;  // xori
                    3'b110:  alu_op = ALU_OR;  // ori
                    3'b111:  alu_op = ALU_AND;  // andi
                    3'b001: begin  // slli — funct7 must be 0000000
                        alu_op = ALU_SLL;
                        if (funct7 != 7'b0000000) dec_illegal = 1'b1;
                    end
                    3'b101: begin  // srli / srai — funct7 0000000 or 0100000
                        alu_op = funct7b5 ? ALU_SRA : ALU_SRL;
                        if (funct7 != 7'b0000000 && funct7 != 7'b0100000) dec_illegal = 1'b1;
                    end
                    default: dec_illegal = 1'b1;
                endcase
            end
            OPC_OP: begin
                reg_write   = 1'b1;
                alu_src_a   = ALU_A_RS1;
                alu_src_b   = ALU_B_RS2;
                uses_rs1    = 1'b1;
                uses_rs2    = 1'b1;
                dec_illegal = 1'b0;
                if (is_m_ext) begin
                    unique case (funct3)
                        3'b000:  alu_op = ALU_MUL;
                        3'b001:  alu_op = ALU_MULH;
                        3'b010:  alu_op = ALU_MULHSU;
                        3'b011:  alu_op = ALU_MULHU;
                        3'b100:  alu_op = ALU_DIV;
                        3'b101:  alu_op = ALU_DIVU;
                        3'b110:  alu_op = ALU_REM;
                        3'b111:  alu_op = ALU_REMU;
                        default: dec_illegal = 1'b1;
                    endcase
                end else begin
                    unique case (funct3)
                        3'b000:  alu_op = funct7b5 ? ALU_SUB : ALU_ADD;  // add/sub
                        3'b001:  alu_op = ALU_SLL;  // sll
                        3'b010:  alu_op = ALU_SLT;  // slt
                        3'b011:  alu_op = ALU_SLTU;  // sltu
                        3'b100:  alu_op = ALU_XOR;  // xor
                        3'b101:  alu_op = funct7b5 ? ALU_SRA : ALU_SRL;  // srl/sra
                        3'b110:  alu_op = ALU_OR;  // or
                        3'b111:  alu_op = ALU_AND;  // and
                        default: dec_illegal = 1'b1;
                    endcase
                    // Base R-type: funct7 must be 0000000 (or 0100000 for
                    // sub/sra, allowed above). Anything else is illegal.
                    if (funct7 != 7'b0000000 && funct7 != 7'b0100000) dec_illegal = 1'b1;
                end
            end
            OPC_AMO: begin
                // Zilx indexed loads (AMO opcode). Encoding:
                //   funct5=[31:27] mode, aq=[26]/rl=[25] (must be 0),
                //   rs2=[24:20]=base, rs1=[19:15]=index, funct3=[14:12]
                //   size/sign, rd=[11:7]. rs1=index, rs2=base — roles
                //   swapped vs base loads. EA = base + (index << shamt),
                //   computed in the ALU (ALU_LX); the load itself is the
                //   LSU's job (mem_read / WB_MEM) — not present yet, so
                //   control rides the D/E register to the debug taps.
                //   Real AMOs (funct5 not in the Zilx set) and RV64-only
                //   encodings decode to illegal=1 this phase.
                // is_unscaled / is_scaled / size_ok_rv32 / zilx_ok default
                // are set at the top of this always_comb (no latch —
                // declaring them here and assigning only in this branch
                // made GowinSynthesis infer a latch, EX2420/EX3101).
                // funct5 == 5'b11110 (lxsuw) is RV64-only -> illegal on RV32.

                // funct3 -> access size/sign (same map as OPC_LOAD).
                unique case (funct3)
                    3'b000: begin
                        mem_size     = MS_B;
                        mem_unsigned = 1'b0;
                    end
                    3'b001: begin
                        mem_size     = MS_H;
                        mem_unsigned = 1'b0;
                    end
                    3'b010: begin
                        mem_size     = MS_W;
                        mem_unsigned = 1'b0;
                    end
                    3'b100: begin
                        mem_size     = MS_B;
                        mem_unsigned = 1'b1;
                    end
                    3'b101: begin
                        mem_size     = MS_H;
                        mem_unsigned = 1'b1;
                    end
                    default: begin
                        mem_size     = MS_W;
                        mem_unsigned = 1'b0;
                    end
                    // 3'b011 / 3'b110 (RV64) and 3'b111 -> reserved
                endcase
                if (aq || rl) begin
                    // aq/rl reserved -> illegal (zilx_ok stays 0 from default)
                end else if (is_unscaled) begin
                    // lx: no unscaled byte (funct3[1:0]==00); RV64 sizes reserved.
                    if (size_ok_rv32 && funct3[1:0] != 2'b00) begin
                        zilx_ok   = 1'b1;
                        mem_shamt = 2'd0;  // unscaled
                    end
                end else if (is_scaled) begin
                    // lxs: byte/half/word, signed/unsigned all valid on RV32.
                    if (size_ok_rv32) begin
                        zilx_ok = 1'b1;
                        case (mem_size)
                            MS_B: mem_shamt = 2'd0;
                            MS_H: mem_shamt = 2'd1;
                            MS_W: mem_shamt = 2'd2;
                            default: mem_shamt = 2'd0;
                        endcase
                    end
                end
                // else: funct5=11110 (RV64) or real AMO -> illegal.

                if (zilx_ok) begin
                    dec_illegal = 1'b0;
                    reg_write   = 1'b1;
                    mem_read    = 1'b1;
                    alu_op      = ALU_LX;
                    alu_src_a   = ALU_A_RS2;  // base (rs2)
                    alu_src_b   = ALU_B_RS1;  // index (rs1); ALU_LX applies <<shamt
                    uses_rs1    = 1'b1;  // index
                    uses_rs2    = 1'b1;  // base
                    imm         = 32'd0;
                    wb_src      = WB_MEM;
                end
            end

            OPC_SYSTEM: begin
                uses_rs2 = 1'b0;  // CSR ops never read rs2
                // Zicsr CSR ops (funct3 != 0) + ecall/ebreak/mret/wfi
                // (funct3 == 0). Only the CSR ops execute this phase; the
                // trap-entry ops stay illegal (no trap machinery yet).
                //
                // RV32 Zicsr:
                //   funct3=001 CSRRW   rd <- csr; csr <- rs1
                //   funct3=010 CSRRS   rd <- csr; csr <- csr |  rs1
                //   funct3=011 CSRRC   rd <- csr; csr <- csr & ~rs1
                //   funct3=101 CSRRWI  rd <- csr; csr <- zimm
                //   funct3=110 CSRRSI  rd <- csr; csr <- csr |  zimm
                //   funct3=111 CSRRCI  rd <- csr; csr <- csr & ~zimm
                //
                // rd always receives the OLD csr value (WB_CSR); the
                // RMW side effect is computed in execute (csr_wdata_o /
                // csr_wren_o). CSRRS/CSRRC with rs1==0 (and CSRRSI/CSRRCI
                // with zimm==0) do NOT write the CSR — execute gates the
                // write; CSRRW always writes. decode only flags the op
                // (csr_wren=1, csr_op, csr_addr); the dec_illegal squash
                // below clears csr_wren for non-CSR / illegal encodings.
                // The ALU result is unused for CSR ops (wb_src=WB_CSR,
                // csr_wdata computed in execute), so no alu_src/alu_op
                // override is needed — defaults leave alu_result_valid=1
                // (combinational), which retires the op in one cycle.
                if (is_csr) begin
                    reg_write   = 1'b1;  // rd <- old CSR value
                    wb_src      = WB_CSR;
                    csr_wren    = 1'b1;  // execute qualifies the actual write
                    dec_illegal = 1'b0;
                    unique case (funct3)
                        3'b001: begin  // CSRRW
                            csr_op   = CSR_RW;
                            uses_rs1 = 1'b1;
                        end
                        3'b010: begin  // CSRRS
                            csr_op   = CSR_RS;
                            uses_rs1 = 1'b1;
                        end
                        3'b011: begin  // CSRRC
                            csr_op   = CSR_RC;
                            uses_rs1 = 1'b1;
                        end
                        3'b101: begin  // CSRRWI  (zimm = instr[19:15])
                            csr_op = CSR_RWI;
                            imm    = {27'b0, rs1_field};  // zimm, zero-extended
                        end
                        3'b110: begin  // CSRRSI
                            csr_op = CSR_RSI;
                            imm    = {27'b0, rs1_field};
                        end
                        3'b111: begin  // CSRRCI
                            csr_op = CSR_RCI;
                            imm    = {27'b0, rs1_field};
                        end
                        default: dec_illegal = 1'b1;  // unreachable (is_csr => funct3!=0)
                    endcase
                end else begin
                    // Trap-entry / return / halt ops (funct3 == 0). Retire
                    // with side effects handled in execute (redirect + CSR
                    // writes via the trap unit); NOT marked illegal. ecall/
                    // ebreak raise a sync trap (exc_req=1) the cycle they
                    // retire; mret/wfi are legal no-side-eff at decode.
                    // SFENCE.VMA (no VM) stays illegal -> traps as illegal.
                    if (funct12 == 12'h000) begin  // ECALL -> env call from M-mode
                        sys_op      = SYS_ECALL;
                        dec_illegal = 1'b0;
                        exc_req     = 1'b1;
                        exc_cause   = MCAUSE_ECALL_M;
                        exc_tval    = '0;
                    end else if (funct12 == 12'h001) begin  // EBREAK -> breakpoint
                        sys_op      = SYS_EBREAK;
                        dec_illegal = 1'b0;
                        exc_req     = 1'b1;
                        exc_cause   = MCAUSE_BREAKPOINT;
                        exc_tval    = '0;
                    end else if (funct7 == 7'b0001000 && rs2_field == 5'b00101 &&
                                 rs1_field == 5'b00000) begin  // WFI
                        sys_op      = SYS_WFI;
                        dec_illegal = 1'b0;
                    end else if (funct7 == 7'b0011000 && rs2_field == 5'b00010 &&
                                 rs1_field == 5'b00000) begin  // MRET
                        sys_op      = SYS_MRET;
                        dec_illegal = 1'b0;
                    end else if (funct7 == 7'b0001001) begin  // SFENCE.VMA -> illegal (no VM)
                        dec_illegal = 1'b1;
                    end else begin
                        dec_illegal = 1'b1;
                    end
                end
            end

            OPC_MISC_MEM: begin
                // Zifencei: fence (funct3=0) / fence.i (funct3=1). Harvard
                // in-order single-core with no D->I path -> both are legal
                // nops here (no ordering side effect needed). Retire as
                // single-cycle nops (no reg/mem/csr/branch). Other funct3
                // reserved -> illegal.
                unique case (funct3)
                    3'b000: begin  // fence
                        sys_op      = SYS_FENCE;
                        dec_illegal = 1'b0;
                    end
                    3'b001: begin  // fence.i
                        sys_op      = SYS_FENCE_I;
                        dec_illegal = 1'b0;
                    end
                    default: dec_illegal = 1'b1;
                endcase
            end

            default: begin
                // Atomics, unknown opcodes -> illegal (traps as illegal
                // instruction in execute). OPC_SYSTEM / OPC_MISC_MEM are
                // handled above.
                dec_illegal = 1'b1;
            end
        endcase

        // Register-read addresses: force x0 for instrs that don't use
        // rs1/rs2 (avoids reading a bogus field, e.g. LUI's rs1 is imm).
        rs1_addr_dec = uses_rs1 ? rs1_field : 5'd0;
        rs2_addr_dec = uses_rs2 ? rs2_field : 5'd0;
        csr_addr_dec = is_csr ? funct12 : 12'd0;  // CSR index is instr[31:20]

        // Any illegal (and present) instruction requests an illegal-instr
        // sync trap in execute. ecall/ebreak set exc_req explicitly above
        // (dec_illegal=0); misaligned-access traps are detected in execute
        // (need the EA) and override cause/tval there. Gate on
        // decoded_valid so a span_wait / target_span bubble (no real
        // instruction) does not raise a spurious trap.
        if (dec_illegal && decoded_valid) begin
            exc_req   = 1'b1;
            exc_cause = MCAUSE_ILLEGAL;
            exc_tval  = src_instr32;
        end

        // An access fault overrides the illegal decode of the don't-care
        // word above: there was no instruction to be illegal. mtval is the
        // address that could not be fetched, which is what makes the trap
        // useful -- it names where the pc went.
        if (take_fault) begin
            exc_req   = 1'b1;
            exc_cause = MCAUSE_INSTR_ACC;
            exc_tval  = fe_pc_i;
        end

        // ---- Branch prediction (prediction-at-decode) ----
        // Everything the predictor exports to fetch is decoded off the SOURCE
        // HALVES, not off src_instr32 and not out of the opcode case tree.
        // pred_redirect_valid_o and pred_redirect_addr_o both reach fetch's
        // issue_addr, and from there the I-mem address pins, in this same
        // cycle, so every level here is a level on that path. Measured on the
        // 2026-09-01 48.965 MHz run, second-worst path group (-0.307 ns):
        //   buffer head 2.271 -> c_expand/src_instr32 5.509 -> [case tree]
        //   -> imm 15.836 -> pred_dir_target 18.634 -> pc_q 22.543
        // i.e. ~3.2 ns of c_expand and ~10.3 ns of decoder in front of a
        // 32-bit carry chain and then a BSRAM address pin.
        //
        // This is a second decoder for three bits of information, so it is
        // only sound while it agrees EXACTLY with the real one. Execute gates
        // its mispredict recovery on de_i.branch_type (cf_resolving), so an
        // OVER-prediction -- steering fetch on an instruction the decoder does
        // not call control flow -- would never be corrected. The `ifdef
        // VERILATOR` block at the end of the file checks the agreement every
        // cycle of every run; sw/isa/bp_pred exercises the cases.
        //
        // Exhaustively, the decoder leaves branch_type != BR_NONE for:
        //   OPC_JAL                          (always legal)
        //   OPC_JALR   with funct3 == 000    (always legal)
        //   OPC_BRANCH with funct3 not in {010, 011}
        // and dec_illegal is 0 in all three. That is why ~dec_illegal is not a
        // term below: it was redundant, and it pulled the whole illegal cone
        // (every funct3/funct7 legality test in the case tree) onto the
        // redirect path.
        cf_c_jal = (src_half16[1:0] == 2'b01) &
            ((src_half16[15:13] == 3'b001) | (src_half16[15:13] == 3'b101));  // c.jal / c.j
        cf_c_cond = (src_half16[1:0] == 2'b01) & (src_half16[15:14] == 2'b11);  // c.beqz / c.bnez
        // c.jr / c.jalr: quadrant 2, funct3=100, rs2 field 0, rd field != 0
        // (rd==0 with rs2==0 is c.ebreak). c.jr (c[12]=0) expands to rd=x0 and
        // is the return form; c.jalr (c[12]=1) expands to rd=x1, never one.
        cf_c_jalr = (src_half16[1:0] == 2'b10) & (src_half16[15:13] == 3'b100) &
            (src_half16[6:2] == 5'd0) & (src_half16[11:7] != 5'd0);
        cf_c_ret = cf_c_jalr & ~src_half16[12] &
            ((src_half16[11:7] == 5'd1) | (src_half16[11:7] == 5'd5));

        cf_n_jal = (src_word32[6:0] == OPC_JAL);
        cf_n_jalr = (src_word32[6:0] == OPC_JALR) & (src_word32[14:12] == 3'b000);
        // The reserved branch funct3 values 010 / 011 are exactly funct3[2:1]==01.
        cf_n_cond = (src_word32[6:0] == OPC_BRANCH) & (src_word32[14:13] != 2'b01);
        cf_n_ret = cf_n_jalr & (src_word32[11:7] == 5'd0) &
            ((src_word32[19:15] == 5'd1) | (src_word32[19:15] == 5'd5));

        cf_jal = src_is_compressed ? cf_c_jal : cf_n_jal;
        cf_cond = src_is_compressed ? cf_c_cond : cf_n_cond;
        cf_jalr = src_is_compressed ? cf_c_jalr : cf_n_jalr;
        cf_ret = src_is_compressed ? cf_c_ret : cf_n_ret;

        is_cf = decoded_valid & (cf_jal | cf_jalr | cf_cond);
        is_return_cf = decoded_valid & cf_ret;

        // Direct PC-relative target for JAL and conditional branches. JALR
        // targets are rs1+imm -- not PC-relative -- so this is unused there.
        // The predictor only ever needs two immediates and both are pure bit
        // permutations of a source half, so the select is one bit: compressed
        // c.j/c.jal have funct3[1]=0 and c.beqz/c.bnez funct3[1]=1; native
        // needs one opcode compare. c_expand packs the CJ / CB offset into a
        // J / B immediate, so unpacking the raw halfword here reproduces
        // exactly what `src_pc + imm` would have produced -- which is also
        // what execute resolves as `de_i.pc + de_i.imm`.
        pred_cj_imm = {
            {20{src_half16[12]}},
            src_half16[12],
            src_half16[8],
            src_half16[10],
            src_half16[9],
            src_half16[6],
            src_half16[7],
            src_half16[2],
            src_half16[11],
            src_half16[5],
            src_half16[4],
            src_half16[3],
            1'b0
        };
        pred_cb_imm = {
            {23{src_half16[12]}},
            src_half16[12],
            src_half16[6:5],
            src_half16[2],
            src_half16[11:10],
            src_half16[4:3],
            1'b0
        };
        pred_n_imm_j = {
            {11{src_word32[31]}},
            src_word32[31],
            src_word32[19:12],
            src_word32[20],
            src_word32[30:21],
            1'b0
        };
        pred_n_imm_b = {
            {19{src_word32[31]}},
            src_word32[31],
            src_word32[7],
            src_word32[30:25],
            src_word32[11:8],
            1'b0
        };
        pred_imm = src_is_compressed ? (src_half16[14] ? pred_cb_imm : pred_cj_imm) :
            ((src_word32[6:0] == OPC_JAL) ? pred_n_imm_j : pred_n_imm_b);
        pred_dir_target = src_pc + pred_imm;

        // Direction bit + gshare index, from whichever source this build uses.
        // The push-time form reads two flops (the entry's two halfword
        // predictions, selected by the PC being decoded) and one xor; the
        // decode-time form reads the PHT array.
        //
        // The halfword selector is fe_pc_i[1], not src_pc[1]: in every case
        // that reaches the selector src_pc IS fe_pc_i (span_complete and
        // is_hold_plain take the stashed copy instead, and the two wait states
        // have decoded_valid=0, so their prediction is never consumed). Using
        // fe_pc_i keeps the src_pc mux off the direction bit. A stashed
        // halfword always has bit[1]=1 and always carried the HIGH-half bit,
        // so one stashed bit is enough. Both facts are checked below under
        // `ifdef VERILATOR`.
        if (BP_PUSH_LOOKUP != 0) begin
            pht_taken_src = (span_complete || is_hold_plain) ?
                hold_pred_q : (fe_pc_i[1] ? fe_pred_hi_i : fe_pred_lo_i);
            pht_ghr_src = (span_complete || is_hold_plain) ? hold_ghr_q : fe_ghr_i;
            pht_index_src = src_pc[BP_PHT_IDX_W:1] ^ pht_ghr_src;
        end else begin
            pht_taken_src = bp_lookup_i.pht_taken;
            pht_ghr_src   = '0;
            pht_index_src = bp_lookup_i.pht_index;
        end

        // The flat forms fetch sees. cf_jal / cf_cond / cf_ret are mutually
        // exclusive (different opcodes), so the direction is an OR of ANDs
        // rather than an if-else chain, and the address is a single 2:1 with
        // the adder output -- the late input -- on it. Both are qualified by
        // is_cf at the output, so they are don't-cares otherwise.
        cf_taken       = cf_jal | (cf_cond & pht_taken_src) | (cf_ret & bp_lookup_i.ras_valid);
        cf_target      = cf_ret ? bp_lookup_i.ras_top : pred_dir_target;

        // Build the D/E prediction metadata. pred_valid marks a classified
        // control-flow instruction (records NT predictions and unpredicted
        // JALR for uniform mispredict accounting in execute); pred_taken is
        // the speculated direction; pred_target is the taken target (valid
        // when pred_taken). These all end at a flop, so they keep the gated
        // shape -- only the two fetch-facing outputs use the flat forms.
        pred_valid     = is_cf;
        pred_source    = PRED_NONE;
        pred_taken     = 1'b0;
        pred_target    = '0;
        pred_pht_index = '0;
        if (is_cf) begin
            pred_taken = cf_taken;
            if (cf_ret) begin
                // Return: target from the RAS; taken only if the RAS has an
                // entry (else fall back to execute, the legacy path).
                pred_source = PRED_RAS;
                pred_target = bp_lookup_i.ras_top;
            end else if (cf_cond) begin
                // Conditional: direction from the gshare PHT, target pc+imm.
                pred_source    = PRED_PHT;
                pred_target    = pred_dir_target;
                pred_pht_index = pht_index_src;
            end else if (cf_jal) begin
                // Unconditional JAL / c.j / c.jal: always taken, pc+imm.
                pred_source = PRED_DIRECT;
                pred_target = pred_dir_target;
            end
            // JALR call / indirect (JALR, not a return): no target prediction
            // (rs1+imm is data-dependent); cf_taken is 0 for it and execute
            // resolves it as before the predictor existed.
        end

        // ---- Assemble the D/E word for this cycle ----
        de_d                 = '0;
        de_d.valid           = decoded_valid;
        de_d.pc              = src_pc;
        de_d.instr           = src_instr32;  // 32-bit word decode treated
        de_d.is_compressed   = src_is_compressed;
        de_d.rs1_addr        = rs1_addr_dec;
        //        de_d.rs1_data      = rs1_data_i;
        //        de_d.rs2_data      = rs2_data_i;
        de_d.rs1_data        = rs1_fwd;
        de_d.rs2_data        = rs2_fwd;
        de_d.imm             = imm;
        de_d.rd              = rd_field;
        de_d.alu_op          = alu_op;
        de_d.alu_src_a       = alu_src_a;
        de_d.alu_src_b       = alu_src_b;
        de_d.mem_size        = mem_size;
        de_d.mem_unsigned    = mem_unsigned;
        de_d.mem_shamt       = mem_shamt;
        de_d.wb_src          = wb_src;
        de_d.branch_type     = branch_type;
        de_d.sys_op          = sys_op;
        de_d.exception       = exc_req;
        de_d.exception_cause = exc_cause;
        de_d.exception_tval  = exc_tval;
        de_d.csr_op          = csr_op;
        de_d.csr_addr        = csr_addr_dec;
        // Branch-prediction metadata. When BP_EN=0 the predictor is disabled:
        // zero the metadata so execute resolves every control-flow instruction
        // exactly as before (pred_valid=0 -> pred_t=0 -> mispredict reduces to
        // the legacy taken-redirect, and no predicted redirect fires).
        de_d.pred_valid      = (BP_EN != 0) & pred_valid;
        de_d.pred_taken      = (BP_EN != 0) & pred_taken;
        de_d.pred_target     = (BP_EN != 0) ? pred_target : '0;
        de_d.pred_source     = (BP_EN != 0) ? pred_source : PRED_NONE;
        de_d.pred_pht_index  = (BP_EN != 0) ? pred_pht_index : '0;
        // reg_write / mem_read / mem_write / csr_wren are squashed by
        // illegal. A spanning stitch decodes a real 32-bit instr through
        // the uniform decoder, so spanning no longer forces illegal.
        if (dec_illegal) begin
            de_d.reg_write   = 1'b0;
            de_d.mem_read    = 1'b0;
            de_d.mem_write   = 1'b0;
            de_d.csr_wren    = 1'b0;
            de_d.branch_type = BR_NONE;
            de_d.illegal     = decoded_valid;  // illegal=1 only when a word was present
        end else begin
            de_d.reg_write = reg_write;
            de_d.mem_read  = mem_read;
            de_d.mem_write = mem_write;
            de_d.csr_wren  = csr_wren;
            de_d.illegal   = 1'b0;
        end
    end

    // Predicted redirect to fetch: fire the cycle a control-flow instruction
    // at the head is consumed into de_d (~stall_i, ~flush_i) and predicted
    // taken. Gated by BP_EN; lower priority than execute's redirect (a
    // trap / mret / interrupt / mispredict flushes decode via flush_i and
    // overrides). Also used to clear the RVC hold buffer below: a
    // predicted-taken redirect must drop any stashed half exactly like an
    // execute flush does, because the half-instruction sitting after the
    // control-flow instr is wrong-path, and the fetch buffer kill does NOT
    // see the hold stash -- without this the fall-through half after a
    // predicted-taken spanning branch survives the redirect and retires.
    // Declared here because the hold next-state below reads it.
    wire pred_redirect_fire = (BP_EN != 0) & is_cf & cf_taken & ~stall_i & ~flush_i;

    // =================================================================
    // Hold-buffer next-state
    // =================================================================
    always_comb begin
        // Defaults: hold (preserve the stash).
        hold_d      = hold_q;
        hold_word_d = hold_word_q;
        hold_pc_d   = hold_pc_q;
        hold_pred_d = hold_pred_q;
        hold_ghr_d  = hold_ghr_q;

        if (flush_i) begin
            hold_d = 1'b0;  // execute redirect: drop any stashed half
        end else if (pred_redirect_fire) begin
            // Predicted-taken redirect: the half sitting after this control-
            // flow instr is wrong-path, drop it. The span_complete /
            // buffer_upper / target_span_complete branches below would
            // otherwise re-stash that half this cycle (the fetch buffer kill
            // does not reach the hold stash), letting it retire.
            hold_d = 1'b0;
        end else if (take_fault) begin
            hold_d = 1'b0;  // the unfetchable half is gone; drop the stash
        end else if (stall_i) begin
            // DIV-REM / mem-wait stall: preserve the stash.
        end else if (span_wait) begin
            // Spanning low half held, upper-half word not here yet:
            // keep waiting (preserve). Do NOT stall fetch -- it must
            // capture word W+1 -- and stall_o reflects that (no term).
        end else if (span_complete) begin
            // Stitch consumed the spanning low (hold) AND fe_instr's low
            // half (bytes 0-1 of word W+1). The next instruction is
            // fe_instr's upper half (bytes 2-3 of W+1) -> stash it
            // unconditionally (its [1:0] flags compressed vs another
            // spanning low; consecutive spanning instrs fall through
            // here). PC = fe_pc + 2.
            hold_d      = 1'b1;
            hold_word_d = fe_instr_i[31:16];
            hold_pc_d   = fe_pc_i + 32'd2;
            hold_pred_d = fe_pred_hi_i;
            hold_ghr_d  = fe_ghr_i;
        end else if (is_hold_plain) begin
            // Compressed upper half consumed; clear.
            hold_d = 1'b0;
        end else if (target_span_complete) begin
            // Same-cycle stitch consumed head (target word: low half branched
            // over, upper half = 32-bit instr's low half) AND head+1's low half
            // (the instr's upper half). The next instruction is head+1's upper
            // half -> stash it (its [1:0] flags compressed vs another spanning
            // low; consecutive spanning instrs chain here). PC = head+1 PC + 2.
            // span_complete (needs hold_q) and target_span_complete (needs
            // !hold_q) are mutually exclusive, so the if-else priority holds.
            hold_d      = 1'b1;
            hold_word_d = fe_next_instr_i[31:16];
            hold_pc_d   = fe_next_pc_i + 32'd2;
            hold_pred_d = fe_next_pred_hi_i;
            hold_ghr_d  = fe_next_ghr_i;
        end else if (target_span_wait) begin
            // Branch target at offset 2/6 is a 32-bit instr's low half, but
            // head+1 is absent or faults: stash the low half and wait for the
            // upper-half word (the old target_span behavior). The spanning
            // instr sits AT the target, so hold_pc = fe_pc (not +2).
            hold_d      = 1'b1;
            hold_word_d = fe_instr_i[31:16];
            hold_pc_d   = fe_pc_i;
            hold_pred_d = fe_pred_hi_i;
            hold_ghr_d  = fe_ghr_i;
        end else if (buffer_upper) begin
            // Decoded a fresh low compressed half; stash the upper half
            // (PC = word_pc + 2). Its [1:0] flags a spanning low.
            hold_d      = 1'b1;
            hold_word_d = fe_instr_i[31:16];
            hold_pc_d   = fe_pc_i + 32'd2;
            hold_pred_d = fe_pred_hi_i;
            hold_ghr_d  = fe_ghr_i;
        end else begin
            hold_d = 1'b0;  // nothing to stash
        end
    end

    // =================================================================
    // D/E register next-state
    // =================================================================
    always_comb begin
        if (flush_i) begin
            de_next = '0;
        end else if (stall_i) begin
            de_next = de_q;  // DIV/REM / mem-wait stall: hold
        end else begin
            de_next = de_d;
        end
    end

    // =================================================================
    // Back-pressure to fetch: a compressed-upper hold, or execute's DIV/REM /
    // mem-wait stall. RAW is NOT a source of back-pressure -- the forward path
    // resolves it without stalling fetch or bubbling D/E.
    //
    // resource_stall fires when a stashed compressed upper half collides with
    // a fresh F/D word: the stash takes priority, so hold F/D. The
    // !hold_is_span folded into hold_plain_pending is what makes that safe --
    // a SPANNING hold with fe_valid is span_complete, which CONSUMES word W+1,
    // and stalling there would make the stitch re-see W+1. span_wait has
    // fe_valid=0 and so contributes no term. The same stall is what parks a
    // fault entry at the head while the stash ahead of it retires (see
    // take_fault). Named rather than inlined because sim_main.cpp taps it for
    // the rvc-hold stall bucket.
    // =================================================================
    wire resource_stall = hold_plain_pending && fe_valid_i;
    assign stall_o                 = resource_stall || stall_i;

    // Same-cycle target-span stitch: pop 2 (head + head+1). target_span_complete
    // has hold_q=0 so resource_stall=0; only stall_i back-pressures, which fetch
    // already gates out of buf_pop_cnt. Asserted only when fe_next_valid_i
    // (count>=2) -> pop-2 <= count.
    assign fe_pop2_o               = target_span_complete;

    // Branch-predictor lookup outputs (PC only, no register data). The
    // predictor exports the PHT entry and the RAS top unconditionally and the
    // selection above picks between them, so no kind bits are needed. The one
    // extra field is the sim-only return-lookup event, which carries decode's
    // consume condition: bp_lookup_o is a held level, so a return waiting out
    // an execute stall must not be counted once per waiting cycle.
    assign bp_lookup_o.pc          = src_pc;
    assign bp_lookup_o.ret_consume = is_return_cf & ~stall_i & ~flush_i;

    // The address fetch launches on. cf_target rather than pred_target: they
    // are equal whenever pred_redirect_valid_o is high (is_cf & cf_taken), and
    // cf_target is a single 2:1 off the adder instead of the is_cf/kind-gated
    // chain the D/E field keeps.
    assign pred_redirect_valid_o   = pred_redirect_fire;
    assign pred_redirect_addr_o    = cf_target;

    // Register-read addresses drive the reg file.
    assign rs1_addr_o              = rs1_addr_dec;
    assign rs2_addr_o              = rs2_addr_dec;

    // D/E output
    assign de_o                    = de_q;

    // de_* per-stage taps (fields of de_o / de_q).
    assign de_pc_o                 = de_q.pc;
    assign de_instr_o              = de_q.instr;
    assign de_valid_o              = de_q.valid;

    // =================================================================
    // Sequential
    // =================================================================
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            hold_q      <= 1'b0;
            hold_word_q <= 32'd0;
            hold_pc_q   <= 32'd0;
            hold_pred_q <= 1'b0;
            hold_ghr_q  <= '0;
            de_q        <= '0;
        end else begin
            hold_q      <= hold_d;
            hold_word_q <= hold_word_d;
            hold_pc_q   <= hold_pc_d;
            hold_pred_q <= hold_pred_d;
            hold_ghr_q  <= hold_ghr_d;
            de_q        <= de_next;
        end
    end

`ifdef VERILATOR
    // =================================================================
    // Predictor fast-path equivalence (simulation only)
    //
    // The predictor decodes its own copy of "is this control flow, of what
    // kind, and what is its PC-relative immediate" off the raw source halves,
    // so that neither c_expand nor the opcode case tree sits on the path to
    // fetch's issue_addr. That is only sound while the copy agrees exactly
    // with the real decoder, and one direction of disagreement is not
    // self-correcting: execute qualifies its mispredict recovery with
    // de_i.branch_type (cf_resolving), so a redirect fired for an instruction
    // the decoder does not call control flow would never be undone.
    //
    // Checked here rather than by a directed test because the interesting
    // inputs are whatever the compiler happens to emit. Every oracle, cosim
    // and benchmark run is therefore also an equivalence proof over the
    // instruction words it executes.
    // =================================================================
    logic slow_is_cf, slow_is_cond_cf, slow_is_return_cf;
    always_comb begin
        slow_is_cf = decoded_valid & ~dec_illegal & (branch_type != BR_NONE);
        slow_is_cond_cf = slow_is_cf &
            (branch_type inside {BR_BEQ, BR_BNE, BR_BLT, BR_BGE, BR_BLTU, BR_BGEU});
        slow_is_return_cf = slow_is_cf & (branch_type == BR_JALR) &
            (rs1_field == 5'd1 || rs1_field == 5'd5) & (rd_field == 5'd0);
    end

    always_ff @(posedge clk_i) begin
        if (rstn_i) begin
            if (is_cf !== slow_is_cf)
                $fatal(
                    1,
                    "decode: fast is_cf=%b vs decoder %b (pc=%08x instr=%08x)",
                    is_cf,
                    slow_is_cf,
                    src_pc,
                    src_instr32
                );
            if ((decoded_valid & cf_cond) !== slow_is_cond_cf)
                $fatal(
                    1,
                    "decode: fast cond CF=%b vs decoder %b (pc=%08x instr=%08x)",
                    decoded_valid & cf_cond,
                    slow_is_cond_cf,
                    src_pc,
                    src_instr32
                );
            if (is_return_cf !== slow_is_return_cf)
                $fatal(
                    1,
                    "decode: fast is_return_cf=%b vs decoder %b (pc=%08x instr=%08x)",
                    is_return_cf,
                    slow_is_return_cf,
                    src_pc,
                    src_instr32
                );
            // The immediate only has to match where it is consumed: a
            // PC-relative prediction, i.e. PRED_DIRECT (JAL) or PRED_PHT
            // (conditional). Execute resolves those as de_i.pc + de_i.imm.
            if (is_cf && (cf_jal || cf_cond) &&
                (pred_imm !== ((opcode == OPC_JAL) ? imm_j : imm_b)))
                $fatal(
                    1,
                    "decode: pred_imm=%08x vs decoder imm=%08x (pc=%08x instr=%08x)",
                    pred_imm,
                    (opcode == OPC_JAL) ? imm_j : imm_b,
                    src_pc,
                    src_instr32
                );
            // pht_taken_src selects the halfword slot with fe_pc_i[1] rather
            // than src_pc[1], which is only equal because src_pc IS fe_pc_i in
            // every case that reaches that selector.
            if (is_cf && !(span_complete || is_hold_plain) && (src_pc !== fe_pc_i))
                $fatal(
                    1,
                    "decode: src_pc=%08x != fe_pc=%08x with the head-entry prediction",
                    src_pc,
                    fe_pc_i
                );
            // Every stashed halfword is a high half, which is what lets the
            // hold buffer carry one prediction bit instead of two.
            if (hold_q && !hold_pc_q[1])
                $fatal(1, "decode: stashed pc=%08x is not a high half", hold_pc_q);
        end
    end
`endif

endmodule

`resetall

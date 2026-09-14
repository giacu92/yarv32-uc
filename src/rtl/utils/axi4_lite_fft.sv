`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* FFT coprocessor (AXI4-Lite slave) with a private ping-pong sample store.
*
* Radix-2 decimation-in-frequency, in fixed point (signed Q15 complex),
* one butterfly per cycle. The sample buffers are the peripheral's OWN
* BSRAM, not the CPU's D-mem: the point of the design is that the engine
* computes frame k out of one buffer while the CPU fills frame k+1 through
* the other, so a continuous input stream never stalls the transform and
* the transform never stalls the stream.
*
* ============================ ADDRESS MAP =========================
*
* The window is 8 KiB, split in half by addr[12]:
*
*   FFT_BASE + 0x0000 .. 0x0FFF   CSR page (six registers, see below)
*   FFT_BASE + 0x1000 .. 0x1FFF   DATA page: NMAX complex samples, one
*                                 32-bit word each, {imag[31:16],
*                                 real[15:0]}, both signed Q15.
*
* DATA-page writes are honoured at HALFWORD granularity, not byte: a word
* store writes both halves, a halfword store writes the real or the
* imaginary part alone, and a BYTE store writes NOTHING (answered OKAY).
* That is a property of the storage, not a choice -- a Gowin DPB has no
* per-byte write enable, and asking for one is what made the array fall
* out of block RAM entirely (see fft_dpram.sv). A complex sample is two
* 16-bit fields; there is no meaningful byte inside one.
*
* The DATA page always shows the buffer the CPU owns. Which physical
* buffer that is flips on every START (and on an explicit SWAP); the
* current owner is readable as STATUS.CPU_BUF.
*
* CSR page (word-addressed, byte offsets):
*
*   0x00 CTRL   (RW) bit0 START   (W1, self-clearing, ignored while BUSY)
*                    bit1 IRQ_EN  (RW)
*                    bit2 INVERSE (RW, latched at START)
*                    bit3 SWAP    (W1, self-clearing, ignored while BUSY)
*   0x04 STATUS (R)  bit0 BUSY, bit1 DONE (sticky), bit2 CPU_BUF,
*                    bit3 IRQ (= DONE & IRQ_EN, the line into the PLIC)
*   0x08 IRQ    (R/W1C) bit0 DONE -- write 1 to clear the sticky flag
*   0x0C CONF   (RW) [3:0] LOG2N, clamped to [2, LOG2NMAX] ON WRITE, so a
*                    readback always reports the value actually in force.
*                    Writes are IGNORED while BUSY.
*   0x10 SCALE  (RW) [LOG2NMAX-1:0], bit s = divide by 2 after stage s.
*                    Reset all-ones (the overflow-proof 1/N mode, below).
*   0x14 CYCLES (R)  clk cycles the last transform took, START to DONE.
*
* ======================= PING-PONG PROTOCOL =======================
*
* START does two things atomically: it hands the buffer the CPU was
* filling to the engine, and it hands the engine's previous buffer -- now
* holding the previous transform's RESULT -- to the CPU. So the steady
* state is:
*
*   fill buffer  -> START -> (read previous result | fill next frame)
*                         -> poll DONE / take the IRQ -> START -> ...
*
* Results are therefore one frame behind: the output of frame k becomes
* CPU-visible at the START of frame k+1. To drain the last frame without
* launching another transform, write SWAP instead of START.
*
* A buffer is never engine-owned and CPU-owned at the same time, which is
* what lets each buffer's port A be muxed between the two (port B is
* engine-only). That is the whole reason two dual-port BSRAMs suffice for
* three simultaneous accessors.
*
* ========================= NUMERIC BEHAVIOUR ======================
*
* Butterfly (DIF):  A' = A + B ;  B' = (A - B) * W_N^k
*
* with W_N^k read from the Q15 forward table (fft_twiddle_rom.sv). INVERSE
* negates the twiddle's imaginary half, which turns the same structure
* into the unnormalised inverse DFT; combined with the default 1/N scaling
* that is the true inverse.
*
* SCALE bit s divides both butterfly outputs by 2 after stage s, with
* round-half-up. With every bit set (the reset value) the total gain is
* 1/N and NO stage can overflow -- |A|,|B| <= 1 implies |A+B|/2 <= 1 and
* |(A-B)*W|/2 <= 1. Clearing scale bits keeps more low-order bits and is
* the right choice for small signals, at the cost of having to know the
* input's headroom. Every write saturates to Q15 rather than wrapping, so
* an overflow degrades instead of inverting the sign.
*
* Rounding: the Q15 product normalisation adds 1<<14 before the >>> 15,
* and each scaling shift adds 1 before the >>> 1. Both are round-half-up,
* which is the cheapest rounding that does not bias towards zero, and the
* bit-exact reference in sim/hw/fft_tb models them exactly.
*
* Output ORDER IS NATURAL. A DIF transform leaves bin X[k] at array
* position bitrev(k); rather than pay a separate unscramble pass, the
* FINAL stage's write address is bit-reversed. That is a wire permutation
* plus one barrel shift (for LOG2N < LOG2NMAX) and costs zero cycles.
*
* =========================== STRUCTURE ============================
*
* Three memories of NMAX complex words:
*
*   u_buf0 / u_buf1 : the ping-pong pair. Port A muxed CPU/engine, port B
*                     engine-only.
*   u_scratch       : engine-private working buffer.
*
* Stage s reads one memory and writes the other, alternating: even stages
* read the engine's buffer and write the scratch, odd stages the reverse.
* NOT in place -- an in-place stage would need two reads AND two writes
* per cycle, i.e. four ports on a two-port memory. Ping-ponging between
* two memories gives exactly two reads on one and two writes on the other,
* which is what a pair of dual-port BSRAMs provides.
*
* A consequence of that alternation: after LOG2N stages the result sits in
* the engine's buffer only if LOG2N is EVEN. For odd LOG2N the engine runs
* one extra pass (`is_copy`) that moves the scratch back into the buffer,
* and the bit-reversal rides on that pass instead. Cost N/2 cycles on a
* transform that takes LOG2N*N/2, i.e. under 1/3 of a stage.
*
* Timing: (LOG2N + odd) * (N/2 + PIPE) cycles, PIPE = 4. For N = 1024 that
* is 10 * 516 = 5160 cycles = 103 us at 50 MHz.
*
* =========================== PIPELINE =============================
*
*   T+0  issue        addresses a,b + twiddle index from the counters
*   T+1  memory out   sample pair and twiddle word land (registered RAM)
*   T+2  add/sub      sum = A+B, dif = A-B, twiddle sign-select
*   T+3  multiply     four 17x16 signed products (DSP)
*   T+4  combine      +/- , >>>15, scale, saturate; write port driven
*
* A stage does not start until the previous one has drained (all four
* pipeline slots empty), because the next stage reads the memory this one
* is still writing. That is the PIPE=4 term above.
*
* Reset is synchronous, active-low, and does NOT clear the sample
* memories -- see fft_dpram.sv.
*
* Naming: ports *_i/_o; internals no prefix; flops _q, next-state _d.
*/

module axi4_lite_fft #(
    // Maximum transform size, in complex samples. Must be a power of two.
    // The DATA page is NMAX words, so NMAX * 4 bytes must fit the 4 KiB
    // half-window: NMAX <= 1024.
    parameter int unsigned NMAX = 1024,
    // log2(NMAX). Kept as a parameter (not $clog2) so an inconsistent
    // override is an elaboration error rather than a silent resize.
    parameter int unsigned LOG2NMAX = 10
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    // Level-sensitive interrupt: DONE & IRQ_EN.
    output wire fft_irq_o
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;

    // Sample-buffer address width, butterfly-counter width, twiddle ROM.
    localparam int unsigned AW = LOG2NMAX;
    localparam int unsigned BF_W = LOG2NMAX - 1;
    localparam int unsigned TW_DEPTH = NMAX / 2;
    localparam int unsigned TW_AW = LOG2NMAX - 1;

    // Issue -> write-complete distance, in cycles (see the pipeline map).
    localparam int unsigned PIPE = 4;

    if (NMAX != (1 << LOG2NMAX)) begin : g_size_check
        $error("axi4_lite_fft: NMAX=%0d does not match LOG2NMAX=%0d", NMAX, LOG2NMAX);
    end
    if (NMAX > 1024) begin : g_window_check
        $error("axi4_lite_fft: NMAX=%0d needs more than the 4 KiB DATA page", NMAX);
    end

    // CSR word offsets (byte addr[4:2]).
    localparam logic [2:0] REG_CTRL = 3'd0;
    localparam logic [2:0] REG_STATUS = 3'd1;
    localparam logic [2:0] REG_IRQ = 3'd2;
    localparam logic [2:0] REG_CONF = 3'd3;
    localparam logic [2:0] REG_SCALE = 3'd4;
    localparam logic [2:0] REG_CYCLES = 3'd5;

    // =================================================================
    // Helper functions
    // =================================================================

    // Reverse all AW bits. The runtime LOG2N shift is applied by the
    // caller: reversing AW bits and then shifting right by
    // LOG2NMAX-LOG2N is the n-bit reversal of the low n bits.
    function automatic logic [AW-1:0] brev_full(input logic [AW-1:0] x);
        for (int i = 0; i < AW; i++) begin
            brev_full[i] = x[AW-1-i];
        end
    endfunction

    // Round-half-up arithmetic right shift by 1, gated. Used for the
    // per-stage SCALE divide.
    function automatic logic signed [20:0] scale_rnd(input logic signed [20:0] v, input logic en);
        scale_rnd = en ? ((v + 21'sd1) >>> 1) : v;
    endfunction

    // Saturate to signed 16-bit. Saturating rather than wrapping means an
    // overflow shows up as a clipped magnitude, not as a sign inversion.
    function automatic logic signed [15:0] sat16(input logic signed [20:0] v);
        if (v > 21'sd32767) sat16 = 16'sh7FFF;
        else if (v < -21'sd32768) sat16 = 16'sh8000;
        else sat16 = v[15:0];
    endfunction

    // =================================================================
    // Control / status registers
    // =================================================================
    logic                irq_en_q;
    logic                inverse_q;  // CTRL bit, may change any time
    logic                inv_run_q;  // latched at START, used by the engine
    logic                done_q;
    logic                cpu_buf_q;
    logic [         3:0] log2n_q;
    logic [LOG2NMAX-1:0] scale_q;
    logic [        31:0] cycles_q;
    logic [        31:0] cyc_q;

    // =================================================================
    // Engine control state
    // =================================================================
    localparam logic [1:0] ST_IDLE = 2'd0;
    localparam logic [1:0] ST_RUN = 2'd1;
    localparam logic [1:0] ST_DRAIN = 2'd2;

    logic [     1:0] state_q;
    logic [     3:0] st_q;  // stage index, 0 .. total_stages-1
    logic [BF_W-1:0] bf_q;  // butterfly counter within the stage
    logic [BF_W-1:0] bf_last_q;  // N/2 - 1, latched at START
    logic [  AW-1:0] half_q;  // one-hot: N >> (s+1)
    logic [  AW-1:0] tw_step_q;  // twiddle exponent step, NMAX units
    logic [  AW-1:0] tw_idx_q;  // twiddle exponent, NMAX units
    logic [PIPE-1:0] vld_q;  // pipeline occupancy, bit i = slot T+1+i
    logic [     3:0] shamt_q;  // LOG2NMAX - LOG2N, for the bit reversal

    wire             busy = (state_q != ST_IDLE);

    // Number of passes: one per radix-2 stage, plus a copy pass when
    // LOG2N is odd (see the STRUCTURE note).
    wire  [     4:0] total_stages = {1'b0, log2n_q} + {4'b0, log2n_q[0]};
    wire             is_copy = ({1'b0, st_q} >= {1'b0, log2n_q});
    wire             is_final_stage = ({1'b0, st_q} == (total_stages - 5'd1));
    wire             scale_eff = is_copy ? 1'b0 : scale_q[st_q[3:0]];

    // =================================================================
    // Butterfly address generation
    //
    // At stage s the pair is the butterfly counter with a zero inserted
    // at bit position log2(half): a = insert0(bf), b = a | half. That is
    // a mask, a one-bit shift and an OR -- no barrel shifter -- because
    // `half` is carried as a one-hot register that shifts right once per
    // stage.
    // =================================================================
    wire  [BF_W-1:0] low_mask = half_q[BF_W-1:0] - {{(BF_W - 1) {1'b0}}, 1'b1};
    wire  [BF_W-1:0] bf_j = bf_q & low_mask;
    wire  [BF_W-1:0] bf_hi = bf_q & ~low_mask;
    wire  [  AW-1:0] iss_a = {bf_hi, 1'b0} | {{(AW - BF_W) {1'b0}}, bf_j};
    wire  [  AW-1:0] iss_b = iss_a | half_q;

    wire             j_last = (bf_j == low_mask);
    wire             bf_last = (bf_q == bf_last_q);
    wire             issue = (state_q == ST_RUN);

    // =================================================================
    // Pipeline slot 1 (T+1): the memory read is in flight. Carries the
    // addresses and the per-butterfly flags alongside it.
    // =================================================================
    logic [AW-1:0] p1_a_q, p1_b_q;
    logic p1_copy_q, p1_scale_q, p1_final_q;

    // Slot 2 (T+2): sums, differences, selected twiddle.
    logic signed [16:0] s2_sum_r_q, s2_sum_i_q;
    logic signed [16:0] s2_dif_r_q, s2_dif_i_q;
    logic signed [15:0] s2_wr_q, s2_wi_q;
    logic [AW-1:0] s2_a_q, s2_b_q;
    logic s2_copy_q, s2_scale_q, s2_final_q;

    // Slot 3 (T+3): the four DSP products, plus the pass-through values.
    logic signed [32:0] s3_prr_q, s3_pii_q, s3_pri_q, s3_pir_q;
    logic signed [16:0] s3_sum_r_q, s3_sum_i_q;
    logic signed [16:0] s3_dif_r_q, s3_dif_i_q;
    logic [AW-1:0] s3_a_q, s3_b_q;
    logic s3_copy_q, s3_scale_q, s3_final_q;

    // Slot 4 (T+4): the results and the (possibly bit-reversed) write
    // addresses. These drive the destination memory's write ports.
    logic [31:0] s4_da_q, s4_db_q;
    logic [AW-1:0] s4_wa_q, s4_wb_q;

    // =================================================================
    // Memories
    // =================================================================
    logic cpu_a_en, cpu_a_we;
    logic [AW-1:0] cpu_a_addr;
    logic [  31:0] cpu_a_wdata;
    logic [   3:0] cpu_a_wstrb;

    logic ebuf_a_en, ebuf_a_we, ebuf_b_en, ebuf_b_we;
    logic [AW-1:0] ebuf_a_addr, ebuf_b_addr;
    logic [31:0] ebuf_a_wdata, ebuf_b_wdata;

    logic scr_a_en, scr_a_we, scr_b_en, scr_b_we;
    logic [AW-1:0] scr_a_addr, scr_b_addr;
    logic [31:0] scr_a_wdata, scr_b_wdata;

    wire [31:0] buf0_a_rdata, buf0_b_rdata, buf1_a_rdata, buf1_b_rdata;
    wire [31:0] scr_a_rdata, scr_b_rdata;

    // Engine-owned buffer = the one the CPU does not own.
    wire eng_buf = ~cpu_buf_q;
    wire buf0_is_eng = (eng_buf == 1'b0);

    fft_dpram #(
        .DEPTH (NMAX),
        .DATA_W(32),
        .ADDR_W(AW)
    ) u_buf0 (
        .clk_i    (clk_i),
        .a_en_i   (buf0_is_eng ? ebuf_a_en : cpu_a_en),
        .a_we_i   (buf0_is_eng ? ebuf_a_we : cpu_a_we),
        .a_addr_i (buf0_is_eng ? ebuf_a_addr : cpu_a_addr),
        .a_wdata_i(buf0_is_eng ? ebuf_a_wdata : cpu_a_wdata),
        .a_wstrb_i(buf0_is_eng ? 4'hF : cpu_a_wstrb),
        .a_rdata_o(buf0_a_rdata),
        .b_en_i   (buf0_is_eng ? ebuf_b_en : 1'b0),
        .b_we_i   (buf0_is_eng ? ebuf_b_we : 1'b0),
        .b_addr_i (ebuf_b_addr),
        .b_wdata_i(ebuf_b_wdata),
        .b_wstrb_i(4'hF),
        .b_rdata_o(buf0_b_rdata)
    );

    fft_dpram #(
        .DEPTH (NMAX),
        .DATA_W(32),
        .ADDR_W(AW)
    ) u_buf1 (
        .clk_i    (clk_i),
        .a_en_i   (buf0_is_eng ? cpu_a_en : ebuf_a_en),
        .a_we_i   (buf0_is_eng ? cpu_a_we : ebuf_a_we),
        .a_addr_i (buf0_is_eng ? cpu_a_addr : ebuf_a_addr),
        .a_wdata_i(buf0_is_eng ? cpu_a_wdata : ebuf_a_wdata),
        .a_wstrb_i(buf0_is_eng ? cpu_a_wstrb : 4'hF),
        .a_rdata_o(buf1_a_rdata),
        .b_en_i   (buf0_is_eng ? 1'b0 : ebuf_b_en),
        .b_we_i   (buf0_is_eng ? 1'b0 : ebuf_b_we),
        .b_addr_i (ebuf_b_addr),
        .b_wdata_i(ebuf_b_wdata),
        .b_wstrb_i(4'hF),
        .b_rdata_o(buf1_b_rdata)
    );

    fft_dpram #(
        .DEPTH (NMAX),
        .DATA_W(32),
        .ADDR_W(AW)
    ) u_scratch (
        .clk_i    (clk_i),
        .a_en_i   (scr_a_en),
        .a_we_i   (scr_a_we),
        .a_addr_i (scr_a_addr),
        .a_wdata_i(scr_a_wdata),
        .a_wstrb_i(4'hF),
        .a_rdata_o(scr_a_rdata),
        .b_en_i   (scr_b_en),
        .b_we_i   (scr_b_we),
        .b_addr_i (scr_b_addr),
        .b_wdata_i(scr_b_wdata),
        .b_wstrb_i(4'hF),
        .b_rdata_o(scr_b_rdata)
    );

    wire [31:0] tw_rdata;

    fft_twiddle_rom #(
        .DEPTH (TW_DEPTH),
        .ADDR_W(TW_AW)
    ) u_twiddle (
        .clk_i (clk_i),
        .en_i  (issue),
        .addr_i(tw_idx_q[TW_AW-1:0]),
        .data_o(tw_rdata)
    );

    // =================================================================
    // Engine port steering.
    //
    // Even stages read the buffer and write the scratch; odd stages the
    // reverse. Reads and writes therefore always land on DIFFERENT
    // memories, which is why the two `if`s below can never fight over the
    // same port (asserted under VERILATOR).
    // =================================================================
    wire src_is_scr = st_q[0];
    wire dst_is_scr = ~st_q[0];
    wire wr_fire = vld_q[PIPE-1];

    always_comb begin
        ebuf_a_en    = 1'b0;
        ebuf_a_we    = 1'b0;
        ebuf_a_addr  = '0;
        ebuf_a_wdata = '0;
        ebuf_b_en    = 1'b0;
        ebuf_b_we    = 1'b0;
        ebuf_b_addr  = '0;
        ebuf_b_wdata = '0;
        scr_a_en     = 1'b0;
        scr_a_we     = 1'b0;
        scr_a_addr   = '0;
        scr_a_wdata  = '0;
        scr_b_en     = 1'b0;
        scr_b_we     = 1'b0;
        scr_b_addr   = '0;
        scr_b_wdata  = '0;

        if (issue) begin
            if (src_is_scr) begin
                scr_a_en   = 1'b1;
                scr_a_addr = iss_a;
                scr_b_en   = 1'b1;
                scr_b_addr = iss_b;
            end else begin
                ebuf_a_en   = 1'b1;
                ebuf_a_addr = iss_a;
                ebuf_b_en   = 1'b1;
                ebuf_b_addr = iss_b;
            end
        end

        if (wr_fire) begin
            if (dst_is_scr) begin
                scr_a_en    = 1'b1;
                scr_a_we    = 1'b1;
                scr_a_addr  = s4_wa_q;
                scr_a_wdata = s4_da_q;
                scr_b_en    = 1'b1;
                scr_b_we    = 1'b1;
                scr_b_addr  = s4_wb_q;
                scr_b_wdata = s4_db_q;
            end else begin
                ebuf_a_en    = 1'b1;
                ebuf_a_we    = 1'b1;
                ebuf_a_addr  = s4_wa_q;
                ebuf_a_wdata = s4_da_q;
                ebuf_b_en    = 1'b1;
                ebuf_b_we    = 1'b1;
                ebuf_b_addr  = s4_wb_q;
                ebuf_b_wdata = s4_db_q;
            end
        end
    end

    // Read data back from whichever memory this stage is sourcing. st_q
    // cannot change while any slot is occupied (a stage advances only
    // from ST_DRAIN with vld_q == 0), so the live select is correct for
    // the data arriving this cycle.
    wire [31:0] ebuf_a_rdata = buf0_is_eng ? buf0_a_rdata : buf1_a_rdata;
    wire [31:0] ebuf_b_rdata = buf0_is_eng ? buf0_b_rdata : buf1_b_rdata;
    wire [31:0] src_a_rdata = src_is_scr ? scr_a_rdata : ebuf_a_rdata;
    wire [31:0] src_b_rdata = src_is_scr ? scr_b_rdata : ebuf_b_rdata;

    wire signed [15:0] rd_a_re = src_a_rdata[15:0];
    wire signed [15:0] rd_a_im = src_a_rdata[31:16];
    wire signed [15:0] rd_b_re = src_b_rdata[15:0];
    wire signed [15:0] rd_b_im = src_b_rdata[31:16];

    // Twiddle: the ROM holds the FORWARD table; the inverse transform
    // negates the imaginary half. -(-32768) has no 16-bit encoding (it is
    // the exact -1.0 that sits at k = N/4), so that one value saturates.
    wire signed [15:0] tw_re = tw_rdata[15:0];
    wire signed [15:0] tw_im_fwd = tw_rdata[31:16];
    wire signed [15:0]
        tw_im = !inv_run_q ? tw_im_fwd : (tw_im_fwd == 16'sh8000 ? 16'sh7FFF : -tw_im_fwd);

    // =================================================================
    // Slot-4 combine (combinational, registered into s4_* below).
    // =================================================================
    wire signed [33:0] mul_r = {s3_prr_q[32], s3_prr_q} - {s3_pii_q[32], s3_pii_q};
    wire signed [33:0] mul_i = {s3_pri_q[32], s3_pri_q} + {s3_pir_q[32], s3_pir_q};

    // Q15 * Q15 -> Q30; round-half-up back to Q15.
    //
    // The sign extension is written out rather than left to the part
    // select: a part select is UNSIGNED in SystemVerilog regardless of
    // what it is sliced out of, so `wire signed [20:0] x = y[33:15]` pads
    // a negative 19-bit result with zeros and turns it into a large
    // positive number, which then saturates to +0x7FFF. That is exactly
    // the failure the fft_tb bit-exact comparison caught first.
    wire signed [33:0] mul_r_rnd = mul_r + 34'sd16384;
    wire signed [33:0] mul_i_rnd = mul_i + 34'sd16384;
    wire [18:0] norm_r_raw = mul_r_rnd[33:15];
    wire [18:0] norm_i_raw = mul_i_rnd[33:15];
    wire signed [20:0] norm_r = {{2{norm_r_raw[18]}}, norm_r_raw};
    wire signed [20:0] norm_i = {{2{norm_i_raw[18]}}, norm_i_raw};

    wire signed [20:0] sum_r_ext = {{4{s3_sum_r_q[16]}}, s3_sum_r_q};
    wire signed [20:0] sum_i_ext = {{4{s3_sum_i_q[16]}}, s3_sum_i_q};
    wire signed [20:0] dif_r_ext = {{4{s3_dif_r_q[16]}}, s3_dif_r_q};
    wire signed [20:0] dif_i_ext = {{4{s3_dif_i_q[16]}}, s3_dif_i_q};

    // Output A is always the sum path. Output B is the twiddled
    // difference, except on the copy pass where the difference register
    // simply carries B through unchanged (see the sum/dif load below).
    wire signed [15:0] out_a_re = sat16(scale_rnd(sum_r_ext, s3_scale_q));
    wire signed [15:0] out_a_im = sat16(scale_rnd(sum_i_ext, s3_scale_q));
    wire signed [15:0] out_b_re = s3_copy_q ? sat16(
        dif_r_ext
    ) : sat16(
        scale_rnd(norm_r, s3_scale_q)
    );
    wire signed [15:0] out_b_im = s3_copy_q ? sat16(
        dif_i_ext
    ) : sat16(
        scale_rnd(norm_i, s3_scale_q)
    );

    // Bit reversal on the final pass only (see the output-order note).
    wire [AW-1:0] rev_a = brev_full(s3_a_q) >> shamt_q;
    wire [AW-1:0] rev_b = brev_full(s3_b_q) >> shamt_q;

    // =================================================================
    // Engine sequencing + datapath registers
    // =================================================================
    wire [3:0]
        start_log2n = (log2n_q < 4'd2) ? 4'd2 : (log2n_q > LOG2NMAX[3:0] ? LOG2NMAX[3:0] : log2n_q);

    // START pulse comes from the AXI write block below.
    logic start_pulse;
    logic swap_pulse;
    logic w1c_irq;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            state_q   <= ST_IDLE;
            st_q      <= '0;
            bf_q      <= '0;
            bf_last_q <= '0;
            half_q    <= '0;
            tw_step_q <= '0;
            tw_idx_q  <= '0;
            vld_q     <= '0;
            shamt_q   <= '0;
            inv_run_q <= 1'b0;
            cyc_q     <= '0;
            cycles_q  <= '0;
            done_q    <= 1'b0;
            cpu_buf_q <= 1'b0;
        end else begin
            // Pipeline occupancy advances every cycle; `issue` is only
            // ever high in ST_RUN.
            vld_q <= {vld_q[PIPE-2:0], issue};

            if (busy) begin
                cyc_q <= cyc_q + 32'd1;
            end

            if (w1c_irq) begin
                done_q <= 1'b0;
            end

            unique case (state_q)
                ST_IDLE: begin
                    if (start_pulse) begin
                        state_q   <= ST_RUN;
                        st_q      <= '0;
                        bf_q      <= '0;
                        tw_idx_q  <= '0;
                        // N/2 - 1 butterflies per stage.
                        bf_last_q <= (BF_W'(1) << (start_log2n - 4'd1)) - BF_W'(1);
                        // half = N/2 (one-hot), twiddle step = NMAX/N.
                        half_q    <= AW'(1) << (start_log2n - 4'd1);
                        tw_step_q <= AW'(1) << (LOG2NMAX[3:0] - start_log2n);
                        shamt_q   <= LOG2NMAX[3:0] - start_log2n;
                        inv_run_q <= inverse_q;
                        cyc_q     <= '0;
                        done_q    <= 1'b0;
                        cpu_buf_q <= ~cpu_buf_q;
                    end else if (swap_pulse) begin
                        cpu_buf_q <= ~cpu_buf_q;
                    end
                end

                ST_RUN: begin
                    bf_q     <= bf_q + BF_W'(1);
                    tw_idx_q <= j_last ? AW'(0) : (tw_idx_q + tw_step_q);
                    if (bf_last) begin
                        state_q <= ST_DRAIN;
                    end
                end

                ST_DRAIN: begin
                    if (vld_q == '0) begin
                        if (is_final_stage) begin
                            state_q  <= ST_IDLE;
                            done_q   <= 1'b1;
                            cycles_q <= cyc_q;
                        end else begin
                            state_q <= ST_RUN;
                            st_q <= st_q + 4'd1;
                            bf_q <= '0;
                            tw_idx_q <= '0;
                            // Hold half at 1 when stepping into the copy
                            // pass: that pass wants sequential pairs
                            // (a, a+1), which is exactly what half = 1
                            // produces.
                            half_q <= ({1'b0, st_q} + 5'd1 == {1'b0, log2n_q}) ?
                                half_q : (half_q >> 1);
                            tw_step_q <= tw_step_q << 1;
                        end
                    end
                end

                default: state_q <= ST_IDLE;
            endcase
        end
    end

    // Datapath pipeline. No reset on the payload registers (they are
    // qualified by vld_q); only the flags that steer memory ports need a
    // defined value out of reset, and those ride vld_q too.
    always_ff @(posedge clk_i) begin
        if (issue) begin
            p1_a_q     <= iss_a;
            p1_b_q     <= iss_b;
            p1_copy_q  <= is_copy;
            p1_scale_q <= scale_eff;
            p1_final_q <= is_final_stage;
        end

        if (vld_q[0]) begin
            // Copy pass: sum carries A, dif carries B, both untouched.
            s2_sum_r_q <= p1_copy_q ?
                {rd_a_re[15], rd_a_re} : ({rd_a_re[15], rd_a_re} + {rd_b_re[15], rd_b_re});
            s2_sum_i_q <= p1_copy_q ?
                {rd_a_im[15], rd_a_im} : ({rd_a_im[15], rd_a_im} + {rd_b_im[15], rd_b_im});
            s2_dif_r_q <= p1_copy_q ?
                {rd_b_re[15], rd_b_re} : ({rd_a_re[15], rd_a_re} - {rd_b_re[15], rd_b_re});
            s2_dif_i_q <= p1_copy_q ?
                {rd_b_im[15], rd_b_im} : ({rd_a_im[15], rd_a_im} - {rd_b_im[15], rd_b_im});
            s2_wr_q <= tw_re;
            s2_wi_q <= tw_im;
            s2_a_q <= p1_a_q;
            s2_b_q <= p1_b_q;
            s2_copy_q <= p1_copy_q;
            s2_scale_q <= p1_scale_q;
            s2_final_q <= p1_final_q;
        end

        if (vld_q[1]) begin
            s3_prr_q   <= s2_dif_r_q * s2_wr_q;
            s3_pii_q   <= s2_dif_i_q * s2_wi_q;
            s3_pri_q   <= s2_dif_r_q * s2_wi_q;
            s3_pir_q   <= s2_dif_i_q * s2_wr_q;
            s3_sum_r_q <= s2_sum_r_q;
            s3_sum_i_q <= s2_sum_i_q;
            s3_dif_r_q <= s2_dif_r_q;
            s3_dif_i_q <= s2_dif_i_q;
            s3_a_q     <= s2_a_q;
            s3_b_q     <= s2_b_q;
            s3_copy_q  <= s2_copy_q;
            s3_scale_q <= s2_scale_q;
            s3_final_q <= s2_final_q;
        end

        if (vld_q[2]) begin
            s4_da_q <= {out_a_im, out_a_re};
            s4_db_q <= {out_b_im, out_b_re};
            s4_wa_q <= s3_final_q ? rev_a : s3_a_q;
            s4_wb_q <= s3_final_q ? rev_b : s3_b_q;
        end
    end

    // =================================================================
    // AXI write path (accept pattern from axi4_lite_gpio / _uart).
    // Address bit 12 selects the DATA page; the CSR page decodes
    // addr[4:2].
    // =================================================================
    logic              aw_seen_q;
    logic              w_seen_q;
    logic [DATA_W-1:0] wdata_q;
    logic [STRB_W-1:0] wstrb_q;
    logic [      10:0] awaddr_q;  // byte addr[12:2]
    logic              bvalid_q;

    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;

    wire              aw_hs = axi.awvalid && axi.awready;
    wire              w_hs = axi.wvalid && axi.wready;

    wire              aw_present = aw_seen_q || aw_hs;
    wire              w_present = w_seen_q || w_hs;

    wire [      10:0] waddr_eff = aw_hs ? axi.awaddr[12:2] : awaddr_q;
    wire [DATA_W-1:0] wdata_eff = w_hs ? axi.wdata : wdata_q;
    wire [STRB_W-1:0] wstrb_eff = w_hs ? axi.wstrb : wstrb_q;

    wire              do_write = aw_present && w_present && !bvalid_q;
    wire              w_is_data = waddr_eff[10];
    wire [       2:0] w_reg = waddr_eff[2:0];
    wire              w_csr = do_write && !w_is_data;

    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;  // OKAY
    wire b_hs = axi.bvalid && axi.bready;

    // Byte-lane mask, same rule as axi4_lite_gpio: a sub-word store
    // MERGES, because the LSU leaves shifted rs2 bits in the unstrobed
    // lanes and gating on "byte 0 strobed" would both drop a legal store
    // to a high field and ghost-write from those lanes.
    logic [DATA_W-1:0] wr_mask;
    always_comb begin
        for (int b = 0; b < STRB_W; b++) begin
            wr_mask[8*b+:8] = {8{wstrb_eff[b]}};
        end
    end

    // Control pulses consumed by the engine block above.
    assign start_pulse = w_csr && (w_reg == REG_CTRL) && wstrb_eff[0] && wdata_eff[0] && !busy;
    assign swap_pulse = w_csr && (w_reg == REG_CTRL) && wstrb_eff[0] && wdata_eff[3] &&
        !wdata_eff[0] && !busy;
    assign w1c_irq = w_csr && (w_reg == REG_IRQ) && wstrb_eff[0] && wdata_eff[0];

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q <= 1'b0;
            w_seen_q  <= 1'b0;
            wdata_q   <= '0;
            wstrb_q   <= '0;
            awaddr_q  <= '0;
            bvalid_q  <= 1'b0;
            irq_en_q  <= 1'b0;
            inverse_q <= 1'b0;
            log2n_q   <= LOG2NMAX[3:0];
            scale_q   <= '1;
        end else begin
            if (aw_hs) begin
                aw_seen_q <= 1'b1;
                awaddr_q  <= axi.awaddr[12:2];
            end
            if (w_hs) begin
                w_seen_q <= 1'b1;
                wdata_q  <= axi.wdata;
                wstrb_q  <= axi.wstrb;
            end

            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;

                if (!w_is_data) begin
                    unique case (w_reg)
                        REG_CTRL: begin
                            if (wstrb_eff[0]) begin
                                irq_en_q  <= wdata_eff[1];
                                inverse_q <= wdata_eff[2];
                            end
                        end
                        REG_CONF: begin
                            // Clamped on write, so a readback always
                            // reports the size actually in force. Ignored
                            // while a transform is running: the engine
                            // derives its stage count from this register.
                            if (wstrb_eff[0] && !busy) begin
                                log2n_q <= (wdata_eff[3:0] < 4'd2) ?
                                    4'd2 : ((wdata_eff[3:0] > LOG2NMAX[3:0]) ? LOG2NMAX[3:0] :
                                            wdata_eff[3:0]);
                            end
                        end
                        REG_SCALE: begin
                            if (!busy) begin
                                scale_q <= (scale_q & ~wr_mask[LOG2NMAX-1:0]) |
                                    (wdata_eff[LOG2NMAX-1:0] & wr_mask[LOG2NMAX-1:0]);
                            end
                        end
                        default: ;  // STATUS / CYCLES read-only, IRQ W1C above
                    endcase
                end
            end

            if (b_hs) begin
                bvalid_q <= 1'b0;
            end
        end
    end

    // The DATA page shares port A of the CPU-owned buffer between a store
    // and a load. The peri bridge is single-outstanding, so the two can
    // never be in flight together; ARREADY drops during a write anyway so
    // the memory never has to resolve it.
    wire ar_hs = axi.arvalid && axi.arready;
    wire r_is_data = axi.araddr[12];
    wire cpu_wr = do_write && w_is_data;
    wire cpu_rd = ar_hs && r_is_data;

    always_comb begin
        cpu_a_en    = cpu_wr || cpu_rd;
        cpu_a_we    = cpu_wr;
        cpu_a_addr  = cpu_wr ? waddr_eff[AW-1:0] : axi.araddr[AW+1:2];
        cpu_a_wdata = wdata_eff;
        cpu_a_wstrb = wstrb_eff;
    end

    // =================================================================
    // AXI read path. CSR reads are registered into rdata_q the usual way;
    // a DATA read comes off the BSRAM's own output register one cycle
    // after the address, which is the same latency -- so the response
    // mux picks the source the pending read was issued to.
    // =================================================================
    logic rvalid_q;
    logic [DATA_W-1:0] rdata_q;
    logic rd_is_data_q;
    logic rd_buf_q;

    assign axi.arready = (!rvalid_q || axi.rready) && !do_write;

    assign axi.rvalid  = rvalid_q;
    assign axi.rresp   = 2'b00;  // OKAY

    wire [31:0] cpu_rd_data = rd_buf_q ? buf1_a_rdata : buf0_a_rdata;
    assign axi.rdata = rd_is_data_q ? cpu_rd_data : rdata_q;

    logic [DATA_W-1:0] rdata_mux;
    always_comb begin
        unique case (axi.araddr[4:2])
            REG_CTRL:   rdata_mux = {28'b0, 1'b0, inverse_q, irq_en_q, 1'b0};
            REG_STATUS: rdata_mux = {28'b0, fft_irq_o, cpu_buf_q, done_q, busy};
            REG_IRQ:    rdata_mux = {31'b0, done_q};
            REG_CONF:   rdata_mux = {28'b0, log2n_q};
            REG_SCALE:  rdata_mux = {{(DATA_W - LOG2NMAX) {1'b0}}, scale_q};
            REG_CYCLES: rdata_mux = cycles_q;
            default:    rdata_mux = '0;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q     <= 1'b0;
            rdata_q      <= '0;
            rd_is_data_q <= 1'b0;
            rd_buf_q     <= 1'b0;
        end else begin
            if (ar_hs) begin
                rvalid_q     <= 1'b1;
                rdata_q      <= rdata_mux;
                rd_is_data_q <= r_is_data;
                // Latch WHICH buffer the read went to: cpu_buf_q flips on
                // a START, and the response mux must still point at the
                // memory whose output register holds this read's data.
                rd_buf_q     <= cpu_buf_q;
            end else if (rvalid_q && axi.rready) begin
                rvalid_q <= 1'b0;
            end
        end
    end

    assign fft_irq_o = done_q & irq_en_q;

    // =================================================================
    // Simulation-only invariants. Each one is a premise the RTL above
    // relies on and that a wave would not make obvious.
    // =================================================================
`ifdef VERILATOR
    always_ff @(posedge clk_i) begin
        if (rstn_i) begin
            // A read and a write in the same cycle must never land on the
            // same memory: reads go to the source, writes to the
            // destination, and those alternate by stage parity.
            if (issue && wr_fire && (src_is_scr == dst_is_scr)) begin
                $error("axi4_lite_fft: read and write on the same memory (stage %0d)", st_q);
            end
            // The twiddle index must stay inside the NMAX/2 table: the
            // largest exponent any stage produces is NMAX/2 - NMAX/N.
            if (issue && tw_idx_q[AW-1]) begin
                $error("axi4_lite_fft: twiddle index %0d out of range", tw_idx_q);
            end
            // The butterfly pair must be two distinct addresses, or the
            // two RAM ports collide.
            if (issue && (iss_a == iss_b)) begin
                $error("axi4_lite_fft: degenerate butterfly pair at %0d", iss_a);
            end
            // The CPU must never touch a buffer the engine owns. cpu_a_*
            // is steered by cpu_buf_q, so this can only fail if the
            // ownership flip were ever allowed to race a transform.
            if (busy && cpu_a_en && (eng_buf == cpu_buf_q)) begin
                $error("axi4_lite_fft: CPU access to the engine's buffer");
            end
            // Single-outstanding premise of the shared port A.
            if (cpu_wr && cpu_rd) begin
                $error("axi4_lite_fft: simultaneous DATA read and write");
            end
        end
    end
`endif

endmodule

`resetall

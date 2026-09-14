`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * True dual-port RAM for the FFT coprocessor's sample buffers.
 *
 * Two fully independent read/write ports, each with a registered read
 * output (1 cycle). Gowin's BSRAM in DPB (true dual-port) mode provides
 * exactly this, and the coding style below is what GowinSynthesis infers
 * it from; Verilator models it directly.
 *
 * ================= WHY THE STORAGE IS SLICED IN 16s =================
 *
 * The array is NOT one DATA_W-wide memory. It is DATA_W/16 separate
 * 16-bit memories, each with its own pair of always_ff blocks. That is
 * not a style preference -- a single 1024x32 true dual-port array does
 * not synthesise on this device:
 *
 *   ERROR (IF0008) : The number(32768) of DFF used to infer "mem"
 *   exceeds the resource limit(15750) of current device
 *   (GW2AR-LV18QN88C8/I7)
 *
 * A GW2A BSRAM in DPB mode tops out at 1K x 18 PER PORT. 1024x32 needs
 * two blocks stitched side by side, and GowinSynthesis does not stitch
 * for a TRUE dual port -- it silently gives up on block RAM and infers
 * registers instead, 32768 of them against a device that has 15750.
 * (It does stitch for the simple/single-port shapes, which is why
 * native_ram.sv's 2048x64 I-mem has always been fine.) Sliced into
 * 16-bit halves, each slice is a 1024x16 array that fits one DPB
 * outright, and the stitching is explicit in RTL instead of being asked
 * of the tool.
 *
 * This is the failure mode the project keeps meeting: an inference that
 * falls back to logic is invisible to every testbench, because
 * simulation elaborates the RTL and not the netlist. It surfaced only as
 * a synthesis error -- and only because the fallback happened to blow
 * the device budget. A design small enough to fit would have shipped as
 * registers and nobody would have known.
 *
 * ===================== WRITE GRANULARITY ============================
 *
 * The write strobes are honoured at HALFWORD granularity, not byte:
 * slice h is written when BOTH of its byte strobes are set. A Gowin DPB
 * has no per-byte write enable, so a byte-granular write would put the
 * fallback back. Consequences for the CPU-facing port:
 *
 *   sw  (strobe 1111) writes both halves  -- the normal case
 *   sh  (strobe 0011 / 1100) writes one half -- real or imaginary alone,
 *       which is exactly the useful sub-word access for a complex sample
 *   sb  (one byte strobed) writes NOTHING, and is answered OKAY
 *
 * The last one is a real sharp edge and is documented in the
 * peripheral's header and in sim/sw/common/fft.h. A sample is two 16-bit
 * fields; there is no meaningful byte inside one.
 *
 * ========================== WRITE MODE ==============================
 *
 * A write leaves this port's read output alone: "no change", Gowin
 * WRITE_MODE 2'b00. The read is the plain `else` of the write, on the
 * SAME condition, and that is load-bearing. The obvious-looking
 *
 *     if (slice_we)      mem[addr] <= wdata;
 *     else if (!we_i)    rdata <= mem[addr];
 *
 * describes the same behaviour and is REJECTED, because the write
 * condition (this slice's strobes) and the read-suppress condition (the
 * port's write enable) are different signals, so the tool cannot express
 * it as no-change and settles on read-before-write instead:
 *
 *   ERROR (PA2122) : Not support 'g_slice[0].mem...'(DPB)
 *   WRITE_MODE0 = 2'b10, please change write mode
 *   WRITE_MODE0 = 2'b00 or 2'b01.
 *
 * With the plain `else`, a write whose strobes miss this slice (a byte
 * store, see above) reads instead, refreshing the output register. That
 * is harmless: the engine always writes full words, and on the CPU port
 * a read response is never in flight during a write -- the bridge is
 * single-outstanding and ARREADY drops while a write is being accepted
 * (axi4_lite_fft asserts both under `ifdef VERILATOR).
 *
 * ======================= OTHER PROPERTIES ===========================
 *
 *   - The read output is held for as long as RVALID waits for RREADY,
 *     which the AXI side depends on: a store landing in that window must
 *     not disturb it. Guaranteed by the single-outstanding premise
 *     above, asserted rather than assumed.
 *
 *   - Nothing here resolves a same-address collision between the two
 *     ports. It does not have to: the engine gives each port a disjoint
 *     half of a radix-2 butterfly pair, never the same address, and a
 *     buffer is never engine-owned and CPU-owned at the same time (see
 *     the ping-pong ownership rule in axi4_lite_fft.sv). Both are
 *     properties of the caller, asserted there under `ifdef VERILATOR
 *     rather than assumed.
 *
 * No reset: an FFT sample buffer has no architected reset value, and a
 * reset port on a BSRAM would cost a mux on the write path. Contents out
 * of reset are undefined -- firmware fills a buffer before it starts a
 * transform on it. Under `ifdef VERILATOR the arrays are zero-initialised
 * so a sim run is deterministic, the same convention reg_file.sv uses.
 *
 * Naming: ports *_i/_o; flops _q.
 */

module fft_dpram #(
    parameter int unsigned DEPTH  = 1024,
    parameter int unsigned DATA_W = 32,
    parameter int unsigned ADDR_W = 10
) (
    input wire clk_i,

    // Port A
    input  wire                   a_en_i,
    input  wire                   a_we_i,
    input  wire  [    ADDR_W-1:0] a_addr_i,
    input  wire  [    DATA_W-1:0] a_wdata_i,
    input  wire  [(DATA_W/8)-1:0] a_wstrb_i,
    output logic [    DATA_W-1:0] a_rdata_o,

    // Port B
    input  wire                   b_en_i,
    input  wire                   b_we_i,
    input  wire  [    ADDR_W-1:0] b_addr_i,
    input  wire  [    DATA_W-1:0] b_wdata_i,
    input  wire  [(DATA_W/8)-1:0] b_wstrb_i,
    output logic [    DATA_W-1:0] b_rdata_o
);

    // One BSRAM-sized slice. 16 and not 18: the extra two bits of a
    // Gowin BSRAM word are parity bits, not addressable data.
    localparam int unsigned SLICE_W = 16;
    localparam int unsigned NSLICE = DATA_W / SLICE_W;

    if (DATA_W % SLICE_W != 0) begin : g_width_check
        $error("fft_dpram: DATA_W=%0d must be a multiple of %0d", DATA_W, SLICE_W);
    end

    genvar s;
    generate
        for (s = 0; s < NSLICE; s = s + 1) begin : g_slice
            // Each slice is its own 1024x16 array -- see the header for
            // why this is not one DATA_W-wide memory.
            (* ram_style = "block" *)
            (* syn_ramstyle = "block_ram" *)
            (* syn_noprune = 1 *)
            logic [SLICE_W-1:0] mem[DEPTH];

            // Halfword write enables: both byte strobes of this slice.
            wire a_slice_we = a_we_i & (&a_wstrb_i[2*s+:2]);
            wire b_slice_we = b_we_i & (&b_wstrb_i[2*s+:2]);

`ifdef VERILATOR
            initial begin
                for (int unsigned i = 0; i < DEPTH; i++) begin
                    mem[i] = '0;
                end
                a_rdata_o[SLICE_W*s+:SLICE_W] = '0;
                b_rdata_o[SLICE_W*s+:SLICE_W] = '0;
            end
`endif

            // The `else` is plain, NOT `else if (!a_we_i)`, and that is
            // load-bearing -- see WRITE MODE in the header.
            always_ff @(posedge clk_i) begin
                if (a_en_i) begin
                    if (a_slice_we) begin
                        mem[a_addr_i] <= a_wdata_i[SLICE_W*s+:SLICE_W];
                    end else begin
                        a_rdata_o[SLICE_W*s+:SLICE_W] <= mem[a_addr_i];
                    end
                end
            end

            always_ff @(posedge clk_i) begin
                if (b_en_i) begin
                    if (b_slice_we) begin
                        mem[b_addr_i] <= b_wdata_i[SLICE_W*s+:SLICE_W];
                    end else begin
                        b_rdata_o[SLICE_W*s+:SLICE_W] <= mem[b_addr_i];
                    end
                end
            end
        end
    endgenerate

endmodule

`resetall

#!/usr/bin/env python3
"""Generate the FFT twiddle ROM module (fft_twiddle_rom.sv).

The ROM holds NMAX/2 entries of W_NMAX^k = exp(-j*2*pi*k/NMAX), packed as
one 32-bit word per entry: {imag[15:0], real[15:0]}, both signed Q15 -- the
same {imag, real} packing the sample buffers use.

Why a generated .sv module and not $readmemh: a $readmemh path has to
resolve identically under Verilator (cwd = sim/) and under gw_sh (cwd =
the project root).  native_ram.sv can take the path as a parameter because
its two instantiations live in the tops; the twiddle table is internal to
the peripheral, so a generated module with an `initial` array is the one
form that needs no path at all and infers a block ROM in both tools.

Regenerate after changing NMAX:

    python3 src/scripts/gen_twiddle.py --nmax 1024 \
        -o src/rtl/utils/fft_twiddle_rom.sv

The output is checked in: synthesis needs it, and nothing in the build
flow runs Python.
"""

import argparse
import math


def q15(x: float) -> int:
    """Round-to-nearest into signed Q15, clamped.

    +1.0 is not representable in Q15, so cos(0) lands on 0x7FFF
    (0.999969...). That one-LSB gain error on W^0 is the standard
    fixed-point FFT compromise and is modelled exactly by the bit-exact
    reference in sim/hw/fft_tb.
    """
    v = int(math.floor(x * 32768.0 + 0.5))
    return max(-32768, min(32767, v))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--nmax", type=int, default=1024)
    ap.add_argument("-o", "--output", default="fft_twiddle_rom.sv")
    args = ap.parse_args()

    nmax = args.nmax
    if nmax < 4 or (nmax & (nmax - 1)) != 0:
        raise SystemExit("nmax must be a power of two >= 4")

    depth = nmax // 2
    addr_w = (depth - 1).bit_length()

    lines = []
    for k in range(depth):
        ang = -2.0 * math.pi * k / nmax
        wr = q15(math.cos(ang))
        wi = q15(math.sin(ang))
        word = ((wi & 0xFFFF) << 16) | (wr & 0xFFFF)
        # Pad the index so the emitted file is already in the project's
        # Verible style (aligned assignments); otherwise `make format`
        # rewrites a generated file and a regeneration undoes it.
        idx = f"rom[{k}]".ljust(len(f"rom[{depth - 1}]"))
        lines.append(f"        {idx} = 32'h{word:08X};  // k={k}")

    body = "\n".join(lines)

    text = f"""`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * FFT twiddle ROM -- GENERATED FILE, DO NOT EDIT BY HAND.
 *
 * Regenerate with:
 *   python3 src/scripts/gen_twiddle.py --nmax {nmax} \\
 *       -o src/rtl/utils/fft_twiddle_rom.sv
 *
 * Contents: W_{nmax}^k = exp(-j*2*pi*k/{nmax}) for k = 0 .. {depth - 1},
 * one 32-bit word per entry packed {{imag[15:0], real[15:0]}}, both signed
 * Q15 -- the same packing the sample buffers use.
 *
 * Only the FORWARD table is stored. The inverse transform negates the
 * imaginary half in the datapath (axi4_lite_fft.sv), which is one 16-bit
 * negate against a second {depth}-entry table.
 *
 * cos(0) stores as 0x7FFF, not 1.0: +1.0 has no Q15 encoding. The
 * resulting one-LSB gain on W^0 is inherent to a Q15 twiddle table and is
 * reproduced exactly by the bit-exact reference model in sim/hw/fft_tb.
 *
 * Read port is registered (1 cycle), matching fft_dpram so the twiddle and
 * the sample pair arrive in the same pipeline stage.
 *
 * Naming: ports *_i/_o; flops _q.
 */

module fft_twiddle_rom #(
    parameter int unsigned DEPTH  = {depth},
    parameter int unsigned ADDR_W = {addr_w}
) (
    input  wire               clk_i,
    input  wire               en_i,
    input  wire  [ADDR_W-1:0] addr_i,
    output logic [      31:0] data_o
);

    // Attributes mirror native_ram.sv: ram_style is the Vivado spelling
    // (a no-op for GowinSynthesis), syn_romstyle is the one Gowin reads,
    // and syn_noprune stops the tool folding a constant array away.
    (* ram_style = "block" *)
    (* syn_romstyle = "block_rom" *)
    (* syn_noprune = 1 *)
    logic [31:0] rom[DEPTH];

    initial begin
{body}
    end

    always_ff @(posedge clk_i) begin
        if (en_i) begin
            data_o <= rom[addr_i];
        end
    end

endmodule

`resetall
"""
    with open(args.output, "w") as fh:
        fh.write(text)
    print(f"wrote {args.output}: {depth} entries, ADDR_W={addr_w}")


if __name__ == "__main__":
    main()

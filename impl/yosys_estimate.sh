#!/usr/bin/env bash
#
# Open-source resource estimate for this design: sv2v -> yosys synth_gowin.
#
# WHY THIS EXISTS. The authoritative numbers come from GowinSynthesis on
# the remote toolchain host (see "Build flow" in CLAUDE.md). When that host
# is not reachable -- or before committing a PnR run to it -- this gives a
# same-machine estimate of what a block costs, and in particular answers
# the one question simulation cannot: WHICH PRIMITIVES a memory or a
# multiplier actually infers. Simulation elaborates the RTL, not the
# netlist, so a memory that quietly falls back to flip-flops is invisible
# in every testbench and fatal on the device.
#
# WHAT TO TRUST, AND WHAT NOT TO. yosys is not GowinSynthesis:
#
#   * TRUST the architectural counts -- BSRAM blocks, DSP multipliers,
#     flip-flops. Those follow from the design, and in the runs this
#     script was written for they agreed with the hand estimate exactly
#     and agreed between a standalone block build and a whole-design
#     build.
#   * DO NOT TRUST the LUT count as a number. Technology mapping differs,
#     and a block synthesised alone measured roughly HALF what the same
#     block cost as a delta inside the flattened design. Read the LUT
#     column as a range, and never as a utilisation percentage.
#   * NOTHING HERE SAYS ANYTHING ABOUT TIMING. yosys produces no slack.
#     A block that fits may still move the critical path, and on this
#     design routing -- not logic depth -- is what binds. Only PnR on the
#     real toolchain answers that.
#
# THE REGISTER FILE IS PATCHED OUT OF BSRAM, deliberately. reg_file.sv
# carries (* ram_style = "block" *) on a memory with TWO ASYNCHRONOUS read
# ports. GowinSynthesis maps that (the A/B in reg_file.sv's header says
# BSRAM wins); yosys's gw2a library has no async-read RAM primitive, so it
# stops with
#
#   ERROR: no valid mapping found for memory top_module.u_cpu.u_regfile.regs
#
# The attribute is stripped in the GENERATED Verilog only -- never in the
# repository -- so the memory falls back to logic. That inflates the
# absolute FF and LUT totals versus a real build, identically in every
# configuration, so DELTAS stay meaningful and TOTALS do not.
#
# Usage:
#   impl/yosys_estimate.sh                 full design + FFT block + delta vs HEAD
#   impl/yosys_estimate.sh --ref main      compare against another git ref
#
# NOTE the default baseline is HEAD, i.e. the last COMMIT -- which on a
# feature branch is your own previous commit, not the integration branch.
# Pass --ref main when you want the cost of the whole branch. The table
# labels the baseline with its resolved sha so there is no ambiguity.
#   impl/yosys_estimate.sh --no-baseline   skip the git-archive baseline run
#   impl/yosys_estimate.sh --keep          keep the work directory
#
# Requires sv2v and yosys on PATH. The source list is read from the .gprj,
# which is the same list GowinSynthesis uses, so it cannot drift.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GPRJ="$REPO_ROOT/rv32imac_Zicsr_Zifencei.gprj"
FFT_TB="$REPO_ROOT/sim/hw/fft_tb/fft_tb.sv"

BASELINE_REF="HEAD"
DO_BASELINE=1
KEEP=0

while [ $# -gt 0 ]; do
    case "$1" in
        --ref)         BASELINE_REF="$2"; shift 2 ;;
        --no-baseline) DO_BASELINE=0; shift ;;
        --keep)        KEEP=1; shift ;;
        -h|--help)     sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)             echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

for t in sv2v yosys; do
    command -v "$t" >/dev/null 2>&1 || { echo "$t not on PATH" >&2; exit 1; }
done

WORK="$(mktemp -d)"
if [ "$KEEP" = 1 ]; then
    echo "work dir: $WORK"
else
    trap 'rm -rf "$WORK"' EXIT
fi

# Source list, straight out of the Gowin project file. Packages and the
# interface go first so sv2v sees the definitions before the users; the
# rest keeps the project's order.
gprj_sources() {
    local gprj="$1" root="$2"
    grep -oE 'path="[^"]*\.sv"' "$gprj" | sed 's/path="//;s/"$//' \
        | awk -v r="$root" '
            /_pkg\.sv$/      { pkg = pkg " " r "/" $0; next }
            /axi4_lite_if\.sv$/ { itf = itf " " r "/" $0; next }
                             { rest = rest " " r "/" $0 }
            END { print pkg itf rest }'
}

# sv2v, then remove the register file's block-RAM attribute (see header).
convert() {
    local out="$1"; shift
    sv2v -w "$out" "$@"
    sed -i 's/(\* ram_style = "block" \*) reg \[31:0\] regs/reg [31:0] regs/' "$out"
}

synth() {
    local src="$1" top="$2" out="$3"
    yosys -p "read_verilog -sv $src; synth_gowin -family gw2a -noiopads -top $top; stat -top $top" \
        > "$out" 2>&1 || { echo "yosys failed for top=$top, see $out" >&2; tail -20 "$out" >&2; exit 1; }
}

# Collapse a `stat` report into one line of totals.
#
# Only the FIRST statistics section is counted, and the guard matters:
# synth_gowin ends its own script with a `stat`, so a run that also asks
# for `stat -top` explicitly prints the report TWICE and a naive sum
# reports exactly double -- which read as 46 of 46 BSRAM blocks on a
# design that uses 23. Within one section, the lines before
# "design hierarchy" are the top's local cells plus its submodules'
# cells, which together are the whole (flattened) design.
summarise() {
    awk '
        /Printing statistics/ { if (!seen) { on = 1; seen = 1 } ; next }
        /design hierarchy/    { on = 0 }
        !on { next }
        /LUT[1-4]$/     { lut  += $1 }
        /MUX2_LUT[5-8]$/{ mux  += $1 }
        / ALU$/         { alu  += $1 }
        /DFF[A-Z]*$/    { ff   += $1 }
        /RAM16SDP4$/    { lram += $1 }
        / (SP|SPX9|DPB|DPX9B|SDPB|SDPX9B|pROM|pROMX9)$/ { bram += $1 }
        /MULT18X18$/    { d18  += $1 }
        /MULT36X36$/    { d36  += $1 }
        END { printf "%d %d %d %d %d %d %d %d\n",
              lut+0, mux+0, alu+0, ff+0, bram+0, d18+0, d36+0, lram+0 }
    ' "$1"
}

row() { printf '%-26s %8s %8s %8s %8s %7s %7s %7s %8s\n' "$@"; }

echo "=== yosys resource estimate (sv2v -> synth_gowin -family gw2a) ==="
echo

# ---- current tree, whole design ------------------------------------
echo "[1/3] current tree, top_module ..."
# shellcheck disable=SC2046
convert "$WORK/cur.v" $(gprj_sources "$GPRJ" "$REPO_ROOT")
synth "$WORK/cur.v" top_module "$WORK/cur.stat"
read -r C_LUT C_MUX C_ALU C_FF C_BRAM C_D18 C_D36 C_LRAM <<<"$(summarise "$WORK/cur.stat")"

# ---- FFT block alone ------------------------------------------------
FFT_OK=0
if [ -f "$FFT_TB" ]; then
    echo "[2/3] FFT coprocessor alone, top fft_tb ..."
    convert "$WORK/fft.v" \
        "$REPO_ROOT/src/rtl/pkg/rv32_pkg.sv" \
        "$REPO_ROOT/src/rtl/bus/axi4_lite_if.sv" \
        "$REPO_ROOT/src/rtl/utils/fft_dpram.sv" \
        "$REPO_ROOT/src/rtl/utils/fft_twiddle_rom.sv" \
        "$REPO_ROOT/src/rtl/utils/axi4_lite_fft.sv" \
        "$FFT_TB"
    synth "$WORK/fft.v" fft_tb "$WORK/fft.stat"
    read -r F_LUT F_MUX F_ALU F_FF F_BRAM F_D18 F_D36 F_LRAM <<<"$(summarise "$WORK/fft.stat")"
    FFT_OK=1
else
    echo "[2/3] skipped (no $FFT_TB)"
fi

# ---- baseline from a git ref ----------------------------------------
BASE_OK=0
if [ "$DO_BASELINE" = 1 ]; then
    echo "[3/3] baseline $BASELINE_REF, top_module ..."
    mkdir -p "$WORK/base"
    BASELINE_SHA="$( cd "$REPO_ROOT" && git rev-parse --short "$BASELINE_REF" )"
    ( cd "$REPO_ROOT" && git archive "$BASELINE_REF" ) | tar -x -C "$WORK/base"
    # shellcheck disable=SC2046
    convert "$WORK/base.v" $(gprj_sources "$WORK/base/rv32imac_Zicsr_Zifencei.gprj" "$WORK/base")
    synth "$WORK/base.v" top_module "$WORK/base.stat"
    read -r B_LUT B_MUX B_ALU B_FF B_BRAM B_D18 B_D36 B_LRAM <<<"$(summarise "$WORK/base.stat")"
    BASE_OK=1
else
    echo "[3/3] skipped (--no-baseline)"
fi

echo
row "configuration" "LUT1-4" "MUX2" "ALU" "FF" "BSRAM" "M18x18" "M36x36" "LUTRAM"
row "--------------------------" "--------" "--------" "--------" "--------" "-------" "-------" "-------" "--------"
[ "$BASE_OK" = 1 ] && row "baseline ($BASELINE_REF=$BASELINE_SHA)" "$B_LUT" "$B_MUX" "$B_ALU" "$B_FF" "$B_BRAM" "$B_D18" "$B_D36" "$B_LRAM"
row "current tree" "$C_LUT" "$C_MUX" "$C_ALU" "$C_FF" "$C_BRAM" "$C_D18" "$C_D36" "$C_LRAM"
if [ "$BASE_OK" = 1 ]; then
    row "delta" \
        "$((C_LUT - B_LUT))" "$((C_MUX - B_MUX))" "$((C_ALU - B_ALU))" "$((C_FF - B_FF))" \
        "$((C_BRAM - B_BRAM))" "$((C_D18 - B_D18))" "$((C_D36 - B_D36))" "$((C_LRAM - B_LRAM))"
fi
[ "$FFT_OK" = 1 ] && row "FFT block alone" "$F_LUT" "$F_MUX" "$F_ALU" "$F_FF" "$F_BRAM" "$F_D18" "$F_D36" "$F_LRAM"

echo
echo "Logic cells (LUT1-4 + MUX2 + ALU):"
[ "$BASE_OK" = 1 ] && echo "  baseline      $((B_LUT + B_MUX + B_ALU))"
echo "  current tree  $((C_LUT + C_MUX + C_ALU))"
[ "$BASE_OK" = 1 ] && echo "  delta         $(( (C_LUT + C_MUX + C_ALU) - (B_LUT + B_MUX + B_ALU) ))"
[ "$FFT_OK" = 1 ]  && echo "  FFT alone     $((F_LUT + F_MUX + F_ALU))"
echo
echo "BSRAM: $C_BRAM of 46 blocks on the GW2AR-18C."
echo
echo "Reminder: BSRAM / DSP / FF are the trustworthy columns. The LUT count"
echo "is an order-of-magnitude figure -- a block synthesised alone measured"
echo "about half its delta inside the flattened design. And no part of this"
echo "says anything about timing; only PnR on the Gowin toolchain does."

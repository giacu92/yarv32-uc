#!/usr/bin/env bash
#
# Open-source resource estimate of the WHOLE Linux SoC top (top_soc:
# core + cache subproject + SDRAM controller + Linux peripherals):
# sv2v -> yosys synth_gowin -family gw2a. PROVISIONAL numbers -- yosys is not
# GowinSynthesis. Same approach, and the same traps, as impl/yosys_estimate.sh
# (read its header); the differences are listed here.
#
#   impl/yosys_estimate_soc.sh              default mapping AND -nowidelut
#   impl/yosys_estimate_soc.sh --keep       keep the work directory (logs)
#
# WHAT TO TRUST. BSRAM blocks, DSP multipliers and flip-flops follow from the
# design; LUT counts are a range, not a utilisation. The cache repo measured
# synth_gowin's default wide-LUT packing (MUX2_LUT5-8) as UNSTABLE on the
# cache (+1.3k / +2.7k cells from equal-function rewrites), so this script
# maps twice and the -nowidelut row is the one to judge changes by: every
# function in LUT1-4 (+ALU), i.e. an upper bound in LUT4 equivalents.
# Nothing here says anything about timing.
#
# TRAPS (inherited, each of which produced a wrong answer once):
#   * reg_file.sv's (* ram_style = "block" *) on a 2-async-read memory has no
#     gw2a mapping in yosys ("no valid mapping found for memory ...regs").
#     The attribute is stripped in the GENERATED Verilog only; the register
#     file then lands in flops/LUT-RAM, which inflates FF/LUT totals versus
#     GowinSynthesis (which puts it in BSRAM).
#   * synth_gowin ends with its own `stat`; only the LAST statistics section
#     (the final flattened netlist) is summed, never all of them -- summing
#     every section reports exactly double.
#
# SoC-SPECIFIC:
#   * The source list is linux_soc.gprj's (packages first), so it cannot
#     drift from what GowinSynthesis compiles. impl/soc_rtl/ must exist
#     (impl/soc_rtl.sh). sdram_controller.v is plain Verilog and is read by
#     yosys directly, not through sv2v (as the cache repo's scripts do).
#   * yosys runs FROM THE REPO ROOT, so top_soc's BOOTROM_FILE
#     (sim/sw/bootrom/build/bootrom.hex) resolves and the boot ROM is an
#     initialised ROM. An uninitialised read-only array is a constant, and
#     a constant ROM may be pruned -- which would under-count it.
#   * `GOWIN is defined by rv32_pkg, so the rPLL instance is kept (a gw2a
#     cell yosys knows).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GPRJ="$REPO_ROOT/linux_soc.gprj"
TOP=top_soc
KEEP=0
case "${1:-}" in
    --keep) KEEP=1 ;;
    "") ;;
    -h | --help) sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
esac

for t in sv2v yosys; do
    command -v "$t" >/dev/null 2>&1 || { echo "$t not on PATH" >&2; exit 1; }
done
[ -f "$REPO_ROOT/impl/soc_rtl/cache_cntrl.sv" ] || { echo "run impl/soc_rtl.sh first" >&2; exit 1; }
[ -f "$REPO_ROOT/sim/sw/bootrom/build/bootrom.hex" ] ||
    { echo "build the boot ROM first: make -C sim/sw/bootrom RISCV_PREFIX=..." >&2; exit 1; }

WORK="$(mktemp -d)"
if [ "$KEEP" = 1 ]; then echo "work dir: $WORK"; else trap 'rm -rf "$WORK"' EXIT; fi

cd "$REPO_ROOT"
mapfile -t ALL < <(grep -oE 'path="[^"]*\.s?v"' "$GPRJ" | sed 's/path="//;s/"$//')
SV=() V=() PKG=()
for f in "${ALL[@]}"; do
    case "$f" in
        *_pkg.sv | */axi4_lite_if.sv) PKG+=("$f") ;;
        *.sv) SV+=("$f") ;;
        *.v) V+=("$f") ;;
    esac
done

sv2v -w "$WORK/soc.v" "${PKG[@]}" "${SV[@]}"
# Strip the regfile's block-RAM attribute (see header), whether sv2v put it
# on the declaration's line or on the line before.
perl -0pi -e 's/\(\*\s*ram_style\s*=\s*"block"\s*\*\)(\s*reg\s*\[31:0\]\s*regs\b)/$1/g' "$WORK/soc.v"
grep -qE 'ram_style = "block" \*\)\s*reg \[31:0\] regs' "$WORK/soc.v" &&
    { echo "regfile attribute strip failed" >&2; exit 1; }

synth() {
    local opts="$1" log="$2"
    nice -n 10 yosys -p "read_verilog -sv $WORK/soc.v ${V[*]}; \
        synth_gowin -family gw2a -noiopads $opts -top $TOP" >"$log" 2>&1 ||
        { echo "yosys failed ($opts), see $log" >&2; tail -20 "$log" >&2; exit 1; }
}

# Totals of the LAST statistics section (synth_gowin's own final stat).
summarise() {
    local last
    last=$(grep -n "Printing statistics" "$1" | tail -1 | cut -d: -f1)
    tail -n +"$last" "$1" | awk '
        /design hierarchy/ { exit }
        $2 ~ /^LUT[1-4]$/          { lut  += $1 }
        $2 ~ /^MUX2_LUT[5-8]$/     { mux  += $1 }
        $2 == "ALU"                { alu  += $1 }
        $2 ~ /^DFF/                { ff   += $1 }
        $2 ~ /^(SP|SPX9|SDP|SDPB|SDPX9B|DP|DPB|DPX9B|pROM|pROMX9)$/ { bram += $1 }
        $2 ~ /^MULT/               { dsp  += $1 }
        $2 ~ /^RAM16/              { lram += $1 }
        END { printf "%d %d %d %d %d %d %d\n", lut+0, mux+0, alu+0, ff+0, bram+0, dsp+0, lram+0 }'
}

echo "=== top_soc yosys estimate (sv2v -> synth_gowin -family gw2a), PROVISIONAL ==="
row() { printf '%-12s %7s %7s %6s %8s %7s %6s %4s %8s\n' "$@"; }
row mapping LUT1-4 MUX2 ALU cells FF BSRAM DSP LUTRAM
for opts in "" "-nowidelut"; do
    log="$WORK/soc${opts}.log"
    synth "$opts" "$log"
    read -r L M A F B D R <<<"$(summarise "$log")"
    row "${opts:-default}" "$L" "$M" "$A" "$((L + M + A))" "$F" "$B" "$D" "$R"
    cp "$log" "$WORK/last.log"
    # Primitive breakdown of the final netlist, for the record.
    last=$(grep -n "Printing statistics" "$log" | tail -1 | cut -d: -f1)
    tail -n +"$last" "$log" | awk '/design hierarchy/{exit} $1 ~ /^[0-9]+$/ && NF==2 {print "    " $0}' \
        >"$WORK/prims${opts}.txt"
done
echo
# Device capacity as the repo documents it (docs/linux_assessment.md 4.1):
# 46 BSRAM, 48 MULT18X18, FF limit 15750 (GowinSynthesis's own IF0008
# message), ~20736 LUT4 (datasheet figure, unverified). 15552 is NOT an FF
# count: it is the 75 % LUT budget of docs/linux_progress.md (20736 x 0.75).
echo "Device (GW2AR-18C): ~20736 LUT4 (75 % budget 15552), FF limit 15750, 46 BSRAM, 48 MULT18X18."
echo "Final primitives (-nowidelut run):"
cat "$WORK/prims-nowidelut.txt"
echo
echo "Memory inference (default run). Every memory yosys saw, with its final primitive:"
grep -hE "mapping memory .* via |using FF mapping for memory " "$WORK/soc.log" |
    sed -E 's/^mapping memory top_soc\.(.*) via \$__GOWIN_(SP|SDP|DP|LUTRAM)_.*/  \1 -> \2/;
            s/^using FF mapping for memory top_soc\.(.*)/  \1 -> FLIP-FLOPS (memory_map)/' |
    sed -E 's/-> SP$/-> BSRAM (SP)/; s/-> LUTRAM$/-> LUT-RAM (RAM16SDP4)/' | sort -u
echo "Sizes of the FF-mapped ones (entries x width, from memory_map):"
awk '/^Mapping memory/ { n=$3 } /created [0-9]+ \$dff cells/ { print "  " n ": " $2 " x " $NF+0 " bits" }' \
    "$WORK/soc.log" | sed 's/\\//g'

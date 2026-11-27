#!/usr/bin/env bash
#
# Generate impl/soc_rtl/: the cache subproject's synthesis sources, taken
# from the commit the superproject PINS for src/sub/yarv32-native-caches and
# made to coexist with rv32_pkg. linux_soc.gprj compiles these files, never
# the submodule's working tree.
#
#   impl/soc_rtl.sh [git-ref] [out-dir]
#     git-ref   superproject ref whose submodule pin to use (default HEAD;
#               "INDEX" = the pin currently staged, which is what sim/soc's
#               Makefile uses)
#     out-dir   default impl/soc_rtl
#
# WHY A GENERATED COPY. Both projects wildcard-import their package at FILE
# scope (`import rv32_pkg::*;` / `import yarv32_cache_pkg::*;`) and both
# packages declare `mem_req_t` / `mem_rsp_t`. In one compilation unit -- which
# is how Verilator, sv2v and (by the safe assumption) GowinSynthesis read a
# file list -- the cache's names would resolve to rv32_pkg's. The SoC sim
# solves this with sim/soc/qualify_pkg.py on a `git archive` snapshot; this
# script applies the SAME script to the SAME files, so the board build and
# the sim compile byte-identical cache RTL. The submodule is only read.
#
# Also: `git archive` of the superproject (impl/remote_pnr_soc.sh) carries no
# submodule content, so this directory is what gets shipped to the Gowin host.
#
# Files produced (the gprj names them; keep the two lists in step):
#   yarv32_cache_pkg.sv  native_ram_cache.sv  cache_cntrl.sv
#   sdram_controller.v   REV (pins used, for the run summary)
#
# native_ram_cache.sv: the cache's native_ram, renamed from the file name
# only -- the MODULE is still `native_ram`. The SoC project does not compile
# the MCU's src/rtl/utils/native_ram.sv, so there is exactly one native_ram.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REF=${1:-HEAD}
OUT=${2:-$ROOT/impl/soc_rtl}
CACHE_DIR="$ROOT/src/sub/yarv32-native-caches"

if [ "$REF" = INDEX ]; then
    CACHE_REV=$(git -C "$ROOT" ls-files -s src/sub/yarv32-native-caches | awk '{print $2}')
else
    CACHE_REV=$(git -C "$ROOT" ls-tree "$REF" src/sub/yarv32-native-caches | awk '{print $3}')
fi
[ -n "$CACHE_REV" ] || { echo "no submodule pin found at $REF" >&2; exit 1; }
SDRC_REV=$(git -C "$CACHE_DIR" ls-tree "$CACHE_REV" src/ips/sdram-controller | awk '{print $3}')
[ -n "$SDRC_REV" ] || { echo "no sdram-controller pin in cache $CACHE_REV" >&2; exit 1; }

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/src/ips/sdram-controller"
git -C "$CACHE_DIR" archive "$CACHE_REV" | tar -x -C "$TMP"
git -C "$CACHE_DIR/src/ips/sdram-controller" archive "$SDRC_REV" | tar -x -C "$TMP/src/ips/sdram-controller"
python3 "$ROOT/sim/soc/qualify_pkg.py" "$TMP/src/rtl/pkg/yarv32_cache_pkg.sv" \
    "$TMP/src/rtl/native_ram.sv" "$TMP/src/rtl/cache_cntrl.sv" >/dev/null

rm -rf "$OUT"
mkdir -p "$OUT"
cp "$TMP/src/rtl/pkg/yarv32_cache_pkg.sv" "$OUT/yarv32_cache_pkg.sv"
cp "$TMP/src/rtl/native_ram.sv" "$OUT/native_ram_cache.sv"
cp "$TMP/src/rtl/cache_cntrl.sv" "$OUT/cache_cntrl.sv"
cp "$TMP/src/ips/sdram-controller/rtl/sdram_controller.v" "$OUT/sdram_controller.v"
printf 'superproject %s\ncache %s\nsdram-controller %s\n' \
    "$(git -C "$ROOT" rev-parse --short "${REF/INDEX/HEAD}")" "$CACHE_REV" "$SDRC_REV" >"$OUT/REV"
echo "impl/soc_rtl: cache $CACHE_REV, sdram-controller $SDRC_REV -> $OUT"

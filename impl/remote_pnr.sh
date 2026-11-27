#!/usr/bin/env bash
#
# Synthesis + place & route of ONE git ref on the Gowin host, in a fresh
# directory of its own, with the reports copied back here.
#
# WHY. The tree used to be rsync'ed over ~/gowin_proj/rv32imac_Zicsr_Zifencei,
# so every run overwrote the last one's reports -- including the frozen run 8
# (2026-09-14) that every later comparison is made against. This script never
# writes outside ~/gowin_proj/<tag>/ and refuses to reuse a tag.
#
# Usage:
#   impl/remote_pnr.sh <git-ref> <tag> [top]
#     git-ref  what to build, e.g. main-fixes, 8943377, HEAD
#     tag      run directory name under ~/gowin_proj/ (must not exist yet)
#     top      top module (default top_module)
#
# Environment:
#   GOWIN_HOST   ssh target            (default giacomo@192.168.10.36)
#   GW_SH        gw_sh path on the host (default ~/gowin_ide/IDE/bin/gw_sh)
#   PNR_EXTRA    extra Tcl lines inserted before `run pnr`, e.g. a placement
#                seed option. Gowin PnR on this project has been observed to
#                be DETERMINISTIC (identical tree + options = identical slack),
#                so "N seeds" only means something if the tool exposes a seed
#                option; run `help set_option` in gw_sh to find its name.
#
# What is shipped: `git archive <ref>` (tracked files only), PLUS the firmware
# hex files the board top's INIT_FILE parameters name, copied from THIS
# working tree -- they are build products and not in git, and an I-mem with
# no init file is a constant that synthesis may delete, which would make the
# timing report describe a different design. Their md5s go in the summary.
#
# Reports land in impl/runs/<tag>/ (gitignored): the synthesis report, the
# PnR timing reports (.tr.html, plain-text .tr.txt / .timing_paths if
# enabled), the resource report, and a grep of the .vg for the PHT
# primitives (the open PHT-in-BSRAM question, CLAUDE.md "Open work").
set -euo pipefail

REF=${1:?usage: impl/remote_pnr.sh <git-ref> <tag> [top]}
TAG=${2:?usage: impl/remote_pnr.sh <git-ref> <tag> [top]}
TOP=${3:-top_module}
HOST=${GOWIN_HOST:-giacomo@192.168.10.36}
GW_SH=${GW_SH:-'~/gowin_ide/IDE/bin/gw_sh'}
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJ=rv32imac_Zicsr_Zifencei
RUN="gowin_proj/$TAG"
OUT="$REPO_ROOT/impl/runs/$TAG"

case "$TAG" in
    */* | .* | "" | rv32imac_Zicsr_Zifencei)
        echo "refusing tag '$TAG' (no slashes, no dot-names, never the frozen run dir)" >&2
        exit 2
        ;;
esac

cd "$REPO_ROOT"
SHA=$(git rev-parse --verify "$REF^{commit}")

ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST" "test ! -e ~/$RUN" || {
    echo "~/$RUN already exists on $HOST -- pick a new tag" >&2
    exit 2
}

# Firmware images named by INIT_FILE in the top, as they exist at REF.
mapfile -t HEX < <(git show "$SHA:src/rtl/core/top_module.sv" |
    sed -n 's/.*INIT_FILE *("\([^"]*\)").*/\1/p')
for h in "${HEX[@]}"; do
    [ -f "$h" ] || {
        echo "missing local build product $h (run make sw-all first)" >&2
        exit 2
    }
done

echo "== shipping $SHA to $HOST:~/$RUN"
ssh "$HOST" "mkdir -p ~/$RUN"
git archive "$SHA" | ssh "$HOST" "tar -x -C ~/$RUN"
tar -c "${HEX[@]}" | ssh "$HOST" "tar -x -C ~/$RUN"

TCL=$(
    cat <<EOF
open_project /home/\$env(USER)/$RUN/$PROJ.gprj
set_option -top_module $TOP
set_option -global_freq 50.000
set_option -place_option 2
set_option -route_option 1
set_option -gen_text_timing_rpt 1
run syn
${PNR_EXTRA:-}
run pnr
EOF
)
ssh "$HOST" "cat > ~/$RUN/run.tcl" <<<"$TCL"

echo "== running gw_sh (synthesis + PnR); log in ~/$RUN/run.log"
ssh "$HOST" "cd ~/$RUN && QT_QPA_PLATFORM=offscreen QT_OPENGL=software \
    LIBGL_ALWAYS_SOFTWARE=1 $GW_SH run.tcl > run.log 2>&1" || {
    echo "gw_sh failed -- see ~/$RUN/run.log on $HOST" >&2
}

mkdir -p "$OUT"
scp -q "$HOST:~/$RUN/run.log" "$OUT/" || true
scp -q "$HOST:~/$RUN/impl/gwsynthesis/*_syn.rpt.html" "$OUT/" 2>/dev/null || true
scp -q "$HOST:~/$RUN/impl/pnr/*.tr.html" "$HOST:~/$RUN/impl/pnr/*.rpt.html" "$OUT/" 2>/dev/null || true
scp -q "$HOST:~/$RUN/impl/pnr/*.tr.txt" "$HOST:~/$RUN/impl/pnr/*.timing_paths" "$OUT/" 2>/dev/null || true
ssh "$HOST" "grep -n 'pht_q' ~/$RUN/impl/gwsynthesis/*.vg | grep -E 'SDPB|DPB|SP |SPX|DPX|RAM16|ROM' | head -50" \
    >"$OUT/pht_primitives.txt" 2>/dev/null || true

{
    echo "ref $REF = $SHA"
    echo "tag $TAG  top $TOP  host $HOST"
    echo "firmware (md5, from the working tree at $(git rev-parse --short HEAD)):"
    md5sum "${HEX[@]}" | sed 's/^/  /'
    echo "PNR_EXTRA: ${PNR_EXTRA:-<none>}"
    grep -h -i -m3 -E 'Max Frequency|Actual Fmax|clk_core +[0-9]' "$OUT"/*.tr.html 2>/dev/null || true
} >"$OUT/SUMMARY.txt"
cat "$OUT/SUMMARY.txt"
echo "== reports in $OUT"

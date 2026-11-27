#!/usr/bin/env bash
#
# Synthesis + place & route of the LINUX SoC (linux_soc.gprj, top top_soc)
# on the Gowin host, one run per call, in a fresh directory of its own, with
# the reports copied back. Modelled on impl/remote_pnr.sh (same host, same
# never-overwrite-a-run rule); see that file for the history.
#
# Usage:
#   impl/remote_pnr_soc.sh <git-ref|WORKTREE> <tag> [seed]
#     git-ref   what to build (e.g. linux-soc, HEAD, a sha). WORKTREE ships
#               the files linux_soc.gprj names from THIS working tree,
#               committed or not -- for a first run before anything is
#               committed. The summary records which it was.
#     tag       run directory name under ~/gowin_proj/ (must not exist yet);
#               a seed run gets "-s<seed>" appended.
#     seed      optional placement seed. The plan asks for >= 3 seeds:
#                 for s in 1 2 3; do impl/remote_pnr_soc.sh linux-soc soc-r1 $s; done
#
# SEEDS -- READ THIS BEFORE COMPARING RUNS. Gowin PnR on this project has
# been measured DETERMINISTIC (identical tree + options = identical slack to
# the picosecond; docs/linux_progress.md D4). A seed therefore only means
# something if gw_sh exposes a seed option. Its name is not documented in
# this repo, so the script asks the tool: it runs `help set_option` on the
# host and looks for an option containing "seed". Found -> it is set to the
# given value. Not found -> the run STOPS (exit 3) instead of silently
# producing N identical runs labelled as N seeds; set SEED_OPTION to the
# right name by hand if the grep misses it, or follow D4 (one run plus a
# confirmation re-run of the same tree).
#
# What is shipped:
#   * the RTL/constraints linux_soc.gprj names, from <git-ref> (git archive)
#     or from the working tree (WORKTREE);
#   * impl/soc_rtl/, REGENERATED here by impl/soc_rtl.sh from the cache pin
#     of that same ref (the working tree's staged pin for WORKTREE), so the
#     cache RTL always matches the superproject being built -- `git archive`
#     carries no submodule content;
#   * the M9 boot ROM image sim/sw/bootrom/build/bootrom.hex (top_soc's
#     BOOTROM_FILE), a gitignored BUILD PRODUCT copied from this working
#     tree; its md5 goes in the summary. Build it first:
#       make -C sim/sw/bootrom RISCV_PREFIX=<prefix>
#
# Environment:
#   GOWIN_HOST   ssh target            (default giacomo@192.168.10.36)
#   GW_SH        gw_sh path on the host (default ~/gowin_ide/IDE/bin/gw_sh)
#   SEED_OPTION  set_option name for the seed (default: probed, see above)
#   PNR_EXTRA    extra Tcl lines inserted before `run pnr`
#
# Reports land in impl/runs/<tag>/ (gitignored): run.log, the synthesis
# report, the PnR timing reports (.tr.html, plain-text .tr.txt and
# .timing_paths -- the 25-worst-paths endpoint list), the resource report,
# and a grep of the .vg for the BSRAM / DSP primitives.
set -euo pipefail

usage="usage: impl/remote_pnr_soc.sh <git-ref|WORKTREE> <tag> [seed]"
REF=${1:?$usage}
TAG=${2:?$usage}
SEED=${3:-}
HOST=${GOWIN_HOST:-giacomo@192.168.10.36}
GW_SH=${GW_SH:-'~/gowin_ide/IDE/bin/gw_sh'}
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJ=linux_soc
TOP=top_soc
BOOTROM=sim/sw/bootrom/build/bootrom.hex

case "$TAG" in
    */* | .* | "" | rv32imac_Zicsr_Zifencei)
        echo "refusing tag '$TAG' (no slashes, no dot-names, never the frozen run dir)" >&2
        exit 2
        ;;
esac
if [ -n "$SEED" ]; then
    [[ "$SEED" =~ ^[0-9]+$ ]] || { echo "seed must be a non-negative integer" >&2; exit 2; }
    TAG="$TAG-s$SEED"
fi
RUN="gowin_proj/$TAG"
OUT="$REPO_ROOT/impl/runs/$TAG"

cd "$REPO_ROOT"
[ -f "$BOOTROM" ] || {
    echo "missing $BOOTROM (build it: make -C sim/sw/bootrom RISCV_PREFIX=...)" >&2
    exit 2
}

STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT

if [ "$REF" = WORKTREE ]; then
    SHA="WORKTREE@$(git rev-parse --short HEAD)$(git diff --quiet HEAD -- . 2>/dev/null || echo '+dirty')"
    # Everything the project file names except the generated cache RTL.
    mapfile -t FILES < <(grep -oE 'path="[^"]*"' "$PROJ.gprj" | sed 's/path="//;s/"$//' |
        grep -v '^impl/soc_rtl/')
    tar -c "$PROJ.gprj" "impl/${PROJ}_process_config.json" "${FILES[@]}" | tar -x -C "$STAGE"
    impl/soc_rtl.sh INDEX "$STAGE/impl/soc_rtl" >/dev/null
else
    SHA=$(git rev-parse --verify "$REF^{commit}")
    git archive "$SHA" | tar -x -C "$STAGE"
    [ -f "$STAGE/$PROJ.gprj" ] || { echo "$PROJ.gprj not in $REF" >&2; exit 2; }
    impl/soc_rtl.sh "$SHA" "$STAGE/impl/soc_rtl" >/dev/null
fi
mkdir -p "$STAGE/$(dirname "$BOOTROM")"
cp "$BOOTROM" "$STAGE/$BOOTROM"

ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST" "test ! -e ~/$RUN" || {
    echo "~/$RUN already exists on $HOST -- pick a new tag" >&2
    exit 2
}

# Seed option name: probed from the tool unless given.
SEED_TCL=""
if [ -n "$SEED" ]; then
    if [ -z "${SEED_OPTION:-}" ]; then
        SEED_OPTION=$(ssh "$HOST" "printf 'help set_option\n' > /tmp/gw_help_\$\$.tcl && \
            QT_QPA_PLATFORM=offscreen QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1 \
            $GW_SH /tmp/gw_help_\$\$.tcl 2>&1; rm -f /tmp/gw_help_\$\$.tcl" |
            grep -oiE -- '-[a-z_]*seed[a-z_]*' | head -1 || true)
    fi
    if [ -z "$SEED_OPTION" ]; then
        echo "gw_sh exposes no *seed* option (help set_option); a seed run would" >&2
        echo "reproduce the unseeded one exactly (PnR is deterministic, D4)." >&2
        echo "Set SEED_OPTION=-<name> if the probe missed it, or run without a seed." >&2
        exit 3
    fi
    SEED_TCL="set_option $SEED_OPTION $SEED"
fi

echo "== shipping $SHA (+ regenerated impl/soc_rtl, bootrom.hex) to $HOST:~/$RUN"
ssh "$HOST" "mkdir -p ~/$RUN"
tar -c -C "$STAGE" . | ssh "$HOST" "tar -x -C ~/$RUN"

# One Tcl wrapper for both steps. Options are set here, before `run`, AND in
# impl/linux_soc_process_config.json (what an IDE run of the gprj reads):
# without that file a GUI run parsed every file as Verilog 2001 and failed on
# the first `package` (EX3444), 2026-11-27. impl/pnr/cmd.do is an OUTPUT. Values and reasons as impl/pnr_check.tcl
# (timing-priority placer/router pair 2 / 1, plain-text per-path report) and
# the cache repo's impl/pnr_check.tcl (-init_primitives 1: top_soc's reset
# synchronizer relies on its power-up value).
TCL=$(
    cat <<EOF
open_project /home/\$env(USER)/$RUN/$PROJ.gprj
set_option -top_module $TOP
set_option -verilog_std sysv2017
set_option -global_freq 50.000
set_option -place_option 2
set_option -route_option 1
set_option -gen_text_timing_rpt 1
set_option -init_primitives 1
run syn
$SEED_TCL
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
scp -q "$HOST:~/$RUN/run.log" "$HOST:~/$RUN/run.tcl" "$OUT/" || true
scp -q "$HOST:~/$RUN/impl/gwsynthesis/*_syn.rpt.html" "$HOST:~/$RUN/impl/gwsynthesis/*_syn_rsc.xml" \
    "$OUT/" 2>/dev/null || true
scp -q "$HOST:~/$RUN/impl/pnr/*.tr.html" "$HOST:~/$RUN/impl/pnr/*.rpt.html" "$OUT/" 2>/dev/null || true
scp -q "$HOST:~/$RUN/impl/pnr/*.tr.txt" "$HOST:~/$RUN/impl/pnr/*.timing_paths" "$OUT/" 2>/dev/null || true
ssh "$HOST" "grep -hoE '^ *(SDPB|SDPX9B|DPB|DPX9B|SP|SPX9|pROM|pROMX9|MULT18X18|MULT36X36|MULTALU[0-9X]*|RAM16S[A-Z0-9]*) ' \
    ~/$RUN/impl/gwsynthesis/*.vg | sort | uniq -c" >"$OUT/primitives.txt" 2>/dev/null || true

{
    echo "ref $REF = $SHA"
    echo "tag $TAG  top $TOP  host $HOST  seed ${SEED:-<none>} ${SEED_TCL:+($SEED_TCL)}"
    cat "$STAGE/impl/soc_rtl/REV" | sed 's/^/  soc_rtl: /'
    echo "bootrom (md5, from the working tree at $(git rev-parse --short HEAD)):"
    md5sum "$BOOTROM" | sed 's/^/  /'
    echo "PNR_EXTRA: ${PNR_EXTRA:-<none>}"
    grep -h -i -m3 -E 'Max Frequency|Actual Fmax|clk_core +[0-9]' "$OUT"/*.tr.html 2>/dev/null || true
} >"$OUT/SUMMARY.txt"
cat "$OUT/SUMMARY.txt"
echo "== reports in $OUT"

# Linux bring-up — running log

Companion to `docs/linux_assessment.md` (plan, criteria). This file records,
per milestone: what was done, what was measured, what was decided without
supervision (with the reasoning), and what remains. Newest entries are at the
bottom of each milestone. Dates are absolute.

Execution mode from 2026-11-22: **unattended**. The owner cannot answer
questions. Ambiguities are resolved conservatively and recorded here.

---

## Needs Giacomo

Physical or host actions that this session cannot do. Each has exact
commands. Nothing below blocks the milestones that do not depend on it.

### NG-1 — Gowin host unreachable from this machine (blocks every PnR checkpoint)

`ssh giacomo@192.168.10.36` → `No route to host` / `Connection timed out`
(2026-11-22, retried several times). This machine's network (WSL,
172.27.176.0/20) has no route to 192.168.10.0/24, presumably because it is
not on that LAN or VPN. Every PnR checkpoint below is queued here as one
command. From a machine that reaches the host, with this repo checked out
and the firmware built (`make sw-all RISCV_PREFIX=...` — the I-mem/D-mem
INIT_FILE hex are build products and are shipped from the working tree):

```bash
git fetch && git checkout linux-soc    # for impl/remote_pnr.sh
# M0 baseline: current main (FFT + I2S, never PnR'd)
impl/remote_pnr.sh b96a49b m0-main-b96a49b
# M1 checkpoints, one variable per run (on main-fixes)
impl/remote_pnr.sh 091b6e2 m1a-misa-091b6e2
impl/remote_pnr.sh 8e647f7 m1b-counters-8e647f7
impl/remote_pnr.sh 8943377 m1c-illegal-csr-8943377
```

Each writes only `~/gowin_proj/<tag>/` on the host (never the frozen
`~/gowin_proj/rv32imac_Zicsr_Zifencei/`), refuses an existing tag, and copies
reports into `impl/runs/<tag>/` (gitignored), including
`pht_primitives.txt`, the `.vg` grep that answers the PHT-in-BSRAM question.

### NG-2 — CoreMark baseline image on the board (GCC 14.3.0, 2000 iterations)

The M0 baseline image is `sim/sw/coremark/build-m0-i2000/{imem,dmem}.hex`
(GCC 14.3.0, `ITERATIONS=2000`, `CLK_HZ=50000000`). It is a build product;
rebuild it identically with:

```bash
make -C sim/sw/coremark RISCV_PREFIX=~/_toolchains/riscv32-ilp32d--glibc--stable-2025.08-1/bin/riscv32-buildroot-linux-gnu \
     ITERATIONS=2000 BUILD=build-m0-i2000
```

To run it on silicon, point `top_module.sv`'s two `INIT_FILE`s at those two
files, synthesise + PnR (NG-1 flow, e.g.
`impl/remote_pnr.sh <commit> m0-coremark-board`), flash the `.fs`, and capture
the UART at 115200 8N1. Record `Ticks`, `Cycles / iteration` and
`crcfinal`. **Compare `Ticks` against the simulated figure in M0 below:
equal means the PHT netlist predicts exactly as the RTL does** (the cheap
closure of the PHT-in-BSRAM question). A difference means the netlist and the
RTL diverge — read `impl/runs/<tag>/pht_primitives.txt`.

---

## Standing decisions (2026-11-22)

| # | Decision | Reasoning |
|---|---|---|
| D1 | Docs and the progress log live on `linux-soc`. That branch starts from `main` at `b96a49b`, and `main-fixes` is merged into it (merge, never rebase) after M1 | `main` is never committed to. The plan documents are Linux work. A merge keeps both branches' history intact |
| D2 | `main-fixes` branches from `main` at `b96a49b` | It carries only the M1 fixes for the owner's review |

---

## M0 — Baseline

**Done 2026-11-22 (local part):**
- Firmware rebuilt with GCC 14.3.0 (`riscv32-ilp32d--glibc--stable-2025.08-1`).
  `make regress` 20/20, `make compliance` 62 pass / 5 XFAIL / 0 fail,
  cosim quicksort PASS 29293 retires, ecall PASS 17. That is the baseline on `main` `b96a49b`.
- yosys estimate of `main` (`impl/yosys_estimate.sh --no-baseline`): LUT1-4
  9860, MUX2 2535, ALU 1784 = **14179 logic cells**, FF 3825, **BSRAM 23**,
  MULT18 4, MULT36 1, LUT-RAM 149. (Yosys caveats: the regfile is patched out of BSRAM,
  and the PHT is forced into flops.)
- yosys estimate of the cache subproject's `fpga_top`: **~34.8k logic cells**,
  1739 FF, 36 BSRAM (single-port SPX9). The M4 diagnosis starts from this.
- CoreMark GCC 14.3.0, 10 iterations, sim: **4 134 117 ticks = 413 411
  cycles/iteration = 2.41 CM/MHz** (not a valid run: too short). 2000-iteration
  sim: see below.

**Blocked (NG-1, NG-2):** PnR of `main`, the GowinSynthesis resource table,
the `.vg` PHT grep, and the board run of the 14.3.0 image.

**Decisions:**
| # | Decision | Reasoning |
|---|---|---|
| D3 | The PHT-in-BSRAM question is closed only by NG-2 / NG-1. M1 and M2 proceed without it | M1 and M2 do not touch the predictor or BSRAM. Blocking on it would idle every milestone |
| D4 | "≥ 3 seeds" is replaced by **one run per configuration, plus a confirmation re-run of the same tree** when the tool exposes no seed option | Recorded on 2026-08-31 (`gowin-pnr-measurement-method` memory note): Gowin PnR here is deterministic, so the same tree reproduces to the picosecond. The ~0.7 ns figure in CLAUDE.md is the effect of any netlist change re-placing the design, not run-to-run noise. Several identical runs would measure nothing. If `gw_sh` exposes a seed, `PNR_EXTRA` in `impl/remote_pnr.sh` takes it and the plan's 3 seeds apply as written |
| D5 | PnR tooling: new `impl/remote_pnr.sh`, one directory per run | The owner asked for separate directories and an untouched run 8; the old flow rsynced over a single directory |
| D6 | Pre-existing `make format-check` failures on `main` (`axi4_lite_i2c.sv`, `axi4_lite_spi.sv`, `top_module.sv`) are left alone; only files a commit touches are formatted | Reformatting unrelated files would mix noise into the reviewed diffs |

---

## M1 — CSR correctness on `main-fixes`

**Done 2026-11-22.** Three commits on `main-fixes` (branched from `main`
`b96a49b`), plus one CLAUDE.md commit. `main-fixes` was merged into
`linux-soc` (merge commit `f956a9c`).

| Commit | Change | Evidence |
|---|---|---|
| `091b6e2` M1a | `misa` 0x4000_2105 → 0x4000_2104 (A no longer claimed) | regress 21/21 with the new `sw/isa/csr_ops`; mutant (old misa) FAILs |
| `8e647f7` M1b | `mcycleh`/`minstreth`; `cycle`/`time`/`instret` + `h` shadows; `time` = CLINT `mtime` (new `clint_timer.mtime_o` → CPU `mtime_i`); write-suppress semantics; **minstret counts on the commit cycle (`execute.commit_o`)** | regress 21/21; compliance **63 / 4 XFAIL / 0** (`instret_overflow` XPASS, removed from the list); two mutants FAIL; cosims PASS; CoreMark ticks unchanged |
| `8943377` M1c | Illegal-instruction trap on unimplemented CSR / write to a read-only CSR; `mhartid` & co, `mstatush`, `tselect`/`tdata1..3` read 0 | regress 21/21; compliance 63/4/0; pre-change RTL FAILs the new checks; cosims PASS; CoreMark ticks unchanged |
| `caf1d6d` | CLAUDE.md updated | — |

**Measured:** CoreMark 10-iteration ticks are identical across M1
(4 134 117), so M1 costs nothing in cycles. **PnR not measured** (NG-1); the
three commands are queued there.

**Decisions:**
| # | Decision | Reasoning |
|---|---|---|
| D7 | `minstret` counts on `op_commits` (new port) instead of compensating in the read mux | The off-by-one was a real timing bug: counting on the registered `ex_valid` lands a cycle late. Fixing it at the source costs one wire. Compensating in the read would add an adder to the read path and still leave the write-suppress semantics wrong |
| D8 | `tselect`/`tdata1..3` read 0 instead of trapping | `rv32mi-breakpoint` (passing before M1c) broke when they trapped. Reading 0 is the debug spec's "no trigger here" answer, which OpenSBI's probe also accepts. This is the more conservative of the two legal behaviours, because it keeps an external test passing |
| D9 | `mstatush` and `mconfigptr` read 0 rather than trapping | Mandatory / harmless for RV32 M-mode. A trap there could break firmware that reads them unguarded |
| D10 | `time` is wired from the CLINT rather than duplicated as a second counter | A duplicate would diverge once the CLINT `mtime` becomes writable (standard layout, M8) |

**Remaining:** PnR ×(1 + confirmation) for each of the three commits (NG-1).
The owner reviews and merges `main-fixes`.

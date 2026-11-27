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
  cycles/iteration = 2.41 CM/MHz** (not a valid run: too short).
- **CoreMark GCC 14.3.0, 2000 iterations, sim on `main` `b96a49b`: 824 488 511
  ticks = 412 244 cycles/iteration = 2.42 CM/MHz, `crcfinal = 0x4983`,
  "Correct operation validated"**; predictor 91.97% accuracy, 15.30 MPKI.
  This is the number NG-2's board run must reproduce exactly.

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

---

## M2 — A extension (`linux-soc`)

**Done 2026-11-22**, commit `b6373b6`.

- LR.W / SC.W / nine AMO*.W, built on the existing LSU (details in the
  commit and in `execute_stage.sv`). Atomics to the peri port take access
  fault 5/7. Reservation cleared on every SC and every trap/interrupt/mret.
  `misa` = 0x4000_2105 on this branch.
- `ZILX_EN = 0` on `linux-soc` (D11). quicksort's hand-encoded `lxs.w`
  falls back to plain C (`QS_ZILX`).

**Measured:** `make regress` 23/23 (new `isa/amo_ops`, 29 checks, and
`amo_stress`, 600 random atomics vs a C shadow); `make compliance` 73 pass /
4 XFAIL / 0 fail with `rv32ua` enabled (10/10); Spike cosim `amo_stress`
PASS 62 581 retires, quicksort PASS 33 589 (no-Zilx build), ecall PASS 17;
CoreMark 10-iteration ticks 4 134 117 (unchanged). PnR pending (NG-1).

**Mutation checks:** AMOMIN signed→unsigned, SC ignoring the address, and
the reservation surviving a trap each FAIL `amo_ops`. One mutant survives:
writing back the load data instead of `amo_old_q`. `native_ram` holds its
read output during the store phase, so the two are equivalent on BSRAM. The
flop stays because a cache's output will not hold. This is a known blind
spot of the oracle on BSRAM, not a bug.

| # | Decision | Reasoning |
|---|---|---|
| D11 | Zilx behind `rv32_pkg::ZILX_EN`, 0 on `linux-soc` | Owner's instruction. The encodings do not collide with A, so a parameter costs nothing and `main` keeps Zilx |
| D12 | Atomics to the peri port raise an access fault instead of being executed through the AXI bridge | A read-modify-write through the single-phase bridge to a side-effecting slave is not used by any supported software. Refusing it keeps the bridge unchanged. This is the conservative option |
| D13 | aq/rl bits accepted and ignored | One hart, in-order, every access already globally ordered |

---

## M3 — Variable-latency tolerance (`linux-soc`)

**Done 2026-11-22** (sim). PnR pending (NG-1).

- `fetch_stage.sv`: the single in-flight PC register became a depth-2 FIFO
  (`req_pc_q` oldest, `req_pc2_q` younger). `issue_addr` stays on the last
  2:1 into `req_pc_q`, an endpoint of the frozen worst path.
- `sim/mem_fuzz.sv`: a latency-fuzz shim between each CPU port and its
  `native_ram`. With `+MEM_FUZZ=<seed>` it refuses ~25% of accepts, caps
  outstanding reads at the port's depth, and delays each response by 0–7
  extra cycles in order. Without the plusarg it is pure wires.
- `sim/Makefile`: `REGRESS_ARGS`, `REGRESS_CYC_MULT`, and the
  `regress-fuzz` target (seeds 1 2 3, MAX_CYC × 8).

**Bug found in review, fixed before use as the gate (2026-11-22).** The
first `regress-fuzz` piped `$(MAKE) regress | tail -1 || rc=1`. The `||`
then saw `tail`'s exit status, so a failing regress could never set `rc`
and the target always passed. The owner spotted it. Fix: capture make's
status into `rc` before trimming the output
(`out=$$($(MAKE) ... 2>&1) || rc=1; echo "$$out" | tail -1`).
**Proven able to fail:** `make regress-fuzz FUZZ_SEEDS=1 FUZZ_CYC_MULT=1`
(cycle bounds too small for a fuzzed memory) reports `guitar_tuner FAIL`
and exits **2**. Only after that was it used as the M3 gate.

**Measured:**
- `make regress-fuzz` (seeds 1 2 3, MAX_CYC × 8): **23/23 on every seed**, exit 0.
- Fuzz off: every regress cycle count is **identical** to the pre-M3 tree
  (diff of the PASS lines empty), so M3 costs zero cycles on BSRAM.
- Mutant (FIFO reverted to the single register): under `+MEM_FUZZ=1`,
  quicksort, bp_pred, csr_ops, amo_ops, amo_stress, div_ops, rvc_scramble,
  span_target and more FAIL. So the fuzz really does put two reads in
  flight, and the FIFO is what makes them correct.
- LSU: no change was needed. Every D-mem path is already handshake-driven,
  and the whole set passes with random accept refusal and 0–7 cycles of
  response delay.

---

## Session state at stop (2026-11-22, owner asked to stop for the day)

**Branches:**
- `main-fixes` @ `caf1d6d` — M1a/M1b/M1c + CLAUDE.md. Ready for review
  once PnR runs (NG-1).
- `linux-soc` — docs, M1 merge, M2 (`b6373b6`), M3 (this commit).
- Cache subproject (`src/sub/yarv32-native-caches`, its own git repo):
  branch `m4-resource` created at `8f67d78` by the parallel M4 agent,
  **no commits**. The agent was stopped before its per-module yosys
  breakdown finished. What it confirmed: baseline `make sim` passes,
  `make bist` PASS, format clean; `iverilog` is not installed, so
  `make gatesim` cannot run on this machine.

**Resume here:** M4 step 1, the per-module cell breakdown of the cache's
`fpga_top` (yosys, non-flattened `stat`), then §6.1 budget → §6.2 option
choice → implementation on `m4-resource`. After that: M5 (Linux SoC top),
M6 (SDRAM bursts, ≤ 20-cycle clean miss), M7 (`fence.i`), M8 (standard
peripherals), M9 (loader), M10 (nommu boot), then the §6.3 review.

**Still blocked:** NG-1 (Gowin host unreachable: every PnR checkpoint) and
NG-2 (board CoreMark run of the 14.3.0 image).

Also done this session: Python allowlisted for Bash in
`.claude/settings.local.json` (gitignored, local only).

---

## Session 2 (2026-11-22 evening → 2026-11-27): parallel workers

Three workers ran in parallel: A on M4 (cache repo), B on M8 (git worktree), and
C on M10 prep (outside the repo). The main thread did the core side of M7
and prepared M5.

### M4 — cache resource reduction (worker A, cache repo branch `m4-resource`)

Commits in `src/sub/yarv32-native-caches`: `c63238f` (64-bit data macros,
a line = four 64-bit beats), `afd50f3` (clean-miss latency probe),
`14747b9` (constant-index way requests, one mux per skid read), `412e656`
(TODO Phase 10 findings). `make sim`, `make bist`, `+RAM_GARBAGE`,
`lint-fpga`, `format-check` all green. No gatesim (no iverilog).

| step | yosys flat logic cells | BSRAM |
|---|---|---|
| baseline `8f67d78` | ~34 840 | 36 |
| `14747b9` | **7 769** | **12** |

**Root cause of the 34.8k:**
- 256-bit data macros (8 blocks each, and the full line width runs through every mux).
- More of it came from **dynamically indexed array writes/reads**
  (`imem_req[fsm_victim_way] = …`, `skid_q[req_sel_q][miss_slot_q]`, seven
  sites). After sv2v + yosys these become per-bit priority muxes and 556-bit
  shifters. The pre-mapping mux weight went from 191 731 to 30 696.

**Clean-miss latency (sim, `make bist`, 50 MHz): min 78 / avg 82 / max 90
core cycles.** This is the M6 baseline; the target is ≤ 20.

**Budget (§6.1, provisional, yosys units): does not fit.**
- Usable: 20 736 × 0.75 = 15 552.
- Worker A's core estimate: 12 570. The M8 worker measured the whole
  Linux-branch top (core + UART16550/CLINT/PLIC/SPI/GPIO, before the cache)
  at **13 543**, so the measured remainder is ~2.0k, against a cache at 7.8k.
- Caveat that cuts the other way: yosys builds the register file in
  flops (the script patches it out of BSRAM; GowinSynthesis maps it to
  BSRAM). So the yosys core figure is inflated by roughly the 32×32 regfile
  plus its read muxes (≈ 1k FF and 1–2k LUT [E]). A GowinSynthesis table is
  the only real answer (NG-1).

| # | Decision | Reasoning |
|---|---|---|
| D14 | Keep option **C1** (write-back) at 7.8k cells | Worker A measured C2's lower bound at 7 753 cells and the C3 hack at 9 815 (unreliable). The write policy is not where the logic is: it is in the request machinery (2-slot skids, response queues, uncached-path arbitration, `rr_q[2][128]`), duplicated for I and D. C2/C3 would cost CPI for no area. The §6.2 rule ("first that fits") has no fitting option, so the conservative reading is to keep the option with the best CPI at equal area and keep reducing |
| D15 | M4 continues as **M4b**: a second reduction pass on the request machinery (I-port skid of 1 if fetch allows, no response queue, round-robin state to LUT-RAM or 1 bit per cache, one uncached path), with the CPU-facing ports frozen so M5 integration can proceed in parallel | The budget is missed even before the margin. Reduction is cheaper now than after M5–M7 build on the controller |
| D16 | M5 integrates the current `m4-resource` cache. `src/sub/yarv32-native-caches` becomes a submodule pinned to a commit of `m4-resource` | Owner's decision (submodule, pinned). The CPU-facing interface is frozen for M4b, so the pin can move forward without M5 rework |

### M7 core side (main thread) — `f6aa60e`
`fence.i` is a real flush + refetch. EX_FENCEI holds `fencei_req_o` until
`fencei_ack_i` (tied 1 in the CPU top until the cache exists). New oracle
`isa/fencei_ops`; regress-fuzz green. Details are in the commit.

### M5 prep (main thread) — `4c9d02a`
`src/rtl/utils/dport_32to64.sv` (32-bit LSU → 64-bit native port) +
`sim/hw/dport_tb`: 1947 checks PASS, and two mutants caught (950 / 1512 errors).

### M8 — standard-compatible peripherals (worker B) — merged `3afeeda`, fix `8a61f1f`
- New Linux-branch map: UART16550 `0x1000_0000` (`ns16550a`, reg-shift 2,
  io-width 4, DT `clock-frequency = <50000000>`, DL 27 → 115 741 baud);
  SPI `0x1000_6000`; GPIO `0x1000_7000`; CLINT `0x1001_0000`/64 KiB
  (`sifive,clint0`); PLIC `0x1040_0000`/4 MiB (`sifive,plic-1.0.0`, ctx 0 = M,
  ctx 1 = S `seip_o` unconnected until M11). FFT/I2S/I2C and the MCU
  CLINT/PLIC/UART are gone from both Linux-branch tops.
- TBs: `clint_tb` 60, `plic_sifive_tb` 146, `uart16550_tb` 125 checks, all
  PASS, each with mutants caught. `docs/linux_soc.dts` compiles with dtc.
- Regress drops five main-only tests on this branch (fft, guitar_tuner,
  i2c_reg, i2c_scan, i2s): **19/19**. After the merge `fencei_ops` still
  used the old MSIP address; `8a61f1f` fixes it.
- Worker decisions recorded in the merge's commits: writable `mtime`,
  3-bit PLIC priority/threshold (all reset 0), 16550 is 8N1-only (LCR
  stored, not applied), THRE edge interrupt, sim UART 3.125 MBaud.
- Open: driver offsets come from the worker's knowledge of the drivers and
  must be re-checked against the pinned 6.18 source (M10). The yosys
  whole-top figure is 13 543 cells / 3 132 FF / 16 BSRAM.

### M10 prep (worker C)
Stopped by an API usage limit during the Buildroot toolchain build in
`~/yarv-linux/`. It is resumed in this session.

### M5 — Linux SoC sim, IN PROGRESS (stopped 2026-11-27 at the owner's request: session limit)

Done:
- `src/sub/yarv32-native-caches` is a submodule pinned to `m4-resource`
  `412e656` (`0b6d83c`). **NG-3: that commit is local-only; push the cache
  repo's `m4-resource` branch before anyone clones.**
- `sim/soc/` scaffolding (WIP commit): `sim_soc.sv` (sim_top with the BSRAMs
  replaced by `dport_32to64` + `cache_cntrl` + sdram controller +
  `sdram_model`, CPU `IMEM_ADDR_W=24`, boot at the bootrom 0x80_0000, a
  CPU-store shadow for PROBE), `mkimage.py` (merges imem.hex + dmem.hex into
  one SDRAM image, refuses overlap), `bootrom_jump0.hex`, and a Makefile that
  builds the cache from the PINNED commit (`git archive`), not the submodule
  working tree. `sim_main.cpp` gains `-DSOC_BUILD`: PROBE reads the shadow, and
  `$time` advances 10 ns per half period, because the SDRAM model checks real
  ns. The main BSRAM sim is unchanged (regress 19/19).

**Blocker hit, with the fix to apply next:** both projects wildcard-import
their package at FILE scope (`import rv32_pkg::*;` / `import
yarv32_cache_pkg::*;`). Verilator compiles everything into one `$unit`, so
the cache's `mem_req_t` resolves to `rv32_pkg`'s (`Member 'valid' not found
in structure`). Fix (next step): in the Makefile's snapshot step, rewrite
the snapshot copies of the cache files to use package-qualified types
(`yarv32_cache_pkg::mem_req_t` etc.) and drop their file-scope import.
The submodule itself is not edited.

Workers at stop time:
- A: M4b (request-machinery reduction), then M6 (SDRAM bursts, target ≤ 20
  cycles from the measured 78/82/90), in the cache repo.
- C: resumed M10 prep (Buildroot nommu 6.18 build in `~/yarv-linux/`, plus
  checking the CLINT/PLIC/8250 offsets against the 6.18 sources).
Both may finish after this log entry; their results go in the next session.

Remaining in order: finish M5 (fix above, then regress on the SoC,
quicksort/amo cosim, CoreMark relinked unified, PnR → NG-1), M6, M7 proper
(cache clean+invalidate ack), M9 (loader on 16550 + SPI flash), M10, §6.3 review.

**Workers stopped at the same point (2026-11-27):**
- A: committed `781c0b4` "store only what the request machinery reads
  (M4b step A)" on `m4-resource` (not yet pinned; the pin stays `412e656`).
  It also left an **uncommitted** edit in `src/rtl/cache_cntrl.sv` of the
  cache repo: its next M4b step, unreviewed. Left untouched. Review it or
  discard it (`git -C src/sub/yarv32-native-caches diff`) when resuming.
- C: the RV32 nommu 6.18 kernel built in `~/yarv-linux/` (`Image`
  3 501 464 B, initramfs built in). It was about to boot it on QEMU with
  8 MiB and icount. NOTES.md / build.sh state is whatever it wrote; verify
  them before relying on them.

### M5 — continued (2026-11-27, after the owner said "57%, continue")

- SoC sim builds and runs (`56a594a`): `qualify_pkg.py` resolves the package
  collision in the build snapshot; `$time` advances in the design precision
  (the SDRAM model saw ps before), so all runs are free of SDRAM protocol
  errors.
- **SoC regress (`make -C sim/soc regress`): 15 passed, 1 failed (amo_ops),
  1 skipped (ifault: the fault boundary is 16 MiB on the SoC).**
  `wfi_trap` needed a timer armed relative to `mtime` (`1f6aa62`): absolute
  400 fired before the program started on the SoC. The BSRAM regress is still 19/19.
- **Spike cosim on the SoC: quicksort PASS 33 589 retires, amo_stress PASS
  62 581** (same image, SoC trace vs Spike).
- Unified link for programs above 8 KiB of text: `sim/soc/link_unified.ld`
  + `start_unified.S` (sp = 8 MiB) + `elf2sdram.sh`.

**CACHE BUG found by M5: the I-port delivers responses out of order.**
The native protocol and the fetch stage need strict in-order delivery. With
`+ITRACE`:
- `amo_ops`: requests 0x40, 0x48 (same missing line) answered 0x48 first,
  then 0x40.
- CoreMark (unified): 3 of 14 820 I responses rotated across a miss, a
  same-line follower and a hit after a redirect. The core then executes
  wrong instructions and traps (a misaligned load from a corrupted pointer).
The core's own latency fuzzer is in-order by construction, so this is not
a core problem. Reported to worker A as top priority, ahead of M4b. The
submodule pin moves to its fix.

| # | Decision | Reasoning |
|---|---|---|
| D17 | The SoC regress skips `ifault` instead of changing the test | Its expectation (fault above 16 KiB) is a property of the MCU's I-mem. The SoC's 16 MiB boundary is checked by the fetch range logic it shares; a SoC-specific ifault test can come with M9 |
| D18 | `wfi_trap` arms its timer relative to `mtime` on this branch | An absolute deadline is an assumption about boot time, not about the WFI logic under test |

### M5 + M9 status at stop (2026-11-27, session at ~82%)

- **Cache I-port ordering bug FIXED by worker A**: `ffa1ad8` on
  `m4-resource` (skid slots are a free list, so the FSM picked the youngest
  waiting miss; now an age bit picks the oldest). New cache `sim_top` phase O
  (O1–O3); the pre-fix RTL fails O1 and O3. **Submodule pin moved to
  `ffa1ad8`** (still local-only, NG-3). M4b steps A/B (`781c0b4`, `aa03132`)
  are included in the pin; M4b's cell count after them was not reported before stop.
- **SoC regress: 17 passed, 0 failed, 1 skipped (ifault)**, including
  `amo_ops` and the new `yload_qs`. BSRAM regress 19/19.
- **M9 loader (`sim/sw/bootrom/yload.S`, 424 of 2048 ROM bytes)**:
  - Framed protocol: `YL`, load address, length, entry, payload, CRC-32.
    `yframe.py` builds the frame.
  - Stores go to SDRAM with `CACHE_BYPASS` set, then `fence.i` and a jump
    to the entry point.
  - Boot order: the SPI flash at **2 MiB** (D19), then the UART if the magic
    is absent or the CRC fails.
  - Sim: UART boot of quicksort PASS (1 876 137 cyc); flash boot of
    quicksort PASS (`+SPIFLASH +FINIT`, new `sim/spi_flash_model.sv`, READ
    0x03 + JEDEC 0x9F). Both reach `0x600D`.
- Not done: CoreMark on the SoC after the fix (unified build ready in
  `sw/coremark/build-soc-i10`, image `soc/img/coremark10.hex`); board pin
  mapping of the configuration flash (NG-4); M6 (bursts, latency baseline
  78/82/90 → target ≤ 20); M7 proper (cache clean+invalidate ack); M10.
- Worker C had built the RV32 nommu 6.18 `Image` (3.5 MB) and was
  rebuilding a trimmed one when stopped (`~/yarv-linux/`). Its QEMU boot
  result is not in yet.

| # | Decision | Reasoning |
|---|---|---|
| D19 | Flash image offset 2 MiB | The GW2AR-18 bitstream is ~0.9 MB (the `.fs` holds ~7.26 M bit characters). 1 MiB would be tight; 2 MiB leaves 6 MiB, enough for the 3.5 MB kernel |
| D20 | The loader tries flash first, then UART | Unattended boot needs flash. UART remains the recovery path, so a bad flash image can never brick the board |

**NG-4 (board):** the Tang Nano 20K configuration-flash pins
(MSPI/SPI pins of the GW2AR-18C QFN88) must be mapped to the SoC SPI
master in the board top's `.cst`, and "MSPI as GPIO" may need to be enabled in
the Gowin device configuration. This is unverified; see the
`ArthurHeymans/tang_20k_spi_flash` reference in the assessment. Then flash
the frame at offset 0x200000, e.g. with `openFPGALoader --external-flash
-o 0x200000 frame.bin` (frame from `sim/sw/bootrom/yframe.py`).

### Cache: M4b step C committed, pin moved (2026-11-27)
- Cache repo `m4-resource`: `3271811` (constant-index skid fill + line-buffer
  writes, finishing worker A's uncommitted step) and `f6c8fe5` (begin/end so
  Verible keeps the indentation). Cache `make sim` (also with `+RAM_GARBAGE`)
  and `make bist` PASS; format-check clean.
- Pin moved to `f6c8fe5`. SoC regress: 17 passed, 0 failed, 1 skipped.
- **Push NOT done (NG-3 still open):** this machine has no GitHub
  credentials. HTTPS has no username/helper, and SSH reports
  `git@github.com: Permission denied (publickey)` (the local
  `~/.ssh/id_ed25519` is not registered on GitHub). To push:
  `git -C src/sub/yarv32-native-caches push -u origin m4-resource`
  from a machine/account with access.

### Paused (2026-11-27, owner: "pause")
- **CoreMark on the SoC** (cache pin `f6c8fe5`, before M6 bursts):
  **617 529 cycles/iteration = 1.61 CoreMark/MHz**, CRCs correct (BSRAM:
  413 411 = 2.41). CPI 2.075: load-wait 0.355, lsu-launch 0.252, imem-starve
  0.260, redirect 0.077. The misses are where the cost is, which M6 targets.
- RAM base decision **D21: Linux at 0x8000_0000**. The cache decodes bits
  [23:0] only, so 0x8000_0000 aliases SDRAM 0. The SoC CPU now uses
  `IMEM_ADDR_W = 32` (no fetch fault). `fetch_stage.sv` computes the fault
  as `|(pc >> IMEM_ADDR_W)` so a width of 32 is legal (same wires below 32).
  BSRAM regress 19/19, SoC regress 17/0/1.
- Workers A (M6 bursts, then the cache side of M7) and C (QEMU boot of the
  trimmed nommu kernel, DT at 0x80000000, built-in DTB) were both resumed and
  then stopped by the API usage limit. Resume them with SendMessage. C was
  told about the 0x80000000 base.

## RESUME HERE (handoff for a fresh session, 2026-11-27)

State: branch `linux-soc` at the commit that adds this section (parent
`b3386a6`). Tree clean. BSRAM `make regress` 19/19, SoC regress (`sim/soc`,
`make regress`) 17 passed / 0 failed / 1 skipped (ifault). Cache submodule pinned
at `f6c8fe5` (local branch `m4-resource`, plus an empty `m6-burst` at the same
commit). No agents are running; the old worker IDs cannot be resumed from a new
session, so start fresh agents with the briefs below.

Work still to do, in order:

1. **M6, SDRAM burst reads.** The target is a clean miss of 20 core cycles or
   less (78/82/90 measured today). The work lives in the cache repo (branch
   `m6-burst` from `f6c8fe5`) and its nested `src/ips/sdram-controller`
   (detached at `a08a1b7`, branch `gw2ar-32bit`). No burst work was saved:
   there is no `burst` branch and no diff, so this starts from zero.
   Acceptance: measured miss latency, cache `make sim`/`make bist` PASS, SoC
   regress green, CoreMark on the SoC re-measured (today 617 529 cyc/iter =
   1.61 CM/MHz; load-wait 0.355 and imem-starve 0.260 CPI are the target).
2. **M7, cache side of fence.i.** Add `fencei_req_i`/`fencei_ack_o` to
   `cache_cntrl` (write back the D-cache, invalidate the I-cache), add a
   `sim_top` phase in the cache repo, then wire it to the CPU in
   `sim/soc/sim_soc.sv`. The CPU side is done (`EX_FENCEI`, `fencei_req_o`);
   today `fencei_ack_i` is tied to 1. Remove `rv32ui-fence_i` from the XFAIL
   list once it passes on the SoC.
3. **M4b, cell reduction.** The cache is about 7.8k yosys cells against a
   2-3k provisional budget (D14: keep C1, keep reducing).
4. **M10 on QEMU (worker C area, `~/yarv-linux/`).** The kernel is linux 6.18.64,
   RV32 nommu M-mode, 3.5 MB Image (`build.sh`, `NOTES.md`, `boot.py`). The last
   boot (`boot-icount-8M.log`) reaches init and panics with
   `binfmt_flat: Unable to allocate RAM for process text/data, errno -12`:
   8 MiB is too tight for the flat `/bin/sh`. Next, cut memory use: a smaller
   initramfs, fewer built-ins, a smaller kernel `LOG_BUF`, a static busybox
   with a smaller stack, or an XIP-less trim. Then report the icount to
   shell, the footprint, and the check of the CLINT 0x1001_0000, PLIC
   0x1040_0000 and 16550 0x1000_0000 offsets against our DT. Base
   0x8000_0000, built-in DTB.
5. **M10 on the SoC sim**: boot the same Image on `sim/soc`. Then write the
   §6.3 go/no-go review in `docs/linux_assessment.md`. **Stop before M11** and
   wait for the owner's decision.

Open owner items (unchanged): NG-1 (PnR on 192.168.10.36, `impl/remote_pnr.sh
<ref> <tag>`), NG-2 (board CoreMark of build-m0-i2000; the sim figure is 412 244
cyc/iter), NG-3 (push the cache repo `m4-resource`, since this machine has no
GitHub credentials), NG-4 (config-flash pins, frame at 0x200000).

Do not touch `~/yarv32-native-caches/` (the owner's own clone, branch
`edit/dmem-32b`, with uncommitted edits). The submodule is the one under
`src/sub/`. Worktree `.claude/worktrees/agent-a325591640f50a755` (M8) is already
merged (`3afeeda`) and can be left alone.

Working rules: unattended, log decisions as D22 onward, few large commits, the
toolchain is in CLAUDE.md / memory, and the owner reports the session-limit
percentage (stop before 95%).

## Session 3 (2026-11-27, unattended, after the RESUME HERE handoff)

Two fresh workers: A in the cache submodule (M6, M7 cache side, M4b), C in
`~/yarv-linux` (M10 on QEMU). Main thread: integration, the SoC Linux boot,
and the core fixes that boot exposed.

### M6 — SDRAM burst reads (cache repo `m6-burst` `d45ad0d`, controller `e190d36`)
- **Clean miss at 50 MHz: 78 / 82 / 90 -> 15 / 15 / 20 core cycles** (min /
  avg / max, `make bist`). Target (≤ 20) met; the max of 20 is a refresh
  falling due as the request arrives.
- Where the 78 went: 72 were the controller doing eight single-word
  transactions of 9 cycles each; 6 were the cache FSM. No CDC cost (one clock).
- The 15 left: 1 arbitrate, 1 ACT, 1 READ (tRCD = 1 clock at 50 MHz), 3 CAS
  (CL plus the controller's input register), 8 data words, 1 unstall. Further
  cuts need early restart at the requested word, CL=2 (unchecked on the
  GW2AR die) or issuing from S_IDLE.
- The SDRAM model now checks tRAS, tRC, auto-precharge + tRP, tREFI and
  commands issued during a burst (it did not before); controller mutants
  breaking tRCD, write recovery, tRAS, refresh period and CL are each caught.
- **CoreMark on the SoC barely moved: 617 529 -> 610 201 cyc/iter (1.61 ->
  1.63 CM/MHz), CRCs correct.** load-wait stays 0.353 CPI and imem-starve
  0.243: CoreMark's working set fits the caches, so its cost is the cache
  HIT path, not the miss. That is the next performance lever, not bursts.

### M7 — fence.i (cache `2d8265b`, wired in this commit)
- `fencei_req_i` level / `fencei_ack_o` one-cycle pulse; the flush drains
  in-flight requests, writes back every dirty D line (lines stay valid,
  clean), invalidates every I line. 2 cycles per set + ~20 per dirty line.
- The CPU top gained `fencei_req_o` / `fencei_ack_i`; the BSRAM tops tie the
  ack to 1 (BSRAM is coherent), `sim_soc.sv` wires the cache.
- **SoC riscv-tests**: new `sim/soc/compliance.py` (`make compliance` in
  `sim/soc`), own expected-failure list. **75 pass, 2 expected failures**
  (`rv32ui-ma_data`, `rv32mi-pmpaddr`); `rv32ui-fence_i` and `rv32uc-rvc`
  pass on the SoC (one coherent space with a real fence.i).

### M4b — cache cells (cache `d26e8df`)
- Measurement reconciled: the old 7 769 and the later 2 996 / 3 090 were the
  same flow on `fpga_top`, which under-counts the cache by ~40% (the
  self-test prunes address decode, fence.i was tied off). New `make cells`
  synthesises a SoC-shaped wrapper. The default yosys count swings by
  thousands on small edits (wide-LUT packing); `-nowidelut` is stable.
- **2d8265b -> d26e8df: 3 221 -> 2 115 cells (-nowidelut), 5 789 -> 3 029
  (default), FF 949 -> 777, BSRAM 12.** Cuts: no line buffer (writeback
  streams from the victim way), explicit 2:1 lookup mux, one address/wdata per
  cache. Latency unchanged (15/15/20).
- Not pushed (NG-3).

### M10 — QEMU (worker C, `~/yarv-linux/NOTES.md` §3)
- **Shell in 8 MiB.** Image 1.75 MiB (was 3.5 MB), initramfs 224 KB, static
  FLAT busybox 199 KB, MemFree 5 340 kB at the prompt.
- **≈ 12.3 M instructions from reset to the prompt** (10.9 M kernel, 1.4 M
  shell start).
- SoC files: `~/yarv-linux/soc/Image` (load and entry 0x8000_0000, a0 = 0,
  built-in DTB `yarv-soc.dts`, cmdline `earlycon console=ttyS0,115200
  rdinit=/bin/sh`). DT checked against the RTL: CLINT, PLIC (`ndev = 15`,
  UART = source 1), 16550 (reg-shift 2, 50 MHz) all match.

### M10 — on the SoC sim: two core bugs found, both fixed
Boot stub `sim/soc/bootrom_linux.hex` (a0 = a1 = 0, jump to 0x8000_0000),
`+NOPROBE` (the 16 KiB probe shadow cannot take a kernel image),
`UART_RX_START=<cycles>` in `sim_main.cpp` (type into the shell only once it
listens).
1. **Boot 1** reached `Run /bin/sh as init process` (≈ 0.75 s of simulated
   time), then `Oops - environment call from M-mode`. A nommu M-mode kernel
   runs userspace in **U-mode** (`start_thread` clears MPP) and handles only
   cause 8. QEMU has U, so it never showed there. **Fix: minimal U-mode**
   (D24).
2. **Boot 2** printed the start of the BusyBox banner and then idled forever
   (IPC 0.083 for the rest of 160 M cycles). **Cause: WFI only woke on
   `int_pending`, which includes `mstatus.MIE`.** Linux's idle loop executes
   `wfi` with MIE = 0 and enables interrupts afterwards; the spec resumes WFI
   on `mip & mie` alone. **Fix: `wfi_wake` = pending & locally enabled,
   independent of MIE** (D25).
3. Boot 3 (with both fixes) is the run this commit is waiting on.

New oracle `sw/isa/umode_ops` (in both regress lists): MPP WARL, ecall from
U = 8 with MPP = 00, CSR / mret / wfi illegal in U, M-to-M mret leaves
MPP = U, M interrupts taken in U with MIE = 0, and a `wfi` with MIE = 0 that
must resume. Mutation-checked: ecall cause, the U interrupt enable, the U CSR
check and the WFI wake are each caught. `csr_ops` / `amo_ops` updated for
the new `misa`.

| # | Decision | Reasoning |
|---|---|---|
| D22 | fence.i is a level request / pulse acknowledge, flush drains first | The CPU's `EX_FENCEI` already holds a level; draining needs no precondition from the caller |
| D23 | The SoC gets its own compliance runner and expected-failure list | Its memory is one coherent space: `fence_i` and `rvc` pass there and must not be reported as XPASS against the Harvard list |
| D24 | Minimal **U-mode now** (M + U, no S, no delegation): privilege flop, MPP WARL {00, 11}, ecall cause 8, every CSR / mret / wfi illegal in U, M interrupts always enabled in U, `mcounteren` read-only zero, `misa` = 0x4010_2105 | M10 cannot pass without it, and this is the least of M11 that Linux nommu needs. S-mode, delegation and the S CSRs stay in M11 |
| D25 | WFI resumes on `mip & mie`, independent of `mstatus.MIE` | Required by the privileged spec and by Linux's idle loop; the previous gate hung the kernel on its first idle |
| D26 | The compliance env's trap vector accepts the U-mode ecall | Upstream runs `rv32u*` bodies in U; with U implemented ours now do too, so the report ecall is cause 8 |
| D27 | The cache figure is the SoC-shaped wrapper counted with `-nowidelut` | The board top prunes the cache and the default count is unstable (worker A) |
| D28 | Bursts stop at 15 cycles | The remainder is SDRAM protocol; the CoreMark measurement shows the hit path, not the miss, is now what costs |
| D29 | Formatting applied to whole touched files | `top_module.sv` and `sim_top.sv` were already off-format on the branch; formatting only the edited lines is not possible with Verible |

### M10 on the SoC sim: shell reached (boot 3, after D24 + D25)
- Same `~/yarv-linux/soc/Image`, `+NOPROBE`, `bootrom_linux.hex`,
  `UART_BIT_CYCLES=432`, `UART_RX_START=60000000`. `Run /bin/sh as init
  process` at 0.743 s of kernel time (~37 M cycles at 50 MHz); BusyBox hush
  prompt; typed `uname -a`, `ls /`, `echo BOOT-OK` all answered. ~5 min of
  wall time. `/proc` is not mounted under `rdinit=/bin/sh`, so
  `cat /proc/meminfo` failed: expected, mount it first next time.
- **Memory torture test** `sim/sw/soc_torture` (SoC only): 30 000 random
  cached byte/half/word ops over 64 KiB with a shadow, then fence.i +
  CACHE_BYPASS device reads, then bypass writes read back cached. PASS,
  9.5 M cycles. Not yet in the SoC regress and not yet mutation-checked.
- Next steps and owner items: `docs/linux_resume_prompt.md`.

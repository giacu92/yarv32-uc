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

# Simulation (Verilator)

Functional simulation of the full pipeline (fetch + decode + execute + LSU +
Zicsr + trap/interrupt unit). Not part of the synthesis file list.

The **Harvard** build wires the CPU to a native read-only I-mem (`+IINIT`) and
a native byte-strobed D-mem (`+DINIT`); the AXI4-Lite peripheral bus carries
three MMIO slaves behind the parametric 1→N xbar (`axi4_lite_xbar`, windows from
`rv32_pkg`; unmapped → DECERR):

| Slave | Base | Role |
|---|---|---|
| `axi4_lite_uart` | `0x1000_0000` | UART, TX+RX FIFOs, level IRQ → `mip.MEIP` |
| `clint_timer` | `0x1000_1000+` | machine timer interrupt (64-bit mtime/mtimecmp) |
| `msip_peri` | `0x1000_3000` | machine software interrupt (mip.MSIP) |

`sim_top.sv` replicates the board top's wiring so memories can be preloaded
and CPU per-stage taps logged. The UART is driven both ways: `uart_rxd_i` is a
real port (double-flopped as on the board) fed with 8N1 frames by the C++
harness, and every byte the CPU transmits is captured to `sim_uart_tx.txt`.

## Setup

```
sudo apt-get install -y verilator
```

## Build & run

```
cd sim
make run                                                       # Harvard oracle
make run RUN_ARGS="+IINIT=sw/quicksort/build/imem.hex +DINIT=sw/quicksort/build/dmem.hex"   # C program
make sw-run     # from repo root: build C program + run the sim loading it
```

The harness drives clk/rst and prints three aligned logs (each stage lags the
previous by one cycle): a **fetch log** (`cycle / fe_pc / fe_instr / c`), a
**decode log** (`cycle / de_pc / de_instr`), and an **execute retire +
writeback log** (`cycle / ex_pc / ex_instr`, plus `wb x<n> = 0x...` when a
register is written). The writeback log makes the execute→decode forward path
and the load-use path verifiable.

**These three transcripts print only for the Harvard oracle** — the run with
no `+IINIT`, i.e. plain `make run`. That program is 36 instructions long and
exists to be read cycle by cycle. A firmware run under `sim/sw/` is millions
of cycles, where the transcripts cost a string per stage per cycle and produce
an output nobody scrolls: at 500k cycles, quicksort emits 803k lines / 26 MB
and peaks at 75 MB resident, against 32 lines / 4 KB and 4.5 MB with them off
(and runs ~14% slower). `INSTR_LOG=1` forces them back on for a firmware run
when something needs reading instruction by instruction; `INSTR_LOG=0` turns
them off for the oracle. The counters under each section (`fetched` /
`decoded` / `retired`), the stall histogram, the CPI decomposition and the
`RTL_TRACE` co-sim log are unaffected either way.

An exit summary prints `retired N instructions in M cycles`, `IPC = N/M`, and
a `stalled K/M cycles (P%)` breakdown (DIV/REM hold, LSU `EX_MEM_WAIT`,
legacy RAW cost — now zero). Every run also checks the WFI-halt invariant
(`WFI-halt check: OK`, or `WFI-HALT FAIL` with nonzero exit). The run stops
early on park detection (8 identical retires); `MAX_CYC` (default 4000) bounds
programs that never park, e.g. `MAX_CYC=20000 make run`.

A VCD waveform `sim_top.vcd` is written; `make wave` opens it in GTKWave. The
trace covers the full hierarchy, so decode internals (`u_decode.opcode`,
`u_alu.alu_op_i`, `imm_*`, ...) are named even though they are not CPU ports.
`gtkwave_alu_op.txt` and `gtkwave_opcode.txt` translate those signals to
mnemonics (Data Format → Translate Filter File).

## Program images

The Harvard build loads two `$readmemh` preloads (one 32-bit hex word per
line): `sim/imem.hex` (I-mem, read-only) and `sim/dmem.hex` (D-mem, empty
placeholder for the oracle). `+IINIT=<path>` / `+DINIT=<path>` override the
defaults so a C-compiled pair loads without clobbering the oracle. Build C →
images via `sim/sw/` (see `sim/sw/README.md`).

## UART console I/O

The harness types into the UART and records what comes out, so a serial
program can be exercised end to end without hardware.

```
cd sim
UART_RX='2000\r' make run RUN_ARGS="+IINIT=sw/yarvmon/build/imem.hex +DINIT=sw/yarvmon/build/dmem.hex"
cat sim_uart_tx.txt          # everything the CPU transmitted
```

Environment knobs:

- `UART_RX="..."` — string to type as real 8N1 frames on `uart_rxd_i`. C
  escapes decoded (`\r` = Enter). Unset = idle line.
- `UART_RX_PACED=0` — ship frames back-to-back instead of waiting for RX FIFO
  room. Provokes an overrun; keep in any UART regression (the paced default
  cannot).
- `UART_BIT_CYCLES=<n>` — clocks/bit in the driver. 434 for the 50 MHz board
  build, 217 for the 25 MHz PLL-bypass one.
- `NO_VCD=1` — skip the waveform dump (a board-accurate run is millions of
  cycles = multi-GB VCD).
- `INSTR_LOG=1` / `INSTR_LOG=0` — force the per-cycle fetch / decode /
  execute+writeback transcripts on or off. Default: on for the Harvard oracle
  (no `+IINIT`), off for every firmware run.

`sim_top`'s UART clock/baud are parameters, so the sim can run fast (default
5 clocks/bit) or with the board's real divisor to check the RX sampling phase:

```
rm -rf obj_dir
make VPARAMS="-GUART_CLK_HZ=50000000 -GUART_BAUD=115200"
NO_VCD=1 UART_BIT_CYCLES=434 UART_RX='2000\r' MAX_CYC=400000 \
  ./obj_dir/Vsim_top +IINIT=sw/yarvmon/build/imem.hex +DINIT=sw/yarvmon/build/dmem.hex
rm -rf obj_dir && make        # back to the fast default
```

A `VPARAMS` change is not tracked by the build dependencies, hence the
explicit `rm -rf obj_dir`.

## Compliance tests (`hw/`)

Independent harnesses (no CPU) that drive a single slave from a C++ BFM.

- **`hw/uart_tb/`** — UART FIFO + IRQ compliance. Queued TX bytes ship in
  order; a write to a full TX FIFO is **held** until room appears; an RX burst
  inside the depth is retained with RX_OVERRUN clear; one frame past the depth
  is dropped and latches RX_OVERRUN while queued bytes survive; reading an
  empty RX FIFO pops nothing; the IRQ is level-sensitive and gated by CTRL.
  → "146 checks, 0 failures".
- **`hw/native_mem_tb/`** — `native_ram` protocol compliance: RVALID held
  until RREADY, byte-strobed partial writes, back-to-back writes,
  single-outstanding, posted-store commit at launch-accept, read-only ignoring
  writes.
- **`hw/ram_tb/`** — `axi4_lite_ram` AXI4-Lite compliance: registered/held
  BVALID + RVALID, AW-first/W-first orderings, byte strobes, back-to-back
  writes, single outstanding.

```
cd hw/uart_tb       && make run   # 146 checks, 0 failures
cd hw/native_mem_tb && make run
cd hw/ram_tb        && make run
```

## Co-sim vs Spike (`cosim/`)

Runs the same C-built ELF on upstream Spike (`--log-commits`, built once via
`build_spike.sh`) and on the Verilator RTL sim, then `cosim_diff.py` diffs
per-retire architectural effects (pc + register write). Spike is the golden
ISA reference; Zilx is already upstream (no patch). Harvard co-sim needs
`.data` at a non-zero VMA (`0x2000` in `sim/sw/common/link.ld`) so Spike's
unified address space holds `.text`@0 and `.data`@0x2000 disjoint; the cosim
`SPIKE_MEM` mirrors the real memory sizes so an access the hardware would
silently alias makes Spike trap instead.

- **`cosim/quicksort/`** — PASS, 29 632 retires matched, then a clean stop at
  the first UART MMIO access (Spike has no UART slave — a harness limit, not a
  CPU bug). Firmware rebuilt `PRINT_ARRAY=0` each run.
- **`cosim/coremark/`** — PASS, 332 803 retires matched. The longest/broadest
  co-sim (linked lists, matrix kernel, state machine over strings). Firmware
  rebuilt `COSIM=1` (no cycle counter — the one register write Spike can't
  reproduce) and `-O2` (Spike is one address space; `.text` must end below the
  `0x2000` `.data` VMA).
- **`cosim/ecall/`** — PASS, 17 retires matched. Spike co-sim of an
  illegal-instruction sync trap (`.word 0x0000007f`); the faulting instruction
  is not retired in either model. `ecall` itself is not Spike-comparable.

**`sw/dhrystone/` has no co-sim, by construction.** Its `Arr_2_Glob` is
`int [50][50]` — 10 000 bytes of `.bss` — so `sw/dhrystone/dhry_link.ld`
takes the whole 16 KiB D-mem with `DMEM ORIGIN = 0`. `.text` has to be at 0
too (the core boots there), and Spike's one address space cannot hold both.
The `0x2000` split above exists precisely to avoid that; Dhrystone is the
one workload that cannot fit inside it.

```
cd cosim/quicksort && make cosim   # PASS -- matched 29293 retires
cd cosim/coremark  && make cosim   # PASS -- matched 306366 retires
cd cosim/ecall     && make cosim   # PASS -- matched 17 retires
```

Each harness saves the sim's stdout to `rtl.stdout` next to `rtl.trace` and
echoes the performance summary (see **Stall analysis** below) before the
diff, so the one long run it already paid for is not wasted. **Read those
figures as belonging to the co-sim build**, which is not the timed build:
quicksort is `PRINT_ARRAY=0` and CoreMark is `COSIM=1 -O2`, so a co-sim CPI
will not match the number the timed build reports.

`build_spike.sh` relocates Spike's fixed debug-module and boot-ROM devices out
of the way of these images and records the patch set in a stamp file so an
older install is rebuilt rather than silently reused. The Spike source tree,
build, install, and per-run logs are gitignored; only the harness is committed.

## Stall analysis

Every run ends with a cause histogram over the cycles that retired nothing.
It exists because the `stalled N/M cycles (X%)` line above it answers a
narrower question than it looks like: `dbg_stall_o` is `dec_stall |
ex_stall`, and a **fetch bubble is neither** — decode holding nothing
because the instruction buffer is empty is a cycle that retires nothing and
is not counted as a stall. So a change that removes fetch bubbles (the
64-bit / 2-outstanding rewrite) lowers the cycle count and *raises* that
percentage at the same time. Never read it as a cross-design or
cross-program comparison.

The histogram partitions the run instead: retire cycles plus the buckets are
the whole run, so the columns sum and IPC falls straight out. Each no-retire
cycle is charged to the first cause that applies, in the order the pipe is
actually blocked — execute owns the retire slot, then decode's own bubbles,
then an empty buffer is the fetch path's fault:

| bucket | signal | meaning |
|---|---|---|
| `load-wait` | `mem_running` | load issued, waiting for `rvalid` |
| `lsu-launch` | `(mem_bus_drive \| peri_capture) & ~store_done` | LSU issuing: D-mem drive, peri capture, or peri drive |
| `lsu-mistrap` | `misaligned_trap` | misaligned access raising its sync trap (`EX_MEM_TRAP`) |
| `div/rem` | `div_running \| alu_start` | multi-cycle divide |
| `csr-read` | `csr_start` | the registered CSR read (`EX_CSR_WAIT`) |
| `wfi-halt` | `wfi_stall` | halted until an interrupt is pending |
| `rvc-hold` | `resource_stall` | compressed upper half held against a fresh word |
| `rvc-span` | `span_wait` | 32-bit instruction straddling a word boundary |
| `redirect` | between `branch_valid_o` and the next retire | flushed D/E + killed buffer + refill |
| `imem-starve` | buffer empty, no redirect | fetch could not keep up |
| `decode-bubble` | `~decoded_valid` | words available, no instruction out |

`redirect` deliberately absorbs the flushed D/E slot and the refill together:
split apart they hide the one number that matters, cycles per taken
branch/trap/`mret`. Event counters (mem-ops, loads, stores, div/rem, CSR
ops, redirects) print underneath, so a bucket can be read per operation —
"load-wait is 20% of the run" is many cheap loads or few expensive ones, and
only the second is a memory-system problem.

### CPI decomposition

The histogram is also printed divided by retires instead of by cycles, and
**that is the form to compare**. A bucket's percentage of the run moves when
any *other* bucket changes; cycles per retired instruction is a
per-instruction cost that stands on its own. The terms are additive by
construction — every cycle is either a retire or exactly one bucket — so 1.0
plus the buckets *is* the CPI, and the 1.0 floor is the retire slot itself
(one instruction per cycle, in order, is all this pipeline can do). A
non-empty bucket below the printed resolution shows as `<0.001`, not
`0.000`, because "present but negligible" is a different claim from "free".

Measured on the three benchmarks (2026-08-27, post-fetch-rewrite):

| cycles per retired instruction | quicksort | CoreMark | Dhrystone |
|---|---|---|---|
| retire (floor) | 1.000 | 1.000 | 1.000 |
| + `redirect` | 0.434 | 0.374 | 0.353 |
| + `lsu-capture` | 0.279 | 0.234 | 0.314 |
| + `lsu-launch` | 0.161 | 0.181 | 0.178 |
| + `div/rem` | — | <0.001 | 0.085 |
| **= CPI** | **1.874** | **1.790** | **1.930** |
| (IPC) | 0.534 | 0.559 | 0.518 |

Each term is a per-operation cost times how often the operation occurs, and
both halves are needed to read it — 3 cycles per redirect is cheap if
branches are rare and is the whole problem at one taken branch in seven:

| | quicksort | CoreMark | Dhrystone |
|---|---|---|---|
| cyc / redirect | 3.10 | 3.20 | 3.16 |
| 1 redirect per N instr | 7.15 | 8.56 | 8.96 |
| no-retire cyc / mem-op | 1.58 | 1.78 | 1.57 |
| mem-ops as % of instr | 27.9% | 23.4% | 31.4% |
| cyc / divide | — | 33.0 | 33.0 |
| `imem-starve`, whole run | 2 cyc | 2 cyc | 2 cyc |

### With the branch predictor (`BP_EN=1`)

The table above is the pre-predictor core (`BP_EN=0`). The branch predictor
(gshare PHT + GHR + RAS, prediction-at-decode, 2026-08-28) is gated by the
sim parameter `BP_EN` (default 1) — toggle it with a clean rebuild:

```
rm -rf obj_dir && make VPARAMS="-GBP_EN=0"     # predictor off (A/B baseline)
rm -rf obj_dir && make                          # predictor on (default)
```

Two more structural knobs landed 2026-09-01, both there so a PnR run can move
one variable at a time. Both now DEFAULT TO 0, i.e. to the form that ships —
`rv32_pkg` has always had them at 0, but the module parameters themselves
defaulted to 1 until 2026-09-02, so a standalone instantiation silently picked
up an unshipped configuration. The alternative form is the one you have to ask
for:

```
rm -rf obj_dir && make VPARAMS="-GMUL_SHARED_DSP=1"   # one shared 33x33 product
rm -rf obj_dir && make VPARAMS="-GBP_PUSH_LOOKUP=1"   # PHT read at buffer-push time
```

`MUL_SHARED_DSP` is behaviour-neutral, so the retire stream and the cycle
count must not move — `sw/isa/mul_ops` and `sw/isa/div_ops` check it both
ways. It defaults to 0 because 1 measured **-2.52 ns** (49.6 -> 44.1 MHz) in
PnR; see the MUL block in `alu.sv`.
`BP_PUSH_LOOKUP=1` reads the PHT when fetch pushes a word into the buffer and
carries the direction bit in the entry, which takes the array read off the
decode -> fetch redirect path and makes `BP_PHT_DEPTH` timing-neutral. It DOES
move the prediction (the bit is read with a slightly older GHR), so cycle
counts shift a little; the retire stream cannot move, and the oracles check
that. Measured, same images:

| config (`BP_PHT_DEPTH` x `BP_GHR_W`) | quicksort | CoreMark cyc/iter | Dhrystone cyc/iter | cm accuracy | cm MPKI |
|---|---|---|---|---|---|
| decode lookup, 128x7 | 40154 | 401976 | 486 | 88.24% | 23.79 |
| push lookup, 128x7 (default) | 40135 | 402827 | 485 | 87.84% | 24.59 |
| push lookup, 512x9 | 40057 | **398441** | 485 | 91.57% | 17.07 |
| decode lookup, 512x9 | 40067 | 398093 | 485 | 91.89% | 16.41 |

The stale GHR costs ~0.2% of CoreMark cycles at 128 and *gains* 0.2% on
Dhrystone; the win is that 512 entries become affordable, which is worth
0.88% of CoreMark cycles over the 128-entry baseline.

With the predictor on, the **`redirect` bucket drops ~5×** (a correct
prediction issues no execute redirect) but a new **`imem-starve` + `other`
~0.23 CPI** appears — the kill+refill bubble on every *correct* predicted-taken
branch, charged to `imem-starve` (no mispredict fires) instead of `redirect`.
That refill bubble is the predictor's own open item. CoreMark:

| cycles per retired instruction | CoreMark `BP=1` | CoreMark `BP=0` |
|---|---|---|
| retire (floor) | 1.000 | 1.000 |
| + `lsu-launch` | 0.186 | 0.171 |
| + `lsu-capture` | 0.238 | 0.221 |
| + `redirect` | 0.069 | 0.383 |
| + `imem-starve` | 0.115 | <0.001 |
| + `other` | 0.115 | <0.001 |
| + `decode-bubble` | 0.008 | — |
| **= CPI** | **1.711** | **1.775** |
| (IPC) | 0.585 | 0.563 |

The LSU and div buckets are unchanged (memory cost is prediction-independent).
A `branch predictor:` line prints underneath the histogram: resolved /
predicted-taken / mispredicts / accuracy / MPKI, plus a `RAS:` hit/miss line.
The full A/B (quicksort/CoreMark/Dhrystone, `BP=1` vs `BP=0`) and the
copy-paste reproduce recipe are in [`bench_ipc_ab.md`](bench_ipc_ab.md).

Two identities hold in every run, and they are the instrumentation's own
proof that it is neither losing nor double-counting cycles: `lsu-launch` ==
loads (a store both issues and retires in `EX_IDLE`, so its cycle is a retire
and never enters the histogram), and `div/rem` == 33 × divides (the
32-iteration restoring FSM plus one). `lsu-mistrap` is 0 in every benchmark —
a misaligned access always traps, so it only appears in the trap oracles.

Before 2026-09-01 there was a third identity, `lsu-capture` == mem-ops, from
the LSU capture stage that used to sit in front of the launch. The bucket is
gone with the stage; the tables above that still name it are the historical
measurements listed with their dates.

Dhrystone has the worst CPI of the three not because the core does worse on
it but because it has the highest load/store density (31.4% — `strcpy` and
`strcmp` a byte at a time) plus the only divides. quicksort has the most
expensive redirect term because it is the branchiest.

### After removing the D-mem LSU capture stage (2026-09-01, shipping)

The `lsu-capture` bucket was one cycle on every memory access — the price of a
launch path that started at a flop instead of at the end of
`regfile -> forward -> ALU`. Removing it on the **D-mem** path (the bus is
driven combinationally from `alu_result` in `EX_IDLE` again; only the
*later-cycle* consumers, the load byte select and a misaligned trap's `mtval`,
still read a latched copy of the address) deletes that cycle outright.

Two things keep a capture cycle, and neither by choice:

- the **peri** path, because behind that port sit the AXI bridge, the crossbar
  and a slave's own address decode, all combinational in the address phase.
  Removing it too failed PnR at -5.533 ns / 39.164 MHz. Costs nothing
  measurable: MMIO is outside every timed region.
- **stores**, because a posted store retires on its own launch, so "did this
  store launch" is a question `stall_o` must answer in the same cycle -- and
  answering it needs the effective address. Driving that from `alu_result` put
  the machine's deepest combinational value on the pipeline's control network
  and failed PnR at -4.956 ns / 40.1 MHz.

A **load** never retires on its launch (it waits for `rvalid`), so nothing about
its launch reaches `stall_o`, and that is the one case that stays live. It is
78% of the win: CoreMark has 62778 loads against 17815 stores.

Same tree, same firmware images, one parameter (`LSU_LIVE_LOAD`):

| | quicksort | CoreMark | Dhrystone |
|---|---|---|---|
| cycles, `LSU_LIVE_LOAD=0` | 49605 | 474767 / iter | 600 / iter |
| cycles, `LSU_LIVE_LOAD=1` | **44833** | **420560 / iter** | **537 / iter** |
| CPI, =0 | 1.691 | 1.622 | 1.735 |
| CPI, =1 | **1.528** | **1.434** | **1.569** |
| score, =0 | — | 2.10 CM/MHz | 0.94 DMIPS/MHz |
| score, =1 | — | **2.37 CM/MHz** | **1.05 DMIPS/MHz** |

−9.6% / −11.4% / −10.5% cycles at an unchanged 50 MHz. Both columns close
timing; `=1` is what ships and is board-verified.

quicksort is built `PRINT_ARRAY=0`, so it retires exactly 29336 instructions
on both sides and the cycle delta is the whole effect. CoreMark and Dhrystone
retire slightly *more* instructions with the live load, because their UART
`TX_READY` poll loops spin more times per byte when the core runs the loop
faster — a benchmark artefact, not a workload difference, and the reason cycles
per iteration is the number to compare rather than IPC.

A D-mem load now costs 1 no-retire cycle (its issue) instead of 2. Stores and
peri accesses still cost their capture cycle, folded into `lsu-launch`.
`load-wait` stays 0: the BSRAM answers the cycle after the accept and that
cycle retires.

**`lsu-launch` 0.252 CPI is now a floor, not a lever.** A load costs its issue
cycle plus the memory's one-cycle response, and the response cycle retires; the
issue cycle cannot be removed by shortening a path. It goes away only with
non-blocking loads — a second outstanding access, so the next instruction
issues while the first load's data is in flight.

Two things to read off it. **The fetch path is no longer a limiter** —
`imem-starve` is 2 cycles in a 1.5 M-cycle run, so outside a redirect the
buffer is never empty; that is the fetch rewrite having worked, and it is
also why the `dbg_stall_o` percentage went up. **The redirect is now the
biggest single cost**: ~3.1 cycles per taken redirect, 38-50% of all
no-retire cycles, which is exactly the `regfile -> forward mux -> branch
compare/target -> PC redirect` path CLAUDE.md names as the next limiter.
After it comes the LSU at a flat 2 cycles per load and 1 per store (capture
+ launch) — the cost the LSU register stage bought +4.8 ns of timing with.
`load-wait` is 0 because the BSRAM answers the cycle after the accept, and
that cycle retires.

## Firmware oracles (`sw/`)

Each oracle writes `0x600D`/`0xBAD` to a known D-mem word unless noted. Build
in its directory, then run from `sim/`:

```
cd sw/<group>/<name> && make
cd .. && make run RUN_ARGS="+IINIT=sw/<group>/<name>/build/imem.hex +DINIT=sw/<group>/<name>/build/dmem.hex"
```

### `make regress` — all of them at once

```
make sw-all            # build every program and oracle (from the repo root)
make regress           # run them all, one line per test, non-zero exit on any failure
make regress VPARAMS="-GLSU_LIVE_LOAD=0"    # ... against an A/B configuration
```

```
TEST            RESULT DETAIL
quicksort       PASS   44833 cyc
div_ops         PASS   3554 cyc
...
uart_echo       PASS   29150 cyc, TX ends GOOD
harvard_oracle  PASS   parks clean

13 passed, 0 failed, 0 skipped
```

Three kinds of check, because the tests really do report differently, and the
distinction matters:

- **`probe@<addr>=<value>`** — the marker word is read back out of the RTL's
  D-mem array (`PROBE=<byte addr>` on the sim binary prints it; word index =
  addr/4). This is the *exact* result. **Do not substitute a grep of the retire
  trace**: the `li` that materialises `0x600D` retires whether or not the store
  after it ever ran, so a trace grep passes a test that died between the two.
- **`trace:<reg>=<value>`** — the last writeback of that register. Only
  quicksort, whose `main()` *returns* `0x600D` in `a0` and never stores it
  (`start.S` just parks). Its real verification is `make cosim`, which compares
  all 29293 retires against Spike; `regress` is a smoke check and a cycle
  number.
- **`park`** — clean park plus the WFI-halt invariant, nothing else. The
  hand-crafted Harvard oracle has no marker at all, and `isa_probe` reports
  through UART strings meant for a human.

Every test also fails on a `WFI-HALT FAIL` or a missing clean park, whatever
its marker says. Missing images are `SKIP`, not `FAIL`, so a partly built tree
still gives a useful run. The list lives in `REGRESS_TESTS` in `sim/Makefile`.

`regress` was checked against a deliberately broken build: with the
pre-2026-09-02 divider restored it reports `div_ops FAIL marker=0xbad` and
exits non-zero. A test harness that has never failed has not been tested.

- **`sw/peri/uart_echo/`** — the only end-to-end test of MEIP
  (`uart_irq_o`→`meip_i`→`mip.MEIP`→`trap_unit` + `wfi` wake). Echoes 4 bytes
  by polling, then 4 more from a machine-interrupt handler. Result @0x3000.
  Doubles as the board bring-up program.
  `UART_RX='abcdefgh' make run RUN_ARGS=...` → "ECHO / abcd / IRQ / efgh / GOOD".
- **`sw/isa/ifault/`** — jumps to 0x100000 (outside the 16 KiB I-mem), checks
  one trap with `mcause=1`, `mtval` = jumped-to address. Handler rewrites
  `mepc` (the address is still unfetchable).
- **`sw/isa/isa_probe/`** — board bring-up probe that reports through fixed
  strings and a binary bit-dump built only from `AND` + mask doubling, never
  the hex printer or any instruction under test. Covers `c.andi`, shifts,
  `AND`/`OR`, byte loads at each lane, `lw`/`lbu`/`lhu`, byte stores. The
  constant-vs-computed load split is what isolated the silicon load
  byte-select bug. `make UART_TX_PACED=1`.
- **`sw/isa/mul_ops/`** — M-extension multiply oracle: `MUL` / `MULH` /
  `MULHSU` / `MULHU` against hand-computed 64-bit references, including
  `INT_MIN` squared, `INT_MIN * -1`, and an operand-swapped pair that pins
  `MULHSU`'s asymmetry. Written because nothing else in the tree emitted the
  three high-word forms at all — gcc only reaches for them on 64-bit
  arithmetic, which no benchmark does — so the whole signed/unsigned half of
  the multiplier was unguarded. Run it at `MUL_SHARED_DSP` 0 and 1.
- **`sw/isa/div_ops/`** — M-extension divide oracle (2026-09-02): `DIV` /
  `DIVU` / `REM` / `REMU`, 22 operand pairs run through all four forms, plus
  `rd == rs1` / `rd == rs2`, distance-1 forwarding into and out of the
  divider, back-to-back divides, and a branch on a quotient. Written to pin a
  real bug: the divider stores `abs(rs1)` and re-applied the sign only on the
  normal path, so `REM rd, rs1, 0` returned `abs(rs1)` where RISC-V requires
  `rs1` unchanged — `rem -5, 0` answered `+5`. Nothing reached it before,
  because the benchmarks emit only ordinary positive divisions: never a
  division by zero, never a signed overflow, never an unsigned divisor above
  2^31. That last shape matters on its own — the divider's partial remainder
  is 32 bits, and it is sound only because the bound comes from the 32-bit
  dividend and the 32-iteration count together, not from the divisor.
- **`sw/isa/rvc_scramble/`** — per-scramble-bit RVC decode oracle for
  `c_expand()`: every RVC immediate form with each bit set in isolation, a
  `mtvec` handler turning a mis-decoded jump into FAIL.
- **`sw/isa/span_target/`** — same-cycle target-span stitch oracle: branches
  to a 32-bit instr at offset 2/6 of a fetch word and self-checks; a `mtvec`
  handler catches a mis-stitch as FAIL. Guards the zero-bubble branch-target
  stitch path no cosim/oracle exercised.
- **`sw/isa/bp_pred/`** — branch-predictor correctness oracle (2026-08-28):
  9 cases (always-taken / always-NT / loop-exit mispredict / alternating /
  correlated / RVC `c.bnez` / nested JAL+JALR call-return for RAS /
  mispredict-with-2-reads-outstanding / ecall-trap-vs-predicted-branch
  priority), each self-checking a known result. Guards the invariant a
  predictor must never alter — a prediction may mispredict freely but must
  never change architectural state; a flush/hold-buffer bug that lets a
  wrong-path instr retire fails a `CHECK`. A `mtvec` handler resumes the one
  expected ecall and fails any stray trap. Pass = park @ `park`, probe
  0x800 = 0x600D. Caught the hold-buffer-survives-predicted-redirect bug.
- **`sw/intr/trap/`** — standalone M-mode: ecall / load-misaligned / illegal /
  MSIP+WFI, self-checking @0x2000.
- **`sw/intr/timer/`** — arms `mtimecmp=200`, `wfi`s, wakes on MTIP, handler
  stores marker=7 @0x2040 and clears MTIP.
- **`sw/intr/wfi_trap/`** — regression for a WFI-halt deadlock: a faulting
  instruction behind `wfi` must not freeze the pipe. Toolchain-free via
  `python3 gen_hex.py`.

`UART_TX_PACED=1` replaces the `TX_READY` poll with a delay longer than a
frame, so only one byte is in flight. Diagnostic only — separates "the FIFO
filled" from "the poll never returned" when a board goes quiet mid-line.

## Files

- `sim_top.sv` — sim wrapper (CPU + native I/D-mem + peri MMIO slaves + VCD
  data-RAM window). RX pin double-flopped; TX monitor writes
  `sim_uart_tx.txt`.
- `sim_main.cpp` — Verilator harness (clk/rst, trace, three logs — oracle
  only, see `INSTR_LOG` — stall breakdown, WFI-halt check, park/`MAX_CYC`
  stop, UART RX frame driver).
- `imem.hex`/`dmem.hex` — Harvard oracle preload.
- `Makefile` — build/run rules (`RUN_ARGS` forwards plusargs).
- `hw/{native_mem_tb,ram_tb,uart_tb}/` — compliance tests.
- `cosim/` — shared co-sim assets (`cosim_diff.py`, `build_spike.sh`) +
  `quicksort/`, `coremark/`, `ecall/` harnesses.
- `sw/` — C → image flow (see `sw/README.md`) and the oracles above.

Build artefacts (`obj_dir/`, `build/`, `*.log`, `*.vcd`,
`sim_uart_tx.txt`) are gitignored.
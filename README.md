# YARV32-uC: Yet Another RISC-V uController

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/giacu92/yarv-uc)

An **RV32IMAC + Zicsr + Zifencei** soft-processor core for a **Gowin
GW2AR-18C** FPGA (QFN88) on a Tang Nano 20k. Hobby/learning project, built
with Claude Code assistance. Sim-verified (Verilator + Spike co-sim) and
**running on silicon**.

## Architecture

In-order **3-stage pipeline — Fetch / Decode / Execute** over a **Harvard**
memory system: a read-only I-mem for fetch, a byte-strobed D-mem for the LSU,
and AXI4-Lite kept only for peripherals. Both memories are 16 KiB
(`ADDR_W=14`, 8 BSRAM blocks each, 16 of the device's 46). The package also
carries 8 MiB of SDRAM on a separate die, currently unused.

### Fetch

64-bit, 2-outstanding reads over a native read-only I-mem port, feeding a
depth-8 instruction buffer of 32-bit words. One 8-byte access delivers two
words, split into PC-stamped buffer entries; two outstanding reads keep the
BSRAM issuing through decode stalls (DIV/REM, memory wait) that would idle a
single-outstanding port. The buffer head presents to decode exactly what the
old single-word F/D register did, so decode is agnostic to the width behind
it. A second read port exposes head+1 so decode can stitch a 32-bit
instruction sitting at a 2-byte-aligned branch target without a bubble.

A redirect kills the buffer and issues the first read at the target **in the
same cycle**, rather than waiting for the PC register to update. Reads already
in flight belong to the killed path; because responses come back in order, they
are discarded by counting them rather than by blocking the issue path, so the
new path starts fetching immediately. The decode-time **prediction** launches
this way — its target is PC- or flop-derived. Execute redirects (mispredict,
trap, `mret`) launch the cycle after (`EXEC_REDIR_INCYCLE=0`): their target
comes off the forwarding path, and putting the register file's read output on
the instruction memory's address pins — which pay a long physical tail — cost
more timing than the cycle bought.

A PC outside the implemented I-mem cannot be fetched: the memory decodes only
`ADDR_W` bits, so the read would alias back into real instructions and execute
them. Fetch raises an instruction access fault instead. On an ordinary cycle
the check is on the request and blocks it, reading a flopped PC. The redirect
cycle is the exception: checking the target there would hang an 18-input
OR-reduce off the predictor's target adder, so the read is issued regardless
and a second check on the response — where the address is a register and costs
no timing — discards the aliased data and pushes a fault entry stamped with
the exact faulting PC.

### Decode

Expand-then-decode: RVC expands to its 32-bit equivalent first, then one
uniform decoder handles RV32I + M + C + Zilx + Zicsr. A hold buffer carries
the upper compressed half of a word across cycles.

Hazards resolve by **execute→decode forwarding**: when execute retires a
writeback to a register the decoding instruction reads, the value is injected
in place of the stale register-file read. Distance-1 RAW — ALU, div-then-use,
branch-on-dependent, load-use — costs zero bubbles. Distance 2 and beyond
needs nothing: the asynchronous register read already sees the committed
value.

That forward path imposes a rule worth knowing before touching execute: a
combinational value derived from the D/E operands may only be consumed in the
cycle it is produced. Operands can arrive from the bypass, so such a value
sits at the end of the critical path — simulation settles it regardless of
depth, silicon does not.

### Execute + LSU

RV32I ALU, single-cycle MUL on a DSP, multi-cycle DIV/REM (32-iteration
restoring FSM), Zilx effective-address, branch resolve with redirect, and CSR
read-modify-write. One FSM drives DIV/REM and the LSU.

**Loads launch live**: an aligned D-mem load drives the bus straight off the
ALU result in the idle state, so it costs one no-retire cycle instead of two —
only the BSRAM's one-cycle response waits. Stores and peripheral accesses
keep their register stage (`LSU_LIVE_LOAD` gates the split): a posted store
retires on its own launch, so answering "did this store launch" in the same
cycle would put the ALU result on the pipeline's control network (PnR:
-4.956 ns), and behind the peri port sit the AXI bridge, the crossbar and a
slave's address decode, all combinational in the address phase (PnR:
-5.533 ns). MMIO sits outside every timed region, so the peri capture cycle
costs nothing measurable (8/4/0 cycles on quicksort/CoreMark/Dhrystone).

Stores are posted (retire on launch accept); loads wait for the read
response. The LSU steers `addr[PERI_ADDR_BIT]` internally: `0` → native
D-mem, `1` → AXI4-Lite peripheral bridge. Misaligned accesses trap and are
never launched.

### Branch predictor

gshare 2-bit PHT (128 entries) + 7-bit GHR + 8-entry RAS, **predicting at
decode**, with execute as the golden resolver.

Lookup depends on the PC and the global history only, never on register data,
so it stays off the `register file → forward → branch` critical path. JAL and
conditional-branch targets are computed directly as `pc + imm` — no BTB, and
therefore no cold-start mispredicts on direct jumps; conditional direction
comes from the PHT; JALR returns come from the RAS. Other indirect jumps are
left unpredicted.

A predicted-taken instruction redirects fetch one cycle before execute would.
Execute still computes the real outcome and compares: a correct prediction
issues no redirect at all — that is the win — and a mispredict reuses the
existing flush path. Training fires at resolve only, so squashed wrong-path
instructions never contaminate the tables. `BP_EN` (default 1) is the A/B
knob and the fallback.

PHT depth is a **timing** parameter here, not an accuracy one. The table is
read combinationally at decode and that read feeds fetch's launch logic in the
same cycle, so a bigger table costs slack directly: 512 entries cost about
1 ns to buy 0.65% of cycles.

### CSRs, traps and interrupts (M-mode)

Machine-mode Zicsr subset — mstatus / misa / mie / mtvec / mscratch / mepc /
mcause / mtval / mip, plus `mcycle` and `minstret`. CSRRW/S/C retire with
`rd ← old CSR`; field semantics are enforced (MPP forced to 11, mtvec MODE
masked). The read is **registered** — decode presents the address, data lands
the next cycle — which took the old asynchronous read off the critical path;
execute holds a CSR op one extra cycle at measured zero cost, because it fits
inside the existing fetch bubble.

Precise synchronous traps at commit: instruction access fault (1), illegal
instruction (2), ebreak (3), load/store misaligned (4/6), ecall-M (11).
`mret` returns; `wfi` halts until an enabled interrupt is pending;
`fence`/`fence.i` are nops (Harvard has no D→I write path). `mtvec` supports
direct and vectored mode. A trapping instruction is not retired.

Three interrupt sources: **MSIP** (`msip_peri` MMIO @0x1000_3000), **MTIP**
(`clint_timer` @0x1000_1000+, 64-bit mtime/mtimecmp), **MEIP** (UART level
IRQ). Priority MEI > MSI > MTI. A dedicated trap-write port updates
mepc/mcause/mtval/mstatus atomically on entry.

### Integration

The CPU exposes three ports: native `imem` (read-only), native `dmem`
(byte-strobed), and an AXI4-Lite master for peripherals. Native→AXI
conversion lives inside the CPU, so the board top is pure point-to-point
wiring. Peripherals sit behind a 1→3 crossbar with a DECERR terminator for
unmapped addresses — UART (TX+RX FIFOs, level IRQ), machine timer, and the
software-interrupt register.

### Clocking

A 25 MHz MS5351M reference feeds an on-chip rPLL that drives the fabric at
**50 MHz** (`clk_core = 25 × 10/5`), single clock domain, no CDC. Synthesis
and PnR meet 50 MHz with the branch predictor enabled, most recently by
+0.024 ns — a pass rather than a margin, on the noise figure given below. A 25 MHz
PLL-bypass build exists as a fallback and as a diagnostic that removes the
rPLL from the picture.

Timing on this device is tight enough that several structural choices exist
only to serve it — the LSU register stage, the registered CSR read, the PHT
depth, and which side of the bus the fetch range check sits on. Run-to-run
placement noise is about 0.7 ns on the same path, comparable to the margin
itself, so a single PnR run is not evidence about an RTL change.

## Performance

At 50 MHz:

| Benchmark | Result |
|---|---|
| CoreMark | **2.39 CoreMark/MHz** (417 809 cycles/iteration) |
| Dhrystone | **1.04 DMIPS/MHz** (543 cycles/iteration) |
| Quicksort (256 words) | 44 833 cycles (print-free build, `PRINT_ARRAY=0`) |

The CoreMark figure is a rules-valid run **on the board**: 2000 iterations,
16.71 s, `crcfinal` 0x4983 matching the published value, sources verbatim from
upstream EEMBC, built `-O3 -march=rv32imac_zicsr_zifencei -mabi=ilp32
-ffunction-sections -fdata-sections -mstrict-align -mbranch-cost=10
-ffp-contract=off -mno-fdiv -Wl,--gc-sections` with **GCC 13.3.0**, the best of
a 2026-09-02 sweep — GCC 16.1.0 -O3 measures 420 897 cycles/iteration (2.37),
the -O2 builds 437 616 / 439 457 (GCC 14.3.0 / 13.3.0, 2.28 / 2.27), and the
GCC 14.3.0 -O3 default 420 560 (2.37). Dhrystone uses verbatim SiFive sources,
200 000 runs in 2.17 s = 92 081 Dhrystones/s (52.40 DMIPS). Quicksort
co-simulates against Spike retire for retire; CoreMark does too, but only in
a dedicated build with the banner and cycle counter compiled out, since Spike
has no UART, timer or MSIP device to diff against.

Five changes account for the current numbers over the pre-predictor core: the
branch predictor (a correct prediction issues no redirect), linker relaxation
(turning `auipc`+`jalr` call pairs back into predictable `jal`), issuing the
fetch at the redirect target in the redirect cycle itself (on the
decode-prediction path — one cycle off every predicted-taken branch), removing
the LSU capture stage from the D-mem load path (one cycle off every load), and
ordering the ALU result mux by measured arrival time.

CoreMark runs at 1.43 cycles per instruction (CPI 1.434 on the
default-toolchain build: floor 1.000 + load launch 0.252 + redirect 0.061 +
decode-bubble 0.010 + other 0.110). The load launch is a floor, not a lever —
it is the load's issue cycle plus the BSRAM's one-cycle response, and the
response cycle retires — so removing it needs non-blocking loads, not a
shorter path. The rest of the no-retire budget goes to the two cycles a
predicted-taken branch still pays for memory latency and for the fetched word
becoming visible. A BTB is deliberately *not* on the list: with linker
relaxation on, Dhrystone has one static `jalr` site and CoreMark two, so it
would buy nothing here.

Methodology, the stall/CPI instrumentation and the A/B recipes live in
[`sim/README.md`](sim/README.md) and `sim/bench_ipc_ab.md`.

## Repository layout

```
src/rtl/pkg/   rv32_pkg.sv          types, opcodes, de_t D/E control struct
src/rtl/core/  pipeline stages + CPU top + reg file + ALU + trap unit + board top
src/rtl/bus/   AXI4-Lite interface + master bridge + peripheral crossbar
src/rtl/utils/ native_ram (Harvard I/D-mem), msip_peri, clint_timer,
               axi4_lite_uart, axi4_lite_xbar (parametric 1→N)
src/phys/      pin assignment (.cst) + timing constraints (.sdc)
impl/          Gowin EDA project + synthesis/PnR Tcl + reports
sim/           Verilator sim, compliance tests, Spike co-sim, firmware oracles
verible.flags  SystemVerilog formatting policy
CLAUDE.md      detailed architecture + build guidance (authoritative)
```

## Quick start

Simulate (Verilator, local):

```
sudo apt-get install -y verilator
make run        # hand-crafted Harvard oracle
make sw-run     # build the C program + run the sim loading it
```

Synthesize and place & route (Gowin EDA, on the build host):

```
QT_QPA_PLATFORM=offscreen QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1 \
  gw_sh impl/synth_check.tcl
QT_QPA_PLATFORM=offscreen QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1 \
  gw_sh impl/pnr_check.tcl
```

See `CLAUDE.md` for the remote-build workflow and the Gowin CLI quirks.

## Roadmap

Done: Harvard split, LSU + forwarding, Zicsr, M-mode traps with all three
interrupt sources, UART with FIFOs, silicon bring-up, CoreMark, 64-bit
2-outstanding fetch with instruction buffer, branch predictor, and 50 MHz
closure with the predictor enabled.

Next, in order:

1. **Hide the rest of the predicted-taken refill bubble** — issuing at the
   target in the redirect cycle removed one of its three cycles; the other two
   are memory latency and the fetched word becoming visible. Fetching into the
   predicted path without killing the buffer at all would remove them.
2. **Cache over the in-package 8 MiB SDRAM** — write-back set-associative
   I/D cache behind the native interfaces. Buys capacity (programs above
   16 KiB), not speed: BSRAM already answers in one cycle at a 100% hit rate.
3. **GPIO** — direction/output/input registers plus interrupt.
4. **PLIC-style interrupt controller** — MEIP is one ORed level with no cause
   register, so an ISR must poll once there is more than one external source.
5. **Illegal-CSR-access trap** — unimplemented CSRs read 0 / ignore writes.
6. **Vectored-mode interrupt co-sim** — only direct mode is co-simulated.
7. **RVC sequential spanning bubble** — the branch-target case is already
   zero-bubble; the fall-through case still costs one cycle and needs a wider
   F/D or dual-issue.

Deferred by choice: S/U mode with delegation, PMP, cross-word sub-word
accesses.

## Known limitations

- Machine mode only (no S/U, no medeleg/mideleg, no PMP).
- MEIP is a single ORed level — no PLIC, so an ISR polls for the source.
- Unimplemented CSR addresses silently read 0 / ignore writes.
- `fence.i` is a nop; no self-modifying code.
- Forward path is distance-1 only (correct: in-order, at most one writeback
  per cycle).
- Programs are capped by the 16 KiB I-mem / 16 KiB D-mem until the SDRAM
  cache lands.

## Formatting & docs

SystemVerilog is formatted with Verible (`make format` / `format-check` /
`format-diff`) per `verible.flags`. `CLAUDE.md` is the authoritative
architecture and build reference — read it for anything beyond this overview.

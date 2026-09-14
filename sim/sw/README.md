# Firmware tree (sim/sw, Harvard)

Compiles bare-metal RISC-V programs into the pair of `$readmemh` word files
the Harvard sim loads — `build/imem.hex` (code → I-mem) and `build/dmem.hex`
(data → D-mem) — so the sim runs real code instead of the hand-crafted
`sim/imem.hex`/`sim/dmem.hex` oracle.

## Layout

```
sim/sw/
  Makefile          aggregator: build every program + every test
  common/           shared assets + build logic (every harness pulls from here)
    sw_build.mk      parameterised build include (toolchain, flags, rules)
    uart.h           UART MMIO register definitions
    ee_printf.c      minimal integer printf over the UART (coremark, dhrystone)
    start.S          freestanding _start: zero .bss, set sp, call main
    link.ld          Harvard link script (.text->IMEM 0, .data->DMEM 0x2000)
    bin2hex.py       .bin -> $readmemh word file
  quicksort/        benchmark program (main.c)
  coremark/         EEMBC CoreMark (eembc/ upstream + local port layer)
  dhrystone/        Dhrystone 2.1 (sifive/ upstream + local port layer)
  yarvmon/          YarvMon serial monitor (board's default product firmware)
  guitar_tuner/     guitar tuner: SPI ADC -> FFT coprocessor -> SSD1306 gauge
                    (builds twice: build/ for the board, build-sim/ for the
                     self-check that needs neither device)
  isa/              ISA oracles: ifault, isa_probe, rvc_scramble,
                    span_target, bp_pred
  intr/             trap + interrupt oracles: trap, timer, wfi_trap
  peri/             peripheral oracle: uart_echo
```

`make -C sim/sw` builds everything; `make -C sim/sw/isa` (or `intr`/`peri`)
one group; `make -C sim/sw/quicksort` one program. Each leaf Makefile is ~15
lines: set a few variables and `include` `common/sw_build.mk`, so the
toolchain, flags and rules live in one place.

`yarvmon/` is the board's default product firmware as well as a sim program.
It is the only harness that sets `IMEM_PAD_WORDS`, because a board image must
make GowinSynthesis size the inferred ROM at the full declared depth. Point
`top_module.sv`'s `INIT_FILE` at `sim/sw/yarvmon/build/imem.hex` to ship it
(it currently loads the CoreMark images).

## Toolchain

Default: buildroot glibc `riscv32-buildroot-linux-gnu` (gcc 14.3.0) at
`~/_toolchains/riscv32-zilx/out`. Override
`RISCV_PREFIX` for another rv32 toolchain (`riscv64-unknown-elf`,
`riscv-none-elf`, ...):

```
make RISCV_PREFIX=/opt/riscv/bin/riscv64-unknown-elf
```

A **linux/glibc** toolchain needs two flags that `common/sw_build.mk` carries,
because both of its defaults are wrong for a freestanding Harvard image:

- **`-fno-pie`** — it defaults to PIE, so `gas` expands `la sym` into a GOT
  load and gcc emits PC-relative data addressing. There is no GOT in a flat
  two-image layout, and `link.ld` needs medlow **absolute** addressing
  (`lui`+`addi`) so a data address computed in `.text` resolves in D-mem.
- **`-no-pie -Wl,-N`** — its `ld` emits a `PT_PHDR` segment the flat `MEMORY`
  layout does not cover, failing the link. `-N` (omagic) drops it. Harmless on
  a bare-metal toolchain.

**Linker relaxation is on.** `sw_build.mk` used to pass `-Wl,--no-relax`, which
left every `call` as `auipc ra,X; jalr ra,off(ra)` — an extra instruction, and
an indirect jump the branch predictor cannot predict where a `jal` is predicted
for free. Removing it took 110 of Dhrystone's 111 static `jalr`/`jr` sites down
to `jal`, shrank `.text` ~12% and cut its cycles/iteration ~4%. `gp`-relative
relaxation stays inert because no linker script defines `__global_pointer$` and
`start.S` never loads `gp` — do not define one without also setting `gp`.

## Build & run

In any harness dir:

```
make            # -> build/imem.hex + build/dmem.hex
make show       # disassemble build/program.elf
```

From the repo root, build the quicksort program and run the sim loading it:

```
make sw-run     # = make sw  +  make -C sim run RUN_ARGS="+IINIT=sw/quicksort/build/imem.hex +DINIT=sw/quicksort/build/dmem.hex"
```

The sim's `+IINIT=<path>` / `+DINIT=<path>` plusargs select the images
(defaults `imem.hex` / `dmem.hex`); see `sim/README.md`. Run an oracle the
same way, e.g.:

```
cd sim && make run RUN_ARGS="+IINIT=sw/intr/trap/build/imem.hex +DINIT=sw/intr/trap/build/dmem.hex"
```

## Shared assets (common/)

- `start.S` — freestanding `_start` at I-mem 0x0: zero `.bss` from
  `__bss_start`/`__bss_end` (NOBITS, so not in the loaded image; on hardware
  it holds power-up BSRAM contents while simulation reads zero), set
  `sp = 0x4000` (top of the 16 KiB D-mem — it decodes only `ADDR_W` bits, so a
  pointer above the top aliases back into the data), zero `a0`/`a1` (a
  `main(int, char **)` reads them and nothing else sets them), `call main`,
  halt loop. C programs link this; standalone `.S` tests carry their own
  `_start`.
- `link.ld` — Harvard link script used by every harness but one:
  `IMEM (rx) ORIGIN = 0` (code) and `DMEM (rwx) ORIGIN = 0x2000` (data).
  (`dhrystone/` links its own, `dhry_link.ld`, with `DMEM ORIGIN = 0` — its
  10 KiB `Arr_2_Glob` does not fit in the 8 KiB above 0x2000.) Small-data sections
  (`.srodata*`/`.sdata*`/`.sbss*`) are collected into the same output
  sections — gcc puts any object up to `-msmall-data-limit` (8 bytes) there,
  and the image is extracted with `objcopy -j .rodata -j .data`, so an
  uncollected `.srodata` never reaches the D-mem hex (a 4-byte `const` then
  reads 0 at runtime while the same constant folded at compile time reads
  correctly). `.data` sits at 0x2000 (not 0) so the Spike co-sim can hold
  `.text`@0 and `.data`@0x2000 disjoint. Requires `medlow` absolute addressing,
  not `medany` (PC-relative would resolve data addresses against the code's
  I-mem base and break the Harvard split).
- `bin2hex.py` — raw little-endian `.bin` → `$readmemh` word file
  (`@<word>` + one 8-hex-digit word per line). `--base <byte_addr>` sets the
  `@` index (`dmem.hex` uses `--base $(DMEM_BASE)`, 0x2000 → `@0x800`;
  `dhrystone/` overrides it to 0).
  `--pad-words N --pad-value W` pads the image to N words with W (ebreak
  default) — board images pad to the full declared I-mem depth so
  GowinSynthesis sizes the inferred ROM from the `$readmemh` content at the
  right depth, else a stray fetch above the image aliases back into real code.
- `uart.h` — UART MMIO register definitions; `-I$(COMMON_DIR)` makes
  `#include "uart.h"` work from any harness.
- `ee_printf.c` — minimal integer `printf` over the UART (`%d %u %x %s %c %%`,
  optional zero-padded width, `\n` → CR+LF). Linked by the two harnesses whose
  workload is vendored and calls `printf` — `coremark/` and `dhrystone/`;
  everything else writes through `uart.h` directly.
- `sw_build.mk` — shared build logic; see the header comment for the variables
  a harness sets.

## Programs and tests

- **`quicksort/`** — recursive quicksort over a 256-word `.data` array (Lomuto
  partition, `volatile` to defeat constant-folding); verifies ascending order
  and returns `0x600D`/`0xBAD`. `partition()` hand-encodes a Zilx scaled
  indexed word load (`lxs.w`) via `.insn` (`-march=rv32imac` has no Zilx
  mnemonics). The array is filled by a deterministic LCG so the co-sim
  compares like for like. Printed before/after the sort; `make PRINT_ARRAY=0`
  compiles the printing out (the co-sim needs that — the first UART access is
  where Spike stops being comparable).
- **`coremark/`** — EEMBC CoreMark. `eembc/` holds the upstream sources
  **vendored verbatim**, byte-identical to github.com/eembc/coremark at
  commit 1f483d5 (Apache-2.0; provenance and hashes in
  `eembc/UPSTREAM.md`); `make verify-eembc` runs before anything compiles and
  fails the build on any edit, because a score from a modified workload is
  not comparable. Everything the port needs sits outside that directory, in
  `core_portme.[ch]` and `ee_printf.c`: `MEM_METHOD=MEM_STATIC`,
  `HAS_FLOAT=0`, `HAS_STDIO=0`, timing from `mcycle`, a small integer
  `printf` over the UART, local `memcpy`/`memset`, a banner before the run,
  and a port summary after CoreMark's own output.
  Built `-O3 -ffunction-sections -fdata-sections -mstrict-align
  -mbranch-cost=10 -ffp-contract=off -mno-fdiv` with `-Wl,--gc-sections` —
  the set comparable published rv32 ports quote. `-mstrict-align` is not
  optional: this core traps on misalignment rather than fixing it up. 11.1 KiB
  `.text` (of 16) / 4.0 KiB D-mem data. A 2026-09-02 sweep: **GCC 13.3.0 -O3
  best (417 809)**, GCC 16.1.0 -O3 0.7% behind (420 897), -O2 builds ~5%
  behind (437 616 GCC 14.3.0 / 439 457 GCC 13.3.0); earlier, gcc 15.1.0 had
  measured ~2% slower than 14.3.0.
  **Score: 417 809 cycles/iteration = 2.39 CoreMark/MHz** (2000 iterations,
  2K, -O3, GCC 13.3.0 — best of the 2026-09-02 sweep), measured on the board.
  Was 420 560 / 2.37 on the GCC 14.3.0 -O3 default; 466 120 / 2.14 before the
  LSU load-path change; 531 025 / 1.88 before the branch predictor and linker
  relaxation. A/B reproduce recipe in `sim/bench_ipc_ab.md`. CRCs `list 0xe714` /
  `matrix 0x1fd7` / `state 0x8e3a` = the official 2K performance-seed values;
  `crcfinal` 0x4983 on a 2000-iteration run. At the board's 50 MHz a
  rules-valid run (≥10 s, under the 32-bit `mcycle` wrap — there is no
  `mcycleh`; the 10-s minimum scales with the clock, the wrap maximum is
  cycle-based) is `ITERATIONS` 1197..10277; 2000 iterations takes ~17 s. A shorter
  run ends in upstream's own "ERROR! Must execute for at least 10 secs" — the
  run rule, not a core failure; the four CRC lines say it computed correctly.
  `TOTAL_DATA_SIZE=6000` (6K profile), `IMEM_PAD_WORDS=2048` (16 KiB / 8,
  the 64-bit I-mem word width) for a board build. `COSIM=1` builds the co-sim variant: no cycle counter (the one
  register write Spike can't reproduce), no banner, and `-O2` — see
  `sim/cosim/coremark/` (332 803 retires matched).
- **`dhrystone/`** — Dhrystone 2.1. `sifive/` holds SiFive's
  `benchmark-dhrystone` tree **vendored verbatim**, byte-identical to
  github.com/sifive/benchmark-dhrystone at commit 0ddff53 (provenance and
  hashes in `sifive/UPSTREAM.md`); `make verify-sifive` runs before anything
  compiles and fails the build on any edit, because `strcpy` and `strcmp` are
  called from inside the measurement loop, so the loop body *is* the
  benchmark. The port sits outside that directory, in `dhry_portme.c` (the
  real `main()`, `time()` over `mcycle`, a bump allocator for the two records
  Dhrystone mallocs, a word-at-a-time `strcpy`, `memcpy`/`memset`, a banner and
  a summary), `stdio.h` (a freestanding shadow of the toolchain's, which
  `dhry.h` includes "for strcpy, strcmp") and `dhry_link.ld`; `printf` is
  `common/ee_printf.c`.
  Built `-O3 -std=gnu17 -mstrict-align -fno-common -falign-functions=4`,
  `-DTIME -DNOENUM` from upstream's own Makefile. Two defines stand in for
  edits upstream must not receive: `-Dmain=dhry_main` on `dhry_1.c` alone, so
  the port can print after the report (`start.S` calls `main` once and spins;
  Dhrystone has no `portable_fini()`), and `-Dfloat=long`, because the seven
  `float` uses are all in the report block after the timer stops, upstream
  already prints both results through an `(int)` cast, and this core has no
  FPU while the toolchain's libgcc is built `ilp32d` — the soft-float helpers
  do not exist to link against. 5.8 KiB `.text` (of 16) / 1.9 KiB `.rodata` +
  10.3 KiB `.bss`.
  **Score: 543 cycles/iteration = 1.04 DMIPS/MHz at 50 MHz** (200 000 runs,
  2.17 s, 92 081 Dhrystones/s = 52.40 DMIPS). Was 537 / 1.05 on the GCC 14.3.0
  -O3 default; 608 / 0.93 post redirect-launch fetch; 686 / 0.82 before the
  branch predictor and linker relaxation — Dhrystone gains most from
  relaxation because it is call-dense.
  All 22 of Dhrystone's
  own `should be:` final values match at every iteration count tried. The
  figure is `strcpy`-sensitive by construction — a plain byte loop measures
  891 cycles/iteration = 0.63 DMIPS/MHz on the same core — which is why the
  port ships the word-at-a-time version published scores are quoted against
  and says so in `sifive/UPSTREAM.md`.
  A run under 2 s ends in upstream's own "Measured time too small to obtain
  meaningful results"; that is Dhrystone's run rule (`Too_Small_Time`), and it
  needs `DHRY_ITERS` ≈ 184 163 at 50 MHz (the 32-bit `mcycle` caps the other
  end at ~7.9 M iterations). The port summary is computed from cycles and has
  no minimum. **Not co-simulated**: `Arr_2_Glob` is 10 000 bytes of `.bss`, so
  `dhry_link.ld` needs the whole 16 KiB D-mem with `DMEM ORIGIN = 0` — which
  is where `.text` must live too, and Spike's single address space cannot hold
  both.
- **`isa/ifault/`** — instruction-access-fault oracle (jump outside the I-mem).
- **`isa/isa_probe/`** — instruction/memory probe that reports without the hex
  printer or any instruction under test; board bring-up probe.
- **`isa/rvc_scramble/`** — per-scramble-bit RVC decode oracle for
  `c_expand()`.
- **`isa/span_target/`** — same-cycle target-span stitch oracle (a 32-bit
  instr at a 2-byte-aligned branch target); guards the zero-bubble
  `target_span_complete` path.
- **`isa/bp_pred/`** — branch-predictor correctness oracle (2026-08-28): 9
  cases over every predictor path, self-checking the architectural result.
  Guards that a prediction may mispredict freely but never alter
  architectural state; caught the hold-buffer-survives-predicted-redirect
  bug. Pure assembly, own `_start`, `mtvec` handler.
- **`intr/trap/`** — standalone M-mode trap-exercise program (ecall /
  load-misaligned / illegal / MSIP + WFI).
- **`intr/timer/`** — standalone M-mode timer-interrupt program.
- **`intr/wfi_trap/`** — WFI-wake arbitration regression (illegal-trap behind
  `wfi`); `python3 gen_hex.py` builds it with no toolchain.
- **`peri/uart_echo/`** — UART echo + external-interrupt (MEIP) oracle, and
  the hardware bring-up program.

See `sim/README.md` for each oracle's expected pass marker and run command.
Build artefacts (`build/`) are gitignored.

## Core coverage (quicksort)

The LSU is live (loads/stores retire through the native D-mem, or the peri
bridge for `addr[PERI_ADDR_BIT]` addresses) and the execute→decode forward
path covers register and load-use hazards (zero bubble), so a memory-heavy
program like quicksort is a real correctness check: fetch / decode / execute
plus the data path (array loads/stores, stack spill/fill from recursion,
branches on loaded values). `.bss` is zeroed at runtime — `start.S` clears it
before `main`. Keep quicksort's data accesses in the D-mem region (low
addresses) — a peri access (`addr[28]` set) hits a peripheral slave and has no
D-mem alias, so it would stall the LSU.
# Linux on yarv32-uc — Phase 1 assessment

Status: **Phase 1 (read-only) deliverable.** No RTL, constraint or build file
was changed to produce it. Written 2026-11-21 against `main` at `b96a49b`.

Conventions used throughout:

- **[V]** = verified in this session by reading the RTL / reports / running a
  tool; a `file:line` citation follows.
- **[E]** = estimate or engineering judgement. Never treat an [E] number as a
  measurement.
- **[X]** = external fact, cited at the end (§9). Where an external source was
  inconclusive it says so.

---

## 0. Summary

1. The core is **RV32IMC + Zicsr + Zifencei(nop) + custom Zilx, M-mode only.**
   The "A" in RV32IMAC is not implemented: every standard AMO/LR/SC decodes
   to illegal. Worse, `misa` *advertises* A, so software that trusts `misa`
   would execute AMOs and trap. (§1)
2. Both Linux routes require the same large, core-invasive base: A
   extension, a variable-latency unified memory over the 8 MiB SDRAM with
   I/D caches, a real `fence.i`, working counter CSRs (`time` in particular),
   Linux-compatible CLINT/PLIC/UART register layouts, and a boot path.
   Route B adds S/U, delegation and Sv32 on top. **Route A's work is a strict
   prefix of Route B's.** (§2)
3. 8 MiB is **sufficient for Route A** and **sufficient but tight for Route
   B** (§3.6). Linux has already run on this exact board: KianV's
   multicycle **nommu** SoC lists the Tang Nano 20K as supported. KianV's
   Sv32 SoC does not list it (§4.4) [X]. So (d) is not the honest answer for
   Route A. For Route B on this board there is no precedent.
3a. **RV32 nommu is in mainline** since Linux 6.4 (2023), and a vanilla
   6.18 LTS nommu kernel boots on another RV32IMAC soft core at 50 MHz
   (NEORV32) [X]. It was proposed for deprecation in 2024, with removal
   targeted for ~2027 [X]. The nommu gate therefore stays, on a pinned LTS.
4. The BSRAM budget does not fit as currently drafted: the cache subproject
   spends **32 of 46 blocks on 16 KiB of cache data** because its line
   macros are 256 bits wide. Re-shaping them to 64-bit-wide data macros
   recovers ~24 blocks. Its logic is the bigger problem: yosys measures the
   cache subproject at ~34.8k logic cells, on the order of the whole device,
   before the CPU is added. It needs a resource-driven redesign. (§4.1)
5. Every Linux-capable configuration loses the two cycle-count wins the
   frozen MCU is built around (1-cycle BSRAM memories, live D-mem load
   launch into a 1-cycle slave). [E] CoreMark/MHz falls from 2.40 to roughly
   1.6–1.9. (§5)
6. **Recommendation: (c) as the structure, executed as Route A then Route B.**
   Keep `main` as the frozen microcontroller; build the Linux SoC on a
   long-lived branch with its own board top. Only the `misa`, counter and
   illegal-CSR fixes land on `main`, each after its own PnR measurement (the
   A extension stays on the branch). The nommu boot (mainline since 6.4) is a
   go/no-go integration gate, not a product, because RISC-V nommu was
   proposed for deprecation [X]. **Approved in principle 2026-11-21; §6 is
   the revised plan.** (§5, §6)
7. **Blockers before any of this:** the PHT-in-BSRAM question, the fact that
   the current `main` tree (FFT + I2S) has **never been through PnR** — the
   frozen +1.224 ns run predates both — and the cache subproject's open board
   bring-up. (§4.3, §6 M0)

---

## 1. Ground truth of the current ISA and privilege support

### 1.1 Instruction set

| Item | Status | Evidence |
|---|---|---|
| RV32I | Implemented | decode case tree, `src/rtl/core/decode_stage.sv` (opcode cases through :1300) [V] |
| M (MUL/DIV/REM) | Implemented; MUL single-cycle DSP, DIV 32-iteration FSM | `alu.sv`; guarded by `sw/isa/mul_ops`, `div_ops` [V by CLAUDE.md + oracle list] |
| C | Implemented, expand-then-decode | `decode_stage.sv` `c_expand` (~:270–:560) [V] |
| **A (LR/SC, AMOs)** | **NOT implemented. All standard AMO encodings are illegal.** | `OPC_AMO` handles only Zilx funct5 `10010`/`11010`; everything else leaves `zilx_ok=0` and the instruction illegal — `decode_stage.sv:1104-1182`, comment `:1169` "real AMO -> illegal"; header `:67-70`, `:79` [V]. The compliance Makefile excludes `rv32ua` as "NOT RUNNABLE. This core has no A extension" — `sim/sw/compliance/Makefile:27` [V] |
| `misa` | **Advertises A although A is absent.** `MISA_RESET = 32'h4000_2105` sets bits 0 (A), 2 (C), 8 (I), 13 (M); the comment says "RV32 I/M/A/C" | `src/rtl/core/csr_regfile.sv:99-103` [V]. Bug independent of Linux: should read `0x4000_2104` until A exists |
| Zilx (custom indexed loads) | Implemented under the standard AMO major opcode | `decode_stage.sv:1104-1182`; `rv32_pkg.sv:403` [V]. Its funct5 values do not collide with A, Zabha or Zacas, so A can coexist; it is still a custom encoding inside standard opcode space |
| Zicsr | CSRRW/S/C(+I) decoded and retired; RMW in execute | `decode_stage.sv:1210-1241` [V] |
| Zifencei | `fence.i` decodes as `SYS_FENCE_I` and **is a nop**; `fence` likewise | `decode_stage.sv:1277-1294`; no consumer of `SYS_FENCE`/`SYS_FENCE_I` in `execute_stage.sv` (grep: only `SYS_WFI`, `:628`) [V]. Compliance XFAIL `rv32ui-fence_i` — `sim/Makefile:259-262` [V] |
| `sfence.vma` | Illegal | `decode_stage.sv:1269-1270` [V] |
| `ecall` | Always cause 11 (ecall-from-M); no U/S variants | `decode_stage.sv:1250-1255` [V] |
| `mret`, `wfi`, `ebreak` | Implemented; no `sret` | `decode_stage.sv:1256-1268` [V] |

### 1.2 Privilege and CSRs

- **Privilege modes: M only.** No privilege-level register exists; `MPP` is
  forced to `11` on every RMW write (`csr_regfile.sv:153-154`) and on every
  trap entry / `mret` (`trap_unit.sv:166-177`) [V].
- **Implemented CSRs (11):** `mstatus misa mie mtvec mscratch mepc mcause
  mtval mip mcycle minstret` — `rv32_pkg.sv:562-572`, read mux
  `csr_regfile.sv:105-120` [V].
- **Everything else reads 0 and ignores writes, without trapping**
  (`csr_regfile.sv:118`, header `:16-18`) [V]. Consequences that matter for
  Linux:
  - `cycle`/`time`/`instret` (`0xC00-0xC02`) and their `h` halves read **0**.
    The riscv-tests `zicntr` test passes only because it checks that the
    access does not trap, not the value
    (`sim/sw/compliance/riscv-tests/isa/rv64mi/zicntr.S`) [V].
  - `mcycleh`/`minstreth` absent (known gap, XFAIL `rv32mi-instret_overflow`,
    `sim/Makefile:272-276`) [V].
  - `mhartid`, `mvendorid`, `marchid`, `mimpid` read 0 — acceptable (hart 0).
  - Because an unimplemented CSR does not trap, **OpenSBI cannot emulate
    `rdtime`** (it does so from the illegal-instruction handler). Route B
    therefore needs either a real `time` CSR or the illegal-CSR trap; Route A
    needs `time` or a clocksource from CLINT MMIO.
- `mstatus` stores all other bits raw (MPRV, SUM, MXR, TVM, TW, TSR…) with
  no effect (`csr_regfile.sv:153-154`) [V].
- `mtvec` direct + vectored (`csr_regfile.sv:159-160`, `trap_unit.sv:127-136`) [V].
- CSR read is **registered**, one cycle late (`csr_regfile.sv:201-208`) [V].
- No PMP CSRs (XFAIL `rv32mi-pmpaddr`) [V].

### 1.3 Traps, misaligned access, interrupts

- Sync causes implemented: 1, 2, 3, 4, 6, 11 (`rv32_pkg.sv:537-545`) [V]. No
  page-fault causes (12/13/15), no access-fault on load/store (5/7), no ecall
  8/9.
- **Misaligned load/store: trapped, never performed.** `misaligned_of()`
  (`execute_stage.sv:419-426`) gates the launch and raises cause 4/6 from
  `EX_MEM_TRAP` (`:428-437`) [V]. Legal per spec; Linux handles it in M-mode
  (`traps_misaligned.c`) and OpenSBI emulates it for S-mode [X, well-known
  behaviour; verify against the pinned kernel].
- Interrupts: MSI/MTI/MEI, priority MEI>MSI>MTI, gated by `mstatus.MIE`
  (`trap_unit.sv:106-119`) [V]. A pending interrupt does not preempt a memory
  op (`execute_stage.sv:669`) [V] — legal.
- **CLINT layout is non-standard** [V]:
  - `clint_timer` at `0x1000_1000`, four 32-bit words: `mtime_lo/hi`,
    `mtimecmp_lo/hi` (`clint_timer.sv:10-17`); `mtime` increments every core
    clock (`:72-74`) so the timebase is 50 MHz.
  - MSIP is a separate 1-bit peripheral at `0x1000_3000`
    (`rv32_pkg.sv:201`).
  - Linux's `timer-clint` driver and OpenSBI's CLINT expect the SiFive
    layout — `msip` at +0, `mtimecmp` at +0x4000, `mtime` at +0xBFF8 in one
    window [X; to verify against the pinned kernel source]. OpenSBI's
    ACLINT binding can describe split MSWI/MTIMER regions, but Linux's M-mode
    nommu path uses the CLINT driver directly.
- **PLIC is non-standard** [V]: three registers — `PENDING` (0x00),
  `ENABLE` (0x04), `CLAIM` (0x08, read-only, side-effect-free), no priority,
  no threshold, no complete-write, 16 sources, 4 KiB window
  (`axi4_lite_plic.sv:26-34`, `:92-94`, `:235-237`; `rv32_pkg.sv:209-210`).
  Linux's `irq-sifive-plic` needs per-source priority (it writes 1 to unmask,
  0 to mask), the enable array at +0x2000, threshold/claim/complete at
  +0x200000 per context [X]. Not usable as-is.
- **UART is a custom register map** (`axi4_lite_uart.sv:23-43`) [V]; no
  mainline Linux driver matches it.

### 1.4 Memory system (verified)

- Harvard. I-mem 64-bit read-only, D-mem 32-bit; both `native_ram`
  `ADDR_W=14` (16 KiB), instantiated in `top_module.sv:324-374` [V].
  `READ_ONLY` removes the I-mem write path entirely (`native_ram.sv:235`) [V],
  so **no D→I path exists** — the root of the `fence.i` and `rvc` case-6
  XFAILs.
- The two spaces overlap at address 0: a data load of address *x* reads
  D-mem, never I-mem. Peripherals are selected by `addr[28]`
  (`rv32_pkg.sv:254`, `execute_stage.sv:393`) [V].
- Fetch faults any PC ≥ 16 KiB (`fetch_stage.sv:301`) [V].
- **Fixed-latency assumption:** fetch keeps a single in-flight PC
  (`req_pc_q`) and asserts that a second launch never overlaps an unanswered
  one (`fetch_stage.sv:626`) [V]. A cache with ≥2-cycle hit latency or any
  miss breaks this; CLAUDE.md plan item 2 already records the fix (depth-2
  in-flight PC FIFO).

---

## 2. Gap analysis

Effort is **[E]**, in engineer-weeks for one person who knows this codebase,
including verification but excluding board debug of surprises. Risk is the
probability that the item costs >2× its estimate or forces a redesign.

### 2.1 Route A — Linux nommu, M-mode only

| # | Item | Touches | Effort [E] | Risk | Notes |
|---|---|---|---|---|---|
| A1 | Fix `misa` (drop A until it exists) | `csr_regfile.sv:103` | 0.1 | low | Correctness bug, independent of Linux |
| A2 | Counters: `cycle/instret/time` + `cycleh/instreth/timeh`, `mcycleh/minstreth` with write-suppress semantics | `csr_regfile.sv`, `rv32_pkg.sv`, a wire from `clint_timer` `mtime` | 1 | low | Turns `rv32mi-instret_overflow` into XPASS |
| A3 | Illegal-instruction trap on unimplemented/read-only-write CSR | `decode_stage.sv` or `execute_stage.sv`, `csr_regfile.sv` | 1 | med | Needs an address-valid decode; the CSR read is registered, so the check must be on the decode-side address. Changes MCU firmware semantics (YarvMon etc. must not touch unimplemented CSRs) |
| A4 | **A extension**: LR.W/SC.W + 9 AMO.W | `decode_stage.sv` (OPC_AMO), `execute_stage.sv` (new FSM states: load → op → store), small AMO ALU fed from the load-data flop, reservation register | 2–3 | med | Single hart, no DMA into SDRAM, so atomicity is trivial; the cost is FSM states and keeping the AMO ALU off the `alu_result` critical path. Reservation cleared on trap/`mret`/SC. Misaligned AMO → cause 6 |
| A5 | **Unified memory over SDRAM**, I-cache + D-cache, write-back | new SoC top; `yarv32-native-caches/cache_cntrl.sv`; `fetch_stage.sv` (in-flight PC FIFO); `execute_stage.sv` (LSU over a multi-cycle slave; 32↔64-bit D port adapter) | 6–10 | **high** | The single biggest item. See §3 |
| A6 | **Real `fence.i`**: clean dirty D-lines + invalidate I-cache, then refetch | cache controller (sweep FSM), `execute_stage.sv` (stall + redirect to pc+4), `fetch_stage.sv` (kill buffer) | 1.5 | med | Linux calls `fence.i` on every module load / exec of new text; without it nommu exec of a freshly loaded binary fetches stale lines |
| A7 | CLINT in the standard layout (one window: msip +0, mtimecmp +0x4000, mtime +0xBFF8) | `clint_timer.sv`, `msip_peri.sv` (merge), `rv32_pkg.sv`, both tops, firmware headers | 0.5–1 | low | Breaks `sw/common/timer.h` users on the MCU — do it only in the Linux top or behind a parameter |
| A8 | PLIC in the SiFive layout (priority per source, enable array, threshold, claim/complete, gateway) | `axi4_lite_plic.sv` (or new module), xbar window ≥ 0x20_1000 → **4 MiB** window | 1–1.5 | low | Context 0 (M) for Route A; Route B adds context 1 (S) |
| A9 | **16550-compatible UART** (decided 2026-11-21): RBR/THR, IER, IIR/FCR, LCR (DLAB divisor latch), MCR, LSR, MSR, SCR, `reg-shift=2`/`reg-io-width=4` | new block or a 16550 front end over the existing FIFOs/baud generator | 1–1.5 | low | Mainline `8250_of` / `ns16550a`, OpenSBI `uart8250`, earlycon `uart8250,mmio32` — no driver to write |
| A9b | **SPI loader path** for kernel/initramfs from flash (decided: SPI stays on the Linux branch) | existing `axi4_lite_spi.sv` + boot ROM code | 1 | med | Which flash (on-board config flash vs external) is open question Q7 |
| A10 | Boot ROM + loader | BSRAM ROM (the cache project already has a 2 KiB bootrom at `0x80_0000`), loader firmware | 1–2 | low (UART) / med (SPI flash) | §3.5 |
| A11 | Device tree, Buildroot config, kernel config, pinned kernel | software only | 2–3 | med | RV32 nommu is in mainline since 6.4 (`rv32_nommu_virt_defconfig`) and is demonstrated on 6.18 LTS by the NEORV32 port [X]. Pin 6.18 LTS; removal proposed for ~2027 [X] |
| A12 | Misaligned access | none | 0 | low | Kernel M-mode trap handler emulates [X, verify]; userspace built with strict alignment helps |

**Route A total [E]: 18–28 engineer-weeks**, dominated by A5.

### 2.2 Route B — full Linux with MMU (everything in Route A, plus)

| # | Item | Touches | Effort [E] | Risk | Notes |
|---|---|---|---|---|---|
| B1 | Privilege level register; U and S modes; `mret` to MPP, `sret` to SPP; MPRV | `csr_regfile.sv`, `trap_unit.sv`, `execute_stage.sv`, `decode_stage.sv` | 2 | med | Today MPP is a constant; trap entry must pick M or S by delegation |
| B2 | `medeleg`/`mideleg`; S-mode trap entry; SEI/STI/SSI with correct priority across levels; `mip.STIP` writable from M | `trap_unit.sv` (doubles), `csr_regfile.sv` | 2 | med | Interrupt eligibility depends on current privilege + `mstatus.SIE/MIE` |
| B3 | Supervisor CSR set: `sstatus sie sip stvec sscratch sepc scause stval satp scounteren`, plus `mcounteren` | `csr_regfile.sv` (≈11 → ≈25 entries) | 1 | low | Read mux grows; it is registered already, so off the timing hot path |
| B4 | Privilege checks: CSR privilege, `sret`/`sfence.vma`/`wfi` with TSR/TVM/TW, U-mode illegal on M-only instructions; ecall causes 8/9 | `decode_stage.sv`, `execute_stage.sv` | 1 | low | |
| B5 | **Sv32 MMU**: ITLB + DTLB (fully associative, 8–16 entries each), hardware page-table walker sharing the D-cache port, superpages (4 MiB), permission checks (R/W/X/U, SUM, MXR), page-fault causes 12/13/15 | new `mmu.sv`/`ptw.sv`; `fetch_stage.sv`, LSU in `execute_stage.sv`, cache tag compare | 6–8 | **high** | See §4.2 for where it lands relative to timing. **VIPT is free at this cache shape**: 8 KiB 2-way = 4 KiB per way = page size, so set index + offset sit entirely in the untranslated 12 bits and the TLB lookup runs parallel to the tag read |
| B6 | `sfence.vma` (flush TLBs; ASID optional — treat as global flush) | `decode_stage.sv`, MMU | 0.5 | low | |
| B7 | A/D bits: **trap approach (Svade)** — raise page fault when A=0, or D=0 on a store; no hardware PTE update | MMU | 0.5 | low | Linux supports this; avoids a PTW write path |
| B8 | PMP: none (CSRs trap or read 0). OpenSBI probes PMP count and copes with zero | — | 0 | low | Requires A3 so the probe sees a trap [X, verify OpenSBI version] |
| B9 | OpenSBI `fw_jump`/`fw_dynamic`, generic platform + DT; SBI timer via `mtimecmp` (or Sstc later) | software | 2 | med | Places ~256 KiB of firmware in SDRAM |
| B10 | Kernel + Buildroot (MMU, RV32, `rv32ima[c]`) | software | 2 | med | Better-trodden than nommu |

**Route B increment [E]: +17–19 engineer-weeks; total ≈ 35–47.**

---

## 3. Memory subsystem

### 3.1 What exists

`src/sub/yarv32-native-caches/` (untracked in this repo; its own git
history, last commit `8f67d78`) is a 2-way, 8 KiB + 8 KiB, 32-byte-line,
write-back cache controller over the embedded SDRAM, with the
`stffrdhrn/sdram-controller` fork (32-bit, CL=3, BL=1, auto-precharge every
access), a 2 KiB bootrom, a cache-bypass control register for loaders, and a
board BIST [V: its `CLAUDE.md` §Files/§Cache configuration, `TODO.md`].
Its own status [V, `TODO.md:220-437`]:

- RTL sim, BIST sim and gate-level sim pass.
- **Board bring-up is not closed.** Several hang root causes were found and
  fixed (unreset power-up state, tag RAM never invalidated, SDRAM 100 µs
  power-up wait, `Initialize_Primitives`). Whether build `V10` passes on the
  board is not recorded. Treat as **open** (question Q3).
- Never PnR'd at 50 MHz ("Re-run the flow" item open, `TODO.md:276`).
- CPU ports are **64-bit on both I and D**; this core's D port is 32-bit
  (`mem_req_t`). An adapter (or a 32-bit D port in the cache) is needed.

### 3.2 Latency and what it costs

- Hit latency [E from `cache_cntrl.sv:868-875`: accept into a skid,
  lookup launched, tag + data answer one cycle later, then the response
  stage]: **≥2 cycles**, versus 1 today. To be measured in sim (M4).
- Miss latency [E]: BL=1 with auto-precharge means every 32-bit word pays
  ACT + tRCD + READ + CL + precharge, ~6–8 cycles at 50 MHz, so a 32-byte
  refill is **~50–70 cycles** and a dirty eviction roughly doubles it. The
  plan's "~12 cycles/miss" (CLAUDE.md plan item 2) assumed bursts; the chosen
  controller does not burst. **Burst reads are now an explicit milestone (M6
  in §6), and they must land before any cache performance figure is
  reported.** Its acceptance criterion is a *measured* line-refill latency.
  An 8-beat burst with one ACT and CL=3 would be ~14–16 SDRAM cycles [E];
  the fork can set a burst length in the mode register or chain column
  reads to an open row.

### 3.3 Sizing within 46 BSRAM blocks

See §4.1 for the full budget. The key finding: the cache data macros are one
**256-bit** word per line, and a GW2A BSRAM is at most 36 bits wide, so each
256-bit macro costs **8 blocks** regardless of depth [V: cache `TODO.md:255-262`
"8 blocks per data macro (32) + 1 per tag macro (4) = 36 of 46"]. 16 KiB of
cache data therefore occupies 32 blocks whose raw capacity is 72 KiB — 22%
utilisation. **Recommendation:** 64-bit-wide data macros (4 KiB/way = 512 ×
64 → 2 blocks per way), refill writes 4 × 64-bit beats (hidden under a ~60
cycle SDRAM refill). Data cost drops 32 → 8 blocks [E, arithmetic; verify with
GowinSynthesis, which has already disagreed with yosys once on this design].
That leaves room either for the FFT or for bigger caches — but keep **4 KiB
per way** (i.e. grow by adding ways, 16 KiB 4-way) if Route B is planned,
or VIPT starts aliasing.

### 3.4 I/D coherence and `fence.i`

With a write-back D-cache, code written by stores (kernel loading an ELF,
module, or the loader itself) lives in dirty D-lines the I-cache cannot see.
`fence.i` must: (1) write back every dirty D-line (or the whole D-cache),
(2) invalidate the I-cache, (3) kill the fetch buffer and refetch from pc+4.
A sweep FSM over 128 sets × 2 ways is ~256 cycles plus writebacks [E]; Linux
issues `fence.i` rarely enough that this is fine. The bypass register in the
cache project is explicitly **not** a coherence mechanism (its `CLAUDE.md`
§System address map) [V]. Acceptance test: `rv32ui-fence_i` turns XPASS, and
`rv32uc-rvc` case 6 passes once data loads can read text.

### 3.5 Boot ROM and getting the kernel in

- **Boot ROM:** the cache project's 2 KiB BSRAM ROM at `0x80_0000`, readable
  by both ports [V]. Enough for a loader.
- **UART loader** (first): 115200 8N1 ≈ 11.5 KB/s, so a 1.5 MiB nommu image
  ≈ 2.3 min and a 4 MiB MMU kernel + initramfs ≈ 6 min [E, arithmetic]. The
  BL616 bridge may run faster rates (§8 Q6). Simple, already-proven path
  (YarvMon is a UART monitor).
- **On-board SPI flash** (second, for real boot): the Tang Nano 20K carries a
  64 Mbit config flash; the bitstream uses well under 1 MiB of it, so a
  kernel fits behind it. Access from fabric depends on the MSPI pins being
  user-accessible after configuration (a community project does this [X]).
  Needs verification (§8 Q7).
- **microSD** (PIN80–85, unused since the SDIO controller was dropped — README
  pin table) via SPI mode: feasible, adds a controller and FAT/raw-sector
  loader. Lower priority.
- **Preload via bitstream:** impossible for SDRAM (it is a separate die;
  only BSRAM initialises from the bitstream).

### 3.6 Is 8 MiB enough?

| | Route A (nommu) | Route B (MMU) |
|---|---|---|
| Firmware | none (kernel runs in M) | OpenSBI ~128–256 KiB, reserved |
| Kernel image (tiny config, RV32) [E] | ~1.0–1.5 MiB | ~2.5–4 MiB |
| Kernel runtime (slab, page tables, buffers) [E] | ~0.5–1 MiB | ~1–2 MiB |
| initramfs (BusyBox) [E] | 0.3–0.8 MiB | 0.5–1 MiB |
| **Free for userspace / page cache** [E] | **~4–5 MiB** | **~1–3 MiB** |
| Verdict | **Sufficient.** K210 boots nommu Linux with ~6 MiB usable [X] | **Sufficient but tight.** Shell + BusyBox, little else. No glibc; uClibc-ng or musl |

Caveat to check (in M13): RV32 Linux wants the kernel at a PMD-aligned physical
address (4 MiB on Sv32); with OpenSBI at the base, whether the memory below
the kernel is reclaimed depends on the kernel version.

---

## 4. FPGA resource and timing budget

### 4.1 Resources

Device: GW2AR-18C — 46 BSRAM blocks (828 Kb), 48 MULT18X18, flip-flop limit
15 750 (from GowinSynthesis's own error message, CLAUDE.md FFT entry), LUT4
count ~20.7k [X: device datasheet figure, verify].

Measured today with `impl/yosys_estimate.sh --no-baseline` [V, run
2026-11-21; yosys LUT figures are order-of-magnitude only, register file
patched out of BSRAM, PHT forced into flops]:

| Config | LUT1-4 | MUX2 | ALU | Logic cells | FF | BSRAM | MULT18 | MULT36 | LUT-RAM |
|---|---|---|---|---|---|---|---|---|---|
| current `main` tree | 9860 | 2535 | 1784 | **14179** | 3825 | 23 | 4 | 1 | 149 |
| FFT alone | 1236 | 149 | 415 | 1800 | 681 | 7 | 4 | 0 | 0 |

**The real GowinSynthesis resource table for the current tree is not
available locally** (`impl/gwsynthesis/` is not checked in) — Q1.

BSRAM budget per configuration [E unless marked]:

| Item | MCU today | Linux SoC, cache as drafted | Linux SoC, 64-bit data macros |
|---|---|---|---|
| I-mem / D-mem 16+16 KiB | 16 [V] | 0 (replaced) | 0 |
| Cache data | — | 32 [V cache TODO] | 8 |
| Cache tags | — | 4 [V cache TODO] | 4 |
| Boot ROM 2 KiB | — | 1 | 1 |
| Register file | 1–2 (GowinSynthesis maps it to BSRAM; yosys cannot) | 1–2 | 1–2 |
| PHT (512) | 0–1 (**unresolved**, §4.3) | 0–1 | 0–1 |
| FFT | 7 [V] | 7 → **0, dropped on the Linux branch (review decision)** | 0 |
| **Total** | **24–26** | **45–47 with FFT; 38–40 without** | **14–16** |

**Cache subproject, measured** [V, `make lint-yosys` in
`src/sub/yarv32-native-caches`, 2026-11-21; whole `fpga_top` = `cache_cntrl`
+ SDRAM controller + BIST + debug UART, final `stat` section of
`/tmp/yarv32-yosys-check/synth.log`]:

| LUT1-4 | MUX2_LUT5-8 | ALU | Logic cells | FF | BSRAM (SPX9) |
|---|---|---|---|---|---|
| 29 792 | 4 282 | 766 | **~34 800** | 1 739 | 36 |

**This is the largest risk in the whole assessment.** Even granting the
script's own caveat that yosys LUT counts can run ~2× high, the cache
controller *alone* is on the order of the entire device (~20.7k LUT4), and
the CPU (~14k cells by the same tool) has to fit beside it. The likely
cause [E, not yet localised] is the 256-bit line datapath: line-wide
store-merge muxes, line buffers and response staging, all replicated for
I and D. The 64-bit-data-macro reshaping recommended in §3.3 attacks the
same root cause and should shrink logic as well as BSRAM, but **the cache
controller needs a resource-driven redesign before integration (M0/M4),
and a GowinSynthesis run is required to know the real figure.** The BIST
and debug reporter are included in the count but are small blocks.

Logic [E, deltas over today]: A extension +0.5–1k cells, +~100 FF; counters
and illegal-CSR trap +0.3k; standard PLIC/CLINT/UART ≈ neutral; cache
controller + SDRAM controller as above (must shrink);
S/U + delegation + S-CSRs +1–1.5k cells, +~500 FF; Sv32 (2 × 8-entry TLB +
PTW) +2–3k cells, +~1k FF. Route B could plausibly exceed the LUT budget if
FFT, I2S, I2C and SPI all stay in the Linux top; the Linux top should start
with UART + CLINT + PLIC + SPI only (review decision; FFT, I2S and I2C stay on `main`).

DSP: unaffected (MUL stays one MULT36X36; FFT 4 × MULT18).

### 4.2 Where new logic lands relative to the critical paths

Frozen-run facts [V, `impl/pnr/yarv32-uc-v0.1.0-tangnano20k-50mhz.tr`,
2026-09-14; CLAUDE.md FROZEN paragraph]: worst setup +1.224 ns, every one of
the 25 worst paths starts at `fetch_stage_i/head_q[0]` or `head_q[2]` and
ends in `req_pc_q`, `pc_q`, `u_imem AD[*]`, or `de_bus.rs*_data`; the ALU,
LSU and regfile do not appear in setup. Logic level 15, routing 54–63% of
every path.

Per the repo's rule — *a knob verdict expires when the limiter moves* —
none of those facts is a prediction for a Linux SoC. What can be said
structurally:

- **Group A (`head_q → … → pred_dir_target → imem_req.addr → u_imem AD`)
  ends on a BSRAM address pin.** In a cache SoC the fetch address goes into
  the I-cache's skid register instead; the BSRAM address tail (≈9.8 ns per
  CLAUDE.md item 1e) moves behind a flop. This path probably gets *easier*
  [E], but a 64-bit fetch from a cache with ≥2-cycle hits lengthens the
  predicted-taken refill bubble.
- **Group B (`head_q → decoder cone → forwarding compare → de_bus`)** is
  untouched by memory changes but **grows with every decode addition**: A
  extension decode, privilege-dependent illegal checks (B4), CSR-address
  validity (A3). These must be kept as late-arriving *exception* bits that
  ride `de_t`, not as terms on the operand/forwarding path. This is the main
  timing risk of the ISA items.
- **The live D-mem load launch (`alu_result → u_dmem ADA`)** is gone in a
  cache SoC: the cache's skid flop captures the address. The cycle that
  `LSU_LIVE_LOAD=1` bought is lost in cycles, but the path stops being a
  candidate limiter [E].
- **MMU**: with VIPT at 4 KiB/way, the TLB CAM sits in the cache's
  tag-compare cycle (registered VA → TLB → PPN compare → hit → way mux →
  response). That is a new, entirely post-flop path inside the cache, not on
  fetch→decode — but it is a CAM + 20-bit compare + 2-way select in one
  cycle, and on a device where routing is >50% of each path it is the
  likeliest new limiter [E]. If it does not close, add a cycle to the hit
  (costs CPI on every load).
- **Trap unit** doubles (M vs S target, delegation mux) and sits on
  `branch_valid_o → fetch`. Its inputs are flops/CSR taps, so it is not on
  the regfile path; keep it that way.

**Noise rule:** run-to-run placement noise is ~0.7 ns; every PnR checkpoint
in §6 is ≥3 seeds or it is not evidence.

### 4.4 Comparison with KianV

Sources: the kianRiscV repository README, `LICENSE.md`, and the READMEs of
`linux_socs/kianv_mc_rv32ima_sv32` and `linux_socs/kianv_harris_mcycle_edition`
[X, read 2026-11-21]. The crowdsupply tape-out write-up returned HTTP 403
and was not read. KianV publishes no LUT/FF/BSRAM figures, no Fmax and no CPI,
so a numeric resource comparison is **not possible from published data**.
The comparison below is architectural.

| | yarv32-uc Linux SoC (this plan) | KianV Sv32 SoC | KianV "Harris mcycle" SoC |
|---|---|---|---|
| Tang Nano 20K | target | **not in its board list** | **listed as supported** |
| Linux | nommu gate → Sv32 | Sv32, "most recent stable" kernel, Buildroot | **nommu**, Buildroot from mini-rv32ima |
| Core | 3-stage pipeline, 64-bit fetch, gshare predictor, forwarding | **multicycle** | multicycle |
| ISA | RV32IMAC + Zicsr/Zifencei/Zicntr (+Zilx) | RV32IMA, Zicntr, Zicsr, Zifencei, **Sstc** | RV32IMA |
| MMU | Sv32, I/D TLB (8 entries each [E]), HW PTW, Svade | Sv32, "2 TLBs (one for data and one for instructions)"; sizes, PTW, A/D handling **not stated** | none |
| Caches | 2-way I + 2-way write-back D, 4 KiB/way (VIPT) | **2-way I-cache only, no D-cache**; size not stated | not stated |
| Memory | 8 MiB in-package SDRAM | SDRAM controller; README cites 64 MiB (ULX3S) for X11 | not stated for this board |
| Boot | boot ROM + UART loader, then SPI flash | `make flash_os` to a block device | SPI NOR / SD mentioned, "not checking it in" |

What the comparison says about this plan:

1. **KianV's feasibility evidence for this board is nommu only.** It does
   not demonstrate Sv32 in the GW2AR-18's 46 BSRAM blocks and ~20k LUTs.
   Route B on this device remains unproven by anyone, as far as this
   assessment found.
2. **KianV saves area in exactly the places this plan spends it.** A
   multicycle core with no D-cache and an I-cache-only hierarchy is the
   minimum-area way to boot Linux. This plan keeps a pipelined core and adds
   a write-back D-cache, so it is necessarily larger. That matters given the
   cache subproject's measured ~34.8k yosys cells (§4.1). **A fallback worth
   keeping open is a write-through or uncached D-side for the Linux branch**:
   no dirty state, no writeback path, and a much simpler `fence.i` (invalidate
   I only). The cost is every store going to SDRAM. Decide at M4/M5 by
   measurement.
3. **Sstc** (S-mode `stimecmp`) is in KianV and not in this plan. It removes
   the SBI timer round-trip through M-mode. It is cheap and can be a post-M13
   option.
4. **License: ISC** (`LICENSE.md`, "Copyright (C) 2022 Hirosh Dabui"; the Sv32
   directory carries its own ISC `LICENSE.md`, 2024). ISC is permissive and
   allows reuse with the copyright and permission notice retained.
   **No reuse is proposed.** The microarchitectures differ: multicycle, with
   no interface in common with `native_ram`/`de_t`. If any piece is reused
   later (for example a page-table-walker reference), the notice must travel
   with it, and the per-file license must be checked first. Only two
   directories' licenses were read.

### 4.3 Blockers to resolve first

1. **PHT-in-BSRAM** (CLAUDE.md Open work, first entry). It changes the
   BSRAM budget by up to one block and it is a netlist-vs-RTL fidelity
   question of exactly the class a Linux bring-up cannot afford. Cheapest
   close: simulate CoreMark at 2000 iterations on the frozen configuration
   and diff against the board's 416 331 cycles/iteration; then grep the
   `.vg` for the `pht_q` primitives.
2. **The current `main` has never been through PnR.** The only report on
   disk is the 2026-09-14 frozen run (`.tr` "Created Time: Mon Sep 14");
   the FFT (2026-09-15) and I2S (2026-09-16) landed after it. There is no
   valid MCU baseline to measure Linux changes against until this runs.
3. **Cache subproject board status** (§3.1). Integrating a cache that has
   not passed its own BIST on silicon into a CPU makes every hang
   ambiguous between core and cache.

---

## 5. Decision

### Options

| Option | What it means | Sacrifices |
|---|---|---|
| (a) Evolve in place, Route A then B | `main` becomes the Linux SoC | The frozen MCU: 2.40 CM/MHz, 53 MHz PnR, 32 KiB of 1-cycle BSRAM, the FFT (BSRAM), the guitar tuner's deterministic timing. Every MCU application becomes a Linux-SoC application |
| (b) Route B directly | Same, skipping the nommu boot | Same as (a), plus loses the intermediate integration gate: the first kernel boot exercises MMU + S-mode + OpenSBI + caches + SDRAM at once, which is the worst place to debug a cache bug |
| (c) Separate Linux SoC (branch + own top), reuse modules | `main` stays the frozen MCU; Linux SoC shares the core with knobs/parameters where possible and diverges where not | Maintenance of two tops and two configurations; some core changes (fetch latency tolerance, MMU) must be parameterised or live on the branch |
| (d) Not worth it on this FPGA | — | — |

### Recommendation: **(c), executed as Route A as a gate, then Route B**

Reasoning:

1. **The MCU and the Linux SoC want opposite memory systems.** Every recent
   performance win on this core — live load launch, in-cycle redirect,
   push-time PHT at depth 512 — is built on 1-cycle BSRAM memories. A
   Linux SoC needs a variable-latency cache over SDRAM. Doing it in place
   (a/b) destroys the frozen, board-verified configuration to get a slower
   one. [E] CoreMark/MHz for the Linux SoC: lsu-launch rises by roughly one
   cycle per load (+~0.25 CPI), I-cache hit latency adds ~0.05–0.1 CPI to
   the predicted-taken bubble, and CoreMark's 11.1 KiB of text overflows an
   8 KiB I-cache (+misses at ~60 cycles) — **CPI ~1.43 → ~1.8–2.2, i.e.
   ~1.6–1.9 CoreMark/MHz**, and Fmax unknown until PnR (plausibly 40–50 MHz).
   Programs lose the 16+16 KiB of deterministic BSRAM; FFT may or may not
   fit (§4.1).
2. **Route A is a strict prefix of Route B**, so doing it first costs only
   the nommu-specific software (A11). What it buys is an integration test
   of the hardest hardware (caches, SDRAM, `fence.i`, interrupts, A
   extension) with the MMU and OpenSBI still out of the picture.
3. **Route A should not be the end state.** RV32 nommu has been in mainline
   since 6.4 and still boots on 6.18 LTS (the NEORV32 port) [X]. It was
   proposed for deprecation in 2024, with removal targeted for ~2027, against
   pushback from FPGA users [X]. Pin 6.18 LTS for the gate, then move to MMU.
4. **Not (d), for Route A:** nommu Linux has run on this board (KianV's
   multicycle SoC) and on ~6 MiB of usable memory (K210) [X]. **Route B on
   this board has no precedent** (§4.4). M11–M13 carry real feasibility risk,
   and the go/no-go review after M10 exists for that reason. Expect slow
   boots: tens of seconds to minutes at ~50 MHz [E]. Boot time is reported
   at every milestone that boots anything.
5. Items that are pure correctness and cost nothing measurable on the MCU
   (`misa` fix, counters, `mcycleh`, illegal-CSR trap, A extension) can land
   on `main` — **each only after a PnR measurement against the post-M0
   baseline**, because the decode-side ones touch the current limiter's
   cone (§4.2, group B).

---

## 6. Milestone plan (revised 2026-11-21, after review)

### Decisions recorded from the review

- Option **(c)** approved in principle. `main` stays the frozen MCU. The
  `misa`, counter and illegal-CSR fixes go to `main` first, **each with its
  own PnR measurement** (M1a/M1b/M1c).
- Linux-branch peripherals: **UART, timer (CLINT), PLIC, and SPI** (for
  loading kernel/initramfs from flash). **FFT, I2S and I2C are dropped on
  the Linux branch** and stay on `main`. This frees 7 BSRAM blocks and their
  logic. GPIO is not in the list, so it is dropped too unless you say
  otherwise.
- Peripherals are made **standard-compatible** rather than writing Linux
  drivers: SiFive-layout CLINT, `sifive,plic-1.0.0` PLIC, 16550 UART (M8).
- Success criterion: **boots to a shell. No minimum speed.** Boot time is
  measured and reported at every milestone that boots anything (column
  below).
- RV32 nommu **is in mainline** (since 6.4, still working on 6.18 LTS), so
  the nommu gate stays (M10). The kernel-free tests you proposed are kept
  as well, as the acceptance of the memory milestones (M5–M7), because they
  localise a memory-system bug far better than a kernel hang does.

### Rules for every milestone

`make regress` green. `make compliance` passes with an updated XFAIL list;
an XPASS is progress and must be recorded. `make format-check` clean. Spike
co-sim wherever a milestone adds ISA. For any change to the core, a PnR
checkpoint of **≥3 seeds** that reports worst slack, Fmax, the endpoint list
and the GowinSynthesis resource table. FROZEN knobs stay unchanged unless a
checkpoint justifies the change in writing. Durable lessons go into CLAUDE.md
as they happen.

### Milestones

| M | Content | Where | Acceptance | Boot time reported |
|---|---|---|---|---|
| **M0** | Baseline. (a) PnR the current `main` (FFT + I2S, never PnR'd) ×3 seeds. (b) Close PHT-in-BSRAM: CoreMark 2000-iteration sim cycle diff against the board's 416 331, then grep the `.vg`. (c) GowinSynthesis resource table for `main`. | `main` | Numbers recorded in CLAUDE.md; the PHT question closed one way or the other | — |
| **M1a** | `misa` = `0x4000_2104` (drop A until A exists) | `main` | `sw/isa/csr_ops` checks `misa`; `regress`; PnR ×3 vs M0 | — |
| **M1b** | `cycle/time/instret` + `h` halves (`time` from CLINT `mtime`), `mcycleh/minstreth` with write-suppresses-increment | `main` | `rv32mi-instret_overflow` XPASS and dropped from XFAIL; oracle checks the values, not just the absence of a trap; PnR ×3 | — |
| **M1c** | Illegal-instruction trap on unimplemented CSR / write to read-only CSR | `main` | `csr_ops` mutation-checked; YarvMon and every `main` firmware still `regress`-green; PnR ×3. **This one sits in the group-B decode cone, so it is the likeliest of the three to cost slack.** | — |
| — | **Branch point `linux-soc`** | | | |
| **M2** | A extension (LR/SC + 9 AMOs), AMO ALU fed from the load-data flop | branch | `rv32ua` enabled in `make compliance`; `sw/isa/amo_ops` (non-vacuous); Spike co-sim of an AMO-heavy C program; `misa` A bit set again; PnR ×3 | — |
| **M3** | Variable-latency tolerance: fetch in-flight PC FIFO, LSU over multi-cycle slaves, **latency-fuzz wrapper** around `native_ram` | branch (parameterised) | Whole `regress` + cosims pass with fuzz on over several seeds; cycle counts identical with fuzz off; PnR ×3 | — |
| **M4** | Cache subproject, in this order: (1) **diagnose the 34.8k cells first** — locate which memories (tags, data, line buffers, response queue) yosys and GowinSynthesis inferred as flops/LUT-RAM instead of BSRAM, from the per-module `stat` and the GowinSynthesis resource report; (2) derive the **cache LUT budget** from the M0 table (§6.1); (3) choose among the three cache options of §6.2 the one that fits the budget, preferring the write-back cache only if it fits; (4) implement, `make sim`/`bist`/`gatesim` green; (5) board BIST | cache repo (submodule pinned to a commit) | Cache logic ≤ budget (GowinSynthesis, or yosys with the M0 yosys→Gowin ratio applied when the host is unreachable); BSRAM as predicted; sim/BIST/gatesim green; board BIST passes (Needs Giacomo); PnR ×3 | — |
| **M5** | Linux SoC top: core + cache + SDRAM + boot ROM + unified map; peripherals UART/CLINT/PLIC/SPI only (FFT/I2S/I2C/GPIO gone). Functional only — **no performance number is reported from this milestone** | branch | **Kernel-free gate, part 1:** `make regress` and `make compliance` relinked to run from SDRAM through the caches; quicksort and CoreMark Spike co-sim (the unified map now matches Spike's single address space); **memory torture test** — random-address, random-size, random-strobe load/store against a shadow model, sized to evict through every set and way, and run uncached-vs-cached; all of it on the board too | Loader-to-`main` time for the torture image |
| **M6** | **SDRAM burst reads** in the controller fork (8-beat line refill, one ACT, CL=3; writeback bursts if cheap) | cache repo + branch | **Measured** refill latency: cycles from miss detect to line commit, from a counter in `cache_cntrl` (sim, then read out on the board). Pass = the measured figure is recorded and at or below the agreed target (proposal: ≤ 20 core cycles for a clean miss, from ~50–70 [E] today). **Only after this: first CoreMark cycles/iteration on the Linux SoC**, with I/D miss counts | — |
| **M7** | `fence.i`: clean dirty D-lines, invalidate I-cache, kill buffer, refetch | branch | `rv32ui-fence_i` and `rv32uc-rvc` XPASS; self-modifying-code oracle (write code through D, `fence.i`, execute it); the torture test extended with code-write phases | — |
| **M8** | **Standard-compatible peripherals, no new drivers:** CLINT in the SiFive layout (`msip` +0x0, `mtimecmp` +0x4000, `mtime` +0xBFF8, one 64 KiB window); PLIC compatible with `sifive,plic-1.0.0` (per-source priority, pending +0x1000, enables +0x2000 + 0x80·ctx, threshold/claim-complete +0x200000 + 0x1000·ctx, gateway holding a claimed source until complete, contexts 0 = M and 1 = S); **16550-compatible UART** (`ns16550a`, `reg-shift = 2`, `reg-io-width = 4`); DT nodes for all of them | branch | Extended TBs (`plic_tb`, `uart_tb`, timer TB) check each register at the offset the mainline driver uses. Driver offsets are taken from the pinned 6.18 sources (`timer-clint.c`, `irq-sifive-plic.c`, `8250_of.c`) and quoted in the TB. Firmware oracles ported. OpenSBI's `fdt_serial`/`aclint`/`plic` drivers probe them in Spike-equivalent sim | — |
| **M9** | Boot ROM loader: UART (CRC-checked) first, then **SPI flash** via `axi4_lite_spi` | branch | Board: CoreMark loaded from SPI flash into SDRAM and run to `crcfinal=0x4983` | **Load time for a 1 MiB and a 4 MiB image, UART and SPI** |
| **M10** | **Route A gate:** 6.18 LTS `rv32_nommu_virt`-derived config + BusyBox initramfs, to a shell | branch | Shell prompt in Verilator (`NO_VCD`, long run) and on the board; Spike boots the same image with the same DT; **go/no-go review before Route B** | **Power-on to shell**, split into load / kernel / init |
| **M11** | Privilege: U/S, `medeleg`/`mideleg`, S-CSRs, `sret`, TSR/TVM/TW, ecall 8/9 | branch | `rv32si` enabled; remaining `rv32mi`; riscv-arch-test privilege suite where applicable; co-sim of S-mode test programs; PnR ×3 | nommu image still boots (regression), time reported |
| **M12** | Sv32: I/D TLB, hardware PTW, VIPT tag compare, `sfence.vma`, page faults 12/13/15, Svade | branch | riscv-tests **`v` environment** (`rv32ui`/`um`/`ua` re-run under paging); PTW corner-case oracle (superpages, permission and A/D faults); PnR ×3. **Most likely milestone to move the limiter** | nommu regression boot |
| **M13** | OpenSBI `fw_dynamic` (generic platform + DT) + 6.18 LTS MMU kernel + Buildroot to a shell | branch | Board boot log to a shell; a userspace program using `rdtime` and `fork` | **Power-on to shell**, split into load / OpenSBI / kernel / init |

[E] Calendar for one person: M0–M1 ~3–4 weeks, M2–M7 ~16–22 weeks (M4 and
M6 widen this range the most), M8–M10 ~6–8 weeks, M11–M13 ~15–18 weeks.

### 6.1 Cache LUT budget (derivation fixed now, numbers filled at M0/M4)

```
budget_cache = LUT_device × (1 − margin)
             − LUT_core(M2, no Zilx, Linux knobs)
             − LUT_peripherals(UART16550 + CLINT + PLIC + SPI + minimal GPIO + xbar + bridge)
             − LUT_bootrom_glue
```

- `LUT_device` = the GW2AR-18C's LUT4 count, from the GowinSynthesis
  resource report (the M0 report states it).
- `margin` = **25%**. Two reasons: routing is already 54–63% of every
  critical path at the frozen design's ~70% utilisation [E], and M11–M12 still
  have to fit their estimated +3–4.5k cells **inside** that margin. The
  margin is that reservation, not slack for the cache.
- `LUT_core` and `LUT_peripherals` come from the M0 GowinSynthesis report,
  split per hierarchy, minus the blocks the Linux branch drops (FFT, I2S,
  I2C, and the MCU's I-mem/D-mem glue).
- Fallback when the Gowin host is unreachable: yosys figures, scaled by the
  measured yosys/Gowin ratio for the same tree. If no ratio exists yet,
  use the raw yosys numbers, which overestimate (conservative), and mark the
  result provisional in `docs/linux_progress.md`.

### 6.2 Cache options (first-class alternatives)

| Option | Instruction side | Data side | `fence.i` | Relative logic [E] | CPI cost [E] |
|---|---|---|---|---|---|
| **C1** write-back (current subproject, reshaped) | 2-way I-cache | 2-way write-back D-cache | clean D + invalidate I | largest (dirty state, writeback path, victim buffer, store merge) | lowest |
| **C2** KianV-style + write-through | 2-way I-cache | 2-way **write-through** D-cache, no-write-allocate, posted store buffer of 1 entry | invalidate I only | medium: no dirty bits, no writeback FSM, no victim path | every store pays an SDRAM write (posted, so mostly hidden unless back-to-back) |
| **C3** KianV minimum | 2-way I-cache | **uncached** (every load to SDRAM, burst-free single word) | invalidate I only | smallest | every load pays ~6–8 SDRAM cycles [E] |

Rule: pick the **first option, in the order C1, C2, C3, that fits the §6.1
budget** with the 64-bit-data-macro reshaping applied. Record the choice
and the measured cells in the log. A C2/C3 choice can be revisited after
M10 if LUTs are left over.

### 6.3 Go/no-go criteria for the post-M10 review

These were fixed on 2026-11-22, **before any M0–M10 number existed**, so
the review cannot be fitted to the results. All figures are for the
Linux-SoC top at the end of M10, from GowinSynthesis/PnR where the host was
reachable, otherwise from the stated fallback, marked provisional.

| Criterion | GO | CONDITIONAL (go only with the stated mitigation planned) | NO-GO |
|---|---|---|---|
| **Free LUT4** after M10 | ≥ 25% of device (M11–M12 estimated +3–4.5k cells) | 15–25%: GO only if a named area cut covers the shortfall (predictor depth, Zicsr subset, C2→C3, smaller TLBs) | < 15% |
| Free FF | ≥ 20% | 10–20% | < 10% |
| Free BSRAM | ≥ 2 blocks | 1 | 0 |
| **Timing**: median worst setup slack over ≥ 3 seeds at the chosen Linux-SoC clock | ≥ +0.7 ns (one noise width) at ≥ 40 MHz | closes at 25–40 MHz, or at ≥ 40 MHz with < +0.7 ns | does not close at 25 MHz |
| Limiter location (endpoint list) | not in the cache tag-compare / hit path, where the TLB will sit | in the tag-compare path, with ≥ +2 ns there | in the tag-compare path, with < +2 ns |
| **Boot time**, nommu 6.18, power-on to shell, board | ≤ 5 min, loader included | 5–15 min | > 15 min |
| Boot reliability | 5/5 consecutive boots | 4/5 | ≤ 3/5 |
| Correctness | `regress`, `compliance` (updated XFAIL), torture test, cosims all green at the M10 commit | one documented, understood non-Linux-relevant failure | any unexplained failure |

Decision rule: **GO** if every row is GO. **NO-GO** if any row is NO-GO.
Otherwise **CONDITIONAL**, listing the mitigations. Rows that could not be
measured because the board or the Gowin host was unavailable are reported as
**UNMEASURED**. An UNMEASURED LUT, timing or boot row makes the verdict at
best CONDITIONAL.

### Decisions of 2026-11-22 (second review)

Unattended execution of M0–M10. After that, a stop and the §6.3 review. The
running log, with every decision taken without supervision, is
`docs/linux_progress.md`.

- CoreMark baseline: **GCC 14.3.0** image. Its board run is a "Needs Giacomo" item.
- PnR on 192.168.10.36, one directory per run and seed under `~/gowin_proj/`.
  The frozen run 8 reports are never overwritten.
- Cache self-test on the board: **assumed failing**.
- GPIO: **minimal GPIO kept** on the Linux branch (debug LEDs).
- Zilx: behind a parameter, **default off on the Linux branch**, on for `main`.
- UART: the highest BL616 rate verified stable, else 115200.
- Boot flash: **the on-board SPI configuration flash**, at an offset past the bitstream.
- Miss latency target: **≤ 20 core cycles** for a clean miss.
- Cache project: **git submodule pinned to a commit**.
- Branches: M1 → `main-fixes` (reviewed and merged by the owner); Linux
  work → `linux-soc`. Never commit to `main`.

---

## 7. What was verified vs estimated

Verified in this session: every row of §1 with a citation; the BSRAM/FF/DSP
figures in §4.1's first table (tool run); the frozen PnR summary and its
date; the cache subproject's status as written in its own `TODO.md`.

Estimated: all effort figures, all footprints in §3.6, cache hit/miss
latency, the LUT deltas, the CoreMark/MHz and Fmax for a Linux SoC, the
64-bit-macro BSRAM saving.

External, confirmed by two sources: RV32 nommu in mainline since 6.4
(release summary + the merged series), working on 6.18 LTS (NEORV32 port).
KianV's license (ISC, read from `LICENSE.md`), its Sv32 SoC's architecture
(multicycle, 2 TLBs, I-cache only), and the Tang Nano 20K appearing only in
its nommu SoC's board list.

External and only partly confirmed: Linux driver register layouts
(CLINT/PLIC/8250) — M8 takes them from the pinned 6.18 source; whether the
2024 nommu deprecation has advanced since 6.18; KianV's TLB sizes,
resources and Tang Nano 20K memory configuration (unpublished); SPI-flash
access from fabric.

---

## 8. Open questions

All questions of the first two reviews are answered: see §6, "Decisions of
2026-11-22". New ambiguities met during execution are resolved
conservatively and logged in `docs/linux_progress.md`, not asked.

---

## 9. External sources

- RISC-V nommu deprecation proposal and objections (2024):
  [LKML "riscv: deprecate CONFIG_MMU=n"](https://lkml.iu.edu/hypermail/linux/kernel/2402.3/02880.html),
  [follow-up](https://lkml.iu.edu/2403.3/04869.html)
- RV32 nommu support series (2023):
  [patchwork cover letter](https://patchwork.kernel.org/project/linux-riscv/cover/20230301002657.352637-1-Mr.Bossman075@gmail.com),
  [LKML v3](https://lkml.indiana.edu/2303.0/08098.html)
- K210 nommu, ~6 MiB usable: [LKML K230/K210 thread](https://lkml.iu.edu/hypermail/linux/kernel/2403.0/04909.html)
- RV32 nommu in mainline 6.4: [Linux 6.4 release summary](https://hardforum.com/threads/linux-6-4-release-main-changes-arm-risc-v-and-mips-architectures.2028947/),
  [v1 series incl. `rv32_nommu_virt_defconfig`](https://patchew.org/linux/20230119052642.1112171-1-Mr.Bossman075@gmail.com/)
- Vanilla nommu 6.18 LTS on an RV32IMAC soft core at 50 MHz:
  [see_neorv32_run_linux](https://github.com/14sea/see_neorv32_run_linux)
- KianV: [repository](https://github.com/splinedrive/kianRiscV),
  [LICENSE.md (ISC)](https://github.com/splinedrive/kianRiscV/blob/master/LICENSE.md),
  [Sv32 SoC README](https://github.com/splinedrive/kianRiscV/tree/master/linux_socs/kianv_mc_rv32ima_sv32),
  [Sv32 LICENSE.md](https://github.com/splinedrive/kianRiscV/blob/master/linux_socs/kianv_mc_rv32ima_sv32/LICENSE.md),
  [nommu "Harris mcycle" SoC README (lists tangNano20K)](https://github.com/splinedrive/kianRiscV/blob/master/linux_socs/kianv_harris_mcycle_edition/README.md),
  [tape-out write-up — HTTP 403, not read](https://www.crowdsupply.com/wafer-space/gf180mcu-run-2/updates/taping-out-kianv-a-linux-xv6-capable-risc-v-soc)
- Tang Nano 20K Linux demos: [CNX Software](https://www.cnx-software.com/2023/05/22/25-sipeed-tang-nano-20k-fpga-board-can-simulate-a-risc-v-core-run-linux-retro-games/),
  [linux-on-litex-vexriscv](https://github.com/enjoy-digital/linux-on-litex-vexriscv)
  (its board PR mentions 32 MB SDRAM — inconsistent with this board's 8 MiB,
  unresolved: [litex-boards PR 31](https://github.com/litex-hub/litex-boards/pull/31))
- Fabric access to the Tang 20K SPI flash: [tang_20k_spi_flash](https://github.com/ArthurHeymans/tang_20k_spi_flash)

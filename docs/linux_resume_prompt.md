# Resume prompt — Linux bring-up, after session 3 (2026-11-27)

Paste the block below into a fresh Claude Code session (after `/clear`).

---

Resume the unattended Linux bring-up on yarv-uc, branch `linux-soc`.

Read first, in this order:
1. memory `linux-bringup-state`;
2. `docs/linux_progress.md`: the "RESUME HERE" section and then everything
   under "Session 3" (it is newer and supersedes the RESUME HERE to-do list);
3. `docs/linux_assessment.md`: the milestone table (M5, M7, M10) and §6.3
   (go/no-go criteria, fixed before any result existed: do not refit them).

Where things stand (commit `6c73a32` plus the commit that adds this file):
- M6 (SDRAM bursts, clean miss 15/15/20 cycles), M7 (fence.i, cache side
  wired), M4b (cache 2 115 cells -nowidelut) are DONE and pinned: cache
  submodule `d26e8df` on its local branch `m6-burst`, nested
  sdram-controller `e190d36` on its branch `m6-burst`. Neither is pushed.
- The core gained a minimal U-mode (D24) and WFI now wakes on `mip & mie`
  (D25); both were found by booting Linux on the SoC sim. New oracle
  `sw/isa/umode_ops`.
- **M10 on the SoC sim works:** the 6.18.64 nommu kernel from
  `~/yarv-linux/soc/Image` boots on `sim/soc` to the BusyBox shell and
  answers typed commands (`uname -a`, `ls /`, `echo BOOT-OK`). `Run /bin/sh`
  at 0.743 s of kernel time (~37 M cycles at 50 MHz), ~5 min of wall time.
  How it was run (not yet a make target):
  - Image -> 32-bit word hex at word 0 (the python block in
    `sim/soc/elf2sdram.sh` does it on a raw binary);
  - `NO_VCD=1 INSTR_LOG=0 MAX_CYC=200000000 UART_BIT_CYCLES=432
    UART_RX_START=60000000 UART_RX=$'uname -a\nls /\necho BOOT-OK\n'
    sim/soc/obj_soc/Vsim_top +NOPROBE +SINIT=<linux.hex>
    +BINIT=sim/soc/bootrom_linux.hex`, run from a scratch directory (the
    console goes to `./sim_uart_tx.txt`).
- QEMU: same kernel to a shell in 8 MiB, ~12.3 M instructions to the prompt
  (`~/yarv-linux/NOTES.md` §3).
- Green at the last commit: BSRAM `make regress` 20/20, SoC regress
  (`sim/soc`, `make regress`) 18 passed / 1 skipped (ifault), compliance
  BSRAM 73 + 4 expected / SoC (`sim/soc`, `make compliance`) 75 + 2
  expected, cosims quicksort / ecall / CoreMark / amo_stress PASS vs Spike.
- Decisions logged up to D29. Continue from D30.

Remaining work, in order:
1. **Torture test (§6.3 correctness row).** `sim/sw/soc_torture/` exists
   and passes on the SoC (0x600D at 0x2000, ~9.5 M cycles; build with
   `make RISCV_PREFIX=...`, merge with `sim/soc/mkimage.py`). Still to do:
   wire it into `sim/soc/regress.py` as a SoC-only test (it uses SDRAM
   addresses that alias on BSRAM, so not in `sim/Makefile`'s list); prove it
   is not vacuous with a cache mutant applied to the build SNAPSHOT
   `sim/soc/snap/<pin>/src/rtl/cache_cntrl.sv`, never the submodule (e.g.
   never take the dirty-victim writeback at the `S_WB_READ` decision near
   line 1708), then delete that snapshot directory so it regenerates.
   Phase B (fence.i, then CACHE_BYPASS reads of the device) is the one that
   must catch it.
2. **Make the SoC Linux boot reproducible:** a `make linux` target in
   `sim/soc` (Image path as a variable defaulting to
   `~/yarv-linux/soc/Image`), pass = `BOOT-OK` echoed by the shell, report
   cycles to `Run /bin/sh` and to the prompt. Add `mount -t proc proc /proc`
   to the typed commands, then `cat /proc/meminfo` and `free`. Check in the
   console whether the kernel's misaligned-access emulation ever ran (QEMU
   never exercised it).
3. **SoC board top.** A previous agent stopped (API limit) after writing
   only `impl/soc_rtl.sh` and its output `impl/soc_rtl/` (a pinned,
   package-qualified copy of the cache RTL for the Gowin project; review it
   before use; it is uncommitted). Still to do: `src/rtl/soc/top_soc.sv`
   mirroring `sim/soc/sim_soc.sv` minus the sim models, with the rPLL from
   `top_module.sv`, reset sync on lock, double-flopped async inputs, the
   GW2AR embedded-SDRAM magic port names exactly as the cache repo's
   `fpga_top` uses them, and the M9 loader as the boot ROM
   (`sim/sw/bootrom/build/bootrom.hex`, path valid for gw_sh from the repo
   root). Plus a `.cst`, an `.sdc`, `linux_soc.gprj` (SDC as
   `<File type="file.sdc">`), and an `impl/remote_pnr_soc.sh` for the
   owner. Verilator `--lint-only` clean. SPI flash: the configuration-flash
   pins only if the documentation already in the repo settles them,
   otherwise header pins and NG-4 stays open.
4. **yosys estimate of the whole SoC top** (approach of
   `impl/yosys_estimate.sh`, with its documented traps), default and
   `-nowidelut`, against the device capacity, as PROVISIONAL rows for §6.3.
   Grep the log for memories falling back to flops.
5. **Spike boot** of the same Image (M10 acceptance asks for it): Spike's
   built-in devices sit at other addresses, so this needs a Spike-specific
   DT build (as the QEMU one is) or is recorded as not done, with the reason.
6. **CLAUDE.md**: durable lessons from session 3, kept short: WFI must
   wake on `mip & mie` independent of `mstatus.MIE`; nommu M-mode Linux
   needs U-mode; the SoC's CoreMark cost is now the cache HIT path (bursts
   moved CoreMark only 617 529 -> 610 201 cyc/iter).
7. **Go/no-go review**: write the §6.3 review in `docs/linux_assessment.md`,
   one row per criterion, each GO / CONDITIONAL / NO-GO / UNMEASURED with
   its evidence. LUT, FF, BSRAM and timing are UNMEASURED until GowinSynthesis
   and PnR run on the Gowin host (yosys figures are only provisional); boot
   time and reliability are UNMEASURED until the board runs. By the decision
   rule the verdict is then at best CONDITIONAL; state exactly what the owner
   must run to turn it into GO or NO-GO.
8. **Stop before M11** and wait for the owner.

Owner-only items (list them in the review, do not attempt): NG-1 PnR of the
SoC top on 192.168.10.36 (unreachable from this WSL machine) at >= 3 seeds
with the endpoint list; NG-2 board CoreMark; NG-3 push the cache repo and the
sdram-controller `m6-burst` branches (no GitHub credentials here); NG-4
configuration-flash pins; the board Linux boot, 5 consecutive times, with
power-on-to-shell time including the loader.

Rules: unattended, no questions; decisions in the progress log from D30;
few, large commits (progress-log updates ride with the code they describe);
`make regress` and the SoC regress green at every commit; never commit to
`main`; never touch `~/yarv32-native-caches` (the owner's own clone); work
in the submodule under `src/sub/` only when the cache itself must change;
toolchain prefix
`~/_toolchains/riscv32-ilp32d--glibc--stable-2025.08-1/bin/riscv32-buildroot-linux-gnu`;
Spike at `sim/cosim/riscv-isa-sim-install/bin/spike`; stop before 95% of the
session limit (the owner reports the percentage). Use parallel agents only
for independent work (items 3-4 are one agent; 1, 2 and 5 can run beside it).

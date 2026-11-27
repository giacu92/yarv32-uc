# Co-sim harness for sw/amo_stress (A extension), cloned from ../quicksort:
# the Verilator RTL sim, then diff per-retire architectural effects (pc +
# register write). See cosim_diff.py / build_spike.sh.
#
#   make cosim        full pipeline: build sw + Spike, run both, diff
#   make spike-build  build the local Spike (once; see build_spike.sh for deps)
#   make spike-run    run Spike -> spike.log
#   make rtl-run      run the Verilator sim -> rtl.trace (loads the sw hex)
#   make diff        diff spike.log vs rtl.trace
#   make clean        remove logs (keeps the Spike build)
#
# Spike is upstream riscv-isa-sim (built with --enable-commitlog); Zilx is
# already implemented upstream as the "zilx" ISA extension -- no patch.
#
# NOTE: the RTL sim must build. If decode_stage.sv has uncommitted WIP that
# breaks the build, fix it first -- cosim cannot run without a building sim.

SCRIPT_DIR := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))
SPIKE      := $(SCRIPT_DIR)../riscv-isa-sim-install/bin/spike
ELF        := $(SCRIPT_DIR)../../sw/amo_stress/build/program.elf
# Harvard: the C build emits a code image (-> I-mem) and a data image
# (-> D-mem), not a single program.hex. The RTL sim loads both.
IMEM_HEX   := $(SCRIPT_DIR)../../sw/amo_stress/build/imem.hex
DMEM_HEX   := $(SCRIPT_DIR)../../sw/amo_stress/build/dmem.hex
SPIKE_LOG  := $(SCRIPT_DIR)spike.log
RTL_LOG    := $(SCRIPT_DIR)rtl.trace
# The sim's stdout, which the run below would otherwise discard. The diff
# does not need it (that reads RTL_LOG), but it carries the stall-cause
# histogram and the CPI decomposition, and throwing those away means the
# one long run this harness already paid for says nothing about
# performance. Read the figures as belonging to the CO-SIM build of the
# firmware, which is not the timed build -- see the notes at the top.
RTL_STDOUT    := $(SCRIPT_DIR)rtl.stdout

# Bound the Spike run. Spike spins forever on the halt self-loop. Stop it
# with Spike's own --instructions cap: Spike exits cleanly after N commits
# (<1s), no pipe/zombie. We do NOT wrap in `timeout` -- on WSL `timeout`
# fails to reap a dead spike child, leaving `make` hung at this recipe even
# though spike.log is complete. `--instructions` makes the cap reliable on
# its own; `|| true` covers a spike crash. The program is a few thousand
# retires; the rest is the spinning `j`, park-truncated by the diff driver.
# The array is 256 words now, so the sort is roughly 30k instructions: the
# cap has to clear that plus Spike's boot stub, and the RTL bound has to
# clear the same work at IPC ~0.5.
SPIKE_MAXINSTR ?= 400000

# zicsr/zifencei: needed only for Spike's boot-ROM stub (csrr mhartid). The
# program emits no csr/fence; the RTL defers them (illegal=1) but never sees
# them because the stub is skipped by the diff driver. zilx: the indexed
# loads (same encoding as this core).
SPIKE_ISA := rv32imac_zicsr_zifencei_zilx
# Memory: 4 KiB at 0x0 (code + .data, below the 0x1000 boot ROM) + 8 KiB at
# 0x2000 (.rodata/.data/.bss + stack; sp=0x4000 grows down). The 0x1000
# boot-ROM gap avoids any overlap between DRAM and the boot-ROM device.
#
# The 8 KiB matches the real D-mem (native_ram ADDR_W=14 spanning
# 0x0000-0x3FFF, with the data image at 0x2000), so an access the hardware
# would silently alias makes Spike trap here instead of quietly succeeding
# and hiding the divergence.
SPIKE_MEM := 0x0:0x1000,0x2000:0x2000

.PHONY: cosim spike-build spike-run rtl-run diff clean

cosim: diff

spike-build:
	@bash $(SCRIPT_DIR)../build_spike.sh

# Build the C program (ELF for Spike, imem/dmem hex for the RTL).
#
# PRINT_ARRAY=0: the program prints the array before and after sorting on
# the board, but the first UART access is where Spike stops being
# comparable (no UART slave there), so a printing build would end the diff
# before the sort. Built unconditionally rather than only-if-missing,
# because the artefacts are shared with `make sw` -- which builds the
# printing variant.
# RISCV_PREFIX is forwarded when given, so `make cosim RISCV_PREFIX=...`
# works: sim/sw defaults to a toolchain path that does not exist on every
# machine, and without this the firmware build fails inside the co-sim.
TC_ARGS := $(if $(RISCV_PREFIX),RISCV_PREFIX=$(RISCV_PREFIX))

.PHONY: sw-cosim-build
sw-cosim-build:
	@$(MAKE) -C $(SCRIPT_DIR)../../sw/amo_stress clean
	@$(MAKE) -C $(SCRIPT_DIR)../../sw/amo_stress $(TC_ARGS)

$(ELF) $(IMEM_HEX) $(DMEM_HEX): sw-cosim-build

spike-run: $(ELF) spike-build
	@echo "== running Spike (golden) =="
	@rm -f $(SPIKE_LOG)
	@$(SPIKE) --isa=$(SPIKE_ISA) -m$(SPIKE_MEM) \
	    --instructions=$(SPIKE_MAXINSTR) --log-commits $(ELF) 2> $(SPIKE_LOG) \
	  || true
	@echo "spike: $$(wc -l < $(SPIKE_LOG)) commit lines -> $(SPIKE_LOG)"

rtl-run: $(IMEM_HEX) $(DMEM_HEX)
	@echo "== running RTL sim (RTL_TRACE=$(RTL_LOG)) =="
	@rm -f $(RTL_LOG)
	@RTL_TRACE=$(RTL_LOG) MAX_CYC=2000000 \
	  $(MAKE) --no-print-directory -C $(SCRIPT_DIR)../.. run \
	    RUN_ARGS="+IINIT=sw/amo_stress/build/imem.hex +DINIT=sw/amo_stress/build/dmem.hex" \
	  > $(RTL_STDOUT)
	@test -s $(RTL_LOG) || { echo "rtl: no trace produced (RTL_TRACE unset? sim_main.cpp edit missing?)"; exit 1; }
	@echo "rtl: $$(wc -l < $(RTL_LOG)) retires -> $(RTL_LOG)"
	@echo "rtl: stdout -> $(RTL_STDOUT)"
	@sed -n "/^retired /,\$$p" $(RTL_STDOUT)

diff: spike-run rtl-run
	@python3 $(SCRIPT_DIR)../cosim_diff.py --rtl $(RTL_LOG) --spike $(SPIKE_LOG)

clean:
	rm -f $(SPIKE_LOG) $(RTL_LOG) $(RTL_STDOUT)
# Shared build logic for Harvard firmware images.
#
# Included by every harness Makefile under sim/sw/{quicksort,isa,intr,peri}.
# The harness sets a few variables (below) and then `include`s this file; this
# defines the toolchain, flags, and the build rules that turn C / assembly into
# a pair of $readmemh images (build/imem.hex -> I-mem, build/dmem.hex -> D-mem).
#
# COMMON_DIR is derived from this file's own path, so it is correct regardless
# of how deep the harness that includes it sits.
#
# Harness-set variables (all optional unless noted):
#   ARCH            : -march string (default rv32imac_zicsr_zifencei).
#                     Plain-integer programs (quicksort, yarvmon) override to
#                     rv32imac; anything that emits CSR / mret / wfi / fence.i
#                     keeps the default.
#   C_SRCS          : C source filenames in the harness dir (empty for pure asm).
#   S_SRCS          : assembly filenames. With C_SRCS empty they are a
#                     standalone program carrying its own _start; alongside
#                     C_SRCS they are library assembly (dhrystone's strcmp)
#                     linked after the C objects, and start.S is still used.
#   START_S         : start.S path (default $(COMMON_DIR)/start.S).
#   LINK_LD         : link script (default $(COMMON_DIR)/link.ld).
#   IMEM_PAD_WORDS  : 0 = no padding (default); 2048 = pad the I-mem image to the
#                     full declared depth with IMEM_PAD_VALUE (ebreak). Board-fw
#                     images pad so GowinSynthesis sizes the inferred ROM from
#                     $readmemh content at the right depth. COUNT IS IN OUTPUT
#                     WORDS: the I-mem hex is 64-bit-wide (--word-width 8), so
#                     16 KiB = 2048 words (was 4096 at the old 32-bit width).
#   IMEM_PAD_VALUE  : filler word (default 0x00100073 = ebreak).
#   DMEM_BASE       : byte address of the first word of the data image
#                     (default 0x2000, the DMEM ORIGIN in common/link.ld).
#                     Must equal the ORIGIN of the DMEM region in whatever
#                     LINK_LD the harness uses -- objcopy -j starts the
#                     binary at the lowest kept VMA, and bin2hex turns this
#                     into the @ element index the D-mem hex loads at. Only
#                     dhrystone/ overrides it (0x0: its 10 KiB Arr_2_Glob
#                     does not fit in the 8 KiB above 0x2000).
#   OPT             : optimisation level (default -O2). CoreMark raises it.
#   EXTRA_CFLAGS    : extra -D flags, e.g. -DPRINT_ARRAY=$(PRINT_ARRAY).
#   EXTRA_LDFLAGS   : extra link flags, e.g. -Wl,--gc-sections. Placed
#                     BEFORE the objects, so it is not the place for -l.
#   EXTRA_LDLIBS    : libraries, placed AFTER the objects, where the linker
#                     will actually resolve them (dhrystone needs -lgcc for
#                     the soft-float helpers -nostdlib drops).
#
# Targets: make (all) -> imem.hex + dmem.hex + objdump; make show; make clean.

COMMON_DIR := $(patsubst %/,%,$(dir $(realpath $(lastword $(MAKEFILE_LIST)))))

# --- Toolchain -----------------------------------------------------------
# Default is the buildroot glibc toolchain (riscv32-buildroot-linux-gnu) -- a
# *linux* toolchain, hence the -fno-pie / -no-pie / -Wl,-N flags below are
# mandatory (its ld emits a PT_PHDR segment this flat MEMORY layout does not
# cover, and it defaults to PIE). Override RISCV_PREFIX to point at another
# rv32-capable toolchain (e.g. the PlatformIO/esphome bare-metal
# riscv32-esp-elf, riscv64-unknown-elf, riscv-none-elf) if preferred.
RISCV_PREFIX ?= $(HOME)/_toolchains/riscv32-zilx/out/bin/riscv32-unknown-elf

CC      := $(RISCV_PREFIX)-gcc
OBJCOPY := $(RISCV_PREFIX)-objcopy
OBJDUMP := $(RISCV_PREFIX)-objdump

ARCH ?= rv32imac_zicsr_zifencei
ABI  := ilp32

# -fno-pie: a linux/glibc toolchain (riscv32-buildroot-linux-gnu) defaults to PIE,
# which makes gas expand `la sym` into a GOT load (auipc + lw from .got) and gcc
# emit PC-relative data addressing. Both break the Harvard two-image layout:
# the GOT does not exist there, and link.ld requires medlow ABSOLUTE (lui + addi)
# so a data address computed in .text lands in D-mem. Without this, mtvec ends up
# loaded from an uninitialised .got word and the box reboot-loops.
#
# -no-pie -Wl,-N: the same linux ld emits a PT_PHDR segment this flat MEMORY
# layout does not cover ("PHDR segment not covered by LOAD segment"); -no-pie
# plus -N (omagic, single writable non-demand-paged LOAD) drops it. Harmless on
# a bare-metal toolchain.
#
# -I$(COMMON_DIR): lets C sources write `#include "uart.h"` against the shared
# header regardless of where the harness lives.
START_S ?= $(COMMON_DIR)/start.S
LINK_LD ?= $(COMMON_DIR)/link.ld
BIN2HEX := $(COMMON_DIR)/bin2hex.py

OPT ?= -O2

CFLAGS := -march=$(ARCH) -mabi=$(ABI) -nostdlib -nostartfiles -ffreestanding \
          -fno-builtin -fno-stack-protector -fomit-frame-pointer $(OPT) -Wall -g -fno-pie \
          -I$(COMMON_DIR) $(EXTRA_CFLAGS)
# Linker relaxation is ON. With --no-relax every `call` stays the two-instruction
# `auipc ra,X; jalr ra,off(ra)` form, which costs an instruction *and* an
# unpredictable indirect jump: the branch predictor computes PC-relative targets
# at decode, so a jal is predicted for free while a jalr is not predicted at all
# and eats a full ~3.1-cycle mispredict flush. Relaxing them to `jal` removed 110
# of Dhrystone's 111 static jalr/jr sites, cut its mispredicts 52% (31995 ->
# 15505) and its score 677 -> 651 cycles/iteration. gp-relative relaxation is
# inert here because no linker script defines __global_pointer$ and start.S never
# loads gp, so ld simply skips it -- do not add one without also setting gp.
LDFLAGS := -march=$(ARCH) -mabi=$(ABI) -nostdlib -nostartfiles -ffreestanding \
           -T $(LINK_LD) -Wl,--no-check-sections -no-pie -Wl,-N \
           $(EXTRA_LDFLAGS)

BUILD    ?= build
ELF      := $(BUILD)/program.elf
IMEM_BIN := $(BUILD)/imem.bin
DMEM_BIN := $(BUILD)/dmem.bin
IMEM_HEX := $(BUILD)/imem.hex
DMEM_HEX := $(BUILD)/dmem.hex
OBJD     := $(BUILD)/program.elf.objdump

IMEM_PAD_WORDS ?= 0
IMEM_PAD_VALUE ?= 0x00100073
DMEM_BASE      ?= 0x2000

# --- Size report ---------------------------------------------------------
# Printed at the end of every build: how much of each physical memory the
# image occupies. IMEM is the full 16 KiB fetch ROM. The D-mem's usable
# region for the image starts at DMEM_BASE and runs to the top of the
# 16 KiB RAM (0x4000), so DMEM "free" is also the room the stack has to
# grow in. .bss is NOBITS -- it is in the ELF but not in dmem.bin -- so
# its size is read from the section table and added to the image size.
IMEM_TOTAL := 16384
DMEM_TOTAL := $(shell echo $$(( 16384 - $(DMEM_BASE) )))
SIZE       := $(RISCV_PREFIX)-size

# Object list: C programs link the common start.o first (so _start / .text.init
# is the first thing linked -> IMEM 0x0); standalone .S programs carry their own
# _start and link alone.
#
# A C program may still name S_SRCS -- assembly that is a *library*, not an
# entry point (dhrystone/ links a hand-written strcmp). Those objects are
# appended after the C ones; only the pure-assembly case (no C_SRCS) treats
# S_SRCS as carrying its own _start.
ifeq ($(strip $(C_SRCS)),)
OBJS := $(patsubst %.S,$(BUILD)/%.o,$(S_SRCS))
else
OBJS := $(BUILD)/start.o $(patsubst %.c,$(BUILD)/%.o,$(C_SRCS)) \
        $(patsubst %.S,$(BUILD)/%.o,$(S_SRCS))
endif

# Rebuild on a flag change, not only on a source change. Several harnesses
# are built more than one way from the same directory (quicksort with and
# without PRINT_ARRAY, coremark with and without COSIM), and make sees only
# timestamps: after a co-sim build, a plain `make` finds every object newer
# than its source and reports success while leaving the co-sim variant in
# place. The stamp file records the varying part of the flags and is
# rewritten only when it changes, so the objects depend on the
# configuration as well as on the sources.
CFG_STAMP := $(BUILD)/.config
CFG_TEXT  := $(ARCH) $(OPT) $(EXTRA_CFLAGS) $(EXTRA_LDFLAGS) $(EXTRA_LDLIBS) $(RISCV_PREFIX) $(LINK_LD) $(DMEM_BASE)
# The stamp is written with make's $(file ...) rather than through a shell
# echo: EXTRA_CFLAGS routinely carries quotes and spaces (-DCOMPILER_FLAGS='...'),
# and passing that through sh mangles it. The shell only ever sees `cmp`.
$(shell mkdir -p $(BUILD))
$(file >$(CFG_STAMP).new,$(CFG_TEXT))
$(shell cmp -s $(CFG_STAMP).new $(CFG_STAMP) || cp $(CFG_STAMP).new $(CFG_STAMP); rm -f $(CFG_STAMP).new)

.PHONY: all clean show
all: $(IMEM_HEX) $(DMEM_HEX) $(OBJD)
	@iu=$$(stat -c %s $(IMEM_BIN)); \
	di=$$(stat -c %s $(DMEM_BIN)); \
	bss=$$($(SIZE) -A $(ELF) | awk '$$1==".bss"{s+=$$2} END{print s+0}'); \
	du=$$(( di + bss )); \
	echo "IMEM  $$iu/$(IMEM_TOTAL) B used, $$(($(IMEM_TOTAL)-iu)) B free ($$(awk -v t=$(IMEM_TOTAL) -v u=$$iu 'BEGIN{printf "%.1f", (t-u)*100/t}')% free)"; \
	echo "DMEM  $$du/$(DMEM_TOTAL) B used ($$di image + $$bss bss), $$(($(DMEM_TOTAL)-du)) B free ($$(awk -v t=$(DMEM_TOTAL) -v u=$$du 'BEGIN{printf "%.1f", (t-u)*100/t}')% free, shared with stack)"

$(BUILD):
	mkdir -p $(BUILD)

$(OBJS): $(CFG_STAMP)

$(BUILD)/start.o: $(START_S) | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

# EXTRA_LDLIBS comes after the objects: ld resolves an archive against the
# undefined symbols it has seen so far, so a -l ahead of them contributes
# nothing and the link fails on the symbol the archive holds.
$(ELF): $(OBJS) $(LINK_LD)
	$(CC) $(LDFLAGS) $(OBJS) $(EXTRA_LDLIBS) -o $@

# Instruction image: .text.init + .text -> IMEM (VMA 0). Fetch's read-only
# 64-bit port: bin2hex --word-width 8 packs two 32-bit instructions per
# $readmemh element (low 32 bits = word at the byte address, high = +4).
$(IMEM_BIN): $(ELF)
	$(OBJCOPY) -O binary -j .text.init -j .text $< $@

# Data image: .rodata + .data -> DMEM (VMA DMEM_BASE). .bss is NOBITS (no file
# content) and placed last, so it does not punch a gap. objcopy -j starts the
# binary at the lowest kept-section VMA (the DMEM ORIGIN), so bin2hex
# --base $(DMEM_BASE) emits the matching @ index and the words land at that
# D-mem offset -- @0x800 for the default 0x2000.
$(DMEM_BIN): $(ELF)
	$(OBJCOPY) -O binary -j .rodata -j .data $< $@

ifeq ($(IMEM_PAD_WORDS),0)
$(IMEM_HEX): $(IMEM_BIN) $(BIN2HEX)
	python3 $(BIN2HEX) --word-width 8 $< $@
else
# Padded so the inferred ROM is built that deep: GowinSynthesis sizes a
# read-only array from its $readmemh content, not from the declared depth, so an
# image-sized ROM lets every fetch above the image alias back into real
# instructions -- a wrong redirect then runs silently instead of faulting. The
# filler is ebreak, not zero: a run of zero words reads as "no init" and may be
# dropped again, and breakpoint (mcause=3) distinguishes a wander into the
# padding from illegal-instruction (mcause=2) on genuine garbage. The pad value
# is 32-bit and bin2hex replicates it across the 64-bit element.
$(IMEM_HEX): $(IMEM_BIN) $(BIN2HEX)
	python3 $(BIN2HEX) --word-width 8 --pad-words $(IMEM_PAD_WORDS) --pad-value $(IMEM_PAD_VALUE) $< $@
endif

$(DMEM_HEX): $(DMEM_BIN) $(BIN2HEX)
	python3 $(BIN2HEX) --base $(DMEM_BASE) $< $@

$(OBJD): $(ELF)
	$(OBJDUMP) -d $< > $@

show: $(ELF)
	$(OBJDUMP) -d $<

clean:
	rm -rf $(BUILD)
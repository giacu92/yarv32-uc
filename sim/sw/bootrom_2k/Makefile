# 2 KiB boot ROM -> imem.hex + dmem.hex.
#
# Prints "YARV32-uC bootrom" on the UART (115200 8N1 on the board) and parks.
#
#   make          build build/imem.hex + build/dmem.hex, then report the
#                 remaining boot-ROM space
#   make usage    the same report on its own
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/):
#   make run RUN_ARGS="+IINIT=sw/bootrom_2k/build/imem.hex \
#                      +DINIT=sw/bootrom_2k/build/dmem.hex"
#   The banner lands in sim/sim_uart_tx.txt.
#
# ONE MEMORY, NOT TWO. Unlike every other harness here, this image is not a
# Harvard pair: the boot ROM is a single 2 KiB memory serving both CPU ports,
# so bootrom_link.ld has one address space and code, rodata, data, bss and the
# stack all come out of the same 2 KiB. The two hex files below are two
# width-formattings of ONE identical image -- 64-bit elements for the fetch
# port, 32-bit for the LSU port -- which is why IMEM_SECTIONS and DMEM_SECTIONS
# are set to the same list and DMEM_BASE is 0 rather than 0x2000.
ARCH := rv32imac

LINK_LD := bootrom_link.ld

# sp = __stack_top (top of the 2 KiB), not the shared start.S's 0x4000, which
# would be 6 KiB past the end of this memory. See bootrom_start.S.
START_S := bootrom_start.S

IMEM_SECTIONS := -j .text.init -j .text -j .rodata -j .data
DMEM_SECTIONS := $(IMEM_SECTIONS)
DMEM_BASE     := 0x0

# Board firmware, so the image is padded to the full declared depth: the fetch
# port's hex is 64-bit-wide (two instructions per element), so 2 KiB = 256
# elements. GowinSynthesis sizes an inferred ROM from its $readmemh content
# rather than from the declared depth, and an image-sized ROM lets a fetch
# above the image alias back into real instructions instead of faulting.
IMEM_PAD_WORDS := 256

# -Os: the budget here is code size, and nothing in a banner print is timed.
OPT := -Os

EXTRA_CFLAGS  := -ffunction-sections -fdata-sections
EXTRA_LDFLAGS := -Wl,--gc-sections

C_SRCS := bootrom.c

include ../common/sw_build.mk

# --- Occupancy report ----------------------------------------------------
# Printed after every build, because the whole point of this program is that
# it fits. The link already fails on overflow (the ASSERTs in
# bootrom_link.ld); this is the gauge that shows the margin shrinking before
# it does. Sizes come from the linker's own symbols, so the report describes
# exactly what the ASSERTs check: __bss_end is the top of the image including
# the zeroed .bss, and STACK_SIZE below it is reserved, not free.
NM := $(RISCV_PREFIX)-nm

BOOTROM_SIZE  := 2048
BOOTROM_STACK := 256
# Link base of the ROM window, and it MUST match bootrom_link.ld's
# ORIGIN(MEM). The linker symbols below are absolute addresses, so the
# usage report subtracts this to get sizes; leave it stale and the report
# prints the ROM's address instead of its occupancy (8388968/2048 bytes,
# which is how the 0x0 -> 0x800000 relink was caught).
BOOTROM_BASE  := 0x00800000

all: usage

.PHONY: usage
usage: $(IMEM_HEX) $(DMEM_HEX) $(OBJD)
	@$(NM) $(ELF) | awk -v total=$(BOOTROM_SIZE) -v stack=$(BOOTROM_STACK) \
	    -v base=$$(printf '%d' $(BOOTROM_BASE)) '\
	    $$3 == "__text_end" { text = strtonum("0x" $$1) - base } \
	    $$3 == "__bss_end"  { img  = strtonum("0x" $$1) - base } \
	    END { \
	        used = img + stack; \
	        free = total - used; \
	        printf "bootrom: %d/%d bytes used (%.1f%%), %d free (%.1f%% remaining)\n", \
	               used, total, 100.0 * used / total, free, 100.0 * free / total; \
	        printf "         .text %d, image incl .bss %d, reserved stack %d\n", \
	               text, img, stack }'

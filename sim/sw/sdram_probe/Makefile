# Board diagnostic boot ROM (sdram_probe): runs from ROM + registers only and
# prints six SDRAM tests over the 16550. Put build/bootrom.hex in place of the
# loader's image (top_soc BOOTROM_FILE) for one diagnostic bitstream.
RISCV_PREFIX ?= $(HOME)/_toolchains/riscv32-ilp32d--glibc--stable-2025.08-1/bin/riscv32-buildroot-linux-gnu

all: build/bootrom.hex

build/probe.elf: probe.S
	mkdir -p build
	$(RISCV_PREFIX)-gcc -march=rv32ima_zicsr_zifencei -mabi=ilp32 -nostdlib -nostartfiles \
	    -no-pie -Wl,-N -Wl,-Ttext=0x00800000 -o $@ $<

build/bootrom.hex: build/probe.elf ../bootrom/bin2rom.py
	$(RISCV_PREFIX)-objcopy -O binary $< build/probe.bin
	python3 ../bootrom/bin2rom.py build/probe.bin $@ 2048

.PHONY: all

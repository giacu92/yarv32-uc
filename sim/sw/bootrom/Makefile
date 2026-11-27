# Boot ROM image for the Linux SoC (M9): yload, the UART loader.
# Output build/bootrom.hex: 64-bit words for the cache's 2 KiB boot ROM
# (cache_cntrl BOOTROM_FILE / sim_soc +BINIT). The build fails if the loader
# outgrows the ROM.
RISCV_PREFIX ?= $(HOME)/_toolchains/riscv32-ilp32d--glibc--stable-2025.08-1/bin/riscv32-buildroot-linux-gnu

all: build/bootrom.hex

build/yload.elf: yload.S
	mkdir -p build
	$(RISCV_PREFIX)-gcc -march=rv32ima_zicsr_zifencei -mabi=ilp32 -nostdlib -nostartfiles \
	    -no-pie -Wl,-N -Wl,-Ttext=0x00800000 -o $@ $<
	$(RISCV_PREFIX)-objdump -d $@ > build/yload.dump

build/bootrom.hex: build/yload.elf bin2rom.py
	$(RISCV_PREFIX)-objcopy -O binary $< build/yload.bin
	python3 bin2rom.py build/yload.bin $@ 2048

.PHONY: all

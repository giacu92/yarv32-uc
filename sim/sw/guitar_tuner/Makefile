# Guitar tuner: SPI ADC -> FFT coprocessor -> SSD1306 needle gauge.
#
#   make          board build   -> build/imem.hex + build/dmem.hex
#   make sim      self-check    -> build-sim/imem.hex + build-sim/dmem.hex
#   make show     disassemble the board build
#   make clean    remove both build directories
#
# The board build drives real hardware: an MCP3208 on the SPI master (audio
# biased to Vref/2 into CH0) and an SSD1306 128x64 at 0x3C on the I2C
# master. Neither is modelled in the simulator.
#
# `make sim` (-DTUNER_SIM=1) swaps the ADC for a synthetic oscillator and
# the display for the UART. It checks the reported note and cents against a
# table of known frequencies, that silence reads as silence, and that the
# CIC decimator rejects an out-of-band tone by at least 18 dB. It exercises
# the whole chain except the two peripherals -- decimator, window, FFT
# hardware, peak pick, interpolation, note maths -- and writes
# 0x600D / 0xBAD to D-mem 0x3000 like the other oracles:
#
#   cd sim && make run RUN_ARGS="+IINIT=sw/guitar_tuner/build-sim/imem.hex \
#                                +DINIT=sw/guitar_tuner/build-sim/dmem.hex"
#   -> probe[0x3000] = 0x600d
#
# Needs Zicsr: the sample clock is a busy-wait on mcycle.

ARCH := rv32imac_zicsr_zifencei

C_SRCS := guitar_tuner.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../common/sw_build.mk

# The default target builds BOTH images: the board one in build/ and the
# self-check in build-sim/, so `make -C sim/sw` and `make regress` need no
# special case. The guard is what stops the recursion -- the sub-make runs
# with BUILD=build-sim on its command line, and there `all` must not pull
# `sim` in again.
.PHONY: sim clean-sim
ifneq ($(BUILD),build-sim)
all: sim
endif

sim:
	$(MAKE) BUILD=build-sim EXTRA_CFLAGS="-DTUNER_SIM=1" all

clean-sim:
	$(MAKE) BUILD=build-sim clean

clean: clean-sim

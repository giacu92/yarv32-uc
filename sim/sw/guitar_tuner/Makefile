# Guitar tuner: INMP441 I2S mic -> FFT coprocessor -> SSD1306 needle gauge.
#
#   make          board build   -> build/imem.hex + build/dmem.hex
#   make sim      self-check    -> build-sim/imem.hex + build-sim/dmem.hex
#   make show     disassemble the board build
#   make clean    remove both build directories
#
# The board build drives real hardware: an INMP441 I2S microphone on the
# I2S pins (SCK->PIN73, WS->PIN74, SD->PIN75, L/R->GND) with the peripheral
# in MASTER mode -- the mic is itself a clock slave, so the FPGA has to
# clock it -- and an SSD1306 128x64 at 0x3C on the I2C master. Neither
# device is modelled in the simulator.
#
# `make sim` (-DTUNER_SIM=1) swaps the mic for a synthetic oscillator and
# the display for the UART. It still checks the I2S front end against the
# sim's INMP441-shaped model (rate, ordering, frame period) -- what it
# cannot do is play a note into it. It checks the reported note and cents against a
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
# special case, and `clean` removes both.
#
# BOTH hooks have to sit inside the guard, and the second one was missing
# until 2026-09-16: the sub-make runs with BUILD=build-sim on its command
# line, so an unguarded `clean: clean-sim` re-entered `make BUILD=build-sim
# clean` from inside itself and recursed until make gave up. `all: sim` had
# the guard from the start, which is exactly why the omission on `clean`
# was easy to miss.
.PHONY: sim clean-sim
ifneq ($(BUILD),build-sim)
all: sim
clean: clean-sim
endif

sim:
	$(MAKE) BUILD=build-sim EXTRA_CFLAGS="-DTUNER_SIM=1" all

clean-sim:
	$(MAKE) BUILD=build-sim clean

# GPIO0 blink, 1 Hz (500 ms on / 500 ms off) via common/gpio.h + timer.h.
# Board bring-up firmware: on the board GPIO0 = PIN41 and also drives
# LED0 (top_module wires it there). The loop never ends, so in the sim
# the run goes to MAX_CYC instead of parking -- expected, not a hang.
#
#   make          build build/imem.hex + build/dmem.hex
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/), bounded, just to see the banner:
#   make run RUN_ARGS="+IINIT=sw/peri/gpio_blink/build/imem.hex +DINIT=sw/peri/gpio_blink/build/dmem.hex" MAX_CYC=200000

ARCH := rv32imac_zicsr_zifencei

C_SRCS := gpio_blink.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../../common/sw_build.mk
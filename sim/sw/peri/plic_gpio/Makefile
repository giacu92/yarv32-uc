# GPIO + PLIC end-to-end MEIP oracle: arms a rising-edge interrupt on GPIO
# pin0 (driven by the firmware itself through the sim's loopback / the board's
# own pad), takes the machine external interrupt through the PLIC, and checks
# the cause/claim contract inside the handler (CLAIM must read
# PLIC_SRC_GPIO=4). Also pins the reset consequence: pulled-up pads + level
# INT_TYPE reset leave stale INT_STATUS bits, so firmware must W1C before
# arming edges. Writes 0x600D / 0xBAD to D-mem 0x3000.
#
#   make          build build/imem.hex + build/dmem.hex
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/):
#   make run RUN_ARGS="+IINIT=sw/peri/plic_gpio/build/imem.hex +DINIT=sw/peri/plic_gpio/build/dmem.hex"
#   -> probe[0x3000] = 0x600d
#
# Needs Zicsr (programs mtvec/mie/mstatus, runs an interrupt handler).

ARCH := rv32imac_zicsr_zifencei

C_SRCS := plic_gpio.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../../common/sw_build.mk
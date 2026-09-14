# I2C bus scanner: probes 0x08..0x77, prints and records what ACKs.
# In the sim (single i2c_slave_model at 0x50) it is an oracle -- exactly one
# ACK at 0x50 writes 0x600D to D-mem 0x3000; on the board it is a plain
# diagnostic and the marker is meaningless.
#
#   make          build build/imem.hex + build/dmem.hex
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/):
#   make run RUN_ARGS="+IINIT=sw/peri/i2c_scan/build/imem.hex +DINIT=sw/peri/i2c_scan/build/dmem.hex" MAX_CYC=5000000
#   -> serial log "I2C SCAN", "ACK 0x50", "FOUND 1", "GOOD"

ARCH := rv32imac_zicsr_zifencei

C_SRCS := i2c_scan.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../../common/sw_build.mk
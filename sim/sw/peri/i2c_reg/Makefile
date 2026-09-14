# I2C register oracle: the common/i2c.h library against sim_top's
# i2c_slave_model (addr 0x50, 16 regs, auto-incrementing pointer). Exercises
# write_reg/read_reg, bursts, a 20-byte write past the TX FIFO depth, probe
# and NACK error returns. Writes 0x600D / 0xBAD to D-mem 0x3000.
#
#   make          build build/imem.hex + build/dmem.hex
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/):
#   make run RUN_ARGS="+IINIT=sw/peri/i2c_reg/build/imem.hex +DINIT=sw/peri/i2c_reg/build/dmem.hex" MAX_CYC=1000000
#   -> serial log "I2C REG" + per-check OK/FAIL lines ending GOOD, probe 0x3000 = 0x600d
#
# Board-fw style: pad the I-mem image to the full declared depth with ebreak.

ARCH := rv32imac_zicsr_zifencei

C_SRCS := i2c_reg.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../../common/sw_build.mk
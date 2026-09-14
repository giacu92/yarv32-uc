# SPI loopback oracle: the common/spi.h library against sim_top's SPI
# loopback (MISO = MOSI). Exercises single transfers, buffered exchanges at
# and past the FIFO depth, send/read forms, 16/32-bit helpers and all four
# CPOL/CPHA modes. Writes 0x600D / 0xBAD to D-mem 0x3000.
#
#   make          build build/imem.hex + build/dmem.hex
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/):
#   make run RUN_ARGS="+IINIT=sw/peri/spi_loop/build/imem.hex +DINIT=sw/peri/spi_loop/build/dmem.hex" MAX_CYC=400000
#   -> serial log "SPI LOOP" + per-check OK/FAIL lines ending GOOD, probe 0x3000 = 0x600d
#
# Board-fw style: pad the I-mem image to the full declared depth with ebreak.

ARCH := rv32imac_zicsr_zifencei

C_SRCS := spi_loop.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../../common/sw_build.mk
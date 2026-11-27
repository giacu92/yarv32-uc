# Memory torture test for the Linux SoC (cache + SDRAM). SoC only: it uses
# SDRAM addresses far above the 16 KiB BSRAM, which alias there.
# 0x600D / 0xBAD @0x2000; on failure the phase @0x2004, the address @0x2008,
# got @0x200C, expected @0x2010.
ARCH := rv32imac_zicsr_zifencei
C_SRCS := main.c
include ../common/sw_build.mk

# SD card (SPI mode) test for the Linux SoC sim (M9b). Needs the SoC sim run
# with +SDCARD +SDINIT=<image> (sim/soc/regress.py generates the image: word
# i = sd_hash(i), see main.c). SoC only: the SD model is not in sim/sim_top.
# 0x600D / 0xBAD @0x2000; on failure the phase @0x2004, detail @0x2008,
# got @0x200C, expected @0x2010. @0x2014 = CCS seen in the OCR,
# @0x2018 = ACMD41 calls, both written on success too.
ARCH := rv32imac_zicsr_zifencei
C_SRCS := main.c
# SPI clock after init = clk / (2*(FAST_DIV+1)).
FAST_DIV ?= 1
EXTRA_CFLAGS := -DFAST_DIV=$(FAST_DIV)
include ../common/sw_build.mk

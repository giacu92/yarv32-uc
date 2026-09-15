# I2S receiver end-to-end oracle: the common/i2s.h library against the
# sim's i2s_master_model, through the real LSU / bridge / crossbar path.
# Checks the window decode, the stereo stream (values, channel tags and
# pairing against a self-describing sequence), the channel filter, the
# watermark and sticky overrun, and the RX interrupt through the PLIC as
# source 6 with a wfi wake.
#
#   make          build build/imem.hex + build/dmem.hex
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/):
#   make run RUN_ARGS="+IINIT=sw/peri/i2s/build/imem.hex +DINIT=sw/peri/i2s/build/dmem.hex" MAX_CYC=400000
#   -> probe[0x3000] = 0x600d
#
# Framing, word lengths, sign extension and the FIFO corner cases are
# sim/hw/i2s_tb's job -- it bit-bangs the audio bus and can produce frames
# a real master cannot.
#
# Needs Zicsr (programs mtvec/mie/mstatus, runs an interrupt handler).

ARCH := rv32imac_zicsr_zifencei

C_SRCS := i2s_rx.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../../common/sw_build.mk

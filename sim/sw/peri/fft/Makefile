# FFT coprocessor end-to-end oracle: register plumbing through the real
# LSU/bridge/crossbar path, the 8 KiB two-page window decode, two exactly
# predictable transforms (impulse and DC), the streaming ping-pong (frame
# B written while frame A is transforming), and the DONE interrupt through
# the PLIC as source 5 with a wfi wake.
#
#   make          build build/imem.hex + build/dmem.hex
#   make show     disassemble build/program.elf
#   make clean    remove build/
#
# Run in the sim (from sim/):
#   make run RUN_ARGS="+IINIT=sw/peri/fft/build/imem.hex +DINIT=sw/peri/fft/build/dmem.hex"
#   -> probe[0x3000] = 0x600d
#
# Numeric accuracy is NOT this test's job -- sim/hw/fft_tb checks every
# transform against a bit-exact datapath model and a double-precision DFT.
#
# Needs Zicsr (programs mtvec/mie/mstatus, runs an interrupt handler).

ARCH := rv32imac_zicsr_zifencei

C_SRCS := fft.c

IMEM_PAD_WORDS := 2048   # 16 KiB / 8 = 2048 64-bit words

include ../../common/sw_build.mk

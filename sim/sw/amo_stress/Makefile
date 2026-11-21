# Atomic-instruction stress program for the Spike cosim (sim/cosim/amo_stress).
# A pseudo-random sequence of LR/SC loops and AMOs over a small array, each
# checked against a plain-C reference; main() returns 0x600D / 0xBAD in a0.
# No MMIO, so Spike can compare every retire.
ARCH := rv32imac
C_SRCS := main.c
include ../common/sw_build.mk

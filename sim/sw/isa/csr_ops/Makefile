# CSR oracle: architected values and behaviour of the machine CSRs this core
# implements, checked by value rather than by "did it trap". Writes 0x600D /
# 0xBAD @0x2000 and, on failure, the number of the failing check @0x2004.
#
# Why it exists: unimplemented CSRs used to read 0 without trapping, so the
# upstream zicntr test (which only checks that cycle/instret reads do not
# trap) passed while cycle/time/instret all read 0, and misa advertised the
# A extension the decoder does not implement. Nothing in the tree checked a
# CSR's VALUE.
#
#   make          build build/imem.hex + build/dmem.hex
# Run (from sim/):
#   PROBE=0x2000 ./obj_dir/Vsim_top +IINIT=sw/isa/csr_ops/build/imem.hex \
#       +DINIT=sw/isa/csr_ops/build/dmem.hex
#
# Pure assembly with its own _start; no toolchain runtime.

S_SRCS := csr_ops_test.S

include ../../common/sw_build.mk

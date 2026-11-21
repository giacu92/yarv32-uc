# A-extension oracle (LR.W / SC.W / the nine AMO*.W). Writes 0x600D / 0xBAD
# @0x2000; on failure the failing check number @0x2004 and the last two
# values compared @0x2008 / @0x200C.
#
# Why it exists: the core had no A extension until 2026-11-22, so nothing in
# the tree executed an atomic. It checks rd (the old value), the memory
# word afterwards (the new value), every signedness corner of MIN/MAX,
# reservation behaviour (SC with no LR, a second SC, an SC to another word,
# an SC across a trap), the refusals (misaligned -> cause 4/6, an atomic to
# the peri port -> access fault 5/7 with mtval = the address), and the
# distance-1 forward path into and out of an AMO.
#
#   make     build build/imem.hex + build/dmem.hex
# Pure assembly with its own _start; no toolchain runtime.

S_SRCS := amo_ops_test.S

include ../../common/sw_build.mk

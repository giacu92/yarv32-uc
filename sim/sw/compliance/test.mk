# Build fragment for ONE riscv-tests ISA test. Not invoked directly -- the
# compliance Makefile recurses into it once per test:
#
#   make -f test.mk SUITE=rv32ui TEST=add
#
# It exists because sw_build.mk builds a single program out of the directory
# it is included from, and this harness builds ~70 of them out of a vendored
# source tree that must not be copied or edited. Two knobs make that work:
# BUILD is overridden to a per-test directory, and VPATH points the pattern
# rule at the vendored suite directory.
#
# ARCH deliberately does NOT include the C extension, matching upstream's
# `-march=rv32g` for every rv32 suite: with C enabled the assembler
# auto-compresses eligible instructions, so a test named `add` would end up
# exercising `c.add`. The one suite that is about compressed encodings,
# rv32uc/rvc.S, turns C on itself with `.option rvc` around each case, which
# works regardless of -march. Zicsr/Zifencei are in because the test
# environment reads and writes mtvec/mepc/mcause and the sources emit fence.i.
#
# The include search path order matters: a quoted #include is resolved against
# the including file's own directory first, which is what lets the vendored
# rv32ui/add.S reach ../rv64ui/add.S while riscv_test.h and test_macros.h come
# from -I. env/ is OURS (the Harvard M-mode environment); riscv-tests/env
# supplies upstream's encoding.h; macros/scalar supplies test_macros.h.

ifeq ($(strip $(SUITE))$(strip $(TEST)),)
$(error test.mk needs SUITE=<rv32ui|rv32um|rv32uc|rv32mi> and TEST=<name>)
endif

BUILD   := build/$(SUITE)-$(TEST)
S_SRCS  := $(TEST).S
VPATH   := riscv-tests/isa/$(SUITE)

ARCH    := rv32ima_zicsr_zifencei
LINK_LD := env/link_harvard.ld

EXTRA_CFLAGS := -Ienv -Iriscv-tests/env -Iriscv-tests/isa/macros/scalar

include ../common/sw_build.mk

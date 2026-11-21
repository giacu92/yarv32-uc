# riscv-tests ISA suite, built for this Harvard RV32IMAC core.
#
#   make                    build every test in every enabled suite
#   make SUITES=rv32ui      build one suite
#   make list               show what would be built
#   make verify-upstream    fail if the vendored riscv-tests tree was edited
#   make clean              remove build/
#
# Then run them from sim/:
#   make -C .. -f Makefile compliance          (see sim/Makefile)
#
# Toolchain: sw_build.mk's default RISCV_PREFIX does not exist on every
# machine. Override it, e.g.
#   make RISCV_PREFIX=$(HOME)/_toolchains/riscv32-ilp32d--glibc--stable-2025.08-1/bin/riscv32-buildroot-linux-gnu
#
# WHY A SEPARATE HARNESS AND NOT sw/isa/: the sources under riscv-tests/ are
# vendored verbatim from upstream and are never edited -- the whole value of a
# compliance suite is that it is the same code everyone else runs. Everything
# this core needs in order to run them lives outside that tree, in env/
# (the M-mode Harvard test environment + linker script) and in test.mk.
#
# SUITES ENABLED, AND WHY THE OTHERS ARE NOT:
#   rv32ui  base integer                     -- enabled
#   rv32um  M extension                      -- enabled
#   rv32uc  compressed                       -- enabled
#   rv32mi  machine-mode traps/CSRs          -- enabled
#   rv32ua  atomics       -- enabled on linux-soc (2026-11-22), which
#           implements LR/SC + AMOs and turns Zilx off. On `main` the AMO
#           opcode is Zilx's and every real AMO is illegal, so it stays off
#           there.
#   rv32si  supervisor    -- NOT RUNNABLE. M-mode only, no delegation, no MMU.
#   rv32uf/ud/uzb*/up     -- NOT RUNNABLE. No F/D, no bitmanip, no packed SIMD.
#
# Individual tests inside the enabled suites are still expected to fail where
# the core deliberately does not implement something; those are listed as
# expected failures in sim/Makefile's `compliance` target, not skipped here,
# so a change in their status is visible instead of hidden.

SUITES ?= rv32ui rv32um rv32ua rv32uc rv32mi

# One entry per test, "suite/name", discovered from the vendored tree so a
# suite gaining a test upstream is picked up by re-vendoring alone.
suite_tests = $(patsubst riscv-tests/isa/$(1)/%.S,$(1)/%,$(wildcard riscv-tests/isa/$(1)/*.S))
TESTS := $(foreach s,$(SUITES),$(call suite_tests,$(s)))

.PHONY: all list clean verify-upstream $(TESTS)

all: $(TESTS)

# Recursive, one make per test: sw_build.mk is written around a single
# program (fixed variable names, one BUILD tree, one .config stamp), so the
# only honest way to reuse it N times is N invocations.
$(TESTS):
	@$(MAKE) --no-print-directory -f test.mk \
	    SUITE=$(patsubst %/,%,$(dir $@)) TEST=$(notdir $@) \
	    $(if $(RISCV_PREFIX),RISCV_PREFIX=$(RISCV_PREFIX),)

list:
	@for t in $(TESTS); do echo $$t; done

verify-upstream:
	@cd riscv-tests && sha256sum --quiet -c ../riscv-tests.sha256 \
	  && echo "riscv-tests: vendored tree matches UPSTREAM.md" \
	  || { echo "riscv-tests: VENDORED TREE MODIFIED -- see UPSTREAM.md"; exit 1; }

clean:
	rm -rf build

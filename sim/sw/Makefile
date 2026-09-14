# Aggregator for all sim/sw firmware. One command builds every program and
# every test oracle:
#
#   make            build quicksort + every isa/intr/peri test
#   make clean      clean all
#   make quicksort  build one group/program
#                   (quicksort | coremark | dhrystone | yarvmon | sha256 |
#                    guitar_tuner | isa | intr | peri)
#
# Each leaf harness has its own Makefile (see common/sw_build.mk for the shared
# build logic); this just recurses into each subdir.

GROUPS := quicksort coremark dhrystone yarvmon sha256 guitar_tuner isa intr peri

.PHONY: all clean $(GROUPS)
all:
	@for d in $(GROUPS); do $(MAKE) -C $$d || exit 1; done

$(GROUPS):
	$(MAKE) -C $@

clean:
	@for d in $(GROUPS); do $(MAKE) -C $$d clean; done
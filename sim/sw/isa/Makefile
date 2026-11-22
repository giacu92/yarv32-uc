# ISA decode/fetch oracles. Builds every test in this group with one command.
#
#   make            build every oracle in the group
#   make clean      clean all
#   make ifault     build one test

TESTS := amo_ops bp_pred csr_ops div_ops fencei_ops ifault ifault_hold isa_probe mul_ops rvc_scramble span_target  

.PHONY: all clean $(TESTS)
all:
	@for t in $(TESTS); do $(MAKE) -C $$t || exit 1; done

$(TESTS):
	$(MAKE) -C $@

clean:
	@for t in $(TESTS); do $(MAKE) -C $$t clean; done
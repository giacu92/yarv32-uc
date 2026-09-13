# Peripheral / integration oracles. Builds every test in this group with one command.
#
#   make            build uart_echo
#   make clean      clean all

TESTS := uart_echo plic_gpio

.PHONY: all clean $(TESTS)
all:
	@for t in $(TESTS); do $(MAKE) -C $$t || exit 1; done

$(TESTS):
	$(MAKE) -C $@

clean:
	@for t in $(TESTS); do $(MAKE) -C $$t clean; done
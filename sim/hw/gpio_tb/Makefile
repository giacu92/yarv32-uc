# Verilator axi4_lite_gpio reg / edge / level / IRQ compliance test.
#
#   make          build the test binary (obj_dir/Vgpio_tb)
#   make run      build + run, prints the compliance check results
#   make clean    remove build artefacts
#
# Requires: verilator, g++.

RTL := \
	../../../src/rtl/pkg/rv32_pkg.sv \
	../../../src/rtl/bus/axi4_lite_if.sv \
	../../../src/rtl/utils/axi4_lite_gpio.sv \
	gpio_tb.sv

VFLAGS := --cc --exe --build --top-module gpio_tb \
	-Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	-I../../../src/rtl/pkg -I../../../src/rtl/bus -I../../../src/rtl/utils

TOP := obj_dir/Vgpio_tb

$(TOP): gpio_tb.cpp $(RTL)
	verilator $(VFLAGS) gpio_tb.cpp $(RTL)

run: $(TOP)
	./$(TOP)

clean:
	rm -rf obj_dir *.vcd *.fst *.log

.PHONY: run clean
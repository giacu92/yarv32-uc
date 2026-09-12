# Verilator axi4_lite_spi mode + FIFO + IRQ compliance test.
#
#   make          build the test binary (obj_dir/Vspi_tb)
#   make run      build + run, prints the compliance check results
#   make clean    remove build artefacts
#
# Requires: verilator, g++.

RTL := \
	../../../src/rtl/pkg/rv32_pkg.sv \
	../../../src/rtl/bus/axi4_lite_if.sv \
	../../../src/rtl/utils/axi4_lite_spi.sv \
	spi_tb.sv

VFLAGS := --cc --exe --build --top-module spi_tb \
	-Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	-I../../../src/rtl/pkg -I../../../src/rtl/bus -I../../../src/rtl/utils

TOP := obj_dir/Vspi_tb

$(TOP): spi_tb.cpp $(RTL)
	verilator $(VFLAGS) spi_tb.cpp $(RTL)

run: $(TOP)
	./$(TOP)

clean:
	rm -rf obj_dir *.vcd *.fst *.log

.PHONY: run clean
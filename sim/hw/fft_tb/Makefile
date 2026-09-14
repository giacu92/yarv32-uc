# Verilator axi4_lite_fft compliance test: register contract, ping-pong
# ownership, interrupt, and every transform checked twice -- against a
# bit-exact model of the datapath AND against a double-precision DFT.
#
#   make          build the test binary (obj_dir/Vfft_tb)
#   make run      build + run, prints the compliance check results
#   make clean    remove build artefacts
#
# Requires: verilator, g++.

RTL := \
	../../../src/rtl/pkg/rv32_pkg.sv \
	../../../src/rtl/bus/axi4_lite_if.sv \
	../../../src/rtl/utils/fft_dpram.sv \
	../../../src/rtl/utils/fft_twiddle_rom.sv \
	../../../src/rtl/utils/axi4_lite_fft.sv \
	fft_tb.sv

VFLAGS := --cc --exe --build --top-module fft_tb \
	-Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	-I../../../src/rtl/pkg -I../../../src/rtl/bus -I../../../src/rtl/utils

TOP := obj_dir/Vfft_tb

$(TOP): fft_tb.cpp $(RTL)
	verilator $(VFLAGS) fft_tb.cpp $(RTL)

run: $(TOP)
	./$(TOP)

clean:
	rm -rf obj_dir *.vcd *.fst *.log

.PHONY: run clean

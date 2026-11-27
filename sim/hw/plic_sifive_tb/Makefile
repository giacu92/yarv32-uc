# linux-soc M8: axi4_lite_plic_sifive register/behaviour test against the offsets the
# mainline driver uses. make run -> prints the check count, exits non-zero on
# any failure.
RTL := \
	../../../src/rtl/pkg/rv32_pkg.sv \
	../../../src/rtl/bus/axi4_lite_if.sv \
	../../../src/rtl/utils/axi4_lite_plic_sifive.sv \
	plic_sifive_tb.sv

VFLAGS := --cc --exe --build --top-module plic_sifive_tb \
	-Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	-I../../../src/rtl/pkg -I../../../src/rtl/bus -I../../../src/rtl/utils -CFLAGS -I$(CURDIR)/..

TOP := obj_dir/Vplic_sifive_tb

$(TOP): plic_sifive_tb.cpp ../axi_bfm.h $(RTL)
	verilator $(VFLAGS) plic_sifive_tb.cpp $(RTL)

run: $(TOP)
	./$(TOP)

clean:
	rm -rf obj_dir *.vcd

.PHONY: run clean

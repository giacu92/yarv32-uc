# linux-soc M8: axi4_lite_uart16550 register/behaviour test against the offsets the
# mainline driver uses. make run -> prints the check count, exits non-zero on
# any failure.
RTL := \
	../../../src/rtl/pkg/rv32_pkg.sv \
	../../../src/rtl/bus/axi4_lite_if.sv \
	../../../src/rtl/utils/axi4_lite_uart16550.sv \
	uart16550_tb.sv

VFLAGS := --cc --exe --build --top-module uart16550_tb \
	-Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	-I../../../src/rtl/pkg -I../../../src/rtl/bus -I../../../src/rtl/utils -CFLAGS -I$(CURDIR)/..

TOP := obj_dir/Vuart16550_tb

$(TOP): uart16550_tb.cpp ../axi_bfm.h $(RTL)
	verilator $(VFLAGS) uart16550_tb.cpp $(RTL)

run: $(TOP)
	./$(TOP)

clean:
	rm -rf obj_dir *.vcd

.PHONY: run clean

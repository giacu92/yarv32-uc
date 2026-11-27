# dport_32to64 vs a 64-bit native_ram, checked against a 32-bit shadow
# model with random sub-word stores and loads. `make run` prints PASS/FAIL.
RTL := ../../../src/rtl/pkg/rv32_pkg.sv ../../../src/rtl/utils/native_ram.sv \
       ../../../src/rtl/utils/dport_32to64.sv dport_tb.sv
run:
	verilator --binary --timing --top-module dport_tb -Wno-WIDTH -Wno-UNUSED \
	    -Wno-INITIALDLY -Wno-TIMESCALEMOD $(RTL) >/dev/null
	./obj_dir/Vdport_tb
clean:
	rm -rf obj_dir
.PHONY: run clean

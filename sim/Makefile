# Verilator simulation for the RV32 fetch + decode subsystem.
#
#   make          build the sim binary (obj_dir/Vsim_top)
#   make run      build + run, prints the fetch + decode logs
#   make wave     open the VCD in gtkwave (if installed)
#   make clean    remove build artefacts + waveforms
#
# Requires: verilator, make, g++. Install on Debian/Ubuntu:
#   sudo apt-get install -y verilator

# RTL sources (relative to this sim/ dir). axi4_lite_xbar is the peri-bus
# 1->2 mux (msip_peri vs clint_timer, addr[12] decode). axi4_lite_ram is
# not instantiated by sim_top (Harvard uses native_ram); it is exercised by
# the standalone sim/hw/ram_tb build, which has its own VSRC.
RTL := \
	../src/rtl/pkg/rv32_pkg.sv \
	../src/rtl/bus/axi4_lite_if.sv \
	../src/rtl/bus/axi4_lite_master_bridge.sv \
	../src/rtl/bus/axi4_lite_xbar.sv \
	../src/rtl/bus/axi4_lite_xbar_3.sv \
	../src/rtl/core/fetch_stage.sv \
	../src/rtl/core/reg_file.sv \
	../src/rtl/core/csr_regfile.sv \
	../src/rtl/core/decode_stage.sv \
	../src/rtl/core/trap_unit.sv \
	../src/rtl/core/alu.sv \
	../src/rtl/core/branch_predictor.sv \
	../src/rtl/core/execute_stage.sv \
	../src/rtl/core/rv32imac_zicsr_zifencei.sv \
	../src/rtl/utils/native_ram.sv \
	../src/rtl/utils/msip_peri.sv \
	../src/rtl/utils/clint_timer.sv \
	../src/rtl/utils/axi4_lite_uart.sv \
	sim_top.sv

# Harvard topology: native u_imem (read-only) + u_dmem (byte-strobed) on
# dedicated CPU ports, AXI only for peripherals (msip_peri + clint_timer
# behind the peri xbar).
# Extra Verilator parameter overrides, e.g. board-accurate UART timing:
#   make VPARAMS="-GUART_CLK_HZ=25000000 -GUART_BAUD=115200"
# (pair it with UART_BIT_CYCLES=217 at run time, and rm -rf obj_dir first --
# a VPARAMS change is not tracked by the build dependencies).
VPARAMS ?=

VFLAGS := --cc --exe --build --top-module sim_top $(VPARAMS) \
	-Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	--public-flat-rw \
	--trace \
	-I../src/rtl/pkg -I../src/rtl/core -I../src/rtl/bus -I../src/rtl/utils

HEX_PREREQ := imem.hex dmem.hex

TOP := obj_dir/Vsim_top

# Extra args for the sim binary.
#   RUN_ARGS="+IINIT=sw/quicksort/build/imem.hex +DINIT=sw/quicksort/build/dmem.hex"
# With no RUN_ARGS the sim loads the defaults baked into sim_top
# (imem.hex/dmem.hex).
RUN_ARGS ?=

$(TOP): sim_main.cpp $(RTL) $(HEX_PREREQ)
	verilator $(VFLAGS) sim_main.cpp $(RTL)

run: $(TOP)
	./$(TOP) $(RUN_ARGS)

# ----------------------------------------------------------------------
# regress — run every self-checking test and report pass/fail
# ----------------------------------------------------------------------
#
#   make regress                                run every test
#   make regress VPARAMS="-GLSU_LIVE_LOAD=0"    ... against an A/B config
#
# Needs the firmware built: `make -C sw` builds all of it. A test whose images
# are missing is reported SKIP, not FAIL, so a partial tree still runs.
#
# Three kinds of check, because the tests genuinely differ in how they report:
#
#   probe@<addr>=<hex>  Read a D-mem word back out of the RTL (PROBE, see
#                       sim_main.cpp) and compare. This is the exact result.
#                       Do NOT substitute a grep of the retire trace: the `li`
#                       that materialises 0x600D retires whether or not the
#                       store after it ever ran, so a trace grep passes a test
#                       that died between the two.
#   trace:<reg>=<hex>   Last writeback of that register in the retire trace.
#                       For quicksort, whose main() RETURNS 0x600D in a0 and
#                       never stores it -- start.S just parks. Its real
#                       verification is `make cosim`, which compares every one
#                       of 29293 retires against Spike; this is a smoke check
#                       plus a cycle number.
#   park                Clean park + WFI-halt invariant only. The hand-crafted
#                       Harvard oracle has no marker at all -- it is a
#                       load/store and RAW round-trip covered by the cosim.
#
# Every test also fails on a WFI-HALT FAIL regardless of its marker.
#
# uart_echo is special-cased at the end: it needs UART_RX stimulus, and its
# pass condition is the "GOOD" its firmware transmits, not a memory word.
REGRESS_TESTS := \
	quicksort:quicksort:200000:trace:x10=600d \
	bp_pred:isa/bp_pred:20000:probe@0x2000=600d \
	mul_ops:isa/mul_ops:20000:probe@0x2000=600d \
	div_ops:isa/div_ops:20000:probe@0x2000=600d \
	rvc_scramble:isa/rvc_scramble:20000:probe@0x2000=600d \
	span_target:isa/span_target:20000:probe@0x2000=600d \
	ifault:isa/ifault:20000:probe@0x2000=600d \
	isa_probe:isa/isa_probe:200000:park \
	trap:intr/trap:20000:probe@0x2000=600d \
	wfi_trap:intr/wfi_trap:20000:probe@0x2000=600d \
	timer:intr/timer:20000:probe@0x2000=600d

# ----------------------------------------------------------------------
# lint — dead wires, unread bits, unused parameters
# ----------------------------------------------------------------------
#
# NOT part of the build, and deliberately not a gate: it is advisory, run it
# after an RTL change.
#
# The build's `-Wno-UNUSED` is NOT what hides these. UNUSEDSIGNAL /
# UNUSEDPARAM / UNDRIVEN are off by default in Verilator (they live under
# -Wall), so dropping -Wno-UNUSED from VFLAGS changes nothing -- the checks
# have to be asked for, which is what this target does.
#
# It found a real dead wire the 2026-09-02 review had missed
# (`peri_bus_drive`), so it earns its place. But roughly two thirds of what it
# reports is benign BY DESIGN and will not go away:
#
#   - unread BITS of a wide signal: `de_i`'s fields that execute does not
#     read, `mul_uu[31:0]` / `mul_su[31:0]` (only the high word is ever
#     selected), the `off[31:12]` slices in decode's RVC immediate helpers,
#     `mip_i` / `mie_i` reserved bits.
#   - parameterised-away logic: `native_ram`'s `req_wdata_i` / `req_wstrb_i` /
#     `launch_write` are unused in the READ_ONLY elaboration.
#   - signals that exist only for a consumer Verilator cannot see: the
#     `mem_probe_w` VCD window, and `mem_bus_drive`, which sim_main.cpp reads
#     through --public-flat-rw for the stall histogram.
#   - documented absences: `reg_file`'s `rstn_i` (no runtime reset, single
#     write port), `axi4_lite_if`'s `aclk` / `aresetn`.
#
# So read it, do not chase it to zero. What to act on is a WHOLE signal
# reported unused that nothing outside the RTL reads.
lint: $(HEX_PREREQ)
	verilator --lint-only --top-module sim_top \
	    -Wwarn-UNUSEDSIGNAL -Wwarn-UNUSEDPARAM -Wwarn-UNDRIVEN \
	    -Wno-WIDTH -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	    -I../src/rtl/pkg -I../src/rtl/core -I../src/rtl/bus -I../src/rtl/utils \
	    $(RTL) 2>&1 | grep -vE '^(- V e r|- Verilator:)' || true

regress: $(TOP)
	@fail=0; skip=0; pass=0; \
	printf '%-15s %-6s %s\n' TEST RESULT DETAIL; \
	for t in $(REGRESS_TESTS); do \
	    name=$$(echo "$$t" | cut -d: -f1); \
	    dir=$$(echo  "$$t" | cut -d: -f2); \
	    maxc=$$(echo "$$t" | cut -d: -f3); \
	    check=$$(echo "$$t" | cut -d: -f4-); \
	    if [ ! -f "sw/$$dir/build/imem.hex" ]; then \
	        printf '%-15s %-6s %s\n' "$$name" SKIP "no build/ -- run: make -C sw/$$dir"; \
	        skip=$$((skip+1)); continue; \
	    fi; \
	    probe=; want=; reg=; \
	    case "$$check" in \
	      probe@*) probe=$$(echo "$$check" | sed 's/^probe@\([^=]*\)=.*/\1/'); \
	               want=$$(echo "$$check" | sed 's/.*=//');; \
	      trace:*) reg=$$(echo  "$$check" | sed 's/^trace:\([^=]*\)=.*/\1/'); \
	               want=$$(echo "$$check" | sed 's/.*=//');; \
	    esac; \
	    out=$$(PROBE=$$probe NO_VCD=1 MAX_CYC=$$maxc RTL_TRACE=.regress.trace ./$(TOP) \
	           +IINIT=sw/$$dir/build/imem.hex +DINIT=sw/$$dir/build/dmem.hex 2>&1); \
	    cyc=$$(echo "$$out" | sed -n 's/^retired [0-9]* instructions in \([0-9]*\) cycles.*/\1/p'); \
	    bad=; \
	    echo "$$out" | grep -q 'WFI-HALT FAIL' && bad="WFI-HALT FAIL"; \
	    echo "$$out" | grep -q 'parked at' || bad="$$bad no clean park"; \
	    got=; \
	    if [ -n "$$probe" ]; then \
	        got=$$(echo "$$out" | sed -n 's/^probe\[.*\] = 0x0*\([0-9a-fA-F]*\)$$/\1/p' | tail -1); \
	    elif [ -n "$$reg" ]; then \
	        got=$$(sed -n "s/^0x[0-9a-f]* $$reg 0x0*\([0-9a-f]*\)$$/\1/p" .regress.trace | tail -1); \
	    fi; \
	    if [ -n "$$want" ] && [ "$$got" != "$$want" ]; then \
	        bad="$$bad marker=0x$${got:-none} want=0x$$want"; \
	    fi; \
	    if [ -z "$$bad" ]; then \
	        printf '%-15s %-6s %s\n' "$$name" PASS "$$cyc cyc"; pass=$$((pass+1)); \
	    else \
	        printf '%-15s %-6s %s\n' "$$name" FAIL "$$cyc cyc --$$bad"; fail=$$((fail+1)); \
	    fi; \
	done; \
	rm -f .regress.trace; \
	if [ -f sw/peri/uart_echo/build/imem.hex ]; then \
	    out=$$(UART_RX='abcdefgh' NO_VCD=1 MAX_CYC=400000 ./$(TOP) \
	           +IINIT=sw/peri/uart_echo/build/imem.hex \
	           +DINIT=sw/peri/uart_echo/build/dmem.hex 2>&1); \
	    cyc=$$(echo "$$out" | sed -n 's/^retired [0-9]* instructions in \([0-9]*\) cycles.*/\1/p'); \
	    if tr -d '\n\r' < sim_uart_tx.txt | grep -q 'GOOD$$'; then \
	        printf '%-15s %-6s %s\n' uart_echo PASS "$$cyc cyc, TX ends GOOD"; pass=$$((pass+1)); \
	    else \
	        printf '%-15s %-6s %s\n' uart_echo FAIL "TX does not end GOOD"; fail=$$((fail+1)); \
	    fi; \
	else \
	    printf '%-15s %-6s %s\n' uart_echo SKIP "no build/"; skip=$$((skip+1)); \
	fi; \
	out=$$(NO_VCD=1 MAX_CYC=4000 ./$(TOP) 2>&1); \
	if echo "$$out" | grep -q 'WFI-halt check: OK' && echo "$$out" | grep -q 'parked at'; then \
	    printf '%-15s %-6s %s\n' harvard_oracle PASS "parks clean"; pass=$$((pass+1)); \
	else \
	    printf '%-15s %-6s %s\n' harvard_oracle FAIL "no clean park"; fail=$$((fail+1)); \
	fi; \
	echo; echo "$$pass passed, $$fail failed, $$skip skipped"; \
	[ $$fail -eq 0 ]

# ----------------------------------------------------------------------
# compliance -- the official riscv-tests ISA suite (sw/compliance/)
# ----------------------------------------------------------------------
#
#   make -C sw/compliance RISCV_PREFIX=<prefix>   build the ~67 test images
#   make compliance                               run them all
#   make compliance COMPLIANCE_MAXC=400000        ... with a longer bound
#
# Kept OUT of `regress` on purpose. `regress` is the fast in-house oracle set
# (13 tests, seconds) and is meant to be run after every RTL edit; this is the
# external suite, needs a separate build tree and a toolchain prefix, and
# answers a different question -- not "did I break what I wrote" but "does
# this core implement RV32IM_Zicsr as specified".
#
# Verdict protocol: env/riscv_test.h puts `tohost` at D-mem 0x2000 for every
# test, so one fixed PROBE reads them all.
#     0x1        pass
#     2*n+1      failed in test case n (look n up in the test source)
#     0x539      unhandled trap (1337, upstream's marker for "the environment
#                trapped and no mtvec_handler was installed")
#     0x0 / no park   the test never reported -- hang or MAX_CYC
#
# EXPECTED FAILURES. Every one of these is a documented property of this core,
# not a defect, so they are listed rather than skipped: a test that starts
# passing (XPASS) fails this target too, because it means the list is stale.
#   rv32ui-fence_i    Harvard. There is no write path from the LSU to the
#                     I-mem, so self-modifying code cannot work by
#                     construction; fence.i is correctly a nop.
#   rv32ui-ma_data    Misaligned load/store is not implemented in hardware --
#                     it traps (mcause 4/6), which the spec permits and the
#                     test does not (it has no handler and expects the access
#                     to complete).
#   rv32uc-rvc        Fails at case 6 only, and for the Harvard reason again:
#                     the case loads from a `.dword` the test embeds in its
#                     own .text, i.e. from the I-mem, through a data load.
#                     Cases 2-5 and 8-30 (every actual RVC encoding) pass.
#   rv32mi-pmpaddr    No PMP.
#   rv32mi-instret_overflow  Fails at case 3: mcycleh / minstreth (the upper
#                     halves RV32 mandates) are not implemented, so a write to
#                     minstreth neither lands nor suppresses that
#                     instruction's count. This is the one entry in the list
#                     that is a genuine gap rather than a design choice.
COMPLIANCE_DIR  := sw/compliance
COMPLIANCE_MAXC ?= 100000

COMPLIANCE_XFAIL := \
	rv32ui-fence_i:539 \
	rv32ui-ma_data:539 \
	rv32uc-rvc:d \
	rv32mi-pmpaddr:3 \
	rv32mi-instret_overflow:7

compliance: $(TOP)
	@builds=$$(ls -d $(COMPLIANCE_DIR)/build/*/ 2>/dev/null); \
	if [ -z "$$builds" ]; then \
	    echo "no test images -- build them first:"; \
	    echo "  make -C $(COMPLIANCE_DIR) RISCV_PREFIX=<riscv32 prefix>"; \
	    exit 1; \
	fi; \
	pass=0; fail=0; xfail=0; \
	printf '%-26s %-7s %s\n' TEST RESULT DETAIL; \
	for d in $$builds; do \
	    n=$$(basename $$d); \
	    want=; \
	    for x in $(COMPLIANCE_XFAIL); do \
	        case "$$x" in $$n:*) want=$${x#*:};; esac; \
	    done; \
	    out=$$(PROBE=0x2000 NO_VCD=1 MAX_CYC=$(COMPLIANCE_MAXC) ./$(TOP) \
	           +IINIT=$${d}imem.hex +DINIT=$${d}dmem.hex 2>&1); \
	    cyc=$$(echo "$$out" | sed -n 's/^retired [0-9]* instructions in \([0-9]*\) cycles.*/\1/p'); \
	    got=$$(echo "$$out" | sed -n 's/^probe\[0x00002000\] = 0x0*\([0-9a-fA-F]*\)$$/\1/p'); \
	    got=$${got:-0}; \
	    if ! echo "$$out" | grep -q 'parked at'; then \
	        printf '%-26s %-7s %s\n' "$$n" FAIL "no park -- hit the $(COMPLIANCE_MAXC)-cycle bound"; \
	        fail=$$((fail+1)); continue; \
	    fi; \
	    v=$$((0x$$got)); \
	    if [ "$$v" = "1" ] && [ -z "$$want" ]; then \
	        printf '%-26s %-7s %s\n' "$$n" PASS "$$cyc cyc"; pass=$$((pass+1)); \
	    elif [ "$$v" = "1" ]; then \
	        printf '%-26s %-7s %s\n' "$$n" XPASS \
	            "now passes -- drop it from COMPLIANCE_XFAIL"; fail=$$((fail+1)); \
	    else \
	        if [ "$$v" = "1337" ]; then why="unhandled trap"; \
	        elif [ "$$v" = "0" ]; then why="never reported (tohost still 0)"; \
	        else why="failed test case $$(( (v - 1) / 2 ))"; fi; \
	        if [ "$$got" = "$$want" ]; then \
	            printf '%-26s %-7s %s\n' "$$n" XFAIL "$$why (expected)"; \
	            xfail=$$((xfail+1)); \
	        else \
	            printf '%-26s %-7s %s\n' "$$n" FAIL "tohost=0x$$got -- $$why"; \
	            fail=$$((fail+1)); \
	        fi; \
	    fi; \
	done; \
	echo; echo "$$pass passed, $$xfail expected-fail, $$fail failed"; \
	[ $$fail -eq 0 ]

wave: sim_top.vcd
	gtkwave sim_top.vcd

clean:
	rm -rf obj_dir *.vcd *.fst *.log

.PHONY: run regress compliance lint wave clean

# =============================================================================
# SDC — Timing constraints for the RV32IMAC core on Tang Nano 20k
#
# Target board:  Sipeed Tang Nano 20k (GW2AR-LV18QN88C8/I7, QFN88)
# Primary clock: 25 MHz single-ended reference from the MS5351M clock
#   generator (crystal-fed; CLK0 on PIN10, LVCMOS33).
#
# Gowin PnR reads this file (cmd.do passes `-sdc .../rv32imac_Zicsr_Zifencei.sdc`).
# The Gowin SDC subset supports:
#   - create_clock / create_generated_clock
#   - set_input_delay / set_output_delay
#   - set_false_path / set_multicycle_path
# (no set_clock_groups / set_max_delay by default — keep it minimal.)
# =============================================================================

# Primary clock: 25 MHz single-ended input on clk_i (PIN10). Period = 40 ns.
# 50/50 duty cycle.
create_clock -name clk25 -period 40 [get_ports {clk_i}]

# CPU core clock.
# *** 50 MHz rPLL MODE (active): clk_core = clk_i * 10 / 5 = 50 MHz, period
# 20 ns. Must agree with top_module's rPLL (IDIV_SEL=4 / FBDIV_SEL=9 /
# ODIV_SEL=16) and with global_freq in pnr_check.tcl / impl/pnr/cmd.do — if
# these three disagree the reports are constrained to something the design
# does not run at. To fall back to the 25 MHz PLL-bypass build: comment the
# rPLL out in top_module, comment this line out again (clk_core then
# inherits clk25's 40 ns), and set global_freq back to 25.000. ***
# Gowin auto-derives a clock on the rPLL output (named *.default_gen_clk);
# this explicit constraint takes precedence (a PnR warning is expected and
# harmless — it just names the clock clk_core for the timing reports).
create_generated_clock -name clk_core -source [get_ports {clk_i}] -master_clock clk25 -multiply_by 10 -divide_by 5 [get_nets {clk_core}]

# Async reset: treat rst_i (board reset button S1, active-high on this
# board) as asynchronous to the fabric clocks.
set_false_path -from [get_ports {rst_i}]

# UART RX pin: driven by a far-end transmitter with its own oscillator, so
# it is asynchronous to clk_core by definition. top_module double-flops it
# before the UART's sampler, which is the synchronizer that makes the
# crossing safe; there is no launch clock to relate it to, so cut it.
set_false_path -from [get_ports {uart_rxd_i}]

# I2S receiver pins. In slave mode BCLK, LRCK and SD all come from an
# external audio device running off its own oscillator, so they are
# asynchronous to clk_core exactly like uart_rxd_i; in master mode the two
# clock pads are driven from clk_core and come back in through the same
# input path, which is a round trip through a pad at 1 MHz and not a timed
# path either. top_module double-flops all three before the peripheral, and
# the peripheral OVERSAMPLES BCLK rather than clocking off it -- BCLK must
# NOT be declared as a clock in either mode, since that would create a
# domain the design does not have.
set_false_path -from [get_ports {i2s_bclk_io}]
set_false_path -from [get_ports {i2s_lrck_io}]
set_false_path -from [get_ports {i2s_sd_i}]

# Debug LEDs are not timing-critical (human eye).
set_false_path -to [get_ports {led_o[*]}]

# Single clock domain: the CPU, both AXI4-Lite bridges, the buses and
# the peripheral slaves all run on clk_core (50 MHz, generated above).
# clk_i (25 MHz, clk25) only feeds the rPLL — there are no user-logic
# paths on clk25, so there is no clock-domain crossing to cut. Slave
# outputs (rdata, rvalid, etc.) are registered in axi4_lite_ram and
# captured on the same clk_core edge; no set_input_delay is needed for
# this single-chip topology.

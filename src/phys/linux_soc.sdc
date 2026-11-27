# =============================================================================
# SDC -- Linux SoC (top_soc) on the Tang Nano 20k (GW2AR-LV18QN88C8/I7)
#
# Must be a <File type="file.sdc"> entry in linux_soc.gprj (it is): merely
# naming it from impl/pnr/cmd.do is not enough, PnR then falls back to an
# unconstrained 100 MHz.
#
# Gowin SDC subset: create_clock / create_generated_clock / set_input_delay /
# set_output_delay / set_false_path / set_multicycle_path. Keep it to those.
# =============================================================================

# Primary clock: 25 MHz reference on clk_i (PIN10). Period = 40 ns.
create_clock -name clk25 -period 40 [get_ports {clk_i}]

# Core clock: rPLL CLKOUT = clk_i * 10 / 5 = 50 MHz, period 20 ns.
# These must agree or the reports constrain a design that does not ship:
# top_soc's rPLL (IDIV_SEL=4 / FBDIV_SEL=9 / ODIV_SEL=16), this constraint,
# -global_freq 50.000 in impl/remote_pnr_soc.sh, rv32_pkg::UART_CLK_HZ, and
# top_soc's CLK_FREQ_MHZ (which sets the SDRAM refresh spacing -- a stale
# value under-refreshes the chip rather than mis-reporting timing).
create_generated_clock -name clk_core -source [get_ports {clk_i}] -master_clock clk25 -multiply_by 10 -divide_by 5 [get_nets {clk_core}]

# SDRAM clock (rPLL CLKOUTP, 180 deg) deliberately NOT constrained, as in the
# cache repo's yarv32_cache.sdc: an explicit generated clock on it was
# rejected there (TA2003 / TA1052) because the net drives only the
# O_sdram_clk pin. No fabric logic runs on it; the SDRAM's setup/hold against
# it is an off-die SIP property tuned with PSDA_SEL.

# Async reset (board button S1, active high); synchronized in top_soc.
set_false_path -from [get_ports {rst_i}]

# Asynchronous inputs, each double-flopped in top_soc before use: no launch
# clock to relate them to.
set_false_path -from [get_ports {uart_rxd_i}]
set_false_path -from [get_ports {spi_miso_i}]
set_false_path -from [get_ports {gpio_io[*]}]

# Debug LEDs are not timing-critical (human eye).
set_false_path -to [get_ports {led_o[*]}]

# Single clock domain: CPU, cache, SDRAM controller, bus and peripherals all
# run on clk_core. clk25 only feeds the rPLL; sdram_clk only leaves the die.

`resetall
`timescale 1ns / 1ps
`default_nettype none

// ---------------------------------------------------------------------------
// Project-wide define for the ZipCPU sdspi SDIO controller.
//
// sdspi/sdio_top.v selects its control port between AXI-Lite and Wishbone
// with a global `ifdef SDIO_AXI. We want the AXI-Lite control port (the
// core's peri bus is AXI4-Lite). Gowin's `set_option` has no Verilog-define
// flag (confirmed against the Tcl command reference SUG1220E), so the macro
// is defined by listing THIS file first in the .gprj FileList: a `define
// persists across files in a single elaboration pass, so every sdspi source
// parsed after it sees SDIO_AXI. This avoids editing the vendored sources.
//
// The file carries no module on purpose -- it is a compile-time directive
// only. (Belt-and-suspenders: sdspi_axi_wrap.sv repeats the same guarded
// `define, so the macro is also set even if a future reordering lists the
// wrapper before this file.)
// ---------------------------------------------------------------------------

`ifndef SDIO_AXI
`define SDIO_AXI
`endif

`resetall

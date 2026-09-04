`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Focused native RAM compliance testbench wrapper (32-bit, OUTSTANDING=1).
 *
 * Instantiates two native_ram slaves and exposes the master-side native
 * signals as flat ports so the C++ BFM (native_mem_tb.cpp) can drive them
 * directly. native_ram's ports are flat parametric logic (sized by
 * DATA_WIDTH / OUTSTANDING), not the cpu_mem_req_t/cpu_mem_rsp_t structs, so this
 * wrapper is plain point-to-point wiring:
 *   - u_dmem : READ_ONLY=0 (the LSU's byte-strobed D-mem).
 *   - u_imem : READ_ONLY=1 (a 32-bit read-only port — the real I-mem is
 *     64-bit/2-outstanding and has its own TB, native_ram64_tb; this one
 *     stays 32-bit/1-outstanding to gate the D-mem-shaped config).
 *
 * This is NOT part of synthesis; it is a protocol-compliance gate for the
 * native interface, analogous to ram_tb for AXI4-Lite. It checks:
 *   - RVALID is registered and held until RREADY (the key fix vs a naive
 *     one-cycle pulse): the master deliberately keeps RREADY low for
 *     several cycles after RVALID rises; RVALID must not drop.
 *   - byte-strobed writes read back correctly.
 *   - back-to-back writes to distinct addresses.
 *   - single-outstanding: WREADY is low while an unread read response is
 *     held (RVALID=1, RREADY=0).
 *   - posted store: a store (we=1) commits at the launch handshake
 *     (wvalid && wready) and is immediately readable.
 *   - read-only: a write to u_imem (READ_ONLY=1) is ignored.
 */

module native_mem_tb (
    input wire clk_i,
    input wire rstn_i,

    // --- D-mem (RW) master -> slave, driven by the BFM ---
    input  wire        d_wvalid,
    input  wire        d_we,
    input  wire [31:0] d_addr,
    input  wire [31:0] d_wdata,
    input  wire [ 3:0] d_wstrb,
    input  wire        d_rready,
    // D-mem slave -> master, sampled by the BFM
    output wire        d_wready,
    output wire        d_rvalid,
    output wire [31:0] d_rdata,
    output wire        d_bvalid,

    // --- I-mem (RO) master -> slave, driven by the BFM ---
    input  wire        i_wvalid,
    input  wire        i_we,
    input  wire [31:0] i_addr,
    input  wire [31:0] i_wdata,
    input  wire [ 3:0] i_wstrb,
    input  wire        i_rready,
    // I-mem slave -> master, sampled by the BFM
    output wire        i_wready,
    output wire        i_rvalid,
    output wire [31:0] i_rdata,
    output wire        i_bvalid
);

    // -----------------------------------------------------------------
    // D-mem (read/write, byte-strobed, single-outstanding)
    // -----------------------------------------------------------------
    native_ram #(
        .ADDR_W     (16),
        .DATA_WIDTH (32),
        .READ_ONLY  (0),
        .OUTSTANDING(1),
        .INIT_FILE  ("")
    ) u_dmem (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
        .req_valid_i (d_wvalid),
        .req_we_i    (d_we),
        .req_addr_i  (d_addr),
        .req_wdata_i (d_wdata),
        .req_wstrb_i (d_wstrb),
        .req_rready_i(d_rready),
        .rsp_wready_o(d_wready),
        .rsp_rvalid_o(d_rvalid),
        .rsp_rdata_o (d_rdata),
        .rsp_bvalid_o(d_bvalid)
    );

    // -----------------------------------------------------------------
    // I-mem (read-only, 32-bit, single-outstanding). Preload two known
    // words so reads can be checked.
    // -----------------------------------------------------------------
    native_ram #(
        .ADDR_W     (16),
        .DATA_WIDTH (32),
        .READ_ONLY  (1),
        .OUTSTANDING(1),
        .INIT_FILE  ("")
    ) u_imem (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
        .req_valid_i (i_wvalid),
        .req_we_i    (i_we),
        .req_addr_i  (i_addr),
        .req_wdata_i (i_wdata),
        .req_wstrb_i (i_wstrb),
        .req_rready_i(i_rready),
        .rsp_wready_o(i_wready),
        .rsp_rvalid_o(i_rvalid),
        .rsp_rdata_o (i_rdata),
        .rsp_bvalid_o(i_bvalid)
    );

    // Preload two words for read checks (sim only).
    initial begin
        u_imem.mem[0] = 32'hCAFE_BABE;
        u_imem.mem[1] = 32'hDEAD_BEEF;
    end

endmodule

`resetall

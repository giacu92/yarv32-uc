`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Compliance testbench wrapper for the 64-bit / 2-outstanding / read-only
 * configuration of native_ram — the shape the widened I-mem fetch port uses.
 *
 * Exposes the master-side native signals as flat ports so the C++ BFM
 * (native_ram64_tb.cpp) can drive them directly. native_ram's ports are flat
 * parametric logic (sized by DATA_WIDTH / OUTSTANDING); this wrapper is plain
 * point-to-point wiring (no cpu_mem_req_t / cpu_mem_rsp_t structs — those are 32-bit
 * only and cannot carry the 64-bit fetch word).
 *
 * Checks (in the BFM):
 *   - Two back-to-back reads are BOTH accepted (OUTSTANDING=2): a second
 *     read launches the cycle the first one's data lands, without waiting
 *     for the first to be drained.
 *   - A third read is NOT accepted while two responses are held (skid full):
 *     wready stays low until one is drained.
 *   - RVALID is registered and held until RREADY (the master holds RREADY low
 *     for several cycles after RVALID rises; RVALID must not drop).
 *   - 64-bit read data: each access returns the full 8-byte word (low 32 bits
 *     = the instruction at the byte address, high 32 bits = +4), preloaded
 *     with recognisable patterns.
 *
 * Not part of synthesis.
 */

module native_ram64_tb (
    input wire clk_i,
    input wire rstn_i,

    // Master -> slave (read-only: valid / addr / rready).
    input  wire        i_valid,
    input  wire [31:0] i_addr,
    input  wire        i_rready,
    // Slave -> master.
    output wire        i_ready,
    output wire        i_rvalid,
    output wire [63:0] i_rdata
);

    // Write-ack is held low by a read-only native_ram (no B channel); sink it
    // to a dangling wire so the port is connected.
    wire unused_bvalid;

    native_ram #(
        .ADDR_W     (16),
        .DATA_WIDTH (64),
        .READ_ONLY  (1),
        .OUTSTANDING(2),
        .INIT_FILE  ("")
    ) u_imem (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
        .req_valid_i (i_valid),
        .req_we_i    (1'b0),          // read-only
        .req_addr_i  (i_addr),
        .req_wdata_i ({64{1'b0}}),
        .req_wstrb_i ({8{1'b0}}),
        .req_rready_i(i_rready),
        .rsp_wready_o(i_ready),
        .rsp_rvalid_o(i_rvalid),
        .rsp_rdata_o (i_rdata),
        .rsp_bvalid_o(unused_bvalid)
    );

    // Preload three 64-bit words with distinct low/high 32-bit halves so a
    // width or lane-swap bug reads back wrong (sim only).
    initial begin
        u_imem.mem[0] = 64'hDEADBEEF_CAFEBABE;
        u_imem.mem[1] = 64'h0BADF00D_12345678;
        u_imem.mem[2] = 64'hAAAABBBB_CCCCDDDD;
    end

endmodule

`resetall

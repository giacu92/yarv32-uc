`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* 32-bit LSU port -> 64-bit native memory port (Linux SoC, M5).
*
* The CPU's D-side is the XLEN-wide mem_req_t / mem_rsp_t; the cache
* subproject's CPU ports are 64 bits wide (a doubleword per access). This
* shim is pure lane steering plus one flop:
*   - request: addr passes through unchanged (the 64-bit slave ignores
*     addr[2:0] for the word it returns); the 32-bit write data is placed in
*     both halves and the 4 strobes are moved to the half addr[2] selects, so
*     the slave writes exactly the bytes the CPU asked for;
*   - response: the half addr[2] selected is returned. addr[2] is latched on
*     the READ ACCEPT, because the CPU may move its request lines before the
*     response arrives (the same lifetime rule as execute's mem_addr_q).
* Single outstanding, like the D-port it serves; handshakes pass through.
*
* Flat ports on the 64-bit side: the cache package defines its own
* mem_req_t / mem_rsp_t with the same names as rv32_pkg's, so the two
* cannot meet as structs in one scope.
*/
module dport_32to64 (
    input wire clk_i,
    input wire rstn_i,

    // CPU side (32-bit)
    input  mem_req_t cpu_req_i,
    output mem_rsp_t cpu_rsp_o,

    // memory side (64-bit native protocol)
    output wire        m_valid_o,
    output wire        m_we_o,
    output wire [31:0] m_addr_o,
    output wire [63:0] m_wdata_o,
    output wire [ 7:0] m_wstrb_o,
    output wire        m_rready_o,
    input  wire        m_wready_i,
    input  wire        m_rvalid_i,
    input  wire [63:0] m_rdata_i,
    input  wire        m_bvalid_i
);
    wire hi = cpu_req_i.addr[2];

    assign m_valid_o  = cpu_req_i.wvalid;
    assign m_we_o     = cpu_req_i.we;
    assign m_addr_o   = cpu_req_i.addr;
    assign m_wdata_o  = {cpu_req_i.wdata, cpu_req_i.wdata};
    assign m_wstrb_o  = hi ? {cpu_req_i.wstrb, 4'b0000} : {4'b0000, cpu_req_i.wstrb};
    assign m_rready_o = cpu_req_i.rready;

    logic hi_q;  // addr[2] of the outstanding read
    always_ff @(posedge clk_i) begin
        if (!rstn_i) hi_q <= 1'b0;
        else if (cpu_req_i.wvalid && m_wready_i && !cpu_req_i.we) hi_q <= hi;
    end

    assign cpu_rsp_o.wready = m_wready_i;
    assign cpu_rsp_o.rvalid = m_rvalid_i;
    assign cpu_rsp_o.rdata  = hi_q ? m_rdata_i[63:32] : m_rdata_i[31:0];
    assign cpu_rsp_o.bvalid = m_bvalid_i;
endmodule

`resetall

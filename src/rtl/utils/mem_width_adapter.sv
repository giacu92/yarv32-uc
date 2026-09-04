`resetall
`timescale 1ns / 1ps
`default_nettype none

// rv32_pkg, not yarv32_cache_pkg: both sides of this module speak the CPU's
// protocol vocabulary (cpu_mem32_req_t is the 32-bit master view, cpu_mem_req_t the
// 64-bit bus both packages define with identical shapes), and the 32-bit
// convention is a CPU property. cache_cntrl's ports carry the cache package's
// expansion of the same shapes -- connections are packed-vector assignments.
import rv32_pkg::*;

/**
 * Native-protocol width adapter: 32-bit CPU side, 64-bit memory side.
 *
 * The RV32 LSU never moves more than 4 bytes, but cache_cntrl's ports are
 * 64 bit — the I port because the fetch unit takes two instructions per
 * beat, the D port because both caches share every array indexed by cache
 * number (skid_q, rq_q, cmp_dw_sel_q, the store merge, the FSM's response
 * select). Narrowing the D side of that datapath would mean splitting all
 * of it for a saving of ~100 flops; this module buys the same thing for a
 * mux and a two-entry queue, and leaves the cache untouched.
 *
 * Loads : the address passes through unchanged (the cache selects the
 *         doubleword with addr[4:3]; addr[2] selects the word inside it
 *         and is only this module's business). The half select is
 *         REGISTERED at the accept, not read off the live request — the
 *         master is allowed to change the address the cycle after wready,
 *         and the response can arrive many cycles later.
 * Stores: wdata is replicated into both halves and wstrb is shifted into
 *         the addressed one. Nothing else is needed: the cache's store
 *         path is already byte-strobed (a store hit merges into the line
 *         the hit way has on its output, a store miss merges into the
 *         refilled line), so an unstrobed half simply does not commit.
 *
 * Zero added latency: the request path is wires and the response path is
 * one mux.
 *
 * The half-select queue is 2 deep, which covers cache_cntrl's deepest
 * port (the I port's 2 outstanding reads); the D port is
 * single-outstanding, so one entry is ever used there. A port that
 * allowed more outstanding reads than this would need a deeper queue —
 * hence the assertion below rather than a silent wrap.
 *
 * Naming: ports use *_i/_o; internals no prefix; flops _q.
 */

module mem_width_adapter (
    input wire clk_i,
    input wire rstn_i,

    // CPU side: 32-bit data
    input  cpu_mem32_req_t cpu_req_i,
    output cpu_mem32_rsp_t cpu_rsp_o,

    // Memory side: 64-bit data (cache_cntrl's I or D port)
    output cpu_mem_req_t mem_req_o,
    input  cpu_mem_rsp_t mem_rsp_i
);

    localparam int NARROW_W = CPU_MEM32_WIDTH;
    localparam int NARROW_STRB = NARROW_W / 8;

    // Which half of the 64-bit word this access lives in.
    wire half_sel = cpu_req_i.addr[2];

    // -----------------------------------------------------------------
    // Request path (combinational)
    // -----------------------------------------------------------------
    always_comb begin
        mem_req_o = '0;
        mem_req_o.valid = cpu_req_i.valid;
        mem_req_o.we = cpu_req_i.we;
        mem_req_o.addr = cpu_req_i.addr;
        // Replicated data, one-sided strobe: only the addressed half can
        // commit, so the other half's copy is never written anywhere.
        mem_req_o.wdata = {cpu_req_i.wdata, cpu_req_i.wdata};
        mem_req_o.wstrb = half_sel ?
            {cpu_req_i.wstrb, {NARROW_STRB{1'b0}}} : {{NARROW_STRB{1'b0}}, cpu_req_i.wstrb};
        mem_req_o.rready = cpu_req_i.rready;
    end

    // -----------------------------------------------------------------
    // Half-select queue: one entry per accepted READ, popped when its
    // response is consumed. Responses come back in accept order (the
    // cache guarantees it), so a plain 2-entry shift register is enough.
    // -----------------------------------------------------------------
    logic [1:0] half_q;  // half_q[0] is the head
    logic [1:0] cnt_q;  // 0 .. 2

    wire launch_read = cpu_req_i.valid && cpu_rsp_o.wready && !cpu_req_i.we;
    wire rsp_done = mem_rsp_i.rvalid && cpu_req_i.rready;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            half_q <= '0;
            cnt_q  <= '0;
        end else begin
            unique case ({
                launch_read, rsp_done
            })
                2'b10: begin
                    if (cnt_q == 2'd0) half_q[0] <= half_sel;
                    else half_q[1] <= half_sel;
                    cnt_q <= cnt_q + 2'd1;
                end
                2'b01: begin
                    half_q[0] <= half_q[1];
                    cnt_q     <= cnt_q - 2'd1;
                end
                2'b11: begin
                    // The head leaves and a new entry arrives in the same
                    // cycle: everything shifts down, the new one lands
                    // where the count leaves it.
                    half_q[0] <= (cnt_q == 2'd1) ? half_sel : half_q[1];
                    if (cnt_q == 2'd2) half_q[1] <= half_sel;
                end
                default: ;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Response path (combinational)
    // -----------------------------------------------------------------
    always_comb begin
        cpu_rsp_o        = '0;
        cpu_rsp_o.wready = mem_rsp_i.wready;
        cpu_rsp_o.rvalid = mem_rsp_i.rvalid;
        cpu_rsp_o.rdata  = half_q[0] ? mem_rsp_i.rdata[63:32] : mem_rsp_i.rdata[31:0];
        cpu_rsp_o.bvalid = mem_rsp_i.bvalid;
    end

`ifdef VERILATOR
    // The queue is sized for the deepest cache_cntrl port. Overflowing it
    // would silently return the wrong half of a later response, which is
    // exactly the kind of failure that looks like a cache bug.
    always_ff @(posedge clk_i) begin
        if (rstn_i && launch_read && !rsp_done) begin
            assert (cnt_q < 2'd2)
            else $fatal(1, "mem_width_adapter: more than 2 outstanding reads");
        end
    end
`endif

endmodule

`resetall

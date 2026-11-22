`timescale 1ns / 1ps
`default_nettype none
import rv32_pkg::*;

// dport_32to64 against a 64-bit native_ram, checked by a 32-bit shadow
// model: random byte / halfword / word stores and word loads at random
// addresses, random rready back-pressure. Every load is compared with the
// shadow. Prints PASS/FAIL and the check count.
module dport_tb;
    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;

    mem_req_t req;
    mem_rsp_t rsp;
    wire m_valid, m_we, m_rready, m_wready, m_rvalid, m_bvalid;
    wire [31:0] m_addr;
    wire [63:0] m_wdata, m_rdata;
    wire [7:0] m_wstrb;

    dport_32to64 dut (
        .clk_i(clk), .rstn_i(rstn), .cpu_req_i(req), .cpu_rsp_o(rsp),
        .m_valid_o(m_valid), .m_we_o(m_we), .m_addr_o(m_addr), .m_wdata_o(m_wdata),
        .m_wstrb_o(m_wstrb), .m_rready_o(m_rready), .m_wready_i(m_wready),
        .m_rvalid_i(m_rvalid), .m_rdata_i(m_rdata), .m_bvalid_i(m_bvalid)
    );

    native_ram #(.ADDR_W(10), .DATA_WIDTH(64), .READ_ONLY(0), .OUTSTANDING(1)) ram (
        .clk_i(clk), .rstn_i(rstn), .req_valid_i(m_valid), .req_we_i(m_we),
        .req_addr_i(m_addr), .req_wdata_i(m_wdata), .req_wstrb_i(m_wstrb),
        .req_rready_i(m_rready), .rsp_wready_o(m_wready), .rsp_rvalid_o(m_rvalid),
        .rsp_rdata_o(m_rdata), .rsp_bvalid_o(m_bvalid)
    );

    logic [31:0] shadow[256];  // 1 KiB = 256 words
    int checks = 0, errors = 0;

    // Handshakes are sampled at the NEGATIVE edge, i.e. with the values the
    // next rising edge will see. Sampling after the rising edge reads the
    // post-accept state (a read's own accept drops wready) and deadlocks.
    task automatic wait_accept();
        // The request was driven at a negative edge (see the callers), so the
        // first rising edge that sees it is the one after this check.
        while (!rsp.wready) @(negedge clk);
        @(posedge clk);
        @(negedge clk);
    endtask

    task automatic store(input int widx, input int off, input int size, input logic [31:0] v);
        logic [3:0] s;
        logic [31:0] d;
        @(negedge clk);  // drive only between edges: no race with the RAM's flops
        s = (size == 1) ? (4'b0001 << off) : (size == 2) ? (4'b0011 << off) : 4'b1111;
        d = v << (8 * off);
        req = '0;
        req.wvalid = 1; req.we = 1; req.addr = widx * 4 + off; req.wdata = d; req.wstrb = s;
        wait_accept();
        req = '0;
        for (int b = 0; b < 4; b++) if (s[b]) shadow[widx][8*b+:8] = d[8*b+:8];
    endtask

    task automatic load_check(input int widx);
        @(negedge clk);
        req = '0;
        req.wvalid = 1; req.we = 0; req.addr = widx * 4;
        wait_accept();
        req.wvalid = 0;
        req.addr = $urandom;  // the CPU may move its lines after the accept
        repeat ($urandom % 3) @(negedge clk);
        req.rready = 1;
        #1;
        while (!rsp.rvalid) @(negedge clk);
        checks++;
        if (rsp.rdata !== shadow[widx]) begin
            errors++;
            if (errors < 10) $display("MISMATCH word %0d: got %08x want %08x", widx, rsp.rdata, shadow[widx]);
        end
        @(posedge clk);  // response consumed at this edge
        @(negedge clk);
        req = '0;
    endtask

    // Watchdog: a stuck handshake is a FAIL, not a hang.
    initial begin
        #5000000;
        $display("dport_tb: TIMEOUT after %0d checks -> FAIL", checks);
        $finish;
    end

    initial begin
        req = '0;
        repeat (4) @(posedge clk);
        rstn = 1;
        for (int i = 0; i < 256; i++) store(i, 0, 4, $urandom);
        for (int n = 0; n < 4000; n++) begin
            automatic int w = $urandom % 256;
            case ($urandom % 4)
                0: store(w, 0, 4, $urandom);
                1: store(w, 2 * ($urandom % 2), 2, $urandom);
                2: store(w, $urandom % 4, 1, $urandom);
                default: load_check(w);
            endcase
        end
        for (int i = 0; i < 256; i++) load_check(i);
        $display("dport_tb: %0d checks, %0d errors -> %s", checks, errors, errors ? "FAIL" : "PASS");
        $finish;
    end
endmodule

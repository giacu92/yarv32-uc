`resetall
`timescale 1ns / 1ps
`default_nettype none

/**
* Latency-fuzz shim for a native-protocol memory port (simulation only).
*
* Sits between a CPU port and a native_ram and, when enabled, makes the
* memory behave like a variable-latency one -- a cache in front of SDRAM --
* while keeping the native protocol exactly:
*   - request side: refuses accepts at random (wready low, the request is not
*     forwarded), and never lets more than DEPTH reads be outstanding;
*   - response side: takes the RAM's responses as soon as they appear and
*     hands each one to the master only after a random extra delay of
*     0..MAX_DELAY cycles, strictly in order.
*
* Enabled by the plusarg +MEM_FUZZ=<seed> (seed != 0). Without it the shim is
* pure wires: every signal passes straight through, so cycle counts are
* identical to a build without it -- which is itself checked by `make
* regress` running unchanged.
*
* Why it exists (M3, 2026-11-22): the core was written against 1-cycle
* BSRAMs, and two places assumed it (fetch's single in-flight PC register,
* the LSU's response timing). Proving the core correct under variable
* latency BEFORE a cache is attached keeps a core bug and a cache bug from
* ever having to be told apart on the same failing run.
*/
module mem_fuzz #(
    parameter int DATA_W = 32,
    parameter int STRB_W = DATA_W / 8,
    parameter int DEPTH = 1,  // max outstanding reads the master may have
    parameter int MAX_DELAY = 7,  // extra response delay, cycles
    parameter int SALT = 0  // decorrelates the I and D streams
) (
    input wire clk_i,
    input wire rstn_i,

    // master side (CPU)
    input  wire              m_valid_i,
    input  wire              m_we_i,
    input  wire [      31:0] m_addr_i,
    input  wire [DATA_W-1:0] m_wdata_i,
    input  wire [STRB_W-1:0] m_wstrb_i,
    input  wire              m_rready_i,
    output wire              m_wready_o,
    output wire              m_rvalid_o,
    output wire [DATA_W-1:0] m_rdata_o,

    // slave side (native_ram)
    output wire              s_valid_o,
    output wire              s_we_o,
    output wire [      31:0] s_addr_o,
    output wire [DATA_W-1:0] s_wdata_o,
    output wire [STRB_W-1:0] s_wstrb_o,
    output wire              s_rready_o,
    input  wire              s_wready_i,
    input  wire              s_rvalid_i,
    input  wire [DATA_W-1:0] s_rdata_i
);
    int unsigned seed;
    bit          en;
    initial begin
        if ($value$plusargs("MEM_FUZZ=%d", seed) && seed != 0) begin
            en   = 1'b1;
            seed = seed * 2654435761 + SALT;
            void'($urandom(seed));
            $display("mem_fuzz[%0d]: enabled, seed %0d, max delay %0d, depth %0d", SALT, seed,
                     MAX_DELAY, DEPTH);
        end else en = 1'b0;
    end

    // ---- request side -------------------------------------------------
    logic block_q;  // refuse accepts this cycle (re-rolled every cycle)
    logic [$clog2(DEPTH+1)-1:0] outst_q;  // accepted reads not yet delivered
    wire full = (outst_q >= DEPTH[$clog2(DEPTH+1)-1:0]);
    wire gate = en & (block_q | full);

    assign s_valid_o  = m_valid_i & ~gate;
    assign s_we_o     = m_we_i;
    assign s_addr_o   = m_addr_i;
    assign s_wdata_o  = m_wdata_i;
    assign s_wstrb_o  = m_wstrb_i;
    assign m_wready_o = s_wready_i & ~gate;

    wire rd_accept = m_valid_i & m_wready_o & ~m_we_i;

    // ---- response side: in-order delay FIFO ---------------------------
    localparam int QD = DEPTH + 1;
    logic [DATA_W-1:0] q_data[QD];
    int unsigned q_due[QD];  // cycle at which the entry may be delivered
    int unsigned q_head, q_tail, q_cnt;
    int unsigned now;

    assign s_rready_o = en ? (q_cnt < QD) : m_rready_i;
    wire take = en & s_rvalid_i & s_rready_o;
    wire head_ready = (q_cnt != 0) && (now >= q_due[q_head]);
    assign m_rvalid_o = en ? head_ready : s_rvalid_i;
    assign m_rdata_o  = en ? q_data[q_head] : s_rdata_i;
    wire give = en & m_rvalid_o & m_rready_i;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            block_q <= 1'b0;
            outst_q <= '0;
            q_head  <= 0;
            q_tail  <= 0;
            q_cnt   <= 0;
            now     <= 0;
        end else if (en) begin
            now     <= now + 1;
            block_q <= ($urandom % 4) == 0;  // refuse ~25% of cycles
            if (take) begin
                q_data[q_tail] <= s_rdata_i;
                q_due[q_tail]  <= now + 1 + ($urandom % (MAX_DELAY + 1));
                q_tail         <= (q_tail + 1) % QD;
            end
            if (give) q_head <= (q_head + 1) % QD;
            q_cnt   <= q_cnt + (take ? 1 : 0) - (give ? 1 : 0);
            outst_q <= outst_q + (rd_accept ? 1 : 0) - (give ? 1 : 0);
        end
    end
endmodule

`resetall

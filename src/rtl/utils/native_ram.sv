`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * Native RAM slave. Harvard on-die memory for the fetch (read-only I-mem,
 * 64-bit, multi-outstanding) and LSU (byte-strobed D-mem, 32-bit,
 * single-outstanding) ports — no AXI. One parametric module serves both.
 *
 * The ports are FLAT parametric logic (sized by DATA_WIDTH / OUTSTANDING),
 * not the cpu_mem_req_t/cpu_mem_rsp_t packed structs: those structs are XLEN-wide
 * (32-bit rdata/wdata/wstrb) and cannot carry the 64-bit fetch word. The
 * struct types stay on the CPU/LSU side; the top level breaks the struct
 * into these flat ports at the instantiation site.
 *
 * Native protocol (valid/ready on both sides):
 *   - Request launch : req_valid_i && rsp_wready_o   (wready = can accept)
 *   - Read response  : rsp_rvalid_o && req_rready_i  (held until rready)
 *
 * OUTSTANDING (depth of the read response skid FIFO):
 *   - 1 (D-mem): single-outstanding. While an unread response is held,
 *     wready stays low (or high the cycle the master drains it, for
 *     back-to-back). Bit-matches the former rvalid_q/rdata_q behaviour.
 *   - 2 (I-mem): a second response can land and be held while the first is
 *     still queued, so fetch keeps the BSRAM issuing through the decode
 *     stalls that would otherwise idle a single-outstanding port. The fetch
 *     side backs this up with its own inflight<2 gate.
 *
 * Read timing (1-cycle BSRAM read, mirrors axi4_lite_ram):
 *   - On a read accept the BSRAM read launches (registered) and rvalid is
 *     raised the NEXT cycle, then HELD until rready (the "rvalid held under
 *     delayed rready" compliance fix, not a one-cycle pulse — a master not
 *     ready the cycle rvalid rises does not lose the data). 1-cycle latency
 *     when rready is already high.
 *
 * Store timing (D-mem only): commits at the accept cycle
 * (req_valid_i && wready && we). The byte-strobed BSRAM write fires that
 * same clock edge (posted store; the LSU retires on launch-accept, so
 * rsp_bvalid_o is held low — a native RAM has no B channel).
 *
 * READ_ONLY gates the write path: an I-mem instance (READ_ONLY=1) ignores
 * we/wdata/wstrb and never writes storage. Fetch never asserts we, so the
 * gate is belt-and-braces. The read path is identical for both modes.
 *
 * No address latch is needed: the read-launch (rd_d_q <= mem[word_addr])
 * and the write-commit happen AT the accept cycle, when req_addr_i is valid
 * by the launch handshake. The slave never needs addr after accept.
 *
 * Storage uses (* ram_style = "block" *) so Gowin infers a simple dual-port
 * BSRAM (one synchronous write port + one synchronous read port, single
 * clock). Byte-strobed writes use the BSRAM byte enables. Storage contents
 * are NOT reset (BSRAM has no clear); only the response FIFO / flags reset.
 * Simulation preloads via INIT_FILE.
 *
 * Naming: ports use *_i/_o; internals no prefix; flops _q.
 */

module native_ram #(
    // Address width in bits (depth = 2^ADDR_W bytes).
    parameter int ADDR_W = 16,
    // Data width in bits (32 for D-mem, 64 for the widened I-mem fetch).
    parameter int DATA_WIDTH = 32,
    // 1 = read-only I-mem (fetch); 0 = read/write D-mem (LSU, byte-strobed).
    parameter int READ_ONLY = 0,
    // Read response skid depth = max outstanding reads (1 for D-mem, 2 for
    // the 2-outstanding I-mem fetch). Caps how many responses can be held
    // while the master is not ready.
    parameter int OUTSTANDING = 1,
    // Optional $readmemh init file (relative to simulation working dir).
    parameter string INIT_FILE = ""
) (
    input wire clk_i,
    input wire rstn_i,

    // Request (master -> slave).
    input wire              req_valid_i,
    input wire              req_we_i,     // 1 = write, 0 = read
    input wire [  XLEN-1:0] req_addr_i,   // byte address
    input wire [DATA_W-1:0] req_wdata_i,  // write data (ignored if !we)
    input wire [STRB_W-1:0] req_wstrb_i,  // byte strobes (writes)
    input wire              req_rready_i, // master ready for read data

    // Response (slave -> master).
    output wire              rsp_wready_o,  // slave accepts the request
    output wire              rsp_rvalid_o,  // read data valid this cycle
    output wire [DATA_W-1:0] rsp_rdata_o,   // read data
    output wire              rsp_bvalid_o   // write-ack (held low: posted)
);

    // -----------------------------------------------------------------
    // Local params
    // -----------------------------------------------------------------
    localparam int DATA_W = DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;  // bytes per word
    localparam int BYTES_W = $clog2(STRB_W);  // byte-select bits
    localparam int WORD_ADDR_W = ADDR_W - BYTES_W;
    localparam int DEPTH_WORDS = 1 << WORD_ADDR_W;
    localparam int CNT_W = $clog2(OUTSTANDING + 1);  // 0..OUTSTANDING

`ifdef VERILATOR
    initial
        assert (STRB_W == $bits(req_wstrb_i))
        else $fatal(1, "STRB_W mismatch: DATA_WIDTH/8 vs req_wstrb_i width");
    initial
        assert (OUTSTANDING >= 1)
        else $fatal(1, "OUTSTANDING must be >= 1");
`endif

    // -----------------------------------------------------------------
    // Storage (BSRAM)
    // -----------------------------------------------------------------
    // ram_style is the Vivado/Xilinx spelling; GowinSynthesis reads
    // syn_ramstyle / syn_romstyle, so the first attribute alone was a no-op
    // there. syn_noprune keeps the tool from folding the array away.
    //
    // Depth comes from ADDR_W / DATA_WIDTH, not from the $readmemh content:
    // GowinSynthesis instantiates the array at its declared depth and the
    // image only has to fit. These attributes only keep the implementation
    // style predictable.
    (* ram_style = "block" *)
    (* syn_ramstyle = "block_ram" *)
    (* syn_romstyle = "block_rom" *)
    //(* syn_noprune = 1 *)
    logic [DATA_W-1:0] mem[DEPTH_WORDS];

    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, mem);
        end
    end

    // -----------------------------------------------------------------
    // Address decode (byte address -> word index). Valid only at the
    // accept cycle (req_valid_i && wready); never sampled after.
    // -----------------------------------------------------------------
    wire [WORD_ADDR_W-1:0] word_addr = req_addr_i[ADDR_W-1:BYTES_W];

    // -----------------------------------------------------------------
    // Read response skid FIFO (depth = OUTSTANDING).
    //
    // A read accepts this cycle and the BSRAM data is registered into
    // rd_d_q, arriving NEXT cycle (rif_q marks "read in flight, data
    // landing now"). The landing data is exposed the same cycle it arrives
    // (rvalid includes rif_q, rdata bypasses to rd_d_q when the FIFO is
    // empty) so a ready master sees 1-cycle read latency — and it is stored
    // into the shift FIFO for the cycles the master is NOT ready.
    //
    // The FIFO is a shift register: slot 0 is the head (read out), new
    // arrivals fill the first free slot from the head. count_q = occupied.
    // -----------------------------------------------------------------
    logic rif_q;  // read launched last cycle -> lands now
    logic [DATA_W-1:0] rd_d_q;  // registered BSRAM read data (lands now)

    logic [CNT_W-1:0] count_q;  // occupied FIFO slots (0..OUTSTANDING)
    logic [DATA_W-1:0] skid_q[OUTSTANDING];  // shift FIFO, slot 0 = head

    wire land = rif_q;  // data landing this cycle
    wire skid_pop = (count_q != '0) & req_rready_i;  // consume FIFO head
    wire land_consumed = land & (count_q == '0) & req_rready_i;  // landing data taken at once
    wire land_stored = land & ~land_consumed;  // landing data -> FIFO

    // rvalid/rdata: a response is valid if the FIFO holds one OR one lands
    // now. When the FIFO is empty the landing data is the head (bypass).
    assign rsp_rvalid_o = (count_q != '0) | land;
    assign rsp_rdata_o  = (count_q != '0) ? skid_q[0] : rd_d_q;
    assign rsp_bvalid_o = 1'b0;  // native RAM: no B channel (LSU posted store)

    // wready: accept a new request when, after this cycle's land+pop, there
    // is a free slot for the new arrival next cycle. new_count = count after
    // this cycle; a launch sets rif_next so its arrival needs new_count < N.
    // Uniform for reads and writes (the LSU serializes accesses, so blocking
    // a write while a read response is held matches the former single-entry
    // behaviour exactly). Depends only on registers (count_q, rif_q) + the
    // master's rready -> no combinational loop through the master.
    wire [CNT_W-1:0] new_count = count_q - skid_pop + land_stored;
    assign rsp_wready_o = (new_count < OUTSTANDING);

    // Launch handshake: req accepted this cycle.
    wire launch_hs = req_valid_i && rsp_wready_o;
    wire launch_read = launch_hs & ~req_we_i;
    wire launch_write = launch_hs & req_we_i;

    // Slot the landing data writes (first free from head). With a
    // simultaneous pop the shift frees slot (count-1); without it the free
    // slot is `count`. skid_pop is accounted for so pop+push coincide lands
    // the new item at the right place (see the shift below).
    wire [CNT_W-1:0] write_idx = count_q - skid_pop;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rif_q   <= 1'b0;
            rd_d_q  <= '0;
            count_q <= '0;
        end else begin
            // Read pipeline: on launch_read register the BSRAM word and mark
            // it in flight. Back-to-back launches keep rif_q high (each
            // cycle's launch re-arms it); rd_d_q is overwritten with the new
            // word while the previous landing word is consumed/stored this
            // same cycle (the push below reads the pre-edge rd_d_q).
            if (launch_read) begin
                rif_q  <= 1'b1;
                rd_d_q <= mem[word_addr];
            end else begin
                rif_q <= 1'b0;
            end

            // Shift FIFO: pop drains the head (shift left); a stored landing
            // word fills the freed tail slot. land_stored's write_idx is
            // computed AFTER the pop so a coincident pop+push places the new
            // item correctly; the land_stored write is emitted after the
            // shift so it wins the shared slot assignment.
            if (skid_pop) begin
                for (int i = 0; i < OUTSTANDING - 1; i++) begin
                    skid_q[i] <= skid_q[i+1];
                end
                skid_q[OUTSTANDING-1] <= '0;
            end
            if (land_stored) begin
                skid_q[write_idx] <= rd_d_q;
            end

            count_q <= new_count;
        end
    end

    // -----------------------------------------------------------------
    // Write path (D-mem only). Commits at the accept cycle: the
    // byte-strobed BSRAM write fires the same clock edge as launch_hs,
    // so a posted store retires and the data lands together. READ_ONLY
    // gates it off entirely (I-mem never writes).
    // -----------------------------------------------------------------
    generate
        if (!READ_ONLY) begin : gen_write
            always_ff @(posedge clk_i) begin
                if (launch_write) begin
                    for (integer i = 0; i < STRB_W; i++) begin
                        if (req_wstrb_i[i]) begin
                            mem[word_addr][8*i+:8] <= req_wdata_i[8*i+:8];
                        end
                    end
                end
            end
        end
    endgenerate

endmodule

`resetall

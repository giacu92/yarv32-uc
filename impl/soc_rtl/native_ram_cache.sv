`resetall
`timescale 1ns / 1ps
`default_nettype none

/**
 * Native yarv32_cache_pkg::mem_req_t / yarv32_cache_pkg::mem_rsp_t RAM slave. Harvard on-die memory for the
 * fetch (read-only I-mem) and LSU (byte-strobed D-mem) ports — no AXI.
 *
 * The native protocol is the one the pipeline stages already use:
 *   - Request launch : req.valid && rsp.wready   (rsp.wready = idle)
 *   - Read response  : rsp.rvalid && req.rready   (held until rready)
 * Single-outstanding: while an unread read response is held (rvalid_q=1
 * and the master has not yet asserted rready) wready stays low, so no
 * second request is accepted until the current response is consumed.
 *
 * Timing (mirrors axi4_lite_ram, the protocol-compliant gate):
 *   - Store: commits at the accept cycle (valid && wready && we). The
 *     byte-strobed BSRAM write fires that same clock edge. wready stays
 *     high the next cycle (no read pending), so back-to-back stores run
 *     at 1 cyc/store. The LSU's posted store retires on launch-accept
 *     (execute_stage.store_done = mem_launch_hs & we), so bvalid is not
 *     consumed here — mem_rsp_o.bvalid is held low (a native RAM has no
 *     B channel; the LSU does not wait on it).
 *   - Load : on a read accept the BSRAM read launches (registered) and
 *     rvalid is raised the NEXT cycle, then HELD until rready. This is the
 *     "rvalid held under delayed rready" compliance fix proven by ram_tb,
 *     not a one-cycle pulse — a master that is not ready the cycle rvalid
 *     rises does not lose the data. 1-cycle latency when rready is already
 *     high (the LSU holds rready=1 throughout EX_MEM_WAIT).
 *
 * No address latch is needed (unlike axi4_lite_master_bridge): both the
 * read-launch (rdata_q <= mem[word_addr]) and the write-commit happen AT
 * the accept cycle, when req.addr is valid by the launch handshake. The
 * slave never needs addr after accept, so a master that drops its request
 * the cycle after wready (allowed by the native convention, same as the
 * bridge tolerates) cannot corrupt the response.
 *
 * READ_ONLY gates the write path: an I-mem instance (READ_ONLY=1) ignores
 * we/wdata/wstrb and never writes storage. Fetch never asserts we, so the
 * gate is belt-and-braces. The read path is identical for both modes.
 *
 * Storage uses (* ram_style = "block" *) so Gowin infers a simple
 * dual-port BSRAM (one synchronous write port + one synchronous read
 * port, single clock). Byte-strobed writes cost BSRAM blocks on this
 * device (see BYTE_WRITE): Gowin has no byte write enable, so a
 * byte-writable array is split into byte-wide blocks.
 * Storage contents are NOT reset (BSRAM has no clear); only the response
 * registers (rvalid_q) reset. Simulation preloads via INIT_FILE.
 *
 * Naming: ports use *_i/_o; internals no prefix; flops _q.
 */

module native_ram #(
    // Address width in bits (depth = 2^ADDR_W bytes).
    parameter int ADDR_W = 16,
    // Data width in bits (must match XLEN / the req/rsp struct widths).
    parameter int DATA_WIDTH = 32,
    // Width of the request's addr field (byte address). The RAM decodes
    // only the low ADDR_W bits; extra MSBs are simply not sampled.
    parameter int REQ_ADDR_W = yarv32_cache_pkg::MEM_WIDTH,
    // 1 = read-only I-mem (fetch); 0 = read/write D-mem (LSU, byte-strobed).
    parameter bit READ_ONLY = 0,
    // 1 = per-byte write enables (wstrb selects which bytes commit).
    // 0 = whole-word writes only; wstrb must be all ones on a write.
    //
    // This is a RESOURCE decision, not a functional one. Gowin BSRAM has
    // no byte write enable, so GowinSynthesis implements one by splitting
    // the array into byte-wide blocks: a 256-bit cache line macro becomes
    // 32 BSRAMs (each holding 128 x 8 bits of an 18 kb block) instead of
    // 8. With four such macros that is 128 blocks against the GW2AR-18's
    // 46 -- RP0002, "the number of BSRAM in the design exceeds the
    // resource limit". Masters that need partial writes into a wide word
    // do the read-modify-write themselves (see cache_cntrl's store-hit
    // path, which merges into the line it already has registered).
    parameter bit BYTE_WRITE = 1,
    // Optional $readmemh init file (relative to simulation working dir).
    parameter string INIT_FILE = "",
    // Native-protocol struct pair, normally built with the
    // `YARV_MEM_TYPES macro at (REQ_ADDR_W, DATA_WIDTH). Defaults are the
    // CPU-width pair. Width safety is enforced by the elaboration checks
    // below (a mismatched REQ_T/RSP_T is caught, not silently truncated).
    parameter type REQ_T = yarv32_cache_pkg::mem_req_t,
    parameter type RSP_T = yarv32_cache_pkg::mem_rsp_t
) (
    input wire clk_i,
    input wire rstn_i,

    input  REQ_T mem_req_i,
    output RSP_T mem_rsp_o
);

    // -----------------------------------------------------------------
    // Local params
    // -----------------------------------------------------------------
    localparam int DATA_W = DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;  // bytes per word
    localparam int BYTES_W = $clog2(STRB_W);  // byte-select bits
    localparam int WORD_ADDR_W = ADDR_W - BYTES_W;
    localparam int DEPTH_WORDS = 1 << WORD_ADDR_W;

`ifdef VERILATOR
    // Port-struct widths come from the same DATA_WIDTH that sizes the
    // storage, so these can only fail if a caller hand-rolls a struct
    // instead of using the package macros.
    initial begin
        assert (DATA_WIDTH % 8 == 0)
        else $fatal(1, "DATA_WIDTH (%0d) is not a multiple of 8", DATA_WIDTH);
        assert ($bits(mem_req_i.wdata) == DATA_WIDTH)
        else
            $fatal(
                1, "req.wdata width (%0d) != DATA_WIDTH (%0d)", $bits(mem_req_i.wdata), DATA_WIDTH
            );
        assert ($bits(mem_req_i.wstrb) == STRB_W)
        else
            $fatal(
                1, "req.wstrb width (%0d) != DATA_WIDTH/8 (%0d)", $bits(mem_req_i.wstrb), STRB_W
            );
        assert ($bits(mem_rsp_o.rdata) == DATA_WIDTH)
        else
            $fatal(
                1, "rsp.rdata width (%0d) != DATA_WIDTH (%0d)", $bits(mem_rsp_o.rdata), DATA_WIDTH
            );
    end
`endif

    // -----------------------------------------------------------------
    // Storage (BSRAM)
    // -----------------------------------------------------------------
    // ram_style is the Vivado/Xilinx spelling; GowinSynthesis reads
    // syn_ramstyle / syn_romstyle, so the first attribute alone was a no-op
    // there. syn_noprune keeps the tool from folding the array away.
    //
    // None of these fix a depth reduction on their own: an uninitialised
    // word is a constant, so GowinSynthesis is entitled to build a
    // read-only array only as deep as its $readmemh content and let the
    // upper address bits alias. Padding the image with a real instruction
    // word (see the firmware Makefiles' IMEM_PAD_VALUE) is what actually
    // pins the depth; these attributes only keep the implementation style
    // predictable.
    (* ram_style = "block" *)
    (* syn_ramstyle = "block_ram" *)
    (* syn_romstyle = "block_rom" *)
    (* syn_noprune = 1 *)
    logic [DATA_W-1:0] mem[DEPTH_WORDS];

    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, mem);
        end
    end

`ifdef VERILATOR
`ifndef NO_SIM_PLUSARGS
    // +RAM_GARBAGE fills the array with junk before time 0. Simulation
    // otherwise hands out zeros for memory nobody wrote, which is how a
    // design that trusts its RAM's power-up state passes here and hangs on
    // a device — the cache's tag valid bits were exactly that bug. A test
    // that passes with this plusarg does not depend on the accident.
    // NO_SIM_PLUSARGS excludes this block from the sv2v/yosys paths
    // (`make lint-yosys`, `make gatesim`): they preprocess with
    // -DVERILATOR to bypass the rPLL, and yosys cannot read either the
    // $test$plusargs call or sv2v's rendering of a size cast. The random
    // value goes through a sized variable for the same reason.
    logic [31:0] garbage_rnd;

    initial begin
        if ($test$plusargs("RAM_GARBAGE") && INIT_FILE == "") begin
            for (int gi = 0; gi < DEPTH_WORDS; gi++) begin
                for (int gb = 0; gb < DATA_W; gb++) begin
                    garbage_rnd = $random();
                    mem[gi][gb] = garbage_rnd[0];
                end
            end
        end
    end
`endif
`endif

    // -----------------------------------------------------------------
    // Address decode (byte address -> word index). Valid only at the
    // accept cycle (req.valid && wready); never sampled after.
    // -----------------------------------------------------------------
    wire  [WORD_ADDR_W-1:0] word_addr = mem_req_i.addr[ADDR_W-1:BYTES_W];

    // -----------------------------------------------------------------
    // Read response register: rvalid held until rready (compliance).
    // rvalid_q is also the single-outstanding busy flag for reads.
    // -----------------------------------------------------------------
    logic                   rvalid_q;
    logic [     DATA_W-1:0] rdata_q;

    // wready: accept a new request when no unread response is held, OR
    // while the master is draining the current one (back-to-back). Depends
    // on the rvalid_q flop and the master's rready only (the master's
    // rready never depends on this wready -> no combinational loop, same
    // property axi4_lite_ram's arready relies on).
    assign mem_rsp_o.wready = !rvalid_q || (rvalid_q && mem_req_i.rready);
    assign mem_rsp_o.rvalid = rvalid_q;
    assign mem_rsp_o.rdata  = rdata_q;
    assign mem_rsp_o.bvalid = 1'b0;  // native RAM: no B channel (LSU posted)

    // Launch handshake: req accepted this cycle.
    wire launch_hs = mem_req_i.valid && mem_rsp_o.wready;
    wire launch_read = launch_hs & ~mem_req_i.we;
    wire launch_write = launch_hs & mem_req_i.we;

    // Read response consumed: rvalid held, master ready, no new read
    // landing this same cycle (a new read overwrites rdata_q safely —
    // wready was high only because rready was, so the old response is
    // being drained).
    wire rsp_done = rvalid_q & mem_req_i.rready & ~launch_read;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q <= 1'b0;
            rdata_q  <= '0;
        end else begin
            if (launch_read) begin
                // Launch the BSRAM read; data registered, rvalid next
                // cycle. Held until rready (rsp_done below clears it).
                rvalid_q <= 1'b1;
                rdata_q  <= mem[word_addr];
            end else if (rsp_done) begin
                rvalid_q <= 1'b0;
            end
        end
    end

    // -----------------------------------------------------------------
    // Write path (D-mem only). Commits at the accept cycle: the
    // byte-strobed BSRAM write fires the same clock edge as launch_hs,
    // so a posted store retires and the data lands together. READ_ONLY
    // gates it off entirely (I-mem never writes).
    // -----------------------------------------------------------------
    generate
        if (!READ_ONLY && BYTE_WRITE) begin : gen_write
            always_ff @(posedge clk_i) begin
                if (launch_write) begin
                    for (integer i = 0; i < STRB_W; i++) begin
                        if (mem_req_i.wstrb[i]) begin
                            mem[word_addr][8*i+:8] <= mem_req_i.wdata[8*i+:8];
                        end
                    end
                end
            end
        end else if (!READ_ONLY) begin : gen_write_word
            // Whole-word write: one BSRAM write enable, no byte lanes.
            always_ff @(posedge clk_i) begin
                if (launch_write) begin
                    mem[word_addr] <= mem_req_i.wdata;
                end
            end

`ifdef VERILATOR
            // A partial strobe here would silently commit the unstrobed
            // bytes too, which is exactly the bug BYTE_WRITE=0 invites.
            always_ff @(posedge clk_i) begin
                if (launch_write) begin
                    assert (&mem_req_i.wstrb)
                    else
                        $fatal(
                            1, "native_ram: partial wstrb (%b) with BYTE_WRITE=0", mem_req_i.wstrb
                        );
                end
            end
`endif
        end
    endgenerate

endmodule

`resetall

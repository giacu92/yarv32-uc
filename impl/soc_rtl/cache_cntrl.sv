`resetall
`timescale 1ns / 1ps
`default_nettype none

/**
 * Cache line size: 32 byte
 * I-Cache size: 8 KiB - D-Cache size: 8 KiB = 16 KiB total
 * 8 MiB SDRAM (GW2AR Internal) --> Address is 23 bit wide
 * addr = {tag, set_idx, offset} (classic bit-slice set index)
 * CPU access width is 64 bit (doubleword select, addr[NBIT_OFFSET-1:3])
 * I-mem port: read-only, up to 2 outstanding reads (fetch instruction buffer)
 *
 * Naming: ports use *_i/_o; internals no prefix; flops _q.
 */

module cache_cntrl #(
    // Main memory size
    parameter int MEM_SIZE = 23,  // 2^23 B = 8 MiB
    // Cache line width in bits
    parameter int CL_SIZE = 5,  // Cache line is 2^5 = 32 bytes (DATA_WIDTH)
    // Number of way (caches are set-associative)
    parameter int N_WAY = 2,
    // Cache width in bits (depth = 2^CACHE_SIZE bytes). Both have same size
    parameter int CACHE_SIZE = 13,  // 2^13 = 8 KiB
    // System clock frequency in MHz. Only used to space the SDRAM refresh
    // bursts (sdram_controller's CYCLES_BETWEEN_REFRESH); it must match the
    // real clk_i rate or the SDRAM is refreshed too rarely (data loss) or
    // too often (wasted bandwidth). Sim runs at 100 MHz (10 ns period in
    // sim_top); the Tang Nano 20K build passes 50 (see fpga_top).
    parameter int CLK_FREQ_MHZ = 100,
    // SDRAM power-up wait, in microseconds. JEDEC SDRAM (and the GW2AR's
    // embedded die) requires stable clock and NOP-only for at least 100 us
    // after power-up before it will accept PRECHARGE / REFRESH / MRS. The
    // controller only counts 15 cycles of its own (300 ns at 50 MHz), so
    // its reset is held here until the wait has elapsed; while it is in
    // reset it drives NOP, which is exactly what the chip wants to see.
    // Skipping this is invisible in simulation but leaves a real device
    // with an unprogrammed mode register and undefined reads.
    parameter int SDRAM_INIT_US = 200,
    // Optional $readmemh image for the bootrom (2 KiB, 256 x 64 bit).
    // Empty means an uninitialised macro: legal, but it reads back
    // whatever the device powered up with.
    parameter string BOOTROM_FILE = "",
    // Power-on value of the control register (see yarv32_cache_pkg::CSR_W / yarv32_cache_pkg::CSR_BIT_BYPASS
    // in the package). Default 0: caches enabled, bypass off — the
    // behaviour of the design before the register existed.
    parameter logic [yarv32_cache_pkg::CSR_W-1:0] CSR_RST_VAL = '0
) (
    input wire clk_i,
    input wire rstn_i,

    // Clock forwarded to the SDRAM's own clock pin. On the FPGA this is a
    // phase-shifted copy of clk_i (the chip samples the command/data pins
    // on ITS clock edge, so the shift is what buys setup/hold margin
    // across the SIP wiring); in sim it is simply clk_i.
    input wire sdram_clk_i,

    // ICACHE Interface
    input  yarv32_cache_pkg::mem_req_t icache_req_i,
    output yarv32_cache_pkg::mem_rsp_t icache_rsp_o,

    // DCACHE Interface
    input  yarv32_cache_pkg::mem_req_t dcache_req_i,
    output yarv32_cache_pkg::mem_rsp_t dcache_rsp_o,

    // fence.i (M7). Makes every store the D port has accepted visible to
    // the I port: on completion every dirty D-line has been written back
    // to SDRAM (D-lines stay valid, now clean) and every I-cache line is
    // invalid, so the next fetch of any address refills from SDRAM.
    //   fencei_req_i : LEVEL. Raise it and hold it until fencei_ack_o is
    //                  seen high; drop it after that. A request is only
    //                  recognised again once req has been low for at least
    //                  one cycle (so a req still high in the ack cycle, as
    //                  a requester that drops it on the following cycle
    //                  leaves it, does not start a second flush).
    //   fencei_ack_o : one-cycle PULSE, from a flop.
    // While req is high both ports' wready are held low; the flush starts
    // only once every request already accepted has completed (no skid slot
    // occupied, miss FSM idle), so misses and posted stores in flight are
    // drained first, never interrupted. Responses already queued for I-port
    // reads accepted BEFORE the request are still delivered in order (the
    // native protocol requires it); they are pre-fence fetches and the
    // requester discards them (the CPU's fence.i redirect kills them).
    // Duration: 2 cycles per set (128 sets) plus ~20 per dirty line.
    input  wire fencei_req_i,
    output wire fencei_ack_o,

    // SDRAM Interface (external pins, driven by the internal sdram_controller)
    output wire        sdram_clk_o,    // SDRAM clock (TODO: PLL phase shift for the FPGA)
    output wire        sdram_cke_o,    // O_sdram_cke
    output wire        sdram_cs_n_o,   // O_sdram_cs_n
    output wire        sdram_cas_n_o,  // O_sdram_cas_n
    output wire        sdram_ras_n_o,  // O_sdram_ras_n
    output wire        sdram_wen_n_o,  // O_sdram_wen_n
    output wire [ 3:0] sdram_dqm_o,    // O_sdram_dqm
    output wire [10:0] sdram_addr_o,   // O_sdram_addr
    output wire [ 1:0] sdram_ba_o,     // O_sdram_ba
    inout  wire [31:0] sdram_dq_io,    // IO_sdram_dq

    // Debug: the miss FSM's current state (fsm_state_e). Board bring-up
    // has no other way to see where a miss got stuck — see fpga_top, which
    // puts this on the LEDs when the self test fails.
    output wire [3:0] dbg_state_o,

    // Debug: why the D port is (not) accepting. A stalled port waits either
    // on its own skid slot or on the miss FSM, and these bits say which:
    //   [3] slot 0 occupied   [2] lookup launched for it
    //   [1] slot missed, waiting for the FSM to pick it up
    //   [0] the response queue holds an unconsumed entry
    output wire [3:0] dbg_dport_o,

    // Debug: D-cache event counters, saturating at 15, in the order a
    // request passes through them — lookups launched, tag answers seen,
    // misses picked up by the FSM, misses unstalled. The final state alone
    // cannot say whether a wedged port never got an answer, got one and
    // lost it, or completed a refill that failed to free the slot; these
    // separate the three in one run.
    //   [3:0] lookups  [7:4] tag answers  [11:8] misses  [15:12] unstalls
    output wire [15:0] dbg_cnt_o,

    // Debug: D-port accepts, saturating at 15. A control counter — the
    // skid slot cannot be occupied without an accept, so a zero here means
    // the counters (or the path that reports them) are lying, not that the
    // event never happened.
    output wire [3:0] dbg_acc_o,

    // Debug, live: the lookup issue path.
    //   [3] cache_lookup_go[1]   [2] fsm_lookup_gate[1]
    //   [1] dtag_req[0].valid    [0] dtag_rsp[0].wready
    output wire [3:0] dbg_go_o,

    // Debug, live: the macro answers this cache is waiting for.
    //   [3] dtag_rsp[0].rvalid   [2] dmem_rsp_d[0].rvalid
    //   [1] slot_rsp_q[1][0]     [0] slot_rsp_q[1][1]
    output wire [3:0] dbg_rsp_o,

    // Debug: free-running tick, incremented every clock with no condition
    // attached. It answers the one question every other counter leaves
    // open — whether this module's flops are advancing at all. A value
    // that never changes between report lines means no clock or a reset
    // held low, and every zero counter elsewhere then says nothing about
    // the design.
    output wire [3:0] dbg_tick_o,

    // Debug: a slow bit of the same free-running tick, for an LED. It
    // reaches a pin through nothing but a flop and a wire — no counters,
    // no report path, no UART. If this LED does not blink while the
    // board's other one does, this module is not being clocked, and that
    // conclusion depends on no other logic being correct.
    output wire dbg_hb_o
);

    // ===================================================================
    // Local params
    // ===================================================================

    // N_LINES: Number of CACHE lines = 256
    localparam int N_LINES = 1 << (CACHE_SIZE - CL_SIZE);
    localparam int N_SETS = N_LINES / N_WAY;

    // NBIT_OFFSET: 32 B Cache lines --> 5 bit
    localparam int NBIT_OFFSET = CL_SIZE;
    // NBIT_SET_IDX: 128 sets --> 7 bit
    localparam int NBIT_SET_IDX = $clog2(N_SETS);

    // {tag, set, offset} split + valid + dirty
    localparam int NBIT_TAG = MEM_SIZE - NBIT_SET_IDX - NBIT_OFFSET + 2;

    // TAG_DATA_W: Round up to the next multiple of 8 (byte-aligned)
    localparam int TAG_DATA_W = ((NBIT_TAG + 7) / 8) * 8;

    // DATA_WIDTH: CL_SIZE Bytes --> 256 bit (32 bytes, 8x32bit words)
    localparam int DATA_WIDTH = 1 << (CL_SIZE + 3);

    // Data macros are MACRO_W = 64 bits wide, NOT one line wide (2026-11-22,
    // M4): a line is BEATS consecutive 64-bit words of the way macro, word
    // address = {set, beat} = addr[WAY_ADDR_W-1:3]. A 256-bit-wide macro
    // costs 8 BSRAM blocks whatever its depth (a block is at most 36 bits
    // wide) and drags 256-bit muxes through the hit, store-merge, victim and
    // commit paths; at 64 bits every lookup returns exactly the CPU's
    // doubleword, and a line moves through the FSM one beat per cycle.
    localparam int MACRO_W = yarv32_cache_pkg::MEM_WIDTH;
    localparam int BEATS = DATA_WIDTH / MACRO_W;  // beats per line (4)
    localparam int BEAT_W = $clog2(BEATS);
    // SDRAM words per line: one controller burst moves a whole line.
    localparam int BURST_LEN = DATA_WIDTH / 32;  // 32-bit SDRAM data bus (8)

    // Tag word layout: [NBIT_TAG-1:2] = tag bits, [1] = dirty, [0] = valid
    localparam int TAG_FIELD_W = NBIT_TAG - 2;

    // NBIT_WAY: way-select bits (N_WAY must be a power of 2)
    localparam int NBIT_WAY = $clog2(N_WAY);

    // WAY_ADDR_W: per-way data macro address width. Halves (by NBIT_WAY)
    // vs. a single direct-mapped macro, so N_WAY * 2**WAY_ADDR_W bytes
    // still equals 2**CACHE_SIZE bytes total.
    localparam int WAY_ADDR_W = CACHE_SIZE - NBIT_WAY;

    // TAG_BYTES_W: native_ram treats mem_req_i.addr as a BYTE address and
    // computes word_addr = addr[ADDR_W-1:BYTES_W], dropping the low
    // BYTES_W bits as an intra-word byte select. TAG_DATA_W can be >8
    // bits (STRB_W>1), so set_idx must be left-shifted by this many bits
    // before being placed in itag_req/dtag_req.addr — otherwise the
    // dropped LSB(s) come out of set_idx itself and consecutive sets
    // alias to the same tag word.
    localparam int TAG_BYTES_W = $clog2(TAG_DATA_W / 8);

    // Tag macro address width: NBIT_SET_IDX word-select bits + TAG_BYTES_W
    // intra-word byte-select bits, so the macro stores one tag word per set
    // (native_ram decodes word_addr = addr[TAG_ADDR_W-1:TAG_BYTES_W] =
    // set_idx). An ADDR_W of NBIT_SET_IDX alone would drop the byte-select
    // bits out of set_idx and alias sets 2**TAG_BYTES_W apart onto one tag
    // word (false hits / clobbered tags).
    localparam int TAG_ADDR_W = NBIT_SET_IDX + TAG_BYTES_W;

    // Per-instance protocol types, re-expanded from the package macros at
    // this module's own geometry (the package yarv32_cache_pkg::mem_req_t/yarv32_cache_pkg::cache_req_t are
    // the fixed-width instances of the same macros). way_*_t is one whole
    // cache line wide; tag_*_t is one tag word wide. This keeps the
    // native_ram port widths and the arrays below matched by construction
    // for any CL_SIZE / N_WAY / CACHE_SIZE parameterization.
    `YARV_MEM_TYPES(way_req_t, way_rsp_t, yarv32_cache_pkg::MEM_WIDTH, MACRO_W)
    `YARV_MEM_TYPES(tag_req_t, tag_rsp_t, yarv32_cache_pkg::MEM_WIDTH, TAG_DATA_W)

`ifdef VERILATOR
    // Elaboration-time geometry check: the tag macros must have one word
    // per set (see TAG_ADDR_W above).
    initial begin
        assert ((1 << (TAG_ADDR_W - TAG_BYTES_W)) >= N_SETS)
        else
            $fatal(
                1,
                "tag macro word depth (%0d) < N_SETS (%0d)",
                1 << (TAG_ADDR_W - TAG_BYTES_W),
                N_SETS
            );
    end
`endif

    // ===================================================================
    // Signal declarations
    // ===================================================================

    yarv32_cache_pkg::mem_req_t bootr_req;  // towards bootrom
    yarv32_cache_pkg::mem_rsp_t bootr_rsp;  // from bootrom

    way_req_t [N_WAY-1:0] imem_req;  // towards icache ways
    way_rsp_t [N_WAY-1:0] imem_rsp_d;  // from icache ways
    way_req_t [N_WAY-1:0] dmem_req;  // towards dcache ways
    way_rsp_t [N_WAY-1:0] dmem_rsp_d;  // from dcache ways

    tag_req_t [N_WAY-1:0] itag_req;  // towards itag ways
    tag_rsp_t [N_WAY-1:0] itag_rsp;  // from itag ways
    tag_req_t [N_WAY-1:0] dtag_req;  // towards dtag ways
    tag_rsp_t [N_WAY-1:0] dtag_rsp;  // from dtag ways

    // SDRAM controller host interface (sdram_controller, src/ips submodule,
    // m6-burst branch): a read is one BURST_LEN-word burst (one ACTIVATE,
    // one READ), its words arriving as one-cycle rd_ready pulses on
    // rd_data; a write is one word, or BURST_LEN words streamed to
    // consecutive columns (wr_burst). A request is taken on the cycle its
    // rd_ack / wr_ack is high; words 1.. of a streamed write are taken on
    // wr_next pulses.
    logic sdram_rd_en;
    logic sdram_wr_en;
    logic sdram_wr_burst;
    logic sdram_rd_ack;
    logic sdram_wr_ack;
    logic sdram_wr_next;
    logic sdram_busy;
    logic sdram_rd_ready;
    logic [20:0] sdram_rd_addr;  // 32-bit word address {bank, row, col}
    logic [20:0] sdram_wr_addr;
    logic [31:0] sdram_rd_data;
    logic [31:0] sdram_wr_data;

    // Per-cache request skid, slot-indexed (freed slots are reused as a
    // free list; at most one slot is ever un-launched, so launch order =
    // accept order and responses stay in order). A CPU request is accepted
    // into a free slot at the launch handshake; the address split, the
    // tag/data macro lookups, and the tag compare all run off the
    // REGISTERED request, so the 1-cycle-old tag RAM output is compared
    // against the address that launched the lookup — no stale-tag race when
    // the CPU changes the address after accept. Indexed by CACHE, not by
    // way: set_idx/tag are broadcast to all N_WAY ways of a cache.
    // 0 = icache, 1 = dcache.
    localparam int N_CACHE = 2;
    localparam int N_SLOT = 2;

    // Outstanding-request limit per cache: 2 for the read-only I-port (the
    // fetch unit's 2 in-flight reads), 1 for the D-port (single-outstanding).
    localparam int SKID_DEPTH[N_CACHE] = '{2, 1};

    yarv32_cache_pkg::mem_req_t [N_CACHE-1:0][N_SLOT-1:0] skid_q;  // accepted requests (skid slots)
    logic [N_CACHE-1:0][N_SLOT-1:0] skid_valid_q;  // slot occupied
    // Which slot holds the OLDER request while both are occupied. Slots are
    // reused as a free list, so slot index says nothing about age: a younger
    // request lands in slot 0 whenever slot 0 frees first. Every decision
    // that orders responses must use this bit, never the index (2026-11-27,
    // the I-port out-of-order bug found by the SoC integration).
    logic [N_CACHE-1:0] old_slot_q;
    // Skid slots read through a selector, each mux built ONCE as an
    // explicit 2:1 on constant indices (see "Skid read selectors" below).
    yarv32_cache_pkg::mem_req_t [N_CACHE-1:0] rsp_req;  // rsp_req[c]
    yarv32_cache_pkg::mem_req_t [N_CACHE-1:0] nc_req;  // nc_req[c]
    yarv32_cache_pkg::mem_req_t latch_req;  // latch_req
    logic [N_CACHE-1:0][N_SLOT-1:0] slot_lookup_q;  // lookup launched for this slot
    logic [N_CACHE-1:0][N_SLOT-1:0] slot_rsp_q;  // this slot's tag answer arrives now
    logic [N_CACHE-1:0][N_SLOT-1:0] miss_seen_q;  // slot missed; waiting for FSM pickup
    logic [N_CACHE-1:0][N_SLOT-1:0] slot_miss_q;  // slot's miss taken by the FSM:
    // blocks younger queue entries until the miss completes (unstall)

    // Response queue (per cache): captured hit data, delivered to the CPU in
    // accept order. rvalid is a LEVEL held until rready pops the head —
    // protocol compliance, not a one-cycle lookup pulse.
    logic [N_CACHE-1:0][N_SLOT-1:0][yarv32_cache_pkg::MEM_WIDTH-1:0]
        rq_q;  // rq_q[c][0] is the head
    logic [N_CACHE-1:0][N_SLOT-1:0] rq_blk_q;  // entry waits behind an older miss
    logic [N_CACHE-1:0][1:0] rq_cnt_q;  // entries in the queue (0..2)

    // Lookup compare context, registered at lookup launch. With back-to-back
    // I-port lookups the address split has already moved on to the next slot
    // when a tag answer arrives, so the compare runs against this registered
    // copy, not the live split.
    logic [N_CACHE-1:0][TAG_FIELD_W-1:0] cmp_tag_q;
    logic [N_CACHE-1:0] cmp_we_q;

    // Slot selection (per cache, always_comb — Verilator's V3Delayed chokes
    // on multiple calls of the same automatic function from a clocked
    // block). At most one slot is ever un-launched, so "first matching
    // slot" is unambiguous. The 0 defaults are unreachable whenever the
    // corresponding strobe (accept / lookup_go / rsp pulse / fsm latch) is
    // asserted.
    // Index widths are MINIMAL, never int. A 32-bit index into a packed
    // array asks the tool what the other four billion positions mean:
    // simulation clamps to the value in range, synthesis is free to build
    // something else, and the two stop agreeing. sv2v made that visible as
    // hundreds of "range select out of bounds" warnings on dmem_req.
    localparam int SLOT_IDX_W = $clog2(N_SLOT);

    logic [N_CACHE-1:0][SLOT_IDX_W-1:0] slot_free;  // first free slot (skid full only if !wready)
    logic [N_CACHE-1:0][SLOT_IDX_W-1:0] slot_lookup_sel;  // slot to launch the lookup for
    logic [N_CACHE-1:0][SLOT_IDX_W-1:0] slot_rsp_sel;  // slot the tag answer belongs to
    logic [N_CACHE-1:0][SLOT_IDX_W-1:0] slot_miss_sel;  // oldest slot the FSM will pick up
    logic [N_CACHE-1:0] slot_miss_wait;  // a slot awaits FSM pickup
    logic [N_CACHE-1:0][2:0] slot_outstanding;  // occupied slots + queue entries

    // -------------------------------------------------------------
    // Request target, decoded from the address at ACCEPT and latched per
    // slot. Decoding once, at accept, is what makes the control register
    // safe to write: a request already in flight keeps the target it was
    // accepted with, so flipping CACHE_BYPASS cannot re-route it halfway.
    //
    //   TGT_CACHE : SDRAM through the cache (tag lookup, refill, the lot)
    //   TGT_MEM   : SDRAM, cache bypassed — the miss FSM moves the
    //               doubleword straight to/from the device
    //   TGT_BOOT  : bootrom read (a write is a posted no-op: it is a ROM)
    //   TGT_CSR   : control register read/write
    // -------------------------------------------------------------
    localparam logic [1:0] TGT_CACHE = 2'd0;
    localparam logic [1:0] TGT_MEM = 2'd1;
    localparam logic [1:0] TGT_BOOT = 2'd2;
    localparam logic [1:0] TGT_CSR = 2'd3;

    logic [N_CACHE-1:0][N_SLOT-1:0][1:0] slot_tgt_q;  // target of each occupied slot
    logic [N_CACHE-1:0][1:0] acc_tgt;  // target of the request being offered now

    // The uncached targets share one rule: a slot holding one is the only
    // thing outstanding on its port (see the wready gating), so ordering
    // against cached responses needs no extra machinery — there is nothing
    // to order against.
    logic [N_CACHE-1:0] nc_busy;  // this port holds an uncached slot
    logic [N_CACHE-1:0][SLOT_IDX_W-1:0] nc_sel;  // which slot that is
    logic [N_CACHE-1:0][1:0] nc_tgt;  // its target
    logic [N_CACHE-1:0] nc_start;  // it has not been serviced yet
    logic [N_CACHE-1:0] nc_free;  // it completes this cycle: free the slot
    logic [N_CACHE-1:0] nc_push;  // ... and pushes a response
    logic [N_CACHE-1:0][yarv32_cache_pkg::MEM_WIDTH-1:0] nc_push_data;
    logic [N_CACHE-1:0] nc_boot_go;  // bootrom read granted this cycle
    logic [N_CACHE-1:0] nc_byp_go;  // bypass slot handed to the miss FSM

    // Bootrom arbitration state: which port owns the read in flight.
    logic boot_busy_q;
    logic boot_owner_q;

    // Control register (yarv32_cache_pkg::CSR_W bits, see the package for the bit map).
    logic [yarv32_cache_pkg::CSR_W-1:0] csr_q;
    logic csr_wr;
    wire cache_bypass = csr_q[yarv32_cache_pkg::CSR_BIT_BYPASS];

    // Victim state for a missed slot, captured at its lookup pulse (the only
    // cycle the tag answers are guaranteed current — a later lookup on the
    // tag macro overwrites rdata_q). Round-robin victim selection per set:
    // prefer an invalid way, else the rr pointer, which advances on every
    // miss of that set.
    // Round-robin victim pointer, one bit per set (M4b step B): held in a
    // small synchronous-read memory per cache (gen_rr, below) rather than
    // in N_SETS flops behind an N_SETS:1 read mux. Read at LOOKUP LAUNCH
    // into rr_rd_q -- the same one-cycle-ahead registration cmp_tag_q gets
    // -- so the value is ready at the tag answer; toggled at a miss answer.
    logic [N_CACHE-1:0] rr_rd_q;
    logic [N_CACHE-1:0][N_SLOT-1:0][NBIT_WAY-1:0] victim_way_q;
    logic [N_CACHE-1:0][N_SLOT-1:0] victim_valid_q;
    logic [N_CACHE-1:0][N_SLOT-1:0] victim_dirty_q;
    logic [N_CACHE-1:0][N_SLOT-1:0][TAG_FIELD_W-1:0] victim_tag_q;

    logic [N_CACHE-1:0][NBIT_SET_IDX-1:0]
        rsp_set;  // set of the slot whose lookup answer is arriving
    logic [N_CACHE-1:0][NBIT_WAY-1:0] rsp_victim_way;  // victim way chosen for that answer

    // Miss-FSM macro access + unstall handshake with the skid/queue logic
    // above. All are driven by the FSM section below (or the store-hit path
    // next to it); declared here so the macro request muxes and the skid
    // always_ff can reference them.
    logic [N_CACHE-1:0] fsm_lookup_gate;  // FSM owns this cache's macros: hold lookups
    logic [N_CACHE-1:0] fsm_unstall;  // completed miss frees its slot (one cycle)
    logic [$clog2(N_SLOT)-1:0] fsm_unstall_slot;  // slot being unstalled
    logic [N_CACHE-1:0] fsm_rsp_push;  // completed miss was a load: push its response
    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] fsm_rsp_data;  // response data from the refilled line
    logic [NBIT_WAY-1:0] fsm_victim_way;  // victim way the FSM operates on
    way_req_t fsm_way_req;  // FSM request to the data macros
    tag_req_t fsm_tag_req;  // FSM request to the tag macros
    logic fsm_imem_access, fsm_dmem_access;  // FSM drives the data macros
    logic fsm_itag_write, fsm_dtag_write;  // FSM drives the tag macros
    logic [MACRO_W-1:0] fsm_victim_beat;  // victim beat read back for WB
    logic [MACRO_W-1:0] commit_beat;  // refilled beat (+ store merge)
    logic miss_is_store;  // the missed request is a store (write-allocate)
    logic dcache_store_hit;  // posted store hit: data+tag writes land now
    logic [NBIT_WAY-1:0] dhit_way;  // hit way of the store above
    way_req_t dstore_way_req;
    tag_req_t dstore_tag_req;

    // fence.i flush state (M7), driven by the miss FSM section.
    logic fi_q, fi_d;  // the FSM is running a flush (owns both caches)
    logic [NBIT_SET_IDX-1:0] fi_set_q, fi_set_d;  // set being scanned
    logic [NBIT_WAY-1:0] fi_way_q, fi_way_d;  // dirty D way being written back
    logic [TAG_FIELD_W-1:0] fi_tag_q, fi_tag_d;  // ... and its tag
    logic fi_wait_low_q;  // acked: ignore req until it has been seen low
    logic fi_tag_scan;  // S_FI_TAG: I tags invalidated, D tags read
    logic fi_tag_clean;  // S_FI_CLEAN: written-back D way's tag rewritten clean
    tag_req_t fi_itag_req, fi_dtag_req;

    always_comb begin
        for (int c = 0; c < N_CACHE; c++) begin
            // A depth-1 port (the D-port) only ever uses slot 0 and queue
            // entry 0: every selector below is tied to 0 for it, so slot 1's
            // and entry 1's flops never load and synthesis drops them
            // (M4b). The VERILATOR asserts at the bottom of the file check
            // that the tied-off state is in fact never reached.
            if (!skid_valid_q[c][0]) slot_free[c] = 0;
            else if ((SKID_DEPTH[c] > 1) && !skid_valid_q[c][1]) slot_free[c] = 1;
            else slot_free[c] = 0;  // full: unreachable while wready=1

            if (skid_valid_q[c][0] && !slot_lookup_q[c][0]) slot_lookup_sel[c] = 0;
            else if (skid_valid_q[c][1] && !slot_lookup_q[c][1]) slot_lookup_sel[c] = 1;
            else slot_lookup_sel[c] = 0;  // no pending lookup; content unused

            if (slot_rsp_q[c][0]) slot_rsp_sel[c] = 0;
            else if (slot_rsp_q[c][1]) slot_rsp_sel[c] = 1;
            else slot_rsp_sel[c] = 0;

            // FSM pickup: the OLDEST waiting miss. With both slots waiting
            // (two misses, e.g. two doublewords of one missing line) the
            // index-first choice served the younger one first and its
            // response took the older request's place in the queue.
            if ((skid_valid_q[c][0] && miss_seen_q[c][0]) &&
                (skid_valid_q[c][1] && miss_seen_q[c][1]))
                slot_miss_sel[c] = old_slot_q[c];
            else if (skid_valid_q[c][0] && miss_seen_q[c][0]) slot_miss_sel[c] = 0;
            else if (skid_valid_q[c][1] && miss_seen_q[c][1]) slot_miss_sel[c] = 1;
            else slot_miss_sel[c] = 0;

            slot_miss_wait[c] = (skid_valid_q[c][0] && miss_seen_q[c][0]) ||
                (skid_valid_q[c][1] && miss_seen_q[c][1]);

            if (SKID_DEPTH[c] == 1) begin
                slot_lookup_sel[c] = 0;
                slot_rsp_sel[c]    = 0;
                slot_miss_sel[c]   = 0;
            end

            // Outstanding units: occupied skid slots plus unconsumed queue
            // entries. Every accepted read stays exactly one unit until the
            // CPU consumes it, so the queue can never overflow SKID_DEPTH.
            slot_outstanding[c] = 3'(skid_valid_q[c][0]) + 3'(skid_valid_q[c][1]) + 3'(rq_cnt_q[c]);

            // Context of the lookup answer arriving this cycle (if any):
            // its set and the victim way chosen for it. The tag macros'
            // rdata is only current while their rvalid pulses, so both are
            // consumed by the miss-pulse capture below, never sampled cold.
            rsp_set[c] = rsp_req[c].addr[NBIT_OFFSET+:NBIT_SET_IDX];
            rsp_victim_way[c] = NBIT_WAY'(rr_rd_q[c]);
            for (int i = N_WAY - 1; i >= 0; i--) begin
                if ((c == 0) ? !itag_rsp[i].rdata[0] : !dtag_rsp[i].rdata[0]) begin
                    rsp_victim_way[c] = NBIT_WAY'(i);
                end
            end
        end
    end

    // ===================================================================
    // Uncached targets: decode, bootrom arbitration, control register
    // ===================================================================

    function automatic logic [1:0] req_target(input logic [yarv32_cache_pkg::MEM_WIDTH-1:0] a,
                                              input logic byp);
        case (yarv32_cache_pkg::yarv_region(
            a
        ))
            yarv32_cache_pkg::RGN_BOOT: req_target = TGT_BOOT;
            yarv32_cache_pkg::RGN_CSR:  req_target = TGT_CSR;
            default:  req_target = byp ? TGT_MEM : TGT_CACHE;
        endcase
    endfunction

    // What a skid slot actually stores (M4b, 2026-11-22). Only the fields
    // and bits something reads are kept; the rest are stored as constant
    // zero so synthesis drops their flops and every mux behind them:
    //   - addr: bits above yarv32_cache_pkg::SYS_MEM_BIT are ignored by the address map
    //     (documented in yarv32_cache_pkg), so only [yarv32_cache_pkg::SYS_MEM_BIT:0] is kept;
    //   - valid / rready: never read from a slot;
    //   - I-port wdata / wstrb: the I-port is read-only, a store there is a
    //     posted no-op that never uses its data.
    // The CPU-facing protocol is unchanged: these are internal copies.
    localparam int KEEP_ADDR_W = yarv32_cache_pkg::SYS_MEM_BIT + 1;
    yarv32_cache_pkg::mem_req_t [N_CACHE-1:0] acc_req;
    always_comb begin
        acc_req[0] = '0;
        acc_req[0].we = icache_req_i.we;
        acc_req[0].addr = {
            {(yarv32_cache_pkg::MEM_WIDTH - KEEP_ADDR_W) {1'b0}}, icache_req_i.addr[KEEP_ADDR_W-1:0]
        };
        acc_req[1] = '0;
        acc_req[1].we = dcache_req_i.we;
        acc_req[1].addr = {
            {(yarv32_cache_pkg::MEM_WIDTH - KEEP_ADDR_W) {1'b0}}, dcache_req_i.addr[KEEP_ADDR_W-1:0]
        };
        acc_req[1].wdata = dcache_req_i.wdata;
        acc_req[1].wstrb = dcache_req_i.wstrb;
    end

    assign acc_tgt[0] = req_target(icache_req_i.addr, cache_bypass);
    assign acc_tgt[1] = req_target(dcache_req_i.addr, cache_bypass);

    // The uncached slot of each port (at most one, by the wready gating).
    always_comb begin
        for (int c = 0; c < N_CACHE; c++) begin
            nc_busy[c] = 1'b0;
            nc_sel[c]  = '0;
            for (int sl = N_SLOT - 1; sl >= 0; sl--) begin
                if (skid_valid_q[c][sl] && (slot_tgt_q[c][sl] != TGT_CACHE)) begin
                    nc_busy[c] = 1'b1;
                    nc_sel[c]  = SLOT_IDX_W'(sl);
                end
            end
            nc_tgt[c]   = slot_tgt_q[c][nc_sel[c]];
            // Not serviced yet. slot_lookup_q is reused as "started": the
            // bootrom read has been launched, or the bypass has been handed
            // to the miss FSM. Single-cycle targets (the register, a write
            // to the ROM) free the slot instead and never set it.
            nc_start[c] = nc_busy[c] && !slot_lookup_q[c][nc_sel[c]];
        end
    end

    // Control register: one cycle, no memory behind it. Only the D port
    // may write (the I port is read-only by spec, so a store there is a
    // posted no-op); both may read.
    logic [N_CACHE-1:0] nc_csr_done;
    assign nc_csr_done[0] = nc_start[0] && (nc_tgt[0] == TGT_CSR);
    assign nc_csr_done[1] = nc_start[1] && (nc_tgt[1] == TGT_CSR);
    assign csr_wr         = nc_csr_done[1] && nc_req[1].we && nc_req[1].wstrb[0];

    // Bootrom: read-only, so a store to it retires with no side effect.
    logic [N_CACHE-1:0] nc_boot_wr;
    assign nc_boot_wr[0] = nc_start[0] && (nc_tgt[0] == TGT_BOOT) && nc_req[0].we;
    assign nc_boot_wr[1] = nc_start[1] && (nc_tgt[1] == TGT_BOOT) && nc_req[1].we;

    // Bootrom read: one macro, two ports. D wins ties, the same fixed
    // priority the miss FSM uses; the loser simply retries next cycle
    // (nc_start stays asserted until its own grant).
    logic [N_CACHE-1:0] boot_rd_req;
    assign boot_rd_req[0] = nc_start[0] && (nc_tgt[0] == TGT_BOOT) && !nc_req[0].we;
    assign boot_rd_req[1] = nc_start[1] && (nc_tgt[1] == TGT_BOOT) && !nc_req[1].we;

    assign nc_boot_go[1]  = boot_rd_req[1] && !boot_busy_q && bootr_rsp.wready;
    assign nc_boot_go[0]  = boot_rd_req[0] && !boot_busy_q && bootr_rsp.wready && !nc_boot_go[1];

    always_comb begin
        bootr_req        = '0;
        bootr_req.valid  = |nc_boot_go;
        bootr_req.we     = 1'b0;
        bootr_req.addr   = nc_boot_go[1] ? nc_req[1].addr : nc_req[0].addr;
        bootr_req.rready = 1'b1;
    end

    // The macro answers one cycle after the launch and holds rvalid until
    // rready, which is tied high here — so this is the completion cycle.
    wire boot_done = boot_busy_q && bootr_rsp.rvalid;
    logic [N_CACHE-1:0] boot_done_c;
    assign boot_done_c[0] = boot_done && (boot_owner_q == 1'b0);
    assign boot_done_c[1] = boot_done && (boot_owner_q == 1'b1);

    // Cache bypass: hand the slot to the miss FSM, which moves the
    // doubleword to/from the SDRAM without touching the cache arrays.
    // miss_seen_q is the same hand-off a real miss uses.
    assign nc_byp_go[0]   = nc_start[0] && (nc_tgt[0] == TGT_MEM);
    assign nc_byp_go[1]   = nc_start[1] && (nc_tgt[1] == TGT_MEM);

    // Completion: which uncached slots retire this cycle, and which of
    // them owe the CPU a response. The bypass path does NOT appear here —
    // it retires through the FSM's unstall, like a miss.
    always_comb begin
        for (int c = 0; c < N_CACHE; c++) begin
            nc_free[c] = nc_csr_done[c] || nc_boot_wr[c] || boot_done_c[c];
            nc_push[c] = boot_done_c[c] || (nc_csr_done[c] && !nc_req[c].we);
            nc_push_data[c] = boot_done_c[c] ?
                bootr_rsp.rdata : {{(yarv32_cache_pkg::MEM_WIDTH - yarv32_cache_pkg::CSR_W) {1'b0}}, csr_q};
        end
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            csr_q        <= CSR_RST_VAL;
            boot_busy_q  <= 1'b0;
            boot_owner_q <= 1'b0;
        end else begin
            if (csr_wr) csr_q <= nc_req[1].wdata[yarv32_cache_pkg::CSR_W-1:0];
            if (|nc_boot_go) begin
                boot_busy_q  <= 1'b1;
                boot_owner_q <= nc_boot_go[1];
            end else if (boot_done) begin
                boot_busy_q <= 1'b0;
            end
        end
    end

    // The address split runs off the skid slot being launched (see skid_q).
    yarv32_cache_pkg::mem_req_t [N_CACHE-1:0] cache_req;

    always_comb begin
        cache_req[0] = (slot_lookup_sel[0] != '0) ? skid_q[0][1] : skid_q[0][0];
        cache_req[1] = skid_q[1][0];
    end

    // Lookup issue: exactly one tag+data macro lookup per accepted request
    // (slot_lookup_q gates relaunch while the CPU holds rready=1).
    logic [N_CACHE-1:0] cache_lookup_go;

    always_comb begin
        for (int c = 0; c < N_CACHE; c++) begin
            // Hold lookups while the miss FSM is accessing this cache's
            // data/tag macros (fsm_lookup_gate): a lookup launch would
            // clobber the victim-line read held in the macro's rdata_q, or
            // race the line/tag commit. Lookups on the OTHER cache and
            // accepts/wready are unaffected — hits-under-miss resume during
            // the (long) refill states, which do not touch the macros.
            //
            // Only TGT_CACHE slots take this path: an uncached slot is
            // serviced by the bootrom/register/bypass logic instead, and a
            // tag lookup for it would be a lookup of an address that has no
            // tag (0x80_0000 aliases onto set 0 of the cache).
            cache_lookup_go[c] =
                ((skid_valid_q[c][0] && !slot_lookup_q[c][0] && (slot_tgt_q[c][0] == TGT_CACHE)) ||
                 (skid_valid_q[c][1] && !slot_lookup_q[c][1] && (slot_tgt_q[c][1] == TGT_CACHE))) &&
                !fsm_lookup_gate[c];
        end
    end

    // One response pulse per lookup: itag_rsp/dtag_rsp rvalid are identical
    // across ways (same broadcast lookup, same native_ram latency), so way 0
    // is a valid representative.
    logic [N_CACHE-1:0] cache_rsp_pulse;
    logic [N_CACHE-1:0] cache_hit_pulse;
    // Qualified by an outstanding lookup (slot_rsp_q): the fence.i flush
    // also reads the D tags, and its answers must not be taken for a CPU
    // lookup's (a phantom store hit or round-robin step otherwise).
    assign cache_rsp_pulse[0] = itag_rsp[0].rvalid && (|slot_rsp_q[0]);
    assign cache_rsp_pulse[1] = dtag_rsp[0].rvalid && (|slot_rsp_q[1]);
    assign cache_hit_pulse[0] = cache_rsp_pulse[0] && icache_hit;
    assign cache_hit_pulse[1] = cache_rsp_pulse[1] && dcache_hit;

    logic [N_CACHE-1:0][NBIT_OFFSET-1:0] offset;
    logic [N_CACHE-1:0][NBIT_SET_IDX-1:0] set_idx;
    logic [N_CACHE-1:0][NBIT_TAG-3:0] tag;
    // Doubleword index within a line (64-bit granularity):
    // 2**(CL_SIZE-3) doublewords, selected by addr[NBIT_OFFSET-1:3]

    // Tag RAM read decode (per way)
    logic [N_WAY-1:0] icache_valid;
    logic [N_WAY-1:0] dcache_valid;
    logic [N_WAY-1:0] icache_dirty;
    logic [N_WAY-1:0] dcache_dirty;
    logic [N_WAY-1:0][TAG_FIELD_W-1:0] icache_tag_stored;
    logic [N_WAY-1:0][TAG_FIELD_W-1:0] dcache_tag_stored;
    logic [N_WAY-1:0] icache_way_hit;
    logic [N_WAY-1:0] dcache_way_hit;

    logic icache_hit;
    logic dcache_hit;


    // ===================================================================
    // Module declarations
    // ===================================================================

    // Bootrom, 2 KiB read-only, mapped at yarv32_cache_pkg::BOOTROM_BASE (0x80_0000).
    //
    // Both CPU ports reach it: the fetch side runs boot code out of it,
    // and the load side reads the payload the boot code copies into
    // SDRAM. The macro has one port, so the two are arbitrated (D wins
    // ties, same rule as the miss FSM) in the uncached-request section
    // below, which drives bootr_req and consumes bootr_rsp.
    native_ram #(
        .ADDR_W    (yarv32_cache_pkg::BOOTROM_ADDR_W),               // 2 KiB
        .DATA_WIDTH(yarv32_cache_pkg::MEM_WIDTH),
        .READ_ONLY (1),
        .INIT_FILE (BOOTROM_FILE)
    ) u_bootrom (
        .clk_i    (clk_i),
        .rstn_i   (rstn_i),
        .mem_req_i(bootr_req),
        .mem_rsp_o(bootr_rsp)
    );

    // ICACHE / DCACHE / I-TAG / D-TAG: N_WAY parallel macros, one per way,
    // looked up in parallel on every request. Data macros are WAY_ADDR_W
    // wide (halved vs. a single direct-mapped macro at N_WAY=2); tag
    // macros are TAG_ADDR_W wide (one tag word per set, see TAG_ADDR_W).
    //
    // All eight are BYTE_WRITE(0): every write here commits a whole word.
    // Gowin BSRAM has no byte write enable, so a byte-writable 256-bit
    // line macro is built out of 32 byte-wide blocks instead of 8 —
    // 128 blocks for the four data macros alone, against 46 on the
    // GW2AR-18 (synthesis error RP0002). The only partial write in the
    // design is the D-cache store hit, and that path merges into the line
    // it has already read out (see dstore_way_req below).
    genvar w;
    generate
        for (w = 0; w < N_WAY; w++) begin : gen_way

            native_ram #(
                .ADDR_W    (WAY_ADDR_W),
                .DATA_WIDTH(MACRO_W),
                .REQ_T     (way_req_t),
                .RSP_T     (way_rsp_t),
                .READ_ONLY (0),
                .BYTE_WRITE(0),           // whole-line writes only, see the store-hit merge
                .INIT_FILE ("")
            ) u_icache (
                .clk_i    (clk_i),
                .rstn_i   (rstn_i),
                .mem_req_i(imem_req[w]),
                .mem_rsp_o(imem_rsp_d[w])
            );

            native_ram #(
                .ADDR_W    (WAY_ADDR_W),
                .DATA_WIDTH(MACRO_W),
                .REQ_T     (way_req_t),
                .RSP_T     (way_rsp_t),
                .READ_ONLY (0),
                .BYTE_WRITE(0),           // whole-line writes only, see the store-hit merge
                .INIT_FILE ("")
            ) u_dcache (
                .clk_i    (clk_i),
                .rstn_i   (rstn_i),
                .mem_req_i(dmem_req[w]),
                .mem_rsp_o(dmem_rsp_d[w])
            );

            native_ram #(
                .ADDR_W    (TAG_ADDR_W),
                .DATA_WIDTH(TAG_DATA_W),
                .REQ_T     (tag_req_t),
                .RSP_T     (tag_rsp_t),
                .READ_ONLY (0),
                .BYTE_WRITE(0),           // whole-line writes only, see the store-hit merge
                .INIT_FILE ("")
            ) u_itag (
                .clk_i    (clk_i),
                .rstn_i   (rstn_i),
                .mem_req_i(itag_req[w]),
                .mem_rsp_o(itag_rsp[w])
            );

            native_ram #(
                .ADDR_W    (TAG_ADDR_W),
                .DATA_WIDTH(TAG_DATA_W),
                .REQ_T     (tag_req_t),
                .RSP_T     (tag_rsp_t),
                .READ_ONLY (0),
                .BYTE_WRITE(0),           // whole-line writes only, see the store-hit merge
                .INIT_FILE ("")
            ) u_dtag (
                .clk_i    (clk_i),
                .rstn_i   (rstn_i),
                .mem_req_i(dtag_req[w]),
                .mem_rsp_o(dtag_rsp[w])
            );

        end
    endgenerate

    // ===================================================================
    // Tag invalidation sweep
    //
    // A cache may not trust the state its tag memory wakes up in. Nothing
    // in this design ever cleared the valid bits: simulation passed only
    // because an uninitialised array reads as zero there, and the board
    // was relying on whatever the tool happens to do with BSRAM contents
    // at configuration. In a 4-state gate-level run the same tags read X,
    // the hit/miss decision became X, and the X reached the skid slot's
    // valid bit and wedged the port — which is the failure the board
    // shows.
    //
    // So: after reset, walk every set and write valid=0 into every way of
    // both caches (the four tag macros are independent, so one set per
    // cycle covers all of them), and hold both ports' wready low until the
    // sweep is done. N_SETS cycles, once, at reset.
    // ===================================================================
    logic [NBIT_SET_IDX:0] tag_init_cnt_q;
    wire tag_init_done = tag_init_cnt_q[NBIT_SET_IDX];
    wire [NBIT_SET_IDX-1:0] tag_init_set = tag_init_cnt_q[NBIT_SET_IDX-1:0];

    always_ff @(posedge clk_i) begin
        if (!rstn_i) tag_init_cnt_q <= '0;
        else if (!tag_init_done) tag_init_cnt_q <= tag_init_cnt_q + 1'b1;
    end

    // Round-robin memories (see rr_rd_q). No reset port -- the sweep above
    // writes every set to 0 while the ports are held, exactly as it clears
    // the tags, so the first victim of every set is way 0. A lookup
    // launched in the cycle a miss toggles the same set reads the old bit;
    // that only shifts the replacement order, never correctness (the
    // victim is always a way of the right set).
    genvar gc;
    generate
        for (gc = 0; gc < N_CACHE; gc++) begin : gen_rr
            (* ram_style = "distributed" *) (* syn_ramstyle = "distributed_ram" *)
            logic rr_mem[N_SETS];
            wire rr_we = !tag_init_done || (cache_rsp_pulse[gc] && !cache_hit_pulse[gc]);
            wire [NBIT_SET_IDX-1:0] rr_wa = !tag_init_done ? tag_init_set : rsp_set[gc];
            wire rr_wd = tag_init_done && !rr_rd_q[gc];
            always_ff @(posedge clk_i) begin
                if (rr_we) rr_mem[rr_wa] <= rr_wd;
                if (cache_lookup_go[gc]) rr_rd_q[gc] <= rr_mem[set_idx[gc]];
            end
        end
    endgenerate

    // SDRAM power-up hold: the controller's reset is released only after
    // SDRAM_INIT_US of stable clock (see the parameter). The counter is
    // sized from the same CLK_FREQ_MHZ that spaces the refreshes, so both
    // follow the real clock rate.
    localparam int INIT_WAIT_CYCLES = CLK_FREQ_MHZ * SDRAM_INIT_US;
    localparam int INIT_CNT_W = $clog2(INIT_WAIT_CYCLES + 1);

    logic [INIT_CNT_W-1:0] init_cnt_q;
    wire init_done = (init_cnt_q == INIT_CNT_W'(INIT_WAIT_CYCLES));
    wire sdrc_rstn = rstn_i && init_done;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) init_cnt_q <= '0;
        else if (!init_done) init_cnt_q <= init_cnt_q + 1'b1;
    end

    // SDRAM controller (src/ips/sdram-controller submodule, BSD; the
    // gw2ar-32bit branch adapts it to the GW2AR-18 embedded SDRAM: 32-bit
    // data, row/col/bank 11/8/2, CL=3; its m6-burst branch adds burst reads
    // of a whole line (BURST_LEN = 8 words = 32 B, one ACTIVATE) and
    // streamed line writes, auto-precharge, refresh handled internally).
    sdram_controller #(
        .ROW_WIDTH    (11),
        .COL_WIDTH    (8),
        .BANK_WIDTH   (2),
        .DATA_WIDTH   (32),
        .BURST_LEN    (BURST_LEN),
        .CLK_FREQUENCY(CLK_FREQ_MHZ)
    ) u_sdram_cntrl (
        .wr_addr     (sdram_wr_addr),
        .wr_data     (sdram_wr_data),
        .wr_enable   (sdram_wr_en),
        .wr_burst    (sdram_wr_burst),
        .wr_ack      (sdram_wr_ack),
        .wr_next     (sdram_wr_next),
        .rd_addr     (sdram_rd_addr),
        .rd_data     (sdram_rd_data),
        .rd_ready    (sdram_rd_ready),
        .rd_enable   (sdram_rd_en),
        .rd_ack      (sdram_rd_ack),
        .busy        (sdram_busy),
        .rst_n       (sdrc_rstn),
        .clk         (clk_i),
        .addr        (sdram_addr_o),
        .bank_addr   (sdram_ba_o),
        .data        (sdram_dq_io),
        .clock_enable(sdram_cke_o),
        .cs_n        (sdram_cs_n_o),
        .ras_n       (sdram_ras_n_o),
        .cas_n       (sdram_cas_n_o),
        .we_n        (sdram_wen_n_o),
        .dqm         (sdram_dqm_o)
    );

    assign sdram_clk_o = sdram_clk_i;


    // ===================================================================
    // Cache controller logic (hit/miss, refill, write-back, arbitration)
    // ===================================================================

    // Address split per cache (classic bit-slice): addr = {tag, set, offset}.
    always_comb begin
        for (int c = 0; c < N_CACHE; c++) begin
            offset[c]  = cache_req[c].addr[NBIT_OFFSET-1:0];
            set_idx[c] = cache_req[c].addr[NBIT_OFFSET+:NBIT_SET_IDX];
            tag[c]     = cache_req[c].addr[MEM_SIZE-1-:TAG_FIELD_W];
        end
    end

    // -------------------------------------------------------------
    // Tag RAM read requests (one lookup per accepted request, from req_q)
    // -------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < N_WAY; i++) begin
            itag_req[i] = '0;
            itag_req[i].valid = cache_lookup_go[0];
            itag_req[i].we = 1'b0;  // lookup only, tag write handled by refill FSM
            // set_idx shifted left by TAG_BYTES_W: native_ram drops the low
            // BYTES_W bits of addr as byte-select, not as part of the set
            // index (see TAG_BYTES_W comment above).
            itag_req[i].addr = {
                {(yarv32_cache_pkg::MEM_WIDTH - NBIT_SET_IDX - TAG_BYTES_W) {1'b0}},
                set_idx[0],
                {TAG_BYTES_W{1'b0}}
            };
            itag_req[i].rready = 1'b1;

            dtag_req[i] = '0;
            dtag_req[i].valid = cache_lookup_go[1];
            dtag_req[i].we = 1'b0;
            dtag_req[i].addr = {
                {(yarv32_cache_pkg::MEM_WIDTH - NBIT_SET_IDX - TAG_BYTES_W) {1'b0}},
                set_idx[1],
                {TAG_BYTES_W{1'b0}}
            };
            dtag_req[i].rready = 1'b1;
        end

        // Invalidation sweep owns the tag macros until it is done. It runs
        // before any request can be accepted (wready is low), so nothing
        // else is driving them here.
        if (!tag_init_done) begin
            for (int i = 0; i < N_WAY; i++) begin
                itag_req[i] = '0;
                itag_req[i].valid = 1'b1;
                itag_req[i].we = 1'b1;
                itag_req[i].addr = {
                    {(yarv32_cache_pkg::MEM_WIDTH - NBIT_SET_IDX - TAG_BYTES_W) {1'b0}},
                    tag_init_set,
                    {TAG_BYTES_W{1'b0}}
                };
                itag_req[i].wdata = '0;  // valid = 0, dirty = 0, tag = 0
                itag_req[i].wstrb = {(TAG_DATA_W / 8) {1'b1}};
                itag_req[i].rready = 1'b1;

                dtag_req[i] = itag_req[i];
            end
        end

        // fence.i flush tag accesses (S_FI_TAG / S_FI_CLEAN). Lookups on
        // both caches are gated for the whole flush and no store hit can
        // pulse (every port was drained before it started).
        if (fi_tag_scan) begin
            for (int w = 0; w < N_WAY; w++) begin
                itag_req[w] = fi_itag_req;
                dtag_req[w] = fi_dtag_req;
            end
        end
        for (int w = 0; w < N_WAY; w++)
        if (fi_tag_clean && (fi_way_q == NBIT_WAY'(w))) dtag_req[w] = fi_dtag_req;

        // Miss-FSM tag write (last refill beat): store the refilled line's tag
        // into the victim way. CPU lookups for this cache are gated off in
        // that state (fsm_lookup_gate), so the mux below never has to
        // arbitrate two drivers.
        //
        // Written as a loop over CONSTANT way indices compared against the
        // selector, never as req[sel] = ...: a dynamic-index write into a
        // packed struct array is lowered (sv2v + yosys) to one wide
        // priority mux per bit with a 32-bit index compare behind it, and
        // these four blocks alone measured ~70% of cache_cntrl's coarse
        // cells (M4, 2026-11-22). Same function, explicit 2:1 per way.
        for (int w = 0; w < N_WAY; w++)
        if (fsm_itag_write && (fsm_victim_way == NBIT_WAY'(w))) itag_req[w] = fsm_tag_req;

        // Posted store hit: set the dirty bit in the hit way's tag word the
        // same cycle the tag answer pulses (D-port is single-outstanding, so
        // no CPU lookup can be launching in this cycle). Else: miss-FSM tag
        // write as above.
        for (int w = 0; w < N_WAY; w++) begin
            if (dcache_store_hit) begin
                if (dhit_way == NBIT_WAY'(w)) dtag_req[w] = dstore_tag_req;
            end else if (fsm_dtag_write && (fsm_victim_way == NBIT_WAY'(w))) begin
                dtag_req[w] = fsm_tag_req;
            end
        end
    end

    // -------------------------------------------------------------
    // Tag compare: hit only once the tag RAM response is valid
    // -------------------------------------------------------------
    always_comb begin
        icache_hit = 1'b0;
        dcache_hit = 1'b0;
        for (int i = 0; i < N_WAY; i++) begin
            icache_valid[i] = itag_rsp[i].rdata[0];
            icache_dirty[i] = itag_rsp[i].rdata[1];
            icache_tag_stored[i] = itag_rsp[i].rdata[2+:TAG_FIELD_W];
            icache_way_hit[i] = itag_rsp[i].rvalid && icache_valid[i] &&
                (icache_tag_stored[i] == cmp_tag_q[0]);
            icache_hit |= icache_way_hit[i];

            dcache_valid[i] = dtag_rsp[i].rdata[0];
            dcache_dirty[i] = dtag_rsp[i].rdata[1];
            dcache_tag_stored[i] = dtag_rsp[i].rdata[2+:TAG_FIELD_W];
            dcache_way_hit[i] = dtag_rsp[i].rvalid && dcache_valid[i] &&
                (dcache_tag_stored[i] == cmp_tag_q[1]);
            dcache_hit |= dcache_way_hit[i];
        end
    end

    // -------------------------------------------------------------
    // ICACHE/DCACHE speculative read, launched with the tag lookup
    // (one per accepted request). If hit, data is already available in the
    // imem_rsp_q/dmem_rsp_q stage one cycle later. If miss, discard read
    // data.
    // -------------------------------------------------------------
    // One address and one write-data value per CACHE, shared by its ways;
    // only valid / we are per way (M4b, 2026-11-27). The three requesters of
    // a cache's macros never overlap: lookups are gated while the FSM owns
    // the macros (fsm_lookup_gate), and a posted D store hit cannot pulse
    // while the FSM services a D miss (the D port's single slot is the
    // miss). A write is always a whole doubleword (BYTE_WRITE 0).
    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] imem_addr, dmem_addr;

    always_comb begin
        imem_addr = fsm_imem_access ? fsm_way_req.addr : cache_req[0].addr;
        dmem_addr = dcache_store_hit ? dstore_way_req.addr :
            fsm_dmem_access ? fsm_way_req.addr : cache_req[1].addr;

        for (int w = 0; w < N_WAY; w++) begin
            imem_req[w]        = '0;
            imem_req[w].addr   = imem_addr;
            imem_req[w].wdata  = fsm_way_req.wdata;
            imem_req[w].wstrb  = {(MACRO_W / 8) {1'b1}};
            imem_req[w].rready = 1'b1;
            if (fsm_imem_access) begin
                imem_req[w].valid = fsm_way_req.valid && (fsm_victim_way == NBIT_WAY'(w));
                imem_req[w].we    = fsm_way_req.we;
            end else begin
                imem_req[w].valid = cache_lookup_go[0];
            end

            dmem_req[w]        = '0;
            dmem_req[w].addr   = dmem_addr;
            dmem_req[w].wdata  = dcache_store_hit ? dstore_way_req.wdata : fsm_way_req.wdata;
            dmem_req[w].wstrb  = {(MACRO_W / 8) {1'b1}};
            dmem_req[w].rready = 1'b1;
            if (dcache_store_hit) begin
                dmem_req[w].valid = (dhit_way == NBIT_WAY'(w));
                dmem_req[w].we    = 1'b1;
            end else if (fsm_dmem_access) begin
                dmem_req[w].valid = fsm_way_req.valid && (fsm_victim_way == NBIT_WAY'(w));
                dmem_req[w].we    = fsm_way_req.we;
            end else begin
                dmem_req[w].valid = cache_lookup_go[1];
            end
        end
    end

    // -------------------------------------------------------------
    // Hit-way line mux over the RAW data-macro outputs. A response is
    // captured into the queue at the tag-answer cycle (the same cycle the
    // lookup pulse fires), one stage earlier than the old imem_rsp_q path.
    // -------------------------------------------------------------
    logic [MACRO_W-1:0] icache_line, dcache_line;

    always_comb begin
        icache_line = '0;
        dcache_line = '0;
        for (int i = 0; i < N_WAY; i++) begin
            if (icache_way_hit[i]) icache_line = imem_rsp_d[i].rdata;
            if (dcache_way_hit[i]) dcache_line = dmem_rsp_d[i].rdata;
        end
    end

    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] icache_push_data, dcache_push_data;
    // The data macro is 64 bits wide and addressed with the doubleword
    // bits, so the hit way's output IS the requested doubleword.
    assign icache_push_data = icache_line;
    assign dcache_push_data = dcache_line;

    // -------------------------------------------------------------
    // Skid + response queue state (per cache):
    //   accept    : raw CPU request latched into the first free slot
    //   lookup_go : launches the one tag+data lookup for that slot and
    //               registers the compare context (cmp_tag_q/
    //               cmp_we_q) off the launching slot's split
    //   rsp pulse : the lookup's tag answer. A hit frees the slot; a load
    //               hit also pushes {rdata, blk} onto the response queue.
    //               A store hit is a posted store — no response (the
    //               data-macro write is TODO Phase 4). A miss flags
    //               miss_seen_q for the FSM and keeps the slot occupied so
    //               no younger response can pass an older miss.
    //   pop       : the CPU consumed the queue head (rready)
    //   fsm_latch : the miss FSM took the slot over: miss_seen_q clears,
    //               slot_miss_q sets. slot_miss_q stays set (blocking
    //               younger entries) until the miss completes and the
    //               requester is unstalled (TODO Phase 4); the slot counts
    //               as outstanding the whole time, so wready stays low on
    //               the single-outstanding D-port.
    // -------------------------------------------------------------
    logic [N_CACHE-1:0] cache_accept;
    assign cache_accept[0] = icache_req_i.valid && icache_rsp_o.wready;
    assign cache_accept[1] = dcache_req_i.valid && dcache_rsp_o.wready;

    // CPU consumed the queue head.
    logic [N_CACHE-1:0] cache_pop;
    assign cache_pop[0] = icache_req_i.rready && icache_rsp_o.rvalid;
    assign cache_pop[1] = dcache_req_i.rready && dcache_rsp_o.rvalid;

    // Queue push: the tag answer was a load hit, or an uncached read
    // (bootrom / control register) completed. The two can never coincide —
    // an uncached slot is alone on its port — so one queue port serves both.
    logic [N_CACHE-1:0] cache_push;
    assign cache_push[0] = (cache_rsp_pulse[0] && cache_hit_pulse[0] && !cmp_we_q[0]) || nc_push[0];
    assign cache_push[1] = (cache_rsp_pulse[1] && cache_hit_pulse[1] && !cmp_we_q[1]) || nc_push[1];

    logic [N_CACHE-1:0][yarv32_cache_pkg::MEM_WIDTH-1:0] cache_push_data;
    assign cache_push_data[0] = nc_push[0] ? nc_push_data[0] : icache_push_data;
    assign cache_push_data[1] = nc_push[1] ? nc_push_data[1] : dcache_push_data;

    // An unresolved miss (pending pickup or owned by the FSM) older than
    // the entry being pushed blocks that entry: responses must be delivered
    // in accept order (the fetch unit's instruction buffer relies on it).
    logic [N_CACHE-1:0] older_miss;
    assign older_miss[0] = (|slot_miss_q[0]) || slot_miss_wait[0];
    assign older_miss[1] = (|slot_miss_q[1]) || slot_miss_wait[1];

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            for (int c = 0; c < N_CACHE; c++) begin
                skid_q[c][0]         <= '0;
                skid_q[c][1]         <= '0;
                skid_valid_q[c][0]   <= 1'b0;
                skid_valid_q[c][1]   <= 1'b0;
                old_slot_q[c]        <= 1'b0;
                slot_lookup_q[c][0]  <= 1'b0;
                slot_lookup_q[c][1]  <= 1'b0;
                slot_rsp_q[c][0]     <= 1'b0;
                slot_rsp_q[c][1]     <= 1'b0;
                miss_seen_q[c][0]    <= 1'b0;
                miss_seen_q[c][1]    <= 1'b0;
                slot_miss_q[c][0]    <= 1'b0;
                slot_miss_q[c][1]    <= 1'b0;
                slot_tgt_q[c][0]     <= TGT_CACHE;
                slot_tgt_q[c][1]     <= TGT_CACHE;
                rq_q[c][0]           <= '0;
                rq_q[c][1]           <= '0;
                rq_blk_q[c][0]       <= 1'b0;
                rq_blk_q[c][1]       <= 1'b0;
                rq_cnt_q[c]          <= '0;
                cmp_tag_q[c]         <= '0;
                cmp_we_q[c]          <= 1'b0;
                victim_way_q[c][0]   <= '0;
                victim_way_q[c][1]   <= '0;
                victim_valid_q[c][0] <= 1'b0;
                victim_valid_q[c][1] <= 1'b0;
                victim_dirty_q[c][0] <= 1'b0;
                victim_dirty_q[c][1] <= 1'b0;
                victim_tag_q[c][0]   <= '0;
                victim_tag_q[c][1]   <= '0;
            end
        end else begin
            for (int c = 0; c < N_CACHE; c++) begin
                // Accept: fill the first free slot (wready guarantees one).
                if (cache_accept[c]) begin
                    for (int s = 0; s < N_SLOT; s++) begin
                        if (slot_free[c] == SLOT_IDX_W'(s)) skid_q[c][s] <= acc_req[c];
                    end
                    // The new request is the younger one if the other slot
                    // is still occupied (and not being freed this cycle by
                    // anything but the accept itself); otherwise it is the
                    // only, hence oldest, request.
                    old_slot_q[c] <= skid_valid_q[c][~slot_free[c]] ? ~slot_free[c] : slot_free[c];
                    skid_valid_q[c][slot_free[c]] <= 1'b1;
                    slot_lookup_q[c][slot_free[c]] <= 1'b0;
                    slot_rsp_q[c][slot_free[c]] <= 1'b0;
                    miss_seen_q[c][slot_free[c]] <= 1'b0;
                    slot_miss_q[c][slot_free[c]] <= 1'b0;
                    // Target decoded from the address (and the bypass bit
                    // as it stands NOW) and frozen for the slot's lifetime.
                    slot_tgt_q[c][slot_free[c]] <= acc_tgt[c];
                end

                // Uncached slot started: the bootrom read has been launched,
                // or the bypass request handed to the miss FSM. Reuses
                // slot_lookup_q as the "already started" flag, so the
                // service logic fires exactly once per request, the same
                // property cache_lookup_go gives a cached one.
                if (nc_boot_go[c] || nc_byp_go[c]) slot_lookup_q[c][nc_sel[c]] <= 1'b1;
                if (nc_byp_go[c]) miss_seen_q[c][nc_sel[c]] <= 1'b1;

                // Uncached slot retired (register access, or a write to the
                // ROM, or the bootrom read answered). The bypass path does
                // not come through here: it retires at the FSM's unstall.
                if (nc_free[c]) skid_valid_q[c][nc_sel[c]] <= 1'b0;

                // Lookup launch for the (single) un-launched slot.
                if (cache_lookup_go[c]) begin
                    slot_lookup_q[c][slot_lookup_sel[c]] <= 1'b1;
                    slot_rsp_q[c][slot_lookup_sel[c]]    <= 1'b1;
                    cmp_tag_q[c]                         <= tag[c];
                    cmp_we_q[c]                          <= cache_req[c].we;
                end

                // Tag answer for the slot it belongs to (answer order =
                // launch order = accept order). On a miss, capture the
                // victim for THIS slot at the pulse (the only cycle the tag
                // answers are current) and advance the set's round-robin
                // pointer — per-slot, so a second miss of the same cache
                // waiting in the other slot cannot clobber the first.
                if (cache_rsp_pulse[c]) begin
                    slot_rsp_q[c][slot_rsp_sel[c]] <= 1'b0;
                    if (cache_hit_pulse[c]) begin
                        skid_valid_q[c][slot_rsp_sel[c]] <= 1'b0;
                    end else begin
                        miss_seen_q[c][slot_rsp_sel[c]] <= 1'b1;
                        victim_way_q[c][slot_rsp_sel[c]] <= NBIT_WAY'(rsp_victim_way[c]);
                        victim_valid_q[c][slot_rsp_sel[c]] <= (c == 0) ? itag_rsp[
                            rsp_victim_way[c]].rdata[0] : dtag_rsp[rsp_victim_way[c]].rdata[0];
                        // The I cache is never dirty (its tag's dirty bit
                        // is always written 0), so only the D side records
                        // one: the I-side writeback logic folds away.
                        victim_dirty_q[c][slot_rsp_sel[c]] <= (c == 0) ? 1'b0 :
                            dtag_rsp[rsp_victim_way[c]].rdata[1];
                        victim_tag_q[c][slot_rsp_sel[c]] <= (c == 0) ?
                            itag_rsp[rsp_victim_way[c]].rdata[2+:TAG_FIELD_W] :
                            dtag_rsp[rsp_victim_way[c]].rdata[2+:TAG_FIELD_W];
                    end
                end

                // FSM unstall (S_UNSTALL cycle): the completed miss's slot is
                // freed (its outstanding unit with it) and entries latched
                // blocked behind the miss are unblocked. A load miss's
                // response is inserted in ACCEPT order: an unblocked head is
                // OLDER than the miss (pushed before it entered transit), a
                // blocked head is YOUNGER — the response goes behind the
                // former and in front of the latter. The queue holds at most
                // one entry while the missed slot is still occupied
                // (outstanding <= SKID_DEPTH), so a same-cycle pop can only
                // be the unblocked-head case. No CPU push can collide: the
                // lookups were gated during the refill, so no answer pulse
                // fires this cycle.
                if (fsm_unstall[c]) begin
                    skid_valid_q[c][fsm_unstall_slot] <= 1'b0;
                    slot_miss_q[c][fsm_unstall_slot]  <= 1'b0;
                    rq_blk_q[c][0]                    <= 1'b0;
                    rq_blk_q[c][1]                    <= 1'b0;
                    if (fsm_rsp_push[c]) begin
                        if (cache_pop[c]) begin
                            // Unblocked head consumed this cycle: take its place.
                            rq_q[c][0] <= fsm_rsp_data;
                        end else if ((SKID_DEPTH[c] == 1) || (rq_cnt_q[c] == 2'd0)) begin
                            rq_cnt_q[c] <= rq_cnt_q[c] + 2'd1;
                            rq_q[c][0]  <= fsm_rsp_data;
                        end else if (!rq_blk_q[c][0]) begin
                            // Older entry at the head: miss response goes behind it.
                            rq_cnt_q[c] <= rq_cnt_q[c] + 2'd1;
                            rq_q[c][1]  <= fsm_rsp_data;
                        end else begin
                            // Younger entry latched blocked behind the miss:
                            // the (older) miss response goes in front of it.
                            rq_cnt_q[c] <= rq_cnt_q[c] + 2'd1;
                            rq_q[c][1]  <= rq_q[c][0];
                            rq_q[c][0]  <= fsm_rsp_data;
                        end
                    end
                end else if (cache_push[c] && cache_pop[c]) begin
                    rq_q[c][0]     <= cache_push_data[c];
                    rq_blk_q[c][0] <= older_miss[c];
                end else if (cache_push[c]) begin
                    rq_cnt_q[c] <= rq_cnt_q[c] + 2'd1;
                    if ((SKID_DEPTH[c] == 1) || (rq_cnt_q[c] == 2'd0)) begin
                        rq_q[c][0]     <= cache_push_data[c];
                        rq_blk_q[c][0] <= older_miss[c];
                    end else begin
                        rq_q[c][1]     <= cache_push_data[c];
                        rq_blk_q[c][1] <= older_miss[c];
                    end
                end else if (cache_pop[c]) begin
                    rq_cnt_q[c] <= rq_cnt_q[c] - 2'd1;
                    if (SKID_DEPTH[c] > 1) begin
                        rq_q[c][0]     <= rq_q[c][1];
                        rq_blk_q[c][0] <= rq_blk_q[c][1];
                    end
                end

                // FSM pickup of the oldest missed slot.
                if (cache_fsm_latch[c]) begin
                    miss_seen_q[c][slot_miss_sel[c]] <= 1'b0;
                    slot_miss_q[c][slot_miss_sel[c]] <= 1'b1;
                end
            end
        end
    end

    // ===================================================================
    // Miss-handling FSM
    // ===================================================================
    // Hits never enter this FSM (they're served by the skid/queue logic
    // above); only a miss triggers arbitration for the shared SDRAM
    // controller. Flow: latch the oldest missed slot (S_IDLE), pick the
    // victim (captured at the miss pulse; prefer an invalid way, else
    // per-set round-robin), write back a dirty valid victim to the VICTIM's
    // SDRAM address, refill the missing line, commit line + tag into the
    // victim way, then unstall the requester (free the slot, push the
    // response in accept order) and return to S_IDLE.
    //
    // SDRAM access model (sdram_controller host interface, m6-burst): a
    // refill is ONE burst read of the whole line (BURST_LEN words, one
    // ACTIVATE), and each 64-bit beat is written into the victim way as
    // soon as its second word arrives, the tag riding the last beat — so
    // the line is committed the cycle its last word lands and no separate
    // commit pass follows. A writeback is ONE streamed line write. Every
    // request is held until its ack (a due refresh delays it).
    //
    // Clean miss, 50 MHz (tRCD = 1 clock), counted by the probe below:
    //   1      S_ARBITRATE: read request, taken by an idle controller
    //   2      ACTIVATE on the pins
    //   3      READ on the pins
    //   4-6    CAS latency 3 (the controller's input register is the 3rd)
    //   7-14   eight words, one per cycle; beats committed at 8/10/12/14
    //   15     S_UNSTALL: response pushed
    // At 100 MHz (sim_top) tRCD is 2 clocks: 16.

    typedef enum logic [3:0] {
        S_IDLE,
        S_ARBITRATE,
        S_WB_READ,  // copy the victim line into line_buf_q, one beat per cycle (pipelined)
        S_WB_ISSUE,  // streamed line write request, held until wr_ack
        S_WB_STREAM,  // feed words 1.. of the line on wr_next
        S_REFILL_ISSUE,  // burst read request, held until rd_ack
        S_REFILL_WAIT,  // capture the words, commit each beat as it completes
        S_UNSTALL,  // free the missed slot, deliver the response
        // Cache-bypass path (TGT_MEM): one CPU doubleword straight
        // to/from the device, no cache array touched. A load is one burst
        // read whose first two words are the doubleword. A store writes
        // its two words one at a time, after that same read unless it
        // strobes all eight bytes — the controller drives dqm itself, so
        // a partial word cannot be masked at the pins.
        S_BP_RD_ISSUE,
        S_BP_RD_WAIT,
        S_BP_WR_ISSUE,
        // fence.i flush (M7): per set, invalidate both I ways and read both
        // D tags (S_FI_TAG), then write back the first dirty D way through
        // the writeback states and mark it clean (S_FI_CLEAN), re-scanning
        // the set until no way is dirty (S_FI_CHK); ack after the last set.
        S_FI_TAG,
        S_FI_CHK,
        S_FI_CLEAN,
        S_FI_ACK
    } fsm_state_e;

    fsm_state_e state_q, state_d;

    assign dbg_state_o = state_q;

    // D-cache event counters (see dbg_cnt_o). Saturating: a stuck port is
    // identified by the first count that stops advancing, so wrapping
    // would destroy exactly the information wanted.
    // syn_preserve / syn_keep: these are probes, and a probe the optimiser
    // is allowed to fold into a constant reports the optimiser's opinion
    // instead of the hardware's behaviour. On the board every one of them
    // read zero while the state they count said the events had happened,
    // so they are pinned here and the next build says whether the freeze
    // is in the design or in what synthesis did to it.
    (* syn_preserve = 1 *)
    (* syn_keep = 1 *)
    logic [3:0] cnt_lookup_q, cnt_pulse_q, cnt_miss_q, cnt_unstall_q, cnt_acc_q;
    (* syn_preserve = 1 *) (* syn_keep = 1 *)
    logic [24:0] tick_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            cnt_lookup_q  <= '0;
            cnt_pulse_q   <= '0;
            cnt_miss_q    <= '0;
            cnt_unstall_q <= '0;
            cnt_acc_q     <= '0;
            tick_q        <= '0;
        end else begin
            tick_q <= tick_q + 1'b1;
            if (cache_accept[1] && !(&cnt_acc_q)) cnt_acc_q <= cnt_acc_q + 1'b1;
            if (cache_lookup_go[1] && !(&cnt_lookup_q)) cnt_lookup_q <= cnt_lookup_q + 1'b1;
            if (cache_rsp_pulse[1] && !(&cnt_pulse_q)) cnt_pulse_q <= cnt_pulse_q + 1'b1;
            if ((state_q == S_IDLE) && miss_pending && miss_sel && !(&cnt_miss_q))
                cnt_miss_q <= cnt_miss_q + 1'b1;
            if (fsm_unstall[1] && !(&cnt_unstall_q)) cnt_unstall_q <= cnt_unstall_q + 1'b1;
        end
    end

    assign dbg_cnt_o = {cnt_unstall_q, cnt_miss_q, cnt_pulse_q, cnt_lookup_q};
    assign dbg_acc_o = cnt_acc_q;
    // Bits chosen so the field changes between two report lines at both
    // rates in use: every 4096 clocks is ~82 us on the board and ~41 us in
    // simulation, both far shorter than a report period.
    // Bits 15:12 tick every 4096 clocks. NOT a multiple of the reporter's
    // period, which is what the first version got wrong: it exported
    // tick_q[15:12] while the reporter sampled every 2**24 clocks, an exact
    // multiple of that field's own period, so every sample landed on the
    // same phase and the field read constant. A constant probe reads
    // exactly like a dead clock, and it cost three board round-trips.
    // 5 bits down from the sample period, offset by a prime-ish shift:
    assign dbg_tick_o = {tick_q[20], tick_q[17], tick_q[14], tick_q[11]};
    assign dbg_hb_o = tick_q[24];
    assign dbg_go_o = {
        cache_lookup_go[1], fsm_lookup_gate[1], dtag_req[0].valid, dtag_rsp[0].wready
    };
    assign dbg_rsp_o = {
        dtag_rsp[0].rvalid, dmem_rsp_d[0].rvalid, slot_rsp_q[1][0], slot_rsp_q[1][1]
    };
    assign dbg_dport_o = {
        skid_valid_q[1][0], slot_lookup_q[1][0], miss_seen_q[1][0], rq_cnt_q[1] != 2'd0
    };

    logic req_sel_q, req_sel_d;  // 0 = icache, 1 = dcache
    // Word counter: SDRAM words of a burst (0 .. BURST_LEN-1), or the
    // writeback copy step (0 .. BEATS) in S_WB_READ.
    logic [3:0] burst_cnt_q, burst_cnt_d;
    // No line buffer (M4b, 2026-11-27): a refill commits each beat into the
    // victim way as it completes and a writeback streams straight off the
    // victim way's registered output, so only two small registers remain.
    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] rsp_q, rsp_d;  // response / bypass doubleword
    logic [31:0] lo_q, lo_d;  // low word of the refill beat in progress
    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] miss_addr_q, miss_addr_d;
    logic [$clog2(N_SLOT)-1:0] miss_slot_q, miss_slot_d;  // skid slot the FSM owns
    logic byp_q, byp_d;  // the latched slot is a cache-bypass access

    logic miss_pending;
    logic miss_sel;  // 0 = icache, 1 = dcache (the entry the FSM will pick up)
    logic [N_CACHE-1:0] cache_fsm_latch;

    // A miss is pending from its lookup's response until the FSM latches it
    // (miss_seen_q is a one-shot per request, not a per-cycle level — the
    // FSM will not re-refill the same line forever after returning to
    // S_IDLE). Fixed priority: dcache wins ties (avoids stalling stores);
    // the loser keeps its miss_seen_q slot until the FSM comes back. The
    // FSM pickup does NOT free the skid slot: the slot stays occupied
    // (slot_miss_q) so no younger response can pass the older miss; it is
    // unstalled when the miss completes (S_UNSTALL).
    assign miss_pending = slot_miss_wait[0] || slot_miss_wait[1];
    assign miss_sel     = slot_miss_wait[1];

    wire fsm_latch_miss = (state_q == S_IDLE) && miss_pending;
    assign cache_fsm_latch[0] = fsm_latch_miss && !miss_sel;
    assign cache_fsm_latch[1] = fsm_latch_miss && miss_sel;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            state_q     <= S_IDLE;
            req_sel_q   <= 1'b0;
            burst_cnt_q <= '0;
            rsp_q       <= '0;
            lo_q        <= '0;
            miss_addr_q <= '0;
            miss_slot_q <= '0;
            byp_q       <= 1'b0;
            fi_q        <= 1'b0;
            fi_set_q    <= '0;
            fi_way_q    <= '0;
            fi_tag_q    <= '0;
        end else begin
            state_q     <= state_d;
            req_sel_q   <= req_sel_d;
            burst_cnt_q <= burst_cnt_d;
            rsp_q       <= rsp_d;
            lo_q        <= lo_d;
            miss_addr_q <= miss_addr_d;
            miss_slot_q <= miss_slot_d;
            byp_q       <= byp_d;
            fi_q        <= fi_d;
            fi_set_q    <= fi_set_d;
            fi_way_q    <= fi_way_d;
            fi_tag_q    <= fi_tag_d;
        end
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) fi_wait_low_q <= 1'b0;
        else if (state_q == S_FI_ACK) fi_wait_low_q <= 1'b1;
        else if (!fencei_req_i) fi_wait_low_q <= 1'b0;
    end

    // A fence.i is being requested (and not yet answered): hold both
    // ports' accepts. The flush starts once nothing accepted is in flight.
    wire fi_hold = (fencei_req_i && !fi_wait_low_q) || fi_q;
    wire fi_drained = !(|skid_valid_q[0]) && !(|skid_valid_q[1]);
    wire fi_start = fencei_req_i && !fi_wait_low_q && fi_drained;

    assign fencei_ack_o = (state_q == S_FI_ACK);

    // -------------------------------------------------------------
    // Skid read selectors (M4, 2026-11-22). Every read of a skid slot
    // through a run-time selector goes through one of these, instead of
    // skid_q[c][sel] written out at each use: sv2v + yosys lower each such
    // dynamic index into its own shifter over the whole 4-slot array.
    // N_SLOT is 2, so each is a 2:1 mux on constant indices.
    // -------------------------------------------------------------
    always_comb begin
        for (int c = 0; c < N_CACHE; c++) begin
            rsp_req[c] = (slot_rsp_sel[c] != '0) ? skid_q[c][1] : skid_q[c][0];
            nc_req[c]  = (nc_sel[c] != '0) ? skid_q[c][1] : skid_q[c][0];
        end
        if (miss_sel) latch_req = (slot_miss_sel[1] != '0) ? skid_q[1][1] : skid_q[1][0];
        else latch_req = (slot_miss_sel[0] != '0) ? skid_q[0][1] : skid_q[0][0];
    end

    // The request the FSM owns, read ONCE (2-bit dynamic index into the skid
    // array) and shared by every FSM consumer below; it used to be indexed
    // in seven places, each lowered to its own 556-bit shifter.
    yarv32_cache_pkg::mem_req_t miss_req;
    always_comb begin
        unique case ({
            req_sel_q, miss_slot_q
        })
            2'b00:   miss_req = skid_q[0][0];
            2'b01:   miss_req = skid_q[0][1];
            2'b10:   miss_req = skid_q[1][0];
            default: miss_req = skid_q[1][1];
        endcase
    end

    // -------------------------------------------------------------
    // Miss-latency probe (M4, 2026-11-22). The acceptance measurement of the
    // Linux plan's M6 (SDRAM bursts: <= 20 core cycles per clean miss).
    // A CLEAN miss is a cached line refill with no dirty-victim writeback
    // and no bypass. Its latency is counted from the cycle the FSM latches
    // the miss (S_IDLE -> S_ARBITRATE; on an idle FSM that is the cycle
    // after the tag answer revealed it) to S_UNSTALL, the cycle the
    // response is pushed to the CPU-side queue -- i.e. the refill, the
    // line commit and the tag write, everything the requester waits for
    // beyond its hit latency. Simulation reads these flops hierarchically
    // (sim_top / bist_tb print them); nothing in the design consumes them.
    // -------------------------------------------------------------
    logic [15:0] miss_cyc_q;  // cycles since the current miss was latched
    logic wb_seen_q;  // the current miss wrote a dirty victim back
    logic lat_early_q;  // latched before the SDRAM finished its power-up wait
    logic [15:0] clean_lat_last_q, clean_lat_min_q, clean_lat_max_q;
    logic [15:0] clean_cnt_q;
    logic [31:0] clean_lat_sum_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            miss_cyc_q       <= '0;
            wb_seen_q        <= 1'b0;
            lat_early_q      <= 1'b0;
            clean_lat_last_q <= '0;
            clean_lat_min_q  <= '1;
            clean_lat_max_q  <= '0;
            clean_cnt_q      <= '0;
            clean_lat_sum_q  <= '0;
        end else begin
            if (state_q == S_IDLE) begin
                miss_cyc_q  <= 16'd1;
                wb_seen_q   <= 1'b0;
                // A miss latched during the 200 us power-up hold measures
                // the hold, not the refill: excluded.
                lat_early_q <= !init_done;
            end else begin
                miss_cyc_q <= miss_cyc_q + 1'b1;
                if (state_q == S_WB_READ) wb_seen_q <= 1'b1;
            end
            if ((state_q == S_UNSTALL) && !byp_q && !wb_seen_q && !lat_early_q) begin
                clean_lat_last_q <= miss_cyc_q;
                clean_cnt_q      <= clean_cnt_q + 1'b1;
                clean_lat_sum_q  <= clean_lat_sum_q + 32'(miss_cyc_q);
                if (miss_cyc_q < clean_lat_min_q) clean_lat_min_q <= miss_cyc_q;
                if (miss_cyc_q > clean_lat_max_q) clean_lat_max_q <= miss_cyc_q;
            end
        end
    end

    // Line-aligned byte addresses: writeback targets the VICTIM's line
    // {victim_tag, set, offset 0} — NOT the missing address (that would
    // overwrite the missing line's own SDRAM location with victim data);
    // refill reads the missing line itself. The controller's host address
    // is the 32-bit word index {bank, row, col} = byte_addr[22:2], so the
    // per-word index is the line base word + burst_cnt_q.
    // The line written back: a miss's victim, or the dirty way a fence.i
    // flush found (its set is in miss_addr_q either way).
    wire [TAG_FIELD_W-1:0] wb_tag = fi_q ? fi_tag_q : victim_tag_q[req_sel_q][miss_slot_q];

    logic [22:0] wb_byte_addr, refill_byte_addr;
    assign wb_byte_addr     = {wb_tag, miss_addr_q[NBIT_OFFSET+:NBIT_SET_IDX], {NBIT_OFFSET{1'b0}}};
    assign refill_byte_addr = {miss_addr_q[MEM_SIZE-1:NBIT_OFFSET], {NBIT_OFFSET{1'b0}}};

    // Cache-bypass datapath. A CPU access is 64 bit and the SDRAM bus is
    // 32: a load is one burst read from the doubleword's low word (the
    // burst wraps inside its aligned 8-word block, so its first two words
    // are the doubleword), a store writes BP_WORDS single words;
    // burst_cnt_q[0] selects which word is written.
    localparam int BP_WORDS = yarv32_cache_pkg::MEM_WIDTH / 32;

    wire bp_hi = burst_cnt_q[0];  // 1 = upper 32 bits of the doubleword
    wire bp_last = ({28'd0, burst_cnt_q} == BP_WORDS - 1);
    wire [MEM_SIZE-1:2] bp_word_addr = {miss_addr_q[MEM_SIZE-1:3], bp_hi};
    wire [MEM_SIZE-1:2] bp_rd_addr = {miss_addr_q[MEM_SIZE-1:3], 1'b0};

    wire [3:0] bp_strb = bp_hi ? miss_req.wstrb[7:4] : miss_req.wstrb[3:0];
    wire [31:0] bp_wdata = bp_hi ? miss_req.wdata[63:32] : miss_req.wdata[31:0];

    // A store that strobes the whole doubleword needs no read first; a
    // partial one is merged into the words just read back (the
    // controller's dqm is not under this FSM's control, so masking at the
    // pins is not an option).
    wire bp_skip_read = miss_is_store && (&miss_req.wstrb);

    logic [31:0] bp_wr_data;

    always_comb begin
        bp_wr_data = bp_hi ? rsp_q[63:32] : rsp_q[31:0];
        for (int b = 0; b < 4; b++) begin
            if (bp_strb[b]) bp_wr_data[b*8+:8] = bp_wdata[b*8+:8];
        end
    end

    wire burst_last = ({28'd0, burst_cnt_q} == BURST_LEN - 1);

    always_comb begin
        // defaults: hold state / datapath
        state_d = state_q;
        req_sel_d = req_sel_q;
        burst_cnt_d = burst_cnt_q;
        rsp_d = rsp_q;
        lo_d = lo_q;
        miss_addr_d = miss_addr_q;
        miss_slot_d = miss_slot_q;
        byp_d = byp_q;
        fi_d = fi_q;
        fi_set_d = fi_set_q;
        fi_way_d = fi_way_q;
        fi_tag_d = fi_tag_q;

        sdram_rd_en = 1'b0;
        sdram_wr_en = 1'b0;
        sdram_wr_burst = !byp_q;
        // Writeback: the victim line base, streamed word by word straight
        // off the victim way's registered output, which holds the beat of
        // word burst_cnt_q (the word the controller takes next). A bypass
        // addresses the CPU doubleword's words directly.
        sdram_wr_addr = byp_q ? bp_word_addr : wb_byte_addr[MEM_SIZE-1:2];
        sdram_wr_data = byp_q ?
            bp_wr_data : (burst_cnt_q[0] ? fsm_victim_beat[63:32] : fsm_victim_beat[31:0]);
        sdram_rd_addr = byp_q ? bp_rd_addr : refill_byte_addr[MEM_SIZE-1:2];

        unique case (state_q)

            S_IDLE: begin
                burst_cnt_d = '0;
                if (miss_pending) begin
                    // Fixed priority: dcache wins ties (avoids stalling
                    // stores). cache_fsm_latch (above) marks the picked-up
                    // slot slot_miss_q; the FSM owns it until the miss
                    // completes.
                    req_sel_d   = miss_sel;
                    miss_addr_d = latch_req.addr;
                    miss_slot_d = $clog2(N_SLOT)'(slot_miss_sel[miss_sel]);
                    // A bypass slot arrives through the same hand-off as a
                    // miss (miss_seen_q); this is what tells the two apart
                    // for the rest of the transfer.
                    byp_d       = (slot_tgt_q[miss_sel][slot_miss_sel[miss_sel]] == TGT_MEM);
                    state_d     = S_ARBITRATE;
                end else if (fi_start) begin
                    // fence.i: the ports are held and drained. The flush
                    // works on the D cache's macros (req_sel 1) and writes
                    // the I tags directly.
                    fi_d      = 1'b1;
                    fi_set_d  = '0;
                    req_sel_d = 1'b1;
                    byp_d     = 1'b0;
                    state_d   = S_FI_TAG;
                end
            end

            S_ARBITRATE: begin
                // Victim way/tag/dirty were captured at the miss's lookup
                // pulse (victim_*_q, per slot — see the skid block). Only a
                // valid AND dirty victim needs its line written back before
                // the refill overwrites the way. A clean miss requests its
                // burst right here: one cycle saved on every refill.
                burst_cnt_d = '0;
                // A bypass has no victim (no tag lookup ever ran for it,
                // so victim_*_q hold whatever the previous miss left) and
                // no line: straight to the doubleword transfer.
                if (byp_q) state_d = S_BP_RD_ISSUE;
                else if (victim_valid_q[req_sel_q][miss_slot_q] &&
                         victim_dirty_q[req_sel_q][miss_slot_q])
                    state_d = S_WB_READ;
                else begin
                    sdram_rd_en = 1'b1;
                    state_d     = sdram_rd_ack ? S_REFILL_WAIT : S_REFILL_ISSUE;
                end
            end

            S_WB_READ: begin
                // Read beat 0 of the victim line out of the victim way
                // (fsm_way_req); it sits on the macro's output from the
                // next cycle on.
                burst_cnt_d = '0;
                state_d     = S_WB_ISSUE;
            end

            S_WB_ISSUE: begin
                // Streamed line write: word 0 is taken with the request.
                sdram_wr_en = 1'b1;
                if (sdram_wr_ack) begin
                    burst_cnt_d = 4'd1;
                    state_d     = S_WB_STREAM;
                end
            end

            S_WB_STREAM: begin
                // Words 1 .. BURST_LEN-1, one per wr_next. The cycle that
                // hands over the HIGH word of beat k also reads beat k+1
                // (fsm_way_req): the controller latches the old output at
                // the same edge the macro loads the new one. Any pacing of
                // wr_next works, since the output holds until the next
                // read. Once the last word is taken the victim way may be
                // overwritten; the refill request then waits for the
                // controller to finish the write and its precharge (rd_ack).
                if (sdram_wr_next) begin
                    burst_cnt_d = burst_cnt_q + 1'b1;
                    if (burst_last) begin
                        burst_cnt_d = '0;
                        // A flush only writes back; a miss refills next.
                        state_d     = fi_q ? S_FI_CLEAN : S_REFILL_ISSUE;
                    end
                end
            end

            S_REFILL_ISSUE: begin
                sdram_rd_en = 1'b1;
                if (sdram_rd_ack) state_d = S_REFILL_WAIT;
            end

            S_REFILL_WAIT: begin
                // One word per rd_ready pulse. Word 2k+1 completes beat k,
                // which is written into the victim way in this same cycle
                // (fsm_way_req, from the live word); the tag write rides the
                // last beat, so a lookup can never see the new tag over a
                // half-written line. Lookups are gated throughout.
                if (sdram_rd_ready) begin
                    // Even word: hold it for its beat. The requested
                    // doubleword's words are kept for the response.
                    if (!burst_cnt_q[0]) lo_d = sdram_rd_data;
                    if (burst_cnt_q[BEAT_W:1] == miss_addr_q[NBIT_OFFSET-1:3]) begin
                        if (burst_cnt_q[0]) rsp_d[63:32] = sdram_rd_data;
                        else rsp_d[31:0] = sdram_rd_data;
                    end
                    burst_cnt_d = burst_cnt_q + 1'b1;
                    if (burst_last) state_d = S_UNSTALL;
                end
            end

            S_UNSTALL: begin
                // Unstall is a wire (fsm_unstall), acted on by the skid block
                // at this cycle's edge: the missed slot is freed, a load
                // miss's response is pushed onto the queue in accept order,
                // and entries latched blocked behind the miss are unblocked.
                state_d = S_IDLE;
            end

            // ---- cache-bypass transfer (byp_q) ----

            S_BP_RD_ISSUE: begin
                burst_cnt_d = '0;
                // A store strobing every byte overwrites everything the
                // read would have returned, so skip it.
                if (bp_skip_read) begin
                    state_d = S_BP_WR_ISSUE;
                end else begin
                    sdram_rd_en = 1'b1;
                    if (sdram_rd_ack) state_d = S_BP_RD_WAIT;
                end
            end

            S_BP_RD_WAIT: begin
                // The doubleword is the first two words of the burst; the
                // rest are drained and dropped.
                if (sdram_rd_ready) begin
                    if (burst_cnt_q == 4'd0) rsp_d[31:0] = sdram_rd_data;
                    if (burst_cnt_q == 4'd1) rsp_d[63:32] = sdram_rd_data;
                    burst_cnt_d = burst_cnt_q + 1'b1;
                    if (burst_last) begin
                        burst_cnt_d = '0;
                        state_d     = miss_is_store ? S_BP_WR_ISSUE : S_UNSTALL;
                    end
                end
            end

            S_BP_WR_ISSUE: begin
                // One single-word write per word of the doubleword, each
                // held until the controller takes it. Posted: the
                // controller serialises every later access behind it.
                sdram_wr_en = 1'b1;
                if (sdram_wr_ack) begin
                    if (bp_last) state_d = S_UNSTALL;
                    else burst_cnt_d = burst_cnt_q + 1'b1;
                end
            end

            // ---- fence.i flush (fi_q) ----

            S_FI_TAG: begin
                // fi_*tag_req: I tags of fi_set_q invalidated (both ways),
                // D tags of fi_set_q read (both ways). The set also goes
                // into miss_addr_q, where the writeback path takes it from.
                miss_addr_d = {
                    {(yarv32_cache_pkg::MEM_WIDTH - NBIT_SET_IDX - NBIT_OFFSET) {1'b0}},
                    fi_set_q,
                    {NBIT_OFFSET{1'b0}}
                };
                state_d = S_FI_CHK;
            end

            S_FI_CHK: begin
                // D tags of the set are on the macro outputs. Write back the
                // lowest valid+dirty way; S_FI_CLEAN then clears its dirty
                // bit and the set is scanned again. A set with no dirty way
                // is done.
                burst_cnt_d = '0;
                state_d     = S_FI_ACK;
                if (fi_set_q != NBIT_SET_IDX'(N_SETS - 1)) begin
                    fi_set_d = fi_set_q + 1'b1;
                    state_d  = S_FI_TAG;
                end
                for (int w = N_WAY - 1; w >= 0; w--) begin
                    if (dtag_rsp[w].rdata[0] && dtag_rsp[w].rdata[1]) begin
                        fi_way_d = NBIT_WAY'(w);
                        fi_tag_d = dtag_rsp[w].rdata[2+:TAG_FIELD_W];
                        fi_set_d = fi_set_q;
                        state_d  = S_WB_READ;
                    end
                end
            end

            S_FI_CLEAN: begin
                // fi_dtag_req: rewrite the written-back way's tag clean
                // (still valid), then re-scan the same set.
                state_d = S_FI_TAG;
            end

            S_FI_ACK: begin
                // fencei_ack_o pulses (state_q == S_FI_ACK).
                fi_d    = 1'b0;
                state_d = S_IDLE;
            end

            default: state_d = S_IDLE;
        endcase
    end

    // fence.i tag-macro requests (driven into itag_req / dtag_req by the
    // tag request block): S_FI_TAG invalidates the scanned set in both I
    // ways and reads it in both D ways; S_FI_CLEAN rewrites the
    // written-back D way's tag with dirty = 0.
    assign fi_tag_scan  = (state_q == S_FI_TAG);
    assign fi_tag_clean = (state_q == S_FI_CLEAN);

    always_comb begin
        fi_itag_req = '0;
        fi_itag_req.valid = 1'b1;
        fi_itag_req.we = 1'b1;
        fi_itag_req.addr = {
            {(yarv32_cache_pkg::MEM_WIDTH - NBIT_SET_IDX - TAG_BYTES_W) {1'b0}},
            fi_set_q,
            {TAG_BYTES_W{1'b0}}
        };
        fi_itag_req.wdata = '0;  // valid = 0
        fi_itag_req.wstrb = {(TAG_DATA_W / 8) {1'b1}};
        fi_itag_req.rready = 1'b1;

        fi_dtag_req = fi_itag_req;
        fi_dtag_req.we = fi_tag_clean;
        fi_dtag_req.wdata = {{(TAG_DATA_W - TAG_FIELD_W - 2) {1'b0}}, fi_tag_q, 2'b01};
        if (!fi_tag_clean) fi_dtag_req.wstrb = '0;
    end

    // -------------------------------------------------------------
    // FSM datapath muxes (declared with the signals above, driven here
    // after the FSM state they depend on).
    // -------------------------------------------------------------

    // Victim beat as read back in S_WB_READ (the victim way's rdata_q, one
    // cycle after the read).
    // Only D lines are ever dirty, so only the D ways are ever written back.
    assign fsm_victim_beat = dmem_rsp_d[fsm_victim_way].rdata;

    // The missed request is a store: write-allocate — the line is committed
    // with the store already merged, so it lands dirty.
    assign miss_is_store   = miss_req.we;

    // Refill commit. In S_REFILL_WAIT, the rd_ready pulse carrying word
    // 2k+1 completes beat k = burst_cnt_q[2:1]: its low word is already in
    // lo_q, its high word is on sdram_rd_data. The store of a store
    // miss is merged into its doubleword's beat: a fixed-position byte
    // merge, no line-wide variable indexing.
    wire [BEAT_W-1:0] refill_beat = burst_cnt_q[BEAT_W:1];
    wire refill_beat_go = (state_q == S_REFILL_WAIT) && sdram_rd_ready && burst_cnt_q[0];
    wire refill_last = refill_beat_go && burst_last;

    always_comb begin
        commit_beat = {sdram_rd_data, lo_q};
        if (miss_is_store && (refill_beat == miss_addr_q[NBIT_OFFSET-1:3])) begin
            for (int b = 0; b < MACRO_W / 8; b++) begin
                if (miss_req.wstrb[b]) begin
                    commit_beat[b*8+:8] = miss_req.wdata[b*8+:8];
                end
            end
        end
    end

    // FSM requests to the data/tag macros of the cache being serviced
    // (req_sel_q). Lookups on that cache are gated while these are active,
    // so the muxes in the macro request blocks never arbitrate two drivers.
    always_comb begin
        fsm_victim_way  = fi_q ? fi_way_q : victim_way_q[req_sel_q][miss_slot_q];
        fsm_way_req     = '0;
        fsm_tag_req     = '0;
        fsm_imem_access = 1'b0;
        fsm_dmem_access = 1'b0;
        fsm_itag_write  = 1'b0;
        fsm_dtag_write  = 1'b0;

        case (state_q)
            S_WB_READ, S_WB_ISSUE, S_WB_STREAM: begin
                // Writeback reads of the victim (always a D way): beat 0 in
                // S_WB_READ, beat k+1 in the S_WB_STREAM cycle that hands
                // the high word of beat k to the controller (wr_next with
                // burst_cnt_q odd, burst_cnt_q = 2k+1). Line base =
                // {victim_tag, set, offset 0}; the macro decodes only the
                // set and beat bits. S_WB_ISSUE drives the port idle, so
                // beat 0 stays on the output until it has been handed over.
                fsm_dmem_access = 1'b1;
                fsm_way_req.valid = (state_q == S_WB_READ) ||
                    ((state_q == S_WB_STREAM) && sdram_wr_next && burst_cnt_q[0] && !burst_last);
                fsm_way_req.we = 1'b0;
                fsm_way_req.addr = {
                    {(yarv32_cache_pkg::MEM_WIDTH - MEM_SIZE) {1'b0}},
                    wb_tag,
                    miss_addr_q[NBIT_OFFSET+:NBIT_SET_IDX],
                    (state_q == S_WB_READ) ? BEAT_W'(0) : BEAT_W'(burst_cnt_q[BEAT_W:1] + 1'b1),
                    3'b000
                };
                fsm_way_req.rready = 1'b1;
            end

            S_REFILL_WAIT: begin
                // Beat commit into the victim way + the tag write with the
                // last beat (both posted at this cycle's edge). Tag =
                // {tag, dirty, valid} of the missing address; dirty is set
                // only if the missing request was a store (write-allocate,
                // merged into commit_beat above).
                if (req_sel_q == 1'b0) begin
                    fsm_imem_access = 1'b1;
                    fsm_itag_write  = refill_last;
                end else begin
                    fsm_dmem_access = 1'b1;
                    fsm_dtag_write  = refill_last;
                end

                fsm_way_req.valid = refill_beat_go;
                fsm_way_req.we = 1'b1;
                // Beat commit at the missing address itself (zero-extended);
                // the macro decodes word_addr = {set, beat}, exactly as the
                // CPU lookups do.
                fsm_way_req.addr = {
                    {(yarv32_cache_pkg::MEM_WIDTH - MEM_SIZE) {1'b0}},
                    miss_addr_q[MEM_SIZE-1:NBIT_OFFSET],
                    refill_beat,
                    3'b000
                };
                fsm_way_req.wdata = commit_beat;
                fsm_way_req.wstrb = {(MACRO_W / 8) {1'b1}};
                fsm_way_req.rready = 1'b1;

                fsm_tag_req.valid = refill_last;
                fsm_tag_req.we = 1'b1;
                // Same set-index shift as the lookup requests.
                fsm_tag_req.addr = {
                    {(yarv32_cache_pkg::MEM_WIDTH - NBIT_SET_IDX - TAG_BYTES_W) {1'b0}},
                    miss_addr_q[NBIT_OFFSET+:NBIT_SET_IDX],
                    {TAG_BYTES_W{1'b0}}
                };
                fsm_tag_req.wdata = {
                    {(TAG_DATA_W - TAG_FIELD_W - 2) {1'b0}},
                    miss_addr_q[MEM_SIZE-1-:TAG_FIELD_W],
                    miss_is_store && req_sel_q,  // the I cache is never dirty
                    1'b1
                };
                fsm_tag_req.wstrb = {(TAG_DATA_W / 8) {1'b1}};
                fsm_tag_req.rready = 1'b1;
            end

            default: ;
        endcase
    end

    // Lookups on the cache the FSM is servicing are held from the
    // writeback copy to the last refill beat. S_WB_READ reads the victim
    // line through the macro's port; S_REFILL_WAIT overwrites the victim
    // way beat by beat while its tag still names the OLD line, so a lookup
    // in that window could hit the old tag and read new data. Gating the
    // whole refill (not only its write cycles) keeps that window shut with
    // no per-cycle reasoning; the cost is that a second I-port request
    // waits for the miss in front of it, which it does for its response
    // anyway. A lookup launched before the gate rises (S_IDLE /
    // S_ARBITRATE) answers before the first beat is written.
    logic fsm_macro_state;
    assign fsm_macro_state = (state_q == S_WB_READ) || (state_q == S_WB_ISSUE) ||
        (state_q == S_WB_STREAM) || (state_q == S_REFILL_ISSUE) || (state_q == S_REFILL_WAIT);
    // A fence.i flush owns both caches' macros for its whole duration.
    assign fsm_lookup_gate[0] = (fsm_macro_state && (req_sel_q == 1'b0)) || fi_q;
    assign fsm_lookup_gate[1] = (fsm_macro_state && (req_sel_q == 1'b1)) || fi_q;

    // Unstall (S_UNSTALL cycle): free the missed slot; a load miss pushes
    // its response — the refilled line's doubleword — in accept order.
    assign fsm_unstall_slot = miss_slot_q;
    assign fsm_unstall[0] = (state_q == S_UNSTALL) && (req_sel_q == 1'b0);
    assign fsm_unstall[1] = (state_q == S_UNSTALL) && (req_sel_q == 1'b1);
    assign fsm_rsp_push[0] = fsm_unstall[0] && !miss_req.we;
    assign fsm_rsp_push[1] = fsm_unstall[1] && !miss_req.we;
    // A refill answers out of the line it just assembled; a bypass answers
    // with the two words it fetched, which sit at the bottom of the same
    // buffer.
    assign fsm_rsp_data = rsp_q;

    // -------------------------------------------------------------
    // D-cache store hit (posted). The data-macro write into the hit way and
    // the dirty-bit tag write fire the same cycle the tag answer pulses.
    //
    // The write is a READ-MODIFY-WRITE of the whole line, not a
    // byte-strobed one: the data macros are BYTE_WRITE(0) (byte enables
    // cost 4x the BSRAM blocks on this device, see the macro
    // instantiation above). The line being modified is already on the
    // macro's registered output this very cycle — dcache_line, the hit
    // way's rdata — so merging the stored bytes into it costs no extra
    // cycle and no extra port. Only the bytes the store strobes change;
    // every other byte of the doubleword is written back with the value just
    // read (the macro is one doubleword wide since 2026-11-22). The I-port is read-only by spec: a we=1
    // request there is a posted no-op.
    // -------------------------------------------------------------
    assign dcache_store_hit = cache_rsp_pulse[1] && cache_hit_pulse[1] && cmp_we_q[1];

    always_comb begin
        dhit_way = '0;
        for (int i = N_WAY - 1; i >= 0; i--) begin
            if (dcache_way_hit[i]) dhit_way = NBIT_WAY'(i);
        end
    end

    always_comb begin
        dstore_way_req       = '0;
        dstore_way_req.valid = dcache_store_hit;
        dstore_way_req.we    = 1'b1;
        dstore_way_req.addr  = rsp_req[1].addr;
        // Start from the line as it stands in the hit way, then overwrite
        // only the strobed bytes of the addressed doubleword.
        dstore_way_req.wdata = dcache_line;
        for (int b = 0; b < MACRO_W / 8; b++) begin
            if (rsp_req[1].wstrb[b]) begin
                dstore_way_req.wdata[b*8+:8] = rsp_req[1].wdata[b*8+:8];
            end
        end
        dstore_way_req.wstrb = {(MACRO_W / 8) {1'b1}};
        dstore_way_req.rready = 1'b1;

        dstore_tag_req = '0;
        dstore_tag_req.valid = dcache_store_hit;
        dstore_tag_req.we = 1'b1;
        dstore_tag_req.addr = {
            {(yarv32_cache_pkg::MEM_WIDTH - NBIT_SET_IDX - TAG_BYTES_W) {1'b0}},
            rsp_req[1].addr[NBIT_OFFSET+:NBIT_SET_IDX],
            {TAG_BYTES_W{1'b0}}
        };
        // Same tag value as the hit (cmp_tag_q, by definition of the hit),
        // with dirty+valid set.
        dstore_tag_req.wdata = {{(TAG_DATA_W - TAG_FIELD_W - 2) {1'b0}}, cmp_tag_q[1], 2'b11};
        dstore_tag_req.wstrb = {(TAG_DATA_W / 8) {1'b1}};
        dstore_tag_req.rready = 1'b1;
    end

    // -------------------------------------------------------------
    // CPU-facing response: the response-queue head.
    //   rvalid is a LEVEL, held until the CPU's rready pops the head
    //   (protocol compliance — the old design exposed the lookup's
    //   one-cycle pulse and lost reads whenever rready was low).
    //   A queue entry pushed while an older miss was unresolved is blocked
    //   (rq_blk_q) until that miss completes — responses are delivered in
    //   accept order, which the fetch unit's instruction buffer relies on.
    //   wready accepts while outstanding (occupied slots + unconsumed
    //   entries) is below the per-port limit: 2 for the I-port's 2
    //   outstanding reads, 1 for the single-outstanding D-port. Hits are
    //   served while the miss FSM is mid-transit; only unresolved-miss
    //   slots gate delivery.
    // -------------------------------------------------------------
    //   An UNCACHED request (bootrom, control register, cache bypass) is
    //   accepted only into an idle port, and blocks the port until it
    //   retires. That is what lets the uncached paths ignore the ordering
    //   machinery entirely: while one is in flight there is nothing else
    //   in flight to order it against. It costs the I-port its second
    //   outstanding read for the duration of a bootrom fetch — boot code
    //   runs one fetch at a time, which is the right trade for not
    //   duplicating the queue's blocking logic per target.
    assign icache_rsp_o.wready = tag_init_done && !fi_hold &&
        (slot_outstanding[0] < 3'(SKID_DEPTH[0])) && !nc_busy[0] &&
        !((acc_tgt[0] != TGT_CACHE) && (slot_outstanding[0] != 3'd0));
    assign icache_rsp_o.rvalid = (rq_cnt_q[0] != 2'd0) && !rq_blk_q[0][0];
    assign icache_rsp_o.rdata = rq_q[0][0];
    assign icache_rsp_o.bvalid = 1'b0;  // posted stores, no B channel

    assign dcache_rsp_o.wready = tag_init_done && !fi_hold &&
        (slot_outstanding[1] < 3'(SKID_DEPTH[1])) && !nc_busy[1] &&
        !((acc_tgt[1] != TGT_CACHE) && (slot_outstanding[1] != 3'd0));
    assign dcache_rsp_o.rvalid = (rq_cnt_q[1] != 2'd0) && !rq_blk_q[1][0];
    assign dcache_rsp_o.rdata = rq_q[1][0];
    assign dcache_rsp_o.bvalid = 1'b0;  // posted stores, no B channel

`ifdef VERILATOR
    logic                                   icache_req_valid;
    logic                                   icache_req_we;
    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] icache_req_addr;
    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] icache_req_wdata;
    logic [                 yarv32_cache_pkg::STRB_WIDTH-1:0] icache_req_wstrb;
    logic                                   icache_req_rready;
    assign icache_req_valid  = icache_req_i.valid;
    assign icache_req_we     = icache_req_i.we;
    assign icache_req_addr   = icache_req_i.addr;
    assign icache_req_wdata  = icache_req_i.wdata;
    assign icache_req_wstrb  = icache_req_i.wstrb;
    assign icache_req_rready = icache_req_i.rready;

    logic                                   itag0_req_valid;
    logic                                   itag0_req_we;
    logic [yarv32_cache_pkg::MEM_WIDTH-1:0] itag0_req_addr;
    logic [                 TAG_DATA_W-1:0] itag0_req_wdata;
    logic [               TAG_DATA_W/8-1:0] itag0_req_wstrb;
    logic                                   itag0_req_rready;
    assign itag0_req_valid  = itag_req[0].valid;
    assign itag0_req_we     = itag_req[0].we;
    assign itag0_req_addr   = itag_req[0].addr;
    assign itag0_req_wdata  = itag_req[0].wdata;
    assign itag0_req_wstrb  = itag_req[0].wstrb;
    assign itag0_req_rready = itag_req[0].rready;

    logic                  itag0_rsp_wready;
    logic                  itag0_rsp_rvalid;
    logic [TAG_DATA_W-1:0] itag0_rsp_rdata;
    logic                  itag0_rsp_bvalid;
    assign itag0_rsp_wready = itag_rsp[0].wready;
    assign itag0_rsp_rvalid = itag_rsp[0].rvalid;
    assign itag0_rsp_rdata  = itag_rsp[0].rdata;
    assign itag0_rsp_bvalid = itag_rsp[0].bvalid;

`endif

`ifdef VERILATOR
    // M4b tie-offs: a depth-1 port (the D-port) never occupies slot 1 and
    // never holds more than one queue entry. The selectors and queue writes
    // above are tied to entry 0 for it on that premise; check it.
    always_ff @(posedge clk_i) begin
        if (rstn_i) begin
            for (int c = 0; c < N_CACHE; c++) begin
                if (SKID_DEPTH[c] == 1) begin
                    assert (!skid_valid_q[c][1])
                    else $fatal(1, "cache_cntrl: depth-1 port %0d occupied slot 1", c);
                    assert (rq_cnt_q[c] <= 2'd1)
                    else $fatal(1, "cache_cntrl: depth-1 port %0d queued %0d", c, rq_cnt_q[c]);
                end
            end
        end
    end
`endif

endmodule

`resetall

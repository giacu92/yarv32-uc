`resetall
`timescale 1ns / 1ps
`default_nettype none

package rv32_pkg;

    localparam int unsigned XLEN = 32;
    localparam int unsigned STRB_WIDTH = XLEN / 8;

    // Core clock, and the frequency the board UART divides down to hit
    // UART_BAUD. MUST track the rPLL settings in top_module: changing one
    // without the other puts the serial line at the wrong baud, which on a
    // board looks exactly like a dead core. See the note above the rPLL
    // instance in top_module.sv, which owns the derivation.
    localparam int unsigned UART_CLK_HZ = 50_000_000;
    // BOARD baud. Not the simulation's: sim_top defaults to 10 MBaud (5
    // clocks per bit) to keep runs short and overrides this through its own
    // parameter, so do NOT copy the sim default here -- at 50 MHz it makes
    // BAUDDIV 4 and nothing on a real serial line can read it.
    localparam int unsigned UART_BAUD = 115200;

    // Branch-predictor enable A/B knob: -GBP_EN=0 disables prediction and
    // reproduces the pre-predictor core exactly (the baseline). Default 1.
    localparam int unsigned BP_EN = 1;

    // MUL structure A/B knob (see alu.sv). Functionally identical either way,
    // so this is a timing knob only -- the retire stream must not move.
    // MEASURED: 0 is the better one. 1 cost 2.52 ns on the 2026-09-01 PnR
    // ladder (49.6 -> 44.1 MHz); the note in alu.sv has the breakdown.
    localparam int unsigned MUL_SHARED_DSP = 0;

    // PHT lookup placement A/B knob: -GBP_PUSH_LOOKUP=0 reads the PHT at
    // decode with the live GHR (the original form); 1 reads it at
    // instruction-buffer push time and carries the bit in the entry. Unlike
    // MUL_SHARED_DSP this one DOES move the retire stream -- the push-time
    // read sees a slightly older GHR, so predictions differ.
    localparam int unsigned BP_PUSH_LOOKUP = 0;

    // -GEXEC_REDIR_INCYCLE=1 restores the 2026-08-31 form where an execute
    // redirect also launches its read in the redirect cycle. Default 0 keeps
    // the register file off the I-mem address pins (see fetch_stage.sv) and
    // costs 1 cycle per mispredict / trap / mret.
    // Timing closure on 2026-09-03 (wrt BT_EN=1, LSU_LIVE_LOAD=1, others=0) 
    // from 52.890 MHz to 50.065 MHz. Benchmarks score:
    // - CoreMark: from 2.33 to 2.37 CoreMark/MHz
    // - Dhrystone: from 52.02 to 52.40 DMIPS
    localparam int unsigned EXEC_REDIR_INCYCLE = 0;

    // LSU launch shape. 1 = an aligned D-mem LOAD launches live from
    // alu_result (one cycle cheaper per load); 0 = every bus op captures into
    // flops first, which is the pre-2026-09-01 LSU and the shape that closed
    // 50 MHz at +0.093 ns and ran CoreMark on silicon. See execute_stage.sv.
    localparam int unsigned LSU_LIVE_LOAD = 1;

    // ---------------------------------------------------------------
    // Branch-predictor geometry. One place, because three files have to
    // agree: branch_predictor.sv sizes the PHT/GHR from it, and the
    // bp_lookup_rsp_t / bp_train_t / de_t index fields below are all
    // BP_PHT_IDX_W wide (the gshare index snapshot rides D/E to resolve).
    // BP_GHR_W must equal BP_PHT_IDX_W: the gshare index is
    // pc[BP_PHT_IDX_W:1] ^ ghr, a full-width xor with no padding on either
    // side, and the push-time lookup slices ghr the same way.
    //
    // 128 entries x 2 bits, 7 bits of global history. Sized by timing, not
    // by accuracy: the table is read combinationally at decode and that read
    // feeds fetch's launch/inflight logic in the same cycle, so PHT depth is
    // directly on a critical path (2026-08-31 PnR: a 512-entry table put
    // pht_index -> read output at 6.1 ns, 3.36 ns of it pure routing across
    // the spread-out primitives, landing that path at -1.024 ns). The table
    // is a flop array read through a LUT mux, so depth costs what a wide mux
    // costs -- Gowin has no async-read RAM to put it in (see the storage note
    // in branch_predictor.sv).
    //
    // Measured CoreMark cost of shrinking it (ITERATIONS=4, -O3):
    //   512 x 9 -> 2004284 ticks, 18608 mispredicts   (1.99 CoreMark/MHz)
    //   256 x 8 -> 2010433 ticks, 23601 mispredicts   (+0.31% cycles)
    //   128 x 7 -> 2017387 ticks, 24944 mispredicts   (+0.65% cycles)
    // 0.65% of cycles is cheap for ~1 ns of slack -- 1 MHz is worth 2%. Move
    // up to 256 (or back to 512) only if PnR says the slack is there; the
    // three localparams below plus nothing else need to change, since the
    // index width and every struct field carrying it are derived.
    //
    // The original 64x2 / 6-bit table was aliasing-bound, not history-bound:
    // a trace-driven model over the real retire streams put a 64-entry gshare
    // *behind* a 64-entry bimodal on CoreMark (7674 vs 6289 mispredicts).
    // 128 is the first size where the history pays for itself.
    //
    // GHR width == index width is deliberate: gshare xors the full history
    // into the full index, so a shorter GHR wastes index bits.
    //
    // If the accuracy of a big table is ever wanted back without the timing
    // cost, the structural fix is to look the PHT up at instruction-buffer
    // PUSH time and carry the 1-bit prediction in the buffer entry -- that
    // takes the RAM read off the decode->fetch path entirely, at the price of
    // a slightly staler GHR.
    localparam int unsigned BP_PHT_DEPTH = 128;
    localparam int unsigned BP_PHT_IDX_W = 7;  // $clog2(BP_PHT_DEPTH)
    localparam int unsigned BP_GHR_W = 7;  // == BP_PHT_IDX_W (full-width gshare)
    localparam int unsigned BP_RAS_DEPTH = 8;

    // ---------------------------------------------------------------
    // Peripheral address map. These are the single source of truth for
    // the peri xbar windows: the board top and the sim top pass them to
    // the parametric peri xbar (axi4_lite_xbar; the board instantiates
    // N=5) rather than
    // repeating literals, so the map cannot drift between the two.
    //
    //   0x1000_0000 .. 0x1000_0FFF  UART   (axi4_lite_uart)
    //   0x1000_1000 .. 0x1000_2FFF  CLINT  (clint_timer)
    //   0x1000_3000 .. 0x1000_3FFF  MSIP   (msip_peri)
    //   0x1000_5000 .. 0x1000_5FFF  I2C    (axi4_lite_i2c)
    //   0x1000_6000 .. 0x1000_6FFF  SPI    (axi4_lite_spi)
    //
    // MSIP_PERI_ADDR: a write of bit[0] sets/clears mip.MSIP.
    // I2C/SPI: board and sim both instantiate them (sim ties the pins off).
    // 0x1000_4000..0x1000_4FFF is deliberately unmapped: it held an SDIO
    // controller that was dropped from this branch. An access there gets a
    // DECERR from the peri xbar.
    // ---------------------------------------------------------------
    localparam logic [XLEN-1:0] UART_BASE = 32'h1000_0000;
    localparam logic [XLEN-1:0] UART_SIZE = 32'h0000_1000;
    localparam logic [XLEN-1:0] MTIMER_BASE = 32'h1000_1000;
    localparam logic [XLEN-1:0] MTIMER_SIZE = 32'h0000_2000;
    localparam logic [XLEN-1:0] MSIP_PERI_ADDR = 32'h1000_3000;
    localparam logic [XLEN-1:0] MSIP_PERI_SIZE = 32'h0000_1000;
    localparam logic [XLEN-1:0] I2C_BASE = 32'h1000_5000;
    localparam logic [XLEN-1:0] I2C_SIZE = 32'h0000_1000;
    localparam logic [XLEN-1:0] SPI_BASE = 32'h1000_6000;
    localparam logic [XLEN-1:0] SPI_SIZE = 32'h0000_1000;

    // ---------------------------------------------------------------
    // Bus address decode. The LSU steers its own accesses on
    // addr[PERI_ADDR_BIT]: =1 goes out the CPU's peri AXI4-Lite master
    // (UART / CLINT / MSIP / future GPIO), =0 goes to the native D-mem.
    // Fetch has its own dedicated native I-mem port (Harvard), so no
    // crossbar splits memory from peripherals; the only xbar is the
    // 1->3 peri mux at the board top. Default bit 28 (0x1000_0000+ is
    // peripheral), a conventional MMIO base.
    // Moveable here so the map lives in one place, not hardcoded in the
    // LSU or the board top.
    //
    // NOTE: the whole 0x1000_0000..0x1FFF_FFFF region routes to the peri
    // bus, but only the windows above are mapped. An access to the gap
    // gets a DECERR (SLVERR) from the peri xbar, not a hang.
    // ---------------------------------------------------------------
    localparam int unsigned PERI_ADDR_BIT = 28;

    typedef logic [STRB_WIDTH-1:0] strb_t;

    // ---------------------------------------------------------------
    // Native memory interface — split BY DIRECTION (AXI style): each
    // bundle carries one direction only, which is what makes it legal
    // as a packed struct passed through a single port. The channels
    // (request / read response) are mixed inside each bundle.
    //
    // Request launch handshake: req.wvalid && rsp.wready.
    // Read response handshake:  rsp.rvalid && req.rready.
    // ---------------------------------------------------------------

    // master -> bridge  (tutti gli OUTPUT del master)
    typedef struct packed {
        logic                  wvalid;  // request valid (launch)
        logic                  we;      // 1 = write, 0 = read
        logic [XLEN-1:0]       addr;    // byte address
        logic [XLEN-1:0]       wdata;   // write data (ignored if we=0)
        logic [STRB_WIDTH-1:0] wstrb;   // byte strobes; all-1 on a word store
        logic                  rready;  // master ready to accept read data
    } mem_req_t;

    // bridge -> master  (tutti gli INPUT del master)
    typedef struct packed {
        logic            wready;  // bridge accepts the request (= idle)
        logic            rvalid;  // read data valid this cycle
        logic [XLEN-1:0] rdata;   // read data
        logic            bvalid;  // write-ack valid this cycle (store retire)
    } mem_rsp_t;

    // ---------------------------------------------------------------
    // Instruction-fetch interface — read-only, 64-bit. The I-mem port is
    // widened to deliver two 32-bit words per access (3-4 with RVC), so it
    // cannot ride the XLEN-wide mem_req_t/mem_rsp_t (whose rdata is 32-bit).
    // Fetch issues aligned 8-byte reads; a read has no write side, so the
    // request carries only valid/addr/rready and the response only
    // ready/rvalid/rdata. D-mem and the peri bridge keep using mem_req_t /
    // mem_rsp_t above.
    // ---------------------------------------------------------------
    localparam int unsigned IFETCH_DW = 64;  // fetch word width (bytes 0..7)

    typedef struct packed {
        logic            valid;   // request valid (launch a read)
        logic [XLEN-1:0] addr;    // byte address (8-byte aligned in steady state)
        logic            rready;  // master ready to accept read data
    } ifetch_req_t;

    typedef struct packed {
        logic                 ready;   // slave accepts the request (skid not full)
        logic                 rvalid;  // read data valid this cycle
        logic [IFETCH_DW-1:0] rdata;   // 64-bit read data
    } ifetch_rsp_t;

    // ---------------------------------------------------------------
    // Branch-predictor interface. Three bundles, split by direction (the
    // same convention as mem_req_t/mem_rsp_t and ifetch_req_t/ifetch_rsp_t):
    // one struct port per direction/source instead of a fan of individual
    // wires on decode / execute / the predictor. The predictor is a
    // combinational-lookup, resolve-trained block (see branch_predictor.sv).
    //
    //   bp_lookup_req_t : decode  -> predictor  (the CF instr at the head)
    //   bp_lookup_rsp_t : predictor -> decode   (PHT direction + RAS top)
    //   bp_train_t      : execute -> predictor  (one resolved CF instr)
    // ---------------------------------------------------------------
    // Decode -> predictor: the control-flow instruction decode is looking at
    // this cycle. The
    // predictor needs no kind bits to answer: the PHT entry and the RAS top
    // are both exported unconditionally and decode selects between them, so
    // the only other field is the return-lookup event used by the sim-only
    // RAS counters. It carries decode's consume condition (~stall & ~flush)
    // because lookup_req is a held level: a return sitting at the head across
    // an execute stall would otherwise be counted once per stall cycle.
    typedef struct packed {
        logic [XLEN-1:0] pc;           // PC of the control-flow instr (gshare index src)
        logic            ret_consume;  // a return lookup is consumed this cycle (stats only)
    } bp_lookup_req_t;

    // Fetch -> predictor: the PUSH-TIME lookup (BP_PUSH_LOOKUP=1). Fetch
    // presents the PC stamp of each 32-bit word it is about to push into the
    // instruction buffer -- up to two per cycle. Both PCs are combinational
    // off req_pc_q / pc_q, i.e. flops, so this read has a whole cycle to
    // itself instead of sitting on the decode -> fetch redirect path.
    typedef struct packed {
        logic [XLEN-1:0] pc0;  // PC stamp of the first word pushed this cycle
        logic [XLEN-1:0] pc1;  // PC stamp of the second (when two are pushed)
    } bp_push_req_t;

    // Predictor -> fetch: two direction bits per word, one for each halfword
    // slot the word can be decoded from (pc with bit[1]=0 and bit[1]=1), plus
    // the GHR snapshot both were read with. Fetch stores these in the buffer
    // entry; decode selects by src_pc[1] and recomputes the gshare index as
    // src_pc[BP_PHT_IDX_W:1] ^ ghr, which is the index the bits were read at.
    //
    // Two bits per word rather than one because a 32-bit buffer entry can be
    // decoded from either halfword: the low half normally, the high half after
    // a redirect into the middle of a word or when the RVC hold buffer carries
    // it. Reading both costs one extra PHT entry, not one extra port: the two
    // gshare indices differ in exactly their LSB (see the push lookup in
    // branch_predictor.sv), so they are an aligned pair.
    typedef struct packed {
        logic [BP_GHR_W-1:0] ghr;       // GHR snapshot both words were read with
        logic                pred0_lo;  // word 0, pc[1]=0
        logic                pred0_hi;  // word 0, pc[1]=1
        logic                pred1_lo;  // word 1, pc[1]=0
        logic                pred1_hi;  // word 1, pc[1]=1
    } bp_push_rsp_t;

    // Predictor -> decode: the looked-up prediction.
    typedef struct packed {
        logic                    pht_taken;  // PHT[counter].MSB (predict taken)
        logic                    ras_valid;  // RAS non-empty
        logic [XLEN-1:0]         ras_top;    // RAS top (predicted return target)
        logic [BP_PHT_IDX_W-1:0] pht_index;  // pc[7:1]^ghr snapshot (carried in de_t)
    } bp_lookup_rsp_t;

    // Execute -> predictor: training at resolve. Kind bits are mutually
    // exclusive; exactly the state for that kind updates. train_pht_index is
    // the de_t snapshot so the PHT update uses the history the branch was
    // predicted with. train_push_pc is pc-link, the return address for a push.
    typedef struct packed {
        logic                    valid;
        logic                    cond;       // conditional -> PHT sat-update + GHR shift
        logic                    call;       // call -> RAS push push_pc
        logic                    ret;        // return -> RAS pop
        logic                    taken;      // resolved taken outcome
        logic [BP_PHT_IDX_W-1:0] pht_index;
        logic [XLEN-1:0]         push_pc;
    } bp_train_t;

    // ---------------------------------------------------------------
    // Decode: opcodes, ALU/branch/source enums, and the D/E control
    // struct. RV32I + M + C + Zilx + Zicsr + Zifencei all decode AND
    // execute; nothing in this list is stubbed out as illegal any more.
    // ---------------------------------------------------------------

    // Major opcodes (instr[6:0]).
    localparam logic [6:0] OPC_LUI = 7'b0110111;
    localparam logic [6:0] OPC_AUIPC = 7'b0010111;
    localparam logic [6:0] OPC_JAL = 7'b1101111;
    localparam logic [6:0] OPC_JALR = 7'b1100111;
    localparam logic [6:0] OPC_BRANCH = 7'b1100011;
    localparam logic [6:0] OPC_LOAD = 7'b0000011;
    localparam logic [6:0] OPC_STORE = 7'b0100011;
    localparam logic [6:0] OPC_OP_IMM = 7'b0010011;
    localparam logic [6:0] OPC_OP = 7'b0110011;  // M is OPC_OP with funct7=0000001
    localparam logic [6:0] OPC_MISC_MEM = 7'b0001111;  // fence / fence.i (Zifencei), retire as nop
    localparam logic [6:0] OPC_SYSTEM = 7'b1110011;  // CSR (Zicsr) / ecall / ebreak / mret / wfi
    localparam logic [6:0] OPC_AMO = 7'b0101111;  // Zilx indexed loads

    // ALU operation (base RV32I + M extension + Zilx EA). 19 values -> 5 bits.
    typedef enum logic [4:0] {
        // -------- Base ALU operations --------
        ALU_ADD,
        ALU_SUB,
        ALU_SLL,
        ALU_SLT,
        ALU_SLTU,
        ALU_XOR,
        ALU_SRL,
        ALU_SRA,
        ALU_OR,
        ALU_AND,

        // -------- M extension (funct7 == 0000001) --------
        ALU_MUL,
        ALU_MULH,
        ALU_MULHSU,
        ALU_MULHU,
        ALU_DIV,
        ALU_DIVU,
        ALU_REM,
        ALU_REMU,

        // -------- Zilx indexed-load EA (RV32) --------
        // EA = base + (index << shamt). operand_a = rs2_data (base),
        // operand_b = rs1_data (index); shamt comes from de_t.mem_shamt
        // (0 unscaled, log2(access_size) scaled). The load itself is the
        // LSU's job (mem_read / WB_MEM); ALU_LX computes the address only.
        ALU_LX
    } alu_op_t;

    // Branch / jump type (JAL/JALR encoded here too — no separate flags).
    typedef enum logic [3:0] {
        BR_NONE,
        BR_BEQ,
        BR_BNE,
        BR_BLT,
        BR_BGE,
        BR_BLTU,
        BR_BGEU,
        BR_JAL,
        BR_JALR
    } branch_t;

    // ALU operand-A source. ALU_A_CSR is gone (2026-09-02): no opcode ever
    // produced it -- a Zicsr op's old value reaches rd through WB_CSR and the
    // RMW through csr_rdata_i directly, neither of which goes near the ALU --
    // and while it existed it hung csr_rdata_i off the operand-A mux, which
    // sits on regfile -> forward -> operand -> DSP, the design's worst path.
    typedef enum logic [1:0] {
        ALU_A_RS1,
        ALU_A_PC,
        ALU_A_RS2   // Zilx base (spec: rs2 = base, rs1 = index)
    } alu_src_a_t;
    // ALU operand-B source. Zilx reuses ALU_B_RS1 (the index, rs1) — the
    // <<shamt shift is keyed on alu_op==ALU_LX inside the ALU, not on this
    // enum, so a dedicated "shifted rs1" select is not needed.
    //
    // THREE values, and the width is what the timing is about (2026-09-02).
    // It used to hold five in 3 bits, which makes the operand-B mux an 8:1
    // tree -- three LUT levels -- in front of the MUL DSP, on the path that
    // limits the design (regfile -> forward -> operand -> DSP -> alu_result
    // -> wb_data). Two of the five were never produced by any opcode
    // (ALU_B_ZERO, and ALU_B_PC4 after the change below), so the mux is now a
    // 4:1 -- two levels -- and every input is a flop:
    //   ALU_B_PC4 existed only for JAL, whose alu_result is DISCARDED
    //     (wb_src=WB_PC4 takes pc_link directly, the redirect target comes
    //     from the separate branch_target adder, and JAL is not a memory or
    //     CSR op). JAL now selects ALU_B_IMM, so alu_result becomes pc+imm --
    //     still unread -- and pc_link, an adder output, leaves the mux.
    //   ALU_B_ZERO was dead from the start: decode's default is ALU_B_IMM.
    typedef enum logic [1:0] {
        ALU_B_RS1,
        ALU_B_IMM,
        ALU_B_RS2
    } alu_src_b_t;
    // Write-back source. WB_CSR writes the OLD CSR value to rd (Zicsr:
    // rd <- csr[addr] before the RMW side effect commits).
    typedef enum logic [1:0] {
        WB_ALU,
        WB_MEM,
        WB_PC4,
        WB_CSR
    } wb_src_t;
    // Memory access size.
    typedef enum logic [1:0] {
        MS_B,
        MS_H,
        MS_W
    } mem_size_t;
    // CSR access type (Zicsr subset).
    typedef enum logic [2:0] {
        CSR_NONE,
        CSR_RW,
        CSR_RS,
        CSR_RC,
        CSR_RWI,
        CSR_RSI,
        CSR_RCI
    } csr_op_t;

    // Branch-prediction source (which predictor produced a prediction). Rides
    // the D/E register so execute can compare the predicted direction/target
    // against the resolved one (mispredict) and so the sim counters attribute
    // hits/misses. PRED_NONE = no prediction was made (decode left it to
    // execute, the legacy path); PRED_DIRECT = unconditional JAL/c.j/c.jal or
    // a not-taken conditional (target computed in decode as pc+imm);
    // PRED_PHT = a conditional branch whose direction came from the gshare
    // PHT; PRED_RAS = a JALR return whose target came from the RAS.
    typedef enum logic [1:0] {
        PRED_NONE,
        PRED_DIRECT,
        PRED_PHT,
        PRED_RAS
    } pred_source_t;

    // System-op kind (OPC_SYSTEM funct3=0 + OPC_MISC_MEM fence/fence.i).
    // Rides the D/E register so execute can pick the trap-entry / return /
    // halt / nop behaviour. SYS_NONE = not a system op.
    typedef enum logic [2:0] {
        SYS_NONE,
        SYS_ECALL,
        SYS_EBREAK,
        SYS_MRET,
        SYS_WFI,
        SYS_FENCE,
        SYS_FENCE_I
    } sys_op_t;

    // mcause exception / interrupt codes (mcause[31]=1 -> interrupt).
    // XLEN-wide so they map 1:1 onto the mcause register.
    localparam logic [XLEN-1:0] MCAUSE_INSTR_ACC = 32'h0000_0001;  // instruction access fault
    localparam logic [XLEN-1:0] MCAUSE_ILLEGAL = 32'h0000_0002;  // illegal instruction
    localparam logic [XLEN-1:0] MCAUSE_BREAKPOINT = 32'h0000_0003;  // ebreak
    localparam logic [XLEN-1:0] MCAUSE_LAD_MIS = 32'h0000_0004;  // load addr misaligned
    localparam logic [XLEN-1:0] MCAUSE_SAD_MIS = 32'h0000_0006;  // store/AMO addr misaligned
    localparam logic [XLEN-1:0] MCAUSE_ECALL_M = 32'h0000_000B;  // env call from M-mode (11)
    localparam logic [XLEN-1:0] MCAUSE_MSI = 32'h8000_0003;  // machine software interrupt
    localparam logic [XLEN-1:0] MCAUSE_MTI = 32'h8000_0007;  // machine timer interrupt
    localparam logic [XLEN-1:0] MCAUSE_MEI = 32'h8000_000B;  // machine external interrupt

    // mstatus machine-mode field bit positions (RV32).
    localparam int MSTATUS_MIE_BIT = 3;  // M-mode global interrupt enable
    localparam int MSTATUS_MPIE_BIT = 7;  // prior MIE (saved on trap entry)
    localparam int MSTATUS_MPP_LO = 11;  // MPP[1:0]: 00=U, 01=S, 11=M
    localparam int MSTATUS_MPP_HI = 12;
    // Only M-mode is implemented, so MPP is effectively read-only 2'b11.
    localparam logic [1:0] MSTATUS_MPP_M = 2'b11;
    // mstatus reset value: MPP = M, everything else 0 (MIE=0, MPIE=0).
    localparam logic [XLEN-1:0] MSTATUS_RESET = 32'h0000_1800;

    // mtvec MODE field (mtvec[1:0]).
    localparam logic [1:0] MTVEC_VECTORED = 2'b01;


    typedef enum logic [11:0] {
        CSR_ADDR_MSTATUS  = 12'h300,
        CSR_ADDR_MISA     = 12'h301,
        CSR_ADDR_MIE      = 12'h304,
        CSR_ADDR_MTVEC    = 12'h305,
        CSR_ADDR_MSCRATCH = 12'h340,
        CSR_ADDR_MEPC     = 12'h341,
        CSR_ADDR_MCAUSE   = 12'h342,
        CSR_ADDR_MTVAL    = 12'h343,
        CSR_ADDR_MIP      = 12'h344,
        CSR_ADDR_MCYCLE   = 12'hB00,
        CSR_ADDR_MINSTRET = 12'hB02
    } csr_addr_t;

    // D/E pipeline register: everything decode produces for a (future)
    // execute stage. Single-direction packed struct, legal as one port
    // (matches the mem_req_t / mem_rsp_t convention).
    typedef struct packed {
        logic valid;  // a decoded instr is held this cycle
        logic [XLEN-1:0] pc;  // instr PC (P for low/32-bit, P+2 for upper half)
        logic [XLEN-1:0] instr;  // 32-bit word decode treated (native or RVC-expanded)
        logic is_compressed;  // source was a 16-bit RVC instr
        // rs1_addr rides D/E because execute needs it to classify a JALR as a
        // RAS return (rs1 in {x1,x5}). There is deliberately NO rs2_addr: it
        // was carried until 2026-09-02 and read by nothing -- the regfile read
        // ports are driven by decode's own combinational rs1_addr_o/rs2_addr_o,
        // not from this struct -- so it was five dead flops per pipeline stage.
        // `make lint` (UNUSEDSIGNAL) is what surfaced it.
        logic [4:0] rs1_addr;
        logic [XLEN-1:0] rs1_data;  // operand captured at decode (async read)
        logic [XLEN-1:0] rs2_data;
        logic [XLEN-1:0] imm;  // sign-extended I/S/B/U/J immediate
        logic [4:0] rd;
        logic reg_write;  // write back to rd
        logic csr_wren;  // write back to CSR (Zicsr subset)
        csr_op_t csr_op;  // CSR access type (Zicsr subset)
        logic [11:0] csr_addr;  // CSR address (Zicsr subset)
        alu_op_t alu_op;
        alu_src_a_t alu_src_a;
        alu_src_b_t alu_src_b;
        logic mem_read;
        logic mem_write;
        mem_size_t mem_size;
        logic mem_unsigned;  // load zero-extend (LBU/LHU)
        logic [1:0] mem_shamt;  // Zilx index scale (0 unscaled, log2 size scaled)
        wb_src_t wb_src;
        branch_t branch_type;
        sys_op_t sys_op;  // OPC_SYSTEM funct3=0 / fence ops
        logic exception;  // decode requests a sync trap now
        logic [XLEN-1:0] exception_cause;  // mcause for the sync trap
        logic [XLEN-1:0] exception_tval;  // mtval for the sync trap (illegal=instr, else 0)
        logic illegal;  // opcode/encoding not decoded this phase
        // Branch-prediction metadata (prediction-at-decode). pred_valid marks
        // a control-flow instr decode attempted to predict; pred_taken is the
        // speculated direction; pred_target is the taken target (valid when
        // pred_taken); pred_source attributes the hit; pred_pht_index is the
        // gshare index snapshot (pc[7:1]^ghr) taken at decode, carried so the
        // PHT update at resolve uses the history the branch was predicted with
        // (an older branch may have shifted the GHR in between). Execute
        // compares these against the resolved outcome -> mispredict.
        logic pred_valid;
        logic pred_taken;
        logic [XLEN-1:0] pred_target;
        // WRITE-ONLY, on purpose. No RTL and no C++ reads pred_source; it
        // exists so a VCD shows WHICH source predicted a branch (direct / PHT
        // / RAS) when a mispredict is being debugged, which the sim's
        // aggregate accuracy and RAS counters cannot attribute. `make lint`
        // reports it as unused every run -- that is expected, do not chase it.
        // Cost is two flops per pipeline stage; delete it if that ever matters.
        pred_source_t pred_source;
        logic [BP_PHT_IDX_W-1:0] pred_pht_index;
    } de_t;

endpackage

`resetall

`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* CSR register file — machine-mode Zicsr subset.
*
* One async read port (decode/execute drives the address via de_t.csr_addr)
* and one sync write port (posedge, whole-word write) driven by execute's
* CSR RMW result (csr_wdata / csr_wren).
*
* Implemented CSRs (machine mode): mstatus, misa, mie, mtvec, mscratch,
* mepc, mcause, mtval, mip, mcycle, minstret (11 entries, NCSR), plus the
* upper halves mcycleh / minstreth (separate flops) and the read-only Zicntr
* shadows cycle / time / instret and their h halves (2026-11-22). time is the
* CLINT's mtime, wired in on mtime_i, so software reading the CSR and the
* MMIO register sees one clock.
*
* Counter write semantics (priv. spec): a CSR write to either half of
* mcycle / minstret replaces that half, and the instruction doing the write
* does not also increment the counter -- the value written is the value the
* next instruction reads. That is what rv32mi-instret_overflow case 3 checks
* (a write to minstreth must suppress the minstret increment too).
* Unimplemented CSR addresses read as 0 and ignore writes (no
* illegal-instruction trap yet, so a read of an unimplemented CSR returns 0
* rather than trapping).
*
* Storage is FF + LUT mux, NOT a BSRAM: the implemented CSRs occupy a
* sparse, small slice of the 12-bit CSR address space (0x300..0x344), so
* they are selected by a case statement on csr_addr_i, not by direct array
* indexing. A BSRAM async read needs a contiguous index, which the sparse
* CSR map does not provide, so `(* ram_style = "block" *)` would be ignored
* anyway (and was misleading). 11 registers is cheap as flops. Unlike the
* GPR file, these CSRs ARE reset: machine-mode CSRs have architected reset
* values, and with only 11 entries a sync reset loop is trivial (no BSRAM
* single-write-port constraint, no DO->DI timing issue).
*
* Naming: ports *_i/_o; internal signals no prefix. The storage array is
* `regs` (one flop per implemented CSR); no _q/_d (array, not single flop).
*/

module csr_regfile (
    input wire clk_i,
    input wire rstn_i,

    // One async read port (decode/execute drives the address).
    input  wire [    11:0] csr_addr_i,
    output wire [XLEN-1:0] csr_data_o,

    // One sync write port (driven by execute's CSR RMW result).
    input wire            csr_wren_i,
    input wire [XLEN-1:0] csr_data_i,

    // Trap-write bundle (from the trap unit, one cycle, each a distinct
    // CSR). Separate from the RMW port so a trap entry can write mepc /
    // mcause / mtval / mstatus in the same cycle a csrrw would have used
    // the single RMW port (trap suppresses the RMW, so they are mutually
    // exclusive — but the trap port wins on conflict).
    input wire            we_mepc_i,
    input wire [XLEN-1:0] d_mepc_i,
    input wire            we_mcause_i,
    input wire [XLEN-1:0] d_mcause_i,
    input wire            we_mtval_i,
    input wire [XLEN-1:0] d_mtval_i,
    input wire            we_mstatus_i,
    input wire [XLEN-1:0] d_mstatus_i,

    // Machine software interrupt pending bit, driven by the MSIP MMIO
    // slave (a write of bit[0] to MSIP_PERI_ADDR sets/clears mip.MSIP).
    // mip.MSIP is read-only from CSR writes (SW cannot clear it by
    // writing mip); the slave is the only source.
    input wire msip_i,

    // Machine timer interrupt pending bit, driven by the CLINT timer MMIO
    // slave (mtime >= mtimecmp). Like MSIP, mip.MTIP is read-only from CSR
    // writes (SW clears it by writing mtimecmp > mtime, not by writing mip);
    // the slave is the only source.
    input wire mtip_i,

    // Machine external interrupt pending bit, driven by the peripheral IRQ
    // OR-tree at the board top (currently the UART's level IRQ). Like MSIP /
    // MTIP, mip.MEIP is read-only from CSR writes; SW clears it by servicing
    // the peripheral (e.g. draining UART RX / disabling its CTRL enable),
    // not by writing mip.
    input wire meip_i,

    // Combinational CSR taps for the trap unit (mtvec / mepc redirect,
    // mstatus MIE/MPIE, mip / mie for interrupt pending). Separate
    // outputs so the trap path does not steal the async RMW read.
    output wire [XLEN-1:0] mtvec_o,
    output wire [XLEN-1:0] mepc_o,
    output wire [XLEN-1:0] mstatus_o,
    output wire [XLEN-1:0] mip_o,
    output wire [XLEN-1:0] mie_o,

    // The CLINT's 64-bit mtime, for the read-only time / timeh CSRs.
    input wire [63:0] mtime_i,

    // Instruction retire stat (consumed by minstret). Future
    // mhpmcounter / branch-prediction counters would add their own
    // retire-side inputs here when implemented.
    input wire instr_retire_i,  // instruction retired (commit)

    // Current privilege (linux-soc: M and U only, no S). One flop, written
    // by the trap unit on trap entry (-> M) and on mret (-> mstatus.MPP).
    // 1 = M, 0 = U. Resets to M.
    input  wire priv_we_i,
    input  wire priv_d_i,
    output wire priv_m_o
);
    // Index map: implemented CSRs packed into a 9-entry array. The case
    // statements below translate the 12-bit CSR address to/from this index.
    localparam int unsigned NCSR = 11;
    logic [XLEN-1:0] regs                                                             [NCSR-1:0];
    logic [XLEN-1:0] csr_rdata;

    // Upper halves of the 64-bit counters; the low halves stay regs[9]
    // (mcycle) and regs[10] (minstret).
    logic [XLEN-1:0] mcycleh_q;
    logic [XLEN-1:0] minstreth_q;

    wire             wr_mcycle = csr_wren_i && (csr_addr_i == CSR_ADDR_MCYCLE);
    wire             wr_mcycleh = csr_wren_i && (csr_addr_i == CSR_ADDR_MCYCLEH);
    wire             wr_minstret = csr_wren_i && (csr_addr_i == CSR_ADDR_MINSTRET);
    wire             wr_minstreth = csr_wren_i && (csr_addr_i == CSR_ADDR_MINSTRETH);
    wire  [    63:0] mcycle_inc = {mcycleh_q, regs[9]} + 64'd1;
    // instr_retire_i is execute's COMMIT-cycle pulse (op_commits), the same
    // cycle a CSR instruction's write lands -- so a minstret/minstreth write
    // suppresses exactly its own instruction's increment. (It used to be the
    // registered ex_valid, a cycle late: a csrr right after an instruction
    // then read a count that did not yet include it.)
    wire  [    63:0] minstret_inc = {minstreth_q, regs[10]} + {63'd0, instr_retire_i};

    // Architected reset values (M-mode). misa = RV32 I/M/A/C:
    //   MXL[31:30]=1 (RV32), A=bit0, C=bit2, I=bit8, M=bit13.
    // linux-soc, 2026-11-22: A is implemented (LR/SC + AMOs, execute_stage),
    // so bit 0 is set again. The history below is `main`'s.
    // A (bit 0) is deliberately CLEAR: this core decodes every standard AMO /
    // LR / SC as illegal (decode_stage.sv, OPC_AMO carries only Zilx), so
    // advertising A would invite software that trusts misa -- a Linux kernel
    // or OpenSBI -- to execute instructions that trap. Until 2026-11-22 this
    // read 0x4000_2105, i.e. it claimed A. Set bit 0 again only together with
    // a real A implementation. Guarded by sw/isa/csr_ops.
    // mstatus resets with MPP = 2'b11 (M): only M-mode is implemented, so
    // MPP is effectively read-only 11 (see MSTATUS_RESET in rv32_pkg).
    // linux-soc, 2026-11-27: U-mode is implemented (bit 20). A nommu M-mode
    // Linux kernel runs userspace in U (start_thread clears MPP) and only
    // handles ecall cause 8, so M-only cannot run a single syscall.
    localparam logic [XLEN-1:0] MISA_RESET = 32'h4010_2105;
    logic priv_m_q;

    always_comb begin
        case (csr_addr_i)
            CSR_ADDR_MSTATUS:   csr_rdata = regs[0];
            CSR_ADDR_MISA:      csr_rdata = regs[1];
            CSR_ADDR_MIE:       csr_rdata = regs[2];
            CSR_ADDR_MTVEC:     csr_rdata = regs[3];
            CSR_ADDR_MSCRATCH:  csr_rdata = regs[4];
            CSR_ADDR_MEPC:      csr_rdata = regs[5];
            CSR_ADDR_MCAUSE:    csr_rdata = regs[6];
            CSR_ADDR_MTVAL:     csr_rdata = regs[7];
            CSR_ADDR_MIP:       csr_rdata = regs[8];
            CSR_ADDR_MCYCLE:    csr_rdata = regs[9];
            CSR_ADDR_MINSTRET:  csr_rdata = regs[10];
            CSR_ADDR_MCYCLEH:   csr_rdata = mcycleh_q;
            CSR_ADDR_MINSTRETH: csr_rdata = minstreth_q;
            CSR_ADDR_CYCLE:     csr_rdata = regs[9];
            CSR_ADDR_TIME:      csr_rdata = mtime_i[31:0];
            CSR_ADDR_INSTRET:   csr_rdata = regs[10];
            CSR_ADDR_CYCLEH:    csr_rdata = mcycleh_q;
            CSR_ADDR_TIMEH:     csr_rdata = mtime_i[63:32];
            CSR_ADDR_INSTRETH:  csr_rdata = minstreth_q;
            default:            csr_rdata = '0;  // Unimplemented CSRs read as 0
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            priv_m_q    <= 1'b1;
            mcycleh_q   <= '0;
            minstreth_q <= '0;
            for (int i = 0; i < NCSR; i++) begin

                if (i == 0) begin
                    regs[i] <= MSTATUS_RESET;  // MPP = 2'b11 (M), MIE = MPIE = 0
                end else if (i == 1) begin
                    regs[i] <= MISA_RESET;  // misa is read-only WARL, so reset to RV32IMAC
                end else begin
                    regs[i] <= '0;
                end
            end
        end else begin

            // 64-bit counters. mcycle counts every cycle, minstret every
            // retire. A CSR write to EITHER half suppresses the whole
            // counter's increment that cycle -- the written value is what
            // the next instruction reads, and the untouched half holds
            // (no carry from an increment that did not happen).
            if (!(wr_mcycle || wr_mcycleh)) begin
                regs[9]   <= mcycle_inc[31:0];
                mcycleh_q <= mcycle_inc[63:32];
            end
            if (!(wr_minstret || wr_minstreth)) begin
                regs[10]    <= minstret_inc[31:0];
                minstreth_q <= minstret_inc[63:32];
            end

            if (priv_we_i) priv_m_q <= priv_d_i;

            // --- Trap-write bundle (one cycle, each a distinct CSR).
            // Priority over the RMW port: a trap suppresses the RMW this
            // cycle, so they never collide, but the trap port wins anyway. ---
            if (we_mstatus_i) regs[0] <= d_mstatus_i;
            if (we_mepc_i) regs[5] <= d_mepc_i;
            if (we_mcause_i) regs[6] <= d_mcause_i;
            if (we_mtval_i) regs[7] <= d_mtval_i;

            // --- mstatus RMW (only if no trap write). M and U exist, so
            // MPP is WARL {00, 11}: 00 stays U, anything else (including the
            // unsupported S encoding 01) reads back M. Other bits raw (only
            // M-mode can write mstatus). ---
            if (!we_mstatus_i && csr_wren_i && (csr_addr_i == CSR_ADDR_MSTATUS))
                regs[0] <= {
                    csr_data_i[31:13],
                    (csr_data_i[12:11] == MSTATUS_MPP_U) ? MSTATUS_MPP_U : MSTATUS_MPP_M,
                    csr_data_i[10:0]
                };

            // --- mtvec RMW: BASE in [31:2] raw; MODE masked to direct /
            // vectored only ({1'b0, bit[0]} — bit[1] cleared, other modes
            // collapse to direct). ---
            if (csr_wren_i && (csr_addr_i == CSR_ADDR_MTVEC))
                regs[3] <= {csr_data_i[31:2], 1'b0, csr_data_i[0]};

            // --- mie RMW: raw. MSIE / MTIE / MEIE are all backed by a real
            // pending source (msip_peri / clint_timer / the peri IRQ tree). ---
            if (csr_wren_i && (csr_addr_i == CSR_ADDR_MIE)) regs[2] <= csr_data_i;

            // --- mscratch RMW: raw. ---
            if (csr_wren_i && (csr_addr_i == CSR_ADDR_MSCRATCH)) regs[4] <= csr_data_i;

            // --- mepc / mcause / mtval RMW: raw (trap write above wins). ---
            if (!we_mepc_i && csr_wren_i && (csr_addr_i == CSR_ADDR_MEPC)) regs[5] <= csr_data_i;
            if (!we_mcause_i && csr_wren_i && (csr_addr_i == CSR_ADDR_MCAUSE))
                regs[6] <= csr_data_i;
            if (!we_mtval_i && csr_wren_i && (csr_addr_i == CSR_ADDR_MTVAL)) regs[7] <= csr_data_i;

            // --- mip: MSIP (bit3), MTIP (bit7) and MEIP (bit11) track
            // msip_i / mtip_i / meip_i every cycle (read-only from CSR write —
            // SW clears them at the source: the MSIP MMIO register, mtimecmp,
            // or the peripheral raising the external IRQ). The other bits are
            // RMW-writable. ---
            regs[8][3]  <= msip_i;
            regs[8][7]  <= mtip_i;
            regs[8][11] <= meip_i;
            if (csr_wren_i && (csr_addr_i == CSR_ADDR_MIP)) begin
                regs[8][2:0]   <= csr_data_i[2:0];
                regs[8][6:4]   <= csr_data_i[6:4];
                regs[8][10:8]  <= csr_data_i[10:8];
                regs[8][31:12] <= csr_data_i[31:12];
            end

            // --- mcycle / minstret RMW (overrides the increment above). ---
            if (wr_mcycle) regs[9] <= csr_data_i;
            if (wr_mcycleh) mcycleh_q <= csr_data_i;
            if (wr_minstret) regs[10] <= csr_data_i;
            if (wr_minstreth) minstreth_q <= csr_data_i;
            // cycle / time / instret and their h halves are read-only
            // shadows: a write to them is ignored here (and is an illegal
            // instruction once the illegal-CSR trap exists).
            // misa is read-only here: ignore writes.
        end
    end

    // Registered read (experiment): the decode-side address is presented one
    // cycle and the data comes out the next, so the read mux is no longer in
    // the same cycle as everything it feeds. Execute holds the CSR op an
    // extra cycle to collect it.
    logic [XLEN-1:0] csr_rdata_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) csr_rdata_q <= '0;
        else csr_rdata_q <= csr_rdata;
    end

    assign csr_data_o = csr_rdata_q;

    // Trap-unit taps (combinational). mip_o reflects the live msip_i /
    // mtip_i / meip_i bits via regs[8] (updated every cycle above).
    assign mtvec_o    = regs[3];
    assign mepc_o     = regs[5];
    assign mstatus_o  = regs[0];
    assign mip_o      = regs[8];
    assign mie_o      = regs[2];
    assign priv_m_o   = priv_m_q;

`ifdef VERILATOR
    // Sim-only: mirror the reset values at time 0 so waveforms/logs do not
    // show X before the first reset edge. Matches the sync reset above.
    initial begin
        regs[0]     = MSTATUS_RESET;
        regs[1]     = MISA_RESET;
        regs[2]     = '0;
        regs[3]     = '0;
        regs[4]     = '0;
        regs[5]     = '0;
        regs[6]     = '0;
        regs[7]     = '0;
        regs[8]     = '0;
        regs[9]     = '0;
        regs[10]    = '0;
        mcycleh_q   = '0;
        minstreth_q = '0;
        priv_m_q    = 1'b1;
    end
`endif

endmodule

`resetall

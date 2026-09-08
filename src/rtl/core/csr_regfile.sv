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
* mepc, mcause, mtval, mip, mcycle/mcycleh, minstret/minstreth
* (13 entries, NCSR). The high halves carry from their low half so the
* pair forms the RV32-mandated 64-bit counter. A write to either half of
* minstret suppresses that instruction's increment of the pair (RISC-V:
* a write to instreth is a write to instret for suppression purposes),
* which is what rv32mi-instret_overflow case 3 checks. mcycle is
* free-running; a write to mcycle overrides that cycle's increment, a
* write to mcycleh sets the high half (mcycle keeps counting).
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

    // Instruction retire stat (consumed by minstret). Future
    // mhpmcounter / branch-prediction counters would add their own
    // retire-side inputs here when implemented.
    input wire instr_retire_i  // instruction retired (commit)
);
    // Index map: implemented CSRs packed into a 13-entry array. The case
    // statements below translate the 12-bit CSR address to/from this index.
    //  {0..8}  = mstatus/misa/mie/mtvec/mscratch/mepc/mcause/mtval/mip
    //  {9,11}  = mcycle / mcycleh   (64-bit free-running cycle counter)
    //  {10,12} = minstret / minstreth (64-bit retired-instruction counter)
    localparam int unsigned NCSR = 13;
    logic [XLEN-1:0] regs[NCSR-1:0];
    logic [XLEN-1:0] csr_rdata;

    // Architected reset values (M-mode). misa = RV32 I/M/A/C:
    //   MXL[31:30]=1 (RV32), A=bit0, C=bit2, I=bit8, M=bit13.
    // mstatus resets with MPP = 2'b11 (M): only M-mode is implemented, so
    // MPP is effectively read-only 11 (see MSTATUS_RESET in rv32_pkg).
    localparam logic [XLEN-1:0] MISA_RESET = 32'h4000_2105;

    // Per-half write strobes, used to gate the free-running increments and
    // the inter-half carry. Declared at module scope (Gowin EX3990: no
    // in-block variable decls).
    logic mcycle_wr;
    logic mcycleh_wr;
    logic minstret_wr;
    logic minstreth_wr;
    logic instret_wr;       // write to either minstret half this cycle
    logic instret_inc;      // this cycle's retire increments minstret
    logic minstret_carry;   // low-half wraps into the high half

    assign mcycle_wr     = csr_wren_i && (csr_addr_i == CSR_ADDR_MCYCLE);
    assign mcycleh_wr    = csr_wren_i && (csr_addr_i == CSR_ADDR_MCYCLEH);
    assign minstret_wr   = csr_wren_i && (csr_addr_i == CSR_ADDR_MINSTRET);
    assign minstreth_wr  = csr_wren_i && (csr_addr_i == CSR_ADDR_MINSTRETH);
    assign instret_wr    = minstret_wr || minstreth_wr;
    // A write to either half of minstret suppresses the retire increment
    // (RISC-V: a write to instreth counts as a write to instret).
    assign instret_inc   = instr_retire_i && !instret_wr;
    assign minstret_carry = instret_inc && (regs[10] == 32'hFFFF_FFFF);

    always_comb begin
        case (csr_addr_i)
            CSR_ADDR_MSTATUS:  csr_rdata = regs[0];
            CSR_ADDR_MISA:     csr_rdata = regs[1];
            CSR_ADDR_MIE:      csr_rdata = regs[2];
            CSR_ADDR_MTVEC:    csr_rdata = regs[3];
            CSR_ADDR_MSCRATCH: csr_rdata = regs[4];
            CSR_ADDR_MEPC:     csr_rdata = regs[5];
            CSR_ADDR_MCAUSE:   csr_rdata = regs[6];
            CSR_ADDR_MTVAL:    csr_rdata = regs[7];
            CSR_ADDR_MIP:       csr_rdata = regs[8];
            CSR_ADDR_MCYCLE:    csr_rdata = regs[9];
            CSR_ADDR_MINSTRET:  csr_rdata = regs[10];
            CSR_ADDR_MCYCLEH:   csr_rdata = regs[11];
            CSR_ADDR_MINSTRETH: csr_rdata = regs[12];
            default:            csr_rdata = '0;  // Unimplemented CSRs read as 0
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
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

            // Free-running 64-bit counters {hi,lo} with carry from the low
            // half. A CSR write to a half overrides that cycle's increment of
            // the pair (mcycle) or suppresses the retire increment of the
            // pair (minstret). The high half is unchanged by a low-half write
            // and vice versa: only the natural carry crosses the boundary.
            //   mcycle:  increments every cycle; write overrides + suppresses.
            //   minstret: increments on retire; a write to EITHER half
            //            suppresses that instruction's count (instret_inc).
            if (mcycle_wr)   regs[9]  <= csr_data_i;
            else             regs[9]  <= regs[9] + 1;             // MCYCLE
            if (!mcycle_wr && (regs[9] == 32'hFFFF_FFFF))
                             regs[11] <= regs[11] + 1;            // MCYCLEH carry
            if (mcycleh_wr)  regs[11] <= csr_data_i;

            if (minstret_wr) regs[10] <= csr_data_i;
            else if (instret_inc) regs[10] <= regs[10] + 1;        // MINSTRET
            if (minstret_carry) regs[12] <= regs[12] + 1;         // MINSTRETH carry
            if (minstreth_wr) regs[12] <= csr_data_i;

            // --- Trap-write bundle (one cycle, each a distinct CSR).
            // Priority over the RMW port: a trap suppresses the RMW this
            // cycle, so they never collide, but the trap port wins anyway. ---
            if (we_mstatus_i) regs[0] <= d_mstatus_i;
            if (we_mepc_i) regs[5] <= d_mepc_i;
            if (we_mcause_i) regs[6] <= d_mcause_i;
            if (we_mtval_i) regs[7] <= d_mtval_i;

            // --- mstatus RMW (only if no trap write). Only M-mode is
            // implemented, so MPP is WARL-forced to 2'b11 (M) on any write;
            // the other bits are taken raw (M-mode code is trusted). ---
            if (!we_mstatus_i && csr_wren_i && (csr_addr_i == CSR_ADDR_MSTATUS))
                regs[0] <= {csr_data_i[31:13], MSTATUS_MPP_M, csr_data_i[10:0]};

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

            // --- mcycle / mcycleh / minstret / minstreth writes are handled
            // in the counter block above (write overrides the increment /
            // carry, gated by the *_wr strobes). misa is read-only here. ---
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

`ifdef VERILATOR
    // Sim-only: mirror the reset values at time 0 so waveforms/logs do not
    // show X before the first reset edge. Matches the sync reset above.
    initial begin
        regs[0]  = MSTATUS_RESET;
        regs[1]  = MISA_RESET;
        regs[2]  = '0;
        regs[3]  = '0;
        regs[4]  = '0;
        regs[5]  = '0;
        regs[6]  = '0;
        regs[7]  = '0;
        regs[8]  = '0;
        regs[9]  = '0;
        regs[10] = '0;
        regs[11] = '0;
        regs[12] = '0;
    end
`endif

endmodule

`resetall

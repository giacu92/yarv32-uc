`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
 * CPU top: instantiates pipeline stages AND the on-die AXI4-Lite
 * bridge that turns the native mem_req_t / mem_rsp_t into AR/AW/W +
 * R/B at the single master port.
 *
 * External view of the CPU (Harvard):
 *
 *   - `axi_peri` : AXI4-Lite master carrying PERIPHERAL traffic only
 *                 (MSIP, CLINT timer, future UART/GPIO). The LSU steers
 *                 addr[PERI_ADDR_BIT]=1 accesses here through an internal
 *                 peri bridge.
 *   - `imem_req_o`/`imem_rsp_i` : native fetch port (read-only I-mem).
 *   - `dmem_req_o`/`dmem_rsp_i` : native LSU data port (byte-strobed D-mem).
 *
 * Internally:
 *
 *   fetch_stage  ──► imem_req_o/imem_rsp_i        (dedicated I-mem, no
 *                                                   contention with the LSU)
 *   LSU (execute)──► addr[PERI_ADDR_BIT] steer:
 *                     0 → dmem_req_o/dmem_rsp_i          (native D-mem)
 *                     1 → axi4_lite_master_bridge → axi_peri (peri, AXI)
 *
 * The LSU does the mem/peri decode itself (it knows PERI_ADDR_BIT), so the
 * board top is pure point-to-point wires — no crossbar. The bridge is a
 * single-outstanding FSM that turns a native req.wvalid/rsp.wready (launch)
 * + rsp.rvalid/req.rready (read) handshake into an AXI4-Lite transaction.
 *
 * No debug signals cross the CPU boundary: the per-stage pc / instr /
 * valid / writeback taps live on the stage blocks and on internal nets
 * here (consumed by the next stage or left unconnected). The simulation
 * wrapper observes them by probing the Verilator hierarchy directly
 * (--public-flat), not through CPU output ports.
 *
 * Naming: ports use *_i/_o; internal signals have no prefix (they are
 * neither inputs nor outputs).
 */

module rv32imac_zicsr_zifencei #(
    // Implemented I-mem size in address bits, forwarded to fetch so a PC
    // outside it raises an instruction access fault instead of aliasing
    // back into the image. Must match the I-mem instantiated at the top
    // level (native_ram's ADDR_W).
    parameter int IMEM_ADDR_W = 14,
    // Branch-predictor enable. 1 = prediction-at-decode active (gshare PHT +
    // direct pc+imm target + RAS); 0 = predictor disabled, decode emits no
    // predicted redirect and zeroes de_t.pred_* -> execute resolves every
    // control-flow instr with a taken redirect exactly as before the
    // predictor existed. The A/B measurement knob and a safety fallback.
    parameter int BP_EN = 1,
    // MUL structure A/B knob, forwarded to the ALU through execute. 1 = one
    // shared signed 33x33 product with the signedness selected on the
    // operands (default, fewer mux levels behind the DSP); 0 = three 32x32
    // products with a 4-way mux on their results (the historical form).
    // Functionally identical -- exists so PnR can move one variable per run.
    parameter int MUL_SHARED_DSP = 0,
    // Where the PHT direction bit is read. 0 = at decode, with the live GHR
    // (default, and what ships); 1 = at instruction-buffer PUSH time, carried
    // in the entry -- takes the PHT array read off decode's redirect path and
    // makes PHT depth timing-neutral, at the cost of a slightly stale GHR.
    // See branch_predictor.sv for the accuracy trade and rv32_pkg for sizing.
    //
    // The default is 0 to match rv32_pkg::BP_PUSH_LOOKUP, which is what every
    // real instantiation passes. It used to default to 1 here -- a form that
    // has never been through PnR -- so a standalone instance, or a new one
    // that forgot to thread the knob, got an unmeasured configuration.
    parameter int BP_PUSH_LOOKUP = 0,
    // Forwarded to fetch: whether the EXECUTE redirect launches its I-mem read
    // in the redirect cycle. 0 (default) keeps the register file off the I-mem
    // address pins at a cost of 1 cycle per mispredict/trap; 1 restores the
    // 2026-08-31 form. See fetch_stage.sv.
    parameter int EXEC_REDIR_INCYCLE = 0,
    // Forwarded to execute: whether an aligned D-mem load launches its bus
    // request live from alu_result. 0 captures every bus op, which is the
    // pre-2026-09-01 LSU -- the safe fallback. See execute_stage.sv.
    parameter int LSU_LIVE_LOAD = 1
) (
    input wire clk_i,
    input wire rstn_i,

    input  wire [XLEN-1:0] boot_addr_i,  // Reset vector boot address
    output wire            dbg_stall_o,  // decode or execute stage stall

    // AXI4-Lite master for peripherals only. Fetch and LSU data RAM use
    // the native imem_/dmem_ ports below (no AXI for memory).
    axi4_lite_if.master axi_peri,

    // Native I-mem (fetch, read-only, 64-bit — one access delivers two
    // 32-bit words; fetch keeps up to 2 reads outstanding).
    output ifetch_req_t imem_req_o,
    input  ifetch_rsp_t imem_rsp_i,

    // Native D-mem (LSU data RAM, byte-strobed).
    output mem_req_t dmem_req_o,
    input  mem_rsp_t dmem_rsp_i,

    // Machine software-interrupt pending bit from the MSIP MMIO slave
    // (on axi_peri at the board top). Drives mip.MSIP. Read-only from CSR
    // write — SW clears it via the MMIO slave, not by writing mip.
    input wire msip_i,

    // Machine timer-interrupt pending bit from the CLINT timer MMIO slave
    // (on axi_peri at the board top). Drives mip.MTIP. Read-only from CSR
    // write — SW clears it by writing mtimecmp > mtime, not by writing mip.
    input wire mtip_i,

    // Machine external-interrupt pending bit from the board-level peripheral
    // IRQ tree (currently the UART's level IRQ). Drives mip.MEIP. Read-only
    // from CSR write — SW clears it by servicing the peripheral.
    input wire meip_i
);

    // ===================================================================
    // Signal declarations
    // ===================================================================

    // -----------------------------------------------------------------
    // Fetch stage
    //
    // stall_i is driven by decode's back-pressure (propagated from the
    // execute stage's div stall). branch_* come from the execute stage's
    // branch-resolve path (redirect + in-flight flush).
    // -----------------------------------------------------------------
    // Fetch owns the I-mem port uncontended (Harvard). fe_* = fetch native
    // side -> imem_req_o/imem_rsp_i (64-bit read-only ifetch interface).
    // -------------------------------------------------------------
    ifetch_req_t                   fe_req;
    ifetch_rsp_t                   fe_rsp;
    // LSU native peri side -> the peri bridge -> axi_peri.
    mem_req_t                      peri_req;
    mem_rsp_t                      peri_rsp;

    // F/D pipeline-register taps, consumed by the decode stage below.
    wire            [    XLEN-1:0] fe_pc;
    wire            [    XLEN-1:0] fe_instr;
    wire                           fe_valid;
    wire                           fe_fault;
    // Buffer head+1 taps (same-cycle RVC spanning stitch) + the decode->fetch
    // pop-2 handshake for the stitch.
    wire            [    XLEN-1:0] fe_next_instr;
    wire            [    XLEN-1:0] fe_next_pc;
    wire                           fe_next_valid;
    wire                           fe_next_fault;
    wire                           fe_pop2;

    // Decode -> fetch back-pressure (propagates execute's stall).
    wire                           dec_stall;
    // Execute -> fetch redirect.
    wire                           ex_branch_valid;
    wire            [    XLEN-1:0] ex_branch_addr;
    // Decode -> fetch predicted redirect (branch predictor). Lower priority
    // than the execute redirect (priority-merged inside fetch).
    wire                           pred_redirect_valid;
    wire            [    XLEN-1:0] pred_redirect_addr;

    // Branch predictor lookup (decode -> predictor) + training (execute ->
    // predictor). Decode queries the PHT/RAS for the control-flow instr at the
    // buffer head; execute trains on every resolved control-flow instr. The
    // three bundles live in rv32_pkg (bp_lookup_req_t / bp_lookup_rsp_t /
    // bp_train_t), split by direction like mem_req_t / mem_rsp_t.
    bp_lookup_req_t                bp_lookup_req;
    bp_lookup_rsp_t                bp_lookup_rsp;
    bp_push_req_t                  bp_push_req;
    bp_push_rsp_t                  bp_push_rsp;
    bp_train_t                     bp_train;

    // Push-time prediction carried from fetch's buffer entry to decode.
    logic                          fe_pred_lo;
    logic                          fe_pred_hi;
    logic           [BP_GHR_W-1:0] fe_ghr;
    logic                          fe_next_pred_hi;
    logic           [BP_GHR_W-1:0] fe_next_ghr;

    // -------------------------------------------------------------
    // Register file + decode/execute
    //
    // Decode reads rs1/rs2 from the reg file asynchronously (addresses
    // come from decode, data returns the same cycle and is latched into
    // the D/E register). The D/E control word (de_bus) feeds execute, which
    // drives the reg-file write port (ALU / PC4 writeback), the fetch
    // redirect, and decode's stall (div) / flush (branch).
    // -------------------------------------------------------------
    wire            [         4:0] rs1_addr;
    wire            [         4:0] rs2_addr;
    wire            [    XLEN-1:0] rs1_data;
    wire            [    XLEN-1:0] rs2_data;

    wire            [         4:0] wb_addr;
    wire            [    XLEN-1:0] wb_data;
    wire                           wb_en;

    de_t                           de_bus;

    // de_* D/E taps (decode stage outputs). Debug only — left
    // unconnected here; the sim probes them via the Verilator hierarchy.
    wire            [    XLEN-1:0] de_pc;
    wire            [    XLEN-1:0] de_instr;
    wire                           de_valid;

    // Execute -> decode back-pressure / flush.
    wire                           ex_stall;
    wire                           ex_flush;

    // CSR
    wire            [    XLEN-1:0] csr_wdata;
    wire                           csr_we;
    wire            [    XLEN-1:0] csr_rdata;

    // CSR taps (trap unit reads mtvec / mepc / mstatus / mip / mie).
    wire [XLEN-1:0] csr_mtvec, csr_mepc, csr_mstatus, csr_mip, csr_mie;

    // Trap-write bundle (trap unit -> CSR file) + triggers/payload
    // (execute "exception logic" -> trap unit) + the pending interrupt, its
    // resolved cause, and the resolved redirect fed back to execute.
    wire we_mepc, we_mcause, we_mtval, we_mstatus;
    wire [XLEN-1:0] d_mepc, d_mcause, d_mtval, d_mstatus;
    wire sync_trap_req, mret_req, take_interrupt;
    wire [XLEN-1:0] trap_cause, trap_tval, trap_pc;
    wire            int_pending;
    wire [XLEN-1:0] int_cause;
    wire [XLEN-1:0] trap_redirect_addr;

    // ex_* E/M taps (retired op pc / instr / valid). Debug only — left
    // unconnected here; the sim probes them via the Verilator hierarchy.
    wire [XLEN-1:0] ex_pc;
    wire [XLEN-1:0] ex_instr;
    wire            ex_valid;
    wire            ex_retire;  // combinational retire -> minstret counter

    // ===================================================================
    // Module instantiations
    // ===================================================================

    // -------------------------------------------------------------
    // Fetch stage
    //
    // stall_i is driven by decode's back-pressure (propagated from the
    // execute stage's div stall). branch_* come from the execute stage's
    // branch-resolve path (redirect + in-flight flush).
    // -------------------------------------------------------------
    fetch_stage #(
        .IMEM_ADDR_W       (IMEM_ADDR_W),
        .EXEC_REDIR_INCYCLE(EXEC_REDIR_INCYCLE)
    ) fetch_stage_i (
        .clk_i            (clk_i),
        .rstn_i           (rstn_i),
        .boot_addr_i      (boot_addr_i),
        .stall_i          (dec_stall),
        .branch_valid_i   (ex_branch_valid),
        .branch_addr_i    (ex_branch_addr),
        .pred_valid_i     (pred_redirect_valid),
        .pred_addr_i      (pred_redirect_addr),
        .imem_req_o       (fe_req),
        .imem_rsp_i       (fe_rsp),
        .bp_push_o        (bp_push_req),
        .bp_push_i        (bp_push_rsp),
        .fe_instr_o       (fe_instr),
        .fe_pc_o          (fe_pc),
        .fe_valid_o       (fe_valid),
        .fe_fault_o       (fe_fault),
        .fe_pred_lo_o     (fe_pred_lo),
        .fe_pred_hi_o     (fe_pred_hi),
        .fe_ghr_o         (fe_ghr),
        .fe_next_instr_o  (fe_next_instr),
        .fe_next_pc_o     (fe_next_pc),
        .fe_next_valid_o  (fe_next_valid),
        .fe_next_fault_o  (fe_next_fault),
        .fe_next_pred_hi_o(fe_next_pred_hi),
        .fe_next_ghr_o    (fe_next_ghr),
        .fe_pop2_i        (fe_pop2)
    );

    // -------------------------------------------------------------
    // On-die AXI4-Lite bridge (single master port)
    //
    // Translates the native mem_req_t / mem_rsp_t into AXI4-Lite
    // AR/AW/W + R/B. The pipeline only ever deals with one-cycle
    // request/response; the bridge owns the bus protocol. Peripheral
    // addresses flow through this bridge to the board top's peri bus.
    // -------------------------------------------------------------
    axi4_lite_master_bridge u_bus_bridge (
        .clk_i (clk_i),
        .rstn_i(rstn_i),
        .req_i (peri_req),
        .rsp_o (peri_rsp),
        .axi   (axi_peri)
    );

    // GPR Regfile
    reg_file u_regfile (
        .clk_i     (clk_i),
        .rstn_i    (rstn_i),
        .rs1_addr_i(rs1_addr),
        .rs2_addr_i(rs2_addr),
        .rs1_data_o(rs1_data),
        .rs2_data_o(rs2_data),
        .wr_addr_i (wb_addr),
        .wr_data_i (wb_data),
        .wr_en_i   (wb_en)
    );

    // CSR file: decode drives the address (de_bus.csr_addr = the op in
    // execute); the async read returns the OLD value to execute, the sync
    // write commits the RMW result from execute (csr_wdata / csr_we).
    // Both ports address the op currently in execute (single CSR op at a
    // time).
    csr_regfile u_csr_regfile (
        .clk_i         (clk_i),
        .rstn_i        (rstn_i),
        .csr_addr_i    (de_bus.csr_addr),
        .csr_data_o    (csr_rdata),
        .csr_wren_i    (csr_we),
        .csr_data_i    (csr_wdata),
        .we_mepc_i     (we_mepc),
        .d_mepc_i      (d_mepc),
        .we_mcause_i   (we_mcause),
        .d_mcause_i    (d_mcause),
        .we_mtval_i    (we_mtval),
        .d_mtval_i     (d_mtval),
        .we_mstatus_i  (we_mstatus),
        .d_mstatus_i   (d_mstatus),
        .msip_i        (msip_i),
        .mtip_i        (mtip_i),
        .meip_i        (meip_i),
        .mtvec_o       (csr_mtvec),
        .mepc_o        (csr_mepc),
        .mstatus_o     (csr_mstatus),
        .mip_o         (csr_mip),
        .mie_o         (csr_mie),
        .instr_retire_i(ex_retire)
    );

    // Trap unit (combinational peer of the execute stage): consumes the
    // triggers + cause/tval/pc the execute "exception logic" produces,
    // drives the CSR trap-write bundle (mepc / mcause / mtval / mstatus) and
    // the fetch redirect, and reports the pending+enabled interrupt and its
    // resolved cause back to execute. int_pending / int_cause depend only on
    // the CSR taps (no loop through the triggers).
    trap_unit u_trap (
        .mtvec_i         (csr_mtvec),
        .mepc_i          (csr_mepc),
        .mstatus_i       (csr_mstatus),
        .mip_i           (csr_mip),
        .mie_i           (csr_mie),
        .take_trap_i     (sync_trap_req),
        .take_interrupt_i(take_interrupt),
        .mret_i          (mret_req),
        .cause_i         (trap_cause),
        .tval_i          (trap_tval),
        .pc_i            (trap_pc),
        .redirect_valid_o(),
        .redirect_addr_o (trap_redirect_addr),
        .csr_we_mepc_o   (we_mepc),
        .csr_d_mepc_o    (d_mepc),
        .csr_we_mcause_o (we_mcause),
        .csr_d_mcause_o  (d_mcause),
        .csr_we_mtval_o  (we_mtval),
        .csr_d_mtval_o   (d_mtval),
        .csr_we_mstatus_o(we_mstatus),
        .csr_d_mstatus_o (d_mstatus),
        .int_pending_o   (int_pending),
        .int_cause_o     (int_cause)
    );

    decode_stage #(
        .BP_EN         (BP_EN),
        .BP_PUSH_LOOKUP(BP_PUSH_LOOKUP)
    ) u_decode (
        .clk_i                (clk_i),
        .rstn_i               (rstn_i),
        .fe_instr_i           (fe_instr),
        .fe_pc_i              (fe_pc),
        .fe_valid_i           (fe_valid),
        .fe_fault_i           (fe_fault),
        .fe_pred_lo_i         (fe_pred_lo),
        .fe_pred_hi_i         (fe_pred_hi),
        .fe_ghr_i             (fe_ghr),
        .fe_next_instr_i      (fe_next_instr),
        .fe_next_pc_i         (fe_next_pc),
        .fe_next_valid_i      (fe_next_valid),
        .fe_next_fault_i      (fe_next_fault),
        .fe_next_pred_hi_i    (fe_next_pred_hi),
        .fe_next_ghr_i        (fe_next_ghr),
        .rs1_addr_o           (rs1_addr),
        .rs2_addr_o           (rs2_addr),
        .rs1_data_i           (rs1_data),
        .rs2_data_i           (rs2_data),
        .stall_i              (ex_stall),
        .flush_i              (ex_flush),
        .ex_wb_en_i           (wb_en),
        .ex_wb_addr_i         (wb_addr),
        .ex_wb_data_i         (wb_data),
        .stall_o              (dec_stall),
        .fe_pop2_o            (fe_pop2),
        .bp_lookup_o          (bp_lookup_req),
        .bp_lookup_i          (bp_lookup_rsp),
        .pred_redirect_valid_o(pred_redirect_valid),
        .pred_redirect_addr_o (pred_redirect_addr),
        .de_o                 (de_bus),
        .de_pc_o              (de_pc),
        .de_instr_o           (de_instr),
        .de_valid_o           (de_valid)
    );

    execute_stage #(
        .MUL_SHARED_DSP(MUL_SHARED_DSP),
        .LSU_LIVE_LOAD (LSU_LIVE_LOAD)
    ) u_execute (
        .clk_i               (clk_i),
        .rstn_i              (rstn_i),
        .de_i                (de_bus),
        .csr_rdata_i         (csr_rdata),           // CSR read value
        .csr_wdata_o         (csr_wdata),           // CSR write value (RMW result)
        .csr_wren_o          (csr_we),              // CSR write enable (execute-qualified)
        .stall_i             (1'b0),                // no downstream stage yet
        .stall_o             (ex_stall),
        .flush_o             (ex_flush),
        .wb_addr_o           (wb_addr),
        .wb_data_o           (wb_data),
        .wb_en_o             (wb_en),
        .branch_valid_o      (ex_branch_valid),
        .branch_addr_o       (ex_branch_addr),
        .bp_train_o          (bp_train),
        .mem_req_o           (dmem_req_o),
        .mem_rsp_i           (dmem_rsp_i),
        .peri_req_o          (peri_req),
        .peri_rsp_i          (peri_rsp),
        // Trap machinery: execute "exception logic" emits the triggers +
        // cause/tval/pc, imports the pending interrupt + its resolved cause
        // + the resolved redirect (from the trap unit). int_pending /
        // int_cause are combinational off the CSR taps (no loop through the
        // triggers); redirect_addr selects trap / mret / interrupt target
        // over the normal branch.
        .int_pending_i       (int_pending),
        .int_cause_i         (int_cause),
        .trap_redirect_addr_i(trap_redirect_addr),
        .sync_trap_req_o     (sync_trap_req),
        .mret_req_o          (mret_req),
        .take_interrupt_o    (take_interrupt),
        .trap_cause_o        (trap_cause),
        .trap_tval_o         (trap_tval),
        .trap_pc_o           (trap_pc),
        .ex_pc_o             (ex_pc),
        .ex_instr_o          (ex_instr),
        .ex_valid_o          (ex_valid),
        .retire_o            (ex_retire)
    );

    // Branch predictor (gshare PHT + GHR + RAS). Decode queries it for the
    // control-flow instr at the buffer head (combinational, PC+GHR only — off
    // the regfile->forward->compare critical path); execute trains it on every
    // resolved control-flow instr. Training at resolve, not at predict, means
    // squashed wrong-path instructions never train it (in-order single-issue).
    // When BP_EN=0 the decode side still queries (harmless) but emits no
    // predicted redirect and zeroes de_t.pred_*, so execute's mispredict logic
    // reduces to the legacy taken-redirect.
    branch_predictor u_bp (
        .clk_i       (clk_i),
        .rstn_i      (rstn_i),
        .lookup_req_i(bp_lookup_req),
        .lookup_rsp_o(bp_lookup_rsp),
        .push_req_i  (bp_push_req),
        .push_rsp_o  (bp_push_rsp),
        .train_i     (bp_train)
    );

    // ===================================================================
    // Assignments
    // ===================================================================

    // Aggregate stall tap for the board / sim (functional only).
    assign dbg_stall_o = dec_stall | ex_stall;

    assign imem_req_o  = fe_req;
    assign fe_rsp      = imem_rsp_i;

endmodule

`resetall

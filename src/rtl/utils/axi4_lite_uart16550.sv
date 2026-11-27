`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* 16550-compatible UART (AXI4-Lite slave), for the Linux branch (M8).
*
* Register-compatible with what the mainline 8250 driver expects for
* `compatible = "ns16550a"` with `reg-shift = <2>` and `reg-io-width = <4>`:
* register i lives at byte offset 4*i, data in bits [7:0] of a 32-bit
* access. Linux needs no driver of ours (drivers/tty/serial/8250/8250_of.c
* + 8250_port.c), OpenSBI's `uart8250` driver and the `uart8250,mmio32`
* earlycon work as-is.
*
*   off  DLAB=0 read   DLAB=0 write   DLAB=1
*   0x00 RBR (pops)    THR (pushes)   DLL
*   0x04 IER           IER            DLM
*   0x08 IIR           FCR            (IIR / FCR)
*   0x0C LCR           LCR
*   0x10 MCR           MCR
*   0x14 LSR           (ignored)
*   0x18 MSR           (ignored)
*   0x1C SCR           SCR
*
* BAUD. Standard 16550 semantics: one bit time = 16 x DL clock cycles, where
* DL = {DLM, DLL}. So the device tree carries the REAL input clock:
* `clock-frequency = <50000000>` at 50 MHz, and Linux computes
* DL = clock-frequency / (16 * baud) itself (27 at 115200, an actual rate of
* 115 741 baud, +0.47%). DL resets to CLK_FREQ_HZ / (16 * BAUD_RATE), so bare
* firmware gets BAUD_RATE without touching the divisor. DL = 0 is treated as
* 1. A divisor write takes effect when both engines are idle (as in the MCU
* UART), so a frame in flight is never cut by a rate change.
*
* What is modelled and what is not:
*   - 8N1 only. LCR is stored and reads back, but word length / parity /
*     stop bits are not applied: the engines always run 8N1. Linux's 8250
*     driver programs 8N1 by default for a console.
*   - FIFOs are always active, 16 bytes each way. FCR[0] only changes what
*     IIR[7:6] reports (11 = FIFOs enabled, as a 16550A). FCR[1]/[2] reset the
*     RX / TX FIFO; FCR[7:6] select the RX trigger level 1/4/8/14.
*   - LSR: DR, OE (sticky, cleared by an LSR read), FE and BI (stop bit
*     sampled low; BI when the data byte is also 0; cleared by an LSR read),
*     THRE (TX FIFO empty), TEMT (TX FIFO and shifter empty). PE is 0 (no
*     parity). LSR[7] = FE|BI (no per-character error tagging).
*   - IIR priority, as the 16550: line status (0x6) > RX data at trigger
*     (0x4) > character timeout (0xC) > THR empty (0x2) > modem status (0x0);
*     0x1 = none. THRE is an EDGE-type source as on the real part: raised when
*     the TX FIFO drains (or IER.ETBEI is enabled while it is empty), cleared
*     by a THR write or by an IIR read that reported it -- the 8250 driver's
*     THRE test depends on exactly this.
*   - Character timeout: RX FIFO non-empty, and no character received or read
*     for 4 character times (40 bit times).
*   - MCR / MSR: MCR[4:0] stored. MCR.LOOP routes TX to RX internally and
*     holds txd_o high; in loop MSR reflects RTS/DTR/OUT1/OUT2 as CTS/DSR/RI/
*     DCD. Out of loop MSR reports CTS=DSR=DCD=1, RI=0, no deltas -- there are
*     no modem pins on this board.
*   - A THR write while the TX FIFO is full is HELD (awready low), not
*     dropped, exactly as the MCU UART does.
*
* uart_irq_o = an interrupt is identified (IIR[0] == 0).
* rxd_i must be pre-synchronized by the caller (the tops double-flop it).
*
* Reset is synchronous, active-low. Naming: ports *_i/_o; flops _q.
*/

module axi4_lite_uart16550 #(
    parameter int unsigned CLK_FREQ_HZ = 50_000_000,
    parameter int unsigned BAUD_RATE   = 115200
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    output wire txd_o,
    input  wire rxd_i,
    output wire uart_irq_o
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;
    localparam int FIFO_DEPTH = 16;
    localparam int PTR_W = 4;
    localparam int unsigned DL_CALC = CLK_FREQ_HZ / (16 * BAUD_RATE);
    localparam logic [15:0] DL_RESET = (DL_CALC == 0) ? 16'd1 : 16'(DL_CALC);

    localparam logic [2:0] R_RBR = 3'd0;
    localparam logic [2:0] R_IER = 3'd1;
    localparam logic [2:0] R_IIR = 3'd2;
    localparam logic [2:0] R_LCR = 3'd3;
    localparam logic [2:0] R_MCR = 3'd4;
    localparam logic [2:0] R_LSR = 3'd5;
    localparam logic [2:0] R_MSR = 3'd6;
    localparam logic [2:0] R_SCR = 3'd7;

    // ---------------------------------------------------------------
    // Registers
    // ---------------------------------------------------------------
    logic [15:0] dl_q;  // live divisor
    logic [15:0] dl_pending_q;
    logic        dl_pending_valid_q;
    logic [ 3:0] ier_q;
    logic [ 7:0] lcr_q;
    logic [ 4:0] mcr_q;
    logic [ 7:0] scr_q;
    logic        fcr_en_q;
    logic [ 1:0] fcr_trig_q;
    wire         dlab = lcr_q[7];
    wire         loop = mcr_q[4];

    // Bit time in clocks, minus one: 16 * DL - 1.
    wire  [19:0] bit_div = {(dl_q == 16'd0) ? 16'd1 : dl_q, 4'b0000} - 20'd1;

    // ---------------------------------------------------------------
    // FIFOs
    // ---------------------------------------------------------------
    logic [ 7:0] tx_fifo_q                                                   [FIFO_DEPTH];
    logic [ 7:0] rx_fifo_q                                                   [FIFO_DEPTH];
    logic [PTR_W:0] tx_wptr_q, tx_rptr_q, rx_wptr_q, rx_rptr_q;
    wire [PTR_W:0] tx_level = tx_wptr_q - tx_rptr_q;
    wire [PTR_W:0] rx_level = rx_wptr_q - rx_rptr_q;
    wire tx_fifo_empty = (tx_level == '0);
    wire tx_fifo_full = tx_level[PTR_W];
    wire rx_fifo_empty = (rx_level == '0);
    wire rx_fifo_full = rx_level[PTR_W];
    wire [7:0] tx_fifo_head = tx_fifo_q[tx_rptr_q[PTR_W-1:0]];
    wire [7:0] rx_fifo_head = rx_fifo_q[rx_rptr_q[PTR_W-1:0]];

    // ---------------------------------------------------------------
    // TX engine
    // ---------------------------------------------------------------
    typedef enum logic [1:0] {
        TX_IDLE,
        TX_START,
        TX_DATA,
        TX_STOP
    } tx_state_e;
    tx_state_e        tx_state_q;
    logic      [ 7:0] tx_shift_q;
    logic      [ 2:0] tx_bit_cnt_q;
    logic             txd_q;
    logic      [19:0] tx_cnt_q;
    wire              tx_idle = (tx_state_q == TX_IDLE);
    wire              tx_tick = (tx_cnt_q == bit_div);
    wire              tx_fifo_pop = tx_idle && !tx_fifo_empty;
    assign txd_o = loop ? 1'b1 : txd_q;

    // ---------------------------------------------------------------
    // RX engine
    // ---------------------------------------------------------------
    typedef enum logic [1:0] {
        RX_IDLE,
        RX_START,
        RX_DATA,
        RX_STOP
    } rx_state_e;
    rx_state_e              rx_state_q;
    logic      [       7:0] rx_shift_q;
    logic      [       2:0] rx_bit_cnt_q;
    logic      [      19:0] rx_cnt_q;
    wire                    rx_idle = (rx_state_q == RX_IDLE);
    wire                    rxd = loop ? txd_q : rxd_i;
    wire       [      19:0] rx_half = {1'b0, bit_div[19:1]};
    wire                    rx_frame_done = (rx_state_q == RX_STOP) && (rx_cnt_q == bit_div);
    wire                    rx_stop_bad = rx_frame_done && !rxd;

    // ---------------------------------------------------------------
    // AXI write side (single outstanding, BVALID held), as the MCU UART
    // ---------------------------------------------------------------
    logic                   aw_seen_q;
    logic                   w_seen_q;
    logic      [DATA_W-1:0] wdata_q;
    logic      [STRB_W-1:0] wstrb_q;
    logic      [       2:0] awaddr_word_q;
    logic                   bvalid_q;

    wire       [       2:0] awaddr_in = axi.awaddr[4:2];
    // Hold a THR write while the TX FIFO is full (DLAB=0 only: with DLAB=1
    // offset 0 is DLL, not the FIFO).
    wire                    tx_full_block_aw = (awaddr_in == R_RBR) && !dlab && tx_fifo_full;
    assign axi.awready = !aw_seen_q && !bvalid_q && !tx_full_block_aw;
    assign axi.wready  = !w_seen_q && !bvalid_q;

    wire              aw_hs = axi.awvalid && axi.awready;
    wire              w_hs = axi.wvalid && axi.wready;
    wire              aw_present = aw_seen_q || aw_hs;
    wire              w_present = w_seen_q || w_hs;
    wire [       2:0] waddr_eff = aw_hs ? axi.awaddr[4:2] : awaddr_word_q;
    // Exposed name kept from the MCU UART: sim_top logs every TX byte from
    // u_uart.tx_push / u_uart.wdata_eff.
    wire [DATA_W-1:0] wdata_eff = w_hs ? axi.wdata : wdata_q;
    wire [STRB_W-1:0] wstrb_eff = w_hs ? axi.wstrb : wstrb_q;
    wire              wr_b0 = wstrb_eff[0];
    wire [       7:0] wbyte = wdata_eff[7:0];

    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;
    wire b_hs = axi.bvalid && axi.bready;

    wire thr_sel = (waddr_eff == R_RBR) && !dlab;
    wire do_write = aw_present && w_present && !bvalid_q && !(thr_sel && tx_fifo_full);
    wire wr = do_write && wr_b0;
    wire tx_push = wr && thr_sel;
    wire fcr_write = wr && (waddr_eff == R_IIR);
    wire tx_fifo_reset = fcr_write && wbyte[2];
    wire rx_fifo_reset = fcr_write && wbyte[1];
    wire ier_write = wr && (waddr_eff == R_IER) && !dlab;

    // ---------------------------------------------------------------
    // AXI read side
    // ---------------------------------------------------------------
    logic rvalid_q;
    logic [DATA_W-1:0] rdata_q;
    assign axi.arready = !rvalid_q || axi.rready;
    wire ar_hs = axi.arvalid && axi.arready;
    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;
    wire [2:0] raddr = axi.araddr[4:2];

    wire rbr_read = ar_hs && (raddr == R_RBR) && !dlab;
    wire rx_fifo_pop = rbr_read && !rx_fifo_empty;
    wire lsr_read = ar_hs && (raddr == R_LSR);
    wire iir_read = ar_hs && (raddr == R_IIR);

    // ---------------------------------------------------------------
    // Line status + interrupt sources
    // ---------------------------------------------------------------
    logic oe_q, fe_q, bi_q;  // sticky, cleared by an LSR read
    logic thre_ip_q;  // THR-empty interrupt pending (edge-type)
    logic tx_empty_d1_q;  // tx_fifo_empty one cycle ago, for the edge
    logic [5:0] to_cnt_q;  // bit times since the last RX activity
    logic [19:0] to_div_q;  // clocks within the current bit time
    wire to_hit = (to_cnt_q >= 6'd40);

    logic [4:0] rx_trig;
    always_comb begin
        unique case (fcr_trig_q)
            2'd0: rx_trig = 5'd1;
            2'd1: rx_trig = 5'd4;
            2'd2: rx_trig = 5'd8;
            default: rx_trig = 5'd14;
        endcase
    end

    wire ls_ip = ier_q[2] && (oe_q || fe_q || bi_q);
    wire rda_ip = ier_q[0] && ({1'b0, rx_level} >= {1'b0, rx_trig});
    wire cto_ip = ier_q[0] && !rx_fifo_empty && to_hit;
    wire thr_ip = ier_q[1] && thre_ip_q;

    logic [3:0] iir_id;
    always_comb begin
        if (ls_ip) iir_id = 4'h6;
        else if (rda_ip) iir_id = 4'h4;
        else if (cto_ip) iir_id = 4'hC;
        else if (thr_ip) iir_id = 4'h2;
        else iir_id = 4'h1;  // none pending
    end
    wire [7:0] iir = {fcr_en_q ? 2'b11 : 2'b00, 2'b00, iir_id};
    assign uart_irq_o = !iir_id[0];

    wire        temt = tx_fifo_empty && tx_idle;
    wire  [7:0] lsr = {fe_q | bi_q, temt, tx_fifo_empty, bi_q, fe_q, 1'b0, oe_q, !rx_fifo_empty};
    wire  [7:0] msr = loop ? {mcr_q[3], mcr_q[2], mcr_q[0], mcr_q[1], 4'b0000} : 8'hB0;

    logic [7:0] rdata_byte;
    always_comb begin
        unique case (raddr)
            R_RBR:   rdata_byte = dlab ? dl_q[7:0] : rx_fifo_head;
            R_IER:   rdata_byte = dlab ? dl_q[15:8] : {4'b0, ier_q};
            R_IIR:   rdata_byte = iir;
            R_LCR:   rdata_byte = lcr_q;
            R_MCR:   rdata_byte = {3'b0, mcr_q};
            R_LSR:   rdata_byte = lsr;
            R_MSR:   rdata_byte = msr;
            default: rdata_byte = scr_q;  // R_SCR
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q <= 1'b0;
            rdata_q  <= '0;
        end else if (ar_hs) begin
            rvalid_q <= 1'b1;
            rdata_q  <= {{(DATA_W - 8) {1'b0}}, rdata_byte};
        end else if (rvalid_q && axi.rready) begin
            rvalid_q <= 1'b0;
        end
    end

    // ---------------------------------------------------------------
    // Register writes
    // ---------------------------------------------------------------
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q          <= 1'b0;
            w_seen_q           <= 1'b0;
            wdata_q            <= '0;
            wstrb_q            <= '0;
            awaddr_word_q      <= '0;
            bvalid_q           <= 1'b0;
            ier_q              <= '0;
            lcr_q              <= 8'h03;  // 8N1, DLAB=0
            mcr_q              <= '0;
            scr_q              <= '0;
            fcr_en_q           <= 1'b0;
            fcr_trig_q         <= 2'd0;
            dl_pending_q       <= '0;
            dl_pending_valid_q <= 1'b0;
        end else begin
            if (aw_hs) begin
                aw_seen_q     <= 1'b1;
                awaddr_word_q <= axi.awaddr[4:2];
            end
            if (w_hs) begin
                w_seen_q <= 1'b1;
                wdata_q  <= axi.wdata;
                wstrb_q  <= axi.wstrb;
            end
            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;
            end
            if (wr) begin
                unique case (waddr_eff)
                    R_RBR:
                    if (dlab) begin
                        dl_pending_q <= {
                            dl_pending_valid_q ? dl_pending_q[15:8] : dl_q[15:8], wbyte
                        };
                        dl_pending_valid_q <= 1'b1;
                    end
                    R_IER:
                    if (dlab) begin
                        dl_pending_q <= {wbyte, dl_pending_valid_q ? dl_pending_q[7:0] : dl_q[7:0]};
                        dl_pending_valid_q <= 1'b1;
                    end else ier_q <= wbyte[3:0];
                    R_IIR: begin  // FCR
                        fcr_en_q   <= wbyte[0];
                        fcr_trig_q <= wbyte[7:6];
                    end
                    R_LCR:   lcr_q <= wbyte;
                    R_MCR:   mcr_q <= wbyte[4:0];
                    R_SCR:   scr_q <= wbyte;
                    default: ;  // LSR / MSR read-only
                endcase
            end
            if (b_hs) bvalid_q <= 1'b0;
            if (dl_pending_valid_q && tx_idle && rx_idle) dl_pending_valid_q <= 1'b0;
        end
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) dl_q <= DL_RESET;
        else if (dl_pending_valid_q && tx_idle && rx_idle) dl_q <= dl_pending_q;
    end

    // ---------------------------------------------------------------
    // FIFO storage + pointers
    // ---------------------------------------------------------------
    wire rx_fifo_push = rx_frame_done && (!rx_fifo_full || rx_fifo_pop);

    always_ff @(posedge clk_i) begin
        if (tx_push) tx_fifo_q[tx_wptr_q[PTR_W-1:0]] <= wbyte;
        if (rx_fifo_push) rx_fifo_q[rx_wptr_q[PTR_W-1:0]] <= rx_shift_q;
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            tx_wptr_q <= '0;
            tx_rptr_q <= '0;
            rx_wptr_q <= '0;
            rx_rptr_q <= '0;
        end else begin
            if (tx_fifo_reset) begin
                tx_wptr_q <= '0;
                tx_rptr_q <= '0;
            end else begin
                if (tx_push) tx_wptr_q <= tx_wptr_q + 1'b1;
                if (tx_fifo_pop) tx_rptr_q <= tx_rptr_q + 1'b1;
            end
            if (rx_fifo_reset) begin
                rx_wptr_q <= '0;
                rx_rptr_q <= '0;
            end else begin
                if (rx_fifo_push) rx_wptr_q <= rx_wptr_q + 1'b1;
                if (rx_fifo_pop) rx_rptr_q <= rx_rptr_q + 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------
    // Line status, THRE edge, character timeout
    // ---------------------------------------------------------------
    // THRE is raised on the TX FIFO becoming empty, or on ETBEI being
    // enabled while it is empty; cleared by a THR write or by an IIR read
    // that reported THRE (the source the IIR shows is the one it clears).
    wire tx_became_empty = tx_fifo_empty && !tx_empty_d1_q;
    wire etbei_rise = ier_write && wbyte[1] && !ier_q[1];
    wire iir_reports_thre = iir_read && (iir_id == 4'h2);

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            oe_q          <= 1'b0;
            fe_q          <= 1'b0;
            bi_q          <= 1'b0;
            thre_ip_q     <= 1'b0;
            tx_empty_d1_q <= 1'b1;
            to_cnt_q      <= '0;
            to_div_q      <= '0;
        end else begin
            tx_empty_d1_q <= tx_fifo_empty;

            if (lsr_read) begin
                oe_q <= 1'b0;
                fe_q <= 1'b0;
                bi_q <= 1'b0;
            end
            if (rx_frame_done && rx_fifo_full && !rx_fifo_pop) oe_q <= 1'b1;
            if (rx_stop_bad) begin
                fe_q <= 1'b1;
                if (rx_shift_q == 8'h00) bi_q <= 1'b1;
            end

            if (tx_push || iir_reports_thre) thre_ip_q <= 1'b0;
            else if (tx_became_empty || (etbei_rise && tx_fifo_empty)) thre_ip_q <= 1'b1;

            // Timeout clock: counts bit times while characters sit in the
            // RX FIFO, restarted by any push or pop.
            if (rx_fifo_push || rx_fifo_pop || rx_fifo_empty || rx_fifo_reset) begin
                to_cnt_q <= '0;
                to_div_q <= '0;
            end else if (to_div_q == bit_div) begin
                to_div_q <= '0;
                if (!to_hit) to_cnt_q <= to_cnt_q + 6'd1;
            end else begin
                to_div_q <= to_div_q + 20'd1;
            end
        end
    end

    // ---------------------------------------------------------------
    // TX engine
    // ---------------------------------------------------------------
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            tx_state_q   <= TX_IDLE;
            tx_shift_q   <= '0;
            tx_bit_cnt_q <= '0;
            txd_q        <= 1'b1;
            tx_cnt_q     <= '0;
        end else begin
            unique case (tx_state_q)
                TX_IDLE: begin
                    tx_cnt_q <= '0;
                    if (tx_fifo_pop) begin
                        tx_shift_q <= tx_fifo_head;
                        tx_state_q <= TX_START;
                        txd_q      <= 1'b0;
                    end
                end
                TX_START:
                if (tx_tick) begin
                    tx_cnt_q     <= '0;
                    tx_state_q   <= TX_DATA;
                    tx_bit_cnt_q <= '0;
                    txd_q        <= tx_shift_q[0];
                end else tx_cnt_q <= tx_cnt_q + 20'd1;
                TX_DATA:
                if (tx_tick) begin
                    tx_cnt_q <= '0;
                    if (tx_bit_cnt_q == 3'd7) begin
                        tx_state_q <= TX_STOP;
                        txd_q      <= 1'b1;
                    end else begin
                        tx_bit_cnt_q <= tx_bit_cnt_q + 3'd1;
                        tx_shift_q   <= tx_shift_q >> 1;
                        txd_q        <= tx_shift_q[1];
                    end
                end else tx_cnt_q <= tx_cnt_q + 20'd1;
                TX_STOP:
                if (tx_tick) begin
                    tx_cnt_q   <= '0;
                    tx_state_q <= TX_IDLE;
                end else tx_cnt_q <= tx_cnt_q + 20'd1;
                default: tx_state_q <= TX_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // RX engine: start edge, re-check at mid start bit, then one sample per
    // bit time at mid-bit; the stop bit is sampled at its middle too, which
    // is where FE/BI come from.
    // ---------------------------------------------------------------
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rx_state_q   <= RX_IDLE;
            rx_shift_q   <= '0;
            rx_bit_cnt_q <= '0;
            rx_cnt_q     <= '0;
        end else begin
            unique case (rx_state_q)
                RX_IDLE: begin
                    rx_cnt_q <= '0;
                    if (!rxd) rx_state_q <= RX_START;
                end
                RX_START:
                if (rx_cnt_q == rx_half) begin
                    rx_cnt_q <= '0;
                    if (!rxd) begin
                        rx_state_q   <= RX_DATA;
                        rx_bit_cnt_q <= '0;
                    end else rx_state_q <= RX_IDLE;  // glitch
                end else rx_cnt_q <= rx_cnt_q + 20'd1;
                RX_DATA:
                if (rx_cnt_q == bit_div) begin
                    rx_cnt_q   <= '0;
                    rx_shift_q <= {rxd, rx_shift_q[7:1]};
                    if (rx_bit_cnt_q == 3'd7) rx_state_q <= RX_STOP;
                    else rx_bit_cnt_q <= rx_bit_cnt_q + 3'd1;
                end else rx_cnt_q <= rx_cnt_q + 20'd1;
                RX_STOP:
                if (rx_cnt_q == bit_div) begin
                    rx_cnt_q   <= '0;
                    rx_state_q <= RX_IDLE;
                end else rx_cnt_q <= rx_cnt_q + 20'd1;
                default: rx_state_q <= RX_IDLE;
            endcase
        end
    end

endmodule

`resetall

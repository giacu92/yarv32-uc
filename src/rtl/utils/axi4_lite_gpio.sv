`resetall
`timescale 1ns / 1ps
`default_nettype none

import rv32_pkg::*;

/**
* GPIO peripheral (AXI4-Lite slave), WIDTH push-pull pins.
*
* Register map (word-addressed, byte offsets from GPIO_BASE; the LSU/bridge
* only ever issues single-beat, word-aligned accesses):
*
*   0x00 VALUE    (R)  : bits [WIDTH-1:0] = gpio_i (the synchronized pin
*                        inputs -- see the sync note below).
*   0x04 OUT      (RW) : bits [WIDTH-1:0] output data -> gpio_o.
*   0x08 DIR      (RW) : bits [WIDTH-1:0] direction, 1 = output ->
*                        gpio_oe_o. Reset 0: every pad released, so a
*                        freshly booted board never drives an external line.
*   0x0C INT_TYPE (RW) : 2 bits per pin, at [2p+1:2p]:
*                          00 = level-high, 01 = rise, 10 = fall, 11 = both.
*                        Reset 00 (level).
*   0x10 INT_EN   (RW) : bits [WIDTH-1:0] interrupt enable. Reset 0.
*   0x14 INT_STATUS (R/W1C) : bits [WIDTH-1:0] pending. Write 1 to clear.
*
* Interrupt model: INT_STATUS is a per-pin sticky event record, set
* REGARDLESS of INT_EN (PLIC-style: pending records the event, the enable
* only gates the IRQ output). So an edge arriving while INT_EN=0 latches and
* raises the IRQ the moment the pin is enabled -- the software-visible
* behaviour of a real PLIC, and what lets a test poll the status without
* arming interrupts.
*
*   gpio_irq_o = |(INT_STATUS & INT_EN)  -- level while pending and enabled.
*
* W1C on a pin whose condition still holds does not stick (set wins over
* clear, the axi4_lite_i2c REG_IRQ rule): a level-high pin re-pends the next
* cycle, and an edge landing in the clear cycle is not lost. Edge bits are
* one-shot -- clearing an edge pin sticks, because the event is the edge,
* not the level.
*
* CONSEQUENCE OF THE RESET VALUES: INT_TYPE resets to level and pulled-up
* pads read 1, so INT_STATUS reads all-1s out of reset. Harmless (INT_EN
* resets 0, so the IRQ stays masked), and software that uses edges never
* sees it: INT_TYPE is write-only configuration, not state.
*
* gpio_i is NOT synchronized by this module: the caller must hand it
* already-synchronized pin inputs (top_module double-flops each pad, same
* as uart_rxd_i / the I2C lines). Edge detect runs off the previous-cycle
* value of the synchronized input, so a metastable sample would fire a
* phantom interrupt -- there is no flag to catch that here.
*
* Protocol follows msip_peri.sv / axi4_lite_uart.sv: registered BVALID
* held until the B handshake, RVALID held until RREADY, single-outstanding
* (AWREADY/WREADY/ARREADY low while a response is pending).
*
* Reset is synchronous, active-low.
*
* Naming: ports *_i/_o; internals no prefix; flops _q, next-state _d.
*/

module axi4_lite_gpio #(
    parameter int unsigned WIDTH = 4
) (
    input wire clk_i,
    input wire rstn_i,

    axi4_lite_if.slave axi,

    // Pad side. gpio_i = synchronized inputs; the top owns the tri-state
    // glue (gpio_oe_o = 1 drives gpio_o onto the pad).
    input  wire [WIDTH-1:0] gpio_i,
    output wire [WIDTH-1:0] gpio_o,
    output wire [WIDTH-1:0] gpio_oe_o,

    // Level-sensitive interrupt (see header comment).
    output wire gpio_irq_o
);

    localparam int DATA_W = axi.DATA_WIDTH;
    localparam int STRB_W = DATA_W / 8;

    // Word offsets (byte addr[4:2], word-aligned).
    localparam logic [2:0] REG_VALUE = 3'd0;
    localparam logic [2:0] REG_OUT = 3'd1;
    localparam logic [2:0] REG_DIR = 3'd2;
    localparam logic [2:0] REG_INT_TYPE = 3'd3;
    localparam logic [2:0] REG_INT_EN = 3'd4;
    localparam logic [2:0] REG_INT_STATUS = 3'd5;

    // =================================================================
    // Pin state: output data, direction, previous-cycle input (for edge
    // detect), interrupt configuration, pending bits.
    // =================================================================
    logic [  WIDTH-1:0] out_q;
    logic [  WIDTH-1:0] dir_q;
    logic [  WIDTH-1:0] gpio_prev_q;
    logic [2*WIDTH-1:0] int_type_q;
    logic [  WIDTH-1:0] int_en_q;
    logic [  WIDTH-1:0] pend_q;

    assign gpio_o    = out_q;
    assign gpio_oe_o = dir_q;

    // Per-pin event set terms (see the interrupt model in the header).
    // TYPE 00 = level-high, 01 = rise, 10 = fall, 11 = both.
    logic [WIDTH-1:0] ev_level;
    logic [WIDTH-1:0] ev_rise;
    logic [WIDTH-1:0] ev_fall;
    logic [WIDTH-1:0] pend_set;

    genvar gp;
    generate
        for (gp = 0; gp < WIDTH; gp = gp + 1) begin : g_events
            assign ev_rise[gp] = ~gpio_prev_q[gp] & gpio_i[gp];
            assign ev_fall[gp] = gpio_prev_q[gp] & ~gpio_i[gp];
            assign ev_level[gp] = gpio_i[gp];
            // level | rise | fall | both, selected on the pin's TYPE field.
            assign pend_set[gp] = ((int_type_q[2*gp+:2] == 2'd0) && ev_level[gp]) ||
                ((int_type_q[2*gp+:2] == 2'd1) && ev_rise[gp]) ||
                ((int_type_q[2*gp+:2] == 2'd2) && ev_fall[gp]) ||
                ((int_type_q[2*gp+:2] == 2'd3) && (ev_rise[gp] | ev_fall[gp]));
        end
    endgenerate

    // =================================================================
    // AXI write path (same accept pattern as axi4_lite_uart).
    // =================================================================
    logic              aw_seen_q;
    logic              w_seen_q;
    logic [DATA_W-1:0] wdata_q;
    logic [STRB_W-1:0] wstrb_q;
    logic [       2:0] awaddr_word_q;
    logic              bvalid_q;

    assign axi.awready = !aw_seen_q && !bvalid_q;
    assign axi.wready  = !w_seen_q && !bvalid_q;

    wire              aw_hs = axi.awvalid && axi.awready;
    wire              w_hs = axi.wvalid && axi.wready;

    wire              aw_present = aw_seen_q || aw_hs;
    wire              w_present = w_seen_q || w_hs;

    wire [       2:0] waddr_eff = aw_hs ? axi.awaddr[4:2] : awaddr_word_q;
    wire [DATA_W-1:0] wdata_eff = w_hs ? axi.wdata : wdata_q;
    wire [STRB_W-1:0] wstrb_eff = w_hs ? axi.wstrb : wstrb_q;

    // Every register here lives in the low WIDTH bits of its word, so a
    // write that does not strobe byte 0 addresses no field (same rationale
    // as axi4_lite_uart's wr_low_byte).
    wire              wr_low_byte = wstrb_eff[0];

    assign axi.bvalid = bvalid_q;
    assign axi.bresp  = 2'b00;  // OKAY
    wire b_hs = axi.bvalid && axi.bready;

    wire do_write = aw_present && w_present && !bvalid_q;
    wire w1c_status = do_write && (waddr_eff == REG_INT_STATUS) && wr_low_byte;

    // Set wins over W1C (see the interrupt model in the header): an edge in
    // the clear cycle is not lost, and a level pin does not un-stick.
    logic [WIDTH-1:0] pend_d;
    assign pend_d = (pend_q & ~wdata_eff[WIDTH-1:0]) | pend_set;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            aw_seen_q     <= 1'b0;
            w_seen_q      <= 1'b0;
            wdata_q       <= '0;
            wstrb_q       <= '0;
            awaddr_word_q <= '0;
            bvalid_q      <= 1'b0;
            out_q         <= '0;
            dir_q         <= '0;
            int_type_q    <= '0;
            int_en_q      <= '0;
            pend_q        <= '0;
            gpio_prev_q   <= '0;
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

            // Edge detect + pending update run every cycle; a W1C write is
            // just one more term in pend_d.
            gpio_prev_q <= gpio_i;
            if (w1c_status) begin
                pend_q <= pend_d;
            end else begin
                pend_q <= pend_q | pend_set;
            end

            if (do_write) begin
                aw_seen_q <= 1'b0;
                w_seen_q  <= 1'b0;
                bvalid_q  <= 1'b1;

                unique case (waddr_eff)
                    REG_OUT:
                    if (wr_low_byte) begin
                        out_q <= wdata_eff[WIDTH-1:0];
                    end
                    REG_DIR:
                    if (wr_low_byte) begin
                        dir_q <= wdata_eff[WIDTH-1:0];
                    end
                    REG_INT_TYPE:
                    if (wr_low_byte) begin
                        int_type_q <= wdata_eff[2*WIDTH-1:0];
                    end
                    REG_INT_EN:
                    if (wr_low_byte) begin
                        int_en_q <= wdata_eff[WIDTH-1:0];
                    end
                    default: ;  // VALUE read-only; INT_STATUS handled above
                endcase
            end

            if (b_hs) begin
                bvalid_q <= 1'b0;
            end
        end
    end

    // =================================================================
    // AXI read path (1-cycle registered latency, side-effect-free: VALUE,
    // OUT, DIR, INT_TYPE, INT_EN and INT_STATUS are all plain reads).
    // =================================================================
    logic              rvalid_q;
    logic [DATA_W-1:0] rdata_q;

    assign axi.arready = !rvalid_q || (rvalid_q && axi.rready);
    wire ar_hs = axi.arvalid && axi.arready;

    assign axi.rvalid = rvalid_q;
    assign axi.rdata  = rdata_q;
    assign axi.rresp  = 2'b00;  // OKAY

    logic [DATA_W-1:0] rdata_mux;
    always_comb begin
        unique case (axi.araddr[4:2])
            REG_VALUE:      rdata_mux = {{(DATA_W - WIDTH) {1'b0}}, gpio_i};
            REG_OUT:        rdata_mux = {{(DATA_W - WIDTH) {1'b0}}, out_q};
            REG_DIR:        rdata_mux = {{(DATA_W - WIDTH) {1'b0}}, dir_q};
            REG_INT_TYPE:   rdata_mux = {{(DATA_W - 2 * WIDTH) {1'b0}}, int_type_q};
            REG_INT_EN:     rdata_mux = {{(DATA_W - WIDTH) {1'b0}}, int_en_q};
            REG_INT_STATUS: rdata_mux = {{(DATA_W - WIDTH) {1'b0}}, pend_q};
            default:        rdata_mux = '0;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            rvalid_q <= 1'b0;
            rdata_q  <= '0;
        end else begin
            if (ar_hs) begin
                rvalid_q <= 1'b1;
                rdata_q  <= rdata_mux;
            end else if (rvalid_q && axi.rready) begin
                rvalid_q <= 1'b0;
            end
        end
    end

    // =================================================================
    // Interrupt (level-sensitive, see header comment).
    // =================================================================
    assign gpio_irq_o = |(pend_q & int_en_q);

endmodule

`resetall

`resetall
`timescale 1ns / 1ps
`default_nettype none

/**
 * I2S MASTER model (simulation only).
 *
 * Generates the audio bus that axi4_lite_i2s consumes: BCLK, LRCK/WS and
 * SD, with the transmitter's edge convention -- WS and SD change on the
 * FALLING BCLK edge, so the receiver's rising edge always lands in the
 * middle of a stable bit.
 *
 * This is the counterpart of i2c_slave_model.sv: the device the DUT talks
 * to, written in RTL so that sim_top (and any testbench) gets the same
 * stimulus without a C++ bit-banger.
 *
 * Frame structure, per channel slot:
 *   - SLOT_BITS BCLK periods, WS low for the left slot, high for the
 *     right one.
 *   - FORMAT_LJ=0 (I2S / Philips): WS changes one BCLK BEFORE the MSB, so
 *     slot bit 0 carries padding and bits 1..WLEN carry the word.
 *   - FORMAT_LJ=1 (left-justified): the MSB is on slot bit 0.
 *   - bits after the word and up to the end of the slot are padding (0).
 *
 * BCLK = clk_i / (2*BCLK_HALF). BCLK_HALF must be >= 2, which is the
 * receiver's oversampling requirement (see axi4_lite_i2s.sv); its
 * `ifdef VERILATOR assertion fails the run if this model is configured
 * below that.
 *
 * left_i / right_i are latched once per frame (at the start of the left
 * slot) and load_o pulses for one core cycle when they are, so the
 * instantiating module can advance a sample generator in step with the
 * bus.
 */
module i2s_master_model #(
    parameter int unsigned BCLK_HALF = 4,    // core cycles per BCLK half period
    parameter int unsigned SLOT_BITS = 32,   // BCLK periods per channel slot
    parameter int unsigned WLEN      = 16,   // data bits per word, MSB first
    parameter bit          FORMAT_LJ = 1'b0  // 0 = I2S (1-BCLK delay), 1 = left-justified
) (
    input wire clk_i,
    input wire rstn_i,

    // Sample source, latched once per frame.
    input  wire [31:0] left_i,
    input  wire [31:0] right_i,
    output wire        load_o,

    // I2S bus (driven).
    output wire bclk_o,
    output wire lrck_o,
    output wire sd_o
);

    localparam int DIV_W = (BCLK_HALF <= 1) ? 1 : $clog2(BCLK_HALF);
    localparam int IDX_W = $clog2(SLOT_BITS);

    logic [DIV_W-1:0] div_q;
    logic             bclk_q;
    logic [IDX_W-1:0] idx_q;  // bit index within the current slot
    logic             ch_q;  // 0 = left, 1 = right
    logic [     31:0] wl_q;
    logic [     31:0] wr_q;
    logic             load_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            div_q  <= '0;
            bclk_q <= 1'b0;
            idx_q  <= '0;
            ch_q   <= 1'b0;
            wl_q   <= left_i;
            wr_q   <= right_i;
            load_q <= 1'b0;
        end else begin
            load_q <= 1'b0;

            if (div_q == DIV_W'(BCLK_HALF - 1)) begin
                div_q  <= '0;
                bclk_q <= ~bclk_q;

                // Falling edge: advance the slot and present the next bit.
                if (bclk_q) begin
                    if (idx_q == IDX_W'(SLOT_BITS - 1)) begin
                        idx_q <= '0;
                        ch_q  <= ~ch_q;
                        // Leaving the right slot starts a new frame: latch
                        // the next sample pair and tell the generator.
                        if (ch_q) begin
                            wl_q   <= left_i;
                            wr_q   <= right_i;
                            load_q <= 1'b1;
                        end
                    end else begin
                        idx_q <= idx_q + 1'b1;
                    end
                end
            end else begin
                div_q <= div_q + 1'b1;
            end
        end
    end

    // Current word and the bit of it this slot position carries.
    wire [31:0] word = ch_q ? wr_q : wl_q;
    wire [31:0] idx = {{(32 - IDX_W) {1'b0}}, idx_q};

    wire        bit_valid = FORMAT_LJ ? (idx < WLEN) : (idx >= 32'd1 && idx <= WLEN);
    wire [31:0] bit_pos = FORMAT_LJ ? (WLEN - 32'd1 - idx) : (WLEN - idx);

    assign bclk_o = bclk_q;
    assign lrck_o = ch_q;
    assign sd_o   = bit_valid ? word[bit_pos[4:0]] : 1'b0;
    assign load_o = load_q;

endmodule

`resetall

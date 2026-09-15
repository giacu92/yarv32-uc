`resetall
`timescale 1ns / 1ps
`default_nettype none

/**
 * I2S microphone model (simulation only) -- an INMP441-shaped device.
 *
 * The counterpart of i2s_master_model: that one drives the whole bus, this
 * one is a CLOCK SLAVE like the real part. It listens to BCLK and LRCK and
 * shifts a word out on SD, which is the configuration the board uses --
 * the FPGA is the master, the mic answers.
 *
 * That difference is the reason both models exist. An INMP441 cannot
 * generate BCLK, so a build where the peripheral is also a slave has
 * nobody clocking the bus; it is this model, not the master one, that
 * exercises what the guitar tuner actually runs.
 *
 * Shape of the real part:
 *   - 24 bits, MSB first, in ONE channel slot (the L/R pin picks which;
 *     CHANNEL below is the equivalent). The other slot is silent.
 *   - I2S framing: the word starts one BCLK after the WS transition.
 *   - Data changes on the FALLING BCLK edge.
 *   - Bits after the 24th are zero until the next WS transition.
 *
 * Edges are detected in clk_i, not used as clocks, for the same reason the
 * receiver does it that way: this stays a single-clock-domain simulation.
 * clk_i must therefore be at least 4x BCLK, which is the same constraint
 * the peripheral documents.
 *
 * sample_i is latched at the start of each of this device's slots, and
 * load_o pulses for one cycle when it is.
 */
module i2s_mic_model #(
    parameter int unsigned WLEN      = 24,    // data bits per word
    parameter bit          CHANNEL   = 1'b0,  // 0 = left slot, 1 = right
    parameter bit          FORMAT_LJ = 1'b0   // 0 = I2S, 1 = left-justified
) (
    input wire clk_i,
    input wire rstn_i,

    // Bus, as seen on the pads (driven by whoever is the master).
    input wire bclk_i,
    input wire lrck_i,

    // Sample source, latched once per frame.
    input  wire [31:0] sample_i,
    output wire        load_o,

    output wire sd_o
);

    logic        bclk_q;
    logic        ws_q;  // WS as seen at the previous falling edge
    logic [ 7:0] idx_q;  // bit position within the current slot
    logic        load_q;
    logic        sd_q;

    logic [31:0] word_hold_q;  // the word currently being shifted out

    wire         bclk_fall = bclk_q & ~bclk_i;

    // With the I2S 1-BCLK delay the MSB sits at slot index 1;
    // left-justified puts it at 0.
    localparam logic [7:0] FIRST = FORMAT_LJ ? 8'd0 : 8'd1;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            bclk_q      <= 1'b0;
            ws_q        <= 1'b0;
            idx_q       <= '0;
            word_hold_q <= sample_i;
            load_q      <= 1'b0;
            sd_q        <= 1'b0;
        end else begin
            bclk_q <= bclk_i;
            load_q <= 1'b0;

            if (bclk_fall) begin
                logic        ws_change;
                logic [ 7:0] idx_next;
                logic [31:0] word_next;
                logic [ 7:0] pos;

                ws_change = (lrck_i != ws_q);
                idx_next  = ws_change ? 8'd0 : (idx_q + 8'd1);
                // Entering this device's own slot: take a new sample.
                word_next = (ws_change && lrck_i == CHANNEL) ? sample_i : word_hold_q;
                pos       = (FIRST + 8'(WLEN) - 8'd1) - idx_next;

                ws_q        <= lrck_i;
                idx_q       <= idx_next;
                word_hold_q <= word_next;
                load_q      <= ws_change && (lrck_i == CHANNEL);

                // Present the bit the receiver samples on the next rising
                // edge. The other slot, and every bit past the word, is 0.
                if (lrck_i == CHANNEL && idx_next >= FIRST && idx_next < FIRST + 8'(WLEN)) begin
                    sd_q <= word_next[pos[4:0]];
                end else begin
                    sd_q <= 1'b0;
                end
            end
        end
    end

    assign sd_o   = sd_q;
    assign load_o = load_q;

endmodule

`resetall

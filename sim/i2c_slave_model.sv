`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Sim-only I2C slave device model, wired into sim_top so firmware I2C
 * libraries and oracles have a real device to talk to (the same role the
 * loopback plays for GPIO and the harness line for UART).
 *
 * A cycle-accurate SV port of the proven C++ model in sim/hw/i2c_tb/
 * i2c_tb.cpp (I2cSlave::step) -- a register-file device at address 0x50:
 * 16 byte registers initialised to 0x10+i, auto-incrementing pointer, the
 * first write byte of a transaction sets the pointer. It detects START /
 * repeated-START / STOP, shifts the address byte, ACKs on match, accepts
 * write bytes, shifts out read bytes MSB-first, stops on a master NACK.
 * No clock stretching (SCL is never pulled by the slave), like the C++
 * original.
 *
 * The bus lines are the caller's wired-AND-with-pull-up (2-state, no
 * tri-state): scl_i / sda_i are the settled line values, sda_oe_o = 1
 * pulls SDA low. The C++ model drives its output at the end of the step
 * following the edge it reacted to; sda_oe_o is the registered image of
 * that (sda_drive_q holds exactly the last step's final sda_drive).
 *
 * Sim-only; not on any synthesis file list.
 */

module i2c_slave_model #(
    parameter logic [6:0] ADDR7 = 7'h50,
    parameter int unsigned REGS = 16
) (
    input  wire clk_i,
    input  wire rstn_i,
    input  wire scl_i,    // settled bus line
    input  wire sda_i,    // settled bus line
    output wire sda_oe_o  // 1 = pull SDA low
);

    localparam int PTR_W = $clog2(REGS);

    typedef enum logic [1:0] {
        IDLE,
        ADDR,
        WR,
        RD
    } st_e;

    st_e st_q, st_d;
    logic [3:0] bit_cnt_q, bit_cnt_d;  // 0..7 data, 8 ACK slot, 9 post-ACK
    logic [7:0] shift_q, shift_d;
    logic is_read_q, is_read_d;
    logic addr_match_q, addr_match_d;
    logic first_wr_q, first_wr_d;  // next WR byte is the pointer
    logic master_ack_q, master_ack_d;  // read ACK slot, 1 = NACK
    logic [7:0] tx_byte_q, tx_byte_d;  // byte being shifted out during RD
    logic sda_drive_q, sda_drive_d;  // 1 = release, 0 = pull low
    logic [PTR_W-1:0] reg_ptr_q, reg_ptr_d;

    logic scl_prev_q, sda_prev_q;

    logic [7:0] regs[REGS];

    initial begin
        for (int i = 0; i < REGS; i++) regs[i] = 8'h10 + i[7:0];
    end

    // Edge detects off the previous-cycle line samples.
    wire scl_r = scl_i & ~scl_prev_q;
    wire scl_f = ~scl_i & scl_prev_q;
    wire sda_f = ~sda_i & sda_prev_q;
    wire sda_r = sda_i & ~sda_prev_q;

    // Next-state + next-drive: a straight port of I2cSlave::step(). The byte
    // machinery operates on the _d values the START/STOP section may have
    // just updated, matching the C++ fall-through order (a START and an SCL
    // edge cannot coincide with this master, which only moves SDA while SCL
    // is low -- but the port keeps the order anyway).
    always_comb begin
        st_d         = st_q;
        bit_cnt_d    = bit_cnt_q;
        shift_d      = shift_q;
        is_read_d    = is_read_q;
        addr_match_d = addr_match_q;
        first_wr_d   = first_wr_q;
        master_ack_d = master_ack_q;
        tx_byte_d    = tx_byte_q;
        sda_drive_d  = sda_drive_q;
        reg_ptr_d    = reg_ptr_q;

        // START / repeated-START / STOP are recognised while SCL is high.
        if (scl_i == 1'b1) begin
            if (sda_f) begin
                st_d         = ADDR;
                bit_cnt_d    = 4'd0;
                shift_d      = 8'h00;
                addr_match_d = 1'b0;
                first_wr_d   = 1'b1;
                sda_drive_d  = 1'b1;
            end else if (sda_r && st_q != IDLE) begin
                st_d        = IDLE;
                sda_drive_d = 1'b1;
            end
        end

        if (st_d != IDLE) begin
            if (scl_r) begin
                if (bit_cnt_d < 4'd8) begin
                    if (st_d == ADDR || st_d == WR) shift_d = {shift_d[6:0], sda_i};
                    bit_cnt_d = bit_cnt_d + 4'd1;
                end else if (bit_cnt_d == 4'd8) begin
                    // ACK slot rising: on a read, the master's ACK/NACK is on SDA.
                    if (st_d == RD) master_ack_d = sda_i;
                    bit_cnt_d = bit_cnt_d + 4'd1;
                end
            end else if (scl_f) begin
                if (bit_cnt_d <= 4'd7) begin
                    // Prepare the next read data bit (stable before SCL rises).
                    if (st_d == RD && bit_cnt_d >= 4'd1) sda_drive_d = tx_byte_d[7-bit_cnt_d];
                end else if (bit_cnt_d == 4'd8) begin
                    // ACK slot falling: the slave drives SDA.
                    if (st_d == ADDR) begin
                        addr_match_d = (shift_d[7:1] == ADDR7);
                        is_read_d    = shift_d[0];
                        sda_drive_d  = addr_match_d ? 1'b0 : 1'b1;  // low = ACK
                    end else if (st_d == WR) begin
                        sda_drive_d = 1'b0;  // ACK every accepted write byte
                    end else begin  // RD
                        sda_drive_d = 1'b1;  // release for the master's ACK
                    end
                end else if (bit_cnt_d == 4'd9) begin
                    // Post-ACK transition.
                    sda_drive_d = 1'b1;
                    bit_cnt_d   = 4'd0;
                    if (st_d == ADDR) begin
                        if (!addr_match_d) begin
                            st_d = IDLE;
                        end else if (is_read_d) begin
                            st_d        = RD;
                            tx_byte_d   = regs[reg_ptr_d];
                            sda_drive_d = tx_byte_d[7];  // present bit 7
                        end else begin
                            st_d = WR;
                        end
                    end else if (st_d == WR) begin
                        if (first_wr_d) begin
                            reg_ptr_d  = shift_d[PTR_W-1:0];
                            first_wr_d = 1'b0;
                        end else begin
                            reg_ptr_d = reg_ptr_d + 1'b1;  // data byte: advance
                        end
                        // The data byte itself goes through the write port
                        // in the sequential block below.
                    end else begin  // RD
                        if (!master_ack_d) begin
                            reg_ptr_d   = reg_ptr_d + 1'b1;
                            tx_byte_d   = regs[reg_ptr_d];  // post-increment, as the C++ model
                            sda_drive_d = tx_byte_d[7];
                        end else begin
                            st_d = IDLE;  // master NACK: it will issue STOP
                        end
                    end
                end
            end
        end
    end

    // Register-file write port: a completed WR data byte (first_wr consumed).
    wire regs_we = rstn_i && scl_f && (st_q == WR) && (bit_cnt_q == 4'd9) && !first_wr_q;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            st_q         <= IDLE;
            bit_cnt_q    <= 4'd0;
            shift_q      <= 8'h00;
            is_read_q    <= 1'b0;
            addr_match_q <= 1'b0;
            first_wr_q   <= 1'b0;
            master_ack_q <= 1'b0;
            tx_byte_q    <= 8'h00;
            sda_drive_q  <= 1'b1;
            reg_ptr_q    <= '0;
            scl_prev_q   <= 1'b1;
            sda_prev_q   <= 1'b1;
        end else begin
            st_q         <= st_d;
            bit_cnt_q    <= bit_cnt_d;
            shift_q      <= shift_d;
            is_read_q    <= is_read_d;
            addr_match_q <= addr_match_d;
            first_wr_q   <= first_wr_d;
            master_ack_q <= master_ack_d;
            tx_byte_q    <= tx_byte_d;
            sda_drive_q  <= sda_drive_d;
            reg_ptr_q    <= reg_ptr_d;
            scl_prev_q   <= scl_i;
            sda_prev_q   <= sda_i;

            if (regs_we) regs[reg_ptr_q] <= shift_q;
        end
    end

    assign sda_oe_o = (sda_drive_q == 1'b0);

endmodule

`resetall

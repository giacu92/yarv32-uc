`timescale 1ns / 1ps
`default_nettype none

/**
* Minimal serial NOR flash model for simulation (M9): SPI mode 0, commands
* 0x03 READ (24-bit address, streaming, wraps at the end of the array) and
* 0x9F JEDEC ID (EF 40 17, a W25Q64 -- an 8 MiB part like the Tang Nano 20K's
* configuration flash). Anything else is ignored until CS rises. Contents come
* from +FINIT=<hex>, 32-bit little-endian words with @word addresses (the
* same format as the SDRAM images); unwritten bytes read 0xFF like an erased
* part.
*/
module spi_flash_model #(
    parameter int ADDR_W = 23  // 8 MiB
) (
    input  wire sck,
    input  wire cs_n,
    input  wire mosi,
    output wire miso
);
    logic [7:0] mem[1 << ADDR_W];
    logic [31:0] words[1 << (ADDR_W - 2)];
    string finit;
    initial begin
        for (int i = 0; i < (1 << ADDR_W); i++) mem[i] = 8'hFF;
        for (int i = 0; i < (1 << (ADDR_W - 2)); i++) words[i] = 32'hFFFF_FFFF;
        if ($value$plusargs("FINIT=%s", finit)) begin
            $readmemh(finit, words);
            for (int i = 0; i < (1 << (ADDR_W - 2)); i++)
                for (int b = 0; b < 4; b++) mem[4*i+b] = words[i][8*b+:8];
        end
    end

    // Bit counter / shift registers, reset by CS high.
    int unsigned nbit;
    logic [7:0] cmd;
    logic [23:0] addr;
    logic [7:0] sh_out;
    logic out_bit;
    logic [2:0] id_idx;
    int unsigned pos;  // temporaries of the output block
    logic [23:0] a;
    logic [7:0] idb;

    always @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            nbit <= 0;
            cmd  <= 8'h00;
        end else begin
            if (nbit < 8) cmd <= {cmd[6:0], mosi};
            else if (nbit < 32 && cmd == 8'h03) addr <= {addr[22:0], mosi};
            nbit <= nbit + 1;
        end
    end

    // Output byte for the bit position about to be shifted out. Mode 0: the
    // slave changes MISO on the falling edge, the master samples on rising.
    always @(negedge sck or posedge cs_n) begin
        if (cs_n) begin
            out_bit <= 1'b0;
            id_idx  <= 0;
        end else begin
            if (cmd == 8'h03 && nbit >= 32) begin
                pos = nbit - 32;  // bits already sent
                a   = addr + 24'(pos / 8);
                out_bit <= mem[a[ADDR_W-1:0]][7-(pos%8)];
            end else if (cmd == 8'h9F && nbit >= 8) begin
                pos = nbit - 8;
                idb = (pos / 8 == 0) ? 8'hEF : (pos / 8 == 1) ? 8'h40 : 8'h17;
                out_bit <= idb[7-(pos%8)];
            end else out_bit <= 1'b0;
        end
    end
    assign miso = out_bit;
endmodule

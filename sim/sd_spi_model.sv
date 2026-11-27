`timescale 1ns / 1ps
`default_nettype none

/**
* Behavioural SD card in SPI mode, for simulation (M9b).
*
* Clocked in the core clock domain: SCK edges are detected by sampling sck at
* every clk_i edge, which also gives the SCK period in core cycles for the
* init-speed check. Edge conventions, matching axi4_lite_spi in mode 0:
*   - MOSI is sampled on the SCK RISING edge (as seen one core cycle after
*     it, i.e. the same value an @(posedge sck) model such as
*     spi_flash_model sees);
*   - MISO changes SDDELAY core cycles after the SCK FALLING edge (minimum 1,
*     default 1). Between bytes, the falling edge after a byte's 8th rising
*     edge loads the next byte's MSB, so it is valid before the first rising
*     edge of that byte, as mode 0 requires.
* MISO is 1 while CS is high or when nothing is queued.
*
* Plusargs
*   +SDCARD         selects the model (sim_soc muxes MISO on it); without it
*                   the model is inert and prints nothing.
*   +SDINIT=<hex>   card contents: 32-bit little-endian words, @word
*                   addresses (byte = 4*word), the +FINIT format. Unwritten
*                   bytes read 0x00. Capacity 2^ADDR_W bytes (16 MiB).
*   +SDSC           standard-capacity card: CMD17/18 take a BYTE address,
*                   OCR.CCS = 0. Default is SDHC: block address, CCS = 1.
*   +SDLOG          one $display line per command (and per CMD18 stream).
*   +SDDELAY=<n>    MISO output delay after SCK fall, core cycles (>= 1).
*   +SDNCR=<n>      bytes of 0xFF between a command's CRC byte and its R1
*                   (default NCR = 2: the master clocks 2 x 0xFF, then R1).
*
* Commands (SD Physical Layer Simplified Spec, SPI mode):
*   CMD0   R1 0x01, resets init state. Needs a valid CRC (0x95 for arg 0),
*          else R1 0x09. Warns if fewer than 74 SCK clocks with CS high were
*          seen before it.
*   CMD8   R7 = R1 0x01 (0x00 after init), 0x00, 0x00, arg[11:8], arg[7:0].
*          Needs a valid CRC (0x87 for 0x1AA), else R1 0x09.
*   CMD55  R1, arms the next command as an ACMD.
*   ACMD41 R1 0x01 for the first ACMD41_BUSY calls, then 0x00 (init done).
*          An SDHC card given HCS=0 (arg bit 30 clear) stays busy forever.
*   CMD58  R3 = R1 + OCR (bit 31 = init done, bit 30 = CCS once done,
*          voltage window 0x00FF8000).
*   CMD59  R1 (CRC on/off: accepted, CRC is never checked beyond CMD0/8).
*   CMD16  R1 0x00 for 512, 0x40 otherwise.
*   CMD9/10 R1 0x00, Nac, 0xFE, 16-byte CSD/CID, CRC16.
*   CMD17  R1 0x00, NAC x 0xFF, 0xFE, 512 bytes, CRC16 (XMODEM/CCITT, real).
*   CMD18  as CMD17, block after block with its own Nac/token/CRC, until
*          CMD12. The command bytes of CMD12 are received while data is still
*          being sent.
*   CMD12  after its CRC byte: one stuff byte (STUFF), R1 0x00, two 0x00 busy
*          bytes, then 0xFF. Accepted (same answer) with no stream active.
*   Out-of-range read address: R1 0x40; SDSC address not 512-aligned: 0x20.
*   Before init completes, any command but CMD0/8/55/41/58/59 gets R1 0x01;
*   an unknown command gets 0x04 (0x05 before init).
*   CS high flushes any pending output and ends a CMD18 stream.
*/
module sd_spi_model #(
    parameter int         ADDR_W       = 24,     // 16 MiB
    parameter int         NCR          = 2,
    parameter int         NAC          = 4,
    parameter int         ACMD41_BUSY  = 3,
    parameter logic [7:0] STUFF        = 8'h5A,  // CMD12 stuff byte: NOT an R1
    parameter int         INIT_MIN_CLK = 125     // 400 kHz at 50 MHz
) (
    input  wire clk,
    input  wire sck,
    input  wire cs_n,
    input  wire mosi,
    output wire miso
);
    localparam int NWORDS = 1 << (ADDR_W - 2);
    logic [31:0] words[NWORDS];
    string sdinit;
    bit sdhc, log_en, en;
    int unsigned delay, ncr;
    initial begin
        for (int i = 0; i < NWORDS; i++) words[i] = 32'h0;
        if ($value$plusargs("SDINIT=%s", sdinit)) $readmemh(sdinit, words);
        en     = $test$plusargs("SDCARD");  // inert (no logs) unless selected
        sdhc   = !$test$plusargs("SDSC");
        log_en = $test$plusargs("SDLOG");
        if (!$value$plusargs("SDDELAY=%d", delay)) delay = 1;
        if (delay < 1) delay = 1;
        if (!$value$plusargs("SDNCR=%d", ncr)) ncr = NCR;
    end

    function automatic logic [7:0] rd(input longint unsigned a);
        logic [31:0] w;
        w = words[(a>>2)&(NWORDS-1)];
        return w[8*(a&3)+:8];
    endfunction

    function automatic logic [6:0] crc7(input logic [39:0] d);
        logic [6:0] c = 0;
        for (int i = 39; i >= 0; i--) begin
            logic fb = c[6] ^ d[i];
            c = {c[5:0], 1'b0};
            if (fb) c ^= 7'h09;
        end
        return c;
    endfunction

    function automatic logic [15:0] crc16_upd(input logic [15:0] c, input logic [7:0] b);
        for (int i = 7; i >= 0; i--) begin
            logic fb = c[15] ^ b[i];
            c = {c[14:0], 1'b0};
            if (fb) c ^= 16'h1021;
        end
        return c;
    endfunction

    // ---- state ----
    logic [7:0] q[$];  // output byte queue
    bit sck_d;
    bit [7:0] in_sh;
    int unsigned rcnt;  // rising edges in the current byte
    bit byte_bnd;  // next fall starts a new output byte
    logic [7:0] cur;
    int obit;
    logic miso_q = 1'b1;
    bit pend;
    int unsigned pend_cnt;
    logic pend_bit;

    logic [7:0] cmdb[6];
    int unsigned ncmd;  // command bytes collected (0 = hunting for a start byte)
    bit init_done, acmd, multi;
    int unsigned acmd41_n, multi_blk, multi_cnt;
    int unsigned pre_clocks;
    bit seen_cmd0;
    longint unsigned cyc, last_rise;
    int unsigned min_per;
    bit warned_fast;

    function automatic void push_block(longint unsigned a);
        logic [15:0] c = 0;
        for (int i = 0; i < NAC; i++) q.push_back(8'hFF);
        q.push_back(8'hFE);
        for (int i = 0; i < 512; i++) begin
            logic [7:0] b = rd(a + i);
            q.push_back(b);
            c = crc16_upd(c, b);
        end
        q.push_back(c[15:8]);
        q.push_back(c[7:0]);
    endfunction

    function automatic void push_reg16(logic [127:0] r);
        logic [15:0] c = 0;
        for (int i = 0; i < NAC; i++) q.push_back(8'hFF);
        q.push_back(8'hFE);
        for (int i = 15; i >= 0; i--) begin
            q.push_back(r[8*i+:8]);
            c = crc16_upd(c, r[8*i+:8]);
        end
        q.push_back(c[15:8]);
        q.push_back(c[7:0]);
    endfunction

    function automatic void push_r1(logic [7:0] r1);
        for (int i = 0; i < ncr; i++) q.push_back(8'hFF);
        q.push_back(r1);
    endfunction

    // Byte address of a read argument; returns 0 on success, else an R1 error.
    function automatic logic [7:0] rd_addr(input logic [31:0] arg, output longint unsigned a);
        a = sdhc ? longint'(arg) * 512 : longint'(arg);
        if (!sdhc && arg[8:0] != 0) return 8'h20;
        if (a + 512 > (longint'(1) << ADDR_W)) return 8'h40;
        return 8'h00;
    endfunction

    function automatic logic [127:0] csd();
        logic [127:0] r = '0;
        if (sdhc) begin  // CSD v2: capacity = (C_SIZE+1) * 512 KiB
            r[127:126] = 2'b01;
            r[69:48]   = 22'((1 << (ADDR_W - 19)) - 1);
        end else begin  // CSD v1: (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^READ_BL_LEN
            r[127:126] = 2'b00;
            r[83:80]   = 4'd9;
            r[49:47]   = 3'd7;
            r[73:62]   = 12'((1 << (ADDR_W - 18)) - 1);
        end
        r[103:96] = 8'h32;  // TRAN_SPEED 25 MHz
        r[0]      = 1'b1;
        return r;
    endfunction

    function automatic void do_cmd();
        logic [ 5:0] idx = cmdb[0][5:0];
        logic [31:0] arg = {cmdb[1], cmdb[2], cmdb[3], cmdb[4]};
        logic [ 7:0] crc = cmdb[5];
        logic [ 7:0] idle = init_done ? 8'h00 : 8'h01;
        logic [7:0] r1, e;
        longint unsigned a;
        bit is_acmd = acmd;
        string what;
        acmd = 0;
        if (!(idx == 12 && multi)) q.delete();
        what = "";
        if (idx == 0) begin
            if (crc != {crc7({cmdb[0], arg}), 1'b1}) r1 = 8'h09;
            else begin
                init_done = 0;
                acmd41_n  = 0;
                multi     = 0;
                r1        = 8'h01;
                if (!seen_cmd0 && pre_clocks < 74)
                    $display(
                        "SD WARNING: CMD0 after only %0d SCK clocks with CS high (need >= 74)",
                        pre_clocks
                    );
                seen_cmd0 = 1;
            end
            push_r1(r1);
        end else if (idx == 8) begin
            if (crc != {crc7({cmdb[0], arg}), 1'b1}) begin
                r1 = 8'h09;
                push_r1(r1);
            end else begin
                r1 = idle;
                push_r1(r1);
                q.push_back(8'h00);
                q.push_back(8'h00);
                q.push_back({4'h0, arg[11:8]});
                q.push_back(arg[7:0]);
            end
        end else if (idx == 55) begin
            r1   = idle;
            acmd = 1;
            push_r1(r1);
        end else if (idx == 41 && is_acmd) begin
            if (init_done) r1 = 8'h00;
            else if (sdhc && !arg[30]) begin
                r1 = 8'h01;
                if (acmd41_n == 0)
                    $display("SD WARNING: ACMD41 with HCS=0 to an SDHC card: stays busy forever");
                acmd41_n++;
            end else if (acmd41_n < ACMD41_BUSY) begin
                acmd41_n++;
                r1 = 8'h01;
            end else begin
                init_done = 1;
                r1        = 8'h00;
                if (log_en)
                    $display("SD: init done, min SCK period during init %0d core clk", min_per);
            end
            push_r1(r1);
        end else if (idx == 58) begin
            r1 = idle;
            push_r1(r1);
            q.push_back({init_done, init_done & sdhc, 6'b0});
            q.push_back(8'hFF);
            q.push_back(8'h80);
            q.push_back(8'h00);
        end else if (idx == 59) begin
            r1 = idle;
            push_r1(r1);
        end else if (!init_done && (idx == 16 || idx == 17 || idx == 18 || idx == 12 || idx == 9 ||
                                    idx == 10)) begin
            r1 = 8'h01;
            push_r1(r1);
        end else if (idx == 16) begin
            r1 = (arg == 512) ? 8'h00 : 8'h40;
            push_r1(r1);
        end else if (idx == 9 || idx == 10) begin
            r1 = 8'h00;
            push_r1(r1);
            push_reg16(
                idx == 9 ? csd() : {8'h03, "SD", "YARVC", 8'h10, 32'h1234_5678, 16'h0123, 8'h01});
        end else if (idx == 17 || idx == 18) begin
            e  = rd_addr(arg, a);
            r1 = e;
            push_r1(r1);
            if (e == 0) begin
                push_block(a);
                if (idx == 18) begin
                    multi     = 1;
                    multi_blk = int'(a / 512) + 1;
                    multi_cnt = 1;
                end
            end
            $sformat(what, " byte 0x%0h", a);
        end else if (idx == 12) begin
            r1 = 8'h00;
            if (multi) begin
                $sformat(what, " (stream ended after %0d blocks started)", multi_cnt);
                q.delete();
            end
            multi = 0;
            q.push_back(STUFF);
            q.push_back(r1);
            q.push_back(8'h00);
            q.push_back(8'h00);
        end else begin
            r1 = idle | 8'h04;
            push_r1(r1);
        end
        if (log_en)
            $display(
                "SD: %sCMD%0d arg=0x%08h crc=0x%02h -> R1=0x%02h%s",
                is_acmd ? "A" : "",
                idx,
                arg,
                crc,
                r1,
                what
            );
    endfunction

    function automatic void got_byte(logic [7:0] b);
        if (ncmd == 0) begin
            if (b[7:6] != 2'b01) return;
        end
        cmdb[ncmd] = b;
        ncmd++;
        if (ncmd == 6) begin
            ncmd = 0;
            do_cmd();
        end
    endfunction

    always @(posedge clk) begin
        cyc++;
        if (!en) begin
        end else if (cs_n) begin
            if (sck && !sck_d) begin
                pre_clocks++;
                if (!init_done) begin
                    if (last_rise != 0 && (min_per == 0 || cyc - last_rise < min_per))
                        min_per = int'(cyc - last_rise);
                    last_rise = cyc;
                end
            end
            if (multi && log_en) $display("SD: CS high during CMD18 stream, stream ended");
            multi = 0;
            q.delete();
            rcnt     = 0;
            byte_bnd = 0;
            cur      = 8'hFF;
            obit     = 7;
            ncmd     = 0;
            pend     = 0;
            miso_q <= 1'b1;
        end else begin
            if (sck && !sck_d) begin  // rising: sample MOSI
                if (!init_done) begin
                    if (last_rise != 0 && (min_per == 0 || cyc - last_rise < min_per))
                        min_per = int'(cyc - last_rise);
                    if (last_rise != 0 && cyc - last_rise < INIT_MIN_CLK && !warned_fast) begin
                        warned_fast = 1;
                        $display(
                            "SD WARNING: SCK period %0d core clk during init (< %0d = 400 kHz)",
                            cyc - last_rise, INIT_MIN_CLK);
                    end
                    last_rise = cyc;
                end
                in_sh = {in_sh[6:0], mosi};
                rcnt++;
                if (rcnt == 8) begin
                    rcnt     = 0;
                    byte_bnd = 1;
                    got_byte(in_sh);
                end
            end
            if (!sck && sck_d) begin  // falling: next MISO bit
                if (byte_bnd) begin
                    byte_bnd = 0;
                    if (q.size() == 0 && multi) begin
                        push_block(longint'(multi_blk) * 512);
                        multi_blk++;
                        multi_cnt++;
                    end
                    cur  = (q.size() != 0) ? q.pop_front() : 8'hFF;
                    obit = 7;
                end else if (obit > 0) obit--;
                pend     = 1;
                pend_cnt = delay - 1;
                pend_bit = cur[obit];
            end
            if (pend) begin
                if (pend_cnt == 0) begin
                    miso_q <= pend_bit;
                    pend = 0;
                end else pend_cnt--;
            end
        end
        sck_d = sck;
    end
    assign miso = cs_n ? 1'b1 : miso_q;
endmodule

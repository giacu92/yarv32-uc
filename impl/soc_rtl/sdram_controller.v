/**
 * simple controller for ISSI IS42S16160G-7 SDRAM found in De0 Nano
 *  16Mbit x 16 data bit bus (32 megabytes)
 *  Default options
 *    133Mhz
 *    CAS 3
 *
 *  Very simple host interface
 *     * No burst support
 *     * haddr - address for reading and wriging 16 bits of data
 *     * data_input - data for writing, latched in when wr_enable is highz0
 *     * data_output - data for reading, comes available sometime
 *       *few clocks* after rd_enable and address is presented on bus
 *     * rst_n - start init ram process
 *     * rd_enable - read enable, on clk posedge haddr will be latched in,
 *       after *few clocks* data will be available on the data_output port
 *     * wr_enable - write enable, on clk posedge haddr and data_input will
 *       be latched in, after *few clocks* data will be written to sdram
 *
 * Theory
 *  This simple host interface has a busy signal to tell you when you are
 *  not able to issue commands.
 *
 * gw2ar-32bit adaptation (for the Gowin GW2AR-18 embedded SDRAM, 8 MiB):
 *   - DATA_WIDTH parameter added (32 bit for this project; 16 bit original
 *     port widths remain the default upstream).
 *   - data_mask_low/data_mask_high (2 bit, 16 bit bus) replaced by a single
 *     dqm output of DATA_WIDTH/8 bits.
 *   - Fixed zero-width replication in the SDRAM address muxes, which
 *     upstream never hit (SDRADDR_WIDTH=13) but a row width of 11 does.
 *   - CMD_MRS x bit replaced by 0 (bank address is a don't-care for MRS).
 *   - Geometry for this project (passed at instantiation): ROW_WIDTH=11,
 *     COL_WIDTH=8, BANK_WIDTH=2, CL=3 -> 2^11 * 2^8 * 4 * 4 B = 8 MiB.
 *     Host address haddr is {bank, row, col}, 21 bits, 32-bit word address.
 *
 * m6-burst rework (2026-11-27, read/write engine rewritten; init and
 * refresh sequences unchanged):
 *   - BURST READS. The mode register is programmed with burst length
 *     BURST_LEN (1/2/4/8), sequential, write burst mode "single location"
 *     (A9=1). One host read = one ACTIVATE, one READ with auto-precharge,
 *     then BURST_LEN consecutive data beats, each delivered on rd_data
 *     with a one-cycle rd_ready pulse. The burst wraps inside its
 *     BURST_LEN-aligned column block (sequential order), so a read at an
 *     aligned address returns words addr, addr+1, ... in that order.
 *   - STREAMED WRITES. With wr_burst=1 a host write moves BURST_LEN words:
 *     one ACTIVATE, then one single-location WRITE per cycle to
 *     consecutive columns of the open row, auto-precharge on the last.
 *     wr_burst=0 is the old single-word write. The address must be
 *     BURST_LEN-aligned for a burst (columns increment, they do not wrap).
 *   - Handshake: a request is taken on the cycle rd_ack / wr_ack is high
 *     (combinational: IDLE, no refresh due, enable high). Word 0 of a write
 *     is latched at the wr_ack edge; word k >= 1 is latched at the edge
 *     ending a cycle in which wr_next is high, so the host advances its
 *     word pointer on wr_ack and on every wr_next. The host must drop its
 *     enable after the ack (an enable still high when the controller next
 *     sits in IDLE is a new request). busy is kept for compatibility
 *     (registered state[4], as upstream).
 *   - Timing is derived from CLK_FREQUENCY and the T_* parameters (ns) and
 *     enforced by cycle counters: ACT->READ/WRITE >= tRCD, ACT->precharge
 *     >= tRAS (auto-precharge starts BURST_LEN cycles after a READ and
 *     T_WR_CLK cycles after the last WRITE), precharge->ACT/PALL >= tRP.
 *     CAS latency is fixed at 3 (CL).
 */


module sdram_controller (
    /* HOST INTERFACE */
    wr_addr,
    wr_data,
    wr_enable,
    wr_burst,
    wr_ack,
    wr_next,

    rd_addr,
    rd_data,
    rd_ready,
    rd_enable,
    rd_ack,

    busy, rst_n, clk,

    /* SDRAM SIDE */
    addr, bank_addr, data, clock_enable, cs_n, ras_n, cas_n, we_n, dqm
);

/* Internal Parameters */
parameter ROW_WIDTH = 13;
parameter COL_WIDTH = 9;
parameter BANK_WIDTH = 2;
parameter DATA_WIDTH = 16;

// Read burst length (and streamed-write length): 1, 2, 4 or 8.
parameter BURST_LEN = 1;

parameter SDRADDR_WIDTH = ROW_WIDTH > COL_WIDTH ? ROW_WIDTH : COL_WIDTH;
parameter HADDR_WIDTH = BANK_WIDTH + ROW_WIDTH + COL_WIDTH;
parameter DQM_WIDTH = DATA_WIDTH / 8;

parameter CLK_FREQUENCY = 133;  // Mhz
parameter REFRESH_TIME =  32;   // ms     (how often we need to refresh)
parameter REFRESH_COUNT = 8192; // cycles (how many refreshes required per refresh time)

// Device timing (ns unless noted). Defaults cover the GW2AR-18 embedded
// 64 Mbit SDR die and every common -6/-7 part.
parameter T_RCD_NS = 18;   // ACTIVATE -> READ/WRITE
parameter T_RP_NS  = 20;   // PRECHARGE -> ACTIVATE / REFRESH
parameter T_RAS_NS = 42;   // ACTIVATE -> PRECHARGE (minimum)
parameter T_WR_CLK = 2;    // last WRITE data -> auto-precharge start (clocks)

// clk / refresh =  clk / sec
//                , sec / refbatch
//                , ref / refbatch
localparam CYCLES_BETWEEN_REFRESH = ( CLK_FREQUENCY
                                      * 1_000
                                      * REFRESH_TIME
                                    ) / REFRESH_COUNT;

// patched: refresh counter widened to the compare width (upstream's 10-bit
// counter against the wider constant warned under Verilator WIDTH checks)
localparam CBR_WIDTH = 14;
localparam [CBR_WIDTH-1:0] CYCLES_BETWEEN_REFRESH_W = CYCLES_BETWEEN_REFRESH;

// ---------------------------------------------------------------------
// Timing in clocks. ns2cyc rounds UP and never returns less than 1.
// ---------------------------------------------------------------------
function integer ns2cyc;
  input integer ns;
  begin
  ns2cyc = (ns * CLK_FREQUENCY + 999) / 1000;
  if (ns2cyc < 1)
    ns2cyc = 1;
  end
endfunction

function integer imax;
  input integer a;
  input integer b;
  begin
  imax = (a > b) ? a : b;
  end
endfunction

localparam integer CL       = 3;
localparam integer RCD_CYC  = ns2cyc(T_RCD_NS);
localparam integer RP_CYC   = ns2cyc(T_RP_NS);
localparam integer RAS_CYC  = ns2cyc(T_RAS_NS);
// ACT command cycle -> READ / first WRITE command cycle. tRAS is met by
// construction: auto-precharge starts BURST_LEN cycles after the READ,
// T_WR_CLK cycles after a (single, worst-case) WRITE.
localparam integer ACT_TO_RD = imax(RCD_CYC, RAS_CYC - BURST_LEN);
localparam integer ACT_TO_WR = imax(RCD_CYC, RAS_CYC - T_WR_CLK);
// Recovery after the last READ data cycle / last WRITE cycle before the
// controller may return to IDLE. From IDLE the next ACT or PALL is on the
// pins one cycle later, so: READ  r -> precharge r+BL -> ACT >= r+BL+RP,
// earliest ACT without recovery = r+CL+BL+1; WRITE w -> precharge
// w+T_WR_CLK -> ACT >= w+T_WR_CLK+RP, earliest without recovery = w+2.
localparam integer RD_REC = imax(0, RP_CYC - CL - 1);
localparam integer WR_REC = imax(0, T_WR_CLK + RP_CYC - 2);

// The same values as 4-bit counter loads (rw_cnt counts down to 0).
function [3:0] cnt4;
  input integer v;
  begin
  cnt4 = v[3:0];
  end
endfunction

localparam [3:0] ACT_TO_RD_M1 = cnt4(ACT_TO_RD - 1);
localparam [3:0] ACT_TO_WR_M1 = cnt4(ACT_TO_WR - 1);
localparam [3:0] CL_M2        = cnt4(CL - 2);
localparam [3:0] BL_M1        = cnt4(BURST_LEN - 1);
localparam [3:0] RD_REC_M1    = cnt4(imax(RD_REC - 1, 0));
localparam [3:0] WR_REC_M1    = cnt4(imax(WR_REC - 1, 0));

// Mode register burst-length code (A2:A0).
localparam [2:0] BL_CODE = (BURST_LEN == 8) ? 3'b011 :
                           (BURST_LEN == 4) ? 3'b010 :
                           (BURST_LEN == 2) ? 3'b001 : 3'b000;

// STATES - State
localparam IDLE      = 5'b00000;

localparam INIT_NOP1 = 5'b01000,
           INIT_PRE1 = 5'b01001,
           INIT_NOP1_1=5'b00101,
           INIT_REF1 = 5'b01010,
           INIT_NOP2 = 5'b01011,
           INIT_REF2 = 5'b01100,
           INIT_NOP3 = 5'b01101,
           INIT_LOAD = 5'b01110,
           INIT_NOP4 = 5'b01111;

localparam REF_PRE  =  5'b00001,
           REF_NOP1 =  5'b00010,
           REF_REF  =  5'b00011,
           REF_NOP2 =  5'b00100;

// state[4] set = a read/write is in progress (bank/addr come from addr_r,
// dqm low, busy high).
localparam READ_ACT  = 5'b10000,  // ACT on the pins, then wait ACT_TO_RD
           RW_REC    = 5'b10001,  // precharge recovery before IDLE
           READ_CAS  = 5'b10010,  // READ (auto-precharge) on the pins
           READ_CL   = 5'b10011,  // CAS latency wait
           READ_DATA = 5'b10100;  // BURST_LEN beats captured

localparam WRIT_ACT  = 5'b11000,  // ACT on the pins, then wait ACT_TO_WR
           WRIT_CAS  = 5'b11010;  // one WRITE per cycle, data on the pins

// Commands              CCRCWBBA
//                       ESSSE100
localparam CMD_PALL = 8'b10010001,
           CMD_REF  = 8'b10001000,
           CMD_NOP  = 8'b10111000,
           CMD_MRS  = 8'b10000000,
           CMD_BACT = 8'b10011000,
           CMD_READ = 8'b10101001,
           CMD_WRIT = 8'b10100001;

/* Interface Definition */
/* HOST INTERFACE */
input  [HADDR_WIDTH-1:0]   wr_addr;
input  [DATA_WIDTH-1:0]    wr_data;
input                      wr_enable;
input                      wr_burst;
output                     wr_ack;
output                     wr_next;

input  [HADDR_WIDTH-1:0]   rd_addr;
output [DATA_WIDTH-1:0]    rd_data;
input                      rd_enable;
output                     rd_ready;
output                     rd_ack;

output                     busy;
input                      rst_n;
input                      clk;

/* SDRAM SIDE */
output [SDRADDR_WIDTH-1:0] addr;
output [BANK_WIDTH-1:0]    bank_addr;
inout  [DATA_WIDTH-1:0]    data;
output                     clock_enable;
output                     cs_n;
output                     ras_n;
output                     cas_n;
output                     we_n;
output [DQM_WIDTH-1:0]     dqm;

/* I/O Registers */

reg  [HADDR_WIDTH-1:0]   haddr_r;
reg  [DATA_WIDTH-1:0]    wr_data_r;
reg  [DATA_WIDTH-1:0]    rd_data_r;
reg  [DATA_WIDTH-1:0]    rd_data_neg;
reg                      busy;
reg  [DQM_WIDTH-1:0]     dqm_r;
reg [SDRADDR_WIDTH-1:0]  addr_r;
reg [BANK_WIDTH-1:0]     bank_addr_r;
reg                      rd_ready_r;
reg                      wr_burst_r;

wire [DQM_WIDTH-1:0]     dqm;

assign dqm       = dqm_r;
assign rd_data   = rd_data_r;

/* Internal Wiring */
reg [3:0] state_cnt;
reg [CBR_WIDTH-1:0] refresh_cnt;

reg [7:0] command;
reg [4:0] state;

// Read/write engine counter: wait cycles, then beats/words remaining - 1.
reg [3:0] rw_cnt;
reg [3:0] rw_cnt_nxt;
// Column offset of the WRITE being issued in a streamed write.
reg [COL_WIDTH-1:0] wcol;

reg [7:0] command_nxt;
reg [3:0] state_cnt_nxt;
reg [4:0] next;

wire refresh_due = (refresh_cnt >= CYCLES_BETWEEN_REFRESH_W);
wire idle_take   = (state == IDLE) & ~refresh_due;

assign rd_ack  = idle_take & rd_enable;
assign wr_ack  = idle_take & ~rd_enable & wr_enable;
assign wr_next = (state == WRIT_CAS) & (rw_cnt != 4'd0);

assign {clock_enable, cs_n, ras_n, cas_n, we_n} = command[7:3];
// state[4] will be set if mode is read/write
assign bank_addr      = (state[4]) ? bank_addr_r : command[2:1];

// Command (non read/write) address: A10 = command[0], all other bits zero.
// patched: built by bit assignment (upstream used a zero-width replication
// count that breaks at SDRADDR_WIDTH 11)
reg [SDRADDR_WIDTH-1:0] cmd_addr_r;
always @*
  begin
  cmd_addr_r = {SDRADDR_WIDTH{1'b0}};
  cmd_addr_r[10] = command[0];
  end

assign addr           = (state[4] | state == INIT_LOAD) ? addr_r : cmd_addr_r;

assign data = (state == WRIT_CAS) ? wr_data_r : {DATA_WIDTH{1'bz}};
assign rd_ready = rd_ready_r;

// HOST INTERFACE
// Read data is captured on the FALLING edge of clk, then handed to the
// posedge pipeline. With the SDRAM clocked 180 degrees from clk (the board:
// rPLL CLKOUTP, PSDA_SEL "1000"), a burst word is valid from ~tAC after the
// device edge (= clk fall) to ~tOH after the next one, i.e. around the clk
// fall one cycle later; the posedge after it already sees the NEXT word
// (board, 2026-11-27: every read one burst slot late). With an in-phase
// device clock (the default sim) the word is valid a whole cycle across the
// same fall. One capture point works for both.
always @ (negedge clk)
  rd_data_neg <= data;

// all registered on posedge
always @ (posedge clk)
  if (~rst_n)
    begin
    state <= INIT_NOP1;
    command <= CMD_NOP;
    state_cnt <= 4'hf;
    rw_cnt <= 4'd0;
    wcol <= {COL_WIDTH{1'b0}};

    haddr_r <= {HADDR_WIDTH{1'b0}};
    wr_data_r <= {DATA_WIDTH{1'b0}};
    rd_data_r <= {DATA_WIDTH{1'b0}};
    rd_ready_r <= 1'b0;
    wr_burst_r <= 1'b0;
    busy <= 1'b0;
    end
  else
    begin

    state <= next;
    command <= command_nxt;
    rw_cnt <= rw_cnt_nxt;

    if (state_cnt == 4'd0)
      state_cnt <= state_cnt_nxt;
    else
      state_cnt <= state_cnt - 1'b1;

    // Write data: word 0 at the accept, word k at the edge ending the
    // cycle that issued WRITE k-1.
    if (wr_ack | wr_next)
      wr_data_r <= wr_data;

    if (state == WRIT_CAS)
      wcol <= wcol + 1'b1;
    else
      wcol <= {COL_WIDTH{1'b0}};

    if (state == READ_DATA)
      begin
      rd_data_r <= rd_data_neg;
      rd_ready_r <= 1'b1;
      end
    else
      rd_ready_r <= 1'b0;

    busy <= state[4];

    if (rd_ack)
      haddr_r <= rd_addr;
    else if (wr_ack)
      begin
      haddr_r <= wr_addr;
      wr_burst_r <= wr_burst;
      end

    end

// Handle refresh counter
always @ (posedge clk)
  if (~rst_n)
    refresh_cnt <= {CBR_WIDTH{1'b0}};
  else
    if (state == REF_NOP2)
      refresh_cnt <= {CBR_WIDTH{1'b0}};
    else
      refresh_cnt <= refresh_cnt + 1'b1;


/* Handle logic for sending addresses to SDRAM based on current state*/
always @*
begin
    if (state[4])
      dqm_r = {DQM_WIDTH{1'b0}};
    else
      dqm_r = {DQM_WIDTH{1'b1}};

   bank_addr_r = haddr_r[HADDR_WIDTH-1:HADDR_WIDTH-(BANK_WIDTH)];
   addr_r = {SDRADDR_WIDTH{1'b0}};

   if (state == READ_ACT | state == WRIT_ACT)
     begin
     // Row address
     addr_r[ROW_WIDTH-1:0] = haddr_r[HADDR_WIDTH-(BANK_WIDTH+1):HADDR_WIDTH-(BANK_WIDTH+ROW_WIDTH)];
     end
   else if (state == READ_CAS)
     begin
     // Column of the burst start, A10 = auto-precharge.
     addr_r[10] = 1'b1;
     addr_r[COL_WIDTH-1:0] = haddr_r[COL_WIDTH-1:0];
     end
   else if (state == WRIT_CAS)
     begin
     // Consecutive columns of the open row; auto-precharge on the last.
     addr_r[10] = (rw_cnt == 4'd0);
     addr_r[COL_WIDTH-1:0] = haddr_r[COL_WIDTH-1:0] + wcol;
     end
   else if (state == INIT_LOAD)
     begin
     // Program mode register during load cycle
     //   A9 = 1  write burst mode: single location
     //   A6:4    CAS latency 3
     //   A3 = 0  sequential burst
     //   A2:0    burst length (BL_CODE)
     addr_r[9:0] = {7'b1000110, BL_CODE};
     end
end

// Next state logic
always @*
begin
   state_cnt_nxt = 4'd0;
   rw_cnt_nxt = rw_cnt;
   command_nxt = CMD_NOP;
   next = state;
   if (state == IDLE)
        // Monitor for refresh or hold
        if (refresh_due)
          begin
          next = REF_PRE;
          command_nxt = CMD_PALL;
          end
        else if (rd_enable)
          begin
          next = READ_ACT;
          command_nxt = CMD_BACT;
          rw_cnt_nxt = ACT_TO_RD_M1;
          end
        else if (wr_enable)
          begin
          next = WRIT_ACT;
          command_nxt = CMD_BACT;
          rw_cnt_nxt = ACT_TO_WR_M1;
          end
        else
          begin
          // HOLD
          next = IDLE;
          end
   else if (state[4])
      // READ / WRITE engine (rw_cnt based)
      case (state)
        READ_ACT:
          if (rw_cnt == 4'd0)
            begin
            next = READ_CAS;
            command_nxt = CMD_READ;
            end
          else
            rw_cnt_nxt = rw_cnt - 1'b1;
        READ_CAS:
          begin
          next = READ_CL;
          rw_cnt_nxt = CL_M2;
          end
        READ_CL:
          if (rw_cnt == 4'd0)
            begin
            next = READ_DATA;
            rw_cnt_nxt = BL_M1;
            end
          else
            rw_cnt_nxt = rw_cnt - 1'b1;
        READ_DATA:
          if (rw_cnt == 4'd0)
            begin
            if (RD_REC > 0)
              begin
              next = RW_REC;
              rw_cnt_nxt = RD_REC_M1;
              end
            else
              next = IDLE;
            end
          else
            rw_cnt_nxt = rw_cnt - 1'b1;
        WRIT_ACT:
          if (rw_cnt == 4'd0)
            begin
            next = WRIT_CAS;
            command_nxt = CMD_WRIT;
            rw_cnt_nxt = wr_burst_r ? BL_M1 : 4'd0;
            end
          else
            rw_cnt_nxt = rw_cnt - 1'b1;
        WRIT_CAS:
          if (rw_cnt == 4'd0)
            begin
            if (WR_REC > 0)
              begin
              next = RW_REC;
              rw_cnt_nxt = WR_REC_M1;
              end
            else
              next = IDLE;
            end
          else
            begin
            command_nxt = CMD_WRIT;
            rw_cnt_nxt = rw_cnt - 1'b1;
            end
        RW_REC:
          if (rw_cnt == 4'd0)
            next = IDLE;
          else
            rw_cnt_nxt = rw_cnt - 1'b1;
        default:
          next = IDLE;
      endcase
    else
      if (state_cnt == 4'd0)
        case (state)
          // INIT ENGINE
          INIT_NOP1:
            begin
            next = INIT_PRE1;
            command_nxt = CMD_PALL;
            end
          INIT_PRE1:
            begin
            next = INIT_NOP1_1;
            end
          INIT_NOP1_1:
            begin
            next = INIT_REF1;
            command_nxt = CMD_REF;
            end
          INIT_REF1:
            begin
            next = INIT_NOP2;
            state_cnt_nxt = 4'd7;
            end
          INIT_NOP2:
            begin
            next = INIT_REF2;
            command_nxt = CMD_REF;
            end
          INIT_REF2:
            begin
            next = INIT_NOP3;
            state_cnt_nxt = 4'd7;
            end
          INIT_NOP3:
            begin
            next = INIT_LOAD;
            command_nxt = CMD_MRS;
            end
          INIT_LOAD:
            begin
            next = INIT_NOP4;
            state_cnt_nxt = 4'd1;
            end
          // INIT_NOP4: default - IDLE

          // REFRESH
          REF_PRE:
            begin
            next = REF_NOP1;
            end
          REF_NOP1:
            begin
            next = REF_REF;
            command_nxt = CMD_REF;
            end
          REF_REF:
            begin
            next = REF_NOP2;
            state_cnt_nxt = 4'd7;
            end
          // REF_NOP2: default - IDLE

          default:
            begin
            next = IDLE;
            end
          endcase
      else
        begin
        // Counter Not Reached - HOLD
        next = state;
        command_nxt = command;
        end
end

endmodule

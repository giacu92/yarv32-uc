// Verilator C++ BFM for the axi4_lite_i2c compliance test.
//
// Models a real I2C slave device at address 0x50 with a 16-byte register
// file and an auto-incrementing pointer: it detects START / repeated-
// START / STOP on the open-drain bus, shifts the address byte, ACKs on a
// match, accepts write bytes (first byte = register pointer, rest = data)
// and shifts out read bytes MSB-first, stopping on a master NACK. This is
// the device the example library in the testbench header talks to.
//
// The bus is driven through the wrapper's 2-state wired-AND pull-up model:
// the slave pulls SDA low by asserting slave_sda_oe (ACK, or a data 0) and
// releases it otherwise. SDA is sampled on SCL rising, driven on SCL
// falling -- the I2C data-change-only-while-SCL-low rule.
//
// Build: make   (in sim/hw/i2c_tb/)
// Run:   make run

#include "Vi2c_tb.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>

// Register offsets (byte addresses within the peripheral window).
static const uint32_t REG_STATUS = 0x00;
static const uint32_t REG_CTRL   = 0x04;
static const uint32_t REG_CLKDIV = 0x08;
static const uint32_t REG_CMD    = 0x0C;
static const uint32_t REG_TXDATA = 0x10;
static const uint32_t REG_RXDATA = 0x14;
static const uint32_t REG_IRQ    = 0x18;

static const uint32_t ST_BUSY       = 0x001;
static const uint32_t ST_TX_FULL    = 0x002;
static const uint32_t ST_RX_EMPTY   = 0x004;
static const uint32_t ST_TX_EMPTY   = 0x008;
static const uint32_t ST_RX_FULL    = 0x010;
static const uint32_t ST_NACK       = 0x020;
static const uint32_t ST_ARBLOST    = 0x040;
static const uint32_t ST_RX_OVERRUN = 0x080;

static const uint32_t CTRL_ENABLE = 0x01;
static const uint32_t CTRL_IE_TX  = 0x02;
static const uint32_t CTRL_IE_RX  = 0x04;
static const uint32_t CTRL_IE_DONE = 0x08;
static const uint32_t CTRL_IE_NACK = 0x10;
static const uint32_t CTRL_IE_ARB  = 0x20;
static const uint32_t CTRL_IE_OVR  = 0x40;

static const uint32_t CMD_START = 0x1;
static const uint32_t CMD_STOP  = 0x2;
static const uint32_t CMD_READ  = 0x4;
static const uint32_t CMD_NACK  = 0x8;

static const uint32_t IRQ_TX_EMPTY = 0x01;
static const uint32_t IRQ_RX_NE    = 0x02;
static const uint32_t IRQ_DONE     = 0x04;
static const uint32_t IRQ_NACK     = 0x08;
static const uint32_t IRQ_ARB      = 0x10;
static const uint32_t IRQ_OVR      = 0x20;

static const uint32_t CLKDIV = 2;       // SCL = clk / (4*(CLKDIV+1)) = clk/12
static const int FIFO_DEPTH = 16;
static const uint8_t SLAVE_ADDR = 0x50;

static Vi2c_tb* top;
static int fails = 0;
static int checks = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        ++checks;                                                         \
        if (!(cond)) {                                                    \
            printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg);        \
            ++fails;                                                      \
        }                                                                 \
    } while (0)

#define CHECK_EQ(got, want, msg)                                          \
    do {                                                                  \
        ++checks;                                                         \
        if ((uint32_t)(got) != (uint32_t)(want)) {                        \
            printf("FAIL [%s:%d]: %s (got 0x%x, want 0x%x)\n",            \
                   __FILE__, __LINE__, msg, (unsigned)(got),              \
                   (unsigned)(want));                                     \
            ++fails;                                                      \
        }                                                                 \
    } while (0)

// ---------------------------------------------------------------------
// I2C slave device model
// ---------------------------------------------------------------------
struct I2cSlave {
    enum St { IDLE, ADDR, WR, RD };
    St st = IDLE;
    int bit_cnt = 0;          // 0..7 data bits, 8 = ACK slot
    uint8_t shift = 0;
    bool is_read = false;
    bool addr_match = false;
    bool first_wr_byte = true;
    int reg_ptr = 0;
    bool master_ack = false;  // sampled on a read ACK slot
    uint8_t tx_byte = 0;      // byte being shifted out during RD
    int sda_drive = 1;        // 1 = release (high), 0 = pull low
    int scl_prev = 1, sda_prev = 1;
    uint8_t regs[16] = {0};

    void init() {
        for (int i = 0; i < 16; ++i) regs[i] = (uint8_t)(0x10 + i);
        st = IDLE;
        sda_drive = 1;
        scl_prev = 1;
        sda_prev = 1;
    }

    void step() {
        int scl = top->scl_line;
        int sda = top->sda_line;
        bool scl_r = (scl == 1 && scl_prev == 0);
        bool scl_f = (scl == 0 && scl_prev == 1);
        bool sda_f = (sda == 0 && sda_prev == 1);
        bool sda_r = (sda == 1 && sda_prev == 0);
        scl_prev = scl;
        sda_prev = sda;

        // START / repeated-START / STOP are recognised while SCL is high.
        if (scl == 1) {
            if (sda_f) {
                // START (or repeated-START)
                st = ADDR;
                bit_cnt = 0;
                shift = 0;
                addr_match = false;
                first_wr_byte = true;
                sda_drive = 1;
            } else if (sda_r && st != IDLE) {
                st = IDLE;
                sda_drive = 1;
            }
        }

        if (st == IDLE) {
            top->slave_sda_oe = 0;
            top->slave_scl_oe = 0;
            return;
        }

        if (scl_r) {
            if (bit_cnt < 8) {
                if (st == ADDR || st == WR) {
                    shift = (uint8_t)((shift << 1) | sda);
                }
                ++bit_cnt;
            } else if (bit_cnt == 8) {
                // ACK slot rising: on a read, the master's ACK/NACK is on SDA.
                if (st == RD) master_ack = (sda == 1);
                ++bit_cnt;
            }
        } else if (scl_f) {
            if (bit_cnt <= 7) {
                // Prepare the next read data bit (stable before next SCL rise).
                if (st == RD && bit_cnt >= 1) {
                    sda_drive = (tx_byte >> (7 - bit_cnt)) & 1;
                }
            } else if (bit_cnt == 8) {
                // ACK slot: slave drives SDA.
                if (st == ADDR) {
                    addr_match = ((shift >> 1) == SLAVE_ADDR);
                    is_read = shift & 1;
                    sda_drive = addr_match ? 0 : 1;  // 0 = pull low = ACK
                } else if (st == WR) {
                    sda_drive = 0;  // ACK every accepted write byte
                } else if (st == RD) {
                    sda_drive = 1;  // release SDA so the master can drive ACK
                }
            } else if (bit_cnt == 9) {
                // Post-ACK transition.
                sda_drive = 1;
                bit_cnt = 0;
                if (st == ADDR) {
                    if (!addr_match) {
                        st = IDLE;
                    } else if (is_read) {
                        st = RD;
                        tx_byte = regs[reg_ptr & 0xF];
                        sda_drive = (tx_byte >> 7) & 1;  // present bit 7
                    } else {
                        st = WR;
                    }
                } else if (st == WR) {
                    if (first_wr_byte) {
                        reg_ptr = shift;
                        first_wr_byte = false;
                    } else {
                        regs[reg_ptr & 0xF] = shift;
                        reg_ptr++;
                    }
                    // stay in WR for the next byte (or a STOP / rep-START)
                } else if (st == RD) {
                    if (!master_ack) {
                        reg_ptr++;
                        tx_byte = regs[reg_ptr & 0xF];
                        sda_drive = (tx_byte >> 7) & 1;  // present bit 7
                    } else {
                        st = IDLE;  // master NACK: it will issue STOP
                    }
                }
            }
        }

        top->slave_sda_oe = (sda_drive == 0) ? 1 : 0;
        top->slave_scl_oe = 0;  // no clock stretching in this test
    }
};

static I2cSlave slave;

static void tick() {
    top->clk_i = 0;
    top->eval();
    top->clk_i = 1;
    top->eval();
    slave.step();
}

static void idle() {
    top->awvalid = 0;
    top->wvalid = 0;
    top->arvalid = 0;
    top->bready = 1;
    top->rready = 1;
    top->eval();
}

static void axi_write(uint32_t addr, uint32_t data, int guard = 128) {
    top->awaddr = addr;
    top->awvalid = 1;
    top->wdata = data;
    top->wstrb = 0xF;
    top->wvalid = 1;
    top->bready = 1;
    bool aw = false, w = false, b = false;
    for (int g = 0; g < guard && !(aw && w && b); ++g) {
        top->eval();
        if (top->awvalid && top->awready) aw = true;
        if (top->wvalid && top->wready) w = true;
        if (top->bvalid && top->bready) b = true;
        tick();
        if (aw) top->awvalid = 0;
        if (w) top->wvalid = 0;
    }
    CHECK(aw && w && b, "axi_write did not complete");
    idle();
}

static uint32_t axi_read(uint32_t addr, int guard = 128) {
    top->araddr = addr;
    top->arvalid = 1;
    top->rready = 1;
    uint32_t val = 0;
    bool ar = false, r = false;
    for (int g = 0; g < guard && !(ar && r); ++g) {
        top->eval();
        if (top->arvalid && top->arready) ar = true;
        if (top->rvalid && top->rready) {
            val = top->rdata;
            r = true;
        }
        tick();
        if (ar) top->arvalid = 0;
    }
    CHECK(ar && r, "axi_read did not complete");
    idle();
    return val;
}

static bool wait_idle(int timeout = 20000) {
    for (int i = 0; i < timeout; ++i) {
        if (!(axi_read(REG_STATUS) & ST_BUSY)) return true;
    }
    return false;
}

// Wait for BUSY clear, then clear the DONE sticky IRQ (common tail).
static void finish_op() {
    CHECK(wait_idle(), "transaction did not complete (BUSY stuck)");
    axi_write(REG_IRQ, IRQ_DONE);
}

// --- example library (matches the testbench header) -----------------

// Write one register: addr+W, reg, val, START+STOP.
static void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    axi_write(REG_TXDATA, (uint32_t)(addr << 1) | 0);
    axi_write(REG_TXDATA, reg);
    axi_write(REG_TXDATA, val);
    axi_write(REG_CMD, CMD_START | CMD_STOP);
    finish_op();
}

// Read one register: addr+W reg (no STOP), repeated-START addr+R (no STOP),
// then one CMD=READ|NACK|STOP for the data byte.
static uint8_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    axi_write(REG_TXDATA, (uint32_t)(addr << 1) | 0);
    axi_write(REG_TXDATA, reg);
    axi_write(REG_CMD, CMD_START);  // write, no STOP (bus stays active)
    finish_op();

    axi_write(REG_TXDATA, (uint32_t)(addr << 1) | 1);
    axi_write(REG_CMD, CMD_START);  // repeated-START, write addr+R, no STOP
    finish_op();

    axi_write(REG_CMD, CMD_READ | CMD_NACK | CMD_STOP);
    finish_op();

    return (uint8_t)(axi_read(REG_RXDATA) & 0xFF);
}

// Write a burst: addr+W, reg, data[0..n-1], START+STOP.
static void i2c_write_burst(uint8_t addr, uint8_t reg, const uint8_t* data,
                            int n) {
    axi_write(REG_TXDATA, (uint32_t)(addr << 1) | 0);
    axi_write(REG_TXDATA, reg);
    for (int i = 0; i < n; ++i) axi_write(REG_TXDATA, data[i]);
    axi_write(REG_CMD, CMD_START | CMD_STOP);
    finish_op();
}

// Read n bytes: addr+W reg, repeated-START addr+R, then n-1 CMD=READ (master
// ACK, continue) and a final CMD=READ|NACK|STOP. Returns bytes via out[].
static void i2c_read_burst(uint8_t addr, uint8_t reg, uint8_t* out, int n) {
    axi_write(REG_TXDATA, (uint32_t)(addr << 1) | 0);
    axi_write(REG_TXDATA, reg);
    axi_write(REG_CMD, CMD_START);
    finish_op();

    axi_write(REG_TXDATA, (uint32_t)(addr << 1) | 1);
    axi_write(REG_CMD, CMD_START);
    finish_op();

    for (int i = 0; i < n - 1; ++i) {
        axi_write(REG_CMD, CMD_READ);  // master ACKs, continues
        finish_op();
    }
    axi_write(REG_CMD, CMD_READ | CMD_NACK | CMD_STOP);
    finish_op();

    for (int i = 0; i < n; ++i) out[i] = (uint8_t)(axi_read(REG_RXDATA) & 0xFF);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vi2c_tb;

    top->clk_i = 0;
    top->rstn_i = 0;
    top->slave_scl_oe = 0;
    top->slave_sda_oe = 0;
    idle();
    slave.init();
    for (int i = 0; i < 4; ++i) tick();
    top->rstn_i = 1;
    tick();

    printf("=== axi4_lite_i2c master + slave-device compliance ===\n");

    // ---- 1. Reset state ------------------------------------------------
    uint32_t st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_BUSY, 0, "BUSY clear out of reset");
    CHECK_EQ(st & ST_RX_EMPTY, ST_RX_EMPTY, "RX_EMPTY set out of reset");
    CHECK_EQ(st & ST_TX_EMPTY, ST_TX_EMPTY, "TX_EMPTY set out of reset");
    CHECK_EQ(top->i2c_irq_o, 0, "IRQ masked out of reset (IEs are 0)");

    // Enable the master + set the divisor.
    axi_write(REG_CTRL, CTRL_ENABLE);
    axi_write(REG_CLKDIV, CLKDIV);

    // ---- 2. write_reg / read_reg round trip ----------------------------
    // The example library: write 0xAB to reg 3, read it back.
    i2c_write_reg(SLAVE_ADDR, 0x03, 0xAB);
    CHECK_EQ(slave.regs[0x03], 0xAB, "slave stored the written register");
    uint8_t got = i2c_read_reg(SLAVE_ADDR, 0x03);
    CHECK_EQ(got, 0xAB, "read_reg returned the written value");

    // ---- 3. write / read burst (pointer auto-increment) ----------------
    const uint8_t wbuf[4] = {0xC0, 0xC1, 0xC2, 0xC3};
    i2c_write_burst(SLAVE_ADDR, 0x00, wbuf, 4);
    for (int i = 0; i < 4; ++i) {
        CHECK_EQ(slave.regs[i], wbuf[i], "burst write stored in order");
    }
    uint8_t rbuf[4] = {0};
    i2c_read_burst(SLAVE_ADDR, 0x00, rbuf, 4);
    for (int i = 0; i < 4; ++i) {
        CHECK_EQ(rbuf[i], wbuf[i], "burst read returned in order");
    }

    // ---- 4. NACK on a write to an absent slave -------------------------
    // Address 0x51 does not ACK; the master must report NACK and STOP.
    axi_write(REG_TXDATA, (uint32_t)(0x51 << 1) | 0);
    axi_write(REG_TXDATA, 0x00);
    axi_write(REG_TXDATA, 0xFF);
    axi_write(REG_CMD, CMD_START | CMD_STOP);
    finish_op();
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_NACK, ST_NACK, "NACK latched when slave does not ACK");
    uint32_t irq = axi_read(REG_IRQ);
    CHECK_EQ(irq & IRQ_NACK, IRQ_NACK, "IRQ register exposes NACK bit");
    axi_write(REG_IRQ, IRQ_NACK);  // W1C
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_NACK, 0, "NACK cleared by IRQ W1C");
    // The absent slave received nothing: reg 0 stays as written above.
    CHECK_EQ(slave.regs[0], 0xC0, "NACKed write did not touch the slave");

    // ---- 5. RX overrun -------------------------------------------------
    // Read more bytes than the RX FIFO depth without draining: the overflow
    // byte is dropped and RX_OVERRUN latches; the queued bytes survive.
    // Re-init the slave so reg 0 onwards holds known contents 0x10..0x1F
    // (tests 2-3 overwrote them) and the pointer read starts at reg 0.
    slave.init();
    uint8_t big[20] = {0};
    i2c_read_burst(SLAVE_ADDR, 0x00, big, FIFO_DEPTH + 1);
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_RX_OVERRUN, ST_RX_OVERRUN, "RX_OVERRUN latched on drop");
    for (int i = 0; i < FIFO_DEPTH; ++i) {
        CHECK_EQ(big[i], (uint8_t)(0x10 + i),
                 "overrun: queued read byte survived");
    }
    axi_write(REG_IRQ, IRQ_OVR);  // W1C clears the sticky overrun flag
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_RX_OVERRUN, 0, "RX_OVERRUN cleared by IRQ W1C");
    st = axi_read(REG_STATUS);
    CHECK_EQ(st & ST_RX_EMPTY, ST_RX_EMPTY, "RX empty after the overrun drain");

    // ---- 6. IRQ: level-sensitive, gated by CTRL ------------------------
    // IE_RX: a completed read leaves a byte pending; IRQ asserts with IE_RX.
    // Do the read without draining RXDATA so the level IRQ is observable.
    axi_write(REG_CTRL, CTRL_ENABLE | CTRL_IE_RX);
    CHECK_EQ(top->i2c_irq_o, 0, "no RX IRQ with RX empty");
    axi_write(REG_TXDATA, (uint32_t)(SLAVE_ADDR << 1) | 0);
    axi_write(REG_TXDATA, 0x00);
    axi_write(REG_CMD, CMD_START);
    finish_op();
    axi_write(REG_TXDATA, (uint32_t)(SLAVE_ADDR << 1) | 1);
    axi_write(REG_CMD, CMD_START);
    finish_op();
    axi_write(REG_CMD, CMD_READ | CMD_NACK | CMD_STOP);
    finish_op();
    CHECK_EQ(top->i2c_irq_o, 1, "RX IRQ asserts with RX not empty + IE_RX");
    (void)axi_read(REG_RXDATA);
    CHECK_EQ(top->i2c_irq_o, 0, "RX IRQ deasserts once RX is drained");
    axi_write(REG_CTRL, 0);
    CHECK_EQ(top->i2c_irq_o, 0, "IRQ masked with all IEs clear");

    printf("%d checks, %d failures\n", checks, fails);
    delete top;
    return fails ? 1 : 0;
}
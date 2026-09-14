#ifndef I2C_H
#define I2C_H

/*
 * Polling driver for the axi4_lite_i2c master peripheral
 * (src/rtl/utils/axi4_lite_i2c.sv) at rv32_pkg::I2C_BASE = 0x1000_5000.
 *
 * Register map (word offsets from the peripheral base):
 *   0x00 STATUS (R)   : BUSY / TX_FULL / RX_EMPTY / TX_EMPTY / RX_FULL /
 *                       NACK(sticky) / ARBLOST(sticky) / RX_OVERRUN(sticky)
 *   0x04 CTRL   (RW)  : ENABLE + 6 interrupt-enable bits
 *   0x08 CLKDIV (RW)  : SCL ~ clk / (4*(CLKDIV+1))
 *   0x0C CMD    (W)   : START / STOP / READ / NACK
 *   0x10 TXDATA (W)   : push one byte (LSB) into the TX FIFO
 *   0x14 RXDATA (R)   : pop one byte from the RX FIFO
 *   0x18 IRQ    (R/W1C)
 *
 * Transaction model (one CMD write starts one operation):
 *   * A write op drains the whole TX FIFO (MSB first), so long writes are
 *     fed by pushing bytes while the engine runs (i2c_push polls TX_FULL).
 *   * The address byte is always master-WRITTEN, even for a read: a read is
 *     a write op carrying addr|R (which leaves the bus active, no STOP),
 *     then one CMD=READ per data byte, NACK|STOP on the last.
 *   * A register read is: write [addr+W, reg] with no STOP, repeated-START
 *     write [addr+R], then the per-byte reads (i2c_read_regs).
 *
 * All busy-waits are infinite, like uart.h -- a slave that stretches SCL
 * forever will hang the caller. Call i2c_init() once before anything else;
 * the engine ignores CMD while CTRL.ENABLE is 0.
 *
 * Error model: NACK and ARBLOST are sticky STATUS bits; every transaction
 * clears them first and checks them after, returning I2C_ERR_*. A NACKed
 * write aborts to STOP and the engine drains the rest of the TX FIFO, so a
 * failed transaction never leaves bytes queued for the next one.
 */

#define I2C_BASE 0x10005000u

#define I2C_STATUS (*(volatile unsigned int *)(I2C_BASE + 0x00))
#define I2C_CTRL   (*(volatile unsigned int *)(I2C_BASE + 0x04))
#define I2C_CLKDIV (*(volatile unsigned int *)(I2C_BASE + 0x08))
#define I2C_CMD    (*(volatile unsigned int *)(I2C_BASE + 0x0C))
#define I2C_TXDATA (*(volatile unsigned int *)(I2C_BASE + 0x10))
#define I2C_RXDATA (*(volatile unsigned int *)(I2C_BASE + 0x14))
#define I2C_IRQ    (*(volatile unsigned int *)(I2C_BASE + 0x18))

/* STATUS bits */
#define I2C_STATUS_BUSY       0x01u
#define I2C_STATUS_TX_FULL    0x02u
#define I2C_STATUS_RX_EMPTY   0x04u
#define I2C_STATUS_TX_EMPTY   0x08u
#define I2C_STATUS_RX_FULL    0x10u
#define I2C_STATUS_NACK       0x20u  /* sticky: slave NACKed a write byte */
#define I2C_STATUS_ARBLOST    0x40u  /* sticky: lost arbitration */
#define I2C_STATUS_RX_OVERRUN 0x80u  /* sticky: read byte dropped, RX full */

/* CTRL bits */
#define I2C_CTRL_ENABLE 0x01u
#define I2C_CTRL_IE_TX  0x02u
#define I2C_CTRL_IE_RX  0x04u
#define I2C_CTRL_IE_ALL 0x7Eu

/* CMD bits */
#define I2C_CMD_START 0x01u
#define I2C_CMD_STOP  0x02u
#define I2C_CMD_READ  0x04u
#define I2C_CMD_NACK  0x08u

/* IRQ W1C bits */
#define I2C_IRQ_TX_EMPTY   0x01u
#define I2C_IRQ_RX_NEMPTY  0x02u
#define I2C_IRQ_DONE       0x04u
#define I2C_IRQ_NACK       0x08u
#define I2C_IRQ_ARBLOST    0x10u
#define I2C_IRQ_RX_OVERRUN 0x20u
#define I2C_IRQ_ALL        0x3Fu

/* Transaction return codes */
#define I2C_OK       0
#define I2C_ERR_NACK (-1)
#define I2C_ERR_ARB  (-2)

/* Core clock feeding CLKDIV, used by the *_hz convenience wrappers.
 * Must track the rPLL (50 MHz shipping). */
#ifndef I2C_CORE_HZ
#define I2C_CORE_HZ 50000000u
#endif

/* ------------------------------------------------------------------ */
/* Init / configuration                                                */
/* ------------------------------------------------------------------ */

/* CLKDIV for a target SCL: SCL ~ core / (4*(div+1)). Rounded down, so the
 * real SCL is at or slightly above the request. */
static unsigned int i2c_clkdiv_for(unsigned int core_hz, unsigned int scl_hz)
{
    unsigned int d = core_hz / (4u * scl_hz);
    return d ? d - 1u : 0u;
}

/* Enable the engine at a fixed divider value. */
static void i2c_set_clkdiv(unsigned int div)
{
    I2C_CLKDIV = div;
}

/* Enable the engine at a target SCL frequency (i2c_clkdiv_for). */
static void i2c_begin(unsigned int core_hz, unsigned int scl_hz)
{
    I2C_CTRL  = I2C_CTRL_ENABLE;  /* no interrupts: pure polling */
    I2C_CLKDIV = i2c_clkdiv_for(core_hz, scl_hz);
    I2C_IRQ   = I2C_IRQ_ALL;  /* clear any sticky flags */
}

/* Shorthand at the compiled-in core clock: i2c_begin_hz(100000) etc. */
static void i2c_begin_hz(unsigned int scl_hz)
{
    i2c_begin(I2C_CORE_HZ, scl_hz);
}

/* ------------------------------------------------------------------ */
/* Low-level primitives (compose your own transactions)                */
/* ------------------------------------------------------------------ */

static int i2c_busy(void)
{
    return (I2C_STATUS & I2C_STATUS_BUSY) != 0u;
}

static void i2c_wait_idle(void)
{
    while (I2C_STATUS & I2C_STATUS_BUSY)
        ;
}

/* Push one byte, waiting for TX space. Safe while the engine drains. */
static void i2c_push(unsigned char b)
{
    while (I2C_STATUS & I2C_STATUS_TX_FULL)
        ;
    I2C_TXDATA = (unsigned int)b;
}

/* Issue a command. Bits: I2C_CMD_{START,STOP,READ,NACK}. */
static void i2c_cmd(unsigned int cmd)
{
    I2C_CMD = cmd;
}

/* Pop one received byte (0 if the RX FIFO is empty). */
static unsigned char i2c_pop(void)
{
    return (unsigned char)(I2C_RXDATA & 0xFFu);
}

/* Read and return the sticky error bits as an I2C_ERR_* code. */
static int i2c_check_err(void)
{
    unsigned int st = I2C_STATUS;
    if (st & I2C_STATUS_ARBLOST)
        return I2C_ERR_ARB;
    if (st & I2C_STATUS_NACK)
        return I2C_ERR_NACK;
    return I2C_OK;
}

/* Clear the sticky error flags before starting a transaction. */
static void i2c_clear_err(void)
{
    I2C_IRQ = I2C_IRQ_NACK | I2C_IRQ_ARBLOST;
}

/* Release the bus after a no-STOP operation: START+STOP with an empty TX
 * FIFO transmits no byte, so this is a clean bus release. */
static void i2c_stop(void)
{
    I2C_CMD = I2C_CMD_START | I2C_CMD_STOP;
    i2c_wait_idle();
}

/* ------------------------------------------------------------------ */
/* Whole transactions (START ... STOP, blocking)                       */
/* ------------------------------------------------------------------ */

/* Master write: START, addr+W, buf[0..len-1], STOP.
 * Any length; the FIFO is fed while the engine drains. */
static int i2c_write(unsigned char addr7, const unsigned char *buf, unsigned int len)
{
    i2c_clear_err();
    i2c_push((unsigned char)(addr7 << 1));  /* addr + W */
    I2C_CMD = I2C_CMD_START | I2C_CMD_STOP;
    for (unsigned int i = 0; i < len; ++i)
        i2c_push(buf[i]);
    i2c_wait_idle();
    return i2c_check_err();
}

/* Master read: START, addr+R (master-written), then one READ per byte with
 * NACK|STOP on the last, as the engine requires. */
static int i2c_read(unsigned char addr7, unsigned char *buf, unsigned int len)
{
    int rc;

    i2c_clear_err();
    i2c_push((unsigned char)((addr7 << 1) | 1u));  /* addr + R */
    I2C_CMD = I2C_CMD_START;  /* no STOP: bus stays active for the reads */
    i2c_wait_idle();
    rc = i2c_check_err();
    if (rc != I2C_OK)
        return rc;

    for (unsigned int i = 0; i < len; ++i) {
        unsigned int last = (i == len - 1u) ? 1u : 0u;
        I2C_CMD = I2C_CMD_READ | (last ? (I2C_CMD_NACK | I2C_CMD_STOP) : 0u);
        i2c_wait_idle();
        buf[i] = i2c_pop();
    }
    if (len == 0u)
        i2c_stop();  /* nothing to read: release the bus anyway */
    return i2c_check_err();
}

/* Presence probe: address-only write. Returns I2C_OK if the slave ACKed. */
static int i2c_probe(unsigned char addr7)
{
    return i2c_write(addr7, 0, 0);
}

/* ------------------------------------------------------------------ */
/* 8-bit-register device helpers (sensors, RTCs, EEPROMs, ...)         */
/* ------------------------------------------------------------------ */

/* Write one register: START, addr+W, reg, val, STOP. */
static int i2c_write_reg(unsigned char addr7, unsigned char reg, unsigned char val)
{
    unsigned char b[2];
    b[0] = reg;
    b[1] = val;
    return i2c_write(addr7, b, 2);
}

/* Write len consecutive registers: START, addr+W, reg, buf..., STOP. */
static int i2c_write_regs(unsigned char addr7, unsigned char reg,
                          const unsigned char *buf, unsigned int len)
{
    i2c_clear_err();
    i2c_push((unsigned char)(addr7 << 1));
    i2c_push(reg);
    I2C_CMD = I2C_CMD_START | I2C_CMD_STOP;
    for (unsigned int i = 0; i < len; ++i)
        i2c_push(buf[i]);
    i2c_wait_idle();
    return i2c_check_err();
}

/* Read len consecutive registers:
 * START, addr+W, reg (no STOP), repeated-START, addr+R, data..., STOP. */
static int i2c_read_regs(unsigned char addr7, unsigned char reg,
                         unsigned char *buf, unsigned int len)
{
    int rc;

    i2c_clear_err();
    i2c_push((unsigned char)(addr7 << 1));  /* addr + W */
    i2c_push(reg);
    I2C_CMD = I2C_CMD_START;  /* no STOP: hold the bus for the repeat */
    i2c_wait_idle();
    rc = i2c_check_err();
    if (rc != I2C_OK)
        return rc;

    i2c_push((unsigned char)((addr7 << 1) | 1u));  /* addr + R */
    I2C_CMD = I2C_CMD_START;  /* repeated-START */
    i2c_wait_idle();
    rc = i2c_check_err();
    if (rc != I2C_OK)
        return rc;

    for (unsigned int i = 0; i < len; ++i) {
        unsigned int last = (i == len - 1u) ? 1u : 0u;
        I2C_CMD = I2C_CMD_READ | (last ? (I2C_CMD_NACK | I2C_CMD_STOP) : 0u);
        i2c_wait_idle();
        buf[i] = i2c_pop();
    }
    if (len == 0u)
        i2c_stop();
    return i2c_check_err();
}

/* Read one register. */
static int i2c_read_reg(unsigned char addr7, unsigned char reg, unsigned char *val)
{
    return i2c_read_regs(addr7, reg, val, 1);
}

/* ------------------------------------------------------------------ */
/* Interrupt plumbing (optional; the rest of the library is polling)   */
/* ------------------------------------------------------------------ */

/* Current sticky IRQ flags (I2C_IRQ_*). */
static unsigned int i2c_irq_flags(void)
{
    return I2C_IRQ;
}

/* Write-1-to-clear the given I2C_IRQ_* bits. */
static void i2c_irq_clear(unsigned int mask)
{
    I2C_IRQ = mask;
}

/* Arm the given I2C_CTRL_IE_* interrupt enables (ORs into CTRL). */
static void i2c_irq_enable(unsigned int ie)
{
    I2C_CTRL = (I2C_CTRL & ~I2C_CTRL_IE_ALL) | (ie & I2C_CTRL_IE_ALL);
}

#endif /* I2C_H */
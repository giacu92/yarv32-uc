#ifndef SPI_H
#define SPI_H

/*
 * Polling driver for the axi4_lite_spi master peripheral
 * (src/rtl/utils/axi4_lite_spi.sv) at rv32_pkg::SPI_BASE = 0x1000_6000.
 *
 * Register map (word offsets from the peripheral base):
 *   0x00 STATUS (R)   : BUSY / TX_FULL / RX_EMPTY / TX_EMPTY / RX_FULL /
 *                       RX_OVERRUN(sticky)
 *   0x04 CTRL   (RW)  : ENABLE / CPOL / CPHA + 4 interrupt-enable bits
 *   0x08 CLKDIV (RW)  : SCLK = clk / (2*(CLKDIV+1)); 0 = clk/2
 *   0x0C CS     (RW)  : bit0 chip-select line, 0 = active (low)
 *   0x10 TXDATA (W)   : push one byte (LSB) into the TX FIFO
 *   0x14 RXDATA (R)   : pop one byte from the RX FIFO
 *   0x18 IRQ    (R/W1C)
 *
 * The engine is free-running: while ENABLE is set it starts shifting
 * whenever the TX FIFO is non-empty, one RX byte per TX byte, MSB first.
 * spi_transfer() is one byte round trip; spi_transfer_buf() keeps up to 16
 * bytes in flight (the FIFO depth) so long exchanges run at wire speed.
 *
 * CS framing is caller-owned, Arduino-style: wrap an exchange in
 * spi_select() / spi_deselect(). The library never touches CS, so one
 * transaction can be several transfer calls.
 *
 * Mode: spi_init() takes an SPI mode 0-3 (CPOL = bit1, CPHA = bit0), the
 * usual (0,0)/(0,1)/(1,0)/(1,1) numbering.
 *
 * All busy-waits are infinite, like uart.h. Call spi_init() once before
 * anything else; the engine ignores the FIFOs while ENABLE is 0.
 */

#define SPI_BASE 0x10006000u

#define SPI_STATUS (*(volatile unsigned int *)(SPI_BASE + 0x00))
#define SPI_CTRL   (*(volatile unsigned int *)(SPI_BASE + 0x04))
#define SPI_CLKDIV (*(volatile unsigned int *)(SPI_BASE + 0x08))
#define SPI_CS     (*(volatile unsigned int *)(SPI_BASE + 0x0C))
#define SPI_TXDATA (*(volatile unsigned int *)(SPI_BASE + 0x10))
#define SPI_RXDATA (*(volatile unsigned int *)(SPI_BASE + 0x14))
#define SPI_IRQ    (*(volatile unsigned int *)(SPI_BASE + 0x18))

/* STATUS bits */
#define SPI_STATUS_BUSY       0x01u
#define SPI_STATUS_TX_FULL    0x02u
#define SPI_STATUS_RX_EMPTY   0x04u
#define SPI_STATUS_TX_EMPTY   0x08u
#define SPI_STATUS_RX_FULL    0x10u
#define SPI_STATUS_RX_OVERRUN 0x20u  /* sticky: byte dropped, RX full */

/* CTRL bits */
#define SPI_CTRL_ENABLE 0x01u
#define SPI_CTRL_CPOL   0x02u
#define SPI_CTRL_CPHA   0x04u
#define SPI_CTRL_IE_TX  0x08u
#define SPI_CTRL_IE_RX  0x10u
#define SPI_CTRL_IE_ALL 0x78u

/* IRQ W1C bits */
#define SPI_IRQ_TX_EMPTY   0x01u
#define SPI_IRQ_RX_NEMPTY  0x02u
#define SPI_IRQ_DONE       0x04u
#define SPI_IRQ_RX_OVERRUN 0x08u
#define SPI_IRQ_ALL        0x0Fu

/* Core clock feeding CLKDIV, used by the *_hz convenience wrappers.
 * Must track the rPLL (50 MHz shipping). */
#ifndef SPI_CORE_HZ
#define SPI_CORE_HZ 50000000u
#endif

/* ------------------------------------------------------------------ */
/* Init / configuration                                                */
/* ------------------------------------------------------------------ */

/* CLKDIV for a target SCLK: SCLK = core / (2*(div+1)). Rounded down, so
 * the real SCLK is at or slightly above the request. */
static unsigned int spi_clkdiv_for(unsigned int core_hz, unsigned int spi_hz)
{
    unsigned int d = core_hz / (2u * spi_hz);
    return d ? d - 1u : 0u;
}

static void spi_set_clkdiv(unsigned int div)
{
    SPI_CLKDIV = div;
}

/* Set SPI mode 0-3 (CPOL = bit1, CPHA = bit0), preserving everything else. */
static void spi_set_mode(unsigned int mode)
{
    unsigned int c = SPI_CTRL & ~(SPI_CTRL_CPOL | SPI_CTRL_CPHA);
    if (mode & 2u)
        c |= SPI_CTRL_CPOL;
    if (mode & 1u)
        c |= SPI_CTRL_CPHA;
    SPI_CTRL = c;
}

/* Enable the engine at a target SCLK and mode, CS released. */
static void spi_begin(unsigned int core_hz, unsigned int spi_hz, unsigned int mode)
{
    SPI_CTRL   = 0u;  /* no interrupts: pure polling */
    SPI_CLKDIV = spi_clkdiv_for(core_hz, spi_hz);
    spi_set_mode(mode);
    SPI_CTRL  |= SPI_CTRL_ENABLE;
    SPI_CS     = 1u;  /* deselected */
    SPI_IRQ    = SPI_IRQ_ALL;  /* clear any sticky flags */
}

/* Shorthand at the compiled-in core clock: spi_begin_hz(1000000, 0) etc. */
static void spi_begin_hz(unsigned int spi_hz, unsigned int mode)
{
    spi_begin(SPI_CORE_HZ, spi_hz, mode);
}

/* ------------------------------------------------------------------ */
/* CS framing (caller-owned)                                           */
/* ------------------------------------------------------------------ */

static void spi_select(void)
{
    SPI_CS = 0u;  /* active low */
}

static void spi_deselect(void)
{
    SPI_CS = 1u;
}

/* ------------------------------------------------------------------ */
/* Transfers (blocking, full-duplex)                                   */
/* ------------------------------------------------------------------ */

static int spi_busy(void)
{
    return (SPI_STATUS & SPI_STATUS_BUSY) != 0u;
}

/* One byte in, one byte out. The simplest correct round trip: push, wait
 * for the engine to finish, pop. */
static unsigned char spi_transfer(unsigned char b)
{
    while (SPI_STATUS & SPI_STATUS_TX_FULL)
        ;
    SPI_TXDATA = (unsigned int)b;
    while (SPI_STATUS & SPI_STATUS_BUSY)
        ;
    return (unsigned char)(SPI_RXDATA & 0xFFu);
}

/* Buffered full-duplex exchange of any length. Keeps up to 16 bytes in
 * flight (both FIFOs are 16 deep and each TX byte produces exactly one RX
 * byte, so the in-flight window bounds both). tx == 0 sends 0xFF fillers,
 * rx == 0 discards the received bytes. */
static void spi_transfer_buf(const unsigned char *tx, unsigned char *rx, unsigned int len)
{
    unsigned int pushed = 0;
    unsigned int popped = 0;

    while (popped < len) {
        while (pushed < len && (pushed - popped) < 16u &&
               !(SPI_STATUS & SPI_STATUS_TX_FULL)) {
            SPI_TXDATA = tx ? (unsigned int)tx[pushed] : 0xFFu;
            ++pushed;
        }
        while (SPI_STATUS & SPI_STATUS_RX_EMPTY)
            ;
        if (rx)
            rx[popped] = (unsigned char)(SPI_RXDATA & 0xFFu);
        else
            (void)SPI_RXDATA;  /* pop anyway: keep the FIFO in step */
        ++popped;
    }
}

/* Write-only exchange (received bytes discarded). */
static void spi_send(const unsigned char *buf, unsigned int len)
{
    spi_transfer_buf(buf, 0, len);
}

/* Read-only exchange: clocks 0xFF fillers, captures the slave's bytes. */
static void spi_read(unsigned char *buf, unsigned int len)
{
    spi_transfer_buf(0, buf, len);
}

/* Multi-byte helpers, MSB first on the wire. */
static unsigned short spi_transfer16(unsigned short v)
{
    unsigned char hi = spi_transfer((unsigned char)(v >> 8));
    unsigned char lo = spi_transfer((unsigned char)(v & 0xFFu));
    return (unsigned short)((hi << 8) | lo);
}

static unsigned int spi_transfer32(unsigned int v)
{
    unsigned char b3 = spi_transfer((unsigned char)(v >> 24));
    unsigned char b2 = spi_transfer((unsigned char)(v >> 16));
    unsigned char b1 = spi_transfer((unsigned char)(v >> 8));
    unsigned char b0 = spi_transfer((unsigned char)v);
    return ((unsigned int)b3 << 24) | ((unsigned int)b2 << 16) |
           ((unsigned int)b1 << 8) | (unsigned int)b0;
}

static void spi_send16(unsigned short v)
{
    spi_transfer((unsigned char)(v >> 8));
    spi_transfer((unsigned char)(v & 0xFFu));
}

static void spi_send32(unsigned int v)
{
    spi_transfer((unsigned char)(v >> 24));
    spi_transfer((unsigned char)(v >> 16));
    spi_transfer((unsigned char)(v >> 8));
    spi_transfer((unsigned char)v);
}

static unsigned short spi_read16(void)
{
    unsigned char hi = spi_transfer(0xFFu);
    unsigned char lo = spi_transfer(0xFFu);
    return (unsigned short)((hi << 8) | lo);
}

static unsigned int spi_read32(void)
{
    unsigned char b3 = spi_transfer(0xFFu);
    unsigned char b2 = spi_transfer(0xFFu);
    unsigned char b1 = spi_transfer(0xFFu);
    unsigned char b0 = spi_transfer(0xFFu);
    return ((unsigned int)b3 << 24) | ((unsigned int)b2 << 16) |
           ((unsigned int)b1 << 8) | (unsigned int)b0;
}

/* ------------------------------------------------------------------ */
/* Interrupt plumbing (optional; the rest of the library is polling)   */
/* ------------------------------------------------------------------ */

/* Current sticky IRQ flags (SPI_IRQ_*). */
static unsigned int spi_irq_flags(void)
{
    return SPI_IRQ;
}

/* Write-1-to-clear the given SPI_IRQ_* bits. */
static void spi_irq_clear(unsigned int mask)
{
    SPI_IRQ = mask;
}

/* Arm the given SPI_CTRL_IE_* interrupt enables (ORs into CTRL). */
static void spi_irq_enable(unsigned int ie)
{
    SPI_CTRL = (SPI_CTRL & ~SPI_CTRL_IE_ALL) | (ie & SPI_CTRL_IE_ALL);
}

#endif /* SPI_H */
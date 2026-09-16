#ifndef SSD1306_H
#define SSD1306_H

/*
 * Driver for the SSD1306 128x64 OLED (the common 0.96" I2C modules) on top
 * of common/i2c.h. Default address 0x3C (SA0 pin low); pass 0x3D to
 * ssd1306_begin if the module is strapped the other way.
 *
 * Transaction format: every byte after the address is prefixed with a
 * "control" byte -- 0x00 means "what follows is a command", 0x40 means
 * "what follows is data". The driver uses i2c_write_regs() for this: its
 * "register" byte lands exactly in the control-byte position, so commands
 * and framebuffer data both go out as a single START..STOP transaction of
 * any length (the i2c engine drains its TX FIFO while shifting, so the
 * 1024-byte framebuffer is fine in one go).
 *
 * The library keeps a full 1 KiB framebuffer in RAM (the D-mem is 16 KiB,
 * so this is a real budget line) and pushes it with ssd1306_update().
 * Pixels: fb byte = 8 vertical pixels of one column, page-major, so
 * ssd1306_pixel(x, y) touches bit (y % 8) of byte (y / 8) * 128 + x.
 *
 * Initialisation ends with the display ON showing the (cleared) RAM.
 */

#include "i2c.h"

#define SSD1306_ADDR     0x3Cu
#define SSD1306_WIDTH    128u
#define SSD1306_HEIGHT   64u
#define SSD1306_FB_SIZE  (SSD1306_WIDTH * SSD1306_HEIGHT / 8u)  /* 1024 */

/* Control bytes. */
#define SSD1306_CTRL_CMD 0x00u
#define SSD1306_CTRL_DAT 0x40u

static unsigned char ssd1306_addr = SSD1306_ADDR;

/* Framebuffer: page-major, 128 columns per page, 8 pages. */
static unsigned char ssd1306_fb[SSD1306_FB_SIZE];

/* ------------------------------------------------------------------ */
/* Low-level transactions                                              */
/* ------------------------------------------------------------------ */

/* One command byte. */
static void ssd1306_cmd(unsigned char c)
{
    i2c_write_reg(ssd1306_addr, SSD1306_CTRL_CMD, c);
}

/* Several command bytes in one transaction. */
static void ssd1306_cmds(const unsigned char *c, unsigned int n)
{
    i2c_write_regs(ssd1306_addr, SSD1306_CTRL_CMD, c, n);
}

/*
 * Optional cooperative yield, called between the I2C chunks of a paged
 * update (ssd1306_update_page). Null by default.
 *
 * It exists because a display push is LONG next to a real-time input: the
 * full framebuffer is 1 KiB, which at 400 kHz is about 25 ms, and anything
 * feeding a small hardware FIFO (an I2S receiver, say) loses samples for
 * the whole of it. Chunking the transfer and draining that FIFO between
 * chunks is what keeps an audio stream contiguous while the screen is
 * being written.
 */
static void (*ssd1306_yield_fn)(void);

/* Bytes per I2C transaction in a paged update. 16 bytes is about 430 us at
 * 400 kHz including the address and control byte, against a 16-entry FIFO
 * at 15625 Hz that fills in 1.02 ms -- a bit over 2x of margin. The
 * transaction overhead is ~68 us either way, so 16 costs about 0.2 ms more
 * per page than 24 and buys 170 us of margin, which is the better trade on
 * a system with a hard deadline. Raise it if nothing needs servicing. */
#ifndef SSD1306_CHUNK
#define SSD1306_CHUNK 16u
#endif

/* Push the whole framebuffer in one transaction.
 *
 * Sets the address window itself rather than relying on the one
 * ssd1306_begin() left behind -- ssd1306_update_page() narrows that window
 * to a single page, so a full update after a paged one would otherwise
 * write 1 KiB into one page's worth of address space. */
static void ssd1306_update(void)
{
    ssd1306_cmd(0x21); /* column range 0..WIDTH-1 */
    ssd1306_cmd(0x00);
    ssd1306_cmd(SSD1306_WIDTH - 1u);
    ssd1306_cmd(0x22); /* page range 0..PAGES-1 */
    ssd1306_cmd(0x00);
    ssd1306_cmd((SSD1306_HEIGHT / 8u) - 1u);
    i2c_write_regs(ssd1306_addr, SSD1306_CTRL_DAT, ssd1306_fb, SSD1306_FB_SIZE);
}

/*
 * Push ONE page (8 pixel rows, WIDTH bytes) in SSD1306_CHUNK-sized
 * transactions, calling ssd1306_yield_fn between them.
 *
 * Two things this buys over ssd1306_update(): only what changed goes out
 * (a caller that tracks dirty pages pushes 128 bytes instead of 1024), and
 * the transfer is interruptible at chunk granularity. Horizontal
 * addressing auto-increments across transactions, so the chunks need no
 * per-chunk addressing.
 */
static void ssd1306_update_page(unsigned int page)
{
    const unsigned char *p = &ssd1306_fb[page * SSD1306_WIDTH];
    unsigned int off = 0u;
    /* Column range 0..WIDTH-1, then page range page..page. */
    const unsigned char win[6] = {0x21u, 0x00u, (unsigned char)(SSD1306_WIDTH - 1u),
                                  0x22u, (unsigned char)page, (unsigned char)page};

    /*
     * ONE transaction for the six setup bytes, not six.
     *
     * Each separate ssd1306_cmd() is a full I2C transaction -- start,
     * address, control byte, data, stop -- about 68 us at 400 kHz, so six
     * of them are ~420 us of bus with no yield in the middle. Landing
     * straight after a data chunk that itself took ~430 us, that put the
     * worst blind window at ~0.85 ms against a FIFO that fills in 1.02 ms,
     * which is not margin, it is luck. As one transaction the burst is
     * ~180 us, and the yields either side bound the gap to a single chunk.
     */
    if (ssd1306_yield_fn)
        ssd1306_yield_fn();
    ssd1306_cmds(win, sizeof win);
    if (ssd1306_yield_fn)
        ssd1306_yield_fn();

    while (off < SSD1306_WIDTH) {
        unsigned int n = SSD1306_WIDTH - off;

        if (n > SSD1306_CHUNK)
            n = SSD1306_CHUNK;
        i2c_write_regs(ssd1306_addr, SSD1306_CTRL_DAT, p + off, n);
        off += n;
        if (ssd1306_yield_fn)
            ssd1306_yield_fn();
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

/* Bring the display up. i2c must already be configured (i2c_begin...).
 * Returns I2C_OK iff the module ACKed its first command. */
static int ssd1306_begin(unsigned char addr)
{
    static const unsigned char init[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock: default divide/oscillator */
        0xA8, 0x3F,       /* multiplex: 64 lines */
        0xD3, 0x00,       /* display offset: 0 */
        0x40,             /* display start line: 0 */
        0x8D, 0x14,       /* charge pump: on (required on these modules) */
        0x20, 0x00,       /* memory addressing mode: horizontal */
        0xA1,             /* segment remap: col127 -> SEG0 (flip horizontally) */
        0xC8,             /* COM scan: remapped (flip vertically, the two together rotate 180) */
        0x81, 0xCF,       /* contrast */
        0xD9, 0xF1,       /* precharge period */
        0xDB, 0x40,       /* VCOMH deselect level */
        0xA4,             /* display follows RAM content */
        0xA6,             /* normal (non-inverted) colors */
        0x2E,             /* scroll off */
        0xAF              /* display on */
    };
    int rc;

    ssd1306_addr = addr;
    rc = i2c_write_regs(addr, SSD1306_CTRL_CMD, init, sizeof init);
    if (rc != I2C_OK)
        return rc;
    /* Set the drawing window once: horizontal addressing mode auto-wraps
     * inside it, so every update starts at fb[0] without more commands. */
    ssd1306_cmd(0x21);  /* column address range */
    ssd1306_cmd(0x00);
    ssd1306_cmd(0x7F);
    ssd1306_cmd(0x22);  /* page address range */
    ssd1306_cmd(0x00);
    ssd1306_cmd(0x07);
    return I2C_OK;
}

/* ------------------------------------------------------------------ */
/* Drawing                                                             */
/* ------------------------------------------------------------------ */

/* Clear the framebuffer to black (all callers start from a blank slate;
 * ssd1306_update still needs to run to push it). */
static void ssd1306_clear(void)
{
    for (unsigned int i = 0; i < SSD1306_FB_SIZE; ++i)
        ssd1306_fb[i] = 0;
}

/* Fill the whole framebuffer (all pixels on). */
static void ssd1306_fill(void)
{
    for (unsigned int i = 0; i < SSD1306_FB_SIZE; ++i)
        ssd1306_fb[i] = 0xFF;
}

/* Set (on != 0) or clear one pixel. Out-of-range coordinates are dropped. */
static void ssd1306_pixel(unsigned int x, unsigned int y, int on)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
        return;
    if (on)
        ssd1306_fb[(y / 8u) * SSD1306_WIDTH + x] |=
            (unsigned char)(1u << (y % 8u));
    else
        ssd1306_fb[(y / 8u) * SSD1306_WIDTH + x] &=
            (unsigned char)~(1u << (y % 8u));
}

/* Horizontal line, inclusive endpoints. */
static void ssd1306_hline(unsigned int x0, unsigned int x1, unsigned int y, int on)
{
    for (unsigned int x = x0; x <= x1; ++x)
        ssd1306_pixel(x, y, on);
}

/* Vertical line, inclusive endpoints. */
static void ssd1306_vline(unsigned int x, unsigned int y0, unsigned int y1, int on)
{
    for (unsigned int y = y0; y <= y1; ++y)
        ssd1306_pixel(x, y, on);
}

/* Rectangle outline. */
static void ssd1306_rect(unsigned int x, unsigned int y,
                         unsigned int w, unsigned int h, int on)
{
    if (w == 0u || h == 0u)
        return;
    ssd1306_hline(x, x + w - 1u, y, on);
    ssd1306_hline(x, x + w - 1u, y + h - 1u, on);
    ssd1306_vline(x, y, y + h - 1u, on);
    ssd1306_vline(x + w - 1u, y, y + h - 1u, on);
}

/* Filled rectangle. */
static void ssd1306_fill_rect(unsigned int x, unsigned int y,
                              unsigned int w, unsigned int h, int on)
{
    for (unsigned int r = 0; r < h; ++r)
        ssd1306_hline(x, x + w - 1u, y + r, on);
}

/* Bresenham line between two points. */
static void ssd1306_line(int x0, int y0, int x1, int y1, int on)
{
    int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int dy = y1 > y0 ? y1 - y0 : y0 - y1;
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        ssd1306_pixel((unsigned int)x0, (unsigned int)y0, on);
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Display control                                                     */
/* ------------------------------------------------------------------ */

static void ssd1306_display_on(int on)
{
    ssd1306_cmd(on ? 0xAFu : 0xAEu);
}

/* Invert colors (light pixels on dark background <-> the opposite). */
static void ssd1306_invert(int on)
{
    ssd1306_cmd(on ? 0xA7u : 0xA6u);
}

/* Contrast 0..255. */
static void ssd1306_set_contrast(unsigned char c)
{
    ssd1306_cmd(0x81u);
    ssd1306_cmd(c);
}

/* 180-degree rotation via the two remap commands (no framebuffer change). */
static void ssd1306_set_rotation(int flipped)
{
    ssd1306_cmd(flipped ? 0xA1u : 0xA0u);
    ssd1306_cmd(flipped ? 0xC8u : 0xC0u);
}

#endif /* SSD1306_H */
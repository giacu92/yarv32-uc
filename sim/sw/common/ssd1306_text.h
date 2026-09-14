#ifndef SSD1306_TEXT_H
#define SSD1306_TEXT_H

/*
 * Text rendering on top of common/ssd1306.h.
 *
 * Kept separate from the driver because it costs ~300 bytes of rodata for
 * the font, and a program that only draws graphics should not pay for it.
 * Include this INSTEAD of ssd1306.h when you want text -- it pulls the
 * driver in itself.
 *
 * Font: 5x7 in a 6x8 cell (one blank column, one blank row), column-major,
 * bit 0 of each byte is the TOP pixel -- the same orientation as the
 * framebuffer's pages, so a glyph column is one OR into one byte when the
 * text sits on an 8-pixel row boundary. It does not have to: the renderer
 * writes pixels, so any y works, it is just slower off a boundary.
 *
 * Coverage is ASCII 0x20..0x5A, space through 'Z'. THERE ARE NO LOWERCASE
 * LETTERS: a 128x64 panel showing status text does not need them, and
 * leaving them out halves the table. A character outside the range renders
 * as a space rather than as garbage, so a stray byte cannot walk off the
 * end of the table. The few punctuation slots nobody asked for ('"', '$',
 * '&', ''', '(', ')', '*', ';', '=', '?', '@') are deliberately blank.
 *
 * At 6x8 a 128x64 panel is 21 columns by 8 rows of text; the 2x renderer
 * gives 10 by 4.
 *
 * Naming: ports *_i/_o do not apply here (this is firmware); functions are
 * ssd1306_* like the driver they extend.
 */

#include "ssd1306.h"

#define SSD1306_FONT_FIRST 0x20u
#define SSD1306_FONT_LAST  0x5Au
#define SSD1306_FONT_W     5u
#define SSD1306_FONT_H     7u
#define SSD1306_CHAR_W     6u /* glyph + 1 blank column */
#define SSD1306_CHAR_H     8u /* glyph + 1 blank row    */

/* 5 column bytes per glyph, bit 0 = top row. */
static const unsigned char ssd1306_font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00,  /* SPACE */
    0x00, 0x00, 0x5F, 0x00, 0x00,  /* ! */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* " */
    0x14, 0x7F, 0x14, 0x7F, 0x14,  /* # */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* $ */
    0x63, 0x13, 0x08, 0x64, 0x63,  /* % */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* & */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* ' */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* ( */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* ) */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* * */
    0x08, 0x08, 0x3E, 0x08, 0x08,  /* + */
    0x00, 0x40, 0x30, 0x00, 0x00,  /* , */
    0x08, 0x08, 0x08, 0x08, 0x08,  /* - */
    0x00, 0x00, 0x60, 0x00, 0x00,  /* . */
    0x60, 0x10, 0x08, 0x04, 0x03,  /* / */
    0x3E, 0x51, 0x49, 0x45, 0x3E,  /* 0 */
    0x00, 0x42, 0x7F, 0x40, 0x00,  /* 1 */
    0x42, 0x61, 0x51, 0x49, 0x46,  /* 2 */
    0x21, 0x41, 0x45, 0x4B, 0x31,  /* 3 */
    0x18, 0x14, 0x12, 0x7F, 0x10,  /* 4 */
    0x27, 0x45, 0x45, 0x45, 0x39,  /* 5 */
    0x3C, 0x4A, 0x49, 0x49, 0x30,  /* 6 */
    0x01, 0x71, 0x09, 0x05, 0x03,  /* 7 */
    0x36, 0x49, 0x49, 0x49, 0x36,  /* 8 */
    0x06, 0x49, 0x49, 0x29, 0x1E,  /* 9 */
    0x00, 0x00, 0x36, 0x00, 0x00,  /* : */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* ; */
    0x08, 0x14, 0x22, 0x41, 0x00,  /* < */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* = */
    0x00, 0x41, 0x22, 0x14, 0x08,  /* > */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* ? */
    0x00, 0x00, 0x00, 0x00, 0x00,  /* @ */
    0x7E, 0x11, 0x11, 0x11, 0x7E,  /* A */
    0x7F, 0x49, 0x49, 0x49, 0x36,  /* B */
    0x3E, 0x41, 0x41, 0x41, 0x22,  /* C */
    0x7F, 0x41, 0x41, 0x22, 0x1C,  /* D */
    0x7F, 0x49, 0x49, 0x49, 0x41,  /* E */
    0x7F, 0x09, 0x09, 0x09, 0x01,  /* F */
    0x3E, 0x41, 0x41, 0x49, 0x7A,  /* G */
    0x7F, 0x08, 0x08, 0x08, 0x7F,  /* H */
    0x00, 0x41, 0x7F, 0x41, 0x00,  /* I */
    0x20, 0x40, 0x41, 0x3F, 0x01,  /* J */
    0x7F, 0x08, 0x14, 0x22, 0x41,  /* K */
    0x7F, 0x40, 0x40, 0x40, 0x40,  /* L */
    0x7F, 0x02, 0x0C, 0x02, 0x7F,  /* M */
    0x7F, 0x02, 0x04, 0x08, 0x7F,  /* N */
    0x3E, 0x41, 0x41, 0x41, 0x3E,  /* O */
    0x7F, 0x09, 0x09, 0x09, 0x06,  /* P */
    0x3E, 0x41, 0x51, 0x21, 0x5E,  /* Q */
    0x7F, 0x09, 0x19, 0x29, 0x46,  /* R */
    0x46, 0x49, 0x49, 0x49, 0x31,  /* S */
    0x01, 0x01, 0x7F, 0x01, 0x01,  /* T */
    0x3F, 0x40, 0x40, 0x40, 0x3F,  /* U */
    0x1F, 0x20, 0x40, 0x20, 0x1F,  /* V */
    0x7F, 0x20, 0x18, 0x20, 0x7F,  /* W */
    0x63, 0x14, 0x08, 0x14, 0x63,  /* X */
    0x03, 0x04, 0x78, 0x04, 0x03,  /* Y */
    0x61, 0x51, 0x49, 0x45, 0x43,  /* Z */
};

/* One glyph at (x, y), top-left corner, unscaled. Pixels outside the panel
 * are dropped by ssd1306_pixel, so a string may run off the right edge
 * without corrupting anything. */
static void ssd1306_char(unsigned int x, unsigned int y, unsigned char c, int on)
{
    unsigned int col, row, idx;

    if (c < SSD1306_FONT_FIRST || c > SSD1306_FONT_LAST) {
        c = ' ';
    }
    idx = (unsigned int)(c - SSD1306_FONT_FIRST) * SSD1306_FONT_W;

    for (col = 0; col < SSD1306_FONT_W; col++) {
        unsigned char bits = ssd1306_font5x7[idx + col];
        for (row = 0; row < SSD1306_FONT_H; row++) {
            if ((bits >> row) & 1u) {
                ssd1306_pixel(x + col, y + row, on);
            }
        }
    }
}

/* One glyph scaled by `scale` in both axes (scale 1 == ssd1306_char). */
static void ssd1306_char_scaled(unsigned int x, unsigned int y, unsigned char c,
                                unsigned int scale, int on)
{
    unsigned int col, row, dx, dy, idx;

    if (scale <= 1u) {
        ssd1306_char(x, y, c, on);
        return;
    }
    if (c < SSD1306_FONT_FIRST || c > SSD1306_FONT_LAST) {
        c = ' ';
    }
    idx = (unsigned int)(c - SSD1306_FONT_FIRST) * SSD1306_FONT_W;

    for (col = 0; col < SSD1306_FONT_W; col++) {
        unsigned char bits = ssd1306_font5x7[idx + col];
        for (row = 0; row < SSD1306_FONT_H; row++) {
            if (!((bits >> row) & 1u)) {
                continue;
            }
            for (dx = 0; dx < scale; dx++) {
                for (dy = 0; dy < scale; dy++) {
                    ssd1306_pixel(x + col * scale + dx, y + row * scale + dy, on);
                }
            }
        }
    }
}

/* NUL-terminated string at (x, y). Returns the x just past the last cell,
 * so calls can be chained. */
static unsigned int ssd1306_text(unsigned int x, unsigned int y, const char *s, int on)
{
    while (*s) {
        ssd1306_char(x, y, (unsigned char)*s++, on);
        x += SSD1306_CHAR_W;
    }
    return x;
}

static unsigned int ssd1306_text_scaled(unsigned int x, unsigned int y, const char *s,
                                        unsigned int scale, int on)
{
    if (scale == 0u) {
        scale = 1u;
    }
    while (*s) {
        ssd1306_char_scaled(x, y, (unsigned char)*s++, scale, on);
        x += SSD1306_CHAR_W * scale;
    }
    return x;
}

/* Pixel width a string will occupy at a given scale -- for centring. */
static unsigned int ssd1306_text_width(const char *s, unsigned int scale)
{
    unsigned int n = 0;
    if (scale == 0u) {
        scale = 1u;
    }
    while (*s++) {
        n++;
    }
    return n * SSD1306_CHAR_W * scale;
}

/* Signed decimal, right-aligned into `width` cells (blank-padded, so a
 * changing number does not leave debris behind it). width 0 = natural
 * width. Returns the x just past the field. */
static unsigned int ssd1306_int(unsigned int x, unsigned int y, int v, unsigned int width,
                                unsigned int scale, int on)
{
    char buf[12];
    unsigned int n = 0, i;
    unsigned int mag;
    int neg = (v < 0);

    mag = (unsigned int)(neg ? -v : v);
    do {
        buf[n++] = (char)('0' + (mag % 10u));
        mag /= 10u;
    } while (mag && n < sizeof(buf) - 1u);
    if (neg && n < sizeof(buf) - 1u) {
        buf[n++] = '-';
    }

    for (i = n; i < width; i++) {
        ssd1306_char_scaled(x, y, ' ', scale, on);
        x += SSD1306_CHAR_W * scale;
    }
    while (n--) {
        ssd1306_char_scaled(x, y, (unsigned char)buf[n], scale, on);
        x += SSD1306_CHAR_W * scale;
    }
    return x;
}

/* Fixed-point decimal: prints v/10^frac with `frac` digits after the point
 * (e.g. ssd1306_fixed(x, y, 8241, 2, ...) -> "82.41"). Integer only -- no
 * FPU and no libc here. */
static unsigned int ssd1306_fixed(unsigned int x, unsigned int y, int v, unsigned int frac,
                                  unsigned int scale, int on)
{
    unsigned int div = 1u, i;
    int whole, rest;

    for (i = 0; i < frac; i++) {
        div *= 10u;
    }
    whole = v / (int)div;
    rest = v - whole * (int)div;
    if (rest < 0) {
        rest = -rest;
    }

    x = ssd1306_int(x, y, whole, 0, scale, on);
    if (frac) {
        ssd1306_char_scaled(x, y, '.', scale, on);
        x += SSD1306_CHAR_W * scale;
        while (div > 1u) {
            div /= 10u;
            ssd1306_char_scaled(x, y, (unsigned char)('0' + (unsigned int)rest / div), scale, on);
            x += SSD1306_CHAR_W * scale;
            rest = (int)((unsigned int)rest % div);
        }
    }
    return x;
}

#endif /* SSD1306_TEXT_H */

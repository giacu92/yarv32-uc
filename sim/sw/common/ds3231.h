#ifndef DS3231_H
#define DS3231_H

/*
 * Driver for the Maxim DS3231 extremely-accurate I2C RTC on top of
 * common/i2c.h. Address 0x68 (fixed).
 *
 * Register map (8-bit registers, BCD-encoded time fields):
 *   0x00 seconds    0x01 minutes    0x02 hours (bit6=0 -> 24h format)
 *   0x03 day-of-week (1=Sunday..7) 0x04 date 0x05 month (bit7=century)
 *   0x06 year (00-99 within the century)
 *   0x07..0x09 alarms 1-2 (not covered here)
 *   0x0E control   0x0F status (bit7 OSF = oscillator stopped)
 *   0x11/0x12 temperature (signed whole + 2 fractional bits)
 *
 * The driver always reads and writes the time in 24-hour format and
 * zero-based day-of-week (0=Sunday..6) -- the struct is binary, the BCD
 * conversion stays inside the driver.
 */

#include "i2c.h"

#define DS3231_ADDR 0x68u

/* Register addresses. */
#define DS3231_REG_SECONDS 0x00u
#define DS3231_REG_CONTROL 0x0Eu
#define DS3231_REG_STATUS  0x0Fu
#define DS3231_REG_TEMP    0x11u

/* Status bits. */
#define DS3231_STATUS_OSF 0x80u  /* oscillator stopped: time is invalid */

/* Control bits (defaults kept: no square wave, no alarms). */
#define DS3231_CTRL_EOSC 0x80u  /* 1 = oscillator STOPPED */

/* Binary time; caller-friendly numbers, no BCD. */
struct ds3231_time {
    unsigned char sec;    /* 0..59 */
    unsigned char min;    /* 0..59 */
    unsigned char hour;   /* 0..23 (24h) */
    unsigned char dow;    /* 0..6, 0 = Sunday */
    unsigned char date;   /* 1..31 */
    unsigned char month;  /* 1..12 */
    unsigned char year;   /* 0..99 (2000+) */
};

/* ------------------------------------------------------------------ */
/* BCD                                                                 */
/* ------------------------------------------------------------------ */

static unsigned char ds3231_bcd2bin(unsigned char b)
{
    return (unsigned char)((b & 0x0Fu) + 10u * ((b >> 4) & 0x0Fu));
}

static unsigned char ds3231_bin2bcd(unsigned char v)
{
    return (unsigned char)(((v / 10u) << 4) | (v % 10u));
}

/* ------------------------------------------------------------------ */
/* Presence / oscillator                                               */
/* ------------------------------------------------------------------ */

/* True if the RTC ACKs. */
static int ds3231_present(void)
{
    return i2c_probe(DS3231_ADDR) == I2C_OK;
}

/* True if the oscillator was stopped (battery first fit / power loss):
 * the time registers are invalid until set once. Reads and clears OSF. */
static int ds3231_time_invalid(void)
{
    unsigned char st = 0;

    if (i2c_read_reg(DS3231_ADDR, DS3231_REG_STATUS, &st) != I2C_OK)
        return 1;
    if (st & DS3231_STATUS_OSF)
        i2c_write_reg(DS3231_ADDR, DS3231_REG_STATUS,
                      (unsigned char)(st & ~DS3231_STATUS_OSF));
    return (st & DS3231_STATUS_OSF) != 0;
}

/* ------------------------------------------------------------------ */
/* Time                                                                */
/* ------------------------------------------------------------------ */

/* Read the time. Returns I2C_OK and fills t, or the i2c error. */
static int ds3231_get_time(struct ds3231_time *t)
{
    unsigned char r[7];
    int rc;

    rc = i2c_read_regs(DS3231_ADDR, DS3231_REG_SECONDS, r, 7);
    if (rc != I2C_OK)
        return rc;

    t->sec   = ds3231_bcd2bin(r[0]);
    t->min   = ds3231_bcd2bin(r[1]);
    t->hour  = ds3231_bcd2bin((unsigned char)(r[2] & 0x3Fu));  /* 24h field */
    t->dow   = (unsigned char)(r[3] ? (r[3] - 1u) : 0u);       /* 1..7 -> 0..6 */
    t->date  = ds3231_bcd2bin(r[4]);
    t->month = ds3231_bcd2bin((unsigned char)(r[5] & 0x1Fu));  /* century off */
    t->year  = ds3231_bcd2bin(r[6]);
    return I2C_OK;
}

/* Write the time (and start the oscillator if it was stopped). Values are
 * range-checked coarse: out-of-range fields make the function return -1
 * without touching the chip. */
static int ds3231_set_time(const struct ds3231_time *t)
{
    unsigned char r[7];

    if (t->sec > 59u || t->min > 59u || t->hour > 23u || t->dow > 6u ||
        t->date < 1u || t->date > 31u || t->month < 1u || t->month > 12u ||
        t->year > 99u)
        return -1;

    r[0] = ds3231_bin2bcd(t->sec);
    r[1] = ds3231_bin2bcd(t->min);
    r[2] = ds3231_bin2bcd(t->hour);          /* bit6 = 0 -> 24h format */
    r[3] = (unsigned char)(t->dow + 1u);     /* 0..6 -> 1..7 */
    r[4] = ds3231_bin2bcd(t->date);
    r[5] = ds3231_bin2bcd(t->month);
    r[6] = ds3231_bin2bcd(t->year);

    return i2c_write_regs(DS3231_ADDR, DS3231_REG_SECONDS, r, 7);
}

/* ------------------------------------------------------------------ */
/* Temperature                                                         */
/* ------------------------------------------------------------------ */

/* Chip temperature in quarter-degrees C, signed: 250 = 25.00 C, -50 =
 * -0.50 C. The DS3231 refreshes it every 64 s; that is the sensor's
 * property, not the bus's. Returns 0 on an i2c error (a real reading can
 * be 0 too; check the return code in rc if you care). */
static int ds3231_get_temp_x4(int *rc_out)
{
    unsigned char r[2];
    int rc;
    int t;

    rc = i2c_read_regs(DS3231_ADDR, DS3231_REG_TEMP, r, 2);
    if (rc_out)
        *rc_out = rc;
    if (rc != I2C_OK)
        return 0;

    t = (int)(r[0] & 0xFFu);
    if (r[0] & 0x80u)
        t -= 256;  /* sign-extend the whole part */
    return t * 4 + (int)(r[1] >> 6);  /* 2 fractional bits */
}

/* Convenience: whole degrees, rounded toward negative infinity. */
static int ds3231_get_temp_c(int *rc_out)
{
    return ds3231_get_temp_x4(rc_out) / 4;
}

#endif /* DS3231_H */
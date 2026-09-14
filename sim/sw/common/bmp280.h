#ifndef BMP280_H
#define BMP280_H

/*
 * Driver for the Bosch BMP280 barometric pressure sensor over I2C, on top
 * of common/i2c.h. Address 0x76 (SDO low) or 0x77 (SDO high) -- most
 * modules are 0x76; try both with bmp280_begin() and keep the one that
 * answers with chip id 0x58.
 *
 * Register map:
 *   0xD0 chip id (0x58 for BMP280; 0x60 = BME280, not covered)
 *   0xF3 status: bit3 measuring, bit0 im_update (calibration copy)
 *   0xF4 ctrl_meas: osrs_t[7:5], osrs_p[4:2], mode[1:0]
 *   0xF7..0xF9 pressure (msb, lsb, xlsb<<4 -> 20 bits)
 *   0xFA..0xFC temperature (same layout)
 *   0x88..0x9F calibration: T1..T3, P1..P9, little endian
 *
 * The driver works in FORCED mode: write ctrl_meas once, wait for
 * status.measuring to clear, read the raw values, compensate. One
 * measurement per bmp280_measure() call, no continuous sampling to
 * configure or stop.
 *
 * Compensation is the Bosch datasheet's integer algorithm. It needs 64-bit
 * multiply and divide; the firmware build links no libgcc, so the driver
 * carries its own small signed-64-bit helpers (mul/div mod 2^64). The
 * temperature comes out in 0.01 C, the pressure in Pa (integer).
 */

#include "i2c.h"

#define BMP280_ADDR_A 0x76u
#define BMP280_ADDR_B 0x77u

#define BMP280_CHIP_ID 0x58u

/* Register addresses. */
#define BMP280_REG_ID       0xD0u
#define BMP280_REG_STATUS   0xF3u
#define BMP280_REG_CTRL_MEAS 0xF4u
#define BMP280_REG_PRESS    0xF7u
#define BMP280_REG_TEMP     0xFAu
#define BMP280_REG_CALIB    0x88u

/* Status bits. */
#define BMP280_STATUS_MEASURING 0x08u
#define BMP280_STATUS_IM_UPDATE 0x01u

/* ctrl_meas oversampling values. */
#define BMP280_OSRS_SKIPPED 0u
#define BMP280_OSRS_X1     1u
#define BMP280_OSRS_X2     2u
#define BMP280_OSRS_X4     3u
#define BMP280_OSRS_X8     4u
#define BMP280_OSRS_X16    5u

/* ctrl_meas mode. */
#define BMP280_MODE_SLEEP  0u
#define BMP280_MODE_FORCED 1u
#define BMP280_MODE_NORMAL 3u

/* Calibration coefficients (little-endian on the chip). T1/P1 unsigned,
 * the rest signed 16-bit. */
struct bmp280_calib {
    unsigned short t1;
    short t2, t3;
    unsigned short p1;
    short p2, p3, p4, p5, p6, p7, p8, p9;
};

static unsigned char bmp280_addr = BMP280_ADDR_A;
static struct bmp280_calib bmp280_cal;
static int bmp280_t_fine;  /* carries temperature state into the pressure calc */

/* ------------------------------------------------------------------ */
/* Minimal signed 64-bit arithmetic (no libgcc in the firmware build)   */
/*                                                                     */
/* mul is exact mod 2^64 (the dropped a1*b1 term is a multiple of 2^64); */
/* div is truncating, divisor must be nonzero and below 2^63.           */
/* ------------------------------------------------------------------ */

typedef long long bmp_s64;
typedef unsigned long long bmp_u64;

static bmp_u64 bmp_u64_mul(bmp_u64 a, bmp_u64 b)
{
    unsigned int a0 = (unsigned int)a, a1 = (unsigned int)(a >> 32);
    unsigned int b0 = (unsigned int)b, b1 = (unsigned int)(b >> 32);

    return (bmp_u64)a0 * b0 +
           (((bmp_u64)a0 * b1 + (bmp_u64)a1 * b0) << 32);
}

static bmp_u64 bmp_u64_div(bmp_u64 n, bmp_u64 d)
{
    bmp_u64 q = 0;
    bmp_u64 r = 0;

    for (int i = 63; i >= 0; --i) {
        r = (r << 1) | ((n >> i) & 1u);
        if (r >= d) {
            r -= d;
            q |= (bmp_u64)1 << i;
        }
    }
    return q;
}

static bmp_s64 bmp_s64_mul(bmp_s64 a, bmp_s64 b)
{
    int neg = 0;
    bmp_u64 ma, mb;

    if (a < 0) {
        neg = !neg;
        ma = (bmp_u64)(-(a + 1)) + 1u;  /* -a without UB at the minimum */
    } else {
        ma = (bmp_u64)a;
    }
    if (b < 0) {
        neg = !neg;
        mb = (bmp_u64)(-(b + 1)) + 1u;
    } else {
        mb = (bmp_u64)b;
    }
    if (neg)
        return -(bmp_s64)bmp_u64_mul(ma, mb);
    return (bmp_s64)bmp_u64_mul(ma, mb);
}

static bmp_s64 bmp_s64_div(bmp_s64 a, bmp_s64 b)
{
    int neg = 0;
    bmp_u64 ma, mb;

    if (a < 0) {
        neg = !neg;
        ma = (bmp_u64)(-(a + 1)) + 1u;
    } else {
        ma = (bmp_u64)a;
    }
    if (b < 0) {
        neg = !neg;
        mb = (bmp_u64)(-(b + 1)) + 1u;
    } else {
        mb = (bmp_u64)b;
    }
    if (neg)
        return -(bmp_s64)bmp_u64_div(ma, mb);
    return (bmp_s64)bmp_u64_div(ma, mb);
}

/* Little-endian signed/unsigned 16-bit out of a byte buffer. */
static unsigned short bmp280_le_u16(const unsigned char *p)
{
    return (unsigned short)(p[0] | ((unsigned short)p[1] << 8));
}

static short bmp280_le_s16(const unsigned char *p)
{
    unsigned short u = bmp280_le_u16(p);

    if (u & 0x8000u)
        return (short)(u - 0x10000u);
    return (short)u;
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

/* Read the calibration coefficients (24 bytes, 0x88..0x9F). */
static int bmp280_read_calib(void)
{
    unsigned char c[24];
    int rc;

    rc = i2c_read_regs(bmp280_addr, BMP280_REG_CALIB, c, 24);
    if (rc != I2C_OK)
        return rc;

    bmp280_cal.t1 = bmp280_le_u16(&c[0]);
    bmp280_cal.t2 = bmp280_le_s16(&c[2]);
    bmp280_cal.t3 = bmp280_le_s16(&c[4]);
    bmp280_cal.p1 = bmp280_le_u16(&c[6]);
    bmp280_cal.p2 = bmp280_le_s16(&c[8]);
    bmp280_cal.p3 = bmp280_le_s16(&c[10]);
    bmp280_cal.p4 = bmp280_le_s16(&c[12]);
    bmp280_cal.p5 = bmp280_le_s16(&c[14]);
    bmp280_cal.p6 = bmp280_le_s16(&c[16]);
    bmp280_cal.p7 = bmp280_le_s16(&c[18]);
    bmp280_cal.p8 = bmp280_le_s16(&c[20]);
    bmp280_cal.p9 = bmp280_le_s16(&c[22]);
    return I2C_OK;
}

/* Adopt the sensor at addr: probe, check the chip id, read the calibration.
 * Returns 0 on success, -1 if nothing answering answered as a BMP280.
 * i2c must already be configured (i2c_begin...). */
static int bmp280_begin(unsigned char addr)
{
    unsigned char id = 0;

    bmp280_addr = addr;
    if (i2c_read_reg(bmp280_addr, BMP280_REG_ID, &id) != I2C_OK)
        return -1;
    if (id != BMP280_CHIP_ID)
        return -1;
    if (bmp280_read_calib() != I2C_OK)
        return -1;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Measurement (forced mode)                                           */
/* ------------------------------------------------------------------ */

/* True while a forced conversion is running. */
static int bmp280_busy(void)
{
    unsigned char st = 0;

    if (i2c_read_reg(bmp280_addr, BMP280_REG_STATUS, &st) != I2C_OK)
        return 1;  /* bus error: keep the caller waiting, it retries */
    return (st & BMP280_STATUS_MEASURING) != 0u;
}

/* Start one forced conversion with the given oversampling settings
 * (BMP280_OSRS_*). */
static int bmp280_start_forced(unsigned int osrs_t, unsigned int osrs_p)
{
    unsigned char ctrl = (unsigned char)
        ((osrs_t << 5) | (osrs_p << 2) | BMP280_MODE_FORCED);

    return i2c_write_reg(bmp280_addr, BMP280_REG_CTRL_MEAS, ctrl);
}

/* Read the raw 20-bit ADC values (only meaningful after a conversion). */
static int bmp280_read_raw(unsigned int *adc_p, unsigned int *adc_t)
{
    unsigned char d[6];
    int rc;

    rc = i2c_read_regs(bmp280_addr, BMP280_REG_PRESS, d, 6);
    if (rc != I2C_OK)
        return rc;

    *adc_p = ((unsigned int)d[0] << 12) | ((unsigned int)d[1] << 4) |
             ((unsigned int)d[2] >> 4);
    *adc_t = ((unsigned int)d[3] << 12) | ((unsigned int)d[4] << 4) |
             ((unsigned int)d[5] >> 4);
    return I2C_OK;
}

/* Bosch integer temperature compensation: 0.01 C steps. Also latches
 * t_fine for bmp280_comp_pressure(). */
static int bmp280_comp_temp_x100(unsigned int adc_t)
{
    bmp_s64 var1, var2;

    var1 = ((((bmp_s64)adc_t >> 3) - ((bmp_s64)bmp280_cal.t1 << 1)) *
            bmp280_cal.t2) >> 11;
    var2 = ((((((bmp_s64)adc_t >> 4) - bmp280_cal.t1) *
              (((bmp_s64)adc_t >> 4) - bmp280_cal.t1)) >> 12) *
            bmp280_cal.t3) >> 14;
    bmp280_t_fine = (int)(var1 + var2);
    return (int)(((var1 + var2) * 5 + 128) >> 8);
}

/* Bosch integer pressure compensation: Pa. Call AFTER the temperature
 * compensation of the same measurement. Returns 0 (and would divide by
 * zero otherwise) if the coefficients degenerate. */
static unsigned int bmp280_comp_pressure_pa(unsigned int adc_p)
{
    bmp_s64 var1, var2, p;

    var1 = (bmp_s64)bmp280_t_fine - 128000;
    var2 = bmp_s64_mul(var1, var1);
    var2 = bmp_s64_mul(var2, bmp280_cal.p6);
    var2 = var2 + bmp_s64_mul(var1, (bmp_s64)bmp280_cal.p5 << 17);
    var2 = var2 + ((bmp_s64)bmp280_cal.p4 << 35);
    var1 = (bmp_s64_mul(bmp_s64_mul(var1, var1), bmp280_cal.p3) >> 8) +
           bmp_s64_mul(var1, (bmp_s64)bmp280_cal.p2 << 12);
    var1 = bmp_s64_mul(((bmp_s64)1 << 47) + var1, bmp280_cal.p1) >> 33;
    if (var1 == 0)
        return 0;  /* avoid the division by zero below */

    p = 1048576 - (bmp_s64)adc_p;
    p = bmp_s64_div(bmp_s64_mul((p << 31) - var2, 3125), var1);
    var1 = bmp_s64_mul(bmp_s64_mul(bmp280_cal.p9, p >> 13), p >> 13) >> 25;
    var2 = bmp_s64_mul(bmp280_cal.p8, p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((bmp_s64)bmp280_cal.p7 << 4);
    return (unsigned int)p;
}

/* One full measurement: start a forced conversion at the given
 * oversampling, wait for it, read the raw values and compensate both.
 * Returns I2C_OK (and fills both outputs) or the i2c error. */
static int bmp280_measure(int *temp_x100, unsigned int *press_pa)
{
    unsigned int adc_p = 0, adc_t = 0;
    int rc;

    rc = bmp280_start_forced(BMP280_OSRS_X16, BMP280_OSRS_X16);
    if (rc != I2C_OK)
        return rc;
    while (bmp280_busy())
        ;
    rc = bmp280_read_raw(&adc_p, &adc_t);
    if (rc != I2C_OK)
        return rc;

    *temp_x100 = bmp280_comp_temp_x100(adc_t);
    *press_pa = bmp280_comp_pressure_pa(adc_p);
    return I2C_OK;
}

#endif /* BMP280_H */
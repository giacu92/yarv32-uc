#ifndef MCP3208_H
#define MCP3208_H

/*
 * MCP3208 / MCP3008 SPI ADC on top of common/spi.h.
 *
 * MCP3208 is 12-bit, MCP3008 10-bit; they share the transaction framing,
 * so this driver covers both -- mcp3208_read() returns the raw code and
 * MCP3208_MAX tells you the full-scale value. Set MCP3208_BITS to 10
 * before including for the MCP3008.
 *
 * Transaction: SPI mode 0,0, MSB first, three bytes with CS low.
 *
 *     tx[0] = 0000 01 S D2      S = 1 single-ended, 0 pseudo-differential
 *     tx[1] = D1 D0 xxxxxx      channel select, in the top two bits
 *     tx[2] = 0                 clocks the result out
 *
 *     result = ((rx[1] & 0x0F) << 8) | rx[2]        (12-bit)
 *
 * The sample-and-hold closes during tx[0] and the conversion runs while
 * tx[1]/tx[2] shift, which is why the channel has to be in the FIRST two
 * bytes and why a single 24-clock burst is all it takes.
 *
 * SPEED. The datasheet ties fSAMPLE to fCLK: 100 ksps needs 2 MHz SCLK at
 * Vdd = 5 V, and at 2.7 V the part is limited to 50 ksps / 1 MHz. Going
 * faster than the part allows does not fail loudly -- it quietly returns
 * codes that have not finished converting. mcp3208_begin() takes the SCLK
 * you want so the choice stays explicit.
 *
 * AUDIO USE. The MCP3208 is unipolar: it measures 0..Vref, so an AC signal
 * must be biased to mid-rail before it reaches the input. mcp3208_centred()
 * subtracts the mid-scale code and returns a SIGNED sample, which is what
 * a DSP path wants; it does not remove the residual DC of a real bias
 * network, so a DC-blocking step in software is still worth doing (see
 * sw/guitar_tuner for one).
 *
 * CS framing is this driver's own: each read is a complete
 * select/transfer/deselect, because the MCP3208 needs CS to rise between
 * conversions.
 */

#include "spi.h"

#ifndef MCP3208_BITS
#define MCP3208_BITS 12u
#endif

#define MCP3208_MAX  ((1u << MCP3208_BITS) - 1u)
#define MCP3208_MID  (1 << (MCP3208_BITS - 1u))

/* Configure the SPI master for the part: mode 0, MSB first, SCLK as
 * given. Call once; it is just spi_begin_hz with the right mode. */
static void mcp3208_begin(unsigned int spi_hz)
{
    spi_begin_hz(spi_hz, 0);
    spi_deselect();
}

/* One single-ended conversion on channel 0..7. Returns the raw code,
 * 0..MCP3208_MAX. */
static unsigned int mcp3208_read(unsigned int ch)
{
    unsigned char hi, lo;

    spi_select();
    (void)spi_transfer((unsigned char)(0x06u | ((ch >> 2) & 0x01u)));
    hi = spi_transfer((unsigned char)((ch & 0x03u) << 6));
    lo = spi_transfer(0x00u);
    spi_deselect();

    return (((unsigned int)hi & 0x0Fu) << 8) | (unsigned int)lo;
}

/* Same conversion, returned as a signed sample centred on mid-scale:
 * -2048..+2047 for a 12-bit part. */
static int mcp3208_centred(unsigned int ch)
{
    return (int)mcp3208_read(ch) - MCP3208_MID;
}

/* Pseudo-differential pair: 0 = CH0-CH1, 1 = CH2-CH3, 2 = CH4-CH5,
 * 3 = CH6-CH7. Still returns a unipolar code -- the part clamps at 0 if
 * IN- exceeds IN+. */
static unsigned int mcp3208_read_diff(unsigned int pair)
{
    unsigned char hi, lo;
    unsigned int ch = pair * 2u;

    spi_select();
    (void)spi_transfer((unsigned char)(0x04u | ((ch >> 2) & 0x01u)));
    hi = spi_transfer((unsigned char)((ch & 0x03u) << 6));
    lo = spi_transfer(0x00u);
    spi_deselect();

    return (((unsigned int)hi & 0x0Fu) << 8) | (unsigned int)lo;
}

#endif /* MCP3208_H */

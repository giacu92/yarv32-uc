#ifndef GPIO_H
#define GPIO_H

/*
 * Arduino-style driver for the axi4_lite_gpio peripheral
 * (src/rtl/utils/axi4_lite_gpio.sv) at rv32_pkg::GPIO_BASE = 0x1000_7000.
 *
 * Register map (word offsets from the peripheral base):
 *   0x00 VALUE      (R)    : pin inputs (already synchronized by the top)
 *   0x04 OUT        (RW)   : output data
 *   0x08 DIR        (RW)   : 1 = output (pad driven), 0 = input (released).
 *                            Reset 0: every pad released, so a freshly
 *                            booted board never drives an external line.
 *   0x0C INT_TYPE   (RW)   : 2 bits per pin, at [2p+1:2p]:
 *                              00 level-high, 01 rise, 10 fall, 11 both.
 *                            Reset 00 (level).
 *   0x10 INT_EN     (RW)   : per-pin interrupt enable. Reset 0.
 *   0x14 INT_STATUS (R/W1C): per-pin pending; latches REGARDLESS of INT_EN
 *                            (PLIC-style: the event records, the enable only
 *                            gates the IRQ output), so a test can poll
 *                            status without arming.
 *
 * gpio_irq_o = |(INT_STATUS & INT_EN) -- level while pending and enabled.
 *
 * RESET CONSEQUENCE, and it will bite if ignored: the board pads pull up
 * (PULL_MODE=UP) and INT_TYPE resets to level-high, so INT_STATUS reads
 * all-1s out of reset. The W1C alone CANNOT clear those bits -- a
 * level-high pin re-pends the cycle after any W1C. Firmware that wants
 * edges must switch the pin's INT_TYPE to an edge type FIRST and then W1C
 * the stale bits, exactly what gpio_int_arm() below does. W1C-then-retype
 * with the type still at level fires a phantom interrupt.
 *
 * Pin numbering is 0..WIDTH-1 (the board exposes 4 header pins,
 * gpio_io[3:0] -> PIN41/42/71/72). Helpers that take a pin index build
 * the mask/field position; *_all helpers operate on the whole word.
 */

#define GPIO_BASE 0x10007000u

#define GPIO_VALUE      (*(volatile unsigned int *)(GPIO_BASE + 0x00))
#define GPIO_OUT        (*(volatile unsigned int *)(GPIO_BASE + 0x04))
#define GPIO_DIR        (*(volatile unsigned int *)(GPIO_BASE + 0x08))
#define GPIO_INT_TYPE   (*(volatile unsigned int *)(GPIO_BASE + 0x0C))
#define GPIO_INT_EN     (*(volatile unsigned int *)(GPIO_BASE + 0x10))
#define GPIO_INT_STATUS (*(volatile unsigned int *)(GPIO_BASE + 0x14))

/* Direction modes (gpio_pin_mode). */
#define GPIO_INPUT  0u
#define GPIO_OUTPUT 1u

/* Interrupt types (gpio_int_config / gpio_int_arm), 2 bits per pin. */
#define GPIO_INT_LEVEL_HIGH 0u
#define GPIO_INT_RISE       1u
#define GPIO_INT_FALL       2u
#define GPIO_INT_BOTH       3u

/* ------------------------------------------------------------------ */
/* Basic pin control (digitalWrite / pinMode / digitalRead)           */
/* ------------------------------------------------------------------ */

static void gpio_pin_mode(unsigned int pin, unsigned int mode)
{
    unsigned int mask = 1u << pin;
    if (mode)
        GPIO_DIR |= mask;   /* output: drive the pad */
    else
        GPIO_DIR &= ~mask;  /* input: release the pad */
}

static void gpio_digital_write(unsigned int pin, unsigned int val)
{
    unsigned int mask = 1u << pin;
    if (val)
        GPIO_OUT |= mask;
    else
        GPIO_OUT &= ~mask;
}

static unsigned int gpio_digital_read(unsigned int pin)
{
    return (GPIO_VALUE >> pin) & 1u;
}

/* Whole-word variants. */
static void gpio_write_all(unsigned int mask)
{
    GPIO_OUT = mask;
}

static unsigned int gpio_read_all(void)
{
    return GPIO_VALUE;
}

/* ------------------------------------------------------------------ */
/* Interrupt configuration                                            */
/* ------------------------------------------------------------------ */

/* Set one pin's INT_TYPE field (2 bits at [2p+1:2p]). */
static void gpio_int_config(unsigned int pin, unsigned int type)
{
    unsigned int shift = 2u * pin;
    unsigned int field = 3u << shift;
    GPIO_INT_TYPE = (GPIO_INT_TYPE & ~field) | ((type & 3u) << shift);
}

/* Per-pin interrupt enable / disable (does not touch pending bits). */
static void gpio_int_enable(unsigned int pin)
{
    GPIO_INT_EN |= (1u << pin);
}

static void gpio_int_disable(unsigned int pin)
{
    GPIO_INT_EN &= ~(1u << pin);
}

/* Read the pending record (latched events, regardless of INT_EN). */
static unsigned int gpio_int_status(void)
{
    return GPIO_INT_STATUS;
}

/* Write-1-to-clear pending pins. On a pin whose condition still holds
 * (a level-high pin) the clear does not stick -- see the header comment. */
static void gpio_int_clear(unsigned int mask)
{
    GPIO_INT_STATUS = mask;
}

/* Arm a pin for interrupts the safe way: retype to an edge, then clear the
 * stale pending bits, then enable. This order is load-bearing -- see the
 * RESET CONSEQUENCE note above. For a level interrupt the caller is
 * responsible for the level being meaningful before calling this. */
static void gpio_int_arm(unsigned int pin, unsigned int type)
{
    gpio_int_config(pin, type);
    gpio_int_clear(1u << pin);  /* after the retype, so it sticks */
    gpio_int_enable(pin);
}

/* Disarm a pin and clear its pending bit (ISR tail for "done with this"). */
static void gpio_int_disarm(unsigned int pin)
{
    gpio_int_disable(pin);
    gpio_int_clear(1u << pin);
}

#endif /* GPIO_H */
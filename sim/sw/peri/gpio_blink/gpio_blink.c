#ifndef GPIO_BLINK_C
#define GPIO_BLINK_C

/*
 * GPIO0 blink, one second period (500 ms on / 500 ms off): the
 * hello-world of common/gpio.h + timer.h.
 *
 * On the board GPIO0 is header pin PIN41 and, since top_module now wires
 * LED0 to GPIO0, the onboard LED blinks with it. In the sim the pin
 * loops back with pull-up semantics and the blink is only visible in a
 * VCD; the UART banner proves the firmware runs.
 *
 * Infinite loop: never parks, a sim run goes to MAX_CYC (expected).
 */

#include "gpio.h"
#include "timer.h"
#include "uart.h"

int main(void)
{
    uart_puts("GPIO BLINK GPIO0 1Hz\r\n");

    gpio_pin_mode(0, GPIO_OUTPUT);

    for (;;) {
        gpio_digital_write(0, 1);
        delay_ms(500);
        gpio_digital_write(0, 0);
        delay_ms(500);
    }

    /* not reached */
    return 0;
}

#endif /* GPIO_BLINK_C */
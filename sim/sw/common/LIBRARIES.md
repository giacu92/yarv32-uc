# Firmware libraries (sim/sw/common)

A small Arduino-flavoured C library set for the core's peripherals. All
headers are header-only: everything is `static`, there is no `.c` file to
add and no dependency between them except where noted. Include them from
any firmware built with `sw_build.mk` (`-I` is already set).

Style, common to all of them:

- **Polling, not interrupts, by default.** Every blocking wait is a busy
  loop. Each library exposes optional IRQ plumbing, but nothing arms an
  interrupt unless you call the `*_irq_enable`-style functions.
- **No malloc, no libc.** Everything works in the freestanding
  `-nostdlib` build; `bmp280.h` even carries its own 64-bit arithmetic
  because libgcc is not linked.
- **One core-clock assumption.** The `*_CORE_HZ` defaults are 50 MHz
  (the shipping rPLL clock). Override with `-DXXX_CORE_HZ=...` if that
  ever changes; the `*_hz()` init functions are the ones that need it.
- **Include order matters only for `sys.h`** (see below): include it in
  exactly one translation unit per program.

Peripheral base addresses (from `rv32_pkg.sv`):

| Peripheral | Base        | Header    |
|------------|-------------|-----------|
| UART       | 0x1000_0000 | `uart.h`  |
| CLINT      | 0x1000_1000 | `timer.h` |
| MSIP       | 0x1000_3000 | —         |
| I2C        | 0x1000_5000 | `i2c.h`   |
| SPI        | 0x1000_6000 | `spi.h`   |
| GPIO       | 0x1000_7000 | `gpio.h`  |
| PLIC       | 0x1000_8000 | `plic.h`  |
| FFT        | 0x1000_A000 | `fft.h`   |

Device drivers sit on top of those: `bmp280.h`, `ds3231.h`, `ssd1306.h`
(+ `ssd1306_text.h` for a font) and `w25q.h` over I2C/SPI, and
`mcp3208.h` for the SPI ADC.

---

## uart.h

The serial console: 115200 8N1 on the board. Nothing to init — the
peripheral is usable out of reset (BAUDDIV is preset by the top that
instantiates it).

```c
#include "uart.h"

uart_puts("hello\r\n");
char c = uart_getc();   /* blocking */
uart_put_hex32(0x600D); /* prints 0x0000600d */
```

`STATUS` polling only; the FIFOs (16 deep each way) are behind the
functions. There is no IRQ API here — for the MEIP-through-UART example
see `sim/sw/peri/uart_echo`.

## i2c.h

I2C master, Arduino-`Wire`-like but with the register model made explicit.

```c
#include "i2c.h"

i2c_begin_hz(100000);                    /* 100 kHz standard bus */

unsigned char who;
i2c_read_reg(0x50, 0x00, &who);          /* one register */

i2c_write_reg(0x50, 0x00, 0xAB);         /* one register */

unsigned char buf[4];
i2c_write_regs(0x50, 0x00, src, 4);      /* burst write, pointer auto-inc */
i2c_read_regs(0x50, 0x00, buf, 4);       /* set-pointer + repeated-START read */

i2c_read(0x50, buf, 8);                  /* raw read, no register phase */
i2c_write(0x50, buf, 8);                 /* raw write, any length */

i2c_probe(0x68);                         /* I2C_OK if a device ACKs */
```

All transactions return `I2C_OK` (0) or `I2C_ERR_NACK` (-1) /
`I2C_ERR_ARB` (-2). A NACKed write is aborted to STOP with the TX FIFO
drained, so the next transaction starts clean. Writes longer than the
16-byte TX FIFO work: bytes are fed while the engine drains them.

In the sim, a register-file slave model answers at address 0x50
(`sim/i2c_slave_model.sv`, 16 registers, first byte = pointer). The
oracles `sim/sw/peri/i2c_reg` (register model) and `sim/sw/peri/i2c_scan`
(bus scanner) exercise all of this end-to-end.

## spi.h

SPI master, mode 0-3, caller-owned CS.

```c
#include "spi.h"

spi_begin_hz(1000000, 0);   /* 1 MHz, mode 0 */

spi_select();               /* CS low (plain register bit, not automatic) */
unsigned char echo = spi_transfer(0xA5);  /* one byte full-duplex */
spi_deselect();             /* CS high */

unsigned char out[40], in[40];
spi_select();
spi_transfer_buf(out, in, 40);  /* runs at wire speed, up to 16 in flight */
spi_deselect();

spi_send(out, 8);           /* write-only */
spi_read(in, 8);            /* read-only: clocks 0xFF fillers */

unsigned int id = spi_read32();          /* MSB-first helpers */
```

The engine is free-running: one RX byte per TX byte, so a "read" is a
write of 0xFF fillers. CS framing is deliberately manual — wrap one
device transaction in `select`/`deselect` and make as many transfers as
you like in between. In the sim, MISO is looped to MOSI
(`sim_top.sv`); the oracle is `sim/sw/peri/spi_loop`.

## gpio.h

Push-pull GPIO with edge/level interrupts, Arduino-pin-style.

```c
#include "gpio.h"

gpio_pin_mode(0, GPIO_OUTPUT);
gpio_digital_write(0, 1);
gpio_pin_mode(1, GPIO_INPUT);
int v = gpio_digital_read(1);

/* Interrupts: ALWAYS use gpio_int_arm, never the pieces separately.
 * Out of reset INT_TYPE is level-high and the (pulled-up) pins read 1,
 * so INT_STATUS starts all-ones; arm() retypes the pin, W1Cs the stale
 * bit, then enables -- in that order, or the line never clears. */
gpio_int_arm(0, GPIO_INT_RISE);
unsigned int st = gpio_int_status();
gpio_int_clear(0x1);        /* W1C: the service that drops the line */
gpio_int_disarm(0);
```

The board exposes 4 header pins (`gpio_io[3:0]`, pulled up). In the sim
the pins are looped back with pull-up semantics, so firmware can drive
and read its own edges — that is what `sim/sw/peri/plic_gpio` does
end-to-end through the PLIC.

## sys.h

CSR access and the interrupt-vector dispatcher. **Include it in exactly
one `.c` file per program** (it defines `static` state: the ISR table and
the entry trampoline).

```c
#include "sys.h"

sys_irq_global_enable();            /* mstatus.MIE */

sys_interrupt_attach(SYS_MCAUSE_MEI, my_isr);   /* direct-mode mtvec */
sys_interrupt_detach(SYS_MCAUSE_MEI);

unsigned int cause = sys_cause_code(mcause);
if (sys_cause_is_interrupt(mcause)) { ... }

sys_cycle();                        /* mcycle (32-bit: wraps ~86 s @ 50 MHz) */
sys_wfi();                          /* sleep until an enabled interrupt */
```

`sys_isr_entry` saves nothing beyond what hardware gives you (mepc,
mcause, mstatus are automatic); it dispatches on the mcause code to
`sys_isr_tab`, and an unhandled cause calls `sys_isr_unhandled` if you
set one (`sys_isr_set_unhandled`) — otherwise it masks MIE and returns,
so an unexpected interrupt happens exactly once. Handlers are plain C
functions; the `interrupt("machine") aligned(4)` attribute that the ABI
wants is applied by the header — do not add it again.

Note the core's CSR read is registered: a CSR read returns the previous
cycle's value, so `sys_cycle()` twice in a row is the reliable way to
time a single cycle.

## timer.h

Two unrelated things share this header: the CLINT wall-clock timer and
`delay`-style spin waits.

```c
#include "timer.h"

delay_ms(100);                  /* busy wait on mcycle, wrap-safe */
unsigned int us = micros();     /* 32-bit: wraps ~86 s @ 50 MHz */
unsigned int ms = millis();

timer_arm_us(1000);             /* mtimecmp = mtime + 1000 us (safe order) */
timer_irq_enable();             /* mie.MTIE; then trap on MTIP */
if (timer_pending()) { ... }    /* mip.MTIP */
timer_cancel();
```

`timer_arm_us` writes mtimecmp high=0xFFFFFFFF, then low, then the real
high — the order that avoids a spurious compare against a half-written
comparator. `mtime` is 64-bit and does not wrap.

## plic.h

The external-interrupt cause/claim layer over the PLIC. Needs `sys.h`
(so: one TU, and include `sys.h` first or let it pull it in).

```c
#include "plic.h"

/* Source IDs: PLIC_SRC_UART=1, PLIC_SRC_I2C=2, PLIC_SRC_SPI=3,
 * PLIC_SRC_GPIO=4, PLIC_SRC_FFT=5 (rv32_pkg.sv). */
plic_attach(PLIC_SRC_UART, uart_rx_isr);
sys_irq_global_enable();        /* after attaching */

/* Inside the ISR: CLAIM is side-effect-free; "claim = complete" is your
 * service routine dropping the source's line (e.g. gpio_int_clear, or
 * draining a FIFO). Claim without service = livelock: you re-take the
 * same interrupt forever. */
unsigned int src = plic_claim();
```

`plic_attach` hooks one dispatcher at mcause MEI; it claims once per
entry, calls your handler, and lets anything still pending re-take after
`mret`. `plic_detach(src)` removes the handler. MSIP/MTIP do not pass
through the PLIC — they are CLINT-direct.

## fft.h

The FFT coprocessor: radix-2 decimation-in-frequency, signed Q15 complex,
4 to 1024 points, one butterfly per cycle. Samples live in the
peripheral's own BSRAM, not in D-mem.

One-shot, ping-pong hidden:

```c
#include "fft.h"

short win[256], re[256], im[256];

fft_begin(8);                       /* 256 points, 1/N scaling, forward */
fft_transform_real(win, re, im);    /* fill, start, wait, swap, read */

unsigned int k = fft_peak_bin(1, 128);          /* skip DC, skip the mirror */
unsigned int hz = fft_bin_hz(k, 48000, 256);    /* bin -> Hz */
```

**Streaming, and the order is not negotiable.** There are two sample
buffers and you only ever see one. `fft_start()` hands the buffer you
filled to the engine *and* hands you back the one holding the PREVIOUS
result, so the loop body is **wait, start, read, fill** — reading after
filling loses the result, filling before starting overwrites it:

```c
fft_begin(8);
fft_fill_real(frame_a, 256);
fft_start();                    /* prime: engine has A */
fft_fill_real(frame_b, 256);    /* fill B while A transforms */

for (;;) {
    fft_wait();                 /* the frame in flight is done */
    fft_start();                /* launch the one just filled */
    fft_read_all(re, im, 256);  /* ... and read the previous result */
    fft_fill_real(next, 256);   /* into the buffer just freed */
}
```

`fft_swap()` drains the last frame without launching another transform.

Numbers: Q15 throughout (-1.0 = -32768, +1.0-2^-15 = 32767). `SCALE`
divides by 2 after each stage; the all-ones default is 1/N overall and
cannot overflow, and every write saturates rather than wrapping. A real
cosine of amplitude A at bin k comes back as A/2 at k and A/2 at N-k.
Output order is natural — no bit-reversal to undo. `fft_power()` is exact
(re²+im²); `fft_mag()` is the sqrt-free max+3·min/8 approximation, within
6.8%, good for peak picking and not for calibrated measurement.

Cost: `(LOG2N + odd) × (N/2 + 4)` cycles — about 21 µs for 256 points and
103 µs for 1024 at 50 MHz. For large N the MMIO **fill** costs more than
the transform, so budget that first.

DONE raises PLIC source 5; `fft_irq_enable()` arms it and
`fft_irq_clear()` is what completes it (the PLIC's CLAIM read is
side-effect-free, so a handler that skips it re-takes forever). Worked
example: `sim/sw/peri/fft`.

## mcp3208.h

MCP3208 (12-bit) / MCP3008 (10-bit) SPI ADC. Same framing for both; set
`MCP3208_BITS` to 10 before including for the smaller part.

```c
#include "mcp3208.h"

mcp3208_begin(1000000);              /* 1 MHz SCLK, SPI mode 0 */

unsigned int code = mcp3208_read(0);     /* raw, 0..MCP3208_MAX */
int          s    = mcp3208_centred(0);  /* signed, -2048..+2047 */
```

Each read is its own select/transfer/deselect -- the part needs CS to
rise between conversions. **The part is unipolar**: an audio signal has
to be biased to Vref/2 outside the chip or the whole negative half is
lost, and `mcp3208_centred()` only removes the mid-scale code, not the
residual DC of a real bias network. `mcp3208_read_diff(pair)` does the
pseudo-differential pairs. Datasheet speed limits are real and fail
quietly: exceeding them returns codes that have not finished converting,
so `mcp3208_begin()` makes the SCLK an explicit argument.

## ssd1306_text.h

Text for the OLED. Include it **instead of** `ssd1306.h` -- it pulls the
driver in, and it is separate only because the font costs ~300 bytes of
rodata that a graphics-only program should not pay.

```c
#include "ssd1306_text.h"

ssd1306_text(0, 0, "TUNER", 1);                /* 6x8 cells */
ssd1306_text_scaled(2, 16, "E2", 2, 1);        /* double height */
ssd1306_int(60, 0, -12, 4, 1, 1);              /* right-aligned in 4 cells */
ssd1306_fixed(0, 24, 8241, 2, 1, 1);           /* prints "82.41" */
unsigned int w = ssd1306_text_width("IN TUNE", 1);   /* for centring */
```

5x7 glyphs in a 6x8 cell, so 21x8 characters on a 128x64 panel (10x4 at
scale 2). **ASCII 0x20..0x5A only -- no lowercase**; anything outside the
range renders as a space rather than as garbage, so a stray byte cannot
walk off the table. `ssd1306_fixed` is integer-only decimal: there is no
FPU and no libc here.

## Device drivers

Four device headers, all on top of `i2c.h`/`spi.h`. They are
compile-tested and logic-checked (the BMP280 compensation is verified
bit-exact against the Bosch reference formula) but there is **no device
model in the sim** — exercise them on the board, or against the slave
model where the register model happens to match.

### w25q.h — Winbond SPI NOR flash

```c
#include "w25q.h"

w25q_begin_hz(33000000);          /* one clock for read AND program */
if (!w25q_present()) { ... }      /* JEDEC ID manufacturer == 0xEF */
unsigned int id = w25q_jedec_id();

w25q_read(0x000000, buf, 256);    /* sequential read, crosses pages */

w25q_erase_sector(0x000000);      /* 4 KiB; sets bits to 1 */
w25q_write(0x000000, buf, 300);   /* splits at page boundaries itself,
                                     blocks until done */
```

Programming can only clear bits (write into erased 0xFF), erase is 4 KiB
minimum — the driver does not hide either fact.

### ssd1306.h — 128x64 OLED, I2C

```c
#include "ssd1306.h"

i2c_begin_hz(400000);             /* these modules take fast mode */
ssd1306_begin(0x3C);              /* 0x3D if SA0 is strapped high */

ssd1306_clear();
ssd1306_pixel(64, 32, 1);
ssd1306_line(0, 0, 127, 63, 1);
ssd1306_rect(10, 10, 20, 20, 1);
ssd1306_update();                 /* pushes the 1 KiB framebuffer */
```

The framebuffer (`ssd1306_fb`, 1024 bytes of the 16 KiB D-mem) is
page-major; `update()` sends it as one I2C transaction. Commands and
data are distinguished by the control byte (0x00/0x40), which rides in
the "register" position of `i2c_write_regs`.

### ds3231.h — RTC

```c
#include "ds3231.h"

if (ds3231_time_invalid()) {      /* first fit of the battery: time lost */
    struct ds3231_time t = { 30, 45, 13, 6, 14, 9, 26 };
    ds3231_set_time(&t);
}

struct ds3231_time now;
ds3231_get_time(&now);            /* binary fields, 24h, dow 0=Sunday */

int quarter_c = ds3231_get_temp_x4(0);   /* chip temp, 0.25 C steps */
```

BCD conversion is internal; the struct is plain numbers.

### bmp280.h — barometric sensor

```c
#include "bmp280.h"

i2c_begin_hz(400000);
if (bmp280_begin(BMP280_ADDR_A))   /* 0x76; try _B (0x77) on failure */
    bmp280_begin(BMP280_ADDR_B);

int t_x100;                        /* 0.01 C steps */
unsigned int pa;
bmp280_measure(&t_x100, &pa);      /* forced mode: one shot, waits, both */
```

One measurement per call (forced mode). Temperature must be compensated
before pressure — `bmp280_measure` does that for you; only split
`bmp280_comp_temp_x100` / `bmp280_comp_pressure_pa` yourself if you are
caching `t_fine`. No FPU and no libgcc: the Bosch integer algorithm runs
on the driver's own 64-bit multiply/divide.

---

## Recipes

**Poll everything (simplest):**

```c
uart_init();
i2c_begin_hz(100000);
spi_begin_hz(1000000, 0);
/* ... loop on i2c_read_regs / spi_transfer ... */
```

**Interrupts, the full stack:**

```c
/* one.c — the ONLY file including sys.h / plic.h */
#include "plic.h"

/* plic_attach handlers take the source ID the dispatcher claimed. */
static void gpio_isr(unsigned int src)
{
    (void)src;
    if (gpio_int_status() & 0x1)
        gpio_int_clear(0x1);      /* service = drop the line */
}

int main(void)
{
    gpio_int_arm(0, GPIO_INT_RISE);
    plic_attach(PLIC_SRC_GPIO, gpio_isr);
    sys_irq_global_enable();      /* mstatus.MIE; PLIC ENABLE is all-ones
                                     out of reset, meip flows already */
    for (;;)
        sys_wfi();
}
```

**FFT under interrupt:**

```c
/* one.c — the ONLY file including sys.h / plic.h */
#include "fft.h"
#include "plic.h"

static volatile int ready;

static void fft_isr(unsigned int src)
{
    (void)src;
    fft_irq_clear();              /* service = drop the line */
    ready = 1;
}

int main(void)
{
    fft_begin(8);
    sys_isr_install();
    plic_attach(PLIC_SRC_FFT, fft_isr);
    sys_irq_source_enable(SYS_MIE_MEIE);
    sys_irq_global_enable();
    fft_irq_enable();

    fft_fill_real(frame, 256);
    fft_start();
    while (!ready)
        sys_wfi();
}
```

**Sim oracles:** self-checking firmware that ends with a `0x600D`/`0xBAD`
marker at a D-mem address, then parks in a self-loop. See any directory
under `sim/sw/peri/` for the Makefile pattern; `make regress` (from
`sim/`) runs them all.
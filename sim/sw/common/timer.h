#ifndef TIMER_H
#define TIMER_H

/*
 * Timing: mcycle-based delays (Arduino delay()/micros()) plus the CLINT
 * machine timer (mtime / mtimecmp) for scheduled and periodic interrupts.
 *
 * Two counters, two jobs:
 *
 *   mcycle (CSR)   free-running cycle count, low 32 bits only
 *                  (mcycleh is not implemented -- see sys.h). Wraps in
 *                  ~85.9 s at 50 MHz. delay_us/delay_ms/micros are built
 *                  on it and are wrap-safe for any single delay below the
 *                  wrap. Good for short waits and micro-timing; NOT good
 *                  for a clock that must survive 86 seconds.
 *
 *   mtime (CLINT)  a real 64-bit counter in the machine-timer peripheral
 *                  (clint_timer.sv, rv32_pkg::MTIMER_BASE = 0x1000_1000),
 *                  exposed as four 32-bit MMIO words. mtimecmp arms the
 *                  machine timer interrupt: mtip = (mtime >= mtimecmp),
 *                  taken when mstatus.MIE & mie.MTIE (see sys.h). The
 *                  interrupt stays pending until software moves mtimecmp
 *                  back above mtime -- re-arm inside the handler.
 *
 * mtimecmp WRITE ORDER is load-bearing (clint_timer.sv): the 64-bit compare
 * is reachable only as two 32-bit words, so writing the halves in the
 * wrong order can transiently arm a value neither old nor new and fire a
 * spurious interrupt. timer_arm_us() does it in the safe order:
 * hi = 0xFFFF_FFFF, then lo, then the real hi.
 */

#include "sys.h"

#ifndef TIMER_CORE_HZ
#define TIMER_CORE_HZ 50000000u  /* must track the rPLL */
#endif

#define TIMER_CYCLES_PER_US (TIMER_CORE_HZ / 1000000u)

/* CLINT machine timer registers (word offsets from MTIMER_BASE). */
#define TIMER_BASE      0x10001000u
#define TIMER_MTIME_LO  (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_MTIME_HI  (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_MTIMECMP_LO (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_MTIMECMP_HI (*(volatile unsigned int *)(TIMER_BASE + 0x0C))

/* ------------------------------------------------------------------ */
/* mcycle-based delays                                                 */
/* ------------------------------------------------------------------ */

/* Busy-wait us microseconds. Wrap-safe via unsigned difference (any single
 * delay < ~85.9 s at 50 MHz). Interrupts are NOT masked: a handler that
 * runs inside the window makes the wait longer, not wrong. */
static void delay_us(unsigned int us)
{
    unsigned int wait = us * TIMER_CYCLES_PER_US;
    unsigned int t0 = sys_cycle();
    while ((sys_cycle() - t0) < wait)
        ;
}

static void delay_ms(unsigned int ms)
{
    while (ms--)
        delay_us(1000u);
}

/* Elapsed microseconds since some fixed point (the mcycle value at call
 * time). 32-bit: wraps at 2^32 / TIMER_CYCLES_PER_US microseconds
 * (~85.9 s at 50 MHz). Use for deltas, not wall time. */
static unsigned int micros(void)
{
    return sys_cycle() / TIMER_CYCLES_PER_US;
}

static unsigned int millis(void)
{
    return micros() / 1000u;
}

/* ------------------------------------------------------------------ */
/* CLINT machine timer (mtime / mtimecmp)                              */
/* ------------------------------------------------------------------ */

/* Read the 64-bit mtime (no wrap within any realistic run). */
static unsigned long long timer_mtime(void)
{
    unsigned int hi = TIMER_MTIME_HI;
    unsigned int lo = TIMER_MTIME_LO;
    /* Re-read hi: if the low half rolled between the two reads, hi must
     * have advanced -- take the second read so the pair is consistent. */
    unsigned int hi2 = TIMER_MTIME_HI;
    if (hi != hi2) {
        hi = hi2;
        lo = TIMER_MTIME_LO;
    }
    return ((unsigned long long)hi << 32) | lo;
}

/* Arm mtimecmp = mtime + delta_us, in the safe order (see header).
 * delta_us is scaled to cycles in 64-bit; up to ~71 minutes of delta at
 * 50 MHz fits without overflow of the multiply. */
static void timer_arm_us(unsigned int delta_us)
{
    unsigned long long target =
        timer_mtime() + (unsigned long long)delta_us * TIMER_CYCLES_PER_US;
    TIMER_MTIMECMP_HI = 0xFFFFFFFFu;  /* disarm hi first: no transient arm */
    TIMER_MTIMECMP_LO = (unsigned int)(target & 0xFFFFFFFFu);
    TIMER_MTIMECMP_HI = (unsigned int)(target >> 32);
}

/* Disarm: mtimecmp back to all-ones (the reset value, "never compares"). */
static void timer_cancel(void)
{
    TIMER_MTIMECMP_HI = 0xFFFFFFFFu;
    TIMER_MTIMECMP_LO = 0xFFFFFFFFu;
    TIMER_MTIMECMP_HI = 0xFFFFFFFFu;
}

/* mtip as software sees it: mip.MTIP, set while mtime >= mtimecmp. */
static int timer_pending(void)
{
    return (sys_mip() & (1u << 7)) != 0u;
}

/* Enable the machine timer interrupt: mie.MTIE (pair with
 * sys_irq_global_enable() and sys_interrupt_attach(SYS_MCAUSE_MTI, ...)
 * from sys.h; the handler must re-arm or cancel, or the level re-takes
 * after mret forever). */
static void timer_irq_enable(void)
{
    sys_irq_source_enable(SYS_MIE_MTIE);
}

static void timer_irq_disable(void)
{
    sys_irq_source_disable(SYS_MIE_MTIE);
}

#endif /* TIMER_H */
#ifndef PLIC_H
#define PLIC_H

/*
 * Driver for the sifive,plic-1.0.0-compatible interrupt controller
 * (src/rtl/utils/axi4_lite_plic_sifive.sv), linux-soc M8, at
 * rv32_pkg::PLIC_BASE = 0x1040_0000 (4 MiB window). Context 0 = M-mode
 * (meip); context 1 = S-mode (seip, unused until S-mode exists).
 *
 *   +0x000000 + 4*id        priority[id] (3 bits; 0 = never interrupts)
 *   +0x001000               pending bits
 *   +0x002000 + 0x80*ctx    enable bits
 *   +0x200000 + 0x1000*ctx  threshold; +4 = claim (read) / complete (write)
 *
 * Gateway semantics, as the real part: CLAIM returns the highest-priority
 * eligible source (lowest ID on a tie, 0 = none) and takes it out of
 * service; it is not presented again until COMPLETE is written with its ID,
 * even if its line stays high. A COMPLETE for a source not enabled in that
 * context is ignored (spec). Everything resets to 0: firmware must set a
 * priority and an enable bit before any source can interrupt -- unlike the
 * MCU PLIC, whose ENABLE reset to all-ones.
 *
 * Source IDs (rv32_pkg::PLIC_SRC_*): 1 UART, 3 SPI, 4 GPIO on this branch
 * (2 I2C, 5 FFT, 6 I2S are main-only; their IDs are kept so they never get
 * reused).
 *
 * Depends on sys.h (included): the dispatcher registers itself at the
 * machine-external cause (11), claims one source, calls its handler, and
 * completes it. Call sys_isr_install() once, then plic_attach() and
 * plic_enable() per source, then sys_irq_source_enable(SYS_MIE_MEIE) +
 * sys_irq_global_enable().
 */

#include "sys.h"

#define PLIC_BASE 0x10400000u
#define PLIC_CTX  0u  /* hart 0, M-mode */

#define PLIC_PRIORITY(id) (*(volatile unsigned int *)(PLIC_BASE + 4u * (id)))
#define PLIC_PENDING      (*(volatile unsigned int *)(PLIC_BASE + 0x1000u))
#define PLIC_ENABLE       (*(volatile unsigned int *)(PLIC_BASE + 0x2000u + 0x80u * PLIC_CTX))
#define PLIC_THRESHOLD    (*(volatile unsigned int *)(PLIC_BASE + 0x200000u + 0x1000u * PLIC_CTX))
#define PLIC_CLAIM        (*(volatile unsigned int *)(PLIC_BASE + 0x200004u + 0x1000u * PLIC_CTX))
#define PLIC_COMPLETE     PLIC_CLAIM

/* Source IDs -- must match rv32_pkg::PLIC_SRC_* (the single source of
 * truth). */
#define PLIC_SRC_NONE 0u
#define PLIC_SRC_UART 1u
#define PLIC_SRC_I2C  2u
#define PLIC_SRC_SPI  3u
#define PLIC_SRC_GPIO 4u
#define PLIC_SRC_FFT  5u
#define PLIC_SRC_I2S  6u
#define PLIC_SRC_N    16u

/* ------------------------------------------------------------------ */
/* Source plumbing                                                     */
/* ------------------------------------------------------------------ */

/* Gateway pending bits (latched; cleared by a claim). */
static unsigned int plic_pending(void)
{
    return PLIC_PENDING;
}

/* Enable a source into meip: priority 1 (above threshold 0) + enable bit.
 * This is exactly what Linux's plic_irq_unmask() does. */
static void plic_enable(unsigned int src)
{
    PLIC_THRESHOLD = 0u;
    PLIC_PRIORITY(src) = 1u;
    PLIC_ENABLE |= (1u << src);
}

static void plic_disable(unsigned int src)
{
    PLIC_ENABLE &= ~(1u << src);
    PLIC_PRIORITY(src) = 0u;
}

/* Claim: highest-priority eligible source, 0 if none. Takes the source out
 * of service until plic_complete(). */
static unsigned int plic_claim(void)
{
    return PLIC_CLAIM & 0xFFu;
}

static void plic_complete(unsigned int src)
{
    PLIC_COMPLETE = src;
}

/* ------------------------------------------------------------------ */
/* Per-source handler dispatch (the attachInterrupt layer)             */
/* ------------------------------------------------------------------ */

typedef void (*plic_handler_fn)(unsigned int src);

static plic_handler_fn plic_tab[PLIC_SRC_N];

/* The machine-external dispatcher: claim once, call the registered handler
 * (if any), complete. The handler must still service its source (drop the
 * line) -- otherwise the source re-pends right after the complete. */
static void plic_mei_isr(void)
{
    unsigned int src = plic_claim();
    if (src != PLIC_SRC_NONE && src < PLIC_SRC_N && plic_tab[src])
        plic_tab[src](src);
    if (src != PLIC_SRC_NONE)
        plic_complete(src);
}

/* Register a handler for a source ID and hook the dispatcher into the
 * machine-external cause. Does not enable anything -- pair with
 * sys_irq_source_enable(SYS_MIE_MEIE) and sys_irq_global_enable(). */
static void plic_attach(unsigned int src, plic_handler_fn fn)
{
    if (src < PLIC_SRC_N) {
        plic_tab[src] = fn;
        sys_interrupt_attach(SYS_MCAUSE_MEI, plic_mei_isr);
    }
}

static void plic_detach(unsigned int src)
{
    if (src < PLIC_SRC_N)
        plic_tab[src] = 0;
}

#endif /* PLIC_H */
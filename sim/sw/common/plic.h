#ifndef PLIC_H
#define PLIC_H

/*
 * Cause/claim layer over the axi4_lite_plic interrupt controller
 * (src/rtl/utils/axi4_lite_plic.sv) at rv32_pkg::PLIC_BASE = 0x1000_8000.
 *
 * Register map (word offsets from the peripheral base):
 *   0x00 PENDING (R)    : bit i = source i pending. THE LINES THEMSELVES,
 *                         combinationally -- every source is a level whose
 *                         edge-ness is latched where it belongs (GPIO
 *                         INT_STATUS, the UART/I2C/SPI conditions), so
 *                         there is no claim-side state.
 *   0x04 ENABLE  (RW)   : bit i = source i enabled into meip. RESETS
 *                         ALL-ONES (meip degenerates to the OR of the
 *                         lines), so pre-PLIC firmware runs unchanged.
 *   0x08 CLAIM   (R)    : lowest-ID enabled pending source. The read is
 *                         SIDE-EFFECT-FREE: "claim = complete" is the
 *                         source's own service mechanism dropping its line
 *                         (e.g. W1C the GPIO INT_STATUS bit, drain the
 *                         UART RX FIFO).
 *
 * Source IDs (rv32_pkg::PLIC_SRC_*): 0 reserved ("nothing pending", CLAIM
 * returning 0), 1 UART, 2 I2C, 3 SPI, 4 GPIO, 5 FFT, 6-15 reserved (tied
 * off).
 * Fixed hardware priority LOWER ID WINS; no per-source priority registers.
 *
 * THE LIVELOCK CONTRACT (pinned by sim/hw/plic_tb): a source whose line is
 * still high pends again immediately, so CLAIM returns the same ID and the
 * core re-takes the interrupt after mret. An ISR that claims without
 * servicing spins forever -- service the source (drop its line) inside the
 * handler, every time. plic_attach()'s dispatcher claims ONE source per
 * interrupt entry; if several are pending the others re-take after mret,
 * which is correct and costs one entry each.
 *
 * Depends on sys.h (included): the dispatcher registers itself at the
 * machine-external cause (11) and calls the per-source handler. Call
 * sys_isr_install() once (sys.h), then plic_attach() per source, then
 * sys_irq_source_enable(SYS_MIE_MEIE) + sys_irq_global_enable().
 */

#include "sys.h"

#define PLIC_BASE 0x10008000u

#define PLIC_PENDING (*(volatile unsigned int *)(PLIC_BASE + 0x00))
#define PLIC_ENABLE  (*(volatile unsigned int *)(PLIC_BASE + 0x04))
#define PLIC_CLAIM   (*(volatile unsigned int *)(PLIC_BASE + 0x08))

/* Source IDs -- must match rv32_pkg::PLIC_SRC_* (the single source of
 * truth for the tops' irq_i concatenation). */
#define PLIC_SRC_NONE 0u
#define PLIC_SRC_UART 1u
#define PLIC_SRC_I2C  2u
#define PLIC_SRC_SPI  3u
#define PLIC_SRC_GPIO 4u
#define PLIC_SRC_FFT  5u
#define PLIC_SRC_N    16u

/* ------------------------------------------------------------------ */
/* Source plumbing                                                     */
/* ------------------------------------------------------------------ */

/* Raw pending picture (the IRQ lines themselves). */
static unsigned int plic_pending(void)
{
    return PLIC_PENDING;
}

/* Enable / disable a source into meip (mask bit, PLIC_ENABLE). Note the
 * all-ones reset: a source is enabled until explicitly disabled. */
static void plic_enable(unsigned int src)
{
    PLIC_ENABLE |= (1u << src);
}

static void plic_disable(unsigned int src)
{
    PLIC_ENABLE &= ~(1u << src);
}

/* Claim: lowest-ID enabled pending source, 0 if none. Side-effect-free
 * (see header) -- completing the interrupt is the source's job. */
static unsigned int plic_claim(void)
{
    return PLIC_CLAIM & 0xFFu;
}

/* ------------------------------------------------------------------ */
/* Per-source handler dispatch (the attachInterrupt layer)             */
/* ------------------------------------------------------------------ */

typedef void (*plic_handler_fn)(unsigned int src);

static plic_handler_fn plic_tab[PLIC_SRC_N];

/* The machine-external dispatcher: claim once, call the registered handler
 * (if any), return. See the LIVELOCK CONTRACT above. */
static void plic_mei_isr(void)
{
    unsigned int src = plic_claim();
    if (src != PLIC_SRC_NONE && src < PLIC_SRC_N && plic_tab[src])
        plic_tab[src](src);
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
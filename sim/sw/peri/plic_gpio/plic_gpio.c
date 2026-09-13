#include <stdint.h>

/*
 * GPIO + PLIC end-to-end MEIP oracle.
 *
 * First test of the whole external-interrupt chain since the bare OR was
 * replaced by the PLIC:
 *
 *   GPIO pin0 rise edge -> INT_STATUS sticky -> gpio_irq_o -> PLIC
 *   pending[4] -> meip_o -> mip.MEIP -> trap_unit -> interrupt entry,
 *   then the CAUSE/CLAIM contract the PLIC exists for: the handler reads
 *   CLAIM, must get PLIC_SRC_GPIO (4), W1Cs the GPIO INT_STATUS (which
 *   drops the line, which drops pending -- "claim = complete" is realized
 *   by servicing the source), and mret.
 *
 * Pin stimulus is self-driven: the firmware sets pin0 DIR=out and toggles
 * OUT, which in the sim feeds back through the loopback model + 2-flop
 * sync, and on the board through the actual pad. No harness support.
 *
 * Reset consequence this test pins (see axi4_lite_gpio header): the pads
 * pull up, INT_TYPE resets to level-high, so INT_STATUS reads all-1s
 * after reset (masked by INT_EN=0). Firmware must W1C INT_STATUS *after*
 * switching the type to rise, or the stale bit pends forever. The W1C
 * happens with the pin driven low and the type set to rise, so it must
 * stick.
 *
 * Result marker: 0x600D (pass) / 0xBAD (fail) written to D-mem 0x3000.
 * Debug fields (claim ID, irq count, unexpected mcause) also in .data so
 * a board failure can be read out of D-mem.
 */

#define GPIO_BASE 0x10007000u
#define GPIO_VALUE      (*(volatile uint32_t *)(GPIO_BASE + 0x00))
#define GPIO_OUT        (*(volatile uint32_t *)(GPIO_BASE + 0x04))
#define GPIO_DIR        (*(volatile uint32_t *)(GPIO_BASE + 0x08))
#define GPIO_INT_TYPE   (*(volatile uint32_t *)(GPIO_BASE + 0x0C))
#define GPIO_INT_EN     (*(volatile uint32_t *)(GPIO_BASE + 0x10))
#define GPIO_INT_STATUS (*(volatile uint32_t *)(GPIO_BASE + 0x14))

/* INT_TYPE encoding, 2 bits per pin: 00 level-high / 01 rise / 10 fall /
 * 11 both. Pin i -> bits [2i+1:2i], so 4 pins pack into one byte: all-rise
 * = 0b01_01_01_01 = 0x55, NOT 0x01010101 (that is rise on pin0, level on
 * the rest -- a byte-per-pin mistake the first version of this test made,
 * and the pulled-up level pins then pended forever). All four pins go to
 * rise: the pins the test does not drive stay pulled up (pads are
 * PULL_MODE=UP), and a level-type pin that reads 1 pends on every cycle --
 * leaving them at the level reset would make the stale-pend W1C below
 * never stick. */
#define INT_TYPE_RISE_ALL 0x55u

#define PLIC_BASE 0x10008000u
#define PLIC_PENDING (*(volatile uint32_t *)(PLIC_BASE + 0x00))
#define PLIC_ENABLE  (*(volatile uint32_t *)(PLIC_BASE + 0x04))
#define PLIC_CLAIM   (*(volatile uint32_t *)(PLIC_BASE + 0x08))

/* Source IDs (rv32_pkg::PLIC_SRC_*). */
#define PLIC_SRC_GPIO 4u

#define MSTATUS_MIE (1u << 3)
#define MIE_MEIE    (1u << 11)
#define MCAUSE_MEI  0x8000000Bu

/* Bounded spins. The interrupt is taken within a few cycles of mstatus.MIE
 * going high with meip already asserted, so a timeout is a diagnosis, not
 * a tolerance. */
#define SPIN_LIMIT 200000u

/* start.S does not zero .bss, and .bss is NOBITS so it is not part of the
 * loaded image either (see uart_echo.c for the board bug this caused).
 * Force the counters into .data so the initialiser actually loads. */
#define IN_DATA __attribute__((section(".data")))

static volatile uint32_t IN_DATA irq_seen   = 0;  /* interrupts taken */
static volatile uint32_t IN_DATA claim_id   = 0;   /* last CLAIM read by the handler */
static volatile uint32_t IN_DATA bad_claim  = 0;  /* claim != PLIC_SRC_GPIO */
static volatile uint32_t IN_DATA bad_cause  = 0;  /* unexpected mcause, if any */
static volatile uint32_t IN_DATA bad_traps  = 0;  /* unexpected traps taken */

static volatile uint32_t *const result = (volatile uint32_t *)0x3000;

static inline uint32_t read_mcause(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, mcause" : "=r"(v));
    return v;
}

/* GCC's machine-interrupt attribute emits the register save/restore and
 * the mret. aligned(4) is not cosmetic: mtvec masks its low two bits, and
 * with RVC a handler can land on a 2-byte boundary (see uart_echo.c). */
__attribute__((interrupt("machine"), aligned(4))) void trap_handler(void)
{
    uint32_t cause = read_mcause();

    if (cause != MCAUSE_MEI) {
        /* Not the external interrupt. Mask MIE to stop a storm and park
         * after a few repeats -- mcause is in D-mem for the post-mortem. */
        bad_cause = cause;
        __asm__ volatile("csrci mstatus, %0" ::"i"(MSTATUS_MIE));
        bad_traps = bad_traps + 1;
        if (bad_traps >= 4) {
            for (;;) { /* park with the diagnosis in D-mem */ }
        }
        return;
    }

    /* CAUSE/CLAIM contract: the lowest-ID enabled pending source. With
     * only GPIO armed, anything but PLIC_SRC_GPIO is a wiring or decode
     * bug. CLAIM is a pure priority read -- completing the claim is the
     * source's own job, done by the W1C below. */
    claim_id = PLIC_CLAIM;
    if (claim_id != PLIC_SRC_GPIO) {
        bad_claim = claim_id;
    }

    /* Service the source: W1C the sticky INT_STATUS bit. That drops
     * gpio_irq -> PLIC pending[4] -> meip, so mret does not re-take.
     * The pin is still driven high, but the type is rise, so no new
     * edge pends -- the level that the sticky bit recorded is gone. */
    GPIO_INT_STATUS = 0x1u;

    irq_seen = irq_seen + 1;
}

int main(void)
{
    /* ---- Arm pin 0 -----------------------------------------------------
     * DIR=out drives the pad low (OUT resets 0), which through the
     * loopback/pad also forces the input low -- the pre-condition for a
     * clean rising edge, and the state that lets the stale W1C below
     * stick. */
    GPIO_DIR = 0x1u;
    GPIO_OUT = 0x0u;

    /* Type first, THEN clear the stale reset pends: pulled-up pads + the
     * level-high type reset left INT_STATUS all-1s. With every pin's type
     * now rise, nothing re-pends, so the W1C sticks. */
    GPIO_INT_TYPE = INT_TYPE_RISE_ALL;
    GPIO_INT_STATUS = 0xFu;   /* W1C stale bits */
    if (GPIO_INT_STATUS != 0x0u) {
        *result = 0xBAD;      /* stale pends did not clear */
        __asm__ volatile("1: j 1b");
        __builtin_unreachable();
    }

    /* Pending must be quiet before anything is armed. */
    if (PLIC_PENDING != 0x0u) {
        *result = 0xBAD;
        __asm__ volatile("1: j 1b");
        __builtin_unreachable();
    }

    GPIO_INT_EN = 0x1u;

    /* Enable the GPIO source in the PLIC explicitly. ENABLE resets
     * all-ones (the drop-in-for-the-OR contract), so read-modify-write
     * must see the reset value AND keep the bit -- it also proves the
     * PLIC window decodes. */
    {
        uint32_t en = PLIC_ENABLE;
        PLIC_ENABLE = en | (1u << PLIC_SRC_GPIO);
        if (!(PLIC_ENABLE & (1u << PLIC_SRC_GPIO))) {
            *result = 0xBAD;  /* PLIC ENABLE write did not land */
            __asm__ volatile("1: j 1b");
            __builtin_unreachable();
        }
    }

    /* ---- Take the interrupt ------------------------------------------- */
    __asm__ volatile("csrw mtvec, %0" ::"r"((uint32_t)&trap_handler));
    __asm__ volatile("csrs mie, %0" ::"r"(MIE_MEIE));
    __asm__ volatile("csrsi mstatus, %0" ::"i"(MSTATUS_MIE));

    /* Rising edge: drive the pad high. Posted store; the edge reaches the
     * interrupt logic ~3 cycles later through the pad/loopback + 2-flop
     * sync. */
    GPIO_OUT = 0x1u;

    {
        uint32_t spin = 0;
        while (!irq_seen && !bad_cause && ++spin < SPIN_LIMIT) { /* spin */ }
    }
    if (!irq_seen) {
        *result = 0xBAD;      /* interrupt never taken */
        __asm__ volatile("1: j 1b");
        __builtin_unreachable();
    }

    /* ---- Post-interrupt contract --------------------------------------
     * Serviced: INT_STATUS cleared and no new edge, so the line is low,
     * pending is low, and a claim now finds nothing. A non-zero claim
     * here means the W1C did not really drop the line (livelock edge). */
    if (GPIO_INT_STATUS != 0x0u || PLIC_PENDING != 0x0u
        || PLIC_CLAIM != 0x0u) {
        *result = 0xBAD;
        __asm__ volatile("1: j 1b");
        __builtin_unreachable();
    }

    /* Loopback sanity: an output pin reads back what it drives. Only pin 0
     * is driven -- the other three stay pulled up and read 1, so mask. */
    if ((GPIO_VALUE & 0x1u) != 0x1u) {
        *result = 0xBAD;
        __asm__ volatile("1: j 1b");
        __builtin_unreachable();
    }

    /* ---- Fall edge must NOT pend (type is rise-only) ------------------- */
    GPIO_OUT = 0x0u;
    {
        /* Let the fall edge propagate through the 2-flop sync before
         * checking. volatile, or -O2 deletes the empty loop and the read
         * races the pad. ~10 cycles per iteration; 100 is generous. */
        volatile uint32_t spin = 0;
        while (++spin < 100u) { /* let the edge settle */ }
    }
    if ((GPIO_VALUE & 0x1u) != 0x0u || GPIO_INT_STATUS != 0x0u
        || PLIC_PENDING != 0x0u || irq_seen != 1u) {
        *result = 0xBAD;      /* fall edge pended or a second IRQ fired */
        __asm__ volatile("1: j 1b");
        __builtin_unreachable();
    }

    /* ---- Verdict ------------------------------------------------------- */
    __asm__ volatile("csrci mstatus, %0" ::"i"(MSTATUS_MIE));
    GPIO_INT_EN = 0x0u;

    if (bad_cause || bad_claim || irq_seen != 1u) {
        *result = 0xBAD;
    } else {
        *result = 0x600D;
    }

    /* Park in a self-loop, not in wfi: the sim harness stops on 8 identical
     * retires, and a wfi with every interrupt masked would trip its
     * "halt with no wake" liveness check. */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}
#include <stdint.h>
#include "uart.h"
#include "plic.h"

/*
 * UART echo + external-interrupt (MEIP) oracle.
 *
 * Two phases, so a failure says *where* the RX path broke:
 *
 *   Phase 1 (polling)  : echo PHASE_CHARS bytes read with uart_getc().
 *   Phase 2 (interrupt): enable CTRL.RX_IE + mie.MEIE + mstatus.MIE, then
 *                        sleep in wfi. The handler drains the RX FIFO,
 *                        echoes each byte and counts it. This is the only
 *                        test in the tree that exercises MEIP end to end:
 *                        uart_irq_o -> meip_i -> mip.MEIP -> trap_unit ->
 *                        interrupt entry, plus the wfi wake on an external
 *                        interrupt.
 *
 * Result marker: 0x600D (pass) / 0xBAD (fail) written to D-mem 0x3000.
 * 0x3000 rather than 0x2000 because .rodata is linked at 0x2000 -- writing
 * there would clobber the strings this program prints.
 *
 * Echo also makes it a hardware bring-up aid: type 8 characters on the
 * board and the first PHASE_CHARS come back by polling, the rest by
 * interrupt. Silence in phase 1 means bytes never reach the CPU (pin,
 * baud, sampling); silence only in phase 2 means the interrupt path is
 * broken while the data path works.
 */

/* linux-soc (M8): 16550 UART -- the RX interrupt enable is IER.RDI. */
#define UART_CTRL UART_IER
#define CTRL_RX_IE UART_IER_RDI

#define MSTATUS_MIE (1u << 3)
#define MIE_MEIE    (1u << 11)
#define MIP_MEIP    (1u << 11)
#define MCAUSE_MEI  0x8000000Bu
#define MCAUSE_ECALL_M 11u
#define MCAUSE_MSI     0x80000003u

#define MIE_MSIE (1u << 3)
#define MIP_MSIP (1u << 3)

/* SiFive CLINT msip[0] (linux-soc): writing bit0 sets/clears mip.MSIP. */
#define MSIP_REG (*(volatile unsigned int *)0x10010000u)

#define PHASE_CHARS 4

/* Iterations before a wait on an interrupt is declared dead. ~10 cycles
 * per iteration, so 200000 is ~80 ms at 25 MHz -- five orders of magnitude
 * more than the handful of cycles an interrupt entry actually takes. */
#define MSI_SPIN_LIMIT 200000u

/* start.S does not zero .bss, and .bss is NOBITS so it is not part of the
 * loaded image either: in hardware these words come up with whatever the
 * BSRAM powered up holding. A `= 0` initialiser is NOT enough -- gcc puts
 * zero-initialised statics in .bss precisely because it assumes a runtime
 * that clears it. Forcing them into .data puts them in the image, so the
 * zero is actually loaded.
 *
 * This bit us: sync_traps came up non-zero on the board, so the ecall
 * probe below printed "TRAP OK" without any trap having been taken, and
 * the simulation could not show it because the sim's D-mem is
 * zero-initialised. */
#define IN_DATA __attribute__((section(".data")))

static volatile uint32_t IN_DATA irq_chars = 0;   /* bytes echoed by the handler */
static volatile uint32_t IN_DATA sync_traps = 0;  /* ecall traps taken */
static volatile uint32_t IN_DATA msi_count = 0;    /* software interrupts taken */
static volatile uint32_t IN_DATA bad_cause = 0;   /* unexpected mcause, if any */
static volatile uint32_t IN_DATA bad_traps = 0;   /* unexpected traps taken */

static volatile uint32_t *const result = (volatile uint32_t *)0x3000;

static inline uint32_t read_mcause(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, mcause" : "=r"(v));
    return v;
}

static inline uint32_t read_mip(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, mip" : "=r"(v));
    return v;
}

static inline uint32_t read_mie(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, mie" : "=r"(v));
    return v;
}

static inline uint32_t read_mstatus(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, mstatus" : "=r"(v));
    return v;
}

static inline uint32_t read_mtvec(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, mtvec" : "=r"(v));
    return v;
}

static inline uint32_t read_mepc(void)
{
    uint32_t v;
    __asm__ volatile("csrr %0, mepc" : "=r"(v));
    return v;
}

/* GCC's machine-interrupt attribute emits the register save/restore and
 * the mret for us; mtvec is programmed to this address in direct mode.
 *
 * aligned(4) is not cosmetic: mtvec holds BASE in bits [31:2] and MODE in
 * [1:0], so a write masks the low two bits away. With RVC enabled a
 * function can land on a 2-byte boundary, and mtvec would then point two
 * bytes BELOW the handler -- into the middle of the preceding
 * instruction. The core would vector into garbage on the first interrupt,
 * with no output and no way to tell it from a dead interrupt path. */
__attribute__((interrupt("machine"), aligned(4))) void trap_handler(void)
{
    uint32_t cause = read_mcause();

    /* Entry marker, before anything can go wrong further down. A burst of
     * 'H' means the vector is re-entered without progressing (livelock);
     * a single 'H' means entry works and the handler body is the problem;
     * no 'H' at all means the interrupt is never taken, or it vectors
     * somewhere that is not this function. */
    uart_putc('H');

    /* Entry state, printed before any branch on it. mepc is the value mret
     * will jump to, so a bogus one here explains a core that goes quiet
     * without stalling: it returns into whatever that address holds, and a
     * tight non-memory loop there (start.S's `j 1b`, say) burns cycles
     * silently with dbg_stall_o low. Format: H<mcause>@<mepc>. */
    uart_put_hex32(cause);
    uart_putc('@');
    uart_put_hex32(read_mepc());
    uart_putc(' ');

    if (cause == MCAUSE_MSI) {
        /* MSIP is a level, held by the peripheral until software clears
         * it -- not clearing it here would re-enter forever. */
        MSIP_REG  = 0;
        msi_count = msi_count + 1;
        uart_putc('X');
        return;
    }

    if (cause == MCAUSE_ECALL_M) {
        /* Sync-trap probe (see main). mret returns to mepc, which points
         * AT the ecall, so step over it or we would trap forever. ecall is
         * always the 4-byte encoding. */
        uint32_t epc;
        __asm__ volatile("csrr %0, mepc" : "=r"(epc));
        __asm__ volatile("csrw mepc, %0" ::"r"(epc + 4));
        sync_traps = sync_traps + 1;
        uart_putc('X');
        return;
    }

    if (cause != MCAUSE_MEI) {
        /* Not the external interrupt. Masking mstatus.MIE stops an
         * interrupt storm but not a synchronous one: an illegal
         * instruction re-faults the moment mret returns into it, whatever
         * MIE says. That is the case worth catching here -- with the ROM
         * built only as deep as its image, a stray fetch reads zero, which
         * is a defined illegal encoding, so a wrong redirect shows up as
         * exactly this trap with mepc naming the address it wandered to.
         *
         * The entry line above has already printed the cause and mepc, so
         * park after a few repeats rather than flooding the line: the
         * first lines carry the whole diagnosis. */
        bad_cause = cause;
        __asm__ volatile("csrci mstatus, %0" ::"i"(MSTATUS_MIE));
        bad_traps = bad_traps + 1;
        if (bad_traps >= 4) {
            uart_puts("PARK\r\n");
            for (;;) { /* park with the diagnosis printed */ }
        }
        uart_putc('X');
        return;
    }

    /* linux-soc PLIC (sifive,plic-1.0.0): claim, service, complete. The
     * gateway keeps the source out of service until the complete, so a
     * handler that skipped it would never see the UART again. Drain the RX
     * FIFO before completing: the UART IRQ is a level, and a byte still
     * queued at the complete re-pends the source at once. */
    unsigned int src = plic_claim();
    while (UART_STATUS & UART_RX_READY_BIT) {
        char c = (char)(UART_RXDATA & 0xFF);
        uart_putc(c);
        irq_chars = irq_chars + 1;
    }
    if (src) plic_complete(src);
    uart_putc('X');
}


/* Write a value to a CSR and read it straight back. Any bit that does not
 * survive is a bit the CSR file does not really have on this device --
 * which is a different fault from a value being computed wrong, and the
 * only way to tell them apart is to control what goes in. mscratch,
 * mcause, mepc and mtval are safe to scribble on here: no trap has been
 * armed yet, and mtvec / mie / mstatus (the ones the test needs) are left
 * alone. */
#define CSR_WALK(name, val)                                             \
    do {                                                                \
        uint32_t rb;                                                    \
        __asm__ volatile("csrw " name ", %1\n\tcsrr %0, " name          \
                         : "=r"(rb)                                     \
                         : "r"((uint32_t)(val)));                       \
        uart_puts(" " name "=");                                        \
        uart_put_hex32(rb);                                             \
    } while (0)

static void csr_bit_walk(uint32_t pattern)
{
    uart_puts("CSRW ");
    uart_put_hex32(pattern);
    CSR_WALK("mscratch", pattern);
    CSR_WALK("mcause", pattern);
    CSR_WALK("mepc", pattern);
    CSR_WALK("mtval", pattern);
    uart_puts("\r\n");
}

int main(void)
{
    uart_puts("\r\nECHO\r\n");

    /* Print-path sanity first: two literals with every nibble distinct.
     * The hex printer indexes a .rodata table with a byte load, so a
     * broken sub-word load would corrupt every value printed anywhere --
     * including the CSR values below -- and would look exactly like a CSR
     * losing bits. If these two lines are right, the printer is right and
     * anything wrong afterwards is real. */
    uart_puts("PAT ");
    uart_put_hex32(0x0123ABCDu);
    uart_putc(' ');
    uart_put_hex32(0xFEDC5432u);
    uart_puts("\r\n");

    /* Then walk the CSR bits with controlled patterns. */
    csr_bit_walk(0xFFFFFFFFu);
    csr_bit_walk(0xAAAAAAAAu);
    csr_bit_walk(0x55555555u);

    /* ---- Phase 1: polling ---- */
    for (int i = 0; i < PHASE_CHARS; ++i) uart_putc(uart_getc());
    uart_puts("\r\nIRQ\r\n");

    /* ---- Phase 2: interrupt-driven ----
     *
     * Split into three observable steps, so a board that stops here says
     * WHICH link of the chain is broken instead of just going quiet:
     *
     *   2a: enable CTRL.RX_IE and mie.MEIE but leave mstatus.MIE clear,
     *       then poll mip. This walks uart_irq_o -> meip_i -> mip.MEIP
     *       with traps still globally disabled, so nothing about trap
     *       entry can be blamed. Prints the mip value it saw.
     *   2b: enable mstatus.MIE with that byte still sitting unread in the
     *       RX FIFO. The IRQ is level-sensitive, so the interrupt must be
     *       taken immediately -- no wfi involved. This isolates trap entry
     *       and the handler from the wfi wake.
     *   2c: sleep in wfi for the remaining bytes, which is the wfi-wake
     *       path.
     */
    UART_CTRL = CTRL_RX_IE;
    plic_enable(PLIC_SRC_UART);  /* the sifive PLIC resets everything to 0 */
    __asm__ volatile("csrw mtvec, %0" ::"r"((uint32_t)&trap_handler));
    __asm__ volatile("csrs mie, %0" ::"r"(MIE_MEIE));

    /* Read mtvec back against the handler address. A wrong mtvec breaks
     * every trap and every interrupt at once, and it breaks them
     * invisibly: the core vectors into whatever the I-mem holds there,
     * which faults, which vectors to the same wrong place -- a silent trap
     * storm. The two values must be equal with MODE=00 in the low bits. */
    uart_puts("MTVEC ");
    uart_put_hex32(read_mtvec());
    uart_puts(" HND ");
    uart_put_hex32((uint32_t)&trap_handler);
    uart_puts("\r\n");

    /* 2a -- send one byte now; it stays queued (nothing reads RXDATA). */
    while (!(read_mip() & MIP_MEIP)) { /* spin */ }
    uart_puts("MIP ");
    uart_put_hex32(read_mip());
    uart_puts(" MIE ");
    uart_put_hex32(read_mie());
    uart_puts(" MST ");
    uart_put_hex32(read_mstatus());
    uart_puts("\r\n");

    /* 2a' -- does trap entry work AT ALL? A synchronous ecall does not
     * depend on mstatus.MIE, mie or the interrupt arbitration, only on
     * mtvec and the redirect. If this prints FAIL the ecall retired
     * without trapping; if nothing prints at all, the vector landed
     * somewhere that never comes back -- either way the interrupt path is
     * not the thing to look at. */
    __asm__ volatile("ecall");
    /* mcause/mepc are the trap's own footprint: mcause == 11 (ecall from
     * M-mode) and mepc pointing at the ecall prove the trap happened,
     * independently of the sync_traps counter in D-mem. */
    uart_puts(sync_traps ? "TRAP OK CAUSE " : "TRAP FAIL CAUSE ");
    uart_put_hex32(read_mcause());
    uart_puts(" EPC ");
    uart_put_hex32(read_mepc());
    uart_puts("\r\n");

    /* 2a'' -- software interrupt (MSIP) before the external one. It is the
     * simplest interrupt in the machine: no peripheral IRQ line, no
     * meip_i, just a bit software sets in a register. If this hangs, the
     * fault is in interrupt entry/arbitration and has nothing to do with
     * the UART. Interrupts are enabled here for the first time. */
    /* Mask MEIE for the duration: MEIP is already pending and MEI outranks
     * MSI, so leaving it enabled would serve the external interrupt first
     * and tangle the two steps into one illegible output. */
    __asm__ volatile("csrc mie, %0" ::"r"(MIE_MEIE));
    __asm__ volatile("csrs mie, %0" ::"r"(MIE_MSIE));
    MSIP_REG = 1;

    /* Read the register back and dump mip/mie BEFORE enabling
     * mstatus.MIE. Three data in one line, each one cutting the search
     * space in a different place:
     *   REG bit0 set   -> the posted store reached msip_peri, so the LSU
     *                     peri steering, the bridge and the xbar decode
     *                     are all good.
     *   MIP bit3 set   -> msip_o reached csr_regfile (top-level wiring).
     *   MIE bit3 set   -> the local enable write took.
     * If all three read right and no interrupt follows, everything except
     * the taking machinery (int_pending -> take_interrupt -> redirect) has
     * been cleared. */
    uart_puts("MSIPREG ");
    uart_put_hex32(MSIP_REG);
    uart_puts(" MIP ");
    uart_put_hex32(read_mip());
    uart_puts(" MIE ");
    uart_put_hex32(read_mie());
    uart_puts("\r\n");

    __asm__ volatile("csrsi mstatus, %0" ::"i"(MSTATUS_MIE));

    /* Bounded wait. An interrupt that is going to be taken is taken within
     * a few cycles of mstatus.MIE going high, so a timeout here is a
     * diagnosis rather than a tolerance: it says every pending and enable
     * bit was set and the core still did not vector. It also keeps the
     * board from going silent, which is the least informative failure a
     * bring-up program can produce. */
    {
        uint32_t spin = 0;
        while (!msi_count && !bad_cause && ++spin < MSI_SPIN_LIMIT) { /* spin */ }
    }
    __asm__ volatile("csrc mie, %0" ::"r"(MIE_MSIE));
    if (msi_count) {
        uart_puts("MSI OK\r\n");
    } else {
        uart_puts("MSI TIMEOUT MST ");
        uart_put_hex32(read_mstatus());
        uart_puts(" MIP ");
        uart_put_hex32(read_mip());
        uart_puts(" MIE ");
        uart_put_hex32(read_mie());
        uart_puts(" BADC ");
        uart_put_hex32(bad_cause);
        uart_puts(" CAUSE ");
        uart_put_hex32(read_mcause());
        uart_puts(" EPC ");
        uart_put_hex32(read_mepc());
        uart_puts("\r\n");
    }
    __asm__ volatile("csrs mie, %0" ::"r"(MIE_MEIE));

    /* 2b + 2c -- external interrupt. mstatus.MIE is already set, so with
     * the byte still queued this fires as soon as we get here. */
    __asm__ volatile("csrsi mstatus, %0" ::"i"(MSTATUS_MIE));

    /* With a byte already queued and the IRQ level-high, the interrupt is
     * taken before this line runs, so the echoed byte appears BEFORE this
     * print. Reading mstatus back here is the last discriminator: if it
     * shows MIE set and no byte was echoed, then mip, mie, mstatus and
     * trap entry are all provably fine and the interrupt simply is not
     * being taken -- which points at the arbitration/timing, not the
     * peripheral. */
    uart_puts("MST2 ");
    uart_put_hex32(read_mstatus());
    uart_puts("\r\n");

    while (irq_chars < PHASE_CHARS && !bad_cause) __asm__ volatile("wfi");

    /* Mask before reporting, so the marker store cannot race a handler. */
    __asm__ volatile("csrci mstatus, %0" ::"i"(MSTATUS_MIE));
    UART_CTRL = 0;

    if (bad_cause || irq_chars < PHASE_CHARS) {
        *result = 0xBAD;
        uart_puts("BAD\r\n");
    } else {
        *result = 0x600D;
        uart_puts("GOOD\r\n");
    }

    /* Park in a self-loop, not in wfi: the sim harness stops on 8 identical
     * retires, and a wfi with every interrupt masked would instead trip its
     * "halt with no wake" liveness check (legal per spec, useless here). */
    __asm__ volatile("1: j 1b");
    __builtin_unreachable();
}

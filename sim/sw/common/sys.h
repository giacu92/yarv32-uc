#ifndef SYS_H
#define SYS_H

/*
 * Machine-mode core plumbing: CSR access, global/source interrupt enables,
 * mtvec installation, WFI, and a small cause-indexed ISR dispatch table --
 * the Zicsr equivalent of what other headers here do for peripherals.
 *
 * Everything a trap/interrupt firmware program needs, so individual tests
 * stop hand-rolling the same asm (uart_echo / plic_gpio each carry ~80
 * lines of it). Include from ONE translation unit per program: the dispatch
 * table and the entry function are `static`, and a second TU would get its
 * own copy -- the mtvec one program installs must be the one that reads
 * the table the handlers were registered in.
 *
 * CSR read note (csr_regfile.sv): reads are REGISTERED, so a value read
 * back reflects the previous cycle. Harmless for polling loops; do not
 * build cycle-exact sequences on a single read-then-branch.
 *
 * mcycle note: only the low 32 bits exist (mcycleh / minstreth are not
 * implemented, RV32 mandates them -- see CLAUDE.md Open work). mcycle
 * wraps every 2^32 cycles, ~85.9 s at 50 MHz. The delay helpers use
 * unsigned subtraction, so they are wrap-safe for any single delay below
 * that; long-horizon timing must use the CLINT timer (timer.h), whose
 * mtime is a real 64-bit counter.
 */

/* ------------------------------------------------------------------ */
/* CSR access macros (compile-time CSR name -> 12-bit immediate)       */
/* ------------------------------------------------------------------ */

#define SYS_CSR_READ(csr)                                                  \
    ({                                                                     \
        unsigned int v_;                                                   \
        __asm__ volatile("csrr %0, " #csr : "=r"(v_));                     \
        v_;                                                                \
    })

#define SYS_CSR_WRITE(csr, val)                                            \
    __asm__ volatile("csrw " #csr ", %0" ::"r"((unsigned int)(val)))

#define SYS_CSR_SET(csr, val)                                              \
    __asm__ volatile("csrs " #csr ", %0" ::"r"((unsigned int)(val)))

#define SYS_CSR_CLEAR(csr, val)                                            \
    __asm__ volatile("csrc " #csr ", %0" ::"r"((unsigned int)(val)))

/* Named accessors for the CSRs firmware actually touches. */
static unsigned int sys_mstatus(void) { return SYS_CSR_READ(mstatus); }
static unsigned int sys_mie(void)     { return SYS_CSR_READ(mie); }
static unsigned int sys_mip(void)     { return SYS_CSR_READ(mip); }
static unsigned int sys_mcause(void)  { return SYS_CSR_READ(mcause); }
static unsigned int sys_mepc(void)    { return SYS_CSR_READ(mepc); }
static unsigned int sys_mtval(void)   { return SYS_CSR_READ(mtval); }
static unsigned int sys_mtvec(void)   { return SYS_CSR_READ(mtvec); }

/* ------------------------------------------------------------------ */
/* Interrupt / trap constants                                         */
/* ------------------------------------------------------------------ */

/* mstatus */
#define SYS_MSTATUS_MIE (1u << 3)
#define SYS_MSTATUS_MPIE (1u << 7)
#define SYS_MSTATUS_MPP (3u << 11)

/* mie / mip source bits */
#define SYS_MIE_MSIE (1u << 3)   /* machine software interrupt */
#define SYS_MIE_MTIE (1u << 7)   /* machine timer interrupt */
#define SYS_MIE_MEIE (1u << 11)  /* machine external interrupt */

/* mcause encoding: bit31 = interrupt, [30:0] = code. */
#define SYS_MCAUSE_IRQ (1u << 31)
#define SYS_MCAUSE_INSTR_ACC  1u
#define SYS_MCAUSE_ILLEGAL    2u
#define SYS_MCAUSE_EBREAK     3u
#define SYS_MCAUSE_LD_MISAL   4u
#define SYS_MCAUSE_ST_MISAL   6u
#define SYS_MCAUSE_ECALL_M    11u
#define SYS_MCAUSE_MSI        3u
#define SYS_MCAUSE_MTI        7u
#define SYS_MCAUSE_MEI        11u

static unsigned int sys_cause_is_interrupt(unsigned int mcause)
{
    return (mcause & SYS_MCAUSE_IRQ) != 0u;
}

static unsigned int sys_cause_code(unsigned int mcause)
{
    return mcause & ~SYS_MCAUSE_IRQ;
}

/* ------------------------------------------------------------------ */
/* Global / source enables, WFI, cycle count                          */
/* ------------------------------------------------------------------ */

/* Global interrupt enable (mstatus.MIE). Source bits in mie must be set
 * too -- an interrupt fires only when mstatus.MIE & mie[x] & mip[x]. */
static void sys_irq_global_enable(void)
{
    SYS_CSR_SET(mstatus, SYS_MSTATUS_MIE);
}

static void sys_irq_global_disable(void)
{
    SYS_CSR_CLEAR(mstatus, SYS_MSTATUS_MIE);
}

/* Source enables (mie). Takes a mask of SYS_MIE_* bits. */
static void sys_irq_source_enable(unsigned int mask)
{
    SYS_CSR_SET(mie, mask);
}

static void sys_irq_source_disable(unsigned int mask)
{
    SYS_CSR_CLEAR(mie, mask);
}

/* Sleep until an enabled interrupt is pending (the architectural WFI --
 * on this core a halt-until-int_pending, see execute_stage.sv). Legal
 * forever if no enabled interrupt ever arrives; firmware that cannot
 * afford that polls with a timeout instead. */
static void sys_wfi(void)
{
    __asm__ volatile("wfi");
}

/* Free-running cycle counter, low 32 bits (see the mcycle note above). */
static unsigned int sys_cycle(void)
{
    return SYS_CSR_READ(mcycle);
}

static unsigned int sys_instret(void)
{
    return SYS_CSR_READ(minstret);
}

/* ------------------------------------------------------------------ */
/* ISR dispatch (direct-mode mtvec + a cause-indexed table)           */
/* ------------------------------------------------------------------ */

#define SYS_ISR_MAX 16u  /* interrupt cause codes 1..15 (0 = no handler) */

typedef void (*sys_isr_fn)(void);
typedef void (*sys_isr_unhandled_fn)(unsigned int mcause);

static sys_isr_fn sys_isr_tab[SYS_ISR_MAX];
static sys_isr_unhandled_fn sys_isr_unhandled;

/* The dispatch entry, defined before its address is taken (install below).
 * GCC's interrupt("machine") attribute emits the register save/restore and
 * the mret; aligned(4) is not cosmetic -- mtvec holds BASE in [31:2] and
 * MODE in [1:0], so a handler on a 2-byte RVC boundary would vector two
 * bytes below itself (see uart_echo.c). */
__attribute__((interrupt("machine"), aligned(4)))
static void sys_isr_entry(void)
{
    unsigned int mcause = SYS_CSR_READ(mcause);

    if (sys_cause_is_interrupt(mcause)) {
        unsigned int code = sys_cause_code(mcause);
        if (code < SYS_ISR_MAX && sys_isr_tab[code]) {
            sys_isr_tab[code]();
            return;
        }
    }

    if (sys_isr_unhandled) {
        sys_isr_unhandled(mcause);
        return;
    }

    SYS_CSR_CLEAR(mstatus, SYS_MSTATUS_MIE);
}

/* Install the dispatch entry at mtvec (direct mode: every trap vectors
 * here and the entry decodes mcause). Call once, before enabling any
 * interrupt source. */
static void sys_isr_install(void)
{
    SYS_CSR_WRITE(mtvec, (unsigned int)&sys_isr_entry);
}

/* Register a handler for an interrupt cause code (SYS_MCAUSE_MSI/MTI/MEI).
 * Overwrites any previous handler. */
static void sys_interrupt_attach(unsigned int cause_code, sys_isr_fn fn)
{
    if (cause_code < SYS_ISR_MAX)
        sys_isr_tab[cause_code] = fn;
}

static void sys_interrupt_detach(unsigned int cause_code)
{
    if (cause_code < SYS_ISR_MAX)
        sys_isr_tab[cause_code] = 0;
}

/* Optional hook for any trap with no registered handler. Without one the
 * entry masks mstatus.MIE and returns: enough for an unexpected INTERRUPT
 * (it will not re-fire until re-enabled), but a synchronous trap whose
 * handler is absent re-faults on return by construction -- attach handlers
 * for every cause you can take, or set this hook. */
static void sys_isr_set_unhandled(sys_isr_unhandled_fn fn)
{
    sys_isr_unhandled = fn;
}

#endif /* SYS_H */
#ifndef UART_INIT_H
#define UART_INIT_H

/* linux-soc (M8): the UART is 16550-compatible (axi4_lite_uart16550,
 * "ns16550a", reg-shift 2, reg-io-width 4): register i at UART_BASE + 4*i.
 * The MCU's TXDATA/RXDATA/STATUS/CTRL/BAUDDIV map is gone on this branch. */
#define UART_BASE   0x10000000u
#define UART_REG(i) (*(volatile unsigned int *)(UART_BASE + 4u * (i)))
#define UART_RBR    UART_REG(0)  /* read, DLAB=0: pops the RX FIFO   */
#define UART_THR    UART_REG(0)  /* write, DLAB=0: pushes the TX FIFO */
#define UART_DLL    UART_REG(0)  /* DLAB=1 */
#define UART_IER    UART_REG(1)  /* DLAB=0 */
#define UART_DLM    UART_REG(1)  /* DLAB=1 */
#define UART_IIR    UART_REG(2)  /* read  */
#define UART_FCR    UART_REG(2)  /* write */
#define UART_LCR    UART_REG(3)
#define UART_MCR    UART_REG(4)
#define UART_LSR    UART_REG(5)
#define UART_MSR    UART_REG(6)
#define UART_SCR    UART_REG(7)

#define UART_IER_RDI   0x01u  /* RX data available / char timeout */
#define UART_IER_THRI  0x02u  /* THR empty */
#define UART_IER_RLSI  0x04u  /* line status */
#define UART_IIR_NO_INT 0x01u
#define UART_IIR_ID     0x0Eu
#define UART_LSR_DR    0x01u
#define UART_LSR_OE    0x02u
#define UART_LSR_FE    0x08u
#define UART_LSR_BI    0x10u
#define UART_LSR_THRE  0x20u
#define UART_LSR_TEMT  0x40u
#define UART_LCR_DLAB  0x80u

/* Kept names: older code polls "RX ready" / "TX ready". */
#define UART_STATUS        UART_LSR
#define UART_RXDATA        UART_RBR
#define UART_TXDATA        UART_THR
#define UART_RX_READY_BIT  UART_LSR_DR
#define UART_TX_READY_BIT  UART_LSR_THRE

static char uart_getc(void)
{
    while (!(UART_LSR & UART_LSR_DR))
        ;
    return (char)(UART_RBR & 0xFF);
}

/* UART_TX_PACED: send with a software delay instead of the TX_READY poll,
 * long enough that only one byte is ever in flight, so the TX FIFO never
 * becomes full and the poll loop is never entered.
 *
 * This exists as a board diagnostic. On silicon both the monitor and the
 * bring-up program stop producing output exactly when the TX FIFO first
 * fills -- 16 bytes into an uninterrupted burst -- while simulation, which
 * fills the same FIFO at the same baud, runs to completion. Paced output
 * removes both suspects at once: no full FIFO, and no polling. If the
 * output then completes, the fault is in the full/pop path of the
 * peripheral; if it still stops, the FIFO is exonerated and the bus or the
 * engine is the place to look.
 *
 * UART_TX_DELAY is in loop iterations, a few cycles each. The default is
 * comfortably longer than one 115200 frame at 25 MHz (2170 cycles). */
#ifndef UART_TX_PACED
#define UART_TX_PACED 0
#endif
#ifndef UART_TX_DELAY
#define UART_TX_DELAY 3000
#endif

static void uart_putc(char c)
{
#if UART_TX_PACED
    for (volatile int d = 0; d < UART_TX_DELAY; ++d)
        ;
    UART_TXDATA = (unsigned int)(unsigned char)c;
#else
    while (!(UART_STATUS & UART_TX_READY_BIT))
        ;
    UART_TXDATA = (unsigned int)(unsigned char)c;
#endif
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

static void uart_put_hex32(unsigned int v)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (int i = 28; i >= 0; i -= 4)
        uart_putc(hex[(v >> i) & 0xF]);
}

#endif /* UART_INIT_H */
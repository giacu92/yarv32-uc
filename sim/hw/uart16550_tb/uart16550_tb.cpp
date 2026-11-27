// axi4_lite_uart16550 register/behaviour test, linux-soc M8.
//
// What the 8250 driver relies on for "ns16550a" with reg-shift 2 /
// reg-io-width 4 (drivers/tty/serial/8250/8250_of.c, 8250_port.c, as known
// to the author -- M10 re-checks against the pinned 6.18 tree): register i
// at byte offset 4*i; LSR.DR/THRE/TEMT/OE/FE/BI; IIR priority and the
// THRE-cleared-by-IIR-read rule; 16 x DL clocks per bit; FCR FIFO reset and
// RX trigger levels; MCR.LOOP.
//
// The DUT is built with CLK_FREQ_HZ = 3.2 MHz, BAUD_RATE = 100 kHz, so DL
// resets to 2 and one bit is 32 clocks.
#include "Vuart16550_tb.h"
#include "axi_bfm.h"
#include "verilated.h"
#include <deque>
#include <vector>

enum { RBR = 0x00, IER = 0x04, IIR = 0x08, FCR = 0x08, LCR = 0x0C,
       MCR = 0x10, LSR = 0x14, MSR = 0x18, SCR = 0x1C };

static int g_bit = 32;  // clocks per bit, tracks DL

// ---- line models (run every cycle from the BFM hook) ---------------------
static std::deque<int> g_rx_bits;  // levels to drive on rxd, one per clock
static std::vector<int> g_txw;     // every txd sample, one per clock
static int g_tx_last_start_len = 0;

static void line_hook(Vuart16550_tb* t) {
    t->rxd_i = g_rx_bits.empty() ? 1 : g_rx_bits.front();
    if (!g_rx_bits.empty()) g_rx_bits.pop_front();
    g_txw.push_back(t->txd_o);
}

// Decode every complete 8N1 frame in the recorded txd waveform: find a
// falling edge, sample at the middle of each data bit and of the stop bit.
static std::vector<uint8_t> tx_frames() {
    std::vector<uint8_t> out;
    size_t i = 1;
    while (i + 10 * g_bit < g_txw.size()) {
        if (g_txw[i - 1] == 1 && g_txw[i] == 0) {
            size_t n = 0;
            while (i + n < g_txw.size() && g_txw[i + n] == 0 && n < 4096) ++n;
            if (n <= (size_t)g_bit) g_tx_last_start_len = (int)n;
            uint8_t v = 0;
            for (int k = 0; k < 8; ++k) v |= g_txw[i + g_bit * k + g_bit + g_bit / 2] << k;
            int stop = g_txw[i + 9 * g_bit + g_bit / 2];
            if (stop) out.push_back(v);
            i += 10 * g_bit;
        } else {
            ++i;
        }
    }
    return out;
}

static void send_rx(uint8_t v, int stop = 1) {
    auto push = [](int lvl) { for (int i = 0; i < g_bit; ++i) g_rx_bits.push_back(lvl); };
    push(0);
    for (int i = 0; i < 8; ++i) push((v >> i) & 1);
    push(stop);
    for (int i = 0; i < 4; ++i) g_rx_bits.push_back(1);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto* top = new Vuart16550_tb;
    Bfm<Vuart16550_tb> b(top);
    b.on_tick = line_hook;
    top->rxd_i = 1;
    b.reset();
    auto drain = [&]() { while (!g_rx_bits.empty()) b.tick(); b.ticks(4 * g_bit); };

    // ---- 1. reset values --------------------------------------------------
    CHECK_EQ(b.read(IER), 0, "IER resets 0");
    CHECK_EQ(b.read(IIR), 0x01, "IIR: no interrupt, FIFOs off");
    CHECK_EQ(b.read(LCR), 0x03, "LCR resets to 8N1");
    CHECK_EQ(b.read(MCR), 0x00, "MCR resets 0");
    CHECK_EQ(b.read(LSR), 0x60, "LSR: THRE | TEMT");
    CHECK_EQ(b.read(MSR), 0xB0, "MSR: CTS DSR DCD, no RI, no deltas");
    CHECK_EQ(top->irq_o, 0, "no irq");
    b.write(SCR, 0xA5);
    CHECK_EQ(b.read(SCR), 0xA5, "SCR scratch");

    // ---- 2. divisor latch ---------------------------------------------------
    b.write(LCR, 0x83);
    CHECK_EQ(b.read(RBR), 2, "DLL resets to CLK/(16*BAUD) = 2");
    CHECK_EQ(b.read(IER), 0, "DLM 0");
    b.write(RBR, 1);  // DLL = 1 -> 16 clocks per bit
    b.write(LCR, 0x03);
    g_bit = 16;
    b.ticks(4);
    CHECK_EQ(b.read(LCR), 0x03, "DLAB cleared");

    // ---- 3. TX: frame shape and bit time ----------------------------------
    b.write(FCR, 0x07);  // FIFOs on, both reset
    b.write(RBR, 0x5B);  // THR
    CHECK_EQ(b.read(LSR) & 0x40, 0, "TEMT clears while sending");
    b.ticks(12 * 16);
    CHECK_EQ(tx_frames().size(), 1, "one frame on txd");
    if (!tx_frames().empty()) CHECK_EQ(tx_frames()[0], 0x5B, "TX byte value");
    CHECK(g_tx_last_start_len >= 15 && g_tx_last_start_len <= 17, "bit = 16 x DL clocks");
    CHECK_EQ(b.read(LSR) & 0x60, 0x60, "THRE|TEMT after the frame");

    // ---- 4. THRE interrupt: raised, cleared by an IIR read, re-armed by THR
    b.write(IER, 0x02);
    b.ticks(2);
    CHECK_EQ(top->irq_o, 1, "ETBEI with empty FIFO raises THRE");
    CHECK_EQ(b.read(IIR), 0xC2, "IIR = THRE, FIFOs on");
    b.ticks(2);
    CHECK_EQ(top->irq_o, 0, "reading IIR cleared the THRE source");
    CHECK_EQ(b.read(IIR), 0xC1, "no interrupt now");
    b.write(RBR, 0x11);
    b.ticks(12 * 16);
    CHECK_EQ(top->irq_o, 1, "FIFO drained again -> THRE again");
    b.write(IER, 0x00);
    CHECK_EQ(top->irq_o, 0, "masked");

    // ---- 5. RX: data ready, interrupt, pop ----------------------------------
    b.write(IER, 0x01);
    send_rx(0xA7);
    drain();
    CHECK_EQ(b.read(LSR) & 0x01, 1, "DR");
    CHECK_EQ(top->irq_o, 1, "RDA irq (trigger 1)");
    CHECK_EQ(b.read(IIR), 0xC4, "IIR = RX data available");
    CHECK_EQ(b.read(RBR), 0xA7, "RBR value");
    CHECK_EQ(b.read(LSR) & 0x01, 0, "DR clears after the pop");
    CHECK_EQ(top->irq_o, 0, "irq drops");

    // ---- 6. trigger 4 + character timeout ----------------------------------
    b.write(FCR, 0x41);  // trigger 4, FIFOs on
    send_rx(0x01);
    send_rx(0x02);
    drain();
    CHECK_EQ(b.read(IIR) & 0x0F, 0x01, "2 < trigger 4: no RDA yet");
    b.ticks(45 * g_bit);
    CHECK_EQ(b.read(IIR), 0xCC, "character timeout after 4 char times");
    CHECK_EQ(b.read(RBR), 0x01, "pop 1");
    CHECK_EQ(b.read(RBR), 0x02, "pop 2");
    CHECK_EQ(b.read(IIR) & 0x0F, 0x01, "empty -> no timeout");
    for (int i = 0; i < 4; ++i) send_rx(0x30 + i);
    drain();
    CHECK_EQ(b.read(IIR), 0xC4, "4 bytes reach trigger 4");
    b.write(FCR, 0x03);  // RX FIFO reset, trigger 1
    CHECK_EQ(b.read(LSR) & 0x01, 0, "FCR RX reset empties the FIFO");

    // ---- 7. overrun ------------------------------------------------------------
    b.write(IER, 0x00);
    for (int i = 0; i < 17; ++i) send_rx(0x40 + i);
    drain();
    uint32_t lsr = b.read(LSR);
    CHECK_EQ(lsr & 0x02, 0x02, "OE after 17 bytes into a 16-byte FIFO");
    CHECK_EQ(b.read(LSR) & 0x02, 0, "OE cleared by the LSR read");
    CHECK_EQ(b.read(RBR), 0x40, "the first 16 bytes are kept");
    b.write(FCR, 0x03);

    // ---- 8. framing error and break; line-status interrupt priority -------
    b.write(IER, 0x05);  // RDI + RLSI
    send_rx(0x55, 0);    // stop bit low
    drain();
    CHECK_EQ(b.read(IIR), 0xC6, "line status outranks RX data");
    lsr = b.read(LSR);
    CHECK_EQ(lsr & 0x08, 0x08, "FE");
    CHECK_EQ(lsr & 0x80, 0x80, "LSR[7] error-in-FIFO summary");
    CHECK_EQ(b.read(IIR), 0xC4, "after the LSR read: RX data");
    b.read(RBR);
    send_rx(0x00, 0);  // break
    drain();
    lsr = b.read(LSR);
    CHECK_EQ(lsr & 0x10, 0x10, "BI on an all-zero frame");
    b.write(FCR, 0x03);
    b.write(IER, 0x00);
    b.read(LSR);

    // ---- 9. MCR loopback: TX comes back on RX, txd stays high --------------
    b.write(MCR, 0x1F);
    CHECK_EQ(b.read(MSR) & 0xF0, 0xF0, "loop: MSR mirrors RTS/DTR/OUT1/OUT2");
    size_t before = tx_frames().size();
    b.write(RBR, 0x3D);
    b.ticks(12 * 16);
    CHECK_EQ(tx_frames().size(), before, "txd held high in loop");
    CHECK_EQ(b.read(LSR) & 0x01, 1, "byte came back internally");
    CHECK_EQ(b.read(RBR), 0x3D, "loopback value");
    b.write(MCR, 0x00);
    CHECK_EQ(b.read(MSR), 0xB0, "out of loop: fixed modem status");

    // ---- 10. THR write with a full FIFO is held, not dropped -------------
    for (int i = 0; i < 18; ++i) b.write(RBR, 0x61 + 2 * i);  // odd bytes: LSB 1
    b.ticks(20 * 12 * 16);
    CHECK_EQ(tx_frames().size(), before + 18, "all 18 bytes sent, none dropped");

    int rc = bfm_report("uart16550_tb");
    delete top;
    return rc;
}

// axi4_lite_plic_sifive register/gateway test, linux-soc M8.
//
// Offsets and behaviour are what drivers/irqchip/irq-sifive-plic.c relies
// on: PRIORITY_BASE 0, PENDING_BASE 0x1000, CONTEXT_ENABLE_BASE 0x2000 +
// 0x80*ctx, CONTEXT_BASE 0x200000 + 0x1000*ctx with THRESHOLD at +0 and
// CLAIM/COMPLETE at +4; plic_irq_unmask() writes priority 1, plic_irq_mask()
// writes 0; plic_irq_eoi() writes the ID to COMPLETE (re-enabling a masked
// source around it, because a complete for a disabled source is ignored).
// (From the driver as known to the author, not re-read against a pinned
// tree here -- M10 pins 6.18 and re-checks.)
#include "Vplic_sifive_tb.h"
#include "axi_bfm.h"
#include "verilated.h"

static uint32_t PRIO(int id) { return 4u * id; }
static const uint32_t PENDING = 0x1000;
static uint32_t ENABLE(int c) { return 0x2000u + 0x80u * c; }
static uint32_t THRESH(int c) { return 0x200000u + 0x1000u * c; }
static uint32_t CLAIM(int c) { return 0x200004u + 0x1000u * c; }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto* top = new Vplic_sifive_tb;
    Bfm<Vplic_sifive_tb> b(top);
    top->irq_i = 0;
    b.reset();

    // ---- 1. reset ---------------------------------------------------------
    for (int id = 0; id < 16; ++id) CHECK_EQ(b.read(PRIO(id)), 0, "priority resets 0");
    CHECK_EQ(b.read(PENDING), 0, "pending resets 0");
    CHECK_EQ(b.read(ENABLE(0)), 0, "ctx0 enable resets 0");
    CHECK_EQ(b.read(ENABLE(1)), 0, "ctx1 enable resets 0");
    CHECK_EQ(b.read(THRESH(0)), 0, "ctx0 threshold resets 0");
    CHECK_EQ(b.read(CLAIM(0)), 0, "claim with nothing pending reads 0");
    CHECK_EQ(top->meip_o, 0, "meip low at reset");
    CHECK_EQ(top->seip_o, 0, "seip low at reset");

    // ---- 2. register widths -----------------------------------------------
    b.write(PRIO(3), 0xFF);
    CHECK_EQ(b.read(PRIO(3)), 7, "priority is 3 bits");
    b.write(PRIO(0), 5);
    CHECK_EQ(b.read(PRIO(0)), 0, "priority[0] (no source) stays 0");
    b.write(THRESH(0), 0xFF);
    CHECK_EQ(b.read(THRESH(0)), 7, "threshold is 3 bits");
    b.write(THRESH(0), 0);
    b.write(PRIO(3), 0);
    b.write(ENABLE(0), 0xFFFFFFFF);
    CHECK_EQ(b.read(ENABLE(0)), 0xFFFF, "enable bits exist for 16 sources only");
    b.write(ENABLE(0), 0);

    // ---- 3. gateway latches; priority 0 masks (Linux's mask) --------------
    top->irq_i = 1u << 4;
    b.ticks(2);
    CHECK_EQ(b.read(PENDING), 1u << 4, "line high -> pending");
    b.write(ENABLE(0), 1u << 4);
    CHECK_EQ(top->meip_o, 0, "enabled but priority 0 -> no meip (masked)");
    b.write(PRIO(4), 1);  // plic_irq_unmask()
    CHECK_EQ(top->meip_o, 1, "priority 1 > threshold 0 -> meip");
    CHECK_EQ(top->seip_o, 0, "ctx1 not enabled -> no seip");
    b.write(THRESH(0), 1);
    CHECK_EQ(top->meip_o, 0, "priority must exceed threshold");
    b.write(THRESH(0), 0);
    CHECK_EQ(top->meip_o, 1, "threshold 0 again");

    // ---- 4. claim takes the source out of service -------------------------
    CHECK_EQ(b.read(CLAIM(0)), 4, "claim returns the source");
    CHECK_EQ(b.read(PENDING), 0, "claim clears pending");
    b.ticks(4);
    CHECK_EQ(top->meip_o, 0, "line still high but in service -> no meip");
    CHECK_EQ(b.read(CLAIM(0)), 0, "nothing else to claim");

    // ---- 5. complete re-presents a still-high level source ----------------
    b.write(CLAIM(0), 4);  // plic_irq_eoi()
    b.ticks(2);
    CHECK_EQ(b.read(PENDING), 1u << 4, "re-pends after complete");
    CHECK_EQ(top->meip_o, 1, "meip again");
    CHECK_EQ(b.read(CLAIM(0)), 4, "claim again");
    top->irq_i = 0;  // device serviced
    b.write(CLAIM(0), 4);
    b.ticks(2);
    CHECK_EQ(b.read(PENDING), 0, "serviced + completed -> quiet");
    CHECK_EQ(top->meip_o, 0, "meip low");

    // ---- 6. complete for a disabled source is ignored (spec) --------------
    top->irq_i = 1u << 4;
    b.ticks(2);
    CHECK_EQ(b.read(CLAIM(0)), 4, "claim");
    top->irq_i = 0;
    b.write(ENABLE(0), 0);
    b.write(CLAIM(0), 4);  // ignored: not enabled in ctx0
    b.write(ENABLE(0), 1u << 4);
    top->irq_i = 1u << 4;
    b.ticks(4);
    CHECK_EQ(top->meip_o, 0, "still in service: the disabled complete was ignored");
    b.write(CLAIM(0), 4);  // now enabled -> honoured
    b.ticks(2);
    CHECK_EQ(top->meip_o, 1, "complete honoured once enabled");
    CHECK_EQ(b.read(CLAIM(0)), 4, "claim");
    top->irq_i = 0;
    b.write(CLAIM(0), 4);
    b.ticks(2);

    // ---- 7. a one-cycle pulse is latched until claimed ---------------------
    top->irq_i = 1u << 4;
    b.tick();
    top->irq_i = 0;
    b.ticks(4);
    CHECK_EQ(b.read(PENDING), 1u << 4, "pulse latched in the gateway");
    CHECK_EQ(top->meip_o, 1, "and presented");
    CHECK_EQ(b.read(CLAIM(0)), 4, "claim");
    b.write(CLAIM(0), 4);
    b.ticks(2);
    CHECK_EQ(top->meip_o, 0, "gone after claim/complete");

    // ---- 8. arbitration: highest priority, then lowest ID -----------------
    b.write(ENABLE(0), (1u << 3) | (1u << 5) | (1u << 7));
    b.write(PRIO(3), 2);
    b.write(PRIO(5), 5);
    b.write(PRIO(7), 5);
    top->irq_i = (1u << 3) | (1u << 5) | (1u << 7);
    b.ticks(2);
    CHECK_EQ(b.read(CLAIM(0)), 5, "highest priority; 5 beats 7 on the tie (lower ID)");
    CHECK_EQ(b.read(CLAIM(0)), 7, "then 7");
    CHECK_EQ(b.read(CLAIM(0)), 3, "then the lower priority 3");
    CHECK_EQ(b.read(CLAIM(0)), 0, "then none");
    top->irq_i = 0;
    b.write(CLAIM(0), 5);
    b.write(CLAIM(0), 7);
    b.write(CLAIM(0), 3);
    b.ticks(2);
    CHECK_EQ(top->meip_o, 0, "all completed");

    // ---- 9. context 1 (S-mode) is independent -----------------------------
    b.write(ENABLE(0), 0);
    b.write(ENABLE(1), 1u << 1);
    b.write(PRIO(1), 3);
    b.write(THRESH(1), 2);
    top->irq_i = 1u << 1;
    b.ticks(2);
    CHECK_EQ(top->seip_o, 1, "ctx1 sees source 1 (3 > 2)");
    CHECK_EQ(top->meip_o, 0, "ctx0 not enabled");
    CHECK_EQ(b.read(CLAIM(0)), 0, "ctx0 claims nothing");
    CHECK_EQ(b.read(CLAIM(1)), 1, "ctx1 claims it");
    CHECK_EQ(top->seip_o, 0, "in service");
    top->irq_i = 0;
    b.write(CLAIM(1), 1);
    b.ticks(2);
    CHECK_EQ(top->seip_o, 0, "quiet");

    // ---- 10. source 0 never interrupts; unmapped offsets read 0 -----------
    b.write(ENABLE(0), 0xFFFF);
    top->irq_i = 1;
    b.ticks(4);
    CHECK_EQ(top->meip_o, 0, "source 0 is 'no source'");
    CHECK_EQ(b.read(PENDING) & 1, 0, "and never pends");
    top->irq_i = 0;
    CHECK_EQ(b.read(0x1004), 0, "pending word 1 (no sources 32+) reads 0");
    CHECK_EQ(b.read(0x2004), 0, "enable word 1 reads 0");
    CHECK_EQ(b.read(0x202000), 0, "context 2 does not exist");
    CHECK_EQ(b.read(0x200008), 0, "reserved word after claim reads 0");

    int rc = bfm_report("plic_sifive_tb");
    delete top;
    return rc;
}

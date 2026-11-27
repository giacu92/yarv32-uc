/*
 * Startup for sim C programs (freestanding, rv32imac).
 *
 * _start is the reset entry (the CPU boots at 0x0; link.ld places
 * .text.init, and thus _start, at IMEM 0x0). It sets up the stack pointer
 * and calls main(); main() returns into an infinite loop (there is no
 * exit mechanism on this core).
 *
 * Harvard: fetch reads the dedicated I-mem (this image, IMEM 0-based);
 * the LSU reads/writes the dedicated D-mem (DMEM 0-based), where .data
 * globals and the stack live. sp = 0x4000 is the top of the 16 KiB
 * D-mem; the stack grows down from there. It must stay inside that
 * range: the D-mem decodes only ADDR_W bits, so a pointer above the top
 * aliases silently back into the data it is meant to sit clear of. The peri (MMIO) bus slave is
 * still tied off, so code that targets the peri address region stalls
 * the LSU; keep programs in the dmem region.
 */

    .section .text.init, "ax"
    .global _start
_start:
    li      sp, 0x00800000       /* top of the 8 MiB SDRAM (Linux SoC, unified) */

    /* Zero .bss. Nothing else does: the section is NOBITS, so it is not
     * part of the image the loader writes into the memories, and on
     * hardware its words come up holding whatever the BSRAM powered up
     * with. In simulation they read as zero, which is exactly what makes
     * this class of bug invisible until it reaches silicon -- an
     * uninitialised counter that happened to be non-zero once made a trap
     * probe report success without any trap having been taken.
     *
     * link.ld word-aligns both ends, so this stores words. It also emits
     * __bss_start == __bss_end when there is no .bss, and then the loop
     * body never runs. */
    la      t0, __bss_start
    la      t1, __bss_end
1:  bgeu    t0, t1, 2f
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       1b
2:

    /* Zero the argc/argv registers before the call. A freestanding _start
     * is the only thing that ever sets them, and a main() declared
     * main(int, char **) -- CoreMark's is -- reads whatever they hold.
     * On this core they come up as whatever the register file powered up
     * with; under Spike they hold the boot stub's hartid/DTB pointer. That
     * difference is architectural state, so the co-sim diverges on the
     * first instruction that touches a1 unless both start from a defined
     * value. */
    li      a0, 0                /* argc = 0 */
    li      a1, 0                /* argv = NULL */

    call    main
1:  j       1b                   /* main returns here; loop forever */
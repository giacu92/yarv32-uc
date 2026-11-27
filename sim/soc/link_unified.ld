/* Unified link for the Linux SoC sim (M5): one 8 MiB SDRAM space at 0,
 * cached on both sides, so .text, .rodata, .data and .bss simply follow one
 * another. Pair with soc/start_unified.S (sp at the top of SDRAM). The
 * section rules are those of sw/common/link.ld (small-data collection, bss
 * symbols); only the memory regions differ. */
OUTPUT_ARCH("riscv")
ENTRY(_start)
MEMORY
{
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 8M
}
SECTIONS
{
    .text : {
        KEEP(*(.text.init))   /* _start first -> IMEM 0x0 */
        *(.text*)
    } > RAM

    /* Small-data sections are collected into these same output sections on
     * purpose. RISC-V gcc puts any constant or variable of at most
     * -msmall-data-limit bytes (8 by default) into .srodata / .sdata /
     * .sbss rather than .rodata / .data / .bss. Left uncollected they
     * become orphan sections: the linker still gives them addresses, but
     * the image is extracted with `objcopy -j .rodata -j .data`, so their
     * contents never reach the D-mem hex and every read of them returns
     * whatever the memory happens to hold -- zero in simulation. A 4-byte
     * const read as 0 while the same constant folded at compile time
     * reads correctly is exactly as confusing as it sounds. */
    .rodata : { *(.rodata*) *(.srodata*) } > RAM
    .data   : { *(.data*) *(.sdata*) }     > RAM
    /* start.S zeroes this range at boot, so the symbols have to be here and
     * the section has to be word-aligned at both ends: the clear loop
     * stores words. Nothing else can do the job -- .bss is NOBITS, so it is
     * not in the image objcopy extracts, and on hardware those addresses
     * come up holding whatever the BSRAM powered up with. */
    . = ALIGN(4);
    __bss_start = .;
    .bss    : { *(.bss*) *(.sbss*) *(COMMON) } > RAM
    . = ALIGN(4);
    __bss_end = .;

    /* Drop metadata that has no place in a flat image. */
    /DISCARD/ : {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
        *(.riscv.attributes)
    }
}
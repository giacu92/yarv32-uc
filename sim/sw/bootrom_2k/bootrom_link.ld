/* Linker script for the 2 KiB boot ROM.
 *
 *   MEM (rwx) : ORIGIN = 0x0000, LENGTH = 2K
 *
 * ONE region, not the two of common/link.ld. On this target the boot ROM is a
 * single 2 KiB memory serving both CPU ports -- fetch reads it and the LSU
 * reads it -- so code, read-only data, data, .bss and the stack all come out
 * of the same 2 KiB budget and share one address space. The build cuts one
 * image from that space and formats it twice (64-bit for the fetch port,
 * 32-bit for the LSU port); see IMEM_SECTIONS / DMEM_SECTIONS in the Makefile.
 *
 * Consequences of the single space, all different from common/link.ld:
 *   - DMEM does NOT start at 0x2000. Data follows .text at whatever address
 *     the code ends, so DMEM_BASE is 0.
 *   - sp is the top of THIS memory (0x800), not 0x4000. bootrom_start.S sets
 *     it; the common start.S would put the stack 6 KiB past the end of the
 *     memory, where the D-mem's partial address decode aliases it silently
 *     back over the image.
 *   - .bss and the stack grow towards each other inside the same 2 KiB, so
 *     the ASSERTs at the bottom are the only thing standing between a program
 *     that grew and a stack that overwrites its own code.
 *
 * Unchanged from common/link.ld and worth repeating: medlow absolute
 * addressing (lui+addi, the gcc default) is mandatory -- do not switch to
 * medany; and the small-data sections (.srodata / .sdata / .sbss) must be
 * collected by hand into the ordinary ones, or the objcopy -j image loses
 * them and every read of them returns whatever the memory holds.
 */

OUTPUT_ARCH("riscv")
ENTRY(_start)

BOOTROM_SIZE = 2K;

/* Reserved for the stack, checked against .bss below. The banner path is
 * shallow (uart_puts -> uart_putc, no recursion, no buffers), so this is
 * generous; it is here to be a hard floor, not an estimate. */
STACK_SIZE = 256;

MEMORY
{
    MEM (rwx) : ORIGIN = 0x00800000, LENGTH = 2K
}

SECTIONS
{
    .text : {
        KEEP(*(.text.init))   /* _start first -> reset address 0x0 */
        *(.text*)
        . = ALIGN(4);
        __text_end = .;
    } > MEM

    .rodata : { *(.rodata*) *(.srodata*) } > MEM
    .data   : { *(.data*) *(.sdata*) }     > MEM

    /* bootrom_start.S zeroes this range at boot: .bss is NOBITS, so it is not
     * part of the image, and on hardware those words come up holding whatever
     * the BSRAM powered up with. Both ends are word-aligned because the clear
     * loop stores words. */
    . = ALIGN(4);
    __bss_start = .;
    .bss    : { *(.bss*) *(.sbss*) *(COMMON) } > MEM
    . = ALIGN(4);
    __bss_end = .;

    /* Top of the memory; bootrom_start.S loads this into sp and the stack
     * grows down from it towards __bss_end. */
    __stack_top = ORIGIN(MEM) + LENGTH(MEM);

    /* Drop metadata that has no place in a flat image. */
    /DISCARD/ : {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
        *(.riscv.attributes)
    }

    /* Enforced, not merely declared: a MEMORY region length does not reliably
     * fail the link on overflow in every ld configuration, and a boot ROM that
     * quietly outgrew its window would be found on the board rather than here.
     * Two checks, because the two ways to overrun 2 KiB have different fixes:
     * too much code/data, or a stack with nowhere to go. */
    ASSERT(__text_end - ORIGIN(MEM) <= BOOTROM_SIZE,
           "bootrom .text does not fit in the 2 KiB boot ROM")
    ASSERT(__bss_end - ORIGIN(MEM) + STACK_SIZE <= BOOTROM_SIZE,
           "bootrom image + reserved stack exceeds the 2 KiB boot ROM")
}

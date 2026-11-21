/*
 * Example program for the C -> program.hex flow (rv32imac).
 *
 * Compiled with the prebuilt bare-metal riscv32-esp-elf-gcc at
 * -march=rv32imac -mabi=ilp32 (freestanding, -nostdlib), linked at 0x0
 * (the CPU boot address) by link.ld, then converted to a $readmemh word
 * hex by bin2hex.py for the Verilator sim's RAM preload.
 *
 * The LSU is live (loads/stores retire through the shared imem bus) and
 * the stall-on-RAW interlock covers register and load-use hazards, so a
 * memory-heavy program like quicksort is now a real correctness check:
 * it exercises the fetch / decode / execute path AND the data path (array
 * loads/stores, stack spill/fill from recursion, branches on loaded
 * values). The retire+writeback log (a0 at exit) is the observable
 * result. Zilx indexed loads are NOT emitted by -march=rv32imac (no
 * zilx mnemonics in the assembler), so partition() hand-encodes a
 * scaled word indexed load (lxs.w) via .insn to exercise that path
 * for real here. The hand-crafted sim/program.hex oracle still covers
 * the other Zilx sizes/signs (b/h/w, signed/unsigned) and the unscaled
 * variant; this program only stresses lxs.w.
 *
 * .bss is NOT zeroed at runtime (start.S does not clear it), so keep
 * state in a .data array, not uninitialized globals.
 *
 * The array is 256 words, filled by a deterministic LCG and printed before
 * and after the sort (PRINT_ARRAY=0 compiles the printing out for the
 * co-simulation, which cannot follow a UART access).
 */

#define N 256

#include "uart.h"

/* Printing the array is a board-side aid, not part of the sort. It is
 * compiled out for the co-simulation: the first UART access is where Spike
 * and the RTL stop being comparable (Spike has no UART slave), so printing
 * before the sort would end the retire-by-retire diff before it ever saw
 * the algorithm. Build with -DPRINT_ARRAY=0 for that case. */
#ifndef PRINT_ARRAY
#define PRINT_ARRAY 1
#endif

/* Forced into .data rather than left to .bss: nothing zeroes .bss here
 * (start.S does not, and .bss is NOBITS so it is not in the loaded image
 * either), so a .bss array holds whatever the BSRAM powered up with. In
 * .data it is part of the image and starts as zeros, which the fill below
 * then overwrites. volatile forces a real load/store per access and stops
 * -O2 from constant-folding the whole sort away.
 *
 * 256 words = 1 KiB of the 8 KiB D-mem, which spans 0x2000-0x3FFF with the
 * stack growing down from 0x4000. */
static volatile int arr[N] __attribute__((section(".data")));

/* Deterministic pseudo-random fill: the same sequence on Spike and on the
 * RTL, so the co-sim still compares like for like, and unsorted enough that
 * quicksort's recursion stays near log2(N) deep instead of degenerating to
 * N frames the way an already-sorted input would. A plain 32-bit LCG
 * (Numerical Recipes constants), taking the high bits because the low ones
 * of an LCG are barely random, mapped to a small signed range so the
 * printout stays readable. */
static void fill_array(void)
{
    unsigned int state = 0x12345678u;
    for (int i = 0; i < N; i++) {
        state  = state * 1664525u + 1013904223u;
        arr[i] = (int)((state >> 16) & 0x1FFu) - 256;
    }
}

#if PRINT_ARRAY
/* One line per 8 entries, hex so no division is needed (this build has no
 * libc, and a software divide per digit would dominate the run). */
static void print_array(const char *label)
{
    uart_puts(label);
    for (int i = 0; i < N; i++) {
        if ((i % 8) == 0) uart_puts("\r\n");
        uart_put_hex32((unsigned int)arr[i]);
        uart_putc(' ');
    }
    uart_puts("\r\n");
}
#else
#define print_array(label) ((void)0)
#endif

/* Zilx (draft) scaled indexed word load: val = *(base + (idx << 2)).
 * Hand-encoded via .insn since -march=rv32imac's assembler has no
 * zilx mnemonics: AMO opcode (0x2f), funct3=010 (word), funct5=11010
 * scaled-indexed mode, aq=rl=0 -> funct7 = 0b1101000 = 0x68.
 */
/* QS_ZILX selects the encoding. It follows rv32_pkg::ZILX_EN: 1 on `main`
 * (the core decodes Zilx), 0 on the Linux branch `linux-soc`, whose core
 * gives the AMO opcode to the A extension and traps Zilx as illegal. With 0
 * the same load is plain C, so the program and its Spike cosim still run.
 */
#ifndef QS_ZILX
#define QS_ZILX 0
#endif
static inline int lxsw(volatile int *base, int idx)
{
#if QS_ZILX
    int val;
    __asm__ volatile (".insn r 0x2f, 0x2, 0x68, %0, %1, %2"
                       : "=r"(val)
                       : "r"(idx), "r"(base)
                       : "memory");
    return val;
#else
    return base[idx];
#endif
}

/* Partition (Lomuto) over arr[lo..hi] with arr[hi] as pivot. */
__attribute__((noinline))
static int partition(int lo, int hi)
{
    int pivot = lxsw(arr, hi);
    int i = lo - 1;
    for (int j = lo; j < hi; j++) {
        int aj = lxsw(arr, j);
        if (aj <= pivot) {
            i++;
            int ai = lxsw(arr, i);
            arr[i] = aj;
            arr[j] = ai;
        }
    }
    int lo_v = lxsw(arr, i + 1);
    int hi_v = lxsw(arr, hi);
    arr[i + 1] = hi_v;
    arr[hi] = lo_v;
    return i + 1;
}

/* Recursive quicksort. Recursion drives stack spill/fill, so this also
 * stresses the LSU's stack path and the RAW interlock on load-use. */
__attribute__((noinline))
static void quicksort(int lo, int hi)
{
    if (lo < hi) {
        int p = partition(lo, hi);
        quicksort(lo, p - 1);
        quicksort(p + 1, hi);
    }
}

int main(void)
{
    fill_array();
    print_array("before:");

    quicksort(0, N - 1);

    print_array("after:");

    /* Verify ascending order; return a recognizable marker in a0.
     * 0x600D == sorted, 0x00000BAD == still unsorted (sort broken). */
    int ok = 1;
    for (int i = 1; i < N; i++) {
        if (arr[i - 1] > arr[i])
        {
            ok = 0;
            break;
        }
    }

    if (ok) {
        uart_puts("OK\r\n");
        return 0x600D;
    } else {
        uart_puts("FAIL\r\n");
        return 0x00000BAD;
    }
}
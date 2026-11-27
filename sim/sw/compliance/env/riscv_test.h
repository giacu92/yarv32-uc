// Harvard / M-mode replacement for riscv-tests' env/p/riscv_test.h.
//
// The upstream "p" (physical, single core) environment cannot be used as-is on
// this core, for two independent reasons:
//
//   1. It links everything -- code, tohost, data -- into ONE address space at
//      0x8000_0000. This core is Harvard: fetch reads a dedicated I-mem at 0
//      and the LSU reads a dedicated D-mem at 0x2000. A single-space image
//      cannot be split after the fact, so the layout has to be decided by the
//      linker script (env/link_harvard.ld) and the environment has to agree
//      with it: the reset vector must sit at I-mem 0 and `tohost` must sit in
//      the D-mem, not next to the code.
//
//   2. Its reset vector programs CSRs this core does not implement (satp,
//      pmpaddr0/pmpcfg0, mnstatus, medeleg/mideleg). Today those writes are
//      silently ignored (unimplemented CSR addresses read 0 / drop writes), so
//      they would be harmless -- but making the environment depend on that is
//      exactly backwards: plan item 5 is to start trapping illegal CSR access,
//      and on the day that lands every test would fail in the environment
//      rather than in the instruction under test. This core is M-mode only
//      with no PMP and no MMU, so those initialisations have nothing to do.
//
// Everything else -- the macro *interface* the test sources use (RVTEST_RV32U,
// RVTEST_CODE_BEGIN, RVTEST_PASS, RVTEST_DATA_BEGIN, TESTNUM ...) -- is kept
// bit-for-bit compatible with upstream, because the test sources under
// riscv-tests/ are vendored verbatim and must never be edited.
//
// Result protocol (what the sim harness reads back):
//   tohost, the first word of the D-mem image, lives at 0x2000 by
//   construction (link_harvard.ld puts the .tohost input section first in the
//   DMEM region, and the region's ORIGIN is 64-byte aligned so the section's
//   own `.align 6` is a no-op). The trap handler writes it and then parks in a
//   one-instruction self-loop, which is what sim_main.cpp's park detector
//   (8 consecutive identical retires) needs to stop the run early.
//     tohost == 1        -> PASS
//     tohost == 2*n+1    -> FAIL in test case n
//     tohost == 0        -> the test never reached pass or fail (hang/timeout)
//   That is upstream's encoding, unchanged, so a failure number can be looked
//   up directly in the test source.

#ifndef _ENV_YARV_HARVARD_M_H
#define _ENV_YARV_HARVARD_M_H

#include "encoding.h"

//-----------------------------------------------------------------------
// Begin Macro
//-----------------------------------------------------------------------

// The rv32* test sources arrive as `#undef RVTEST_RV64U` / `#define
// RVTEST_RV64U RVTEST_RV32U` wrappers around the rv64* body, so both spellings
// have to exist. `init` is the per-suite hook RVTEST_CODE_BEGIN invokes.
#define RVTEST_RV32U                                                    \
  .macro init;                                                          \
  .endm

#define RVTEST_RV64U RVTEST_RV32U

#define RVTEST_ENABLE_MACHINE                                           \
  li a0, MSTATUS_MPP;                                                   \
  csrs mstatus, a0;

#define RVTEST_RV32M                                                    \
  .macro init;                                                          \
  RVTEST_ENABLE_MACHINE;                                                \
  .endm

#define RVTEST_RV64M RVTEST_RV32M

// No S-mode on this core. The rv32mi wrappers around the rv64si sources
// already redefine RVTEST_RV64S to RVTEST_RV32M themselves; these definitions
// only exist so that a source which does not can still assemble, and it will
// run in M-mode.
#define RVTEST_RV32S RVTEST_RV32M
#define RVTEST_RV64S RVTEST_RV32M

// XLEN self-check: on RV32, 1<<31 is negative. A 64-bit core would fall
// through to RVTEST_PASS, i.e. report "passed" rather than execute a test
// whose expectations are all 64-bit wide.
#define CHECK_XLEN li a0, 1; slli a0, a0, 31; bltz a0, 1f; RVTEST_PASS; 1:

#define INIT_XREG                                                       \
  li x1, 0;                                                             \
  li x2, 0;                                                             \
  li x3, 0;                                                             \
  li x4, 0;                                                             \
  li x5, 0;                                                             \
  li x6, 0;                                                             \
  li x7, 0;                                                             \
  li x8, 0;                                                             \
  li x9, 0;                                                             \
  li x10, 0;                                                            \
  li x11, 0;                                                            \
  li x12, 0;                                                            \
  li x13, 0;                                                            \
  li x14, 0;                                                            \
  li x15, 0;                                                            \
  li x16, 0;                                                            \
  li x17, 0;                                                            \
  li x18, 0;                                                            \
  li x19, 0;                                                            \
  li x20, 0;                                                            \
  li x21, 0;                                                            \
  li x22, 0;                                                            \
  li x23, 0;                                                            \
  li x24, 0;                                                            \
  li x25, 0;                                                            \
  li x26, 0;                                                            \
  li x27, 0;                                                            \
  li x28, 0;                                                            \
  li x29, 0;                                                            \
  li x30, 0;                                                            \
  li x31, 0;

// Upstream's EXTRA_* / FILTER_* hooks. No vendored rv32ui/um/uc/mi source
// uses them, but they are part of the interface a test source may reference,
// so they exist and expand to nothing.
#define EXTRA_TVEC_USER
#define EXTRA_TVEC_MACHINE
#define EXTRA_INIT
#define EXTRA_INIT_TIMER
#define EXTRA_DATA
#define FILTER_TRAP
#define FILTER_PAGE_FAULT

#define RVTEST_CODE_BEGIN                                               \
        .section .text.init;                                            \
        .align  6;                                                      \
        .weak mtvec_handler;                                            \
        .globl _start;                                                  \
_start:                                                                 \
        /* reset vector: the core boots at I-mem 0 and link_harvard.ld   \
           places .text.init first, so this instruction IS address 0. */ \
        j reset_vector;                                                 \
        .align 2;                                                       \
trap_vector:                                                            \
        /* Only three things can land here: the ecall that RVTEST_PASS / \
           RVTEST_FAIL use to report, a trap the test installed an       \
           mtvec_handler for, or a genuine failure. linux-soc has M and  \
           U (no S): the rv32u* bodies run in U exactly as upstream's,   \
           since mret goes to the cleared MPP, so their report ecall is  \
           CAUSE_USER_ECALL; rv32m* bodies report with                    \
           CAUSE_MACHINE_ECALL. */                                        \
        csrr t5, mcause;                                                \
        li t6, CAUSE_USER_ECALL;                                        \
        beq t5, t6, write_tohost;                                       \
        li t6, CAUSE_MACHINE_ECALL;                                     \
        beq t5, t6, write_tohost;                                       \
        /* if the test defined an mtvec_handler, hand the trap to it */  \
        la t5, mtvec_handler;                                           \
        beqz t5, other_exception;                                       \
        jr t5;                                                          \
other_exception:                                                        \
        /* Unhandled trap. Upstream's encoding: OR 1337 into the test    \
           number so the reported value is unmistakably not a plain      \
           per-case failure. Interrupts land here too -- none should     \
           occur, nothing arms one. */                                   \
        ori TESTNUM, TESTNUM, 1337;                                     \
write_tohost:                                                           \
        sw TESTNUM, tohost, t5;                                         \
        sw zero, tohost + 4, t5;                                        \
        /* Park in a ONE-instruction self-loop, not in a loop back to    \
           write_tohost as upstream does: sim_main.cpp stops the run     \
           early on 8 consecutive identical retires, and a multi-        \
           instruction loop never produces those -- the run would burn   \
           its whole MAX_CYC budget after every test. */                 \
  1:    j 1b;                                                           \
reset_vector:                                                           \
        INIT_XREG;                                                      \
        li TESTNUM, 0;                                                  \
        la t0, trap_vector;                                             \
        csrw mtvec, t0;                                                 \
        CHECK_XLEN;                                                     \
        csrwi mstatus, 0;                                               \
        init;                                                           \
        EXTRA_INIT;                                                     \
        EXTRA_INIT_TIMER;                                               \
        /* mret into the test body. mstatus.MPP was just cleared, so a   \
           rv32u* body runs in U (as upstream); RVTEST_RV32M's init sets \
           MPP back to M first. a0 = 0 stands in for mhartid, which this \
           core does not implement (it reads 0 anyway). */                \
        la t0, 1f;                                                      \
        csrw mepc, t0;                                                  \
        li a0, 0;                                                       \
        mret;                                                           \
1:

//-----------------------------------------------------------------------
// End Macro
//-----------------------------------------------------------------------

// Unreachable in practice: every test body ends in TEST_PASSFAIL, which
// always leaves via RVTEST_PASS or RVTEST_FAIL. Kept because the test sources
// emit it.
#define RVTEST_CODE_END                                                 \
        unimp

//-----------------------------------------------------------------------
// Pass/Fail Macro
//-----------------------------------------------------------------------

#define TESTNUM gp

#define RVTEST_PASS                                                     \
        fence;                                                          \
        li TESTNUM, 1;                                                  \
        li a7, 93;                                                      \
        li a0, 0;                                                       \
        ecall

#define RVTEST_FAIL                                                     \
        fence;                                                          \
1:      beqz TESTNUM, 1b;                                               \
        sll TESTNUM, TESTNUM, 1;                                        \
        or TESTNUM, TESTNUM, 1;                                         \
        li a7, 93;                                                      \
        addi a0, TESTNUM, 0;                                            \
        ecall

//-----------------------------------------------------------------------
// Data Section Macro
//-----------------------------------------------------------------------

// Identical to upstream. .tohost is a separate input section so the linker
// script can pin it at the bottom of the D-mem (0x2000) regardless of how
// much .data a given test carries.
#define RVTEST_DATA_BEGIN                                               \
        EXTRA_DATA                                                      \
        .pushsection .tohost,"aw",@progbits;                            \
        .align 6; .global tohost; tohost: .dword 0; .size tohost, 8;    \
        .align 6; .global fromhost; fromhost: .dword 0; .size fromhost, 8;\
        .popsection;                                                    \
        .align 4; .global begin_signature; begin_signature:

#define RVTEST_DATA_END .align 4; .global end_signature; end_signature:

#endif

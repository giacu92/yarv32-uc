# U-mode oracle (linux-soc): privilege transitions between M and U, the
# U-mode ecall cause, CSR / mret / wfi protection in U, MPP WARL, and the
# rule that M interrupts are taken in U whatever mstatus.MIE says.
# Writes 0x600D / 0xBAD @0x2000 and, on failure, the failing check @0x2004.
#
# Why it exists: a nommu M-mode Linux kernel runs userspace in U and only
# handles ecall cause 8, so the M-only core panicked at the first syscall
# of /bin/sh (2026-11-27). Pure assembly with its own _start.

S_SRCS := umode_ops_test.S

include ../../common/sw_build.mk

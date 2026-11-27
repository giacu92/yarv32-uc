#!/usr/bin/env python3
"""Toolchain-free builder for the WFI-wake arbitration oracle.

wfi_trap_test.S is the readable source of truth; this script hand-encodes
the SAME program so the regression can run on a machine that has Verilator
but no riscv32 toolchain (`make` in this directory needs the toolchain,
`make hex` does not). Keep the two in sync -- the instruction list below is
a line-for-line transcription of the .S file.

Emits build/imem.hex (word-indexed $readmemh, I-mem at 0) and
build/dmem.hex (placeholder; the program writes all its own data).
"""
import os
import sys

X = {"x0": 0, "sp": 2, "t0": 5, "t1": 6, "t2": 7, "t3": 28, "t4": 29, "t5": 30}
CSR = {"mstatus": 0x300, "mie": 0x304, "mtvec": 0x305,
       "mepc": 0x341, "mcause": 0x342}


def _u(v, n):
    return v & ((1 << n) - 1)


def r_type(f7, rs2, rs1, f3, rd, op):
    return (_u(f7, 7) << 25) | (_u(rs2, 5) << 20) | (_u(rs1, 5) << 15) \
        | (_u(f3, 3) << 12) | (_u(rd, 5) << 7) | _u(op, 7)


def i_type(imm, rs1, f3, rd, op):
    return (_u(imm, 12) << 20) | (_u(rs1, 5) << 15) | (_u(f3, 3) << 12) \
        | (_u(rd, 5) << 7) | _u(op, 7)


def s_type(imm, rs2, rs1, f3, op):
    return (_u(imm >> 5, 7) << 25) | (_u(rs2, 5) << 20) | (_u(rs1, 5) << 15) \
        | (_u(f3, 3) << 12) | (_u(imm, 5) << 7) | _u(op, 7)


def b_type(imm, rs2, rs1, f3, op):
    return (((imm >> 12) & 1) << 31) | (_u(imm >> 5, 6) << 25) \
        | (_u(rs2, 5) << 20) | (_u(rs1, 5) << 15) | (_u(f3, 3) << 12) \
        | (_u(imm >> 1, 4) << 8) | (((imm >> 11) & 1) << 7) | _u(op, 7)


def j_type(imm, rd, op):
    return (((imm >> 20) & 1) << 31) | (_u(imm >> 1, 10) << 21) \
        | (((imm >> 11) & 1) << 20) | (_u(imm >> 12, 8) << 12) \
        | (_u(rd, 5) << 7) | _u(op, 7)


def lui(rd, imm20):
    return (_u(imm20, 20) << 12) | (_u(rd, 5) << 7) | 0x37


def addi(rd, rs1, imm):
    return i_type(imm, rs1, 0b000, rd, 0x13)


def andi(rd, rs1, imm):
    return i_type(imm, rs1, 0b111, rd, 0x13)


def and_(rd, rs1, rs2):
    return r_type(0, rs2, rs1, 0b111, rd, 0x33)


def sw(rs2, off, rs1):
    return s_type(off, rs2, rs1, 0b010, 0x23)


def lw(rd, off, rs1):
    return i_type(off, rs1, 0b010, rd, 0x03)


def bne(rs1, rs2, off):
    return b_type(off, rs2, rs1, 0b001, 0x63)


def jal(rd, off):
    return j_type(off, rd, 0x6F)


def csrrw(rd, csr, rs1):
    return i_type(csr, rs1, 0b001, rd, 0x73)


def csrrs(rd, csr, rs1):
    return i_type(csr, rs1, 0b010, rd, 0x73)


def div(rd, rs1, rs2):
    return r_type(0b0000001, rs2, rs1, 0b100, rd, 0x33)


WFI = 0x10500073
MRET = 0x30200073
ILLEGAL = 0x0000007F

# li rd, imm32 -> always two words (lui+addi) so instruction addresses do
# not shift between the sizing pass and the emit pass.
def li(rd, imm):
    imm &= 0xFFFFFFFF
    hi = (imm + 0x800) >> 12
    lo = imm - (_u(hi, 20) << 12)
    lo = ((lo + 0x800) & 0xFFF) - 0x800  # sign-extend to 12 bits
    return [lui(rd, hi), addi(rd, rd, lo)]


# (label, mnemonic, args). Transcribed from wfi_trap_test.S.
PROG = [
    ("_start",        "li",    ("sp", 0x4000)),
    (None,            "li",    ("t0", "trap_handler")),
    (None,            "csrw",  ("mtvec", "t0")),
    (None,            "li",    ("t0", 0x2040)),
    (None,            "sw",    ("x0", 0, "t0")),
    (None,            "sw",    ("x0", 4, "t0")),
    (None,            "li",    ("t1", 0x80)),
    (None,            "csrs",  ("mie", "t1")),
    (None,            "li",    ("t1", 0x8)),
    (None,            "csrs",  ("mstatus", "t1")),
    (None,            "li",    ("t0", 0x10014000)),
    (None,            "sw",    ("x0", 4, "t0")),
    (None,            "li",    ("t2", 0x1001BFF8)),
    (None,            "lw",    ("t1", 0, "t2")),
    (None,            "addi",  ("t1", "t1", 2000)),
    (None,            "sw",    ("t1", 0, "t0")),
    # Long multi-cycle op so fetch runs ahead and fills F/D + skid: the
    # cycle the wfi retires, decode must latch the illegal word behind it
    # into D/E. Without that, D/E holds a bubble through the halt, the
    # sync trap cannot race the interrupt at the wake, and the arbitration
    # case this oracle exists to test is never reached.
    (None,            "li",    ("t1", 3)),
    (None,            "li",    ("t2", 77)),
    (None,            "div",   ("t2", "t2", "t1")),
    (None,            "wfi",   ()),
    (None,            "word",  (ILLEGAL,)),
    (None,            "li",    ("t0", 0x2040)),
    (None,            "lw",    ("t1", 0, "t0")),
    (None,            "li",    ("t2", 2)),
    (None,            "bne",   ("t1", "t2", "bad")),
    (None,            "lw",    ("t1", 4, "t0")),
    (None,            "li",    ("t2", 7)),
    (None,            "bne",   ("t1", "t2", "bad")),
    (None,            "li",    ("t0", 0x2000)),
    (None,            "li",    ("t1", 0x600D)),
    (None,            "sw",    ("t1", 0, "t0")),
    (None,            "j",     ("park",)),
    ("bad",           "li",    ("t0", 0x2000)),
    (None,            "li",    ("t1", 0xBAD)),
    (None,            "sw",    ("t1", 0, "t0")),
    ("park",          "j",     ("park",)),
    ("trap_handler",  "csrr",  ("t0", "mcause")),
    (None,            "li",    ("t2", 0x80000000)),
    (None,            "and",   ("t1", "t0", "t2")),
    (None,            "bne",   ("t1", "x0", "int_path")),
    (None,            "andi",  ("t0", "t0", 0x1FF)),
    (None,            "li",    ("t3", 0x2040)),
    (None,            "sw",    ("t0", 0, "t3")),
    (None,            "csrr",  ("t1", "mepc")),
    (None,            "addi",  ("t1", "t1", 4)),
    (None,            "csrw",  ("mepc", "t1")),
    (None,            "mret",  ()),
    ("int_path",      "andi",  ("t0", "t0", 0x1FF)),
    (None,            "li",    ("t3", 0x2040)),
    (None,            "sw",    ("t0", 4, "t3")),
    (None,            "li",    ("t4", 0x10014000)),
    (None,            "li",    ("t5", -1)),
    (None,            "sw",    ("t5", 4, "t4")),
    (None,            "sw",    ("t5", 0, "t4")),
    (None,            "mret",  ()),
]

SIZE = {"li": 2}  # every other mnemonic is one word


def assemble():
    labels, pc = {}, 0
    for label, mn, _ in PROG:
        if label:
            labels[label] = pc
        pc += 4 * SIZE.get(mn, 1)

    words, pc = [], 0
    for _, mn, a in PROG:
        if mn == "li":
            imm = labels[a[1]] if isinstance(a[1], str) else a[1]
            out = li(X[a[0]], imm)
        elif mn == "word":
            out = [a[0]]
        elif mn == "wfi":
            out = [WFI]
        elif mn == "mret":
            out = [MRET]
        elif mn == "sw":
            out = [sw(X[a[0]], a[1], X[a[2]])]
        elif mn == "lw":
            out = [lw(X[a[0]], a[1], X[a[2]])]
        elif mn == "addi":
            out = [addi(X[a[0]], X[a[1]], a[2])]
        elif mn == "andi":
            out = [andi(X[a[0]], X[a[1]], a[2])]
        elif mn == "and":
            out = [and_(X[a[0]], X[a[1]], X[a[2]])]
        elif mn == "div":
            out = [div(X[a[0]], X[a[1]], X[a[2]])]
        elif mn == "bne":
            out = [bne(X[a[0]], X[a[1]], labels[a[2]] - pc)]
        elif mn == "j":
            out = [jal(0, labels[a[0]] - pc)]
        elif mn == "csrw":
            out = [csrrw(0, CSR[a[0]], X[a[1]])]
        elif mn == "csrs":
            out = [csrrs(0, CSR[a[0]], X[a[1]])]
        elif mn == "csrr":
            out = [csrrs(X[a[0]], CSR[a[1]], 0)]
        else:
            raise SystemExit("unknown mnemonic: " + mn)
        words += out
        pc += 4 * len(out)
    return labels, words


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "build"
    os.makedirs(out_dir, exist_ok=True)
    labels, words = assemble()

    with open(os.path.join(out_dir, "imem.hex"), "w") as f:
        # 64-bit $readmemh elements (I-mem fetch port is 8 bytes wide): pair
        # two 32-bit instruction words per line, low 32 bits = the word at the
        # byte address, high 32 bits = the next (+4). Zero-pad the upper half
        # if the image has an odd word count. @0 is an element index.
        f.write("@00000000\n")
        f.write("// WFI-wake arbitration oracle -- generated by gen_hex.py\n")
        f.write("// from wfi_trap_test.S (keep the two in sync).\n")
        for name, addr in sorted(labels.items(), key=lambda kv: kv[1]):
            f.write("// %-14s 0x%08x\n" % (name, addr))
        padded = words + [0] if len(words) % 2 else words
        for i in range(0, len(padded), 2):
            f.write("%016x\n" % ((padded[i + 1] << 32) | padded[i]))

    with open(os.path.join(out_dir, "dmem.hex"), "w") as f:
        f.write("@00000800\n")
        f.write("// Placeholder: the program writes its own data at runtime\n")
        f.write("// (pass sentinel at 0x2000 = probe word 0x800, markers at\n")
        f.write("// 0x2040 / 0x2044).\n")
        f.write("00000000\n")

    print("wrote %s/imem.hex (%d instr words), %s/dmem.hex" %
          (out_dir, len(words), out_dir))
    for name, addr in sorted(labels.items(), key=lambda kv: kv[1]):
        print("  %-14s 0x%08x" % (name, addr))


if __name__ == "__main__":
    main()

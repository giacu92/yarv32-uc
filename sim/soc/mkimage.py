#!/usr/bin/env python3
"""Merge a Harvard image pair into one unified SDRAM image (Linux SoC sim).

  mkimage.py <imem.hex> <dmem.hex> <out sdram.hex>

imem.hex : 64-bit words, one per line, low word first, optional @<word64>
dmem.hex : 32-bit words, optional @<word32> address records
out      : 32-bit words with @<word32> address records, for $readmemh into
           the SDRAM model (word index = byte address / 4)

The Harvard link puts .text at 0 and .data at 0x2000 in two separate spaces.
In one space they collide unless .text ends below 0x2000; that is checked,
and an image that does not fit is refused rather than silently corrupted.
"""
import sys

def words(path, width):
    addr, out = 0, {}
    for tok in open(path).read().split():
        if tok.startswith('@'):
            addr = int(tok[1:], 16)
            continue
        out[addr] = int(tok, 16)
        addr += 1
    return out

imem, dmem, dst = sys.argv[1:4]
mem = {}
for a64, v in words(imem, 64).items():
    for half in (0, 1):
        w = (v >> (32 * half)) & 0xFFFFFFFF
        if w or True:
            mem[2 * a64 + half] = w
text_end = max(mem) + 1 if mem else 0
# trailing padding of the I-image is not code: zeros, or the IMEM_PAD_VALUE
# fill sw_build.mk can append (0x00100073, ebreak)
PAD = (0x00000000, 0x00100073)
while text_end and mem.get(text_end - 1, 0) in PAD:
    text_end -= 1
for a in [a for a in mem if a >= text_end]:
    del mem[a]
dm = words(dmem, 32)
lo_data = min((a for a, v in dm.items() if v), default=None)
if lo_data is not None and lo_data < text_end:
    sys.exit(f"mkimage: .text ends at byte 0x{4*text_end:x} but data starts at "
             f"0x{4*lo_data:x}: the Harvard layout does not fit one space")
for a, v in dm.items():
    if a >= text_end or v:
        mem[a] = v
with open(dst, 'w') as f:
    prev = None
    for a in sorted(mem):
        if prev is None or a != prev + 1:
            f.write(f"@{a:08x}\n")
        f.write(f"{mem[a]:08x}\n")
        prev = a

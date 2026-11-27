#!/usr/bin/env python3
"""Build a yload frame (see yload.S) from an SDRAM word image.

  yframe.py <sdram.hex> [--entry 0x0] [--escape]

<sdram.hex> is a 32-bit word image with @word addresses (soc/mkimage.py or
soc/elf2sdram.sh output). The frame loads the contiguous byte range from the
lowest to the highest word present. Words missing inside that range are sent
as zero. Output is the raw frame on stdout, or with --escape a C-escaped
string for sim_main's UART_RX environment variable.
"""
import sys, zlib, argparse

ap = argparse.ArgumentParser()
ap.add_argument('hexfile')
ap.add_argument('--entry', default='0')
ap.add_argument('--escape', action='store_true')
a = ap.parse_args()

words, addr = {}, 0
for tok in open(a.hexfile).read().split():
    if tok.startswith('@'):
        addr = int(tok[1:], 16)
        continue
    words[addr] = int(tok, 16)
    addr += 1
lo, hi = min(words), max(words) + 1
payload = b''.join(words.get(w, 0).to_bytes(4, 'little') for w in range(lo, hi))
u32 = lambda v: v.to_bytes(4, 'little')
frame = (b'YL' + u32(4 * lo) + u32(len(payload)) + u32(int(a.entry, 0)) + payload +
         u32(zlib.crc32(payload) & 0xFFFFFFFF))
if a.escape:
    sys.stdout.write(''.join(f'\\x{c:02x}' for c in frame))
else:
    sys.stdout.buffer.write(frame)

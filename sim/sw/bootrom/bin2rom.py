#!/usr/bin/env python3
"""bin2rom.py <in.bin> <out.hex> <max bytes>: flat binary -> 64-bit word hex
(low byte first) for the cache's boot ROM macro. Refuses an oversized image."""
import sys

src, dst, cap = sys.argv[1], sys.argv[2], int(sys.argv[3])
b = open(src, 'rb').read()
if len(b) > cap:
    sys.exit(f"bin2rom: {len(b)} bytes does not fit the {cap}-byte boot ROM")
b += b'\0' * (-len(b) % 8)
with open(dst, 'w') as f:
    f.write('@00000000\n')
    for i in range(0, len(b), 8):
        f.write(f"{int.from_bytes(b[i:i + 8], 'little'):016x}\n")
print(f"{src}: {len(b)} of {cap} bytes")

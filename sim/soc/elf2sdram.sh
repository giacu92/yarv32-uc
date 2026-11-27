#!/usr/bin/env bash
# elf2sdram.sh <program.elf> <out.hex> <riscv prefix>
# Flat binary of every loadable section (VMA 0 based) -> 32-bit word hex for
# the SDRAM model. For images linked with soc/link_unified.ld.
set -euo pipefail
"$3-objcopy" -O binary "$1" "$2.bin"
python3 - "$2.bin" "$2" <<'PY'
import sys
b = open(sys.argv[1], 'rb').read()
b += b'\0' * (-len(b) % 4)
with open(sys.argv[2], 'w') as f:
    f.write('@00000000\n')
    for i in range(0, len(b), 4):
        f.write(f"{int.from_bytes(b[i:i+4], 'little'):08x}\n")
PY
rm -f "$2.bin"

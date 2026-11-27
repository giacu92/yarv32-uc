#!/usr/bin/env python3
"""Send a program to the yload boot loader over the board UART, then stay
on the line as a serial terminal.

    yboot.py <frame.yl | sdram.hex> [--port /dev/ttyUSB1] [--baud 115200]
             [--entry 0x80000000] [--no-wait]

<frame.yl> is a raw yload frame (yframe.py output). A .hex SDRAM word image
is framed on the fly with --entry (default 0x8000_0000, the Linux entry; use
0 for a bare-metal program linked at SDRAM 0).

Sequence: wait for the loader's 'Y' (press the board's reset button S1 if it
does not come: the loader sends it once, when it falls back to the UART),
send the frame with a progress line, expect 'K' (CRC good, jumping) or 'E'
(CRC bad), then act as a terminal: everything received is printed,
keystrokes are sent. Ctrl-] quits.

Tang Nano 20K: the BL616 debugger enumerates as an FT2232-compatible device
(0403:6010); interface 0 is JTAG, interface 1 (/dev/ttyUSB1,
/dev/serial/by-id/usb-SIPEED_USB_Debugger_*-if01-port0) is the UART.
Needs pyserial.
"""
import argparse, os, select, sys, termios, time, tty, zlib

import serial

ap = argparse.ArgumentParser()
ap.add_argument('image')
ap.add_argument('--port', default='/dev/ttyUSB1')
ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--entry', default='0x80000000')
ap.add_argument('--no-wait', action='store_true', help="send without waiting for 'Y'")
a = ap.parse_args()


def frame_from_hex(path, entry):
    words, addr = {}, 0
    for tok in open(path).read().split():
        if tok.startswith('@'):
            addr = int(tok[1:], 16)
            continue
        words[addr] = int(tok, 16)
        addr += 1
    lo, hi = min(words), max(words) + 1
    payload = b''.join(words.get(w, 0).to_bytes(4, 'little') for w in range(lo, hi))
    u32 = lambda v: v.to_bytes(4, 'little')
    return (b'YL' + u32(4 * lo) + u32(len(payload)) + u32(entry) + payload +
            u32(zlib.crc32(payload) & 0xFFFFFFFF))


if a.image.endswith('.hex'):
    frame = frame_from_hex(a.image, int(a.entry, 0))
else:
    frame = open(a.image, 'rb').read()
if frame[:2] != b'YL':
    sys.exit('not a yload frame (no YL magic)')
length = int.from_bytes(frame[6:10], 'little')
print(f'frame: load 0x{int.from_bytes(frame[2:6], "little"):08x}, {length} bytes, '
      f'entry 0x{int.from_bytes(frame[10:14], "little"):08x}; '
      f'~{len(frame) * 10 / a.baud:.0f} s at {a.baud} baud')

s = serial.Serial(a.port, a.baud, timeout=0.1)
s.reset_input_buffer()
if not a.no_wait:
    print("waiting for the loader's 'Y' -- press reset (S1) on the board if nothing happens")
    while True:
        c = s.read(1)
        if c == b'Y':
            break
        if c:
            sys.stdout.write(c.decode('latin-1'))
            sys.stdout.flush()
print('sending...')
t0 = time.time()
CH = 4096
for i in range(0, len(frame), CH):
    s.write(frame[i:i + CH])
    s.flush()
    done = min(i + CH, len(frame))
    el = time.time() - t0
    sys.stdout.write(f'\r  {done}/{len(frame)} bytes  {100 * done // len(frame)}%  {el:.0f} s')
    sys.stdout.flush()
print()
deadline = time.time() + 10
ack = b''
while time.time() < deadline and not ack:
    ack = s.read(1)
if ack == b'K':
    print(f"'K': CRC good, loader jumped to the entry point ({time.time() - t0:.0f} s)")
elif ack == b'E':
    sys.exit("'E': CRC mismatch, the loader is waiting for a new frame")
else:
    print(f'no ack (got {ack!r}); staying on the line anyway')

print('--- terminal (Ctrl-] quits) ---')
fd = sys.stdin.fileno()
old = termios.tcgetattr(fd) if os.isatty(fd) else None
try:
    if old:
        tty.setraw(fd)
    while True:
        r, _, _ = select.select([s.fileno(), fd], [], [])
        if s.fileno() in r:
            d = s.read(s.in_waiting or 1)
            if d:
                os.write(sys.stdout.fileno(), d)
        if fd in r:
            k = os.read(fd, 64)
            if b'\x1d' in k:
                break
            s.write(k.replace(b'\n', b'\r') if not old else k)
finally:
    if old:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print()

#!/usr/bin/env python3
"""Measure the M9 boot loader's (sw/bootrom/yload.S) load time on the SoC sim.

    python3 sim/soc/loadtime.py [--work DIR] [--uart-kib 1,4,16] [--sd-kib 4,16,64,256]
                                [--spi-kib ...] [--fast-uart-kib 4,16] [--quick]

Milestone acceptance (M9): load time for a 1 MiB and a 4 MiB image, over the
UART and over the SPI flash, at the board clock of 50 MHz. Full-size loads are
too slow to simulate (UART: 10 bits x 432 clk = 4320 clk per byte), so each
path is run at a few frame sizes, fitted to cycles = a + b * payload_bytes,
checked for linearity, and extrapolated to 1 MiB, 4 MiB and the Linux Image.

Paths:
  sd    +SDCARD (M9b, D42): frame at sector 2048 of the SD card model, the
        loader's own init + CLKDIV. Interval = reset -> 'K' written to the
        16550 THR, i.e. it includes the card init at 390 kHz. --sdsc for a
        standard-capacity (byte-addressed) card.
  spi   (pre-M9b loaders only: the current loader no longer reads the flash;
        default off) +SPIFLASH: frame at flash offset 2 MiB (spi_flash_model), the loader's
        own CLKDIV. Interval = reset -> 'K' written to the 16550 THR.
  uart  the board's UART rate (host drives 434 clk/bit = 115200 baud). The loader never programs the 16550 divisor, so
        the board runs at the reset DL = 50e6 / (16 * 115200) = 27, i.e.
        432 clk/bit (115 741 baud). This needs a sim binary built with
        -GUART_BAUD=115200 (obj_soc's default is DL = 1); the script builds one
        into <work>/obj_board from the same Verilator command `make` uses.
        The host is unpaced (UART_RX_PACED=0, what a PC does) and starts at
        the cycle the loader prints its 'Y' prompt (found by a short probe
        run). Interval = first start bit -> 'K'.
  uart-fast  diagnostic only: obj_soc's DL = 1 (160 clk/byte on the line) with
        pacing ON, so the transfer is limited by the loader's own per-byte
        cost. Its slope is the CPU cost per UART byte, i.e. the headroom
        against the 4320 clk/byte line.

The payload is random (fixed seed); its first word is `j .` so the jump to the
entry point parks the core and the simulation stops by itself after 'K'.
"""
import argparse, os, random, re, subprocess, sys, tempfile, zlib

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
FAST_BIN = os.environ.get('SOC_BIN', os.path.join(HERE, 'obj_soc', 'Vsim_top'))
LOADER = os.environ.get('LOADER', os.path.join(SIM, 'sw', 'bootrom', 'build', 'bootrom.hex'))
FLASH_OFF = 0x200000
SD_OFF = 2048 * 512
CLK = 50e6
LOAD_ADDR, ENTRY = 0x0, 0x80000000   # SDRAM 0, entered through the 0x8000_0000 alias
# Host side of the line: a PC sends at exactly 115200 baud = 434.03 clk/bit at
# 50 MHz (the board's own DL = 27 receiver runs at 432, +0.47%, well inside
# 8N1 tolerance). The host sets the pace, so the sim drives 434.
BOARD_BIT = 434
LINUX_IMAGE = os.path.expanduser('~/yarv-linux/soc/Image')
TARGETS = [('1 MiB', 1 << 20), ('4 MiB', 4 << 20), ('Linux Image', 1835244)]


def frame(n, seed):
    rnd = random.Random(seed)
    payload = bytearray(rnd.getrandbits(8) for _ in range(n))
    payload[0:4] = (0x0000006f).to_bytes(4, 'little')    # j .
    u32 = lambda v: v.to_bytes(4, 'little')
    return (b'YL' + u32(LOAD_ADDR) + u32(n) + u32(ENTRY) + bytes(payload) +
            u32(zlib.crc32(payload) & 0xFFFFFFFF))


def escape_min(b):
    """sim_main's UART_RX unescape(): only NUL and backslash need escaping,
    everything else goes through the environment raw (bytes)."""
    return b.replace(b'\\', b'\\\\').replace(b'\0', b'\\0')


def events(work):
    out = []
    try:
        for ln in open(os.path.join(work, 'sim_events.txt')):
            p = ln.split()
            if len(p) == 3 and p[0] == 'U':
                out.append((int(p[1]), chr(int(p[2], 16))))
    except FileNotFoundError:
        pass
    return out


def run(binary, work, args, env_extra):
    os.makedirs(work, exist_ok=True)
    env = {k.encode(): v.encode() for k, v in os.environ.items()}
    env.update({b'NO_VCD': b'1', b'INSTR_LOG': b'0'})
    env.update({k.encode(): (v if isinstance(v, bytes) else str(v).encode())
                for k, v in env_extra.items()})
    with open(os.path.join(work, 'sim.log'), 'wb') as log:
        subprocess.run([binary, '+NOPROBE', f'+BINIT={LOADER}'] + args, cwd=work, env=env,
                       stdout=log, stderr=subprocess.STDOUT, check=False)
    return events(work)


def k_cycle(ev, what):
    txt = ''.join(c for _, c in ev)
    if 'E' in txt or 'K' not in txt:
        sys.exit(f'{what}: loader did not answer K (tx={txt!r})')
    return next(c for c, ch in ev if ch == 'K')


def run_spi(work, n):
    fr = frame(n, n)
    fr += b'\xff' * (-len(fr) % 4)
    hexf = os.path.join(work, 'flash.hex')
    os.makedirs(work, exist_ok=True)
    with open(hexf, 'w') as f:
        f.write(f'@{FLASH_OFF // 4:08x}\n')
        for i in range(0, len(fr), 4):
            f.write(f"{int.from_bytes(fr[i:i+4], 'little'):08x}\n")
    ev = run(FAST_BIN, work, ['+SPIFLASH', f'+FINIT={hexf}'],
             {'MAX_CYC': 400 * n + 2_000_000})
    txt = ''.join(c for _, c in ev)
    if txt.startswith('Y'):
        sys.exit(f'spi {n}: loader fell back to the UART (tx={txt!r})')
    return k_cycle(ev, f'spi {n}')


def run_sd(work, n, sdsc):
    fr = frame(n, n)
    fr += b'\xff' * (-len(fr) % 4)
    hexf = os.path.join(work, 'sd.hex')
    os.makedirs(work, exist_ok=True)
    with open(hexf, 'w') as f:
        f.write(f'@{SD_OFF // 4:08x}\n')
        for i in range(0, len(fr), 4):
            f.write(f"{int.from_bytes(fr[i:i+4], 'little'):08x}\n")
    ev = run(FAST_BIN, work, ['+SDCARD', f'+SDINIT={hexf}'] + (['+SDSC'] if sdsc else []),
             {'MAX_CYC': 400 * n + 2_000_000})
    txt = ''.join(c for _, c in ev)
    if txt.startswith('Y'):
        sys.exit(f'sd {n}: loader fell back to the UART (tx={txt!r})')
    return k_cycle(ev, f'sd {n}')


def y_cycle(binary, work):
    ev = run(binary, work, [], {'MAX_CYC': 300_000})
    ys = [c for c, ch in ev if ch == 'Y']
    if not ys:
        sys.exit(f'probe: no Y prompt from the loader ({binary})')
    return ys[0]


def run_uart(binary, work, n, start, bit, paced):
    fr = frame(n, n)
    ev = run(binary, work, [], {'MAX_CYC': (bit * 10 + 200) * (n + 30) + 2_000_000,
                                'UART_RX': escape_min(fr), 'UART_BIT_CYCLES': bit,
                                'UART_RX_START': start, 'UART_RX_PACED': int(paced)})
    return k_cycle(ev, f'uart {n}') - start


def build_board(work):
    binary = os.path.join(work, 'obj_board', 'Vsim_top')
    if os.path.exists(binary):
        return binary
    os.makedirs(work, exist_ok=True)
    cmd = subprocess.run(['make', '-n', 'obj_soc/Vsim_top', '-B'], cwd=HERE, capture_output=True,
                         text=True).stdout
    vl = next(l for l in cmd.splitlines() if l.startswith('verilator'))
    vl = vl.replace('-Mdir obj_soc', f'-Mdir {os.path.join(work, "obj_board")} '
                    '-GUART_BAUD=115200 -j 8')
    print('building the board-baud sim (DL = 27) ...', flush=True)
    subprocess.run(vl, shell=True, cwd=HERE, check=True, stdout=subprocess.DEVNULL)
    return binary


def fit(pts):
    n = len(pts)
    mx = sum(x for x, _ in pts) / n
    my = sum(y for _, y in pts) / n
    b = sum((x - mx) * (y - my) for x, y in pts) / sum((x - mx) ** 2 for x, _ in pts)
    a = my - b * mx
    worst = max(abs(y - (a + b * x)) for x, y in pts)
    return a, b, worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--work', default=os.path.join(tempfile.gettempdir(), 'yarv_loadtime'))
    ap.add_argument('--uart-kib', default='1,4,16,32')
    ap.add_argument('--sd-kib', default='4,16,64,256')
    ap.add_argument('--sdsc', action='store_true')
    ap.add_argument('--spi-kib', default='')
    ap.add_argument('--fast-uart-kib', default='4,16')
    ap.add_argument('--jobs', type=int, default=os.cpu_count())
    ap.add_argument('--quick', action='store_true', help='smallest sizes only')
    a = ap.parse_args()
    if a.quick:
        a.uart_kib, a.sd_kib, a.fast_uart_kib = '1,2', '4,16', '1,4'
    work = os.path.abspath(a.work)
    kib = lambda s: [int(float(x) * 1024) for x in s.split(',') if x]

    from concurrent.futures import ThreadPoolExecutor
    board = build_board(work)
    yb = y_cycle(board, os.path.join(work, 'probe_board'))
    yf = y_cycle(FAST_BIN, os.path.join(work, 'probe_fast'))
    print(f'  uart: loader prompts Y at cycle {yb}; the host starts there', flush=True)
    jobs = [('sd', n, lambda n=n: run_sd(os.path.join(work, f'sd_{n}'), n, a.sdsc))
            for n in kib(a.sd_kib)]
    jobs += [('spi', n, lambda n=n: run_spi(os.path.join(work, f'spi_{n}'), n))
            for n in kib(a.spi_kib)]
    jobs += [('uart', n, lambda n=n: run_uart(board, os.path.join(work, f'uart_{n}'), n, yb,
                                              BOARD_BIT, False)) for n in kib(a.uart_kib)]
    jobs += [('uart-fast', n, lambda n=n: run_uart(FAST_BIN, os.path.join(work, f'uartfast_{n}'),
                                                   n, yf, 16, True)) for n in kib(a.fast_uart_kib)]
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [(p, n, ex.submit(f)) for p, n, f in jobs]
        res = {}
        for p, n, fu in futs:
            res.setdefault(p, []).append((n, fu.result()))
            print(f'  {p:9s} {n:8d} B  {res[p][-1][1]:12d} cyc', flush=True)

    print(f'\n{"path":10s} {"payload B":>11s} {"cycles":>13s} {"s @50MHz":>10s}  source')
    fits = {}
    for path, pts in res.items():
        for n, c in pts:
            print(f'{path:10s} {n:11d} {c:13d} {c / CLK:10.4f}  measured')
        if len(pts) >= 2:
            fits[path] = fit(pts)
        if path == 'uart-fast':
            continue
        if path in fits:
            fa, fb, _ = fits[path]
            for name, n in TARGETS:
                c = fa + fb * n
                print(f'{path:10s} {n:11d} {c:13.0f} {c / CLK:10.4f}  extrapolated ({name})')
    print()
    for path, (fa, fb, worst) in fits.items():
        print(f'fit {path:9s}: cycles = {fa:.0f} + {fb:.4f} * bytes   '
              f'(worst residual {worst:.0f} cyc, {fb / CLK * 1e6:.3f} us/byte, '
              f'{CLK / fb / 1024:.1f} KiB/s)')
    if 'uart' in fits:
        print(f'uart line floor: 10 bits x {BOARD_BIT} = {10 * BOARD_BIT} clk/byte; the sim '
              f'harness adds a fixed 10-clk idle gap per byte (sim_main GAP_CYCLES), so a '
              f'back-to-back host is {fits["uart"][1] - 10:.1f} clk/byte')
    if 'uart-fast' in fits:
        print(f'loader CPU cost per UART byte (paced, DL=1): {fits["uart-fast"][1]:.1f} clk '
              f'-> headroom x{10 * BOARD_BIT / fits["uart-fast"][1]:.1f} vs the board line')


if __name__ == '__main__':
    main()

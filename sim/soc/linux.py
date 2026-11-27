#!/usr/bin/env python3
"""Boot the nommu Linux Image on the SoC sim to the BusyBox shell (M10).

    linux.py <Image> [workdir] [--flash | --sd]

Converts the raw Image to 32-bit word hex at SDRAM word 0 (the kernel's load
and entry address 0x8000_0000 aliases SDRAM 0), boots it through
bootrom_linux.hex (a0 = a1 = 0, jump to 0x8000_0000), types COMMANDS into
the 16550 once the shell is listening, and stops the simulation as soon as
the shell has echoed BOOT-OK.

PASS = a console line that is exactly "BOOT-OK" (the shell's output, not the
echo of the typed `echo BOOT-OK`). Reported from sim_events.txt, which
sim_soc.sv writes with a cycle stamp per TX byte and per misaligned trap:
  - cycles to the "Run /bin/sh as init process" line;
  - cycles to the first shell prompt;
  - how many misaligned-access traps the kernel took (its emulation path).
The console is left in <workdir>/sim_uart_tx.txt.

--flash: boot the way the board does. The boot ROM is the M9 loader
(sw/bootrom/build/bootrom.hex), the Image goes into a yload frame
(sw/bootrom/yframe.py, entry 0x8000_0000) placed at offset 2 MiB of the SPI
flash model (+SPIFLASH +FINIT), and the loader copies it into SDRAM, fence.i,
and jumps with the boot-protocol registers it sets. Covers loader -> kernel,
which bootrom_linux.hex (a0 = a1 = 0, a bare jump) does not. NOTE: since M9b
(D42) the loader no longer reads the flash -- this path now falls back to the
UART and never boots; kept only for an older loader ROM.

--sd: the board's way since M9b (D42). Same as --flash, but the frame is at
sector 2048 (byte offset 1 MiB) of the SPI-mode SD card model (+SDCARD
+SDINIT). SD_ARGS="+SDSC ..." passes extra plusargs to the sim (standard-
capacity card, per-command log, ...).
"""
import os, re, signal, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.environ.get('SOC_BIN', os.path.join(HERE, 'obj_soc', 'Vsim_top'))
ROM = os.path.join(HERE, 'bootrom_linux.hex')
LOADER = os.path.join(HERE, '..', 'sw', 'bootrom', 'build', 'bootrom.hex')
YFRAME = os.path.join(HERE, '..', 'sw', 'bootrom', 'yframe.py')
FLASH_OFF = 0x200000  # D19
SD_OFF = 2048 * 512  # D42: sector 2048
COMMANDS = ('mount -t proc proc /proc\n'
            'cat /proc/meminfo\n'
            'free\n'
            'uname -a\n'
            'ls /\n'
            'echo BOOT-OK\n')
MAX_CYC = int(os.environ.get('MAX_CYC', '200000000'))
RX_START = int(os.environ.get('UART_RX_START', '60000000'))
TIMEOUT_S = int(os.environ.get('LINUX_TIMEOUT_S', '3600'))


def image_to_hex(img, out):
    b = open(img, 'rb').read()
    b += b'\0' * (-len(b) % 4)
    with open(out, 'w') as f:
        f.write('@00000000\n')
        for i in range(0, len(b), 4):
            f.write(f"{int.from_bytes(b[i:i+4], 'little'):08x}\n")
    return len(b)


def read_events(path):
    """-> (text, per-char cycle list, misaligned trap list)."""
    text, cyc, mis = [], [], []
    try:
        for ln in open(path):
            p = ln.split()
            if len(p) == 3 and p[0] == 'U':
                text.append(chr(int(p[2], 16)))
                cyc.append(int(p[1]))
            elif len(p) == 3 and p[0] == 'M':
                mis.append((int(p[1]), p[2]))
    except FileNotFoundError:
        pass
    return ''.join(text), cyc, mis


def flash_hex(sdram_hex, out, off=FLASH_OFF):
    frame = subprocess.run(['python3', YFRAME, sdram_hex, '--entry', '0x80000000'],
                           capture_output=True, check=True).stdout
    frame += b'\xff' * (-len(frame) % 4)
    with open(out, 'w') as f:
        f.write(f'@{off // 4:08x}\n')
        for i in range(0, len(frame), 4):
            f.write(f"{int.from_bytes(frame[i:i+4], 'little'):08x}\n")
    return len(frame)


def main():
    args = [a for a in sys.argv[1:] if a not in ('--flash', '--sd')]
    flash = '--flash' in sys.argv[1:]
    sd = '--sd' in sys.argv[1:]
    image = args[0]
    work = os.path.abspath(args[1] if len(args) > 1 else os.path.join(HERE, 'linux_run'))
    os.makedirs(work, exist_ok=True)
    hexf = os.path.join(work, 'linux.hex')
    nbytes = image_to_hex(image, hexf)
    print(f'image {image}: {nbytes} bytes -> {hexf}')
    env = dict(os.environ, NO_VCD='1', INSTR_LOG='0', MAX_CYC=str(MAX_CYC),
               UART_BIT_CYCLES='432', UART_RX_START=str(RX_START), UART_RX=COMMANDS)
    if flash or sd:
        # The loader's copy time comes first and is not a constant: type only
        # once the shell has printed its prompt.
        env['UART_RX_WAIT'] = '# '
        env['UART_RX_START'] = os.environ.get('UART_RX_START', '100000')
    evt = os.path.join(work, 'sim_events.txt')
    t0 = time.time()
    if flash:
        fhex = os.path.join(work, 'flash.hex')
        print(f'flash frame: {flash_hex(hexf, fhex)} bytes at 0x{FLASH_OFF:x} -> {fhex}')
        cmd = [BIN, '+NOPROBE', '+SPIFLASH', f'+FINIT={fhex}', f'+BINIT={os.path.abspath(LOADER)}']
    elif sd:
        shex = os.path.join(work, 'sdcard.hex')
        print(f'sd frame: {flash_hex(hexf, shex, SD_OFF)} bytes at sector {SD_OFF // 512} -> {shex}')
        cmd = [BIN, '+NOPROBE', '+SDCARD', f'+SDINIT={shex}', f'+BINIT={os.path.abspath(LOADER)}']
        cmd += os.environ.get('SD_ARGS', '').split()
    else:
        cmd = [BIN, '+NOPROBE', f'+SINIT={hexf}', f'+BINIT={ROM}']
    p = subprocess.Popen(cmd, cwd=work, env=env,
                         stdout=open(os.path.join(work, 'sim.log'), 'w'), stderr=subprocess.STDOUT)
    ok = False
    while p.poll() is None:
        time.sleep(5)
        text, _, _ = read_events(evt)
        if re.search(r'^BOOT-OK\r?$', text, re.M):
            ok = True
            time.sleep(1)  # let the prompt after it flush
            p.send_signal(signal.SIGTERM)
            break
        if time.time() - t0 > TIMEOUT_S:
            p.send_signal(signal.SIGTERM)
            break
    p.wait()
    wall = time.time() - t0
    text, cyc, mis = read_events(evt)
    ok = ok or bool(re.search(r'^BOOT-OK\r?$', text, re.M))

    def fmt0(c):
        return f'{c} cycles ({c / 50e6:.3f} s at 50 MHz)' if c is not None else 'not reached'

    def cyc_at(idx):
        return cyc[idx] if 0 <= idx < len(cyc) else None

    i_k = text.find('K') if (flash or sd) else -1
    if i_k >= 0:
        print(f"loader 'K' : {fmt0(cyc_at(i_k))}")
    i_sh = text.find('Run /bin/sh')
    c_sh = cyc_at(text.rfind('\n', 0, i_sh) + 1) if i_sh >= 0 else None
    m = re.compile(r'[#$] $', re.M).search(text, i_sh if i_sh >= 0 else 0)
    c_prompt = cyc_at(m.end() - 1) if m else None
    fmt = lambda c: f'{c} cycles ({c / 50e6:.3f} s at 50 MHz)' if c is not None else 'not reached'
    print(f'Run /bin/sh : {fmt(c_sh)}')
    print(f'first prompt: {fmt(c_prompt)}')
    print(f'misaligned-access traps: {len(mis)}'
          + (f' (first at cycle {mis[0][0]}, mtval 0x{mis[0][1]})' if mis else ''))
    print(f'console: {os.path.join(work, "sim_uart_tx.txt")}   wall {wall:.0f} s')
    print('PASS' if ok else 'FAIL: no "BOOT-OK" line from the shell')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Run the BSRAM regress oracle set on the Linux SoC sim (M5).

Reads REGRESS_TESTS from ../Makefile, merges each Harvard image pair into one
SDRAM image (mkimage.py), boots it through the bootrom stub and checks the
same marker the BSRAM regress checks. MAX_CYC is scaled (cache misses +
the 200 us SDRAM power-up wait). Exit status is non-zero on any failure.

Skipped, with the reason printed:
  ifault  -- it jumps to 0x100000 expecting an instruction access fault; on
             the SoC that address is SDRAM (fetch faults only at >= 16 MiB).
"""
import os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
BIN = os.path.join(HERE, 'obj_soc', 'Vsim_top')
SKIP = {'ifault': 'fault boundary is 16 MiB on the SoC, the test expects 16 KiB'}
MULT = int(os.environ.get('SOC_CYC_MULT', '8'))
EXTRA = 20000  # SDRAM power-up wait (200 us at 50 MHz = 10000) + margin

mk = open(os.path.join(SIM, 'Makefile')).read()
block = mk[mk.index('REGRESS_TESTS :='):]
block = block[:block.index('\n\n')]
tests = re.findall(r'^\s*([\w]+):([\w/.-]+):(\d+):(\S+?)\s*\\?$', block, re.M)

os.makedirs(os.path.join(HERE, 'img'), exist_ok=True)
npass = nfail = nskip = 0
for name, d, maxc, check in tests:
    if name in SKIP:
        print(f'{name:15s} SKIP   {SKIP[name]}'); nskip += 1; continue
    imem = os.path.join(SIM, 'sw', d, 'imem.hex')
    dmem = os.path.join(SIM, 'sw', d, 'dmem.hex')
    img = os.path.join(HERE, 'img', name + '.hex')
    r = subprocess.run(['python3', os.path.join(HERE, 'mkimage.py'), imem, dmem, img],
                       capture_output=True, text=True)
    if r.returncode:
        print(f'{name:15s} FAIL   image: {r.stderr.strip()}'); nfail += 1; continue
    env = dict(os.environ, NO_VCD='1', MAX_CYC=str(int(maxc) * MULT + EXTRA),
               RTL_TRACE=os.path.join(HERE, 'img', name + '.trace'))
    want = None; probe = None; reg = None
    if check.startswith('probe@'):
        probe, want = check[6:].split('=')
        env['PROBE'] = probe
    elif check.startswith('trace:'):
        reg, want = check[6:].split('=')
    out = subprocess.run([BIN, f'+SINIT={img}', f'+BINIT={os.path.join(HERE, "bootrom_jump0.hex")}'],
                         cwd=SIM, env=env, capture_output=True, text=True, errors='replace').stdout
    cyc = re.search(r'^retired \d+ instructions in (\d+) cycles', out, re.M)
    cyc = cyc.group(1) if cyc else '?'
    bad = []
    if 'WFI-HALT FAIL' in out: bad.append('WFI-HALT FAIL')
    if 'parked at' not in out: bad.append('no clean park')
    if 'SDRAM MODEL ERROR' in out: bad.append('SDRAM protocol error')
    got = None
    if probe:
        m = re.findall(r'^probe\[.*\] = 0x0*([0-9a-fA-F]*)$', out, re.M)
        got = m[-1] if m else None
    elif reg:
        tr = open(env['RTL_TRACE']).read()
        m = re.findall(rf'^0x[0-9a-f]* {reg} 0x0*([0-9a-f]*)$', tr, re.M)
        got = m[-1] if m else None
    if want and (got or '').lower() != want.lower():
        bad.append(f'marker=0x{got or "none"} want=0x{want}')
    if bad:
        print(f'{name:15s} FAIL   {cyc} cyc -- {"; ".join(bad)}'); nfail += 1
    else:
        print(f'{name:15s} PASS   {cyc} cyc'); npass += 1
print(f'\n{npass} passed, {nfail} failed, {nskip} skipped')
sys.exit(1 if nfail else 0)

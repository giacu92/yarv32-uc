#!/usr/bin/env python3
"""riscv-tests compliance on the Linux SoC sim (cache + SDRAM, unified space).

Each sw/compliance/build/<test>/ Harvard pair is merged into one SDRAM image
(mkimage.py) and booted through bootrom_jump0.hex; the verdict is tohost at
0x2000, read with PROBE like the BSRAM `make compliance`. Expected failures
differ from the BSRAM list: the SoC is one coherent address space with a real
fence.i, so rv32ui-fence_i and rv32uc-rvc (case 6 loads from .text) pass here.
An unexpected pass (XPASS) fails the run, as in the BSRAM target.
"""
import os, re, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
BIN = os.environ.get('SOC_BIN', os.path.join(HERE, 'obj_soc', 'Vsim_top'))
BUILD = os.path.join(SIM, 'sw', 'compliance', 'build')
XFAIL = {
    'rv32ui-ma_data': '539',  # misaligned load/store traps; the test has no handler
    'rv32mi-pmpaddr': '3',    # no PMP
}

tests = sorted(d for d in os.listdir(BUILD) if os.path.isdir(os.path.join(BUILD, d)))
if not tests:
    sys.exit('no test images -- make -C sw/compliance RISCV_PREFIX=<prefix> first')
npass = nx = nfail = 0
tmp = tempfile.mkdtemp(prefix='soc_compliance_')
print(f"{'TEST':26s} {'RESULT':7s} DETAIL")
for t in tests:
    d = os.path.join(BUILD, t)
    img = os.path.join(tmp, t + '.hex')
    r = subprocess.run(['python3', os.path.join(HERE, 'mkimage.py'),
                        os.path.join(d, 'imem.hex'), os.path.join(d, 'dmem.hex'), img],
                       capture_output=True, text=True)
    if r.returncode:
        print(f'{t:26s} FAIL    image: {r.stderr.strip()}'); nfail += 1; continue
    env = dict(os.environ, NO_VCD='1', INSTR_LOG='0', PROBE='0x2000', MAX_CYC='2000000')
    out = subprocess.run([BIN, f'+SINIT={img}', f'+BINIT={os.path.join(HERE, "bootrom_jump0.hex")}'],
                         cwd=SIM, env=env, capture_output=True, text=True, errors='replace').stdout
    cyc = re.search(r'^retired \d+ instructions in (\d+) cycles', out, re.M)
    cyc = cyc.group(1) if cyc else '?'
    m = re.findall(r'^probe\[0x00002000\] = 0x0*([0-9a-fA-F]*)$', out, re.M)
    got = (m[-1] if m else '') or '0'
    want = XFAIL.get(t)
    if 'parked at' not in out or 'SDRAM MODEL ERROR' in out:
        print(f'{t:26s} FAIL    no clean park / SDRAM error'); nfail += 1
    elif got == '1' and want is None:
        print(f'{t:26s} PASS    {cyc} cyc'); npass += 1
    elif got == '1':
        print(f'{t:26s} XPASS   expected tohost=0x{want} -- update XFAIL'); nfail += 1
    elif want is not None and got.lower() == want.lower():
        print(f'{t:26s} XFAIL   tohost=0x{got}'); nx += 1
    else:
        print(f'{t:26s} FAIL    tohost=0x{got}'); nfail += 1
print(f'\n{npass} passed, {nx} expected failures, {nfail} failed')
sys.exit(1 if nfail else 0)

#!/usr/bin/env python3
"""Make the cache sources coexist with rv32_pkg in one Verilator $unit.

  qualify_pkg.py <package.sv> <source.sv>...

Both projects wildcard-import their package at FILE scope, and Verilator puts
every file in one compilation unit, so the cache's `mem_req_t` resolves to
rv32_pkg's. This rewrites the given sources IN PLACE (only ever run on the
build snapshot, never on the submodule): the file-scope
`import yarv32_cache_pkg::*;` is removed and every name the package declares
is prefixed with `yarv32_cache_pkg::`.
"""
import re, sys

pkg, srcs = sys.argv[1], sys.argv[2:]
ptxt = open(pkg).read()
body = ptxt[ptxt.index('package yarv32_cache_pkg'):]
names = set(re.findall(r'localparam\s+(?:[\w\[\]: -]*?\s)?(\w+)\s*=', body))
names |= set(re.findall(r'function\s+automatic\s+(?:logic\s*(?:\[[^\]]*\])?\s*)?(\w+)\s*\(', body))
names |= set(re.findall(r'`YARV_MEM_TYPES\((\w+),\s*(\w+)', body)[0])
for a, b in re.findall(r'`YARV_MEM_TYPES\((\w+),\s*(\w+)', body):
    names |= {a, b}
names.discard('')
pat = re.compile(r'(?<![\w.:$`])(' + '|'.join(sorted(names, key=len, reverse=True)) + r')\b')
for s in srcs:
    t = open(s).read()
    t = re.sub(r'^\s*import\s+yarv32_cache_pkg::\*;\s*$', '', t, flags=re.M)
    t = pat.sub(r'yarv32_cache_pkg::\1', t)
    open(s, 'w').write(t)
print('qualified', len(names), 'names:', ' '.join(sorted(names)))

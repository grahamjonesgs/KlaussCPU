#!/usr/bin/env python3
"""Convert a klausscc .txt.code listing into the $readmemh dword format the
testbenches consume (same layout as klausscc --mem-out on an ELF: one 64-bit
dword per line from address 0; first listed 32-bit word = low half).
usage: code2mem.py <in.txt.code> <out.mem>"""
import re
import sys

words = {}   # 32-bit word address -> value
for line in open(sys.argv[1]):
    m = re.match(r"0x([0-9A-Fa-f]{8}):\s+([0-9A-Fa-f ]+?)\s+--", line)
    if not m:
        continue
    addr = int(m.group(1), 16)
    toks = m.group(2).split()
    if len(toks) == 1 and len(toks[0]) == 16:      # 64-bit header dword
        v = int(toks[0], 16)
        words[addr >> 2] = v & 0xFFFFFFFF
        words[(addr >> 2) + 1] = v >> 32
        continue
    for i, t in enumerate(toks):
        assert len(t) == 8, (line, t)
        words[(addr >> 2) + i] = int(t, 16)

top = max(words) if words else 0
with open(sys.argv[2], "w") as f:
    for dw in range(0, (top // 2) + 1):
        lo = words.get(dw * 2, 0)
        hi = words.get(dw * 2 + 1, 0)
        f.write(f"{(hi << 32) | lo:016X}\n")
print(f"{sys.argv[2]}: {(top // 2) + 1} dwords")

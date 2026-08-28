#!/usr/bin/env python3
"""Fill core1_p2btest.kla.tmpl placeholders with the actual image blocks:
  @@WINDOW_LOAD_LOCAL@@ -> mailbox window-load of loop_local.mem
  @@DDR_STORE_LOOP@@    -> cached stores of loop_ddr.mem to 0x07E00000
usage: fill_p2b.py"""
DDR_BASE = 0x07E00000


def emit64(L, reg, tmp, val):
    hi, lo = val >> 32, val & 0xFFFFFFFF
    L.append("SETR %s 0x%X" % (reg, hi))
    L.append("SHLV %s 32" % reg)
    L.append("SETR %s 0x%X" % (tmp, lo))
    L.append("SHLV %s 32" % tmp)
    L.append("SHRV %s 32" % tmp)
    L.append("ORR %s %s %s" % (reg, reg, tmp))


local = [int(l, 16) for l in open("loop_local.mem") if l.strip()]
ddr   = [int(l, 16) for l in open("loop_ddr.mem") if l.strip()]

win = []
win.append("SETR G 0xF0100010")
win.append("SETR A 0")
win.append("MEMSET64 A G")
win.append("SETR G 0xF0100018")
for dw in local:
    emit64(win, "A", "B", dw)
    win.append("MEMSET64 A G")

sto = []
sto.append("SETR G 0x%X" % DDR_BASE)
for dw in ddr:
    emit64(sto, "A", "B", dw)
    sto.append("MEMSET64 A G")
    sto.append("ADDV G 8")

out = []
for line in open("core1_p2btest.kla.tmpl"):
    s = line.rstrip("\n")
    if s == "@@WINDOW_LOAD_LOCAL@@":
        out.extend(win)
    elif s == "@@DDR_STORE_LOOP@@":
        out.extend(sto)
    else:
        out.append(s)
open("core1_p2btest.kla", "w").write("\n".join(out) + "\n")
print("core1_p2btest.kla: %d lines (%d local dw, %d ddr dw)"
      % (len(out), len(local), len(ddr)))

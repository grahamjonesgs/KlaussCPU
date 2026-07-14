#!/usr/bin/env python3
# decode_wedge.py — pretty-print the bus-wedge flight recorder dump
# (two hex lines from wedge_dump.kla). Bit map: r_wedge_* decl in KlaussCPU.sv.
import sys

lines = [l.strip() for l in sys.stdin if l.strip() and all(c in '0123456789abcdefABCDEF' for c in l.strip())]
if len(lines) < 2:
    sys.exit("need the two snapshot hex lines on stdin")
s0, s1 = int(lines[0], 16), int(lines[1], 16)

def b(v, i): return (v >> i) & 1

print(f"snap0 = {s0:016x}   snap1 = {s1:016x}")
if not b(s0, 63):
    print("NO WEDGE CAPTURED (valid bit clear)")
    sys.exit(0)
print(f"  stuck addr        = 0x{s0 & 0xFFFFFFFF:08x}")
print(f"  [39:32] reserved  = {(s0 >> 32) & 0xFF}  (was st.SM; dropped for timing — see w_pipe_owns)")
print(f"  cpu read_DV       = {b(s0,40)}   write_DV = {b(s0,41)}   cpu.ready = {b(s0,42)}")
print(f"  w_mmio_read_DV    = {b(s0,43)}   r_mmio_read_dv_d = {b(s0,44)}   w_mmio_ready = {b(s0,45)}")
print(f"  w_eth_ready       = {b(s0,46)}   dram.ready = {b(s0,47)}   w_pipe_owns = {b(s0,48)}")
print(f"  r_timer_interrupt = {b(s0,49)}   w_irq_ready = {b(s0,50)}")
print(f"  mem_busy          = {b(s0,51)}   if_miss = {b(s0,52)}   pip_bus_idle = {b(s0,53)}")
print(f"  int_mask          = 0x{(s1 >> 32) & 0xF:x}   irq_src0 = {b(s1,36)}   irq_src1 = {b(s1,37)}")
print(f"  timer_count[25:0] = {(s1 >> 38) & 0x3FFFFFF}")
print(f"  cycles at latch   = {s1 & 0xFFFFFFFF}")

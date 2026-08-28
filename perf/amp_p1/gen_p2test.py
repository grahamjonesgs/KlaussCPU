#!/usr/bin/env python3
"""Generate the AMP P2a board-gate programs:
  core1_p2test.kla — the full coherency gate: writes the hello image to DDR
    (cached stores) + FLUSH -> core 2 executes it from DDR; writes a data
    buffer + FLUSH -> core 2 (ddr_sum from local RAM) sums it and writes the
    result to DDR; core 1 INVALIDATEs, reads the result back, compares.
  ddr_spin.kla — infinite uncached DDR reads on core 2 (master-C pressure
    for the core-1 CPI non-intrusion probe).
  core1_spinload.kla — loads+starts ddr_spin on core 2, prints, HALTs
    (core 2 keeps running across subsequent core-1 program loads).
usage: gen_p2test.py <hello.mem> <ddr_sum.mem>"""
import sys

hello = [int(l, 16) for l in open(sys.argv[1]) if l.strip()]
dsum  = [int(l, 16) for l in open(sys.argv[2]) if l.strip()]

DATA = [0x0123456789ABCDEF, 0xFEDCBA9876543210, 0xDEADBEEFCAFEF00D,
        0x0F1E2D3C4B5A6978, 0x1111111122222222, 0xA5A5A5A55A5A5A5A,
        0x00000000FFFFFFFF, 0x700DFACE00C0FFEE]
SUM = sum(DATA) & 0xFFFFFFFFFFFFFFFF


def emit64(L, reg, tmp, val):
    """Build a 64-bit value in reg (uses tmp; SETR sign-extends, so
    zero-extend the low half explicitly)."""
    hi, lo = val >> 32, val & 0xFFFFFFFF
    L.append("SETR %s 0x%X" % (reg, hi))
    L.append("SHLV %s 32" % reg)
    L.append("SETR %s 0x%X" % (tmp, lo))
    L.append("SHLV %s 32" % tmp)
    L.append("SHRV %s 32" % tmp)
    L.append("ORR %s %s %s" % (reg, reg, tmp))


def flush(L, val):                      # 2 = FLUSH, 4 = INVALIDATE
    L.append("SETR B 0xF0050000")
    L.append("SETR A %d" % val)
    L.append("MEMSET64 A B")
    L.append("SETR B 0xF0050010")
    L.append("mnt%d_wait:" % flush.n)
    L.append("MEMREADRR C B")
    L.append("ANDV C 1")
    L.append("CMPRV C 0")
    L.append("JMPNE mnt%d_wait:" % flush.n)
    flush.n += 1


flush.n = 0


def drain(L, count, tag):
    L.append("SETR E %d" % count)
    L.append("SETR G 0xF0100020")
    L.append("%s_drain:" % tag)
    L.append("MEMREADRR D G")
    L.append("COPY A D")
    L.append("SHRV A 8")
    L.append("ANDV A 1")
    L.append("CMPRV A 1")
    L.append("JMPNE %s_drain:" % tag)
    L.append("COPY A D")
    L.append("ANDV A 0xFF")
    L.append("CALL putc:")
    L.append("SETR A 0")
    L.append("MEMSET64 A G")
    L.append("MINUSV E 1")
    L.append("JMPNZ %s_drain:" % tag)
    L.append("SETR G 0xF0100000")
    L.append("%s_park:" % tag)
    L.append("MEMREADRR D G")
    L.append("COPY A D")
    L.append("ANDV A 2")
    L.append("CMPRV A 2")
    L.append("JMPNE %s_park:" % tag)
    L.append("SETR A 0")
    L.append("MEMSET64 A G")            # RUN=0: stop core 2


def putstr(L, s):
    for ch in s:
        L.append("SETR A %d" % ord(ch))
        L.append("CALL putc:")


def putc_routine(L):
    L.append("")
    L.append("putc:")
    L.append("SETR B 0xF0010010")
    L.append("putc_wait:")
    L.append("MEMREADRR C B")
    L.append("ANDV C 1")
    L.append("CMPRV C 0")
    L.append("JMPNE putc_wait:")
    L.append("SETR B 0xF0010000")
    L.append("MEMSET8 A B")
    L.append("RET")


def window_load(L, image):
    L.append("SETR G 0xF0100010")
    L.append("SETR A 0")
    L.append("MEMSET64 A G")
    L.append("SETR G 0xF0100018")
    for dw in image:
        emit64(L, "A", "B", dw)
        L.append("MEMSET64 A G")


def start_core2(L, pc):
    L.append("SETR G 0xF0100008")
    L.append("SETR A 0x%X" % pc)
    L.append("MEMSET64 A G")
    L.append("SETR G 0xF0100000")
    L.append("SETR A 0")                 # RUN 0 first: a clean 0->1 edge even
    L.append("MEMSET64 A G")             # if core 2 was left parked with RUN=1
    L.append("SETR A 1")
    L.append("MEMSET64 A G")


# ---------------------------------------------------------------- p2test ---
L = []
L.append("// core1_p2test — GENERATED (gen_p2test.py): AMP P2a board gate.")
L.append("_start")
# A: hello image -> DDR 0x07E00000 via cached stores, then FLUSH.
L.append("SETR G 0x07E00000")
for dw in hello:
    emit64(L, "A", "B", dw)
    L.append("MEMSET64 A G")
    L.append("ADDV G 8")
flush(L, 2)
start_core2(L, 0x07E00020)
drain(L, 18, "hA")                      # 18 bytes of hello, forwarded to UART
# B: data buffer -> DDR 0x01000000 (cached stores), FLUSH.
L.append("SETR G 0x01000000")
for dw in DATA:
    emit64(L, "A", "B", dw)
    L.append("MEMSET64 A G")
    L.append("ADDV G 8")
flush(L, 2)
window_load(L, dsum)
start_core2(L, 0x20)
drain(L, 4, "hB")                       # "sum\n"
flush(L, 4)                             # INVALIDATE before reading the result
L.append("SETR G 0x01000100")
L.append("MEMREADRR D G")
emit64(L, "A", "B", SUM)
L.append("CMPRR D A")
L.append("JMPNE fail:")
putstr(L, "P2 PASS\n")
L.append("HALT")
L.append("fail:")
putstr(L, "P2 FAIL\n")
L.append("HALT")
putc_routine(L)
open("core1_p2test.kla", "w").write("\n".join(L) + "\n")

# ---------------------------------------------------------------- spin -----
open("ddr_spin.kla", "w").write("""// ddr_spin — GENERATED: infinite uncached DDR reads on core 2 (master-C
// pressure for the core-1 CPI non-intrusion probe).  Runs from local RAM.
_start
SETR B 0x01000000
spin:
MEMREADRR C B
JMP spin:
""")

print("generated core1_p2test.kla + ddr_spin.kla; expected sum %016X" % SUM)

# ------------------------------------------------- spinload (2nd pass) -----
# Run again with a third arg (the assembled ddr_spin.mem) to emit the loader
# that starts the spinner and exits, leaving core 2 running.
if len(sys.argv) > 3:
    spin = [int(l, 16) for l in open(sys.argv[3]) if l.strip()]
    L = []
    L.append("// core1_spinload — GENERATED: start ddr_spin on core 2, exit.")
    L.append("// Core 2 keeps running across later core-1 program loads.")
    L.append("_start")
    window_load(L, spin)
    start_core2(L, 0x20)
    putstr(L, "core2 spinning\n")
    L.append("HALT")
    putc_routine(L)
    open("core1_spinload.kla", "w").write("\n".join(L) + "\n")
    print("generated core1_spinload.kla (%d spin dwords)" % len(spin))

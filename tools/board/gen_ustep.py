#!/usr/bin/env python3
"""Generate ustep_user.h for ustep_test.c (MMU_PLAN.md §13 board test).

The user routines are assembled with tools/vsim/kasm.py at base 0 (every
branch, call and LEAPC is PC-relative, so they run wherever ustep_test.c
copies them: USER_BASE).  Each routine comes with its expected exception
log: {ECAUSE, EADDR, frame PC, frame >> 44}, where an EADDR / frame PC
flagged relative is an offset into the routine.

  A  stepped (SS = 1): straight code, a taken FBR, CALL / RET, SYSCALL, a
     user MMIO load (ACCESS_LOAD), a misaligned load, a JMP, HALT — one
     STEP per retired instruction, EADDR = its PC, frame = the next PC.
  B  watchpoints (SS = 0), configured by the user through SYSCALL 1 (r5-r8
     = WP0_ADDR, WP0_CTRL, WP1_ADDR, WP1_CTRL): 64-bit store, byte loads
     (hit / BE miss), PUSH, CALL (frame = target), RET (frame = return
     address), a spanning MEMGET32 hitting only its 2nd doubleword
     (EADDR = its start), both watchpoints on one access, R-only vs a store.
  C  step + watch (SS = 1): WATCH with SSTEP on the stepped store.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'vsim'))
from kasm import Asm, WP_R, WP_W, WP_U   # noqa: E402

DATA = 0x00620000          # (ustep_test.c: USER_DATA)
USP = 0x00630000           # (ustep_test.c: USER_SP)
HALT_W = 0x6C010000
STEP, WATCH, SSTEP = 14, 15, 1 << 12
SYSCALL, ACC_LD, MISAL_LD, PRIV = 4, 7, 5, 2


def routine_a():
    a = Asm(base=0)
    P = {}
    P['s1'] = a.setr(1, 5)
    P['add'] = a.alu('add', 2, 1, 1)
    P['fbr'] = a.fbr('eq', 1, 5, 'T', imm=True)
    a.nop()
    a.label('T')
    P['call'] = a.call('F')
    P['sys'] = a.syscall()
    P['s3'] = a.setr(3, 0xF0000000)
    P['ld'] = a.ld(64, 4, 3)
    P['s3b'] = a.setr(3, DATA + 1)
    P['ldm'] = a.ld(16, 4, 3)
    P['jmp'] = a.jmp('E')
    a.nop()
    a.label('F')
    P['inc'] = a.inc(1)
    P['ret'] = a.ret()
    a.label('E')
    hp = a.halt()
    s = lambda at, nx: (STEP | SSTEP, at, 1, nx, 3)
    exp = [s(P['s1'], P['add']), s(P['add'], P['fbr']), s(P['fbr'], a.labels['T']),
           s(P['call'], P['inc']), s(P['inc'], P['ret']), s(P['ret'], P['sys']),
           (SYSCALL, 0, 0, P['s3'], 3), s(P['s3'], P['ld']), (ACC_LD, 0xF0000000, 0, P['ld'], 3),
           s(P['s3b'], P['ldm']), (MISAL_LD, DATA + 1, 0, P['ldm'], 3), s(P['jmp'], hp),
           (PRIV, HALT_W, 0, hp, 3)]
    return a, exp


def routine_b():
    a = Asm(base=0)
    exp = []

    def cfg(wp0=(0, 0), wp1=(0, 0)):
        a.setr(5, wp0[0]); a.setr(6, wp0[1]); a.setr(7, wp1[0]); a.setr(8, wp1[1])
        a.syscall()

    W = lambda c, ea, at, rel=0: exp.append((WATCH | c, ea, rel, at, 1))
    d = DATA + 0x100
    a.setr(9, d); a.setr64(10, 0x1122334455667788)
    cfg((d, 0xFF | WP_W))
    at = a.st(64, 10, 9); W(1 << 9 | 1 << 11, d, at + 4)
    cfg((d, 0x04 | WP_R | WP_U))
    at = a.ld(8, 4, 9, 2); W(1 << 9, d + 2, at + 4)
    a.ld(8, 4, 9, 3)                                          # BE miss
    cfg((d, 0xFF | WP_R))
    a.st(64, 10, 9)                                           # R only: a store does not hit
    cfg((USP - 8, 0xFF | WP_W))
    at = a.push(10); W(1 << 9 | 1 << 11, USP - 8, at + 4)
    a.pop(10)
    cfg((USP - 8, 0xFF | WP_W))
    at = a.call('F'); exp.append(None)                       # (frame = F: patched below)
    cfg((USP - 8, 0xFF | WP_R))
    a.call('G')
    a.label('RG')
    a.setr(9, d + 6)
    cfg((d, 0x0F | WP_R), (d + 8, 0x01 | WP_R))
    at = a.ld(32, 4, 9); W(1 << 10, d + 8, at + 4)            # MEMGET32 [d+6]: bytes 6, 7 | 8, 9
    a.setr(9, d)
    cfg((d, 0x0F | WP_R), (d, 0x01 | WP_R))
    at = a.ld(8, 4, 9); W(3 << 9, d, at + 4)
    a.jmp('E')
    a.label('F'); a.ret()
    a.label('G'); g = a.ret()
    W(1 << 9, USP - 8, a.labels['RG'])
    a.label('E')
    hp = a.halt()
    i = exp.index(None)
    exp[i] = (WATCH | 1 << 9 | 1 << 11, USP - 8, 0, a.labels['F'], 1)
    # (RET's hit was appended after the CALL's: reorder by program order)
    rg = exp.pop()
    exp.insert(i + 1, rg)
    exp.append((PRIV, HALT_W, 0, hp, 1))
    return a, exp


def routine_c():
    a = Asm(base=0)
    P = {}
    d = DATA + 0x200
    P['s9'] = a.setr(9, d)
    P['st'] = a.st(64, 9, 9)
    P['ld'] = a.ld(64, 4, 9)
    hp = a.halt()
    s = lambda at, nx: (STEP | SSTEP, at, 1, nx, 3)
    exp = [s(P['s9'], P['st']), (WATCH | SSTEP | 1 << 9 | 1 << 11, d, 0, P['ld'], 3), s(P['ld'], hp),
           (PRIV, HALT_W, 0, hp, 3)]
    return a, exp, d


def emit(name, a, exp, f):
    ws = a.assemble()
    f.write(f'static const uint32_t {name}[{len(ws)}] = {{\n')
    for i in range(0, len(ws), 6):
        f.write('    ' + ', '.join(f'0x{w:08X}u' for w in ws[i:i + 6]) + ',\n')
    f.write('};\n')
    f.write(f'static const struct uexp {name}_EXP[{len(exp)}] = {{\n')
    for c, ea, rel, fp, hi in exp:
        f.write(f'    {{0x{c:04X}u, 0x{ea:08X}u, {rel}, 0x{fp:04X}u, {hi}}},\n')
    f.write('};\n')


def main():
    out = os.path.join(HERE, 'ustep_user.h')
    with open(out, 'w') as f:
        f.write('// generated by tools/board/gen_ustep.py — do not edit\n#include <stdint.h>\n')
        f.write('struct uexp { uint32_t cause; uint32_t eaddr; int eaddr_rel; uint32_t frame_off; uint32_t hi; };\n')
        f.write(f'#define USER_DATA 0x{DATA:08X}u\n#define USER_SP 0x{USP:08X}u\n')
        a, e = routine_a(); emit('USTEP_A', a, e, f)
        a, e = routine_b(); emit('USTEP_B', a, e, f)
        a, e, d = routine_c(); emit('USTEP_C', a, e, f)
        f.write(f'#define USTEP_C_WP 0x{d:08X}u\n')
    print(out)


if __name__ == '__main__':
    main()

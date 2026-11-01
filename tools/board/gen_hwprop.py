#!/usr/bin/env python3
"""Generate hwprop_snippets.h: position-independent instruction sequences
(encoded by tools/vsim/kasm.py) as inline-asm `.4byte` strings, for the
board tests in hwprop_test.c.  The toolchain does not know the new
encodings yet, so they are emitted as raw words."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'vsim'))
from kasm import Asm, CMP_PRED

M64 = 2**64 - 1
out = []


def words(fn):
    a = Asm(base=0x1000)
    fn(a)
    return a.assemble()


def asm_str(ws):
    return ' '.join(f'".4byte 0x{w:08X}\\n"' for w in ws) or '""'


def br(emit):
    """r3 = 2 if the branch emitted by emit(a, 'T') is taken, else 1"""
    def f(a):
        emit(a, 'T')
        a.setr(3, 1)
        a.jmp('E')
        a.label('T')
        a.setr(3, 2)
        a.label('E')
    return words(f)


def s64(v):
    v &= M64
    return v - 2**64 if v >> 63 else v


def s32(v):
    v &= 0xFFFFFFFF
    return v - 2**32 if v >> 31 else v


def cmp(op, a, b):
    sa, sb, ua, ub = s64(a), s64(b), a & M64, b & M64
    return {'eq': ua == ub, 'ne': ua != ub, 'lt': sa < sb, 'ge': sa >= sb, 'le': sa <= sb,
            'gt': sa > sb, 'ult': ua < ub, 'uge': ua >= ub, 'ule': ua <= ub, 'ugt': ua > ub}[op]


W_PRED = {'eqw': (5, 0, lambda a, b: a & 0xFFFFFFFF == b & 0xFFFFFFFF),
          'new': (5, 1, lambda a, b: a & 0xFFFFFFFF != b & 0xFFFFFFFF),
          'ltw': (6, 0, lambda a, b: s32(a) < s32(b)), 'gew': (6, 1, lambda a, b: s32(a) >= s32(b)),
          'ultw': (7, 0, lambda a, b: a & 0xFFFFFFFF < b & 0xFFFFFFFF),
          'ugew': (7, 1, lambda a, b: a & 0xFFFFFFFF >= b & 0xFFFFFFFF)}

VALS = [0, 1, 5, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, 0x100000005, 0xFFFFFFFF00000005,
        0x8000000000000000, M64, 0x12345678ABCDEF01, (-100000) & M64]

br_cases = []   # (name, words, a, b, expect)
for nm, (p, inv, fn) in W_PRED.items():
    for i, a in enumerate(VALS):
        b = VALS[(i * 5 + 3) % len(VALS)]
        br_cases.append((f'{nm} reg', br(lambda x, t, p=p, inv=inv: x.fbr(None, 1, 2, t, pred=p, inv=inv)), a, b, fn(a, b)))
    for imm in (-8, -1, 0, 5, 7):
        a = VALS[(imm + 9) % len(VALS)]
        br_cases.append((f'{nm} simm4 {imm}', br(lambda x, t, p=p, inv=inv, imm=imm: x.fbr(None, 1, imm, t, imm=True, pred=p, inv=inv)),
                         a, 0, fn(a, imm & M64)))
for op in CMP_PRED:
    for imm in (0, 1000, -100000, 0x7FFFFFFF, -1):
        for a in (0, 999, 1000, (-100000) & M64, 0x7FFFFFFF, 0x80000000, M64):
            br_cases.append((f'{op} imm32 {imm}', br(lambda x, t, op=op, imm=imm: x.fbr_imm32(op, 1, imm, t)), a, 0, cmp(op, a, imm & M64)))
for nm, (p, inv, fn) in W_PRED.items():
    for imm in (-5, 7, 0x12345678):
        for a in (0x00000123FFFFFFFB, 0x00000001FFFFFFF0, 0x12345678, 7):
            br_cases.append((f'{nm} imm32 {imm}', br(lambda x, t, p=p, inv=inv, imm=imm: x.fbr_imm32(None, 1, imm, t, pred=p, inv=inv)),
                             a, 0, fn(a, imm & M64)))

# short CMPRVW: branch on the flags it sets (Z / LT / ULT and inverses)
for cond, fn in (('eq', lambda a, b: a == b), ('ne', lambda a, b: a != b), ('lt', lambda a, b: s32(a) < s32(b)),
                 ('ge', lambda a, b: s32(a) >= s32(b)), ('ult', lambda a, b: a < b), ('ugt', lambda a, b: a > b)):
    for imm in (-128, -3, 0, 5, 127):
        for a in (0x100000005, 0xFFFFFFFD, 0x7FFFFFFF, 0x80000000, 5, 0xFFFFFF80):
            br_cases.append((f'cmprvw.s {cond} {imm}',
                             br(lambda x, t, imm=imm, cond=cond: (x.cmpi_s_w(1, imm), x.jmp(t, cond=cond))),
                             a, 0, fn(a & 0xFFFFFFFF, imm & 0xFFFFFFFF)))

val_cases = []  # (name, words, a, expect r3)
for v, imm, sgn in [(7, 6, True), (7, -6, True), ((-3) & M64, 100, True), (2**40 + 3, -128, True),
                    (5, 0xF0, False), (2**63, 2, True), (0x123456789, 127, False), (M64, -1, True)]:
    ref = (s64(v) * (imm if sgn else imm & 0xFF)) & M64
    val_cases.append((f'mulv.s {imm}', words(lambda x, imm=imm, sgn=sgn: x.mulv_s(3, 1, imm, sgn=sgn)), v, ref))
for off in (0, 8, 0x7FF0, -0x8000, -4, 0x1234):
    def lp(x, off=off):
        x.label('h')
        x.leapc_s(3, x.pc + off)     # r3 = here + off
        x.leapc(4, 'h')              # r4 = here
        x.alu('sub', 3, 3, 4)        # r3 = off
        x.alui('and', 3, 3, 0xFFFFFFFF, sgn=False)   # LEA is a 32-bit address
    val_cases.append((f'leapc.s {off}', words(lp), 0, off & 0xFFFFFFFF))

DI3 = words(lambda x: x.di(3))[0]
EI1 = words(lambda x: x.ei(1))[0]
EI3 = words(lambda x: x.ei(3))[0]
ADDI0 = words(lambda x: x.alui_s('add', 0, 0, 1))[0]      # addi r0, r0, 1
ADDI16 = words(lambda x: x.alui_s('add', 0, 0, 16))[0]
RET = words(lambda x: x.ret())[0]


def smc_seq(size, dist):
    """r1 = code buffer (addi r0,r0,1 ; ret), r2 = patch value; store the
    patch, `dist` independent ops, then CALLR r1 (r0 = result)"""
    def f(x):
        if size == 64:
            x.st(64, 2, 1)
        elif size == 32:
            x.st(32, 2, 1)
        else:
            x.st(8, 2, 1, 2)
        for _ in range(dist):
            x.alu('add', 5, 5, 5)
        x.jmpr(1, link=True)
    return words(f)


smc = []
for size in (64, 32, 8):
    for dist in (0, 1, 2, 3, 5, 9, 20):
        smc.append((size, dist, smc_seq(size, dist)))

# micro-kernels: N unrolled bodies; cycles measured around them from C
def kern(body, n=16):
    return words(lambda x: [body(x) for _ in range(n)])

KERN = [
    ('(empty: timer-read overhead)', [], 1),
    ('alu (1-word, independent)', kern(lambda x: x.alu('add', 3, 1, 2), 64), 64),
    ('setr (2-word)', kern(lambda x: x.setr(3, 0x12345), 32), 32),
    ('store 64', kern(lambda x: x.st(64, 1, 12, 0)), 16),
    ('store ; alu', kern(lambda x: (x.st(64, 1, 12, 0), x.alu('add', 3, 1, 2))), 16),
    ('load 64', kern(lambda x: x.ld(64, 3, 12, 8)), 16),
    ('store ; load', kern(lambda x: (x.st(64, 1, 12, 0), x.ld(64, 3, 12, 8))), 16),
    ('push ; pop', kern(lambda x: (x.push(1), x.pop(3))), 16),
    ('fbr ne taken (+1)', kern(lambda x: (x.label(f'k{x.pc}'), x.fbr('ne', 1, 0, x.pc + 4, imm=True)), 16), 16),
    ('fbr eq not taken', kern(lambda x: x.fbr('eq', 1, 0, x.pc + 8, imm=True)), 16),
    ('mul', kern(lambda x: x.mul(3, 1, 2)), 16),
    ('di ; ei', kern(lambda x: (x.di(3), x.ei(3))), 16),
]

L = ['// generated by tools/board/gen_hwprop.py — do not edit', '#pragma once', '#include <stdint.h>']
L.append('typedef struct { const char *name; uint64_t a, b; int expect; } br_case_t;')
L.append('typedef struct { const char *name; uint64_t a, expect; } val_case_t;')
L.append(f'#define N_BR {len(br_cases)}')
L.append(f'#define N_VAL {len(val_cases)}')
L.append('static const br_case_t BR_CASES[N_BR] = {')
L += [f'  {{"{n}", 0x{a & M64:x}ull, 0x{b & M64:x}ull, {2 if e else 1}}},' for n, w, a, b, e in br_cases]
L.append('};')
L.append('static const val_case_t VAL_CASES[N_VAL] = {')
L += [f'  {{"{n}", 0x{a & M64:x}ull, 0x{e & M64:x}ull}},' for n, w, a, e in val_cases]
L.append('};')
# one function per case: r1 = a, r2 = b -> r3
L.append('#define RUN_REGS(seq, a_, b_, out) do { \\')
L.append('  register uint64_t _r1 __asm__("r1") = (a_); register uint64_t _r2 __asm__("r2") = (b_); \\')
L.append('  register uint64_t _r3 __asm__("r3"); register uint64_t _r4 __asm__("r4"); \\')
L.append('  __asm__ volatile(seq : "=r"(_r3), "=r"(_r4), "+r"(_r1), "+r"(_r2) : : "memory", "cc"); (out) = _r3; } while (0)')
L.append('static uint64_t run_br(int i, uint64_t a, uint64_t b) { uint64_t o = 0; switch (i) {')
for i, (n, w, a, b, e) in enumerate(br_cases):
    L.append(f'  case {i}: RUN_REGS({asm_str(w)}, a, b, o); break;')
L.append('  } return o; }')
L.append('static uint64_t run_val(int i, uint64_t a) { uint64_t o = 0; switch (i) {')
for i, (n, w, a, e) in enumerate(val_cases):
    L.append(f'  case {i}: RUN_REGS({asm_str(w)}, a, 0, o); break;')
L.append('  } return o; }')
L.append(f'#define DI_R3 ".4byte 0x{DI3:08X}\\n"')
L.append(f'#define EI_R1 ".4byte 0x{EI1:08X}\\n"')
L.append(f'#define EI_R3 ".4byte 0x{EI3:08X}\\n"')
L.append(f'#define W_ADDI1 0x{ADDI0:08X}u')
L.append(f'#define W_ADDI16 0x{ADDI16:08X}u')
L.append(f'#define W_RET 0x{RET:08X}u')
L.append(f'#define N_SMC {len(smc)}')
L.append('static const int SMC_SIZE[N_SMC] = {' + ','.join(str(s) for s, d, w in smc) + '};')
L.append('static const int SMC_DIST[N_SMC] = {' + ','.join(str(d) for s, d, w in smc) + '};')
L.append('static uint64_t run_smc(int i, uint32_t *buf, uint64_t patch) {')
L.append('  register uint64_t r0 __asm__("r0") = 0; register uint64_t r1 __asm__("r1") = (uintptr_t)buf;')
L.append('  register uint64_t r2 __asm__("r2") = patch; register uint64_t r5 __asm__("r5") = 0; switch (i) {')
for i, (s, d, w) in enumerate(smc):
    L.append(f'  case {i}: __asm__ volatile({asm_str(w)} : "+r"(r0), "+r"(r1), "+r"(r2), "+r"(r5) : : "memory"); break;')
L.append('  } return r0; }')
L.append(f'#define N_KERN {len(KERN)}')
L.append('static const char *KERN_NAME[N_KERN] = {' + ','.join(f'"{n}"' for n, w, c in KERN) + '};')
L.append('static const int KERN_N[N_KERN] = {' + ','.join(str(c) for n, w, c in KERN) + '};')
L.append('static uint64_t run_kern(int i, uint64_t *buf) {')
L.append('  register uint64_t r1 __asm__("r1") = 1; register uint64_t r2 __asm__("r2") = 2;')
L.append('  register uint64_t r3 __asm__("r3"); register uint64_t r12 __asm__("r12") = (uintptr_t)buf;')
L.append('  volatile uint64_t *pc = (volatile uint64_t *)0xF00D0008u; uint64_t t0 = *pc; switch (i) {')
for i, (n, w, c) in enumerate(KERN):
    L.append(f'  case {i}: __asm__ volatile({asm_str(w)} : "=r"(r3), "+r"(r1), "+r"(r2), "+r"(r12) : : "memory"); break;')
L.append('  } return *pc - t0; }')
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hwprop_snippets.h'), 'w').write('\n'.join(L) + '\n')
print(len(br_cases), 'branch cases', len(val_cases), 'value cases', len(smc), 'smc', len(KERN), 'kernels')

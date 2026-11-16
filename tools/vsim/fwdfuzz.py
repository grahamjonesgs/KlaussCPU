"""Forward fuzzer (tmg review): producers into r0..r11 / r13 feeding every
memory-op addressing mode at distance 0..2.

randprog writes only r1..r10, so a producer in MEM whose rd equals a field
the consumer does NOT use as a register — rs1 = 0 of an [imm] access, rs2 =
0 of an [rs1+imm32] or [rs1+simm8] one — never occurs there, and the ID ->
EX address operands' forward selects (`t_ta_d` / `t_tb_d`, the tmg shift
forward `fs_a` / `fs_b`) were never tested against such a false match.

Each case: a producer — a shift (SHL / SHR / SAR / ROL, weighted: the MEM
stage-2 forward), an ALU op, an ALU immediate, a load, a MUL or a POP —
writes register t; 0..2 independent fillers; then a memory op (a 64-bit or
narrower load / store; [rs1], [rs1+simm8], [rs1+imm32], [imm], [rs1+rs2];
PUSH / POP; MEMGET32) whose base, index or data register is t (the
producer then makes a valid address / offset), or whose unused field
aliases t (the producer writes anything).  Every access stays in the data
window 0x10000..0x10800 (the TB's dumped memory) or the stack.

run.py fwdfuzz: the same programs on a base revision and the working tree
(identical retire traces), at +LAT 1 / 2 / 3 / 5 and +HIT 0 / 70 / 95, and
optionally through tb_soc.
"""
import random
from kasm import Asm

DATA = 0x10000
TGT = list(range(0, 12)) + [13]          # producer targets (r12 = the data base, r14 scratch, r15 = SP-ish)


def gen(seed, n_cases=140):
    rnd = random.Random(seed)
    a = Asm()
    a.setr(12, DATA)
    for r in range(0, 12):
        a.setr(r, DATA + 8 * rnd.randrange(0, 200))
    a.setr(13, 0x40); a.setr(14, 1); a.setr(15, 0)
    # a table of valid addresses at DATA + 0x700 for the load producers
    slots = []
    for i in range(16):
        v = DATA + 8 * rnd.randrange(0, 200)
        a.setr(14, v); a.st(64, 14, 12, 0x700 + 8 * i)
        slots.append(v)
    a.setr(14, 1)

    def produce(t, v, kind):
        """t = v (a valid value), by kind; src registers other than t"""
        srcs = [r for r in range(0, 12) if r != t]
        s = rnd.choice(srcs)
        if kind == 'shl':
            a.setr(s, v >> 3); a.shift_n('shl', t, s, 3)
        elif kind == 'shr':
            a.setr64(s, v << 4); a.shift_n('shr', t, s, 4)
        elif kind == 'sar':
            a.setr64(s, v << 2); a.shift_n('sar', t, s, 2)
        elif kind == 'rol':
            a.setr64(s, ((v >> 5) | (v << 59)) & ((1 << 64) - 1)); a.shift_n('rol', t, s, 5)
        elif kind == 'alu':
            k = rnd.randrange(1, 1000)
            a.setr(s, v - k); a.setr(14, k); a.alu('add', t, s, 14); a.setr(14, 1)
        elif kind == 'alui':
            a.setr(s, v - 7); a.addi_s(t, s, 7)
        elif kind == 'load':
            i = rnd.randrange(16)
            v = slots[i]
            a.ld(64, t, 12, 0x700 + 8 * i)
        elif kind == 'mul':
            a.setr(s, v); a.mul(t, s, 14)            # r14 = 1
        elif kind == 'pop':
            a.pushv(v); a.pop(t)
        return v

    def filler(n, avoid):
        for _ in range(n):
            f = rnd.random()
            if f < 0.4:
                a.nop()
            else:
                r = rnd.choice([x for x in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11) if x not in avoid])
                a.alui_s('xor', r, r, rnd.randrange(-8, 8) * 0)   # (xor 0: keeps the value)

    kinds = ['shl', 'shr', 'sar', 'rol'] * 3 + ['alu', 'alui', 'load', 'mul', 'pop']
    for _ in range(n_cases):
        t = rnd.choice(TGT)
        kind = rnd.choice(kinds)
        d = rnd.choice([0, 1, 2])
        mode = rnd.choice(['rs1', 'short', 'long', 'abs', 'rr_b', 'rr_i', 'data', 'alias_long', 'alias_abs',
                           'alias_short', 'push', 'pop', 'ld32'])
        sz = rnd.choice([8, 16, 32, 64])
        st = rnd.random() < 0.4
        off = 8 * rnd.randrange(0, 40)
        b = 12 if t != 12 else 1                      # (t is never 12)
        if mode in ('rs1', 'short', 'long', 'ld32'):
            # t is the base: a valid address
            v = produce(t, DATA + 8 * rnd.randrange(0, 160), kind)
            filler(d, {t})
            if mode == 'rs1':
                (a.st(sz, 14, t) if st else a.ld(sz, rnd.choice([1, 2, 3]) if t not in (1, 2, 3) else 4, t))
            elif mode == 'short':
                o = (off // (sz // 8)) * (sz // 8) if sz > 8 else off
                o = min(o, 127 * (sz // 8))
                (a.st(sz, 14, t, o) if st else a.ld(sz, 5 if t != 5 else 6, t, o))
            elif mode == 'long':
                (a.st(sz, 14, t, off, mode='long') if st else a.ld(sz, 5 if t != 5 else 6, t, off, mode='long'))
            else:
                a.ld(32, 7 if t != 7 else 8, t, 4)       # MEMGET32 at +4 (spans dwords)
        elif mode == 'rr_b':
            v = produce(t, DATA + 8 * rnd.randrange(0, 160), kind)
            filler(d, {t})
            a.setr(13 if t != 13 else 11, off) if False else None
            (a.st_rr(sz, 14, t, 15) if st else a.ld_rr(sz, 9 if t != 9 else 10, t, 15))   # r15 = 0
        elif mode == 'rr_i':
            # t is the index: a small offset
            v = produce(t, 8 * rnd.randrange(0, 160), kind if kind not in ('load',) else 'alui')
            filler(d, {t})
            (a.st_rr(sz, 14, b, t) if st else a.ld_rr(sz, 9 if t != 9 else 10, b, t))
        elif mode == 'data':
            # t is a store's data
            produce(t, rnd.getrandbits(32), kind if kind not in ('load', 'pop') else 'shl')
            filler(d, {t})
            a.st(sz, t, b, off)
        elif mode in ('alias_long', 'alias_abs', 'alias_short'):
            # t aliases an unused field: rs2 = 0 of a long / short form, rs1 =
            # 0 of an [imm] one; any value
            t = 0
            produce(t, rnd.getrandbits(48) | 1, kind if kind != 'load' else 'shl')
            filler(d, {t})
            if mode == 'alias_long':
                (a.st(sz, 14, b, off, mode='long') if st else a.ld(sz, 5, b, off, mode='long'))
            elif mode == 'alias_short':
                (a.st(64, 14, b, off) if st else a.ld(64, 5, b, off))
            else:
                (a.st_abs(sz, 14, DATA + off) if st else a.ld_abs(sz, 5, DATA + off))
            a.setr(0, DATA + 8 * rnd.randrange(0, 160))
        elif mode == 'push':
            produce(t, rnd.getrandbits(40), kind if kind != 'pop' else 'shl')
            filler(d, {t})
            a.push(t); a.pop(14); a.st(64, 14, 12, off); a.setr(14, 1)
        else:   # pop into t, then t as a base
            a.pushv(DATA + 8 * rnd.randrange(0, 160)); a.pop(t)
            filler(d, {t})
            a.ld(64, 5 if t != 5 else 6, t, 0)
        # keep every base register valid for later cases
        if t not in (12, 14, 15):
            a.setr(t, DATA + 8 * rnd.randrange(0, 200))
        a.setr(14, 1)
    a.halt()
    return a

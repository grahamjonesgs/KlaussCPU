"""Random terminating programs for old-vs-new differential runs.

Registers: r1..r10 random data, r11 loop counter, r12 data base (0x10000),
r13/r14 scratch, r15 frame pointer (ENTER/LEAVE).  Branches go forward
except the bounded loops, so every program halts.  Memory traffic stays in
the TB's dumped window 0x10000..0x10800 plus the stack.
"""
import random
from kasm import Asm, CMP_PRED, JCOND

DATA = 0x10000
R = list(range(1, 11))


def gen(seed, n_blocks=40, irq=False):
    rnd = random.Random(seed)
    a = Asm()
    lbl = [0]

    def L():
        lbl[0] += 1
        return f'L{lbl[0]}'

    for r in R:
        if rnd.random() < 0.3:
            a.setr64(r, rnd.getrandbits(64))
        else:
            a.setr(r, rnd.getrandbits(32), sgn=rnd.random() < 0.5)
    a.setr(12, DATA)
    a.setr(15, 0)
    if irq:
        a.set_int_mask(1)
    funcs = []

    def off(size):
        return rnd.randrange(0, 0x800 // size) * size if size > 1 else rnd.randrange(0, 0x800)

    def straight(n):
        for _ in range(n):
            k = rnd.random()
            rd, r1, r2 = rnd.choice(R), rnd.choice(R), rnd.choice(R)
            if k < 0.22:
                op = rnd.choice(['add', 'sub', 'and', 'or', 'xor', 'adc', 'sbc'])
                a.alu(op, rd, r1, r2, w=op in ('add', 'sub') and rnd.random() < 0.15)
            elif k < 0.30:
                a.alui_s(rnd.choice(['add', 'sub', 'and', 'or', 'xor']), rd, r1, rnd.randrange(-128, 128))
            elif k < 0.34:
                a.alui(rnd.choice(['add', 'sub', 'and', 'or', 'xor']), rd, r1, rnd.getrandbits(32))
            elif k < 0.48:
                sz = rnd.choice([8, 16, 32, 64])
                o = off(sz // 8)
                f = rnd.random()
                if f < 0.5:
                    a.ld(sz, rd, 12, o, sgn=rnd.random() < 0.5 and sz < 64)
                elif f < 0.75:
                    a.ld(sz, rd, 12, o, sgn=rnd.random() < 0.5 and sz < 64, mode='long')
                else:
                    a.setr(13, o); a.ld_rr(sz, rd, 12, 13, sgn=rnd.random() < 0.5 and sz < 64)
            elif k < 0.60:
                sz = rnd.choice([8, 16, 32, 64])
                o = off(sz // 8)
                if rnd.random() < 0.7:
                    a.st(sz, rd, 12, o)
                else:
                    a.setr(13, o); a.st_rr(sz, rd, 12, 13)
            elif k < 0.63:
                # MEMGET32 ([rs1], any byte alignment)
                a.setr(13, DATA + rnd.randrange(0, 0x7F8)); a.ld(32, rd, 13)
            elif k < 0.68:
                a.shift_n(rnd.choice(['shl', 'shr', 'sar', 'rol', 'ror']), rd, r1, rnd.randrange(64),
                          f=rnd.random() < 0.3)
            elif k < 0.73:
                a.mul(rd, r1, r2, sgn=rnd.random() < 0.5, high=rnd.random() < 0.3)
            elif k < 0.745:
                (a.div if rnd.random() < 0.5 else a.mod)(rd, r1, r2, sgn=rnd.random() < 0.5)
            elif k < 0.78:
                a.cmp(r1, r2, w=rnd.random() < 0.2)
                a.setcc(rnd.choice(list(CMP_PRED)), rd, r1, r2)
            elif k < 0.82:
                a.push(r1); straight_simple(1); a.pop(rd)
            elif k < 0.86:
                a.inc(rd) if rnd.random() < 0.5 else a.dec(rd)
            elif k < 0.90 and funcs:
                a.call(rnd.choice(funcs))
            elif k < 0.93:
                a.mov(rd, r1)
            else:
                a.nop()

    def straight_simple(n):
        for _ in range(n):
            a.alu(rnd.choice(['add', 'xor']), rnd.choice(R), rnd.choice(R), rnd.choice(R))

    # leaf functions first (jumped over)
    skip = L()
    a.jmp(skip)
    for i in range(3):
        f = f'F{i}'
        funcs.append(f)
        a.label(f)
        if rnd.random() < 0.5:
            a.enter(rnd.randrange(0, 3))
            straight_simple(rnd.randrange(1, 5))
            a.leaveret() if rnd.random() < 0.6 else (a.leave(), a.ret())
        else:
            straight_simple(rnd.randrange(1, 5))
            a.ret()
    a.label(skip)

    for _ in range(n_blocks):
        k = rnd.random()
        if k < 0.5:
            straight(rnd.randrange(1, 8))
        elif k < 0.7:
            # forward conditional skip
            t = L()
            r1, r2 = rnd.choice(R), rnd.choice(R)
            if rnd.random() < 0.5:
                a.fbr(rnd.choice(list(CMP_PRED)), r1, r2, t)
            else:
                a.cmp(r1, r2, w=rnd.random() < 0.2)
                a.jmp(t, cond=rnd.choice(list(JCOND)))
            straight(rnd.randrange(1, 5))
            a.label(t)
        else:
            # bounded loop
            t = L()
            a.setr(11, rnd.randrange(1, 6))
            a.label(t)
            straight(rnd.randrange(1, 6))
            a.addi_s(11, 11, -1)
            if rnd.random() < 0.5:
                a.fbr('ne', 11, 0, t, imm=True)
            else:
                a.cmpi_s(11, 0)
                a.jmp(t, cond='ne')
    a.halt()
    return a

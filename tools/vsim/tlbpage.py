"""tools/vsim/tlbpage.py (run.py tlbpage) — from the stage 3 review (review 3,
which found the page-step-under-mem_busy bug with it): translated random
programs with NON-identity data and stack pages and page-crossing accesses.
The data window straddles a page boundary in VA and PA (offsets biased to
it, MEMGET32 biased to the page end), the user stack sits so a CALL+ENTER
frame straddles a page, R15 lands on a page's last doubleword; a kseg
refill handler (TLBP must miss) over K entries, injected SYSCALLs bump
ASID and TLBINV on the wrap, optional IRQs.  The TE = 1 user run's final
state must equal the physical run's.
"""
import random, re
import randprog
from kasm import Asm, CMP_PRED, JCOND
from tests import _tlb_set, HALT_W, KSP, TLB_RWX

DATA_PA = 0x1FC00                      # straddles PA 0x20000 (pages 1 | 2)
USP_PA_BASE = 0x50000                  # stack straddles PA 0x50000 (pages 4 | 5)


def gen(seed, A, data, n_blocks=40):
    """randprog.gen with offsets biased to the page boundary at data+0x400"""
    rnd = random.Random(seed)
    a = A
    lbl = [0]
    Rr = list(range(1, 11))

    def L():
        lbl[0] += 1
        return f'L{lbl[0]}'
    for r in Rr:
        if rnd.random() < 0.3:
            a.setr64(r, rnd.getrandbits(64))
        else:
            a.setr(r, rnd.getrandbits(32), sgn=rnd.random() < 0.5)
    a.setr(12, data)
    a.setr(15, 0)
    funcs = []

    def off(size):
        if rnd.random() < 0.5:
            o = 0x400 + rnd.randrange(-24, 24)
            return max(0, o - o % size)
        return rnd.randrange(0, 0x800 // size) * size if size > 1 else rnd.randrange(0, 0x800)

    def straight(n):
        for _ in range(n):
            k = rnd.random()
            rd, r1, r2 = rnd.choice(Rr), rnd.choice(Rr), rnd.choice(Rr)
            if k < 0.18:
                op = rnd.choice(['add', 'sub', 'and', 'or', 'xor', 'adc', 'sbc'])
                a.alu(op, rd, r1, r2, w=op in ('add', 'sub') and rnd.random() < 0.15)
            elif k < 0.24:
                a.alui_s(rnd.choice(['add', 'sub', 'and', 'or', 'xor']), rd, r1, rnd.randrange(-128, 128))
            elif k < 0.40:
                sz = rnd.choice([8, 16, 32, 64])
                o = off(sz // 8)
                f = rnd.random()
                if f < 0.5:
                    a.ld(sz, rd, 12, o, sgn=rnd.random() < 0.5 and sz < 64)
                elif f < 0.75:
                    a.ld(sz, rd, 12, o, sgn=rnd.random() < 0.5 and sz < 64, mode='long')
                else:
                    a.setr(13, o); a.ld_rr(sz, rd, 12, 13, sgn=rnd.random() < 0.5 and sz < 64)
            elif k < 0.55:
                sz = rnd.choice([8, 16, 32, 64])
                o = off(sz // 8)
                if rnd.random() < 0.7:
                    a.st(sz, rd, 12, o)
                else:
                    a.setr(13, o); a.st_rr(sz, rd, 12, 13)
            elif k < 0.62:
                # MEMGET32 ([rs1]), biased to straddle the page
                o = 0x3F8 + rnd.randrange(0, 8) if rnd.random() < 0.6 else rnd.randrange(0, 0x7F8)
                a.setr(13, o); a.alu('add', 13, 13, 12); a.ld(32, rd, 13)
            elif k < 0.66:
                a.shift_n(rnd.choice(['shl', 'shr', 'sar', 'rol', 'ror']), rd, r1, rnd.randrange(64),
                          f=rnd.random() < 0.3)
            elif k < 0.70:
                hi = rnd.random() < 0.3
                a.mul(rd, r1, r2, sgn=rnd.random() < 0.5, high=hi, w=(not hi) and rnd.random() < 0.25)
            elif k < 0.715:
                (a.div if rnd.random() < 0.5 else a.mod)(rd, r1, r2, sgn=rnd.random() < 0.5)
            elif k < 0.75:
                a.cmp(r1, r2, w=rnd.random() < 0.2)
                a.setcc(rnd.choice(list(CMP_PRED)), rd, r1, r2)
            elif k < 0.80:
                a.push(r1); straight_simple(1); a.pop(rd)
            elif k < 0.83:
                a.inc(rd) if rnd.random() < 0.5 else a.dec(rd)
            elif k < 0.90 and funcs:
                a.call(rnd.choice(funcs))
            elif k < 0.93:
                a.mov(rd, r1)
            else:
                a.nop()

    def straight_simple(n):
        for _ in range(n):
            k = rnd.random()
            if k < 0.3:      # a load / store inside the frame too
                sz = 64
                o = off(8)
                if rnd.random() < 0.5:
                    a.ld(sz, rnd.choice(Rr), 12, o)
                else:
                    a.st(sz, rnd.choice(Rr), 12, o)
            else:
                a.alu(rnd.choice(['add', 'xor']), rnd.choice(Rr), rnd.choice(Rr), rnd.choice(Rr))

    skip = L()
    a.jmp(skip)
    for i in range(3):
        f = f'F{i}'
        funcs.append(f)
        a.label(f)
        if rnd.random() < 0.8:
            a.enter(rnd.randrange(0, 3))
            straight_simple(rnd.randrange(1, 5))
            a.leaveret() if rnd.random() < 0.6 else (a.leave(), a.ret())
        else:
            straight_simple(rnd.randrange(1, 5))
            a.ret()
    a.label(skip)
    for _ in range(n_blocks):
        k = rnd.random()
        if k < 0.6:
            straight(rnd.randrange(1, 8))
        elif k < 0.8:
            t = L()
            r1, r2 = rnd.choice(Rr), rnd.choice(Rr)
            if rnd.random() < 0.5:
                a.fbr(rnd.choice(list(CMP_PRED)), r1, r2, t)
            else:
                a.cmp(r1, r2, w=rnd.random() < 0.2)
                a.jmp(t, cond=rnd.choice(list(JCOND)))
            straight(rnd.randrange(1, 5))
            a.label(t)
        else:
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


def phys(seed, usp):
    a = Asm()
    a.setr(1, usp); a.setsp(1)
    return gen(seed, a, DATA_PA)


def virt(seed, usp_pa, p_sys=0.04, sup=False, irq=False):
    rnd = random.Random(seed * 7919 + 5)
    K = rnd.choice([8, 16] if irq else [4, 8, 16])
    CVP = rnd.choice([0x0040, 0x0123, 0x07F0])
    DVP = rnd.choice([0x0201, 0x0333, 0x1000])      # VA data pages DVP, DVP+1 -> PA 1, 2
    SVP = rnd.choice([0x0405, 0x0600, 0x2222])      # VA stack pages SVP-1, SVP -> PA 4, 5
    data_va = (DVP << 16) | (DATA_PA & 0xFFFF)
    usp_va = (SVP << 16) | (usp_pa & 0xFFFF)
    CNT = 0x80200000
    maps = {CVP: 0, DVP: 1, DVP + 1: 2, SVP - 1: 4, SVP: 5}

    class TA(Asm):
        def __init__(self, *aa, **kk):
            super().__init__(*aa, **kk)
            self.inj = False
            self.leapc(1, 'h'); self.setr(2, 0x80000000); self.alu('or', 1, 1, 2)
            self.mtsr('evec', 1); self.mtsr('tlbvec', 1)
            self.setr(1, KSP); self.setsp(1)
            _tlb_set(self, 0, 0x0000, 0x000, TLB_RWX, g=1)
            self.setr(1, 0x200); self.mtsr('status', 1)
            self.leapc(1, 'kz'); self.setr(2, 0x80000000); self.alu('or', 1, 1, 2); self.jmpr(1)
            self.label('kz')
            self.tlbinv()
            if sup:
                # supervisor: SP is the "user" stack VA (not kseg) - no, a
                # supervisor run needs SP in kseg for frames: keep KSP; the
                # program's own SP is its stack VA, swapped back on entry?
                # (not supported: run user mode)
                pass
            self.setr(1, usp_va); self.mtsr('usp', 1)
            self.leapc(2, 'u'); self.alui('and', 2, 2, 0xFFFF, sgn=False)
            self.setr(3, CVP << 16); self.alu('or', 2, 2, 3)
            self.setr64(3, (1 << 44) | ((1 if irq else 0) << 39)); self.alu('or', 2, 2, 3)
            self.push(2); self.iret()
            self.label('u')
            self.inj = True

        def emit(self, *ws):
            if self.inj and rnd.random() < p_sys:
                super().emit(0x6C080000)                  # SYSCALL
            return super().emit(*ws)

    a = TA()
    gen(seed, a, data_va)
    a.inj = False
    a.label('h')
    a.mtsr('scratch0', 13); a.mtsr('scratch1', 14)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, 'h_nh'); a.halt()
    a.label('h_nh')
    a.mfsr(13, 'ecause'); a.alui('and', 13, 13, 0xFF, sgn=False)
    a.fbr('eq', 13, 4, 'h_sys', imm=True)
    a.addi_s(14, 13, -11); a.fbr('ule', 14, 2, 'h_miss', imm=True)
    a.label('h_bad')
    a.halt()
    a.label('h_miss')
    a.tlbp(); a.mfsr(13, 'tlb_index')
    a.shift_n('shr', 13, 13, 31); a.fbr('ne', 13, 1, 'h_bad', imm=True)
    a.mfsr(13, 'tlb_hi'); a.shift_n('shr', 13, 13, 16)
    for i, (v, p) in enumerate(maps.items()):
        a.setr(14, v); a.fbr('ne', 13, 14, f'h_m{i}')
        a.setr(13, p); a.jmp('h_id')
        a.label(f'h_m{i}')
    a.label('h_id')
    a.shift_n('shl', 13, 13, 16); a.alui('or', 13, 13, TLB_RWX, sgn=False)
    a.mtsr('tlb_lo', 13)
    a.setr(13, CNT); a.ld(64, 14, 13); a.inc(14)
    a.alui('and', 14, 14, K - 1, sgn=False); a.st(64, 14, 13)
    a.mtsr('tlb_index', 14); a.tlbwi()
    a.jmp('h_ret')
    a.label('h_sys')
    a.mfsr(13, 'asid'); a.inc(13); a.alui('and', 13, 13, 3, sgn=False)
    a.mtsr('asid', 13)
    a.fbr('ne', 13, 0, 'h_ret', imm=True)
    a.tlbinv()
    a.label('h_ret')
    a.mfsr(13, 'scratch0'); a.mfsr(14, 'scratch1')
    a.iret()
    return a, (K, hex(CVP), hex(DVP), hex(SVP))


def main(s0, n, lats=(3,), hit=None, irq=False, plus=()):
    import run as R
    exe = R.build(None)
    fails = 0
    for s in range(s0, s0 + n):
        usp_pa = USP_PA_BASE + 8 * (s % 4)
        for lat in lats:
            plus = [f'+LAT={lat}'] + ([f'+HIT={hit}'] if hit is not None else []) + ([f'+IRQ_PERIOD={301 + s % 97}'] if irq else []) + list(plus)
            sa, _ = R.run(exe, phys(s, usp_pa), f'tpa{s}_{lat}', plus)
            asm, info = virt(s, usp_pa, irq=irq)
            sb, _ = R.run(exe, asm, f'tpb{s}_{lat}', plus)
            keys = [f'r{i}' for i in list(range(1, 12)) + [15]] + \
                sorted({k for k in list(sa) + list(sb) if re.fullmatch(r'm[0-9a-f]{8}', k) and k != 'm00180000'})
            bad = [k for k in keys if sa.get(k) != sb.get(k)]
            ok = sa.get('end') == 'park' and sb.get('end') == 'park' and sb.get('kind') == '0' and \
                sa.get('kind') == '0' and not bad
            if 'fatal' in (sa['log'] + sb['log']).lower():
                ok = False
            if not ok:
                fails += 1
                print(f'DIFF seed={s} lat={lat} info={info} exc={sb.get("exc_n")} end {sa.get("end")}/{sb.get("end")} '
                      f'kind {sa.get("kind")}/{sb.get("kind")} {[(k, sa.get(k), sb.get(k)) for k in bad[:3]]}', flush=True)
    print(f'{fails} differing of {n * len(lats)}')
    return 1 if fails else 0

#!/usr/bin/env python3
"""permfuzz — the MMU permission fuzzer (Fable review pass 2, privilege /
MMU; adopted into tools/vsim in HW_PERF_PLAN §9).

    python3 tools/vsim/run.py permfuzz --seeds 100 --fetch --remap [--walk] [--irq]
    python3 tools/vsim/run.py permfuzz --seeds 20 --soc [--walk] [--fetch]
    python3 tools/vsim/run.py permfuzz --selfcheck      # PF_NOFLUSH: must fail

Random user-mode programs under TE = 1 whose data window straddles two
pages with random permissions (rw / r / w / none / x / absent), plus probes
of kernel addresses (kseg secret, the hole, kmap, MMIO), a MEMGET32 walking
off the top of useg, LR and DCBZ.  A kseg handler steps over every data
ACCESS / (absent-page) TLB miss.

Oracle: the SAME program with every statically-faulting instruction replaced
by NOPs, run with every page RWX (the reference).  Final registers and the
data window (copied out by the handler at the end) must match, the MMIO
access counts must match, and the kernel secret must be intact.  Also the
test run's fault log hash must be the same at every latency / hit model.

Self-check: PF_NOFLUSH=1 removes the remap flush (TLB invalidate / ASID
bump) from the test program only, so the fetch side keeps stale
translations; with --remap most seeds must then differ (--selfcheck runs
that and fails if they do not).

Uses this directory's run.py / kasm.py / tests.py; products in $VSIM_WORK.
"""
import os, sys, random, re, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run as R
from kasm import Asm, CMP_PRED, JCOND, TLB_V, TLB_R, TLB_W, TLB_X
from tests import _tlb_set, HALT_W, KSP, TLB_RWX

DATA_PA = 0x1FC00                      # straddles PA 0x20000 (pages 1 | 2)
SPAN = 0x780                           # data footprint (boundary at 0x400)
USP_PA_BASE = 0x50000                  # stack straddles PA 0x50000 (pages 4 | 5)
TOP_PA = 0x9                           # PA page for VA page 0x7FFF
SECRET_PA = 0x70000
SECRET = 0x5EC2E7C0DE5EC2E7
PTB = 0xA0000
OUT = 0x10000                          # the TB's dumped window
CNT = 0x80200000
HASHA = 0x80200008
PERMS = {'fabsent': 0, 'rw': TLB_V | TLB_R | TLB_W, 'rwx': TLB_RWX, 'r': TLB_V | TLB_R, 'rx': TLB_V | TLB_R | TLB_X,
         'w': TLB_V | TLB_W, 'none': TLB_V, 'x': TLB_V | TLB_X, 'absent': 0}


def gen(seed, a, data, perm, ref, p_sys=0.03, n_blocks=40, probes=True, fvp=None, fperm=None, mark=None, iretk=None, strad=None):
    """perm = (perm of data page 0, perm of data page 1) names"""
    rnd = random.Random(seed)
    lbl = [0]
    Rr = list(range(1, 10 if mark else 11))
    stats = {'fault': 0, 'acc': 0}

    def L():
        lbl[0] += 1
        return f'L{lbl[0]}'

    def page_of(o):                     # data page index of byte offset o
        return 0 if (DATA_PA & 0xFFFF) + o < 0x10000 else 1

    def bad(kind, pg):
        p = PERMS[perm[pg]]
        return not (p & (TLB_W if kind == 'st' else TLB_R))

    def mem(fault, emit):
        """emit one instruction; in the reference a faulting one becomes NOPs"""
        stats['acc'] += 1
        stats['fault'] += bool(fault)
        n0 = len(a.words)
        s0 = len(a.insn_starts)
        emit()
        if ref and fault:
            n = len(a.words) - n0
            del a.words[n0:]
            del a.insn_starts[s0:]
            for _ in range(n):
                a.nop()

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
        return rnd.randrange(0, SPAN // size) * size if size > 1 else rnd.randrange(0, SPAN)

    def probe():
        rd = rnd.choice(Rr)
        k = rnd.random()
        if k < 0.30:                    # kseg secret (mapped DDR, kernel only)
            a.setr(13, 0x80000000 | SECRET_PA + 8 * rnd.randrange(0, 4))
        elif k < 0.40:
            a.setr(13, rnd.choice([0x88000000, 0xBFFFFFF8, 0xA0001000]))        # the hole
        elif k < 0.50:
            a.setr(13, rnd.choice([0xC0000000, 0xE0001238, 0xEFFFFFF8]))        # kmap
        elif k < 0.70:
            a.setr(13, rnd.choice([0xF0FD0000, 0xF0FD0008, 0xF00F0000, 0xFFFFFFF8]))  # MMIO
        elif k < 0.85:
            # MEMGET32 at the top of useg: its 2nd dword is 0x8000_0000
            j = rnd.randrange(0, 8)
            a.setr(13, 0x7FFFFFF8 + j)
            mem(j > 4, lambda: a.ld(32, rd, 13))
            return
        else:
            # a kernel address through [rs1 + rs2] / [rs1 + imm] from a useg base
            tgt = 0x80000000 | SECRET_PA
            if rnd.random() < 0.5:
                a.setr(13, (tgt - data) & 0xFFFFFFFF)
                mem(True, lambda: a.ld_rr(64, rd, 12, 13))
            else:
                mem(True, lambda: a.ld(64, rd, 12, (tgt - data) & 0xFFFFFFFF, mode='long'))
            return
        sz = rnd.choice([8, 16, 32, 64])
        f = rnd.random()
        if f < 0.45:
            mem(True, lambda: a.ld(sz, rd, 13, 0 if sz != 32 or rnd.random() < 0.5 else None))
        elif f < 0.9:
            mem(True, lambda: a.st(sz, rd, 13, 0))
        elif f < 0.95:
            mem(True, lambda: a.dcbz(13))
        else:
            mem(True, lambda: a.lr(64, rd, 13))

    def fprobe():
        k = rnd.random()
        if k < 0.55 and funcs:
            # CALL a leaf function through an alias page of the code frame
            which = rnd.randrange(2)
            f = rnd.choice(funcs)
            flt = not (PERMS[fperm[which]] & TLB_X)
            stats['acc'] += 1; stats['fault'] += flt
            if ref and flt:
                a.nop(); a.nop()
            else:
                a.emit((2 << 30) | (8 << 26) | (1 << 25), lambda pc, f=f, w=which: (fvp[w] << 16) | (a.labels[f] & 0xFFFF))
        elif k < 0.8:
            # CALL a kernel / device / unmapped address
            tgt = rnd.choice([0x80000020, 0x80000000 | SECRET_PA, 0xF0FD0000, 0xF00F0000, 0x88000000, 0xC0000100,
                              0x7FFFFFFC, 0x7FFFFFF8, 0xFFFFFFF8])
            stats['acc'] += 1; stats['fault'] += 1
            if ref:
                a.nop(); a.nop()
            else:
                a.emit((2 << 30) | (8 << 26) | (1 << 25), tgt)
        elif k < 0.86 and strad:
            # CALL a code stub that straddles from an X page into the next page
            ev, lay, ok, imm = strad
            stats['acc'] += 1; stats['fault'] += (not ok)
            if ref:
                if not ok:
                    a.nop(); a.nop()
                elif lay == 0:
                    a.setr(1, imm & 0xFFFFFFFF)
                else:
                    a.setr64(1, imm)
            else:
                a.emit((2 << 30) | (8 << 26) | (1 << 25), (ev << 16) | 0xFFF8)
        elif k < 0.9 and iretk:
            # an IRET straight to a forbidden / misaligned PC (TRAP: the handler returns to r13)
            tgt = rnd.choice(iretk)
            t = L()
            stats['acc'] += 1; stats['fault'] += 1
            a.leapc(13, t); a.push(13)
            if callable(tgt):
                a.emit((2 << 30) | (2 << 26) | (15 << 22) | (13 << 8), lambda pc, g=tgt: g())
            else:
                a.setr(13, tgt)
            if ref:
                a.addsp(8)
            else:
                a.trap()
            a.label(t)
        else:
            # RET to such an address
            tgt = rnd.choice([0x80000020, 0xF0FD0008, 0xA0000000, 0xE0000000, 0x7FFFFFFC])
            t = L()
            stats['acc'] += 1; stats['fault'] += 1
            a.leapc(13, t); a.push(13)
            if ref:
                a.addsp(8); a.nop()
            else:
                a.pushv(tgt); a.ret()
            a.label(t)

    def badaddr(kind):
        # an address where a `kind` access must fault (None: use a kernel one)
        c = [0x80000000 | SECRET_PA + 8, 0x88000008, 0xC0000008, 0xF0FD0008, 0xF00F0000]
        for pg in (0, 1):
            if bad(kind, pg):
                c += [data + (0x100 if pg == 0 else 0x500)] * 3
        return rnd.choice(c)

    def sprobe():
        k = rnd.random()
        rd = rnd.choice(Rr)
        if k < 0.25:                    # LEAVE / LEAVERET with R15 at a forbidden frame
            lr = rnd.random() < 0.5
            ad = 0x7FFFFFF8 if lr and rnd.random() < 0.3 else badaddr('ld')
            a.mov(13, 15); a.setr(15, ad)
            mem(True, a.leaveret if lr else a.leave)
            a.mov(15, 13)
            return
        op = rnd.choice(['pop', 'push', 'ret', 'call', 'enter', 'pushv'])
        ad = badaddr('ld' if op in ('pop', 'ret') else 'st')
        if op not in ('pop', 'ret'):
            ad += 8                     # the store is at SP - 8
        a.getsp(13); a.setr(14, ad); a.setsp(14)
        if op == 'pop':
            mem(True, lambda: a.pop(rd))
        elif op == 'push':
            mem(True, lambda: a.push(rd))
        elif op == 'pushv':
            mem(True, lambda: a.pushv(0x12345678))
        elif op == 'ret':
            mem(True, a.ret)
        elif op == 'enter':
            mem(True, lambda: a.enter(rnd.randrange(0, 3)))
        elif funcs:
            f = rnd.choice(funcs)
            mem(True, lambda: a.call(f))
        a.setsp(13)

    def straight(n):
        for _ in range(n):
            if rnd.random() < p_sys:
                a.syscall()
            if probes and rnd.random() < 0.05:
                sprobe()
                continue
            if fvp and rnd.random() < 0.06:
                fprobe()
                continue
            if mark and rnd.random() < 0.10:
                # a world marker: the constant differs between the two code frames
                rm = rnd.choice(Rr)
                if mark == 'reg':
                    a.alu('xor', rm, rm, 10); a.nop()
                else:
                    a.alui('xor', rm, rm, mark)
                continue
            k = rnd.random()
            rd, r1, r2 = rnd.choice(Rr), rnd.choice(Rr), rnd.choice(Rr)
            if k < 0.16:
                op = rnd.choice(['add', 'sub', 'and', 'or', 'xor', 'adc', 'sbc'])
                a.alu(op, rd, r1, r2, w=op in ('add', 'sub') and rnd.random() < 0.15)
            elif k < 0.21:
                a.alui_s(rnd.choice(['add', 'sub', 'and', 'or', 'xor']), rd, r1, rnd.randrange(-128, 128))
            elif k < 0.38:
                sz = rnd.choice([8, 16, 32, 64])
                o = off(sz // 8)
                flt = bad('ld', page_of(o))
                sg = rnd.random() < 0.5 and sz < 64
                f = rnd.random()
                if f < 0.5:
                    mem(flt, lambda: a.ld(sz, rd, 12, o, sgn=sg))
                elif f < 0.75:
                    mem(flt, lambda: a.ld(sz, rd, 12, o, sgn=sg, mode='long'))
                else:
                    a.setr(13, o)
                    mem(flt, lambda: a.ld_rr(sz, rd, 12, 13, sgn=sg))
            elif k < 0.54:
                sz = rnd.choice([8, 16, 32, 64])
                o = off(sz // 8)
                flt = bad('st', page_of(o))
                if rnd.random() < 0.7:
                    mem(flt, lambda: a.st(sz, rd, 12, o))
                else:
                    a.setr(13, o)
                    mem(flt, lambda: a.st_rr(sz, rd, 12, 13))
            elif k < 0.61:
                # MEMGET32 ([rs1]), biased to straddle the page
                o = 0x3F8 + rnd.randrange(0, 8) if rnd.random() < 0.6 else rnd.randrange(0, SPAN - 8)
                flt = bad('ld', page_of(o & ~7)) or ((o & 7) > 4 and bad('ld', page_of((o & ~7) + 8)))
                a.setr(13, o); a.alu('add', 13, 13, 12)
                mem(flt, lambda: a.ld(32, rd, 13))
            elif k < 0.63:
                o = off(8)
                a.setr(13, o); a.alu('add', 13, 13, 12)
                mem(bad('ld', page_of(o)), lambda: a.lr(64, rd, 13))
            elif k < 0.65:
                o = off(1)
                a.setr(13, o); a.alu('add', 13, 13, 12)
                mem(bad('st', page_of(o & ~31)), lambda: a.dcbz(13))
            elif k < 0.71 and probes:
                probe()
            elif k < 0.74:
                a.shift_n(rnd.choice(['shl', 'shr', 'sar', 'rol', 'ror']), rd, r1, rnd.randrange(64),
                          f=rnd.random() < 0.3)
            elif k < 0.77:
                hi = rnd.random() < 0.3
                a.mul(rd, r1, r2, sgn=rnd.random() < 0.5, high=hi, w=(not hi) and rnd.random() < 0.25)
            elif k < 0.785:
                (a.div if rnd.random() < 0.5 else a.mod)(rd, r1, r2, sgn=rnd.random() < 0.5)
            elif k < 0.81:
                a.cmp(r1, r2, w=rnd.random() < 0.2)
                a.setcc(rnd.choice(list(CMP_PRED)), rd, r1, r2)
            elif k < 0.85:
                a.push(r1); straight_simple(1); a.pop(rd)
            elif k < 0.87:
                a.inc(rd) if rnd.random() < 0.5 else a.dec(rd)
            elif k < 0.93 and funcs:
                a.call(rnd.choice(funcs))
            elif k < 0.96:
                a.mov(rd, r1)
            else:
                a.nop()

    def straight_simple(n):
        for _ in range(n):
            k = rnd.random()
            if k < 0.4:      # a load / store inside the frame too
                o = off(8)
                rr = rnd.choice(Rr)
                if rnd.random() < 0.5:
                    mem(bad('ld', page_of(o)), lambda: a.ld(64, rr, 12, o))
                else:
                    mem(bad('st', page_of(o)), lambda: a.st(64, rr, 12, o))
            else:
                a.alu(rnd.choice(['add', 'xor']), rnd.choice(Rr), rnd.choice(Rr), rnd.choice(Rr))

    skip = L()
    a.jmp(skip)
    for i in range(3):
        f = f'F{i}'
        funcs.append(f)
        a.label(f)
        if mark:
            rm = rnd.choice(Rr)
            if mark == 'reg':
                a.alu('xor', rm, rm, 10); a.nop()
            else:
                a.alui('xor', rm, rm, mark)
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
    return stats


CA, CB = 0x1357A5C3, 0x2468C35A
FB = 3                                  # the 2nd code frame (PA page)
WVAR = 0x80200010


def build_prog(seed, usp_pa, ref, walk=False, irq=False, probes=True, fetch=False, remap=False, mark=None, soc=False):
    rnd = random.Random(seed * 7919 + 5)
    K = rnd.choice([8, 16] if irq else [4, 8, 16])
    CVP = rnd.choice([0x0040, 0x0123, 0x07F0])
    DVP = rnd.choice([0x0201, 0x0333, 0x1000])      # VA data pages DVP, DVP+1 -> PA 1, 2
    SVP = rnd.choice([0x0405, 0x0600, 0x2222])      # VA stack pages SVP-1, SVP -> PA 4, 5
    plist = ['rw', 'rwx', 'r', 'rx', 'w', 'none', 'x', 'absent']
    perm = [rnd.choice(plist), rnd.choice(plist)]
    perm[rnd.randrange(2)] = rnd.choice(['rw', 'rwx'] + plist)     # usually one page is plain
    tperm = ['rwx', 'rwx'] if ref else perm
    data_va = (DVP << 16) | (DATA_PA & 0xFFFF)
    usp_va = (SVP << 16) | (usp_pa & 0xFFFF)
    maps = {CVP: (0, 'rwx'), DVP: (1, tperm[0]), DVP + 1: (2, tperm[1]), SVP - 1: (4, 'rw'), SVP: (5, 'rw'),
            0x7FFF: (TOP_PA, rnd.choice(['rw', 'rwx', 'rwx', 'rx']) if fetch else 'rw')}
    topw = rnd.choice([0x6C000000, 0x8BC00100, 0x8BC00100])   # the word at 0x7FFF_FFFC: NOP / SETR r1 (2 words)
    fvp = fperm = None
    if fetch:
        fvp = [CVP + 0x11, CVP + 0x12]
        fperm = [rnd.choice(['rx', 'rwx', 'x', 'x']), rnd.choice(['r', 'rw', 'none', 'w', 'fabsent', 'rx'])]
        maps[fvp[0]] = (0, fperm[0]); maps[fvp[1]] = (0, fperm[1])

    strad = None
    if fetch:
        EV = CVP + 0x20
        sperm = rnd.choice(['rx', 'rwx', 'fabsent', 'none', 'rw', 'r', 'fabsent'])
        lay = rnd.randrange(3)
        simm = rnd.getrandbits(64)
        maps[EV] = (8, rnd.choice(['rx', 'x'])); maps[EV + 1] = (0xC, sperm)
        strad = (EV, lay, bool(PERMS[sperm] & TLB_X), simm)
        NOPW, SETR1, SETR641, RETW = 0x6C000000, 0x8BC00100, 0xCBC00100, 0x65800000
        lo, hi = simm & 0xFFFFFFFF, simm >> 32
        w = {0: [NOPW, SETR1, lo, RETW], 1: [SETR641, lo, hi, RETW], 2: [NOPW, SETR641, lo, hi, RETW, NOPW]}[lay]
        sdw = {(8 << 16) | 0xFFF8: (w[1] << 32) | w[0], (0xC << 16): (w[3] << 32) | w[2]}
        if lay == 2:
            sdw[(0xC << 16) | 8] = (w[5] << 32) | w[4]
    def mkpte(p, pm):
        return (0x5A5A0000F800FFE0 & ~0x07FF0010) | (p << 16) | (0 if pm in ('absent', 'fabsent') else 0x10) | PERMS[pm]
    pages = dict(maps)
    pages[0x10] = (0x10, 'rwx'); pages[0x18] = (0x18, 'rwx')
    tabs, pte_addr = {}, {}
    for v in pages:
        d = v >> 10
        if d not in tabs:
            tabs[d] = PTB + 0x2000 * (1 + len(tabs))
        pte_addr[v] = tabs[d] + 8 * (v & 0x3FF)

    a = Asm()
    inits = {}
    for i in range(4):
        inits[SECRET_PA + 8 * i] = SECRET + i
    inits[(TOP_PA << 16) | 0xFFF8] = (topw << 32) | 0x6C000000 if fetch else 0x7071727374757677
    if strad:
        inits.update(sdw)
    if walk:
        for d, t in tabs.items():
            inits[PTB + 8 * d] = 0xA5A50000F8001FFE & ~0x07FFE000 | t | 1
        for v, (p, pm) in pages.items():
            inits[pte_addr[v]] = mkpte(p, pm)
    if soc:
        # tb_soc's boot image is small: everything outside the code is stored here (TE = 0)
        for ad in (CNT & 0x7FFFFFF, HASHA & 0x7FFFFFF, WVAR & 0x7FFFFFF, (WVAR & 0x7FFFFFF) + 8):
            inits[ad] = 0
        for ad, v in inits.items():
            a.setr64(1, v); a.setr(2, ad); a.st(64, 1, 2)
        a.setr64(1, (seed * 0x9E3779B97F4A7C15 + 12345) & 0xFFFFFFFFFFFFFFFF | 1)
        a.setr64(3, 6364136223846793005); a.setr(2, DATA_PA); a.setr(4, 0)
        a.label('di')
        a.mul(1, 1, 3, sgn=False); a.inc(1); a.st_rr(64, 1, 2, 4); a.addi_s(4, 4, 8)
        a.fbr_imm32('ne', 4, SPAN, 'di')
    a.leapc(1, 'h'); a.setr(2, 0x80000000); a.alu('or', 1, 1, 2)
    a.mtsr('evec', 1); a.mtsr('tlbvec', 1)
    a.setr(1, KSP); a.setsp(1)
    _tlb_set(a, 0, 0x0000, 0x000, TLB_RWX, g=1)
    if walk:
        a.setr(1, PTB); a.mtsr('tlb_ptb', 1)
        a.setr(1, 1); a.mtsr('tlb_walk', 1)
    a.setr(1, 0x200); a.mtsr('status', 1)
    a.leapc(1, 'kz'); a.setr(2, 0x80000000); a.alu('or', 1, 1, 2); a.jmpr(1)
    a.label('kz')
    a.tlbinv()
    if remap and ref:
        a.setr(10, CA, sgn=True)
    a.setr(1, usp_va); a.mtsr('usp', 1)
    a.leapc(2, 'u'); a.alui('and', 2, 2, 0xFFFF, sgn=False)
    a.setr(3, CVP << 16); a.alu('or', 2, 2, 3)
    a.setr64(3, (1 << 44) | ((1 if irq else 0) << 39)); a.alu('or', 2, 2, 3)
    a.push(2); a.iret()
    a.label('u')
    iretk = None
    if fetch:
        iretk = [lambda: 0x80000000 | a.labels['h_after'], lambda: 0x80000000 | a.labels['h_iret'],
                 lambda: 0x80000000 | (a.labels['h_after'] + 16), lambda: 0x80000000 | a.labels['h'],
                 0xF0FD0000, 0x88000000, 0xC0000000, 0x7FFFFFFC, 0xFFFFFFFC, 0x80000000,
                 (CVP << 16) | 0x101, (CVP << 16) | 0x202, (CVP << 16) | 0x303, 0x80000021]
    stats = gen(seed, a, data_va, perm, ref, probes=probes, fvp=fvp, fperm=fperm, mark=mark, iretk=iretk, strad=strad)
    cpages = [CVP] + (fvp or [])
    # ---------------------------------------------------------- the handler
    a.label('h')
    a.mtsr('scratch0', 13); a.mtsr('scratch1', 14)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, 'h_nh')
    if soc:
        # the end: r13 = a checksum of the data window, r12 = of the witnesses, r14 = the fault hash
        a.setr(12, 0); a.setr(13, 0)
        a.label('ck')
        a.setr(14, 0x80000000 | DATA_PA); a.alu('add', 14, 14, 12); a.ld(64, 14, 14)
        a.shift_n('rol', 13, 13, 5); a.alu('xor', 13, 13, 14)
        a.addi_s(12, 12, 8)
        a.fbr_imm32('ne', 12, SPAN, 'ck')
        a.setr(12, 0)
        for ad in [0x80000000 | SECRET_PA + 8 * i for i in range(4)] + [0x80000000 | (TOP_PA << 16) | 0xFFF8]:
            a.setr(14, ad); a.ld(64, 14, 14); a.shift_n('rol', 12, 12, 9); a.alu('xor', 12, 12, 14)
        a.setr(14, HASHA); a.ld(64, 14, 14)
        a.halt()
    # the end: copy the data window and the witnesses to the dumped window
    a.setr(12, 0)
    a.label('cp')
    a.setr(13, 0x80000000 | DATA_PA); a.ld_rr(64, 14, 13, 12)
    a.setr(13, 0x80000000 | OUT); a.st_rr(64, 14, 13, 12)
    a.addi_s(12, 12, 8)
    a.fbr_imm32('ne', 12, SPAN, 'cp')
    a.setr(12, 0x80000000 | OUT)
    a.setr(13, HASHA); a.ld(64, 14, 13); a.st(64, 14, 12, SPAN, mode='long')
    for i in range(4):
        a.setr(13, 0x80000000 | SECRET_PA + 8 * i); a.ld(64, 14, 13); a.st(64, 14, 12, SPAN + 8 + 8 * i, mode='long')
    a.setr(13, 0x80000000 | (TOP_PA << 16) | 0xFFF8); a.ld(64, 14, 13); a.st(64, 14, 12, SPAN + 0x28, mode='long')
    a.setr(13, 0xF0FD0000); a.ld(64, 14, 13); a.st(64, 14, 12, SPAN + 0x30, mode='long')
    a.setr(13, 0xF0FD0008); a.ld(64, 14, 13); a.st(64, 14, 12, SPAN + 0x38, mode='long')
    a.halt()
    a.label('h_nh')
    a.mfsr(13, 'ecause'); a.alui('and', 13, 13, 0xFF, sgn=False)
    a.fbr('eq', 13, 4, 'h_sys', imm=True)
    a.addi_s(14, 13, -11); a.fbr('ule', 14, 2, 'h_miss', imm=True)
    a.addi_s(14, 13, -7); a.fbr('ule', 14, 1, 'h_skip', imm=True)
    a.addi_s(14, 13, -9); a.fbr('eq', 14, 0, 'h_fret', imm=True)
    a.addi_s(14, 13, -10); a.fbr('eq', 14, 0, 'h_fret', imm=True)       # MISALIGNED_FETCH
    a.addi_s(14, 13, -3); a.fbr('eq', 14, 0, 'h_trap', imm=True)
    a.label('h_bad')
    a.setr64(1, 0xBADBADBADBAD0000); a.mfsr(2, 'ecause'); a.mfsr(3, 'eaddr')
    a.halt()
    a.label('h_skip')
    a.setr(12, HASHA); a.ld(64, 13, 12)
    a.shift_n('rol', 13, 13, 7)
    a.mfsr(14, 'ecause'); a.alu('xor', 13, 13, 14)
    a.shift_n('rol', 13, 13, 9)
    a.mfsr(14, 'eaddr'); a.alu('xor', 13, 13, 14)
    a.shift_n('rol', 13, 13, 11)
    a.getsp(14); a.ld(64, 14, 14); a.alu('xor', 13, 13, 14)
    a.st(64, 13, 12)
    a.getsp(13); a.ld(64, 14, 13)
    a.setr(12, 0xFFFF); a.alu('and', 12, 14, 12); a.setr(13, 0x80000000); a.alu('or', 12, 12, 13)
    a.ld(32, 12, 12)
    a.shift_n('shr', 12, 12, 30); a.shift_n('shl', 12, 12, 2)
    a.alu('add', 14, 14, 12)
    a.getsp(13); a.st(64, 14, 13)
    a.setr(12, data_va)
    a.jmp('h_ret')
    a.label('h_fret')                    # a fetch fault at a CALL / RET target: return to the caller
    a.setr(12, HASHA); a.ld(64, 13, 12)
    a.shift_n('rol', 13, 13, 7)
    a.mfsr(14, 'ecause'); a.alu('xor', 13, 13, 14)
    a.shift_n('rol', 13, 13, 9)
    a.mfsr(14, 'eaddr'); a.alu('xor', 13, 13, 14)
    a.shift_n('rol', 13, 13, 11)
    a.getsp(14); a.ld(64, 14, 14); a.alu('xor', 13, 13, 14)
    a.st(64, 13, 12)
    a.mfsr(13, 'usp')
    a.shift_n('shr', 14, 13, 16); a.setr(12, SVP)
    a.fbr('eq', 14, 12, 'fr5'); a.setr(12, 0x80040000); a.jmp('fr6')
    a.label('fr5'); a.setr(12, 0x80050000)
    a.label('fr6')
    a.setr(14, 0xFFFF); a.alu('and', 14, 13, 14); a.alu('or', 12, 12, 14)
    a.ld(64, 14, 12)
    a.addi_s(13, 13, 8); a.mtsr('usp', 13)
    a.getsp(13); a.ld(64, 12, 13)
    a.shift_n('shr', 12, 12, 32); a.shift_n('shl', 12, 12, 32); a.alu('or', 12, 12, 14); a.st(64, 12, 13)
    a.setr(12, data_va)
    a.jmp('h_ret')
    a.label('h_miss')
    a.tlbp(); a.mfsr(13, 'tlb_index')
    a.shift_n('shr', 13, 13, 31); a.fbr('ne', 13, 1, 'h_bad', imm=True)
    a.mfsr(13, 'tlb_hi'); a.shift_n('shr', 13, 13, 16)
    for i, (v, (p, pm)) in enumerate(maps.items()):
        a.setr(14, v); a.fbr('ne', 13, 14, f'h_m{i}')
        if pm == 'absent':
            # (a data miss on the absent page: step over; a fetch miss there cannot happen)
            a.jmp('h_skip')
        elif pm == 'fabsent':
            a.jmp('h_fret')
        elif remap and not ref and v in cpages:
            a.setr(12, WVAR); a.ld(64, 14, 12)
            a.setr(13, (0 << 16) | PERMS[pm]); a.fbr('eq', 14, 0, 'h_wr', imm=True)
            a.setr(13, (FB << 16) | PERMS[pm]); a.jmp('h_wr')
        else:
            a.setr(13, (p << 16) | PERMS[pm]); a.jmp('h_wr')
        a.label(f'h_m{i}')
    a.shift_n('shl', 13, 13, 16); a.alui('or', 13, 13, TLB_RWX, sgn=False)
    a.label('h_wr')
    a.mtsr('tlb_lo', 13)
    a.setr(13, CNT); a.ld(64, 14, 13); a.inc(14)
    a.alui('and', 14, 14, K - 1, sgn=False); a.st(64, 14, 13)
    a.mtsr('tlb_index', 14); a.tlbwi()
    a.jmp('h_ret')
    a.label('h_sys')
    if remap:
        a.setr(12, WVAR); a.ld(64, 13, 12); a.alui_s('xor', 13, 13, 1); a.st(64, 13, 12)   # the world toggles
        if ref:
            a.setr(10, CA, sgn=True); a.fbr('eq', 13, 0, 'hs1', imm=True); a.setr(10, CB, sgn=True)
            a.label('hs1')
        elif walk:
            for i, v in enumerate(cpages):
                pm = maps[v][1]
                if pm == 'fabsent':
                    continue
                a.setr(12, 0x80000000 | pte_addr[v])
                a.setr64(14, mkpte(0, pm)); a.fbr('eq', 13, 0, f'hp{i}', imm=True); a.setr64(14, mkpte(FB, pm))
                a.label(f'hp{i}')
                a.st(64, 14, 12)
        # the flush, by one of three methods in turn
        a.setr(12, WVAR + 8); a.ld(64, 14, 12); a.inc(14); a.st(64, 14, 12)
        a.setr(13, 3); a.mod(14, 14, 13, sgn=False)
        if os.environ.get('PF_NOFLUSH') and not ref:
            a.jmp('h_ret')
        a.fbr('eq', 14, 1, 'fl_inv', imm=True)
        a.fbr('eq', 14, 2, 'fl_asid', imm=True)
        for i, v in enumerate(cpages):          # Linux-like: TLBP, and TLBWI V = 0 on a hit
            a.mfsr(13, 'asid'); a.setr(14, v << 16); a.alu('or', 13, 13, 14); a.mtsr('tlb_hi', 13)
            a.tlbp(); a.mfsr(13, 'tlb_index'); a.shift_n('shr', 13, 13, 31)
            a.fbr('ne', 13, 0, f'fp{i}', imm=True)
            a.setr(13, 0); a.mtsr('tlb_lo', 13); a.tlbwi()
            a.label(f'fp{i}')
        a.jmp('h_ret')
        a.label('fl_inv')
        a.tlbinv(); a.jmp('h_ret')
        a.label('fl_asid')
        a.mfsr(13, 'asid'); a.inc(13); a.alui('and', 13, 13, 0xFF, sgn=False)
        a.mtsr('asid', 13)
        a.fbr('ne', 13, 0, 'h_ret', imm=True)
        a.tlbinv()
    else:
        a.mfsr(13, 'asid'); a.inc(13); a.alui('and', 13, 13, 3, sgn=False)
        a.mtsr('asid', 13)
        a.fbr('ne', 13, 0, 'h_ret', imm=True)
        a.tlbinv()
    a.jmp('h_ret')
    a.label('h_trap')                    # TRAP: return to the PC in the user's r13
    a.mfsr(14, 'scratch0')
    a.getsp(13); a.ld(64, 12, 13)
    a.shift_n('shr', 12, 12, 32); a.shift_n('shl', 12, 12, 32); a.alu('or', 12, 12, 14); a.st(64, 12, 13)
    a.label('h_ret')
    a.setr(12, data_va)
    a.mfsr(13, 'scratch0'); a.mfsr(14, 'scratch1')
    a.label('h_iret')
    a.iret()
    a.label('h_after')                   # (kernel code right behind the IRET: never run from user mode)
    for i in range(1, 10):
        a.setr64(i, 0xDEAD00000000DEAD + i)
    a.setr(13, 0x80000000 | SECRET_PA); a.st(64, 1, 13)
    a.halt()
    assert a.pc < 0xF000, hex(a.pc)
    # ---------------------------------------------------------------- data
    drnd = random.Random(seed * 31 + 7)
    if not soc:
        for o in range(0, SPAN, 8):
            a.dword(DATA_PA + o, drnd.getrandbits(64) | 1)
        for ad, v in inits.items():
            a.dword(ad, v)
    return a, dict(K=K, CVP=hex(CVP), DVP=hex(DVP), SVP=hex(SVP), perm=perm, fperm=fperm, **stats)


def main_soc(a):
    """the same through tb_soc (real D-cache / hit pipe / L2 / bus)"""
    exe = R.build_soc(None)
    fails = n = 0
    for s in range(a.seed0, a.seed0 + a.seeds):
        usp_pa = USP_PA_BASE + 8 * (s % 4)
        ra, _ = build_prog(s, usp_pa, True, a.walk, False, not a.noprobe, a.fetch, soc=True)
        ta, info = build_prog(s, usp_pa, False, a.walk, False, not a.noprobe, a.fetch, soc=True)
        plus = ['+' + p for p in a.plus]
        rr = R.run_soc(exe, ra, f'pfs_a{s}_{int(a.walk)}', plus=plus)
        rt = R.run_soc(exe, ta, f'pfs_b{s}_{int(a.walk)}', plus=plus)
        ga, gb = R._soc_final(rr['log'])[0], R._soc_final(rt['log'])[0]
        n += 1
        keys = [f'r{i}' for i in list(range(1, 14)) + [15]]
        bad = [k for k in keys if not ga or not gb or ga.get(k) != gb.get(k)]
        ok = bool(ga) and bool(gb) and not bad and 'fatal' not in (rr['log'] + rt['log']).lower() \
            and 'HALTED' in rr['log'] and 'HALTED' in rt['log']
        if ok and (ga['r1'] >> 16) == 0xBADBADBADBAD:
            ok = False
        if not ok:
            fails += 1
            print(f'DIFF seed={s} info={info} {[(k, ga and hex(ga.get(k, 0)), gb and hex(gb.get(k, 0))) for k in bad[:4]]}',
                  flush=True)
            print(rt['log'][-600:])
        elif a.v:
            print(f'same seed={s} info={info} r13={gb["r13"]:#x} hash={gb.get("r14", 0):#x}', flush=True)
    print(f'{fails} failing of {n} soc runs')
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog='run.py permfuzz')
    ap.add_argument('--seed0', type=int, default=0)
    ap.add_argument('--seeds', type=int, default=20)
    ap.add_argument('--lats', type=int, nargs='*', default=[1, 3, 7])
    ap.add_argument('--hits', nargs='*', default=['none', '70'])
    ap.add_argument('--irq', action='store_true')
    ap.add_argument('--walk', action='store_true')
    ap.add_argument('--noprobe', action='store_true')
    ap.add_argument('--fetch', action='store_true')
    ap.add_argument('--remap', action='store_true')
    ap.add_argument('--soc', action='store_true')
    ap.add_argument('--plus', nargs='*', default=[])
    ap.add_argument('-v', action='store_true')
    ap.add_argument('--selfcheck', action='store_true',
                    help='PF_NOFLUSH self-check: --fetch --remap with the flush removed must differ')
    a = ap.parse_args(argv)
    if a.selfcheck:
        # the fuzzer must be able to fail: without the flush most seeds differ
        # (the review: 5 of 6); require at least half of them
        os.environ['PF_NOFLUSH'] = '1'
        a.fetch = a.remap = True
        a.lats, a.hits = a.lats[:1], a.hits[:1]
        import io, contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = _main(a)
        os.environ.pop('PF_NOFLUSH')
        m = re.search(r'(\d+) failing of (\d+) runs', buf.getvalue())
        nf, n = (int(m.group(1)), int(m.group(2))) if m else (0, 0)
        ok = rc == 1 and n and 2 * nf >= n
        print(f'permfuzz self-check (PF_NOFLUSH): {nf} of {n} runs differ -> {"pass" if ok else "FAIL: the fuzzer cannot see a missing flush"}')
        return 0 if ok else 1
    return _main(a)


def _main(a):
    if a.soc:
        return main_soc(a)
    exe = R.build(None)
    fails = n = nflt = 0
    for s in range(a.seed0, a.seed0 + a.seeds):
        usp_pa = USP_PA_BASE + 8 * (s % 4)
        hashes = set()
        for lat in a.lats:
            for hit in a.hits:
                plus = [f'+LAT={lat}'] + ([f'+HIT={hit}'] if hit != 'none' else []) + \
                       ([f'+IRQ_PERIOD={301 + s % 97}'] if a.irq else []) + ['+' + p for p in a.plus]
                tag = f'{s}_{lat}_{hit}_{int(a.walk)}{int(a.irq)}{int(a.fetch)}{int(a.remap)}'
                if a.remap:
                    ra, _ = build_prog(s, usp_pa, True, a.walk, a.irq, not a.noprobe, a.fetch, True, 'reg')
                    ta, info = build_prog(s, usp_pa, False, a.walk, a.irq, not a.noprobe, a.fetch, True, CA)
                    tb, _ = build_prog(s, usp_pa, False, a.walk, a.irq, not a.noprobe, a.fetch, True, CB)
                    wa, wb = ta.assemble(), tb.assemble()
                    assert len(wa) == len(wb)
                    wb += [0] * (len(wb) % 2)
                    base = (FB << 16) | ta.base
                    assert base % 8 == 0
                    for i in range(0, len(wb), 2):
                        ta.dword(base + 4 * i, (wb[i + 1] << 32) | wb[i])
                else:
                    ra, _ = build_prog(s, usp_pa, True, a.walk, a.irq, not a.noprobe, a.fetch)
                    ta, info = build_prog(s, usp_pa, False, a.walk, a.irq, not a.noprobe, a.fetch)
                sa, _ = R.run(exe, ra, f'pfa{tag}', plus)
                sb, _ = R.run(exe, ta, f'pfb{tag}', plus)
                n += 1
                nflt += int(sb.get('exc_n', 0))
                hk = f'm{OUT + SPAN:08x}'
                keys = [f'r{i}' for i in list(range(1, 10)) + ([] if a.remap else [10]) + [11, 15]] + \
                    sorted({k for k in list(sa) + list(sb) if re.fullmatch(r'm[0-9a-f]{8}', k)
                            and k not in ('m00180000', hk)}) + \
                    (['mmio_rd', 'mmio_wr'] if not a.irq else [])
                bad = [k for k in keys if sa.get(k) != sb.get(k)]
                hashes.add(sb.get(hk))
                ok = sa.get('end') == 'park' and sb.get('end') == 'park' and sb.get('kind') == '0' and \
                    sa.get('kind') == '0' and not bad
                if 'fatal' in (sa['log'] + sb['log']).lower():
                    ok = False
                if not ok:
                    fails += 1
                    print(f'DIFF seed={s} lat={lat} hit={hit} info={info} exc={sa.get("exc_n")}/{sb.get("exc_n")} '
                          f'end {sa.get("end")}/{sb.get("end")} kind {sa.get("kind")}/{sb.get("kind")} '
                          f'{[(k, sa.get(k), sb.get(k)) for k in bad[:4]]}', flush=True)
                elif a.v:
                    print(f'same seed={s} lat={lat} hit={hit} info={info} exc={sa.get("exc_n")}/{sb.get("exc_n")} '
                          f'walks={sb.get("wk_n")} cyc={sb.get("cycles")}', flush=True)
        if len(hashes) != 1:
            fails += 1
            print(f'HASH seed={s}: fault-log hash differs across timings: {hashes}', flush=True)
    print(f'{fails} failing of {n} runs; {nflt} exception entries in the test runs')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

"""HW_PERF_PLAN §9 item 4 (Fable review, exceptions finding 1): precise EI
right behind the MMIO store that makes the interrupt pending, through
tb_soc (the real interrupt controller and MMIO latency).  Adapted from the
reviewers' ei_soc.py (review-tests/fable-exc).  Run as run.py socei.

Program: DI; the source's other enables set; wait until the source is
pending; then a subroutine `[enabling store] nop*k EI(1) nop*j DI` runs
twice (pass 1 with a non-enabling store, so pass 2 is warm in the
I-cache); the handler (at H) counts entries and logs frame PCs at LOG.
`pad` NOPs before the subroutine vary the alignment.  A cell is
`entries@framePC-EI`; precise = 1@+4 (CPU_ARCHITECTURE §13.1: taken at the
boundary right after the EI).

Sources (the enabling store):
  mask   INT_MASK 0 -> 1        (the timer pending, INT_VEC0 set)
  vec0   INT_VEC0 0 -> H        (the timer pending, INT_MASK[0] set)
  vec1   INT_VEC1 0 -> H        (the blitter DONE with IRQ_EN, INT_MASK[1])
  vec2   INT_VEC2 0 -> H        (SD CMD_DONE with SDM_IRQEN, INT2_EN = 2, INT_MASK[2])
  vec3   INT_VEC3 0 -> H        (VSYNC_PENDING with VSYNC_IRQ_EN, INT_MASK[3])
  int2en INT2_EN 1 -> 3         (SD CMD_DONE pending, INT_VEC2, INT_MASK[2])
  sdirqen SDM_IRQEN 0 -> CMD_DONE (SD CMD_DONE pending, INT2_EN = 2, INT_VEC2,
                               INT_MASK[2]; §9 review MINOR 1: the slowest
                               source, sd_native's registered path + r_eth_irq,
                               whose MMIO write ack the SoC stretches)
  vga    VGA_CTRL VSYNC_IRQ_EN  (VSYNC_PENDING, INT_VEC3, INT_MASK[3])
(The blitter's IRQ_EN is written only together with START, which clears
DONE: that store never makes an interrupt pending itself — DONE comes at
the blit's end, asynchronously — so the blitter source is tested through
INT_VEC1.  The timer is source 0 of mask / vec0.)
`loop`: 2000 x {INT_MASK = 0; nop; INT_MASK = 1; nop*k; EI; nop*j; DI}
with the timer pending (period 300) must take interrupts (it took none
before the fix: an unbounded delay).
"""
import os, re, shutil
from concurrent.futures import ThreadPoolExecutor

IC, VGA, BLT, SD = 0xF00F0000, 0xF0110000, 0xF00E0000, 0xF0000000
LOG = 0x20000          # [0] entries, [8*k] k-th frame PC
H = 0x1000             # the handler
SRCS = ('mask', 'vec0', 'vec1', 'vec2', 'vec3', 'int2en', 'sdirqen', 'vga')


def _run():
    import run
    return run


def handler(s, src):
    s.push(1); s.push(2); s.push(3)
    s.setr(1, LOG); s.ld(64, 2, 1, 0); s.inc(2); s.st(64, 2, 1, 0)
    s.shift_n('shl', 2, 2, 3); s.alu('add', 1, 1, 2)
    s.getsp(3); s.ld(64, 3, 3, 24)
    s.shift_n('shl', 3, 3, 32); s.shift_n('shr', 3, 3, 32); s.st(64, 3, 1, 0)
    # level sources: clear them so they cannot re-fire after the IRET
    if src in ('vga', 'vec3'):
        s.setr(1, VGA); s.setr(2, 2); s.st(64, 2, 1, 0x18); s.setr(2, 8); s.st(64, 2, 1, 0x00)
    elif src == 'vec1':
        s.setr(1, BLT); s.setr(2, 2); s.st(64, 2, 1, 0x08)
    elif src in ('vec2', 'int2en', 'sdirqen'):
        s.setr(1, SD); s.setr(2, 0); s.st(64, 2, 1, 0x130)
    s.pop(3); s.pop(2); s.pop(1)
    s.iret()


def _blit_setup(a, w):
    """an FILL of w x 1 pixels at 5 MiB"""
    a.setr(13, BLT)
    for off, v in ((0x10, 0x500000), (0x18, 1280), (0x30, w), (0x38, 1), (0x40, 0x1234)):
        a.setr(1, v); a.st(64, 1, 13, off)


def prog(src, k, j, pad, loop=0):
    from kasm import Asm
    from tests import _place
    a = Asm()
    a.setr(1, 0x08000000); a.setsp(1)
    a.di(9)
    a.setr(15, IC); a.setr(1, 0)
    a.setr(2, LOG); a.st(64, 1, 2, 0)
    a.st(64, 1, 15, 0x00)                                  # INT_MASK = 0
    for off in (0x10, 0x18, 0x20, 0x28):
        a.st(64, 1, 15, off)                               # INT_VEC0..3 = 0
    a.setr(3, H)
    # ---- the source made pending, everything but the enabling store set
    if src in ('mask', 'vec0'):
        if src == 'mask':
            a.st(64, 3, 15, 0x10)                          # INT_VEC0 = H (mask still 0)
        else:
            a.setr(1, 1); a.st(64, 1, 15, 0x00)            # INT_MASK = 1 (vector still 0)
        a.setr(1, 300); a.st(64, 1, 15, 0x30)              # TIMER_PERIOD = 300 (counter reset)
        a.label('pw'); a.ld(64, 1, 15, 0x08); a.setr(2, 1); a.alu('and', 1, 1, 2)
        a.fbr('eq', 1, 0, 'pw', imm=True)                  # until the timer is pending
    elif src in ('vga', 'vec3'):
        a.setr(14, VGA)
        if src == 'vga':
            a.st(64, 3, 15, 0x28)                          # INT_VEC3 = H
        else:
            a.setr(1, 24); a.st(64, 1, 14, 0x00)           # VSYNC_IRQ_EN (+ TEST_PATTERN)
        a.setr(1, 8); a.st(64, 1, 15, 0x00)                # INT_MASK = 8
        a.label('pw'); a.ld(64, 1, 14, 0x18); a.setr(2, 2); a.alu('and', 1, 1, 2)
        a.fbr('eq', 1, 0, 'pw', imm=True)                  # until VSYNC_PENDING
    elif src == 'vec1':
        _blit_setup(a, 64)
        a.setr(1, 0x11); a.st(64, 1, 13, 0x00)             # START + IRQ_EN, FILL
        a.label('pw'); a.ld(64, 1, 13, 0x08); a.setr(2, 2); a.alu('and', 1, 1, 2)
        a.fbr('eq', 1, 0, 'pw', imm=True)                  # until DONE
        a.setr(1, 2); a.st(64, 1, 15, 0x00)                # INT_MASK = 2
    elif src in ('vec2', 'int2en', 'sdirqen'):
        a.setr(14, SD)
        a.setr(1, (1 << 18) | (1 << 17) | 1); a.st(64, 1, 14, 0)     # SD mode, power, CLK
        a.setr(1, 0xFFFFFFFF); a.st(64, 1, 14, 0x128)                # clear the status
        a.setr(1, 0x10000); a.st(64, 1, 14, 0x108)                   # ARG (RCA)
        a.setr(1, (1 << 8) | 13); a.st(64, 1, 14, 0x100)             # CMD13, R1
        a.label('pw'); a.ld(64, 1, 14, 0x128); a.setr(2, 1 << 8); a.alu('and', 1, 1, 2)
        a.fbr('eq', 1, 0, 'pw', imm=True)                  # until CMD_DONE
        if src != 'sdirqen':
            a.setr(1, 1 << 8); a.st(64, 1, 14, 0x130)      # SDM_IRQEN = CMD_DONE: the SD level is up
        else:
            # SDM_IRQEN = 0 and an MMIO store right behind it: its stretched
            # ack must leave the low-ready cycle before the next one's
            a.setr(1, 0); a.setr(2, 0x10000)
            a.st(64, 1, 14, 0x130); a.st(64, 2, 14, 0x108)
        if src != 'int2en':
            a.setr(1, 2); a.st(64, 1, 15, 0x50)            # INT2_EN = SD
        if src != 'vec2':
            a.st(64, 3, 15, 0x20)                          # INT_VEC2 = H
        a.setr(1, 4); a.st(64, 1, 15, 0x00)                # INT_MASK = 4
    # r4 = the enabling value, r5 = the address it goes to; r6 = pass 1's (disabling) value
    if src == 'mask':
        a.mov(5, 15); a.setr(4, 1); a.setr(6, 0)
    elif src.startswith('vec'):
        a.setr(5, IC + 0x10 + 8 * int(src[3])); a.setr(4, H); a.setr(6, 0)
    elif src == 'int2en':
        a.setr(5, IC + 0x50); a.setr(4, 3); a.setr(6, 1)
    elif src == 'sdirqen':
        a.setr(5, SD + 0x130); a.setr(4, 1 << 8); a.setr(6, 0)
    elif src == 'vga':
        a.setr(5, VGA); a.setr(4, 24); a.setr(6, 8)
    a.setr(1, 1)
    if loop:
        a.setr(8, loop)
        for _ in range(pad):
            a.nop()
        a.label('lp')
        a.st(64, 6, 5, 0); a.nop()
        a.st(64, 4, 5, 0)
        for _ in range(k):
            a.nop()
        a.label('ei'); a.ei(1)
        for _ in range(j):
            a.nop()
        a.di(2)
        a.dec(8); a.fbr('ne', 8, 0, 'lp', imm=True)
        a.setr(3, LOG); a.ld(64, 10, 3, 0)                 # r10 = entries during the loop
        a.ei(1)
        for _ in range(12):
            a.nop()
        a.di(2)
        a.ld(64, 11, 3, 0)                                 # r11 = entries after a control EI
        a.halt()
    else:
        a.mov(7, 6)                                        # pass 1: the disabling value
        a.setr(8, 2)
        a.label('again')
        a.call('seq')
        a.mov(7, 4)                                        # pass 2: the enabling value
        a.dec(8)
        a.fbr('eq', 8, 0, 'out', imm=True)
        a.setr(12, LOG); a.ld(64, 13, 12, 0)               # r13 = entries before pass 2 (want 0)
        a.jmp('again')
        a.label('out')
        a.setr(2, LOG); a.ld(64, 10, 2, 0); a.ld(64, 11, 2, 8)
        a.halt()
        for _ in range(pad):
            a.nop()
        a.label('seq')
        a.st(64, 7, 5, 0)
        for _ in range(k):
            a.nop()
        a.label('ei'); a.ei(1)
        for _ in range(j):
            a.nop()
        a.di(2)
        a.ret()
    assert a.pc < H, hex(a.pc)
    _place(a, H, lambda s: handler(s, src))
    return a


def _go(exe, a, tag, plus=()):
    run = _run()
    r = run.run_soc(exe, a, tag, timeout=1800, gate=False, plus=list(plus))
    shutil.rmtree(os.path.join(run.WORK, 'socruns', tag), ignore_errors=True)
    m = re.search(r'TB_SOC: REGS(.*)', r['log'])
    if not m:
        return None, r['log']
    return {k: int(v, 16) for k, v in re.findall(r'(r\d+)=([0-9a-f]{16})', m.group(1))}, r['log']


def cmd_socei(a):
    run = _run()
    exe = run.build_soc(a.rev, tb_wt=bool(a.rev))
    srcs = a.srcs or SRCS
    jobs = []
    for src in srcs:
        pads = range(0, 8) if src in ('mask', 'vec0') or a.full else range(0, 4)
        for k in a.k:
            for j in a.j:
                for pad in pads:
                    jobs.append((src, k, j, pad))

    def one(job):
        src, k, j, pad = job
        p = prog(src, k, j, pad)
        plus = ['+VGA_V0=478'] if src in ('vga', 'vec3') else []
        rg, log = _go(exe, p, f'ei_{src}_{k}_{j}_{pad}', plus)
        if rg is None:
            return job, 'NOHALT', False
        ei = p.labels['ei']
        cell = f'{rg["r10"]}@{(rg["r11"] - ei):+d}' if rg['r10'] else '0'
        if rg['r13']:
            cell += '(!pass1)'
        return job, cell, cell == '1@+4'

    with ThreadPoolExecutor(a.jobs) as ex:
        res = {job: (cell, ok) for job, cell, ok in ex.map(one, jobs)}
    fails = sum(not ok for _, ok in res.values())
    print('src    k j | pad 0..  (entries@framePC-EI; precise = 1@+4)')
    for src in srcs:
        for k in a.k:
            for j in a.j:
                row = [res[(s, kk, jj, p)][0] for (s, kk, jj, p) in jobs if s == src and kk == k and jj == j]
                if row:
                    print(f'{src:6} {k} {j} | ' + ' '.join(f'{x:>7}' for x in row), flush=True)
    # the loop: it must take interrupts at all
    lres = {}
    if a.loop:
        def lone(kj):
            k, j = kj
            p = prog('mask', k, j, 0, loop=2000)
            rg, _ = _go(exe, p, f'eiloop_{k}_{j}')
            return kj, rg
        with ThreadPoolExecutor(a.jobs) as ex:
            lres = dict(ex.map(lone, ((0, 0), (1, 0), (0, 2), (0, 3), (2, 0))))
        ref = lres[(2, 0)] and lres[(2, 0)]['r10']
        for (k, j), rg in lres.items():
            n = rg and rg['r10']
            ok = bool(n) and ref and n >= ref // 2
            fails += not ok
            print(f'{"pass" if ok else "FAIL"} loop x2000 store;nop*{k};EI;nop*{j};DI: entries during the loop = {n} '
                  f'(store;nop*2;EI;DI: {ref}), after a control EI = {rg and rg["r11"]}')
    print(f'socei: {fails} failing of {len(jobs) + len(lres)}')
    return 1 if fails else 0

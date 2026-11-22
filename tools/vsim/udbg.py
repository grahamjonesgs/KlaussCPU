"""MMU_PLAN.md §13: the user single step and watchpoint fuzzer (run.py
udbgfuzz) and its reference model.

A random program (randprog) runs in user mode behind a fixed supervisor
prologue that reads its whole configuration from data dwords (so every
variant of a seed is the same code at the same addresses):
  CFG + 0 / 8 / 16 / 24   WP0_ADDR, WP0_CTRL, WP1_ADDR, WP1_CTRL
  CFG + 32                 the IRET frame's high bits (PM, SS, INT_MASK)
The handler logs every STEP / WATCH entry and the final user HALT
(PRIVILEGED) as three dword stores {ECAUSE, EADDR, frame} at LOG + 24 i,
which the retire trace shows (wr= records); other exceptions (the
injected SYSCALL / TRAP / illegal / privileged words) are stepped over.

Checks, per seed and variant:
- the user retire lines (pc in the user code, the whole register file,
  SP, flags, the store record) equal the plain run's (no WP, no step);
- the log equals the reference model's prediction, computed in Python
  from the run's own user retire lines: which accesses each instruction
  makes (decoded from its words, with the registers / SP before it — the
  previous user line), the WP rule (§13.3), one STEP per retired user
  instruction when stepping, WATCH (+ SSTEP) where a WP hits, EADDR as
  specified, and the frame PC = the next instruction (after skipping the
  injected words, which fault and are stepped over by the handler);
- every simulator assertion holds (the core's §13 invariants).
"""
import random

from kasm import Asm

CFG = 0x300000
LOG = 0x310000
USP = 0x07F80000
HALT_W = 0x6C010000
DATA = 0x10000


def sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


# ---------------------------------------------------------------- the model
def accesses(words, pc, regs, sp, after_regs, after_sp):
    """The data accesses of the instruction at pc: a list of
    (dw, bytes, is_store, raw_ea, first), dw = EA[31:3].  regs / sp: the
    state before it; after_*: after (CALL taken, SC success)."""
    w0 = words(pc)
    ln = (w0 >> 30) & 3
    cls = (w0 >> 26) & 0xF
    rd, rs1, rs2 = (w0 >> 8) & 0xF, (w0 >> 4) & 0xF, w0 & 0xF
    r = lambda i: regs[i] & 0xFFFFFFFF
    out = []
    if cls in (6, 7) and ln in (1, 2):
        msz = (w0 >> 24) & 3
        sgn = (w0 >> 23) & 1
        mode = (w0 >> 21) & 3
        st = cls == 7
        if st and w0 & 0xFFFFFF0F == 0x5F002000:            # DCBZ [rs1]
            ea = r(rs1)
            base = ea & ~31
            return [((base >> 3) + k, 0xFF, True, ea, True) for k in range(4)]
        lrsc = mode == 0 and (w0 >> 12) & 1
        if mode == 0:
            ea = r(rs1)
        elif mode == 1:
            ea = (r(rs1) + (sx(w0 >> 12, 8) << msz if ln == 1 else words(pc + 4))) & 0xFFFFFFFF
        elif mode == 2:
            ea = words(pc + 4)
        else:
            ea = (r(rs1) + r(rs2)) & 0xFFFFFFFF
        if lrsc and st and (after_regs[rd] & 1):
            return []                                          # a failing SC stores nothing
        e = ea & 7
        if not st and msz == 2 and not sgn and mode == 0 and not lrsc:   # MEMGET32
            m = 0xF << e
            out = [(ea >> 3, m & 0xFF, False, ea, True)]
            if m >> 8:
                out.append(((ea >> 3) + 1, m >> 8, False, ea, False))
            return out
        m = {0: 1 << e, 1: 3 << (e & 6), 2: 0xF0 if e & 4 else 0x0F, 3: 0xFF}[msz]
        return [(ea >> 3, m, st, ea, True)]
    if cls == 8 and (w0 >> 25) & 1:                            # CALL forms (link)
        if (after_sp & 0xFFFFFFFF) == ((sp - 8) & 0xFFFFFFFF):
            ea = (sp - 8) & 0xFFFFFFFF
            return [(ea >> 3, 0xFF, True, ea, True)]
        return []
    if cls == 9:
        op = (w0 >> 22) & 0xF
        if op in (0, 1, 8):                                    # PUSH, PUSHV, ENTER
            ea = (sp - 8) & 0xFFFFFFFF
            return [(ea >> 3, 0xFF, True, ea, True)]
        if op in (2, 6):                                       # POP, RET
            return [(sp >> 3, 0xFF, False, sp, True)]
        if op == 9:                                            # LEAVE
            ea = r(15)
            return [(ea >> 3, 0xFF, False, ea, True)]
        if op == 10:                                           # LEAVERET
            ea = r(15)
            return [(ea >> 3, 0xFF, False, ea, True), ((ea >> 3) + 1, 0xFF, False, ea, False)]
    return []


def watch_hit(acc, wps):
    """§13.3 for a user-mode instruction: (hits bitmask, store, eaddr)"""
    hits, eaddr, first_hit, store = 0, None, False, False
    for dw, m, st, raw, first in acc:
        for n, (k, be, rr, ww, uu) in enumerate(wps):
            if (rr and not st) or (ww and st):
                if dw == (k >> 3) & 0x1FFFFFFF and m & be:
                    hits |= 1 << n
                    store = st
                    if first:
                        first_hit = True
                    elif eaddr is None:
                        eaddr = dw << 3
    if not hits:
        return 0, False, None
    return hits, store, (acc[0][3] if first_hit else eaddr)


# ---------------------------------------------------------------- the program
def gen(seed, p_inj=0.0, irq=False, extra=0.0, lrsc=False):
    """randprog `seed` in user mode behind the fixed prologue; p_inj:
    injected exception words; extra: injected DCBZ (and, with lrsc, LR / SC
    of the same value: only where no entry can fall between them)"""
    import randprog
    from tests import _xsetup
    rnd = random.Random(seed * 104729 + 7)
    words_x = [0x6C080000, 0x6C040000, 0x7C000000, 0x6C060000, 0x6C070000, 0x6C401000, 0x6C020000,
               0x6C030000, 0x70000010]

    class UAsm(Asm):
        def __init__(self, *aa, **kk):
            super().__init__(*aa, **kk)
            self.inj = False
            self.injs = set()
            _xsetup(self)
            for i, sr in enumerate(('wp0_addr', 'wp0_ctrl', 'wp1_addr', 'wp1_ctrl')):
                self.ld_abs(64, 1, CFG + 8 * i)
                self.mtsr(sr, 1)
            self.setr(1, LOG); self.mtsr('scratch0', 1)
            self.setr(1, USP); self.mtsr('usp', 1)
            self.leapc(2, 'u')
            self.ld_abs(64, 3, CFG + 32)
            self.alu('or', 2, 2, 3)
            self.push(2)
            self.setr(2, 0); self.setr(3, 0)      # (the variants' frames differ)
            self.iret()
            self.label('u')
            self.ulo = self.pc
            self.inj = True

        def emit(self, *ws):
            if self.inj:
                self.inj = False
                if rnd.random() < p_inj:
                    self.injs.add(super().emit(rnd.choice(words_x)))
                if rnd.random() < extra:
                    k = rnd.random()
                    # (r13 / r14 are randprog's scratch, live between a
                    # setr and its use: saved around the group)
                    self.push(13); self.push(14)
                    if k < 0.5 or not lrsc:
                        self.setr(13, DATA + rnd.randrange(0, 0x800)); self.dcbz(13)
                    else:
                        self.setr(13, DATA + 8 * rnd.randrange(0, 0x100))
                        self.lr(64, 14, 13); self.sc(64, 14, 14, 13)
                    self.pop(14); self.pop(13)
                self.inj = True
            return super().emit(*ws)

    old = randprog.Asm
    randprog.Asm = UAsm
    try:
        a = randprog.gen(seed, irq=irq)
    finally:
        randprog.Asm = old
    a.inj = False
    a.uhi = a.pc
    # the handler: log STEP / WATCH and the user HALT, step over the rest
    a.label('h')
    a.push(13); a.push(14)
    a.mfsr(14, 'scratch0')                                     # log every entry
    a.mfsr(13, 'ecause'); a.st(64, 13, 14, 0)
    a.mfsr(13, 'eaddr');  a.st(64, 13, 14, 8)
    a.getsp(13); a.ld(64, 13, 13, 16); a.st(64, 13, 14, 16)
    a.addi_s(14, 14, 24); a.mtsr('scratch0', 14)
    a.mfsr(13, 'ecause'); a.alui('and', 13, 13, 0xFF, sgn=False)
    a.addi_s(14, 13, -14)
    a.fbr('ule', 14, 1, 'h_ret', imm=True)                     # 14 / 15: return
    a.fbr('eq', 13, 4, 'h_ret', imm=True)                      # SYSCALL: frame PC is already next
    a.fbr('ne', 13, 2, 'h_skip', imm=True)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, 'h_skip')
    a.halt()                                                   # the user HALT: done
    a.label('h_skip')
    a.getsp(13); a.ld(64, 14, 13, 16); a.addi_s(14, 14, 4); a.st(64, 14, 13, 16)
    a.label('h_ret')
    a.pop(14); a.pop(13)
    a.iret()
    return a


def configure(a, wps=None, ss=False, mask=0):
    """write the data dwords: wps = [(addr, be, r, w, u)] x 2"""
    wps = wps or [(0, 0, 0, 0, 0), (0, 0, 0, 0, 0)]
    for i, (k, be, rr, ww, uu) in enumerate(wps):
        a.data[CFG + 16 * i] = k & 0xFFFFFFFF
        a.data[CFG + 16 * i + 8] = be | rr << 8 | ww << 9 | uu << 10
    a.data[CFG + 32] = 1 << 44 | int(ss) << 45 | mask << 39


# ---------------------------------------------------------------- the checker
def parse_trace(path):
    out = []
    for line in open(path):
        f = line.split()
        d = dict(t.split('=', 1) for t in f if '=' in t and not t.startswith('t='))
        regs = [int(x, 16) for x in f if len(x) == 16 and '=' not in x]
        out.append({'pc': int(d['pc'], 16), 'op': int(d['op'], 16), 'regs': regs,
                    'sp': int(d['sp'], 16), 'wr': d.get('wr'), 'line': ' '.join(t for t in f if not t.startswith('t='))})
    return out


STATS = {}


def check(a, tr_plain, tr_run, wps, ss):
    """returns a list of problems"""
    bad = []
    img = {}
    for i, w in enumerate(a.assemble()):
        img[a.base + 4 * i] = w
    words = lambda p: img.get(p & 0xFFFFFFFF, 0)
    uin = lambda p: a.ulo <= p < a.uhi
    up = [t for t in tr_plain if uin(t['pc'])]
    ur = [t for t in tr_run if uin(t['pc'])]
    if [t['line'] for t in up] != [t['line'] for t in ur]:
        n = next((i for i, (x, y) in enumerate(zip(up, ur)) if x['line'] != y['line']), min(len(up), len(ur)))
        bad.append(f'user retire lines differ at #{n} of {len(up)}/{len(ur)}: '
                   f'{up[n]["line"][:60] if n < len(up) else "-"} / {ur[n]["line"][:60] if n < len(ur) else "-"}')
        return bad
    # the handler's log, from the run's store records
    log = []
    cur = []
    for t in tr_run:
        if t['wr'] and LOG <= int(t['wr'].split('/')[0], 16) < LOG + 0x10000:
            cur.append(int(t['wr'].split('/')[2], 16))
            if len(cur) == 3:
                log.append(tuple(cur)); cur = []
    if not log or (log[-1][0] & 0xFF) != 2 or log[-1][1] != HALT_W:
        return bad + [f'no final HALT entry in the log ({log[-1:] if log else "empty"})']
    halt_pc = log[-1][2] & 0xFFFFFFFF
    # the model
    pre = None
    for t in tr_run:
        if uin(t['pc']):
            break
        pre = t
    exp = []
    for i, t in enumerate(ur):
        acc = accesses(words, t['pc'], pre['regs'], pre['sp'], t['regs'], t['sp'])
        hits, st, ea = watch_hit(acc, wps) if wps else (0, False, None)
        if hits:
            w0 = words(t['pc'])
            kind = ('dcbz' if w0 & 0xFFFFFF0F == 0x5F002000 else 'stack' if (w0 >> 26) & 0xF in (8, 9)
                    else 'lrsc' if (w0 >> 26) & 0xF in (6, 7) and not (w0 >> 21) & 3 and (w0 >> 12) & 1
                    else 'ld32' if len(acc) == 2 and (w0 >> 26) & 0xF == 6 else 'mem')
            STATS[kind] = STATS.get(kind, 0) + 1
            STATS['both'] = STATS.get('both', 0) + (hits == 3)
            STATS['dw2'] = STATS.get('dw2', 0) + (ea != acc[0][3])
            STATS['st'] = STATS.get('st', 0) + st
        nxt = ur[i + 1]['pc'] if i + 1 < len(ur) else halt_pc
        if hits:
            exp.append((15 | hits << 9 | st << 11 | int(ss) << 12, ea, nxt, t['pc']))
        elif ss:
            exp.append((14 | 1 << 12, t['pc'], nxt, t['pc']))
        pre = t
    # (the other entries: faults, stepped over by the handler, and
    # SYSCALL; a STEP / WATCH frame PC may be such an instruction)
    full = log
    idx = [i for i, g in enumerate(full) if (g[0] & 0xFF) in (14, 15)]
    log = [full[i] for i in idx]
    if len(exp) != len(log):
        bad.append(f'{len(log)} log entries, {len(exp)} expected')
    for i, (e, g) in enumerate(zip(exp, log)):
        cause, ea, nxt, at = e
        fr = g[2]
        f = fr & 0xFFFFFFFF
        j = idx[i] + 1
        while j < len(full) and (full[j][0] & 0xFF) not in (14, 15) and f != nxt:
            fc, ff = full[j][0] & 0xFF, full[j][2] & 0xFFFFFFFF
            if fc == 4 and ff == f + 4:
                f = ff
            elif fc != 4 and ff == f:
                f += 4
            else:
                break
            j += 1
        want_ss = int(ss)
        if g[0] != cause or g[1] != ea or f != nxt or (fr >> 44) & 1 != 1 or (fr >> 45) & 1 != want_ss:
            bad.append(f'entry {i} (insn at {at:08x}): got ecause={g[0]:x} eaddr={g[1]:08x} frame={fr:016x}; '
                       f'want ecause={cause:x} eaddr={ea if ea is None else hex(ea)} next={nxt:08x} ss={want_ss}')
            if len(bad) > 4:
                break
    return bad


def rand_wps(rnd, accs, n_hit=0.8):
    """two random watchpoints, mostly on doublewords the program accesses"""
    out = []
    for n in range(2):
        if accs and rnd.random() < n_hit:
            dw = rnd.choice(accs[rnd.choice(sorted(accs))])     # (a kind first: stack ops dominate)
            k = dw << 3
        else:
            k = rnd.choice([DATA + rnd.randrange(0, 0x800), USP - rnd.randrange(0, 0x100), rnd.getrandbits(32)])
        be = rnd.choice([0xFF, 0xFF, 1 << rnd.randrange(8), 0xF << rnd.choice([0, 4]), rnd.getrandbits(8) | 1])
        rw = rnd.choice([(1, 0), (0, 1), (1, 1), (1, 1)])
        out.append((k & ~7, be, rw[0], rw[1], rnd.randrange(2)))
    return out


def plain_accesses(a, tr):
    img = {}
    for i, w in enumerate(a.assemble()):
        img[a.base + 4 * i] = w
    words = lambda p: img.get(p & 0xFFFFFFFF, 0)
    pre, out = None, {}
    for t in tr:
        if a.ulo <= t['pc'] < a.uhi and pre is not None:
            acc = accesses(words, t['pc'], pre['regs'], pre['sp'], t['regs'], t['sp'])
            w0 = words(t['pc'])
            cls = (w0 >> 26) & 0xF
            kind = ('stack' if cls in (8, 9) else 'dcbz' if w0 & 0xFFFFFF0F == 0x5F002000
                    else 'span' if len(acc) == 2 else 'ld' if cls == 6 else 'st')
            for i, x in enumerate(acc):
                out.setdefault(kind + ('2' if i else ''), []).append(x[0])
        pre = t
    return out


def cmd_udbgfuzz(a, build, run):
    """run.py udbgfuzz: see the module docstring"""
    import os
    exe = build(None)
    fails = runs = n_ent = 0
    for s in range(a.seed0, a.seed0 + a.seeds):
        rnd = random.Random(s * 7 + 3)
        irq = a.irq and s % 2 == 0
        p_inj = 0.05 if s % 3 == 0 else 0.0
        lat = (1, 3, 7)[s % 3]
        plus = [f'+LAT={lat}'] + (['+HIT=70'] if s % 4 >= 2 else []) + \
               ([f'+IRQ_PERIOD={97 + s % 60}'] if irq else []) + list(a.plus)
        variants = []
        if a.mode in ('all', 'step'):
            variants.append(('step', False, True))
        if a.mode in ('all', 'watch'):
            variants.append(('watch', True, False))
            variants.append(('wstep', True, True))
        if a.mode != 'step' and not irq and s % 4 == 3:
            variants = [('watch', True, False)]      # (LR / SC: no entry may fall between them)
        lrsc = not irq and not any(v[2] for v in variants)
        prog = gen(s, p_inj=p_inj, irq=irq, extra=0.06, lrsc=lrsc)
        configure(prog, None, ss=False, mask=1 if irq else 0)
        st0, tr0 = run(exe, prog, f'ud0_{s}', plus, trace=True)
        if st0.get('end') != 'park' or 'fatal' in st0['log'].lower():
            fails += 1
            print(f'FAIL seed={s} plain run: end={st0.get("end")} ' + st0['log'][-300:])
            continue
        trp = parse_trace(tr0)
        accs = plain_accesses(prog, trp)
        for name, w, ss in variants:
            wps = rand_wps(rnd, accs) if w else None
            if wps and lrsc:
                # (a watched load's WATCH entry would clear the LR's
                # reservation, and the SC fail where the plain run's
                # succeeds — §13.2: correct, but not comparable)
                wps = [(k, be, 0, 1, u) for k, be, r, w_, u in wps]
            configure(prog, wps, ss=ss, mask=1 if irq else 0)
            st, tr = run(exe, prog, f'ud{name}_{s}', plus, trace=True)
            runs += 1
            bad = []
            if st.get('end') != 'park' or st.get('kind') != '0':
                bad.append(f'end={st.get("end")} kind={st.get("kind")}')
            if 'fatal' in st['log'].lower() or 'assert' in st['log'].lower():
                bad.append('assertion: ' + ' '.join(l for l in st['log'].splitlines() if 'atal' in l)[:300])
            if not bad:
                bad = check(prog, trp, parse_trace(tr), wps, ss)
                n_ent += sum(1 for t in parse_trace(tr) if t['wr'] and LOG <= int(t['wr'].split('/')[0], 16) < LOG + 0x10000) // 3
            os.remove(tr)
            if bad:
                fails += 1
                print(f'FAIL seed={s} {name} lat={lat} irq={irq} inj={len(prog.injs)} wps={wps}: ' + '; '.join(bad))
            elif a.verbose:
                print(f'pass seed={s} {name} lat={lat} irq={irq} inj={len(prog.injs)}')
        os.remove(tr0)
    print(f'{fails} failing of {runs} runs ({n_ent} logged entries); watch hits by kind: {STATS}')
    return 1 if fails else 0


def elf_boot_mem(elf, path, llvm_bin=None):
    """a tools/board ELF (linked at 0x20) as a tb_soc boot image"""
    import os, struct, subprocess
    b = llvm_bin or os.environ.get('KLAUSSCPU_LLVM_BIN', os.path.expanduser('~/Documents/src/klausscpu-llvm/build/bin'))
    binp = path + '.bin'
    subprocess.check_call([os.path.join(b, 'llvm-objcopy'), '-O', 'binary', elf, binp])
    img = b'\0' * 0x20 + open(binp, 'rb').read()
    img += b'\0' * (-len(img) % 8)
    dws = list(struct.unpack(f'<{len(img) // 8}Q', img))
    dws[0] = len(dws) * 8
    with open(path, 'w') as f:
        for v in dws:
            f.write(f'{v:016X}\n')
    return path


def cmd_socustep(a, run_mod):
    """run.py socustep: the board test (tools/board/ustep_test.c, built with
    tools/board/build.sh) on tb_soc — the real D-cache, hit pipe, L2, bus
    and CPU_FEATURES; its UART must end with 'USTEP PASS'"""
    import os, subprocess
    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    elf = a.elf or os.path.join(root, 'tools', 'board', 'out', 'ustep_test.elf')
    if not a.elf:
        subprocess.check_call([os.path.join(root, 'tools', 'board', 'build.sh'), 'ustep_test'],
                              stdout=subprocess.DEVNULL)
    exe = run_mod.build_soc(None)
    fails = 0
    for i, plus in enumerate([[]] + [[p] for p in a.plus]):
        d = os.path.join(run_mod.WORK, 'socruns')
        os.makedirs(d, exist_ok=True)
        mem = elf_boot_mem(elf, os.path.join(d, f'ustep{i}.mem'))
        r = run_mod.run_soc(exe, mem, f'socustep{i}', plus=plus)
        ok = 'USTEP PASS' in r['uart'] and 'FAIL' not in r['uart'] and 'atal' not in r['log']
        print(('pass' if ok else 'FAIL') + f' socustep {plus}')
        if not ok or a.verbose:
            print(r['uart'][-1500:])
            if not ok:
                print('\n'.join(l for l in r['log'].splitlines() if 'atal' in l or 'DEADLOCK' in l)[:600])
        fails += not ok
    print(f'{fails} failing')
    return 1 if fails else 0

"""Directed tests for tools/vsim/run.py.  Each test returns (Asm, expected
final-state dict, plusargs) or a list of them.  Expected keys: r0..r15 (int),
m<addr8hex> (int, dword at addr), kind / end / pc (park), any `key=value`
token the testbench writes to its .out file."""
from kasm import Asm

DATA = 0x10000          # the TB dumps 0x10000..0x10800
ALL = {}


def test(fn):
    ALL[fn.__name__] = fn
    return fn


def m(addr):
    return f'm{addr:08x}'


# ------------------------------------------------------------------ baseline
@test
def smoke():
    a = Asm()
    a.setr(1, 5); a.setr(2, 7)
    a.alu('add', 3, 1, 2)
    a.setr(4, DATA)
    a.st(64, 3, 4)
    a.ld(64, 5, 4)
    a.addi_s(5, 5, 1)
    a.halt()
    return a, {'end': 'park', 'kind': '0', 'r3': 12, 'r5': 13, m(DATA): 12}, []


@test
def loop_fbr():
    a = Asm()
    a.setr(1, 10); a.setr(2, 0)
    a.label('l')
    a.alu('add', 2, 2, 1)
    a.addi_s(1, 1, -1)
    a.fbr('ne', 1, 0, 'l', imm=True)
    a.halt()
    return a, {'end': 'park', 'r2': 55, 'r1': 0}, []


@test
def call_ret():
    a = Asm()
    a.setr(1, 3)
    a.call('f'); a.call('f')
    a.halt()
    a.label('f')
    a.enter(1)
    a.alu('add', 1, 1, 1)
    a.leaveret()
    return a, {'end': 'park', 'r1': 12, 'sp': 0x08000000}, []


# ------------------------------------------------- HW proposal 6: FBR W/imm32
W_PRED = {'eqw': (5, 0), 'new': (5, 1), 'ltw': (6, 0), 'gew': (6, 1),
          'ultw': (7, 0), 'ugew': (7, 1)}


def _br_cases(a, cases):
    """cases: (emit(a, tgt), a_val, b_val, taken).  r1/r2 hold the operands;
    the outcome (1 = fell through, 2 = taken) is stored at DATA+8*i."""
    exp = {}
    a.setr(12, DATA)
    for i, (emit, av, bv, taken) in enumerate(cases):
        a.setr64(1, av); a.setr64(2, bv)
        t, e = f'T{i}', f'E{i}'
        emit(a, t)
        a.setr(3, 1)
        a.jmp(e)
        a.label(t)
        a.setr(3, 2)
        a.label(e)
        a.st(64, 3, 12, 8 * i)
        exp[m(DATA + 8 * i)] = 2 if taken else 1
    a.halt()
    exp.update({'end': 'park', 'kind': '0'})
    return exp


def _w(name, imm=None):
    p, i = W_PRED[name]
    if imm is None:
        return lambda a, t: a.fbr(None, 1, 2, t, pred=p, inv=i)
    return lambda a, t: a.fbr(None, 1, imm, t, imm=True, pred=p, inv=i)


@test
def fbr_w():
    a = Asm()
    cases = [
        (_w('eqw'), 0xFFFFFFFF00000005, 0x5, True),          # low halves equal
        (_w('eqw'), 0x0000000100000005, 0x6, False),
        (_w('new'), 0xFFFFFFFF00000005, 0x5, False),
        (_w('new'), 0x5, 0x7, True),
        (_w('ltw'), 0x0000000180000000, 0x1, True),         # low32 negative
        (_w('ltw'), 0x1, 0x0000000180000000, False),
        (_w('gew'), 0x1, 0x0000000180000000, True),
        (_w('gew'), 0x7FFFFFFF, 0x7FFFFFFF, True),
        (_w('ultw'), 0xFFFF000000000001, 0x2, True),        # 1 <u 2 in 32 bits
        (_w('ultw'), 0x2, 0xFFFF000000000001, False),
        (_w('ugew'), 0x80000000, 0x7FFFFFFF, True),
        (_w('eqw', -1), 0x12345678FFFFFFFF, 0, True),       # simm4 -1, W
        (_w('ltw', 0), 0x00000000FFFFFFFF, 0, True),        # -1 < 0 (32-bit)
        (_w('ultw', 0), 0x00000000FFFFFFFF, 0, False),
        # 64-bit forms unchanged
        (lambda a, t: a.fbr('eq', 1, 2, t), 0xFFFFFFFF00000005, 0x5, False),
        (lambda a, t: a.fbr('lt', 1, 2, t), 0x0000000180000000, 0x1, False),
    ]
    return a, _br_cases(a, cases), []


@test
def fbr_imm32():
    a = Asm()
    cases = [
        (lambda a, t: a.fbr_imm32('eq', 1, 0x12345678, t), 0x12345678, 0, True),
        (lambda a, t: a.fbr_imm32('eq', 1, 0x12345678, t), 0x12345679, 0, False),
        (lambda a, t: a.fbr_imm32('eq', 1, -100000, t), (-100000) & (2**64 - 1), 0, True),
        (lambda a, t: a.fbr_imm32('ne', 1, -100000, t), 0xFFFFFFFE7960, 0, True),
        (lambda a, t: a.fbr_imm32('lt', 1, 1000, t), 999, 0, True),
        (lambda a, t: a.fbr_imm32('lt', 1, 1000, t), 1000, 0, False),
        (lambda a, t: a.fbr_imm32('ult', 1, -1, t), 2**64 - 2, 0, True),  # imm sext to all-ones
        (lambda a, t: a.fbr_imm32('ugt', 1, 0x7FFFFFFF, t), 0x80000000, 0, True),
        # W + imm32
        (lambda a, t: a.fbr_imm32(None, 1, -5, t, pred=5, inv=0), 0x00000123FFFFFFFB, 0, True),
        (lambda a, t: a.fbr_imm32(None, 1, 7, t, pred=6, inv=0), 0x00000001FFFFFFF0, 0, True),
        (lambda a, t: a.fbr_imm32(None, 1, 7, t, pred=7, inv=0), 0x00000001FFFFFFF0, 0, False),
        # backward taken wide branch is exercised by a loop below
    ]
    exp = _br_cases(a, cases)
    return a, exp, []


@test
def fbr_imm32_loop():
    a = Asm()
    a.setr(1, 0); a.setr(2, 0)
    a.label('l')
    a.inc(1)
    a.alu('add', 2, 2, 1)
    a.fbr_imm32('ne', 1, 300, 'l')         # 300 iterations
    a.nop()
    a.halt()
    return [(a, {'end': 'park', 'r1': 300, 'r2': 300 * 301 // 2}, [f'+LAT={l}']) for l in (1, 3, 9)]


# ------------------------------------------------- HW proposal 7a: DI / EI
CNT = 0x180000       # TB IRQ handler counter


@test
def diei_values():
    a = Asm()
    a.di(1)            # was enabled -> 1
    a.di(2)            # back-to-back: sees the in-flight DI -> 0
    a.ei(1)            # restore enabled (r1 = 1)
    a.di(3)            # sees the in-flight EI -> 1
    a.nop(); a.nop(); a.nop(); a.nop()
    a.di(4)            # committed: 0
    a.setr(5, 0)
    a.ei(5)            # enable = 0: stays disabled
    a.di(6)            # -> 0
    a.setr(5, 3)       # bit 0 = 1
    a.ei(5)
    a.nop(); a.nop(); a.nop()
    a.di(7)            # -> 1
    a.ei(7)
    a.alu('add', 8, 1, 3)   # forwarding of the DI result into an ALU op
    a.halt()
    return a, {'end': 'park', 'kind': '0', 'r1': 1, 'r2': 0, 'r3': 1, 'r4': 0,
               'r6': 0, 'r7': 1, 'r8': 2}, []


def _crit(use_di=True):
    """IRQs every ~40 cycles; 300 critical sections each read the handler
    counter at entry and exit (must be equal), with IRQs allowed between."""
    a = Asm()
    a.set_int_mask(1)
    a.setr(10, CNT); a.setr(9, 0); a.setr(11, 300); a.setr(12, DATA)
    a.label('top')
    a.di(1) if use_di else a.nop()
    a.ld(64, 2, 10)
    for i in range(12):                 # ~critical work incl. memory ops
        a.st(64, 2, 12, 8 * i)
        a.inc(2)
    a.setr(6, 8)                        # + a spin, so the window is long
    a.label('spin')
    a.addi_s(6, 6, -1)
    a.fbr('ne', 6, 0, 'spin', imm=True)
    a.ld(64, 3, 10)
    a.ld(64, 4, 10)
    a.ei(1) if use_di else a.nop()
    a.alu('sub', 4, 4, 3)               # 0 unless the handler ran inside
    a.alu('or', 9, 9, 4)
    a.setr(7, 3)
    a.label('gap')                      # interrupts allowed here
    a.addi_s(7, 7, -1)
    a.fbr('ne', 7, 0, 'gap', imm=True)
    a.addi_s(11, 11, -1)
    a.fbr('ne', 11, 0, 'top', imm=True)
    a.ld(64, 8, 10)                     # total IRQs seen
    a.halt()
    return a


@test
def diei_irq_window():
    out = []
    # periods longer than the TB handler (~54 cycles): a faster timer starves
    # the main program (pre-existing, unrelated to DI)
    for lat in (1, 3, 7):
        for per in (lat * 30 + 60, lat * 50 + 90, 401):
            # +LATR: random extra latency breaks timer / loop phase lock
            out.append((_crit(), {'end': 'park', 'kind': '0', 'r9': 0},
                        [f'+LAT={lat}', f'+IRQ_PERIOD={per}', '+LATR=4']))
    return out


@test
def diei_irq_window_control():
    """negative control: without DI/EI the window check must see handlers"""
    return [(_crit(False), {'end': 'park', 'kind': '0', 'r9': 'nonzero'},
             [f'+LAT={l}', f'+IRQ_PERIOD={p}', '+LATR=4']) for l, p in ((3, 150), (1, 211))]


@test
def diei_wait():
    a = Asm()
    a.setr(10, CNT)
    a.di(1)
    a.set_int_mask(1)
    a.wait()               # disabled: wakes on the pending IRQ, no handler
    a.ld(64, 2, 10)        # 0
    a.ei(1)                # enable: the pending IRQ is taken now
    for _ in range(6):
        a.nop()
    a.ld(64, 3, 10)        # 1
    a.halt()
    return [(a, {'end': 'park', 'kind': '0', 'r2': 0, 'r3': 1}, [f'+LAT={l}', '+IRQ_PERIOD=200'])
            for l in (1, 3)]


@test
def diei_iret_frame():
    a = Asm()
    # hand-built frame: bit 43 = disabled, int_mask (bits 42:39) = 1, pc = 'back'
    a.setr(1, 0)
    a.leapc(2, 'back')
    a.setr64(3, (1 << 43) | (1 << 39))
    a.alu('or', 3, 3, 2)
    a.push(3)
    a.iret()
    a.halt()                 # not reached
    a.label('back')
    a.di(4)                  # frame restored "disabled" -> 0
    a.ei(4)                  # stays disabled
    a.setr(5, 1); a.ei(5)
    a.di(6)                  # -> 1
    a.halt()
    return a, {'end': 'park', 'kind': '0', 'r4': 0, 'r6': 1, 'int_mask': '1'}, []


# ------------------------------------------------- §3.3 misaligned trap
def _misal(kind_fn, lat=3, irq=None):
    a = Asm()
    a.setr(12, DATA)
    a.setr(1, 0x1111); a.setr(2, 0x2222)
    a.st(64, 1, 12, 0x100)          # older store: must land
    a.setr(3, 0xABCD)
    a.set_int_mask(1)
    pc = kind_fn(a)                  # the misaligned access
    a.st(64, 2, 12, 0x108)          # younger store: must NOT land
    a.setr(4, 0x55)                  # younger: must not retire
    a.halt()
    exp = {'end': 'park', 'kind': '5', 'pc': pc, m(DATA + 0x100): 0x1111, 'r3': 0xABCD, 'r4': 0,
           'r9': 0}
    plus = [f'+LAT={lat}'] + ([f'+IRQ_PERIOD={irq}'] if irq else [])
    return a, exp, plus


@test
def misal_trap():
    cases = [
        lambda a: (a.setr(9, 0), a.ld(16, 9, 12, 1, mode='long'))[1],          # 16-bit [rs1+1]
        lambda a: (a.setr(9, 0), a.ld(32, 9, 12, 2, mode='long'))[1],          # 32-bit [rs1+2]
        lambda a: (a.setr(9, 0), a.ld(32, 9, 12, 7, mode='long'))[1],
        lambda a: (a.setr(9, 0), a.ld_abs(16, 9, DATA + 3))[1],                 # [imm]
        lambda a: (a.setr(9, 0), a.setr(13, 5), a.ld_rr(32, 9, 12, 13))[2],    # [rs1+rs2]
        lambda a: (a.setr(9, 0), a.setr(13, DATA + 1), a.ld(16, 9, 13))[2],    # 16-bit [rs1]
        lambda a: (a.setr(9, 0), a.st(16, 3, 12, 0x101, mode='long'))[1],      # stores
        lambda a: (a.setr(9, 0), a.st(32, 3, 12, 0x102, mode='long'))[1],
        lambda a: (a.setr(9, 0), a.setr(13, DATA + 0x103), a.st(32, 3, 13))[2],
        lambda a: (a.setr(9, 0), a.st_abs(16, 3, DATA + 0x10F))[1],
    ]
    out = []
    for i, c in enumerate(cases):
        a, exp, plus = _misal(c, lat=1 + (i % 3) * 3)
        out.append((a, exp, plus))
    # with a fast pending timer IRQ: still parks, handler never entered after
    a, exp, plus = _misal(cases[1], irq=70)
    out.append((a, exp, plus))
    return out


@test
def misal_store_untouched():
    """the misaligned store itself must not write anything"""
    a = Asm()
    a.setr(12, DATA); a.setr(3, 0xFFFFFFFF)
    a.st(64, 0, 12, 0x200)
    pc = a.st(32, 3, 12, 0x201, mode='long')
    a.halt()
    return a, {'end': 'park', 'kind': '5', 'pc': pc, 'bus_wr': '1'}, []


@test
def misal_wrong_path():
    """misaligned accesses behind a taken FBR / taken JMP are never executed"""
    a = Asm()
    a.setr(12, DATA); a.setr(1, 1)
    a.fbr('ne', 1, 0, 'ok', imm=True)
    a.ld(16, 2, 12, 1, mode='long')
    a.st(32, 2, 12, 2, mode='long')
    a.label('ok')
    a.jmp('ok2')
    a.ld(32, 2, 12, 3, mode='long')
    a.label('ok2')
    a.ld(16, 2, 12, 2, mode='long')    # aligned: fine
    a.setr(13, DATA + 1)
    a.ld(32, 5, 13)                     # MEMGET32: unaligned-tolerant, fine
    a.halt()
    return a, {'end': 'park', 'kind': '0'}, []


# ------------------------------------------------- §2.6 I-cache counters
@test
def ic_counters():
    """a 2-line loop: cold misses on the first pass only, then all hits"""
    a = Asm()
    a.setr(1, 50)
    a.label('l')
    for _ in range(6):
        a.nop()
    a.addi_s(1, 1, -1)
    a.fbr('ne', 1, 0, 'l', imm=True)
    a.halt()
    # (EN_BP: the loop-exit fall-through is fetched after the mispredict: +1 miss)
    return a, {'end': 'park', 'kind': '0', 'ic_miss': '4', 'ic_fill': '20', 'bus_rd': '4'}, []


# ------------------------------------------------- HW proposal 9: short forms
M64 = 2**64 - 1


@test
def short_cmprvw():
    """short CMPRVW simm8 sets the same flags as the 2-word CMPRVW"""
    out = []
    for v, imm in [(0x100000005, 5), (0xFFFFFFFD, 2), (0x7FFFFFFF, -1), (0x80000000, 127),
                   (0x123400000000, 0), (5, 5), (3, -128), (0xFFFFFF80, -128)]:
        for short in (False, True):
            a = Asm()
            a.setr64(1, v)
            a.cmpi_s_w(1, imm) if short else a.cmpi(1, imm, w=True)
            a.halt()
            out.append((a, short, v, imm))
    # reference flags from the 2-word form, run on the same core
    import run
    exe = run.build(None)
    res = []
    for a, short, v, imm in out:
        if not short:
            st, _ = run.run(exe, a, f'cmprvw_ref_{v:x}_{imm & 0xFF:x}')
            ref = st['flags']
        else:
            res.append((a, {'end': 'park', 'kind': '0', 'flags': ref}, []))
    return res


@test
def short_mulv():
    out = []
    for v, imm, sgn in [(7, 6, True), (7, -6, True), (-3 & M64, 100, True), (2**40 + 3, -128, True),
                        (5, 0xF0, False), (2**63, 2, True), (0x123456789, 127, False)]:
        a = Asm()
        a.setr64(1, v)
        a.mulv_s(2, 1, imm, sgn=sgn)
        a.mulv(3, 1, imm if sgn else imm & 0xFF, sgn=sgn)   # 2-word reference
        a.alu('sub', 4, 2, 3)
        a.mulv_s(5, 1, imm, sgn=sgn)
        a.halt()
        sv = v - 2**64 if (sgn and v >> 63) else v
        prod = (sv * (imm if sgn else imm & 0xFF)) & M64
        out.append((a, {'end': 'park', 'kind': '0', 'r2': prod, 'r3': prod, 'r4': 0, 'r5': prod}, []))
    return out


@test
def short_leapc():
    a = Asm()
    a.label('top')
    a.leapc_s(1, 'top')
    a.leapc(2, 'top')
    a.leapc_s(3, 'far')
    a.leapc(4, 'far')
    a.alu('sub', 5, 1, 2)
    a.alu('sub', 6, 3, 4)
    a.jmp('end')
    for _ in range(3000):
        a.nop()
    a.label('far')
    a.nop()
    a.label('end')
    a.leapc_s(7, 'top')          # backward ~12 KiB
    a.halt()
    return a, {'end': 'park', 'kind': '0', 'r1': 0x20, 'r2': 0x20, 'r5': 0, 'r6': 0,
               'r3': a.labels['far'], 'r7': 0x20}, []


# ------------------------------------------------- 1c: SMC under posted stores
def _word(fn):
    t = Asm()
    fn(t)
    return t.assemble()[0]


def _smc(dist, size, warm, lat, pad_align=0):
    """overwrite the instruction at X (= `addi r1,r1,1`) with `addi r1,r1,16`
    using a `size`-bit store placed `dist` instructions before X.  With warm,
    X runs once first (it is then in the I-cache / NLB / IFB)."""
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    a = Asm()
    a.setr(1, 0); a.setr(3, new); a.setr(5, 0)
    a.leapc(2, 'X')
    for _ in range(pad_align):
        a.nop()
    if warm:
        a.setr(6, 2)                        # run X twice before the store
        a.label('warm')
        a.call('X')
        a.addi_s(6, 6, -1)
        a.fbr('ne', 6, 0, 'warm', imm=True)
    a.setr(7, 0x10000)
    a.st(64, 7, 7, 0)                       # an unrelated (postable) store first
    if size == 64:
        a.ld(64, 4, 2)                      # the dword holding X: keep its other half
        a.setr64(8, 0xFFFFFFFF00000000)
        a.alu('and', 4, 4, 8)
        a.alu('or', 4, 4, 3)                # X sits in the low half (aligned below)
        a.st(64, 4, 2)
    elif size == 32:
        a.st(32, 3, 2)
    else:                                   # byte store of the changed byte (imm8 [19:12] -> byte 2)
        a.setr(10, (new >> 16) & 0xFF)
        a.st(8, 10, 2, 2)
    for _ in range(dist):
        a.inc(5)
    if warm:
        a.call('X')
        a.jmp('done')
    while (a.pc & 7) != 0:
        a.nop()
    a.label('X')
    a.alui_s('add', 1, 1, 1)
    if warm:
        a.ret()
    a.label('done')
    a.halt()
    # (the byte store rewrites only imm8's high nibble: 1 -> 0x11)
    want = (2 if warm else 0) + (0x11 if size == 8 else 16)
    return a, {'end': 'park', 'kind': '0', 'r1': want, 'r5': dist}, [f'+LAT={lat}']


@test
def smc_posted():
    out = []
    for dist in (0, 1, 2, 3, 4, 6, 9, 15, 40):
        for size in (64, 32, 8):
            for warm in (False, True):
                out.append(_smc(dist, size, warm, lat=1 + (dist + size) % 7))
    return out


@test
def smc_stale_tag():
    """a once-code line, invalidated by a store, keeps its tag in the I-cache
    tag shadow: later data stores into it must post (st_post_ok checks the
    valid bits), and a code patch into it must still run the new code"""
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    out = []
    for nst in (1, 4):
        for lat in (1, 3, 7):
            a = Asm()
            a.setr(1, 0); a.setr(3, new); a.setr(9, 0x5A)
            a.leapc(2, 'C')
            a.call('C')                         # r1 = 1; line C installed
            for _ in range(nst):
                a.st(64, 9, 2, 8)               # data dword of the code line
                a.alu('add', 5, 5, 5)
            a.st(32, 3, 2)                      # patch C: addi r1,r1,16
            a.call('C')                         # r1 = 17
            a.ld(64, 4, 2, 8)
            a.halt()
            while (a.pc & 15) != 0:
                a.nop()
            a.label('C')
            a.alui_s('add', 1, 1, 1)
            a.ret()
            a.raw(0, 0)                         # the data dword at C+8
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 17, 'r4': 0x5A}, [f'+LAT={lat}']))
    return out


@test
def smc_data_in_code_line():
    """a data store into a code line (not into an instruction) must not
    corrupt the instructions around it"""
    a = Asm()
    a.setr(1, 0); a.setr(9, 0x5A)
    a.leapc(2, 'D')
    a.setr(4, 3)
    a.label('loop')
    a.st(64, 9, 2)                       # store into the data dword in this code line
    a.inc(1)
    a.addi_s(4, 4, -1)
    a.fbr('ne', 4, 0, 'loop', imm=True)
    a.ld(64, 3, 2)
    a.halt()
    while (a.pc & 7) != 0:
        a.nop()
    a.label('D')
    a.raw(0xDEAD0000, 0xBEEF)            # 8 data bytes after the code
    return [(a, {'end': 'park', 'kind': '0', 'r1': 3, 'r3': 0x5A}, [f'+LAT={l}']) for l in (1, 3, 8)]


# ------------------------------------------------- 2b-lite (opt-in switch)
@test
def fbr_eq_loops():
    """EQ/NE/EQW fused branches, taken and not, back-to-back and in loops —
    exercised whichever stage resolves them (EN_FBR_EQ_EX)"""
    a = Asm()
    a.setr(1, 0); a.setr(2, 0); a.setr(3, 37); a.setr64(4, 0xFFFFFFFF00000025)
    a.label('l')
    a.inc(1)
    a.fbr('eq', 1, 3, 'out')               # taken once at 37
    a.nop()
    a.alu('add', 2, 2, 1)
    a.fbr('ne', 1, 0, 'l', imm=True)       # always taken
    a.label('never')
    a.setr(2, 0xBAD)
    a.label('out')
    a.fbr(None, 1, 4, 'w_ok', pred=5, inv=0)   # EQW: low halves equal (37) -> taken
    a.setr(2, 0xBAD2)
    a.label('w_ok')
    a.fbr('ne', 1, 1, 'bad')               # not taken
    a.fbr('eq', 1, 1, 'ok')                # taken, right behind a not-taken one
    a.label('bad')
    a.setr(2, 0xBAD3)
    a.label('ok')
    a.halt()
    return [(a, {'end': 'park', 'kind': '0', 'r1': 37, 'r2': 36 * 37 // 2}, [f'+LAT={l}']) for l in (1, 4)]


# ------------------------------------------------- 2a: return-address stack
def _fib(n):
    return n if n < 2 else _fib(n - 1) + _fib(n - 2)


@test
def ras_recursion():
    """recursive fib (depth 12 > RAS depth): every return predicted or
    corrected, result exact; LEAVERET and plain RET frames"""
    out = []
    for leaveret in (False, True):
        a = Asm()
        a.setr(1, 12)
        a.call('fib')
        a.halt()
        a.label('fib')                     # r1 = n -> r2 = fib(n); clobbers r3
        if leaveret:
            a.enter(2)
            a.st(64, 1, 15, -8)            # save n (frame slot)
        else:
            a.push(1)
        a.cmpi_s(1, 2)
        a.jmp('rec', cond='ge')
        a.mov(2, 1)
        a.jmp('ret')
        a.label('rec')
        a.addi_s(1, 1, -1)
        a.call('fib')
        a.push(2)
        a.addi_s(1, 1, -1)
        a.call('fib')
        a.pop(3)
        a.alu('add', 2, 2, 3)
        a.label('ret')
        if leaveret:
            a.ld(64, 1, 15, -8)
            a.leaveret()
        else:
            a.pop(1)
            a.ret()
        for lat in (1, 3, 7):
            out.append((a, {'end': 'park', 'kind': '0', 'r2': _fib(12), 'r1': 12, 'sp': 0x08000000},
                        [f'+LAT={lat}']))
    return out


@test
def ras_mispredict():
    """the callee rewrites its return address: the predicted path (a store
    of a marker, then HALT) must never take effect"""
    out = []
    for kind in ('ret', 'leaveret'):
        for gap in (0, 1, 3):
            a = Asm()
            a.setr(12, DATA); a.setr(9, 0x77)
            a.call('f')
            # predicted (wrong) path: store a marker, then HALT
            a.st(64, 9, 12, 0)
            a.halt()
            a.label('real')
            a.setr(5, 0x55)
            a.halt()
            a.label('f')
            if kind == 'leaveret':
                a.enter(0)
                a.leapc(4, 'real')
                a.st(64, 4, 15, 8)         # [R15+8] = return address
            else:
                a.pop(4)
                a.leapc(4, 'real')
                a.push(4)
            for _ in range(gap):
                a.nop()
            a.leaveret() if kind == 'leaveret' else a.ret()
            for lat in (1, 4):
                out.append((a, {'end': 'park', 'kind': '0', 'r5': 0x55, m(DATA): 0},
                            [f'+LAT={lat}']))
    return out


@test
def ras_irq():
    """calls and returns under a timer: same result, handler count > 0"""
    a = Asm()
    a.set_int_mask(1)
    a.setr(10, 0); a.setr(11, 400)
    a.label('l')
    a.call('g')
    a.addi_s(11, 11, -1)
    a.fbr('ne', 11, 0, 'l', imm=True)
    a.setr(13, CNT); a.ld(64, 8, 13)
    a.halt()
    a.label('g')
    a.call('h')
    a.inc(10)
    a.ret()
    a.label('h')
    a.inc(10)
    a.ret()
    return [(a, {'end': 'park', 'kind': '0', 'r10': 800, 'r8': 'nonzero'},
             [f'+LAT={l}', f'+IRQ_PERIOD={p}', '+LATR=3']) for l, p in ((1, 150), (3, 233), (6, 401))]


@test
def smc_ahead():
    """a 32-bit store rewrites the instruction k slots ahead in straight-line
    code (k = 1..8, every alignment, warm and cold): the new code must run"""
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    out = []
    for k in range(1, 9):
        for pad in range(4):
            for warm in (False, True):
                a = Asm()
                a.setr(1, 0); a.setr(3, new); a.setr(6, 2 if warm else 1)
                a.leapc(2, 'X')
                a.label('again')
                for _ in range(pad):
                    a.nop()
                # second pass (warm) performs the patch; first pass only runs X
                a.fbr('ne', 6, 1, 'skip', imm=True) if warm else None
                a.st(32, 3, 2)
                a.label('skip')
                for _ in range(k - 1):
                    a.alu('add', 5, 5, 5)
                a.label('X')
                a.alui_s('add', 1, 1, 1)
                a.addi_s(6, 6, -1)
                a.fbr('ne', 6, 0, 'again', imm=True)
                a.halt()
                want = (1 + 16) if warm else 16
                out.append((a, {'end': 'park', 'kind': '0', 'r1': want}, [f'+LAT={1 + (k + pad) % 6}']))
    return out


# ============================================ MMU stage 1 (MMU_PLAN.md)
# The common exception handler records, at XD + 0/8/16/24/32: ECAUSE,
# EADDR, the frame word, STATUS at entry, and a running count.  With
# skip=True it steps the frame PC over a 1-word faulting instruction
# (not for SYSCALL, whose frame PC is already the next instruction); with
# fix=reg it instead sets that register in the frame-saved register file
# — see _uhandler for the user-mode variant.
XD = DATA + 0x400
X_ILL, X_PRIV, X_TRAP, X_SYSCALL = 1, 2, 3, 4
X_MISAL_LD, X_MISAL_ST, X_ACC_LD, X_ACC_ST, X_ACC_FE, X_MISAL_FE = 5, 6, 7, 8, 9, 10


def _xhandler(a, skip=True, name='h'):
    a.label(name)
    a.push(13); a.push(14)
    a.setr(14, XD)
    a.mfsr(13, 'ecause'); a.st(64, 13, 14, 0)
    a.mfsr(13, 'eaddr');  a.st(64, 13, 14, 8)
    a.getsp(13); a.ld(64, 13, 13, 16); a.st(64, 13, 14, 16)
    a.mfsr(13, 'status'); a.st(64, 13, 14, 24)
    a.ld(64, 13, 14, 32); a.inc(13); a.st(64, 13, 14, 32)
    if skip:
        a.mfsr(13, 'ecause')
        a.fbr('eq', 13, X_SYSCALL, name + '_s', imm=True)
        a.getsp(13); a.ld(64, 14, 13, 16); a.addi_s(14, 14, 4); a.st(64, 14, 13, 16)
        a.label(name + '_s')
    a.pop(14); a.pop(13)
    a.iret()


def _xsetup(a, handler='h'):
    a.leapc(1, handler)
    a.mtsr('evec', 1)


def _xexp(cause, eaddr, frame_pc, count=1, status=2, frame_hi=0):
    return {m(XD): cause, m(XD + 8): eaddr, m(XD + 16): (frame_hi << 32) | frame_pc,
            m(XD + 24): status, m(XD + 32): count}


@test
def exc_syscall():
    a = Asm()
    _xsetup(a)
    a.setr(5, 0)
    at = a.syscall()
    a.setr(5, 0x55)
    a.mfsr(6, 'status')           # DI restored by IRET: 0
    a.halt()
    _xhandler(a)
    exp = {'end': 'park', 'kind': '0', 'r5': 0x55, 'r6': 0, 'sp': 0x08000000}
    exp.update(_xexp(X_SYSCALL, 0, at + 4))
    return a, exp, []


@test
def exc_trap_ill():
    out = []
    for kind, word, cause in (('trap', 0x6C040000, X_TRAP), ('ill', 0x7C000000, X_ILL),
                              ('mfsr9', 0x6C409100, X_ILL), ('mtsr_cycle', 0x6C608010, X_ILL),
                              ('syscall_rs1', 0x6C080010, X_ILL)):
        a = Asm()
        _xsetup(a)
        a.setr(5, 1)
        at = a.raw(word)
        a.setr(5, 2)
        a.halt()
        _xhandler(a)
        exp = {'end': 'park', 'kind': '0', 'r5': 2}
        exp.update(_xexp(cause, word, at))
        out.append((a, exp, []))
    return out


@test
def exc_misal():
    """misaligned load / store: rd and memory untouched, EADDR = raw EA"""
    out = []
    for st, size, off in ((False, 16, 1), (False, 32, 2), (True, 16, 3), (True, 32, 6)):
        a = Asm()
        _xsetup(a)
        a.setr(2, DATA); a.setr(3, 0x77); a.setr(4, 0x1234); a.setr(7, off)
        # 1-word reg+reg form, so the handler's 4-byte skip lands on the next op
        at = (a.st_rr(size, 4, 2, 7) if st else a.ld_rr(size, 3, 2, 7))
        a.setr(6, 9)
        a.halt()
        _xhandler(a)
        exp = {'end': 'park', 'kind': '0', 'r3': 0x77, 'r6': 9, m(DATA): 0}
        exp.update(_xexp(X_MISAL_ST if st else X_MISAL_LD, DATA + off, at))
        out.append((a, exp, []))
    return out


@test
def exc_evec0_parks():
    """EVEC = 0 (the reset value): every cause parks exactly as before"""
    out = []
    for word, kind in ((0x6C040000, 1), (0x7C000000, 2), (0x6C080000, 2), (0x6C409100, 2),
                       (0x6C608010, 2)):
        a = Asm()
        a.setr(5, 1)
        at = a.raw(word)
        a.setr(5, 2)
        a.halt()
        out.append((a, {'end': 'park', 'kind': str(kind), 'pc': at, 'op': word, 'r5': 1}, []))
    a = Asm()
    a.setr(2, DATA + 1)
    at = a.ld(16, 3, 2)
    a.halt()
    out.append((a, {'end': 'park', 'kind': '5', 'pc': at}, []))
    return out


@test
def sr_roundtrip():
    a = Asm()
    a.setr(12, DATA)
    vals = {'evec': 0x12345677, 'ecause': 0x1A5, 'eaddr': 0xDEADBEEF, 'usp': 0x07F00010,
            'scratch0': 0x0123456789ABCDEF, 'scratch1': 0xFEDCBA9876543210, 'tp': 0x1122334455667788}
    for i, (sr, v) in enumerate(vals.items()):
        a.setr64(1, v)
        a.mtsr(sr, 1)
        a.mfsr(2, sr)
        a.st(64, 2, 12, 8 * i)
    a.mtsr('evec', 0)                # leave EVEC clear
    a.mfsr(3, 'status')
    a.di(4)
    a.mfsr(5, 'status')              # DI forwarded: 2
    a.ei(4)
    a.mfsr(6, 'status')
    a.mfsr(7, 'cycle')
    a.nop(); a.nop()
    a.mfsr(8, 'cycle')
    a.alu('sub', 9, 8, 7)
    a.mtsr('status', 1)              # no writable bits: ignored
    a.mfsr(10, 'status')
    a.halt()
    exp = {'end': 'park', 'kind': '0', 'r3': 0, 'r5': 2, 'r6': 0, 'r10': 0, 'r9': 'nonzero'}
    want = {'evec': 0x12345674, 'ecause': 0xA5, 'eaddr': 0xDEADBEEF, 'usp': 0x07F00010,
            'scratch0': 0x0123456789ABCDEF, 'scratch1': 0xFEDCBA9876543210, 'tp': 0x1122334455667788}
    for i, sr in enumerate(vals):
        exp[m(DATA + 8 * i)] = want[sr]
    return a, exp, []


@test
def exc_di_irq():
    """exception entry sets DI: a pending timer IRQ is not taken in the
    handler, and is taken after the IRET restores DI = 0"""
    a = Asm()
    _xsetup(a)
    a.setr(12, XD)
    a.set_int_mask(1)
    a.setr(1, 300)
    a.label('w'); a.dec(1); a.fbr('ne', 1, 0, 'w', imm=True)   # IRQs run here
    a.syscall()
    a.setr(1, 300)
    a.label('w2'); a.dec(1); a.fbr('ne', 1, 0, 'w2', imm=True)
    a.halt()
    a.label('h')
    a.push(13); a.push(14)
    a.ld_abs(64, 13, 0x180000)                  # IRQ count at entry
    a.setr(14, 400)
    a.label('hw'); a.dec(14); a.fbr('ne', 14, 0, 'hw', imm=True)
    a.ld_abs(64, 14, 0x180000)                  # ... and after a long wait
    a.alu('sub', 13, 14, 13)
    a.st(64, 13, 12, 0x40)                      # 0: no IRQ inside the handler
    a.pop(14); a.pop(13)
    a.iret()
    return a, {'end': 'park', 'kind': '0', m(XD + 0x40): 0, 'm00180000': 'nonzero'}, ['+IRQ_PERIOD=60']


USP = 0x07F80000           # user stack (in the TB's high memory window)
HALT_W = 0x6C010000
XLOG = XD + 0x100          # the user handler's cause log (8 bytes per entry)


def _to_user(a, ucode, usp=USP, mask=0):
    """supervisor -> user: set USP, build a PM = 1 frame, IRET"""
    a.setr(1, usp); a.mtsr('usp', 1)
    a.leapc(2, ucode)
    a.setr64(3, (1 << 44) | (mask << 39)); a.alu('or', 2, 2, 3)
    a.push(2)
    a.iret()


def _uhandler(a, name='h'):
    """records like _xhandler plus: XD+40 = USP at entry, XD+48 = SP after
    the two pushes, XLOG[count-1] = ECAUSE; a user-mode HALT (PRIVILEGED with
    EADDR = the HALT word) halts in supervisor mode; otherwise it steps over
    the faulting 1-word op (not SYSCALL) and returns"""
    a.label(name)
    a.push(13); a.push(14)
    a.setr(14, XD)
    a.mfsr(13, 'ecause'); a.st(64, 13, 14, 0)
    a.mfsr(13, 'eaddr');  a.st(64, 13, 14, 8)
    a.getsp(13); a.st(64, 13, 14, 48); a.ld(64, 13, 13, 16); a.st(64, 13, 14, 16)
    a.mfsr(13, 'status'); a.st(64, 13, 14, 24)
    a.mfsr(13, 'usp');    a.st(64, 13, 14, 40)
    a.ld(64, 13, 14, 32); a.inc(13); a.st(64, 13, 14, 32)
    a.shift_n('shl', 13, 13, 3); a.alu('add', 13, 13, 14)
    a.mfsr(14, 'ecause'); a.st(64, 14, 13, 0x100 - 8)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, name + '_nh')
    a.halt()
    a.label(name + '_nh')
    a.mfsr(13, 'ecause')
    a.fbr('eq', 13, X_SYSCALL, name + '_s', imm=True)
    a.getsp(13); a.ld(64, 14, 13, 16); a.addi_s(14, 14, 4); a.st(64, 14, 13, 16)
    a.label(name + '_s')
    a.pop(14); a.pop(13)
    a.iret()


@test
def user_syscall_stacks():
    a = Asm()
    _xsetup(a)
    _to_user(a, 'u')
    a.label('u')
    a.getsp(5)                     # the user SP
    a.mfsr(6, 'status')            # UM = 1, DI = 0
    a.push(5)                      # user stack in use: USP - 8
    at = a.syscall()
    a.pop(7)
    a.getsp(8)
    a.mfsr(9, 'status')
    a.halt()                       # PRIVILEGED -> the handler halts
    _uhandler(a)
    exp = {'end': 'park', 'kind': '0', 'r5': USP, 'r6': 1, 'r7': USP, 'r8': USP, 'r9': 1,
           m(XLOG): X_SYSCALL, m(XLOG + 8): X_PRIV,
           m(XD): X_PRIV, m(XD + 8): HALT_W, m(XD + 24): 2, m(XD + 32): 2,
           m(XD + 40): USP,                       # USP seen by the handler
           m(XD + 48): 0x08000000 - 8 - 16}       # kernel SP: frame + 2 pushes
    return a, exp, []


@test
def user_priv_ops():
    """every privileged op traps in user mode; the allowed SRs do not"""
    a = Asm()
    _xsetup(a)
    a.setr64(10, 0x5566778899AABBCC)
    _to_user(a, 'u')
    a.label('u')
    priv = [lambda: a.di(1), lambda: a.ei(1), lambda: a.iret(), lambda: a.wait(),
            lambda: a.sys(3), lambda: a.sys(5, 1), lambda: a.raw(0x70000010),   # RESET, DELAYR, LCDCMD
            lambda: a.mfsr(1, 'evec'), lambda: a.mfsr(1, 'usp'), lambda: a.mfsr(1, 'scratch0'),
            lambda: a.mtsr('evec', 1), lambda: a.mtsr('status', 1), lambda: a.mtsr('ecause', 1)]
    for f in priv:
        f()
    a.mtsr('tp', 10); a.mfsr(11, 'tp')          # allowed
    a.mfsr(12, 'cycle'); a.mfsr(4, 'status')
    a.halt()
    _uhandler(a)
    exp = {'end': 'park', 'kind': '0', 'r11': 0x5566778899AABBCC, 'r12': 'nonzero', 'r4': 1,
           m(XD + 32): len(priv) + 1}
    for i in range(len(priv) + 1):
        exp[m(XLOG + 8 * i)] = X_PRIV
    return a, exp, []


@test
def user_irq():
    """a timer IRQ taken in user mode runs its handler on the kernel stack
    and returns to user mode with the user SP intact"""
    a = Asm()
    _xsetup(a)
    _to_user(a, 'u', mask=1)
    a.label('u')
    # canaries below the user SP: an IRQ frame / handler push on the user
    # stack would overwrite them
    a.setr64(7, 0xC0FFEE0011223344); a.setr(4, USP)
    a.st(64, 7, 4, -16); a.st(64, 7, 4, -24); a.st(64, 7, 4, -32)
    a.setr(1, 0); a.setr(2, 2000)
    a.label('l'); a.alu('add', 1, 1, 2); a.push(1); a.pop(3); a.dec(2)
    a.fbr('ne', 2, 0, 'l', imm=True)
    a.getsp(5); a.mfsr(6, 'status')
    a.ld(64, 8, 4, -16); a.ld(64, 9, 4, -24); a.ld(64, 10, 4, -32)
    a.halt()
    _uhandler(a)
    # sum 1..2000 = 2001000
    c = 0xC0FFEE0011223344
    return a, {'end': 'park', 'kind': '0', 'r1': 2001000, 'r3': 2001000, 'r5': USP, 'r6': 1,
               'r8': c, 'r9': c, 'r10': c, 'm00180000': 'nonzero', m(XD): X_PRIV}, ['+IRQ_PERIOD=97']


# ---------------------------------------------- (c) access and fetch faults
GOOD = DATA + 0x300        # a usable stack top / data area (dumped)
BADSP = 0xF0000100         # MMIO
FLOG = XD + 0x100          # 0x30 bytes per fault: cause, eaddr, usp, r15, r9, frame
GETF_ZE = 0xC000000000000000


def _getf(a, rd):
    return a.raw(0x40000000 | 5 << 26 | 12 << 22 | rd << 8)


def _fixhandler(a, name='h'):
    """logs every exception at FLOG + 0x30*i (the state AT the fault), then:
    a user HALT halts; ACCESS_LOAD/STORE: r9 <- r10, r15 <- r11, USP <- r12
    if USP is in MMIO, and retry (no skip); fetch faults: frame PC <- r10;
    others: step over 1 word (not SYSCALL)."""
    a.label(name)
    a.push(13); a.push(14)
    a.setr(14, XD)
    a.ld(64, 13, 14, 32); a.inc(13); a.st(64, 13, 14, 32)          # count
    a.mulv_s(13, 13, 0x30); a.alu('add', 13, 13, 14); a.addi_s(13, 13, 0x100 - 0x30 - 128)
    a.addi_s(13, 13, 127); a.addi_s(13, 13, 1)                       # 13 = FLOG + 0x30*(count-1)
    a.mfsr(14, 'ecause'); a.st(64, 14, 13, 0)
    a.mfsr(14, 'eaddr');  a.st(64, 14, 13, 8)
    a.mfsr(14, 'usp');    a.st(64, 14, 13, 16)
    a.st(64, 15, 13, 24); a.st(64, 9, 13, 32)
    a.getsp(14); a.ld(64, 14, 14, 16); a.st(64, 14, 13, 40)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, name + '_nh'); a.halt()
    a.label(name + '_nh')
    a.mfsr(13, 'ecause')
    a.fbr('eq', 13, X_SYSCALL, name + '_ret', imm=True)
    a.addi_s(14, 13, -X_ACC_LD); a.fbr('ule', 14, 1, name + '_acc', imm=True)   # 7 / 8
    a.addi_s(14, 13, -X_ACC_FE); a.fbr('ule', 14, 1, name + '_fe', imm=True)    # 9 / 10
    a.getsp(13); a.ld(64, 14, 13, 16); a.addi_s(14, 14, 4); a.st(64, 14, 13, 16)
    a.jmp(name + '_ret')
    a.label(name + '_acc')
    a.mov(9, 10); a.mov(15, 11)
    a.mfsr(13, 'usp'); a.shift_n('shr', 13, 13, 28); a.setr(14, 15)
    a.fbr('ne', 13, 14, name + '_ret')
    a.mtsr('usp', 12)
    a.jmp(name + '_ret')
    a.label(name + '_fe')
    a.getsp(13); a.ld(64, 14, 13, 16)
    a.shift_n('shr', 14, 14, 32); a.shift_n('shl', 14, 14, 32); a.alu('or', 14, 14, 10)
    a.st(64, 14, 13, 16)
    a.label(name + '_ret')
    a.pop(14); a.pop(13)
    a.iret()


def _ufault(body, exp_extra, plus=(), log0=None):
    """user-mode program: body(a) runs in user mode then HALTs (PRIVILEGED,
    which the handler turns into a supervisor halt)"""
    a = Asm()
    _xsetup(a)
    _to_user(a, 'u')
    a.label('u')
    a.setr(12, GOOD)
    body(a)
    a.halt()
    _fixhandler(a)
    exp = {'end': 'park', 'kind': '0', 'mmio_wr': '0', 'mmio_rd': '0'}
    if log0:
        for k, v in zip((0, 8, 16, 24, 32), log0):
            if v is not None:
                exp[m(FLOG + k)] = v
    exp.update(exp_extra)
    return a, exp, list(plus)


def _stk_cases():
    cases = {}

    def push(a):
        a.setr(1, 0xAAAA); a.setr(2, BADSP); a.cmp(1, 1); a.setsp(2)
        a.push(1); _getf(a, 8); a.getsp(2)
    cases['push'] = (push, {'r2': GOOD - 8, m(GOOD - 8): 0xAAAA, 'r8': GETF_ZE},
                     (X_ACC_ST, BADSP - 8, BADSP))

    def pushv(a):
        a.setr(2, BADSP); a.setsp(2); a.pushv(0x1234); a.getsp(2)
    cases['pushv'] = (pushv, {'r2': GOOD - 8, m(GOOD - 8): 0x1234}, (X_ACC_ST, BADSP - 8, BADSP))

    def pop(a):
        a.setr(3, 0x5555); a.st(64, 3, 12); a.setr(3, 0x77)
        a.setr(2, BADSP); a.setsp(2); a.pop(3); a.getsp(2)
    cases['pop'] = (pop, {'r2': GOOD + 8, 'r3': 0x5555}, (X_ACC_LD, BADSP, BADSP))

    def call(a):
        a.setr(2, BADSP); a.setsp(2); a.setr(4, 0)
        a.call('f'); a.getsp(2); a.jmp('e')
        a.label('f'); a.getsp(4); a.ret()
        a.label('e')
    cases['call'] = (call, {'r2': GOOD, 'r4': GOOD - 8}, (X_ACC_ST, BADSP - 8, BADSP))

    def callcc_nt(a):
        a.setr(2, BADSP); a.setsp(2); a.setr(1, 1); a.setr(3, 2); a.cmp(1, 3)   # Z = 0
        a.call('f', cond='z'); a.getsp(2); a.jmp('e')
        a.label('f'); a.setr(4, 0xBAD); a.ret()
        a.label('e')
    cases['callcc_nt'] = (callcc_nt, {'r2': BADSP, 'r4': 0, m(FLOG): X_PRIV}, None)

    def callr(a):
        a.leapc(5, 'f'); a.setr(2, BADSP); a.setsp(2); a.setr(4, 0)
        a.jmpr(5, link=True); a.getsp(2); a.jmp('e')
        a.label('f'); a.getsp(4); a.ret()
        a.label('e')
    cases['callr'] = (callr, {'r2': GOOD, 'r4': GOOD - 8}, (X_ACC_ST, BADSP - 8, BADSP))

    def ret(a):
        a.leapc(6, 'back'); a.st(64, 6, 12)
        a.setr(2, BADSP); a.setsp(2); a.ret()
        a.setr(4, 0xBAD)
        a.label('back'); a.getsp(2)
    cases['ret'] = (ret, {'r2': GOOD + 8, 'r4': 0}, (X_ACC_LD, BADSP, BADSP))

    def ret_ras(a):
        a.setr(2, GOOD + 8); a.setsp(2)
        a.call('f'); a.getsp(2); a.jmp('e')
        a.label('f')
        a.setr(3, BADSP); a.setsp(3); a.ret()          # RAS-predicted; its read faults
        a.label('e')
    cases['ret_ras'] = (ret_ras, {'r2': GOOD + 8}, (X_ACC_LD, BADSP, BADSP))

    def enter(a):
        a.setr(15, 0x777); a.setr(11, 0x777); a.setr(2, BADSP); a.setsp(2)
        a.enter(2); a.getsp(2); a.mov(3, 15)
    cases['enter'] = (enter, {'r2': GOOD - 8 - 16, 'r3': GOOD - 8, m(GOOD - 8): 0x777},
                      (X_ACC_ST, BADSP - 8, BADSP, 0x777))

    def leave(a):
        a.setr(3, 0x999); a.st(64, 3, 12); a.setr(11, GOOD)
        a.setr(15, 0xF0000200); a.leave(); a.getsp(2); a.mov(3, 15)
    cases['leave'] = (leave, {'r2': GOOD + 8, 'r3': 0x999}, (X_ACC_LD, 0xF0000200, USP, 0xF0000200))

    def leaveret(a):
        a.setr(3, 0x999); a.st(64, 3, 12); a.leapc(6, 'back'); a.st(64, 6, 12, 8)
        a.setr(11, GOOD); a.setr(15, 0xF0000200); a.leaveret()
        a.setr(4, 0xBAD)
        a.label('back'); a.getsp(2); a.mov(3, 15)
    cases['leaveret'] = (leaveret, {'r2': GOOD + 16, 'r3': 0x999, 'r4': 0},
                         (X_ACC_LD, 0xF0000200, USP, 0xF0000200))

    def leaveret_x(a):          # [R15] is DRAM-side, [R15+8] = 0xF000_0000
        a.setr(3, 0x999); a.st(64, 3, 12); a.leapc(6, 'back'); a.st(64, 6, 12, 8)
        a.setr(11, GOOD); a.setr(15, 0xEFFFFFF8); a.leaveret()
        a.setr(4, 0xBAD)
        a.label('back'); a.getsp(2); a.mov(3, 15)
    cases['leaveret_x'] = (leaveret_x, {'r2': GOOD + 16, 'r3': 0x999, 'r4': 0},
                           (X_ACC_LD, 0xF0000000, USP, 0xEFFFFFF8))

    def ld32_span(a):           # MEMGET32 at 0xEFFF_FFFD spans into 0xF000_0000
        a.setr64(3, 0x8877665544332211); a.st(64, 3, 12)
        a.setr(10, GOOD + 1); a.setr(9, 0xEFFFFFFD); a.setr(3, 0)
        a.ld(32, 3, 9)
    cases['ld32_span'] = (ld32_span, {'r3': 0x55443322}, (X_ACC_LD, 0xF0000000, USP, None, 0xEFFFFFFD))

    def ld(a):
        a.setr64(3, 0x0102030405060708); a.st(64, 3, 12, 8)
        a.setr(10, GOOD); a.setr(9, 0xF0010010); a.setr(3, 0)
        a.cmp(3, 3); a.ld(64, 3, 9, 8); _getf(a, 8)
    cases['ld'] = (ld, {'r3': 0x0102030405060708, 'r8': GETF_ZE},
                   (X_ACC_LD, 0xF0010018, USP, None, 0xF0010010))

    def st32(a):
        a.setr(10, GOOD); a.setr(9, 0xF0010000); a.setr(1, 0x41424344)
        a.st(32, 1, 9, 4)
    cases['st32'] = (st32, {m(GOOD): 0x4142434400000000}, (X_ACC_ST, 0xF0010004, USP, None, 0xF0010000))

    def st_rr(a):
        a.setr(10, GOOD); a.setr(9, 0xF0010000); a.setr(1, 0x61); a.setr(7, 3)
        a.st_rr(8, 1, 9, 7)
    cases['st_rr'] = (st_rr, {m(GOOD): 0x61000000}, (X_ACC_ST, 0xF0010003, USP, None, 0xF0010000))
    return cases


@test
def user_access_faults():
    out = []
    for name, (body, extra, log0) in _stk_cases().items():
        for lat in (1, 3, 7):
            out.append(_ufault(body, extra, [f'+LAT={lat}'], log0))
    return out


@test
def user_fault_pipe_mix():
    """a faulting PUSH with a missed load waiting in WB, a posted store
    draining and a DIV behind it; all of it right after the retry"""
    def body(a):
        a.setr(3, 0x1111); a.st(64, 3, 12, 0x40)
        a.setr(1, 100); a.setr(4, 7); a.setr(7, 0x2222); a.setr(2, BADSP)
        a.setsp(2)
        a.ld(64, 5, 12, 0x40)
        a.st(64, 7, 12, 0x48)
        a.push(1)
        a.div(6, 1, 4)
        a.alu('add', 8, 5, 6)
        a.getsp(2)
    out = []
    for lat in (1, 3, 7):
        out.append(_ufault(body, {'r5': 0x1111, 'r6': 14, 'r8': 0x1111 + 14, 'r2': GOOD - 8,
                                  m(GOOD - 8): 100, m(GOOD + 0x48): 0x2222},
                           [f'+LAT={lat}'], (X_ACC_ST, BADSP - 8, BADSP)))
    return out


@test
def user_fetch_faults():
    out = []

    def mis(a):
        a.leapc(10, 'ok'); a.leapc(5, 't'); a.addi_s(5, 5, 2); a.jmpr(5)
        a.label('t'); a.setr(4, 0xBAD)
        a.label('ok'); a.setr(6, 0x60)
    a, exp, plus = _ufault(mis, {'r6': 0x60, 'r4': 0, m(FLOG): X_MISAL_FE}, ())
    out.append((a, exp, plus))

    def acc(a):
        a.leapc(10, 'ok'); a.jmp_abs(0xF0010000)
        a.setr(4, 0xBAD)
        a.label('ok'); a.setr(6, 0x60)
    a, exp, plus = _ufault(acc, {'r6': 0x60, 'r4': 0, m(FLOG): X_ACC_FE, m(FLOG + 8): 0xF0010000,
                                 m(FLOG + 40): (1 << 44) | 0xF0010000}, ())
    out.append((a, exp, plus))

    # a 2-word op at 0xEFFF_FFFC whose immediate is at 0xF000_0000
    edw = '+EDW=8BC001006C000000'          # [0xEFFFFFFC] = SETR r1, [0xEFFFFFF8] = NOP

    def strad(a):
        a.leapc(10, 'ok'); a.setr(1, 0x33); a.jmp_abs(0xEFFFFFFC)
        a.label('ok'); a.setr(6, 0x60)
    a, exp, plus = _ufault(strad, {'r6': 0x60, 'r1': 0x33, m(FLOG): X_ACC_FE,
                                   m(FLOG + 8): 0xF0000000, m(FLOG + 40): (1 << 44) | 0xEFFFFFFC},
                           (edw,))
    out.append((a, exp, plus))

    # supervisor controls: the same fetches DO reach the bus
    a = Asm(); a.jmp_abs(0xF0010000); a.halt()
    out.append((a, {'end': 'park', 'kind': '2', 'mmio_rd': 'nonzero'}, []))
    a = Asm(); a.setr(1, 0x33); a.jmp_abs(0xEFFFFFFC); a.halt()
    out.append((a, {'end': 'park', 'kind': '2', 'r1': 0, 'mmio_rd': 'nonzero'}, [edw]))
    return out


@test
def user_wrongpath_fetch():
    """a wrong-path, predicted-taken JMP to MMIO behind a mispredicted
    fused branch: no MMIO read in user mode.  The supervisor variant is the
    control (it shows whether the speculative fill happens at all)."""
    out = []
    for user in (True, False):
        for lat in (1, 3, 7):
            a = Asm()
            _xsetup(a)
            if user:
                _to_user(a, 'u')
            a.label('u')
            a.setr(12, GOOD); a.setr(6, 0); a.setr(7, 3)
            # the FBR consumes a load through the bypass, so it waits in EX
            # for the load's response while the JMP behind it redirects fetch
            # (a loop: once the code is in the I-cache, fetch outruns the load;
            # the 2-bit counter still predicts not-taken on the 2nd pass)
            a.label('top')
            a.ld(64, 1, 12)
            a.fbr('eq', 1, 1, 'tk')                # taken; predicted not taken
            a.jmp_abs(0xF0010000)                  # wrong path, predicted taken
            a.nop(); a.nop()
            a.label('tk'); a.dec(7); a.fbr('ne', 7, 0, 'top', imm=True)
            a.setr(6, 0x66)
            a.halt()
            _fixhandler(a)
            exp = {'end': 'park', 'kind': '0', 'r6': 0x66}
            if user:
                exp.update({'mmio_rd': '0', m(FLOG): X_PRIV})
            out.append((a, exp, [f'+LAT={lat}']))
    return out


@test
def user_early_fetch():
    """fetch running ahead of execution toward MMIO: a JMP latched in ID
    behind a DIV redirects fetch (2a early redirect) while the DIV holds EX.
    In user mode no MMIO read happens and the JMP raises ACCESS_FETCH; the
    supervisor control fetches (and parks on the zero words it reads)."""
    out = []
    for user in (True, False):
        a = Asm()
        if user:                       # (the control parks with EVEC = 0)
            _xsetup(a)
            _to_user(a, 'u')
        a.label('u')
        a.leapc(10, 'ok')
        a.setr(1, 1000000); a.setr(2, 3)
        a.div(3, 1, 2)
        a.jmp_abs(0xF0010000)
        a.label('ok'); a.setr(6, 0x66)
        a.halt()
        _fixhandler(a)
        if user:
            exp = {'end': 'park', 'kind': '0', 'r6': 0x66, 'r3': 333333, 'mmio_rd': '0',
                   m(FLOG): X_ACC_FE, m(FLOG + 8): 0xF0010000}
        else:
            exp = {'end': 'park', 'kind': '2', 'r3': 333333, 'mmio_rd': 'nonzero'}
        out.append((a, exp, ['+LAT=3']))
    return out


# --------------------------------------------- (d) the debugger and EVEC
import os as _os
_DBG = _os.environ.get('VSIM_DEBUG_EN') == '1'


@test
def exc_debugger():
    """attached: ILLEGAL / TRAP / MISALIGNED debug-halt (park 4) even with
    EVEC set, unless EXC_PASS; SYSCALL and PRIVILEGED always reach EVEC.
    (Without a DEBUG_EN core the debugger inputs are ignored.)"""
    out = []
    for word, cause, dbgable in ((0x6C040000, X_TRAP, True), (0x7C000000, X_ILL, True),
                                 (0x6C080000, X_SYSCALL, False)):
        for plus in (['+DBG_ATTACHED'], ['+DBG_ATTACHED', '+DBG_EXCPASS']):
            a = Asm()
            _xsetup(a)
            a.setr(5, 1)
            at = a.raw(word)
            a.setr(5, 2)
            a.halt()
            _xhandler(a)
            if _DBG and dbgable and '+DBG_EXCPASS' not in plus:
                exp = {'end': 'park', 'kind': '4', 'pc': at, 'r5': 1}
            elif _DBG:                       # the final HALT is a debug halt too
                exp = {'end': 'park', 'kind': '4', 'r5': 2, m(XD): cause}
            else:
                exp = {'end': 'park', 'kind': '0', 'r5': 2, m(XD): cause}
            out.append((a, exp, plus))
    return out


# --------------------------------------------------------------- (e) LR/SC
@test
def lrsc_basic():
    out = []
    for lat in (1, 3, 7):
        a = Asm()
        a.setr(1, DATA); a.setr(2, DATA + 8); a.setr(14, DATA + 16)
        a.setr(3, 0x1111); a.st(64, 3, 1)
        a.setr(3, 0x2222); a.st(64, 3, 2)
        a.setr(3, 0x80000001); a.st(64, 3, 14)
        a.setr64(5, 0xAAAABBBBCCCCDDDD); a.setr(4, 0x5A5A5A5A)
        a.lr(64, 3, 1)                 # back to back: the SC sees the LR in MEM
        a.sc(64, 6, 5, 1)              # ok -> 0
        a.sc(64, 7, 5, 1)              # reservation gone -> 1
        a.lr(64, 8, 1)
        a.sc(64, 9, 5, 2)              # other doubleword -> 1, no store
        a.lr(32, 10, 2)                # zero-extended
        a.lr(32, 11, 14, sgn=True)     # sign-extended
        a.nop()                        # the SC sees the LR in WB
        a.sc(32, 12, 4, 14)            # SC.W to the reserved word -> 0 (its low half)
        a.lr(64, 13, 1); a.nop(); a.nop(); a.nop()
        a.sc(64, 15, 4, 1)             # committed reservation -> 0
        a.halt()
        out.append((a, {'end': 'park', 'kind': '0', 'r3': 0x1111, 'r6': 0, 'r7': 1,
                        'r8': 0xAAAABBBBCCCCDDDD, 'r9': 1, 'r10': 0x2222,
                        'r11': 0xFFFFFFFF80000001, 'r12': 0, 'r13': 0xAAAABBBBCCCCDDDD,
                        'r15': 0, m(DATA): 0x5A5A5A5A, m(DATA + 8): 0x2222,
                        m(DATA + 16): 0x5A5A5A5A}, [f'+LAT={lat}']))
    return out


@test
def lrsc_cancel():
    """an exception (SYSCALL), an interrupt or an intervening SC between LR
    and SC makes the SC fail; an IRET does too (it ends every handler)"""
    out = []
    a = Asm()
    _xsetup(a)
    a.setr(1, DATA); a.setr(5, 0x77)
    a.lr(64, 3, 1); a.syscall(); a.sc(64, 6, 5, 1)
    a.lr(64, 3, 1); a.sc(64, 7, 5, 1); a.sc(64, 8, 5, 1)
    a.halt()
    _xhandler(a)
    out.append((a, {'end': 'park', 'kind': '0', 'r6': 1, 'r7': 0, 'r8': 1, m(DATA): 0x77}, []))
    a = Asm()
    a.setr(1, DATA); a.setr(5, 0x77)
    a.set_int_mask(1)
    a.lr(64, 3, 1)
    a.setr(2, 300)
    a.label('w'); a.dec(2); a.fbr('ne', 2, 0, 'w', imm=True)   # IRQs taken here
    a.sc(64, 6, 5, 1)
    a.halt()
    out.append((a, {'end': 'park', 'kind': '0', 'r6': 1, m(DATA): 0, 'm00180000': 'nonzero'},
                ['+IRQ_PERIOD=60']))
    return out


@test
def lrsc_faults():
    out = []
    # misaligned LR.D / SC.D / SC.W: rd and memory untouched, cause, EADDR
    for kind, cause, ea in (('lrd', X_MISAL_LD, DATA + 4), ('scd', X_MISAL_ST, DATA + 4),
                            ('scw', X_MISAL_ST, DATA + 2), ('lrw', X_MISAL_LD, DATA + 2)):
        a = Asm()
        _xsetup(a)
        a.setr(1, ea); a.setr(3, 0x33); a.setr(5, 0x55)
        at = {'lrd': lambda: a.lr(64, 3, 1), 'scd': lambda: a.sc(64, 3, 5, 1),
              'scw': lambda: a.sc(32, 3, 5, 1), 'lrw': lambda: a.lr(32, 3, 1)}[kind]()
        a.halt()
        _xhandler(a)
        exp = {'end': 'park', 'kind': '0', 'r3': 0x33, m(DATA): 0}
        exp.update(_xexp(cause, ea, at))
        out.append((a, exp, []))
    # LR / SC to MMIO fault in supervisor mode too; nothing reaches the bus
    for kind, cause in (('lr', X_ACC_LD), ('sc', X_ACC_ST)):
        a = Asm()
        _xsetup(a)
        a.setr(1, 0xF0010000); a.setr(3, 0x33); a.setr(5, 0x55)
        at = a.lr(64, 3, 1) if kind == 'lr' else a.sc(64, 3, 5, 1)
        a.halt()
        _xhandler(a)
        exp = {'end': 'park', 'kind': '0', 'r3': 0x33, 'mmio_rd': '0', 'mmio_wr': '0'}
        exp.update(_xexp(cause, 0xF0010000, at))
        out.append((a, exp, []))
    # a faulting SC leaves rd alone; its exception entry clears the
    # reservation (MMU_PLAN.md §3.3 step 4), so the retried SC fails
    a = Asm()
    _xsetup(a)
    a.setr(1, DATA); a.setr(2, DATA + 4); a.setr(5, 0x55); a.setr(6, 0x66)
    a.lr(64, 3, 1); a.sc(64, 6, 5, 2); a.sc(64, 7, 5, 1)
    a.halt()
    _xhandler(a)
    out.append((a, {'end': 'park', 'kind': '0', 'r6': 0x66, 'r7': 1, m(DATA): 0}, []))
    # EVEC = 0: a misaligned LR parks 5; an LR to MMIO parks as ILLEGAL
    a = Asm(); a.setr(1, DATA + 4); at = a.lr(64, 3, 1); a.halt()
    out.append((a, {'end': 'park', 'kind': '5', 'pc': at}, []))
    a = Asm(); a.setr(1, 0xF0010000); at = a.lr(64, 3, 1); a.halt()
    out.append((a, {'end': 'park', 'kind': '2', 'pc': at, 'mmio_rd': '0'}, []))
    # reserved encodings with [12] = 1 stay illegal: LR.D with SGN, SC with
    # SGN, 16-bit size, A = 1, LR with rs2 != 0
    for w in (0x5B801310, 0x5F801615, 0x59001310, 0x5B101310, 0x5B001311):
        a = Asm(); at = a.raw(w); a.halt()
        out.append((a, {'end': 'park', 'kind': '2', 'pc': at}, []))
    return out


def _cas_loop(a, n, cnt='L'):
    a.setr(1, DATA); a.setr(7, n); a.setr(9, 0)
    a.label(cnt)
    a.lr(64, 3, 1); a.addi_s(4, 3, 1); a.sc(64, 6, 4, 1)
    a.fbr('eq', 6, 0, cnt + '_ok', imm=True)
    a.inc(9); a.jmp(cnt)
    a.label(cnt + '_ok')
    a.dec(7); a.fbr('ne', 7, 0, cnt, imm=True)


@test
def lrsc_cas_irq():
    """an LR/SC increment loop under frequent timer IRQs: every increment
    lands exactly once; some SCs fail and retry"""
    out = []
    for lat, per in ((1, 61), (3, 151), (7, 307)):   # (shorter periods livelock at lat 7: §5.1)
        a = Asm()
        a.set_int_mask(1)
        _cas_loop(a, 300)
        a.halt()
        out.append((a, {'end': 'park', 'kind': '0', m(DATA): 300, 'r9': 'nonzero'},
                    [f'+LAT={lat}', f'+IRQ_PERIOD={per}']))
    return out


@test
def lrsc_user():
    def body(a):
        _cas_loop(a, 50)
        a.ld(64, 5, 1)
    return _ufault(body, {'r5': 50, m(DATA): 50, m(FLOG): X_PRIV}, ['+LAT=3'])


# ------------------------- clarifications recorded in MMU_PLAN.md (507990d)
@test
def clar_park_keeps_srs():
    """parking with EVEC = 0 does not write ECAUSE / EADDR"""
    out = []
    for kind, emit in (('1', lambda a: a.trap()), ('2', lambda a: a.raw(0x7C000000)),
                       ('2', lambda a: a.syscall()),
                       ('5', lambda a: (a.setr(2, DATA + 1), a.ld(16, 3, 2)))):
        a = Asm()
        a.setr(1, 0x55); a.mtsr('ecause', 1)
        a.setr(1, 0x1234); a.mtsr('eaddr', 1)
        emit(a)
        a.halt()
        out.append((a, {'end': 'park', 'kind': kind, 'ecause': '55', 'eaddr': '00001234'}, []))
    return out


@test
def clar_user_illegal_sr():
    """legality before privilege: MTSR CYCLE and SR numbers above 8 are
    ILLEGAL in user mode too (not PRIVILEGED)"""
    words = (0x6C608010,          # MTSR cycle, r1
             0x6C409100,          # MFSR r1, sr9
             0x6C609010,          # MTSR sr9, r1
             0x6C4FF100)          # MFSR r1, sr255
    def body(a):
        for w in words:
            a.raw(w)
    exp = {m(FLOG + 0x30 * i): X_ILL for i in range(len(words))}
    exp.update({m(FLOG + 0x30 * i + 8): w for i, w in enumerate(words)})
    exp[m(FLOG + 0x30 * len(words))] = X_PRIV       # the closing HALT
    return _ufault(body, exp)


@test
def clar_priority():
    """MISALIGNED_FETCH > ACCESS_FETCH; MISALIGNED_LOAD/STORE > ACCESS_*"""
    out = []

    def fe(a):                     # misaligned AND in MMIO
        a.leapc(10, 'ok'); a.setr(5, 0xF0010002); a.jmpr(5)
        a.label('ok'); a.setr(6, 0x60)
    out.append(_ufault(fe, {'r6': 0x60, m(FLOG): X_MISAL_FE, m(FLOG + 8): 0xF0010002}))

    def mld(a):                    # misaligned 16-bit load from MMIO
        a.setr(9, 0xF0010001); a.setr(10, 0xF0010001); a.setr(3, 0x33)
        a.ld(16, 3, 9)
    out.append(_ufault(mld, {'r3': 0x33, m(FLOG): X_MISAL_LD, m(FLOG + 8): 0xF0010001}))

    def mst(a):                    # misaligned 32-bit store to MMIO
        a.setr(9, 0xF0010002); a.setr(1, 0x11)
        a.st(32, 1, 9)
    out.append(_ufault(mst, {m(FLOG): X_MISAL_ST, m(FLOG + 8): 0xF0010002}))

    def mlr(a):                    # misaligned LR.D to MMIO
        a.setr(9, 0xF0010004); a.setr(3, 0x33)
        a.lr(64, 3, 9)
    out.append(_ufault(mlr, {'r3': 0x33, m(FLOG): X_MISAL_LD, m(FLOG + 8): 0xF0010004}))
    return out


@test
def clar_eaddr_raw():
    """EADDR is the raw EA: A = 1 64-bit accesses (whose port address is
    aligned down) and sub-word stores report the address as computed"""
    def ld64a(a):        # LDIDX64A r3, [r9 + 3]
        a.setr64(3, 0x0102030405060708); a.st(64, 3, 12)
        a.setr(10, GOOD - 3); a.setr(9, 0xF0010000); a.setr(3, 0)
        a.raw(0x80000000 | 6 << 26 | 3 << 24 | 1 << 21 | 1 << 20 | 3 << 8 | 9 << 4, 3)
    def st64a(a):        # STIDX64A [r9 + 5], r1
        a.setr(10, GOOD - 5); a.setr(9, 0xF0010000); a.setr(1, 0x77)
        a.raw(0x80000000 | 7 << 26 | 3 << 24 | 1 << 21 | 1 << 20 | 1 << 8 | 9 << 4, 5)
    def st8(a):
        a.setr(10, GOOD); a.setr(9, 0xF0010000); a.setr(1, 0x61)
        a.st(8, 1, 9, 7)
    def st16(a):
        a.setr(10, GOOD); a.setr(9, 0xF0010000); a.setr(1, 0x6162)
        a.st(16, 1, 9, 6)
    return [_ufault(ld64a, {'r3': 0x0102030405060708, m(FLOG): X_ACC_LD, m(FLOG + 8): 0xF0010003}),
            _ufault(st64a, {m(GOOD): 0x77, m(FLOG): X_ACC_ST, m(FLOG + 8): 0xF0010005}),
            _ufault(st8, {m(GOOD): 0x61 << 56, m(FLOG): X_ACC_ST, m(FLOG + 8): 0xF0010007}),
            _ufault(st16, {m(GOOD): 0x6162 << 48, m(FLOG): X_ACC_ST, m(FLOG + 8): 0xF0010006})]


@test
def clar_misal_fetch_no_halt():
    """with the debugger attached (no EXC_PASS), MISALIGNED_FETCH and
    ACCESS_* still go to EVEC; only ILLEGAL / TRAP / MISALIGNED_LOAD/STORE
    debug-halt"""
    out = []

    def fe(a):
        a.leapc(10, 'ok'); a.leapc(5, 'ok'); a.addi_s(5, 5, 2); a.jmpr(5)
        a.label('ok'); a.setr(6, 0x60)
    def acc(a):
        a.setr(10, GOOD); a.setr(9, 0xF0010000); a.ld(64, 3, 9)
    for body, cause in ((fe, X_MISAL_FE), (acc, X_ACC_LD)):
        a, exp, plus = _ufault(body, {m(FLOG): cause}, ['+DBG_ATTACHED'])
        if _DBG:                   # the closing HALT (in the handler) is a debug halt
            exp['kind'] = '4'
        exp.pop('mmio_rd', None)
        out.append((a, exp, plus))
    # control: a misaligned LOAD does debug-halt when attached
    def mld(a):
        a.setr(9, DATA + 1); a.ld(16, 3, 9)
    a, exp, plus = _ufault(mld, {}, ['+DBG_ATTACHED'])
    if _DBG:
        exp = {'end': 'park', 'kind': '4'}
    else:
        exp[m(FLOG)] = X_MISAL_LD
    out.append((a, exp, plus))
    return out


# ------------------------------------- review finding: squashed queued store
@test
def sq_spec_squashed_store():
    """S1 (posted speculatively) patches the dword holding S2 to a NOP; S2,
    already in MEM when S1's SMC squash lands, must not reach memory —
    neither through the store queue (sq_enq_now) nor chained onto the port
    as S1 drains.  (Also with S2 = an SC.)  From the independent review's
    t_sq.py repro; fails on b6fa71c / master."""
    NOP = 0x6C000000
    out = []
    for sc in (False, True):
        for fill in range(0, 6):
            for lat in (1, 3, 6):
                a = Asm()
                a.setr(4, DATA); a.setr(3, 0x55); a.setr(10, 0)
                if sc:
                    a.lr(64, 9, 4)
                a.leapc(5, 's1')
                a.setr(7, 0); a.setr(6, NOP); a.shift_n('shl', 6, 6, 32)
                a.ld(64, 7, 5)
                a.alui('and', 7, 7, 0xFFFFFFFF, sgn=False)
                a.alu('or', 6, 6, 7)           # {NOP, S1}
                for _ in range(fill):
                    a.nop()
                while (a.pc & 7) != 0:
                    a.nop()
                a.label('s1')
                a.st(64, 6, 5)                 # S1: patch S2's word
                if sc:
                    a.sc(64, 10, 3, 4)         # S2 = SC.D r10, r3, [r4]
                else:
                    a.st(64, 3, 4)             # S2 = ST [r4], r3
                a.nop(); a.nop()
                a.halt()
                out.append((a, {'end': 'park', 'kind': '0', m(DATA): 0, 'r10': 0}, [f'+LAT={lat}']))
    return out

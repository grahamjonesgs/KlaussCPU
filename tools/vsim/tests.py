"""Directed tests for tools/vsim/run.py.  Each test returns (Asm, expected
final-state dict, plusargs) or a list of them.  Expected keys: r0..r15 (int),
m<addr8hex> (int, dword at addr), kind / end / pc (park), any `key=value`
token the testbench writes to its .out file."""
from kasm import Asm, H

DATA = 0x10000          # the TB dumps 0x10000..0x10800
ALL = {}


def test(fn):
    ALL[fn.__name__] = fn
    return fn


# the MPU is an optional build feature, off on the Nexys A7: its tests run
# only in a VSIM_MPU_EN=1 build (run.py elaborates MPU_EN=1 then)
import os as _os0
MPU = _os0.environ.get('VSIM_MPU_EN') == '1'


# the TLB's entry count: VSIM_TLB_N (run.py elaborates TLB_N with it), else
# pipeline_core's default (32 on the xc7a100t, MMU_PLAN §11.5)
def _tlb_n():
    import re as _re
    if _os0.environ.get('VSIM_TLB_N'):
        return int(_os0.environ['VSIM_TLB_N'])
    src = open(_os0.path.join(_os0.path.dirname(_os0.path.abspath(__file__)), '..', '..',
                              'KlaussCPU.srcs', 'sources_1', 'new', 'pipeline_core.sv')).read()
    return int(_re.search(r'parameter int TLB_N = (\d+)', src).group(1))


# core 1's I-cache: VSIM_IC=<lines>x<line bytes> (run.py elaborates it),
# else pipeline_core's default (2048 x 32 B = 64 KiB, HW_PERF_PLAN §5)
def _ic_geom():
    import re as _re
    v = _os0.environ.get('VSIM_IC', '')
    if v:
        n, b = v.lower().split('x')
        return int(n), int(b)
    src = open(_os0.path.join(_os0.path.dirname(_os0.path.abspath(__file__)), '..', '..',
                              'KlaussCPU.srcs', 'sources_1', 'new', 'pipeline_core.sv')).read()
    n = _re.search(r'parameter int IC_LINES = (\d+)', src)
    b = _re.search(r'parameter int IC_LB\s*= (\d+)', src)
    return int(n.group(1)), (int(b.group(1)) if b else 16)


IC_LINES, IC_LB = _ic_geom()
IC_BYTES = IC_LINES * IC_LB
IC32 = IC_LB == 32          # 32 B lines, installed whole (HW_PERF_PLAN §5)
NOWALK = _os0.environ.get('VSIM_TLB_WALK') == '0'     # TLB_EN = 1, TLB_WALK = 0 (MMU_PLAN §11.8)
TLB_N = _tlb_n()
TLB_IW = TLB_N.bit_length() - 1          # TLB_INDEX bits (log2 N)


def mpu_test(fn):
    if MPU:
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
    # (32 B lines: the setup line and the loop's line; the exit is in the latter)
    if IC32:
        return a, {'end': 'park', 'kind': '0', 'ic_miss': '2', 'ic_fill': '10', 'bus_rd': '2'}, []
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
    a.setr(1, 0x100)
    a.mtsr('status', 1)              # [8] MPU_EN only ([9] TE would translate the next fetch)
    a.mfsr(10, 'status')
    a.halt()
    # (STATUS[8] reads 0 and ignores writes in an MPU-less build)
    exp = {'end': 'park', 'kind': '0', 'r3': 0, 'r5': 2, 'r6': 0, 'r10': 0x100 if MPU else 0,
           'r9': 'nonzero'}
    # (ECAUSE is 9 bits: [8] GUARD, stage 2)
    want = {'evec': 0x12345674, 'ecause': 0x1A5, 'eaddr': 0xDEADBEEF, 'usp': 0x07F00010,
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
        out.append((a, {'end': 'park', 'kind': kind, 'ecause': '055', 'eaddr': '00001234'}, []))
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


# ------------------------------ review finding: IRET to a user 0xF... target
@test
def iret_user_mmio_target():
    """IRET to user mode at T in 0xF...: a lookup pending from before the
    IRET, whose index matches T's, must not issue a real fill at T in the
    cycle the IRET sits in WB (UM not yet committed).  ACCESS_FETCH at T and
    no MMIO read.  (From the independent review's t_iret.py; the
    non-matching index is the control.)"""
    out = []
    FR = DATA + 0x200
    for match in (True, False):
        for pad in range(0, 4):
            for lat in (1, 3, 6):
                a = Asm()
                a.leapc(1, 'h'); a.mtsr('evec', 1)
                a.setr(1, USP); a.mtsr('usp', 1)
                a.setr(4, FR); a.setsp(4)
                for _ in range(pad):
                    a.nop()
                while (a.pc & 0xF) != 0xC:     # IRET at offset 0xC of a 16 B line
                    a.nop()
                A = a.pc
                T = 0xF0000000 | (((A + 4) & 0x1FF0) if match else (((A + 4) ^ 0x800) & 0x1FF0))
                a.iret()
                a.label('h')
                a.setr(14, XD)
                a.mfsr(13, 'ecause'); a.st(64, 13, 14, 0)
                a.mfsr(13, 'eaddr');  a.st(64, 13, 14, 8)
                a.halt()
                a.dword(FR, (1 << 44) | T)
                out.append((a, {'end': 'park', 'kind': '0', 'mmio_rd': '0', 'um': '0',
                                m(XD): X_ACC_FE, m(XD + 8): T}, [f'+LAT={lat}']))
    return out


# ------------------- MIN/MAX select in MEM (timing): values and forwarding
@test
def minmax_fwd():
    """MIN/MAX/MINU/MAXU with consumers at distance 1, 2 and 3 (forwarded
    from MEM / WB / the register file) and a store of the result"""
    def s64(v):
        return v - (1 << 64) if v >> 63 else v
    vals = [(5, 7), (7, 5), (0xFFFFFFFFFFFFFFFF, 1), (1, 0xFFFFFFFFFFFFFFFF),
            (0x8000000000000000, 0x7FFFFFFFFFFFFFFF), (3, 3)]
    out = []
    for lat in (1, 3):
        a = Asm()
        a.setr(12, DATA)
        exp = {'end': 'park', 'kind': '0'}
        i = 0
        for x, y in vals:
            for op in ('min', 'max', 'minu', 'maxu'):
                f = {'min': lambda: x if s64(x) < s64(y) else y, 'max': lambda: x if s64(x) > s64(y) else y,
                     'minu': lambda: min(x, y), 'maxu': lambda: max(x, y)}[op]
                r = f()
                for dist in (1, 2, 3):
                    a.setr64(1, x); a.setr64(2, y)
                    a.minmax(op, 3, 1, 2)
                    for _ in range(dist - 1):
                        a.nop()
                    a.alu('xor', 4, 3, 3)          # 0, consumes r3 twice
                    a.alu('add', 4, 4, 3)          # r4 = r3
                    a.st(64, 4, 12, 8 * i)
                    exp[m(DATA + 8 * i)] = r
                    i += 1
        a.halt()
        out.append((a, exp, [f'+LAT={lat}']))
    return out


@test
def sr_reset_values():
    """after reset every SR except CYCLE reads 0 (the LUTRAM-backed ones via
    their valid flops), and a written SR reads back"""
    a = Asm()
    a.setr(12, DATA)
    for i, sr in enumerate(('status', 'evec', 'ecause', 'eaddr', 'usp', 'scratch0', 'scratch1', 'tp')):
        a.mfsr(1, sr); a.st(64, 1, 12, 8 * i)
    a.setr64(2, 0x123456789ABCDEF0)
    a.mtsr('scratch1', 2); a.mfsr(3, 'scratch1')
    a.mfsr(4, 'scratch0'); a.mfsr(5, 'eaddr')      # still 0
    a.mfsr(6, 'cycle')
    a.halt()
    exp = {'end': 'park', 'kind': '0', 'r3': 0x123456789ABCDEF0, 'r4': 0, 'r5': 0, 'r6': 'nonzero'}
    exp.update({m(DATA + 8 * i): 0 for i in range(8)})
    return a, exp, []


# =========================================== MMU stage 2: MPU (MMU_PLAN §10)
from kasm import MPU_EN as A_EN, MPU_R as A_R, MPU_W as A_W, MPU_X as A_X, MPU_G as A_G
G_ = 0x100          # ECAUSE[8] GUARD


def _mpu_reg(a, n, base, limit, attr):
    """region n covers [base, limit] by 32-byte granule (low bits ignored)"""
    a.setr(1, (base & ~0x1F) | attr); a.mtsr(0x10 + 2 * n, 1)
    a.setr(1, limit); a.mtsr(0x11 + 2 * n, 1)


def _mpu_on(a, on=True):
    a.setr(1, 0x100 if on else 0); a.mtsr('status', 1)


def _mpuhandler(a, name='h'):
    """logs like _fixhandler (FLOG + 0x30*i: ECAUSE incl. GUARD, EADDR,
    USP, r15, r9, frame); a user HALT halts; a fetch fault resumes at r10;
    anything else steps over its (1-word) instruction"""
    a.label(name)
    a.push(13); a.push(14)
    a.setr(14, XD)
    a.ld(64, 13, 14, 32); a.inc(13); a.st(64, 13, 14, 32)
    a.mulv_s(13, 13, 0x30); a.alu('add', 13, 13, 14); a.addi_s(13, 13, 0x100 - 0x30 - 128)
    a.addi_s(13, 13, 127); a.addi_s(13, 13, 1)
    a.mfsr(14, 'ecause'); a.st(64, 14, 13, 0)
    a.mfsr(14, 'eaddr');  a.st(64, 14, 13, 8)
    a.mfsr(14, 'usp');    a.st(64, 14, 13, 16)
    a.st(64, 15, 13, 24); a.st(64, 9, 13, 32)
    a.getsp(14); a.ld(64, 14, 14, 16); a.st(64, 14, 13, 40)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, name + '_nh'); a.halt()
    a.label(name + '_nh')
    a.mfsr(13, 'ecause'); a.alui('and', 13, 13, 0xFF, sgn=False)
    a.fbr('eq', 13, X_SYSCALL, name + '_ret', imm=True)
    a.addi_s(14, 13, -X_ACC_FE); a.fbr('ule', 14, 1, name + '_fe', imm=True)    # 9 / 10
    a.fbr('eq', 14, X_TLB_FE - X_ACC_FE, name + '_fe', imm=True)             # 13: TLB_MISS_FE
    a.getsp(13); a.ld(64, 14, 13, 16); a.addi_s(14, 14, 4); a.st(64, 14, 13, 16)
    a.jmp(name + '_ret')
    a.label(name + '_fe')
    a.getsp(13); a.ld(64, 14, 13, 16)
    a.shift_n('shr', 14, 14, 32); a.shift_n('shl', 14, 14, 32); a.alu('or', 14, 14, 10)
    a.st(64, 14, 13, 16)
    a.label(name + '_ret')
    a.pop(14); a.pop(13)
    a.iret()


CODE_RX = (0, 0, 0xFFFF, A_EN | A_R | A_X)          # region 0: the program (user code)


def _mpu(body, regions, user=True, exp=None, plus=(), mpu=True, logs=()):
    """regions: (n, base, limit, attr); logs: expected FLOG entries
    (ECAUSE incl. GUARD, EADDR) in order; the closing user HALT adds one"""
    a = Asm()
    _xsetup(a)
    for (n, b, l, at) in regions:
        _mpu_reg(a, n, b, l, at)
    if mpu:
        _mpu_on(a)
    if user:
        _to_user(a, 'u')
    a.label('u')
    a.setr(12, GOOD)
    body(a)
    a.halt()
    _mpuhandler(a)
    e = {'end': 'park', 'kind': '0'}
    for i, (c, ea) in enumerate(logs):
        e[m(FLOG + 0x30 * i)] = c
        if ea is not None:
            e[m(FLOG + 0x30 * i + 8)] = ea
    if user:
        e[m(FLOG + 0x30 * len(logs))] = X_PRIV
        e[m(XD + 32)] = len(logs) + 1
    else:
        e[m(XD + 32)] = len(logs)
    e.update(exp or {})
    return a, e, list(plus)


@mpu_test
def mpu_sr_access():
    """reset values, BASE/LIMIT read-back (LIMIT[4:0] read 0), STATUS[8],
    and the MPU SRs are privileged in user mode"""
    a = Asm()
    _xsetup(a)
    a.setr(12, DATA); a.setr(3, 0)
    for sr in range(0x10, 0x30):
        a.mfsr(2, sr); a.alu('or', 3, 3, 2)            # all 0 after reset
    a.mfsr(4, 'status')
    a.setr(1, 0x1234567F); a.mtsr('mpu_base0', 1); a.mfsr(5, 'mpu_base0')
    a.setr(1, 0xABCDEF7F); a.mtsr('mpu_limit0', 1); a.mfsr(6, 'mpu_limit0')
    a.setr(1, 0xFFFFFFE1); a.mtsr('mpu_base15', 1); a.mfsr(7, 'mpu_base15')
    a.setr(1, 0x00000020); a.mtsr('mpu_limit15', 1); a.mfsr(8, 'mpu_limit15')
    a.mfsr(9, 'mpu_base1')                              # untouched: 0
    _mpu_on(a); a.mfsr(10, 'status'); _mpu_on(a, False); a.mfsr(11, 'status')
    a.st(64, 3, 12)                                     # (_to_user reuses r1-r3)
    _to_user(a, 'u')
    a.label('u')
    a.mfsr(1, 'mpu_base0'); a.raw(0x6C611010)          # MTSR mpu_base0 / mpu_limit0? (0x11)
    a.mtsr('status', 1); a.mfsr(1, 'status')
    a.halt()
    _uhandler(a)
    # (r1: MFSR STATUS is allowed in user mode and returns UM = 1)
    exp = {'end': 'park', 'kind': '0', m(DATA): 0, 'r4': 0, 'r5': 0x1234567F, 'r6': 0xABCDEF60,
           'r7': 0xFFFFFFE1, 'r8': 0x20, 'r9': 0, 'r10': 0x100, 'r11': 0, 'r1': 1,
           m(XLOG): X_PRIV, m(XLOG + 8): X_PRIV, m(XLOG + 16): X_PRIV, m(XLOG + 24): X_PRIV,
           m(XD + 32): 4}
    return a, exp, []


@mpu_test
def mpu_guard_both_modes():
    """a GUARD region denies loads and stores in either mode and sets
    ECAUSE[8]; neighbouring granules are fine; in supervisor mode a GUARD
    over MMIO denies too (no device access).  In user mode an MMIO access
    under a GUARD is the stage 1 MMIO fault (GUARD = 0)."""
    GA = DATA + 0x100
    out = []
    for user in (False, True):
        def body(a):
            a.setr(9, GA); a.setr(3, 0x33); a.setr(4, 0x44)
            a.ld(64, 3, 9, 8)                 # denied
            a.st(64, 4, 9, 16)                # denied
            a.ld(64, 5, 9, 0x20)              # next granule: fine
            a.ld(64, 6, 9, -8)                # previous granule: fine
            a.setr(9, 0xF0010010)
            a.ld(64, 7, 9)                    # MMIO under a GUARD
        regs = [(0, 0, 0x7FFFFFFF, A_EN | A_R | A_W | A_X), (1, GA, GA + 0x1F, A_EN | A_G),
                (2, 0xF0010000, 0xF001001F, A_EN | A_G)]
        logs = [(G_ | X_ACC_LD, GA + 8), (G_ | X_ACC_ST, GA + 16),
                ((X_ACC_LD) if user else (G_ | X_ACC_LD), 0xF0010010)]
        out.append(_mpu(body, regs, user, {'r3': 0x33, 'mmio_rd': '0', m(GA + 16): 0}, logs=logs))
    return out


@mpu_test
def mpu_user_default_deny_union():
    """user mode: no covering grant = deny (ECAUSE[8] = 0); regions combine
    as a union (an R-only and a W-only region over the same bytes = RW); a
    covering region without the permission does not grant"""
    def body(a):
        a.setr(9, DATA); a.setr(3, 0x5A)
        a.st(64, 3, 9)                        # W from region 2: ok
        a.ld(64, 4, 9)                        # R from region 1: ok
        a.ld(64, 5, 9, 0x100)                 # region 3 (EN only): denied
        a.st(64, 3, 9, 0x200 - 8)             # no region: denied
        a.setr(9, 0x40); a.ld(64, 6, 9)       # code: R+X region 0: ok
        a.setr(9, 0x08000000 - 0x100); a.ld(64, 7, 9)    # uncovered: denied
    regs = [CODE_RX, (1, DATA, DATA + 0xFF, A_EN | A_R), (2, DATA, DATA + 0xFF, A_EN | A_W),
            (3, DATA + 0x100, DATA + 0x1FF, A_EN)]
    logs = [(X_ACC_LD, DATA + 0x100), (X_ACC_ST, DATA + 0x1F8), (X_ACC_LD, 0x08000000 - 0x100)]
    return _mpu(body, regs, True, {'r4': 0x5A, m(DATA): 0x5A, 'r5': 0, 'r7': 0}, logs=logs)


@mpu_test
def mpu_access_types():
    """per access type: R / W for loads / stores incl. stack ops by
    direction, LR (R) and SC (W); X does not grant data"""
    RO, WO = DATA + 0x100, DATA + 0x200            # R-only, W-only granules (0x40 each)
    def body(a):
        a.setr(3, 0x77); a.setr(9, RO + 0x20)
        a.st(64, 3, 9)                         # ACC_ST (R-only)
        a.lr(64, 4, 9)                         # ok
        a.sc(64, 5, 3, 9)                      # ACC_ST (SC needs W); r5 unchanged
        a.setr(9, WO + 0x20)
        a.lr(64, 6, 9)                         # ACC_LD (LR needs R)
        a.st(64, 3, 9)                         # ok
        a.setr(2, RO + 0x30); a.setsp(2)
        a.push(3)                              # ACC_ST, SP unchanged
        a.call('f')                            # ACC_ST
        a.enter(1)                             # ACC_ST
        a.pop(7)                               # ok (R)
        a.getsp(8)                             # RO + 0x38
        a.setr(2, WO + 0x30); a.setsp(2)
        a.pop(7)                               # ACC_LD
        a.ret()                                # ACC_LD
        a.setr(15, WO + 0x18); a.leave()       # ACC_LD (R15 in W-only)
        a.setr(9, 0x40); a.st(64, 3, 9)        # code is R+X: ACC_ST
        a.jmp('e')
        a.label('f'); a.ret()
        a.label('e')
    regs = [CODE_RX, (1, RO, RO + 0x3F, A_EN | A_R), (2, WO, WO + 0x3F, A_EN | A_W)]
    logs = [(X_ACC_ST, RO + 0x20), (X_ACC_ST, RO + 0x20), (X_ACC_LD, WO + 0x20),
            (X_ACC_ST, RO + 0x28), (X_ACC_ST, RO + 0x28), (X_ACC_ST, RO + 0x28),
            (X_ACC_LD, WO + 0x30), (X_ACC_LD, WO + 0x30), (X_ACC_LD, WO + 0x18), (X_ACC_ST, 0x40)]
    return _mpu(body, regs, True, {'r5': 0, 'r6': 0, 'r8': RO + 0x38, m(WO + 0x20): 0x77}, logs=logs)


@mpu_test
def mpu_two_access_edge():
    """MEMGET32 spanning two doublewords and LEAVERET at a region edge:
    both doublewords are checked; a denial of only the 2nd reports its
    start (ECAUSE[8] per that doubleword); a denied 1st reports the EA"""
    E = DATA + 0x100                               # region 1: one granule [E, E+0x1F], R
    out = []
    for guard in (False, True):
        def body(a):
            a.setr(9, E + 0x1D); a.setr(3, 0x33)
            a.ld(32, 3, 9)                         # MEMGET32 [E+0x1D]: 2nd dword E+0x20 denied
            a.setr(9, E + 0x1C); a.ld(32, 4, 9)    # within the dword: fine
            a.setr(15, E + 0x18); a.leaveret()     # [E+0x18] ok, [E+0x20] denied
            a.setr(9, E - 3); a.ld(32, 5, 9)       # 1st dword (E-8) denied: EADDR = EA
        if guard:      # everything RWX, a GUARD on the next granule
            regs = [(0, 0, 0x7FFFFFFF, A_EN | A_R | A_W | A_X), (1, E + 0x20, E + 0x3F, A_EN | A_G),
                    (2, E - 0x20, E - 1, A_EN | A_G)]
            g = G_
        else:
            regs = [CODE_RX, (1, E, E + 0x1F, A_EN | A_R)]
            g = 0
        logs = [(g | X_ACC_LD, E + 0x20), (g | X_ACC_LD, E + 0x20), (g | X_ACC_LD, E - 3)]
        out.append(_mpu(body, regs, True, {'r3': 0x33, 'r5': 0}, logs=logs))
    return out


@mpu_test
def mpu_fetch():
    """X for fetch: an instruction straddling the end of the X region faults
    at the 2nd granule's start; a jump into a non-X region faults at the
    target; a GUARD over code faults in supervisor mode (ECAUSE[8])"""
    out = []

    def body(a):
        a.leapc(10, 'ok')
        while (a.pc & 0x1F) != 0x1C:
            a.nop()
        a.label('st')
        a.setr(6, 0x66)                            # 2-word: word 2 in the next granule
        while (a.pc & 0x1F) != 0:                  # (the rest of that granule: no X)
            a.nop()
        a.label('ok'); a.setr(7, 0x77)
        a.leapc(10, 'ok2'); a.setr(5, DATA); a.jmpr(5)
        a.label('ok2'); a.setr(8, 0x88)
    # region 0: code X up to the granule holding 'st' — found by assembling once
    probe = Asm(); _xsetup(probe)
    for (n, b, l, at) in [CODE_RX, (1, DATA, DATA + 0xFF, A_EN | A_R)]:
        _mpu_reg(probe, n, b, l, at)
    _mpu_on(probe); _to_user(probe, 'u'); probe.label('u'); probe.setr(12, GOOD); body(probe)
    st = probe.labels['st']
    regs = [(0, 0, st | 0x1F, A_EN | A_X), (1, DATA, DATA + 0xFF, A_EN | A_R),
            (3, (st | 0x1F) + 1, 0xFFFF, A_EN | A_X)]
    # region 3 covers the code after the gap granule; the gap granule (st+4's) has no X
    regs[2] = (3, (st | 0x1F) + 0x21, 0xFFFF, A_EN | A_X)
    out.append(_mpu(body, regs, True, {'r6': 0, 'r7': 0x77, 'r8': 0x88},
                    logs=[(X_ACC_FE, (st | 0x1F) + 1), (X_ACC_FE, DATA)]))
    return out


@mpu_test
def mpu_fetch_guard_super():
    """supervisor mode: a GUARD over the next instruction faults its fetch
    (ACCESS_FETCH, ECAUSE[8]); MTSR STATUS takes effect from the very next
    instruction, including its fetch"""
    a = Asm()
    _xsetup(a)
    a.leapc(10, 'ok')
    a.leapc(1, 'nx'); a.alui('or', 1, 1, A_EN | A_G, sgn=False); a.mtsr('mpu_base0', 1)
    a.leapc(1, 'nx'); a.mtsr('mpu_limit0', 1)
    a.setr(2, 0x100)
    while (a.pc & 0x1F) != 0x1C:
        a.nop()
    a.mtsr('status', 2)                    # enable: the next fetch is checked
    a.label('nx'); a.setr(5, 0xBAD)
    for _ in range(7):
        a.nop()
    a.label('ok')
    a.setr(12, DATA); a.setr(3, 0x31); a.st(64, 3, 12)
    # region 1: GUARD over DATA; then MPU off on the very next instruction
    a.setr(1, DATA | A_EN | A_G); a.mtsr('mpu_base1', 1); a.setr(1, DATA); a.mtsr('mpu_limit1', 1)
    a.ld(64, 6, 12)                        # denied (GUARD, supervisor)
    a.setr(2, 0); a.mtsr('status', 2)
    a.ld(64, 7, 12)                        # MPU off: fine
    a.halt()
    _mpuhandler(a)
    nx = a.labels['nx']
    return a, {'end': 'park', 'kind': '0', 'r5': 0, 'r6': 0, 'r7': 0x31,
               m(FLOG): G_ | X_ACC_FE, m(FLOG + 8): nx,
               m(FLOG + 0x30): G_ | X_ACC_LD, m(FLOG + 0x38): DATA, m(XD + 32): 2}, []


@mpu_test
def mpu_frame_push_unchecked():
    """entry's frame push is not checked: a SYSCALL with SP just above a
    GUARD pushes its frame into the guard; the vector (which must not touch
    memory) runs and halts"""
    a = Asm()
    a.leapc(1, 'v'); a.mtsr('evec', 1)
    _mpu_reg(a, 0, GOOD - 0x20, GOOD - 1, A_EN | A_G)
    _mpu_on(a)
    a.setr(2, GOOD); a.setsp(2)
    at = a.syscall()
    a.halt()
    a.label('v')
    a.mfsr(5, 'ecause'); a.getsp(6)
    a.halt()
    return a, {'end': 'park', 'kind': '0', 'r5': X_SYSCALL, 'r6': GOOD - 8,
               m(GOOD - 8): at + 4}, []


@mpu_test
def mpu_mtsr_region_next():
    """an MTSR to a region applies to the very next instruction's access"""
    a = Asm()
    _xsetup(a)
    _mpu_on(a)
    a.setr(12, DATA); a.setr(3, 0x31); a.st(64, 3, 12)
    a.setr(1, DATA); a.mtsr('mpu_limit2', 1)
    a.setr(1, DATA | A_EN | A_G); a.mtsr('mpu_base2', 1)
    a.ld(64, 6, 12)                        # denied
    a.setr(1, 0); a.mtsr('mpu_base2', 1)
    a.ld(64, 7, 12)                        # region off: fine
    a.halt()
    _mpuhandler(a)
    return a, {'end': 'park', 'kind': '0', 'r6': 0, 'r7': 0x31,
               m(FLOG): G_ | X_ACC_LD, m(XD + 32): 1}, []


@mpu_test
def mpu_limit_edges():
    """LIMIT is the last granule, inclusive; [4:0] of BASE/LIMIT do not
    move the edges; LIMIT < BASE covers nothing"""
    B = DATA + 0x40
    def body(a):
        a.setr(9, B)
        a.ld(8, 3, 9, -1)                      # B-1: denied
        a.ld(8, 4, 9, 0)                       # ok
        a.ld(8, 5, 9, 0x3F)                    # last byte of the last granule: ok
        a.ld(8, 6, 9, 0x40)                    # denied
        a.setr(9, DATA + 0x200); a.ld(8, 7, 9) # region 2 has LIMIT < BASE: denied
    regs = [CODE_RX, (1, B + 0x1F, B + 0x3F - 0x1F, A_EN | A_R),     # BASE/LIMIT low bits set
            (2, DATA + 0x200, DATA + 0x100, A_EN | A_R)]
    regs[1] = (1, B, (B + 0x3F) | 0x1F, A_EN | A_R)
    logs = [(X_ACC_LD, B - 1), (X_ACC_LD, B + 0x40), (X_ACC_LD, DATA + 0x200)]
    return _mpu(body, regs, True, logs=logs)


@mpu_test
def mpu_off():
    """regions set but MPU_EN = 0: no MPU decision in either mode"""
    def body(a):
        a.setr(9, DATA); a.setr(3, 0x5A); a.st(64, 3, 9); a.ld(64, 4, 9)
        a.setr(9, 0x08000000 - 0x100); a.ld(64, 5, 9)
    regs = [(1, DATA, DATA + 0xFF, A_EN | A_G)]
    return _mpu(body, regs, True, {'r4': 0x5A}, mpu=False)


@mpu_test
def mpu_debugger():
    """with the debugger attached, an MPU ACCESS fault goes to EVEC with or
    without EXC_PASS (only ILLEGAL / TRAP / MISALIGNED debug-halt)"""
    out = []
    for plus in (['+DBG_ATTACHED'], ['+DBG_ATTACHED', '+DBG_EXCPASS']):
        def body(a):
            a.setr(9, DATA); a.ld(64, 3, 9)
        a, e, p = _mpu(body, [(1, DATA, DATA + 0x1F, A_EN | A_G)], False, plus=plus,
                       logs=[(G_ | X_ACC_LD, DATA)])
        if _DBG:
            e['kind'] = '4'          # the closing supervisor HALT is a debug halt
        out.append((a, e, p))
    return out


@mpu_test
def mpu_two_access_precedence():
    """both doublewords of a spanning MEMGET32 / LEAVERET fault for
    different reasons: the lower one is reported (EADDR = the EA, its
    GUARD bit), whatever the upper one's cause (MMU_PLAN §10.2 / 424e236)"""
    E = DATA + 0x100
    out = []
    # A: lower dword user-denied (no region), upper = 0xF000_0000 (MMIO rule)
    def bA(a):
        a.setr(9, 0xEFFFFFFD); a.setr(3, 0x33); a.ld(32, 3, 9)
        a.setr(15, 0xEFFFFFF8); a.leaveret()
    out.append(_mpu(bA, [CODE_RX], True, {'r3': 0x33},
                    logs=[(X_ACC_LD, 0xEFFFFFFD), (X_ACC_LD, 0xEFFFFFF8)]))
    # B: lower dword user-denied (granule E has no region), upper a GUARD
    def bB(a):
        a.setr(9, E + 0x1D); a.ld(32, 3, 9)
        a.setr(15, E + 0x18); a.leaveret()
    out.append(_mpu(bB, [CODE_RX, (1, E + 0x20, E + 0x3F, A_EN | A_G)], True,
                    logs=[(X_ACC_LD, E + 0x1D), (X_ACC_LD, E + 0x18)]))
    # C: lower dword a GUARD, upper user-denied
    out.append(_mpu(bB, [CODE_RX, (1, E, E + 0x1F, A_EN | A_G)], True,
                    logs=[(G_ | X_ACC_LD, E + 0x1D), (G_ | X_ACC_LD, E + 0x18)]))
    # D: lower dword fine (R), upper a GUARD: the upper one is reported
    out.append(_mpu(bB, [CODE_RX, (1, E, E + 0x1F, A_EN | A_R), (2, E + 0x20, E + 0x3F, A_EN | A_G)], True,
                    logs=[(G_ | X_ACC_LD, E + 0x20), (G_ | X_ACC_LD, E + 0x20)]))
    return out


# ============================================ MMU stage 3: TLB (MMU_PLAN §11)
from kasm import TLB_V, TLB_R, TLB_W, TLB_X
X_TLB_LD, X_TLB_ST, X_TLB_FE = 11, 12, 13


def _tlb_set(a, idx, vpn, pfn, xwr_v, g=0, asid=0):
    """entry[idx] <- VA page vpn -> PA page pfn (xwr_v = TLB_LO[3:0])"""
    a.setr(1, (vpn << 16) | (g << 8) | asid); a.mtsr('tlb_hi', 1)
    a.setr(1, (pfn << 16) | xwr_v); a.mtsr('tlb_lo', 1)
    a.setr(1, idx); a.mtsr('tlb_index', 1)
    a.tlbwi()


@test
def tlb_sr_and_ops():
    """TLB SRs after reset and their writable bits; TLBWI / TLBR round trip;
    TLBP hit (index, P = 0), miss (P = 1, [4:0] unchanged), G- and
    ASID-aware; a V = 0 entry never matches; TLBINV; STATUS[9]"""
    a = Asm()
    a.setr(12, DATA); a.setr(3, 0)
    for sr in ('tlb_hi', 'tlb_lo', 'tlb_index', 'asid', 'tlbvec'):
        a.mfsr(2, sr); a.alu('or', 3, 3, 2)
    a.mfsr(4, 'status')
    a.setr(1, 0xFFFFFFFF)
    for i, sr in enumerate(('tlb_hi', 'tlb_lo', 'tlb_index', 'asid', 'tlbvec')):
        a.mtsr(sr, 1); a.mfsr(2, sr); a.st(64, 2, 12, 8 * i)
    a.setr(1, 0); a.mtsr('asid', 1)
    _tlb_set(a, 5, 0x1234, 0x7FF, TLB_V | TLB_R | TLB_X, g=0, asid=0x42)
    _tlb_set(a, 9, 0x00C0, 0x001, TLB_V | TLB_W, g=1, asid=0x17)
    _tlb_set(a, 12, 0x0BAD, 0x002, 0, asid=0)                  # V = 0
    a.setr(1, 5); a.mtsr('tlb_index', 1); a.setr(1, 0); a.mtsr('tlb_hi', 1); a.mtsr('tlb_lo', 1)
    a.tlbr(); a.mfsr(5, 'tlb_hi'); a.mfsr(6, 'tlb_lo')
    a.setr(1, 9); a.mtsr('tlb_index', 1); a.tlbr(); a.mfsr(7, 'tlb_hi'); a.mfsr(8, 'tlb_lo')
    a.setr(1, 3); a.mtsr('tlb_index', 1)
    a.setr(1, (0x1234 << 16) | 0x42); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(9, 'tlb_index')   # hit: 5
    a.setr(1, (0x1234 << 16) | 0x43); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(10, 'tlb_index')  # ASID: miss
    a.setr(1, (0x00C0 << 16) | 0x99); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(11, 'tlb_index')  # G: hit 9
    a.setr(1, 0x0BAD << 16); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(13, 'tlb_index')           # V = 0: miss
    a.tlbinv()
    a.setr(1, 5); a.mtsr('tlb_index', 1); a.tlbr(); a.mfsr(14, 'tlb_lo')                      # V gone
    a.setr(1, (0x1234 << 16) | 0x42); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(15, 'tlb_index')
    a.halt()
    exp = {'end': 'park', 'kind': '0', 'r3': 0, 'r4': 0,
           m(DATA): 0xFFFF01FF, m(DATA + 8): 0x07FF000F, m(DATA + 16): 0x80000000 | (TLB_N - 1),  # (log2 N index bits)
           m(DATA + 24): 0xFF, m(DATA + 32): 0xFFFFFFFC,
           'r5': (0x1234 << 16) | 0x42, 'r6': (0x7FF << 16) | TLB_V | TLB_R | TLB_X,
           'r7': (0x00C0 << 16) | 0x100 | 0x17, 'r8': (0x001 << 16) | TLB_V | TLB_W,
           'r9': 5, 'r10': 0x80000005, 'r11': 9, 'r13': 0x80000009,
           'r14': (0x7FF << 16) | TLB_R | TLB_X, 'r15': 0x80000005}
    return a, exp, []


@test
def tlb_user_priv():
    """the TLB instructions and SRs are privileged in user mode"""
    a = Asm()
    _xsetup(a)
    _to_user(a, 'u')
    a.label('u')
    a.tlbwi(); a.tlbr(); a.tlbp(); a.tlbinv()
    a.mfsr(1, 'tlb_hi'); a.mtsr('asid', 1); a.mfsr(1, 'tlbvec')
    a.halt()
    _uhandler(a)
    exp = {'end': 'park', 'kind': '0', m(XD + 32): 8}
    for i in range(8):
        exp[m(XLOG + 8 * i)] = X_PRIV
    return a, exp, []


@test
def tlb_reservation_clears():
    """MTSR ASID (same value), MTSR STATUS and TLBWI / TLBINV clear the LR/SC
    reservation; MTSR TP and TLBR / TLBP do not"""
    a = Asm()
    a.setr(12, DATA); a.setr(3, 0x77); a.setr(2, 0)
    for k, op in enumerate([lambda: a.mtsr('asid', 2), lambda: a.mtsr('status', 2),
                            lambda: a.tlbwi(), lambda: a.tlbinv(),
                            lambda: a.mtsr('tp', 2), lambda: a.tlbr(), lambda: a.tlbp()]):
        a.lr(64, 4, 12); op(); a.sc(64, 5, 3, 12); a.st(64, 5, 12, 8 + 8 * k)
    a.halt()
    exp = {'end': 'park', 'kind': '0'}
    for k, want in enumerate([1, 1, 1, 1, 0, 0, 0]):
        exp[m(DATA + 8 + 8 * k)] = want
    return a, exp, []


TLB_RWX = TLB_V | TLB_R | TLB_W | TLB_X
KSP = 0x88000000        # kernel SP in kseg (PA 0x0800_0000, the top of DDR)


def _te_on(a, ksp=KSP):
    """global identity entries for page 0 (code, handlers) and page 1 (the
    data window), the kernel SP into kseg, then TE (the next fetch is
    translated: through the page-0 entry)"""
    _tlb_set(a, 0, 0x0000, 0x000, TLB_RWX, g=1)
    _tlb_set(a, 1, 0x0001, 0x001, TLB_RWX, g=1)
    a.setr(1, ksp); a.setsp(1)
    a.setr(1, 0x200); a.mtsr('status', 1)


def _te_prog(body, user=False, exp=None, logs=(), plus=(), tlbvec=False, setup=None):
    a = Asm()
    _xsetup(a)
    if tlbvec:
        a.leapc(1, 'ht'); a.mtsr('tlbvec', 1)
    if setup:
        setup(a)
    _te_on(a)
    if user:
        _to_user(a, 'u')
    a.label('u')
    a.setr(12, GOOD)
    body(a)
    a.halt()
    _mpuhandler(a)
    if tlbvec:          # the TLBVEC entry: counts at XD+0x58, then the common handler
        a.label('ht')
        a.push(13); a.push(14)
        a.setr(14, XD + 0x58); a.ld(64, 13, 14); a.inc(13); a.st(64, 13, 14)
        a.pop(14); a.pop(13)
        a.jmp('h')
    e = {'end': 'park', 'kind': '0'}
    for i, (c, ea) in enumerate(logs):
        e[m(FLOG + 0x30 * i)] = c
        if ea is not None:
            e[m(FLOG + 0x30 * i + 8)] = ea
    n = len(logs) + (1 if user else 0)
    if user:
        e[m(FLOG + 0x30 * len(logs))] = X_PRIV
    e[m(XD + 32)] = n
    e.update(exp or {})
    return a, e, list(plus)


@test
def tlb_data_segments():
    """TE = 1, supervisor: useg through the TLB, kseg direct (VA - 0x8000_0000),
    kmap through the TLB, MMIO identity, the 0x8800_0000-0xBFFF_FFFF hole
    faults; a miss raises TLB_MISS_LOAD / _STORE and loads TLB_HI; a
    permission failure is ACCESS_* (TLB_HI loaded too); TLBVEC takes the
    misses only"""
    def setup(a):
        _tlb_set(a, 2, 0x0100, 0x001, TLB_V | TLB_R | TLB_W)        # 0x0100_0000 -> PA 0x10000
        _tlb_set(a, 3, 0x0300, 0x001, TLB_V | TLB_R)                # read-only alias
        _tlb_set(a, 4, 0xC000, 0x001, TLB_V | TLB_R, g=1)           # kmap
    def body(a):
        a.setr(9, 0x01000008); a.setr(3, 0x1111); a.st(64, 3, 9); a.ld(64, 4, 9)
        a.setr(9, 0x80010010); a.setr(3, 0x2222); a.st(64, 3, 9); a.ld(64, 5, 9)
        a.setr(9, 0xF0010010); a.ld(64, 6, 9)                       # MMIO identity (UART status 2)
        a.setr(9, 0xC0000008); a.ld(64, 7, 9)                       # kmap -> PA 0x10008
        a.setr(9, 0x88000000); a.ld(64, 8, 9)                       # hole: ACC_LD
        a.setr(9, 0x02000010); a.ld(64, 8, 9)                       # miss: TLB_MISS_LD
        a.st(64, 3, 9, 8)                                           # TLB_MISS_ST
        a.setr(9, 0x03000020); a.st(64, 3, 9)                       # R-only: ACC_ST
        a.mfsr(11, 'tlb_hi')
    logs = [(X_ACC_LD, 0x88000000), (X_TLB_LD, 0x02000010), (X_TLB_ST, 0x02000018),
            (X_ACC_ST, 0x03000020)]
    return _te_prog(body, False, {'r4': 0x1111, 'r5': 0x2222, 'r6': 2, 'r7': 0x1111,
                                  m(DATA + 8): 0x1111, m(DATA + 0x10): 0x2222, m(DATA + 0x20): 0,
                                  'r11': 0x03000000, m(XD + 0x58): 2},
                    logs=logs, tlbvec=True, setup=setup)


@test
def tlb_data_user_asid():
    """TE = 1, user mode: useg through the TLB; kseg and kmap are ACCESS_*;
    an entry of another ASID misses, a global one hits; MTSR ASID makes the
    ASID entry hit (from supervisor)"""
    def setup(a):
        _tlb_set(a, 2, 0x0100, 0x001, TLB_V | TLB_R | TLB_W, asid=0)
        _tlb_set(a, 3, 0x0200, 0x001, TLB_V | TLB_R, asid=5)
        _tlb_set(a, 4, 0x0400, 0x001, TLB_V | TLB_R, g=1, asid=9)
    def body(a):
        a.setr(9, 0x01000008); a.setr(3, 0x33); a.st(64, 3, 9); a.ld(64, 4, 9)
        a.setr(9, 0x80010008); a.ld(64, 5, 9)                       # kseg: ACC_LD
        a.setr(9, 0xC0000008); a.ld(64, 5, 9)                       # kmap: ACC_LD
        a.setr(9, 0x02000008); a.ld(64, 6, 9)                       # ASID 5 != 0: TLB_MISS_LD
        a.setr(9, 0x04000008); a.ld(64, 7, 9)                       # global: ok
    logs = [(X_ACC_LD, 0x80010008), (X_ACC_LD, 0xC0000008), (X_TLB_LD, 0x02000008)]
    return _te_prog(body, True, {'r4': 0x33, 'r5': 0, 'r6': 0, 'r7': 0x33}, logs=logs, setup=setup)


@test
def tlb_data_asid_switch():
    """MTSR ASID selects which non-global entries match"""
    return _te_asid()


def _te_asid():
    def setup(a):
        _tlb_set(a, 3, 0x0200, 0x001, TLB_V | TLB_R, asid=5)
    def body(a):
        a.setr(9, DATA + 8); a.setr(3, 0x44); a.st(64, 3, 9)       # PA 0x10008 (identity page 1)
        a.setr(9, 0x02000008)
        a.ld(64, 6, 9)                                              # ASID 0: miss
        a.setr(1, 5); a.mtsr('asid', 1)
        a.ld(64, 7, 9)                                              # ASID 5: hit
        a.setr(1, 0); a.mtsr('asid', 1)
    return _te_prog(body, False, {'r6': 0, 'r7': 0x44}, logs=[(X_TLB_LD, 0x02000008)], setup=setup)


@test
def tlb_two_access_pages():
    """a LEAVERET / spanning MEMGET32 crossing a page: both pages translated
    (different frames); the 2nd page unmapped reports its start; the 1st
    unmapped reports the EA even though the 2nd is mapped"""
    def setup(a):
        _tlb_set(a, 2, 0x0101, 0x002, TLB_V | TLB_R | TLB_W)        # -> PA 0x20000
        _tlb_set(a, 3, 0x0102, 0x003, TLB_V | TLB_R | TLB_W)        # -> PA 0x30000
        _tlb_set(a, 4, 0x0104, 0x004, TLB_V | TLB_R)                # 0x0105 unmapped
        _tlb_set(a, 5, 0x0107, 0x005, TLB_V | TLB_R)                # 0x0106 unmapped
    def body(a):
        a.setr(9, 0x8002FFF8); a.setr64(3, 0x1122334455667788); a.st(64, 3, 9)   # via kseg
        a.leapc(3, 'back'); a.setr(9, 0x80030000); a.st(64, 3, 9)
        a.setr(15, 0x0101FFF8); a.leaveret()                        # R15 <- [p2 + FFF8], PC <- [p3 + 0]
        a.setr(4, 0xBAD)
        a.label('back')
        a.mov(5, 15); a.getsp(6)
        a.setr(1, KSP); a.setsp(1)
        a.setr(9, 0x0101FFFD); a.ld(32, 7, 9)                       # bytes 11 22 33 of p2 end + low byte of p3's
        a.setr(15, 0x0104FFF8); a.leaveret()                        # 2nd page unmapped
        a.setr(9, 0x0106FFFD); a.ld(32, 8, 9)                       # 1st page unmapped
    back = None
    logs = [(X_TLB_LD, 0x01050000), (X_TLB_LD, 0x0106FFFD)]
    a, e, p = _te_prog(body, False, {'r5': 0x1122334455667788, 'r6': 0x01020008, 'r4': 0},
                       logs=logs, setup=setup)
    back = a.labels['back']
    e['r7'] = ((back & 0xFF) << 24) | 0x112233
    return a, e, p


@test
def tlb_double_fault():
    """TE = 1: an entry whose frame push (SP - 8) is not in kseg parks with
    kind 6 and writes none of ECAUSE / EADDR / TLB_HI; SP = 0x8800_0000 is
    fine; SP = 0x8000_0000 (push at 0x7FFF_FFF8) is not"""
    out = []
    for ksp, kind in ((0x08000000, '6'), (0x80000000, '6'), (0x88000000, '0')):
        a = Asm()
        _xsetup(a)
        a.setr(1, 0x55); a.mtsr('ecause', 1)
        _te_on(a, ksp)
        at = a.syscall()
        a.halt()
        _mpuhandler(a)
        e = {'end': 'park', 'kind': kind}
        if kind == '6':
            e.update({'pc': at + 4, 'ecause': '055', 'eaddr': '00000000'})
        out.append((a, e, []))
    return out


def _place(a, base, emit):
    """assemble emit(sub) at base (its VA's page offset = base's) and store
    its words at PA base as image data (dword-aligned)"""
    b0 = base & ~7
    sub = Asm(base=b0)
    if base & 4:
        sub.nop()
    emit(sub)
    ws = sub.assemble()
    ws += [0x6C000000] * (len(ws) & 1)
    for i in range(0, len(ws), 2):
        a.dword(b0 + 4 * i, ws[i] | (ws[i + 1] << 32))
    return sub


@test
def tlb_fetch_translate():
    """TE = 1 fetch, supervisor: code runs through its kseg alias and a kmap
    entry; an instruction straddling two useg pages mapped to different
    frames runs; a page without X is ACCESS_FETCH and one with no entry
    TLB_MISS_FETCH (through TLBVEC), both loading TLB_HI; straddling into
    an unmapped page faults at that page's start; the hole is ACCESS_FETCH
    (TLB_HI not loaded)"""
    def setup(a):
        _tlb_set(a, 2, 0xC000, 0x000, TLB_V | TLB_X, g=1)          # kmap -> the code page
        _tlb_set(a, 3, 0x0060, 0x001, TLB_V | TLB_X)                # 0x0060xxxx -> PA 0x1xxxx
        _tlb_set(a, 4, 0x0061, 0x002, TLB_V | TLB_X)                # 0x0061xxxx -> PA 0x2xxxx
        _tlb_set(a, 5, 0x0070, 0x000, TLB_V | TLB_R | TLB_W)        # the code page, no X
        _tlb_set(a, 6, 0x0080, 0x001, TLB_V | TLB_X)                # 0x0081: no entry
    def body(a):
        def far(va_or, lbl, ret):
            a.leapc(9, lbl); a.setr(5, va_or); a.alu('or', 9, 9, 5)
            a.leapc(10, ret); a.jmpr(9)
        far(0x80000000, 'kx', 'kb')
        a.label('kx'); a.setr(4, 0x44); a.jmpr(10)
        a.label('kb')
        far(0xC0000000, 'mx', 'mb')
        a.label('mx'); a.setr(3, 0x33); a.jmpr(10)
        a.label('mb')
        a.setr(9, 0x0060FFFC); a.leapc(10, 'sb'); a.jmpr(9)          # the straddle snippet
        a.label('sb')
        far(0x00700000, 'nx', 'nb')                                   # no X: ACC_FE
        a.label('nx'); a.setr(6, 0xBAD); a.jmpr(10)
        a.label('nb'); a.mfsr(8, 'tlb_hi')
        far(0x00710000, 'nx2', 'nb2')                                 # no entry: TLB_MISS_FE
        a.label('nx2'); a.setr(6, 0xBAD); a.jmpr(10)
        a.label('nb2')
        a.setr(9, 0x0080FFFC); a.leapc(10, 'sb2'); a.jmpr(9)         # straddle into 0x0081
        a.label('sb2')
        a.setr(9, 0x88000000); a.leapc(10, 'hb'); a.jmpr(9)          # the hole
        a.label('hb'); a.mfsr(11, 'tlb_hi')
    def snip(s):
        s.setr(7, 0x12345678)                                         # at 0xFFFC: imm at 0x10000
        s.addi_s(7, 7, 1)
        s.jmpr(10)
    a, e, p = _te_prog(body, False, {'r3': 0x33, 'r4': 0x44, 'r6': 0, 'r7': 0x12345679,
                                     'r8': 0x00700000, 'r11': 0x00810000, m(XD + 0x58): 2},
                       tlbvec=True, setup=setup)
    _place(a, 0x1FFFC, snip)
    L = a.labels
    logs = [(X_ACC_FE, 0x00700000 | L['nx']), (X_TLB_FE, 0x00710000 | L['nx2']),
            (X_TLB_FE, 0x00810000), (X_ACC_FE, 0x88000000)]
    for i, (c, ea) in enumerate(logs):
        e[m(FLOG + 0x30 * i)] = c
        e[m(FLOG + 0x30 * i + 8)] = ea
    e[m(XD + 32)] = len(logs)
    return a, e, p


@test
def tlb_fetch_user():
    """TE = 1 fetch, user mode: kseg, kmap (with X) and MMIO are
    ACCESS_FETCH; a useg entry of another ASID misses, a matching one runs"""
    def setup(a):
        _tlb_set(a, 2, 0xC000, 0x000, TLB_V | TLB_X, g=1)
        _tlb_set(a, 3, 0x0070, 0x000, TLB_V | TLB_X, asid=0)
        _tlb_set(a, 4, 0x0071, 0x000, TLB_V | TLB_X, asid=3)
    def body(a):
        def far(va_or, lbl, ret, reg, val):
            a.leapc(9, lbl); a.setr(5, va_or); a.alu('or', 9, 9, 5)
            a.leapc(10, ret); a.jmpr(9)
            a.label(lbl); a.setr(reg, val); a.jmpr(10)
            a.label(ret)
        far(0x80000000, 'k', 'kb', 6, 0xBAD)
        far(0xC0000000, 'c', 'cb', 6, 0xBAD)
        a.setr(9, 0xF0000000); a.leapc(10, 'fb'); a.jmpr(9)
        a.label('fb')
        far(0x00700000, 'w', 'wb', 4, 0x44)
        far(0x00710000, 'v', 'vb', 6, 0xBAD)
    a, e, p = _te_prog(body, True, {'r6': 0, 'r4': 0x44, m(XD + 0x58): 1}, tlbvec=True, setup=setup)
    L = a.labels
    logs = [(X_ACC_FE, 0x80000000 | L['k']), (X_ACC_FE, 0xC0000000 | L['c']),
            (X_ACC_FE, 0xF0000000), (X_TLB_FE, 0x00710000 | L['v'])]
    for i, (c, ea) in enumerate(logs):
        e[m(FLOG + 0x30 * i)] = c
        e[m(FLOG + 0x30 * i + 8)] = ea
    e[m(FLOG + 0x30 * len(logs))] = X_PRIV
    e[m(XD + 32)] = len(logs) + 1
    return a, e, p


@test
def tlb_fetch_alias_icinv():
    """code written through kseg and run through a different useg VA after
    ICACHE_INV; rewritten (through kseg) and run again after another
    ICACHE_INV: the new code runs"""
    def setup(a):
        _tlb_set(a, 2, 0x0050, 0x002, TLB_V | TLB_R | TLB_X)       # 0x0050xxxx -> PA 0x2xxxx
    def code(val):
        s = Asm(base=0)
        s.setr(7, val); s.jmpr(10)
        ws = s.assemble()
        ws += [0x6C000000] * (len(ws) & 1)
        return [ws[i] | (ws[i + 1] << 32) for i in range(0, len(ws), 2)]
    def body(a):
        for k, val in enumerate((0x71, 0x72)):
            a.setr(9, 0x80020100)
            for i, dw in enumerate(code(val)):
                a.setr64(3, dw); a.st(64, 3, 9, 8 * i)
            a.setr(1, 0xF0050000); a.setr(2, 8); a.st(32, 2, 1)       # ICACHE_INV
            a.setr(9, 0x00500100); a.leapc(10, f'b{k}'); a.jmpr(9)
            a.label(f'b{k}')
            a.mov(8 - k, 7)
    return _te_prog(body, False, {'r8': 0x71, 'r7': 0x72}, setup=setup)


@test
def tlb_te_on_refill():
    """TE turned on with no TLB entry: the next fetch is TLB_MISS_FETCH, a
    kseg refill handler at TLBVEC installs an identity entry from TLB_HI and
    returns; the data page then misses (TLB_MISS_STORE) and is refilled the
    same way; the program completes"""
    CNT = 0x10700
    a = Asm()
    a.leapc(1, 'rf'); a.setr(2, 0x80000000); a.alu('or', 1, 1, 2)
    a.mtsr('evec', 1); a.mtsr('tlbvec', 1)
    a.setr(1, KSP); a.setsp(1)
    a.setr(1, 0x200); a.mtsr('status', 1)                          # TE on: the next fetch misses
    a.setr(12, DATA); a.setr(3, 0x5A); a.st(64, 3, 12)              # page 1: TLB_MISS_ST
    a.ld(64, 4, 12)
    a.mfsr(5, 'ecause'); a.mfsr(6, 'eaddr')
    a.halt()
    a.label('rf')                                                   # runs at its kseg alias
    a.push(13); a.push(14)
    a.mfsr(13, 'tlb_hi')
    a.shift_n('shr', 13, 13, 16); a.shift_n('shl', 13, 13, 16)
    a.alui('or', 13, 13, TLB_RWX, sgn=False); a.mtsr('tlb_lo', 13)
    a.setr(13, 0x80000000 | CNT); a.ld(64, 14, 13); a.inc(14); a.st(64, 14, 13)
    a.mtsr('tlb_index', 14); a.tlbwi()
    a.pop(14); a.pop(13)
    a.iret()
    return a, {'end': 'park', 'kind': '0', 'r4': 0x5A, m(DATA): 0x5A, m(CNT): 2,
               'r5': X_TLB_ST, 'r6': DATA}, []


@test
def lcd_sq_spec_once():
    """review 2 #3: an LCD op in EX on the EN_ST1 squash edge (a store that
    posted speculatively overlaps code) is squashed and refetched; it must
    send its byte once, not twice (a plain-data store: once too)"""
    out = []
    for smc in (True, False):
        for lat in (1, 3):
            a = Asm()
            while a.pc % 16:
                a.nop()
            a.label('L'); a.nop(); a.nop()               # an executed code dword
            a.setr(3, 0x41)
            a.setr64(1, (0x6C000000 << 32) | 0x6C000000)  # the same two NOPs
            if smc:
                a.leapc(2, 'L')
            else:
                a.setr(2, DATA)
            for _ in range(3):
                a.nop()
            a.st(64, 1, 2)                                # posts; overlaps code -> sq_spec
            a.nop()                                       # MEM on the squash edge
            a.emit(H(1, 0xC) | 1 << 24 | 3 << 4)          # LCD data r3: EX on the squash edge
            for _ in range(4):
                a.nop()
            a.halt()
            out.append((a, {'end': 'park', 'kind': '0', 'lcd_n': '1'}, [f'+LAT={lat}']))
    return out


@test
def call_sq_spec_target():
    """A CALL whose return-address push posts speculatively (EN_ST1) and
    then turns out to overlap code (sq_spec: here SP - 8 is an executed,
    I-cache-valid code dword) squashes everything younger and refetches.
    The refetch must start at the CALL's target, not its fall-through: on
    be78350 it restarted at the fall-through and skipped the whole callee
    (Linux: bootstrap()'s memcpy got a NULL kmem_cache, early-boot panic;
    there the trigger was the tag-shadow guard's conservative sn_rc, when
    the callee's own line fill landed on the stack slot's I-cache index).
    Relative, absolute and register-indirect calls; the callee sets r6 and
    returns, the fall-through counts itself in r5."""
    out = []
    for kind in ('rel', 'abs', 'reg'):
        for fill in range(0, 4):
            for lat in (1, 3, 6):
                a = Asm()
                while a.pc % 16:
                    a.nop()
                a.label('L'); a.nop(); a.nop()        # an executed code dword
                a.setr(5, 0); a.setr(6, 0)
                a.leapc(2, 'L'); a.addi_s(2, 2, 8)
                a.setsp(2)                            # the push lands on L
                a.leapc(3, 'f')
                for _ in range(fill):
                    a.nop()
                if kind == 'rel':
                    a.call('f')
                elif kind == 'abs':
                    a.jmp_abs('f', link=True)
                else:
                    a.jmpr(3, link=True)
                a.addi_s(5, 5, 1)                     # the fall-through, once
                a.halt()
                a.label('f')
                a.setr(6, 0x77)
                a.ret()
                out.append((a, {'end': 'park', 'kind': '0', 'r5': 1, 'r6': 0x77},
                            [f'+LAT={lat}']))
    return out


@mpu_test
def wp_mpu_denied_load():
    """review 2 #2: a read watchpoint on a dword whose load the MPU denies
    (GUARD) must not fire: the load takes ACCESS_LOAD|GUARD and the program
    completes.  Control, MPU off: the same load fires the watchpoint
    (DEBUG_EN builds; without the debug module it just completes)"""
    dbg = _os.environ.get('VSIM_DEBUG_EN') == '1'
    out = []
    for mpu in (True, False):
        a = Asm()
        _xsetup(a)
        a.setr(12, DATA); a.setr(3, 0x31); a.st(64, 3, 12)
        _mpu_reg(a, 1, DATA, DATA, A_EN | A_G)
        if mpu:
            _mpu_on(a)
        for _ in range(4):
            a.nop()
        a.ld(64, 6, 12)                       # MPU on: denied -> never reads DATA
        a.setr(7, 0x77)
        a.halt()
        a.label('h')                          # record, step over, return
        a.mfsr(8, 'ecause'); a.mfsr(9, 'eaddr')
        a.getsp(13); a.ld(64, 14, 13); a.addi_s(14, 14, 4); a.st(64, 14, 13)
        a.nop(); a.nop()
        a.iret()
        if mpu:
            e = {'end': 'park', 'kind': '0', 'r6': 0, 'r7': 0x77, 'r8': G_ | X_ACC_LD, 'r9': DATA}
        elif dbg:
            e = {'end': 'park', 'kind': '4', 'dbg_cause': '3', 'watch_addr': '00010000', 'r6': 0x31}
        else:
            e = {'end': 'park', 'kind': '0', 'r6': 0x31, 'r7': 0x77}
        out.append((a, e, ['+WP0RD=%x' % DATA, '+LAT=3']))
    return out


@test
def wp_te_physical():
    """watchpoints compare physical addresses: with TE = 1 a load through
    DATA's kseg alias fires a read watchpoint set on PA DATA (port path,
    and the hit pipe with +HIT=100); DEBUG_EN builds only"""
    dbg = _os.environ.get('VSIM_DEBUG_EN') == '1'
    out = []
    for plus in ([], ['+HIT=100']):
        a = Asm()
        _xsetup(a)
        _te_on(a)
        a.setr(12, 0x80000000 | DATA); a.setr(3, 0x31); a.st(64, 3, 12)
        for _ in range(4):
            a.nop()
        a.ld(64, 6, 12)
        a.setr(7, 0x77)
        a.halt()
        _mpuhandler(a)
        e = ({'end': 'park', 'kind': '4', 'dbg_cause': '3', 'watch_addr': '00010000', 'r6': 0x31}
             if dbg else {'end': 'park', 'kind': '0', 'r6': 0x31, 'r7': 0x77})
        out.append((a, e, ['+WP0RD=%x' % DATA, '+LAT=3'] + plus))
    return out


@test
def mmio_fetch_no_device_read():
    """review 2 #1: with the MPU on (or TE = 1) a fetch fill at 0xF... is a
    fake fill: no MMIO read.  Supervisor, GUARD over the UART block:
    ACCESS_FETCH|GUARD; MPU on, no region there: zeros (ILLEGAL); TE = 1:
    zeros (ILLEGAL).  MPU and TE off: the fill reads the device, as before."""
    out = []
    for mode in (('guard', 'mpu') if MPU else ()) + ('te', 'off'):
        a = Asm()
        _xsetup(a)
        if mode == 'guard':
            _mpu_reg(a, 0, 0xF0010000, 0xF001001F, A_EN | A_G)
        if mode in ('guard', 'mpu'):
            _mpu_on(a)
        if mode == 'te':
            _te_on(a)
        a.leapc(10, 'back')
        a.setr(9, 0xF0010010); a.jmpr(9)       # supervisor jump into MMIO
        a.label('back'); a.setr(7, 0x77); a.halt()
        a.label('h')
        a.mfsr(8, 'ecause'); a.mfsr(11, 'eaddr')
        a.getsp(13); a.ld(64, 14, 13); a.setr64(12, 0xFFFFFFFF00000000); a.alu('and', 14, 14, 12)
        a.alu('or', 14, 14, 10); a.st(64, 14, 13)
        a.iret()
        e = {'end': 'park', 'kind': '0', 'r7': 0x77, 'mmio_rd': '1' if mode == 'off' else '0'}
        if mode == 'guard':
            e.update({'r8': G_ | X_ACC_FE, 'r11': 0xF0010010})
        elif mode != 'off':
            e.update({'r8': X_ILL, 'r11': 0})                 # (EADDR = the op word)
        out.append((a, e, ['+LAT=3', '+MAXC=20000']))
    return out


@test
def tlb_stlf_alias():
    """TE = 1 store-to-load forwarding: a load right after a store to the
    same PA through another VA (kseg vs useg alias) sees the store; a
    store, a TLBWI remapping its VA to another frame, then a load of that
    VA reads the new frame, not the store; a load at the same page offset
    but another PA right after a store is not forwarded; same-VA push/pop
    through the kseg stack (the values)"""
    out = []
    for plus in ([], ['+HIT=100'], ['+HIT=70', '+LAT=1']):
        def setup(a):
            _tlb_set(a, 2, 0x0050, 0x001, TLB_V | TLB_R | TLB_W)      # 0x0050xxxx -> PA 0x1xxxx
        def body(a):
            a.setr(9, 0x80000000 | DATA); a.setr(10, 0x00500000 | DATA & 0xFFFF)
            a.setr(3, 0x1111); a.st(64, 3, 9, 8); a.ld(64, 4, 10, 8)        # kseg store, useg load
            a.setr(3, 0x2222); a.st(64, 3, 10, 16); a.ld(64, 5, 9, 16)      # useg store, kseg load
            a.setr(13, 0x80020000)                                          # kseg -> PA 0x20000
            a.setr(3, 0x6666); a.st(64, 3, 10, 32); a.ld(64, 11, 13, 32)   # same offset, other PA
            a.setr(3, 0x3333); a.st(64, 3, 10, 24)
            _tlb_set(a, 2, 0x0050, 0x002, TLB_V | TLB_R | TLB_W)            # remap -> PA 0x2xxxx
            a.ld(64, 6, 10, 24)                                              # PA 0x20018: 0x7777
            a.setr(3, 0x4444); a.push(3); a.pop(7)
            a.setr(3, 0x5555); a.push(3); a.alu('add', 1, 1, 1); a.pop(8)
        a, e, p = _te_prog(body, False, {'r4': 0x1111, 'r5': 0x2222, 'r6': 0x7777, 'r7': 0x4444,
                                         'r8': 0x5555, 'r11': 0x8888, m(DATA + 24): 0x3333,
                                         m(DATA + 32): 0x6666},
                           setup=setup, plus=plus)
        a.dword(0x20018, 0x7777); a.dword(0x20020, 0x8888)
        out.append((a, e, p))
    return out


@test
def tlb_index_width():
    """TLB_INDEX holds log2 N bits ([4:0] with 32 entries, [3:0] with 16);
    the bits above them (to [30]) are ignored on write and read 0.  A TLBWI
    with index 0x7FFF_FFF3 writes entry 0x13 mod N (19 with 32 entries, so
    TLB_INDEX[4] is significant; 3 with 16), which TLBR at that index reads
    and TLBP finds; entry 0x13 - 16 (3) is a different entry"""
    E = 0x13 & (TLB_N - 1)
    a = Asm()
    a.setr(1, 0x7FFFFFF3); a.mtsr('tlb_index', 1); a.mfsr(4, 'tlb_index')
    a.setr(1, (0x0777 << 16) | 0x12); a.mtsr('tlb_hi', 1)
    a.setr(1, (0x005 << 16) | TLB_V | TLB_R); a.mtsr('tlb_lo', 1)
    a.tlbwi()
    if TLB_N > 16:
        _tlb_set(a, 3, 0x0333, 0x003, TLB_V | TLB_W, asid=0x12)
    a.setr(1, 0); a.mtsr('tlb_hi', 1); a.mtsr('tlb_lo', 1)
    a.setr(1, E); a.mtsr('tlb_index', 1); a.tlbr(); a.mfsr(5, 'tlb_hi'); a.mfsr(6, 'tlb_lo')
    a.setr(1, 0); a.mtsr('tlb_index', 1)
    a.setr(1, (0x0777 << 16) | 0x12); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(7, 'tlb_index')
    a.setr(1, (0x0333 << 16) | 0x12); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(8, 'tlb_index')
    a.halt()
    return a, {'end': 'park', 'kind': '0', 'r4': E, 'r5': (0x0777 << 16) | 0x12,
               'r6': (0x005 << 16) | TLB_V | TLB_R, 'r7': E,
               'r8': 3 if TLB_N > 16 else 0x80000000 | E}, []      # (a miss keeps the index)


@test
def tlb_entries_upper():
    """every entry 0..N-1 written by TLBWI with its own VPN / ASID / PFN /
    XWR; TLBR at each of the upper half (16-31 with 32 entries) reads its
    own entry back, and TLBP of each upper entry's VPN finds its index
    (P = 0).  TLBINV then empties all of them: a TLBP misses"""
    a = Asm()
    def hi(i): return ((0x0200 + i) << 16) | ((i & 1) << 8) | (0x40 + i)
    def lo(i): return ((0x400 + 3 * i) << 16) | ((i % 7) << 1) | TLB_V
    for i in range(TLB_N):
        a.setr(1, hi(i)); a.mtsr('tlb_hi', 1)
        a.setr(1, lo(i)); a.mtsr('tlb_lo', 1)
        a.setr(1, i); a.mtsr('tlb_index', 1)
        a.tlbwi()
    a.setr(12, DATA + 0x100)
    exp = {'end': 'park', 'kind': '0'}
    for k, i in enumerate(range(TLB_N // 2, TLB_N)):
        a.setr(1, 0); a.mtsr('tlb_hi', 1); a.mtsr('tlb_lo', 1)
        a.setr(1, i); a.mtsr('tlb_index', 1); a.tlbr()
        a.mfsr(2, 'tlb_hi'); a.st(64, 2, 12, 24 * k)
        a.mfsr(2, 'tlb_lo'); a.st(64, 2, 12, 24 * k + 8)
        a.setr(1, 0); a.mtsr('tlb_index', 1)
        a.setr(1, hi(i)); a.mtsr('tlb_hi', 1); a.tlbp()
        a.mfsr(2, 'tlb_index'); a.st(64, 2, 12, 24 * k + 16)
        exp[m(DATA + 0x100 + 24 * k)] = hi(i)
        exp[m(DATA + 0x100 + 24 * k + 8)] = lo(i)
        exp[m(DATA + 0x100 + 24 * k + 16)] = i
    a.tlbinv()
    a.setr(1, 1); a.mtsr('tlb_index', 1)
    a.setr(1, hi(TLB_N - 1)); a.mtsr('tlb_hi', 1); a.tlbp(); a.mfsr(3, 'tlb_index')
    a.halt()
    exp['r3'] = 0x80000001
    return a, exp, []


@test
def tlb_working_set():
    """TE = 1: a working set of W = N - 4 data pages (28 with 32 entries;
    every VA page aliases PA page 1) next to the identity code and data
    entries 0 and 1, refilled by a kseg handler round-robin over entries
    2..N-1.  Three passes (store, load, load) take exactly W refills: after
    the first pass every page hits (with 16 entries and W = 28 it would be
    3 W)"""
    W = TLB_N - 4
    CNT, NREF = 0x10700, 0x10708          # round-robin index, refill count
    a = Asm()
    a.leapc(1, 'rf'); a.setr(2, 0x80000000); a.alu('or', 1, 1, 2)
    a.mtsr('evec', 1); a.mtsr('tlbvec', 1)
    a.setr(1, 2); a.setr(2, CNT); a.st(64, 1, 2)
    _te_on(a)
    a.setr(11, 0x10008)                    # VA step: next page, next dword
    a.setr(9, 0x01000200); a.setr(10, W); a.setr(3, 1)
    a.label('p1')                          # pass 1: page i offset 0x200 + 8 i <- i + 1
    a.st(64, 3, 9); a.inc(3); a.alu('add', 9, 9, 11); a.dec(10)
    a.fbr('ne', 10, 0, 'p1', imm=True)
    a.setr(5, 0); a.setr(7, 2)
    a.label('pp')                          # passes 2 and 3: sum
    a.setr(9, 0x01000200); a.setr(10, W)
    a.label('p2')
    a.ld(64, 4, 9); a.alu('add', 5, 5, 4); a.alu('add', 9, 9, 11); a.dec(10)
    a.fbr('ne', 10, 0, 'p2', imm=True)
    a.dec(7); a.fbr('ne', 7, 0, 'pp', imm=True)
    a.halt()
    a.label('rf')                          # runs at its kseg alias: VPN >= 0x100 -> PA page 1
    a.push(13); a.push(14)
    a.mfsr(13, 'tlb_hi'); a.shift_n('shr', 13, 13, 16)
    a.fbr_imm32('ult', 13, 0x100, 'rf_id')
    a.setr(13, 1)
    a.label('rf_id')
    a.shift_n('shl', 13, 13, 16); a.alui('or', 13, 13, TLB_RWX, sgn=False); a.mtsr('tlb_lo', 13)
    a.setr(13, 0x80000000 | NREF); a.ld(64, 14, 13); a.inc(14); a.st(64, 14, 13)
    a.setr(13, 0x80000000 | CNT); a.ld(64, 14, 13)
    a.mtsr('tlb_index', 14); a.tlbwi()
    a.inc(14); a.fbr_imm32('ult', 14, TLB_N, 'rf_st')
    a.setr(14, 2)
    a.label('rf_st')
    a.st(64, 14, 13)
    a.pop(14); a.pop(13)
    a.iret()
    exp = {'end': 'park', 'kind': '0', 'r5': W * (W + 1), m(NREF): W}
    for i in range(W):
        exp[m(0x10200 + 8 * i)] = i + 1
    return a, exp, []


@test
def div_by_zero_operand_paths():
    """DIV / MOD by zero with the divisor from each ex_b source: the
    register file, forwarded from MEM, forwarded from WB, a load just
    completed; and a divisor zero only in its low 32 bits (not zero).
    DIV by zero = all ones, MOD by zero = the dividend (ex_b_zero)"""
    out = []
    for plus in (['+LAT=1'], ['+LAT=3'], ['+HIT=100'], ['+LAT=7', '+HIT=70']):
        a = Asm()
        a.setr(12, DATA); a.setr(1, 100); a.setr(4, 0)
        for _ in range(4):
            a.nop()
        a.div(5, 1, 4); a.mod(6, 1, 4)                    # rf
        a.setr(4, 0); a.div(7, 1, 4)                      # forwarded from MEM
        a.setr(4, 0); a.nop(); a.mod(8, 1, 4)             # from WB
        a.setr(3, 0); a.st(64, 3, 12); a.ld(64, 4, 12); a.div(9, 1, 4)   # a load's zero
        a.setr64(4, 1 << 32); a.div(10, 1, 4)             # low 32 zero, not zero: 0
        a.setr(4, 7); a.div(11, 1, 4)                     # 14
        a.halt()
        out.append((a, {'end': 'park', 'kind': '0', 'r5': 0xFFFFFFFFFFFFFFFF, 'r6': 100,
                        'r7': 0xFFFFFFFFFFFFFFFF, 'r8': 100, 'r9': 0xFFFFFFFFFFFFFFFF,
                        'r10': 0, 'r11': 14}, plus))
    return out



@test
def mpu_absent():
    """MPU-less build (the Nexys A7): MFSR / MTSR of SRs 0x10-0x2F are
    ILLEGAL (unimplemented SRs, §6.1), STATUS[8] reads 0 and ignores writes
    (MFSR STATUS and GETST), and no MPU check happens.  In an MPU build:
    the SRs work"""
    a = Asm()
    _xsetup(a)
    a.setr(1, DATA | 0x0E); a.setr(3, 0)     # (X W R, not EN)
    a.mtsr('mpu_base0', 1)                   # ILLEGAL without the MPU
    a.mfsr(3, 'mpu_base0')                   # ILLEGAL
    a.mfsr(4, 'mpu_limit15')                 # ILLEGAL (0x2F)
    a.setr(1, 0x100); a.mtsr('status', 1); a.mfsr(5, 'status')
    a.setr(12, DATA); a.setr(6, 0x66); a.st(64, 6, 12); a.ld(64, 7, 12)   # unchecked
    a.halt()
    _xhandler(a)
    if MPU:
        exp = {'end': 'park', 'kind': '0', 'r3': DATA | 0x0E, 'r4': 0, 'r5': 0x100, 'r7': 0x66}
    else:
        exp = {'end': 'park', 'kind': '0', 'r3': 0, 'r5': 0, 'r7': 0x66,
               m(XD): X_ILL, m(XD + 32): 3}
    return a, exp, []


# ===================== review 3 probes =====================
def _r3_lr_wait(pre, lead=None):
    """TE = 1 supervisor: LEAVERET with R15 at the last dword of a kseg page,
    right behind a load (WB waits on it).  Decoy frames in the following pages."""
    def setup(a):
        pass
    def body(a):
        a.setr(9, 0x8002FFF8); a.setr64(3, 0x1122334455667788); a.st(64, 3, 9)
        a.leapc(3, 'back'); a.setr(9, 0x80030000); a.st(64, 3, 9)
        for k in range(1, 8):
            a.setr(9, 0x8002FFF8 + k * 0x10000); a.setr(3, 0xDEAD0000 + k); a.st(64, 3, 9)
            a.leapc(3, 'bad'); a.setr(9, 0x80030000 + k * 0x10000); a.st(64, 3, 9)
        a.setr(15, 0x8002FFF8)
        a.setr(8, 0x80010100)
        pre(a)
        a.leaveret()
        a.setr(4, 0xBAD)
        a.label('bad')
        a.mov(5, 15); a.setr(6, 0xBAD0); a.setr(1, KSP); a.setsp(1); a.jmp('out')
        a.label('back')
        a.mov(5, 15); a.setr(6, 0x600D); a.setr(1, KSP); a.setsp(1)
        a.label('out')
    return _te_prog(body, False, {'r5': 0x1122334455667788, 'r6': 0x600D}, logs=[], setup=setup)


@test
def r3_lr_wait_ld():
    return _r3_lr_wait(lambda a: a.ld(64, 3, 8))

@test
def r3_lr_wait_ld2():
    return _r3_lr_wait(lambda a: (a.ld(64, 3, 8), a.ld(64, 2, 8, 8)))

@test
def r3_lr_wait_ldlat():
    a, e, p = _r3_lr_wait(lambda a: a.ld(64, 3, 8))
    return a, e, p + ['+LAT=9']

@test
def r3_lr_nowait():
    return _r3_lr_wait(lambda a: a.nop())

@test
def r3_mg32_wait():
    """MEMGET32 spanning a kseg page end, behind a load"""
    def body(a):
        a.setr(9, 0x8002FFF8); a.setr64(3, 0x1122334455667788); a.st(64, 3, 9)
        a.setr(9, 0x80030000); a.setr64(3, 0x99AABBCCDDEEFF00); a.st(64, 3, 9)
        for k in range(1, 8):
            a.setr(9, 0x8002FFF8 + k * 0x10000); a.setr(3, 0); a.st(64, 3, 9)
            a.setr(9, 0x80030000 + k * 0x10000); a.st(64, 3, 9)
        a.setr(9, 0x8002FFFD); a.setr(8, 0x80010100)
        a.ld(64, 3, 8)
        a.ld(32, 7, 9)
    return _te_prog(body, False, {'r7': 0x00112233}, logs=[])

def _r3_mg(nops, lat):
    def body(a):
        a.setr(9, 0x8002FFF8); a.setr64(3, 0x1122334455667788); a.st(64, 3, 9)
        a.setr(9, 0x80030000); a.setr64(3, 0x99AABBCCDDEEFF00); a.st(64, 3, 9)
        for k in range(1, 8):
            a.setr(9, 0x8002FFF8 + k * 0x10000); a.setr(3, 0); a.st(64, 3, 9)
            a.setr(9, 0x80030000 + k * 0x10000); a.st(64, 3, 9)
        a.setr(9, 0x8002FFFD); a.setr(8, 0x80010100)
        a.ld(64, 3, 8)
        for _ in range(nops): a.nop()
        a.ld(32, 7, 9)
    a, e, p = _te_prog(body, False, {'r7': 0x00112233}, logs=[])
    return a, e, p + [f'+LAT={lat}']
for _n in range(3):
    for _l in (2, 3, 6, 9):
        def _mk(n=_n, l=_l):
            return lambda: _r3_mg(n, l)
        f = _mk(); f.__name__ = f'r3_mg_{_n}_{_l}'; test(f)

def _r3_mg2(pad, lat):
    def body(a):
        a.setr(9, 0x8002FFF8); a.setr64(3, 0x1122334455667788); a.st(64, 3, 9)
        a.setr(9, 0x80030000); a.setr64(3, 0x99AABBCCDDEEFF00); a.st(64, 3, 9)
        for k in range(1, 8):
            a.setr(9, 0x8002FFF8 + k * 0x10000); a.setr(3, 0); a.st(64, 3, 9)
            a.setr(9, 0x80030000 + k * 0x10000); a.st(64, 3, 9)
        a.setr(9, 0x8002FFFD); a.setr(8, 0x80010100)
        for _ in range(pad): a.nop()
        a.ld(64, 3, 8)
        a.ld(32, 7, 9)
    a, e, p = _te_prog(body, False, {'r7': 0x00112233}, logs=[])
    return a, e, p + [f'+LAT={lat}']
for _n in range(4):
    def _mk(n=_n):
        return lambda: _r3_mg2(n, 3)
    f = _mk(); f.__name__ = f'r3_mgp_{_n}'; test(f)

@test
def r3_len00_page_end():
    """an illegal LEN=00 word (e.g. 0x00000000) in the last word of a page
    whose next page is unmapped: ILLEGAL expected (it occupies one word)"""
    def setup(a):
        _tlb_set(a, 6, 0x0080, 0x001, TLB_V | TLB_X)                # 0x0081: no entry
    def body(a):
        a.setr(9, 0x0080FFFC); a.leapc(10, 'sb2'); a.jmpr(9)
        a.label('sb2')
    def snip(s):
        s.emit(0x00000000)
    a, e, p = _te_prog(body, False, {}, setup=setup)
    _place(a, 0x1FFFC, snip)
    logs = [(X_ILL, 0), (X_TLB_FE, 0x00810000)]
    for i, (c, ea) in enumerate(logs):
        e[m(FLOG + 0x30 * i)] = c
        e[m(FLOG + 0x30 * i + 8)] = ea
    e[m(XD + 32)] = len(logs)
    return a, e, p

@test
def r3_len00_ctl():
    """control: the same word mid-page is ILLEGAL"""
    def setup(a):
        _tlb_set(a, 6, 0x0080, 0x001, TLB_V | TLB_X)
    def body(a):
        a.setr(9, 0x0080FFF4); a.leapc(10, 'sb2'); a.jmpr(9)
        a.label('sb2')
    def snip(s):
        s.emit(0x00000000); s.jmpr(10)
    a, e, p = _te_prog(body, False, {}, setup=setup)
    _place(a, 0x1FFF4, snip)
    logs = [(X_ILL, 0)]
    for i, (c, ea) in enumerate(logs):
        e[m(FLOG + 0x30 * i)] = c
        e[m(FLOG + 0x30 * i + 8)] = ea
    e[m(XD + 32)] = len(logs)
    return a, e, p

@test
def r3_mtsr_status_cost():
    """TE = 0: LR; MTSR STATUS (unchanged value); SC; and a loop with MTSR STATUS"""
    a = Asm()
    a.setr(9, 0x10000)
    a.lr(64, 2, 9)
    a.mfsr(1, 'status'); a.mtsr('status', 1)
    a.setr(3, 0x77); a.sc(64, 4, 3, 9)
    a.setr(11, 20)
    a.label('t')
    a.mtsr('status', 1); a.addi_s(5, 5, 1); a.call('f'); a.addi_s(11, 11, -1); a.fbr('ne', 11, 0, 't', imm=True)
    a.halt()
    a.label('f'); a.ret()
    return a, {'end': 'park', 'kind': '0'}, []


@test
def tlb_double_fault_irq():
    """review 3 #4: an IRQ entry with TE = 1 and SP - 8 outside kseg is a
    DOUBLE_FAULT: park 6, and the core stays parked — fetch halted, so the
    still-pending IRQ is not re-taken and nothing dispatches (the RTL
    asserts that a parked, running core has fetch halted)"""
    out = []
    for per in (60, 97):
        a = Asm()
        _xsetup(a)
        _te_on(a, 0x08000000)                       # SP not in kseg
        a.set_int_mask(1)
        a.label('l'); a.inc(5); a.jmp('l')
        a.halt()
        _mpuhandler(a)
        out.append((a, {'end': 'park', 'kind': '6'}, [f'+IRQ_PERIOD={per}', '+LAT=3', '+PARK_HOLD=400']))
    return out


# ---- precise EI (an EI enabling with an IRQ pending takes it right after itself)
ELOG = DATA + 0x700        # [0] = IRQ entries, [8 * k] = the k-th frame's PC


def _irq_logger(a):
    """an IRQ handler at 0x100000 (+NO_IRQ_HANDLER): counts entries at ELOG
    and logs each frame's PC at ELOG + 8 * count"""
    def h(s):
        s.push(1); s.push(2); s.push(3)
        s.setr(1, ELOG); s.ld(64, 2, 1); s.inc(2); s.st(64, 2, 1)
        s.shift_n('shl', 2, 2, 3); s.alu('add', 1, 1, 2)
        s.getsp(3); s.ld(64, 3, 3, 24)
        s.shift_n('shl', 3, 3, 32); s.shift_n('shr', 3, 3, 32); s.st(64, 3, 1)
        s.pop(3); s.pop(2); s.pop(1)
        s.iret()
    _place(a, 0x100000, h)


def _ei_prog(body, plus=(), per=400):
    a = Asm()
    a.setr(1, 0x08000000); a.setsp(1)
    a.di(9)
    a.set_int_mask(1)
    body(a)
    a.halt()
    _irq_logger(a)
    return a, ['+NO_IRQ_HANDLER', f'+IRQ_PERIOD={per}'] + list(plus)


@test
def ei_precise_take():
    """xv6's idle pattern: DI; WAIT (wakes on the pending IRQ, no handler);
    EI; DI.  The IRQ is taken between EI and DI: one entry, frame PC = the
    DI's address, and the DI runs after the IRET (its old-enable result 1)"""
    out = []
    for plus in (['+LAT=1'], ['+LAT=3'], ['+LAT=7', '+HIT=70']):
        def body(a):
            a.wait()
            a.setr(1, 1); a.ei(1)
            a.label('d'); a.di(2)
            for _ in range(8):
                a.nop()
        a, p = _ei_prog(body, plus)
        out.append((a, {'end': 'park', 'kind': '0', 'r2': 1, m(ELOG): 1, m(ELOG + 8): a.labels['d']}, p))
    return out


@test
def ei_enable_store():
    """HW_PERF_PLAN §9 item 4 (Fable review): the INT_MASK store that
    unmasks a pending timer IRQ 0, 1 or 2 instructions ahead of the EI
    (warm code: the sequence runs twice through one subroutine, pass 1
    storing 0).  The IRQ is taken right after the EI: one entry, frame PC =
    EI + 4 (it used to be latched as an ordinary op before the mask
    changed, and the DI behind it won)"""
    out = []
    for k in (0, 1, 2):
        for pad in range(4):
            for lat in (1, 3):
                a = Asm()
                a.setr(1, 0x08000000); a.setsp(1)
                a.di(9)
                a.set_int_mask(0)
                a.setr(9, 150)
                a.label('w'); a.dec(9); a.fbr('ne', 9, 0, 'w', imm=True)   # the timer pends
                a.setr(5, 0xF00F0000); a.setr(1, 1)
                a.setr(7, 0); a.call('seq')
                a.setr(7, 1); a.call('seq')
                a.halt()
                for _ in range(pad):
                    a.nop()
                a.label('seq')
                a.st(64, 7, 5)
                for _ in range(k):
                    a.nop()
                a.label('ei'); a.ei(1)
                a.di(2)
                a.ret()
                _irq_logger(a)
                out.append((a, {'end': 'park', 'kind': '0', 'r2': 1, m(ELOG): 1, m(ELOG + 8): a.labels['ei'] + 4},
                            ['+NO_IRQ_HANDLER', '+IRQ_PERIOD=400', f'+LAT={lat}']))
    return out


@test
def ei_nothing_pending():
    """EI; DI with no interrupt pending (masked): nothing taken, EI is not
    serialized (same cycles as before precise EI)"""
    a = Asm()
    a.setr(1, 1)
    for _ in range(4):
        a.ei(1); a.di(2)
    a.halt()
    return a, {'end': 'park', 'kind': '0', 'r2': 1}, ['+IRQ_PERIOD=40']


@test
def ei_zero_pending():
    """EI 0 with an IRQ pending: nothing taken (it serializes, harmlessly);
    a later EI 1 takes it at once"""
    def body(a):
        a.wait()
        a.setr(1, 0); a.ei(1)
        a.di(2)                                  # old enable 0
        a.setr(1, 1); a.ei(1)
        a.label('n'); a.nop()
        a.di(3)
    a, p = _ei_prog(body)
    return a, {'end': 'park', 'kind': '0', 'r2': 0, 'r3': 1, m(ELOG): 1, m(ELOG + 8): a.labels['n']}, p


@test
def ei_debugger_attached():
    """the same with a debugger attached (DEBUG_EN builds): the IRQ is still
    taken right after the EI (only single-steps are IRQ-free)"""
    def body(a):
        a.wait()
        a.setr(1, 1); a.ei(1)
        a.label('d'); a.di(2)
    a, p = _ei_prog(body, ['+DBG_ATTACHED'])
    # (attached, the closing HALT is a debug halt: park kind 4)
    kind = '4' if _os.environ.get('VSIM_DEBUG_EN') == '1' else '0'
    return a, {'end': 'park', 'kind': kind, 'r2': 1, m(ELOG): 1, m(ELOG + 8): a.labels['d']}, p


@test
def ei_in_exception_handler():
    """inside an exception handler (entry set DI): EI; DI takes the pending
    IRQ between them, nested; both handlers return"""
    a = Asm()
    a.setr(1, 0x08000000); a.setsp(1)
    a.leapc(1, 'xh'); a.mtsr('evec', 1)
    a.set_int_mask(1)
    a.di(9)
    a.wait()                                     # the IRQ is now pending
    a.syscall()
    a.label('back'); a.setr(5, 0x55)
    a.halt()
    a.label('xh')                                # exception handler: DI is set
    a.setr(1, 1); a.ei(1)
    a.label('d'); a.di(2)
    a.iret()
    _irq_logger(a)
    return a, {'end': 'park', 'kind': '0', 'r2': 1, 'r5': 0x55, m(ELOG): 1,
               m(ELOG + 8): a.labels['d']}, ['+NO_IRQ_HANDLER', '+IRQ_PERIOD=400']


@test
def iret_enable_pending():
    """an IRET whose frame enables interrupts while one is pending takes it
    before the target's first instruction (frame PC = the target)"""
    def body(a):
        a.wait()                                 # pending, DI
        a.leapc(2, 't'); a.setr64(3, 1 << 39); a.alu('or', 2, 2, 3)
        a.push(2); a.iret()                      # frame: PC = t, DI = 0, INT_MASK = 1
        a.label('t'); a.setr(6, 0x66)
    a, p = _ei_prog(body)
    return a, {'end': 'park', 'kind': '0', 'r6': 0x66, m(ELOG): 1, m(ELOG + 8): a.labels['t']}, p


@test
def ei_single_step():
    """DEBUG_EN: single-stepping over EI; DI with an IRQ pending takes no
    interrupt (steps never do); the IRQ is taken after a later EI run
    freely.  (Without DEBUG_EN the breakpoint is ignored: the first EI
    takes it.)"""
    dbg = _os.environ.get('VSIM_DEBUG_EN') == '1'
    def body(a):
        a.wait()
        a.setr(1, 1)
        a.label('e'); a.ei(1)
        a.label('d'); a.di(2)
        a.nop(); a.nop()
        a.ei(1)
        a.label('n'); a.nop()
        a.di(3)
    a, p = _ei_prog(body)
    p += ['+DBG_BP0=%x' % a.labels['e'], '+DBG_STEPS=2']
    want = a.labels['n'] if dbg else a.labels['d']
    return a, {'end': 'park', 'kind': '0', m(ELOG): 1, m(ELOG + 8): want}, p


# ============================================ HW_PERF_PLAN.md §1: DCBZ
# DCBZ [rs1] = 0x5F00_2000 | rs1 << 4 (kasm Asm.dcbz).  It zeroes the 32 B
# line holding rs1, as a store: every later load of the line returns 0.
# The TB's memory model zeroes the line on the request's ready edge (its
# hit-pipe model, +HIT, answers from memory: until then the core's
# same-line block / store forwarding must cover the line).
import os as _osz
NOEXC = _osz.environ.get('VSIM_EXC_EN') == '0'      # core 2's shape: DCBZ illegal
ZL = DATA + 0x40                                    # the line zeroed by most tests
DZ_V = [0x1111111111111111 * (i + 1) for i in range(4)]


def _dz_fill(a, base_reg=12):
    """the line at ZL = DZ_V, and sentinels in the dwords either side"""
    a.setr64(1, 0xA5A5A5A5A5A5A5A5); a.st(64, 1, base_reg, 0x38)
    a.setr64(1, 0x5A5A5A5A5A5A5A5A); a.st(64, 1, base_reg, 0x60)
    for i, v in enumerate(DZ_V):
        a.setr64(1, v); a.st(64, 1, base_reg, 0x40 + 8 * i)


def _dz_exp(extra=None):
    e = {'end': 'park', 'kind': '0', m(DATA + 0x38): 0xA5A5A5A5A5A5A5A5,
         m(DATA + 0x60): 0x5A5A5A5A5A5A5A5A}
    for i in range(4):
        e[m(ZL + 8 * i)] = 0
    e.update(extra or {})
    return e


_DZ_PLUS = [['+LAT=1'], ['+LAT=3', '+HIT=70'], ['+LAT=7', '+HIT=100'], ['+LAT=2', '+LATR=5', '+HIT=50'],
            ['+LAT=1', '+HIT=100']]


@test
def dcbz_load_after():
    """a load of each dword of the line 0..4 instructions after the DCBZ
    (the stores filling the line just before it): 0, every size; the
    sentinels either side untouched.  Covers STLF / the same-line block in
    every position (MEM, port, store queue) with and without hit-pipe hits."""
    out = []
    n = 0
    for k in range(4):
        for d in (0, 1, 2, 4):
            for off in (0, (13 + 8 * k) & 31):
                a = Asm()
                a.setr(12, DATA); a.setr(9, ZL + off)
                _dz_fill(a)
                a.dcbz(9)
                for _ in range(d):
                    a.nop()
                a.ld(64, 3, 12, 0x40 + 8 * k)
                a.ld(8, 4, 12, 0x40 + 8 * ((k + 1) & 3) + 5, sgn=True)
                a.ld(32, 5, 12, 0x40 + 8 * ((k + 2) & 3) + 4, sgn=True, mode='long')
                a.ld(16, 6, 12, 0x40 + 8 * ((k + 3) & 3) + 2, sgn=True)
                a.ld(64, 7, 12, 0x38)
                a.ld(64, 8, 12, 0x60)
                a.halt()
                exp = _dz_exp({'r3': 0, 'r4': 0, 'r5': 0, 'r6': 0, 'r7': 0xA5A5A5A5A5A5A5A5,
                               'r8': 0x5A5A5A5A5A5A5A5A, 'zero_lines': '1'})
                out.append((a, exp, _DZ_PLUS[n % len(_DZ_PLUS)]))
                n += 1
    return out


@test
def dcbz_sq():
    """a DCBZ queued behind a posted store (EN_SQ) to another line, then a
    load of the zeroed line while it waits there (sf_c)"""
    out = []
    n = 0
    for k in range(4):
        for d in (0, 1, 2):
            for lat in (5, 9):
                a = Asm()
                a.setr(12, DATA); a.setr(9, ZL + 3); a.setr(2, 0x99)
                _dz_fill(a)
                for _ in range(30):
                    a.nop()                        # everything drained
                a.st(64, 2, 12, 0x100)             # posted (another line)
                a.dcbz(9)                          # queued behind it
                for _ in range(d):
                    a.nop()
                a.ld(64, 3, 12, 0x40 + 8 * k)
                a.ld(64, 4, 12, 0x100)
                a.halt()
                exp = _dz_exp({'r3': 0, 'r4': 0x99, m(DATA + 0x100): 0x99})
                out.append((a, exp, [f'+LAT={lat}', '+HIT=100' if n % 2 else '+HIT=60']))
                n += 1
    return out


@test
def dcbz_store_load():
    """DCBZ, then a 64-bit store into dword k and a byte store into dword j,
    then loads: the stores' bytes, zeros elsewhere (younger stores win)"""
    out = []
    n = 0
    for k in range(4):
        for j in range(4):
            for d in (0, 2):
                a = Asm()
                a.setr(12, DATA); a.setr(9, ZL + 31)
                _dz_fill(a)
                a.setr64(2, 0xC0FFEE00C0FFEE00 + k); a.setr(10, 0x7E)
                a.dcbz(9)
                for _ in range(d):
                    a.nop()
                a.st(64, 2, 12, 0x40 + 8 * k)
                a.st(8, 10, 12, 0x40 + 8 * j + 3)
                for i in range(4):
                    a.ld(64, 3 + i, 12, 0x40 + 8 * i)
                a.halt()
                want = [0, 0, 0, 0]
                want[k] = 0xC0FFEE00C0FFEE00 + k
                want[j] = (want[j] & ~(0xFF << 24)) | (0x7E << 24)
                exp = _dz_exp({f'r{3 + i}': want[i] for i in range(4)})
                for i in range(4):
                    exp[m(ZL + 8 * i)] = want[i]
                out.append((a, exp, _DZ_PLUS[n % len(_DZ_PLUS)]))
                n += 1
    return out


@test
def dcbz_forms():
    """rs1 in any register, EA anywhere in the line (aligned down; the line
    after is untouched, also from the last byte), back-to-back DCBZs of the
    same and of adjacent lines, and a DCBZ chained behind posted stores"""
    out = []
    for rs1 in (0, 5, 13, 15):
        for off in (0, 1, 8, 31):
            a = Asm()
            a.setr(12, DATA)
            _dz_fill(a)
            a.setr(rs1, ZL + off)
            a.dcbz(rs1)
            a.halt()
            out.append((a, _dz_exp({f'r{rs1}': ZL + off}), ['+LAT=3']))
    a = Asm()
    a.setr(12, DATA); a.setr(9, ZL); a.setr(8, ZL + 0x20)
    _dz_fill(a)
    a.setr64(1, 0x77); a.st(64, 1, 12, 0x60); a.st(64, 1, 12, 0x78)
    a.dcbz(9); a.dcbz(9); a.dcbz(8); a.dcbz(9)
    a.ld(64, 3, 12, 0x60); a.ld(64, 4, 12, 0x78); a.ld(64, 5, 12, 0x58)
    a.halt()
    exp = _dz_exp({'r3': 0, 'r4': 0, 'r5': 0, m(DATA + 0x60): 0, m(DATA + 0x78): 0, 'zero_lines': '4'})
    out += [(a, exp, p) for p in _DZ_PLUS]
    return out


def _dz_code_ahead(k, pad, warm, lat, pre_ld=False):
    """DCBZ zeroes the code line holding X, k instructions ahead in straight
    code (pass 2 when warm: pass 1 runs X and the line is in the I-cache /
    NLB / IFB).  The first instruction at or after the line start executes
    the zeros: ILLEGAL (LEN 00) at max(DCBZ + 4, line base)."""
    a = Asm()
    a.setr(1, 0); a.setr(6, 2 if warm else 1); a.setr(5, 0)
    a.leapc(2, 'X')
    a.label('again')
    for _ in range(pad):
        a.nop()
    if warm:
        a.fbr('ne', 6, 1, 'skip', imm=True)
    if pre_ld:
        a.ld(64, 8, 2)          # a slow-path load on the port: the DCBZ chains
                                # behind it and completes unposted (smc_st)
    dz = a.dcbz(2)
    a.label('skip')
    for _ in range(k - 1):
        a.inc(5)
    a.label('X')
    a.alui_s('add', 1, 1, 1)
    a.addi_s(6, 6, -1)
    a.fbr('ne', 6, 0, 'again', imm=True)
    a.halt()
    x = a.labels['X']
    at = max(dz + 4, x & ~31)
    # instructions between the DCBZ and `at` ran (pass 2)
    n_inc = (at - (dz + 4)) // 4
    exp = {'end': 'park', 'kind': '2', 'pc': at, 'op': 0, 'r1': 1 if warm else 0,
           'r5': (k - 1 if warm else 0) + n_inc}
    return a, exp, [f'+LAT={lat}']


@test
def dcbz_code_ahead():
    out = []
    for k in range(1, 11):
        for pad in range(8):
            for warm in (False, True):
                for pre in (False, True):
                    out.append(_dz_code_ahead(k, pad, warm, 1 + (k + pad) % 7, pre))
    return out


@test
def dcbz_code_call():
    """a function (warm: called twice — I-cache, BTIC, RAS) in either 16 B
    half of a line, zeroed by a DCBZ through an address in the other half
    (or the same); the next call executes the zeros (ILLEGAL at its entry)"""
    out = []
    for half in (0, 16):
        for ea_off in (0, 5, 16, 27):
            for lat in (1, 4):
                a = Asm()
                a.setr(1, 0)
                a.leapc(2, 'F')
                a.setr(7, 0)
                a.call('F'); a.call('F')
                if half:
                    a.call('G')                  # the even half's I-cache line: another tag
                a.alui('and', 3, 2, ~31 & 0xFFFFFFFF, sgn=False)
                a.addi_s(3, 3, ea_off)
                a.dcbz(3)
                a.call('F')
                a.halt()
                a.nop()
                while (a.pc & 31) != 0:             # F's line holds no other code
                    a.nop()
                for _ in range(half // 4):
                    a.nop()
                a.label('F')
                a.inc(1)
                a.ret()
                f = a.labels['F']
                if half:
                    while a.pc != (f & ~31) + IC_BYTES:   # I-cache size on: same index
                        a.nop()
                    a.label('G')
                    a.inc(7)
                    a.ret()
                out.append((a, {'end': 'park', 'kind': '2', 'pc': f, 'op': 0, 'r1': 2,
                                'r7': 1 if half else 0}, [f'+LAT={lat}']))
    return out


@test
def dcbz_mmio():
    """DCBZ to MMIO raises ACCESS_STORE in supervisor AND user mode (EADDR =
    the raw EA, nothing reaches the bus); EVEC = 0: parks as ILLEGAL"""
    out = []
    if NOEXC:
        return out
    for ea in (0xF0010000, 0xF001000D, 0xFFFFFFFF):
        a = Asm()
        _xsetup(a)
        a.setr(9, ea); a.setr(3, 0x33)
        at = a.dcbz(9)
        a.setr(4, 0x44)
        a.halt()
        _xhandler(a)
        exp = {'end': 'park', 'kind': '0', 'r3': 0x33, 'r4': 0x44, 'mmio_wr': '0', 'zero_lines': '0'}
        exp.update(_xexp(X_ACC_ST, ea, at))
        out.append((a, exp, []))
    a = Asm(); a.setr(9, 0xF0010008); at = a.dcbz(9); a.halt()
    out.append((a, {'end': 'park', 'kind': '2', 'pc': at, 'mmio_wr': '0'}, []))

    a = Asm()                                       # user mode
    _xsetup(a)
    a.setr(12, DATA); _dz_fill(a)
    _to_user(a, 'u')
    a.label('u')
    a.setr(9, 0xF0010010); a.dcbz(9)                # ACCESS_STORE
    a.setr(9, ZL + 9); a.dcbz(9)                    # user DRAM: zeroes the line
    a.halt()
    _uhandler(a)
    out.append((a, _dz_exp({m(XLOG): X_ACC_ST, m(XLOG + 8): X_PRIV, m(XD + 32): 2, 'mmio_wr': '0'}), []))
    return out


@test
def dcbz_tlb():
    """TE = 1, user mode: DCBZ needs store permission — a read-only page is
    ACCESS_STORE, an unmapped one TLB_MISS_STORE (EADDR = the raw EA,
    TLB_HI loaded); a writable page zeroes the PA line, and loads right
    behind it (same VA, and the identity alias of the PA) read 0"""
    if NOEXC:
        return []
    def setup(a):
        _tlb_set(a, 2, 0x0100, 0x001, TLB_V | TLB_R | TLB_W)        # 0x0100_0000 -> PA 0x10000
        _tlb_set(a, 3, 0x0300, 0x001, TLB_V | TLB_R)                # read-only alias
        a.setr(12, DATA); _dz_fill(a)
    def body(a):
        a.setr(9, 0x03000045); a.dcbz(9)                            # R only: ACC_ST
        a.setr(9, 0x02000013); a.dcbz(9)                            # unmapped: TLB_MISS_ST
        a.ld(64, 4, 12, ZL - GOOD)                                  # still DZ_V[0]
        a.setr(9, 0x01000047); a.setr(10, 0x01000058)
        a.dcbz(9)
        a.ld(64, 5, 10)                                             # same VA page: 0
        a.ld(64, 6, 12, ZL + 8 - GOOD)                              # the identity alias: 0
        a.ld(64, 7, 9, 0x60 - 0x47, mode='long')                   # the next line: sentinel
    logs = [(X_ACC_ST, 0x03000045), (X_TLB_ST, 0x02000013)]
    return [_te_prog(body, True, _dz_exp({'r4': DZ_V[0], 'r5': 0, 'r6': 0, 'r7': 0x5A5A5A5A5A5A5A5A}),
                     logs=logs, setup=setup, plus=p) for p in (['+LAT=3'], ['+LAT=1', '+HIT=100'])]


@test
def dcbz_illegal():
    """only 0x5F00_2000 | rs1 << 4 is DCBZ: its [13] = 1 neighbours stay
    ILLEGAL ([12] = 1 too, another SIZE, SGN, A, MODE 10 / 11, a nonzero
    [11:8] / [3:0] / [19:14], LEN 10, class 6).  (MODE 01 with [13] = 1 is
    the existing short store, imm8 = [19:12].)  Core 2's shape (EXC_EN = 0,
    no D-cache): DCBZ itself is ILLEGAL."""
    out = []
    words = [0x5F003010, 0x5E002010, 0x5D002010, 0x5C002010, 0x5F802010, 0x5F102010,
             0x5F402010, 0x5F602010, 0x5F002110, 0x5F002011, 0x5F006010, 0x5F0A2010,
             0x5F082010, 0x5B002010]
    if NOEXC:
        words.append(0x5F002010)
    for w in words:
        a = Asm(); a.setr(1, DATA); at = a.raw(w); a.halt()
        out.append((a, {'end': 'park', 'kind': '2', 'pc': at, 'op': w, 'bus_wr': '0'}, []))
    a = Asm(); a.setr(1, DATA); at = a.raw(0x9F002010, 0); a.halt()
    out.append((a, {'end': 'park', 'kind': '2', 'pc': at, 'bus_wr': '0'}, []))
    return out


# ============================================ HW_PERF_PLAN.md §4: STATUS.MAL64
MLOG = XD + 0x100           # _lenhandler: 16 bytes per entry (ECAUSE, EADDR)


def _lenhandler(a, name='h'):
    """logs ECAUSE / EADDR at MLOG + 16*(n-1) (count at XD+32) and steps the
    frame PC over the faulting instruction, whatever its length"""
    a.label(name)
    a.push(13); a.push(14)
    a.setr(14, XD)
    a.ld(64, 13, 14, 32); a.inc(13); a.st(64, 13, 14, 32)
    a.shift_n('shl', 13, 13, 4); a.alui('add', 13, 13, MLOG - 16)
    a.mfsr(14, 'ecause'); a.st(64, 14, 13, 0)
    a.mfsr(14, 'eaddr');  a.st(64, 14, 13, 8)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, name + '_nh'); a.halt()                # a user HALT halts
    a.label(name + '_nh')
    a.getsp(13); a.ld(64, 14, 13, 16)                           # the frame word
    a.ld(32, 13, 14)                                            # the op at its PC (MEMGET32)
    a.shift_n('shr', 13, 13, 30); a.shift_n('shl', 13, 13, 2)   # 4 * LEN
    a.alu('add', 14, 14, 13)
    a.getsp(13); a.st(64, 14, 13, 16)
    a.pop(14); a.pop(13)
    a.iret()


def _a1(a, at):
    """set A ([20]) on the instruction just emitted at `at` (64-bit forms)"""
    i = (at - a.base) // 4
    a.words[i] |= 1 << 20


MB = DATA + 0x80            # the MAL64 tests' data (MB .. MB+0x3F)
MV = [0x0102030405060708 * (i + 1) for i in range(8)]


def _mal_ops(a, mis):
    """every 64-bit LOAD / STORE addressing mode, A = 0 and A = 1, at an EA
    with EA[2:0] = mis (0: aligned).  r12 = MB, r9 = MB + mis, r11 = mis.
    Loads write r3 (preset 0x33); stores write r4 (0x44).  Returns the list
    of (pc, is_store, raw EA)."""
    ops = []
    for A in (0, 1):
        def lds():
            yield lambda: a.ld(64, 3, 9), MB + mis                         # [rs1]
            yield lambda: a.ld(64, 3, 9, 8), MB + 8 + mis                  # short simm8
            yield lambda: a.ld(64, 3, 12, 16 + mis, mode='long'), MB + 16 + mis
            yield lambda: a.ld_abs(64, 3, MB + 24 + mis), MB + 24 + mis
            yield lambda: a.ld_rr(64, 3, 12, 11), MB + mis
        def sts():
            yield lambda: a.st(64, 4, 9), MB + mis
            yield lambda: a.st(64, 4, 9, 8), MB + 8 + mis
            yield lambda: a.st(64, 4, 12, 16 + mis, mode='long'), MB + 16 + mis
            yield lambda: a.st_abs(64, 4, MB + 24 + mis), MB + 24 + mis
            yield lambda: a.st_rr(64, 4, 12, 11), MB + mis
        for st, gen in ((False, lds), (True, sts)):
            for fn, ea in gen():
                at = fn()
                if A:
                    _a1(a, at)
                ops.append((at, st, ea))
    return ops


def _mal_prog(mis, mal, user=False):
    a = Asm()
    _xsetup(a)
    a.setr(12, MB); a.setr(9, MB + mis); a.setr(11, mis)
    for i, v in enumerate(MV[:5]):
        a.setr64(1, v); a.st(64, 1, 12, 8 * i)
    a.mfsr(5, 'status')                                    # 0 after reset
    a.setr(1, 0x400 if mal else 0); a.mtsr('status', 1)
    a.mfsr(6, 'status')
    if user:
        _to_user(a, 'u')
    a.label('u')
    a.setr(3, 0x33); a.setr(4, 0x44)
    a.mfsr(7, 'status')                                    # readable in user mode
    ops = _mal_ops(a, mis)
    a.halt()
    _lenhandler(a)
    return a, ops


@test
def mal64_traps():
    """STATUS[10] MAL64 = 1: every misaligned 64-bit LOAD / STORE traps with
    cause 5 / 6, EADDR = the raw EA, nothing loaded or stored; aligned ones
    run.  MAL64 = 0: a misaligned 64-bit access uses the doubleword aligned
    down (unchanged).  Also in user mode."""
    if NOEXC:
        return []
    out = []
    for mis in (0, 1, 3, 4, 7):
        for mal in (True, False):
            for user in ((False, True) if (mal and mis == 3) else (False,)):
                a, ops = _mal_prog(mis, mal, user)
                trap = mal and mis != 0
                exp = {'end': 'park', 'kind': '0', 'r5': 0, 'r6': 0x400 if mal else 0,
                       'r7': (0x400 if mal else 0) | (1 if user else 0)}
                if trap:
                    exp['r3'] = 0x33
                    for i, v in enumerate(MV[:5]):
                        exp[m(MB + 8 * i)] = v                     # untouched
                    for n, (at, st, ea) in enumerate(ops):
                        exp[m(MLOG + 16 * n)] = X_MISAL_ST if st else X_MISAL_LD
                        exp[m(MLOG + 16 * n + 8)] = ea
                    exp[m(XD + 32)] = len(ops) + (1 if user else 0)
                else:
                    # the aligned-down dword (the last load, A = 1 [r12 + r11],
                    # reads MB, which the A = 0 stores wrote)
                    exp['r3'] = 0x44
                    for i in range(5):
                        exp[m(MB + 8 * i)] = 0x44 if i < 4 else MV[4]
                    exp[m(XD + 32)] = 1 if user else 0
                if user:
                    exp[m(MLOG + 16 * len(ops) * trap)] = X_PRIV
                for p in (['+LAT=3'], ['+LAT=1', '+HIT=100']):
                    out.append((a, exp, p))
    return out


@test
def mal64_exempt():
    """with MAL64 = 1 the stack ops (misaligned SP), MEMGET32, DCBZ and 8 /
    16 / 32-bit accesses keep their behaviour; LR.D still traps; EVEC = 0
    parks a misaligned 64-bit load with kind 5; MTSR STATUS leaves MAL64
    clear when [10] = 0"""
    if NOEXC:
        return []
    out = []
    a = Asm()
    _xsetup(a)
    a.setr(12, MB)
    for i, v in enumerate(MV[:5]):
        a.setr64(1, v); a.st(64, 1, 12, 8 * i)
    a.setr(1, 0x400); a.mtsr('status', 1)
    a.getsp(8); a.setr(1, MB + 0x3C); a.setsp(1)
    a.setr64(2, 0xDEAD); a.push(2); a.pop(3)               # [MB+0x34] (aligned down by the port)
    a.setsp(8)
    a.setr(9, MB + 5); a.ld(32, 4, 9)                      # MEMGET32, spanning
    a.ld(8, 5, 12, 0x13)                                   # byte
    a.setr(9, MB + 0x2B); a.dcbz(9)                        # DCBZ at a misaligned EA: zeroes MB+0x20..0x3F
    a.setr(9, MB + 4); a.lr(64, 6, 9)                      # LR.D: MISALIGNED_LOAD (as always)
    a.halt()
    _lenhandler(a)
    exp = {'end': 'park', 'kind': '0', 'r3': 0xDEAD,
           'r4': ((MV[0] >> 40) | (MV[1] << 24)) & 0xFFFFFFFF, 'r5': (MV[2] >> 24) & 0xFF,
           m(MLOG): X_MISAL_LD, m(MLOG + 8): MB + 4, m(XD + 32): 1,
           m(MB + 0x20): 0, m(MB + 0x28): 0, m(MB + 0x30): 0, m(MB + 0x38): 0, 'zero_lines': '1'}
    out.append((a, exp, ['+LAT=3']))
    a = Asm()
    a.setr(1, 0x400); a.mtsr('status', 1)
    a.setr(9, MB + 2); at = a.ld(64, 3, 9)
    a.halt()
    out.append((a, {'end': 'park', 'kind': '5', 'pc': at}, []))
    a = Asm()
    a.setr(12, MB); a.setr64(1, MV[0]); a.st(64, 1, 12)
    a.setr(1, 0x400); a.mtsr('status', 1)
    a.setr(1, 0); a.mtsr('status', 1)                      # MAL64 back to 0
    a.mfsr(6, 'status')
    a.setr(9, MB + 2); a.ld(64, 3, 9)                      # aligned down again
    a.halt()
    out.append((a, {'end': 'park', 'kind': '0', 'r6': 0, 'r3': MV[0]}, []))
    return out


# ------------------------------------- HW_PERF_PLAN §2: 16 KiB I-cache
# (IC_BYTES = 16 KiB: index VA[13:4], tag PA[31:14].  Under the old 8 KiB
# geometry an address and the one 0x2000 above it shared an index.)
IC_MISS_LOOP12 = 385 if IC32 else 770    # the first pass: 384 / 768 lines + the setup (/ exit) line(s)


def _ic_loop(nbytes, passes):
    """a loop whose body is nbytes of straight code (16 B aligned)"""
    a = Asm()
    a.setr(1, passes); a.setr(2, 0)
    while a.pc & 15:
        a.nop()
    a.label('l')
    for _ in range(nbytes // 4 - 3):
        a.nop()
    a.inc(2)
    a.addi_s(1, 1, -1)
    a.fbr('ne', 1, 0, 'l', imm=True)
    a.halt()
    return a


@test
def ic16_loop():
    """a 12 KiB loop (> 8 KiB, < 16 KiB): only the first pass misses — the
    I-cache miss count after 1, 2 and 4 passes is the same (8 KiB: every
    pass refetched the whole loop)"""
    out = []
    for passes in (1, 2, 4):
        for lat in (1, 5):
            a = _ic_loop(12 * 1024, passes)
            out.append((a, {'end': 'park', 'kind': '0', 'r2': passes, 'r1': 0,
                            'ic_miss': str(IC_MISS_LOOP12)}, [f'+LAT={lat}']))
    return out


def _ic_pad(a, addr):
    assert a.pc <= addr, (hex(a.pc), hex(addr))
    while a.pc != addr:
        a.nop()


def _ic_funcs(a, funcs):
    """emit (label, addr, emit_body) functions in address order"""
    for lb, at, fn in sorted(funcs, key=lambda f: f[1]):
        _ic_pad(a, at)
        a.label(lb)
        fn()


@test
def ic16_smc_alias():
    """functions X at 0x1000 and Y at 0x3000 (old geometry: one index; now
    two), both warm; a store patches X, then Y, through a 64 / 32 / 8-bit
    store; each call runs the current code.  A data store to X + 0x4000
    (the same index now, another tag) leaves X cached and correct."""
    new1 = _word(lambda t: t.alui_s('add', 1, 1, 16))
    new2 = _word(lambda t: t.alui_s('add', 2, 2, 16))
    out = []
    for size in (64, 32, 8):
        for lat in (1, 4):
            a = Asm()
            a.setr(1, 0); a.setr(2, 0)
            a.leapc(8, 'X'); a.leapc(9, 'Y')
            a.setr(4, 0x5000); a.setr(5, 0x5A)
            for _ in range(2):
                a.call('X'); a.call('Y')
            a.st(64, 5, 4, 0x0)                     # X + 0x4000: data only
            a.call('X'); a.call('Y')                # r1 = 3, r2 = 3

            def patch(reg, new, addr):
                if size == 64:
                    a.ld(64, 6, addr)
                    a.setr64(7, 0xFFFFFFFF00000000)
                    a.alu('and', 6, 6, 7)
                    a.setr(7, new)
                    a.alu('or', 6, 6, 7)
                    a.st(64, 6, addr)
                elif size == 32:
                    a.setr(7, new); a.st(32, 7, addr)
                else:                               # byte 2 only: imm8 1 -> 0x11 (as _smc)
                    a.setr(7, (new >> 16) & 0xFF); a.st(8, 7, addr, 2)
            inc = 0x11 if size == 8 else 16
            patch(1, new1, 8)
            a.call('X'); a.call('Y')                # r1 = 3 + inc, r2 = 4
            patch(2, new2, 9)
            a.call('Y'); a.call('X')                # r2 = 4 + inc, r1 = 3 + 2 inc
            a.ld(64, 10, 4)
            a.halt()
            _ic_pad(a, 0x1000)
            a.label('X'); a.alui_s('add', 1, 1, 1); a.ret()
            _ic_pad(a, 0x3000)
            a.label('Y'); a.alui_s('add', 2, 2, 1); a.ret()
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 3 + 2 * inc, 'r2': 4 + inc, 'r10': 0x5A},
                        [f'+LAT={lat}']))
    # warm X / Y calls alternate with no misses after the cold ones (8 KiB:
    # each call evicted the other)
    a = Asm()
    a.setr(1, 0); a.setr(2, 0); a.setr(3, 20)
    a.label('l')
    a.call('X'); a.call('Y')
    a.addi_s(3, 3, -1)
    a.fbr('ne', 3, 0, 'l', imm=True)
    a.halt()
    _ic_pad(a, 0x1000)
    a.label('X'); a.alui_s('add', 1, 1, 1); a.ret()
    _ic_pad(a, 0x3000)
    a.label('Y'); a.alui_s('add', 2, 2, 1); a.ret()
    out.append((a, {'end': 'park', 'kind': '0', 'r1': 20, 'r2': 20,
                    'ic_miss': '4' if IC32 else '5'}, []))   # (8 KiB: 43)
    return out


@test
def ic16_dcbz_pair():
    """F (even 16 B half) and G (odd half) share a 32 B line at L, both warm;
    so do H / K at L ^ 0x2000 (the old geometry's alias).  A DCBZ of L (EA
    in either half) kills both halves: H / K still run, then G executes the
    zeros (ILLEGAL at G).  A 64-bit store into G's half only: F runs its
    old code, G the patched one."""
    out = []
    for base in (0x1000, 0x3000):
        for ea_off in (0, 9, 16, 31):
            for lat in (1, 4):
                a = Asm()
                a.setr(1, 0); a.setr(7, 0)
                for _ in range(2):
                    a.call('F'); a.call('G'); a.call('H'); a.call('K')
                a.setr(3, base + ea_off)
                a.dcbz(3)
                a.call('H'); a.call('K')
                a.call('G')
                a.halt()
                f1 = lambda: (a.inc(1), a.ret())
                f7 = lambda: (a.inc(7), a.ret())
                _ic_funcs(a, [('F', base, f1), ('G', base + 16, f1),
                              ('H', base ^ 0x2000, f7), ('K', (base ^ 0x2000) + 16, f7)])
                out.append((a, {'end': 'park', 'kind': '2', 'pc': base + 16, 'op': 0,
                                'r1': 4, 'r7': 6}, [f'+LAT={lat}']))
    newg = _word(lambda t: t.alui_s('add', 1, 1, 16))
    for base in (0x1000, 0x3000):
        a = Asm()
        a.setr(1, 0); a.setr(7, 0)
        for _ in range(2):
            a.call('F'); a.call('G'); a.call('H')
        a.setr(3, base + 16)
        a.setr64(4, (_word(lambda t: t.ret()) << 32) | newg)   # G: addi r1,16 ; ret
        a.st(64, 4, 3)
        a.call('F'); a.call('G'); a.call('H')       # r1 = 4 + 1 + 16
        a.halt()
        f1 = lambda: (a.inc(1), a.ret())
        _ic_funcs(a, [('F', base, f1), ('G', base + 16, f1),
                      ('H', base ^ 0x2000, lambda: (a.inc(7), a.ret()))])
        out.append((a, {'end': 'park', 'kind': '0', 'r1': 21, 'r7': 3}, ['+LAT=3']))
    return out


@test
def ic16_icinv():
    """an 8 KiB function in the I-cache's upper half (0x2000-0x3FFF: index
    bit 9 set, lines the 8 KiB cache did not have), called twice: warm, the
    second call does not miss; with ICACHE_INV between the calls it misses
    again on every line; code patched in DRAM then ICACHE_INV runs anew"""
    out = []
    for inv in (False, True):
        a = Asm()
        a.setr(2, 0)
        a.call('B')
        if inv:
            a.setr(1, 0xF0050000); a.setr(3, 8); a.st(32, 3, 1)      # ICACHE_INV
        a.call('B')
        a.halt()
        _ic_pad(a, 0x2000)
        a.label('B')
        for _ in range(0x2000 // 4 - 2):
            a.nop()
        a.inc(2); a.ret()
        out.append((a, {'end': 'park', 'kind': '0', 'r2': 2,
                        'ic_miss': str(IC_MISS_INV if inv else IC_MISS_WARM)}, []))
    new = _word(lambda t: t.alui_s('add', 2, 2, 16))
    for lat in (1, 4):
        a = Asm()
        a.setr(2, 0)
        a.call('P'); a.call('P')
        a.setr(9, 0x3FF0); a.setr(4, new); a.st(32, 4, 9)        # P's first word
        a.setr(1, 0xF0050000); a.setr(3, 8); a.st(32, 3, 1)      # ICACHE_INV
        a.call('P')
        a.halt()
        _ic_pad(a, 0x3FF0)
        a.label('P'); a.inc(2); a.ret()
        out.append((a, {'end': 'park', 'kind': '0', 'r2': 18}, [f'+LAT={lat}']))
    return out


IC_MISS_WARM = 257 if IC32 else 514     # 256 / 512 lines of B + 1 / 2 of main
# every line twice; (32 B lines) + the lines fetched during the 64-cycle
# flash-clear sweep after ICACHE_INV, which miss once per 16 B half
IC_MISS_INV = 518 if IC32 else 1028


@test
def ic16_te_pages():
    """TE = 1, user mode: snippets at VAs with the same low 14 bits (one
    I-cache index) in different pages / frames, and at VA[15:14] = 01 in the
    same page (the tag's VA bits); one at an index differing only in bit 13;
    a synonym (another VA page, the same frame).  Called round-robin three
    times: each runs its own code"""
    if NOEXC:
        return []
    def setup(a):
        _tlb_set(a, 2, 0x0050, 0x002, TLB_V | TLB_X)                 # 0x0050xxxx -> PA 0x2xxxx
        _tlb_set(a, 3, 0x0060, 0x003, TLB_V | TLB_X)                 # 0x0060xxxx -> PA 0x3xxxx
        _tlb_set(a, 4, 0x0070, 0x002, TLB_V | TLB_X)                 # synonym of 0x0050
    snips = [(0x00501230, 0x21230, 0x1), (0x00601230, 0x31230, 0x10),
             (0x00505230, 0x25230, 0x100), (0x00503230, 0x23230, 0x1000),
             (0x00701230, None, 0x1)]                                 # = the first one's code
    def body(a):
        a.setr(3, 0); a.setr(11, 3)
        a.label('it')
        for k, (va, _, _) in enumerate(snips):
            a.setr(9, va); a.leapc(10, f'r{k}'); a.jmpr(9)
            a.label(f'r{k}')
        a.addi_s(11, 11, -1)
        a.fbr('ne', 11, 0, 'it', imm=True)
    a, e, p = _te_prog(body, True, {'r3': 3 * sum(v for _, _, v in snips)}, setup=setup)
    for _, pa, val in snips:
        if pa is not None:
            _place(a, pa, lambda s, v=val: (s.setr(5, v), s.alu('add', 3, 3, 5), s.jmpr(10)))
    return a, e, p


@test
def ic16_smc_fill():
    """cold SMC sweep: the store patches an instruction whose line is being
    fetched (or was just installed) while the store waits in MEM — the
    case the tag-shadow guard (pipeline_core ri_v) covers"""
    out = []
    for dist in (0, 1, 2, 3, 5):
        for pad in range(8):
            for lat in (1, 2, 3, 5, 7):
                out.append(_smc(dist, 32, False, lat, pad_align=pad))
    return out


# ================================ HW_PERF_PLAN §5: 32 B I-cache lines
# The default geometry is 2048 x 32 B = 64 KiB (VSIM_IC=1024x16 runs the
# same programs on the 16 KiB / 16 B I-cache, with its own counts).  LDLYS:
# how long after the fill's ready the tb answers with the line (+LDLY): 0 =
# a D-cache hit, else a miss whose ready was the early restart.
LDLYS = (0, 3, 8)
POKE_A, POKE_D = 0xF0FE0000, 0xF0FE0008      # tb_vsim's non-coherent writer


def _ic_lines(lo, hi):
    """I-cache lines spanned by the bytes [lo, hi)"""
    return (hi - 1) // IC_LB - lo // IC_LB + 1


def _ic_dm_misses(seq):
    """direct-mapped misses of a sequence of byte addresses (one per line
    visit), with this geometry"""
    tags = {}
    n = 0
    for addr in seq:
        ln = addr // IC_LB
        if tags.get(ln % IC_LINES) != ln:
            tags[ln % IC_LINES] = ln
            n += 1
    return n


@test
def ic32_fill_dw():
    """a cold line entered at each of its four dwords (a CALL to L + 8k), run
    to its end, then the whole line from its start: one miss fills the 32 B
    line, so the second call does not miss (16 B lines: the half before the
    entry point misses then).  The line answer comes with the ready or
    LDLY cycles after it (the early restart of a D-cache miss)."""
    out = []
    L = 0x1000
    for k in range(4):
        for lat in (1, 4):
            for ld in LDLYS:
                a = Asm()
                a.setr(1, 0)
                a.call(f'T{k}')
                a.call('T0')
                a.halt()                       # main: 5 words in [0x20, 0x34)
                _ic_pad(a, L)
                for w in range(8):
                    if w % 2 == 0:
                        a.label(f'T{w // 2}')
                    a.inc(1)
                a.ret()                        # the next line
                # misses: main, the line(s) of L, the RET's line.  (16 B
                # lines, per-dword valid: an odd entry installs only its own
                # dword of the half, so the second call misses on the rest)
                miss = _ic_lines(0x20, 0x34) + (1 if IC32 else (2, 4, 2, 3)[k]) + 1
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 16 - 2 * k,
                                'ic_miss': str(miss)}, [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


def _ic_straight(n, reg):
    return lambda s: ([s.inc(reg) for _ in range(n)], s.jmpr(10))


def _ic_incs(s, n):
    """n increments alternating r3 / r12 (no back-to-back dependency)"""
    for i in range(n):
        s.inc(3 if i % 2 == 0 else 12)


@test
def ic32_prefetch():
    """straight 1-word code through 32 B line boundaries and a 64 KiB page
    boundary, three passes: only the first pass misses, and a warm pass runs
    at about one instruction per cycle (the prefetch stays a line ahead
    across lines and pages).  TE = 1: the two VA pages map to frames that are
    not adjacent; the frame after the first holds other code (r4) — fetching
    it would be a wrong translation."""
    out = []
    # words before / after the page boundary: the code after it stays in the
    # boundary's own (16 or 32 B) line — index 0, which main (0x20 on) does
    # not use, so the passes cannot evict each other
    n0, n1 = 64, 2
    for te in (False, True):
        if te and NOEXC:
            continue
        for lat in (1, 3, 6):
            for ld in (0, 5):
                va = (0x00510000 if te else 0x00030000) - 4 * n0
                fr = {va >> 16: 0x2, (va >> 16) + 1: 0x4} if te else None
                pa = (lambda v: (fr[v >> 16] << 16) | (v & 0xFFFF)) if te else (lambda v: v)

                def body(a):
                    a.setr(3, 0); a.setr(12, 0); a.setr(4, 0); a.setr(11, 3)
                    a.label('it')
                    a.mfsr(6, 'cycle')
                    a.setr(9, va); a.leapc(10, 'back'); a.jmpr(9)
                    a.label('back')
                    a.mfsr(7, 'cycle')
                    a.addi_s(11, 11, -1)
                    a.fbr('ne', 11, 0, 'it', imm=True)
                    a.alu('sub', 8, 7, 6)          # the last (warm) pass's cycles
                exp = {'r3': 3 * (n0 + n1) // 2, 'r12': 3 * (n0 + n1) // 2, 'r4': 0,
                       # n0 + n1 incs, the jumps there and back (an indirect
                       # JMPR each way) and the MFSRs: ~1 cycle per inc warm
                       'r8': (n0 + n1 + 8, n0 + n1 + 30)}
                if te:
                    a, e, p = _te_prog(body, False, exp, setup=lambda a: (
                        _tlb_set(a, 2, va >> 16, 0x2, TLB_V | TLB_X),
                        _tlb_set(a, 3, (va >> 16) + 1, 0x4, TLB_V | TLB_X)))
                    # the frame after 0x2 (a wrong translation would run it)
                    _place(a, 0x30000, _ic_straight(n1, 4))
                else:
                    a = Asm()
                    body(a)
                    a.halt()
                    e, p = dict(exp, end='park', kind='0'), []
                _wk_snip(a, va, pa, lambda s: (_ic_incs(s, n0 + n1), s.jmpr(10)))
                # first pass only: the code's lines (its VA page offsets
                # index the I-cache; both frames' lines are distinct)
                if te:
                    # (perf_ic[0] also counts each micro-ITLB refill cycle
                    # with TE = 1 — as on master — so count the installs:
                    # the code's lines plus the program's own, first pass only)
                    if IC32:
                        e['ic_inst'] = (_ic_lines(va, va + 4 * (n0 + n1 + 1)),
                                        _ic_lines(va, va + 4 * (n0 + n1 + 1)) + 12)
                else:
                    main_end = a.labels['back'] + 16
                    e['ic_miss'] = str(_ic_lines(0x20, main_end + 8) +
                                       _ic_lines(va, va + 4 * (n0 + n1 + 1)))
                out.append((a, e, p + [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_snoop_half():
    """a warm function F filling one 32 B line (7 x addi r1,1; ret) patched
    at word j — either 16 B half — by a 64 / 32 / 8-bit store: the next call
    runs the new word.  Then a data store into the second half of a line
    whose first half is code G: the snoop drops the whole 32 B line (G misses
    once more; on 16 B lines it does not) and G runs correctly."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    F = 0x1000
    for size in (64, 32, 8):
        for j in (0, 3, 4, 6):
            for lat, ld in ((1, 0), (4, 5), (2, 9)):
                a = Asm()
                a.setr(1, 0)
                a.call('F'); a.call('F')
                a.setr(2, F + 4 * j)
                if size == 64:
                    a.setr(2, F + 8 * (j // 2))
                    a.ld(64, 4, 2)
                    keep = 0xFFFFFFFF00000000 if j % 2 == 0 else 0x00000000FFFFFFFF
                    a.setr64(8, keep)
                    a.alu('and', 4, 4, 8)
                    a.setr64(8, new << (32 * (j % 2)))
                    a.alu('or', 4, 4, 8)
                    a.st(64, 4, 2)
                elif size == 32:
                    a.setr(3, new); a.st(32, 3, 2)
                else:                              # imm8 is byte 2: 1 -> 0x11
                    a.setr(3, (new >> 16) & 0xFF); a.st(8, 3, 2, 2)
                a.call('F')
                a.halt()
                _ic_pad(a, F)
                a.label('F')
                for _ in range(7):
                    a.alui_s('add', 1, 1, 1)
                a.ret()
                inc = 0x11 if size == 8 else 16
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 14 + 6 + inc},
                            [f'+LAT={lat}', f'+LDLY={ld}']))
    G = 0x2000
    for lat, ld in ((1, 0), (3, 6)):
        a = Asm()
        a.setr(7, 0); a.setr(9, G + 0x18); a.setr(5, 0x5A)
        a.call('G'); a.call('G')
        a.st(64, 5, 9)                             # data in G's line, other half
        a.call('G')
        a.ld(64, 6, 9)
        a.halt()
        main_end = a.pc
        _ic_pad(a, G)
        a.label('G')
        a.inc(7); a.inc(7); a.inc(7); a.ret()
        # (16 B lines: G's half and the run-ahead fill of the next half,
        # which the data store's snoop drops instead of G)
        miss = _ic_lines(0x20, main_end) + 2
        out.append((a, {'end': 'park', 'kind': '0', 'r7': 9, 'r6': 0x5A, 'ic_miss': str(miss)},
                    [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_dcbz():
    """DCBZ over cached code: a warm function F in either 16 B half of its
    line, the line zeroed through any address in it; the next call executes
    the zeros (ILLEGAL at F).  Line answers with and without delay."""
    out = []
    if NOEXC:
        return out
    for half in (0, 16):
        for ea_off in range(0, 32, 4):
            ea_off |= (ea_off >> 2) & 3          # some unaligned EAs too
            for lat, ld in ((1, 0), (4, 6)):
                a = Asm()
                a.setr(1, 0)
                a.leapc(2, 'F')
                a.call('F'); a.call('F')
                a.alui('and', 3, 2, ~31 & 0xFFFFFFFF, sgn=False)
                a.addi_s(3, 3, ea_off)
                a.dcbz(3)
                a.call('F')
                a.halt()
                a.nop()
                while (a.pc & 31) != 0:
                    a.nop()
                for _ in range(half // 4):
                    a.nop()
                a.label('F')
                a.inc(1)
                a.ret()
                out.append((a, {'end': 'park', 'kind': '2', 'pc': a.labels['F'], 'op': 0, 'r1': 2},
                            [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_icinv():
    """ICACHE_INV against a non-coherent writer (tb_vsim's poke device, which
    the store snoop does not see): a warm function P's first word is
    rewritten in DRAM; the next call still runs the cached old code, the one
    after ICACHE_INV the new.  Then the fence executed while another line's
    install is pending (its line answer LDLY cycles late, as read before
    the fence): that install must not survive it."""
    out = []
    new = _word(lambda t: t.alui_s('add', 2, 2, 16))
    P = 0x3000
    for lat, ld in ((1, 0), (3, 4), (5, 9)):
        a = Asm()
        a.setr(2, 0)
        a.call('P'); a.call('P')
        a.setr(8, POKE_A); a.setr(9, P); a.st(64, 9, 8)
        a.setr(9, P + 8); a.ld(64, 4, 9)           # P's second dword ...
        a.setr(9, P); a.ld(64, 3, 9)               # ... and its first
        a.setr64(7, 0xFFFFFFFF00000000); a.alu('and', 3, 3, 7)
        a.setr(7, new); a.alu('or', 3, 3, 7)
        a.setr(8, POKE_D); a.st(64, 3, 8)          # P's first word, behind the snoop
        a.call('P')                                # old code: +1
        a.setr(1, 0xF0050000); a.setr(3, 8); a.st(32, 3, 1)    # ICACHE_INV
        a.call('P')                                # new code: +16
        a.halt()
        _ic_pad(a, P)
        a.label('P'); a.inc(2); a.ret()
        out.append((a, {'end': 'park', 'kind': '0', 'r2': 19}, [f'+LAT={lat}', f'+LDLY={ld}']))
    # the fence meets a pending install: a cold line A is entered at its
    # last dword (a JMP to the warm R); R rewrites A's first dword behind
    # the snoop while A's line answer is still to come, fences, waits out
    # the sweep, then calls A + 0: it must run the new code (the install
    # read before the fence would bring back the old)
    A = 0x5000
    newdw = _word(lambda t: t.alui_s('add', 2, 2, 16)) | (_word(lambda t: t.ret()) << 32)
    # (LDLY 80 / 100: A's line arrives after the sweep — the case only
    # dropping the pending install at the fence covers; with an earlier
    # answer the re-fetch's install after the fence replaces A's deferred
    # valid set anyway)
    for lat, ld in ((1, 0), (3, 20), (2, 30), (1, 80), (3, 100)):
        a = Asm()
        a.setr(2, 0); a.setr(1, 0xF0050000); a.setr(3, 8); a.setr(5, 0)
        a.setr(8, POKE_A); a.setr(7, POKE_D); a.setr(9, A); a.setr64(6, newdw)
        a.call('R')                    # warm R (r5 = 0: it only returns)
        a.setr(5, 1)
        a.call('A3')
        a.halt()
        a.label('R')
        a.fbr('eq', 5, 0, 'rr', imm=True)
        a.st(64, 9, 8); a.st(64, 6, 7)             # poke A's first dword
        a.st(32, 3, 1)                             # ICACHE_INV
        a.setr(4, 60)
        a.label('rd')
        a.addi_s(4, 4, -1)
        a.fbr('ne', 4, 0, 'rd', imm=True)          # > 64 cycles: the sweep is over
        a.call('A')
        a.label('rr')
        a.ret()
        _ic_pad(a, A)
        a.label('A')
        a.alui_s('add', 2, 2, 1); a.ret()          # (old code: +1)
        for _ in range(4):
            a.nop()
        a.label('A3')
        a.jmp('R'); a.nop()
        out.append((a, {'end': 'park', 'kind': '0', 'r2': 16}, [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_smc_pending():
    """self-modifying code against a pending install: a cold function F
    whose first instruction stores into a later word of F's own 32 B line
    (the other 16 B half, or the same) while F's line answer is still to
    come — the install must be dropped (or cleared by the store's snoop),
    the BTIC window captured from the fill killed, and F runs the new word,
    now and on two later calls."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    F = 0x21000          # (above 64 KiB: the shadow's power-up tag 0 would match it)
    for p in (2, 5, 6):
        for lat in (1, 3, 5):
            for ld in (0, 1, 2, 3, 5, 8, 12):
                a = Asm()
                a.setr(1, 0); a.setr(2, F + 4 * p); a.setr(3, new)
                a.call('F')
                a.setr(2, DATA + 0x100)
                a.call('F')                    # the store now writes data: F as patched
                a.call('F')
                a.halt()
                _ic_pad(a, F)
                a.label('F')
                a.st(32, 3, 2)
                for _ in range(p - 1):
                    a.nop()
                a.alui_s('add', 1, 1, 1)       # F + 4p
                a.ret()
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 3 * 16},
                            [f'+LAT={lat}', f'+LDLY={ld}']))
    # (the first call's CALL captured F's first window in the BTIC from the
    # fill — before the store; the store must kill it, also when it drops
    # the pending install, or the later calls run the old word from it)
    return out



@test
def ic32_smc_btic():
    """the BTIC window of a line whose install is pending: one CALL site
    calls F three times.  The first call (cold: F's line filled, its first
    window captured in the BTIC) branches at once to the warm G, which
    patches F's word 2 — inside that window — while F's line answer is still
    to come; the later calls hit the BTIC entry, which the store must have
    killed (the dropped install does not), so they run the new word."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    F, G = 0x21000, 0x22000
    for lat in (1, 3):
        for ld in (0, 6, 12, 20):
            a = Asm()
            a.setr(1, 0); a.setr(5, 0); a.setr(2, DATA + 0x100); a.setr(3, new)
            a.call('G')                        # warm G (its store: data)
            a.setr(2, F + 8); a.setr(5, 1); a.setr(11, 3)
            a.label('lp')
            a.call('F')                        # one call site: one BTIC entry
            a.addi_s(11, 11, -1)
            a.fbr('ne', 11, 0, 'lp', imm=True)
            a.halt()
            _ic_pad(a, F)
            a.label('F')
            a.fbr('ne', 5, 0, 'G', imm=True)   # (first call: to G)
            while a.pc != F + 8:
                a.nop()
            a.alui_s('add', 1, 1, 1)           # F + 8: patched to +16
            a.ret()
            _ic_pad(a, G)
            a.label('G')
            a.st(32, 3, 2)
            a.setr(5, 0)
            a.ret()
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 32}, [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_drop_btic():
    """a dropped install must still cover the BTIC window captured from its
    fill (the review's scenario).  A cold 16 B loop at the start of the 32 B
    line X stores into X's other half every iteration: the first store
    issues while X's line answer is pending (the pending-line snoop kills
    the BTIC entries of X that exist then), the line arrives with the store
    on the port and the install is dropped, and the loop branch's BTIC
    capture comes after that kill, from the window the early restart
    filled.  After the loop, the loop's first word is patched (snoop-
    coherent: no ICACHE_INV) and the loop entered again, cold, its line
    late: a BTIC hit taken before that fill's install must not bring back
    the old word (a <= 16 B loop then runs from the window for good).
    (With EN_ST1 the dropping store is posted speculatively, does not pass
    the post check and squashes everything younger, the window included,
    so no capture from that fill's window outlives its snoop: this passes
    on 112fc8b too, as it does with EN_ST1 = 0.  btic_unc — tb_vsim's
    check that every valid entry's line is in the tag shadow or pending —
    stays 0; ic32_dbg_write is the case where a plain drop broke that.)"""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    nops = _word(lambda t: (t.nop(), t.nop()))
    X = 0x21000          # (above 64 KiB: the shadow's power-up tag 0 is another line's)
    for ldv in (False, True):
        for lat in (1, 2, 3, 5):
            for ld in (4, 6, 8, 10, 12, 16):
                a = Asm()
                a.setr(1, 0); a.setr(2, X + 16); a.setr64(3, nops | nops << 32); a.setr(13, 3)
                a.setr(5, DATA + 0x200); a.setr(12, 0)
                a.jmp('X')
                a.label('after')
                a.setr(6, new); a.setr(7, X); a.st(32, 6, 7)      # patch X + 0
                a.setr(2, DATA + 0x100); a.setr(1, 0); a.setr(13, 40); a.setr(12, 1)
                a.jmp('X')
                a.label('done')
                a.halt()
                _ic_pad(a, X)
                a.label('X')
                a.alui_s('add', 1, 1, 1)           # X + 0: patched to +16
                if ldv:
                    a.ld(64, 4, 5)                 # (the store chains onto this load's answer)
                else:
                    a.nop()
                a.st(64, 3, 2)                     # X + 16 (the line's other half), then data
                a.fbr('lt', 1, 13, 'X')            # X + 12
                a.nop(); a.nop()                   # X + 16: the stored dword (two NOPs)
                a.fbr('ne', 12, 0, 'dn', imm=True)
                a.jmp('after')
                a.label('dn')
                a.jmp('done')
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 48, 'btic_unc': '0'},
                            [f'+LAT={lat}', f'+LDLY={ld}']))
    return out

ICINV_A = 0xF0050000      # CACHE_CTRL: bit 3 = ICACHE_INV


def _icinv(a, ra=13, rv=14):
    a.setr(ra, ICINV_A); a.setr(rv, 8); a.st(32, rv, ra)


def _spin(a, reg, n, tag):
    """a counted delay loop (about 2 cycles an iteration)"""
    a.setr(reg, n)
    a.label(tag)
    a.addi_s(reg, reg, -1)
    a.fbr('ne', reg, 0, tag, imm=True)


@test
def ic32_fill_cyc():
    """PERF_IC_FILL_CYC with 32 B lines: straight 1-word code through 40
    cold lines whose line answers come LDLY cycles after the early restart.
    The fetch's wait for the rest of each line counts as fill time (it grows
    with LDLY: about LDLY + LAT a line), not as window bubble"""
    out = []
    if not IC32:
        return out
    for ld in (8, 16):
        a = Asm()
        a.setr(3, 0)
        a.jmp('S')
        while a.pc % 32:
            a.nop()
        a.label('S')
        for _ in range(8 * 40):
            a.inc(3)
        a.halt()
        out.append((a, {'end': 'park', 'kind': '0', 'r3': 320,
                        'ic_fill': (40 * ld - 20, 40 * (ld + 6) + 20)}, ['+LAT=3', f'+LDLY={ld}']))
    return out


@test
def sh_bz_form():
    """the shifter's zero test for ex_b_zero (bz_mem) from the stage-1
    registers' byte-zero flags: every shift kind (SHL / SHR / SAR / ROL /
    ROR, BTS / BTR / BTC, BTSTRR), immediate and register counts, on sparse
    random values, each followed at once by a consumer of its result that
    tests 32 bits (SETCC W: id_w) or 64, or by a DIV / MOD dividing by it.  pipeline_core asserts the
    byte-flag form equal to the 64-bit test every cycle (and ex_b_zero
    equal to ex_b == 0); the DIVs check the divide-by-zero results."""
    import random
    out = []
    rnd = random.Random(5)
    for v in range(10):
        a = Asm()
        a.setr(12, 0)
        for _ in range(200):
            sub = rnd.choice((0, 1, 2, 3, 4, 8, 9, 10, 11))
            n = rnd.randrange(64)
            val = 0
            r = rnd.random()
            if r < 0.5:      # one non-zero byte: often the BT ops' own bit
                val = rnd.choice((1 << (n % 8), 1 << (n % 8), rnd.randrange(1, 256))) << (8 * rnd.randrange(8))
                if rnd.random() < 0.5:
                    val = rnd.choice((1 << (n % 8), rnd.randrange(1, 256))) << (8 * (n // 8))
            elif r < 0.75:
                for b in range(8):
                    if rnd.random() < 0.35:
                        val |= rnd.choice((1, 0x80, rnd.randrange(256))) << (8 * b)
            a.setr64(1, val)
            if sub == 11 or rnd.random() < 0.5:
                a.setr(2, n)
                a.emit(H(1, 4) | sub << 22 | 3 << 8 | 1 << 4 | 2)          # rd 3 = r1 op r2
            else:
                a.emit(H(1, 4) | sub << 22 | 1 << 21 | n << 15 | 3 << 8 | 1 << 4)
            c = rnd.randrange(3)
            if c == 0:                     # (id_w: a 32-bit compare / FBR reads rs2)
                a.setcc('eq', 4, 0, 3, w=True)
                a.alu('add', 6, 0, 3)
            elif c == 1:
                a.alu('add', 4, 0, 3)
                a.setcc('eq', 6, 0, 3, w=True)
            else:
                a.setr(5, 1000003)
                a.div(4, 5, 3, sgn=rnd.random() < 0.5)
            a.alu('add', 12, 12, 4)
        a.halt()
        out.append((a, {'end': 'park', 'kind': '0'}, [f'+LAT={1 + v % 3}']))
    return out


@test
def ic32_vdefer_snoop():
    """a deferred valid set met by the snoop of a store to its own line
    (g_vl's vdrop): a cold function L stores first into the line J — in the
    tag shadow but invalid, so the store posts and its snoop clears J's
    index — then into L's own second half.  L's line answer arrives on the
    first store's snoop edge (L's set waits behind that clear) with the
    second store on its issue edge (queued, not yet on the port: no drop);
    the second store's snoop must cancel the waiting set, or L's line turns
    valid with the word from before the store.  Swept over the line delay
    so the arrival meets that edge."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    J, L = 0x22000, 0x23000
    for lat in (1, 2, 3):
        for ld in range(1, 11):
            a = Asm()
            a.setr(1, 0); a.setr(4, 0); a.setr(8, J + 24); a.setr(9, L + 16); a.setr(3, new)
            a.call('J')                        # J installed
            a.st(64, 4, 8)                     # J's valid bit 0, its tag stays in the shadow
            _spin(a, 5, 8, 'w0')
            a.call('L')                        # +16 (the store's squash refetches L + 16)
            a.call('L2')                       # +16
            a.halt()
            _ic_pad(a, J)
            a.label('J'); a.ret()
            _ic_pad(a, L)
            a.label('L')
            a.st(64, 4, 8)                     # J + 24: a snoop hit on an invalid line (posts)
            a.st(32, 3, 9)                     # L + 16: patched to +16
            _ic_pad(a, L + 16)
            a.label('L2')
            a.alui_s('add', 1, 1, 1)
            a.ret()
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 32}, [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_sweep_snoop():
    """a line installed during a flash-clear sweep (its valid set waits for
    the sweep's end) and then written by a snooped store: the store must
    cancel the waiting set (vdrop), or the line turns valid after the sweep
    with the word from before the store.  L (one window: no lookup after
    its fill) stores into its own last dword — outside the window, so it
    posts and nothing refetches L — and returns; the return's line answers
    after the sweep (+LDLYA), so no other install replaces L's waiting set
    meanwhile.  After the sweep a call to L's last dword must run the new
    word."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    L = 0x23000
    for lat in (1, 3):
        for ld in (0, 1, 2):
            for mn in (0, 90):
                a = Asm()
                a.setr(1, 0); a.setr(9, L + 24); a.setr(3, new)
                _icinv(a)                          # the sweep (main's lines were valid)
                while (a.pc + 4) % 32:
                    a.nop()
                a.call('L')                        # cold, inside the sweep
                a.label('back')                    # (a line of its own)
                _spin(a, 5, 50, 'w1')              # past the sweep (64 cycles)
                a.call('L2')                       # +16
                a.halt()
                _ic_pad(a, L)
                a.label('L')
                a.nop(); a.nop()                   # (the install lands before the store's MEM)
                a.st(32, 3, 9)                     # L + 24: patched to +16
                a.ret()
                _ic_pad(a, L + 24)
                a.label('L2')
                a.alui_s('add', 1, 1, 1)
                a.ret()
                e = {'end': 'park', 'kind': '0', 'r1': 16}
                if IC32:
                    e['ic_sweeps'] = (1, 9)        # (16 B lines: a one-cycle flash clear)
                out.append((a, e,
                            [f'+LAT={lat}', f'+LDLY={ld}', '+LDLYA=%x' % a.labels['back'],
                             f'+LDLYN={mn}']))
    return out


@test
def ic32_inv_defer():
    """ICACHE_INV while a valid set waits for the end of a sweep (swgo's
    vs_pend <= 0), i.e. a sweep restarted mid-sweep with a deferred set: a
    fence starts a sweep; L's second half is entered cold inside it (its
    line's set waits); from that window L has the non-coherent writer
    rewrite L's first half and fences again — mid-sweep — as the line's
    last word.  The refetch after that fence is of the next line, whose
    answer comes after the new sweep (+LDLYA): no install replaces the
    deferred set meanwhile.  A call to L's first half must then run the new
    word: the set from before the fence would revalidate the line read
    before the rewrite."""
    out = []
    newdw = _word(lambda t: t.alui_s('add', 1, 1, 16)) | (_word(lambda t: t.ret()) << 32)
    L = 0x23000
    for lat in (1, 3):
        for ld in (0, 2, 5):
            for mn in (0, 90):
                a = Asm()
                a.setr(1, 0); a.setr(8, POKE_A); a.setr(7, POKE_D); a.setr(9, L)
                a.setr64(6, newdw); a.setr(13, ICINV_A); a.setr(14, 8)
                a.st(32, 14, 13)                   # ICACHE_INV: sweep 1
                a.call('Lh')                       # cold, inside sweep 1
                _spin(a, 5, 60, 'w1')              # past sweep 2
                a.call('L')                        # +16
                a.halt()
                _ic_pad(a, L)
                a.label('L')
                a.alui_s('add', 1, 1, 1); a.ret()  # (rewritten: +16)
                _ic_pad(a, L + 16)
                a.label('Lh')
                a.st(64, 9, 8); a.st(64, 6, 7)     # poke L + 0 (no snoop)
                a.nop()
                a.st(32, 14, 13)                   # L + 28: ICACHE_INV, sweep 2 (restarts sweep 1)
                a.ret()                            # L + 32: the next line
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 16},
                            [f'+LAT={lat}', f'+LDLY={ld}', '+LDLYA=%x' % (L + 32), f'+LDLYN={mn}']))
    return out


@test
def ic32_sweep_restart():
    """ICACHE_INV during a sweep restarts it: a warm P is rewritten by the
    non-coherent writer, fenced, rewritten again and fenced again k cycles
    later (mid-sweep for small k; P called in between for some), then
    called: it must run the second rewrite"""
    out = []
    w = lambda v: _word(lambda t: t.alui_s('add', 1, 1, v)) | (_word(lambda t: t.ret()) << 32)
    P = 0x23000
    for gap in (0, 1, 4, 20, 40, 70):
        for mid in (False, True):
            for ld in (0, 6):
                a = Asm()
                a.setr(1, 0); a.setr(8, POKE_A); a.setr(7, POKE_D); a.setr(9, P)
                a.call('P')                        # +1, P warm
                a.setr64(6, w(4)); a.st(64, 9, 8); a.st(64, 6, 7)
                _icinv(a)
                if gap:
                    _spin(a, 5, gap, 'g')
                if mid:
                    a.call('P')                    # +4 (refilled, maybe inside the sweep)
                a.setr64(6, w(16)); a.st(64, 9, 8); a.st(64, 6, 7)
                _icinv(a)
                a.call('P')                        # +16
                a.halt()
                _ic_pad(a, P)
                a.label('P')
                a.alui_s('add', 1, 1, 1)
                a.ret()
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 1 + (4 if mid else 0) + 16},
                            [f'+LDLY={ld}']))
    return out


@test
def ic32_dbg_write():
    """DEBUG_EN builds: a PC breakpoint inside a cold line X whose line
    answer is still to come; at the halt the debugger writes X's second
    dword through the core's port (no snoop), so the install is dropped
    (the write is on the port as the line arrives) or, without a delay,
    the line installed before it.  After the resume (a sweep, the BTIC
    emptied) X must run the new code, twice."""
    out = []
    if not _os0.environ.get('VSIM_DEBUG_EN') == '1':
        return out
    newdw = _word(lambda t: t.alui_s('add', 1, 1, 16)) | (_word(lambda t: t.ret()) << 32)
    X = 0x24000
    for lat in (1, 3):
        for ld in (0, 4, 12, 30):
            a = Asm()
            a.setr(1, 0)
            a.call('X')
            a.call('X')
            a.halt()
            _ic_pad(a, X)
            a.label('X')
            a.nop(); a.nop()
            a.alui_s('add', 1, 1, 1); a.ret()      # X + 8: rewritten to +16
            # (btic_unc: tb_vsim's check that every valid BTIC entry's line
            # is in the snoop's tag shadow or pending — the CALL's capture
            # from X's fill must die with the dropped install: a plain drop
            # left it uncovered until the resume emptied the BTIC)
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 32, 'btic_unc': '0'},
                        [f'+LAT={lat}', f'+LDLY={ld}', '+DBG_BP0=%x' % (X + 4),
                         '+DBG_MW=%x' % (X + 8), '+DBG_MD=%x' % newdw]))
    return out


@test
def ic32_btic_inst_edge():
    """review M1: a BTIC capture landing on an install edge.  O (warm) and L
    (cold) share an I-cache index (64 KiB apart).  L's fill serves the
    window, a CALL to O misses the BTIC, O's window is served from the
    cache (O still valid), and L's line answer (+LDLYA / +LDLYN, swept)
    lands around the capture: on its edge (the review's case), on the
    edge O's window is served (the capture is the next edge), or on the
    edge O's line is read (served after it).  The install overwrites the
    tag shadow with L, so an entry captured from O's window would survive
    every later store to O.  O's first word is then patched (snoop-coherent
    store, no ICACHE_INV) and the CALL run again: it must run the new word,
    and btic_unc (tb_vsim's coverage check) must stay 0.  The CALL is L's
    own (direct: then it and O share the index) or B's (via: L jumps to B,
    warm, at another index).  wr: a store into L's other half while L is
    pending (install-invalid, the review's scenario)."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    nops = _word(lambda t: (t.nop(), t.nop()))
    O, L, B = 0x21000, 0x31000, 0x24000
    for via in (False, True):
        for wr in (False, True):
            for lat in (1, 3):
                for mn in range(0, 15 if not via else 21):
                    a = Asm()
                    a.setr(1, 0); a.setr(12, 0); a.setr(6, new); a.setr(7, O); a.setr(9, L + 24)
                    a.setr64(3, nops | nops << 32)
                    a.call('O')                        # O warm: +1
                    a.call('Bw')                       # B warm
                    a.jmp('L')
                    a.label('after')
                    a.setr(12, 1)
                    a.st(32, 6, 7)                     # patch O + 0: +16
                    a.jmp('L')
                    a.label('done')
                    a.halt()
                    _ic_pad(a, O)
                    a.label('O')
                    a.alui_s('add', 1, 1, 1)
                    a.ret()
                    _ic_pad(a, B)
                    a.label('B')
                    a.call('O')                        # (via) the capture of O's window
                    a.jmp('L2')
                    a.label('Bw')
                    a.ret()
                    _ic_pad(a, L)
                    a.label('L')
                    if wr:
                        a.st(64, 3, 9)                 # L + 24, unchanged (pending: install-invalid)
                    else:
                        a.nop()                        # (the CALL's ID latch is not the JMP's capture edge)
                    if via:
                        a.jmp('B')
                    else:
                        a.call('O')                    # the capture of O's window
                        a.jmp('L2')
                    _ic_pad(a, L + 32)                 # (L + 24: two NOPs, the stored dword)
                    a.label('L2')
                    a.fbr('ne', 12, 0, 'dn', imm=True)
                    a.jmp('after')
                    a.label('dn')
                    a.jmp('done')
                    out.append((a, {'end': 'park', 'kind': '0', 'r1': 18, 'btic_unc': '0'},
                                [f'+LAT={lat}', '+LDLYA=%x' % L, f'+LDLYN={mn}']))
    return out


@test
def ic32_btic_win_loop():
    """review M1, generalised: the window outlives the install that replaces
    its line.  L (cold, its line late: +LDLY) calls O, a 16 B loop in one
    window at O's index; the loop runs from that window (no refetch) while
    L's install replaces O, so the loop branch's BTIC capture — every one
    after the install kills the earlier — copies a line the shadow no
    longer holds.  O's first word is then patched (snoop-coherent) and the
    loop entered again at O + 4 (O refetched, its line late): a BTIC hit
    taken before that install must not run the old word."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    O, L = 0x21000, 0x31000
    for lat in (1, 3):
        for ld in (2, 4, 6, 9, 12, 16, 20, 28):
            a = Asm()
            a.setr(1, 0); a.setr(6, new); a.setr(7, O)
            a.setr(2, 4); a.call('O')                  # O warm, the branch trained: +4
            a.setr(2, 40); a.jmp('L')                  # +40
            a.label('after')
            a.st(32, 6, 7)                             # patch O + 0: +16
            a.setr(2, 2); a.call('O4')                 # +16 (one pass from O + 0)
            a.halt()
            _ic_pad(a, O)
            a.label('O')
            a.alui_s('add', 1, 1, 1)
            a.label('O4')
            a.addi_s(2, 2, -1)
            a.fbr('ne', 2, 0, 'O', imm=True)           # O + 8
            a.ret()
            _ic_pad(a, L)
            a.label('L')
            a.nop()
            a.call('O')
            a.jmp('after')
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 4 + 40 + 16, 'btic_unc': '0'},
                        [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_btic_nlb():
    """review M1, generalised: the NLB outlives the install that replaces its
    line.  L (cold, its line late: +LDLY) calls O; O's window is served
    and the prefetch takes O's other half (O + 16) into the NLB; a load
    (held behind L's line answer) stalls O until after L's install, then
    O's JMP to O + 16 is served from the NLB and captured.  O + 16 is then
    patched (snoop-coherent) and O's JMP entered again from its own dword
    (O refetched, its line late): a BTIC hit taken before that install
    must not run the old word.  Second form: the JMP is in A2, the half
    line before O (another index), and the NLB holds O itself."""
    out = []
    new = _word(lambda t: t.alui_s('add', 1, 1, 16))
    O, L = 0x21000, 0x31000
    for lat in (1, 3):
        for ld in (2, 4, 6, 9, 12, 16, 20):
            a = Asm()
            a.setr(1, 0); a.setr(6, new); a.setr(7, O + 16); a.setr(5, DATA)
            a.call('O')                                # O warm: +1
            a.jmp('L')                                 # +1
            a.label('after')
            a.st(32, 6, 7)                             # patch O + 16: +16
            a.call('O8')                               # +16
            a.halt()
            _ic_pad(a, O)
            a.label('O')
            a.ld(64, 4, 5)
            a.alui_s('add', 4, 4, 1)                   # (waits for the load)
            a.label('O8')
            a.jmp('O16')                               # O + 8
            a.nop()
            a.label('O16')
            a.alui_s('add', 1, 1, 1)                   # O + 16
            a.ret()
            _ic_pad(a, L)
            a.label('L')
            a.nop()
            a.call('O')
            a.jmp('after')
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 18, 'btic_unc': '0'},
                        [f'+LAT={lat}', f'+LDLY={ld}']))
    # the branch elsewhere: A2, the second half of the line before O (warm),
    # jumps to O, which the NLB took as the line after A2's window (or, from
    # the first half, A2 - 16: as the line after that, the second entry)
    for back in (16, 32):
        A2 = O - back
        for lat in (1, 3):
            for ld in (2, 4, 6, 9, 12, 16, 20):
                a = Asm()
                a.setr(1, 0); a.setr(6, new); a.setr(7, O); a.setr(5, DATA)
                a.call('A2')                               # A2 and O warm: +1
                a.jmp('L')                                 # +1
                a.label('after')
                a.st(32, 6, 7)                             # patch O + 0: +16
                a.call('A2')                               # +16
                a.halt()
                _ic_pad(a, A2)
                a.label('A2')
                a.ld(64, 4, 5)
                a.alui_s('add', 4, 4, 1)                   # (waits for the load)
                a.jmp('O')                                 # A2 + 8
                _ic_pad(a, O)
                a.label('O')
                a.alui_s('add', 1, 1, 1)
                a.ret()
                _ic_pad(a, L)
                a.label('L')
                a.nop()
                a.call('A2')
                a.jmp('after')
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 18, 'btic_unc': '0'},
                            [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_btic_br_win():
    """review M1, the branch's line from a window the install has replaced:
    L (cold, its line late: +LDLY) calls O; O's window is served, a load
    held behind L's line answer stalls the op after it in ID, so O's JMP to
    P (another index, warm) enters ID after L's install replaced O — from
    the window that still holds O.  Its capture (P's window: covered) would
    leave the branch's line uncovered.  O's JMP is then rewritten to jump
    to Q (not a snoop hit: O is out) and O entered again, its line late: a
    BTIC hit by the JMP's pc before that install would run P's code at Q."""
    out = []
    O, L, P, Q = 0x21000, 0x31000, 0x22000, 0x23000
    for lat in (1, 3):
        for ld in (2, 4, 6, 9, 12, 16, 20):
            t = Asm(base=O + 8)
            t.jmp(Q)
            neww = t.assemble()[0]
            a = Asm()
            a.setr(1, 0); a.setr(6, neww); a.setr(7, O + 8); a.setr(5, DATA); a.setr(12, 0)
            a.call('O')                                # O and P warm: +1
            a.jmp('L')                                 # P: +1
            a.label('after')
            a.setr(12, 1)
            a.st(32, 6, 7)                             # O's JMP now jumps to Q
            a.call('O')                                # Q: +16
            a.halt()
            _ic_pad(a, O)
            a.label('O')
            a.ld(64, 4, 5)
            a.alui_s('add', 4, 4, 1)                   # (waits for the load)
            a.jmp('P')                                 # O + 8
            _ic_pad(a, P)
            a.label('P')
            a.alui_s('add', 1, 1, 1)
            a.ret()
            _ic_pad(a, Q)
            a.label('Q')
            a.alui_s('add', 1, 1, 16)
            a.ret()
            _ic_pad(a, L)
            a.label('L')
            a.nop()
            a.call('O')
            a.jmp('after')
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 18, 'btic_unc': '0'},
                        [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_btic_br_alias():
    """review M1, the branch's line: a CALL at the end of L to O, 64 KiB
    away at L's index.  O's fill replaces L, so the CALL's capture (of O's
    window) is of a branch whose line the shadow no longer holds, and the
    return lands in the next line (L stays out).  The CALL is then
    rewritten to call X (a store to L: not a snoop hit) and run again from
    a fill of L whose line is late: a BTIC hit by the CALL's pc before that
    install would run O's code at X."""
    out = []
    O, L, X = 0x21000, 0x31000, 0x22000
    for lat in (1, 3):
        for mn in (0, 2, 4, 8, 16):
            t = Asm(base=L + 28)
            t.call(X)
            neww = t.assemble()[0]
            a = Asm()
            a.setr(1, 0); a.setr(12, 0); a.setr(6, neww); a.setr(7, L + 28)
            a.jmp('L24')                               # O: +1
            a.label('after')
            a.setr(12, 1)
            a.st(32, 6, 7)                             # the CALL now calls X
            a.jmp('L24')                               # X: +16
            a.label('done')
            a.halt()
            _ic_pad(a, O)
            a.label('O')
            a.alui_s('add', 1, 1, 1)
            a.ret()
            _ic_pad(a, X)
            a.label('X')
            a.alui_s('add', 1, 1, 16)
            a.ret()
            _ic_pad(a, L + 24)
            a.label('L24')
            a.nop()                                    # (the CALL's ID latch is not the JMP's capture edge)
            a.call('O')                                # L + 28
            a.fbr('ne', 12, 0, 'dn', imm=True)         # L + 32: the next line
            a.jmp('after')
            a.label('dn')
            a.jmp('done')
            out.append((a, {'end': 'park', 'kind': '0', 'r1': 17, 'btic_unc': '0'},
                        [f'+LAT={lat}', '+LDLYA=%x' % L, f'+LDLYN={mn}']))
    return out


# ---- HW_PERF_PLAN §5 review 3: BTIC soundness (both line sizes) ----
@test
def btic_strad():
    """review 3, a branch across a line end: a 2-word JMP / CALL abs at a
    line's last word (A + 28: the last word of a 32 B and of a 16 B line)
    keeps its target in the next line.  It runs once (to T1), then a store
    rewrites that target word (a snoop of the NEXT line, not of the
    branch's) and it runs again: it must reach T2 (r1 = 1 + 16).  A BTIC
    entry {A + 28 -> T1} that survived the store would replay T1's window
    at the new target."""
    out = []
    for A, T1, T2 in ((0x21000, 0x22000, 0x23000), (0x21040, 0x22100, 0x23200)):
        for kind in ('jmp', 'call'):
            for lat in (1, 3):
                for extra in ([], ['+LDLY=4'], ['+LDLY=9']):
                    a = Asm()
                    a.setr(1, 0); a.setr(12, 0); a.setr(6, T2); a.setr(7, A + 32)
                    a.jmp('A28')
                    a.label('back')
                    a.fbr('ne', 12, 0, 'dn', imm=True)
                    a.setr(12, 1)
                    a.st(32, 6, 7)                              # rewrite the word in the next line
                    a.jmp('A28')
                    a.label('dn')
                    a.halt()
                    _ic_pad(a, A + 28)
                    a.label('A28')
                    if kind == 'jmp':
                        a.jmp_abs(T1)
                    elif kind == 'call':
                        a.jmp_abs(T1, link=True)
                        a.jmp('back')                           # (A + 36: the return)
                    _ic_pad(a, T1)
                    a.label('T1')
                    a.alui_s('add', 1, 1, 1)
                    a.ret() if kind == 'call' else a.jmp('back')
                    _ic_pad(a, T2)
                    a.label('T2')
                    a.alui_s('add', 1, 1, 16)
                    a.ret() if kind == 'call' else a.jmp('back')
                    out.append((a, {'end': 'park', 'kind': '0', 'r1': 17,
                                    'btic_unc': '0', 'btic_stale': '0'}, [f'+LAT={lat}'] + extra))
    return out


def _btic_pend_prog(P, T1, T3, gap, patch, new=False, pdw=0):
    """review 3, a pending capture across a squash: J at P (`jmp T1`) arms
    the capture {P -> T1} on its first ID cycle; an older patch rewrites P
    to `fbr ne r12, 0, T3` and squashes J (`patch(a)`: an SMC store, or a
    non-coherent write and ICACHE_INV).  The refetched P falls through to
    a JMPR to T1 (no arm): a capture then would pair the new P with T1's
    window, which the BTIC replays at T3 once the PHT says taken.  T3 runs
    6 times: r1 = 1 + 6 * 16.  (new: the program as patched; pdw: the
    patched dword at P & ~7, for the non-coherent writer.)"""
    t = Asm(base=P)
    t.fbr('ne', 12, 0, T3, imm=True)
    neww = t.assemble()[0]
    a = Asm()
    a.setr(1, 0); a.setr(12, 0); a.setr(13, 0); a.setr(6, neww); a.setr(7, P); a.setr(8, T1)
    a.setr64(9, pdw)
    a.setr(10, 0xF0FE0000); a.setr(11, 0xF0050000); a.setr(14, 8)
    a.jmp('S')
    pt = Asm()
    patch(pt)
    _ic_pad(a, P - 4 * (gap + len(pt.assemble())))
    a.label('S')
    patch(a)
    for _ in range(gap):
        a.nop()
    assert a.pc == P
    a.label('P')
    if new:
        a.fbr('ne', 12, 0, T3, imm=True)
    else:
        a.jmp(T1)                 # J: unconditional, predicted taken -> arm {P, T1}
    a.jmpr(8)                     # P + 4: indirect to T1 (no arm)
    _ic_pad(a, T3)
    a.label('T3')
    a.alui_s('add', 1, 1, 16); a.addi_s(13, 13, 1); a.fbr('eq', 13, 6, 'fin', imm=True); a.jmp('P')
    a.label('fin')
    a.halt()
    _ic_pad(a, T1)
    a.label('T1')
    a.alui_s('add', 1, 1, 1); a.setr(12, 1); a.jmp('P')
    return a


def _btic_pend(patch, gaps, places, lats, extras=([], ['+LDLY=4'], ['+LDLY=9'])):
    out = []
    for P, T1, T3 in places:
        for gap in gaps:
            n = _btic_pend_prog(P, T1, T3, gap, patch, new=True)
            ws = n.assemble()
            i = ((P & ~7) - n.base) // 4
            pdw = ws[i] | ws[i + 1] << 32
            for lat in lats:
                for extra in extras:
                    a = _btic_pend_prog(P, T1, T3, gap, patch, pdw=pdw)
                    out.append((a, {'end': 'park', 'kind': '0', 'r1': 97, 'btic_unc': '0', 'btic_stale': '0'},
                                [f'+LAT={lat}'] + extra))
    return out


@test
def btic_pend_snoop():
    """review 3: the pending capture across an SMC squash (a coherent store
    S, `gap` NOPs before J, rewrites J)"""
    def patch(a):
        a.st(32, 6, 7)
    return _btic_pend(patch, range(0, 4), ((0x21010, 0x24000, 0x21800), (0x21018, 0x24040, 0x21800)),
                      (1, 2, 3, 5))


@test
def btic_pend_inv():
    """review 3: the pending capture across ICACHE_INV — J is rewritten
    behind the core's back (tb_vsim's non-coherent writer: no snoop, no
    SMC squash), then ICACHE_INV, older than J, squashes J and refetches"""
    def patch(a):
        a.st(32, 7, 10)                   # the writer's address: P
        a.st(64, 9, 10, 8)                # the new dword at P & ~7
        a.st(32, 14, 11)                  # ICACHE_INV
    # (P + 0x1C: ICACHE_INV and J in one 16 B line; a long memory latency:
    # with 32 B lines the refetches during ICACHE_INV's sweep install no
    # valid line, and a later install of P's line would kill the entry)
    return _btic_pend(patch, range(0, 4), ((0x21010, 0x24000, 0x21800), (0x21018, 0x24040, 0x21800),
                                           (0x2101C, 0x24040, 0x21800)), (1, 3, 40, 70), ([], ['+LDLY=4']))


@test
def btic_oddfill():
    """review 3, 16 B lines: a fill at a 32 B line's dword 1 (X = L + 8)
    returns the next dword too, the first of the other 16 B line (J at
    L + 16), into window slot 1 — but installs only X's line.  J (`jmp T`)
    runs from that copy; a store then rewrites it to `jmp T2` (no snoop
    hit: J's line is not in the I-cache) and A, at L's index, evicts X's
    line; X then misses again and J comes from slot 1 again.  A BTIC entry
    {J -> T} captured the first time would survive both (its lines' indices
    see no snoop, no install) and replay T at T2: r1 = 1 + 16.  (With 32 B
    lines the fill installs J's line too: the same program, the ordinary
    snoop.)"""
    out = []
    for L, T, T2 in ((0x21000, 0x22000, 0x23000), (0x21040, 0x22100, 0x23200)):
        A = L + IC_BYTES
        for lat in (1, 3):
            for extra in ([], ['+LDLY=4']):
                t = Asm(base=L + 16)
                t.jmp(T2)
                neww = t.assemble()[0]
                a = Asm()
                a.setr(1, 0); a.setr(12, 0); a.setr(6, neww); a.setr(7, L + 16)
                a.jmp('X')
                a.label('back')
                a.fbr('ne', 12, 0, 'dn', imm=True)
                a.setr(12, 1)
                a.st(32, 6, 7)                       # J now jumps to T2
                a.jmp('A')                           # evict X's line, then X again
                a.label('dn')
                a.halt()
                _ic_pad(a, L + 8)
                a.label('X')
                a.nop(); a.nop()
                a.label('J')
                a.jmp(T)                             # L + 16
                _ic_pad(a, T)
                a.label('T')
                a.alui_s('add', 1, 1, 1); a.jmp('back')
                _ic_pad(a, T2)
                a.label('T2')
                a.alui_s('add', 1, 1, 16); a.jmp('back')
                _ic_pad(a, A)
                a.label('A')
                a.jmp('X')
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 17, 'btic_unc': '0', 'btic_stale': '0'},
                            [f'+LAT={lat}'] + extra))
    return out


@test
def btic_nlb_inst():
    """review 3, 16 B lines: the NLB outlives the install that replaces its
    line.  Pass 1 runs L0 and O (= L0 + 16, J: `jmp T`) once.  Pass 2: L0's
    window is served and the prefetch takes O into the NLB; L0's JMPR then
    goes to B, O's alias (O + the I-cache size), whose fill replaces O; B
    jumps back to L0 + 8, whose slot 1 is O from the NLB — J's capture
    would leave its line uncovered (the coverage check; no wrong result
    here: a store to O kills the NLB copy).  (32 B lines: the same class,
    review M1's NLB kill.)"""
    out = []
    for L0, T in ((0x21000, 0x22000), (0x21040, 0x22100), (0x21020, 0x22200)):
        B = L0 + 16 + IC_BYTES
        for lat in (1, 2, 3):
            for extra in ([], ['+LDLY=4'], ['+HIT=50']):
                a = Asm()
                a.setr(1, 0); a.setr(12, 0); a.setr(9, L0 + 8)
                a.jmp('L0')
                _ic_pad(a, L0)
                a.label('L0')
                a.jmpr(9)                              # pass 1: L0 + 8; pass 2: B
                a.nop()
                a.label('L8')
                a.nop(); a.nop()
                a.jmp(T)                               # J at O = L0 + 16
                _ic_pad(a, T)
                a.label('T')
                a.alui_s('add', 1, 1, 1)
                a.fbr('ne', 12, 0, 'dn', imm=True)
                a.setr(12, 1); a.setr(9, B)
                a.jmp('L0')
                a.label('dn')
                a.halt()
                _ic_pad(a, B)
                a.label('B')
                a.jmp('L8')
                out.append((a, {'end': 'park', 'kind': '0', 'r1': 2, 'btic_unc': '0', 'btic_stale': '0'},
                            [f'+LAT={lat}'] + extra))
    return out


@test
def ic32_remap():
    """TE = 1: code at a VA re-mapped by TLBWI to another frame (other code)
    and back; then the same through the hardware walker (PTE rewritten,
    TLBINV, the walk refills): each call runs the current frame's code, with
    line installs delayed so one is pending at the discard"""
    out = []
    if NOEXC:
        return out
    VA = 0x00501000
    for lat, ld in ((1, 0), (3, 4), (2, 10)):
        def body(a):
            for k, fr in enumerate((2, 3, 2)):
                if k:
                    _tlb_set(a, 2, VA >> 16, fr, TLB_V | TLB_X)
                a.setr(9, VA); a.leapc(10, f'b{k}'); a.jmpr(9)
                a.label(f'b{k}')
                a.mov(6 + k, 5)
        a, e, p = _te_prog(body, False, {'r6': 0x111, 'r7': 0x222, 'r8': 0x111},
                           setup=lambda a: _tlb_set(a, 2, VA >> 16, 2, TLB_V | TLB_X))
        _place(a, 0x21000, lambda s: (s.setr(5, 0x111), s.jmpr(10)))
        _place(a, 0x31000, lambda s: (s.setr(5, 0x222), s.jmpr(10)))
        out.append((a, e, p + [f'+LAT={lat}', f'+LDLY={ld}']))
    if NOWALK:
        return out
    for lat, ld in ((1, 0), (3, 7)):
        ptes = {VA >> 16: _wk_pte(2, TLB_V | TLB_X)}
        pte_a = WK_TAB[0] + 8 * (VA >> 16)
        def body(a):
            for k, fr in enumerate((2, 3, 2)):
                if k:
                    # (the PTE through kseg: with TE = 1 a useg store is translated)
                    a.setr(1, 0x80000000 + pte_a); a.setr64(4, _wk_pte(fr, TLB_V | TLB_X)); a.st(64, 4, 1)
                    a.tlbinv()
                a.setr(9, VA); a.leapc(10, f'b{k}'); a.jmpr(9)
                a.label(f'b{k}')
                a.mov(6 + k, 5)
        a, e, p = _wk_prog(body, ptes, {'r6': 0x111, 'r7': 0x222, 'r8': 0x111})
        _place(a, 0x21000, lambda s: (s.setr(5, 0x111), s.jmpr(10)))
        _place(a, 0x31000, lambda s: (s.setr(5, 0x222), s.jmpr(10)))
        out.append((a, e, p + [f'+LAT={lat}', f'+LDLY={ld}']))
    return out


@test
def ic32_loop48k():
    """a 48 KiB loop (> 16 and > 32 KiB, < 64 KiB) after 1, 2 and 4 passes:
    with 64 KiB only the first pass misses; smaller I-caches miss again on
    every pass (the counts are those of a direct-mapped cache of this
    geometry over the loop's lines)"""
    out = []
    for passes in (1, 2, 4):
        a = Asm()                      # (_ic_loop's FBR back reaches 16 KiB only)
        a.setr(1, passes); a.setr(2, 0)
        while a.pc & 31:
            a.nop()
        a.label('l')
        for _ in range(48 * 1024 // 4 - 4):
            a.nop()
        a.inc(2)
        a.addi_s(1, 1, -1)
        a.fbr('eq', 1, 0, 'x', imm=True)
        a.jmp('l')
        a.label('x')
        a.halt()
        lo, x = a.labels['l'], a.labels['x']
        # setup (main's first line(s)), the passes (up to the JMP), then the
        # exit (the HALT's line, fetched once)
        seq = list(range(0x20, lo, IC_LB)) + \
              [v for _ in range(passes) for v in range(lo - lo % IC_LB, x, IC_LB)] + [x]
        n = _ic_dm_misses(seq)
        # (16 B lines: per-dword valid — the exit's dword can be missing
        # from a line an odd fill brought in: one miss more)
        out.append((a, {'end': 'park', 'kind': '0', 'r2': passes, 'r1': 0,
                        'ic_miss': str(n) if IC32 else (n, n + 1)}, ['+LAT=2']))
    return out


# ============ HW_PERF_PLAN §5: the 4-entry micro-ITLB (tagged by TLB index)
UT_N = 4


@test
def ut_pingpong():
    """TE = 1: calls round-robin through k code pages (TLBWI-mapped, frames
    not adjacent) ten times.  With k + 1 pages (main's included) <= 4 the
    micro-ITLB refills once per page and never after warm-up; with 5 it
    thrashes (round-robin), with the right code every time"""
    out = []
    if NOEXC:
        return out
    for k in (1, 2, 3, 4):
        vas = [0x00500100 + 0x10000 * j for j in range(k)]
        frs = [0x2 + 2 * j for j in range(k)]

        def setup(a, vas=vas, frs=frs):
            for j, (va, fr) in enumerate(zip(vas, frs)):
                _tlb_set(a, 2 + j, va >> 16, fr, TLB_V | TLB_X)

        def body(a, vas=vas):
            a.setr(3, 0); a.setr(11, 10)
            a.label('it')
            for j, va in enumerate(vas):
                a.setr(9, va); a.leapc(10, f'r{j}'); a.jmpr(9)
                a.label(f'r{j}')
            a.addi_s(11, 11, -1)
            a.fbr('ne', 11, 0, 'it', imm=True)
        exp = {'r3': 10 * sum(1 << j for j in range(k))}
        if k + 1 <= UT_N:
            exp['itlb_ref'] = str(k + 1)        # page 0 (after TE on) + each code page once
        else:
            exp['itlb_ref'] = (10 * (k + 1) - 5, 10 ** 6)
        a, e, p = _te_prog(body, False, exp, setup=setup)
        for j, (va, fr) in enumerate(zip(vas, frs)):
            _place(a, (fr << 16) | (va & 0xFFFF), lambda s, j=j: (s.addi_s(3, 3, 1 << j), s.jmpr(10)))
        out.append((a, e, p))
    return out


@test
def ut_walk_evict():
    """the walker refills code pages A, B (TLB entries 8, 9) and the micro-
    ITLB holds them; a walk of an unrelated data page (entry 10) and a TLBWI
    to an unrelated entry drop neither (no refill: the ping-pong goes on);
    a walk that overwrites A's entry 8 drops only A's micro-ITLB entry: A is
    walked again (to entry 12), B is not refilled; a TLBWI that remaps B's
    entry takes effect on B's next fetch"""
    out = []
    if NOEXC or NOWALK:
        return out
    A, B = 0x00600100, 0x00610100
    D1, D2 = 0x01000000, 0x01010000
    ptes = {A >> 16: _wk_pte(5, TLB_V | TLB_X), B >> 16: _wk_pte(6, TLB_V | TLB_X),
            D1 >> 16: _wk_pte(7, TLB_V | TLB_R), D2 >> 16: _wk_pte(8, TLB_V | TLB_R)}

    def pp(a, tag, n=3):
        a.setr(11, n)
        a.label(tag)
        a.setr(9, A); a.leapc(10, tag + 'a'); a.jmpr(9); a.label(tag + 'a')
        a.setr(9, B); a.leapc(10, tag + 'b'); a.jmpr(9); a.label(tag + 'b')
        a.addi_s(11, 11, -1)
        a.fbr('ne', 11, 0, tag, imm=True)

    def body(a):
        a.setr(3, 0)
        pp(a, 'w0')                                      # walks A -> 8, B -> 9
        a.setr(1, D1); a.ld(64, 4, 1)                    # walk -> 10
        _tlb_set(a, 20, 0x0123, 0x009, TLB_V | TLB_R)    # TLBWI to an unrelated entry
        pp(a, 'w1')
        a.setr(1, (8 << 8) | 1); a.mtsr('tlb_walk', 1)   # RR = 8: A's entry
        a.setr(1, D2); a.ld(64, 5, 1)                    # walk -> 8 (A's gone)
        a.setr(1, (12 << 8) | 1); a.mtsr('tlb_walk', 1)  # RR = 12
        pp(a, 'w2')                                      # A walked again -> 12
        _tlb_set(a, 9, B >> 16, 0xB, TLB_V | TLB_X)      # B's entry -> frame 0xB
        pp(a, 'w3', 1)
    # r3: A adds 1, B adds 0x10 (frame 6) / 0x100 (frame 0xB)
    exp = {'r3': 9 * 0x11 + 1 + 0x100, 'wk_n': '5',
           # page 0 after TE on; A, B (each: refill = TLB miss, then the
           # walk's refill); after the D1 walk / TLBWI 20: none; after the
           # D2 walk: A (miss + refill after its walk) only; B's remap: 1
           'itlb_ref': str(1 + 2 + 2 + 2 + 1)}
    a, e, p = _wk_prog(body, ptes, exp)
    _place(a, 0x50100, lambda s: (s.addi_s(3, 3, 1), s.jmpr(10)))
    _place(a, 0x60100, lambda s: (s.alui_s('add', 3, 3, 0x10), s.jmpr(10)))
    _place(a, 0xB0100, lambda s: (s.setr(7, 0x100), s.alu('add', 3, 3, 7), s.jmpr(10)))
    out.append((a, e, p))
    return out


# ================================ MMU_PLAN §11.8: the hardware TLB walker
# Page tables (the Linux format, arch/klausscpu tlbex.S): TLB_PTB -> 32
# directory dwords (VA[30:26]); D[0] = present, D & 0x07FF_E000 = an 8 KiB
# table of 1024 PTEs (VA[25:16]); a PTE is TLB_LO's layout plus [4] PRESENT.
# The junk bits must be ignored by the walker.
WK_PTB = 0x20000                      # the directory (PA)
WK_TAB = (0x22000, 0x24000)           # the tables for D[0] / D[1] (VA < 0x0800_0000)
WK_PRES = 0x10
WK_JD = 0xA5A50000F8001FFE            # junk in a directory entry (bits the walker ignores)
WK_JT = 0x5A5A0000F800FFE0            # junk in a PTE
WK_OUT = DATA + 0x600                 # result dwords (dumped; below ELOG, above FLOG)


def _wk_pte(pfn, xwrv, present=True):
    return WK_JT | (pfn << 16) | (WK_PRES if present else 0) | xwrv


def _wk_tables(a, ptes, dirs=None):
    """ptes: {va_page: pte}; dirs: {index: dword} (default: D[0], D[1]
    present); identity RWX PTEs for pages 0 and 1 (code / data window)
    unless given"""
    d = {0: WK_JD | WK_TAB[0] | 1, 1: WK_JD | WK_TAB[1] | 1}
    d.update(dirs or {})
    for i, v in d.items():
        a.dword(WK_PTB + 8 * i, v)
    p = {0x0000: _wk_pte(0, TLB_RWX), 0x0001: _wk_pte(1, TLB_RWX)}
    p.update(ptes)
    for vp, v in p.items():
        if v is not None:
            a.dword(WK_TAB[vp >> 10] + 8 * (vp & 0x3FF), v)


def _wk_on(a, rr=8, en=1, ptb=WK_PTB):
    a.setr(1, ptb | 0xF8000007); a.mtsr('tlb_ptb', 1)      # (ignored bits set)
    a.setr(1, (rr << 8) | en | 0xFFFF00FE & ~(0x1F << 8)); a.mtsr('tlb_walk', 1)


SENT = {'ecause': 0x55, 'eaddr': 0x66, 'tlb_hi': 0x77770011, 'tlb_lo': 0x0123000F, 'tlb_index': 3}


def _wk_sent(a):
    """sentinels in the SRs a walk must not touch"""
    for sr, v in SENT.items():
        a.setr(1, v); a.mtsr(sr, 1)


def _wk_dump(a, slot0=0, entries=()):
    """WK_OUT + 8*slot: ECAUSE, EADDR, TLB_HI, TLB_LO, TLB_INDEX, TLB_WALK,
    TLB_WCNT, then TLBR of each entry index: TLB_HI, TLB_LO (r1 / r2)"""
    a.setr(2, WK_OUT + 8 * slot0)
    for k, sr in enumerate(('ecause', 'eaddr', 'tlb_hi', 'tlb_lo', 'tlb_index', 'tlb_walk', 'tlb_wcnt')):
        a.mfsr(1, sr); a.st(64, 1, 2, 8 * k)
    for j, i in enumerate(entries):
        a.setr(1, i); a.mtsr('tlb_index', 1); a.tlbr()
        a.mfsr(1, 'tlb_hi'); a.st(64, 1, 2, 56 + 16 * j)
        a.mfsr(1, 'tlb_lo'); a.st(64, 1, 2, 64 + 16 * j)


def _wk_exp(walk, wcnt, entries=(), sent=True, slot0=0):
    """entries: [(hi, lo)] as _wk_dump read them"""
    e = {}
    o = WK_OUT + 8 * slot0
    if sent:
        for k, sr in enumerate(('ecause', 'eaddr', 'tlb_hi', 'tlb_lo', 'tlb_index')):
            e[m(o + 8 * k)] = SENT[sr]
    e[m(o + 40)] = walk
    e[m(o + 48)] = wcnt
    for j, (hi, lo) in enumerate(entries):
        e[m(o + 56 + 16 * j)] = hi
        e[m(o + 64 + 16 * j)] = lo
    return e


def _wk_prog(body, ptes, exp, user=False, logs=(), plus=(), dirs=None, rr=8, en=1, setup=None,
             evec=True):
    """_te_prog with page tables and the walker on (TLB_WALK.RR = rr)"""
    def su(a):
        _wk_tables(a, ptes, dirs)
        _wk_on(a, rr, en)
        if setup:
            setup(a)
    a, e, p = _te_prog(body, user, exp, logs=logs, plus=plus, setup=su)
    return a, e, p


def _wk_ret(va, pfn, xwrv, asid=0):
    """an entry as TLBR reads it after a walk"""
    return ((va >> 16) << 16) | asid, (pfn << 16) | xwrv


@test
def wk_sr():
    """§11.8 SRs: reset 0; writable bits (TLB_PTB [26:3], TLB_WALK [12:8]
    (log2 N bits) and [0], TLB_WCNT [47:0]); user MFSR / MTSR of them is
    PRIVILEGED"""
    if NOEXC:
        return []
    a = Asm()
    _xsetup(a)
    a.setr(12, DATA); a.setr(3, 0)
    for sr in ('tlb_ptb', 'tlb_walk', 'tlb_wcnt'):
        a.mfsr(2, sr); a.alu('or', 3, 3, 2)
    a.st(64, 3, 12, 24); a.setr(3, 0x77); a.st(64, 3, 12, 32)
    a.setr64(1, -1)
    for i, sr in enumerate(('tlb_ptb', 'tlb_walk', 'tlb_wcnt')):
        a.mtsr(sr, 1); a.mfsr(2, sr); a.st(64, 2, 12, 8 * i)
    a.setr(1, 0)
    for sr in ('tlb_ptb', 'tlb_walk', 'tlb_wcnt'):
        a.mtsr(sr, 1)
    _to_user(a, 'u')
    a.label('u')
    for sr in ('tlb_ptb', 'tlb_walk', 'tlb_wcnt'):
        a.mfsr(1, sr); a.mtsr(sr, 1)
    a.halt()
    _uhandler(a)
    exp = {'end': 'park', 'kind': '0', m(DATA + 24): 0, m(DATA + 32): 0x77, m(DATA): 0x07FFFFF8,
           m(DATA + 8): ((TLB_N - 1) << 8) | 1, m(DATA + 16): 0xFFFFFFFFFFFF, m(XD + 32): 7}
    for i in range(7):
        exp[m(XLOG + 8 * i)] = X_PRIV
    return a, exp, []


@test
def wk_load_store():
    """a load and a store to unmapped useg pages are refilled by the walker:
    no exception; ECAUSE / EADDR / TLB_HI / TLB_LO / TLB_INDEX unchanged;
    entry[RR] = {VPN, G 0, ASID, PFN, XWRV}; RR + 1 per walk; WCNT counts.
    Data visible at the frames; the 2nd access to each page hits"""
    if NOEXC:
        return []
    ptes = {0x0200: _wk_pte(3, TLB_V | TLB_R), 0x0471: _wk_pte(4, TLB_V | TLB_R | TLB_W)}
    def body(a):
        _wk_sent(a)
        a.setr(9, 0x02000008); a.ld(64, 4, 9)                 # walk 1 (entry 8)
        a.setr(9, 0x0471FFF0); a.setr(3, 0xBEEF); a.st(64, 3, 9)   # walk 2 (entry 9)
        a.ld(64, 5, 9)
        a.setr(9, 0x02000010); a.ld(64, 6, 9)                 # hits
        _wk_dump(a, 0, (8, 9))
    exp = {'r4': 0x1122334455667788, 'r5': 0xBEEF, 'r6': 0x99, m(XD + 32): 0}
    exp.update(_wk_exp((10 << 8) | 1, 2, [_wk_ret(0x02000000, 3, TLB_V | TLB_R),
                                          _wk_ret(0x04710000, 4, TLB_V | TLB_R | TLB_W)]))
    a, e, p = _wk_prog(body, ptes, exp)
    a.dword(0x30008, 0x1122334455667788); a.dword(0x30010, 0x99)
    return a, e, p


def _wk_snip(a, va, pa_of, emit):
    """assemble emit(sub) at VA va (dword-aligned start below it) and store
    each dword at PA pa_of(dword VA): code that may straddle pages mapped to
    non-adjacent frames"""
    b0 = va & ~7
    sub = Asm(base=b0)
    if va & 4:
        sub.nop()
    emit(sub)
    ws = sub.assemble()
    ws += [0x6C000000] * (len(ws) & 1)
    for i in range(0, len(ws), 2):
        a.dword(pa_of(b0 + 4 * i), ws[i] | (ws[i + 1] << 32))
    return sub


@test
def wk_fetch():
    """fetch misses refilled by the walker: a jump into an unmapped code
    page; an instruction straddling from a mapped page into an unmapped one
    (frames not adjacent); one straddling with both pages unmapped (two
    walks); then the same snippets again (hits)"""
    if NOEXC:
        return []
    ptes = {0x0060: _wk_pte(5, TLB_V | TLB_X), 0x0061: _wk_pte(6, TLB_V | TLB_X),
            0x0062: _wk_pte(9, TLB_V | TLB_X), 0x0071: _wk_pte(0xB, TLB_V | TLB_X),
            0x0072: _wk_pte(0xD, TLB_V | TLB_X)}
    def setup(a):
        _tlb_set(a, 2, 0x0071, 0xB, TLB_V | TLB_X)            # 0x0071 mapped, 0x0072 not
    def body(a):
        _wk_sent(a)
        a.setr(3, 0); a.setr(11, 2)
        a.label('it')
        for k, va in enumerate((0x00600100, 0x0061FFFC, 0x0071FFFC)):
            a.setr(9, va); a.leapc(10, f'r{k}'); a.jmpr(9)
            a.label(f'r{k}')
        a.addi_s(11, 11, -1)
        a.fbr('ne', 11, 0, 'it', imm=True)
        _wk_dump(a, 0, (8, 9, 10, 11))
    exp = {'r3': 2 * (0x1 + 0x12345678 + 0x10 + 0x1234567), m(XD + 32): 0, 'wk_n': '4'}
    exp.update(_wk_exp((12 << 8) | 1, 4, [_wk_ret(0x00600000, 5, TLB_V | TLB_X),
                                          _wk_ret(0x00610000, 6, TLB_V | TLB_X),
                                          _wk_ret(0x00620000, 9, TLB_V | TLB_X),
                                          _wk_ret(0x00720000, 0xD, TLB_V | TLB_X)]))
    a, e, p = _wk_prog(body, ptes, exp, setup=setup)
    fr = {0x0060: 5, 0x0061: 6, 0x0062: 9, 0x0071: 0xB, 0x0072: 0xD}
    pa = lambda va: (fr[va >> 16] << 16) | (va & 0xFFFF)
    _wk_snip(a, 0x00600100, pa, lambda s: (s.addi_s(3, 3, 1), s.jmpr(10)))
    _wk_snip(a, 0x0061FFFC, pa, lambda s: (s.setr(7, 0x12345678), s.alu('add', 3, 3, 7), s.jmpr(10)))
    _wk_snip(a, 0x0071FFFC, pa, lambda s: (s.setr(7, 0x1234567), s.alu('add', 3, 3, 7),
                                           s.addi_s(3, 3, 0x10), s.jmpr(10)))
    return a, e, p


@test
def wk_two_access():
    """a LEAVERET whose 2nd doubleword is in an unmapped page (the 1st
    mapped): one walk, of the 2nd page; a spanning MEMGET32 with both pages
    unmapped: two walks; the values come from non-adjacent frames"""
    if NOEXC:
        return []
    ptes = {0x0102: _wk_pte(0xA, TLB_V | TLB_R), 0x0106: _wk_pte(0xC, TLB_V | TLB_R),
            0x0107: _wk_pte(0xE, TLB_V | TLB_R)}
    def setup(a):
        _tlb_set(a, 2, 0x0101, 0x002, TLB_V | TLB_R | TLB_W)
    def body(a):
        _wk_sent(a)
        a.setr(15, 0x0101FFF8); a.leaveret()                  # R15 <- [p 0x0101 FFF8], PC <- [0x0102_0000]
        a.setr(4, 0xBAD)
        a.label('back')
        a.mov(5, 15); a.getsp(6)
        a.setr(1, KSP); a.setsp(1)
        a.setr(9, 0x0106FFFD); a.ld(32, 7, 9)                 # 3 bytes of 0x0106, 1 of 0x0107
        _wk_dump(a, 0, (8, 9, 10))
    exp = {'r5': 0x1122334455667788, 'r6': 0x01020008, 'r4': 0, 'r7': 0xAB112233, m(XD + 32): 0}
    exp.update(_wk_exp((11 << 8) | 1, 3, [_wk_ret(0x01020000, 0xA, TLB_V | TLB_R),
                                          _wk_ret(0x01060000, 0xC, TLB_V | TLB_R),
                                          _wk_ret(0x01070000, 0xE, TLB_V | TLB_R)]))
    a, e, p = _wk_prog(body, ptes, exp, setup=setup)
    a.dword(0x2FFF8, 0x1122334455667788)
    a.dword(0xA0000, a.labels['back'])
    a.dword(0xCFFF8, 0x1122330000000000)
    a.dword(0xE0000, 0xAB)
    return a, e, p


def _wk_fb(kind):
    """one fallback case: (ptes, dirs, en, va, cause, op) — op 'ld' / 'st'"""
    return {
        'en0':   ({0x0200: _wk_pte(3, TLB_RWX)}, None, 0, 0x02000008, X_TLB_LD, 'ld'),
        'd0':    ({}, {2: WK_JD & ~1 | WK_TAB[0]}, 1, 0x08000008, X_TLB_LD, 'ld'),
        'tpres': ({0x0200: _wk_pte(3, TLB_RWX, present=False)}, None, 1, 0x02000010, X_TLB_ST, 'st'),
        'tv':    ({0x0200: _wk_pte(3, TLB_RWX & ~TLB_V)}, None, 1, 0x02000018, X_TLB_LD, 'ld'),
        # kmap: a D / T that a walker ignoring VA[31] would use
        'kmap':  ({0x0200: _wk_pte(3, TLB_RWX)}, {16: WK_JD | WK_TAB[0] | 1}, 1, 0xC2000008, X_TLB_LD, 'ld'),
    }[kind]


@test
def wk_fallback():
    """each fallback reason (TLB_WALK.EN = 0, D[0] = 0, PTE PRESENT = 0,
    PTE V = 0, a kmap VA) raises today's TLB_MISS_* (ECAUSE, EADDR,
    TLB_HI = {VPN, 0, ASID}) through the handler; RR and WCNT unchanged,
    no entry written"""
    if NOEXC:
        return []
    out = []
    for kind in ('en0', 'd0', 'tpres', 'tv', 'kmap'):
        ptes, dirs, en, va, cause, op = _wk_fb(kind)
        def body(a, va=va, op=op):
            a.setr(1, 0x12345); a.mtsr('tlb_wcnt', 1)
            a.setr(1, 0x33); a.mtsr('asid', 1)
            a.setr(9, va); a.setr(3, 0x77)
            if op == 'ld':
                a.ld(64, 4, 9)
            else:
                a.st(64, 3, 9)
            a.mfsr(5, 'tlb_hi')
            a.setr(1, 0); a.mtsr('asid', 1)
            _wk_dump(a, 0, (8,))
        exp = {'r4': 0, 'r5': (va & 0xFFFF0000) | 0x33, m(0x30000 | (va & 0xFFF8)): 0}
        e = _wk_exp((8 << 8) | en, 0x12345, [(0, 0)], sent=False)
        e[m(WK_OUT)] = cause; e[m(WK_OUT + 8)] = va           # (ECAUSE / EADDR as the handler left them)
        exp.update(e)
        a, ee, p = _wk_prog(body, ptes, exp, logs=[(cause, va)], dirs=dirs, en=en)
        ee['wk_n'] = '0'
        ee['wk_s'] = '0' if kind in ('en0', 'kmap') else '1'
        out.append((a, ee, p))
    return out


@test
def wk_fallback_fetch():
    """a fetch whose PTE is absent falls back to TLB_MISS_FETCH (EADDR = the
    target); a straddle whose 2nd page's PTE is absent: the 1st page is
    walked, the 2nd falls back with EADDR = its start"""
    if NOEXC:
        return []
    ptes = {0x0061: _wk_pte(6, TLB_V | TLB_X), 0x0062: _wk_pte(9, TLB_V | TLB_X, present=False)}
    def body(a):
        a.setr(9, 0x00600100); a.leapc(10, 'r0'); a.jmpr(9)
        a.label('r0')
        a.setr(9, 0x0061FFFC); a.leapc(10, 'r1'); a.jmpr(9)
        a.label('r1')
        a.mfsr(5, 'tlb_hi')
        _wk_dump(a, 0, (8,))
    exp = {'r5': 0x00620000}
    e = _wk_exp((9 << 8) | 1, 1, [_wk_ret(0x00610000, 6, TLB_V | TLB_X)], sent=False)
    exp.update(e)
    a, e, p = _wk_prog(body, ptes, exp, logs=[(X_TLB_FE, 0x00600100), (X_TLB_FE, 0x00620000)])
    e['wk_n'] = '1'; e['wk_s'] = '3'
    _wk_snip(a, 0x0061FFFC, lambda va: 0x60000 | (va & 0xFFFF), lambda s: (s.setr(7, 0x12345678), s.jmpr(10)))
    return a, e, p


@test
def wk_rr_wcnt():
    """RR wraps (N - 1 -> 0, entry N - 1 written); WCNT is 48 bits: MTSR
    sets [47:0], it counts every successful walk and wraps"""
    if NOEXC:
        return []
    ptes = {0x0200: _wk_pte(3, TLB_V | TLB_R), 0x0300: _wk_pte(4, TLB_V | TLB_R)}
    def body(a):
        _wk_sent(a)
        a.setr64(1, 0xABCDFFFFFFFFFFFF); a.mtsr('tlb_wcnt', 1)
        a.setr(9, 0x02000000); a.ld(64, 4, 9)
        a.setr(9, 0x03000000); a.ld(64, 4, 9)
        _wk_dump(a, 0, (TLB_N - 1, 0))
    # walks: 0x0200 -> entry N-1, 0x0300 -> entry 0, which held the code
    # page: it is refetched through a walk -> entry 1 (the data window's),
    # whose page the dump then walks -> entry 2.  WCNT: 2^48 - 1 + 4 wraps.
    exp = {m(XD + 32): 0, 'wk_n': '4'}
    exp.update(_wk_exp((3 << 8) | 1, 3, [_wk_ret(0x02000000, 3, TLB_V | TLB_R),
                                         _wk_ret(0x03000000, 4, TLB_V | TLB_R)]))
    return _wk_prog(body, ptes, exp, rr=TLB_N - 1)


@test
def wk_asid():
    """the refill takes the ASID SR (G = 0): under another ASID the same VA
    walks again; back under the first, its entry still hits"""
    if NOEXC:
        return []
    ptes = {0x0200: _wk_pte(3, TLB_V | TLB_R)}
    def body(a):
        _wk_sent(a)
        for asid in (5, 6, 5):
            a.setr(1, asid); a.mtsr('asid', 1)
            a.setr(9, 0x02000008); a.ld(64, 4, 9)
        a.setr(1, 0); a.mtsr('asid', 1)
        _wk_dump(a, 0, (8, 9))
    exp = {'r4': 0x42, m(XD + 32): 0, 'wk_n': '2'}
    exp.update(_wk_exp((10 << 8) | 1, 2, [_wk_ret(0x02000000, 3, TLB_V | TLB_R, asid=5),
                                          _wk_ret(0x02000000, 3, TLB_V | TLB_R, asid=6)]))
    a, e, p = _wk_prog(body, ptes, exp)
    a.dword(0x30008, 0x42)
    return a, e, p


@test
def wk_uitlb_evict():
    """RR = 0: the first walk overwrites entry 0, the code page the
    micro-ITLB holds.  The walk discards the micro-ITLB (as TLBWI), so the
    next fetch misses and walks the code page (entry 1, evicting the data
    window's), and the dump's first store walks that (entry 2): 3 walks.
    Without the discard the code would run on through the stale micro-ITLB
    copy (2 walks)"""
    if NOEXC:
        return []
    ptes = {0x0200: _wk_pte(3, TLB_V | TLB_R)}
    def body(a):
        _wk_sent(a)
        a.setr(9, 0x02000008); a.ld(64, 4, 9)
        _wk_dump(a, 0, (0, 1, 2))
    exp = {'r4': 0x42, m(XD + 32): 0, 'wk_n': '3'}
    exp.update(_wk_exp((3 << 8) | 1, 3, [_wk_ret(0x02000000, 3, TLB_V | TLB_R),
                                         _wk_ret(0x00000000, 0, TLB_RWX),
                                         _wk_ret(0x00010000, 1, TLB_RWX)]))
    a, e, p = _wk_prog(body, ptes, exp, rr=0)
    a.dword(0x30008, 0x42)
    return a, e, p


@test
def wk_coherent():
    """the walker sees the newest page-table contents: a PTE and a
    directory entry written by stores (through kseg) right before the access,
    still posted / queued when the access misses (wk_pend, from LAT 3 up); a PTE first absent (the
    access falls back: TLB_MISS), then written, and the retried access walks
    (the Linux page-fault path).  Several latencies.  The TB's wk_pend / wk_post /
    wk_sq count the walks that start with a store still in st_post / the store
    queue: they must be non-zero (the case really is the posted one)"""
    if NOEXC:
        return []
    out = []
    for lat in ('+LAT=1', '+LAT=3', '+LAT=7'):
        for hit in ([], ['+HIT=70']):
            def body(a):
                a.setr(8, 0x80000000 | (WK_TAB[0] + 8 * 0x200))
                a.setr64(3, _wk_pte(3, TLB_V | TLB_R)); a.st(64, 3, 8)
                a.setr(9, 0x02000008); a.ld(64, 4, 9)            # PTE store still posted
                a.setr(8, 0x80000000 | (WK_PTB + 8 * 3))         # D[3] (VA 0x0C00_0000+), new table
                a.setr(7, 0x80000000 | (0x26000 + 8 * 0x005))
                a.setr64(3, _wk_pte(4, TLB_V | TLB_R)); a.st(64, 3, 7)
                a.setr64(3, WK_JD | 0x26000 | 1); a.st(64, 3, 8)
                a.setr(6, 0x80000000 | (0x26000 + 8 * 0x006))
                a.setr(9, 0x0C050010); a.st(64, 3, 6)            # (a 3rd store: the queue is in use)
                a.ld(64, 5, 9)                                   # the stores still in flight
                a.setr(9, 0x02100018); a.ld(64, 6, 9)            # absent: TLB_MISS_LD (handler steps over)
                a.setr(8, 0x80000000 | (WK_TAB[0] + 8 * 0x210))
                a.setr64(3, _wk_pte(4, TLB_V | TLB_R)); a.st(64, 3, 8)
                a.ld(64, 6, 9)                                   # walks now
                a.mfsr(10, 'tlb_wcnt')
            exp = {'r4': 0x42, 'r5': 0x4343, 'r6': 0x4444, 'r10': 3, 'wk_n': '3', 'wk_s': '4'}
            if lat != '+LAT=1':      # (at latency 1 the posted stores have drained by the miss)
                exp['wk_pend'] = 'nonzero'
            a, e, p = _wk_prog(body, {}, exp, logs=[(X_TLB_LD, 0x02100018)], plus=[lat] + hit)
            a.dword(0x30008, 0x42); a.dword(0x40010, 0x4343); a.dword(0x40018, 0x4444)
            out.append((a, e, p))
    return out


@test
def wk_coherent_sq():
    """as wk_coherent, in a warm loop (the code is fetched without gaps, so
    the stores really queue): two plain stores, then the PTE store (or the
    directory-entry store: 'dir'), then the access that misses and walks it:
    the store is still on the port or in the store queue (sq_v) when the walk
    starts.  The TB's wk_pend / wk_post / wk_sq count that (all non-zero)"""
    if NOEXC:
        return []
    out = []
    for kind in ('pte', 'dir'):
        for lat in ('+LAT=3', '+LAT=7', '+LAT=12'):
            for hit in ([], ['+HIT=70']):
                def body(a):
                    a.setr(1, 5); a.setr(5, 0)
                    a.setr(7, 0x80000000 | (DATA + 0x100))
                    if kind == 'pte':
                        a.setr(2, 0x10000)
                        a.setr(8, 0x80000000 | 0x26000); a.setr(9, 0x0C000008)
                        a.setr64(3, _wk_pte(3, TLB_V | TLB_R))
                    else:      # D[4 + i] -> table i (PTE 0 present in the image), VA = (4 + i) << 26
                        a.setr(2, 0x2000); a.setr(6, 0x04000000)
                        a.setr(8, 0x80000000 | (WK_PTB + 8 * 4)); a.setr(9, 0x10000008)
                        a.setr64(3, WK_JD | 0x28000 | 1)
                    a.label('lp')
                    a.st(64, 1, 7); a.st(64, 1, 7, 8)
                    a.st(64, 3, 8)                                   # the PTE / directory entry, last
                    a.ld(64, 4, 9)                                   # misses, walks it
                    a.alu('add', 5, 5, 4)
                    a.addi_s(8, 8, 8)
                    if kind == 'pte':
                        a.alu('add', 9, 9, 2)
                    else:
                        a.alu('add', 3, 3, 2); a.alu('add', 9, 9, 6)
                    a.addi_s(1, 1, -1); a.fbr('ne', 1, 0, 'lp', imm=True)
                    a.mfsr(10, 'tlb_wcnt')
                exp = {'r5': 5 * 0x42, 'r10': 5, 'wk_n': '5', 'wk_s': '5', m(XD + 32): 0,
                       'wk_pend': 'nonzero', 'wk_post': 'nonzero', 'wk_sq': 'nonzero'}
                a, e, p = _wk_prog(body, {}, exp, dirs={3: WK_JD | 0x26000 | 1}, plus=[lat] + hit)
                a.dword(0x30008, 0x42)
                if kind == 'dir':
                    for k in range(5):
                        a.dword(0x28000 + 0x2000 * k, _wk_pte(3, TLB_V | TLB_R))
                out.append((a, e, p))
    return out


@test
def wk_user():
    """user-mode load, store and fetch misses are walked (the entries are
    useg, no G); then in supervisor mode a copy_to_user-like store to a
    useg page walks too, and a read-only page's store is ACCESS_STORE after
    its walk (permission faults are not the walker's)"""
    if NOEXC:
        return []
    ptes = {0x0200: _wk_pte(3, TLB_V | TLB_R | TLB_W), 0x0300: _wk_pte(4, TLB_V | TLB_R),
            0x0060: _wk_pte(5, TLB_V | TLB_X), 0x0400: _wk_pte(6, TLB_V | TLB_R | TLB_W)}
    def body(a):
        a.setr(9, 0x02000008); a.ld(64, 4, 9)
        a.setr(3, 0x77); a.st(64, 3, 9, 8)
        a.setr(9, 0x00600100); a.leapc(10, 'r0'); a.jmpr(9)
        a.label('r0')
        a.setr(9, 0x03000000); a.st(64, 3, 9)                    # walk, then ACCESS_STORE
        a.setr(9, 0x02000010); a.ld(64, 6, 9)
    a, e, p = _wk_prog(body, ptes, {'r4': 0x42, 'r5': 0x55, 'r6': 0x77},
                       user=True, logs=[(X_ACC_ST, 0x03000000)])
    a.dword(0x30008, 0x42)
    _wk_snip(a, 0x00600100, lambda va: 0x50000 | (va & 0xFFFF), lambda s: (s.setr(5, 0x55), s.jmpr(10)))
    out = [(a, dict(e, wk_n='3'), p)]
    # supervisor: copy_to_user-like
    def sbody(a):
        a.setr(9, 0x04000020); a.setr(3, 0x99); a.st(64, 3, 9)
        a.setr(9, 0x03000000); a.st(64, 3, 9)
        a.setr(9, 0x04000020); a.ld(64, 6, 9)
    a, e, p = _wk_prog(sbody, ptes, {'r6': 0x99, 'wk_n': '2'}, logs=[(X_ACC_ST, 0x03000000)])
    out.append((a, e, p))
    return out


@test
def wk_irq():
    """interrupts while walking: a loop that TLBINVs and then touches a data
    page and its own code page (two walks per pass), with a timer IRQ whose
    handler (at 0x100000, logging its frame PCs at ELOG) is walked too.  An
    IRQ pending at a walk is taken at the faulting instruction (frame PC =
    it), before or after it re-executes; the sum and the logs are exact"""
    if NOEXC:
        return []
    out = []
    for per, lat in ((140, '+LAT=1'), (161, '+LAT=3'), (297, '+LAT=7'), (153, '+LAT=2')):
        ptes = {0x0200: _wk_pte(3, TLB_V | TLB_R), 0x0010: _wk_pte(0x10, TLB_V | TLB_X)}
        def body(a):
            a.setr(1, 0); a.mtsr('ecause', 1)
            a.set_int_mask(1)
            a.setr(3, 0); a.setr(11, 40)
            a.label('lp')
            a.tlbinv()
            a.setr(9, 0x02000008); a.ld(64, 4, 9); a.alu('add', 3, 3, 4)
            a.addi_s(11, 11, -1)
            a.fbr('ne', 11, 0, 'lp', imm=True)
            a.set_int_mask(0)
            a.mfsr(10, 'tlb_wcnt'); a.mfsr(6, 'ecause')
        a, e, p = _wk_prog(body, ptes, {'r3': 40 * 5, 'r6': 0, m(XD + 32): 0, 'r10': 'nonzero'},
                           plus=['+NO_IRQ_HANDLER', f'+IRQ_PERIOD={per}', lat])
        a.dword(0x30008, 5)
        _irq_logger(a)
        e[m(ELOG)] = 'nonzero'
        e['wk_irq'] = 'nonzero'          # (IRQs taken at a walk's restart boundary)
        out.append((a, e, p))
    return out


@test
def wk_evec0():
    """EVEC = 0: a walk that succeeds runs on (no park); one that falls back
    parks as today (kind 2 at the instruction, ECAUSE / EADDR / TLB_HI not
    written); with TLB_WALK.EN = 0 the same park"""
    if NOEXC:
        return []
    out = []
    for en in (1, 0):
        a = Asm()
        _wk_tables(a, {0x0200: _wk_pte(3, TLB_V | TLB_R)})
        _te_on(a)
        _wk_on(a, 8, en)
        a.setr(1, 0x55); a.mtsr('ecause', 1)
        a.setr(9, 0x02000008); a.ld(64, 4, 9)
        a.setr(9, 0x02100008)
        at = a.ld(64, 5, 9)
        a.halt()
        a.dword(0x30008, 0x42)
        e = {'end': 'park', 'kind': '2', 'ecause': '055', 'eaddr': '00000000', 'wk_n': str(en)}
        if en:
            e.update({'pc': at, 'op': a.words[(at - a.base) // 4], 'r4': 0x42})
        else:
            e['pc'] = a.insn_starts[a.insn_starts.index(at) - 2]   # the first load
        out.append((a, e, []))
    return out


@test
def wk_debug():
    """DEBUG_EN builds (else the halt log stays empty: dh_n = 0): with a
    debugger attached and EVEC = 0 a successful walk does not halt, a
    fallback debug-halts as today's park would (cause ILL, at the
    instruction; one halt).  A PC breakpoint on a walked load: the first
    resume skips the breakpoint for one dispatch, the walk restart refetches
    the same pc and matches again, so there are two BP halts at that pc and
    the second plain resume executes the load (MMU_PLAN §11.8).  Single
    steps from the breakpoint: the first step re-halts at the load (cause
    STEP: the walk counted as the step), the next executes it, then one
    instruction per step; all give the plain run's result.  Each halt's pc /
    cause is checked (TB: dh_n, dh<k> = pc:cause); debug_en proves the
    elaboration"""
    if NOEXC:
        return []
    dbg = _os.environ.get('VSIM_DEBUG_EN') == '1'
    out = []
    a = Asm()
    _wk_tables(a, {0x0200: _wk_pte(3, TLB_V | TLB_R)})
    _te_on(a)
    _wk_on(a, 8, 1)
    a.setr(9, 0x02000008); a.ld(64, 4, 9)
    a.setr(9, 0x02100008)
    at = a.ld(64, 5, 9)
    a.halt()
    a.dword(0x30008, 0x42)
    e = {'end': 'park', 'pc': at, 'r4': 0x42, 'wk_n': '1', 'debug_en': '1' if dbg else '0'}
    e.update({'kind': '4', 'dbg_cause': '5', 'dh_n': '1', 'dh0': f'{at:08x}:5'} if dbg
             else {'kind': '2', 'dh_n': '0'})
    out.append((a, e, ['+DBG_ATTACHED', '+PARK_HOLD=4']))    # (hold: the halt is logged a cycle after)

    def prog():
        a = Asm()
        _xsetup(a)
        _wk_tables(a, {0x0200: _wk_pte(3, TLB_V | TLB_R)})
        _te_on(a)
        _wk_on(a, 8, 1)
        a.setr(9, 0x02000008)
        a.label('b')
        a.ld(64, 4, 9); a.addi_s(4, 4, 1); a.setr(10, 0x55)
        a.halt()
        _mpuhandler(a)
        a.dword(0x30008, 0x42)
        return a
    # plain resumes with the breakpoint still set (D1): two halts at the load
    for lat in (1, 3, 7):
        a = prog()
        b = a.labels['b']
        e = {'end': 'park', 'kind': '0', 'r4': 0x43, 'r10': 0x55, 'wk_n': '1', m(XD + 32): 0,
             'debug_en': '1' if dbg else '0'}
        e.update({'dh_n': '2', 'dh0': f'{b:08x}:2', 'dh1': f'{b:08x}:2'} if dbg else {'dh_n': '0'})
        out.append((a, e, ['+DBG_BP0=%x' % b, '+DBG_PLAIN=2', f'+LAT={lat}']))
    for steps in (1, 2, 3, 4, 5):
        a = prog()
        b = a.labels['b']
        ix = a.insn_starts.index(b)
        seq = a.insn_starts[ix:]           # ld, addi, setr, halt, ...
        e = {'end': 'park', 'kind': '0', 'r4': 0x43, 'r10': 0x55, 'wk_n': '1', m(XD + 32): 0,
             'debug_en': '1' if dbg else '0'}
        if dbg:
            # halt 0: the breakpoint (cause 2) at the load; step 1: STEP at
            # the load again (the walk); step k >= 2: STEP at the (k-1)-th
            # instruction after it; stepping onto the HALT (step 5) executes it
            hl = [(b, 2), (b, 1)] + [(seq[k - 1], 1) for k in range(2, steps + 1) if k - 1 < len(seq) - 0 and k <= 4]
            e['dh_n'] = str(len(hl))
            for k, (pc, c) in enumerate(hl):
                e[f'dh{k}'] = f'{pc:08x}:{c}'
        else:
            e['dh_n'] = '0'
        out.append((a, e, ['+DBG_BP0=%x' % b, f'+DBG_STEPS={steps}', '+LAT=3']))
    return out


# ---- MMU_PLAN §11.8: double fault / TLBVEC / stack ops / D and T together /
# ---- the TLB_EN = 1, TLB_WALK = 0 build

@test
def wk_df_fallback():
    """a walk that falls back (PTE not PRESENT, or D[0] = 0) enters the
    exception with a kernel SP whose frame push (SP - 8) is not in kseg:
    DOUBLE_FAULT (park kind 6 at the instruction, ECAUSE / EADDR not
    written), exactly as without the walker; the walk was tried (wk_s) and
    did not succeed (wk_n = 0).  Load and store"""
    if NOEXC:
        return []
    out = []
    for why in ('absent', 'dir0'):
        for op in ('ld', 'st'):
            for ksp in (0x08000000, 0x80000000):
                a = Asm()
                _xsetup(a)
                if why == 'absent':
                    _wk_tables(a, {0x0200: _wk_pte(3, TLB_RWX, present=False)})
                else:
                    _wk_tables(a, {0x0200: _wk_pte(3, TLB_RWX)}, dirs={0: 0})
                _wk_on(a, 8, 1)
                a.setr(1, 0x55); a.mtsr('ecause', 1)
                a.setr(1, 0x66); a.mtsr('eaddr', 1)
                _te_on(a, ksp)
                a.setr(9, 0x02000008); a.setr(3, 0x77)
                at = a.ld(64, 4, 9) if op == 'ld' else a.st(64, 3, 9)
                a.setr(5, 0x99)
                a.halt()
                _mpuhandler(a)
                e = {'end': 'park', 'kind': '6', 'pc': at, 'ecause': '055', 'eaddr': '00000066',
                     'wk_s': '1', 'wk_n': '0', 'r4': 0, 'r5': 0, m(XD + 32): 0}
                out.append((a, e, []))
    return out


@test
def wk_df_ok():
    """a successful walk takes no frame push, so a kernel SP that would
    double-fault on an exception entry (not in kseg) does not: the program
    runs on (load and store), no handler entry, kind 0.  Control: with the
    SP fine and the PTE absent the same access takes TLB_MISS normally"""
    if NOEXC:
        return []
    out = []
    for ksp in (0x08000000, 0x80000000, 0x88000000):
        a = Asm()
        _xsetup(a)
        _wk_tables(a, {0x0200: _wk_pte(3, TLB_V | TLB_R | TLB_W), 0x0300: _wk_pte(4, TLB_V | TLB_R)})
        _wk_on(a, 8, 1)
        _te_on(a, ksp)
        a.setr(9, 0x02000008); a.ld(64, 4, 9)
        a.setr(9, 0x03000010); a.ld(64, 5, 9)
        a.setr(9, 0x02000020); a.setr(3, 0xBEEF); a.st(64, 3, 9)
        a.ld(64, 6, 9)
        a.mfsr(10, 'tlb_wcnt')
        a.halt()
        _mpuhandler(a)
        a.dword(0x30008, 0x42); a.dword(0x40010, 0x43)
        e = {'end': 'park', 'kind': '0', 'r4': 0x42, 'r5': 0x43, 'r6': 0xBEEF, 'r10': 2,
             'wk_n': '2', 'wk_s': '2', m(XD + 32): 0, 'sp': ksp}
        out.append((a, e, []))
    return out


@test
def wk_tlbvec():
    """a fallback with TLBVEC != 0 enters at TLBVEC (load, store and a kmap
    miss: the walker never runs for it), not at EVEC; walks that succeed
    do not enter either; ECAUSE / EADDR / TLB_HI as for the software path"""
    if NOEXC:
        return []
    ptes = {0x0200: _wk_pte(3, TLB_V | TLB_R), 0x0210: _wk_pte(4, TLB_V | TLB_R | TLB_W),
            0x0220: _wk_pte(5, TLB_V | TLB_R, present=False),
            0x0230: _wk_pte(6, TLB_V | TLB_R, present=False)}
    def body(a):
        a.setr(9, 0x02000008); a.ld(64, 4, 9)               # walks
        a.setr(9, 0x02200018); a.ld(64, 6, 9)               # absent: TLB_MISS_LD via TLBVEC
        a.mfsr(11, 'tlb_hi')
        a.setr(9, 0x02300020); a.setr(3, 0x1234); a.st(64, 3, 9)    # absent: TLB_MISS_ST via TLBVEC
        a.setr(9, 0x02100010); a.ld(64, 5, 9)               # walks
        a.setr(9, 0xC2000008); a.ld(64, 7, 9)               # kmap: TLB_MISS_LD via TLBVEC (no walk)
        a.mfsr(10, 'tlb_wcnt')
    exp = {'r4': 0x42, 'r5': 0x43, 'r6': 0, 'r7': 0, 'r10': 2, 'r11': 0x02200000,
           m(XD + 0x58): 3, 'wk_n': '2', 'wk_s': '4'}
    a, e, p = _te_prog(body, False, exp,
                       logs=[(X_TLB_LD, 0x02200018), (X_TLB_ST, 0x02300020), (X_TLB_LD, 0xC2000008)],
                       tlbvec=True, setup=lambda a: (_wk_tables(a, ptes), _wk_on(a, 8, 1)))
    a.dword(0x30008, 0x42); a.dword(0x40010, 0x43)
    return a, e, p


@test
def wk_stack_ops():
    """stack instructions whose stack page is missing and walked (the SP in
    useg, supervisor): PUSH, POP, CALL (the return address push), RET (the
    pop), ENTER (the R15 push) and LEAVE (the R15 pop), each on its own page
    (6 walks); every one completes with the SP / R15 / memory effect of a
    plain run"""
    if NOEXC:
        return []
    PG = lambda k: 0x0100 + k                       # VA page of op k
    FR = lambda k: 0x08 + k                         # its frame (PA 0x80000..)
    ptes = {PG(k): _wk_pte(FR(k), TLB_V | TLB_R | TLB_W) for k in range(6)}
    va = lambda k, off: (PG(k) << 16) | off
    pa = lambda k, off: (FR(k) << 16) | off
    kseg = lambda p_: 0x80000000 | p_
    def body(a):
        a.getsp(11)
        a.setr(10, 0)
        a.setr64(5, 0x1111222233334444)
        a.setr(1, va(0, 0x1000)); a.setsp(1); a.push(5)                  # PUSH: store at va(0, FF8)
        a.setsp(11)
        a.setr(8, kseg(pa(0, 0xFF8))); a.ld(64, 6, 8)                    # r6 = what PUSH stored
        a.setr(1, va(1, 0xFF8)); a.setsp(1); a.pop(7)                    # POP: load from va(1, FF8)
        a.getsp(2); a.setsp(11)                                          # r2 = SP after POP
        a.setr(1, va(2, 0x1000)); a.setsp(1)
        a.call('f')                                                      # CALL: the return address push
        a.label('cret')
        a.setsp(11)
        a.setr(8, kseg(pa(2, 0xFF8))); a.ld(64, 9, 8)                    # r9 = the pushed return address
        a.setr(1, va(3, 0xFF8)); a.setsp(1)
        a.ret()                                                          # RET: pops 'rt' from va(3, FF8)
        a.setr(10, 0xBAD)
        a.label('rt')
        a.setsp(11)
        a.setr(15, 0xF15); a.setr(1, va(4, 0x1000)); a.setsp(1)
        a.enter(2)                                                       # ENTER: push R15 at va(4, FF8)
        a.getsp(3); a.alui('add', 14, 15, 0)                             # r3 = SP, r14 = R15 after ENTER
        a.setsp(11)
        a.setr(8, kseg(pa(4, 0xFF8))); a.ld(64, 4, 8)                    # r4 = the pushed R15
        a.setr(15, va(5, 0xFF0)); a.setsp(11)
        a.leave()                                                        # LEAVE: SP = R15, pop R15 from va(5, FF0)
        a.getsp(1); a.setsp(11)                                          # r1 = SP after LEAVE
        a.mfsr(13, 'tlb_wcnt')
        a.jmp('done')
        a.label('f')
        a.ret()                                                          # (hits)
        a.label('done')
    exp = {'r6': 0x1111222233334444, 'r7': 0x7007, 'r2': va(1, 0x1000), 'r10': 0, 'r4': 0xF15,
           'r3': va(4, 0x1000) - 8 - 16, 'r14': va(4, 0x1000) - 8,
           'r15': 0xCAFE, 'r1': va(5, 0xFF8), 'r13': 6, 'wk_n': '6', 'wk_s': '6',
           m(XD + 32): 0}
    a, e, p = _wk_prog(body, ptes, exp)
    e['r9'] = a.labels['cret']
    a.dword(pa(1, 0xFF8), 0x7007)
    a.dword(pa(3, 0xFF8), a.labels['rt'])
    a.dword(pa(5, 0xFF0), 0xCAFE)
    return a, e, p


@test
def wk_dt_same():
    """a layout whose D read and T read are the same doubleword: the
    directory is the page table (PTB = table + 8 * j, D index 0, VA[25:16]
    = j, so D[0] and PTE j are one word).  PFN = D's table bits, so the
    word is also a valid PTE.  Variants: load; store + readback (the second
    layout uses D[1]); several latencies, with the hit-pipe model.  The TB's
    wk_same counts walks whose T read hit D's doubleword"""
    if NOEXC:
        return []
    out = []
    JT = 0x5A5A0000F8000000                      # junk both reads ignore (bits [63:27])
    for lat in ('+LAT=1', '+LAT=3', '+LAT=7'):
        for hit in ([], ['+HIT=70']):
            for layout in (0, 1):
                if layout == 0:                  # D[0] at TAB + 8 * 5 = PTB; VA 0x0005_0010
                    tab, ptb, va, flags = 0x24000, 0x24028, 0x00050010, 0x13
                else:                            # D[1] at TAB + 8 * 1; VA 0x0401_0010 (store)
                    tab, ptb, va, flags = 0x26000, 0x26000, 0x04010010, 0x17
                word = JT | tab | flags          # PFN = tab[26:16] = 2: PA 0x20000
                def body(a, layout=layout, ptb=ptb, va=va):
                    a.setr(9, va)
                    if layout == 0:
                        a.ld(64, 4, 9)
                        a.ld(64, 5, 9, 8)
                    else:
                        a.setr(3, 0xBEEF); a.st(64, 3, 9)
                        a.ld(64, 4, 9)
                        a.setr(8, 0x80020010); a.ld(64, 5, 8)
                    a.mfsr(10, 'tlb_wcnt')
                def setup(a, layout=layout, tab=tab, ptb=ptb, word=word):
                    if layout == 0:
                        a.dword(ptb, word)       # = PTE 5 of the table at tab
                    else:
                        a.dword(ptb + 8, word)   # D[1] = PTE 1 of the table at tab
                    _wk_on(a, 8, 1, ptb=ptb)
                if layout == 0:
                    exp = {'r4': 0x1357, 'r5': 0x2468, 'r10': 1, 'wk_n': '1', 'wk_s': '1',
                           'wk_same': '1', m(XD + 32): 0}
                else:
                    exp = {'r4': 0xBEEF, 'r5': 0xBEEF, 'r10': 1, 'wk_n': '1', 'wk_s': '1',
                           'wk_same': '1', m(XD + 32): 0}
                a, e, p = _te_prog(body, False, exp, plus=[lat] + hit, setup=setup)
                if layout == 0:
                    a.dword(0x20010, 0x1357); a.dword(0x20018, 0x2468)
                out.append((a, e, p))
    return out


@test
def wk_nowalk():
    """(TLB_EN = 1, TLB_WALK = 0 builds only) MFSR / MTSR of 0x35-0x37 are
    ILLEGAL (the handler steps over each), TLBVEC (0x34) is not; the
    software TLB path works as before: a miss takes TLB_MISS_LD, and the
    TB's walker counters stay 0.  (CPU_FEATURES[25] = 0: run.py socwalk)"""
    if NOEXC or not NOWALK:
        return []
    a = Asm()
    _xsetup(a)
    a.setr(1, 0x1234)
    for sr in ('tlb_ptb', 'tlb_walk', 'tlb_wcnt'):
        a.mfsr(1, sr); a.mtsr(sr, 1)
    a.mfsr(2, 'tlbvec'); a.mtsr('tlbvec', 1)
    a.setr(5, 0x77)
    a.halt()
    _xhandler(a)
    exp = {'end': 'park', 'kind': '0', 'r1': 0x1234, 'r5': 0x77, m(XD + 32): 6, m(XD): X_ILL,
           'wk_n': '0', 'wk_s': '0'}
    out = [(a, exp, [])]
    def body(b):
        b.setr(9, 0x02000008); b.ld(64, 4, 9)
    b, e, p = _te_prog(body, False, {'r4': 0, 'wk_n': '0', 'wk_s': '0', 'wk_rd': '0'},
                       logs=[(X_TLB_LD, 0x02000008)])
    out.append((b, e, p))
    return out



# =================== MMU_PLAN §11.9: the data TLB lookup in the first EX cycle
# The data CAM matches the page of the EA's two operands (base + offset,
# registered at dispatch) carry-free, with no adder in front of it.  These
# tests put EAs on both sides of page boundaries through bases in other
# pages (forward / backward, several pages away, wrapping 2^32), in every
# addressing mode and size, with the operands produced right before the
# access (forwarded from MEM), one instruction earlier (WB) or long before.
TX_MAP = {0x0100: 0x005, 0x0101: 0x003, 0x0102: 0x002}   # VA page -> frame (not in VA order)
TX_RO = (0x0103, 0x006)                                   # a read-only page; 0x0104 is unmapped
TX_WIN = ((0x0100FF00, 0x01010100), (0x0101FF00, 0x01020100), (0x0102FF00, 0x01030000))
TX_B = (0x01010000, 0x01020000)                           # the boundaries inside the windows
M64 = 2 ** 64 - 1


def _tx_pa(va):
    vp = va >> 16
    f = TX_MAP[vp] if vp in TX_MAP else TX_RO[1]
    return (f << 16) | (va & 0xFFFF)


def _tx_pat(pa):
    return (pa * 0x9E3779B97F4A7C15 + 0x0123456789ABCDEF) & M64


def _tx_setup(a):
    for i, (vp, f) in enumerate(sorted(TX_MAP.items())):
        _tlb_set(a, 2 + i, vp, f, TLB_V | TLB_R | TLB_W)
    _tlb_set(a, 5, TX_RO[0], TX_RO[1], TLB_V | TLB_R)


def _tx_init():
    """the byte model (by PA) of the preloaded windows (and TX_RO's first
    0x100 bytes)"""
    mem = {}
    for lo, hi in list(TX_WIN) + [(TX_RO[0] << 16, (TX_RO[0] << 16) + 0x100)]:
        for va in range(lo, hi, 8):
            pa = _tx_pa(va)
            v = _tx_pat(pa)
            for i in range(8):
                mem[pa + i] = (v >> (8 * i)) & 0xFF
    return mem


def _tx_preload(a):
    for pa in sorted(_tx_init()):
        if pa % 8 == 0:
            a.dword(pa, _tx_pat(pa))


def _rol(v, n=1):
    return ((v << n) | (v >> (64 - n))) & M64


def _tx_rd(mem, va, n):
    return sum(mem[_tx_pa(va + i)] << (8 * i) for i in range(n))


def _tx_battery(seed, user, plus, soc=False):
    """random loads / stores (every mode and size; MEMGET32 at any byte
    alignment) at EAs near the page boundaries, through bases anywhere;
    runs of back-to-back accesses.  r5 folds every loaded value, r6 a final
    sweep of the windows (through kseg in supervisor mode, so each store's
    PA is checked)"""
    import random
    rnd = random.Random(seed * 1009 + user)
    mem = _tx_init()
    loads = []

    def pick_ea(n, una):
        lo, hi = rnd.choice(TX_WIN)
        bs = [b for b in TX_B if lo <= b < hi]
        if bs and rnd.random() < 0.6:
            ea = bs[0] + rnd.randrange(-24, 24)
        else:
            ea = rnd.randrange(lo, hi)
        ea = max(lo, min(hi - n, ea))
        return ea if una else ea - ea % n

    def operands(ea, n):
        """(mode, base, off): EA = base + off (mod 2^32)"""
        k = rnd.random()
        if k < 0.12:
            return 'ind', ea, 0
        if k < 0.22:
            return 'abs', 0, ea
        if k < 0.45:
            off = rnd.randrange(-128, 128) * n
            return 'short', (ea - off) & 0xFFFFFFFF, off
        c = rnd.random()
        if c < 0.3:
            off = rnd.randrange(-0x300, 0x300)
        elif c < 0.6:
            off = rnd.choice([-1, 1]) * (rnd.randrange(1, 4) << 16) + rnd.randrange(-0x40, 0x40)
        elif c < 0.8:
            off = rnd.getrandbits(32)
        else:                           # the base across 2^31 / 2^32
            off = rnd.choice([0x80000000, 0x7FFF0000, 0xFFFF0000, 0x00010000]) + rnd.randrange(-8, 8)
        off &= 0xFFFFFFFF
        return ('rr' if rnd.random() < 0.5 else 'long'), (ea - off) & 0xFFFFFFFF, off

    def body(a):
        a.setr(5, 0); a.setr(6, 0)
        for _ in range(100):
            pend = []
            for j in range(rnd.choice([1, 1, 2, 3, 4])):          # back-to-back runs
                st = rnd.random() < 0.35
                size = rnd.choice([8, 16, 32, 64])
                n = size // 8
                memget = not st and size == 32 and rnd.random() < 0.3
                ea = pick_ea(n, memget)
                mode, base, off = ('ind', ea, 0) if memget else operands(ea, n)
                if mode == 'ind' and size == 32 and not memget:
                    mode = 'short'                                  # ([rs1] 32-bit load = MEMGET32)
                rb, ro = rnd.choice([(9, 11), (10, 13)])
                setv = []
                if mode != 'abs':
                    setv.append(lambda rb=rb, base=base: a.setr(rb, base))
                if mode == 'rr':
                    setv.append(lambda ro=ro, off=off: a.setr(ro, off))
                rnd.shuffle(setv)
                for f in setv:
                    f()
                for _ in range(rnd.choice([0, 0, 1, 2])):          # MEM / WB / rf operands
                    a.nop()
                so = off - (1 << 32) if off >= 1 << 31 else off
                if st:
                    v = rnd.getrandbits(64) & ((1 << size) - 1)
                    a.setr64(14, v)
                    if mode == 'ind':     a.st(size, 14, rb)
                    elif mode == 'abs':   a.st_abs(size, 14, ea)
                    elif mode == 'short': a.st(size, 14, rb, so)
                    elif mode == 'long':  a.st(size, 14, rb, off, mode='long')
                    else:                 a.st_rr(size, 14, rb, ro)
                    for i in range(n):
                        mem[_tx_pa(ea + i)] = (v >> (8 * i)) & 0xFF
                else:
                    rd = [1, 2, 3, 4][j]
                    if mode == 'ind':     a.ld(size, rd, rb)
                    elif mode == 'abs':   a.ld_abs(size, rd, ea)
                    elif mode == 'short': a.ld(size, rd, rb, so)
                    elif mode == 'long':  a.ld(size, rd, rb, off, mode='long')
                    else:                 a.ld_rr(size, rd, rb, ro)
                    pend.append(rd)
                    loads.append(_tx_rd(mem, ea, n))
            for rd in pend:
                a.shift_n('rol', 5, 5, 1); a.alu('xor', 5, 5, rd)
        for lo, hi in TX_WIN:                                      # the sweep
            for va in range(lo, hi, 8):
                a.ld_abs(64, 7, va if user else 0x80000000 + _tx_pa(va))
                a.shift_n('rol', 6, 6, 1); a.alu('xor', 6, 6, 7)

    def setup(a):
        if soc:     # (tb_soc's boot image holds 256 KiB: store the windows, TE = 0)
            for pa in sorted(_tx_init()):
                if pa % 8 == 0:
                    a.setr(9, pa); a.setr64(3, _tx_pat(pa)); a.st(64, 3, 9)
        _tx_setup(a)
    a, e, p = _te_prog(body, user, None, setup=setup, plus=plus)
    if not soc:
        _tx_preload(a)
    acc = 0
    for v in loads:
        acc = _rol(acc) ^ v
    sw = 0
    for lo, hi in TX_WIN:
        for va in range(lo, hi, 8):
            sw = _rol(sw) ^ _tx_rd(mem, va, 8)
    e['r5'] = acc
    e['r6'] = sw
    return a, e, p


@test
def tx_battery():
    """TE = 1: random loads / stores at page boundaries through bases in
    other pages, every mode and size, MEMGET32 at any alignment, operands
    from MEM / WB / the register file, back-to-back runs; supervisor and
    user mode, memory latency 1 / 3 / 7, the hit-pipe model"""
    if NOEXC:
        return []
    return [_tx_battery(seed, user, plus)
            for seed in range(3) for user in (False, True)
            for plus in ([], ['+LAT=1'], ['+LAT=7'], ['+HIT=70'], ['+HIT=100', '+LAT=2'])]


@test
def tx_stack():
    """TE = 1, stack ops whose SP / R15 base is in the page above the access:
    PUSH / POP, CALL / RET, ENTER / LEAVE with SP at a page start (the access
    at SP - 8 is in the page below), SP from a SETSP right before (MEM
    forward); LEAVERET crossing (R15 at a page's last dword) and not, with
    R15 from an ALU op right before, from a load, and long before; MEMGET32
    at every byte offset of a page's last dword, rs1 from MEM / a load"""
    if NOEXC:
        return []
    B2 = TX_B[1]
    out = []
    for plus in ([], ['+LAT=1'], ['+HIT=70'], ['+HIT=100', '+LAT=5']):
        mem = _tx_init()
        exp = {}
        def body(a):
            a.jmp('tx_go')
            _tx_f(a)
            a.label('tx_go')
            # MEMGET32 at each offset of P1's last dword (5..7 span into P2)
            a.setr(6, 0)
            for o in range(8):
                ea = 0x0101FFF8 + o
                if o % 2:
                    a.setr(13, ea); a.ld(32, 7, 13)
                else:
                    a.setr(1, ea); a.st(64, 1, 12, 0x40); a.ld(64, 13, 12, 0x40); a.ld(32, 7, 13)
                a.shift_n('rol', 6, 6, 1); a.alu('xor', 6, 6, 7)
            a.setr(1, B2); a.setsp(1); a.setr64(2, 0x1111222233334444); a.push(2); a.pop(3)
            a.setr(1, B2); a.setsp(1); a.setr64(2, 0x5555666677778888); a.push(2)
            a.nop(); a.nop(); a.nop(); a.pop(4); a.getsp(5)
            a.setr(1, B2); a.setsp(1); a.call('tx_f')
            a.label('tx_after')
            a.getsp(8)
            a.setr(15, 0xABCD); a.setr(1, B2); a.setsp(1); a.enter(1)
            a.getsp(9); a.mov(10, 15); a.ld(64, 11, 15); a.leave(); a.getsp(1); a.mov(2, 15)
            a.alu('xor', 9, 9, 1); a.alu('xor', 10, 10, 2)      # (B2 - 16) ^ B2, (B2 - 8) ^ 0xABCD
            a.setr(1, KSP); a.setsp(1)
            # LEAVERET: [R15] -> R15, [R15 + 8] -> PC; four bases
            for i, (r15, how) in enumerate(((0x0101FFF8, 'alu'), (0x0101FFF8, 'load'),
                                            (0x0101FFF8, 'rf'), (0x0101FFE0, 'alu'))):
                a.setr(1, 0x80000000 + _tx_pa(r15)); a.setr(2, 0x7000 + i); a.st(64, 2, 1)
                a.setr(1, 0x80000000 + _tx_pa(r15 + 8)); a.leapc(2, f'tx_lr{i}'); a.st(64, 2, 1)
                if how == 'alu':
                    a.setr(15, r15 - 8); a.addi_s(15, 15, 8)
                elif how == 'load':
                    a.setr(1, r15); a.st(64, 1, 12, 0x40); a.nop(); a.nop(); a.ld(64, 15, 12, 0x40)
                else:
                    a.setr(15, r15); a.nop(); a.nop(); a.nop()
                a.leaveret()
                a.setr(15, 0xBAD)
                a.label(f'tx_lr{i}')
                a.getsp(1); a.alu('xor', 1, 1, 15); a.st(64, 1, 12, 0x48 + 8 * i)   # (r15 + 16) ^ 0x700i
                a.setr(1, KSP); a.setsp(1)
                exp[m(GOOD + 0x48 + 8 * i)] = (r15 + 16) ^ (0x7000 + i)
        a, e, p = _te_prog(body, False, None, setup=_tx_setup, plus=plus)
        _tx_preload(a)
        f = a.labels
        e.update(exp)
        e.update({'r3': 0x1111222233334444, 'r4': 0x5555666677778888, 'r5': B2,
                  'r8': B2, 'r9': (B2 - 16) ^ B2, 'r10': (B2 - 8) ^ 0xABCD, 'r11': 0xABCD})
        mg = 0
        for o in range(8):
            mg = _rol(mg) ^ _tx_rd(mem, 0x0101FFF8 + o, 4)
        e['r6'] = mg
        # the CALL's return-address slot (P1's last dword) and tx_f's view of it
        e[m(GOOD + 0x30)] = f['tx_after']
        e[m(GOOD + 0x38)] = B2 - 8
        out.append((a, e, p))
    return out


def _tx_f(a):
    a.label('tx_f')
    a.getsp(1); a.ld(64, 2, 1); a.st(64, 2, 12, 0x30); a.st(64, 1, 12, 0x38)
    a.ret()


@test
def tx_faults():
    """TE = 1: faults of accesses whose base and EA pages differ (1-word
    forms, the handler steps over them): a miss in the EA's page with the
    base's mapped, a read-only EA page (store), an unmapped base page with a
    mapped EA page (no fault), kseg base + offset into useg (translated) /
    into the hole, useg base into kseg (user: ACCESS) and MMIO (user:
    ACCESS); LEAVERET / MEMGET32 whose 2nd page is unmapped (its start) or
    1st (the EA); operands from MEM / WB / the register file"""
    if NOEXC:
        return []
    out = []
    for user, plus, dist in [(u, pl, d) for u in (False, True) for pl in ([], ['+HIT=70', '+LAT=1'])
                             for d in (0, 1, 3)]:
            mem = _tx_init()
            logs = []
            vals = []
            def body(a):
                k = [0]
                def fault(cause, ea):
                    logs.append((cause, ea))
                def rr(op, b, o, dist):
                    if op == 'st':
                        a.setr64(7, 0x77)
                    a.setr(9, b); a.setr(11, o & 0xFFFFFFFF)
                    for _ in range(dist):
                        a.nop()
                    if op == 'ld':
                        a.ld_rr(64, 7, 9, 11)
                    else:
                        a.st_rr(64, 7, 9, 11)
                a.setr(5, 0)
                if True:
                    rr('ld', 0x0103FFF8, 8, dist);  fault(X_TLB_LD, 0x01040000)    # base RO, EA unmapped
                    rr('st', 0x0102FFF8, 8, dist);  fault(X_ACC_ST, 0x01030000)    # EA read-only
                    rr('ld', 0x0104FFF8, -0x20000, dist)                           # base unmapped, EA in P2
                    a.shift_n('rol', 5, 5, 1); a.alu('xor', 5, 5, 7); vals.append(_tx_rd(mem, 0x0102FFF8, 8))
                    a.setr(9, 0x01040008); a.ld(64, 7, 9, -8); fault(X_TLB_LD, 0x01040000)   # short form
                    a.setr(9, 0x0102FFF8); a.setr(7, 0x99); a.st(64, 7, 9, 8); fault(X_ACC_ST, 0x01030000)
                    rr('ld', 0x80000000, 0x8102FFF8, dist)                         # kseg base, EA in P2
                    a.shift_n('rol', 5, 5, 1); a.alu('xor', 5, 5, 7); vals.append(_tx_rd(mem, 0x0102FFF8, 8))
                    if user:
                        rr('ld', 0x7FFFFFF8, 0x10, dist); fault(X_ACC_LD, 0x80000008)   # useg -> kseg
                        rr('ld', 0xEFFFFFF8, 8, dist);    fault(X_ACC_LD, 0xF0000000)   # -> MMIO
                    else:
                        rr('ld', 0x80000000, -8, dist);   fault(X_TLB_LD, 0x7FFFFFF8)   # kseg -> useg
                        rr('ld', 0x87FFFFF8, 8, dist);    fault(X_ACC_LD, 0x88000000)   # -> the hole
                        rr('ld', 0x01030000, 0x8002FF00 - 0x01030000, dist)             # useg -> kseg (PA 0x2FF00)
                        a.shift_n('rol', 5, 5, 1); a.alu('xor', 5, 5, 7); vals.append(_tx_pat(0x2FF00))
                    # two-access ops: the 2nd page unmapped / the 1st
                    a.setr(15, 0x0103FFF0); a.addi_s(15, 15, 8)
                    for _ in range(dist):
                        a.nop()
                    a.leaveret(); fault(X_TLB_LD, 0x01040000)
                    a.setr(9, 0x0103FFFD)
                    for _ in range(dist):
                        a.nop()
                    a.ld(32, 7, 9); fault(X_TLB_LD, 0x01040000)
                    a.setr(9, 0x0104FFFE); a.ld(32, 7, 9); fault(X_TLB_LD, 0x0104FFFE)
            a, e, p = _te_prog(body, user, None, logs=logs, setup=_tx_setup, plus=plus)
            _tx_preload(a)
            acc = 0
            for v in vals:
                acc = _rol(acc) ^ v
            e['r5'] = acc
            out.append((a, e, p))
    return out


@test
def tx_stlf():
    """TE = 1 store-to-load forwarding when the EA's page is not the base's:
    the same VA through another base / offset forwards (the stored value);
    the same page offset in another page (another PA) does not; a VA alias of
    the stored PA does not forward but reads the stored value; runs of
    stores then loads (the MEM / port / queued store candidates)"""
    if NOEXC:
        return []
    out = []
    for plus in ([], ['+HIT=100'], ['+HIT=70', '+LAT=1'], ['+HIT=100', '+LAT=7']):
        mem = _tx_init()
        vals = []
        def setup(a):
            _tx_setup(a)
            _tlb_set(a, 6, 0x0105, TX_MAP[0x0101], TLB_V | TLB_R | TLB_W)   # an alias of P1
        def body(a):
            a.setr(5, 0)
            def fold(rd, v):
                a.shift_n('rol', 5, 5, 1); a.alu('xor', 5, 5, rd); vals.append(v)
            # store [P1 end + 8] (= P2 start), load it as [P2 start + 16 - 16]
            a.setr(9, 0x0101FFF8); a.setr(10, 0x01020010); a.setr64(3, 0xA1)
            a.st(64, 3, 9, 8); a.ld(64, 7, 10, -16); fold(7, 0xA1)
            mem.update({_tx_pa(0x01020000) + i: (0xA1 >> 8 * i) & 0xFF for i in range(8)})
            # store via rs1 + rs2 (base 3 pages away), load [imm]
            a.setr(9, 0x01050008); a.setr(11, 0xFFFCFFF8); a.setr64(3, 0xB2)
            a.st_rr(64, 3, 9, 11); a.ld_abs(64, 7, 0x01020000); fold(7, 0xB2)
            # same page offset, other page: no forward
            a.setr(9, 0x0101FF00); a.setr64(3, 0xC3); a.st(64, 3, 9); a.ld_abs(64, 7, 0x0102FF00)
            fold(7, _tx_rd(mem, 0x0102FF00, 8))
            mem.update({_tx_pa(0x0101FF00) + i: (0xC3 >> 8 * i) & 0xFF for i in range(8)})
            # an alias VA of the stored PA: the stored value
            a.setr(9, 0x0101FF10); a.setr64(3, 0xD4); a.st(64, 3, 9); a.ld_abs(64, 7, 0x0105FF10)
            fold(7, 0xD4)
            # back-to-back stores, then loads of each through crossing bases
            vs = [0xE5, 0xF6, 0x17]
            eas = [0x0101FFF0, 0x01020008, 0x0101FFF8]
            for v, ea in zip(vs, eas):
                a.setr64(3, v); a.setr(9, ea + 0x18); a.st(64, 3, 9, -0x18)
            for v, ea in zip(reversed(vs), reversed(eas)):
                a.setr(10, ea - 0x10000); a.setr(11, 0x10000); a.ld_rr(64, 7, 10, 11); fold(7, v)
            a.setr(9, 0x01020000); a.st(16, 3, 9, -2); a.ld(16, 7, 9, -2); fold(7, 0x17)
        a, e, p = _te_prog(body, False, None, setup=setup, plus=plus)
        _tx_preload(a)
        acc = 0
        for v in vals:
            acc = _rol(acc) ^ v
        e['r5'] = acc
        out.append((a, e, p))
    return out


# ---- review fix-ups (2026-11-14): supervisor kmap, two-access priority, STLF partial cover, hit-pipe watchpoint
TX_KM = {0xC001: 0x005, 0xC002: 0x003}          # kmap VA page -> frame (alias PA windows already preloaded)


def _tx_kmsetup(a):
    _tx_setup(a)
    for i, (vp, f) in enumerate(sorted(TX_KM.items())):
        _tlb_set(a, 7 + i, vp, f, TLB_V | TLB_R | TLB_W, g=1)


def _tx_kpa(va):
    """PA of a supervisor VA in the tx windows (useg, kseg, kmap)"""
    va &= 0xFFFFFFFF
    if 0x80000000 <= va < 0x88000000:
        return va - 0x80000000
    if (va >> 16) in TX_KM:
        return (TX_KM[va >> 16] << 16) | (va & 0xFFFF)
    return _tx_pa(va)


def _tx_krd(mem, va, n):
    return sum(mem[_tx_kpa(va + i)] << (8 * i) for i in range(n))


@test
def tx_kmap():
    """TE = 1, supervisor: kmap (0xC000_0000-0xEFFF_FFFF, TLB-translated)
    accesses whose base is in another segment (useg, kseg, MMIO) and the
    reverse (kmap base, EA in useg / kseg / MMIO); faults: kmap EA in an
    unmapped page (TLB miss), kmap base with an EA in the 0x8800_0000 hole
    (ACCESS), a store through one segment read back through another"""
    if NOEXC:
        return []
    out = []
    for plus, dist in [(pl, d) for pl in ([], ['+HIT=70', '+LAT=1'], ['+HIT=100']) for d in (0, 1, 3)]:
        mem = _tx_init()
        logs = []
        vals = []
        def body(a):
            a.setr(5, 0)
            def fold(v):
                a.shift_n('rol', 5, 5, 1); a.alu('xor', 5, 5, 7); vals.append(v)
            def rr(op, base, ea, fault=None, val=None):
                off = (ea - base) & 0xFFFFFFFF
                if op == 'st':
                    a.setr64(7, val)
                a.setr(9, base); a.setr(11, off)
                for _ in range(dist):
                    a.nop()
                (a.ld_rr if op == 'ld' else a.st_rr)(64, 7, 9, 11)
                if fault:
                    logs.append((fault, ea))
                elif op == 'ld':
                    fold(_tx_krd(mem, ea, 8) if val is None else val)
                else:
                    for i in range(8):
                        mem[_tx_kpa(ea) + i] = (val >> (8 * i)) & 0xFF
            K1, K2 = 0xC001FF00, 0xC0020000
            rr('ld', 0x0100FF00, 0xC001FF00)               # useg base -> kmap EA
            rr('ld', 0x8005FF00, 0xC001FFF8)               # kseg base -> kmap
            rr('ld', 0xF0000000, 0xC0020008)               # MMIO base -> kmap (page 2)
            rr('ld', 0x01010000, 0xC001FFF8)               # useg -> kmap, just below the P1/P2 split
            rr('ld', K1, 0x0100FF00)                       # kmap base -> useg
            rr('ld', K1, 0x8005FF08)                       # kmap base -> kseg
            rr('ld', K2 - 8, 0x01010000)                   # kmap base -> useg page 1
            rr('ld', K1, 0xF0010010, val=2)                # kmap base -> MMIO (UART status)
            rr('ld', 0xF0000000, 0xC001FF00)               # MMIO base -> kmap
            rr('ld', K1, 0xC0020000 + 0x10)                # kmap -> kmap, next page
            rr('ld', 0x0100FF00, 0xC0000000, fault=X_TLB_LD)    # kmap page 0xC000 unmapped
            rr('ld', K1, 0xC0030008, fault=X_TLB_LD)            # next kmap page unmapped
            rr('ld', 0x80000000, 0xEFFFFFF8, fault=X_TLB_LD)    # kseg base -> unmapped kmap end
            rr('ld', K1, 0xBFFFFFF8, fault=X_ACC_LD)            # kmap base -> the hole
            rr('ld', 0x7FFFFFF8, 0xBFFFFFF8, fault=X_ACC_LD)    # useg base -> the hole
            rr('st', 0x0100FF00, 0xC001FF18, val=0xA5A5)        # store through kmap from useg ...
            rr('ld', 0x80000000, 0x8005FF18, val=0xA5A5)        # ... read through kseg
            rr('st', 0x8005FF20, 0xC0020028, val=0x5A5A)        # store through kmap from kseg
            rr('ld', K1, 0xC0020028, val=0x5A5A)                # ... read through kmap
            rr('ld', 0x8005FF20, 0xC0020028, val=0x5A5A)        # (a different base: the load forwards / reads)
            rr('st', 0xC0030000, 0x0100FF38, val=0x7777)        # store, kmap base -> useg
            rr('ld', 0x8005FF38, 0xC001FF38, val=0x7777)
        a, e, p = _te_prog(body, False, None, logs=logs, setup=_tx_kmsetup, plus=plus)
        _tx_preload(a)
        acc = 0
        for v in vals:
            acc = _rol(acc) ^ v
        e['r5'] = acc
        out.append((a, e, p))
    return out


@test
def tx_pair():
    """TE = 1: a two-access op (spanning MEMGET32, LEAVERET) whose first
    doubleword faults on the segment and the second on the TLB, and the
    reverse; both are translated before either access, the fault of the lowest
    faulting byte wins (the 1st access's, whatever its kind; EADDR = the raw
    EA for MEMGET32).  User and
    supervisor; operands from the register file / 1 / 3 instructions
    before"""
    if NOEXC:
        return []
    out = []
    for user, plus, dist in [(u, pl, d) for u in (False, True) for pl in ([], ['+HIT=70', '+LAT=1'])
                             for d in (0, 1, 3)]:
        logs = []
        def body(a):
            def ld32(ea, fault, eaddr):
                a.setr(9, ea)
                for _ in range(dist):
                    a.nop()
                a.ld(32, 7, 9)
                logs.append((fault, eaddr))
            def lr(sp, fault, eaddr):
                a.setr(15, sp)
                for _ in range(dist):
                    a.nop()
                a.leaveret()
                logs.append((fault, eaddr))
            if user:
                ld32(0x7FFFFFFD, X_TLB_LD, 0x7FFFFFFD)   # 1st: TLB (0x7FFF unmapped), 2nd: segment; the lowest byte's fault wins
                ld32(0xFFFFFFFD, X_ACC_LD, 0xFFFFFFFD)   # 1st: segment, 2nd: page 0 (wraps; TLB)
                lr(0x7FFFFFF8, X_TLB_LD, 0x7FFFFFF8)
                lr(0xFFFFFFF8, X_ACC_LD, 0xFFFFFFF8)
                ld32(0xEFFFFFFD, X_ACC_LD, 0xEFFFFFFD)   # both: segment
                ld32(0x0103FFFD, X_TLB_LD, 0x01040000)  # 1st mapped, 2nd TLB (control)
            else:
                ld32(0xBFFFFFFD, X_ACC_LD, 0xBFFFFFFD)   # 1st: the hole, 2nd: kmap page 0xC000 unmapped
                lr(0xBFFFFFF8, X_ACC_LD, 0xBFFFFFF8)
                ld32(0x7FFFFFFD, X_TLB_LD, 0x7FFFFFFD)   # 1st: TLB, 2nd: kseg (fine)
                ld32(0xC002FFFD, X_TLB_LD, 0xC0030000)  # 1st kmap mapped, 2nd unmapped
                ld32(0x87FFFFFD, X_ACC_LD, 0x88000000)  # kseg 1st, 2nd the hole
                # (a supervisor MEMGET32 at 0xFFFFFFFD, whose 2nd dword wraps to VA 0, takes no fault
                # HERE because _te_on maps page 0 (the code) — not a bug; tx_wrap makes page 0
                # execute-only and gets the fault, EADDR 0: HW_PERF_PLAN §8 item 3a)
        a, e, p = _te_prog(body, user, None, logs=logs, setup=_tx_kmsetup, plus=plus)
        _tx_preload(a)
        out.append((a, e, p))
    return out


@test
def tx_stlf_part():
    """TE = 1 store-to-load forwarding where the store covers only part of
    the load's doubleword (the load must wait for the store and merge it),
    with the store's and the load's EAs equal but their bases (and the
    offsets' split between page-crossing base and offset) different: sizes
    1 / 2 / 4 at every offset in a dword near the page boundaries, then a
    full-cover store with a narrower load (forwarded and shifted)"""
    if NOEXC:
        return []
    import random
    out = []
    for plus in ([], ['+HIT=100'], ['+HIT=70', '+LAT=1'], ['+HIT=100', '+LAT=7']):
        rnd = random.Random(len(plus) * 31 + 5)
        mem = _tx_init()
        vals = []
        def body(a):
            a.setr(5, 0)
            def fold(v):
                a.shift_n('rol', 5, 5, 1); a.alu('xor', 5, 5, 7); vals.append(v)
            cases = []
            for D in (0x0101FFF8, 0x01020000, 0x0102FFF8, 0x0100FFF8, 0x0101FF00):
                for sz in (1, 2, 4):
                    for k in range(0, 8, sz):
                        cases.append((D, sz, k))
            rnd.shuffle(cases)
            for n, (D, sz, k) in enumerate(cases[:36]):
                v = rnd.getrandbits(8 * sz) | 1
                sb = rnd.choice([D + 0x10000, D - 0x10000, D + 0x30000, D - 0x7F000000, 0x80000000, 0])
                lb = rnd.choice([D - 0x20000, D + 0x10000, D + 0x50000, 0x7FFF0000, D])
                a.setr64(3, v)
                a.setr(9, sb & 0xFFFFFFFF); a.setr(11, (D + k - sb) & 0xFFFFFFFF)
                a.setr(10, lb & 0xFFFFFFFF); a.setr(13, (D - lb) & 0xFFFFFFFF)
                a.st_rr(sz * 8, 3, 9, 11)
                if n % 3 == 2:
                    a.nop()
                a.ld_rr(64, 7, 10, 13)
                for i in range(sz):
                    mem[_tx_pa(D + k + i)] = (v >> (8 * i)) & 0xFF
                fold(_tx_rd(mem, D, 8))
            # a full-cover store, then a narrower load (forwarded, shifted)
            a.setr64(3, 0x1122334455667788)
            a.setr(9, 0x0102FFF8 + 0x10000); a.setr(11, 0xFFFF0000)
            a.st_rr(64, 3, 9, 11)
            for i in range(8):
                mem[_tx_pa(0x0102FFF8) + i] = (0x1122334455667788 >> (8 * i)) & 0xFF
            a.setr(10, 0x0102FFF8 + 0x20000); a.setr(13, 0xFFFE0004)
            a.ld_rr(32, 7, 10, 13); fold(_tx_rd(mem, 0x0102FFFC, 4))
        a, e, p = _te_prog(body, False, None, setup=_tx_setup, plus=plus)
        _tx_preload(a)
        acc = 0
        for v in vals:
            acc = _rol(acc) ^ v
        e['r5'] = acc
        out.append((a, e, p))
    return out


@test
def tx_wp():
    """TE = 1: a read watchpoint (on the PA) fires on a hit-pipe load whose
    base is in another page than its EA (DEBUG_EN builds; without the debug
    module it completes)"""
    dbg = _os.environ.get('VSIM_DEBUG_EN') == '1'
    out = []
    for plus in (['+HIT=100'], ['+HIT=100', '+LAT=7']):
        mem = _tx_init()
        pa = _tx_pa(0x0101FFF8)
        def body(a):
            a.setr(9, 0x01020000 + 0x20000); a.setr(11, 0xFFFDFFF8 & 0xFFFFFFFF)
            for _ in range(4):
                a.nop()
            a.ld_rr(64, 6, 9, 11)           # EA 0x0101FFF8 through a base 3 pages up
            a.setr(7, 0x77)
        a, e, p = _te_prog(body, False, None, setup=_tx_setup, plus=plus + ['+WP0RD=%x' % pa])
        _tx_preload(a)
        if dbg:
            e.update({'kind': '4', 'dbg_cause': '3', 'watch_addr': '%08x' % pa, 'r6': _tx_pat(pa)})
            e[m(XD + 32)] = 0
        else:
            e.update({'r6': _tx_pat(pa), 'r7': 0x77})
        out.append((a, e, p))
    return out


@test
def tx_walk():
    """TE = 1 with the walker: the walked page is the EA's, not the base's:
    a load whose base is in an unmapped page and EA in another (one walk, of
    the EA's page), back the other way (a 2nd walk), a 3rd access hits; a
    TLBINV, then a crossing LEAVERET walks both pages"""
    if NOEXC or NOWALK:
        return []
    fr = {0x0101: 0x3, 0x0102: 0x7}
    ptes = {vp: _wk_pte(f, TLB_RWX) for vp, f in fr.items()}
    def pa(va):
        return (fr[va >> 16] << 16) | (va & 0xFFFF)
    def body(a):
        a.setr(9, 0x0101FFF8); a.setr(11, 0x18); a.ld_rr(64, 3, 9, 11)     # EA 0x0102_0010: walks 0x0102
        a.setr(9, 0x01020000); a.ld(64, 4, 9, -0x10)                       # EA 0x0101_FFF0: walks 0x0101
        a.setr(9, 0x01000010); a.ld(64, 5, 9, 0x20000, mode='long')       # EA 0x0102_0010: hits
        a.tlbinv()                                                         # (the next fetch walks page 0)
        a.setr(15, 0x0101FFF0); a.addi_s(15, 15, 8); a.leaveret()          # walks 0x0101, then 0x0102
        a.setr(6, 0xBAD)
        a.label('tx_wb')
        a.mov(6, 15)
        a.mfsr(7, 'tlb_wcnt')
    exp = {'r3': 0x3333, 'r4': 0x4444, 'r5': 0x3333, 'r6': 0x6666, 'r7': 5, m(XD + 32): 0}
    a, e, p = _wk_prog(body, ptes, exp)
    a.dword(pa(0x01020010), 0x3333)
    a.dword(pa(0x0101FFF0), 0x4444)
    a.dword(pa(0x0101FFF8), 0x6666)
    a.dword(pa(0x01020000), a.labels['tx_wb'])
    return a, e, p

# the TLB_WALK = 0 build has no walker: only wk_nowalk applies there
if NOWALK:
    for _n in [n for n in ALL if n.startswith('wk_') and n != 'wk_nowalk']:
        del ALL[_n]



# ------------------------------------- HW_PERF_PLAN §8: frames popped from MMIO
MMIO_RAM = 0xF0FD0000      # tb_vsim's MMIO scratch RAM (32 dwords, reads back)


def _mmio_frame(kind, called, n=1, plus=(), irq=False):
    """supervisor RET / LEAVERET whose frame (return address, saved R15) is
    in MMIO (tb_vsim's scratch RAM), and a MEMGET32 spanning two MMIO
    dwords, n times in a loop.  LEAVERET and the spanning MEMGET32 read two
    MMIO dwords: the second read must reach the device as a new request
    (tb_vsim models the SoC's level MMIO ready, mmio_hold).  called: the
    frame belongs to a CALL, so the return is RAS-predicted (and the
    prediction right) — an MMIO return address still counts as a miss."""
    a = Asm()
    mb = MMIO_RAM + 0x80       # (room below for an IRQ frame + the handler's pushes)
    if irq:
        a.set_int_mask(1)
    a.setr(12, DATA); a.setr(11, GOOD); a.setr(9, mb); a.getsp(13)
    a.setr(8, 0); a.setr(7, n)
    a.setr64(3, 0x8877665544332211); a.st(64, 3, 9, 0)
    a.setr(3, 0x99); a.st(64, 3, 9, 8)
    a.label('top')
    if kind == 'ld32':
        a.setr(5, 0)
        a.addi_s(10, 9, 5); a.ld(32, 5, 10)     # MEMGET32: bytes 5..8, dword 0's top 3, dword 1's first
        a.setr(3, 0x99887766)
        a.fbr('ne', 5, 3, 'bad')
    else:
        if called:
            a.call('f')
            a.label('back')
        else:
            a.jmp('f')
            a.label('back')
        a.getsp(2); a.setsp(13)
        a.setr(3, mb + 16); a.fbr('ne', 2, 3, 'bad')
        if kind == 'leaveret':
            a.fbr('ne', 15, 11, 'bad')
    a.inc(8)
    a.addi_s(7, 7, -1); a.fbr('ne', 7, 0, 'top', imm=True)
    a.halt()
    a.label('bad'); a.setr(4, 0xBAD); a.halt()
    if kind != 'ld32':
        a.label('f')
        a.leapc(6, 'back'); a.st(64, 6, 9, 8); a.st(64, 11, 9, 0)
        a.setr(15, 0)
        if kind == 'ret':
            a.addi_s(10, 9, 8); a.setsp(10); a.ret()
        else:
            a.mov(15, 9); a.leaveret()
        a.setr(4, 0xBAD); a.halt()
    exp = {'end': 'park', 'kind': '0', 'r4': 0, 'r8': n, 'mmio_hold': '0'}
    return a, exp, list(plus)


@test
def mmio_frame():
    """HW_PERF_PLAN §8: RET / LEAVERET popping their frame from MMIO and a
    MEMGET32 spanning two MMIO dwords (supervisor), RAS-predicted or not, at
    two latencies; LEAVERET and MEMGET32 deadlocked on the SoC (the second
    MMIO read under a held DV)"""
    out = []
    for kind in ('ret', 'leaveret', 'ld32'):
        for called in ((False, True) if kind != 'ld32' else (False,)):
            for lat in (1, 3):
                out.append(_mmio_frame(kind, called, 2, [f'+LAT={lat}']))
    return out


@test
def mmio_frame_irq():
    """mmio_frame in a loop under a timer IRQ (the IRQ frame may land in the
    MMIO scratch too, while SP points there)"""
    out = []
    for kind in ('ret', 'leaveret', 'ld32'):
        for called in ((False, True) if kind != 'ld32' else (False,)):
            for per in (97, 151):
                a, exp, plus = _mmio_frame(kind, called, 40, [f'+IRQ_PERIOD={per}', '+LAT=2', '+MAXC=200000'],
                                           irq=True)
                exp['m00180000'] = 'nonzero'
                out.append((a, exp, plus))
    return out


@test
def tx_wrap():
    """MMU_PLAN §11.9 (HW_PERF_PLAN §8 item 3a): TE = 1 supervisor, page 0
    execute-only: a MEMGET32 at 0xFFFF_FFFD / a LEAVERET at R15 =
    0xFFFF_FFF8 (1st doubleword MMIO, 2nd wraps to VA 0) take page 0's
    permission fault (EADDR = VA 0, the lowest faulting byte); a plain load
    from page 0 is the control"""
    if NOEXC:
        return []
    out = []
    for plus, dist in [(pl, d) for pl in ([], ['+HIT=70', '+LAT=1']) for d in (0, 1, 3)]:
        logs = []
        def body(a):
            _tlb_set(a, 0, 0x0000, 0x000, TLB_V | TLB_X, g=1)
            a.setr(9, 0x8); a.ld(64, 7, 9)
            logs.append((X_ACC_LD, 0x8))
            a.setr(9, 0xFFFFFFFD)
            for _ in range(dist):
                a.nop()
            a.ld(32, 7, 9)
            logs.append((X_ACC_LD, 0x0))
            a.setr(15, 0xFFFFFFF8)
            for _ in range(dist):
                a.nop()
            a.leaveret()
            logs.append((X_ACC_LD, 0x0))
            _tlb_set(a, 0, 0x0000, 0x000, TLB_RWX, g=1)
        a, e, p = _te_prog(body, False, None, logs=logs, plus=plus)
        out.append((a, e, p))
    return out


@test
def fwd_wl_prio():
    """HW_PERF_PLAN §10: a fast load in WB and a younger producer of the
    same register in MEM: the MEM value wins for every ID -> EX operand —
    rs1 (an ALU op and an address base), rs2, and a store's data (ex_c) —
    now that the WB load / MUL forward is a separate, later select"""
    out = []
    for plus in ([], ['+LAT=1'], ['+LAT=3'], ['+HIT=70'], ['+HIT=95', '+LAT=2']):
        a = Asm()
        a.dword(DATA, 0x1111)
        a.setr(1, DATA); a.setr(2, 0x40)
        # store data (ex_c): LD r3; ADDI r3 <- r2 + 7 (in MEM as the ST dispatches); ST r3
        a.ld(64, 3, 1)
        a.addi_s(3, 2, 7)
        a.st(64, 3, 1, 8)
        # rs1: LD r4; ADDI r4; ADD r5 <- r4 + r2
        a.ld(64, 4, 1)
        a.addi_s(4, 2, 9)
        a.alu('add', 5, 4, 2)
        # rs2: LD r6; ADDI r6; ADD r7 <- r2 + r6
        a.ld(64, 6, 1)
        a.addi_s(6, 2, 11)
        a.alu('add', 7, 2, 6)
        # address base (rs1 of a load): LD r8; SETR r8 <- DATA + 8; LD r9 <- [r8]
        a.ld(64, 8, 1)
        a.setr(8, DATA + 8)
        a.ld(64, 9, 8)
        a.halt()
        out.append((a, {'end': 'park', 'kind': '0', m(DATA + 8): 0x47, 'r5': 0x89, 'r7': 0x8B,
                        'r9': 0x47}, plus))
    return out


# ================================ MMU_PLAN.md §13: user single step / watch
# _udhandler logs every exception entry at UDLOG + 32 i: ECAUSE, EADDR, the
# frame PC (frame[31:0]) and frame >> 44 ({SS, PM}); XD + 32 counts them.
# A user HALT (PRIVILEGED, EADDR = the HALT word) halts (logged); STEP /
# WATCH / SYSCALL return; other faults are stepped over (+4).  With
# cfg=True a SYSCALL instead loads WP0 / WP1 from r5-r8 (ADDR, CTRL, ADDR,
# CTRL) and returns, unlogged — so user code configures its watchpoints.
X_STEP, X_WATCH = 14, 15
SSTEP = 1 << 12
UDLOG = XD + 0x40
from kasm import WP_R, WP_W, WP_U


def _udhandler(a, name='h', cfg=False, skip=True):
    a.label(name)
    a.push(13); a.push(14)
    a.mfsr(13, 'ecause'); a.alui('and', 13, 13, 0xFF, sgn=False)
    if cfg:
        a.fbr('ne', 13, X_SYSCALL, name + '_l', imm=True)
        a.mtsr('wp0_addr', 5); a.mtsr('wp0_ctrl', 6); a.mtsr('wp1_addr', 7); a.mtsr('wp1_ctrl', 8)
        a.jmp(name + '_r')
        a.label(name + '_l')
    a.setr(14, XD); a.ld(64, 13, 14, 32); a.inc(13); a.st(64, 13, 14, 32)
    a.shift_n('shl', 13, 13, 5); a.alu('add', 14, 14, 13); a.addi_s(14, 14, 0x40 - 32)
    a.mfsr(13, 'ecause'); a.st(64, 13, 14, 0)
    a.mfsr(13, 'eaddr');  a.st(64, 13, 14, 8)
    a.getsp(13); a.ld(64, 13, 13, 16); a.shift_n('shl', 13, 13, 32); a.shift_n('shr', 13, 13, 32)
    a.st(64, 13, 14, 16)
    a.getsp(13); a.ld(64, 13, 13, 16); a.shift_n('shr', 13, 13, 44); a.st(64, 13, 14, 24)
    a.mfsr(13, 'ecause'); a.alui('and', 13, 13, 0xFF, sgn=False)
    a.addi_s(14, 13, -X_STEP)
    a.fbr('ule', 14, 1, name + '_r', imm=True)
    a.fbr('eq', 13, X_SYSCALL, name + '_r', imm=True)
    a.fbr('ne', 13, X_PRIV, name + '_s', imm=True)
    a.mfsr(13, 'eaddr'); a.setr(14, HALT_W)
    a.fbr('ne', 13, 14, name + '_s')
    a.halt()
    a.label(name + '_s')
    if skip:
        a.getsp(13); a.ld(64, 14, 13, 16); a.addi_s(14, 14, 4); a.st(64, 14, 13, 16)
    a.label(name + '_r')
    a.pop(14); a.pop(13)
    a.iret()


def _ud_user(a, ucode='u', ss=1, mask=0, pm=1, usp=USP):
    """IRET into ucode with frame PM / SS"""
    a.setr(1, usp); a.mtsr('usp', 1)
    a.leapc(2, ucode)
    a.setr64(3, (pm << 44) | (ss << 45) | (mask << 39)); a.alu('or', 2, 2, 3)
    a.push(2)
    a.iret()


def _udexp(entries, count=None):
    """entries: (ecause, eaddr, frame pc, frame >> 44)"""
    e = {m(XD + 32): len(entries) if count is None else count}
    for i, (c, ea, fp, hi) in enumerate(entries):
        o = UDLOG + 32 * i
        e.update({m(o): c, m(o + 8): ea, m(o + 16): fp, m(o + 24): hi})
    return e


def _udprog(body, ss=1, cfg=False, mask=0, plus=(), exp=None, entries=None, wps=None, pre=None):
    """supervisor prologue (EVEC, optional WPs = [(addr, ctrl)] x <= 2),
    then body in user mode (IRET with SS = ss) ending in HALT"""
    a = Asm()
    _xsetup(a)
    for n, (addr, ctrl) in enumerate(wps or []):
        a.setr(1, addr); a.mtsr(f'wp{n}_addr', 1)
        a.setr(1, ctrl); a.mtsr(f'wp{n}_ctrl', 1)
    if pre:
        pre(a)
    _ud_user(a, ss=ss, mask=mask)
    a.label('u')
    info = body(a) or {}
    hp = a.halt()
    _udhandler(a, cfg=cfg)
    e = {'end': 'park', 'kind': '0'}
    if entries is not None:
        ents = entries(a, info, hp) if callable(entries) else entries
        e.update(_udexp(ents))
    e.update(exp or {})
    return a, e, list(plus)


@test
def ud_step_basic():
    """§13.2: a stepped user routine — straight code, a taken FBR, CALL /
    RET, a SYSCALL (entry with SS = 1, frame = next), a faulting load
    (ACCESS_LOAD with SS = 1, stepped over by the handler), a JMP; one STEP
    per retired instruction, EADDR = its PC, frame = the next PC"""
    out = []
    for plus in ([], ['+LAT=1'], ['+LAT=7'], ['+HIT=70']):
        P = {}

        def body(a):
            P['s1'] = a.setr(1, 5)
            P['add'] = a.alu('add', 2, 1, 1)
            P['fbr'] = a.fbr('eq', 1, 5, 'T', imm=True)
            P['skp'] = a.nop()
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
            return P

        def ents(a, P, hp):
            s = lambda at, nx: (X_STEP | SSTEP, at, nx, 3)
            return [s(P['s1'], P['add']), s(P['add'], P['fbr']), s(P['fbr'], a.labels['T']),
                    s(P['call'], P['inc']), s(P['inc'], P['ret']), s(P['ret'], P['sys']),
                    (X_SYSCALL, 0, P['s3'], 3), s(P['s3'], P['ld']),
                    (X_ACC_LD, 0xF0000000, P['ld'], 3), s(P['s3b'], P['ldm']),
                    (X_MISAL_LD, DATA + 1, P['ldm'], 3), s(P['jmp'], hp), (X_PRIV, HALT_W, hp, 3)]
        out.append(_udprog(body, entries=ents, plus=plus, exp={'r1': 6, 'r2': 10, 'r4': 0, 'sp': 0x08000000 - 24}))
    return out


@test
def ud_step_arm_rules():
    """§13.2: SS arms only on an IRET to user mode with EVEC != 0: an IRET
    to supervisor with SS = 1 steps nothing; EVEC = 0 at the IRET steps
    nothing (the user HALT then parks as ILLEGAL, kind 2); SS = 0 steps
    nothing"""
    out = []
    # supervisor target
    a = Asm()
    _xsetup(a)
    a.leapc(2, 's'); a.setr64(3, 1 << 45); a.alu('or', 2, 2, 3); a.push(2); a.iret()
    a.label('s')
    a.setr(1, 1); a.inc(1); a.inc(1)
    a.syscall()
    a.inc(1)
    a.halt()
    _udhandler(a)
    e = {'end': 'park', 'kind': '0', 'r1': 4}
    e.update(_udexp([(X_SYSCALL, 0, a.labels['s'] + 20, 0)]))
    out.append((a, e, []))
    # EVEC = 0
    a = Asm()
    _ud_user(a, ss=1)
    a.label('u')
    a.setr(1, 1); a.inc(1)
    hp = a.halt()
    out.append((a, {'end': 'park', 'kind': '2', 'pc': hp, 'r1': 2, m(XD + 32): 0}, []))
    # SS = 0
    out.append(_udprog(lambda a: (a.setr(1, 1), a.inc(1), a.inc(1)) and None, ss=0,
                       entries=lambda a, P, hp: [(X_PRIV, HALT_W, hp, 1)], exp={'r1': 3}))
    return out


@test
def ud_step_irq():
    """§13.2: stepping a user loop under timer IRQs: exactly one STEP per
    user instruction (the loop's dynamic count), never one in the ISR
    (the core asserts that too), and the program's result intact.  An
    interrupt pending at the arming IRET is taken first (frame SS = 1)."""
    out = []
    for per in (61, 79, 97, 131):   # (the TB's ISR takes ~57 cycles: shorter periods livelock, stepped or not)
        a = Asm()
        _xsetup(a)
        _ud_user(a, ss=1, mask=1)
        a.label('u')
        a.setr(1, 0); a.setr(2, 40)
        a.label('l'); a.alu('add', 1, 1, 2); a.push(1); a.pop(3); a.dec(2)
        a.fbr('ne', 2, 0, 'l', imm=True)
        a.halt()
        # a counting handler: STEP count at XD, EADDR outside the user code at XD+8
        a.label('h')
        a.push(13); a.push(14)
        a.mfsr(13, 'ecause'); a.alui('and', 13, 13, 0xFF, sgn=False)
        a.addi_s(13, 13, -X_STEP); a.fbr('ne', 13, 0, 'h_n', imm=True)
        a.setr(14, XD); a.ld(64, 13, 14); a.inc(13); a.st(64, 13, 14)
        a.mfsr(13, 'eaddr'); a.setr(14, a.labels['u']); a.alu('sub', 13, 13, 14)
        a.setr(14, a.pc - a.labels['u']); a.fbr('ult', 13, 14, 'h_r')
        a.setr(14, XD); a.ld(64, 13, 14, 8); a.inc(13); a.st(64, 13, 14, 8)
        a.jmp('h_r')
        a.label('h_n')
        a.halt()                                    # the user HALT (PRIVILEGED)
        a.label('h_r')
        a.pop(14); a.pop(13)
        a.iret()
        out.append((a, {'end': 'park', 'kind': '0', 'r1': 820, m(XD): 2 + 5 * 40, m(XD + 8): 0,
                        'm00180000': 'nonzero'}, [f'+IRQ_PERIOD={per}']))
    return out


@test
def ud_step_lrsc():
    """§13.2: a stepped SC always fails (every entry and IRET clear the
    reservation); unstepped, the same LR / SC succeeds"""
    out = []
    for ss in (0, 1):
        def body(a):
            a.setr(9, DATA); a.setr(8, 0x55)
            a.lr(64, 4, 9); a.sc(64, 5, 8, 9)
        out.append(_udprog(body, ss=ss, exp={'r5': ss, m(DATA): 0 if ss else 0x55}))
    return out


@test
def ud_step_flow():
    """§13.2: the frame PC is the architectural successor whatever the
    predictor did: a JMPcc not taken and taken (mispredicted), a CALL whose
    return address the callee overwrites (RET: a RAS miss), an indirect
    CALLR into ENTER / LEAVERET, a serializing MTSR TP, and a store that
    rewrites the next instruction (SMC: the new one runs)"""
    out = []
    for plus in ([], ['+LAT=1'], ['+HIT=70'], ['+LAT=5']):
        P = {}
        seq = []

        def body(a):
            E = lambda k, f: (P.__setitem__(k, f()), seq.append(k))
            E('s1', lambda: a.setr(1, 0))
            E('cmp', lambda: a.cmpi_s(1, 0))
            E('jnt', lambda: a.jmp('A', cond='ne'))
            E('n1', lambda: a.nop())
            E('cmp2', lambda: a.cmpi_s(1, 1))
            E('jt', lambda: a.jmp('B', cond='ne'))
            a.nop()
            a.label('B')
            E('call', lambda: a.call('G'))
            a.nop()
            a.label('R2')
            P['R2'] = a.labels['R2']
            E('lp', lambda: a.leapc(9, 'H'))
            E('cr', lambda: a.jmpr(9, link=True))
            E('tp', lambda: a.mtsr('tp', 1))
            while (a.pc + 8 + 8 + 4) % 8:
                E(f'pad{a.pc}', lambda: a.nop())
            E('s10', lambda: a.leapc(10, 'NEWI'))
            E('lda', lambda: a.ld_abs(64, 6, DATA + 0x100))
            E('st', lambda: a.st(64, 6, 10))
            a.label('NEWI')
            E('newi', lambda: a.setr(7, 0x11))
            E('jmp', lambda: a.jmp('E'))
            a.label('A'); a.nop()
            a.label('G')
            E('g0', lambda: a.leapc(8, 'R2'))
            E('g1', lambda: a.getsp(11))
            E('g2', lambda: a.st(64, 8, 11))
            E('g3', lambda: a.ret())
            a.label('H')
            E('h0', lambda: a.enter(0))
            E('h1', lambda: a.leaveret())
            a.label('E')
            return P
        a, e, pl = _udprog(body, plus=plus, exp={'r7': 0x22})
        assert a.labels['NEWI'] % 8 == 0
        a.dword(DATA + 0x100, 0x22 << 32 | (2 << 30 | 2 << 26 | 15 << 22 | 7 << 8))
        order = ['s1', 'cmp', 'jnt', 'n1', 'cmp2', 'jt', 'call', 'g0', 'g1', 'g2', 'g3', 'lp', 'cr',
                 'h0', 'h1', 'tp'] + [k for k in seq if k.startswith('pad')] + ['s10', 'lda', 'st', 'newi', 'jmp']
        hp = a.labels['E']
        ents = [(X_STEP | SSTEP, P[k], P[order[i + 1]] if i + 1 < len(order) else hp, 3)
                for i, k in enumerate(order)] + [(X_PRIV, HALT_W, hp, 3)]
        e.update(_udexp(ents))
        e[m(DATA + 0x100)] = 0x22 << 32 | (2 << 30 | 2 << 26 | 15 << 22 | 7 << 8)
        out.append((a, e, pl))
    return out


def _uwcfg(a, wp0=(0, 0), wp1=(0, 0)):
    """user code: configure WP0 / WP1 through the cfg SYSCALL"""
    a.setr(5, wp0[0]); a.setr(6, wp0[1]); a.setr(7, wp1[0]); a.setr(8, wp1[1])
    a.syscall()


def _uw_bytes(sz, ea):
    e = ea & 7
    return {8: 1 << e, 16: 3 << (e & 6), 32: 0xF0 if e & 4 else 0x0F, 64: 0xFF}[sz]


def _uw_chunks(cases, emit_case, ss=0, cap=26, plus=()):
    """cases -> programs of at most cap logged entries each; emit_case(a,
    case) returns the expected log entries (after the access executes)"""
    out, cur = [], []
    def flush():
        if not cur:
            return
        batch = list(cur)
        ents = []

        def body(a):
            for c in batch:
                ents.extend(emit_case(a, c))
        a, e, p = _udprog(body, ss=ss, cfg=True, plus=plus)
        hp = a.insn_starts[a.insn_starts.index(a.labels['h']) - 1]
        e.update(_udexp(ents + [(X_PRIV, HALT_W, hp, 1 | ss << 1)]))
        out.append((a, e, p))
        cur.clear()
    n = 0
    for c in cases:
        cur.append(c)
        n += 1
        if n >= cap:
            flush(); n = 0
    flush()
    return out


@test
def uw_sizes():
    """§13.3: 8 / 16 / 32 / 64-bit loads and stores (reg + reg form) at every
    legal offset of a doubleword (64-bit: every offset, the aligned-down
    doubleword with MAL64 = 0) against WP0 configured through a SYSCALL:
    BE = exactly the touched bytes / one of them (hit), their complement,
    the next doubleword, the wrong direction (no hit), U = 1 (a user access
    hits).  EADDR = the raw EA, frame = the next instruction."""
    cases = []
    base = DATA + 0x200
    for sz in (8, 16, 32, 64):
        for st in (0, 1):
            for off in range(8):
                if off % (sz // 8) and sz < 64:
                    continue
                ea = base + 0x40 + off
                bt = _uw_bytes(sz, ea)
                rw = WP_W if st else WP_R
                lo = bt & -bt
                for k, (wa, ctrl, hit) in enumerate([(ea, bt | rw, 1), (ea & ~7, lo | rw | WP_U, 1),
                                                     (ea, (~bt & 0xFF) | WP_R | WP_W, 0),
                                                     (ea + 8, 0xFF | WP_R | WP_W, 0),
                                                     (ea, 0xFF | (WP_R if st else WP_W), 0)]):
                    if ctrl & 0xFF:
                        cases.append((sz, st, ea, wa, ctrl, hit))

    # (half the cases split the EA between rs1 and rs2 at random: every
    # carry of the EA's operands — the watch compare is carry-free on them)
    import random as _r
    rnd = _r.Random(13)
    cases = [c + ((rnd.getrandbits(32) if rnd.random() < 0.5 else rnd.randrange(0, 0x100)),) for c in cases]

    def emit(a, c):
        sz, st, ea, wa, ctrl, hit, k = c
        _uwcfg(a, (wa, ctrl))
        a.setr(9, (ea - k) & 0xFFFFFFFF); a.setr(10, k); a.setr64(11, 0x1122334455667788)
        at = a.st_rr(sz, 11, 9, 10) if st else a.ld_rr(sz, 4, 9, 10)
        return [(X_WATCH | 1 << 9 | st << 11, ea, at + 4, 1)] if hit else []
    out = []
    for plus in ([], ['+HIT=70', '+LAT=2']):
        out += _uw_chunks(cases, emit, plus=plus)
    return out


@test
def uw_stack():
    """§13.3: the stack and frame ops — PUSH, PUSHV, POP, CALL (frame = its
    target), RET (frame = the return address, RAS-predicted), ENTER,
    LEAVE, LEAVERET on either doubleword (the 2nd only: EADDR = its start),
    and a CALLcc not taken (no access, no hit)"""
    out = []
    for plus in ([], ['+HIT=70'], ['+LAT=1']):
        ents = []

        def body(a):
            W = lambda dw, rw: _uwcfg(a, (dw, 0xFF | rw))
            sp = USP
            W(sp - 8, WP_W); at = a.push(1); ents.append((X_WATCH | 1 << 9 | 1 << 11, sp - 8, at + 4, 1))
            W(sp - 8, WP_R); at = a.pop(2); ents.append((X_WATCH | 1 << 9, sp - 8, at + 4, 1))
            W(sp - 8, WP_W); at = a.pushv(0x77); ents.append((X_WATCH | 1 << 9 | 1 << 11, sp - 8, at + 8, 1))
            a.pop(2)
            W(sp - 8, WP_W); at = a.call('F'); ents.append((X_WATCH | 1 << 9 | 1 << 11, sp - 8, a.labels.get('F', 0), 1))
            ret_at = at + 4
            a.setr(1, 1); a.cmpi_s(1, 0)
            at = a.call('F', cond='eq'); a.nop()                       # not taken: no hit
            W(sp - 8, WP_R); a.call('G')
            a.label('RG')
            # ENTER / LEAVE / LEAVERET
            W(sp - 8, WP_W); at = a.call('K')
            a.jmp('Z')
            a.label('F'); a.ret()
            a.label('G'); g = a.ret()                                    # RET: hit, frame = RG
            ents.append((X_WATCH | 1 << 9, sp - 8, a.labels['RG'], 1))
            a.label('K')                                                  # (the CALL to K hit: frame = K)
            ents.append((X_WATCH | 1 << 9 | 1 << 11, sp - 8, a.labels['K'], 1))
            W(sp - 16, WP_W); at = a.enter(1)                            # R15 pushed at SP - 8 = USP - 16
            ents.append((X_WATCH | 1 << 9 | 1 << 11, sp - 16, at + 4, 1))
            W(sp - 16, WP_R); at = a.leave()                             # LEAVE: [R15] = USP - 16
            ents.append((X_WATCH | 1 << 9, sp - 16, at + 4, 1))
            a.ret()
            a.label('Z')
            a.call('L1'); a.call('L2')
            a.jmp('Y')
            a.label('L1'); a.enter(0)                                    # R15 = USP - 16
            W(sp - 8, WP_R); at = a.leaveret()                           # 2nd dword (the return address)
            ents.append((X_WATCH | 1 << 9, sp - 8, a.labels['Z'] + 4, 1))
            a.label('L2'); a.enter(0)
            W(sp - 16, WP_R); at = a.leaveret()                          # 1st dword (saved R15)
            ents.append((X_WATCH | 1 << 9, sp - 16, a.labels['Z'] + 8, 1))
            a.label('Y')
        a, e, p = _udprog(body, ss=0, cfg=True, plus=plus)
        hp = a.labels['Y']
        # (fix the CALL-to-F frame: F's label was not known when recorded)
        ents[3] = (ents[3][0], ents[3][1], a.labels['F'], 1)
        e.update(_udexp(ents + [(X_PRIV, HALT_W, hp, 1)]))
        out.append((a, e, p))
    return out


@test
def uw_ld32_dcbz():
    """§13.3: MEMGET32 at every offset with WP0 on its first doubleword and
    WP1 on the next (only bytes it really touches hit; the 2nd doubleword
    alone: EADDR = its start); DCBZ with a WP on each doubleword of its line
    (EADDR = the raw rs1)"""
    cases = []
    b = DATA + 0x300
    for off in range(8):
        for be0, be1 in ((0xFF, 0), (0, 0x01), (0, 0x0F), (0, 0x80), (0x80, 0x01), (0x0F, 0)):
            cases.append(('l32', b + off, be0, be1))
    for k in range(4):
        for be in (0xFF, 0x80):
            cases.append(('dz', b + 0x40 + 0x13, k, be))

    def emit(a, c):
        if c[0] == 'l32':
            _, ea, be0, be1 = c
            _uwcfg(a, (ea & ~7, be0 | WP_R) if be0 else (0, 0), ((ea & ~7) + 8, be1 | WP_R) if be1 else (0, 0))
            a.setr(9, ea)
            at = a.ld(32, 4, 9)
            m32 = 0xF << (ea & 7)
            h0 = bool(m32 & be0)
            h1 = bool((m32 >> 8) & be1)
            if not (h0 or h1):
                return []
            return [(X_WATCH | h0 << 9 | h1 << 10, ea if h0 else (ea & ~7) + 8, at + 4, 1)]
        _, ea, k, be = c
        _uwcfg(a, ((ea & ~31) + 8 * k, be | WP_W))
        a.setr(9, ea)
        at = a.dcbz(9)
        return [(X_WATCH | 1 << 9 | 1 << 11, ea, at + 4, 1)]
    out = []
    for plus in ([], ['+HIT=70', '+LAT=1']):
        out += _uw_chunks(cases, emit, plus=plus)
    return out


@test
def uw_cases():
    """§13.3 / §13.4: LR hits (R); a successful SC hits (W), a failing one
    does not; a user MMIO load and a misaligned 16-bit load on a watched
    doubleword take their fault, not WATCH; both WPs on one access set both
    bits; step + watch on one instruction is one WATCH with SSTEP"""
    out = []
    # LR / SC, faults, both WPs (unstepped)
    ents = []

    def body(a):
        d = DATA + 0x380
        _uwcfg(a, (d, 0xFF | WP_W))
        a.setr(9, d); a.setr(8, 0x99)
        a.lr(64, 4, 9)                                           # (W only: no hit)
        at = a.sc(64, 3, 8, 9); ents.append((X_WATCH | 1 << 9 | 1 << 11, d, at + 4, 1))
        at = a.sc(64, 2, 8, 9)                                   # no reservation: fails, no store
        _uwcfg(a, (d, 0xFF | WP_R | WP_W))
        at = a.lr(64, 4, 9); ents.append((X_WATCH | 1 << 9, d, at + 4, 1))
        a.sc(64, 10, 8, 9)                                       # the WATCH entry cleared the reservation
        _uwcfg(a, (0xF0000000, 0xFF | WP_R | WP_W | WP_U))
        a.setr(9, 0xF0000000)
        at = a.ld(64, 4, 9); ents.append((X_ACC_LD, 0xF0000000, at, 1))
        _uwcfg(a, (d, 0xFF | WP_R))
        a.setr(9, d + 1)
        at = a.ld(16, 4, 9); ents.append((X_MISAL_LD, d + 1, at, 1))
        _uwcfg(a, (d, 0x0F | WP_R), (d, 0x01 | WP_R | WP_U))
        a.setr(9, d)
        at = a.ld(8, 4, 9, 0); ents.append((X_WATCH | 3 << 9, d, at + 4, 1))
        _uwcfg(a, (d, 0x0F | WP_R), (d, 0xF0 | WP_R))
        at = a.ld(8, 4, 9, 5); ents.append((X_WATCH | 1 << 10, d + 5, at + 4, 1))
    a, e, p = _udprog(body, ss=0, cfg=True)
    hp = a.insn_starts[a.insn_starts.index(a.labels['h']) - 1]
    e.update(_udexp(ents + [(X_PRIV, HALT_W, hp, 1)]))
    e.update({'r3': 0, 'r2': 1, 'r10': 1, m(DATA + 0x380): 0x99})
    out.append((a, e, p))
    # step + watch: SSTEP on the WATCH of the stepped instruction
    P = {}

    def body2(a):
        d = DATA + 0x388
        P['s9'] = a.setr(9, d)
        P['st'] = a.st(64, 9, 9)
        P['ld'] = a.ld(64, 4, 9)
        P['n'] = a.nop()
    a, e, p = _udprog(body2, ss=1, wps=[(DATA + 0x388, 0xFF | WP_W)])
    hp = P['n'] + 4
    s = lambda at, nx: (X_STEP | SSTEP, at, nx, 3)
    e.update(_udexp([s(P['s9'], P['st']), (X_WATCH | SSTEP | 1 << 9 | 1 << 11, DATA + 0x388, P['ld'], 3),
                     s(P['ld'], P['n']), s(P['n'], hp), (X_PRIV, HALT_W, hp, 3)]))
    out.append((a, e, p))
    return out


@test
def uw_supervisor():
    """§13.3 supervisor rules: U = 1 never hits in supervisor mode; U = 0
    hits only with DI = 0 in program order (a DI just before the access
    blocks it, an EI just before enables it); a kernel read of a user-only
    watched address does not hit; EVEC = 0: no hit at all"""
    out = []
    a = Asm()
    _xsetup(a)
    d = DATA + 0x390
    a.setr(9, d); a.setr(1, 1)
    ents = []
    a.setr(2, d); a.mtsr('wp0_addr', 2); a.setr(2, 0xFF | WP_R | WP_W | WP_U); a.mtsr('wp0_ctrl', 2)
    a.ld(64, 4, 9); a.st(64, 1, 9)                              # U = 1: no hit
    a.setr(2, 0xFF | WP_R | WP_W); a.mtsr('wp0_ctrl', 2)
    at = a.st(64, 1, 9); ents.append((X_WATCH | 1 << 9 | 1 << 11, d, at + 4, 0))
    a.di(3)
    a.ld(64, 4, 9)                                              # DI = 1: no hit
    a.ei(1)
    at = a.ld(64, 4, 9); ents.append((X_WATCH | 1 << 9, d, at + 4, 0))
    a.di(3); a.ei(1); at = a.ld(64, 5, 9); ents.append((X_WATCH | 1 << 9, d, at + 4, 0))
    a.ei(1); a.di(3); a.ld(64, 5, 9)                            # no hit
    a.ei(1)
    # an LCD op right behind a watched load (a hit-pipe load leaves MEM on
    # its fire edge): squashed before it strobes, sent once after the handler
    at = a.ld(64, 5, 9); ents.append((X_WATCH | 1 << 9, d, at + 4, 0))
    a.raw(0x70000010)                                           # LCDCMD r1
    a.halt()
    _udhandler(a)
    e = {'end': 'park', 'kind': '0', 'lcd_n': '1'}
    e.update(_udexp(ents))
    out.append((a, e, []))
    out.append((a, e, ['+HIT=100']))
    # EVEC = 0: a supervisor hit-able store does nothing
    a = Asm()
    a.setr(9, DATA + 0x398); a.setr(2, DATA + 0x398); a.mtsr('wp0_addr', 2)
    a.setr(2, 0xFF | WP_R | WP_W); a.mtsr('wp0_ctrl', 2)
    a.setr(1, 7); a.st(64, 1, 9); a.ld(64, 4, 9)
    a.halt()
    out.append((a, {'end': 'park', 'kind': '0', 'r4': 7, 'exc_n': '0'}, []))
    return out


@test
def uw_irq():
    """§13.3: a watched store in a user loop under timer IRQs: one WATCH per
    iteration, the result intact"""
    out = []
    for per in (61, 89, 131):
        a = Asm()
        _xsetup(a)
        a.setr(1, DATA + 0x3A0); a.mtsr('wp0_addr', 1); a.setr(1, 0xFF | WP_W | WP_U); a.mtsr('wp0_ctrl', 1)
        _ud_user(a, ss=0, mask=1)
        a.label('u')
        a.setr(9, DATA + 0x3A0); a.setr(2, 50); a.setr(1, 0)
        a.label('l'); a.alu('add', 1, 1, 2); a.st(64, 1, 9); a.st(64, 1, 9, 8); a.dec(2)
        a.fbr('ne', 2, 0, 'l', imm=True)
        a.halt()
        a.label('h')
        a.push(13); a.push(14)
        a.mfsr(13, 'ecause'); a.setr(14, X_WATCH | 1 << 9 | 1 << 11)
        a.fbr('ne', 13, 14, 'h_n')
        a.setr(14, XD); a.ld(64, 13, 14); a.inc(13); a.st(64, 13, 14)
        a.pop(14); a.pop(13); a.iret()
        a.label('h_n'); a.halt()
        out.append((a, {'end': 'park', 'kind': '0', 'r1': 1275, m(XD): 50, m(DATA + 0x3A0): 1275,
                        'm00180000': 'nonzero'}, [f'+IRQ_PERIOD={per}']))
    return out


@test
def ud_sr():
    """§13.1: WPn_ADDR keeps [31:3], WPn_CTRL [10:0]; ECAUSE is 13 bits; all
    four are 0 after reset; user MFSR / MTSR of them is PRIVILEGED"""
    a = Asm()
    _xsetup(a)
    a.setr(12, DATA); a.setr(3, 0)
    for sr in ('wp0_addr', 'wp0_ctrl', 'wp1_addr', 'wp1_ctrl'):
        a.mfsr(2, sr); a.alu('or', 3, 3, 2)
    a.st(64, 3, 12, 40); a.setr(3, 0x77); a.st(64, 3, 12, 48)
    a.setr64(1, -1)
    for i, sr in enumerate(('wp0_addr', 'wp0_ctrl', 'wp1_addr', 'wp1_ctrl', 'ecause')):
        a.mtsr(sr, 1); a.mfsr(2, sr); a.st(64, 2, 12, 8 * i)
    a.setr(1, 0)
    for sr in ('wp0_ctrl', 'wp1_ctrl'):
        a.mtsr(sr, 1)
    _ud_user(a, ss=0)
    a.label('u')
    a.mfsr(1, 'wp0_addr'); a.mtsr('wp1_ctrl', 1); a.mfsr(1, 'wp1_ctrl'); a.mtsr('wp0_addr', 1)
    a.halt()
    _uhandler(a)
    exp = {'end': 'park', 'kind': '0', m(DATA + 40): 0, m(DATA + 48): 0x77, m(DATA): 0xFFFFFFF8, m(DATA + 8): 0x7FF,
           m(DATA + 16): 0xFFFFFFF8, m(DATA + 24): 0x7FF, m(DATA + 32): 0x1FFF, m(XD + 32): 5}
    for i in range(5):
        exp[m(XLOG + 8 * i)] = X_PRIV
    return a, exp, []


@test
def ud_te_walk():
    """§13.2 / §13.3 with TE = 1 and the hardware walker: a stepped user
    store whose page the walker refills (no entry: the store re-executes
    and is the stepped one) hits WP0 by its VA (WATCH + SSTEP, EADDR = the
    VA); WP1 on another VA of the same PA does not hit; a jump to a code
    page the walker refills is stepped onto it"""
    if NOEXC or NOWALK:
        return []
    out = []
    for plus in ([], ['+LAT=1'], ['+HIT=70']):
        a = Asm()
        _xsetup(a)
        _wk_tables(a, {0x0003: _wk_pte(3, TLB_RWX), 0x0100: _wk_pte(1, TLB_V | TLB_R | TLB_W)})
        _wk_on(a)
        a.setr(1, KSP); a.setsp(1)
        a.setr(1, 0x0100_0010); a.mtsr('wp0_addr', 1); a.setr(1, 0xFF | WP_W | WP_U); a.mtsr('wp0_ctrl', 1)
        a.setr(1, 0x0001_0018); a.mtsr('wp1_addr', 1); a.setr(1, 0xFF | WP_W); a.mtsr('wp1_ctrl', 1)
        a.setr(1, 0x200); a.mtsr('status', 1)                      # TE (the next fetch walks page 0)
        _ud_user(a, ss=1)
        a.label('u')
        P = {}
        P['s9'] = a.setr(9, 0x0100_0010)
        P['st1'] = a.st(64, 9, 9)
        P['st2'] = a.st(64, 9, 9, 8)
        P['j'] = a.jmp_abs(0x30000)
        _udhandler(a)
        _place(a, 0x30000, lambda s: (s.setr(4, 5), s.inc(4), s.halt()))
        s = lambda at, nx: (X_STEP | SSTEP, at, nx, 3)
        e = {'end': 'park', 'kind': '0', 'r4': 6, m(DATA + 0x10): 0x0100_0010, m(DATA + 0x18): 0x0100_0010,
             'wk_n': (3, 99)}
        e.update(_udexp([s(P['s9'], P['st1']), (X_WATCH | SSTEP | 1 << 9 | 1 << 11, 0x0100_0010, P['st2'], 3),
                         s(P['st2'], P['j']), s(P['j'], 0x30000), s(0x30000, 0x30008), s(0x30008, 0x3000C),
                         (X_PRIV, HALT_W, 0x3000C, 3)]))
        out.append((a, e, plus))
    return out


@test
def ud_debugger():
    """§13.2: the JTAG debug module wins and the user step survives it: a
    DM breakpoint on a stepped user instruction, then DM single steps
    (across the STEP entry), then a resume — the step log is exactly the
    one without the debugger.  §13.3: with the debugger attached the user
    watchpoints are off.  (VSIM_DEBUG_EN=1 builds only)"""
    if not _os0.environ.get('VSIM_DEBUG_EN') == '1':
        return []
    out = []
    base = ud_step_basic()[0]
    a, e, _ = base
    labels = sorted(a.insn_starts)
    u = a.labels['u']
    for k, steps in ((1, 0), (1, 3), (2, 1), (5, 2), (6, 4), (8, 2)):
        bp = [p for p in labels if p >= u][k]
        e2 = dict(e); e2['dh_n'] = str(1 + steps)
        out.append((a, e2, [f'+DBG_BP0={bp:x}', f'+DBG_STEPS={steps}']))
    # attached: no WATCH
    ents = []

    def body(a):
        a.setr(9, DATA + 0x3B0); a.st(64, 9, 9)
    a, e, p = _udprog(body, ss=0, wps=[(DATA + 0x3B0, 0xFF | WP_W)])
    hp = a.insn_starts[a.insn_starts.index(a.labels['h']) - 1]
    e.update(_udexp([(X_PRIV, HALT_W, hp, 1)]))
    e['kind'] = '4'                       # (the handler's HALT debug-halts while attached)
    out.append((a, e, ['+DBG_ATTACHED']))
    return out

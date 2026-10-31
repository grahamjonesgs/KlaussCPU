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

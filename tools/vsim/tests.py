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
            out.append((_crit(), {'end': 'park', 'kind': '0', 'r9': 0}, [f'+LAT={lat}', f'+IRQ_PERIOD={per}']))
    return out


@test
def diei_irq_window_control():
    """negative control: without DI/EI the window check must see handlers"""
    return _crit(False), {'end': 'park', 'kind': '0', 'r9': 'nonzero'}, ['+LAT=3', '+IRQ_PERIOD=150']


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

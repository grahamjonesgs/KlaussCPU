"""kasm — minimal KlaussCPU encoder for RTL directed tests (tools/vsim).

Not a replacement for klausscc: it encodes exactly the fields that
pipeline_core.sv's f_decode_raw reads, so hand-written tests can be built
without the toolchain.  Field layout (ISA_ENCODING_V2_MAP.md):
  [31:30] LEN (words)  [29:26] class  [25:22] op  [11:8] rd [7:4] rs1 [3:0] rs2

Usage:
    a = Asm()
    a.setr(1, 0x1234); a.label('loop'); a.addi_s(1, 1, -1)
    a.fbr('ne', 1, 0, 'loop', imm=True); a.halt()
    a.write_mem('prog.mem')
"""

CMP_PRED = {  # (PRED, INV) for class 3 boolean / class D fused branch
    'eq': (0, 0), 'ne': (0, 1), 'lt': (1, 0), 'ge': (1, 1), 'le': (2, 0),
    'gt': (2, 1), 'ult': (3, 0), 'uge': (3, 1), 'ule': (4, 0), 'ugt': (4, 1),
}
# class-8 conditions: 0=always 1=Z 2=C 3=V 4=S 5=LT 6=LE 7=ULT 8=ULE 9=E
JCOND = {'al': (0, 0), 'z': (1, 0), 'nz': (1, 1), 'eq': (1, 0), 'ne': (1, 1),
         'c': (2, 0), 'nc': (2, 1), 'v': (3, 0), 's': (4, 0), 'lt': (5, 0),
         'ge': (5, 1), 'le': (6, 0), 'gt': (6, 1), 'ult': (7, 0), 'uge': (7, 1),
         'ule': (8, 0), 'ugt': (8, 1)}
ALU = {'add': 0, 'sub': 1, 'adc': 2, 'sbc': 3, 'and': 4, 'or': 5, 'xor': 6}
SHIFT = {'shl': 0, 'shr': 1, 'sar': 2, 'rol': 3, 'ror': 4}
SZ = {8: 0, 16: 1, 32: 2, 64: 3}


def H(length, cls):
    return (length << 30) | (cls << 26)


def u32(v):
    return v & 0xFFFFFFFF


def simm(v, bits):
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    assert lo <= v <= hi, f'{v} does not fit simm{bits}'
    return v & ((1 << bits) - 1)


class Asm:
    def __init__(self, base=0x20):
        self.base = base
        self.words = []          # list of (int | callable(addr_of_insn)->int)
        self.labels = {}
        self.data = {}           # dword address -> value (extra init)
        self.insn_starts = []

    # ------------------------------------------------------------ plumbing
    @property
    def pc(self):
        return self.base + 4 * len(self.words)

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = self.pc

    def emit(self, *ws):
        start = self.pc
        self.insn_starts.append(start)
        for w in ws:
            self.words.append(w)
        return start

    def _addr(self, tgt):
        return self.labels[tgt] if isinstance(tgt, str) else tgt

    def raw(self, *ws):
        return self.emit(*[u32(w) for w in ws])

    def dword(self, addr, value):
        assert addr % 8 == 0
        self.data[addr] = value & 0xFFFFFFFFFFFFFFFF

    def assemble(self):
        out = []
        pcs = {}
        # map word index -> instruction start pc (for pc-relative lambdas)
        starts = sorted(self.insn_starts)
        si = 0
        cur = self.base
        for i, w in enumerate(self.words):
            a = self.base + 4 * i
            while si < len(starts) and starts[si] <= a:
                cur = starts[si]
                si += 1
            out.append(u32(w(cur) if callable(w) else w))
        return out

    def write_mem(self, path, size_dw=None):
        words = self.assemble()
        dws = {}
        for i, w in enumerate(words):
            a = self.base + 4 * i
            dw = a >> 3
            lo, hi = dws.get(dw, (0, 0))
            if a & 4:
                hi = w
            else:
                lo = w
            dws[dw] = (lo, hi)
        mem = {k: (v[1] << 32) | v[0] for k, v in dws.items()}
        for a, v in self.data.items():
            assert (a >> 3) not in mem, f'data overlaps code at {a:#x}'
            mem[a >> 3] = v
        last = max(mem)
        with open(path, 'w') as f:
            # sparse: use @address records
            for k in sorted(mem):
                f.write(f'@{k:x} {mem[k]:016x}\n')
        return words

    # ------------------------------------------------------------ ALU
    def alu(self, op, rd, rs1, rs2, w=False):
        o = ALU[op]
        f = 1 if o <= 3 else 0
        return self.emit(H(1, 1) | o << 22 | f << 21 | (w << 19) | rd << 8 | rs1 << 4 | rs2)

    def alui(self, op, rd, rs1, imm, sgn=True, w=False):
        o = ALU[op]
        f = 1 if o <= 3 else 0
        return self.emit(H(2, 2) | o << 22 | f << 21 | sgn << 20 | (w << 19) | rd << 8 | rs1 << 4,
                         u32(imm))

    def alui_s(self, op, rd, rs1, imm8, sgn=True):
        o = ALU[op]
        f = 1 if o <= 3 else 0
        v = simm(imm8, 8) if sgn else imm8 & 0xFF
        return self.emit(H(1, 2) | o << 22 | f << 21 | sgn << 20 | v << 12 | rd << 8 | rs1 << 4)

    def addi_s(self, rd, rs1, imm8):
        return self.alui_s('add', rd, rs1, imm8)

    def mov(self, rd, rs):            # COPY = OR rd, rs, rs
        return self.alu('or', rd, rs, rs)

    def setr(self, rd, imm32, sgn=False):
        return self.emit(H(2, 2) | 15 << 22 | sgn << 20 | rd << 8, u32(imm32))

    def setr64(self, rd, imm64):
        return self.emit(H(3, 2) | 15 << 22 | rd << 8, u32(imm64), u32(imm64 >> 32))

    def leapc(self, rd, tgt):
        return self.emit(H(2, 2) | 14 << 22 | rd << 8,
                         lambda pc: u32(self._addr(tgt) - pc))

    # ------------------------------------------------------------ compare
    def cmp(self, rs1, rs2, w=False):
        return self.emit(H(1, 3) | (w << 19) | rs1 << 4 | rs2)

    def cmpi(self, rs1, imm, w=False):
        return self.emit(H(2, 3) | 1 << 20 | (w << 19) | rs1 << 4, u32(imm))

    def cmpi_s(self, rs1, imm8):
        return self.emit(H(1, 3) | 1 << 20 | simm(imm8, 8) << 12 | rs1 << 4)

    def setcc(self, cond, rd, rs1, rs2, w=False):
        p, inv = CMP_PRED[cond]
        return self.emit(H(1, 3) | p << 23 | inv << 22 | 1 << 21 | (w << 19) | rd << 8 | rs1 << 4 | rs2)

    # ------------------------------------------------------------ shift/unary
    def shift_n(self, op, rd, rs1, n, f=False):
        return self.emit(H(1, 4) | SHIFT[op] << 22 | 1 << 21 | n << 15 | f << 14 | rd << 8 | rs1 << 4)

    def shift_r(self, op, rd, rs1, rs2, f=False):
        return self.emit(H(1, 4) | SHIFT[op] << 22 | f << 14 | rd << 8 | rs1 << 4 | rs2)

    def inc(self, rd, rs1=None):
        rs1 = rd if rs1 is None else rs1
        return self.emit(H(1, 5) | 14 << 22 | 1 << 19 | rd << 8 | rs1 << 4)

    def dec(self, rd, rs1=None):
        rs1 = rd if rs1 is None else rs1
        return self.emit(H(1, 5) | 15 << 22 | 1 << 19 | rd << 8 | rs1 << 4)

    # ------------------------------------------------------------ memory
    def ld(self, size, rd, rs1, off=None, sgn=False, mode=None):
        """load: off None -> [rs1]; small off -> short rs1+simm8<<size; else 2-word."""
        msz = SZ[size]
        if off is None:
            return self.emit(H(1, 6) | msz << 24 | sgn << 23 | 0 << 21 | rd << 8 | rs1 << 4)
        if off % (1 << msz) == 0 and -128 <= off >> msz <= 127 and mode != 'long':
            return self.emit(H(1, 6) | msz << 24 | sgn << 23 | 1 << 21 |
                             simm(off >> msz, 8) << 12 | rd << 8 | rs1 << 4)
        return self.emit(H(2, 6) | msz << 24 | sgn << 23 | 1 << 21 | rd << 8 | rs1 << 4, u32(off))

    def ld_rr(self, size, rd, rs1, rs2, sgn=False):
        return self.emit(H(1, 6) | SZ[size] << 24 | sgn << 23 | 3 << 21 | rd << 8 | rs1 << 4 | rs2)

    def ld_abs(self, size, rd, addr, sgn=False):
        return self.emit(H(2, 6) | SZ[size] << 24 | sgn << 23 | 2 << 21 | rd << 8, u32(addr))

    def st(self, size, rdata, rs1, off=None, mode=None):
        msz = SZ[size]
        if off is None:
            return self.emit(H(1, 7) | msz << 24 | 0 << 21 | rdata << 8 | rs1 << 4)
        if off % (1 << msz) == 0 and -128 <= off >> msz <= 127 and mode != 'long':
            return self.emit(H(1, 7) | msz << 24 | 1 << 21 |
                             simm(off >> msz, 8) << 12 | rdata << 8 | rs1 << 4)
        return self.emit(H(2, 7) | msz << 24 | 1 << 21 | rdata << 8 | rs1 << 4, u32(off))

    def st_rr(self, size, rdata, rs1, rs2):
        return self.emit(H(1, 7) | SZ[size] << 24 | 3 << 21 | rdata << 8 | rs1 << 4 | rs2)

    def st_abs(self, size, rdata, addr):
        return self.emit(H(2, 7) | SZ[size] << 24 | 2 << 21 | rdata << 8, u32(addr))

    # ------------------------------------------------------------ control
    def jmp(self, tgt, cond='al'):
        c, inv = JCOND[cond]
        return self.emit(lambda pc: H(1, 8) | 1 << 24 | c << 19 | inv << 18 |
                         simm((self._addr(tgt) - pc) >> 2, 18))

    def call(self, tgt, cond='al'):
        c, inv = JCOND[cond]
        return self.emit(lambda pc: H(1, 8) | 1 << 25 | 1 << 24 | c << 19 | inv << 18 |
                         simm((self._addr(tgt) - pc) >> 2, 18))

    def jmp_abs(self, tgt, cond='al', link=False):
        c, inv = JCOND[cond]
        return self.emit(H(2, 8) | link << 25 | c << 19 | inv << 18,
                         lambda pc: u32(self._addr(tgt)))

    def jmpr(self, rs2, cond='al', link=False):
        c, inv = JCOND[cond]
        return self.emit(H(1, 8) | link << 25 | 1 << 23 | c << 19 | inv << 18 | rs2)

    def fbr(self, cond, rs1, rs2, tgt, imm=False, w=False, pred=None, inv=None):
        """fused compare-and-branch.  rs2 is a register, or simm4 when imm.
        pred/inv override the CMP_PRED mapping (raw PRED 5-7 = W forms)."""
        p, i = CMP_PRED[cond] if cond else (pred, inv)
        rhs = simm(rs2, 4) if imm else rs2
        return self.emit(lambda pc: H(1, 0xD) | p << 23 | i << 22 | imm << 21 |
                         simm((self._addr(tgt) - pc) >> 2, 13) << 8 | rs1 << 4 | rhs)

    def ret(self):
        return self.emit(H(1, 9) | 6 << 22)

    def iret(self):
        return self.emit(H(1, 9) | 7 << 22)

    def push(self, rs1):
        return self.emit(H(1, 9) | 0 << 22 | rs1 << 4)

    def pop(self, rd):
        return self.emit(H(1, 9) | 2 << 22 | rd << 8)

    def pushv(self, imm32):
        return self.emit(H(2, 9) | 1 << 22, u32(imm32))

    def getsp(self, rd):
        return self.emit(H(1, 9) | 3 << 22 | rd << 8)

    def setsp(self, rs1):
        return self.emit(H(1, 9) | 4 << 22 | rs1 << 4)

    def addsp(self, imm32):
        return self.emit(H(2, 9) | 5 << 22, u32(imm32))

    def enter(self, n):
        return self.emit(H(1, 9) | 8 << 22 | n)

    def leave(self):
        return self.emit(H(1, 9) | 9 << 22)

    def leaveret(self):
        return self.emit(H(1, 9) | 10 << 22)

    # ------------------------------------------------------------ mul/div
    def mul(self, rd, rs1, rs2, sgn=True, high=False, w=False):
        return self.emit(H(1, 0xA) | 0 << 24 | sgn << 23 | high << 22 | w << 19 | rd << 8 | rs1 << 4 | rs2)

    def mulv(self, rd, rs1, imm32, sgn=True):
        return self.emit(H(2, 0xA) | sgn << 23 | rd << 8 | rs1 << 4, u32(imm32))

    def div(self, rd, rs1, rs2, sgn=True):
        return self.emit(H(1, 0xA) | 1 << 24 | sgn << 23 | rd << 8 | rs1 << 4 | rs2)

    def mod(self, rd, rs1, rs2, sgn=True):
        return self.emit(H(1, 0xA) | 2 << 24 | sgn << 23 | rd << 8 | rs1 << 4 | rs2)

    # ------------------------------------------------------------ system
    def sys(self, op, rs1=0):
        return self.emit(H(1, 0xB) | op << 16 | rs1 << 4)

    def nop(self):  return self.sys(0)
    def halt(self): return self.sys(1)
    def wait(self): return self.sys(2)
    def trap(self): return self.sys(4)

    # INT_MASK (0xF00F_0000) helpers via a scratch register
    def set_int_mask(self, val, tmp=14, tmp2=13):
        self.setr(tmp, 0xF00F0000)
        self.setr(tmp2, val)
        self.st(64, tmp2, tmp)

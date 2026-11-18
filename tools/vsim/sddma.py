"""SoC programs for the SD DMA engine and the cache range operations
(SD DMA spec v1.2; SD_NATIVE_PLAN.md §8, HW_PERF_PLAN.md §7) under tb_soc,
whose sd_card_sync card starts in the transfer state, 4-bit.

The card's data: dword k of card block b reads as P(b, k) = (b << 32) |
(k ^ 0xA5A50000) unless written; a written dword is compared with P (the
card's wr_bad).  Programs check DDR through the CPU (after the range
invalidate) and leave an error count in a register; the runner checks the
registers, the card's counters and tb_soc's checkers.

Each test is (name, Asm, expected regs, plusargs, log checks); see run.py
socsddma.
"""
from kasm import Asm

SD, CC, IRQ = 0xF0000000, 0xF0050000, 0xF00F0000
SDM_CMD, SDM_ARG, SDM_BLK, SDM_STATUS, SDM_IRQEN, SDM_CFG, SDM_FIFO, SDM_DMA, SDM_BLKDONE = \
    0x100, 0x108, 0x120, 0x128, 0x130, 0x140, 0x148, 0x150, 0x158
CACHE_CTRL, CACHE_STATUS, OP_ADDR, OP_LEN, L2_CTRL = 0x00, 0x10, 0x30, 0x38, 0x28
FLUSH, INVAL, ICINV, RCLEAN, RINV, RIINV = 2, 4, 8, 0x10, 0x20, 0x40
ST_DATA_BUSY, ST_DMA_BUSY, ST_DATA_DONE, ST_DMA_ERR = 1 << 1, 1 << 5, 1 << 9, 1 << 27
ERRS = 0x0FFF0000
RT_R1, RT_R1B = 1, 2


def P(b, k):
    return ((b & 0xFFFFFFFF) << 32) | ((k ^ 0xA5A50000) & 0xFFFFFFFF)


class SdAsm(Asm):
    """r15 = the SD block, r14 = the cache controller; r1..r4 scratch for
    the helpers; r12 = the error count of the checks"""
    _n = 0

    def _lab(self, s):
        SdAsm._n += 1
        return f'{s}_{SdAsm._n}'

    def prologue(self, div=1, w4=True):
        self.setr(15, SD); self.setr(14, CC); self.setr(12, 0)
        self.setr(1, (1 << 18) | (1 << 17) | div); self.st(64, 1, 15, 0)      # SD mode, power, CLK
        self.setr(1, 1 if w4 else 0); self.st(64, 1, 15, SDM_CFG)
        return self

    def wr(self, base, off, val):
        self.setr64(1, val) if val >> 32 else self.setr(1, val)
        self.st(64, 1, base, off)

    def poll(self, base, off, mask, want_set):
        """wait until (reg & mask) != 0 (want_set) / == 0"""
        lab = self._lab('poll')
        self.setr(2, mask)
        self.label(lab); self.ld(64, 1, base, off); self.alu('and', 1, 1, 2)
        self.fbr('eq' if want_set else 'ne', 1, 0, lab, imm=True)

    def mnt_wait(self):
        self.poll(14, CACHE_STATUS, 1, False)

    def cache_op(self, ctrl, addr=None, ln=None, wait=True):
        if addr is not None:
            self.wr(14, OP_ADDR, addr); self.wr(14, OP_LEN, ln)
        self.wr(14, CACHE_CTRL, ctrl)
        if wait:
            self.mnt_wait()

    def cmd(self, idx, arg, rt=RT_R1, data=0, write=0, stop=0):
        self.wr(15, SDM_ARG, arg)
        self.wr(15, SDM_CMD, (stop << 13) | (write << 12) | (data << 11) | (rt << 8) | idx)

    def stop(self):
        """CMD12 with [13]; wait for CMD_BUSY = 0"""
        self.wr(15, SDM_STATUS, 0xFFFFFFFF)
        self.cmd(12, 0, RT_R1B, stop=1)
        self.poll(15, SDM_STATUS, 1, False)

    def dma(self, idx, blk, a, bsz, cnt, write, cfg=1, irq=None):
        """a DMA data command (the FIFO reset, DMA_ADDR, size / count, DMA_EN)"""
        self.wr(15, SDM_FIFO, 0x80000000)
        self.wr(15, SDM_STATUS, 0xFFFFFFFF)
        self.wr(15, SDM_DMA, a)
        self.wr(15, SDM_BLK, (cnt << 16) | bsz)
        self.wr(15, SDM_CFG, cfg | (1 << 16))
        if irq is not None:
            self.wr(15, SDM_IRQEN, irq)
        self.cmd(idx, blk, RT_R1, data=1, write=write)

    def wait_done(self):
        """DATA_DONE, then DMA_BUSY = 0; r3 = SDM_STATUS's error bits"""
        self.poll(15, SDM_STATUS, ST_DATA_DONE | ERRS, True)
        self.poll(15, SDM_STATUS, ST_DMA_BUSY, False)
        self.ld(64, 3, 15, SDM_STATUS); self.setr(2, ERRS); self.alu('and', 3, 3, 2)

    def fill(self, a, blk0, nblk, words=64, xor=0):
        """DDR [a, +nblk x words x 8) = P(blk0 + i, k) ^ xor through the cache"""
        la, lb = self._lab('fa'), self._lab('fb')
        self.setr(10, a); self.setr(11, blk0); self.setr(9, nblk); self.setr(6, words)
        if xor:
            self.setr64(5, xor)
        self.label(lb); self.setr(8, 0)
        self.label(la)
        self.shift_n('shl', 1, 11, 32); self.alui('xor', 2, 8, 0xA5A50000, sgn=False)
        self.alu('or', 1, 1, 2)
        if xor:
            self.alu('xor', 1, 1, 5)
        self.st(64, 1, 10, 0); self.addi_s(10, 10, 8); self.inc(8)
        self.fbr('ne', 8, 6, la)
        self.inc(11); self.dec(9); self.fbr('ne', 9, 0, lb, imm=True)

    def check(self, a, blk0, nblk, words=64, xor=0, k0=0):
        """r12 += the dwords of [a, ...) that differ from P(blk0 + i, k) ^ xor
        (k from k0 to words - 1 in each block)"""
        la, lb, lok = self._lab('ca'), self._lab('cb'), self._lab('cok')
        self.setr(10, a); self.setr(11, blk0); self.setr(9, nblk); self.setr(6, words)
        if xor:
            self.setr64(5, xor)
        self.label(lb); self.setr(8, k0)
        self.label(la)
        self.shift_n('shl', 1, 11, 32); self.alui('xor', 2, 8, 0xA5A50000, sgn=False)
        self.alu('or', 1, 1, 2)
        if xor:
            self.alu('xor', 1, 1, 5)
        self.ld(64, 2, 10, 0)
        self.fbr('eq', 1, 2, lok); self.inc(12)
        self.label(lok)
        self.addi_s(10, 10, 8); self.inc(8)
        self.fbr('ne', 8, 6, la)
        self.inc(11); self.dec(9); self.fbr('ne', 9, 0, lb, imm=True)

    def touch(self, a, n_lines, store=False, val=0):
        """load (or store val to) one dword of each of n_lines 32 B lines"""
        la = self._lab('t')
        self.setr(10, a); self.setr(9, n_lines); self.setr(6, 32)
        if store:
            self.setr64(5, val)
        self.label(la)
        if store:
            self.st(64, 5, 10, 0)
        else:
            self.ld(64, 1, 10, 0)
        self.alu('add', 10, 10, 6); self.dec(9); self.fbr('ne', 9, 0, la, imm=True)


JUNK = 0x5A5A_5A5A_0000_0000


def t_m3(write, variant='nowait'):
    """(m3) ordering, both halves.  A 32 KiB range, all dirty (the CPU's
    data over an older copy in DDR for an SD write; the CPU's stale copy for
    an SD read), CLEANed with the 4 KiB DMA buffer at its END: the walk
    (~8200 cycles) reaches the buffer's lines last, so a DMA not held off
    by it reads stale DDR (write) or has its data written over (read).  The
    data command follows the CACHE_CTRL write at once (variant 'nowait': no
    MNT_BUSY poll — the claim), after the poll ('wait'), with the clean
    queued behind another long walk ('queued': it is pending, not running,
    when the command lands), or ('last_store') with the buffer's last dword
    stored by the instruction right before the CACHE_CTRL write, that
    store a D-cache miss whose line arrives after the write (half 1: the
    walk starts only after the store is in the D-cache).
    The sequence from the CACHE_CTRL write to SDM_CMD runs twice, the first
    time with CACHE_CTRL = 0 and a CMD13: its code must be in the I-cache,
    or its fetches wait for the walk and the race never happens.  r12 =
    mismatches (read), the card's wr_bad (write), r4 = error bits."""
    a = SdAsm().prologue(div=0)
    A0, B0, N = 0x00200000, 100, 8
    A = A0 + 0x8000 - N * 512
    BB = B0 + 64 - N
    OTHER = 0x00300000
    if write:
        a.fill(A0, B0, 64, xor=JUNK)
        a.cache_op(FLUSH)                         # DDR: the old copy
        a.fill(A0, B0, 64)                        # the CPU's data, dirty
    else:
        a.fill(A0, B0, 64, xor=JUNK)              # the CPU's stale copy, dirty
    if variant == 'queued':
        a.touch(OTHER, 1024, store=True, val=9)
    # (last_store: pass 1 stores junk there, then that line is written back
    # and dropped, so pass 2's store of the data is a miss)
    a.setr64(7, JUNK if variant == 'last_store' else P(BB + N - 1, 63)); a.setr(11, A0 + 0x8000 - 8)
    cmd_d = (int(write) << 12) | (1 << 11) | (RT_R1 << 8) | (25 if write else 18)
    a.setr(3, 0); a.setr(13, (RT_R1 << 8) | 13)  # pass 1: CACHE_CTRL = 0, CMD13
    a.setr(9, 2)
    a.label('m3p')
    if variant == 'queued':
        a.wr(14, OP_ADDR, OTHER); a.wr(14, OP_LEN, 0x8000); a.st(64, 3, 14, CACHE_CTRL)
    a.wr(14, OP_ADDR, A0); a.wr(14, OP_LEN, 0x8000)
    if variant == 'last_store':
        a.st(64, 7, 11, 0)                        # the store (a miss) ...
    a.st(64, 3, 14, CACHE_CTRL)                   # ... and the clean, back to back
    if variant == 'wait':
        a.mnt_wait()
    a.wr(15, SDM_FIFO, 0x80000000); a.wr(15, SDM_STATUS, 0xFFFFFFFF)
    a.wr(15, SDM_DMA, A); a.wr(15, SDM_BLK, (N << 16) | 512); a.wr(15, SDM_CFG, 1 | (1 << 16))
    a.wr(15, SDM_ARG, BB)
    a.st(64, 13, 15, SDM_CMD)
    a.poll(15, SDM_STATUS, 1, False)              # CMD_BUSY = 0
    if variant == 'last_store':
        a.fbr('ne', 9, 2, 'm3n', imm=True)
        a.cache_op(RINV, A0 + 0x8000 - 32, 32)
        a.setr64(7, P(BB + N - 1, 63))
        a.label('m3n')
    a.setr(3, RCLEAN); a.setr(13, cmd_d)
    a.dec(9); a.fbr('ne', 9, 0, 'm3p', imm=True)
    a.wait_done(); a.mov(4, 3)
    if not write:
        a.cache_op(RINV, A, N * 512)
        a.check(A, BB, N)
    a.stop()
    a.halt()
    exp = {'r4': 0} if write else {'r4': 0, 'r12': 0}
    chk = {'host_err': 0}
    if write:
        chk.update(wr_bad=0, wr_blocks=N)
    return a, exp, ['+SDC_NAC=2'], chk


def t_range_partial():
    """partial-line ADDR / LEN and lines straddling the range's ends:
    RANGE_CLEAN of [A + 8, A + 8 + 1000) on a dirty buffer must write back
    lines A .. A + 1024 - 32 (the first and last partly in the range) and
    nothing else: the line before (A - 32) and the one after (A + 1024)
    stay dirty (DCP probes); a DMA write of 2 blocks from A then sends the
    CPU's data (wr_bad = 0).  Then RANGE_INV of [A + 520, + 1): one line
    (A + 512 .. +32) dropped, its neighbours kept valid (probes)."""
    a = SdAsm().prologue(div=0)
    A, B0 = 0x00220400, 500
    a.fill(A - 512, B0 - 1, 4, xor=JUNK)
    a.cache_op(FLUSH)
    a.fill(A - 512, B0 - 1, 4)                    # all four blocks dirty with P
    a.cache_op(RCLEAN, A + 8, 1000)
    a.dma(25, B0, A, 512, 2, 1)
    a.wait_done(); a.mov(4, 3)
    a.stop()
    a.cache_op(RINV, A + 520, 1)
    a.halt()
    plus = [f'+DCP0={A - 32:x}', f'+DCP1={A:x}', f'+DCP2={A + 1024 - 32:x}', f'+DCP3={A + 1024:x}',
            f'+DCP4={A + 512:x}', f'+DCP5={A + 512 - 32:x}', f'+DCP6={A + 512 + 32:x}']
    want = {0: (1, 1), 1: (1, 0), 2: (1, 0), 3: (1, 1), 4: (0, 0), 5: (1, 0), 6: (1, 0)}
    return a, {'r4': 0}, plus, {'wr_bad': 0, 'host_err': 0, 'dcp': want}


def t_range_inv():
    """RANGE_INV: (1) a DMA read into a buffer the CPU has cached (clean):
    after RANGE_INV of its first 544 bytes (17 lines; ADDR, LEN not line
    multiples) the CPU sees the card's data there and its own stale copy
    in the rest (not invalidated: the op is precise); (2) RANGE_INV of a
    dirty line writes it back first (spec v1.2): the CPU's store survives.
    (3) LEN = 0: a no-op (MNT_BUSY stays 0, the dirty line stays)."""
    a = SdAsm().prologue(div=0)
    A, B0 = 0x00230000, 700
    a.fill(A, B0, 2, xor=JUNK)
    a.cache_op(FLUSH)                             # cached, clean, = DDR
    a.cache_op(RCLEAN, A, 1024)                   # (the clean before the DMA)
    a.dma(18, B0, A, 512, 2, 0)
    a.wait_done(); a.mov(4, 3)
    a.stop()
    a.cache_op(RINV, A + 4, 540)                  # lines A .. A + 544
    a.check(A, B0, 1)                             # 544 B: the card's
    a.check(A + 512, B0 + 1, 1, words=4)
    a.mov(13, 12); a.setr(12, 0)
    a.check(A + 544, B0 + 1, 1, xor=JUNK, k0=4)   # the rest: the CPU's stale copy
    a.mov(7, 12); a.setr(12, 0)
    # (2) a dirty line, RANGE_INV: written back, then dropped
    C = 0x00231000
    a.setr64(1, 0x1122334455667788); a.setr(10, C); a.st(64, 1, 10, 0)
    a.cache_op(RINV, C, 8)
    a.ld(64, 8, 10, 0)
    # (3) LEN = 0
    D = 0x00232000
    a.setr64(1, 0x99); a.setr(10, D); a.st(64, 1, 10, 0)
    a.wr(14, OP_ADDR, D); a.wr(14, OP_LEN, 0); a.wr(14, CACHE_CTRL, RINV | RCLEAN)
    a.ld(64, 9, 14, CACHE_STATUS)
    a.halt()
    # (C: written back by the RANGE_INV — the load gets the store's value
    # from DDR, refetched clean)
    return a, {'r4': 0, 'r13': 0, 'r7': 0, 'r8': 0x1122334455667788, 'r9': 0}, \
        [f'+DCP0={C:x}', f'+DCP1={D:x}'], {'host_err': 0, 'dcp': {0: (1, 0), 1: (1, 1)},
                                           'walks': ['whole', 'range', 'range', 'range']}


def t_range_queue(phase):
    """ops queued like FLUSH (range ops: a single go, no whole-walk re-arm).
    phase 1: a long RANGE_CLEAN (32 KiB all dirty, 1024 sets) is running
    when a RANGE_INV of A is written; ADDR / LEN are then changed to B
    before the walk ends: the queued op keeps A (copied when accepted) — A
    dropped, B untouched (still dirty).  phase 2: the same long walk
    running, a RANGE_CLEAN of X queued, a second RANGE_CLEAN (of Cx): no
    second slot, so both become one whole-cache walk of the stronger kind (a
    FLUSH: Cx cleaned, kept valid; B cleaned too)."""
    a = SdAsm().prologue(div=0)
    # (sets >= 512: away from the program's own lines — code is fetched
    # through the D-cache)
    R, A, B, Cx, X = 0x00300000, 0x00244000, 0x00254100, 0x00264200, 0x00274400
    a.touch(R, 1024, store=True, val=7)           # 32 KiB dirty: one way of every set
    a.touch(A, 4, store=True, val=1); a.touch(B, 4, store=True, val=2)
    # (twice, the first time with CACHE_CTRL = 0: the code must be in the
    # I-cache, or its fetches wait for the walk)
    a.setr(3, 0); a.setr(7, 0); a.setr(9, 2)
    a.label('rq1')
    a.wr(14, OP_ADDR, R); a.wr(14, OP_LEN, 0x8000); a.st(64, 3, 14, CACHE_CTRL)   # a long walk ...
    a.wr(14, OP_ADDR, A); a.wr(14, OP_LEN, 128); a.st(64, 7, 14, CACHE_CTRL)      # ... this one queued
    a.wr(14, OP_ADDR, B); a.wr(14, OP_LEN, 128)   # (must not retarget it)
    a.mnt_wait()
    a.setr(3, RCLEAN); a.setr(7, RINV); a.dec(9); a.fbr('ne', 9, 0, 'rq1', imm=True)
    if phase == 1:
        a.halt()
        plus = [f'+DCP0={A:x}', f'+DCP1={A + 96:x}', f'+DCP2={B:x}', f'+DCP3={B + 96:x}']
        return a, {}, plus, {'dcp': {0: (0, 0), 1: (0, 0), 2: (1, 1), 3: (1, 1)},
                             'walks': ['range', 'range']}
    a.touch(R, 1024, store=True, val=8)
    a.touch(Cx, 2, store=True, val=3); a.touch(X, 2, store=True, val=4)
    # (twice: the first pass, CACHE_CTRL = 0, brings the code into the
    # I-cache — an I-fill during the walk would wait for it)
    a.setr(7, 0); a.setr(13, 2)
    a.label('rq2')
    for addr, ln in ((R, 0x8000), (X, 64), (Cx, 64)):     # the slot is taken at Cx: one whole walk
        a.wr(14, OP_ADDR, addr); a.wr(14, OP_LEN, ln); a.st(64, 7, 14, CACHE_CTRL)
    a.mnt_wait()
    a.setr(7, RCLEAN); a.dec(13); a.fbr('ne', 13, 0, 'rq2', imm=True)
    a.halt()
    plus = [f'+DCP0={A:x}', f'+DCP2={B:x}', f'+DCP3={Cx:x}', f'+DCP4={R:x}', f'+DCP5={X:x}']
    return a, {}, plus, {'dcp': {0: (0, 0), 2: (1, 0), 3: (1, 0), 4: (1, 0), 5: (1, 0)},
                         'walks': ['range', 'range', 'range', 'whole']}


def t_range_prec():
    """the range walk's precision within a set: lines of the range share
    their sets with dirty lines of another tag (A2 = A + 32 KiB, the other
    way); RANGE_CLEAN then RANGE_INV of A's 8 lines must leave every A2 line
    valid and dirty (sets >= 512, away from the program's lines)."""
    a = SdAsm().prologue(div=0)
    A = 0x00414000
    A2 = A + 0x8000
    a.touch(A, 8, store=True, val=1); a.touch(A2, 8, store=True, val=2)
    a.cache_op(RCLEAN, A, 256)
    a.cache_op(RINV, A, 256)
    a.halt()
    plus = [f'+DCP0={A:x}', f'+DCP1={A + 224:x}', f'+DCP2={A2:x}', f'+DCP3={A2 + 224:x}']
    return a, {}, plus, {'dcp': {0: (0, 0), 1: (0, 0), 2: (1, 1), 3: (1, 1)}}


def t_range_combo():
    """(1) a range op over more than 64 KiB: a whole-cache walk (RANGE_CLEAN
    -> FLUSH: the lines stay valid, clean); (2) FLUSH | RANGE_INV: the
    whole op wins, no range walk (the range's line stays valid); (3)
    INVALIDATE | RANGE_CLEAN: the INVALIDATE (all dropped); (4) ICACHE_INV
    | RANGE_INV: both (the range dropped, the rest kept)."""
    a = SdAsm().prologue(div=0)
    A, B, Cx, D = 0x00284000, 0x00294400, 0x002A4800, 0x002B4C00   # (distinct sets, >= 512)
    a.touch(Cx, 2, store=True, val=3)
    a.cache_op(INVAL | RCLEAN, Cx, 64)            # (3) INVALIDATE: everything dropped
    a.touch(A, 2, store=True, val=1)
    a.cache_op(RCLEAN, A, 0x10020)                # (1) 64 KiB + 32: whole FLUSH
    a.touch(B, 2, store=True, val=2)
    a.cache_op(FLUSH | RINV, B, 64)               # (2) FLUSH only
    a.touch(D, 2, store=True, val=4); a.touch(D + 0x1000, 1, store=True, val=5)
    a.cache_op(ICINV | RINV, D, 64)               # (4) both
    a.halt()
    plus = [f'+DCP0={A:x}', f'+DCP1={B:x}', f'+DCP2={Cx:x}', f'+DCP3={D:x}', f'+DCP4={D + 0x1000:x}']
    return a, {}, plus, {'dcp': {0: (1, 0), 1: (1, 0), 2: (0, 0), 3: (0, 0), 4: (1, 1)},
                         # (one walk per op: before review 1 a whole op walked twice)
                         'walks': ['whole', 'whole', 'whole', 'range']}


def _code_dwords(base, val):
    """F: r5 = val; RET — as dwords at base (zero-padded)"""
    f = Asm(base=base)
    f.setr(5, val); f.ret()
    w = f.assemble() + [0]
    return [(w[i] | (w[i + 1] << 32)) for i in range(0, len(w) - 1, 2)]


def t_iinv(variant, ln=512):
    """RANGE_IINV (CACHE_CTRL[6]): F (r5 = 1; RET) at X runs from a loop's
    CALL (I-cache line, NLB, BTIC entry for the CALL's target window); F'
    (r5 = 2) goes to the card by DMA from a staging buffer, comes back by DMA
    over X; RANGE_INV X in the D-cache (I-fills come through it); then:
      none   — no I-cache op: the loop still runs F (r6 = 4 + 2: the
               control, the test sees stale code);
      iinv   — RANGE_IINV [X, +ln): F' (r6 = 4 + 4);
      other  — RANGE_IINV of another range: the I-cache line of X stays
               (r6 = 6: only the range's lines are dropped);
      icinv  — ICACHE_INV: F'.
    The same CALL site runs before and after (the BTIC entry); tb_soc's BTIC
    checks (btic_unc / btic_stale) gate every run; RKSWEEP gives the sweep."""
    a = SdAsm().prologue(div=0)
    X, S, K = 0x8000, 0x9000, 50
    for i, d in enumerate(_code_dwords(X, 1)):
        a.dword(X + 8 * i, d)
    a.setr(6, 0); a.setr(13, 0)
    a.label('run'); a.setr(9, 4)
    a.label('lp'); a.call(X); a.alu('add', 6, 6, 5); a.dec(9); a.fbr('ne', 9, 0, 'lp', imm=True)
    a.fbr('ne', 13, 0, 'after1', imm=True)
    # F' to the card through a staging buffer, then back over X
    a.setr(10, S)
    for i, d in enumerate(_code_dwords(X, 2)):
        a.setr64(1, d); a.st(64, 1, 10, 8 * i)
    a.cache_op(RCLEAN, S, 512)
    a.dma(24, K, S, 512, 1, 1); a.wait_done(); a.mov(4, 3)
    # (clean before the device writes X: the boot copy left X dirty in the
    # D-cache, and RANGE_INV would write that line back over the DMA's data)
    a.cache_op(RCLEAN, X, 512)
    a.dma(17, K, X, 512, 1, 0); a.wait_done(); a.alu('or', 4, 4, 3)
    a.cache_op(RINV, X, 512)
    if variant == 'iinv':
        a.cache_op(RIINV, X, ln)
    elif variant == 'other':
        a.cache_op(RIINV, X + 0x1000, ln)
    elif variant == 'icinv':
        a.wr(14, CACHE_CTRL, ICINV)
    a.setr(13, 1); a.jmp('run')
    a.label('after1')
    a.mov(7, 5)
    a.halt()
    new = variant in ('iinv', 'icinv')
    return a, {'r4': 0, 'r6': 12 if new else 8, 'r7': 2 if new else 1}, \
        [], {'host_err': 0}


def t_range_two():
    """a range of 1536 lines (48 KiB): every set visited once, the first 512
    of them holding two lines of the range (tags T and T + 1, the
    r_rw_two / r_rw_tag1 path).  48 KiB dirty with the CPU's data over an
    older copy in DDR, RANGE_CLEAN of all of it, then a DMA write of the
    last 8 KiB (lines L0 + 1024 + ..., the second tag in their sets): the
    card must get the CPU's data (wr_bad = 0)."""
    a = SdAsm().prologue(div=0)
    A, B0, N = 0x00340000, 1000, 96
    a.fill(A, B0, N, xor=JUNK)
    a.cache_op(FLUSH)                             # DDR: the old data
    a.fill(A, B0, N)                              # dirty (the first 512 sets hold two lines each)
    a.cache_op(RCLEAN, A, N * 512, wait=False)
    a.dma(25, B0, A, 512, N, 1)
    a.wait_done(); a.mov(4, 3)
    a.stop()
    a.halt()
    return a, {'r4': 0}, [], {'wr_bad': 0, 'host_err': 0, 'wr_blocks': N}


def t_iinv_snoop(then_icinv=False):
    """a store's I-cache snoop during a RANGE_IINV sweep: G (r5 = 3; RET) at
    Y is cached and run; a RANGE_IINV of 64 lines elsewhere (a 64-cycle
    sweep); the very next instructions patch G to r5 = 4 (CPU stores: the
    snoop clears Y's line, outside the range — it must land, the sweep
    waits a cycle for it); G must then return 4.  then_icinv: instead, X
    is rewritten by DMA (as t_iinv) after the range sweep, and an ICACHE_INV
    must still drop it (the range sweep left other lines valid, so the
    whole-cache sweep must not be skipped)."""
    a = SdAsm().prologue(div=0)
    X, Y, S, K = 0x8000, 0xA000, 0x9000, 51
    for i, d in enumerate(_code_dwords(Y, 3)):
        a.dword(Y + 8 * i, d)
    for i, d in enumerate(_code_dwords(X, 1)):
        a.dword(X + 8 * i, d)
    a.call(Y); a.call(Y); a.mov(6, 5)              # G cached
    a.call(X); a.call(X)
    if not then_icinv:
        a.setr(10, Y); g = _code_dwords(Y, 4)
        a.setr64(7, g[0]); a.setr64(8, g[1])
        a.wr(14, OP_ADDR, 0x20000); a.wr(14, OP_LEN, 64 * 32); a.setr(1, RIINV)
        a.st(64, 1, 14, CACHE_CTRL)                # the sweep starts as this store completes ...
        a.st(64, 7, 10, 0); a.st(64, 8, 10, 8)     # ... and these snoop Y during it
        a.mnt_wait()
        a.call(Y); a.mov(7, 5)
        a.halt()
        return a, {'r6': 3, 'r7': 4}, [], {}
    a.setr(10, S)
    for i, d in enumerate(_code_dwords(X, 2)):
        a.setr64(1, d); a.st(64, 1, 10, 8 * i)
    a.cache_op(RCLEAN, S, 512)
    a.dma(24, K, S, 512, 1, 1); a.wait_done(); a.mov(4, 3)
    a.cache_op(RCLEAN, X, 512)
    a.dma(17, K, X, 512, 1, 0); a.wait_done(); a.alu('or', 4, 4, 3)
    a.cache_op(RINV, X, 512)
    a.cache_op(RIINV, 0x20000, 64 * 32)            # a range sweep elsewhere
    a.wr(14, CACHE_CTRL, ICINV)                   # then the whole I-cache
    a.call(X); a.mov(7, 5)
    a.halt()
    return a, {'r4': 0, 'r7': 2}, [], {'host_err': 0}


def t_irq():
    """the SD interrupt through the SoC's source-2 cascade: SDM_IRQ_EN =
    DMA_ERR | DATA_DONE, INT2_EN = SD; a refused DMA (a range leaving DDR)
    raises INT2_PENDING[1] / INT_PENDING[2] (r5, r6) with SDM_STATUS[27]
    (r7); W1C drops it (r8); a DMA read's DATA_DONE raises it again (r9),
    only once its data is in DDR (r12 = mismatches read right then, after
    a RANGE_INV)."""
    a = SdAsm().prologue(div=0)
    A, B0 = 0x00380000, 1200
    a.setr(13, IRQ)
    a.wr(13, 0x50, 2)                             # INT2_EN = SD
    a.wr(15, SDM_IRQEN, ST_DMA_ERR | ST_DATA_DONE)
    a.wr(15, SDM_DMA, 0x07FFFE00); a.wr(15, SDM_BLK, (2 << 16) | 512); a.wr(15, SDM_CFG, 1 | (1 << 16))
    a.cmd(18, B0, RT_R1, data=1)
    a.nop(); a.nop(); a.nop(); a.nop()
    # (r4 = INT2_PENDING | (INT_PENDING & 4) << 4 | INT2_PENDING after the
    # W1C << 8: the check helpers use r1, r2, r5, r6, r8 .. r12)
    a.ld(64, 4, 13, 0x58); a.ld(64, 6, 13, 0x08); a.setr(1, 4); a.alu('and', 6, 6, 1)
    a.shift_n('shl', 6, 6, 4); a.alu('or', 4, 4, 6)
    a.ld(64, 7, 15, SDM_STATUS); a.setr(1, ST_DMA_ERR); a.alu('and', 7, 7, 1)
    a.wr(15, SDM_STATUS, 0xFFFFFFFF)
    a.nop(); a.nop(); a.nop()
    a.ld(64, 8, 13, 0x58); a.shift_n('shl', 8, 8, 8); a.alu('or', 4, 4, 8)
    a.cache_op(RCLEAN, A, 1024)
    a.dma(18, B0, A, 512, 2, 0)
    a.poll(13, 0x58, 2, True)                     # the interrupt line
    a.cache_op(RINV, A, 1024)
    a.check(A, B0, 2)
    a.ld(64, 9, 13, 0x58)
    a.stop()
    a.halt()
    return a, {'r4': 0x42, 'r7': ST_DMA_ERR, 'r9': 2, 'r12': 0}, [], {'host_err': 0}


def t_inv_hit(whole=False, wait=False):
    """a load right after a RANGE_INV (or a whole INVALIDATE) without the
    MNT_BUSY poll, while the D-cache is still busy with an earlier store's
    miss (so the walk is pending, not running): does the load see the
    invalidated (stale) line?  B is cached with an old copy; a DMA put the
    card's data in DDR; then a store that misses, the invalidate, and a load
    of B at once (twice: warm code, the first pass with CACHE_CTRL = 0).
    r7 = the load's value, r6 = the card's.  Characterisation (see run)."""
    a = SdAsm().prologue(div=0)
    B, K, X = 0x00424000, 1300, 0x00500000
    a.fill(B, K, 1, xor=JUNK)                     # the old copy, cached
    a.cache_op(RCLEAN, B, 512)
    a.dma(17, K, B, 512, 1, 0); a.wait_done(); a.mov(4, 3)
    a.wr(14, OP_ADDR, B); a.wr(14, OP_LEN, 512)
    a.setr(3, 0); a.setr(9, 2); a.setr(10, B); a.setr(11, X); a.setr(13, 0)
    a.label('ih')
    a.st(64, 9, 11, 0)                            # a store that misses (a new line each pass)
    a.st(64, 3, 14, CACHE_CTRL)
    if wait:
        a.mnt_wait()
    a.ld(64, 7, 10, 0)                            # B's first dword, at once
    a.setr(1, 0x1000); a.alu('add', 11, 11, 1)
    a.setr(3, INVAL if whole else RINV); a.dec(9); a.fbr('ne', 9, 0, 'ih', imm=True)
    a.setr64(6, P(K, 0))
    a.stop()
    a.halt()
    # the load waits for the walk (the hit pipe is held off while an
    # invalidating walk is pending: RANGE_INV and, since review 1, INVALIDATE)
    return a, {'r4': 0, 'r7': P(K, 0)}, [], {'host_err': 0}


def t_conv_hit(ln):
    """(review 1, MAJOR 1) t_inv_hit with a RANGE_INV turned into a whole
    INVALIDATE (LEN > 64 KiB): the load right after it must wait too"""
    a = SdAsm().prologue(div=0)
    B, K, X = 0x00424000, 1300, 0x00500000
    a.fill(B, K, 1, xor=JUNK)
    a.cache_op(RCLEAN, B, 512)
    a.dma(17, K, B, 512, 1, 0); a.wait_done(); a.mov(4, 3)
    a.wr(14, OP_ADDR, B); a.wr(14, OP_LEN, ln)
    a.setr(3, 0); a.setr(9, 2); a.setr(10, B); a.setr(11, X); a.setr(13, 0)
    a.label('ih')
    a.st(64, 9, 11, 0)
    a.st(64, 3, 14, CACHE_CTRL)
    a.ld(64, 7, 10, 0)
    a.setr(1, 0x1000); a.alu('add', 11, 11, 1)
    a.setr(3, RINV); a.dec(9); a.fbr('ne', 9, 0, 'ih', imm=True)
    a.setr64(6, P(K, 0))
    a.stop()
    a.halt()
    return a, {'r4': 0, 'r7': P(K, 0)}, [], {'host_err': 0}


def t_lost(d, first=FLUSH, second=RINV, ln=32):
    """(review 1, MAJOR 2) a store that misses keeps the D-cache busy; d NOPs
    later `first` (pending behind the miss), then `second` — a range op
    converted into a whole walk (one is pending), or a whole op.  At some d
    the second go lands in the very cycle the first's walk starts: it must
    still get a walk of its own.  B (cached, clean) must end invalid when
    either op invalidates; tb_soc's op accounting checks nothing is lost.
    (The sequence runs twice, the first time with CACHE_CTRL = 0: warm code.)"""
    a = SdAsm()
    a.setr(15, SD); a.setr(14, CC); a.setr(12, 0)
    B, X = 0x00424000, 0x00500000
    a.setr(10, B); a.ld(64, 1, 10, 0)
    a.wr(14, OP_ADDR, B); a.wr(14, OP_LEN, ln)
    a.setr(3, first); a.setr(5, second); a.setr(11, X); a.setr(9, 0x77)
    a.setr(13, 2); a.setr(7, 0); a.setr(8, 0); a.setr(6, ln)
    a.label('lp')
    a.st(64, 9, 11, 0)
    for _ in range(d):
        a.nop()
    a.st(64, 7, 14, CACHE_CTRL)
    a.st(64, 6, 14, OP_LEN)
    a.st(64, 8, 14, CACHE_CTRL)
    a.mnt_wait()
    a.setr(1, 0x1000); a.alu('add', 11, 11, 1)
    a.mov(7, 3); a.mov(8, 5)
    a.dec(13); a.fbr('ne', 13, 0, 'lp', imm=True)
    a.halt()
    inv = bool((first | second) & (INVAL | RINV))
    return a, {}, [f'+DCP0={B:x}'], {'dcp': {0: (0, 0) if inv else (1, 0)}}


def t_b2b():
    """(review 1) RANGE_CLEAN then RANGE_INV of one dirty line, CACHE_CTRL
    stores back to back: both walk; the line ends invalid"""
    a = SdAsm()
    a.setr(15, SD); a.setr(14, CC); a.setr(12, 0)
    B = 0x00424000
    a.setr(10, B); a.setr(1, 5); a.st(64, 1, 10, 0)
    a.wr(14, OP_ADDR, B); a.wr(14, OP_LEN, 32)
    a.setr(3, RCLEAN); a.setr(5, RINV)
    a.st(64, 3, 14, CACHE_CTRL); a.st(64, 5, 14, CACHE_CTRL)
    a.mnt_wait()
    a.halt()
    return a, {}, [f'+DCP0={B:x}'], {'dcp': {0: (0, 0)}, 'walks': ['range', 'range']}


def t_iinv_wrap(rs, ln, X=0x10000, ctrl=RIINV):
    """(review 1) RANGE_IINV over a range that wraps the I-cache's 64 KiB
    index (more than 64 lines: the bank mask), X (index 0) rewritten by DMA;
    ctrl may add FLUSH / INVALIDATE (RANGE_IINV is honoured with them)"""
    a = SdAsm().prologue(div=0)
    S, K = 0x9000, 50
    for i, d in enumerate(_code_dwords(X, 1)):
        a.dword(X + 8 * i, d)
    a.setr(6, 0); a.setr(13, 0)
    a.label('run'); a.setr(9, 4)
    a.label('lp'); a.call(X); a.alu('add', 6, 6, 5); a.dec(9); a.fbr('ne', 9, 0, 'lp', imm=True)
    a.fbr('ne', 13, 0, 'after1', imm=True)
    a.setr(10, S)
    for i, d in enumerate(_code_dwords(X, 2)):
        a.setr64(1, d); a.st(64, 1, 10, 8 * i)
    a.cache_op(RCLEAN, S, 512)
    a.dma(24, K, S, 512, 1, 1); a.wait_done(); a.mov(4, 3)
    a.cache_op(RCLEAN, X, 512)
    a.dma(17, K, X, 512, 1, 0); a.wait_done(); a.alu('or', 4, 4, 3)
    a.cache_op(RINV, X, 512)
    a.cache_op(ctrl, rs, ln)
    a.setr(13, 1); a.jmp('run')
    a.label('after1')
    a.mov(7, 5)
    a.halt()
    return a, {'r4': 0, 'r6': 12, 'r7': 2}, [], {'host_err': 0}


def t_two_wrap(A, N, a_kept=True):
    """(review 1) a RANGE_CLEAN of N blocks from A, the range's second tag
    in its sets (A = set 960: 1,536 lines wrap the index), lines just
    outside it dirty and kept dirty; a DMA write of all of it"""
    a = SdAsm().prologue(div=0)
    B0 = 1000
    a.fill(A, B0, N, xor=JUNK)
    a.cache_op(FLUSH)
    a.fill(A, B0, N)
    E = A + N * 512
    a.setr(10, A - 32); a.setr(1, 1); a.st(64, 1, 10, 0)
    a.setr(10, E); a.st(64, 1, 10, 0)
    a.cache_op(RCLEAN, A, N * 512, wait=False)
    a.dma(25, B0, A, 512, N, 1)
    a.wait_done(); a.mov(4, 3)
    a.stop()
    a.halt()
    return a, {'r4': 0}, [f'+DCP0={A-32:x}', f'+DCP1={E:x}', f'+DCP2={A:x}', f'+DCP3={E-32:x}'], \
        {'wr_bad': 0, 'host_err': 0, 'wr_blocks': N,
         # (N = 128: 64 KiB fills both ways of every set, and the store at E,
         # A's set, evicts A — written back, so the DMA still reads it)
         'dcp': {0: (1, 1), 1: (1, 1), 2: (1, 0) if a_kept else (0, 0), 3: (1, 0)}}


def t_iinv_time(ln, addr=0x10000):
    """RANGE_IINV's sweep for ln bytes from addr with the I-cache warm
    (RKSWEEP lines= cycles=): a timing figure; every LEN 32 .. 64 KiB"""
    a = SdAsm().prologue(div=0)
    a.setr(10, 0)
    a.cache_op(RIINV, addr, ln)
    a.halt()
    return a, {}, [], {}


def t_vga(lines=120, wbinv_every=1500, sd=True, blit=False, sd_write=False, span=0x80000):
    """DMA alongside VGA, the blitter and L2 walks (and the L2 review's
    question: an SD-DMA stream of writes into DDR during WBINV walks under
    VGA load).  run.py's _vga_prog traffic (640 x 480 scanout at 4 MiB, a
    strided CPU walk with dirty victims on both levels, an L2 WBINV every
    wbinv_every iterations, optionally the blitter FILLing back to back),
    plus an open-ended SD DMA at 50 MHz started first: a read stream into
    6 MiB (DMA writes into DDR) or, sd_write, a write stream from 6 MiB.
    At the end: CMD12, DMA_BUSY = 0, RANGE_INV of the buffer and the CPU
    checks every block the card delivered (r12 = mismatches; the card's
    wr_bad for a write stream).  r1 = VGA_STATUS."""
    a = SdAsm()
    a.setr(15, SD); a.setr(14, CC); a.setr(12, 0)
    SB = 0x00600000
    if sd:
        a.setr(1, (1 << 18) | (1 << 17) | 0); a.st(64, 1, 15, 0); a.wr(15, SDM_CFG, 1)
        if sd_write:
            a.fill(SB, 2000, 1024)                 # (512 KiB: more than the stream sends)
            a.cache_op(RCLEAN, SB, 1024 * 512)
        a.dma(25 if sd_write else 18, 2000, SB, 512, 0, 1 if sd_write else 0)
    a.setr(15, 0xF0110000)
    a.setr(1, 0x400000); a.st(64, 1, 15, 0x08)
    a.setr(1, 1280); a.st(64, 1, 15, 0x10)
    a.setr(1, 480); a.st(64, 1, 15, 0x38)
    a.setr(1, 0); a.st(64, 1, 15, 0x28)
    a.setr(1, 4); a.st(64, 1, 15, 0x18)
    a.setr(1, 1); a.st(64, 1, 15, 0x00)
    a.setr(10, 0x100000); a.setr(9, (lines * 3200) // 40); a.setr(8, 0); a.setr(7, wbinv_every)
    if blit:
        a.setr(13, 0xF00E0000)
        for off, v in ((0x10, 0x500000), (0x18, 1280), (0x30, 640), (0x38, 64), (0x40, 0x1234)):
            a.setr(1, v); a.st(64, 1, 13, off)
    a.label('top')
    a.alu('add', 11, 10, 8)
    a.ld(64, 2, 11, 0); a.addi_s(2, 2, 1); a.st(64, 2, 11, 0)
    a.ld(64, 3, 11, 0x1000); a.st(64, 3, 11, 0x1008)
    if blit:
        a.ld(64, 4, 13, 0x08); a.setr(6, 1); a.alu('and', 4, 4, 6)
        a.fbr('ne', 4, 0, 'noblit', imm=True)
        a.setr(4, 2); a.st(64, 4, 13, 0x08)
        a.setr(4, 1); a.st(64, 4, 13, 0x00)
        a.label('noblit')
    a.setr(6, 37 * 64); a.alu('add', 8, 8, 6)
    a.setr(6, span - 1); a.alu('and', 8, 8, 6)
    a.dec(7)
    a.fbr('ne', 7, 0, 'nowrap', imm=True)
    a.setr(7, wbinv_every)
    a.setr(1, 3); a.st(64, 1, 14, 0x28)
    a.label('wpoll')
    a.ld(64, 1, 14, 0x28); a.setr(6, 0x100); a.alu('and', 1, 1, 6)
    a.fbr('ne', 1, 0, 'wpoll', imm=True)
    a.label('nowrap')
    a.dec(9)
    a.fbr('ne', 9, 0, 'top', imm=True)
    a.ld(64, 13, 15, 0x18)                              # VGA_STATUS
    if sd:
        a.setr(15, SD)
        a.stop()
        a.poll(15, SDM_STATUS, ST_DMA_BUSY, False)
        if not sd_write:
            a.ld(64, 9, 15, SDM_BLKDONE)
            a.cache_op(RINV, SB, 0x40000)
            # check BLKDONE blocks (r9), card blocks 2000 ..
            a.setr(10, SB); a.setr(11, 2000); a.setr(6, 64)
            a.fbr('eq', 9, 0, 'vdone', imm=True)
            a.label('vb'); a.setr(8, 0)
            a.label('vk')
            a.shift_n('shl', 1, 11, 32); a.alui('xor', 2, 8, 0xA5A50000, sgn=False); a.alu('or', 1, 1, 2)
            a.ld(64, 2, 10, 0); a.fbr('eq', 1, 2, 'vok'); a.inc(12)
            a.label('vok'); a.addi_s(10, 10, 8); a.inc(8); a.fbr('ne', 8, 6, 'vk')
            a.inc(11); a.dec(9); a.fbr('ne', 9, 0, 'vb', imm=True)
            a.label('vdone')
        a.ld(64, 9, 15, SDM_BLKDONE)
    a.mov(1, 13)
    a.halt()
    return a


def t_abort(rounds=40, write=False, fifo=False, tight=False, wbinv_every=150, blit=True):
    """runtime-96's stop budget: DMA_EN = 0 (or, fifo, SDM_FIFO[31]) -> DMA_BUSY
    = 0 under stress — VGA scanout, the blitter FILLing back to back, a
    miss-heavy CPU loop with L2 WBINVs.  Each round starts an open-ended DMA
    (a read stream into 6 MiB, or a write stream from it), runs a
    pseudo-random number of traffic iterations (so the abort lands anywhere
    in a block), aborts, then polls DMA_BUSY — between traffic iterations,
    or (tight) in a tight loop as the kernel does — then CMD12 and a FIFO
    reset.  tb_soc's SDABORT line has the latencies and the per-transaction
    bus waits."""
    a = SdAsm()
    a.setr(15, SD); a.setr(14, CC); a.setr(12, 0)
    SB = 0x00600000
    a.setr(1, (1 << 18) | (1 << 17) | 0); a.st(64, 1, 15, 0); a.wr(15, SDM_CFG, 1)
    a.setr(13, 0xF0110000)                      # VGA
    for off, v in ((0x08, 0x400000), (0x10, 1280), (0x38, 480), (0x28, 0), (0x18, 4), (0x00, 1)):
        a.setr(1, v); a.st(64, 1, 13, off)
    if blit:
        a.setr(13, 0xF00E0000)
        for off, v in ((0x10, 0x500000), (0x18, 1280), (0x30, 640), (0x38, 64), (0x40, 0x1234)):
            a.setr(1, v); a.st(64, 1, 13, off)
    a.setr(10, 0x100000); a.setr(8, 0); a.setr(7, wbinv_every); a.setr64(6, 0x2545F4914F6CDD1D)
    a.setr(4, rounds)
    a.label('round')
    a.dma(25 if write else 18, 2000, SB, 512, 0, 1 if write else 0)
    # r9 = 40 + (lfsr >> 20) & 511 traffic iterations
    a.setr64(1, 6364136223846793005); a.mul(6, 6, 1); a.setr(1, 1442695040888963407 & 0xFFFFFFFF); a.alu('add', 6, 6, 1)
    a.shift_n('shr', 9, 6, 20); a.setr(1, 511); a.alu('and', 9, 9, 1); a.addi_s(9, 9, 40)
    a.label('tr')
    a.call('traffic')
    a.dec(9); a.fbr('ne', 9, 0, 'tr', imm=True)
    if fifo:
        a.wr(15, SDM_FIFO, 0x80000000)
    else:
        a.wr(15, SDM_CFG, 1)                     # DMA_EN = 0
    a.label('pw')
    if not tight:
        a.call('traffic')
    a.ld(64, 1, 15, SDM_STATUS); a.setr(2, ST_DMA_BUSY); a.alu('and', 1, 1, 2)
    a.fbr('ne', 1, 0, 'pw', imm=True)
    a.stop()
    a.wr(15, SDM_FIFO, 0x80000000)
    a.dec(4); a.fbr('ne', 4, 0, 'round', imm=True)
    a.halt()
    # one traffic iteration (_vga_prog's): a strided miss / write-back walk,
    # the blitter restarted when idle, an L2 WBINV every wbinv_every
    a.label('traffic')
    a.alu('add', 11, 10, 8)
    a.ld(64, 2, 11, 0); a.addi_s(2, 2, 1); a.st(64, 2, 11, 0)
    a.ld(64, 3, 11, 0x1000); a.st(64, 3, 11, 0x1008)
    if blit:
        a.ld(64, 2, 13, 0x08); a.setr(3, 1); a.alu('and', 2, 2, 3)
        a.fbr('ne', 2, 0, 'nb', imm=True)
        a.setr(2, 2); a.st(64, 2, 13, 0x08); a.setr(2, 1); a.st(64, 2, 13, 0x00)
        a.label('nb')
    a.setr(2, 37 * 64); a.alu('add', 8, 8, 2); a.setr(2, 0x80000 - 1); a.alu('and', 8, 8, 2)
    a.dec(7); a.fbr('ne', 7, 0, 'nw', imm=True)
    a.setr(7, wbinv_every)
    a.setr(2, 3); a.st(64, 2, 14, 0x28)
    a.label('wp'); a.ld(64, 2, 14, 0x28); a.setr(3, 0x100); a.alu('and', 2, 2, 3); a.fbr('ne', 2, 0, 'wp', imm=True)
    a.label('nw')
    a.ret()
    return a


def t_hs(write, phase=2, n=16):
    """SD high speed (SD_NATIVE_PLAN.md §9): a DMA read / write of n blocks
    at 50 MHz with the HS sampling at sample phase `phase`, the card in
    HS timing (+SDC_HS).  The synchronous card changes its lines a cycle
    after the edge it sees, so in this model every phase samples a valid
    bit (the eye is a whole clock): the test is the HS strobe / sample path
    through the SoC, not the window (tb_sd_native sweeps that).  r4 = error
    bits, r12 = mismatches (read)."""
    a = SdAsm().prologue(div=0)
    cfg = 1 | 2 | (phase << 4)
    A, B0 = 0x00280000, 4000 + 64 * phase
    if write:
        a.fill(A, B0, n)
        a.cache_op(RCLEAN, A, n * 512)
    else:
        a.cache_op(RINV, A, n * 512)
    a.dma(25 if write else 18, B0, A, 512, n, write, cfg=cfg)
    a.wait_done(); a.mov(4, 3)
    if not write:
        a.cache_op(RINV, A, n * 512)
        a.check(A, B0, n)
    a.stop()
    a.halt()
    exp = {'r4': 0} if write else {'r4': 0, 'r12': 0}
    chk = {'host_err': 0}
    if write:
        chk.update(wr_bad=0, wr_blocks=n)
    return a, exp, ['+SDC_HS'], chk


def tests():
    return [
        (f'hs_rd_p{p}', *t_hs(False, p)) for p in range(7)
    ] + [
        ('hs_wr_p2', *t_hs(True, 2)),
        ('hs_wr_p5', *t_hs(True, 5)),
    ] + [
        ('rd_m3_nowait', *t_m3(False)),
        ('rd_m3_queued', *t_m3(False, 'queued')),
        ('rd_m3_wait', *t_m3(False, 'wait')),
        ('wr_m3_nowait', *t_m3(True)),
        ('wr_m3_queued', *t_m3(True, 'queued')),
        ('wr_m3_last_store', *t_m3(True, 'last_store')),
        ('wr_m3_wait', *t_m3(True, 'wait')),
        ('range_prec', *t_range_prec()),
        ('range_partial', *t_range_partial()),
        ('range_inv', *t_range_inv()),
        ('range_queue_copy', *t_range_queue(1)),
        ('range_queue_full', *t_range_queue(2)),
        ('range_combo', *t_range_combo()),
        ('iinv_none', *t_iinv('none')),
        ('iinv', *t_iinv('iinv')),
        ('iinv_wide', *t_iinv('iinv', ln=0x3000)),
        ('iinv_other', *t_iinv('other')),
        ('iinv_icinv', *t_iinv('icinv')),
        ('iinv_snoop', *t_iinv_snoop()),
        ('iinv_then_icinv', *t_iinv_snoop(then_icinv=True)),
        ('range_two', *t_range_two()),
        ('irq', *t_irq()),
        ('inv_hit_range', *t_inv_hit()),
        ('inv_hit_whole', *t_inv_hit(whole=True)),
        ('inv_hit_wait', *t_inv_hit(wait=True)),
        ('conv_hit_big', *t_conv_hit(0x10020)),
        ('conv_hit_ok', *t_conv_hit(512)),
        ('b2b', *t_b2b()),
        ('iinv_wrap80', *t_iinv_wrap(0xF800, 0x800 + 512)),
        ('iinv_wrap101', *t_iinv_wrap(0x10000 - 0x20 * 100, 0x20 * 101)),
        ('iinv_wrap_end', *t_iinv_wrap(0x10000 - 0x20 * 3, 0x20 * 4)),
        ('iinv_flush', *t_iinv_wrap(0x10000, 64, ctrl=FLUSH | RIINV)),
        ('iinv_inval', *t_iinv_wrap(0x10000, 64, ctrl=INVAL | RIINV)),
        ('two_wrap', *t_two_wrap(0x0034F800, 96)),
        ('two_wrap_full', *t_two_wrap(0x0035F820, 128, a_kept=False)),
    ] + [
        (f'iinv_time_{ln}', *t_iinv_time(ln)) for ln in (32, 1024, 2048, 4096, 8192, 0xFFE0, 0x10000)
    ] + [
        (f'lost_{nm}_{d}', *t_lost(d, f, g)) for nm, f, g in (('fr', FLUSH, RINV), ('fi', FLUSH, INVAL), ('if', INVAL, FLUSH))
        for d in range(0, 120)
    ]

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


def t_read_coherent(order_wait):
    """SD read by DMA into a buffer the CPU has dirty in the D-cache: CLEAN
    the range (the dirty lines must reach DDR before the DMA writes it, or
    they would land over the card's data), then the data command — at once
    (order_wait False: the (m3) claim, no MNT_BUSY poll) or after the poll;
    after DATA_DONE a RANGE_INV and the CPU reads the card's data.  The
    cleaned range is 32 KiB, all dirty (a ~8200-cycle walk), with the 4 KiB
    buffer at its END: the walk reaches the buffer's lines ~7000 cycles in,
    long after the card's first block (~2200 cycles at 50 MHz), so a DMA
    not held off by the walk would write lines it has not yet written back.
    r12 = mismatches, r4 = error bits."""
    a = SdAsm().prologue(div=0)
    A0, B0, N = 0x00200000, 100, 8
    A = A0 + 0x8000 - N * 512
    a.fill(A0, B0, 64, xor=JUNK)                  # the CPU's stale copy, dirty
    a.cache_op(RCLEAN, A0, 0x8000, wait=order_wait)
    a.dma(18, B0, A, 512, N, 0)
    a.wait_done()
    a.mov(4, 3)
    a.cache_op(RINV, A, N * 512)
    a.check(A, B0, N)
    a.stop()
    a.halt()
    return a, {'r12': 0, 'r4': 0}, ['+SDC_NAC=2'], {'host_err': 0}


def t_write_coherent(order_wait, last_store=True):
    """SD write by DMA from a buffer the CPU just wrote (dirty in the
    D-cache, DDR holding an older copy after a FLUSH): RANGE_CLEAN, then the
    data command at once (no MNT_BUSY poll: m3) — the engine fetches its
    first block right away, so it must wait for the walk; the card checks
    every dword (wr_bad).  last_store: the last dword is stored by the
    instruction right before the CACHE_CTRL write (m3's first half: the
    clean covers a store still on its way to the D-cache)."""
    a = SdAsm().prologue(div=0)
    A, B0, N = 0x00210000, 300, 16
    a.fill(A, B0, N, xor=JUNK)
    a.cache_op(FLUSH)                             # DDR: the old data
    if last_store:
        a.fill(A, B0, N - 1)
        a.fill(A + (N - 1) * 512, B0 + N - 1, 1, words=63)
        a.setr64(7, P(B0 + N - 1, 63)); a.setr(13, A + N * 512 - 8)
        a.wr(14, OP_ADDR, A); a.wr(14, OP_LEN, N * 512)
        a.setr(13, A + N * 512 - 8); a.setr(1, RCLEAN)
        a.st(64, 7, 13, 0)                        # the last dword ...
        a.st(64, 1, 14, CACHE_CTRL)               # ... and the clean, back to back
        if order_wait:
            a.mnt_wait()
    else:
        a.fill(A, B0, N)
        a.cache_op(RCLEAN, A, N * 512, wait=order_wait)
    a.dma(25, B0, A, 512, N, 1)
    a.wait_done()
    a.mov(4, 3)
    a.stop()
    a.halt()
    return a, {'r4': 0}, [], {'wr_bad': 0, 'host_err': 0, 'wr_blocks': N}


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
                                           'walks': ['whole', 'whole', 'range', 'range', 'range']}


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
    a.cache_op(RCLEAN, R, 0x8000, wait=False)     # a long walk ...
    a.cache_op(RINV, A, 128, wait=False)          # ... this one queued behind it
    a.wr(14, OP_ADDR, B); a.wr(14, OP_LEN, 128)   # (must not retarget it)
    a.mnt_wait()
    if phase == 1:
        a.halt()
        plus = [f'+DCP0={A:x}', f'+DCP1={A + 96:x}', f'+DCP2={B:x}', f'+DCP3={B + 96:x}']
        return a, {}, plus, {'dcp': {0: (0, 0), 1: (0, 0), 2: (1, 1), 3: (1, 1)},
                             'walks': ['range', 'range']}
    a.touch(R, 1024, store=True, val=8)
    a.touch(Cx, 2, store=True, val=3)
    # (twice: the first pass, CACHE_CTRL = 0, brings the code into the
    # I-cache — an I-fill during the walk would wait for it)
    a.setr(7, 0); a.setr(13, 2)
    a.label('rq2')
    for addr, ln in ((R, 0x8000), (X, 64), (Cx, 64)):     # the slot is taken at Cx: one whole walk
        a.wr(14, OP_ADDR, addr); a.wr(14, OP_LEN, ln); a.st(64, 7, 14, CACHE_CTRL)
    a.mnt_wait()
    a.setr(7, RCLEAN); a.dec(13); a.fbr('ne', 13, 0, 'rq2', imm=True)
    a.halt()
    plus = [f'+DCP0={A:x}', f'+DCP2={B:x}', f'+DCP3={Cx:x}', f'+DCP4={R:x}']
    return a, {}, plus, {'dcp': {0: (0, 0), 2: (1, 0), 3: (1, 0), 4: (1, 0)},
                         'walks': ['range', 'range', 'range', 'whole']}


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
                         # (a whole op's go is a level: its walk runs twice — not a range op's)
                         'walks': ['whole', 'whole', 'whole', 'whole', 'whole', 'range']}


def tests():
    return [
        ('rd_m3_nowait', *t_read_coherent(False)),
        ('rd_wait', *t_read_coherent(True)),
        ('wr_m3_nowait', *t_write_coherent(False, last_store=False)),
        ('wr_m3_last_store', *t_write_coherent(False, last_store=True)),
        ('wr_wait', *t_write_coherent(True, last_store=False)),
        ('range_partial', *t_range_partial()),
        ('range_inv', *t_range_inv()),
        ('range_queue_copy', *t_range_queue(1)),
        ('range_queue_full', *t_range_queue(2)),
        ('range_combo', *t_range_combo()),
    ]

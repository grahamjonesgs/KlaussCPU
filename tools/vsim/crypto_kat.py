#!/usr/bin/env python3
"""tools/vsim/crypto_kat.py — MMIO-level KATs for crypto_aes / crypto_sha.

  python3 tools/vsim/crypto_kat.py          build (Verilator) + run

Generates an op list (KlaussCPU.srcs/sim_1/new/tb_crypto_mmio.sv documents the
format) with expected values from hashlib / hmac / cryptography, then runs the
bench.  Covers the legacy map (ECB, SHA, HMAC wrapper) as regression plus the
SHA RESTORE / ZERO and AES CTR / INC32 additions.  No traces are written.
"""
import hashlib, hmac, os, struct, subprocess, sys

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
SRC = os.path.join(ROOT, 'KlaussCPU.srcs', 'sources_1', 'new')
TB = os.path.join(ROOT, 'KlaussCPU.srcs', 'sim_1', 'new', 'tb_crypto_mmio.sv')
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'crypto_kat')

AES, SHA = 0, 1
M64 = (1 << 64) - 1
# AES offsets
A_CTRL, A_STATUS, A_KEY0, A_KEY1 = 0x00, 0x08, 0x10, 0x18
A_IN0, A_IN1, A_OUT0, A_OUT1 = 0x40, 0x48, 0x50, 0x58
A_CTR0, A_CTR1, A_MODE, A_CAP = 0xC0, 0xC8, 0xD0, 0xF8
# SHA offsets
S_CTRL, S_STATUS, S_BLOCK0, S_DIGEST0 = 0x00, 0x08, 0x10, 0x50
S_HIN0, S_CAP = 0xC0, 0xF8
H_CTRL, H_STATUS, H_KEY0 = 0x80, 0x88, 0x90

ops = []


def W(dev, a, d): ops.append((1, dev, a, d & M64, M64))
def R(dev, a, d, m=M64): ops.append((2, dev, a, d & m, m))
def POLL(dev, a, d, m): ops.append((3, dev, a, d, m))
def SAVE(dev, a, slot): ops.append((5, dev, a, slot, 0))
def REPLAY(dev, a, slot): ops.append((6, dev, a, slot, 0))
def MARK(n): ops.append((7, 0, 0, n, 0))


def le64(b, i=0): return struct.unpack_from('<Q', b, i)[0]


# ---------------------------------------------------------------- AES ----
KEY = bytes.fromhex('2b7e151628aed2a6abf7158809cf4f3c')
ecb = Cipher(algorithms.AES(KEY), modes.ECB()).encryptor()
def E(block): return ecb.update(block)
def xor(a, b): return bytes(x ^ y for x, y in zip(a, b))


def aes_wait(): POLL(AES, A_STATUS, 0, 1)                 # BUSY == 0
def aes_wait_done(): POLL(AES, A_STATUS, 2, 2)            # DONE == 1


def ctr_inc(c, inc32):
    if inc32:
        lo = (int.from_bytes(c[12:], 'big') + 1) & 0xFFFFFFFF
        return c[:12] + lo.to_bytes(4, 'big')
    return ((int.from_bytes(c, 'big') + 1) % (1 << 128)).to_bytes(16, 'big')


def ctr_case(name_mark, ctr0, nblk, inc32, poll_done, pts=None):
    MARK(name_mark)
    W(AES, A_MODE, 1 | (8 if inc32 else 0))
    R(AES, A_MODE, 1 | (8 if inc32 else 0))
    W(AES, A_CTR0, le64(ctr0, 0)); W(AES, A_CTR1, le64(ctr0, 8))
    R(AES, A_CTR0, le64(ctr0, 0)); R(AES, A_CTR1, le64(ctr0, 8))
    c = ctr0
    stream = b''
    for i in range(nblk):
        pt = pts[i] if pts else bytes((17 * i + j * 3 + name_mark) & 0xFF for j in range(16))
        W(AES, A_IN0, le64(pt, 0)); W(AES, A_IN1, le64(pt, 8))
        (aes_wait_done if poll_done else aes_wait)()
        ct = xor(E(c), pt)
        stream += ct
        R(AES, A_OUT0, le64(ct, 0)); R(AES, A_OUT1, le64(ct, 8))
        c = ctr_inc(c, inc32)
    R(AES, A_CTR0, le64(c, 0)); R(AES, A_CTR1, le64(c, 8))    # start + N
    return stream


MARK(1)                                    # capability registers
R(AES, A_CAP, 0x3); R(SHA, S_CAP, 0x3)
R(AES, A_MODE, 0)

MARK(2)                                    # legacy ECB encrypt (FIPS-197 / SP800-38A)
W(AES, A_KEY0, le64(KEY, 0)); W(AES, A_KEY1, le64(KEY, 8)); W(AES, A_CTRL, 4)
aes_wait()
pt = bytes.fromhex('6bc1bee22e409f96e93d7e117393172a')
W(AES, A_IN0, le64(pt, 0)); W(AES, A_IN1, le64(pt, 8)); W(AES, A_CTRL, 3)
aes_wait()
R(AES, A_STATUS, 2, 3)                     # DONE=1, BUSY=0
R(AES, A_OUT0, le64(E(pt), 0)); R(AES, A_OUT1, le64(E(pt), 8))

# 3: SP 800-38A F.5.1 CTR-AES128 vector (also cross-checks with cryptography CTR)
iv = bytes.fromhex('f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff')
sp_pt = [bytes.fromhex(h) for h in (
    '6bc1bee22e409f96e93d7e117393172a', 'ae2d8a571e03ac9c9eb76fac45af8e51',
    '30c81c46a35ce411e5fbc1191a0a52ef', 'f69f2445df4f9b17ad2b417be66c3710')]
s = ctr_case(3, iv, 4, False, poll_done=True, pts=sp_pt)
assert s == bytes.fromhex('874d6191b620e3261bef6864990db6ce9806f66b7970fdff8617187bb9fffdff'
                          '5ae4df3edbd5d35e5b4f09020db03eab1e031dda2fbe03d1792170a0f3009cee')

# 4: full 128-bit carry: bytes 8..15 = FF -> carry into byte 7; bytes 4..15 = FF
#    crosses both internal adder boundaries; all-FF wraps to zero.
s = ctr_case(4, bytes.fromhex('0001020304050607fffffffffffffffe'), 3, False, poll_done=False)
assert s == Cipher(algorithms.AES(KEY), modes.CTR(bytes.fromhex('0001020304050607fffffffffffffffe'))
                   ).encryptor().update(b''.join(bytes((17 * i + j * 3 + 4) & 0xFF for j in range(16)) for i in range(3)))
ctr_case(5, bytes.fromhex('00010203fffffffffffffffffffffffe'), 3, False, poll_done=True)
ctr_case(6, bytes.fromhex('fffffffffffffffffffffffffffffffe'), 3, False, poll_done=False)

# 7: inc32: bytes 12..15 wrap, no carry into byte 11
ctr_case(7, bytes.fromhex('cafebabe0011223344556677fffffffe'), 4, True, poll_done=True)

# 8: legacy GO after CTR (mode still on) returns the raw block, not CTR output
MARK(8)
pt = bytes.fromhex('ae2d8a571e03ac9c9eb76fac45af8e51')
W(AES, A_IN0, le64(pt, 0))
W(AES, A_MODE, 0)                          # CTR off: IN1 write must NOT start
W(AES, A_IN1, le64(pt, 8))
R(AES, A_STATUS, 2, 3)                     # nothing ran: still idle, DONE from the last CTR block
W(AES, A_CTRL, 3); aes_wait()
R(AES, A_OUT0, le64(E(pt), 0)); R(AES, A_OUT1, le64(E(pt), 8))
# and CTR again afterwards picks up its own output path
ctr_case(9, bytes.fromhex('00000000000000000000000000000000'), 2, False, poll_done=True)
W(AES, A_MODE, 0)

# ---------------------------------------------------------------- SHA ----
def sha_wait(): POLL(SHA, S_STATUS, 0, 1)
def pad(msg, prefix_len=0):
    L = (len(msg) + prefix_len) * 8
    m = msg + b'\x80'
    m += b'\x00' * ((56 - len(m) % 64) % 64)
    return m + L.to_bytes(8, 'big')
def write_block(b):
    for i in range(8): W(SHA, S_BLOCK0 + 8 * i, le64(b, 8 * i))
def check_digest(dig):
    for i in range(4): R(SHA, S_DIGEST0 + 8 * i, le64(dig, 8 * i))


msgA = bytes((i * 7 + 1) & 0xFF for i in range(150))
msgB = b'interleaved other hash ' * 5
pA, pB = pad(msgA), pad(msgB)
dA, dB = hashlib.sha256(msgA).digest(), hashlib.sha256(msgB).digest()

MARK(20)                                   # one-shot reference
W(SHA, S_CTRL, 1)
for k in range(0, len(pA), 64):
    write_block(pA[k:k + 64]); W(SHA, S_CTRL, 2); sha_wait()
R(SHA, S_STATUS, 2, 3)
check_digest(dA)

for combined, mark in ((False, 21), (True, 22)):
    MARK(mark)                             # save after block 1, hash B, restore, finish A
    W(SHA, S_CTRL, 1)
    write_block(pA[0:64]); W(SHA, S_CTRL, 2); sha_wait()
    for i in range(4): SAVE(SHA, S_DIGEST0 + 8 * i, i)
    W(SHA, S_CTRL, 1)
    for k in range(0, len(pB), 64):
        write_block(pB[k:k + 64]); W(SHA, S_CTRL, 2); sha_wait()
    check_digest(dB)
    for i in range(4): REPLAY(SHA, S_HIN0 + 8 * i, i)
    write_block(pA[64:128])
    if combined:
        W(SHA, S_CTRL, 4 | 2)              # RESTORE|START in one write
    else:
        W(SHA, S_CTRL, 4); sha_wait()
        R(SHA, S_STATUS, 0, 2)             # RESTORE clears DONE
        W(SHA, S_CTRL, 2)
    sha_wait()
    for k in range(128, len(pA), 64):
        write_block(pA[k:k + 64]); W(SHA, S_CTRL, 2); sha_wait()
    check_digest(dA)

MARK(23)                                   # HIN readback, INIT beats RESTORE
for i in range(4): W(SHA, S_HIN0 + 8 * i, 0x1111111111111111 * (i + 1))
for i in range(4): R(SHA, S_HIN0 + 8 * i, 0x1111111111111111 * (i + 1))
W(SHA, S_CTRL, 1 | 4); sha_wait()
iv = b''.join(x.to_bytes(4, 'big') for x in (
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19))
check_digest(iv)
# RESTORE of an arbitrary H reads straight back through DIGEST
W(SHA, S_CTRL, 4); sha_wait()
for i in range(4): R(SHA, S_DIGEST0 + 8 * i, 0x1111111111111111 * (i + 1))

MARK(24)                                   # ZERO wipes H, HIN, BLOCK
write_block(pA[0:64])
W(SHA, S_CTRL, 8); sha_wait()
for i in range(4): R(SHA, S_DIGEST0 + 8 * i, 0)
for i in range(4): R(SHA, S_HIN0 + 8 * i, 0)
for i in range(8): R(SHA, S_BLOCK0 + 8 * i, 0)

# 25: HMAC-SHA-256 from software midstates (the driver's PSA path), two keys
#     interleaved, vs hmac.new.
def mid_block(key, padbyte):
    k = key.ljust(64, b'\x00')
    return bytes(x ^ padbyte for x in k)
keys = [bytes(range(1, 33)), b'\x0b' * 20]
msgs = [b'Hi There', bytes((i * 13) & 0xFF for i in range(100))]
MARK(25)
slot = 0
for key in keys:                           # setkey: inner/outer midstates -> scratch
    for pb in (0x36, 0x5c):
        W(SHA, S_CTRL, 1); write_block(mid_block(key, pb)); W(SHA, S_CTRL, 2); sha_wait()
        for i in range(4): SAVE(SHA, S_DIGEST0 + 8 * i, slot + i)
        slot += 4
for kidx in (1, 0, 1):                     # interleave the two keys
    key, msg = keys[kidx], msgs[kidx]
    base = 8 * kidx
    for i in range(4): REPLAY(SHA, S_HIN0 + 8 * i, base + i)
    pm = pad(msg, prefix_len=64)
    first = True
    for k in range(0, len(pm), 64):
        write_block(pm[k:k + 64]); W(SHA, S_CTRL, (4 | 2) if first else 2); sha_wait()
        first = False
    inner = hashlib.sha256(mid_block(key, 0x36) + msg).digest()
    check_digest(inner)
    for i in range(4): REPLAY(SHA, S_HIN0 + 8 * i, base + 4 + i)
    write_block(pad(inner, prefix_len=64)); W(SHA, S_CTRL, 4 | 2); sha_wait()
    check_digest(hmac.new(key, msg, hashlib.sha256).digest())

MARK(26)                                   # legacy HMAC wrapper, RFC 4231 TC1
k = (b'\x0b' * 20).ljust(32, b'\x00')
for i in range(4): W(SHA, H_KEY0 + 8 * i, le64(k, 8 * i))
W(SHA, H_CTRL, 1); POLL(SHA, H_STATUS, 2, 3)
W(SHA, H_CTRL, 2)
write_block(pad(b'Hi There', 64)); W(SHA, S_CTRL, 2); sha_wait()
inner = hashlib.sha256(mid_block(b'\x0b' * 20, 0x36) + b'Hi There').digest()
check_digest(inner)
W(SHA, H_CTRL, 4)
write_block(pad(inner, 64)); W(SHA, S_CTRL, 2); sha_wait()
check_digest(bytes.fromhex('b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7'))


def main():
    os.makedirs(WORK, exist_ok=True)
    opsf = os.path.join(WORK, 'ops.txt')
    with open(opsf, 'w') as f:
        for op, dev, a, d, m in ops:
            f.write(f'{op:x} {dev:x} {a:x} {d:x} {m:x}\n')
    srcs = [os.path.join(SRC, n) for n in (
        'mmio_if.sv', 'aes_sbox.sv', 'aes_core.sv', 'ghash.sv', 'crypto_aes.sv',
        'sha256_core.sv', 'crypto_sha.sv')] + [TB]
    obj = os.path.join(WORK, 'obj')
    cmd = ['verilator', '--binary', '--timing', '-j', '8', '-O2', '-Wno-fatal', '-Wno-lint',
           '-Wno-style', '-Wno-MULTIDRIVEN', '-Wno-UNOPTFLAT', '--top-module', 'tb_crypto_mmio',
           '-Mdir', obj, *srcs]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-4000:], r.stderr[-4000:])
        sys.exit('verilator build failed')
    r = subprocess.run([os.path.join(obj, 'Vtb_crypto_mmio'), f'+ops={opsf}'],
                       capture_output=True, text=True)
    print(r.stdout.strip()[-6000:])
    sys.exit(0 if 'PASS' in r.stdout and 'FAIL' not in r.stdout else 1)


if __name__ == '__main__':
    main()

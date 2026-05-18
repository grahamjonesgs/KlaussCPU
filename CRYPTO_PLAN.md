# KlaussCPU Hardware Crypto Acceleration Plan

This document covers the design, HDL implementation, and software-consumption
guide for the five crypto blocks needed to run an SSH server at the 100 Mbps
line rate of the on-board LAN8720A PHY.

## 1. Motivation

Pure-software crypto on a 100 MHz CPU caps out around 1–3 MB/s for AES and
SHA-256, well below the ~12 MB/s peak of the 100 Mbps PHY. Every received
SSH packet has to be decrypted *and* MAC-verified before lwIP can hand it
upstream; every transmitted packet has to be authenticated and encrypted.
A modest set of HDL accelerators turns the bulk-crypto path into a single
cycle per byte and shifts the bottleneck back onto the network rather than
the CPU.

## 2. Block inventory

Five blocks, three new MMIO device IDs:

| Block         | Device base   | Function                              | Used by SSH for                            |
|---------------|---------------|---------------------------------------|--------------------------------------------|
| AES-128 core  | `0xF00A_0000` | ECB encrypt/decrypt; CTR via software | bulk cipher (CTR & GCM)                    |
| AES-GCM mode  | `0xF00A_0000` | GHASH + counter sequencing            | `aes128-gcm@openssh.com`                   |
| SHA-256 core  | `0xF00B_0000` | 64-byte compression function          | handshake transcript hash, key derivation  |
| HMAC wrapper  | `0xF00B_0080` | precomputed ipad/opad states          | `hmac-sha2-256` MAC mode                   |
| TRNG          | `0xF00C_0000` | ring-osc entropy + AES whitening      | host key, session key, IV generation       |

The address layout slots into the existing MMIO map (`MMIO_MAP.md`) at the
next free device-id rows (`0x009`–`0x00E` were unallocated; `0x00F` is the
interrupt controller). Three IDs are used to keep the blocks independently
testable — the alternative of a single shared device with sub-region decode
makes incremental bring-up harder.

## 3. CPU/bus integration model

All three blocks follow the existing pattern (`sd_spi`, `eth_mmio_bridge`):

1. New device-id wires in [bus_splitter.v](KlaussCPU.srcs/sources_1/new/bus_splitter.v)
   — none needed; everything sits in the `0xF` MMIO region, not the Ethernet
   range, so the existing MMIO port carries crypto traffic.
2. New chip-select decode in [KlaussCPU.v](KlaussCPU.srcs/sources_1/new/KlaussCPU.v)
   for each device id (`0x00A`, `0x00B`, `0x00C`), gating `w_mmio_write_DV` /
   `w_mmio_read_DV` per module.
3. Each module exposes the same 64-bit read-data / ready interface the SD
   controller uses, muxed into `r_mmio_read_data_comb`.

No CPU instruction changes, no new IRQ source (polling-only initially —
adding an IRQ source for "AES batch done" / "TRNG entropy ready" is a future
optimisation, see §10).

## 4. Item 1 — AES-128 core (`0xF00A_0000`)

### Architecture

128-bit data path. One AES round per clock. Key schedule expanded on-write
into 11 × 128-bit round-key registers; encrypt/decrypt then walks them
forward / backward over 10 cycles.

```
       MMIO write          aes_key_schedule.v
   ┌────────────────┐     ┌─────────────────────┐
   │ AES_KEY0..1    │ ──► │ expand to 11 round  │
   │ (128-bit key)  │     │ keys (1 cyc each)   │
   └────────────────┘     └──────────┬──────────┘
                                     │
   ┌────────────────┐                ▼
   │ AES_IN0..1     │ ──►  ┌──────────────────┐
   │ 128-bit plain  │      │ aes_round (comb) │
   └────────────────┘      │ SubBytes/        │
                           │ ShiftRows/       │
   ┌────────────────┐      │ MixColumns/      │
   │ AES_OUT0..1    │ ◄──  │ AddRoundKey      │
   │ 128-bit cipher │      └──────────────────┘
   └────────────────┘
```

S-box is implemented as a flat 256-entry case statement — Vivado infers
either a distributed-RAM ROM or pure LUT logic. Per-byte parallelism: 16
S-box lookups per cycle = 1 round per cycle.

### MMIO register layout

| Offset | Reg            | RW | Width | Description |
|--------|----------------|----|-------|-------------|
| 0x000  | `AES_CTRL`     | RW | 8     | `[0]` GO (self-clearing, starts one encrypt/decrypt), `[1]` ENC (1=encrypt, 0=decrypt), `[2]` KEY_LOAD (self-clearing, kicks key-schedule expansion), `[3]` KEY_ZERO (self-clearing, wipes key regs) |
| 0x008  | `AES_STATUS`   | R  | 8     | `[0]` BUSY (round or key-schedule in progress), `[1]` DONE (last operation completed; clears on next GO) |
| 0x010  | `AES_KEY0`     | RW | 64    | Key bits [63:0] |
| 0x018  | `AES_KEY1`     | RW | 64    | Key bits [127:64] |
| 0x040  | `AES_IN0`      | RW | 64    | Input block bits [63:0] |
| 0x048  | `AES_IN1`      | RW | 64    | Input block bits [127:64] |
| 0x050  | `AES_OUT0`     | R  | 64    | Output block bits [63:0] |
| 0x058  | `AES_OUT1`     | R  | 64    | Output block bits [127:64] |

### Internal FSM

```
IDLE → on GO+ENC=1  → ROUND0..ROUND10 → DONE → IDLE   (encrypt, 10 cycles)
IDLE → on GO+ENC=0  → IROUND0..IROUND10 → DONE → IDLE (decrypt, 10 cycles)
IDLE → on KEY_LOAD  → KSCHED0..KSCHED10 → DONE → IDLE (~11 cycles)
```

`BUSY` is high in any non-IDLE/DONE state. `DONE` latches and reads back as 1
until the next `GO`.

### Software contract

```c
/* Encrypt one block */
REG_AES_KEY0 = key_lo; REG_AES_KEY1 = key_hi;
REG_AES_CTRL = AES_CTRL_KEY_LOAD;
while (REG_AES_STATUS & AES_STATUS_BUSY) { }

REG_AES_IN0 = in_lo;  REG_AES_IN1  = in_hi;
REG_AES_CTRL = AES_CTRL_GO | AES_CTRL_ENC;
while (REG_AES_STATUS & AES_STATUS_BUSY) { }
out_lo = REG_AES_OUT0; out_hi = REG_AES_OUT1;
```

CTR mode (the standard SSH stream cipher) is built entirely in software on
top of ECB:

```c
void aes_ctr(uint64_t key_lo, uint64_t key_hi,
             uint64_t nonce_lo, uint64_t nonce_hi,
             const uint8_t *in, uint8_t *out, size_t len) {
    REG_AES_KEY0 = key_lo;  REG_AES_KEY1 = key_hi;
    REG_AES_CTRL = AES_CTRL_KEY_LOAD;
    while (REG_AES_STATUS & AES_STATUS_BUSY) { }

    uint64_t ctr_lo = nonce_lo, ctr_hi = nonce_hi;
    while (len >= 16) {
        REG_AES_IN0 = ctr_lo;  REG_AES_IN1 = ctr_hi;
        REG_AES_CTRL = AES_CTRL_GO | AES_CTRL_ENC;
        while (REG_AES_STATUS & AES_STATUS_BUSY) { }
        uint64_t ks_lo = REG_AES_OUT0, ks_hi = REG_AES_OUT1;

        ((uint64_t *)out)[0] = ((const uint64_t *)in)[0] ^ ks_lo;
        ((uint64_t *)out)[1] = ((const uint64_t *)in)[1] ^ ks_hi;

        ctr_lo += 1; if (ctr_lo == 0) ctr_hi += 1;   /* 128-bit counter */
        in += 16;  out += 16;  len -= 16;
    }
    /* … tail handling for non-block-multiple lengths … */
}
```

Throughput estimate (back-to-back blocks):
- 10 cycles encrypt + ~6 cycles MMIO handshake + 4 MMIO writes (4 cyc each)
  + 2 reads (2 cyc each) ≈ 38 cycles per 16 B = **2.4 cycle/byte ≈ 42 MB/s**.

That's enough headroom for the 100 Mbps line (12 MB/s peak).

## 5. Item 2 — AES-GCM / GHASH (`0xF00A_0080`)

GCM = CTR + GHASH (a GF(2^128) carryless multiplier accumulating into an
authentication tag).  The same AES core handles the CTR cipher; new
hardware is the GHASH unit plus a thin tag-accumulating wrapper.

### As-built scope

The original plan envisaged a full hardware FSM that sequenced "encrypt
counter / XOR into output / multiply-and-add into tag" automatically.
The as-built version is **simpler** — hardware exposes one primitive:

> *On `GCM_CTRL.GO`, compute  `tag ← (tag XOR X) • H`*

…and software sequences AES-CTR encryption with these GHASH steps to
build the full AEAD.  Rationale:

1. The hot loop is the same number of MMIO operations whether the FSM
   is in fabric or software — software has to write the plaintext block
   anyway, and the AES + GHASH are sequenced one after the other regardless.
2. A software-driven flow lets SSH (and TLS 1.3) reuse the same
   primitive blocks for any AEAD they want, including non-GCM modes
   (`AES-CCM`, `XChaCha20-Poly1305`'s GHASH-analog, etc.).
3. The HDL is ~3× smaller without the sequencing FSM, key schedule
   muxing, J0 caching, and length tracking that a full hardware FSM
   would need.

The throughput tradeoff is negligible: per-block, software adds ~30 cyc
of MMIO bookkeeping on top of the 128 cyc GHASH + 10 cyc AES, which
caps GCM at ~9 cyc/byte ≈ 11 MB/s — still comfortably above the 100
Mbps line rate.

### GHASH unit

Bit-serial GF(2^128) multiplication: 128 shift-and-XOR steps + 1 FSM cycle
per multiply.  See [ghash.v](KlaussCPU.srcs/sources_1/new/ghash.v).  A
**digit-serial** variant (4-bit or 8-bit per cycle) would bring this to
32 or 16 cycles per block respectively — same algorithm with the shift+XOR
inner loop unrolled in space instead of time.  Not needed for single-stream
SSH at 100 Mbps but the natural next upgrade if profiling shows GHASH is
the bottleneck (see §10a).  ("Karatsuba" is a *different* technique — it
splits operands in half to reduce the multiplication count and is normally
used in single-cycle parallel multipliers, not iterative ones.)

### MMIO register additions

| Offset | Reg          | RW | Width | Description |
|--------|--------------|----|-------|-------------|
| 0x080  | `GCM_CTRL`   | W  | 8     | `[0]` GO (kick the multiply).  `[1]` RESET (`tag ← 0`).  Both self-clearing. |
| 0x088  | `GCM_STATUS` | R  | 8     | `[0]` BUSY.  `[1]` DONE (sticky). |
| 0x090  | `GCM_H0`     | RW | 64    | Hash subkey H bytes 0..7 (software writes `AES_K(0^128)` here). |
| 0x098  | `GCM_H1`     | RW | 64    | H bytes 8..15. |
| 0x0A0  | `GCM_X0`     | RW | 64    | Next-block X bytes 0..7. |
| 0x0A8  | `GCM_X1`     | RW | 64    | X bytes 8..15. |
| 0x0B0  | `GCM_TAG0`   | R  | 64    | Current tag bytes 0..7. |
| 0x0B8  | `GCM_TAG1`   | R  | 64    | Tag bytes 8..15. |

All GCM regs use the natural little-endian `uint64_t` byte view; hardware
byteswaps internally to GHASH's network-bit-order convention.  The `AES_IN0/1`
and `AES_OUT0/1` registers are reused for the per-block CTR encryption.

### Software flow (per SSH frame)

```c
/* Once per session — key schedule + H = AES_K(0^128) */
aes_set_key(K);
aes_encrypt_zero_to(&h_lo, &h_hi);    /* enc 0^128 → H */
REG_GCM_H0 = h_lo; REG_GCM_H1 = h_hi;

/* Per frame: compute J0_KS = AES_K(J0) (saved in SW), tag = 0,
   then for each AAD/PT/length block: GHASH-accumulate.
   Final tag = REG_GCM_TAG XOR J0_KS. */
aes_gcm_frame(iv96, aad, aad_len, pt, ct, pt_len, tag);
```

The full reference flow is in [MMIO_MAP.md §AES-GCM](MMIO_MAP.md).

## 6. Item 3 — SHA-256 core (`0xF00B_0000`)

64-byte block in, 32-byte digest out. Standard 64-round compression
function: one round per cycle, message schedule maintained as a 16-deep
sliding window (W[t..t+15]) that shifts each round with the same recurrence
applied uniformly from round 0 onwards.

Implementation choice: round-iterative (one round per cycle, 64 cycles per
block) keeps area small. A 4-round-per-cycle unrolled version would hit ~16
cycles/block at higher LUT cost; not needed for SSH bandwidth.

**Padding is software's job.** Hardware just does the compression function.
This keeps the HDL simple (no `LAST` bit, no `BITLEN` register, no
end-of-message FSM branch) and lets software handle SHA-256, HMAC, and
truncation variants (SHA-224, eventually SHA-512/256) with the same compressor.

### MMIO register layout

| Offset       | Reg              | RW | Width | Description |
|--------------|------------------|----|-------|-------------|
| 0x000        | `SHA_CTRL`       | W  | 8     | `[0]` INIT (reset H to FIPS-180-4 IV; self-clearing). `[1]` START (compress block currently in BLOCK regs; self-clearing). |
| 0x008        | `SHA_STATUS`     | R  | 8     | `[0]` BUSY, `[1]` DONE (sticky; cleared by next INIT/START). |
| 0x010..0x048 | `SHA_BLOCK0..7`  | RW | 64×8  | 512-bit message block.  Each 64-bit slot holds two message words; hardware byteswaps each 32-bit half so software writes message bytes in their natural memory order. |
| 0x050..0x068 | `SHA_DIGEST0..3` | R  | 64×4  | 256-bit current digest, packed and byteswapped identically to BLOCK regs. |

### Software contract

```c
void sha256(const uint8_t *msg, size_t len, uint8_t out[32]) {
    REG_SHA_CTRL = SHA_CTRL_INIT;        /* zero-cycle: reset takes effect immediately */

    size_t off = 0;
    while (len - off >= 64) {
        for (int i = 0; i < 8; i++)
            SHA_BLOCK_PTR[i] = ((const uint64_t *)(msg + off))[i];
        REG_SHA_CTRL = SHA_CTRL_START;
        sha_wait_done();
        off += 64;
    }

    /* Final padded block(s): append 0x80, zero-pad, append 64-bit big-endian
       total message length in bits.  May produce one or two extra blocks. */
    uint8_t buf[128] = {0};
    size_t rem = len - off;
    memcpy(buf, msg + off, rem);
    buf[rem] = 0x80;
    uint64_t bitlen = (uint64_t)len * 8;
    /* Bitlen goes at the end of the last block in big-endian byte order. */
    size_t pad_blocks = (rem + 9 > 64) ? 2 : 1;
    size_t bitlen_off = pad_blocks * 64 - 8;
    for (int i = 0; i < 8; i++)
        buf[bitlen_off + i] = (uint8_t)(bitlen >> (56 - 8 * i));

    for (size_t b = 0; b < pad_blocks; b++) {
        for (int i = 0; i < 8; i++)
            SHA_BLOCK_PTR[i] = ((uint64_t *)(buf + 64 * b))[i];
        REG_SHA_CTRL = SHA_CTRL_START;
        sha_wait_done();
    }

    for (int i = 0; i < 4; i++)
        ((uint64_t *)out)[i] = SHA_DIGEST_PTR[i];
}
```

Throughput: 64 round-cycles + 8 MMIO writes per block ≈ ~80 cycles per 64 B
= **1.25 cycle/byte ≈ 80 MB/s**, plenty for SSH.

## 7. Item 4 — TRNG (`0xF00C_0000`)

Two-stage design:

1. **Entropy source** — 16 ring oscillators built from 5-stage inverter
   chains, with `(* DONT_TOUCH = "true" *)` and
   `(* ALLOW_COMBINATORIAL_LOOPS = "TRUE" *)` to keep Vivado from
   collapsing the rings.  Each output is captured by a double-FF
   synchroniser in the system-clock domain; the 16 sampled bits are
   XOR-folded to produce one raw entropy bit per cycle.
2. **Debiasing + accumulation** — a Von Neumann debiaser consumes pairs of
   raw bits and emits the first bit of any 01/10 pair (drops 00/11),
   feeding a 64-bit shift accumulator and a 2-deep output FIFO.
3. **Health monitor** — a NIST SP 800-90B Repetition-Count Test latches a
   fault if 32 consecutive raw bits are identical (≈ 2⁻³¹ false-positive
   on a balanced source).

### Conditioning policy — as-built vs. originally planned

The original plan called for an AES-CBC-MAC conditioning stage using an
internal AES instance.  The as-built design drops that and relies on:

1. 16-way XOR distillation across independent oscillators (raises per-bit
   min-entropy near 1 even if any single RO is biased).
2. Von Neumann debiasing (mathematically removes residual single-bit bias).
3. Software-side cryptographic conditioning: SSH wraps the TRNG in
   HMAC-DRBG (§9.4), which is itself a NIST-compliant conditioner using
   the SHA-256 hardware from item 3.

This keeps the HDL small (~200 LUTs vs. ~2.5k for a dedicated AES
instance) while preserving the security posture: the cryptographic
conditioning step is still present, just moved out of fabric into
software using existing accelerators.  If profiling ever shows the
HMAC-DRBG seed path is the bottleneck, a fabric AES-CBC-MAC stage can be
added later — but for seeding a CSPRNG once per SSH session, software is
faster than the per-block hardware MMIO handshake would have been.

### MMIO register layout (as built)

| Offset | Reg            | RW | Width | Description |
|--------|----------------|----|-------|-------------|
| 0x000  | `TRNG_CTRL`    | RW | 8     | `[0]` ENABLE (start sampling).  `[1]` RESEED (self-clearing — drains FIFO, resets accumulator, restarts the RCT). |
| 0x008  | `TRNG_STATUS`  | R  | 8     | `[0]` READY (≥1 64-bit word in FIFO).  `[1]` HEALTH_OK (RCT has not tripped). |
| 0x010  | `TRNG_DATA`    | R  | 64    | Side-effecting read: consumes one FIFO entry. |

(`TRNG_RATE` from the original plan was dropped — without an AES
conditioner there's no inter-batch rate to throttle.)

### Software contract — seeding a CSPRNG

```c
int trng_seed(uint8_t out[32]) {
    REG_TRNG_CTRL = TRNG_CTRL_ENABLE;
    for (int i = 0; i < 4; i++) {
        if (!trng_read64(&((uint64_t *)out)[i])) return -1;   /* health fault */
    }
    return 0;
}
```

`trng_read64()` (defined in `mmio.h`) blocks on `READY`, checks
`HEALTH_OK`, and returns 0 on fault.  Callers MUST check the return
value for crypto-key use.

### Throughput

At ~25 % Von Neumann yield on a balanced source: one fresh 64-bit word
every ~256 system cycles ≈ 2.6 µs at 100 MHz ≈ **400 k words/s**.  Way
more than any reasonable HMAC-DRBG reseed cadence — SSH typically
reseeds once per connection, so the TRNG is functionally always-ready.

## 8. Item 5 — HMAC wrapper (`0xF00B_0080`)

Extension to the SHA-256 device.  Precomputes the inner/outer hash midstates
when a key is loaded:

- `inner_state = SHA256_compress(IV, K' ⊕ ipad)` — one block, ~67 cycles
- `outer_state = SHA256_compress(IV, K' ⊕ opad)` — one block, ~67 cycles

Where K' = K (32 bytes) || 0×32 padded to 64 bytes, ipad = 0x36×64,
opad = 0x5c×64.

These two midstates are cached in private register banks inside
`crypto_sha.v`.  When software triggers `HMAC_CTRL.START`, the SHA core
gets a single `i_h_load` pulse that overwrites H[0..7] with inner_state;
software then streams the message via the normal SHA path.  After the
inner pass, software triggers `HMAC_CTRL.FINAL` (single-cycle H reload to
outer_state) and issues one more SHA block containing the inner digest
plus padding to produce the tag.

### As-built note

The plan originally said FINAL would "automatically swap to outer_state
and feed the inner digest through one more block."  The as-built version
is one step less automatic: FINAL only reloads H — software then has to
write the inner-digest-plus-padding block to `SHA_BLOCK0..7` and trigger
`SHA_CTRL.START` itself.  Same final result, ~8 fewer lines of HDL FSM,
and the resulting per-MAC code path is identical to the inner-pass code
path which makes the SSH glue smaller.

### MMIO register layout (as built)

| Offset | Reg            | RW | Width | Description |
|--------|----------------|----|-------|-------------|
| 0x080  | `HMAC_CTRL`    | W  | 8     | `[0]` KEY_LOAD (recompute inner/outer midstates from `HMAC_KEY0..3`). `[1]` START (H ← inner_state). `[2]` FINAL (H ← outer_state). `[3]` KEY_ZERO. All self-clearing. |
| 0x088  | `HMAC_STATUS`  | R  | 8     | `[0]` BUSY (KEY_LOAD FSM running). `[1]` KEY_VALID (midstates have been computed). |
| 0x090..0x0AC | `HMAC_KEY0..3` | RW | 64×4 | 256-bit key.  Longer keys must be SHA-256-hashed by software first (per RFC 2104); shorter keys must be zero-padded. |

Data-block writes reuse the SHA `BLOCK0..7` registers; output reuses
`SHA_DIGEST0..3`.  `SHA_STATUS.BUSY` is gated against the HMAC FSM, so a
software wait on `SHA_STATUS.BUSY` won't see the FSM's internal SHA passes
during KEY_LOAD — software polls `HMAC_STATUS.BUSY` for that.

### Why a wrapper

Doing HMAC in software on top of the bare SHA core costs ~30–50 extra
cycles per packet: two extra SHA setups (each = ~64 cycles of block
processing) and a 32-byte XOR.  The wrapper amortises both ipad/opad
midstate computations across the *session* (computed once when the SSH
key is installed) rather than per packet.

For SSH at 8000 packets/sec on a 100 MHz CPU, that's ~400k cycles/sec
saved = ~0.4% of CPU.  Marginal — and consciously kept simple here: ~150
LUTs of FSM plus the cached midstates.

## 9. SSH server software consumption guide

This section is the contract for whoever writes the SSH integration. The
underlying SSH implementation will be a port of one of:
- **wolfSSH** (recommended — small, MIT-style licence, plugs into wolfCrypt
  whose API is hardware-friendly)
- **Dropbear** (also viable; uses LibTomCrypt — slightly more glue)
- **OpenSSH portable** (heaviest; pulls in BoringSSL — most rewriting)

Replace four pluggable pieces in the chosen library:

### 9.1 Bulk cipher — replace `aes_ctr` / `aes_gcm`

wolfCrypt's `wc_AesSetKey` / `wc_AesCtrEncrypt` / `wc_AesGcmEncrypt` map
directly onto the AES MMIO block. Two implementation files to fork:

- `wolfcrypt/src/aes.c` → KlaussCPU port that delegates to `0xF00A_0000`.
- Add `klauss_aes.c` in the firmware tree with the loop shown in §4.

The CTR mode lives in software (`aes_ctr.c`). GCM has two options:
- **Phase A** (initial): software GCM on top of hardware ECB. Works
  immediately, ~5× slower than full hardware GCM but still 5× faster than
  full software.
- **Phase B**: switch to hardware GCM once item 2 lands. Same C API, just
  a different backend.

### 9.2 Hash — replace `Sha256`

wolfCrypt's `wc_Sha256Update` / `wc_Sha256Final` map to the SHA-256 block.
SSH uses SHA-256 in three places:
1. **Handshake transcript hash** (`session_id`) — one long streaming hash
   per connection. Hardware streaming model fits exactly.
2. **Key derivation** (`KDF`) — repeated SHA-256 calls with small inputs.
   Also fine, but the per-block MMIO overhead dominates for tiny messages
   (~96 cycles per block + ~50 cycles MMIO ≈ 150 cycles vs ~1300 cycles
   pure software — still a 9× win even with the overhead).
3. **MAC** (`hmac-sha2-256`) — see §9.3 below.

### 9.3 MAC — replace `Hmac` for SHA-256

Until item 5 lands, run HMAC in software on top of the SHA hardware. With
item 5, set the SSH key once via `HMAC_KEY0..3 + HMAC_CTRL.KEY_LOAD` and
let hardware do the wrap.

### 9.4 RNG — replace `wc_RNG_GenerateBlock`

Wire the TRNG output to wolfCrypt's `wc_RNG` seeding path. Use the TRNG
purely as a *seed* for an in-software CSPRNG (HMAC-DRBG, which the
hardware HMAC now accelerates) — never read keys directly out of the
TRNG. This is both faster (CSPRNG output is byte-wise free) and conformant
with FIPS 800-90A recommendations.

Recommended seed cadence:
- Boot: read 256 bits, instantiate DRBG.
- Every SSH connection: reseed with 64 fresh TRNG bits.
- Health check fail (`HEALTH_OK == 0`): refuse all further key operations
  until reboot.

### 9.5 Curve25519 / Ed25519

Stays pure-software (wolfCrypt's curve25519 / ed25519 modules). The
existing `MULR` / `MULHR` 64-bit multiplier in the CPU is enough to keep
the one-shot handshake cost under ~30 ms.

### 9.6 Things SSH needs that AREN'T crypto

These are unblocked once the existing TCP stack (lwIP) is up:
- A TCP listening socket on port 22.
- A pseudo-tty layer (`vfork`/`exec` won't exist — wolfSSH supports a
  "command callback" mode that runs a single in-process command, perfect
  for a minimal shell).
- An authorized-keys store. Easiest: a fixed Ed25519 public key compiled
  into firmware. Slightly fancier: read from SD.

None of those need new HDL.

## 10. Future hardware extensions (not in scope)

Listed for completeness — DON'T do these as part of this work:

1. **AES-256** — add 7 more round-key slots, change `KEY_LOAD` FSM to
   expand 14 rounds. Touches `aes_core` only.
2. **AES IRQ** — `AES_CTRL` could include "interrupt on done" using IRQ
   source 1 or 2 from the existing interrupt controller (one of the three
   reserved sources). Useful once the FreeRTOS port is up and AES calls
   are made from blocking RTOS tasks.
3. **DMA path** — point hardware at a DDR2 source/dest pair and let it
   stream. Cuts the MMIO write/read overhead to zero. Only worthwhile if
   we move to a gigabit PHY (not on this board).
4. **ChaCha20-Poly1305** — alternative to AES-GCM. Smaller area than
   AES+GHASH together (~1k LUTs vs ~3k), but redundant once both are in.
5. **SHA-512** — needed for newer SSH KEX (`curve25519-sha512`). Same
   shape as SHA-256, wider data path.

## 10a. Timing closure notes (as built)

### Build history

| Build state                                                | WNS (post-route) | Phase 5 iterations | Comment |
|------------------------------------------------------------|------------------|--------------------|---------|
| pre-GCM (items 1, 3, 4, 5)                                 | comfortably positive | 1                  | clean |
| post-GCM, pre-pipeline (all 5 items, no pipeline regs)     | +0.360 ns           | 3 (5.1 came back −0.345 ns) | total impl ~28 min; Phase 6 delay-cleanup was the long pole at 27 min |
| **post-GCM, pipelined (current as-built)**                 | TBD on next build    | TBD                | expected ~+0.5–0.6 ns, faster Phase 5 convergence |

### What the post-GCM build actually told us

Reading `KlaussCPU_timing_summary_postroute_physopted.rpt` from the
previous build, the *worst-case* paths weren't the wide muxes the §10a
draft anticipated — they were:

1. **AES round-key BRAM clock → address** (0.367 ns slack).  Vivado
   mapped the 11 × 128-bit `r_round_key` array into RAMB18E1; that
   BRAM's `CLKBWRCLK → ADDRARDADDR` setup path is what gates timing.
2. **AES state-data BRAM clock → address** (0.382 ns slack).  Same shape
   as above, on the per-round state register that also got BRAM-packed.
3. **SHA round counter → working variable** (0.375 ns slack).
   `r_round_reg[4] → a_reg[31]` — the K-constant lookup feeds into the
   5-input T1 adder.
4. **CPU instruction-fetch path** (0.360 ns slack).
   `r_var1_mem_reg → r_PC_reg` — pre-existing CPU path.

The wide GHASH / HMAC mux paths I worried about in the original draft
were sitting around 0.4–0.5 ns slack — tight but not the worst.  The
right read of the report is: **the BRAM-mapped registers are now the
limiter, not the wide muxes**.  Pipelining the wide muxes still helps
the placer (less wide-fanout to route, more freedom for the BRAM
placements), it's just not directly on the worst path.

### What's been applied (current build)

All four moves below are in the source tree as of the current build:

1. **GHASH input pipeline register** (`crypto_aes.v`):
   ```verilog
   reg [127:0] r_ghash_X_pipe, r_ghash_H_pipe;
   always @(posedge i_Clk) begin
       r_ghash_X_pipe <= bswap128(r_gcm_tag ^ r_gcm_X);
       r_ghash_H_pipe <= bswap128(r_gcm_H);
   end
   ```
   Fed into `ghash u_ghash`'s `i_X` / `i_H`.  Adds 1 cycle to each
   GHASH multiply (130 → 131 cycles).  Negligible at SSH bandwidths.
   No FSM changes needed — the existing `GCM_STARTING` state already
   gives the pipeline reg the cycle it needs to settle.

2. **SHA block input pipeline register** (`crypto_sha.v`):
   ```verilog
   reg [511:0] r_sha_block_pipe;
   always @(posedge i_Clk) r_sha_block_pipe <= w_sha_block_pre;
   ```
   The 512-bit 3-way mux (software block / HMAC inner block / HMAC outer
   block) is now registered before reaching `sha256_core.i_block`.  Adds
   1 cycle before the SHA round loop starts — invisible against 64-round
   compression.  Both software-driven SHA and the HMAC FSM emit at least
   1 cycle between data settling and the start pulse, so no further
   sequencing changes were needed.

3. **`KEEP_HIERARCHY = "yes"` on inner cores**:
   `aes_core u_aes`, `ghash u_ghash` (both inside `crypto_aes.v`), and
   `sha256_core u_sha` (inside `crypto_sha.v`).  Pairs with the existing
   `KEEP_HIERARCHY` attributes on the wrappers in `KlaussCPU.v` — the
   placer now clusters each crypto module as a single physical block
   rather than smearing into the surrounding CPU logic.  Ring
   oscillators in `trng.v` already had this attribute.

### What was NOT applied (and why)

- **Pipelining `r_gcm_start_pulse`** — not needed.  The `GCM_STARTING`
  state already gives the pipeline reg the settle cycle it requires
  before ghash sees `i_start`.  Adding another delay would be redundant.
- **Pipelining inside `aes_core` / `sha256_core`** — the worst-case
  paths run through Vivado-inferred BRAMs (round_key, state_data) that
  can't be fixed by adding pipeline registers in HDL; they're already
  fully-pipelined at the BRAM hard-block level.  If margin tightens
  further, the right move is to ATTRIBUTE-pin those registers to slice
  flip-flops instead of BRAM (`(* ram_style = "distributed" *)`) to get
  more placement flexibility at some LUT cost.
- **Registering the GHASH output path** — `bswap128(w_ghash_Z_out)`
  feeding `r_gcm_tag` is FF→wires→FF (0 LUT levels), not a path the
  placer struggles with.

### When to revisit

If a future addition (4-bit-per-cycle digit-serial GHASH, AES-256,
SHA-512, multi-session SSH, gigabit PHY) drives WNS below ~0.2 ns,
in order of preference:

1. Switch `r_round_key` and `r_state_data` from inferred BRAM to LUTRAM
   via `(* ram_style = "distributed" *)`.  Frees up the BRAM-clocked
   critical paths but increases LUT count by ~1500.
2. Pipeline the K-constant lookup inside `sha256_core` (1 extra round
   per SHA block).
3. Look at the `r_var1_mem → r_PC_reg` CPU fetch path — pre-existing
   and unrelated to crypto, but it's the next limiter after the BRAMs.

## 11. Implementation order and milestones

Day-count estimates assume one engineer, 6 productive hours/day, with HDL
sim available.

| # | Item                  | HDL files                              | Days  | Milestone |
|---|-----------------------|----------------------------------------|-------|-----------|
| 1 | AES-128 core + MMIO   | `aes_core.v`, `aes_sbox.v`, `crypto_aes.v` | 4 | `openssl enc -aes-128-ecb` round-trips via C test |
| 2 | AES-GCM mode          | `ghash.v`, `aes_gcm.v`                 | 3     | Test vectors from NIST GCM AE pass |
| 3 | SHA-256 core + MMIO   | `sha256_core.v`, `crypto_sha.v`        | 3     | NIST SHA-256 test vectors pass |
| 4 | TRNG                  | `ring_osc.v`, `trng.v`                 | 3     | `dieharder` on 10 MB of output |
| 5 | HMAC wrapper          | `hmac_wrapper.v`                       | 2     | RFC 4231 vectors pass |
| - | SSH integration       | wolfSSH port + firmware                | 5–10  | OpenSSH client connects, runs `ls` |

Total HDL: ~3 weeks. SSH glue: ~1–2 weeks on top.

## 12. Validation strategy

Every block needs three tiers of test before declaring done:

1. **Verilog testbench** — drive known plaintexts, compare output to
   expected ciphertext. Reference: NIST CAVP / FIPS-197 known-answer
   vectors. Lives in `KlaussCPU.srcs/sim_1/new/test_<block>.v`.
2. **On-FPGA C self-test** — small C program that loads vectors via
   `mmio.h`, runs the block, prints PASS/FAIL over UART. Lives in
   `tools/crypto_selftest/`.
3. **Interop test** — encrypt with KlaussCPU, decrypt with OpenSSL on a
   host (or vice versa). For TRNG: pipe output to `dieharder -a` /
   `rngtest`.

Skipping (3) is the most common way crypto hardware ships broken. Don't.

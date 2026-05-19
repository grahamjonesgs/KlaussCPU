/* crypto_test.c — on-FPGA self-test for the AES/SHA-256/HMAC/GHASH/TRNG
 * crypto blocks (CRYPTO_PLAN.md items 1–5).
 *
 * Style follows eth_test.c — uses REG_SEG_ALL on the 7-segment display for
 * pass/fail reporting (no UART driver wired into C yet).  Each test stage
 * paints two codes:
 *
 *   PRE  = 0x000N_0000          before the test runs (N = test number)
 *   POST = 0x000N_0000          after PASS
 *        | 0x000N_EEEE          after FAIL (low 4 hex digits = first
 *                               mismatched 32-bit word index for AES/SHA,
 *                               or 0xEEEE for everything else)
 *
 * After all tests:
 *   0x00990000u  = ALL PASS  (a friendly "00" prefix; lower 0000 = pass)
 *   0x00FFEEEEu  = ANY FAIL
 *
 * Stages:
 *   1. AES-128 encrypt + round-trip decrypt   (FIPS-197 §B vector)
 *   2. SHA-256 of "abc"                       (FIPS-180-4 §B.1)
 *   3. HMAC-SHA-256 of "Hi There" key=0x0b×20 (RFC 4231 TC1)
 *   4. GHASH NIST GCM TC2                     (chained 2-step multiply)
 *   5. TRNG: enable, read 4 words, check HEALTH_OK and non-zero
 *
 * Each stage holds its result on the display for ~1.5 seconds so a human can
 * read it sequentially.
 */

#include <stdint.h>
#include <string.h>
#include "mmio.h"

/* ---------------- Helpers (same calibration as eth_test.c) ---------------- */

static inline void delay_loops(uint32_t n) {
    volatile uint32_t i;
    for (i = 0; i < n; i++) { __asm__ volatile (""); }
}
static inline void delay_ms(uint32_t ms) { delay_loops(ms * 20000u); }

static inline void show(uint32_t code) { REG_SEG_ALL = code; }

static inline uint32_t bswap32_c(uint32_t v) {
    return ((v & 0x000000ffu) << 24)
         | ((v & 0x0000ff00u) <<  8)
         | ((v & 0x00ff0000u) >>  8)
         | ((v & 0xff000000u) >> 24);
}

static inline uint64_t bswap64_c(uint64_t v) {
    return ((uint64_t)bswap32_c((uint32_t)v) << 32) | bswap32_c((uint32_t)(v >> 32));
}

/* Compare two 16-byte buffers; return 0 on match else 1. */
static int bcmp16(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < 16; i++) if (a[i] != b[i]) return i + 1;  /* nonzero = first mismatch+1 */
    return 0;
}
static int bcmp32(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < 32; i++) if (a[i] != b[i]) return i + 1;
    return 0;
}

/* ---------------- AES helpers ---------------- */

static void aes_set_key(const uint8_t key[16]) {
    REG_AES_KEY0 = ((const uint64_t *)key)[0];
    REG_AES_KEY1 = ((const uint64_t *)key)[1];
    REG_AES_CTRL = AES_CTRL_KEY_LOAD;
    aes_wait_done();
}

static void aes_encrypt_block(const uint8_t pt[16], uint8_t ct[16]) {
    REG_AES_IN0 = ((const uint64_t *)pt)[0];
    REG_AES_IN1 = ((const uint64_t *)pt)[1];
    REG_AES_CTRL = AES_CTRL_GO | AES_CTRL_ENC;
    aes_wait_done();
    ((uint64_t *)ct)[0] = REG_AES_OUT0;
    ((uint64_t *)ct)[1] = REG_AES_OUT1;
}

static void aes_decrypt_block(const uint8_t ct[16], uint8_t pt[16]) {
    REG_AES_IN0 = ((const uint64_t *)ct)[0];
    REG_AES_IN1 = ((const uint64_t *)ct)[1];
    REG_AES_CTRL = AES_CTRL_GO;                    /* ENC bit = 0 → decrypt */
    aes_wait_done();
    ((uint64_t *)pt)[0] = REG_AES_OUT0;
    ((uint64_t *)pt)[1] = REG_AES_OUT1;
}

/* ---------------- SHA-256 helper (single-block) ---------------- */

/* Hash a buffer that fits in a single SHA-256 block after padding (≤ 55 bytes).
   Writes 32-byte digest to `out`. */
static void sha256_one_block(const uint8_t *msg, uint32_t len, uint8_t out[32]) {
    uint8_t block[64];
    for (int i = 0; i < 64; i++) block[i] = 0;
    for (uint32_t i = 0; i < len; i++) block[i] = msg[i];
    block[len] = 0x80;
    uint64_t bitlen = (uint64_t)len * 8;
    /* big-endian bit length in bytes 56..63 */
    for (int i = 0; i < 8; i++)
        block[63 - i] = (uint8_t)(bitlen >> (8 * i));

    REG_SHA_CTRL = SHA_CTRL_INIT;
    /* No wait needed for INIT — it completes the same cycle. */
    for (int i = 0; i < 8; i++)
        SHA_BLOCK_PTR[i] = ((const uint64_t *)block)[i];
    REG_SHA_CTRL = SHA_CTRL_START;
    sha_wait_done();
    for (int i = 0; i < 4; i++)
        ((uint64_t *)out)[i] = SHA_DIGEST_PTR[i];
}

/* ---------------- HMAC helper (single-block message) ---------------- */

/* HMAC-SHA-256 with key ≤ 32 bytes (zero-padded) and message ≤ 55 bytes. */
static void hmac_sha256(const uint8_t *key, uint32_t klen,
                        const uint8_t *msg, uint32_t mlen,
                        uint8_t tag[32]) {
    /* Build 32-byte padded key in software, write to HMAC_KEY0..3. */
    uint8_t k32[32];
    for (uint32_t i = 0; i < 32; i++) k32[i] = (i < klen) ? key[i] : 0;
    REG_HMAC_KEY0 = ((const uint64_t *)k32)[0];
    REG_HMAC_KEY1 = ((const uint64_t *)k32)[1];
    REG_HMAC_KEY2 = ((const uint64_t *)k32)[2];
    REG_HMAC_KEY3 = ((const uint64_t *)k32)[3];

    /* Compute midstates. */
    REG_HMAC_CTRL = HMAC_CTRL_KEY_LOAD;
    hmac_wait_keyload();

    /* Inner pass: H = inner_state, then hash (data || padding) with
       bit length = 64*8 + mlen*8 = 512 + mlen*8. */
    REG_HMAC_CTRL = HMAC_CTRL_START;

    uint8_t blk[64];
    for (int i = 0; i < 64; i++) blk[i] = 0;
    for (uint32_t i = 0; i < mlen; i++) blk[i] = msg[i];
    blk[mlen] = 0x80;
    uint64_t inner_bits = 512ULL + (uint64_t)mlen * 8ULL;
    for (int i = 0; i < 8; i++)
        blk[63 - i] = (uint8_t)(inner_bits >> (8 * i));
    for (int i = 0; i < 8; i++)
        SHA_BLOCK_PTR[i] = ((const uint64_t *)blk)[i];
    REG_SHA_CTRL = SHA_CTRL_START;
    sha_wait_done();

    /* Read inner digest. */
    uint8_t inner_digest[32];
    for (int i = 0; i < 4; i++)
        ((uint64_t *)inner_digest)[i] = SHA_DIGEST_PTR[i];

    /* Outer pass: H = outer_state, then hash (inner_digest || padding)
       with bit length = 64*8 + 32*8 = 768. */
    REG_HMAC_CTRL = HMAC_CTRL_FINAL;

    for (int i = 0; i < 64; i++) blk[i] = 0;
    for (int i = 0; i < 32; i++) blk[i] = inner_digest[i];
    blk[32] = 0x80;
    uint64_t outer_bits = 768ULL;
    for (int i = 0; i < 8; i++)
        blk[63 - i] = (uint8_t)(outer_bits >> (8 * i));
    for (int i = 0; i < 8; i++)
        SHA_BLOCK_PTR[i] = ((const uint64_t *)blk)[i];
    REG_SHA_CTRL = SHA_CTRL_START;
    sha_wait_done();

    for (int i = 0; i < 4; i++)
        ((uint64_t *)tag)[i] = SHA_DIGEST_PTR[i];
}

/* ---------------- Main self-test ---------------- */

void main(void) {
    int any_fail = 0;
    uint8_t buf[32];

    /* ===== Stage 1: AES-128 FIPS-197 §B vector ===== */
    show(0x00010000u);
    delay_ms(500);
    {
        static const uint8_t key[16] = {
            0x2b,0x7e,0x15,0x16, 0x28,0xae,0xd2,0xa6,
            0xab,0xf7,0x15,0x88, 0x09,0xcf,0x4f,0x3c
        };
        static const uint8_t pt[16] = {
            0x32,0x43,0xf6,0xa8, 0x88,0x5a,0x30,0x8d,
            0x31,0x31,0x98,0xa2, 0xe0,0x37,0x07,0x34
        };
        static const uint8_t expected_ct[16] = {
            0x39,0x25,0x84,0x1d, 0x02,0xdc,0x09,0xfb,
            0xdc,0x11,0x85,0x97, 0x19,0x6a,0x0b,0x32
        };
        uint8_t ct[16], pt2[16];

        aes_set_key(key);
        aes_encrypt_block(pt, ct);
        int e1 = bcmp16(ct, expected_ct);
        aes_decrypt_block(expected_ct, pt2);
        int e2 = bcmp16(pt2, pt);

        if (e1 == 0 && e2 == 0) {
            show(0x00010000u);
        } else {
            show(0x0001EEEEu);
            any_fail = 1;
        }
    }
    delay_ms(1500);

    /* ===== Stage 2: SHA-256("abc") ===== */
    show(0x00020000u);
    delay_ms(500);
    {
        static const uint8_t msg[3] = { 'a', 'b', 'c' };
        static const uint8_t expected[32] = {
            0xba,0x78,0x16,0xbf, 0x8f,0x01,0xcf,0xea,
            0x41,0x41,0x40,0xde, 0x5d,0xae,0x22,0x23,
            0xb0,0x03,0x61,0xa3, 0x96,0x17,0x7a,0x9c,
            0xb4,0x10,0xff,0x61, 0xf2,0x00,0x15,0xad
        };
        sha256_one_block(msg, 3, buf);
        if (bcmp32(buf, expected) == 0) {
            show(0x00020000u);
        } else 
        static const uint8_t expected[32] = {
            0xb0,0x34,0x4c,0x61, 0xd8,0xdb,0x38,0x53,
            0x5c,0xa8,0xaf,0xce, 0xaf,0x0b,0xf1,0x2b,
            0x88,0x1d,0xc2,0x00, 0xc9,0x83,0x3d,0xa7,
            0x26,0xe9,0x37,0x6c, 0x2e,0x32,0xcf,0xf7
        };
        hmac_sha256(key, 20, data, 8, buf);
        if (bcmp32(buf, expected) == 0) {
            show(0x00030000u);
        } else {
            show(0x0003EEEEu);
            any_fail = 1;
        }
    }
    delay_ms(1500);

    /* ===== Stage 4: GHASH NIST GCM TC2 (chained two-step multiply) ===== */
    show(0x00040000u);
    delay_ms(500);
    {
        /* Compute H = AES_K=0(0^128) using the AES core. */
        uint8_t zero16[16] = {0};
        aes_set_key(zero16);
        uint8_t H[16];
        aes_encrypt_block(zero16, H);
        REG_GCM_H0 = ((uint64_t *)H)[0];
        REG_GCM_H1 = ((uint64_t *)H)[1];

        /* tag = 0; then tag = (tag XOR CT) • H */
        REG_GCM_CTRL = GCM_CTRL_RESET;

        /* CT in network byte order: 03 88 da ce 60 b6 a3 92 f3 28 c2 b9 71 b2 fe 78 */
        static const uint8_t ct[16] = {
            0x03,0x88,0xda,0xce, 0x60,0xb6,0xa3,0x92,
            0xf3,0x28,0xc2,0xb9, 0x71,0xb2,0xfe,0x78
        };
        REG_GCM_X0 = ((const uint64_t *)ct)[0];
        REG_GCM_X1 = ((const uint64_t *)ct)[1];
        REG_GCM_CTRL = GCM_CTRL_GO;
        gcm_wait_done();

        /* tag = (tag XOR lengths) • H, where lengths = lenA(64) || lenC(64) BE.
           lenA = 0, lenC = 128 bits.  As bytes: 00×8 || 00×7 || 0x80. */
        static const uint8_t lengths[16] = {
            0,0,0,0, 0,0,0,0,
            0,0,0,0, 0,0,0,0x80
        };
        REG_GCM_X0 = ((const uint64_t *)lengths)[0];
        REG_GCM_X1 = ((const uint64_t *)lengths)[1];
        REG_GCM_CTRL = GCM_CTRL_GO;
        gcm_wait_done();

        /* Final tag = GHASH XOR AES_K(J0).  J0 = IV(96 zeros) || 0x00000001. */
        uint8_t j0[16] = {0};
        j0[15] = 0x01;
        uint8_t j0_ks[16];
        aes_encrypt_block(j0, j0_ks);

        uint8_t tag[16];
        ((uint64_t *)tag)[0] = REG_GCM_TAG0 ^ ((uint64_t *)j0_ks)[0];
        ((uint64_t *)tag)[1] = REG_GCM_TAG1 ^ ((uint64_t *)j0_ks)[1];

        /* NIST GCM TC2 expected T = ab 6e 47 d4 2c ec 13 bd f5 3a 67 b2 12 57 bd df */
        static const uint8_t expected[16] = {
            0xab,0x6e,0x47,0xd4, 0x2c,0xec,0x13,0xbd,
            0xf5,0x3a,0x67,0xb2, 0x12,0x57,0xbd,0xdf
        };
        if (bcmp16(tag, expected) == 0) {
            show(0x00040000u);
        } else {
            show(0x0004EEEEu);
            any_fail = 1;
        }
    }
    delay_ms(1500);

    /* ===== Stage 5: TRNG ===== */
    show(0x00050000u);
    delay_ms(500);
    {
        REG_TRNG_CTRL = TRNG_CTRL_ENABLE;
        delay_ms(10);   /* let the ring oscillators settle + Von Neumann fill */
        uint64_t words[4];
        int ok = 1;
        for (int i = 0; i < 4; i++) {
            if (!trng_read64(&words[i])) { ok = 0; break; }
        }
        /* Reject the trivially-stuck-at-zero case (would indicate ROs not
           actually oscillating on this fabric).  All four words being zero
           on a working TRNG is astronomically unlikely. */
        if (ok && (words[0] | words[1] | words[2] | words[3]) != 0) {
            show(0x00050000u);
        } else {
            show(0x0005EEEEu);
            any_fail = 1;
        }
    }
    delay_ms(1500);

    /* ===== Final summary ===== */
    if (any_fail) {
        show(0x00FFEEEEu);
    } else {
        show(0x00990000u);   /* "99" = all-pass marker */
    }

    /* Spin so the final code stays visible. */
    while (1) { delay_ms(1000); }
}

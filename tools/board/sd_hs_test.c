// sd_hs_test.c — SD_NATIVE_PLAN.md phase 3 (§9): bare-metal board test of
// SD high speed (50 MHz) on the native 4-bit controller.  READ-ONLY: the
// program has no write path (no CMD24 / CMD25); the only state it changes on
// the card is the CMD6 function switch, which a power cycle undoes.
//
//   1. identification at 400 kHz, 4-bit, 25 MHz default speed;
//   2. read a fixed region (SIZE_MIB, default 8 MiB, from LBA; default: the
//      start of partition 1, else 2048) twice at 25 MHz: the reference and
//      its checksum;
//   3. CMD6 mode 0 (check), then mode 1 (switch function group 1 to high
//      speed); clk_div 0 (50 MHz);
//   4. calibration: for every IDELAY tap group (TAPS) and sample phase 0..6
//      (SDM_CFG[7:4]: the bit launched on CLK's rising edge is sampled
//      10 + 5p ns after it, minus 78 ps per tap), read SWEEP_BLKS blocks
//      SWEEP_REP times and compare with the reference; the first try drains
//      with the FIFO full, so CLK parks after every block (rd_parked); a
//      table of passes and error bits per phase / tap;
//   5. the setting: the centre of the widest passing window of the sample
//      instant modulo the 20 ns clock (phase p and p + 4 are the same
//      instant one bit apart), phase p <= 2 moved to p + 4 (pick());
//      verified by a parked read of VERIFY_BLKS blocks;
//   6. the region read REPEAT times at 50 MHz by PIO and REPEAT times by DMA,
//      each compared with the reference; MB/s.
// SDM_CFG: the IDELAY tap is reloaded on every SDM_CFG write (the DMA
// abort, DMA_EN = 0, included), so every write here carries the current
// [0] | [1] | [7:4] | [12:8] (SD_NATIVE_PLAN.md §9.2).
// Ends with "SDHS PASS" or "SDHS FAIL", the slot powered off, SPI mode.
//
// Build: tools/board/build.sh sd_hs_test [-O2]
//        (DEFS="-DLBA=100000 -DSIZE_MIB=4" tools/board/build.sh sd_hs_test)
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define R64(a)       (*(volatile uint64_t *)(uintptr_t)(a))
#define SD           0xF0000000u
#define SD_CTRL      R64(SD + 0x000)
#define SDM_CMD      R64(SD + 0x100)
#define SDM_ARG      R64(SD + 0x108)
#define SDM_RESP0    R64(SD + 0x110)
#define SDM_RESP1    R64(SD + 0x118)
#define SDM_BLK      R64(SD + 0x120)
#define SDM_STATUS   R64(SD + 0x128)
#define SDM_TIMEOUT  R64(SD + 0x138)
#define SDM_CFG      R64(SD + 0x140)
#define SDM_FIFO     R64(SD + 0x148)
#define SDM_DMA      R64(SD + 0x150)
#define SDM_BLKDONE  R64(SD + 0x158)
#define SDM_DATA     R64(SD + 0x180)
#define SDM_CAPS     R64(SD + 0x1F8)
#define CPU_FEATURES R64(0xF00F0048u)
#define CLOCK_MS     R64(0xF00F0040u)
#define CACHE_CTRL   R64(0xF0050000u)

#define ST_DMA_BUSY  (1u << 5)
#define ST_IDLY_RDY  (1u << 6)
#define ST_CMD_DONE  (1u << 8)
#define ST_DATA_DONE (1u << 9)
#define ST_THRESH    (1u << 4)
#define ST_ERR       0x0FFF0000u

#define CFG_W4    1u
#define CFG_HS    2u
#define CFG_CLKON 4u
#define CFG_DMA   (1u << 16)
#define CFG_HSAT(p, tap) (CFG_W4 | CFG_HS | ((uint32_t)(p) << 4) | ((uint32_t)(tap) << 8))

enum { RT_NONE = 0, RT_R1 = 1, RT_R1B = 2, RT_R2 = 3, RT_R3 = 4 };
#define F_DATA  (1u << 11)
#define F_STOP  (1u << 13)

#ifndef OFF_MS
#define OFF_MS 30
#endif
#ifndef SIZE_MIB
#define SIZE_MIB 8
#endif
#ifndef SWEEP_BLKS
#define SWEEP_BLKS 128          // 64 KiB a try
#endif
#ifndef SWEEP_REP
#define SWEEP_REP 4
#endif
#ifndef REPEAT
#define REPEAT 3
#endif
#ifndef VERIFY_BLKS
#define VERIFY_BLKS 1024        // the chosen setting's parked read (512 KiB, ~1016 parks)
#endif
#define NPH 7
static const uint8_t TAPS[] = { 0, 8, 16, 24, 31 };
#define NTAP ((int)(sizeof TAPS / sizeof TAPS[0]))

#define CTRL(div, pwr, mode) ((uint64_t)(div) | (1u << 16) | ((uint64_t)(pwr) << 17) | ((uint64_t)(mode) << 18))
#define NBLK ((uint32_t)SIZE_MIB * 2048u)

static uint8_t *const REF = (uint8_t *)(uintptr_t)0x01000000u;    // the 25 MHz reference
static uint8_t *const BUF = (uint8_t *)(uintptr_t)0x02000000u;    // reads under test (16 MiB max)
static int fails;
static uint32_t rca, ccs;

#define CHECK(c, ...) do { if (!(c)) { fails++; printf("SDHS FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static void delay_ms(uint32_t ms) { uint64_t t = CLOCK_MS; while (CLOCK_MS - t < ms) ; }

// 64-bit FNV-1a over 64-bit words (fast enough at ~100 MB/s)
static uint64_t csum(const uint8_t *p, uint32_t n) {
    const uint64_t *w = (const uint64_t *)p;
    uint64_t h = 0xCBF29CE484222325ull;
    for (uint32_t i = 0; i < n / 8; i++) { h ^= w[i]; h *= 0x100000001B3ull; }
    return h;
}

// a command; wait for CMD_DONE; the error bits (0 = OK)
static uint32_t cmd(uint32_t idx, uint32_t arg, uint32_t rt, uint32_t fl) {
    SDM_STATUS = 0xFFFFFFFFu;
    SDM_ARG = arg;
    SDM_CMD = idx | (rt << 8) | fl;
    uint64_t t = CLOCK_MS;
    while (!(SDM_STATUS & ST_CMD_DONE))
        if (CLOCK_MS - t > 2000) return 0x80000000u;
    return (uint32_t)SDM_STATUS & ST_ERR;
}
static uint32_t acmd(uint32_t idx, uint32_t arg, uint32_t rt, uint32_t fl) {
    uint32_t e = cmd(55, rca << 16, RT_R1, 0);
    return e ? e : cmd(idx, arg, rt, fl);
}
static uint32_t wait_data(uint32_t ms) {
    uint64_t t = CLOCK_MS;
    for (;;) {
        uint32_t s = (uint32_t)SDM_STATUS;
        if (s & ST_ERR) return s & ST_ERR;
        if (s & ST_DATA_DONE) return 0;
        if (CLOCK_MS - t > ms) return 0x80000000u;
    }
}

// PIO read: nblk blocks of bsz bytes (cmd idx, card address arg) into dst
static uint32_t pio_read(uint32_t idx, uint32_t arg, uint32_t nblk, uint32_t bsz, uint8_t *dst, int app) {
    uint32_t words = (bsz + 7) / 8;
    SDM_FIFO = 1u << 31;
    SDM_BLK = ((uint64_t)nblk << 16) | bsz;
    uint32_t e = app ? acmd(idx, arg, RT_R1, F_DATA) : cmd(idx, arg, RT_R1, F_DATA);
    if (e) return e;
    uint64_t *d = (uint64_t *)dst, tmp[64];
    for (uint32_t b = 0; b < nblk; b++) {
        uint64_t t = CLOCK_MS;
        while (!(SDM_STATUS & ST_THRESH)) {
            uint32_t s = (uint32_t)SDM_STATUS;
            if (s & ST_ERR) return s & ST_ERR;
            if (CLOCK_MS - t > 500) return 0x80000000u;
        }
        if (bsz != 512) {
            for (uint32_t w = 0; w < words; w++) tmp[w] = SDM_DATA;
            memcpy(dst + (size_t)b * bsz, tmp, bsz);
        } else {
            for (uint32_t w = 0; w < 64; w += 8) {
                d[0] = SDM_DATA; d[1] = SDM_DATA; d[2] = SDM_DATA; d[3] = SDM_DATA;
                d[4] = SDM_DATA; d[5] = SDM_DATA; d[6] = SDM_DATA; d[7] = SDM_DATA;
                d += 8;
            }
        }
    }
    e = wait_data(500);
    if (e) return e;
    if ((uint32_t)SDM_BLKDONE != nblk) return 0x40000000u;
    if (idx == 18) return cmd(12, 0, RT_R1B, F_STOP);
    return 0;
}
static uint32_t rd(uint32_t lba, uint32_t n, uint8_t *dst) {
    return pio_read(n > 1 ? 18 : 17, ccs ? lba : lba * 512, n, 512, dst, 0);
}

// A CMD18 of n blocks with the drain held back, so that CLK parks: the FIFO
// (8 blocks, 512 words) fills before the first block is taken, and after
// each block the drain waits until the FIFO is full again while 8 or more
// blocks are still to come, i.e. CLK parks after the end bit of nearly
// every block (n - 8 parks).  rd() drains about as fast as the bus fills
// and may never park; a phase that takes the bit launched one clock early
// (SD_NATIVE_PLAN.md §9.3) passes rd() and fails here (DATA_CRC + DATA_END
// on the block after the first park).
static uint32_t rd_parked(uint32_t lba, uint32_t n, uint8_t *dst) {
    SDM_FIFO = 1u << 31;
    SDM_BLK = ((uint64_t)n << 16) | 512;
    uint32_t e = cmd(18, ccs ? lba : lba * 512, RT_R1, F_DATA);
    if (e) return e;
    uint64_t *d = (uint64_t *)dst;
    for (uint32_t b = 0; b < n; b++) {
        uint32_t want = (n - b >= 8) ? 512u : 64u;               // words: full, or this block
        uint64_t t = CLOCK_MS;
        while (((uint32_t)SDM_FIFO & 0x3FFu) < want) {
            uint32_t s = (uint32_t)SDM_STATUS;
            if (s & ST_ERR) return s & ST_ERR;
            if (CLOCK_MS - t > 500) return 0x80000000u;
        }
        for (uint32_t w = 0; w < 64; w += 8) {
            d[0] = SDM_DATA; d[1] = SDM_DATA; d[2] = SDM_DATA; d[3] = SDM_DATA;
            d[4] = SDM_DATA; d[5] = SDM_DATA; d[6] = SDM_DATA; d[7] = SDM_DATA;
            d += 8;
        }
    }
    e = wait_data(500);
    if (e) return e;
    if ((uint32_t)SDM_BLKDONE != n) return 0x40000000u;
    return cmd(12, 0, RT_R1B, F_STOP);
}

// DMA read (SD DMA spec v1.2) of n blocks into DDR at pa; the caller
// invalidates the D-cache around it.  cfg = the current SDM_CFG (HS fields
// included): both writes here carry it, the DMA_EN = 0 one too (each
// SDM_CFG write reloads the IDELAY tap)
static uint32_t dma_rd(uint32_t lba, uint32_t n, uint32_t pa, uint32_t cfg) {
    SDM_FIFO = 1u << 31;
    SDM_DMA = pa;
    SDM_BLK = ((uint64_t)n << 16) | 512;
    SDM_CFG = cfg | CFG_DMA;
    uint32_t e = cmd(18, ccs ? lba : lba * 512, RT_R1, F_DATA);
    if (!e) e = wait_data(3000);
    uint64_t t = CLOCK_MS;
    while ((SDM_STATUS & ST_DMA_BUSY) && CLOCK_MS - t < 1000) ;
    if (!e && (SDM_STATUS & ST_DMA_BUSY)) e = 0x20000000u;
    if (!e && (uint32_t)SDM_BLKDONE != n) e = 0x40000000u;
    uint32_t e2 = cmd(12, 0, RT_R1B, F_STOP);
    SDM_CFG = cfg;
    return e ? e : e2;
}

// after a failed try at a bad phase: stop whatever the card is sending (it
// hears commands fine: the host's outputs do not depend on the phase), let
// it finish with CLK running, and empty the FIFO.  cfg = the current SDM_CFG
// (the CLK_ON writes keep its HS fields)
static void recover(uint32_t cfg) {
    (void)cmd(12, 0, RT_R1B, F_STOP);
    SDM_CFG = cfg | CFG_CLKON;
    delay_ms(2);
    SDM_CFG = cfg;
    SDM_FIFO = 1u << 31;
    SDM_STATUS = 0xFFFFFFFFu;
}

// CMD6 with its 64-byte status; returns the error bits
static uint32_t cmd6(uint32_t arg, uint8_t st[64]) {
    return pio_read(6, arg, 1, 64, st, 0);
}

// (the end: back to SDM_CFG = 0, default-speed sampling, tap 0, on purpose)
static void power_off(void) { SDM_CFG = 0; SD_CTRL = CTRL(249, 0, 0); }

static uint32_t mbps(uint64_t bytes, uint64_t ms) { return ms ? (uint32_t)(bytes * 10 / ms / 1000) : 0; }  // tenths of MB/s

// The setting: every (phase, tap) is a sample instant T = 10 + 5p - 0.078
// tap ns after CLK's rising edge, periodic in the 20 ns clock (p and p + 4
// are the same instant, one bit apart).  Find the widest gap between
// failing instants (in ps, modulo 20000) that holds passing ones, and take
// the passing instant nearest its centre (the smaller tap on a tie).
// Then the alias (SD_NATIVE_PLAN.md §9.3): a phase p <= 2 samples bit j - 1
// in slot j when the card's output hold plus the pad delays reach past
// T = 10 + 5p ns, and such a phase loses the next block's start bit after a
// CLK park (a block read fails with DATA_CRC + DATA_END once the FIFO
// fills), while a single block reads fine.  p + 4 is the same instant one
// bit later and never takes bit j - 1 (nor does p = 3): so when the chosen
// p <= 2 and p + 4 also passes at the same tap, take p + 4.
// pass[ti][p] == SWEEP_REP: every try passed.  (A reference for a driver's
// calibration: the HOST_TEST build exports it as sdhs_pick() below.)
static void pick(uint8_t pass[NTAP][NPH], int ntap, int *best_p, int *best_t,
                 int32_t *centre_o, int32_t *width_o, int32_t *bestd_o) {
    int np = 0;
    int32_t tps[NTAP * NPH];
    uint8_t ok[NTAP * NPH], pp[NTAP * NPH], tt[NTAP * NPH];
    for (int ti = 0; ti < ntap; ti++)
        for (int p = 0; p < NPH; p++) {
            int32_t t = 10000 + 5000 * p - 78 * TAPS[ti];
            tps[np] = ((t % 20000) + 20000) % 20000;
            ok[np] = pass[ti][p] == SWEEP_REP; pp[np] = (uint8_t)p; tt[np] = TAPS[ti];
            np++;
        }
    int nfail = 0;
    for (int i = 0; i < np; i++) nfail += !ok[i];
    int32_t centre = -1, width = 0;
    if (nfail == 0) {
        centre = 15000; width = 20000;                         // everything passes
    } else {
        for (int i = 0; i < np; i++) {
            if (ok[i]) continue;
            int32_t nxt = 20000;                               // to the next failing instant
            for (int j = 0; j < np; j++) {
                if (ok[j] || j == i) continue;
                int32_t d = (tps[j] - tps[i] + 20000) % 20000;
                if (d > 0 && d < nxt) nxt = d;
            }
            int any = 0;
            for (int j = 0; j < np; j++) {
                int32_t d = (tps[j] - tps[i] + 20000) % 20000;
                if (ok[j] && d > 0 && d < nxt) any = 1;
            }
            if (any && nxt > width) { width = nxt; centre = (tps[i] + nxt / 2) % 20000; }
        }
    }
    int32_t bestd = 1 << 30;
    int bi = -1;
    *best_p = -1; *best_t = -1;
    for (int i = 0; i < np; i++) {
        if (!ok[i] || centre < 0) continue;
        int32_t d = (tps[i] - centre + 20000) % 20000;
        if (d > 10000) d = 20000 - d;
        if (d < bestd || (d == bestd && tt[i] < *best_t)) { bestd = d; bi = i; *best_p = pp[i]; *best_t = tt[i]; }
    }
    // the alias rule: p <= 2 may take bit j - 1; p + 4 samples the same
    // instant one bit later (entry i = ti * NPH + p, so p + 4 is i + 4)
    if (bi >= 0 && pp[bi] <= 2 && ok[bi + 4]) *best_p = pp[bi] + 4;
    *centre_o = centre; *width_o = width; *bestd_o = bestd;
}

#ifdef HOST_TEST
// pick() for tb_sd_native (DPI-C: xsc -DHOST_TEST) and host checks:
// pass01[ti * NPH + p] != 0 when every try at TAPS[ti], phase p passed
// (ti < ntap <= NTAP); the setting, and the centre (ps mod 20000, -1: none)
void sdhs_pick(const char *pass01, int ntap, int *best_p, int *best_t, int *centre) {
    uint8_t pass[NTAP][NPH];
    int32_t c, w, d;
    for (int ti = 0; ti < NTAP; ti++)
        for (int p = 0; p < NPH; p++) pass[ti][p] = (ti < ntap && pass01[ti * NPH + p]) ? SWEEP_REP : 0;
    pick(pass, ntap, best_p, best_t, &c, &w, &d);
    *centre = c;
}
#endif

#ifndef HOST_TEST
int main(void) {
    uint64_t f = CPU_FEATURES, caps = SDM_CAPS;
    printf("SDHS CPU_FEATURES %016lx SDM_CAPS %016lx\n", (unsigned long)f, (unsigned long)caps);
    if (!(f & (1u << 21)) || !(caps & (1u << 9))) {
        printf("SDHS FAIL: no native SD controller with high speed (CPU_FEATURES[21], SDM_CAPS[9])\nSDHS FAIL\n");
        return 1;
    }
    int has_dly = (caps >> 11) & 1, has_dma = (caps >> 10) & 1 && (f >> 27) & 1;
    printf("SDHS %u sample phases, IDELAY %s, DMA %s, IDELAY_RDY %lu\n", (unsigned)((caps >> 24) & 0xF),
           has_dly ? "yes" : "no", has_dma ? "yes" : "no", (unsigned long)((SDM_STATUS >> 6) & 1));

    // ---- identification, 4-bit, 25 MHz
    SD_CTRL = CTRL(124, 0, 1);
    delay_ms(OFF_MS);
    SD_CTRL = CTRL(124, 1, 1);
    delay_ms(5);
    SDM_CFG = CFG_CLKON; delay_ms(2); SDM_CFG = 0;
    uint32_t e = cmd(0, 0, RT_NONE, 0);
    e |= cmd(8, 0x1AA, RT_R1, 0);
    uint32_t ocr = 0;
    rca = 0;
    for (int i = 0; i < 1000; i++) {
        e = acmd(41, 0x40FF8000u, RT_R3, 0);
        ocr = (uint32_t)SDM_RESP0;
        if (e || (ocr & 0x80000000u)) break;
        delay_ms(2);
    }
    CHECK(e == 0 && (ocr & 0x80000000u), "ACMD41 e=%08x ocr=%08x", e, ocr);
    ccs = (ocr >> 30) & 1;
    e = cmd(2, 0, RT_R2, 0);
    e |= cmd(3, 0, RT_R1, 0);
    rca = (uint32_t)(SDM_RESP0 >> 16) & 0xFFFF;
    e |= cmd(7, rca << 16, RT_R1B, 0);
    e |= acmd(6, 2, RT_R1, 0);
    SDM_CFG = CFG_W4;
    SD_CTRL = CTRL(1, 1, 1);
    CHECK(e == 0, "identification e=%08x", e);
    if (fails) goto out;

    // ---- the region and its reference at 25 MHz
    uint32_t lba;
#ifdef LBA
    lba = LBA;
#else
    e = rd(0, 1, BUF);
    lba = BUF[0x1C6] | BUF[0x1C7] << 8 | BUF[0x1C8] << 16 | (uint32_t)BUF[0x1C9] << 24;
    if (e || BUF[510] != 0x55 || BUF[511] != 0xAA || lba == 0) lba = 2048;
#endif
    printf("SDHS region: LBA %u, %u MiB (%u blocks), read only\n", lba, SIZE_MIB, NBLK);
    uint64_t t0 = CLOCK_MS;
    e = rd(lba, NBLK, REF);
    uint64_t ms25 = CLOCK_MS - t0;
    CHECK(e == 0, "25 MHz reference read e=%08x", e);
    if (fails) goto out;
    uint64_t ref = csum(REF, NBLK * 512);
    e = rd(lba, NBLK, BUF);
    CHECK(e == 0 && csum(BUF, NBLK * 512) == ref, "25 MHz re-read differs (e=%08x)", e);
    printf("SDHS 25 MHz PIO: checksum %016lx, %lu ms = %u.%u MB/s\n", (unsigned long)ref, (unsigned long)ms25,
           mbps((uint64_t)NBLK * 512, ms25) / 10, mbps((uint64_t)NBLK * 512, ms25) % 10);
    if (fails) goto out;

    // ---- CMD6: check, then switch to high speed
    uint8_t st[64];
    e = cmd6(0x00FFFFF1u, st);
    printf("SDHS CMD6 check: e=%08x max %u mA, group 1 support %02x%02x, result %x, version %u\n",
           e, st[0] << 8 | st[1], st[12], st[13], st[16] & 0xF, st[17]);
    CHECK(e == 0, "CMD6 check e=%08x", e);
    CHECK((st[13] & 2) && (st[16] & 0xF) == 1, "the card does not offer high speed (group 1 support %02x, result %x)",
          st[13], st[16] & 0xF);
    if (fails) goto out;
    e = cmd6(0x80FFFFF1u, st);
    CHECK(e == 0 && (st[16] & 0xF) == 1, "CMD6 switch e=%08x result %x", e, st[16] & 0xF);
    if (fails) goto out;
    printf("SDHS CMD6 switch: function group 1 = high speed\n");
    SD_CTRL = CTRL(0, 1, 1);                                   // 50 MHz
    SDM_TIMEOUT = (25000000ull << 32) | 250000u;               // busy 500 ms; data 5 ms (sweep)

    // ---- calibration sweep
    static uint8_t pass[NTAP][NPH], pkfail[NTAP][NPH];
    static uint32_t errs[NTAP][NPH];
    int ntap = has_dly ? NTAP : 1;
    for (int ti = 0; ti < ntap; ti++)
        for (int p = 0; p < NPH; p++) {
            uint32_t cfg = CFG_HSAT(p, TAPS[ti]);
            SDM_CFG = cfg;
            pass[ti][p] = 0; errs[ti][p] = 0; pkfail[ti][p] = 0;
            for (int r = 0; r < SWEEP_REP; r++) {
                uint32_t off = (uint32_t)(r * SWEEP_BLKS) % (NBLK - SWEEP_BLKS + 1);
                memset(BUF, 0, SWEEP_BLKS * 512);
                e = r == 0 ? rd_parked(lba + off, SWEEP_BLKS, BUF)   // CLK parks after every block
                           : rd(lba + off, SWEEP_BLKS, BUF);
                if (!e && memcmp(BUF, REF + (size_t)off * 512, SWEEP_BLKS * 512)) e = 0x10000000u;   // data differ
                if (e) { errs[ti][p] |= e; pkfail[ti][p] |= r == 0; recover(cfg); }
                else pass[ti][p]++;
            }
        }
    printf("SDHS sweep: %d tries of %u blocks a cell, the first with CLK parked after every block; P = all good,\n"
           "SDHS   n = n good, then the error bits (* = the parked try failed)\n", SWEEP_REP, SWEEP_BLKS);
    printf("SDHS   phase (ns after CLK rise):");
    for (int ti = 0; ti < ntap; ti++) printf("   tap %2u       ", TAPS[ti]);
    printf("\n");
    for (int p = 0; p < NPH; p++) {
        printf("SDHS   %d (%2d ns)                 :", p, 10 + 5 * p);
        for (int ti = 0; ti < ntap; ti++) {
            if (pass[ti][p] == SWEEP_REP) printf("   P            ");
            else printf("   %d %08x%c  ", pass[ti][p], errs[ti][p], pkfail[ti][p] ? '*' : ' ');
        }
        printf("\n");
    }

    // ---- the setting
    int best_p, best_t;
    int32_t centre, width, bestd;
    pick(pass, ntap, &best_p, &best_t, &centre, &width, &bestd);
    CHECK(best_p >= 0, "no passing phase / tap");
    if (fails) goto out;
    uint32_t hs_cfg = CFG_HSAT(best_p, best_t);
    printf("SDHS passing window ~%ld.%ld ns wide (between failing instants), centre %ld.%ld ns mod 20\n",
           (long)(width / 1000), (long)(width % 1000 / 100), (long)(centre / 1000), (long)(centre % 1000 / 100));
    printf("SDHS chosen: phase %d, tap %d (sample %ld ps after CLK rise, %ld ps from the centre); SDM_CFG %08x\n",
           best_p, best_t, (long)(10000 + 5000 * best_p - 78 * best_t), (long)bestd, hs_cfg);

    // ---- the setting, verified with CLK parked after every block
    SDM_CFG = hs_cfg;
    SDM_TIMEOUT = (25000000ull << 32) | 5000000u;              // 100 ms / 500 ms at 50 MHz
    {
        uint32_t nv = VERIFY_BLKS < NBLK ? VERIFY_BLKS : NBLK;
        memset(BUF, 0, nv * 512);
        e = rd_parked(lba, nv, BUF);
        int same = e == 0 && memcmp(BUF, REF, nv * 512) == 0;
        printf("SDHS chosen setting, parked read of %u blocks: %s (e=%08x)\n", nv, same ? "same data" : "BAD", e);
        CHECK(same, "the chosen setting fails a parked multi-block read: e=%08x", e);
        if (e) recover(hs_cfg);
        if (fails) goto out;
    }

    // ---- 50 MHz: PIO, then DMA, each REPEAT times over the region
    for (int r = 0; r < REPEAT; r++) {
        memset(BUF, 0, NBLK * 512);
        t0 = CLOCK_MS;
        e = rd(lba, NBLK, BUF);
        uint64_t ms = CLOCK_MS - t0;
        uint64_t c = csum(BUF, NBLK * 512);
        CHECK(e == 0 && c == ref, "50 MHz PIO read %d: e=%08x checksum %016lx", r, e, (unsigned long)c);
        printf("SDHS 50 MHz PIO %d: %s, %lu ms = %u.%u MB/s\n", r, (e == 0 && c == ref) ? "same checksum" : "BAD",
               (unsigned long)ms, mbps((uint64_t)NBLK * 512, ms) / 10, mbps((uint64_t)NBLK * 512, ms) % 10);
        if (e) recover(hs_cfg);
    }
    if (has_dma) {
        for (int r = 0; r < REPEAT; r++) {
            memset(BUF, 0, NBLK * 512);
            CACHE_CTRL = 4;                                    // INVALIDATE (the zeroes to DDR, no stale lines)
            t0 = CLOCK_MS;
            e = dma_rd(lba, NBLK, (uint32_t)(uintptr_t)BUF, hs_cfg);
            uint64_t ms = CLOCK_MS - t0;
            CACHE_CTRL = 4;                                    // drop anything fetched meanwhile
            uint64_t c = csum(BUF, NBLK * 512);
            CHECK(e == 0 && c == ref, "50 MHz DMA read %d: e=%08x checksum %016lx", r, e, (unsigned long)c);
            printf("SDHS 50 MHz DMA %d: %s, %lu ms = %u.%u MB/s\n", r, (e == 0 && c == ref) ? "same checksum" : "BAD",
                   (unsigned long)ms, mbps((uint64_t)NBLK * 512, ms) / 10, mbps((uint64_t)NBLK * 512, ms) % 10);
            if (e) recover(hs_cfg);
        }
    }
    // a last 25 MHz-equivalent sanity check: CMD13 status in HS
    e = cmd(13, rca << 16, RT_R1, 0);
    CHECK(e == 0 && ((SDM_RESP0 >> 9) & 0xF) == 4, "CMD13 at 50 MHz e=%08x state %lu", e,
          (unsigned long)((SDM_RESP0 >> 9) & 0xF));

out:
    power_off();
    printf("SDHS done fails=%d\n", fails);
    printf(fails ? "SDHS FAIL\n" : "SDHS PASS\n");
    return fails != 0;
}
#endif

// sd_native_test.c — SD_NATIVE_PLAN.md phase 2: bare-metal board test of the
// native 4-bit SD controller (mmu-sd).  Polled, no interrupts.
//   - identification at 400 kHz (CMD0/8/ACMD41/2/3/9/7), then ACMD6 4-bit, 25 MHz;
//   - ACMD51 (SCR, 8-byte block) and ACMD13 (SD status, 64 B);
//   - MBR read (CMD17) in 1-bit and in 4-bit mode, compared;
//   - CMD18 multi-block vs CMD17 single-block on the same blocks, compared;
//   - read throughput (CMD18, 1 MiB);
//   - write test ONLY in the gap before the first partition (LBA 1000..1063,
//     refused unless partition 1 starts at >= 2048): save, CMD25 a pattern,
//     read back, ACMD22 (4-byte block), restore the originals, verify;
//   - write throughput from that test.
// Leaves the slot powered off in SPI mode.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define R64(a)     (*(volatile uint64_t *)(uintptr_t)(a))
#define SD         0xF0000000u
#define SD_CTRL    R64(SD + 0x000)
#define SDM_CMD    R64(SD + 0x100)
#define SDM_ARG    R64(SD + 0x108)
#define SDM_RESP0  R64(SD + 0x110)
#define SDM_RESP1  R64(SD + 0x118)
#define SDM_BLK    R64(SD + 0x120)
#define SDM_STATUS R64(SD + 0x128)
#define SDM_CFG    R64(SD + 0x140)
#define SDM_FIFO   R64(SD + 0x148)
#define SDM_BLKDONE R64(SD + 0x158)
#define SDM_DATA   R64(SD + 0x180)
#define SDM_CAPS   R64(SD + 0x1F8)
#define CPU_FEATURES R64(0xF00F0048u)
#define CLOCK_MS   R64(0xF00F0040u)

#define ST_CMD_DONE  (1u << 8)
#define ST_DATA_DONE (1u << 9)
#define ST_THRESH    (1u << 4)
#define ST_ERR       0x07FF0000u

enum { RT_NONE = 0, RT_R1 = 1, RT_R1B = 2, RT_R2 = 3, RT_R3 = 4 };
#define F_DATA  (1u << 11)
#define F_WRITE (1u << 12)
#define F_STOP  (1u << 13)

#ifndef OFF_MS
#define OFF_MS 30
#endif
#define CTRL(div, pwr, mode) ((uint64_t)(div) | (1u << 16) | ((uint64_t)(pwr) << 17) | ((uint64_t)(mode) << 18))

static uint8_t *const BUF  = (uint8_t *)(uintptr_t)0x00400000u;   // 1 MiB
static uint8_t *const BUF2 = (uint8_t *)(uintptr_t)0x00500000u;   // 1 MiB
static uint8_t *const SAVE = (uint8_t *)(uintptr_t)0x00600000u;   // 32 KiB
static int fails;
static uint32_t rca, ccs;

#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL " __VA_ARGS__); printf("\n"); } } while (0)

static void delay_ms(uint32_t ms) { uint64_t t = CLOCK_MS; while (CLOCK_MS - t < ms) ; }

// Send a command; wait for CMD_DONE.  Returns the error bits (0 = OK).
static uint32_t cmd(uint32_t idx, uint32_t arg, uint32_t rt, uint32_t fl) {
    SDM_STATUS = 0xFFFFFFFFu;                         // W1C everything sticky
    SDM_ARG = arg;
    SDM_CMD = idx | (rt << 8) | fl;
    uint64_t t = CLOCK_MS;
    while (!(SDM_STATUS & ST_CMD_DONE))
        if (CLOCK_MS - t > 2000) { printf("  cmd%u: no CMD_DONE (status %08lx)\n", idx, (unsigned long)SDM_STATUS); return 0x80000000u; }
    return (uint32_t)SDM_STATUS & ST_ERR;
}

static uint32_t acmd(uint32_t idx, uint32_t arg, uint32_t rt, uint32_t fl) {
    uint32_t e = cmd(55, rca << 16, RT_R1, 0);
    if (e) return e;
    return cmd(idx, arg, rt, fl);
}

// Wait for DATA_DONE or an error.  Returns error bits.
static uint32_t wait_data(void) {
    uint64_t t = CLOCK_MS;
    for (;;) {
        uint32_t s = (uint32_t)SDM_STATUS;
        if (s & ST_ERR) return s & ST_ERR;
        if (s & ST_DATA_DONE) return 0;
        if (CLOCK_MS - t > 3000) { printf("  data: timeout (status %08x)\n", s); return 0x80000000u; }
    }
}

// Read nblk blocks of bsz bytes with the given command (data already set up).
static uint32_t read_blocks(uint32_t idx, uint32_t arg, uint32_t nblk, uint32_t bsz, uint8_t *dst, int app) {
    uint32_t words = (bsz + 7) / 8;
    SDM_FIFO = 1u << 31;
    SDM_BLK = ((uint64_t)nblk << 16) | bsz;
    uint32_t e = app ? acmd(idx, arg, RT_R1, F_DATA) : cmd(idx, arg, RT_R1, F_DATA);
    if (e) return e;
    uint64_t *d = (uint64_t *)dst;
    uint64_t tmp[64];
    for (uint32_t b = 0; b < nblk; b++) {
        uint64_t t = CLOCK_MS;
        while (!(SDM_STATUS & ST_THRESH)) {
            uint32_t s = (uint32_t)SDM_STATUS;
            if (s & ST_ERR) return s & ST_ERR;
            if (CLOCK_MS - t > 3000) { printf("  read blk %u: no THRESH (status %08x)\n", b, s); return 0x80000000u; }
        }
        if (bsz != 512) {                             // other sizes: unpack via tmp
            for (uint32_t w = 0; w < words; w++) tmp[w] = SDM_DATA;
            memcpy(dst + (size_t)b * bsz, tmp, bsz);
        } else {
            for (uint32_t w = 0; w < words; w += 8) {
                d[0] = SDM_DATA; d[1] = SDM_DATA; d[2] = SDM_DATA; d[3] = SDM_DATA;
                d[4] = SDM_DATA; d[5] = SDM_DATA; d[6] = SDM_DATA; d[7] = SDM_DATA;
                d += 8;
            }
        }
    }
    e = wait_data();
    if (e) return e;
    if ((uint32_t)SDM_BLKDONE != nblk) { printf("  BLKDONE %lu != %u\n", (unsigned long)SDM_BLKDONE, nblk); return 0x40000000u; }
    if (idx == 18) return cmd(12, 0, RT_R1B, F_STOP);
    return 0;
}

static uint32_t write_blocks(uint32_t lba, uint32_t nblk, const uint8_t *src) {
    SDM_FIFO = 1u << 31;
    SDM_BLK = ((uint64_t)nblk << 16) | 512;
    uint32_t e = cmd(nblk > 1 ? 25 : 24, ccs ? lba : lba * 512, RT_R1, F_DATA | F_WRITE);
    if (e) return e;
    const uint64_t *s = (const uint64_t *)src;
    for (uint32_t b = 0; b < nblk; b++) {
        uint64_t t = CLOCK_MS;
        while (!(SDM_STATUS & ST_THRESH)) {
            uint32_t st = (uint32_t)SDM_STATUS;
            if (st & ST_ERR) return st & ST_ERR;
            if (CLOCK_MS - t > 3000) { printf("  write blk %u: no THRESH (status %08x)\n", b, st); return 0x80000000u; }
        }
        for (uint32_t w = 0; w < 64; w += 8) {
            SDM_DATA = s[0]; SDM_DATA = s[1]; SDM_DATA = s[2]; SDM_DATA = s[3];
            SDM_DATA = s[4]; SDM_DATA = s[5]; SDM_DATA = s[6]; SDM_DATA = s[7];
            s += 8;
        }
    }
    e = wait_data();
    if (e) return e;
    if ((uint32_t)SDM_BLKDONE != nblk) { printf("  write BLKDONE %lu != %u\n", (unsigned long)SDM_BLKDONE, nblk); return 0x40000000u; }
    if (nblk > 1) return cmd(12, 0, RT_R1B, F_STOP);
    return 0;
}

static uint32_t rd(uint32_t lba, uint32_t n, uint8_t *dst) {
    return read_blocks(n > 1 ? 18 : 17, ccs ? lba : lba * 512, n, 512, dst, 0);
}

static void power_off(void) { SD_CTRL = CTRL(249, 0, 0); }

int main(void) {
    uint64_t f = CPU_FEATURES;
    printf("SDN CPU_FEATURES %016lx CAPS %016lx\n", (unsigned long)f, (unsigned long)SDM_CAPS);
    if (!(f & (1u << 21))) { printf("SDN FAIL no native SD controller\nSDN done fails=1\n"); return 1; }

    // Power cycle into SD mode, 400 kHz.
    SD_CTRL = CTRL(124, 0, 1);
    delay_ms(OFF_MS);
    SD_CTRL = CTRL(124, 1, 1);
    delay_ms(5);
    SDM_CFG = 4;                                       // CLK_ON: >= 74 clocks
    delay_ms(2);
    SDM_CFG = 0;

    uint32_t e = cmd(0, 0, RT_NONE, 0);
    CHECK(e == 0, "CMD0 %08x", e);
    e = cmd(8, 0x1AA, RT_R1, 0);
    CHECK(e == 0 && (SDM_RESP0 & 0xFFF) == 0x1AA, "CMD8 e=%08x resp=%08lx", e, (unsigned long)SDM_RESP0);
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
    CHECK(e == 0, "CMD2 %08x", e);
    printf("SDN OCR %08x CCS %u CID %016lx%016lx\n", ocr, ccs, (unsigned long)SDM_RESP1, (unsigned long)SDM_RESP0);
    e = cmd(3, 0, RT_R1, 0);
    rca = (uint32_t)(SDM_RESP0 >> 16) & 0xFFFF;
    CHECK(e == 0 && rca, "CMD3 e=%08x rca=%04x", e, rca);
    e = cmd(9, rca << 16, RT_R2, 0);
    uint64_t c1 = SDM_RESP1, c0 = SDM_RESP0;
    CHECK(e == 0, "CMD9 %08x", e);
    uint32_t csize = (uint32_t)(((c1 & 0x3F) << 16) | ((c0 >> 48) & 0xFFFF));   // CSD v2: C_SIZE = bits [69:48]
    if (ccs) printf("SDN CSD %016lx%016lx capacity %lu MiB\n", (unsigned long)c1, (unsigned long)c0, (unsigned long)(((uint64_t)csize + 1) / 2));
    e = cmd(7, rca << 16, RT_R1B, 0);
    CHECK(e == 0, "CMD7 %08x", e);

    // 1-bit, 400 kHz -> 25 MHz, still 1-bit: MBR
    SD_CTRL = CTRL(1, 1, 1);
    e = rd(0, 1, BUF2);
    CHECK(e == 0 && BUF2[510] == 0x55 && BUF2[511] == 0xAA, "MBR 1-bit e=%08x sig %02x%02x", e, BUF2[510], BUF2[511]);

    // 4-bit
    e = acmd(6, 2, RT_R1, 0);
    CHECK(e == 0, "ACMD6 %08x", e);
    SDM_CFG = 1;
    e = rd(0, 1, BUF);
    CHECK(e == 0 && memcmp(BUF, BUF2, 512) == 0, "MBR 4-bit e=%08x (differs from 1-bit)", e);
    uint32_t p1 = BUF[0x1C6] | BUF[0x1C7] << 8 | BUF[0x1C8] << 16 | (uint32_t)BUF[0x1C9] << 24;
    printf("SDN partition 1 starts at LBA %u, type %02x\n", p1, BUF[0x1C2]);

    // SCR (8 bytes) and SD status (64 bytes)
    uint8_t scr[8], sds[64];
    e = read_blocks(51, 0, 1, 8, scr, 1);
    CHECK(e == 0, "ACMD51 %08x", e);
    printf("SDN SCR %02x%02x%02x%02x%02x%02x%02x%02x (bus widths %x)\n", scr[0], scr[1], scr[2], scr[3], scr[4], scr[5], scr[6], scr[7], scr[1] & 0xF);
    CHECK((scr[1] & 0x5) == 0x5, "SCR bus widths %x", scr[1] & 0xF);
    e = read_blocks(13, 0, 1, 64, sds, 1);
    CHECK(e == 0 && (sds[0] >> 6) == 2, "ACMD13 e=%08x bus width field %u", e, sds[0] >> 6);

    // CMD18 vs CMD17 on blocks 0..63
    e = rd(0, 64, BUF);
    CHECK(e == 0, "CMD18 x64 %08x", e);
    for (uint32_t b = 0; b < 64 && !e; b++) {
        e = rd(b, 1, BUF2 + b * 512);
        CHECK(e == 0, "CMD17 blk %u %08x", b, e);
    }
    CHECK(memcmp(BUF, BUF2, 64 * 512) == 0, "CMD18 data != CMD17 data");

    // Read throughput: 1 MiB from LBA p1 (inside the partition, read only)
    uint64_t t0 = CLOCK_MS;
    e = rd(p1 ? p1 : 0, 2048, BUF);
    uint64_t ms = CLOCK_MS - t0;
    CHECK(e == 0, "CMD18 x2048 %08x", e);
    if (!e) printf("SDN read 1 MiB in %lu ms = %lu KB/s\n", (unsigned long)ms, (unsigned long)(ms ? 1048576ull / ms : 0));
    // Re-read and compare (stability)
    e = rd(p1 ? p1 : 0, 2048, BUF2);
    CHECK(e == 0 && memcmp(BUF, BUF2, 1u << 20) == 0, "1 MiB re-read differs e=%08x", e);

    // Write test in the pre-partition gap only.
    const uint32_t WL = 1000, WN = 64;
    if (p1 >= 2048) {
        e = rd(WL, WN, SAVE);
        CHECK(e == 0, "save read %08x", e);
        for (uint32_t i = 0; i < WN * 512; i++) BUF[i] = (uint8_t)(i * 7 + (i >> 9) + 0x5A);
        t0 = CLOCK_MS;
        e = write_blocks(WL, WN, BUF);
        ms = CLOCK_MS - t0;
        CHECK(e == 0, "CMD25 x%u %08x", WN, e);
        if (!e) printf("SDN write 32 KiB in %lu ms = %lu KB/s\n", (unsigned long)ms, (unsigned long)(ms ? 32768ull / ms : 0));
        uint8_t nwb[4];
        e = read_blocks(22, 0, 1, 4, nwb, 1);
        uint32_t nw = (uint32_t)nwb[0] << 24 | nwb[1] << 16 | nwb[2] << 8 | nwb[3];
        CHECK(e == 0 && nw == WN, "ACMD22 e=%08x written=%u", e, nw);
        e = rd(WL, WN, BUF2);
        CHECK(e == 0 && memcmp(BUF, BUF2, WN * 512) == 0, "write read-back differs e=%08x", e);
        e = write_blocks(WL, 1, SAVE);                 // single-block write (CMD24)
        CHECK(e == 0, "CMD24 restore %08x", e);
        e = write_blocks(WL + 1, WN - 1, SAVE + 512);
        CHECK(e == 0, "CMD25 restore %08x", e);
        e = rd(WL, WN, BUF2);
        CHECK(e == 0 && memcmp(SAVE, BUF2, WN * 512) == 0, "restore read-back differs e=%08x", e);
    } else {
        printf("SDN write test skipped: partition 1 starts at %u (< 2048)\n", p1);
    }

    power_off();
    printf("SDN done fails=%d\n", fails);
    return fails != 0;
}

// qspi_test.c — on-board test of qspi_ctrl (MMIO 0xF012_0000, the S25FL128S
// configuration flash).  See MMIO_MAP.md "QSPI flash".
//
// Uses only the test-scratch sector at 0xFC_0000..0xFF_FFFF (the last 256 KiB;
// the keystore partition is 0xF0_0000..0xFB_FFFF and is never touched here).
//
//   default      : ID/status reads, bitstream header read, guard checks
//                  (program/erase below the floor and non-whitelisted opcodes
//                  must be refused), erase + program + read-back in the
//                  scratch area, timings.  Leaves a known pattern at
//                  0xFF_0000 for VERIFY_ONLY.
//   -DVERIFY_ONLY: only check the pattern at 0xFF_0000 (run after a
//                  reconfiguration from flash / power cycle).
// Prints "=== ALL PASS ===" or "=== FAIL n ===".
// Build: tools/board/build.sh qspi_test -O1
//        SRC=qspi_test DEFS=-DVERIFY_ONLY tools/board/build.sh qspi_verify
#include <stdint.h>
#include <stdio.h>

#define QB          0xF0120000ul
#define Q(o)        (*(volatile uint64_t *)(QB + (o)))
#define Q_CMD       0x000
#define Q_ADDR      0x008
#define Q_STATUS    0x010
#define Q_CLKDIV    0x018
#define Q_FLOOR     0x020
#define Q_ID        0x028
#define Q_BUF       0x100
#define CYCLES      (*(volatile uint64_t *)0xF00D0008ul)

#define SCRATCH     0xFC0000u
#define PAT_ADDR    0xFF0000u

static int fails;

static void check(int ok, const char *what) {
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) fails++;
}

// Run one command; returns 0 if accepted, -1 if refused.
static int qcmd(uint8_t op, unsigned len, uint32_t addr) {
    Q(Q_ADDR) = addr;
    Q(Q_CMD) = (uint64_t)op | ((uint64_t)len << 16);
    while (Q(Q_STATUS) & 1) {}
    return (Q(Q_STATUS) & 2) ? -1 : 0;
}

static uint8_t bufb(unsigned i) {
    return (uint8_t)(Q(Q_BUF + (i & ~7u)) >> ((i & 7) * 8));
}

static uint8_t rdsr(uint8_t op) {
    qcmd(op, 1, 0);
    return bufb(0);
}

static int wait_wip(uint64_t *cyc) {
    uint64_t t0 = CYCLES;
    uint8_t sr;
    unsigned long n = 0;
    // P_ERR/E_ERR keep WIP set until CLSR: check them on every poll.
    do { sr = rdsr(0x05); n++; } while ((sr & 1) && !(sr & 0x60) && n < 50000000ul);
    if (cyc) *cyc = CYCLES - t0;
    if (sr & 0x60) { printf("  SR1 error bits %02x\n", sr); qcmd(0x30, 0, 0); return -1; }
    return (sr & 1) ? -1 : 0;
}

static uint8_t pat(uint32_t a) { return (uint8_t)((a * 7u) ^ (a >> 8) ^ 0x3Cu); }

static int read_and_check(uint32_t addr, unsigned len, int erased) {
    int bad = 0;
    if (qcmd(0x03, len, addr)) return -1;
    for (unsigned i = 0; i < len; i++) {
        uint8_t want = erased ? 0xFF : pat(addr + i);
        uint8_t got = bufb(i);
        if (got != want) { if (bad < 4) printf("  @%06lx got %02x want %02x\n", (unsigned long)(addr + i), got, want); bad++; }
    }
    return bad;
}

static void verify_pattern(void) {
    int bad = 0;
    for (uint32_t a = PAT_ADDR; a < PAT_ADDR + 1024; a += 256) bad += read_and_check(a, 256, 0);
    check(bad == 0, "pattern at 0xFF0000 (1 KiB) intact");
}

int main(void) {
    printf("qspi_test: ID %016llx FLOOR %06llx STATUS %llx\n",
           (unsigned long long)Q(Q_ID), (unsigned long long)Q(Q_FLOOR), (unsigned long long)Q(Q_STATUS));
    check(Q(Q_ID) == 0x5153504900000001ull, "QSPI_ID");
    check((Q(Q_STATUS) & 4) != 0, "engine ready");

#ifdef VERIFY_ONLY
    verify_pattern();
#else
    check(Q(Q_FLOOR) == 0xF00000, "floor 0xF00000");

    // ---- identification / status ----
    check(qcmd(0x9F, 6, 0) == 0, "RDID accepted");
    printf("  RDID %02x %02x %02x %02x %02x %02x\n", bufb(0), bufb(1), bufb(2), bufb(3), bufb(4), bufb(5));
    check(bufb(0) == 0x01 && bufb(1) == 0x20 && bufb(2) == 0x18, "RDID = S25FL128S (01 20 18)");
    uint8_t sr1 = rdsr(0x05), sr2 = rdsr(0x07), cr = rdsr(0x35), bar = rdsr(0x16);
    printf("  SR1 %02x SR2 %02x CR1 %02x BAR %02x  (sector arch %s, TBPARM %d)\n", sr1, sr2, cr, bar,
           bufb(4) == 0 ? "uniform 256K" : "64K + 4K param", (cr >> 2) & 1);
    check((bar & 0x80) == 0, "3-byte addressing (BAR.EXTADD = 0)");

    // ---- bitstream header at 0 (read is allowed) ----
    check(qcmd(0x03, 64, 0) == 0, "READ @0 accepted");
    int sync = 0;
    for (unsigned i = 0; i + 4 <= 64; i++)
        if (bufb(i) == 0xAA && bufb(i + 1) == 0x99 && bufb(i + 2) == 0x55 && bufb(i + 3) == 0x66) sync = 1;
    check(sync, "bitstream sync word AA995566 in first 64 bytes");

    // ---- guard: everything here must be refused without reaching the flash ----
    qcmd(0x06, 0, 0);                                   // WREN (allowed) to arm the flash
    check(qcmd(0x02, 4, 0x000000) < 0, "PP @0 refused");
    check(qcmd(0x02, 4, 0xEFFF00) < 0, "PP @0xEFFF00 refused");
    check(qcmd(0xD8, 0, 0x000000) < 0, "SE @0 refused");
    check(qcmd(0xD8, 0, 0xEFFFFF) < 0, "SE @0xEFFFFF refused");
    check(qcmd(0x20, 0, 0x000000) < 0, "P4E @0 refused");
    check(qcmd(0x60, 0, 0xF00000) < 0, "BE 60 refused");
    check(qcmd(0xC7, 0, 0xF00000) < 0, "BE C7 refused");
    check(qcmd(0x01, 2, 0xF00000) < 0, "WRR refused");
    check(qcmd(0x17, 1, 0xF00000) < 0, "BRWR refused");
    check(qcmd(0x12, 4, 0xF00000) < 0, "4PP refused");
    check(qcmd(0xDC, 0, 0xF00000) < 0, "4SE refused");
    check(qcmd(0x03, 257, 0xF00000) < 0, "len 257 refused");
    check(qcmd(0x03, 512, 0xF00000) < 0, "len 512 refused");
    qcmd(0x04, 0, 0);                                   // WRDI
    check((rdsr(0x05) & 2) == 0, "WEL clear after WRDI");
    check(rdsr(0x05) == sr1 && rdsr(0x35) == cr, "SR1/CR1 unchanged");

    // ---- erase + program + read back in the scratch sector ----
    uint64_t t_se, t_pp, t;
    qcmd(0x06, 0, 0);
    check(qcmd(0xD8, 0, PAT_ADDR) == 0, "SE @0xFF0000 accepted");
    check(wait_wip(&t_se) == 0, "SE completes");
    int bad = 0;
    for (uint32_t a = PAT_ADDR; a < PAT_ADDR + 4096; a += 256) bad += read_and_check(a, 256, 1);
    check(bad == 0, "erased (first 4 KiB reads FF)");

    for (uint32_t a = PAT_ADDR; a < PAT_ADDR + 1024; a += 256) {
        for (unsigned i = 0; i < 256; i += 8) {
            uint64_t w = 0;
            for (unsigned k = 0; k < 8; k++) w |= (uint64_t)pat(a + i + k) << (8 * k);
            Q(Q_BUF + i) = w;
        }
        qcmd(0x06, 0, 0);
        if (qcmd(0x02, 256, a)) { check(0, "PP accepted"); break; }
        if (wait_wip(&t_pp)) { check(0, "PP completes"); break; }
    }
    verify_pattern();

    // partial page + MMIO byte stores into the buffer
    qcmd(0x06, 0, 0); qcmd(0xD8, 0, SCRATCH); wait_wip(0);
    *(volatile uint8_t *)(QB + Q_BUF + 0) = 0xA5;
    *(volatile uint8_t *)(QB + Q_BUF + 1) = 0x5A;
    *(volatile uint8_t *)(QB + Q_BUF + 2) = 0x00;
    qcmd(0x06, 0, 0); qcmd(0x02, 3, SCRATCH + 0x10); wait_wip(0);
    qcmd(0x03, 4, SCRATCH + 0x10);
    check(bufb(0) == 0xA5 && bufb(1) == 0x5A && bufb(2) == 0x00 && bufb(3) == 0xFF, "3-byte program via byte stores");

    // ---- timings ----
    t = CYCLES;
    for (uint32_t a = 0; a < 65536; a += 256) qcmd(0x03, 256, a);
    t = CYCLES - t;
    printf("  SE 64K %llu ms, PP 256B %llu us, READ 64 KiB %llu us (%llu KB/s) @25 MHz\n",
           (unsigned long long)(t_se / 100000), (unsigned long long)(t_pp / 100),
           (unsigned long long)(t / 100), (unsigned long long)(65536ull * 100000000ull / t / 1000));
    Q(Q_CLKDIV) = 0;
    check(Q(Q_CLKDIV) == 1, "CLKDIV 0 clamps to 1 (no 50 MHz)");
    Q(Q_CLKDIV) = 3;
    bad = 0;
    for (uint32_t a = PAT_ADDR; a < PAT_ADDR + 1024; a += 256) bad += read_and_check(a, 256, 0);
    check(bad == 0, "read-back at 12.5 MHz");
    Q(Q_CLKDIV) = 1;

    // a write while busy is flagged and makes the next command fail
    Q(Q_ADDR) = 0; Q(Q_CMD) = 0x03 | (256ull << 16);
    Q(Q_ADDR) = PAT_ADDR;                               // dropped (busy)
    while (Q(Q_STATUS) & 1) {}
    check((Q(Q_STATUS) & 8) != 0, "dropped write flagged");
    check(qcmd(0x05, 1, 0) < 0, "CMD after dropped write refused");
    check(qcmd(0x05, 1, 0) == 0, "next CMD accepted");

    // the bitstream must still be there
    check(qcmd(0x03, 64, 0) == 0, "READ @0 again");
    sync = 0;
    for (unsigned i = 0; i + 4 <= 64; i++)
        if (bufb(i) == 0xAA && bufb(i + 1) == 0x99 && bufb(i + 2) == 0x55 && bufb(i + 3) == 0x66) sync = 1;
    check(sync, "bitstream header unchanged");
#endif
    if (fails) printf("=== FAIL %d ===\n", fails);
    else printf("=== ALL PASS ===\n");
    return fails;
}

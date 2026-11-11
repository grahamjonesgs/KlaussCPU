// dcbz_test.c — HW_PERF_PLAN.md §1 board check: DCBZ correctness and the
// cost of zeroing memory with DCBZ against four 64-bit stores per line.
//   correctness: hot (hit) and cold (miss) lines, every byte offset of the
//                EA, neighbours untouched, zeros reach DDR after a flush;
//   throughput:  64 KiB zeroed with clean victims (cache invalidated first)
//                and with dirty victims (another 64 KiB dirtied first).
// Prints "DCBZ done fails=N".
#include <stdint.h>
#include <stdio.h>
#include "mmio.h"

#define FEAT      (*(volatile uint64_t *)(uintptr_t)0xF00F0048u)
#define ZLINES    (*(volatile uint64_t *)(uintptr_t)(CACHE_BASE + 0x0070u))
#define CC_FLUSH  (1u << 1)
#define CC_INVAL  (1u << 2)
#define BUF_A     0x00400000u
#define BUF_B     0x00500000u
#define SPAN      0x10000u            /* 64 KiB = the whole D-cache */

static unsigned fails;
static uint32_t L;

static inline void dcbz(uintptr_t a) { __asm__ volatile("dcbz %0" ::"r"(a) : "memory"); }
static uint64_t now(void) { return REG_PERF_CYCLES; }
static void cache_wait(void) { while (*(volatile uint64_t *)(uintptr_t)(CACHE_BASE + 0x10u) & 1) { } }
static void invalidate(void) { REG_CACHE_CTRL = CC_INVAL; cache_wait(); }
static void flush(void) { REG_CACHE_CTRL = CC_FLUSH; cache_wait(); }

static uint8_t pat(uintptr_t a) { return (uint8_t)(a * 7u + (a >> 8) + 0x5Au) | 1u; }   /* never 0 */

static void fill(uintptr_t base, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) ((volatile uint8_t *)base)[i] = pat(base + i);
}

/* check [base, base+n): bytes in a zeroed line must be 0, others pat() */
static unsigned check(uintptr_t base, uint32_t n, const uint8_t *zeroed, const char *what) {
    unsigned bad = 0;
    for (uint32_t i = 0; i < n; i++) {
        uintptr_t a = base + i;
        uint8_t want = zeroed[i / L] ? 0 : pat(a);
        uint8_t got = ((volatile uint8_t *)base)[i];
        if (got != want && bad++ < 4) printf("  %s: @%08lx got %02x want %02x\n", what, (unsigned long)a, got, want);
    }
    return bad;
}

static void correctness(int cold) {
    static uint8_t zeroed[4096 / 8];
    uint32_t n = 4096, lines = n / L;
    for (uint32_t i = 0; i < lines; i++) zeroed[i] = 0;
    fill(BUF_A, n);
    if (cold) invalidate();               /* write back the pattern; every line misses */
    for (uint32_t i = 0; i < lines; i += 3) {     /* every third line, EA at offset i % L */
        dcbz(BUF_A + i * L + (i % L));
        zeroed[i] = 1;
    }
    unsigned b1 = check(BUF_A, n, zeroed, cold ? "cold" : "hot");
    invalidate();                         /* now read it back from DDR */
    unsigned b2 = check(BUF_A, n, zeroed, cold ? "cold/ddr" : "hot/ddr");
    printf("DCBZ correct %-4s lines=%lu zeroed=%lu bad=%u ddr_bad=%u\n", cold ? "cold" : "hot",
           (unsigned long)lines, (unsigned long)((lines + 2) / 3), b1, b2);
    fails += (b1 != 0) + (b2 != 0);
}

struct cnt { uint64_t cyc, zl, wm, rm, wb; };
static struct cnt snap(void) {
    struct cnt c = { now(), ZLINES, REG_CACHE_WR_MISSES, REG_CACHE_RD_MISSES, REG_CACHE_WRITEBACKS };
    return c;
}

static void zero_dcbz(uintptr_t b) { for (uint32_t o = 0; o < SPAN; o += L) dcbz(b + o); }
static void zero_st(uintptr_t b) {
    for (uint32_t o = 0; o < SPAN; o += 32) {
        volatile uint64_t *p = (volatile uint64_t *)(b + o);
        p[0] = 0; p[1] = 0; p[2] = 0; p[3] = 0;
    }
}

static void bench(const char *name, int use_dcbz, int dirty) {
    invalidate();
    if (dirty) fill(BUF_A, SPAN);         /* every cache line dirty with other data */
    __asm__ volatile("" ::: "memory");
    struct cnt a = snap();
    if (use_dcbz) zero_dcbz(BUF_B); else zero_st(BUF_B);
    uint64_t c0 = now() - a.cyc;          /* before the flush (the sim's figure) */
    flush();                              /* include writing the zeros out */
    struct cnt b = snap();
    uint64_t c = b.cyc - a.cyc, lines = SPAN / L;
    unsigned long mbx10 = c ? (unsigned long)((uint64_t)SPAN * 1000u / c) : 0;   /* MB/s x10 @100 MHz */
    printf("DCBZ bench %-12s %-5s noflush %6lu cyc (%lu.%lu/line)\n", name, dirty ? "dirty" : "clean",
           (unsigned long)c0, (unsigned long)(c0 * 10 / lines / 10), (unsigned long)(c0 * 10 / lines % 10));
    printf("DCBZ bench %-12s %-5s %7lu cyc %3lu.%lu cyc/line %4lu.%lu MB/s  zl=%lu wmiss=%lu rmiss=%lu wb=%lu\n",
           name, dirty ? "dirty" : "clean", (unsigned long)c, (unsigned long)(c * 10 / lines / 10),
           (unsigned long)(c * 10 / lines % 10), mbx10 / 10, mbx10 % 10, (unsigned long)(b.zl - a.zl),
           (unsigned long)(b.wm - a.wm), (unsigned long)(b.rm - a.rm), (unsigned long)(b.wb - a.wb));
    /* verify */
    unsigned bad = 0;
    for (uint32_t o = 0; o < SPAN; o += 8) if (*(volatile uint64_t *)(BUF_B + o)) bad++;
    if (bad) { printf("  %s: %u nonzero dwords\n", name, bad); fails++; }
}

int main(void) {
    uint64_t f = FEAT, ci = REG_CACHE_INFO;
    L = (uint32_t)((ci >> 24) & 0xFF);
    printf("DCBZ CPU_FEATURES=%08lx CACHE_INFO=%016llx line=%lu\n", (unsigned long)f,
           (unsigned long long)ci, (unsigned long)L);
    if (!(f & (1u << 22)) || L < 8 || (L & (L - 1))) {
        printf("DCBZ not present (CPU_FEATURES[22]=0) or bad line size\nDCBZ done fails=1\n");
        return 1;
    }
    correctness(0);
    correctness(1);
    for (int dirty = 0; dirty < 2; dirty++) {
        bench("dcbz", 1, dirty);
        bench("4x store", 0, dirty);
    }
    printf("DCBZ done fails=%u\n", fails);
    return 0;
}

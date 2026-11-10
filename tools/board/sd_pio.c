// sd_pio.c — SD_NATIVE_PLAN.md phase 0: the CPU cost of PIO through a 64-bit
// MMIO data port.  Uses today's SD_BUF window (0xF000_0200, plain LUTRAM on the
// same MMIO path a native-SD FIFO port will use) and reports PERF_CYCLES per
// 512-byte block for:
//   rd_port  — 64 x 64-bit loads from ONE MMIO address (the SDM_DATA FIFO shape),
//              each stored to a DDR buffer (cached);
//   rd_win   — 64 x 64-bit loads from the 512-byte window, stored to DDR;
//   wr_port  — 64 x 64-bit loads from DDR, stored to ONE MMIO address;
//   memcpy   — DDR to DDR 512 bytes (reference).
// MB/s at 100 MHz = 512 * 100 / cycles.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mmio.h"

#define SDBUF  0xF0000200u
#define DBUF   0x00300000u
#define REPS   2000u

static uint64_t now(void) { return REG_PERF_CYCLES; }

static uint64_t rd_port(void) {
    volatile uint64_t *p = (volatile uint64_t *)(uintptr_t)SDBUF;
    uint64_t t0 = now();
    for (uint32_t r = 0; r < REPS; r++) {
        uint64_t *d = (uint64_t *)(uintptr_t)DBUF;
        for (uint32_t i = 0; i < 64; i += 8) {
            d[i]     = *p; d[i + 1] = *p; d[i + 2] = *p; d[i + 3] = *p;
            d[i + 4] = *p; d[i + 5] = *p; d[i + 6] = *p; d[i + 7] = *p;
        }
        __asm__ volatile("" ::: "memory");
    }
    return (now() - t0) / REPS;
}

static uint64_t rd_win(void) {
    volatile uint64_t *p = (volatile uint64_t *)(uintptr_t)SDBUF;
    uint64_t t0 = now();
    for (uint32_t r = 0; r < REPS; r++) {
        uint64_t *d = (uint64_t *)(uintptr_t)DBUF;
        for (uint32_t i = 0; i < 64; i += 8) {
            d[i]     = p[i];     d[i + 1] = p[i + 1]; d[i + 2] = p[i + 2]; d[i + 3] = p[i + 3];
            d[i + 4] = p[i + 4]; d[i + 5] = p[i + 5]; d[i + 6] = p[i + 6]; d[i + 7] = p[i + 7];
        }
        __asm__ volatile("" ::: "memory");
    }
    return (now() - t0) / REPS;
}

static uint64_t wr_port(void) {
    volatile uint64_t *p = (volatile uint64_t *)(uintptr_t)SDBUF;
    uint64_t t0 = now();
    for (uint32_t r = 0; r < REPS; r++) {
        const uint64_t *s = (const uint64_t *)(uintptr_t)DBUF;
        for (uint32_t i = 0; i < 64; i += 8) {
            *p = s[i];     *p = s[i + 1]; *p = s[i + 2]; *p = s[i + 3];
            *p = s[i + 4]; *p = s[i + 5]; *p = s[i + 6]; *p = s[i + 7];
        }
        __asm__ volatile("" ::: "memory");
    }
    return (now() - t0) / REPS;
}

static uint64_t ref_memcpy(void) {
    uint64_t t0 = now();
    for (uint32_t r = 0; r < REPS; r++) {
        memcpy((void *)(uintptr_t)(DBUF + 0x1000), (const void *)(uintptr_t)DBUF, 512);
        __asm__ volatile("" ::: "memory");
    }
    return (now() - t0) / REPS;
}

static void show(const char *n, uint64_t c) {
    unsigned long mbx10 = c ? (unsigned long)(512u * 1000u / c) : 0;   // MB/s x10
    printf("SDPIO %-8s %5lu cyc/block  %3lu.%lu MB/s  (25 MB/s costs %2lu%% CPU)\n", n,
           (unsigned long)c, mbx10 / 10, mbx10 % 10, mbx10 ? 25000ul / mbx10 : 0);
}

int main(void) {
    for (uint32_t i = 0; i < 64; i++) ((volatile uint64_t *)(uintptr_t)SDBUF)[i] = i * 0x0101010101010101ull;
    for (uint32_t i = 0; i < 128; i++) ((uint64_t *)(uintptr_t)DBUF)[i] = i;
    rd_port(); rd_win(); wr_port();                      // warm the caches
    show("rd_port", rd_port());
    show("rd_win", rd_win());
    show("wr_port", wr_port());
    show("memcpy", ref_memcpy());
    printf("SDPIO done\n");
    return 0;
}

// perfdump.c — print the perf block's counters and the cache controller's.
#include <stdint.h>
#include <stdio.h>
#include "mmio.h"
#define P(o) (*(volatile uint64_t *)(uintptr_t)(0xF00D0000u + (o)))
#define C(o) (*(volatile uint64_t *)(uintptr_t)(0xF0050000u + (o)))
int main(void) {
    static const struct { const char *n; unsigned o; } p[] = {
        {"cycles", 0x08}, {"instr", 0x10}, {"stall_data", 0xB0}, {"stall_loaduse", 0xB8},
        {"stall_flags", 0xC0}, {"stall_sp", 0xC8}, {"stall_exbusy", 0xD0}, {"branch_flush", 0xD8},
        {"if_miss", 0xE0}, {"mem_wait", 0xE8}, {"ic_miss", 0xF0}, {"ic_fill", 0xF8}};
    static const struct { const char *n; unsigned o; } c[] = {
        {"cache_rd_hits", 0x40}, {"cache_rd_miss", 0x48}, {"cache_wr_hits", 0x50},
        {"cache_wr_miss", 0x58}, {"cache_wbacks", 0x60}, {"cache_stall", 0x68}};
    for (unsigned i = 0; i < sizeof p / sizeof p[0]; i++)
        printf("PD %-14s %12lu\n", p[i].n, (unsigned long)P(p[i].o));
    for (unsigned i = 0; i < sizeof c / sizeof c[0]; i++)
        printf("PD %-14s %12lu\n", c[i].n, (unsigned long)C(c[i].o));
    return 0;
}

// perfclr.c — clear the perf and cache counters and set PERF_CTRL[1]
// (exclusive stall attribution); pair with perfdump.c around another program
// (the counters survive program loads).
#include <stdint.h>
#include <stdio.h>
#include "mmio.h"
int main(void) {
    REG_CACHE_CTRL = 1;
    REG_PERF_CTRL  = 3;
    printf("perfclr: counters cleared, exclusive mode\n");
    return 0;
}

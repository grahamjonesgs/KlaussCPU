// vga_bw.c — CPU DRAM cost of VGA scan-out (handoff item 3 / OPEN_ITEMS 27).
// Measures, with scan-out off and then on (640x480 RGB565, the Zephyr GUI
// mode), in PERF_CYCLES:
//   chase  — dependent loads over a 1 MiB list (every load a cache miss)
//   fill   — a full-screen run of 32-bit stores (lv_draw_sw_blend_fill shape)
//   stream — a 512 KiB sequential read
// and the VGA underflow count.  Runs on the board and in tools/vsim's SoC sim.
#include <stdint.h>
#include <stdio.h>
#include "mmio.h"
#include "vga.h"

#define FB     0x00200000u            // 614400 B framebuffer
#define CHASE  0x00300000u            // 1 MiB list
#define NODE   1056u                  // stride: misses every load
#define STEPS  2000u

static uint64_t now(void) { return REG_PERF_CYCLES; }

static void build_chase(void) {
    uint32_t n = (1u << 20) / NODE;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t nxt = (i * 389u + 1u) % n;              // a long cycle
        *(volatile uint64_t *)(uintptr_t)(CHASE + i * NODE) = CHASE + nxt * NODE;
    }
}

static uint64_t chase(void) {
    uint64_t p = CHASE, t0 = now();
    for (uint32_t i = 0; i < STEPS; i++) p = *(volatile uint64_t *)(uintptr_t)p;
    uint64_t t = now() - t0;
    if (p == 1) printf("?");
    return t;
}

static uint64_t fill(uint32_t v) {
    volatile uint32_t *q = (volatile uint32_t *)(uintptr_t)FB;
    uint64_t t0 = now();
    for (uint32_t i = 0; i < 640u * 480u / 2u; i += 4) { q[i] = v; q[i + 1] = v; q[i + 2] = v; q[i + 3] = v; }
    return now() - t0;
}

static uint64_t stream(void) {
    volatile uint64_t *q = (volatile uint64_t *)(uintptr_t)CHASE;
    uint64_t s = 0, t0 = now();
    for (uint32_t i = 0; i < (512u << 10) / 8u; i += 4) s += q[i] + q[i + 1] + q[i + 2] + q[i + 3];
    uint64_t t = now() - t0;
    if (s == 1) printf("?");
    return t;
}

static void run(const char *tag) {
    REG_VGA_STATUS = 4;                                  // clear UNDERFLOW
    uint64_t c = chase(), f = fill(0x12345678u), s = stream();
    unsigned uf = (unsigned)(REG_VGA_STATUS >> 48);
    printf("VGABW %-4s chase %6lu cyc/load  fill %6lu cyc/kstore  stream %6lu cyc/KiB  underflow %u\n", tag,
           (unsigned long)(c / STEPS), (unsigned long)(f * 1000u / (640u * 480u / 2u)),
           (unsigned long)(s / 512u), uf);
}

int main(void) {
    printf("=== vga_bw (VGA_CTRL at start = %lu) ===\n", (unsigned long)REG_VGA_CTRL);
    build_chase();
    fill(0);
    vga_off();
    run("off");
    REG_VGA_FB_BASE = FB;
    REG_VGA_STRIDE  = 1280;
    REG_VGA_HEIGHT  = 480;
    REG_VGA_VSTART  = 0;
    REG_VGA_CTRL    = VGA_CTRL_SCANOUT_EN;
    uint64_t t0 = now();
    while (now() - t0 < 900000u) {}                      // > 2 frames: mode latched, fetching
    run("on");
    printf("=== done ===\n");
    return 0;
}

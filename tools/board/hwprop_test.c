// hwprop_test.c — on-silicon checks for the HARDWARE_PROPOSALS RTL
// (branch hw-proposals).  The new instructions are emitted as raw words
// from hwprop_snippets.h (tools/board/gen_hwprop.py; the toolchain does not
// know them yet).  Prints PASS/FAIL per group, cycle counts per kernel, and
// "=== ALL PASS ===" or a fail count.  Build: tools/board/build.sh hwprop_test
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mmio.h"
#include "hwprop_snippets.h"

static int fails;
static void check(const char *what, int ok) {
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) fails++;
}

// ---- timer ISR (as test_atomic.c) ----
volatile uint64_t ticks;
void hw_timer_isr_c(void) { ticks++; }
__asm__(
    "  .text\n  .p2align 3\n  .globl hw_timer_isr\n"
    "hw_timer_isr:\n"
    "  push r0\n  push r1\n  push r2\n  push r3\n  push r8\n  push r9\n"
    "  push r10\n  push r11\n  push r12\n  push r13\n  push r14\n"
    "  call hw_timer_isr_c\n"
    "  pop r14\n  pop r13\n  pop r12\n  pop r11\n  pop r10\n  pop r9\n"
    "  pop r8\n  pop r3\n  pop r2\n  pop r1\n  pop r0\n"
    "  iret\n");
extern void hw_timer_isr(void);

static uint64_t di(void) {
    register uint64_t r3 __asm__("r3");
    __asm__ volatile(DI_R3 : "=r"(r3) : : "memory");
    return r3;
}
static void ei(uint64_t v) {
    register uint64_t r1 __asm__("r1") = v;
    __asm__ volatile(EI_R1 : "+r"(r1) : : "memory");
}

static void test_branches(void) {
    int bad = 0;
    for (int i = 0; i < N_BR; i++) {
        uint64_t o = run_br(i, BR_CASES[i].a, BR_CASES[i].b);
        if ((int)o != BR_CASES[i].expect) {
            if (bad < 10) printf("  case %d %s a=%llx b=%llx got %d want %d\n", i, BR_CASES[i].name,
                                 (unsigned long long)BR_CASES[i].a, (unsigned long long)BR_CASES[i].b,
                                 (int)o, BR_CASES[i].expect);
            bad++;
        }
    }
    printf("  %d branch / compare cases, %d wrong\n", N_BR, bad);
    check("FBR W / imm32, short CMPRVW", bad == 0);
    bad = 0;
    for (int i = 0; i < N_VAL; i++) {
        uint64_t o = run_val(i, VAL_CASES[i].a);
        if (o != VAL_CASES[i].expect) {
            printf("  %s a=%llx got %llx want %llx\n", VAL_CASES[i].name, (unsigned long long)VAL_CASES[i].a,
                   (unsigned long long)o, (unsigned long long)VAL_CASES[i].expect);
            bad++;
        }
    }
    check("short MULV / LEAPC", bad == 0);
}

static void test_diei(void) {
    uint64_t a = di(), b = di();
    ei(a);
    uint64_t c = di();
    ei(c);
    check("DI returns old enable (1, 0, restore 1)", a == 1 && b == 0 && c == 1);

    // a real timer: no ISR may run inside a DI window; some run outside
    ticks = 0;
    REG_INT_MASK = 0;
    REG_INT_VEC(INT_SRC_TIMER) = (uint32_t)(uintptr_t)hw_timer_isr;
    REG_TIMER_PER = 1500;
    REG_INT_MASK = 1u << INT_SRC_TIMER;
    int inside = 0;
    for (int i = 0; i < 4000; i++) {
        uint64_t e = di();
        uint64_t t0 = ticks;
        for (volatile int k = 0; k < 20; k++) {}
        uint64_t t1 = ticks;
        ei(e);
        if (t1 != t0) inside++;
        for (volatile int k = 0; k < 7; k++) {}
    }
    uint64_t total = ticks;
    // WAIT while disabled wakes without entering the handler
    uint64_t e = di();
    uint64_t w0 = ticks;
    __asm__ volatile("wait" ::: "memory");
    uint64_t w1 = ticks;
    ei(e);
    for (volatile int k = 0; k < 2000; k++) {}
    uint64_t w2 = ticks;
    REG_INT_MASK = 0;
    REG_INT_VEC(INT_SRC_TIMER) = 0;
    printf("  %llu ISR runs, %d inside a DI window\n", (unsigned long long)total, inside);
    check("no ISR inside DI ... EI under a 15 us timer", inside == 0 && total > 50);
    check("WAIT while disabled wakes, handler runs after EI", w1 == w0 && w2 > w1);
}

static uint32_t codebuf[64] __attribute__((aligned(64)));

static void test_smc(void) {
    int bad = 0;
    for (int i = 0; i < N_SMC; i++) {
        codebuf[0] = W_ADDI1; codebuf[1] = W_RET; codebuf[2] = 0; codebuf[3] = 0;
        __asm__ volatile("" ::: "memory");
        long (*f)(long) = (long (*)(long))(uintptr_t)codebuf;
        f(0); f(0);                        // warm: the code is now in the I-cache
        uint64_t patch;
        if (SMC_SIZE[i] == 64) patch = ((uint64_t)W_RET << 32) | W_ADDI16;
        else if (SMC_SIZE[i] == 32) patch = W_ADDI16;
        else patch = (W_ADDI16 >> 16) & 0xFF;    // byte 2 of the word
        uint64_t r = run_smc(i, codebuf, patch);
        long want = SMC_SIZE[i] == 8 ? 0x11 : 16;
        if ((long)r != want) {
            printf("  smc %d-bit dist %d: got %ld want %ld\n", SMC_SIZE[i], SMC_DIST[i], (long)r, want);
            bad++;
        }
    }
    check("store into just-executed code (posted-store SMC)", bad == 0);
}

static uint64_t kbuf[8] __attribute__((aligned(64)));

static void kernels(void) {
    printf("  kernel: cycles per body (best of 9, PERF_CYCLES, minus the empty kernel)\n");
    uint64_t base = 0;
    for (int i = 0; i < N_KERN; i++) {
#ifdef KERN_ONLY
        if (strstr(KERN_NAME[i], "di ")) continue;
#endif
        uint64_t best = ~0ull;
        for (int r = 0; r < 9; r++) {
            uint64_t c = run_kern(i, kbuf);
            if (c < best) best = c;
        }
        if (i == 0) { base = best; printf("  KERN %-28s %lu\n", KERN_NAME[i], (unsigned long)best); continue; }
        unsigned long x100 = (unsigned long)((best - base) * 100 / KERN_N[i]);
        printf("  KERN %-28s %lu.%02lu\n", KERN_NAME[i], x100 / 100, x100 % 100);
    }
}

int main(void) {
#ifdef KERN_ONLY
    // A/B variant that runs on any bitstream: no new encodings executed
    printf("=== hwprop_kern ===\n");
#else
    printf("=== hwprop_test ===\n");
    test_branches();
    test_diei();
    test_smc();
#endif
    kernels();
    printf(fails ? "=== %d FAIL ===\n" : "=== ALL PASS ===\n", fails);
    return 0;
}

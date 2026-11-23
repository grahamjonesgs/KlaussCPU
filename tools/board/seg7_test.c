// seg7_test.c — on-board check of the 7-segment SEG_RAW / SEG_MODE registers
// (MMIO_MAP.md, CPU_FEATURES[30]) and the lowercase b / d glyphs.
// Part 1 checks the registers by reading them back and prints PASS / FAIL.
// Part 2 is visual: each step prints what the display should show and holds
// it for a moment; someone looking at the board confirms it.  The program
// ends with "PASS" or "FAIL" in raw mode (HALT keeps a raw display).
// Build: tools/board/build.sh seg7_test
#include <stdint.h>
#include <stdio.h>

#define SEG_BASE   0xF0030000u
#define SEG_LOW    (*(volatile uint64_t *)(SEG_BASE + 0x00))
#define SEG_HIGH   (*(volatile uint64_t *)(SEG_BASE + 0x08))
#define SEG_ALL    (*(volatile uint64_t *)(SEG_BASE + 0x10))
#define SEG_BLANK  (*(volatile uint64_t *)(SEG_BASE + 0x18))
#define SEG_RAW    (*(volatile uint64_t *)(SEG_BASE + 0x20))
#define SEG_MODE   (*(volatile uint64_t *)(SEG_BASE + 0x28))
#define SEG_RAW8   ((volatile uint8_t *)(SEG_BASE + 0x20))
#define SEG_RAW16  ((volatile uint16_t *)(SEG_BASE + 0x20))
#define SEG_RAW32  ((volatile uint32_t *)(SEG_BASE + 0x20))
#define CPU_FEATURES (*(volatile uint64_t *)0xF00F0048u)
#define CLOCK_MS   (*(volatile uint64_t *)0xF00F0040u)

// segments: bit 0..6 = a..g, bit 7 = DP
static const uint8_t DIG[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
enum { S_H = 0x76, S_E = 0x79, S_L = 0x38, S_O = 0x3F, S_P = 0x73, S_A = 0x77, S_S = 0x6D,
       S_F = 0x71, S_I = 0x30, S_DP = 0x80 };

static int fails;
static void check(const char *what, int ok) {
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) fails++;
}
static void hold(uint64_t ms) {
#ifdef SIM
    (void)ms; return;          // tb_soc: SRC=seg7_test DEFS=-DSIM build.sh seg7_sim
#endif
    uint64_t t0 = CLOCK_MS;
    while (CLOCK_MS - t0 < ms) { }
}
// the 8 bytes, leftmost digit first (s[0] = byte 7)
static uint64_t raw8(const uint8_t s[8]) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v = v << 8 | s[i];
    return v;
}
static void show(const char *what, uint64_t ms) {
    printf("  display: %s\n", what);
    hold(ms);
}

int main(void) {
    printf("seg7_test: SEG_RAW / SEG_MODE (CPU_FEATURES[30])\n");
    uint64_t feat = CPU_FEATURES;
    printf("  CPU_FEATURES = 0x%llx\n", (unsigned long long)feat);
    check("CPU_FEATURES[30]", (feat >> 30 & 1) == 1);
    if (!(feat >> 30 & 1)) { printf("=== %d FAIL ===\n", fails); return 1; }

    // ---- part 1: registers ----
    check("SEG_MODE = 0 after the load", SEG_MODE == 0);
    const uint64_t P = 0x807F3F065B4F666Dull;
    SEG_RAW = P;
    check("SEG_RAW 64-bit read-back", SEG_RAW == P);
    check("SEG_ALL reads the same bytes", SEG_ALL == P);
    check("SEG_MODE = 1 after a SEG_RAW store", SEG_MODE == 1);
    SEG_RAW8[3] = 0xAB;
    SEG_RAW32[1] = 0x11223344u;
    SEG_RAW16[0] = 0x5566;
    uint64_t want = 0x11223344AB4F5566ull;
    check("byte / 16 / 32-bit stores update only their digits", SEG_RAW == want);
    check("narrow loads", SEG_RAW8[3] == 0xAB && SEG_RAW16[1] == 0xAB4F && SEG_RAW32[1] == 0x11223344u &&
                          SEG_RAW8[7] == 0x11);
    SEG_LOW = 0x1234;
    check("SEG_LOW in raw mode blanks the upper half", SEG_ALL == 0x2222222201020304ull);
    check("SEG_LOW leaves raw mode", SEG_MODE == 0);
    SEG_RAW = P;
    SEG_HIGH = 0xABCD;
    check("SEG_HIGH in raw mode blanks the lower half", SEG_ALL == 0x0A0B0C0D22222222ull);
    check("SEG_HIGH leaves raw mode", SEG_MODE == 0);
    SEG_LOW = 0x5678;
    check("SEG_LOW outside raw mode keeps the upper half", SEG_ALL == 0x0A0B0C0D05060708ull);
    SEG_RAW = P;
    SEG_ALL = 0xDEADBEEF;
    check("SEG_ALL leaves raw mode", SEG_MODE == 0 && SEG_ALL == 0x0D0E0A0D0B0E0E0Full);
    SEG_RAW8[7] = 0x01;
    check("a byte store enters raw mode", SEG_MODE == 1 && SEG_RAW == 0x010E0A0D0B0E0E0Full);
    SEG_BLANK = 0;
    check("SEG_BLANK leaves raw mode", SEG_MODE == 0 && SEG_ALL == 0x2222222222222222ull);
    printf("part 1: %d failing\n", fails);

    // ---- part 2: visual ----
    printf("part 2 (visual): check each step on the board\n");
    { uint8_t s[8]; for (int i = 0; i < 8; i++) s[i] = DIG[7 - i]; SEG_RAW = raw8(s); }
    show("7 6 5 4 3 2 1 0  (digit order: byte 7 leftmost, byte 0 rightmost)", 3000);
    static const char *segname[8] = {"a (top)", "b (top right)", "c (bottom right)", "d (bottom)",
                                     "e (bottom left)", "f (top left)", "g (middle)", "DP (decimal point)"};
    for (int b = 0; b < 8; b++) {
        char msg[64];
        snprintf(msg, sizeof msg, "segment %s only, on all 8 digits", segname[b]);
        SEG_RAW = 0x0101010101010101ull << b;
        show(msg, 1200);
    }
    { uint8_t s[8] = {S_H, S_E, S_L, S_L, S_O | S_DP, 0, 0, 0}; SEG_RAW = raw8(s); }
    show("H E L L O.  then 3 blank digits", 2500);
    { uint8_t s[8] = {0, 0, 0, DIG[1], DIG[2] | S_DP, DIG[3], DIG[4], DIG[5]}; SEG_RAW = raw8(s); }
    show("3 blank digits then 1 2.3 4 5  (right-aligned decimal)", 2500);
    SEG_RAW = 0xFFFFFFFFFFFFFFFFull;
    show("8. 8. 8. 8. 8. 8. 8. 8.  (every segment lit)", 2000);
    SEG_RAW8[0] = 0;
    SEG_RAW8[7] = 0;
    show("blank, 8. x6, blank  (byte stores to the two end digits)", 2000);
    SEG_ALL = 0xBDBD0B8D;
    show("b d b d 0 b 8 d  (glyph mode: lowercase b and d, no decimal points)", 3000);
    SEG_BLANK = 0;
    show("blank", 1000);

    printf("=== %s (%d failing) ===\n", fails ? "FAIL" : "ALL PASS", fails);
    if (fails) { uint8_t s[8] = {S_F, S_A, S_I, S_L, 0, 0, 0, DIG[fails % 10]}; SEG_RAW = raw8(s); }
    else       { uint8_t s[8] = {S_P, S_A, S_S, S_S, 0, 0, 0, 0}; SEG_RAW = raw8(s); }
    printf("  display (left after exit): %s\n", fails ? "FAIL and the fail count" : "PASS");
    return fails != 0;
}

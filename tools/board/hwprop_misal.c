// hwprop_misal.c — a misaligned 32-bit load three calls deep must stop the
// CPU with a crash dump: ERR=0A, and F1/F3/F5 = the return addresses of
// level3 -> level2 -> level1 (HW proposals §3.3 / §3.4).  The load is a raw
// word: LDIDX32 r3, [r1 + 2] (the compiler never emits a misaligned one).
#include <stdint.h>
#include <stdio.h>
static uint64_t buf[4] = {0x1122334455667788ull, 0x99aabbccddeeff00ull};
__attribute__((noinline)) static uint64_t level3(uint64_t *p) {
    register uint64_t r1 __asm__("r1") = (uintptr_t)p;
    register uint64_t r3 __asm__("r3");
    printf("level3: misaligned load next\n");
    __asm__ volatile(".4byte 0x9A200310\n.4byte 0x00000002\n" : "=r"(r3) : "r"(r1) : "memory");
    return r3;
}
__attribute__((noinline)) static uint64_t level2(uint64_t *p) { return level3(p) + 1; }
__attribute__((noinline)) static uint64_t level1(uint64_t *p) { return level2(p) + 2; }
int main(void) {
    printf("level1=%p level2=%p level3=%p\n", (void *)level1, (void *)level2, (void *)level3);
    printf("unexpected: %llx\n", (unsigned long long)level1(buf));
    return 0;
}

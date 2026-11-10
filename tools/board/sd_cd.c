// sd_cd.c — print the raw SD card-detect pin (SD_STATUS[1]) and the native
// controller's CARD_PRESENT (SDM_STATUS[3]) and the QSPI ID/status, to
// calibrate the CD polarity (insert/remove the card between runs).
#include <stdint.h>
#include <stdio.h>
#define R64(a) (*(volatile uint64_t *)(uintptr_t)(a))
int main(void) {
    printf("SDCD SD_STATUS %016lx SDM_STATUS %016lx CPU_FEATURES %016lx\n",
           (unsigned long)R64(0xF0000010u), (unsigned long)R64(0xF0000128u), (unsigned long)R64(0xF00F0048u));
    for (unsigned o = 0; o < 0x40; o += 8)
        printf("SDCD QSPI+%02x %016lx\n", o, (unsigned long)R64(0xF0120000u + o));
    // Read-only: RDID (0x9F, 3 bytes) and READ (0x03) of the keystore's first 16 bytes.
    R64(0xF0120008u) = 0; R64(0xF0120000u) = 0x9F | (3ull << 16);
    while (R64(0xF0120010u) & 1) ;
    printf("SDCD RDID status %lx buf %016lx\n", (unsigned long)R64(0xF0120010u), (unsigned long)R64(0xF0120100u));
    R64(0xF0120008u) = 0xF00000u; R64(0xF0120000u) = 0x03 | (16ull << 16);
    while (R64(0xF0120010u) & 1) ;
    printf("SDCD READ@F00000 status %lx %016lx %016lx\n", (unsigned long)R64(0xF0120010u),
           (unsigned long)R64(0xF0120100u), (unsigned long)R64(0xF0120108u));
    return 0;
}

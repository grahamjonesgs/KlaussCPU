/* eth_test.c — Phase 5 bring-up tests for KlaussCPU + LiteEth.
 *
 * Runs four tests in order, each gating the next.  Status is shown on the
 * 7-segment display so a host UART is not strictly required.  Output codes:
 *
 *   T1 (scratchpad)  : "0050xxxx"  → "Pxxxxxxx" pass / "Exxxxxxx" fail
 *   T2 (MDIO PHY ID) : "0051xxxx"  → "0051" + ID1 (expect 0x0007 for LAN8720A)
 *   T3 (loopback)    : "0052xxxx"  → "P052xxxx" pass / "E052xxxx" fail
 *   T4 (broadcast)   : "0053xxxx"  → frame length on RX
 *
 * Wait at least ~1 second per stage to read the display.
 */

#include <stdint.h>
#include <string.h>
#include "mmio.h"

/* ---------------------------------------------------------------------------
 * Tiny helpers
 * ------------------------------------------------------------------------ */

/* Cycle-accurate at 100 MHz: ~5 cycles per loop iteration on this CPU.
 * Calibrate empirically — these values are conservative. */
static inline void delay_loops(uint32_t n) {
    volatile uint32_t i;
    for (i = 0; i < n; i++) { __asm__ volatile (""); }
}
static inline void delay_us(uint32_t us) { delay_loops(us * 20u); }
static inline void delay_ms(uint32_t ms) { delay_loops(ms * 20000u); }

static inline void show(uint32_t code) { REG_SEG_ALL = code; }

/* ---------------------------------------------------------------------------
 * MDIO bit-bang (IEEE 802.3 Clause 22)
 *
 * REG_ETH_MDIO_W  layout:  [0] MDC clock   [1] MDIO output enable   [2] MDIO data out
 * REG_ETH_MDIO_R  layout:  [0] live MDIO line value
 *
 * We drive MDC manually and hold each bit for ~200 ns (well under the
 * 2.5 MHz max).  The PHY samples MDIO on MDC's rising edge, and drives
 * MDIO during the read-data window when we tristate (OE=0).
 *
 * LAN8720A PHY address on Nexys A7 = 0x01 (per Digilent ref manual, RXER
 * pin strapped low at reset).
 * ------------------------------------------------------------------------ */

#define MDIO_PHY_ADDR    0x01u
#define MDIO_W_MDC       (1u << 0)
#define MDIO_W_OE        (1u << 1)
#define MDIO_W_DOUT      (1u << 2)

#define MDIO_HALF_DELAY()  delay_loops(40)   /* ~ tBIT/2 at 100 MHz */

/* Drive one MDC cycle: data=output bit (when oe=1), capture inbit if !oe. */
static uint32_t mdio_clock(uint32_t outbit, uint32_t oe) {
    uint32_t base = (oe ? MDIO_W_OE : 0u) | (outbit ? MDIO_W_DOUT : 0u);
    REG_ETH_MDIO_W = base;                       /* MDC low, present data  */
    MDIO_HALF_DELAY();
    REG_ETH_MDIO_W = base | MDIO_W_MDC;          /* MDC high (PHY samples) */
    uint32_t in = REG_ETH_MDIO_R & 1u;
    MDIO_HALF_DELAY();
    return in;
}

static void mdio_shift_out(uint32_t value, int nbits) {
    for (int i = nbits - 1; i >= 0; i--) {
        mdio_clock((value >> i) & 1u, /*oe=*/1);
    }
}

/* Read a 16-bit register from `phy_addr` reg `reg_addr`. */
static uint16_t mdio_read(uint32_t phy_addr, uint32_t reg_addr) {
    /* Preamble: 32 bits of 1 */
    for (int i = 0; i < 32; i++) mdio_clock(1, 1);
    /* Start (01) + OP read (10) */
    mdio_shift_out(0b01u, 2);
    mdio_shift_out(0b10u, 2);
    /* PHY addr (5b) + Reg addr (5b) */
    mdio_shift_out(phy_addr & 0x1Fu, 5);
    mdio_shift_out(reg_addr & 0x1Fu, 5);
    /* Turnaround: 1 cycle master tristates, 1 cycle PHY drives 0 */
    mdio_clock(0, 0);
    mdio_clock(0, 0);                            /* PHY's '0' ack — ignored */
    /* Read 16 data bits, MSB first */
    uint16_t v = 0;
    for (int i = 15; i >= 0; i--) {
        v = (uint16_t)((v << 1) | mdio_clock(0, 0));
    }
    /* Idle (one cycle of MDIO floating) */
    mdio_clock(0, 0);
    return v;
}

/* Write `value` to `phy_addr` reg `reg_addr`. */
static void mdio_write(uint32_t phy_addr, uint32_t reg_addr, uint16_t value) {
    for (int i = 0; i < 32; i++) mdio_clock(1, 1);
    mdio_shift_out(0b01u, 2);
    mdio_shift_out(0b01u, 2);                    /* OP = write */
    mdio_shift_out(phy_addr & 0x1Fu, 5);
    mdio_shift_out(reg_addr & 0x1Fu, 5);
    mdio_shift_out(0b10u, 2);                    /* TA = 10 */
    mdio_shift_out(value, 16);
    mdio_clock(0, 0);                            /* idle */
}

/* ---------------------------------------------------------------------------
 * Test 5.0 — Scratchpad sanity check
 *
 * Proves the whole MMIO → bus_splitter → eth_mmio_bridge → LiteEth Wishbone
 * → SoCController CSR path round-trips.  Three patterns to catch stuck-bit /
 * width issues.
 * ------------------------------------------------------------------------ */

static int test_scratchpad(void) {
    static const uint32_t patterns[] = {
        0xCAFEBABEu, 0x12345678u, 0xAAAA5555u, 0x55555555u, 0xFFFFFFFFu, 0x00000000u
    };
    for (unsigned i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++) {
        REG_ETH_CTRL_SCRATCH = patterns[i];
        if (REG_ETH_CTRL_SCRATCH != patterns[i]) {
            show(0xE0000050u | i);              /* show which pattern failed */
            return -1;
        }
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Test 5.1 — MDIO reachability + PHY ID
 *
 * Reads PHY register 0x02 (PHY ID1).  LAN8720A returns 0x0007.  If MDIO
 * isn't reachable (PHY in reset, bus floating, or the bridge mangles
 * narrow CSR accesses), the read returns 0xFFFF instead.
 * ------------------------------------------------------------------------ */

static int test_mdio_phy_id(void) {
    /* Hold PHY in reset for ≥25 ms (LAN8720A datasheet), then release.
       Allow ~120 ms for auto-negotiation to settle before MDIO use. */
    REG_ETH_PHY_RESET = 1;
    delay_ms(30);
    REG_ETH_PHY_RESET = 0;
    delay_ms(120);

    uint16_t id1 = mdio_read(MDIO_PHY_ADDR, 0x02);
    /* Show the ID1 in low 4 hex digits regardless of pass/fail */
    show(0x00510000u | id1);
    if (id1 != 0x0007u) return -1;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Test 5.2 — PHY internal loopback
 *
 * Set PHY register 0x00 bit 14 (internal MII loopback), TX a hand-crafted
 * frame, expect to see the same frame at RX.  Validates the entire MAC
 * datapath end-to-end without putting any signal on the wire.
 * ------------------------------------------------------------------------ */

static int test_phy_loopback(void) {
    /* Enable internal loopback (BMCR bit 14) */
    uint16_t bmcr = mdio_read(MDIO_PHY_ADDR, 0x00);
    mdio_write(MDIO_PHY_ADDR, 0x00, bmcr | (1u << 14));
    delay_ms(10);

    /* Drain any stale RX events */
    REG_ETH_RX_EV_PENDING = 1;
    REG_ETH_TX_EV_PENDING = 1;

    /* Build a 64-byte test frame (smallest legal Ethernet frame).  The MAC
       handles preamble/SFD; we provide only DST/SRC/EtherType/payload. */
    static const uint8_t frame[64] = {
        /* dst MAC */ 0x02, 0x00, 0x00, 0x00, 0x00, 0x02,
        /* src MAC */ 0x02, 0x00, 0x00, 0x00, 0x00, 0x01,
        /* etype  */  0x88, 0xB5,                           /* 0x88B5 = local experimental */
        /* payload (50 bytes) — counting pattern */
        0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
        0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
        0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
        0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23,
        0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B,
        0x2C, 0x2D
    };

    /* Stage frame into TX slot 0 */
    while (!REG_ETH_TX_READY) { }
    volatile uint8_t *tx = ETH_TX_SLOT(0);
    for (uint32_t i = 0; i < sizeof(frame); i++) tx[i] = frame[i];
    REG_ETH_TX_SLOT   = 0;
    REG_ETH_TX_LENGTH = sizeof(frame);
    REG_ETH_TX_START  = 1;

    /* Wait for RX event with a timeout.  At 100 Mbps a 64-byte frame
       round-trips through the MAC in well under a millisecond. */
    for (uint32_t i = 0; i < 200000u; i++) {
        if (REG_ETH_RX_EV_PENDING & 1u) goto got_rx;
        delay_loops(50);
    }
    show(0xE0520001u);                          /* timeout */
    return -1;

got_rx: {
        uint32_t slot = REG_ETH_RX_SLOT;
        uint32_t len  = REG_ETH_RX_LENGTH;
        if (len != sizeof(frame)) {
            show(0xE0520002u | (len << 16));    /* length mismatch */
            return -1;
        }
        volatile uint8_t *rx = ETH_RX_SLOT(slot);
        for (uint32_t i = 0; i < sizeof(frame); i++) {
            if (rx[i] != frame[i]) {
                show(0xE0520003u | (i << 16));  /* show offset that mismatched */
                return -1;
            }
        }
        REG_ETH_RX_EV_PENDING = 1;              /* W1C release */
    }

    /* Disable loopback for subsequent tests */
    mdio_write(MDIO_PHY_ADDR, 0x00, bmcr);
    delay_ms(10);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Test 5.3 — External TX (one ARP request on the wire)
 *
 * Plug into a switch (or a host running tcpdump on a crossover cable).
 * Send a gratuitous ARP request from `192.168.1.50` looking for `.1`.
 * The host should see the broadcast in `tcpdump -i ethX -e arp -nn`.
 *
 * No automated pass/fail — visual inspection on the host side.
 * ------------------------------------------------------------------------ */

static void test_external_tx(void) {
    static const uint8_t our_mac[6] = {
        ETH_DEFAULT_MAC_0, ETH_DEFAULT_MAC_1, ETH_DEFAULT_MAC_2,
        ETH_DEFAULT_MAC_3, ETH_DEFAULT_MAC_4, ETH_DEFAULT_MAC_5
    };
    uint8_t f[64] = {0};                        /* 14 ETH + 28 ARP, padded to 60 + CRC=64 */

    /* Ethernet header — broadcast destination */
    for (int i = 0; i < 6; i++) f[i] = 0xFF;
    for (int i = 0; i < 6; i++) f[6 + i] = our_mac[i];
    f[12] = 0x08; f[13] = 0x06;                 /* etype = ARP */

    /* ARP payload (network byte order) */
    f[14] = 0x00; f[15] = 0x01;                 /* HTYPE = Ethernet */
    f[16] = 0x08; f[17] = 0x00;                 /* PTYPE = IPv4 */
    f[18] = 6;    f[19] = 4;                    /* HLEN, PLEN */
    f[20] = 0x00; f[21] = 0x01;                 /* OPER = request */
    for (int i = 0; i < 6; i++) f[22 + i] = our_mac[i];   /* SHA */
    f[28] = 192; f[29] = 168; f[30] = 1; f[31] = 50;       /* SPA = 192.168.1.50 */
    /* THA already zero */
    f[38] = 192; f[39] = 168; f[40] = 1; f[41] = 1;        /* TPA = 192.168.1.1 */
    /* f[42..63] padding to reach 64 bytes (MAC pads to 60 then adds 4-byte CRC,
       so we send 60 here. Actually let's send 60.) */

    while (!REG_ETH_TX_READY) { }
    volatile uint8_t *tx = ETH_TX_SLOT(1);      /* use slot 1 to keep slot 0 from loopback */
    for (uint32_t i = 0; i < 60u; i++) tx[i] = f[i];
    REG_ETH_TX_SLOT   = 1;
    REG_ETH_TX_LENGTH = 60;
    REG_ETH_TX_START  = 1;

    /* Wait for TX completion */
    while (!(REG_ETH_TX_EV_PENDING & 1u)) { }
    REG_ETH_TX_EV_PENDING = 1;
}

/* ---------------------------------------------------------------------------
 * Test 5.4 — Passive RX
 *
 * Sit in a polling loop and display the length of each incoming frame on
 * the 7-segment.  On any normal Ethernet network the host or switch will
 * be sending broadcasts (ARP, mDNS, LLDP, …) and you'll see the display
 * change every few seconds.
 * ------------------------------------------------------------------------ */

static void test_passive_rx(void) {
    uint32_t frame_count = 0;
    while (1) {
        if (REG_ETH_RX_EV_PENDING & 1u) {
            uint32_t slot = REG_ETH_RX_SLOT;
            uint32_t len  = REG_ETH_RX_LENGTH;
            volatile uint8_t *rx = ETH_RX_SLOT(slot);
            uint16_t etype = ((uint16_t)rx[12] << 8) | rx[13];
            (void)etype;                        /* could log/filter here */
            REG_ETH_RX_EV_PENDING = 1;
            frame_count++;
            /* Display: high half = running count, low half = last len */
            show((frame_count << 16) | (len & 0xFFFFu));
        }
    }
}

/* ---------------------------------------------------------------------------
 * Main
 * ------------------------------------------------------------------------ */

void main(void) {
    /* Banner */
    show(0xEEEE0000u);
    delay_ms(500);

    /* T1: scratchpad — must pass before anything else */
    show(0x00500000u);
    delay_ms(300);
    if (test_scratchpad() != 0) { while (1) { } }
    show(0x00500001u);                          /* T1 passed */
    delay_ms(1000);

    /* T2: MDIO PHY ID */
    show(0x00510000u);
    delay_ms(300);
    if (test_mdio_phy_id() != 0) { while (1) { } }
    /* test_mdio_phy_id() already shows the ID1; hold for inspection */
    delay_ms(2000);

    /* T3: PHY internal loopback */
    show(0x00520000u);
    delay_ms(300);
    if (test_phy_loopback() != 0) { while (1) { } }
    show(0x00520001u);                          /* T3 passed */
    delay_ms(1000);

    /* T4: external TX (ARP) — visual inspection on host */
    show(0x00530000u);
    delay_ms(300);
    test_external_tx();
    show(0x00530001u);
    delay_ms(1000);

    /* T5: passive RX (loops forever, displaying frame counts) */
    show(0x00540000u);
    delay_ms(300);
    test_passive_rx();
}

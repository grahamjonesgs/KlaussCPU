/*
 * netboot.c — KlaussCPU network program loader (Phase 1: receive + run).
 *
 * Brings up the same lwIP NO_SYS=1 stack as lwip_demo.c, gets an IP via DHCP
 * (falls back to a static address), and listens on TCP port NETBOOT_PORT.  A
 * host connects and streams:
 *
 *     [ 12-byte header, little-endian ]
 *         u32 magic     = NETBOOT_MAGIC  ("KNET")
 *         u32 img_len   = bytes of DDR image that follow
 *         u32 entry_pc  = byte address to jump to (board byte address, e.g. 0x20)
 *     [ img_len bytes ] = the full DDR image (heap header @0x00 + code @0x20),
 *                         exactly the layout a kbt load would write, minus framing.
 *
 * The image is written into a DDR *staging* region (STAGING_BASE) as it arrives.
 * When img_len bytes have been received the board computes a 32-bit checksum,
 * shows it on the 7-seg, lights LED bit 4, and sends an 8-byte reply:
 *
 *     u32 status   (0 = OK)
 *     u32 checksum (sum of 32-bit LE words of the image)
 *
 * PHASE 1: after the reply flushes, the board emits a small relocation
 * trampoline into high DDR (TRAMP_BASE — clear of both the staging source and
 * the 0x0 destination), then jumps to it.  The trampoline copies the staged
 * image down to 0x0, restores SP=0x0800_0000 (as the HW loader's LOAD_COMPLETE
 * does), and jumps to the program entry — so the program runs exactly as if it
 * had been kbt-loaded.  netboot itself (running from low DDR) is overwritten by
 * the copy, which is why the copy+jump must run from the trampoline, not here.
 * Set NETBOOT_LAUNCH 0 to revert to Phase-0 receive+verify only.
 *
 * The trampoline words come from trampoline.kla (assembled with klausscc at
 * base 0x20); see that file for the source.  Running resident from boot SRAM is
 * Phase 2 (see NETBOOT_PLAN.md).
 *
 * Build: drop this in runtime/programs/ alongside lwip_demo.c — it reuses the
 * same includes (mmio.h, src/eth.h, lwip_port/ethernetif.h) and lwIP config.
 *
 * 7-seg status (no LEDs):
 *   1E110000    startup marker ("INIT"), shown before eth/lwIP bring-up
 *   D8C90000    DHCP in progress ("dHCP")
 *   <IP>        IP assigned, host byte order (C0A8443B = 192.168.68.59)
 *   <bytes>     receiving — image bytes received so far (matches the UART loader)
 *   <checksum>  transfer complete (32-bit additive checksum of the image)
 */

#include <stdio.h>
#include <string.h>
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"   /* tcp_ack_now() — force immediate ACK */
#include "lwip/etharp.h"
#include "netif/ethernet.h"   /* ethernet_input() */
#include "../mmio.h"
#include "../src/eth.h"
#include "../lwip_port/ethernetif.h"

// ── Configuration ─────────────────────────────────────────────────────────

#define DHCP_TIMEOUT_MS  10000u
#define NETBOOT_PORT     5000u
#define NETBOOT_MAGIC    0x54454E4Bu          /* "KNET" little-endian */

// DDR staging region: well clear of a small UART-loaded bootloader at low DDR
// and below the stack at 0x0800_0000.  Holds up to (0x0800_0000-STAGING_BASE)
// bytes — 64 MiB here, far more than any program.
#define STAGING_BASE     0x04000000u

// Phase 1: emit + run the relocation trampoline (1) or stop at verify (0).
#define NETBOOT_LAUNCH   1

// Where the trampoline is emitted and run: just below the stack top, clear of
// both the staging source (STAGING_BASE..+img) and the 0x0 destination.
#define TRAMP_BASE       0x07FF0000u
// Largest image we accept (16 MiB — far above any real program). Keeps the four
// regions disjoint:
//   dst      0x0000_0000 .. +img      (<= 0x0100_0000)
//   staging  0x0400_0000 .. +img      (<= 0x0500_0000)
//   tramp    0x07FF_0000 .. +104 B
//   stack    0x0800_0000 (grows down)
#define MAX_IMG_BYTES    0x01000000u

// Static fallback if DHCP does not answer. Set to the Deco-reserved address
// (MAC 00:AB:CD:00:00:01 → 192.168.68.59) so the board is reachable at the same
// IP the host targets whether or not DHCP responds.
#define STATIC_IP   IPADDR4_INIT_BYTES(192, 168, 68, 59)
#define STATIC_NM   IPADDR4_INIT_BYTES(255, 255, 255, 0)
#define STATIC_GW   IPADDR4_INIT_BYTES(192, 168, 68, 1)

// ── Receive state ─────────────────────────────────────────────────────────

enum rx_phase { RX_HEADER, RX_IMAGE, RX_DONE };

static struct netif g_netif;
static volatile int g_ip_assigned = 0;

static enum rx_phase g_phase    = RX_HEADER;
static uint8_t       g_hdr[12];
static uint32_t      g_hdr_have = 0;
static uint32_t      g_img_len  = 0;
static uint32_t      g_entry_pc = 0;
static uint32_t      g_offset   = 0;        // bytes written to staging so far
static volatile int  g_image_ready = 0;     // set when a full image is staged
static uint64_t      g_start_ms = 0;        // ms timestamp at first image byte

static inline uint32_t rd_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// 32-bit additive checksum over the staged image (sum of LE words).
static uint32_t image_checksum(uint32_t len) {
    const volatile uint8_t *img = (const volatile uint8_t *)STAGING_BASE;
    uint32_t sum = 0;
    for (uint32_t i = 0; i + 4 <= len; i += 4) {
        sum += (uint32_t)img[i] | ((uint32_t)img[i + 1] << 8) |
               ((uint32_t)img[i + 2] << 16) | ((uint32_t)img[i + 3] << 24);
    }
    return sum;
}

// ── Relocation trampoline ─────────────────────────────────────────────────
//
// Machine code from trampoline.kla, assembled at base 0x20 (ASM_BASE).  Copies
// the staged image (src 0x04000000, dst 0x0) one 64-bit word at a time, restores
// SP, and jumps to the entry register.  Emitted to TRAMP_BASE at run time with
// the two absolute jump targets relocated and the len/entry immediates patched.
//
//   idx 5  = copy length in bytes (multiple of 8)   ← patched
//   idx 7  = entry PC                               ← patched
//   idx 13 = JMPE target (DONE, base 0x80)          ← + (TRAMP_BASE-ASM_BASE)
//   idx 23 = JMP  target (LOOP, base 0x48)          ← + (TRAMP_BASE-ASM_BASE)

#define ASM_BASE     0x20u
#define TRAMP_WORDS  26

static const uint32_t TRAMP_TEMPLATE[TRAMP_WORDS] = {
    0x00000800u, 0x04000000u,   //  0,1  SETR A, 0x04000000   (src)
    0x00000801u, 0x00000000u,   //  2,3  SETR B, 0x0          (dst)
    0x00000802u, 0x00000000u,   //  4,5  SETR C, len          (PATCH idx 5)
    0x00000803u, 0x00000000u,   //  6,7  SETR D, entry        (PATCH idx 7)
    0x00000805u, 0x08000000u,   //  8,9  SETR F, 0x08000000   (stack top)
    0x00000832u, 0x00000000u,   // 10,11 CMPRV C, 0           (LOOP)
    0x00001003u, 0x00000080u,   // 12,13 JMPE DONE            (PATCH idx 13)
    0x00007B40u,                // 14    MEMGET64 E, A
    0x00007A41u,                // 15    MEMSET64 E, B
    0x00000810u, 0x00000008u,   // 16,17 ADDV A, 8
    0x00000811u, 0x00000008u,   // 18,19 ADDV B, 8
    0x00000822u, 0x00000008u,   // 20,21 MINUSV C, 8
    0x00001000u, 0x00000048u,   // 22,23 JMP LOOP             (PATCH idx 23)
    0x00004045u,                // 24    SETSP F
    0x00001023u,                // 25    JMPR D
};

// Emit the patched trampoline to TRAMP_BASE and jump to it. Never returns.
static void launch_image(uint32_t entry_pc, uint32_t img_len) {
    uint32_t reloc = TRAMP_BASE - ASM_BASE;
    uint32_t tramp[TRAMP_WORDS];
    for (int i = 0; i < TRAMP_WORDS; i++) tramp[i] = TRAMP_TEMPLATE[i];
    tramp[5]   = (img_len + 7u) & ~7u;      // bytes to copy, rounded to 8
    tramp[7]   = entry_pc;
    tramp[13] += reloc;                     // JMPE DONE → run address
    tramp[23] += reloc;                     // JMP  LOOP → run address

    volatile uint32_t *dst = (volatile uint32_t *)TRAMP_BASE;
    for (int i = 0; i < TRAMP_WORDS; i++) dst[i] = tramp[i];

    // Hand off. The unified cache is coherent and a store into a buffered code
    // line invalidates it, so the just-written trampoline (and, after it copies,
    // the program) is fetched correctly — same property the LLEXT loader uses.
    ((void (*)(void))TRAMP_BASE)();
}

// ── Netif status callback (IP assigned) ───────────────────────────────────

static void netif_status_cb(struct netif *nif) {
    if (nif->ip_addr.addr != 0) {
        // ip_addr.addr is network byte order; ntohl gives host order so the
        // 7-seg reads as the dotted quad (e.g. C0A8443B = 192.168.68.59).
        REG_SEG_ALL = ntohl(nif->ip_addr.addr);
        g_ip_assigned = 1;
    } else {
        g_ip_assigned = 0;
    }
}

// ── TCP receive: stream image into DDR staging ────────────────────────────

static void send_ack(struct tcp_pcb *pcb, uint32_t status, uint32_t checksum) {
    uint8_t reply[8];
    reply[0] = status & 0xFF;       reply[1] = (status >> 8) & 0xFF;
    reply[2] = (status >> 16) & 0xFF; reply[3] = (status >> 24) & 0xFF;
    reply[4] = checksum & 0xFF;      reply[5] = (checksum >> 8) & 0xFF;
    reply[6] = (checksum >> 16) & 0xFF; reply[7] = (checksum >> 24) & 0xFF;
    tcp_write(pcb, reply, sizeof(reply), TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);
}

// Consume `n` bytes from `src`, advancing the receive state machine.
static void consume(struct tcp_pcb *pcb, const uint8_t *src, uint32_t n) {
    uint8_t *stage = (uint8_t *)STAGING_BASE;

    while (n > 0) {
        if (g_phase == RX_HEADER) {
            uint32_t need = 12u - g_hdr_have;
            uint32_t take = (n < need) ? n : need;
            memcpy(g_hdr + g_hdr_have, src, take);
            g_hdr_have += take;
            src += take;
            n   -= take;
            if (g_hdr_have == 12u) {
                if (rd_le32(g_hdr) != NETBOOT_MAGIC) {
                    send_ack(pcb, 1u, 0u);      // bad magic
                    g_phase = RX_DONE;
                    return;
                }
                g_img_len  = rd_le32(g_hdr + 4);
                g_entry_pc = rd_le32(g_hdr + 8);
                g_offset   = 0;
                if (g_img_len == 0 || g_img_len >= MAX_IMG_BYTES) {
                    send_ack(pcb, 2u, 0u);      // image too large / empty
                    g_phase = RX_DONE;
                    return;
                }
                g_start_ms = REG_CLOCK_MS;      // start the throughput clock
                g_phase = RX_IMAGE;
            }
        } else if (g_phase == RX_IMAGE) {
            uint32_t need = g_img_len - g_offset;
            uint32_t take = (n < need) ? n : need;
            memcpy(stage + g_offset, src, take);
            g_offset += take;
            src += take;
            n   -= take;
            REG_SEG_ALL = g_offset;            // bytes received (matches UART loader)
            if (g_offset == g_img_len) {
                uint32_t cks = image_checksum(g_img_len);
                REG_SEG_ALL = cks;
                send_ack(pcb, 0u, cks);
                uint64_t elapsed_ms = REG_CLOCK_MS - g_start_ms;
                if (elapsed_ms == 0) elapsed_ms = 1;   // avoid /0 on tiny images
                uint32_t kbps = (uint32_t)(((uint64_t)g_img_len * 1000u) /
                                           (elapsed_ms * 1024u));
                printf("netboot: received %lu bytes, entry 0x%08lx, cks 0x%08lx\n",
                       (unsigned long)g_img_len, (unsigned long)g_entry_pc,
                       (unsigned long)cks);
                printf("netboot: %lu ms, %lu KiB/s\n",
                       (unsigned long)elapsed_ms, (unsigned long)kbps);
                g_phase = RX_DONE;
                // Defer the jump to the main loop so the ACK/FIN flush first and
                // we're out of lwIP's receive callback stack.
                g_image_ready = 1;
            }
        } else { /* RX_DONE */
            return;                            // ignore trailing bytes
        }
    }
}

static err_t netboot_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    (void)arg;
    if (err != ERR_OK || p == NULL) {          // peer closed / error
        if (p) pbuf_free(p);
        tcp_close(pcb);
        return ERR_OK;
    }
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        consume(pcb, (const uint8_t *)q->payload, q->len);
    }
    tcp_recved(pcb, p->tot_len);
    /* Force an immediate ACK instead of waiting for lwIP's ~250 ms delayed-ACK
       timer.  A Nagle-enabled sender otherwise stalls one segment per tick
       (~6 KiB/s); instant ACKs keep its pipe full at line rate. */
    tcp_ack_now(pcb);
    tcp_output(pcb);
    pbuf_free(p);
    return ERR_OK;
}

static err_t netboot_accept(void *arg, struct tcp_pcb *new_pcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || new_pcb == NULL) return ERR_VAL;
    // Reset receive state for each new connection.
    g_phase = RX_HEADER; g_hdr_have = 0; g_offset = 0; g_img_len = 0;
    tcp_recv(new_pcb, netboot_recv);
    printf("netboot: connection accepted\n");
    return ERR_OK;
}

static void start_netboot_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) { printf("tcp_new failed\n"); return; }
    if (tcp_bind(pcb, IP_ADDR_ANY, NETBOOT_PORT) != ERR_OK) {
        printf("tcp_bind failed\n"); return;
    }
    struct tcp_pcb *listen_pcb = tcp_listen(pcb);
    if (!listen_pcb) { printf("tcp_listen failed\n"); return; }
    tcp_accept(listen_pcb, netboot_accept);
    printf("netboot: listening on TCP port %u\n", (unsigned)NETBOOT_PORT);
}

// ── main ──────────────────────────────────────────────────────────────────

int main(void) {
    printf("=== KlaussCPU netboot ===\n");
    REG_SEG_ALL = 0x1E110000u;   // startup marker ("INIT")

    eth_init();                  // PHY reset + MDIO verify + enable events
    lwip_init();

    ip4_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr,  0, 0, 0, 0);
    IP4_ADDR(&netmask, 0, 0, 0, 0);
    IP4_ADDR(&gw,      0, 0, 0, 0);

    netif_add(&g_netif, &ipaddr, &netmask, &gw,
              NULL, ethernetif_init, ethernet_input);
    netif_set_default(&g_netif);
    netif_set_status_callback(&g_netif, netif_status_cb);
    netif_set_up(&g_netif);

    printf("Starting DHCP...\n");
    REG_SEG_ALL = 0xD8C90000u;   // "dHCP" — DHCP in progress
    dhcp_start(&g_netif);

    uint64_t dhcp_deadline = REG_CLOCK_MS + DHCP_TIMEOUT_MS;
    while (!g_ip_assigned && REG_CLOCK_MS < dhcp_deadline) {
        ethernetif_input(&g_netif);
        sys_check_timeouts();
    }
    if (!g_ip_assigned) {
        printf("DHCP timeout — using static fallback\n");
        dhcp_stop(&g_netif);
        ip4_addr_t sip = STATIC_IP, snm = STATIC_NM, sgw = STATIC_GW;
        netif_set_addr(&g_netif, &sip, &snm, &sgw);
    }

    start_netboot_server();
    printf("\nnetboot ready — send an image with `klausscc --net-load`.\n\n");

    for (;;) {
        ethernetif_input(&g_netif);
        sys_check_timeouts();

#if NETBOOT_LAUNCH
        if (g_image_ready) {
            // Service lwIP ~100 ms so the TCP reply (and FIN) reach the host
            // before we overwrite ourselves, then hand off to the program.
            uint64_t flush_until = REG_CLOCK_MS + 100u;
            while (REG_CLOCK_MS < flush_until) {
                ethernetif_input(&g_netif);
                sys_check_timeouts();
            }
            printf("netboot: launching program at 0x%08lx\n",
                   (unsigned long)g_entry_pc);
            launch_image(g_entry_pc, g_img_len);   // never returns
        }
#endif
    }
    return 0;
}

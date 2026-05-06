# KlaussCPU MMIO Memory Map

Memory-mapped I/O region for on-chip peripherals. Accessed with the existing
`LD`/`ST` (MEMSETxx / MEMGETxx) opcodes — no new opcodes required. The
`bus_splitter` module (KlaussCPU.srcs/sources_1/new/bus_splitter.v) routes any
access whose top address nibble is `0xF` to the MMIO bus instead of the DDR2
cache controller.

## Address layout

```
0x0000_0000 – 0x07FF_FFFF   DDR2 (128 MB, cached)
0x0800_0000 – 0xEFFF_FFFF   unmapped
0xF000_0000 – 0xFFFF_FFFF   MMIO   ──┐
                                    ├── decode rule:
0xF000_xxxx                 SD card │   addr[31:28] == 0xF  → MMIO
0xF001_xxxx                 UART    │   addr[27:16]         → device id
0xF002_xxxx                 RGB LED │   addr[15:0]          → register offset
0xF003_xxxx                 7-segment
0xF004_xxxx                 LEDs / switches
0xF005_xxxx                 cache controller (counters + control)
0xF006_xxxx – 0xF00E_xxxx   reserved
0xF00F_xxxx                 timers / IRQ controller
```

Reserved ranges read as 0 and ignore writes (no bus error).

## Access semantics

- All MMIO registers are accessible as 8/16/32/64-bit loads and stores. The
  width of the load/store determines how many bytes of the register are
  read/written; bits outside the register-defined width read as 0.
- **MMIO is never cached.** The splitter sits in front of the cache, so reads
  always reflect current device state and writes take effect immediately.
- MMIO writes complete in a single cycle (no read-data path involved).
- MMIO reads take two cycles: the read strobe is captured in cycle 1, the
  registered data and `ready` arrive in cycle 2. The pipeline FF on the
  read return path (`r_mmio_read_data` in `KlaussCPU.v`) is
  what enables timing closure — without it the combinational path from
  peripheral RAMs through `bus_splitter` and into the UART helpers blows
  through the 100 MHz budget.
- Writes to undefined offsets within a mapped device are silently dropped.
- Reads from undefined offsets return 0.

## Currently implemented

### SD card — base `0xF000_0000`

Bare-metal SPI controller for the Nexys A7 microSD slot
([sd_spi.v](KlaussCPU.srcs/sources_1/new/sd_spi.v)). Hardware exposes the
smallest useful primitive — single-byte SPI transfer plus a 512-byte sector
buffer — and the full SD protocol (CMD0, ACMD41, CMD17, CRC, …) lives in
software. This keeps the hardware ~250 lines and lets the protocol evolve in C
without re-synthesising.

| Offset | Reg         | RW | Width | Bits | Description |
|--------|-------------|----|-------|------|-------------|
| 0x000  | `SD_CTRL`   | RW | 18    | `[15:0]` `clk_div`, `[16]` `cs_n`, `[17]` `pwr_en` | Clock divisor, chip-select, slot power |
| 0x008  | `SD_DATA`   | W  | 8     | `[7:0]` byte to send | Triggers a single-byte SPI transfer when written (ignored if busy) |
| 0x008  | `SD_DATA`   | R  | 8     | `[7:0]` last RX byte | Byte received on the most recent transfer |
| 0x010  | `SD_STATUS` | R  | 2     | `[0]` `busy`, `[1]` `card_present` | Transfer in progress / card detect line |
| 0x200..0x3F8 | `SD_BUF` | RW | 512 B | — | Sector buffer (64 × 64-bit doublewords). Plain `LD`/`ST` to addresses `0xF000_0200`..`0xF000_03F8`. |

**`clk_div`** sets the SCK half-period in `i_Clk` cycles. With `i_Clk` =
100 MHz, `SCK frequency = 100e6 / (2 × (clk_div + 1))`:

| clk_div | SCK rate    | Use case                      |
|---------|-------------|-------------------------------|
| 249     | ~200 kHz    | reset default — safe for init |
| 124     | ~400 kHz    | upper limit for SD init phase |
| 1       | ~25 MHz     | maximum speed per SD spec     |

**`cs_n`** drives the SD card's chip-select line directly. Software is
responsible for asserting it (write 0) before a command and releasing it
(write 1) after, with at least 8 dummy clocks before/after.

**`pwr_en`** drives the slot's active-low power gate (`SD_RESET` on the Nexys
A7 schematic). At reset `pwr_en = 0` → slot is OFF. Software **must** set
`pwr_en = 1` and wait ≥1 ms before any SPI activity.

**`busy`** is 1 while a single-byte transfer is in progress. After writing
`SD_DATA`, software polls `SD_STATUS` until `busy == 0`, then reads back
`SD_DATA` for the received byte.

**`card_present`** reflects the live `SD_CD` pin. Polarity is board-dependent
on the Nexys A7 (mechanical contact); calibrate empirically by inserting and
removing a card.

**Sector buffer** is plain memory. Software stages an outgoing sector here
before issuing CMD24, or reads the incoming sector here after CMD17. Use
`MEMSET64`/`MEMGET64` for fast 8-byte transfers; byte/halfword/word stores
also work via byte enables.

### RGB LED — base `0xF002_0000`

Each LED is a 12-bit value with three 4-bit PWM channels. Channel 0 (`0..15`)
is fully off, channel 15 is maximum brightness (~25 % duty cycle in hardware).

| Offset | Reg     | RW | Width | Bits         | Description                |
|--------|---------|----|-------|--------------|----------------------------|
| 0x0000 | `RGB1`  | RW | 12    | `[11:8]` R, `[7:4]` G, `[3:0]` B | RGB LED 1 |
| 0x0008 | `RGB2`  | RW | 12    | `[11:8]` R, `[7:4]` G, `[3:0]` B | RGB LED 2 |

Reading returns the last value written (or boot default `0x000`).

Legacy opcodes that touch the same registers (kept for now):
`RGB1R/RGB2R/RGB1V/RGB2V` — see `opcode_select.vh` `0x305?` / `0x306?` /
`0x3074` / `0x3075`.

### 7-Segment Display — base `0xF003_0000`

The Nexys A7 has eight 7-segment digits arranged as two 4-digit groups
("upper" and "lower"). Hardware encodes each digit as `{4'h0, hex_digit}` per
byte, so the on-screen state is two 32-bit padded values (`r_seven_seg_value1`
upper, `r_seven_seg_value2` lower). The MMIO interface accepts unpadded hex
digits and pads them automatically.

| Offset | Reg        | RW | Width | Description                                  |
|--------|------------|----|-------|----------------------------------------------|
| 0x0000 | `SEG_LOW`  | W  | 16    | 4 hex digits → lower display, packed `[15:0]` |
| 0x0008 | `SEG_HIGH` | W  | 16    | 4 hex digits → upper display, packed `[15:0]` |
| 0x0010 | `SEG_ALL`  | W  | 32    | 8 hex digits across both displays, `[31:0]` (high → upper, low → lower) |
| 0x0018 | `SEG_BLANK`| W  | —     | any write blanks both displays                |
| 0x0000 | `SEG_LOW`  | R  | 32    | raw padded value of lower display             |
| 0x0008 | `SEG_HIGH` | R  | 32    | raw padded value of upper display             |
| 0x0010 | `SEG_ALL`  | R  | 64    | concatenation `{upper_padded, lower_padded}`  |

Read width is wider than write width because reads return the raw padded
(`{4'h0, digit}` per byte) representation.

Legacy opcodes that touch the same registers:
`7SEGV1/7SEGV2/7SEGR1/7SEGR2/7SEGR/7SEGBLANK` — see `opcode_select.vh`
`0x4xxx` block.

### LEDs and switches — base `0xF004_0000`

| Offset | Reg        | RW | Width | Description                              |
|--------|------------|----|-------|------------------------------------------|
| 0x0000 | `LEDS`     | RW | 16    | 16-bit LED bar                           |
| 0x0008 | `SWITCHES` | R  | 16    | live state of the 16 board slide switches|

Writes to `SWITCHES` are dropped. Reading `LEDS` returns the last value written.

Legacy opcodes: `LEDR/LEDV` (set), `SWITCHR` (read switches) — see
`opcode_select.vh` `0x3xxx` block.

### Cache controller — base `0xF005_0000`

Performance counters and control for the L1 cache (2-way set-associative
write-back, 64 KB total, 16-byte lines — see
[mem_read_write.v](KlaussCPU.srcs/sources_1/new/mem_read_write.v)). The events
mirror the cache-related set of RISC-V Zihpm performance counters
(`mhpmcounter` / `mhpmevent`); RISC-V leaves the actual counter exposure
implementation-defined, so they live in MMIO here rather than as CSRs.

| Offset  | Reg                  | RW | Width | Description |
|---------|----------------------|----|-------|-------------|
| 0x0000  | `CACHE_CTRL`         | RW | 1     | `[0]` write-1-clear-counters (self-clearing). Bit 0 reads as 0. Other bits reserved (read 0). |
| 0x0008  | `CACHE_INFO`         | R  | 64    | Read-only geometry. `[7:0]` = ways, `[23:8]` = sets, `[31:24]` = line bytes, `[63:32]` = total bytes. For the current build returns `64'h0001_0000_1008_0002` (2 ways, 2048 sets, 16 B/line, 64 KB total). |
| 0x0040  | `CNT_READ_HITS`      | R  | 64    | Read accesses that hit a valid cache line. |
| 0x0048  | `CNT_READ_MISSES`    | R  | 64    | Read accesses that missed and triggered a DDR refill. |
| 0x0050  | `CNT_WRITE_HITS`     | R  | 64    | Write accesses that hit a valid cache line. |
| 0x0058  | `CNT_WRITE_MISSES`   | R  | 64    | Write accesses that missed and triggered a fetch-then-merge refill. |
| 0x0060  | `CNT_WRITEBACKS`     | R  | 64    | Dirty-line evictions (the cache wrote a dirty line back to DDR before refilling its slot). Counts both write-miss and read-miss eviction paths. |
| 0x0068  | `CNT_STALL_CYCLES`   | R  | 64    | `i_Clk` cycles the cache spent in the miss/refill chain (writeback, CDC gap, DDR fetch, install). Counts only beyond the single-cycle hit case, so this is the cycle cost of cache misses, not total memory-access latency. |

**Counters are 64-bit and free-running.** At `i_Clk = 100 MHz` even an event
that fires every cycle takes ~5.8 × 10⁹ years to wrap, so software never has
to manage rollover. Reset values: all zero. Counters are not affected by
`CPU_RESETN` or program load — only by writing `CACHE_CTRL` bit 0.

**Why not flush?** A cache flush + invalidate (writeback all dirty + drop all
tags) requires walking every set, which is a multi-thousand-cycle FSM
extension to `mem_read_write.v`. The cache is currently never observed
externally (no DMA, MMIO already bypasses the cache), so flush is not
required for correctness. It is reserved for future use under a separate
`CACHE_CTRL` bit.

**How to use:**

1. Clear counters at the start of the region of interest.
2. Run the workload.
3. Read all the counters and compute hit rate / miss rate / writeback rate /
   average miss penalty offline. The total access count is
   `READ_HITS + READ_MISSES + WRITE_HITS + WRITE_MISSES`. Average miss
   penalty in cycles is `STALL_CYCLES / (READ_MISSES + WRITE_MISSES)`.

```c
#include "mmio.h"

void cache_profile(void (*workload)(void)) {
    REG_CACHE_CTRL = 1;                 /* clear */
    workload();
    uint64_t rh = REG_CACHE_RD_HITS,    wh = REG_CACHE_WR_HITS;
    uint64_t rm = REG_CACHE_RD_MISSES,  wm = REG_CACHE_WR_MISSES;
    uint64_t wb = REG_CACHE_WRITEBACKS, st = REG_CACHE_STALL_CYC;
    uint64_t total   = rh + rm + wh + wm;
    uint64_t misses  = rm + wm;
    /* hit_rate_per_million = total ? (rh + wh) * 1000000ull / total : 0; */
}
```

### Timers / interrupts — base `0xF00F_0000`

Per-source interrupt controller and the source-0 timer. The CPU supports up
to 4 interrupt sources; source 0 is wired to the periodic timer below, and
sources 1–3 are reserved for future peripherals. The only interrupt-related
opcode is `IRET` (return from handler) — everything else is configured here.

| Offset  | Reg            | RW | Width | Description |
|---------|----------------|----|-------|-------------|
| 0x0000  | `INT_MASK`     | RW | 4     | Per-source enable; bit N = source N. Only bits [3:0] are used; upper bits ignored on write, read as 0. |
| 0x0008  | `INT_PENDING`  | R  | 4     | Live pending bits. Bit 0 = `r_timer_interrupt` (source 0); bits 1–3 reserved (read 0). |
| 0x0010  | `INT_VEC0`     | RW | 32    | Handler byte address for source 0 (timer). A vector of 0 disables the source even when its mask bit is set. |
| 0x0018  | `INT_VEC1`     | RW | 32    | Handler for source 1 (reserved). |
| 0x0020  | `INT_VEC2`     | RW | 32    | Handler for source 2 (reserved). |
| 0x0028  | `INT_VEC3`     | RW | 32    | Handler for source 3 (reserved). |
| 0x0030  | `TIMER_PERIOD` | RW | 32    | Source-0 period in raw `i_Clk` cycles. Writing it resets the cycle counter so the new period takes effect immediately. |
| 0x0038  | `TIMER_COUNT`  | R  | 32    | Live cycle counter — useful for profiling. |
| 0x0040  | `CLOCK_MS`     | R  | 64    | Free-running millisecond counter since FPGA power-on. Increments every 100 000 cycles at the 100 MHz `i_Clk` (constrained in `nexys_ddr.xdc`). 64-bit so it takes ~5.8 × 10⁸ years to wrap. Not reset by `CPU_RESETN` or by program load — programs that need a "time since started" value should snapshot it on entry. |

**Reset values:** `INT_MASK = 0`, all four `INT_VECn = 0`, `TIMER_PERIOD =
0x000F_FFFF` (≈ 10.5 ms at 100 MHz), `CLOCK_MS = 0` (at FPGA power-on only).
Interrupts are disabled at boot until software writes a vector and unmasks
the source. `CLOCK_MS` keeps ticking across `CPU_RESETN` and program loads —
it represents real time since the FPGA was configured, not since the
program started.

**Atomicity of `CLOCK_MS`:** read it as a single 64-bit `MEMGET64` to avoid
tearing — the upper half can change between two 32-bit reads. There is no
shadow latch, so a 32-bit read of the low half followed by the high half
will occasionally observe a +1 ms carry between them. Either use `MEMGET64`
or do the read-twice-and-retry-if-high-changed dance.

**Dispatch behaviour** (handled inside the CPU, not the MMIO regs):

- An interrupt fires when `INT_MASK[N] & INT_PENDING[N] & (INT_VECn != 0)`.
- On dispatch the hardware pushes a 64-bit context slot onto the stack
  (PC, flags, mask — see `CPU_ARCHITECTURE.md` §13.1), clears `INT_MASK[N]`
  for the dispatched source so the handler cannot re-enter on the same
  source, and jumps to `INT_VECn`.
- `IRET` (opcode `0x0000_6011`) pops the slot and restores all three.

**Pending-bit semantics:** `INT_PENDING[0]` follows the timer counter
edge — it asserts when the counter rolls over, and is auto-cleared by
hardware when the interrupt is dispatched. There is no W1C path; software
that wants to "swallow" a pending timer event without running a handler
should clear `INT_MASK[0]` first, then later re-enable.

**Coexistence with the LOAD_COMPLETE reset:** pressing the load button
re-zeroes `INT_MASK` and all four vectors, so a freshly-loaded program
always starts with interrupts off.

## Reserved (planned)

### UART — base `0xF001_0000`

Will expose TX (write triggers send), RX (read consumes a FIFO byte), and
STATUS (TX busy / RX FIFO empty / RX FIFO full bits). Detailed layout TBD.

## Software view

### C header

For the compiler and C library, define the regions as `volatile` pointers.
Width of access maps directly to the load/store opcode the compiler emits.

```c
/* mmio.h — KlaussCPU memory-mapped peripherals */
#ifndef KLAUSS_MMIO_H
#define KLAUSS_MMIO_H

#include <stdint.h>

#define MMIO_BASE       0xF0000000u

/* SD card (0xF000_0xxx) */
#define SD_BASE         (MMIO_BASE + 0x00000000u)
#define REG_SD_CTRL     (*(volatile uint32_t *)(SD_BASE + 0x0000))
#define REG_SD_DATA     (*(volatile uint32_t *)(SD_BASE + 0x0008))
#define REG_SD_STATUS   (*(volatile uint32_t *)(SD_BASE + 0x0010))
#define SD_BUF_PTR      ((volatile uint8_t  *)(SD_BASE + 0x0200))   /* 512-byte sector buffer */
#define SD_BUF_PTR_64   ((volatile uint64_t *)(SD_BASE + 0x0200))

/* SD_CTRL bits */
#define SD_CTRL_CLKDIV(d)  ((d) & 0xFFFFu)         /* SCK half-period in clocks */
#define SD_CTRL_CS_HI      (1u << 16)              /* deassert chip-select */
#define SD_CTRL_PWR_ON     (1u << 17)              /* power the slot */

/* SD_STATUS bits */
#define SD_STATUS_BUSY     (1u << 0)
#define SD_STATUS_CARD     (1u << 1)

/* RGB LEDs (0xF002_0xxx) */
#define RGB_BASE        (MMIO_BASE + 0x00020000u)
#define REG_RGB1        (*(volatile uint32_t *)(RGB_BASE + 0x0000))
#define REG_RGB2        (*(volatile uint32_t *)(RGB_BASE + 0x0008))

/* 7-segment display (0xF003_0xxx) */
#define SEG_BASE        (MMIO_BASE + 0x00030000u)
#define REG_SEG_LOW     (*(volatile uint32_t *)(SEG_BASE + 0x0000))
#define REG_SEG_HIGH    (*(volatile uint32_t *)(SEG_BASE + 0x0008))
#define REG_SEG_ALL     (*(volatile uint32_t *)(SEG_BASE + 0x0010))
#define REG_SEG_BLANK   (*(volatile uint32_t *)(SEG_BASE + 0x0018))

/* LEDs / switches (0xF004_0xxx) */
#define IO_BASE         (MMIO_BASE + 0x00040000u)
#define REG_LEDS        (*(volatile uint32_t *)(IO_BASE + 0x0000))
#define REG_SWITCHES    (*(volatile uint32_t *)(IO_BASE + 0x0008))

/* Cache controller (0xF005_0xxx) — counters + control */
#define CACHE_BASE        (MMIO_BASE + 0x00050000u)
#define REG_CACHE_CTRL    (*(volatile uint32_t *)(CACHE_BASE + 0x0000))
#define REG_CACHE_INFO    (*(volatile uint64_t *)(CACHE_BASE + 0x0008))
#define REG_CACHE_RD_HITS    (*(volatile uint64_t *)(CACHE_BASE + 0x0040))
#define REG_CACHE_RD_MISSES  (*(volatile uint64_t *)(CACHE_BASE + 0x0048))
#define REG_CACHE_WR_HITS    (*(volatile uint64_t *)(CACHE_BASE + 0x0050))
#define REG_CACHE_WR_MISSES  (*(volatile uint64_t *)(CACHE_BASE + 0x0058))
#define REG_CACHE_WRITEBACKS (*(volatile uint64_t *)(CACHE_BASE + 0x0060))
#define REG_CACHE_STALL_CYC  (*(volatile uint64_t *)(CACHE_BASE + 0x0068))

#define CACHE_CTRL_CLEAR  (1u << 0)        /* W1AC: zero all counters */

/* Interrupt controller / timer (0xF00F_0xxx) */
#define INTC_BASE       (MMIO_BASE + 0x000F0000u)
#define REG_INT_MASK    (*(volatile uint32_t *)(INTC_BASE + 0x0000))
#define REG_INT_PEND    (*(volatile uint32_t *)(INTC_BASE + 0x0008))
#define REG_INT_VEC(n)  (*(volatile uint32_t *)(INTC_BASE + 0x0010 + 8u*(n)))
#define REG_TIMER_PER   (*(volatile uint32_t *)(INTC_BASE + 0x0030))
#define REG_TIMER_CNT   (*(volatile uint32_t *)(INTC_BASE + 0x0038))
#define REG_CLOCK_MS    (*(volatile uint64_t *)(INTC_BASE + 0x0040))   /* atomic 64-bit read */

#define INT_SRC_TIMER   0u

/* Pack a 24-bit RGB triple (R,G,B each 0..15) into a 12-bit register value. */
static inline uint32_t rgb12(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r & 0xF) << 8) | ((uint32_t)(g & 0xF) << 4) | (b & 0xF);
}

#endif
```

### Example usage

```c
#include "mmio.h"

void demo(void) {
    REG_LEDS    = 0xAAAA;            /* alternating LED bar */
    REG_RGB1    = rgb12(15, 0, 0);   /* red on RGB1 */
    REG_RGB2    = rgb12(0, 0, 15);   /* blue on RGB2 */
    REG_SEG_ALL = 0xDEADBEEF;        /* "DEADBEEF" across all 8 digits */

    while ((REG_SWITCHES & 0x1) == 0) { /* spin until switch 0 raised */ }

    REG_SEG_BLANK = 0;               /* value ignored — write triggers */
}
```

### Interrupt setup

```c
#include "mmio.h"

extern void timer_isr(void);  /* must end with the IRET opcode */

void enable_timer(uint32_t period_cycles) {
    REG_TIMER_PER         = period_cycles;        /* e.g. 100000 ≈ 1 ms @ 100 MHz */
    REG_INT_VEC(INT_SRC_TIMER) = (uint32_t)&timer_isr;
    REG_INT_MASK          = 1u << INT_SRC_TIMER;  /* unmask source 0 */
}
```

The handler must end in `IRET` (currently no compiler intrinsic — write
the prologue/epilogue in inline assembly). Hardware auto-clears
`INT_MASK[0]` on dispatch and `IRET` restores it, so the handler runs
without re-entering on its own source. Other sources stay enabled.

### SD card library (sketch)

```c
/* sd.c — minimal SD-over-SPI driver for KlaussCPU. SDHC/SDXC only
   (block-addressed; CMD8 echo and ACMD41 HCS=1). */
#include "mmio.h"

/* clk_div presets (i_Clk = 100 MHz) */
#define SD_CLK_INIT   249   /* ~200 kHz */
#define SD_CLK_FAST     1   /* ~25 MHz */

/* Send one byte, return the byte received in the same SPI exchange. */
static uint8_t sd_xfer(uint8_t tx) {
    REG_SD_DATA = tx;
    while (REG_SD_STATUS & SD_STATUS_BUSY) { }
    return (uint8_t)REG_SD_DATA;
}

static void sd_cs(int assert) {
    uint32_t ctrl = REG_SD_CTRL & 0xFFFFu;          /* keep clk_div */
    if (!assert) ctrl |= SD_CTRL_CS_HI;
    REG_SD_CTRL = ctrl | SD_CTRL_PWR_ON;
}

static void sd_set_clk(uint16_t div) {
    REG_SD_CTRL = (REG_SD_CTRL & ~0xFFFFu) | div;
}

/* Send a 6-byte SD command and read R1 response (poll up to 8 bytes). */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    sd_xfer(0xFF);                                  /* extra clock */
    sd_xfer(0x40 | (cmd & 0x3F));
    sd_xfer((uint8_t)(arg >> 24));
    sd_xfer((uint8_t)(arg >> 16));
    sd_xfer((uint8_t)(arg >>  8));
    sd_xfer((uint8_t)(arg      ));
    sd_xfer(crc | 0x01);                            /* stop bit */
    for (int i = 0; i < 8; i++) {
        uint8_t r = sd_xfer(0xFF);
        if ((r & 0x80) == 0) return r;
    }
    return 0xFF;                                    /* timeout */
}

/* Application-specific command: CMD55, then ACMD<n>. */
static uint8_t sd_acmd(uint8_t acmd, uint32_t arg) {
    sd_cmd(55, 0, 0x65);
    return sd_cmd(acmd, arg, 0x77);
}

int sd_init(void) {
    sd_set_clk(SD_CLK_INIT);
    REG_SD_CTRL = SD_CLK_INIT | SD_CTRL_CS_HI;      /* CS high, slot off */
    REG_SD_CTRL = SD_CLK_INIT | SD_CTRL_CS_HI | SD_CTRL_PWR_ON;
    for (volatile int i = 0; i < 100000; i++) { }   /* ~1 ms power-up */

    for (int i = 0; i < 10; i++) sd_xfer(0xFF);     /* 80 dummy clocks, CS high */

    sd_cs(1);
    if (sd_cmd(0, 0, 0x95) != 0x01) { sd_cs(0); return -1; }   /* CMD0 → idle */
    if (sd_cmd(8, 0x1AA, 0x87) != 0x01) { sd_cs(0); return -2; } /* CMD8: SDv2 */
    for (int i = 0; i < 4; i++) sd_xfer(0xFF);      /* discard R7 trailing 4 B */

    /* ACMD41 with HCS=1 — loop until card leaves idle. */
    for (int tries = 0; tries < 1000; tries++) {
        if (sd_acmd(41, 0x40000000u) == 0x00) goto ready;
    }
    sd_cs(0);
    return -3;

ready:
    sd_cs(0);
    sd_set_clk(SD_CLK_FAST);
    return 0;
}

/* Read a 512-byte block at LBA `lba` into the sector buffer. */
int sd_read_block(uint32_t lba) {
    sd_cs(1);
    if (sd_cmd(17, lba, 0xFF) != 0x00) { sd_cs(0); return -1; }

    /* Wait for data start token 0xFE. */
    for (int tries = 0; tries < 100000; tries++) {
        if (sd_xfer(0xFF) == 0xFE) goto got_token;
    }
    sd_cs(0);
    return -2;

got_token:
    /* Read 512 bytes directly into the sector buffer. */
    for (int i = 0; i < 512; i++) SD_BUF_PTR[i] = sd_xfer(0xFF);
    sd_xfer(0xFF); sd_xfer(0xFF);                    /* discard CRC16 */
    sd_cs(0);
    return 0;
}

/* Write the sector buffer to block `lba`. */
int sd_write_block(uint32_t lba) {
    sd_cs(1);
    if (sd_cmd(24, lba, 0xFF) != 0x00) { sd_cs(0); return -1; }
    sd_xfer(0xFF);                                   /* gap */
    sd_xfer(0xFE);                                   /* start token */
    for (int i = 0; i < 512; i++) sd_xfer(SD_BUF_PTR[i]);
    sd_xfer(0xFF); sd_xfer(0xFF);                    /* dummy CRC16 */
    uint8_t resp = sd_xfer(0xFF) & 0x1F;             /* data response */
    if (resp != 0x05) { sd_cs(0); return -2; }
    while (sd_xfer(0xFF) == 0x00) { }                /* wait until card not busy */
    sd_cs(0);
    return 0;
}
```

### SD usage example

```c
#include "mmio.h"
extern int sd_init(void);
extern int sd_read_block(uint32_t lba);

void show_sector_zero(void) {
    if (sd_init() != 0) {
        REG_SEG_ALL = 0xE0000000;       /* "E" for error on top digit */
        return;
    }
    if (sd_read_block(0) != 0) {
        REG_SEG_ALL = 0xE0000001;
        return;
    }
    /* MBR signature lives at offset 510 — should be 0x55AA. */
    uint32_t sig = SD_BUF_PTR[510] | (SD_BUF_PTR[511] << 8);
    REG_SEG_ALL = sig;                  /* lower 4 digits show "55AA" */
}
```

## Compiler / linker notes

- The MMIO range (`0xF000_0000`+) is far above the 128 MB DRAM (`0x0800_0000`)
  and the stack base (`0x0800_0000` growing down), so **no linker change is
  needed** — the MMIO addresses cannot collide with normal data placement.
- Mark MMIO addresses as `volatile` (or the equivalent in your IR) so the
  compiler does not eliminate or reorder accesses. There is no cache to flush.
- Stores must be word-sized (or the matching `MEMSETxx` opcode) so the byte
  enables hit the right lanes; writing a single byte to a 32-bit register
  works (the byte enables drive only one lane), but the upper bytes of the
  register are then undefined — pick a width and stick to it.
- All MMIO registers are 8-byte aligned to keep `LD64`/`ST64` access trivial.
  Register offsets within a device are spaced by 8 even when the register is
  narrower, so future widening doesn't move the address.

## Migration plan (legacy opcodes → MMIO)

Both paths currently coexist. Any legacy opcode (`RGB1R`, `7SEGR`, `LEDR`,
etc.) updates the same underlying register that MMIO writes update — only one
can fire on a given clock, so there is no driver conflict.

To deprecate the opcodes:

1. Stop emitting them from the compiler / assembler. Programs use plain
   `ST` to MMIO addresses instead.
2. Once no shipped program uses an opcode, mark it deprecated in
   `opcode_select.vh` (comment) and run for a release cycle.
3. Delete the opcode entry from the dispatch table and the corresponding task
   from `led_tasks.vh` / `seven_seg.vh`.
4. The peripheral state regs and the MMIO handler stay unchanged.

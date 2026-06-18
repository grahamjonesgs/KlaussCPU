# FPGA handoff: 2D DMA blitter for KlaussCPU

Self-contained spec for adding a **2D DMA blitter** to the KlaussCPU soft core
to accelerate framebuffer fill / copy / alpha-blend (GUI + VNC workloads).
Assumes only the core + memory map — no Zephyr/LVGL knowledge needed.

Companion analysis: [`../vnc/PERFORMANCE.md`](../vnc/PERFORMANCE.md) (VNC pixel
pipeline) and the `gui_lvgl` render benchmark (`apps/gui_lvgl`, with
`CONFIG_GUI_LVGL_BENCHMARK=y`) which produced the numbers below.

## Why (measured)

From CPU/cache perf counters (`board_io.h`, blocks `0xF005` cache / `0xF00D`
pipeline) on an LVGL full-screen render benchmark:

- **Core is CPI ≈ 8** (≈ one retired instruction per 8 cycles), with memory
  **stall only 7–19%** and cache **read hit 98% / write hit 93%**. So per-pixel
  work is *execution*-bound on a slow core, **not** memory-latency bound — a
  bigger cache would not help.
- The framebuffer flush (600 KB memcpy) takes **~367 ms ≈ 1.6 MB/s**, and that
  did **not** improve with word-wise vs byte-wise `memcpy` → it is
  **memory-write-bandwidth bound** (latency-bound writebacks, no bursting).

A blitter helps **even without changing the core**:
- It runs **concurrently** with the CPU → frame time becomes
  `max(render, copy)` instead of `render + copy` (~1.5–1.9× on its own).
- With **burst** memory access it moves bytes far faster than the CPU's
  1.6 MB/s.
- Fills/blends done in fabric skip the CPI-8 core entirely.

Bounded by the non-offloadable CPU work (LVGL overhead, glyph/​shadow
algorithms), so expect ~1.5–3× on typical UIs; the systemic ~5× still needs
core pipelining. The blitter and core-IPC work are complementary.

## Interface: MMIO DMA engine — no new CPU opcodes required

The blitter is a **bus-master DMA**: the CPU writes operand registers + a START
bit and continues; the engine reads/writes main memory autonomously and signals
DONE (poll or IRQ). No new instructions are needed for the blitter itself. (A
`BLEND565` instruction is a separate, optional scalar accelerator — see end.)

## Operations

Rectangular, row-major, **RGB565 (16bpp, little-endian)**. Per-row strides let a
rect be a sub-region of a larger buffer (src and dst strides may differ).

| op | semantics |
|----|-----------|
| `FILL`        | `dst = color` |
| `FILL_BLEND`  | `dst = blend(color, dst, alpha)` (global alpha) |
| `COPY`        | `dst = src` |
| `COPY_BLEND`  | `dst = blend(src, dst, alpha)` (global alpha) |
| `MASK_BLEND`  | `dst = blend(color|src, dst, mask[x,y])`, `mask` = 8bpp (A8) per-pixel alpha — for anti-aliased text/edges |

Blend math, per RGB565 channel (unpack R5/G6/B5 → blend → repack):
`out = (fg*a + bg*(255-a) + 128) >> 8`  (or the `*257 >> 16` /255 approximation).
Match LVGL's `>>8` if bit-exactness vs the software path is wanted.

## Memory model

- Buffers live in main RAM, byte-addressed. RAM ≤ 128 MB, so **32-bit addresses
  suffice**.
- `*_ADDR` = byte address of the rect's top-left pixel; `*_STRIDE` = bytes per
  row. (Arbitrary sub-rects; e.g. a 320-wide source into a 640-wide
  framebuffer.)
- `WIDTH`/`HEIGHT` in pixels. **No clipping in HW** — the CPU pre-clips and
  passes an exact rect.
- **Burst** reads/writes are the point — size to saturate the memory
  controller, not pay per-access latency.

## Proposed MMIO register map

Base **`0xF00B0000`** (free block — `0xF005` cache, `0xF006/8` eth, `0xF00D`
perf, `0xF004` LEDs, `0xF003` 7-seg are taken). 32-bit registers.

| offset | reg | meaning |
|--------|-----|---------|
| 0x00 | `CTRL`       | bit0 START (W1); bits[3:1] OP (0 FILL, 1 COPY, 2 FILL_BLEND, 3 COPY_BLEND, 4 MASK_BLEND); bit4 IRQ_EN |
| 0x04 | `STATUS`     | bit0 BUSY; bit1 DONE (write-1-clear) |
| 0x08 | `DST_ADDR`   | dst top-left byte address |
| 0x0C | `DST_STRIDE` | dst row bytes |
| 0x10 | `SRC_ADDR`   | src top-left byte address (COPY/COPY_BLEND) |
| 0x14 | `SRC_STRIDE` | src row bytes |
| 0x18 | `WIDTH`      | pixels |
| 0x1C | `HEIGHT`     | rows |
| 0x20 | `COLOR`      | RGB565 in bits[15:0] (FILL / FILL_BLEND / MASK_BLEND-with-color) |
| 0x24 | `ALPHA`      | global alpha 0..255 (*_BLEND) |
| 0x28 | `MASK_ADDR`  | A8 mask top-left byte address (MASK_BLEND) |
| 0x2C | `MASK_STRIDE`| mask row bytes |
| 0x30 | `CYCLES`     | (optional) cycle count of last blit, for profiling |

**Programming sequence:** write operands → write `CTRL` (OP | START) → poll
`STATUS.BUSY == 0` (or wait for IRQ) → optionally read `CYCLES`.

## ⚠ Cache coherency — critical companion requirement

The CPU accesses the framebuffer/source through its **write-back cache**; the
blitter hits main memory directly. Without coherency, stale reads/writes occur.
Choose one:

1. **Cache-coherent DMA** (cache snoops blitter traffic) — cleanest for
   software, most RTL effort.
2. **Non-coherent DMA + cache-maintenance MMIO** (typical embedded pattern):
   the driver must **flush** src/dst from cache before a blit and **invalidate**
   dst after. This needs **new cache-control MMIO** — e.g. range
   flush/invalidate registers at the cache block (`0xF005…`):
   `FLUSH_ADDR`/`FLUSH_LEN`/`FLUSH_GO` and `INVAL_*`. **This is the extra MMIO
   beyond the blitter that the project needs** — software cannot be correct
   without it (or option 1/3).
3. **Uncached region** for the buffers — simplest HW, but slows CPU access to
   them (the CPU writes the render buffer through cache, so this hurts).

Recommended: option 2 unless coherent DMA is cheap on this design.

## Performance target

Beat the CPU's ~1.6 MB/s. DDR2 on the Nexys A7 is capable of 100s of MB/s; even
~50 MB/s does the 600 KB copy in **~12 ms vs 367 ms (~30×)**, concurrently with
the CPU. Burst full cache-line / DDR-row-sized transfers.

## Software consumers (so the register design fits)

- Zephyr display driver: replaces its per-row `memcpy` (render buffer →
  framebuffer at an x,y offset) with one `COPY` blit — needs independent
  src/dst strides + addresses (provided).
- LVGL GPU hooks: fill → `FILL`/`FILL_BLEND`, image → `COPY`/`COPY_BLEND`,
  glyph/AA → `MASK_BLEND`.
- VNC server: `COPY` for its framebuffer → TX-buffer path.

All pass explicit addresses/strides; the blitter hardcodes no buffer locations.

## Verification

- **Unit:** `FILL` a known colour; `COPY` a pattern with *differing* src/dst
  strides; `BLEND` with alpha=128 and with a mask — read back and compare to a
  CPU reference (bit-exact for fill/copy; ±1 LSB acceptable for the /255-approx
  blend unless matching LVGL's `>>8`).
- **Coherency:** CPU writes a buffer → blit it → CPU reads dst → must see fresh
  data (exercises the flush/invalidate path).
- **Perf:** read `CYCLES` (or the `0xF00D` cycle counter) around a 600 KB
  `COPY`; target ≪ 367 ms, and confirm the CPU runs concurrently during BUSY.

## Optional (separate, lower priority): `BLEND565` opcode

A single instruction `rd = blend565(rfg, rbg, ralpha)` — unpack both RGB565,
per-channel `(fg*a + bg*(255-a)) >> 8`, repack — collapses ~20–30
instructions/pixel to one. Helps the *non-blittable* per-pixel paths and the
existing software blend. Complementary to the blitter; not required for it.

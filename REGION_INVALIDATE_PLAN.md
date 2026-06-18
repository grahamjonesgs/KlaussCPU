# Sizing plan: region-based cache flush / invalidate

Upgrade over the shipped **full-cache** flush/invalidate (Phase 4). Goal: flush
or invalidate only the cache lines covering a byte range `[base, base+len)`,
instead of walking all 2048 sets every time. This is the handoff doc's
"range flush/invalidate" option and the natural efficiency win for partial
framebuffer updates (LVGL dirty rects).

## TL;DR — size

**MEDIUM (~1–2 days).** It's an *extension* of the existing Phase-4 `MAINT`
FSM, not new infrastructure — the arbiter, CDC writeback gap, coherency
bracketing, MMIO block, and testbench harness are all reused. Rough deltas:

| Area | Change | ~Lines |
|------|--------|--------|
| `mem_read_write.v` | range-mode in `MAINT` (line-address counter + per-way tag-match gating) + new ports | ~80–110 |
| `KlaussCPU.v` | `RANGE_ADDR`/`RANGE_LEN` regs, 2 `CACHE_CTRL` bits, read-mux, port wiring | ~20 |
| `MMIO_MAP.md` | doc the range registers + driver guidance | ~30 |
| driver (C, external repo) | range helpers + size/2D selection | ~40 |
| `tb_cache.v` | 5–6 range coherency cases | ~80 |

**Risk: medium**, concentrated in one spot (the per-way tag-match gating of the
dirty/valid clears — see Risk R1). No new CDC, no arbiter changes.

## The key fact that decides the design

This cache is **2-way set-assoc, 2048 sets, 16-byte lines**, with
`index = addr[14:4]`, `tag = addr[31:15]`. **A byte address maps to exactly one
set**, and the address span that covers every set once is `2048 × 16 = 32 KB`.

That gives a hard crossover between the two possible hardware strategies:

- **(A) Address-walk** — iterate the range one 16-byte line at a time; for each
  line address `A`, read both ways at `index(A)`, and act on the way whose tag
  matches `tag(A)`. Cost ≈ `len/16` line-probes. **Exact** (tag-match ⇒ no false
  hits, every line probed ⇒ no misses).
- **(B) Set-walk + range filter** — walk all 2048 sets (like the full walk) and
  act on a line only if its reconstructed address is in range. Cost ≈ the full
  walk (~8 k cycles) **regardless of range size**.

Address-walk beats a full/set walk only while `len/16 < ~2048×2`, i.e. roughly
**`len < 32 KB`**. Above that it does *more* work than the existing full walk
(e.g. a 600 KB region = 37 500 probes ≫ the 2048-set full walk). So:

> **Recommendation: implement (A) address-walk for ranges, and keep the existing
> full-cache ops as the large-region path. The driver picks full vs range by
> size.** Don't build (B) — it never wins for the common small-dirty-rect case
> and is redundant with the full walk for large ones.

Address-walk is also the *smaller* RTL delta (it reuses the per-line probe shape
the cache already has), which is why it's the recommendation on both axes.

## 2D rectangles (blitter) — push the 2D logic to the driver

A blit rect `(base, stride, width, height)` is **non-contiguous** when
`stride > width*2`. Keep the hardware to a simple **contiguous `[base, len)`
range** and let the driver decide how to cover a 2D rect:

- **Small / near-contiguous rect** → one range op over the bounding span
  `[base, base + (height-1)*stride + width*2)`. Over-invalidating the inter-row
  gaps is always **safe**, just slightly wasteful.
- **Tall-thin rect in a wide buffer** (bounding span ≈ whole framebuffer) → the
  driver either issues **one range op per row** (`height` ops, each
  `width*2` bytes, exact) or just falls back to the **full-cache** op. A simple
  driver heuristic: `if bounding_span ≥ ½ cache_size → full op; elif
  height ≤ N → per-row range ops; else → bounding range op`.

This keeps the hardware change minimal (no stride/width/height awareness in the
maintenance FSM) and is where the existing full-cache ops earn their keep.

## MMIO additions (reuse the `0xF005` block)

| Offset | Reg | Notes |
|--------|-----|-------|
| 0x0018 | `RANGE_ADDR` | range base byte address (HW aligns down to 16 B) |
| 0x0020 | `RANGE_LEN`  | range length in bytes (HW rounds up to a 16 B multiple) |
| 0x0000 | `CACHE_CTRL` | add `[3]` RANGE_FLUSH_GO, `[4]` RANGE_INVAL_GO (self-clearing; consume `RANGE_ADDR/LEN`) |

`CACHE_STATUS.MNT_BUSY` (0x0010) already covers the busy/poll path unchanged.
Existing `[1]` full-flush / `[2]` full-invalidate stay as the large-region path.
Contract: write `RANGE_ADDR`/`RANGE_LEN`, then pulse the GO bit; it runs
synchronously exactly like the full ops (CPU's next access stalls until done).

## RTL changes (`mem_read_write.v`) — the actual work

The `MAINT` sub-FSM already does the hard parts (BRAM read pipeline, per-way
writeback, CDC gap, dirty/valid clears, arbiter hold). Add a **range mode**:

1. **New state:** `r_rng_active` / `r_rng_mode` (flush vs inval) + a 32-bit
   **line-address counter** `r_line_addr` and an end `r_rng_end` (= aligned
   base+len). New module inputs `i_rng_addr`, `i_rng_len`, `i_rng_flush_go`,
   `i_rng_inval_go` (KlaussCPU holds the `RANGE_*` regs and pulses).
2. **Pending capture:** extend the existing pending always-block to latch the
   range base/end/mode alongside the full-mode request.
3. **`MS_READ` (range):** read both ways at `index = r_line_addr[14:4]`.
4. **`MS_W0`/`MS_W1` (range):** writeback a way **only if** `valid && dirty &&
   way_tag == r_line_addr[31:15]` (vs full mode: every valid+dirty line). Address
   is `{r_line_addr[31:4], 4'b0}`.
5. **`MS_CLR` (range):** clear dirty (flush) or valid (invalidate) **only on the
   tag-matching way(s)** — not unconditionally. Then `r_line_addr += 16`; if
   `r_line_addr >= r_rng_end` → `MS_DONE`, else → `MS_READ`.
6. **`dirty0/1_wen` qualifier:** today they add `state==MAINT && MS_CLR`
   unconditionally; in range mode they must be gated by the per-way tag match
   (`w_rng_match_way0/1`). This is the one place that needs care to preserve the
   single-write-port distributed-RAM inference (R1).

Full-cache mode is unchanged (the `r_rng_active==0` path). The arbiter
`r_mnt_active` guard, the writeback CDC gap (`r_gap_count`), and the WAIT-state
priority entry all work as-is for range mode.

## Driver changes (external Zephyr/LVGL repo)

- `cache_flush_range(addr, len)` / `cache_inval_range(addr, len)`: write
  `RANGE_ADDR/LEN`, pulse the GO bit, (optionally poll `MNT_BUSY`).
- Blitter coherency helpers updated to call the range ops with the size/2D
  heuristic above, falling back to full ops for large/tall-thin rects.

## Test plan (`tb_cache.v`, reuses the Phase-4 harness)

- range-flush: dirty lines **inside** the range reach DDR; dirty lines **outside**
  the range stay dirty and are **not** written back.
- range-invalidate (clean line): in-range line drops (CPU refills fresh from
  DDR); out-of-range line still hits stale-cached.
- range-invalidate (dirty line): in-range dirty line written back then dropped.
- partial overlap / unaligned base+len (alignment rounding).
- `len = 0` (no-op) and range ≥ cache size (covers everything; still correct).
- 2D bounding-range case (over-invalidate of inter-row gaps is harmless).

## Risks

- **R1 (main):** the per-way tag-match gating of the dirty/valid clears must keep
  the `dirty0/1_wen` / `cache_val_addr` writes to a single write port per array
  (the existing DRAM/BRAM inference depends on this). Mismatched-index or
  multi-condition writes can degrade inference or add mux trees. Mitigate: gate
  by a clean combinational `w_rng_match_wayN` wire, keep one write address.
- **R2:** address-walk on a large range is *slower* than the full walk — purely a
  performance footgun, prevented by the driver size heuristic, not a correctness
  issue. Document the crossover.
- **R3:** alignment contract (base/len rounding) — pick HW-rounds-internally to
  avoid driver bugs; cheap.
- No new CDC, no arbiter/CPU-interaction changes, no new interrupt interaction —
  this is why it stays Medium, not Large.

## What this does NOT need

- No change to the blitter, the DMA arbiter, the IRQ path, or the coherency
  *sequence* (still flush-before / invalidate-after — just with range ops).
- No second FSM — it folds into the proven `MAINT` state.

# Implementation plan: 2D DMA blitter for KlaussCPU

**STATUS: all 6 phases complete, sim-verified, and CLOSED IN VIVADO.** RTL done;
two adversarial review passes (Phase 3, Phase 4/5) + a doc-vs-RTL verification
pass, all clean. **Implementation on XC7A100T: P&R complete, timing MET at
100 MHz — WNS +0.070 ns / WHS +0.022 ns, 0 failing endpoints; LUTRAM 2584/19000,
BRAM 93/135.** Two bring-up issues were found and fixed (see Phase 6 notes):
DRC UTLZ-1 LUT-as-DRAM over-util (unified cache read index) and WNS −4.037 ns on
the blend datapath (S_BLEND pipeline stage). Only remaining work: the user's
on-board performance test (and bitstream generation).

Implements [`blitter-fpga-handoff.md`](blitter-fpga-handoff.md). Decisions taken:

- **Coherency:** full-cache flush + invalidate MMIO (new FSM in the cache).
- **Scope of first deliverable:** all 5 ops (FILL, COPY, FILL_BLEND, COPY_BLEND,
  MASK_BLEND) **plus** a DONE interrupt wired into the CPU interrupt table.

## Key facts that shape the design (verified against RTL)

1. **The CPU is the sole DDR2 master.** `ddr2_control` (the MIG wrapper) is
   instantiated *inside* [`mem_read_write.v`](KlaussCPU.srcs/sources_1/new/mem_read_write.v#L883)
   — it is not a top-level block. The clean 128-bit `o_ddr_mem_*` / `i_ddr_mem_*`
   request/ready interface lives between the cache FSM and `ddr2_control` *within*
   that module ([mem_read_write.v:87-94](KlaussCPU.srcs/sources_1/new/mem_read_write.v#L87)).
   Therefore **the blitter's DMA traffic must be arbitrated onto DDR inside
   `mem_read_write.v`**, not at the top level.
2. **Cache:** 2-way set-assoc, write-back, 64 KB, 2048 sets × 128-bit lines,
   index `addr[14:4]`, tag `addr[31:15]`. Arrays: `cache_val_addr_way{0,1}`,
   `cache_val_data_way{0,1}`, `cache_dirty_way{0,1}`, `cache_lru`. **No flush
   exists today** — we add one. The flush/invalidate FSM must live in
   `mem_read_write.v` because it owns these BRAMs.
3. **DDR transactions are 128-bit, burst-aligned** (`addr[31:4]`, low 4 bits
   zeroed). The blitter will issue the same 128-bit aligned transactions; "burst"
   = back-to-back 128-bit ops keeping the DDR row open.
4. **MMIO:** decode is `addr[31:28]==0xF` → device = `addr[27:16]` → register =
   `addr[15:0]`. Read mux at [KlaussCPU.v:787-870](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L787),
   write handler at [KlaussCPU.v:1362-1434](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L1362).
   The handoff doc suggested `0xF00B`, but that block is **live SHA-256**
   ([KlaussCPU.v:723](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L723)). Used blocks:
   `000` SD, `002` RGB, `003` 7-seg, `004` LEDs, `005` cache, `006`/`008` eth,
   `00A` AES, `00B` SHA, `00C` TRNG, `00D` perf, `00F` IRQ/timer. **The blitter
   takes free block `0xF00E`** (`0xF00E0000`).
5. **IRQ:** `r_interrupt_table[3:0]` + `r_int_mask[3:0]` exist; only source 0
   (timer) is wired into `w_irq_ready` ([KlaussCPU.v:307](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L307)).
   Sources 1-3 are free — **blitter DONE → source 1.**
6. **Cache-control block `0xF005`** already exists (counters + `CACHE_CTRL`
   stat-clear). Flush/invalidate registers attach here, keeping all cache MMIO
   in one block.

## Architecture

```
            MMIO bus (0xF00E blitter regs, 0xF005 cache-ctrl regs)
                 │                                  │
        ┌────────▼─────────┐              ┌─────────▼──────────┐
        │  blitter_dma.v   │              │  KlaussCPU.v        │
        │  (new top block) │              │  MMIO read/write    │
        │  - reg file      │              │  + IRQ source 1     │
        │  - 2D address    │              └─────────┬──────────┘
        │    generator     │                        │ flush/inval cmd
        │  - blend datapath│                        │
        │  - 128-bit DMA   │   128-bit DMA port     ▼
        │    master FSM    │──────────────►┌──────────────────────┐
        └──────────────────┘   (rd/wr/rdy) │  mem_read_write.v     │
                                            │  - cache FSM (master A)│
                                            │  - flush/inval FSM     │
                                            │  - NEW: DDR arbiter    │
                                            │      A=cache  B=blitter │
                                            │  - ddr2_control (MIG)  │
                                            └──────────────────────┘
```

The blitter is a **non-coherent** master (hits DDR directly, bypasses the cache).
Software keeps it coherent with the new flush/invalidate operations.

## New / changed files

| File | Change |
|------|--------|
| `KlaussCPU.srcs/sources_1/new/blitter_dma.v` | **New.** Register file, 2D address generator, RGB565 blend datapath, 128-bit DMA master FSM. |
| `KlaussCPU.srcs/sources_1/new/mem_read_write.v` | DDR arbiter (cache vs blitter), expose a 128-bit DMA master port, add flush/invalidate FSM over the cache arrays. |
| `KlaussCPU.srcs/sources_1/new/KlaussCPU.v` | Instantiate `blitter_dma`; MMIO decode for `0xF00E`; flush/inval regs at `0xF005`; wire blitter port to `mem_read_write`; wire DONE to interrupt source 1. |
| `KlaussCPU.srcs/sim_1/new/tb_blitter.v` | **New.** Unit + coherency + perf testbench. |
| `MMIO_MAP.md` | Document `0xF00B` blitter regs and new `0xF005` flush/inval regs. |
| `blitter_driver.{c,h}` (software, `src/klausscc/src` tree) | Optional: C driver shim for Zephyr/LVGL/VNC consumers. |

## Phase breakdown

### Phase 1 — DDR arbitration in `mem_read_write.v` (enables a 2nd master) ✅ DONE
- Refactor the cache↔`ddr2_control` path so the cache FSM is **master A** behind
  a small priority arbiter onto the single `ddr2_control` request port.
- Add **master B**: a 128-bit DMA port exposed on the module boundary:
  `i_dma_read_DV, i_dma_write_DV, i_dma_addr[31:0], i_dma_write_data[127:0],
   i_dma_byte_en[15:0], o_dma_read_data[127:0], o_dma_ready, o_dma_grant`.
- **Arbiter policy:** CPU/cache has priority (it's latency-sensitive, CPI-8);
  blitter fills idle DDR cycles. Grant is per-transaction (release after each
  128-bit op) so a long blit cannot starve the CPU. Respect the existing
  CDC/settling gaps in `ddr2_control` (the 7-cycle write→read gap,
  [mem_read_write.v:512-530](KlaussCPU.srcs/sources_1/new/mem_read_write.v#L512)).
- **Milestone:** existing CPU + cache traffic unchanged; blitter port idle.
  Regression: boot + run an existing program in sim, confirm no behavioral diff.
- **Status:** Implemented. Registered grant `r_grant_blit`, muxed `mig_*`
  controller inputs, and per-master gated ready (`w_cache_ddr_ready` /
  `w_blit_ddr_ready`) added to `mem_read_write.v`; DMA master port exposed and
  tied off in `KlaussCPU.v`. `tb_cache.v` extended with a direct DMA driver —
  **all cache regression checks pass (read-hit latency still 2 cycles) and the
  new DMA-arbiter test passes** (DMA write→read-back, CPU sees DMA data in main
  RAM, bus returns cleanly to the cache). The blitter releases per-transaction
  via `i_dma_done` so the CPU is never starved; grant only moves to the blitter
  while the cache is not `is_miss_path`, preserving the CDC writeback gaps.

### Phase 2 — `blitter_dma.v` core: FILL + COPY, poll-only ✅ DONE
- MMIO register file per the map below; START (W1) latches operands and kicks the
  DMA FSM; BUSY/DONE in STATUS.
- **2D address generator:** nested row/col counters producing burst-aligned
  128-bit DST (and SRC for COPY) addresses from `*_ADDR` + `*_STRIDE`.
  Handle non-16-byte-aligned rect edges with byte-enable masking on the first/last
  128-bit word of each row (RGB565 = 2 bytes/pixel, 8 pixels per 128-bit word).
- **FILL:** replicate `COLOR` across the 128-bit word; stream writes.
- **COPY:** read SRC word → write DST word; pipeline read-ahead to keep the DDR
  row busy. Independent src/dst strides (handoff requirement).
- `CYCLES` counter: free-running during BUSY, latched at DONE.
- **Milestone:** FILL a rect and COPY a strided sub-rect in sim; bit-exact vs a
  CPU reference model in the testbench.
- **Status:** Implemented `blitter_dma.v` — MMIO register file (64-bit regs,
  8-byte spaced to match the AES/SHA convention; the handoff's 4-byte map was
  rebased), pixel-streaming engine with a 128-bit source read-buffer + 128-bit
  masked destination write-accumulator (handles arbitrary src/dst alignment,
  differing strides, and partial edge words via per-byte `wdf_mask` — no
  destination read-modify-write for opaque ops), free-running `CYCLES`. Wired
  into `KlaussCPU.v` at device `0x00E` with the DMA master on the Phase 1
  arbiter; read-mux case added. New `tb_blitter.v` (mask-aware DDR model) —
  **all 4 checks pass: aligned FILL, unaligned partial-mask FILL, aligned COPY,
  unaligned/differing-stride COPY**, vs a byte-level reference. Phase 1
  regression still green; full top-level elaborates (only vendor IP missing).
  Note: a testbench reset bug (held `i_Rst_L` high) initially masked X-state —
  fixed; the engine relies on the active-low reset that `~w_reset_H` provides on
  the real top level.

### Phase 3 — Blend datapath: FILL_BLEND, COPY_BLEND, MASK_BLEND ✅ DONE
- RGB565 unpack (R5/G6/B5) → per-channel `(fg*a + bg*(255-a) + 128) >> 8` → repack.
  Use the `>>8` form to match LVGL bit-exactly (handoff §Blend math).
- FILL_BLEND/COPY_BLEND use global `ALPHA`; MASK_BLEND fetches A8 alpha per pixel
  from `MASK_ADDR`/`MASK_STRIDE` (a **third** address stream — extend the address
  generator). Blend reads DST (read-modify-write), so each blend op is
  read-DST + (read-SRC/MASK) + write-DST.
- Pipeline: 8 pixels per 128-bit word → 8 parallel blend lanes, or a smaller
  lane count iterated, chosen for timing (see Risk R1).
- **Milestone:** all blend ops match the reference model (±0 LSB with `>>8`).
- **Status:** Implemented. Added a combinational `blend565` function
  (`(fg*a + bg*(255-a) + 128) >> 8` per 5/6/5 channel) and restructured the
  engine to a unified word-open model: opaque ops keep the masked-write path;
  blend ops read-modify-write the destination word (the dst is the blend
  background). A generic DMA-read sub-FSM (`S_RD_*`) fills any of three buffers —
  source (`r_rbuf`), A8 mask (`r_mbuf`), or destination background (`r_wbuf`).
  Foreground/alpha selection per op; **MASK_BLEND uses COLOR as the foreground**
  (anti-aliased text/glyph case) — documented in the module header, revisit if
  src-as-foreground is needed. `tb_blitter.v` extended to **11 passing checks**
  including FILL_BLEND at α=0/128/255, unaligned partial-mask blend,
  COPY_BLEND with differing strides/alignment, and MASK_BLEND with an A8 alpha
  ramp; reference `tb_blend565` matches `blend565` bit-for-bit. Phase 1
  regression still green; top-level elaborates.
- **Adversarial review (15 agents, 5 dimensions, per-finding verification):** one
  confirmed defect — CYCLES read 0 for an empty rect (w=0 or h=0) because the
  engine jumped to S_FINISH before the cycle counter ticked. **Fixed** (empty
  blits now seed `r_cycles<=1`) and locked with an `emptyrect-*` regression test.
  All other candidate findings (blend overflow, FSM deadlock, CDC races,
  mask-offset, addressing) were refuted by the verifiers. **13 tb_blitter checks
  now pass.**

### Phase 4 — Cache flush / invalidate FSM (coherency) ✅ DONE
- New FSM in `mem_read_write.v` that walks all 2048 sets × 2 ways:
  - **FLUSH:** for each dirty line, issue a 128-bit writeback via arbiter master A
    path, clear dirty. (~2-8k cycles; acceptable per-frame.)
  - **INVALIDATE:** clear valid bits (and optionally flush-then-invalidate).
- MMIO at `0xF005`: `FLUSH_GO` (W1, full-cache flush), `INVAL_GO` (W1, full
  invalidate), `CACHE_MNT_BUSY` status bit. (Range variants
  `FLUSH_ADDR/FLUSH_LEN` are a documented phase-6 upgrade — see handoff option 2.)
- Driver sequence: flush DST+SRC region → start blit → wait DONE → invalidate DST.
- **Milestone:** coherency test — CPU writes a buffer (lands in cache), flush,
  blit it, invalidate, CPU reads dst → sees fresh data.
- **Status:** Implemented as one new `MAINT` main-state with an internal sub-FSM
  (`MS_READ→W0→W0_WAIT→W0_GAP→W1→W1_WAIT→W1_GAP→CLR→DONE`) walking all 2048 sets ×
  2 ways. Reuses the `r_tag/r_dirty/r_data` pipeline regs, `r_cache_index` as the
  walk counter, and `r_gap_count` for the writeback CDC settle (all idle while
  maintenance runs, since the FSM only services it from `WAIT`). Writebacks go
  through the master-A DDR path; the arbiter holds the blitter off while
  `r_mnt_active` so a writeback is never interrupted mid-gap. **Maintenance has
  priority in `WAIT`**, so the CPU's next cached access transparently stalls until
  the walk finishes — software may treat flush/invalidate as synchronous.
  MMIO at `0xF005`: `CACHE_CTRL` (0x00) bit1 = FLUSH, bit2 = INVALIDATE
  (self-clearing pulses); `CACHE_STATUS` (0x10) bit0 = MNT_BUSY. `tb_cache.v`
  extended with 3 coherency checks — **flush writes back dirty (valid kept);
  invalidate exposes fresh main-memory data on a clean line; invalidate on a
  dirty line writes back first then drops (no data loss)** — all pass, alongside
  the existing cache + arbiter regression. tb_blitter still green; top-level
  elaborates.

### Phase 5 — IRQ integration ✅ DONE (under adversarial review)
- Add `r_blitter_done_irq` set on blitter DONE, cleared by STATUS write-1-clear.
- Extend `w_irq_ready` ([KlaussCPU.v:307](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L307))
  to OR in source 1:
  `(r_blitter_done_irq && r_interrupt_table[1]!=0 && r_int_mask[1])`.
  Generalize the dispatch (currently hard-coded to source 0) to select the vector
  for the firing source.
- CTRL bit4 `IRQ_EN` gates whether DONE raises the line.
- **Milestone:** ISR fires on blit completion; masked when `IRQ_EN`/`int_mask` off.
- **Status:** Implemented. `blitter_dma.o_irq = r_done & r_irq_en` (already
  present). In `KlaussCPU.v`: split `w_irq_ready` into per-source `w_irq_src0`
  (timer) / `w_irq_src1` (blitter: `w_blit_irq && table[1]!=0 && mask[1]`) with
  `w_irq_sel` (timer priority). The `OPCODE_REQUEST` dispatch is now
  source-selected — pushes the pre-dispatch mask (so IRET re-enables the source),
  sets `r_PC<=table[sel]`, masks `r_int_mask[sel]`, and clears `r_timer_interrupt`
  only for sel==0 (the blitter is level/sticky, acked by the ISR via STATUS W1C).
  Source 1 = blitter; 2-3 free. Blitter-side IRQ verified in `tb_blitter.v`
  (asserts on DONE w/ IRQ_EN, clears on W1C ack, stays low when IRQ_EN=0). The CPU
  dispatch path can't run under iverilog (needs the full core + vendor IP), so it
  is covered by elaboration + the adversarial review workflow.

### Phase 6 — Integration, timing, docs, software ✅ DONE (in-repo); ⏳ synthesis/board = user toolchain
- **Docs:** `MMIO_MAP.md` updated — new **2D DMA blitter** section (`0xF00E`, full
  register table, op table, blend math, programming sequence, coherency
  bracketing, IRQ source 1 + level-ack ISR example, C driver sketches
  `blit_copy`/`blit_fill`/`blit_isr`); **Cache controller** section updated with
  `CACHE_CTRL[1]`=FLUSH / `[2]`=INVALIDATE + `CACHE_STATUS` and the coherency
  sequence; **Timers/interrupts** section documents source 1 = blitter and the
  level-triggered ack contract; address-layout map updated (`0xF00E` no longer
  "reserved"). `INT_PENDING[1]` now reflects the blitter (RTL + doc).
- **Confirmed-review item (Phase 4/5 review):** the only finding — "blitter ISR
  re-entry if DONE not acked before IRET" — is standard level-IRQ semantics, not
  an RTL defect (a hardware auto-clear would break poll-mode + STATUS). Addressed
  by documentation: the ISR-ack contract is now explicit in `MMIO_MAP.md` and the
  blitter module header, with a correct ISR example (ack first, then IRET).
- **Software:** the C driver lives with the external Zephyr/LVGL/VNC consumers
  (separate repo); the `MMIO_MAP.md` C sketches are the in-repo reference. This
  repo's own software tree is the Rust `klausscc` assembler (`.kbt` programs).
- **Timing (R1) — analysis (Vivado run is the user's step):** the new long paths
  are (a) the blend datapath and (b) the 128-bit arbiter mux.
  - Arbiter mux: a registered-select 2:1 128-bit mux feeding `ddr2_control` —
    trivial, not a concern.
  - Blend datapath: **Vivado FLAGGED this (WNS -4.037 ns)** — the routed
    critical path was `blitter_dma_i/r_mask_row_base → … → r_wbuf` (the
    MASK_BLEND combinational chain: mask-address add → A8 alpha lookup →
    `blend565` multiplies → part-select write), 13.99 ns / 17 logic levels, with
    routing 56% of the delay (the path spanned the whole module).
    **FIXED** by adding the planned `S_BLEND` pipeline stage: stage 1 (`S_BLEND`)
    latches the blend operands (`r_fg_q`/`r_bg_q`/`r_alpha_q`) after the
    address/mux logic; stage 2 (`S_PLACE`) does the multiply/add and writes
    `r_wbuf`. Blend ops only (+1 cycle/pixel on blend; FILL/COPY unchanged).
    Bit-identical results — all 16 tb_blitter checks pass. Re-run P&R to confirm
    closure. If stage 2 (the blend arithmetic) is still marginal, the next cut is
    to register the multiply products (2-stage blend) or force the multiplies to
    DSP48; expected unnecessary since pipelining also shortens the dominant route.
- **On-board perf (user's step):** 600 KB COPY, confirm `BLIT_CYCLES` ≪ 367 ms
  equivalent and that the CPU runs concurrently during BUSY (read `0xF00D`).
- **Resource fix (Vivado DRC UTLZ-1, LUT-as-DRAM over-util):** the Phase-4
  maintenance walk originally read the cache arrays at a *second* address
  (`r_cache_index` in `MS_READ`, vs `w_cache_index` in `WAIT`). LUT-distributed
  RAM has only one async read port, so Vivado replicated the dirty arrays and
  spilled extra read ports into LUT-DRAM → 19320 cells vs 19000 sites. **Fixed**
  by reading every cache array through a single muxed index
  `w_rd_index = (state==MAINT) ? r_cache_index : w_cache_index` in both the CPU
  and maintenance paths (mutually exclusive states → one read port per array,
  1R1W). No functional change; tb_cache + tb_blitter still pass. The blitter
  module itself adds no LUTRAM (no array regs).

## Register map — blitter `0xF00E0000`

| off | reg | notes |
|-----|-----|-------|
|0x00|CTRL|bit0 START(W1); [3:1] OP; bit4 IRQ_EN|
|0x04|STATUS|bit0 BUSY; bit1 DONE (W1C)|
|0x08|DST_ADDR|0x0C DST_STRIDE|
|0x10|SRC_ADDR|0x14 SRC_STRIDE|
|0x18|WIDTH (px)|0x1C HEIGHT (rows)|
|0x20|COLOR (RGB565)|0x24 ALPHA (0..255)|
|0x28|MASK_ADDR|0x2C MASK_STRIDE|
|0x30|CYCLES (RO)| last-blit cycle count|

OP: 0 FILL, 1 COPY, 2 FILL_BLEND, 3 COPY_BLEND, 4 MASK_BLEND.

## New cache-maintenance regs — `0xF005`
| off | reg | notes |
|-----|-----|-------|
| (new) | FLUSH_GO | W1 → full-cache writeback of dirty lines |
| (new) | INVAL_GO | W1 → full invalidate (flush+inval) |
| (new) | CACHE_MNT_BUSY | RO status bit |

(Exact offsets: pick unused offsets in the existing `0xF005` register file.)

## Verification (testbench `tb_blitter.v`)
- **Unit:** FILL known color; COPY pattern with *differing* src/dst strides;
  blends with alpha=128 and with an A8 mask — read back, compare to a behavioral
  reference model (bit-exact fill/copy; `>>8` blend bit-exact).
- **Edge cases:** non-16B-aligned rect origins/widths (byte-enable masking);
  1×N and N×1 rects; src/dst overlap (document as unsupported or handle).
- **Coherency:** CPU-write → flush → blit → invalidate → CPU-read sees fresh data.
- **Arbiter:** concurrent CPU traffic + blit → CPU not starved, no deadlock,
  correct data both sides.
- **Perf:** cycle count for a 600 KB COPY ≪ 367 ms-equivalent; CPU progresses
  during BUSY.

## Risks
- **R1 — Timing (highest).** Closure is already tight at 100 MHz. The blend
  datapath (8 parallel mul/add lanes) and the arbiter mux are the new long paths.
  Mitigate: register datapath stages; fewer blend lanes if needed; keep the
  arbiter a simple priority mux with registered grant.
- **R2 — Arbiter correctness / CDC.** Second master must honor `ddr2_control`'s
  existing CDC settling gaps; mis-timing corrupts CPU memory. Mitigate: route
  blitter through the *same* `o_ddr_mem_*` protocol the cache already uses; reuse
  its gap logic. Heavy sim before board.
- **R3 — Coherency bugs in software sequencing.** Wrong flush/inval order →
  intermittent stale pixels. Mitigate: encapsulate the sequence in the driver;
  coherency testbench.
- **R4 — Resource/BRAM pressure** from blend pipeline + any line buffers on an
  already-full design.

*(Resolved: `0xF00B` is live SHA-256; blitter uses free block `0xF00E`.)*

## Out of scope (handoff-noted, deferred)
- Range flush/invalidate (`FLUSH_ADDR/LEN`) — phase-6+ optimization over full flush.
- Optional `BLEND565` scalar opcode — separate, complementary, lower priority.
- HW clipping — CPU pre-clips, passes exact rects (handoff explicitly).
```

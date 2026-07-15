# Blitter burst engine — design

> **STATUS (2026-07-14, branch feat/blitter-burst):** stage 1 + word fast
> paths IMPLEMENTED — held-grant chunked streaming (CHUNK=8 transactions per
> grant tenure, req held through chunk releases), write settle gap 7→2, and
> whole-word fast paths for FILL (S_FILLW) and same-offset COPY (S_COPYW).
> ddr2_control is UNTOUCHED — measurement showed the ~35 cyc/transaction was
> almost entirely the blitter's own acquire/gap/release handshake, not the
> controller's IDLE drain (which passes in 1-2 cycles once DV drops promptly).
> tb_blitter revived for the interface era (+ DMA half-swap-aware stub,
> perf cases, same-offset edge test): 19/19 byte-exact.
> Unit-tb (LAT=6 DDR model): FILL 4.75→2.13 cyc/px, COPY 6.38→3.50 cyc/px.
>
> **BOARD-VERIFIED** (WNS +0.042/WHS +0.024; m5e 7/7; wedge recorder zero;
> blit_selftest full sweep PASS, 0 wrong pixels): FILL 640×480
> 1,382,403→576,004 CYC (**2.40×**, 1.88 cyc/px); COPY real 640×480
> 2,706,316→1,894,378 (**1.43×**, 6.17 cyc/px — now read-latency-bound);
> COPY real under CPU DDR traffic 4,545,171→2,242,866 (**2.03×** — the held
> grant also cuts arbitration thrash); skewed dst+2 1.15× (per-pixel path,
> handshake savings only). LVGL benchmark: blit 27.1→23.7 ms/frame, copy
> 28.2→~24.5 ms (the contended number — LVGL's threads run during the blit,
> matching COPY real+ddr). All 6 scenes PASS to the VNC desktop.
>
> **ASYNC FLUSH + CHUNK TUNING (2026-07-15).** The zephyr async flush
> (driver-level: vncd_write starts the blit and returns; fence retires it —
> LVGL 8.3's full-size double-buffer mode only overlaps when display_write
> returns early) collapsed the blocking copy to ~0.3 ms/frame but exposed
> grant-tenure contention: with CHUNK=8 the CPU's cache misses wait out the
> tenure during the (hidden) blit, inflating render by ≈ the blit time. The
> tenure is now MMIO-tunable (`BLIT_CHUNK` 0xF00E_0068, default 8 = sync
> optimum). Board sweep, async LVGL totals ms/frame (fill/gradient/shadow):
> c8 113/70/524, c4 107/64/517, c2 102/58/512, **c1 101/57/510** — winner
> CHUNK=1 (blit stretches 26.7→32.5 ms taking leftover bandwidth, stays
> hidden; residual +9 ms vs solo render is raw DDR bandwidth sharing, not
> tenure). The driver sets CHUNK=1 at init on the async path. Versus the
> original sync per-word baseline: fill 120→101, gradient 77→57 (−26%),
> rounded 165→147. blit_selftest at default 8: CYC identical to the
> pre-register build; m5e 7/7; WNS +0.001.
>
> REMAINING (next increments, in expected-value order): src-read prefetch or
> Opt-2 MIG-native back-to-back streaming in ddr2_control (COPY is now
> demand-read-bound — each src word blocks the pixel engine ~20+ cycles),
> skewed-COPY word path (barrel shift), decoupled write skid buffer. The
> remaining ~+9 ms/frame async render inflation is DDR bandwidth, so Opt-2
> (fewer controller cycles per word) also shrinks it.

**Problem:** the blitter does ~8.8 cyc/pixel (was 12.8 pre-DDR-rework). It is
one-pixel-per-step, and the DDR sub-FSMs **acquire → single 128-bit transaction →
release** per word — so a row of N pixels = ceil(N/8) reads/writes, each paying
acquire + transact + release + gap. Faster DDR cut the per-access latency but it's
still one access at a time. **Target: ~1–2 cyc/px (~3–5 ms vs 27 ms for a 600 KB copy).**

## Current structure (blitter_dma.v)
- Read sub-FSM `S_RD_REQ`(511)→`S_RD_DRIVE`(520)→`S_RD_REL`(547): acquire, 1 read, release.
- Write-flush `S_WF_REQ`(472)→`S_WF_DRIVE`(483)→`S_WF_GAP`(491, **7-cycle** settle)→`S_WF_REL`:
  acquire, 1 write, gap, release. The 7-cycle gap was sized for the OLD async write-DV
  settle — now single-domain (P3/P4), the cache's equivalent gap is already `r_gap_count<=2`;
  the blitter's 7 is **vestigial** and can drop to ~2.
- `S_PIX` re-evaluates fetch needs every pixel; only every 8th crosses a 128-bit word.

## The three layers (heavy lifting is in ddr2_control; arbiter needs nothing)

**(A) Arbiter (mem_read_write.v) — NO CHANGE.** `r_grant_blit` already stays high while
`i_dma_req` is held and only clears on `i_dma_done` (951). The blitter just holds `o_dma_req`
across a whole row and pulses `o_dma_done` **once** at row end. **Bound the CPU stall by
chunking:** release+re-request every **K** bursts (e.g. K=16 ≈ 2 KB) — amortizes 16
acquires/gaps into one while keeping the CPU stall to a few hundred cycles. (`is_miss_path`/
`r_mnt_active` still gate when the grant can be handed over.)

**(B) ddr2_control.v — add a streaming path (the real work). Gate it behind `r_grant_blit`
so the cache's master-A path is byte-for-byte unchanged** (contains the full-CPU-regression risk):
- *Opt 1 (minimal):* a "burst-run" mode — while a `i_burst_more` is held, after WRITE_DONE/
  READ_DONE go WAIT→op again **without** the IDLE DV-low drain, taking a fresh app_addr each pass.
  Saves the IDLE round-trip per access. Keeps the 2-beat WRITE/WRITE_B1/READ_DONE logic intact.
- *Opt 2 (fuller):* drive `app_en` for consecutive incrementing `app_addr` on consecutive ui_clk
  cycles while `app_rdy`, consume `app_rd_data_valid` into a small FIFO — MIG-native back-to-back,
  true ~1–2 cyc/128b. Higher payoff, more rework of the rd_beat / write-beat sequencing.

**(C) blitter_dma.v — per-word sub-FSMs → per-ROW streamers.**
- WRITE: acquire once at row start, drive `o_dma_write_DV`+addr (incr by 16) back-to-back per
  completed accumulator word, `o_dma_done` once at row end; collapse the 7-cycle gap to ~2.
- READ (COPY/blend src+mask): pre-fetch the whole source row into a small line buffer
  (128b × ceil(N/8) BRAM / streaming FIFO) under one grant, then consume per-pixel.
- BLEND: the dst read-modify-write keeps a per-word read→write dependency, so burst gain is
  smaller — keep it word-at-a-time until streaming is proven.

## Sequencing (land lowest-risk biggest-win first)
1. **(A) + (C-write) for FILL** — write-only, no read dependency, the biggest and lowest-risk
   win. Proves the held-grant + chunked-release model + the gap reduction.
2. **(B opt-1) + (C-read prefetch) for COPY.**
3. **BLEND** last, once the streaming controller is proven.

**Coherency unchanged at every step** — the full-cache flush/invalidate around the whole blit
(MMIO 0xF005) is independent of per-access granularity, so no software/coherency rework.

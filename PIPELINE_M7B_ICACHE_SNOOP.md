# M7b — I-cache store-snoop: index-only → tag-checked (LVGL hang root cause)

**Status: DIAGNOSED, fix WIP (not yet correct). Tree reverted to known-good M7a
(`689db0c`).** WIP fix saved as `perf/m7/m7b_tagcheck_wip.patch`.

## Symptom
Zephyr LVGL (`build_lvgl/zephyr/zephyr.elf`, 3 MB image) loads on the board
("Load Complete OK") then produces **no UART output** — appears hung. Regression
suite (hello/bst/expr/test_64bit/queens/crypto/dhrystone) + test_rtos all PASS
on the same M7a bitstream.

## Root cause (hazard-counter-confirmed)
Board test: clear perf counters (`perf/m5a/perf_dump.kla` reads 0xF00D_00B0-E8),
run LVGL 45 s (hangs), dump the surviving counters. Result over 4.58 B cycles:
- **INSTR = 16 M → CPI ≈ 286** (normal ~4). Not wedged — *crawling* ~100× slow.
- **IF_MISS = 2.35 B (51 % of cycles)**, **MEM_WAIT = 2.32 B (51 %)**.

The pipeline spends essentially every cycle on fetch-misses + memory waits.
Cause = the M7a I-cache **store-snoop is index-only** (`pipeline_core.sv`
~L1178, `mem_port_wr → invalidate ic_v{hi,lo}[mem_iaddr[12:4]]`, no tag read).
LVGL writes a 640×480 framebuffer + large `.bss`; those data-store addresses
**index-alias** the 512-line I-cache, so each aliasing store spuriously evicts a
cached **code** line — including the executing loop's own code (a `.bss`-zero /
memcpy loop stores to addresses that alias its own instruction line) → refill
every iteration → the ~51 % IF_MISS thrash. LVGL grinds through early boot so
slowly it never reaches its first `printf` in a human's patience window.

The M7a RESUME note already flagged this: *"the current index-snoop is already
SMC-correct for the regression"* → M7b = tag-checked. The regression suite is
small/compute-bound so it never index-aliases code with data writes; LVGL does.

## Why this is a real bug, not a test artifact
Two false leads were ruled out first:
- **eth_mmio_bridge**: an `eth_probe2.kla` "reproduction" crashed in tb_soc, but
  that was MY test's bug — the `.kla` assembler sets `heap_start = 0x174` (NOT
  8-byte aligned, unlike the ELF `build_ddr_image` path), so the dword-granular
  boot copy dropped the last dword (the `RET`). Not the pipeline.
- **Core can't run LVGL**: false — the standalone `tb_pipeline_isa` runs LVGL and
  prints the banner + render header (299 UART bytes). The core executes it fine;
  the board just crawls.

## The fix (WIP — first attempt broke the IRQ-storm test)
`perf/m7/m7b_tagcheck_wip.patch`: add `ic_tagff[0:511]` — an FF shadow of the
I-cache tags, written on every install alongside `ic_tag`, read combinationally
by the snoop so it invalidates **only** when the store's tag matches the cached
line's tag (`ic_tagff[idx] == mem_iaddr[31:13]`), i.e. only when the store
actually overwrites that cached code line. Bit alignment verified: install tag =
`if_req_dw[28:10]` = byte[31:13]; snoop tag = `mem_iaddr[31:13]`; index
[12:4] both sides.

**Result:** M5a golden-trace (hello/bst/test_64bit) PASS, **M5c SMC PASS**, but
**M5c IRQ-storm (bst @997) REGRESSED** → `InvalidOpcode(0x7b8)` after an IRET
(garbage fetch returning to bst code at 0x7b8), stopping at ~59 k instr vs 85 k.

Interpretation: the tag-check is conceptually correct (a real code-modifying
store has a matching tag, so SMC can't be missed), so the storm crash is most
likely **a latent I-cache DATA-path bug that the index-only over-invalidation
was masking** — the tag-check reduces invalidations and exposes it. That is the
thing to chase next, NOT necessarily a bug in the tag comparison itself.

## Next-session plan
1. Re-apply `perf/m7/m7b_tagcheck_wip.patch`.
2. Reproduce the storm crash deterministically (`run_m5c.sh`, or
   `xsim tpisa ... IMAGE=bst.mem IRQ_PERIOD=997`). Crash = InvalidOpcode at
   0x7b8 after IRET at i≈59535. Instrument the I-cache around the 0x7b8 refill
   after the IRQ return: dump ic_hit / ic_rd_data / the install of 0x7b8's line.
   Hypothesis to test: an install/snoop or fill-capture data race, or a fill
   that captures wrong data when a redirect (IRET) changes miss_dw mid-lookup —
   the comment at the `if_look`/`ic_hit` block (~L1246) already warns about
   stale base+data mixes on redirect.
3. Once storm + SMC + goldens all pass: build (Performance_Explore is set),
   check WNS (the +9.7 Kb FF shadow is cheap; watch the snoop's comb tag-compare
   doesn't hit the store path timing), JTAG-program, re-run LVGL — expect
   IF_MISS to collapse and LVGL to reach its banner. Then full regression 7/7 +
   test_rtos, and re-read the hazard counters to quantify the IF_MISS drop.
4. Also worth doing regardless (M7b scope): a `fence.i`-style MMIO
   invalidate-all for loaders-under-pipeline (LLEXT/netboot deposit code as data
   then jump) — orthogonal to the snoop but the same coherence area.

## Repro assets (in perf/m5a/, gitignored)
- `perf_dump.kla` — dumps 0xF00D counters over UART (survive program loads).
- `perf_clear.kla` — clears counters (PERF_CTRL bit 0).
- Board flow: clear → run LVGL 45 s → `perf_dump` → read CYCLES/INSTR/IF_MISS/…
- `mmio_probe.kla` / `eth_probe*.kla` — MMIO-region probes (eth path is FINE;
  keep in mind the `.kla` heap-align gotcha above if hand-assembling).

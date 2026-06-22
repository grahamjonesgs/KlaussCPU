# DDR2 miss-penalty rework — make ui_clk synchronous, shed the async CDC

**Goal:** cut the ~54-cycle DDR2 miss penalty (ptr_chase 54.1c, mem_stream 65.9c).
~90% of it is overhead, not physical DDR latency (~4–5 cyc). The dominant overhead:
the MIG UI runs at **50 MHz** (4:1 PHY ratio of the 200 MHz mem clock), **asynchronous**
to the 100 MHz CPU — crossed with 2-FF synchronizers + conservative 8-cycle settle gaps.

## Verdict (DS181-grounded, high confidence)
Get **ui_clk = 100 MHz, synchronous with the CPU**, via **2:1 PHY @ the existing 200 MHz**
mem clock (DDR2-400, identical signaling, zero new SI risk). 4:1@400 is **out of spec on
the -1 part** (800 Mb/s > the 667 Mb/s -1 ceiling) and blocked by the Nexys 100 MHz
clock-gen limit — do NOT attempt. 2:1 also has lower controller latency than 4:1.

**Catch:** 2:1 halves the MIG UI data width 128→**64-bit** (APP_DATA_WIDTH = 2·nCK·16,
nCK 4→2), so a 128-bit line = **two 64-bit UI beats** — `ddr2_control` needs a 2-beat
datapath (cache stays 128-bit; ddr2_control assembles).

## The one file to edit (MIG config)
`KlaussCPU.srcs/sources_1/ip/mig_7series_0/mig_a.prj` — the active `XML_INPUT_FILE`.
Change **only** `<PHYRatio>4:1</PHYRatio>` → `2:1`. Keep `<TimePeriod>5000</TimePeriod>`
(200 MHz mem) and `<InputClkFreq>200</InputClkFreq>` (the IDELAYCTRL refclk is hard-tied
to 200 MHz). clk_wiz_0 needs no change. Do NOT touch the inner mig.prj, the stale
mig_b.prj, or the dead mig_7series_0_1/ tree.

## Staged, timing-gated, board-rollback-able plan
Rollback at every board-touching phase: reprogram the known-good `master` bitstream
(always available). Each phase = its own commit; the original 4:1 mig_a.prj is in git.

- **P0 — baseline lock.** Anchor numbers (already have: master b9b4ce6, WNS +0.100,
  ptr_chase CPI 9959, mem_stream 11038; perf is hardware-only). master .bit = rollback.
- **P1 — MIG 2:1 regen TIMING SPIKE (go/no-go, throwaway).** Edit prj → 2:1, regenerate.
  **GATE #1:** does `generate_target` validate 2:1@5000 on -1 (no PHY/MMCM freq/ratio
  error)? **GATE #2:** ui_clk reports 100 MHz; MIG OOC fabric timing closeable. No RTL
  rewrite, no board. Fail → STOP, revert prj, reload master.
- **P2 — ddr2_control 128b→two-64b-beat datapath** (KEEP the async syncs). Isolate the
  risky burst-reassembly change from the clocking change. Verify: functional suite +
  cache-coherency/eviction + blitter-flush on board (a beat-reassembly bug corrupts DRAM
  on eviction). Then reload master.
- **P3 — make i_Clk & ui_clk synchronous** (re-clock the cache FSM / CPU on the MIG
  ui_clk net so the crossing becomes one net). Keep syncs/gaps for now (harmless; lets a
  regression bisect to clocking vs sync-removal). **GATE #3 (headline):** post-route WNS ≥ 0
  with CPU+MIG both on 100 MHz fabric, -1 grade, from WNS +0.100. This is where it lives
  or dies on timing. Fail → revert to 4:1/50 MHz (no synchronous win), design == today.
- **P4 — delete the 3 async syncs + 4 settle gaps** (THE PAYOFF). ddr2_control:94-110 wr/rd
  DV sync; mem_read_write:190-196 ddr_ready sync; remove r_gap_count + collapse the GAP
  states (WRITE/READ_EVICT + MAINT MS_W0/W1). Incremental (syncs first, then one gap pair
  at a time) so a regression bisects. **DO NOT TOUCH COOL_DOWN/PRE_WAIT** — they compensate
  for bus_splitter's *same-domain* registered ready, not the async crossing. Verify: hammer
  cache-coherency + blitter-flush; ptr_chase/mem_stream penalty should drop sharply.

## Top risks
1. **Timing closure (P3)** — MIG controller + CPU both on 100 MHz fabric on the slowest
   grade from +0.100 WNS; 2:1 ~doubles controller fabric activity. Mitigate: P1 spike gates
   it; the r_msg/LED relaxations already reclaimed margin.
2. **MIG rejects 2:1@5000 on -1** — likely accepted (100 MHz controller is mid-range for
   2:1) but P1 generate log is the hard gate.
3. **2-beat datapath correctness (P2)** — beat-reassembly bug corrupts DRAM; isolated from
   clocking + hammered by coherency tests.
4. **Eviction race (P4)** — the gaps make a spurious write-DV re-fire idempotent; only safe
   to remove once truly synchronous (P3 done). Remove incrementally + stress-test.

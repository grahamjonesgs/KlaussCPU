# M10 — D-side load latency (Phase 0 findings, 2026-07-16)

Phase 0 came free: perf_haz already prints the cache miss-stall counter
(0xF005 CNT_STALL_CYC) per kernel. From the M9 capture (perf_haz_m9.raw):
  ptr_chase : 80% D-miss rate, miss-stall 5.06M / 12.06M cycles = 42%, avgpen 31.6c
  mem_stream: 17% miss,      miss-stall 2.56M / 8.32M  = 31%, avgpen 39.0c
  calls_fib : ~0 misses (residual is fetch/hit-path) ; alu/branchy/muldiv ~0

VERDICT: build M10a = critical-word-first + early restart on the D-miss
read path (mem_read_write READ_WAIT/WRITE_FETCH + ddr2_control beat
sequencing): present the requested dword the beat it arrives, finish the
32B line install in the shadow. Expected: ~10-15c off the 31-39c penalty →
ptr_chase ~-15% CPI, mem_stream ~-10%. M10b (hit-path cut) deferred —
Phase 0 shows the pool is miss-dominated. Discipline: golden traces, M5c
storm, m5e 7/7, perf_haz A/B vs haz_real_m9.csv.

# M10a implementation (feat/m10-cwf, 2026-07-16)

Three cooperating pieces:

1. CWF command reorder (ddr2_control): a wide read issues the BL8 burst
   holding the requested dword FIRST (new i_mem_dw_off input; burst_hi
   before burst_lo when dw_off[1]), so the requested dword always arrives
   on beat 0 (odd offsets) or beat 1 (even). The beat->dword map gains an
   r_hi_first leg; the full 32 B line still assembles in o_mem_read_data.
2. Early dword channel (ddr2_control -> mem_read_write): o_mem_rd_dw +
   1-cycle o_mem_dw_ready pulse the cycle after the requested beat lands,
   3+ cycles before o_mem_ready. Even offsets also carry the companion
   (dw_off+1) dword (o_mem_rd_dw_next / o_mem_dw_next_ok) — it rides
   beat 0, before the requested dword itself. next_valid on the miss path
   is therefore ~dw_off[0]: exactly the cases the IFB lookahead and the
   ld32 span fast path can use (pipeline_core's odd-offset fills never
   read m_rdata_next; a spanning ld32 at an odd offset falls back to its
   second-read path, which parks in WAIT until the install completes).
3. Early restart + fetch-first shadow writeback (mem_read_write):
   READ_WAIT/WRITE_FETCH pulse cpu.ready on the early dword (r_miss_served
   suppresses the second present at install); dirty misses now fetch FIRST
   and write the victim back AFTERWARDS in EVICT_SHADOW_GAP/WAIT (replacing
   WRITE_MISS_EVICT/WRITE_EVICT_*/READ_EVICT_* — the old evict-then-fetch
   serialization is gone, ~write-latency+gap cycles off every dirty miss).
   The victim lives only in r_evict_data_hold/r_evict_ddr_addr_r during the
   shadow (its tags are overwritten at install); is_miss_path spans the
   shadow states so the blitter grant — the only other DDR master — cannot
   observe DDR's stale copy of the victim line. CNT_STALL_CYC now counts
   only CPU-visible stall (stops at the early-restart pulse), so avgpen in
   perf_haz A/Bs measures what the CPU actually waits.

BOARD-VERIFIED results (perf/m7/haz_real_m10a.csv vs haz_real_m9.csv,
timing MET WNS +0.032 after one phys_opt AggressiveExplore pass; the -0.016
initial WNS was the pre-existing ifb_base->ifb_dw fetch path, P&R lottery):
  ptr_chase  4.637 -> 4.462 CPI (-3.8%), CPU-visible avgpen 31.6 -> 28.7c
  mem_stream 4.233 -> 4.061 CPI (-4.1%), avgpen 39.0 -> 29.8c (shadow
             writeback pays the old evict-first serialization)
  alu/branchy/calls_fib/muldiv: unchanged (+-0.01%) — no regression anywhere.
  m5e 7/7 UART-identical.
Short of the -15%/-10% projection, and the counters say why: the cache is
BLOCKING. Early restart releases the CPU, but the very next access (ptr_chase's
dependent load, mem_stream's next store) parks in WAIT until the line install
+ shadow writeback finish — the per-miss win is only the CPU's compute overlap
(~3c ptr_chase, ~5c mem_stream). The MIG command->first-beat latency (~20c)
dominates the remaining 28c penalty and CWF cannot touch it. Remaining levers,
in leverage order: hit-under-shadow (serve BRAM hits while the DDR side
finishes install/writeback — the tag/data side is idle in the shadow states),
skip PRE_WAIT/COOL_DOWN when early-served (2c/miss off the parked follow-up),
D-side prefetch (helps mem_stream, defeated by ptr_chase's pointer chains).

Verification: NEW unit gate perf/m5a/run_m10_cache.sh (tb_cache revived:
REAL ddr2_control on a beat-level fake MIG — covers the CWF command
reorder, hi-first beat remap, early channel timing, single-ready-pulse
discipline, shadow-writeback DDR image equality after a 4000-op random
soak + concurrent DMA, grant-rise lockout, maintenance walks). tb_soc's
behavioral DDR model gained the early channel (DW_LAT=4), so M5d boots the
full SoC through early restart; tb_blitter's stub is inert on the early
channel and covers the fallback path. Gates run: tb_cache PASS, M5a
hello/bst/expr/test_64bit PASS, M5c storm PASS, M5d SoC bst PASS,
tb_blitter PASS.

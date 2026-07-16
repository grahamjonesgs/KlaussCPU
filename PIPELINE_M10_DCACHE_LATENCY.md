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

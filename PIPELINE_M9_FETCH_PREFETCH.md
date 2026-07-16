# M9 — next-line fetch prefetch (NLB): alignment-immune sequential fetch

**Status: spec + implementation in progress (feat/m9-fetch-prefetch).**

## 1. Why (measured, 2026-07-16)

Post-M7/M8, IFMISS (fetch-wait cycles) is the largest stall column on the
hazard kernels: 30-55% of cycles (perf/m7/haz_real_m8align.csv). Root cause
is structural, not capacity: the IFB is ONE 16 B line, and every line cross —
**even on an I-cache hit** — serializes a ~2-cycle `if_look` BRAM lookup in
front of dispatch. Corollary (M8 rounds 1-3): ±1 line boundary per hot loop =
±several % CPI, so compiler layout tuning is whack-a-mole (branchy +6.5% with
bit-identical instruction counts; ptr_chase ±8% from alignment padding alone).

## 2. Design — next-line buffer (NLB)

One extra 16 B buffer + a background I-cache lookup:

- `nlb = {data[127:0], vhi, vlo, base[28:1]}` — one line, per-dword valid.
- **Prefetch trigger**: whenever the IFB holds line L and the NLB does not
  hold L+1 (and no demand lookup/fill is active), run the normal 1-cycle
  I-cache BRAM read at line L+1 through the existing read port (the
  `ic_look_idx` mux gains a prefetch leg; demand always wins the mux).
- **Prefetch fills on I-cache HIT only.** A prefetch miss does NOT touch the
  shared memory port — no contention with MEM, no pollution, no new port
  arbitration. Cold/capacity misses behave exactly as today (demand fill).
- **Promote**: when `pc` crosses into the NLB's line, the window rebases from
  the NLB combinationally-adjacent (1-cycle register move, replacing the
  2-cycle lookup); the NLB then prefetches the following line.
- **Coherence**: the M7b tag-checked store snoop must also kill a matching
  NLB line (same one-line rule as the IFB store-match invalidate); redirects
  simply mismatch `nlb.base` and are ignored; IRQ entry/reset clear `nlb_val`.

Sequential code resident in the I-cache then streams with zero line-cross
stalls at ANY alignment. Still paid (out of scope): taken-branch redirect
lookup (~3 cyc; a BTB is M10-shaped), true fills to the unified cache/DDR,
and MEM_WAIT / DATA stalls (other levers).

## 3. Expected effect (first-order, from haz_real_m8align)

Recovering 50-80% of each kernel's IFMISS-minus-redirect pool:
branchy 2.53→~1.8-1.4, muldiv 2.48→~1.9-1.7, alu 2.31→~1.85-1.6,
calls_fib 3.68→~3.1-2.9, ptr_chase 4.94→~4.1-3.6, mem_stream 4.40→~3.7-3.3.
Post-M9, residual IFMISS ≈ redirects + fills — the board A/B itself yields
the sequential/redirect split (a dedicated Phase-0 counter split is optional
and skipped).

Second-order: compiler layout sensitivity disappears by construction (the M8
branchy/ptr_chase regressions dissolve); M8's DATA wins become fully visible.

## 4. Gates (unchanged discipline)

1. Sim: tb_pipeline_isa golden traces bit-identical (M5a set); M5c storm
   (IRQ/SMC) — the snoop/NLB interaction is the risk spot; tb_soc bst golden
   + blitflush_probe.
2. Build: timing met (fetch side is off the critical-path families).
3. Board: m5e 7/7 UART-identical; perf_haz A/B vs haz_real_m8align.csv
   (expect IFMISS to collapse toward brflush×~3; NO regression on any
   kernel); LVGL benchmark sanity.

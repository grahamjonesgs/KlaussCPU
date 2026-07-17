# M9 — next-line fetch prefetch (NLB): alignment-immune sequential fetch

**Status: M9 merged to master (6e476a0). M9b 0-cycle serve BOARD-VERIFIED on
feat/m10-cwf — see §5.**

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

# 5. M9b — 0-cycle NLB serve: the sliding window (2026-07-17)

The M9 promote was a 1-cycle register move at every sequential line cross,
and a slot-1 straddle (op starting in the window's second dword and spilling
into the next line) still took the full ~3c if_look path (odd-read rebase +
NLB top-up). M9b removes both with a SLIDING window:

- **Eager slide**: whenever pc is dispatching out of IFB slot 1 (`in1`) and
  the NLB holds the dword after the window (`ifb_base+2` — its line is
  `want_line` for BOTH base parities, its dword-in-line bit is `ifb_base[0]`),
  shift slot1→slot0 and refill slot1 from the NLB at the same edge. pc then
  never exits the window on a sequential path — the line cross costs 0
  cycles, at any alignment, and the NLB is consumed dword-by-dword (two
  slides per line; `want_line` advances after the second, retriggering the
  prefetch engine with the same lead time the 1-cycle serve had).
- **Straddle slide**: on `!fetch_ok && in1 && w_span` (where `nlb_dw_ok`
  cannot see `ifb_base+2` because miss_dw is still in the window's own
  line), slide instead of starting if_look: at most 1 cycle instead of ~3.
- Register moves only — nothing new lands on the w_op/dispatch cone; the
  slide-enable compares (in1, `nlb_line == want_line`, parity valid bit) are
  register-sourced and gate only the IFB write enables.
- **SMC discipline**: the slide is suppressed on a DRAM-store COMPLETION
  edge (`mem_done_now && mem_is_write && !MMIO`) — the SMC squash indexes
  IFB slots by the current base, and a same-edge slide would move the
  store's dword out from under its invalidate; the slide retries next
  cycle (frequency-irrelevant). Issue-time store races need no gate: the
  issue-edge NLB kill + ic_v* clear stop any re-capture of the stored
  line, and a pre-store copy already slid in is still at base/base+1 (or
  squashes via hit_id/hit_ex) when the completion snoop lands — the same
  window the 1-cycle serve always had. The 1-cycle serve remains for
  redirects landing in the NLB line and partial-fill top-ups.

BOARD-VERIFIED (haz_real_m9slide.csv vs haz_real_m10b.csv, perf_haz.elf,
instr counts identical; WNS +0.012 in-flow; m5e 7/7 UART-identical):

    kernel      CPI before -> after   dCPI      difmiss cycles
    branchy     2.242 -> 2.047   -8.7%      -5.88M (40% of its pool)
    muldiv      2.277 -> 2.011  -11.7%      -0.15M (25% of pool)
    ptr_chase   4.462 -> 4.232   -5.2%      -1.20M
    alu         2.000 -> 1.938   -3.1%      -1.50M (33% of pool)
    mem_stream  3.968 -> 3.895   -1.8%      -0.79M
    calls_fib   3.441 -> 3.382   -1.7%      -3.09M (19% of pool)

Every delta is ifmiss (memwait/data/loaduse unchanged) — the pure designed
effect, −12.6M cycles suite-wide (−5.9% total). Largest single-milestone
win since M6. Residual ifmiss ≈ taken-branch redirect lookups (~3c each:
branchy's 8.87M ≈ its 1.59M-flush × ~3c + cold fills) → the next fetch
lever is a BTB/return-stack, not more sequential machinery.

Sim gates: M5a golden traces bit-identical ×5 (hello/bst/expr/test_64bit/
queens — queens' golden regenerated: its cached .mem/.trace pair had gone
INCOHERENT, old-toolchain image vs new-toolchain trace; run_m5a.sh/
run_m5d_soc.sh mem-out grep also fixed for the new klausscc's stderr
logging), M5c IRQ+SMC storm, M5d SoC bst.

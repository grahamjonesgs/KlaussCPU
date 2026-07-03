# Post-P4.1 performance baseline — 2026-07-03

The reference every recommendation in [PERFORMANCE_REVIEW.md](PERFORMANCE_REVIEW.md)
must be measured against. Captured on-silicon **after** the P4.1 fetch/execute
overlap (and the DDR 2:1 clock rework) landed — [PHASE0_BASELINE.md](PHASE0_BASELINE.md)
is **pre-P4.1** and is kept only for trend. Both CPI (functional) and timing
(Fmax) are recorded here, because roughly half the review's items trade one for
the other.

## Provenance

| Field | Value |
|---|---|
| Date captured | 2026-07-03 |
| Board | Nexys A7-100T on `/dev/ttyUSB1`, 100 MHz |
| Loader | `klausscc --input perf_baseline.elf --serial /dev/ttyUSB1 --monitor` (Linux build) |
| Workload | `baremetal/perf_baseline.elf`, built 2026-07-02 with the in-tree clang (`klausscpu-unknown-elf`), `-O1` |
| Tunables (unchanged) | ALU_ITERS 500000, MEM_WORDS 32768, MEM_REPS 4, CHASE_STEPS 200000, BR_ITERS 500000, FIB_N 28, MD_ITERS 50000 |
| Timing source | `impl_1` routed report (2026-07-02) |
| System state | idle (`int=0% idle=0%` every kernel) |

> **Provenance gap to close:** confirm the exact KlaussCPU RTL git commit the
> loaded bitstream + `impl_1` run were built from, and pin it here. This review
> branch (`claude/cpu-architecture-review-8koeim`) only adds docs, so the RTL is
> current `master` — but the loaded bitstream should be verified against it.

---

## Part A — CPI baseline (6 micro-kernels)

Two consecutive runs; instruction counts and CPI were **bit-identical**, cycle
counts varied by ≤ 55 (of tens of millions) — the known non-atomic counter-read
artifact, negligible. Run 1 is canonical:

```
CSV,alu,355,35500811,8000022,4437,0,0
CSV,mem_stream,166,16597532,1966142,8441,909,1310730000
CSV,ptr_chase,177,17692658,2600021,6804,1434,0
CSV,branchy,687,68686574,14687112,4676,0,16881
CSV,calls_fib,1065,106448020,16969566,6272,0,0
CSV,muldiv,37,3708545,750019,4944,0,0
```
*(CSV = name, ms, cycles, instr, cpi_milli, missrate_bp, taken_bp. The `taken_bp`
field is garbage on mem_stream/branchy — a harness overflow artifact, not a
hardware bug; CPI is the trustworthy number.)*

### Derived

| kernel | **CPI** | cycles | instr | **fast-path %** | fetch % | exec % | miss % | mem stall | notes |
|---|---|---|---|---|---|---|---|---|---|
| alu        | **4.437** | 35.50M  | 8.00M  | 43.8% | 69.0 | 31.0 | 0.00% | — | compute-bound |
| mem_stream | **8.441** | 16.60M  | 1.97M  | 60.0% | 41.9 | 58.1 | 9.09% | 4.67M (28%) | writeback-heavy, avgpen 35.6c |
| ptr_chase  | **6.804** | 17.69M  | 2.60M  | 38.5% | 46.3 | 53.7 | 14.34% | 5.09M (29%) | dependent reads, avgpen 29.6c |
| branchy    | **4.676** | 68.69M  | 14.69M | **14.7%** | **74.8** | 25.2 | 0.00% | — | compare-branch dense |
| calls_fib  | **6.272** | 106.45M | 16.97M | 21.2% | 59.4 | 40.6 | 0.00% | — | call/return heavy |
| muldiv     | **4.944** | 3.71M   | 0.75M  | 46.7% | 63.4 | 24.3 | 0.00% | — | mul 5.00c, div 2.08c (CLZ-skip) |

---

## Part B — Timing baseline (impl_1 routed, 2026-07-02)

Every timing-risky RTL change (review §B6, §C1, §C2, §C4, §E1) must hold
**WNS ≥ +0.077 ns**, or net a wall-clock win at a documented lower Fmax.

| metric | value |
|---|---|
| **WNS (setup)** | **+0.077 ns** (MET; 0 / 67827 failing endpoints) |
| TNS | 0.000 ns |
| Hold (WHS) | +0.023 ns (MET) |
| **Fmax** | **~100.8 MHz** (9.923 ns) |
| LUT | 53.42% (33867 / 63400) |
| FF | 15.12% (19176 / 126800) |
| **BRAM** | **68.89%** (93 / 135) — the tight resource |
| DSP | 6.67% (16 / 240) |

> The specific binding endpoint (the review's §B6 assumes it is still the
> `r_SM → r_led` display-decode path from the 2026-06-19 report) was **not**
> cleanly reconfirmed from this routed report by grep — open `impl_1` in Vivado
> and `report_timing -max_paths 1` to confirm before acting on §B6.

---

## What this baseline confirms about the review

- **Fetch still dominates, post-P4.1.** 59–75% of cycles on compute code are
  fetch (alu 69%, branchy 75%, muldiv 63%, calls_fib 59%). The front-end
  (§C1–§C4) remains the single biggest lever, exactly as the review argues.
- **The fast path is under-firing — 14.7% to 60%.** `branchy` is worst at
  **14.7%** with the highest fetch share (75%). This is direct on-silicon
  evidence for **§C1** (1-cycle ops never arm the fast path) and **§A3/§E1**
  (compare-and-branch), which together target exactly the branchy profile.
- **Memory-bound kernels spend ~28–29% of cycles stalled** (mem_stream 9.1%
  miss, ptr_chase 14.3% miss), validating **§D1/§D4**. Note avgpen is now
  ~30–36c (down from the pre-P4.1 54–66c — the DDR 2:1 rework helped), so the
  §D estimates should be re-derived off ~30c, not ~60c.
- **Divide is niche for this workload** (2.08c avg, CLZ-skip fully effective;
  div only 5.6% of muldiv cycles) — the §B4 worst case (same-magnitude 64-bit)
  is not exercised here, consistent with the review ranking it medium.

## Comparison to pre-P4.1 (2026-06-19) — read with care

Wall-clock cycles dropped on every kernel (~1.25× geomean), but this bundles
P4.1 + the DDR 2:1 rework + LLVM codegen evolution, and **instruction counts
changed** on 4 of 6 kernels (codegen is no longer apples-to-apples with the old
anchors), so it is **not** a pure-CPI delta:

| kernel | cycles 2026-06-19 → now | instr changed? |
|---|---|---|
| alu | 37.50M → 35.50M (−5%) | 6.50M → 8.00M ✗ |
| mem_stream | 22.67M → 16.60M (−27%) | 1.97M → 1.97M ✓ same |
| ptr_chase | 25.51M → 17.69M (−31%) | 2.40M → 2.60M ✗ |
| branchy | 78.50M → 68.69M (−13%) | 12.69M → 14.69M ✗ |
| calls_fib | 125.99M → 106.45M (−16%) | 16.46M → 16.97M ✗ |
| muldiv | 5.06M → 3.71M (−27%) | 0.75M → 0.75M ✓ same |

**Use this capture as the new anchor.** Future **RTL-only** changes keep the
instruction counts fixed, so compare them as pure cycles/CPI against the Part-A
numbers here. Future **codegen** changes will move the instruction counts and
must be compared with that in mind.

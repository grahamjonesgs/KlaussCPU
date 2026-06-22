# Phase 0 — Pipelining baseline capture

Records the **pre-pipelining** reference for [PIPELINE_PLAN.md](PIPELINE_PLAN.md)
Phase 0: the CPI/perf baseline and the timing (WNS/Fmax) baseline that every
later phase must beat (CPI) and defend (timing). Capture on the **current RTL
bitstream**, on an **idle** board, with the `perf_baseline.c` `#define` tunables
and build flags **unchanged** (otherwise the numbers aren't apples-to-apples).

Three things to capture — fill the tables and commit:
1. **Synthetic CPI** — `perf_baseline` (6 micro-kernels), bare-metal. *(Part A)*
2. **Real-workload CPI** — queens via the live `perf` profiler. *(Part B, optional but matches the ~8.8 memory note)*
3. **Timing baseline** — post-route WNS/Fmax + utilization from Vivado. *(Part C)*

> Paths below are on the dev machine (`klausscpu-runtime` tree + Vivado), not the
> Linux analysis sandbox. The runtime Makefile builds with the in-tree clang
> (`$(git toplevel)/build/bin/clang`, target `klausscpu-unknown-elf`).

---

## Provenance (fill in)

| Field | Value |
|---|---|
| Date captured | 2026-06-19 |
| KlaussCPU RTL git commit | `969191b` (verify the loaded bitstream was synthesized from this) |
| klausscpu-runtime git commit | _fill from dev machine_ |
| Bitstream / impl run | impl_1 |
| Clock constraint | 100 MHz (10.0 ns), `sys_clk_pin` |
| Build flags | `-O1` runtime build (Makefile `CFLAGS`) |
| System state | idle (expect `int=0% idle=0%` in all kernels) |

---

## Part A — Synthetic CPI baseline (`perf_baseline`)

### Build + load + run

```bash
# in the klausscpu-runtime tree (where the Makefile lives)
make perf_baseline.elf

# load onto the board with your usual flow, e.g.:
#   bare-metal UART:   klausscc --serial /dev/tty.usbserial-XXXX  ... perf_baseline
#   or netboot/TCP:    klausscc --net-load perf_baseline.elf --ip <board-ip>
# perf_baseline runs all 6 kernels and prints human-readable blocks plus one
# CSV line per kernel to the serial terminal. Capture the terminal output.
```

### Result — paste the 6 `CSV,` lines here (current RTL)

```
CSV,alu,375,37501562,6500025,5769,0,0
CSV,mem_stream,227,22665106,1966143,11527,909,4999
CSV,ptr_chase,255,25507322,2400022,10627,1075,0
CSV,branchy,785,78498935,12687118,6187,0,5627
CSV,calls_fib,1260,125990814,16455338,7656,0,4999
CSV,muldiv,51,5058817,750021,6744,0,0
```

### Derived (CPI = cpi_milli / 1000; busy CPI excludes idle)

| kernel | CPI | fetch% | exec% | div% | miss rate | avg miss pen | notes |
|---|---|---|---|---|---|---|---|
| alu | **5.769** | 61.3 | 38.7 | – | 0.00% | – | compute-bound; exec floor ≈ 2.23 |
| mem_stream | **11.527** | 35.9 | 64.1 | – | 9.09% | 65.9c | writeback-heavy; stall 8.64M (38%) |
| ptr_chase | **10.627** | 37.6 | 62.4 | – | 10.75% | 54.1c | dependent random reads; stall 9.31M (37%) |
| branchy | **6.187** | 60.5 | 39.5 | – | 0.00% | – | taken rate 56.27% |
| calls_fib | **7.656** | 55.5 | 44.5 | – | 0.00% | – | CALL 3.12% / IND(ret) 3.12% / OTH 9.37% |
| muldiv | **6.744** | 53.4 | 37.6 | 4.11 | 0.00% | – | mul 50k @ 5.00c, div 100k @ 2.08c (CLZ-skip) |

**Observations (2026-06-19 capture):**
- Big drop vs the May anchors (alu 8.08→**5.77**, branchy 9.05→**6.19**, muldiv 17.20→**6.74**); `instr` matches per kernel → pure CPI, apples-to-apples. The current RTL is well below the memory note's "queens ~8.8" (queens is call/return/recursion-heavy; these microkernels sit lower).
- **Pipelining lever intact, just from a lower base.** Fetch is still the top bucket on compute code (alu 61%, branchy 60%, calls_fib 55%), so fetch/execute overlap remains the big win. Exec-only floor for alu ≈ CPI 2.23 → ~2.5× still on the table; memory-bound kernels (mem_stream/ptr_chase, ~37% stall) are gated by the single-port miss penalty (Phase 5 territory), not fetch.
- The `(mix sum vs instr)` MISMATCH is the **+29 constant** harness artifact documented in PERF_BASELINE.md (non-atomic counter read), not a hardware bug — benign.
- Re-baselining the plan's trajectory: Phases 1+2 target a further fetch-cycle cut off **this** ~5.8–7.7 compute-CPI, not off 8.8.

### Sanity reference — prior anchors (do NOT overwrite; for trend only)

```
# 2026-05-29 original multicycle baseline
CSV,alu,725,72501653,6500025,11154,0,0           # CPI 11.15
CSV,mem_stream,316,31546085,1966143,16044,500,4999
CSV,ptr_chase,384,38382359,2400022,15992,478,0
CSV,branchy,1506,150620587,12687118,11871,0,5627
CSV,calls_fib,2109,210838408,16455338,12812,0,4999
CSV,muldiv,151,15150709,750021,20200,0,0

# 2026-05-29 after fetch-function optimization (pre-pipeline)
CSV,alu,525,52501613,6500025,8077,0,0            # CPI 8.08
CSV,branchy,1148,114840928,12687118,9051,0,5627  # CPI 9.05
CSV,calls_fib,1723,172271342,16455338,10469,0,4999
CSV,ptr_chase,316,31574145,2400022,13155,783,0
CSV,muldiv,129,12900671,750021,17200,0,0
CSV,mem_stream,271,27081087,1966143,13773,769,4999
```

> **Instruction counts (`instr` column) must match the anchors per kernel** —
> the workloads are deterministic, so if `instr` differs the build changed
> (different tunables/flags) and the comparison is invalid. Only `cycles`/`ms`
> (hence CPI) should move with RTL changes.

---

## Part B — Real-workload CPI (queens, live profiler) — optional

The memory note's "busy CPI ~8.8" was queens.llext under Zephyr via the live
`perf` SSH command (delta-samples the `0xF00D` counters over a window,
non-destructive). To reproduce:

```bash
make queens.llext           # relocatable ELF32 for the Zephyr loader
# load + run over SSH (your normal flow), then in another SSH session:
#   perf 1000               # 1000 ms window while queens runs
```

| workload | busy CPI | fetch cyc/instr | exec cyc/instr | int% | idle% |
|---|---|---|---|---|---|
| queens (live) | | | | | |
| shell idle | | | | | |

---

## Part C — Timing baseline (Vivado)

On the current implemented design (`impl_1`), in the Vivado Tcl console:

```tcl
open_run impl_1
report_timing_summary -delay_type max -max_paths 10 -file phase0_timing.rpt
report_utilization -file phase0_util.rpt
# key numbers:
puts "WNS = [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]"
```

Recorded from `impl_1` post-physopt reports (2026-06-18 run, RTL `969191b`) — this
is the line every later phase must hold ≥:

| metric | value |
|---|---|
| **WNS (setup)** | **+0.070 ns** (MET; 0 / 69660 failing endpoints) |
| TNS | 0.000 ns |
| **Worst-path endpoint** | **`r_SM_reg[19]/C` → `r_led_reg[12]/R`**, 8 levels (LUT4×2 LUT5×1 LUT6×5) |
| Hold WNS (WHS) | +0.022 ns (MET) |
| LUT util | **54.86%** (34783 / 63400) |
| FF util | 15.47% (19611 / 126800) |
| BRAM / DSP | **68.89%** (93/135) / 6.67% (16/240) |
| Fmax (10 ns − WNS) | **~100.7 MHz** (9.930 ns) |

> **Key finding — the worst path is NOT the divide carry chain the plan predicted.**
> It is an **FSM-state-decode path** (`r_SM_reg[19]` → an LED register's async-reset
> pin), only 8 logic levels but binding at +0.070 ns. The 64-bit divide /
> `r_reg_port_b → r_carry_flag` chain currently has *more* slack (effectively
> relaxed / absorbed by physopt). Implications for pipelining:
> 1. The binding path **starts at `r_SM`** (the state register) — exactly the
>    cluster every pipelining phase modifies. The +0.070 ns margin is the real,
>    razor-thin constraint; any added FSM/decode logic threatens it directly.
> 2. **BRAM is 69% used** — relevant to Phase 5 (a split I/D cache adds BRAM).
> 3. The `r_SM → r_led` reset path looks like a benign candidate for a
>    `set_multicycle_path` / `set_false_path` relaxation (LED clear is not
>    timing-critical) — doing so would reclaim margin and expose the next real
>    path. Worth a cheap experiment before/alongside Phase 1.

---

## Exit criteria (Phase 0)

- [x] Part A: 6 `CSV,` lines captured on the current RTL (2026-06-19); `instr` matches anchors. ✓
- [ ] Part B: queens busy CPI captured (optional).
- [x] Part C: WNS/Fmax + worst-path endpoint + utilization recorded (impl_1 post-physopt, 2026-06-18). **Worst path = `r_SM`→LED decode, NOT the divide chain; WNS +0.070 ns.** ✓
- [~] Provenance table filled (date ✓; runtime git commit + bitstream-commit confirmation outstanding).
- [ ] Committed. *(The golden functional trace — emulator + RTL self-trace — is
      tracked separately; see PIPELINE_PLAN.md Phase 0.)*

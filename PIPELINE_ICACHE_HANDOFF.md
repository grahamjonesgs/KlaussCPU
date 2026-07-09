# KlaussCPU — Fetch Decoupling & I-Cache: Design Handoff

**Repo:** KlaussCPU core (Verilog/SystemVerilog, Nexys A7 / Artix-7). **Not** the
LLVM fork — this is RTL + microarchitecture work. Toolchain touches are minimal
and called out in §9.

**One-line goal:** attack the fetch bottleneck (currently ~80% of cycles in the
fetch FSM, CPI ~11) by putting fast, decoupled instruction storage between the
DDR2 memory path and the front end — a prefetch buffer plus a small direct-mapped
I-cache over unified memory ("modified Harvard"). This is the prerequisite that
makes a later 5-stage pipeline actually pay off.

---

## 1. Why this, why now

- **The machine is fetch-bound, not compute-bound.** Baseline: CPI ~11, ~59–75%+
  of cycles sit in the fetch FSM. Instructions are pulled from the Nexys A7's
  **128 MiB DDR2** (the 100T has only ~600 KB BRAM — the 128 MiB space is DDR),
  one word at a time, and every 2–3-word instruction is 2–3 trips down that path.
- **ISA V2 already made the front end pipelineable.** Fixed register fields
  (`rd[11:8]/rs1[7:4]/rs2[3:0]`) and a trivially-decoded length prefix
  (`LEN[31:30]` = 1/2/3 words) mean decode + operand-read are now clean. The
  encoding is no longer the blocker.
- **So the remaining lever is the memory path**, and it is *microarchitectural*,
  not ISA. No encoding change helps here; the fix is a fast instruction store.
- **Pure Harvard-into-BRAM was rejected**: your software stack (Zephyr, wolfSSL,
  lwIP, netboot-loaded flat images, LLEXT programs) does not fit in ~600 KB BRAM,
  and it would break the flat unified address space that the LLEXT / netboot
  loaders depend on. The chosen design keeps unified DDR and caches instructions
  in front of it — full stack still runs, loaders still work (with one flush
  hook, §6).

**Success metric:** measurable drop in CPI and in fetch-stall cycles on the hot
loops of the regression programs (queens, crypto, bst, expr). Target for a first
cut: turn steady-state loop fetch from DDR-latency-bound into ~1 cycle/word
(BRAM-speed) on cache hits. Full pipeline (separate effort) then chases CPI→~1.

---

## 2. Scope

**In scope (this effort):**
- Phase 0: hardware perf counters + baseline capture (you cannot measure the win
  without these).
- Phase 1: prefetch buffer / fetch queue (sequential decoupling; LEN-driven word
  assembly).
- Phase 2: direct-mapped I-cache in BRAM over DDR.
- Phase 3: I-cache invalidate/flush hook + loader integration.

**Explicitly deferred (separate handoffs):**
- The EX/MEM pipeline itself (IF/ID/EX/MEM/WB). This effort makes IF fast; the
  pipeline consumes it later. The I-cache *becomes* the IF stage's backing store.
- D-cache. Load/store still goes straight to DDR for now; add later if data
  bandwidth becomes the new bottleneck after fetch is fixed.
- Flag-model cleanups (CMP-drives-Z unification etc.) — relevant to the pipeline
  effort, not to fetch.

---

## 3. Confirm-on-arrival (unknowns — RTL is not in the LLVM fork)

Verify these against the actual core before committing to numbers:
1. **Current fetch FSM shape:** does it already burst from DDR, or read strictly
   one 32-bit word per DDR transaction? (Determines how big the Phase-1 win is.)
2. **DDR2 controller interface:** native burst length / data width (e.g. does a
   read return 32/64/128 bits per beat?). Cache line size should be a multiple of
   the DDR burst to fill efficiently.
3. **Physical address width:** RAM is `0x0–0x07FF_FFFF` (27-bit). PC is 32-bit
   architecturally but only 27 bits are backed. Tag width derives from this.
4. **Any existing buffering/cache** in the fetch path (assume none).
5. **Fclk and timing headroom** — BRAM read + tag compare + hit mux must close at
   your current clock. If tight, register the hit/miss decision (adds a cycle on
   hit; usually still a huge net win vs DDR).
6. **Fetch/data memory arbitration today:** single shared DDR port? Confirms the
   structural hazard the cache relieves on hits.

---

## 4. Baseline capture (Phase 0 — do this FIRST, before any RTL change)

You need numbers before *and* after, and you need on-chip counters to attribute
them. Two layers:

### 4a. Golden-model instruction counts (cheap, deterministic, do immediately)
Per `reference_emulator_testing`: `klausscc --emulate --input X.elf` prints UART
output + a total instruction count. Capture for each regression `.elf`:
- queens, crypto, bst, expr (and hello as a smoke). These are the A/B workloads.
- Record **instruction count** (this won't change from caching — it's the
  denominator for CPI). Save to `perf/baseline_icount.csv`.

### 4b. On-board cycle + fetch-stall counters (the real baseline)
Add lightweight MMIO performance counters to the core (they are also how you'll
measure every later phase — build them now):

| Counter | Meaning |
|---|---|
| `CYCLES` | free-running cycle count |
| `INSTR_RETIRED` | instructions completed |
| `FETCH_STALL` | cycles the front end waited on memory for an instruction word |
| `ICACHE_HIT` / `ICACHE_MISS` | (wired in Phase 2; read 0 until then) |

Expose as read-only MMIO (suggested block near the existing peripherals, e.g.
`0xF00F_00xx` — pick a free slot, document it). Snapshot at program start/end
(or add a "reset counters" write). Then for each regression program record:
`CYCLES`, `INSTR_RETIRED`, derived `CPI`, and `FETCH_STALL / CYCLES` (the % that
motivates this work). Save to `perf/baseline_board.csv` with the git SHA of the
core.

**Deliverable of Phase 0:** committed `perf/baseline_*.csv` + a short
`perf/METHOD.md` (exact ELFs, loader path, how counters are read) so every later
phase is an apples-to-apples A/B.

---

## 5. Design specification

### 5.1 Hierarchy
```
   ┌────────────┐   words    ┌───────────────┐  line fill  ┌─────────┐
   │ Front end  │◀──────────▶│ Prefetch buf  │◀───────────▶│ I-cache │
   │ (decode)   │  LEN-count │  + fetch ctrl │   on miss   │  (BRAM) │
   └────────────┘            └───────────────┘             └────┬────┘
                                                        miss →  │ (DDR burst)
   Load/store ───────────────────────────────────────────────▶ DDR2 (unified)
```
Unified DDR behind everything (von Neumann / flat 128 MiB preserved). Fetch and
load/store share DDR **only on an I-cache miss**; on hits they don't contend.

### 5.2 Prefetch buffer (Phase 1)
- Small FIFO of 32-bit words (start with 8–16 entries) filled by a fetch
  controller that runs ahead of decode on sequential addresses whenever the
  memory path is idle.
- **LEN-driven assembly:** decode reads `LEN[31:30]` of the head word and pops
  1/2/3 words. The buffer hides variable instruction length from decode entirely
  — this is where V2's length prefix pays off.
- **Branch/redirect:** on a taken branch / `JMPR` / `CALLR` / trap, flush the
  buffer and restart fetch at the new PC. (This is also the natural place to hang
  the pipeline's later redirect logic.)
- Phase-1 alone decouples fetch from decode for straight-line code; the loop win
  comes in Phase 2.

### 5.3 I-cache (Phase 2)
- **Direct-mapped** to start (simplest; no replacement policy, no LRU). Upgrade
  to 2-way only if hit-rate on the regressions demands it.
- **Representative sizing (tune to DDR burst + BRAM budget):**
  - 8 KB data, 32-byte lines (8 × 32-bit words) → 256 lines.
  - Address (27-bit physical): `offset[4:0]` (32B line), `index[12:5]` (256 sets),
    `tag[26:13]` (14 bits) + valid bit.
  - Data ≈ 2 BRAM36 tiles; tag+valid ≈ distributed RAM or a small BRAM. Trivial
    against the 100T's ~135 tiles.
- **Line size = multiple of DDR burst** so a miss issues one efficient burst,
  not N single-word reads. This is the single biggest tuning knob — set it after
  confirming §3.2.
- **Miss FSM:** on miss, stall front end, burst-fill the line from DDR, set
  valid+tag, then serve the requested word (critical-word-first is a later
  optimization — start simple, whole-line-then-serve).
- **Wire `ICACHE_HIT/MISS` counters** so hit-rate is visible from Phase 2 on.

### 5.4 Interaction with the future pipeline
The I-cache is the IF-stage backing store. Keep the redirect/flush interface
(§5.2) clean and register-bounded so the pipeline's branch-resolution logic can
drive it without a redesign. Don't build pipeline logic here — just don't paint
it into a corner.

---

## 6. Coherence — the one thing that touches your loaders

The moment instructions live in a cache separate from where you *wrote* them as
data, **loaded code can be stale.** Your LLEXT loader and netboot loader write a
program into DDR as data and jump to it — the I-cache may still hold old bytes
for those addresses (or the prefetch buffer may hold pre-jump words).

**Required:** an I-cache invalidate/flush mechanism, RISC-V `fence.i`-style:
- **Hardware:** an MMIO "invalidate I-cache" register (invalidate-all is fine to
  start; ranged invalidate is a later optimization). Also flush the prefetch
  buffer on the indirect jump into loaded code.
- **Software (runtime repo, not this repo):** call it in the loaders after
  depositing a program and before transferring control —
  - LLEXT loader (`run` command path)
  - netboot loader (after receiving the flat image, before jump)
  - any self-modifying / JIT path.
- **Toolchain (LLVM fork):** optionally expose a `__builtin_klausscpu_iflush()` /
  inline-asm MMIO write so C loader code is clean. Low priority; a volatile MMIO
  store works without any backend change.

Decide the flush interface (MMIO addr + semantics) up front and document it —
discovering this on-board after a mysterious "loaded program executes garbage"
is the classic failure mode.

---

## 7. Phased plan (each phase is independently testable & shippable)

| Phase | Change | Expected effect | Test / exit criteria | Risk |
|---|---|---|---|---|
| **0** | Perf counters + baseline capture | none (measurement) | `perf/baseline_*.csv` committed; counters read correct on board | low |
| **1** | Prefetch buffer + LEN-driven assembly + redirect/flush | fewer fetch bubbles on straight-line code; decode never stalls mid-instruction | regressions pass bit-identical UART output; FETCH_STALL% drops on sequential code; no CPI regression | med (redirect correctness) |
| **2** | Direct-mapped I-cache over DDR + hit/miss counters | hot loops run at BRAM speed after 1st iter; big CPI drop | queens/crypto CPI ↓, ICACHE_HIT rate high on loops; UART bit-identical | med (miss FSM, timing) |
| **3** | I-cache invalidate MMIO + loader flush calls | correctness for loaded code | LLEXT `run` + netboot load a program and execute correctly *with cache enabled* | med (coherence bugs) |
| **4** *(later, separate)* | 5-stage pipeline consuming the I-cache | CPI→~1 territory | own handoff | high |

**Golden rule for every phase:** UART output of every regression program must be
**bit-identical** to the Phase-0 baseline. Caching/buffering is a performance
transform; any output diff is a correctness bug. The emulator golden model
(§4a) is the reference oracle.

---

## 8. Test & validation methodology

1. **Functional (must never regress):** run hello, queens, crypto, bst, expr;
   diff UART output vs golden model. Add a script `perf/run_regressions.sh` that
   loads each `.bin`, captures UART, and diffs.
2. **Performance (the point):** after each phase, snapshot the counters and
   append to `perf/results_phaseN.csv`; compare CPI and FETCH_STALL% / hit-rate
   vs baseline. A phase that doesn't move its target metric is a signal to
   investigate (wrong line size, buffer too small, miss FSM stalling too long).
3. **Coherence (Phase 3):** a dedicated test — load a program via LLEXT `run`
   twice (or load two different programs to the same address region) and confirm
   the second runs correctly with the cache warm. This is the test that catches a
   missing flush.
4. **Timing:** check Vivado timing closes at target Fclk after Phase 2; if the
   tag-compare→hit-mux path fails, register it (1 extra hit cycle) and re-measure.

---

## 9. What lives where

- **CPU repo (this work):** all RTL — counters, prefetch buffer, I-cache, miss
  FSM, flush MMIO. `perf/` baseline + results + scripts.
- **klausscpu-runtime repo:** loader flush calls (LLEXT `run`, netboot), and the
  MMIO counter-read helpers used by `perf/run_regressions.sh`.
- **LLVM fork (this repo):** nothing required. *Optional* convenience:
  `__builtin_klausscpu_iflush()` or documented inline-asm for the flush MMIO
  store. Do only if loader C wants it — a `volatile` store needs no backend
  change.

---

## 10. First executable steps (in the CPU repo)

1. `git checkout -b feat/fetch-decouple-icache` in the CPU repo.
2. Answer §3 (read the fetch FSM + DDR controller; write findings into this doc's
   §3 as you go).
3. Implement Phase-0 counters (`CYCLES`, `INSTR_RETIRED`, `FETCH_STALL`, plus
   `ICACHE_HIT/MISS` reading 0 for now) as read-only MMIO; document the address.
4. Capture the baseline (§4) → commit `perf/baseline_*.csv` + `perf/METHOD.md`.
5. Build Phase 1 (prefetch buffer); re-measure; confirm bit-identical UART +
   FETCH_STALL% improvement on straight-line code.
6. Only then start Phase 2.

Do **not** skip Phase 0 — without counters you're flying blind on whether Phase
1/2 actually helped, and "it feels faster" is not a baseline.

---

## Appendix A — condensed rationale (for reviewers)

- Fetch-bound + DDR-backed 128 MiB + big software stack + working byte-then-jump
  loaders ⇒ modified Harvard (split at cache level, unified DDR behind) beats
  strict Harvard-to-BRAM (capacity + loader-model breakage) and beats doing
  nothing (statistical contention relief on hits is exactly what hot loops need).
- ISA V2's `LEN[31:30]` prefix and fixed register fields already de-risked the
  decode/operand front end; this work de-risks the *supply* of instructions to
  it. Both are prerequisites for the pipeline; neither is sufficient alone.

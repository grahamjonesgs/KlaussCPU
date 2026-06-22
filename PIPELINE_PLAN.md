# KlaussCPU Pipelining Plan

Plan to reduce busy CPI (currently ~8.8, originally 11–20) by overlapping
instruction execution. This document is the design rationale and a **staged,
timing-gated** implementation plan — *not* a commitment to a textbook 5-stage
pipeline. See [CPU_ARCHITECTURE.md](CPU_ARCHITECTURE.md) for the ISA and
[MMIO_MAP.md](MMIO_MAP.md) §`0xF00D` for the perf counters every phase is
measured against.

---

## 1. Executive summary

**The binding constraint is timing, not CPI.** The design closes 100 MHz only
marginally and is **routing/congestion-bound** (~47% LUT, ~14% FF), so failures
are placement/route pressure, not capacity. The single documented worst path —
`r_reg_port_b → 16×CARRY4 → 7×LUT6 → r_carry_flag`, ~25 logic levels (the
ADD/SUB/CMP ALU path, commented at
[KlaussCPU.v:645](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L645)) — *starts at
the registered read-port output*
([KlaussCPU.v:616-619](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L616)). That is
exactly where a classic pipeline's forwarding/bypass mux has to sit, and there
is no slack to absorb it. A forced Fmax drop to 80–90 MHz can convert a CPI win
into a **wall-clock loss**.

Therefore: **do not build the 5-stage pipeline first.** Escalate in
individually-shippable phases, each gated on `WNS ≥ baseline` and a measured CPI
delta, and gate any datapath rewrite behind a cheap throwaway timing experiment.

**Expected trajectory:**

| Milestone | Busy CPI | Fmax risk |
|---|---|---|
| Today | ~8.8 | — |
| Phases 1+2 (no forwarding) | ~7.0–7.5 | **none** (guaranteed wall-clock win) |
| + Phase 4 (2-stage overlap) | ~6.5–7.5 | medium (gated by Phase 3 spike) |
| 3-/5-stage (after Phase 5 cache split) | ~3–4 | high; contingent |

---

## 2. Why CPI is ~8.8 — attack the right cost

The CPU is one large sequential `always` block walking a 34-bit one-hot state
`r_SM` ([KlaussCPU.v:1353-2532](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L1353));
decode/operand-read/ALU/memory/writeback are all expressed as state transitions,
with per-opcode datapath in the `*.vh` task includes dispatched from
`OPCODE_EXECUTE` via `t_opcode_select`
([opcode_select.vh:58](KlaussCPU.srcs/sources_1/new/opcode_select.vh#L58)).

A straight-line instruction costs roughly:

| Cost bucket | Cyc/instr | States | Pipelineable? |
|---|---|---|---|
| Fetch **sequencing** (IFB hit) | ~2 | `OPCODE_REQUEST`→`FETCH2`→`EXECUTE` | ✅ overlap |
| Fetch **latency** (IFB miss) | ~5/miss | `OPCODE_FETCH` polling `w_mem_ready` | ⚠️ needs 2nd port |
| Execute **occupancy** | 2–3 | `EXECUTE`→`ALU_FINISH`→`WRITEBACK` | ✅ fuse |
| **Data-cache miss** (~54c, ~21% instr) | **~1.9** | miss/refill in `mem_read_write` | ❌ memory floor |
| mul (6c) / div (~10c after CLZ skip) | amortized | `MULTIPLY_*` / `DIVIDE_PREP`+`DIVIDE_STEP` | ❌ stays a stall |

Key state references:
[`OPCODE_REQUEST`:1779](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L1779),
[`OPCODE_FETCH2`:1895](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L1895),
[`WRITEBACK`:2497](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L2497),
[`ALU_FINISH`:2513](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L2513),
[`DIVIDE_STEP`:2457](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L2457).

**The controllable terms are fetch sequencing and execute occupancy.** The
~1.9 cyc/instr data-cache miss stall and the ~5-cycle hit latency are
*memory-system* costs that no pipeline depth removes while fetch and data share
**one cache port** ([mem_read_write.v](KlaussCPU.srcs/sources_1/new/mem_read_write.v)
is single-master, single-transaction, with a COOL_DOWN re-accept gap). That port
is the structural ceiling — addressed only in Phase 5.

---

## 3. Architectures considered

Four candidates were designed and adversarially reviewed against the hard
constraints (one cache port, marginal 100 MHz closure, 25-level carry chain,
precise interrupts, self-modifying-code coherency, variable-length 1/2/3-word
instructions, ~72% taken branches).

| Approach | Designer CPI | Skeptic CPI | Feasibility | Verdict |
|---|---|---|---|---|
| Decoupled prefetch front-end (execute FSM untouched) | 6.5–7.5 | **8.3–8.7** (≈ baseline) | 3/10 | Insufficient alone — taken branches + port-locking misses aren't prefetchable. Keep as Phase 1. |
| 2-stage FD \| EMW overlap | 6.8–7.5 | 7.6–8.4 | 5/10 | The forwarding mux that delivers the win lands on the carry chain. Phase 4, gated. |
| Classic 3-stage F \| DX \| MW | 2.8–3.6 | **3.5–4.8** | 4/10 | Real ~2× win, but mandatory ADD/CMP 2-phase split makes "DX" effectively 2 stages; HIGH Fmax risk. Deferred. |
| Full 5-stage + forwarding + prediction | 2.2–3.0 | 2.6–3.5 | 3/10 | Highest ceiling; needs I/D-cache split that *worsens* the congestion threatening closure. Eventual goal only. |

The deeper pipelines offer a genuine ~2–3.5× CPI win, but every one puts a bypass
mux on the 25-level carry chain in the most congested corner, and their headline
CPI is capped by the memory floor unless the cache port is also split (which
itself fights timing). On a chip where `r_PC → r_mem_addr` closed at only
+0.068 ns, that is a poor risk-adjusted first move.

---

## 4. Recommended plan — staged escalation, timing-gated

Each phase is independently shippable, measurable against the `0xF00D` counters,
and defends **`WNS ≥ Phase-0 baseline` as a hard exit gate**.

### Phase 0 — Regression + perf baseline (no RTL surgery)

**Goal:** a golden architectural-state reference and a captured CPI baseline so
every later phase is provably non-regressing.

- Self-checking simulation harness dumping a **golden trace**: per-retired-instruction
  PC, all 16 regs, SP, the 7 flags, `INT_MASK`, and every memory write, against
  the *current* multicycle RTL.
- Regression corpus covering every hard idiom: `CMP→branch`; call/return chains;
  `RET`/`IRET` indirect; 3-source indexed `LDIDX64R`/`STIDX64R`; unaligned
  `MEMGET32` line-span; mul; div/mod incl. div-by-zero and full-64-bit operands;
  interrupt storm + `WAIT`; and an explicit **SMC/LLEXT store-into-buffered-code-line**
  test.
- Capture `0xF00D` baseline per workload: `PERF_CYCLES`, `PERF_INSTR` (busy CPI),
  `PERF_FETCH_CYCLES`, `PERF_EXEC_CYCLES`, mul/div/int cycles, and the
  branch/branch_taken/call/indirect mix counters.
- Record current post-route **WNS/Fmax** and utilization as the timing baseline.

**Exit:** golden trace reproduces bit-identically across runs; corpus covers all
idioms; perf + WNS baselines committed. **Δ CPI: 0 (reference).**

### Phase 1 — Decoupled fetch front-end (execute FSM UNCHANGED)

**Goal:** convert cross-line sequential IFB-misses into queue hits by prefetching
during the multi-cycle execute/ALU/WB/mul/div dead cycles of prior instructions,
without touching the execute datapath or carry chain.

- Self-contained prefetch FSM in its **own** `always` block driving its **own**
  registers; touches shared signals only via the arbiter and the widened hit mux.
- Generalize the 1-line/2-dword IFB to **3 doublewords first** (conservative),
  serving opcode + var1 + var2 combinationally; go to 4 only if timing holds.
- Port arbiter in front of `r_mem_addr`/`r_mem_read_DV`/`r_mem_write_DV` giving
  **data accesses absolute priority** and respecting the cache COOL_DOWN gap; the
  prefetch fill path is never on a correctness deadline (a late prefetch degrades
  to a demand fetch) — mark it `multicycle_path`/`false_path`.
- Extend the store→IFB invalidate
  ([KlaussCPU.v:1414-1419](KlaussCPU.srcs/sources_1/new/KlaussCPU.v#L1414)) to
  **all** queue slots and squash any in-flight prefetch of a written line (SMC/LLEXT).
- **Pre-register the multi-slot tag compares** so the hit decision is a registered
  mux-select, not a deep combinational path into `r_SM`.

**Risk:** arbiter must be provably collision-free against ~40 scattered DV sites;
widened hit mux + arbiter on `r_mem_addr` land in the congestion-bound corner.
**Δ CPI: −0.3 to −1.0 cyc/instr** (bounded by 72–74% taken + ~21% port-locking
misses). **Exit:** golden trace bit-identical (incl. SMC + IRQ storm); WNS ≥
baseline; `PERF_FETCH_CYCLES` down. If the hit mux fails timing, fall back to
3 slots or a pre-registered hit.

### Phase 2 — Execute-occupancy fusion (no overlap, no forwarding)

**Goal:** cut FSM exec-cycle occupancy by fusing terminal retire states into
dispatch — attacking `PERF_EXEC_CYCLES`, which Phase 1 does not touch — while
staying in the in-order single-retire model so precise interrupts stay free.

- Fuse the `WRITEBACK` write and the CMP-exit of `ALU_FINISH` into the
  `OPCODE_REQUEST` dispatch for **simple/logic/move/SETR** ops where it does not
  lengthen the carry chain.
- **Keep the EX1/EX2 (`EXECUTE`→`ALU_FINISH`) split strictly intact for
  ADD/SUB/CMP** to preserve `r_reg_port_b → r_carry_flag`.
- Keep `DIVIDE_STEP` and the 65-bit trial-subtract completely isolated.
- Re-validate the commit gate (crash trace ring + `r_instr_count` + `PERF_INSTR`
  at `OPCODE_FETCH2`) fires exactly once per retired instruction.

**Risk:** added logic in the `OPCODE_REQUEST`/decode cluster near the carry chain;
the single write port must not be driven from two states in the same cycle (WAW).
**Δ CPI: −0.5 to −1.0 cyc/instr** on top of Phase 1. **Exit:** golden trace
bit-identical; WNS ≥ baseline; `PERF_EXEC_CYCLES` down; `PERF_INSTR` unchanged.

### Phase 3 — Forwarding-mux TIMING SPIKE (decision gate, throwaway RTL)

**Goal:** decisively measure, *before* any rewrite, whether a read-port forwarding
mux can coexist with the carry chain at 100 MHz. This one cheap experiment decides
whether any true overlap pipeline is viable.

- Insert **only** a 3-input (EX2/MEM/WB) forwarding mux on the inputs to
  `r_reg_port_a`/`r_reg_port_b`, wired to plausible-but-functionally-inert sources
  so synthesis cannot prune it; **no control-flow change**.
- Full synthesis + place-and-route; measure WNS specifically on
  `r_reg_port_b → r_carry_flag` and the divide trial-subtract, plus
  utilization/congestion deltas. Trial `KEEP_HIERARCHY`/floorplanning of the
  decode-ALU cluster.
- **Decide:**
  - **GREEN** (≥ ~0.4 ns slack) → Phase 4 with full forwarding.
  - **AMBER** (marginal) → Phase 4 with simple/logic-only forwarding + accept the
    arith/CMP interlock.
  - **RED** (WNS negative, floorplanning can't recover) → **STOP**, ship Phases 1+2
    (see Fallback).

**Δ CPI: 0 (measurement only).** Output is the go/no-go and the forwarding scope.

### Phase 4 — 2-stage FD \| EMW overlap (CONDITIONAL on Phase 3 ≠ RED)

**Goal:** overlap the ~2 fetch-sequencing cycles of instruction N+1 under N's
execute, keeping registered read ports and the EX1/EX2 ALU split, with forwarding
scoped to what Phase 3 proved safe.

- Replace the `OPCODE_REQUEST`/`FETCH`/`FETCH2` scalar `r_SM` walk with an FD
  valid/IR register + front/back handshake; **reuse** the datapath, ALU tasks,
  mul/div, cache port, and interrupt push verbatim.
- Forwarding only at the Phase-3 scope (simple/logic ops final at end of EX1); for
  ADD/SUB/CMP accept a **1-cycle interlock** rather than forward `r_alu_pipe_*`
  into the carry chain.
- **New SMC edge:** a store in EMW must **squash an already-served N+1 IR**, not
  merely clear the IFB valid bit.
- **Precise interrupts:** take dispatch only at the committed-instruction boundary
  now that FD holds a speculative N+1 (drain/squash younger before the 64-bit
  context push).
- Convert the in-state `w_mem_ready` poll into a stall/hazard signal that freezes
  FD; regression every multicycle stall path (div/mul/miss/3-source/CALL/RET/IRET).

**Risk:** forwarding-mux Fmax (mitigated by the Phase 3 gate + scope); SMC squash
race (silent miscompare); precise-interrupt boundary drain; branch-squash claws
back part of the win (72–74% taken). **Δ CPI: −0.3 to −1.0 cyc/instr** on top of
Phases 1-2; combined trajectory **~6.5–7.5**. **Exit:** golden trace bit-identical
incl. SMC/LLEXT, interrupt-at-boundary, RET target, var1-spill; WNS ≥ baseline (or
an explicitly documented relaxation that still nets a wall-clock win).

### Phase 5 — Split I/D cache or dedicated I-fetch port (prerequisite for any deeper pipeline)

**Goal:** remove the single-port structural floor (F-vs-M contention; ~1.9
cyc/instr miss stall freezing the front end) that caps CPI well above 1 and blocks
any 3-/5-stage pipeline from reaching throughput.

- Evaluate a dedicated read-only I-fetch port / small I-cache vs a full I/D split,
  measuring the BRAM + 2nd-FSM + 2nd-bus congestion cost against the marginal
  closure.
- Re-measure the real instruction mix from the `0xF00D` counters captured in
  Phases 1-4 to size the benefit before committing.
- Only if this closes timing **and** the mix justifies it, scope the deeper
  3-stage (F\|DX\|MW) or 5-stage pipeline as a **separate** follow-on with its own
  Phase-3-style spike.

**Risk:** a 2nd port worsens the exact congestion that threatens closure — a
circular dependency between the CPI fix and the timing problem; may not close at
100 MHz at all. **Δ CPI: −0.4 to −0.6 cyc/instr**; *unlocks* (does not itself
deliver) the deeper-pipeline ~2–3× ceiling. **Exit:** spike shows the 2nd port
closes with positive WNS and the mix shows F/M contention is material; else STOP
at Phase 4.

---

## 5. Risk register

| Risk | Sev | Mitigation |
|---|---|---|
| Forwarding mux on `r_reg_port_b → r_carry_flag` (~25 levels) pushes WNS negative → Fmax 80–90 MHz erases the CPI win in wall-clock | High | Gate ALL overlap behind the Phase 3 spike. Scope forwarding to simple/logic only; keep EX1/EX2 split; accept arith/CMP interlock. **Never** forward into the carry chain or divide subtract. Floorplan/`KEEP_HIERARCHY` the decode-ALU cluster. |
| Port arbiter collides with one of ~40 data-access DV sites or violates COOL_DOWN → corrupts a load/store | High | Data gets absolute priority; prefetch is never on a deadline. Enumerate every DV site, prove mutual exclusion, regress the full memory corpus. |
| SMC/LLEXT silent miscompare: store into served code line doesn't squash a latched IR / in-flight prefetch | High | Extend the `r_mem_write_DV` invalidate to all slots + in-flight prefetch; in Phase 4 squash the IR, not just the valid bit. Directed SMC/LLEXT test must pass every phase. |
| Precise-interrupt violation once a speculative N+1 is in flight | High | Single-point in-order retire; take IRQs only at the commit boundary; drain younger before the context push. Interrupt-storm + `WAIT` test must stay bit-identical. |
| Logic in the FSM/decode cluster tips the routing-bound corner even without touching the worst path (`r_mem_addr` closed at +0.068 ns) | Med | Pre-register multi-slot tag compares. Keep each phase's new logic in its own block/hierarchy. Re-run full P&R; defend WNS ≥ baseline every phase. |
| CPI underdelivers (prefetch attacks only residual ~0.75 acc/instr) | Med | Measure `PERF_FETCH_CYCLES`/`PERF_EXEC_CYCLES` per phase; set a ≥0.3 cyc/instr exit threshold; below it, scope down rather than escalate. |
| Verification surface explosion in Phase 4+ (load-use, flag-forward, mispredict-during-miss, IRQ-during-drain, var1-spill across flush) | Med | Golden-trace bit-identity is the gate; corpus covers every idiom and grows per hazard. No merge without a clean full-corpus diff. |

---

## 6. Success metrics

- Busy CPI (`PERF_CYCLES / PERF_INSTR`) decreases monotonically across shipped
  phases; ~8.8 → ~6.5–7.5 after Phases 1-2-4, each delta attributable to the right
  counter (fetch vs exec).
- `PERF_FETCH_CYCLES` drops after Phase 1; `PERF_EXEC_CYCLES` drops after Phase 2.
- **Post-route WNS ≥ Phase-0 baseline on every shipped phase** (100 MHz held), OR
  a documented, accepted Fmax relaxation whose `CPI × clock period` is still a net
  speedup.
- Golden trace bit-identical to the multicycle reference across the full corpus for
  every shipped phase — zero functional regressions, SMC/LLEXT + interrupt-storm
  explicitly passing.
- `PERF_INSTR` per workload unchanged after Phase 2's commit-gate refactor.
- **Net wall-clock speedup (`cycles × measured clock period`) reported per phase** —
  the true bottom line, since a CPI win at a lower Fmax can be a wall-clock loss.

---

## 7. Fallback

If the **Phase 3** forwarding-mux spike comes back **RED** (WNS negative,
unrecoverable by floorplanning), **STOP escalating and ship Phases 1+2** as the
final result: decoupled prefetch + execute-occupancy fusion deliver a measurable
busy-CPI improvement (~8.8 → ~7.0–7.5) with **zero Fmax loss**, no forwarding
network, and precise interrupts / SMC coherency preserved for free — a guaranteed
wall-clock win with bounded risk.

Do **not** pursue the 2-stage overlap, the 3-/5-stage pipeline, or the I/D-cache
split if they cannot demonstrably hold 100 MHz: on this routing-bound, single-port
design a dropped Fmax converts a CPI improvement into a wall-clock regression. If
even Phase 1's widened hit mux fails timing, trim the IFB to 3 slots (or a
pre-registered hit decision) and ship **Phase 2 alone** (it adds the least logic to
the congested corner) as the minimum guaranteed improvement.

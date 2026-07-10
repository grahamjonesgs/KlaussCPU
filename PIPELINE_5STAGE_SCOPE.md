# KlaussCPU — Pipelined Core: Scoping (Phase 4 datapath)

**Status:** SCOPING (design, not yet RTL). Grounded in the current FSM map
(`master`, post flags + 32 B, WNS +0.080 with 2-input forwarding). Owned by
`PIPELINE_MASTER_PLAN.md` §6.

---

## 0. Bottom line up front

- **The forwarding-timing gate is retired.** P3 spike: full 3-input forward mux
  fails (−0.209, ALU congestion), **2-input EX-only forward closes (+0.080)**. So
  a pipeline with **EX→EX forwarding + load-use interlock** is timing-viable.
- **But forwarding was never the big risk — the *structure* is.** The whole
  datapath is a single `cpu_state_t st` struct updated by whole-struct NBAs
  (`st <= f_x(st)`), one instruction in flight, one-hot 34-state FSM. A true
  5-stage means **decomposing that atomic struct into independent per-stage
  registers and refactoring every `f_*` next-state function into stage-local
  combinational logic**, while preserving precise interrupts, SMC coherency,
  HCF/crash, debug, boot/loader, and every peripheral. That is a **ground-up
  rearchitecture of the 3200-line core** — the single largest, highest-risk item
  in the whole roadmap.
- **The CPI ceiling is memory-floor-capped**, not ~1. One shared cache port +
  ~1.9 cyc/instr miss stall + multi-cycle mul/div/loads cap a first pipeline at
  **~1.5–2 CPI** (vs. today's ~4.4–8.8) — a real ~2–3× wall-clock win, but not
  "CPI→1".
- **Recommendation: stage it.** **Phase A** = *incremental overlap deepening*
  (extend the existing 2-stage machinery to overlap the load / ALU-finish tails)
  — lower risk, no rewrite, ~1–1.5× and it's the guaranteed floor. **Phase B** =
  the full stage-register pipeline, gated on Phase A results + a decision to
  accept the rewrite risk. Do **not** start Phase B without the de-risking spike
  in §7.

---

## 1. Where we start (the current core)

Verified from the FSM map:

- **Monolithic FSM.** `cpu_state_t st` (all of PC/SP/flags/wb/div/alu_pipe/mem_*/…)
  updated atomically by `st <= f_x(st,…)`; one-hot 34-state `e_sm_t`; **exactly
  one instruction in flight**. This is *not* a pipeline — it's a fast multicycle
  sequencer.
- **A 2-stage fetch|execute overlap already exists** (`r_FPC` prefetch pointer +
  one-entry IR latch + fast-path dispatch that skips `OPCODE_FETCH2`). It overlaps
  *fetch* with the current instruction's *execute tail* → CPI ~4.4–8.8. **This is
  the seed the pipeline extends, and the only concurrency that exists today.**
- **Per-op cycle costs** (execute region, deferred writeback folded into next
  `OPCODE_REQUEST`): simple logic 1c; arith+flags 2c (`EXECUTE→ALU_FINISH`); load
  3c+DDR; store 2c+DDR; **MUL 6c fixed** (DSP48 walk); **DIV 2 + (64−clz) ≈ 34c
  typ**; push/pop/call/ret 2–3c+DDR.
- **One shared CPU→cache port** (`cpu_mem`, `membus_if`). Fetch and load/store
  never drive it the same cycle *because the FSM is in one state per cycle* — the
  IFB serves sequential fetches combinationally, and the under-execute prefetch is
  buffer-hit-only and explicitly gated off load/store cycles. **A real pipeline
  removes that mutual exclusion** → IF and MEM want the port simultaneously (§3).
- **Precise model is clean:** single IRQ dispatch point (`OPCODE_REQUEST` only),
  single retire point (`OPCODE_FETCH2` / fast-path), deferred-WB commits before
  the IRQ check. Good news — a well-defined commit boundary to preserve.
- **SMC coherency:** a non-MMIO store poisons matching IFB slots + the IR latch +
  the loop-head slot, in the pre-case block. Must generalize to "squash any
  in-flight fetched instruction that the store hits" once >1 instr is in flight.

---

## 2. What a true 5-stage requires (the honest scope)

### 2.1 The structural rewrite (the bulk of the work)
Split the one `cpu_state_t st` into **per-stage bundles** with independent
`always_ff` updates:

| Stage | Reads | Produces | ISA-V2 fields used |
|---|---|---|---|
| **IF** | `r_FPC`/IFB | latched instr words (via `LEN`) + next-PC | `LEN[31:30]` |
| **ID** | IF bundle | 3-port RF read, imm select/extend, decode class/op, forward-mux operands | `CLASS/OP/attr`, `rd/rs1/rs2`, `SGN` |
| **EX** | ID bundle | ALU result + flags, branch cond+target, AGEN; issue mul/div to the multicycle unit | class 1/2/4/5/A ALU; branch `COND/REL/RIND`; AGEN `MODE` |
| **MEM** | EX bundle | data load/store on the (shared) cache port | class 6/7 `SIZE/SGN/A` |
| **WB** | MEM bundle | reg write + flags commit; **the single retire point** | `rd`, flags |

The `f_*` next-state functions each take/return the *whole* struct — they must be
**refactored into stage-local combinational logic** (e.g. `f_alu`'s add/sub/flag
compute becomes EX-stage logic; the load handshakes become MEM-stage logic). This
refactor is the largest and riskiest part.

### 2.2 The hard problems and how each is handled
1. **Multi-cycle MUL/DIV** (6c / ~34c) — *cannot* be a single EX stage. Keep them
   as **separate iterative units**; EX issues, then the pipe **stalls (busy
   interlock)** until they retire. Size the interlock to DIV worst case (66c).
2. **Loads (3c+DDR)** — MEM stage drives the cache; on a hit it's ~5c (registered
   cache + bus_splitter), on a miss ~54c. **Load-use interlock** (1 bubble, or
   MEM→EX forward — but P3 scoped MEM→EX *out*, so use the interlock).
3. **Shared cache port** — IF and MEM both want memory. First pipeline: **stall IF
   whenever MEM accesses the port** (structural hazard resolved by priority to
   MEM). This caps throughput on load/store-dense code; the **split I/D cache
   port** (`PIPELINE_MASTER_PLAN` §4.3) removes it and is the prerequisite for
   4–5-stage throughput — a big sub-effort, gated separately.
4. **Precise interrupts** — retire only at WB; on IRQ, **drain** younger stages
   (or checkpoint) and take the trap at the WB boundary. The context-push (PC +
   7-bit flag word + int_mask to the DDR stack) becomes a WB-driven action;
   `IRET` restores at WB. The clean single-retire model today makes this tractable
   but it must be rebuilt for N-in-flight.
5. **SMC coherency** — a store in MEM must squash any *younger* in-flight
   instruction (IF/ID) whose address it hits — a superset of today's IFB/IR-latch
   poison. Needs an address-compare against the IF/ID PCs.
6. **Variable-length fetch** — IF assembles 1/2/3-word instrs via the IFB (already
   works); a mid-instruction I-cache miss stalls IF only.
7. **HCF / crash / debug / boot-loader** — these exceptional FSMs assume one instr
   in flight. On entry, **drain the pipe and fall into a non-pipelined mode**
   (they run rarely and need not be fast). The crash dump must snapshot the
   *retiring* (WB) instruction for a precise fault PC.

### 2.3 Hazard unit (all timing-checked by P3 except where noted)
- **GPR RAW:** EX→EX forward (2-input, **P3 +0.080**); MEM→EX via **load-use
  interlock** (not forwarded — keeps the mux 2-input).
- **Flags:** one forwarded flags register (clean post-unification — EX→EX).
- **MUL/DIV:** busy interlock.
- **Control:** resolve branch in EX, **static predict-not-taken**, flush IF+ID on
  taken (2 bubbles). Loop-back-edge codegen (compiler) makes fall-through common.
- **Structural:** the 3-read RF removes operand contention; IF/MEM port contention
  per (3).

---

## 3. CPI ceiling / expected win (be honest)

- **Today:** ~4.4–8.8 CPI (2-stage overlap).
- **First 5-stage (shared port):** memory-floor-capped. Hit-path ~1 CPI, but +
  ~1.9 cyc/instr d-cache miss stall + IF/MEM port stalls + mul/div/branch bubbles
  ⇒ **~1.5–2 CPI realistic.** Wall-clock ~**2–3×**.
- **With split I/D port + predictor (later):** approaches the ~1.2–1.5 floor.
- **Phase A (incremental overlap, no rewrite):** targets the load / ALU-finish
  tails only ⇒ **~1–1.5×**, guaranteed, low risk. This is the fallback floor.

---

## 4. Recommended staged path

### Phase A — incremental overlap deepening *(low risk, no rewrite)*
Extend the existing `r_FPC`/IR-latch machinery to overlap **more** of the current
instruction's tail with the next instruction's front end:
- **A1: overlap the load/store DDR wait** with the next instruction's fetch +
  operand pre-settle (today the prefetch is gated *off* load/store cycles — lift
  that once the CPU-side ready-ordering guard from the parked B1 work exists).
- **A2: fuse `ALU_FINISH`/`OPCODE_REQUEST`** dispatch into the execute tail
  (the "~3-CPI floor" lever from `cpu-perf-effort`).
Each is a bounded FSM change, individually board-testable, UART-bit-identical.
**Ship these regardless** — they're the guaranteed win and don't bet on the
rewrite.

### Phase B — full stage-register pipeline *(high risk, gated)*
Only after Phase A + the §7 spike + an explicit decision to accept the rewrite.
Sequenced:
1. **Split I/D cache port** (`PIPELINE_MASTER_PLAN` §4.3) — prerequisite for real
   throughput; do it first so MEM never stalls IF.
2. **Interlock-only skeleton** — IF/ID/EX/MEM/WB stage registers, *no forwarding*,
   full interlock stalls. Correctness first; CPI improves modestly. Bit-identical.
3. **Add EX-only forwarding** (P3-verified) + load-use/mul-div interlocks.
4. **Branch resolve + predict-not-taken + flush.**
5. **Precise-interrupt drain/checkpoint + IRET; SMC squash; HCF drain.**
6. **LLVM `MCSchedModel` + hazard recognizer + TTI re-tune** (unroll/peel now pay).

Every phase keeps UART bit-identical vs. the emulator oracle.

---

## 5. Invariants every phase must keep bit-identical
- **Precise interrupts** — single retire; frame = PC + 7-bit flag word + int_mask;
  IRQ-mask zero-window contract.
- **SMC/LLEXT coherency** — store-into-in-flight-instruction squash.
- **Variable-length 1/2/3-word fetch.**
- **HCF/crash precise fault PC**, debug single-step, boot/loader, all MMIO
  peripherals (UART, perf counters, blitter, crypto, eth, LCD, 7-seg).

---

## 6. Risk register
| Risk | Severity | Mitigation |
|---|---|---|
| Whole-struct → stage-register rewrite regresses a corner (SMC, IRQ, HCF) | **high** | interlock-only skeleton first; emulator bit-identical gate every phase; keep `master` FSM as the reference oracle |
| Shared cache port stalls throughput | med | split I/D port first (Phase B.1) |
| Forwarding timing | **retired** | P3 +0.080 (2-input) |
| ALU-cluster congestion (P3) | med | 2-input forward + floorplan/pblock the ALU + phys_opt |
| Multi-cycle mul/div stalls | low | busy interlock (correctness unaffected) |
| Effort/schedule (months) | **high** | Phase A ships the floor independently; Phase B is opt-in |

---

## 7. Next de-risking spike (before any Phase-B rewrite)
A **throwaway 3-stage prototype on ONE op class** to measure the real CPI gain and
validate the squash/hazard model cheaply — e.g. overlap a load's MEM wait with the
next instruction's IF/ID (a `IF|EX|MEM-WB` slice), on a branch of the current FSM,
measured on board. If the measured gain is small (memory-floor dominates) it
argues for stopping at Phase A; if large, it justifies the full Phase-B rewrite.
Same discipline as the BL16 and P3 spikes: **measure the payoff before the big
build.**

---

## 8. Open decisions for the user
1. **Scope:** Phase A (incremental, guaranteed floor) first, then decide on B — or
   commit to the full Phase-B rewrite now (accepting months + high regression
   risk)?
2. **Split I/D port:** do it *before* Phase B (prerequisite for throughput) or
   stall-IF for the first pipeline and add the port later?
3. **The §7 spike:** run the 3-stage-slice measurement spike before committing to
   the rewrite? (Recommended.)

# KlaussCPU — Pipelined Core: Design Handoff

**Repo:** KlaussCPU core (Verilog/SystemVerilog, Nexys A7 / Artix-7). RTL +
microarchitecture, with a real toolchain (LLVM fork) coordination surface — see
§8. This is the **Phase 4** effort referenced by
`PIPELINE_ICACHE_HANDOFF.md`; that fetch/I-cache work is a **prerequisite**.

**One-line goal:** turn the fetch-decoupled KlaussCPU into a classic in-order
pipeline (IF/ID/EX/MEM/WB) driving CPI from ~11 toward ~1. ISA V2 already made
the decode/operand front end regular enough to stage cleanly; the I-cache effort
makes instruction supply fast enough that a pipeline pays off. What remains are
(a) a small set of **ISA flag cleanups** (§4) and (b) the pipeline datapath +
hazard logic itself (§5–7).

---

## 1. Relationship to the other handoff

| Doc | Effort | Delivers |
|---|---|---|
| `PIPELINE_ICACHE_HANDOFF.md` | Phases 0–3 | perf counters, prefetch buffer, I-cache, coherence flush — **fast instruction supply** |
| **this doc** | Phase 4 | flag cleanups + IF/ID/EX/MEM/WB pipeline + hazard handling — **overlapped execution** |

Do the I-cache work first. A pipelined EX behind a single-port serial fetch just
relocates the stall; the I-cache *is* this pipeline's IF backing store.

---

## 2. Scope

**In scope:**
- ISA flag-model cleanups (§4) — required before, or jointly with, the datapath.
- 5-stage in-order pipeline datapath (§5).
- Hazard handling: forwarding, interlocks, branch resolution, multi-cycle and
  multi-word handling (§6–7).
- Toolchain coordination for the flag changes + a real scheduling model (§8).

**Deferred / separate:**
- Superscalar / out-of-order (not now — in-order single-issue is the target).
- D-cache (from the I-cache handoff's deferred list; add if data BW becomes the
  new bottleneck after fetch + pipeline).
- Branch prediction beyond static predict-not-taken (a BTB/predictor is a later
  increment once the pipeline is stable).

---

## 3. Confirm-on-arrival (RTL is not in the LLVM fork)

1. **Register file ports:** ISA V2 stores read their data from `rd`, so the RF
   grew a **third read port** (`rd`, `rs1`, `rs2`). Confirm this is a true 3-read
   structure (LUT-RAM or mirrored BRAM) — the pipeline needs all three reads in
   ID, and forwarding assumes it.
2. **Where flags physically live today** and exactly which ops write which flags
   (validate §4's "current model" against the RTL — it's reconstructed from
   `ISA_ENCODING_V2.md` note #8 + `CLAUDE.md` "NEVER MIX").
3. **MUL/DIV/MOD latency** (cycles, iterative vs pipelined unit) — drives the §6
   interlock.
4. **Interrupt entry/exit path** and how `IRET` restores state — the pipeline
   must checkpoint flags on IRQ (ties to §4 item 4). Cross-check the IRQ-mask
   MMIO zero-window contract.
5. **DDR arbitration** between I-side (cache miss) and D-side (load/store miss).

---

## 4. ISA flag cleanups required before pipelining  ⟵ *the part previously deferred*

### 4.1 Why flags are the ISA-level blocker
Flags are a **hidden architectural register**: every flag-writer is a producer,
every conditional branch / `ADC` / `SBC` / rotate-through-carry is a consumer. In
a pipeline they need the same forwarding + hazard interlock as GPRs. The current
model makes that hard because there are **two overlapping flag namespaces**, so
"did this instruction produce the condition my branch needs?" is not a single
lookup.

### 4.2 Current model (ISA V2 "no benefit taken")
- **Arithmetic** (`ADDR/SUBR/ADDC/SUBC/INCR/DECR/…`) writes **Z, S, C, V**.
- **Compare** (`CMPRR/CMPRV`) writes a *separate* set **E (equal), L (less),
  U (ult), S** — and **does not drive Z** (note #8).
- Consequence: equality after arithmetic uses **`JMPZ/JMPNZ`** (reads zero_flag);
  equality after a compare uses **`JMPE/JMPNE`** (reads equal_flag). Two encodings,
  two physical flags, for the same question.
- Signed/unsigned relations (`JMPLT/JMPGE/JMPULT/…`) read `L`/`U` from the compare
  set only.
- **Partial writers:** `SETFR` exposes only `{Z,E,C,V}` in the top nibble;
  rotates by N≠1 update `Z` but **not** carry, while `ROLR1/RORR1/ROLCR/RORCR`
  update `Z,C`.
- "**NEVER MIX** arithmetic vs compare flags" (CLAUDE.md) — the branch's flag
  source depends on the producer's class.

### 4.3 Target model (pipeline-friendly)
**One architectural flags register, one writer discipline, one forwarding path.**

1. **Unify equal/zero — `CMP` is `SUB` without writeback.** Make `CMPRR/CMPRV`
   drive the *same* `Z,S,C,V` as `SUB`. Derive relations at branch-decode the
   standard way (ARM/MIPS-style): `EQ=Z`, `NE=¬Z`, signed `LT = S≠V`,
   `GE = S=V`, `LE = Z∨(S≠V)`, unsigned `ULT = ¬C`, `UGE = C`, etc. The separate
   `E/L/U` namespace goes away.
2. **Alias the duplicate branches.** `JMPE/JMPNE` become aliases of `JMPZ/JMPNZ`
   (one encoding, one flag source). Same for any other equal/zero duplication.
3. **Deterministic flag writers.** Publish a complete table: each instruction
   writes either the **full** `Z,S,C,V` set or **none** — no partial-nibble
   writers. Rotates update carry uniformly. Now the hazard unit answers "does
   producer X supply the flags branch Y needs?" with one bit.
4. **Full flag save/restore.** `SETFR`/`GETF` expose the complete flags register
   so an interrupt taken mid-pipeline can checkpoint and `IRET` restore it.

Minimum-viable subset if you want to stage it: **item 1 + 2** alone collapse the
two namespaces and remove most of the forwarding pain; 3 + 4 are cleanups that
make the hazard table and IRQ path clean.

### 4.4 Cross-repo impact of the flag change (LLVM fork) — must land together
- **`BR_CC` lowering** (`KlaussCPUISelDAGToDAG.cpp` `Select(BR_CC)`): equality
  after `CMP` now lowers to `JMPZ/JMPNZ` (or aliased `JMPE/JMPNE`); relational
  conditions map to the derived tests. One condition→branch table, not two.
- **A3a peephole (`KlaussCPUFlagReuse.cpp`) — direct payoff.** Its *entire* risk
  today is "it changes which HW flag a branch reads (zero_flag vs equal_flag),"
  which is why it's **default-OFF** behind `-klausscpu-arith-flag-reuse`. After
  unification that risk disappears → **promote to `cl::init(true)`**, or delete
  the peephole and lower directly to the fused `arith; JMPZ/NZ` form. Satisfying
  and concrete.
- **Carry-consuming patterns:** with carry uniformly defined, `ADC/SBC` and
  rotate-through-carry isel patterns become safe to add (i128 add/sub chains).
- **Keep encoder/assembler in lockstep** — any retired/aliased branch opcode
  must update `KlaussCPUMCCodeEmitter.cpp` + the AsmParser mnemonic table.
- **Regression:** extend `flag-reuse.ll` and add a `cmp-branch.ll` pinning the
  new condition→branch mapping.

---

## 5. Pipeline structure — ISA V2 mapped to 5 stages

Single-issue, in-order. What each stage does and which fields it consumes:

| Stage | Work | ISA V2 fields consumed |
|---|---|---|
| **IF** | I-cache read (from Phase-2 cache); assemble 1/2/3-word instr; next-PC | `LEN[31:30]` (word count); imm32 @PC+4, imm64 hi @PC+8 |
| **ID** | decode; 3-port RF read; immediate select + extend | `CLASS[29:26]` + per-class OP/attr; `rd[11:8]`, `rs1[7:4]`, `rs2[3:0]`; `SGN` for imm ext |
| **EX** | ALU; flag compute; branch cond + target; address-gen | class 1/2/4/5/A ALU; branch `COND/REL/RIND`; AGEN `MODE` (rs1+imm / rs1+rs2 / [imm32]) |
| **MEM** | data load/store | class 6/7 `SIZE`, `SGN`, alignment `A` |
| **WB** | register write; flags commit | `rd` writeback; flags register |

Notes specific to this ISA:
- **`LEN` makes IF trivial**: word count is a 2-bit combinational read; the
  prefetch buffer supplies words and ID sees one instruction per LEN-group. The
  immediate is part of the instruction and available by ID — no separate fetch.
- **Store data path**: base `rs1` and index `rs2` are needed at EX (AGEN); store
  **data `rd`** is needed only at MEM — so it can be forwarded as late as MEM.
- **Branch target**: `REL` → `PC + imm32`; absolute `JMP` → `imm32`; `JMPR/CALLR`
  → `rs2`. All resolvable in EX.

---

## 6. Hazards & handling

- **Data (RAW) on GPRs:** 3-way forwarding EX→EX, MEM→EX, WB→ID across R0–R15.
  Store-data (`rd`) forwardable through MEM.
- **Flag hazard:** with §4's single flags register, one forwarding path EX→EX
  (and MEM→EX for a mul/div that resolves flags late) feeds the branch condition
  and `ADC/SBC/RCL` carry-in. *This is the concrete reason §4 must precede §6* —
  two flag namespaces would double this network and make the producer lookup
  class-dependent.
- **Load-use:** load data ready end of MEM; a dependent op in EX the next cycle
  needs a 1-cycle interlock (or MEM→EX forward with a bubble). Provide the
  interlock; let the compiler scheduler (§8) minimize how often it fires.
- **Control:** resolve in EX; **static predict-not-taken**, flush IF+ID on taken
  (2 bubbles). This makes fall-through-is-common-case codegen valuable — connect
  to the loop back-edge layout item in the perf baseline. A predictor is a later
  increment.
- **Multi-cycle MUL/DIV/MOD:** iterative unit with a busy interlock; stall issue
  of a consumer until the result commits. DIV is the worst case — size the
  interlock to its latency (§3.3).
- **Structural:** the 3-read RF (§3.1) removes operand-read contention. I-side
  (fetch) and D-side (MEM) hit separate stores; they contend only when a fetch
  miss and a load/store miss reach DDR together — arbitrate (give MEM priority to
  avoid a fetch starving an in-flight store; document the policy).
- **Interrupts:** on IRQ, drain or checkpoint the pipeline and **save flags**
  (needs §4 item 4); `IRET` restores PC + flags. Respect the IRQ-mask MMIO
  zero-window contract.

---

## 7. Multi-word & multi-cycle specifics
- A 2/3-word instruction is a single pipeline transaction — the prefetch buffer
  delivered the words in IF, ID assembles per `LEN`. No mid-instruction stall
  once the buffer is warm; a buffer-underrun (I-cache miss mid-instruction)
  stalls IF only.
- Immediates never need a separate cycle: `imm32` is the instruction's 2nd word,
  `imm64` its 2nd+3rd. They're register-file-independent and ready at EX.
- `SETR64` (3-word 64-bit load) and `PUSHV64` are the widest — verify the buffer
  depth in the I-cache handoff covers a 3-word instruction without underrun on a
  cache hit.

---

## 8. Toolchain (LLVM fork) coordination

Beyond the flag-lowering changes in §4.4:
- **Add a real `MCSchedModel` + hazard recognizer.** Today the schedule model is
  minimal. A pipelined target wants the scheduler to spread dependent ops
  (load-use, flag-producer→branch, mul/div consumers) so interlocks fire less.
  This is a genuine backend addition, not a tweak.
- **Revisit TTI (`KlaussCPUTargetTransformInfo`).** It currently **disables**
  partial/runtime unrolling and peeling because the core is *fetch-bound*. Once
  pipelined (and I-cached), unrolling/peeling to hide load-use and branch
  latency can become *beneficial*. Re-tune these after the pipeline lands and
  the I-cache makes duplicated body words cheap — don't leave the fetch-bound
  assumptions in place.
- **Sequencing:** the flag cleanup (§4) is a **flag-day-ish** change — RTL
  semantics and backend lowering must ship together, or compares miscompile.
  Gate the backend side behind a subtarget feature during bring-up if you want
  to A/B old vs new silicon.

---

## 9. Phased plan

| Phase | Change | Exit criteria |
|---|---|---|
| **4a** | ISA flag cleanup (§4) in RTL + backend lowering (§4.4), *no pipeline yet* | all regressions bit-identical UART on the still-unpipelined core; A3a promotable to default-on; `cmp-branch.ll` pinned |
| **4b** | Datapath skeleton IF/ID/EX/MEM/WB with **full interlock stalls** (no forwarding) — correctness first | regressions bit-identical; CPI improves modestly; counters show pipeline occupancy |
| **4c** | Add forwarding (GPR 3-way + flags + store-data) | CPI drops sharply; interlock-stall counter falls |
| **4d** | Branch resolution + predict-not-taken + flush | taken-branch bubble count matches model; loops fast |
| **4e** | MUL/DIV interlock + interrupt checkpoint/IRET | div-heavy (crypto) correct; IRQ under load correct |
| **4f** *(later)* | scheduling model + TTI re-tune (§8); optional predictor | compiler spreads deps; measured CPI→~1 territory |

Each phase keeps the **UART-bit-identical** golden rule against the emulator
oracle. Build interlock-only first (4b) so every later phase is an optimization
over a known-correct baseline.

---

## 10. Testing
- **Functional:** same regression set (hello/queens/crypto/bst/expr) diffed vs
  golden model after every phase. Crypto exercises MUL/rotate/carry — the flag
  cleanup's best stressor. Add an IRQ-under-load test for 4e.
- **Performance:** reuse the Phase-0 perf counters; add `STALL_DATA`,
  `STALL_LOAD_USE`, `STALL_MULDIV`, `BRANCH_FLUSH` so each hazard class is
  attributable. Compare CPI vs the I-cache-only baseline (isolate the pipeline's
  contribution from the cache's).
- **Flag correctness (4a):** a targeted `.s`/`.c` suite over every condition code
  after both an arithmetic producer and a `CMP` producer — this is where a
  botched unification hides.

---

## 11. Risks
- **Flag unification is the highest-risk item** (touches RTL + backend together;
  a wrong condition derivation miscompiles silently). Mitigate with the §10
  flag-correctness suite and a subtarget-feature A/B gate.
- **Timing closure** of the forwarding muxes + flag network at target Fclk on
  Artix-7 — register-bound the long paths; measure after 4c.
- **Load-use interlock frequency** if the scheduler (4f) lands late — acceptable,
  just slower; correctness unaffected.
- **Variable-length underrun** on I-cache miss mid-3-word instruction — verify
  buffer depth (cross-ref I-cache handoff §5.2).

---

## 12. What lives where
- **CPU repo:** all RTL (flag semantics, datapath, forwarding, interlocks,
  interrupt checkpoint), the added hazard-class counters, the flag-correctness
  vectors.
- **LLVM fork (this repo):** flag lowering (§4.4), A3a promotion, ADC/SBC +
  carry-rotate patterns, `MCSchedModel` + hazard recognizer, TTI re-tune;
  `cmp-branch.ll` / extended `flag-reuse.ll`.
- **klausscpu-runtime:** re-validate anything that hand-reads flags or assumes
  the old `JMPE`≠`JMPZ` distinction (inline asm / crt / context-switch).

---

## Appendix — the flag cleanup in one paragraph (for reviewers)
Today KlaussCPU has two flag namespaces — arithmetic `Z/S/C/V` and compare
`E/L/U/S` — and `CMP` doesn't drive `Z`, so equality has two branch encodings
(`JMPZ` vs `JMPE`) reading two physical flags. In a pipeline, flags are an
implicit register needing forwarding; two namespaces double that network and make
the branch's flag source depend on the producer's instruction class. The fix is
to make `CMP` a non-writing `SUB` (one `Z/S/C/V` set, relations derived at branch
decode), alias the duplicate branches, make every op write the full set or none,
and expose the whole register via `SETFR`/`GETF`. It must ship with the matching
backend lowering change, and it retroactively de-risks the existing A3a flag-reuse
peephole (promotable from default-off to default-on).

# KlaussCPU — Core Micro-architecture Roadmap (Fetch/Memory · Flags · Pipeline)

**Status:** OWNED, current plan. Reconciled against `master` @ `a19a853` on
2026-07-09.
**Supersedes:** `PIPELINE_ICACHE_HANDOFF.md` (Phases 0–3) and
`PIPELINE_CORE_HANDOFF.md` (Phase 4). Those two remain in-tree as source
material; **this document is the authority** — where they disagree with what's
below, this wins, because they were written against a machine that is ~2 months
behind `master`.

**Repo:** KlaussCPU core (SystemVerilog, Nexys A7 / Artix-7 100T). RTL +
microarchitecture. The flag work (§5) has a cross-repo LLVM coordination surface
(§8); everything else is RTL-only.

---

## 0. TL;DR — what changed after confirm-on-arrival

Both handoffs assume a naïve multicycle core: one word per DDR trip, **no
buffering, no cache, no perf counters, CPI ~11, fetch-bound.** That is **not**
the current machine. Verified against the RTL, `master` already has:

- **Perf counters** (handoff Phase 0) — full Tier 0/1/2 block at `0xF00D_xxxx`.
- **Prefetch/fetch decoupling** (handoff Phase 1) — an IFB *plus* a 2-stage
  fetch|execute overlap with prefetch pointer, IR latch, fast-path dispatch and
  RAW forwarding (`pipeline-p41`, merged).
- **A unified L1 cache** fronting DDR (handoff Phase 2's premise) — and bigger
  lines were **already tried (32 B) and rejected**: the system is
  **penalty-bound, not miss-rate/capacity-bound.**
- **ISA V2 field-driven decode** — `LEN`-driven length, one `f_cond_eval` branch
  mux, 3-read register file.

Current CPI is **~4.4–8.8** across kernels, not ~11. So Phase 0 is done, Phase 1
is done-and-surpassed, and Phase 2's "no cache, make it bigger" instinct is
contradicted by an on-silicon experiment already in the record.

**What is actually left, in priority order:**

1. **Flag-model cleanup (§5)** — ✅ **COMPLETE & FULL-STACK BOARD-VERIFIED.**
   RTL (commit `fbb77d7`, WNS +0.203) + toolchain (LLVM/emulator/assembler, user)
   + regenerated boot ROM (`netboot.mem`, `1832787`) + recompiled ELFs all
   validated together on silicon: every regression PASS, board UART byte-identical
   to the pre-flag-day golden, board == updated emulator, `tb_flags` 256192/0.
   Retired `E/L/U` → one 4-bit `Z/S/C/V` register; `CMP`≡`SUB` flags; conditions
   derived (borrow convention `ULT=C`). **Measured perf win** from the new flag
   codegen: branchy −2.5% cyc, calls_fib −3.3% cyc, mem_stream −5% cyc/−6.7% instr
   (`perf/baseline_flagday.csv`). Deferred **Phase 1b** (retire the now-dead E/L/U
   ISA-word positions + `JMPE`/COND-9 alias; shrink IRQ frame 7→4-bit) — a small
   post-recompile flag-day cleanup, ~zero perf/area, not gating anything.
2. **Memory penalty-reduction (§4)** — the measured highest-value CPI lever on
   memory-bound kernels: MIG BL16 / critical-word-first / next-line prefetch.
   *Not* a bigger cache. Independent of the flag work.
3. **Split I/D cache port (§4)** — removes the fetch↔load/store structural hazard
   on the one shared cache port; a prerequisite for a deep pipeline, not for
   flags.
4. **5-stage pipeline (§6)** — high timing risk; gated by the forward-mux timing
   spike (`PIPELINE_PLAN.md` P3). The 2-stage overlap already banked the
   low-risk CPI; only proceed past the spike.

---

## 1. Verified current baseline (the "confirm-on-arrival" answers)

Both handoffs open with a "confirm against the RTL" section. Done — here are the
answers, with file references, so no later phase re-litigates them.

### 1a. Fetch / supply path (ICACHE handoff §3)
| Question | Answer (verified) |
|---|---|
| Fetch FSM shape | Multicycle FSM, but **fetch-decoupled**: IFB + 2-stage overlap. `r_FPC` prefetch pointer + IR latch feed a fast-path dispatch that skips the `FETCH2` settle. `KlaussCPU.sv:407-454`. |
| Existing buffering/cache | **Yes** — IFB (one 16 B line, 2 doublewords, store-invalidated, coherent) `KlaussCPU.sv:432-454`; unified L1 cache (16 B lines) in front of DDR (`tb_cache`). Handoff's "assume none" is false. |
| Perf counters | **Present** at `0xF00D_xxxx`: cycles, retired, fetch/exec/mul/div/int/idle buckets, mul/div/int op counts, alu/load/store/branch/branch-taken/jump/call, fast-path fires. `KlaussCPU.sv:343-370`. Documented in `MMIO_MAP.md`. |
| DDR / cache arbitration | **One shared cache port on the CPU side.** `mem_read_write.sv` arbitrates master A = cache FSM (priority) vs master B = blitter DMA. Fetch and load/store **both** ride master A — they contend. No I/D split. `mem_read_write.sv:146-197, 918-927`. |
| Physical address width | RAM `0x0–0x07FF_FFFF` (27-bit) backed of a 32-bit architectural PC. |
| Line size vs DDR burst | MIG UI is **BL8 = 128 b/beat**. A 16 B line = one beat. This is why 32 B (2-beat) lines regressed — see §4. |
| Timing headroom | Design closes **barely** at 100 MHz, but the binding paths are **not** the CPU core: crash-dump `r_msg` mux (debug artifact), then genuine SHA-256/AES round datapaths. The CPU core itself sits comfortably (`r_reg_port_b→carry_flag` ~ +0.278 ns). See `klausscpu-timing-critical-path` notes. |

### 1b. Datapath / flags (CORE handoff §3)
| Question | Answer (verified) |
|---|---|
| Register file ports | **3 read ports** (`r_reg_port_a/b/c` = rs1/rs2/rd), each with a RAW-forward mux off the deferred writeback. `KlaussCPU.sv:605-623`. Port C (rd data) was added for V2 stores. |
| Where flags live | **One packed `flags_t` register**, 7 bits: `zero(Z) equal(E) carry(C) overflow(V) sign(S) less(L) ult(U)`. `klauss_pkg.sv:85-93`. |
| Which ops write which flags | See §5.1 — the crux. Arithmetic writes `Z/S/C/V`; **flag-setting CMP writes a *separate* `E/L/U`** set and does *not* drive `Z`. |
| Branch consumer | **Already unified**: one `f_cond_eval` 10:1 mux over the single register. `klauss_pkg.sv:1004-1025`. The "class-dependent branch lookup" the handoff fears is already gone. |
| MUL/DIV/MOD latency | Iterative; DIV worst-case ~9.5–10.5 c after the CLZ-skip `DIVIDE_PREP`. Multicycle interlock already exists in the FSM. |
| IRQ flag save/restore | **Already present** — all 7 flag bits saved/restored as one context field on interrupt entry / `IRET`. `klauss_pkg.sv:864-882`. CORE handoff §4 item 4 is largely done. |

### 1c. Current CPI reference baseline
From the last clean-master on-silicon capture (2026-07-06, WNS +0.077),
`perf_baseline` kernels: **alu 4.437, branchy 4.676, muldiv 5.211, calls_fib
6.394, ptr_chase 7.152, mem_stream 8.812.** ⚠ These were captured with a
specific ELF build. Per the **stale-ELF-baseline trap**, re-capture clean master
with the *current* ELF set before trusting any A/B delta — same instruction
count, different memory layout ⇒ different miss pattern ⇒ different CPI on
memory-sensitive kernels.

---

## 2. Strategic reframes that govern every decision here

These are hard-won, on-silicon findings. They override the handoffs' instincts.

1. **Penalty-bound, not capacity-bound.** Halving the miss *rate* (32 B lines)
   *regressed* perf because per-miss penalty ~doubled on the BL8 MIG path.
   **Do not chase bigger caches/lines on the 2-beat path.** The lever is
   miss-*penalty* reduction: MIG BL16 (256 b/burst → 32 B in one burst),
   critical-word-first, or a prefetch port to hide latency. See §4.
2. **Timing is the binding constraint on the pipeline, not CPI.** The worst CPU
   path `r_reg_port_b → 16×CARRY4 → carry_flag` (~25 levels) starts exactly where
   a classic forwarding/bypass mux must sit. A deep pipeline needs an
   ALU/writeback datapath *restructure*, not a constraint tweak. The 2-stage
   overlap dodged this by forwarding on the read-port *input*, not the carry
   output. See `PIPELINE_PLAN.md`.
3. **Golden rule — UART bit-identical.** Every phase's regression UART output
   must match the emulator golden model (`klausscc --emulate`) byte-for-byte.
   Caching/buffering/pipelining are performance transforms; any output diff is a
   correctness bug.
4. **Flag-day changes ship cross-repo together.** The flag cleanup (§5) is RTL +
   LLVM lowering in lockstep, gated behind a subtarget feature for A/B bring-up.
5. **Do handshake/load-overlap correctness work in iverilog sim, not on 30-min
   silicon builds.** The parked B1 load-writeback fuse crashed twice from
   mis-reasoned cycle timing; the rule now is golden-model self-trace against
   waveforms first.

---

## 3. Two workstreams

The remaining work splits cleanly into two independent tracks. They share the
perf counters and the golden-model oracle, but touch disjoint RTL and can
proceed in parallel.

- **Workstream M — Memory/fetch supply** (reconciled ICACHE handoff): §4.
- **Workstream P — Flags + pipeline** (reconciled CORE handoff): §5 (flags) → §6
  (pipeline).

**Recommended sequencing:** start with **§5 flags** (lowest risk, unblocks LLVM
now, prerequisite-quality for the pipeline) and, in parallel where bandwidth
allows, a memory penalty-reduction spike (§4). Defer the deep pipeline (§6)
behind the timing spike.

---

## 4. Workstream M — memory/fetch supply (reconciled ICACHE handoff)

The handoff's Phases 0/1 are **done**; Phase 2 "bigger direct-mapped I-cache" is
**the wrong lever** per reframe #1. What survives, in value order:

### 4.1 MIG BL16 reconfiguration *(highest-value, structural)*
Reconfigure the MIG UI from BL8 (128 b/burst) to BL16 (256 b/burst). Then a 32 B
line fills in **one** burst — no per-miss penalty doubling — and the previously
rejected 32 B-line win likely flips positive. This is the root-cause fix for the
penalty-bound wall. **Risk:** MIG re-gen + whole-CPU re-validate; verify the UI
data width / clock (the core is already synchronous to `ui_clk` post the 2:1
rework). **Exit:** ptr_chase/mem_stream CPI ↓, timing closes, UART bit-identical.

### 4.2 Critical-word-first *(medium, no MIG change)*
On a miss, return the requested word to the front end **before** the rest of the
line finishes filling. Attacks the fixed per-miss latency directly on the
current BL8 path. **Risk:** miss-FSM restructure + the cache DV/COOL_DOWN
sequencing assumptions. **Exit:** miss penalty (avg ~54 c) drops; hit path
unchanged.

### 4.3 Split I/D cache port *(prerequisite for the deep pipeline)*
Give instruction fetch its **own** I-cache + memory port so it stops contending
with load/store on the single shared master-A port (§1a). This is the handoff's
one genuinely-new architectural idea, and it's `PIPELINE_PLAN.md` P5 — required
before any 3/5-stage pipeline where IF and MEM want memory in the same cycle.
**Relieves contention, not penalty** — so it pairs with (not replaces) 4.1/4.2.
Keep the redirect/flush + coherence-invalidate interface register-bounded so the
pipeline's branch-resolution can drive it. Coherence: the existing IFB
store-invalidate + a `fence.i`-style invalidate-all MMIO must cover the new
I-cache too (LLEXT/netboot loaders deposit code as data then jump — §7 of the
ICACHE handoff still applies to the *new* I-cache).

### 4.4 Concurrent next-line prefetch *(optional, after a port exists)*
A 2nd cache requester that prefetches the next line under bus-idle. Only sensible
once 4.3 provides a non-contending port; otherwise it fights load/store. Prior
analysis deprioritized this while there was one port — 4.3 changes that.

**Coherence note (applies to all of M):** any new instruction storage must honor
the existing invalidate contract — IFB store-invalidate + MMIO invalidate-all —
or loaded/self-modifying code executes stale bytes. This is already solved for
the IFB+unified cache; do not regress it.

---

## 5. Workstream P, part 1 — the flag-model cleanup ⟵ *near-term priority*

This is the "architectural flags" work called out for this effort. It is a clean,
well-scoped, low-timing-risk ISA change that (a) unblocks concrete LLVM wins
*today* and (b) is prerequisite-quality for the pipeline's flag hazard logic.

### 5.1 Current model (verified against RTL — corrects CORE handoff §4.2)
- **One physical `flags_t` register**, 7 bits: `Z E C V S L U`
  (`klauss_pkg.sv:85-93`). Not two separate register files — one struct.
- **One unified branch consumer**: `f_cond_eval` (`klauss_pkg.sv:1004-1025`), a
  single mux: `1=Z 2=C 3=V 4=S 5=LT(less) 6=LE(less|equal) 7=ULT(ult)
  8=ULE(ult|equal) 9=E(equal)`. **The consumer side is already unified** — this
  is the ISA V2 win the handoff didn't know about.
- **Producer duplication remains (the real problem):**
  - Arithmetic (`ADD/SUB/ADC/SBC/INC/DEC/…`) writes **`Z/S/C/V`** (carry+overflow
    via the ALU pipe in `ALU_FINISH`; zero via the deferred `set_zero`).
  - **Flag-setting `CMP` (`ALU_CMP`, Class 3 B=0) writes a separate `E/L/U`
    (+S)** via `alu_pipe_equal/less/ult`, committed in `ALU_FINISH`
    (`KlaussCPU.sv:2944-2946`, `klauss_pkg.sv:296`), and **does not drive `Z`**.
  - Consequence: equality after arithmetic reads `Z` (`JMPZ`); equality after
    `CMP` reads `E` (`JMPE`). Two producers, two bits, one question. The
    "NEVER MIX arith vs compare flags" footgun.
- **Partial writers:** `SETFR` exposes only a top-nibble subset; rotate-by-N≠1
  updates `Z` but not `C` while `ROLR1/RORR1/ROLCR/RORCR` update `Z,C`
  (`f_rot1`). Non-uniform.
- **Already clean:** full 7-bit flag save/restore on IRQ entry / `IRET`
  (`klauss_pkg.sv:864-882`). The `F` bit (`w_opcode[21]`, `KlaussCPU.sv:1256`)
  gates flag-setting on some ops (ARM-S-style).

### 5.2 Target model
**One writer discipline, `Z/S/C/V` only, relations derived at branch-decode.**

1. **`CMP` becomes `SUB`-without-writeback.** `CMPRR/CMPRV` drive the *same*
   `Z/S/C/V` as `SUB`. Retire the `E/L/U` producer bits. `flags_t` shrinks 7 → 4.
2. **Derive relations in `f_cond_eval`**: `EQ=Z`, `NE=¬Z`, signed `LT = S⊕V`,
   `GE = ¬(S⊕V)`, `LE = Z∨(S⊕V)`, `GT = ¬Z∧¬(S⊕V)`.
   **CARRY POLARITY — VERIFIED against the RTL (klauss_pkg.sv:305-316):** this CPU
   uses the **x86 BORROW convention**, not ARM. `SUB` computes
   `{1'b0,a}-{1'b0,b}` so `carry = sum[64] = 1` exactly when `a < b` (unsigned).
   Therefore **`ULT = C`, `UGE = ¬C`, `ULE = C∨Z`, `UGT = ¬C∧¬Z`** (NOT the ARM
   `ULT=¬C`). This matches today's `alu_pipe_ult = (a<b)` bit. Pin it with a test.
3. **`JMPE/JMPNE` alias `JMPZ/JMPNZ`** — one encoding per condition.
4. **Deterministic writers:** every flag-writer writes the **full `Z/S/C/V` set
   or none**. Fix `SETFR` partial-nibble and the rotate carry non-uniformity.
5. **Full save/restore** already exists — just shrink the saved field 7 → 4 bits
   and keep `SETFR`/`GETF` exposing the whole register.

**Honest scoping of the payoff** (given the consumer is already unified): the
CORE handoff's "two namespaces double the pipeline forwarding network" is
*overstated* — the pipeline forwards the one `flags_t` register regardless of
4 vs 7 bits. The **real** wins that remain are:
- **LLVM, now:** after unification a branch after arithmetic and after `CMP` read
  the *same* bits ⇒ the A3a flag-reuse peephole (`KlaussCPUFlagReuse.cpp`) loses
  its entire risk and can be **promoted default-off → default-on** (or the
  `arith; JMPZ/NZ` fusion lowered directly). Concrete, shippable win.
- **`ADC/SBC` + rotate-through-carry isel** become safe once carry is uniformly
  defined (i128 add/sub chains).
- **Clean hazard table** for the pipeline: uniform full-set-or-none writers make
  "does producer X supply the flags branch Y needs?" a single bit.
- **Removes the NEVER-MIX footgun** and shrinks the flags register / context save.

### 5.3 RTL touch points (CPU repo)
- `klauss_pkg.sv`: shrink/redefine `flags_t` (drop `equal/less/ult`); rewrite
  `f_cond_eval` to derive `LT/LE/GT/GE/ULT/ULE` from `S/V/C/Z`; make `ALU_CMP`
  set `Z/S/C/V` like `SUB`; uniform carry in `f_rot1`/rotate paths.
- `KlaussCPU.sv`: retire the `alu_pipe_equal/less/ult` commit
  (`KlaussCPU.sv:2944-2946`) and the Class-3 flag-setting-CMP E/L/U path
  (`~KlaussCPU.sv:2178-2196`); update `SETFR`/`GETF` width; update the IRQ/`IRET`
  saved-flag field width (`klauss_pkg.sv:864-882`).
- Grep the tree for any remaining `flags.equal/less/ult` reader before deleting
  the bits (verified today: only `f_cond_eval` + IRET restore + reset).

### 5.4 Cross-repo (LLVM fork) — must land together (§8)
`BR_CC` lowering, A3a promotion, `ADC/SBC`/carry-rotate patterns, encoder/
assembler alias updates, `cmp-branch.ll` + extended `flag-reuse.ll`. Detail in
§8. **Gate the backend behind a subtarget feature** so old vs new silicon can
A/B during bring-up.

### 5.5 Test plan (flags)
- A targeted `.s`/`.c` suite exercising **every condition code after both an
  arithmetic producer and a `CMP` producer** — this is exactly where a botched
  relation-derivation hides. Crypto (MUL/rotate/carry) is the best stressor.
- UART bit-identical vs golden model across hello/queens/crypto/bst/expr.
- Pin the carry-borrow polarity (`ULT=¬C`) with a dedicated vector.

---

## 6. Workstream P, part 2 — the 5-stage pipeline (reconciled CORE handoff)

**Reality check first:** a partial pipeline **already exists** — the 2-stage
fetch|execute overlap (`pipeline-p41`) banked −10..−31 % CPI at zero Fmax cost by
forwarding on the read-port input. The CORE handoff's "CPI ~11 → ~1" framing and
"do the I-cache first, a pipelined EX behind serial fetch just relocates the
stall" are both stale: fetch is already decoupled, and CPI is already ~4.4–8.8.

**The binding constraint is timing, not CPI** (reframe #2). Per `PIPELINE_PLAN.md`
the deep pipeline is **gated by a throwaway timing spike (P3):** insert only a
3-input forward mux on `r_reg_port_a/b`, run P&R, measure WNS on the carry chain.
GREEN (≥0.4 ns) → full forwarding; AMBER → simple/logic-only; **RED → stop and
ship what exists.** Do not build the 5-stage datapath before this spike.

Keeping the CORE handoff's stage mapping (it's sound), corrected for what's built:

| Stage | Work | Notes for THIS core |
|---|---|---|
| IF | I-cache read; assemble 1/2/3-word instr via `LEN`; next-PC | supply already fast (IFB/overlap); split I-port (§4.3) needed for a non-contending IF |
| ID | decode; 3-port RF read; imm select/extend | 3 read ports exist (`KlaussCPU.sv:605-623`) |
| EX | ALU; **flag compute**; branch cond+target; AGEN | flag hazard is clean **iff §5 lands first** |
| MEM | load/store | the parked **B1 load-writeback fuse** lives here — do its handshake fix in sim (reframe #5) before overlapping loads |
| WB | reg write; flags commit | deferred-WB machinery already exists (`st.wb`) |

Hazards: 3-way GPR forwarding (EX→EX, MEM→EX, WB→ID); single flags-register
forward path (clean post-§5); load-use 1-cycle interlock; branch resolve in EX
with static predict-not-taken + 2-bubble flush; MUL/DIV busy interlock; IRQ
checkpoint (flags save already exists). Add hazard-class counters
(`STALL_DATA/STALL_LOAD_USE/STALL_MULDIV/BRANCH_FLUSH`) so each is attributable.

**Phased (gated):** §5 flags → P3 timing spike → (if not RED) interlock-only
skeleton → forwarding → branch/predict → MUL/DIV interlock + IRQ checkpoint →
LLVM sched-model + TTI re-tune. Each phase UART-bit-identical.

---

## 7. Consolidated phased plan (the order I intend to work)

| # | Phase | Workstream | Timing risk | Depends on | Exit criteria |
|---|---|---|---|---|---|
| **1** | **Flag-model cleanup** (RTL) + LLVM lowering | P | low | — | every CC correct after arith *and* CMP; UART bit-identical; A3a promotable; `cmp-branch.ll` pinned; flags register 7→4 |
| **2** | **Memory penalty spike:** MIG BL16 *or* critical-word-first | M | med | — (parallel with 1) | ptr_chase/mem_stream CPI ↓; timing closes; UART bit-identical |
| **3** | **Split I/D cache port** + coherence-invalidate for the new I-cache | M | med | 2 | fetch/LS no longer contend on hits; loaders correct with cache warm |
| **4** | **P3 forward-mux timing spike** (throwaway) | P | — (measurement) | 1 | WNS on carry chain measured; GREEN/AMBER/RED decision recorded |
| **5** | **5-stage datapath** (interlock-only → forwarding → branch → MUL/DIV+IRQ) | P | high | 1,3,4≠RED | CPI toward ~1; hazard counters attributable; UART bit-identical |
| **6** | **LLVM sched-model + TTI re-tune** (unrolling now beneficial) | P (LLVM) | low | 5 | compiler spreads deps; interlock-stall counter falls |

Phases 1 and 2 are independent and can run in parallel. Everything after is
gated as shown. **Phases 1+2 alone** are a guaranteed, low-risk win (LLVM flag
wins + the real memory lever) with no Fmax exposure — that's the floor.

---

## 8. Cross-repo coordination (LLVM fork) — flags flag-day

The flag cleanup (§5) must ship RTL + backend **together**, gated behind a
subtarget feature for A/B:
- **`BR_CC` lowering** (`KlaussCPUISelDAGToDAG.cpp`): equality → `JMPZ/JMPNZ`
  (aliased `JMPE/JMPNE`); relations → derived tests. One condition→branch table.
- **A3a peephole** (`KlaussCPUFlagReuse.cpp`): promote `cl::init(false)` →
  `true`, or lower `arith; JMPZ/NZ` directly and delete it.
- **Carry patterns:** add `ADC/SBC` + rotate-through-carry isel (i128 chains).
- **Encoder/assembler:** update `KlaussCPUMCCodeEmitter.cpp` + AsmParser mnemonic
  table for any retired/aliased branch opcode.
- **Regression:** add `cmp-branch.ll` (pins the new condition→branch mapping);
  extend `flag-reuse.ll`.
- **Later (Phase 6):** real `MCSchedModel` + hazard recognizer; revisit
  `KlaussCPUTargetTransformInfo` (it disables unroll/peel *because the core was
  fetch-bound* — re-tune once pipelined + I-cached).

**klausscpu-runtime:** re-validate any inline asm / crt / context-switch that
hand-reads flags or assumes `JMPE≠JMPZ`. Loader flush calls already exist for the
IFB/unified cache; extend to the new I-cache (§4.3) if added.

---

## 9. Testing & measurement (all phases)
- **Functional:** `perf/run_regressions.sh`-style diff of hello/queens/crypto/
  bst/expr UART vs `klausscc --emulate` golden model. Bit-identical or it's a bug.
- **Performance:** reuse the `0xF00D_xxxx` counters; add hazard-class counters at
  Phase 5. Always re-capture the clean-master baseline with the **current ELF
  set** before an A/B (stale-ELF trap, reframe wisdom).
- **Timing:** interactive exploration — `open_run impl_1` / `open_checkpoint` +
  `get_timing_paths -to <reg>` to rank binding paths in ~2 min without a re-synth
  per experiment. The CPU core is *not* the current binding path (SHA/AES/crash-
  dump are) — don't sequence CPU work behind an Fmax number that's a debug
  artifact.
- **Build/measure loop:** Vivado via `~/.klausscpu_scratch/build.tcl` (persistent
  — not `/tmp`, which gets wiped); flash `prog.tcl`/`qspi.tcl`; measure
  `klausscc --input X.elf --serial /dev/ttyUSB1 --monitor`. Launch ~30-min builds
  via the background-Bash mechanism with a ~1 hr fallback re-poll.

---

## 10. Open decisions for the user
1. **Confirm the sequencing** — I recommend starting with the **flag cleanup
   (Phase 1)** now, as the lowest-risk, highest-immediate-payoff item, with a
   memory penalty spike (Phase 2) in parallel if you want two tracks. OK?
2. **Memory penalty lever** — Phase 2 as **MIG BL16** (root-cause, bigger
   validate) vs **critical-word-first** (no MIG change, FSM surgery). Preference?
3. **LLVM lockstep** — the flag change touches the LLVM fork. Confirm you want the
   backend changes gated behind a subtarget feature for silicon A/B, and that
   I should treat the fork as a coordinated deliverable (it's a separate repo).
4. **Deep pipeline appetite** — proceed to the P3 timing spike (Phase 4) after
   flags, or park the 5-stage pipeline and bank Phases 1–3 (the low-risk floor)?

---

## Appendix — provenance
This plan reconciles two handoffs against `master` @ `a19a853`:
- `PIPELINE_ICACHE_HANDOFF.md` — Phases 0–3 (fetch/I-cache). Superseded: Phase
  0/1 done; Phase 2 premise disproven; split-port (its one net-new idea) folded
  into §4.3.
- `PIPELINE_CORE_HANDOFF.md` — Phase 4 (flags + pipeline). Flag "current model"
  corrected against RTL in §5.1; save/restore found already done; pipeline
  re-scoped around the existing 2-stage overlap and the timing-spike gate.
Verified RTL anchors: `klauss_pkg.sv:85-93,864-882,1004-1025`;
`KlaussCPU.sv:343-370,407-454,605-623,2178-2196,2944-2946`;
`mem_read_write.sv:146-197,918-927`.

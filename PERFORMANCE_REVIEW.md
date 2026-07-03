# KlaussCPU Performance Review — CPU, ISA, and LLVM-facing recommendations

**Date:** 2026-07-03
**Scope:** performance only — cycles/CPI, Fmax headroom, DDR2 bandwidth, and the
dynamic instruction count / code size of LLVM-compiled programs. Correctness and
style are out of scope.
**Method:** multi-agent review (8 subsystem deep-readers → 6 review dimensions →
independent adversarial verification of each finding against the RTL), plus a
manual trace of the fetch/execute/cache FSMs. Every item below cites current
sources (`KlaussCPU.sv`, `klauss_pkg.sv`, `mem_read_write.sv`, `bus_splitter.sv`,
`ddr2_control.sv`). Verification status per item:

- **[Confirmed]** — verifier reproduced the claim from the RTL, cycle-for-cycle.
- **[Confirmed, corrected]** — mechanism real, but the verifier corrected cycle
  counts / gain estimates / implementation details; the corrected version is
  what appears below.
- **[Unverified]** — reviewer finding that did not get an independent
  verification pass; treat the numbers as estimates.

**Two caveats that apply to every number in this document:**

1. **The CPI baseline is stale.** PHASE0_BASELINE.md was captured 2026-06-19
   (alu 5.77, mem_stream 11.53, ptr_chase 10.63, branchy 6.19, calls_fib 7.66,
   muldiv 6.74; fetch 55–61% of cycles), *before* the P4.1 IR-latch fast path
   landed. Re-capture the six kernels on the current bitstream before and after
   each change; absolute CPI deltas quoted below are bounds, not predictions.
2. **Timing is the binding constraint.** WNS is +0.070 ns at 100 MHz (LUT 54.9%,
   BRAM 68.9%). Every suggestion that adds logic near the FSM/decode cluster is
   explicitly flagged and must be gated on `WNS ≥ baseline`, as PIPELINE_PLAN.md
   already mandates. Backend/ABI changes carry zero Fmax risk and are listed
   first for that reason.

---

## 0. Executive summary — ranked by value ÷ (effort × risk)

> **Update 2026-07-03 — verified against the real LLVM backend** at
> `/media/psf/src/klausscpu-llvm/llvm/lib/Target/KlaussCPU` (this review was
> first written from the in-tree markdown docs, which lag the actual code).
> Several LLVM items are **already implemented**; see the new **§G
> reconciliation** for the file-cited, item-by-item verdict. The struck-through
> rows below are done. The standing LLVM-only wins are **A3, B2, E3, and the
> cost-model half of A4**.

| # | Recommendation | Type | Est. effect | Fmax risk |
|---|---|---|---|---|
| ~~1~~ | ~~Callee-saved register split (§A1)~~ — **already done**: R4–R7 are callee-saved (§G) | — | — | — |
| ~~2~~ | ~~Fold (negative) imm32 offsets into LDIDX/STIDX (§A2)~~ — **already done**: `simm32` + general fold patterns (§G) | — | — | — |
| 3 | Flag reuse: delete redundant CMPs, count-down loops (§A3) — **stands** | LLVM only | ~4–7 cyc per loop iteration in counted loops | none |
| 4 | Retire loads/POP through deferred writeback, not the WRITEBACK state (§B1) | tiny RTL | −1 cyc per load/POP, guaranteed | minimal |
| 5 | Add LDIDX32_S (sign-extending i32 load) (§B2) | tiny RTL + LLVM | −4 B and ~−3 cyc per signed-int load | small (3 decode arms) |
| 6 | Pre-settle successors of 1-cycle ops (§C1) | RTL, decode cluster | −1 cyc per instruction following a 1-cycle op | must be timing-gated |
| 7 | Dedicated redirect-target IFB slot (§C2) | RTL, front-end | −4 cyc per buffered taken-branch target (loop heads) | must be timing-gated |
| 8 | Second BRAM port as a free I-fetch port — de-scoped Phase 5 (§C4) | RTL, structural | unlocks continuous fetch/execute overlap at **zero** extra BRAM | spike required |
| 9 | Memory-system bundle: fetch-first dirty miss, store posting, DDR framing (§D) | RTL, cache/DDR | −10–25 cyc per miss; ~−4 cyc per store | low (off decode cluster) |
| 10 | Fused compare-and-branch (macro-fusion in ALU_FINISH) (§E1) | RTL, ISA | −2 cyc per taken compare-branch pair | must be timing-gated |

Also included: divider re-normalization (§B4), a dead multiply state (§B5), the
display-register timing path that is masking your true Fmax margin (§B6), and a
set of smaller LLVM scheduling/layout policies (§A4–A7).

---

## A. Zero-RTL wins: compiler, ABI, and documentation

These change no hardware, carry no timing risk, and several of them are the
largest absolute wins available.

### A1. Split the register file into caller- and callee-saved [Confirmed, corrected]

CPU_ARCHITECTURE.md §2 declares R0–R14 caller-saved. Under that ABI every value
live across a call must be spilled and reloaded around the call site. Traced
cost on 100% cache hits: PUSH ≈ 7 cycles (2-cycle IFB fetch + 5-cycle store
handshake, `f_push` klauss_pkg.sv:595-613), POP ≈ 8 cycles (5-cycle read
handshake + the WRITEBACK state + fetch, klauss_pkg.sv:962-980) — **~15 cycles
per spilled register per call site**, plus the dirty cache lines it creates.

**Recommendation:** declare R8–R14 callee-saved in the backend's
`CalleeSavedRegs` TableGen def (R15/FP is already callee-saved by convention).
Callers stop dumping long-lived values around every call; leaf and small
functions typically save nothing.

*Corrected expectation:* on `calls_fib` specifically the gain is bounded
(~0.3–1.0 CPI) because fib is self-recursive — callee-saving moves some traffic
into prologues rather than deleting it, and the benchmark runs at 0% miss so
there is no writeback amplification to recover. Real mixed code with leaf
callees gains considerably more. Measure with `PERF_CNT_CALL` and
`CNT_WRITE_HITS`/`CNT_WRITEBACKS` on a mixed workload (queens), not just
calls_fib.

### A2. Fold immediate offsets — including negative ones — into LDIDX/STIDX [Confirmed]

Two related discoveries, one of them a documentation bug worth fixing today:

- **The hardware already supports negative offsets.** Every immediate-indexed
  load/store computes its effective address as a plain 32-bit modular add:
  `r_reg_port_b[31:0] + w_var1[31:0]` (KlaussCPU.sv:2115-2126, all 12 forms;
  klauss_pkg.sv:670-672). `imm32 = 0xFFFFFFF8` correctly yields `base − 8`.
  But CPU_ARCHITECTURE.md §5/§8.3 and LLVM_NEW_INSTRUCTIONS.md:173-174 document
  the offset as `zero_ext(imm32)`, so the backend cannot fold negative frame
  offsets (`R15-8`, `a[i-1]`, `p[-1]`) and instead materializes the address.
- **FrameIndex lowering wastes an instruction per stack access.** The documented
  lowering is `ADDI rd, R15, off` (or the older `SETR`+`ADDR`) followed by a
  plain load/store. ADDI is a 2-word fetch plus **2** execute cycles (it goes
  through ALU_FINISH, klauss_pkg.sv:373-390) — ~5–6 cycles and 8 bytes per
  stack access, and it burns a scratch register. `LDIDX64 rd, R15, off` does
  the same work in one instruction.

**Recommendation:**
1. Fix the docs: the offset is a signed/modular 32-bit displacement.
2. Change the LDIDX/STIDX TableGen operand from `uimm32` to `simm32` and
   implement a `SelectAddr` complex pattern so `(add GPR, simm32)` and
   `(add frameindex, simm32)` fold into the addressing mode;
   `eliminateFrameIndex` should fold the offset into the memory instruction and
   fall back to ADDI only for address-taken cases.
3. Add a 2-line board test that loads via `LDIDX64 rd, base, -8` to pin the
   behaviour before the backend starts relying on it.

Frame accesses are among the most frequent instructions in compiled C; combined
with A1 this attacks the same stack traffic from both sides.

### A3. Exploit the flag model: delete redundant compares [Unverified]

The documented TableGen plan (llvm_backend_plan.md:314-374) emits a fresh
`CMPRR`/`CMPRV` for every `brcond`. Two free wins:

- **Count-down loops need no compare at all.** `DECR Rn` sets `zero_flag`
  (klauss_pkg.sv:387, committed at KlaussCPU.sv:1738-1744), so
  `DECR Rn; JMPNZ top` replaces `DECR; CMPRV Rn,0; JMPNZ` — deleting a 2-word
  instruction and 2 execute cycles (~5–7 cycles) *per loop iteration*.
- **One CMPRR feeds multiple branches.** `CMPRR` sets equal/less/ult in one
  shot; an `if (a<b) … else if (a==b)` cascade needs one compare, not one per
  arm.

**Recommendation:** model the flags as a physical register def in the backend,
implement `analyzeCompare`/`optimizeCompareInstr` to delete compares against
zero after arithmetic on the same value (respecting the JMPZ-vs-JMPE
discipline), CSE identical CMPRRs, and prefer count-down loop rewriting for
counted loops (`HardwareLoops`-style). Backend-only.

### A4. Code size *is* speed here — but with corrected accounting [Confirmed, corrected]

Fetch dominates (55–61% of cycles pre-P4.1), the IFB is one 16-byte line, and
every instruction word costs fetch bandwidth. Policies that pay:

- Peephole `±1` add/sub to the 1-word `INCR`/`DECR` (KlaussCPU.sv:2020-2021)
  instead of 2-word `ADDI`/`ADDV`. Worth ~1–1.5 amortized fetch cycles per
  instance (one ~5-cycle line round trip per 4 words removed).
- Hoist loop-invariant immediates into registers so loop bodies use 1-word RRR
  forms — subject to the 16-GPR budget; a spill costs far more than it saves.
- Bias toward `-Os`-like heuristics (`setJumpIsExpensive`, conservative inline
  thresholds).

*Corrections from verification worth knowing:*
- The "2-word instruction at line offset 12 pays a VAR1_FETCH" penalty is
  roughly **cost-neutral for fall-through code**, because VAR1_FETCH refills
  the IFB with the next code line (KlaussCPU.sv:1948-1957), pre-paying the next
  fetch. It only genuinely wastes ~4–5 cycles when the 2-word instruction is a
  **taken branch/call** — so if you nop-pad anything, pad only hot taken
  branches that would start at `PC % 16 == 12`; general alignment padding is a
  wash or a loss.
- **Unrolling is not automatically harmful.** Hot loops already stream through
  the cache-fetch path at ~0% miss; unrolling that removes per-iteration
  compare/branch overhead saves cycles. Disable it only where it causes spills
  or grows a ≤4-word loop past the single IFB line.

### A5. Leaf-function link-register convention using LEAPC/JMPR [Confirmed, corrected]

CALL pushes the return address through the full ~5-cycle stack-write handshake
and RET reads it back through ~5 more (f_cond_call klauss_pkg.sv:1034-1061,
f_ret klauss_pkg.sv:944-959) — pure protocol on always-L1-hot stack lines. The
instructions to synthesize a link register already exist: `LEAPC` (1 cycle,
KlaussCPU.sv:2047) + `JMP`, returning with `JMPR R14` (1 cycle).

**Recommendation:** add a leaf-call lowering in the backend: materialize the
return address with LEAPC into a designated register, `JMP` to the callee,
return via `JMPR`. Net saving ~5–7 cycles per leaf call/return pair at +8 bytes
per call site.

*Corrected expectation:* the dynamic CALL+RET mix is only ~6% on calls_fib, and
recursive frames aren't leaves — this is a low-single-digit CPI item on that
benchmark. Worth doing because it's cheap and compounds on leaf-heavy real code
(e.g. every soft-float libcall — all FP is libcalls on this target).

### A6. Lay out for fall-through [Unverified]

Taken control transfers cost 2–6+ cycles more than not-taken ones (redirects
drop the IR latch and usually miss the IFB; not-taken branches even pre-settle
the successor's register reads — klauss_pkg.sv:1024-1029). Enable
`MachineBlockPlacement`-style layout, loop rotation so the back edge is the
only taken branch, and static-probability fall-through bias.

### A7. Branchless SELECT where the ISA already pays for it [Unverified]

`ISD::SELECT` is currently expanded to a branch diamond
(llvm_backend_plan.md:252-270) while single-cycle `MINR/MAXR/MINUR/MAXUR` and
the ten boolean `CMPxxR` ops (rd = 0/1, no flags) sit unused. Min/max/clamp
idioms and simple masks (`cond ? a : 0` via `CMPxxR` + `SUBR`/`ANDR` mask
formation) beat a mispredict-free-but-serialized branch diamond on this core.
Pattern-match at least min/max and abs before expanding SELECT.

---

## B. Low-risk RTL micro-fixes (mostly off the critical decode cluster)

### B1. Retire loads and POP through the deferred writeback [Confirmed]

Every load-class op (`MEMGET*`, `MEMREAD*`, all `LDIDX*`, `POP`) ends with
`n.SM = WRITEBACK` (klauss_pkg.sv:651, 700, 770, 976) — a dedicated cycle that
does nothing but `r_register[rd] <= value` (KlaussCPU.sv:2739-2745). ALU, MUL,
and DIV results already skip this via `wb.pending`, committed for free inside
the next OPCODE_REQUEST (KlaussCPU.sv:1738-1744), with the RAW-forward mux on
the read-port inputs (KlaussCPU.sv:589-590) covering the hazard window.
`f_setr64` (klauss_pkg.sv:911-917) already proves the pattern is safe for a
memory-sourced result.

**Fix:** change the ready-cycle exits of `f_mem_load`/`f_ld_idx`/`f_ld_idxreg`/
`f_pop`/`f_memget32` to `n.SM = OPCODE_REQUEST; n.wb.pending = 1'b1`.
**Gain:** −1 cycle per load/POP guaranteed (~12% of an 8-cycle hit load);
directly helps mem_stream/ptr_chase and every function epilogue. Optionally add
the `r_ir_valid` presettle handoff in the same ready cycle (as DIVIDE_STEP does
at KlaussCPU.sv:2730-2734) for a second cycle.

### B2. Add LDIDX32_S — the missing sign-extending i32 load [Confirmed, corrected]

`int` is the dominant C type. Signed-int loads that need the sign in 64-bit
context (compares, promoted arithmetic, division) pay `LDIDX32` + `SEXTW` —
one extra 4-byte instruction ≈ +3 cycles typical. The plumbing already exists:
`f_ld_idx` takes `is_signed` and honours it for i8/i16 (klauss_pkg.sv:693-694);
only the MSZ_32 mux (klauss_pkg.sv:695) ignores it, and `LDIDX8_S`/`LDIDX16_S`
already have opcodes (KlaussCPU.sv:2119, 2121).

**Fix:** one dispatch arm at the free encoding `0x0000_C8??` calling
`f_ld_idx(MSZ_32, is_signed=1)`, honour `is_signed` in the MSZ_32 mux, **and
update all three enumeration sites**: the dispatch casez, `f_predecode_len`
(klauss_pkg.sv:~203 — without it the fast-path prefetch breaks around the new
op), and `f_perf_class`. Backend: `setLoadExtAction(SEXTLOAD, i64, i32, Legal)`
+ one pattern. Three added decode arms ⇒ re-run STA.

### B3. LDIDX64R/STIDX64R: kill the dead settle cycle; add a scale [Unverified]

`f_ld_idxreg`'s `extra_clock == 1` cycle (klauss_pkg.sv:761-762) is a pure
wait for the port-B redirect to settle; the RAW-forward mux pattern
(KlaussCPU.sv:589-590) could supply the offset a cycle earlier (−2 cycles per
register-indexed access). Separately, the form has no scale, so `a[i]` costs a
2-word `SHLV` first; a 2-bit scale in the unused imm bits applied at the
cycle-2 address add (klauss_pkg.sv:765) deletes that instruction. Both changes
live inside the registered `f_ld_idxreg` path, away from the decode cluster.

### B4. Divider: normalize by the divisor, not the dividend [Confirmed]

`DIVIDE_PREP` pre-shifts the *dividend* and skips `clz(dividend)` iterations
(KlaussCPU.sv:2677-2689), so iteration count = 64 − clz(dividend). Dividing two
similar-magnitude 64-bit values (hash % prime, fixed-point) still runs ~64
iterations (~67 cycles — worse than a cache miss) to produce a 1–4-bit
quotient. The information-theoretic count is the *quotient* width:
`clz(divisor) − clz(dividend) + 1`.

**Fix:** in DIVIDE_PREP, compute `shift = clz(divisor) − clz(dividend)`,
pre-shift the **divisor** left, iterate `shift+1` times shifting the divisor
right against a static remainder. The protected DIVIDE_STEP trial-subtract
carry chain (comment KlaussCPU.sv:2673-2676) is untouched; cost is a second CLZ
(~70 LUTs) + one more 64-bit shifter in PREP (split PREP into 2 cycles if
needed). 64-bit similar-magnitude divides: 67 → ~7–15 cycles; no regression
for small operands. Avoid radix-4 — it doubles logic on the flagged path.

### B5. MULTIPLY_PIPE is a dead state [verified first-hand during this review]

The DSP pipeline is free-running: `a_q/b_q` load during MULTIPLY_SETUP,
`r_mul_pipe1` is valid after MULTIPLY_BREG, `r_mul_pipe2` after MULTIPLY_CALC
(KlaussCPU.sv:510-536). `r_mul_result_hi/lo` are therefore already stable
*during* MULTIPLY_PIPE — MULTIPLY_WRITEBACK's work can execute one state
earlier. **Fix:** `MULTIPLY_CALC → MULTIPLY_WRITEBACK` directly. −1 cycle per
multiply (6 → 5 including dispatch), zero datapath change. Verify against the
DSP48 inference (the state names vs. actual register stages are already one
off; the product registers, not the names, are what matter).

### B6. Reclaim the Fmax margin hiding behind the LED reset path [Confirmed, corrected]

The design's binding setup path is `r_SM_reg[19]/C → r_led_reg[12]/R`
(+0.070 ns, PHASE0_BASELINE.md Part C) — FSM state decode into a *display
register's* synchronous-clear pin, while the real datapath (divide chain,
forwarding spike) has ≥ +0.78 ns of slack. Your true architectural margin is
several times what the headline WNS suggests, and every timing-gated item in
this document is being judged against an artificially tight number.

*Corrected fix (the original "move the 104 display bits out of cpu_state_t"
does not remove any mux — no `f_*` function writes those fields, so the struct
arms are identity pass-throughs synthesis already collapses):* **pipeline the
display clear/write strobes by one register stage** so the path no longer
launches from `r_SM` (display updates tolerate any latency; zero CPI change).
Use `set_multicycle_path 2` from `r_SM_reg*` to the display regs as a
*measurement* experiment first (PHASE0_BASELINE.md already proposes this), but
don't leave it as the permanent fix — the display registers are MMIO-readable
(KlaussCPU.sv:848-863), so a genuinely multi-cycle path could corrupt
software-visible values.

**Do this first.** It's nearly free and it widens the timing budget that gates
§C1, §C2, §C4, and §E1.

### B7. SETR64/PUSHV64 self-fetch their third word without checking the IFB [Unverified]

`f_setr64`/`f_pushv64` issue a full ~5-cycle cache read for the hi32 word at
PC+8 (klauss_pkg.sv:900-920, 1068-1097) even when the IFB/IR already buffered
that doubleword. Low priority (SETR64 is rare if A2/§A4 constant policies land),
but it's a contained fix in two functions. Prefer the backend-side mitigation:
emit `SETR` (+`ZEXTW` for 32-bit unsigned addresses) instead of SETR64 whenever
the constant fits.

---

## C. Front-end: the dominant lever

Fetch sequencing was 55–61% of all cycles at the Phase-0 capture. P4.1 (IR
latch + fast-path dispatch) attacks it, but verification shows the fast path
fires only 15–53% (comment KlaussCPU.sv:366-370) for structural reasons that
are fixable.

### C1. Let 1-cycle ops arm the fast path [Confirmed]

The fast-path dispatch requires `r_ir_presettled` (KlaussCPU.sv:1787), which is
armed **only** in ALU_FINISH (2772-2776), MULTIPLY_WRITEBACK (2618-2622),
DIVIDE finish (2730-2734), and the not-taken branch arms (2157-2200). The ~35+
single-cycle ops (bitwise `f_alu`, all `f_wb`/`f_wb_ns`/`f_rot1` ops, `f_cmpr`
boolean compares/min/max, shifts, COPY, SETR, GETSP…) return to OPCODE_REQUEST
without pre-loading `st.reg_1/2` — so their successor **always** pays the
2-cycle IFB path and *discards a valid prefetched IR* (1839-1841). One pure
wasted cycle per dynamic single-cycle instruction with an in-IFB successor.
(Note: INCR/DECR and all ADD/SUB-class ops are *not* affected — they go through
ALU_FINISH, which arms the fast path.)

**Fix:** extend the proven not-taken-branch pattern — pass
`w_ps_ok/w_ps_r1/w_ps_r2` (KlaussCPU.sv:477-479) into `f_wb`/`f_cmpr`/bitwise
`f_alu` (set `n.reg_1/2` when `ps_ok`, as `f_cond_jump` does at
klauss_pkg.sv:1024-1029) and assert `r_ir_presettled` in those arms. The RAW
case is already covered by the wb-forward mux (589-590).
**Risk:** widens the `st.reg_1/2` source mux inside the OPCODE_EXECUTE decode
cluster — do the highest-frequency arms first (COPY/SETR/bitwise/shifts) and
gate on STA. **Gain:** −1 cycle per affected instruction; a few tenths of CPI
on ALU-heavy code; directly visible in the fast-path perf counter.

### C2. One dedicated redirect-target IFB slot [Confirmed, corrected]

A taken control transfer whose target is outside the single buffered line pays
the full 6-cycle cache fetch at the target (1 REQUEST + 4 OPCODE_FETCH + 1
FETCH2; taken `f_cond_jump` does no target prefetch, klauss_pkg.sv:1021-1023).
Any loop spanning more than one line evicts its head line before the back edge
— so the loop head misses **every iteration** by construction. The earlier
"S=4 IFB captured zero extra hits" experiment (comment KlaussCPU.sv:443-450)
tested *sequential* capacity, not a target slot; it doesn't contradict this.

**Fix:** add one separate `{2×dw, tag, valid}` slot filled only on
redirect-target fetches — best restricted to **backward-taken branches** (loop
heads) so call/ret alternation doesn't thrash it — with the same SMC
invalidation as the existing slots (KlaussCPU.sv:1431-1445). Loop-head fetch
goes 6 → 2 cycles and re-primes the FPC prefetch chain.
**Risk:** one more tag compare feeding the dispatch mux in the thin cluster;
timing-gate it. **Gain (corrected):** bounded by the fraction of iterations
with no internal taken transfer — realistically ~−0.3 to −0.5 CPI on branchy
code; near-zero BRAM (it's FFs).

### C3. Issue the next-line fetch during the execute tail (P4.1c) [Confirmed, corrected]

The IR-latch prefetch is IFB-hit-only ("no cache request",
KlaussCPU.sv:1401-1409), so the first fetch in every new line takes the
serialized 6-cycle cache path even at 100% hit. Dense 4-byte code partially
self-mitigates (the last-in-line instruction's VAR1_FETCH refills the IFB with
the next line), but 8-byte/mixed code and post-redirect code pay it in full.

**Fix (= the plan's own P4.1c):** arbitrate the single cache port so an
IFB-miss read at `r_FPC` issues during the bus-idle execute tail
(`w_exec_tail`, KlaussCPU.sv:486-489), with a response-capture register
(cpu.ready is a strict 1-cycle pulse — mem_read_write.sv:437-459 — that would
otherwise be missed mid-execute) and redirect cancellation via the existing
consume-guard philosophy.
**Gain (corrected):** ~2 hidden cycles per line crossing under 2-cycle ALU
tails (up to ~4 under mul/div tails) — roughly −0.3 to −0.5 CPI on compute
kernels, **not** the −1.0 the plan's early estimates suggested. The plan
already marks P4.1c "droppable if it destabilizes timing"; that judgement is
right — do it after §C4's spike result is known, since §C4 subsumes most of it.

### C4. The de-scoped Phase 5: the second BRAM port is free [Confirmed, corrected]

This is the most important structural finding of the review. PIPELINE_PLAN.md
Phase 5 fears a split I/D cache "adds BRAM… may not close at 100 MHz" with
BRAM already at 68.9%. But an exhaustive trace of `mem_read_write.sv` shows
**every** tag/data/LRU BRAM read occurs only in WAIT (489-495) or MAINT/MS_READ
(788-793), and **every** write only in CHECK (553-556, 596), WRITE_FETCH
(691-698), READ_WAIT (750-757), or MS_CLR (853-854) — mutually exclusive
one-hot states. Reads and writes can therefore be merged onto physical port A
(the address is `r_cache_index` in every write state, and the existing
`w_rd_index` mux at 360-361 already handles the read-address select), freeing
**port B of the same arrays as a dedicated read-only instruction-fetch port at
zero additional BRAM** — and, because it's the same physical cache, SMC
coherency comes for free (no I-cache invalidation protocol needed).

That removes the structural fetch-vs-data hazard that caps every pipeline
phase: a small fetch FSM on port B keeps the IFB/IR latch streaming while
port A serves loads/stores, and it is the prerequisite the 3-stage pipeline's
~2× ceiling is gated on.

**Required care (from verification):** (a) port B must stall/retry on same-set
port-A writes — TDP cross-port write/read collisions return undefined data;
(b) the fetch FSM feeding the IFB/IR latch lands next to the dispatch cluster,
so the +0.070 ns constraint applies to the front-end half too (§B6 first);
(c) mem_stream/ptr_chase stay gated by the DDR miss penalty — this wins on
compute kernels.

**Recommendation:** run this as a Phase-3-style throwaway timing spike **now**
(port-merge + port-B read path + registered tag compare, no functional hookup).
If it closes, it replaces both Phase 5 and most of P4.1c at a fraction of the
feared cost.

### C5. Dispatch-from-tail (skip OPCODE_REQUEST) — later, after re-baselining [Confirmed, corrected]

Every instruction pays the OPCODE_REQUEST cycle. It is tempting to consume the
IR and jump straight to OPCODE_EXECUTE from ALU_FINISH, but verification shows
that cycle is not dead: it is the operand-settle cycle for the registered read
ports, with the wb-forward mux firing precisely because `st.wb.pending` is high
then. Dispatch-from-tail therefore needs an additional forward leg from
`st.alu_pipe_value` (the result isn't in `st.wb` yet at the ALU_FINISH edge) —
a wider forwarding path than the GREEN +0.78 ns spike covered. Re-spike before
attempting; only the ALU_FINISH tail is worth it, and only after §§C1–C4 have
raised the fast-path rate enough for this to be the binding cost.

---

## D. Memory system

The cache-hit handshake is ~5 CPU-visible cycles of which only 2 (WAIT + CHECK)
are tag+data work; verification showed the PRE_WAIT/COOL_DOWN dead states
overlap with the return-path registration and re-arm exactly in time — deleting
them alone saves ~nothing for this blocking CPU. The real levers are below.
(All of §D lives in `mem_read_write.sv`/`ddr2_control.sv`, away from the decode
cluster — low Fmax risk, but same 100 MHz domain: re-run STA.)

### D1. Fetch-first dirty misses + hit-under-writeback [Confirmed, corrected]

On a dirty miss the refill the CPU is stalled on waits behind the entire victim
write transaction plus exactly 3 dead gap cycles (READ_EVICT →
READ_EVICT_DONE → READ_EVICT_GAP with `r_gap_count=2`,
mem_read_write.sv:713-739; same chain for write misses at 622-662). The
mem_stream-vs-ptr_chase penalty delta (65.9 vs 54.1 cycles/miss) is ~12 cycles
of pure writeback serialization.

**Fix:** issue the refill read first, holding the victim in the already-existing
`r_evict_data_hold`/`r_evict_ddr_addr_r` (293-294) as a 1-entry victim buffer,
and drain the writeback afterwards. Two necessary details: the drain must
still respect ddr2_control's both-DVs-low IDLE gate (or remove it — §D2), and
the big win requires **hit-under-writeback** (return to WAIT and serve hits
while the victim drains, with an address compare against the buffered victim);
a blocking drain recovers only ~2–5 cycles. With hit-under-writeback:
most of ~12 cycles per dirty miss; mem_stream roughly −0.4 to −0.8 CPI.

### D2. Replace the level-DV framing between cache and ddr2_control [Unverified]

Every DDR transaction pays: WAIT sample + command/beat cycles + WRITE_DONE/
READ_DONE drain + IDLE-until-both-DVs-low (ddr2_control.sv:177-193) — ~5–8
cycles of framing per transaction, run twice (plus the 3-cycle gap) on a dirty
miss. Of the 54–66-cycle miss penalty, the actual data transfer is 2 ui_clk
beats. A one-shot command pulse with a latched request deletes the IDLE gate,
the `r_gap_count` states, and lets ready assert on beat-1 acceptance — and it
unlocks command pipelining (issue the refill read while the writeback data
drains), stacking with §D1. Same-domain, two-module change.
Est. ~5–8 cycles off clean misses, ~10–12 off dirty ones.

### D3. Post stores [Unverified]

A write hit completes inside the cache 2 cycles after acceptance
(mem_read_write.sv:522-559), yet the CPU spins in OPCODE_EXECUTE for the full
~5-cycle ready round trip (f_mem_store klauss_pkg.sv:617-634; same for
f_push) for an acknowledgment it never uses. Posting the store — return to
OPCODE_REQUEST on the issue cycle, plus one "store in flight" bit that stalls
the *next* memory op / IFB-missing fetch until ready — recovers ~4 cycles per
store/PUSH/CALL-push. The IFB SMC invalidate already keys off the store DV at
issue time (KlaussCPU.sv:1431-1445), so coherence is preserved. Touches the
fetch-dispatch guard ⇒ timing-gate. Est. −1 to −1.5 CPI on store-heavy code;
helps every CALL.

### D4. Streaming: next-line prefetch (first) or 32-byte lines (second) [Confirmed]

A streaming read uses ~3% of DDR2-400 bandwidth: one 54–66-cycle round trip
per 16 bytes, MIG idle in between; mem_stream's 9.09% miss rate *is* the
line-crossing rate. Ranked fixes:
(a) **1-line stream buffer:** on demand-miss completion, speculatively issue
the +16 B line into a 128-bit side buffer; a subsequent miss that hits it
installs in ~4 cycles. Contained in mem_read_write + ddr2_control.
(b) 32-byte lines at constant 64 KB (2 back-to-back BL8 bursts per fill,
+~4–8 cycles per miss, half the misses; tag BRAM halves). Riskier — it touches
line-offset logic, `next_valid`, and the IFB. Do (a) first.
Est. mem_stream −2 to −3 CPI; instruction-side cold misses halve too.

### D5. Return-address stack (1–2 entries) [Unverified]

CALL/RET stack traffic is always L1-hot, so the ~10-cycle round trip is pure
handshake. A tiny RAS beside the FSM — CALL captures `{SP−8, PC_ret}` while
the push proceeds architecturally; RET whose SP matches uses the captured PC
immediately (memory read becomes verify-only or posted) — saves ~4–5 cycles
per RET and re-enables the target fast path. Invalidate on a store matching
the saved slot (the SMC compare pattern is the template). Do after §§D1–D3;
overlaps with A5 for leaf calls.

---

## E. ISA evolution (larger changes, all timing-gated)

### E1. Fused compare-and-branch [Confirmed (cost) / corrected (gain)]

Every compiled condition is `CMPRR` (1 word, 2 execute cycles through
ALU_FINISH) + `JMPcc` (2 words, own fetch/dispatch/execute) — ~5 cycles best
case not-taken, ~7–11 taken, 12 bytes, for what a RISC `blt` does in one
instruction. Two implementation options:

- **Macro-fusion (preferred — no ISA change):** in ALU_FINISH's CMP branch,
  when the prefetched IR holds a conditional jump for the same PC
  (`alu_pipe_mode==CMP && r_ir_valid && r_ir_pc==st.PC && opcode ∈ the 18
  conditional-jump encodings && r_ir_var1_prefetched`), resolve the condition
  from `alu_pipe_equal/less/ult`, set PC to target or fall-through, retire
  both, return to OPCODE_REQUEST. Fall back to the normal path on any miss
  condition. *Corrections that must be respected:* match the exact 18 encodings
  (0x1001-0x1008, 0x1013-0x101C — a loose 0x10xx match would fuse JMP/CALL/RET
  wrongly); only the equal/less/ult/sign families; keep the perf-branch strobes
  and the debug-step gate. Taken pairs save 2 cycles; not-taken pairs save ~1
  (the consumed IR forfeits today's not-taken pre-settle).
- **New instruction (`CMPBcc rs1, rs2, target`, RRV):** same effect for new
  code, but costs encoding space, three enumeration-site updates, and backend
  work.

Est. branchy CPI −0.5 to −0.9 (taken-ratio-dependent). This lands in the thin
decode/ALU_FINISH cluster: do §B6 first, gate on STA.

### E2. 1-word short conditional branches [Confirmed, corrected]

All 18+18 conditional jumps are 2-word/8-byte (klauss_pkg.sv:211-223) even
though most compiled branches are short-range back edges. A 1-word PC-relative
form with the offset in the opcode word (e.g. signed word-offset in the free
[7:0] byte, ±512 B; or imm16 in [31:16] — which predecodes as 1-word for free
via the upper-16-nonzero test at klauss_pkg.sv:169 provided the tag avoids the
RRR register-field space) saves 4 bytes per branch and removes the branch's
own VAR1 exposure. Requires a PC+4 fall-through variant of `f_cond_jump`
(klauss_pkg.sv:1025 hardcodes +8) and updates to all three enumeration sites.
Est. 10–20% code-size cut on branchy code, ~−0.15 to −0.4 CPI — compounds with
E1 and with the backend's branch relaxation choosing short vs. long.

### E3. SP-relative addressing → frame-pointer elimination [Unverified]

There is no SP-relative load/store, so the backend keeps FP in R15: every
non-trivial function pays PUSH R15 + GETSP/COPY + POP R15 (~15 cycles of frame
plumbing), and the allocatable set shrinks to 15. `LDIDXSP rd, imm32` /
`STIDXSP rs, imm32` are two casez arms calling the existing `f_ld_idx`/
`f_st_idx` with `eaddr = st.SP + w_var1` — no new datapath. Then enable FP
elimination in the backend: frees R15 (16th allocatable register — the best
spill-pressure relief available anywhere in this document) and deletes the
prologue/epilogue plumbing. Interim backend-only step: FP elimination via
GETSP for fixed-size frames.

### E4. Flag-free ADD/SUB variants [Confirmed, corrected]

ADD/SUB-class ops pay the 2-cycle EXECUTE→ALU_FINISH split purely for the
carry/overflow flag path (klauss_pkg.sv:38-44), yet LLVM-generated code
consumes CMPRR flags, essentially never arithmetic carry. A flag-free mode
taking the bitwise 1-cycle exit (`wb.value = sum`, keep `set_zero`) is sound —
**but** the saved cycle only materializes if paired with §C1's presettle
extension, because today ALU_FINISH is where the successor's fast path gets
armed; without the pairing, a "1-cycle" add still costs 3 cycles end-to-end.
Implement as: repurpose/add ADDNF/SUBNF opcodes (or make ADDI flag-free —
nothing consumes its carry), emit them from the backend when flags are dead,
and land §C1 first. Combined with §C1: alu-kernel CPI toward ~4.5 (pre-P4.1
numbers; re-baseline).

---

## F. Process and measurement

1. **Re-baseline first.** Phase-0 CPIs predate P4.1 (landed 2026-06-26). Re-run
   the six `perf_baseline` kernels and record the post-P4.1 fetch/exec split
   and the fast-path counter before starting anything above; re-rank §C by the
   new numbers.
2. **Order of work:** §B6 (timing margin) → §A1–A3 + B1/B2 (cheap, big) → §C4
   spike (decides the front-end strategy) → §C1/C2 → §D1–D3 → §E1/E2 → rest.
3. **The predecode triplication hazard** (dispatch casez, `f_predecode_len`,
   `f_perf_class`) is now a *performance* regression vector: a missed
   `f_predecode_len` arm silently disables the fast path around a new opcode
   (the consume-guard eats the error). Every ISA addition in §B2/§E needs all
   three sites plus a `PREDECODE_MISMATCH`-style counter check on the board —
   consider generating all three from one table to kill the hazard class.
4. Findings marked **[Unverified]** (§A3, A6, A7, B3, B7, D2, D3, D5, E3, and
   the seven per-item notes) had their verification pass cut short; the RTL
   citations were produced by reviewers reading current sources, but re-check
   cycle counts before scheduling work on them.

---

## Appendix: what was checked and found sound

Worth recording so future reviews don't re-litigate them:

- The P4.1 IR-latch design (consume guard, SMC poisoning of the latch and IFB,
  IRQ-priority ordering ahead of the fast path) verified clean — no
  correctness-relevant gaps found in the prefetch path.
- The deferred-writeback commit inside OPCODE_REQUEST plus the read-port
  RAW-forward mux (KlaussCPU.sv:589-590) is sound and is the right foundation
  for §B1/§C1.
- PRE_WAIT/COOL_DOWN in the cache are *not* wasted cycles for the current
  blocking CPU (they overlap the registered return path); the earlier plan
  assumption that deleting them is a cheap win is wrong — overlap (§C3/§C4) or
  de-registering a return stage is what actually cuts hit latency.
- The 2-way 64 KB cache organization itself (2048 sets, BRAM tags, hits
  completing in CHECK) is well matched to the FPGA; the wins are in the
  handshake, the miss path, and the second port — not in reorganizing the
  arrays.
- `LDIDX`/`STIDX` effective-address hardware handles negative offsets today —
  and, verified 2026-07-03, the **real backend already models them as `simm32`
  and folds them** (§A2/§G); only the RTL-side markdown docs
  (CPU_ARCHITECTURE.md, LLVM_NEW_INSTRUCTIONS.md) still say `zero_ext`, which is
  now purely a doc bug to fix.

---

## G. Reconciliation with the actual LLVM backend (2026-07-03)

Sections A–E were written from the in-tree markdown (`llvm_backend_plan.md`,
`LLVM_NEW_INSTRUCTIONS.md`), which describe an **earlier** state of the backend.
Reading the real out-of-tree sources at
`/media/psf/src/klausscpu-llvm/llvm/lib/Target/KlaussCPU` changes the verdict on
several LLVM items. Every row is cited to real backend source.

| Rec | Verdict | Evidence in the real backend |
|---|---|---|
| **A1** callee-saved split | **ALREADY DONE** | `CSR_KlaussCPU = (add R4,R5,R6,R7)` (KlaussCPUCallingConv.td:37); `CalleeSavedRegs[] = {R4,R5,R6,R7}` (KlaussCPURegisterInfo.cpp:37-40). Not "caller-saved-everything." |
| **A2** fold imm offsets (incl. negative) | **ALREADY DONE** | Offsets are `simm32` (KlaussCPUInstrInfo.td:269,282); general `(load (add GPR, simm32_imm))→LDIDX64` / store / i32 / i8_S / i16_S fold patterns (td:768,776,798-800,1049-1051); `eliminateFrameIndex` folds the frame offset (incl. negative `BaseOffset`) into the load/store imm (KlaussCPURegisterInfo.cpp:89-95); spills emit LDIDX64/STIDX64 directly (KlaussCPUInstrInfo.cpp:41,64). ADDI is only the variable-index fallback. |
| **A3** flag reuse / compare elimination | **STANDS** | No `optimizeCompareInstr`/`analyzeCompare` anywhere in the backend; `BR_CC` emits a fresh `CMPRR_I`/`CMPRV_I` before every conditional jump (KlaussCPUISelDAGToDAG.cpp:367-412). Count-down loops still emit a redundant compare; identical compares are not CSE'd. |
| **A4** code-size cost model / no-unroll | **PARTIAL** | INCR/DECR peephole **is** done (`(add GPR,1)→INCR`, `(add GPR,-1)→DECR`, td:752-753). But **no `KlaussCPUTargetTransformInfo` exists** — no `getUnrollingPreferences`, no cost hooks — so LLVM unrolls/inlines on generic defaults with no signal that fetch is the bottleneck. The TTI half stands. |
| **A5** leaf link-register convention | **STANDS** (low priority) | `CLI.IsTailCall = false` unconditionally (KlaussCPUISelLowering.cpp:676); CALL always saves LR to the DDR2 stack. But **E3 dominates** — see below. |
| **A6** fall-through block layout | **ALREADY DONE** (MI level) | `analyzeBranch`/`insertBranch`/`removeBranch` implemented (KlaussCPUInstrInfo.cpp:100-217), so the default `MachineBlockPlacement` runs. Only front-end (`__builtin_expect`/PGO) hints remain, and those are minor. |
| **A7** branchless SELECT (min/max/abs) | **ALREADY DONE** | `SMIN/SMAX/UMIN/UMAX` and `ABS` are all `Legal` (KlaussCPUISelLowering.cpp:155-158,170), so DAGCombiner forms them branchlessly before SELECT. The remaining `SELECT`→diamond (td/EmitInstrWithCustomInserter) is only for genuine ternaries — acceptable. |
| **B2** LDIDX32_S (signed i32 load) | **STANDS** | `setLoadExtAction(SEXTLOAD, i64, i32, Expand)` (KlaussCPUISelLowering.cpp:92-93); no `LDIDX32_S` opcode in the td (only 8_S/16_S at td:674-675); signed `int` load still lowers to `LDIDX32 + SEXTW` (SEXTW = `sext_inreg i32`, td:540-541). Needs the RTL arm **and** the backend `Legal` + pattern. |
| **E3** SP-relative addressing → FP elimination | **STANDS — stronger than stated** | The backend **always** emits a frame pointer: `emitPrologue` unconditionally does `PUSH R15; GETSP R15; [ADDSP -N]` and `emitEpilogue` `SETSP R15; POP R15` (KlaussCPUFrameLowering.cpp:80-108); `getReservedRegs` always reserves R15 (KlaussCPURegisterInfo.cpp:62). So **every** function — even a leaf with no locals — pays a PUSH/POP R15 stack round trip and loses R15 from allocation (15 usable GPRs). There is no `hasFP()` gate and no shrink-wrapping. `LDIDXSP/STIDXSP` (the §E3 hardware ops) are the enabler that lets the backend drop the frame pointer. |
| **E4** flag-free ADD/SUB emission | **STANDS** (paired) | Backend models no flag liveness (same gap as A3), so it cannot know when carry/overflow are dead; depends on both A3-style flag modeling and the §E4 hardware mode. |

**Net effect on the LLVM plan:** the two items I had ranked #1 and #2 (A1, A2)
are already implemented and well-engineered — drop them. The real standing
LLVM-only work, in priority order, is:

1. **A3 — compare/flag optimization** (`optimizeCompareInstr` + count-down loop
   flag reuse). Biggest pure-software CPI win on loop-heavy code; nothing in the
   backend does it today.
2. **E3 — frame-pointer elimination**, enabled by adding `LDIDXSP`/`STIDXSP` to
   the RTL. Frees a 16th register and deletes the unconditional PUSH/POP R15
   from every function. This is the single largest register-pressure +
   per-call-overhead lever, and it is currently paid on 100% of functions.
3. **B2 — `LDIDX32_S`** (RTL arm + `setLoadExtAction(SEXTLOAD,i64,i32,Legal)` +
   pattern). Removes a SEXTW from every sign-relevant `int` load.
4. **A4 (TTI half)** — add a `KlaussCPUTargetTransformInfo` that reports high
   instruction cost / low unroll preference, so LLVM stops unrolling and
   over-inlining on a fetch-bound core with a one-line IFB.
5. **A5** — leaf link-register convention (minor; mostly subsumed once E3 lands,
   since the dominant per-call cost is the FP push/pop, not the return-address
   push).

**One correctness-driven regression to be aware of (not a recommendation):**
jump tables are disabled (`setMinimumJumpTableEntries(INT_MAX)`,
KlaussCPUISelLowering.cpp:132) to work around a register-allocator reload bug on
indirect-branch predecessor edges. Dense `switch` statements therefore compile
to binary-search compare chains (more branches, more I-footprint) instead of a
single indexed jump. Re-enabling once the RA limitation is fixed would help
switch-heavy code — but that is a correctness prerequisite, not a perf knob to
flip now.

**Doc-hygiene item (zero code):** `CPU_ARCHITECTURE.md` §5/§8.3 and
`LLVM_NEW_INSTRUCTIONS.md` still document LDIDX/STIDX offsets as
`zero_ext(imm32)`. The hardware and the backend both treat them as signed
32-bit; fix the wording so it stops contradicting the shipped `simm32` patterns.

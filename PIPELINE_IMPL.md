# KlaussCPU — Pipeline Implementation (Phase B, interlock-first)

> **RESUME (session handoff).** Branch `feat/pipeline`. Done + xsim-verified:
> **M0** stage types, **M1** ALU pipeline, **M2** MEM/loads/stores, **M3**
> branches, **M4** mul/div busy interlock — all in `pipeline_core.sv` +
> `tb_pipeline.sv` (§6 milestone table). M4 models the silicon units: mul =
> free-running DSP48 chain (4 EX cycles), div = CLZ-prep + 1-bit/cycle restoring
> loop + 1-cycle by-zero path (`f_div_setup` semantics); mul/div write partial
> flags, so ID holds them on `flag_busy` like branches (see core header).
> **Next: M5** (integrate into the real `KlaussCPU` — the big one).
> Discipline: interlock-first (no forwarding before
> M5), and **xsim-verify every step before committing**. Verify loop:
> `xvlog -sv klauss_pkg.sv pipeline_core.sv tb_pipeline.sv && xelab tb_pipeline
> -s t && xsim t -runall` (Vivado on PATH via `/opt/Xilinx/2025.2/Vivado/bin`).
> `master` = flags + 32 B (board-verified, on QSPI) — untouched.


**Branch:** `feat/pipeline`. **Oracle:** `master` FSM (@81cc547) + `klausscc
--emulate` — every milestone must be UART-bit-identical to it.
**Approach:** clean rewrite on this branch; the whole-struct FSM stays on `master`
as the reference. Build the pipeline **interlock-only first** (correctness), then
layer forwarding / split-port / branch-predict / precise-IRQ as separate,
individually board-tested milestones. See `PIPELINE_5STAGE_SCOPE.md` for the why.

---

## 1. Datapath decomposition — `cpu_state_t st` → per-stage bundles

Today one `st` holds everything and advances atomically. The pipeline splits it
into five stage-latch bundles (SystemVerilog packed structs in `klauss_pkg`), each
with a `valid` bit, advanced by its own `always_ff`. Architectural state that is
NOT staged (the register file, SP, flags, int_mask, the caches, MMIO) stays
central and is written only at **WB** (the single retire point).

| Bundle | Fields (carried down the pipe) |
|---|---|
| **`if_t`** | `valid, pc, words[0:2] (assembled via LEN), len, fault` |
| **`id_t`** | `valid, pc, opcode, class, op, attr, rd/rs1/rs2, imm (extended), len, is_branch/is_mem/is_mul/is_div/is_store/writes_reg/writes_flags` |
| **`ex_t`** | `valid, pc, rd, result, flags_out, writes_reg/writes_flags, is_mem/is_store, mem_addr, store_data, taken/target (branch), mul_div_busy` |
| **`mem_t`** | `valid, pc, rd, value, flags_out, writes_reg/writes_flags` |
| (**WB**) | commits `value→r_register[rd]`, `flags_out→flags`; retire bookkeeping |

**Central (non-staged), written at WB only:** `r_register[16]`, `flags`, `SP`
(push/pop adjust in MEM but commit at WB), `int_mask`, `PC`-redirect. Keeping
these single-writer at WB is what makes interrupts precise (§4).

## 2. Stage logic — where each `f_*` goes

The `f_*` next-state functions (klauss_pkg) are refactored into **stage-local
combinational** blocks; the whole-struct plumbing is dropped.

- **IF:** existing `r_FPC`/IFB machinery → `if_t`. Assemble 1/2/3 words via
  `f_predecode_len`. I-side of the cache (shared port for now, §5).
- **ID:** decode = the class/len dispatch that today lives in `casez(w_opcode)`,
  but producing *control bits* (not executing). 3-port RF read + imm extend +
  the **forward mux** (§3) → `id_t`. `f_cond_eval`'s COND stays here for branch.
- **EX:** ALU compute = the arithmetic/logic/shift/compare bodies of `f_alu` /
  `f_wb` / `f_cmpr` / `f_rot1`. Flag compute (Z/S/C/V). Branch cond+target. AGEN
  (`f_ld_idx`/`f_st_idx` address math). Issue mul/div to their units → stall.
- **MEM:** the load/store handshake bodies of `f_ld_idx`/`f_st_idx`/`f_push`/
  `f_pop`/`f_mem*` — D-side cache access. Multi-cycle: hold the pipe.
- **WB:** commit `value`+`flags`; the retire bookkeeping (`r_instr_count`, trace)
  moves here (was `OPCODE_FETCH2`); deferred-WB machinery is subsumed.

## 3. Hazard unit
- **GPR RAW:** **EX→EX forward, 2-input** (reg / EX-result), P3-verified +0.080.
  Mux sits on the ID→EX operand path (the reg-read output), gated by
  `ex.writes_reg && ex.rd == id.rs`. **MEM→EX is NOT forwarded** — handled by the
  load-use interlock (keeps the mux 2-input; that's the timing budget).
- **Load-use interlock:** if `ex.is_mem_load && (ex.rd == id.rs1|rs2)` → stall
  ID/IF one cycle (bubble into EX).
- **MUL/DIV busy interlock:** EX issues to the iterative unit and asserts a global
  stall until it retires (mul 6c / div ≤66c). Freeze IF/ID/EX; MEM/WB drain.
- **Flags:** one forwarded flags register EX→EX (clean post-unification).
- **Control:** resolve branch in EX; **static predict-not-taken**; on taken, flush
  IF+ID (2 bubbles) and redirect PC. Reuses the existing `r_ir_pc==PC` consume
  idea generalized to per-stage valid+squash.

## 4. Precise interrupts / SMC / HCF (the invariants)
- **Retire = WB only.** IRQ sampled at the WB/commit boundary; on take, **flush
  all younger stages** (IF/ID/EX/MEM valid←0), push the frame (PC of the *next*
  un-retired instr + 7-bit flag word + int_mask) via the MEM port, redirect to the
  vector. `IRET` restores at WB. Preserves the single-dispatch/single-retire model
  the FSM has today, just with a flush.
- **SMC:** a store committing at WB/MEM compares its address against the PCs in
  IF/ID/EX; on match, squash those stages and refetch (superset of today's IFB
  poison). Plus the existing IFB/I-cache line invalidate.
- **HCF/crash/debug/boot:** on entry, **drain the pipe** and run these in a
  non-pipelined fallback mode (rare, need not be fast). Crash dump snapshots the
  WB-retiring instruction for a precise fault PC.

## 5. Memory port
First pipeline: **shared cache port, MEM priority** — stall IF whenever MEM
accesses (structural hazard). Correct but throughput-limited on load/store-dense
code. The **split I/D cache port** (`PIPELINE_MASTER_PLAN` §4.3) is a later
milestone (M7) that removes the stall; it adds I-cache invalidate-on-store for SMC.

## 6. Milestones (each board-tested, UART-bit-identical, own commit)
| M | Scope | Exit |
|---|---|---|
| **M0** | Stage-bundle structs + top-level stage registers + valid/stall/flush skeleton (no real logic; passthrough of a NOP stream) | compiles, xvlog clean, sim harness runs |
| **M1** | Straight-line **single-cycle ALU** ops through IF/ID/EX/WB, interlock-only, shared port | a pure-ALU test is bit-identical |
| **M2** | + **loads/stores** (MEM stage) + load-use interlock | mem tests bit-identical |
| **M3** | + **branches/jumps/calls/ret** (resolve in EX, flush) | queens/expr bit-identical |
| **M4** | + **MUL/DIV** busy interlock | muldiv/test_64bit bit-identical |
| **M5** | + **precise interrupts, SMC squash, HCF drain, debug, boot/loader** | full regression suite bit-identical; IRQ-under-load correct |
| **M6** | + **EX→EX forwarding** (P3) | CPI drops; interlock-stall counter falls; timing closes (2-input, floorplan if needed) |
| **M7** | + **split I/D cache port** | IF/MEM no longer contend; throughput up |
| **M8** | LLVM `MCSchedModel` + hazard recognizer + TTI re-tune | compiler spreads deps |

**M1–M5 are the correctness build (interlock-only). M6+ are optimizations on a
known-good pipeline.** Do not add forwarding before M5 is bit-identical.

## 7. Testing
- Per milestone: run the relevant regression subset on board + diff UART vs the
  `master`/emulator golden (`perf/golden*`). Add per-stage-stall perf counters at
  M6 (`STALL_DATA/LOAD_USE/MULDIV/BRANCH_FLUSH`).
- Keep a `master`-built reference bitstream to A/B CPI at each milestone.
- Reframe #5: debug hazards in iverilog/golden-trace sim, not 30-min silicon.

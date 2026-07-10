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

---

## 8. M5 execution plan (sub-milestones; each xsim-verified + committed)

M5 = integrate into the real `KlaussCPU`. Split into a—e; a—c stay in the
standalone `pipeline_core` (fast sim iteration), d—e touch the SoC.

**The golden-trace harness (the M5a enabler).** `klausscc --emulate --trace`
prints one line per retired instruction — `i= pc= op= r0..r15 sp f=<7bits>
[wr=addr/be/data]` (f = Z,S,C,V,E=Z,L=S^V,U=C) — a format designed to diff
against an RTL self-trace (`klausscc/src/EMULATOR_ISA_SEMANTICS.md`).
`klausscc --input X.elf --mem-out --mem-file X.mem` flattens the ELF to the
$readmemh DDR image (heap header dword at 0x0, word0.lo32 = heap_start; code
base 0x20; entry = board entry printed by --emulate). So: tb loads real
compiled ELF images, the WB stage $displays the same trace line, tb captures
UART (MMIO store 0xF001_0000) — diff both against the emulator. Bit-exact
per-instruction oracle, no board.
`K=~/Documents/src/klausscc/target/release/klausscc`,
`E=/media/psf/src/klausscpu-runtime/baremetal` (ELFs prebuilt).

| Sub | Scope | Exit gate |
|---|---|---|
| **M5a** | **Full ISA v2 in the standalone core, ideal memory.** IF = 128-bit window (any 1/2/3-word instr spans ≤2 dwords) assembling op/var1/var2; ID = full field decode (classes 1-C) + 3-port RF read (rs1/rs2/rd-data) + imm ext; EX = ALU/CMP/bool-cmp/shift/rot/bit/unary/BEXTR/BDEP/GETF/LEA/MOV + branch resolve (f_cond_eval, LINK/REL/RIND) + AGEN + mul/div issue (M4 units) + SP ops; MEM = all load/store sizes/modes/lanes (f_ld_idx/f_st_idx math), MEMGET32 unaligned 2-access, PUSH/POP/CALL-push/RET-pop/IRET-pop; WB = single retire (reg + flags + SP + trace hook). System: NOP; HALT/TRAP/illegal → drain+park w/ error code; DELAY = EX spin; WAIT = drain+park (IRQ in M5c); LCD = EX side-port. MMIO model in tb (UART TX capture, STATUS reads 0). | hello, bst, expr, test_64bit (+ queens, capped) run from real ELF images in xsim: RTL trace == emulator trace line-for-line AND UART bytes identical |
| **M5b** | **Memory realism**: one shared membus-style port (addr/rdata/wdata/byte_en/read_DV/write_DV/ready), IF vs MEM arbitration (MEM priority), variable-latency memory model (hit ~5c / miss ~50c sweep + random), MMIO 2-cycle ready path split at addr[31:28]==F | same golden gates across the latency sweep |
| **M5c** | **Precise IRQ + IRET + WAIT + SMC + HCF drain** standalone: IRQ sampled at retire boundary, flush younger, frame push {mask,7-bit flags,PC} via MEM port (layout = KlaussCPU.sv:1855-1876), source auto-mask; IRET restores; WAIT parks until irq; SMC: store addr vs in-flight IF/ID PCs → squash+refetch; TRAP/illegal → drain, park, error code | IRQ-storm + SMC + WAIT tb tests pass; frame layout bit-checked; goldens still pass |
| **M5d** | **SoC integration** into KlaussCPU.sv: pipeline transplanted INSIDE the existing main always_ff as the `PIPE_RUN` engine (single-driver rule — st stays FSM-owned; pipeline owns its stage regs and writes r_register/flags/SP/PC at WB). FSM keeps NO_PROGRAM/LOADING/LOAD_COMPLETE/boot-copy/HCF/DEBUG/HALTED (+ their UART machinery); handoff = drain→FSM state / FSM→PIPE_RUN with latches invalidated. IF reuses the IFB (2-dw window) + cache port; MEM shares the port (MEM priority). MMIO write handler stays after the pipeline logic ("MMIO store wins" ordering — the M1 clobber lesson). Perf counters re-homed (cycles/instr/class buckets min). Out-of-case-writer audit per agent map §10 (UART break, timer, MMIO handler, IRQ push). | xvlog + synth clean; tb_pipeline suite still passes; (stretch) mini-SoC xsim with behavioral ddr2_control |
| **M5e** | **Board**: `vivado -mode batch -source ~/.klausscpu_scratch/build.tcl` (background, ~30 min) → `prog.tcl` → full regression: `$K --input $E/{hello,queens,bst,expr,crypto,test_64bit,dhrystone}.elf --serial /dev/ttyUSB1 --monitor` diffed vs `perf/golden*`; perf_baseline CSV capture (CPI A/B vs 4.4-8.8 baseline); WNS ≥ ~0 | full suite UART bit-identical on silicon; CPI + WNS recorded |

**Design decisions (locked during M5a):**
- **Flags = masked merge at WB.** Each op carries {flag values, 4-bit write
  mask Z/S/C/V} down the pipe; WB does `flags <= (flags & ~mask)|(vals & mask)`.
  Writers never read flags → no writer-writer hazard. Only flag READERS
  interlock on flag_busy: branches (cond≠always), ADC/SBC/RCL/RCR (carry-in),
  GETF, IRQ frame save. Per-op masks from the FSM: arith Z/S/C/V; CMP Z/S/C/V;
  bitwise none; shifts Z; rot1 Z+C; SEXTB/H Z+S; ABS Z+V; NEG/NOT/ZEXT/POPCNT/
  BEXTR/BTST Z (F-gated); mul Z/S/V; div Z/V (by-zero: V only).
- **SP serialization interlock**: SP readers/writers (class 9, CALL, IRQ)
  stall until no SP-writer in flight; SP commits at WB.
- **Store data = 3rd RF read port** (rd field) in ID, carried to MEM.
- **RESET opcode → PC=4** (f_ctrl quirk), HALT holds PC, WAIT PC+4.
- **SETR64/PUSHV64 var2 comes from IF assembly** (not a PC+8 self-fetch like
  the FSM) — memory-traffic differs, architecture identical; trace unaffected.

# KlaussCPU — Pipeline Implementation (Phase B, interlock-first)

> **RESUME (session handoff).** Branch `feat/pipeline`. Done + xsim-verified:
> **M0-M4** (stage types, ALU, MEM, branches, mul/div interlock), and **M5a** —
> `pipeline_core.sv` now implements the FULL ISA v2 (classes 1-C) and runs real
> compiled ELFs: hello/bst/expr/test_64bit (to Halt) + queens (2M-instr cap)
> are **trace-identical line-for-line** to `klausscc --emulate --trace` and
> UART-byte-identical (§8 harness; runner `perf/m5a/run_m5a.sh <prog> [maxi]`).
> Two known, DELIBERATE oracle deltas (documented in run_m5a.sh): the f= trace
> field is excluded (emulator doesn't model silicon mul/div flag writes — the
> pipeline implements the FSM's Z/S/V / Z/V semantics), and the emulator's
> UART golden is Latin-1→UTF-8 expanded (undone with iconv before compare).
> **M5b done**: one shared membus-shape port (= cpu_mem signal set), ready
> handshake, IFB-style 2-dword fetch window, MEM-priority arbitration —
> trace-identical across latency 1/9/random. **M5c done**: precise IRQ entry
> (drain at dispatch boundary, frame push, auto-mask), IRET, WAIT wake, SMC
> squash-and-refetch — WAIT+SMC directed tests PASS; bst under a 997-cycle
> IRQ storm (829 dispatches) and test_64bit under 463-cycle storm + random
> latency (3989 dispatches) both retire program-only traces IDENTICAL to the
> no-IRQ goldens (runner `perf/m5a/run_m5c.sh`).
> **M5d done (sim)**: pipeline_core instantiated in KlaussCPU as the PIPE_RUN
> engine (FSM keeps boot/loader/HCF/HALTED; park = drain + snapshot into the
> FSM's arch copies; cpu_mem muxed; IRQ/int_mask/LCD/perf re-wired — see the
> M5d commits). Full-SoC tb (tb_soc + run_m5d_soc.sh, behavioral DDR +
> stubs): hello boots through the REAL boot-ROM→DDR→loader→PIPE_RUN chain,
> runs to HALTED, UART **byte-identical**; trace identical up to the first
> UART-TX busy poll (emulator transmits instantly — extra poll retires are
> legitimate, the board behaves the same). Two integration bugs found IN SIM,
> both fixed+committed: xsim-only X from the un-reset RX FIFO (tb now pulses
> CPU_RESETN), and the stale-MMIO-ready back-to-back hazard → **ready-edge
> arming** in all port engines. Timing: first impl WNS −0.124 (DSP CE cone) →
> mul chain moved to its own free-running block (silicon structure).
> **M5e DONE — M5 COMPLETE, BOARD-VERIFIED (2026-07-10).** Timing closed at
> WNS +0.002 after 3 RTL iterations (DSP-CE cone → free-running mul block;
> EX ==0 reduction → deferred Z at WB; ALU b-operand mux → resolved at
> dispatch — all three are the FSM's own structural patterns; margin is thin,
> firm with floorplan/phys_opt at M6). On silicon (volatile JTAG, master
> still on QSPI): **hello/bst/expr/test_64bit/queens/crypto/dhrystone all
> UART BYTE-IDENTICAL** to the emulator golden (`perf/m5a/run_m5e_board.sh`,
> 7/7), and **test_rtos PASS** (251 timer ticks, 300 context switches —
> precise IRQ/IRET under a preemptive RTOS on hardware). CPI (interlock-only,
> `perf/baseline_m5_pipeline.csv`, NEW ELF set — directional vs the 2-stage
> FSM): alu 4.75 (was 4.44), branchy 4.74 (~flat), calls_fib 6.18 (−1.5%),
> ptr_chase 6.58 (−3%), mem_stream 7.75 (−8%), muldiv 4.48 (−9%). The
> interlock-only pipeline already matches the fast-path-optimized FSM;
> **M6 (EX→EX 2-input forwarding, P3-verified +0.080) is the CPI lever** —
> every ALU RAW currently pays 3 bubbles. Also M6: per-hazard STALL_*
> counters, Tier-2 branch-taken wire-up, firm the timing margin. Then M7
> split I/D port, M8 LLVM sched model. NOT yet on QSPI; debug single-step is
> FSM-era (documented gap); Zephyr bring-up on the pipeline still to do.
>
> **M6 IN FLIGHT (same day).** M6a+b+c implemented + full regression green +
> committed: forwarding + per-hazard counters (MMIO 0xF00D_00B0-E8, in
> MMIO_MAP.md). TIMING LESSON: the zero-bubble version (producer exo_result →
> consumer operand registers) failed WNS −0.750/242 endpoints — the 12-level
> shifter/result-mux tree + operand-register fanout can't close at 100 MHz.
> **M6 v2 = register-sourced forwarding** (what P3 actually measured — the
> FSM forwards from the wb.value REGISTER): operand-register D-inputs muxed
> mem_result (producer in MEM, non-load) / wb_value (producer in WB, loads
> included) / rf; producer-in-EX = exactly ONE bubble; load-use = just the
> load's latency; flags mirror via the WB registers (flags_eff, deferred-Z =
> register-sourced wb_value==0 NOR). CMP→JMPcc 1 bubble (was 3).
>
> **M6 DONE — BOARD-VERIFIED (2026-07-10).** Timing: v2 closed the CPU core;
> impl_1 strategy set to Performance_Explore (route congestion in the EX
> cluster — logic depth was fine at 10 levels; the strategy stays on the
> project). Final WNS −0.016 on 2 endpoints, BOTH in the SHA-256 round
> datapath (crypto_sha/u_sha — the die's documented pre-existing hot path,
> NOT the pipeline; regression doesn't exercise HW SHA). **A QSPI/production
> build needs a seed/directive respin or the SHA relaxation first.**
> On silicon: 7/7 regression UART byte-identical + test_rtos PASS.
> **CPI (perf/baseline_m6_pipeline.csv) vs M5 / vs final FSM (32B, same
> ELFs):** alu 4.125 (−13% / ±0 — TIES the FSM exactly), branchy 4.402
> (−7% / +9.5% — taken-flush cost, the M7 predictor/back-edge territory),
> calls_fib 5.909 (−4% / −4%), ptr_chase 5.926 (−10% / −13%), mem_stream
> 7.463 (−4% / +0.7%), muldiv 4.010 (−10% / −15%); dhrystone 0.154 DMIPS/MHz
> (FSM 0.160). taken_bp counter live again (branchy 56.27% = FSM-exact).
> Net vs the ISA-v2 start: mem_stream −17%, muldiv −15%, ptr_chase −13%,
> calls_fib −8%, alu flat, branchy +7%.
>
> **M7 IN FLIGHT — Phase 0 (measure) DONE (2026-07-13).** Hand-assembled hazard
> probe `perf/m7/haz_probe_*.kla` (gen_probe.py → clear/run/snapshot/print of the
> M6 counters `0xF00D_00B0..E8`, reproducing the 6 perf kernels) validated
> emulator→tb_soc→**board**. Measured per-kernel attribution (`perf/m7/RESULTS.md`,
> `haz_attribution_board.csv`) says: **IF_MISS is the dominant hazard — 33–73% of
> cycles in 5/6 kernels — and large even with zero memory traffic** (alu 93%
> straight-line, MEM_WAIT=0, still 60% IF_MISS; branchy 73%). ⇒ M7 = **1-cycle-hit
> I-cache with its own fetch port** (kills IF_MISS everywhere; the big lever) **+
> split I/D port** (memory kernels' IF_MISS 33–54% and MEM_WAIT 37–54% currently
> serialize on the one shared port). Design + sub-milestones (M7a cache+port,
> M7b fence.i/SMC coherence, M7c re-measure) in §9. Probe cross-checks the real
> baseline (branchy CPI 4.41 ≈ 4.402). Board holds the M6 bitstream (volatile JTAG,
> prog.tcl) as of this measurement. NEXT: implement M7a. Later: branch handling
> (predict/back-edge) for the BRFLUSH residual, M8 LLVM sched model; QSPI after the
> SHA-path respin.
> `master` = flags + 32 B (board-verified, on QSPI) — untouched.
> `tb_pipeline.sv` (toy-ISA M1-M4 tb) was retired with the M5a rewrite —
> superseded by `tb_pipeline_isa.sv` + the golden-trace harness.


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
| **M7** | + **split I/D port + 1-cycle-hit I-cache** (§9; Phase-0 measured 2026-07-13) | IF_MISS (33–73% of cycles) collapses; IF/MEM no longer contend; throughput up |
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

---

## 9. M7 execution plan — split I/D port + 1-cycle-hit I-cache

**Phase 0 (measure) — DONE (2026-07-13).** Hand-assembled hazard probe
`perf/m7/haz_probe_*.kla` (generated by `gen_probe.py`; reproduces the six
`perf_baseline.c` kernels, `clear PERF_CTRL → run → snapshot → print` of
`0xF00D_00B0..E8`). Validated emulator → tb_soc (pipeline reads the M6 counters
correctly, IF_MISS<cycles, clean HALT) → **board (M6 bitstream)**. Full numbers +
analysis in `perf/m7/RESULTS.md`; machine-readable `perf/m7/haz_attribution_board.csv`.
Cross-check: probe branchy CPI 4.41 ≈ `baseline_m6_pipeline.csv` 4.402.

**What the measurement decided (do not re-estimate — this is silicon):**
- **IF_MISS dominates — 33–73 % of cycles in 5 of 6 kernels**, and it is large even
  with *zero* memory traffic: `alu` is 93 % straight-line (1 branch / 15 instrs),
  `MEM_WAIT`=0, yet **60 % IF_MISS**; `branchy` `MEM_WAIT`=0, **73 % IF_MISS**. The
  2-dword IFB refilled through the shared, MEM-priority port cannot sustain 1 instr/
  cycle even on cache-resident straight-line code. ⇒ **the 1-cycle-hit I-cache with
  its own fetch port is the biggest single lever, and it is what helps the compute
  kernels that a split port alone would not.**
- **Memory kernels pay IF_MISS *and* MEM_WAIT and they serialize** on the one port
  (mem_stream 43.9+38.6, ptr_chase 32.6+54.4, calls_fib 53.7+36.6). ⇒ **the split
  I/D port lets fetch run from the I-cache during data misses** — pairs with (does
  not replace) the cache. The I-cache *is* the second port.
- **Out of M7 scope (measured, so we don't over-claim):** ptr_chase DATA 62.5 /
  LOADUSE 56.0 (dependent load-use → M8 scheduling); muldiv MULDIV 14.3 (inherent EX);
  calls_fib SP 7.3; BRFLUSH 1.6–2.6 everywhere (taken-redirect refill = the residual
  IF_MISS after a cache → predictor / back-edge layout, later).

**Design (per §5.3 of `PIPELINE_ICACHE_HANDOFF.md`, tuned to the measurement):**
- **Direct-mapped I-cache, own read port**, 1-cycle hit (register the tag-compare/
  hit-mux if it doesn't close — a hit cycle still beats the shared-port refill). Phys
  addr 27-bit (`0x0–0x07FF_FFFF`). Start 8 KB / 32 B lines (256 sets): `offset[4:0]`,
  `index[12:5]`, `tag[26:13]`+valid. Line = a multiple of the DDR burst so a miss is
  one 32 B fill (reuse the 4.1 dual-BL8 path).
- **Fetch delivers a dword/cycle on a hit** into the existing IFB window (keep the
  LEN-driven assembly + the register-bounded redirect/flush interface intact so the
  M6 branch-resolution drives it unchanged).
- **Second port**: fetch gets its own `membus`-shape master into the arbiter; MEM
  keeps the data port. On an I-cache miss the two contend only for that fill.
- **Coherence (§6)**: `fence.i`-style MMIO invalidate-all for the new I-cache, plus
  the existing IFB store-invalidate must also snoop I-cache lines — LLEXT/netboot/SMC
  deposit code as data then jump. This is the M5c SMC squash's cache-side partner;
  regression must stay UART-bit-identical (SMC directed test + queens self-mod paths).

**Sub-milestones (each: golden-trace `run_m5a`+`run_m5c`+`run_m5d_soc`, then board):**
- **M7a** — I-cache module + own fetch port + arbiter entry; hit/miss counters
  (`ICACHE_HIT/MISS` MMIO). Exit: regressions bit-identical; IF_MISS drops on
  alu/branchy; re-run `haz_probe` on board — IF_MISS % must fall.
- **M7b** — `fence.i` invalidate-all + I-cache store-snoop; SMC/loader coherence.
  Exit: SMC directed test + full regression bit-identical.
- **M7c** — re-measure CPI (`baseline_m7_pipeline.csv`) + re-run the hazard probe;
  update `RESULTS.md` with the post-M7 attribution (IF_MISS/MEM_WAIT overlap gone).
  Firm timing (the M6 SHA hot path already needs a respin before QSPI).

# KlaussCPU hardware proposals for compiled code (measured)

For the RTL session (klaussCPU repo).  Written 2026-10-28 by the klausscpu-llvm
session (phase 5, step `hw-report`), from board measurements and emulator
traces of compiled code.  It follows `ISA_V3_PROPOSAL.md` (the short forms,
FBR, ENTER/LEAVE and the W ops, all built since).

What was measured:

- The board (Nexys A7, 100 MHz) as it is today, klaussCPU `db0f315`.
- Code from klausscpu-llvm `cbc0e107b29b`, ABI v2, with the klausscpu-runtime
  `558db84` harness and picolibc.
- Every board figure was run twice.  The two runs were identical, except
  CoreMark, whose cycles, IFMISS and MEMWAIT differed by at most 6 in
  2·10⁹.

The ranked list is in §1.  Each proposal gives its evidence, the estimated gain
(count × measured cycles per event), its ISA/encoding impact and the compiler
work it would need.

Status (2026-10-29): klaussCPU `9ad6614` did §3 items 1 and 5, and fixed a
third squash bug (§3 item 6).  The toolchain now requires `9ad6614` or later.
The measurements below are from `db0f315`; `9ad6614` did not change the
timing of normal code.

## 0. Board ≠ emulator divergences: none

The harness step ran Csmith on the board and on the emulator with the same
ELF (`tools/validate/csmith_board.sh`, scratch `p5/board-csmith/`):

- seeds 120000-120499, -O1 and -O2, 100 of them with `--float`;
- 858 runs, all three checksums equal (host gcc, emulator, board) in every
  run;
- no board ≠ emulator case, so nothing to reduce.

Csmith code uses no LCD, MMIO or interrupts, and the compiler then never put
a divide behind an FBR, so it could not reach the RTL bug in §3.1 (fixed
since, in klaussCPU `9ad6614`).

One tooling trap turned up while tracing.  It is a stale binary, not an RTL
or emulator bug:

- `~/.cargo/bin/klausscc` (built 2026-10-16) is the `klausscc` on `PATH`.  It
  predates klausscc `f74fe4a` ("Match the RTL's flag writes for logic, ...")
  and still writes Z on reg-reg `ORR`/`XORR`.
- Embench `qrduino` never ends on it.  `cmprvw r12, 6 ; orr ; jmperel` never
  sees EQ.  Repro: `p5/hw-report/emubug/t.s`.
- The validation harness uses `klausscc/target/release/klausscc`, which is
  correct, so no harness result is affected.
- `run-benchmarks.sh` and a bare `klausscc --emulate` do use the stale
  binary.  Reinstall it (`cargo install --path .` in klausscc).

## 1. Ranked proposals

The ranking is by measured cycle gain.  The CoreMark column is -O2, 2000
iterations, board.  Embench is all 19 benchmarks at -O2: summed cycles, plus
the geomean speed-up, which is the Embench score ratio.  Dhrystone is -O1
`-fno-builtin`.  Gains overlap and do not add up: 1a contains 1b and 1c, and
2a contains 2b.

| # | Proposal | CoreMark | Embench (geomean) | Dhrystone | RTL size / risk | ISA change |
|---|---|---:|---:|---:|---|---|
| 1a | D-cache hits that do not freeze the pipeline (pipelined hit, ~1 cycle MEM occupancy) | −24.6% | −32.8% (1.46×) | −33.2% | large: cache/MEM redesign | none |
| 1b | load hit 4 → 3 cycles (M10b stage B, already built and parked for timing) | −6.6% | −7.1% (1.07×) | −6.5% | medium: timing closure | none |
| 1c | 1-entry store buffer (posted stores, CALL push) | −3.5% | −3.9% (1.04×) | −7.9% | small–medium | none |
| 2a | fetch-stage branch prediction: direction table + return stack; direct targets come from word 0 | −15.9% | −11.4% (1.15×) | −20.4% | large: fetch path timing | none |
| 2b | FBR resolved in EX instead of MEM | −2.2% | −1.5% | −3.9% | small logic, tight timing (+0.41 ns noted in ISA_V3_IMPL) | none |
| 3 | front end: no bubble at a 16-byte fetch-line crossing (then the compiler drops loop-alignment NOPs) | ≤ −2.6% | ≤ −7.8% (1.09×) | ≤ −2.8% | medium | none |
| 4 | non-blocking multiplier (no EX hold, scoreboard the result) | −5.1% | −3.1% (edn −24.9%) | −0.7% | medium | none |
| 5 | EX→EX forwarding of 1-cycle ALU results | −2.1% | −3.2% (crc32 −8.3%) | −2.1% | timing-bound (WNS −0.75 ns when tried in M6) | none |
| 6 | FBR with a W compare and a 32-bit immediate | −1.1% | −0.9% (slre −4.0%) | −0.3% | small | class D PRED 5–7 and LEN=10 (both trap today) |
| 7 | interrupt disable/enable instructions (+ optional atomic swap/CAS) | 0 (no atomics) | 0 | 0 | small | class B OP 6/7; class 7 SGN=1 |
| 8 | misaligned 16/32-bit loads and stores: do them, or trap | 0 | 0 | 0 | small (trap) / medium (support) | none |
| 9 | missing short forms: CMPRVW, MULV, LEAPC simm16 | size −0.5% / −0.4% / −1.7% of corpus `.text` | | | small (decode) | free combinations, §6 |

Items 7 and 8 do not show in these benchmarks; their gains are per event
(§2.7, §2.8).

Recommended order (gain per unit of work):

1. **1b** and **1c**: the memory hit path is the largest single cost, see §1.1.
2. The remaining **§3 correctness items** (3 and 4; 1, 5 and 6 are done in
   klaussCPU `9ad6614`).
3. **4** and **6**: contained datapath and decode work.
4. **2a** and **1a**: the large redesigns that remove most of the remaining
   cycles.

Measured and not proposed (§5): a static backward-taken predictor in ID,
select/CMOV, and more SP bypassing.

### 1.1 Where the cycles go

This is the board's cycle stack.  The hazard counters overlap, so the rows
use the non-overlapping parts:

- **D-cache hit freeze**: `PERF_MEM_WAIT`.
- **load-use beyond the freeze**: 2 cycles per load whose result the next
  instruction uses, 1 when one instruction is in between (measured, §2.2).
  Pairs are counted in the traces.
- **ALU back-to-back**: 1 cycle per dependent pair, counted in the traces.
- **MUL/DIV, flag, SP**: their counters.
- **redirect squash**: 2 × `PERF_BRANCH_FLUSH`.
- **fetch empty**: `PERF_IF_MISS`.

| cycles go to | CoreMark -O2 | Dhrystone -O1 -fno-builtin | Embench -O2 (19 summed) |
|---|---:|---:|---:|
| issue (1 per instruction) | 33.9% | 27.3% | 34.1% |
| **D-cache hit freeze (MEMWAIT)** | **31.7%** | **38.4%** | **36.9%** |
| load-use beyond the freeze | 7.1% | 5.2% | 4.1% |
| ALU back-to-back | 2.1% | 2.1% | 3.2% |
| MUL/DIV EX hold | 5.3% | 0.7% | 4.8% |
| flag stall | 1.4% | 0.3% | 0.7% |
| SP serialisation | 0.0% | 0.0% | 0.1% |
| redirect squash (2 per redirect) | 7.4% | 9.0% | 4.9% |
| fetch empty (IFMISS) | 11.9% | 13.9% | 14.4% |
| residual | −0.7% | 3.0% | −3.0% |
| total cycles | 1,978,438,670 | 299,996,199 | 143,206,589 |

CPI is 2.95 for CoreMark -O2, 2.93 for Embench -O2 and 3.66 for Dhrystone
`-fno-builtin` (3.49 with the built-ins).

**Every D-cache access in these runs hits:** `CACHE_RD_MISSES` is 0 and
`WR_MISSES` is 0 or 11.  So the freeze row is entirely the hit path: 3.4–3.8
cycles per cache access.  On the default Dhrystone build it is 54% of all
cycles, because the inlined `strcpy` stores dominate.

## 2. Evidence

### 2.1 Method

- **Board counters around the timed region.**  Scratch copies of the
  CoreMark and Embench ports and of Dhrystone snapshot all 26 counters at
  the start and stop of the timed region and print an `HZ,` line
  (`p5/hw-report/root/baremetal/benchmarks/hazsnap.h`).  The counters cover
  `PERF_*` 0xF00D_0008…00E8 and the cache counters 0xF005_0040…0068.  The
  snapshot's own MMIO reads add about 150 cycles per run, under 0.01%.  The
  benchmarks' published cycle counts are read outside the snapshots.
- **Per-event costs** come from `ubench/`: 36 assembly loops, each unrolled
  4–8× inside a counted loop, minus an empty-loop baseline.
- **Dynamic counts** come from `klausscc --emulate --trace` (the current
  build) through `tools/pchist`, which records per-pc execution counts and
  non-sequential edges.  These are joined with `llvm-objdump` of the same
  ELF.  The CoreMark trace runs 20 iterations; Embench and Dhrystone use the
  board ELFs.
- **The dependency model** decodes rd/rs1/rs2 from word 0 as `f_decode_raw`
  does and counts sequentially executed producer→consumer pairs.  It
  reproduces the board counters:
  - FLAGS: 0.0415 vs 0.0407 stall cycles per instruction (CoreMark);
  - load-use cycles: 0.627 vs 0.609 per instruction;
  - DATA − LOADUSE: 0.148 vs 0.171.  The counter also counts cycles in which
    a dependent pair sits behind a frozen MEM; the model does not.
- **Gains** (`tools/gains.py`): event rate per traced instruction × board
  instructions × measured cycles per event.  The board's own counters are
  used directly where they count the event exactly: read hits, stores,
  calls, MEMWAIT, MULDIV, IFMISS.

### 2.2 Measured per-event costs (board, cycles)

| kernel (per unrolled body) | cycles | of which | reading |
|---|---:|---|---|
| independent 1-word ALU op | 1.25 | IFMISS 0.25 | fetch delivers 4 words then bubbles: one bubble per 16-byte line |
| dependent ALU op (`addr r8,r8,r9`) | 2.00 | DATA 1 | **1-cycle back-to-back stall** |
| ALU → MUL that reads it | 6.25 | DATA 1, MULDIV 3 | the Zephyr mandel case: the same 1-cycle stall |
| load, result unused | 5.25 | MEMWAIT 4.00 | **a load hit freezes MEM 4 cycles** |
| load ; use | 8.50 | MEMWAIT 4 + 2 | 2 load-use cycles beyond the freeze |
| load ; 1 independent ; use | 8.25 | MEMWAIT 4 + 1 | |
| load ; 2 independent ; use | 8.50 (4 instructions) | MEMWAIT 4 | no stall |
| `MEMGET32` aligned / spanning a dword | 5.25 / 5.25 | MEMWAIT 4 | the misaligned span path is free |
| store (8- or 64-bit) | 4.25 | MEMWAIT 3.00 | **a store hit freezes MEM 3 cycles** |
| store ; load | 9.25 | MEMWAIT 7 | no overlap |
| PUSH ; POP | 14.5 | MEMWAIT 7, SP stall | |
| ENTER 1 ; LEAVE | 14.5 | MEMWAIT 7, SP stall | |
| CALL leaf (`ret`) round trip | 16.9 | MEMWAIT 7, 2 redirects | |
| CALL + `enter 1` + 1 store + LEAVERET | 27.1 | MEMWAIT 13 | |
| MUL, independent work behind it | 4.25 per MUL; MUL + 4 ALU = 8.5 | MULDIV 3 | **EX is held; nothing overlaps** |
| MUL ; dependent ALU | 6.25 | MULDIV 3 + 1 | |
| taken FBR | 6.12 | 1 redirect, IFMISS 2.6 | **5.1 cycles more than not taken** |
| not-taken FBR | 1.25 | | |
| W compare + JMPcc, taken / not taken | 6.62 / 3.12 | FLAGS 1 | JMPcc redirects from EX, FBR from MEM |
| short JMP (to the next instruction) | 5.00 | 1 redirect, IFMISS 2.5 | a direct jump costs 4 extra cycles |
| 2-word SETR / 3-word SETR64 | 2.13 / 4.37 | IFMISS | one fetch cycle per word plus the line bubble |
| INT_MASK read (uncached MMIO) | 5.37 | MEMWAIT 4 | |
| irq lock + unlock (read INT_MASK, write 0, write back) | 16.5 | MEMWAIT 12 | |
| `__sync_fetch_and_add` loop / plain `volatile` increment loop | 57 / 19 per iteration | | **38 cycles per atomic RMW** |

### 2.3 Instruction mix

**Static**: `llvm-objdump` share of instructions.  The columns are:

- the bare-metal corpus: 39 ELFs, `.text` 1,045,484 B at -O2;
- CoreMark and Embench as built for the board;
- seven Zephyr images: hello, mandel, gui_lvgl, doom, irq_soak, llext_demo,
  ssh_shell; 437,149 instructions.

| category | corpus -O1 | corpus -O2 | CoreMark -O2 | Embench -O2 | Embench -Os | Zephyr |
|---|---:|---:|---:|---:|---:|---:|
| loads | 15.5% | 15.4% | 14.5% | 16.0% | 16.1% | 17.8% |
| stores | 11.0% | 11.5% | 9.3% | 10.6% | 10.3% | 12.6% |
| moves (SETR/SETRU/LEAPC/COPY) | 19.7% | 19.4% | 16.2% | 15.2% | 15.3% | 20.6% |
| FBR | 8.6% | 8.5% | 7.5% | 7.4% | 7.7% | 8.1% |
| JMPcc | 4.2% | 4.1% | 4.1% | 4.0% | 4.3% | 2.6% |
| W flag compares | 1.0% | 1.0% | 1.7% | 1.2% | 1.3% | 0.9% |
| calls | 3.9% | 4.0% | 3.0% | 2.6% | 2.4% | 6.2% |
| MUL | 1.6% | 1.6% | 3.1% | 2.4% | 2.4% | 1.3% |
| NOP (loop-alignment padding) | 3.3% | 3.1% | 3.9% | 2.5% | 1.6% | 1.5% |
| 2-/3-word instructions | 9.6% | 9.9% | 7.8% | 9.2% | 9.3% | 10.7% |

Of the 2-word instructions in the -O2 corpus, the commonest are:

| instruction | count |
|---|---:|
| `LEAPC` (PIC addresses) | 9370 |
| `SETR` | 2453 |
| `CMPRVW` | 1569 |
| `MULV` | 1469 |
| `ANDV` | 1274 |
| `ADDI` | 1058 |
| `CMPRV` | 1019 |
| `SETR64` (3 words) | 887 |

In the three largest Zephyr images (doom, gui_lvgl, ssh_shell) the top one is
`SETRU sym`, the absolute address of a symbol: 18,562 of the 42,588
multi-word instructions.

**Dynamic**: share of executed instructions.

| | CoreMark -O2 | Dhrystone -O1 -fno-builtin | Embench -O2 |
|---|---:|---:|---:|
| loads (incl. POP) | 19.3% | 23.4% | 18.4% |
| stores (incl. PUSH) | 4.7% | 14.4% | 8.1% |
| ALU, moves, shifts, bit, unary, boolean compares | 42.2% | 35.3% | 52.1% |
| flag compares (64-bit and W) | 4.7% | 1.3% | 2.5% |
| FBR | 15.0% | 20.6% | 8.9% |
| JMPcc | 5.5% | 1.5% | 2.8% |
| JMP / CALL / RET+LEAVERET | 0.9 / 0.5 / 0.5% | 1.0 / 0.5 / 0.5% | 1.2 / 0.8 / 0.8% |
| MUL | 4.5% | 0.5% | 3.6% |
| executed alignment NOPs | 2.2% | 1.0% | 0.4% |
| words fetched per instruction | 1.050 | 1.025 | 1.102 |
| taken FBR / taken JMPcc | 6.4% / 3.5% | 14.2% / 0.8% | 4.3% / 1.4% |

### 2.4 Proposal 1: the D-cache hit path

**Evidence.**

- Every load hit holds MEM for 4 extra cycles and every store hit for 3, and
  the whole pipeline freezes behind it (`mem_busy`, `pipeline_core.sv:1237`).
  The microbenchmarks give exactly 4.00 and 3.00 MEMWAIT cycles per access.
- CALL, RET, PUSH, POP, ENTER, LEAVE and LEAVERET go through the same port.
  A leaf CALL+RET costs 7 memory cycles; a call into a function with a frame
  and one spill costs 13.
- MEMWAIT is 31.7% of CoreMark, 36.9% of Embench and 38.4% of Dhrystone
  `-fno-builtin`, all with zero misses.
- Per benchmark it ranges from 0.5% (aha-mont64, register-only) to 52%
  (nettle-aes, statemate, matmult).

**Gain.**

- **1a**, a hit that does not freeze: −24.6% CoreMark, −32.8% Embench
  (geomean 1.46×), −33.2% Dhrystone.
  - It saves the MEMWAIT cycles, minus the load-use stalls that the freeze
    hides today and that would come back.  Those are 2 per directly
    dependent pair and 1 per pair with one instruction between: CoreMark
    0.56 M + 0.32 M pairs per 6.9 M traced instructions.
  - The largest per-benchmark gains: nettle-aes −51.6%, statemate −51.2%,
    matmult −45.2%, ud −41.7%.
- **1b**, M10b stage B (load hit 4→3): 1 cycle per read hit
  (`CACHE_RD_HITS`).  −6.6% CoreMark (131.1 M hits), −7.1% Embench, −6.5%
  Dhrystone, nsichneu −14.0%.
  - Stage B is built and simulation-verified on `feat/m10b-fused`.  It was
    parked at WNS −0.115 ns on paths outside its own logic
    (`PIPELINE_M10_DCACHE_LATENCY.md`).
  - These numbers say it is worth the timing work.
- **1c**, a 1-entry store buffer: −3.5% CoreMark, −3.9% Embench, −7.9%
  Dhrystone (statemate −7.6%).
  - A store retires from MEM at once; the next memory access waits only for
    what is left of the drain.
  - Each store saves min(3, instructions until the next memory access in its
    block, counted as 3 at a branch).  In the traces that averages 0–2
    cycles, because a store is often followed directly by a memory access.
  - Each CALL push saves 3, since the redirect refill follows it.
  - A buffer that lets a later hit proceed while the store drains
    (hit-under-store) would save the full 3 per store: −5.3% CoreMark,
    −9.4% Embench, −12.2% Dhrystone.

**ISA.** None.

**Compiler work.**

- None for correctness.
- The schedule model already treats a load as latency 3 with no freeze
  (`KlaussCPUSchedule.td`); only its comment on the freeze changes.
- Re-measure the inline-`memcpy` grouping (`MaxGluedStoresPerMemcpy`, 4 loads
  then 4 stores) and the frame-folding and `KlaussCPUConstAddrBase` size/speed
  trades, all of which were tuned on today's port.

### 2.5 Proposal 2: taken-branch cost

**Evidence.**

- Extra cycles per taken transfer, over a not-taken one:

  | transfer | extra cycles |
  |---|---:|
  | FBR (redirects from MEM, squashes 2) | 5.1 |
  | JMPcc (redirects from EX) | 3.5 |
  | short JMP | 4.0 |
  | CALL + RET | ≈ 8, beyond their 7 memory cycles |

- Redirects per 1000 instructions: 109 in CoreMark, 71 in Embench, 164 in
  Dhrystone.
- Squash plus refill is 19% of CoreMark cycles (7.4% squash + most of the
  11.9% IFMISS).
- Taken branches per executed instruction:

  | | taken FBR | taken JMPcc | JMP | CALL | RET |
  |---|---:|---:|---:|---:|---:|
  | CoreMark | 6.4% | 3.5% | 0.9% | 0.45% | 0.45% |
  | Embench | 4.3% | 1.4% | 1.2% | 0.8% | 0.8% |

**Gain.**

- **2a**, a fetch-stage predictor removing 90% of the taken penalties:
  −15.9% CoreMark, −11.4% Embench (1.15×; st −22.2%, aha-mont64 −19.8%),
  −20.4% Dhrystone.  The formula is 0.9 × (5.1·FBRtaken + 3.5·JMPcc_taken +
  4·(JMP+CALL+RET)).
- **2b**, FBR resolved in EX: 1 cycle per taken FBR.  That is the measured
  gap: an FBR taken costs 1.6 cycles more than a taken JMPcc.  −2.2%
  CoreMark, −1.5% Embench, −3.9% Dhrystone.
- A return-address stack alone saves 3 cycles per RET/LEAVERET: −0.5%
  CoreMark, −0.8% Embench (crc32 −5.2%).

**ISA.** None.  Since ISA v3 every short direct transfer carries its
displacement in word 0:

- JMP, JMPcc and CALL: simm18 `[17:0]`;
- FBR: simm13 `[20:8]`.

So a predecoder in IF can compute the target from the fetched word alone.
The predictor needs:

- a direction table (e.g. 2-bit counters indexed by PC) for FBR and JMPcc;
- a 2–4 entry return stack for RET and LEAVERET;
- no target storage for direct branches.

Only JMPR/CALLR (0.7% of instructions statically, ~0 dynamically) would need
a BTB.

**Compiler work.**

- Small.
- `MispredictPenalty` and the TTI branch costs.
- The jump-table threshold (12 cases at 40% density was the break-even with
  today's 4–5 cycle taken cost) comes down.
- FuseCmpBr's choice between FBR and CMP+JMPcc for likely-taken branches.
- The loop-alignment policy.

### 2.6 Proposals 3–6

**3: front-end bubble.**

- Evidence:
  - Straight-line 1-word code issues at 1.25 cycles per instruction: one
    `IF_MISS` bubble per 16-byte fetch line (alu_indep and setr_short both
    0.25 per word).
  - 2-word instructions take 2.13 cycles; SETR64 takes 4.37.
  - IFMISS not explained by redirects (IFMISS − 2.4 × (redirects + returns),
    where 2.4 is the measured refill) is 2.6% of CoreMark, 7.8% of Embench
    and 2.8% of Dhrystone.  nsichneu, 16 KB of straight-line code, reaches
    28.3%.
  - This remainder may include I-cache refills: there is no I-cache miss
    counter.  **Adding one is a cheap request**; it would split the number.
  - The compiler pads loop heads to 16 bytes (`-klausscpu-pref-loop-align`)
    to avoid the bubble.  That padding is:
    - 3.1% of corpus instructions (2.8% of `.text`);
    - 2.2% of the instructions executed in CoreMark;
    - and a 4-byte shift of the queens loop cost +1.43% cycles in phase 4.
- Gain: up to the IFMISS figures above, plus −2.8% corpus `.text` and the
  executed NOPs once the padding can go.
- ISA: none.
- Compiler: set `-klausscpu-pref-loop-align` to 4 after re-measuring.

**4: non-blocking multiplier.**

- Evidence:
  - A MUL holds EX for 3 extra cycles whatever follows: MUL + 4 independent
    ALU ops = 8.5 cycles, with no overlap.
  - The DSP chain (`mul_a_q` → `mul_pipe1` → `mul_pipe2`,
    `pipeline_core.sv` ~897–905) is already pipelined; only the EX hold
    (`ex_busy`, :923) serialises it.
  - MULDIV is 5.3% of CoreMark (core_matrix), 26% of edn, 10.6% of ud and
    4.8% of Embench.
- Gain: the MULDIV cycles minus 3 for each MUL whose result is read by the
  next instruction.  −5.1% CoreMark, −3.1% Embench (edn −24.9%, ud −9.5%),
  −0.7% Dhrystone.  The divider can keep its hold: DIV/MOD is ≤0.1% of
  executed instructions.
- ISA: none.  The RTL lets the next instruction enter EX while `mul_cnt`
  counts, and interlocks readers of the MUL's rd until it completes.
- Compiler:
  - `WriteMul` `ReleaseAtCycles` 4 → 1;
  - drop the post-RA EX-busy recognizer (`-klausscpu-postra-ex-busy`);
  - re-tune `-klausscpu-mul-imm-shift-add`.

**5: EX→EX forwarding.**

- Evidence:
  - A dependent ALU op costs 2.00 cycles against 1.25 independent.
  - Dependent ALU-producer pairs are 6.3% of executed CoreMark instructions,
    9.4% of Embench and 7.8% of Dhrystone.  Dhrystone's are mostly ALU→FBR
    in `strcpy`/`strcmp` loops.
  - The DATA−LOADUSE counter is larger (5.8% of CoreMark cycles) because it
    also counts cycles behind a frozen MEM, which forwarding would not
    recover.
- Gain: −2.1% CoreMark, −3.2% Embench (crc32 −8.3%, aha-mont64 −7.4%), −2.1%
  Dhrystone.
- Cost: M6 measured the full zero-bubble `exo_result` feedback at
  WNS −0.75 ns.  A partial bypass is cheaper, and it covers the mandel loop
  (ALU → MUL):
  - only from ADD/SUB/logic/MOV;
  - or only into the FBR comparator and the address adder.
- Compiler:
  - `WriteALU` and `WriteFlags` latency 2 → 1;
  - `KlaussCPUEarlyIVInc` and the pre-RA compare/branch pairing lose their
    reason to exist;
  - the post-RA EX-hold hazard (1697618593b9), which already fixed the
    phase-4 mandel slowdown in software (1840 -> 2059 kiter/s at
    milestone 5), would cover only MUL/DIV holds.

**6: FBR with a W compare and a wide immediate.**

- Evidence:
  - Executed compare+JMPcc pairs that did not fuse: 3.3% of CoreMark
    instructions (55 k of them W) and 2.2% of Embench.
  - Why they did not fuse:

    | | immediate outside simm4 | W compare |
    |---|---:|---:|
    | CoreMark | 45% | 15% |
    | nettle-aes | 97% | |
    | slre | 69% | 22% |
    | huffbench | | 69% |
    | sglib | | 48% |

  - Static, corpus -O2: 1527 W-compare+JMPcc and 6086 other unfused pairs,
    against 20,230 FBRs.
  - Measured: compare+JMPcc costs 3.12 cycles not taken / 6.62 taken; FBR
    costs 1.25 / 6.12.
- Gain: −1.1% CoreMark, −0.9% Embench (slre −4.0%, minver −2.7%), −0.3%
  Dhrystone.  The fused branch also saves 1–2 words per pair.
- ISA, two free combinations:
  1. Class D `PRED` 5, 6, 7 (they trap today) as EQW, LTW and ULTW, in the
     existing 1-word layout.  With INV and swapped operands these give all
     ten W conditions: LE = !(b<a), GT = b<a.
  2. Class D `LEN=10` (traps today): an FBR whose right-hand side is
     sext(imm32) from word 1 (`IMM=1`, `[3:0]=0`), `PRED` 0–7, so 64-bit or
     W.
  - The W path is D2's operand shift at dispatch (`f_wsh`).  The imm32 is
    `id_var1`, as for every 2-word op.
- Compiler:
  - FuseCmpBr fuses W compares and any simm32 constant;
  - drop the loop-constant register trick (`-klausscpu-fuse-loop-const`),
    which frees a register in such loops;
  - new instruction definitions, disassembler, klausscc.

### 2.7 Proposal 7: interrupt disable and atomics (simplicity, RTOS cost)

**Evidence.**

- There is no atomic or interrupt-disable instruction.  The backend uses
  "forced atomics": every RMW and cmpxchg is a `__sync_*` libcall that
  saves, zeroes and restores `INT_MASK` through uncached MMIO
  (`src/atomic.c`).
- Measured: 38 cycles per `__sync_fetch_and_add` beyond a plain increment.
  A lock/unlock pair is 16.5 cycles (one MMIO read and two writes).
- Static sites:
  - 121–132 `__sync_*` call sites in the corpus;
  - in each Zephyr app image, 160–180 `INT_MASK` accesses and 74 calls to
    `atomic_*` (Zephyr uses `ATOMIC_OPERATIONS_C`, a spinlock, which is an
    irq lock);
  - the irq_soak stress test executes 217,618 `INT_MASK` accesses per 20 M
    instructions.
- The benchmarks execute none, so there is no benchmark gain.
- Nothing makes these atomic against core 2 (AMP).

**Proposal.**

- (a) `DI rd` (rd = old enable; disable) and `EI rs1` (restore) on a
  core-internal interrupt-enable bit, saved and restored with the flags in
  the interrupt context word.  Encoding: class B system `OP` 6/7 (free,
  `[21:16]`), LEN=01.
- (b) Optionally, `SWAP` and `CAS` in the reserved store space, class 7
  `SGN=1` (traps today):
  - SWAP: rd ↔ [rs1];
  - CAS: compare with rs2, store rd, old value to rd;
  - atomic against interrupts, as forced atomics are today.

**Gain.**

- A critical section drops from 16.5 to ~2 cycles.
- An atomic RMW done inline (DI, load, op, store, EI) costs about the plain
  loop's 19 cycles per iteration plus 2, against 57 today.
- `atomic.c`, Zephyr `arch_irq_lock` and the FreeRTOS critical sections
  become two instructions.

**Compiler work.**

- (a) needs none at first: only the runtime and RTOS ports change.
- Later, an inline pseudo for atomics expanded after register allocation,
  which retires the forced-atomics libcalls.
- (b) native `xchg`/`cmpxchg` lowering.

### 2.8 Proposal 8: misaligned accesses (simplicity, correctness)

**Evidence.**

- The RTL silently aligns sub-word addresses down: loads at
  `pipeline_core.sv:1187-1188`, stores at :1106-1112.  Only `MEMGET32` (mode
  00) handles misalignment, and it costs nothing: 5.25 cycles aligned and
  spanning.
- The backend therefore splits every under-aligned 16/32-bit store into
  bytes, and every 16-bit load.
  - An i32 store becomes 4 byte stores + 3 shifts: ~19 cycles against 4.
  - An i16 load becomes 2 byte loads + shift + OR: ~12 cycles against 5.
- A pointer that claims natural alignment but is misaligned silently reads
  or writes the wrong address.  This is a Known Issue in the backend's
  `CLAUDE.md`.
- In the IR, the under-aligned accesses are:
  - lwIP: 6.1% of loads and 5.5% of stores (packed protocol headers: 90 i16
    and 47 i32 loads, 55 i16 and 17 i32 stores);
  - the corpus programs: 132 loads and 31 stores.

**Proposal.**

- Extend the `MEMGET32` span path to the other 16/32-bit loads, and to
  stores as two byte-enabled writes when the access crosses a dword.
- Or, at least, trap a misaligned 16/32-bit access, so that the silent
  wrong address becomes a crash dump.

**ISA.** None (behaviour of existing encodings).

**Compiler work.**

- Support: `allowsMisalignedMemoryAccesses` reports these accesses as fast,
  and the splits go.
- Trap: nothing.

### 2.9 Proposal 9: missing short forms (code size)

The short forms that are missing, by static count in the -O2 corpus
(`.text` 1,045,484 B):

| form | count | encoding (free today) | saving |
|---|---:|---|---:|
| `CMPRVW` with a simm8 immediate (W compares have no short form) | 1351; 1880 in three Zephyr apps | class 3 `LEN=01 SGN=1 B=0 INV=1` (INV on a flag compare traps today), imm8 `[19:12]` | −0.52% |
| `MULV` with a simm8 immediate | 996 | class A `OP=3` (free), LEN=01, imm8 `[19:12]`, SGN `[23]` | −0.38% |
| `LEAPC` within ±32 KiB (PIC) | 4447 of 9370 | class 2 `LEN=01 OP=14 F=1` (traps today), simm16 = `{[19:12],[7:0]}` | −1.70% |

The compiler needs CompressPats, plus a 16-bit PC-relative fixup and
relaxation for LEAPC.

## 3. Correctness items (do these first; they are cheap)

1. **The LCD strobe and the divider start are not squashed after a taken
   FBR or an `sq_ex` squash.**  **Fixed in klaussCPU `9ad6614`.**
   - As reported (`db0f315`): `pipeline_core.sv:1392` was `assign lcd_dv =
     ex_is_lcd && lcd_ready;`, and `:1582` started the divider (`MD_IDLE: if
     (ex_is_div && ex_b != 64'd0)`) on the edge a taken FBR or an `sq_ex`
     squash killed the op.  A squashed LCD write still sent its byte.  A
     squashed DIV/MOD left the divider running on its dead operands, and the
     next divide within about 64 cycles got that quotient (board-tested).
   - The fix: `lcd_dv = ex_is_lcd && lcd_ready && !mem_busy && !fbr_taken &&
     !sq_ex` (`:1396`), gated like LCDRST.  `!mem_busy` also fixes a double
     strobe that was not reported: an LCD write held in EX behind a busy
     memory op sent its byte twice.  Both squashes now reset `div_ph` to
     `MD_IDLE` (`:2052`, `:2092`), as they reset `dly_on`.  The LCD part is
     reasoned from the RTL (not observable from software).  The divider part
     is board-tested by klausscc's `tests/fixtures/hw_div_fbr.kla`: `0x8E`
     (the squashed 1000 / 7) on every pass before, `0x21` (100 / 3) now.
   - The compiler: until klausscpu-llvm `e65c3d2fb8e7`, FuseCmpBr refused to
     fuse a compare and branch in front of a divide or LCD write
     (`ActsInEXShadow`), and the assembler warned about that shape.  It could
     not work around the SMC / I-cache-invalidate squash, which could repeat
     an LCD byte.  Both are gone: a branch fuses whatever follows it, and
     the toolchain supports only `9ad6614` and later, with no subtarget
     feature or option for older bitstreams (`CLAUDE.md`).  Everything that
     runs on the board is rebuilt.  The runtime's `test_divshadow` checks,
     on the board, a fused branch directly in front of DIVR, DIVUR, MODR and
     MODUR, taken and not taken, with another divide at the target.
2. **Keep CALL, RET and JMP flag-free.**  The MachineOutliner (on at -Oz)
   relies on it (`fmask = 0` for `U_CALL`/`U_RET`/`U_JMP` today).  No change
   requested.
3. **Misaligned sub-word accesses**: trap them if proposal 8 is not done.
4. **The crash dump cannot show the call chain.**  It holds R0–R15 and the
   four stack words at SP (S0–S3; `CRASH_DUMP.md`).  The ABI keeps a frame
   record at R15: [R15] is the caller's R15 and [R15+8] the return address.
   Dumping [R15], [R15+8] and the next one or two records would give a
   backtrace without a debugger.  Today the caller-of-caller comes only from
   the fetch trace (phase-4 milestone, item 4).
5. **Docs.**  **Applied in klaussCPU `9ad6614`.**
   - `ISA_ENCODING_V2_MAP.md` §4 note 7 said the field-driven forms were not
     implemented, which contradicted §5, the RTL and the backend.  It now
     says they are (the corrected paragraph this session supplied).
   - Note 8, §2 and the §3 compare rows said CMP writes E/L/U and leaves Z
     alone.  Since the flag unification CMP is SUB without writeback (it
     writes Z/S/C/V, and E = Z); they now say so.
6. **An SMC squash dropped a busy multicycle op in EX.**  Not reported from
   here; found and **fixed in klaussCPU `9ad6614`**, recorded for
   completeness.  A store that hit the code of the instruction in ID (the
   SMC squash, `!sq_ex`) cleared EX and refetched from `id_pc`.  Normally
   the EX op had just advanced into MEM, but a busy DIV/MUL, DELAY or LCD
   wait had not, so it was dropped and never executed.  EX is now cleared
   only when it is squashed itself or has advanced (`if (sq_ex || !ex_busy)
   ex_valid <= 0`), and `dly_on`/`div_ph` are reset only for an EX squash.
   Compiled code keeps `.text` and `.data` apart and never hit it, so no
   compiler change follows.  Board test:
   klausscc's `tests/fixtures/hw_div_smc.kla` (`0x55`, the DIV's result
   never written, before; `0x8E` now).

## 4. Free encoding space (from `f_decode_raw`, klaussCPU `db0f315`)

| where | free today (traps) | used above by |
|---|---|---|
| classes 0x0, 0xE, 0xF | everything | — |
| class D (FBR) | `PRED` 5–7; `LEN` 10/11 | proposal 6 |
| class 1 ALU reg-reg | `OP` 11–15; `F=1` on AND/OR/XOR/MIN/MAX | — |
| class 2 ALU imm | `OP` 7–13; `F=1` on AND/OR/XOR/LEA/MOV | 9 (LEA F=1) |
| class 3 compare | flag compare (B=0) with `PRED≠0` or `INV=1`; boolean `PRED` 5–7 | 9 (short CMPRVW) |
| class 4 shift/bit | LEN=01 `OP` 7, 12–15; LEN=10 `OP` 14/15 | — |
| class 5 unary | `OP` 11, 13 (no SETF exists) | — |
| class 6/7 load/store | `A=1` below 64 bits; **class 7 `SGN=1`** (half the store space) | 7b |
| class 8 branch | `COND` 10–15; INV of always | — |
| class 9 stack | `OP` 11–15; LEN=01 `OP` 1/5 (short PUSHV/ADDSP) | — |
| class A mul/div | `OP=3` | 9 (short MULV) |
| class B system | `OP` `[21:16]` 6–63 (and `[25:22]` ≠ 0) | 7a (DI/EI) |
| class C I/O | `OP=3` | — |

## 5. Measured and not proposed

- **Static backward-taken prediction with a redirect in ID** (targets from
  word 0):
  - It saves 2 cycles per taken backward FBR and 1 per taken JMPcc, JMP or
    CALL.
  - It costs the full redirect on every backward branch that is not taken.
  - Net: −1.8% CoreMark, +0.1% Embench (worse; aha-mont64 +4.4%).  A real
    direction predictor (2a) is needed.
- **Select / CMOV / CZERO.**  Forward branches over 1–2 ALU or move
  instructions are 0.17% of executed CoreMark instructions and 0.63% of
  Embench; at ~5 cycles per taken one that is ≤0.7%.  The backend already
  shares one compare/branch across a group of selects.
- **SP bypass / SP-relative addressing.**  The SP counter is 0.0% of
  CoreMark, 0.1% of Embench and 1.7% of calls_fib.  Frameless leaves already
  exist, so freeing R15 was not measured.
- **Faster divider.**  DIV/MOD is ≤0.1% of executed instructions here.

## 6. Reproducing

All files are in the phase-5 session's scratch directory `p5/hw-report/`
and are not committed.  The tools are short and can be re-created from this
description.

| file | what it is |
|---|---|
| `board/run{1,2}/*.out` | board captures with the `HZ,` lines |
| `board_table.md` | per-program counter table |
| `ubench/` | `kernels.S`, `ubench.c`, `board{1,2}.out` |
| `traces/*.hist` | per-pc counts and edges |
| `dyn.json` | dynamic statistics |
| `static_mix.md` | static mix |
| `gains.md` | per-program gains |
| `tools/` | `pchist.c`, `kdis.py`, `static_mix.py`, `dyn.py`, `gains.py`, `board_table.py` |
| `root/` | the instrumented build tree: symlinks to the runtime plus a copy of `baremetal/` |

The ELFs are in `elf/`.  Build and run them with:

```
KLAUSSCPU_LLVM_BIN=<llvm>/build/bin make -C root/baremetal coremark embench OPT=-O2
flock <board.lock> klausscc -i <elf> -s -m
```

### Appendix: per-program gains (% of board cycles)

| program | board cycles | 1c store buf | 1b hit 4→3 | 1a pipelined hit | 2b FBR in EX | 2a predictor | 5 EX→EX | 4 MUL | 3 front end | 6 FBR W/imm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| CoreMark -O2 | 1978438670 | 3.5 | 6.6 | 24.6 | 2.2 | 15.9 | 2.1 | 5.1 | 2.6 | 1.1 |
| Dhrystone -O1 nobuiltin | 299996199 | 7.9 | 6.5 | 33.2 | 3.9 | 20.4 | 2.1 | 0.7 | 2.8 | 0.3 |
| aha-mont64 | 3368970 | 0.0 | 0.1 | 0.0 | 2.7 | 19.8 | 7.4 | 0.9 | 3.1 | 0.3 |
| crc32 | 9886378 | 5.3 | 5.3 | 24.5 | 0.5 | 14.9 | 8.3 | 0.1 | 10.3 | 0.0 |
| cubic | 5935525 | 3.1 | 2.0 | 8.8 | 1.6 | 18.4 | 3.5 | 3.7 | 18.4 | 2.5 |
| edn | 10112321 | 1.5 | 9.3 | 39.4 | 1.4 | 7.3 | 0.8 | 24.9 | 2.5 | 0.2 |
| huffbench | 7703157 | 5.3 | 5.6 | 28.5 | 3.3 | 18.7 | 3.2 | 0.0 | 0.3 | 0.3 |
| matmult-int | 12052094 | 2.9 | 8.6 | 45.2 | 0.8 | 4.1 | 3.0 | 0.3 | 0.6 | 0.0 |
| minver | 6228379 | 5.1 | 4.2 | 24.5 | 1.5 | 15.8 | 5.3 | 1.8 | 14.4 | 2.7 |
| nbody | 4286878 | 2.3 | 2.4 | 7.1 | 1.5 | 17.7 | 2.8 | 2.6 | 19.8 | 2.6 |
| nettle-aes | 9401343 | 2.0 | 11.8 | 51.6 | 0.5 | 2.7 | 0.5 | 0.6 | 5.0 | 0.2 |
| nettle-sha256 | 7577533 | 6.4 | 7.1 | 39.8 | 0.4 | 2.6 | 0.8 | 0.0 | 6.3 | 0.1 |
| nsichneu | 11137504 | 0.0 | 14.0 | 33.7 | 1.4 | 6.7 | 0.1 | 0.0 | 28.3 | 0.1 |
| picojpeg | 8998235 | 6.0 | 5.5 | 35.1 | 1.9 | 13.0 | 6.3 | 2.8 | 4.2 | 0.5 |
| qrduino | 7311752 | 2.0 | 6.8 | 25.4 | 2.0 | 13.3 | 5.9 | 0.7 | 4.7 | 0.1 |
| sglib-combined | 8601436 | 4.7 | 7.6 | 33.4 | 2.1 | 15.2 | 3.4 | 0.4 | 2.0 | 1.1 |
| slre | 9570465 | 5.4 | 7.3 | 37.5 | 1.0 | 14.8 | 1.9 | 0.0 | 2.9 | 4.0 |
| st | 4584257 | 3.2 | 1.4 | 8.8 | 2.8 | 22.2 | 5.7 | 4.0 | 13.5 | 2.0 |
| statemate | 5432969 | 7.6 | 7.1 | 51.2 | 1.3 | 8.5 | 1.1 | 0.0 | 10.3 | 0.1 |
| ud | 8410652 | 5.8 | 7.9 | 41.7 | 1.4 | 9.6 | 0.9 | 9.5 | 4.2 | 0.1 |
| wikisort | 2606741 | 6.0 | 5.1 | 32.7 | 1.8 | 14.3 | 4.6 | 2.1 | 4.7 | 1.9 |

# Response to HARDWARE_PROPOSALS.md (RTL side)

Branch `hw-proposals`, 2026-10-31. Answers `HARDWARE_PROPOSALS.md` (copied
from klausscpu-llvm `main`, 2026-10-29). Written by an unattended session
**without Vivado**: everything here is simulation-verified, and **nothing is
timing-verified or board-verified yet**. Every change has its own build switch
(`localparam bit EN_*` at the top of `pipeline_core.sv`), so a timing miss can
be bisected one feature at a time.

## 1. Status

| # | Proposal | State | Switch (default) | Measured (full-SoC sim, cycles) |
|---|---|---|---|---|
| 1c | store buffer | **done**: posted DRAM stores | `EN_POST_ST` (1) | store 12.17 → 10.18 per loop iter (−2.0); CALL+RET −2.0; PUSH;POP −2.0; CALL+ENTER+store+LEAVERET −4.0; loads unchanged |
| 3 | front-end line bubble | **done** for 1-word code | `EN_PF_EARLY` (1) | 32 straight-line ALU ops 44.34 → 40.36 per iter; loop sweep (body 2..32 × 4 alignments) −5.2%, no case worse |
| 6 | FBR W / imm32 | **done** (RTL; needs toolchain) | `EN_FBR_W` (1) | — (gain needs FuseCmpBr) |
| 7a | DI / EI | **done** (RTL; needs runtime/toolchain) | `EN_DIEI` (1) | — |
| 8 / §3.3 | misaligned 16/32-bit | **done as a trap** (park 5, `ERR=0A`) | `EN_MISAL_TRAP` (1) | — |
| 9 | short CMPRVW / MULV / LEAPC | **done** (RTL; needs toolchain) | `EN_SHORT9` (1) | — (code size) |
| §3.4 | call chain in crash dump | **done** (F0..F5 lines) | — | — |
| §2.6 | I-cache miss counter | **done** (`PERF_IC_MISS` 0xF0, `PERF_IC_FILL_CYC` 0xF8) | — | — |
| 2b | FBR resolved in EX | **partial, opt-in**: EQ/NE (+W) only | `EN_FBR_EQ_EX` (**0**) | taken FBR NE −1.0 per iteration when on |
| 1b | load hit 4 → 3 | not done: stage B (`feat/m10b-fused`, `daa0795`) is not on GitHub | — | — |
| 4 | non-blocking MUL | not done: design below (§5.1) | — | — |
| 2a | branch prediction | not done: design below (§5.2) | — | — |
| 5 | EX→EX forwarding | not done (−0.75 ns measured in M6) | — | — |
| 1a | pipelined D-cache hit | not done (cache redesign) | — | — |

The SoC numbers come from `sim_1/new/tb_soc.sv` (real `mem_read_write` cache,
`bus_splitter`, MMIO, behavioural DDR) under Verilator. That model is
calibrated against your board figures: on master it gives store 4.05, load
5.05, leaf CALL+RET 17.07 cycles, against §2.2's 4.25 / 5.25 / 16.9.

Rough benchmark effect, scaled from §1's own estimates (not measured):

- **1c**: saves 2 of the 3 store cycles §2.4 assumed, so about two thirds of
  its gain (≈ −2.3% CoreMark, −2.6% Embench, −5% Dhrystone).
- **3**: removes the window bubble in straight-line 1-word code (the
  IFMISS-not-from-redirects row). It does not remove I-cache misses or 2-word
  fetch cost.

## 2. What changed, item by item

### 1c — posted stores (`EN_POST_ST`)

A DRAM store leaves MEM **one cycle after issue**; the port stays busy
(`st_post`, DV held) until its ready. Every other port user waits for that
drain: MEM issue, I-cache miss fill, IRQ frame push, debugger access, the
IRQ/debug drains, and `bus_idle` (so the SoC never takes the bus mid-drain).

It posts one cycle after issue rather than at issue because of the SMC
protection. That protection relies on the store's *completion-time* squash to
clean up an I-cache serve whose BRAM read was registered before the
issue-time snoop clear. So eligibility is decided **at issue, from registers,
into a flop** (`st_post_ok`):

- a DRAM store, and
- the line's tag is not in the I-cache tag shadow, and
- no IFB / NLB / ID / EX code overlap.

`mem_done_now` then gains only a registered term, so nothing new lands on the
`mem_busy` fan-out cone. Code-touching stores and every MMIO store take the old
path, so SMC, `ICACHE_INV` and `INT_MASK` ordering are unchanged.

Known conservatism: the tag shadow has no valid bits, so a store whose 8 KB
block shares a tag with a stale shadow entry at its index doesn't post. That
mostly hits data in the same 8 KB block as cached code. Posting at issue
(saving the third cycle) needs that eligibility one cycle earlier, off the EA
adder. Not attempted.

### 3 — line-crossing bubble (`EN_PF_EARLY`)

Root cause, from cycle traces: the NLB holds one line, and the prefetch of the
next line started only after the slide that emptied the NLB had moved
`want_line`. The 3-cycle BRAM prefetch then landed one cycle after the
2-dword window ran dry: one bubble per 8 one-word instructions.

The fix has two parts:

- **Early start.** On a slide from an odd base (the one that consumes the
  NLB's last dword), start the lookup of `want_line+1` on that edge.
- **Abort on redirect.** Any redirect (`ev_pcz`) aborts an in-flight
  prefetch, so the BRAM port reads the new `miss_dw` on the very next cycle.
  Without this, the earlier prefetch phase collided with short loops'
  back-branch lookups (+1 cycle/iteration in about half the alignments).

Both are enables on `pf_look` / `pf_line` only.

**For the compiler:** loop-head alignment **still matters** (+1..2
cycles/iteration with the head at line offset 12). That cost comes from where
the redirect target lands in the window, not from this bubble, so keep
`-klausscpu-pref-loop-align` for now. 2-word code is unchanged at one fetch
cycle per word: that's window bandwidth (one slide per cycle), a separate
problem.

### 6 — FBR W and imm32 (`EN_FBR_W`)

- **PRED 5/6/7** = EQW/LTW/ULTW. They reuse D2's dispatch-time `{x[31:0],0}`
  shift (`id_w`), so EX is unchanged. The PRED remap folds into `f_cmp_op`'s
  existing 4 inputs.
- **`LEN=10`** compares rs1 against sext(imm32) from word 1 (`id_var1`), with
  the simm13 displacement still in word 0.
- Encodings are in `ISA_ENCODING_V2_MAP.md` §8.1.

### 7a — DI / EI (`EN_DIEI`)

- `DI rd` returns the old *enable* bit and disables; `EI rs1` sets
  enable = rs1[0].
- The bit commits at WB. DI's result comes from the youngest older in-flight
  DI/EI/IRET in MEM/WB, all register-sourced, so there's no stall and no
  forwarding change.
- An IRQ boundary is taken only while enabled. A DI still in flight when a
  boundary is taken is caught by the existing drain-time re-check, exactly
  like an `INT_MASK` store.
- `WAIT` wakes on a pending IRQ even while disabled (WFI semantics), so
  `DI; test; WAIT` can't deadlock.
- The frame saves the bit at [43] in *disable* sense, so software-built
  frames (bit 43 = 0) resume enabled.
- `SWAP`/`CAS` (7b) were not done.

### §3.3 / 8 — misaligned trap (`EN_MISAL_TRAP`)

This applies to 16-bit accesses at odd addresses and 32-bit accesses not
4-aligned, both loads and stores.

- **Exempt:** MEMGET32, byte, 64-bit and stack accesses. The klausscc
  emulator specifies and tests those as defined behaviour.
- **Detection:** in EX, from separate 2-bit low-EA adds (off the 32-bit EA
  carry chain).
- **Handling:** the op enters MEM converted to a non-memory parking op, so
  `mem_port_op`/`mem_busy` are unchanged. Younger ops are dropped, a same-edge
  IRQ/debug boundary is cancelled, and older FBR/SMC squashes still win.
- **Result:** park kind 5, then HCF with `ERR_MISALIGN` (0x0A). No retire
  line is emitted (as for ILLEGAL).
- **Software risk:** any existing hand-written asm that relied on silent
  align-down now crashes. The netboot image runs its first 600k instructions
  identically and prints its banner (see §4).

### 9 — short forms (`EN_SHORT9`)

CMPRVW simm8 (class 3 short with INV=1), MULV simm8 (class A OP=3) and LEAPC
simm16 (class 2 OP=14 F=1). Immediates are formed in `f_short_imm` at the
IF→ID latch like the other short forms. Encodings are in §8.3 of the ISA map.

### §3.4 — call chain in the crash dump

Six lines `F0..F5` follow `S0..S3`: three R15 frame records (saved R15,
return address), each read through the same DDR path as the stack lines. An
unaligned or out-of-DRAM base prints the FF sentinel. Checked on the full-SoC
sim with a three-deep chain (exact values) and with `ERR=0A`. See
`CRASH_DUMP.md`.

### 2b-lite — EQ/NE FBR in EX (`EN_FBR_EQ_EX`, default **off**)

An equality FBR redirects from EX through the JMPcc path; ordered compares
stay in MEM. The new logic is a 64-bit XOR-reduce from the `ex_a`/`ex_b`
registers, in parallel with the equally deep `wb_value==0` reduce already on
the `flags_eff → exo_taken` side. It may fit, but ISA_V3_IMPL measured this
family at +0.41 ns, so it ships off until an A/B build decides.

## 3. Timing-closure plan (needs your Vivado runs)

None of this was built. In order:

1. **Build the branch as is** (Explore tier 1). For each switch, here is what
   it adds, worst-risk first:

   | Switch | Adds to |
   |---|---|
   | `EN_SHORT9` | legs in `f_short_imm` on the **w_op → id_var1** latch path (the old fetch-serve family); class 2/3/A legality terms (ID decode cone) |
   | `EN_FBR_W` | class D legality (+`op[21]`, `op[3:0]`); one class D term in `id_w` (w_op → register) |
   | `EN_MISAL_TRAP` | a 2-bit add + size/uop terms into the EX→MEM enables (`id_valid`, `ex_valid`, `fetch_halt`, `irq_active`, `mem_d.uop`), alongside the existing deeper `exo_taken` cone |
   | `EN_POST_ST` | at MEM issue: an `ic_tagff` LUTRAM read + 19-bit compare + seven 28/29-bit compares into **one flop**; then a registered term in `mem_done_now` |
   | `EN_DIEI` | a leg in `exo_result`'s U_DI case (1 bit); one term in the IRQ boundary and re-check |
   | `EN_PF_EARLY` | enables on `pf_look`/`pf_line` and a registered `want_line+1` |

2. **If WNS misses,** check which family the report names before flipping
   anything (the M12 rule: diagnose, don't strategy-hunt). The switches are
   ordered above by how much they add to an already-named family:
   `EN_SHORT9` (fetch latch), then `EN_FBR_W` (ID decode), then the rest.
3. **Then the opt-in:** a second build with `EN_FBR_EQ_EX=1`, compared on the
   `exo_taken → pc` family.
4. **Board checks:** m5e, then the §2.2 microbenchmarks.
   - Expected: store / PUSH / CALL ≈ 2 cycles cheaper; loads, MUL and the
     not-taken FBR unchanged; straight-line 1-word ALU code ≈ 1.03–1.06
     cycles/instruction, from 1.25.
   - Run the CoreMark / Embench / Dhrystone A/B as in §2.1. Read
     `PERF_IC_MISS` / `PERF_IC_FILL_CYC` to split the remaining IFMISS.

## 4. Verification done here

`tools/vsim` is new and runs without Vivado. `tools/vsim/run.py`, with
Verilator 5.020 and small sim-copy-only patches for that version's limits:

- **`test`:** 112 directed cases in `tests.py`:
  - every new instruction, compared against its 2-word equivalent where one
    exists;
  - DI windows under a fast timer with random latency, plus a negative control
    that fails without DI;
  - WAIT while disabled; IRET restoring a disabled frame;
  - misaligned traps (10 size/mode cases, younger store suppressed, wrong-path
    behind FBR/JMP);
  - 54 store-into-code SMC variants under posted stores (distance 0–40,
    8/32/64-bit, cold/warm I-cache) and data stores into code lines;
  - I-cache counters.

  The pos-tracker and predecode `$fatal` assertions are live throughout.
- **`diff --base REV`:** random terminating programs (ALU/compare/shift/MUL/
  DIV/loads/stores of all sizes/MEMGET32/PUSH-POP/CALL/ENTER/LEAVE/loops/
  FBR/JMPcc), with and without timer IRQs, at three memory latencies. Retire
  traces are identical to the previous commit at every step of this branch.
- **`socdiff --base REV`:** the same programs through the **full SoC**.
  Identical to master. `run_soc` also boots the real `netboot.mem` for 1M
  retires. The first 600,733 retires are identical to master, and so is the
  UART output. The traces then diverge only because timing-dependent polling
  starts (the first difference is a UART `TX_BUSY` poll at pc 0x150 spinning
  a different number of times).

The xsim flows (`perf/m5a/*.sh`) remain canonical. `tb_pipeline_isa.sv` now
merges sub-word store lanes itself for its trace line (a posted store can
retire before it lands) and treats park kind 5 as terminal. klausscc couldn't
be built here, so no emulator-golden diffs were run: that's the first thing
to do with xsim.

## 5. Not done: designs and the timing cost they face

### 5.1 Proposal 4 (non-blocking MUL)

With one in-order retire point, a MUL that leaves EX early still can't retire
before its result (cycle 3), so overlapping younger work means **out-of-order
completion**. That needs four things:

- **Scoreboard:** a one-entry MUL scoreboard (`rd`, count). Readers and
  writers of that `rd` stall in ID. That's a new `raw_stall` term, the same
  shape as `hit1`, but `raw_stall` is the dispatch-hold critical path.
- **Write port:** a second `rf` write port at completion (FF register file,
  so it's area, not a port problem).
- **Drains:** IRQ entry, debug halt and parks also wait for the scoreboard to
  empty.
- **Flags:** MUL writes Z/S/V. A younger flag reader or writer would also
  have to wait.

**Suggestion:** decide on the ISA side that **MUL does not write flags**. The
klausscc emulator already doesn't model MUL/DIV flag writes, and the M5 trace
diff strips `f=` for that reason, so validated compiled code can't depend on
them. That removes the flag scoreboard.

The retire trace then shows a MUL's `rd` only at completion, which the
emulator-golden flow would have to accept. Recommended as its own milestone,
with a board A/B on edn and ud.

### 5.2 Proposal 2a (prediction)

Start with the **return-address stack** alone (−0.5% CoreMark, −0.8% Embench,
crc32 −5.2%):

- Push `ex_pc_next` at a taken CALL in EX.
- On a RET/LEAVERET dispatch, redirect fetch from ID to the top entry instead
  of halting fetch.
- Verify at MEM against the loaded address; the MEM RET redirect already
  exists, so it fires only on a mismatch.

It touches only `fetch_halt` and one new pc source. Then add an ID-stage
direction predictor:

- 2-bit counters in a LUTRAM indexed by `pc` at fetch, registered into ID.
- The target formed in ID from `id_pc` + the word-0 displacement, as
  `id_pc_nx` is.
- A redirect from ID; recovery at EX/MEM using the carried fall-through
  `id_pc_nx`.

That leaves the IF serve cone alone (M11a's lesson). An IF-stage predecoder
(the "no extra bubble" version) puts an adder on `w_op`, the family M12 just
cleared.

### 5.3 Proposals 5, 1a, 1b

- **5 (EX→EX forwarding):** M6 measured the full `exo_result` feedback at
  −0.75 ns. The partial bypass (ADD/SUB/logic/MOV into the next op's
  `ex_a`/`ex_b`) still closes the same loop, adder → mux → adder, in one
  cycle; it needs a floorplan or EX pblock experiment. Not attempted blind.
- **1b:** stage B exists only on your machine. Its own notes say landing it
  needs EX-cluster/SHA margin or a floorplan first.
- **1a:** a pipelined hit is the `mem_read_write` redesign the proposal
  describes. 1c's drain logic (`st_post`) is the first piece of the
  "MEM occupancy 1" model it would need on the core side.

## 6. Toolchain / runtime asks (klausscpu-llvm, klausscc, runtime)

1. **klausscc emulator:**
   - misaligned 16/32-bit → stop as *Misaligned* (no trace line);
   - DI/EI (frame bit 43; WAIT wakes while disabled);
   - FBR PRED 5–7 and LEN=10;
   - the three short forms;
   - park kind 5 in the trace stop reasons.
2. **klausscc assembler / disassembler:** the encodings in
   `ISA_ENCODING_V2_MAP.md` §8 (templates verified against the RTL decode by
   `tools/vsim`).
3. **LLVM:**
   - FuseCmpBr for W compares and simm32 (drop `-klausscpu-fuse-loop-const`
     once in);
   - CompressPats for the short forms, plus a 16-bit PC-relative fixup and
     relaxation for LEAPC.S;
   - store latency in the schedule model (a store now occupies MEM 2 cycles,
     not 4);
   - re-measure `MaxGluedStoresPerMemcpy` and the frame-folding trades;
   - keep loop alignment (§2, item 3).
4. **Runtime / RTOS:** `arch_irq_lock` / `atomic.c` / FreeRTOS critical
   sections can become `DI r` / `EI r` once the bitstream has `EN_DIEI`.
   Inline atomics: DI, load, op, store, EI.
5. **Crash-dump parser (if any):** six new `F` lines, and `T0..TF`/footer
   moved down by six.

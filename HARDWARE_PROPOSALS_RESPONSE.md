# Response to HARDWARE_PROPOSALS.md (RTL side)

Branch `hw-proposals`. Answers `HARDWARE_PROPOSALS.md` (copied from
klausscpu-llvm `main`, 2026-10-29). The RTL was written in simulation on
2026-10-31, then built, timing-closed and **board-verified on 2026-11-01**
(Nexys A7, 100 MHz, Vivado 2025.2). The tested bitstream is from `0de5b21`.
Every change has its own build switch (`localparam bit EN_*` at the top of
`pipeline_core.sv`).

Merged to master as `3508f87`; the `0de5b21` bitstream was written to QSPI on
2026-11-01, verified by read-back, and boot-tested from flash: netboot banner,
DHCP, a UART load, and `hwprop_test` all pass. Rollback (outside the repo):

- `~/.cache/kbuild/rollback/`: the full pre-write flash dump and master
  `e58530b`'s bitstream, with restore commands in its `README.txt`.

## 0. Board results (2026-11-01)

**Timing:** met at tier 1 (Performance_Explore, no phys_opt rescue).

| Build | WNS | WHS | Worst path |
|---|---:|---:|---|
| master `e58530b` (same flow, same day) | +0.077 | +0.013 | core 2 `mul_a_q → DSP PCIN` |
| hw-proposals `23e616f` | +0.145 | +0.018 | core 2 `ex_b → pc` |
| hw-proposals `0de5b21` (**shipped**) | +0.130 | +0.024 | — |
| `0de5b21` with `EN_FBR_EQ_EX=1` | −0.123 (tier 2) | +0.024 | core 1 `ex_a → mul DSP CEA1` / `ex_a → pc` (−0.082) |

**Correctness on silicon, `0de5b21`:**

- M5e: 15 programs. 13 are byte-identical to the emulator. dhrystone and
  test_atomic print timing-dependent numbers (as on master); both pass their
  self-checks (`Self-check: PASS`; `test_atomic PASSED`, 137/0).
- `tools/board/hwprop_test`: ALL PASS. It covers 704 FBR W / imm32 /
  short-CMPRVW branch cases, short MULV and LEAPC, DI/EI values, 0 of 2342
  timer ISRs inside a DI window, WAIT while disabled, and 21 posted-store SMC
  cases.
- `tools/board/hwprop_misal`: `ERR=0A`, with F1/F3/F5 = the three return
  addresses of the call chain.
- CoreMark (validation run) and all 19 Embench benchmarks verify.
- xsim (M5a TB): the RTL retire traces of 15 programs (21M instructions) are
  identical to master's.

**Performance** (board cycles, master vs `0de5b21`; same ELFs, toolchain
`56ee1aba697b`, -O2; Dhrystone -O1 as built by the runtime Makefile):

| Benchmark | master | hw-proposals | change |
|---|---:|---:|---:|
| CoreMark (2000 it) | 1,978,438,739 | 1,904,365,769 | **−3.7%** (CoreMark/MHz 1.010 → 1.050) |
| Dhrystone, cycles/run | 220 | 211 | **−4.1%** (DMIPS/MHz 2.587 → 2.697) |
| Embench speed geomean | 0.554 | 0.579 | **+4.5%** |
| aha-mont64 | 3,368,556 | 3,275,486 | −2.8% |
| crc32 | 9,668,180 | 9,450,577 | −2.3% |
| cubic | 5,964,424 | 5,654,580 | −5.2% |
| edn | 10,111,893 | 9,861,322 | −2.5% |
| huffbench | 7,702,720 | 7,329,033 | −4.9% |
| matmult-int | 12,051,686 | 11,701,722 | −2.9% |
| minver | 6,227,393 | 5,989,017 | −3.8% |
| nbody | 4,154,365 | 3,938,662 | −5.2% |
| nettle-aes | 9,400,924 | 9,129,353 | −2.9% |
| nettle-sha256 | 7,587,069 | 7,007,204 | −7.6% |
| nsichneu | 11,137,068 | 10,986,886 | −1.3% |
| picojpeg | 8,997,125 | 8,497,143 | −5.6% |
| qrduino | 7,314,940 | 7,112,953 | −2.8% |
| sglib-combined | 8,600,966 | 8,244,659 | −4.1% |
| slre | 9,570,491 | 9,096,269 | −5.0% |
| st | 4,587,171 | 4,398,423 | −4.1% |
| statemate | 5,432,553 | 5,019,666 | −7.6% |
| ud | 8,410,233 | 7,926,206 | −5.8% |
| wikisort | 2,608,484 | 2,471,588 | −5.2% |

All of this gain comes from 1c and 3; the other items need compiler support
first. No benchmark is slower.

**Micro-kernels** (`hwprop_kern`, board `PERF_CYCLES` per body, minus the
timer-read overhead):

| Kernel | master | hw-proposals |
|---|---:|---:|
| 1-word ALU, independent | 1.12 | 1.01 |
| SETR (2-word) | 2.00 | 2.03 |
| store 64 (back to back) | 4.06 | 3.93 |
| store ; ALU | 5.00 | 3.93 |
| load 64 | 5.06 | 5.06 |
| store ; load | 9.06 | 9.06 |
| PUSH ; POP | 14.81 | 12.87 |
| taken FBR | 6.00 | 5.81 |
| not-taken FBR | 1.12 | 1.06 |
| MUL | 4.00 | 4.06 |
| DI ; EI | (traps) | 2.93 |

How to read these:

- A posted store hides behind non-memory work, but back-to-back stores still
  wait for each other's drain. A store directly followed by a load gains
  nothing.

Two bugs only xsim/Vivado showed, both fixed in `23e616f`:

- localparams used before their declaration;
- `ic_tagff` had no initial value, so it was X in 4-state sim.

## 0c. 1a stage 2: branch dcache-1a (2026-11-02)

`cd9b76e`, on master `8e6b8e5`. Timing met at tier 1 (WNS +0.135 / WHS
+0.028). **Merged to master as `67efcfd` and written to QSPI on 2026-11-02**
(that exact bitstream, sha256 `996b6950e002…`, read-back verified; it boots
from flash and `hwprop_test` passes). Rollback to mem-early:
`~/.cache/kbuild/rollback/README.txt`.

- **Load leaves MEM once issued:** a DRAM load hit (LOAD/POP/LEAVE) moves on
  once its request is on the port. WB collects the data, and a consumer
  dispatching that cycle takes it directly (`EN_LD_WB`, `EN_LD_FWD`).
- **Chaining:** the next request chains onto the completion edge when the
  address differs.
- **Blocking path unchanged:** MEMGET32, RET/IRET/LEAVERET, MMIO and
  non-posted stores.
- **Board, against mem-early (same ABI-v3 binaries):**

  | Benchmark | Change |
  |---|---|
  | CoreMark | −7.7% (1.65 → 1.79 /MHz) |
  | Embench geomean | +8.6% (0.706 → 0.767) |
  | Dhrystone | 173 → 143 cycles/run |

- **Kernels (`perf_kern`, cycles per body):**

  | Kernel | mem-early | stage 2 |
  |---|---:|---:|
  | ld;use | 5.99 | 4.99 |
  | st;ld | 6.99 | 4.99 |
  | ld;st(dep) | 8.87 | 6.93 |
  | `cp_4l4s` | 27.9 | 19.9 |
  | ld;ld, different addresses (sim) | 3.95 | 2.96 |
  | ld;ld, same address | 3.93 | 3.87 |

- **What's left of 1a:** the bus allows one outstanding request, so memory
  code is bounded at ~one access per 2–3 cycles. A load still holds the
  pipe one cycle in WB. Stage 3, a pipelined cache with an accept
  handshake, is deferred behind 2a at the toolchain session's request.

## 0b. Second round: branch mem-early (2026-11-02)

Built from the klausscpu-llvm handoff (HANDOFF-m8.md). Branch `mem-early`,
bitstream from `9ea49da`. **Merged to master as `8e6b8e5` and written to QSPI
on 2026-11-02** (that exact bitstream, sha256 `28efc551f080…`; read-back
verified; boot from flash shows the netboot banner and `hwprop_test` passes).
Rollback to `3508f87`: `~/.cache/kbuild/rollback/` (`README.txt`).

| Item | Change | Switch |
|---|---|---|
| 2a (a) | 4-entry return-address stack; RET/LEAVERET predicted in ID, checked in MEM | `EN_RAS` |
| 2a (b) | ID-stage prediction: 512 × 2-bit counters for JMPcc/FBR (taken only at counter 3), direct JMP/CALL always taken | `EN_BP`, `BP_STRONG` |
| 1 (step) | early load issue: a load entering MEM with the port idle issues on that edge | `EN_EARLY_LD` |
| 1 (step) | DRAM response to the core without the bus_splitter register (MMIO stays registered); `w_pipe_owns` registered | `COMB_DRAM_RET` |
| 3 (handoff) | VGA scan-out ends its DDR tenure early when a cache miss waits (not when the line fetch runs late); `VGA_CTRL` resets at program start | — |
| fix | SMC: an op latched into ID from an IFB slot that the completing store invalidates is refetched (pre-existing hole on master) | — |

**Timing:** met at tier 1, WNS +0.176 / WHS +0.023. The `EN_FBR_EQ_EX` and
ownership lessons are in the commit messages.

**Board, ABI-v3 binaries (toolchain c335750ad0d9), against QSPI `3508f87`:**

| Benchmark | Change |
|---|---|
| CoreMark | 1,880,300,228 → 1,534,185,701 cycles (**−18.4%**) |
| Embench speed geomean | 0.588 → 0.706 (**+20.1%**; all 19 faster) |
| Dhrystone | 211 → 173 cycles/run (DMIPS/MHz 2.697 → 3.289) |

Correctness:

- M5e 13/15 byte-identical; the other two are timing prints and pass their
  self-checks.
- `hwprop_test` passes everything (704 branch cases, DI window, SMC).
- `hwprop_misal` gives `ERR=0A` with the call chain.
- All benchmarks verify.

Costs (board, cycles per body):

| Kernel | QSPI | mem-early |
|---|---:|---:|
| load64 back-to-back | 5.06 | 3.93 |
| store64 back-to-back | 3.93 | 3.00 |
| store;load | 9.06 | 7.00 |
| PUSH;POP | 12.81 | 10.75 |
| predicted-taken FBR | 5.81 | 3.25 |

Also, from simulation: a mispredicted FBR costs ≈ 7, and CALL leaf + RET
15.1 → 12.8.

VGA scan-out (640×480×16) costs a framebuffer fill +37.6% on QSPI and
+11.7% on mem-early. Pre-proposal bitstreams can no longer run phase-9
binaries (they use the 2-word FBR), so the LVGL A/B was done this way.

Still open: true 1a (load hits that don't freeze the pipe, which needs the
pipelined cache/bus redesign); early issue for stores and for back-to-back
accesses.

## 0n. Crypto for the Zephyr 4.5 port: branch crypto-restore-ctr (2026-11-03)

Asked by klausscpu-runtime (its `ZEPHYR_4_5_PLAN.md` §4). Zephyr 4.5's SSH
offers only aes128-ctr + hmac-sha2-256 on PSA multipart operations, which
interleave hashes. RTL `fb6bf4e`, on master `da5371a`. **Merged to master and
written to QSPI on 2026-11-03** (`--verify`; it boots from flash). Rollback to
stq: `~/.cache/kbuild/rollback/README.txt`. Register details: `MMIO_MAP.md`.
Every addition is new registers or bits. With them at reset, the old map
behaves as before.

**What shipped (bitstream 1):**

- **SHA state save/restore.** `SHA_HIN0..3` (`0xF00B_00C0..D8`) are packed
  like `SHA_DIGEST`. `SHA_CTRL[2]` RESTORE loads H from them and `[3]` ZERO
  wipes H, HIN and BLOCK. Software keeps HMAC inner/outer midstates per key,
  so the one-key HMAC wrapper isn't needed and is unchanged. `SHA_STATUS.BUSY`
  now also covers a request queued for the next core tick. Without that, the
  sim showed RESTORE|START reading not-busy for a few cycles, and software
  overwrote BLOCK under the pending START.
- **AES-CTR.** `AES_CTR0/1` (`0xF00A_00C0/C8`) and `AES_MODE` (`0x0D0`):
  `[0]` CTR (a write to `AES_IN1` starts a block, `OUT = E(CTR) ^ IN`,
  CTR++ with full 128-bit big-endian carry), `[3]` INC32. `AES_STATUS.DONE` is
  now set when busy falls. Before, a level copy of the core's sticky done
  could set it again just after a GO; the runtime polls BUSY, so it wasn't
  affected.
- **Capability registers** at offset `0x0F8` in both blocks: `SHA_CAP = 0x3`
  (RESTORE, ZERO) and `AES_CAP = 0x3` (CTR, INC32). Older bitstreams read 0,
  so drivers choose the fast path at run time.
- **TRNG:** unchanged. HEALTH_OK already latches until RESEED.
- **Bitstream 2 (agreed):** GCM_FUSE / GCM_DEC / LEN (LEN masks the GHASH
  input only, never OUT) and a writable GCM_TAG. The bits are reserved in
  `AES_MODE` / `AES_CAP`. RSA/Montgomery help only if the runtime's
  measurement asks for it.

**Timing:** build_fast (Performance_Explore + tier 2) gave −0.009. The single
failing endpoint was a core-1 path, `ic_tag → r_want_line`, moved by the added
area; stq had +0.066. Re-placing from the same `_opt.dcp` with
`tools/build_alt_place.tcl` gave ExtraPostPlacementOpt **+0.050** / WHS +0.024
(shipped) and ExtraTimingOpt +0.012.

**Verification:**

- `tools/vsim/crypto_kat.py` drives both wrappers at the MMIO level under
  Verilator. The write strobe is held 3 cycles, as on the SoC. Expected values
  come from Python hashlib / hmac / cryptography; 159 checks:
  - SP 800-38A F.5.1 CTR;
  - carry into byte 7, carry across bytes 3..15, and the all-FF wrap;
  - inc32 with no carry into byte 11;
  - CTR readback = start + N;
  - legacy GO after CTR;
  - SHA save → another whole hash → restore → finish, both as RESTORE;START
    and as RESTORE|START;
  - ZERO;
  - HMAC from software midstates with two keys interleaved;
  - the legacy HMAC wrapper on RFC 4231 TC1.
- On the board, `tools/board/crypto_kat_hw.c` replays the same op list
  (`--emit-c`): ALL PASS on the JTAG image and again after flash boot.
  `hwprop_test` ALL PASS.

**Board numbers** (`crypto_kat_hw`, 100 MHz):

| Loop | Cycles |
|---|---:|
| AES-CTR, hardware counter, tight C loop | 69 / block (≈22 MB/s) |
| AES-CTR, software counter + ECB GO + XOR, tight C loop | 69 / block |
| SHA state save + restore (4 reads, 4 writes, RESTORE, poll) | 65 |

**Runtime confirmation (same day, its shared driver picks the fast paths from
`AES_CAP` / `SHA_CAP`):**

- `crypto_selftest`: 27/27, identical to its board baseline.
- New `crypto_selftest2`: 35/35. It covers the CTR carry/wrap/readback
  cases, a 37-byte tail, GCM round trips 0..4096, SHA restore and interleaved
  contexts, ZERO, and HMAC midstates on RFC 4231 TC1-4, a 131-byte key and
  interleaved keys.
- Bare-metal 16 KiB throughput:
  - CTR mode: 15.5 MB/s, against 6.4 MB/s for its old per-block loop;
  - GCM: 5.3 MB/s;
  - SHA via the restore path in 1500-byte updates: 18.4 MB/s (one-shot 20.1).
- Bitstream 2 is deferred until TLS GCM matters. The Zephyr 4.5 SSH path is
  CTR + HMAC only.

The hardware counter saves no cycles over a tight software-counter loop. The
~1,400 cycles/block the runtime measured is in its software layers, not on the
bus. Hardware CTR mainly makes a tight driver simpler, and SHA RESTORE makes
the hardware usable at all under PSA.

## 0m. A second store entry: branch stq (2026-11-03)

`27439c7`, on master `e87c722`. **Merged to master and written to QSPI on
2026-11-03** (`--verify`; it boots from flash). Toolchain validation passed:
the hw_* fixtures, the board suite, csmith-board 150 and zephyr (tick_cost
median 282; irq_soak +2%, as IRQ entry now also drains the queued store).
Current compiler: CoreMark -O2 205.78, -Os 162.73; Embench 1.067 / 0.934.
Rollback to ldbyp: `~/.cache/kbuild/rollback/README.txt`. This is "back-to-back
stores, step A", the user's choice of next step.

**Change (`EN_SQ`).**
- A DRAM store that finds the port busy with a posted store leaves MEM into
  a queue entry (`sq_*`), with the same issue-time snoop, post check and
  squash as `EN_ST1`.
- If the posted store drains on that edge to another address, it goes
  straight onto the port instead.
- The queue goes on the port as soon as the posted store drains. Every other
  port user waits for it.
- The hit pipe checks the queued store like the other in-flight stores
  (forwarding when it covers the load).

**Timing:** met at tier 1, WNS +0.066 / WHS +0.020.

**SoC sim, cycles per group:**

| Group | Before | After |
|---|---:|---:|
| st;1 | 2.54 | 2.04 |
| st;ld | 2.54 | 2.04 |
| st;st | 2.02 | 1.90 |

**Board, same binaries, vs ldbyp:**

| Benchmark | Change |
|---|---|
| Dhrystone | 105 → 96 cycles/run |
| Embench speed geomean | 1.033 → 1.046 (cycles −1.21%; statemate −4.7%, slre −2.8%; none slower) |
| CoreMark | −0.2% |

**Not done: step B** (1 store/cycle: a pulse write protocol, a cache write
buffer, pipelined write hits). With step A in, stores wait for the port only
0.9% of cycles on average (statemate 3.2%), so B was dropped, with the user's
agreement.

## 0l. Load → ALU/BOOL/FBR bypass: branch ldbyp (2026-11-02)

`c6ba1db`, on master `f8fd12c`. **Merged to master and written to QSPI on
2026-11-02** (`--verify`; it boots from flash). Toolchain validation passed:
the hw_* fixtures, csmith-board 150, zephyr and the board suite. Current
compiler: CoreMark -O2 205.29, -Os 162.44; Embench 1.049 / 0.919. The
compiler's ReadAdvance model of the latency-2 consumers is in (Embench
−0.27% at -O2 and -Os). Rollback to btic64: `~/.cache/kbuild/rollback/README.txt`.

**Change (`EN_LDBYP`).**
- An ALU / BOOL / FBR consumer may dispatch while the fast load (LOAD / POP /
  LEAVE, DRAM) that writes its rs1/rs2 is in MEM.
- The load's hit-pipe or forwarded value is extracted at its MEM→WB edge
  (`wb_hext`) and feeds the consumer's ALU / compare operands in EX, with the
  W-compare shift.
- A consumer that does not leave EX on the edge its load commits (a miss held
  it, or the pipe is frozen) captures the value into `ex_a`/`ex_b` then.

**Load-use distance** is 2 for these consumers, 3 for everything else (shifts,
MUL/DIV, MOV/LEA, address and store-data operands, JMPR).

**Timing:** met at tier 1, WNS +0.027 / WHS +0.027.

**SoC sim, cycles per group:**

| Group | Before | After |
|---|---:|---:|
| ld;use | 4.02 | 3.02 |
| ld;1;use | 4.04 | 3.05 |
| ld;fbr | 4.02 | 3.02 |

**Board, same binaries, vs btic64:**

| Benchmark | Change |
|---|---|
| CoreMark | −4.9% (1.961 → 2.063 /MHz) |
| Embench speed geomean | 1.014 → 1.033 (cycles −1.85%; nsichneu −10.4%, sglib −5.9%, huffbench −5.4%; none slower) |
| Dhrystone | 105 cycles/run |

CoreMark at 2000 iterations now runs under 10 s and reports `SHORT` (CRCs
correct), so the harness iteration count needs raising.

## 0k. BTIC 64 entries: branch btic64 (2026-11-02)

`51a3e2b`, on mul2 `54cb0a4`. **Merged to master and written to QSPI on
2026-11-02** (`--verify`; it boots from flash). Toolchain validation:
- on `51a3e2b`: the hw_* fixtures, the board suite (-O2, no differences or
  regressions) and csmith-board 50;
- on `be7fa38` (one guard term different): zephyr and csmith-board 150.

Board, current compiler: CoreMark -O2 196.12, -Os 156.19; Embench 1.030 /
0.908. Rollback to mul2: `~/.cache/kbuild/rollback/README.txt`.

**Changes:**
- **BTIC 32 → 64 entries,** indexed by `pc[7:2] ^ pc[13:8]`. The sim
  attribution had about 2.1% of cycles in ifmiss after a taken branch.
- **The early-redirect hit check comes from the ID snapshot only.** It
  carries its entry's line indices and is cleared by matching invalidations;
  one landing on the very edge the snapshot was taken is caught a cycle later
  from a registered index. This replaces a 64:1 `bt_v` re-read on the
  redirect cone.
- **`EN_BTPF` is guarded by `!fbr_taken` only.** The full `!ev_ovr` guard put
  the RAS-miss compare cone into the `pf_line` enable: one build failed at
  −0.903. With no guard at all, aha-mont64 lost 0.85% to prefetches started
  on a fused-branch mispredict edge.

**Timing:** met at tier 1, WNS +0.077 / WHS +0.027.

**Board, same binaries, vs mul2:**

| Benchmark | Change |
|---|---|
| CoreMark | −0.8% (1.961 /MHz) |
| Embench speed geomean | 1.005 → 1.014 (cycles −0.93%; slre −4.0%, st −3.8%, minver −2.6%; none slower) |

## 0j. MUL: 2-cycle issue, product committed from the DSP: branch mul2 (2026-11-02)

`54cb0a4`, on callret `c39ef27`. **Merged to master and written to QSPI on
2026-11-02** (`--verify`; it boots from flash; mul 2.00 cycles). Toolchain
validation passed: the hw_* fixtures, the board suite, csmith-board 150 and
zephyr. CoreMark -O2 194.97, -Os 154.66; Embench 1.021 / 0.898. The
compiler models MUL latency 4 with a 2-cycle hold. Rollback to callret:
`~/.cache/kbuild/rollback/README.txt`.

**Change (`EN_MUL2`).**
- A MUL holds EX 2 cycles (it was 4), then travels on through MEM.
- `mul_pipe2` holds its product on its 2nd and 3rd cycle after leaving EX:
  - a MUL that reaches WB on time commits straight from the DSP;
  - one held in MEM captures the product (`mem_mres`).
- Hazards:
  - a MUL in MEM is not forwardable, like a load;
  - from WB, the product enters the load-forward path's first select (timing);
  - flag readers wait while a MUL is in MEM/WB, so the product stays off
    `flags_eff`.
- The MEM op class (load / MUL) is registered at EX→MEM for the RAW-hazard
  cone.

**Timing:** met at tier 1, WNS +0.005 / WHS +0.015.

**SoC sim, cycles per group:**

| Group | Before | After |
|---|---:|---:|
| mul | 4.00 | 2.01 |
| mul;use | 6.01 | 5.01 (latency 4) |
| mul;mul | 8.01 | 4.02 |
| mul + 3 independent | 7.05 | 5.05 |

**Board, same binaries, vs callret:**

| Benchmark | Change |
|---|---|
| CoreMark | −5.3% (1.843 → 1.946 /MHz) |
| Embench speed geomean | 0.974 → 1.005 (cycles −3.10%; edn −21.3%, matmult −6.6%, ud −5.9%; none slower) |

## 0i. CALL/RET and store→load: branch callret (2026-11-02)

`c39ef27`, on master `3f2f82a`. **Merged to master and written to QSPI on
2026-11-02** (`--verify`; it boots from flash; push;pop 4.00 cycles).
Toolchain validation passed: the hw_* fixtures, csmith-board 150, zephyr and
the board suite. CoreMark -O2 185.35, -Os 153.44; Embench 0.988 / 0.879.
With this forwarding the compiler's same-line store→load scheduling rule no
longer pays (it measured +0.08% Embench) and has been removed. Rollback to
fetch-bypass: `~/.cache/kbuild/rollback/README.txt`.

**Diagnosis** (sim trace). A call + ret of an empty function cost about 10
cycles (`perf_kern` call1 9.93):
- the RET waited in ID for the CALL's SP to commit;
- its stack read then froze MEM on the port, behind the CALL's still-posted
  push of that same dword.

**Changes:**
- **`EN_SPFWD`:** `ex_sp` is captured at dispatch from the youngest older SP
  writer in MEM/WB, so only an SP writer still in EX holds an SP reader back.
- **`EN_STLF`:** a LOAD/POP/LEAVE/RET takes the youngest older store's
  lane-replicated dword when the addresses match and the byte enables cover
  the load (from the store leaving MEM on this edge, or the one posted on the
  port). It enters MEM as an answered hit-pipe load. The data select sits
  after the registers (timing).
- **`EN_HRET`:** RET joins the hit pipe:
  - forwarded: 1 cycle in MEM;
  - hit: it holds MEM for one cycle and the return check reads the captured
    address;
  - miss: the port read in MEM.

**Store→load cases** (cycles over the no-hazard reference):

| Case | Penalty |
|---|---|
| Same dword, covered | none |
| Same dword, not covered | +2.1 / +2.9 / +1.5 with 0 / 1 / 2 ops between |
| Same line, other dword | +1.5 only with exactly 2 ops between (the load's read collides with the cache's line write) |

**Timing:** met at tier 1, WNS +0.071 / WHS +0.030.

**SoC sim, cycles per group:**

| Group | Before | After |
|---|---:|---:|
| call+ret+add | 10.88 | 7.50 |
| push+pop+add | 8.12 | 5.25 |

**Board, same binaries, vs fetch-bypass:**

| Benchmark | Change |
|---|---|
| CoreMark | −0.5% (1.843 /MHz) |
| Embench speed geomean | 0.966 → 0.974 (crc32 −7.2%, statemate −3.0%; none slower) |
| crc32 -Os pair | 6.62M → 6.28M |

## 0h. Fetch bandwidth: branch fetch-bypass (2026-11-02)

`90dcb2d`, on master `0cccefa`. **Merged to master and written to QSPI on
2026-11-02** (`--verify`; it boots from flash). Toolchain validation passed:
the board suite with no differences, csmith-board 150, zephyr (tick_cost 373,
irq_soak unchanged, so no cold-code penalty is visible) and the hw_* fixtures.
With llvm 148dc75: CoreMark -O2 184.31, -Os 150.05; Embench 0.979 / 0.869.
Rollback to st-1slot: `~/.cache/kbuild/rollback/README.txt`.

**Diagnosis** (`perf-attrib`, a sim-only per-cycle attribution). Most of the
board's "unattributed" cycles are front end, about 15–18% of cycles:

- the ID latch bubble after a refill;
- taken-branch bubbles;
- above all, fetch bandwidth.

Straight-line 2-word ops ran at 2.0 cycles/op: the window slides a dword per
cycle, but the single NLB was refilled about once per 3 cycles. Dynamic
2-word share: statemate 37%, minver 25%, slre 13%, CoreMark 6.5% (older
compiler; llvm 148dc75's global merging removes most of statemate's LEAPCs).

**Changes:**
- **`EN_BTPF`:** a BTIC-hit redirect starts the next-line prefetch on its own
  edge.
- **`EN_NLB2`:** two NLB entries, both serving slides and line crosses. The
  prefetch fills `want`, then `want+1`, back to back.
- **Dropped:**
  - a "demand waits for the in-flight prefetch" rule, which interacted badly
    with `EN_BTPF` on crc32;
  - a "prefetch yields the BRAM port" rule, −1.1 ns on the IF cone.

**Timing:** met after phys_opt, WNS +0.027 (tier 1 −0.054). The worst family
is `st → ex_d` / `pc`, not the fetch cone.

**Costs, SoC sim:**

| Case | Before | After |
|---|---:|---:|
| Straight-line 2-word op | 2.01 | 1.24 cycles/op |
| Straight-line 3-word op | 4.3 | 2.5 cycles/op |
| Taken JMP + one op, target at word 0 / 1 / 2 / 3 of its line | 3.40 / 3.10 / 3.44 / 5.10 | 3.4 / 3.1 / 3.4 / 4.1 |

Cold, branchy random programs are 1.3–2.5% slower: the prefetch holds the
BRAM port more often.

**Board, same binaries, vs st-1slot:**

| Benchmark | Change |
|---|---|
| CoreMark | −0.8% (1.833 /MHz) |
| Embench speed geomean | 0.938 → 0.966 (cycles −2.91%; statemate −12.6%, nettle-aes −8.5%, nbody −6.0%; none slower) |
| Dhrystone | 107 cycles/run |

The crc32 -Os layout pair is now 6,624,989 / 6,624,636 (layout-insensitive).

## 0g. One-slot stores, miss trim, BTIC race fix, post-check fix: branch st-1slot (2026-11-02)

`ccdc289`, on dcache-s3 `822baeb`. **Merged to master and written to QSPI on
2026-11-02** (that exact bitstream, `--verify`; it boots from flash and
`hwprop_test` passes). Toolchain validation passed: the board suite with no
UART differences, csmith-board 150, zephyr and the klausscc hw_* fixtures.
CoreMark -O2 183.24, -Os 148.21; Embench 0.946 / 0.840. Rollback to
dcache-s3: `~/.cache/kbuild/rollback/README.txt`.

- **One-slot stores (`EN_ST1`).**
  - A DRAM store with an idle port leaves MEM on its issue edge, posted at
    once.
  - The registered "no fetch-side code copy" check (`st_post_ok`) decides the
    cycle after. If the store overlaps code, it squashes MEM/EX/ID/IFB, drops
    in-flight I-cache lookups, prefetch and the NLB, and refetches from its
    fall-through (`sq_spec`).
  - The IFB may now slide on such a store's edge. The old gate cost a fetch
    bubble per store whenever the two coincided.
  - Back-to-back stores are still about 2 cycles each, limited by the cache
    FSM.
- **`st_post_ok` needs a VALID I-cache line** (also a fix for dcache-s3).
  - The check compared only the I-cache tag shadow. A snoop or program start
    clears the valid bits but not the tags, so once a line's index had held
    code with that tag, every later store into the line was refused.
  - On dcache-s3 such stores just don't post. On `4026d92` they squashed on
    every call: board crc32 -Os went 6.63M → 8.02M cycles, depending on what
    ran before. The SoC sim starts with zero tags, so it never showed.
  - A new directed test, `smc_stale_tag`, covers it, and
    `tools/board/perfclr` / `perfdump` dump the counters around any program.
- **Miss trim.** A hit-pipe miss issues its port request on the edge it leaves
  MEM, not from WB. In sim, a pointer chase that misses every step goes 13 → 12
  cycles per step.
- **BTIC race fix (also latent in 2a).** A BTIC capture that landed on a
  store-snoop edge for its lines survived the invalidation with the pre-store
  window. The capture now waits for a later window.
- **Timing:** met at tier 1, WNS +0.180 / WHS +0.024.
- **Board, same binaries, vs dcache-s3:**

  | Benchmark | Change |
  |---|---|
  | CoreMark | −2.1% (1.780 → 1.818 /MHz) |
  | Embench speed geomean | 0.918 → 0.938 (cycles −2.06%; matmult −7.0%, sha256 −4.2%, picojpeg −4.1%; none slower) |
  | Dhrystone | 111 → 108 cycles/run |

  The crc32 -Os layout pair: 6,624,982 / 6,798,718, the same as sim.
- **Compiler model:** a store costs 1 slot. A same-line load is fast from
  store issue +4, so 3 instructions between.
- **Verification:**
  - vsim: directed tests (including `smc_*`), diff with IRQ, and socdiff plus
    about 600 dense same-line programs (`--span 16..256K`). The dense mode
    caught a same-edge store→load hazard, fixed before the build.

## 0f. 1a stage 3, the load-hit pipe: branch dcache-s3 (2026-11-02)

`822baeb`, on master `060dd09`. **Merged to master as `9e15047` and written
to QSPI on 2026-11-02** (that exact bitstream, `--verify`; it boots from flash
and `hwprop_test` passes). Rollback to timing-headroom:
`~/.cache/kbuild/rollback/README.txt`. The design and measurements are in
`STAGE3_DCACHE_PLAN.md` (§0 has the as-built notes).

- **How it works.**
  - A DRAM LOAD/POP/LEAVE looks itself up in shadow copies of the cache's
    tag and data arrays (+14 RAMB36) on its EX→MEM edge.
  - On a hit it leaves MEM after one cycle with its data.
  - On a miss or a conflict it issues the ordinary port request from WB.
- **Timing:** met at tier 1, WNS +0.155 / WHS +0.018.
- **Board, same binaries, vs `7d84761`:**

  | Benchmark | Change |
  |---|---|
  | CoreMark | −12.7% (1.554 → 1.780 /MHz) |
  | Embench speed geomean | 0.806 → 0.918 (cycles −12.3%) |
  | Dhrystone | 137 → 111 cycles/run |

- **Toolchain validation** (current compiler):
  - CoreMark -O2 179.16, -Os 146.54; Embench 0.928 / 0.825.
  - The board suite passes (only the known `test_rtos` interleaving), as do
    csmith-board 150, zephyr (gui_demo / gui_lvgl / doom, over the blitter
    and VGA) and blit_selftest.
- **Known cost:** `perf_baseline` `ptr_chase` is +3.8% cycles. A dependent
  load that misses pays one extra cycle, because it issues from WB.
- **Compiler model:**
  - load latency 3, issue to issue;
  - 1 slot per load;
  - no load serialization;
  - a same-line load is fast from store issue +3.

## 0e. Timing headroom + exclusive stall counters: branch timing-headroom (2026-11-02)

`7d84761`, on master `d573c7b`. **Merged to master as `646f44e` and written to
QSPI on 2026-11-02** (that exact bitstream, `--verify`; it boots from flash
and `hwprop_test` passes). Toolchain validation passed: the board suite,
csmith-board 150 and zephyr. Benchmarks are cycle-identical to fetch-2a
(CoreMark 1,286,584,050). Rollback to fetch-2a:
`~/.cache/kbuild/rollback/README.txt`.

- **Timing headroom** (behaviour unchanged). The four changes:
  - EX operands load on every non-busy cycle.
  - The redundant dispatch-chain IRQ latch is gone.
  - The deferred Z is reduced at MEM→WB (`wb_zero`).
  - `id_rs1`/`id_rd` are predecoded at the latch.

  Tier-1 WNS went from −0.194 to +0.150 on the same core, and no EX family
  is left within 0.4 ns. The shipped build shows +0.003 because its worst
  path is now in `blitter_dma` (placement); the core's worst is +0.195.
- **`PERF_CTRL[1]`, exclusive stall attribution.** At most one bucket per
  cycle, in priority memwait > exbusy > data > flags > sp > ifmiss.
  MMIO_MAP.md has the exact rules.
- **`PERF_STALL_LOADUSE`** now counts a load producer in EX, MEM or WB; it
  used to count only the MEM case. A SoC-sim RAW histogram showed load
  producers are most of the RAW stall (CoreMark ~10 of 13.9 points).
- **Not taken:** a restricted ALU→ALU bypass (`alu-bypass`, `72e7374`) is
  correct but gives −0.6% in sim, too little to spend margin on.

## 0d. 2a fetch stage + core 2 removed: branch fetch-2a (2026-11-02)

`a2c9e4b`, on master `67efcfd`. **Merged to master as `b26385d` and written
to QSPI on 2026-11-02** (that exact bitstream, `--verify`; it boots from flash
and `hwprop_test` passes). Toolchain validation passed: the board suite,
csmith-board 150 and zephyr; CoreMark -O2 155.89 / -Os 131.25. Rollback to
dcache-1a: `~/.cache/kbuild/rollback/README.txt`.

| Item | Change | Switch |
|---|---|---|
| 2a (c) | branch-target instruction cache: 32 entries (index `pc[6:2]^pc[11:7]`) holding the first fetch dword pair at a predicted-taken target, so the redirect from ID serves the target without waiting for the I-cache | `EN_BTIC` (`BTIC_EN` parameter) |
| 2a (c) | the predicted-taken redirect fires on the branch's first cycle in ID (unless an EX redirect may fire that cycle) | — |
| area | AMP core 2 not built: `0xF010_0000` still decodes (reads 0, writes ignored), it never requests DDR, LiteEth belongs to core 1 | `EN_CORE2` (top level, **0**) |
| timing | branch-taken perf strobes registered before their 48-bit counter (counts land 1 cycle late) | — |

The BTIC is invalidated by I-cache line index on fills and stores (the SMC
snoop), and cleared on ICINV and program start.

**Timing:** met only after one tier-2 phys_opt pass, WNS +0.007 / WHS
+0.021; tier 1 was −0.194. The design is route-bound (54% LUTs): the
failing paths are 70–80% routing, and placement noise is around 0.1 ns.
Builds before the final one:

- with core 2 still in (BTIC off in core 2): −0.082;
- with core 2 removed, before the perf-strobe register: −0.165. The remaining near-critical paths are all in core 1:

- `id_op → ex_a`;
- `id_op → pos`;
- `wb_value → flags → mul-pipe enable`;
- `ex_b → m_addr`.

**Board, same ABI-v3 binaries, against dcache-1a (`67efcfd`):**

| Benchmark | Change |
|---|---|
| CoreMark | 1,417,339,321 → 1,286,584,050 cycles (**−9.2%**) |
| Embench speed geomean | 0.767 → 0.806 (cycles geomean −5.0%; all 19 faster, huffbench −13.4%) |
| Dhrystone | 143 → 137 cycles/run |

- **`perf_kern` loops:** all 21 are faster, −12% to −47%, for example
  `e_cmp_jne` 1027 → 729 and `e_jmp_back` 920 → 719.
- **Correctness:** M5e 13/15 (the same two timing prints as before, both
  self-check PASS); `hwprop_test`, `hwprop_kern` and `hwprop_misal` pass;
  all benchmarks verify; `vga_bw` shows no underflow.
- **Sim branch costs** (cycles per taken branch, predicted):
  - FBR: 2.11;
  - JMP: 2.04;
  - CMP+JMPcc: 3.09.

## 1. Status

| # | Proposal | State (all items board-verified unless noted) | Switch (default) | Measured (full-SoC sim, cycles) |
|---|---|---|---|---|
| 1c | store buffer | **done**: posted DRAM stores | `EN_POST_ST` (1) | store 12.17 → 10.18 per loop iter (−2.0); CALL+RET −2.0; PUSH;POP −2.0; CALL+ENTER+store+LEAVERET −4.0; loads unchanged |
| 3 | front-end line bubble | **done** for 1-word code | `EN_PF_EARLY` (1) | 32 straight-line ALU ops 44.34 → 40.36 per iter; loop sweep (body 2..32 × 4 alignments) −5.2%, no case worse |
| 6 | FBR W / imm32 | **done** (RTL; needs toolchain) | `EN_FBR_W` (1) | — (gain needs FuseCmpBr) |
| 7a | DI / EI | **done** (RTL; needs runtime/toolchain) | `EN_DIEI` (1) | — |
| 8 / §3.3 | misaligned 16/32-bit | **done as a trap** (park 5, `ERR=0A`) | `EN_MISAL_TRAP` (1) | — |
| 9 | short CMPRVW / MULV / LEAPC | **done** (RTL; needs toolchain) | `EN_SHORT9` (1) | — (code size) |
| §3.4 | call chain in crash dump | **done** (F0..F5 lines) | — | — |
| §2.6 | I-cache miss counter | **done** (`PERF_IC_MISS` 0xF0, `PERF_IC_FILL_CYC` 0xF8) | — | — |
| 2b | FBR resolved in EX | **partial, opt-in, does not close**: EQ/NE (+W) only; −0.12 ns on the core-1 `ex_a → pc` / mul-enable families | `EN_FBR_EQ_EX` (**0**) | taken FBR NE −1.0 per iteration when on |
| 1b | load hit 4 → 3 | not done: stage B (`feat/m10b-fused`, `daa0795`) is not on GitHub | — | — |
| 4 | non-blocking MUL | not done: design below (§5.1) | — | — |
| 2a | branch prediction | **done**: RAS + ID predictor (§0b), BTIC + first-cycle redirect (§0d) | `EN_RAS`, `EN_BP`, `EN_BTIC` (1) | §0b, §0d |
| 5 | EX→EX forwarding | not done (−0.75 ns measured in M6) | — | — |
| 1a | pipelined D-cache hit | **stage 2 done** (§0c); stage 3, the pipelined cache, not done | `EN_LD_WB`, `EN_LD_FWD` (1) | §0c |

The SoC numbers come from `sim_1/new/tb_soc.sv` (real `mem_read_write` cache,
`bus_splitter`, MMIO, behavioural DDR) under Verilator. That model is
calibrated against your board figures: on master it gives store 4.05, load
5.05, leaf CALL+RET 17.07 cycles, against §2.2's 4.25 / 5.25 / 16.9.

The measured benchmark effect is in §0: CoreMark −3.7%, Embench +4.5%,
Dhrystone −4.1%, all from 1c and 3. The simulation-time estimate (from the
proposal's own model) had been ≈ −2.3% CoreMark for 1c alone, plus 3's
straight-line fetch gain.

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
registers.

**Measured (2026-11-01): it does not close at 100 MHz.**

- **First build: −0.203 ns**, on the perf-counter strobe (`perf_fbr_taken` →
  the SoC's 48-bit branch-taken counter adder).
  - Fixed in `0de5b21`: the counters now read `ex_bcvt_t[0]` for JMP/CALL and
    a registered strobe for the EX-resolved FBR.
- **Rebuilt: −0.123 ns** (tier 2), now on the real families: core 1
  `ex_a → pc` (−0.082 ns) and `ex_a → multiplier DSP clock enable`.

Landing it would need the equality computed at dispatch, as a registered
compare of the forwarded operands, which puts the forwarding muxes in front
of the reduce instead. Not attempted.

## 3. Timing closure (done 2026-11-01)

See §0. The default configuration met at tier 1 with more margin than master
had on the same day: +0.130 ns against +0.077. WNS is set by core 2's EX/DSP
paths in both. No switch had to be turned off.

How to rebuild: `vivado -mode batch -source tools/build_fast.tcl -tclargs
-resynth` (about 12–14 min on this machine).

## 4. Verification (simulation, 2026-10-31; see §0 for the board)

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

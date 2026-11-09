# MMU plan: exceptions, user mode, MPU, TLB

Branch `mmu` (klaussCPU, with klausscpu-runtime and klausscpu-llvm on their
own `mmu` branches). Status: **design note for review, no RTL yet.**

Goal: a path to xv6 and then Linux (arch/klausscpu + musl), with Zephyr
USERSPACE and NuttX protected and kernel builds along the way. Today there
are no privilege levels, and TRAP, illegal instructions and misaligned
accesses park the core.

| Stage | Contents | Detail here |
|---|---|---|
| 1 | Precise restartable exceptions, user and supervisor modes, SYSCALL, kernel stack switch | full (§2–§9) |
| 2 | MPU: 8 base/limit/permission regions (Zephyr USERSPACE, NuttX protected) | outline (§10) |
| 3 | Software-refilled TLB with ASIDs, 64 KiB pages, LR/SC | outline (§11) |

Section numbers in `CPU_ARCHITECTURE.md` are written as "arch §n".

---

## 1. Compatibility contract

At reset and on program load (`start`), the CPU is in **supervisor mode**,
the exception vector `EVEC` is 0 and translation is off. In that state, code
that never uses the new features behaves **exactly as now**:

- Illegal instructions, TRAP and misaligned accesses park the core with
  today's `park_kind` and crash dump, because `EVEC = 0` (§4.4).
- IRQ entry from supervisor mode is unchanged: same frame, same SP, same
  masking. Frame bit 44 (new: previous mode) is 0, so IRET stays in
  supervisor mode. All three software frame builders in klausscpu-runtime
  (Zephyr `thread.c`, FreeRTOS `port.c`, `src/rtos.c`) write bits 63:44 as
  0. I checked them on 2026-11-09.
- The new encodings (SYSCALL, MFSR, MTSR) are illegal today. On the new
  core, with `EVEC = 0`, they park as illegal (`park_kind` 2), which is the
  same crash an old bitstream gives.
- Every privilege check and every user-mode fetch or MMIO check is gated
  by the user-mode bit, which supervisor code can only set through an IRET
  with frame bit 44 = 1.
- Software can probe for the features with `CPU_FEATURES` (§6.3). That
  register reads 0 on old bitstreams, because reserved MMIO offsets read 0.

Proof obligations before merging: `tools/vsim run.py diff --base master`
retire traces must be identical (random programs, several latencies, with
and without `--irq`). `socdiff` must be identical. The board's existing
Zephyr, FreeRTOS and bare-metal suites must pass on the new bitstream, and
the perf and microbenchmark numbers must not change.

---

## 2. Stage 1 overview

Four things are added:

1. **Precise exceptions.** A faulting instruction commits nothing,
   everything younger is flushed, and the core pushes a frame and vectors
   to `EVEC`. The cause and fault address go in system registers.
2. **User/supervisor mode** (a `UM` bit). In user mode the privileged
   instructions trap, and so does any access to MMIO (`0xF000_0000` and up).
3. **SYSCALL** instruction.
4. **Kernel stack switch** (m68k model). There are two stack pointers. An
   exception or interrupt taken from user mode pushes its frame on the
   kernel SP. IRET back to user mode switches back to the user SP.

What stage 1 does **not** give: memory protection between user programs,
or between user and kernel *DDR*. That is stage 2 (MPU) or stage 3 (TLB).
Stage 1 protects only the CPU control state and the devices.

---

## 3. Programmer's model (stage 1)

### 3.1 New state

| State | Width | Reset / `start` | Notes |
|---|---|---|---|
| `UM` | 1 | 0 | current mode: 1 = user |
| `SP_ALT` | 32 | 0 | the inactive stack pointer. In supervisor mode it holds the **user** SP. In user mode it holds the **kernel** SP. Accessed as SR `USP` from supervisor mode. |
| `EVEC` | 32 | 0 | exception vector; bits [1:0] read as 0 |
| `ECAUSE` | 8 | 0 | cause of the last exception (§4.1) |
| `EADDR` | 32 | 0 | fault address or instruction word of the last exception |
| `SCRATCH0/1` | 64 each | 0 | free for the kernel (per-CPU / current-thread pointer) |

The architectural `SP` (GETSP/SETSP/PUSH/…) is always the current mode's
stack pointer. The hardware never touches `SP_ALT` except on a mode change,
so a supervisor-only system never sees it.

`start` (program load) clears `UM`, `EVEC`, `ECAUSE` and `EADDR`, so a newly
loaded program cannot inherit a user mode or a vector into the old image.
Debugger resume does not clear them.

### 3.2 Frame format (extends arch §13.1)

One 64-bit frame for both interrupts and exceptions:

```
[63:45] = 0            reserved
[44]    = PM           previous mode: 1 = user        (restored by IRET)  NEW
[43]    = DI disable                                   (restored by IRET)
[42:39] = INT_MASK                                     (restored by IRET)
[38:32] = flags, as today (Z, Z, C, V, S, S^V, C)      (Z/C/V/S restored)
[31:0]  = PC            interrupt: resume PC; exception: §4.1
```

Cause and fault address are **not** in the frame. They are in `ECAUSE` and
`EADDR`, which the handler must read before anything that could fault again.
This keeps the frame a single word, so IRET, the existing software-built
frames and the RTOS ports stay the same.

### 3.3 Entry sequence (interrupt or exception)

Taken with the pipe drained. Every older instruction has committed and every
posted store has drained, as for IRQ entry today.

1. `old_um = UM`. If `UM = 1`: swap the stack pointers (`SP` becomes the
   kernel SP, `SP_ALT` the user SP) and set `UM = 0`.
2. Push the frame on the (now kernel) `SP`: `SP -= 8`, `mem64[SP] = frame`,
   with `PM = old_um` and DI, INT_MASK and flags as they were before entry.
3. **Interrupt:** clear `INT_MASK[n]`, and jump to `INT_VECn`. Unchanged
   apart from steps 1–2.
   **Exception:** write `ECAUSE` and `EADDR`, set the **DI bit**
   (interrupts disabled), and jump to `EVEC`. `INT_MASK` is unchanged.

Exceptions disable interrupts so that a nested interrupt handler cannot take
a fault (stage 3: a TLB miss) and overwrite `ECAUSE`/`EADDR` before the
exception handler has read them. The old DI bit is in the frame, and IRET
restores it. Interrupts keep today's behaviour and leave DI alone. A kernel
that wants interrupt entry with interrupts disabled starts its handler with
`DI`.

Exceptions are synchronous: they are taken whatever DI and `INT_MASK` say.

### 3.4 IRET

IRET pops the frame from the current SP (it is privileged, so this is the
kernel SP) and restores PC, Z/S/C/V, INT_MASK, DI and **`UM = PM`**. If
`PM = 1`, after the pop the stack pointers swap: `SP` becomes the user SP,
and `SP_ALT` becomes the kernel SP after the pop (8 above the frame).

To enter user mode for the first time, the kernel builds a frame with
`PM = 1`, sets `USP` (MTSR), and runs IRET. The kernel SP after that IRET,
which is the SP value just above the popped frame, is where the next
user→kernel entry pushes. With one kernel stack per thread, the kernel
switches threads with SETSP in supervisor mode as now, and saves or
restores each thread's user SP with MFSR/MTSR `USP` (or in its trap frame).

### 3.5 Privileged instructions

In user mode these raise **PRIVILEGED** and do not execute:

`DI`, `EI`, `IRET`, `HALT`, `WAIT`, `RESET`, `DELAYR`/`DELAYV` (a long DELAY
holds EX and so delays interrupt entry), `LCDCMD`/`LCDDATA`/`LCDRST`, `MFSR`
(except SR `STATUS`), `MTSR`.

Everything else is allowed. `TRAP` raises TRAP in either mode, and `SYSCALL`
raises SYSCALL in either mode.

### 3.6 User-mode memory rules (stage 1)

- **MMIO.** In user mode, any load, store or stack access with an effective
  address in `0xF000_0000`–`0xFFFF_FFFF` raises ACCESS_LOAD / ACCESS_STORE.
  The device never sees it. This covers the interrupt controller,
  `CACHE_CTRL` (including ICACHE_INV), timers, UART, everything. User code
  gets I/O and cache maintenance through syscalls.
- **Fetch.** In user mode, an instruction with any of its words in
  `0xF000_0000`+ raises ACCESS_FETCH. The fetch engine never puts such an
  address on the memory port in user mode, not even speculatively (a
  predicted-taken direct branch could otherwise read a device register).
- **Misaligned PC.** In user mode, an instruction at a PC with `[1:0] != 0`
  raises MISALIGNED_FETCH. Supervisor mode keeps today's behaviour (the low
  bits are ignored).
- **DDR.** Unprotected in stage 1.

Alignment rules for data accesses are as arch §8.3 in both modes:
MEMGET32, byte, 64-bit and stack accesses never raise MISALIGNED.

---

## 4. Exceptions

### 4.1 Causes

| ECAUSE | Name | Raised by | Frame PC | EADDR |
|---|---|---|---|---|
| 0 | — | (reset value) | | |
| 1 | ILLEGAL | undefined encoding, nonzero reserved field, unimplemented SR | the instruction | instruction word 0 |
| 2 | PRIVILEGED | §3.5 in user mode | the instruction | instruction word 0 |
| 3 | TRAP | `TRAP` | the TRAP itself | instruction word 0 |
| 4 | SYSCALL | `SYSCALL` | **the next instruction** | 0 |
| 5 | MISALIGNED_LOAD | arch §8.3 trap rows, loads | the instruction | effective address |
| 6 | MISALIGNED_STORE | same, stores | the instruction | effective address |
| 7 | ACCESS_LOAD | user MMIO read (§3.6); stage 2/3 read permission | the instruction | effective address |
| 8 | ACCESS_STORE | user MMIO write; stage 2/3 write permission | the instruction | effective address |
| 9 | ACCESS_FETCH | user fetch from `0xF…` (§3.6); stage 2/3 execute permission | the instruction | address of its first faulting word |
| 10 | MISALIGNED_FETCH | user-mode PC with `[1:0] != 0` | the PC | the PC |
| 11–13 | TLB_MISS_LOAD / _STORE / _FETCH | stage 3 | the instruction | faulting VA |
| 14–255 | reserved | | | |

- **Restartable:** for every cause except SYSCALL, the frame PC is the
  faulting instruction and **nothing of it has happened**: no register, flag,
  SP, R15 or PC change, and no memory or device access. IRET re-executes it.
  To skip it, add its length (TRAP is always 1 word) to the frame PC.
- **SYSCALL** puts the next PC in the frame, so the kernel returns with a
  plain IRET. A restarted syscall (`-ERESTARTSYS`) subtracts 4.
- **Stack instructions are loads or stores by direction.** PUSH, PUSHV,
  CALL and ENTER are stores. POP, RET, LEAVE, LEAVERET and IRET are loads.
- **EADDR for an instruction with two accesses** (MEMGET32 spanning two
  doublewords, LEAVERET's `[R15]` and `[R15+8]`) is the lowest faulting byte
  address. In stage 1 these can only fault on MMIO, and both accesses are
  checked before either happens.

### 4.2 Multi-side-effect instructions

The rule: **memory and device accesses are the only side effects before
WB.** Every other architectural write (registers, flags, SP, R15, INT_MASK,
DI, UM, PC redirect) commits at WB, in order. A fault is decided **before**
the instruction's first memory access, so the instruction either does all
of it or none of it.

| Instruction | Side effects | On a fault |
|---|---|---|
| PUSH / PUSHV / PUSHV64 | store, SP | neither |
| POP | load, rd, SP | none |
| CALL / CALLcc / CALLR / CALLREL | store (return address), SP, PC | none; a not-taken conditional call cannot fault |
| RET | load, SP, PC | none |
| ENTER | store (old R15), R15, SP | none |
| LEAVE | load, R15, SP | none |
| LEAVERET | two loads, R15, SP, PC | none; both addresses are checked first |
| IRET | load, SP, flags, mask, DI, UM, PC | in user mode: PRIVILEGED. In supervisor mode in stage 1 it cannot fault (supervisor may touch MMIO) |
| MEMGET32 (spanning) | one or two loads, rd | none; both addresses are checked first |
| DI | rd, DI bit | PRIVILEGED in user mode; cannot fault in supervisor |

A fault on the *target* of a CALL, RET or IRET (ACCESS_FETCH or
MISALIGNED_FETCH at the new PC) belongs to the next instruction. The
CALL/RET/IRET has completed, and the frame PC is the target.

Speculative predictor state is not rolled back. A faulting RET has already
popped the return stack in ID, and a faulting CALL has pushed it. That costs
at most one later return misprediction; it is never a correctness issue,
because every predicted return is checked in MEM.

### 4.3 Nesting and double faults

- An exception in a handler (supervisor mode) pushes another frame on the
  current SP and overwrites `ECAUSE`/`EADDR`. Read them first.
- Stage 1 frame pushes cannot fault (supervisor, no translation). As today,
  a misaligned SP is not checked.
- Stage 3 adds a double fault (frame push to an unusable kernel SP). It
  parks with a new `park_kind` 6.

### 4.4 `EVEC = 0` and the debugger

The order of checks when an exception reaches WB:

1. **Debugger attached** and cause ILLEGAL, TRAP or MISALIGNED_*: debug halt
   with today's `dbg_cause` (TRAP/ILL). This is unchanged, in either mode.
2. **`EVEC = 0`**: park as today. ILLEGAL → `park_kind` 2, TRAP → 1,
   MISALIGNED_* → 5. Any new cause (PRIVILEGED, SYSCALL, ACCESS_*,
   MISALIGNED_FETCH) → 2, the same crash an old bitstream gives for an
   undefined opcode. A debugger, if attached, halts on these too.
3. Otherwise: exception entry (§3.3).

HALT in supervisor mode parks or debug-halts as today. WAIT in supervisor
mode is unchanged.

### 4.5 Interrupts and user mode

An interrupt in user mode is taken at a dispatch boundary as now. Entry does
the stack switch and `PM = 1` (§3.3). Nothing else changes: priority,
vectors, the drain-time recheck and abandon, and the auto-mask of the
source.

---

## 5. New instructions (stage 1)

All are class B (system), LEN = 01 (1 word). Unused fields must be 0, or the
instruction is ILLEGAL.

| Mnemonic | Template | Fields | Semantics |
|---|---|---|---|
| `SYSCALL` | `0x6C08_0000` | none (OP `[21:16]` = 8) | raise SYSCALL (§4.1) |
| `MFSR rd, sr` | `0x6C40_0000` | `[25:22]` = 1, `[21]` = 0, sr `[19:12]`, rd `[11:8]`, `[7:0]` = 0 | rd = SR[sr], zero-extended |
| `MTSR sr, rs1` | `0x6C60_0000` | `[25:22]` = 1, `[21]` = 1, sr `[19:12]`, rs1 `[7:4]`, `[11:8]` = 0, `[3:0]` = 0 | SR[sr] = rs1 (bits that do not exist are ignored); **serializing** |

`[20]` is reserved (0) in MFSR and MTSR. Examples: `MFSR r3, ECAUSE` =
`0x6C402300`. `MTSR EVEC, r1` = `0x6C601010`. `SYSCALL` = `0x6C080000`.

Behaviour:
- **MTSR serializes.** Fetch stops behind it and restarts at the next
  instruction once it has committed, so a changed SR is seen by everything
  after it, including fetch (this matters from stage 2 on).
- **MFSR** reads the committed SR. No forwarding is needed: every SR writer
  is either serializing (MTSR) or happens with the pipe drained (entry,
  IRET).
- MFSR or MTSR with an SR number not in §6 is ILLEGAL. Software can trust
  that an SR it reads exists.
- With the debugger attached, SYSCALL behaves as SYSCALL (no halt).

Assembler syntax proposal: SR operands by name (`status`, `evec`, `ecause`,
`eaddr`, `usp`, `scratch0`, `scratch1`) or number.

---

## 6. System registers and feature probe

### 6.1 SR numbers (stage 1)

| SR | Name | Access | Contents |
|---|---|---|---|
| 0x00 | `STATUS` | R (**user mode too**), W (supervisor) | `[0]` UM (read-only), `[1]` DI (read-only, the committed bit), `[8]` MPU_EN (stage 2), `[9]` TE (stage 3). Other bits read 0. Writes to UM and DI are ignored; use IRET / DI / EI. |
| 0x01 | `EVEC` | RW | exception vector, `[1:0]` read 0 |
| 0x02 | `ECAUSE` | RW | §4.1 code in `[7:0]` |
| 0x03 | `EADDR` | RW | `[31:0]` |
| 0x04 | `USP` | RW | `SP_ALT`, which is the user SP while in supervisor mode |
| 0x05 | `SCRATCH0` | RW | 64-bit |
| 0x06 | `SCRATCH1` | RW | 64-bit |
| 0x10–0x1F | MPU | | stage 2 (§10) |
| 0x20–0x2F | TLB / ASID | | stage 3 (§11) |

`STATUS` is readable in user mode so that code like Zephyr's
`arch_is_user_context()` can test the mode without a syscall.

### 6.2 Debugger

The debug module gets two more register indices: 20 = `SP_ALT`, 21 =
`{UM, DI}` (read; write allowed while halted). It needs nothing else.

### 6.3 `CPU_FEATURES` (MMIO, read-only)

At `0xF00F_0048`, in the timer/IRQ block. It reads 0 on every older
bitstream.

| Bits | Meaning |
|---|---|
| `[0]` | stage 1: exceptions, user mode, SYSCALL, MFSR/MTSR |
| `[1]` | stage 2 MPU present (`[7:4]` = region count / 2 or similar, defined then) |
| `[2]` | stage 3 TLB present (`[15:8]` = entry count) |
| `[3]` | LR/SC |

---

## 7. Pipeline changes (stage 1)

Today's pipeline already has the properties this needs (`PIPELINE_IMPL.md`,
`pipeline_core.sv` header): WB is the single commit point, side effects
start only in MEM, and the misaligned trap converts the faulting op at
EX→MEM and drops everything younger. Stage 1 generalizes that path. It adds
no new kind of flush.

### 7.1 The exception op (XOP)

A faulting instruction becomes an **exception op**: a non-memory,
non-writing uop that carries `{cause, eaddr}` down the pipe and never
commits. There are three places it is created, and each kills the younger
work as the existing mechanism at that point already does:

| Where | Causes | How younger work is stopped |
|---|---|---|
| **ID** (decode) | ILLEGAL, PRIVILEGED, TRAP, SYSCALL, ACCESS_FETCH, MISALIGNED_FETCH | as today's serializers: the op sets `fetch_halt` as it dispatches, so nothing younger is fetched. ILLEGAL and TRAP already do exactly this. |
| **EX→MEM** | MISALIGNED_LOAD/STORE | the existing `ex_misal` conversion (`mem_d.uop <= U_TRAP`, younger EX/ID dropped, `fetch_halt`) |
| **MEM, first cycle** | ACCESS_LOAD/STORE (user MMIO). Stage 2/3 permission and TLB faults use the same slot. | the check is computed in EX and **registered** into MEM. The op enters MEM already converted (no port access). In its first MEM cycle it squashes EX/ID through the same path as a taken fused branch (`fbr_taken`): EX/ID invalidated, LCD strobe, RAS push, early load issue, DIV and DELAY suppressed, and `fetch_halt` set instead of a redirect. As with `fbr_taken`, this happens only on the edge it leaves MEM (`!wb_wait`). |

The user-MMIO check needs EA[31:28], which comes after the 32-bit EA adder.
That is why it is a register input (D-path) and acts one cycle later,
instead of joining the `ex_misal` conversion. `ex_misal` sits on the EX→MEM
handoff enables and was deliberately kept to 2-bit adds. Stage 2/3 checks
have the same shape (compute in EX, act in MEM cycle 1), so this slot is
built once.

Early load issue (`w_early_ld`, MEMGET32 / LEAVERET / IRET) puts a read on
the port at the EX→MEM edge. For those ops the EA is a base register (no
adder), so the gate `!(UM && exo_iaddr[31:28] == F)` is cheap. The hit pipe
already excludes MMIO addresses.

An IRQ or debug boundary taken while an XOP is in flight is cancelled, as
`ex_misal` does now (`irq_active <= 0`). Invariant: an XOP in MEM or WB
implies `!irq_active`.

### 7.2 WB: commit becomes entry

Today a parking op at WB (`wb_park`) parks. New: an XOP at WB (`wb_valid &&
!wb_wait`) commits nothing and either parks or debug-halts (§4.4), or
latches `{cause, eaddr, epc}` and starts the entry sequencer. `epc` is
`wb_pc`, or the next PC for SYSCALL (carried, like `mem_pc_next`). A data
fault's `eaddr` comes from a new `wb_eaddr` (`mem_eaddr` carried). An
ID-detected fault's `eaddr` is `wb_op`. No retire (trace) line is emitted
for an XOP, as for illegal and misaligned today.

### 7.3 Entry sequencer

The IRQ entry sequencer is generalized. It is the same drain condition
(`!id/ex/mem/wb_valid`, port idle, no posted store or queued store, no
outstanding load) and the same single-store frame push. The changes:

- push address `(UM ? SP_ALT : SP) - 8`. Frame bit 44 = `UM`.
- on completion: the stack swap and `UM <= 0` when `UM` was 1. For
  exceptions, `irq_dis <= 1` and `ECAUSE`/`EADDR` written. `pc <=` vector,
  `fetch_halt <= 0`.
- the IRQ path's abandon-on-recheck stays IRQ-only. An exception cannot be
  abandoned.

### 7.4 IRET

IRET already reads the frame in MEM (`m_rdata`) and commits at WB. New:
`wb_um_new = m_rdata[44]`. When it is 1, `wb_sp_new = SP_ALT` (the user SP)
and `SP_ALT <= ` the popped kernel SP (`sp + 8`). Both are computed at the
MEM→WB edge, so SP forwarding (`EN_SPFWD`) to a younger op forwards the
right value without depending on fetch latency. `UM` commits at WB. The
first instruction after IRET cannot reach ID before then, because IRET
redirects as it leaves MEM and fetch needs at least one cycle to rebase.
This gets an assertion: no `id_valid` op was latched under a stale `UM`.

### 7.5 Decode

- `f_decode` gets `UM` as an input. A privileged op in user mode is decoded
  as an XOP (PRIVILEGED, serializing). The latch-time serialize predecode
  (`id_ser`) uses the same function, so it stays equal to the full decode
  (the existing assertion covers it). `UM` only changes with the pipe
  drained or behind a serializing IRET, so the predecode can never see a
  stale `UM`.
- New legal encodings: SYSCALL (serializing XOP), MFSR, MTSR (serializing).
- ACCESS_FETCH / MISALIGNED_FETCH: at the IF→ID latch, `UM && (pc[31:28] ==
  F || last_dw is in F || pc[1:0] != 0)` is registered into a flag that
  overrides the decode. These are registers and small compares, already
  computed for `id_last_dw`.

### 7.6 Fetch

In user mode the fill engine does not issue a miss-fill whose dword is in
`0xF…`. It serves a window of zeros (no port access, no I-cache install)
instead, and the ID override (§7.5) turns the op into ACCESS_FETCH. The NLB
prefetch only reads the I-cache, so it never touches the port. The BTIC only
holds copies of I-cache lines. Supervisor fetch is unchanged.

### 7.7 MFSR / MTSR datapath

- MTSR writes its SR at WB, then restarts fetch at `wb_pc + 4`, which is
  the RESET-style redirect done at WB.
- MFSR needs its 64-bit value somewhere on the result path. Candidates: a
  leg on `exo_result`, or loading `ex_a` at dispatch and executing as COPY.
  Both touch timing-sensitive muxes. I'll pick by building both. This is an
  RTL detail with no software impact.

### 7.8 Build switches and core 2

`EN_EXC` (stage 1), `EN_MPU`, `EN_TLB`, as localparams in the existing
`EN_*` style. Core 2 (AMP) gets a module parameter, default off, because it
has no use for user mode and its area matters. With `EN_EXC = 0`, the new
encodings are illegal and nothing else changes.

### 7.9 Cost estimate

Stage 1: about 250 FFs (SP_ALT, EVEC, ECAUSE, EADDR, 2×SCRATCH, pipe tags)
and well under 1k LUTs. No BRAM. The timing exposure is the `UM` term on
the decode/serialize predecode, the MEM-cycle-1 squash term (it joins the
`fbr_taken` squash family, all registered), and MFSR's result leg.

---

## 8. Verification plan (stage 1)

RTL side, done by me before anything reaches the board:

1. **Compatibility.** `run.py diff --base master` (random programs, latencies
   1/3/7, with and without `--irq`): identical retire traces. `socdiff`:
   identical. All existing directed tests pass, with `DEBUG_EN` 0 and 1.
2. **Directed vsim tests** (kasm.py gets SYSCALL/MFSR/MTSR):
   - each cause from supervisor mode and from user mode, with the frame,
     `ECAUSE`, `EADDR`, SP and SP_ALT checked;
   - user-MMIO fault on every multi-side-effect instruction (PUSH, PUSHV,
     POP, CALL, CALLR, CALLcc taken and not taken, RET with and without a
     RAS hit, ENTER, LEAVE, LEAVERET, MEMGET32 spanning): register, SP,
     R15, flags and memory unchanged; the handler fixes up and IRETs, and
     the instruction completes on retry;
   - each privileged op in user mode;
   - SYSCALL round trip, nested SYSCALL in supervisor mode, an IRQ taken in
     user mode, an IRQ in an exception handler after EI, and an exception
     with an IRQ pending;
   - `EVEC = 0`: each existing cause parks with the same `park_kind` as
     master; with the debugger attached, it debug-halts as master;
   - user fetch toward `0xF…`, including a predicted-taken direct branch on a
     wrong path: no MMIO read reaches the bus model;
   - a fault in MEM with a fast load waiting in WB, a posted store draining,
     a DIV in EX, and an LCD op in EX.
3. **Random programs with faults.** randprog gains a user-mode variant that
   inserts faulting ops with a fix-up-and-retry handler. The final state
   must equal the same program without faults.
4. **Timing.** A full build must meet 100 MHz with the same strategy as
   master. CPI checks on the board microbenchmarks.

Software side (klausscpu-runtime, yours, once the interface is fixed):
bare-metal exception and syscall tests on the board, then the runtime and
Zephyr handlers.

---

## 9. Implementation order (stage 1)

1. This note reviewed and the interface frozen (§3–§6). Encodings are
   added to `ISA_ENCODING_V2_MAP.md` §9 and `CPU_ARCHITECTURE.md` gets an
   exceptions section, both on `mmu`.
2. Toolchain: MC-layer support for SYSCALL/MFSR/MTSR in klausscpu-llvm (on
   its `mmu` branch) and kasm.py. Until then, `.word` works.
3. RTL in steps, each passing the compatibility diff:
   (a) XOP plumbing and WB→entry with `EVEC`, `ECAUSE`, `EADDR`,
   MFSR/MTSR, SYSCALL (supervisor only);
   (b) `UM`, `SP_ALT`, IRET/entry stack switch, the privileged decode;
   (c) user MMIO data faults (the MEM-cycle-1 slot), then the fetch checks;
   (d) `CPU_FEATURES`, the debug register indices, core-2 parameter.
4. Timing build, board run of the existing suites, then hand the bitstream
   to the runtime session.

---

## 10. Stage 2 outline: MPU

For Zephyr USERSPACE and NuttX protected builds, with translation off.

- **8 regions**, each a base/limit pair at 32-byte granularity (cache-line
  sized), with attributes R, W, X (for user mode) and GUARD. SRs
  `0x10–0x1F`: `MPU_BASEn` = `{base[31:5], attrs}`, `MPU_LIMITn` = the last
  byte, `[31:5]`. Direct access to every region (no index register), so a
  context switch can reprogram the per-thread partitions with plain MTSRs.
  `STATUS.MPU_EN` enables the MPU.
- **User mode:** default deny. An access is allowed if any enabled region
  covering it grants the permission (union, no priority). That covers
  Zephyr memory domains (text RX, rodata R, thread stack RW, partitions)
  and NuttX's user text/data.
- **Supervisor mode:** default allow. A region marked GUARD denies
  supervisor access, which gives stack guards (Zephyr
  `HW_STACK_PROTECTION`).
- **Checks:** data in EX on the EA, registered, and acted on in MEM cycle 1
  (the §7.1 slot). Fetch on the PC at the IF→ID latch, registered (the
  §7.5 override). That is 16 data and 16 fetch 27-bit comparators, about
  1k LUTs. Faults are ACCESS_LOAD / ACCESS_STORE / ACCESS_FETCH with
  `EADDR` = the address.
- The user-mode MMIO rule (§3.6) still applies on top.
- When `TE = 1` (stage 3) the MPU is ignored.
- Open: should 32-byte granularity be 64 bytes to save comparator bits?
  Zephyr only needs a declared granularity.

---

## 11. Stage 3 outline: TLB, pages, LR/SC

### 11.1 Page size versus cache aliasing: **64 KiB pages**

The constraint: the D-cache is 2-way with **32 KiB per way** (index
`[14:5]`), and the I-cache is direct-mapped **8 KiB** (index `[12:4]`). With
4 KiB pages, index bits `[14:12]` come from the translated part of the
address.

| Option | Load latency | Software cost | Verdict |
|---|---|---|---|
| A. 4 KiB pages, physically indexed (translate before the cache) | the TLB then sits between the EA adder and the BRAM address pins (`h_addr`). That does not fit in a cycle at 100 MHz, so +1 cycle on **every** load-use, kernel included. The load path was the most heavily optimized part of the core (ldbyp, hit pipe, STLF). | none | no |
| B. 4 KiB pages, virtually indexed, OS avoids aliases (page colouring, 8 colours) | none | Linux MIPS-style `flush_dcache_page` / `SHMLBA` / colour-aware `copy_user_page`; xv6 needs a colour-aware allocator. Bugs here are silent data corruption. | no (painful, as you said) |
| C. **64 KiB pages**, virtually indexed and physically tagged | **none**: every index bit is inside the page offset, so the cache is indexed exactly as now and only the tag compare uses the translated frame | 64 KiB allocation granularity: 2048 frames in 128 MiB; each xv6 process needs at least ~4 pages (256 KiB); Linux page cache in 64 KiB units | **chosen** |
| D. 32 KiB pages | none | Linux supports 32 KiB only on MIPS. 64 KiB is mainstream (arm64, ppc64, LoongArch) | no; 64 KiB also leaves room for a D-cache with larger ways |

Side benefit: 32 entries × 64 KiB = 2 MiB TLB reach.

Linux implications: `PAGE_SIZE` 64 KiB, `PAGE_OFFSET` `0x8000_0000`,
`TASK_SIZE` `0x8000_0000`, no highmem (128 MiB fits in the 1 GiB direct
segment). The page-table format is entirely software's choice (soft TLB).

### 11.2 Address map with translation on (`STATUS.TE = 1`)

| VA | Mode | Translation |
|---|---|---|
| `0x0000_0000–0x7FFF_FFFF` | user + supervisor | TLB (ASID or global). User programs start above page 0, so NULL faults. |
| `0x8000_0000–0xBFFF_FFFF` | supervisor only | **direct**: PA = VA − `0x8000_0000` (DDR, 128 MiB used), cached. Kernel text, data, direct map, **kernel stacks**. |
| `0xC000_0000–0xEFFF_FFFF` | supervisor only | TLB (global entries). vmalloc, modules. |
| `0xF000_0000–0xFFFF_FFFF` | supervisor only | **direct, identity**, uncached MMIO, as now |

With `TE = 0`, all addresses are physical, as now. The kernel boots with
`TE = 0` at its physical address, sets up, then sets `TE` and jumps to its
`0x8000_0000` alias.

TLB entries may map only DDR frames (PFN = PA[26:16], 11 bits). So
"is MMIO" stays a function of VA[31:28], which keeps every existing
MMIO/posting/hit-pipe decision on the virtual address with no translation in
front of it. Mapping device registers into a user process is not supported;
mapping DDR such as a framebuffer is.

### 11.3 TLB

- 32 entries, fully associative, in FFs and LUTs (no BRAM). Each entry:
  VPN[15:0], ASID[7:0], G, V, R, W, X, PFN[10:0]. `W = 0` is how software
  tracks dirty pages; `V = 0` is how it tracks accessed pages. About 1.3k
  FFs and 0.5k LUTs per lookup port.
- **Software refill**, MIPS style. A miss raises TLB_MISS_LOAD / _STORE /
  _FETCH with `EADDR` = VA. A permission failure raises ACCESS_*. A
  separate `TLBVEC` (0 = use `EVEC`) gives the refill handler its own
  entry point. That is an SR, so it needs no new encoding.
- SRs `0x20+`: `ASID`, `TLB_INDEX`, `TLB_HI` {VPN, ASID, G}, `TLB_LO` {PFN,
  V, R, W, X}, `TLB_RANDOM`, `TLBVEC`. Instructions (class B OPs 9–12,
  proposed): `TLBWI`, `TLBWR`, `TLBP`, `TLBR`, and a flush (all / non-global
  / by ASID). All serialize. Writing `ASID` or the TLB also discards the
  fetch side's virtual copies: window, NLB and BTIC.
- **Data-side lookup** on the EA in EX, with the result **registered** into
  MEM (frame number, hit, permissions). The cache keeps reading its BRAMs
  with VA index bits on the EX→MEM edge (VIPT). The hit pipe's tag compare
  in MEM cycle 1 uses `{registered PFN, VA[15]}` instead of the cache's own
  registered address tag, the same logic depth as now. Faults use the §7.1
  MEM-cycle-1 slot, so a store is never posted and an MMIO access never
  issued for a faulting op.
  *Timing risk:* EA adder → CAM → frame mux → flop is a full-cycle path.
  The fallback is to look up the **base register** (no adder in front) and
  replay in MEM when EA and base fall in different pages. The compiler
  would then canonicalize the pointer into rs1 for reg+reg addressing. I
  will decide with a timing experiment before writing the stage 3 note.
- **Two-access ops** (MEMGET32 or LEAVERET crossing a page) translate the
  second page in MEM before either access commits. Reads have no side
  effects, so a fault there is still all-or-nothing.
- **Instruction side:** a 1–2 entry micro-ITLB feeds the I-cache tag compare
  as a register. A micro-ITLB miss stalls fetch while the main TLB is looked
  up through a second port. A miss or permission failure there serves a
  poisoned window, and ID turns it into a TLB_MISS_FETCH / ACCESS_FETCH XOP
  (the §7.6 mechanism, generalized with a per-slot fault tag).
- **Ordering and coherence with virtual addresses.** Store-to-load
  forwarding requires an exact VA match. The "store to this line in flight"
  block compares page offsets only, which is conservative for aliases.
  Self-modifying-code squash checks compare VAs for the fetch window. The
  I-cache store snoop compares physical tags, which is correct for any
  alias. Code written through a *different* mapping (kernel writing user
  text) needs ICACHE_INV, which is what Linux's `flush_icache_*` hooks
  already provide.
- **Frame pushes are not translated.** With `TE = 1` the kernel SP must be
  in the direct segment. Otherwise the core takes a double fault and parks
  with `park_kind` 6. Linux: no `VMAP_STACK`.

### 11.4 LR/SC

`LR.D rd, [rs1]` and `SC.D rd, rs2, [rs1]` (rd = 0 on success, 1 on
failure), with 64-bit and 32-bit forms. There is one reservation (address +
valid). It is cleared by SC, by any interrupt or exception entry, and by
IRET. It is atomic only against this core. DMA, the blitter and core 2 are
not snooped, as with the I-cache today. Proposed encodings use bits that are
illegal today: SC = class 7 with SGN `[23]` = 1 (reserved for stores), and
LR = class 6, LEN = 01, MODE = 00 with `[12]` = 1 (in MODE = 00, `[19:12]`
must be 0 today; in the short MODE = 01 form those bits are the imm8, so
they are not free there). They will be fixed in the stage 3 note.

---

## 12. Open questions for the runtime/OS side

1. SYSCALL frame PC = **next** instruction (no `+4` in the kernel). Or do you
   prefer RISC-V style (the SYSCALL's own PC)?
2. Exceptions set DI, interrupts don't (§3.3). Does that work for the
   Zephyr port's locking? Note that IRET restores `INT_MASK` from the frame,
   so a handler's `INT_MASK` change is undone by its IRET, exactly as for
   interrupts today.
3. With the JTAG debugger attached, ILLEGAL, TRAP and MISALIGNED halt into
   the debugger even when an OS would handle them (for example misaligned
   emulation). Keep that, or add a debug-module control bit that passes
   them to `EVEC`?
4. User mode has no clock. Do you want a user-readable cycle or ms counter
   as an SR (like `STATUS`, unprivileged), or is a syscall fine?
5. Is all MMIO privileged in user mode (§3.6) right for Zephyr and NuttX
   user threads?
6. Are 64 KiB pages acceptable for your xv6 and Linux plans (§11.1)?
7. klausscc (assembler and emulator, used for the xsim goldens) also needs
   SYSCALL/MFSR/MTSR and the exception model. Who takes it: you, me, or
   later?

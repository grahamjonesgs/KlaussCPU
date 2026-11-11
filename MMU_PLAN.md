# MMU plan: exceptions, user mode, MPU, TLB

Branch `mmu` (klaussCPU, with klausscpu-runtime, klausscpu-llvm and klausscc
on their own `mmu` branches). Status: **stage 1 interface (§3–§6) frozen
2026-11-09** after review by the runtime session, which approved it with
two additions: the TP and CYCLE SRs, and LR/SC moved into stage 1. No RTL
yet. Any change to §3–§6 from here on is agreed with the runtime session
first.

Who does what: RTL, kasm.py and the klausscpu-llvm MC layer are done here.
klausscc (assembler, emulator, GDB server) and all runtime and OS software
are done by the klausscpu-runtime session.

Goal: a path to xv6 and then Linux (arch/klausscpu + musl), with Zephyr
USERSPACE and NuttX protected and kernel builds along the way. Today there
are no privilege levels, and TRAP, illegal instructions and misaligned
accesses park the core.

| Stage | Contents | Detail here |
|---|---|---|
| 1 | Precise restartable exceptions, user and supervisor modes, SYSCALL, kernel stack switch, TP/CYCLE SRs, LR/SC | full (§2–§9) |
| 2 | MPU: 16 base/limit/permission regions, for Zephyr USERSPACE and NuttX protected | full (§10) |
| 3 | Software-refilled TLB with ASIDs, 64 KiB pages | full (§11) |

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
5. **User-mode atomics and TLS.** LR/SC (§5.1), because DI/EI, which
   `src/atomic.c` uses today, are privileged in user mode. A thread-pointer
   SR (TP) and a cycle counter SR (CYCLE), both usable from user mode.

What stage 1 does **not** give: memory protection between user programs,
or between user and kernel *DDR*. That is stage 2 (MPU) or stage 3 (TLB).
Stage 1 protects only the CPU control state and the devices.

---

## 3. Programmer's model (stage 1)

### 3.1 New state

| State | Width | After reset | Notes |
|---|---|---|---|
| `UM` | 1 | 0 | current mode: 1 = user |
| `SP_ALT` | 32 | 0 | the inactive stack pointer. In supervisor mode it holds the **user** SP. In user mode it holds the **kernel** SP. Accessed as SR `USP` from supervisor mode. |
| `EVEC` | 32 | 0 | exception vector; bits [1:0] read as 0 |
| `ECAUSE` | 8 | 0 | cause of the last exception (§4.1) |
| `EADDR` | 32 | 0 | fault address or instruction word of the last exception |
| `SCRATCH0/1` | 64 each | 0 | free for the kernel (per-CPU / current-thread pointer) |
| `TP` | 64 | 0 | thread pointer, no hardware meaning; read and write in either mode. The kernel context-switches it. |
| `CYCLE` | 64 | 0 | core cycles since the last reset, read-only, readable in either mode. Every program load resets the core, so in practice this is cycles since the program was loaded. It counts every core clock (`ce`), including while parked or debug-halted. |
| reservation | 32 + 1 | invalid | LR/SC reservation (§5.1) |

The architectural `SP` (GETSP/SETSP/PUSH/…) is always the current mode's
stack pointer. The hardware never touches `SP_ALT` except on a mode change,
so a supervisor-only system never sees it.

Every row above is reset with the core. On core 1 the SoC holds the core
in reset through every program load (`w_pip_hold_rst` in KlaussCPU.sv), so
`start` always follows a reset and each program starts with all of this
state at its reset value. `start` on its own also clears `UM`, `EVEC`,
`ECAUSE`, `EADDR` and the reservation; it does not clear `SP_ALT`,
`SCRATCH0/1`, `TP` or `CYCLE`, but on core 1 it never comes alone.
Debugger resume clears nothing.

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
4. Both kinds clear the LR/SC reservation.

Exceptions disable interrupts so that a nested interrupt handler cannot take
a fault (stage 3: a TLB miss) and overwrite `ECAUSE`/`EADDR` before the
exception handler has read them. The old DI bit is in the frame, and IRET
restores it. Interrupts keep today's behaviour and leave DI alone. A kernel
that wants interrupt entry with interrupts disabled starts its handler with
`DI`.

Exceptions are synchronous: they are taken whatever DI and `INT_MASK` say.

**Precise EI (2026-11-09, `d9cf6c2`).** An `EI` that sets enable = 1
while an interrupt is pending and unmasked takes it at the boundary
immediately after the `EI`, before the next instruction executes (RISC-V
semantics): the frame's PC is EI+4. So `EI; DI` takes a pending interrupt
between the two, which xv6's scheduler loop (`intr_on(); intr_off()`) and
Linux `local_irq_enable()` / `local_irq_disable()` pairs rely on. An `EI`
with no interrupt pending costs nothing. A debugger single-step never takes
an interrupt. `IRET` restoring enable = 1 and `MTSR STATUS` (both
serializing) take a pending interrupt at the first boundary after they
commit. (Implementation: an `EI` latched with an interrupt pending
serializes, commits at WB and refetches at EI+4; the next boundary takes
the interrupt.)

### 3.4 IRET

IRET pops the frame from the current SP (it is privileged, so this is the
kernel SP) and restores PC, Z/S/C/V, INT_MASK, DI and **`UM = PM`**. It
also clears the LR/SC reservation. If
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
of any SR except `STATUS`, `TP` and `CYCLE`, and `MTSR` of any SR except
`TP`.

Legality is checked before privilege: an encoding that is ILLEGAL in
supervisor mode (an SR number above 8, MTSR to CYCLE) is ILLEGAL in user
mode too, not PRIVILEGED.

Everything else is allowed, including LR/SC. `TRAP` raises TRAP in either
mode, and `SYSCALL` raises SYSCALL in either mode.

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
| 5 | MISALIGNED_LOAD | arch §8.3 trap rows, loads; LR not naturally aligned | the instruction | effective address |
| 6 | MISALIGNED_STORE | same, stores; SC not naturally aligned | the instruction | effective address |
| 7 | ACCESS_LOAD | user MMIO read (§3.6); LR to MMIO in either mode; stage 2/3 read permission | the instruction | effective address |
| 8 | ACCESS_STORE | user MMIO write; SC to MMIO in either mode; stage 2/3 write permission | the instruction | effective address |
| 9 | ACCESS_FETCH | user fetch from `0xF…` (§3.6); stage 2/3 execute permission | the instruction | address of its first faulting word |
| 10 | MISALIGNED_FETCH | user-mode PC with `[1:0] != 0` | the PC | the PC |
| 11–13 | TLB_MISS_LOAD / _STORE / _FETCH | stage 3 (§11.3) | the instruction | faulting VA |
| 14–255 | reserved | | | |

- **Priority** when several causes apply to one instruction:
  MISALIGNED_FETCH > ACCESS_FETCH > ILLEGAL > PRIVILEGED > (TRAP, SYSCALL)
  > MISALIGNED_LOAD/STORE > ACCESS_LOAD/STORE. So a misaligned user access
  to MMIO is MISALIGNED, and an LR/SC is checked for alignment before MMIO.
- **EADDR for data faults** is the effective address as computed (base +
  offset), before any alignment: a 64-bit A=1 access and a sub-word store
  report the raw address, not the aligned-down one. The exception is a
  two-access instruction whose first doubleword is fine and whose second
  faults: it reports the start of the second doubleword (the lowest
  faulting byte).
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
| SC | store (only on success), rd, reservation cleared | none: no store, rd unchanged. The SC itself does not clear the reservation, but its exception entry does (§3.3 step 4), so the SC restarted by IRET fails (rd = 1) and the LR/SC loop retries from the LR. Correct, just one extra loop pass |
| LR | load, rd, reservation set | none; the reservation is not set |

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

1. **Debugger attached** (`DMCONTROL[3]`), **`DMCONTROL[5]` EXC_PASS = 0**,
   and cause ILLEGAL, TRAP, MISALIGNED_LOAD or MISALIGNED_STORE (not
   MISALIGNED_FETCH, which is new and user-mode-only): debug halt with today's
   `dbg_cause` (TRAP/ILL), in either mode. This is unchanged and is the
   default. With EXC_PASS = 1 these causes skip this step and go to step 2.
   EXC_PASS is cleared on program load like `attached`, unless
   `halt_on_load` is set. The debug module's `DMSTATUS` version goes from 1
   to 2, so a GDB server can tell that EXC_PASS and register indices 20/21
   (§6.2) exist.
2. **`EVEC = 0`**: park as today. ILLEGAL → `park_kind` 2, TRAP → 1,
   MISALIGNED_* → 5. Any new cause (PRIVILEGED, SYSCALL, ACCESS_*,
   MISALIGNED_FETCH) → 2, the same crash an old bitstream gives for an
   undefined opcode. A debugger, if attached, halts on these too.
   Parking does **not** write `ECAUSE` or `EADDR`; only exception entry
   does.
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
- **MTSR serializes**, for every SR including TP. Fetch stops behind it and
  restarts at the next instruction once it has committed, so a changed SR is
  seen by everything after it, including fetch (this matters from stage 2
  on). The cost is roughly one pipeline refill (about 5 cycles), paid only
  at context switch and thread setup.
- **MFSR** reads the committed SR. No forwarding is needed: every SR writer
  is either serializing (MTSR) or happens with the pipe drained (entry,
  IRET). `MFSR rd, TP` is an ordinary 1-issue-slot instruction, so TLS
  access costs one instruction.
- **MFSR CYCLE is not serializing.** It reads the counter in its EX cycle.
  Older instructions may still be finishing in MEM/WB (for example a load
  miss waiting for DDR), and younger ones have not started. It is not
  ordered against memory. For timing an interval, the error is a few cycles
  plus any outstanding miss. Software that needs a hard order puts a
  serializing instruction (for example `MTSR TP, rX` with its own value)
  before it.
- MFSR or MTSR with an SR number not in §6 is ILLEGAL. Software can trust
  that an SR it reads exists.
- With the debugger attached, SYSCALL behaves as SYSCALL (no halt).

Assembler syntax: SR operands by name (`status`, `evec`, `ecause`,
`eaddr`, `usp`, `scratch0`, `scratch1`, `tp`, `cycle`) or number.

### 5.1 LR/SC

| Mnemonic | Template | Fields | Semantics |
|---|---|---|---|
| `LR.D rd, [rs1]` | `0x5B00_1000` | class 6, LEN = 01, SIZE `[25:24]` = 11, SGN `[23]` = 0, MODE `[22:21]` = 00, A `[20]` = 0, **`[12]` = 1**, rd `[11:8]`, rs1 `[7:4]`, `[3:0]` = 0 | rd = mem64[rs1]; reserve |
| `LR.W rd, [rs1]` | `0x5A00_1000` | as LR.D with SIZE = 10, SGN = 0 | rd = zext(mem32[rs1]); reserve |
| `LR.W.S rd, [rs1]` | `0x5A80_1000` | as LR.W with SGN = 1 | rd = sext(mem32[rs1]); reserve |
| `SC.D rd, rs2, [rs1]` | `0x5F00_1000` | class 7, LEN = 01, SIZE = 11, SGN = 0, MODE = 00, A = 0, **`[12]` = 1**, rd `[11:8]` = **result**, rs1 `[7:4]` = address, rs2 `[3:0]` = **data** | if reserved: mem64[rs1] = rs2, rd = 0; else rd = 1, no store. Always clears the reservation. |
| `SC.W rd, rs2, [rs1]` | `0x5E00_1000` | as SC.D with SIZE = 10 | 32-bit store of rs2[31:0] |

These encodings are illegal today. In MODE = 00, `[19:12]` must be 0. The
short MODE = 01 form uses those bits as its imm8, so they are only free in
MODE = 00. **SC's register fields differ from the other stores:** rd is the
result, and the data comes from rs2. Other bit combinations with `[12]` = 1
(8/16-bit size, A = 1, SGN = 1 on LR.D or on SC, other modes) are ILLEGAL.

Rules:
- The address must be naturally aligned (4 or 8), or the instruction
  raises MISALIGNED_LOAD / MISALIGNED_STORE. This applies to 64-bit too,
  unlike ordinary 64-bit accesses.
- LR or SC to MMIO (`0xF…`) raises ACCESS_LOAD / ACCESS_STORE in **either**
  mode. These are new instructions, so this is no compatibility change.
- There is **one reservation**: a doubleword address `EA[31:3]` plus a valid
  bit. LR sets it when it commits. SC succeeds only if it is valid and
  `EA[31:3]` matches. LR.W/SC.W use the doubleword that contains the word,
  so an SC.W to the other half of the reserved doubleword also succeeds. Do
  not rely on that.
- The reservation is cleared by every SC (success or failure, but not a
  faulting SC), by interrupt and exception entry, by IRET, and by `start`.
  Ordinary stores do not clear it.
- It is atomic only against this core. DMA, the blitter, the debugger's
  memory writes and core 2 are not tracked, as with the I-cache today.
- An interrupt between LR and SC makes the SC fail, so the loop retries.
  There is no forward-progress guarantee beyond "the loop is shorter than
  the interrupt interval".
- The SC result is architectural: 0 = success, 1 = failure. Other values
  are reserved.

Typical compare-and-swap (64-bit):

```
retry:  LR.D   r3, [r1]          ; r1 = address
        FBR.NE r3, r4, fail      ; r4 = expected
        SC.D   r6, r5, [r1]      ; r5 = new value, r6 = status
        FBR.NE r6, #0, retry
fail:
```

---

## 6. System registers and feature probe

### 6.1 SR numbers (stage 1)

| SR | Name | Access | Contents |
|---|---|---|---|
| 0x00 | `STATUS` | R (**user mode too**), W (supervisor) | `[0]` UM (read-only), `[1]` DI (read-only, the committed bit), `[8]` MPU_EN (stage 2), `[9]` TE (stage 3), `[10]` MAL64 (HW_PERF_PLAN.md §4: 1 = a 64-bit load / store with EA[2:0] ≠ 0 raises MISALIGNED_*; 0 after reset and at program load). Other bits read 0. Writes to UM and DI are ignored; use IRET / DI / EI. |
| 0x01 | `EVEC` | RW | exception vector, `[1:0]` read 0 |
| 0x02 | `ECAUSE` | RW | §4.1 code in `[7:0]`; `[8]` GUARD = this ACCESS_* fault was a denial by an MPU GUARD region (stage 2, §10.2), else 0 |
| 0x03 | `EADDR` | RW | `[31:0]` |
| 0x04 | `USP` | RW | `SP_ALT`, which is the user SP while in supervisor mode |
| 0x05 | `SCRATCH0` | RW | 64-bit |
| 0x06 | `SCRATCH1` | RW | 64-bit |
| 0x07 | `TP` | RW **in user mode too** | 64-bit thread pointer, no hardware meaning |
| 0x08 | `CYCLE` | R **in user mode too** | 64-bit core-cycle counter (§3.1, §5); MTSR to it is ILLEGAL |
| 0x10–0x2F | `MPU_BASEn` / `MPU_LIMITn` | supervisor | stage 2 (§10.1) |
| 0x30–0x34 | `TLB_HI`, `TLB_LO`, `TLB_INDEX`, `ASID`, `TLBVEC` | supervisor | stage 3 (§11.3) |

`STATUS` is readable in user mode so that code like Zephyr's
`arch_is_user_context()` can test the mode without a syscall. `TP` serves
musl/Linux TLS, Zephyr `THREAD_LOCAL_STORAGE` and Rust `thread_local`; the
ABI has no free register for it. In user mode, MFSR is allowed only for
{STATUS, TP, CYCLE} and MTSR only for {TP}.

### 6.2 Debugger

The debug module gets two more register indices: 20 = `SP_ALT`, 21 =
`UM` in bit 0 and `DI` in bit 1, the same layout as `STATUS` (read;
write allowed while halted). It also gets
`DMCONTROL[5]` EXC_PASS (§4.4), and its `DMSTATUS` version becomes 2.

### 6.3 `CPU_FEATURES` (MMIO, read-only)

At `0xF00F_0048`, in the timer/IRQ block. It reads 0 on every older
bitstream.

| Bits | Meaning |
|---|---|
| `[0]` | stage 1: exceptions, user mode, SYSCALL, MFSR/MTSR, TP, CYCLE |
| `[1]` | stage 2 MPU present; `[7:4]` = region count − 1 (§10.1). An optional build feature, **off on the Nexys A7** (timing margin, Graham 2026-11-09): `[1]` = 0, `[7:4]` = 0 |
| `[2]` | stage 3 TLB present (`[15:8]` = entry count) |
| `[3]` | LR/SC (§5.1). Set together with `[0]` in stage 1, but kept separate so software tests for atomics by name. |
| `[16]` | AES-128 present (optional build feature) |
| `[17]` | AES-GCM / GHASH present (only with AES) |
| `[18]` | SHA-256 present |
| `[19]` | HMAC-SHA-256 present (only with SHA-256) |
| `[20]` | TRNG present (always built: always 1) |
| `[21]` | native SD controller (`SD_CTRL[18]` SD_MODE, `SDM_*` at `0xF000_0100`; SD_NATIVE_PLAN.md, branch `mmu-sd`) |
| `[22]` | DCBZ (HW_PERF_PLAN.md §1; branch `perf-hw`) |
| `[23]` | `STATUS[10]` MAL64 (HW_PERF_PLAN.md §4; branch `perf-hw`) |

If `CPU_FEATURES[20]` = 0, the bitstream predates the crypto presence bits: assume every crypto block is present. (A future bitstream without a TRNG would therefore read as 'all present'; that is acceptable while the TRNG is mandatory.) An absent crypto block's MMIO window reads 0, ignores writes and
completes in the normal MMIO time (MMIO_MAP.md).

The lean Nexys A7 MMU bitstream reads **`0x0010_100D`**: stage 1, LR/SC, a
16-entry TLB, no MPU, and the TRNG as the only crypto block. The full
crypto configuration reads `0x001F_100D`. With the native SD controller
(`mmu-sd`), `[21]` is also set: `0x0030_100D` lean, `0x003F_100D` full.
With DCBZ and MAL64 (`perf-hw`), `[23:22]` too: `0x00F0_100D` lean,
`0x00FF_100D` full.

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

### 7.7 MFSR / MTSR datapath (as built)

- **MTSR** decodes as a flag-less COPY of rs1 (`dec_t.mtsr`, serializing).
  It writes its SR at WB, then restarts fetch at `wb_pc + 4`: the
  RESET-style redirect, done at WB.
- **MFSR of any SR except STATUS** decodes as `U_MOV64`. Its 64-bit value
  is loaded into `ex_var1`/`ex_var2` at dispatch, from a mux on `id_op[15:12]`
  selected by a latch-time predecode (`id_mfsr`), so `exo_result` gains no
  leg. CYCLE is sampled on the edge the MFSR enters EX.
- **MFSR STATUS** is its own uop (`U_GETST`) with a 2-bit `exo_result` leg
  `{ex_dis_eff, umode}`. Its DI bit is forwarded from an older DI/EI in
  MEM/WB, exactly as DI's own result is. That is program-order correct,
  which is stronger than "the committed bit" in §6.1.

### 7.7a LR/SC datapath (as built)

- LR and SC take the **slow port path**: no load-hit pipe, no early issue.
  Misaligned and MMIO checks use the §7.1 slots (EX→MEM for misaligned,
  MEM cycle 1 for ACCESS).
- **The reservation** is a register pair `{res_valid, res_dw}`. LR sets it
  as it commits at WB.
- **Deviation:** SC decides success **in EX**, not in MEM. It uses the
  reservation as it will be once the older LR/SC in WB, then in MEM, has
  committed. Both always commit: a faulting one is no longer an LR/SC
  uop, and a squash of either kills this SC too. SC is `[rs1]`-only, so the
  compare uses `ex_a` with no adder in front. The 0/1 result is the SC's EX
  result (`mem_result[0]`, so MEM forwarding of rd is correct), and the
  port write is `U_SC && !mem_result[0]`, a register term. Deciding in MEM
  would have put a 29-bit compare on `mem_port_wr`, which feeds `mem_busy`.
- **Deviation:** a successful SC is an ordinary store, and it **may be
  posted** (`EN_ST1`/`EN_SQ`) like any other. That is architecturally
  invisible on one core and keeps SC off the post-decision paths.
- SC reads its data from rs2 (`ex_b`). Its lane-replicated data is what the
  retire trace prints as the stored value.
- SC's store goes through the normal self-modifying-code and store-snoop
  checks.
- A faulting SC leaves rd and memory alone, and does not itself clear the
  reservation. Its exception entry does (§3.3 step 4), so the retried SC
  fails and the LR/SC loop starts again. §4.2 originally said the retry
  could still succeed, which contradicted §3.3/§5.1. It was corrected on
  2026-11-09 to match this behaviour.

### 7.7b Other implementation notes

- **Access faults** (§7.1) are converted at EX→MEM into `U_EXC`, with
  every MEM-side effect flag cleared (SP write, flags, DI/EI, load class,
  hit pipe, RAS prediction). `mem_xsq` then squashes EX/ID through the
  `fbr_taken` shape. The gated EX side effects are the LCD strobe, LCDRST,
  the RAS push, early load issue and PHT training; DIV and DELAY are reset.
  The squash runs before the SMC and `EN_ST1` squashes, so an older store's
  squash still wins.
- **User fetch from 0xF…** is a **fake fill**. The miss-issue branch issues
  it with no DV on the port (`if_fake`). It completes on the next cycle in
  a branch of its own: zeros into the window, no I-cache install. The
  0xF… test (`w_ufz`) only steers `m_read_DV`/`if_fake`, which keeps it off
  the `ic_hit` serve chain. The first build's zero-serve there was its worst
  path. The BTIC never captures a window whose base is in 0xF…, so a zero
  window can never become a BTIC entry.
- **Latch-time predecode** (`w_decf`: `id_ser` and the branch / return
  predictor bits) is the plain decode, without the MMU overrides. A fetch
  fault gates the predictor bits where they are used (`id_bp_tk` and the
  RAS dispatch) with the registered `id_fx`. The overrides' serialize is a
  parallel OR term (`w_xser`) into `id_ser`. A privileged op is never a
  branch or a return. The overrides on that predecode cone were the first
  build's second-worst path.
- **Wrong-path fetch toward MMIO** could not be constructed in this
  pipeline. While EX is busy nothing new latches into ID, so a wrong-path
  JMP's early redirect only fires on the edge its older branch resolves,
  and a long-lived wrong path (RET, load-dependent branch) holds the
  memory port, so no I-cache fill can issue. The fill block is
  unconditional on the fill path. It is tested with a correct-path jump
  and with early fetch behind a DIV (`user_early_fetch`, with a supervisor
  control that does read MMIO).
- **Fetch-side user mode** (`fe_um`) is `UM`, or an IRET returning to user
  mode sitting in WB. UM commits at WB, a cycle after the IRET redirected
  fetch from MEM, and a lookup pending from before the IRET can issue its
  fill in that cycle. The independent review found that it issued a real
  fill at a user-mode MMIO target there (test `iret_user_mmio_target`).
  `fe_um` drives the fill engine's fake-fill decision and the IF→ID latch
  checks. The EX-side and sequencer uses keep `umode`: nothing is in flight
  behind a committing IRET (asserted).
- **BTIC**: capture refuses a window whose base is in 0xF… in supervisor
  mode too. That is performance-only; no code runs from 0xF….
- **`start` with an exception entry in progress** (`exc_ent`) is
  unreachable on core 1: the SoC resets the core before every `start`.
- **Exception entry** reuses the IRQ sequencer. `exc_ent` marks an
  exception: no abandon, EPC in the frame, EVEC, DI, ECAUSE/EADDR on
  completion.
- **Tests**: `tools/vsim/tests.py` (`exc_*`, `user_*`, `lrsc_*`, `clar_*`,
  `sr_roundtrip`), `run.py excdiff` (random programs with injected
  exception ops, user and supervisor, against the plain program), `run.py
  dmtest` (`tb_dm.sv`), and `VSIM_EXC_EN=0 run.py diff` for the core-2
  configuration.

### 7.8 Build switches and core 2

`EN_EXC` (stage 1), `EN_MPU`, `EN_TLB`, as localparams in the existing
`EN_*` style. Core 2 (AMP) gets a module parameter, default off, because it
has no use for user mode and its area matters. With `EN_EXC = 0`, the new
encodings are illegal and nothing else changes.

### 7.9 Cost (measured)

The plan estimated about 500 FFs and well under 1k LUTs. Measured:

| Measure | LUTs | FFs |
|---|---|---|
| Placed, whole design, vs master (`qspi2` build): first stage 1 build | +2,214 (39,285 → 41,499) | +655 (27,903 → 28,558) |
| Placed, whole design, vs master: after the §7.10 timing work (`273bd78`) | **+1,015** (39,285 → 40,300) | +603 (27,903 → 28,506) |
| Post-synthesis, `pipeline_core_i` in the design | +1,744 (15,077 → 16,821) | +735 |
| Out-of-context `pipeline_core`, `EXC_EN` 0 → 1 | **+757** (15,338 → 16,095) | +619 |

The features themselves cost about 760 LUTs, in line with the estimate.
The rest of the in-design increase is not attributable to a feature: the
same RTL synthesized in context restructures the timing-critical cones
(LUT-name families shift by hundreds in both directions, e.g. `ex_c` +664,
`wb_value` −499), and placement adds timing-driven replication.
Out-of-context split of the +757 LUTs:

| Feature | LUTs | FFs |
|---|---|---|
| User-mode checks: privileged decode, fetch and data access faults | ~216 | ~59 |
| TP, SCRATCH0/1, CYCLE (registers, MFSR mux, 64-bit counter) | ~111 | ~246 |
| LR/SC | ~76 | ~0 |
| The rest: exception ops and records, entry/IRET stack switch, EVEC/ECAUSE/EADDR/USP, MFSR/MTSR | ~354 | ~314 |

No BRAM or DSP. The timing exposure was the fill-serve chain and the
latch-time predecode (§7.7b). Both are now off the new logic.

### 7.10 Timing closure (2026-11-09)

Goal: WNS ≥ 0 with the standard `build_fast` flow, by design changes,
not by alternate placement. Every step keeps the sim set clean and is
CPI-neutral: the master diffs and `socdiff` cycle totals are unchanged.

| Build | Commit | Change | Tier 1 WNS | Final WNS / TNS | LUTs | FFs |
|---|---|---|---|---|---|---|
| master reference | (`qspi2`) | — | −0.225 | −0.015 / −0.102 | 39,285 | 27,903 |
| first stage 1 | `cd4fcdf` | stage 1 RTL | −0.516 | −0.324 / −20.131 | 41,499 | 28,558 |
| | `40037a6` | fake fill; plain latch predecode (§7.7b) | −0.324 | −0.252 / −7.478 | 41,397 | 28,471 |
| | `28d09b6` | + review fixes (`fe_um`, `sq_spec`) | −0.442 | −0.180 / −4.071 | 40,435 | 28,496 |
| step 1 | `660ecfd` | MIN/MAX select moved to MEM | −0.054 | **+0.000** / 0 (tier 2) | 40,577 | 28,543 |
| step 2 | `5705342` | SRs in a kept-hierarchy `mmu_sr` | −0.303 | −0.298 / −10.124 | 40,808 | 28,556 |
| step 3 | `24a6f56` | SR file in LUTRAM with valid flops | +0.008 | **+0.008** / 0 (tier 1) | 40,453 | 28,576 |
| step 4 | `273bd78` | bus_splitter response data select registered | +0.086 | **+0.086** / 0 (tier 1) | 40,300 | 28,506 |
| (check) | `273bd78` without `keep_hierarchy` | | +0.028 | +0.028 / 0 (tier 1) | 40,249 | 28,443 |

- **Step 1** removed the three worst paths: `ex_w` → compare → a fanout-81
  64-bit select → `mem_result`. EX now registers the MIN/MAX pick, opa and
  opb; `mem_result_eff` selects.
- **Step 2 on its own** showed nothing attributable: none of its worst
  paths touch `mmu_sr`, and the build-to-build spread is about ±0.3 ns.
  With and without `keep_hierarchy`, the final configuration is +0.086 vs
  +0.028. It is kept; it is cheap either way.
- **Step 3** keeps EVEC, ECAUSE and CYCLE in flops (see `mmu_sr`'s
  header); EADDR, SCRATCH0/1 and TP are LUTRAM.
- **Step 4** took the worst remaining tier-1 path (`m_addr` → bus_splitter
  `is_mmio` → read-data select → `m_rdata` → RET target / RAS check →
  `pc` / `pos`) off the live address decode. Only the DATA select is
  registered; READY stays live (a registered ready select showed the MMIO
  ready's tail for a cycle against the next request — caught by `run.py
  socmmio`).
- **Remaining near-critical** at `273bd78` (all pre-existing):
  `ex_b` → the I-cache tag shadow address (+0.086), and `m_addr` →
  bus_splitter's live READY select → the window load enables / `pos`
  (+0.16–0.19).
- **One MMU-added path** showed up as a violator in one build (`5705342`,
  −0.298): `ex_var1` → EA adder → `[31:28] == F` → the access-fault
  conversion of `mem_d.uop`. If it reappears, the fix is to register the
  fault and gate the MEM-side consumers (`mem_port_rd/wr`, `wb_ld`, the
  RET redirect) with `mem_xsq` instead of converting `mem_d.uop` at
  EX→MEM.

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
     a DIV in EX, and an LCD op in EX;
   - LR/SC: success; failure after an interrupt, after an exception, after
     IRET, and after an intervening SC; a mismatched address; LR or SC
     faulting (misaligned, MMIO) with the reservation unchanged and rd
     unchanged; an SC immediately behind its LR (the WB forward); SC in user
     mode;
   - TP read and write in both modes; CYCLE monotonic; the user-mode
     MFSR/MTSR allow-list, with every other SR PRIVILEGED;
   - the debugger's EXC_PASS bit.
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

1. Done: this note was reviewed and §3–§6 frozen. The encodings are in
   `ISA_ENCODING_V2_MAP.md` §9. `CPU_ARCHITECTURE.md` gets its exceptions
   section when the RTL lands.
2. Toolchain: MC-layer support for SYSCALL, MFSR, MTSR (named SRs) and
   LR/SC in klausscpu-llvm (on its `mmu` branch) and kasm.py, done here.
   klausscc is done by the runtime session. Until then, `.word` works.
3. RTL in steps, each passing the compatibility diff:
   (a) XOP plumbing and WB→entry with `EVEC`, `ECAUSE`, `EADDR`,
   MFSR/MTSR (including TP, CYCLE), SYSCALL (supervisor only);
   (b) `UM`, `SP_ALT`, IRET/entry stack switch, the privileged decode;
   (c) user MMIO data faults (the MEM-cycle-1 slot), then the fetch checks;
   (d) `CPU_FEATURES`, the debug register indices and EXC_PASS, core-2
   parameter;
   (e) LR/SC.
4. Timing build, board run of the existing suites, then hand the bitstream
   to the runtime session.

---

## 10. Stage 2: MPU

> **Optional build feature** (`EN_MPU` / `MPU_EN`; KlaussCPU
> `CORE1_MPU_EN`). **Off in the Nexys A7 bitstream for timing margin**
> (Graham, 2026-11-09); Zephyr runs in supervisor mode. With it off,
> MFSR/MTSR of SRs 0x10–0x2F is ILLEGAL (§6.1), `STATUS[8]` reads 0 and
> ignores writes, nothing is checked, and hardware never sets `ECAUSE[8]`
> (MTSR ECAUSE can still write it). The RTL
> stays and is simulated in an `MPU_EN = 1` build.

Interface written 2026-11-09 (Graham: carry on through all stages). For
Zephyr USERSPACE and NuttX protected builds, with translation off. Same
rules as stage 1: everything is off at reset, so existing code is
unaffected.

### 10.1 Registers

| SR | Name | Contents |
|---|---|---|
| `0x10 + 2n` | `MPU_BASEn` (n = 0–15) | `[31:5]` first 32-byte granule, `[4]` GUARD, `[3]` X, `[2]` W, `[1]` R, `[0]` EN |
| `0x11 + 2n` | `MPU_LIMITn` | `[31:5]` **last** granule (inclusive); `[4:0]` read 0 |
| `0x00` | `STATUS` | `[8]` MPU_EN (supervisor read/write) |

- 16 regions. `CPU_FEATURES[1]` = 1 and `CPU_FEATURES[7:4]` = region count
  − 1 (15).
- Reset clears every region and `MPU_EN`. The SoC resets the core on
  every program load, so a newly loaded program always starts with the MPU
  off. All MPU SRs are supervisor-only.
  MTSR to them serializes, so the change applies from the next instruction,
  including its fetch.
- Region *n* **covers** address *a* when `EN` and `BASE[31:5] ≤ a[31:5] ≤
  LIMIT[31:5]`. The granularity is 32 bytes, as on ARMv8-M.

### 10.2 Rule

With `MPU_EN = 1` and `TE = 0`, every checked access is decided as follows:

1. If any enabled **GUARD** region covers it: **denied**, in either mode.
   GUARD regions are deny regions, used for stack guards; their R/W/X bits
   are ignored.
2. Otherwise, in supervisor mode: **allowed** (default allow).
3. Otherwise, in user mode: allowed only if some enabled non-GUARD region
   covering it has the permission. Loads need R, stores need W, and
   instruction fetch needs X. Regions combine as a union with no priority,
   so user mode is default deny.

Denied accesses raise ACCESS_LOAD / ACCESS_STORE / ACCESS_FETCH. `EADDR`
follows §4.1: the raw EA, or the start of the second doubleword. For a fetch
it is the first denied word of the instruction.

What is checked:
- Every data access by an instruction, including stack operations (by
  direction), LR (load) and SC (store). For a two-access instruction
  (MEMGET32 spanning two doublewords, LEAVERET), **both** doublewords are
  checked before either access happens.
- Every instruction fetched, at every doubleword it occupies.
- **Not checked:** the frame push of exception or interrupt entry (so a
  stack overflow into a guard cannot double-fault; leave one frame of slack
  below a guard); debugger memory accesses; speculative I-cache fills and
  prefetches (DDR reads have no side effects, and the decision is made when
  the instruction is decoded).

**Stack guards need a careful vector (required for any MPU-guarded
kernel).** A supervisor PUSH into a GUARD region faults with SP unchanged.
Entry then pushes its frame, unchecked, at SP − 8, which is inside the
guard. A vector whose first instruction pushes would fault again, and every
nested entry pushes another frame lower: an unbounded descent through the
guard. So the vector must not touch memory until it knows SP is safe.
- Free registers with `MTSR scratch0/scratch1` (no memory access).
- Read `ECAUSE`. **`ECAUSE[8]` (GUARD)** says the fault was a GUARD denial;
  for Zephyr that is directly the "stack overflow" reason.
- If GUARD is set, or SP is near a guard, switch to an emergency stack
  before any push.
There is no hardware backstop for this recursion in stage 2. It is a
software rule.

The stage 1 user-mode MMIO rule (§3.6) still applies first. In supervisor
mode, GUARD regions also cover MMIO. Cause priority stays as in §4.1: an
MPU denial is an ACCESS_* fault. With `MPU_EN = 1`, an instruction fetch
never reads MMIO, even in supervisor mode: the fill returns zeros, so code
executed from `0xF…` raises ACCESS_FETCH with GUARD where a GUARD region
covers it and ILLEGAL elsewhere (with `MPU_EN = 0` it reads the device, as
before).

### 10.3 Pipeline (as built, `d546911` + `da8c085`)

- **State**: `mmu_mpu` (in `pipeline_core.sv`) holds 16 × {BASE[31:5],
  GUARD/X/W/R/EN, LIMIT[31:5]} and MPU_EN in flops (944 + 1). It is written
  by an MTSR to 0x10–0x2F or STATUS as it commits at WB. MTSR serializes,
  so the next instruction, including its fetch, sees the change. MFSR
  reads it at dispatch, from a LUTRAM copy with a valid flop per SR (area:
  not a 32:1 mux of the flops). The MFSR value is selected between `mmu_sr` and
  `mmu_mpu` by SR[7:4]. `mmu_sr`'s writes (and USP's) are gated on
  SR[7:4] = 0, because its low-nibble decode would alias 0x13 to EADDR.
- **Data**: 16 × 2 magnitude compares on the EA's granule (EA[31:5]) in
  EX. **While MPU_EN = 1, a memory op (`dec.is_mem`) spends one extra cycle
  in EX** (`ex_busy`, from registered terms only). The EA is registered
  (`ex_ea_q`) in the first cycle, and the compares run from that register
  in the second. So the EA adder and the compares are never in series: the
  first build with them in series (`d546911`) missed by 0.76 ns on exactly
  that path. Cost: +2.7 % cycles on user-mode random programs with the MPU
  on; with MPU_EN = 0, nothing changes. For a two-access op (MEMGET32 spanning two doublewords, LEAVERET;
  both have EA = `ex_a`) whose 2nd doubleword is in the next granule
  (`ex_a[4:3] = 11`), that granule g+1 is decided from the same compares
  plus two equalities per region: BASE ≤ g+1 ⇔ BASE ≤ g ∨ BASE = g+1, and
  g+1 ≤ LIMIT ⇔ g ≤ LIMIT ∧ g ≠ LIMIT. Here g+1 comes from an incrementer
  on `ex_a`, not on the EA adder. Per doubleword the stage 1 user-MMIO
  rule applies first, then the MPU. The result is only the registered
  access-fault bit of §7.1, plus "2nd doubleword only" (EADDR) and GUARD.
  Since `081952c` that bit is the only thing an access fault changes at
  EX→MEM: the op enters MEM as it is, and MEM gates its port access, WB
  commit, RET redirect and RAS check, and never holds it (§7.1).
- **Fetch**: the same compares on `pc`'s granule. For an instruction
  crossing into the next granule (from `pc[4:2]` and the window's LEN
  bits), g+1 is decided by the same equality trick. This runs at the IF→ID
  latch and is registered into the §7.5 fetch-fault override, with
  `id_fx2` (EADDR = the next granule's start) and `id_fxg` (GUARD). The
  stage 1 user `0xF…` rule applies first per part.
- **ECAUSE[8] (GUARD)** describes the reported (first denied) doubleword
  or fetch part: 1 when an MPU GUARD region denied it, 0 when the stage 1
  MMIO rule did (that rule applies first) or a user default-deny did.
- **Early load issue** (MEMGET32, LEAVERET, IRET) is off while MPU_EN = 1:
  its decision is on the EX→MEM enables, which the MPU decision must not
  reach. With MPU_EN = 0 nothing changes (cycle-identical).
- **Not checked**: entry's frame push; debugger accesses; I-cache fills and
  prefetches, which only read. A user fetch from `0xF…` is still a fake
  fill (§7.7b), and so, with MPU_EN = 1, is a supervisor one (review 2:
  no fetch may read a device the MPU would deny; the cheap rule).
- **Build switches**: the module parameter `MPU_EN` (on; off on core 2),
  and `EN_MPU = EXC_EN && MPU_EN`. KlaussCPU's `CORE1_MPU_EN` drives
  CPU_FEATURES[1] and [7:4] = 15.
- **Tests**: `tests.py` `mpu_*` (one per §10.2 rule, ECAUSE[8] in both
  modes); `run.py mpudiff` runs user-mode random programs under random
  regions, where the handler turns the MPU off and retries on a fault and
  turns it back on at an injected SYSCALL, and the final state must equal
  the plain program's; `socmmio` also checks CPU_FEATURES.

---

## 11. Stage 3: TLB and paging

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

Side benefit: 16 entries × 64 KiB = 1 MiB TLB reach (32 entries: 2 MiB).

Linux implications: `PAGE_SIZE` 64 KiB, `PAGE_OFFSET` `0x8000_0000`,
`TASK_SIZE` `0x8000_0000`, no highmem (128 MiB fits in the 1 GiB direct
segment). The page-table format is entirely software's choice (soft TLB).

### 11.2 Address map with translation on (`STATUS.TE = 1`)

| VA | Mode | Translation |
|---|---|---|
| `0x0000_0000–0x7FFF_FFFF` (useg) | user + supervisor | TLB (ASID-matched or global entry). User programs start above page 0, so NULL faults. |
| `0x8000_0000–0x87FF_FFFF` (kseg) | supervisor only | **direct**: PA = VA − `0x8000_0000` (all 128 MiB of DDR), cached. Kernel text, data, direct map, **kernel stacks**. |
| `0x8800_0000–0xBFFF_FFFF` | — | ACCESS_* fault (no DDR there) |
| `0xC000_0000–0xEFFF_FFFF` (kmap) | supervisor only | TLB. vmalloc and modules; normally global entries. |
| `0xF000_0000–0xFFFF_FFFF` | supervisor only | **direct, identity**, uncached MMIO, as now |

- Any user access (data or fetch) at or above `0x8000_0000` raises
  ACCESS_*.
- With `TE = 0`, every address is physical and the MPU applies (§10). With
  `TE = 1`, the MPU is ignored.
- `TE` is `STATUS[9]` and is set by MTSR STATUS, which serializes. The
  kernel boots with `TE = 0` at its physical address, sets up, then sets
  `TE` and jumps to its `0x8000_0000` alias. **The instruction after the
  MTSR that sets TE is fetched translated.** Code running at a physical
  (useg) address must first install a TLB entry covering its own page,
  typically a global identity entry, or the very next fetch misses. After
  the jump to kseg it can remove that entry. Alternatively, run the
  enabling MTSR from code that is already at an address valid both ways.
- **What must be in kseg while `TE = 1`** (software rule): the targets of
  `EVEC` and `TLBVEC` (the vector fetch is translated like any other), the
  TLB refill handler and every page table it reads, and every kernel
  stack (§11.3). Then the refill path can never miss. A TLB miss inside the
  refill handler would re-enter `TLBVEC` and overwrite `EADDR`, `TLB_HI` and
  `ECAUSE` before the outer handler had used them; there is no hardware
  "exception level" guard against that. Linux and xv6 lay memory out this
  way anyway.
- TLB entries map DDR frames only (PFN = PA[26:16], 11 bits). So "is MMIO"
  stays a function of VA[31:28], with no translation in front of any MMIO
  decision. Mapping device registers into a user process is not supported;
  mapping DDR such as a framebuffer is.

### 11.3 TLB programmer's model

- **Size:** fully associative; the entry count N is a build parameter:
  **16 on the xc7a100t** (§11.5, area), 32 or 64 on a bigger part.
  `CPU_FEATURES[2]` = 1, and `CPU_FEATURES[15:8]` = N (16).
- **TLB_INDEX width:** log2 N bits, `[3:0]` with 16 entries. The bits above
  them (`[4]` with 16 entries) are ignored on write and read 0. Below,
  `TLB_INDEX[4:0]` means those log2 N bits.
- **Entry:** VPN `[15:0]` (= VA[31:16]), ASID `[7:0]`, G, PFN `[10:0]`, V, R,
  W, X.
- **Match:** an entry matches VA when V = 1, its VPN = VA[31:16], and (G = 1
  or its ASID = the `ASID` SR). An entry with V = 0 never matches. If more
  than one entry matches, the PA is some DDR address (never MMIO). That is
  a kernel bug, so software must avoid duplicates.
- **Permissions:** R, W, X per entry, checked for loads, stores and fetches
  respectively. Whether user mode may use the entry comes from the segment
  (useg yes, kmap no), not from an entry bit. `W = 0` is how software
  tracks dirty pages. Accessed tracking is software's (refill on first
  touch).

System registers (supervisor only):

| SR | Name | Contents |
|---|---|---|
| `0x30` | `TLB_HI` | `[31:16]` VPN, `[8]` G, `[7:0]` ASID |
| `0x31` | `TLB_LO` | `[26:16]` PFN, `[3]` X, `[2]` W, `[1]` R, `[0]` V |
| `0x32` | `TLB_INDEX` | `[4:0]` entry index (`[3:0]` with 16 entries; `[4]` ignored, reads 0); `[31]` P = the last TLBP found no match |
| `0x33` | `ASID` | `[7:0]` current address-space ID |
| `0x34` | `TLBVEC` | vector for TLB_MISS_* exceptions; 0 = use `EVEC` |
| `0x00` | `STATUS` | `[9]` TE |

Instructions (class B, LEN = 01, no operands, privileged, serializing):

| Mnemonic | Template | Semantics |
|---|---|---|
| `TLBWI` | `0x6C09_0000` | entry[`TLB_INDEX[4:0]`] ← {`TLB_HI`, `TLB_LO`} |
| `TLBR` | `0x6C0A_0000` | {`TLB_HI`, `TLB_LO`} ← entry[`TLB_INDEX[4:0]`] |
| `TLBP` | `0x6C0B_0000` | search for `TLB_HI`'s VPN with `TLB_HI.ASID` (G-aware). Hit: `TLB_INDEX` ← {P = 0, index}; miss: `TLB_INDEX[31]` ← 1 |
| `TLBINV` | `0x6C0C_0000` | V ← 0 in every entry |

After a TLBP miss, `TLB_INDEX[4:0]` is unchanged and only bit 31 is set.

Clarifications (klausscc emulator questions, 2026-11-09):
- **`EVEC = 0` parks every exception**, TLB misses included. `TLBVEC` is
  only consulted when exceptions are enabled (`EVEC ≠ 0`).
- **Parking and DOUBLE_FAULT** write none of `ECAUSE`, `EADDR` or `TLB_HI`.
- **`TLBVEC[1:0]`** read 0, as for `EVEC`.
- **MTSR `ASID` and MTSR `STATUS`** clear the LR/SC reservation even when
  the value written is unchanged.
- **TLB R/W/X apply in supervisor mode too**, in useg and kmap: the kernel
  needs W on a user page it writes. `copy_to_user` into a page with
  `W = 0` faults, and the kernel handles it, as on MIPS/Linux.
- **A two-access instruction checks the lower doubleword first.** If that
  doubleword faults, its fault is reported, whatever the second doubleword
  would have raised. The second doubleword's fault is reported only when
  the first is clean. This holds for MMIO, MPU and TLB faults alike.
- **A user MMIO access inside a GUARD region** reports the stage 1 MMIO
  ACCESS fault with `ECAUSE[8]` = 0: the MMIO rule comes first.
- **MTSR `ECAUSE`** may set or clear bit 8.

Reset (and so every program load) clears `TE`, `ASID` and `TLBVEC`, and
sets V = 0 in every entry.

Replacement is software's choice: `TLB_INDEX` is plain software state,
typically a round-robin counter. To invalidate a single ASID, software
walks the entries with TLBR/TLBWI.

**Faults.**
- A useg or kmap access with no matching entry raises TLB_MISS_LOAD /
  _STORE / _FETCH (11/12/13). It vectors to `TLBVEC` if that is nonzero,
  else `EVEC`.
- A matching entry without the needed permission raises ACCESS_*.
- On both, `EADDR` = the VA (§4.1 rules), and hardware also loads
  `TLB_HI` ← {VPN of `EADDR`, G = 0, current `ASID`}. A refill handler
  therefore only has to find the PTE, write `TLB_LO`, pick an index and
  run `TLBWI`.
- **Priority:** within the §4.1 order, for one access: MISALIGNED >
  ACCESS (segment, MMIO, kseg beyond DDR) > TLB_MISS > ACCESS (TLB
  permission). Two-access instructions translate both doublewords before
  either access, and report the lowest faulting byte.

**Side effects of TLB changes.** TLBWI, TLBINV, MTSR `ASID` and MTSR
`STATUS` serialize. They also discard the fetch side's virtual copies (the
window, the NLB and the BTIC) and clear the LR/SC reservation. The I-cache
and D-cache are physically tagged and are kept. The reservation holds a VA.
Because every TLB or ASID change clears it, a VA is enough.

**Frame pushes are not translated.** With `TE = 1`, the stack an entry
pushes on (the kernel SP) must be a kseg address, and the push writes PA =
SP − 8 − `0x8000_0000`. The check is on the push address SP − 8: SP = `0x8800_0000` works, but SP =
`0x8000_0000` (push to `0x7FFF_FFF8`) does not. If SP − 8 is not in kseg,
the core parks with
`park_kind` 6 (DOUBLE_FAULT), or debug-halts if a debugger is attached. For
Linux: no `VMAP_STACK`.

**Coherence.** Code written through a different mapping (for example the
kernel writing user text through kseg) needs ICACHE_INV, which Linux's
`flush_icache_*` hooks provide. Same-VA self-modifying code still works as
today. Debugger memory accesses use physical addresses.

### 11.4 Pipeline approach

*As built (§11.5), the data lookup runs off the registered EA in the MPU's
extra EX cycle instead of the base-register lookup with a MEM replay below;
§11.5 says why.*

**Decision (2026-11-09): look up the base register, not the EA.** In an
out-of-context Vivado experiment on xc7a100t at 10 ns (32 entries; VPN,
ASID and G compare; PFN and permission mux into flops):
- **EA-based:** EA adder → CAM → mux → flops: 8.9 ns data path, 13 levels,
  slack +1.06 ns. That is on an empty chip. In context the operand fanout
  and congestion would eat that, because the core already runs at about 0
  margin.
- **Base-register:** base register → CAM → mux → flops: 7.6 ns, 7 levels,
  slack +2.37 ns.

So the data-side lookup uses the **base**: rs1 (modes 00/01/11), the
immediate (mode 10), SP for stack ops, and R15 for LEAVE/LEAVERET. In MEM
cycle 1, a registered compare of EA[31:16] against the base's VPN detects a
page crossing. A crossing access (a large offset, `rs1 + rs2` spanning
pages, a PUSH at exactly a page boundary) is replayed: one more lookup on
the registered EA while it holds MEM, before any access. That costs one or
two cycles, only on crossings. The compiler should keep the pointer in rs1
for reg+reg addressing (ABI note for klausscpu-llvm). The paragraphs below
are the original sketch; where they say "on the EA in EX", read "on the
base in EX, with the MEM replay above".

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

### 11.5 Pipeline (as built, stage 3: `7670218` onwards)

- **Data lookup: the MPU's extra EX cycle, not the base-register replay.**
  §11.4 chose a base-register lookup with a MEM replay on a page crossing.
  Stage 2 then built the extra EX cycle (`ex_mpu_hold`): the EA is
  registered (`ex_ea_q`) in the first cycle and the checks run from that
  register in the second, so the EA adder and the CAM are never in series
  — exactly the path the replay was meant to avoid, closed by a pattern
  that already met timing. With TE = 1 the TLB's data CAM runs in that same
  cycle, off `ex_ea_q`. That was the cheaper choice: no replay state in MEM
  (the port engine, hit pipe, posted stores and STLF all live there and
  are the tightest logic in the core), the access-fault bit of §7.1 is
  reused as is, and there is no compiler ABI rule (pointer in rs1). Cost:
  one cycle per memory instruction while TE = 1, as for the MPU. A
  two-access op whose 2nd doubleword is in the next page (`ex_xpg`:
  LEAVERET, or a spanning MEMGET32, with `ex_a[15:3]` all ones) takes a
  third EX cycle with `ex_ea_q` stepped to the next page, translating both
  pages before either access; the 2nd read uses the registered `mem_pa2hi`.
- **Data side, VIPT.** MEM gets the PA (`mem_iaddr` = {PFN, VA[15:0]}); the
  D-cache indexes with VA bits as before and the hit pipe compares
  `h_addr` (the PA). Stores and MMIO decisions use VA[31:28]. Self-modifying
  -code squashes compare VAs (`mem_eaddr`); the I-cache store snoop compares
  PA tags. The LR/SC reservation holds the VA (`wb_va_dw`); every TLB / ASID
  / TE change clears it.
- **STLF with TE = 1.** The in-flight-store *block* compares the page offset
  only (conservative for aliases). A *forward* needs the youngest
  offset-matching store's VA page to equal the load's (`mem_eaddr`, or
  `m_vahi` / `sq_vahi` for the store on the port / queued, valid only for
  stores issued from MEM and dropped on a TLB / ASID / TE change). Without
  it, STLF was off with TE and every POP / LEAVE behind a PUSH / ENTER on a
  kseg stack (always VA ≠ PA) waited for the store: +5.1 % instead of
  +1.5 %.
- **Fetch: a one-entry micro-ITLB.** {VPN, fetch mode, PA[31:16], verdict,
  miss}. It is read alongside the I-cache BRAM into `r_look_*`, so the
  I-cache tag compare ({`r_look_pa`, VA[15:13]} against the stored PA tag)
  reads registers only; the prefetch's compare too. A lookup whose page it
  does not cover (in the current fetch mode) stalls while `it_ref` refills
  it from the TLB's second (fetch) port in one cycle — a TLBP committing
  that cycle owns the port. The verdict applies §11.2 for the fetch mode:
  user at `0x8000_0000+` and the supervisor hole are segment faults; useg /
  kmap need a matching entry with X; kseg is direct; MMIO is identity. A
  refused page is a **fake fill** (zeros, nothing on the port) with a
  per-slot fault tag (`ifb_ft` / `ifb_fm`). So fetch never puts an
  untranslated or faulting useg / kmap address on the port, speculatively
  or not. The IF→ID latch decides the fetch fault from the pc segment rule
  (first) and the tags: TLB_MISS_FETCH or ACCESS_FETCH, `EADDR` = pc, or the
  next page's start for an instruction straddling into a refused page; a
  TLB-related one loads `TLB_HI`. The NLB is only filled from translated
  hits and the BTIC never captures a tagged window. The micro-ITLB is
  dropped by a TLB / ASID / TE change, program load and debugger resume.
  With TE = 1, as with MPU_EN = 1, a fetch fill to `0xF…` is always a fake
  fill: supervisor code at MMIO runs as zeros (ILLEGAL). Fetching from a
  device is meaningless, and a stray or wrong-path fetch must not pop a
  device FIFO (review 2 #1).
- **Discard.** TLBWI, TLBINV, MTSR ASID and MTSR STATUS clear the window,
  NLB, prefetch, BTIC, micro-ITLB and the reservation, and drop a fill in
  flight from the window (`if_drop`; it still installs, physically
  tagged). They serialize, so the next instruction is fetched under the
  new state.
- **Frame push** (§11.3): PA = SP − 8 − `0x8000_0000`; SP − 8 not in kseg
  parks with kind 6 (HCF `ERR_DFAULT` 0x0B; a debug halt, cause TRAP, when a
  debugger is attached). Neither writes ECAUSE / EADDR / TLB_HI.
- **Watchpoints** compare PAs (`mem_iaddr`, `h_addr`), like debugger memory
  accesses.
- **Entry count: 16 on the xc7a100t** (decision 2026-11-09, with the
  runtime). With 32 entries the core missed timing by about 0.3 ns after
  every design-level fix: the stage 3 area (+2.7k LUTs over stage 2, the
  TLB 1.4k of it) tipped long-standing stage 2 paths, and the data CAM
  path itself, over the edge (§11.6). 16 entries was chosen over sharing
  one CAM between the data and fetch ports, because:
  - the kernel runs in untranslated kseg, so the TLB maps only user pages
    and kmap, and 16 × 64 KiB = 1 MiB of reach is ample (four times an
    R3000's);
  - it saves about twice the area of sharing, and congestion is what
    failed;
  - it is a parameter change, with no new EX hold or stall logic.
  The count is the parameter `TLB_N` (`CORE1_TLB_N` in KlaussCPU); 32 or 64
  fit a bigger part. CPU_FEATURES[15:8] reports it, and TLB_INDEX uses
  log2 N bits (§11.3).
- **Build switches**: the module parameter `TLB_EN` (on; off on core 2) and
  `EN_TLB = EXC_EN && TLB_EN`. KlaussCPU's `CORE1_TLB_EN` drives
  CPU_FEATURES[2] and [15:8] = the entry count; `TLB_N` / `CORE1_TLB_N`
  = 16 here. With the MPU off on the A7 (§10) and lean crypto, CPU_FEATURES =
  0x0010_100D.
- **MPU off on the Nexys A7** (Graham, 2026-11-09): core 1 builds the TLB
  and not the MPU, for timing margin; Zephyr runs in supervisor mode. The
  MPU is a build option (`CORE1_MPU_EN`), simulated in an `MPU_EN = 1`
  build (`VSIM_MPU_EN=1`: its tests, `mpudiff`, and a diff against
  `7893230`).
- **Compatibility**: TE = 0 is cycle-identical to `7893230` for code that
  does not write STATUS / ASID or execute TLB ops (diff, --irq, +HIT,
  EXC_EN = 0, socdiff). MTSR STATUS / ASID and the TLB ops discard the
  fetch side and clear the reservation even with TE = 0 (about 4 cycles
  each), as §11.3 specifies; MTSR is new in stage 1, so nothing older is
  affected.
- **Cost** (60 randprog seeds, every page mapped; STATUS the only
  difference): TE = 1 +1.52 % supervisor, +1.87 % user; +2.17 % / +2.42 %
  with the hit-pipe model. MPU_EN = 1 on the same programs: +1.36 %.
- **Tests**: `tests.py` `tlb_*` (SRs and ops, privilege, each segment,
  misses / permissions / ASID / G, TLBVEC, TLB_HI auto-load, two-access
  ops across pages, DOUBLE_FAULT, fetch through kseg / kmap / useg and its
  faults, a straddle into an unmapped page, user fetch rules, code written
  through kseg and run through a useg alias after ICACHE_INV, TE turned on
  with no entry and refilled by a handler, STLF across aliases);
  `run.py tlbdiff` runs random user programs translated (code at a
  non-identity VA, an empty TLB, a kseg refill handler over K entries,
  injected SYSCALLs bumping ASID / TLBINV, optional IRQs) and the final
  state must equal the physical run's. K = 1 would livelock by design (a
  load needs its code and data pages at once); with IRQs K ≥ 4, since K = 2
  starves at the TB's IRQ rate (it completes at a 1500-cycle period).

### 11.6 Timing closure (stage 3, 2026-11-09)

Goal: WNS ≥ 0 with the standard `build_fast` flow by design changes,
CPI-neutral, with TE = 0 cycle-identical to the stage 2 head `7893230`.
Every change below passed the full sim set. "Median" is over three
placements of the same netlist: the standard flow plus `build_alt_place`
with ExtraPostPlacementOpt and ExtraTimingOpt. It is used only to judge a
change (the spread between placements is about ±0.2 ns), never to pick
the shipped bitstream.

| Build | Commit | Change | Tier 1 WNS | Final WNS / TNS | Median | LUTs | FFs |
|---|---|---|---|---|---|---|---|
| stage 2 (`mmu-m2`) | `da8c085` | — | −0.128 | **+0.016** / 0 | | 42,449 | 29,486 |
| `mmu3` | `16bfb10` | stage 3, 32 entries | −0.646 | −0.621 / −76.2 | | 45,302 | 31,115 |
| `mmu3b` | `534794f` | registered exception vector; one fetch-port CAM on registers; `xlt_q`; I-cache reference-tag mux; `keep_hierarchy` on `mmu_tlb` | −0.316 | −0.305 / −8.1 | | 45,116 | 31,146 |
| `mmu3c` | `48e9b47` | TLB writes a cycle after commit; `ex_xpg_q` | −0.427 | −0.347 / −16.9 | | 44,638 | 31,068 |
| `mmu3d` | `daee899` | + `wb_rfch`; **16 entries** | −0.173 | −0.161 / −2.3 | | 43,743 | 30,475 |
| `mmu3e` | `1b87316` | STLF page compare from `ex_ea_q`; `ex_mok` | −0.392 | −0.262 / −5.9 | −0.262 | 43,603 | 30,549 |
| `mmu3f` | `8f4ad33` | SMC hit-latch test without the pc adder | −0.048 | −0.013 / −0.01 | −0.184 | 44,619 | 30,657 |
| `mmu3g` | `f9d4aac` | RAS miss as compare-then-select | −0.417 | −0.398 / −34.6 | −0.165 | 44,743 | 30,710 |
| `mmu3i` | `875296e` | + `keep_hierarchy` on `mmu_mpu`; registered `ex_b_zero` | −0.200 | −0.106 / −1.4 | −0.021 | 43,828 | 30,302 |
| `mmu3j` | `5a615e8` | STLF VA check folded into each candidate's coverage bit | −0.744 | −0.467 / — | | 43,626 | 30,497 |
| `mmu3k` | `e82a4a1` | **MPU off** on the A7 | −0.204 | −0.115 / — | −0.050 | 42,442 | 29,498 |
| `mmu3l` | `52bece8` | **lean crypto** (TRNG only); review 3 fixes | −0.127 | −0.016 / — | −0.014 | 34,516 | 22,562 |
| `mmu3m` | `10413af` | `ex_b_zero`'s WB leg from byte-zero flags | −0.422 | −0.108 / −1.3 | −0.122 | 34,221 | 22,715 |
| `mmu3n` | `98fe470` | base+K candidates as separate kept adders | **+0.011** | **+0.011** / 0 (tier 1) | | 34,445 | 22,757 |

- **What failed and why.** The first build's worst paths ran through new
  stage 3 legs on old near-critical cones: TLBVEC in the `pc` mux behind
  the RAS-miss compare, the TLBP probe select from the WB commit (which
  waits on the bus) in front of the fetch-port CAM, the TLBWI enable fanned
  out to every entry, `MPU_EN ∨ TE` and the page-crossing term in
  `ex_busy` / the early-issue enable, and the exact-VA STLF check on the
  EA adder. Each was taken back to the inputs it had at stage 2. What was
  left were stage 2 paths (the `pc` next-value mux, the SMC hit latch, the
  RAS-miss compare, DIV's zero test → the fill-issue enable, the `pos`
  tracker, the hit pipe), tipped over by stage 3's area. That is why the
  entry count went to 16 (§11.5) and why the later steps are stage 2
  walls. They are fixed exactly and CPI-neutrally, and also buy margin for
  master.
- **Release bitstream: `d9cf6c2`** (`~/.cache/kbuild/mmu_stage3_release_d9cf6c2.bit`,
  md5 `bdaed149…`), 2026-11-10. It is `98fe470` plus **precise EI** (§3.3:
  an EI that enables while an interrupt is pending takes it before the next
  instruction). It closed with the **standard flow** (tier 1 −0.064, tier 2
  AggressiveExplore WNS +0.041 / TNS 0, WHS +0.013 / THS 0, 12 min 54 s).
  **LEAN**: MPU off, TRNG-only crypto, CPU_FEATURES `0x0010_100D`. Board
  validation: klausscc all_tests 31/32 with 0 failed (hw_tlb.kla 19/19),
  test_tlb 26/26, test_exc 53/53, test_atomic 139/139, test_mpu SKIP,
  board_suite -O2 with only the expected crypto_selftest SKIPs,
  perf_baseline within ±0.03 % of master, ssh_shell 5/5, xv6 with its
  **original upstream idle** (`intr_on(); intr_off(); … wait` under DI,
  which hung on `98fe470`) echoing shell input, `tools/validate/xv6.sh full
  board` (usertests -q ALL TESTS PASSED), and Linux 7.3.3 booting to a
  busybox shell (`tools/validate/linux.sh … busybox board`). The first
  attempt to load it failed silently (the JTAG step printed nothing; `/tmp`
  was 75 % full) and its first "validation" ran on `98fe470`; it was
  reloaded with "DONE" checked and re-validated.
- **Previous release, superseded: `98fe470`** (`~/.cache/kbuild/mmu_stage3_release_98fe470.bit`,
  md5 `9a6ee8bc…`), chosen by Graham on 2026-11-09. It closed with the
  **standard flow in tier 1** (WNS +0.011 / TNS 0, WHS +0.026 / THS 0,
  12 min 40 s), so a normal `build_fast` of the commit reproduces it.
  **LEAN**: MPU off, TRNG-only crypto, CPU_FEATURES `0x0010_100D`. It is
  `52bece8` plus two exact timing fixes (`10413af`, `98fe470`). Board
  validation (2026-11-09, runtime mmu with the mnemonic switch): klausscc
  all_tests 30/31 with 0 failed, hw_tlb.kla 19/19, test_tlb 26/26 (T11a/b,
  the review 3 CRITICAL check), test_exc 53/53, test_atomic 139/139,
  test_mpu SKIP, board_suite -O2 with only the expected crypto_selftest
  SKIPs, perf_baseline within ±0.03 % of master; ssh_shell 5/5 (SSH
  session, HTTPS, MicroPython) on software AES.
- **First release, superseded: `52bece8`** (`mmu_stage3_release_52bece8.bit`,
  md5 `c75aa83d…`), an alt-place ExtraPostPlacementOpt closure (WNS +0.008,
  WHS +0.015) of the same design without the last two fixes. Its board
  validation also covered ssh_shell / HTTPS on software AES. Graham's aim
  for the "standard placement" rule was fast builds, not a ban on alt-place
  (master's own bitstream is an alt-place closure too). (`875296e`'s
  alt-place bitstream was an earlier SRAM-only interim, without the review 3
  CRITICAL fix.)
- **Synthesis folding.** Without `keep_hierarchy`, synthesis placed core
  logic inside `mmu_tlb` (`pc`-mux LUTs) and `mmu_mpu` (`id_ser`, the
  multiplier). Both are kept now, like `mmu_sr`.

---

## 12. Review outcome (runtime session, 2026-11-09)

1. SYSCALL frame PC = the next instruction: agreed.
2. DI on exception entry: agreed. Zephyr keeps `INT_MASK = 0xF`
   permanently, uses `INT_VECn` for per-source enable and DI/EI for
   irq_lock. So IRET restoring `INT_MASK` is harmless. xv6 and Linux will
   use the same constant-mask rule. Handlers EI after reading
   ECAUSE/EADDR.
3. The debugger halt stays the default; `DMCONTROL[5]` EXC_PASS was added
   (§4.4), in step (d).
4. A user-readable clock: SR `CYCLE` (§6.1).
5. All MMIO privileged in user mode: agreed for Zephyr, NuttX, xv6 and
   Linux. Zephyr cannot grant a user thread a device-register partition,
   and that is accepted.
6. 64 KiB pages: agreed. User programs link with `-z
   max-page-size=0x10000`. The page-table format is software's.
7. klausscc (assembler, emulator, GDB server, regs 20/21) and
   `debug/klausscpu_rtos.py` (frame bit 44): runtime session, on klausscc
   `mmu`.
8. Added: SR `TP` (user RW) and SR `CYCLE` (user R); LR/SC moved into
   stage 1, because `src/atomic.c`'s DI/EI is privileged in user mode. The
   runtime will reimplement it with LR/SC loops, probing `CPU_FEATURES[3]`
   with a DI/EI fallback.
9. MPU: 16 regions preferred (§10).
10. Spec clarifications from the klausscc emulator (2026-11-09), all
    matching the RTL as built: reg 21 bit order (§6.2); MISALIGNED_FETCH
    does not debug-halt (§4.4); legality before privilege (§3.5); cause
    priority and EADDR (§4.1); parking does not write ECAUSE/EADDR (§4.4).
    The emulator first proposed the aligned-down EADDR for A=1 accesses;
    the raw EA was kept.

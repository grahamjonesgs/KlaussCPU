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
| 2 | MPU: 16 base/limit/permission regions, if the LUTs allow (8 at minimum), for Zephyr USERSPACE and NuttX protected | outline (§10) |
| 3 | Software-refilled TLB with ASIDs, 64 KiB pages | outline (§11) |

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
| 11–13 | TLB_MISS_LOAD / _STORE / _FETCH | stage 3 | the instruction | faulting VA |
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
| 0x00 | `STATUS` | R (**user mode too**), W (supervisor) | `[0]` UM (read-only), `[1]` DI (read-only, the committed bit), `[8]` MPU_EN (stage 2), `[9]` TE (stage 3). Other bits read 0. Writes to UM and DI are ignored; use IRET / DI / EI. |
| 0x01 | `EVEC` | RW | exception vector, `[1:0]` read 0 |
| 0x02 | `ECAUSE` | RW | §4.1 code in `[7:0]` |
| 0x03 | `EADDR` | RW | `[31:0]` |
| 0x04 | `USP` | RW | `SP_ALT`, which is the user SP while in supervisor mode |
| 0x05 | `SCRATCH0` | RW | 64-bit |
| 0x06 | `SCRATCH1` | RW | 64-bit |
| 0x07 | `TP` | RW **in user mode too** | 64-bit thread pointer, no hardware meaning |
| 0x08 | `CYCLE` | R **in user mode too** | 64-bit core-cycle counter (§3.1, §5); MTSR to it is ILLEGAL |
| 0x10–0x2F | MPU | | stage 2 (§10); room for 16 regions × 2 |
| 0x30–0x3F | TLB / ASID | | stage 3 (§11) |

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
| `[1]` | stage 2 MPU present (`[7:4]` = region count / 2 or similar, defined then) |
| `[2]` | stage 3 TLB present (`[15:8]` = entry count) |
| `[3]` | LR/SC (§5.1). Set together with `[0]` in stage 1, but kept separate so software tests for atomics by name. |

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

## 10. Stage 2 outline: MPU

For Zephyr USERSPACE and NuttX protected builds, with translation off.

- **16 regions if the LUTs allow, 8 at minimum.** The runtime review says 8
  is tight for Zephyr memory domains (text, rodata, stack and partitions
  per thread). Each region is a base/limit pair at 32-byte granularity
  (cache-line sized, the same as ARMv8-M, so Zephyr's base/limit MPU code
  maps over), with attributes R, W, X (for user mode) and GUARD. SRs
  `0x10–0x2F`: `MPU_BASEn` = `{base[31:5], attrs}`, `MPU_LIMITn` = the last
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
  §7.5 override). With 8 regions that is 16 data and 16 fetch
  27-bit comparators, about 1k LUTs; with 16 regions, about 2k. Faults are ACCESS_LOAD / ACCESS_STORE / ACCESS_FETCH with
  `EADDR` = the address.
- The user-mode MMIO rule (§3.6) still applies on top.
- When `TE = 1` (stage 3) the MPU is ignored.

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
- SRs `0x30+`: `ASID`, `TLB_INDEX`, `TLB_HI` {VPN, ASID, G}, `TLB_LO` {PFN,
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

Moved into stage 1 (§5.1). With translation on, the reservation will hold the
physical doubleword address.

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

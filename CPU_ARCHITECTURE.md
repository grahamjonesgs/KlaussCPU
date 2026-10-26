# CPU Architecture Reference — for C Compiler / Assembly Writers

> **Currency (2026-10).** This document describes the CPU as executed by
> `pipeline_core.sv` (5-stage in-order pipeline) with ISA encoding v2 + the
> v3 additions. It is the *programmer's model*: semantics, flags, alignment,
> calling convention, interrupts, and the pipeline costs that matter for code
> generation. It deliberately does **not** repeat the opcode numbers:
> - [ISA_ENCODING_V2_MAP.md](ISA_ENCODING_V2_MAP.md) — authoritative
>   symbol → encoding map (§7 there = v3 additions).
> - [ISA_V3_IMPL.md](ISA_V3_IMPL.md) — v3 design notes and measurements.
> - [PIPELINE_IMPL.md](PIPELINE_IMPL.md) — pipeline implementation.
> - [MMIO_MAP.md](MMIO_MAP.md) — all memory-mapped devices.
>
> Where this document and the RTL disagree, the RTL wins — please fix the doc.

## 1. Overview

Custom 64-bit RISC-style CPU on a Xilinx 7-series FPGA (Nexys A7), with
128 MiB of DDR2 SDRAM behind a 2-way set-associative write-back data cache
(64 KB, 32-byte lines). Memory is **little-endian**. Addresses are 32 bits
(byte-addressed); registers are 64 bits. Execution is a 5-stage in-order
pipeline (IF, ID, EX, MEM, WB) at 100 MHz, synchronous with the DDR2
controller (2:1 MIG ui_clk). Instruction fetch has its own I-cache in front
of the shared memory port (§3.2).

The multicycle FSM in `KlaussCPU.sv` survives only for boot, program load
and the crash dump; it does not execute programs.

---

## 2. Register File

| Register | Name | Role |
|----------|------|------|
| R0–R14 | General purpose | 64-bit each |
| R15 | Frame pointer | FP; used implicitly by `ENTER` / `LEAVE` / `LEAVERET` |
| SP | Stack pointer | 32-bit dedicated register (not one of R0–R15) |

- All general registers are **64-bit**. No register is hardwired to zero.
- SP is a separate 32-bit register, accessed with `GETSP` / `SETSP` /
  `ADDSP` and implicitly by push/pop/call/return. Reset value
  `0x0800_0000` (top of DDR; the `SP_RESET` parameter — core 2 of the AMP
  build uses the top of its local BRAM).
- **ABI (software convention, defined by the LLVM backend):** R4–R7 and
  R15 are callee-saved; the rest are caller-saved. Hardware enforces none of
  this, except that the frame instructions name R15.

---

## 3. Memory Model

### 3.1 Data

- **Address space**: 32-bit byte addresses; DDR2 at `0x0000_0000`–`0x07FF_FFFF`,
  MMIO at `0xF000_0000` and up (see MMIO_MAP.md).
- **Endianness**: little-endian throughout. Byte address `N` occupies bits
  `[8*(N mod 8) + 7 : 8*(N mod 8)]` of the 64-bit doubleword at `N & ~7`.
- **Code and data share one flat address space.** The first instruction is
  at `0x0020`; `0x0000–0x001F` (four doublewords) is the heap header used by
  the runtime — do not place code there.
- **Stack** lives at the top of DDR2 and grows downward.
- All data accesses go through the data cache (32-byte lines; fills are two
  pipelined BL8 bursts with critical-word-first early restart). The core has
  **one** memory port, shared by fetch and the MEM stage; MEM has priority.

### 3.2 Instruction fetch and code coherence

The fetch path has three layers, all transparent to software:

1. **IFB window** — two consecutive doublewords. Any 1/2/3-word instruction
   spans at most two, so an instruction issues from the window in one cycle.
2. **I-cache** — direct-mapped, 512 lines × 16 B (8 KiB), BRAM, with a
   valid bit per doubleword. A hit refills the window without using the
   shared port.
3. **Next-line buffer (NLB)** — a background I-cache lookup of the next
   16 B line, so straight-line code crosses line boundaries without stalling.
   It only prefetches lines that hit in the I-cache, so it never competes
   for the memory port.

**Coherence rules:**
- **Stores by this core are handled automatically.** Each DRAM store is
  tag-checked against the I-cache and invalidates the matching line, and also
  clears matching window and NLB copies. A store that overwrites an
  instruction already fetched into ID or EX squashes that instruction and
  everything after it, then refetches. Self-modifying code and loaders work
  with no fence.
- **Code written by anything else (DMA, the blitter, core 2) is not
  snooped.** After such a write, store a 1 to bit 3 (`ICACHE_INV`) of
  `CACHE_CTRL` (`0xF005_0000`). This acts as a fence.i: the core drops the
  whole I-cache, NLB and window and refetches everything after the store. A
  data-cache flush (bits 1/2) may also be needed, depending on the writer.
- `start` (program load) and debugger resume also invalidate the whole
  I-cache.

---

## 4. Instruction Encoding (summary)

All instruction words are 32 bits. Word 0 is laid out as follows:

```
 31 30 29    26 25                 12 11    8 7     4 3     0
┌─────┬────────┬─────────────────────┬───────┬───────┬───────┐
│ LEN │ CLASS  │ attributes / OP     │  rd   │  rs1  │  rs2  │
└─────┴────────┴─────────────────────┴───────┴───────┴───────┘
```

- **LEN [31:30]** = instruction length: `01` = 1 word, `10` = 2 words
  (imm32 at PC+4), `11` = 3 words (imm64: lo32 at PC+4, hi32 at PC+8). `00`
  is illegal, so any v1 binary traps on its first instruction.
- **CLASS [29:26]**: 1 ALU reg-reg · 2 ALU imm (incl. MOV/LEAPC) · 3 compare ·
  4 shift/rotate/bit · 5 unary · 6 load · 7 store · 8 branch/call ·
  9 stack/SP/frame · A mul/div · B system · C LCD · D fused compare-and-branch.
  Classes 0, E and F trap.
- **rd [11:8]** = destination (**for stores, the data source**); **rs1 [7:4]**
  = first source / base address; **rs2 [3:0]** = second source / count /
  indirect target.
- Unused register fields and reserved bits **must be 0**. The decoder
  checks them strictly, and any unlisted combination is illegal (§12).
- **v3 short forms** (1 word, LEN=01) put a small immediate in word 0:
  imm8 for ALU-imm, SETR and LEAPC; simm8 for CMPRV; simm8 << size for
  `LDIDX`/`STIDX` offsets; simm18 word displacement for PC-relative
  branches and calls; and the class D fused branch. The toolchain picks
  these automatically, and each has a 2-word equivalent.

Code is stored little-endian: an instruction word at address A with `A[2]=0`
is in bits [31:0] of its doubleword, and with `A[2]=1` in bits [63:32].

Full field definitions and every template are in ISA_ENCODING_V2_MAP.md
§1–§3 and §7.

---

## 5. Immediate Extension Rules

The encoding chooses the extension explicitly; nothing is implied by the
mnemonic.

| Where | Extension |
|-------|-----------|
| Class 2 ALU-imm / MOV (SETR), class A MUL/DIV/MOD imm | SGN bit `[20]`: 1 = sign-extend imm32, 0 = zero-extend. Toolchain convention: ADDI/SETR/MULV/DIVV/MODV sign-extend; ADDV/MINUSV/ANDV/ORV/XORV zero-extend. |
| CMPRV (class 3 imm) | SGN bit; the short form is always sign-extended |
| Load/store offset (`rs1 + imm32`) | **Effectively signed.** The effective address is a 32-bit modular add, so `imm32 = 0xFFFF_FFF8` gives `base − 8`. Short form: simm8 × access size. |
| ADDSP | 32-bit modular add to SP (negative allocates) |
| PUSHV | zero-extend imm32 → 64 |
| SETR64, PUSHV64 | full 64-bit immediate (3 words) |
| PC-relative branch/call, LEAPC | 32-bit add to **this instruction's own PC** |

---

## 6. Condition Flags

Four flags are **stored**: Z (zero), S (sign), C (carry/borrow), V (signed
overflow). All other conditions are derived from them whenever they're
read: equal = Z, signed less = S^V, unsigned less = C. There is no separate
E/L/U state.

### 6.1 Who writes which flags

Every flag writer updates only its own subset; the other flags keep their
values.

| Instructions | Flags written |
|--------------|---------------|
| ADD/SUB/ADC/SBC (reg and imm), INC/DEC, ADDW/SUBW/ADDIW | Z S C V (Z follows the 32-bit result for W ops; S/C/V come from the 64-bit operation) |
| CMP (flag form, class 3 B=0, incl. W) | Z S C V of `rs1 − rhs`; no register write |
| AND/OR/XOR, MIN/MAX(U), boolean compares, MOV/SETR/LEAPC | **none** |
| Shifts, rotates, BSET/BCLR/BTGL/BTSTRR | Z, **only if the F bit is set** |
| ROL/ROR by 1 with F (ROLR1/RORR1), RCL/RCR | Z C |
| BTST #N (immediate form) | Z = ~bit (no register write) |
| BEXTR | Z |
| Unary NEG/NOT/COPY/ZEXT/BSWAP/BITREV/POPCNT/CLZ/CTZ/GETF | Z, only if F |
| ABS (F required) | Z V |
| SEXT | Z S, only if F |
| MUL (all forms, MULW) | Z S V |
| DIV/MOD | Z V (divide by zero: V only, see §7) |
| IRET | all four, restored from the frame (§13.1) |

So `JMPZ`/`JMPNZ` after a logic op does **not** test that op's result.
Compare it explicitly, or use a fused branch.

### 6.2 Reading flags

- Conditional branches and calls take a 4-bit COND and an INV bit:
  0 always · 1 Z · 2 C · 3 V · 4 S · 5 LT (S^V) · 6 LE (Z|(S^V)) ·
  7 ULT (C) · 8 ULE (C|Z) · 9 E (= Z). With INV set, the condition is
  negated (NZ, GE, GT, UGE, UGT, NE, …). COND 10–15, and INV with
  COND 0, are illegal.
- ADC/SBC and RCL/RCR read C.
- `GETF rd` (assembler name `SETFR`) gives `rd = {Z, Z, C, V, 60'b0}`:
  Z, equal (= Z), carry and overflow in bits 63..60. **S is not readable
  this way.** The full 7-bit flag word only appears in the interrupt frame.

---

## 7. ALU Instructions

All are 3-operand (`rd`, `rs1`, `rs2`/imm). An old in-place form is
written with `rd = rs1`.

| Group | Operations | Notes |
|-------|------------|-------|
| Add/sub (class 1/2) | ADD, SUB, ADC, SBC | reg or imm (incl. ADC/SBC imm) |
| Logic (class 1/2) | AND, OR, XOR | no flags |
| Min/max (class 1) | MIN, MAX, MINU, MAXU | reg-reg only |
| Move (class 2) | SETR (MOV imm, SGN picks sext/zext), SETR64 (3-word), LEAPC `rd = PC + imm` | |
| 32-bit (v3) | ADDW, SUBW, ADDIW (2-word only), MULW | result = low 32 bits of the 64-bit op, **sign-extended** |
| Boolean compare (class 3, B=1) | `rd = (rs1 PRED rhs) ? 1 : 0`; PRED EQ/LT/LE/ULT/ULE, INV gives NE/GE/GT/UGE/UGT | reg or imm rhs; W = compare low 32 bits; no flags |
| Shift/rotate (class 4) | SHL, SHR, SAR, ROL, ROR | count = `rs2[5:0]`, or embedded N `[20:15]` (1 word) |
| Rotate through carry | RCL, RCR | by 1 only |
| Bit ops (class 4) | BSET, BCLR, BTGL (`rd = rs1` with bit changed), BTSTRR (`rd = bit`), BTST #N (flag only) | position from rs2 or N |
| Bit field (class 4, 2-word) | BEXTR: `rd = (rs1 >> start) & mask(len)`; BDEP: deposit `rs2` into `rs1` | imm32: start `[4:0]`, len `[12:8]`. **32-bit fields only; the result's upper 32 bits are zero.** |
| Unary (class 5) | COPY, NEG, NOT, ABS, SEXT/ZEXT (8/16/32), BSWAP, BITREV, POPCNT, CLZ, CTZ, INC, DEC, GETF | CLZ/CTZ of 0 = 64 |
| Mul (class A) | MUL low (signed/unsigned), MULH high (signed/unsigned) | reg or imm; V = high half not just the sign extension of the low half (MULH: V = 0) |
| Div/mod (class A) | DIV, DIVU, MOD, MODU | truncate toward zero; MOD takes the dividend's sign |

**Divide by zero** does not trap: DIV gives `0xFFFF_FFFF_FFFF_FFFF`, MOD
gives `rs1`, V is set and the other flags are unchanged.

---

## 8. Memory Access Instructions

Loads are class 6 (`rd = ext(mem[EA])`). Stores are class 7
(`mem[EA] = rd`, so **the data comes from the rd field**). Attributes: SIZE
`[25:24]` (8/16/32/64), SGN `[23]` (loads only), MODE `[22:21]`, A `[20]`
(64-bit only).

### 8.1 Addressing modes

| MODE | EA | Length |
|------|----|--------|
| 00 | `rs1` | 1 word |
| 01 | `rs1 + imm32` (`LDIDX*` / `STIDX*`) | 2 words, or 1-word short form with simm8 × size |
| 10 | `imm32` absolute (`MEMREADR` / `MEMSETR`) | 2 words |
| 11 | `rs1 + rs2` (`LDIDX64R` / `STIDX64R`) | 1 word |

Every combination of SIZE, SGN and MODE is legal, so there are
sign-extending, register-indexed and absolute forms at every width.

### 8.2 Sign extension

Sub-64-bit loads **zero-extend when SGN=0 and sign-extend when SGN=1**
(`MEMGET8_S`, `LDIDX16_S`, `LDIDX32_S`, …). No separate SEXT is needed.

### 8.3 Alignment

| Access | Behaviour |
|--------|-----------|
| 8-bit | any address |
| 16-bit load/store | address bit 0 ignored (forced 2-byte aligned) |
| 32-bit store; 32-bit load in modes 01/10/11; any signed 32-bit load | address bits [1:0] ignored (forced 4-byte aligned) |
| **32-bit unsigned load, mode 00 (`MEMGET32`)** | **unaligned-tolerant**, including across a doubleword boundary (a second read if the next doubleword is in another line) |
| 64-bit with A=1 (`MEMGET64`, `LDIDX64A`, …) | address bits [2:0] ignored (forced 8-byte aligned) |
| 64-bit with A=0 ("raw": `MEMREADRR`, `LDIDX64`, …) | address passed to the memory system unmodified; keep it 8-byte aligned |

A misaligned access never faults; it silently uses the aligned slot.

Sub-word stores replicate the data across lanes and use byte enables, and
MMIO devices see the same `be` pattern.

---

## 9. Flow Control

Class 8 holds jumps and calls; class D holds the fused compare-and-branch.

| Form | Target | Length |
|------|--------|--------|
| Absolute (`JMP`, `JMPcc`, `CALL`, `CALLcc`) | `imm32` | 2 words |
| PC-relative (`JMPREL`, `JMPccREL`, `CALLREL`, …) | `PC + imm32` (PC of the branch itself) | 2 words |
| Short PC-relative (v3) | `PC + 4·simm18` (±512 KB) | 1 word |
| Register-indirect (`JMPR`, `CALLR`, conditional variants) | `rs2[31:0]` | 1 word |
| Fused compare-and-branch (class D, v3) | `PC + 4·simm13` (±16 KB) | 1 word |

- Any jump or call can be conditional (COND/INV, §6.2), including
  conditional PC-relative and indirect calls.
- **CALL** pushes the 64-bit return address `zero_ext(PC + length)`
  (SP −= 8). A conditional call that is **not taken** does not touch SP
  or memory.
- **RET** pops 8 bytes and jumps to bits [31:0].
- **Fused compare-and-branch**: `if ((rs1 PRED rhs) ^ INV) PC += 4·simm13`.
  PRED is EQ/LT/LE/ULT/ULE (64-bit compare), and rhs is `rs2` or a simm4
  in `[3:0]` (IMM bit `[21]`). **It writes no flags.** It replaces a
  CMP + Jcc pair and avoids the flag interlock (§14).

```
CMPRR  rs1, rs2        ; Z/S/C/V from rs1 - rs2 (1 word)
JMPLT  target          ; taken if S^V
```

The same test as one fused class-D word (PRED=LT, rhs=rs2) has no flag
dependency. The LLVM backend emits it automatically (disable with
`-mllvm -klausscpu-fuse-cmp-br=false`).

---

## 10. Stack and Frames

Full-descending stack: SP points at the **last pushed** item. Every slot is
8 bytes, and every push and pop is a real (cached) memory access.

```
PUSH rs1      ; SP -= 8; mem64[SP] = rs1                       (1 word)
PUSHV imm32   ; SP -= 8; mem64[SP] = zero_ext(imm32)           (2 words)
PUSHV64 imm64 ; SP -= 8; mem64[SP] = imm64                     (3 words)
POP  rd       ; rd = mem64[SP]; SP += 8                        (1 word)
GETSP rd      ; rd = zero_ext(SP)
SETSP rs1     ; SP = rs1[31:0]
ADDSP imm32   ; SP += imm32                                    (2 words)
ENTER N       ; push R15; R15 = SP; SP -= 8*N    (N = [21:0] dwords, 1 word)
LEAVE         ; SP = R15; pop R15                              (1 word)
LEAVERET      ; SP = R15; pop R15; pop PC  (= LEAVE; RET)      (1 word)
```

Typical frame:

```
CALL f            ; push return address
f:  ENTER n       ; save FP, FP = SP, allocate n locals
    ...           ; locals at [R15 - 8*k], args/RA above R15
    LEAVERET      ; restore SP and FP, return
```

`ENTER`/`LEAVE`/`LEAVERET` are equivalent to the older sequences
(`PUSH R15; GETSP R15; ADDSP -8N` …). The toolchain uses them by default.

---

## 11. C Type Mappings

| C type | Size | Align | Load (unsigned / signed) | Store |
|--------|------|-------|--------------------------|-------|
| `char` / `int8_t` | 1 | 1 | MEMGET8 / MEMGET8_S, LDIDX8 / LDIDX8_S | MEMSET8, STIDX8 |
| `short` / `int16_t` | 2 | 2 | MEMGET16 / _S, LDIDX16 / LDIDX16_S | MEMSET16, STIDX16 |
| `int` / `int32_t` | 4 | 4 | MEMGET32 / _S, LDIDX32 / LDIDX32_S | MEMSET32, STIDX32 |
| `long` / `int64_t` | 8 | 8 | MEMGET64, LDIDX64A | MEMSET64, STIDX64A |
| pointer | 4 | 4 | MEMGET32, LDIDX32 | MEMSET32, STIDX32 |

Pointers are 32-bit and zero-extended in registers. 32-bit `int`
arithmetic can use the W ops (ADDW/SUBW/ADDIW/MULW) and the 32-bit compares
(`CMPRRW`/`CMPRVW` and boolean W), which keep the value sign-extended
without a separate SEXTW.

---

## 12. System Instructions

| Mnemonic | Description |
|----------|-------------|
| NOP | no operation |
| HALT | stop: drains the pipe and parks the core (`park_kind` 0). Nothing after it is dispatched, so no padding is needed. |
| WAIT | sleep until an unmasked, vectored interrupt is pending, then take it through the normal dispatch. `IRET` resumes at WAIT+4. It never sleeps if an interrupt is already pending, and wake is level-sensitive, so "enable interrupts; WAIT" has no lost-wakeup race. Use it for idle loops (`arch_cpu_idle`). |
| RESET | jump to `0x0004`. A PC redirect only: registers, flags, SP and the mask are **not** reset. `0x0004` is inside the heap header (§3.1), so RESET is only useful if that word holds a valid instruction. |
| TRAP | software trap: parks the core (`park_kind` 1); the SoC produces the crash dump (CRASH_DUMP.md) |
| DELAYR rs1 / DELAYV imm32 | spin for **N × 8192 core cycles** (≈ 81.9 µs per unit at 100 MHz) |
| COPY, BSWAP, GETF | see §7 / §6.2 |

**Illegal instructions** (anything not in the encoding map, including
non-zero reserved fields) do not execute. The core parks with
`park_kind` 2 at that instruction, and no trace line is emitted for it.

With a debugger attached, HALT, TRAP and illegal instructions become
debug halts instead (§15).

---

## 13. I/O and Interrupts

The only I/O instructions left are the LCD ones (class C: LCDCMD/LCDDATA
send the low byte of a register or imm32; LCDRST drives the reset line
from imm32[0]). They hold EX until the LCD side port is
ready. **Everything else is memory-mapped**: UART (`0xF001_0000`), RGB
LEDs, 7-segment display, LEDs/switches, SD card, Ethernet, blitter, crypto,
timers and performance counters. See MMIO_MAP.md. The v1 UART/LED opcodes
(TXR, TXSTRMEM, …) no longer exist.

### 13.1 Interrupt model

Up to 4 sources: 0 = periodic timer, 1 = blitter DONE, 2 = Ethernet,
3 = reserved. When several are pending, the priority is
timer > blitter > Ethernet. Configuration is all MMIO at `0xF00F_0000`
(`INT_MASK`, `INT_PENDING`, `INT_VECn`, `TIMER_PERIOD`, … — see MMIO_MAP.md).
The only interrupt opcode is `IRET`.

A source is dispatched when `INT_MASK[n] & INT_PENDING[n] & (INT_VECn != 0)`.

**Entry sequence (in hardware):**
1. At an instruction boundary, the core takes the interrupt *instead of*
   fetching the next instruction. Instructions already dispatched complete.
2. Once the pipe has drained, the core checks the request again. If it has
   gone (for example, an older store just wrote `INT_MASK` or acknowledged a
   level IRQ), the interrupt is abandoned and execution continues normally.
   Source and vector are re-read at this point.
3. It pushes one 64-bit frame (SP −= 8), clears `INT_MASK[n]` for the
   dispatched source, and jumps to `INT_VECn`.

Single-stepping under the debugger never takes interrupts.

**Saved frame** (pushed on entry, popped by `IRET`):

```
[63:43] = 0            (reserved)
[42:39] = INT_MASK     (restored by IRET)
[38]    = Z            (restored)
[37]    = Z  (equal)   (informational)
[36]    = C            (restored)
[35]    = V            (restored)
[34]    = S            (restored)
[33]    = S^V (less)   (informational)
[32]    = C  (ult)     (informational)
[31:0]  = resume PC    (first instruction not yet executed)
```

`IRET` restores PC, Z/S/C/V (from bits 38/36/35/34) and the 4-bit mask.
A handler that edits the saved flags must edit those bits. If an MMIO write
to `INT_MASK` and an `IRET` restore land on the same cycle, the MMIO write
wins. A debugger unwinding through an interrupt must mask the frame to
32 bits to get the PC; it is not a normal call frame.

**Typical use:**
```asm
; Install timer handler → INT_VEC0 (0xF00F_0010)
SETR     R0, timer_handler
SETR     R1, 0xF00F0010
MEMSET32 R0, R1
; Timer period → 0xF00F_0030
SETR     R0, 0x000FFFFF           ; ≈10.5 ms @ 100 MHz
SETR     R1, 0xF00F0030
MEMSET32 R0, R1
; Unmask source 0 → INT_MASK (0xF00F_0000)
SETR     R0, 0x1
SETR     R1, 0xF00F0000
MEMSET32 R0, R1

timer_handler:
    ; ... PC, flags and mask are saved/restored by hardware;
    ; save any registers you use ...
    IRET
```

Level-triggered sources (blitter, Ethernet) must be acknowledged at the
device before `IRET`, or they are dispatched again immediately.

---

## 14. Pipeline Model (for scheduling)

Five stages: **IF** (window/I-cache/NLB) → **ID** (decode, hazard check,
operand read + forwarding) → **EX** (ALU, branch resolve, EA) → **MEM**
(memory port, shifter second stage, fused-branch resolve) → **WB** (the
single commit point: registers, flags, SP and mask commit here, in order).
Every instruction becomes visible in program order at WB, so interrupts and
debug halts are precise.

Costs relative to the ideal one instruction per cycle:

| Situation | Cost |
|-----------|------|
| ALU result used by the **next** instruction | 1 bubble (forwarded from MEM after that) |
| **Load** (or POP/LEAVE) result used by the next instruction | 2 bubbles (forwarded from WB) |
| Flag reader (Jcc, ADC/SBC, RCL/RCR, GETF) right after a flag writer | 1 bubble |
| SP user after an SP writer still in flight (push/pop/call/ADDSP/…) | waits until the writer commits |
| Taken branch/call (resolved in EX; predict not-taken) | younger fetches flushed |
| Taken fused branch (resolved in MEM) | the 2 younger instructions squashed |
| RET, IRET, LEAVERET | fetch stops until the return address is read in MEM |
| HALT, WAIT, RESET, TRAP, illegal | serialize (fetch stops behind them) |
| MUL | occupies EX ~4 cycles |
| DIV/MOD | occupies EX: setup + up to 64 steps (leading zeros of the dividend are skipped) |
| DELAY, LCD | occupy EX until done / port ready |
| Unaligned MEMGET32 or LEAVERET whose second dword is in another line | one extra memory read |
| Data access while fetch holds the port, or a cache miss | MEM waits on the port |

Practical rules: put an independent instruction between a load and its
use; prefer the fused branch to CMP + Jcc in tight loops; ENTER/LEAVERET
and the 1-word short forms reduce fetch traffic.

The core exports per-cycle hazard counters (RAW, load-use, flag, SP,
EX-busy, branch flush, fetch miss, port wait). They are readable through the
performance-counter block at `0xF00D_0000` (MMIO_MAP.md).

---

## 15. Debug Support

When built with `DEBUG_EN` (core 1), the JTAG debug module can halt, resume
and single-step the core, set 4 instruction breakpoints and 2 data
watchpoints (doubleword + byte mask, read/write), and read and write
registers (R0–R15, SP, resume PC, flags, INT_MASK) and memory while halted.

- A breakpoint stops *before* the matching instruction executes.
- A watchpoint stops at the next instruction boundary after the access (the
  access itself completes).
- HALT, TRAP and illegal instructions become debug halts while a debugger
  is attached.
- A halt request stops a WAIT in place. A single step completes the WAIT as
  if it had been woken.
- Resume refetches through the start path, so code patched while halted
  takes effect.

Details: [DEBUG_PLAN.md](DEBUG_PLAN.md).

---

## 16. History

- **v1 → v2 (flag-day re-encode):** all opcodes renumbered to the
  LEN/CLASS layout. Register fields unified to rd/rs1/rs2. UART/LED
  opcodes retired to MMIO. Flags unified from 7 stored to 4 stored + derived
  (FLAG_UNIFICATION_CHANGES.md).
- **v3 (2026-09):** short 1-word forms, fused compare-and-branch,
  ENTER/LEAVE/LEAVERET, LDIDX32_S, 32-bit compares and W ALU ops
  (ISA_V3_IMPL.md).
- **Multicycle FSM → 5-stage pipeline** (PIPELINE_IMPL.md).
- **Little-endian conversion fixes (April 2026, FSM era):** SETR64/PUSHV64
  high-word selection, LDIDX/STIDX 8/16/32 lane and byte-enable inversions,
  and null-terminated TXSTRMEM. The `.vh` task files they referred to are
  gone, and the behaviour is now in the encoding map and the pipeline RTL.

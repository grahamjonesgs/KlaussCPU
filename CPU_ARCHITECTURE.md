# CPU Architecture Reference — for C Compiler Implementation

## 1. Overview

Custom 64-bit RISC-style CPU implemented on a Xilinx 7-series FPGA, targeting 128 MiB of DDR2 SDRAM. The CPU has a 2-way set-associative write-back cache (64 KB, 128-bit lines). Memory is **little-endian**. The address space is 32 bits (byte-addressed); registers are 64 bits wide.

---

## 2. Register File

| Register | Name | Role |
|----------|------|------|
| R0–R14 | General purpose | 64-bit each, caller-saved by convention |
| R15 | Frame pointer | FP — callee must save/restore |
| SP | Stack pointer | 32-bit, hardware register (separate from R0–R15) |

- All registers are **64-bit**.
- SP is a dedicated 32-bit hardware register, not one of R0–R15. Instructions `GETSP`/`SETSP` read/write it.
- R15 is a software convention for the frame pointer; the hardware treats it as a general-purpose register.
- No register is hardwired to zero.

---

## 3. Memory Model

- **Address space**: 32-bit byte addresses, 0x00000000–0x07FFFFFF (128 MiB DDR2).
- **Endianness**: little-endian throughout. Byte address `N` occupies bits `[8*(N mod 8) + 7 : 8*(N mod 8)]` of the 64-bit DDR doubleword at aligned address `N & ~7`.
  - Address 0x0000 → bits [7:0] (LSByte of first doubleword)
  - Address 0x0007 → bits [63:56] (MSByte of first doubleword)
- **Code and data share the same flat address space.** The assembler places the first instruction at `0x0020` (`HEAP_HEADER_WORDS * 8 = 4 * 8 = 32`). Addresses `0x0000–0x001F` (four 64-bit doublewords) are the heap header used by the runtime; the compiler/assembler must not emit code into this region.
- **Stack** lives at the top of DDR2 and grows downward.
- All memory accesses go through the cache. The DDR2 bus is 64-bit wide; the cache line is 128-bit (two 64-bit doublewords).

---

## 4. Instruction Encoding

### 4.1 Instruction Sizes

All instruction words are 32 bits. Instructions are either 1, 2, or 3 consecutive words:

| Format | Words | Bytes | PC advance | Payload |
|--------|-------|-------|------------|---------|
| R | 1 | 4 | PC += 4 | 1 register |
| RR | 1 | 4 | PC += 4 | 2 registers |
| RRR | 1 | 4 | PC += 4 | 3 registers (ALU) |
| RV | 2 | 8 | PC += 8 | 1 register + 32-bit immediate at PC+4 |
| RRV | 2 | 8 | PC += 8 | 2 registers + 32-bit immediate at PC+4 |
| V | 2 | 8 | PC += 8 | 32-bit immediate at PC+4 |
| V64 | 3 | 12 | PC += 12 | lo32 at PC+4, hi32 at PC+8 (full 64-bit immediate) |

Immediate words are always inline (no literal pool). The instruction fetcher pre-fetches PC+4 in parallel with opcode decode when they are in the same cache doubleword.

### 4.2 Two Opcode Encodings

**3-register ALU format** (upper 16 bits non-zero):
```
[31:16] = operation code (e.g. 0x0001 = ADDR)
[15:12] = 0x0 (fixed zero field)
[11:8]  = rd   (destination register)
[7:4]   = rs1  (source 1)
[3:0]   = rs2  (source 2)
```

**Legacy / non-ALU format** (upper 16 bits = 0x0000):
```
[31:16] = 0x0000
[15:12] = primary opcode class
[11:8]  = secondary opcode
[7:4]   = rs1  (first operand; destination for COPY)
[3:0]   = rs2  (second operand; destination for single-R ops like INCR, SETR)
```

**Register field conventions by format:**
- **R**: destination/source in `[3:0]` (rs2 field)
- **RR**: rs1=`[7:4]`, rs2=`[3:0]`; for MEMSET ops: rs1=data, rs2=address; for MEMGET ops: rs1=destination, rs2=address; for COPY: rs1=destination, rs2=source
- **RRR (ALU)**: rd=`[11:8]`, rs1=`[7:4]`, rs2=`[3:0]`
- **RV, RRV**: as above with imm32 following

---

## 5. Immediate Extension Rules

| Instruction | Extension |
|-------------|-----------|
| SETR | Sign-extend imm32 → 64 bits |
| CMPRV | Sign-extend imm32 → 64 bits (comparison only) |
| ADDSP | Sign-extend imm32 → 64 bits (used for allocate/free) |
| ADDV, MINUSV | Zero-extend imm32 → 64 bits |
| ANDV, ORV, XORV | Zero-extend imm32 → 64 bits |
| PUSHV | Zero-extend imm32 → 64 bits |
| LDIDX\*, STIDX\* | Zero-extend imm32 → 64 bits (used as offset) |
| MULV, DIVV, MODV | Sign-extend imm32 → 64 bits |
| SETR64, PUSHV64 | Full 64 bits (V64 format, no extension) |

---

## 6. Condition Flags

Seven flags, set by various instructions:

| Flag | Name | Set by |
|------|------|--------|
| `zero_flag` | Zero | Arithmetic/logic when result == 0 |
| `sign_flag` | Sign | MSB of arithmetic result (result was negative) |
| `carry_flag` | Carry | Carry/borrow out of bit 63 |
| `overflow_flag` | Overflow | Signed overflow |
| `equal_flag` | Equal | CMPRR, CMPRV when operands equal |
| `less_flag` | Less (signed) | CMPRR, CMPRV signed less-than |
| `ult_flag` | Less (unsigned) | CMPRR, CMPRV unsigned less-than |

**Important distinction:**
- `zero_flag` is set by arithmetic operations (ADD, SUB, AND, etc.) when the **result** is zero.
- `equal_flag` is set only by `CMPRR`/`CMPRV` (explicit compare instructions).
- Conditional jumps use flags from the most recent instruction that set them. `JMPZ` tests `zero_flag`; `JMPE` tests `equal_flag`. Use `CMPRR`/`CMPRV` before `JMPE`/`JMPLT`/`JMPULT` etc.; arithmetic instructions before `JMPZ`/`JMPNZ`.

`SETFR` reads all flags into a register: `rd = {zero_flag, equal_flag, carry_flag, overflow_flag, 60'b0}`.

`IRET` restores PC and flags from the stack (interrupt return).

---

## 7. ALU Instructions

### 7.1 Register-Register (RRR format, opcode range 0x0001_0??? – 0x0053_0???)

| Mnemonic | Opcode | Operation |
|----------|--------|-----------|
| ADDR | 0x0001_0??? | rd = rs1 + rs2 (signed; sets zero/sign/carry/overflow) |
| SUBR | 0x0002_0??? | rd = rs1 − rs2 |
| ANDR | 0x0003_0??? | rd = rs1 & rs2 |
| ORR | 0x0004_0??? | rd = rs1 \| rs2 |
| XORR | 0x0005_0??? | rd = rs1 ^ rs2 |
| ADDC | 0x0006_0??? | rd = rs1 + rs2 + carry_flag |
| SUBC | 0x0007_0??? | rd = rs1 − rs2 − carry_flag |
| MULR | 0x0010_0??? | rd = rs1 × rs2 signed, lower 64 bits |
| MULUR | 0x0011_0??? | rd = rs1 × rs2 unsigned, lower 64 bits |
| MULHR | 0x0012_0??? | rd = upper64(rs1 × rs2) signed |
| MULHUR | 0x0013_0??? | rd = upper64(rs1 × rs2) unsigned |
| DIVR | 0x0014_0??? | rd = rs1 / rs2 signed, truncate toward zero |
| DIVUR | 0x0015_0??? | rd = rs1 / rs2 unsigned |
| MODR | 0x0016_0??? | rd = rs1 % rs2 signed |
| MODUR | 0x0017_0??? | rd = rs1 % rs2 unsigned |
| SHLR | 0x0020_0??? | rd = rs1 << rs2[5:0] logical left |
| SHRR | 0x0021_0??? | rd = rs1 >> rs2[5:0] logical right |
| SARR | 0x0022_0??? | rd = rs1 >>> rs2[5:0] arithmetic right |
| ROLR | 0x0023_0??? | rd = rs1 rotate-left rs2[5:0] |
| RORR | 0x0024_0??? | rd = rs1 rotate-right rs2[5:0] |
| CMPEQR–CMPUGER | 0x0030–0x0039_0??? | rd = 0 or 1 boolean (does not touch flags) |
| MINR/MAXR | 0x0040/0041_0??? | rd = min/max signed |
| MINUR/MAXUR | 0x0042/0043_0??? | rd = min/max unsigned |
| BSETRR/BCLRRR | 0x0050/0051_0??? | rd = rs1 with bit rs2[5:0] set/cleared |
| BTGLRR/BTSTRR | 0x0052/0053_0??? | toggle / test bit rs2[5:0] of rs1 |

### 7.2 Register-Immediate (RV or R format, legacy encoding)

| Mnemonic | Format | Operation |
|----------|--------|-----------|
| SETR | RV | rd = sign_ext(imm32) |
| SETR64 | V64 | rd = {hi32, lo32} full 64-bit |
| ADDV | RV | rd = rs + zero_ext(imm32) |
| MINUSV | RV | rd = rs − zero_ext(imm32) |
| ANDV/ORV/XORV | RV | rd = rs & \| ^ zero_ext(imm32) |
| CMPRV | RV | flags from rs − sign_ext(imm32), no writeback |
| CMPRR | RR | flags from rs1 − rs2, no writeback |
| INCR / DECR | R | rd = rs ± 1 |
| NEGR | R | rd = −rs (two's complement) |
| NOTR | R | rd = ~rs bitwise NOT |
| ABSR | R | rd = \|rs\| signed |
| SHLV/SHRV/SHRAV | RV | shift by imm[5:0] |
| SHLR1/SHRR1/SHRAR | R | shift by 1 |
| ROLV/RORV | RV | rotate by imm[5:0] |
| MULV/DIVV/MODV | RV | signed mul/div/mod by sign_ext(imm32) |
| SEXTB/SEXTW/SEXTH | R | sign-extend 8/32/16 bits to 64 |
| ZEXTB/ZEXTW/ZEXTH | R | zero-extend 8/32/16 bits to 64 |
| BSWAP | R | reverse all 8 bytes (endian swap) |
| POPCNT/CLZ/CTZ | R | population count / leading / trailing zeros |
| BSET/BCLR/BTGL/BTST | RV | bit set/clear/toggle/test by immediate position |
| BEXTR/BDEP | RV | bit-field extract/deposit |

---

## 8. Memory Access Instructions

All memory instructions that take register operands are **RR format** (1 word, 4 bytes, PC += 4) except indexed variants which are **RRV** (2 words). The register convention for all MEM ops: `rs1` = data (for stores) or destination (for loads), `rs2` = address.

### 8.1 Sub-word Load/Store (MEMSET8 family)

All are little-endian. The byte address in `rs2` addresses a byte within the DDR2 doubleword:
- `rs2[2:0]` = byte lane within the 8-byte doubleword (0 = LSByte = bits[7:0], 7 = MSByte = bits[63:56])
- `rs2[31:3]` = which 8-byte doubleword in the address space

| Mnemonic | Opcode | Description | Alignment |
|----------|--------|-------------|-----------|
| MEMSET8 | 0x0000_74?? | `mem8[rs2] = rs1[7:0]` | byte (no alignment) |
| MEMGET8 | 0x0000_75?? | `rd = zero_ext(mem8[rs2])` | byte (no alignment) |
| MEMSET16 | 0x0000_76?? | `mem16[rs2 & ~1] = rs1[15:0]` | 2-byte aligned |
| MEMGET16 | 0x0000_77?? | `rd = zero_ext(mem16[rs2 & ~1])` | 2-byte aligned |
| MEMSET32 | 0x0000_78?? | `mem32[rs2 & ~3] = rs1[31:0]` | 4-byte aligned |
| MEMGET32 | 0x0000_79?? | `rd = zero_ext(mem32[rs2 & ~3])` | 4-byte aligned |
| MEMSET64 | 0x0000_7A?? | `mem64[rs2 & ~7] = rs1` | 8-byte aligned |
| MEMGET64 | 0x0000_7B?? | `rd = mem64[rs2 & ~7]` | 8-byte aligned |

Hardware enforces alignment by masking the low address bits. Unaligned stores write to the aligned slot silently (no fault).

### 8.2 Full-width 64-bit Load/Store

| Mnemonic | Opcode | Format | Description |
|----------|--------|--------|-------------|
| MEMSET64RR | 0x0000_70?? | RR | `mem64[rs2] = rs1`; 8-byte aligned |
| MEMREADRR | 0x0000_71?? | RR | `rd = mem64[rs2]`; 8-byte aligned |
| MEMSETR | 0x0000_720? | RV | `mem64[imm32] = rs`; imm32 is absolute address |
| MEMREADR | 0x0000_721? | RV | `rd = mem64[imm32]`; imm32 is absolute address |

### 8.3 Indexed Load/Store

All use **RRV format**: opcode word + imm32 at PC+4. Effective address = `rs2[31:0] + zero_ext(imm32)`.

| Mnemonic | Opcode | Width | Notes |
|----------|--------|-------|-------|
| LDIDX64 | 0x0000_0C?? | 64-bit | `rd = mem64[rs2 + zero_ext(imm32)]` |
| STIDX64 | 0x0000_0D?? | 64-bit | `mem64[rs2 + zero_ext(imm32)] = rs1` |
| LDIDX64R | 0x0000_0E?? | 64-bit | `rd = mem64[rs2 + reg[imm[3:0]]]` (3-cycle pipeline) |
| STIDX64R | 0x0000_73?? | 64-bit | `mem64[rs2 + reg[imm[3:0]]] = rs1` |
| LDIDX32 | 0x0000_C0?? | 32-bit | `rd = zero_ext(mem32[(rs2+imm32) & ~3])` |
| STIDX32 | 0x0000_C1?? | 32-bit | `mem32[(rs2+imm32) & ~3] = rs1[31:0]` |
| LDIDX16 | 0x0000_C2?? | 16-bit | `rd = zero_ext(mem16[(rs2+imm32) & ~1])` |
| STIDX16 | 0x0000_C3?? | 16-bit | `mem16[(rs2+imm32) & ~1] = rs1[15:0]` |
| LDIDX8 | 0x0000_C4?? | 8-bit | `rd = zero_ext(mem8[rs2+imm32])` |
| STIDX8 | 0x0000_C5?? | 8-bit | `mem8[rs2+imm32] = rs1[7:0]` |

For struct/array access, `LDIDX64`/`STIDX64` are the workhorses: `rs2` = base pointer, `imm32` = constant byte offset (zero-extended, so max offset = 4 GB).

---

## 9. Flow Control

All jumps use **absolute 32-bit byte addresses**. All branch instructions are 2-word (V format, PC += 8).

### 9.1 Unconditional

| Mnemonic | Format | Operation |
|----------|--------|-----------|
| JMP | V | PC = imm32 |
| JMPR | R | PC = rs2[31:0] |

### 9.2 Conditional (V format, 2 words)

Flags are tested as-is; no compare is implicit.

| Mnemonic | Condition |
|----------|-----------|
| JMPZ / JMPNZ | zero_flag |
| JMPE / JMPNE | equal_flag (set by CMPRR/CMPRV) |
| JMPC / JMPNC | carry_flag |
| JMPO / JMPNO | overflow_flag |
| JMPS / JMPNS | sign_flag |
| JMPLT / JMPLE / JMPGT / JMPGE | less_flag (signed, from CMPRR/CMPRV) |
| JMPULT / JMPULE / JMPUGT / JMPUGE | ult_flag (unsigned, from CMPRR/CMPRV) |

**Typical compare-and-branch pattern:**
```
CMPRR  rs1, rs2        ; sets less_flag, equal_flag, ult_flag (1 word, PC+=4)
JMPLT  target          ; 2 words, PC+=8
```

### 9.3 Call and Return

| Mnemonic | Format | Operation |
|----------|--------|-----------|
| CALL | V | SP−=8; mem64[SP] = PC+8; PC = imm32 |
| CALLR | R | SP−=8; mem64[SP] = PC+4; PC = rs2[31:0] |
| CALLZ/CALLNZ/CALLE/CALLNE/... | V | Conditional CALL |
| RET | 1-word | tmp = mem64[SP]; SP+=8; PC = tmp[31:0] |

Return address is stored as a 64-bit value (upper 32 bits zero). Only bits[31:0] are used as the return PC.

---

## 10. Stack

**Convention**: full-descending. SP points to the **last-pushed** item (lowest occupied address).

```
PUSH rs2      ; SP -= 8; mem64[SP] = rs2              (8 bytes, 1-word instruction)
POP  rd       ; rd = mem64[SP]; SP += 8               (8 bytes, 1-word instruction)
PUSHV imm32   ; SP -= 8; mem64[SP] = zero_ext(imm32)  (2-word)
PUSHV64 lo,hi ; SP -= 8; mem64[SP] = {hi32,lo32}      (3-word, V64)
GETSP rd      ; rd = zero_ext(SP)
SETSP rs      ; SP = rs[31:0]
ADDSP imm32   ; SP += sign_ext(imm32)   (negative = allocate locals, positive = free)
```

Stack slots are always 8 bytes, regardless of the value size. Stack is in DDR2 RAM, not in registers — every push/pop requires a cache access.

**Calling convention (software convention — not hardware-enforced):**
1. Caller places arguments via registers or pushes to stack (ABI defined by assembler/compiler).
2. CALL pushes the 8-byte return address (zero_ext(PC+8) for V format CALL).
3. Callee may push R15 (frame pointer) and set FP = SP.
4. Local variables allocated via `ADDSP -N` (subtracts N bytes).
5. Locals freed with `ADDSP +N`.
6. RET pops 8 bytes and restores PC from bits[31:0].

---

## 11. C Type Mappings

| C type | Size | Alignment | Load/Store instruction |
|--------|------|-----------|------------------------|
| `char` / `uint8_t` | 1 byte | 1 | MEMSET8 / MEMGET8, STIDX8 / LDIDX8 |
| `short` / `uint16_t` | 2 bytes | 2 | MEMSET16 / MEMGET16, STIDX16 / LDIDX16 |
| `int` / `uint32_t` | 4 bytes | 4 | MEMSET32 / MEMGET32, STIDX32 / LDIDX32 |
| `long` / `uint64_t` | 8 bytes | 8 | MEMSET64 / MEMGET64, STIDX64 / LDIDX64 |
| pointer | 4 bytes | 4 | 32-bit addresses; zero-extend to 64 in registers |

Pointers are 32-bit values (the address space is 32-bit). When loaded into a 64-bit register the upper 32 bits will be zero. The compiler should use MEMGET32 / LDIDX32 to load pointers from memory, and MEMSET32 / STIDX32 for pointer writes.

**Signed vs unsigned loads**: all sub-64-bit loads (MEMGET8/16/32, LDIDX8/16/32) are **zero-extending**. For signed C types the compiler must follow the load with a sign-extension instruction:
- `SEXTB` after loading `signed char` (sign-extend 8→64)
- `SEXTH` after loading `signed short` (sign-extend 16→64)
- `SEXTW` after loading `int` / `int32_t` (sign-extend 32→64)

---

## 12. Special Instructions

| Mnemonic | Description |
|----------|-------------|
| COPY RR | `rd = rs` (1 word; rd is in rs1 field [7:4], source is rs2 field [3:0]) |
| SETFR R | `rd = {zero_flag, equal_flag, carry_flag, overflow_flag, 60'b0}` |
| BSWAP R | byte-reverse all 8 bytes (useful for explicit big-endian I/O) |
| NOP | no operation; PC+=4 |
| HALT | freeze execution until hard reset. **Note:** the pipeline fetches the word immediately after HALT before freezing — place a NOP or a second HALT after every HALT to prevent the following data from being interpreted as an instruction. |
| RESET | restart: PC = 0x0020 (first instruction). **Note:** the current hardware reset vector may target 0x0004; if so, place a `JMP 0x0020` at that address to redirect into code. |
| DELAYR R | spin-wait rs2 clock cycles |
| DELAYV V | spin-wait imm32 clock cycles |

---

## 13. I/O Peripherals

The CPU has hardware instructions (not memory-mapped) for on-board peripherals:
- **UART** (opcodes 5xxx): `TXR`, `TXCHARMEMR`, `TXSTRMEMR`, `RXRB`/`RXRNB` for serial debug output/input.
  - `TXCHARMEMR rs` — transmit one byte from memory. The hardware reads the 64-bit doubleword at `rs & ~7`, then selects byte lane `rs[2:0]` (identical byte-lane semantics to `MEMGET8`): lane 0 = bits[7:0] (lowest address/LSByte), lane 7 = bits[63:56]. The selected byte is transmitted over UART.
  - `TXSTRMEMR rs` — transmit 8 bytes from memory starting at `rs`, in little-endian order (byte at `rs` first, byte at `rs+7` last). Byte-lane selection within the doubleword follows the same scheme as `TXCHARMEMR`.
- **LCD** (opcodes 2xxx): SPI LCD command/data send.
- **LEDs** (opcodes 3xxx): LED register, RGB LEDs, 7-segment displays, switches.
- **Interrupts** (opcodes 6xxx): `INTSETRR` to set handler address; `IRET` for interrupt return.

These are unlikely to be generated by a C compiler directly but are available via `__asm` or intrinsic wrappers.

---

## 14. Fixed Issues (April 2026)

The following bugs were present in the original big-endian implementation and were fixed during the little-endian conversion:

**SETR64** (`register_tasks.vh` line 395): the hi32 word (at PC+8) was selected from the wrong half of the cache doubleword — `r_PC[2]` condition was inverted. Fixed: `r_PC[2]=0` now correctly reads bits[31:0] (low half), `r_PC[2]=1` reads bits[63:32] (high half).

**PUSHV64** (`stack_tasks.vh` line 143): hi32 fetch always used `w_mem_read_data[63:32]` regardless of instruction alignment. Fixed: now selects the correct half based on `r_PC[2]`, matching SETR64.

**LDIDX32** (`alu_extended_tasks.vh` line 1096): the 32-bit half-select after a cache fetch had `addr[2]` inverted. Fixed: `effective_addr[2]=0` now correctly reads bits[31:0], `effective_addr[2]=1` reads bits[63:32].

**STIDX32** (`alu_extended_tasks.vh` line 1116): the byte-enable mask had the same inversion. Fixed: `effective_addr[2]=0` now enables the lower four byte lanes (`8'b0000_1111`), `effective_addr[2]=1` enables the upper four (`8'b1111_0000`).

---

## 15. Instruction Encoding Quick Reference

```
3-register ALU:  [31:16]=opN  [15:12]=0  [11:8]=rd  [7:4]=rs1  [3:0]=rs2
Legacy R:        [31:16]=0    [15:8]=op  [7:4]=rs1  [3:0]=rs2  (rd usually in rs2 field)
Legacy RR:       [31:16]=0    [15:8]=op  [7:4]=rs1  [3:0]=rs2
Legacy RV:       word0 as RR; word1 = imm32
Legacy V:        word0: [31:16]=0, [15:8]=op, [7:0]=0x00; word1 = imm32
V64:             word0 as above; word1 = lo32; word2 = hi32
```

Code and data are stored **little-endian**: at address A where `A[2]=0`, the 32-bit instruction word occupies bits [31:0] of the 64-bit DDR doubleword; at `A[2]=1` it occupies bits [63:32].

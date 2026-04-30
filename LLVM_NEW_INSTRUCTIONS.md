# New KlaussCPU Instructions for LLVM Backend

Status of the FPGA-side changes from `FPGA_FIXES.md` as of the current branch.
This is a one-stop reference for the LLVM team to wire up patterns and
remove existing software workarounds.

---

## Summary

| # | Instruction(s) | Status | LLVM action |
|---|---|---|---|
| 1 | LE physical memory | ✅ on hardware | Already in use; toolchain emits LE byte order |
| 2 | `JMPR` / `CALLR` (indirect branch / call) | ✅ on hardware | Wire `ISD::BRIND` and reg-target `ISD::CALL`; drop `setMinimumJumpTableEntries(INT_MAX)` |
| 3 | `MEMGET32` unaligned | ✅ on hardware | `ALIGN i32` may now be `1` (any byte alignment) for `mem32` operands |
| 4 | `ADDI` rd=rs+sign_ext(imm32) | ✅ on hardware | Use for `FrameIndex` materialisation; remove `SETR + ADDR` workaround |
| 5 | `LDIDX8_S` / `LDIDX16_S` sign-extending loads | ✅ on hardware | Mark `SEXTLOAD i8/i16` as `Legal`; emit single-instruction sextloads |

---

## 2. JMPR / CALLR — indirect register branch and call

### `JMPR` — unconditional jump to address in register

- **Encoding:** `0x0000_102N` where `N = rs2[3:0]` is the source register holding the target byte address.
- **Format:** R (1-word, `PC += 0` — branches absolutely).
- **Operation:** `PC = reg[N][31:0]`. Sets no flags. Does not push a return address.
- **Reads:** lower 32 bits of `rs2`. Address is treated as a byte address (must be word-aligned).

### `CALLR` — call to address in register

- **Encoding:** `0x0000_407N` where `N = rs2[3:0]` is the target register.
- **Format:** R (1-word, `SP -= 8`, `PC = reg[N][31:0]`).
- **Operation:** Pushes `PC + 4` (return address) to the DDR2 stack as a zero-extended 64-bit value, then jumps to `reg[N][31:0]`. Mirror of the existing direct `CALL imm32` (`0x0000_1009`).

### LLVM changes needed

Both instructions are already implemented in the Verilog (the FPGA_FIXES.md
note saying they were missing was stale documentation). The backend can:

- Remove `setOperationAction(ISD::BRIND, Expand)` and the
  `setMinimumJumpTableEntries(INT_MAX)` from `KlaussCPUISelLowering.cpp`.
- Add `JMPR` / `CALLR` instruction defs to `KlaussCPUInstrInfo.td`:

  ```tablegen
  def JMPR  : InstKlaussCPU<(outs), (ins GPR:$rs),
                            "jmpr $rs", [(brind GPR:$rs)]> {
    let Inst{31:8} = 0x000010;
    let Inst{7:4}  = 0;
    let Inst{3:0}  = rs;
    let isBranch = 1; let isTerminator = 1; let isBarrier = 1; let isIndirectBranch = 1;
  }

  def CALLR : InstKlaussCPU<(outs), (ins GPR:$rs),
                            "callr $rs", [(KlaussCPUcall GPR:$rs)]> {
    let Inst{31:8} = 0x000040;
    let Inst{7:4}  = 7;     // 0x407N
    let Inst{3:0}  = rs;
    let isCall = 1;
  }
  ```

  (Adjust to match your existing `Inst` field encoding.)

- Add a pattern wiring `ISD::CALL` with a `GPR` target to `CALLR`.

---

## 3. MEMGET32 unaligned addresses

### Old behaviour

`MEMGET32` aligned its address argument down to the nearest 4-byte boundary
(`addr & ~3`), so a load from an unaligned pointer silently returned bytes
from the previous word. The LLVM backend had to lower unaligned 32-bit loads
to four `LDIDX8 + shift + or` sequences.

### New behaviour

`MEMGET32 rs1, rs2` (`0x0000_79RR`, RR format, 1-word) now reads the **4
bytes starting at the exact byte address in `rs2`**, with no alignment
constraint. Result is zero-extended into `rs1`.

| `rs2[2:0]` (offset) | `rs2[3]` | Latency | Behaviour |
|---|---|---|---|
| `0` (4-byte aligned) | any | 1 cache transaction | Aligned read, fast path |
| `1..4` | any | 1 cache transaction | All 4 bytes within one returned doubleword |
| `5..7` | `0` | 1 cache transaction | Spans within same cache line; fed by cache lookahead (`o_mem_read_data_next`) |
| `5..7` | `1` | 2 cache transactions | Spans into next cache line; needs second read |

Worst case (cross-line span) takes one extra cache transaction (~5–10 cycles
for a hit, longer on a line miss). Aligned and same-line-spanning cases are
unchanged in latency from the old aligned-only implementation.

### LLVM changes needed

- For `i32` loads where the underlying alignment is `< 4`, you can now emit
  a single `MEMGET32` instead of the byte-shift-or sequence.
- The `ALLOWS_UNALIGNED_MEM_ACCESSES` (or equivalent) hook can return `true`
  for `i32`. **Note:** `i64` (`MEMGET64`) has *not* been updated — it still
  requires 8-byte alignment. Same for `MEMSET32` (write path is still
  4-byte aligned).
- `__builtin_klausscpu_memget32` no longer needs the byte-extraction prefix
  in `uart_stubs.c` (and `uart_puts` should already have switched to
  `TXSTRMEMR` per Fix 1).

### Interaction with stores

`MEMSET32` (write) is **still 4-byte aligned**. If a corresponding
unaligned 32-bit store is needed, lower it to 4× `STIDX8` for now.

---

## 4. ADDI — sign-extending immediate add

### Encoding

- **Opcode:** `0x0000_02RR` (RRV format, 2-word: opcode + imm32 at PC+4).
- **Encoding:** `[7:4] = rd`, `[3:0] = rs`. PC += 8.
- **Operation:** `rd = rs + sign_extend(imm32)`. Sets `zero`, `sign`,
  `carry`, `overflow` flags (same convention as `ADDV` / `ADDR`).

### Why it's distinct from ADDV

`ADDV` (`0x0000_081?`) is RV (single register, in-place: `rd = rd + imm`)
and **zero-extends** the immediate. `ADDI` is the RRV variant that allows
`rd ≠ rs` AND sign-extends the immediate, making it correct for negative
offsets like frame addresses (`R15 - 32`, `R15 - 8`, etc.).

### LLVM changes needed

- Add `ADDI` to `KlaussCPUInstrInfo.td`:

  ```tablegen
  def ADDI : InstKlaussCPU<(outs GPR:$rd), (ins GPR:$rs, simm32:$imm),
                           "addi $rd, $rs, $imm",
                           [(set GPR:$rd, (add GPR:$rs, simm32:$imm))]> {
    let Inst{31:8} = 0x000002;
    let Inst{7:4}  = rd;
    let Inst{3:0}  = rs;
    // imm32 follows in next word
  }
  ```

- **Simplify `eliminateFrameIndex`** in `KlaussCPURegisterInfo.cpp`. The
  current sequence:

  ```asm
  setr  rd, <frame_offset>     ; SETR rd, imm32        — 8 bytes
  addr  rd, R15, rd            ; ADDR rd = R15 + rd    — 4 bytes
  ```

  becomes:

  ```asm
  addi  rd, R15, <frame_offset>  ; ADDI rd = R15 + imm32  — 8 bytes
  ```

  Saves 4 bytes and one cycle per frame-address materialisation.

- Remove the `ISD::FrameIndex → SETR(TFI)` special-case in
  `KlaussCPUISelDAGToDAG.cpp`; replace with the standard
  `ADDI(TFI_base, frame_offset)` lowering used by RISC-V-style backends.

---

## 5. LDIDX8_S / LDIDX16_S — sign-extending byte/halfword loads

### Encoding

| Instr      | Opcode        | Format                                                    | Operation |
|------------|---------------|-----------------------------------------------------------|-----------|
| `LDIDX8_S` | `0x0000_C6RR` | RRV (2-word). `[7:4] = rd`, `[3:0] = rs2`, imm32 at PC+4. | `rd = sign_ext(mem8[rs2 + zero_ext(imm32)])` |
| `LDIDX16_S`| `0x0000_C7RR` | RRV (2-word). Same field layout, halfword aligned.        | `rd = sign_ext(mem16[(rs2 + zero_ext(imm32)) & ~1])` |

These mirror the existing zero-extending `LDIDX8` (`0x0000_C4??`) and
`LDIDX16` (`0x0000_C2??`) — same address calculation, same byte-lane
semantics, only the upper-bits filling changes.

### LLVM changes needed

- Add `LDIDX8_S` / `LDIDX16_S` to `KlaussCPUInstrInfo.td`.
- In `KlaussCPUISelLowering.cpp`, change:

  ```cpp
  setLoadExtAction(ISD::SEXTLOAD, MVT::i64, MVT::i8,  Expand);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i64, MVT::i16, Expand);
  ```

  to `Legal`.

- Add tablegen `Pat` patterns:

  ```tablegen
  def : Pat<(i64 (sextloadi8  (add GPR:$rs2, simm32:$imm))),
            (LDIDX8_S  GPR:$rs2, simm32:$imm)>;
  def : Pat<(i64 (sextloadi16 (add GPR:$rs2, simm32:$imm))),
            (LDIDX16_S GPR:$rs2, simm32:$imm)>;
  // Plus the imm-only and reg-only variants with a zero offset.
  ```

- Existing `LDIDX8 + SEXTB` / `LDIDX16 + SEXTH` 2-instruction sequences
  collapse to a single instruction. No correctness change — only code-size
  and one-cycle perf.

---

## Things that have NOT changed

- **Byte ordering:** still pure little-endian throughout. Hex stream emitted
  by the toolchain must remain in LE byte order (LSByte first per word),
  and the loader checksum still operates on `o_ram_write_value` directly
  (the natural 32-bit value). No further change from the post-Fix-1 state.
- **`MEMGET64` / `MEMSET64`:** still 8-byte aligned only.
- **`MEMSET32`:** still 4-byte aligned only.
- **`MEMSET16` / `MEMSET8`:** unchanged (already byte-addressable in their
  existing forms).
- **`STIDX8_S` / `STIDX16_S`:** **do not exist** — sign-extending stores
  don't make sense (a store discards the upper bits anyway). Use the
  existing `STIDX8` / `STIDX16`.
- **TXSTRMEMR / TXSTRMEM:** the multi-chunk null-terminated string transmit
  path is fully working (handles unaligned base addresses, multi-chunk
  transmission, UART handshake). `uart_puts` should be a single
  `TXSTRMEMR` instruction with no software loop.

---

## Quick encoding reference

```
ADDI       0x0000_02RR     [7:4]=rd, [3:0]=rs, imm32 at PC+4    PC+=8
JMPR       0x0000_102N     [3:0]=rs                              PC=rs[31:0]
CALLR      0x0000_407N     [3:0]=rs                              SP-=8, PC=rs[31:0]
LDIDX8_S   0x0000_C6RR     [7:4]=rd, [3:0]=rs2, imm32 at PC+4    PC+=8
LDIDX16_S  0x0000_C7RR     [7:4]=rd, [3:0]=rs2, imm32 at PC+4    PC+=8
MEMGET32   0x0000_79RR     [7:4]=rd, [3:0]=rs2 (any alignment)   PC+=4
```

R = 1 reg field, RR = 2 reg fields, RRV = 2 reg fields + imm32 word.
All values little-endian on the wire; opcode bits as shown above are the
natural 32-bit value the CPU sees after the LE byte unpack.

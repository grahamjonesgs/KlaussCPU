# LLVM Backend Readiness Plan

## Overview

Five items stand between the current ISA and a working LLVM backend.  One requires
a hardware change (IRET); the rest are LLVM TableGen/C++ configuration.

| # | Item | Scope | Priority |
|---|------|-------|----------|
| 1 | IRET — interrupt return (+ MMIO interrupt controller) | Hardware (RTL + opcode + MMIO) — **shipped** | **Critical** |
| 2 | TRAP opcode | Hardware (RTL + opcode) | Recommended |
| 3 | Sign-extending loads | LLVM backend only | Required |
| 4 | Conditional move (SELECT) | LLVM backend only | Required |
| 5 | Soft-float | LLVM backend only | Required |
| 6 | Flag-model TableGen patterns | LLVM backend only | Required |

---

## Item 1 — IRET (hardware change, critical) — **DONE**

> **Status:** Implemented. The dispatch path saves flags and the per-source
> interrupt mask alongside PC, and `IRET` restores all three. All other
> interrupt control (handler table, mask, timer period) is exposed via MMIO
> at base `0xF00F_0000` rather than dedicated opcodes — `IRET` is the only
> interrupt-related opcode. See `CPU_ARCHITECTURE.md` §13.1 and
> `MMIO_MAP.md` "Timers / interrupts" for the runtime model. The summary
> below reflects the shipped layout.

### Original problem

The interrupt dispatch hardware originally pushed only `{32'b0, r_PC}` onto the
stack and provided no IRET opcode, so handlers could not return. Flags were
also not saved, so any handler touching arithmetic corrupted the interrupted
program's flags.

### Fix — pack mask + flags + PC into one 64-bit slot

The push word is 64 bits wide; the lower 32 carry PC, the next 7 bits hold the
flags, and the next 4 bits hold the per-source interrupt mask. The remaining
21 high bits are reserved (zero).

```
Saved slot layout (64-bit):
  [63:43]  = 21'b0 (reserved / zero)
  [42:39]  = r_int_mask  (per-source enable; restored by IRET)
  [38]     = r_zero_flag
  [37]     = r_equal_flag
  [36]     = r_carry_flag
  [35]     = r_overflow_flag
  [34]     = r_sign_flag
  [33]     = r_less_flag
  [32]     = r_ult_flag
  [31:0]   = r_PC (byte address of interrupted instruction)
```

#### 1a. FPGA_CPU_32_bits_cache.v — interrupt dispatch

File: `KlaussCPU.srcs/sources_1/new/FPGA_CPU_32_bits_cache.v`

The dispatch block now (a) gates on `r_int_mask[N]`, (b) packs mask + flags + PC,
and (c) auto-clears the dispatched source's mask bit so the handler cannot
re-enter on the same source:

```verilog
end else if (r_timer_interrupt && r_interrupt_table[0] != 32'h0 && r_int_mask[0]) begin
   r_SP             <= r_SP - 8;
   r_mem_addr       <= r_SP - 32'd8;
   r_mem_write_data <= {21'b0, r_int_mask,
                        r_zero_flag, r_equal_flag, r_carry_flag,
                        r_overflow_flag, r_sign_flag, r_less_flag, r_ult_flag,
                        r_PC};
   r_mem_byte_en     <= 8'hFF;
   r_mem_write_DV    <= 1'b1;
   r_timer_interrupt <= 1'b0;
   r_int_mask[0]     <= 1'b0;        // mask source 0 while handler runs
   r_PC              <= r_interrupt_table[0];
   r_int_push_wait   <= 1'b1;
end
```

The free-running counter compares against `r_timer_period` (32-bit register)
instead of the previous hardcoded `0xFFFFF`. Reset value of `r_timer_period`
is `0x000F_FFFF` (≈10.5 ms @ 100 MHz); reset value of `r_int_mask` is `4'h0`
(all sources masked).

#### 1b. control_tasks.vh — t_iret only

File: `KlaussCPU.srcs/sources_1/new/control_tasks.vh`

`t_iret` is the only interrupt-related task; mask/period/handler writes
are handled by the MMIO write block in `FPGA_CPU_32_bits_cache.v` (see
1d).

```verilog
// IRET — return from interrupt handler.
// Pops the 64-bit context slot saved by interrupt dispatch:
//   [31:0]   → PC          (resume address)
//   [38:32]  → flags       (zero, equal, carry, overflow, sign, less, ult)
//   [42:39]  → r_int_mask  (per-source enables, restored)
// Uses the same multi-cycle DDR2 read pattern as t_ret.
task t_iret;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr    <= r_SP;
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_PC            <= w_mem_read_data[31:0];
            r_zero_flag     <= w_mem_read_data[38];
            r_equal_flag    <= w_mem_read_data[37];
            r_carry_flag    <= w_mem_read_data[36];
            r_overflow_flag <= w_mem_read_data[35];
            r_sign_flag     <= w_mem_read_data[34];
            r_less_flag     <= w_mem_read_data[33];
            r_ult_flag      <= w_mem_read_data[32];
            r_int_mask      <= w_mem_read_data[42:39];
            r_SP            <= r_SP + 8;
            r_mem_read_DV   <= 1'b0;
            r_SM            <= OPCODE_REQUEST;
         end
      end
   end
endtask
```

#### 1c. opcode_select.vh — IRET only

File: `KlaussCPU.srcs/sources_1/new/opcode_select.vh`

The 6xxx block now has a single dispatch entry; everything else is MMIO.

```verilog
//=====================================================================
// Interrupt control (6xxx)
// IRET is the only interrupt-control opcode; everything else (handler
// table, per-source mask, timer period) is configured via MMIO at base
// 0xF00F_0000. See MMIO_MAP.md.
//=====================================================================
32'h0000_6011: t_iret;                                // IRET restore PC, flags, mask from stack
```

#### 1d. FPGA_CPU_32_bits_cache.v — MMIO interrupt controller

The MMIO write block handles all configuration; the read mux exposes the
same registers plus a live pending bit and a free-running counter. See
`MMIO_MAP.md` "Timers / interrupts" for the offsets.

```verilog
12'h00F: begin  // Interrupt controller / timer
   case (w_mmio_addr[15:0])
      16'h0000: r_int_mask           <= w_mmio_write_data[3:0];
      16'h0010: r_interrupt_table[0] <= w_mmio_write_data[31:0];
      16'h0018: r_interrupt_table[1] <= w_mmio_write_data[31:0];
      16'h0020: r_interrupt_table[2] <= w_mmio_write_data[31:0];
      16'h0028: r_interrupt_table[3] <= w_mmio_write_data[31:0];
      16'h0030: begin
         r_timer_period            <= w_mmio_write_data[31:0];
         r_timer_interrupt_counter <= 32'h0;
      end
      default: ;
   endcase
end
```

### Assembler support

The assembler needs only one new zero-operand mnemonic; mask/period/vector
configuration is plain `MEMSET32` to the MMIO addresses listed in
`MMIO_MAP.md`.

| Mnemonic   | Format | Encoding         | Operand semantics |
|------------|--------|------------------|-------------------|
| `IRET`     | none   | `0x0000_6011`    | (no operands; PC += 4) |

---

## Item 2 — TRAP opcode (hardware, recommended)

### Problem

`ISD::TRAP` in LLVM signals an unconditional abort (assert failure, UB trap).  If this
is lowered to `HALT`, a crash and a normal halt are indistinguishable at the board level.

### Fix — dedicated TRAP opcode with distinct error code

#### 2a. FPGA_CPU_32_bits_cache.v — add ERR_TRAP error code

File: `FPGA_CPU_32_DDR_cache.srcs/sources_1/new/FPGA_CPU_32_bits_cache.v`

```verilog
// Old error codes end at ERR_SEG_EXEC_DATA = 8'h8
localparam ERR_TRAP = 8'h9;   // Explicit software trap (TRAP opcode)
```

#### 2b. control_tasks.vh — add t_trap task

```verilog
// TRAP — software trap.  Halts with ERR_TRAP error code.
// Distinguishable from HALT (normal stop) and ERR_INV_OPCODE (illegal instruction).
task t_trap;
   begin
      r_error_code <= ERR_TRAP;
      r_SM         <= HCF_1;
   end
endtask
```

#### 2c. opcode_select.vh — assign opcode

In the Miscellaneous (Fxxx) section, after `DELAYV`:

```verilog
32'h0000_F014: t_trap;   // TRAP software trap; HCF with ERR_TRAP (0x9)
```

---

## Item 3 — Sign-extending loads (LLVM backend only)

### Problem

The ISA has zero-extending sub-word loads (`MEMGET8`, `MEMGET16`, `MEMGET32`) and
explicit sign-extend instructions (`SEXTB`, `SEXTH`, `SEXTW`).  There are no single-cycle
signed loads (`MEMGET8S`, etc.).

LLVM `SelectionDAGTargetInfo` will attempt `sextload` patterns.  Without a pattern match
it will ICE unless the operation is explicitly expanded.

### Fix — LLVM TargetLowering

In your target's `TargetLowering` constructor (the file that extends
`TargetLowering`/`KLACPUTargetLowering` — likely `KLACPUISelLowering.cpp`):

```cpp
// Zero-extending loads are natively supported; sign-extending loads are not.
// Expand them: the DAG legaliser will emit a zero-load + sign-extend pair,
// matching the existing MEMGET8/SEXTB, MEMGET16/SEXTH, MEMGET32/SEXTW sequences.
for (MVT VT : {MVT::i8, MVT::i16, MVT::i32}) {
    setLoadExtAction(ISD::SEXTLOAD, MVT::i64, VT, Expand);
    setLoadExtAction(ISD::EXTLOAD,  MVT::i64, VT, Expand);
    // Zero-extending loads stay Legal (default, but make explicit for clarity)
    setLoadExtAction(ISD::ZEXTLOAD, MVT::i64, VT, Legal);
}
```

No hardware change needed.  Cost: one extra instruction per signed sub-word load
(`SEXTB`/`SEXTH`/`SEXTW` after the zero-extending load).

---

## Item 4 — Conditional move / SELECT (LLVM backend only)

### Problem

The ISA has no `CMOVcc`-style instruction.  LLVM will try to lower `ISD::SELECT` to a
conditional move unless told otherwise.

### Fix — LLVM TargetLowering

```cpp
// No conditional-move instruction exists.  Expand SELECT to a branch diamond.
// The DAG legaliser emits: test condition → branch → phi merge.
setOperationAction(ISD::SELECT,    MVT::i64, Expand);
setOperationAction(ISD::SELECT_CC, MVT::i64, Expand);
```

No hardware change needed.  Cost: every ternary expression becomes a short branch
sequence instead of a branchless CMOVcc.  This is acceptable for an in-order
non-superscalar CPU — branch prediction overhead is minimal.

---

## Item 5 — Soft-float (LLVM backend only)

### Problem

The CPU has no floating-point hardware.  LLVM must emit calls to soft-float library
routines (`__addsf3`, `__mulsf3`, etc.) for all floating-point operations.

### Fix — subtarget declaration

In `KLACPUSubtarget.cpp` (or wherever your subtarget is defined):

```cpp
// Tell LLVM this target has no FP hardware.
// All FP operations are lowered to libcall sequences.
bool KLACPUSubtarget::usesSoftFloat() const { return true; }
```

And in `KLACPUSubtarget.h`:

```cpp
bool usesSoftFloat() const override;
```

Also in `KLACPUISelLowering.cpp`, register the softfloat library:

```cpp
// Use the compiler-rt / libgcc soft-float ABI.
setLibcallName(RTLIB::ADD_F32,  "__addsf3");
setLibcallName(RTLIB::ADD_F64,  "__adddf3");
// ... etc. for all FP operations
```

No hardware change needed.

---

## Item 6 — Flag-model TableGen patterns (LLVM backend only)

### Problem

The CPU uses a two-step compare-then-branch model:
- `CMPRR rs1, rs2` / `CMPRV rs, imm` — set `equal_flag`, `less_flag`, `ult_flag`
- `JMPcc target` — branch on the chosen flag

LLVM's SelectionDAG represents this as `(brcond (setcc ...) ...)`.  Without explicit
TableGen patterns, isel will not combine the setcc + brcond into the correct
`CMPRR` + `JMPcc` pair and may instead lower via the slower `CMPEQRcc` boolean
compare instructions that write a register.

There is also a hazard: `ADDR`/`SUBR` set `zero_flag`, while `CMPRR` sets
`equal_flag`.  These must never be mixed (e.g., `ADDR` followed by `JMPE` checks
the wrong flag).

### Fix — TableGen patterns

In `KLACPUInstrInfo.td`, add combined isel patterns for each condition:

```tablegen
// Signed equal/not-equal — uses equal_flag from CMPRR
def : Pat<(brcond (i64 (seteq GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPE (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

def : Pat<(brcond (i64 (setne GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPNE (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

// Signed less/greater — uses less_flag from CMPRR
def : Pat<(brcond (i64 (setlt GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPLT (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

def : Pat<(brcond (i64 (setle GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPLE (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

def : Pat<(brcond (i64 (setgt GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPGT (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

def : Pat<(brcond (i64 (setge GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPGE (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

// Unsigned less/greater — uses ult_flag from CMPRR
def : Pat<(brcond (i64 (setult GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPULT (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

def : Pat<(brcond (i64 (setule GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPULE (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

def : Pat<(brcond (i64 (setugt GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPUGT (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

def : Pat<(brcond (i64 (setuge GPR:$rs1, GPR:$rs2)), bb:$dst),
          (JMPUGE (CMPRR GPR:$rs1, GPR:$rs2), bb:$dst)>;

// Zero-test (from arithmetic) — uses zero_flag from ADDR/SUBR/etc.
// JMPZ checks zero_flag; JMPE checks equal_flag — they are different registers!
def : Pat<(brcond (i64 (seteq GPR:$rs, (i64 0))), bb:$dst),
          (JMPZ GPR:$rs, bb:$dst)>;   // after ADDR/SUBR that sets zero_flag
```

**Critical discipline**: never emit `JMPE` after an arithmetic instruction, and never
emit `JMPZ` after `CMPRR`.  The TableGen patterns enforce this naturally because
`JMPE` is only generated as the second word of a `CMPRR+JMPE` pair, and `JMPZ` is
matched against a compare-with-zero pattern that does not involve CMPRR.

---

## Implementation order

1. **IRET + MMIO interrupt controller** — **DONE** (hardware shipped;
   assembler/LLVM mnemonic for `IRET` still pending). Mask, timer period,
   and handler vectors are MMIO at `0xF00F_0000`, configured via plain
   `MEMSET32` — no dedicated opcodes.
   - ✅ `FPGA_CPU_32_bits_cache.v` dispatch saves flags + mask, gates on mask
   - ✅ MMIO read mux + write handler at offset `0x00F` in `FPGA_CPU_32_bits_cache.v`
   - ✅ `t_iret` in `control_tasks.vh`
   - ✅ Single dispatch entry in `opcode_select.vh` (`0x0000_6011`)
   - ⬜ Assembler mnemonic: `IRET`

2. **TRAP** — one-line hardware change, avoids debug confusion.
   - Add `ERR_TRAP` constant, `t_trap` task, opcode entry

3. **LLVM backend** — items 3–6 are all software, can be done in parallel with
   or after the hardware changes.
   - `setLoadExtAction` for sextload/extload
   - `setOperationAction` for SELECT/SELECT_CC
   - `usesSoftFloat()` in subtarget
   - Combined `CMPRR+JMPcc` TableGen patterns

---

## Opcode table additions summary

| Mnemonic    | Encoding              | Format | Action |
|-------------|-----------------------|--------|--------|
| `IRET`      | `0x0000_6011`         | (none) | Pop {mask[42:39],flags[38:32],PC[31:0]} from stack; SP+=8 |
| `TRAP`      | `0x0000_F014`         | (none) | HCF with ERR_TRAP (0x9) — software abort |

Mask, timer period, and handler vectors are reachable as plain MMIO stores
at `0xF00F_0000`+. See `MMIO_MAP.md` for the register layout.

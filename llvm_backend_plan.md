# LLVM Backend Readiness Plan

## Overview

Five items stand between the current ISA and a working LLVM backend.  One requires
a hardware change (IRET); the rest are LLVM TableGen/C++ configuration.

| # | Item | Scope | Priority |
|---|------|-------|----------|
| 1 | IRET — interrupt return | Hardware (RTL + opcode) | **Critical** |
| 2 | TRAP opcode | Hardware (RTL + opcode) | Recommended |
| 3 | Sign-extending loads | LLVM backend only | Required |
| 4 | Conditional move (SELECT) | LLVM backend only | Required |
| 5 | Soft-float | LLVM backend only | Required |
| 6 | Flag-model TableGen patterns | LLVM backend only | Required |

---

## Item 1 — IRET (hardware change, critical)

### Problem

The interrupt dispatch hardware (FPGA_CPU_32_bits_cache.v:763–773) pushes only
`{32'b0, r_PC}` onto the stack, then jumps to the handler.  There is no IRET opcode
to pop and restore that context.  An interrupt handler cannot return.

Additionally, flags are **not** saved, so any handler that touches arithmetic will
corrupt the interrupted program's flags.

### Fix — pack flags + PC into one 64-bit slot

The existing push word is 64 bits wide and the lower 32 carry PC.  The upper 32 are
currently zero.  Seven flags fit in 7 bits; pack them into `[38:32]`.

```
Saved slot layout (64-bit):
  [63:39]  = 25'b0 (reserved / zero)
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

File: `FPGA_CPU_32_DDR_cache.srcs/sources_1/new/FPGA_CPU_32_bits_cache.v`

Change line ~767:
```verilog
// Old
r_mem_write_data <= {32'b0, r_PC};

// New — pack 7 flags into [38:32], PC into [31:0]
r_mem_write_data <= {25'b0,
                     r_zero_flag, r_equal_flag, r_carry_flag,
                     r_overflow_flag, r_sign_flag, r_less_flag, r_ult_flag,
                     r_PC};
```

#### 1b. control_tasks.vh — add t_iret task

File: `FPGA_CPU_32_DDR_cache.srcs/sources_1/new/control_tasks.vh`

Add after `t_ret` (line ~67):

```verilog
// IRET — return from interrupt handler.
// Pops the 64-bit context slot saved by interrupt dispatch:
//   [31:0]  → PC (resume address)
//   [38:32] → flags (zero, equal, carry, overflow, sign, less, ult)
// Uses the same multi-cycle DDR2 read pattern as t_ret.
task t_iret;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr    <= r_SP;
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_PC           <= w_mem_read_data[31:0];
            r_zero_flag    <= w_mem_read_data[38];
            r_equal_flag   <= w_mem_read_data[37];
            r_carry_flag   <= w_mem_read_data[36];
            r_overflow_flag <= w_mem_read_data[35];
            r_sign_flag    <= w_mem_read_data[34];
            r_less_flag    <= w_mem_read_data[33];
            r_ult_flag     <= w_mem_read_data[32];
            r_SP           <= r_SP + 8;
            r_mem_read_DV  <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
         end
      end
   end
endtask
```

#### 1c. opcode_select.vh — assign opcode

File: `FPGA_CPU_32_DDR_cache.srcs/sources_1/new/opcode_select.vh`

In the interrupt control (6xxx) section, add after `INTSETRR`:

```verilog
32'h0000_6011: t_iret;   // IRET pop {flags,PC} from stack; resume interrupted context
```

Updated comment block:

```verilog
//=====================================================================
// Interrupt control (6xxx)
// INTSETRR: rs1[1:0] = interrupt number (0–3); rs2[31:0] = handler byte address.
// IRET:     pop 64-bit interrupt context; restore PC[31:0] and flags[38:32].
//=====================================================================
32'h0000_60??: t_set_interrupt_regs;   // INTSETRR RR set interrupt[rs1[1:0]] handler = rs2[31:0]
32'h0000_6011: t_iret;                 // IRET restore PC and flags from stack; SP+=8
```

### Assembler support

Add `IRET` as a zero-operand mnemonic (format: no operand word, PC += 4) to the
assembler.  The opcode word is `0x0000_6011`.

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

1. **IRET** — hardware change, enables interrupt-driven programs to be compiled at all.
   - Edit `FPGA_CPU_32_bits_cache.v` (interrupt dispatch, flags save)
   - Add `t_iret` to `control_tasks.vh`
   - Add opcode to `opcode_select.vh`
   - Add `IRET` mnemonic to assembler

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

| Mnemonic | Encoding      | Format | Action |
|----------|---------------|--------|--------|
| `IRET`   | `0x0000_6011` | (none) | Pop {flags[38:32],PC[31:0]} from stack; SP+=8 |
| `TRAP`   | `0x0000_F014` | (none) | HCF with ERR_TRAP (0x9) — software abort |

# Crash Dump Format

When the CPU enters the HCF (Halt-and-Catch-Fire) state — either because of a
hardware fault or an explicit `TRAP` opcode — it emits a multi-line crash dump
over UART before settling into the 7-segment error display loop. Each field is
a single line terminated with `\r\n`. After the footer, the TX line is held low
for ~2.5 frame times as a UART break, so a host parser can detect the
unambiguous end of the dump (mirrors the clean-halt path).

The dump is built by `t_hcf_dump_build_line` in
[uart_tasks.vh](KlaussCPU.srcs/sources_1/new/uart_tasks.vh) and driven by the
`HCF_DUMP` state in [KlaussCPU.v](KlaussCPU.srcs/sources_1/new/KlaussCPU.v).
Phase ordering and indices are defined by the `DUMP_*` localparams in
[KlaussCPU.v](KlaussCPU.srcs/sources_1/new/KlaussCPU.v).

## Layout

```
*** CRASH DUMP ***
ERR=xx PC=xxxxxxxx
OPC=xxxxxxxx SP=xxxxxxxx
V1=xxxxxxxx IDX=xxxxxxxx
V1H=xxxxxxxx
OPCM=xxxxxxxx
SM=xxxxxxxxx
IV0=xxxxxxxx
FLG Z=x E=x C=x V=x
    S=x L=x U=x
INSTR=NNNNNNNN
R0=NNNNNNNNNNNNNNNN
R1=NNNNNNNNNNNNNNNN
...
RF=NNNNNNNNNNNNNNNN
S0=NNNNNNNNNNNNNNNN
S1=NNNNNNNNNNNNNNNN
S2=NNNNNNNNNNNNNNNN
S3=NNNNNNNNNNNNNNNN
T0 P=xxxxxxxx OP=xxxxxxxx
T1 P=xxxxxxxx OP=xxxxxxxx
...
TF P=xxxxxxxx OP=xxxxxxxx
*** END ***
```

## Fields

| Field    | Width        | Meaning |
|----------|--------------|---------|
| `ERR`    | 8-bit        | Halt cause (see table below). |
| `PC`     | 32-bit       | Program counter at the fault. |
| `OPC`    | 32-bit       | Opcode word that the FSM was executing (`w_opcode`). |
| `SP`     | 32-bit       | Stack pointer. Stack grows downward; initial value `0x0800_0000`. |
| `V1`     | 32-bit       | Immediate operand at PC+4 (`w_var1`). The lo32 for V64 opcodes. |
| `IDX`    | 32-bit       | Saved base address used by indexed loads/stores (`r_idx_base_addr`). |
| `V1H`    | 32-bit       | Hi32 of a V64 immediate. Re-read from DRAM at PC+8 — see notes below. |
| `OPCM`   | 32-bit       | DRAM-side re-read of the word at PC. Compare against `OPC` to detect cache staleness. |
| `SM`     | 33-bit       | One-hot FSM state at fault time (`r_fault_sm` — a snapshot of `r_SM` taken before the HCF chain overwrites it). 9 hex digits — top nibble holds only bit 32 (`ALU_FINISH`). |
| `IV0`    | 32-bit       | Timer ISR entry vector (`r_interrupt_table[0]`) at crash time. Compare against `PC` to verify dispatch landed where software set it. |
| `Z E C V`| 1-bit each   | Zero, Equal, Carry, Overflow flags. |
| `S L U`  | 1-bit each   | Sign, signed Less-than, Unsigned-Less-than flags. |
| `INSTR`  | 32-bit       | Committed instruction count since program load (`LOAD_COMPLETE`). |
| `Rn`     | 64-bit × 16  | General registers R0..RF. |
| `Sn`     | 64-bit × 4   | Top 4 doublewords of stack: `S0 = mem64[SP]`, `Sn = mem64[SP + n*8]`. Reads beyond `STACK_TOP` print as `FFFF_FFFF_FFFF_FFFF`. |
| `Tn`     | `P=`32 + `OP=`32 × 16 | Fetch trace ring. T0 is the most recent fetch, T1 one before, … TF = 16 fetches back. |

## Error codes (`ERR`)

Defined in [KlaussCPU.v](KlaussCPU.srcs/sources_1/new/KlaussCPU.v) (`ERR_*`
localparams).

| Hex | Name                   | Trigger |
|-----|------------------------|---------|
| 01  | `ERR_INV_OPCODE`       | Decoder hit an unknown opcode. |
| 02  | `ERR_INV_FSM_STATE`    | FSM reached an unreachable state. Use `SM` to identify which one. |
| 03  | `ERR_STACK`            | Stack underflow / overflow. |
| 04  | `ERR_DATA_LOAD`        | Loader data error. |
| 05  | `ERR_CHECKSUM_LOAD`    | Loader checksum mismatch. |
| 06  | `ERR_OVERFLOW`         | Arithmetic overflow trapped. |
| 07  | `ERR_SEG_WRITE_TO_CODE`| Write into the code segment. |
| 08  | `ERR_SEG_EXEC_DATA`    | Execute from the data segment. |
| 09  | `ERR_TRAP`             | Software `TRAP` opcode (`0xF014`). |

## Reading a dump

1. **`ERR`** tells you the *kind* of fault.
2. **`PC` / `OPC` / `V1`** is the failing instruction (PC = address, OPC =
   encoded word, V1 = its 32-bit immediate).
3. **`OPCM` vs `OPC`** — if they differ, the opcode cache served a stale word.
   Equal values mean the fetch path is healthy and the bug is elsewhere.
4. **`V1H`** is meaningful only when the crashing opcode uses the **V64**
   encoding (3-word: opcode @ PC, lo32 @ PC+4, hi32 @ PC+8). Currently:
   `SETR64` (`0FE?`) and `PUSHV64` (`4060`). For other opcodes the field shows
   whatever happens to live at PC+8 — ignore it.
5. **`SM`** is the FSM state (one-hot). Cross-reference against the `localparam`
   block in [KlaussCPU.v](KlaussCPU.srcs/sources_1/new/KlaussCPU.v) (`OPCODE_REQUEST`,
   `OPCODE_FETCH`, `OPCODE_EXECUTE`, `WRITEBACK`, `MULTIPLY_*`, `DIVIDE_STEP`,
   etc.). Combined with `ERR=02` it pinpoints the unreachable state.
6. **`IV0` vs `PC`** — for crashes inside (or just after) a timer interrupt, this
   tells you whether the dispatch went where software set it. `PC == IV0` means
   the jump landed correctly and the fault is in the ISR body; `PC != IV0` means
   either the dispatch went to the wrong address or the ISR ran far enough to
   advance PC past the entry. The trace `T0..TF` disambiguates.
7. **`T0..TF`** is the run-up: T0 is the fetch that faulted, T1 the one before,
   etc. — gives 16 instructions of execution history.
8. **Registers / flags / stack** are the architectural state at fault time
   (already updated for any retired instructions; not for the faulting one).

## V64 immediate encoding

V64 opcodes are 3 words / 12 bytes wide:

```
PC+0 : opcode word
PC+4 : bits [31:0]  of the 64-bit immediate (lo32)  → fetched into w_var1, dumped as V1
PC+8 : bits [63:32] of the 64-bit immediate (hi32)  → DRAM-read at dump time, dumped as V1H
```

The DDR2 controller returns full 64-bit doublewords; the right 32-bit half is
selected with `r_PC[2]` (PC and PC+8 share the same bit-2 parity).

## DRAM read mechanics in the dump

Phases that need DRAM data (`S0..S3`, `V1H`, `OPCM`) issue a read in the PREP
sub-state, transition to `STACK_FETCH` (sub-state `3'b011`), wait for
`w_mem_ready`, latch `w_mem_read_data` into `r_hcf_stack_data`, and return to
PREP — which now sees `r_hcf_stack_loaded=1` and emits the line. The `loaded`
flag is cleared in `DONE_WAIT` so the next phase fetches fresh data.

Reads past `STACK_TOP` (when the stack is empty) are skipped and replaced with
`FFFF_FFFF_FFFF_FFFF` to avoid out-of-bounds DDR2 access.

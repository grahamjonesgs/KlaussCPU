# ISA v3 — what was implemented (M13)

Implements `klausscpu-llvm/llvm/lib/Target/KlaussCPU/ISA_V3_PROPOSAL.md` items
A (all), B (resolved in MEM), C, D1 and D2 across the RTL, the `klausscc` emulator/assembler and the
LLVM fork. Everything is backwards compatible: every v2 binary runs unchanged
(board-measured cycle-identical on the v3 core). Branch `isa-v3` in all repos.

## Encodings (all additions were previously-trapping combinations)

| Item | Class | Distinguished by | Payload | Semantics |
|---|---|---|---|---|
| A5 short ALU-imm / SETR | 2 | `LEN=01`, `rs2=0` | imm8 `[19:12]`, `SGN [20]` = sext/zext | as the 2-word form |
| A3 short CMPRV | 3 | `LEN=01`, `SGN=1`, `B=0`, `rd=rs2=0` | simm8 `[19:12]` | flags ← rs1 − sext(imm8) |
| A4 short load/store | 6/7 | `LEN=01`, `MODE=01`, `rs2=0` | simm8 `[19:12]` | EA = rs1 + (sext(imm8) << SIZE) |
| A1 short branch/call | 8 | `LEN=01`, `RIND=0`, `REL=1` | simm18 `[17:0]` | target = PC + 4·disp; CALL pushes PC+4 |
| D1 `LDIDX32_S` | 6 | `SIZE=10`, `SGN=1`, `MODE=01/10/11` | — | already in the v2 RTL; now emitted |
| D2 W compare | 3 | `W [19]` on the reg form and the 2-word form (not the short form) | — | 32-bit compare of the low halves (flags and boolean results) |
| C `ENTER N` | 9 | `LEN=01`, op 8 | N `[21:0]` (dwords) | push R15; R15 = SP; SP −= 8N |
| C `LEAVE` | 9 | `LEN=01`, op 9, `[21:0]=0` | — | SP = R15; pop R15 |
| C `LEAVERET` | 9 | `LEN=01`, op 10, `[21:0]=0` | — | LEAVE; RET (R15=[R15], PC=[R15+8], SP=R15+16) |
| B fused compare-and-branch | **D** (was reserved) | `LEN=01`, PRED `[25:23]` ≤ 4 | INV `[22]`, IMM `[21]`, simm13 `[20:8]`, rs1 `[7:4]`, rs2/simm4 `[3:0]` | if (rs1 PRED rhs) ^ INV: PC += 4·disp; flags untouched |

Short CMPRV always sign-extends (SGN=1 is its discriminator), so a compare
against 128..255 keeps the 2-word form. The class-2 `SGN` bit was already
honoured by the RTL for AND/OR/XOR (proposal §3.2 needs no RTL change).

## RTL (`pipeline_core.sv`)

- **Immediates are formed at the IF→ID latch** (`f_short_imm` → `id_var1`),
  so every EX consumer (ex_b imm leg, EA adder, branch-target adder, MOV,
  ADDSP adder for ENTER's −(8+8N)) is unchanged. That path had 5.6 ns slack.
- **ex_b operand select predecoded at the latch** (`id_b_one/imm/sext`),
  replacing a `dec.len`/`dec.uop` test on the id_op→ex_b cone.
- **D2 needs no EX change:** both operands are shifted left 32 at the dispatch
  mux (`f_wsh`). A 64-bit compare of `{a[31:0],0}` vs `{b[31:0],0}` gives
  exactly the 32-bit Z/S/C/V and the 32-bit signed/unsigned order.
- **ENTER** = PUSH-shaped store + `exo_result = SP−8` (forwards from MEM like
  any ALU result) + ADDSP-shaped SP update. **LEAVE** = POP-shaped load from
  R15 (load-interlocked) with SP = R15+8. **LEAVERET** also takes the return
  address: from the line lookahead (`m_rdata_next`) when [R15+8] is in the
  same 16 B line, else via the existing MEMGET32 second-read path.
- **B resolves in MEM, not EX.** The fused op is a boolean compare in EX
  (the existing SETcc path → `mem_result[0]`), its target `PC + 4·disp`
  rides in the unused EA register, and a taken one redirects when it leaves
  MEM, squashing the two younger instructions (EX, ID) like the SMC squash
  does. No new logic on the EX compare→redirect path (+0.41 ns slack, a
  64-bit compare there would not close). Even resolved a stage later it beats
  the old pair: CMP→JMPcc paid a flag-interlock bubble. The imm4 is formed
  at the latch into `id_var1`, the displacement into `id_var2`.
- M13 margin fixes that came first: `id_pc_next` and the SMC squash's
  last-word dword are latched at IF→ID (`id_pc_nx`, `id_last_dw`), and the
  hazard predecode reads the raw decode (no illegal-op legality cone).
- Sim assertions pin every predecoded flag to the full decode for legal ops.

## LLVM (`klausscpu-llvm`)

- A3/A4/A5: `*_SH` isCodeGenOnly twins; `KlaussCPUMCCompress` swaps the
  opcode at MC emission (AsmPrinter and AsmParser, RVC-style) when a literal
  immediate fits. `.s` output reassembles byte-identically.
- A1: branches to symbols start as 1-word `JMPcc_SH` with a PC-relative
  `FK_KlaussCPU_PCREL18` fixup; the asm backend relaxes them to the 2-word
  PC-relative form when out of range or not resolvable in-section, so no new
  relocation exists (no lld / Zephyr LLEXT change). Calls stay 2-word.
- D1: `SEXTLOAD i32` legal → `LDIDX32_S`.
- D2: `BR_CC` and `SELECT_CC` use `CMPRRW`/`CMPRVW` when both operands are
  32-bit extended and an extension instruction disappears (sext pair: any
  condition; zext pair: EQ/NE/unsigned only).
- C: `ENTER`/`LEAVE`/`LEAVERET` from `KlaussCPUFrameLowering`.
- B: post-RA pre-emit pass `KlaussCPUFuseCmpBr` fuses an adjacent
  CMPRR / CMPRV(#-8..7) + JMPcc (EQ/NE/Z/NZ, signed and unsigned LT/GE/LE/GT,
  C/NC) when the flags are dead afterwards and the function is < 16 KB at
  worst-case sizes, so the `FK_KlaussCPU_PCREL13` fixup always resolves.
- A/B switches (`-mllvm`): `-klausscpu-short-forms`, `-klausscpu-short-branches`,
  `-klausscpu-w32-compare`, `-klausscpu-enter-leave`, `-klausscpu-leaveret`,
  `-klausscpu-fuse-cmp-br`.

## klausscc

Emulator implements every new form (`short_imm()` feeds the shared class
handlers; W compare; ENTER/LEAVE/LEAVERET) with unit tests. The assembler has
`.S` short mnemonics (literal immediates), `LDIDX32_S`, `CMPRRW/CMPRVW`,
`ENTER/LEAVE/LEAVERET`. Short and fused branches are LLVM-only (they need
PC-relative label arithmetic the assembler doesn't do).

## Verification

- Directed RTL programs vs the emulator, trace **and flags** identical:
  every short form (incl. negative CMPRV, scaled load/store, backward loop,
  short CALL returning to PC+4), W compares, ENTER/LEAVE incl. back-to-back
  R15 use and nesting.
- M5a golden traces (hello/bst/expr/test_64bit/queens) and M5c (WAIT, SMC,
  MASKRACE ×2, IRQ storms) on binaries built with each new feature.
- `run_m5c.sh` storm compare now ignores the merged-dword data of SUB-WORD
  store annotations (it includes neighbouring bytes, which the handler's
  pushes perturb in uninitialised stack); address/BE and all 64-bit store
  data, registers, SP and PC are still compared.
- Board self-checks (bst, expr, test_64bit, queens, test_printf, test_fp,
  crypto, test_asm, test_switch) and perf_baseline/dhrystone.

## Results

(filled in below by the M13 report)

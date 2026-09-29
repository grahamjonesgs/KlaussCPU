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

## Results (2026-09-29, measured)

### Board cycles — one CPU (v3, all features), only the binary changes

perf_baseline kernels (cycles) and Dhrystone. Each column adds features to
the compiler; every self-check (bst, expr, test_64bit, queens, test_printf,
test_fp, crypto, test_asm, test_switch) passes in every column. v2 binaries
run cycle-identical on the v3 core (e.g. calls_fib 55,024,161 either way).

| kernel | v2 | +A (short forms) | +D2 | +ENTER/LEAVE | +LEAVERET | +B (fused) | total |
|---|---|---|---|---|---|---|---|
| alu | 12,000,401 | 12,000,348 | 12,000,390 | 12,000,354 | 12,000,277 | 12,000,274 | 0.0% |
| mem_stream | 7,500,166 | 7,466,651 | 7,467,016 | 7,466,875 | 7,467,124 | 7,142,998 | −4.8% |
| ptr_chase | 8,809,685 | 8,809,689 | 8,786,642 | 8,775,782 | 8,775,941 | 8,790,271 | −0.2% |
| branchy | 22,188,400 | 20,781,911 | 19,781,911 | 19,781,906 | 19,781,901 | 19,188,306 | **−13.5%** |
| calls_fib | 55,024,161 | 49,367,535 | 44,889,530 | 40,775,676 | 36,661,855 | 36,661,840 | **−33.4%** |
| muldiv | 1,508,381 | 1,308,363 | 1,308,365 | 1,308,353 | 1,308,323 | 1,308,352 | **−13.3%** |
| Dhrystones/s | 57,536 | 60,240 | 60,495 | 61,086 | 61,689 | 63,572 | **+10.5%** |

(The "+B" column is on the EN_FBR=1 build; the others on the build without
B — the non-B columns are cycle-identical on both.) Note: the SoC branch
counter (`perf_br`) counts EX-stage JMPs only, so fused branches don't show
in perf_baseline's branch-rate column.

### Dynamic instructions / fetched words / code size (emulator, vs v2)

| program | instructions | fetched words | .text |
|---|---|---|---|
| hello | −6.4% | −38.6% | −36.4% |
| bst | −14.3% | −42.6% | −34.9% |
| expr | −13.9% | −42.4% | −34.6% |
| test_64bit | −19.1% | −43.9% | −33.4% |
| queens | −16.1% | −38.5% | −35.7% |
| crypto | −8.3% | −32.9% | −32.3% |
| test_printf | −11.0% | −40.4% | −34.5% |
| test_fp | −17.1% | −42.1% | −30.4% |

Short forms alone (A) cut fetched words 28–35% — the proposal predicted 30%.

### Timing (tier-1 Performance_Explore, no tier-2 needed in any build)

| build | WNS | WHS | build time | limiting paths |
|---|---|---|---|---|
| master before M13 (snoop + hazard fixes) | +0.235 | +0.024 | 10m00s | CPU id_op→pc +0.236 |
| + margin fixes + A1/A3/A4/A5 | +0.086 | +0.018 | 12m07s | SoC st_reg→cache BRAM; CPU ≥ +0.186 |
| + D2 | +0.192 | +0.027 | 17m00s | — |
| + ENTER/LEAVE/LEAVERET | +0.163 | +0.013 | 14m30s | SoC st_reg→cache/IFB; CPU-internal ≥ +0.248 |
| + B (EN_FBR=1) | +0.033 | +0.019 | 12m21s | CPU id_op→pc, wb_value→pc (B's MEM pc source) |

Without B the CPU is no longer the limiter (the old id_op→pc and ex_b
families are gone from the ≤0.4 ns list). B costs ~0.2 ns of pc-path margin
for its +3% Dhrystone / −3% branchy; `EN_FBR` drops it.

FreeRTOS demo (preemption, 1 kHz tick, yields) PASSes on the B build.

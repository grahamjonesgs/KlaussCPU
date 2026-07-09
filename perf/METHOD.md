# Perf & correctness baseline — method (reproducible A/B reference)

Captured **2026-07-09** against **master `a19a853`** (ISA v2 phase-2). This is the
apples-to-apples reference for every phase in `PIPELINE_MASTER_PLAN.md`. Re-capture
this exact way before trusting any A/B delta (see the stale-ELF trap below).

## What was measured
- **RTL:** master `a19a853`, flashed to QSPI on 2026-07-08. The board reproduces
  `~/.klausscpu_scratch/perf_isav2.log` (2026-07-08) **exactly** (instruction
  counts byte-identical, CPI identical to 3 dp) ⇒ the flashed CPU *is* `a19a853`.
  No rebuild was needed.
- **ELFs:** `/media/psf/src/klausscpu-runtime/baremetal/*.elf`, built 2026-07-07
  (ISA v2). This is the current ELF set — pin it for A/B.
- **Board:** Nexys A7 on `/dev/ttyUSB1`.

## Tooling — IMPORTANT
- **Use** `/home/graham/Documents/src/klausscc/target/release/klausscc`
  (v2-capable, x86-64 Linux, rebuilt 2026-07-09). Does both send/monitor and
  `--emulate`.
- **Do NOT use** `/home/graham/klausscc-linux-target/release/klausscc` — it is the
  stale **v1** build (2026-07-03); its `--emulate` rejects every v2 ELF with
  `InvalidOpcode(0x64000000)`. (Its send-to-board path happens to still work
  because that just byte-streams the ELF, but don't rely on it.)
- The `/media/psf/src/rust/klausscc/.../klausscc` binary is a **Mac** build —
  `Exec format error` on this Linux VM.

## Reproduce
Let `K=/home/graham/Documents/src/klausscc/target/release/klausscc` and
`E=/media/psf/src/klausscpu-runtime/baremetal`.

**Performance (board):**
```
$K --input $E/perf_baseline.elf --serial /dev/ttyUSB1 --monitor   # 6 kernels, CSV lines
$K --input $E/dhrystone.elf     --serial /dev/ttyUSB1 --monitor   # DMIPS/MHz
```
CSV line = `CSV,<kernel>,<time_ms>,<cycles>,<instr>,<CPI×1000>,<miss_bp>,<br_rate>`.

**Functional (board) — the correctness oracle is each ELF's own self-check:**
```
$K --input $E/<prog>.elf --serial /dev/ttyUSB1 --monitor          # expect "N pass, 0 fail"
```

**Golden model (host, no board) — bit-identical reference:**
```
$K --input $E/<prog>.elf --emulate                                # UART + instruction count
```
Board UART == emulator UART is proven byte-identical for queens; every regression
self-check agrees (see results). Use `--emulate --trace` for a per-instruction
golden trace when debugging a correctness diff (reframe #5 in the plan: debug in
the golden model / sim, not on 30-min silicon builds).

## Results — the numbers to beat / preserve

### CPI (perf_baseline, board)
| kernel | CPI | miss% | fastpath% |
|---|---|---|---|
| alu | 4.125 | 0.00 | 75.0 |
| branchy | 4.121 | 0.00 | 56.6 |
| muldiv | 4.744 | 0.00 | 66.7 |
| calls_fib | 6.394 | 0.00 | 21.2 |
| ptr_chase | 6.808 | 14.36 | 38.5 |
| mem_stream | 8.424 | 9.09 | 60.0 |

Machine-readable: `perf/baseline_board.csv`.

### Dhrystone (board)
608.009 instr/Dhry · 28128 Dhrystones/s · **16.009 DMIPS · 0.160 DMIPS/MHz** · self-check PASS.

### Functional regressions — board self-check == emulator (all PASS)
| program | board | emulator | emu instr count |
|---|---|---|---|
| hello | prints OK, halt | Halt | 49,878 |
| queens | 11 pass, 0 fail | 11 pass, 0 fail | 9,438,409 |
| bst | 11 pass, 0 fail | 11 pass, 0 fail | 85,191 |
| expr | 35 pass, 0 fail | 35 pass, 0 fail | 112,563 |
| crypto | 17 pass, 0 fail | 17 pass, 0 fail | 107,725 |
| test_64bit | 36 pass, 0 fail | 36 pass, 0 fail | 133,071 |

Captured UART is in `perf/golden/<prog>.{board,emu}.txt`.

## The golden rule (every later phase)
UART output of every regression must stay **bit-identical** to these goldens
(against the emulator oracle) — caching/flags/pipelining are performance
transforms; any output diff is a correctness bug. **Caveat for the flag-day flag
change:** it will recompile the ELFs (new codegen), so instruction *counts* will
change legitimately; the invariant that must hold is the **program UART text**
(the `N pass, 0 fail` self-checks and printed results), re-diffed against a
freshly re-emulated golden of the *new* ELFs. Re-capture this whole baseline with
the new ELF set at that point, and record the new SHA.

## Stale-ELF trap
Same instruction count + different memory layout ⇒ different cache-miss pattern ⇒
different CPI on memory-sensitive kernels (ptr_chase/mem_stream/muldiv/calls_fib);
compute kernels (alu/branchy, ~0 miss) are unchanged. Only ever compare a build's
CPI against a baseline captured with the **same ELF set**.

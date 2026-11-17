# tools/vsim — pipeline_core / SoC simulation without Vivado

These tools are for open-source simulation (Verilator ≥ 5.020). They
complement the xsim flows in `perf/m5a/`; they don't replace them. xsim plus
the klausscc emulator goldens is still the reference, and none of this says
anything about timing.

| File | What it is |
|---|---|
| `tb_vsim.sv` | Lean core testbench. It uses `tb_pipeline_isa`'s memory / MMIO / timer-IRQ model and IRQ handler, without the debugger driver, and writes a final-state file plus an optional retire trace. |
| `kasm.py` | Minimal encoder for the fields `f_decode_raw` reads, with labels, plus `write_mem` (core TB) and `write_boot_mem` (SoC boot image). |
| `tests.py` | Directed tests: `@test` functions returning `(Asm, expected, plusargs)`. |
| `randprog.py` | Random terminating programs, used for differential runs. |
| `run.py` | Build, test and diff driver. |

```
pip-free; needs: verilator, python3
export VSIM_WORK=/tmp/vsim                 # build/run products (default /tmp/vsim)
python3 tools/vsim/run.py test             # all directed tests on the working tree
python3 tools/vsim/run.py test fbr_w --rev HEAD~3     # one test on an older commit
python3 tools/vsim/run.py diff --base master --seeds 40 --lats 1 3 7
python3 tools/vsim/run.py diff --base master --seeds 30 --lats 2 5 --irq
python3 tools/vsim/run.py socdiff --base master --seeds 30   # full SoC (tb_soc.sv)
python3 tools/vsim/run.py socsd                    # native SD MMIO through the SoC bus
VSIM_DEBUG_EN=1 python3 tools/vsim/run.py test        # elaborate DEBUG_EN=1 (wk_debug checks the halt log only here)
VSIM_TLB_WALK=0 python3 tools/vsim/run.py test 'wk_*' 'tlb_*'   # TLB_EN=1, TLB_WALK=0 (test names take globs)
python3 tools/vsim/run.py socwalk                  # the hardware TLB walker through the real D-cache (tb_soc)
python3 tools/vsim/run.py dcbzdiff --seeds 200     # DCBZ programs vs 4-store references (core TB)
python3 tools/vsim/run.py socdcbz --seeds 20       # DCBZ through the SoC: counters, DDR traffic, SMC, random
python3 tools/vsim/run.py tecost --seeds 60          # cycle cost of TE = 1 on randprog (MMU_PLAN §11.9)
```

- **`diff`** runs the same random programs on a base revision and on the
  working tree. Without `--irq`, the retire traces must be identical. With
  `--irq`, IRQ timing can legitimately shift, so it compares the main
  program's final registers and memory instead. It reports total cycles for a
  quick A/B.
- **`dcbzdiff`** / **`socdcbz`** (HW_PERF_PLAN.md §1): randprog programs
  with DCBZ groups injected, against the same programs with each DCBZ
  replaced by four zero stores; the final registers (and, in the core TB,
  the data window; on the SoC a checksum of it) must match. `socdcbz` also
  checks the cache counters, that a DCBZ miss reads nothing from DDR
  (tb_soc prints `DDR reads= writes=` at HALTED), the trace record, and a
  zeroed-then-patched code line.
- **`tecost`** (MMU_PLAN §11.9): each randprog seed behind a prologue that
  maps every page it touches with global identity entries and writes
  STATUS with TE = 1 or 0 (the only difference), in supervisor and user
  mode; prints the total cycles and the TE = 1 cost (`--plus +HIT=70` for
  the hit-pipe model, `--rev` for another revision).
- **`socwalk`** (MMU_PLAN §11.8): directed SoC programs for the walker
  through `mem_read_write`: PTE / directory lines dirty in the D-cache when
  walked, walk reads missing with a dirty victim, a walk read meeting the
  eviction writeback of its own line (D and T), and `CPU_FEATURES[25]` with
  `CORE1_TLB_WALK = 0`. The page tables are stored by the program (the boot
  image holds 256 KiB) and written back with CACHE_CTRL INVALIDATE. A probe
  appended to the sim copy of tb_soc prints one `WKEV` line per walker read
  (what it found in the cache), which the runner checks.
- **`socdiff`** builds `sim_1/new/tb_soc.sv` (real cache, bus splitter, MMIO,
  HCF, behavioural DDR) and boots each program through the boot-ROM path. It
  compares retire traces and reports PIPE_RUN cycles. That model reproduces
  the board's §2.2 microbenchmark costs to within ~0.2 cycles, so it is a
  usable A/B for core changes. `run.run_soc(exe, image_or_mem_path, name)`
  also boots any `netboot.mem`-format image (e.g. the real netboot).

## MMIO in tb_vsim; frames popped from MMIO (HW_PERF_PLAN.md §8)

- tb_vsim's MMIO side follows the SoC's protocol: one answer per DV pulse.
  Once the core has seen an MMIO ready, a DV it keeps up at an MMIO address
  in the next cycle (the same address or not) holds ready high and is never
  answered — the SoC's MMIO ready is the strobe delayed, the Ethernet bridge
  waits for DV low — so such a core deadlocks here as on the board.  The
  `.out` file's `mmio_hold=` counts those cycles; `run.py test` fails a test
  with `mmio_hold` ≠ 0.  `+MMIOPULSE`: the old model (a new request on any
  cycle).
- `0xF0FD_0000..0xF0FD_00FF`: an MMIO scratch RAM (32 dwords, byte enables)
  that reads back what was written — `tests.py`'s `mmio_frame` /
  `mmio_frame_irq` pop RET / LEAVERET frames from it and run a spanning
  MEMGET32 there.
- **`socframe`** (`--rev`): the same through tb_soc (INT_VEC0 / 1 as the
  frame; a spanning MEMGET32 on INT_VEC and on the Ethernet bridge).
- **`socorphan`** (`--rev`, `--k`, `--modes`, `-v`): CPU_RESETN pressed
  while a DMA master holds the DDR grant — with a read accepted below and
  the D-cache waiting (tb_soc `+RSTORPH=<k>`), or at any moment of a tenure
  (`+RSTORPH_ANY`); the framebuffer holds a pattern.  No wrong answer, no
  grant held at the end, no D-cache request waiting > 2,000 cycles; with
  the L2 also in runtime bypass (§8 item 3d and its review).

## The SoC FSM, precise EI, MMU permissions (HW_PERF_PLAN.md §9)

```
python3 tools/vsim/run.py socreset [case-regex ...] [--rev REV] [-v]
python3 tools/vsim/run.py socbreak [case-regex ...] [--rev REV] [-v]
python3 tools/vsim/run.py socsload [sd|blit] [--rev REV] [-v]
python3 tools/vsim/run.py socei [--srcs mask vec0 ...] [--k 0 1 2] [--j 0 4] [--full] [--rev REV]
python3 tools/vsim/run.py permfuzz --seeds 40 --fetch --remap [--walk] [--irq] [--soc]
python3 tools/vsim/run.py permfuzz --selfcheck
python3 tools/vsim/fablemut.py [-j 6]           # the §9 mutations (each must be killed)
```

- **`socreset` / `socbreak` / `socsload`** (`socfsm.py`) run tb_soc with
  a probe appended (a separate build, `soc_*_fr`) that drives CPU_RESETN
  and the UART RX line (`uart_rx_tb` in tb_soc).  The probe checks that
  every boot-copy ack the FSM takes follows a D-cache write latch of
  `st.mem_addr` (`BAD-ACK`: a stale ready taken as the ack), detects hangs,
  compares the FSM's retire count with the pipeline's, and counts SD DMA /
  blitter grants while the UART loader runs.
  - `socreset`: CPU_RESETN pulses of 1-200 cycles during a read miss,
    short-release-long shapes, release bounce in the power-up copy, two
    glitches with a boot write miss in the shadow, a press during the
    crash dump's DDR reads.  The program corrupts DDR dword 0 and reboots:
    it must boot with the ROM's dword 0.
  - `socbreak`: a UART break swept over 48 offsets through the boot copy
    and 40 over the crash dump's first DDR read (two passes: the first
    finds that read's cycle); breaks + 'A' while a program runs; a break +
    'A' inside a .kbt stream.  Coverage: some pulse must land on a boot ack
    and on an HCF read ack.
  - `socsload`: break + S + .kbt while an SD DMA read / back-to-back
    blitter FILLs run, with the S placed on a tenure (two passes: the
    grant rises of a run without the S).  No SD / blitter grant after a
    200-cycle grace, none held at the end, the image intact; a blitter
    grant held at the S must be released by the orphaned-grant guard.
- **`socei`** (`socei.py`): `[enabling store] nop*k EI nop*j DI` for
  INT_MASK, INT_VEC0-3, INT2_EN and VGA_CTRL.VSYNC_IRQ_EN, warm code, 4-8
  alignments; every cell must be `1@+4` (one entry, frame PC = EI + 4).
  Plus a loop of `{INT_MASK = 0; nop; INT_MASK = 1; nop*k; EI; nop*j; DI}`
  that must take interrupts.  `tests.py ei_enable_store` is the core-TB
  version.
- **`permfuzz`** (`permfuzz.py`, from the review's MMU pass): random
  user-mode programs under TE = 1 whose data window straddles two pages
  with random permissions, kernel / MMIO probes, fetch through pages with
  and without X, remaps with three flush methods; the reference is the
  same program with each statically faulting instruction replaced by
  NOPs, run with every page RWX.  `--soc`: through tb_soc.
  `--selfcheck`: with `PF_NOFLUSH=1` (the remap's flush removed from the
  test program) most runs must differ.
- `--rev REV` runs another revision's RTL under the working tree's tb_soc
  (as `socorphan`); against `d612472` every one of these fails.
- **tb_vsim's INT_MASK** is written on the store's DV, as the SoC does
  (`w_pip_mask_wr`), not after the store completed.

## The L2 (HW_PERF_PLAN.md §6)

- **tb_soc's DDR** is the behavioural MIG (`sim_1/new/mig_model.sv`) behind
  the real `ddr2_control` (`VSIM_L2=0`, `CORE1_L2_EN` = 0) or `l2_cache`
  (default).  Plusargs (also through `VSIM_SOC_PLUS="..."` for every run, or
  `--plus` where a command has it): `+MIGLAT=<n>` (default 36: ~44 cycles
  from a D-cache miss to its critical dword, as on the board, for a read to
  a closed bank), `+MIGJIT`, `+MIGRDY` (% of cycles app_rdy drops),
  `+MIGREF=<period>` (refresh, default 780; 0 = off), `+MIGRFC`,
  `+MIGFREE=1` (the pre-review-1 model).  Since review 1 it models DDR2-400
  bank / row / turnaround / write-recovery / refresh timing, ignores the
  rank bit PA[27], and `$fatal`s when write data follows its command by
  more than 2 cycles (UG586); see HW_PERF_PLAN §6 review 1.  Older revisions' tb_soc (a behavioural `ddr2_control`, 8
  cycles a line) are built as they were; `socdiff --base <rev> --base-tb-wt`
  runs an older revision's RTL under the current tb_soc instead (same DDR
  model: cycle counts comparable).  `socdiff --base wt --base-env VSIM_L2=0`
  compares the L2 off / on in the working tree.  `VSIM_L2_WAYS=8`;
  `VSIM_L2_SPEC=0` / `VSIM_L2_XTRA=<n>` (measurement only: 4-cycle hits,
  n more cycles a hit).
- **Memory-consistency checker** (every tb_soc run): a reference memory at
  the arbiter's port; a wrong read answer or early dword prints `FATAL
  consistency` and the command fails.
- **`socl2`**: L2_CTRL semantics (BUSY at once, writes ignored while BUSY,
  stores during the walk), blitter ordering against D-cache writebacks
  (L2 on, L2 off, reversed), the counters.  **`socbench`**: 96–112 KiB
  footprints, L2 off vs on.  **`socvga`**: VGA scanout under CPU miss /
  writeback traffic and WBINV walks (starved lines must be 0; `+VGA_V0`
  moves the raster to the next vblank).
- **`tb_l2`** (`sim_1/new/tb_l2.sv`): the L2 alone with the behavioural MIG,
  all masters, a reference memory, directed tests (`+TEST=hit`, `sector`,
  `evict`, `wbhit`, `plru`, `dma`, `wbinv`, `toggle`, `alias`, `wbinvq`,
  `wkturn`, `rtdrain`, `vgastream`) and `+TEST=random +SEED=<n> +N=<n>`;
  built with `+define+L2_WAYS=<n>`; masters hold DV one cycle after ready
  (`+DVTAIL=0`: not).  **`l2mut.py`**: the coherence mutations on it;
  `l2mut.py --sweep`: every test at 2 / 4 / 8 ways under four MIG timings.
- **`l2_regress.sh [section ...]`**: the whole L2 regression (tb_l2 sweep,
  mutations, the SoC tests at `MIGLAT` 36 and 8, L2 off / 8 ways / other
  timings, the `L2_EN` = 0 equivalence, the core regression).
- tb_soc with the L2 also counts drains started in VGA's / DMA's window
  with no request (nor the WBINV walk) waiting on the buffer (`L2 rt
  drains bad=`, the need spelled out as in tb_l2; a gated run fails on any,
  and an L2-on gated run without that line fails too); a gated run whose
  consistency checker checked nothing fails.  `socvga --blit` adds a back-to-back blitter FILL.

## Sim-copy patches (Verilator 5.020 limits)

These are applied only to copies under `$VSIM_WORK`; the RTL is unchanged.

- `pipeline_core.sv`: drop the `dbg_i = '0` port default, and turn the
  `for`-loop NBAs to `ic_vhi`/`ic_vlo`/`rf` into whole-array assignments.
- `uart_send_msg.sv`: the mixed blocking/NBA `i_msg_count` reset becomes
  blocking.
- `KlaussCPU.sv` of revisions before HW_PERF_PLAN §9: the UART loader's
  NBA `o_ram_write_value <= 32'h0` becomes blocking (Verilator dropped the
  loader's blocking nibble writes next to it, so every S load failed its
  checksum).  Since §9 the loader packs the nibbles in a local variable
  and needs no patch.
- `tb_soc.sv`: print PIPE_RUN cycles at HALTED; add an `ODDR` behavioural
  stub; the §9 probe (`socfsm.py`) for socreset / socbreak / socsload.
  (`+IDLEMAX=<n>`: tb_soc's no-retire deadlock limit, default 600,000.)

## Adding a test

```python
@test
def my_test():
    a = Asm()
    a.setr(1, 5); a.addi_s(1, 1, 2); a.halt()
    return a, {'end': 'park', 'kind': '0', 'r1': 7}, ['+LAT=3']
```

Expected keys:

- `rN` (int), `m<8-hex-addr>` (int, a dword in 0x10000..0x10800, or the IRQ
  counter `m00180000`);
- `kind` / `pc` / `end` (park);
- any `key=value` the TB prints (`flags`, `ic_miss`, `bus_wr`, `posted`, …);
- the value `'nonzero'`.

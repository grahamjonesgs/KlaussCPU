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
VSIM_DEBUG_EN=1 python3 tools/vsim/run.py test        # elaborate DEBUG_EN=1
```

- **`diff`** runs the same random programs on a base revision and on the
  working tree. Without `--irq`, the retire traces must be identical. With
  `--irq`, IRQ timing can legitimately shift, so it compares the main
  program's final registers and memory instead. It reports total cycles for a
  quick A/B.
- **`socdiff`** builds `sim_1/new/tb_soc.sv` (real cache, bus splitter, MMIO,
  HCF, behavioural DDR) and boots each program through the boot-ROM path. It
  compares retire traces and reports PIPE_RUN cycles. That model reproduces
  the board's §2.2 microbenchmark costs to within ~0.2 cycles, so it is a
  usable A/B for core changes. `run.run_soc(exe, image_or_mem_path, name)`
  also boots any `netboot.mem`-format image (e.g. the real netboot).

## Sim-copy patches (Verilator 5.020 limits)

These are applied only to copies under `$VSIM_WORK`; the RTL is unchanged.

- `pipeline_core.sv`: drop the `dbg_i = '0` port default, and turn the
  `for`-loop NBAs to `ic_vhi`/`ic_vlo`/`rf` into whole-array assignments.
- `uart_send_msg.sv`: the mixed blocking/NBA `i_msg_count` reset becomes
  blocking.
- `tb_soc.sv`: print PIPE_RUN cycles at HALTED; add an `ODDR` behavioural
  stub.

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

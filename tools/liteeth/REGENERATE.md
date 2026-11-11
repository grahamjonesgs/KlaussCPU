# Regenerating `liteeth_core.v`

The LiteEth MAC core that lives at
[KlaussCPU.srcs/sources_1/new/liteeth_core.v](../../KlaussCPU.srcs/sources_1/new/liteeth_core.v)
is **generated** from
[liteeth_nexys_a7.yml](liteeth_nexys_a7.yml) using the upstream
[LiteEth](https://github.com/enjoy-digital/liteeth) tooling. You only need
to regenerate it if you change the YAML config (e.g. add more slots, change
the bus standard, etc.) — for normal development you just edit
`liteeth_core.v` indirectly via the YAML, never by hand. The project-level
edits to the generated file are applied by
[patch_liteeth_core.py](patch_liteeth_core.py) (see "What we changed").

The generated file is checked in so the Vivado project builds without
requiring LiteEth/LiteX on the user's machine.

## One-time toolchain setup

LiteX uses bytecode introspection to auto-derive CSR names. **The released
PyPI wheels of `migen` / `litex` are broken on Python 3.11+** with
`ValueError: Cannot extract CSR name from code`. Install from the upstream
git mainlines instead.

### macOS (Homebrew)

```bash
brew install python@3.12

# Build the venv with explicit 3.12
$(brew --prefix python@3.12)/bin/python3.12 -m venv ~/.venvs/litex
source ~/.venvs/litex/bin/activate

# Install from git, not pip wheels
pip install --upgrade pip
pip install git+https://github.com/m-labs/migen.git
pip install git+https://github.com/enjoy-digital/litex.git
pip install git+https://github.com/litex-hub/litex-boards.git
pip install git+https://github.com/enjoy-digital/liteiclink.git
pip install git+https://github.com/enjoy-digital/liteeth.git
```

### Pinned versions (used for the checked-in core)

The checked-in `liteeth_core.v` (2026-11-11, `nrxslots: 8`) was generated
with these commits; regenerating the previous YAML (`nrxslots` absent, i.e.
2) with them reproduces the old checked-in file except the date stamp, the
order of a few `[BB:...]` lines in the hierarchy comment, and the
`dont_touch` edit. Python 3.14 (Ubuntu 26.04) works.

| Package | Repository | Commit |
|---|---|---|
| migen | https://github.com/m-labs/migen | `e19524c963a8342952840983047557707fbe0b6a` (2026-01-05) |
| litex | https://github.com/enjoy-digital/litex | `30dbdec17c6af430ddf291157d57d9149ade06b3` (2026-05-12) |
| liteiclink | https://github.com/enjoy-digital/liteiclink | `878b9efcf08ab757432f00b09c3d38b1e60407fe` (2026-05-11) |
| liteeth | https://github.com/enjoy-digital/liteeth | `cd479a4044ce1ef8dda3e5cd72f5899a0fe8d0aa` (2026-05-11) |

(`pip list` reports them as 2025.12 / migen 0.9.2.) litex-boards is not
needed by `liteeth.gen`. To install exactly these:

```bash
python3 -m venv ~/.venvs/litex
~/.venvs/litex/bin/pip install --upgrade pip
~/.venvs/litex/bin/pip install \
    git+https://github.com/m-labs/migen.git@e19524c963a8342952840983047557707fbe0b6a \
    git+https://github.com/enjoy-digital/litex.git@30dbdec17c6af430ddf291157d57d9149ade06b3 \
    git+https://github.com/enjoy-digital/liteiclink.git@878b9efcf08ab757432f00b09c3d38b1e60407fe \
    git+https://github.com/enjoy-digital/liteeth.git@cd479a4044ce1ef8dda3e5cd72f5899a0fe8d0aa
```

If `PYTHONPATH` points at another Python tree (e.g. ROS), run the
generator with `env -u PYTHONPATH`.

### Linux (Debian/Ubuntu, PEP 668-aware)

```bash
sudo apt install python3-venv python3-full

python3 -m venv ~/.venvs/litex
source ~/.venvs/litex/bin/activate

pip install --upgrade pip
pip install git+https://github.com/m-labs/migen.git
pip install git+https://github.com/enjoy-digital/litex.git
pip install git+https://github.com/litex-hub/litex-boards.git
pip install git+https://github.com/enjoy-digital/liteiclink.git
pip install git+https://github.com/enjoy-digital/liteeth.git
```

## Regenerating the core

With the venv active and from this directory:

```bash
source ~/.venvs/litex/bin/activate
cd tools/liteeth/

python -m liteeth.gen liteeth_nexys_a7.yml \
    --output-dir generated \
    --soc-json   generated/liteeth_csrs.json \
    --no-compile
```

Outputs land under `tools/liteeth/generated/`:
- `generated/gateware/liteeth_core.v` — the standalone Verilog core
- `generated/gateware/liteeth_core.xdc` — LiteEth's recommended constraints (we
  rewrote the relevant pieces into the project XDC; see "What we changed"
  below)
- `generated/liteeth_csrs.json` — CSR map (drives the MMIO layout in
  [MMIO_MAP.md](../../MMIO_MAP.md))
- `generated/csr.csv` — same map in CSV form

`--no-compile` keeps the generator from trying to invoke Vivado on its own
— we only want the Verilog and the CSR map.

## Copying the result into the project

Never copy the generated file as-is: apply the project patch on the way in.

```bash
python3 patch_liteeth_core.py generated/gateware/liteeth_core.v \
    ../../KlaussCPU.srcs/sources_1/new/liteeth_core.v
```

The script fails (and writes nothing) if any edit does not match exactly
once, e.g. after a LiteEth version that generates different text; then
redo the edits by hand from the descriptions below and update the script.

The XDC LiteEth emits is **not** used as-is. The Eth-specific constraints
in [nexys_ddr.xdc](../../KlaussCPU.srcs/constrs_1/imports/new/nexys_ddr.xdc)
already include the needed elements (clock groups, false paths for CDC
markers, RMII I/O delays) with project-specific net/clock names.

## Sanity checks after regenerating

```bash
# Confirm the byte-order setting stuck
grep endianness liteeth_nexys_a7.yml         # → endianness: little

# Confirm the generator produced something fresh
ls -la generated/gateware/liteeth_core.v

# Confirm the top-level port list still matches KlaussCPU.v wiring
grep -A 30 "^module liteeth_core" generated/gateware/liteeth_core.v | head -25
```

Expected top-level ports (must match the instantiation in
[KlaussCPU.sv](../../KlaussCPU.srcs/sources_1/new/KlaussCPU.sv)):
- `sys_clock`, `sys_reset`, `interrupt`
- `rmii_clocks_ref_clk`, `rmii_crs_dv`, `rmii_mdc`, `rmii_mdio`, `rmii_rst_n`,
  `rmii_rx_data[1:0]`, `rmii_tx_data[1:0]`, `rmii_tx_en`
- Classic Wishbone slave: `wishbone_{adr[29:0], bte[1:0], cti[2:0], cyc, dat_r[31:0], dat_w[31:0], err, sel[3:0], stb, we, ack}`
- Added by the patch: `rx_nslots[3:0]`, `rx_slot_reset`

If the port list changes (LiteEth bumps its API), the top-level
instantiation in `KlaussCPU.sv`, the stub in `sim_1/new/tb_soc.sv` and the
bridge in `eth_mmio_bridge.sv` may need adjusting.

After regenerating, run `tools/ethsim/run_eth_tb.sh` (the real core under
Verilator: RX slots, TX, and the 2-slot behaviour against the previous core)
and `python3 tools/vsim/run.py soceth`.

If the CSR map changes (new register / different offset), regenerate or
manually update the Ethernet section of [MMIO_MAP.md](../../MMIO_MAP.md)
from `generated/liteeth_csrs.json`.

## What we changed at the project level (won't be regenerated)

These are project-specific decisions that aren't captured in the YAML —
they live in HDL or XDC and stay across regenerations:

1. **`liteeth_core.v` is wrapped** by [eth_mmio_bridge.sv](../../KlaussCPU.srcs/sources_1/new/eth_mmio_bridge.sv)
   which translates the CPU's 64-bit MMIO bus to LiteEth's 32-bit
   Wishbone. Since the 8-slot core (HW_PERF_PLAN.md §3) it also remaps the
   slot SRAM (LiteEth puts RX k at WB `0x20000 + k·0x800` and TX0 at
   `0x24000`; the CPU sees RX0/RX1 at `0xF008_0000/0800`, TX0 at
   `0xF008_1000`, RX2..RX7 at `0xF008_2000..4800`) and owns the
   `ETH_RX_SLOTS` register at `0xF007_0000`, which drives the patch's
   `rx_nslots` / `rx_slot_reset`.

2. **`ODDR_eth_refclk` in [KlaussCPU.sv](../../KlaussCPU.srcs/sources_1/new/KlaussCPU.sv)** has
   `D1=0, D2=1` (inverted), shifting the forwarded REFCLK by 180° so the
   PHY's sample point lands in the middle of the data-valid window.  The
   matching XDC has `create_generated_clock ... -edges {2 3 4}` so STA
   models the inversion correctly, plus a `set_false_path -fall_from`
   on the falling-edge launch (which for RMII SDR doesn't transition real
   data).  Don't simplify these.

3. **MAC address** is `02:00:00:00:00:01` — software writes the source MAC
   field of each TX frame.  LiteEth itself does **not** filter incoming
   frames by destination MAC; software does that.

4. **Hand edits to the generated `liteeth_core.v`**, applied by
   [patch_liteeth_core.py](patch_liteeth_core.py) (each one is marked in the
   file with a comment):
   - **`dont_touch` removed from the `rmii_clocks_ref_clk` port.** The
     attribute on a clock input port causes Synth 8-5396 (extra LUT logic on
     the WCLK net of the distributed-RAM CRC/stat FIFOs). The CDC sync FFs
     keep their own `async_reg` + `dont_touch`.
   - **RX slot count (`ETH_RX_SLOTS`).** The YAML builds `nrxslots: 8`. Two
     new inputs, `rx_nslots[3:0]` (2..8) and `rx_slot_reset`, both in the
     `sys_clock` domain and driven by the bridge, change the sram_writer
     (the RX DMA into slot SRAM):
     - the slot counter wraps at `rx_nslots` instead of 8;
     - the stat FIFO (one entry {slot, length} per filled slot; its head is
       what `RX_SLOT` / `RX_LENGTH` show) is full at `rx_nslots` entries
       instead of 8, so a frame that finds every slot in use is dropped and
       counted in `RX_ERRORS`;
     - the stat FIFO's produce / consume pointers wrap at `rx_nslots` too,
       so produce always equals the slot counter and, at 2 slots, the FIFO
       behaves bit-for-bit like the depth-2 FIFO of the old `nrxslots: 2`
       core (including what `RX_SLOT` reads while nothing is pending);
     - `rx_slot_reset` (one cycle: program load, or any `ETH_RX_SLOTS`
       write) zeroes the slot counter, the stat FIFO pointers and level, and
       the write length, and sends the writer FSM to WRITE — or to
       DISCARD-ALL if a frame is arriving, so the rest of that frame is not
       stored as a truncated frame. `RX_ERRORS` and everything else are
       untouched.

     With `rx_nslots` = 2 the core behaves exactly like the old 2-slot core
     (`tools/ethsim/run_eth_tb.sh all` compares the two).

5. **`CTRL_RESET` (CSR 0x0000) does nothing.** LiteX's SoCController
   `soc_rst` bit is not connected to any reset in the standalone core
   (`sys_rst` is just the `sys_reset` input). This is upstream behaviour,
   not a project edit; it is why `ETH_RX_SLOTS` writes flush the RX slot
   state themselves.

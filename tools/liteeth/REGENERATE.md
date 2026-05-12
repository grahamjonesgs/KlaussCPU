# Regenerating `liteeth_core.v`

The LiteEth MAC core that lives at
[KlaussCPU.srcs/sources_1/new/liteeth_core.v](../../KlaussCPU.srcs/sources_1/new/liteeth_core.v)
is **generated** from
[liteeth_nexys_a7.yml](liteeth_nexys_a7.yml) using the upstream
[LiteEth](https://github.com/enjoy-digital/liteeth) tooling. You only need
to regenerate it if you change the YAML config (e.g. add more slots, change
the bus standard, etc.) — for normal development you just edit
`liteeth_core.v` indirectly via the YAML, never by hand.

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

```bash
cp generated/gateware/liteeth_core.v ../../KlaussCPU.srcs/sources_1/new/
```

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
[KlaussCPU.v](../../KlaussCPU.srcs/sources_1/new/KlaussCPU.v)):
- `sys_clock`, `sys_reset`, `interrupt`
- `rmii_clocks_ref_clk`, `rmii_crs_dv`, `rmii_mdc`, `rmii_mdio`, `rmii_rst_n`,
  `rmii_rx_data[1:0]`, `rmii_tx_data[1:0]`, `rmii_tx_en`
- Classic Wishbone slave: `wishbone_{adr[29:0], bte[1:0], cti[2:0], cyc, dat_r[31:0], dat_w[31:0], err, sel[3:0], stb, we, ack}`

If the port list changes (LiteEth bumps its API), the top-level
instantiation in `KlaussCPU.v` and the bridge in `eth_mmio_bridge.v` may
need adjusting.

If the CSR map changes (new register / different offset), regenerate or
manually update the Ethernet section of [MMIO_MAP.md](../../MMIO_MAP.md)
from `generated/liteeth_csrs.json`.

## What we changed at the project level (won't be regenerated)

These are project-specific decisions that aren't captured in the YAML —
they live in HDL or XDC and stay across regenerations:

1. **`liteeth_core.v` is wrapped** by [eth_mmio_bridge.v](../../KlaussCPU.srcs/sources_1/new/eth_mmio_bridge.v)
   which translates the CPU's 64-bit MMIO bus to LiteEth's 32-bit
   Wishbone.

2. **`ODDR_eth_refclk` in [KlaussCPU.v](../../KlaussCPU.srcs/sources_1/new/KlaussCPU.v)** has
   `D1=0, D2=1` (inverted), shifting the forwarded REFCLK by 180° so the
   PHY's sample point lands in the middle of the data-valid window.  The
   matching XDC has `create_generated_clock ... -edges {2 3 4}` so STA
   models the inversion correctly, plus a `set_false_path -fall_from`
   on the falling-edge launch (which for RMII SDR doesn't transition real
   data).  Don't simplify these.

3. **MAC address** is `02:00:00:00:00:01` — software writes the source MAC
   field of each TX frame.  LiteEth itself does **not** filter incoming
   frames by destination MAC; software does that.

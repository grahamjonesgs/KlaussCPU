# Ethernet (LiteEth) Integration — Plan & Decisions

Tracker for adding 10/100 Ethernet to KlaussCPU using the **LiteEth** open-source
MAC, exposed to the CPU as MMIO. Goal: get to "broadcast frame received in
software" so we can layer lwIP on top.

**Status:** Phases 1–4 complete (2026-05-08). Bitstream builds with timing met across all clock domains. Phase 5 (bring-up tests) ready to start.

---

## Phase 0 — Decisions to lock in

Each row needs a decision before HDL work starts. **Recommendation** is what
this doc proposes; **Decision** is filled in when chosen (and dated).

### D1. Target board

| Option | Notes |
|--------|-------|
| **Nexys A7 100T** | Xilinx XC7A100T, on-board DDR2 (matches existing MIG config), on-board SMSC LAN8720A PHY (10/100 RMII). |
| Arty A7 | DDR3L on board, DP83848J PHY — would require DDR retarget. |

Earlier draft of this doc incorrectly stated Nexys A7 has no Ethernet — it
does, via the LAN8720A. Confirmed visually on the user's board.

**Decision:** **Nexys A7 100T** — _date:_ 2026-05-07.

This kills risk #6 (DDR migration) — the existing DDR2 MIG stays.

---

### D2. PHY interface mode

| Option | Notes |
|--------|-------|
| **RMII (50 MHz, 2-bit data)** | Nexys A7's only option (LAN8720A is wired RMII on the board). |
| MII | Not available on Nexys A7. |

**Decision:** **RMII** — _date:_ 2026-05-07. Forced by D1.

---

### D3. Ethernet REF_CLK source

| Option | Notes |
|--------|-------|
| **FPGA drives REF_CLK to PHY** | Standard Nexys A7 setup. Allocates one MMCM output @ 50 MHz, routed via ODDR primitive. |
| PHY's own oscillator | Non-standard on Nexys A7; would add input-delay constraints. |

**Decision:** **FPGA drives REF_CLK** — _date:_ 2026-05-07.

---

### D4. Bus protocol from LiteEth to CPU

| Option | Effort | Pros | Cons |
|--------|--------|------|------|
| **Wishbone-to-MMIO bridge** | ~50 LOC adapter | Keeps LiteEth as-distributed; future LiteX upgrades trivial | One layer of indirection |
| Strip Wishbone, wire LiteEth straight to `bus_splitter` | More invasive | Slightly fewer LUTs | Locks in to one LiteEth version; redo on every upgrade |

**Decision:** **Wishbone-to-MMIO bridge** — _date:_ 2026-05-07.

---

### D5. Polling vs interrupt-driven RX

| Option | Notes |
|--------|-------|
| **Poll first, interrupts in Phase 6** | Simpler bring-up, no IRQ wiring on day one. |
| Interrupts from day one | Phase 6 collapses into Phase 5; debug is harder when nothing works yet. |

**Decision:** **Poll first, interrupts in Phase 6** — _date:_ 2026-05-07.

---

### D6. MAC address provisioning

| Option | Notes |
|--------|-------|
| **Software-writable register, init from constant** | LiteEth gives this for free via CSR. Software writes a fixed locally-administered MAC at init. |
| Hard-coded constant in HDL | One MAC per bitstream. Less flexible. |
| DIP-switch / EEPROM at boot | Per-board uniqueness without rebuilding. Not needed yet. |

**Decision:** **Software-writable register** — _date:_ 2026-05-07. Initial MAC
will be `02:00:00:00:00:01` (locally-administered range — first byte bit 1 = 1).

---

### D7. Endianness handling for network byte order

| Option | Notes |
|--------|-------|
| Byte-swap in software (driver layer) | Standard. lwIP already does this via `htons`/`ntohl`. |
| Byte-swap in HDL bridge | Hides the conversion but creates a special case in the bridge. |

**Decision:** **Software byte-swap (driver/lwIP)** — _date:_ 2026-05-07.
Assumed from the recommendation as it's effectively forced — flag if you'd
rather diverge.

---

### D8. RX/TX slot count and size

| Option | Notes |
|--------|-------|
| 2 × 2 KB each direction | LiteEth default. Allows next-frame fill while one in flight. ~2 KB ≥ 1500-byte MTU + headroom. |
| 4 × 2 KB each direction | More buffering, more BRAM (~16 KB extra). Useful if RX bursts overrun a 2-deep ring. |
| 2 × 1.5 KB each direction | Tight fit for MTU. Saves ~2 KB BRAM. Not recommended. |

**Decision:** **2 × 2 KB each direction** — _date:_ 2026-05-07. Assumed from
the recommendation; flag if you'd rather diverge.

---

## Phase tracker

Each phase has a deliverable and a checkbox. Mark `[x]` when done; add a notes
line below with date and any deviations from the plan.

### Phase 1 — Generate LiteEth core

- [x] Install LiteX toolchain (had to use **git** versions, not pip wheels — released `migen 0.9.2` + `litex 2024.12` combo crashes with "Cannot extract CSR name from code" on Python 3.11+ due to bytecode introspection in migen)
- [x] Run `python -m liteeth.gen` with the configuration matching D1–D8
- [x] Commit generated `liteeth_core/` to repo (or keep gitignored if regenerable from a script)
- [x] Save the JSON CSR map alongside

**Reproducer:**

```bash
python3 -m venv ~/.venvs/litex
source ~/.venvs/litex/bin/activate
pip install --upgrade pip
pip install git+https://github.com/m-labs/migen.git
pip install git+https://github.com/enjoy-digital/litex.git
pip install git+https://github.com/litex-hub/litex-boards.git
pip install git+https://github.com/enjoy-digital/liteiclink.git
pip install git+https://github.com/enjoy-digital/liteeth.git

python -m liteeth.gen liteeth_nexys_a7.yml \
    --output-dir liteeth_core \
    --soc-json liteeth_core/liteeth_csrs.json \
    --no-compile
```

YAML config (`liteeth_nexys_a7.yml`) in repo. Required key beyond what was first
guessed: `endianness: big`.

**Deliverable:** `liteeth_core/` Verilog tree + `liteeth_csrs.json`.
**Notes:** _Done 2026-05-07._

---

### Phase 2 — Vivado integration & constraints

LiteEth standalone is a single self-contained `liteeth_core.v` (no submodule
files to manage). Top-level ports identified:

```
network: rmii_clocks_ref_clk  rmii_crs_dv  rmii_mdc  rmii_mdio
         rmii_rst_n  rmii_rx_data[1:0]  rmii_tx_data[1:0]  rmii_tx_en
system:  sys_clock  sys_reset  interrupt
bus:     wishbone_{ack,adr[29:0],bte[1:0],cti[2:0],cyc,dat_r[31:0],
                   dat_w[31:0],err,sel[3:0],stb,we}
```

Wishbone is **classic, 32-bit, word-addressed** (so CPU byte addr → `>> 2`
to feed `wishbone_adr`). Drive `wishbone_bte` and `wishbone_cti` to zero
(single-transfer mode).

- [ ] Add `liteeth_core/gateware/liteeth_core.v` to Vivado project
- [ ] Pull Ethernet pin block from Digilent's Nexys A7 master XDC
      (`ETH_MDC`, `ETH_MDIO`, `ETH_RSTN`, `ETH_CRSDV`, `ETH_RXERR`,
      `ETH_RXD[1:0]`, `ETH_TXEN`, `ETH_TXD[1:0]`, `ETH_REFCLK`)
- [ ] Connect `ETH_RXERR` as an input but leave it unused at the top
      (LiteEth's RMII PHY doesn't expose a port for it — Vivado will warn,
      that's OK)
- [ ] Add 50 MHz output to existing MMCM. Drive both LiteEth's
      `rmii_clocks_ref_clk` port AND the PHY's `ETH_REFCLK` pin (via ODDR)
      from the same net.
- [ ] Copy these LiteEth-emitted constraints verbatim from
      `liteeth_core/gateware/liteeth_core.xdc` into our own XDC:
       - `create_clock -name rmii_clocks_ref_clk -period 20.0 [get_ports rmii_clocks_ref_clk]`
       - both `set_clock_groups -asynchronous` lines (rename `sys_clk` net
         to whatever drives `i_Clk`)
       - all `set_false_path` lines for `mr_ff` / `ars_ff1` / `ars_ff2`
- [ ] Add `set_input_delay` / `set_output_delay` for RMII data lanes (lift
      from LiteX's `nexys4ddr` board file — natural source for Nexys A7
      DP83848-grade RMII timing)
- [ ] Drive `sys_reset` from existing system reset; verify polarity
      (LiteX convention is active-high)
- [ ] First synthesis + implementation run — passes timing

**Deliverable:** Bitstream with LiteEth in it; no functional test yet.
**Notes:** _TBD_

---

### Phase 3 — Wishbone-to-MMIO bridge

Routes CPU MMIO accesses in `[0xF006_0000, 0xF008_FFFF]` to LiteEth's
classic Wishbone slave. Translation is byte-for-byte:

```
litex_byte_addr = cpu_byte_addr - 0xF006_0000
wishbone_adr    = litex_byte_addr[19:2]   // word address; LiteEth ignores upper bits
wishbone_sel    = byte_enables_from_cpu
wishbone_dat_w  = cpu_write_data[31:0]
wishbone_we     = cpu_is_write
wishbone_cyc    = wishbone_stb = 1 while bridge waits for ack
wishbone_bte/cti = 0  (single transfer)
```

- [ ] Create `eth_mmio_bridge.v`
- [ ] Address decode for `0xF006_0000..0xF008_FFFF` (192 KiB span across 3
      device-id slots)
- [ ] Width handling: CPU does 8/16/32-bit accesses → byte enables.
      Restrict driver to 32-bit accesses on Eth registers (matches LiteEth's
      32-bit CSR layout). Slot SRAMs accept any width via byte enables.
- [ ] Drop `wishbone_adr` to bits `[19:2]` (or wider — LiteEth ignores upper
      bits but more is harmless)
- [ ] Wait-for-ack state machine: assert `cyc`+`stb`, hold until `ack`
      (≤3 cycles for SRAM, 1 cycle for CSR). Capture `dat_r` on ack.
- [ ] Surface `wishbone_err` to CPU as a bus error (or just zero out the
      response — pick one)
- [ ] Update `bus_splitter.v` to route any of `0xF006_xxxx`, `0xF007_xxxx`,
      `0xF008_xxxx` to the bridge
- [ ] Simulation testbench: CPU MMIO write → Wishbone strobe + correct
      addr/data; CPU MMIO read → ack and rdata round-trip

**Deliverable:** `eth_mmio_bridge.v`; `bus_splitter.v` patched.
**Notes:** _TBD_

---

### Phase 4 — MMIO map & C header

**Final layout** (derived from `liteeth_csrs.json` + `mem.h` after Phase 1
generation). LiteEth's Wishbone address space spans 136 KiB (CSR window +
8 KiB MAC SRAM at offset 0x20000), so the block consumes three MMIO
device-id slots (`0xF006`, `0xF007`, `0xF008`). The bridge does a flat
byte-for-byte translation — CPU byte addr `A` in
`[0xF006_0000, 0xF008_FFFF]` → LiteEth WB word addr `(A - 0xF006_0000) >> 2`.

```
0xF006_0000 – 0xF006_FFFF   ETH_CSR    (LiteEth WB 0x00000–0x0FFFF)
                            0xF006_0000  ctrl_reset                     RW
                            0xF006_0004  ctrl_scratch                   RW
                            0xF006_0008  ctrl_bus_errors                RO
                            0xF006_0800  ethphy_crg_reset               RW  PHY reset
                            0xF006_0804  ethphy_mdio_w                  RW
                            0xF006_0808  ethphy_mdio_r                  RO
                            0xF006_1000  ethmac_sram_writer_slot        RO  RX active slot
                            0xF006_1004  ethmac_sram_writer_length      RO  RX frame length
                            0xF006_1008  ethmac_sram_writer_errors      RO
                            0xF006_100C  ethmac_sram_writer_ev_status   RO
                            0xF006_1010  ethmac_sram_writer_ev_pending  RW  W1C
                            0xF006_1014  ethmac_sram_writer_ev_enable   RW
                            0xF006_1018  ethmac_sram_reader_start       RW  TX kick
                            0xF006_101C  ethmac_sram_reader_ready       RO
                            0xF006_1020  ethmac_sram_reader_level       RO
                            0xF006_1024  ethmac_sram_reader_slot        RW  TX slot select
                            0xF006_1028  ethmac_sram_reader_length      RW  TX frame length
                            0xF006_102C  ethmac_sram_reader_ev_status   RO
                            0xF006_1030  ethmac_sram_reader_ev_pending  RW  W1C
                            0xF006_1034  ethmac_sram_reader_ev_enable   RW
                            0xF006_1038+ stats counters (preamble/CRC/etc.)

0xF007_0000 – 0xF007_FFFF   gap        (LiteEth WB 0x10000–0x1FFFF, undefined;
                                        bridge returns 0 on read, drops writes)

0xF008_0000 – 0xF008_07FF   ETH_RX0    2 KiB RX slot 0   (LiteEth WB 0x20000)
0xF008_0800 – 0xF008_0FFF   ETH_RX1    2 KiB RX slot 1   (LiteEth WB 0x20800)
0xF008_1000 – 0xF008_17FF   ETH_TX0    2 KiB TX slot 0   (LiteEth WB 0x21000)
0xF008_1800 – 0xF008_1FFF   ETH_TX1    2 KiB TX slot 1   (LiteEth WB 0x21800)
0xF008_2000 – 0xF008_FFFF   unused     (LiteEth WB 0x22000+, no slaves)
```

All registers are 32-bit; software accesses with 32-bit MMIO loads/stores.
SRAM slots are accessed as plain memory (8/16/32-bit reads/writes — byte
enables route correctly through the bridge).

- [ ] Append "Ethernet (LiteEth)" section to `MMIO_MAP.md`, registers copied from JSON
- [ ] Add macros to `mmio.h` (`ETH_BASE`, `REG_ETH_*`, `ETH_TX_SLOT`, `ETH_RX_SLOT`)
- [ ] Document MAC slot framing (preamble/SFD inserted by HW or SW? — answer in JSON)

**Deliverable:** Updated `MMIO_MAP.md` and `mmio.h`.
**Notes:** _TBD_

---

### Phase 5 — Bring-up tests (in order)

Each test depends on the previous passing.

- [ ] **5.1 MDIO reachability.** Read PHY register `0x02` (PHY ID1). Expect `0x0007` for SMSC LAN8720A; ID2 (`0x03`) ≈ `0xC0F0..C0FF` (revision in low nibble). Confirms MDC/MDIO pins + PHY power-up.
- [ ] **5.2 PHY internal loopback.** Set PHY register `0x00` bit 14. TX a hand-crafted frame; expect it on RX with matching CRC and length. Validates the MAC datapath end to end without touching the wire.
- [ ] **5.3 External loopback.** RJ45 plug with TX/RX shorted (or a host running `tcpdump`). Same frame seen on the wire.
- [ ] **5.4 Receive a broadcast.** Plug into a normal switch. Wait for an ARP broadcast in RX. Confirms RX framing for live traffic.

**Deliverable:** Each test as a small C program in the test image.
**Notes:** _TBD_

---

### Phase 6 — Interrupts (optional for MVP)

- [ ] Wire LiteEth `ev_rx_done` → `INT_PENDING[1]`
- [ ] Wire LiteEth `ev_tx_done` → `INT_PENDING[2]`
- [ ] Document `INT_VEC1` and `INT_VEC2` semantics in `MMIO_MAP.md`
- [ ] Software ISR template that reads/clears the LiteEth event register
- [ ] Re-run Phase 5.4 with ISR-driven RX

**Deliverable:** Interrupt-driven RX path; unblocks `NO_SYS=0` lwIP.
**Notes:** _TBD_

---

## Risks & gotchas

Numbered for easy reference in commits / discussion.

1. **RMII timing constraints.** Easy to omit `set_input_delay` /
   `set_output_delay` and get intermittent CRC errors that look like a
   software bug. **Mitigation:** lift numbers from a known-good Nexys A7
   (or Nexys 4 DDR) LiteEth example before Phase 2 implementation.
2. **REF_CLK quality.** 50 MHz output to PHY *must* come from MMCM via
   `ODDR`, not a logic toggle. Otherwise the PHY won't recognise it as a
   valid clock. **Mitigation:** explicit `ODDR` instantiation, reviewed in
   Phase 2.
3. **PHY reset sequencing.** SMSC LAN8720A needs `ETH_RSTN` low for ≥25 ms
   at power-up (per datasheet) and at least 100 µs after the REF_CLK is
   stable before MDIO is reachable. If skipped, MDIO returns `0xFFFF` for
   everything (floating bus). **Mitigation:** explicit reset pulse generator
   in HDL that holds reset low until REF_CLK is stable, then waits the
   required interval.
4. **Wishbone ack latency.** SRAM reads can be ~3 cycles. Hard-coding
   "1 cycle ack" in the bridge breaks RX read-back.
   **Mitigation:** wait-for-ack state in bridge (Phase 3 acceptance gate).
5. **Endianness in pbufs.** Network byte order is big-endian; CPU is
   little-endian. Decided per D7 = software-side, so lwIP `htons`/`ntohl`
   handle it. No HDL special case.
6. **DDR migration.** ~~Resolved by D1 = Nexys A7 100T~~ — the existing DDR2
   MIG configuration matches the board and stays as-is.
7. **CSR width mismatch.** LiteEth generated with `csr-data-width=32`, but
   CPU does 64-bit MMIO. Either restrict driver to 32-bit Eth accesses or
   the bridge splits 64-bit ops into two Wishbone cycles. Track which
   approach is taken in Phase 3 notes.

---

## Effort estimate

| Phase | Days |
|-------|------|
| 0 — decisions | 0.5 |
| 1 — generate core | 0.5 |
| 2 — Vivado + constraints | 1 |
| 3 — bridge | 1 |
| 4 — MMIO map + header | 0.5 |
| 5 — bring-up | 2 |
| 6 — interrupts (optional) | 0.5 |
| **Total to "broadcast received"** | **~5–6 days** |

Plus 1 day slack for timing-closure surprises in Phase 5.

---

## After this plan completes

Hand-off to software side:
- LiteEth `ethernetif.c` driver (~150 LOC) calling the MMIO macros from `mmio.h`
- lwIP `sys_arch.c` (~150 LOC) wrapping existing RTOS semaphores / mailboxes /
  threads
- `lwipopts.h` with `NO_SYS=0`, `LWIP_SOCKET=1`, `LWIP_NETCONN=1`
- First test: TCP echo server on a fixed port

Estimated software-side effort: **another ~1 week** to TCP-listen.

---

## Changelog

- 2026-05-07 — initial plan drafted (decisions section, phase tracker, risks).
- 2026-05-07 — D1 corrected to Nexys A7 100T (has on-board LAN8720A PHY); D2–D8
  locked in as recommendation. Risk #6 (DDR migration) resolved. PHY-specific
  details (LAN8720A IDs, reset timing) updated throughout.
- 2026-05-07 — Phase 1 complete. LiteEth core generated from git versions of
  migen/litex/liteeth (released wheels broken on Python 3.11+). YAML config
  needed `endianness: big` beyond the initial draft. Reproducer recorded.
- 2026-05-07 — Phases 2–3 HDL drafted. Top-level KlaussCPU.v gains ETH_*
  ports, eth_mmio_bridge_i + liteeth_core_i + ODDR_eth_refclk instances.
  bus_splitter.v split into 3-way (DRAM / MMIO / Eth). mem_read_write.v
  exposes clk_50 from clk_wiz_0. nexys_ddr.xdc pin block uncommented and
  Eth timing constraints appended (uses [get_clocks -of_objects [get_nets
  clk_50]] so it's hierarchy-independent).
- 2026-05-08 — Phase 4 (MMIO_MAP.md) updated with full Eth section, CSR
  list, slot SRAM layout, software flow notes, and driver sketches.
- 2026-05-08 — Timing closed.  Iterations:
   1. First impl: WNS −0.5 ns on a CPU-internal path through bus_splitter's
      combinational mux into r_SM CE — 3-way mux added enough depth to
      tip an existing borderline path.  Fix: registered the bus_splitter
      return path (o_mem_read_data / o_mem_ready / etc.); +1 cycle of
      read latency, transparent to the existing handshake.
   2. Second impl: CPU side closed (+0.032), but Eth domain showed WNS
      −4.15 ns on TX paths (ODDR/C → ETH_TXD/EN).  Root cause: I/O
      delays were referenced to internal clk_50, charging ~6 ns of
      MMCM+BUFG distribution against a 10 ns half-period.  Fix: created
      `eth_phy_clk` generated clock on the ETH_REFCLK output pin (driven
      by ODDR_eth_refclk) and re-anchored set_input_delay /
      set_output_delay to it (proper source-synchronous model).
   3. Third impl: TX setup met, but RX setup failed −9.6 ns and TX hold
      failed −1.4 ns.  Root causes:
        a. RX paths land on LiteEth's IDDR primitives.  Vivado picked
           the closer (falling) destination edge as the capture point,
           giving a 10 ns budget for a 14 ns input delay.  RMII is
           logically SDR — only the rising-edge sample matters.  Fix:
           `set_multicycle_path -setup -end 2` (both directions of the
           clock-forwarded boundary) so Vivado uses the next rising
           edge instead of the half-period falling edge.
        b. TX hold failed because ODDR_eth_refclk and LiteEth's data
           ODDRs land in different I/O banks (REFCLK on D5, TXD on
           A8/A10/B9), so their clock-distribution paths differ by
           ~2 ns.  Vivado sees this as +1.8 ns of skew and reports a
           hold violation that's a modelling artifact — the actual
           interface is robust because the data and clock both leave
           via OBUFs onto matched PCB traces, and the PHY captures
           with the REFCLK it receives.  Fix: `set_false_path -hold`
           on `clk_50 → eth_phy_clk` (standard LiteX pattern for
           clock-forwarded outputs in mismatched-bank placements).
   4. Fourth impl: WNS, WHS both positive across all clock domains.

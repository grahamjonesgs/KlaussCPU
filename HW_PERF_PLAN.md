# HW_PERF_PLAN — Linux-driven hardware improvements (2026-11-11)

Branch `perf-hw` (off master 2743116). Requested by Graham via the runtime
session (klausscpu-runtime-8f): estimate the benefit first, then build.
The runtime session owns the software side (klausscc, kernel, musl); this
repository owns the RTL. The klaussCPU session owns the ISA map and the LLVM MC
work.

Backward-compatibility rule (as for the MMU): a program that does not use
the new features sees identical behaviour. Every new behaviour is either a
new encoding (illegal before) or opt-in through a register that resets to
the old behaviour.

Order agreed with the runtime session: 1 → 3 → 2, with 4 wherever
convenient.

## Estimates (runtime session, 2026-11-11)

Board counters come from the current kernel. Miss counts come from the
klausscc emulator cache model running the Linux fork/exec/fault workloads.

| Item | Evidence | Expected gain |
|---|---|---|
| 1. DCBZ | 96.8 / 96.7 / 98.4 % of D write misses (fork / exec / fault) are lines that are then written whole; a write miss costs ~86 cycles/line vs 67 for a read stream, so the avoidable fetch is ~55 cycles | fork −24 %, exec −24 %, fault −36 %; memset/clear bandwidth 2–3× (37 MB/s today) |
| 2. I-cache 8 → 16 KiB DM, 16 B lines | I-fill cycles are 26.7 / 28.5 / 33.8 % of cycles; misses −13 / −20 / −18 %. 32 B lines give no gain once a D-miss fill delivers 2 dwords and the NLB is modelled | ≈ −4 / −6 / −6 % time (32 KiB ≈ −8 / −10 / −14 %, if cheap) |
| 3. LiteEth 8 RX slots | PC→board 11.7 Mbit/s with 2192 TCP retransmits in 15 s | retransmits ≈ 0, receive +10–20 % (CPU-bound) |
| 4. Misaligned 64-bit trap (opt-in) | lets the kernel emulate misaligned i64 | correctness only, no speed |

## 1. DCBZ — zero a cache line without fetching it

### Encoding

`DCBZ [rs1]` = **`0x5F00_2000 | rs1 << 4`**

Fields: class 7 (store), LEN = 01, SIZE `[25:24]` = 11, SGN `[23]` = 0,
MODE `[22:21]` = 00, A `[20]` = 0, `[19:14]` = 0, **`[13]` = 1**, `[12]` = 0,
`[11:8]` = 0, rs1 `[7:4]` = address, `[3:0]` = 0. One word.

This form is illegal on older bitstreams (MODE = 00 requires `[19:12]` = 0;
`[12]` = 1 is SC, MMU_PLAN §5.1). Any other combination with `[13]` = 1 is
ILLEGAL: `[12]` = 1 as well, another SIZE, SGN = 1, A = 1, another MODE, or
a nonzero `[11:8]` or `[3:0]`.

### Semantics

- **Line.** The instruction zeroes the whole cache line that contains EA = rs1:
  bytes `EA & ~(L−1)` to `(EA | (L−1))`, where L = `CACHE_INFO[31:24]` (32 on every
  current bitstream). Software must read L and must not hard-code 32. The
  address is aligned down. DCBZ **never** raises MISALIGNED, not even with
  STATUS.MAL64 = 1 (§4).
- **No fetch.** On a D-cache hit the line is overwritten with zeros and marked
  dirty. On a miss the victim is evicted in the usual way (shadow writeback
  if it is dirty) and a zero line is installed dirty, with no DDR read.
- **Permissions.** DCBZ is checked as a store of the whole line, in both
  supervisor and user mode. TLB (TE = 1): it needs store permission on the
  page, and a fault is a TLB-miss or ACCESS store fault exactly as for a
  64-bit store at EA. MPU (where built): a store check at EA; the line
  cannot cross a region granule, because granules are ≥ 32 B. EADDR = the raw EA
  (not aligned down).
- **MMIO.** If EA is `0xF…`, DCBZ raises ACCESS_STORE in **both** modes. It is
  a new instruction, so this changes no existing behaviour (the same rule
  as LR/SC).
- **Ordering.** It behaves as a store: every later load of any byte of the
  line, in program order, returns 0. It is ordered against earlier stores
  exactly as a store is.
- **LR/SC.** It affects the reservation exactly as an ordinary store does.
- **I-cache.** It snoops the I-cache like a store, covering **every** I-cache line
  inside the zeroed line (two 16 B lines today, one if the I-cache line
  ever becomes 32 B), so the existing self-modifying-code guarantee holds.
- **DMA (blitter, VGA, SD).** Same as an ordinary store: the line is dirty in
  the cache until it is evicted or flushed.
- **Counters.** It counts as one write hit or write miss in
  `CNT_WRITE_HITS` / `CNT_WRITE_MISSES`. The new counter `CNT_ZERO_LINES`
  at `0xF005_0070` counts DCBZ operations.
- **Trace / debug.** It retires one trace line like a store.
- **Core 2 / builds without the D-cache.** The build parameter `DCBZ_EN` is 1 on
  core 1. On core 2 it is 0, and there the encoding is ILLEGAL.
- **Feature bit.** `CPU_FEATURES[22]` = 1 means DCBZ is present.

### Intended software use

`clear_page`, `copy_page` (DCBZ the destination line, then 4 stores),
and `memset`/`memcpy` for line-aligned destination runs ≥ L. A 64 KiB
`clear_page` through the cache still evicts the whole 64 KiB D-cache; a
cache-bypassing clear is a possible later item.

## 2. I-cache 16 KiB direct-mapped, 16 B lines

Today: 512 × 16 B direct-mapped, VIPT with index VA[12:4] (pipeline_core
M7a/M7b). Target: 1024 × 16 B, index VA[13:4], tag PA[31:14]. 64 KiB pages
mean any size up to 64 KiB direct-mapped has no aliasing problem. The work
that scales with the line count:

- `ic_tagff`, the tag shadow read combinationally by the store snoop (the
  tier-1 timing family), grows 512 → 1024 entries.
- The per-dword valid bits in flip-flops grow 1024 → 2048, cleared in one
  cycle on fence.i / reset.

If 16 KiB does not close timing as it stands, move the snoop's tag read
to the tag BRAM's second port and the valid bits to BRAM with a
generation counter. That redesign would also open the way to 32 KiB.

A new read-only `ICACHE_INFO` at `0xF005_0018`, in the same format as
`CACHE_INFO`: 1 way, 1024 sets, 16 B, 16 KiB.

### §2 implementation notes (2026-11-11)

- **One constant.** `pipeline_core` has a parameter `IC_LINES` (1024 on
  core 1 through the top's `CORE1_IC_LINES`; core 2 keeps 512). `IC_IDXW`,
  the tag width `IC_TAGW` = 28 − `IC_IDXW` and every index / tag slice
  derive from it. With TE = 1 the tag is the constant shift
  `{PA[31:16], VA[15:3]} >> (IC_IDXW + 1)`, i.e. `{PA[31:16], VA[15:14]}`
  at 16 KiB; it stays correct for any size up to 64 KiB (the index has to
  stay inside the page). `ICACHE_INFO` is built from the same parameter.
- **Not the BRAM-port / generation-counter redesign.** The snoop's tag
  shadow is already a block RAM: Vivado absorbs `mem_iaddr`'s register as
  the RAM's read address (8 KiB synthesis logs: "Read Address Register
  Found"), so the snoop path is BRAM clock-to-out → tag compare, and
  1024 × 18 still fits one RAMB18. What grows with the line count is the
  flip-flop valid bits and their read multiplexers. Those are kept at the
  8 KiB depth by splitting on index bit 0 (the even / odd 16 B line of a
  32 B pair, which is how DCBZ already snooped):
  - the tag shadow is two banks, `ic_tagff_e` / `ic_tagff_o`, 512 × 18,
    each with **one** read address (the pair index `mem_iaddr[13:5]`).
    The DCBZ-era single shadow read at two addresses became
    RAM64M × 112 (~416 LUTRAM) in synthesis; each bank now maps like the
    8 KiB build's shadow;
  - the snoop reads the valid bits at the pair index (512:1 muxes, as
    before);
  - the fetch lookup registers the valid bits of both lines of the pair
    (`ic_rd_v*_e` / `_o`, 512:1 muxes) and `ic_rd_idx[0]` picks one
    after the register, inside the existing valid-select LUTs
    (`ic_dw_valid`, `ifr_next_valid`).
  The valid bits are packed vectors (2 × 1024 flip-flops), still cleared in
  one cycle by reset, `start` and ICACHE_INV.
- **Tag-shadow guard (separate commit).** Because the shadow is a block
  RAM read at `mem_iaddr`'s load edge, an I-cache install landing at or
  after that edge (a fill completing while a store waits in MEM) can be
  missing from the store's compare on hardware, while every simulator
  reads the current value. The core now tracks the installs since
  `mem_iaddr` was loaded (`ri_v`, `ri_idx`, `ri_m`) and treats a store to
  such a line (a DCBZ: its pair) as a tag hit: it clears the line and does
  not post. It fired in 3 of ~1000 directed tests (+9 to +15 cycles each);
  no directed or swept self-modifying-code case produced a stale read in a
  simulation model of the RAM's sampling, so it is defensive.
- **Area estimate** (not synthesised here): BRAM +2 RAMB36 for `ic_data`
  (1024 × 128); `ic_tag` stays one RAMB18; the tag shadow is one or two
  RAMB18 (and the DCBZ-era LUTRAM shadow goes away). About +1.1k flip-flops
  (valid bits, BTIC line indices one bit wider) and +1 to 1.5k LUTs
  (valid-bit write decode, the extra 512:1 read muxes).
- **Timing-sensitive paths.** The valid-bit flash clear (ICACHE_INV,
  `start`, reset) now fans out to 2048 flip-flops; the snoop clear decode
  likewise. `ic_data` spans four RAMB36 instead of two (longer routes on
  the 128-bit read and on its address). The lookup's valid read depth, the
  snoop's tag and valid read depth, and the hit path's logic depth are as
  at 8 KiB.
- **Tests** (tools/vsim): `ic16_loop`, `ic16_smc_alias`, `ic16_dcbz_pair`,
  `ic16_icinv`, `ic16_te_pages`, `ic16_smc_fill`; `socmmio` checks
  `ICACHE_INFO`. With `IC_LINES` = 512 every test passes except the
  miss-count checks of the 16 KiB-specific ones (they then show the 8 KiB
  numbers, e.g. a 12 KiB loop misses 770 / 1284 / 2311 times after 1 / 2 / 4
  passes instead of 770 each time).

## 3. LiteEth: 8 RX slots, opt-in

- Regenerate with `nrxslots: 8` (tools/liteeth/liteeth_nexys_a7.yml). TX
  stays at 1 slot (see MMIO_MAP "Only one TX slot").
- **CPU address map.** The bridge remaps the slot SRAM so that old addresses do not
  move:
  - RX k is at `0xF008_0000 + (k < 2 ? k·0x800 : 0x1000 + k·0x800)`
    (RX2 at `0xF008_2000` … RX7 at `0xF008_4800`).
  - TX0 stays at `0xF008_1000`.
- **`ETH_RX_SLOTS`** at `0xF007_0000` (the gap window, bridge-owned). 32-bit RW:
  - `[3:0]` is the number of slots the MAC cycles through. It resets to 2.
    A write of 0 or 1 gives 2, and a write above 8 gives 8.
  - `[11:8]` is read-only and holds the maximum (8).
  - Other bits read 0.
  - It is also reset to 2 by program load (LOAD_COMPLETE), and the MAC's
    RX slot counter restarts at 0. That way an old program loaded after Linux
    sees today's 2-slot behaviour.
- **Changing the slot count.** Software writes `ETH_RX_SLOTS`, then pulses
  `CTRL_RESET` (CSR 0x0000), then enables RX. A change while frames are
  pending is undefined.
- `RX_SLOT` (CSR 0x1000) reads 0 … N−1.
- Hardware drops are already counted by `RX_ERRORS` (CSR 0x1008, free-running).
- This needs a small project-level patch to the generated sram_writer (wrap
  the slot counter at `ETH_RX_SLOTS`). It goes in REGENERATE.md "What we
  changed".
- **Feature bit.** `CPU_FEATURES[24]` = 1 means `ETH_RX_SLOTS` is present.

### §3 implementation notes (branch `perf-eth`, 2026-11-11)

Implemented as specified, with these choices where the specification could
not be followed literally or left a detail open:

- **`CTRL_RESET` does nothing** in the standalone LiteEth core (LiteX's
  `soc_rst` bit is not wired to any reset; true of the old core too), so it
  cannot restart the slot counter. Closest to the intent: **every write of
  `ETH_RX_SLOTS` flushes the MAC's RX slot state** (slot counter to 0,
  unreleased frames dropped). The specified sequence (write `ETH_RX_SLOTS`,
  pulse `CTRL_RESET`, enable RX) still works; a change while frames are
  pending is now defined (they are dropped) instead of undefined.
- **Program load flushes too**, not just restarting the counter: frames left
  pending by Linux would otherwise show `RX_SLOT` values ≥ 2 to an old
  program. A frame arriving during a flush is discarded (not stored
  truncated). `RX_ERRORS` is never cleared.
- **Clamp** compares the whole 32-bit written value (16 → 8, not 0 → 2). A
  write must enable byte 0; one that does not is ignored.
- **The whole `0xF007` window is bridge-owned**: everything except
  `0xF007_0000` reads 0 and drops writes (MMIO_MAP already said so; before,
  an access timed out after 256 cycles and read `0xFFFF_FFFF`).
- **2-slot equivalence**: the stat FIFO pointers also wrap at
  `ETH_RX_SLOTS`, so at 2 slots the sram_writer is bit-for-bit the old one,
  including the value `RX_SLOT` reads while nothing is pending.
- Generated core: LiteX `30dbdec` / LiteEth `cd479a4` / migen `e19524c`
  (tools/liteeth/REGENERATE.md); hand edits in
  tools/liteeth/patch_liteeth_core.py.
- Cost estimate (no synthesis run): +6 RAMB18 (8 RX + 1 TX slot = 9 RAMB18,
  was 3), about +150–250 LUTs (8-way slot read mux, decoder, 8-entry stat
  FIFO, slot/pointer wrap compares) and a few dozen FFs.
- Verification: `tools/ethsim/run_eth_tb.sh all` (the real generated core
  under Verilator, RMII frames injected / captured; at 2 slots the
  transcript is identical to the old core's) and `tools/vsim/run.py soceth`
  (`ETH_RX_SLOTS` and `CPU_FEATURES[24]` through the SoC bus).

## 4. Misaligned 64-bit accesses trap (opt-in)

- **`STATUS[10]` MAL64**: supervisor read/write through MTSR STATUS, readable in user mode like
  the other STATUS bits. It resets to 0 and is cleared at `start` / program load.
- **MAL64 = 1.** Every 64-bit load or store whose EA[2:0] ≠ 0 raises
  MISALIGNED_LOAD / MISALIGNED_STORE (causes 5 / 6) with EADDR = the raw EA.
  This covers A = 0 **and** A = 1, in every addressing mode (MEMGET64,
  LDIDX64/LDIDX64U/LDIDX64R and the short forms, MEMSET64/MEMSET64RR,
  STIDX64*, absolute forms), in both modes. The access does not happen.
- **Exempt:** stack operations (PUSH, POP, CALL, RET, ENTER, LEAVE, LEAVERET,
  IRET), MEMGET32 and DCBZ. LR/SC already trap regardless.
- **MAL64 = 0.** Unchanged: a 64-bit access uses the doubleword aligned down.
- **Feature bit.** `CPU_FEATURES[23]` = 1 means STATUS.MAL64 is implemented.

## CPU_FEATURES additions

| Bit | Meaning |
|---|---|
| `[22]` | DCBZ (§1) |
| `[23]` | STATUS.MAL64 (§4) |
| `[24]` | `ETH_RX_SLOTS` (§3) |

## Implementation notes (§1 and §4, 2026-11-11)

Decisions taken while implementing the frozen text above:

- **MODE = 01 neighbours.** §1 says any other MODE with `[13]` = 1 is
  ILLEGAL. For MODE = 10 and 11 that holds (those LEN = 01 words were
  already illegal). MODE = 01 is the existing v3 short store / load, whose
  `[19:12]` is the imm8 displacement, so `[13]` = 1 there is an ordinary
  displacement bit and keeps its meaning (backward compatibility). The
  exact DCBZ word is `0x5F00_2000 | rs1 << 4`; every other class 6/7
  MODE ∈ {00, 10, 11} word with `[13]` = 1 is ILLEGAL. An emulator that
  rejects MODE = 01 words with `[13]` = 1 would reject existing code.
- **Trace.** The retire record of a DCBZ is a write of the line's first
  doubleword: address `EA & ~31`, byte enables `ff`, data 0 (tb_vsim /
  tb_soc: `wr=<base>/ff/0000000000000000`). With TE = 1 the address is the
  PA, as for every store's trace record.
- **Build gating.** DCBZ needs stage 1 (its MMIO fault): the core's
  `EN_DCBZ = EXC_EN && DCBZ_EN`, and `CPU_FEATURES[22]` =
  `CORE1_EXC_EN`. MAL64 needs MTSR STATUS: `CPU_FEATURES[23]` =
  `CORE1_EXC_EN`. Core 2 passes `DCBZ_EN = 0` (and has `EXC_EN = 0`).
- **Core.** DCBZ is a `U_STORE` with a decode flag (`zl`): EA aligned down
  to the line, data 0, byte enables FF, so the existing store machinery
  (permissions, posting, the store queue, ordering) applies. A new port
  flag `m_zl` (membus `zero_line`) marks the request. Every compare that
  tracks a store against loads or code is line-granular for a DCBZ: store
  → load forwarding / the hit-pipe block (`sf_a` / `sf_b` / `sf_c`: a load
  of any dword of the line forwards the zeros), the post check, the issue
  and completion SMC squashes, the NLB kills, and watchpoints (any dword of
  the line). The I-cache snoop compares both 16 B halves and, if either
  tag matches, clears both (the other half may then be an unrelated line
  at the same index: one extra I-cache miss, never a stale hit); the BTIC
  kills follow the same pair rule.
- **Reservation.** An ordinary store does not touch the LR/SC
  reservation on this single-hart core, so neither does DCBZ.
- **Cache.** `mem_read_write` latches `zero_line` with the write (acked at
  the latch, M10b). In CHECK a hit writes a zero line. A miss takes the
  write-miss path without raising the DDR read DV: WRITE_FETCH installs a
  zero line, dirty, in the victim way on its first cycle, through the same
  install write as a refill, and a dirty victim then goes out through the
  shadow writeback. (A first version installed in CHECK; that second write
  site on the tag / hit-pipe tag arrays made Vivado implement them in
  registers — Synth 8-4767, 248k LUTs. Every cache array keeps the write
  sites it had before DCBZ.) `CNT_ZERO_LINES` (0xF005_0070) counts both;
  they are also write hits / misses.
- **MAL64** is a flop in the core, written by MTSR STATUS as it commits
  (serializing), cleared by reset and `start`. The misaligned check uses
  EA[2:0] from a 3-bit add (was 2). Trapping 64-bit accesses become the
  existing converted misaligned op (cause 5 / 6, EADDR = raw EA).

## Status

- 2026-11-11: plan and specification agreed with the runtime session.
- 2026-11-11: §1 DCBZ and §4 MAL64 implemented in RTL on `perf-hw`
  (pipeline_core, mem_read_write, membus_if, bus_splitter, KlaussCPU,
  core2_subsys). Simulation only (Verilator tools/vsim directed and random
  tests, the xsim cache bench `perf/m5a/run_m10_cache.sh`); not yet built
  or run on the board. §2 and §3 not started here.
- 2026-11-11: §2 16 KiB I-cache implemented on `perf-hw` (pipeline_core,
  KlaussCPU, core2_subsys; ICACHE_INFO). Simulation only: tools/vsim
  `test` (1190 variants; also DEBUG_EN = 1, MPU_EN = 1), `dcbzdiff` (300
  seeds), `diff --base master` (identical traces), `excdiff`, `tlbdiff`,
  `tlbpage`, `mpudiff`, and on the SoC `socdiff --base 04b4055`, `socmmio`
  (with ICACHE_INFO), `soccrypto`, `socsd`, `socdcbz`, `soceth`. The
  perf/m5a programs (tb_soc, 04b4055 → this) change by −0.35 % to +0.12 %
  (their hot code fits in 8 KiB; UART identical except the
  timing-dependent lines of dhrystone and test_atomic). Not yet built.

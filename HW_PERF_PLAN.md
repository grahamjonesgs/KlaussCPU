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
  one cycle by reset, `start` and ICACHE_INV.  (Superseded by the timing
  closure below: one line-valid flip-flop per line plus the dword bits in
  LUTRAM.)
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
  **It exposed a latent EN_ST1 bug** (be78350's Linux early-boot panic,
  reproduced in tb_soc): a store that posts speculatively and then turns
  out not postable (`sq_spec`) squashes everything younger and refetches
  from `st_spec_nx` — the store's fall-through, also for a CALL's
  return-address push, so the callee was skipped. Before the guard a
  CALL's push was never refused (its stack slot is not code); with it,
  the callee's own line fill landing on the stack slot's I-cache index
  refuses it. The refetch pc is now the op's successor (`mem_succ`: a
  CALL's target); test `call_sq_spec_target`.
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

### §2 timing closure (2026-11-11)

The first 16 KiB build (`2e9dc8a`, `~/.cache/kbuild/perfhw3`) missed:
tier 1 WNS −0.267, tier 2 −0.185, 38,290 LUTs (+1.9k over `perf-hw-a`
`88acc21`, which met at +0.034 with 36,375), 25,254 FFs, 81 BRAM tiles.
Synthesis showed +2.3k logic LUTs: the 2 × 1024 per-dword valid
flip-flops, their next-state decode (install, snoop clear, flash clear)
and the 512:1 read muxes.  The worst paths were mostly the core's
long-standing zero-margin families (EA adder → `mem_hok`, the bus ready →
`pos`, the IRQ vector → `pc` enable, `ex_a` / `mem_result` forwards)
pushed over by that area; `ic_tag` → `if_req_dw` enable (−0.115) was
directly the I-cache.  Every fix below is a design change with identical
retire traces **and cycle counts** (`run.py diff --base HEAD` before each
commit); the full sim set passed on each.

| Build | Commit | Change | Tier 1 WNS | Final WNS / WHS | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| `perfhw3` | `2e9dc8a` | 16 KiB as first written | −0.267 | −0.185 / — | 38,290 | 25,254 | 81 |
| `perfhw4` | `bc527b1` | line-valid FFs + dword bits in LUTRAM | −0.105 | +0.005 / +0.017 (tier 2) | 34,601 | 23,966 | 81 |
| `perfhw5` | `445631a` | + fill registers off the hit enable; DELAY down-counter | −0.371 | −0.362 / — | 34,398 | 23,887 | 81 |
| `perfhw6` | `30eeab2` | + fetch tag in LUTRAM | −0.125 | −0.060 / — | 35,272 | 24,012 | 80.5 |
| **`perfhw7`** | **`be78350`** | + MUL WB forward selected a stage early | **+0.033** | **+0.033 / +0.024 (tier 1)** | 35,259 | 23,939 | 80.5 |

- **`bc527b1` valid bits.** `ic_lv` (one flip-flop per line: set by an
  install, cleared by a snoop and by the flash clear) and `ic_dv` (LUTRAM,
  1024 × 2: which dwords, written only by the install, which always writes
  both and sets `ic_lv`).  vhi = lv ∧ dv[1], vlo = lv ∧ dv[0] and
  vhi ∨ vlo = lv exactly, so the snoop and the lookup read one bit per line.
  Half the valid flip-flops and their decode: −3.7k LUTs placed, below
  `perf-hw-a` (the flash clear now fans out to 1024 flip-flops, as at
  8 KiB).  The generation-counter / BRAM redesign was not needed.
- **`d64006e` fill registers.** `if_req_dw`, `r_ifq_p1`, `if_rebase`,
  `if_tghi`, `if_fflt`, `if_fmiss` and `if_fake` are read only while a
  fill is in flight, so they now load on every cycle without one (enable
  `!if_xc`) instead of at the miss issue (enable = the issue condition,
  i.e. `!ic_hit`: the `ic_tag` → compare → 170-fanout enable path).
- **`445631a` DELAY.** `perfhw4`'s single tier-1 violator was `dly_cnt` →
  the 45-bit `dly_cnt >= dly_max` carry chain → `ex_busy`.  It is now a
  down-counter with a registered zero flag (DELAYR 0–3: identical cycle
  counts).
- **`30eeab2` fetch tag in LUTRAM.** `perfhw5`'s whole failing set was the
  hit path: the `ic_tag` RAMB18's clock-to-out plus a ~1.4 ns route out of
  the RAM column (four RAMB36 for `ic_data` spread the RAMs) → compare →
  `ic_hit` → the IFB load enables (−0.37).  `ic_tag` is now distributed RAM
  (RAM64M × 96, ~0.4k LUTs, one RAMB18 fewer) read into `ic_rd_tag` as
  before, so the compare starts at a flip-flop.  The snoop shadow
  `ic_tagff_e/_o` stays block RAM.
- **`be78350` MUL forward.** `perfhw6`'s worst was a direct MUL's WB
  forward: `wb_mhi` (fanout 90) → half select → W extension → forward
  muxes → `ex_a`.  `mul_r2` registers that select from `mul_pipe1` and
  `mem_d` on the MEM → WB edge (a MUL never waits in WB, so it equals
  `mul_w` whenever WB holds one — asserted); `wbz_mul` reads its byte
  flags.
- **Release bitstream** `~/.cache/kbuild/perf-hw_be78350.bit` (md5
  `4241b554…`), build dir `~/.cache/kbuild/perfhw7`, standard
  `build_fast -resynth` flow, **tier 1 met: WNS +0.033, WHS +0.024**, 8 min
  37 s.  35,259 LUTs (3,476 as memory), 23,939 FFs, 80.5 BRAM tiles, 19
  DSPs; every cache array RAM-inferred (no Synth 8-4767).  No alternate
  placement was used.  The worst remaining path is `m_addr` → qspi_ctrl's
  register enables (+0.033).  The core's own near-critical families
  remain (in `perfhw6` after tier 2: `ex_b_zero` from the MEM shifter
  −0.018, the ID decode → `ex_d.sub` −0.023, `mem_d.uop` → the multiplier
  inputs −0.010); they are the next candidates if a later feature tips
  them over.
- **Tag-shadow guard (`2a2ffb3`) kept.**  It never appeared on a near-
  critical path in these builds, so it stays as the defensive measure it
  was written as.
- **Not changed:** `ICACHE_INFO`, the miss behaviour (`ic16_*` pass with the
  16 KiB counts), the snoop's tag compare.

### §2 timing after the CALL-squash fix (2026-11-11)

`c1874cf` (`mem_succ`, the squashed CALL push refetching at the target)
built at tier 1 WNS −0.298 (`perfhw8`; tier 2 AggressiveExplore +0.010,
met, and that bitstream boots Linux on the board).  `mem_succ` itself is
nowhere near critical (+2.07; `st_spec_nx` +1.75): the one-register change
re-synthesised and re-placed the core, and two long-standing families
went over — the IF→ID latch predecode (`pos` → `w_op` → `w_dec.uop ==
U_EI` → `w_eiser` → `id_ser`: 9 LUT levels at +0.336 in `perfhw7`, 12
levels at −0.298 in `perfhw8`, the uop compare sharing the whole decode
tree) and the bus ready (`r_pipe_owns` → the `cpu_mem.addr` ownership
mux → the splitter's MMIO decode → `cpu.ready` → `wb_wait` / stall →
`ex_a` / `ex_b` / `pc` / multiplier inputs, −0.061 … +0.04).

`a5d20f2` (trace- and cycle-identical to `c1874cf`):
- `w_eiser` matches EI on the op's bits (exactly `f_decode_raw`'s U_EI
  case, asserted) instead of through the uop encoding.
- the core's `m_ready` is rebuilt in the top from its own registered
  `m_addr[31:28]` (MMIO → the splitter's registered ready, new port
  `o_ready_q`; else `dram.ready`) and `r_pipe_owns` — equal to
  `cpu_mem.ready && owns`, without the ownership mux and splitter decode.

| Build | Commit | Change | Tier 1 WNS / WHS | Final | LUTs | FFs |
|---|---|---|---|---|---|---|
| `perfhw8` | `c1874cf` | `mem_succ` | −0.298 / +0.015 | +0.010 (tier 2) | 35,054 | 24,058 |
| **`perfhw9`** | **`a5d20f2`** | EI raw match; ready off the mux | **+0.016 / +0.024** | tier 1 | 35,144 | 23,952 |
| `perfhw10` | (not committed) | + shift byte-zero flags (EX side) | −0.485 / +0.019 | −0.277 | 35,389 | 24,169 |
| `perfhw11` | (not committed) | + byte-zero flags (MEM side), IRQ vector ≠ 0 flags | −0.257 / +0.015 | −0.257 | 35,338 | 24,057 |

- **Release bitstream** `~/.cache/kbuild/perf-hw_a5d20f2.bit` (md5
  `b2c79baa…`), build dir `~/.cache/kbuild/perfhw9`, standard
  `build_fast -resynth`, **tier 1 met: WNS +0.016, WHS +0.024** (no tier
  2, no alternate placement), 11 min 35 s.  35,144 LUTs (3,476 memory),
  23,952 FFs, 80.5 BRAM tiles, 19 DSPs; no Synth 8-4767.  Not yet run on
  the board.  Short of the +0.02 margin aimed for.
- `perfhw9`'s remaining worst: `r_shhi` → the MEM shifter's 64-bit result
  → zero reduce → `ex_b_zero` (+0.016, L15; the DIV-by-zero forward, the
  `ex_b_zero` family above), then +0.089 (`r_bt_wbase` → `bt_lidx`, a
  pure route), `ex_a` → `mem_result` +0.094.
- **Tried, not kept.**  Two further identical-behaviour fixes removed
  their targets but the re-placed builds missed elsewhere: (1) the shift
  result's byte-zero flags (stage 1's per-byte zero flags moved by the
  byte shift `r_shhi`, asserted equal to `f_bz(w_sh2)`) for `ex_b_zero` and
  `wb_zero`; computed in EX (`perfhw10`) they loaded `ex_a` and the build
  fell to −0.485 (`id_ser` −0.485 via the full-decode legality /
  `id_ras_ret` tree, the IRQ vector's `!= 0` → `w_irq_ready` → `pc`
  −0.406, the EA adder → `mem_h` −0.353); (2) the same flags from the
  stage 1 registers in MEM plus `r_ivec_nz` (each vector's `!= 0` written
  with the table) (`perfhw11`): those paths gone, but the EA adder →
  store-forward compare → `mem_h` / `mem_fq` (−0.257, L19), `mem_d.uop` →
  the ID-latch / BTIC enables (−0.158) and `wb_fmask` → `pc` CE (−0.131)
  went over.
- **Reading.**  The core has five or six independent families (ID
  predecode, IRQ → `pc`, bus ready → stall, EA → store forward, the
  stall → ID / BTIC enables, the shifter → zero) within about ±0.3 ns of
  zero; which one fails is decided by placement, and tier 1 swings by
  ±0.4 ns between builds of near-identical netlists.  A dependable
  +0.02 needs several of them restructured together, not one per build.

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

## 5. I-cache 64 KiB direct-mapped, 32 B lines (2026-11-14, branch `icache`)

**Why.** On native clang under Linux, instruction-fetch miss cycles were
51–60 % of all cycles on the 16 KiB / 16 B I-cache.  The runtime session's
emulator sweep (klausscc `f8f9d95`, the NLB modelled from `e242c1d`) chose
64 KiB direct-mapped with 32 B lines: demand misses −65 % / −58 % on deflate /
cxx, bytes filled −31 % / −16 %; 2-way was not worth its timing risk.

**Geometry.** `pipeline_core` takes `IC_LINES` and `IC_LB` (16 or 32 bytes
a line); core 1 is 2048 × 32 B (`CORE1_IC_LINES` = 2048, `CORE1_IC_LB` = 32
in `KlaussCPU`; 1024 × 32 B = 32 KiB also elaborates and passes).  Index
VA[15:5], tag PA[31:16] — at 64 KiB the tag is the PFN itself, and the
index is the page offset, so VIPT needs nothing new (MMU_PLAN §11.1).
`IC_LB` = 16 is the §2 design unchanged: core 2 (512 × 16 B) passes
`IC_LB(16)` and its netlist is the same; `1024 × 16` is cycle-identical to
master `e242c1d` (below).

**ICACHE_INFO** (`0xF005_0018`, `CACHE_INFO`'s format) already has a
line-size field (`[31:24]`): it reads `0x0001_0000_2008_0001` (64 KiB,
32 B lines, 2048 sets, 1 way).  No format change.

### §5 design

- **Sub-lines.**  The front end — the 2-dword window, the two 16 B NLB
  entries, the prefetch, the BTIC windows and the M12 position tracker —
  keeps working in 16 B sub-lines (`ic_look_idx`, `ic_rd_idx`, `pf_line`);
  only the arrays are indexed by the 32 B line (`ic_look_li`).  The
  registered read holds the whole line, so a lookup of either half of the
  line just read is current at once (`ic_rd_current` compares lines), a
  hit serves the port's own triple for a 32 B line (the dword at
  `miss_dw[1:0]` and the next one unless it is dword 3 — on 16 B lines an
  odd dword had no successor), and a prefetch takes the half `pf_line[0]`
  (`pf_rd_half`).  With NLB2 the prefetch still runs one 16 B entry, then
  the next, ahead of the window: one new 32 B line ahead; the second
  half of a line it has just read needs no new RAM read.
- **Whole-line installs.**  A fill still asks the port for `miss_dw` and
  serves the window from the port's answer (the D-cache's early restart on
  a miss).  The I-cache installs the whole line from the D-cache's **line
  answer**: `mem_read_write` registers the 32 B line of every CPU read it
  answers (`o_cpu_line`, `o_cpu_line_v`): with the ready on a hit (CHECK),
  or on a miss when the line has arrived (READ_WAIT's install — after the
  early-restart ready).  It goes straight to the core (`m_line` /
  `m_line_v`, no splitter select).  The core installs the first line
  answer at or after its fill's ready (the cache is in order, so an earlier
  request's answer always comes before that ready, a later one after).
  Between the ready and the line (`lp_v`), the fill's own registers
  (`if_req_dw`, `if_tghi`) hold its address, and no new fill issues
  (`!lp_v`, and one cycle more after the install, `li_q`, so a held lookup
  re-reads the line it missed instead of fetching it again).  MMIO and
  fake fills install nothing.
- **Writes racing an install.**  A store issued after the fill's ready
  reaches the D-cache after the line was read.  So (a) the install is
  dropped when a write to its PA line is on the port or in the store queue
  as the line arrives (`lp_wrh`, registers only); (b) a store to the
  pending line's PA is a snoop hit at issue (`sn_rc`): it clears that index
  (covering an install landing on the store's own issue edge), kills the
  BTIC entries of the line (a CALL's capture of the window served by the
  fill — `ic32_smc_btic` fails without it), and does not post.
  ICACHE_INV, `start` and debugger resume drop a pending install (a line
  read before the fence).
- **Valid bits: LUTRAM, one per line, flash clear by sweep.**  Every
  install writes the whole line, so one bit per line says everything (no
  per-dword presence).  2048 flip-flops with a one-cycle clear (the §2
  design scaled) would have been ~3k LUTs of decode and read muxes and a
  2048-way clear fanout — §2's tier-1 family.  Instead 32 banks of 64 × 1
  LUTRAM (`g_vl`: RAM64M × 2 each, 256 LUTs), the bank = the index's high
  bits, one shared write address and a write enable per bank, two
  asynchronous reads (the lookup's, registered; the snoop's at `sn_pra`).
  One write a cycle, by priority: the **sweep** (every bank at `ic_swa`, 0),
  a **snoop clear** (`snoop_q`), a **valid set** (an install's, or a
  deferred one, `vs_pend`).  A set that meets a clear of the same line is
  dropped; one that meets another clear, or the sweep, waits (the latest
  one).  The flash clear (reset, `start` / resume, ICACHE_INV) is a
  64-cycle sweep during which every lookup misses; it is skipped when no
  bit has been set since the last one (`vl_any`; configuration leaves the
  LUTRAM 0).  Lines filled during a sweep miss once per 16 B half.
- **Snoop.**  One tag shadow, `ic_tagsh` (2048 × 16, one RAMB36), read at
  `sn_pra` = the store's line index; a store and a DCBZ each cover exactly
  one I-cache line, so the §2 even / odd banks are gone and `f_snm` is an
  equality.  Stores to the other half of a line holding code now drop the
  whole line (it is refetched on next use).  The tag-shadow guard (`ri_v`)
  tracks the line installs.
- **BTIC.**  Entries record 32 B line indices; slot 1 is kept when it is in
  the same 32 B line (base dword ≠ 3, was: even base).
- **Unchanged.**  The micro-ITLB, its refill and the fake fills; the
  walker / TLBWI discards (an install in flight still lands, physically
  tagged); the window / NLB / ID / EX SMC checks; DCBZ's line semantics.

### §5 deferred §11.9 fix

`f_sf_ln` / `f_sf_dw` take `ex_ta`, `ex_tb` and the carries as arguments
(MMU_PLAN §11.9's deferred note; netlist identical).  The xsim SoC run of
`perf/m5a/hello` (`run_m5d_soc.sh`, which also gains `sd_native.sv`) now
halts with the exact UART text; the same tree with the old functions
halts after 53,187 instructions with a corrupted UART stream, as noted
there.

### §5 verification (tools/vsim)

- **Cycle identity of the 16 B path.**  `VSIM_IC=1024x16` (the 16 KiB /
  16 B I-cache) against master `e242c1d`: `diff` 40 seeds × latency 1 / 3 /
  7 with `+TRACECYC` (every retire stamped with its cycle: identical), 30 ×
  2 / 5 with IRQs, `+HIT=70`, `DEBUG_EN`, `MPU_EN`, `EXC_EN = 0` (core 2's
  shape); `socdiff` 30 seeds and 20 with a 64 B span (identical traces and
  PIPE_RUN cycles); and all 1379 master tests with identical cycle counts.
- **New knobs.**  `VSIM_IC=<lines>x<bytes>` (tb_vsim and tb_soc);
  tb_vsim answers each DRAM read with its line (`m_line_v`), with the
  ready or `+LDLY` (+ random `+LDLYR`) cycles later, the line as it was
  read, holding later DRAM requests meanwhile (MMIO passes); a
  non-coherent writer at `0xF0FE_0000` / `_0008` (poke address / dword);
  `+ICDBG` prints misses, installs, drops, sweeps and BTIC events; the
  `.out` file has `ic_inst`, `ic_drop`, `ic_vdefer`, `ic_sweeps`; expected
  values may be ranges.
- **Directed tests** (`ic32_*`, all also run at 1024 × 32 and 1024 × 16):
  `ic32_fill_dw` (a cold line entered at each dword, then from its start:
  one miss a line); `ic32_prefetch` (straight code through 32 B lines and a
  64 KiB page boundary, TE = 0 and TE = 1 with non-adjacent frames and a
  decoy frame; a warm pass costs 80 cycles for 66 increments, 83 on 16 B
  lines; TE = 1: 92 / 94); `ic32_snoop_half` (a 64 / 32 / 8-bit patch in
  either half; a data store into the other half of a code line drops the
  line); `ic32_dcbz` (any EA in the line, code in either half);
  `ic32_icinv` (a non-coherent writer: old code until ICACHE_INV, the new
  after; a fence meeting a pending install whose line arrives after the
  sweep — fails if the fence keeps the install); `ic32_smc_pending` (a cold
  function patching its own line before its line answer: fails without
  `lp_wrh`); `ic32_smc_btic` (above: fails without the pending-line snoop);
  `ic32_remap` (TLBWI and walker remaps of a code page, installs pending
  at the discard); `ic32_loop48k` (below).  The §2 `ic16_*` tests and
  `ic_counters` take geometry-dependent counts.
- **Large footprint**: a 48 KiB loop, I-cache demand misses after 1 / 2 / 4
  passes — 64 KiB / 32 B: 1538 / 1538 / 1538; 32 KiB / 32 B: 1538 / 2562 /
  4610; 16 KiB / 16 B: 3076 / 6148 / 12292 (each equal to a direct-mapped
  model of the geometry over the loop's lines).
- **Random programs** against master (traces identical; tb_vsim at a fixed
  memory latency, so every saved fill counts fully): `diff` −21 %
  cycles, with IRQs −21 %, `+HIT=70` −22 %, `socdiff` (the real D-cache)
  −17 % (−18 % with a 64 B data span) — these short programs are mostly
  cold misses, half as many with 32 B lines.  The rest of the set
  (`socmmio`, `socwalk`, `socx`, `socdcbz`, `tlbdiff` incl. `--walk`,
  `--absent`, `--span`, `--irq`, TLB_N 4 / 8, `tlbpage`, `dcbzdiff` 200
  seeds, `mpudiff`, `excdiff`, `dmtest`; `test` with DEBUG_EN, MPU_EN,
  TLB_N = 16, TLB_WALK = 0, and with `+LDLY=3` / `+LDLY=1 +LDLYR=12
  +HIT=50`) passes.  Under `+LDLY` two count/timing-only tests differ:
  `exc_di_irq` livelocks at its fixed 60-cycle IRQ period when every read
  is slower (master does the same at `+LAT=8`), and `ic16_icinv.1`'s
  sweep-dependent miss count (517 / 518) is specified at the default timing.
- **Mutation checks**: dropping ICACHE_INV's `lp_v` clear fails
  `ic32_icinv` (a late line answer); dropping `lp_wrh` fails 18 SMC
  tests; dropping the pending-line snoop (`sn_rc`) fails `ic32_smc_btic`.

### §5 build (standard `build_fast -resynth`)

A synth-only check came first (no Synth 8-4767; `ic_data` 14 RAMB36 + 1
RAMB18, `ic_tagsh` 1 RAMB36, the valid banks 32 × 2 RAM64M, `ic_tag`
192 RAM64M).

| Build | Commit | Change | Tier 1 WNS / WHS | Tier 2 | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| `tlbx_b8` (master) | `db67f3f` | — | −0.014 / +0.017 | +0.018 / +0.017 | 37,200 | 25,192 | 80.5 |
| `icache_b1` | `8939eb2` | 64 KiB / 32 B as first written | −0.176 / +0.023 | −0.011 (not met) | 36,944 | 24,615 | 91 |
| **`icache_b2`** | **`5bcf24e`** | the core's reset off the install; the install index unconditional | −0.047 / +0.021 | **+0.013 / +0.021** | 36,549 | 24,543 | 91 |

- **`icache_b1`.**  Tier 1's worst (−0.176, six of eight endpoints) was the
  new logic: the top's reset (`w_pip_hold_rst`, fanout 2225) → `li_now`
  (it carried `ce && !rst` for the valid-bit block) → `ev_icinst_idx`'s mux
  → the 64 BTIC kill compares → `bt_v`.  Fixed by design in `5bcf24e`
  (identical behaviour): `li_now` reads neither, and `ev_icinst_idx` is
  `li_idx` (registers) whenever lines are 32 B.  The rest was the
  long-standing shifter → `ex_b_zero` path, where tier 2 stopped (−0.011).
- **Release bitstream `5bcf24e`** (`~/.cache/kbuild/icache_5bcf24e.bit`,
  md5 `6a7daffb7c3f711803fdb8bf95f4a71f`), build dir
  `~/.cache/kbuild/icache_b2`: tier 1 −0.047 (two endpoints, the MEM
  shifter → multiplier-input forward), **tier 2 AlternateFlowWithRetiming
  WNS +0.013 / TNS 0, WHS +0.021 / THS 0**, 20 min 3 s; no alternate
  placement.  36,549 LUTs (3,849 as memory), 24,543 FFs, **91 BRAM tiles
  (+10.5 over master's 80.5)**, 19 DSPs.  −651 LUTs and −649 FFs against
  master: the 1024 per-line valid flip-flops and their decode went to 256
  LUTs of LUTRAM; +384 LUTs as memory (the 2048-entry fetch tag).  Worst
  paths left: the MEM shifter → `mul_pipe1` input (+0.013), the hit pipe's
  H2 `r_h_tag1` → `wb_hext` (+0.068) — none through the I-cache.  As for
  master, closure is in tier 2 and the margin is thin.  Not run on the
  board (the supervisor coordinates the board).

### §5 additions from the emulator sweep (separate commits)

The runtime session's sweep (klausscc `f8f9d95`) found two small items;
they are separate commits after the I-cache, so either can be dropped.

1. **`PERF_IC_MISS` (`3b96669`).**  `perf_ic[0]` mirrored the fill-issue
   condition, so with TE = 1 it also counted each cycle of a micro-ITLB
   refill (about 3 per micro-ITLB miss: roughly half the board's count)
   and fake fills.  It is now registered at the issue that raises
   `m_read_DV`.  New `PERF_ITLB_REFILL` (`0xF00D_0110`, MMIO_MAP): one
   event per micro-ITLB refill.
2. **Micro-ITLB, 4 entries tagged by TLB index (`112fc8b`).**  One entry,
   emptied by every TLB write (every walker refill included), gave 53–60
   micro-ITLB misses per 1k instructions; the model gave −30…55 % with 4
   entries not emptied by unrelated writes.  Each entry filled from a
   main-TLB hit carries that entry's index (`mmu_tlb` `f_idx`); a TLBWI or
   walk writing index i (`w_idx`, known at its discard edge) drops only the
   entries tagged i and the negative ones (a useg / kmap page the TLB did
   not hold); TLBINV, MTSR ASID / STATUS, start and resume drop all, so no
   entry outlives its main-TLB entry.  The drop happens on the discard edge
   (the next registered lookup must see it) and a refill on that edge is
   suppressed (it would read the old entry).  Refill into the entry holding
   the page and mode, else an empty one, else round-robin; no second
   request in the cycle after a refill (the old design refilled each miss
   twice).  The lookup is still registers → four compares → `r_look_*`;
   the I-cache tag compare reads registers only.  Tests: `ut_pingpong`
   (2–4 code pages + main: one refill per page, none after warm-up; 5
   pages thrash), `ut_walk_evict` (a walk to an unrelated entry and a
   TLBWI to another index drop nothing; a walk overwriting A's entry
   drops only A; a TLBWI remapping B takes effect on its next fetch), and
   `ic32_remap`.  `ic32_prefetch` with TE = 1: 20 → 6 refills.  TE = 0 is
   cycle-identical to `5bcf24e` (`diff` with `+TRACECYC`).

| Build | Commit | Change | Tier 1 WNS / WHS | Tier 2 | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| **`icache_b3`** | **`112fc8b`** | + items 1 and 2 | **+0.070 / +0.011** | — | 36,706 | 24,686 | 91 |

- **Bitstream `112fc8b`** (`~/.cache/kbuild/icache_112fc8b.bit`, md5
  `39fc03946d18cb1dbb6f7d2f26cf6c77`), build dir `~/.cache/kbuild/icache_b3`,
  standard flow, **met in tier 1** (no tier 2, no alternate placement),
  15 min 57 s, no Synth 8-4767.  The items cost +157 LUTs, +143 FFs.  Worst
  paths: the hit pipe's H2 (`r_h_tag` → `wb_hext`, +0.070) and, new, the
  valid bits' write enable (`if_req_dw` → `li_idx` → bank decode → a
  bank's LUTRAM WE, +0.071): if it ever fails, set every valid bit through
  the registered `vs_pend` path (one cycle later; `li_q` would then hold
  the lookup two cycles).  The tier-1 / tier-2 difference between b2 and
  b3 is placement, not the items (b2's violators were the MEM shifter
  forward).

### §5 review fix-ups (2026-11-15)

**The finding (major): a dropped install left BTIC entries no store could
kill.**  BTIC entries die by line index only (`ev_icinst`, the store snoop
`snoop_q`, ICACHE_INV / TLB discard, start / resume).  That is safe only
if every captured window's line sits in the snoop's tag shadow at the
entry's `bt_lidx` (or is the pending install, `sn_rc`), so that a later
store to it is a snoop hit.  A dropped install (`lp_ev && lp_wrh`) left
the shadow with the old line's tag and fired no `ev_icinst`: a capture
taken from the window the fill's early restart served, after the
dropping store's pending-line snoop, would have survived every later
store to its line — a snoop-coherent patch of a <= 16 B loop could then
run stale from a BTIC hit taken before the next fill's install.

- **Reproduction.**  `tools/vsim` gained an invariant check in tb_vsim
  (`btic_unc` in the `.out` file: cycles in which a valid BTIC entry's
  line is neither in the shadow at its `bt_lidx` nor the pending install;
  32 B lines, TE = 0).  The review's scenario (`ic32_drop_btic`: a cold
  16 B loop storing into its line's other half, `+LDLY` 4–16, patched
  without ICACHE_INV, re-entered under a late line) does **not** fail on
  `112fc8b`, and the check stays 0 there: the dropping write is always
  issued after the fill's ready (a fill needs an idle port and an empty
  store queue, a store issue needs no fill in flight), so it is a store
  posted speculatively (EN_ST1, or queued, EN_SQ) whose post check fails
  on `sn_rc`; the cycle after, its squash empties the window together
  with the snoop's BTIC kill, and a capture on that edge is blocked.  A
  non-speculative store issue during `lp_v` would have to chain onto a
  load's answer, which the D-cache cannot give while it installs the
  line.  The check also stayed 0 over every directed test at `+LDLY` 1 /
  2 / 4 / 6 / 9 / 12 (~10k runs), with EN_ST1 = 0 in a sim copy (~8k
  runs), and over 3,600 runs of a cold-line SMC fuzzer (patches of
  code words in other functions' lines, stores into the own line, loops,
  calls; `+LAT` 1–5, `+LDLY` 1–16, `+LDLYR`, `+HIT`; EN_ST1 on and off; on
  average 0.8 drops a run).  The one writer that broke the invariant on
  `112fc8b` is the **debugger**: its memory write does not snoop, so the
  CALL's capture from a fill whose install it drops stayed live and
  uncovered (`ic32_dbg_write`: `btic_unc` 9 / 11 cycles at `+LDLY` 12 /
  30) until the resume emptied the BTIC — harmless, but only through the
  resume.
- **Fix: install-invalid.**  A dropped install writes `ic_data`, `ic_tag`
  and `ic_tagsh` and fires `ev_icinst` exactly like an install; only its
  valid bit is written 0 (`g_vl`: the request is `lp_ev`, the data bit
  `!lpi`; it cancels a deferred set and is never deferred itself — when a
  clear or the sweep owns the write its bit is 0 already: the dropping
  store's `sn_rc` snoop cleared the index, nothing set it since, and a
  debugger write is followed by the resume's sweep).  Firing `ev_icinst`
  alone would not do: a capture after the drop would still be uncovered.
- **Why the invariant now holds for every copy.**  A window is filled from
  (a) a lookup hit — the line is installed, so in the shadow; (b) a fill
  F of line L — until L's answer `sn_rc` covers L (pending), and at the
  answer the shadow gets L whether installed or dropped, while
  `ev_icinst` kills every entry captured before at that index; (c) a
  BTIC hit — a copy of a covered entry; (d) the NLB or a slide — copies
  of valid I-cache lines.  The shadow at an index changes only with an
  `ev_icinst`, which kills every entry at that index, so a covered entry
  stays covered until it dies.  A pending install that never arrives is
  dropped only by ICACHE_INV, start and resume, which empty the BTIC.
  NLB / prefetch copies do not depend on the shadow: the prefetch captures
  only lines whose valid bit it read (an install-invalid leaves it 0), and
  the NLB entries are killed by the stores' line-address compares at issue
  and completion.  The window itself is covered by the dword compares
  (post check, completion squashes) and the EN_ST1 squash.
- **Minor: `PERF_ITLB_REFILL`** counts `it_ref && !tlb_psel &&
  !tlb_discard`, ut_refill's condition (a refill pending on a discard
  edge is not done; the lookup asks again).  tb_vsim now checks every
  cycle that `perf_itlb` equals the refills done (FATAL otherwise) and
  counts the dropped ones (`itlb_disc`); no test reaches that case (TLB
  writes and walks serialise the fetch), so the fix is by inspection.
- **Minor: `PERF_IC_FILL_CYC`** (`perf_ic[1]`) is `if_xc` or, registered,
  a fetch-miss cycle (`perf_stall[6]`) while `lp_v` or `li_q`, except a
  lookup's hit cycle (MMIO_MAP).  Straight code through 40 cold lines at
  `+LAT=3`: the bubble part (`PERF_IF_MISS` − fill) was 84 / 125 / 249 /
  577 cycles at `+LDLY` 0 / 4 / 8 / 16, now 84 / 123 / 123 / 123
  (`ic32_fill_cyc`).
- **Minor: clock enable.**  `pipeline_core` takes `CE_TIED` (default 1);
  `IC_LB` = 32 with `CE_TIED` = 0 is an elaboration error (the one-cycle
  `m_line_v` pulse is taken under `ce`, so one in a `ce` = 0 cycle would be
  lost and `lp_v` would stick), and a simulation assertion checks `ce`
  stays 1 when `CE_TIED` = 1.  Core 2 passes `CE_TIED(0)` with `IC_LB(16)`.
- **Tests for the review's gaps** (each also at 1024 × 32 and 1024 × 16):
  `ic32_vdefer_snoop` (a set deferred behind one store's snoop, met by a
  second store's snoop of its own line on the next edge — `vdrop`);
  `ic32_sweep_snoop` (a line installed during a sweep, then a snooped
  store into it; the return's line answers after the sweep, `+LDLYA`, so
  no other install replaces the waiting set); `ic32_inv_defer` (ICACHE_INV
  restarting a sweep while a set waits — `swgo`'s `vs_pend <= 0`);
  `ic32_sweep_restart` (fences 0–70 cycles apart, refills between);
  `ic32_dbg_write` (DEBUG_EN: a debugger write to code while the install
  is pending; requires `btic_unc` = 0).  tb_vsim gained `+DBG_MW` /
  `+DBG_MD` (a debugger memory write at the first halt) and `+LDLYA` /
  `+LDLYN` (one line's answer delay).
- **Mutation checks.**  `vdrop` = 0 fails `ic32_vdefer_snoop` (3 variants)
  and `ic32_sweep_snoop` (6); without `swgo`'s `vs_pend <= 0`,
  `ic32_inv_defer` fails (6); reverting the install-invalid (install only
  on `li_now`) fails `ic32_dbg_write` (`btic_unc`, 4 variants, DEBUG_EN);
  writing the drop's valid bit as 1 fails `ic32_smc_pending` (18),
  `ic32_vdefer_snoop` (12) and `ic32_smc_btic` (4).
- **Regression** (`bb23b01`; 2048 × 32, 1024 × 32 and 1024 × 16): `test`
  (also DEBUG_EN, MPU_EN, TLB_N = 16, TLB_WALK = 0), the `ic*` / `ut_*`
  tests at `+LDLY` 6 and 11 + `+HIT=50` and under DEBUG_EN, `diff` (incl.
  `--irq`, `+HIT`, `+LDLY`, DEBUG_EN, EXC_EN = 0), `socdiff` (incl. a 64 B
  span), `socmmio`, `socwalk`, `socx`, `socdcbz`, `tlbdiff` (all variants),
  `tlbpage`, `dcbzdiff` 200 seeds, `mpudiff`, `excdiff`, `dmtest`: all pass.
  Under a global `+LDLY` only count / timing-specified expectations
  differ: `exc_di_irq` and `ic16_icinv.1` as before, `ic_counters`'
  `ic_fill` (it now counts the line wait — at 1024 × 16 it differs under
  `+LDLYR` on `112fc8b` too) and `ic32_fill_cyc` under a random `+LDLYR`.
  **1024 × 16** is cycle-identical to `112fc8b` on all 1,703 tests both
  pass, and trace- and cycle-identical to master `e242c1d` on `diff`
  (`+TRACECYC`: 40 seeds × 1 / 3 / 7, `+HIT=70`, DEBUG_EN, MPU_EN,
  EXC_EN = 0; `--irq`) and `socdiff` (30 seeds, 20 with a 64 B span);
  against `e242c1d`, 112 variants of 35 TE = 1 tests differ in cycles,
  all since `112fc8b`'s micro-ITLB (not this round).

### §5 review fix-ups: timing

- **`icache_b4` (`c1834a8`, the fixes)**: no Synth 8-4767, the memories as
  before (82 RAMB36 + 18 RAMB18, the valid banks 32 × 2 RAM64M).  Standard
  flow **not met**: tier 1 −0.107 (two endpoints: the MEM shifter →
  `ex_b_zero`, `r_shhi` → `w_sh2` → a 64-bit carry-chain zero test, 17
  levels; and `wb_hok` → a TLB entry), tier 2 −0.023.  The valid-bit
  LUTRAM write enable — the near-critical path of `112fc8b` (+0.071) — is
  at +0.646 (its enable no longer depends on `lp_wrh`).  An alternate
  placement (ExtraPostPlacementOpt) met at +0.008 (cross-check only; not
  kept).
- **Design fix (`bb23b01`)**: `bz_mem`'s shift case (the forward into
  `ex_b_zero`) comes from byte-zero flags of the shifter's stage-1
  registers, selected per result byte by the stage-2 byte shift — no
  barrel and no carry chain after `r_shhi`.  Combinationally equal
  (asserted every cycle), CPI-free; `sh_bz_form` (every shift kind, both
  count forms, 32 / 64-bit and DIV consumers) fails under each of nine
  single-term mutations of the flag form.
- **`icache_b5` (`bb23b01`)**: standard `build_fast -resynth`, tier 1
  −0.138 (now the D-cache hit pipe's TLB → `r_h_tag`, the family that was
  `112fc8b`'s worst at +0.070, and two shifter forwards), **tier 2
  AggressiveExplore WNS +0.007 / WHS +0.050: met**, 15 min 43 s, no
  alternate placement.  Worst paths after tier 2: TLB → `r_h_tag` (+0.007),
  `ex_sp` → `mem_xsq` (+0.042), `m_addr` → `pc` (+0.051), the shifter →
  multiplier input (+0.053); the valid banks' data input +0.834; nothing
  of this round's logic is near-critical.

| Build | Commit | Change | Tier 1 WNS / WHS | Tier 2 | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| `icache_b4` | `c1834a8` | review fix-ups | −0.107 / +0.023 | −0.023 (not met) | — | — | 91 |
| **`icache_b5`** | **`bb23b01`** | + the shifter's zero test from byte flags | −0.138 / +0.050 | **+0.007 / +0.050** | 37,118 | 24,700 | 91 |

- **Bitstream `bb23b01`** (`~/.cache/kbuild/icache_bb23b01.bit`, md5
  `fe6a53c97092e2dfd782c0743f565fcd`), build dir `~/.cache/kbuild/icache_b5`:
  +412 LUTs and +14 FFs against `112fc8b` (3,849 LUTs as memory, 91 BRAM
  tiles, as before).  The margin is thin, as it was at `5bcf24e` (tier 2
  +0.013); the tier-1 misses of b4 and b5 are placement-sensitive core
  paths outside the I-cache.  Not run on the board.

### §5 review 2: BTIC captures across a line install (2026-11-15)

**The finding (M1, minor): a BTIC capture on an install edge was left
uncovered.**  The capture was blocked on a store-snoop edge, not on an
install edge; the install's kill compares the entries' old `bt_lidx`, so
an entry captured on that edge survived while `ic_tagsh[li_idx]` took the
installed line's tag.  Scenario: L pending, O at L's index (64 KiB apart)
read valid, O's window served, the capture of O's target landing on L's
`lp_ev` edge; a later store to O misses the shadow and a BTIC hit replays
the old code.  It applies to every 32 B install (16 B lines installed on
the fill's own edge).

- **It is a class, not an edge.**  The invariant (§5 review fix-ups) is
  that every valid entry's line is in the shadow at its index or is the
  pending install.  It also needs the entry's *branch* line: an entry
  dies with its branch's line too (a tag hit means "same branch, same
  target"), and with 32 B lines a refetch of a patched branch reaches ID
  before its line's install kills the entry.  And a 32 B install lands
  after the fill's ready, so a copy of the line it replaces (O) can be in
  the fetch side on and after its edge: the window, the registered read
  sampled on the install edge, an NLB entry, the branch in ID.  A capture
  from any of them was uncovered, as was every capture whose branch and
  target are two lines at one index (a call to a 64 KiB alias of its own
  line: they cannot both be in the shadow).
- **Monitor (M2).**  tb_vsim's `btic_unc` now checks both lines of every
  valid entry (target at `bt_lidx`, branch at `bt_blidx`), with TE = 1
  too: the PA lines are taken when the entry is written, from the
  micro-ITLB entry holding the page (a TLB discard or TE / ASID change
  empties the BTIC, so the translation is fixed for the entry's life;
  `btic_pau` counts entries whose page the micro-ITLB no longer held —
  0 in every TE = 1 test).  A run with `btic_unc` > 0 ends with a FATAL
  line, so every tb_vsim-based command fails on it (`test`, `diff`,
  `excdiff`, `mpudiff`, `tlbdiff`, `tlbpage`, `dcbzdiff`, `tecost`), and
  run.py exits non-zero on any violation.  tb_soc gets the same check as a
  sim-copy probe (`socdiff`, `socmmio`, `socwalk`, `socx`, `socdcbz`).
- **Reproduction** (each fails on `bb23b01` and passes after the fix; all
  also run at 1024 × 32 and 1024 × 16):
  `ic32_btic_inst_edge` (the review's scenario: L's line answer swept with
  `+LDLYA` / `+LDLYN` across the capture of O's window — on its edge, on
  the edge O's window is served, on the edge O's line is read; the CALL in
  L or in a warm B at another index; with and without the store into L
  that makes it an install-invalid): on `bb23b01` 66 of 144 variants
  fail the check and **6 end with r1 = 3 instead of 18** (the patched O
  never runs: the BTIC replays the old word); `ic32_btic_win_loop` (a
  loop running from a window across the install that replaces its line;
  4 of 16); `ic32_btic_nlb` (an NLB entry across the install, also from
  the second entry; 18 of 42, **4 with r1 = 3**); `ic32_btic_br_win`
  (the branch's line replaced while it comes from the window; 8 of 14);
  and `ic32_btic_br_alias` (a CALL to an alias of its own line, then
  rewritten while its line is out: 10 of 10, and in 6 the core's own
  assertion "BTIC base != target" fires — O's code would run at the new
  target).  The M1 edge itself is `ic32_btic_inst_edge`'s via form at the
  matching `+LDLYN` (the direct form has CALL and O at one index).
- **Fix** (`pipeline_core`, all under `IC32`; 16 B lines unchanged):
  - the install on the last edge is registered (`r_ii_v`, its index, its
    fill's VA tag); `f_iia(dw)`: dw's line is another VA line at that
    index — a copy of it taken earlier is no longer covered;
  - the registered read sampled on an install edge at its index is
    invalid (`ic_rd_ik`: a lookup re-reads, a prefetch gives up), and a
    prefetch capture on that edge at that index is written invalid
    (`pf_ik`); the NLB loses the install's index;
  - `ifb_unc`: the window may hold a replaced line — set the edge after
    the install (`f_iia` of either slot's line), cleared by a whole-window
    load (fill, serve, BTIC: their sources hold no replaced line after
    the above), sticky through slides and slot-1 top-ups; `id_bunc`: the
    op in ID came from such a window, or its line was replaced since;
  - the capture is blocked while `ifb_unc` or `f_iia(target)`; dropped
    if `f_iia(branch)`, or if branch and target are two lines at one
    index; a branch with `id_bunc` arms no capture; on the install's own
    edge the entry is written invalid (`bt_cap_ii` — only the valid bit
    reads the late `lp_ev`, not the capture's write enable, so the
    capture logic stays registers-only).  This replaces the review's
    "kill on the next edge": that left the entry valid for one cycle,
    which the monitor (rightly) counts.
- **Mutations** (`ic32_btic_*`, failing variants with one part off):
  `ifb_unc` 16, `ic_rd_ik` 10, `pf_ik` 2, the first NLB entry's kill 2,
  `bt_cap_ii` 2, `f_iia(target)` 2, `f_iia(branch)` 2, `id_bunc` at the
  redirect 16; the static alias rule with `f_iia(branch)` 72 (each covers
  the other's cases in these tests).  No directed test isolates the
  second NLB entry's kill (with `pf_ik` the second entry is only written
  at an install edge in the tests that reach it) or the `id_bunc`
  accumulation in ID (it needs a branch held in ID across an install);
  both are kept by the invariant's argument.
- **`btfuzz`** (new run.py command): random code in 4–8 functions placed
  in slots 64 KiB apart (many share an index), with tight loops, loads,
  calls and snoop-coherent stores rewriting functions' first words,
  under 7 line / memory timings (`+LDLY`, `+LDLYR`, `+HIT`) against the
  same program on a `BTIC_EN = 0` build, with the coverage check.  On
  `bb23b01`: 250 of 2,100 runs (300 seeds × 7 timings) violate the invariant (no
  register difference seen); after the fix 1,500 seeds × 7 timings, and
  1,000 more in each 32 B regression, clean.
- **Cost.**  Only alias cases change behaviour (a replaced line's copies
  are refetched, captures not taken); `diff` / `socdiff` against
  `bb23b01`: see the regression below.  An unrelated observation: the
  capture's `id_bt_idx == r_bt_widx` snapshot kill compares the *old*
  snapshot's index, so when a capture lands on the next instruction's
  ID latch and the previous snapshot was the capturing branch's, the new
  snapshot is dropped (a lost BTIC hit, never a wrong one; the tests
  above place a NOP to avoid it).

- **Regression** (`823b5ea`, every run under the new coverage gate;
  2048 × 32 and 1024 × 32): `test` (default, DEBUG_EN, MPU_EN,
  TLB_N = 16, TLB_WALK = 0; 1,900+ variants each), the `ic*` / `ut_*`
  tests at `+LDLY=6`, `+LDLY=11 +HIT=50` and (DEBUG_EN) `+LDLY=1
  +LDLYR=12`, `diff` against `bb23b01` (40 seeds × 1 / 3 / 7, `--irq`,
  `+HIT=70`, `+LDLY=4 +LDLYR=8`, DEBUG_EN, MPU_EN, EXC_EN = 0), `excdiff`,
  `tlbdiff` (plain, `--irq`, `--walk`, `--absent`, TLB_N 4 / 8, a 512 KiB
  span), `tlbpage`, `dcbzdiff` 200 seeds, `mpudiff`, `dmtest`, `btfuzz`
  1,000 seeds, `socdiff` (30 seeds, 20 with a 64 B span), `socmmio`,
  `socwalk`, `socx`, `socdcbz`: all pass, and the random diffs' cycle
  totals equal `bb23b01`'s (+0.00 % everywhere).  Under a global `+LDLY`
  only the count / timing-specified expectations listed above differ
  (`ic_counters`' `ic_fill`, `ic16_icinv.1`, `ic32_fill_cyc.1` under a
  random `+LDLYR`).  **1024 × 16** is cycle-identical to `bb23b01`: every
  test in the five configurations (1,864–1,962 variants each, identical
  cycle counts), `diff` with `+TRACECYC` (40 seeds × 1 / 3 / 7, DEBUG_EN,
  EXC_EN = 0) and `socdiff` 30 seeds.

- **Build** (`icache_b6`, `823b5ea`): the synth-only check first (no
  Synth 8-4767; 82 RAMB36 + 18 RAMB18, the memories as in `bb23b01`),
  then the standard `build_fast -resynth`: **met in tier 1, WNS +0.017 /
  WHS +0.024** (no tier 2, no alternate placement), 11 min 5 s.  Worst
  paths: a TLB entry → `mem_xsq` (+0.017), `r_base_p1` → `ifb_dw` (+0.062),
  QSPI (+0.063), the hit pipe's `wb_h` → multiplier input (+0.076), the
  TLB → `t1_pahi` / `mem_pa2hi` / `r_h_tag` (+0.076 … +0.093); none ends
  in the new logic (the capture enable stays registers-only; the late
  `lp_ev` reaches only the BTIC valid bits, the NLB valid bits and
  `ic_rd_lv32`).  38,318 LUTs (3,849 as memory), 24,903 FFs, 91 BRAM
  tiles, 19 DSPs: +1,200 LUTs / +203 FFs against `bb23b01`'s build, of
  which `pipeline_core` +849 / +130 — almost all in unrelated datapath
  (`ex_a`, `mem_result`, `m_wdata`, `ex_c`: duplication / retiming choices
  of this run; the new registers are ~30 FFs); post-synthesis the same
  RTL varies by ~900 LUTs when any one part of the fix is removed, so this
  is the tool's variance, not the fix's size.

| Build | Commit | Change | Tier 1 WNS / WHS | Tier 2 | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| **`icache_b6`** | **`823b5ea`** | review 2: BTIC captures across an install | **+0.017 / +0.024** | — | 38,318 | 24,903 | 91 |

- **Bitstream `823b5ea`** (`~/.cache/kbuild/icache_823b5ea.bit`, md5
  `b36021a5ae4343972d1ee7b176e20f81`), build dir `~/.cache/kbuild/icache_b6`.
  Not run on the board.

### §5 review 3: straddling branches, stale pending captures, 16 B lines (2026-11-15)

Two findings of the review of review 2 (both pre-existing since 2a, and
both seen as a wrong BTIC window, "BTIC base != target"), and three more
that the coverage check found once it ran with 16 B lines.

- **M1: a branch across a line end.**  An entry's kills compare its
  target's line (`bt_lidx`) and its branch's first line (`bt_blidx`).  A
  2-word JMP / CALL abs at a line's last word keeps its target word in the
  next line, so a store rewriting it snooped a line no entry names, and the
  next run of the branch hit the old target's window.  **Fix:** such a branch
  arms no capture (`id_strad`, in `id_unc_now`: the op's last dword is at
  most one past its first, so the line changes iff the index's low bit
  does — a 1-bit compare of two registers).  The reviewer's other option, a
  second branch-line index per entry, would add 64 × 11 flip-flops and two
  more kill compares per entry.  Cost: only 2-word branches at a line's last
  word (1 slot in 8 with 32 B lines) lose the BTIC; randprog and the SoC
  programs have none (their cycle totals are unchanged, below).
- **M2: a pending capture outlived squashes.**  `r_bt_pend` ended only at
  the capture, reset / start and `bt_cap_drop`; a snoop blocked the
  capture on its own edge only.  An older store rewriting `jmp T1` at P
  into a conditional branch to T3 squashed J (armed {P, T1}); the refetched
  P fell through to a JMPR to T1, whose window the still-pending capture
  paired with P; once the PHT said taken, P's BTIC hit ran T1's code at T3.
  ICACHE_INV (code written behind the core's back), exceptions, IRQs and
  TLB / ASID changes were the same.  **Fix:** the pending capture is the
  arm's own redirect's: it ends at the capture, at any other move of pc
  (`ev_pcz`) or older squash (`ev_ovr`, which also cancels an arm on its
  own edge), and at a snoop of its lines.  An arm on a capture's edge is
  kept (the review's minor case: the capture's clear used to drop it).
- **16 B lines** (the check ran only with 32 B lines before).  Three
  coverage holes, the M1 class at 16 B: (a) the branch-and-target-at-one-
  index drop rule was IC32-only (a CALL to a 16 KiB alias of its own line);
  (b) the NLB, keyed by address, kept a copy of a line an install had
  replaced — now an install drops the NLB entries at its index, as with
  32 B lines; (c) a fill at a 32 B line's dword 1 also returns the next
  dword, the other 16 B line's first, into window slot 1, but installs
  only its own line — a branch latched from that copy was captured with its
  line not in the I-cache, so a store to it snooped nothing, and after an
  eviction and a refill through the same path the entry replayed the old
  code (a wrong result).  Now `r_nx_*` remembers that dword until the next
  whole-window load or redirect and a branch from it is `id_bunc`.  All
  three are `!IC32` / `EN_BTIC` logic: none of it is in the release build.
- **Checks** (tb_vsim, and the tb_soc probe):
  - a line is covered only if it is in the shadow (or the pending install)
    **at an index the entry's kills compare** (`bt_lidx` / `bt_blidx`); the
    lines checked are the target's, the branch's and the branch's last
    word's (from the arm that wrote the entry: tb records each arm's
    `id_last_dw`; `btic_noarm` counts an entry with no arm, FATAL);
  - both line sizes (16 B: the shadow is `ic_tagff_e` / `_o`);
  - currency: `btic_stale` counts cycles a BTIC hit's window is not at the
    ID op's predicted target — FATAL, independent of the core's assertion;
  - `btic_chk` / `btic_hits`: entries and hits checked, printed for every
    command (`BTIC checks: N entries checked, M hits checked in R gated
    runs`), so a vacuous run shows; `diff` / `socdiff` print both
    revisions' BTIC hits;
  - every run fails on a FATAL line or the core's "BTIC base != target" /
    "pos0 unsound" (also when it stops the run); the base revision's runs
    in `diff` / `socdiff` / `socmmio` are not gated.
- **`btfuzz`** now has patchable branch sites: a forward branch over 1-4
  signature adds, 1-word (JMP, FBR on r5: always / never taken) or 2-word
  (JMP abs; a 2-word FBR, word 1 its compare value), placed at random, at a
  32 B or 16 B line's last word (a 2-word site then straddles) or in a
  loop; stores patch a site's target (the offset, or the JMP abs's address
  word — a label in its skip, or a far trampoline at the functions'
  indices) and its kind, from anywhere or right before the site runs.

**Reproduction** (the review's t_straddle / t_pendsnoop / t_pendinv, now
tests.py; failing variants on `823b5ea` → after, at 2048 × 32 / 1024 × 32 /
1024 × 16): `btic_strad` 24 / 24 / 24 of 24 → 0 (the JMP and the CALL abs
forms; the core's assertion fires in 8 / 8 / 24, the rest are caught by the
coverage check's kill-index rule); `btic_pend_snoop` 24 / 24 / 48 of 96 → 0;
`btic_pend_inv` (the non-coherent writer and ICACHE_INV, older than J,
with a long memory latency: the refetches during the 32 B sweep install no
valid line) 12 / 12 / 8 of 96 → 0; `btic_oddfill` 0 / 0 / 8 of 8 → 0 (16 B
(c), a wrong result); `btic_nlb_inst` 0 / 0 / 27 of 27 → 0 (16 B (b)); the
review-2 `ic32_btic_*` tests at 1024 × 16: 104 variants fail the check on
`823b5ea` (16 B (a)) → 0.  `btfuzz` (300 seeds × 7 timings) on `823b5ea`:
181 of 2,100 runs fail at 2048 × 32 and at 1024 × 32 (28 seeds; 11 stop on
the core's assertion, the rest fail the coverage check) and 833 at
1024 × 16 (119 seeds; 7 on the assertion); after: 0 of 7,000 (1,000 seeds)
in each geometry.

**Mutations** (failing variants with one part off; directed = `btic_*` +
`ic32_btic_*`, btfuzz = 300 seeds × 7):

| Part off | 2048 × 32 | 1024 × 16 |
|---|---|---|
| `id_strad` | `btic_strad` 24 (8 of them also the core's assertion), btfuzz 177 | `btic_strad` 24, btfuzz 441 |
| every new `r_bt_pend` clear (capture only, as before) | `btic_pend_snoop` 16, `btic_pend_inv` 12, btfuzz 4 | 48, 8, btfuzz 0 |
| `ev_pcz` and `ev_ovr` (snoop kept) | `btic_pend_inv` 12 | `btic_pend_snoop` 24, `btic_pend_inv` 8 |
| 16 B drop rule (IC32-only again) | — | `ic32_btic_*` 104, btfuzz 357 |
| 16 B NLB kill | — | `btic_nlb_inst` 27, btfuzz 49 |
| `r_nx` (`id_bunc` from it) | — | `btic_oddfill` 8, btfuzz 14 |

`ev_pcz`, `ev_ovr`, the snoop clear and the arm-edge `ev_ovr` alone: 0
everywhere — they overlap (every squash sets both `ev_pcz` and `ev_ovr`;
ICACHE_INV and a TLB discard set both; an SMC squash and the snoop both
see a patched branch that is in flight), and are kept as one rule: the
pending capture is the arm's own redirect's.  The arm-on-a-capture-edge
case changed no hit in any test or fuzz run.  The checks with the core's
assertions off (`BTIC base != target`, `pos0..2 unsound`): `id_strad` off
fails 24 of `btic_strad` on coverage and 8 on currency (each alone still
catches its share); all pending clears off: currency catches every failing
variant (28 at 32 B, 56 at 16 B), coverage only 12 / 24 — the stale entry's
lines are all in the shadow, so currency is the check for M2.

**Regression** (run on `af1a1c4` and again, with identical results, on
the final RTL `079d686` (tools `064cc78`); every gated run under the new
checks, in 2048 × 32, 1024 × 32 and 1024 × 16): `test` in the five configurations
(default, DEBUG_EN, MPU_EN, TLB_N = 16, TLB_WALK = 0; 2,115–2,215 variants
each); the `ic*` / `ut_*` tests at `+LDLY=6`, `+LDLY=11 +HIT=50` and
(DEBUG_EN) `+LDLY=1 +LDLYR=12` (only the known count / timing expectations
differ: `ic_counters`' `ic_fill`, and with 32 B lines `ic16_icinv.1` and,
under `+LDLYR`, `ic32_fill_cyc.1`); `diff` against `823b5ea` (40 seeds ×
1 / 3 / 7; `--irq` × 2 / 5; `+HIT=70`; `+LDLY=4 +LDLYR=8`; DEBUG_EN; MPU_EN;
EXC_EN = 0), `excdiff` (and `--irq`), `mpudiff`, `dcbzdiff` 200 seeds,
`tlbdiff` (plain, `--irq`, `--walk`, `--walk --absent 0.3`, TLB_N 4 / 8, a
512 KiB span), `tlbpage`, `dmtest`, `btfuzz` 1,000 seeds × 7, `socdiff`
(30 seeds, and 20 with a 64 B span), `socmmio`, `socwalk`, `socx`,
`socdcbz`: all pass.  (On `af1a1c4` `tlbpage` ran outside the gate — its
`import run` was a second copy of the module; fixed in `064cc78`, gated
since.)

| Cycles, `823b5ea` → `af1a1c4` / `079d686` | 2048 × 32 and 1024 × 32 | 1024 × 16 |
|---|---|---|
| `diff` × 1 / 3 / 7 (also DEBUG_EN, MPU_EN, EXC_EN = 0) | 169,092 = | 215,050 = |
| `diff --irq` | 263,362 → 263,361 | 334,382 = |
| `diff +HIT=70` | 154,538 = | 199,675 = |
| `diff +LDLY=4 +LDLYR=8` | 202,874 = | 251,089 = |
| `socdiff` / 64 B span (PIPE_RUN) | 38,442 = / 21,040 = | 46,497 = / 25,766 = |

BTIC hits (both revisions under the same check) are identical everywhere
except `diff --irq`: 883 → 882 at 32 B, 881 → 878 at 16 B (−0.1 % /
−0.3 %; a capture pending across an IRQ entry is dropped).  (randprog, behind
`diff` and `socdiff`, has no 2-word branch, so the straddle rule's cost is
not in these numbers; it is one slot in 8 for 2-word JMP / CALL abs and
2-word FBRs with 32 B lines.)

**Build.**  Synth-only check first (`af1a1c4`): no Synth 8-4767, 82
RAMB36 + 18 RAMB18 as in `823b5ea`.  Then the standard `build_fast
-resynth`:

| Build | Commit | Change | Tier 1 WNS / WHS | Tier 2 | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| `btic_b1` | `af1a1c4` | review 3 | −0.025 / +0.015 | −0.025 | 37,868 | 24,909 | 91 |
| `btic_b2` | `9165799` | + the access-fault register as two | −0.171 / +0.051 | −0.005 | 37,075 | 24,815 | 91 |
| **`btic_b3`** | **`079d686`** | + 8 TLB CAM key copies | **+0.062 / +0.014** | — | 37,519 | 24,846 | 91 |

- b1 missed on one endpoint, a TLB CAM key copy → `d_pok0` → `t_dn` →
  `ex_ufault` → `mem_xsq` (review 2's worst path, +0.017 in `icache_b6`;
  nothing in it is new logic).  Every one of the 10 worst paths but two was
  a TLB CAM path (key copy → page match → PFN / grants OR → `mem_pa2hi`,
  `r_h_tag`, `t1_pahi`, `mem_xsq`), route-dominated (~85 %).
- b2 (`9165799`): `mem_xsq` = `mem_xsa || mem_xsp`, two registers —
  `mem_xsa <= ex_fa` (the CAM-independent terms), `mem_xsp <= ex_fb &&
  !pok` — so the grants bit reaches a flip-flop through one LUT (b1's path
  went through the shared `t_dn` LUT).  The placement moved: 10 CAM
  endpoints failed in tier 1 (−0.171, CAM → `mem_pa2hi`), −0.005 after
  tier 2 (CAM → `t1_pahi`).  Kept: it removes a level from that path.
- b3 (`079d686`): the CAM's key copies (`q_a` / `q_b`) are 8 at 32 entries
  (were 4: each key bit drove ~17 loads, and that first route led every
  failing path).  Met in tier 1, **WNS +0.062 / WHS +0.014**, 12 min 16 s,
  no tier 2, no alternate placement; the CAM paths now start at +0.257
  (CAM → `r_h_tag`); the worst are the shifter's `r_shhi` → multiplier
  operand pipeline (+0.062 … +0.23), `r_pipe_owns` → the D-cache tag
  (+0.229), `ex_b_zero` (+0.250).  Both timing changes are
  behaviour-identical (`diff` / `socdiff` against the previous commit:
  identical traces and cycles; `test`, `excdiff`, `mpudiff`, `tlbdiff`,
  `tlbpage`, `tecost`, `socwalk`, `socx`: pass).
- Utilisation (b3): 37,519 LUTs (3,849 as memory), 24,846 FFs, 91 BRAM
  tiles, 19 DSPs — −799 LUTs / −57 FFs against `823b5ea`'s build
  (38,318 / 24,903); post-synthesis 35,165 LUTs at `af1a1c4` against
  35,428 at `823b5ea` (the review-3 logic is a few LUTs and no flip-flop
  with 32 B lines: `r_nx_*` is 16 B-only; the rest is run-to-run
  variance); the four extra key copies are 128 FFs.

- **Bitstream `079d686`** (`~/.cache/kbuild/btic_079d686.bit`, md5
  `20acd72938f9480b5df82303f4e0240f`), build dir `~/.cache/kbuild/btic_b3`.
  Not run on the board.

### Review of review 3 (2026-11-15)

- **Result**: no critical or major findings; board validation passed; QSPI
  written 2026-11-15 (`btic_079d686`).
- **Deferred minor 1 (latent, reasoned only).**  A branch that arms after
  a non-speculative store's snoop of its own line, and is captured before
  the store completes, records the old bytes.  Unreachable today: EN_ST1
  stores are speculative and are squashed at the snoop by `sq_spec`, and a
  non-speculative store is chained behind a load, so the branch arms
  first.  A probe over 4,100 non-speculative snooping stores saw 0 such
  arms.  Planned hardening for the next RTL build: a 1-bit "snoop
  outstanding until `smc_st`" that blocks `bt_cap`, or a testbench
  assertion.
- **Deferred minor 2 (16 B lines only).**  The 16 B NLB kill has no
  `pf_ik` equivalent, so a same-edge prefetch capture may stay valid.  Not
  constructible (0 in 354 `btfuzz` runs).
- **Monitor gaps.**  There is no content check of `bt_d0` / `bt_d1`
  against memory (planned); the `ba_*` arm lookup matches by tag only.
- **Gating fixes (tools only, no RTL change).**  (1) `tests.py`'s
  `import run` (`short_cmprvw`) made a second copy of the module, so its
  runs bypassed the BTIC gate; `main()` now registers this module as `run`
  for every command (as 064cc78 did for `tlbpage`).  (2) Vacuity gates:
  a command fails when BTIC hits happened but the coverage monitor checked
  zero entries and skipped none for an unknown PA (runs with no BTIC
  activity, e.g. `BTIC_EN=0` or tiny tests, have hits = 0 and are
  unaffected; aggregated over the command, since one short run may hit
  without a checkable entry).  `build_soc` now fails if the tb_soc probe's
  signal-name test fails on the current sources (it was a silent skip),
  and a probed `tb_soc` run that prints no `BTPROBE` line fails.
  `btic_pau` (entries skipped because TE = 1 and the PA is unknown) is in
  the printed summary and the tb_soc `BTPROBE` line.


## 6. Unified L2 (2026-11-15, branch `l2`)

**Why.**  HW performance item 3, approved by Graham.  The design point comes
from the runtime session's timed model (klausscc `442306f`,
`src/emu/cachesim/l2t.rs`: the board's L1s, I fills through the D-cache, a
blocking memory unit, one DDR transaction at a time, `rc` = 44 cycles from a
D-cache miss to its critical dword today): 128 KiB, 64 B lines of two 32 B
sectors, 4-way, a 2-entry write buffer, hit latency `h` = 6 gives −19 % /
−22 % cycles on deflate / cxx-hello, each cycle of `h` about one point
(`H4`).  The model assumes separate data ports for fills and hits and a
take-back of write-buffer lines (both below).

### §6 survey: where the L2 sits, and every path to DDR

- **Clock.**  There is no DDR clock crossing in the fabric to remove: since
  MEM_CLOCK_PLAN P3 / P4 the whole CPU runs on the MIG's 100 MHz `ui_clk`
  (2:1 PHY), so the D-cache ↔ MIG interface is one domain.  The ~44 cycles
  of a miss are the MIG controller and PHY.  The L2 is in the same domain;
  a hit skips the MIG.
- **The one funnel.**  `mem_read_write` holds the DDR arbiter: master A (the
  D-cache, which also serves every I-cache fill and every core access: the
  pipeline, the boot-ROM copy and UART loader FSM, the HCF dump, the
  debugger's memory access through the core), B (the blitter), C (core 2;
  `EN_CORE2` = 0 in the release build) and D (VGA scanout, read-only).  The
  arbiter's output is the only user of `ddr2_control`.  Nothing else reaches
  DDR: the native SD controller has no DMA (`SDM_DMA_ADDR` reserved),
  LiteEth uses its own slot SRAM (the CPU copies), QSPI and crypto are MMIO.
- **Coherence model (unchanged).**  Flush by software: the D-cache is
  write-back, DMA does not snoop it, and software uses `CACHE_CTRL` FLUSH /
  INVALIDATE (full-cache walks) around DMA and ICACHE_INV for DMA-written
  code.  The I-cache snoops core stores above the D-cache; DCBZ is a D-cache
  operation (no DDR read).
- **Decision: the L2 replaces `ddr2_control` below the arbiter**
  (`l2_cache.sv`, instantiated by `mem_read_write` when `L2_EN`; it drives the
  MIG UI itself).  Every master's DDR access goes through it, so the L2 and
  DDR are one memory for all of them, and the software contract does not
  change: flushing the D-cache makes data visible to DMA (it lands in the
  L2, which DMA reads), invalidating it after DMA makes the CPU re-read the
  L2 / DDR.  "Blitter output needs no flush" for VGA stays true.  No L2
  flush is needed by any existing path:
  - DMA **reads** (blitter, core 2: 16 B; VGA: 32 B) are served from the L2
    when the sector is valid (clean or dirty), else from DDR; they never
    allocate and never touch the PLRU (VGA's frames would flush the L2).
  - DMA **writes** (16 B, byte-masked) merge into a valid L2 sector with byte
    enables and mark it dirty; otherwise they are posted to DDR through the
    write buffer.
  - **Every DDR read first drains any write-buffer entry of its line** (the
    MIG is configured `Ordering = Strict`, so a read after the drained write
    sees it).  This is the "ordered correctly" option for a demand miss on a
    buffered line (the model takes the line back; measured cost below).
  - D-cache FLUSH / INVALIDATE walks write their dirty lines into the L2 (or
    the buffer): the DMA that follows reads them there.  DCBZ, the I-cache
    snoop, the debugger's accesses, the boot and UART loaders all work above
    the D-cache and are unaffected.  The I-cache's `ICACHE_INV` rule for
    DMA-written code is unchanged.
  - **The sub-line hazard** (MMIO_MAP "Cache controller") is unchanged: a
    D-cache line is still written back whole, now into a 32 B L2 sector.
- **Aliases** (review 1).  Tags cover PA[26:6] (all 128 MiB).  PA[31:27]
  alias PA[26:0] in DDR itself (`ddr2_control` drops PA[31:28], and the MIG,
  with RANKS = 1, ignores `app_addr[26]` = PA[27]), so the L2 folds them the
  same way: one line per DDR line whatever the alias, the write buffer and
  the MIG address on PA[26:6].  (Before review 1 an access with PA[31:27] ≠
  0 bypassed the L2, so an alias write could go to DDR under a dirty L2
  copy of the same bytes.)  MMIO never reaches `mem_read_write`.
- **Not inclusive, no back-invalidation**: the L1s are never told about L2
  evictions (none needed: L1 data is newer or equal).

### §6 design

- **Geometry** (`l2_cache` parameters `WAYS` 2 / 4 / 8, `L2_KIB`): 128 KiB,
  512 sets × 4 ways (256 × 8), 64 B lines of two 32 B sectors with a valid
  and a dirty bit each; tag PA[26:15] (4-way); tree pseudo-LRU (an invalid
  way first).  `KlaussCPU` parameters `CORE1_L2_EN` (1) and `CORE1_L2_WAYS`.
- **Arrays.**  Data: one memory per way of (sets × 2) × 256 bits, a sector
  per word in the D-cache's line layout; port A reads the sector of every
  way at once (BRAM latch + output register), port B writes one way with
  byte enables, from registers (a write is decided a cycle before it lands).
  Tags: one word per set {PLRU, way × {tag, V[1:0], D[1:0]}} (67 bits at 4
  ways), port A read with no output register, port B written by the front
  end only.  32 + 1 RAMB36.  Write buffer: 2 entries × (64 B in LUTRAM, a
  64-bit byte mask, the line).
- **Front end** (one request at a time; posted writes — a write is acked on
  its accept edge, like the D-cache's write-ack-at-latch).  While idle, the
  port-A address is the D-cache's request in its CHECK cycle (`i_spec_*`,
  its latched `r_computed_ddr_addr`): when that becomes a miss, the L2's
  tags are already read as its DV arrives.  Hit timeline (E0 = the D-cache's
  CHECK edge that raises the read DV): E0 the L2 arrays latch the address;
  cycle 1 the tags are compared (the request accepted), E1 the decode is
  registered and the data output register holds every way's sector; cycle 2
  the 4:1 way mux; E2 `o_mem_ready` with the 32 B sector; E3 the D-cache's
  `cpu.ready` with the critical dword (its fallback path, which also
  installs the line).  **h = 3** (the model's `h` = 6; without the
  speculative read — the L2 busy at E0 — 4).  An L2 miss issues the DDR
  command in the same cycle as `ddr2_control` did (no added latency); the
  critical burst first, the D-cache restarts on the early dword channel as
  before, then the rest of the requested sector; the other sector follows
  in the background (requests to the set wait for it).
- **Allocation.**  A read miss allocates the whole line (both sectors
  fetched: 4 BL8 bursts) — or, when the line is there without that sector,
  fetches the sector only.  A D-cache victim (32 B) that misses allocates
  its sector without a fetch (the other sector invalid), and one that hits
  the line writes in place.  A dirty L2 victim's dirty sectors are copied to
  a write-buffer entry (2 port-A reads) before the fill can overwrite them
  (asserted).
- **DDR engine** (one transaction at a time): a read of 1 / 2 / 4 bursts,
  the critical one first, assembling sectors and writing them through port
  B; or the drain of the oldest buffer entry (command + beat 0, then beat
  1, per burst with any byte to write; FIFO).  Priority: a demand read
  issues first; the buffer drains when the engine is idle — except (VGA,
  below) — and a full buffer drains first in effect, since an allocation
  with a dirty victim waits for a free entry.  I vs D priority does not
  arise: the D-cache serialises them (one memory port).
- **VGA scanout (real time).**  Its reads now pass the L2 lookup and could
  wait behind drains.  While VGA requests the bus (`i_rt_req` = its req) or
  any DMA master holds the grant, a drain starts only for the request in
  F_DEC that waits on the buffer (`w_fe_req`: a read miss of a buffered
  line, or a full buffer in the way of an allocation's victim copy or of a
  posted write) or for the WBINV walk waiting on it (`w_wk_need`: a dirty
  way to copy and the buffer full, or its final drain; review 2), and a
  demand miss fetches only its own sector (2 bursts, as `ddr2_control`).
  So in VGA's window the L2 starts DDR traffic only for a request in hand —
  its sector, the drain it is blocked on, (VGA's own) the stream fill
  below, or the drain the walk is blocked on — never a background drain or
  a whole-line fetch (review 1 M2; before, any front-end state but idle
  let a drain start, including the F_LOOK every VGA read passes).  **The
  walk counts as a request in hand** (the `L2_CTRL` write, review 2): left
  out, a VGA or DMA tenure held BUSY up for as long as it lasted.
- **VGA stream buffer** (review 1).  A VGA read (a wide DMA read) of sector
  0 of a line the L2 does not hold fetches the whole 64 B line (4 bursts,
  row hits); sector 1 is kept in a 256-bit register for the scanout's next
  read (its addresses ascend), which it answers at hit latency with no DDR
  round trip.  Any write accepted to that line (any master) drops it, also
  while sector 1 is still on its way; a line the L2 holds (a sector there
  may be dirty) never fills it.  Only while the L2 is enabled (review 2: a
  disabled L2 is a plain path to DDR; the buffer stays coherent meanwhile,
  as writes still drop it).  A stream hit counts as a DMA hit and its fill
  as a DMA miss, so a scanout of a framebuffer outside the L2 shows DMA
  hits = DMA misses.  It halves the scanout's DDR round trips:
  the margin numbers below.
- **`L2_CTRL`** (MMIO_MAP): write 1 = enable (instant: a disabled L2 holds no
  line); 3 = write back and invalidate, stay enabled; 0 = write back,
  invalidate, then bypass.  BUSY is set on the write's own edge, so the next
  read sees it.  **A write while BUSY is ignored** (the runtime session's
  driver polls BUSY; requested 2026-11-15).  The walk visits every set
  (tags read, each dirty way's sectors copied to the buffer, the tag word
  written 0), yielding to requests between sets and while the buffer is
  full; a copied way's dirty bits are cleared in the tag RAM at the copy,
  so a set walked again after a yield copies only ways dirtied meanwhile
  (review 1 M1).  The walk and the requests take turns: after a request
  accepted during the walk, the next set goes first (review 1).  A walk
  that finds the buffer full is a waiter like a request (review 2): its
  drain may start in VGA's / DMA's window and goes before a demand read;
  the slot it frees is the walk's (a posted write then waits for an empty
  buffer); and a write to the set it waits in is held off until the walk
  has copied it, so each way of a set is copied at most once.  During a
  walk nothing allocates, hits are served, and writes that miss are posted
  to DDR, so no data is lost or reordered.  A write to the set being walked
  waits until the walk has copied that set (the l2b delta review: a single
  write's worst wait ~48–96 cycles under adversarial traffic, 162–186 at 8
  ways; an SD DMA stream during WBINVs under VGA load: §7).  It ends when the buffer entries
  present when the last set was done have drained (writes posted after
  that are later writes: traffic that keeps the buffer busy cannot hold
  BUSY up).  A write of 0 / 3 to a disabled L2 only drains the buffer, so
  BUSY = 0 means DDR holds every write acknowledged before the walk's
  end.  CPU reset and
  program load enable the L2 (a disable walk then running ends as a plain
  write-back-and-invalidate).  `L2_INFO` = `0x0002_0000_4002_0004`,
  `CPU_FEATURES[26]`, counters `0xF00D_0118` … `0x0168` (I / D / writeback
  hits and misses, DDR read and write beats, write-buffer-full cycles, DMA
  hits and misses); the I / D split comes from a new port bit `m_if`
  (= the core's `if_xc`) through `membus_if.ifetch`.
- **`L2_EN` = 0** builds `ddr2_control` exactly as before (a `generate`);
  the only shared-RTL changes are dead without the L2: `membus_if.ifetch`,
  `pipeline_core.m_if`, the D-cache's shadow-writeback DV gap (a parameter:
  2 with `ddr2_control`, 0 with the L2, which needs one DV-low cycle), the
  `L2_CTRL` decode and the counters (constant 0, read 0 as before).

### §6 verification

**Benches.**
- **`tb_l2`** (Verilator, `sim_1/new/tb_l2.sv`): the L2 alone on the
  behavioural MIG, driven with the arbiter's protocol by every kind of
  master (the D-cache's 32 B reads with a critical dword and the
  speculative lookup — right and wrong —, its 32 B victim writes, 16 B
  masked DMA reads and writes, VGA's 32 B reads), `L2_CTRL` writes and the
  VGA-request input at random.  **Memory-consistency checker:** a reference
  memory takes every accepted write; every read answer and every early
  critical dword must equal it; after each write-back-and-invalidate the
  MIG's memory must equal it too.  Directed tests: `hit` (a miss fills the
  line; the same sector hits 2 cycles after the DV with the speculative
  lookup, 3 without; the other sector hits; an I-fill hit), `sector`
  (a victim allocates its sector with no DDR read; the other sector then
  fetches 32 B only), `evict` (WAYS + 2 dirty lines in a set: two whole
  lines drained), `wbhit` (a demand read of a line still in the write
  buffer waits for its drain; asserted not vacuous), `plru` (the victim
  against a reference tree-PLRU over 40 rounds), `dma` (the D-cache's
  dirty line read by DMA and VGA from the L2 with no DDR read; a DMA write
  merged into a valid sector and read back by the D-cache; a missing DMA
  write posted then read; **ordering**: a D-cache writeback then a DMA
  write of the same bytes, and the reverse, with the L2 on, off — both in
  the write buffer — and across both), `wbinv` (dirty lines in 240 sets,
  WBINV with D-cache and DMA traffic to walked and not-yet-walked sets
  during the walk; BUSY at once; DDR then equals the reference; every
  later read misses), `toggle` (disable with dirty data, a write of 1
  during the walk ignored, bypass traffic counts no lookups, DDR checked,
  re-enable, the program-load enable cancelling a disable), `range`
  (PA[31:27] ≠ 0: no lookup, no allocation).
- **tb_soc**: DDR is now the behavioural MIG (`mig_model.sv`: Strict order,
  `+MIGLAT` default 36 = the board's ~44-cycle miss, `+MIGJIT`, `+MIGRDY`,
  `+MIGREF`) behind the real `ddr2_control` or `l2_cache`, with the same
  consistency checker at the arbiter's port on every run (`FATAL
  consistency` fails the command).  **Baseline check (the testbench change
  hides nothing):** `CORE1_L2_EN` = 0 against `d7f3994`'s RTL under the same
  new tb_soc (`socdiff --base-tb-wt`): identical traces **and PIPE_RUN
  cycles** (30 seeds: 69,032 = 69,032; 20 with a 64 B span: 25,584 =
  25,584), and the whole SoC set passes with `VSIM_L2=0`.  The RTL of an
  `L2_EN` = 0 build differs from master only in logic with no load (the
  `ifetch` bit, `m_if`, the counters and the `L2_CTRL` decode) and in a
  parameter that keeps its old value (the DV gap, 2).
- New SoC commands: `socl2` (L2_CTRL: BUSY reads 1 on the instruction
  after a write of 0, a write of 1 during that walk is ignored, 400 stores
  and loads across every set during the walk, BUSY polled, then EN = 0,
  enable, the region's sum through the bypassed and the re-enabled L2;
  the blitter's FILL over bytes a D-cache writeback just put in the L2 (L2
  on), in the write buffer (L2 off), and the reverse order with a COPY that
  must read the L2's dirty data; the counters), `socbench`, `socvga`.

(Review 1 below replaced `range` by `alias`, made the DV tail the
default, and gave the MIG model DRAM timing; the results and VGA numbers
in this section are `fba47c0`'s on the earlier model, and review 1 gives
them again.)

**Results** (final RTL; every run under the BTIC gates and the consistency
checker):
- `tb_l2`: every directed test and 12 random seeds × 3,000 transactions,
  at 2, 4 and 8 ways, under four MIG timings (`+MIGLAT=4`; 36; 8 with
  `+MIGJIT=20 +MIGRDY=30`; 60 with `+MIGREF=200 +MIGRDY=10`): SWEEP.
- tb_soc, L2 on (4-way), also under `+MIGLAT=8 +MIGJIT=20 +MIGRDY=30`,
  `+MIGLAT=60 +MIGREF=500`, with 8 ways and with `VSIM_L2=0`: `socdiff`
  against `d7f3994` (30 seeds, 20 with a 64 B span: identical traces),
  `socmmio` (`L2_INFO`, `L2_CTRL`, `CPU_FEATURES` = `0x07F0_200D`),
  `socwalk`, `socx`, `socdcbz`, `socl2`, `socsd`, `soceth`: all pass.
  (`socsd` and `soceth` expected `CPU_FEATURES` without the walker's bit
  [25] and failed on master too; their constants are fixed.)
- Core regression (`pipeline_core` gained only the `m_if` output):
  `test` (default, DEBUG_EN, MPU_EN, TLB_N = 16, TLB_WALK = 0; 2,200
  variants each), `diff` against `d7f3994` with `+TRACECYC` (40 seeds × 1 /
  3 / 7: 169,092 = 169,092 cycles) and `--irq` (209,357 =), `excdiff`,
  `mpudiff` (MPU_EN), `tlbdiff`, `tlbpage`, `dcbzdiff` 200 seeds, `dmtest`,
  `btfuzz` 300 seeds × 7: all pass.

**Cycles** (tb_soc, `+MIGLAT=36`, L2 off → on in the same tree; identical
traces / registers):

| Program | L2 off | L2 on | |
|---|---|---|---|
| `socdiff` randprog, 30 seeds | 69,032 | 61,458 | −11.0 % |
| randprog, 64 B data span, 20 seeds | 25,584 | 24,613 | −3.8 % |
| randprog at `+MIGLAT=8 +MIGJIT=20 +MIGRDY=30` (vs `d7f3994` there: same DDR model only for L2 on/off) | 44,120 (`MIGLAT=8`) | 41,996 | −4.8 % |
| `data96k` (4 passes, load + store over 96 KiB) | 901,258 | 381,047 | −57.7 % |
| `code96k` (96 × 1 KiB functions, 3 passes) | 311,576 | 142,666 | −54.2 % |
| `chase112k` (dependent loads round a 112 KiB cycle) | 465,382 | 195,480 | −58.0 % |

randprog's footprint fits the L1s: its gain is the posted D-cache
writebacks and the 64 B fetch (a next-32 B prefetch).  The large-footprint
programs are the L2's case (pass 2 of a 96 KiB read: 10.0 cycles a load
including the loop, against 30.5 cold).  Latency checks: a dependent load
chain that misses both levels costs 48.11 / 42.01 cycles a load (128 B /
4 KiB stride) with and without the L2 — an L2 miss adds nothing.

**Hit latency, each extra cycle** (L2 off = 100 %):

| Hit | randprog | data96k | code96k | chase112k |
|---|---|---|---|---|
| **3 (design)** | **−10.97 %** | **−57.7 %** | **−54.2 %** | **−58.0 %** |
| 4 (no speculative lookup, `SPEC_EN` = 0) | −9.70 % | −56.4 % | −52.9 % | −56.2 % |
| 5 | −9.43 % | −55.2 % | −51.5 % | −54.8 % |
| 6 (the model's `h`) | −9.18 % | −54.0 % | −50.2 % | −53.4 % |

So the speculative lookup is worth ~1.3 points on randprog and 1.3–1.8 on
the large footprints, and each further cycle ~1.2–1.4 points there,
matching the model's "about 1 point a cycle".

**VGA scanout** (`socvga`: 640 × 480 RGB565, CPU traffic, WBINV walks;
starved display lines, from the scanout's `o_underflow` — the board's
`VGA_STATUS[63:48]`; worst fetch of a line in cycles, of 3,200):

| Load, MIG | L2 off: starved / worst fetch | L2 on: starved / worst fetch |
|---|---|---|
| heavy (a 512 KiB stride walk, load + 2 stores a step; WBINV every 1,500 steps), `MIGLAT=36` | 0 / 3,185 | **0 / 3,178** |
| heavy, `MIGLAT=36 +MIGREF=780 +MIGRDY=5` | 584 / — | 40 / — |
| heavy, `MIGLAT=44` | 832 / — | 777 / — |
| moderate (a 96 KiB walk: L2 hits; WBINV every 300), `MIGLAT=44` | 0 / 2,208 | **0 / 2,320** |
| moderate, `MIGLAT=44 +MIGREF=780 +MIGRDY=5` | 0 / 2,251 | **0 / 2,406** |

Under the adversarial load (every access misses both levels, dirty
victims everywhere) the scanout has almost no margin **without** the L2
(3,185 of 3,200 cycles at the board's latency; it starves at `MIGLAT` 40+
or with refresh-like stalls); with the L2 it starves no more than without
in every setting.  Before the VGA rule (no background drains and
sector-only fills while VGA fetches) the L2 build starved 2 lines at
`MIGLAT=36` and 47 after a first, drain-only version of the rule; the
whole-line fetches had raised DDR read traffic by 42 % in this load.  A
scanout-side fix for the pre-existing margin (more line buffering, or
pipelined reads) is outside this item.

### §6 mutations

`tools/vsim/l2mut.py`: one textual change of `l2_cache.sv` each, built into
`tb_l2`, run on every directed test and 4 random seeds under two MIG
timings (stopping at the first failing run):

| Mutation | What it breaks | First failing run |
|---|---|---|
| `wbm` | a DDR read does not wait for a buffer entry of its line | `toggle` (+MIGLAT=20): narrow read 00010010: … want …_… |
| `dma_rd_hit` | DMA reads ignore the L2 (always DDR) | `dma` (+MIGLAT=20): narrow read 00020250: … want …_… |
| `dma_wr_hit` | a DMA write to a valid sector goes to DDR (the L2 copy stays stale) | `dma` (+MIGLAT=20): cache read 00020240 dw0: … want … |
| `dma_wr_clean` | a DMA write merged into the L2 leaves the sector clean | `random` seed 1 (+MIGLAT=20): cache read 00028060 dw0: 0 want … (lost at eviction) |
| `fill_set` | a request to the set a fill is writing is accepted | `evict` (+MIGLAT=20): cache read 000100e0 dw0: … want … |
| `victim_drop` | an allocation's dirty victim is not written back | `evict` (+MIGLAT=20): evict: event 7 = 0 want 16 |
| `walk_nocopy` | the walk invalidates dirty ways without writing them back | `dma` (+MIGLAT=20): dma: DDR 00020240 = … want … |
| `walk_nodrain` | BUSY drops before the write buffer has drained | `wbinv` (+MIGLAT=8 +MIGJIT=20 +MIGRDY=30): wbinv: DDR 0002fee0 = … want … |
| `walk_yield_skip` | a walk that yields mid-set moves on to the next set | `wbinv` (+MIGLAT=20): wbinv: DDR 00018000 = … want … |
| `dis_nowalk` | a write of 0 disables at once (no write-back) | `dma` (+MIGLAT=20): dma: DDR 00020240 = … want … |
| `alloc_both` | a sector allocation marks the other sector valid | `sector` (+MIGLAT=20): sector: event 3 = 0 want 1 |
| `refetch_present` | a sector miss refetches the whole line (over the other sector's data) | `sector` (+MIGLAT=20): sector: event 6 = 8 want 4 |
| `rt_valid` | a sector-only fill (VGA fetching) marks both sectors valid | `random` seed 1 (+MIGLAT=20): cache read 000300e0 dw0: 0 want … |
| `wb_order` | the write buffer drains entry 0 first, not the oldest | `dma` (+MIGLAT=20): dma read 000302c0 dw2: … want … |
| `spec_noaddr` | the speculative lookup is used for any address | `hit` (+MIGLAT=20): hit latency 2 want 3 |
| `spec_wrcoll` | a speculative read next to a data-array write is used | `evict` (+MIGLAT=20): cache read 000100e0 dw0: … want … |
| `acc_wrcoll` | a request is accepted while a write to its set is pending | `evict` (+MIGLAT=20): cache read 000100e0 dw0: … want … |
| `early_beat` | the early critical dword is taken from the wrong beat | `evict` (+MIGLAT=20): early dw 000100c8: … want … |
| `range` | an access outside DDR is looked up (aliases a DDR line) | `range` (+MIGLAT=20): range: looked up |
| `busy_write` | L2_CTRL writes are taken while BUSY | `toggle` (+MIGLAT=20): write of 1 during the 0-walk was not ignored |

All 20 killed (the unmutated bench: 0 of 26 runs fail).  `spec_noaddr` is
caught first by the `hit` test's latency check (a stale tag word decided
a request it was not read for).

### §6 build (standard `build_fast -resynth`)

A synth-only check came first (`d8b88fd`): no Synth 8-4767; the
data arrays as 4 × 8 RAMB36 (1K × 256, read port with the output register),
the tags 1 RAMB36 (512 × 67, no output register), the write buffer 43
RAM32M + 64 RAM16X1D; BRAM tiles 91 → 124.

| Build | Commit | Change | Tier 1 WNS / WHS | Tier 2 | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| `btic_b3` (master) | `079d686` | — | +0.062 / +0.014 | — | 37,519 | 24,846 | 91 |
| `l2_b1` | `7cc10f7` | the L2 as first written | **+0.228 / +0.020** | — | 40,911 | 26,242 | 124 |
| **`l2_b2`** | **`fba47c0`** | + the data arrays' write port from registers; the speculative lookup kept across other sets' fill writes | −0.022 / +0.020 (one endpoint) | **+0.030 / +0.020** (AggressiveExplore) | 40,586 | 26,581 | 124 |

- **`l2_b1`** met in tier 1, but its post-place report had two L2 paths at
  −0.30 (F_DEC's write decision → the 32 data BRAMs' write enables; the
  fill-beat merge `e_bc` → their data inputs), fixed in `1e16603` by writing
  the data arrays from registers one edge later (the accept, speculation
  and walk guards cover the extra read-after-write edge;
  mutations `spec_wrcoll`, `acc_wrcoll`).
- **`l2_b2`** (`fba47c0`, the final RTL): tier 1 missed by 0.022 ns on
  one endpoint, the core's load-hit pipe `wb_hext` → `mem_result` (13
  levels; the hit-pipe family of §5 review 2 / 3), nothing of the L2;
  tier 2 AggressiveExplore **WNS +0.030 / WHS +0.020**, 26 min 15 s, no
  alternate placement.  The L2's own worst path is +0.36 ns
  (`r_tw` → the drain's `e_wph`, 8 levels); the 3-cycle hit (tag BRAM
  without output register → compare → `r_hit` / `r_msel`; then the
  registered way mux) is not near critical, so the 4-cycle fallback was
  not needed.
- **Utilisation** (`l2_b2`): 40,586 LUTs (4,089 as memory), 26,581 FFs,
  124 BRAM tiles (+33: 32 data + 1 tag), 19 DSPs — +3,067 LUTs, +1,735 FFs
  against `btic_b3`.  Hierarchy (`l2_b1`): `l2_cache` 4,529 LUTs (236 as
  memory) and 1,319 FFs of its own (the MIG beside it, 2.7k, was there
  before inside `ddr2_control`); most of it is the 256-bit datapaths — the
  4:1 way mux, the answer mux (hit / fill), the fill assembly and its
  write-data mux, the write buffer's input mux and LUTRAM, the registered
  write port.  If LUTs get short: assemble fills without the merged-beat
  path (the line one cycle later; −~500), or a 128-bit internal datapath
  (+1 cycle a hit).
- **Bitstream `fba47c0`** (`~/.cache/kbuild/l2_fba47c0.bit`, md5
  `73ae95fd1d690a8625f7e2ab97cfce41`), build dir `~/.cache/kbuild/l2_b2`.
  Not run on the board.

### §6 deferred / not done

- **Board validation** (Graham arranges it): boot, the board suite, the
  `PERF_L2_*` counters under Linux, `L2_CTRL` A/B (the runtime session's
  `kernel/l2.c` uses these offsets), `VGA_STATUS` underflows with scanout
  on under load.
- **Write-buffer take-back.**  A demand miss on a line in the write buffer
  waits for that entry's drain instead of taking it back (the model's
  `wb_fwd`).  `tb_l2`'s `wbhit` shows the wait; its cost on real workloads
  is unmeasured here.
- **Hit-under-fill** of the same set, and fills on a second data port (the
  model assumes separate ports): a request to the set a fill is still
  writing waits for it (a few cycles after the critical sector).
- **8 ways** builds and passes every test (`WAYS` = 8: 256 sets, 3 tag
  BRAMs → 35 tiles) but was not built for timing; the model gives +1.5–1.8
  points only if it adds no hit latency (the way mux becomes 8:1).
- **VGA margin.**  Without the L2 the scanout starves under an all-miss
  load at the board's latency (review 1's DRAM model); with the L2 and its
  stream buffer it holds to `MIGLAT` 44 (review 1).  A scanout-side fix
  (pipelined reads, more buffering) would help both.
- **Review 1 minors 5 / 6** (pre-existing, or performance only): the
  orphaned-grant window of `mem_read_write`'s arbiter (a DMA master reset
  mid-tenure holds the grant for 15 quiet cycles first), and a D-cache
  shadow writeback that waits for the L2 fill of its own set before it is
  accepted.
- **Write combining.**  Each posted write takes a write-buffer entry of its
  own (a blitter FILL of 16 B writes to lines the L2 does not hold drains
  one burst per write); merging into the tail entry of the same line would
  cut the drains.
- **Core 2** (`EN_CORE2` = 0 in the release build) is covered by `tb_l2`'s
  DMA traffic (the same 16 B masked protocol as the blitter), not by a
  SoC run.
- No prefetcher (as specified).

### §6 review 1 (2026-11-16)

An adversarial review of `fba47c0` found three majors and six minors, all
fixed in `c9cadea` (RTL, benches, tools, MMIO_MAP; the bitstream's RTL).
The commit after it changes only `tb_l2` (two test fixes, found by the
8-way sweep and a surviving mutation) and the documentation.

**Findings and fixes.**

| # | Finding | Fix |
|---|---|---|
| M1 | The WBINV walk livelocked on a set with 3+ dirty ways: F_WK2 cleared a copied way's dirty bits only in `r_tw`, the tag RAM only when the set was done; on a full buffer the walk yielded to a request, restarted the set from its stale tags and copied ways 0 and 1 again, for as long as requests kept coming (the reviewer: set 0 for 300 VGA reads, ~2,400 duplicate write beats, a 64 B drain before every VGA read) | the copy writes the way's cleared dirty bits to the tag RAM (`tb_din` of F_WK2), so a restarted set copies only ways dirtied meanwhile |
| M2 | "No background drains while VGA fetches" leaked: the guard was `f_st != F_IDLE`, true in F_LOOK (which every VGA read passes), F_DEC for hits and writes, F_WDAT, F_WRSP and the walk; a VGA read then waited in F_DEC for a drain it did not need (+3 to +11 cycles, hundreds per seed) | while VGA wants the bus or a DMA master holds it, a drain starts only for the request in F_DEC that waits on the buffer (`w_fe_need`: a read miss of a buffered line, a full buffer in the way of a victim copy or a posted write) |
| M3 | `mig_model` was optimistic: writes free, fixed read latency, refresh only stalled `app_rdy`, PA[27] kept distinct, the UG586 write-data rule unchecked | DRAM timing (below); refresh closes the banks and holds queued commands, reads included; `app_addr[26]` ignored; a `$fatal` when write data comes more than 2 cycles after its command; `+MIGFREE=1` keeps the old model for comparison |
| m1 | `tb_l2` dropped DV on the ready edge; real masters hold it one more cycle | the DV tail is the default (`+DVTAIL=0` for the old protocol); every ready must answer a request (a tail taken as a new request fails); mutation `served` |
| m2 | The walk started only in an F_IDLE cycle with DV low: a master re-raising DV after one low cycle (write hits: the data write blocks that cycle) held BUSY for its whole tenure | the walk and requests take turns (`r_wk_turn`: after a request accepted during the walk, the next set goes first); and the walk's end waits only for the buffer entries present when its last set was done (`r_wk_fl`), so posted writes after that cannot hold BUSY up |
| m3 | "Orders aliases conservatively" was wrong: the MIG ignores the rank bit, so PA[27] aliases, and a bypassed alias write could go to DDR under a dirty L2 copy | the L2 folds PA[31:27] (tags, write buffer and MIG address on PA[26:6]; no bypass) |
| m4 | `run.py` passed a gated tb_soc run whose checker checked nothing | a gated run with no `consistency checks=` line, or 0 checks, fails |
| m5 | tb_soc's default moved the old SoC tests from 8-cycle DDR to `MIGLAT=36` | `tools/vsim/l2_regress.sh` runs them at `+MIGLAT=8` too (section `soc8`) |
| m6 | MMIO_MAP's SD DMA reservation said `SDM_CFG[1]` | the SD DMA spec v1.2 reservations: `SDM_CFG[16]` DMA_EN, `SDM_STATUS[5]` DMA_BUSY / `[27]` DMA_ERR, `SDM_CAPS[10]`, `CACHE_OP_ADDR` / `CACHE_OP_LEN` (32-bit, `0xF005_0030` / `0x0038`), `CACHE_CTRL[4]` RANGE_CLEAN / `[5]` RANGE_INV / `[6]` RANGE_IINV, `CPU_FEATURES[27]`; `SDM_CFG[1]` stays the HS edge (SD_NATIVE_PLAN) |

The review's minors 5 and 6 (the orphaned-grant window; shadow writebacks
waiting for the fill) are pre-existing or performance only: deferred
(list above).

**The DRAM model** (`mig_model.sv`; the board's MIG is `mig_c.prj`:
MT47H64M16HR-25E at tCK = 5 ns — DDR2-400, CL 3, WL 2, BL8 — with
ui_clk = 10 ns = 2 tCK; datasheet ns rounded up to ui cycles): T_RCD 2,
T_RP 2, T_RAS 4, T_RRD 1, T_FAW 5; CAS to CAS 2 (a BL8 is 4 tCK of data
bus); write-to-read T_WTR 4 (WL + BL/2 + tWTR = 8 tCK); read-to-write
T_RTW 4 (RL + BL/2 + 2 − WL = 7 tCK); read-to-precharge 2; write recovery
to precharge T_WR 5 (WL + BL/2 + tWR = 9 tCK); refresh every 780 (tREFI
7.8 µs): all banks closed (each after its tRAS / tRTP / tWR), then T_RP +
tRFC 13 (127.5 ns) with every queued command waiting.  ROW_BANK_COLUMN map
(bank PA[13:11], row PA[26:14]); 4 bank machines open / close rows for the
first 4 queued commands of different banks; a row stays open only while a
queued command hits it (closed-page with lookahead, an approximation of the
7-series controller).  `MIGLAT` (default 36) is still the latency of an
isolated read to a closed bank (the board's ~44 cycles from a D-cache miss
to its critical dword); row hits are 2 sooner, everything else later.  It
reports activates, conflicts, refreshes and write-to-read turnarounds.

**VGA under the new model: the L2 starved more, fixed with a stream
buffer.**  With M1 / M2 fixed, the heavy `socvga` load (every access misses
both levels, dirty victims, WBINV every 1,500 steps) starved in both builds
at the default `MIGLAT=36` — 246 lines without the L2, **352 with it** (a 40-
line run: 64 / 100).  The L2 makes the CPU faster, so its misses (and the
scanout's yields to them) come more often, and a VGA read took 47.7 cycles
against 45.2.  The fix (RTL) is the **VGA stream buffer** (design above): a
VGA read of sector 0 of a line the L2 does not hold fetches the whole line
and answers the next read from a register, so half of the scanout's reads
have no DDR round trip (its average read latency 27 cycles against 45).

Starved display lines of 120 frames' worth of program (`socvga`, raster
moved to vblank; heavy = a 512 KiB stride walk, load + 2 stores a step;
moderate = a 96 KiB walk (L2 hits), WBINV every 300; blit = the blitter
FILLs a 640 × 64 rect (80 KiB of 16 B DMA writes) back to back; worst fetch
of a line in cycles, of 3,200):

| Load, MIG | L2 off: starved (worst fetch) | L2 on: starved (worst fetch) | VGA read latency off / on |
|---|---|---|---|
| heavy, `MIGLAT=36` (default, refresh on) | 246 | **0** (2,769) | 45.2 / 27.3 |
| heavy + blit, 36 | 316 | **0** (2,784) | 45.2 / 26.8 |
| heavy + blit, 36, `+MIGJIT=20 +MIGRDY=10` | 1,012 | 3 | 58.6 / 34.3 |
| heavy, 44 | 834 | **0** (2,986) | 53.1 / 31.0 |
| heavy + blit, 44 | 960 | **0** (3,009) | 53.1 / 30.5 |
| heavy, 52 | 960 | 2 | 61.0 / 34.7 |
| heavy + blit, 52 | — | 17 | — / 34.6 |
| heavy, 60 | — | 259 | — / 38.6 |
| moderate, 36 | 0 (1,907) | 0 (1,180) | 44.2 / 24.2 |
| moderate, 44 | 0 (2,235) | 0 (1,335) | 52.2 / 28.3 |
| moderate + blit, 44 | 0 (2,323) | 0 (1,418) | 52.2 / 28.6 |
| moderate, 52 | 0 (2,564) | 0 (1,500) | 60.2 / 32.6 |

The L2 build starves less than the L2-off build in every setting.  Its
margin under the all-miss load is ~190–430 cycles of the 3,200 at
`MIGLAT` 36–44 (blit included); it starts to starve at `MIGLAT` 52 (2
lines; 17 with the blitter) and with jitter at 36, where the build without
the L2 starves nearly every line.  Under the realistic (moderate) load the
worst fetch is 1,180–1,500 cycles: over half the line time to spare.  The
drain rule held in every run: the tb_soc probe counted 0 drains in VGA's /
DMA's window without a request waiting on the buffer (and 7,044 that had
one, in the heavy load: the CPU's own misses between VGA chunks).

**Tests and checks added.**  `tb_l2`: `alias` (writes through one alias,
reads through another, DMA and D-cache, then WBINV and DDR checked),
`wbinvq` (every set fully dirty, WBINV with reads back to back during the
walk: the walk must end within `3 × SETS × WAYS` requests and write
exactly `SETS × WAYS × 8` beats — on `fba47c0` it stayed at set 0 after
4,096 requests, 47,488 beats for 16,384), `wkturn` (DMA write hits re-raising DV after one low cycle in
16-transaction tenures: BUSY must drop within `4 × SETS` requests — 514
now; `fba47c0` stuck at set 128 after 2,048), `rtdrain` (a posted entry
survives VGA hits and misses with VGA's request up, VGA hits keep their
3-cycle latency, a VGA read of the buffered line gets exactly one drain —
`fba47c0` drained during the VGA reads), `vgastream` (the stream buffer:
one 8-beat fetch for two reads, the second at hit latency; a DMA write or
a D-cache writeback after the fetch, and writes while sector 1 is still on
its way under jittered DDR, drop it; a line held with only sector 1 dirty
never fills it).  `wbinv` now leaves out of its DDR check the writes
acknowledged after BUSY dropped (they may stay in the L2; the 8-way sweep
caught the test assuming otherwise), and `random` waits out a walk a
random `L2_CTRL` write started before its own WBINV (a write while BUSY is
ignored).  Every `tb_l2` run checks that every ready answers a request and
fails on any drain started in VGA's / DMA's window with no request waiting
on the buffer (`rt_bad`); tb_soc counts the same (`L2 rt drains bad=`,
built in with the L2, gated by `run.py`).  `socvga --blit` adds the
blitter load and reports the scanout's read latency; `socbench --base`
adds a revision's column.

**Mutations** (`l2mut.py`, now 28: every directed test and 4 random seeds
under two MIG timings, in parallel; the unmutated bench: 0 of 34 runs
fail).  `range` became `alias_bypass`; `walk_nodrain` now drops the walk's
final-drain condition.  The new ones:

| Mutation | What it breaks | First failing run |
|---|---|---|
| `alias_bypass` | PA[31:27] ≠ 0 bypasses the L2 (the pre-review rule) | `alias`: early dw 18010100: 0 want … |
| `walk_recopy` | the copy clears the dirty bits in `r_tw` only (M1) | `wbinvq`: walk still at set 0 after 6,144 requests (livelock) |
| `rt_drain` | the pre-review drain guard (`f_st != F_IDLE`, M2) | `wbhit`: 3 drains started while VGA / DMA held the bus with no request waiting on the buffer |
| `rt_need_wbm` | a read of a buffered line does not count as waiting on the buffer | `alias`: hang (a DMA read of its buffered line waits for a drain that never starts) |
| `wk_noturn` | no turns between the walk and requests (minor 2) | `wkturn`: BUSY after 2,048 requests (walk at set 128) |
| `strm_snoop` | a write to the line does not drop the stream buffer | `vgastream`: dma read 000407a0: 0 want … |
| `strm_fill_snoop` | a write during a stream fill does not stop its capture | `vgastream`: dma read 000487e0: 0 want … |
| `strm_any` | a stream fill also for a line the L2 holds (a dirty sector 1 there) | `vgastream`: dma read 00018a20: 0 want … |
| `served` | a DV still high after its ready is taken as a new request (minor 1) | `hit`: hit latency 1 want 3 (the tail's phantom answer) |

**28 of 28 killed** (on the final benches; the unmutated bench: 0 of 34
runs fail).  `walk_nodrain` is now caught by `wbinvq` (16,376 beats for
16,384 when BUSY drops: the last entry had not drained).  `strm_snoop`
survived the first version of `vgastream` (its write came while sector 1
was still on its way, which only `e_strm_ok` covers); the test now also
writes after the fill.

**Regression** (`tools/vsim/l2_regress.sh`, all on `c9cadea`'s RTL):
- `tb_l2` sweep (`l2mut.py --sweep`): every directed test and 12 random
  seeds × 3,000 transactions at 2, 4 and 8 ways under four MIG timings
  (`+MIGLAT=4`; 36; 8 with `+MIGJIT=20 +MIGRDY=30`; 60 with `+MIGREF=200
  +MIGRDY=10`): all 300 runs pass (13 directed tests and 12 seeds × 4
  timings × 3 geometries).
- SoC tests at `MIGLAT=36` (default): `socdiff` against `d7f3994` (30
  seeds; 20 with a 64 B span), `socmmio`, `socwalk`, `socx`, `socdcbz`,
  `socl2`, `socsd`, `soceth`, `socvga`; at `+MIGLAT=8` (minor 5) the same
  pre-L2 set and `socl2`; and with the L2 off, with 8 ways, at `+MIGLAT=8
  +MIGJIT=20 +MIGRDY=30` and at `+MIGLAT=60 +MIGREF=500`: all pass, every
  run gated (BTIC probe, consistency checker with > 0 checks, the drain
  probe).
- `L2_EN` = 0 equivalence: `VSIM_L2=0` against `d7f3994` under the same
  tb_soc: identical traces and PIPE_RUN cycles (30 seeds 69,249 = 69,249;
  64 B span 25,610 = 25,610).
- Core regression (`pipeline_core` is unchanged since `fba47c0`): `test` in
  5 configurations (default, DEBUG_EN, MPU_EN, TLB_N = 16, TLB_WALK = 0),
  `diff` against `d7f3994` with `+TRACECYC` (40 seeds × 1 / 3 / 7: 169,092
  = 169,092) and `--irq` (107,709 =), `excdiff`, `mpudiff`, `tlbdiff`,
  `tlbpage`, `dcbzdiff` 200 seeds, `dmtest`, `btfuzz` 300 seeds × 7: all
  pass.

**Cycles** (tb_soc, the new model at `MIGLAT=36`): `c9cadea` against
`fba47c0` under the same tb_soc: **identical** on randprog (30 seeds 61,698
= 61,698; 64 B span 24,645 = 24,645) and on `socbench` (data96k 382,024,
code96k 142,666, chase112k 199,846, each = `fba47c0`) — the fixes act only
with VGA / DMA traffic, walks or aliases.  L2 off → on under the new model:
randprog 69,249 → 61,698 (−10.9 %), 64 B span 25,610 → 24,645 (−3.8 %),
data96k 906,210 → 382,024 (−57.8 %), code96k 323,972 → 142,666 (−56.0 %),
chase112k 475,595 → 199,846 (−58.0 %) — the same gains as on the old
model.

**Build** (standard `build_fast -resynth`; a synth-only check first: no
new RAM inference, the same 32 + 1 RAMB36, 43 RAM32M):

| Build | Commit | Tier 1 WNS / WHS | LUTs | FFs | BRAM |
|---|---|---|---|---|---|
| `l2_b2` | `fba47c0` | −0.022 / +0.020 (tier 2 +0.030) | 40,586 | 26,581 | 124 |
| **`l2_b3`** | **`c9cadea`** | **+0.040 / +0.010** (no tier 2, 15 min 21 s) | 40,740 (4,085 as memory) | 26,783 | 124 |

The worst path is the core's (the TLB → the D-cache's `r_h_tag1`, 7
levels); the L2's own worst is +0.474 (`e_c` → `o_mem_read_data`, the
fill-beat answer mux), its review-1 logic nowhere near.  +154 LUTs and +202
FFs, most of it the stream buffer's 256-bit register and answer mux.
Bitstream `~/.cache/kbuild/l2_c9cadea.bit` (md5 `c4b1bd598877774ed65bf1a0aa360272`),
build dir `~/.cache/kbuild/l2_b3` (`l2_b2` deleted).  Not run on the
board.

### §6 review 2 (2026-11-16)

A delta review of `c9cadea` (review 1's RTL, the bitstream about to be
flashed) found no critical or major findings, three minors and three
nits; the coordinator added one item from the board (the DMA counters with
the L2 disabled).  The SD DMA engine's posted write streams come next, so
m-A mattered.  Fixed in `0af3aa0` (RTL, benches, tools, MMIO_MAP); `32c318c`
changes only benches and tools; `13be986` restructures one signal for
area and timing (identical behaviour: every simulation below is
cycle-identical between the two).

**Findings and fixes.**

| # | Finding | Fix |
|---|---|---|
| m-A | A posted-write stream held BUSY up.  The walk's turn helped only if a buffer slot was free when it reached F_WK2; a posted write (a write that misses during the walk) waiting in F_DEC took each freed slot, so the walk found the buffer full again and yielded again.  While a DMA master holds the grant only `w_fe_need` drains start, and the walk was not a waiter, so a DMA tenure of posted writes held BUSY at any rate (the reviewer's `dmapost`: the walk at set 1 after 20,000 requests, also 10 or 40 idle cycles apart); back-to-back D-cache writes that miss did too (`cpupost`).  A regression from M2 (at `fba47c0` a drain could start while the walk waited) | The walk is a **request in hand** while it waits on the buffer (`w_wk_need`: a dirty way to copy and the buffer full — `r_wk_wb`, set in F_WK2 — or its end with entries left from before its last set): its drain starts in VGA's / DMA's window, and before a demand read.  The slot that drain frees is the walk's: while `r_wk_wb` a posted write needs an empty buffer (`wb_full_p`).  A write to the set the walk waits in is not accepted until the walk has copied it: without this `wkset` (a DMA master writing into the walk's set, re-dirtying a way already copied) kept the walk at set 0, the lowest dirty way going into the reserved slot every time.  Bound below |
| m-B | The drain probe was looser than the RTL: `rt_pneed` (tb_l2) and tb_soc's `L2_PROBE` took any F_DEC non-hit with a full buffer as a need, so a mutant draining for VGA read misses whenever the buffer is full passed the `rt_bad` gate | Both probes spell the need out from the design's rule (the request's kind, its line, the victim's dirty bits, the walk's state), not from `w_fe_need`; `rtdrain` fills the buffer and sends VGA and DMA read misses and VGA stream fills with their stream hits (no drain may start); mutation `rt_need_full` |
| m-C | Removing the same-edge term of the stream buffer's capture (`!(w_wr_acc && i_mem_addr[26:6] == e_line)`) survived `vgastream` and the random seeds | `strmedge`: a stream fill, then a DMA write timed at every offset 0–15 cycles after the read's ready (2 land on the capture edge at `MIGLAT=36`), then the sector-1 read; mutation `strm_edge` |
| nit | `run.py` passed an L2-on gated tb_soc run with no `L2 rt drains` line | such a run fails (`SOC_L2P`) |
| nit | `w_imm` compared `[31:5]`, not the folded `[26:5]`: aliases lost the 3-cycle speculative hit | compares `[26:5]` (5 comparator bits fewer) |
| nit | `w_fe_need` does not exclude `w_strm_hit` | an assertion: never a stream-buffer hit while the request in F_DEC waits on the buffer (`w_fe_req`) — the stream buffer holds a line no write has touched since its fill, so no entry of it exists |
| board | With the L2 disabled (`L2_CTRL` = 0) the DMA counters showed hits = half of VGA's reads and misses = 0: the stream buffer still served the scanout in bypass and counted its hits, while the misses were gated by `q_lk` | **Not intended**: the stream buffer now works only while the L2 is enabled (`q_lk` in `w_strm_hit` / `w_strm_new`), so bypass is the plain DDR path of an A/B run and counts no lookup at all.  MMIO_MAP says a stream hit counts as a DMA hit and its fill as a DMA miss (a scanout over a framebuffer outside the L2 shows hits = misses, as the board does).  `toggle` checks VGA reads in bypass (no stream hit); mutation `strm_bypass` |

**The walk's bound.**  Let `T_x` be the longest DDR transaction (a 4-burst
read or a 64 B drain, MIG stalls — refresh, turnarounds — included) and
`T_q` the longest time a request accepted during the walk holds the front
end (a read: a drain of its line and its read; a posted write: the
transaction in progress and two drains; ≤ 3 `T_x`).  The walk spends one
step (4 cycles) on each set, preceded by at most one request (the turns).
Each way of a set is copied at most once — its dirty bits clear at the
copy, and writes to the set it waits in are held off — so there are at
most `SETS × WAYS` copies.  A copy that finds the buffer full waits for at
most the transaction in progress and its own drain (which goes before any
read; no posted write takes the slot it frees), then the one request
accepted meanwhile.  The end waits for at most one request and two drains.
So under any request stream, from the write that starts the walk:

    BUSY ≤ SETS × (T_q + 4) + SETS × WAYS × (2 T_x + T_q + 7) + T_q + 2 T_x

— a few ms at 4 ways with refresh-length `T_x`.  In simulation a WBINV of
a fully dirty L2 (2,048 lines) with back-to-back DMA writes into the set
being walked took 42,701 cycles at `MIGLAT=36` and 47,309 at `MIGLAT=60
+MIGREF=200 +MIGRDY=10` (~0.45 ms); with a dirty line in every set and a
DMA posted-write stream (`dmapost`), 15,863.  MMIO_MAP's "no request
stream can hold BUSY up" is now true.

**VGA.**  The walk's drains now also run in VGA's window (one each time
the walk finds the buffer full, before a demand read).  `socvga`'s heavy
load (a 512 KiB stride walk, WBINV every 1,500 steps): starved lines
(worst fetch of 3,200 cycles) and the scanout's read latency, `c9cadea`
against review 2 (`0af3aa0` = `13be986`, cycle for cycle):

| Load, MIG | `c9cadea` | review 2 | VGA read latency |
|---|---|---|---|
| heavy, `MIGLAT=36` | 0 (2,769) | 0 (2,786) | 27.3 / 27.6 |
| heavy + blit, 36 | 0 (2,784) | 0 (2,794) | 26.8 / 27.1 |
| heavy, 44 | 0 (2,986) | 0 (3,013) | 31.0 / 31.3 |
| heavy + blit, 44 | 0 (3,009) | 0 (2,996) | 30.5 / 31.1 |
| heavy, 52 | 2 (3,201) | **0** (3,192) | 34.7 / 34.9 |
| heavy + blit, 52 | 17 (3,198) | 16 (3,202) | 34.6 / 34.8 |

No setting starves more; the worst fetch moves by −13 … +27 cycles (in
this load the WBINV walks now drain while VGA fetches: a VGA read can meet
one of the walk's drains, where before the walk waited for VGA to leave
the bus).  The drain probe counted 0 drains in VGA's / DMA's window
without a waiter in every run.

**Tests added** (`tb_l2`; the first four fail on `c9cadea`):
`dmapost` (a dirty line in every set, WBINV, a DMA tenure of posted 16 B
writes to lines half the cache away from the walk's set, the grant
dropped one cycle every 8, 0 / 10 / 40 idle cycles between them: BUSY
within `4 × SETS` requests — 1,025 / 1,024 / 217; `c9cadea` still at set 1
after 2,048, also with the gaps), `cpupost` (the same with the D-cache's
32 B writes, 0 / 10 apart: 823 / 551; `c9cadea` at set 2), `rtwalk` (VGA's
request up for the whole walk, reads back to back — hits, DMA misses,
sector-0 stream fills, DDR beats spread out —, nothing needing a slot:
BUSY within `8 × SETS` requests with VGA still on the bus, 1,074;
`c9cadea` stopped at set 2; a read never starts before the walk's drain,
`wk_prio_bad`, and the case is met 120–160 times a run), `wkset` (every
set fully dirty, a DMA master writing into the set being walked: BUSY
within `2 × SETS × WAYS` requests, 514; without the held-off writes the
walk stayed at set 0), `strmedge` (m-C).  `rtdrain` and `toggle` extended
(above).  The posted writes of `dmapost` / `cpupost` go to sets away from
the walk's: to the walk's own set they are held off until its copy, which
hid the slot race (mutation `wk_reserve` survived a first version that
wrote to the set the walk was in).  `wbinv`'s final check reads a line its
traffic never touches (the 8-way sweep caught the traffic's last request,
accepted after BUSY dropped, allocating the line it read).

**Mutations** (`l2mut.py`, now 36; every directed test and 4 random seeds
under two MIG timings).  `spec_noaddr`, `strm_any`, `acc_wrcoll`,
`rt_need_wbm` follow the RTL's new text.  The new ones:

| Mutation | What it breaks | First failing run |
|---|---|---|
| `wk_need_copy` | the walk waiting on a full buffer is not a waiter (m-A) | `wbinv`: 3 demand reads started before the drain the walk waited on |
| `wk_need_end` | the walk's final drain is not a waiter (m-A) | `rtwalk`: BUSY after 4,096 requests with VGA on the bus |
| `wk_reserve` | a posted write takes the slot the walk waits for (m-A) | `cpupost`: BUSY after 2,048 requests (walk at set 2) |
| `wk_rd_first` | a demand read starts before the drain the walk waits on | `rtwalk`: reads started before the walk's drain (`wk_prio_bad`) |
| `wk_set_wr` | a write to the set the walk waits in is accepted | `wkset`: BUSY after 4,096 requests (walk at set 0) |
| `rt_need_full` | a DMA / VGA read miss counts as waiting on a full buffer (m-B) | `random` seed 2: the stream-hit assertion; with the assertion taken out, `rtdrain`'s probe (`rt_bad` = 1) |
| `strm_edge` | a write accepted on the edge sector 1 is captured does not drop the stream buffer (m-C) | `strmedge`: the stream-hit assertion (the stale buffer answered a read of a buffered line) |
| `strm_bypass` | the stream buffer serves VGA while the L2 is disabled | `toggle`: VGA reads in bypass used the stream buffer |

**36 of 36 killed** on `13be986` (the unmutated bench: 0 of 44 runs
fail).  `rt_need_full` can start a drain only on a stream-buffer hit (a
demand miss starts before any drain), so `rtdrain` now also sends stream
fills and their stream hits with the buffer full; the old, looser probe
counted that drain as needed.

**Regression** (`tools/vsim/l2_regress.sh`, on `13be986`'s RTL):
- `tb_l2` sweep (`l2mut.py --sweep`): every directed test (now 18) and 12
  random seeds × 3,000 transactions at 2, 4 and 8 ways under four MIG
  timings: all 360 runs pass.
- SoC tests at `MIGLAT=36` (incl. `socsd`, `soceth`, `socvga`), at
  `+MIGLAT=8`, with the L2 off, with 8 ways, at `+MIGLAT=8 +MIGJIT=20
  +MIGRDY=30` and at `+MIGLAT=60 +MIGREF=500` (`socdiff` against
  `d7f3994`, 30 seeds and 20 with a 64 B span, `socmmio`, `socwalk`,
  `socx`, `socdcbz`, `socl2`): all pass, every run gated (BTIC probe,
  consistency checker, the exact drain probe — 0 bad drains).
- `L2_EN` = 0 equivalence (`VSIM_L2=0` against `d7f3994` under the same
  tb_soc): identical traces and PIPE_RUN cycles (69,249 = 69,249; 64 B
  span 25,610 = 25,610).
- Core regression (`pipeline_core` unchanged): `test` in 5 configurations,
  `diff` with `+TRACECYC` (169,092 = 169,092), `--irq` (107,709 =),
  `excdiff`, `mpudiff`, `tlbdiff`, `tlbpage`, `dcbzdiff` 200 seeds,
  `dmtest`, `btfuzz` 300 seeds × 7: all pass.

**Cycles** against `c9cadea` (tb_soc, `MIGLAT=36`): **identical** —
randprog 61,698 (30 seeds; 64 B span 24,645), and `socbench --base
c9cadea`: data96k 382,024, code96k 142,666, chase112k 199,846, each
+0.00 %.  The changes act only during walks, with the L2 disabled, and on
alias speculation (the programs use none).

**Build** (standard `build_fast -resynth`; `l2b_c9ref` rebuilt `c9cadea`
the same day as a reference — the flow is deterministic, it reproduced
`l2_b3` exactly):

| Build | Commit | Tier 1 WNS / WHS | Tier 2 | Alt-place (`ExtraPostPlacementOpt`) | LUTs | `mem_read_write` / `l2_cache` own (in context) | FFs |
|---|---|---|---|---|---|---|---|
| `l2b_c9ref` | `c9cadea` | +0.040 / +0.010 | — | — | 40,779 | 12,898 / 4,430 | 26,791 |
| `l2b_b1` | `0af3aa0` | −0.076 / +0.021 | +0.006 / +0.021 | +0.032 / +0.010 | 41,366 | 13,276 / 4,795 | 26,870 |
| **`l2b_b2`** | **`13be986`** | −0.187 / +0.027 | −0.001 (not met) | **+0.028 / +0.038** (ExtraTimingOpt +0.008 / +0.024) | 41,018 (alt: 40,990) | 12,754 / 4,433 | 26,861 |

- **`l2b_b1`** (`0af3aa0`) missed tier 1 by 0.076 on the D-cache hit →
  the core's load extension (`r_h_tq0` → `wb_hext`, 10 levels, 74 %
  routing; +0.218 at `c9cadea`).  In context the L2's own logic had grown
  by 365 LUTs, much more than the new terms: synthesised with each change
  left out, the slot reservation and the held-off writes each cost ~330
  (`mem_read_write` 10,715 → 10,382 / 10,381; `c9cadea` 10,349).  The
  posted write's own `wb_full_p` kept the write-buffer entry updates from
  sharing their logic with the victim-copy paths that used `wb_full`.
- **`13be986`** uses `wb_full_p` for all of the front end's slot
  decisions: the two differ only while `r_wk_wb`, during a walk, when the
  victim-copy paths are dead (nothing allocates; asserted).
  `mem_read_write` synthesises to 10,171 LUTs (below `c9cadea`), and
  placed, `l2_cache`'s own logic is 4,433 LUTs against `c9cadea`'s 4,430.
- **`l2b_b2`** nonetheless missed tier 1 by 0.187, and tier 2 ended at
  −0.001.  Every failing path is outside the L2 and unchanged since
  `c9cadea`: the D-cache's line-fill BRAM → hit-copy BRAM write data
  (`cache_val_data_way1` → `h_data_way1` `DIADI`, 3 levels, −0.187), the
  hit pipe → `wb_hext` (−0.144), the TLB → `r_h_tag` / `r_h_tag1`
  (−0.105 / −0.098), `wb_hok` → the TLB's `q_b` (−0.102),
  `id_op` → `ex_d` (−0.042).  With the L2's area equal to `c9cadea`'s,
  this is placement variance on the core's and the D-cache's own
  near-zero-margin paths (they move ±0.2 ns between two placements of an
  L2 of the same size), not something the L2's RTL can close.
- The alt-place cross-check from `l2b_b2`'s opt checkpoint met with
  margin: **+0.028 / +0.038** (worst: the core's `ex_lb_a` →
  `mem_result`, 15 levels; then `bus_splitter.r_is_mmio` → `pc` CE, and
  the hit pipe → `wb_hext` +0.067).  Nothing of the L2 within +0.19.
- A core-side timing item (the hit pipe → `wb_hext`, the TLB → `r_h_tag`,
  the D-cache's BRAM → BRAM fill copy) would make tier 1 robust for the
  next features; it is outside this review.
- Bitstreams: **`~/.cache/kbuild/l2b_13be986_epp.bit`** (md5
  `995f412ee3187f6dfd1dd588f91b0484`, alt-place, the review's final RTL);
  `l2b_0af3aa0.bit` (standard flow, tier 2 +0.006; md5
  `ffa0f11add8a8a28ea53cda3864c9308`) and `l2b_0af3aa0_epp.bit` (+0.032 /
  +0.010; md5 `69754f0ef93d45256f5c70fc822df96a`) are the same behaviour
  with the larger L2.  Build dirs `~/.cache/kbuild/l2b_b1`, `l2b_b2`,
  `l2b_c9ref`.  Not run on the board.

### §6 timing: the core / D-cache families restructured (2026-11-16, branch `tmg`)

`l2b_b2` (`13be986`) missed tier 1 by 0.187 on core / D-cache paths that
the L2 never touched; only alt-place closed it.  Branch `tmg` (from
`c0bf7b2`) restructures those families so the standard flow meets with
margin.  Every change is behaviour- and **cycle-identical** (one
documented exception, unexercised: a RET popping its return address
from MMIO); each has a simulation assertion of its old form.

| Family (`l2b_b2` tier 1) | Fix | `tmg_b3` |
|---|---|---|
| D-cache line BRAM → hit-copy BRAM `DIADI` (−0.187): a write hit read-modified-wrote the whole line from the unregistered BRAM output (`cache_val_data` → merge → both arrays) | `mem_read_write`: the four data arrays are written with **byte write enables** from registers (`w_dwdata` / `w_dwe0` / `w_dwe1`): a write hit writes only the store's bytes (`{4{r_write_data}}`, the hit on the enables), DCBZ all 32 bytes of zeros, the installs every byte; the other bytes keep their contents, which is the old merge.  1 K × 256 now maps to 8 RAMB36 per array (was 7 + 1 RAMB18): +2 BRAM tiles | no such path; the data arrays' worst input +0.419 (tag → `WEA`) |
| hit pipe `r_h_tq0` → `wb_hext` (−0.144) | `mem_read_write` exports each way's dword at the offset (`o_h_dw0` / `o_h_dw1`) and the way select (`o_h_sel0`); the core extracts the load-bypass value from both and from the captured / forwarded dword in parallel (three `f_ldext`), and `mem_h1` / the tag compare pick at the register | +0.124 |
| TLB CAM → `r_h_tag` / `r_h_tag1` (−0.105) | none of its own: the hit pipe's H2 slack let the placer move the registers toward the TLB | +0.505 |
| `wb_hok` → TLB key copies / `ex_a` (−0.102) | the WB forward value of a hit load is `wb_hext` (= `f_ldext(wb_hdw, …)`, registered on the same edge), so `wb_h && wb_hok` is the **last** select of `wb_ldval` instead of the first, ahead of the extraction | +0.848 |
| `id_op` → `ex_d.uop` / the field clears (−0.042) | `id_legal` / `id_privop` decoded from `w_op` at the IF → ID latch (cones already there for `id_ser` / `w_xser`); `f_decode_q` = `f_decode` with those registers | +0.729 |
| `ex_d.uop` → adder → `mem_result` (−0.001) | `ex_res_add` / `ex_res_cmp` and the adder's subtract / carry-in controls registered at dispatch; the sum and a compare's bit are the last select of `mem_result` | the family's worst is now UNARY's count ops, +0.061 |
| `r_is_mmio` → read data → RAS compare → `pc` (+0.011) | the RET / LEAVERET return-address compare uses the DRAM side's data (`m_rdata_dram`, new `m_rdata_next_dram`), off the splitter's response select; a return address popped from **MMIO** (`m_addr` in 0xF…) always counts as mispredicted — the redirect goes to the real value, so it is correct, only a squash more (no program keeps its stack in MMIO; the RAS-miss counter would count it) | +0.834 |
| `qspi` `r_div_cnt` → `r_tx` (+0.063) | the engine's buffer index no longer waits on the SCK tick (it only differs where the read byte is unused); `tb_qspi_ctrl` passes | +0.621 |
| (new in `tmg_b1`, −0.310) `r_shhi` → barrel stage 2 → `mem_result_eff` → forward muxes → `ex_ta` / TLB keys / `ex_a` / the multiplier | a MEM shift's `w_sh2` is the **last** select of every ID → EX operand (`fs_a` / `fs_b` / `fs_c` from registers; the other candidates take `mem_res_nsh`) | +0.183 (`wb_zero`) |
| (`tmg_b1` −0.046) `ifb_val` / `pos` → `in0` (fanout ~300) → `w_op` → predecode → `id_ser`, and the fetch engine | `in0 = ifb_val[0] && pos[0]` is a register, `r_in0` (`max_fanout` 48): `ifb_val` takes a blocking next value (`ifb_val_n`, the `ev_*` pattern) loaded with `pos` in `pos_track` | `id_ser` +0.139, `ifb_dw` / `pos` +0.336 |

**Builds** (standard `build_fast -resynth`; placed utilisation):

| Build | Commit | Tier 1 WNS / WHS | Final | LUTs | FFs | BRAM tiles |
|---|---|---|---|---|---|---|
| `l2b_b2` | `13be986` | −0.187 / +0.027 | tier 2 −0.001 (not met) | 40,915 | 26,849 | 124 |
| `tmg_b1` | `2eae6a4` | −0.310 / +0.029 (the shifter forward) | tier 2 −0.045 (not met) | 41,263 | 26,865 | 126 |
| `tmg_b2` | `831138e` | **+0.069 / +0.038** | tier 1 | 41,206 | 26,636 | 126 |
| **`tmg_b3`** | **`2709e0a`** | **+0.061 / +0.014** | **tier 1** | **41,001** | **26,868** | **126** |
| `tmg_b3` alt `ExtraPostPlacementOpt` | `2709e0a` | +0.024 / +0.020 (routed, met) | cross-check only | | | |

- Utilisation against `l2b_b2`: **+86 LUTs, +19 FFs, +2 BRAM tiles**
  (126 / 135); DSPs 19 as before; the cache arrays all RAM-inferred.
- `tmg_b3`'s worst remaining paths: `ex_a` → UNARY POPCNT / CLZ / CTZ →
  `mem_result` (+0.061, 12 LUT levels), the L2's `e_c` →
  `o_mem_read_data` (+0.090), the hit pipe's BRAM → `wb_hext` (+0.124),
  `r_in0` → `id_ser` (+0.139), `ex_lb_a` → `mem_result` (+0.171), the D-cache
  `cpu.ready` → `ex_a` (+0.172), `r_shhi` → `wb_zero` (+0.183).  Hold:
  +0.014 (`o_TX_LCD_Byte` → the LCD SPI master).
- `tmg_b2` (without `r_in0`) and `tmg_b3` are two placements of nearly
  the same netlist: +0.069 and +0.061, with different worst families
  (`wb_hext` → `mem_result` vs the count ops) — the margin is no longer
  carried by one lucky family.  The alt-place cross-check from `tmg_b3`'s
  opt checkpoint (`ExtraPostPlacementOpt`, its own phys_opt / route
  directives) met at only +0.024 — this time alt-place is the *worse*
  placement — on the same two families: `ex_a` → `mem_result[0]`
  (+0.024) and `r_shhi` → `wb_zero` (+0.025).  So the margin across three
  placements is +0.02 … +0.07, and those two (the ALU's non-adder legs —
  the count ops, MIN/MAX's compare — and the shifter's zero test into
  `wb_zero`) are the next to restructure.  Its bitstream is not kept.

**Verification** (`tools/vsim`, on `2709e0a`): the core regression in
all five configurations, `diff` with `+TRACECYC` (169,092 = 169,092
against `c0bf7b2`), `--irq` (107,709 =), `excdiff`, `mpudiff`,
`tlbdiff`, `tlbpage`, `dcbzdiff` 200 seeds, `dmtest`, `btfuzz` 300 seeds
× 7; `l2_regress.sh` `soc36`, `soc8`, `socvar` (L2 off, 8 ways, jittered,
slow MIG) and `eq` (`VSIM_L2=0` against `d7f3994`, identical traces and
cycles): all pass (`socvga` worst fetch 2,786 of 3,200, as at
`13be986`).  `tb_qspi_ctrl` passes (Verilator, with the model's
tri-state `miso` driven high in a sim copy).  The tb_l2 sweep was not
rerun: nothing under the L2 changed.  The new assertions (the decode,
the ALU legs, the shift-last forwards, `wb_hext` = the hit's extraction,
the per-way dwords, `r_in0`, the RET compare) fire nowhere.

**Cycles** against `c0bf7b2` (tb_soc, `MIGLAT=36`): **identical** —
randprog 61,698 (30 seeds; 64 B span 24,645); `socbench`: data96k
382,024, code96k 142,666, chase112k 199,846, each +0.00 %.

- **Bitstream** `~/.cache/kbuild/tmg_2709e0a.bit` (md5
  `b945088e944ec4d288c935df47b64cb1`), standard flow, tier 1, no tier 2,
  no alternate placement.  Build dir `~/.cache/kbuild/tmg_b3`.  Not run
  on the board.
- Next candidates, if a feature tips them: the UNARY count ops (register
  their result a stage later, as the shifter's, or select them last),
  `wb_zero`'s reduce of the shift result (the byte-flag form `bz_sh`
  already exists for `ex_b_zero`), the L2's `e_c` read mux, and the hit
  pipe's BRAM → `wb_hext` data path.

## 7. SD DMA engine and cache range operations (2026-11-16, branch `sddma`)

Branch `sddma` from `l2b` (`c0bf7b2`).  SD DMA spec v1.2, agreed with the
runtime session (the kernel's `sd_native.c` / `cacheflush.c` at f2afeb7
and klausscc's `--sd-dma` / `--dcache-range-ops` model are written to it).
Phase 1 here: RTL, simulation and regression; no Vivado build (one waits for
the core / D-cache timing work on `tmg`, and this branch is rebased onto it
first).  The controller's behaviour is SD_NATIVE_PLAN.md §8; the registers
are in MMIO_MAP.md ("Native SD mode", "Cache controller").
Phase 2 (review 1, below): rebased onto `tmg` as `sddma2`, the review's
fixes, and the build `sddma_b3` (`3ae5ed6`, tier 1 +0.041 / +0.034).

### §7 design

- **The engine** (`sd_native.sv`, `DMA_EN` parameter, default 1).  A data
  command written with `SDM_CFG[16]` starts a DMA phase.  Up-front checks,
  decided in the command's accept cycle so a refused command never leaves:
  the block size's bit 2 (not a multiple of 8), and for a known count
  `[ADDR, ADDR + size × count)` inside 128 MiB — a two-stage pipeline from
  the registers (product, then add-and-compare: every register write lands
  at least two edges before the next MMIO access; a simulation assertion
  recomputes it at every DMA data command).  The engine then works a block
  at a time (`M_IDLE` → `M_CHK`, the per-block range check → per 16-byte
  transaction `M_GATH` (pop 1–2 words from the FIFO head) or `M_PUSH` (push
  them), `M_XFER` → `M_WAIT`).  A transaction is an aligned 16 bytes: a pair
  of words when the address is 16-byte aligned and two words remain, else
  one word with the other half masked.
  - Reads move a block once it is committed in the FIFO (its CRC passed),
    so a bad block never reaches DDR, and the FIFO's flow control stops the
    card's clock if DDR is slow.  A counted read's last block sets
    `r_dd_pend` instead of DATA_DONE; the engine sets DATA_DONE when that
    block's last write has been accepted.
  - Writes fetch a block whenever the FIFO has room for one: at most 4 KiB
    ahead of the card, never past size × count.
- **The bus adapter** (`r_a_*`): req → grant → DV → ready → (writes: two
  settle cycles) → done, one transaction per tenure as core 2.  It has no
  reset: once started a transaction always completes, so a reset or a FIFO
  reset in the middle of one never drops a DV or a grant; the engine just
  stops owning it (`r_x_own`) and a read's data is not pushed.
  `DMA_BUSY = adapter busy || (phase && !(write phase && card side done))`:
  0 means nothing in flight and nothing to come (m1), on every path; the
  write phase's term makes DMA_BUSY drop in the very cycle DATA_DONE rises.
- **Ends** (SD_NATIVE_PLAN §8.4): DMA_EN = 0 and clearing SD_MODE finish the
  block in progress (block-granular `SDM_DMA_ADDR`; the card side goes on as
  PIO); a FIFO reset or the native reset stop after the transaction in
  flight; CMD12 and card errors on a read move the good blocks still in the
  FIFO first.  While DMA_BUSY the FIFO port is the engine's (`SDM_DATA` reads
  0 / stores set FIFO_ERR, FIFO_THRESH 0) and data commands are ignored.
- **Master E** (`mem_read_write.sv`): a fifth requester on the arbiter's
  muxes, narrow (16 B, `mig_wide` = 0), a DMA master to the L2 (`i_dma`),
  priority VGA > SD > blitter > core 2, granted only from the no-grant state
  with no cache miss in flight and `o_mnt_busy` = 0 (the blitter's gate),
  released by its done, with the orphaned-grant guard.  With no SD request
  the arbiter's choices are those of `c0bf7b2`.
- **Range walk** (`mem_read_write.sv`, the MAINT sub-FSM).  `RANGE_CLEAN` /
  `RANGE_INV` come from the SoC as a one-cycle pulse with the op's first line
  and ceil(LEN / 32) lines (`KlaussCPU.sv` keeps `CACHE_OP_ADDR` / `LEN` and
  the count, registered).  One range op waits in `r_rng_*` with its copied
  range; one that finds that slot taken, a whole-cache walk pending, or that
  covers more than 2,048 lines becomes a whole-cache walk (FLUSH, or
  INVALIDATE if either op invalidates: a superset).  The walk visits
  min(n, 1024) sets from the first line's index; at visit j a way is in the
  range iff its tag is T_j (= (L0 + j)[26:10]) or, while j < n − 1024, T_j + 1
  (`r_rw_tag` / `r_rw_tag1` / `r_rw_two`, two 17-bit compares).  `MS_READ`,
  `MS_RCHK`: with nothing dirty among the selected ways they are dropped
  (RANGE_INV) and the walk moves on — 2 cycles a set; otherwise the whole
  walk's `MS_W0` … `MS_CLR`, qualified by the selection (`w_ms_sel*`, 1 for a
  whole walk, so its behaviour is unchanged).  **RANGE_INV writes a dirty
  line back before it drops it** (the spec; see the disagreements).
  `o_mnt_busy` also covers the pending range op.
- **RANGE_IINV** (`pipeline_core.sv`, `RIINV_EN`, core 1 only).  The core
  decodes its own completing store to `CACHE_CTRL` with `[6]` (and neither
  `[1]`, `[2]` nor `[3]`, LEN ≠ 0), exactly where it decodes ICACHE_INV, and
  does everything ICACHE_INV does — squash and refetch the younger
  instructions, empty the window, the NLB, the BTIC (and the ID snapshot),
  drop a pending install — except that its valid-bit sweep is limited to the
  range: the 2,048 valid bits are 32 LUTRAM banks of 64, and at each bank
  address the sweep clears only the banks whose line there is in the range
  (a registered 32-bit mask from two thermometer compares, wrapping), from
  the range's first address for min(n, 64) steps.  A store's snoop clear
  during a range sweep takes the write port and the sweep waits a cycle
  (a whole sweep needs no such care).  As in a whole sweep, every lookup
  misses and installs wait while it runs, so fetch is coherent with the op
  at once.  Emptying the whole BTIC / NLB rather than only the range's
  entries is a superset (16 + 2 entries, refilled in a few branches), and
  keeps the BTIC kill compares — a timing path — untouched.  `rk_busy`
  feeds `MNT_BUSY`.  A range op that meets a sweep already running, or that
  covers the I-cache, sweeps everything; with 16 B lines (core 2's
  geometry, `RIINV_EN` = 0 there anyway) it is a whole ICACHE_INV.
- **(m3)**  *A `CACHE_CTRL` write takes effect only after every earlier
  store is in the D-cache:* the core has one in-order port; an MMIO store
  issues only once the posted store (`st_post`) has had its ready and the
  store queue (`sq_v`) is empty; the D-cache acknowledges a store when it
  latches it (WAIT) and finishes it in CHECK or its miss states; a walk, of
  either kind, starts only from WAIT.  *A data command after a range op
  cannot reach DDR before the walk's write-backs:* the op sets its pending
  flag in its MMIO write's first cycle, the `SDM_CMD` write is a later
  access, master E is not granted while `o_mnt_busy` (pending, queued or
  running), and the walk ends only after its last write-back's ready (the
  L2 has accepted it).  **No software wait is needed** between the
  `CACHE_CTRL` write and `SDM_CMD`, in either direction; tests `rd_m3_*` /
  `wr_m3_*` below, and the mutations `m3_gate` / `m3_pending` that break the
  gate are killed by them.

### §7 verification

- **`tb_sd_dma`** (`tools/sdsim/run_sd_tb.sh dma`, xsim; `SDDMA_SEED`): the
  engine in `sd_spi` against `sd_card_model` and a DDR-port model with
  random grant and ready latencies that checks the master-E contract (DV
  only with the grant, address held, req until done, nothing outside DDR).
  ~625 checks a seed, seeds 1–17 pass: registers; counted reads / writes in
  both lanes, CMD17 / 24, 8- and 64-byte blocks; DATA_DONE only with the data
  in DDR; the refusals (size, range end past DDR, exactly at its end, above
  4 GiB, the registers written back to back before `SDM_CMD`); open-ended
  reads and writes into DDR's end (per-block DMA_ERR, the card side going on
  as PIO); the 4 KiB read-ahead (fetched − sent ≤ 512 words, measured);
  CRC errors (the bad block never in DDR, BLKDONE = good blocks); a bad CRC
  token; the registers during a DMA (DMA_ADDR, `SDM_DATA` both ways,
  FIFO_THRESH with blocks waiting, data commands ignored, CMD13 sent);
  every abort (DMA_EN = 0 on reads and writes, FIFO reset at random points,
  CMD12 on open-ended and counted reads and an open-ended write, SD_MODE =
  0 on both, the native reset with a DDR read in flight); interrupts; 24
  random transfers (sizes 8–512, counts, lanes, latencies).  **(m1)** is
  checked on every one of these paths: once an MMIO read shows DMA_BUSY = 0
  the port must be idle, stay idle, and DDR must already hold the data; a
  monitor also checks every cycle that DMA_BUSY = 0 implies an idle port.
- **`socsddma`** (`tools/vsim/sddma.py`, tb_soc with `sd_card_sync`, a
  synchronous card model for Verilator; the memory-consistency checker sees
  the DMA's transactions): (m3) both directions, the data command written at
  once after the clean (`nowait`), behind another walk (`queued`), after a
  store that misses (`last_store`), and after the poll (`wait`) — a 32 KiB
  dirty range with the DMA buffer at its end, the sequence run twice so its
  code is in the I-cache (else the fetches wait for the walk and the race
  never happens: the first version of these tests could not see the gate
  removed); range ops with partial ADDR / LEN, the lines straddling the
  ends cleaned and their neighbours untouched (D-cache probes), the range's
  precision within a set (the other way's dirty lines stay), RANGE_INV
  writing back, LEN = 0, the queue (a queued op keeps its copied range; a
  second op becomes a whole walk), a range of 1,536 lines (the second tag),
  the combinations with FLUSH / INVALIDATE / ICACHE_INV; RANGE_IINV with
  code rewritten by DMA — stale without it, fresh with it or ICACHE_INV,
  stale after a RANGE_IINV of another range (only the range is dropped), a
  store's snoop during a range sweep, ICACHE_INV after a range sweep — with
  tb_soc's BTIC coverage / currency checks gating every run; the DMA_ERR
  and DATA_DONE interrupts through the INT2 cascade.
- **`socsdvga`**: an open-ended SD DMA at 50 MHz (a read stream: DMA writes
  into DDR; or a write stream) alongside the VGA scanout, `_vga_prog`'s CPU
  miss / write-back traffic, L2 WBINVs every 1,500 or 150 iterations, and
  (`--blit`) the blitter, against the same run without the DMA, at MIGLAT
  36 and 44 (below); the DMA's data checked at the end.
- **Mutations** (`tools/vsim/sddmamut.py`, the unit bench or the named SoC
  tests per mutant): **34 of 34 killed** — DATA_DONE before the data is in
  DDR; DMA_BUSY without the transaction in flight; each refusal and the
  per-block check; the fetch's room and count limits; a FIFO reset that
  leaves the engine waiting; aborts ignored; `SDM_DATA` / FIFO_THRESH /
  `SDM_DMA_ADDR` / data commands not gated; the lane mask; CMD12 dropping
  the received blocks; the adapter reset by the native reset; the DMA_ERR
  interrupt; the m3 gate and the pending op in MNT_BUSY; the walk's second
  tag, its precision, its last set, RANGE_INV without the write-back (the
  emulator's reading), the copied range, the queue, the 64 KiB conversion,
  FLUSH | range bits; the hit pipe during a pending RANGE_INV; RANGE_IINV's mask (none, off by a bank), its snoop
  stall, its decode.  (Not listed: clearing `vl_any` at a range sweep's end
  — equivalent in practice, every fetch during a sweep reinstalls.)

### §7 results

**Walk timings** (tb_soc, cycles at 100 MHz; the emulator's models in
brackets):

| Op | Cycles | Emulator |
|---|---|---|
| RANGE_CLEAN / RANGE_INV, n lines | 1 + 2 × min(n, 1024) + 6 per set with a dirty range line (+3 for a second) | 8 + 2n + 10 per dirty line |
| … 32 lines clean / dirty | 65 / 257 | 72 / 392 |
| … 1,024 lines clean / dirty | 2,049 / 8,193 | 2,056 / 12,296 |
| … 1,536 lines, all dirty | 9,726 | 18,440 |
| RANGE_IINV, n lines | min(n, 64) (+1 per store snoop during it; 0 with the I-cache empty) | 8 + 2n |
| … 1 / 32 / 64 / 128 (4 KiB) / 2,048 lines | 1 / 32 / 64 / 64 / 64 | 10 / 72 / 136 / 264 / 4,104 |
| FLUSH / INVALIDATE (whole) | 4,097 + ~3 per dirty line, **run twice** (below; once since review 1) | 2,048 + 10 per dirty line |

RANGE_IINV: one line a cycle up to 64 lines, then n / 64 lines a cycle (a
4 KiB page: 2 lines a cycle; 64 KiB: 32).

**A finding in the existing design (not changed here, to keep `c0bf7b2`'s
timing):** every whole-cache FLUSH / INVALIDATE walks the cache twice.  The
SoC's decode of `CACHE_CTRL[2:1]` is a level for the store's whole MMIO
strobe (~4 cycles); the walk starts a cycle after the first, its start clears
`r_mnt_pending`, and the strobe still high re-arms it, so a second walk
(4,097 cycles, nothing dirty left) follows every time.  A whole-cache op
therefore keeps MNT_BUSY (and the core's cached accesses) for ~8,200 cycles
+ ~3 per dirty line.  The range ops' go is the strobe's first cycle (as
L2_CTRL's), so they walk once.  Fixing it (an edge on the whole-cache go)
is a one-line change; it changes the timing of every program that uses
FLUSH / INVALIDATE.

**VGA under an SD DMA stream during L2 WBINVs** (the l2b delta review's
question; `socsdvga`, 120+ display lines, underflows = starved lines):

| MIGLAT | WBINV every | Blitter | SD DMA | Underflows | Worst line fetch (of 3,200) | VGA's worst grant wait | SD: worst DV → ready | SD: worst req → ready |
|---|---|---|---|---|---|---|---|---|
| 36 | 1,500 | — | off | 0 | 2,766 | 82 | — | — |
| 36 | 1,500 | — | read | 0 | 2,785 | 96 | 29 | 2,877 |
| 44 | 1,500 | — | off | 0 | 3,013 | 90 | — | — |
| 44 | 1,500 | — | read | 0 | 3,021 | 92 | 35 | 3,139 |
| 36 | 150 | — | off | 0 | 2,388 | 68 | — | — |
| 36 | 150 | — | read | 0 | 2,392 | 84 | 16 | 2,457 |
| 44 | 150 | — | off | 0 | 2,628 | 76 | — | — |
| 44 | 150 | — | read | 0 | 2,630 | 92 | 17 | 2,749 |
| 36 | 150 | FILLs | off | 0 | 2,439 | 134 | — | — |
| 36 | 150 | FILLs | read | 0 | 2,407 | 129 | 13 | 2,551 |
| 36 | 150 | FILLs | write | 0 | 2,430 | 113 | 86 | 2,555 |
| 44 | 150 | FILLs | off | 0 | 2,644 | 142 | — | — |
| 44 | 150 | FILLs | read | 0 | 2,644 | 152 | 12 | 2,800 |
| 44 | 150 | FILLs | write | 0 | 2,625 | 125 | 92 | 2,787 |

No line starved with the DMA; the worst line fetch moves by at most 0.7 %.
The SD DMA's writes into the walked set did not show as long waits (worst
DV → ready 35 cycles for the read stream; a write stream's reads, 92); VGA's
worst grant wait rises by at most 14 cycles (none with the blitter).  Its worst wait for the bus
(~2,500–3,100 cycles) is the arbiter's rule for every DMA master — no
grant while a CPU miss is in flight, VGA first — under this miss-heavy loop;
the 4 KiB FIFO absorbs it (the card's clock stops between blocks).

**Regression** (against `c0bf7b2`): `l2_regress.sh` core (the 5
configurations — default, DEBUG_EN, MPU, 16-entry TLB, no walker — diff /
diff_irq / excdiff / mpudiff / tlbdiff / tlbpage / dcbzdiff / dmtest, btfuzz
300 seeds), soc36, soc8, socvar (L2 off, 8 ways, jittered and slow MIG), eq
(`L2_EN` = 0 against `c0bf7b2` under the same tb_soc) and the tb_l2 sweep
(120 of 120): all pass.  l2mut's inputs (`l2_cache.sv`, `tb_l2.sv`,
`mig_model.sv`, `l2mut.py`) are byte-identical to `c0bf7b2` (36 of 36
killed there); its first 7 mutants were re-run here (all killed) and the
run was then stopped.  `tools/sdsim/run_sd_tb.sh`: tb_sd_native
337 checks, tb_sd_spi_compat, tb_sd_dma seeds 1–17.  **DMA_EN = 0 is
cycle-identical to `c0bf7b2`**: `socdiff --cycles` (identical retire traces
*and* PIPE_RUN cycles, per program) over 30 randprog seeds with the L2 and
20 without, the regression's socdiff / eq totals, and `_vga_prog` with WBINVs
every 300 iterations, with and without the blitter (cycles, display lines,
worst fetch and MIG bursts identical).  The CPU_FEATURES expectations
(socmmio-style checks, soccrypto, socwalk, socsd) are updated to `[27]`;
soccrypto's full-configuration value was stale on `c0bf7b2` (`0x01FF200D`)
and is now derived.

**Utilisation** (synthesis only, `44580c7` vs `13be986` = `c0bf7b2`'s RTL,
`build_fast -stop_after_synth`): LUTs as logic 35,321 (+813), LUTs as memory
3,932 (+0), registers 24,535 (+780), BRAM 124 (+0), DSP 20 (+1: the
up-front range check's size × count).

**A second finding, fixed for the range ops only: loads could pass a
pending invalidate.**  The documented rule is that the CPU's next cached
access waits for a walk.  A walk starts only when the D-cache is idle, so
one requested while the cache finishes an earlier store's miss is pending
for up to that miss; meanwhile the load-hit pipe (HW_PERF_PLAN §1a stage 3,
its own read port) still answered loads — from lines the walk was about to
invalidate.  `inv_hit_whole` (a store that misses, then INVALIDATE, then a
load of a line the invalidate should drop, with a DMA's new data in DDR):
the load returns the stale line on `c0bf7b2` and here; `inv_hit_wait`
(MNT_BUSY polled first): the new data.  For RANGE_INV the hit pipe is now
held off while the op is pending (`r_h_conf` += `r_rng_pend && r_rng_inv`;
`inv_hit_range`, mutation `hit_pend`), so a load after a RANGE_INV always
sees memory.  The whole-cache INVALIDATE is left as it was (the same one
term, `r_mnt_pending && r_mnt_mode`, would fix it, and change those programs'
timing): software must poll MNT_BUSY after INVALIDATE before reading the
data (the kernel does, after every op).

**(m3): the answer for the runtime session.**  The `MNT_BUSY` poll between a
range op and `SDM_CMD` is **not needed**: the engine cannot touch DDR while
a walk is pending or running, and the walk's end means its write-backs are
in the L2 (`wr_m3_nowait` / `_queued` / `_last_store`, `rd_m3_nowait` /
`_queued` pass without it; `m3_gate` and `m3_pending` are killed by them).
After RANGE_INV the CPU's loads now wait for the walk too.  The waits that
remain useful: before writing `CACHE_OP_ADDR` / `LEN` (with interrupts off,
so a handler's op cannot interleave its registers — the queue holds one
range op, a second becomes a whole-cache walk, correct but slower), and
after a whole-cache INVALIDATE before reading (above).

**Disagreements with the spec / the emulator** (decided as stated; for the
runtime session to confirm):

1. **RANGE_INV alone.**  Spec v1.2: "write back, then invalidate".  The
   kernel's header (`KL_CACHE_RANGE_INV`: "alone: drop them, no
   writeback") and the emulator's coherence checker ("range invalidate (no
   clean) dropped line …: its data is lost") read it as a drop without
   write-back.  The RTL follows the spec.  For correct software (a buffer
   cleaned before the device writes it and left alone during the DMA) both
   readings give the same memory; they differ only for a dirty line, which
   the hardware writes back where the emulator would report lost data.
   The mutation `rinv_nowb` (the other reading) is killed by `range_inv`.
2. **CACHE_OP_ADDR width**: 32 bits in the RTL (MMIO_MAP's width; the
   D-cache tags a 32-bit PA); 64 bits in the emulator.  An address written
   above 4 GiB reads back differently (and means nothing either way).
3. **The range-op queue**: the emulator queues any number of ops, each with
   its range; the RTL holds one and turns a second into a whole-cache walk.
4. **Abort granularity.**  The emulator stores each read block whole as it
   arrives (so after DMA_EN = 0 every received block is in DDR, and DMA_BUSY
   reads 1 for 200 more cycles); the RTL moves blocks as fast as DDR takes
   them, finishes the block in progress on DMA_EN = 0 / SD_MODE = 0 and
   leaves any further committed block in the FIFO for PIO (`SDM_DMA_ADDR`
   says how far it got).  A FIFO reset stops the RTL after its 16-byte
   transaction (not block-granular).
5. **PIO after an abort**: the emulator gives the FIFO back to `SDM_DATA` as
   soon as the phase is no longer a DMA phase; the RTL when DMA_BUSY drops.
   Likewise a data command written while DMA_BUSY (DATA_BUSY clear: after a
   CMD12 / abort, the engine finishing) is ignored by the RTL (it would race
   the engine for the FIFO); the emulator accepts it.
6. **A DMA read uses the FIFO**: words left in it before the command would
   be moved to DDR ahead of the card's data; the emulator's card bypasses
   the FIFO.  Drivers reset the FIFO before a data command (the kernel does).
7. **RANGE_IINV** drops the whole BTIC, NLB and fetch window (and refetches),
   not only the range's entries: a superset, invisible except in cycles.
8. **Walk timings** differ from the emulator's models (the table above),
   and every whole-cache walk runs twice (the first finding).

### §7 review 1 (branch `sddma2`, rebased onto `tmg` dea9b1b)

`sddma` was rebased onto `tmg` as one commit (the net diff from `c0bf7b2`,
the `pipeline_core` and HW_PERF_PLAN conflicts merged by hand) and pushed as
`sddma2`; `sddma` is unchanged.

**Fixes**

| Finding | Fix |
|---|---|
| MAJOR 1: a load could hit a line that a pending whole INVALIDATE (a converted RANGE_INV, or a plain INVALIDATE written while a walk ran) was about to drop | the hit pipe's conflict `r_h_conf` also holds while `r_mnt_pending && r_mnt_mode`; also fixes the pre-existing case of a load passing a pending whole INVALIDATE (`inv_hit_whole` now expects fresh data) |
| MAJOR 2: a converted op whose go landed in the cycle the pending whole walk started was lost (`r_mnt_pending` cleared a cycle late, after the new go had set it) | `r_mnt_pending` clears on the FSM's start condition `w_whole_start` itself; a go in that cycle wins and re-arms it (mode = this go's, or the pending one's INVALIDATE) |
| the whole-cache walk ran twice for one FLUSH / INVALIDATE (the go was a level for the write's whole MMIO cycle) | the CACHE_CTRL go is the write's first cycle, registered (`r_cc_go`, `r_cc_d`); one walk per op |
| `DMA_ERR` set by an open-ended block refused in the cycle an abort lands | `w_dma_seterr` masked with `w_dm_abort` (SD_NATIVE_PLAN §8.4) |
| RANGE_IINV ignored with FLUSH / INVALIDATE | honoured (as ICACHE_INV is): `icinv_r` no longer excludes `[1]` / `[2]` |
| MMIO_MAP: ceil(LEN / 32) lines from `ADDR & ~31` miss the tail of an unaligned buffer; the text on loads after RANGE_INV / INVALIDATE | written: LEN + (ADDR & 31); loads wait for a pending or running walk, no poll needed |

Walk cycle deltas against `tmg` (tb_soc, the whole program): FLUSH and
INVALIDATE of a clean cache −4,097 (8,259 → 4,162; 8,262 → 4,165), the
second walk gone; with 256 / 2,048 lines just made dirty 0 (the cache was
still busy with those stores when the CACHE_CTRL strobe ended, so the walk
started after it and was never repeated); INVALIDATE of 256 dirty lines
followed by a load −3 (MAJOR 1: the load now waits for the walk instead of
hitting the old line; the program ends with the walk's MNT_BUSY poll either
way, so only the poll's phase moves).  Programs without maintenance ops and
every DMA_EN = 0 program: identical (`socdiff --cycles`, below).

**Tests added** (`socsddma`, all pass): the reviewer's `conv_hit_big` /
`conv_hit_ok`, `b2b`, `iinv_wrap80` / `iinv_wrap101` / `iinv_wrap_end`,
`two_wrap` (a 1,536-line RANGE_CLEAN from set 960: the range's second tag
in its sets) and `two_wrap_full` (the reviewer's DCP2 expectation (1, 0) was
wrong: the 64 KiB fill and the store at E, A's set, evict A — written back,
so the DMA still reads it: (0, 0)); `lost_{fr,fi,if}_{0..119}` (FLUSH then
INVALIDATE, FLUSH then RANGE_INV, INVALIDATE then FLUSH at every distance d
in cycles: one walk each, nothing lost); `iinv_flush` / `iinv_inval`.
tb_soc counts maintenance ops (written, absorbed into a whole walk, walked)
and fails a run that loses one.  `tb_sd_dma`: a CMD12 after a counted read's
card side is done (DATA_BUSY held by `r_dd_pend` until the data is in DDR,
then DATA_DONE) and a response error on a DMA read / write (no data phase;
a write may have fetched into the FIFO, nothing written to DDR).  The
reviewer's exhaustive `f_rkm` model (`rkm.py`, every start line × 270
lengths): 0 bad.

**Mutations** (`sddmamut.py`): `hit_pend_whole` (MAJOR 1's term dropped:
`inv_hit_whole`), `lost_start` (MAJOR 2: the clear a cycle late: `lost_*`),
`whole_level` (the go as a level: the walk lists), `iinv_with_whole`
(FLUSH / INVALIDATE drop RANGE_IINV: `iinv_flush`), `hit_pend` (updated),
`fwd_tb_md` / `fwd_ta_md` (below) — all killed; the full set: 40 of 40 killed.

**Forward fuzzer** (`tools/vsim/fwdfuzz.py`, `run.py fwdfuzz`; the tmg
review): producers (SHL / SHR / SAR / ROL weighted, ALU, ALU immediate,
load, MUL, POP) into r0..r11 / r13 feeding every memory-op mode — `[rs1]`,
`[rs1+simm8]`, `[rs1+imm32]`, `[imm]`, `[rs1+rs2]` as base and as index,
store data, PUSH / POP, MEMGET32, and the aliases (rs2 = 0 of a long /
short form, rs1 = 0 of `[imm]`) — at distance 0..2, at +LAT 1 / 2 / 3 / 5 ×
+HIT 0 / 70 / 95, against a base revision, optionally through tb_soc.  vs
`tmg`: 144 tb_vsim runs (12 seeds) and 8 tb_soc runs, 0 differing.  It
kills `t_tb_d`'s `t_md == 2'b11` term dropped (`fwd_tb_md`) and the matching
`t_ta_d` one (`fwd_ta_md`): the shift-last forward's assertion fires.

**Stop latency** (runtime-96's `KSD_DMA_STOP_US`): SD_NATIVE_PLAN §8.4 —
DMA_EN = 0 → DMA_BUSY = 0 at most 19,105 cycles measured (a write stream,
MIGLAT 44, the CPU loop still running; 7,657 with a tight poll), a FIFO
reset 2,691; per-transaction bus waits max 2,791 cycles.

**Confirmed and documented** (runtime-96): (a) a CMD12 during a counted read
whose card side is done leaves DATA_BUSY up until the engine has moved the
data — intended (`r_dd_pend`: DATA_DONE means "in DDR"); (b) a data command
whose response fails runs no data phase, but **its DMA phase has begun**: a
write fetches from DDR into the FIFO from the command's acceptance (ahead of
the response) and stops after the block in progress, so DMA_BUSY rises
briefly and the FIFO may hold data (DDR is never written); a read moves
nothing.  Intended (the fetch-ahead keeps the write's first block ready for
the card), but it differs from the emulator, which starts no DMA phase
(SD_NATIVE_PLAN §8.4).  MMIO_MAP's range-op queue wording is runtime-96's.

**Builds** (standard flow, `build_fast -resynth` under the lock):

| Build | Commit | Change | Tier 1 WNS / WHS | Final | LUTs | FFs | BRAM |
|---|---|---|---|---|---|---|---|
| `sddma_b1` | `516950b` | the rebase + the review-1 fixes | −1.081 / +0.031 | tier 2 −0.621, stopped | | | |
| `sddma_b2` | `2d7bc79` | RANGE_IINV's sweep mask from registers; the CACHE_CTRL go registered | −0.280 / +0.020 | stopped | | | |
| **`sddma_b3`** | **`3ae5ed6`** | **the MEM shifter as one rotate; wb_zero of a shift from byte flags; CACHE_CTRL's completion term** | **+0.041 / +0.034** | **tier 1, met** | **41,089** | **27,475** | **126** |
| `sddma_b3` alt `ExtraTimingOpt` | `3ae5ed6` | second build: another placer directive | +0.084 / +0.017 (routed, met) | cross-check | | | |

(`tmg_b3`: 41,001 LUTs, 26,868 FFs, 126 BRAM; 20 DSPs both.)

| Family (failing) | Build | Fix | Slack in `sddma_b3` |
|---|---|---|---|
| `m_addr` → `icinv_r` → `rk_go` → `f_rkm(…)` → `g_vl.r_sw_mask` (mine) | b1 −1.081 | both candidate masks computed from registers (the SoC's `r_cop_*`, the sweep's own), `rk_go` only selects; LEN ≠ 0 and the "whole cache" test registered | +0.668 |
| `m_addr` → the bus splitter's MMIO decode → `w_cache_rng_go` → conversion → `r_rng_pend` (mine) | b1 −0.560 | the CACHE_CTRL go and data bits registered in the SoC (`r_cc_go`, `r_cc_d`); still cycles ahead of the store's ready, so the (m3) order holds (`*m3*` tests) | +1.152 |
| `r_shhi` → the shifter's 2nd stage → `w_sh2` → the forwards into `ex_a` / `ex_b`, and → `wb_zero` (tmg's) | b2 −0.280 / −0.239 | stage 1 picks the kind: every shift is one byte rotate of one register `r_src` by `r_rk` plus a byte keep / fill (SHL keeps bytes ≥ hi, SHR / SAR bytes < 8 − hi, SAR fills the sign), BTS / BTR / BTC / BTSTRR done in stage 1; `wb_zero` of a shift = AND of the byte-zero flags. Asserted against the plain shift | ex_a not in the worst 40 (≥ +0.342); wb_zero +1.028 |
| `mem_d.uop` → `mem_is_write` → `icinv` → `pos` → `r_in0` | b2 −0.073 | a CACHE_CTRL store is MMIO, so only the port's terms of `mem_done_now` can complete it (`cc_done`; asserted equal) | +0.819 |

`sddma_b3`'s worst paths: `ifb_dw` → `id_ser` +0.041 and → `id_privop`
+0.105 (ID's legality decode), the L2's `e_bc` → `o_mem_read_data` +0.067,
`mem_d.uop` → `ex_d` resets / `ex_pred_tgt` CE +0.158 — all `tmg`'s
families, unchanged here.  Mine: `r_mnt_pending` +1.462, `r_h_conf`
+1.695, `r_grant_sd` (the arbiter's fifth leg) +2.633, `sd_native` +0.996.
Bitstream `~/.cache/kbuild/sddma_3ae5ed6.bit`, md5
`4386fa73003a37da1b76399e1b4701f2` (build dir `~/.cache/kbuild/sddma_b3`;
the alt build's in `alt_eto/`).  Not run on the board.

**Known, deferred** (pre-existing, not changed here): a LEAVERET whose
frame is popped from MMIO deadlocks (tb_vsim DEADLOCK sm=16; also on `l2b`);
`mpudiff`'s harness is stale (it fails on `l2b`; the MPU is off on the A7) —
in this branch's core regression against `tmg` it reports 0 differing.

**Regression** (on the final RTL `3ae5ed6`, against `tmg` where an A/B),
all passing:

- core (`l2_regress.sh core`, BASE = `tmg`): `test` in the five configs
  (default, DEBUG_EN, MPU_EN, TLB_N = 16, no walker); `diff` 40 seeds ×
  LAT 1 / 3 / 7 and `diff --irq` 40 seeds: 0 differing, cycles identical
  (169,092 / 107,709); `excdiff`, `mpudiff`, `tlbdiff`, `tlbpage`,
  `dcbzdiff` 200 seeds, `dmtest`, `btfuzz` 300 seeds.
- `fwdfuzz` vs `tmg`: 12 seeds × 12 latency / hit configs (144 runs) and 8
  tb_soc runs: 0 differing.
- `l2_regress.sh` mut, soc36, soc8, socvar, eq (BASE = `d7f3994`, the
  L2-off equivalence): 0 failing; `l2mut` 36 of 36 killed.  (The tb_l2 sweep not rerun:
  `l2_cache.sv` is unchanged.)
- `tb_sd_dma` seeds 1–25 (644–659 checks each), `tb_sd_native`,
  `tb_sd_spi_compat`; `socsd`, `socsddma` (all 402 tests), `socsdvga`
  (MIGLAT 36 / 44, 0 underflows, as before), `socsdabort`; `sddmamut.py` 40
  of 40 (42 of 42 with the delta-review mutants).
- Cycles vs `tmg`: `socdiff --cycles` 30 seeds identical (PIPE_RUN 61,698
  both); `socbench` data96k / code96k / chase112k identical (382,024 /
  142,666 / 199,846).  Only programs with a whole FLUSH / INVALIDATE change
  (above).

### §7 delta review (branch `sddma2`, after review 1)

- MINOR (latent): the registered CACHE_CTRL go (`r_cc_go` / `r_cc_d`,
  KlaussCPU.sv) sets the pending-walk flag that holds loads (`r_h_conf`:
  `r_rng_pend && inv`, `r_mnt_pending && r_mnt_mode`) exactly one cycle before
  the core completes the store (DV to core completion = 3 cycles, measured;
  a 1-cycle later go still passes, 2 returns stale data).  A shorter MMIO
  write ack would silently reopen MAJOR 1.  Now a sim-only assertion
  (`ifndef SYNTHESIS`, `$fatal`) fires when a CACHE_CTRL write's ready
  (`r_mmio_write_dv_d2`) arrives without `r_cc_go` having been high in an
  earlier cycle, so it also flags the lost margin at +1; the comment at
  `r_cc_go` states the margin.  `sddmamut.py`: `cc_go_late1` (only the
  assertion kills it) and `cc_go_late2`, 42 of 42.
- NIT: the whole-cache decode needs `byte_en[0]`, as MMIO_MAP says.
- NIT: the RANGE_IINV registered-mask mutant is killed only by `iinv_wrap101`
  / `iinv_wrap_end`.

### §7 deferred

- (Done in review 1: the build, the double walk, the hit pipe during a
  pending whole INVALIDATE.)
- A range walk at one set a cycle (a second read port or a read-ahead of
  the next set); a BTIC kill by range instead of emptying it.
- Board test: the kernel's driver with `klausscpu_sd.dma=1`.

## CPU_FEATURES additions

| Bit | Meaning |
|---|---|
| `[22]` | DCBZ (§1) |
| `[23]` | STATUS.MAL64 (§4) |
| `[24]` | `ETH_RX_SLOTS` (§3) |
| `[26]` | the unified L2 (§6) |
| `[27]` | the cache range operations: RANGE_CLEAN / RANGE_INV / RANGE_IINV (§7; the SD DMA engine itself is `SDM_CAPS[10]`) |

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
  they are also write hits / misses. A DCBZ's dirty victim goes out from
  WRITE_FETCH directly, without the 3-cycle EVICT_SHADOW_GAP: no refill
  was issued, so ddr2_control is already waiting for a DV.
- **Throughput** (xsim cache bench, `run_m10_cache.sh` Phase P: 64 KiB
  back-to-back, cycles per 32 B line, real ddr2_control on the bench's MIG
  model). Clean or invalid victim: DCBZ 3 (WAIT, CHECK, WRITE_FETCH, one
  each), 4 stores 27 (19 waiting for the fetch). Dirty victim: DCBZ 13 (10
  waiting for the writeback; 16 with the gap), 4 stores 40. With
  scanout-shaped VGA traffic a clean DCBZ stays at 3: it needs no DDR
  access and nothing waits for the grant. The first board run
  (`tools/board/dcbz_test.c`, 88acc21) showed 18.1 cycles a line for
  clean DCBZ because the loop reloaded the global line size after every
  DCBZ (the asm's `memory` clobber). The MEMGET32 → add → address chain
  alone costs 14 cycles a line in tb_soc (the same machine code with the
  DCBZ replaced by a NOP), and a MEMGET32 right behind a DCBZ waits about
  4 more. dcbz_test now keeps L in a local and does 4 lines an iteration.
  That loop measures 3.25 (clean) and 13 (dirty) cycles a line in tb_soc.
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
- 2026-11-11: §2 timing closed on `perf-hw` (`be78350`): the 16 KiB build
  missed (−0.185); valid bits restructured, the fetch tag moved to LUTRAM,
  three CPI-neutral core fixes; standard flow tier 1 WNS +0.033 / WHS
  +0.024, `~/.cache/kbuild/perf-hw_be78350.bit` (§2 timing closure). Not
  yet run on the board.
- 2026-11-11: after the CALL-squash fix (`c1874cf`, tier 1 −0.298),
  `a5d20f2` (EI raw-bit match, the core's ready off the ownership mux):
  tier 1 WNS +0.016 / WHS +0.024, `~/.cache/kbuild/perf-hw_a5d20f2.bit`
  (§2 timing after the CALL-squash fix). Not yet run on the board.
- 2026-11-14: §5 64 KiB I-cache with 32 B lines on branch `icache`
  (pipeline_core, mem_read_write, KlaussCPU, core2_subsys parameters;
  ICACHE_INFO `0x0001_0000_2008_0001`); the §11.9 `f_sf_*` fix; standard
  flow tier 2 WNS +0.013 / WHS +0.021, `~/.cache/kbuild/icache_5bcf24e.bit`.
  Not yet run on the board.
- 2026-11-15: §5 additions (PERF_IC_MISS fix + PERF_ITLB_REFILL; the
  4-entry tagged micro-ITLB) on `icache`: `112fc8b`, standard flow tier 1
  WNS +0.070 / WHS +0.011, `~/.cache/kbuild/icache_112fc8b.bit`.  Not yet
  run on the board.
- 2026-11-15: §5 review fix-ups (install-invalid on a dropped install,
  PERF_IC_FILL_CYC / PERF_ITLB_REFILL, CE_TIED, new directed tests) and
  the shifter zero-test timing fix: `bb23b01`, standard flow tier 2 WNS
  +0.007 / WHS +0.050, `~/.cache/kbuild/icache_bb23b01.bit`.  Not yet run
  on the board.
- 2026-11-15: §5 review 2 (M1: BTIC captures across a 32 B install, with
  the branch-line and window / NLB / ID cases; M2: the BTIC coverage
  check on every tb_vsim / tb_soc run, `btfuzz`): `823b5ea`, standard
  flow tier 1 WNS +0.017 / WHS +0.024, `~/.cache/kbuild/icache_823b5ea.bit`.
  Not yet run on the board.
- 2026-11-15: §5 review 3 (a branch across a line end arms no capture;
  the pending capture ends at any other redirect / squash / snoop; three
  16 B-line coverage holes; the checks cover the branch's last word, the
  kill indices, 16 B lines and BTIC-hit currency; `btfuzz` branch sites)
  and two TLB CAM timing changes: branch `btic-fix`, `079d686`, standard
  flow tier 1 WNS +0.062 / WHS +0.014, `~/.cache/kbuild/btic_079d686.bit`.
  Not yet run on the board.
- 2026-11-15: §6 unified L2 (128 KiB, 4-way, 64 B lines of two 32 B
  sectors, below the DDR arbiter: DMA goes through it), `L2_INFO`,
  `L2_CTRL`, `CPU_FEATURES[26]`, the `PERF_L2_*` counters; tb_soc on a
  behavioural MIG with a memory-consistency checker; `tb_l2`, `l2mut`,
  `socl2`, `socbench`, `socvga`: branch `l2`, `fba47c0`, standard flow
  tier 2 WNS +0.030 / WHS +0.020, `~/.cache/kbuild/l2_fba47c0.bit`.  Not
  yet run on the board.
- 2026-11-16: §6 review 1 (the WBINV walk livelock, drains in VGA's
  window, the walk's turns and end, PA[27] aliases, a DRAM timing model
  for the MIG with the UG586 check, a VGA stream buffer that made the L2
  build starve less than the build without it): branch `l2`, `c9cadea`,
  standard flow tier 1 WNS +0.040 / WHS +0.010,
  `~/.cache/kbuild/l2_c9cadea.bit`.  Not yet run on the board.
- 2026-11-16: §6 review 2 (posted-write streams held BUSY up: the walk
  waiting on the buffer is a request in hand, its slot reserved, writes to
  its set held — BUSY now bounded under any request stream; exact drain
  probes; the stream buffer's capture edge tested; no stream buffer in
  bypass): branch `l2b`, `13be986`; standard flow tier 1 −0.187 on core /
  D-cache paths (placement: the L2's area equals `c9cadea`'s), alt-place
  `ExtraPostPlacementOpt` +0.028 / +0.038,
  `~/.cache/kbuild/l2b_13be986_epp.bit`.  Not yet run on the board.
- 2026-11-16: §7 SD DMA engine (SD DMA spec v1.2: `SDM_DMA_ADDR`,
  DMA_EN / DMA_BUSY / DMA_ERR, master E) and the cache range operations
  (RANGE_CLEAN / RANGE_INV / RANGE_IINV, `CACHE_OP_ADDR` / `LEN`,
  `CPU_FEATURES[27]`): branch `sddma`, RTL and simulation only (tb_sd_dma,
  socsddma, socsdvga, 34 of 34 mutations, the L2 regression; cycle-identical
  to `c0bf7b2` without DMA); synthesis +813 LUTs.  Not yet built (waits for
  `tmg`).

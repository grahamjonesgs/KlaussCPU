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

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

## Status

- 2026-11-11: plan and specification agreed with the runtime session.
  Implementation has not started.

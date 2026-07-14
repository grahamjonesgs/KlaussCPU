# M7b — I-cache store-snoop: index-only → tag-checked (LVGL hang root cause)

**Status: IMPLEMENTED + sim-verified (M5a goldens, M5c WAIT/SMC/storms, M5d SoC
boot all PASS). The earlier storm regression is ROOT-CAUSED and fixed — it was
not a tag-check bug but a latent M7a-introduced IF-fill vs IRQ-frame-push port
race (see §Storm regression below).**

## Symptom
Zephyr LVGL (`build_lvgl/zephyr/zephyr.elf`, 3 MB image) loads on the board
("Load Complete OK") then produces **no UART output** — appears hung. Regression
suite (hello/bst/expr/test_64bit/queens/crypto/dhrystone) + test_rtos all PASS
on the same M7a bitstream.

## Root cause (hazard-counter-confirmed)
Board test: clear perf counters (`perf/m5a/perf_dump.kla` reads 0xF00D_00B0-E8),
run LVGL 45 s (hangs), dump the surviving counters. Result over 4.58 B cycles:
- **INSTR = 16 M → CPI ≈ 286** (normal ~4). Not wedged — *crawling* ~100× slow.
- **IF_MISS = 2.35 B (51 % of cycles)**, **MEM_WAIT = 2.32 B (51 %)**.

The pipeline spends essentially every cycle on fetch-misses + memory waits.
Cause = the M7a I-cache **store-snoop is index-only** (`pipeline_core.sv`
~L1178, `mem_port_wr → invalidate ic_v{hi,lo}[mem_iaddr[12:4]]`, no tag read).
LVGL writes a 640×480 framebuffer + large `.bss`; those data-store addresses
**index-alias** the 512-line I-cache, so each aliasing store spuriously evicts a
cached **code** line — including the executing loop's own code (a `.bss`-zero /
memcpy loop stores to addresses that alias its own instruction line) → refill
every iteration → the ~51 % IF_MISS thrash. LVGL grinds through early boot so
slowly it never reaches its first `printf` in a human's patience window.

The M7a RESUME note already flagged this: *"the current index-snoop is already
SMC-correct for the regression"* → M7b = tag-checked. The regression suite is
small/compute-bound so it never index-aliases code with data writes; LVGL does.

## Why this is a real bug, not a test artifact
Two false leads were ruled out first:
- **eth_mmio_bridge**: an `eth_probe2.kla` "reproduction" crashed in tb_soc, but
  that was MY test's bug — the `.kla` assembler sets `heap_start = 0x174` (NOT
  8-byte aligned, unlike the ELF `build_ddr_image` path), so the dword-granular
  boot copy dropped the last dword (the `RET`). Not the pipeline.
- **Core can't run LVGL**: false — the standalone `tb_pipeline_isa` runs LVGL and
  prints the banner + render header (299 UART bytes). The core executes it fine;
  the board just crawls.

## The fix (tag-checked snoop)
Add `ic_tagff[0:511]` — an FF shadow of the I-cache tags, written on every
install alongside `ic_tag`, read combinationally by the snoop so it invalidates
**only** when the store's tag matches the cached line's tag
(`ic_tagff[idx] == mem_iaddr[31:13]`), i.e. only when the store actually
overwrites that cached code line. Bit alignment verified: install tag =
`if_req_dw[28:10]` = byte[31:13]; snoop tag = `mem_iaddr[31:13]`; index
[12:4] both sides.

## Storm regression: root-caused (NOT the tag-check)
The first attempt regressed the M5c IRQ storm (bst @997):
`InvalidOpcode(0x7b8)` at i=59535 after an IRET. Instrumented monitors
(`m_read_DV && m_write_DV`, IF-issue-during-IRQ, IRQ-push-with-live-if_look)
caught the smoking gun at IRQ #310:

- A fetch-window lookup (`if_look`) was live when the IRQ was taken. For 309
  IRQs it resolved as an I-cache HIT (no port use — harmless). At #310 it
  MISSED in the very cycle the IRQ sequencer issued the frame push.
- The IF miss-issue path checked only `!mem_port_op && mem_xc==0`; the IRQ
  sequencer checks only `!if_xc` (not `if_look`). Both issued in the same
  cycle; the IF engine's NBA writes come later in the block, so `m_addr` was
  clobbered from `sp-8` to the fetch address `0x7b8` with `m_write_DV` AND
  `m_read_DV` both high for 5 cycles → **the IRQ frame was written over the
  code at 0x7b8** → InvalidOpcode when execution returned there.

This race is **M7a-introduced and pre-dates the tag-check**: pre-M7a the single
fill-issue point checked `!irq_active`; the M7a lookup/issue split kept the
check on lookup-START but lost it on miss-ISSUE. It is latent in the
board-deployed M7a bitstream (any lookup that goes miss exactly at IRQ-entry
drain corrupts code at the fetch address). The tag-check merely re-timed
hit/miss patterns enough to expose it in the storm test.

**Fix:** gate the IF miss-issue on `!irq_active`
(`pipeline_core.sv` ~L1275). `irq_xc ⊆ irq_active`, so this covers both the
same-cycle race and the mid-push window; `if_look` simply holds and
re-evaluates after entry (hit-serves stay allowed — no port use, and they warm
the window the IRET returns to). No deadlock: the sequencer never waits on
`if_look`.

## Verification
- Sim (all PASS after fix): M5a golden traces hello/bst/expr/test_64bit
  (trace + UART identical); M5c WAIT + SMC + bst storm @997 (89304 instr,
  457 IRQs, program trace identical) + test_64bit storm @463 random-latency
  (150234 instr, 1907 IRQs, identical); M5d full-SoC boot (bst) UART identical.
- Board (M7b bitstream, WNS +0.022): m5e regression 7/7 UART-identical;
  test_rtos healthy (timer IRQ preemption, task interleave correct).

## LVGL RE-DIAGNOSIS: it is a deterministic BUS WEDGE, not (only) thrash
With M7b on the board, LVGL still prints nothing. Counters over a 343 s
window: CYCLES 34.3 B, **INSTR 15,993,341 — exactly +2 vs the M7a 45.8 s hung
run (15,993,339)**, IF_MISS 30.7 B ≈ MEM_WAIT 30.7 B (~90 % of cycles, ~100 %
of the post-load tail). Running 7.5× longer retired 2 more instructions ⇒ the
core is NOT crawling; it **wedges hard at a deterministic instruction**,
identically under index-only (M7a) and tag-checked (M7b) snoops.

Emulator trace pins the site: from i=15,990,511 the code spins in Zephyr's
k_busy_wait poll at 0xcf8e4–0xcf900 — `SETR64 r14,0xF00F0038; MEMGET32
r14,[r14]; SUBR; ZEXT; CMPRR r0(=1,000,000),r14; JMP` — reading **TIMER_COUNT
(0xF00F0038)**. The board retires the emulator's exact stream (wedge lands
mid-loop; no ISR instruction ever retires), i.e. the poll's MMIO read is the
only port op in flight. MEM_WAIT pegged = that read never completes;
IRQ dispatch needs `!mem_busy`, so the pending timer IRQ can never be taken
(irq_active never sets — consistent with IF_MISS counting, which requires
`!irq_active`). The wedge coincides with the first timer fire inside the poll.

Ruled out: `timer_probe.kla` (INT_VEC[0]+period 997+unmask, tight
TIMER_COUNT poll, IRET handler, 200 alignment-swept IRQs) **passes on the
same board bitstream AND in tb_soc**, both with 64-bit MEMREADRR. So the
minimal read-while-timer-fires pattern is fine; some LVGL-state ingredient
(eth live? int_mask combo? cache state?) is required.

## Bus-wedge flight recorder (added for this, kept as a facility)
`KlaussCPU.sv`: if a CPU bus DV is held ~10.5 ms (2^20 cycles), latch ONCE
into `0xF00D_0100` (snap0: valid, pip_bus_idle, if_miss, mem_busy,
w_irq_ready, r_timer_interrupt, w_pipe_owns, dram/eth/mmio readys,
r_mmio_read_dv_d, w_mmio_read_DV, cpu ready/write_DV/read_DV, st.SM,
addr[31:0]) and `0xF00D_0108` (snap1: timer_count[25:0], irq_src1/0,
int_mask, cycles[31:0] at latch). Survives program reloads; cleared by
PERF_CTRL bit 0. Read it after a hang with `perf/m5a/wedge_dump.kla`.
Next step: run LVGL on the recorder build, dump, decode which ready in the
chain is dead (mmio dv path vs splitter vs pipe_owns vs eth decode).

## Remaining M7b-area follow-up (not this change)
A `fence.i`-style MMIO invalidate-all for loaders-under-pipeline (LLEXT/netboot
deposit code as data then jump) — orthogonal to the snoop but the same
coherence area. Note DMA writers (blitter, LiteEth RX) do NOT snoop the
I-cache at all; code deposited by DMA needs that invalidate-all too.

## Repro assets (in perf/m5a/, gitignored)
- `perf_dump.kla` — dumps 0xF00D counters over UART (survive program loads).
- `perf_clear.kla` — clears counters (PERF_CTRL bit 0).
- Board flow: clear → run LVGL 45 s → `perf_dump` → read CYCLES/INSTR/IF_MISS/…
- `mmio_probe.kla` / `eth_probe*.kla` — MMIO-region probes (eth path is FINE;
  keep in mind the `.kla` heap-align gotcha above if hand-assembling).

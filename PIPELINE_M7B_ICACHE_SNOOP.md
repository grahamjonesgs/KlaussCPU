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
PERF_CTRL bit 0. Read it after a hang with `perf/m5a/wedge_dump.kla`,
decode with `perf/m5a/decode_wedge.py`.

## WEDGE ROOT CAUSE (flight-recorder capture): MMIO comb write-ack deadlock
The LVGL capture read: **stuck WRITE to 0xF00F_0000 (INT_MASK) with
cpu.ready=1 and w_mmio_ready=1 held forever**, st.SM=PIPE_RUN, int_mask=0,
timer pending, w_irq_ready=0. Not the TIMER_COUNT read at all — the wedge is
Zephyr's `irq_lock()` = **`load int_mask; store int_mask` back-to-back**.

Mechanism (ready-EDGE protocol deadlock):
- MMIO READ ready = `r_mmio_read_dv_d` (delayed DV) + the bus_splitter output
  FF ⇒ after a read completes, the CPU-visible ready stays stale-HIGH for
  2 cycles.
- MMIO WRITE ready was **combinational** (`w_mmio_write_DV | ...`).
- The pipeline's `rdy_armed` protocol requires ready to be seen LOW after
  issue before a completion counts. A store entering MEM **1 cycle** behind a
  completing MMIO load (consecutive load;store with no data dep — no stall
  separates them) issues into the stale-high tail (armed=0), and its own comb
  ack then bridges the tail: ready NEVER falls ⇒ never arms ⇒ `w_mrdy` never
  fires ⇒ DV held forever ⇒ w_mmio_ready held forever. Total bus wedge; the
  pending IRQ can never dispatch (needs !mem_busy) and fetch starves.
- k=2+ (any instruction between), read;read, write;write, write;read, and
  DDR combos are all safe (their DV gaps or the cache's edge-tracking create
  a ready falling edge). The eth bridge pulse-ack is safe (ack arrives after
  ready has been low). Only the periph-MMIO comb write-ack can bridge.
- Why nothing else ever hit it: hand-written .kla and the suites always have
  ≥1 instruction between MMIO read and write (putc polls status, then SETRs,
  then stores). Compiled C hits it naturally: irq_lock() is exactly the
  load;store pair. The FSM core is immune (its state machine spaces ops).

Fix: `r_mmio_write_dv_d` — the write ack is now delayed-DV (edge-tracking),
identical in shape to the read ack. +1 cycle per MMIO write; DV was already
held multiple cycles so side-effect consumers (UART TX etc., all level-based)
see nothing new.

Verification of the fix:
- `perf/m5a/irqlock_probe.kla` (the k=1 load;store pair): tb_soc DEADLOCK
  (sm=16, i=3) on the old RTL → **OK + clean halt** with the fix (sim AND
  board).
- `perf/m5a/timer_probe.kla` still passes; M5d SoC boot (bst) byte-identical.
- Board: m5e regression 7/7 UART-identical; test_rtos healthy; **LVGL BOOTS**:
  eth PHY ID reads 0x0007 (real value), Zephyr banner + LVGL benchmark header
  print. Counters over the full run: IF_MISS 43.4 M cycles (the wedged run
  read 30.7 B — collapsed ~700×), MEM_WAIT 15.0 M, 21.9 M instr retired.
  Wedge recorder: all-zero (no bus wedge). M7b's success criteria are met.

## Timing closure notes (this change set)
The MMIO restage was forced by closure, not preference: the first fix build
lost the giant `st.SM → w_pipe_owns → addr mux → MMIO read decode` cone
(-0.45), so the read decode now selects from a registered address
(`r_mmio_addr_q`) — MMIO reads are 2-stage and BOTH acks are 2-cycle
delayed-DV (the deadlock analysis in the RTL comment covers the longer tail).
Also: do NOT sample `st.SM` into debug regs — the FSM is tool-re-encoded
(one-hot); a binary cast builds a wide encoder that anchors state replication
(cost ~0.2 ns here; the recorder's [39:32] is now reserved). Final closure
needed post-route phys_opt iterations on the routed checkpoint
(AggressiveExplore / AlternateFlowWithRetiming alternating — WNS -0.131 →
+0.004, hold +0.025); flow scripts: `~/.klausscpu_scratch/build_physopt.tcl`
(enables STEPS.POST_ROUTE_PHYS_OPT_DESIGN) + `physopt_iterate.tcl` (iterates
directives on `KlaussCPU_postroute_physopt.dcp` and writes the bitstream when
met).

## NEW open item: blitter flush corrupts code memory (post-wedge crash)
With the wedge fixed, LVGL boots and starts its render benchmark with
**flush=blitter** (on hardware the app runtime-detects the blitter; every sim
ran flush=memcpy — the blitter path was never exercised under the pipeline).
~21.9 M instr in, HCF crash dump: ERR=01 at PC=0x000D69A4, OPC=00000000,
OPCM/V1H=0xA600A600 — pixel-pattern data physically in DDR where code lives
(the FSM dump re-reads memory). So the blitter flush is writing over code
under the pipeline SoC: wild dest, src/dst mix-up, or blitter-vs-cache
coherency on this path. Distinct from everything fixed here. Next session:
reproduce in tb_soc (blitter_dma is compiled there) with a small blit that
mirrors the LVGL flush rectangle; check the M7a-era note that DMA writers
do not snoop the I-cache either (irrelevant to DDR corruption but same area).

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

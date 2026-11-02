# 1a stage 3: a load-hit pipe beside the cache FSM

Branch `dcache-s3`, on `timing-headroom` (`7d84761`). Status: design. It is
the next RTL step agreed with the toolchain session (2026-11-02), after the
cycle breakdown showed loads are the largest cost.

## 1. Why

On 2a (`a2c9e4b`), loads are most of the remaining stall:

- **Board, -O2, mean over CoreMark and Embench:**
  - `mem_wait` is 18% of cycles.
  - RAW stalls are 20%. A SoC-sim histogram shows most of them are on a load
    producer: CoreMark has ~10 of 13.9 points, and load → FBR alone is 5.2.
- **A hit today, SoC sim, cycles per group:**

  | Group | Cycles |
  |---|---:|
  | `ld` (same address, back to back) | 3.89 |
  | `ld;ld`, different addresses | 5.97 |
  | `ld;use` | 5.03 |
  | `ld;2 independent;use` | 5.05 |
  | `ld;3 independent;use` | 6.08 |

  So:
  - the load-use distance (issue to issue) is **4**;
  - a load holds **2 issue slots**;
  - the cache accepts **one request per 2–3 cycles**.

The reason is structural. Every load goes through the cache FSM: WAIT latches
the request and reads the BRAMs; CHECK compares the tags and registers
data/ready. The request reaches WAIT through the core's port registers, the
`r_pipe_owns` mux and the bus splitter. The response returns the same way, and
the next request can only follow a ready (one transaction outstanding,
`ld_out` / `st_post`).

## 2. Target (for a hit)

| | today | stage 3 |
|---|---:|---:|
| load-use distance, issue to issue | 4 | **3** |
| issue slots held by a load | 2 | **1** |
| independent hits, including the same address | 1 per 2–3 cycles | **1 per cycle** |
| miss | ~20–50 | the same + 1 |

A distance of 2 is out of reach at 100 MHz. It would need BRAM clock-to-out,
tag compare, way and dword select, lane extract, the forward mux and the
operand register all in one cycle.

The FBR is unchanged: it reads its operands at dispatch, like an ALU op.

## 3. Design: a hit pipe (H) beside the FSM

The FSM keeps everything it does today: stores (posted), misses and refills,
the shadow writeback, IF fills, MMIO-free DRAM traffic, maintenance, and the
debugger. A load **also** looks itself up on a dedicated path; if that lookup
hits, the FSM never sees the load.

**H1, at the EX→MEM edge.**
- The core drives `h_addr = exo_iaddr` (the AGU result, the same value
  `w_early_ld` puts on `m_addr`) straight to the cache. The read index of the
  tag and data BRAMs selects it whenever the FSM is not latching a request of
  its own that edge, and not in MAINT.
- The BRAM read is unconditional, so it has no side effect.
- The cache registers `h_rd_q` = "this edge's read was H's", plus the index
  and tag.

**H2, the load's first MEM cycle.**
- The cache compares both tags against `h_tag_q`, selects the way and the
  dword, and qualifies the result: `h_ok = h_rd_q && hit && !conflict`. The
  conflicts are in §4.
- `h_ok` and `h_dword` are registered **in the cache** at the end of H2 and
  exported.
- The core captures them into its MEM stage on that edge, whether or not MEM
  is frozen, so a later FSM read cannot overwrite them.

**MEM → WB.**
- The load leaves MEM after one cycle; it no longer waits for a port.
- In WB, `wb_ldval` extracts from the captured dword. The rf write and the
  dispatch forward work exactly as for 1a's fast load.
- A consumer in ID during the load's WB cycle dispatches at its end. So the
  load is in EX at t, the consumer at t+3.

**H miss or conflict (slow path).**
- The load waits in WB (`wb_wait`) and issues the ordinary request on the
  port from WB, one cycle later than 1a's MEM issue. The FSM serves it as
  today.
- `wb_wait` is computed from the registered `h_ok`, so the freeze stays
  register-driven.

H is limited to the loads `mem_fastld` covers today: LOAD, POP and LEAVE;
DRAM; aligned or within a dword. MEMGET32 spanning, RET/IRET/LEAVERET, MMIO
and the debugger keep the port path.

## 4. Consistency

Only the FSM writes the arrays: write hits at the end of CHECK, refill
installs, MAINT, and the LRU bit. An H read is stale only if a write to its
line has not landed yet. The ways it can be stale, and what covers each:

1. **The FSM is latching a request at H1's edge** (its own BRAM read). H's
   read is lost: `h_rd_q = 0`, slow path.
2. **CHECK with a write hit on the same index during H2.** BRAM read-first
   returns the pre-store line. Detected as `state == CHECK && r_is_write &&
   r_cache_hit && r_cache_index == h_idx_q`; slow path, ordered behind the
   store in the FSM.
3. **A posted store still on the port, not yet latched by the FSM.** Checked
   in the core at H1: `st_post && m_addr[31:5] == exo_iaddr[31:5]` (line
   granular) forces the slow path.
4. **A store miss in WRITE_FETCH, or a read refill in READ_WAIT, of H's
   line.** The line isn't installed, so the tags miss and it goes slow,
   behind the FSM. A hit on the other way is a different, valid line.
5. **MAINT.** H is disabled in MAINT, and `h_rd_q` is low for the whole walk.
6. **EVICT_SHADOW.** The arrays already hold the new line and the shadow only
   writes DDR, so H is safe.

DMA, the blitter and VGA access DDR around the cache, exactly as today; the
software flush rules are unchanged.

The **LRU** is not updated on H hits in the first version: its write port
belongs to the FSM. LRU only affects which way a miss evicts, never
correctness. If the miss rate moves, a deferred update in idle FSM cycles can
follow.

## 5. Timing risk

- **`exo_iaddr` → BRAM read address (new).** It runs from the core's AGU
  adder, across modules, to the RAMB36 address pins, through a 3:1 index mux
  (MAINT, FSM latch, H). `ex_b → m_addr` is the same adder and has ≥ 0.4 ns
  on `timing-headroom`. The fallback is to register `exo_iaddr` (H1 then
  starts in MEM, and the target distance becomes 4 but with 1 slot per load
  and 1 per cycle).
- **BRAM clock-to-out → 2 × 18-bit tag compare → way/dword mux → FF, inside
  the cache.** About 6 levels, comparable to CHECK's existing read-hit
  present.
- **`h_ok` / `h_dword` → core MEM capture.** A register-to-register cross.
- **`wb_wait` from a register.** No new comb logic in the freeze cone.

## 6. Verification

- **vsim:** directed tests, `diff` (40 seeds × latency 1/3/7, IRQ),
  `socdiff`. The socdiff programs need store→load-same-line, back-to-back
  loads and load-after-miss mixes. `randprog` gets a load/store-dense mode
  over a small footprint, so conflicts 2–4 fire.
- **Assertions:**
  - every H-served value equals a shadow read of the arrays one cycle later
    when no write is pending;
  - `h_ok` never fires with `st_post` on the same line.
- **Coverage counters (sim):** H hits, H misses, and slow-path loads by
  cause.
- **Board:** the usual suite, plus `perf_kern` load kernels, the benchmarks,
  csmith-board and zephyr (toolchain session).

## 7. Steps

1. **Cache side:** the H read index, `h_rd_q`, H2 compare/select, the
   conflict qualify, and `h_ok`/`h_dword` outputs. The core still ignores
   them. Assert H agrees with the FSM's result for the same address whenever
   both run.
2. **Core side:** `h_addr` out; the MEM capture; WB serving from the capture;
   the slow-path issue from WB; and `EN_DC_HPIPE`, so 0 = today's 1a.
3. Sim A/B on the benchmarks, then a timing build. Then the board.

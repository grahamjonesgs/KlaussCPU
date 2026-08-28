# AMP core 2 — 50 MHz KlaussCPU running lwIP, owning display + network

The coprocessor milestone: a second KlaussCPU `pipeline_core` instance at an
effective 50 MHz that owns LiteEth and the VNC/display path, so core 1 only
renders.  Kills the measured single-core spiral (doom tick 175→323 ms under
send-thread preemption; send = ~175 ms/frame of Zephyr TCP-stack CPU).

## 1. Why this shape (recovered 2026-08 analysis + board data)

- Send cost is **TCP-stack CPU proportional to byte count** — copies,
  checksum, and DMA were measured and rejected as levers (vnc/PERFORMANCE.md).
  A hardware ethernet ladder attacks the already-cheap parts (~10% ceiling).
- The decisive twist: Zephyr's stack costs ~2 ms/segment (~200k cycles);
  **baremetal lwIP does a segment in ~10–20k cycles**.  Core 2 at 50 MHz
  running lwIP sends a 125 KB frame in ~30–40 ms, fully parallel to render →
  the frame ceiling becomes **render alone (~85–90 ms ≈ 11 fps)** — lever 6's
  target without fabric TCP.
- A general-purpose second core running the *Zephyr* stack at 50 MHz computes
  to a LOSS (430 ms/frame).  The win is specifically **core 2 + lwIP**.
- 2026-08-24 board A/B (post zero-copy + hextile): doom convert=0 ms but fps
  still ~3, gated by tick 175→323 ms send preemption — software levers are
  exhausted; the remaining cost IS the stack sharing core 1.

## 2. Phase-0 feasibility (measured 2026-08-24, full-M12 image)

| Resource | Used | Free | Core-2 budget |
|----------|------|------|---------------|
| LUT      | 31,912 / 63,400 (50.3%) | ~31k | core ~12–15k (ALU 5.9k, div 1.3k, mul 0.6k + wrapper/I-cache/mailbox ~3k) — fits |
| BRAM     | 96 / 135 tiles (71%)    | 39 tiles ≈ 175 KB | local data store 24–26 tiles + I-cache 4 + FIFOs 1 ≈ **~30 tiles** — fits, THE constraint |
| DSP      | 16 / 240                | 224  | core-2 mul → DSP48s, free |

Software footprint (existing baremetal lwIP apps, llvm-size):
`lwip_demo.elf` = **172 KB text + 0.5 KB data + 190 KB bss**.  Conclusion:
**code cannot live in BRAM** → hybrid memory plan (§4).  bss is dominated by
generous lwipopts pools — target ≤ ~100 KB with a VNC-only tuning pass.

Assets already in the runtime repo: full lwIP tree + `lwip_port/`
(ethernetif.c, sys_arch.c, lwipopts.h) + baremetal LiteEth driver
(`src/eth.c`, `src/mdio.c`) + picolibc + `klausscpu.ld` to derive core2.ld
from.  The hextile encoder and LUT conversion in vnc_server.c are pure C —
reusable on core 2 as-is.

## 3. Clocking — CE/2 + multicycle, NOT a CDC domain

Core 2 runs on the 100 MHz clock with a wrapper-owned 2-cycle clock enable +
blanket `set_multicycle_path 2` over its hierarchy — the **M12 Stage C SHA
pattern** (ea605c2, board-proven), scaled up.  One clock domain: mailbox,
BRAM sharing, and the DDR arbiter port need no CDC.

**RULE (from the SHA war + M5 ready-edge lessons): every 100 MHz-side signal
core 2 samples must be LEVEL-HELD until acknowledged, never a 1-cycle pulse**
— a 100 MHz pulse can fall entirely inside core 2's off cycle.  Applies to:
DDR-port ready (hold until DV drop, the existing served-guard protocol
already behaves this way — audit, don't assume), mailbox doorbells (W1C
levels), IRQ (already level).  Core-2 outputs change only on CE edges — safe
for 100 MHz consumers by construction.

## 4. Memory architecture (hybrid — decided by the footprint numbers)

Core-2 address map (32-bit, same ISA view as core 1):

```
0x0000_0000 – 0x07FF_FFFF  DDR (shared).  Core-2 .text lives in a reserved
                           1 MB block at the top of DDR (core-1 linker/heap
                           sentry already stops short — verify + carve).
                           Framebuffer readable here (uncached).
0xE000_0000 – +BRAM_SIZE   Core-2 local BRAM: .rodata/.data/.bss/stack/heap/
                           lwIP pools (96–112 KB, single-cycle at 50 MHz).
0xF00x_xxxx                Core-2 MMIO decode (subset): mailbox, log FIFO,
                           LiteEth CSR+slot SRAM (via OWNER mux), timer tick.
```

- **I-path**: small read-only I-cache, direct-mapped, 8–16 KB, 32 B lines,
  fills as DDR arbiter master C (burst reuses the 32B dual-BL8 pipelined fill
  shape).  No writeback, no snoop; core 1 loads core-2 .text then core 2's
  reset release invalidates-all.  SMC on core 2 is out of contract.
- **D-path**: BRAM local store (fast stores — lwIP memcpys stay local) +
  UNCACHED DDR window for framebuffer reads (~125 KB linear read ≈ 8 ms at
  50 MHz uncached — inside the 30–40 ms send budget) + MMIO.
- **Coherency contract**: core-1 → core-2 handoff = core 1 flushes the fb
  region (MAINT FLUSH, dirty-writeback walk) then mailbox-posts the dirty
  rect.  Full-cache walk is v1-acceptable at 3–10 fps; region flush
  (REGION_INVALIDATE_PLAN.md) is the later optimisation — **that plan is
  written against pre-32B geometry (16 B lines, index addr[14:4]) and needs
  a geometry pass first** (icache-snoop lesson: audit every address concat).
  Core-2 .text block: flushed once after load, before reset release.

## 5. Bus + peripheral ownership

- **DDR arbiter**: mem_read_write grows master C (core-2 I-fill + fb window),
  blitter-pattern: registered grant, never during `is_miss_path`, per-
  transaction release, cache (core 1) keeps priority.  Port shapes: I-fill =
  32 B burst read; fb window = 128-bit single reads (blitter-style).
- **LiteEth OWNER mux** (preserves netboot): eth window (0xF006/0xF008)
  reachable from both cores through a mux, owner = mailbox-block register,
  **reset default core 1** so the boot-ROM netboot path is untouched; the
  doom app flips ownership to core 2 after loading its program.  No
  concurrent access by contract (never arbitrated).
- **Mailbox block** (new MMIO device id, e.g. 0xF010): doorbell IRQs both
  directions (level, W1C), 4–8 × 64-bit scratch registers (dirty-rect posts,
  stats returns), core-2 reset/start control (start_pc), OWNER register,
  **core-2 log FIFO** (core 2 writes bytes, core 1 drains to its console —
  the only sane debug path; no second UART needed).
- Core-2 wrapper around `pipeline_core` (interface verified clean: one
  memory port m_addr/DV/be/rdata/rdata_next/ready + IRQ + park + trace).
  LCD port tied off.  No loader/HCF/debug FSM in v1 — reset-vector start
  from the mailbox start_pc; crash visibility via log FIFO + a parked-state
  status register.

## 6. Software

- **Core 2 image**: baremetal, picolibc, lwIP (existing port) + LiteEth
  driver (existing) + thin RFB server ported from vnc_server.c to lwIP
  netconn (encoder/hextile/LUT code reused verbatim; "zero-copy" becomes
  read-from-fb-window-into-slot-SRAM) + mailbox protocol + DHCP (static
  192.168.68.59 fallback).  New `core2.ld` (BRAM data / DDR text).
  lwipopts tuning pass: bss 190 KB → ≤ 100 KB (PBUF_POOL, TCP windows sized
  to the 32 KB proven on Zephyr).
- **Core 1 (Zephyr doom)**: networking/eth OFF entirely (RAM + CPU back),
  display path = render → MAINT flush → mailbox post; loader utility loads
  core-2 image (from its own image or SD) into DDR + BRAM, flips OWNER,
  releases core-2 reset.
- **Loading**: core-2 BRAM is written by core 1 through a mailbox-block
  window (or core 2's .data init is copied by its own crt0 from DDR — crt0
  copy is simpler: only .text needs placing, everything else initialises
  from the DDR image.  Decide in Phase 1).

## 7. Phases + gates (each sim-gated, then board-gated, house style)

- **P0 ✓ (this doc)** — feasibility numbers, architecture decisions.
- **P1 skeleton — COMPLETE, BOARD-VERIFIED (2026-08-24)**: core2 wrapper
  (CE/2 + MCP constraint) + local BRAM + mailbox/log-FIFO/reset MMIO.
  pipeline_core grew a `ce` port (all 4 sequential blocks gated; core 1
  ties it high — m5a trace-identical on hello/test_64bit/bst/expr);
  core2_subsys.sv (64 KB 4K×128 BRAM block-inferred, UART-shape console →
  512B log FIFO, load/readback window, start sequencing); mailbox device
  0x010 + XDC blanket MCP on c2_core_i.  **Sim gate PASS** (tb_core2:
  window load + readback, 18 log bytes identical, clean HALT park).
  **Build MET: tier-1 −0.103 (both violations = pre-existing families,
  st.SM→r_mmio_read_data + ex_a→mem_result — NOTHING in core 2, MCP
  proven), tier-2 AggressiveExplore → WNS +0.019 / WHS +0.018.  Core-2
  cost: 11,760 LUTs (wrapper 354) + 16 BRAM + 2 DSPs.**  **BOARD GATE
  PASS: core 1 window-loaded hello_core2, core 2 ran at CE/2 and delivered
  "hello from core 2" through the log FIFO to the real UART; HALT park
  verified (CTRL==0x3); "P1 PASS" printed by the on-board checker.**
  Harness: perf/amp_p1/ (run_p1_core2.sh sim gate, gen_loader.py,
  board_gate.sh).  Lessons: MEMSET operands are VALUE, ADDRESS;
  SETR SIGN-EXTENDS imm32 (loader zero-extends explicitly); protocol =
  DV-held / level-held 1-core-cycle ready / accept-guard !ready;
  conservative next_valid=0 at odd dwords is safe.
- **P2a — COMPLETE, BOARD-VERIFIED (2026-08-24)**: arbiter master C
  (mem_read_write: cache > blitter > core 2, per-transaction release;
  3-way mig muxes; ready-gating extended to both grants) + core-2 uncached
  DDR window (full-rate adapter FSM presenting LEVELS to the CE side;
  dword-order swap per the blitter convention; adapter drains in-flight
  bursts on !r_run so the grant can never wedge).  Core-2 map: local BRAM
  shadows DDR's first 64 KB; DDR visible 0x0001_0000..0x07FF_FFFF.
  Sim: tb_core2 3 phases (local hello / execute-from-DDR / DDR data
  read×8 + masked write) ALL PASS on a behavioral master-C model.
  Build MET (tier-1 −0.079, all violations pre-existing families; tier-2
  → WNS +0.018/WHS +0.013).  **Board: coherency contract PROVEN both
  directions** (cached-write+FLUSH → core-2 execute-from-DDR AND data
  reads; core-2 DDR write → INVALIDATE → readback compare = P2 PASS).
  **Non-intrusion A/B (perf_haz vs a core-2 100%-duty uncached read
  spinner): compute kernels ≤+0.003%; mem_stream +6.4%, ptr_chase +8.6%
  = the bounded worst-case ceiling** (real fb traffic is a small duty
  cycle).  Programs: gen_p2test.py → core1_p2test/ddr_spin/
  core1_spinload; gate = board_gate_p2.sh.
- **P2b — COMPLETE, BOARD-VERIFIED (2026-08-24)**: read cache for the
  core-2 text window.  Direct-mapped 8 KB, 512 × 16 B lines (a miss = the
  ordinary single-burst uncached read + install — no new burst logic),
  window = 0x07E0_0000 + 1 MB only (immutable-after-flush contract; fb
  reads stay uncached), valid bits in FFs → one-cycle invalidate-all on
  every RUN 0→1, write-into-window clears its line as a backstop.  Hits
  serve in the same 2-core-cycle shape as local BRAM including the
  next-dword pair.  Sim: 4 phases PASS (incl. a 2000-iter cached-fetch
  loop).  Build MET (tier-1 −0.057 → tier-2 +0.011/+0.051).  **Board:
  P2a coherency regression PASS + fetch-rate probe: 1M-iteration loop =
  L=0x8C (140 ms) from local BRAM, D=0x8C (140 ms) from the cached DDR
  window — CYCLE-EXACT, the cache fully hides DDR for looping code.**
  m5e 7/7 UART-identical (arbiter changes harmless to core 1).
  **WATCH ITEM opened by this build: the ce retrofit re-exposed the M5d
  id_op→DSP-CEA1 cone on CORE 1's mul (tier-1 −0.057, tier-2 closed).
  Queued structural fix for the next respin: restore the mul block to
  enable-free (correct at CE/2 — operands change only on CE edges,
  results settle early and hold) + an XDC carve-out putting core-2's
  mul-pipe cells back on 1-cycle timing (the blanket MCP-2 would be
  unsound for a free-running pipe).**
- **P3 eth handover — COMPLETE, BOARD-VERIFIED (2026-08-27)**: OWNER mux +
  core-2 lwIP bring-up.  **RESULT: core 2 runs baremetal lwIP from the DDR
  text window (102 KB text, 36 KB bss in BRAM): PHY init over MDIO, link
  up, DHCP → 192.168.68.59, and answers ping from the LAN — 20/20, 0% loss,
  avg 18 ms (min 2.6 ms) — while core 1 only drains the log FIFO.**  Build
  p3b (ce-gated mul): tier-1 −0.121 on the ic_tagff→ic_v* fetch family,
  tier-2 → WNS +0.002 / WHS +0.019 (thin).  Netboot re-test: the boot ROM
  accepts + launches over TCP with the OWNER mux at reset default (NB a
  pre-existing, non-AMP netboot fault: net-booted hello.elf dies at
  PC=0x07FF0000 on the M12 QSPI image too; serial-loaded it runs).  P1
  regression PASS.  Gate lesson: RUN must be pulsed 0→1 (generators fixed).  DONE (RTL, sim-gated, build in flight): C2_ETH_OWNER (0x0038,
  reset 0 = core 1); top-level mux routes exactly one core to
  eth_mmio_bridge — the non-owner core 1 gets a 1-cycle self-ack (reads 0),
  the non-owner core 2's adapter self-completes; core-2 eth adapter
  (full-rate, level handshake to the CE side, holds the strobe until the
  bridge's ready pulse, drops it for S_COOL); clock_ms mirrored at
  0xF00F_0040 for lwIP sys_now; pipeline_core SP_RESET parameter (core 2 =
  top of local BRAM); **mul block back to enable-free + XDC carve-out
  (core-2 mul-pipe cells on 1-cycle timing) — the queued M5d-cone fix**;
  tb_core2 phase 4A/4B (non-owner reads 0 + clock mirror; owner
  write+readback) PASS; m5a hello/test_64bit still trace-identical.
  SOFTWARE (Mac tree, user builds): core2/{lwipopts.h (24 KB heap, 6
  pbufs, no TCP), core2.ld (text @0x07E0_0020 DDR window, data/bss/heap/
  16 KB stack in 64 KB BRAM), crt0_core2.c, main.c (eth_init + lwIP + DHCP
  → static .60 fallback + 5 s heartbeat)}, programs/core1_amp_host.c
  (memcpy image → FLUSH → OWNER=1 → start → forward log), Makefile targets
  `core2` / `core1_amp_host` (core2.bin → core2_image.h embed), mmio.h
  REG_C2_* defines.  Gate script board_gate_p3.sh: JTAG → NETBOOT RE-TEST
  (net-load hello.elf with OWNER at reset default) → P1/P2 regression →
  serial-load core1_amp_host → wait "core2: IP" → ping.
  **Timing note (P3 builds):** the enable-free mul block (the intended
  M5d-cone fix) was WORSE with core 2 in the fabric — tier-1 −0.266 on
  id_op→DSP A[15]: phys_opt's DSP register optimisation pulled ex_a into
  the DSP's 2nd A register, exposing the decode→operand-mux cone; tier-2
  only reached −0.104 and the retiming pass ran 2 h without converging
  (killed).  Reverted to the ce-gated mul (closed in P1/P2a/P2b; the XDC
  carve-out removed).  **Structural fix owed (its own design pass):**
  register the mul operands at DISPATCH (the M5 ALU-b pattern) or
  dont_touch on mul_a_q/mul_b_q so nothing upstream can be absorbed.
  Core-2 image (Mac build 2026-08-26): 102 KB text + 288 B data + 36.4 KB
  bss — fits the 64 KB BRAM with ~12 KB heap + 16 KB stack.
- **P4 VNC on core 2 — IN PROGRESS (2026-08-27)**.  Split: **P4a** = the
  core-2 RFB server (core2/vnc_c2.c: lwIP RAW API, chunked flow-controlled
  update pump, LUT/Raw/Hextile encoders ported verbatim, tiles pulled from
  the uncached fb with 64-bit loads into a local buffer) + shared-memory
  frame descriptor (amp/amp_proto.h @0x07D0_0000: producer writes
  seq+rect then one FLUSH; core 2 polls it — no doorbell IRQ in v1) +
  baremetal demo host (programs/core1_amp_vnc.c: 320x200 fb @0x0100_0000,
  bouncing box, FLUSH per frame, forwards core-2 console) — measured with
  vnc_probe.py hextile vs raw.  HW: core-2 BRAM 64→128 KB (32 tiles, 5
  spare), SP_RESET 0x20000, shadow = DDR<128 KB; lwipopts: TCP on, heap
  40 KB, SND_BUF 16×MSS.  **P4b** = Zephyr doom: networking OFF on core 1,
  display path = render → FLUSH → post; tick A/B (target ~175 ms, client
  fps ≈ render rate).  Gate script board_gate_p4.sh.
  **P4a RESULT (2026-08-27, board):** build MET +0.029/+0.021 (best of the
  series).  VNC SERVED BY CORE 2 first try: dirty-rect stream 41 fps
  hextile (1,056 B/upd) / 40.8 fps raw — producer-capped (50 fps posts),
  core-1 cost per frame = a <1 ms cache flush.  FULL-FRAME service rate:
  raw 3.8 fps = 474 KB/s, hextile 4.8 fps (test pattern 2.1×) — SAME wire
  throughput core 1's Zephyr stack got on 08-24 over this path (~420 KB/s):
  the ceiling is now TCP window (23 KB) × path RTT (VM→Parallels NAT→Mac
  WiFi→router→board, ping avg 18 ms), not either core.  Next lever = the
  window: SND_BUF 16→32×MSS (46 KB), MEM_SIZE 64 KB (BRAM headroom exists)
  → Mac rebuild only.  Then P4b (doom).  Open: host printed ack=0 — likely
  the multi-arg printf garble; split into ≤2-arg lines.
  **P4b RESULT — COMPLETE, BOARD-VERIFIED (2026-08-28), clean run:**
  Zephyr doom with networking OFF on core 1, VNC served by core 2 (net
  via the OWNER mux, frames via the shared descriptor + one FLUSH/frame):
  **doom tick 161-184 ms WHILE a client streams (vs 323 ms single-core on
  08-24) — the renderer never pays for the network again.**  Client:
  hextile 3.70 fps / raw 3.83 fps @128 KB/update, ~470 KB/s (vs ~3.0 on
  08-24); core 2 boot=1 throughout (no restarts), vnc=2/3 healthy.  Chain
  is now render 5.7/s > core-2 full-frame service 3.8/s, bounded by TCP
  window × path RTT on the VM→NAT→Mac-WiFi route (~480 KB/s), not by
  either core.  Bugs fixed on the way (all lessons in memory): descriptor
  false sharing (consumer fields on their own cache line), misaligned
  u64 buffers (__aligned(8)), pump starving the 2-slot RX (one chunk per
  call), stale-client bricked accepts (evict), console drain starved by
  doom (prio 11), stale log-FIFO replay (pre-drain), handover from a
  running core 2 (clean-slate reprogram in gates), core-2 stack 24 KB.
  Known gap: no keyboard path in the AMP build (RFB key events would need
  forwarding through the descriptor).  NEXT (P5): bytes/RTT — larger
  window / wired client test / cheaper encodings for doom content; then
  the region-flush + cacheable-fb hardware levers if encode ever limits.
- **P5 hextile + tuning**: encoder port, lwipopts/TCP window tuning,
  region-flush RTL if the MAINT walk shows up in the profile.  Target:
  the ~11 fps ceiling (render-limited).

## 8. Risks / watch items

- **BRAM budget** (~30 of 39 free tiles): if lwipopts can't get bss ≤ 100 KB,
  fall back to 112 KB store + shallower I-cache, or spill cold bss to a
  cached-in-I... no — spill cold data to the fb window region (uncached DDR)
  via section attributes.  Escape hatch exists; measure in P1.
- **CE/2 handshake rule** (§3) — audit every core-2-facing ready/valid.
  Board CPI A/B on core 1 after every phase (the M11a process rule).
- **Vivado hierarchy**: keep core 2 under one instance for the blanket MCP
  constraint + KEEP_HIERARCHY so the 3028-LUT mis-attribution class of
  confusion doesn't recur.
- **Netboot regression** — P3 gate explicitly re-tests power-on netboot.
- **DDR bandwidth**: core-2 I-misses + fb reads + core-1 misses + blitter
  share one MIG; core 2 is latency-tolerant by design — keep it lowest
  priority; watch core-1 avgpen counters at P2 gate.

- **Mul operand family (RESOLVED-FOR-NOW, 2026-08-26)**: the ce-gated mul
  block is the shape that closes (P1/P2a met; P2b −0.057 → tier-2 +0.011).
  The enable-free variant (tried in P3) was WORSE: tier-1 −0.266 on
  id_op→mul_pipe/A — phys_opt's DSP register optimization absorbs ex_a into
  the DSP's second A register, exposing the decode→operand-mux cone as a
  full-cycle path into the DSP (M5d's family); tier-2 got only to −0.104
  and the retiming pass ran 2 h without converging (killed).  Reverted to
  ce-gated; the XDC carve-out was dropped with it (a gated pipe is validly
  under the blanket MCP).  Proper fix, its own design pass: register the mul
  operands at DISPATCH (the M5 ALU b-operand pattern) or dont_touch on
  mul_a_q/mul_b_q to forbid the second absorption.

## 9. Open (deferred, decide when reached)

- Region flush geometry pass (P5, only if profiled).
- Core-2 image source: embedded blob in doom app vs SD file (P4).
- BRAM load path: mailbox window vs crt0 self-copy (P1).
- hit-under-shadow/BTB and other core-1 items proceed independently.

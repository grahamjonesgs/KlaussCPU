# VGA output plan

**STATUS: Phases 0 and 1 DONE (board-verified). Phase 2 (DDR scanout) next.**

Drive the Nexys A7's on-board VGA connector (12-bit colour, a 4:4:4 resistor
DAC plus HS/VS) from the SoC, displaying a framebuffer that lives in **DDR**.
The monitor has no VGA input, so the board drives a Gizzu active VGA→HDMI
converter.

## Decision: scan out from DDR (option C)

Two designs were compared:

- **B: BRAM framebuffer.** Rejected. 320×200 or 320×240 at 8 bpp only
  (16–19 of the ~33 free RAMB36 tiles), one buffer, the blitter can't reach it,
  core 2/VNC can't see it, and it uses BRAM needed for future growth.
- **C: DDR scanout.** Chosen. A display DMA engine becomes a 4th DDR master and
  streams lines into a small line buffer. It reuses everything that already
  draws RGB565 into DDR: the blitter, the doom/Zephyr framebuffer at
  `0x0100_0000`, and the cache flush. VGA and VNC then show **the same
  buffer**. Page flipping is a register write. Costs: a change to the
  `mem_read_write.sv` arbiter, and a real underflow risk that has to be
  designed out.

## Fixed parameters

| Item | Value | Why |
|------|-------|-----|
| Video mode | 640×480 @ 60 Hz, HS and VS **negative** | Universal; every converter recognises it |
| H timing (pixels) | 640 visible, 16 front porch, 96 sync, 48 back porch = 800 | VESA DMT |
| V timing (lines) | 480 visible, 10 front porch, 2 sync, 33 back porch = 525 | VESA DMT |
| Pixel clock | 25.000 MHz = ui_clk (100 MHz) **÷4 clock enable** | Same clock domain as the CPU: no MMCM output, no CDC, no new clocks. 25.0 vs 25.175 MHz (59.5 Hz frame) is within tolerance. |
| Pins | `VGA_R/G/B[3:0]`, `VGA_HS`, `VGA_VS`, bank 35/15, LVCMOS33 | Nexys A7 master XDC |
| MMIO block | `0xF011_xxxx` | Next free device ID |
| IRQ | Source 3 = vsync (start of vblank) | Sources 0–2 are timer, blitter and Ethernet |

The board has no DDC/EDID lines, so the converter auto-detects the mode from
sync timing alone. The porches and polarity must be exact.

## Architecture (end state)

```
                      MMIO 0xF011 (ctrl, fb base, stride, status, palette)
                                     │
  ┌──────────────────────────────────▼───────────────────────────────────┐
  │ vga_ctrl.sv                                                          │
  │  vga_timing ──x,y,active,hs,vs──► pixel pipe ──► output FFs ──► pins │
  │      │ line_start / frame_start      ▲                               │
  │      ▼                               │ RGB565→444 or 8-bit palette    │
  │  vga_scanout (DMA master D) ──► ping-pong line buffer (1× RAMB36)    │
  └──────┬───────────────────────────────────────────────────────────────┘
         │ req / grant / read_DV / ready / done (same contract as blitter)
  ┌──────▼──────────────────────────┐
  │ mem_read_write.sv DDR arbiter   │  priority: cache > VGA > blitter > core 2
  └─────────────────────────────────┘
```

## Bandwidth budget

A display line lasts 800 × 4 = **3,177 cycles** at 100 MHz (31.8 µs). An
isolated narrow 128-bit DDR read costs about 25–30 cycles. That figure is
inferred from the latency-bound blitter COPY at 6.17 cyc/px and still has to
be measured for scanout.

| Mode | Reads per display line | DDR occupancy, serial reads |
|------|------------------------|-----------------------------|
| 320×240 RGB565, doubled to 640×480 | 40 narrow reads per 2 lines | ~19% |
| 320×240 8 bpp + palette, doubled | 20 narrow reads per 2 lines | ~10% |
| 640×480 RGB565 native | 80 narrow reads per line | ~75%: **not acceptable** |
| 640×480 RGB565 using wide 32 B reads (cache's dual-BL8 path) | 40 per line | ~35–40% |

The first milestone is **320×240, pixel- and line-doubled**. Native 640×480
RGB565 waits until scanout uses the wide/pipelined read path (Phase 4).

## Phases

### Phase 0: RTL test pattern (no CPU involvement)
- Uncomment the VGA pins in `nexys_ddr.xdc`; add `set_false_path` to them like
  the other board outputs.
- `vga_timing.sv`: 800×525 counters on the ÷4 CE; outputs x, y, active, HS,
  VS, plus line_start/frame_start strobes for later phases.
- `vga_ctrl.sv`: timing + test-pattern pixel source + registered outputs (all
  signals take one identical register stage, so they stay aligned).
- Pattern: 8 colour bars (top 3/4), a 16-step grey ramp per channel (bottom
  1/4), a 1-pixel white border on all four edges, and a small square that
  moves once per frame (proves the frame rate and that the image isn't frozen).
- Instantiate in `KlaussCPU.sv` on `i_Clk` (ui_clk) / `w_reset_H`.
- `tb_vga.sv` + `perf/m5a/run_vga.sh`: checks 800/525 totals, sync widths,
  porches, polarity, the active-region count, and that RGB is 0 in blanking.
- **Gate (board):** the Gizzu shows a stable 640×480 image with no "out of
  range" message; all four border edges are visible (proves porch placement);
  the ramp shows 16 distinct steps per channel (proves all 12 DAC bits and pin
  order). Timing still meets at 100 MHz.

### Phase 1: MMIO block + palette
- `0xF011_0000` register file in `vga_ctrl.sv`, decoded in the `KlaussCPU.sv`
  MMIO read hub (the device owns its writes through `mmio_if`). As built:

| Offset | Reg | RW | Description |
|--------|-----|----|-------------|
| 0x000 | `VGA_CTRL` | RW | `[0]` SCANOUT_EN (Phase 2), `[1]` DOUBLE (Phase 2), `[2]` BPP8 (Phase 2), `[3]` TEST_PATTERN (**reset 1**), `[4]` VSYNC_IRQ_EN, `[5]` PALETTE_VIEW. Source priority: test pattern > palette view > (scanout) > border colour. |
| 0x008 | `VGA_FB_BASE` | RW | Framebuffer byte address `[31:4]`. **Copied to FB_ACTIVE at the start of vblank**, so a write is a tear-free page flip. Reads return the pending value. |
| 0x010 | `VGA_STRIDE` | RW | Bytes per source line `[15:4]` (reset 640) |
| 0x018 | `VGA_STATUS` | R / W | R: `[0]` IN_VBLANK, `[1]` VSYNC_PENDING, `[31:16]` frame count, `[41:32]` current raster line, `[63:48]` underflow count. W: `[1]`=1 clears VSYNC_PENDING, `[2]`=1 clears the underflow count. |
| 0x020 | `VGA_BORDER` | RW | `[11:0]` RGB444 outside the source image (letterboxing) |
| 0x028 | `VGA_VSTART` | RW | `[9:0]` first display line of the image (Phase 2; 40 centres 640×400) |
| 0x030 | `VGA_FB_ACTIVE` | R | The FB_BASE latched at the last vblank (lets software see a flip land) |
| 0x800–0xFF8 | `VGA_PALETTE` | RW | 256 entries × 8 B, `[11:0]` RGB444 (LUTRAM; not reset) |

- PALETTE_VIEW draws a 16×16 grid of 40×30 cells, cell (row, col) =
  `palette[row*16 + col]`. It tests the palette on the board before scanout
  exists, using the same lookup as Phase 2's 8-bit mode.
- The pixel path is two stages on the pixel CE (source select → palette
  lookup → IOB FFs), with HS/VS delayed by the same amount.
- Vsync IRQ → `w_irq_src3`, `INT_PENDING[3]`; `w_irq_sel` priority timer >
  blitter > eth > vga.
- **Gate (sim):** `run_vga.sh` covers 7 frames: pattern → border → palette
  view; register reset values and readback, all 256 palette entries, the IRQ
  (line 480, a 16.8 ms period, W1C, IRQ_EN gating) and the FB latch. Four
  deliberate faults (palette index swap, an immediate FB latch, an ungated
  IRQ, an HS glitch) are all caught.
- **Gate (board):** `Klausscpu-runtime/baremetal/test_vga.elf` checks T1–T5
  (readback, palette, a 59–60 frame/s count, 118–120 IRQs in 2 s, the FB
  latch), then shows border colours, the palette grid and 5 s of palette
  cycling driven by vsync.

### Phase 2: scanout DMA master
- `vga_scanout.sv`: on each source line, issue narrow 128-bit reads from
  `fb_base + line × stride` into the idle half of a ping-pong line buffer
  (one RAMB36: 2 × 640 B for doubled RGB565, 2 × 1280 B for native).
- Fetch starts at the previous display line's `line_start`. Doubled mode
  fetches once and displays twice.
- Underflow: if the pixel side reaches a half that isn't full, output the
  border colour for the rest of the line and bump the counter. It never
  stalls and never shows garbage.
- **Arbiter (`mem_read_write.sv`):** add master D using the existing
  req/grant/done contract, priority **cache > VGA > blitter > core 2**. VGA
  holds the grant for a whole line fetch (≤ 40 reads). It can wait at most
  one blitter chunk (8 txns) or one core-2 txn for the bus. The
  orphaned-grant guard covers D as well.
- **Gate (sim):** `tb_vga` with a model DDR shows a correct image CRC;
  `tb_cache` passes with a VGA master saturating; `tb_blitter`, `tb_core2`,
  `tb_soc` and `run_m5a`/`run_m5c` are unchanged; and there are **0
  underflows** while a full-screen blitter COPY and core-2 VNC traffic run.
- **Gate (board):** timing met at 100 MHz; doom renders at 320×200 on VGA;
  the underflow counter stays 0 over a 10-minute doom + VNC session.

### Phase 3: software
- klausscc runtime `vga.h`: init, set mode, `vga_flip(base)` (write
  FB_BASE, wait for vsync), palette load.
- Coherency: the CPU draws through the write-back cache, so **flush the
  framebuffer region before flipping** (the existing MAINT flush — same
  contract as the VNC handoff in AMP_CORE2_PLAN.md). Blitter output needs no
  flush (it writes DDR directly).
- Zephyr display driver (RGB565, 320×240) with double buffering by flipping.
- doom: present via VGA, optionally alongside VNC (same buffer).
- Document `0xF011` in MMIO_MAP.md and the IRQ in CPU_ARCHITECTURE.md.

### Phase 4 (optional)
- Native 640×480 RGB565 via the wide 32 B read path (measure occupancy first).
- Text-console overlay (80×30, 8×16 font ROM, ~3 BRAM) mirroring UART and crash
  dumps.
- Hardware cursor sprite.

## Risks

| Risk | Mitigation |
|------|------------|
| The converter rejects the mode | Phase 0 settles this before any DDR work; exact VESA timing and polarity |
| Line-buffer underflow under DDR load | VGA is second only to the cache; bounded blitter/core-2 tenure; whole-line prefetch with ~80% slack at 320×240; underflow counter + border fill |
| Timing closure (last P&R +0.070 ns, 73% LUTs) | VGA logic is small and on a 4-cycle CE; registered outputs; pblock or KEEP_HIERARCHY if it lands in the core's area |
| CPU slowdown from scanout traffic | ~10–19% DDR occupancy at 320×240; measure with the perf counters (cache-miss latency) before/after |
| Stale pixels (cache not flushed) | Driver API flushes before flip; documented contract |

## Log

- 2026-10-09: Plan written; option C chosen. Phase 0 RTL written:
  `vga_timing.sv`, `vga_ctrl.sv` (test pattern), top-level `VGA_*` ports,
  XDC pins + false paths, xpr entries, `tb_soc`/`run_m5d_soc.sh` updated.
  `perf/m5a/run_vga.sh` → **TB_VGA PASS** (2 full frames: HS 800/96,
  VS 525 lines/2, active window px 144..783, 480 lines after 35 blank,
  bars + ramps). Mutation check: H_BP 47, V_SYNC 3 and a wrong ramp slice all
  FAIL as expected. Full SoC (`tb_soc`) compiles and elaborates.
  Bitstream: `build_fast.tcl -resynth` **MET at tier 1, WNS +0.090 /
  WHS +0.020** (27 min); 44,007 LUTs (69.4%), BRAM unchanged at 101.5.
  Programmed via `tools/program_fpga.tcl` (new: JTAG from the Windows host);
  `hello.elf` serial-loads and runs correctly on it (SoC unaffected).
  **Board gate PASSED** through the Gizzu: locks at 640×480, all four border
  edges visible, bar order correct, 16 even steps in every ramp, square
  moving smoothly.
- 2026-10-09: Phase 1 RTL: register file, palette (LUTRAM, 2-port), palette
  view, vsync IRQ on source 3; tb_vga extended (PASS, 7 frames) and
  mutation-checked; SoC elaborates. `test_vga.c` + `mmio.h` VGA block added in
  Klausscpu-runtime. Bitstream build started.
  Bitstream (full Performance_Explore, `-resynth`): **MET tier 1, WNS +0.192 /
  WHS +0.015** (26 min); 44,223 LUTs (69.8%). `test_vga.elf` on the board:
  **6/6 PASS**: readback, palette, 60 frames/s, raster line moves, 0 IRQs while
  disabled, 120 IRQs in 2 s, pending clear, FB_BASE latch.
  Visual check confirmed by the user: border colours, palette grid, smooth vsync palette cycling.

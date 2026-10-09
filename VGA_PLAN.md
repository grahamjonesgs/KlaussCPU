# VGA output plan

**STATUS: Phase 0 DONE: test pattern verified on the board through the Gizzu
VGA→HDMI converter. Phase 1 (MMIO + palette + vsync IRQ) next.**

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
  MMIO read/write hubs.

| Offset | Reg | RW | Description |
|--------|-----|----|-------------|
| 0x00 | `VGA_CTRL` | RW | `[0]` enable scanout (0 = test pattern / border colour), `[1]` double (320×240 → 640×480), `[2]` bpp (0 = RGB565, 1 = 8-bit indexed), `[3]` test pattern, `[4]` vsync IRQ enable |
| 0x08 | `VGA_FB_BASE` | RW | Framebuffer byte address (16 B aligned). **Latched at the start of vblank**, so a write is a tear-free page flip. Readback = the pending value. |
| 0x10 | `VGA_STRIDE` | RW | Bytes per source line (16 B aligned) |
| 0x18 | `VGA_STATUS` | R/W1C | `[0]` in vblank, `[1]` vsync IRQ pending (W1C), `[31:16]` frame counter; `[63:48]` underflow count (W1C clears it) |
| 0x20 | `VGA_BORDER` | RW | 12-bit colour outside the source image (letterboxing) |
| 0x28 | `VGA_VSTART` | RW | First display line of the image (centres 320×200 as 640×400: 40) |
| 0x400–0x7FF | `VGA_PALETTE` | W | 256 × 12-bit entries (LUTRAM), 8 B per entry |

- Vsync IRQ → `w_irq_src3` (extend `w_irq_ready` / `w_irq_sel` to 2 bits).
- **Gate:** the test pattern and border colour can be switched from software;
  the IRQ fires at 59.5 Hz (count over 10 s).

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

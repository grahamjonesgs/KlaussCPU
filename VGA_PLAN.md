# VGA output plan

**STATUS: Phases 0–3 DONE (Phase 3: Zephyr/doom on VGA verified by serial; visual check of the Zephyr apps pending). Phase 4 optional.**

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

### Phase 2: scanout DMA master (as built)
- **`vga_scanout.sv`** (inside `vga_ctrl`) is DDR master D. It uses **wide
  reads**: one transaction is one 32 B line, the cache's two pipelined BL8
  bursts, which halves the transaction count compared with narrow 16 B
  reads. Lines go into a ping-pong line buffer (one RAMB36, 512×64, 2 KB per
  half; source line *s* lives in half *s*[0]).
- **Fetch schedule:** source line 0 is prefetched at display line
  (VSTART−1) mod 525. At the first display line of source line *s*, *s*+1 is
  fetched into the other half, so each fetch has one source line of time
  (1 display line native, 2 doubled) and never writes the half on screen.
  Reads per line: 10 (320×8 bpp), 20 (320×16 / 640×8), 40 (640×16). Tenures
  are CHUNK = 4 reads. A fetch that is still running when the next one is
  due is abandoned after its in-flight read, and the new line starts.
- **Frame latching:** SCANOUT_EN, DOUBLE, BPP8, STRIDE, HEIGHT and VSTART
  latch at the start of vblank (line 480), the same moment FB_BASE becomes
  FB_ACTIVE. Anything written during a frame applies from the next one.
  TEST_PATTERN and PALETTE_VIEW (debug views), BORDER, the palette and
  VSYNC_IRQ_EN still act immediately. Sim found and fixed three bugs here:
  a mid-frame VSTART write could re-fire the line-0 prefetch; a frame whose
  VSTART moved down replayed the previous frame above its image; and with
  latching at line VSTART−1, settings written in the active picture landed
  a frame early.
- **Underflow:** a display line whose half is not yet VALID shows the border
  colour (or, if the fetch lands mid-line, border then correct pixels, never
  garbage), and `VGA_STATUS[63:48]` counts it (saturating, W1C via bit 2).
- **Pixel path:** stage 1 registers the line-buffer dword (the BRAM read
  issued the cycle after x changes). Stage 2 extracts the byte or halfword
  (little-endian), then either looks it up in the palette or expands
  RGB565→444 (top 4 bits per channel).
- **Arbiter (`mem_read_write.sv`):** new master D ports. The grant priority
  among DMA masters is **VGA > blitter > core 2**; the cache stays the
  default owner. VGA's gate is looser than the others: read-only, so a
  pending or running maintenance walk does not block it, only a miss in
  flight or a walk that is mid-writeback. It may be granted at the walk's
  between-sets point (MS_READ), and the walk then parks in MS_Wx_WAIT.
  Without this, a dirty full-cache flush (up to ~1.2 ms) would underflow
  ~38 lines. `mig_wide` is 1 for VGA, `mig_dw_off` is 0, and the cache's
  ready/CWF pulses are gated off while VGA holds the bus. The orphan guard
  covers D.
- **New register:** `VGA_HEIGHT` (0x038, source lines, reset 240). FB_BASE
  and STRIDE are now 32 B aligned.
- **Gate (sim), all PASS:**
  - `tb_vga` runs 12 frames. Scanout frames are compared pixel by pixel
    against a hash-of-address DDR model: 320×240 RGB565 doubled; 640×400
    BPP8 letterboxed with stride 1024; 320×200 doubled with grants up to 2 µs
    late; and a starved 640×480×16 frame (every line underflows, border or
    correct only, counter counts and clears). Every read is checked to be
    32 B aligned, granted, DV-dropped, and inside the frame.
  - `tb_cache` (real `ddr2_control` + fake MIG) adds V1, the wide-read
    layout, and V2: VGA streaming against CPU thrash, the blitter, and a
    dirty flush walk, with 1153 VGA reads and 112 grants inside the walk.
    The full DDR image compare still matches.
  - `tb_blitter` is unchanged; the full SoC elaborates.
  - Fault injection: arbiter (cache CWF gating, walk gate, miss gate) and
    scanout (RGB565 byte order, line-read count, wrong half, extra fetch past
    the image, underflow counter) faults are all caught. Two survivors are
    benign: `mig_dw_off`, because `ddr2_control` assembles both burst orders
    identically; and VALID set 3 cycles early.
- **Gate (board):** `Klausscpu-runtime/baremetal/test_vga_scan.elf` covers
  S1 320×240 doubled, S2 page flip + flush every frame, S3 under
  blitter/memcpy/flush stress, S4 letterboxed 320×200, S5 640×480 8 bpp,
  S6 640×480 RGB565 idle and under memcpy, and M, the memcpy throughput cost.
- `tb_soc` on Windows: `xsim.bat` splits the `-testplusarg K=V` arguments at
  `=`, so `run_m5d_soc.sh` only runs on the Linux VM.

### Phase 3: software (as built, Klausscpu-runtime)
- **`vga.h`** (runtime root, header-only, baremetal + Zephyr):
  `vga_set_mode()` / `vga_set_mode_centred()`, `vga_present()` (flip),
  `vga_wait_vsync()`, `vga_cache_flush()`, `vga_set_palette_rgb888()`,
  `vga_set_border()`, `vga_off()`, `vga_underflows()`.
- **Zephyr `CONFIG_KLAUSSCPU_VGA`** (`vnc/vga_out.c`): from boot, VGA scans
  the shared RGB565 `fb` (320 wide is doubled; centred vertically). `fb` is
  now 32 B aligned. `fb_mark_dirty()` kicks a low-priority thread that
  flushes the cache once per 16 ms while `fb` is dirty. This works unchanged
  for gui_demo, mandel, LVGL (`display_vnc` writes into `fb`) and Doom over
  VNC.
- **`vga_out_show_indexed()`**: apps that render 8-bit palette frames hand
  them over with their palette. The frame is copied into one of two
  32 B-aligned heap buffers (`k_aligned_alloc`), flushed, and flipped at
  vblank, so there is no tearing and no RGB565 conversion.
  - Doom uses it (and skips the RGB565 conversion when no VNC consumer
    exists); `vga.conf` builds VGA-only Doom with no networking.
  - Mandel uses it for its rotating-palette posts. Without it, VGA would
    show nothing in AMP builds, which only fill RGB565 for true-colour VNC
    clients.
- VGA is on by default in the prj.conf of doom, gui_demo, gui_lvgl and
  mandel. All seven variants build: doom VNC / AMP / VGA-only, gui_demo,
  gui_lvgl, and mandel VNC / AMP.
- **Board:**
  - VGA-only Doom: ~52 fps on the title screen, 16–25 fps in the
    attract-mode demos; the VGA copy and flush cost ~2 ms per frame; **0
    underflows** over ~90 s.
  - mandel AMP: VGA costs ~6% of compute (1595/1644/1826 vs
    1695/1756/1955 kiter/s at the same depths).
  - gui_demo (640×480 native) boots with VGA.
  - Visual confirmation of the Zephyr apps is pending.
- **Docs:** `0xF011` in MMIO_MAP.md; interrupt sources 1–3 in
  CPU_ARCHITECTURE.md; a VGA section in the Doom README.
- **Not done:** keyboard input for VGA-only Doom (input still arrives only
  via VNC; a UART or PS/2 keyboard path would be new work).

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
- 2026-10-09: Phase 2 RTL (vga_scanout, master D, frame latching) +
  tb_vga / tb_cache / tb_blitter PASS (see Phase 2 section). First quick
  build silently reused a stale synth netlist (NEEDS_REFRESH misses
  out-of-GUI edits) — build_fast.tcl now re-synthesises when any source is
  newer than the synth checkpoint. Quick build: WNS -0.274 (all failing
  paths in core 2 / boot FSM, none in VGA), 44,504 LUTs, BRAM 102.5 (+1 for
  the line buffer). Board: `test_vga_scan.elf` **7/7 PASS, 0 underflows in
  every mode**, incl. 640x480 RGB565 under memcpy and 180 page flips with a
  full-cache flush per frame. memcpy 7 -> 6 -> 5 MB/s (off / 320x240x16 /
  640x480x16). Photo of S1 confirmed by the user: bar order, gradient axes,
  8x8 checkerboard, white frame; 16-level banding is the 12-bit DAC; the
  monitor stretches 4:3 to 16:9.
- 2026-10-09: Phase 2 full build (Performance_Explore): **MET tier 1, WNS
  +0.078 / WHS +0.022**, 44,482 LUTs (70.2%), BRAM 102.5. On that bitstream:
  test_vga_scan 7/7, test_vga 6/6, blit_selftest 19 PASS / 0 FAIL.
- 2026-10-09: Phase 3 (software) — see the Phase 3 section; runtime commit on vga-output.
- 2026-10-09: Doom zero-copy VGA path: vga_out pool is now 3 buffers;
  vga_out_indexed_buffer()/vga_out_present_indexed() let Doom render straight
  into VGA memory (DG_ScreenBuffer re-pointed each frame; AMP builds keep the
  copy). Gameplay fps over the same 27 demo windows: VGA-only 21.3 -> **24.7**
  (repeatable; was 23.7 for AMP without VGA), hand-off 2 ms -> <1 ms,
  0 underflows. AMP + VGA unchanged at 20.8. Next lever is Doom's render
  itself (~39 ms R_RenderPlayerView).

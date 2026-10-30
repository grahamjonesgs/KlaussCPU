//===========================================================================
// tb_vga — VGA_PLAN.md Phase 0/1/2 gate: vga_ctrl's 640×480 @ 60 Hz pin
// timing, pixel sources, registers, palette, vsync IRQ and DDR scanout.
//
// Watches ONLY the five pin-level outputs (as the converter would), one sample
// per pixel, and checks against VESA DMT 640×480:
//   * HS: period 800 px, low for 96 px (negative polarity)
//   * VS: period 525 lines (420 000 px), low for 2 lines (1600 px)
//   * active window: on every active line the first non-black pixel is 144 px
//     after the HS falling edge (sync 96 + back porch 48) and the last is 783
//     (the white border guarantees both ends are lit) — i.e. the 640-px image
//     sits exactly between the porches; nothing is lit in blanking
//   * 480 active lines per frame, preceded by exactly 35 blank lines from the
//     VS falling edge (sync 2 + back porch 33): the first active line is the
//     one that ends at HS fall #34, counting the fall inside the VS line as #0
//   * content, per frame, by the mode the test sequence selected:
//       frames 1-2  reset TEST_PATTERN: full-white top line, bars, ramps
//       frame  3    border mode (CTRL=0, BORDER=0x123): every pixel 0x123
//       frames 4-7  PALETTE_VIEW: every pixel = palette[(y/30)*16 + x/40]
//       frames 8-11 SCANOUT from a DDR model (hash-of-address memory), every
//                   pixel compared: 320x240 RGB565 doubled; 640x400 BPP8
//                   letterboxed, stride 1024; 320x200 doubled under heavy
//                   grant contention; 640x480 RGB565 with a DDR too slow to
//                   keep up (underflow: each pixel must be border or correct,
//                   never garbage, and UNDERFLOW must count)
//       frame  12   SCANOUT_EN off again: border only
//   * MMIO: reset values, STATUS fields, palette readback, vsync IRQ (fires at
//     line 480, once per 420 000 px, W1C, gated by IRQ_EN), FB_BASE → FB_ACTIVE
//     latched only at vblank
//
// Run: perf/m5a/run_vga.sh   (prints "TB_VGA PASS" on success)
//===========================================================================
`timescale 1ns / 1ps
module tb_vga;
   logic clk = 0, rst_l = 0;
   always #5 clk = ~clk;

   logic [3:0] r, g, b;
   logic       hs, vs;

   logic       irq;

   mmio_if bus();
   assign bus.byte_en = 8'hFF;
   initial begin
      bus.write_DV = 0; bus.read_DV = 0; bus.addr = 0; bus.write_data = 0;
   end

   // DDR master D
   logic         dma_req, dma_done, dma_rd;
   logic [31:0]  dma_addr;
   logic [255:0] dma_rdata = '0;
   logic         dma_ready = 0, dma_grant = 0;

   vga_ctrl dut (
      .i_Clk(clk), .i_Rst_L(rst_l), .mmio(bus.slave), .o_irq(irq),
      .o_dma_req(dma_req), .o_dma_done(dma_done), .o_dma_read_DV(dma_rd),
      .o_dma_addr(dma_addr), .i_dma_read_data(dma_rdata), .i_dma_ready(dma_ready),
      .i_dma_grant(dma_grant),
      .o_vga_r(r), .o_vga_g(g), .o_vga_b(b), .o_vga_hs(hs), .o_vga_vs(vs)
   );

   int          mode = 0;         // 0 test pattern, 1 border, 2 palette view, 3 scanout
   // scanout frame config (applied by the pixel monitor at the VS fall)
   typedef struct { logic dbl, bpp8; int base, stride, height, vstart; logic allow_uf; } scfg_t;
   scfg_t       cfg, cfg_next;
   logic        cfg_pending = 0;
   int          uf_lines = 0;     // lines that showed border inside the image

   // ---------------- DDR model (mem_read_write master-D contract) ----------
   // Memory = a hash of the byte address.  Grants come after a random delay
   // (0..ddr_gdelay cycles: contention) and drop the cycle after done; each
   // read returns its 32 B line in the cache-line layout after ddr_lat cycles.
   function automatic logic [7:0] mem_byte(input logic [31:0] a);
      return 8'((a * 7) ^ ((a >> 7) * 3) ^ ((a >> 13) * 5) ^ (a >> 21));
   endfunction
   function automatic logic [255:0] mem_line(input logic [31:0] a);
      logic [255:0] l;
      for (int k = 0; k < 4; k++)
         for (int j = 0; j < 8; j++)
            l[255 - 64*k - 56 + 8*j -: 8] = mem_byte(a + 32'(8*k + j));
      return l;
   endfunction

   int ddr_lat = 20, ddr_gdelay = 0, ddr_reads = 0;
   int dma_errors_shown = 0;
   initial forever begin
      @(posedge clk);
      if (dma_req && !dma_grant) begin
         repeat ($urandom_range(0, ddr_gdelay)) @(posedge clk);
         dma_grant <= 1'b1;
         // serve reads until done
         forever begin
            @(posedge clk);
            if (dma_done) begin dma_grant <= 1'b0; break; end
            if (dma_rd) begin
               if (dma_addr[4:0] != 0 && dma_errors_shown++ < 3)
                  fail($sformatf("misaligned scanout read %h", dma_addr));
               // every read must lie inside the frame being scanned (the
               // monitor's cfg switches at the VS fall, before any prefetch)
               if (mode == 3 && (int'(dma_addr) < cfg.base ||
                                 int'(dma_addr) >= cfg.base + cfg.height * cfg.stride) &&
                   dma_errors_shown++ < 3)
                  fail($sformatf("scanout read %h outside the frame [%h, %h)", dma_addr,
                                 cfg.base, cfg.base + cfg.height * cfg.stride));
               repeat (ddr_lat) @(posedge clk);
               dma_rdata <= mem_line(dma_addr);
               dma_ready <= 1'b1;
               ddr_reads++;
               @(posedge clk);
               dma_ready <= 1'b0;
               // the master must drop DV before the next read
               @(posedge clk);
               if (dma_rd && dma_errors_shown++ < 3)
                  fail("read DV not dropped after ready");
            end
         end
      end
   end
   // a read while not granted is a protocol error
   always @(posedge clk)
      if (dma_rd && !dma_grant && dma_errors_shown++ < 3)
         fail("read DV without grant");

   // ---------------- MMIO helpers (device window 0xF011_xxxx) ----------------
   task automatic mmio_write(input logic [15:0] off, input logic [63:0] d);
      @(negedge clk);
      bus.addr = {16'hF011, off}; bus.write_data = d; bus.write_DV = 1'b1;
      @(negedge clk);
      bus.write_DV = 1'b0;
   endtask

   task automatic mmio_read(input logic [15:0] off, output logic [63:0] d);
      @(negedge clk);
      bus.addr = {16'hF011, off}; bus.read_DV = 1'b1;
      @(posedge clk);
      d = bus.read_data;
      @(negedge clk);
      bus.read_DV = 1'b0;
   endtask

   task automatic expect_reg(input logic [15:0] off, input logic [63:0] want,
                             input logic [63:0] mask = '1);
      logic [63:0] d;
      mmio_read(off, d);
      if ((d & mask) !== (want & mask))
         fail($sformatf("reg %03h = %016h (want %016h mask %016h)", off, d, want, mask));
   endtask

   int errors = 0;
   task automatic fail(input string msg);
      if (errors < 20) $display("FAIL @%0t: %s", $time, msg);
      errors++;
   endtask

   // One sample per pixel: the output FFs load on the pixel CE, so sample on
   // the edge AFTER each CE (a whole CE period of stable outputs follows).
   logic ce_d = 0;
   always @(posedge clk) ce_d <= dut.w_pix_ce;

   // Pixel-domain bookkeeping
   logic        hs_q = 1, vs_q = 1;
   int          px = -1;          // pixels since the last HS falling edge
   int          hs_low = 0, vs_low = 0;
   int          ln = -1;          // HS falls since the last VS fall (fall in the VS line = 0)
   int          frames = 0;       // VS falling edges seen
   int          px_since_vs = 0;
   int          lit_first, lit_last;
   int          active_lines = 0;
   int          first_active_ln = -1;
   logic [11:0] line_buf [0:639];

   localparam logic [11:0] BORDER_COL = 12'h123;
   function automatic logic [11:0] pal_val(input int i);
      logic [7:0] b = i;
      return {b[7:4], b[3:0], 4'hA};               // never black (keeps the window check valid)
   endfunction

   // expected scanout pixel; in_img says whether (x, y) is inside the image
   function automatic logic [11:0] exp_scan(input int x, input int y, output logic in_img);
      int s, px, a;
      logic [15:0] p565;
      in_img = (y >= cfg.vstart) && (y - cfg.vstart < (cfg.height << cfg.dbl)) && (y < 480);
      if (!in_img) return BORDER_COL;
      s  = (y - cfg.vstart) >> cfg.dbl;
      px = x >> cfg.dbl;
      a  = cfg.base + s * cfg.stride + px * (cfg.bpp8 ? 1 : 2);
      if (cfg.bpp8) return pal_val(mem_byte(a));
      p565 = {mem_byte(a + 1), mem_byte(a)};
      return {p565[15:12], p565[10:7], p565[4:1]};
   endfunction

   function automatic logic [11:0] expect_bar(input int x);
      int bar = x / 80;
      return {{4{~bar[1]}}, {4{~bar[2]}}, {4{~bar[0]}}};
   endfunction

   // Line y has just finished (its pixels are px 144..783 after the HS fall
   // that preceded it). Check its window and, for chosen lines, its content.
   task automatic end_of_line(input int y);
      if (lit_first < 0) return;                 // blank line
      active_lines++;
      if (first_active_ln < 0) first_active_ln = ln;
      if (mode == 3) begin                       // image pixels may be black: bounds only
         automatic logic in_img, uf = 0;
         automatic logic [11:0] want;
         if (lit_first < 144 || lit_last > 783)
            fail($sformatf("y=%0d lit outside the active window (%0d..%0d)", y, lit_first, lit_last));
         for (int x = 0; x < 640; x++) begin
            want = exp_scan(x, y, in_img);
            if (line_buf[x] == want) continue;
            if (in_img && cfg.allow_uf && line_buf[x] == BORDER_COL) begin uf = 1; continue; end
            fail($sformatf("scanout y=%0d x=%0d = %03h (want %03h)", y, x, line_buf[x], want));
            break;
         end
         if (uf) uf_lines++;
         return;
      end
      if (lit_first != 144) fail($sformatf("y=%0d first lit px %0d (want 144)", y, lit_first));
      if (lit_last  != 783) fail($sformatf("y=%0d last lit px %0d (want 783)",  y, lit_last));
      if (mode == 1) begin
         for (int x = 0; x < 640; x++)
            if (line_buf[x] != BORDER_COL) begin
               fail($sformatf("border y=%0d x=%0d = %03h", y, x, line_buf[x])); break;
            end
         return;
      end
      if (mode == 2) begin
         for (int x = 0; x < 640; x++)
            if (line_buf[x] != pal_val((y / 30) * 16 + x / 40)) begin
               fail($sformatf("palette view y=%0d x=%0d = %03h (want %03h)", y, x,
                              line_buf[x], pal_val((y / 30) * 16 + x / 40)));
               break;
            end
         return;
      end
      if (y == 0)
         for (int x = 0; x < 640; x++)
            if (line_buf[x] != 12'hFFF) begin
               fail($sformatf("top line x=%0d = %03h (want FFF)", x, line_buf[x])); break;
            end
      if (y == 350)              // bars (square is confined to y <= 331)
         for (int k = 0; k < 8; k++)
            if (line_buf[80*k + 40] != expect_bar(80*k + 40))
               fail($sformatf("bar %0d = %03h (want %03h)", k, line_buf[80*k + 40],
                              expect_bar(80*k + 40)));
      if (y == 375 || y == 405 || y == 435 || y == 465)
         for (int k = 0; k < 16; k++) begin
            automatic logic [11:0] want;
            automatic logic [3:0]  l = k;
            case (y)
               375:     want = {l, 4'h0, 4'h0};
               405:     want = {4'h0, l, 4'h0};
               435:     want = {4'h0, 4'h0, l};
               default: want = {l, l, l};
            endcase
            // sample mid-step and both step edges (40k+1 avoids x=0 border)
            if (line_buf[40*k + 20] != want)
               fail($sformatf("ramp y=%0d step %0d = %03h (want %03h)", y, k, line_buf[40*k+20], want));
            if (k > 0 && line_buf[40*k] != want)
               fail($sformatf("ramp y=%0d step %0d edge = %03h (want %03h)", y, k, line_buf[40*k], want));
            if (k < 15 && line_buf[40*k + 39] != want)
               fail($sformatf("ramp y=%0d step %0d end = %03h (want %03h)", y, k, line_buf[40*k+39], want));
         end
   endtask

   always @(posedge clk) if (rst_l && ce_d) begin
      // ---------- HS ----------
      if (!hs) hs_low++;
      if (hs_q && !hs) begin                       // HS falling edge
         if (px >= 0 && px != 800) fail($sformatf("HS period %0d px (want 800)", px));
         if (frames >= 1) begin
            if (ln >= 0) end_of_line(ln - 34);
            ln++;                                  // -1 → 0 on the first HS after VS
         end
         px = 0;
         lit_first = -1; lit_last = -1;
      end
      if (!hs_q && hs) begin                       // HS rising edge
         if (hs_low != 96) fail($sformatf("HS low %0d px (want 96)", hs_low));
         hs_low = 0;
      end

      // ---------- VS ----------
      if (!vs) vs_low++;
      if (vs_q && !vs) begin                       // VS falling edge
         if (frames >= 1) begin
            if (px_since_vs != 420000)
               fail($sformatf("VS period %0d px (want 420000)", px_since_vs));
            if (active_lines != 480 && mode != 3)
               fail($sformatf("%0d active lines (want 480)", active_lines));
            if (first_active_ln != 34 && mode != 3)
               fail($sformatf("first active line ends at HS #%0d after VS (want 34)", first_active_ln));
            $display("frame %0d: %0d active lines, first at HS #%0d after VS",
                     frames, active_lines, first_active_ln);
         end
         frames++;
         if (cfg_pending) begin cfg = cfg_next; mode = 3; cfg_pending = 0; end
         px_since_vs = 0;
         ln = -1;                                  // the HS fall in this line becomes 0
         active_lines = 0;
         first_active_ln = -1;
      end
      if (!vs_q && vs) begin                       // VS rising edge
         if (vs_low != 1600) fail($sformatf("VS low %0d px (want 1600)", vs_low));
         vs_low = 0;
      end
      if (frames >= 1) px_since_vs++;

      // ---------- pixel ----------
      if (px >= 0) begin
         if ({r, g, b} != 12'h000) begin
            if (lit_first < 0) lit_first = px;
            lit_last = px;
         end
         if (px >= 144 && px < 784) line_buf[px - 144] = {r, g, b};
         // sync and colour must never overlap
         if ((!hs || !vs) && {r, g, b} != 12'h000) fail($sformatf("lit pixel during sync, px=%0d", px));
         px++;
      end
      hs_q = hs;
      vs_q = vs;
   end

   // wait until the raster is in the active picture (not vblank)
   task automatic wait_active();
      logic [63:0] d;
      do mmio_read(16'h018, d); while (d[0] || d[41:32] < 10'd8);
   endtask

   // program a scanout frame (takes effect at the next frame) and tell the
   // monitor to expect it from the next VS fall
   task automatic set_scan(input scfg_t c, input logic [5:0] extra = 6'h0);
      mmio_write(16'h008, 64'(c.base));
      mmio_write(16'h010, 64'(c.stride));
      mmio_write(16'h038, 64'(c.height));
      mmio_write(16'h028, 64'(c.vstart));
      mmio_write(16'h000, {58'b0, extra | {3'b000, c.bpp8, c.dbl, 1'b1}});
      cfg_next = c;
      cfg_pending = 1;
   endtask

   initial begin
      logic [63:0] d;
      realtime     t1, t2;
      repeat (10) @(posedge clk);
      rst_l = 1;

      // ---- frames 1-2: reset test pattern (checked by the pixel monitor) ----
      wait (frames == 3);                          // just after VS fall, line 490
      expect_reg(16'h000, 64'h08);                 // CTRL reset = TEST_PATTERN
      expect_reg(16'h008, 64'h0);
      expect_reg(16'h010, 64'd640);
      expect_reg(16'h020, 64'h0);
      expect_reg(16'h028, 64'h0);
      expect_reg(16'h030, 64'h0);
      // STATUS: in vblank, pending (never cleared), frame 3, line 490, underflow 0
      expect_reg(16'h018, {16'h0, 6'h0, 10'd490, 16'd3, 14'h0, 1'b1, 1'b1});
      expect_reg(16'h7F8, 64'h0);                  // unmapped offset reads 0
      if (irq !== 1'b0) fail("irq high with IRQ_EN=0");
      mmio_write(16'h020, BORDER_COL);
      mmio_write(16'h000, 64'h0);                  // border mode
      expect_reg(16'h020, BORDER_COL);
      mode = 1;

      // ---- frame 3: border; load + read back the palette ----
      wait (frames == 4);
      for (int i = 0; i < 256; i++) mmio_write(16'h800 + 8*i, {52'hFFFF_FFFF_FFFF_F, pal_val(i)});
      for (int i = 0; i < 256; i++) expect_reg(16'h800 + 8*i, pal_val(i));
      mmio_write(16'h000, 64'h20);                 // PALETTE_VIEW
      mode = 2;

      // ---- frame 4+: palette view; vsync IRQ and the FB_BASE latch ----
      wait (frames == 5);
      mmio_write(16'h000, 64'h30);                 // + VSYNC_IRQ_EN
      @(negedge clk);
      if (irq !== 1'b1) fail("irq not raised by an already-pending vsync");
      mmio_write(16'h018, 64'h2);                  // W1C pending
      @(negedge clk);
      if (irq !== 1'b0) fail("irq still high after W1C");
      expect_reg(16'h018, 64'h0, 64'h2);

      // flip request during the active picture must not take effect until vblank
      do mmio_read(16'h018, d); while (d[0]);      // wait for the active area
      mmio_write(16'h008, 64'h00AB_C000);
      expect_reg(16'h008, 64'h00AB_C000);
      expect_reg(16'h030, 64'h0);                  // still the old base

      @(posedge irq); t1 = $realtime;
      mmio_read(16'h018, d);
      if (d[41:32] != 10'd480) fail($sformatf("IRQ at line %0d (want 480)", d[41:32]));
      if (!d[0])               fail("IRQ outside vblank");
      if (d[31:16] != 16'd6)   fail($sformatf("frame count %0d at IRQ (want 6)", d[31:16]));
      expect_reg(16'h030, 64'h00AB_C000);          // latched at vblank
      mmio_write(16'h018, 64'h2);
      @(posedge irq); t2 = $realtime;
      if (t2 - t1 != 420000 * 4 * 10)
         fail($sformatf("vsync IRQ period %0t (want 16.8 ms)", t2 - t1));
      mmio_write(16'h018, 64'h2);
      $display("vsync IRQ period %0.3f ms", (t2 - t1) / 1.0e6);

      // ---- frames 8-11: DDR scanout ----
      // Settings latch at the start of vblank (line 480), so each frame's
      // config is written in the preceding frame's ACTIVE period; the monitor
      // switches its expectation at the next VS fall.
      mmio_write(16'h018, 64'h4);                  // clear UNDERFLOW
      wait (frames == 7);                          // palette view frame 7 still on
      wait_active();
      // PALETTE_VIEW (an immediate pixel source) stays on for the rest of
      // frame 7; it is dropped in the vblank before frame 8 below.
      set_scan('{dbl:1, bpp8:0, base:32'h00AB_C000, stride:640,  height:240, vstart:0,  allow_uf:0}, 6'h20);
      wait (frames == 8);
      mmio_write(16'h000, 64'h03);                 // SCANOUT_EN | DOUBLE, palette view off
      wait_active();
      set_scan('{dbl:0, bpp8:1, base:32'h0020_0000, stride:1024, height:400, vstart:40, allow_uf:0});
      wait (frames == 9);
      ddr_gdelay = 200;                            // contention: grant up to 2 us late
      wait_active();
      set_scan('{dbl:1, bpp8:0, base:32'h0030_0020, stride:640,  height:200, vstart:40, allow_uf:0});
      wait (frames == 10);
      wait_active();
      set_scan('{dbl:0, bpp8:0, base:32'h0040_0000, stride:1280, height:480, vstart:0, allow_uf:1});
      wait (frames == 11);
      ddr_gdelay = 0;
      expect_reg(16'h018, 64'h0, 64'hFFFF_0000_0000_0000);   // frames 8-10: no underflow
      ddr_lat = 150;                               // 40 reads x ~150 cycles > one line
      wait_active();
      mmio_write(16'h000, 64'h00);                 // frame 12: scanout off -> border
      wait (frames == 12);
      mode = 1;
      ddr_lat = 20;
      mmio_read(16'h018, d);
      $display("underflow: %0d display lines counted, %0d lines showed border in the image, %0d DDR reads",
               d[63:48], uf_lines, ddr_reads);
      if (d[63:48] == 0 || uf_lines == 0) fail("starved frame produced no underflow");
      mmio_write(16'h018, 64'h4);
      expect_reg(16'h018, 64'h0, 64'hFFFF_0000_0000_0000);   // W1C cleared it

      wait (frames == 13);                         // border frame 12 checked
      repeat (10) @(posedge clk);
      if (errors == 0) $display("TB_VGA PASS");
      else             $display("TB_VGA FAIL (%0d errors)", errors);
      $finish;
   end

   initial begin
      #280ms;
      $display("TB_VGA TIMEOUT (frames=%0d)", frames);
      $finish;
   end
endmodule

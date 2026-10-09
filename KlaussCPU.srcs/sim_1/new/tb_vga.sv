//===========================================================================
// tb_vga — VGA_PLAN.md Phase 0 gate: vga_ctrl's 640×480 @ 60 Hz pin timing.
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
//   * pattern spot checks: full-white top line, colour bars, ramp levels
//
// Run: perf/m5a/run_vga.sh   (prints "TB_VGA PASS" on success)
//===========================================================================
`timescale 1ns / 1ps
module tb_vga;
   logic clk = 0, rst_l = 0;
   always #5 clk = ~clk;

   logic [3:0] r, g, b;
   logic       hs, vs;

   vga_ctrl dut (
      .i_Clk(clk), .i_Rst_L(rst_l),
      .o_vga_r(r), .o_vga_g(g), .o_vga_b(b), .o_vga_hs(hs), .o_vga_vs(vs)
   );

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
      if (lit_first != 144) fail($sformatf("y=%0d first lit px %0d (want 144)", y, lit_first));
      if (lit_last  != 783) fail($sformatf("y=%0d last lit px %0d (want 783)",  y, lit_last));
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
            if (active_lines != 480)
               fail($sformatf("%0d active lines (want 480)", active_lines));
            if (first_active_ln != 34)
               fail($sformatf("first active line ends at HS #%0d after VS (want 34)", first_active_ln));
            $display("frame %0d: %0d active lines, first at HS #%0d after VS",
                     frames, active_lines, first_active_ln);
         end
         frames++;
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

   initial begin
      repeat (10) @(posedge clk);
      rst_l = 1;
      wait (frames == 3);                          // two complete frames checked
      repeat (10) @(posedge clk);
      if (errors == 0) $display("TB_VGA PASS");
      else             $display("TB_VGA FAIL (%0d errors)", errors);
      $finish;
   end

   initial begin
      #80ms;
      $display("TB_VGA TIMEOUT (frames=%0d)", frames);
      $finish;
   end
endmodule

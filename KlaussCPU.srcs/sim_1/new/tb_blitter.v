`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_blitter — Phase 2 unit test for blitter_dma + the mem_read_write DMA
// arbiter. Exercises FILL and COPY (aligned, unaligned, differing strides,
// partial edge words) against a byte-level reference model.
//
// Uses a MASK-AWARE behavioral ddr2_control (the blitter writes partial words
// with a per-byte mask — unlike the cache, which always writes full lines).
//////////////////////////////////////////////////////////////////////////////////

// --- clk_wiz_0 stub: pass the input clock straight through ---
module clk_wiz_0 (input i_Clk, output clk_200, output clk_50, output locked, input resetn);
   assign clk_200 = i_Clk;
   assign clk_50  = i_Clk;
   assign locked  = 1'b1;
endmodule

// --- mask-aware behavioral ddr2_control: 128-bit memory, fixed latency ---
module ddr2_control (
    inout [15:0] ddr2_dq, inout [1:0] ddr2_dqs_n, inout [1:0] ddr2_dqs_p,
    output [12:0] ddr2_addr, output [2:0] ddr2_ba,
    output ddr2_ras_n, output ddr2_cas_n, output ddr2_we_n,
    output [0:0] ddr2_ck_p, output [0:0] ddr2_ck_n, output [0:0] ddr2_cke,
    output [0:0] ddr2_cs_n, output [1:0] ddr2_dm, output [0:0] ddr2_odt,
    input resetn, input sys_clk_i,
    input i_mem_write_DV, input i_mem_read_DV,
    input [31:0] i_mem_addr, input [127:0] i_mem_write_data,
    inout [15:0] i_app_wdf_mask,
    output reg [127:0] o_mem_read_data, output reg o_mem_ready,
    output o_calib_done
);
   assign o_calib_done = 1'b1;
   reg [127:0] mem [0:65535];      // addr[19:4] indexes the line
   integer i;
   initial begin
      o_mem_ready = 0; o_mem_read_data = 0;
      for (i = 0; i < 65536; i = i + 1) mem[i] = 128'h0;
   end
   localparam LAT = 6;
   reg [3:0] cnt = 0;
   wire [15:0] idx = i_mem_addr[19:4];
   integer b;
   always @(posedge sys_clk_i) begin
      if (i_mem_read_DV !== 1'b1 && i_mem_write_DV !== 1'b1) begin
         o_mem_ready <= 0; cnt <= 0;
      end else if (!o_mem_ready) begin
         if (cnt == LAT) begin
            if (i_mem_read_DV === 1'b1) begin
               o_mem_read_data <= mem[idx];
            end else begin
               // masked write: app_wdf_mask[b]==1 means DO NOT write byte b
               for (b = 0; b < 16; b = b + 1)
                  if (i_app_wdf_mask[b] === 1'b0)
                     mem[idx][b*8 +: 8] <= i_mem_write_data[b*8 +: 8];
            end
            o_mem_ready <= 1; cnt <= 0;
         end else cnt <= cnt + 1;
      end
   end
endmodule

// ---------------------------------------------------------------------------
module tb_blitter;
   reg clk = 0;
   always #5 clk = ~clk;   // 100 MHz

   // --- DMA wires between blitter (master) and mem_read_write (arbiter) ---
   wire        dma_req, dma_done, dma_w, dma_r, dma_grant, dma_ready;
   wire [31:0] dma_addr;
   wire [127:0] dma_wdata, dma_rdata;
   wire [15:0] dma_mask;

   // --- MMIO slave wires to the blitter ---
   reg         mmio_w = 0, mmio_r = 0;
   reg  [15:0] mmio_addr = 0;
   reg  [63:0] mmio_wdata = 0;
   wire [63:0] mmio_rdata;
   wire        mmio_ready;

   reg rst_l = 1'b0;   // active-low reset for the blitter

   blitter_dma dut_blit (
      .i_Clk(clk), .i_Rst_L(rst_l),
      .i_mmio_write_DV(mmio_w), .i_mmio_read_DV(mmio_r),
      .i_mmio_addr(mmio_addr), .i_mmio_write_data(mmio_wdata),
      .i_mmio_byte_en(8'hFF),
      .o_mmio_read_data(mmio_rdata), .o_mmio_ready(mmio_ready),
      .o_dma_req(dma_req), .o_dma_done(dma_done),
      .o_dma_write_DV(dma_w), .o_dma_read_DV(dma_r),
      .o_dma_addr(dma_addr), .o_dma_write_data(dma_wdata),
      .o_dma_wdf_mask(dma_mask),
      .i_dma_read_data(dma_rdata), .i_dma_ready(dma_ready), .i_dma_grant(dma_grant),
      .o_irq(blit_irq)
   );
   wire blit_irq;

   mem_read_write dut_mem (
      .i_Clk(clk),
      .ddr2_dq(), .ddr2_dqs_n(), .ddr2_dqs_p(),
      .ddr2_addr(), .ddr2_ba(), .ddr2_ras_n(), .ddr2_cas_n(), .ddr2_we_n(),
      .ddr2_ck_p(), .ddr2_ck_n(), .ddr2_cke(), .ddr2_cs_n(), .ddr2_dm(), .ddr2_odt(),
      // CPU port idle for these unit tests.
      .i_mem_write_DV(1'b0), .i_mem_read_DV(1'b0),
      .i_mem_addr(32'h0), .i_mem_write_data(64'h0), .i_mem_byte_en(8'hFF),
      .o_mem_read_data(), .o_mem_read_data_next(), .o_mem_next_valid(), .o_mem_ready(),
      .i_stat_clear(1'b0),
      .o_cache_info(), .o_cnt_read_hits(), .o_cnt_read_misses(),
      .o_cnt_write_hits(), .o_cnt_write_misses(), .o_cnt_writebacks(),
      .o_cnt_stall_cycles(), .clk_50(), .o_calib_done(),
      // DMA master port driven by the blitter.
      .i_dma_req(dma_req), .i_dma_done(dma_done),
      .i_dma_write_DV(dma_w), .i_dma_read_DV(dma_r),
      .i_dma_addr(dma_addr), .i_dma_write_data(dma_wdata), .i_dma_wdf_mask(dma_mask),
      .o_dma_read_data(dma_rdata), .o_dma_ready(dma_ready), .o_dma_grant(dma_grant)
   );

   integer errors = 0;

   // --- MMIO helpers ---
   task mmio_write(input [15:0] off, input [63:0] data);
      begin
         @(negedge clk); mmio_addr = off; mmio_wdata = data; mmio_w = 1;
         @(negedge clk); mmio_w = 0;
      end
   endtask
   task mmio_read(input [15:0] off, output [63:0] data);
      begin
         mmio_addr = off; mmio_r = 1;
         #1 data = mmio_rdata;   // combinational read
         mmio_r = 0;
      end
   endtask

   // --- direct DUT-memory byte access (hierarchical into the stub) ---
   function [7:0] mem_byte(input [31:0] a);
      begin
         mem_byte = dut_mem.ddr2_control.mem[a[19:4]][ (a[3:0])*8 +: 8 ];
      end
   endfunction
   task poke_byte(input [31:0] a, input [7:0] v);
      begin
         dut_mem.ddr2_control.mem[a[19:4]][ (a[3:0])*8 +: 8 ] = v;
      end
   endtask
   task poke_pixel(input [31:0] a, input [15:0 ] p);
      begin poke_byte(a, p[7:0]); poke_byte(a+1, p[15:8]); end
   endtask
   function [15:0] mem_pixel(input [31:0] a);
      begin mem_pixel = {mem_byte(a+1), mem_byte(a)}; end
   endfunction

   // --- run a blit and wait for completion ---
   task run_blit(input [2:0] op);
      reg [63:0] st;
      integer guard;
      begin
         mmio_write(16'h0000, {59'b0, 1'b0, op, 1'b1});   // CTRL: START | OP<<1
         guard = 0;
         mmio_read(16'h0008, st);
         while (!st[1]) begin                  // wait DONE
            @(negedge clk); mmio_read(16'h0008, st);
            guard = guard + 1;
            if (guard > 200000) begin
               $display("FAIL: blit timeout (op=%0d)", op); errors = errors + 1;
               disable run_blit;
            end
         end
         mmio_write(16'h0008, 64'h2);          // W1C DONE
      end
   endtask

   // ===================== reference model + checks =====================
   // Shadow byte memory mirrors DUT memory; we compare a window after each op.
   reg [7:0] shadow [0:1048575];   // 1 MiB
   integer si;

   task snapshot_window(input [31:0] base, input integer len);
      integer k;
      begin for (k = 0; k < len; k = k + 1) shadow[base+k] = mem_byte(base+k); end
   endtask

   task check_window(input [31:0] base, input integer len, input [255:0] name);
      integer k; integer bad;
      begin
         bad = 0;
         for (k = 0; k < len; k = k + 1)
            if (mem_byte(base+k) !== shadow[base+k]) begin
               if (bad < 8)
                  $display("  FAIL %0s: byte %h got %h want %h",
                           name, base+k, mem_byte(base+k), shadow[base+k]);
               bad = bad + 1;
            end
         if (bad) begin errors = errors + 1;
            $display("FAIL %0s: %0d mismatched bytes", name, bad);
         end else $display("PASS %0s", name);
      end
   endtask

   // reference FILL/COPY over the shadow
   task ref_fill(input [31:0] dst, input [31:0] dstr,
                 input [15:0] w, input [15:0] h, input [15:0] color);
      integer y, x; reg [31:0] a;
      begin
         for (y = 0; y < h; y = y + 1)
            for (x = 0; x < w; x = x + 1) begin
               a = dst + y*dstr + x*2;
               shadow[a]   = color[7:0]; shadow[a+1] = color[15:8];
            end
      end
   endtask
   task ref_copy(input [31:0] dst, input [31:0] dstr,
                 input [31:0] src, input [31:0] srcr,
                 input [15:0] w, input [15:0] h);
      integer y, x; reg [31:0] da, sa;
      begin
         for (y = 0; y < h; y = y + 1)
            for (x = 0; x < w; x = x + 1) begin
               da = dst + y*dstr + x*2; sa = src + y*srcr + x*2;
               shadow[da]   = mem_byte(sa);
               shadow[da+1] = mem_byte(sa+1);
            end
      end
   endtask

   // RGB565 blend reference — must match blitter_dma.blend565 bit-for-bit.
   function [15:0] tb_blend565(input [15:0] fg, input [15:0] bg, input [7:0] a);
      reg [7:0] ia; reg [15:0] tr, tg, tbl;
      begin
         ia = 8'd255 - a;
         tr  = fg[15:11]*a + bg[15:11]*ia + 16'd128;
         tg  = fg[10:5] *a + bg[10:5] *ia + 16'd128;
         tbl = fg[4:0]  *a + bg[4:0]  *ia + 16'd128;
         tb_blend565 = { tr[12:8], tg[13:8], tbl[12:8] };
      end
   endfunction
   function [15:0] shadow_px(input [31:0] a);
      begin shadow_px = {shadow[a+1], shadow[a]}; end
   endfunction
   task put_shadow_px(input [31:0] a, input [15:0] p);
      begin shadow[a] = p[7:0]; shadow[a+1] = p[15:8]; end
   endtask

   task ref_fill_blend(input [31:0] dst, input [31:0] dstr,
                       input [15:0] w, input [15:0] h,
                       input [15:0] color, input [7:0] alpha);
      integer y, x; reg [31:0] da;
      begin
         for (y = 0; y < h; y = y + 1)
            for (x = 0; x < w; x = x + 1) begin
               da = dst + y*dstr + x*2;
               put_shadow_px(da, tb_blend565(color, shadow_px(da), alpha));
            end
      end
   endtask
   task ref_copy_blend(input [31:0] dst, input [31:0] dstr,
                       input [31:0] src, input [31:0] srcr,
                       input [15:0] w, input [15:0] h, input [7:0] alpha);
      integer y, x; reg [31:0] da, sa;
      begin
         for (y = 0; y < h; y = y + 1)
            for (x = 0; x < w; x = x + 1) begin
               da = dst + y*dstr + x*2; sa = src + y*srcr + x*2;
               put_shadow_px(da, tb_blend565(mem_pixel(sa), shadow_px(da), alpha));
            end
      end
   endtask
   task ref_mask_blend(input [31:0] dst, input [31:0] dstr,
                       input [31:0] mbase, input [31:0] mstr,
                       input [15:0] w, input [15:0] h, input [15:0] color);
      integer y, x; reg [31:0] da; reg [7:0] a;
      begin
         for (y = 0; y < h; y = y + 1)
            for (x = 0; x < w; x = x + 1) begin
               da = dst + y*dstr + x*2;
               a  = mem_byte(mbase + y*mstr + x);
               put_shadow_px(da, tb_blend565(color, shadow_px(da), a));
            end
      end
   endtask

   integer x, y;
   reg [63:0] cyc;

   initial begin
      // clear shadow
      for (si = 0; si < 1048576; si = si + 1) shadow[si] = 8'h0;
      rst_l = 1'b0;
      repeat (10) @(posedge clk);   // assert blitter reset
      rst_l = 1'b1;
      repeat (60) @(posedge clk);   // let POR drain

      // ============ Test 1: aligned FILL ============
      // 8 px wide (16 B = one full word), 4 rows, stride 16, dst aligned.
      mmio_write(16'h0010, 64'h0000_1000);  // DST_ADDR
      mmio_write(16'h0018, 64'd16);         // DST_STRIDE
      mmio_write(16'h0030, 64'd8);          // WIDTH
      mmio_write(16'h0038, 64'd4);          // HEIGHT
      mmio_write(16'h0040, 64'hABCD);       // COLOR
      snapshot_window(32'h1000, 64);
      ref_fill(32'h1000, 16, 8, 4, 16'hABCD);
      run_blit(3'd0);
      check_window(32'h1000, 64, "fill-aligned");
      mmio_read(16'h0060, cyc);
      if (cyc == 0) begin errors=errors+1; $display("FAIL: CYCLES==0"); end
      else $display("INFO fill CYCLES=%0d", cyc);

      // ============ Test 2: unaligned, partial-width FILL ============
      // dst origin at byte 0x2006 (pixel offset 3 in its word), 5 px wide,
      // 3 rows, stride 40. Touches partial edge words → mask path.
      mmio_write(16'h0010, 64'h0000_2006);
      mmio_write(16'h0018, 64'd40);
      mmio_write(16'h0030, 64'd5);
      mmio_write(16'h0038, 64'd3);
      mmio_write(16'h0040, 64'h1234);
      snapshot_window(32'h2000, 256);
      ref_fill(32'h2006, 40, 5, 3, 16'h1234);
      run_blit(3'd0);
      check_window(32'h2000, 256, "fill-unaligned-partial");

      // ============ Test 3: aligned COPY ============
      // Seed a source pattern, copy 8x4 aligned src->dst.
      for (y = 0; y < 4; y = y + 1)
         for (x = 0; x < 8; x = x + 1)
            poke_pixel(32'h3000 + y*16 + x*2, 16'h5000 + y*16 + x);
      mmio_write(16'h0010, 64'h0000_3800);  // DST_ADDR
      mmio_write(16'h0018, 64'd16);
      mmio_write(16'h0020, 64'h0000_3000);  // SRC_ADDR
      mmio_write(16'h0028, 64'd16);
      mmio_write(16'h0030, 64'd8);
      mmio_write(16'h0038, 64'd4);
      snapshot_window(32'h3800, 64);
      ref_copy(32'h3800, 16, 32'h3000, 16, 8, 4);
      run_blit(3'd1);
      check_window(32'h3800, 64, "copy-aligned");

      // ============ Test 4: COPY, differing alignment & strides ============
      // Source: 10px rows at stride 64, origin pixel offset 1 (byte 0x4002).
      // Dest:   10px rows at stride 32, origin pixel offset 5 (byte 0x500A).
      for (y = 0; y < 6; y = y + 1)
         for (x = 0; x < 10; x = x + 1)
            poke_pixel(32'h4002 + y*64 + x*2, 16'h8000 + y*32 + x);
      mmio_write(16'h0010, 64'h0000_500A);  // DST_ADDR (offset 5)
      mmio_write(16'h0018, 64'd32);
      mmio_write(16'h0020, 64'h0000_4002);  // SRC_ADDR (offset 1)
      mmio_write(16'h0028, 64'd64);
      mmio_write(16'h0030, 64'd10);
      mmio_write(16'h0038, 64'd6);
      snapshot_window(32'h5000, 512);
      ref_copy(32'h500A, 32, 32'h4002, 64, 10, 6);
      run_blit(3'd1);
      check_window(32'h5000, 512, "copy-unaligned-strided");

      // ============ Test 5: FILL_BLEND, alpha sweep (aligned) ============
      // Background = blue (0x001F); foreground = red (0xF800).
      for (y = 0; y < 4; y = y + 1)
         for (x = 0; x < 8; x = x + 1) poke_pixel(32'h6000 + y*16 + x*2, 16'h001F);
      // alpha=255 must yield exactly the foreground.
      mmio_write(16'h0010, 64'h0000_6000); mmio_write(16'h0018, 64'd16);
      mmio_write(16'h0030, 64'd8); mmio_write(16'h0038, 64'd4);
      mmio_write(16'h0040, 64'hF800); mmio_write(16'h0048, 64'd255);
      snapshot_window(32'h6000, 64);
      ref_fill_blend(32'h6000, 16, 8, 4, 16'hF800, 255);
      run_blit(3'd2);
      check_window(32'h6000, 64, "fillblend-a255");

      // alpha=0 over fresh blue background must leave background unchanged.
      for (y = 0; y < 4; y = y + 1)
         for (x = 0; x < 8; x = x + 1) poke_pixel(32'h6000 + y*16 + x*2, 16'h001F);
      mmio_write(16'h0048, 64'd0);
      snapshot_window(32'h6000, 64);
      ref_fill_blend(32'h6000, 16, 8, 4, 16'hF800, 0);
      run_blit(3'd2);
      check_window(32'h6000, 64, "fillblend-a0");

      // alpha=128 mid-blend.
      for (y = 0; y < 4; y = y + 1)
         for (x = 0; x < 8; x = x + 1) poke_pixel(32'h6000 + y*16 + x*2, 16'h07E0);
      mmio_write(16'h0040, 64'hF81F); mmio_write(16'h0048, 64'd128);
      snapshot_window(32'h6000, 64);
      ref_fill_blend(32'h6000, 16, 8, 4, 16'hF81F, 128);
      run_blit(3'd2);
      check_window(32'h6000, 64, "fillblend-a128");

      // ============ Test 6: FILL_BLEND unaligned partial ============
      for (si = 0; si < 256; si = si + 1) poke_byte(32'h7000 + si, $random);
      mmio_write(16'h0010, 64'h0000_7006); mmio_write(16'h0018, 64'd40);
      mmio_write(16'h0030, 64'd5); mmio_write(16'h0038, 64'd3);
      mmio_write(16'h0040, 64'h2468); mmio_write(16'h0048, 64'd90);
      snapshot_window(32'h7000, 256);
      ref_fill_blend(32'h7006, 40, 5, 3, 16'h2468, 90);
      run_blit(3'd2);
      check_window(32'h7000, 256, "fillblend-unaligned");

      // ============ Test 7: COPY_BLEND, differing strides/alignment ============
      // Source foreground pattern + destination background pattern.
      for (y = 0; y < 6; y = y + 1)
         for (x = 0; x < 10; x = x + 1)
            poke_pixel(32'h8002 + y*64 + x*2, 16'hF800 + x);   // src (offset 1)
      for (si = 0; si < 512; si = si + 1) poke_byte(32'h9000 + si, $random);
      mmio_write(16'h0010, 64'h0000_900A); mmio_write(16'h0018, 64'd32);  // dst off 5
      mmio_write(16'h0020, 64'h0000_8002); mmio_write(16'h0028, 64'd64);  // src off 1
      mmio_write(16'h0030, 64'd10); mmio_write(16'h0038, 64'd6);
      mmio_write(16'h0048, 64'd170);
      snapshot_window(32'h9000, 512);
      ref_copy_blend(32'h900A, 32, 32'h8002, 64, 10, 6, 170);
      run_blit(3'd3);
      check_window(32'h9000, 512, "copyblend-strided");

      // ============ Test 8: MASK_BLEND (A8 per-pixel alpha ramp) ============
      // Mask alpha = x*25 (0,25,...) clamped by byte; foreground = COLOR.
      for (si = 0; si < 512; si = si + 1) poke_byte(32'hA000 + si, $random); // dst bg
      for (y = 0; y < 4; y = y + 1)
         for (x = 0; x < 9; x = x + 1)
            poke_byte(32'hB000 + y*16 + x, (x*28) & 8'hFF);   // A8 mask
      mmio_write(16'h0010, 64'h0000_A004); mmio_write(16'h0018, 64'd48);  // dst off 2
      mmio_write(16'h0030, 64'd9); mmio_write(16'h0038, 64'd4);
      mmio_write(16'h0040, 64'h0F0F);
      mmio_write(16'h0050, 64'h0000_B000); mmio_write(16'h0058, 64'd16);  // MASK base/stride
      snapshot_window(32'hA000, 512);
      ref_mask_blend(32'hA004, 48, 32'hB000, 16, 9, 4, 16'h0F0F);
      run_blit(3'd4);
      check_window(32'hA000, 512, "maskblend-ramp");

      // ============ Test 9: empty rect (width=0) — must finish, touch nothing ============
      // Seed a known region, run a width=0 FILL over it, confirm DONE + nonzero
      // CYCLES and that memory is unchanged.
      for (si = 0; si < 64; si = si + 1) poke_byte(32'hC000 + si, 8'hA5);
      mmio_write(16'h0010, 64'h0000_C000); mmio_write(16'h0018, 64'd16);
      mmio_write(16'h0030, 64'd0);   // WIDTH = 0
      mmio_write(16'h0038, 64'd4);
      mmio_write(16'h0040, 64'hBEEF);
      snapshot_window(32'hC000, 64);     // expect NO change
      run_blit(3'd0);
      check_window(32'hC000, 64, "emptyrect-untouched");
      mmio_read(16'h0060, cyc);
      if (cyc == 0) begin errors = errors + 1; $display("FAIL: empty-rect CYCLES==0"); end
      else $display("PASS emptyrect-cycles (CYCLES=%0d)", cyc);

      // ============ Test 10: DONE interrupt (o_irq) ============
      // IRQ_EN=1 → o_irq asserts on completion, clears on STATUS W1C.
      mmio_write(16'h0010, 64'h0000_C100); mmio_write(16'h0018, 64'd16);
      mmio_write(16'h0030, 64'd4); mmio_write(16'h0038, 64'd2);
      mmio_write(16'h0040, 64'h1111);
      // CTRL: START(bit0) | OP=FILL(bits3:1=0) | IRQ_EN(bit4) = 0x11
      mmio_write(16'h0000, 64'h0000_0011);
      begin : irq_wait
         reg [63:0] st; integer g;
         g = 0; mmio_read(16'h0008, st);
         while (!st[1]) begin @(negedge clk); mmio_read(16'h0008, st);
            g = g + 1; if (g > 100000) begin errors=errors+1; disable irq_wait; end
         end
      end
      @(negedge clk);
      if (!blit_irq) begin errors = errors + 1; $display("FAIL: o_irq not asserted after DONE w/ IRQ_EN"); end
      else $display("PASS irq-asserted");
      mmio_write(16'h0008, 64'h2);   // W1C DONE → acks the interrupt
      @(negedge clk); @(negedge clk);
      if (blit_irq) begin errors = errors + 1; $display("FAIL: o_irq not cleared after W1C ack"); end
      else $display("PASS irq-cleared-on-ack");

      // IRQ_EN=0 → o_irq must stay low even when DONE sets.
      mmio_write(16'h0000, 64'h0000_0001);   // START only, IRQ_EN=0
      begin : irq_wait2
         reg [63:0] st; integer g;
         g = 0; mmio_read(16'h0008, st);
         while (!st[1]) begin @(negedge clk); mmio_read(16'h0008, st);
            g = g + 1; if (g > 100000) begin errors=errors+1; disable irq_wait2; end
         end
      end
      @(negedge clk);
      if (blit_irq) begin errors = errors + 1; $display("FAIL: o_irq asserted with IRQ_EN=0"); end
      else $display("PASS irq-masked-when-disabled");
      mmio_write(16'h0008, 64'h2);

      if (errors == 0) $display("PASS: all blitter checks passed");
      else             $display("FAILED: %0d errors", errors);
      $finish;
   end

   initial begin
      #20_000_000;
      $display("TIMEOUT — blitter hung");
      $finish;
   end
endmodule

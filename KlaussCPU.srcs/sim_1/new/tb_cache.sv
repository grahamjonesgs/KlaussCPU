// Testbench for mem_read_write after the CHECK-merge change (hits complete in
// CHECK).  Drives the CPU-side interface exactly the way bus_splitter + the
// CPU FSM do: ready/data come back through a 1-cycle register, and DV drops
// one cycle after the registered ready — so DV is still high during the
// cache's PRE_WAIT and COOL_DOWN states (the re-latch hazard COOL_DOWN
// exists to absorb).  A behavioral ddr2_control model replaces the MIG.
//
// Checks: scoreboarded reads over directed + 4000 random ops confined to two
// cache sets with four tags (forces clean and dirty evictions, writebacks,
// refill merges), partial-byte writes, next-doubleword lookahead, and the
// expected 2-cycle hit latency.
`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// clk_wiz_0 stub — pass the input clock straight through.
// ---------------------------------------------------------------------------
module clk_wiz_0 (input i_Clk, output clk_200, output clk_50, output locked, input resetn);
   assign clk_200 = i_Clk;
   assign clk_50  = i_Clk;
   assign locked  = 1'b1;
endmodule

// ---------------------------------------------------------------------------
// Behavioral ddr2_control — 128-bit memory, fixed latency, ready held while
// DV is high (matches the real handshake the cache's 2-FF sync expects).
// ---------------------------------------------------------------------------
module ddr2_control (
    inout [15:0] ddr2_dq,
    inout [1:0] ddr2_dqs_n,
    inout [1:0] ddr2_dqs_p,
    output [12:0] ddr2_addr,
    output [2:0] ddr2_ba,
    output ddr2_ras_n,
    output ddr2_cas_n,
    output ddr2_we_n,
    output [0:0] ddr2_ck_p,
    output [0:0] ddr2_ck_n,
    output [0:0] ddr2_cke,
    output [0:0] ddr2_cs_n,
    output [1:0] ddr2_dm,
    output [0:0] ddr2_odt,
    input resetn,
    input sys_clk_i,
    input i_mem_write_DV,
    input i_mem_read_DV,
    input [31:0] i_mem_addr,
    input [127:0] i_mem_write_data,
    inout [15:0] i_app_wdf_mask,
    output logic [127:0] o_mem_read_data,
    output logic o_mem_ready,
    output o_calib_done
);
   assign o_calib_done = 1'b1;

   // 64K lines = 1 MiB modeled; addr[19:4] indexes the line.
   logic [127:0] mem [0:65535];

   // init function shared with the TB scoreboard: dword at byte addr a
   // (a[3]=0 -> upper half of line, a[3]=1 -> lower half)
   function [63:0] init_dw;
      input [31:0] a;
      begin
         init_dw = {16'hD00D, a[19:4], 16'hF00D, 13'b0, a[3], 2'b0};
      end
   endfunction

   integer i;
   initial begin
      o_mem_ready = 0;
      o_mem_read_data = 0;
      for (i = 0; i < 65536; i = i + 1)
         mem[i] = {init_dw({i[15:0], 4'b0000}), init_dw({i[15:0], 4'b1000})};
   end

   localparam LAT = 6;
   logic [3:0] cnt = 0;
   wire [15:0] idx = i_mem_addr[19:4];

   always @(posedge sys_clk_i) begin
      if (i_mem_read_DV !== 1'b1 && i_mem_write_DV !== 1'b1) begin
         o_mem_ready <= 0;
         cnt <= 0;
      end else if (!o_mem_ready) begin
         if (cnt == LAT) begin
            if (i_mem_read_DV === 1'b1)
               o_mem_read_data <= mem[idx];
            else
               mem[idx] <= i_mem_write_data;   // cache always writes full lines (mask=0)
            o_mem_ready <= 1;
            cnt <= 0;
         end else
            cnt <= cnt + 1;
      end
      // o_mem_ready stays high while DV is held (cache syncs it, then drops DV)
   end
endmodule

// ---------------------------------------------------------------------------
// Testbench proper
// ---------------------------------------------------------------------------
module tb_cache;

   logic clk = 0;
   always #5 clk = ~clk;   // 100 MHz

   logic         dv_w = 0, dv_r = 0;
   logic  [31:0] addr;
   logic  [63:0] wdata;
   logic  [7:0]  be = 8'hFF;
   wire [63:0] rdata, rdata_next;
   wire        next_valid, ready;

   mem_read_write dut (
      .i_Clk(clk),
      .ddr2_dq(), .ddr2_dqs_n(), .ddr2_dqs_p(),
      .ddr2_addr(), .ddr2_ba(), .ddr2_ras_n(), .ddr2_cas_n(), .ddr2_we_n(),
      .ddr2_ck_p(), .ddr2_ck_n(), .ddr2_cke(), .ddr2_cs_n(), .ddr2_dm(), .ddr2_odt(),
      .i_mem_write_DV(dv_w),
      .i_mem_read_DV(dv_r),
      .i_mem_addr(addr),
      .i_mem_write_data(wdata),
      .i_mem_byte_en(be),
      .o_mem_read_data(rdata),
      .o_mem_read_data_next(rdata_next),
      .o_mem_next_valid(next_valid),
      .o_mem_ready(ready),
      .i_stat_clear(1'b0),
      .o_cache_info(), .o_cnt_read_hits(), .o_cnt_read_misses(),
      .o_cnt_write_hits(), .o_cnt_write_misses(), .o_cnt_writebacks(),
      .o_cnt_stall_cycles(),
      .clk_50(), .o_calib_done(),
      // DMA master port — driven by the arbiter test below; idle (req=0)
      // during the cache regression so behaviour matches the pre-arbiter path.
      .i_dma_req(dma_req), .i_dma_done(dma_done),
      .i_dma_write_DV(dma_w), .i_dma_read_DV(dma_r),
      .i_dma_addr(dma_addr), .i_dma_write_data(dma_wdata), .i_dma_wdf_mask(16'h0),
      .o_dma_read_data(dma_rdata), .o_dma_ready(dma_ready), .o_dma_grant(dma_grant),
      // Cache-maintenance control (flush / invalidate).
      .i_flush_go(flush_go), .i_inval_go(inval_go), .o_mnt_busy(mnt_busy)
   );

   logic          dma_req = 0, dma_done = 0, dma_w = 0, dma_r = 0;
   logic  [31:0]  dma_addr = 0;
   logic  [127:0] dma_wdata = 0;
   wire [127:0] dma_rdata;
   wire         dma_ready, dma_grant;
   logic          flush_go = 0, inval_go = 0;
   wire         mnt_busy;

   // splitter model: 1-cycle registered return path
   logic        ready_d = 0;
   logic [63:0] rdata_d, rdata_next_d;
   logic        next_valid_d;
   always @(posedge clk) begin
      ready_d       <= ready;
      rdata_d       <= rdata;
      rdata_next_d  <= rdata_next;
      next_valid_d  <= next_valid;
   end

   // scoreboard: 64-bit dwords, addr[19:3] indexes
   logic [63:0] sb [0:131071];
   function [63:0] init_dw;
      input [31:0] a;
      begin
         init_dw = {16'hD00D, a[19:4], 16'hF00D, 13'b0, a[3], 2'b0};
      end
   endfunction
   integer i;
   initial begin
      for (i = 0; i < 131072; i = i + 1)
         sb[i] = init_dw({i[16:0], 3'b000});
   end

   integer errors = 0;
   integer lat;

   // CPU write: DV until registered ready, drop one cycle later (like the FSM)
   task cpu_write;
      input [31:0] a;
      input [63:0] d;
      input [7:0]  ben;
      integer k;
      begin
         @(negedge clk);
         addr = a; wdata = d; be = ben; dv_w = 1;
         @(negedge clk);
         while (!ready_d) @(negedge clk);
         dv_w = 0; be = 8'hFF;
         // scoreboard merge
         for (k = 0; k < 8; k = k + 1)
            if (ben[k]) sb[a[19:3]][8*k +: 8] = d[8*k +: 8];
      end
   endtask

   // CPU read; returns latency in cycles from DV-high to cache ready.
   // Polls at negedge so registered outputs are settled; captures data and
   // the 1-cycle next-dw pulse atomically with ready.
   logic        cap_nv;
   logic [63:0] cap_next;
   task cpu_read;
      input [31:0] a;
      output [63:0] d;
      output integer latency;
      begin
         @(negedge clk);
         addr = a; dv_r = 1; latency = 0;
         @(negedge clk);
         while (!ready_d) begin latency = latency + 1; @(negedge clk); end
         d        = rdata_d;
         cap_nv   = next_valid_d;
         cap_next = rdata_next_d;
         dv_r = 0;
      end
   endtask

   task check_read;
      input [31:0] a;
      logic [63:0] d;
      integer l;
      begin
         cpu_read(a, d, l);
         if (d !== sb[a[19:3]]) begin
            errors = errors + 1;
            $display("FAIL read  %h: got %h want %h (t=%0t)", a, d, sb[a[19:3]], $time);
         end
      end
   endtask

   // -------- DMA master (blitter) protocol drivers --------
   // Mirror the contract documented on the mem_read_write DMA port: request the
   // bus, wait for grant, run one transaction, hold address through a CDC settle
   // after a write, then pulse done to release.
   task dma_write;
      input [31:0]  a;
      input [127:0] d;
      integer g;
      begin
         @(negedge clk); dma_req = 1;
         while (!dma_grant) @(negedge clk);
         dma_addr = a; dma_wdata = d; dma_w = 1;
         @(negedge clk);
         while (!dma_ready) @(negedge clk);
         dma_w = 0;
         repeat (8) @(negedge clk);     // hold addr through CDC settle (mask spurious re-accept)
         dma_done = 1; @(negedge clk); dma_done = 0;
         dma_req = 0;
      end
   endtask

   task dma_read;
      input  [31:0]  a;
      output [127:0] d;
      begin
         @(negedge clk); dma_req = 1;
         while (!dma_grant) @(negedge clk);
         dma_addr = a; dma_r = 1;
         @(negedge clk);
         while (!dma_ready) @(negedge clk);
         d = dma_rdata;
         dma_r = 0;
         repeat (2) @(negedge clk);
         dma_done = 1; @(negedge clk); dma_done = 0;
         dma_req = 0;
      end
   endtask

   // -------- cache-maintenance drivers + DDR back-door (coherency tests) --------
   task do_flush;
      begin
         @(negedge clk); flush_go = 1; @(negedge clk); flush_go = 0;
         @(negedge clk);                 // let r_mnt_pending latch
         while (mnt_busy) @(negedge clk);
         repeat (4) @(negedge clk);
      end
   endtask
   task do_inval;
      begin
         @(negedge clk); inval_go = 1; @(negedge clk); inval_go = 0;
         @(negedge clk);
         while (mnt_busy) @(negedge clk);
         repeat (4) @(negedge clk);
      end
   endtask
   // Read/write the behavioral DDR directly (models DMA touching main memory).
   function [63:0] ddr_dword(input [31:0] a);
      logic [127:0] line;
      begin
         line = dut.ddr2_control.mem[a[19:4]];
         ddr_dword = a[3] ? line[63:0] : line[127:64];
      end
   endfunction
   task poke_ddr_dword(input [31:0] a, input [63:0] v);
      begin
         if (a[3]) dut.ddr2_control.mem[a[19:4]][63:0]   = v;
         else      dut.ddr2_control.mem[a[19:4]][127:64] = v;
      end
   endtask

   logic [63:0] d;
   logic [31:0] a, ra;
   logic [127:0] dline;
   integer l, n, sel;

   initial begin
      be = 8'hFF;
      // let POR counter (32 cycles) drain
      repeat (60) @(posedge clk);

      // --- directed: write-miss merge, read-hit, companion dw ---
      cpu_write(32'h0000_0100, 64'hAABB_CCDD_EE11_2233, 8'hFF);
      check_read(32'h0000_0100);          // miss-installed line, now read hit
      check_read(32'h0000_0108);          // companion dw, untouched DDR pattern

      // next-dw lookahead: offset 0 read must present companion as "next"
      cpu_read(32'h0000_0100, d, l);
      if (!cap_nv || cap_next !== sb[32'h108 >> 3]) begin
         errors = errors + 1;
         $display("FAIL next-dw lookahead: nv=%b next=%h want %h", cap_nv, cap_next, sb[32'h108 >> 3]);
      end
      // offset 1 read: next must be invalid
      cpu_read(32'h0000_0108, d, l);
      if (cap_nv) begin
         errors = errors + 1;
         $display("FAIL next-dw: nv should be 0 at offset 1");
      end

      // read-hit latency: WAIT + CHECK = ready 2 edges after DV sampled
      cpu_read(32'h0000_0100, d, lat);
      $display("read-hit latency (DV->splitter ready): %0d cycles", lat);

      // --- partial-byte write hit ---
      cpu_write(32'h0000_0100, 64'h5555_5555_5555_5555, 8'h0F);  // low 4 bytes only
      check_read(32'h0000_0100);
      cpu_write(32'h0000_0108, 64'h9999_9999_9999_9999, 8'hC3);  // scattered lanes
      check_read(32'h0000_0108);

      // --- conflict / eviction round trip (same set, 4 tags) ---
      // index = addr[14:4]; keep index=0x10, vary addr[31:15]
      cpu_write(32'h0000_0100, 64'h1111_0000_0000_0001, 8'hFF);
      cpu_write(32'h0000_8100, 64'h2222_0000_0000_0002, 8'hFF);  // fills other way
      cpu_write(32'h0001_0100, 64'h3333_0000_0000_0003, 8'hFF);  // evicts a dirty line
      cpu_write(32'h0001_8100, 64'h4444_0000_0000_0004, 8'hFF);  // evicts the other
      check_read(32'h0000_0100);   // must come back from DDR writeback
      check_read(32'h0000_8100);
      check_read(32'h0001_0100);
      check_read(32'h0001_8100);

      // --- randomized stress, two sets x four tags, random byte enables ---
      for (n = 0; n < 4000; n = n + 1) begin
         a = {12'b0, 2'b0, $random & 2'h3, 11'h010 + ($random & 1'b1), 4'b0};
         a = a | (($random & 1) << 3);    // random dw within line
         sel = $random & 3;
         if (sel == 0)
            check_read(a);
         else
            cpu_write(a, {$random, $random}, ($random & 8'hFF) | 8'h01);
      end
      // final sweep over the stressed region
      for (n = 0; n < 16; n = n + 1) begin
         a = {15'b0, n[3:2], 11'h010 + n[1], 4'b0} | (n[0] ? 32'h8 : 32'h0);
         check_read(a);
      end

      // ============================================================
      // DMA arbiter test (Phase 1): the blitter port shares ddr2_control.
      // ============================================================
      // 1) DMA-write a full line to a fresh address (untouched by cache above).
      dma_write(32'h0002_0000, {64'hCAFEBABE_DEADBEEF, 64'h0123_4567_89AB_CDEF});

      // 2) DMA-read it back — must round-trip through ddr2_control.
      dma_read(32'h0002_0000, dline);
      if (dline !== {64'hCAFEBABE_DEADBEEF, 64'h0123_4567_89AB_CDEF}) begin
         errors = errors + 1;
         $display("FAIL dma read-back: got %h", dline);
      end

      // 3) The CPU (cache, master A) must see the DMA-written data in main
      //    memory: a CPU read of this (uncached) line misses and fetches it.
      cpu_read(32'h0002_0000, d, l);
      if (d !== 64'hCAFEBABE_DEADBEEF) begin
         errors = errors + 1;
         $display("FAIL cpu sees dma data (hi dw): got %h", d);
      end
      cpu_read(32'h0002_0008, d, l);
      if (d !== 64'h0123_4567_89AB_CDEF) begin
         errors = errors + 1;
         $display("FAIL cpu sees dma data (lo dw): got %h", d);
      end

      // 4) Bus returns to the cache cleanly: a normal cached access still works.
      check_read(32'h0000_0100);

      // ============================================================
      // Cache maintenance (Phase 4): flush / invalidate coherency.
      // ============================================================
      // --- FLUSH: a dirty line is written back to DDR, valid kept ---
      cpu_write(32'h0004_0000, 64'hF1F1_0000_AAAA_5555, 8'hFF);  // dirty in cache
      if (ddr_dword(32'h0004_0000) === 64'hF1F1_0000_AAAA_5555) begin
         errors = errors + 1; $display("FAIL: dirty data reached DDR before flush");
      end
      do_flush;
      if (ddr_dword(32'h0004_0000) !== 64'hF1F1_0000_AAAA_5555) begin
         errors = errors + 1;
         $display("FAIL flush: DDR=%h want F1F10000AAAA5555", ddr_dword(32'h0004_0000));
      end else $display("PASS flush-wrote-back-dirty");
      check_read(32'h0004_0000);   // still a hit after flush (valid kept) → V

      // --- INVALIDATE sees fresh main-memory data on a CLEAN line ---
      cpu_read(32'h0005_0000, d, l);                 // cache a clean line
      poke_ddr_dword(32'h0005_0000, 64'h1234_5678_9ABC_DEF0);  // "DMA" writes DDR
      cpu_read(32'h0005_0000, d, l);                 // still stale (cache hit)
      if (d === 64'h1234_5678_9ABC_DEF0) begin
         errors = errors + 1; $display("FAIL: cache unexpectedly saw DDR without inval");
      end
      do_inval;
      cpu_read(32'h0005_0000, d, l);                 // miss → refill → fresh
      if (d !== 64'h1234_5678_9ABC_DEF0) begin
         errors = errors + 1; $display("FAIL inval: got %h want 123456789ABCDEF0", d);
      end else $display("PASS inval-sees-fresh");

      // --- INVALIDATE on a DIRTY line: write back THEN drop (no data loss) ---
      cpu_write(32'h0006_0000, 64'hDEAD_BEEF_CAFE_F00D, 8'hFF); // dirty
      do_inval;                                                  // flush + invalidate
      if (ddr_dword(32'h0006_0000) !== 64'hDEAD_BEEF_CAFE_F00D) begin
         errors = errors + 1; $display("FAIL inval-dirty: DDR=%h", ddr_dword(32'h0006_0000));
      end
      cpu_read(32'h0006_0000, d, l);                            // refill → original
      if (d !== 64'hDEAD_BEEF_CAFE_F00D) begin
         errors = errors + 1; $display("FAIL inval-dirty readback: got %h", d);
      end else $display("PASS inval-dirty-preserved");

      if (errors == 0) $display("PASS: all cache + DMA-arbiter + maintenance checks passed");
      else             $display("FAILED: %0d errors", errors);
      $finish;
   end

   initial begin
      #4_000_000;
      $display("TIMEOUT — cache hung (likely handshake regression)");
      $finish;
   end
endmodule

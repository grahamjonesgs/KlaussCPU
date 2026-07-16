// tb_cache — unit bench for mem_read_write + the REAL ddr2_control (M10a).
//
// Unlike tb_soc/tb_blitter (which shadow ddr2_control with a one-shot-ready
// behavioral model), this bench compiles the real controller on top of a
// beat-level fake MIG UI (mig_7series_0 below): BL8 bursts return as two
// back-to-back 64-bit beats after a fixed command latency, pipelined bursts
// chain with a ~2-cycle gap. That gives true sim coverage of the M10a
// critical-word-first machinery:
//   * CWF command reorder (burst_hi first for dw2/dw3) + the r_hi_first
//     beat->dword remap (checked by hit-reading whole lines after misses),
//   * the early dword channel (o_mem_rd_dw/_next/_ok/dw_ready) and the
//     cache's early-restart arms (checked via dut.r_miss_served),
//   * fetch-first dirty misses with the victim writeback in the shadow
//     (checked by eviction round-trips + a final flush + full DDR image
//     compare against the scoreboard),
//   * ready single-pulse discipline and the blitter-grant lockout across
//     miss/shadow states (checked by always-on monitors).
//
// Drives the CPU side the way bus_splitter + the CPU FSM do: ready/data come
// back through a 1-cycle register and DV drops one cycle after the registered
// ready — so DV is still high during PRE_WAIT/COOL_DOWN (the re-latch hazard
// those states absorb).
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
// Fake MIG 2:1 UI — beat-level model behind the REAL ddr2_control.
//
// Memory is 64-bit dwords keyed by app_addr>>2 (one BL8 burst = keys k, k+1;
// beat0 = mem[k], beat1 = mem[k+1]). Reads return 2 back-to-back beats
// RD_LAT cycles after command accept; a pipelined second burst's beats chain
// ~2 cycles after the first burst's. Writes commit when a command and both
// wdf beats have arrived (mask bit = 1 masks the byte, MIG semantics).
//
// The deterministic init pattern (init_dw, function of the KEY) must stay in
// sync with tb_cache's copy — the TB scoreboard models this same image.
// ---------------------------------------------------------------------------
module mig_7series_0 (
    inout  [15:0] ddr2_dq,
    inout  [1:0]  ddr2_dqs_n,
    inout  [1:0]  ddr2_dqs_p,
    output [12:0] ddr2_addr,
    output [2:0]  ddr2_ba,
    output        ddr2_ras_n,
    output        ddr2_cas_n,
    output        ddr2_we_n,
    output [0:0]  ddr2_ck_p,
    output [0:0]  ddr2_ck_n,
    output [0:0]  ddr2_cke,
    output [0:0]  ddr2_cs_n,
    output [1:0]  ddr2_dm,
    output [0:0]  ddr2_odt,
    output        init_calib_complete,
    input  [26:0] app_addr,
    input  [2:0]  app_cmd,
    input         app_en,
    input  [63:0] app_wdf_data,
    input         app_wdf_end,
    input         app_wdf_wren,
    output logic [63:0] app_rd_data,
    output logic  app_rd_data_end,
    output logic  app_rd_data_valid,
    output        app_rdy,
    output        app_wdf_rdy,
    input         app_sr_req,
    input         app_ref_req,
    input         app_zq_req,
    output        app_sr_active,
    output        app_ref_ack,
    output        app_zq_ack,
    output        ui_clk,
    output        ui_clk_sync_rst,
    input  [7:0]  app_wdf_mask,
    input         sys_clk_i,
    input         sys_rst
);
   assign ddr2_addr = '0; assign ddr2_ba = '0;
   assign ddr2_ras_n = 1; assign ddr2_cas_n = 1; assign ddr2_we_n = 1;
   assign ddr2_ck_p = '0; assign ddr2_ck_n = '0; assign ddr2_cke = '0;
   assign ddr2_cs_n = '0; assign ddr2_dm = '0; assign ddr2_odt = '0;
   assign app_sr_active = 0; assign app_ref_ack = 0; assign app_zq_ack = 0;

   assign ui_clk = sys_clk_i;
   // sys_rst is the POR resetn (active-low reset): low while POR counts.
   logic [3:0] rstp = '1;
   always @(posedge sys_clk_i) rstp <= {rstp[2:0], ~sys_rst};
   assign ui_clk_sync_rst     = rstp[3] | ~sys_rst;
   assign init_calib_complete = ~ui_clk_sync_rst;
   assign app_rdy     = 1'b1;
   assign app_wdf_rdy = 1'b1;

   // 8 MiB modeled: 1M 64-bit dwords, key = app_addr[21:2].
   logic [63:0] mem [0:1048575];
   function automatic logic [63:0] init_dw(input logic [19:0] k);
      init_dw = {12'hD0D, k, 12'hF0F, ~k} ^ 64'h5A5A_5A5A_5A5A_5A5A;
   endfunction
   initial begin
      for (int i = 0; i < 1048576; i++) mem[i] = init_dw(20'(i));
   end

   localparam RD_LAT = 10;      // command accept -> burst beat0
   int cyc = 0;
   always @(posedge sys_clk_i) cyc <= cyc + 1;

   // Command / write-data capture.
   logic [26:0] rq_a [$];   int rq_t [$];
   logic [26:0] wq_a [$];
   logic [63:0] wq_d [$];   logic [7:0] wq_m [$];

   always @(posedge sys_clk_i) begin
      if (!ui_clk_sync_rst) begin
         if (app_en && app_rdy && app_cmd == 3'b001) begin
            rq_a.push_back(app_addr);  rq_t.push_back(cyc);
         end
         if (app_en && app_rdy && app_cmd == 3'b000)
            wq_a.push_back(app_addr);
         if (app_wdf_wren && app_wdf_rdy) begin
            wq_d.push_back(app_wdf_data);  wq_m.push_back(app_wdf_mask);
         end
         // Commit a write burst once its command and both beats are in.
         if (wq_a.size() > 0 && wq_d.size() >= 2) begin
            automatic logic [26:0] wa;
            automatic logic [63:0] d0, d1;
            automatic logic [7:0]  m0, m1;
            wa = wq_a.pop_front();
            d0 = wq_d.pop_front();  m0 = wq_m.pop_front();
            d1 = wq_d.pop_front();  m1 = wq_m.pop_front();
            for (int b = 0; b < 8; b++) begin
               if (!m0[b]) mem[20'(wa >> 2)    ][8*b +: 8] = d0[8*b +: 8];
               if (!m1[b]) mem[20'(wa >> 2) + 1][8*b +: 8] = d1[8*b +: 8];
            end
         end
      end
   end

   // Read return engine: 2 beats per burst; a queued second burst chains with
   // a ~2-cycle gap (the pipelined dual-BL8 shape the real MIG shows).
   initial begin
      app_rd_data = '0; app_rd_data_valid = 0; app_rd_data_end = 0;
      forever begin
         @(posedge sys_clk_i);
         if (!ui_clk_sync_rst && rq_a.size() > 0) begin
            automatic logic [26:0] ra;
            automatic int t0;
            ra = rq_a[0];  t0 = rq_t[0];
            while (cyc < t0 + RD_LAT) @(posedge sys_clk_i);
            void'(rq_a.pop_front());  void'(rq_t.pop_front());
            app_rd_data       <= mem[20'(ra >> 2)];
            app_rd_data_valid <= 1;  app_rd_data_end <= 0;
            @(posedge sys_clk_i);
            app_rd_data       <= mem[20'(ra >> 2) + 1];
            app_rd_data_end   <= 1;
            @(posedge sys_clk_i);
            app_rd_data_valid <= 0;  app_rd_data_end <= 0;
         end
      end
   end
endmodule

// ---------------------------------------------------------------------------
// Testbench proper
// ---------------------------------------------------------------------------
module tb_cache;

   logic clk = 0;
   always #5 clk = ~clk;   // 100 MHz

   membus_if mb();

   logic          dma_req = 0, dma_done = 0, dma_w = 0, dma_r = 0;
   logic  [31:0]  dma_addr = 0;
   logic  [127:0] dma_wdata = 0;
   wire  [127:0]  dma_rdata;
   wire           dma_ready, dma_grant;
   logic          flush_go = 0, inval_go = 0;
   wire           mnt_busy;

   mem_read_write dut (
      .i_Clk_board(clk),
      .ddr2_dq(), .ddr2_dqs_n(), .ddr2_dqs_p(),
      .ddr2_addr(), .ddr2_ba(), .ddr2_ras_n(), .ddr2_cas_n(), .ddr2_we_n(),
      .ddr2_ck_p(), .ddr2_ck_n(), .ddr2_cke(), .ddr2_cs_n(), .ddr2_dm(), .ddr2_odt(),
      .cpu(mb),
      .i_stat_clear(1'b0),
      .o_cache_info(), .o_cnt_read_hits(), .o_cnt_read_misses(),
      .o_cnt_write_hits(), .o_cnt_write_misses(), .o_cnt_writebacks(),
      .o_cnt_stall_cycles(),
      .clk_50(), .o_ui_clk(), .o_calib_done(),
      .i_dma_req(dma_req), .i_dma_done(dma_done),
      .i_dma_write_DV(dma_w), .i_dma_read_DV(dma_r),
      .i_dma_addr(dma_addr), .i_dma_write_data(dma_wdata), .i_dma_wdf_mask(16'h0),
      .o_dma_read_data(dma_rdata), .o_dma_ready(dma_ready), .o_dma_grant(dma_grant),
      .i_flush_go(flush_go), .i_inval_go(inval_go), .o_mnt_busy(mnt_busy)
   );

   // splitter model: 1-cycle registered return path
   logic        ready_d = 0;
   logic [63:0] rdata_d, rdata_next_d;
   logic        next_valid_d;
   always @(posedge clk) begin
      ready_d      <= mb.ready;
      rdata_d      <= mb.read_data;
      rdata_next_d <= mb.read_data_next;
      next_valid_d <= mb.next_valid;
   end

   // -------------------------------------------------------------------------
   // Scoreboard — models the DDR image as the fake MIG holds it, keyed by the
   // MIG dword key. The cache's dword<->beat convention (board-verified):
   // within each 16 B burst the CPU's two dwords land in SWAPPED beat order,
   // so cpu byte addr a maps to key {a[22:5], a[4], ~a[3]}. The blitter's
   // narrow port maps straight: 16 B-aligned a -> keys a[22:3], a[22:3]+1
   // with d[63:0] in the first (this is the DMA-vs-cache half-swap contract).
   // -------------------------------------------------------------------------
   function automatic logic [19:0] k_cpu(input logic [31:0] a);
      k_cpu = {a[22:5], a[4], ~a[3]};
   endfunction
   function automatic logic [63:0] init_dw(input logic [19:0] k);
      init_dw = {12'hD0D, k, 12'hF0F, ~k} ^ 64'h5A5A_5A5A_5A5A_5A5A;
   endfunction

   logic [63:0] sb [logic [19:0]];       // sparse: only touched keys
   function automatic logic [63:0] sb_rd_key(input logic [19:0] k);
      sb_rd_key = sb.exists(k) ? sb[k] : init_dw(k);
   endfunction
   function automatic logic [63:0] sb_rd(input logic [31:0] a);
      sb_rd = sb_rd_key(k_cpu(a));
   endfunction

   integer errors = 0;

   // -------------------------------------------------------------------------
   // Always-on monitors
   // -------------------------------------------------------------------------
   // 1) cpu.ready is a 1-cycle pulse, exactly one per transaction (early
   //    restart must never double-serve: no second pulse at install time).
   logic ready_q = 0, dv_q = 0;
   integer pulses_this_txn = 0;
   always @(posedge clk) begin
      if ((mb.write_DV || mb.read_DV) && !dv_q) pulses_this_txn = 0;
      dv_q    <= (mb.write_DV || mb.read_DV);
      if (mb.ready && ready_q) begin
         errors++; $display("FAIL monitor: cpu.ready held >1 cycle (t=%0t)", $time);
      end
      if (mb.ready && !ready_q) begin
         pulses_this_txn = pulses_this_txn + 1;
         if (pulses_this_txn > 1) begin
            errors++; $display("FAIL monitor: multiple ready pulses in one transaction (t=%0t)", $time);
         end
      end
      ready_q <= mb.ready;
   end

   // 2) The blitter grant must never RISE while the cache is in a miss or
   //    shadow-writeback state (or during a maintenance walk) — the victim
   //    line's DDR copy is stale until the shadow write completes.
   logic grant_q = 0, imp_q = 0, mnt_q = 0;
   always @(posedge clk) begin
      if (dma_grant && !grant_q && (imp_q || mnt_q)) begin
         errors++; $display("FAIL monitor: DMA grant rose during miss/shadow/maintenance (t=%0t)", $time);
      end
      grant_q <= dma_grant;
      imp_q   <= dut.is_miss_path;
      mnt_q   <= dut.r_mnt_active;
   end

   // -------------------------------------------------------------------------
   // CPU-side drivers (bus_splitter-faithful timing)
   // -------------------------------------------------------------------------
   task cpu_write(input [31:0] a, input [63:0] d, input [7:0] ben);
      integer k;
      logic [63:0] cur;
      begin
         @(negedge clk);
         mb.addr = a; mb.write_data = d; mb.byte_en = ben; mb.write_DV = 1;
         @(negedge clk);
         while (!ready_d) @(negedge clk);
         mb.write_DV = 0; mb.byte_en = 8'hFF;
         cur = sb_rd(a);
         for (k = 0; k < 8; k = k + 1)
            if (ben[k]) cur[8*k +: 8] = d[8*k +: 8];
         sb[k_cpu(a)] = cur;
      end
   endtask

   // Read; returns data + latency (cycles from DV-high to registered ready).
   // Captures the next-dw lookahead pulse atomically with ready and applies
   // the universal invariant: next_valid=1 implies read_data_next matches the
   // scoreboard's companion dword (and the offset cannot be dw3).
   logic        cap_nv;
   logic [63:0] cap_next;
   task cpu_read(input [31:0] a, output [63:0] d, output integer latency);
      begin
         @(negedge clk);
         mb.addr = a; mb.read_DV = 1; latency = 0;
         @(negedge clk);
         while (!ready_d) begin latency = latency + 1; @(negedge clk); end
         d        = rdata_d;
         cap_nv   = next_valid_d;
         cap_next = rdata_next_d;
         mb.read_DV = 0;
         if (cap_nv) begin
            if (a[4:3] == 2'b11) begin
               errors++; $display("FAIL: next_valid at dw3 %h (t=%0t)", a, $time);
            end else if (cap_next !== sb_rd(a + 8)) begin
               errors++; $display("FAIL next-dw %h: got %h want %h (t=%0t)",
                                  a, cap_next, sb_rd(a + 8), $time);
            end
         end
      end
   endtask

   task check_read(input [31:0] a);
      logic [63:0] d;
      integer l;
      begin
         cpu_read(a, d, l);
         if (d !== sb_rd(a)) begin
            errors++; $display("FAIL read  %h: got %h want %h (t=%0t)", a, d, sb_rd(a), $time);
         end
      end
   endtask

   // -------------------------------------------------------------------------
   // DMA (blitter) drivers — same contract as the DMA port comment in the DUT.
   // -------------------------------------------------------------------------
   task dma_write(input [31:0] a, input [127:0] d);
      begin
         @(negedge clk); dma_req = 1;
         while (!dma_grant) @(negedge clk);
         dma_addr = a; dma_wdata = d; dma_w = 1;
         @(negedge clk);
         while (!dma_ready) @(negedge clk);
         dma_w = 0;
         repeat (8) @(negedge clk);     // hold addr through the settle gap
         dma_done = 1; @(negedge clk); dma_done = 0;
         dma_req = 0;
         sb[{a[22:3]}]        = d[63:0];
         sb[{a[22:3]} + 20'd1] = d[127:64];
      end
   endtask

   task dma_read(input [31:0] a, output [127:0] d);
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

   // -------------------------------------------------------------------------
   // Maintenance drivers + DDR back-door (into the fake MIG image)
   // -------------------------------------------------------------------------
   task do_flush;
      begin
         @(negedge clk); flush_go = 1; @(negedge clk); flush_go = 0;
         @(negedge clk);
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
   function automatic logic [63:0] ddr_dword(input logic [31:0] a);
      ddr_dword = dut.ddr2_control.mig_7series_0.mem[k_cpu(a)];
   endfunction
   task poke_ddr_dword(input logic [31:0] a, input logic [63:0] v);
      begin
         dut.ddr2_control.mig_7series_0.mem[k_cpu(a)] = v;
         sb[k_cpu(a)] = v;
      end
   endtask

   // Directed helper: read that MUST be a miss served by the early-restart
   // path — checks data, the early flag, and miss next_valid semantics
   // (companion present exactly on even dword offsets).
   task check_miss_early(input [31:0] a);
      logic [63:0] d;
      integer l;
      begin
         cpu_read(a, d, l);
         if (d !== sb_rd(a)) begin
            errors++; $display("FAIL miss read %h: got %h want %h", a, d, sb_rd(a));
         end
         if (dut.r_miss_served !== 1'b1) begin
            errors++; $display("FAIL: early restart did not fire on miss %h (fallback path used)", a);
         end
         if (cap_nv !== ~a[3]) begin
            errors++; $display("FAIL miss nv %h: got %b want %b (CWF companion rule)", a, cap_nv, ~a[3]);
         end
         $display("  miss @off%0d: lat=%0d nv=%b", a[4:3], l, cap_nv);
      end
   endtask

   // -------------------------------------------------------------------------
   // Stimulus
   // -------------------------------------------------------------------------
   logic [63:0]  d;
   logic [31:0]  a;
   logic [127:0] dline;
   integer l, n, sel, lat;
   logic [19:0] key;

   initial begin
      mb.write_DV = 0; mb.read_DV = 0; mb.addr = '0;
      mb.write_data = '0; mb.byte_en = 8'hFF;
      // POR (32 board cycles) + fake-MIG reset pipe
      repeat (60) @(posedge clk);

      // ===== Phase A: CWF early restart on clean read misses, all offsets ===
      // Distinct line per offset; after each miss, hit-read the WHOLE line —
      // for off 2/3 the commands were issued hi-first, so this validates the
      // r_hi_first beat->dword remap end to end.
      $display("Phase A: CWF clean read misses");
      for (n = 0; n < 4; n = n + 1) begin
         a = 32'h0010_0000 + n[1:0] * 32'h20;
         check_miss_early(a + n[1:0] * 8);
         check_read(a + 0);  check_read(a + 8);
         check_read(a + 16); check_read(a + 24);
         // hit next-dw semantics: nv = (off != 3)
         cpu_read(a + 8, d, l);
         if (!cap_nv) begin errors++; $display("FAIL hit nv at off1 should be 1"); end
         cpu_read(a + 24, d, l);
         if (cap_nv)  begin errors++; $display("FAIL hit nv at off3 should be 0"); end
      end

      // read-hit latency reference
      cpu_read(32'h0010_0000, d, lat);
      $display("read-hit latency (DV -> splitter ready): %0d cycles", lat);

      // ===== Phase B: write-miss early ack + merge correctness ==============
      $display("Phase B: write misses (early ack + merge)");
      cpu_write(32'h0011_0000, 64'hAABB_CCDD_EE11_2233, 8'hFF);
      if (dut.r_miss_served !== 1'b1) begin
         errors++; $display("FAIL: early ack did not fire on write miss");
      end
      check_read(32'h0011_0000);
      check_read(32'h0011_0008);   // neighbours must carry the DDR pattern
      check_read(32'h0011_0010);
      check_read(32'h0011_0018);
      // partial-byte write MISS at an odd offset (merge on the install path)
      cpu_write(32'h0012_0008, 64'h5555_5555_5555_5555, 8'h3C);
      check_read(32'h0012_0008);
      check_read(32'h0012_0000);
      // partial-byte write hits
      cpu_write(32'h0011_0000, 64'h9999_9999_9999_9999, 8'h0F);
      check_read(32'h0011_0000);
      cpu_write(32'h0011_0008, 64'h7777_7777_7777_7777, 8'hC3);
      check_read(32'h0011_0008);

      // ===== Phase C: dirty evictions — shadow writeback round trips ========
      // 4 tags in one set (index = addr[14:5]): each write evicts a dirty
      // line whose writeback now runs in the shadow AFTER the refill; the
      // read-backs prove the victims reached DDR intact.
      $display("Phase C: dirty eviction round trips (shadow writeback)");
      cpu_write(32'h0000_0100, 64'h1111_0000_0000_0001, 8'hFF);
      cpu_write(32'h0000_8100, 64'h2222_0000_0000_0002, 8'hFF);
      cpu_write(32'h0001_0100, 64'h3333_0000_0000_0003, 8'hFF);
      cpu_write(32'h0001_8100, 64'h4444_0000_0000_0004, 8'hFF);
      check_read(32'h0000_0100);
      check_read(32'h0000_8100);
      check_read(32'h0001_0100);
      check_read(32'h0001_8100);

      // ===== Phase D: randomized stress, 2 sets x 4 tags x 4 dwords =========
      $display("Phase D: 4000 random ops");
      for (n = 0; n < 4000; n = n + 1) begin
         a = {15'b0, 2'($urandom), 10'h010, 5'b0}
             | (32'($urandom & 1) << 5)          // 2 sets (index 0x10/0x11)
             | (32'($urandom & 3) << 3);         // random dword in line
         sel = $urandom & 3;
         if (sel == 0)
            check_read(a);
         else
            cpu_write(a, {$urandom, $urandom}, (8'($urandom) | 8'h01));
      end
      for (n = 0; n < 32; n = n + 1) begin
         a = {15'b0, n[4:3], 10'h010, 5'b0} | (32'(n[2]) << 5) | (32'(n[1:0]) << 3);
         check_read(a);
      end

      // ===== Phase E: concurrent CPU + DMA traffic ==========================
      // Blitter round-trips on a disjoint region while the CPU thrashes its
      // two sets — exercises grant handoffs against miss/shadow states (the
      // grant-rise monitor is armed the whole time).
      $display("Phase E: concurrent CPU + DMA");
      fork
         begin
            for (int m = 0; m < 300; m++) begin
               automatic logic [31:0] ca;
               ca = {15'b0, 2'($urandom), 10'h010, 5'b0}
                    | (32'($urandom & 1) << 5) | (32'($urandom & 3) << 3);
               if (($urandom & 3) == 0) check_read(ca);
               else cpu_write(ca, {$urandom, $urandom}, (8'($urandom) | 8'h01));
            end
         end
         begin
            for (int m = 0; m < 20; m++) begin
               automatic logic [31:0]  da;
               automatic logic [127:0] dv, dr;
               da = 32'h0040_0000 + 32'(m) * 16;
               dv = {$urandom, $urandom, $urandom, $urandom};
               dma_write(da, dv);
               dma_read(da, dr);
               if (dr !== dv) begin
                  errors++; $display("FAIL dma round-trip %h: got %h want %h", da, dr, dv);
               end
            end
         end
      join

      // ===== Phase F: DMA <-> cache coherency (directed, from the old TB) ===
      $display("Phase F: DMA arbiter directed");
      dma_write(32'h0042_0000, {64'hCAFEBABE_DEADBEEF, 64'h0123_4567_89AB_CDEF});
      dma_read(32'h0042_0000, dline);
      if (dline !== {64'hCAFEBABE_DEADBEEF, 64'h0123_4567_89AB_CDEF}) begin
         errors++; $display("FAIL dma read-back: got %h", dline);
      end
      // CPU (uncached line) must see the DMA data through a normal miss.
      check_read(32'h0042_0000);
      check_read(32'h0042_0008);
      check_read(32'h0000_0100);   // and the cache still behaves afterwards

      // ===== Phase G: maintenance flush / invalidate ========================
      $display("Phase G: maintenance");
      cpu_write(32'h0004_0000, 64'hF1F1_0000_AAAA_5555, 8'hFF);
      if (ddr_dword(32'h0004_0000) === 64'hF1F1_0000_AAAA_5555) begin
         errors++; $display("FAIL: dirty data reached DDR before flush");
      end
      do_flush;
      if (ddr_dword(32'h0004_0000) !== 64'hF1F1_0000_AAAA_5555) begin
         errors++; $display("FAIL flush: DDR=%h want F1F10000AAAA5555", ddr_dword(32'h0004_0000));
      end else $display("PASS flush-wrote-back-dirty");
      check_read(32'h0004_0000);   // valid kept -> still a hit

      cpu_read(32'h0005_0000, d, l);                 // cache a clean line
      poke_ddr_dword(32'h0005_0000, 64'h1234_5678_9ABC_DEF0);
      cpu_read(32'h0005_0000, d, l);                 // stale hit expected
      if (d === 64'h1234_5678_9ABC_DEF0) begin
         errors++; $display("FAIL: cache saw DDR without inval");
      end
      do_inval;
      cpu_read(32'h0005_0000, d, l);
      if (d !== 64'h1234_5678_9ABC_DEF0) begin
         errors++; $display("FAIL inval: got %h want 123456789ABCDEF0", d);
      end else $display("PASS inval-sees-fresh");

      cpu_write(32'h0006_0000, 64'hDEAD_BEEF_CAFE_F00D, 8'hFF);
      do_inval;
      if (ddr_dword(32'h0006_0000) !== 64'hDEAD_BEEF_CAFE_F00D) begin
         errors++; $display("FAIL inval-dirty: DDR=%h", ddr_dword(32'h0006_0000));
      end
      cpu_read(32'h0006_0000, d, l);
      if (d !== 64'hDEAD_BEEF_CAFE_F00D) begin
         errors++; $display("FAIL inval-dirty readback: got %h", d);
      end else $display("PASS inval-dirty-preserved");

      // ===== Phase H: final image equality ==================================
      // Flush everything, then the fake DDR must equal the scoreboard on every
      // key ever touched — a lost or mis-addressed shadow writeback anywhere
      // in the run fails here.
      $display("Phase H: final flush + full DDR image compare");
      do_flush;
      if (sb.first(key)) begin
         do begin
            if (dut.ddr2_control.mig_7series_0.mem[key] !== sb[key]) begin
               errors++;
               $display("FAIL image key %h: DDR=%h sb=%h", key,
                        dut.ddr2_control.mig_7series_0.mem[key], sb[key]);
            end
         end while (sb.next(key));
      end

      if (errors == 0) $display("TB_CACHE PASS: all CWF/early-restart/shadow/DMA/maintenance checks passed");
      else             $display("TB_CACHE FAILED: %0d errors", errors);
      $finish;
   end

   initial begin
      #40_000_000;
      $display("TB_CACHE TIMEOUT — cache hung (handshake regression)");
      $finish;
   end
endmodule

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

   // read commands accepted (DCBZ checks: a zero-line miss issues none)
   int rd_cmds = 0, wr_cmds = 0;
   always @(posedge sys_clk_i)
      if (!ui_clk_sync_rst && app_en && app_rdy) begin
         if (app_cmd == 3'b001) rd_cmds <= rd_cmds + 1;
         if (app_cmd == 3'b000) wr_cmds <= wr_cmds + 1;
      end

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
   wire [63:0] cnt_zero_lines;

   logic          dma_req = 0, dma_done = 0, dma_w = 0, dma_r = 0;
   logic  [31:0]  dma_addr = 0;
   logic  [127:0] dma_wdata = 0;
   wire  [127:0]  dma_rdata;
   wire           dma_ready, dma_grant;
   logic          flush_go = 0, inval_go = 0;
   wire           mnt_busy;
   // VGA scanout master D (read-only, wide 32 B reads)
   logic          vga_req = 0, vga_done = 0, vga_r = 0;
   logic  [31:0]  vga_addr = 0;
   wire  [255:0]  vga_rdata;
   wire           vga_ready, vga_grant;

   mem_read_write dut (
      .i_Clk_board(clk),
      .ddr2_dq(), .ddr2_dqs_n(), .ddr2_dqs_p(),
      .ddr2_addr(), .ddr2_ba(), .ddr2_ras_n(), .ddr2_cas_n(), .ddr2_we_n(),
      .ddr2_ck_p(), .ddr2_ck_n(), .ddr2_cke(), .ddr2_cs_n(), .ddr2_dm(), .ddr2_odt(),
      .cpu(mb),
      .i_stat_clear(1'b0),
      .o_cache_info(), .o_cnt_read_hits(), .o_cnt_read_misses(),
      .o_cnt_write_hits(), .o_cnt_write_misses(), .o_cnt_writebacks(),
      .o_cnt_stall_cycles(), .o_cnt_zero_lines(cnt_zero_lines),
      .clk_50(), .o_ui_clk(), .o_calib_done(),
      .i_dma_req(dma_req), .i_dma_done(dma_done),
      .i_dma_write_DV(dma_w), .i_dma_read_DV(dma_r),
      .i_dma_addr(dma_addr), .i_dma_write_data(dma_wdata), .i_dma_wdf_mask(16'h0),
      .o_dma_read_data(dma_rdata), .o_dma_ready(dma_ready), .o_dma_grant(dma_grant),
      .i_c2_req(1'b0), .i_c2_done(1'b0), .i_c2_write_DV(1'b0), .i_c2_read_DV(1'b0),
      .i_c2_addr(32'h0), .i_c2_write_data(128'h0), .i_c2_wdf_mask(16'h0),
      .o_c2_read_data(), .o_c2_ready(), .o_c2_grant(),
      .i_vga_req(vga_req), .i_vga_done(vga_done), .i_vga_read_DV(vga_r),
      .i_vga_addr(vga_addr), .o_vga_read_data(vga_rdata), .o_vga_ready(vga_ready),
      .o_vga_grant(vga_grant),
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
   // Phase P streams re-address in the ready cycle itself (the core's
   // combinational return), which this one-cycle-late monitor attributes to
   // the next request: it is muted for a stream (Phase H checks the data).
   bit     p_quiet = 0;

   // -------------------------------------------------------------------------
   // Always-on monitors
   // -------------------------------------------------------------------------
   // 1) cpu.ready is a 1-cycle pulse, exactly one per transaction (early
   //    restart must never double-serve: no second pulse at install time).
   // A transaction begins on a DV rise OR on an address change while DV is
   // held (the ld32 spanning-load shape: DV held, atomically re-addressed).
   logic ready_q = 0, dv_q = 0;
   logic [31:0] dv_addr_q = '0;
   integer pulses_this_txn = 0;
   always @(posedge clk) begin
      if ((mb.write_DV || mb.read_DV) && (!dv_q || mb.addr !== dv_addr_q))
         pulses_this_txn = 0;
      dv_q      <= (mb.write_DV || mb.read_DV);
      dv_addr_q <= mb.addr;
      if (mb.ready && ready_q) begin
         errors++; $display("FAIL monitor: cpu.ready held >1 cycle (t=%0t)", $time);
      end
      if (mb.ready && !ready_q) begin
         pulses_this_txn = pulses_this_txn + 1;
         if (pulses_this_txn > 1 && !p_quiet) begin
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

   // 3) The VGA grant may rise only out of a cycle with no miss in flight and
   //    the maintenance walk (if any) parked at its between-sets point; and no
   //    two grants are ever up together.
   logic vgrant_q = 0, vga_ok_q = 0;
   integer vga_grants_in_walk = 0;
   always @(posedge clk) begin
      if (vga_grant && !vgrant_q) begin
         if (!vga_ok_q) begin
            errors++; $display("FAIL monitor: VGA grant rose during miss / mid-walk writeback (t=%0t)", $time);
         end
         if (dut.state == dut.MAINT) vga_grants_in_walk++;
      end
      if ((vga_grant + dma_grant) > 1) begin
         errors++; $display("FAIL monitor: VGA and blitter grants overlap (t=%0t)", $time);
      end
      vgrant_q <= vga_grant;
      vga_ok_q <= !dut.is_miss_path && (dut.state != dut.MAINT || dut.r_mnt_sub == dut.MS_READ);
   end

   // -------------------------------------------------------------------------
   // CPU-side drivers (bus_splitter-faithful timing)
   // -------------------------------------------------------------------------
   logic zl = 0;      // cpu_write issues a DCBZ (cpu_zero)
   task cpu_write(input [31:0] a, input [63:0] d, input [7:0] ben);
      integer k;
      logic [63:0] cur;
      begin
         @(negedge clk);
         mb.addr = a; mb.write_data = d; mb.byte_en = ben; mb.write_DV = 1; mb.zero_line = zl;
         @(negedge clk);
         while (!ready_d) @(negedge clk);
         mb.write_DV = 0; mb.byte_en = 8'hFF;
         if (zl) begin
            // DCBZ (HW_PERF_PLAN.md §1): the whole 32 B line reads 0
            for (k = 0; k < 4; k = k + 1) sb[k_cpu({a[31:5], 5'b0} + 32'(8 * k))] = 64'h0;
         end else begin
         cur = sb_rd(a);
         for (k = 0; k < 8; k = k + 1)
            if (ben[k]) cur[8*k +: 8] = d[8*k +: 8];
         sb[k_cpu(a)] = cur;
         end
         mb.zero_line = 0;
      end
   endtask
   // DCBZ: a zero-line write request (addr = the line base, as the core sends it)
   task cpu_zero(input [31:0] a);
      begin
         zl = 1;
         cpu_write({a[31:5], 5'b0}, 64'h0, 8'hFF);
         zl = 0;
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

   // Held-DV atomic re-address (the pipeline's ld32_need2 shape): consume the
   // first ready, then change the address WITHOUT dropping read_DV. The
   // served-DV guard must release on the address change — a regression here
   // deadlocks (caught by the timeout).
   task cpu_read_held2(input [31:0] a1, input [31:0] a2);
      logic [63:0] d;
      begin
         @(negedge clk);
         mb.addr = a1; mb.read_DV = 1;
         @(negedge clk);
         while (!ready_d) @(negedge clk);
         d = rdata_d;
         if (d !== sb_rd(a1)) begin
            errors++; $display("FAIL held-DV first %h: got %h want %h", a1, d, sb_rd(a1));
         end
         mb.addr = a2;         // DV stays high — atomic re-address
         @(negedge clk);
         while (!ready_d) @(negedge clk);
         d = rdata_d;
         if (d !== sb_rd(a2)) begin
            errors++; $display("FAIL held-DV second %h: got %h want %h", a2, d, sb_rd(a2));
         end
         mb.read_DV = 0;
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
   // VGA scanout driver: one tenure of n consecutive wide 32 B reads from a,
   // compared against the scoreboard (the cache-line layout: dword at byte
   // offset 8k in bits [255-64k -: 64]).  Same shape vga_scanout.sv uses:
   // DV low for a cycle between reads, done 2 cycles after the last ready.
   // -------------------------------------------------------------------------
   integer vga_reads = 0;
   logic   v2_cpu_done = 0;
   task vga_burst(input [31:0] a, input integer n);
      logic [255:0] line;
      begin
         @(negedge clk); vga_req = 1;
         while (!vga_grant) @(negedge clk);
         for (int i = 0; i < n; i++) begin
            vga_addr = a + 32'(i) * 32; vga_r = 1;
            @(negedge clk);
            while (!vga_ready) @(negedge clk);
            line  = vga_rdata;
            vga_r = 0;
            @(negedge clk);
            for (int k = 0; k < 4; k++)
               if (line[255 - 64*k -: 64] !== sb_rd(vga_addr + 32'(8*k))) begin
                  errors++;
                  $display("FAIL vga read %h dw%0d: got %h want %h", vga_addr, k,
                           line[255 - 64*k -: 64], sb_rd(vga_addr + 32'(8*k)));
               end
            vga_reads++;
         end
         @(negedge clk);
         vga_done = 1; vga_req = 0; @(negedge clk); vga_done = 0;
      end
   endtask

   // -------------------------------------------------------------------------
   // Phase P throughput driver: a back-to-back stream of n line writes from
   // `base`, as fast as the cache accepts them — the core's shape with the
   // combinational DRAM return (bus_splitter COMB_DRAM_RET): the next
   // request is on the bus in the cycle its predecessor's ready shows (DV
   // held, re-addressed).  dz = 1: one DCBZ per line; 0: four 64-bit stores
   // per line.  Counts the cycles spent in each FSM state from the first
   // request to the last ready (p_hist, p_cyc).
   // -------------------------------------------------------------------------
   bit     p_on = 0;
   longint p_t0 = 0;
   bit     p_dz = 0, p_vdirty = 0;
   int     p_cyc = 0;
   int     p_hist [int];
   always @(posedge clk) if (p_on) begin
      p_cyc++;
      p_hist[int'(dut.state)] = (p_hist.exists(int'(dut.state)) ? p_hist[int'(dut.state)] : 0) + 1;
   end
   task automatic stream_lines(input [31:0] base, input integer n, input bit dz);
      logic [31:0] a;
      logic [63:0] d;
      begin
         p_hist.delete(); p_cyc = 0;
         @(negedge clk);
         p_on = 1; p_quiet = 1;
         for (int i = 0; i < (dz ? n : 4 * n); i++) begin
            a = dz ? base + 32'(32 * i) : base + 32'(32 * (i / 4) + 8 * (i % 4));
            d = dz ? 64'h0 : {32'hC0DE_0000 | 32'(i), a};
            mb.addr = a; mb.write_data = d; mb.byte_en = 8'hFF;
            mb.zero_line = dz; mb.write_DV = 1;
            if (dz) for (int k = 0; k < 4; k++) sb[k_cpu(a + 32'(8 * k))] = 64'h0;
            else    sb[k_cpu(a)] = d;
            @(negedge clk);
            while (!mb.ready) @(negedge clk);
         end
         p_on = 0;
         mb.write_DV = 0; mb.zero_line = 0;
         while (dut.state != dut.WAIT) @(negedge clk);
         repeat (4) @(negedge clk);
         p_quiet = 0;
      end
   endtask
   task p_report(input string name, input integer n);
      string s;
      begin
         s = "";
         foreach (p_hist[k]) begin
            string nm;
            case (k)
               2:     nm = "WAIT";        4:    nm = "CHECK";
               16:    nm = "EV_GAP";      32:   nm = "EV_WAIT";
               128:   nm = "WR_FETCH";    4096: nm = "RD_WAIT";
               16384: nm = "MAINT";       default: nm = $sformatf("%0d", k);
            endcase
            s = {s, $sformatf(" %s=%0d.%02d", nm, p_hist[k] / n, (p_hist[k] * 100 / n) % 100)};
         end
         $display("  P %-22s %0d lines %0d cyc = %0d.%02d cyc/line; per line:%s", name, n, p_cyc,
                  p_cyc / n, (p_cyc * 100 / n) % 100, s);
      end
   endtask

   // the same stream after an invalidate (p_vdirty: and after dirtying
   // every line), against scanout-shaped VGA traffic: a 4-read tenure every
   // 320 cycles (640 x 16 bpp: 40 reads per 3200-cycle display line)
   task p_vga_stream;
      begin
         do_inval;
         if (p_vdirty) stream_lines(32'h0054_0000, 2048, 0);   // every victim dirty
         v2_cpu_done = 0;
         fork
            begin
               stream_lines(p_dz ? 32'h0050_0000 : 32'h0052_0000, 2048, p_dz);
               v2_cpu_done = 1;
            end
            begin
               while (!v2_cpu_done) begin
                  p_t0 = $time;
                  vga_burst(32'h0061_0000 + 32'(vga_reads % 5) * 128, 4);
                  while ($time - p_t0 < 3200 && !v2_cpu_done) @(negedge clk);
               end
            end
         join
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
      mb.write_data = '0; mb.byte_en = 8'hFF; mb.zero_line = 0;
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

      // ===== Phase B2 (M10b): write-ack-at-latch + served-DV guard ==========
      $display("Phase B2: M10b ack/guard directed");
      // held-DV atomic re-address: hit->hit (same line) and hit->MISS
      // (dw3 -> next-line dw0, the real ld32 line-cross shape)
      check_read(32'h0013_0000);                      // warm the line
      cpu_read_held2(32'h0013_0000, 32'h0013_0008);   // hit -> hit
      cpu_read_held2(32'h0013_0018, 32'h0013_0020);   // dw3 hit -> next-line MISS
      $display("PASS held-DV re-address");
      // store->load same address back-to-back (RAW through the early ack),
      // including a store MISS immediately consumed by a load
      cpu_write(32'h0014_0000, 64'h0123_4567_89AB_CDEF, 8'hFF);  // write MISS
      check_read(32'h0014_0000);                                 // parked load
      cpu_write(32'h0014_0000, 64'hFFFF_0000_FFFF_0000, 8'h55);  // partial hit
      check_read(32'h0014_0000);
      check_read(32'h0014_0008);                                 // neighbour intact

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

      // ===== Phase Z: DCBZ (HW_PERF_PLAN.md §1) — zero a line, no fetch ====
      // One set (index 0x200), tags T1..T4.  Every case: no DDR read command
      // is issued for the DCBZ, all four dwords then read 0, CNT_ZERO_LINES
      // counts it; a dirty victim reaches DDR through the shadow writeback.
      $display("Phase Z: DCBZ");
      begin : phase_z
         logic [31:0] t1, t2, t3, t4;
         int rd0, wr0;
         logic [63:0] z0;
         t1 = 32'h0030_4000; t2 = 32'h0031_4000; t3 = 32'h0032_4000; t4 = 32'h0033_4000;
         z0 = cnt_zero_lines;
         // Z1: hit on a clean line
         check_read(t1 + 8);
         rd0 = dut.ddr2_control.mig_7series_0.rd_cmds;
         cpu_zero(t1);
         for (int k = 0; k < 4; k++) check_read(t1 + 32'(8*k));
         if (dut.ddr2_control.mig_7series_0.rd_cmds != rd0) begin
            errors++; $display("FAIL Z1: DCBZ hit issued a DDR read");
         end
         // Z2: hit on a dirty line (back-to-back store / DCBZ / load)
         cpu_write(t1 + 16, 64'h0BAD_0BAD_0BAD_0BAD, 8'hFF);
         cpu_zero(t1);
         check_read(t1 + 16); check_read(t1); check_read(t1 + 24);
         // the zeroed line is dirty: DDR still holds the old pattern
         if (ddr_dword(t1 + 16) === 64'h0) begin
            errors++; $display("FAIL Z2: DCBZ hit wrote through to DDR");
         end
         // Z3: miss, clean victim (set = {t1 dirty-zero, t2 clean}; t1 MRU after a read)
         check_read(t2);
         check_read(t1 + 8);                     // t1 MRU: t2 is the victim
         rd0 = dut.ddr2_control.mig_7series_0.rd_cmds;
         wr0 = dut.ddr2_control.mig_7series_0.wr_cmds;
         cpu_zero(t3);
         repeat (40) @(negedge clk);
         if (dut.ddr2_control.mig_7series_0.rd_cmds != rd0 ||
             dut.ddr2_control.mig_7series_0.wr_cmds != wr0) begin
            errors++; $display("FAIL Z3: DCBZ miss (clean victim) touched DDR: rd %0d wr %0d",
                               dut.ddr2_control.mig_7series_0.rd_cmds - rd0,
                               dut.ddr2_control.mig_7series_0.wr_cmds - wr0);
         end
         rd0 = dut.ddr2_control.mig_7series_0.rd_cmds;
         for (int k = 0; k < 4; k++) check_read(t3 + 32'(8*k));
         check_read(t1 + 24);                    // still resident (hit)
         if (dut.ddr2_control.mig_7series_0.rd_cmds != rd0) begin
            errors++; $display("FAIL Z3: the zeroed line or its neighbour missed afterwards");
         end
         // Z4: miss, dirty victim: t1 (dirty zeros) is LRU after the t3 reads
         rd0 = dut.ddr2_control.mig_7series_0.rd_cmds;
         wr0 = dut.ddr2_control.mig_7series_0.wr_cmds;
         cpu_write(t3 + 8, 64'h1357_9BDF_2468_ACE0, 8'hFF);   // t3 dirty, MRU
         cpu_zero(t4);                           // evicts t1 (dirty)
         check_read(t4 + 24);                    // a load right behind it
         repeat (60) @(negedge clk);
         if (dut.ddr2_control.mig_7series_0.rd_cmds != rd0) begin
            errors++; $display("FAIL Z4: DCBZ miss issued a DDR read");
         end
         if (dut.ddr2_control.mig_7series_0.wr_cmds != wr0 + 2) begin   // two BL8 bursts
            errors++; $display("FAIL Z4: victim writeback: %0d write bursts",
                               dut.ddr2_control.mig_7series_0.wr_cmds - wr0);
         end
         for (int k = 0; k < 4; k++)
            if (ddr_dword(t1 + 32'(8*k)) !== 64'h0) begin
               errors++; $display("FAIL Z4: victim dw%0d not in DDR: %h", k, ddr_dword(t1 + 32'(8*k)));
            end
         // Z5: miss evicting the dirty data line t3 (t4 MRU): DDR gets it
         cpu_zero(t2);
         repeat (60) @(negedge clk);
         if (ddr_dword(t3 + 8) !== 64'h1357_9BDF_2468_ACE0) begin
            errors++; $display("FAIL Z5: victim t3 not written back: %h", ddr_dword(t3 + 8));
         end
         for (int k = 0; k < 4; k++) begin
            check_read(t3 + 32'(8*k));           // refetched from DDR
            check_read(t2 + 32'(8*k));
            check_read(t1 + 32'(8*k));
         end
         if (cnt_zero_lines - z0 != 64'd5) begin
            errors++; $display("FAIL Z: CNT_ZERO_LINES +%0d, want 5", cnt_zero_lines - z0);
         end
         if (errors == 0) $display("PASS DCBZ directed");
      end

      // ===== Phase D: randomized stress, 2 sets x 4 tags x 4 dwords =========
      $display("Phase D: 4000 random ops");
      for (n = 0; n < 4000; n = n + 1) begin
         a = {15'b0, 2'($urandom), 10'h010, 5'b0}
             | (32'($urandom & 1) << 5)          // 2 sets (index 0x10/0x11)
             | (32'($urandom & 3) << 3);         // random dword in line
         sel = $urandom & 15;
         if (sel < 4)
            check_read(a);
         else if (sel == 15)
            cpu_zero(a);                    // DCBZ (HW_PERF_PLAN §1)
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
               sel = $urandom & 7;
               if (sel < 2)       check_read(ca);
               else if (sel == 7) cpu_zero(ca);    // DCBZ, often a dirty victim, under DMA grants
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

      // Mid-walk re-arm: an INVALIDATE issued while a FLUSH walk is still
      // running must not be dropped — a fresh walk must follow the current
      // one. (Real shape: blit driver FLUSHes, the blit is short, and its
      // post-blit INVALIDATE lands while the flush walk is still going.)
      cpu_read(32'h0007_0000, d, l);                 // cache a clean line
      poke_ddr_dword(32'h0007_0000, 64'hA5A5_0123_4567_89AB);
      @(negedge clk); flush_go = 1; @(negedge clk); flush_go = 0;
      wait (dut.r_mnt_active === 1'b1);              // FLUSH walk running
      repeat (12) @(negedge clk);                    // well inside the walk
      @(negedge clk); inval_go = 1; @(negedge clk); inval_go = 0;
      @(negedge clk);
      while (mnt_busy) @(negedge clk);               // both walks complete
      repeat (4) @(negedge clk);
      cpu_read(32'h0007_0000, d, l);
      if (d !== 64'hA5A5_0123_4567_89AB) begin
         errors++; $display("FAIL midwalk-inval: got %h (request dropped)", d);
      end else $display("PASS midwalk-inval-rearmed");

      // ===== Phase V1: VGA wide-read layout =================================
      // CPU writes four distinct dwords into one line, flushes them to DDR,
      // then one VGA read must return them in cache-line order.
      $display("Phase V1: VGA master D wide-read layout");
      for (int k = 0; k < 4; k++)
         cpu_write(32'h0060_0000 + 32'(8*k), 64'hA0A0_0000_0000_0000 | 64'(k), 8'hFF);
      do_flush;
      vga_burst(32'h0060_0000, 1);
      check_read(32'h0060_0010);                     // cache still sane after

      // ===== Phase V2: VGA vs CPU thrash + blitter + a dirty flush walk =====
      // Scanout-shaped traffic (4-read tenures over a 640 B line region) runs
      // against everything else.  VGA data is checked against the scoreboard,
      // so the VGA region is only ever written via flushed CPU stores (below,
      // before the fork).  The walk must let VGA in at MS_READ.
      $display("Phase V2: VGA concurrent with CPU, blitter and a flush walk");
      v2_cpu_done = 0;
      for (int k = 0; k < 80; k++)
         cpu_write(32'h0061_0000 + 32'(8*k), {$urandom, $urandom}, 8'hFF);
      do_flush;
      fork
         begin
            for (int m = 0; m < 200; m++) begin
               automatic logic [31:0] ca;
               ca = {15'b0, 2'($urandom), 10'h010, 5'b0}
                    | (32'($urandom & 1) << 5) | (32'($urandom & 3) << 3);
               sel = $urandom & 7;
               if (sel < 2)       check_read(ca);
               else if (sel == 7) cpu_zero(ca);    // DCBZ, often a dirty victim, under DMA grants
               else cpu_write(ca, {$urandom, $urandom}, (8'($urandom) | 8'h01));
            end
            // dirty a spread of sets, then flush while VGA is streaming
            for (int m = 0; m < 64; m++)
               cpu_write(32'h0008_0000 + 32'(m) * 32, {$urandom, $urandom}, 8'hFF);
            do_flush;
            v2_cpu_done = 1;
         end
         begin
            for (int m = 0; m < 10; m++) begin
               automatic logic [127:0] dv, dr;
               dv = {$urandom, $urandom, $urandom, $urandom};
               dma_write(32'h0044_0000 + 32'(m) * 16, dv);
               dma_read(32'h0044_0000 + 32'(m) * 16, dr);
               if (dr !== dv) begin
                  errors++; $display("FAIL V2 dma round-trip: got %h want %h", dr, dv);
               end
            end
         end
         begin
            for (int m = 0; !v2_cpu_done; m++)          // stream until the walk is over
               vga_burst(32'h0061_0000 + 32'(m % 5) * 128, 4);
         end
      join
      $display("  VGA reads: %0d, VGA grants during a maintenance walk: %0d",
               vga_reads, vga_grants_in_walk);
      if (vga_grants_in_walk == 0) begin
         errors++; $display("FAIL V2: VGA was never granted inside a maintenance walk");
      end

      // ===== Phase P: zeroing throughput (DCBZ vs 4 stores per line) ========
      // 64 KiB streams (the whole cache), clean (invalid) and dirty victims,
      // and with scanout-shaped VGA traffic (a 4-read tenure every 320
      // cycles: 640 x 16 bpp, 40 reads per 3200-cycle display line).
      // Cycle counts and the per-state split are printed, not checked.
      $display("Phase P: zeroing throughput (cycles per 32 B line)");
      begin : phase_p
         do_inval;
         stream_lines(32'h0050_0000, 2048, 1);  p_report("dcbz clean", 2048);
         do_inval;
         stream_lines(32'h0052_0000, 2048, 0);  p_report("4x st64 clean", 2048);
         stream_lines(32'h0050_0000, 2048, 1);  p_report("dcbz dirty", 2048);
         stream_lines(32'h0054_0000, 2048, 0);  p_report("4x st64 dirty", 2048);
         p_dz = 1; p_vga_stream; p_report("dcbz clean +VGA", 2048);
         p_dz = 0; p_vga_stream; p_report("4x st64 clean +VGA", 2048);
         p_vdirty = 1;
         p_dz = 1; p_vga_stream; p_report("dcbz dirty +VGA", 2048);
         p_dz = 0; p_vga_stream; p_report("4x st64 dirty +VGA", 2048);
      end

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

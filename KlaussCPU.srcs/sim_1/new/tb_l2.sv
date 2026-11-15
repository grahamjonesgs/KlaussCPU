//===========================================================================
// tb_l2 — l2_cache unit bench (Verilator, HW_PERF_PLAN.md §6).
//
// Drives the L2's front port with mem_read_write's arbiter protocol (DV level
// until ready, then DV low >= 1 cycle; one master at a time): the D-cache
// (32 B reads with a critical dword and the speculative lookup, 32 B
// victim writes), blitter / core 2 (16 B reads, 16 B masked writes) and VGA
// (32 B reads), plus L2_CTRL writes.  The behavioural MIG (mig_model.sv)
// is the DDR.
//
// Memory-consistency checker: a reference memory takes every accepted
// write; every read answer (and every early critical dword) must equal it.
// After a write-back-and-invalidate the MIG's memory must equal it too.
//
// +TEST=<name> runs a directed test (see the case below); +TEST=random
// (default) runs +N transactions from +SEED with random control writes.
// Prints "TB_L2 PASS" / "TB_L2 FAIL <why>" and the counters.
//===========================================================================
`timescale 1ns / 1ps
module tb_l2;
   logic clk = 0;
   always #5 clk = ~clk;

   logic        wdv = 0, rdv = 0, wide = 1, dma = 0, ifetch = 0;
   logic [31:0] addr = 0;
   logic [1:0]  dwoff = 0;
   logic [255:0] wdata = 0;
   logic [15:0] mask = 0;
   logic        spec_v = 0;
   logic [31:0] spec_a = 0;
   logic        ctrl_wr = 0, ctrl_en = 0;
   logic [1:0]  ctrl_wd = 0;
   wire  [255:0] rdata;
   wire         ready, dw_ready, next_ok, en, busy, calib;
   wire  [63:0] rd_dw, rd_dw_next;
   wire  [10:0] ev;

   l2_cache #(.WAYS(`L2_WAYS)) dut (
      .ddr2_dq(), .ddr2_dqs_n(), .ddr2_dqs_p(), .ddr2_addr(), .ddr2_ba(), .ddr2_ras_n(),
      .ddr2_cas_n(), .ddr2_we_n(), .ddr2_ck_p(), .ddr2_ck_n(), .ddr2_cke(), .ddr2_cs_n(),
      .ddr2_dm(), .ddr2_odt(), .resetn(1'b1), .sys_clk_i(clk),
      .i_mem_write_DV(wdv), .i_mem_read_DV(rdv), .i_mem_addr(addr), .i_mem_wide(wide),
      .i_mem_dw_off(dwoff), .i_mem_write_data(wdata), .i_app_wdf_mask(mask),
      .o_mem_read_data(rdata), .o_mem_ready(ready), .o_mem_rd_dw(rd_dw),
      .o_mem_rd_dw_next(rd_dw_next), .o_mem_dw_next_ok(next_ok), .o_mem_dw_ready(dw_ready),
      .o_calib_done(calib), .o_ui_clk(),
      .i_dma(dma), .i_ifetch(ifetch), .i_spec_v(spec_v), .i_spec_addr(spec_a),
      .i_ctrl_wr(ctrl_wr), .i_ctrl_wdata(ctrl_wd), .i_ctrl_en(ctrl_en),
      .o_en(en), .o_busy(busy), .o_ev(ev));

   // ---------------- reference memory (dword granular) ----------------
   logic [63:0] ref_m [logic [31:0]];
   function automatic logic [63:0] rget(input logic [31:0] a);
      a = {a[31:3], 3'b0};
      return ref_m.exists(a) ? ref_m[a] : 64'h0;
   endfunction
   int fails = 0;
   string why = "";
   task automatic fail(input string s);
      fails++;
      if (fails <= 10) $display("TB_L2 ERR t=%0t %s", $time, s);
      if (why == "") why = s;
   endtask

   // MIG memory (after a write-back): byte address -> slot
   function automatic logic [63:0] mig_get(input logic [31:0] a);
      logic [31:0] b = {a[31:4], 4'b0};
      int s = int'(b >> 3) + (a[3] ? 0 : 1);
      return dut.mig_7series_0.mem_lo[s];
   endfunction

   // ---------------- counters ----------------
   int evc [11];
   always @(posedge clk) for (int k = 0; k < 11; k++) if (ev[k]) evc[k]++;
   localparam I_HIT = 0, I_MISS = 1, D_HIT = 2, D_MISS = 3, W_HIT = 4, W_MISS = 5,
              RD_BEAT = 6, WR_BEAT = 7, WBFULL = 8, DMA_HIT = 9, DMA_MISS = 10;
   // probes: demand reads that waited on a buffer entry of their line
   int wbm_wait = 0, spec_used = 0, walks = 0;
   always @(posedge clk) begin
      if (dut.f_st == 4'd2 && !dut.q_wr && !dut.w_rd_hit && dut.f_wbm(dut.q_line)) wbm_wait++;
      if (dut.f_st == 4'd0 && dut.w_acc_ok && dut.r_en && dut.w_imm && dut.f_in_ddr(addr)) spec_used++;
      if (ctrl_wr && !busy && (ctrl_wd != 2'b01)) walks++;
   end

   // ---------------- one transaction ----------------
   int lat_last = 0;
   int dw_seen = 0;
   task automatic xact(input bit w, input bit d, input bit wd, input logic [31:0] a,
                       input logic [1:0] off, input logic [255:0] data, input logic [15:0] m,
                       input bit spec, input bit ifx);
      int t0;
      bit got_dw;
      // the master's request: D-cache 32 B aligned, narrow 16 B aligned
      a = wd ? {a[31:5], 5'b0} : {a[31:4], 4'b0};
      if (wd && w) m = 16'h0;
      if (spec && !d && wd) begin
         @(negedge clk);
         spec_v = 1; spec_a = a;
      end
      @(negedge clk);
      spec_v = 0;
      wdv = w; rdv = !w; dma = d; wide = wd; addr = a; dwoff = off; wdata = data; mask = m;
      ifetch = ifx;
      t0 = 0; got_dw = 0;
      forever begin
         @(posedge clk); #1;
         t0++;
         if (dw_ready && !w && wd && !d) begin
            logic [31:0] ca = a + 8 * off;
            got_dw = 1; dw_seen++;
            if (rd_dw !== rget(ca)) fail($sformatf("early dw %08x: %h want %h", ca, rd_dw, rget(ca)));
            if (next_ok && rd_dw_next !== rget(ca + 8))
               fail($sformatf("early next %08x: %h want %h", ca + 8, rd_dw_next, rget(ca + 8)));
         end
         if (ready) break;
         if (t0 > 20000) begin fail($sformatf("hang %s %08x", w ? "wr" : "rd", a)); break; end
      end
      lat_last = t0;
      // check / apply at the ready (the L2 serialises; writes are posted)
      if (w) begin
         if (wd) for (int j = 0; j < 4; j++) ref_m[a + 8 * j] = data[255 - 64 * j -: 64];
         else begin
            logic [63:0] hi = rget(a), lo = rget(a + 8);
            for (int i = 0; i < 8; i++) begin
               if (!m[8 + i]) hi[i*8 +: 8] = data[64 + i*8 +: 8];
               if (!m[i])     lo[i*8 +: 8] = data[i*8 +: 8];
            end
            ref_m[a] = hi; ref_m[a + 8] = lo;
         end
      end else begin
         if (wd) begin
            for (int j = 0; j < 4; j++)
               if (rdata[255 - 64 * j -: 64] !== rget(a + 8 * j))
                  fail($sformatf("%s read %08x dw%0d: %h want %h", d ? "dma" : "cache",
                                 a, j, rdata[255 - 64 * j -: 64], rget(a + 8 * j)));
         end else begin
            if (rdata[127:64] !== rget(a) || rdata[63:0] !== rget(a + 8))
               fail($sformatf("narrow read %08x: %h want %h_%h", a, rdata[127:0], rget(a), rget(a + 8)));
         end
      end
      wdv = 0; rdv = 0;
      // DV low at least one cycle
      @(negedge clk);
   endtask

   task automatic ctrl(input logic [1:0] v);
      @(negedge clk);
      ctrl_wr = 1; ctrl_wd = v;
      @(negedge clk);
      ctrl_wr = 0;
   endtask
   task automatic wait_idle();
      int n = 0;
      @(negedge clk);
      while (busy && n < 400000) begin @(negedge clk); n++; end
      if (busy) fail("walk hang");
   endtask
   // the whole reference must be in the MIG's memory
   task automatic check_ddr(input string tag);
      foreach (ref_m[a]) if (a < 32'h0080_0000 && mig_get(a) !== ref_m[a])
         fail($sformatf("%s: DDR %08x = %h want %h", tag, a, mig_get(a), ref_m[a]));
   endtask

   function automatic logic [255:0] rnd256();
      return {$urandom, $urandom, $urandom, $urandom, $urandom, $urandom, $urandom, $urandom};
   endfunction

   // addresses: lines L(k, s) = base + k * set stride + s * 64, k = 0..9
   localparam int SETS = 128 * 1024 / 64 / `L2_WAYS;
   localparam logic [31:0] STRIDE = 32'(SETS * 64);
   function automatic logic [31:0] L(input int k, input int s);
      return 32'h0001_0000 + 32'(k) * STRIDE + 32'(s) * 64;
   endfunction

   int expect_ev [11];
   task automatic ev_check(input string tag, input int k, input int want);
      repeat (80) @(negedge clk);     // the fill's tail, the strobes
      if (evc[k] != want) fail($sformatf("%s: event %0d = %0d want %0d", tag, k, evc[k], want));
   endtask

   // reference PLRU
   function automatic int plru_vic(input int b[8]);
      int node = 1, lo = 0, span = `L2_WAYS;
      while (span > 1) begin span /= 2; if (b[node]) begin node = 2 * node + 1; lo += span; end else node = 2 * node; end
      return lo;
   endfunction
   task automatic plru_touch(inout int b[8], input int w);
      int node = 1, lo = 0, span = `L2_WAYS;
      while (span > 1) begin
         span /= 2;
         if (w >= lo + span) begin b[node] = 0; node = 2 * node + 1; lo += span; end
         else begin b[node] = 1; node = 2 * node; end
      end
   endtask

   string test = "random";
   int seed = 1, nx = 2000;
   initial begin
      void'($value$plusargs("TEST=%s", test));
      void'($value$plusargs("SEED=%d", seed));
      void'($value$plusargs("N=%d", nx));
      void'($urandom(seed));
      wait (calib);
      repeat (3) @(negedge clk);
      case (test)
         // ---------------------------------------------------------------
         "hit": begin
            // a miss fills the line (critical sector first); the same sector
            // hits with the speculative lookup in 2 cycles from DV, the other
            // sector hits too (the whole line was fetched), with no DDR read
            xact(0, 0, 1, L(0, 0) + 32, 2'd1, 0, 0, 1, 0);
            ev_check("hit", D_MISS, 1); ev_check("hit", RD_BEAT, 8);
            xact(0, 0, 1, L(0, 0) + 32, 2'd3, 0, 0, 1, 0);
            if (lat_last != 2) fail($sformatf("spec hit latency %0d want 2", lat_last));
            xact(0, 0, 1, L(0, 0), 2'd0, 0, 0, 0, 0);
            if (lat_last != 3) fail($sformatf("hit latency %0d want 3", lat_last));
            xact(0, 0, 1, L(0, 0), 2'd2, 0, 0, 0, 1);    // I-fill hit
            ev_check("hit", D_HIT, 2); ev_check("hit", I_HIT, 1); ev_check("hit", RD_BEAT, 8);
         end
         // ---------------------------------------------------------------
         "sector": begin
            // a D-cache victim (32 B) that misses allocates its sector without
            // a fetch; that sector then hits, the other sector misses and
            // fetches 32 B only
            ref_m.delete();
            xact(1, 0, 1, L(1, 0) + 32, 0, rnd256(), 0, 0, 0);
            repeat (5) @(negedge clk);
            ev_check("sector", W_MISS, 1); ev_check("sector", RD_BEAT, 0);
            xact(0, 0, 1, L(1, 0) + 32, 2'd2, 0, 0, 1, 0);
            ev_check("sector", D_HIT, 1); ev_check("sector", RD_BEAT, 0);
            xact(0, 0, 1, L(1, 0), 2'd0, 0, 0, 1, 0);
            ev_check("sector", D_MISS, 1); ev_check("sector", RD_BEAT, 4);
            xact(1, 0, 1, L(1, 0), 0, rnd256(), 0, 0, 0);
            ev_check("sector", W_HIT, 1);
            xact(0, 0, 1, L(1, 0), 2'd1, 0, 0, 1, 0);
            ev_check("sector", D_HIT, 2);
         end
         // ---------------------------------------------------------------
         "evict": begin
            // WAYS + 2 dirty lines in one set: each allocation past WAYS evicts
            // a dirty line through the write buffer; read everything back
            for (int k = 0; k < `L2_WAYS + 2; k++) begin
               xact(1, 0, 1, L(k, 3), 0, rnd256(), 0, 0, 0);
               xact(1, 0, 1, L(k, 3) + 32, 0, rnd256(), 0, 0, 0);
            end
            repeat (200) @(negedge clk);
            ev_check("evict", WR_BEAT, 2 * 8);       // two whole lines drained
            for (int k = 0; k < `L2_WAYS + 2; k++) begin
               xact(0, 0, 1, L(k, 3), 2'd1, 0, 0, 1, 0);
               xact(0, 0, 1, L(k, 3) + 32, 2'd2, 0, 0, 1, 0);
            end
         end
         // ---------------------------------------------------------------
         "wbhit": begin
            // a demand read of a line whose dirty copy is still in the write
            // buffer: the entry drains first, then the read (no stale data);
            // long DDR latency keeps the buffer busy
            for (int r = 0; r < 6; r++) begin
               for (int k = 0; k <= `L2_WAYS; k++)
                  xact(1, 0, 1, L(k + 10 * r, 5) + 32 * (r % 2), 0, rnd256(), 0, 0, 0);
               // the oldest (way 0's line) was just evicted into the buffer
               xact(0, 0, 1, L(10 * r, 5) + 32 * (r % 2), 2'(r), 0, 0, 1, 0);
               xact(0, d_of(r), 0, L(10 * r, 5) + 16 * (r % 4), 0, 0, 0, 0, 0);
            end
            if (wbm_wait == 0) fail("no read waited on a buffer entry (test vacuous)");
         end
         // ---------------------------------------------------------------
         "plru": begin
            int b[8];
            int lines[8];
            for (int i = 0; i < 8; i++) b[i] = 0;
            // fill the ways in order (invalid ways first), then touch a pattern
            for (int k = 0; k < `L2_WAYS; k++) begin
               xact(0, 0, 1, L(k, 7), 0, 0, 0, 1, 0);
               plru_touch(b, k);
               lines[k] = k;
            end
            for (int r = 0; r < 40; r++) begin
               int t = $urandom_range(`L2_WAYS - 1, 0);
               int v, n0;
               xact(0, 0, 1, L(lines[t], 7), 0, 0, 0, 1, 0);   // hit: touch
               plru_touch(b, t);
               if (r % 5 == 4) begin
                  // a new line: replaces the PLRU victim
                  v = plru_vic(b);
                  xact(0, 0, 1, L(20 + r, 7), 0, 0, 0, 1, 0);
                  plru_touch(b, v);
                  n0 = evc[D_MISS];
                  xact(0, 0, 1, L(lines[v], 7), 0, 0, 0, 1, 0);   // the victim: a miss
                  if (evc[D_MISS] != n0 + 1) fail($sformatf("plru: way %0d not the victim", v));
                  // that refill replaced the new victim
                  begin int v2 = plru_vic(b); plru_touch(b, v2); lines[v2] = lines[v]; end
                  lines[v] = 20 + r;
               end
            end
         end
         // ---------------------------------------------------------------
         "dma": begin
            // (a) the D-cache's dirty line in the L2: DMA and VGA reads see it,
            //     from the L2 (no DDR read)
            xact(1, 0, 1, L(2, 9), 0, rnd256(), 0, 0, 0);
            xact(0, 1, 0, L(2, 9) + 16, 0, 0, 0, 0, 0);
            xact(0, 1, 1, L(2, 9), 0, 0, 0, 0, 0);
            ev_check("dma a", DMA_HIT, 2); ev_check("dma a", RD_BEAT, 0);
            // (b) a DMA write into a valid sector merges; the D-cache reads it
            xact(1, 1, 0, L(2, 9), 0, rnd256(), 16'h0F3C, 0, 0);
            xact(0, 0, 1, L(2, 9), 2'd0, 0, 0, 1, 0);
            ev_check("dma b", DMA_HIT, 3); ev_check("dma b", RD_BEAT, 0);
            // (c) a DMA write that misses goes to DDR (posted): the next
            //     read of the line (a miss) waits for it
            xact(1, 1, 0, L(3, 9) + 48, 0, rnd256(), 16'h00FF, 0, 0);
            xact(0, 0, 1, L(3, 9) + 32, 2'd2, 0, 0, 1, 0);
            xact(0, 1, 0, L(3, 9) + 48, 0, 0, 0, 0, 0);
            // (d) ordering: a D-cache writeback, then a DMA write of the same
            //     bytes, and the reverse — in the L2, in the buffer (L2 off),
            //     and across both
            for (int r = 0; r < 3; r++) begin
               if (r == 1) begin ctrl(2'b00); wait_idle(); end
               if (r == 2) begin ctrl(2'b01); end
               xact(1, 0, 1, L(4, 10 + r), 0, rnd256(), 0, 0, 0);
               xact(1, 1, 0, L(4, 10 + r), 0, rnd256(), 16'h0000, 0, 0);
               xact(1, 1, 0, L(4, 10 + r) + 16, 0, rnd256(), 16'h00F0, 0, 0);
               xact(1, 0, 1, L(5, 10 + r), 0, rnd256(), 0, 0, 0);
               if (r == 2) xact(1, 0, 1, L(5, 10 + r) + 32, 0, rnd256(), 0, 0, 0);
               xact(1, 1, 0, L(5, 10 + r) + 32, 0, rnd256(), 16'hF00F, 0, 0);
               xact(1, 0, 1, L(5, 10 + r) + 32, 0, rnd256(), 0, 0, 0);
               xact(0, 1, 1, L(4, 10 + r), 0, 0, 0, 0, 0);
               xact(0, 0, 1, L(5, 10 + r) + 32, 2'd3, 0, 0, 1, 0);
            end
            ctrl(2'b11); wait_idle(); check_ddr("dma");
         end
         // ---------------------------------------------------------------
         "wbinv": begin
            // dirty lines everywhere; WBINV writes them all back (DDR checked)
            // and invalidates (every later read misses); the D-cache and DMA
            // keep running during the walk (reads, writes to walked and
            // not-yet-walked sets)
            for (int k = 0; k < 6; k++) for (int s = 0; s < 40; s++)
               xact(1, 0, 1, L(k, s * 13) + 32 * (s % 2), 0, rnd256(), 0, 0, 0);
            ctrl(2'b11);
            @(negedge clk); if (!busy) fail("BUSY not set right after the write");
            for (int i = 0; i < 300 && busy; i++) begin
               int s = $urandom_range(SETS - 1, 0);
               xact($urandom_range(1, 0), $urandom_range(3, 0) == 0, 1, L($urandom_range(5, 0), s / 64) + 32'(s) * 0 + 32 * $urandom_range(1, 0),
                    2'($urandom), rnd256(), 0, $urandom_range(1, 0), 0);
               xact(1, 0, 1, L(0, $urandom_range(SETS - 1, 0)), 0, rnd256(), 0, 0, 0);
            end
            wait_idle();
            check_ddr("wbinv");
            begin
               int m0 = evc[D_MISS];
               xact(0, 0, 1, L(0, 0), 0, 0, 0, 1, 0);
               if (evc[D_MISS] != m0 + 1) fail("read after WBINV hit");
            end
            if (!en) fail("EN dropped by WBINV");
         end
         // ---------------------------------------------------------------
         "toggle": begin
            // dirty data, then disable (0): written back, bypass; a write of 1
            // during the walk is ignored (still bypass after it); EN=1
            // re-enables; then data written while disabled is read back
            for (int s = 0; s < 64; s++) xact(1, 0, 1, L(s % 7, s), 0, rnd256(), 0, 0, 0);
            ctrl(2'b00);
            @(negedge clk); if (!busy) fail("BUSY not 1 after a write of 0");
            ctrl(2'b01);                    // ignored: BUSY
            xact(1, 0, 1, L(1, 1), 0, rnd256(), 0, 0, 0);
            wait_idle();
            if (en || dut.r_en) fail("write of 1 during the 0-walk was not ignored");
            check_ddr("toggle disable");
            begin
               int h0 = evc[D_HIT] + evc[D_MISS] + evc[W_HIT] + evc[W_MISS];
               for (int s = 0; s < 16; s++) begin
                  xact(1, 0, 1, L(s % 3, s), 0, rnd256(), 0, 0, 0);
                  xact(0, 0, 1, L(s % 3, s), 2'(s), 0, 0, 1, 0);
                  xact(1, 1, 0, L(s % 3, s) + 16, 0, rnd256(), 16'h1234, 0, 0);
                  xact(0, 1, 0, L(s % 3, s) + 16, 0, 0, 0, 0, 0);
               end
               if (evc[D_HIT] + evc[D_MISS] + evc[W_HIT] + evc[W_MISS] != h0) fail("bypass counted lookups");
            end
            repeat (100) @(negedge clk);
            check_ddr("toggle bypass");
            ctrl(2'b01);
            @(negedge clk); if (!en || busy) fail("enable");
            for (int s = 0; s < 16; s++) xact(0, 0, 1, L(s % 3, s), 2'(s), 0, 0, 1, 0);
            xact(0, 0, 1, L(0, 0), 0, 0, 0, 1, 0);
            xact(0, 0, 1, L(0, 0), 0, 0, 0, 1, 0);
            if (evc[D_HIT] == 0) fail("no hit after re-enable");
            // the CPU reset / program-load enable cancels a pending disable
            ctrl(2'b00);
            @(negedge clk); ctrl_en = 1; @(negedge clk); ctrl_en = 0;
            wait_idle();
            if (!en || !dut.r_en) fail("program-load enable did not cancel the disable");
         end
         // ---------------------------------------------------------------
         "range": begin
            // outside DDR (PA[31:27] != 0): no lookup, no allocation
            xact(0, 0, 1, 32'h0800_0040, 0, 0, 0, 1, 0);
            xact(1, 0, 1, 32'h0800_0080, 0, rnd256(), 0, 0, 0);
            xact(0, 0, 1, 32'h4000_0040, 0, 0, 0, 1, 0);
            if (evc[D_HIT] + evc[D_MISS] + evc[W_HIT] + evc[W_MISS] != 0) fail("range: looked up");
            for (int s = 0; s < SETS; s++)
               if (dut.tag_ram[s] != '0) fail($sformatf("range: set %0d allocated", s));
            ref_m.delete();    // (aliases of the model's memory: not checked)
         end
         // ---------------------------------------------------------------
         default: begin
            // random: all masters, all kinds, a few hot sets (conflicts),
            // spec right and wrong, L2_CTRL writes at random
            for (int i = 0; i < nx; i++) begin
               int kind = $urandom_range(99, 0);
               int k = $urandom_range(9, 0), s = $urandom_range(3, 0);
               logic [31:0] a = L(k, s) + 32'($urandom_range(63, 0));
               if (kind < 30)      xact(0, 0, 1, a, 2'($urandom), 0, 0, $urandom_range(3, 0) != 0, $urandom_range(1, 0));
               else if (kind < 55) xact(1, 0, 1, a, 0, rnd256(), 0, 0, 0);
               else if (kind < 67) xact(0, 1, 0, a, 0, 0, 0, 0, 0);
               else if (kind < 75) xact(0, 1, 1, a, 0, 0, 0, 0, 0);
               else if (kind < 90) xact(1, 1, 0, a, 0, rnd256(), 16'($urandom), 0, 0);
               else if (kind < 92) begin
                  // a wrong speculation
                  @(negedge clk); spec_v = 1; spec_a = L($urandom_range(9, 0), s);
                  xact(0, 0, 1, a, 2'($urandom), 0, 0, 0, 0);
               end else if (kind < 93 && $urandom_range(9, 0) == 0) begin
                  logic [1:0] v = 2'($urandom_range(3, 0));
                  ctrl(v == 2'b10 ? 2'b11 : v);
               end else if (kind < 94 && $urandom_range(19, 0) == 0) begin
                  ctrl(2'b11); wait_idle(); check_ddr($sformatf("random #%0d", i));
               end else begin
                  repeat ($urandom_range(30, 0)) @(negedge clk);
               end
            end
            ctrl(2'b11); wait_idle(); ctrl(2'b01);
            check_ddr("random end");
         end
      endcase
      repeat (20) @(negedge clk);
      $display("TB_L2 ev i_hit=%0d i_miss=%0d d_hit=%0d d_miss=%0d w_hit=%0d w_miss=%0d rd_beat=%0d wr_beat=%0d wbfull=%0d dma_hit=%0d dma_miss=%0d wbm_wait=%0d spec_used=%0d dw_seen=%0d",
               evc[0], evc[1], evc[2], evc[3], evc[4], evc[5], evc[6], evc[7], evc[8], evc[9], evc[10],
               wbm_wait, spec_used, dw_seen);
      if (fails == 0) $display("TB_L2 PASS %s", test);
      else            $display("TB_L2 FAIL %s %0d errors: %s", test, fails, why);
      $finish;
   end

   function automatic bit d_of(input int r);
      return r % 2;
   endfunction

   initial begin
      #500ms;
      $display("TB_L2 FAIL timeout");
      $finish;
   end
endmodule

//===========================================================================
// tb_l2 — l2_cache unit bench (Verilator, HW_PERF_PLAN.md §6).
//
// Drives the L2's front port with mem_read_write's arbiter protocol (DV level
// until ready, held one more cycle — the masters drop it on the edge after
// they see ready; +DVTAIL=0: dropped on that edge — then DV low >= 1 cycle;
// one master at a time): the D-cache
// (32 B reads with a critical dword and the speculative lookup, 32 B
// victim writes), blitter / core 2 (16 B reads, 16 B masked writes) and VGA
// (32 B reads), plus L2_CTRL writes.  The behavioural MIG (mig_model.sv)
// is the DDR.
//
// Memory-consistency checker: a reference memory takes every accepted
// write; every read answer (and every early critical dword) must equal it.
// After a write-back-and-invalidate the MIG's memory must equal it too.
// PA[31:27] alias PA[26:0] (the MIG's rank bit and the dropped high bits).
// Every ready must answer a request (a DV tail must not be taken as a new
// one).  VGA / DMA rule (review 1, M2; review 2): while the VGA request or
// a DMA master is on the bus, a write-buffer drain may start only for a
// request that waits on the buffer, or for the WBINV walk waiting on it
// (rt_bad must stay 0; the terms are spelled out here, not taken from the
// RTL's w_fe_need).  While the walk waits on a full buffer no demand read
// starts before its drain (wk_prio_bad must stay 0).
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
   logic        rt_req = 0;       // random in TEST=random (+RT=1)
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
      .i_dma(dma), .i_ifetch(ifetch), .i_rt_req(rt_req), .i_spec_v(spec_v), .i_spec_addr(spec_a),
      .i_ctrl_wr(ctrl_wr), .i_ctrl_wdata(ctrl_wd), .i_ctrl_en(ctrl_en),
      .o_en(en), .o_busy(busy), .o_ev(ev));

   // ---------------- reference memory (dword granular) ----------------
   logic [63:0] ref_m [logic [31:0]];
   function automatic logic [31:0] fold(input logic [31:0] a);
      return {5'b0, a[26:0]};
   endfunction
   function automatic logic [63:0] rget(input logic [31:0] a);
      a = {5'b0, a[26:3], 3'b0};
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
      if (dut.f_st == 4'd0 && dut.w_acc_ok && dut.r_en && dut.w_imm) spec_used++;
      if (ctrl_wr && !busy && (ctrl_wd != 2'b01)) walks++;
   end
   // walk yields (F_WK2, dirty way, buffer full, a request taken instead)
   int yields = 0;
   always @(posedge clk)
      if (dut.f_st == 4'd10 && dut.r_wk_any && dut.wb_full && dut.w_acc_ok) yields++;
   // drains started while VGA / a DMA master is on the bus: rt_need when
   // a request or the walk waited on the buffer, rt_bad otherwise (judged
   // on the edge the drain starts).  The need, from the design's rule:
   //   - the request in F_DEC: a read miss of a buffered line; a cache read
   //     miss that allocates a new line over a dirty victim, buffer full; a
   //     cache write that allocates over a dirty victim, buffer full; a
   //     posted write (the line not in the L2), buffer full or (the walk
   //     holding the next slot) not empty
   //   - the walk: a dirty way to copy and the buffer full; or its end with
   //     entries left from before its last set
   // (DMA and VGA read misses need no slot: a full buffer alone is no need)
   function automatic bit rt_need_now();
      bit hit, many, lk, dmaq, alloc, vdirty, full, wbm, post;
      hit   = dut.r_hit; many = dut.r_many; lk = dut.q_lk; dmaq = dut.q_dma;
      alloc = dut.r_en && !dut.r_walking;
      vdirty = dut.r_td[dut.r_vic] != 2'b00;
      full  = dut.wb_v == 2'b11;
      wbm   = (dut.wb_v[0] && dut.wb_line[0] == dut.q_addr[26:6]) ||
              (dut.wb_v[1] && dut.wb_line[1] == dut.q_addr[26:6]);
      post  = !(lk && !dmaq && many) && !(alloc && lk && !dmaq && !many) && !(lk && dmaq && hit);
      if (dut.r_wk_wb && full) return 1;
      if (dut.r_walk_end && dut.r_wk_snap && dut.r_wk_fl != 2'b00) return 1;
      if (dut.f_st != 4'd2) return 0;
      if (!dut.q_wr)
         return !(lk && hit) && (wbm || (alloc && lk && !dmaq && !many && vdirty && full));
      return (alloc && lk && !dmaq && !many && vdirty && full) ||
             (post && (full || (dut.r_wk_wb && dut.wb_v != 2'b00)));
   endfunction
   int rt_bad = 0, rt_need = 0, wk_prio_bad = 0, wk_prio = 0;
   bit rt_pend = 0, rt_pneed = 0, wk_rd = 0;
   always @(posedge clk) begin
      if (rt_pend && dut.e_st == 2'd2) begin
         if (rt_pneed) rt_need++;
         else begin
            rt_bad++;
            if (rt_bad <= 3) $display("TB_L2 rt drain at t=%0t f_st=%0d", $time, dut.f_st);
         end
      end
      // the walk waits on a full buffer whose oldest entry may drain, a
      // read waits in F_DEC, the engine is idle: the drain must start first
      if (wk_rd) begin
         if (dut.e_st == 2'd1) begin
            wk_prio_bad++;
            if (wk_prio_bad <= 3) $display("TB_L2 read before the walk's drain at t=%0t", $time);
         end else if (dut.e_st == 2'd2) wk_prio++;
      end
      wk_rd    = dut.e_st == 2'd0 && dut.f_st == 4'd2 && !dut.q_wr && !(dut.q_lk && dut.r_hit) &&
                 dut.r_wk_wb && dut.wb_v == 2'b11 && dut.wb_rdy[dut.wb_h];
      rt_pend  = dut.e_st == 2'd0 && (rt_req || dma);
      rt_pneed = rt_need_now();
   end
   // the VGA stream buffer: reads it answered; writes accepted to the line
   // of a stream read still in flight; ... on the edge sector 1 is captured
   int strm_hits = 0, strm_snoops = 0, strm_edge = 0;
   always @(posedge clk) begin
      if (dut.f_st == 4'd2 && !dut.q_wr && !dut.w_rd_hit && dut.w_strm_hit) strm_hits++;
      if (dut.w_wr_acc && dut.e_st == 2'd1 && dut.e_strm && addr[26:6] == dut.e_line) strm_snoops++;
      if (dut.w_wr_acc && dut.e_st == 2'd1 && dut.e_strm && !dut.e_rsp && dut.rb_sec_done &&
          addr[26:6] == dut.e_line) strm_edge++;
   end
   // every ready answers a request
   int n_req = 0, n_rdy = 0;
   always @(posedge clk) if (ready) n_rdy++;

   // ---------------- one transaction ----------------
   int lat_last = 0;
   int dw_seen = 0;
   int dvtail = 1;
   bit tenure = 0;
   // writes acknowledged after a walk ended (late_track): they may stay in
   // the L2, so check_ddr skips their dwords
   bit late_track = 0;
   bit late_m [logic [31:0]];
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
      if (n_rdy != n_req) begin
         fail($sformatf("%0d readies for %0d requests (before %08x)", n_rdy, n_req, a));
         n_rdy = n_req;
      end
      n_req++;
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
      // check / apply at the ready (the L2 serialises; writes are posted);
      // the reference is indexed by the DDR address (PA[31:27] alias)
      a = fold(a);
      if (w && late_track && !busy) for (int j = 0; j < 4; j++) late_m[a + 8 * j] = 1;
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
      // the DV tail: the masters drop DV on the edge after the ready
      if (dvtail) begin @(posedge clk); #1; end
      wdv = 0; rdv = 0;
      // the DMA grant (i_dma) ends with the transaction, or (tenure = 1)
      // stays for the next one, as a blitter's chunk of transactions
      if (!tenure) dma = 0;
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
   // the whole reference must be in the MIG's memory.  Right after a walk
   // with no traffic since the write that started it (BUSY = 0: DDR holds
   // every write acknowledged before the walk's end); check_ddr_drained
   // first waits for the write buffer (writes posted during the walk's end)
   task automatic check_ddr_drained(input string tag);
      for (int n = 0; n < 20000 && (dut.wb_v != 2'b00 || dut.e_st != 2'd0); n++) @(negedge clk);
      check_ddr(tag);
   endtask
   task automatic check_ddr(input string tag);
      // (writes the MIG accepted reach its memory at their CAS)
      for (int n = 0; n < 2000 && dut.mig_7series_0.cq.size() != 0; n++) @(negedge clk);
      foreach (ref_m[a]) if (a < 32'h0080_0000 && !late_m.exists(a) && mig_get(a) !== ref_m[a])
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

   // the VGA-request input: toggled at random in TEST=random (+RT=0: off)
   int rt_rand = 0;
   always @(negedge clk) if (rt_rand && $urandom_range(63, 0) == 0) rt_req = !rt_req;

   string test = "random";
   int seed = 1, nx = 2000;
   initial begin
      void'($value$plusargs("TEST=%s", test));
      void'($value$plusargs("SEED=%d", seed));
      void'($value$plusargs("N=%d", nx));
      void'($value$plusargs("DVTAIL=%d", dvtail));
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
            late_track = 1;
            for (int i = 0; i < 300 && busy; i++) begin
               int s = $urandom_range(SETS - 1, 0);
               xact($urandom_range(1, 0), $urandom_range(3, 0) == 0, 1, L($urandom_range(5, 0), s / 64) + 32'(s) * 0 + 32 * $urandom_range(1, 0),
                    2'($urandom), rnd256(), 0, $urandom_range(1, 0), 0);
               xact(1, 0, 1, L(0, $urandom_range(SETS - 1, 0)), 0, rnd256(), 0, 0, 0);
            end
            wait_idle();
            late_track = 0;
            check_ddr_drained("wbinv");
            late_m.delete();
            begin
               // a line dirty before the walk that the traffic above never
               // touches (it may run past BUSY's drop and allocate L(0, 0):
               // review 2's 8-way sweep caught that)
               int m0 = evc[D_MISS];
               xact(0, 0, 1, L(3, 7 * 13) + 32, 0, 0, 0, 1, 0);
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
               int h0 = evc[D_HIT] + evc[D_MISS] + evc[W_HIT] + evc[W_MISS] + evc[DMA_HIT] + evc[DMA_MISS];
               int b0, s0;
               for (int s = 0; s < 16; s++) begin
                  xact(1, 0, 1, L(s % 3, s), 0, rnd256(), 0, 0, 0);
                  xact(0, 0, 1, L(s % 3, s), 2'(s), 0, 0, 1, 0);
                  xact(1, 1, 0, L(s % 3, s) + 16, 0, rnd256(), 16'h1234, 0, 0);
                  xact(0, 1, 0, L(s % 3, s) + 16, 0, 0, 0, 0, 0);
               end
               // review 2: VGA's sector 0 then sector 1 in bypass — no stream
               // buffer (a disabled L2 is a plain path to DDR: two 32 B reads)
               repeat (100) @(negedge clk);
               b0 = evc[RD_BEAT]; s0 = strm_hits;
               for (int s = 0; s < 4; s++) begin
                  xact(0, 1, 1, L(6, 50 + s), 0, 0, 0, 0, 0);
                  while (dut.e_st != 2'd0) @(negedge clk);
                  xact(0, 1, 1, L(6, 50 + s) + 32, 0, 0, 0, 0, 0);
               end
               repeat (20) @(negedge clk);
               if (evc[RD_BEAT] - b0 != 4 * 8 || strm_hits != s0)
                  fail($sformatf("bypass: VGA reads used the stream buffer (%0d read beats want 32, %0d stream hits)",
                                 evc[RD_BEAT] - b0, strm_hits - s0));
               if (evc[D_HIT] + evc[D_MISS] + evc[W_HIT] + evc[W_MISS] + evc[DMA_HIT] + evc[DMA_MISS] != h0)
                  fail("bypass counted lookups");
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
         "alias": begin
            // PA[31:27] alias PA[26:0] (the MIG ignores the rank bit PA[27];
            // PA[31:28] never reach it): one L2 line per DDR line, whatever
            // the alias — writes through one alias, reads through another
            for (int r = 0; r < 4; r++) begin
               logic [31:0] h1 = 32'(r + 1) << 27, h2 = 32'((r * 5 + 3) % 32) << 27;
               xact(1, 0, 1, L(0, 4 + r), 0, rnd256(), 0, 0, 0);               // dirty in the L2
               xact(1, 1, 0, h1 | (L(0, 4 + r) + 16), 0, rnd256(), 16'h0FF0, 0, 0);
               xact(0, 0, 1, h2 | L(0, 4 + r), 2'(r), 0, 0, 1, 0);
               xact(0, 1, 1, h1 | L(0, 4 + r), 0, 0, 0, 0, 0);
               xact(1, 0, 1, h1 | (L(1, 4 + r) + 32), 0, rnd256(), 0, 0, 0);   // allocated via an alias
               xact(0, 0, 1, L(1, 4 + r) + 32, 2'd1, 0, 0, 1, 0);
               xact(1, 1, 0, h2 | (L(2, 4 + r) + 48), 0, rnd256(), 16'h00FF, 0, 0);   // posted
               xact(0, 1, 0, L(2, 4 + r) + 48, 0, 0, 0, 0, 0);
               xact(0, 0, 1, h1 | (L(2, 4 + r) + 32), 2'd3, 0, 0, 1, 0);
            end
            if (evc[D_HIT] == 0 || evc[DMA_HIT] == 0) fail("alias: no hit through an alias");
            ctrl(2'b11); wait_idle(); check_ddr("alias");
         end
         // ---------------------------------------------------------------
         "wbinvq": begin
            // review 1 M1: every set fully dirty (every way, both sectors),
            // then WBINV with a request waiting at nearly every yield (reads
            // back to back, one DV-low cycle apart): the walk must end, and
            // write each dirty line exactly once (a yield must not make it
            // copy a way again)
            int n0, nr;
            for (int k = 0; k < `L2_WAYS; k++) for (int s = 0; s < SETS; s++) begin
               xact(1, 0, 1, L(k, s), 0, rnd256(), 0, 0, 0);
               xact(1, 0, 1, L(k, s) + 32, 0, rnd256(), 0, 0, 0);
            end
            repeat (200) @(negedge clk);
            n0 = evc[WR_BEAT];
            ctrl(2'b11);
            nr = 0;
            while (busy && nr < 3 * SETS * `L2_WAYS) begin
               int k = $urandom_range(`L2_WAYS + 3, 0), s = $urandom_range(SETS - 1, 0);
               if ($urandom_range(1, 0)) xact(0, 0, 1, L(k, s) + 32 * $urandom_range(1, 0), 2'($urandom), 0, 0, 1, 0);
               else                      xact(0, 1, $urandom_range(1, 0), L(k, s) + 16 * $urandom_range(3, 0), 0, 0, 0, 0, 0);
               nr++;
            end
            if (busy) fail($sformatf("wbinvq: walk still at set %0d after %0d requests (livelock)", dut.r_walk_set, nr));
            wait_idle();
            if (evc[WR_BEAT] - n0 != SETS * `L2_WAYS * 8)
               fail($sformatf("wbinvq: %0d write beats want %0d (a line written twice)", evc[WR_BEAT] - n0, SETS * `L2_WAYS * 8));
            if (yields == 0) fail("wbinvq: the walk never yielded (test vacuous)");
            $display("TB_L2 wbinvq: %0d requests during the walk, %0d yields", nr, yields);
            check_ddr("wbinvq");
         end
         // ---------------------------------------------------------------
         "wkturn": begin
            // review 1 minor 2: a master whose requests keep the front end
            // busy (DMA write hits, DV re-raised after one low cycle) must
            // not hold BUSY up for its whole tenure: the walk takes turns
            // (the grant is held across 16 transactions, then dropped for
            // a few cycles, as a blitter's tenure; without the turns the
            // walk advanced only in those gaps)
            int nr;
            for (int s = 0; s < SETS; s += 16) xact(1, 0, 1, L(s % 3, s), 0, rnd256(), 0, 0, 0);
            xact(1, 0, 1, L(0, SETS - 1), 0, rnd256(), 0, 0, 0);
            ctrl(2'b11);
            nr = 0;
            tenure = 1;
            while (busy && nr < 4 * SETS) begin
               xact(1, 1, 0, L(0, SETS - 1) + 16 * (nr % 2), 0, rnd256(), 16'(nr * 7919), 0, 0);
               nr++;
               if (nr % 16 == 0) begin dma = 0; repeat (3) @(negedge clk); end
            end
            tenure = 0; dma = 0;
            if (busy) fail($sformatf("wkturn: BUSY after %0d requests (walk at set %0d)", nr, dut.r_walk_set));
            $display("TB_L2 wkturn: walk done after %0d requests", nr);
            // (BUSY = 0: DDR holds the writes acknowledged before the walk's
            // end; the stream's later ones go out with the next walk)
            wait_idle(); ctrl(2'b11); wait_idle(); check_ddr("wkturn");
         end
         // ---------------------------------------------------------------
         "vgastream": begin
            // review 1 (VGA margin): a VGA read of sector 0 of a line the L2
            // does not hold fetches the line; the next read (sector 1) is
            // answered from the stream buffer at hit latency, no DDR read.
            // A write to the line (after the fetch, or while sector 1 is
            // still on its way) drops it.
            int b0, h0;
            b0 = evc[RD_BEAT];
            xact(0, 1, 1, L(5, 30), 0, 0, 0, 0, 0);
            while (dut.e_st != 2'd0) @(negedge clk);     // (sector 1 in)
            xact(0, 1, 1, L(5, 30) + 32, 0, 0, 0, 0, 0);
            if (lat_last > 3) fail($sformatf("vgastream: stream hit latency %0d", lat_last));
            repeat (20) @(negedge clk);
            if (evc[RD_BEAT] - b0 != 8) fail($sformatf("vgastream: %0d read beats want 8", evc[RD_BEAT] - b0));
            if (strm_hits != 1) fail("vgastream: no stream hit");
            // writes to sector 1 after the fetch (sector 1 in the buffer):
            // a DMA write (posted), a D-cache writeback (allocated)
            for (int r = 0; r < 4; r++) begin
               xact(0, 1, 1, L(6, 30 + r), 0, 0, 0, 0, 0);
               while (dut.e_st != 2'd0) @(negedge clk);
               if (r % 2 == 0) xact(1, 1, 0, L(6, 30 + r) + 48, 0, rnd256(), 16'h0F0F, 0, 0);
               else            xact(1, 0, 1, L(6, 30 + r) + 32, 0, rnd256(), 0, 0, 0);
               repeat (40) @(negedge clk);       // (the posted write drains)
               if (r % 2 == 1) begin ctrl(2'b11); wait_idle(); end   // (out of the L2 again)
               xact(0, 1, 1, L(6, 30 + r) + 32, 0, 0, 0, 0, 0);
            end
            // writes while sector 1 is on its way (DDR beats spread out)
            dut.mig_7series_0.jit = 40;
            h0 = strm_hits;
            for (int r = 0; r < 40; r++) begin
               logic [31:0] b = L(7 + r % 3, 31 + r);
               xact(0, 1, 1, b, 0, 0, 0, 0, 0);
               if (r % 4 == 0)      xact(1, 1, 0, b + 32 + 16 * (r % 2), 0, rnd256(), 16'(r * 4099), 0, 0);
               else if (r % 4 == 1) xact(1, 0, 1, b + 32, 0, rnd256(), 0, 0, 0);
               else if (r % 4 == 2) xact(1, 1, 0, b + 16, 0, rnd256(), 16'(r * 977), 0, 0);
               xact(0, 1, 1, b + 32, 0, 0, 0, 0, 0);
            end
            dut.mig_7series_0.jit = 0;
            // a line the L2 holds with only sector 1 (dirty): a VGA read of
            // sector 0 must not fill the stream buffer from DDR (stale
            // sector 1), which a read after WBINV would then return
            for (int r = 0; r < 4; r++) begin
               xact(1, 0, 1, L(1, 40 + r) + 32, 0, rnd256(), 0, 0, 0);
               xact(0, 1, 1, L(1, 40 + r), 0, 0, 0, 0, 0);
               ctrl(2'b11); wait_idle();
               xact(0, 1, 1, L(1, 40 + r) + 32, 0, 0, 0, 0, 0);
            end
            if (strm_snoops == 0) fail("vgastream: no write during a stream fill (test vacuous)");
            if (strm_hits == h0) fail("vgastream: no stream hit in the second part");
            $display("TB_L2 vgastream: %0d stream hits, %0d writes during a stream fill", strm_hits, strm_snoops);
            ctrl(2'b11); wait_idle(); check_ddr("vgastream");
         end
         // ---------------------------------------------------------------
         "rtdrain": begin
            // review 1 M2: while VGA wants the bus no drain starts unless a
            // request waits on the buffer.  A posted DMA write leaves an
            // entry; VGA reads (a hit, a miss) do not drain it and keep
            // their latency; a VGA read of the buffered line needs it
            int h;
            xact(1, 0, 1, L(0, 20), 0, rnd256(), 0, 0, 0);       // dirty in the L2
            xact(0, 0, 1, L(1, 21), 0, 0, 0, 1, 0);              // clean in the L2
            @(negedge clk); rt_req = 1;
            xact(1, 1, 0, L(2, 22) + 16, 0, rnd256(), 16'h0FF0, 0, 0);   // posted: an entry
            repeat (60) @(negedge clk);
            if (dut.wb_v == 2'b00) fail("rtdrain: the entry drained while VGA wanted the bus");
            for (int i = 0; i < 6; i++) begin
               xact(0, 1, 1, L(i % 2, 20 + i % 2), 0, 0, 0, 0, 0);         // hits
               if (lat_last > 3) fail($sformatf("rtdrain: VGA hit latency %0d", lat_last));
               xact(0, 1, 1, L(3 + i, 23), 0, 0, 0, 0, 0);                // misses
            end
            if (dut.wb_v == 2'b00) fail("rtdrain: drained during VGA reads");
            h = rt_need;
            xact(0, 1, 1, L(2, 22), 0, 0, 0, 0, 0);              // its line: waits for the drain
            if (rt_need != h + 1) fail($sformatf("rtdrain: %0d needed drains want 1", rt_need - h));
            // review 2 (m-B): a full buffer (two posted writes); VGA and DMA
            // read misses need no slot, so they must not drain it
            repeat (60) @(negedge clk);
            xact(1, 1, 0, L(2, 24) + 16, 0, rnd256(), 16'h00F0, 0, 0);
            xact(1, 1, 0, L(2, 25), 0, rnd256(), 16'h0F00, 0, 0);
            repeat (5) @(negedge clk);                           // (posted after its ready)
            if (dut.wb_v != 2'b11) fail("rtdrain: buffer not full (test vacuous)");
            h = strm_hits;
            for (int i = 0; i < 6; i++) begin
               xact(0, 1, 1, L(3 + i, 26), 0, 0, 0, 0, 0);              // VGA misses (stream fills)
               while (dut.e_st != 2'd0) @(negedge clk);
               xact(0, 1, 1, L(3 + i, 26) + 32, 0, 0, 0, 0, 0);         // ... stream hits
               xact(0, 1, 0, L(3 + i, 27) + 16, 0, 0, 0, 0, 0);         // DMA misses
               xact(0, 1, 1, L(i % 2, 20 + i % 2), 0, 0, 0, 0, 0);      // VGA hits
            end
            if (strm_hits == h) fail("rtdrain: no stream hit with the buffer full (test vacuous)");
            if (dut.wb_v != 2'b11) fail("rtdrain: a full buffer drained for VGA / DMA reads");
            @(negedge clk); rt_req = 0;
            ctrl(2'b11); wait_idle(); check_ddr("rtdrain");
         end
         // ---------------------------------------------------------------
         "dmapost": begin
            // review 2 (m-A): a DMA master's posted writes (lines the L2 does
            // not hold) during a WBINV walk with a dirty line in every set,
            // the grant held over chunks of 8 and dropped one cycle between
            // them (a blitter), with 0, 10 and 40 idle cycles between the
            // writes.  On c9cadea each posted write took the slot a drain
            // freed and the walk stayed at set 1; now BUSY must drop within
            // 4 x SETS requests
            int gaps[3] = '{0, 10, 40};
            foreach (gaps[g]) begin
               int nr = 0, t0 = 0;
               for (int s = 0; s < SETS; s++) xact(1, 0, 1, L(s % 3, s), 0, rnd256(), 0, 0, 0);
               ctrl(2'b11);
               t0 = int'($time / 10);
               tenure = 1;
               while (busy && nr < 4 * SETS) begin
                  xact(1, 1, 0, L(5 + g, far_set(nr)) + 16 * (nr % 4), 0, rnd256(), 16'(nr * 31), 0, 0);
                  nr++;
                  repeat (gaps[g]) @(negedge clk);
                  if (nr % 8 == 0) begin dma = 0; @(negedge clk); dma = 1; end
               end
               tenure = 0; dma = 0;
               if (busy) fail($sformatf("dmapost gap %0d: BUSY after %0d requests (walk at set %0d of %0d)",
                                        gaps[g], nr, dut.r_walk_set, SETS));
               $display("TB_L2 dmapost gap %0d: walk done after %0d requests, %0d cycles", gaps[g], nr,
                        int'($time / 10) - t0);
               wait_idle(); ctrl(2'b11); wait_idle(); check_ddr($sformatf("dmapost gap %0d", gaps[g]));
            end
         end
         // ---------------------------------------------------------------
         "cpupost": begin
            // review 2 (m-A): the D-cache's 32 B writes to lines the L2 does
            // not hold (posted: nothing allocates during the walk), back to
            // back or 10 cycles apart, during a WBINV walk with a dirty line
            // in every set: BUSY must drop within 4 x SETS requests
            int gaps[2] = '{0, 10};
            foreach (gaps[g]) begin
               int nr = 0;
               for (int s = 0; s < SETS; s++) xact(1, 0, 1, L(s % 3, s), 0, rnd256(), 0, 0, 0);
               ctrl(2'b11);
               while (busy && nr < 4 * SETS) begin
                  xact(1, 0, 1, L(5 + g, far_set(nr)) + 32 * (nr % 2), 0, rnd256(), 0, 0, 0);
                  nr++;
                  repeat (gaps[g]) @(negedge clk);
               end
               if (busy) fail($sformatf("cpupost gap %0d: BUSY after %0d requests (walk at set %0d of %0d)",
                                        gaps[g], nr, dut.r_walk_set, SETS));
               $display("TB_L2 cpupost gap %0d: walk done after %0d requests", gaps[g], nr);
               wait_idle(); ctrl(2'b11); wait_idle(); check_ddr($sformatf("cpupost gap %0d", gaps[g]));
            end
         end
         // ---------------------------------------------------------------
         "rtwalk": begin
            // review 2 (m-A): VGA requests the bus for the whole walk (its
            // request never drops) with reads back to back — hits, misses,
            // sector-0 stream fills — and no request needs a buffer slot:
            // only the walk waits on the buffer, so only its own drains can
            // move it.  BUSY must drop within 8 x SETS requests with VGA
            // still on the bus (on c9cadea the walk stopped at its first
            // full buffer); a read must not start before the walk's drain
            int nr = 0;
            for (int s = 0; s < SETS; s++) xact(1, 0, 1, L(s % 3, s) + 32 * (s % 2), 0, rnd256(), 0, 0, 0);
            for (int s = 0; s < SETS; s += 8) xact(0, 0, 1, L(4, s), 0, 0, 0, 1, 0);   // clean lines: VGA hits
            @(negedge clk); rt_req = 1;
            // (DDR beats spread out: a stream fill's tail then often ends
            // while the next read waits, the case the priority decides)
            dut.mig_7series_0.jit = 40;
            ctrl(2'b11);
            while (busy && nr < 8 * SETS) begin
               int k = $urandom_range(3, 0);
               if (k == 0)      xact(0, 1, 1, L(4, 8 * $urandom_range(SETS / 8 - 1, 0)), 0, 0, 0, 0, 0);
               else if (k == 1) xact(0, 1, 0, L(8, $urandom_range(SETS - 1, 0)) + 16 * $urandom_range(3, 0), 0, 0, 0, 0, 0);
               else             xact(0, 1, 1, L(6 + nr % 2, nr % SETS), 0, 0, 0, 0, 0);   // sector 0: stream fills
               nr++;
            end
            dut.mig_7series_0.jit = 0;
            if (busy) fail($sformatf("rtwalk: BUSY after %0d requests with VGA on the bus (walk at set %0d)",
                                     nr, dut.r_walk_set));
            $display("TB_L2 rtwalk: walk done after %0d requests, %0d drains before a waiting read", nr, wk_prio);
            if (wk_prio == 0) fail("rtwalk: no read waited for the walk's drain (priority untested)");
            @(negedge clk); rt_req = 0;
            wait_idle(); check_ddr("rtwalk");
         end
         // ---------------------------------------------------------------
         "wkset": begin
            // review 2 (m-A): a DMA master writing into the very set the walk
            // waits on (every way dirty), re-dirtying a way the walk already
            // copied — chasing the walk from set to set.  Each set must be
            // copied at most once a way, so BUSY must drop within
            // 2 x SETS x WAYS requests and the walk write at most
            // SETS x WAYS lines
            int nr = 0, n0, t0;
            for (int k = 0; k < `L2_WAYS; k++) for (int s = 0; s < SETS; s++) begin
               xact(1, 0, 1, L(k, s), 0, rnd256(), 0, 0, 0);
               xact(1, 0, 1, L(k, s) + 32, 0, rnd256(), 0, 0, 0);
            end
            repeat (200) @(negedge clk);
            n0 = evc[WR_BEAT];
            ctrl(2'b11);
            t0 = int'($time / 10);
            tenure = 1;
            while (busy && nr < 2 * SETS * `L2_WAYS) begin
               int s = int'(dut.r_walk_set);
               xact(1, 1, 0, L(nr % 2, s) + 16 * (nr % 4), 0, rnd256(), 16'(nr * 13), 0, 0);
               nr++;
               if (nr % 8 == 0) begin dma = 0; @(negedge clk); dma = 1; end
            end
            tenure = 0; dma = 0;
            if (busy) fail($sformatf("wkset: BUSY after %0d requests (walk at set %0d)", nr, dut.r_walk_set));
            $display("TB_L2 wkset: walk done after %0d requests, %0d cycles, %0d write beats (whole cache %0d)", nr,
                     int'($time / 10) - t0, evc[WR_BEAT] - n0, SETS * `L2_WAYS * 8);
            wait_idle(); ctrl(2'b11); wait_idle(); check_ddr("wkset");
         end
         // ---------------------------------------------------------------
         "strmedge": begin
            // review 2 (m-C): a write accepted on the very edge the stream
            // buffer captures sector 1 (the fill's last beat) must leave it
            // empty.  The write is timed at every offset from the read's
            // ready over the fill's tail, so one of them lands on that edge
            int same = 0;
            // (DMA writes: posted, so the line stays out of the L2 and the
            // sector-1 read below can only be answered by the stream buffer
            // or DDR)
            for (int r = 0; r < 32; r++) begin
               logic [31:0] b = L(7 + r % 3, 100 + r);
               xact(0, 1, 1, b, 0, 0, 0, 0, 0);                  // sector 0: the stream fill
               repeat (r % 16) @(negedge clk);
               xact(1, 1, 0, b + 32 + 16 * (r % 2), 0, rnd256(), 16'(r * 4099), 0, 0);
               while (dut.e_st != 2'd0) @(negedge clk);
               xact(0, 1, 1, b + 32, 0, 0, 0, 0, 0);             // sector 1: must see the write
            end
            same = strm_edge;
            if (same == 0) fail("strmedge: no write accepted on the capture edge (test vacuous)");
            $display("TB_L2 strmedge: %0d writes on the capture edge", same);
            ctrl(2'b11); wait_idle(); check_ddr("strmedge");
         end
         // ---------------------------------------------------------------
         default: begin
            // random: all masters, all kinds, a few hot sets (conflicts),
            // spec right and wrong, L2_CTRL writes at random, the VGA request
            rt_rand = 1;
            void'($value$plusargs("RT=%d", rt_rand));
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
                  // (a walk a random write started may be running: the
                  // new one must not be ignored)
                  wait_idle(); ctrl(2'b11); wait_idle(); check_ddr($sformatf("random #%0d", i));
               end else begin
                  repeat ($urandom_range(30, 0)) @(negedge clk);
               end
            end
            wait_idle(); ctrl(2'b11); wait_idle(); ctrl(2'b01);
            check_ddr("random end");
         end
      endcase
      repeat (20) @(negedge clk);
      if (n_rdy != n_req) fail($sformatf("%0d readies for %0d requests", n_rdy, n_req));
      if (rt_bad != 0) fail($sformatf("%0d drains started while VGA / DMA held the bus with no request waiting on the buffer", rt_bad));
      if (wk_prio_bad != 0) fail($sformatf("%0d demand reads started before the drain the walk waited on", wk_prio_bad));
      $display("TB_L2 ev i_hit=%0d i_miss=%0d d_hit=%0d d_miss=%0d w_hit=%0d w_miss=%0d rd_beat=%0d wr_beat=%0d wbfull=%0d dma_hit=%0d dma_miss=%0d wbm_wait=%0d spec_used=%0d dw_seen=%0d rt_need=%0d rt_bad=%0d",
               evc[0], evc[1], evc[2], evc[3], evc[4], evc[5], evc[6], evc[7], evc[8], evc[9], evc[10],
               wbm_wait, spec_used, dw_seen, rt_need, rt_bad);
      if (fails == 0) $display("TB_L2 PASS %s", test);
      else            $display("TB_L2 FAIL %s %0d errors: %s", test, fails, why);
      $finish;
   end

   // dmapost / cpupost: a set away from the one the walk is in (a write to
   // that set waits for the walk's copy, which would hide the slot race)
   function automatic int far_set(input int nr);
      return (int'(dut.r_walk_set) + SETS / 2 + nr % 64) % SETS;
   endfunction
   function automatic bit d_of(input int r);
      return r % 2;
   endfunction

   initial begin
      #500ms;
      $display("TB_L2 FAIL timeout");
      $finish;
   end
endmodule

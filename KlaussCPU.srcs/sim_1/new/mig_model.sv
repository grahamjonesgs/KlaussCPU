//===========================================================================
// mig_model — behavioural mig_7series_0 (2:1 PHY, 64-bit UI, BL8 = two beats)
// for Verilator benches (tb_soc, tb_l2).  HW_PERF_PLAN.md §6 (review 1: the
// DRAM timing model).
//
// The UI as the real core presents it, Strict ordering: column commands
// (CAS) issue in acceptance order; a write's CAS needs its two wdf beats
// (data may lead the command, or follow it within 2 cycles — UG586, checked
// below); a read samples memory at its CAS, so it sees every earlier write,
// and returns its two beats in order, one a cycle.
//
// DRAM timing (the board's MIG: mig_c.prj, MT47H64M16HR-25E at tCK = 5 ns,
// DDR2-400, CL 3, WL 2, BL8; ui_clk = 10 ns = 2 tCK; datasheet ns rounded
// up to ui cycles):
//   T_RCD 2 (15 ns)   T_RP 2 (12.5)   T_RAS 4 (40)   T_RRD 1 (10)
//   T_FAW 5 (45: four activates)
//   CAS to CAS 2 (BL8 = 4 tCK of data bus, either direction)
//   write CAS to read CAS  T_WTR 4 (WL 2 + BL/2 4 + tWTR 7.5 ns = 8 tCK)
//   read CAS to write CAS  T_RTW 4 (RL 3 + BL/2 4 + 2 - WL 2 = 7 tCK)
//   read CAS to precharge  T_RTP 2 (BL/2 - 2 + max(tRTP, 2) = 4 tCK)
//   write CAS to precharge T_WR 5 (WL 2 + BL/2 4 + tWR 15 ns 3 = 9 tCK)
//   refresh: every T_REFI 780 (7.8 us) all banks are closed (each after its
//   own T_RAS / T_RTP / T_WR) and REF holds them T_RP + T_RFC 13 (127.5
//   ns) more; queued commands, reads already accepted included, wait.
// Address map ROW_BANK_COLUMN (app_addr = PA[27:1]): column app_addr[9:0],
// bank [12:10] (PA[13:11]), row [25:13] (PA[26:14]); app_addr[26] = PA[27]
// is the rank bit, ignored with RANKS = 1 (PA[27] aliases).
// Bank machines (BankMachineCnt 4): the first 4 queued commands of
// different banks precharge / activate ahead of the CAS order; a bank's row
// stays open after a CAS only while a queued command hits it (else it
// closes: auto-precharge), an approximation of the 7-series controller.
//
// MIGLAT (default 36: ~44 cycles from a D-cache miss to its critical dword
// through ddr2_control, the board's measured latency) is from a read's
// acceptance to its first beat when its bank is closed and nothing is
// queued: the activate (T_RCD) is inside it; a row hit is T_RCD sooner, a
// row conflict, a turnaround, a refresh or queued commands later.
//
// Plusargs: +MIGLAT=<n>, +MIGJIT=<n> (random 0..n more per read),
// +MIGRDY=<pct> (app_rdy / app_wdf_rdy drop at random), +MIGREF=<period>
// (default 780; 0 = no refresh), +MIGRFC=<n> (default 13), +MIGSEED=<n>,
// +MIGFREE=1 (the pre-review model: no bank / bus / turnaround timing,
// writes free — for comparisons only).
// Storage: the low 8 MiB and the top 1 MiB of the 128 MiB space, one 64-bit
// slot per beat (byte address / 16 * 2 + beat); elsewhere reads 0.
//===========================================================================
module mig_7series_0 (
    output [12:0] ddr2_addr, output [2:0] ddr2_ba, output ddr2_cas_n,
    output [0:0] ddr2_ck_n, output [0:0] ddr2_ck_p, output [0:0] ddr2_cke,
    output ddr2_ras_n, output ddr2_we_n, inout [15:0] ddr2_dq,
    inout [1:0] ddr2_dqs_n, inout [1:0] ddr2_dqs_p, output [0:0] ddr2_cs_n,
    output [1:0] ddr2_dm, output [0:0] ddr2_odt,
    output logic init_calib_complete,
    input  [26:0] app_addr, input [2:0] app_cmd, input app_en,
    input  [63:0] app_wdf_data, input app_wdf_end, input app_wdf_wren,
    output logic [63:0] app_rd_data, output logic app_rd_data_end,
    output logic app_rd_data_valid, output logic app_rdy, output logic app_wdf_rdy,
    input  app_sr_req, input app_ref_req, input app_zq_req,
    output app_sr_active, output app_ref_ack, output app_zq_ack,
    output ui_clk, output logic ui_clk_sync_rst,
    input  [7:0] app_wdf_mask,
    input  sys_clk_i, input sys_rst
);
   assign ui_clk = sys_clk_i;
   assign app_sr_active = 1'b0; assign app_ref_ack = 1'b0; assign app_zq_ack = 1'b0;
   assign ddr2_addr = '0; assign ddr2_ba = '0; assign ddr2_cas_n = 1'b1; assign ddr2_ck_n = '0;
   assign ddr2_ck_p = '0; assign ddr2_cke = '0; assign ddr2_ras_n = 1'b1; assign ddr2_we_n = 1'b1;
   assign ddr2_cs_n = '1; assign ddr2_dm = '0; assign ddr2_odt = '0;

   localparam int LO_BEATS = 8 * 1024 * 1024 / 8;
   localparam int HI_BEATS = 1024 * 1024 / 8;
   localparam logic [31:0] HI_BASE = 32'h07F0_0000;
   logic [63:0] mem_lo [LO_BEATS];
   logic [63:0] mem_hi [HI_BEATS];

   // DRAM timing, ui cycles (see the header)
   localparam int T_RCD = 2, T_RP = 2, T_RAS = 4, T_RRD = 1, T_FAW = 5;
   localparam int T_CCD = 2, T_WTR = 4, T_RTW = 4, T_RTP = 2, T_WR = 5;
   localparam int NBM = 4;              // bank machines
   localparam int QD  = 4;              // commands accepted ahead (app_rdy)

   int lat = 36, jit = 0, rdyp = 0, refp = 780, rfc = 13, seed = 1, free_m = 0;
   int n_rd = 0, n_wr = 0;           // completed BL8 commands
   // statistics: activates, row conflicts (a precharge for another row),
   // refreshes, reads within T_WTR + 2 of a write (row hits = n_rd + n_wr -
   // n_act)
   int n_act = 0, n_conf = 0, n_ref = 0, n_wtr = 0;
   longint cyc = 0;
   initial begin
      for (int i = 0; i < LO_BEATS; i++) mem_lo[i] = '0;
      for (int i = 0; i < HI_BEATS; i++) mem_hi[i] = '0;
      void'($value$plusargs("MIGLAT=%d", lat));
      void'($value$plusargs("MIGJIT=%d", jit));
      void'($value$plusargs("MIGRDY=%d", rdyp));
      void'($value$plusargs("MIGREF=%d", refp));
      void'($value$plusargs("MIGRFC=%d", rfc));
      void'($value$plusargs("MIGSEED=%d", seed));
      void'($value$plusargs("MIGFREE=%d", free_m));
      if (free_m) refp = 0;
      void'($urandom(seed));
      init_calib_complete = 1'b0;
      ui_clk_sync_rst = 1'b1;
      app_rdy = 1'b0; app_wdf_rdy = 1'b0;
      app_rd_data_valid = 1'b0; app_rd_data_end = 1'b0; app_rd_data = '0;
   end

   // beat slot of byte address a (16 B aligned burst) + beat b
   function automatic int f_slot(input logic [31:0] a, input int b, output bit hi);
      logic [31:0] ba = {a[31:4], 4'b0};
      hi = 0;
      if (ba < 32'h0080_0000) return int'(ba >> 3) + b;
      if (ba >= HI_BASE && ba < 32'h0800_0000) begin hi = 1; return int'((ba - HI_BASE) >> 3) + b; end
      return -1;
   endfunction
   function automatic logic [63:0] f_rd(input logic [31:0] a, input int b);
      bit hi; int s = f_slot(a, b, hi);
      if (s < 0) return '0;
      return hi ? mem_hi[s] : mem_lo[s];
   endfunction
   task automatic t_wr(input logic [31:0] a, input int b, input logic [63:0] d, input logic [7:0] m);
      bit hi; int s = f_slot(a, b, hi);
      logic [63:0] cur;
      if (s < 0) begin $display("MIG: WARN write outside model %08x", a); return; end
      cur = hi ? mem_hi[s] : mem_lo[s];
      for (int i = 0; i < 8; i++) if (!m[i]) cur[i*8 +: 8] = d[i*8 +: 8];
      if (hi) mem_hi[s] = cur; else mem_lo[s] = cur;
   endtask

   // command queue (Strict: CAS in order)
   typedef struct { bit rd; logic [31:0] a; int bank; int row; } cmd_t;
   cmd_t cq [$];
   typedef struct { logic [63:0] d; logic [7:0] m; } wb_t;
   wb_t  wq [$];
   typedef struct { longint t; logic [63:0] d; bit last; } rb_t;
   rb_t  rq [$];
   longint last_rt = 0;

   // bank state
   bit     b_open [8];
   int     b_row  [8];
   longint b_act  [8];                  // its activate
   longint b_pre_ok [8];                // earliest precharge (T_RAS / T_RTP / T_WR)
   longint b_free [8];                  // closed: earliest activate (after T_RP)
   longint last_cas = -100, last_rd_cas = -100, last_wr_cas = -100, last_act = -100;
   longint acts [$];                    // recent activates (T_FAW)
   bit     ref_due = 0;
   longint ref_end = 0;
   initial for (int b = 0; b < 8; b++) begin
      b_open[b] = 0; b_row[b] = 0; b_act[b] = -100; b_pre_ok[b] = -100; b_free[b] = 0;
   end

   // UG586: write data follows its command by at most two cycles (cycles
   // with app_wdf_rdy low excused) — counts of accepted commands / beats
   longint wcmd_n = 0, wbeat_n = 0;
   typedef struct { longint need; int slack; } wdl_t;
   wdl_t   wdl [$];

   function automatic bit f_act_ok();
      if (cyc < ref_end || ref_due) return 0;
      if (cyc < last_act + T_RRD) return 0;
      if (acts.size() >= 4 && cyc < acts[acts.size() - 4] + T_FAW) return 0;
      return 1;
   endfunction

   always @(posedge sys_clk_i) begin
      cyc++;
      if (cyc == 4) ui_clk_sync_rst <= 1'b0;
      if (cyc == 10) init_calib_complete <= 1'b1;
      // accept (on this edge) what was offered with rdy high; app_addr[26]
      // (the rank bit, PA[27]) is ignored
      if (app_en && app_rdy) begin
         logic [31:0] a = {5'b0, app_addr[25:0], 1'b0};
         cq.push_back('{app_cmd == 3'b001, a, int'(app_addr[12:10]), int'(app_addr[25:13])});
         if (app_cmd != 3'b001) begin
            wcmd_n++;
            if (wbeat_n < 2 * wcmd_n) wdl.push_back('{2 * wcmd_n, 2});
         end
      end
      if (app_wdf_wren && app_wdf_rdy) begin
         wq.push_back('{app_wdf_data, app_wdf_mask});
         wbeat_n++;
      end
      while (wdl.size() > 0 && wbeat_n >= wdl[0].need) void'(wdl.pop_front());
      if (wdl.size() > 0) begin
         if (app_wdf_rdy) wdl[0].slack--;    // (a cycle the MIG could take data)
         if (wdl[0].slack < 0)
            $fatal(1, "MIG: write data more than 2 cycles after its command (UG586), cycle %0d", cyc);
      end

      if (free_m) begin
         // the pre-review model: one command a cycle, writes free
         if (cq.size() > 0) begin
            if (cq[0].rd) begin
               longint t = cyc + lat + (jit > 0 ? $urandom_range(jit, 0) : 0);
               if (t < last_rt + 2) t = last_rt + 2;
               last_rt = t;
               rq.push_back('{t,     f_rd(cq[0].a, 0), 1'b0});
               rq.push_back('{t + 1, f_rd(cq[0].a, 1), 1'b1});
               n_rd++;
               void'(cq.pop_front());
            end else if (wq.size() >= 2) begin
               t_wr(cq[0].a, 0, wq[0].d, wq[0].m);
               t_wr(cq[0].a, 1, wq[1].d, wq[1].m);
               void'(wq.pop_front()); void'(wq.pop_front());
               n_wr++;
               void'(cq.pop_front());
            end
         end
      end else begin
         // ---- refresh: close every bank, then REF ----
         if (refp > 0 && (cyc % refp) == 0) ref_due = 1;
         if (ref_due) begin
            bit all = 1;
            for (int b = 0; b < 8; b++) if (b_open[b]) begin
               if (cyc >= b_pre_ok[b]) begin b_open[b] = 0; b_free[b] = cyc + T_RP; end
               else all = 0;
            end
            if (all) begin
               longint t0 = cyc;
               for (int b = 0; b < 8; b++) if (b_free[b] > t0) t0 = b_free[b];
               ref_end = t0 + rfc;
               ref_due = 0;
               n_ref++;
            end
         end
         // ---- bank machines: precharge / activate for the first NBM
         //      commands of distinct banks ----
         begin
            bit used [8];
            int k = 0;
            for (int b = 0; b < 8; b++) used[b] = 0;
            for (int i = 0; i < cq.size() && k < NBM; i++) begin
               int b = cq[i].bank;
               if (used[b]) continue;
               used[b] = 1; k++;
               if (b_open[b] && b_row[b] == cq[i].row) continue;           // row hit
               if (b_open[b]) begin                                         // conflict
                  if (cyc >= b_pre_ok[b] && !ref_due) begin
                     b_open[b] = 0; b_free[b] = cyc + T_RP;
                     n_conf++;
                  end
               end else if (cyc >= b_free[b] && f_act_ok()) begin           // activate
                  b_open[b] = 1; b_row[b] = cq[i].row; b_act[b] = cyc;
                  b_pre_ok[b] = cyc + T_RAS;
                  last_act = cyc; acts.push_back(cyc);
                  if (acts.size() > 4) void'(acts.pop_front());
                  n_act++;
               end
            end
         end
         // ---- the head's CAS ----
         if (cq.size() > 0 && !ref_due) begin
            int b = cq[0].bank;
            bit rd = cq[0].rd;
            if (b_open[b] && b_row[b] == cq[0].row && cyc >= b_act[b] + T_RCD &&
                cyc >= last_cas + T_CCD &&
                (rd ? cyc >= last_wr_cas + T_WTR : cyc >= last_rd_cas + T_RTW) &&
                (rd || wq.size() >= 2)) begin
               bit keep = 0;
               if (rd && last_wr_cas > last_rd_cas && cyc < last_wr_cas + T_WTR + 2) n_wtr++;
               if (rd) begin
                  longint t = cyc + (lat - T_RCD) + (jit > 0 ? $urandom_range(jit, 0) : 0);
                  if (t < last_rt + 2) t = last_rt + 2;
                  last_rt = t;
                  rq.push_back('{t,     f_rd(cq[0].a, 0), 1'b0});
                  rq.push_back('{t + 1, f_rd(cq[0].a, 1), 1'b1});
                  n_rd++;
                  last_rd_cas = cyc;
                  if (cyc + T_RTP > b_pre_ok[b]) b_pre_ok[b] = cyc + T_RTP;
               end else begin
                  t_wr(cq[0].a, 0, wq[0].d, wq[0].m);
                  t_wr(cq[0].a, 1, wq[1].d, wq[1].m);
                  void'(wq.pop_front()); void'(wq.pop_front());
                  n_wr++;
                  last_wr_cas = cyc;
                  if (cyc + T_WR > b_pre_ok[b]) b_pre_ok[b] = cyc + T_WR;
               end
               last_cas = cyc;
               void'(cq.pop_front());
               // the row stays open only while a queued command hits it
               for (int i = 0; i < cq.size(); i++)
                  if (cq[i].bank == b && cq[i].row == b_row[b]) keep = 1;
               if (!keep) begin
                  // auto-precharge at b_pre_ok, then T_RP
                  b_open[b] = 0;
                  b_free[b] = b_pre_ok[b] + T_RP;
               end
            end
         end
      end
      // return read beats
      app_rd_data_valid <= 1'b0; app_rd_data_end <= 1'b0;
      if (rq.size() > 0 && rq[0].t <= cyc) begin
         app_rd_data_valid <= 1'b1;
         app_rd_data       <= rq[0].d;
         app_rd_data_end   <= rq[0].last;
         void'(rq.pop_front());
      end
      // ready for the next edge
      app_rdy     <= init_calib_complete && cq.size() < (free_m ? 8 : QD) &&
                     !(rdyp > 0 && $urandom_range(99, 0) < rdyp);
      app_wdf_rdy <= init_calib_complete && wq.size() < 16 &&
                     !(rdyp > 0 && $urandom_range(99, 0) < rdyp);
   end
endmodule

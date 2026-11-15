//===========================================================================
// mig_model — behavioural mig_7series_0 (2:1 PHY, 64-bit UI, BL8 = two beats)
// for Verilator benches (tb_soc, tb_l2).  HW_PERF_PLAN.md §6.
//
// The UI as the real core presents it, Strict ordering: commands complete
// in acceptance order; a write applies when its two wdf beats are in (data
// may lead the command, or follow it, as ddr2_control and l2_cache do); a
// read samples memory when it reaches the head of the queue, so it sees
// every earlier write, and returns its two beats MIGLAT cycles later (plus a
// random 0..MIGJIT), in order, one beat a cycle.  app_rdy / app_wdf_rdy drop
// at random (MIGRDY %) and while the queue is deep; MIGREF > 0 stalls the UI
// for 30 cycles every MIGREF cycles (refresh-like).
//
// Plusargs: +MIGLAT=<n> (default 36: ~44 cycles from a D-cache miss to its
// critical dword through ddr2_control, the board's measured latency),
// +MIGJIT=<n>, +MIGRDY=<pct>, +MIGREF=<period>, +MIGSEED=<n>.
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

   int lat = 36, jit = 0, rdyp = 0, refp = 0, seed = 1;
   int n_rd = 0, n_wr = 0;           // completed BL8 commands
   longint cyc = 0;
   initial begin
      for (int i = 0; i < LO_BEATS; i++) mem_lo[i] = '0;
      for (int i = 0; i < HI_BEATS; i++) mem_hi[i] = '0;
      void'($value$plusargs("MIGLAT=%d", lat));
      void'($value$plusargs("MIGJIT=%d", jit));
      void'($value$plusargs("MIGRDY=%d", rdyp));
      void'($value$plusargs("MIGREF=%d", refp));
      void'($value$plusargs("MIGSEED=%d", seed));
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

   // command queue (Strict: in order)
   typedef struct { bit rd; logic [31:0] a; } cmd_t;
   cmd_t cq [$];
   typedef struct { logic [63:0] d; logic [7:0] m; } wb_t;
   wb_t  wq [$];
   typedef struct { longint t; logic [63:0] d; bit last; } rb_t;
   rb_t  rq [$];
   longint last_rt = 0;
   int stall = 0;

   always @(posedge sys_clk_i) begin
      cyc++;
      if (cyc == 4) ui_clk_sync_rst <= 1'b0;
      if (cyc == 10) init_calib_complete <= 1'b1;
      // accept (on this edge) what was offered with rdy high
      if (app_en && app_rdy) cq.push_back('{app_cmd == 3'b001, {4'b0, app_addr, 1'b0}});
      if (app_wdf_wren && app_wdf_rdy) wq.push_back('{app_wdf_data, app_wdf_mask});
      // execute the queue head (one command a cycle)
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
      // return read beats
      app_rd_data_valid <= 1'b0; app_rd_data_end <= 1'b0;
      if (rq.size() > 0 && rq[0].t <= cyc) begin
         app_rd_data_valid <= 1'b1;
         app_rd_data       <= rq[0].d;
         app_rd_data_end   <= rq[0].last;
         void'(rq.pop_front());
      end
      // ready for the next edge
      if (refp > 0 && (cyc % refp) == 0) stall = 30;
      if (stall > 0) stall--;
      app_rdy     <= init_calib_complete && stall == 0 && cq.size() < 8 &&
                     !(rdyp > 0 && $urandom_range(99, 0) < rdyp);
      app_wdf_rdy <= init_calib_complete && wq.size() < 16 &&
                     !(rdyp > 0 && $urandom_range(99, 0) < rdyp);
   end
endmodule

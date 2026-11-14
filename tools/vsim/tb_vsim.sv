//===========================================================================
// tb_vsim — lean Verilator/Icarus harness for pipeline_core (no Vivado).
//
// The memory / MMIO / timer-IRQ model mirrors sim_1/new/tb_pipeline_isa.sv
// (same bus handshake, ready-edge behaviour, line lookahead, UART capture,
// INT_MASK mirror, IRQ handler image), minus the debugger driver, so it
// compiles under open-source simulators.  Used by tools/vsim/run.py for
// directed tests of new RTL and for old-vs-new differential runs.
//
// Plusargs:
//   +IMAGE=<dword $readmemh file>   program (code at 0x20)
//   +TRACE=<file>   per-retire record: pc op, written reg file, store
//   +OUT=<file>     final state: park, regs, sp, flags, counters, memory
//   +LAT=<n>        DRAM latency (default 3)   +LATR=<n> random extra
//   +IRQ_PERIOD=<n> periodic timer IRQ (handler at 0x100000 counts at 0x180000)
//   +MAXC=<n>       cycle cap (default 2,000,000)
//===========================================================================
`timescale 1ns/1ps
module tb_vsim #(parameter bit DEBUG_EN = 1'b0, parameter bit EXC_EN = 1'b1,
                 parameter bit MPU_EN = 1'b1,    // (MPU_EN: only with +define+HAS_MPU)
                 parameter int TLB_N = 32,       // (TLB_N: only with +define+HAS_TLBN)
                 parameter bit TLB_WALK = 1'b1); // (TLB_WALK: only with +define+HAS_WALK)
   logic clk = 0, rst = 1, start = 0;
   logic [31:0]  start_pc = 32'h20;
   logic [31:0]  m_addr;
   logic         m_read_DV, m_write_DV;
   logic [63:0]  m_wdata, m_rdata = '0, m_rdata_next = '0;
   logic [7:0]   m_be;
   logic         m_zl = 1'b0;      // DCBZ: zero the whole 32 B line (HW_PERF_PLAN §1)
   logic         m_next_valid = 1'b0, m_ready = 1'b0;
   logic [7:0]   lcd_byte;  logic lcd_dc, lcd_dv, lcd_rst_n, lcd_rst_wr, bus_idle;
   logic [7:0]   perf_stall;
   logic         perf_br, perf_br_taken, perf_fbr_taken;
`ifdef HAS_PERF_IC
   logic [1:0]   perf_ic;
`endif
   // load-hit pipe model (+HIT=<percent>, default 0 = every load takes the
   // slow path): the address the core presents is answered the cycle after
   // with the CURRENT memory doubleword.  A store reaches the model's memory
   // on its ready edge; until then it is on the port, where the core's own
   // same-line block / store forwarding covers it, as with the real cache.
   logic [31:0]  h_addr, h_addr_q = '0;
   logic         h_ok = 1'b0;
   logic [63:0]  h_dword;
   int           hit_pct = 0;
   logic         irq_ready;
   logic [1:0]   irq_sel = 2'd0;
   logic [31:0]  irq_vector = 32'h0010_0000;
   logic         irq_ack;
   logic [1:0]   irq_ack_sel;
   logic [3:0]   int_mask_o;
   logic         mask_wr = 0;
   logic [3:0]   mask_wdata = '0;
   logic         ret_valid, ret_wr;
   logic [31:0]  ret_pc, ret_op, ret_wr_addr;
   logic [7:0]   ret_wr_be;
   logic [63:0]  ret_wr_raw;
   logic         parked;
   logic [2:0]   park_kind;
   logic [31:0]  park_pc, park_op;
   logic [63:0]  dbg_r [0:15];
   logic [31:0]  dbg_sp;
   klauss_pkg::flags_t dbg_flags;
   klauss_pkg::dbg_in_t  dbg_i = '0;
   klauss_pkg::dbg_out_t dbg_o;

   pipeline_core #(.DEBUG_EN(DEBUG_EN)
`ifdef HAS_EXC
      , .EXC_EN(EXC_EN)
`endif
`ifdef HAS_MPU
      , .MPU_EN(MPU_EN)
`endif
`ifdef HAS_TLBN
      , .TLB_N(TLB_N)
`endif
`ifdef HAS_WALK
      , .TLB_WALK(TLB_WALK)
`endif
   ) dut (
      .clk, .rst, .ce(1'b1), .start, .start_pc,
      .m_addr, .m_read_DV, .m_write_DV, .m_wdata, .m_be,
`ifdef HAS_DCBZ
      .m_zl,
`endif
      .m_rdata,
`ifdef HAS_RDATA_DRAM
      .m_rdata_dram(m_rdata),
`endif
`ifdef HAS_DCH
      .h_addr(h_addr), .h_ok(h_ok), .h_dword(h_dword),   // +HIT=<pct> models hits (default: none)
`endif
      .m_rdata_next, .m_next_valid, .m_ready,
      .lcd_byte, .lcd_dc, .lcd_dv, .lcd_rst_n, .lcd_rst_wr, .lcd_ready(1'b1),
      .irq_ready, .irq_sel, .irq_vector, .irq_ack, .irq_ack_sel, .int_mask_o,
      .mask_wr, .mask_wdata,
      .ret_valid, .ret_pc, .ret_op, .ret_wr, .ret_wr_addr, .ret_wr_be, .ret_wr_raw,
      .bus_idle, .perf_stall, .perf_br, .perf_br_taken, .perf_fbr_taken,
`ifdef HAS_PERF_IC
      .perf_ic,
`endif
      .parked, .park_kind, .park_pc, .park_op,
      .dbg_r, .dbg_sp, .dbg_flags, .dbg_i, .dbg_o);

   always #5 clk = ~clk;

   // ---------------- memory model: 8 MiB low + 1 MiB stack top -------------
   localparam int LO_DW = 1048576;
   localparam logic [31:0] HI_BASE = 32'h07F0_0000;
   localparam int HI_DW = 131072;
   logic [63:0] edw = 64'h0;          // the dword at 0xEFFF_FFF8 (+EDW=<hex>)
   logic [63:0] mem_lo [0:LO_DW-1];
   logic [63:0] mem_hi [0:HI_DW-1];

   function automatic logic [63:0] rd_dw(input logic [31:0] a);
      if (a[31:28] == 4'hF) begin
         if (a[27:16] == 12'h001 && (a[15:0] & 16'hFFF8) == 16'h0010) rd_dw = 64'h2;
         else if (a[27:16] == 12'h00F && (a[15:0] & 16'hFFF8) == 16'h0000) rd_dw = {60'b0, int_mask_o};
         else rd_dw = 64'h0;
      end
      else if (a[31:3] == 29'h1DFF_FFFF) rd_dw = edw;   // +EDW: code straddling into 0xF...
      else if (a < 32'(LO_DW*8)) rd_dw = mem_lo[a[31:3]];
      else if (a >= HI_BASE && a < 32'h0800_0000) rd_dw = mem_hi[(a - HI_BASE) >> 3];
      else rd_dw = 64'h0;
   endfunction

   int lat_fix = 3, lat_rand = 0;
   assign h_dword = rd_dw({h_addr_q[31:3], 3'b000});
   always @(posedge clk) begin
      h_addr_q <= h_addr;
      h_ok     <= (hit_pct > 0) && (h_addr[31:28] != 4'hF) && (($urandom % 100) < hit_pct);
   end
   logic        in_flight = 0;
   logic [31:0] q_addr;
   logic        q_wr;
   logic [63:0] q_wdata;
   logic [7:0]  q_be;
   logic        q_zl;
   longint      n_zl = 0;
   int          lat_cnt;
   int          uart_n = 0;
   string       uart_s = "";
   longint      n_rd = 0, n_wr = 0, n_mmio_wr = 0, n_mmio_rd = 0;

   task automatic apply_write(input logic [31:0] a, input logic [63:0] wd, input logic [7:0] be);
      logic [63:0] cur, nxt;
      if (a[31:28] == 4'hF) begin
         n_mmio_wr++;
         if (a[27:16] == 12'h001 && (a[15:0] & 16'hFFF8) == 16'h0000) begin
            uart_s = {uart_s, string'(wd[7:0])};
            uart_n++;
         end
         if (a[27:16] == 12'h00F && (a[15:0] & 16'hFFF8) == 16'h0000) begin
            mask_wr    <= 1'b1;
            mask_wdata <= wd[3:0];
         end
      end else begin
         cur = rd_dw(a);
         for (int i = 0; i < 8; i++) nxt[i*8 +: 8] = be[i] ? wd[i*8 +: 8] : cur[i*8 +: 8];
         if (a < 32'(LO_DW*8)) mem_lo[a[31:3]] = nxt;
         else if (a >= HI_BASE && a < 32'h0800_0000) mem_hi[(a - HI_BASE) >> 3] = nxt;
         else $display("TB: WARN write outside model at %08x", a);
      end
   endtask

   always @(posedge clk) begin
      m_ready <= 1'b0;
      mask_wr <= 1'b0;
      if (rst) begin
         in_flight <= 1'b0;
      end else if (!in_flight && !m_ready && (m_read_DV || m_write_DV)) begin
         in_flight <= 1'b1;
         q_addr <= m_addr;  q_wr <= m_write_DV;  q_wdata <= m_wdata;  q_be <= m_be;
         q_zl <= m_write_DV && m_zl;
         if (m_addr[31:28] == 4'hF) lat_cnt <= m_read_DV ? 2 : 1;
         else lat_cnt <= lat_fix + (lat_rand ? ($urandom % lat_rand) : 0);
      end else if (in_flight) begin
         if (lat_cnt > 1) lat_cnt <= lat_cnt - 1;
         else begin
            if (q_wr && q_zl) begin
               // DCBZ: the line (L = 32) — the core never sends one to MMIO
               if (q_addr[31:28] == 4'hF || q_addr[4:0] != 5'd0)
                  $display("TB: FATAL zero-line request at %08x", q_addr);
               for (int k = 0; k < 4; k++) apply_write({q_addr[31:5], 5'b0} + 32'(8 * k), 64'h0, 8'hFF);
               n_wr++;  n_zl++;
            end
            else if (q_wr) begin apply_write(q_addr, q_wdata, q_be); n_wr++; end
            else begin
               m_rdata      <= rd_dw(q_addr);
               m_rdata_next <= rd_dw({q_addr[31:3], 3'b000} + 32'd8);
               m_next_valid <= (q_addr[31:28] != 4'hF) && (q_addr[4:3] != 2'b11);
               n_rd++;
               if (q_addr[31:28] == 4'hF) n_mmio_rd++;
            end
            m_ready   <= 1'b1;
            in_flight <= 1'b0;
         end
      end
   end

   // ---------------- timer IRQ model (as tb_pipeline_isa) -----------------
   int     irq_period = 0;
   longint cyc = 0;
   int     irq_acks = 0;
   logic   timer_pending = 0;
   int     timer_cnt = 0;
   assign irq_ready = timer_pending && int_mask_o[0];
   always @(posedge clk) begin
      if (!rst) begin
         cyc <= cyc + 1;
         if (irq_period != 0) begin
            timer_cnt <= timer_cnt + 1;
            if (timer_cnt >= irq_period) begin timer_pending <= 1'b1; timer_cnt <= 0; end
         end
         if (irq_ack && irq_ack_sel == 2'd0) begin
            timer_pending <= 1'b0;
            irq_acks      <= irq_acks + 1;
         end
      end
   end

   // handler at 0x100000: count at 0x180000, callee-saved, IRET
   task automatic install_handler();
      mem_lo['h20000] = {32'h64000020, 32'h64000010}; // PUSH r1 / PUSH r2
      mem_lo['h20001] = {32'h00180000, 32'h8BD00100}; // SETR r1, 0x180000
      mem_lo['h20002] = {32'h57880220, 32'h5B000210}; // MEMREADRR r2,[r1] / INCR r2
      mem_lo['h20003] = {32'h64800200, 32'h5F000210}; // MEMSET64RR [r1],r2 / POP r2
      mem_lo['h20004] = {32'h65C00000, 32'h64800100}; // POP r1 / IRET
   endtask

   // ---------------- counters / trace ----------------------------------------
   longint icount = 0;
   always @(posedge clk) if (!rst && dbg_drv) begin
      dbg_i.resume <= 1'b0;  dbg_i.step <= 1'b0;
      if (parked && park_kind == 3'd4 && !dbg_i.resume) begin
         if (dbg_gap < 4) dbg_gap <= dbg_gap + 1;      // let the halt settle
         else begin
            dbg_gap <= 0;
            dbg_i.resume <= 1'b1;
            if (dbg_stepped < dbg_steps) begin dbg_i.step <= 1'b1; dbg_stepped <= dbg_stepped + 1; end
            else if (dbg_plain_done < dbg_plain) dbg_plain_done <= dbg_plain_done + 1;   // plain resume, bp stays
            else dbg_i.bp_en[0] <= 1'b0;                // done: run freely
         end
      end
   end
   longint exc_n = 0;          // exception entries completed (MMU)
`ifdef HAS_EXC
   always @(posedge clk) if (!rst && dut.sr_ent_we) exc_n++;
`endif
   // MMU_PLAN §11.8 walker (+define+HAS_WALK): walks started (wk_s), the
   // successful ones (wk_n), and the cycles from the faulting op's WB edge
   // to the successful walk's restart (wk_cyc)
   // wk_irq: an IRQ taken at the restart boundary right after a successful
   // walk (before the faulting instruction re-executes)
   longint wk_s = 0, wk_n = 0, wk_cyc = 0, wk_irq = 0;
   // wk_post / wk_sq / wk_pend: walks that start with a store still in
   // st_post / the store queue (sq_v) / either; wk_rd: walker reads issued;
   // wk_same: walks whose T read is the same doubleword as its D read
   longint sq_cyc = 0;      // cycles with a queued store (sq_v), whatever the core is doing
   longint wk_post = 0, wk_sq = 0, wk_pend = 0, wk_rd = 0, wk_same = 0;
   logic [31:0] wk_da = '0;
   logic wkr_q = 1'b0;
`ifdef HAS_WALK
   logic wk_q = 1'b0, wkf_q = 1'b0, ia_q = 1'b0;
   always @(posedge clk) if (!rst) begin
      wk_q <= dut.wk;
      ia_q <= dut.irq_active;
      // (wkf_q: a walk finished and nothing has dispatched since)
      if (dut.wk_fin) wkf_q <= 1'b1;
      else if (dut.id_valid) wkf_q <= 1'b0;
      if (wkf_q && dut.irq_active && !ia_q && !dut.exc_ent) wk_irq++;
      if (dut.wk && !wk_q) begin
         wk_s++;
         // a store still posted / queued when the miss reaches the walker
         // (the walk then waits for it to drain)
         if (dut.st_post) wk_post++;
         if (dut.sq_v)    wk_sq++;
         if (dut.st_post || dut.sq_v) wk_pend++;
      end
      // the walker's reads: D then T (wk_ph 0 / 1); the same doubleword?
      if (dut.wk && dut.irq_xc && m_read_DV && !wkr_q) begin
         wk_rd++;
         if (dut.wk_ph == 2'd0) wk_da <= m_addr;
         else if (dut.wk_ph == 2'd1 && m_addr[31:3] == wk_da[31:3]) wk_same++;
      end
      wkr_q <= dut.wk && dut.irq_xc && m_read_DV;
      if (dut.wk_fin) wk_n++;
      if (dut.wk) wk_cyc++;
      if (dut.sq_v) sq_cyc++;
   end
`endif
   // LCD strobes (a wrong-path LCD op must not send)
   longint lcd_n = 0, lcdrst_n = 0;
   always @(posedge clk) if (!rst) begin if (lcd_dv) lcd_n++; if (lcd_rst_wr) lcdrst_n++; end
   longint pst [0:7];
   longint n_br = 0, n_br_taken = 0, n_fbr_taken = 0, n_ic_miss = 0, n_ic_fill = 0;
   integer trace_f = 0;
   bit     trace_cyc = 0;      // +TRACECYC: stamp each retire with its cycle
   int     park_hold = 0, park_hold_n = 0;
   // a minimal debugger driver (DEBUG_EN builds): +DBG_BP0=<pc> halts at a
   // PC breakpoint, then +DBG_STEPS=<n> single steps (step + resume pulses,
   // each waiting for the re-halt), then a plain resume
   bit     dbg_drv = 0;
   int     dbg_steps = 0, dbg_stepped = 0, dbg_gap = 0;
   int     dbg_plain = 0, dbg_plain_done = 0;     // +DBG_PLAIN=<n>: n plain resumes with the breakpoint still set
   // debug halts (park_kind 4): count, and each one's pc:cause (first 8)
   int     dh_n = 0;
   string  dh_s [0:7];
   logic   dh_q = 1'b0, dh_p = 1'b0;
   always @(posedge clk) if (!rst) begin
      dh_q <= parked && park_kind == 3'd4;
      dh_p <= parked && park_kind == 3'd4 && !dh_q;      // one cycle after the halt: pc / cause settled
      if (dh_p) begin
         if (dh_n < 8) dh_s[dh_n] = $sformatf("%08x:%0d", park_pc, dbg_o.cause);
         dh_n++;
      end
   end
   string  out_name, trace_name, img;
   longint maxc = 2000000;

   longint n_post = 0, n_nopost_dram = 0, n_ras_pred = 0, n_ras_miss = 0;
   initial for (int i = 0; i < 8; i++) pst[i] = 0;
`ifdef HAS_RAS
   // 2a coverage: predicted returns completing in MEM / mispredicted ones
   always @(posedge clk) if (!rst) begin
      if (dut.mem_valid && dut.mem_pred && !dut.mem_busy) n_ras_pred++;
      if (dut.ras_miss) n_ras_miss++;
   end
`endif
`ifdef HAS_POST
   // 1c coverage: stores that posted / DRAM stores that took the slow path
   always @(posedge clk) if (!rst) begin
      if (dut.mem_xc == 2'd1 && dut.st_post_ok && !dut.w_mrdy) n_post++;
      if (dut.mem_xc == 2'd1 && dut.mem_is_write && !dut.st_post_ok && dut.w_mrdy &&
          dut.mem_iaddr[31:28] != 4'hF) n_nopost_dram++;
   end
`endif
   always @(posedge clk) if (!rst && start == 0) begin
      for (int i = 0; i < 8; i++) if (perf_stall[i]) pst[i]++;
      if (perf_br) n_br++;
      if (perf_br_taken && perf_br) n_br_taken++;
      if (perf_fbr_taken) n_fbr_taken++;
`ifdef HAS_PERF_IC
      if (perf_ic[0]) n_ic_miss++;
      if (perf_ic[1]) n_ic_fill++;
`endif
   end

   always @(negedge clk) begin
      if (!rst && ret_valid) begin
         icount++;
         if (trace_f != 0) begin
            if (trace_cyc) $fwrite(trace_f, "t=%0d ", cyc);
            $fwrite(trace_f, "pc=%08x op=%08x", ret_pc, ret_op);
            for (int i = 0; i < 16; i++) $fwrite(trace_f, " %016x", dbg_r[i]);
            $fwrite(trace_f, " sp=%08x f=%b%b%b%b", dbg_sp, dbg_flags.zero, dbg_flags.sign,
                    dbg_flags.carry, dbg_flags.overflow);
            if (ret_wr) $fwrite(trace_f, " wr=%08x/%02x/%016x", ret_wr_addr, ret_wr_be, ret_wr_raw);
            $fwrite(trace_f, "\n");
         end
      end
      // the SoC takes the bus only once the core's port is idle (a posted
      // store may still be draining when the core parks)
      // (+PARK_HOLD=<n>: keep running n cycles first, so anything that still
      // moves after a park — a re-taken IRQ, a dispatch — trips the core's
      // assertions)
      if (!rst && parked && park_kind != 3'd3 && !(dbg_drv && park_kind == 3'd4) && bus_idle && !in_flight) begin
         if (park_hold_n >= park_hold) done("park");
         park_hold_n++;
      end
      if (cyc > maxc) done("timeout");
   end

   task automatic done(input string why);
      integer f;
      f = $fopen(out_name, "w");
      $fdisplay(f, "end=%s kind=%0d pc=%08x op=%08x", why, park_kind, park_pc, park_op);
      $fdisplay(f, "dbg_cause=%0d watch_addr=%08x", dbg_o.cause, dbg_o.watch_addr);
      $fdisplay(f, "debug_en=%0d", DEBUG_EN);
      $fdisplay(f, "dh_n=%0d", dh_n);
      for (int i = 0; i < 8 && i < dh_n; i++) $fdisplay(f, "dh%0d=%s", i, dh_s[i]);
      $fdisplay(f, "cycles=%0d instrs=%0d irqs=%0d", cyc, icount, irq_acks);
      for (int i = 0; i < 16; i++) $fdisplay(f, "r%0d=%016x", i, dbg_r[i]);
      $fdisplay(f, "sp=%08x", dbg_sp);
      $fdisplay(f, "flags=%b%b%b%b", dbg_flags.zero, dbg_flags.sign, dbg_flags.carry, dbg_flags.overflow);
      $fdisplay(f, "int_mask=%x", int_mask_o);
      for (int i = 0; i < 8; i++) $fdisplay(f, "perf%0d=%0d", i, pst[i]);
      $fdisplay(f, "br=%0d br_taken=%0d fbr_taken=%0d ic_miss=%0d ic_fill=%0d", n_br, n_br_taken,
                n_fbr_taken, n_ic_miss, n_ic_fill);
      $fdisplay(f, "bus_rd=%0d bus_wr=%0d mmio_wr=%0d mmio_rd=%0d", n_rd, n_wr, n_mmio_wr, n_mmio_rd);
      $fdisplay(f, "posted=%0d nopost_dram=%0d zero_lines=%0d", n_post, n_nopost_dram, n_zl);
      $fdisplay(f, "ras_pred=%0d ras_miss=%0d", n_ras_pred, n_ras_miss);
      $fdisplay(f, "uart_n=%0d lcd_n=%0d lcdrst_n=%0d", uart_n, lcd_n, lcdrst_n);
`ifdef HAS_EXC
      $fdisplay(f, "ecause=%03x eaddr=%08x um=%0d exc_n=%0d", dut.ecause, dut.eaddr, dut.umode, exc_n);
`endif
`ifdef HAS_WALK
      $fdisplay(f, "wk_s=%0d wk_n=%0d wk_cyc=%0d wk_irq=%0d", wk_s, wk_n, wk_cyc, wk_irq);
      $fdisplay(f, "wk_post=%0d wk_sq=%0d wk_pend=%0d wk_rd=%0d wk_same=%0d sq_cyc=%0d", wk_post, wk_sq, wk_pend, wk_rd, wk_same, sq_cyc);
`endif
      // data window 0x10000..0x10800 (dword 0x2000..0x20FF) and IRQ counter
      for (int i = 'h2000; i < 'h2100; i++)
         if (mem_lo[i] != 64'h0) $fdisplay(f, "m%08x=%016x", i*8, mem_lo[i]);
      $fdisplay(f, "m%08x=%016x", 32'h180000, mem_lo['h30000]);
      $fclose(f);
      if (trace_f != 0) $fclose(trace_f);
      $display("TB_VSIM: %s after %0d cycles, %0d instructions", why, cyc, icount);
      $finish;
   endtask

   initial begin
      for (int i = 0; i < LO_DW; i++) mem_lo[i] = 64'h0;
      for (int i = 0; i < HI_DW; i++) mem_hi[i] = 64'h0;
      if (!$value$plusargs("IMAGE=%s", img)) begin $display("TB: need +IMAGE"); $finish; end
      if (!$value$plusargs("OUT=%s", out_name)) out_name = "tb_vsim.out";
      if ($value$plusargs("TRACE=%s", trace_name)) trace_f = $fopen(trace_name, "w");
      trace_cyc = $test$plusargs("TRACECYC");
      begin
         logic [31:0] bpa;
         if ($value$plusargs("DBG_BP0=%h", bpa)) begin
            dbg_drv = 1;  dbg_i.bp_en[0] = 1'b1;  dbg_i.bp_addr[0] = bpa;
            void'($value$plusargs("DBG_STEPS=%d", dbg_steps));
            void'($value$plusargs("DBG_PLAIN=%d", dbg_plain));
         end
      end
      void'($value$plusargs("PARK_HOLD=%d", park_hold));
      void'($value$plusargs("LAT=%d", lat_fix));
      void'($value$plusargs("LATR=%d", lat_rand));
      void'($value$plusargs("IRQ_PERIOD=%d", irq_period));
      void'($value$plusargs("MAXC=%d", maxc));
      void'($value$plusargs("EDW=%h", edw));
      void'($value$plusargs("HIT=%d", hit_pct));
      // debugger state without a debugger driver (DEBUG_EN builds only)
      if ($test$plusargs("DBG_ATTACHED")) dbg_i.attached = 1'b1;
      // watchpoint 0 on a dword (read / write), without a debugger driver
      begin
         logic [31:0] wpa;
         if ($value$plusargs("WP0RD=%h", wpa)) begin
            dbg_i.wp_en[0] = 1'b1; dbg_i.wp_rd[0] = 1'b1; dbg_i.wp_dw[0] = wpa[31:3]; dbg_i.wp_be[0] = 8'hFF;
         end
         if ($value$plusargs("WP0WR=%h", wpa)) begin
            dbg_i.wp_en[0] = 1'b1; dbg_i.wp_wr[0] = 1'b1; dbg_i.wp_dw[0] = wpa[31:3]; dbg_i.wp_be[0] = 8'hFF;
         end
      end
`ifdef HAS_EXC
      if ($test$plusargs("DBG_EXCPASS")) dbg_i.exc_pass = 1'b1;
`endif
      $readmemh(img, mem_lo);
      // (+NO_IRQ_HANDLER: the image supplies its own handler at 0x100000)
      if (!$test$plusargs("NO_IRQ_HANDLER")) install_handler();
      repeat (4) @(posedge clk);
      rst <= 0;
      @(posedge clk); start <= 1;
      @(posedge clk); start <= 0;
   end
endmodule

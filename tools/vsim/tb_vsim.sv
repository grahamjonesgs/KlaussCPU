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
module tb_vsim;
   logic clk = 0, rst = 1, start = 0;
   logic [31:0]  start_pc = 32'h20;
   logic [31:0]  m_addr;
   logic         m_read_DV, m_write_DV;
   logic [63:0]  m_wdata, m_rdata = '0, m_rdata_next = '0;
   logic [7:0]   m_be;
   logic         m_next_valid = 1'b0, m_ready = 1'b0;
   logic [7:0]   lcd_byte;  logic lcd_dc, lcd_dv, lcd_rst_n, lcd_rst_wr, bus_idle;
   logic [7:0]   perf_stall;
   logic         perf_br, perf_br_taken, perf_fbr_taken;
`ifdef HAS_IC_MISS
   logic         perf_ic_miss;
`endif
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

   pipeline_core dut (
      .clk, .rst, .ce(1'b1), .start, .start_pc,
      .m_addr, .m_read_DV, .m_write_DV, .m_wdata, .m_be,
      .m_rdata, .m_rdata_next, .m_next_valid, .m_ready,
      .lcd_byte, .lcd_dc, .lcd_dv, .lcd_rst_n, .lcd_rst_wr, .lcd_ready(1'b1),
      .irq_ready, .irq_sel, .irq_vector, .irq_ack, .irq_ack_sel, .int_mask_o,
      .mask_wr, .mask_wdata,
      .ret_valid, .ret_pc, .ret_op, .ret_wr, .ret_wr_addr, .ret_wr_be, .ret_wr_raw,
      .bus_idle, .perf_stall, .perf_br, .perf_br_taken, .perf_fbr_taken,
`ifdef HAS_IC_MISS
      .perf_ic_miss,
`endif
      .parked, .park_kind, .park_pc, .park_op,
      .dbg_r, .dbg_sp, .dbg_flags, .dbg_i, .dbg_o);

   always #5 clk = ~clk;

   // ---------------- memory model: 8 MiB low + 1 MiB stack top -------------
   localparam int LO_DW = 1048576;
   localparam logic [31:0] HI_BASE = 32'h07F0_0000;
   localparam int HI_DW = 131072;
   logic [63:0] mem_lo [0:LO_DW-1];
   logic [63:0] mem_hi [0:HI_DW-1];

   function automatic logic [63:0] rd_dw(input logic [31:0] a);
      if (a[31:28] == 4'hF) begin
         if (a[27:16] == 12'h001 && (a[15:0] & 16'hFFF8) == 16'h0010) rd_dw = 64'h2;
         else if (a[27:16] == 12'h00F && (a[15:0] & 16'hFFF8) == 16'h0000) rd_dw = {60'b0, int_mask_o};
         else rd_dw = 64'h0;
      end
      else if (a < 32'(LO_DW*8)) rd_dw = mem_lo[a[31:3]];
      else if (a >= HI_BASE && a < 32'h0800_0000) rd_dw = mem_hi[(a - HI_BASE) >> 3];
      else rd_dw = 64'h0;
   endfunction

   int lat_fix = 3, lat_rand = 0;
   logic        in_flight = 0;
   logic [31:0] q_addr;
   logic        q_wr;
   logic [63:0] q_wdata;
   logic [7:0]  q_be;
   int          lat_cnt;
   int          uart_n = 0;
   string       uart_s = "";
   longint      n_rd = 0, n_wr = 0, n_mmio_wr = 0;

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
         if (m_addr[31:28] == 4'hF) lat_cnt <= m_read_DV ? 2 : 1;
         else lat_cnt <= lat_fix + (lat_rand ? ($urandom % lat_rand) : 0);
      end else if (in_flight) begin
         if (lat_cnt > 1) lat_cnt <= lat_cnt - 1;
         else begin
            if (q_wr) begin apply_write(q_addr, q_wdata, q_be); n_wr++; end
            else begin
               m_rdata      <= rd_dw(q_addr);
               m_rdata_next <= rd_dw({q_addr[31:3], 3'b000} + 32'd8);
               m_next_valid <= (q_addr[31:28] != 4'hF) && (q_addr[4:3] != 2'b11);
               n_rd++;
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
   longint pst [0:7];
   longint n_br = 0, n_br_taken = 0, n_fbr_taken = 0, n_ic_miss = 0;
   integer trace_f = 0;
   string  out_name, trace_name, img;
   longint maxc = 2000000;

   initial for (int i = 0; i < 8; i++) pst[i] = 0;
   always @(posedge clk) if (!rst && start == 0) begin
      for (int i = 0; i < 8; i++) if (perf_stall[i]) pst[i]++;
      if (perf_br) n_br++;
      if (perf_br_taken && perf_br) n_br_taken++;
      if (perf_fbr_taken) n_fbr_taken++;
`ifdef HAS_IC_MISS
      if (perf_ic_miss) n_ic_miss++;
`endif
   end

   always @(negedge clk) begin
      if (!rst && ret_valid) begin
         icount++;
         if (trace_f != 0) begin
            $fwrite(trace_f, "pc=%08x op=%08x", ret_pc, ret_op);
            for (int i = 0; i < 16; i++) $fwrite(trace_f, " %016x", dbg_r[i]);
            $fwrite(trace_f, " sp=%08x f=%b%b%b%b", dbg_sp, dbg_flags.zero, dbg_flags.sign,
                    dbg_flags.carry, dbg_flags.overflow);
            if (ret_wr) $fwrite(trace_f, " wr=%08x/%02x/%016x", ret_wr_addr, ret_wr_be, ret_wr_raw);
            $fwrite(trace_f, "\n");
         end
      end
      if (!rst && parked && park_kind != 3'd3) done("park");
      if (cyc > maxc) done("timeout");
   end

   task automatic done(input string why);
      integer f;
      f = $fopen(out_name, "w");
      $fdisplay(f, "end=%s kind=%0d pc=%08x op=%08x", why, park_kind, park_pc, park_op);
      $fdisplay(f, "cycles=%0d instrs=%0d irqs=%0d", cyc, icount, irq_acks);
      for (int i = 0; i < 16; i++) $fdisplay(f, "r%0d=%016x", i, dbg_r[i]);
      $fdisplay(f, "sp=%08x", dbg_sp);
      $fdisplay(f, "flags=%b%b%b%b", dbg_flags.zero, dbg_flags.sign, dbg_flags.carry, dbg_flags.overflow);
      $fdisplay(f, "int_mask=%x", int_mask_o);
      for (int i = 0; i < 8; i++) $fdisplay(f, "perf%0d=%0d", i, pst[i]);
      $fdisplay(f, "br=%0d br_taken=%0d fbr_taken=%0d ic_miss=%0d", n_br, n_br_taken, n_fbr_taken, n_ic_miss);
      $fdisplay(f, "bus_rd=%0d bus_wr=%0d mmio_wr=%0d", n_rd, n_wr, n_mmio_wr);
      $fdisplay(f, "uart_n=%0d", uart_n);
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
      void'($value$plusargs("LAT=%d", lat_fix));
      void'($value$plusargs("LATR=%d", lat_rand));
      void'($value$plusargs("IRQ_PERIOD=%d", irq_period));
      void'($value$plusargs("MAXC=%d", maxc));
      $readmemh(img, mem_lo);
      install_handler();
      repeat (4) @(posedge clk);
      rst <= 0;
      @(posedge clk); start <= 1;
      @(posedge clk); start <= 0;
   end
endmodule

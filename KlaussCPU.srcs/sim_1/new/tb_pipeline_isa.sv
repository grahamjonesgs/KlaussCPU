//===========================================================================
// tb_pipeline_isa — M5a golden-trace harness for the full-ISA pipeline core.
//
// Loads a real compiled program image (klausscc --mem-out $readmemh dword
// format: heap header at 0x0, code at 0x20), runs it on pipeline_core with
// the ideal M5a memory model, and writes:
//   - a per-retired-instruction trace in the EXACT klausscc --emulate --trace
//     line format (diff line-for-line against the emulator golden), and
//   - the captured UART byte stream (MMIO stores to 0xF001_0000).
//
// MMIO model mirrors klausscc/src/emulate.rs: only the UART decodes —
// STATUS (0xF001_0010) reads {rx_full=0, rx_empty=1, tx_busy=0} = 0x2,
// RX_DATA reads 0, everything else reads 0; only TX_DATA writes take effect.
//
// Plusargs: +IMAGE=<file.mem> +TRACE=<out> +UARTF=<out> +MAXI=<n> +ENTRY=<hex>
//===========================================================================
module tb_pipeline_isa;
   logic clk = 0, rst = 1, start = 0;
   logic [31:0]  start_pc = 32'h20;
   logic [31:0]  m_addr;
   logic         m_read_DV, m_write_DV;
   logic [63:0]  m_wdata, m_rdata, m_rdata_next;
   logic [7:0]   m_be;
   logic         m_next_valid, m_ready;
   logic [7:0]   lcd_byte;  logic lcd_dc, lcd_dv, lcd_rst_n;
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

   pipeline_core dut (.*, .lcd_ready(1'b1));

   always #5 clk = ~clk;

   // ---------------- memory model: 8 MiB low + 1 MiB stack top -------------
   localparam int LO_DW = 1048576;                 // 8 MiB
   localparam logic [31:0] HI_BASE = 32'h07F0_0000; // top 1 MiB (stack)
   localparam int HI_DW = 131072;
   logic [63:0] mem_lo [0:LO_DW-1];
   logic [63:0] mem_hi [0:HI_DW-1];

   function automatic logic [63:0] rd_dw(input logic [31:0] a);
      if (a[31:28] == 4'hF) begin
         if (a[27:16] == 12'h001) begin
            case (a[15:0] & 16'hFFF8)
               16'h0010: rd_dw = 64'h2;   // STATUS: rx_empty=1, tx_busy=0
               default:  rd_dw = 64'h0;   // RX_DATA / TX_DATA readback
            endcase
         end else rd_dw = 64'h0;
      end
      else if (a < 32'(LO_DW*8)) rd_dw = mem_lo[a[31:3]];
      else if (a >= HI_BASE && a < 32'h0800_0000) rd_dw = mem_hi[(a - HI_BASE) >> 3];
      else rd_dw = 64'h0;
   endfunction

   // ---------------- membus model: latency + ready handshake ---------------
   // Mirrors the cache/bus_splitter behavior the FSM sees: registered request
   // (DV held until ready), ready pulses 1 cycle with the data, MMIO reads
   // take 2 cycles / writes 1, DRAM takes +LAT (fixed or randomized), and the
   // read lookahead returns the next dword within the same 32 B line.
   int lat_fix = 3;
   int lat_rand = 0;
   logic        in_flight = 0;
   logic [31:0] q_addr;
   logic        q_wr;
   logic [63:0] q_wdata;
   logic [7:0]  q_be;
   int          lat_cnt;

   always @(posedge clk) begin
      m_ready <= 1'b0;
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
            if (q_wr) apply_write(q_addr, q_wdata, q_be);
            else begin
               m_rdata      <= rd_dw(q_addr);
               m_rdata_next <= rd_dw({q_addr[31:3], 3'b000} + 32'd8);
               m_next_valid <= (q_addr[31:28] != 4'hF) && (q_addr[4:3] != 2'b11);
            end
            m_ready   <= 1'b1;
            in_flight <= 1'b0;
         end
      end
   end

   integer uart_f, trace_f;
   longint icount = 0;
   longint maxi = 0;
   int     uart_n = 0;

   task automatic apply_write(input logic [31:0] a, input logic [63:0] wd, input logic [7:0] be);
      logic [63:0] cur, nxt;
      if (a[31:28] == 4'hF) begin
         if (a[27:16] == 12'h001 && (a[15:0] & 16'hFFF8) == 16'h0000) begin
            $fwrite(uart_f, "%c", wd[7:0]);
            uart_n++;
         end
      end else begin
         cur = rd_dw(a);
         for (int i = 0; i < 8; i++) nxt[i*8 +: 8] = be[i] ? wd[i*8 +: 8] : cur[i*8 +: 8];
         if (a < 32'(LO_DW*8)) mem_lo[a[31:3]] <= nxt;
         else if (a >= HI_BASE && a < 32'h0800_0000) mem_hi[(a - HI_BASE) >> 3] <= nxt;
         else $display("TB: WARN write outside model at %08x", a);
      end
   endtask

   // ---------------- trace: emulator line format ---------------------------
   function automatic string fbit(input logic b);
      return b ? "1" : "0";
   endfunction

   task automatic emit_trace();
      string s;
      logic [31:0] wa;
      logic [63:0] wdat;
      icount++;
      s = $sformatf("i=%0d pc=%08x op=%08x", icount, ret_pc, ret_op);
      for (int i = 0; i < 16; i++) s = {s, $sformatf(" r%0d=%016x", i, dbg_r[i])};
      s = {s, $sformatf(" sp=%08x f=%s%s%s%s%s%s%s", dbg_sp,
                        fbit(dbg_flags.zero), fbit(dbg_flags.sign),
                        fbit(dbg_flags.carry), fbit(dbg_flags.overflow),
                        fbit(dbg_flags.zero),
                        fbit(dbg_flags.sign ^ dbg_flags.overflow),
                        fbit(dbg_flags.carry))};
      if (ret_wr) begin
         if (ret_wr_be == 8'hFF) begin
            // 64-bit store: raw address, raw data (emulator write64)
            s = {s, $sformatf(" wr=%08x/ff/%016x", ret_wr_addr, ret_wr_raw)};
         end else begin
            // sub-word: dword-aligned address; data = raw for MMIO, else the
            // merged dword read back after the write (emulator write_sub)
            wa   = {ret_wr_addr[31:3], 3'b000};
            wdat = (ret_wr_addr[31:28] == 4'hF) ? ret_wr_raw : rd_dw(wa);
            s = {s, $sformatf(" wr=%08x/%02x/%016x", wa, ret_wr_be, wdat)};
         end
      end
      $fdisplay(trace_f, "%s", s);
   endtask

   always @(negedge clk) begin
      if (!rst && ret_valid) begin
         emit_trace();
         if (maxi != 0 && icount >= maxi) begin
            $display("TB_M5A: stop = InstructionCap (%0d)", icount);
            done();
         end
      end
      if (!rst && parked) begin
         case (park_kind)
            3'd0: $display("TB_M5A: stop = Halt after %0d instructions", icount);
            3'd1: $display("TB_M5A: stop = Trap at pc=%08x", park_pc);
            3'd2: $display("TB_M5A: stop = InvalidOpcode(%08x) at pc=%08x", park_op, park_pc);
            default: $display("TB_M5A: stop = Wait at pc=%08x", park_pc);
         endcase
         done();
      end
   end

   task automatic done();
      $display("TB_M5A: %0d instructions retired, %0d UART bytes", icount, uart_n);
      $fclose(trace_f);  $fclose(uart_f);
      $finish;
   endtask

   // watchdog: no retire for a long time -> deadlock
   longint last_i = -1;
   int     idle_n = 0;
   always @(posedge clk) begin
      if (!rst) begin
         if (icount == last_i) begin
            idle_n++;
            if (idle_n > 5000) begin
               $display("TB_M5A: DEADLOCK at pc=%08x (no retire for 5000 cycles, i=%0d)",
                        dut.pc, icount);
               done();
            end
         end else begin idle_n = 0; last_i = icount; end
      end
   end

   string image_f, trace_fn, uart_fn;
   initial begin
      if (!$value$plusargs("IMAGE=%s", image_f)) image_f = "hello.mem";
      if (!$value$plusargs("TRACE=%s", trace_fn)) trace_fn = "rtl.trace";
      if (!$value$plusargs("UARTF=%s", uart_fn))  uart_fn = "rtl.uart";
      void'($value$plusargs("MAXI=%d", maxi));
      void'($value$plusargs("ENTRY=%h", start_pc));
      void'($value$plusargs("LAT=%d", lat_fix));    // fixed DRAM latency (cycles)
      void'($value$plusargs("RAND=%d", lat_rand));  // + $urandom%RAND extra
      for (int i = 0; i < LO_DW; i++) mem_lo[i] = 64'h0;
      for (int i = 0; i < HI_DW; i++) mem_hi[i] = 64'h0;
      $readmemh(image_f, mem_lo);
      trace_f = $fopen(trace_fn, "w");
      uart_f  = $fopen(uart_fn, "w");
      if (trace_f == 0 || uart_f == 0) begin
         $display("TB_M5A: cannot open output files"); $finish;
      end
      $display("TB_M5A: image=%s entry=%08x maxi=%0d", image_f, start_pc, maxi);
      rst = 1;  repeat (4) @(posedge clk);
      rst = 0;  @(posedge clk);
      start = 1; @(posedge clk); start = 0;
   end
endmodule

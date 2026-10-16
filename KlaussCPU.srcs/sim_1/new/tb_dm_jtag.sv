//===========================================================================
// tb_dm_jtag — the debugger end to end below the host: JTAG DR scans ->
// dm_dtm (TCK -> clk crossing) -> debug_module (DMI registers) ->
// pipeline_core (DEBUG_EN=1), running a real program image.
// perf/m5a/run_m5_dbg.sh <prog> jtag runs it and diffs the program trace +
// UART against the klausscc emulator golden: debugging must be invisible to
// the program.
//
// Sequence: halt-on-load at the entry; read every register over the DMI and
// compare with the core; memory reads (auto-increment) vs the memory model
// and a scratch write / readback; +BPN breakpoint hits at +BP; +STEPS
// single-steps; then run to the program's HALT.
//
// Plusargs: +IMAGE +TRACE +UARTF (as tb_pipeline_isa), +BP=<pc hex>
//           +BPN=<hits, default 20> +STEPS=<n, default 200>
//===========================================================================
module tb_dm_jtag;
   import klauss_pkg::*;
   logic clk = 0, rst = 1, start = 0;
   logic [31:0]  start_pc = 32'h20;
   logic [31:0]  m_addr;
   logic         m_read_DV, m_write_DV;
   logic [63:0]  m_wdata, m_rdata, m_rdata_next;
   logic [7:0]   m_be;
   logic         m_next_valid, m_ready;
   logic         ret_valid, ret_wr;
   logic [31:0]  ret_pc, ret_op, ret_wr_addr;
   logic [7:0]   ret_wr_be;
   logic [63:0]  ret_wr_raw;
   logic         parked, bus_idle;
   logic [2:0]   park_kind;
   logic [31:0]  park_pc, park_op;
   logic [63:0]  dbg_r [0:15];
   logic [31:0]  dbg_sp;
   flags_t       dbg_flags;
   logic [3:0]   int_mask_o;
   dbg_in_t      dbg_i;
   dbg_out_t     dbg_o;

   always #5 clk = ~clk;                 // 100 MHz core clock

   pipeline_core #(.DEBUG_EN(1'b1)) dut (
      .clk, .rst, .ce(1'b1), .start, .start_pc,
      .m_addr, .m_read_DV, .m_write_DV, .m_wdata, .m_be, .m_rdata, .m_rdata_next,
      .m_next_valid, .m_ready,
      .lcd_byte(), .lcd_dc(), .lcd_dv(), .lcd_rst_n(), .lcd_rst_wr(), .lcd_ready(1'b1),
      .irq_ready(1'b0), .irq_sel(2'd0), .irq_vector(32'h0), .irq_ack(), .irq_ack_sel(),
      .int_mask_o, .mask_wr(1'b0), .mask_wdata(4'h0),
      .ret_valid, .ret_pc, .ret_op, .ret_wr, .ret_wr_addr, .ret_wr_be, .ret_wr_raw,
      .bus_idle, .perf_stall(), .perf_br(), .perf_br_taken(), .perf_fbr_taken(),
      .parked, .park_kind, .park_pc, .park_op, .dbg_r, .dbg_sp, .dbg_flags,
      .dbg_i, .dbg_o);

   // ---- debug module + JTAG transport ----
   logic        dmi_req, dmi_rsp;
   logic [1:0]  dmi_op;
   logic [7:0]  dmi_addr;
   logic [63:0] dmi_wdata, dmi_rdata;
   logic        tck = 0, sel = 0, capture = 0, shift = 0, update = 0, tdi = 0, tdo;

   debug_module dm (
      .clk, .rst, .dmi_req, .dmi_op, .dmi_addr, .dmi_wdata, .dmi_rsp, .dmi_rdata,
      .dbg_i, .dbg_o, .parked, .park_kind, .park_pc, .core_r(dbg_r),
      .core_sp(dbg_sp), .core_flags(dbg_flags), .core_int_mask(int_mask_o),
      .prog_start(start));
   dm_dtm dtm (
      .tck, .sel, .capture, .shift, .update, .tdi, .tdo,
      .clk, .rst, .dmi_req, .dmi_op, .dmi_addr, .dmi_wdata, .dmi_rsp, .dmi_rdata);

   // ---------------- memory model (as tb_pipeline_isa) ----------------------
   localparam int LO_DW = 1048576;
   localparam logic [31:0] HI_BASE = 32'h07F0_0000;
   localparam int HI_DW = 131072;
   logic [63:0] mem_lo [0:LO_DW-1];
   logic [63:0] mem_hi [0:HI_DW-1];
   integer uart_f, trace_f;
   longint icount = 0;

   function automatic logic [63:0] rd_dw(input logic [31:0] a);
      if (a[31:28] == 4'hF) rd_dw = (a[27:16] == 12'h001 && (a[15:0] & 16'hFFF8) == 16'h0010) ? 64'h2 : 64'h0;
      else if (a < 32'(LO_DW*8)) rd_dw = mem_lo[a[31:3]];
      else if (a >= HI_BASE && a < 32'h0800_0000) rd_dw = mem_hi[(a - HI_BASE) >> 3];
      else rd_dw = 64'h0;
   endfunction

   task automatic apply_write(input logic [31:0] a, input logic [63:0] wd, input logic [7:0] be);
      logic [63:0] cur, nxt;
      if (a[31:28] == 4'hF) begin
         if (a[27:16] == 12'h001 && (a[15:0] & 16'hFFF8) == 16'h0000) $fwrite(uart_f, "%c", wd[7:0]);
      end else begin
         cur = rd_dw(a);
         for (int i = 0; i < 8; i++) nxt[i*8 +: 8] = be[i] ? wd[i*8 +: 8] : cur[i*8 +: 8];
         if (a < 32'(LO_DW*8)) mem_lo[a[31:3]] <= nxt;
         else if (a >= HI_BASE && a < 32'h0800_0000) mem_hi[(a - HI_BASE) >> 3] <= nxt;
      end
   endtask

   logic        in_flight = 0;
   logic [31:0] q_addr;  logic q_wr;  logic [63:0] q_wdata;  logic [7:0] q_be;
   int          lat_cnt;
   always @(posedge clk) begin
      m_ready <= 1'b0;
      if (rst) in_flight <= 1'b0;
      else if (!in_flight && !m_ready && (m_read_DV || m_write_DV)) begin
         in_flight <= 1'b1;
         q_addr <= m_addr;  q_wr <= m_write_DV;  q_wdata <= m_wdata;  q_be <= m_be;
         lat_cnt <= (m_addr[31:28] == 4'hF) ? (m_read_DV ? 2 : 1) : 3;
      end else if (in_flight) begin
         if (lat_cnt > 1) lat_cnt <= lat_cnt - 1;
         else begin
            if (q_wr) apply_write(q_addr, q_wdata, q_be);
            else begin
               m_rdata      <= rd_dw(q_addr);
               m_rdata_next <= rd_dw({q_addr[31:3], 3'b000} + 32'd8);
               m_next_valid <= (q_addr[31:28] != 4'hF) && (q_addr[4:3] != 2'b11);
            end
            m_ready <= 1'b1;  in_flight <= 1'b0;
         end
      end
   end

   // ---------------- trace (klausscc --emulate --trace format) ----------------
   function automatic string fbit(input logic b); return b ? "1" : "0"; endfunction
   always @(negedge clk) if (!rst && ret_valid) begin
      string s;
      logic [31:0] wa;
      icount++;
      s = $sformatf("i=%0d pc=%08x op=%08x", icount, ret_pc, ret_op);
      for (int i = 0; i < 16; i++) s = {s, $sformatf(" r%0d=%016x", i, dbg_r[i])};
      s = {s, $sformatf(" sp=%08x f=%s%s%s%s%s%s%s", dbg_sp, fbit(dbg_flags.zero),
                        fbit(dbg_flags.sign), fbit(dbg_flags.carry), fbit(dbg_flags.overflow),
                        fbit(dbg_flags.zero), fbit(dbg_flags.sign ^ dbg_flags.overflow),
                        fbit(dbg_flags.carry))};
      if (ret_wr) begin
         if (ret_wr_be == 8'hFF) s = {s, $sformatf(" wr=%08x/ff/%016x", ret_wr_addr, ret_wr_raw)};
         else begin
            wa = {ret_wr_addr[31:3], 3'b000};
            s = {s, $sformatf(" wr=%08x/%02x/%016x", wa, ret_wr_be,
                              (ret_wr_addr[31:28] == 4'hF) ? ret_wr_raw : rd_dw(wa))};
         end
      end
      $fdisplay(trace_f, "%s", s);
   end

   // ---------------- JTAG host side ------------------------------------------
   always #18.5 tck = ~tck;              // 27 MHz, asynchronous to clk

   int errors = 0;
   task automatic err(input string m);
      $display("TB_DBG: ERROR %s (i=%0d)", m, icount);  errors++;
   endtask

   // one DR scan: CAPTURE, 74 x SHIFT (LSB first), UPDATE, 3 x Run-Test/Idle
   task automatic scan(input logic [73:0] din, output logic [73:0] dout);
      @(negedge tck);  sel = 1;  capture = 1;
      @(negedge tck);  capture = 0;  shift = 1;
      for (int i = 0; i < 74; i++) begin
         tdi = din[i];  dout[i] = tdo;
         @(negedge tck);
      end
      shift = 0;  update = 1;
      @(negedge tck);  update = 0;
      repeat (3) @(negedge tck);
   endtask

   // DMI access: issue, then nop scans until the response is not busy
   task automatic dmi(input logic [1:0] op, input logic [7:0] a, input logic [63:0] d,
                      output logic [63:0] rd);
      logic [73:0] o;
      int n = 0;
      scan({op, a, d}, o);
      do begin
         scan({2'b00, 8'h00, 64'h0}, o);
         if (++n > 50) begin err("DMI stuck busy"); break; end
      end while (o[73:72] == 2'd3);
      if (o[71:64] !== a) err($sformatf("DMI response for %02x, sent %02x", o[71:64], a));
      rd = o[63:0];
   endtask
   task automatic wr(input logic [7:0] a, input logic [63:0] d);
      logic [63:0] x;  dmi(2'd2, a, d, x);
   endtask
   task automatic rd(input logic [7:0] a, output logic [63:0] v);
      dmi(2'd1, a, 64'h0, v);
   endtask
   task automatic wait_halted(output logic [63:0] st);
      int n = 0;
      do begin
         rd(8'h01, st);
         if (++n > 200000) begin err("halt timeout"); $finish; end
      end while (!st[0]);
   endtask

   initial begin : host
      string image_f, trace_fn, uart_fn;
      logic [63:0] st, v;
      logic [31:0] bp;
      int bpn, steps, hits;
      if (!$value$plusargs("IMAGE=%s", image_f)) image_f = "hello.mem";
      if (!$value$plusargs("TRACE=%s", trace_fn)) trace_fn = "rtl.trace";
      if (!$value$plusargs("UARTF=%s", uart_fn))  uart_fn = "rtl.uart";
      if (!$value$plusargs("BP=%h", bp)) bp = 32'h0;
      if (!$value$plusargs("BPN=%d", bpn)) bpn = 20;
      if (!$value$plusargs("STEPS=%d", steps)) steps = 200;
      for (int i = 0; i < LO_DW; i++) mem_lo[i] = 64'h0;
      for (int i = 0; i < HI_DW; i++) mem_hi[i] = 64'h0;
      $readmemh(image_f, mem_lo);
      trace_f = $fopen(trace_fn, "w");  uart_f = $fopen(uart_fn, "w");
      $display("TB_DM_JTAG: image=%s bp=%08x", image_f, bp);
      repeat (4) @(posedge clk);  rst = 0;  repeat (4) @(posedge clk);

      // DM identity, then halt-on-load before the program starts
      rd(8'h01, st);
      if (st[31:24] !== 8'h4B || st[15:12] !== 4'd1) err($sformatf("DMSTATUS id %016x", st));
      wr(8'h00, 64'h10);                         // halt_on_load
      @(negedge clk);  start = 1;  @(negedge clk);  start = 0;
      wait_halted(st);
      if (st[6:4] !== DBG_CAUSE_REQ) err($sformatf("entry halt cause %0d", st[6:4]));
      rd(8'h06, v);
      if (v[31:0] !== start_pc) err($sformatf("entry PC %08x", v[31:0]));

      // registers over the DMI vs the core
      for (int r = 0; r < 20; r++) begin
         logic [63:0] want;
         wr(8'h02, 64'(r));                       // REG_CMD read
         rd(8'h03, v);
         want = (r < 16) ? dbg_r[r] : (r == 16) ? {32'b0, dbg_sp} : (r == 17) ? {32'b0, park_pc}
              : (r == 18) ? {60'b0, dbg_flags.zero, dbg_flags.sign, dbg_flags.carry, dbg_flags.overflow}
              : {60'b0, int_mask_o};
         if (v !== want) err($sformatf("reg %0d = %016x, core %016x", r, v, want));
      end
      wr(8'h03, 64'hDEAD_BEEF_0000_1234);  wr(8'h02, 64'h10D);   // r13 = DATA0
      if (dbg_r[13] !== 64'hDEAD_BEEF_0000_1234) err("reg write r13");
      wr(8'h03, 64'h0);  wr(8'h02, 64'h10D);                       // restore (r13 = 0 at entry)

      // memory: auto-increment reads of the program vs the model
      wr(8'h04, 64'h20);
      for (int k = 0; k < 16; k++) begin
         wr(8'h05, 64'h2FF);                        // read, all lanes, post-increment
         rd(8'h03, v);
         if (v !== mem_lo[4 + k]) err($sformatf("mem %08x = %016x, model %016x", 32'h20 + 8*k, v, mem_lo[4 + k]));
      end
      rd(8'h04, v);
      if (v[31:0] !== 32'h20 + 32'd128) err($sformatf("MEM_ADDR after 16 reads = %08x", v[31:0]));
      wr(8'h04, 64'(HI_BASE));  wr(8'h03, 64'h1111_2222_3333_4444);  wr(8'h05, 64'h10F);
      wr(8'h05, 64'h0FF);  rd(8'h03, v);
      if (v !== 64'h0000_0000_3333_4444) err($sformatf("mem write/readback %016x", v));
      wr(8'h03, 64'h0);  wr(8'h05, 64'h1FF);       // restore

      hits = 0;
      if (bp != 0) begin
         wr(8'h10, {31'b0, 1'b1, bp});              // BP0 enable
         for (hits = 0; hits < bpn; hits++) begin
            wr(8'h00, 64'hA);                       // resume (attached)
            wait_halted(st);
            if (st[0] && st[6:4] == DBG_CAUSE_HALT) break;
            if (st[6:4] !== DBG_CAUSE_BP) err($sformatf("bp cause %0d", st[6:4]));
            rd(8'h06, v);
            if (v[31:0] !== bp) err($sformatf("bp PC %08x", v[31:0]));
         end
         wr(8'h10, 64'h0);                          // BP0 off
      end

      // single steps: each retires exactly one instruction
      for (int k = 0; k < steps; k++) begin
         automatic longint i0 = icount;
         wr(8'h00, 64'hE);                          // resume + step (attached)
         wait_halted(st);
         if (st[6:4] !== DBG_CAUSE_STEP) err($sformatf("step cause %0d", st[6:4]));
         if (icount != i0 + 1) err($sformatf("step retired %0d", icount - i0));
      end

      // free run to the program's HALT (attached so it stays debug-halted)
      wr(8'h00, 64'h0A);                            // resume + attached
      wait_halted(st);
      if (st[6:4] !== DBG_CAUSE_HALT) err($sformatf("final cause %0d", st[6:4]));
      $display("TB_M5A: stop = Halt after %0d instructions", icount);
      $display("TB_DBG: mode=jtag halts=0 steps=%0d bp hits=%0d wp hits=0 mem checks=17 errors=%0d",
               steps, hits, errors);
      $fclose(trace_f);  $fclose(uart_f);
      $finish;
   end
endmodule

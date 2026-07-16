//===========================================================================
// tb_soc — M5d full-SoC smoke: boots the resident boot-ROM image (netboot.mem
// in the sim cwd — the runner copies the program image there) through the
// REAL chain: NO_PROGRAM boot copy -> DDR (behavioral) -> LOAD_COMPLETE ->
// START_WAIT -> PIPE_RUN, with the real cache, bus_splitter, and MMIO timing.
//
// Verifies:
//  - the retire stream (tapped from pipeline_core_i) matches the emulator
//    golden trace line-for-line (same format as tb_pipeline_isa; f= excluded
//    by the runner's diff),
//  - the UART TX byte stream (tapped at uart_send_msg's input handshake)
//    CONTAINS the program's golden byte stream (the loader's own messages
//    precede it),
//  - the run ends in HALTED via HALTED_BREAK.
//
// Behavioral ddr2_control / clk_wiz_0 shadow the real ones (do NOT compile
// ddr2_control.sv into this sim). Line layout matches mem_read_write:
// addr[4:3] 00=dw0 [255:192], 01=[191:128], 10=[127:64], 11=[63:0].
//===========================================================================

// ---------------------------------------------------------------------------
// liteeth_core stub — Ethernet is not exercised by the boot smoke; the real
// generated core is simulation-hostile (zero-delay loops at time 0).
// ---------------------------------------------------------------------------
module liteeth_core (
    input  sys_clock, input sys_reset, output interrupt,
    input  rmii_clocks_ref_clk, input rmii_crs_dv, output rmii_mdc,
    inout  rmii_mdio, output rmii_rst_n, input [1:0] rmii_rx_data,
    output [1:0] rmii_tx_data, output rmii_tx_en,
    input  [29:0] wishbone_adr, input [31:0] wishbone_dat_w,
    output [31:0] wishbone_dat_r, input [3:0] wishbone_sel,
    input  wishbone_we, input wishbone_cyc, input wishbone_stb,
    input  [1:0] wishbone_bte, input [2:0] wishbone_cti,
    output wishbone_ack, output wishbone_err
);
   assign interrupt = 1'b0;
   assign rmii_mdc = 1'b0;  assign rmii_rst_n = 1'b1;
   assign rmii_tx_data = 2'b00;  assign rmii_tx_en = 1'b0;
   assign wishbone_dat_r = 32'h0;
   logic ack_q = 0;
   always @(posedge sys_clock) ack_q <= wishbone_cyc && wishbone_stb && !ack_q;
   assign wishbone_ack = ack_q;
   assign wishbone_err = 1'b0;
endmodule

// ---------------------------------------------------------------------------
// ring_osc stub — deterministic, no combinational ring.
// ---------------------------------------------------------------------------
module ring_osc (output wire o_bit);
   assign o_bit = 1'b0;
endmodule

// ---------------------------------------------------------------------------
// clk_wiz_0 stub — pass the input clock through.
// ---------------------------------------------------------------------------
module clk_wiz_0 (input i_Clk, output clk_200, output clk_50, output locked,
                  input resetn);
   assign clk_200 = i_Clk;
   assign clk_50  = i_Clk;
   assign locked  = 1'b1;
endmodule

// ---------------------------------------------------------------------------
// Behavioral ddr2_control — 256-bit (32 B) lines, fixed latency, ready held
// while DV is high; o_ui_clk passes sys_clk_i through (whole CPU runs on it).
// Models the low 8 MiB (code/heap) + top 1 MiB (stack) of the 128 MiB space.
// ---------------------------------------------------------------------------
module ddr2_control (
    inout [15:0] ddr2_dq,
    inout [1:0] ddr2_dqs_n,
    inout [1:0] ddr2_dqs_p,
    output [12:0] ddr2_addr,
    output [2:0] ddr2_ba,
    output ddr2_ras_n,
    output ddr2_cas_n,
    output ddr2_we_n,
    output [0:0] ddr2_ck_p,
    output [0:0] ddr2_ck_n,
    output [0:0] ddr2_cke,
    output [0:0] ddr2_cs_n,
    output [1:0] ddr2_dm,
    output [0:0] ddr2_odt,
    input resetn,
    input sys_clk_i,
    input i_mem_write_DV,
    input i_mem_read_DV,
    input [31:0] i_mem_addr,
    input i_mem_wide,
    input [1:0] i_mem_dw_off,
    input [255:0] i_mem_write_data,
    inout [15:0] i_app_wdf_mask,
    output logic [255:0] o_mem_read_data,
    output logic o_mem_ready,
    // M10a CWF early channel — same contract as the real controller: on a wide
    // read the requested dword pulses o_mem_dw_ready DW_LAT cycles before
    // o_mem_ready; even offsets also carry the companion dword.
    output logic [63:0] o_mem_rd_dw,
    output logic [63:0] o_mem_rd_dw_next,
    output logic        o_mem_dw_next_ok,
    output logic        o_mem_dw_ready,
    output o_calib_done,
    output o_ui_clk
);
   assign o_calib_done = 1'b1;
   assign o_ui_clk     = sys_clk_i;

   localparam int LO_LINES = 262144;                 // 8 MiB / 32 B
   localparam logic [31:0] HI_BASE = 32'h07F0_0000;  // top 1 MiB
   localparam int HI_LINES = 32768;
   logic [255:0] mem_lo [0:LO_LINES-1];
   logic [255:0] mem_hi [0:HI_LINES-1];

   initial begin
      o_mem_ready = 0;
      for (int i = 0; i < LO_LINES; i++) mem_lo[i] = '0;
      for (int i = 0; i < HI_LINES; i++) mem_hi[i] = '0;
   end

   function automatic logic [255:0] line_rd(input logic [31:0] a);
      logic [31:0] la;
      la = {a[31:5], 5'b0};
      if (la < 32'(LO_LINES) * 32)                      line_rd = mem_lo[la[31:5]];
      else if (la >= HI_BASE && la < 32'h0800_0000)     line_rd = mem_hi[(la - HI_BASE) >> 5];
      else                                              line_rd = '0;
   endfunction

   task automatic line_wr(input logic [31:0] a, input logic [255:0] d, input logic wide);
      logic [31:0] la;
      logic [255:0] cur;
      la = {a[31:5], 5'b0};
      cur = line_rd(a);
      if (!wide) begin
         // blitter narrow path: 16 B in the LOW 128 bits; a[4] picks the half
         if (a[4]) cur[127:0]   = d[127:0];
         else      cur[255:128] = d[127:0];
      end else cur = d;
      if (la < 32'(LO_LINES) * 32)                  mem_lo[la[31:5]] <= cur;
      else if (la >= HI_BASE && la < 32'h0800_0000) mem_hi[(la - HI_BASE) >> 5] <= cur;
      else $display("TB_SOC: WARN DDR write outside model %08x", a);
   endtask

   function automatic logic [63:0] dw_of(input logic [255:0] l, input logic [1:0] off);
      case (off)
         2'b00:   dw_of = l[255:192];
         2'b01:   dw_of = l[191:128];
         2'b10:   dw_of = l[127:64];
         default: dw_of = l[63:0];
      endcase
   endfunction

   localparam LAT    = 8;
   localparam DW_LAT = 4;   // CWF dword leads the full line by this many cycles
   logic [4:0] cnt = 0;
   initial begin
      o_mem_dw_ready   = 0;
      o_mem_dw_next_ok = 0;
   end
   always @(posedge sys_clk_i) begin
      o_mem_dw_ready <= 1'b0;   // 1-cycle pulse
      if (i_mem_read_DV !== 1'b1 && i_mem_write_DV !== 1'b1) begin
         o_mem_ready <= 0;
         cnt <= 0;
      end else if (!o_mem_ready) begin
         if (cnt == LAT) begin
            if (i_mem_read_DV === 1'b1) o_mem_read_data <= line_rd(i_mem_addr);
            else                        line_wr(i_mem_addr, i_mem_write_data, i_mem_wide);
            o_mem_ready <= 1;
            cnt <= 0;
         end else begin
            // M10a CWF early channel (wide reads only): requested dword (and
            // its beat-0 companion for even offsets) before the full line.
            if (cnt == LAT - DW_LAT && i_mem_read_DV === 1'b1 && i_mem_wide) begin
               o_mem_rd_dw      <= dw_of(line_rd(i_mem_addr), i_mem_dw_off);
               o_mem_rd_dw_next <= dw_of(line_rd(i_mem_addr), i_mem_dw_off + 2'd1);
               o_mem_dw_next_ok <= ~i_mem_dw_off[0];
               o_mem_dw_ready   <= 1'b1;
            end
            cnt <= cnt + 1;
         end
      end
   end
endmodule

// ---------------------------------------------------------------------------
module tb_soc;
   import klauss_pkg::*;

   logic clk100 = 0;
   always #5 clk100 = ~clk100;

   wire        o_uart_tx;
   wire [15:0] o_led;
   wire        o_SPI_LCD_Clk, o_SPI_LCD_MOSI, o_SPI_LCD_CS_n, o_LCD_DC, o_LCD_reset_n;
   wire [7:0]  o_Anode_Activate, o_LED_cathode;
   wire [2:0]  o_LED_RGB_1, o_LED_RGB_2;
   wire        o_SD_RESET, o_SD_SCK, o_SD_MOSI, o_SD_CS_n, o_SD_DAT1, o_SD_DAT2;
   wire        ETH_MDC, ETH_RSTN, ETH_TXEN, ETH_REFCLK;
   wire [1:0]  ETH_TXD;
   wire [15:0] ddr2_dq;
   wire [1:0]  ddr2_dqs_n, ddr2_dqs_p, ddr2_dm;
   wire [12:0] ddr2_addr;
   wire [2:0]  ddr2_ba;

   // Power-on reset: silicon initializes registers from the bitstream; xsim
   // starts them X, so pulse the reset like a real button press.
   logic cpu_resetn = 1'b0;
   initial begin
      repeat (40) @(posedge clk100);
      cpu_resetn = 1'b1;
   end

   KlaussCPU dut (
      .CPU_RESETN (cpu_resetn),
      .i_Clk_board(clk100),
      .i_uart_rx  (1'b1),
      .i_load_H   (1'b0),
      .o_uart_tx  (o_uart_tx),
      .o_led      (o_led),
      .o_SPI_LCD_Clk(o_SPI_LCD_Clk), .i_SPI_LCD_MISO(1'b0),
      .o_SPI_LCD_MOSI(o_SPI_LCD_MOSI), .o_SPI_LCD_CS_n(o_SPI_LCD_CS_n),
      .o_LCD_DC(o_LCD_DC), .o_LCD_reset_n(o_LCD_reset_n),
      .o_Anode_Activate(o_Anode_Activate), .o_LED_cathode(o_LED_cathode),
      .i_switch   (16'h0),
      .o_LED_RGB_1(o_LED_RGB_1), .o_LED_RGB_2(o_LED_RGB_2),
      .o_SD_RESET(o_SD_RESET), .i_SD_CD(1'b0), .o_SD_SCK(o_SD_SCK),
      .o_SD_MOSI(o_SD_MOSI), .i_SD_MISO(1'b0), .o_SD_CS_n(o_SD_CS_n),
      .o_SD_DAT1(o_SD_DAT1), .o_SD_DAT2(o_SD_DAT2),
      .ETH_MDC(ETH_MDC), .ETH_MDIO(), .ETH_RSTN(ETH_RSTN),
      .ETH_CRSDV(1'b0), .ETH_RXERR(1'b0), .ETH_RXD(2'b00),
      .ETH_TXEN(ETH_TXEN), .ETH_TXD(ETH_TXD), .ETH_REFCLK(ETH_REFCLK),
      .ddr2_dq(ddr2_dq), .ddr2_dqs_n(ddr2_dqs_n), .ddr2_dqs_p(ddr2_dqs_p),
      .ddr2_addr(ddr2_addr), .ddr2_ba(ddr2_ba),
      .ddr2_ras_n(), .ddr2_cas_n(), .ddr2_we_n(),
      .ddr2_ck_p(), .ddr2_ck_n(), .ddr2_cke(), .ddr2_cs_n(),
      .ddr2_dm(ddr2_dm), .ddr2_odt()
   );

   // silicon powers up with this at 0 (bitstream init); xsim needs the deposit
   initial dut.r_start_wait_counter = 32'h0;

   // ---------------- UART TX capture (tap the message sender) -------------
   integer uart_f, trace_f;
   int     uart_n = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.r_msg_send_DV === 1'b1 && !dut.w_sending_msg) begin
         for (int b = 0; b < dut.r_msg_length; b++)
            $fwrite(uart_f, "%c", dut.r_msg[b*8 +: 8]);
         uart_n += dut.r_msg_length;
      end
   end

   // ---------------- retire trace (emulator format, from the core) --------
   longint icount = 0;
   function automatic string fbit(input logic b);
      return b ? "1" : "0";
   endfunction
   always @(negedge dut.i_Clk) begin
      if (dut.pip_ret_valid === 1'b1) begin
         string s;
         icount++;
         s = $sformatf("i=%0d pc=%08x op=%08x", icount, dut.pip_ret_pc, dut.pip_ret_op);
         for (int i = 0; i < 16; i++)
            s = {s, $sformatf(" r%0d=%016x", i, dut.pipeline_core_i.rf[i])};
         s = {s, $sformatf(" sp=%08x f=%s%s%s%s%s%s%s", dut.pipeline_core_i.sp,
                fbit(dut.pipeline_core_i.flags.zero), fbit(dut.pipeline_core_i.flags.sign),
                fbit(dut.pipeline_core_i.flags.carry), fbit(dut.pipeline_core_i.flags.overflow),
                fbit(dut.pipeline_core_i.flags.zero),
                fbit(dut.pipeline_core_i.flags.sign ^ dut.pipeline_core_i.flags.overflow),
                fbit(dut.pipeline_core_i.flags.carry))};
         if (dut.pip_ret_wr) begin
            // Sub-word DRAM stores: the golden shows the MERGED dword (read
            // back after the write) but the cache line isn't cheaply readable
            // here, so we emit the RAW value and the runner normalizes the
            // data field away on non-ff stores on BOTH sides (addr + byte
            // enables still compared; full data compared for 64-bit stores
            // and MMIO). The standalone M5a gate already checks merged data.
            if (dut.pip_ret_wr_be == 8'hFF)
               s = {s, $sformatf(" wr=%08x/ff/%016x", dut.pip_ret_wr_addr, dut.pip_ret_wr_raw)};
            else
               s = {s, $sformatf(" wr=%08x/%02x/%016x",
                                 {dut.pip_ret_wr_addr[31:3], 3'b000},
                                 dut.pip_ret_wr_be, dut.pip_ret_wr_raw)};
         end
         $fdisplay(trace_f, "%s", s);
      end
   end

   // ---------------- end / progress ----------------
   longint last_i = -1;
   int     idle_n = 0;
   longint maxi = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.st.SM == HALTED) begin
         $display("TB_SOC: HALTED after %0d retired instructions, %0d UART bytes", icount, uart_n);
         $fclose(uart_f); $fclose(trace_f);
         $finish;
      end
      if (maxi != 0 && icount >= maxi) begin
         $display("TB_SOC: instruction cap %0d reached", maxi);
         $fclose(uart_f); $fclose(trace_f);
         $finish;
      end
      if (icount == last_i) begin
         idle_n++;
         // generous: a legitimate HCF crash dump emits ~1k UART bytes with no
         // retires; only a genuinely stuck pipe survives this long
         if (idle_n > 600000) begin
            $display("TB_SOC: DEADLOCK sm=%0d pc=%08x i=%0d", dut.st.SM, dut.pipeline_core_i.pc, icount);
            $fclose(uart_f); $fclose(trace_f);
            $finish;
         end
      end else begin idle_n = 0; last_i = icount; end
   end

   string trace_fn, uart_fn;
   initial begin
      if (!$value$plusargs("TRACE=%s", trace_fn)) trace_fn = "soc.trace";
      if (!$value$plusargs("UARTF=%s", uart_fn))  uart_fn = "soc.uart";
      void'($value$plusargs("MAXI=%d", maxi));
      trace_f = $fopen(trace_fn, "w");
      uart_f  = $fopen(uart_fn, "w");
      $display("TB_SOC: booting resident image (netboot.mem in sim cwd)");
   end
endmodule

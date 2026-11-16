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
// clk_wiz_0 is a stub; DDR is the behavioural MIG (mig_model.sv, compiled
// alongside) behind the REAL ddr2_control (CORE1_L2_EN = 0) or l2_cache
// (HW_PERF_PLAN.md §6).  (Before §6 a behavioural ddr2_control, 8 cycles a
// line, stood in for both.)  Line layout matches mem_read_write:
// addr[4:3] 00=dw0 [255:192], 01=[191:128], 10=[127:64], 11=[63:0].
//
// Memory-consistency checker (§6): a reference memory takes every write the
// DDR side (L2 or ddr2_control) acknowledges at the arbiter's port, from any
// master; every read answer, and every early critical dword, must equal it
// (FATAL line otherwise).  Indexed by the DDR address: PA[31:27] alias
// PA[26:0] (the MIG ignores its rank bit PA[27]; PA[31:28] never reach it).
// With the L2 (+define+L2_PROBE): drains that start while VGA wants the bus
// or a DMA master holds it, with no request (nor the WBINV walk) waiting on
// the write buffer (review 1 M2, review 2), are counted ("L2 rt drains
// bad="; the need spelled out as in tb_l2, not taken from the RTL).
//===========================================================================

// ---------------------------------------------------------------------------
// liteeth_core stub — Ethernet is not exercised by the boot smoke; the real
// generated core is simulation-hostile (zero-delay loops at time 0).
// ---------------------------------------------------------------------------
module liteeth_core (
    input  sys_clock, input sys_reset, output interrupt,
    input  rmii_clocks_ref_clk, input rmii_crs_dv, output rmii_mdc,
    input  [3:0] rx_nslots, input rx_slot_reset,
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
module tb_soc;
   import klauss_pkg::*;

   logic clk100 = 0;
   always #5 clk100 = ~clk100;

   wire        o_uart_tx;
   wire [15:0] o_led;
   wire        o_SPI_LCD_Clk, o_SPI_LCD_MOSI, o_SPI_LCD_CS_n, o_LCD_DC, o_LCD_reset_n;
   wire [7:0]  o_Anode_Activate, o_LED_cathode;
   wire [2:0]  o_LED_RGB_1, o_LED_RGB_2;
   wire        o_SD_RESET, o_SD_SCK;
   wire        io_SD_CMD;
   wire [3:0]  io_SD_DAT;
   pullup (io_SD_CMD);
   pullup (io_SD_DAT[0]); pullup (io_SD_DAT[1]); pullup (io_SD_DAT[2]); pullup (io_SD_DAT[3]);
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
      .io_SD_CMD(io_SD_CMD), .io_SD_DAT(io_SD_DAT),
      .o_QSPI_CS_n(), .o_QSPI_DQ0(), .i_QSPI_DQ1(1'b1), .o_QSPI_DQ2(), .o_QSPI_DQ3(),
      .ETH_MDC(ETH_MDC), .ETH_MDIO(), .ETH_RSTN(ETH_RSTN),
      .ETH_CRSDV(1'b0), .ETH_RXERR(1'b0), .ETH_RXD(2'b00),
      .ETH_TXEN(ETH_TXEN), .ETH_TXD(ETH_TXD), .ETH_REFCLK(ETH_REFCLK),
      .VGA_R(), .VGA_G(), .VGA_B(), .VGA_HS(), .VGA_VS(),
      .ddr2_dq(ddr2_dq), .ddr2_dqs_n(ddr2_dqs_n), .ddr2_dqs_p(ddr2_dqs_p),
      .ddr2_addr(ddr2_addr), .ddr2_ba(ddr2_ba),
      .ddr2_ras_n(), .ddr2_cas_n(), .ddr2_we_n(),
      .ddr2_ck_p(), .ddr2_ck_n(), .ddr2_cke(), .ddr2_cs_n(),
      .ddr2_dm(ddr2_dm), .ddr2_odt()
   );

   // the SD card on the native bus (sd_card_sync: the SD DMA tests; idle
   // unless a program powers the slot and talks to it)
   sd_card_sync sdcard (.clk(dut.i_Clk), .sck(o_SD_SCK), .pwr(!o_SD_RESET), .cmd(io_SD_CMD), .dat(io_SD_DAT));

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

   // ---------------- front-port counts and the consistency checker ----------
`ifndef DDRPATH
`define DDRPATH g_l2.l2_cache
`endif
   int fp_rd = 0, fp_wr = 0;
   longint cc_n = 0;
   int cc_err = 0;
   logic [63:0] ref_m [logic [31:0]];
   function automatic logic [63:0] rget(input logic [31:0] a);
      a = {5'b0, a[26:3], 3'b0};
      return ref_m.exists(a) ? ref_m[a] : 64'h0;
   endfunction
   function automatic bit in_model(input logic [31:0] a);
      a = {5'b0, a[26:0]};
      return a < 32'h0080_0000 || (a >= 32'h07F0_0000 && a < 32'h0800_0000);
   endfunction
   task automatic cc_bad(input string s);
      cc_err++;
      if (cc_err <= 5) $display("TB_SOC: FATAL consistency %s", s);
   endtask
   always @(posedge dut.i_Clk) begin
      if (dut.mem_read_write.w_cache_ddr_ready) begin
         if (dut.mem_read_write.o_ddr_mem_read_DV) fp_rd++; else fp_wr++;
      end
      if (dut.mem_read_write.w_cache_dw_ready) begin
         logic [31:0] ca = {dut.mem_read_write.mig_addr[31:5], 5'b0} + 8 * dut.mem_read_write.mig_dw_off;
         if (in_model(ca)) begin
            cc_n++;
            if (dut.mem_read_write.w_ddr_rd_dw !== rget(ca))
               cc_bad($sformatf("early dw %08x %h want %h", ca, dut.mem_read_write.w_ddr_rd_dw, rget(ca)));
            if (dut.mem_read_write.w_ddr_dw_next_ok && dut.mem_read_write.w_ddr_rd_dw_next !== rget(ca + 8))
               cc_bad($sformatf("early next %08x", ca + 8));
         end
      end
      if (dut.mem_read_write.i_ddr_mem_ready) begin
         logic [31:0] a;
         logic [255:0] d;
         logic [15:0] m;
         bit wd;
         wd = dut.mem_read_write.mig_wide;
         a  = wd ? {5'b0, dut.mem_read_write.mig_addr[26:5], 5'b0} : {5'b0, dut.mem_read_write.mig_addr[26:4], 4'b0};
         if (in_model(a)) begin
            cc_n++;
            if (dut.mem_read_write.mig_write_DV) begin
               d = dut.mem_read_write.mig_write_data; m = dut.mem_read_write.mig_wdf_mask;
               if (wd) for (int j = 0; j < 4; j++) ref_m[a + 8 * j] = d[255 - 64 * j -: 64];
               else begin
                  logic [63:0] hi = rget(a), lo = rget(a + 8);
                  for (int i = 0; i < 8; i++) begin
                     if (!m[8 + i]) hi[i*8 +: 8] = d[64 + i*8 +: 8];
                     if (!m[i])     lo[i*8 +: 8] = d[i*8 +: 8];
                  end
                  ref_m[a] = hi; ref_m[a + 8] = lo;
               end
            end else if (dut.mem_read_write.mig_read_DV) begin
               d = dut.mem_read_write.i_ddr_mem_read_data;
               if (wd) begin
                  for (int j = 0; j < 4; j++) if (d[255 - 64 * j -: 64] !== rget(a + 8 * j))
                     cc_bad($sformatf("read %08x dw%0d %h want %h", a, j, d[255 - 64 * j -: 64], rget(a + 8 * j)));
               end else if (d[127:64] !== rget(a) || d[63:0] !== rget(a + 8))
                  cc_bad($sformatf("narrow read %08x", a));
            end
         end
      end
   end

   // ---------------- SD DMA transactions (master E) ----------------
   longint runcyc_all = 0;
   always @(posedge dut.i_Clk) runcyc_all++;
   // (SD_DMA_PROBE: run.py defines it for an RTL with master E)
   int sd_dma_wr = 0, sd_dma_rd = 0;
`ifdef SD_DMA_PROBE
   always @(posedge dut.i_Clk) if (dut.mem_read_write.r_grant_sd && dut.mem_read_write.i_ddr_mem_ready) begin
      if (dut.mem_read_write.mig_write_DV) sd_dma_wr++; else sd_dma_rd++;
   end
   // the worst SD DMA transaction (cycles from its req to its ready; and DV
   // to ready) and the worst wait of VGA's req for its grant
   int sd_lat_max = 0, sd_dv_max = 0, vga_gw_max = 0, sd_lat = 0, sd_dv = 0, vga_gw = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.w_sd_dma_req) sd_lat++; else sd_lat = 0;
      if (dut.w_sd_dma_write_DV || dut.w_sd_dma_read_DV) sd_dv++; else sd_dv = 0;
      if (dut.w_sd_dma_ready) begin
         if (sd_lat > sd_lat_max) sd_lat_max = sd_lat;
         if (sd_dv > sd_dv_max) sd_dv_max = sd_dv;
      end
      if (dut.vga_ctrl_i.scanout.o_dma_req && !dut.vga_ctrl_i.scanout.i_dma_grant) vga_gw++;
      else begin if (vga_gw > vga_gw_max) vga_gw_max = vga_gw; vga_gw = 0; end
   end
   // SD DMA abort latency (runtime-96's stop budget): cycles from an
   // SDM_CFG write with DMA_EN = 0 (or a FIFO reset) during DMA_BUSY to
   // DMA_BUSY = 0, max per kind; and each 16 B transaction's wait for the
   // bus (req -> grant), as a histogram
   longint ab_t0 = -1; int ab_kind = 0; int ab_max[2] = '{0, 0}; int ab_n[2] = '{0, 0};
   int bw = 0, bw_max = 0;
   int bw_h[7] = '{0, 0, 0, 0, 0, 0, 0};        // < 16, < 64, < 256, < 1024, < 2048, < 4096, more
   always @(posedge dut.i_Clk) begin
      if (ab_t0 < 0 && dut.sd_spi_i.native_i.w_dma_busy &&
          (dut.sd_spi_i.native_i.w_cfg_dma_off || dut.sd_spi_i.native_i.w_fifo_rs)) begin
         ab_t0 = runcyc_all; ab_kind = dut.sd_spi_i.native_i.w_fifo_rs ? 1 : 0;
      end else if (ab_t0 >= 0 && !dut.sd_spi_i.native_i.w_dma_busy) begin
         if (int'(runcyc_all - ab_t0) > ab_max[ab_kind]) ab_max[ab_kind] = int'(runcyc_all - ab_t0);
         ab_n[ab_kind]++;
         ab_t0 = -1;
      end
      if (dut.w_sd_dma_req && !dut.w_sd_dma_grant) bw++;
      else if (dut.w_sd_dma_req && dut.w_sd_dma_grant && bw >= 0) begin
         bw_h[bw < 16 ? 0 : bw < 64 ? 1 : bw < 256 ? 2 : bw < 1024 ? 3 : bw < 2048 ? 4 : bw < 4096 ? 5 : 6]++;
         if (bw > bw_max) bw_max = bw;
         bw = -1;
      end
      if (!dut.w_sd_dma_req) bw = 0;
   end
   // cache maintenance timing (SD DMA spec v1.2 walk timing): one line per
   // D-cache walk ("WALK range|whole lines= dirty_wb= cycles=", from the
   // WAIT -> MAINT edge to MNT_BUSY's drop, queue wait excluded) and per
   // RANGE_IINV sweep ("RKSWEEP lines= cycles=", MNT_BUSY's share)
   longint wk_t0 = 0, rk_t0 = 0;
   int wk_wb = 0, wk_n = 0, rk_n = 0;
   bit wk_q = 0, rk_q = 0, wk_rng = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.mem_read_write.r_mnt_active && !wk_q) begin
         wk_t0 = runcyc_all; wk_wb = 0; wk_rng = dut.mem_read_write.r_mnt_rng;
         wk_n = dut.mem_read_write.r_mnt_rng ? int'(dut.mem_read_write.r_rw_left) : 2048;
      end
      if (dut.mem_read_write.state == 16'd16384 && dut.mem_read_write.w_cache_ddr_ready) wk_wb++;
      if (!dut.mem_read_write.r_mnt_active && wk_q)
         $display("TB_SOC: WALK %s sets=%0d dirty_wb=%0d cycles=%0d", wk_rng ? "range" : "whole", wk_n, wk_wb, runcyc_all - wk_t0);
      wk_q = dut.mem_read_write.r_mnt_active;
      if (dut.w_ic_rk_busy && !rk_q) begin rk_t0 = runcyc_all; rk_n = int'(dut.r_cop_n); end
      if (!dut.w_ic_rk_busy && rk_q)
         $display("TB_SOC: RKSWEEP lines=%0d cycles=%0d", rk_n, runcyc_all - rk_t0);
      rk_q = dut.w_ic_rk_busy;
   end
   // maintenance-op accounting (SD DMA review 1): every whole-cache go
   // (FLUSH / INVALIDATE, or a range op turned into one) must be followed by
   // a whole walk that STARTS after it (a walk starting in the go's own
   // cycle froze its mode before it) and, for an invalidating go, by an
   // INVALIDATE walk; every range go that takes the slot, by a range walk
   // starting after it.  An op still owed at HALTED, with MNT_BUSY = 0,
   // is lost (FATAL, gated like the consistency checker).
   bit mo_any = 0, mo_inv = 0, mo_rng = 0;
   int mo_lost = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.mem_read_write.w_whole_start) begin
         mo_any = 0;
         if (dut.mem_read_write.r_mnt_mode) mo_inv = 0;
      end
      if (dut.mem_read_write.w_rng_start) mo_rng = 0;
      if (dut.mem_read_write.w_wgo) begin
         // (a pending range op absorbed here is owed by the whole walk now:
         // w_wgo_inv includes its invalidate)
         if (dut.mem_read_write.w_rng_pq) mo_rng = 0;
         mo_any = 1;
         if (dut.mem_read_write.w_wgo_inv) mo_inv = 1;
      end
      if (dut.mem_read_write.i_rng_go && !dut.mem_read_write.w_rng_cvt) mo_rng = 1;
   end
   task automatic mo_check();
      if (!dut.w_cache_mnt_busy && (mo_any || mo_inv || mo_rng))
         $display("TB_SOC: FATAL consistency: a maintenance op was lost (whole %0d, invalidate %0d, range %0d)", mo_any, mo_inv, mo_rng);
   endtask
   // +DCP<k>=<hex PA> (k = 0..7): the D-cache's state of that line at HALTED
   task automatic dcp_dump();
      for (int k = 0; k < 8; k++) begin
         logic [31:0] pa;
         if ($value$plusargs($sformatf("DCP%0d=%%h", k), pa)) begin
            automatic int ix = pa[14:5];
            automatic logic h0 = dut.mem_read_write.cache_val_addr_way0[ix] == {1'b1, pa[31:15]};
            automatic logic h1 = dut.mem_read_write.cache_val_addr_way1[ix] == {1'b1, pa[31:15]};
            automatic logic d  = h0 ? dut.mem_read_write.cache_dirty_way0[ix] : h1 ? dut.mem_read_write.cache_dirty_way1[ix] : 1'b0;
            $display("TB_SOC: DCP%0d %08x valid=%0d dirty=%0d", k, pa, h0 || h1, d);
         end
      end
   endtask
`else
   task automatic dcp_dump(); endtask
   task automatic mo_check(); endtask
`endif

   // ---------------- L2 drains in VGA's / DMA's window (review 1 M2) --------
   int rt_bad = 0, rt_need = 0;
`ifdef L2_PROBE
`define L2C dut.mem_read_write.g_l2.l2_cache
   // the need (tb_l2's rt_need_now): the request in F_DEC — a read miss of
   // a buffered line, a cache read / write allocating over a dirty victim
   // with the buffer full, a posted write with the buffer full (or not
   // empty while the walk holds the next slot) — or the walk: a dirty way
   // and the buffer full, or its end with entries left.  A DMA / VGA read
   // miss needs no slot.
   function automatic bit rt_need_now();
      bit hit, many, lk, dmaq, alloc, vdirty, full, wbm, post, wk_wb;
      hit   = `L2C.r_hit; many = `L2C.r_many; lk = `L2C.q_lk; dmaq = `L2C.q_dma;
      alloc = `L2C.r_en && !`L2C.r_walking;
      vdirty = `L2C.r_td[`L2C.r_vic] != 2'b00;
      full  = `L2C.wb_v == 2'b11;
      wbm   = (`L2C.wb_v[0] && `L2C.wb_line[0] == `L2C.q_addr[26:6]) ||
              (`L2C.wb_v[1] && `L2C.wb_line[1] == `L2C.q_addr[26:6]);
      post  = !(lk && !dmaq && many) && !(alloc && lk && !dmaq && !many) && !(lk && dmaq && hit);
      wk_wb = 0;
`ifdef L2_PROBE_WK
      // (review 2's walk terms; run.py defines L2_PROBE_WK for an RTL that
      // has them — an older L2 under this tb_soc has no r_wk_wb)
      wk_wb = `L2C.r_wk_wb;
      if (`L2C.r_walk_end && `L2C.r_wk_snap && `L2C.r_wk_fl != 2'b00) return 1;
`endif
      if (wk_wb && full) return 1;
      if (`L2C.f_st != 4'd2) return 0;
      if (!`L2C.q_wr)
         return !(lk && hit) && (wbm || (alloc && lk && !dmaq && !many && vdirty && full));
      return (alloc && lk && !dmaq && !many && vdirty && full) ||
             (post && (full || (wk_wb && `L2C.wb_v != 2'b00)));
   endfunction
   bit rt_pend = 0, rt_pneed = 0;
   always @(posedge dut.i_Clk) begin
      // a drain started on the last edge (E_IDLE -> E_WR): judged by that
      // cycle's state
      if (rt_pend && `L2C.e_st == 2'd2) begin
         if (rt_pneed) rt_need++; else rt_bad++;
      end
      rt_pend  = `L2C.e_st == 2'd0 && (`L2C.i_rt_req || `L2C.i_dma);
      rt_pneed = rt_need_now();
   end
`undef L2C
`endif

   // ---------------- VGA scanout probe (HW_PERF_PLAN.md §6) ----------------
   // +VGA_V0=<line>: 2000 cycles into PIPE_RUN, move the raster to <line>
   // (a sim shortcut to the next vblank, where the scanout configuration
   // latches).  Reported at HALTED: lines fetched, the worst fetch time of a
   // line (cycles from its start; a native 640-px line has 3200 before it is
   // shown) and the display lines that starved.
   int vga_v0 = -1, vga_lines = 0, vga_under = 0;
   longint vga_rd_cyc = 0, vga_rd_n = 0, vga_gnt_cyc = 0;
   int vga_age_max = 0;
   initial begin
      if ($value$plusargs("VGA_V0=%d", vga_v0)) begin
         wait (dut.st.SM == PIPE_RUN);
         repeat (2000) @(posedge dut.i_Clk);
         dut.vga_ctrl_i.timing.r_v = 10'(vga_v0);
      end
   end
   always @(posedge dut.i_Clk) begin
      if (dut.vga_ctrl_i.scanout.r_wcnt == 3'd1 && dut.vga_ctrl_i.scanout.r_wlast) begin
         vga_lines++;
         if (int'(dut.vga_ctrl_i.scanout.r_age) > vga_age_max) vga_age_max = int'(dut.vga_ctrl_i.scanout.r_age);
      end
      if (dut.vga_ctrl_i.scanout.o_underflow) vga_under++;
      // the scanout's reads: cycles with DV up (each read's latency) and
      // cycles waiting for the grant
      if (dut.vga_ctrl_i.scanout.o_dma_read_DV) vga_rd_cyc++;
      if (dut.vga_ctrl_i.scanout.o_dma_read_DV && dut.vga_ctrl_i.scanout.i_dma_ready) vga_rd_n++;
      if (dut.vga_ctrl_i.scanout.o_dma_req && !dut.vga_ctrl_i.scanout.i_dma_grant) vga_gnt_cyc++;
   end

   // ---------------- a CPU_RESETN press mid-DMA-tenure (HW_PERF_PLAN §8 3d)
   // +RSTORPH=<k>: at the k-th start of a cycle run in which a DMA master
   // holds the DDR grant with its read DV up (its read accepted below, or
   // about to be) while the D-cache waits with a request of its own, pull
   // CPU_RESETN low for 40 board cycles: the DMA masters reset (their req
   // drops with no done: the arbiter's orphaned grant), the memory system
   // does not.  4,000 cycles later the run ends with the consistency
   // checker's count (an answer to the dead master's read delivered to the
   // D-cache is a wrong read answer).
   int rst_k = 0, rst_seen = 0;
   bit rst_go = 0, rst_c_q = 0;
   always @(posedge dut.i_Clk) if (rst_k != 0 && !rst_go) begin
      // (the read accepted below: the L2's front end busy with a DMA read,
      // or ddr2_control in a read state)
      automatic bit acc =
`ifdef L2_PROBE
         dut.mem_read_write.`DDRPATH.f_st != 0 && dut.mem_read_write.`DDRPATH.q_dma && !dut.mem_read_write.`DDRPATH.q_wr;
`else
         dut.mem_read_write.`DDRPATH.state inside {4'd4, 4'd5, 4'd7};
`endif
      automatic bit c = (dut.mem_read_write.r_grant_vga || dut.mem_read_write.r_grant_blit ||
                         dut.mem_read_write.r_grant_sd || dut.mem_read_write.r_grant_c2) &&
                        dut.mem_read_write.mig_read_DV && dut.mem_read_write.o_ddr_mem_read_DV && acc;
      if (c && !rst_c_q) begin
         rst_seen++;
         if (rst_seen == rst_k) rst_go = 1;
      end
      rst_c_q = c;
   end
   initial begin
      if ($value$plusargs("RSTORPH=%d", rst_k)) begin
         // the framebuffer the scanout reads (4 MiB, 600 KiB) holds a
         // pattern, in DDR and in the checker's reference: an answer to a
         // scanout read then differs from any other line's
         wait (dut.st.SM == PIPE_RUN);
         for (int i = 0; i < 640 * 480 * 2 / 8; i++) begin
            automatic logic [31:0] a = 32'h0040_0000 + 32'(8 * i);
            // (the model's 16 B burst: beat 0 = the dword at +8)
            dut.mem_read_write.`DDRPATH.mig_7series_0.mem_lo[(a >> 3) ^ 1] = {~a, a};
            ref_m[a] = {~a, a};
         end
         wait (rst_go);
         $display("TB_SOC: RSTORPH reset at grant vga=%0d blit=%0d sd=%0d c2=%0d", dut.mem_read_write.r_grant_vga,
                  dut.mem_read_write.r_grant_blit, dut.mem_read_write.r_grant_sd, dut.mem_read_write.r_grant_c2);
         cpu_resetn = 1'b0;
         repeat (40) @(posedge clk100);
         cpu_resetn = 1'b1;
         repeat (4000) @(posedge dut.i_Clk);
         $display("TB_SOC: RSTORPH done consistency checks=%0d errors=%0d", cc_n, cc_err);
         $finish;
      end
   end

   // ---------------- end / progress ----------------
   longint last_i = -1;
   int     idle_n = 0;
   longint maxi = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.st.SM == HALTED) begin
         // the D-cache's memory transactions (its DDR traffic without an L2)
         $display("TB_SOC: DDR reads=%0d writes=%0d", fp_rd, fp_wr);
         $display("TB_SOC: MIG bursts rd=%0d wr=%0d; consistency checks=%0d errors=%0d",
                  dut.mem_read_write.`DDRPATH.mig_7series_0.n_rd,
                  dut.mem_read_write.`DDRPATH.mig_7series_0.n_wr, cc_n, cc_err);
         $display("TB_SOC: MIG activates=%0d conflicts=%0d refreshes=%0d wtr=%0d",
                  dut.mem_read_write.`DDRPATH.mig_7series_0.n_act,
                  dut.mem_read_write.`DDRPATH.mig_7series_0.n_conf,
                  dut.mem_read_write.`DDRPATH.mig_7series_0.n_ref,
                  dut.mem_read_write.`DDRPATH.mig_7series_0.n_wtr);
         $display("TB_SOC: VGA lines=%0d max_fetch=%0d underflows=%0d", vga_lines, vga_age_max, vga_under);
         $display("TB_SOC: SDC cmds=%0d rd_blocks=%0d wr_blocks=%0d wr_bad=%0d host_err=%0d rd_bad_sent=%0d sd_dma_wr=%0d sd_dma_rd=%0d",
                  sdcard.n_cmds, sdcard.blocks_read, sdcard.blocks_written, sdcard.wr_bad, sdcard.host_err,
                  sdcard.rd_bad_sent, sd_dma_wr, sd_dma_rd);
`ifdef SD_DMA_PROBE
         $display("TB_SOC: SDLAT req_to_ready_max=%0d dv_to_ready_max=%0d vga_grant_wait_max=%0d", sd_lat_max, sd_dv_max, vga_gw_max);
         $display("TB_SOC: SDABORT en_n=%0d en_max=%0d fifo_n=%0d fifo_max=%0d busw_max=%0d busw_hist=%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                  ab_n[0], ab_max[0], ab_n[1], ab_max[1], bw_max, bw_h[0], bw_h[1], bw_h[2], bw_h[3], bw_h[4], bw_h[5], bw_h[6]);
`endif
         $display("TB_SOC: VGA reads=%0d read_cycles=%0d grant_cyc=%0d", vga_rd_n, vga_rd_cyc, vga_gnt_cyc);
`ifdef L2_PROBE
         $display("TB_SOC: L2 rt drains bad=%0d need=%0d", rt_bad, rt_need);
`endif
         mo_check();
         dcp_dump();
         // the final registers (runs without a retire trace)
         begin
            string rs = "TB_SOC: REGS";
            for (int i = 0; i < 16; i++) rs = {rs, $sformatf(" r%0d=%016x", i, dut.pipeline_core_i.rf[i])};
            $display("%s", rs);
         end
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

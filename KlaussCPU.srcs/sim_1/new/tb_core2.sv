//===========================================================================
// tb_core2 — AMP P1+P2a gate: core2_subsys standalone.
//
// Drives the mailbox MMIO exactly like the top level does (level-held DV,
// held address) and models the mem_read_write master-C arbiter + DDR with
// the ddr2_control raw dword order (addressed dword in the HIGH half).
//
// Phases:
//   1 (P1)  local hello: window-load +IMAGE, run from BRAM, log identical,
//           HALT park.
//   2 (P2a) execute-from-DDR: the same (position-independent) image placed
//           at 0x07E0_0000 in the DDR model, START_PC=0x07E0_0020 — every
//           fetch crosses the uncached window.
//   3 (P2a) DDR data: +IMAGE2 (ddr_sum) from local BRAM reads 8 dwords at
//           0x0100_0000, sums, WRITES the sum to 0x0100_0100, prints "sum".
//           TB checks the stored dword.
//
// Plusargs: +IMAGE=<hello.mem> +NDW=<n> +IMAGE2=<ddr_sum.mem> +NDW2=<n>
//===========================================================================
module tb_core2;
   logic clk = 0, rst_l = 0;

   mmio_if bus();

   logic         c2ddr_req, c2ddr_done, c2ddr_wdv, c2ddr_rdv;
   logic [31:0]  c2ddr_addr;
   logic [127:0] c2ddr_wdata;
   logic [15:0]  c2ddr_wmask;
   logic [127:0] c2ddr_rdata;
   logic         c2ddr_ready, c2ddr_grant;
   logic         c2e_owner, c2e_wdv, c2e_rdv;
   logic [31:0]  c2e_addr;
   logic [63:0]  c2e_wdata, c2e_rdata;
   logic [7:0]   c2e_be;
   logic         c2e_ready;
   localparam logic [63:0] TB_CLOCK_MS = 64'h0000_0000_0001_2345;

   core2_subsys dut (
      .i_Clk(clk), .i_Rst_L(rst_l), .mmio(bus),
      .o_ddr_req(c2ddr_req), .o_ddr_done(c2ddr_done),
      .o_ddr_write_DV(c2ddr_wdv), .o_ddr_read_DV(c2ddr_rdv),
      .o_ddr_addr(c2ddr_addr), .o_ddr_write_data(c2ddr_wdata),
      .o_ddr_wdf_mask(c2ddr_wmask), .i_ddr_read_data(c2ddr_rdata),
      .i_ddr_ready(c2ddr_ready), .i_ddr_grant(c2ddr_grant),
      .o_eth_owner(c2e_owner), .o_eth_write_DV(c2e_wdv),
      .o_eth_read_DV(c2e_rdv), .o_eth_addr(c2e_addr),
      .o_eth_write_data(c2e_wdata), .o_eth_byte_en(c2e_be),
      .i_eth_read_data(c2e_rdata), .i_eth_ready(c2e_ready),
      .i_clock_ms(TB_CLOCK_MS)
   );

   // ---------------------- behavioral eth bridge (top-mux + bridge shape) --
   // Only routed when core 2 owns eth (like the top's OWNER mux).  Registers
   // at word granularity; ready = 1-cycle pulse a few cycles after DV, data
   // registered; then ignores the held strobe until it drops (S_COOL).
   logic [31:0] ethreg [logic [15:0]];
   int          eth_lat;
   logic        eth_cool;
   always @(posedge clk) begin
      c2e_ready <= 1'b0;
      if (!rst_l) begin
         eth_lat  <= 0;
         eth_cool <= 1'b0;
      end else if (!(c2e_wdv || c2e_rdv)) begin
         eth_cool <= 1'b0;
         eth_lat  <= 3;
      end else if (c2e_owner && !eth_cool) begin
         if (eth_lat > 0) eth_lat <= eth_lat - 1;
         else begin
            if (c2e_wdv) begin
               if (c2e_be[3:0] != 0) ethreg[c2e_addr[15:0] & 16'hFFF8] = c2e_wdata[31:0];
               if (c2e_be[7:4] != 0) ethreg[(c2e_addr[15:0] & 16'hFFF8) + 4] = c2e_wdata[63:32];
            end else begin
               c2e_rdata <= { ethreg.exists((c2e_addr[15:0] & 16'hFFF8) + 4) ?
                                 ethreg[(c2e_addr[15:0] & 16'hFFF8) + 4] : 32'h0,
                              ethreg.exists(c2e_addr[15:0] & 16'hFFF8) ?
                                 ethreg[c2e_addr[15:0] & 16'hFFF8] : 32'h0 };
            end
            c2e_ready <= 1'b1;
            eth_cool  <= 1'b1;
         end
      end
   end

   always #5 clk = ~clk;

   // ------------------------- behavioral master-C arbiter + DDR (sparse) ---
   // Raw order mirrors ddr2_control: read_data[127:64] = dword@addr,
   // [63:0] = dword@addr+8 (the blitter-corruption convention).  Grant one
   // cycle after req; ready after a fixed latency; done releases.
   logic [63:0] ddr [logic [28:0]];    // keyed by dword address (addr>>3)
   int          mdl_lat;

   function automatic logic [63:0] ddr_rd(input logic [31:0] a);
      if (ddr.exists(a[31:3])) return ddr[a[31:3]];
      return 64'h0;
   endfunction

   task automatic ddr_masked_wr(input logic [31:0] a, input logic [63:0] d,
                                input logic [7:0] mask);
      logic [63:0] cur;
      cur = ddr_rd(a);
      for (int i = 0; i < 8; i++) begin
         if (!mask[i]) cur[i*8 +: 8] = d[i*8 +: 8];
      end
      ddr[a[31:3]] = cur;
   endtask

   always @(posedge clk) begin
      c2ddr_ready <= 1'b0;
      if (!rst_l) begin
         c2ddr_grant <= 1'b0;
         mdl_lat     <= 0;
      end else if (!c2ddr_grant) begin
         if (c2ddr_req) begin
            c2ddr_grant <= 1'b1;
            mdl_lat     <= 4;                       // MIG-ish latency
         end
      end else begin
         if ((c2ddr_rdv || c2ddr_wdv) && !c2ddr_ready) begin
            if (mdl_lat > 0) begin
               mdl_lat <= mdl_lat - 1;
            end else begin
               if (c2ddr_rdv) begin
                  c2ddr_rdata[127:64] <= ddr_rd({c2ddr_addr[31:4], 4'b0});
                  c2ddr_rdata[63:0]   <= ddr_rd({c2ddr_addr[31:4], 4'b0} + 32'd8);
               end else begin
                  ddr_masked_wr({c2ddr_addr[31:4], 4'b0},
                                c2ddr_wdata[127:64], c2ddr_wmask[15:8]);
                  ddr_masked_wr({c2ddr_addr[31:4], 4'b0} + 32'd8,
                                c2ddr_wdata[63:0], c2ddr_wmask[7:0]);
               end
               c2ddr_ready <= 1'b1;
               mdl_lat     <= 4;
            end
         end
         if (c2ddr_done) c2ddr_grant <= 1'b0;
      end
   end

   // ------------------------------------------------- master-side MMIO tasks
   task automatic mmio_wr(input logic [15:0] off, input logic [63:0] data);
      @(negedge clk);
      bus.addr       = {16'hF010, off};
      bus.write_data = data;
      bus.byte_en    = 8'hFF;
      bus.write_DV   = 1'b1;
      repeat (4) @(negedge clk);
      bus.write_DV   = 1'b0;
      repeat (2) @(negedge clk);
   endtask

   task automatic mmio_rd(input logic [15:0] off, output logic [63:0] data);
      @(negedge clk);
      bus.addr    = {16'hF010, off};
      bus.read_DV = 1'b1;
      repeat (3) @(negedge clk);
      data = bus.read_data;
      bus.read_DV = 1'b0;
      repeat (2) @(negedge clk);
   endtask

   // ---------------------------------------------------------- test helpers
   logic [63:0] img  [0:16383];
   logic [63:0] img2 [0:16383];
   string       image_f, image2_f;
   int          ndw, ndw2;
   logic [63:0] d;
   byte         got  [$];
   byte         want [$];
   int          idle;

   task automatic load_window(input logic [63:0] arr [0:16383], input int n);
      mmio_wr(16'h0010, 64'h0);
      for (int i = 0; i < n; i++) mmio_wr(16'h0018, arr[i]);
   endtask

   task automatic run_and_drain(input logic [31:0] start_pc, input string exp);
      want.delete();
      got.delete();
      for (int i = 0; i < exp.len(); i++) want.push_back(byte'(exp[i]));
      mmio_wr(16'h0008, {32'h0, start_pc});
      mmio_wr(16'h0000, 64'h1);                       // RUN
      idle = 0;
      while (got.size() < want.size() && idle < 20000) begin
         mmio_rd(16'h0020, d);
         if (d[8]) begin
            got.push_back(byte'(d[7:0]));
            mmio_wr(16'h0020, 64'h0);
            idle = 0;
         end else idle++;
      end
      idle = 0;
      d = '0;
      while (!d[1] && idle < 20000) begin
         mmio_rd(16'h0000, d);
         idle++;
      end
      if (!d[1]) $fatal(1, "core 2 never parked (CTRL=%h)", d);
      if (d[4:2] != 3'd0) $fatal(1, "park kind %0d != HALT", d[4:2]);
      if (got.size() != want.size()) begin
         for (int i = 0; i < got.size(); i++) $write("%c", got[i]);
         $fatal(1, "log bytes: got %0d want %0d", got.size(), want.size());
      end
      for (int i = 0; i < want.size(); i++) begin
         if (got[i] !== want[i])
            $fatal(1, "log byte %0d: got %02h want %02h", i, got[i], want[i]);
      end
      mmio_wr(16'h0000, 64'h0);                       // stop / reset core 2
   endtask

   // ------------------------------------------------------------------ test
   logic [63:0] sum_exp;

   initial begin
      if (!$value$plusargs("IMAGE=%s", image_f))   $fatal(1, "need +IMAGE");
      if (!$value$plusargs("NDW=%d", ndw))         $fatal(1, "need +NDW");
      if (!$value$plusargs("IMAGE2=%s", image2_f)) $fatal(1, "need +IMAGE2");
      if (!$value$plusargs("NDW2=%d", ndw2))       $fatal(1, "need +NDW2");
      $readmemh(image_f, img);
      $readmemh(image2_f, img2);

      bus.write_DV = 0;
      bus.read_DV  = 0;
      repeat (10) @(negedge clk);
      rst_l = 1;
      repeat (10) @(negedge clk);

      // ---- Phase 1 (P1): hello from local BRAM --------------------------
      load_window(img, ndw);
      mmio_wr(16'h0010, 64'h20);
      mmio_rd(16'h0018, d);
      if (d !== img[4]) $fatal(1, "readback @0x20: got %h want %h", d, img[4]);
      run_and_drain(32'h20, "hello from core 2\n");
      $display("TB_CORE2: phase 1 (local BRAM hello) PASS");

      // ---- Phase 2 (P2a): the same image executed FROM DDR --------------
      for (int i = 0; i < ndw; i++) ddr[(32'h07E0_0000 >> 3) + i] = img[i];
      run_and_drain(32'h07E0_0020, "hello from core 2\n");
      $display("TB_CORE2: phase 2 (execute from DDR, uncached fetch) PASS");

      // ---- Phase 2b: cached-fetch loop from the text window -------------
      // A tight countdown loop at 0x07E0_0020 (absolute back-jump): every
      // iteration re-fetches the same line — miss+install once, hits after.
      begin
         string image3_f;
         int    ndw3;
         if ($value$plusargs("IMAGE3=%s", image3_f) &&
             $value$plusargs("NDW3=%d", ndw3)) begin
            logic [63:0] img3 [0:63];
            $readmemh(image3_f, img3);
            for (int i = 0; i < ndw3; i++)
               ddr[(32'h07E0_0000 >> 3) + i] = img3[i];
            run_and_drain(32'h07E0_0020, "");
            $display("TB_CORE2: phase 2b (cached-fetch loop, %0d dw) PASS",
                     ndw3);
         end
      end

      // ---- Phase 3 (P2a): DDR data reads + write ------------------------
      sum_exp = 0;
      for (int i = 0; i < 8; i++) begin
         ddr[(32'h0100_0000 >> 3) + i] = {$urandom(), $urandom()};
         sum_exp += ddr[(32'h0100_0000 >> 3) + i];
      end
      ddr[32'h0100_0100 >> 3] = 64'hDEAD_DEAD_DEAD_DEAD;
      load_window(img2, ndw2);
      run_and_drain(32'h20, "sum\n");
      if (ddr[32'h0100_0100 >> 3] !== sum_exp)
         $fatal(1, "DDR sum: got %h want %h", ddr[32'h0100_0100 >> 3], sum_exp);
      $display("TB_CORE2: phase 3 (DDR data read x8 + write, sum %h) PASS",
               sum_exp);

      // ---- Phase 4 (P3): LiteEth window + OWNER + clock_ms mirror -------
      begin
         string image4_f;
         int    ndw4;
         if ($value$plusargs("IMAGE4=%s", image4_f) &&
             $value$plusargs("NDW4=%d", ndw4)) begin
            logic [63:0] img4 [0:63];
            $readmemh(image4_f, img4);
            // A: core 1 owns eth -> core-2 access must complete, read 0.
            mmio_wr(16'h0038, 64'h0);
            ddr[32'h0100_0200 >> 3] = 64'hBAD0_BAD0_BAD0_BAD0;
            ddr[32'h0100_0208 >> 3] = 64'hBAD0_BAD0_BAD0_BAD0;
            for (int i = 0; i < ndw4; i++) img[i] = img4[i];
            load_window(img, ndw4);
            run_and_drain(32'h20, "");
            if (ddr[32'h0100_0200 >> 3] !== 64'h0)
               $fatal(1, "eth non-owner read: got %h want 0", ddr[32'h0100_0200 >> 3]);
            if (ddr[32'h0100_0208 >> 3] !== TB_CLOCK_MS)
               $fatal(1, "clock_ms mirror: got %h want %h",
                      ddr[32'h0100_0208 >> 3], TB_CLOCK_MS);
            $display("TB_CORE2: phase 4A (non-owner eth reads 0, clock_ms mirror) PASS");
            // B: core 2 owns eth -> write lands, readback matches.
            mmio_wr(16'h0038, 64'h1);
            mmio_rd(16'h0038, d);
            if (d[0] !== 1'b1) $fatal(1, "C2_ETH_OWNER readback %h", d);
            ddr[32'h0100_0200 >> 3] = 64'hBAD0_BAD0_BAD0_BAD0;
            run_and_drain(32'h20, "");
            if (ddr[32'h0100_0200 >> 3] !== 64'h5A5A)
               $fatal(1, "eth owner readback: got %h want 5a5a", ddr[32'h0100_0200 >> 3]);
            if (!ethreg.exists(16'h0010) || ethreg[16'h0010] !== 32'h5A5A)
               $fatal(1, "eth model reg not written");
            mmio_wr(16'h0038, 64'h0);
            $display("TB_CORE2: phase 4B (owner eth write+readback) PASS");
         end
      end

      $display("TB_CORE2: ALL PHASES PASS");
      $finish;
   end

   initial begin
      repeat (6_000_000) @(posedge clk);
      $fatal(1, "TB_CORE2: global timeout");
   end
endmodule

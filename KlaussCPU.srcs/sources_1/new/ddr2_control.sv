`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 05/10/2021 10:12:31 AM
// Design Name:
// Module Name: ddr2_control
// Project Name:
// Target Devices:
// Tool Versions:
// Description: MIG 2:1 UI wrapper — 128-bit cache line <-> two 64-bit UI beats.
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module ddr2_control (
    inout [15:0] ddr2_dq,
    inout [1:0] ddr2_dqs_n,
    inout [1:0] ddr2_dqs_p,
    // Outputs
    output [12:0] ddr2_addr,
    output [2:0] ddr2_ba,
    output ddr2_ras_n,
    output ddr2_cas_n,
    output ddr2_we_n,
    //output ddr2_reset_n,
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
    input i_mem_wide,                    // 1 = 32 B line (two pipelined BL8 bursts); 0 = 128 b (blitter)
    input [1:0] i_mem_dw_off,            // M10a CWF: requested dword within the 32 B line (wide reads only)
    input [255:0] i_mem_write_data,
    inout [15:0] i_app_wdf_mask,
    output logic [255:0] o_mem_read_data,
    output logic o_mem_ready,
    // M10a critical-word-first early channel (wide reads only). The burst
    // holding the requested dword is issued FIRST, so the requested dword
    // always arrives on beat 0 (odd dw_off) or beat 1 (even dw_off) — never
    // on the final beat. o_mem_dw_ready pulses for one cycle when it has been
    // captured, 3+ cycles before o_mem_ready; the full 32 B line in
    // o_mem_read_data completes as usual. For even dw_off the companion
    // (dw_off+1) dword rides beat 0 and is exposed on o_mem_rd_dw_next
    // (o_mem_dw_next_ok qualifies it, stable through the transaction).
    output logic [63:0] o_mem_rd_dw,
    output logic [63:0] o_mem_rd_dw_next,
    output       o_mem_dw_next_ok,
    output logic o_mem_dw_ready,
    output o_calib_done,          // MIG init_calib_complete (DDR2 ready)
    output o_ui_clk               // MIG ui_clk (100 MHz at 2:1) — the CPU clock (CPU runs synchronous to it)

);


   wire calib_done;
   wire ui_clk;
   wire ui_clk_sync_rst;
   assign o_calib_done = calib_done;
   assign o_ui_clk = ui_clk;      // expose the MIG UI clock to clock the whole CPU synchronously

   logic [26:0] app_addr = 0;
   logic [2:0] app_cmd = 0;
   logic app_en;
   wire app_rdy;

   // 2:1 PHY ratio: the MIG user interface is 64-bit (APP_DATA_WIDTH =
   // 2*nCK_PER_CLK*16 = 2*2*16). A 128-bit cache line is therefore TWO 64-bit
   // UI beats. beat0 = line[63:0] (pushed/returned first), beat1 = line[127:64].
   logic  [63:0] app_wdf_data;
   logic         app_wdf_end;        // 0 on beat0, 1 on beat1 (last beat of the burst)
   logic         app_wdf_wren;
   wire        app_wdf_rdy;

   wire [63:0] app_rd_data;
   logic  [ 7:0] app_wdf_mask;       // 8 byte-mask bits per 64-bit beat
   wire        app_rd_data_end;
   wire        app_rd_data_valid;

   wire app_sr_req = 0;
   wire app_ref_req = 0;
   wire app_zq_req = 0;
   wire app_sr_active;
   wire app_ref_ack;
   wire app_zq_ack;

   // -------------------------------------------------------------------------
   // The cache and ddr2_control are in the SAME ui_clk domain (the whole CPU
   // runs synchronous to the 100 MHz MIG ui_clk), so the DV control signals do
   // not cross clocks — there are no 2-FF synchronisers and DV is sampled
   // directly. The `synced_*` names are retained from the old async path; they
   // are now plain wire aliases of the inputs.
   // -------------------------------------------------------------------------
   wire synced_write_dv = i_mem_write_DV;
   wire synced_read_dv  = i_mem_read_DV;



   localparam IDLE = 4'd0;
   localparam WAIT = 4'd1;
   localparam WRITE = 4'd2;
   localparam WRITE_DONE = 4'd3;
   localparam READ = 4'd4;
   localparam READ_DONE = 4'd5;
   localparam WRITE_B1 = 4'd6;   // push the second 64-bit write beat (2:1 UI)
   localparam READ_CMD2 = 4'd7;  // BL16 spike: issue the 2nd pipelined BL8 read command
   logic [3:0] state = IDLE;

   logic rd_beat = 1'b0;           // 0 = awaiting beat0 (line[63:0]), 1 = beat1 (line[127:64])
   // 32 B lines: a pipelined dual-BL8 read returns 4 beats reassembled into the
   // 256-bit line; the blitter (narrow) uses a single BL8. rd_cnt counts the beats.
   logic [1:0] rd_cnt   = 2'd0;
   logic       r_wide   = 1'b0;    // latched i_mem_wide for the current transaction
   logic       wr_burst = 1'b0;    // wide writeback: 0 = burst_lo (base), 1 = burst_hi (base+16)
   // M10a CWF: latched request-dword offset and burst order for the current read.
   // r_hi_first = 1 when burst_hi (base+16, dw2/dw3) is issued before burst_lo.
   logic [1:0] r_dw_off  = 2'd0;
   logic       r_hi_first = 1'b0;

   // Companion qualifier: for an even-offset request the companion (dw_off+1)
   // dword arrives on beat 0, before the requested dword on beat 1. Stable
   // through the transaction — sampled by the cache on the o_mem_dw_ready pulse.
   assign o_mem_dw_next_ok = r_wide & ~r_dw_off[0];

   parameter CMD_WRITE = 3'b000;
   parameter CMD_READ = 3'b001;

   initial begin
      o_mem_ready    <= 0;
      o_mem_dw_ready <= 0;
   end

   mig_7series_0 mig_7series_0 (
       // DDR2 Physical interface ports
       .ddr2_addr (ddr2_addr),
       .ddr2_ba   (ddr2_ba),
       .ddr2_cas_n(ddr2_cas_n),
       .ddr2_ck_n (ddr2_ck_n),
       .ddr2_ck_p (ddr2_ck_p),
       .ddr2_cke  (ddr2_cke),
       .ddr2_ras_n(ddr2_ras_n),
       // .ddr2_reset_n(ddr2_reset_n),
       .ddr2_we_n (ddr2_we_n),
       .ddr2_dq   (ddr2_dq),
       .ddr2_dqs_n(ddr2_dqs_n),
       .ddr2_dqs_p(ddr2_dqs_p),
       .ddr2_cs_n (ddr2_cs_n),
       .ddr2_dm   (ddr2_dm),
       .ddr2_odt  (ddr2_odt),

       .init_calib_complete(calib_done),

       // User interface ports
       .app_addr         (app_addr),
       .app_cmd          (app_cmd),
       .app_en           (app_en),
       .app_wdf_data     (app_wdf_data),
       .app_wdf_end      (app_wdf_end),
       .app_wdf_wren     (app_wdf_wren),
       .app_rd_data      (app_rd_data),
       .app_rd_data_end  (app_rd_data_end),
       .app_rd_data_valid(app_rd_data_valid),
       .app_rdy          (app_rdy),
       .app_wdf_rdy      (app_wdf_rdy),
       .app_sr_req       (app_sr_req),
       .app_ref_req      (app_ref_req),
       .app_zq_req       (app_zq_req),
       .app_sr_active    (app_sr_active),
       .app_ref_ack      (app_ref_ack),
       .app_zq_ack       (app_zq_ack),
       .ui_clk           (ui_clk),
       .ui_clk_sync_rst  (ui_clk_sync_rst),
       .app_wdf_mask     (app_wdf_mask),
       // Clock and Reset input ports
       .sys_clk_i        (sys_clk_i),

       .sys_rst(resetn)

   );


   always_ff @(posedge ui_clk) begin
      if (ui_clk_sync_rst) begin
         state <= IDLE;
         app_en <= 0;
         app_wdf_wren <= 0;
         app_wdf_end <= 0;
         rd_beat <= 1'b0;
         rd_cnt <= 2'd0;
         r_wide <= 1'b0;
         wr_burst <= 1'b0;
         r_hi_first <= 1'b0;
         o_mem_dw_ready <= 1'b0;
      end else begin
         o_mem_dw_ready <= 1'b0;   // 1-cycle pulse; the READ_DONE capture below overrides
         case (state)
            // IDLE: stay here until BOTH DVs are deasserted before returning to
            // WAIT. This is critical for transaction framing: the cache holds its
            // DV high for several cycles per access (it spans multiple FSM
            // states), so without this gate, after a WRITE_DONE→IDLE→WAIT
            // transition we would re-sample write_dv as still high and immediately
            // re-fire WRITE → another o_mem_ready pulse. The cache's
            // WRITE_FETCH/READ_WAIT then sees that spurious ready as the read
            // completion, latching stale o_mem_read_data into the cache line and
            // corrupting DRAM on subsequent evictions. Holding IDLE until DV=0
            // forces the cache to produce a real rising edge for the next
            // transaction.
            IDLE: begin
               o_mem_ready <= 1'b0;
               if (calib_done & ~synced_write_dv & ~synced_read_dv) begin
                  state <= WAIT;
               end
            end

            WAIT: begin
               if (synced_write_dv) begin
                  state <= WRITE;
               end else if (synced_read_dv) begin
                  state <= READ;
               end
            end

            // 2:1 write, per BL8 burst: one command + TWO 64-bit wdf beats.
            // A 32 B writeback (i_mem_wide) is TWO serial BL8 bursts (writeback is
            // off the CPU critical path, so serial is fine). wr_burst selects
            // burst_lo(base)=line[255:128] then burst_hi(base+16)=line[127:0], with
            // the same per-burst beat order as the read (beat0→[63:0], beat1→[127:64]).
            WRITE: begin                          // command + beat0
               if (app_rdy & app_wdf_rdy) begin
                  app_en       <= 1;
                  app_cmd      <= CMD_WRITE;
                  app_addr     <= wr_burst ? (i_mem_addr[27:1] + 27'd8) : i_mem_addr[27:1];
                  app_wdf_data <= (i_mem_wide & ~wr_burst) ? i_mem_write_data[191:128]  // wide burst_lo: dw1
                                                           : i_mem_write_data[63:0];    // narrow / wide burst_hi: dw3
                  app_wdf_mask <= i_app_wdf_mask[7:0];
                  app_wdf_wren <= 1;
                  app_wdf_end  <= 1'b0;              // beat0 is not the last beat
                  state        <= WRITE_B1;
               end
            end

            WRITE_B1: begin                       // clear cmd; push beat1
               if (app_rdy & app_en) begin
                  app_en <= 0;
               end
               if (app_wdf_rdy & app_wdf_wren) begin   // beat0 accepted → present beat1
                  app_wdf_data <= (i_mem_wide & ~wr_burst) ? i_mem_write_data[255:192]  // wide burst_lo: dw0
                                                           : i_mem_write_data[127:64];  // narrow / wide burst_hi: dw2
                  app_wdf_mask <= i_app_wdf_mask[15:8];
                  app_wdf_end  <= 1'b1;                 // beat1 is the last beat
                  // app_wdf_wren stays high for beat1
                  state        <= WRITE_DONE;
               end
            end

            WRITE_DONE: begin                     // drain command + beat1
               if (app_rdy & app_en) begin
                  app_en <= 0;
               end

               if (app_wdf_rdy & app_wdf_wren) begin   // beat1 accepted
                  app_wdf_wren <= 0;
                  app_wdf_end  <= 1'b0;
               end

               if (~app_en & ~app_wdf_wren) begin
                  if (i_mem_wide & ~wr_burst) begin
                     wr_burst    <= 1'b1;          // wide: now write burst_hi (base + 16 B)
                     state       <= WRITE;
                  end else begin
                     wr_burst    <= 1'b0;
                     o_mem_ready <= 1'b1;
                     state <= IDLE;
                  end
               end
            end


            // 32 B read (i_mem_wide) = two pipelined BL8 bursts; blitter = one BL8.
            // Wide: both commands go back-to-back into the MIG FIFO; 4 beats
            // reassemble the 256-bit line (see READ_DONE map). The 2nd row-hit
            // read pipelines ~free (measured ~+2.6c/miss on silicon).
            // M10a CWF: the burst holding the REQUESTED dword is issued first
            // (r_hi_first when the request is in dw2/dw3), so the requested
            // dword always arrives on beat 0/1 and is exposed early on
            // o_mem_rd_dw / o_mem_dw_ready while the tail beats finish the line.
            READ: begin
               if (app_rdy) begin
                  app_en   <= 1;
                  app_addr <= (i_mem_wide && i_mem_dw_off[1])
                              ? i_mem_addr[27:1] + 27'd8   // CWF: burst_hi (base+16 B) first
                              : i_mem_addr[27:1];          // byte addr → MIG halfword addr
                  app_cmd  <= CMD_READ;
                  rd_cnt   <= 2'd0;
                  r_wide   <= i_mem_wide;
                  r_dw_off <= i_mem_dw_off;
                  r_hi_first <= i_mem_wide && i_mem_dw_off[1];
                  state    <= i_mem_wide ? READ_CMD2 : READ_DONE;  // narrow (blitter) = 1 burst
               end
            end

            READ_CMD2: begin                    // command 1 accepted → present command 2
               if (app_rdy & app_en) begin
                  app_addr <= r_hi_first ? i_mem_addr[27:1]           // CWF: burst_lo second
                                         : i_mem_addr[27:1] + 27'd8;  // burst 2 = base + 16 B
                  // app_en stays high so command 2 is presented next cycle
                  state    <= READ_DONE;
               end
            end

            // 2:1 read: each BL8 burst returns as TWO 64-bit app_rd_data beats.
            // Wide beat→dword map depends on burst order (line layout:
            // dw0[255:192] dw1[191:128] dw2[127:64] dw3[63:0]):
            //   lo-first: beat0=dw1, beat1=dw0, beat2=dw3, beat3=dw2
            //   hi-first: beat0=dw3, beat1=dw2, beat2=dw1, beat3=dw0
            // The requested dword is therefore always beat (r_dw_off[0] ? 0 : 1)
            // and its companion (dw_off+1, even offsets) is always beat 0.
            READ_DONE: begin
               if (app_rdy & app_en) begin
                  app_en <= 0;                   // command 2 accepted
               end

               if (app_rd_data_valid) begin
                  if (r_wide) begin
                     case ({r_hi_first, rd_cnt})
                        3'b0_00: o_mem_read_data[191:128] <= app_rd_data;  // burst_lo beat0 = dw1
                        3'b0_01: o_mem_read_data[255:192] <= app_rd_data;  // burst_lo beat1 = dw0
                        3'b0_10: o_mem_read_data[63:0]    <= app_rd_data;  // burst_hi beat0 = dw3
                        3'b0_11: begin o_mem_read_data[127:64]  <= app_rd_data; o_mem_ready <= 1'b1; state <= IDLE; end // dw2
                        3'b1_00: o_mem_read_data[63:0]    <= app_rd_data;  // burst_hi beat0 = dw3
                        3'b1_01: o_mem_read_data[127:64]  <= app_rd_data;  // burst_hi beat1 = dw2
                        3'b1_10: o_mem_read_data[191:128] <= app_rd_data;  // burst_lo beat0 = dw1
                        3'b1_11: begin o_mem_read_data[255:192] <= app_rd_data; o_mem_ready <= 1'b1; state <= IDLE; end // dw0
                        default: ;
                     endcase
                     // CWF early channel: capture the requested dword the beat
                     // it arrives (beat 0 for odd offsets, beat 1 for even) and
                     // pulse o_mem_dw_ready; the companion of an even-offset
                     // request rides beat 0.
                     if (rd_cnt == {1'b0, ~r_dw_off[0]}) begin
                        o_mem_rd_dw    <= app_rd_data;
                        o_mem_dw_ready <= 1'b1;
                     end
                     if (rd_cnt == 2'd0 && !r_dw_off[0]) begin
                        o_mem_rd_dw_next <= app_rd_data;
                     end
                  end else begin
                     if (rd_cnt == 2'd0) begin
                        o_mem_read_data[63:0]   <= app_rd_data;   // beat0 = line[63:0]
                     end else begin
                        o_mem_read_data[127:64] <= app_rd_data;   // beat1 = line[127:64]
                        o_mem_ready <= 1'b1;
                        state <= IDLE;
                     end
                  end
                  rd_cnt <= rd_cnt + 2'd1;
               end
            end


            default: state <= IDLE;
         endcase
      end
   end



endmodule

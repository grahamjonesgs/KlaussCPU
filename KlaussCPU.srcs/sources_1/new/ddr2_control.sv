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
    input [127:0] i_mem_write_data,
    inout [15:0] i_app_wdf_mask,
    output logic [127:0] o_mem_read_data,
    output logic o_mem_ready,
    output o_calib_done,          // MIG init_calib_complete (DDR2 ready)
    output o_ui_clk               // MIG ui_clk (100 MHz at 2:1) — the CPU clock (CPU runs synchronous to it)

);


   wire calib_done;
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

   wire ui_clk;
   wire ui_clk_sync_rst;

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
   // BL16 SPIKE: 4-beat counter for a pipelined dual-BL8 (256-bit) read. Measures
   // whether a 2nd back-to-back row-hit read pipelines cheaply (~+8c to ready) or
   // serialises (~+54c) — the go/no-go for real 32-byte cache lines.
   logic [1:0] rd_cnt = 2'd0;

   parameter CMD_WRITE = 3'b000;
   parameter CMD_READ = 3'b001;

   initial begin
      o_mem_ready <= 0;
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
      end else begin
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

            // 2:1 write: issue the command once + push TWO 64-bit wdf beats.
            WRITE: begin                          // command + beat0 (line[63:0])
               if (app_rdy & app_wdf_rdy) begin
                  app_en       <= 1;
                  app_cmd      <= CMD_WRITE;
                  app_addr     <= i_mem_addr[27:1]; // byte addr → MIG halfword addr
                  app_wdf_data <= i_mem_write_data[63:0];
                  app_wdf_mask <= i_app_wdf_mask[7:0];
                  app_wdf_wren <= 1;
                  app_wdf_end  <= 1'b0;              // beat0 is not the last beat
                  state        <= WRITE_B1;
               end
            end

            WRITE_B1: begin                       // clear cmd; push beat1 (line[127:64])
               if (app_rdy & app_en) begin
                  app_en <= 0;
               end
               if (app_wdf_rdy & app_wdf_wren) begin   // beat0 accepted → present beat1
                  app_wdf_data <= i_mem_write_data[127:64];
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
                  o_mem_ready <= 1'b1;
                  state <= IDLE;
               end
            end


            // BL16 SPIKE — pipelined dual-BL8 read. Issue command 1 (base), then
            // command 2 (base + 16 B) back-to-back into the MIG command FIFO, then
            // consume 4 × 64-bit beats. The cache uses only the first 128 b
            // (beats 0/1); beats 2/3 (burst 2) are DISCARDED. Purpose: does the 2nd
            // row-hit read pipeline (small delta to o_mem_ready) or serialise
            // (~54c)? That is the go/no-go for real 32-byte lines.
            READ: begin
               if (app_rdy) begin
                  app_en   <= 1;
                  app_addr <= i_mem_addr[27:1]; // byte addr → MIG halfword addr (burst 1 = base)
                  app_cmd  <= CMD_READ;
                  rd_cnt   <= 2'd0;
                  state    <= READ_CMD2;
               end
            end

            READ_CMD2: begin                    // command 1 accepted → present command 2
               if (app_rdy & app_en) begin
                  app_addr <= i_mem_addr[27:1] + 27'd8; // burst 2 = base + 16 B (8 halfwords)
                  // app_en stays high so command 2 is presented next cycle
                  state    <= READ_DONE;
               end
            end

            // 2:1 read: each BL8 burst returns as TWO 64-bit app_rd_data beats.
            // beat0/1 = burst1 = line[63:0]/[127:64]; beat2/3 = burst2 (discarded).
            READ_DONE: begin
               if (app_rdy & app_en) begin
                  app_en <= 0;                   // command 2 accepted
               end

               if (app_rd_data_valid) begin
                  case (rd_cnt)
                     2'd0: o_mem_read_data[63:0]   <= app_rd_data;   // beat0 = line[63:0]
                     2'd1: o_mem_read_data[127:64] <= app_rd_data;   // beat1 = line[127:64]
                     2'd2: ;                                         // beat2 = burst2 low (discard)
                     2'd3: begin o_mem_ready <= 1'b1; state <= IDLE; end // beat3 = burst2 high → done
                     default: ;
                  endcase
                  rd_cnt <= rd_cnt + 2'd1;
               end
            end


            default: state <= IDLE;
         endcase
      end
   end



endmodule

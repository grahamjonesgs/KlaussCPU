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
// Description:
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
    output reg [127:0] o_mem_read_data,
    output reg o_mem_ready,
    output o_calib_done,          // MIG init_calib_complete (DDR2 ready)
    output o_ui_clk               // MIG ui_clk (100 MHz at 2:1) — the CPU clock (P3 synchronous)

);


   wire calib_done;
   assign o_calib_done = calib_done;
   assign o_ui_clk = ui_clk;      // expose the MIG UI clock to clock the whole CPU synchronously

   reg [26:0] app_addr = 0;
   reg [2:0] app_cmd = 0;
   reg app_en;
   wire app_rdy;

   // 2:1 PHY ratio: the MIG user interface is 64-bit (APP_DATA_WIDTH =
   // 2*nCK_PER_CLK*16 = 2*2*16). A 128-bit cache line is therefore TWO 64-bit
   // UI beats. beat0 = line[63:0] (pushed/returned first), beat1 = line[127:64].
   reg  [63:0] app_wdf_data;
   reg         app_wdf_end;        // 0 on beat0, 1 on beat1 (last beat of the burst)
   reg         app_wdf_wren;
   wire        app_wdf_rdy;

   wire [63:0] app_rd_data;
   reg  [ 7:0] app_wdf_mask;       // 8 byte-mask bits per 64-bit beat
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
   // 2-FF synchronisers for control signals crossing from i_Clk (100 MHz CPU
   // clock) into ui_clk (50 MHz MIG UI clock).
   // (* ASYNC_REG = "true" *) tells Vivado to:
   //   • place both FFs in the same slice (tight hold margin)
   //   • exclude the path from setup/hold STA (CDC false-path)
   //   • suppress the TIMING-10 "missing ASYNC_REG" warning
   // Data/address buses (i_mem_addr, i_mem_write_data) need no synchroniser
   // because the CPU holds them stable from the DV assertion until o_mem_ready.
   // -------------------------------------------------------------------------
   (* ASYNC_REG = "true" *) reg sync_wr_dv_0 = 1'b0, sync_wr_dv_1 = 1'b0;
   (* ASYNC_REG = "true" *) reg sync_rd_dv_0 = 1'b0, sync_rd_dv_1 = 1'b0;

   always @(posedge ui_clk or posedge ui_clk_sync_rst) begin
      if (ui_clk_sync_rst) begin
         sync_wr_dv_0 <= 1'b0;  sync_wr_dv_1 <= 1'b0;
         sync_rd_dv_0 <= 1'b0;  sync_rd_dv_1 <= 1'b0;
      end else begin
         sync_wr_dv_0 <= i_mem_write_DV;
         sync_wr_dv_1 <= sync_wr_dv_0;
         sync_rd_dv_0 <= i_mem_read_DV;
         sync_rd_dv_1 <= sync_rd_dv_0;
      end
   end

   wire synced_write_dv = sync_wr_dv_1;
   wire synced_read_dv  = sync_rd_dv_1;



   localparam IDLE = 4'd0;
   localparam WAIT = 4'd1;
   localparam WRITE = 4'd2;
   localparam WRITE_DONE = 4'd3;
   localparam READ = 4'd4;
   localparam READ_DONE = 4'd5;
   localparam WRITE_B1 = 4'd6;   // push the second 64-bit write beat (2:1 UI)
   reg [3:0] state = IDLE;

   reg rd_beat = 1'b0;           // 0 = awaiting beat0 (line[63:0]), 1 = beat1 (line[127:64])

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


   always @(posedge ui_clk) begin
      //always @ (posedge i_Clk) begin
      if (ui_clk_sync_rst) begin
         state <= IDLE;
         app_en <= 0;
         app_wdf_wren <= 0;
         app_wdf_end <= 0;
         rd_beat <= 1'b0;
      end else begin
         case (state)
            // IDLE: stay here until BOTH synced DVs are deasserted before
            // returning to WAIT. This is critical for CDC correctness:
            // synced_*_dv come from a 2-FF synchroniser, so they lag the CPU's
            // actual deassertion by ~2 ui_clk. Without this gate, after a
            // WRITE_DONE→IDLE→WAIT transition we would re-sample synced_write_dv
            // as still high (because the CPU has lowered i_mem_write_DV but the
            // 0 hasn't propagated yet) and immediately re-fire WRITE → another
            // o_mem_ready pulse. The cache's WRITE_FETCH/READ_WAIT then sees
            // that spurious ready as the read completion, latching stale
            // o_mem_read_data into the cache line and corrupting DRAM on
            // subsequent evictions. Holding IDLE until DV=0 forces the cache to
            // produce a real rising edge for the next transaction.
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


            READ: begin
               if (app_rdy) begin
                  app_en <= 1;
                  app_addr <= i_mem_addr[27:1]; // byte addr → MIG halfword addr
                  app_cmd <= CMD_READ;
                  rd_beat <= 1'b0;
                  state <= READ_DONE;
               end
            end

            // 2:1 read: the BL8 burst returns as TWO 64-bit app_rd_data beats.
            // First valid = line[63:0], second = line[127:64] — reassembled so
            // o_mem_read_data is byte-identical to the old single-128-bit-beat path.
            READ_DONE: begin
               if (app_rdy & app_en) begin
                  app_en <= 0;
               end

               if (app_rd_data_valid) begin
                  if (rd_beat == 1'b0) begin
                     o_mem_read_data[63:0]   <= app_rd_data;   // beat0 = line[63:0]
                     rd_beat <= 1'b1;
                  end else begin
                     o_mem_read_data[127:64] <= app_rd_data;   // beat1 = line[127:64]
                     o_mem_ready <= 1'b1;
                     state <= IDLE;
                  end
               end
            end


            default: state <= IDLE;
         endcase
      end
   end



endmodule

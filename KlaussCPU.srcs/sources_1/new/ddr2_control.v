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
    output reg o_mem_ready

);


   wire calib_done;

   reg [26:0] app_addr = 0;
   reg [2:0] app_cmd = 0;
   reg app_en;
   wire app_rdy;

   reg [127:0] app_wdf_data;
   wire app_wdf_end = 1;
   reg app_wdf_wren;
   wire app_wdf_rdy;

   wire [127:0] app_rd_data;
   reg [15:0] app_wdf_mask;  // Only first quarter
   wire app_rd_data_end;
   wire app_rd_data_valid;

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
   reg [3:0] state = IDLE;

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
      end else begin
         case (state)
            IDLE: begin
               o_mem_ready <= 1'b0;
               if (calib_done) begin
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

            WRITE: begin
               if (app_rdy & app_wdf_rdy) begin
                  state <= WRITE_DONE;
                  app_en <= 1;
                  app_wdf_wren <= 1;
                  app_addr <= i_mem_addr[27:1]; // byte addr → MIG halfword addr
                  app_cmd <= CMD_WRITE;
                  app_wdf_data <= i_mem_write_data;
                  app_wdf_mask <= i_app_wdf_mask;
               end
            end

            WRITE_DONE: begin
               if (app_rdy & app_en) begin
                  app_en <= 0;
               end

               if (app_wdf_rdy & app_wdf_wren) begin
                  app_wdf_wren <= 0;
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
                  state <= READ_DONE;
               end
            end

            READ_DONE: begin
               if (app_rdy & app_en) begin
                  app_en <= 0;
               end

               if (app_rd_data_valid) begin
                  o_mem_read_data <= app_rd_data;
                  o_mem_ready <= 1'b1;
                  state <= IDLE;

               end
            end


            default: state <= IDLE;
         endcase
      end
   end



endmodule

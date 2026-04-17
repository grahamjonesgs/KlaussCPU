`timescale 1ns / 1ps
module rams_sp_nc (

    input i_clk,

    // Program load and data update
    input [31:0] i_write_addr,
    input [15:0] i_write_value,
    input        i_write_en,

    // Opcode
    input      [31:0] i_opcode_read_addr,
    output reg [15:0] o_dout_opcode,
    output reg [31:0] o_dout_var1,
    output reg [31:0] o_dout_var2,

    // Data
    input      [31:0] i_mem_read_addr,
    output reg [31:0] o_dout_mem
);


   (* ram_style = "block" *) reg [15:0] RAM[32767:0];
   //(* ram_style = "block" *) reg [15:0] RAM [65535:0];

   initial begin
      $readmemh("lcd_data.mem", RAM);
   end

   always @(posedge i_clk) begin
      o_dout_opcode <= RAM[i_opcode_read_addr];
      o_dout_var1 <= {RAM[i_opcode_read_addr+1], RAM[i_opcode_read_addr+2]};
      o_dout_var2 <= {RAM[i_opcode_read_addr+3], RAM[i_opcode_read_addr+4]};

      o_dout_mem <= {RAM[i_mem_read_addr+1], RAM[i_mem_read_addr]};

      if (i_write_en) begin
         RAM[i_write_addr] <= i_write_value;
      end

   end
endmodule

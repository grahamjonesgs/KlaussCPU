// Single-Port Block RAM No-Change Mode
// File: rams_sp_nc.v
`timescale 1ns / 1ps
module stack (
    input clk,
    input i_reset,
    input i_read_flag,
    input i_write_flag,
    input [31:0] i_write_value,
    input i_stack_reset,
    output reg [31:0] o_stack_top_value,
    output reg o_stack_error
);

  (* ram_style = "block" *) reg [31:0] RAM[8192:0];
  reg [13:0] r_stack_pointer;
  integer i;

  initial begin
    r_stack_pointer = 14'h0;
    o_stack_error   = 1'b0;
  end

  always @(posedge clk) begin
    if (i_reset || i_stack_reset) begin
      r_stack_pointer <= 14'h0;
      o_stack_error   <= 1'b0;
    end //if (i_reset)
    else
    begin
      if (i_read_flag) begin
        if (r_stack_pointer == 0) begin
          o_stack_error = 1;
        end // if(r_stack_pointer==0)
            else
            begin
          r_stack_pointer <= r_stack_pointer - 1;
        end  // else if if(r_stack_pointer==0)
      end  // if (i_read==1)



      if (i_write_flag) begin
        if (r_stack_pointer > 8192) begin
          o_stack_error = 1;
        end // if (r_stack_pointer>8192)
            else
            begin
          RAM[r_stack_pointer] <= i_write_value;
          r_stack_pointer <= r_stack_pointer + 1;
        end  // else if (r_stack_pointer>8192)
      end  //if (i_write_flag==1)



    end  // else if (i_reset)
  end  // always clock

  always @(posedge clk) begin
    o_stack_top_value <= RAM[r_stack_pointer-1];
  end

endmodule

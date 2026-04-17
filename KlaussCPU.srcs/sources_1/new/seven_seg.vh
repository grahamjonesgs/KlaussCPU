// Set 7 Seg 1 LED value
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_7_seg1_value;
   input [31:0] i_byte;
   begin
      r_seven_seg_value1 <= {
         4'h0, i_byte[15:12], 4'h0, i_byte[11:8], 4'h0, i_byte[7:4], 4'h0, i_byte[3:0]
      };
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 8;
   end
endtask

// Set 7 Seg 2 LED  lower 16 bits
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_7_seg2_value;
   input [31:0] i_byte;
   begin
      r_seven_seg_value2 <= {
         4'h0, i_byte[15:12], 4'h0, i_byte[11:8], 4'h0, i_byte[7:4], 4'h0, i_byte[3:0]
      };
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 8;
   end
endtask

// Set 7 Seg 1 reg lower 16 bits
// On completion
// Increment PC
// Increment r_SM_msg

task t_7_seg1_reg;
   begin
      r_seven_seg_value1 <= {
         4'h0,
         r_reg_port_b[15:12],
         4'h0,
         r_reg_port_b[11:8],
         4'h0,
         r_reg_port_b[7:4],
         4'h0,
         r_reg_port_b[3:0]
      };
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 4;
   end
endtask

// Set 7 Seg 2 reg
// On completion
// Increment PC
// Increment r_SM_msg

task t_7_seg2_reg;
   begin
      r_seven_seg_value2 <= {
         4'h0,
         r_reg_port_b[15:12],
         4'h0,
         r_reg_port_b[11:8],
         4'h0,
         r_reg_port_b[7:4],
         4'h0,
         r_reg_port_b[3:0]
      };
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 4;
   end
endtask

// Set 7 Seg all reg
// On completion
// Increment PC
// Increment r_SM_msg
task t_7_seg_reg;
   begin
      r_seven_seg_value1 <= {
         4'h0,
         r_reg_port_b[31:28],
         4'h0,
         r_reg_port_b[27:24],
         4'h0,
         r_reg_port_b[23:20],
         4'h0,
         r_reg_port_b[19:16]
      };
      r_seven_seg_value2 <= {
         4'h0,
         r_reg_port_b[15:12],
         4'h0,
         r_reg_port_b[11:8],
         4'h0,
         r_reg_port_b[7:4],
         4'h0,
         r_reg_port_b[3:0]
      };
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 4;
   end
endtask

// Blank 7 Seg LED value
// On completion
// Increment PC
// Increment r_SM_msg

task t_7_seg_blank;
   begin
      r_seven_seg_value1 <= 32'h22222222;
      r_seven_seg_value2 <= 32'h22222222;
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 4;
   end
endtask


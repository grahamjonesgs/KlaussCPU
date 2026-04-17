//=============================================================================
// Extended ALU Tasks - Using Dedicated Read Ports
// Include this file in your main module
// 
// REQUIRES: r_reg_port_a and r_reg_port_b to be declared and updated each cycle:
//   reg [31:0] r_reg_port_a;
//   reg [31:0] r_reg_port_b;
//   always @(posedge i_Clk) begin
//       r_reg_port_a <= r_register[r_reg_1];
//       r_reg_port_b <= r_register[r_reg_2];
//   end
//=============================================================================

//=============================================================================
// ROTATE OPERATIONS
//=============================================================================

// ROLR - Rotate left by 1
task t_rotate_left;
   begin
      r_writeback_value <= {r_reg_port_b[62:0], r_reg_port_b[63]};
      r_writeback_reg <= r_reg_2;
      r_carry_flag <= r_reg_port_b[63];
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// RORR - Rotate right by 1
task t_rotate_right;
   begin
      r_writeback_value <= {r_reg_port_b[0], r_reg_port_b[63:1]};
      r_writeback_reg <= r_reg_2;
      r_carry_flag <= r_reg_port_b[0];
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// ROLCR - Rotate left through carry (65-bit rotate)
task t_rotate_left_carry;
   reg [64:0] temp;
   begin
      temp = {r_reg_port_b, r_carry_flag};
      r_writeback_value <= {temp[63:0]};
      r_writeback_reg <= r_reg_2;
      r_carry_flag <= temp[64];
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// RORCR - Rotate right through carry (65-bit rotate)
task t_rotate_right_carry;
   reg [64:0] temp;
   begin
      temp = {r_carry_flag, r_reg_port_b};
      r_writeback_value <= temp[64:1];
      r_writeback_reg <= r_reg_2;
      r_carry_flag <= temp[0];
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// ROLV - Rotate left by N bits
task t_rotate_left_n;
   input [31:0] i_count;
   reg [5:0] count;
   reg [63:0] result;
   begin
      count = i_count[5:0];  // Only use lower 6 bits (0-63)
      result = (r_reg_port_b << count) | (r_reg_port_b >> (64 - count));
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// RORV - Rotate right by N bits
task t_rotate_right_n;
   input [31:0] i_count;
   reg [5:0] count;
   reg [63:0] result;
   begin
      count = i_count[5:0];
      result = (r_reg_port_b >> count) | (r_reg_port_b << (64 - count));
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// ROLR - rd = rs1 rol rs2[5:0]
task t_rotate_left_reg;
   reg [5:0] count;
   reg [63:0] result;
   begin
      count = r_reg_port_b[5:0];
      result = (r_reg_port_a << count) | (r_reg_port_a >> (64 - count));
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_dst;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// RORR - rd = rs1 ror rs2[5:0]
task t_rotate_right_reg;
   reg [5:0] count;
   reg [63:0] result;
   begin
      count = r_reg_port_b[5:0];
      result = (r_reg_port_a >> count) | (r_reg_port_a << (64 - count));
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_dst;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask


//=============================================================================
// BIT MANIPULATION OPERATIONS
//=============================================================================

// BSET - Set bit N in register
task t_bit_set_value;
   input [31:0] i_bit;
   begin
      r_writeback_value <= r_reg_port_b | (64'b1 << i_bit[5:0]);
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// BCLR - Clear bit N in register
task t_bit_clear_value;
   input [31:0] i_bit;
   begin
      r_writeback_value <= r_reg_port_b & ~(64'b1 << i_bit[5:0]);
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// BTGL - Toggle bit N in register
task t_bit_toggle_value;
   input [31:0] i_bit;
   begin
      r_writeback_value <= r_reg_port_b ^ (64'b1 << i_bit[5:0]);
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// BTST - Test bit N, result in zero flag (zero if bit is 0)
task t_bit_test_value;
   input [31:0] i_bit;
   begin
      r_zero_flag <= ~r_reg_port_b[i_bit[5:0]];
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 8;
   end
endtask

// BSETRR - rd = rs1 with bit rs2 set
task t_bit_set_reg;
   begin
      r_writeback_value <= r_reg_port_a | (64'b1 << r_reg_port_b[5:0]);
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// BCLRRR - rd = rs1 with bit rs2 cleared
task t_bit_clear_reg;
   begin
      r_writeback_value <= r_reg_port_a & ~(64'b1 << r_reg_port_b[5:0]);
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// BTGLRR - rd = rs1 with bit rs2 toggled
task t_bit_toggle_reg;
   begin
      r_writeback_value <= r_reg_port_a ^ (64'b1 << r_reg_port_b[5:0]);
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// BTSTRR - rd = (rs1 >> rs2[5:0]) & 1
task t_bit_test_reg;
   reg [5:0] bit_pos;
   begin
      bit_pos = r_reg_port_b[5:0];
      r_writeback_value <= {63'b0, r_reg_port_a[bit_pos]};
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// POPCNT - Population count (count 1 bits)
task t_popcnt;
   begin
      r_writeback_value <= {57'b0, popcount(r_reg_port_b)};
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CLZ - Count leading zeros
task t_clz;
   begin
      r_writeback_value <= {57'b0, count_leading_zeros(r_reg_port_b)};
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CTZ - Count trailing zeros
task t_ctz;
   begin
      r_writeback_value <= {57'b0, count_trailing_zeros(r_reg_port_b)};
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// BITREV - Reverse all bits
task t_bit_reverse;
   begin
      r_writeback_value <= bit_reverse(r_reg_port_b);
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// BEXTR - Extract bit field
// Value format: bits [7:0] = start position, bits [15:8] = length
task t_extract_bits;
   input [31:0] i_params;
   reg [4:0] start_pos;
   reg [4:0] length;
   reg [31:0] mask;
   reg [31:0] result;
   begin
      start_pos = i_params[4:0];
      length = i_params[12:8];
      mask = (32'hFFFFFFFF >> (32 - length));
      result = (r_reg_port_b >> start_pos) & mask;
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// BDEP - Deposit bit field (insert bits at position)
// Uses r_reg_1 as source, r_reg_2 as destination
// Value format: bits [7:0] = start position, bits [15:8] = length
task t_deposit_bits;
   input [31:0] i_params;
   reg [4:0] start_pos;
   reg [4:0] length;
   reg [31:0] mask;
   reg [31:0] insert_val;
   begin
      start_pos = i_params[4:0];
      length = i_params[12:8];
      mask = (32'hFFFFFFFF >> (32 - length)) << start_pos;
      insert_val = (r_reg_port_a << start_pos) & mask;
      r_writeback_value <= (r_reg_port_b & ~mask) | insert_val;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask


//=============================================================================
// COMPARISON OPERATIONS - result written to rd (0 or 1)
// These benefit most from dedicated read ports - removes mux from compare path
//=============================================================================

// CMPEQR - rd = (rs1 == rs2) ? 1 : 0
task t_cmpeqr;
   begin
      r_writeback_value <= (r_reg_port_a == r_reg_port_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPNER - rd = (rs1 != rs2) ? 1 : 0
task t_cmpner;
   begin
      r_writeback_value <= (r_reg_port_a != r_reg_port_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPLTR - rd = (rs1 < rs2) ? 1 : 0, signed
task t_cmpltr;
   reg signed [63:0] s_a;
   reg signed [63:0] s_b;
   begin
      s_a = r_reg_port_a;
      s_b = r_reg_port_b;
      r_writeback_value <= (s_a < s_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPLER - rd = (rs1 <= rs2) ? 1 : 0, signed
task t_cmpler;
   reg signed [63:0] s_a;
   reg signed [63:0] s_b;
   begin
      s_a = r_reg_port_a;
      s_b = r_reg_port_b;
      r_writeback_value <= (s_a <= s_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPGTR - rd = (rs1 > rs2) ? 1 : 0, signed
task t_cmpgtr;
   reg signed [63:0] s_a;
   reg signed [63:0] s_b;
   begin
      s_a = r_reg_port_a;
      s_b = r_reg_port_b;
      r_writeback_value <= (s_a > s_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPGER - rd = (rs1 >= rs2) ? 1 : 0, signed
task t_cmpger;
   reg signed [63:0] s_a;
   reg signed [63:0] s_b;
   begin
      s_a = r_reg_port_a;
      s_b = r_reg_port_b;
      r_writeback_value <= (s_a >= s_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPULTR - rd = (rs1 < rs2) ? 1 : 0, unsigned
task t_cmpultr;
   begin
      r_writeback_value <= (r_reg_port_a < r_reg_port_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPULER - rd = (rs1 <= rs2) ? 1 : 0, unsigned
task t_cmpuler;
   begin
      r_writeback_value <= (r_reg_port_a <= r_reg_port_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPUGTR - rd = (rs1 > rs2) ? 1 : 0, unsigned
task t_cmpugtr;
   begin
      r_writeback_value <= (r_reg_port_a > r_reg_port_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPUGER - rd = (rs1 >= rs2) ? 1 : 0, unsigned
task t_cmpuger;
   begin
      r_writeback_value <= (r_reg_port_a >= r_reg_port_b) ? 32'b1 : 32'b0;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask


//=============================================================================
// HARDWARE MULTIPLY - PIPELINED (2 cycles)
// ALL multiply operations must go through the pipeline to avoid timing violations
//=============================================================================

// MULR - rd = rs1 * rs2, signed lo
task t_mul_regs_hw;
begin
    r_mul_operand_a   <= r_reg_port_a;
    r_mul_operand_b   <= r_reg_port_b;
    r_mul_dest_reg    <= r_reg_dst;
    r_mul_is_high     <= 1'b0;
    r_mul_is_unsigned <= 1'b0;
    r_mul_is_immediate <= 1'b0;
    r_SM              <= MULTIPLY_CALC;
end
endtask

// MULUR - rd = rs1 * rs2, unsigned lo
task t_mulu_regs_hw;
begin
    r_mul_operand_a   <= r_reg_port_a;
    r_mul_operand_b   <= r_reg_port_b;
    r_mul_dest_reg    <= r_reg_dst;
    r_mul_is_high     <= 1'b0;
    r_mul_is_unsigned <= 1'b1;
    r_mul_is_immediate <= 1'b0;
    r_SM              <= MULTIPLY_CALC;
end
endtask

// MULHR - rd = high(rs1 * rs2), signed
task t_mulh_regs_hw;
begin
    r_mul_operand_a   <= r_reg_port_a;
    r_mul_operand_b   <= r_reg_port_b;
    r_mul_dest_reg    <= r_reg_dst;
    r_mul_is_high     <= 1'b1;
    r_mul_is_unsigned <= 1'b0;
    r_mul_is_immediate <= 1'b0;
    r_SM              <= MULTIPLY_CALC;
end
endtask

// MULHUR - rd = high(rs1 * rs2), unsigned
task t_mulhu_regs_hw;
begin
    r_mul_operand_a   <= r_reg_port_a;
    r_mul_operand_b   <= r_reg_port_b;
    r_mul_dest_reg    <= r_reg_dst;
    r_mul_is_high     <= 1'b1;
    r_mul_is_unsigned <= 1'b1;
    r_mul_is_immediate <= 1'b0;
    r_SM              <= MULTIPLY_CALC;
end
endtask

// MULV - Multiply register by immediate value (signed) - NOW PIPELINED
task t_mul_value_hw;
   input [31:0] i_value;
begin
    r_mul_operand_a   <= r_reg_port_b;      // Register value
    r_mul_operand_b   <= i_value;           // Immediate value
    r_mul_dest_reg    <= r_reg_2;           // Result goes back to same register
    r_mul_is_high     <= 1'b0;
    r_mul_is_unsigned <= 1'b0;
    r_mul_is_immediate <= 1'b1;             // Flag for PC increment
    r_SM              <= MULTIPLY_CALC;
end
endtask

//=============================================================================
// DIVISION (Multi-cycle, but optimized)
// These use the division state machine defined in the main module
//=============================================================================

// DIVR - rd = rs1 / rs2, signed
task t_div_regs_hw;
   reg [63:0] abs_dividend;
   reg [63:0] abs_divisor;
   begin
      if (r_reg_port_b == 64'b0) begin
         // Divide by zero
         r_writeback_value <= 64'hFFFFFFFFFFFFFFFF;
         r_writeback_reg <= r_reg_dst;
         r_overflow_flag <= 1'b1;
         r_SM <= WRITEBACK;
         r_PC <= r_PC + 4;
      end
      else begin
         abs_dividend = r_reg_port_a[63] ? (~r_reg_port_a + 1) : r_reg_port_a;
         abs_divisor = r_reg_port_b[63] ? (~r_reg_port_b + 1) : r_reg_port_b;
         r_div_dividend <= abs_dividend;
         r_div_divisor <= abs_divisor;
         r_div_quotient <= 64'b0;
         r_div_remainder <= 64'b0;
         r_div_counter <= 7'd0;
         r_div_sign_q <= r_reg_port_a[63] ^ r_reg_port_b[63];
         r_div_sign_r <= r_reg_port_a[63];
         r_div_is_signed <= 1'b1;
         r_div_op <= DIV_OP_DIV;
         r_div_dest_reg <= r_reg_dst;
         r_div_pc_inc <= 1'b0;  // PC += 1
         r_SM <= DIVIDE_STEP;
      end
   end
endtask

// DIVUR - rd = rs1 / rs2, unsigned
task t_divu_regs_hw;
   begin
      if (r_reg_port_b == 64'b0) begin
         r_writeback_value <= 64'hFFFFFFFFFFFFFFFF;
         r_writeback_reg <= r_reg_dst;
         r_overflow_flag <= 1'b1;
         r_SM <= WRITEBACK;
         r_PC <= r_PC + 4;
      end
      else begin
         r_div_dividend <= r_reg_port_a;
         r_div_divisor <= r_reg_port_b;
         r_div_quotient <= 64'b0;
         r_div_remainder <= 64'b0;
         r_div_counter <= 7'd0;
         r_div_is_signed <= 1'b0;
         r_div_op <= DIV_OP_DIV;
         r_div_dest_reg <= r_reg_dst;
         r_div_pc_inc <= 1'b0;  // PC += 1
         r_SM <= DIVIDE_STEP;
      end
   end
endtask

// MODR - rd = rs1 % rs2, signed
task t_mod_regs_hw;
   reg [63:0] abs_dividend;
   reg [63:0] abs_divisor;
   begin
      if (r_reg_port_b == 64'b0) begin
         r_writeback_value <= r_reg_port_a;  // Return dividend
         r_writeback_reg <= r_reg_dst;
         r_overflow_flag <= 1'b1;
         r_SM <= WRITEBACK;
         r_PC <= r_PC + 4;
      end
      else begin
         abs_dividend = r_reg_port_a[63] ? (~r_reg_port_a + 1) : r_reg_port_a;
         abs_divisor = r_reg_port_b[63] ? (~r_reg_port_b + 1) : r_reg_port_b;
         r_div_dividend <= abs_dividend;
         r_div_divisor <= abs_divisor;
         r_div_quotient <= 64'b0;
         r_div_remainder <= 64'b0;
         r_div_counter <= 7'd0;
         r_div_sign_r <= r_reg_port_a[63];  // Remainder sign follows dividend
         r_div_is_signed <= 1'b1;
         r_div_op <= DIV_OP_MOD;
         r_div_dest_reg <= r_reg_dst;
         r_div_pc_inc <= 1'b0;  // PC += 1
         r_SM <= DIVIDE_STEP;
      end
   end
endtask

// MODUR - rd = rs1 % rs2, unsigned
task t_modu_regs_hw;
   begin
      if (r_reg_port_b == 64'b0) begin
         r_writeback_value <= r_reg_port_a;
         r_writeback_reg <= r_reg_dst;
         r_overflow_flag <= 1'b1;
         r_SM <= WRITEBACK;
         r_PC <= r_PC + 4;
      end
      else begin
         r_div_dividend <= r_reg_port_a;
         r_div_divisor <= r_reg_port_b;
         r_div_quotient <= 64'b0;
         r_div_remainder <= 64'b0;
         r_div_counter <= 7'd0;
         r_div_is_signed <= 1'b0;
         r_div_op <= DIV_OP_MOD;
         r_div_dest_reg <= r_reg_dst;
         r_div_pc_inc <= 1'b0;  // PC += 1
         r_SM <= DIVIDE_STEP;
      end
   end
endtask

// DIVV - Divide by immediate value (signed, initialization only)
task t_div_value_hw;
   input [31:0] i_value;
   reg [63:0] abs_dividend;
   reg [63:0] abs_divisor;
   begin
      if (i_value == 32'b0) begin
         r_writeback_value <= 64'hFFFFFFFFFFFFFFFF;
         r_writeback_reg <= r_reg_2;
         r_overflow_flag <= 1'b1;
         r_SM <= WRITEBACK;
         r_PC <= r_PC + 8;
      end
      else begin
         abs_dividend = r_reg_port_b[63] ? (~r_reg_port_b + 1) : r_reg_port_b;
         abs_divisor = i_value[31] ? (~{{32{i_value[31]}}, i_value} + 1) : {{32{1'b0}}, i_value};
         r_div_dividend <= abs_dividend;
         r_div_divisor <= abs_divisor;
         r_div_quotient <= 64'b0;
         r_div_remainder <= 64'b0;
         r_div_counter <= 7'd0;
         r_div_sign_q <= r_reg_port_b[63] ^ i_value[31];
         r_div_is_signed <= 1'b1;
         r_div_op <= DIV_OP_DIV;
         r_div_dest_reg <= r_reg_2;
         r_div_pc_inc <= 1'b1;  // PC += 2
         r_SM <= DIVIDE_STEP;
      end
   end
endtask

// MODV - Modulo by immediate value (signed, initialization only)
task t_mod_value_hw;
   input [31:0] i_value;
   reg [63:0] abs_dividend;
   reg [63:0] abs_divisor;
   begin
      if (i_value == 32'b0) begin
         // Return dividend on mod by zero
         r_overflow_flag <= 1'b1;
         r_SM <= OPCODE_REQUEST;
         r_PC <= r_PC + 8;
      end
      else begin
         abs_dividend = r_reg_port_b[63] ? (~r_reg_port_b + 1) : r_reg_port_b;
         abs_divisor = i_value[31] ? (~{{32{i_value[31]}}, i_value} + 1) : {{32{1'b0}}, i_value};
         r_div_dividend <= abs_dividend;
         r_div_divisor <= abs_divisor;
         r_div_quotient <= 64'b0;
         r_div_remainder <= 64'b0;
         r_div_counter <= 7'd0;
         r_div_sign_r <= r_reg_port_b[63];
         r_div_is_signed <= 1'b1;
         r_div_op <= DIV_OP_MOD;
         r_div_dest_reg <= r_reg_2;
         r_div_pc_inc <= 1'b1;  // PC += 2
         r_SM <= DIVIDE_STEP;
      end
   end
endtask


//=============================================================================
// ADDITIONAL REGISTER OPERATIONS
//=============================================================================

// ABSR - Absolute value
task t_abs_reg;
   begin
      if (r_reg_port_b[63]) begin
         r_writeback_value <= ~r_reg_port_b + 1;
         // Check for overflow (abs of -2^63)
         r_overflow_flag <= (r_reg_port_b == 64'h8000000000000000) ? 1'b1 : 1'b0;
      end else begin
         r_writeback_value <= r_reg_port_b;
      end
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// SEXTB - Sign extend byte to 64 bits
task t_sign_extend_byte;
   begin
      r_writeback_value <= {{56{r_reg_port_b[7]}}, r_reg_port_b[7:0]};
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= r_reg_port_b[7];
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// SEXTH - Sign extend halfword to 64 bits
task t_sign_extend_half;
   begin
      r_writeback_value <= {{48{r_reg_port_b[15]}}, r_reg_port_b[15:0]};
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= r_reg_port_b[15];
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// ZEXTB - Zero extend byte to 64 bits
task t_zero_extend_byte;
   begin
      r_writeback_value <= {56'b0, r_reg_port_b[7:0]};
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// ZEXTH - Zero extend halfword to 64 bits
task t_zero_extend_half;
   begin
      r_writeback_value <= {48'b0, r_reg_port_b[15:0]};
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// BSWAP - Byte swap (endian conversion, 8 bytes)
task t_byte_swap;
   begin
      r_writeback_value <= {
         r_reg_port_b[7:0],
         r_reg_port_b[15:8],
         r_reg_port_b[23:16],
         r_reg_port_b[31:24],
         r_reg_port_b[39:32],
         r_reg_port_b[47:40],
         r_reg_port_b[55:48],
         r_reg_port_b[63:56]
      };
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// SHLV - Shift left by N bits
task t_left_shift_n;
   input [31:0] i_count;
   reg [63:0] result;
   begin
      result = r_reg_port_b << i_count[5:0];
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// SHRV - Shift right by N bits (logical)
task t_right_shift_n;
   input [31:0] i_count;
   reg [63:0] result;
   begin
      result = r_reg_port_b >> i_count[5:0];
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// SHRAV - Shift right arithmetical by N bits
task t_right_shift_a_n;
   input [31:0] i_count;
   reg signed [63:0] signed_val;
   reg [63:0] result;
   begin
      signed_val = r_reg_port_b;
      result = signed_val >>> i_count[5:0];
      r_writeback_value <= result;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// MINR - rd = min(rs1, rs2), signed
task t_min_regs;
   reg signed [63:0] s_a;
   reg signed [63:0] s_b;
   begin
      s_a = r_reg_port_a;
      s_b = r_reg_port_b;
      r_writeback_value <= (s_a < s_b) ? r_reg_port_a : r_reg_port_b;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// MAXR - rd = max(rs1, rs2), signed
task t_max_regs;
   reg signed [63:0] s_a;
   reg signed [63:0] s_b;
   begin
      s_a = r_reg_port_a;
      s_b = r_reg_port_b;
      r_writeback_value <= (s_a > s_b) ? r_reg_port_a : r_reg_port_b;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// MINUR - rd = min(rs1, rs2), unsigned
task t_minu_regs;
   begin
      r_writeback_value <= (r_reg_port_a < r_reg_port_b) ?
                              r_reg_port_a : r_reg_port_b;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// MAXUR - rd = max(rs1, rs2), unsigned
task t_maxu_regs;
   begin
      r_writeback_value <= (r_reg_port_a > r_reg_port_b) ?
                              r_reg_port_a : r_reg_port_b;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

//=============================================================================
// 3-REGISTER SHIFT OPERATIONS (shift amount from register)
//=============================================================================

// SHLR - rd = rs1 << rs2[5:0], logical left shift
task t_shlr3;
   begin
      r_writeback_value <= r_reg_port_a << r_reg_port_b[5:0];
      r_writeback_reg <= r_reg_dst;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// SHRR - rd = rs1 >> rs2[5:0], logical right shift
task t_shrr3;
   begin
      r_writeback_value <= r_reg_port_a >> r_reg_port_b[5:0];
      r_writeback_reg <= r_reg_dst;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// SARR - rd = rs1 >>> rs2[5:0], arithmetic right shift
task t_sarr3;
   reg signed [63:0] signed_val;
   begin
      signed_val = r_reg_port_a;
      r_writeback_value <= signed_val >>> r_reg_port_b[5:0];
      r_writeback_reg <= r_reg_dst;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// JMPR - Jump to address in register
task t_jump_reg;
   begin
      r_PC <= r_reg_port_b[31:0];
      r_SM <= OPCODE_REQUEST;
   end
endtask


//=============================================================================
// INDEXED MEMORY ACCESS
//=============================================================================

// LDIDX - Load indexed: reg1 = mem[reg2 + immediate]
task t_load_indexed;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr = r_reg_port_b[31:0] + i_offset[31:0];
         r_mem_addr <= effective_addr;
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end
      else begin
         if (w_mem_ready) begin
            r_writeback_value <= w_mem_read_data;
            r_writeback_reg <= r_reg_1;
            r_SM <= WRITEBACK;
            r_mem_read_DV <= 1'b0;
            r_PC <= r_PC + 8;
         end
      end
   end
endtask

// STIDX - Store indexed: mem[reg2 + immediate] = reg1
task t_store_indexed;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr = r_reg_port_b[31:0] + i_offset[31:0];
         r_mem_addr <= effective_addr;
         r_mem_write_data <= r_reg_port_a;
         r_mem_write_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end
      else begin
         if (w_mem_ready) begin
            r_SM <= OPCODE_REQUEST;
            r_mem_write_DV <= 1'b0;
            r_PC <= r_PC + 8;
         end
      end
   end
endtask

// LDIDX64 - Load 64-bit doubleword indexed: reg1 = mem64[reg2 + immediate]
task t_load_indexed64;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr = r_reg_port_b[31:0] + i_offset[31:0];
         r_mem_addr    <= {effective_addr[31:3], 3'b000};  // 8-byte aligned
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end
      else begin
         if (w_mem_ready) begin
            r_writeback_value <= w_mem_read_data;
            r_writeback_reg   <= r_reg_1;
            r_SM              <= WRITEBACK;
            r_mem_read_DV     <= 1'b0;
            r_PC              <= r_PC + 8;
         end
      end
   end
endtask

// STIDX64 - Store 64-bit doubleword indexed: mem64[reg2 + immediate] = reg1
task t_store_indexed64;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr   = r_reg_port_b[31:0] + i_offset[31:0];
         r_mem_addr       <= {effective_addr[31:3], 3'b000};  // 8-byte aligned
         r_mem_write_data <= r_reg_port_a;
         r_mem_byte_en    <= 8'hFF;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end
      else begin
         if (w_mem_ready) begin
            r_SM           <= OPCODE_REQUEST;
            r_mem_write_DV <= 1'b0;
            r_PC           <= r_PC + 8;
         end
      end
   end
endtask

// LDIDXR - Load indexed with register offset: reg1 = mem[reg2 + reg3]
// Opcode format: 0EXY where X=dest, Y=base, var1[3:0] = offset register
// Uses 3-stage pipeline to read offset register through existing read port
// (avoids direct r_register[n] mux which causes timing violations)
task t_load_indexed_reg;
   input [31:0] i_var1;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         // Stage 0: Save base address, redirect port B to offset register
         r_idx_base_addr <= r_reg_port_b[31:0];
         r_reg_2 <= i_var1[3:0];
         r_extra_clock <= 2'd1;
      end
      else if (r_extra_clock == 1) begin
         // Stage 1: Wait for read port to update with offset register value
         r_extra_clock <= 2'd2;
      end
      else begin
         if (!r_mem_read_DV) begin
            // Stage 2a: Compute effective address and start memory read
            effective_addr = r_idx_base_addr + r_reg_port_b[31:0];
            r_mem_addr <= effective_addr;
            r_mem_read_DV <= 1'b1;
         end
         else if (w_mem_ready) begin
            // Stage 2b: Memory ready, capture result
            r_writeback_value <= w_mem_read_data;
            r_writeback_reg <= r_reg_1;
            r_SM <= WRITEBACK;
            r_mem_read_DV <= 1'b0;
            r_PC <= r_PC + 8;
         end
      end
   end
endtask

// STIDXR - Store indexed with register offset: mem[reg2 + reg3] = reg1
// Opcode format: 73XY where X=source, Y=base, var1[3:0] = offset register
// Uses 3-stage pipeline to read offset register through existing read port
task t_store_indexed_reg;
   input [31:0] i_var1;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         // Stage 0: Save base address and store data, redirect port B to offset register
         r_idx_base_addr <= r_reg_port_b[31:0];
         r_mem_write_data <= r_reg_port_a;
         r_reg_2 <= i_var1[3:0];
         r_extra_clock <= 2'd1;
      end
      else if (r_extra_clock == 1) begin
         // Stage 1: Wait for read port to update with offset register value
         r_extra_clock <= 2'd2;
      end
      else begin
         if (!r_mem_write_DV) begin
            // Stage 2a: Compute effective address and start memory write
            effective_addr = r_idx_base_addr + r_reg_port_b[31:0];
            r_mem_addr <= effective_addr;
            r_mem_write_DV <= 1'b1;
         end
         else if (w_mem_ready) begin
            // Stage 2b: Memory write complete
            r_SM <= OPCODE_REQUEST;
            r_mem_write_DV <= 1'b0;
            r_PC <= r_PC + 8;
         end
      end
   end
endtask

// ---------------------------------------------------------------------------
// Sub-word indexed load/store  (RRV, 2-word instructions, PC += 8)
// effective_addr = r_reg_port_b[31:0] + zero_ext(i_offset)
// Data register  = r_reg_port_a  (stores: source;  loads: dest = r_reg_1)
// Big-endian 64-bit bus — same byte-lane conventions as MEMGET/MEMSET tasks.
// r_reg_port_b remains stable across both cycles (no writeback until WRITEBACK
// state), so the lane selector can be recomputed in cycle 1 from the same inputs.
// ---------------------------------------------------------------------------

// LDIDX32 RRV — rd = zero_ext(mem32[(rs2 + imm32) & ~3])
task t_load_indexed32;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr = r_reg_port_b[31:0] + i_offset;
         r_mem_addr    <= {effective_addr[31:2], 2'b00};
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            effective_addr = r_reg_port_b[31:0] + i_offset;
            r_mem_read_DV     <= 1'b0;
            r_writeback_value <= effective_addr[2] ?
               {32'b0, w_mem_read_data[63:32]} :
               {32'b0, w_mem_read_data[31:0]};
            r_writeback_reg <= r_reg_1;
            r_SM            <= WRITEBACK;
            r_PC            <= r_PC + 8;
         end
      end
   end
endtask

// STIDX32 RRV — mem32[(rs2 + imm32) & ~3] = rs1[31:0]
task t_store_indexed32;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr   = r_reg_port_b[31:0] + i_offset;
         r_mem_addr       <= {effective_addr[31:2], 2'b00};
         r_mem_write_data <= {r_reg_port_a[31:0], r_reg_port_a[31:0]};
         r_mem_byte_en    <= effective_addr[2] ? 8'b1111_0000 : 8'b0000_1111;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_write_DV <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
            r_PC           <= r_PC + 8;
         end
      end
   end
endtask

// LDIDX16 RRV — rd = zero_ext(mem16[(rs2 + imm32) & ~1])
task t_load_indexed16;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr = r_reg_port_b[31:0] + i_offset;
         r_mem_addr    <= {effective_addr[31:1], 1'b0};
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            effective_addr = r_reg_port_b[31:0] + i_offset;
            r_mem_read_DV <= 1'b0;
            case (effective_addr[2:1])
               2'b00: r_writeback_value <= {48'b0, w_mem_read_data[63:48]};
               2'b01: r_writeback_value <= {48'b0, w_mem_read_data[47:32]};
               2'b10: r_writeback_value <= {48'b0, w_mem_read_data[31:16]};
               2'b11: r_writeback_value <= {48'b0, w_mem_read_data[15:0]};
            endcase
            r_writeback_reg <= r_reg_1;
            r_SM            <= WRITEBACK;
            r_PC            <= r_PC + 8;
         end
      end
   end
endtask

// STIDX16 RRV — mem16[(rs2 + imm32) & ~1] = rs1[15:0]
task t_store_indexed16;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   reg [2:0]  byte_lane;
   begin
      if (r_extra_clock == 0) begin
         effective_addr   = r_reg_port_b[31:0] + i_offset;
         byte_lane        = {effective_addr[2:1], 1'b0};
         r_mem_addr       <= {effective_addr[31:1], 1'b0};
         r_mem_write_data <= {4{r_reg_port_a[15:0]}};
         r_mem_byte_en    <= 8'b1100_0000 >> byte_lane;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_write_DV <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
            r_PC           <= r_PC + 8;
         end
      end
   end
endtask

// LDIDX8 RRV — rd = zero_ext(mem8[rs2 + imm32])
task t_load_indexed8;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   begin
      if (r_extra_clock == 0) begin
         effective_addr = r_reg_port_b[31:0] + i_offset;
         r_mem_addr    <= effective_addr;
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            effective_addr = r_reg_port_b[31:0] + i_offset;
            r_mem_read_DV <= 1'b0;
            case (effective_addr[2:0])
               3'b000: r_writeback_value <= {56'b0, w_mem_read_data[63:56]};
               3'b001: r_writeback_value <= {56'b0, w_mem_read_data[55:48]};
               3'b010: r_writeback_value <= {56'b0, w_mem_read_data[47:40]};
               3'b011: r_writeback_value <= {56'b0, w_mem_read_data[39:32]};
               3'b100: r_writeback_value <= {56'b0, w_mem_read_data[31:24]};
               3'b101: r_writeback_value <= {56'b0, w_mem_read_data[23:16]};
               3'b110: r_writeback_value <= {56'b0, w_mem_read_data[15:8]};
               3'b111: r_writeback_value <= {56'b0, w_mem_read_data[7:0]};
            endcase
            r_writeback_reg <= r_reg_1;
            r_SM            <= WRITEBACK;
            r_PC            <= r_PC + 8;
         end
      end
   end
endtask

// STIDX8 RRV — mem8[rs2 + imm32] = rs1[7:0]
task t_store_indexed8;
   input [31:0] i_offset;
   reg [31:0] effective_addr;
   reg [2:0]  byte_lane;
   begin
      if (r_extra_clock == 0) begin
         effective_addr   = r_reg_port_b[31:0] + i_offset;
         byte_lane        = effective_addr[2:0];
         r_mem_addr       <= effective_addr;
         r_mem_write_data <= {8{r_reg_port_a[7:0]}};
         r_mem_byte_en    <= 8'b1000_0000 >> byte_lane;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_write_DV <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
            r_PC           <= r_PC + 8;
         end
      end
   end
endtask
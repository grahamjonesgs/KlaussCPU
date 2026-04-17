/*// Set reg from memory location
// On completion
// Increment PC 3
// Increment r_SM_msg
task t_set_reg_from_memory;
input [31:0] i_location; // Not used here, but needed to show this is a two word opcode
    begin
        if(r_extra_clock==0)
        begin
           r_extra_clock<=1'b1;
        end
        else
        begin
            r_register[r_reg_2]<=w_mem; // the memory location, allows read of code as well as data
            r_SM<=OPCODE_REQUEST;
            r_PC<=r_PC+2;
        end
    end
endtask

// Set mem location from register
// On completion
// Increment PC 3
// Increment r_SM_msg
task t_set_memory_from_reg;
input [31:0] i_location; // Not used here, but needed to show this is a two word opcode
    begin
        if(r_extra_clock==0)
        begin
            r_extra_clock<=1'b1;
        end
        else
        begin

            //    if (w_dout_opcode_exec) // works as the memory read address is set to same as i_location already
            //    begin
            //         r_SM<=HCF_1; // Halt and catch fire error
            //        r_error_code<=ERR_SEG_WRITE_TO_CODE;
             //   end
             //   else
             //   begin
            o_ram_write_addr<=w_var1;
            o_ram_write_value<=r_register[r_reg_2];
            o_ram_write_DV<=1'b1;
            r_SM<=OPCODE_REQUEST;
            r_PC<=r_PC+2;
       // end
       end
    end
endtask
*/

// COPY - Copy second register into first (2-register format)
task t_copy_regs;
   begin
      r_writeback_value <= r_reg_port_b;
      r_writeback_reg <= r_reg_1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Set reg with value (sign-extends 32-bit immediate to 64 bits)
task t_set_reg;
   input [31:0] i_value;
   begin
      r_writeback_value <= {{32{i_value[31]}}, i_value};
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// Set reg with flags
task t_set_reg_flags;
   begin
      r_writeback_value <= {r_zero_flag, r_equal_flag, r_carry_flag, r_overflow_flag, 60'b0};
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Bitwise operations

// ANDR - rd = rs1 & rs2
task t_andr3;
   begin
      r_writeback_value <= r_reg_port_a & r_reg_port_b;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// ORR - rd = rs1 | rs2
task t_orr3;
   begin
      r_writeback_value <= r_reg_port_a | r_reg_port_b;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// XORR - rd = rs1 ^ rs2
task t_xorr3;
   begin
      r_writeback_value <= r_reg_port_a ^ r_reg_port_b;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// AND reg with value
task t_and_reg_value;
   input [31:0] i_value;
   begin
      r_writeback_value <= r_reg_port_b & i_value;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// OR reg with value
task t_or_reg_value;
   input [31:0] i_value;
   begin
      r_writeback_value <= r_reg_port_b | i_value;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// XOR reg with value
task t_xor_reg_value;
   input [31:0] i_value;
   begin
      r_writeback_value <= r_reg_port_b ^ i_value;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// Arithmetic operations

// Add value to reg
task t_add_value;
   input [31:0] i_value;
   reg [63:0] hold;
   begin
      {r_carry_flag, hold} = {1'b0, r_reg_port_b} + {1'b0, i_value};
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (r_reg_port_b[63]&&i_value[31]&&!hold[63])||(!r_reg_port_b[63]&&!i_value[31]&&hold[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// Subtract value from reg
task t_minus_value;
   input [31:0] i_value;
   reg [63:0] hold;
   begin
      {r_carry_flag, hold} = {1'b0, r_reg_port_b} - {1'b0, i_value};
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (r_reg_port_b[63]&&!i_value[31]&&!hold[63])||(!r_reg_port_b[63]&&i_value[31]&&hold[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 8;
   end
endtask

// Decrement reg
task t_dec_reg;
   reg [63:0] hold;
   begin
      {r_carry_flag, hold} = {1'b0, r_reg_port_b} - {65'b1};
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (r_reg_port_b[63] && !hold[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Increment reg
task t_inc_reg;
   reg [63:0] hold;
   begin
      {r_carry_flag, hold} = {1'b0, r_reg_port_b} + 65'b1;
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (!r_reg_port_b[63] && hold[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Compare register to value (no register write)
task t_compare_reg_value;
   input [31:0] i_value;
   reg signed [63:0] s_reg;
   reg signed [63:0] s_val;
   begin
      s_reg = r_reg_port_b;
      s_val = {{32{i_value[31]}}, i_value};  // sign-extend 32-bit value to 64-bit
      r_equal_flag <= (r_reg_port_b == s_val) ? 1'b1 : 1'b0;
      r_less_flag <= (s_reg < s_val) ? 1'b1 : 1'b0;
      r_ult_flag <= (r_reg_port_b < i_value) ? 1'b1 : 1'b0;
      r_sign_flag <= (s_reg - s_val) < 0 ? 1'b1 : 1'b0;
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 8;
   end
endtask

// SUBR - rd = rs1 - rs2
task t_subr3;
   reg [63:0] hold;
   begin
      {r_carry_flag, hold} = {1'b0, r_reg_port_a} - {1'b0, r_reg_port_b};
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (r_reg_port_a[63]&&!r_reg_port_b[63]&&!hold[63])||(!r_reg_port_a[63]&&r_reg_port_b[63]&&hold[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold;
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Negate reg
task t_negate_reg;
   begin
      r_writeback_value <= ~r_reg_port_b + 1;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Bitwise NOT reg
task t_not_reg;
   begin
      r_writeback_value <= ~r_reg_port_b;
      r_writeback_reg <= r_reg_2;
      r_writeback_set_zero_flag <= 1'b1;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Left shift reg
task t_left_shift_reg;
   begin
      r_writeback_value <= r_reg_port_b << 1;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Right shift reg
task t_right_shift_reg;
   begin
      r_writeback_value <= r_reg_port_b >> 1;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Left shift arithmetical reg
task t_left_shift_a_reg;
   begin
      r_writeback_value <= r_reg_port_b <<< 1;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// Right shift arithmetical reg
task t_right_shift_a_reg;
   reg signed [63:0] signed_val;
   begin
      signed_val = r_reg_port_b;
      r_writeback_value <= signed_val >>> 1;
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// CMPRR - set flags from first-second, no writeback (for use with conditional jumps)
task t_cmprr3;
   reg signed [63:0] s_a;
   reg signed [63:0] s_b;
   begin
      s_a = r_reg_port_a;
      s_b = r_reg_port_b;
      r_equal_flag <= (r_reg_port_a == r_reg_port_b) ? 1'b1 : 1'b0;
      r_less_flag <= (s_a < s_b) ? 1'b1 : 1'b0;
      r_ult_flag <= (r_reg_port_a < r_reg_port_b) ? 1'b1 : 1'b0;
      r_sign_flag <= (s_a - s_b) < 0 ? 1'b1 : 1'b0;
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 4;
   end
endtask

//=============================================================================
// ADDR - rd = rs1 + rs2
//=============================================================================
task t_addr3;
   reg [65:0] hold;
   begin
      hold = {1'b0, r_reg_port_a} + {1'b0, r_reg_port_b};
      r_carry_flag <= hold[64];
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (r_reg_port_a[63] == r_reg_port_b[63]) &&
                         (hold[63] != r_reg_port_a[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold[63:0];
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// ADDC RRR — rd = rs1 + rs2 + carry_flag  (add with carry)
// Overflow: same-sign inputs producing opposite-sign result.
task t_addc3;
   reg [65:0] hold;
   begin
      hold = {1'b0, r_reg_port_a} + {1'b0, r_reg_port_b} + {65'b0, r_carry_flag};
      r_carry_flag <= hold[64];
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (r_reg_port_a[63] == r_reg_port_b[63]) &&
                         (hold[63] != r_reg_port_a[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold[63:0];
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// SUBC RRR — rd = rs1 - rs2 - carry_flag  (subtract with borrow)
// carry_flag acts as borrow-in (x86 SBB convention).
// Overflow: operands have opposite signs and result sign differs from rs1.
task t_subc3;
   reg [65:0] hold;
   begin
      hold = {1'b0, r_reg_port_a} - {1'b0, r_reg_port_b} - {65'b0, r_carry_flag};
      r_carry_flag <= hold[64];
      r_writeback_set_zero_flag <= 1'b1;
      r_sign_flag <= hold[63];
      r_overflow_flag <= (r_reg_port_a[63] != r_reg_port_b[63]) &&
                         (hold[63] != r_reg_port_a[63]) ? 1'b1 : 1'b0;
      r_writeback_value <= hold[63:0];
      r_writeback_reg <= r_reg_dst;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// SETR64 - Load 64-bit immediate (3-word instruction: opcode + lo32 + hi32)
// i_lo = w_var1 (already fetched), hi32 must be fetched from PC+8
task t_set_reg64;
   input [31:0] i_lo;
   input [31:0] i_hi;  // not used — fetched via r_extra_clock
   begin
      if (r_extra_clock == 0) begin
         // Fetch hi32 from PC+8
         r_mem_addr    <= r_PC + 8;
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_read_DV     <= 1'b0;
            r_writeback_value <= {(r_PC[2] ? w_mem_read_data[63:32] : w_mem_read_data[31:0]), i_lo};
            r_writeback_reg   <= r_reg_2;
            r_SM              <= WRITEBACK;
            r_PC              <= r_PC + 12;  // opcode(4) + lo32(4) + hi32(4)
         end
      end
   end
endtask

// SEXTW - Sign-extend lower 32 bits to 64
task t_sextw;
   begin
      r_writeback_value <= {{32{r_reg_port_b[31]}}, r_reg_port_b[31:0]};
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask

// ZEXTW - Zero-extend lower 32 bits to 64
task t_zextw;
   begin
      r_writeback_value <= {32'b0, r_reg_port_b[31:0]};
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask


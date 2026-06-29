
// COPY - Copy second register into first (2-register format)
// rd = reg[7:4] (=r_reg_1), rs = reg[3:0] (=r_reg_2): rd <- rs.
// Operand order is REVERSED vs the 3-register ALU encoding (there rd is [11:8]).
// 1-word, PC += 4.
task t_copy_regs;
   begin
      r_wb.value <= r_reg_port_b;
      r_wb.rd <= r_reg_1;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// Set reg with value (sign-extends 32-bit immediate to 64 bits)
task t_set_reg;
   input [31:0] i_value;
   begin
      r_wb.value <= {{32{i_value[31]}}, i_value};
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 8;
   end
endtask

// LEAPC — rd = PC_of_this_instruction + sign_ext(imm32), zero-extended to 64 bits.
// r_PC is the opcode address at task time (RISC-V convention).  The 32-bit
// add wraps mod 2^32; result is zero-extended to 64 bits to match the
// CPU's address-in-register convention (e.g. CALL pushes {32'b0, ret_addr}).
// Used by the compiler for position-independent global / function pointer
// materialisation, replacing SETR for any non-MMIO address.
task t_lea_pc;
   input [31:0] i_value;
   begin
      r_wb.value <= {32'b0, r_PC + i_value};
      r_wb.rd   <= r_reg_2;
      r_SM              <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC              <= r_PC + 8;
   end
endtask

// SETFR - Set reg from condition flags.
// rd = {zero_flag, equal_flag, carry_flag, overflow_flag, 60'b0}
//   (flags occupy the top 4 bits, MSB = zero_flag; low 60 bits = 0).
// 1-word, PC += 4.
task t_set_reg_flags;
   begin
      r_wb.value <= {r_flags.zero, r_flags.equal, r_flags.carry, r_flags.overflow, 60'b0};
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// Bitwise operations

// ANDR - rd = rs1 & rs2
task t_andr3;
   begin
      r_wb.value <= r_reg_port_a & r_reg_port_b;
      r_wb.rd <= r_reg_dst;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// ORR - rd = rs1 | rs2
task t_orr3;
   begin
      r_wb.value <= r_reg_port_a | r_reg_port_b;
      r_wb.rd <= r_reg_dst;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// XORR - rd = rs1 ^ rs2
task t_xorr3;
   begin
      r_wb.value <= r_reg_port_a ^ r_reg_port_b;
      r_wb.rd <= r_reg_dst;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// AND reg with value
task t_and_reg_value;
   input [31:0] i_value;
   begin
      r_wb.value <= r_reg_port_b & i_value;
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 8;
   end
endtask

// OR reg with value
task t_or_reg_value;
   input [31:0] i_value;
   begin
      r_wb.value <= r_reg_port_b | i_value;
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 8;
   end
endtask

// XOR reg with value
task t_xor_reg_value;
   input [31:0] i_value;
   begin
      r_wb.value <= r_reg_port_b ^ i_value;
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 8;
   end
endtask

// Arithmetic operations

// Add value to reg
task t_add_value;
   input [31:0] i_value;
   logic [63:0] hold;
   logic        carry;
   begin
      {carry, hold} = {1'b0, r_reg_port_b} + {1'b0, i_value};
      r_alu_pipe_value    <= hold;
      r_alu_pipe_carry    <= carry;
      // ADDV zero-extends imm32, so the addend's 64-bit sign is always 0:
      // signed overflow only when a non-negative reg + non-negative addend goes negative.
      r_alu_pipe_overflow <= (!r_reg_port_b[63] && hold[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;     // ARITH
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_2;
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 8;
   end
endtask

// ADDI — Add sign-extended immediate to register, write to a (potentially
// different) destination register. RRV format, 2-word.
//   Encoding: opcode bits [7:4] = rd, [3:0] = rs;  imm32 at PC+4.
//   Operation:  rd = rs + sign_ext(imm32)
//   Sets zero/sign/carry/overflow.
//
// Distinct from ADDV (0x0000_081?) which is RV (single register, in-place)
// and zero-extends the immediate. ADDI exists primarily to materialise frame
// addresses in a single instruction (e.g. `addi rd, R15, frame_offset`)
// instead of the prior SETR + ADDR pair.
task t_addi;
   input [31:0] i_value;
   logic [63:0] sign_ext_imm;
   logic [63:0] hold;
   logic        carry;
   begin
      sign_ext_imm = {{32{i_value[31]}}, i_value};
      {carry, hold} = {1'b0, r_reg_port_b} + {1'b0, sign_ext_imm};
      r_alu_pipe_value    <= hold;
      r_alu_pipe_carry    <= carry;
      r_alu_pipe_overflow <= (r_reg_port_b[63] == sign_ext_imm[63]) &&
                             (hold[63] != r_reg_port_b[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_1;   // rd = bits[7:4]
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 8;
   end
endtask

// Subtract value from reg
task t_minus_value;
   input [31:0] i_value;
   logic [63:0] hold;
   logic        carry;
   begin
      {carry, hold} = {1'b0, r_reg_port_b} - {1'b0, i_value};
      r_alu_pipe_value    <= hold;
      r_alu_pipe_carry    <= carry;
      // MINUSV zero-extends imm32, so the subtrahend's 64-bit sign is always 0:
      // signed overflow only when a negative reg - non-negative subtrahend goes positive.
      r_alu_pipe_overflow <= (r_reg_port_b[63] && !hold[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_2;
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 8;
   end
endtask

// Decrement reg
task t_dec_reg;
   logic [63:0] hold;
   logic        carry;
   begin
      {carry, hold} = {1'b0, r_reg_port_b} - {65'b1};
      r_alu_pipe_value    <= hold;
      r_alu_pipe_carry    <= carry;
      r_alu_pipe_overflow <= (r_reg_port_b[63] && !hold[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_2;
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 4;
   end
endtask

// Increment reg
task t_inc_reg;
   logic [63:0] hold;
   logic        carry;
   begin
      {carry, hold} = {1'b0, r_reg_port_b} + 65'b1;
      r_alu_pipe_value    <= hold;
      r_alu_pipe_carry    <= carry;
      r_alu_pipe_overflow <= (!r_reg_port_b[63] && hold[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_2;
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 4;
   end
endtask

// Compare register to value (no register write)
task t_compare_reg_value;
   input [31:0] i_value;
   logic signed [63:0] s_reg;
   logic signed [63:0] s_val;
   logic [63:0] u_val;
   logic [63:0] diff;
   begin
      s_reg = r_reg_port_b;
      s_val = {{32{i_value[31]}}, i_value};  // sign-extend 32-bit value to 64-bit
      u_val = {{32{i_value[31]}}, i_value};  // same sign-extended bits, unsigned-typed for ult
      diff  = r_reg_port_b - s_val;
      r_alu_pipe_value <= diff;       // sign flag = diff[63] in ALU_FINISH
      r_alu_pipe_equal <= (r_reg_port_b == s_val) ? 1'b1 : 1'b0;
      r_alu_pipe_less  <= (s_reg < s_val) ? 1'b1 : 1'b0;
      r_alu_pipe_ult   <= (r_reg_port_b < u_val) ? 1'b1 : 1'b0;  // sign-extend then unsigned-compare (matches CMPRR)
      r_alu_pipe_mode  <= 1'b1;        // CMP
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 8;
   end
endtask

// SUBR - rd = rs1 - rs2
task t_subr3;
   logic [63:0] hold;
   logic        carry;
   begin
      {carry, hold} = {1'b0, r_reg_port_a} - {1'b0, r_reg_port_b};
      r_alu_pipe_value    <= hold;
      r_alu_pipe_carry    <= carry;
      r_alu_pipe_overflow <= (r_reg_port_a[63]&&!r_reg_port_b[63]&&!hold[63])||(!r_reg_port_a[63]&&r_reg_port_b[63]&&hold[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_dst;
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 4;
   end
endtask

// Negate reg
task t_negate_reg;
   begin
      r_wb.value <= ~r_reg_port_b + 1;
      r_wb.rd <= r_reg_2;
      r_wb.set_zero <= 1'b1;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// Bitwise NOT reg
task t_not_reg;
   begin
      r_wb.value <= ~r_reg_port_b;
      r_wb.rd <= r_reg_2;
      r_wb.set_zero <= 1'b1;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// Left shift reg
task t_left_shift_reg;
   begin
      r_wb.value <= r_reg_port_b << 1;
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// Right shift reg
task t_right_shift_reg;
   begin
      r_wb.value <= r_reg_port_b >> 1;
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// Left shift arithmetical reg
task t_left_shift_a_reg;
   begin
      r_wb.value <= r_reg_port_b <<< 1;
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// Right shift arithmetical reg
task t_right_shift_a_reg;
   logic signed [63:0] signed_val;
   begin
      signed_val = r_reg_port_b;
      r_wb.value <= signed_val >>> 1;
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// CMPRR - set flags from first-second, no writeback (for use with conditional jumps)
task t_cmprr3;
   logic signed [63:0] s_a;
   logic signed [63:0] s_b;
   logic [63:0] diff;
   begin
      s_a  = r_reg_port_a;
      s_b  = r_reg_port_b;
      diff = r_reg_port_a - r_reg_port_b;
      r_alu_pipe_value <= diff;       // sign flag = diff[63] in ALU_FINISH
      r_alu_pipe_equal <= (r_reg_port_a == r_reg_port_b) ? 1'b1 : 1'b0;
      r_alu_pipe_less  <= (s_a < s_b) ? 1'b1 : 1'b0;
      r_alu_pipe_ult   <= (r_reg_port_a < r_reg_port_b) ? 1'b1 : 1'b0;
      r_alu_pipe_mode  <= 1'b1;        // CMP
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 4;
   end
endtask

//=============================================================================
// ADDR - rd = rs1 + rs2
//=============================================================================
task t_addr3;
   logic [65:0] hold;
   begin
      hold = {1'b0, r_reg_port_a} + {1'b0, r_reg_port_b};
      r_alu_pipe_value    <= hold[63:0];
      r_alu_pipe_carry    <= hold[64];
      r_alu_pipe_overflow <= (r_reg_port_a[63] == r_reg_port_b[63]) &&
                             (hold[63] != r_reg_port_a[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_dst;
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 4;
   end
endtask

// ADDC RRR — rd = rs1 + rs2 + carry_flag  (add with carry)
// Overflow: same-sign inputs producing opposite-sign result.
task t_addc3;
   logic [65:0] hold;
   begin
      hold = {1'b0, r_reg_port_a} + {1'b0, r_reg_port_b} + {65'b0, r_flags.carry};
      r_alu_pipe_value    <= hold[63:0];
      r_alu_pipe_carry    <= hold[64];
      r_alu_pipe_overflow <= (r_reg_port_a[63] == r_reg_port_b[63]) &&
                             (hold[63] != r_reg_port_a[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_dst;
      r_SM <= ALU_FINISH;
      r_PC <= r_PC + 4;
   end
endtask

// SUBC RRR — rd = rs1 - rs2 - carry_flag  (subtract with borrow)
// carry_flag acts as borrow-in (x86 SBB convention).
// Overflow: operands have opposite signs and result sign differs from rs1.
task t_subc3;
   logic [65:0] hold;
   begin
      hold = {1'b0, r_reg_port_a} - {1'b0, r_reg_port_b} - {65'b0, r_flags.carry};
      r_alu_pipe_value    <= hold[63:0];
      r_alu_pipe_carry    <= hold[64];
      r_alu_pipe_overflow <= (r_reg_port_a[63] != r_reg_port_b[63]) &&
                             (hold[63] != r_reg_port_a[63]) ? 1'b1 : 1'b0;
      r_alu_pipe_mode     <= 1'b0;
      r_wb.set_zero <= 1'b1;
      r_wb.rd <= r_reg_dst;
      r_SM <= ALU_FINISH;
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
            r_wb.value <= {(r_PC[2] ? w_mem_read_data[63:32] : w_mem_read_data[31:0]), i_lo};
            r_wb.rd   <= r_reg_2;
            r_SM              <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
            r_PC              <= r_PC + 12;  // opcode(4) + lo32(4) + hi32(4)
         end
      end
   end
endtask

// SEXTW - Sign-extend lower 32 bits to 64
task t_sextw;
   begin
      r_wb.value <= {{32{r_reg_port_b[31]}}, r_reg_port_b[31:0]};
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask

// ZEXTW - Zero-extend lower 32 bits to 64
task t_zextw;
   begin
      r_wb.value <= {32'b0, r_reg_port_b[31:0]};
      r_wb.rd <= r_reg_2;
      r_SM <= OPCODE_REQUEST; r_wb.pending <= 1'b1;
      r_PC <= r_PC + 4;
   end
endtask


// Stack tasks — stack lives in DDR2 RAM, top of 128 MiB, growing downward.
// Convention: st.SP is a byte address pointing to the last pushed item (full descending).
//   PUSH: st.SP -= 8; DDR2[st.SP] = value (64-bit)
//   POP:  value = DDR2[st.SP]; st.SP += 8
// All operations use the multi-cycle DDR2 interface (st.extra_clock pattern).
// R15 is the frame pointer by software convention.

// Push register onto stack (64-bit)
// 1-word instruction (PC+4)
task t_stack_push_reg;
   begin
      if (st.extra_clock == 0) begin
         st.SP             <= st.SP - 8;
         st.mem_addr       <= st.SP - 32'd8;
         st.mem_write_data <= r_reg_port_b;
         st.mem_byte_en    <= 8'hFF;
         st.mem_write_DV   <= 1'b1;
         st.extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            st.mem_write_DV <= 1'b0;
            st.SM           <= OPCODE_REQUEST;
            st.PC           <= st.PC + 4;
         end
      end
   end
endtask

// Push 32-bit immediate value onto stack (zero-extended to 64-bit)
// 2-word instruction (PC+8)
task t_stack_push_value;
   input [31:0] i_value;
   begin
      if (st.extra_clock == 0) begin
         st.SP             <= st.SP - 8;
         st.mem_addr       <= st.SP - 32'd8;
         st.mem_write_data <= {32'b0, i_value};
         st.mem_byte_en    <= 8'hFF;
         st.mem_write_DV   <= 1'b1;
         st.extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            st.mem_write_DV <= 1'b0;
            st.SM           <= OPCODE_REQUEST;
            st.PC           <= st.PC + 8;
         end
      end
   end
endtask

// Pop stack into register (64-bit)
// 1-word instruction (PC+4)
task t_stack_pop_reg;
   begin
      if (st.extra_clock == 0) begin
         st.mem_addr    <= st.SP;
         st.mem_read_DV <= 1'b1;
         st.extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            st.wb.value <= w_mem_read_data;
            st.wb.rd   <= st.reg_2;
            st.SP              <= st.SP + 8;
            st.mem_read_DV     <= 1'b0;
            st.SM              <= WRITEBACK;
            st.PC              <= st.PC + 4;
         end
      end
   end
endtask

// GETSP — copy stack pointer value into register
// 1-word instruction (PC+4)
task t_get_sp;
   begin
      st.wb.value <= {32'b0, st.SP};
      st.wb.rd   <= st.reg_2;
      st.SM              <= WRITEBACK;
      st.PC              <= st.PC + 4;
   end
endtask

// SETSP — set stack pointer from register
// 1-word instruction (PC+4)
task t_set_sp;
   begin
      st.SP <= r_reg_port_b[31:0];
      st.SM <= OPCODE_REQUEST;
      st.PC <= st.PC + 4;
   end
endtask

// ADDSP — add signed 32-bit immediate to stack pointer (allocate/free locals)
// 2-word instruction (PC+8)
task t_add_sp;
   input [31:0] i_value;
   begin
      st.SP <= st.SP + $signed(i_value);
      st.SM <= OPCODE_REQUEST;
      st.PC <= st.PC + 8;
   end
endtask

// CALLR — call to address held in register (for function pointers)
// Pushes PC+4 as return address, jumps to register value.
// 1-word instruction (PC+4)
task t_call_reg;
   begin
      if (st.extra_clock == 0) begin
         st.SP             <= st.SP - 8;
         st.mem_addr       <= st.SP - 32'd8;
         st.mem_write_data <= {32'b0, st.PC + 32'd4};  // return after 1-word instruction
         st.mem_byte_en    <= 8'hFF;
         st.mem_write_DV   <= 1'b1;
         st.extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            st.mem_write_DV <= 1'b0;
            st.SM           <= OPCODE_REQUEST;
            st.PC           <= r_reg_port_b[31:0];
         end
      end
   end
endtask

// PUSHV64 — push 64-bit immediate value (3-word instruction: opcode + lo32 + hi32)
// hi32 is self-fetched via st.extra_clock pattern
task t_stack_push_value64;
   input [31:0] i_lo;
   input [31:0] i_hi;  // not used — fetched via st.extra_clock
   begin
      if (st.extra_clock == 0) begin
         // Fetch hi32 from PC+8
         st.mem_addr    <= st.PC + 8;
         st.mem_read_DV <= 1'b1;
         st.extra_clock <= 1'b1;
      end else if (st.extra_clock == 1) begin
         if (w_mem_ready) begin
            // Got hi32, now write to stack
            st.mem_read_DV    <= 1'b0;
            st.SP             <= st.SP - 8;
            st.mem_addr       <= st.SP - 32'd8;
            st.mem_write_data <= {(st.PC[2] ? w_mem_read_data[63:32] : w_mem_read_data[31:0]), i_lo};
            st.mem_byte_en    <= 8'hFF;
            st.mem_write_DV   <= 1'b1;
            st.extra_clock    <= 2'd2;
         end
      end else begin
         if (w_mem_ready) begin
            st.mem_write_DV <= 1'b0;
            st.SM           <= OPCODE_REQUEST;
            st.PC           <= st.PC + 12;
         end
      end
   end
endtask

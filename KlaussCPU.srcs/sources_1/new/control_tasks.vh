// Jump if condition met
// On completion
// Increment PC 2 or jump
// Increment st.SM
task t_cond_jump;
   input [31:0] i_value;
   input i_condition;
   begin
      // Perf: record taken/not-taken for conditional jumps only. JMP (0x1000)
      // also routes here with i_condition=1; exclude it so the perf BRANCH_TAKEN
      // denominator matches the classifier's PC_BRANCH (conditional jumps only).
      r_perf_br_valid <= (w_opcode != 32'h0000_1000);
      r_perf_br_taken <= i_condition;
      if (i_condition) begin
         st.SM <= OPCODE_REQUEST;
         st.PC <= i_value[31:0];  // jump (byte address)
      end // if(i_condition)
        else
        begin
         st.SM <= OPCODE_REQUEST;
         st.PC <= st.PC + 8;
         // Not-taken fall-through == r_FPC. The prefetch (above the FSM case)
         // latches it this cycle; pre-settle st.reg_1/2 from the same buffer data
         // so the successor takes the fast-path dispatch. A single-cycle branch
         // can't use the latch's own reg fields (not registered until next cycle),
         // so read the instruction buffer directly. r_ir_pc==st.PC is the safety net.
         if (!r_ir_valid && w_ifb_fpc_hit) begin
            st.reg_1         <= r_FPC[2] ? w_ifb_fpc_data[39:36] : w_ifb_fpc_data[7:4];
            st.reg_2         <= r_FPC[2] ? w_ifb_fpc_data[35:32] : w_ifb_fpc_data[3:0];
            r_ir_presettled <= 1'b1;
         end
      end  // else if(i_condition)
   end
endtask

// Call if condition met — push return address (PC+8) onto DDR2 stack, jump to target.
// 2-word instruction; uses multi-cycle DDR2 write (st.extra_clock pattern).
// w_var1 (the jump target) stays valid across cycles as the instruction stays latched.
task t_cond_call;
   input [31:0] i_value;
   input i_condition;
   begin
      if (i_condition) begin
         if (st.extra_clock == 0) begin
            st.SP             <= st.SP - 8;
            st.mem_addr       <= st.SP - 32'd8;
            st.mem_write_data <= {32'b0, st.PC + 32'd8};  // return after 2-word instruction
            st.mem_byte_en    <= 8'hFF;
            st.mem_write_DV   <= 1'b1;
            st.extra_clock    <= 1'b1;
         end else begin
            if (w_mem_ready) begin
               st.mem_write_DV <= 1'b0;
               st.SM           <= OPCODE_REQUEST;
               st.PC           <= i_value[31:0];
            end
         end
      end else begin
         st.SM <= OPCODE_REQUEST;
         st.PC <= st.PC + 8;
         if (!r_ir_valid && w_ifb_fpc_hit) begin   // not-taken fall-through pre-settle (see t_cond_jump)
            st.reg_1         <= r_FPC[2] ? w_ifb_fpc_data[39:36] : w_ifb_fpc_data[7:4];
            st.reg_2         <= r_FPC[2] ? w_ifb_fpc_data[35:32] : w_ifb_fpc_data[3:0];
            r_ir_presettled <= 1'b1;
         end
      end
   end
endtask

// Return from call — pop return address from DDR2 stack, jump to it.
// 1-word instruction; uses multi-cycle DDR2 read (st.extra_clock pattern).
task t_ret;
   begin
      if (st.extra_clock == 0) begin
         st.mem_addr    <= st.SP;
         st.mem_read_DV <= 1'b1;
         st.extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            st.PC          <= w_mem_read_data[31:0];
            st.SP          <= st.SP + 8;
            st.mem_read_DV <= 1'b0;
            st.SM          <= OPCODE_REQUEST;
         end
      end
   end
endtask

// PC-relative conditional jump.  Target = PC_of_this_instruction + sign_ext(imm32).
// st.PC at task-execution time is the opcode address (RISC-V convention),
// so no pre-adjustment is needed.  imm32 is treated as signed 32-bit;
// 32-bit add wraps mod 2^32 (the high bits of st.PC are always 0 in physical memory).
task t_cond_jump_rel;
   input [31:0] i_value;
   input i_condition;
   begin
      // Perf: as t_cond_jump, but JMPREL (0x1030) is the unconditional form here.
      r_perf_br_valid <= (w_opcode != 32'h0000_1030);
      r_perf_br_taken <= i_condition;
      if (i_condition) begin
         st.SM <= OPCODE_REQUEST;
         st.PC <= st.PC + i_value;
      end else begin
         st.SM <= OPCODE_REQUEST;
         st.PC <= st.PC + 8;
         if (!r_ir_valid && w_ifb_fpc_hit) begin   // not-taken fall-through pre-settle (see t_cond_jump)
            st.reg_1         <= r_FPC[2] ? w_ifb_fpc_data[39:36] : w_ifb_fpc_data[7:4];
            st.reg_2         <= r_FPC[2] ? w_ifb_fpc_data[35:32] : w_ifb_fpc_data[3:0];
            r_ir_presettled <= 1'b1;
         end
      end
   end
endtask

// PC-relative conditional call.  Push return PC (PC+8) onto the DDR2 stack,
// then jump to PC_of_this_instruction + sign_ext(imm32).  Same multi-cycle
// DDR2 write pattern as t_cond_call.
task t_cond_call_rel;
   input [31:0] i_value;
   input i_condition;
   begin
      if (i_condition) begin
         if (st.extra_clock == 0) begin
            st.SP             <= st.SP - 8;
            st.mem_addr       <= st.SP - 32'd8;
            st.mem_write_data <= {32'b0, st.PC + 32'd8};  // return after 2-word instruction
            st.mem_byte_en    <= 8'hFF;
            st.mem_write_DV   <= 1'b1;
            st.extra_clock    <= 1'b1;
         end else begin
            if (w_mem_ready) begin
               st.mem_write_DV <= 1'b0;
               st.SM           <= OPCODE_REQUEST;
               st.PC           <= st.PC + i_value;
            end
         end
      end else begin
         st.SM <= OPCODE_REQUEST;
         st.PC <= st.PC + 8;
         if (!r_ir_valid && w_ifb_fpc_hit) begin   // not-taken fall-through pre-settle (see t_cond_jump)
            st.reg_1         <= r_FPC[2] ? w_ifb_fpc_data[39:36] : w_ifb_fpc_data[7:4];
            st.reg_2         <= r_FPC[2] ? w_ifb_fpc_data[35:32] : w_ifb_fpc_data[3:0];
            r_ir_presettled <= 1'b1;
         end
      end
   end
endtask

// Do nothing
// On completion
// Increment PC
// Increment st.SM
task t_nop;
   begin
      st.SM <= OPCODE_REQUEST;
      st.PC <= st.PC + 4;
   end
endtask

// Stop and hang - enters low-power halted state until reset
// On completion
// Do not change PC
task t_halt;
   begin
      st.SM <= HALTED_BREAK;
   end
endtask

// WAIT — interruptible core-suspend. Advance PC past the WAIT (so the resume
// point is the next instruction), then park in WAITING until an unmasked,
// vectored interrupt becomes pending (w_irq_ready). The WAITING state wakes
// via the normal OPCODE_REQUEST dispatch, which saves this PC and jumps to the
// handler; IRET resumes after the WAIT. No register/flag side effects; no UART
// break (unlike HALT). If an interrupt is already pending, WAITING exits next
// cycle — it never sleeps with work pending.
task t_wait;
   begin
      st.PC <= st.PC + 4;
      st.SM <= WAITING;
   end
endtask

// Reset PC
// On completion
// Do not change PC
// Increment st.SM
task t_reset;
   begin
      st.SM <= OPCODE_REQUEST;
      st.PC <= 32'h4;  // byte address of word 1 (first instruction word after header)
   end  // Case FFFF
endtask

// IRET — return from interrupt handler.
// Pops the 64-bit context slot saved by interrupt dispatch:
//   [31:0]  → PC        (resume address of interrupted instruction)
//   [38:32] → flags     (zero, equal, carry, overflow, sign, less, ult)
//   [42:39] → st.int_mask (per-source enables; restored so masked-on-entry
//                         source becomes active again outside the handler)
// Uses the same multi-cycle DDR2 read pattern as t_ret.
task t_iret;
   begin
      if (st.extra_clock == 0) begin
         st.mem_addr    <= st.SP;
         st.mem_read_DV <= 1'b1;
         st.extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            st.PC            <= w_mem_read_data[31:0];
            st.flags.zero     <= w_mem_read_data[38];
            st.flags.equal    <= w_mem_read_data[37];
            st.flags.carry    <= w_mem_read_data[36];
            st.flags.overflow <= w_mem_read_data[35];
            st.flags.sign     <= w_mem_read_data[34];
            st.flags.less     <= w_mem_read_data[33];
            st.flags.ult      <= w_mem_read_data[32];
            st.int_mask      <= w_mem_read_data[42:39];
            st.SP            <= st.SP + 8;
            st.mem_read_DV   <= 1'b0;
            st.SM            <= OPCODE_REQUEST;
         end
      end
   end
endtask

// TRAP — software trap.  Halts with ERR_TRAP, distinct from HALT (normal stop)
// and ERR_INV_OPCODE (illegal instruction).  Maps to ISD::TRAP in the LLVM backend.
// Routes via HCF_1, so it runs the full HCF crash-dump path (f_dump_* in
// uart_tasks.vh) — unlike HALT, which simply parks in HALTED_BREAK with no dump.
task t_trap;
   begin
      st.error_code <= ERR_TRAP;
      st.SM         <= HCF_1;
   end
endtask

// Interrupt control state (handler vector table, per-source mask, timer period)
// is configured via MMIO at base 0xF00F_0000. See MMIO_MAP.md.




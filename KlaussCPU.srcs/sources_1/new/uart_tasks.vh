// uart_tasks.vh — UART opcode tasks RETIRED (UART is MMIO at 0xF001_0000).
// Only the crash-dump formatter (f_dump_length / f_dump_byte) and the
// debug-break message helper (t_debug_message) remain; both drive the
// shared message transmitter over the same physical UART.


// t_debug_message — build "PC is <hex>\r" into r_msg and pulse r_msg_send_DV.
// Helper only: does NOT set st.SM or advance st.PC (caller drives those).
// Prints PC[26:0] as 7 hex chars (PC[26:24] shown as a single nibble).
task t_debug_message;
    begin
        if (!w_sending_msg) begin
            r_msg[7:0] <= 8'h50;  // P
            r_msg[15:8] <= 8'h43;  // C
            r_msg[23:16] <= 8'h20;  //  
            r_msg[31:24] <= 8'h69;  // i
            r_msg[39:32] <= 8'h73;  // s
            r_msg[47:40] <= 8'h20;  //  

            r_msg[55:48] <= return_ascii_from_hex({1'b0, st.PC[26:24]});
            r_msg[63:56] <= return_ascii_from_hex(st.PC[23:20]);
            r_msg[71:64] <= return_ascii_from_hex(st.PC[19:16]);
            r_msg[79:72] <= return_ascii_from_hex(st.PC[15:12]);
            r_msg[87:80] <= return_ascii_from_hex(st.PC[11:8]);
            r_msg[95:88] <= return_ascii_from_hex(st.PC[7:4]);
            r_msg[103:96] <= return_ascii_from_hex(st.PC[3:0]);

            r_msg[111:104] <= 8'h0D;  // CR

            r_msg_length <= 8'h0E;
            r_msg_send_DV <= 1'b1;
        end
    end

endtask  // Send test message



// HCF crash-dump byte/length functions.
//
// Replaces the legacy single-cycle t_hcf_dump_build_line task that assigned
// all 32 bytes of r_msg in one branch of a wide case statement.  That case
// collapsed in synthesis to a 207-input × 256-bit multiplexer — the largest
// routing-congestion driver in the design (post-place congestion 8x8 E/W,
// route_design wallclock ~35 min).
//
// The byte/length functions compile to 8-bit-wide lookups instead.  The HCF
// FSM (sub-state BYTE_BUILD in KlaussCPU.v) walks pos = 0..length-1 and writes
// one byte of r_msg per cycle, so the per-byte write-data is an 8-bit mux,
// not a 256-bit one (~32x smaller).
//
// Cost: adds N cycles of latency per dump line (13..27 cycles depending on
// phase), trivially hidden under the UART transmission time (~100 µs/line at
// 1 Mbaud).
//
// Phases (same as before):
//   0           "*** CRASH DUMP ***"        (banner)
//   1           "ERR=xx PC=xxxxxxxx"
//   2           "OPC=xxxxxxxx SP=xxxxxxxx"
//   3           "V1=xxxxxxxx IDX=xxxxxxxx"
//   4           "V1H=xxxxxxxx"              (hi32 of V64 immediate; DRAM read at PC+8)
//   5           "OPCM=xxxxxxxx"             (DRAM-side re-read at PC)
//   6           "SM=xxxxxxxxx"              (FSM state, 34-bit one-hot)
//   7           "IV0=xxxxxxxx"              (timer ISR vector, r_interrupt_table[0])
//   8           "FLG Z=x E=x C=x V=x"
//   9           "    S=x L=x U=x"
//   10          "INSTR=NNNNNNNN"
//   11..26      "RX=NNNNNNNNNNNNNNNN"       16 register dumps (R0..RF)
//   27..30      "SX=NNNNNNNNNNNNNNNN"       4 top-of-stack doublewords (S0..S3)
//   31..46      "TX P=xxxxxxxx OP=xxxxxxxx" 16 trace entries (T0..TF, newest-first)
//   47          "*** END ***"

// Byte length of the dump line for the given phase.
function [7:0] f_dump_length;
    input [6:0] phase;
    begin
        case (phase)
            DUMP_HEADER:  f_dump_length = 8'd22;
            DUMP_ERR_PC:  f_dump_length = 8'd20;
            DUMP_OPC_SP:  f_dump_length = 8'd26;
            DUMP_V1_V2:   f_dump_length = 8'd26;
            DUMP_V1H:     f_dump_length = 8'd14;
            DUMP_OPCM:    f_dump_length = 8'd15;
            DUMP_SM:      f_dump_length = 8'd14;
            DUMP_IV0:     f_dump_length = 8'd14;
            DUMP_FLAGS_A: f_dump_length = 8'd21;
            DUMP_FLAGS_B: f_dump_length = 8'd17;
            DUMP_INSTR:   f_dump_length = 8'd16;
            DUMP_FOOTER:  f_dump_length = 8'd13;
            default: begin
                // Family phases: regs (R/S) = 21B, trace (T) = 27B
                if (phase < DUMP_TRACE_BASE) f_dump_length = 8'd21;
                else                         f_dump_length = 8'd27;
            end
        endcase
    end
endfunction



// Returns the byte at position `pos` for the given dump phase.
// The HCF FSM only walks pos = 0..f_dump_length(phase)-1; out-of-range
// positions return 0x00 and are never sent.
function [7:0] f_dump_byte;
    input [6:0] phase;
    input [4:0] pos;
    logic   [3:0]  k;          // family index (which reg / stack / trace entry)
    logic   [63:0] data;       // 64-bit data word for family lines
    logic   [31:0] half;       // 32-bit V1H/OPCM half-word
    begin
        f_dump_byte = 8'h00;
        case (phase)
            DUMP_HEADER: begin
                // "\r\n*** CRASH DUMP ***\r\n"
                case (pos)
                    5'd0:  f_dump_byte = 8'h0D;
                    5'd1:  f_dump_byte = 8'h0A;
                    5'd2:  f_dump_byte = "*";
                    5'd3:  f_dump_byte = "*";
                    5'd4:  f_dump_byte = "*";
                    5'd5:  f_dump_byte = " ";
                    5'd6:  f_dump_byte = "C";
                    5'd7:  f_dump_byte = "R";
                    5'd8:  f_dump_byte = "A";
                    5'd9:  f_dump_byte = "S";
                    5'd10: f_dump_byte = "H";
                    5'd11: f_dump_byte = " ";
                    5'd12: f_dump_byte = "D";
                    5'd13: f_dump_byte = "U";
                    5'd14: f_dump_byte = "M";
                    5'd15: f_dump_byte = "P";
                    5'd16: f_dump_byte = " ";
                    5'd17: f_dump_byte = "*";
                    5'd18: f_dump_byte = "*";
                    5'd19: f_dump_byte = "*";
                    5'd20: f_dump_byte = 8'h0D;
                    5'd21: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_ERR_PC: begin
                // "ERR=xx PC=xxxxxxxx\r\n"
                case (pos)
                    5'd0:  f_dump_byte = "E";
                    5'd1:  f_dump_byte = "R";
                    5'd2:  f_dump_byte = "R";
                    5'd3:  f_dump_byte = "=";
                    5'd4:  f_dump_byte = return_ascii_from_hex(st.error_code[7:4]);
                    5'd5:  f_dump_byte = return_ascii_from_hex(st.error_code[3:0]);
                    5'd6:  f_dump_byte = " ";
                    5'd7:  f_dump_byte = "P";
                    5'd8:  f_dump_byte = "C";
                    5'd9:  f_dump_byte = "=";
                    5'd10: f_dump_byte = return_ascii_from_hex(st.PC[31:28]);
                    5'd11: f_dump_byte = return_ascii_from_hex(st.PC[27:24]);
                    5'd12: f_dump_byte = return_ascii_from_hex(st.PC[23:20]);
                    5'd13: f_dump_byte = return_ascii_from_hex(st.PC[19:16]);
                    5'd14: f_dump_byte = return_ascii_from_hex(st.PC[15:12]);
                    5'd15: f_dump_byte = return_ascii_from_hex(st.PC[11: 8]);
                    5'd16: f_dump_byte = return_ascii_from_hex(st.PC[ 7: 4]);
                    5'd17: f_dump_byte = return_ascii_from_hex(st.PC[ 3: 0]);
                    5'd18: f_dump_byte = 8'h0D;
                    5'd19: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_OPC_SP: begin
                // "OPC=xxxxxxxx SP=xxxxxxxx\r\n"
                case (pos)
                    5'd0:  f_dump_byte = "O";
                    5'd1:  f_dump_byte = "P";
                    5'd2:  f_dump_byte = "C";
                    5'd3:  f_dump_byte = "=";
                    5'd4:  f_dump_byte = return_ascii_from_hex(w_opcode[31:28]);
                    5'd5:  f_dump_byte = return_ascii_from_hex(w_opcode[27:24]);
                    5'd6:  f_dump_byte = return_ascii_from_hex(w_opcode[23:20]);
                    5'd7:  f_dump_byte = return_ascii_from_hex(w_opcode[19:16]);
                    5'd8:  f_dump_byte = return_ascii_from_hex(w_opcode[15:12]);
                    5'd9:  f_dump_byte = return_ascii_from_hex(w_opcode[11: 8]);
                    5'd10: f_dump_byte = return_ascii_from_hex(w_opcode[ 7: 4]);
                    5'd11: f_dump_byte = return_ascii_from_hex(w_opcode[ 3: 0]);
                    5'd12: f_dump_byte = " ";
                    5'd13: f_dump_byte = "S";
                    5'd14: f_dump_byte = "P";
                    5'd15: f_dump_byte = "=";
                    5'd16: f_dump_byte = return_ascii_from_hex(st.SP[31:28]);
                    5'd17: f_dump_byte = return_ascii_from_hex(st.SP[27:24]);
                    5'd18: f_dump_byte = return_ascii_from_hex(st.SP[23:20]);
                    5'd19: f_dump_byte = return_ascii_from_hex(st.SP[19:16]);
                    5'd20: f_dump_byte = return_ascii_from_hex(st.SP[15:12]);
                    5'd21: f_dump_byte = return_ascii_from_hex(st.SP[11: 8]);
                    5'd22: f_dump_byte = return_ascii_from_hex(st.SP[ 7: 4]);
                    5'd23: f_dump_byte = return_ascii_from_hex(st.SP[ 3: 0]);
                    5'd24: f_dump_byte = 8'h0D;
                    5'd25: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_V1_V2: begin
                // "V1=xxxxxxxx IDX=xxxxxxxx\r\n"
                case (pos)
                    5'd0:  f_dump_byte = "V";
                    5'd1:  f_dump_byte = "1";
                    5'd2:  f_dump_byte = "=";
                    5'd3:  f_dump_byte = return_ascii_from_hex(w_var1[31:28]);
                    5'd4:  f_dump_byte = return_ascii_from_hex(w_var1[27:24]);
                    5'd5:  f_dump_byte = return_ascii_from_hex(w_var1[23:20]);
                    5'd6:  f_dump_byte = return_ascii_from_hex(w_var1[19:16]);
                    5'd7:  f_dump_byte = return_ascii_from_hex(w_var1[15:12]);
                    5'd8:  f_dump_byte = return_ascii_from_hex(w_var1[11: 8]);
                    5'd9:  f_dump_byte = return_ascii_from_hex(w_var1[ 7: 4]);
                    5'd10: f_dump_byte = return_ascii_from_hex(w_var1[ 3: 0]);
                    5'd11: f_dump_byte = " ";
                    5'd12: f_dump_byte = "I";
                    5'd13: f_dump_byte = "D";
                    5'd14: f_dump_byte = "X";
                    5'd15: f_dump_byte = "=";
                    5'd16: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[31:28]);
                    5'd17: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[27:24]);
                    5'd18: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[23:20]);
                    5'd19: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[19:16]);
                    5'd20: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[15:12]);
                    5'd21: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[11: 8]);
                    5'd22: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[ 7: 4]);
                    5'd23: f_dump_byte = return_ascii_from_hex(st.idx_base_addr[ 3: 0]);
                    5'd24: f_dump_byte = 8'h0D;
                    5'd25: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_V1H: begin
                // "V1H=xxxxxxxx\r\n"  (hi32 picked by st.PC[2])
                half = st.PC[2] ? r_hcf_stack_data[63:32] : r_hcf_stack_data[31:0];
                case (pos)
                    5'd0:  f_dump_byte = "V";
                    5'd1:  f_dump_byte = "1";
                    5'd2:  f_dump_byte = "H";
                    5'd3:  f_dump_byte = "=";
                    5'd4:  f_dump_byte = return_ascii_from_hex(half[31:28]);
                    5'd5:  f_dump_byte = return_ascii_from_hex(half[27:24]);
                    5'd6:  f_dump_byte = return_ascii_from_hex(half[23:20]);
                    5'd7:  f_dump_byte = return_ascii_from_hex(half[19:16]);
                    5'd8:  f_dump_byte = return_ascii_from_hex(half[15:12]);
                    5'd9:  f_dump_byte = return_ascii_from_hex(half[11: 8]);
                    5'd10: f_dump_byte = return_ascii_from_hex(half[ 7: 4]);
                    5'd11: f_dump_byte = return_ascii_from_hex(half[ 3: 0]);
                    5'd12: f_dump_byte = 8'h0D;
                    5'd13: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_OPCM: begin
                // "OPCM=xxxxxxxx\r\n"
                half = st.PC[2] ? r_hcf_stack_data[63:32] : r_hcf_stack_data[31:0];
                case (pos)
                    5'd0:  f_dump_byte = "O";
                    5'd1:  f_dump_byte = "P";
                    5'd2:  f_dump_byte = "C";
                    5'd3:  f_dump_byte = "M";
                    5'd4:  f_dump_byte = "=";
                    5'd5:  f_dump_byte = return_ascii_from_hex(half[31:28]);
                    5'd6:  f_dump_byte = return_ascii_from_hex(half[27:24]);
                    5'd7:  f_dump_byte = return_ascii_from_hex(half[23:20]);
                    5'd8:  f_dump_byte = return_ascii_from_hex(half[19:16]);
                    5'd9:  f_dump_byte = return_ascii_from_hex(half[15:12]);
                    5'd10: f_dump_byte = return_ascii_from_hex(half[11: 8]);
                    5'd11: f_dump_byte = return_ascii_from_hex(half[ 7: 4]);
                    5'd12: f_dump_byte = return_ascii_from_hex(half[ 3: 0]);
                    5'd13: f_dump_byte = 8'h0D;
                    5'd14: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_SM: begin
                // "SM=xxxxxxxxx\r\n"  (34-bit one-hot FSM)
                case (pos)
                    5'd0:  f_dump_byte = "S";
                    5'd1:  f_dump_byte = "M";
                    5'd2:  f_dump_byte = "=";
                    5'd3:  f_dump_byte = return_ascii_from_hex({2'b0, r_fault_sm[33:32]});
                    5'd4:  f_dump_byte = return_ascii_from_hex(r_fault_sm[31:28]);
                    5'd5:  f_dump_byte = return_ascii_from_hex(r_fault_sm[27:24]);
                    5'd6:  f_dump_byte = return_ascii_from_hex(r_fault_sm[23:20]);
                    5'd7:  f_dump_byte = return_ascii_from_hex(r_fault_sm[19:16]);
                    5'd8:  f_dump_byte = return_ascii_from_hex(r_fault_sm[15:12]);
                    5'd9:  f_dump_byte = return_ascii_from_hex(r_fault_sm[11: 8]);
                    5'd10: f_dump_byte = return_ascii_from_hex(r_fault_sm[ 7: 4]);
                    5'd11: f_dump_byte = return_ascii_from_hex(r_fault_sm[ 3: 0]);
                    5'd12: f_dump_byte = 8'h0D;
                    5'd13: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_IV0: begin
                // "IV0=xxxxxxxx\r\n"
                case (pos)
                    5'd0:  f_dump_byte = "I";
                    5'd1:  f_dump_byte = "V";
                    5'd2:  f_dump_byte = "0";
                    5'd3:  f_dump_byte = "=";
                    5'd4:  f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][31:28]);
                    5'd5:  f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][27:24]);
                    5'd6:  f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][23:20]);
                    5'd7:  f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][19:16]);
                    5'd8:  f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][15:12]);
                    5'd9:  f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][11: 8]);
                    5'd10: f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][ 7: 4]);
                    5'd11: f_dump_byte = return_ascii_from_hex(r_interrupt_table[0][ 3: 0]);
                    5'd12: f_dump_byte = 8'h0D;
                    5'd13: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_FLAGS_A: begin
                // "FLG Z=x E=x C=x V=x\r\n"
                case (pos)
                    5'd0:  f_dump_byte = "F";
                    5'd1:  f_dump_byte = "L";
                    5'd2:  f_dump_byte = "G";
                    5'd3:  f_dump_byte = " ";
                    5'd4:  f_dump_byte = "Z";
                    5'd5:  f_dump_byte = "=";
                    5'd6:  f_dump_byte = st.flags.zero     ? "1" : "0";
                    5'd7:  f_dump_byte = " ";
                    5'd8:  f_dump_byte = "E";
                    5'd9:  f_dump_byte = "=";
                    5'd10: f_dump_byte = st.flags.equal    ? "1" : "0";
                    5'd11: f_dump_byte = " ";
                    5'd12: f_dump_byte = "C";
                    5'd13: f_dump_byte = "=";
                    5'd14: f_dump_byte = st.flags.carry    ? "1" : "0";
                    5'd15: f_dump_byte = " ";
                    5'd16: f_dump_byte = "V";
                    5'd17: f_dump_byte = "=";
                    5'd18: f_dump_byte = st.flags.overflow ? "1" : "0";
                    5'd19: f_dump_byte = 8'h0D;
                    5'd20: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_FLAGS_B: begin
                // "    S=x L=x U=x\r\n"
                case (pos)
                    5'd0:  f_dump_byte = " ";
                    5'd1:  f_dump_byte = " ";
                    5'd2:  f_dump_byte = " ";
                    5'd3:  f_dump_byte = " ";
                    5'd4:  f_dump_byte = "S";
                    5'd5:  f_dump_byte = "=";
                    5'd6:  f_dump_byte = st.flags.sign ? "1" : "0";
                    5'd7:  f_dump_byte = " ";
                    5'd8:  f_dump_byte = "L";
                    5'd9:  f_dump_byte = "=";
                    5'd10: f_dump_byte = st.flags.less ? "1" : "0";
                    5'd11: f_dump_byte = " ";
                    5'd12: f_dump_byte = "U";
                    5'd13: f_dump_byte = "=";
                    5'd14: f_dump_byte = st.flags.ult  ? "1" : "0";
                    5'd15: f_dump_byte = 8'h0D;
                    5'd16: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_INSTR: begin
                // "INSTR=NNNNNNNN\r\n"
                case (pos)
                    5'd0:  f_dump_byte = "I";
                    5'd1:  f_dump_byte = "N";
                    5'd2:  f_dump_byte = "S";
                    5'd3:  f_dump_byte = "T";
                    5'd4:  f_dump_byte = "R";
                    5'd5:  f_dump_byte = "=";
                    5'd6:  f_dump_byte = return_ascii_from_hex(r_instr_count[31:28]);
                    5'd7:  f_dump_byte = return_ascii_from_hex(r_instr_count[27:24]);
                    5'd8:  f_dump_byte = return_ascii_from_hex(r_instr_count[23:20]);
                    5'd9:  f_dump_byte = return_ascii_from_hex(r_instr_count[19:16]);
                    5'd10: f_dump_byte = return_ascii_from_hex(r_instr_count[15:12]);
                    5'd11: f_dump_byte = return_ascii_from_hex(r_instr_count[11: 8]);
                    5'd12: f_dump_byte = return_ascii_from_hex(r_instr_count[ 7: 4]);
                    5'd13: f_dump_byte = return_ascii_from_hex(r_instr_count[ 3: 0]);
                    5'd14: f_dump_byte = 8'h0D;
                    5'd15: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_FOOTER: begin
                // "*** END ***\r\n"
                case (pos)
                    5'd0:  f_dump_byte = "*";
                    5'd1:  f_dump_byte = "*";
                    5'd2:  f_dump_byte = "*";
                    5'd3:  f_dump_byte = " ";
                    5'd4:  f_dump_byte = "E";
                    5'd5:  f_dump_byte = "N";
                    5'd6:  f_dump_byte = "D";
                    5'd7:  f_dump_byte = " ";
                    5'd8:  f_dump_byte = "*";
                    5'd9:  f_dump_byte = "*";
                    5'd10: f_dump_byte = "*";
                    5'd11: f_dump_byte = 8'h0D;
                    5'd12: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            default: begin
                // Family phases (regs / stack / trace).  Resolve k and data
                // first so the per-position case below stays simple.
                if (phase < DUMP_STACK_BASE) begin
                    // Register dump R0..RF.  data is the register value
                    // pre-fetched into r_hcf_stack_data during PREP
                    // (KlaussCPU.v) — keeping the live register file off
                    // f_dump_byte's combinational cone removes the 16:1 64-bit
                    // mux that was the crash-dump routing-congestion source.
                    // k is still derived from phase only, for the "RX=" label.
                    k    = phase[3:0] - DUMP_REG_BASE[3:0];
                    data = r_hcf_stack_data;
                end else if (phase < DUMP_TRACE_BASE) begin
                    // Stack dump S0..S3 (data pre-fetched into r_hcf_stack_data)
                    k    = phase[3:0] - DUMP_STACK_BASE[3:0];
                    data = r_hcf_stack_data;
                end else begin
                    // Trace dump T0..TF.  The r_trace_buf BRAM read is lifted
                    // into a PREP-time pre-fetch (KlaussCPU.v) — the latched
                    // entry lives in r_hcf_stack_data by the time BYTE_BUILD
                    // runs.  This keeps the BRAM read off f_dump_byte's
                    // combinational path so the trace byte mux closes timing.
                    k    = phase[3:0] - DUMP_TRACE_BASE[3:0];
                    data = r_hcf_stack_data;
                end

                if (phase < DUMP_TRACE_BASE) begin
                    // Reg/stack: "?X=NNNNNNNNNNNNNNNN\r\n"   (21 bytes)
                    case (pos)
                        5'd0:  f_dump_byte = (phase < DUMP_STACK_BASE) ? "R" : "S";
                        5'd1:  f_dump_byte = return_ascii_from_hex(k);
                        5'd2:  f_dump_byte = "=";
                        5'd3:  f_dump_byte = return_ascii_from_hex(data[63:60]);
                        5'd4:  f_dump_byte = return_ascii_from_hex(data[59:56]);
                        5'd5:  f_dump_byte = return_ascii_from_hex(data[55:52]);
                        5'd6:  f_dump_byte = return_ascii_from_hex(data[51:48]);
                        5'd7:  f_dump_byte = return_ascii_from_hex(data[47:44]);
                        5'd8:  f_dump_byte = return_ascii_from_hex(data[43:40]);
                        5'd9:  f_dump_byte = return_ascii_from_hex(data[39:36]);
                        5'd10: f_dump_byte = return_ascii_from_hex(data[35:32]);
                        5'd11: f_dump_byte = return_ascii_from_hex(data[31:28]);
                        5'd12: f_dump_byte = return_ascii_from_hex(data[27:24]);
                        5'd13: f_dump_byte = return_ascii_from_hex(data[23:20]);
                        5'd14: f_dump_byte = return_ascii_from_hex(data[19:16]);
                        5'd15: f_dump_byte = return_ascii_from_hex(data[15:12]);
                        5'd16: f_dump_byte = return_ascii_from_hex(data[11: 8]);
                        5'd17: f_dump_byte = return_ascii_from_hex(data[ 7: 4]);
                        5'd18: f_dump_byte = return_ascii_from_hex(data[ 3: 0]);
                        5'd19: f_dump_byte = 8'h0D;
                        5'd20: f_dump_byte = 8'h0A;
                        default: f_dump_byte = 8'h00;
                    endcase
                end else begin
                    // Trace: "TX P=xxxxxxxx OP=xxxxxxxx\r\n"  (27 bytes)
                    case (pos)
                        5'd0:  f_dump_byte = "T";
                        5'd1:  f_dump_byte = return_ascii_from_hex(k);
                        5'd2:  f_dump_byte = " ";
                        5'd3:  f_dump_byte = "P";
                        5'd4:  f_dump_byte = "=";
                        5'd5:  f_dump_byte = return_ascii_from_hex(data[63:60]);
                        5'd6:  f_dump_byte = return_ascii_from_hex(data[59:56]);
                        5'd7:  f_dump_byte = return_ascii_from_hex(data[55:52]);
                        5'd8:  f_dump_byte = return_ascii_from_hex(data[51:48]);
                        5'd9:  f_dump_byte = return_ascii_from_hex(data[47:44]);
                        5'd10: f_dump_byte = return_ascii_from_hex(data[43:40]);
                        5'd11: f_dump_byte = return_ascii_from_hex(data[39:36]);
                        5'd12: f_dump_byte = return_ascii_from_hex(data[35:32]);
                        5'd13: f_dump_byte = " ";
                        5'd14: f_dump_byte = "O";
                        5'd15: f_dump_byte = "P";
                        5'd16: f_dump_byte = "=";
                        5'd17: f_dump_byte = return_ascii_from_hex(data[31:28]);
                        5'd18: f_dump_byte = return_ascii_from_hex(data[27:24]);
                        5'd19: f_dump_byte = return_ascii_from_hex(data[23:20]);
                        5'd20: f_dump_byte = return_ascii_from_hex(data[19:16]);
                        5'd21: f_dump_byte = return_ascii_from_hex(data[15:12]);
                        5'd22: f_dump_byte = return_ascii_from_hex(data[11: 8]);
                        5'd23: f_dump_byte = return_ascii_from_hex(data[ 7: 4]);
                        5'd24: f_dump_byte = return_ascii_from_hex(data[ 3: 0]);
                        5'd25: f_dump_byte = 8'h0D;
                        5'd26: f_dump_byte = 8'h0A;
                        default: f_dump_byte = 8'h00;
                    endcase
                end
            end
        endcase
    end
endfunction



// t_tx_message — load one of the canned UART strings into r_msg/r_msg_length
// and pulse r_msg_send_DV, selected by i_message_number:
//   1 = "Load Complete OK"   2 = "Load Error, bad CRC"
//   3 = "Test message"       4 = "Segfault: exec data" (ERR_SEG_EXEC_DATA; unused)
//   default = empty message (length 0)
// Helper only: does NOT set st.SM or advance st.PC (caller drives those).
task t_tx_message;
    input [7:0] i_message_number;
    begin
        case (i_message_number)
            1: // Load Complete OK
            begin
                r_msg[7:0] <= 8'h4C;
                r_msg[15:8] <= 8'h6F;
                r_msg[23:16] <= 8'h61;
                r_msg[31:24] <= 8'h64;
                r_msg[39:32] <= 8'h20;
                r_msg[47:40] <= 8'h43;
                r_msg[55:48] <= 8'h6F;
                r_msg[63:56] <= 8'h6D;
                r_msg[71:64] <= 8'h70;
                r_msg[79:72] <= 8'h6C;
                r_msg[87:80] <= 8'h65;
                r_msg[95:88] <= 8'h74;
                r_msg[103:96] <= 8'h65;
                r_msg[111:104] <= 8'h20;
                r_msg[119:112] <= 8'h4F;
                r_msg[127:120] <= 8'h4B;
                r_msg[135:128] <= 8'h0A;
                r_msg[143:136] <= 8'h0D;
                r_msg_length <= 18;
            end
            2: // Load Error, bad CRC
            begin
                r_msg[7:0] <= 8'h4C;
                r_msg[15:8] <= 8'h6F;
                r_msg[23:16] <= 8'h61;
                r_msg[31:24] <= 8'h64;
                r_msg[39:32] <= 8'h20;
                r_msg[47:40] <= 8'h45;
                r_msg[55:48] <= 8'h72;
                r_msg[63:56] <= 8'h72;
                r_msg[71:64] <= 8'h6F;
                r_msg[79:72] <= 8'h72;
                r_msg[87:80] <= 8'h2C;
                r_msg[95:88] <= 8'h20;
                r_msg[103:96] <= 8'h62;
                r_msg[111:104] <= 8'h61;
                r_msg[119:112] <= 8'h64;
                r_msg[127:120] <= 8'h20;
                r_msg[135:128] <= 8'h43;
                r_msg[143:136] <= 8'h52;
                r_msg[151:144] <= 8'h43;
                r_msg[159:152] <= 8'h0A;
                r_msg[167:160] <= 8'h0D;
                r_msg_length <= 20;
            end
            3: // Test message
            begin
                r_msg[7:0] <= 8'h54;
                r_msg[15:8] <= 8'h65;
                r_msg[23:16] <= 8'h73;
                r_msg[31:24] <= 8'h74;
                r_msg[39:32] <= 8'h20;
                r_msg[47:40] <= 8'h6D;
                r_msg[55:48] <= 8'h65;
                r_msg[63:56] <= 8'h73;
                r_msg[71:64] <= 8'h73;
                r_msg[79:72] <= 8'h61;
                r_msg[87:80] <= 8'h67;
                r_msg[95:88] <= 8'h65;
                r_msg[103:96] <= 8'h0A;
                r_msg[111:104] <= 8'h0D;
                r_msg_length <= 14;
            end
            4: // "Segfault: exec data" — ERR_SEG_EXEC_DATA report (currently unused;
               //  the segmentation traps route through the HCF crash dump instead)
            begin
                r_msg[7:0]     <= 8'h53;  // S
                r_msg[15:8]    <= 8'h65;  // e
                r_msg[23:16]   <= 8'h67;  // g
                r_msg[31:24]   <= 8'h66;  // f
                r_msg[39:32]   <= 8'h61;  // a
                r_msg[47:40]   <= 8'h75;  // u
                r_msg[55:48]   <= 8'h6C;  // l
                r_msg[63:56]   <= 8'h74;  // t
                r_msg[71:64]   <= 8'h3A;  // :
                r_msg[79:72]   <= 8'h20;  // space
                r_msg[87:80]   <= 8'h65;  // e
                r_msg[95:88]   <= 8'h78;  // x
                r_msg[103:96]  <= 8'h65;  // e
                r_msg[111:104] <= 8'h63;  // c
                r_msg[119:112] <= 8'h20;  // space
                r_msg[127:120] <= 8'h64;  // d
                r_msg[135:128] <= 8'h61;  // a
                r_msg[143:136] <= 8'h74;  // t
                r_msg[151:144] <= 8'h61;  // a
                r_msg[159:152] <= 8'h0A;  // \n
                r_msg[167:160] <= 8'h0D;  // \r
                r_msg_length <= 8'd21;
            end
            default: begin
                r_msg[7:0]   <= 8'h00;
                r_msg_length <= 8'h0;
            end
        endcase
        r_msg_send_DV <= 1'b1;
    end
endtask

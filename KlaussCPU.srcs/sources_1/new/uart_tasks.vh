// Send debug message
// On completion
// Increment PC 1
// Increment r_SM_msg

// Blocking UART receive — stalls CPU until a byte is in the FIFO, then writes
// it zero-extended into the destination register. zero_flag is cleared (data valid).
// Opcode: RXRB R
task t_rx_blocking;
    begin
        if (w_rx_fifo_empty) begin
            r_SM <= OPCODE_EXECUTE;  // retry next clock
        end else begin
            r_rx_fifo_read            <= 1'b1;
            r_writeback_value         <= {24'b0, w_rx_fifo_byte};
            r_writeback_reg           <= r_reg_2;
            r_zero_flag               <= 1'b0;
            r_SM <= WRITEBACK;
            r_PC <= r_PC + 4;
        end
    end
endtask

// Non-blocking UART receive — always advances PC immediately.
// If FIFO has data: writes byte to register, zero_flag=0 (data valid).
// If FIFO empty:    leaves register unchanged, zero_flag=1 (no data).
// Programmer must check zero_flag before trusting the register value.
// Opcode: RXRNB R
task t_rx_nonblocking;
    begin
        if (w_rx_fifo_empty) begin
            r_zero_flag <= 1'b1;
            r_SM <= OPCODE_REQUEST;
            r_PC <= r_PC + 4;
        end else begin
            r_rx_fifo_read    <= 1'b1;
            r_writeback_value <= {24'b0, w_rx_fifo_byte};
            r_writeback_reg   <= r_reg_2;
            r_zero_flag       <= 1'b0;
            r_SM <= WRITEBACK;
            r_PC <= r_PC + 4;
        end
    end
endtask

task t_debug_message;
    begin
        if (!w_sending_msg) begin
            r_msg[7:0] <= 8'h50;  // P
            r_msg[15:8] <= 8'h43;  // C
            r_msg[23:16] <= 8'h20;  //  
            r_msg[31:24] <= 8'h69;  // i
            r_msg[39:32] <= 8'h73;  // s
            r_msg[47:40] <= 8'h20;  //  

            r_msg[55:48] <= return_ascii_from_hex({1'b0, r_PC[26:24]});
            r_msg[63:56] <= return_ascii_from_hex(r_PC[23:20]);
            r_msg[71:64] <= return_ascii_from_hex(r_PC[19:16]);
            r_msg[79:72] <= return_ascii_from_hex(r_PC[15:12]);
            r_msg[87:80] <= return_ascii_from_hex(r_PC[11:8]);
            r_msg[95:88] <= return_ascii_from_hex(r_PC[7:4]);
            r_msg[103:96] <= return_ascii_from_hex(r_PC[3:0]);

            r_msg[111:104] <= 8'h0D;  // CR

            r_msg_length <= 8'h0E;
            r_msg_send_DV <= 1'b1;
        end
    end

endtask  // Send test message
// On completion
// Increment PC 1
// Increment r_SM_msg

task t_test_message;
    begin
        if (!w_sending_msg) begin
            t_tx_message(8'd3);
            r_SM <= OPCODE_REQUEST;
            r_PC <= r_PC + 4;
        end
    end
endtask

// Print to serial values from register location
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_tx_char_from_reg_value;
    begin
        if (r_extra_clock == 0) begin
            r_mem_addr <= r_reg_port_b[26:0];
            r_mem_read_DV <= 1'b1;
            r_mem_was_ready <=1'b0;
            r_extra_clock <= 1'b1;
        end // if first loop
        else
        begin
            if (w_mem_ready) begin
                r_mem_read_DV <= 1'b0;
                r_mem_was_ready <= 1'b1;
                
            end       
            if ((w_mem_ready||r_mem_was_ready) && !w_sending_msg) begin
                // Little-endian: select the byte at the given byte-lane offset within
                // the 64-bit doubleword.  addr[2:0]=0 → bits[7:0] (LSByte), 7 → bits[63:56].
                case (r_reg_port_b[2:0])
                    3'b000: r_msg[7:0] <= w_mem_read_data[7:0];
                    3'b001: r_msg[7:0] <= w_mem_read_data[15:8];
                    3'b010: r_msg[7:0] <= w_mem_read_data[23:16];
                    3'b011: r_msg[7:0] <= w_mem_read_data[31:24];
                    3'b100: r_msg[7:0] <= w_mem_read_data[39:32];
                    3'b101: r_msg[7:0] <= w_mem_read_data[47:40];
                    3'b110: r_msg[7:0] <= w_mem_read_data[55:48];
                    3'b111: r_msg[7:0] <= w_mem_read_data[63:56];
                endcase
                r_msg_length <= 8'h1;
                r_msg_send_DV <= 1'b1;
                r_SM <= UART_DELAY;
                r_mem_read_DV <= 1'b0;
                r_PC <= r_PC + 4;
            end  // if ready asserted, else will loop until ready
        end  // if subsequent loop
    end
endtask

// Print to serial value from memory at register location
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_tx_value_of_mem_at_reg;
    begin
        if (r_extra_clock == 0) begin
            r_mem_addr <= r_reg_port_b[26:0];
            r_mem_read_DV <= 1'b1;
            r_mem_was_ready <=1'b0;
            r_extra_clock <= 1'b1;
        end // if first loop
        else
        begin
            if (w_mem_ready) begin
                r_mem_read_DV <= 1'b0;
                r_mem_was_ready <= 1'b1;
                
            end       
            if ((w_mem_ready||r_mem_was_ready) && !w_sending_msg) begin
                // Print all 64 bits as 16 hex chars, MSB first.
                r_msg[7:0]    <= return_ascii_from_hex(w_mem_read_data[63:60]);
                r_msg[15:8]   <= return_ascii_from_hex(w_mem_read_data[59:56]);
                r_msg[23:16]  <= return_ascii_from_hex(w_mem_read_data[55:52]);
                r_msg[31:24]  <= return_ascii_from_hex(w_mem_read_data[51:48]);
                r_msg[39:32]  <= return_ascii_from_hex(w_mem_read_data[47:44]);
                r_msg[47:40]  <= return_ascii_from_hex(w_mem_read_data[43:40]);
                r_msg[55:48]  <= return_ascii_from_hex(w_mem_read_data[39:36]);
                r_msg[63:56]  <= return_ascii_from_hex(w_mem_read_data[35:32]);
                r_msg[71:64]  <= return_ascii_from_hex(w_mem_read_data[31:28]);
                r_msg[79:72]  <= return_ascii_from_hex(w_mem_read_data[27:24]);
                r_msg[87:80]  <= return_ascii_from_hex(w_mem_read_data[23:20]);
                r_msg[95:88]  <= return_ascii_from_hex(w_mem_read_data[19:16]);
                r_msg[103:96] <= return_ascii_from_hex(w_mem_read_data[15:12]);
                r_msg[111:104]<= return_ascii_from_hex(w_mem_read_data[11:8]);
                r_msg[119:112]<= return_ascii_from_hex(w_mem_read_data[7:4]);
                r_msg[127:120]<= return_ascii_from_hex(w_mem_read_data[3:0]);
                r_msg_length <= 8'h10;
                r_msg_send_DV <= 1'b1;
                r_SM <= UART_DELAY;
                r_mem_read_DV <= 1'b0;
                r_PC <= r_PC + 4;
            end  // if ready asserted, else will loop until ready
        end  // if subsequent loop
    end
endtask

// Print to serial character from memory location
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_tx_value_of_mem;
    input [31:0] i_location;
    begin
        if (r_extra_clock == 0) begin
            r_mem_addr <= i_location[26:0];
            r_mem_read_DV <= 1'b1;
            r_mem_was_ready <=1'b0;
            r_extra_clock <= 1'b1;
        end // if first loop
        else
        begin
            if (w_mem_ready) begin
                r_mem_read_DV <= 1'b0;
                r_mem_was_ready <= 1'b1;
                
            end       
            if ((w_mem_ready||r_mem_was_ready) && !w_sending_msg) begin
                // Print all 64 bits as 16 hex chars, MSB first.
                r_msg[7:0]    <= return_ascii_from_hex(w_mem_read_data[63:60]);
                r_msg[15:8]   <= return_ascii_from_hex(w_mem_read_data[59:56]);
                r_msg[23:16]  <= return_ascii_from_hex(w_mem_read_data[55:52]);
                r_msg[31:24]  <= return_ascii_from_hex(w_mem_read_data[51:48]);
                r_msg[39:32]  <= return_ascii_from_hex(w_mem_read_data[47:44]);
                r_msg[47:40]  <= return_ascii_from_hex(w_mem_read_data[43:40]);
                r_msg[55:48]  <= return_ascii_from_hex(w_mem_read_data[39:36]);
                r_msg[63:56]  <= return_ascii_from_hex(w_mem_read_data[35:32]);
                r_msg[71:64]  <= return_ascii_from_hex(w_mem_read_data[31:28]);
                r_msg[79:72]  <= return_ascii_from_hex(w_mem_read_data[27:24]);
                r_msg[87:80]  <= return_ascii_from_hex(w_mem_read_data[23:20]);
                r_msg[95:88]  <= return_ascii_from_hex(w_mem_read_data[19:16]);
                r_msg[103:96] <= return_ascii_from_hex(w_mem_read_data[15:12]);
                r_msg[111:104]<= return_ascii_from_hex(w_mem_read_data[11:8]);
                r_msg[119:112]<= return_ascii_from_hex(w_mem_read_data[7:4]);
                r_msg[127:120]<= return_ascii_from_hex(w_mem_read_data[3:0]);
                r_msg_length <= 8'h10;
                r_msg_send_DV <= 1'b1;
                r_SM <= UART_DELAY;
                r_mem_read_DV <= 1'b0;
                r_PC <= r_PC + 8;
            end  // if ready asserted, else will loop until ready
        end  // if subsequent loop
    end
endtask

// Send null-terminated string from memory location (at imm32).
// Mirror of t_tx_string_at_reg — see that task for the state-machine commentary.
// On completion: r_PC += 8 (2-word instruction: opcode + imm32).
task t_tx_string_at_mem;
    input [31:0] i_location;
    reg [3:0]  null_pos;
    reg        has_null;
    reg [2:0]  offset;          // addr & 7 — non-zero only on the first chunk
    reg [3:0]  usable_bytes;    // 8 - offset
    reg [63:0] shifted_data;    // doubleword right-shifted so requested byte → pos 0
    begin
        if (r_tx_str_state_mem == 3'b000) begin
            // Latch base address and assert read DV. Then go to a 1-cycle
            // bubble (state 110) before checking w_mem_ready. The bubble
            // prevents state 001 from reading a stale w_mem_ready=1 left over
            // from the previous instruction's pipeline (typically the var1
            // fetch in OPCODE_FETCH2 → VAR1_FETCH path), which would otherwise
            // make state 010 scan the var1 doubleword instead of the string.
            r_tx_str_addr_mem  <= i_location[26:0];
            r_mem_addr         <= i_location[26:0];
            r_mem_read_DV      <= 1'b1;
            r_tx_str_state_mem <= 3'b110;
        end
        else if (r_tx_str_state_mem == 3'b110) begin
            r_tx_str_state_mem <= 3'b001;
        end
        else if (r_tx_str_state_mem == 3'b001) begin
            if (w_mem_ready) begin
                r_mem_read_DV      <= 1'b0;
                r_tx_str_state_mem <= 3'b010;
            end
        end
        else if (r_tx_str_state_mem == 3'b010) begin
            // Wait until any prior UART transmission has fully drained before
            // we queue our own. Tasks like uart_newline / t_tx_value_of_mem_at_reg
            // pulse r_msg_send_DV and drop straight to UART_DELAY without waiting
            // for the UART to actually latch — UART_DELAY only checks
            // !w_sending_msg, which is still 0 on that cycle, so the next
            // instruction starts while the UART is still mid-transmission. If
            // we don't gate here, state 011 below would see w_sending_msg=1
            // from that lingering prior message, treat it as "UART acknowledged
            // our DV", and clear DV before the UART had actually latched our
            // r_msg — silently dropping the first chunk.
            //
            // The cache returns 8-byte aligned doublewords. If the string base
            // is unaligned (offset = addr[2:0] != 0), bytes [0..offset-1] of
            // the doubleword are pre-string data (and may contain stray nulls
            // from prior code, e.g. the high bytes of a RET opcode), so we
            // must skip them. Shift right by offset bytes and scan only the
            // lower (8 - offset) bytes. After the first chunk, state 100
            // realigns to an 8-byte boundary so offset is always 0 thereafter.
            if (!w_sending_msg) begin
                offset       = r_tx_str_addr_mem[2:0];
                usable_bytes = 4'h8 - {1'b0, offset};
                case (offset)
                    3'h0:    shifted_data = w_mem_read_data;
                    3'h1:    shifted_data = {8'h00,  w_mem_read_data[63:8]};
                    3'h2:    shifted_data = {16'h00, w_mem_read_data[63:16]};
                    3'h3:    shifted_data = {24'h00, w_mem_read_data[63:24]};
                    3'h4:    shifted_data = {32'h00, w_mem_read_data[63:32]};
                    3'h5:    shifted_data = {40'h00, w_mem_read_data[63:40]};
                    3'h6:    shifted_data = {48'h00, w_mem_read_data[63:48]};
                    3'h7:    shifted_data = {56'h00, w_mem_read_data[63:56]};
                    default: shifted_data = w_mem_read_data;
                endcase

                null_pos = usable_bytes;
                has_null = 1'b0;
                if      ((usable_bytes >= 4'h1) && shifted_data[7:0]   == 8'h00) begin null_pos = 4'h0; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h2) && shifted_data[15:8]  == 8'h00) begin null_pos = 4'h1; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h3) && shifted_data[23:16] == 8'h00) begin null_pos = 4'h2; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h4) && shifted_data[31:24] == 8'h00) begin null_pos = 4'h3; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h5) && shifted_data[39:32] == 8'h00) begin null_pos = 4'h4; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h6) && shifted_data[47:40] == 8'h00) begin null_pos = 4'h5; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h7) && shifted_data[55:48] == 8'h00) begin null_pos = 4'h6; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h8) && shifted_data[63:56] == 8'h00) begin null_pos = 4'h7; has_null = 1'b1; end

                r_tx_str_done_mem <= has_null;
                r_msg[63:0]       <= shifted_data;

                if (has_null && null_pos == 4'h0) begin
                    r_SM               <= UART_DELAY;
                    r_PC               <= r_PC + 8;
                    r_tx_str_state_mem <= 3'b000;
                end else begin
                    r_msg_length       <= has_null ? {4'b0, null_pos} : {4'b0, usable_bytes};
                    r_msg_send_DV      <= 1'b1;
                    r_tx_str_state_mem <= 3'b011;
                end
            end
        end
        else if (r_tx_str_state_mem == 3'b011) begin
            if (w_sending_msg) begin
                r_msg_send_DV      <= 1'b0;
                r_tx_str_state_mem <= 3'b100;
            end
        end
        else if (r_tx_str_state_mem == 3'b100) begin
            if (i_msg_sent_DV) begin
                if (r_tx_str_done_mem) begin
                    r_SM               <= UART_DELAY;
                    r_PC               <= r_PC + 8;
                    r_tx_str_state_mem <= 3'b000;
                end else begin
                    // Advance to next 8-byte boundary. For an aligned current
                    // addr this is a regular +8; for an unaligned first chunk
                    // it rounds up so subsequent reads see offset 0.
                    r_mem_addr         <= (r_tx_str_addr_mem + 8) & ~32'h7;
                    r_mem_read_DV      <= 1'b1;
                    r_tx_str_addr_mem  <= (r_tx_str_addr_mem + 8) & ~32'h7;
                    r_tx_str_state_mem <= 3'b001;
                end
            end
        end
    end
endtask

// Send null-terminated string from memory location given by register.
// Reads consecutive doublewords until a null byte is found, sending each chunk
// over UART before advancing.
//
// State machine (3-bit r_tx_str_state_reg):
//   000  init: latch base address, issue first read
//   110  one-cycle bubble — lets stale w_mem_ready (from the previous
//        instruction's var1 fetch, which could still be high when we enter
//        OPCODE_EXECUTE) deassert before state 001 starts polling. Without
//        this, state 001 fires immediately on the stale ready, state 010
//        scans the var1 doubleword instead of the string base, and the
//        instruction either transmits nothing (var1 had a 0x00 byte at
//        position 0) or sends garbage from PC+4. Symptom: TXSTRMEMR works
//        intermittently depending on what instruction follows it in memory.
//   001  wait for memory read to complete
//   010  scan doubleword for null; queue chunk to UART (r_msg, r_msg_length, DV pulse)
//   011  wait for UART to acknowledge (w_sending_msg=1) — closes the race where
//        the CPU would otherwise read !w_sending_msg as still-idle on the cycle
//        immediately after setting DV. Once acknowledged, clear DV so the UART
//        does not auto-re-trigger the same message on its next IDLE.
//   100  wait for UART completion pulse (i_msg_sent_DV); then either advance
//        the pointer by 8 and loop (no null in chunk) or finish the instruction
//        (null in chunk — done flag latched in state 010). 100→001 doesn't
//        need the 110 bubble because by the time i_msg_sent_DV fires the cache
//        has been idle for thousands of cycles and ready is firmly 0.
//
// On completion: r_PC += 4 (1-word instruction).
//
// Unaligned base addresses: the cache returns 8-byte aligned doublewords
// regardless of the requested low bits, so for a string starting at addr
// where addr[2:0] != 0 the bytes [0..offset-1] of the first returned doubleword
// are pre-string data and may contain stray nulls (e.g. zero bytes from the
// high half of the preceding RET opcode). State 010 right-shifts the doubleword
// by `offset` bytes and scans only the lower `(8 - offset)` bytes; state 100
// then realigns to the next 8-byte boundary so subsequent reads have offset 0.
task t_tx_string_at_reg;
    reg [3:0]  null_pos;
    reg        has_null;
    reg [2:0]  offset;
    reg [3:0]  usable_bytes;
    reg [63:0] shifted_data;
    begin
        if (r_tx_str_state_reg == 3'b000) begin
            r_tx_str_addr_reg  <= r_reg_port_b[26:0];
            r_mem_addr         <= r_reg_port_b[26:0];
            r_mem_read_DV      <= 1'b1;
            r_tx_str_state_reg <= 3'b110;
        end
        else if (r_tx_str_state_reg == 3'b110) begin
            r_tx_str_state_reg <= 3'b001;
        end
        else if (r_tx_str_state_reg == 3'b001) begin
            if (w_mem_ready) begin
                r_mem_read_DV      <= 1'b0;
                r_tx_str_state_reg <= 3'b010;
            end
        end
        else if (r_tx_str_state_reg == 3'b010) begin
            // Wait for any prior UART transmission to fully drain before we
            // queue our own. Without this gate, state 011 below would see
            // w_sending_msg=1 from a lingering prior message (e.g. a
            // uart_newline that pulsed DV and dropped to UART_DELAY without
            // waiting for the UART to latch), treat that as "UART acknowledged
            // our DV", and clear DV before the UART had actually latched our
            // r_msg — silently dropping the first chunk of this transmission.
            if (!w_sending_msg) begin
                offset       = r_tx_str_addr_reg[2:0];
                usable_bytes = 4'h8 - {1'b0, offset};
                case (offset)
                    3'h0:    shifted_data = w_mem_read_data;
                    3'h1:    shifted_data = {8'h00,  w_mem_read_data[63:8]};
                    3'h2:    shifted_data = {16'h00, w_mem_read_data[63:16]};
                    3'h3:    shifted_data = {24'h00, w_mem_read_data[63:24]};
                    3'h4:    shifted_data = {32'h00, w_mem_read_data[63:32]};
                    3'h5:    shifted_data = {40'h00, w_mem_read_data[63:40]};
                    3'h6:    shifted_data = {48'h00, w_mem_read_data[63:48]};
                    3'h7:    shifted_data = {56'h00, w_mem_read_data[63:56]};
                    default: shifted_data = w_mem_read_data;
                endcase

                null_pos = usable_bytes;
                has_null = 1'b0;
                if      ((usable_bytes >= 4'h1) && shifted_data[7:0]   == 8'h00) begin null_pos = 4'h0; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h2) && shifted_data[15:8]  == 8'h00) begin null_pos = 4'h1; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h3) && shifted_data[23:16] == 8'h00) begin null_pos = 4'h2; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h4) && shifted_data[31:24] == 8'h00) begin null_pos = 4'h3; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h5) && shifted_data[39:32] == 8'h00) begin null_pos = 4'h4; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h6) && shifted_data[47:40] == 8'h00) begin null_pos = 4'h5; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h7) && shifted_data[55:48] == 8'h00) begin null_pos = 4'h6; has_null = 1'b1; end
                else if ((usable_bytes >= 4'h8) && shifted_data[63:56] == 8'h00) begin null_pos = 4'h7; has_null = 1'b1; end

                r_tx_str_done_reg <= has_null;
                r_msg[63:0]       <= shifted_data;

                if (has_null && null_pos == 4'h0) begin
                    // Either the string is empty (first-chunk byte 0 is null), or
                    // a subsequent chunk's byte 0 is null after prior chunks were
                    // already sent. Either way, this chunk has zero bytes to send;
                    // length=0 would otherwise underflow the UART byte counter.
                    r_SM               <= UART_DELAY;
                    r_PC               <= r_PC + 4;
                    r_tx_str_state_reg <= 3'b000;
                end else begin
                    r_msg_length       <= has_null ? {4'b0, null_pos} : {4'b0, usable_bytes};
                    r_msg_send_DV      <= 1'b1;
                    r_tx_str_state_reg <= 3'b011;
                end
            end
        end
        else if (r_tx_str_state_reg == 3'b011) begin
            // Wait for UART to acknowledge (start transmitting). Then clear DV
            // so the UART does not latch the same r_msg again when it next idles.
            if (w_sending_msg) begin
                r_msg_send_DV      <= 1'b0;
                r_tx_str_state_reg <= 3'b100;
            end
        end
        else if (r_tx_str_state_reg == 3'b100) begin
            // Wait for UART completion pulse (one cycle, fires on s_CLEANUP entry).
            if (i_msg_sent_DV) begin
                if (r_tx_str_done_reg) begin
                    r_SM               <= UART_DELAY;
                    r_PC               <= r_PC + 4;
                    r_tx_str_state_reg <= 3'b000;
                end else begin
                    // Round up to the next 8-byte boundary. For an aligned
                    // current addr this is a regular +8; for an unaligned
                    // first chunk it skips the partial bytes so subsequent
                    // chunks see offset 0 in state 010.
                    r_mem_addr         <= (r_tx_str_addr_reg + 8) & ~32'h7;
                    r_mem_read_DV      <= 1'b1;
                    r_tx_str_addr_reg  <= (r_tx_str_addr_reg + 8) & ~32'h7;
                    r_tx_str_state_reg <= 3'b001;
                end
            end
        end
    end
endtask


// Send message newline
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_tx_newline;
    begin
        if (!w_sending_msg) begin
            r_msg[7:0] <= 8'h0A;
            r_msg[15:8] <= 8'h0D;
            r_msg_length <= 8'h2;
            r_msg_send_DV <= 1'b1;
            r_SM <= UART_DELAY;
            r_PC <= r_PC + 4;
        end
    end
endtask

// Send message of reg contents (64-bit = 16 hex chars)
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_tx_reg;
    begin
        if (!w_sending_msg) begin
            r_msg[7:0]   <= return_ascii_from_hex(r_reg_port_b[63:60]);
            r_msg[15:8]  <= return_ascii_from_hex(r_reg_port_b[59:56]);
            r_msg[23:16] <= return_ascii_from_hex(r_reg_port_b[55:52]);
            r_msg[31:24] <= return_ascii_from_hex(r_reg_port_b[51:48]);
            r_msg[39:32] <= return_ascii_from_hex(r_reg_port_b[47:44]);
            r_msg[47:40] <= return_ascii_from_hex(r_reg_port_b[43:40]);
            r_msg[55:48] <= return_ascii_from_hex(r_reg_port_b[39:36]);
            r_msg[63:56] <= return_ascii_from_hex(r_reg_port_b[35:32]);
            r_msg[71:64] <= return_ascii_from_hex(r_reg_port_b[31:28]);
            r_msg[79:72] <= return_ascii_from_hex(r_reg_port_b[27:24]);
            r_msg[87:80] <= return_ascii_from_hex(r_reg_port_b[23:20]);
            r_msg[95:88] <= return_ascii_from_hex(r_reg_port_b[19:16]);
            r_msg[103:96]  <= return_ascii_from_hex(r_reg_port_b[15:12]);
            r_msg[111:104] <= return_ascii_from_hex(r_reg_port_b[11:8]);
            r_msg[119:112] <= return_ascii_from_hex(r_reg_port_b[7:4]);
            r_msg[127:120] <= return_ascii_from_hex(r_reg_port_b[3:0]);
            r_msg_length <= 8'h10;
            r_msg_send_DV <= 1'b1;
            r_SM <= UART_DELAY;
            r_PC <= r_PC + 4;
        end
    end
endtask


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
            4: // Segmentation error. Attempt to execute data.
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
                r_msg_length <= 8'h0E;
            end
            default: begin
                r_msg[7:0]   <= 8'h00;
                r_msg_length <= 8'h0;
            end
        endcase
        r_msg_send_DV <= 1'b1;
    end
endtask


// Build a single line of the HCF crash dump into r_msg / r_msg_length.
// Phase index is r_hcf_dump_phase (see DUMP_* localparams in KlaussCPU.v).
// Caller (HCF_DUMP state, PREP sub-state) handles UART handshake and phase advance.
//
// All lines fit within the 32-byte r_msg buffer.  Lines end with "\r\n" so a
// host terminal (minicom etc.) renders one dump field per line.
//
// Phases:
//   0           "*** CRASH DUMP ***"        (banner)
//   1           "ERR=xx PC=xxxxxxxx"
//   2           "OPC=xxxxxxxx SP=xxxxxxxx"
//   3           "V1=xxxxxxxx IDX=xxxxxxxx"
//   4           "FLG Z=x E=x C=x V=x"
//   5           "    S=x L=x U=x"
//   6..21       "RX=NNNNNNNNNNNNNNNN"       16 register dumps (R0..RF, hex index)
//   22..25      "SX=NNNNNNNNNNNNNNNN"       4 top-of-stack doublewords (S0..S3)
//   26..41      "TX P=xxxxxxxx OP=xxxxxxxx" 16 trace entries (T0..TF, newest-first)
//   42          "*** END ***"               (footer)
task t_hcf_dump_build_line;
    reg [5:0]  k_phase;     // phase relative to base for repeated phases
    reg [3:0]  trace_pos;   // ring index for trace lookup
    reg [63:0] reg_val;
    reg [63:0] trace_val;
    begin
        case (r_hcf_dump_phase)
            DUMP_HEADER: begin
                // "\r\n*** CRASH DUMP ***\r\n"  (22 bytes)
                r_msg[ 0*8 +: 8] <= 8'h0D;
                r_msg[ 1*8 +: 8] <= 8'h0A;
                r_msg[ 2*8 +: 8] <= "*";
                r_msg[ 3*8 +: 8] <= "*";
                r_msg[ 4*8 +: 8] <= "*";
                r_msg[ 5*8 +: 8] <= " ";
                r_msg[ 6*8 +: 8] <= "C";
                r_msg[ 7*8 +: 8] <= "R";
                r_msg[ 8*8 +: 8] <= "A";
                r_msg[ 9*8 +: 8] <= "S";
                r_msg[10*8 +: 8] <= "H";
                r_msg[11*8 +: 8] <= " ";
                r_msg[12*8 +: 8] <= "D";
                r_msg[13*8 +: 8] <= "U";
                r_msg[14*8 +: 8] <= "M";
                r_msg[15*8 +: 8] <= "P";
                r_msg[16*8 +: 8] <= " ";
                r_msg[17*8 +: 8] <= "*";
                r_msg[18*8 +: 8] <= "*";
                r_msg[19*8 +: 8] <= "*";
                r_msg[20*8 +: 8] <= 8'h0D;
                r_msg[21*8 +: 8] <= 8'h0A;
                r_msg_length     <= 8'd22;
            end

            DUMP_ERR_PC: begin
                // "ERR=xx PC=xxxxxxxx\r\n"  (20 bytes)
                r_msg[ 0*8 +: 8] <= "E";
                r_msg[ 1*8 +: 8] <= "R";
                r_msg[ 2*8 +: 8] <= "R";
                r_msg[ 3*8 +: 8] <= "=";
                r_msg[ 4*8 +: 8] <= return_ascii_from_hex(r_error_code[7:4]);
                r_msg[ 5*8 +: 8] <= return_ascii_from_hex(r_error_code[3:0]);
                r_msg[ 6*8 +: 8] <= " ";
                r_msg[ 7*8 +: 8] <= "P";
                r_msg[ 8*8 +: 8] <= "C";
                r_msg[ 9*8 +: 8] <= "=";
                r_msg[10*8 +: 8] <= return_ascii_from_hex(r_PC[31:28]);
                r_msg[11*8 +: 8] <= return_ascii_from_hex(r_PC[27:24]);
                r_msg[12*8 +: 8] <= return_ascii_from_hex(r_PC[23:20]);
                r_msg[13*8 +: 8] <= return_ascii_from_hex(r_PC[19:16]);
                r_msg[14*8 +: 8] <= return_ascii_from_hex(r_PC[15:12]);
                r_msg[15*8 +: 8] <= return_ascii_from_hex(r_PC[11: 8]);
                r_msg[16*8 +: 8] <= return_ascii_from_hex(r_PC[ 7: 4]);
                r_msg[17*8 +: 8] <= return_ascii_from_hex(r_PC[ 3: 0]);
                r_msg[18*8 +: 8] <= 8'h0D;
                r_msg[19*8 +: 8] <= 8'h0A;
                r_msg_length     <= 8'd20;
            end

            DUMP_OPC_SP: begin
                // "OPC=xxxxxxxx SP=xxxxxxxx\r\n"  (26 bytes)
                r_msg[ 0*8 +: 8] <= "O";
                r_msg[ 1*8 +: 8] <= "P";
                r_msg[ 2*8 +: 8] <= "C";
                r_msg[ 3*8 +: 8] <= "=";
                r_msg[ 4*8 +: 8] <= return_ascii_from_hex(w_opcode[31:28]);
                r_msg[ 5*8 +: 8] <= return_ascii_from_hex(w_opcode[27:24]);
                r_msg[ 6*8 +: 8] <= return_ascii_from_hex(w_opcode[23:20]);
                r_msg[ 7*8 +: 8] <= return_ascii_from_hex(w_opcode[19:16]);
                r_msg[ 8*8 +: 8] <= return_ascii_from_hex(w_opcode[15:12]);
                r_msg[ 9*8 +: 8] <= return_ascii_from_hex(w_opcode[11: 8]);
                r_msg[10*8 +: 8] <= return_ascii_from_hex(w_opcode[ 7: 4]);
                r_msg[11*8 +: 8] <= return_ascii_from_hex(w_opcode[ 3: 0]);
                r_msg[12*8 +: 8] <= " ";
                r_msg[13*8 +: 8] <= "S";
                r_msg[14*8 +: 8] <= "P";
                r_msg[15*8 +: 8] <= "=";
                r_msg[16*8 +: 8] <= return_ascii_from_hex(r_SP[31:28]);
                r_msg[17*8 +: 8] <= return_ascii_from_hex(r_SP[27:24]);
                r_msg[18*8 +: 8] <= return_ascii_from_hex(r_SP[23:20]);
                r_msg[19*8 +: 8] <= return_ascii_from_hex(r_SP[19:16]);
                r_msg[20*8 +: 8] <= return_ascii_from_hex(r_SP[15:12]);
                r_msg[21*8 +: 8] <= return_ascii_from_hex(r_SP[11: 8]);
                r_msg[22*8 +: 8] <= return_ascii_from_hex(r_SP[ 7: 4]);
                r_msg[23*8 +: 8] <= return_ascii_from_hex(r_SP[ 3: 0]);
                r_msg[24*8 +: 8] <= 8'h0D;
                r_msg[25*8 +: 8] <= 8'h0A;
                r_msg_length     <= 8'd26;
            end

            DUMP_V1_V2: begin
                // "V1=xxxxxxxx IDX=xxxxxxxx\r\n"  (26 bytes)
                // V1  = w_var1, the 32-bit immediate at PC+4 (instruction operand
                //       fetched in OPCODE_FETCH / VAR1_FETCH).
                // IDX = r_idx_base_addr, the saved base address used by indexed
                //       register ops (LDIDX/STIDX family).  Replaces the dead
                //       w_var2 wire which has no driver in the current FSM.
                r_msg[ 0*8 +: 8] <= "V";
                r_msg[ 1*8 +: 8] <= "1";
                r_msg[ 2*8 +: 8] <= "=";
                r_msg[ 3*8 +: 8] <= return_ascii_from_hex(w_var1[31:28]);
                r_msg[ 4*8 +: 8] <= return_ascii_from_hex(w_var1[27:24]);
                r_msg[ 5*8 +: 8] <= return_ascii_from_hex(w_var1[23:20]);
                r_msg[ 6*8 +: 8] <= return_ascii_from_hex(w_var1[19:16]);
                r_msg[ 7*8 +: 8] <= return_ascii_from_hex(w_var1[15:12]);
                r_msg[ 8*8 +: 8] <= return_ascii_from_hex(w_var1[11: 8]);
                r_msg[ 9*8 +: 8] <= return_ascii_from_hex(w_var1[ 7: 4]);
                r_msg[10*8 +: 8] <= return_ascii_from_hex(w_var1[ 3: 0]);
                r_msg[11*8 +: 8] <= " ";
                r_msg[12*8 +: 8] <= "I";
                r_msg[13*8 +: 8] <= "D";
                r_msg[14*8 +: 8] <= "X";
                r_msg[15*8 +: 8] <= "=";
                r_msg[16*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[31:28]);
                r_msg[17*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[27:24]);
                r_msg[18*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[23:20]);
                r_msg[19*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[19:16]);
                r_msg[20*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[15:12]);
                r_msg[21*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[11: 8]);
                r_msg[22*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[ 7: 4]);
                r_msg[23*8 +: 8] <= return_ascii_from_hex(r_idx_base_addr[ 3: 0]);
                r_msg[24*8 +: 8] <= 8'h0D;
                r_msg[25*8 +: 8] <= 8'h0A;
                r_msg_length     <= 8'd26;
            end

            DUMP_FLAGS_A: begin
                // "FLG Z=x E=x C=x V=x\r\n"  (21 bytes)
                r_msg[ 0*8 +: 8] <= "F";
                r_msg[ 1*8 +: 8] <= "L";
                r_msg[ 2*8 +: 8] <= "G";
                r_msg[ 3*8 +: 8] <= " ";
                r_msg[ 4*8 +: 8] <= "Z";
                r_msg[ 5*8 +: 8] <= "=";
                r_msg[ 6*8 +: 8] <= r_zero_flag    ? "1" : "0";
                r_msg[ 7*8 +: 8] <= " ";
                r_msg[ 8*8 +: 8] <= "E";
                r_msg[ 9*8 +: 8] <= "=";
                r_msg[10*8 +: 8] <= r_equal_flag   ? "1" : "0";
                r_msg[11*8 +: 8] <= " ";
                r_msg[12*8 +: 8] <= "C";
                r_msg[13*8 +: 8] <= "=";
                r_msg[14*8 +: 8] <= r_carry_flag   ? "1" : "0";
                r_msg[15*8 +: 8] <= " ";
                r_msg[16*8 +: 8] <= "V";
                r_msg[17*8 +: 8] <= "=";
                r_msg[18*8 +: 8] <= r_overflow_flag ? "1" : "0";
                r_msg[19*8 +: 8] <= 8'h0D;
                r_msg[20*8 +: 8] <= 8'h0A;
                r_msg_length     <= 8'd21;
            end

            DUMP_FLAGS_B: begin
                // "    S=x L=x U=x\r\n"  (17 bytes)
                r_msg[ 0*8 +: 8] <= " ";
                r_msg[ 1*8 +: 8] <= " ";
                r_msg[ 2*8 +: 8] <= " ";
                r_msg[ 3*8 +: 8] <= " ";
                r_msg[ 4*8 +: 8] <= "S";
                r_msg[ 5*8 +: 8] <= "=";
                r_msg[ 6*8 +: 8] <= r_sign_flag ? "1" : "0";
                r_msg[ 7*8 +: 8] <= " ";
                r_msg[ 8*8 +: 8] <= "L";
                r_msg[ 9*8 +: 8] <= "=";
                r_msg[10*8 +: 8] <= r_less_flag ? "1" : "0";
                r_msg[11*8 +: 8] <= " ";
                r_msg[12*8 +: 8] <= "U";
                r_msg[13*8 +: 8] <= "=";
                r_msg[14*8 +: 8] <= r_ult_flag  ? "1" : "0";
                r_msg[15*8 +: 8] <= 8'h0D;
                r_msg[16*8 +: 8] <= 8'h0A;
                r_msg_length     <= 8'd17;
            end

            DUMP_FOOTER: begin
                // "*** END ***\r\n"  (13 bytes)
                r_msg[ 0*8 +: 8] <= "*";
                r_msg[ 1*8 +: 8] <= "*";
                r_msg[ 2*8 +: 8] <= "*";
                r_msg[ 3*8 +: 8] <= " ";
                r_msg[ 4*8 +: 8] <= "E";
                r_msg[ 5*8 +: 8] <= "N";
                r_msg[ 6*8 +: 8] <= "D";
                r_msg[ 7*8 +: 8] <= " ";
                r_msg[ 8*8 +: 8] <= "*";
                r_msg[ 9*8 +: 8] <= "*";
                r_msg[10*8 +: 8] <= "*";
                r_msg[11*8 +: 8] <= 8'h0D;
                r_msg[12*8 +: 8] <= 8'h0A;
                r_msg_length     <= 8'd13;
            end

            default: begin
                // Repeated-line phases — pick the right family from the phase
                // index.  Note that DUMP_TRACE_BASE+15 = DUMP_FOOTER-1, so the
                // "< DUMP_FOOTER" guard is enough to keep this branch safe.
                if (r_hcf_dump_phase < DUMP_STACK_BASE) begin
                    // ============== Register dump (R0..RF) ==============
                    // "RX=NNNNNNNNNNNNNNNN\r\n"  (21 bytes)
                    k_phase = r_hcf_dump_phase - DUMP_REG_BASE;
                    reg_val = r_register[k_phase[3:0]];
                    r_msg[ 0*8 +: 8] <= "R";
                    r_msg[ 1*8 +: 8] <= return_ascii_from_hex(k_phase[3:0]);
                    r_msg[ 2*8 +: 8] <= "=";
                    r_msg[ 3*8 +: 8] <= return_ascii_from_hex(reg_val[63:60]);
                    r_msg[ 4*8 +: 8] <= return_ascii_from_hex(reg_val[59:56]);
                    r_msg[ 5*8 +: 8] <= return_ascii_from_hex(reg_val[55:52]);
                    r_msg[ 6*8 +: 8] <= return_ascii_from_hex(reg_val[51:48]);
                    r_msg[ 7*8 +: 8] <= return_ascii_from_hex(reg_val[47:44]);
                    r_msg[ 8*8 +: 8] <= return_ascii_from_hex(reg_val[43:40]);
                    r_msg[ 9*8 +: 8] <= return_ascii_from_hex(reg_val[39:36]);
                    r_msg[10*8 +: 8] <= return_ascii_from_hex(reg_val[35:32]);
                    r_msg[11*8 +: 8] <= return_ascii_from_hex(reg_val[31:28]);
                    r_msg[12*8 +: 8] <= return_ascii_from_hex(reg_val[27:24]);
                    r_msg[13*8 +: 8] <= return_ascii_from_hex(reg_val[23:20]);
                    r_msg[14*8 +: 8] <= return_ascii_from_hex(reg_val[19:16]);
                    r_msg[15*8 +: 8] <= return_ascii_from_hex(reg_val[15:12]);
                    r_msg[16*8 +: 8] <= return_ascii_from_hex(reg_val[11: 8]);
                    r_msg[17*8 +: 8] <= return_ascii_from_hex(reg_val[ 7: 4]);
                    r_msg[18*8 +: 8] <= return_ascii_from_hex(reg_val[ 3: 0]);
                    r_msg[19*8 +: 8] <= 8'h0D;
                    r_msg[20*8 +: 8] <= 8'h0A;
                    r_msg_length     <= 8'd21;
                end else if (r_hcf_dump_phase < DUMP_TRACE_BASE) begin
                    // ============== Stack dump (S0..S3) ==============
                    // "SX=NNNNNNNNNNNNNNNN\r\n"  (21 bytes)
                    // Data was pre-fetched in HCF_DUMP STACK_FETCH sub-state
                    // and is sitting in r_hcf_stack_data.
                    k_phase = r_hcf_dump_phase - DUMP_STACK_BASE;
                    r_msg[ 0*8 +: 8] <= "S";
                    r_msg[ 1*8 +: 8] <= return_ascii_from_hex(k_phase[3:0]);
                    r_msg[ 2*8 +: 8] <= "=";
                    r_msg[ 3*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[63:60]);
                    r_msg[ 4*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[59:56]);
                    r_msg[ 5*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[55:52]);
                    r_msg[ 6*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[51:48]);
                    r_msg[ 7*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[47:44]);
                    r_msg[ 8*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[43:40]);
                    r_msg[ 9*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[39:36]);
                    r_msg[10*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[35:32]);
                    r_msg[11*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[31:28]);
                    r_msg[12*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[27:24]);
                    r_msg[13*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[23:20]);
                    r_msg[14*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[19:16]);
                    r_msg[15*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[15:12]);
                    r_msg[16*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[11: 8]);
                    r_msg[17*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[ 7: 4]);
                    r_msg[18*8 +: 8] <= return_ascii_from_hex(r_hcf_stack_data[ 3: 0]);
                    r_msg[19*8 +: 8] <= 8'h0D;
                    r_msg[20*8 +: 8] <= 8'h0A;
                    r_msg_length     <= 8'd21;
                end else begin
                    // ============== Trace dump (T0..TF, newest-first) ==============
                    // "TX P=xxxxxxxx OP=xxxxxxxx\r\n"  (27 bytes)
                    // T0 is the most-recent fetch; T15 is 16 fetches back.
                    // Buffer layout: r_trace_idx points to the *next* write
                    // slot, so the most recent entry is at r_trace_idx-1.
                    k_phase   = r_hcf_dump_phase - DUMP_TRACE_BASE;
                    trace_pos = r_trace_idx - 4'd1 - k_phase[3:0];
                    trace_val = r_trace_buf[trace_pos];
                    r_msg[ 0*8 +: 8] <= "T";
                    r_msg[ 1*8 +: 8] <= return_ascii_from_hex(k_phase[3:0]);
                    r_msg[ 2*8 +: 8] <= " ";
                    r_msg[ 3*8 +: 8] <= "P";
                    r_msg[ 4*8 +: 8] <= "=";
                    r_msg[ 5*8 +: 8] <= return_ascii_from_hex(trace_val[63:60]);
                    r_msg[ 6*8 +: 8] <= return_ascii_from_hex(trace_val[59:56]);
                    r_msg[ 7*8 +: 8] <= return_ascii_from_hex(trace_val[55:52]);
                    r_msg[ 8*8 +: 8] <= return_ascii_from_hex(trace_val[51:48]);
                    r_msg[ 9*8 +: 8] <= return_ascii_from_hex(trace_val[47:44]);
                    r_msg[10*8 +: 8] <= return_ascii_from_hex(trace_val[43:40]);
                    r_msg[11*8 +: 8] <= return_ascii_from_hex(trace_val[39:36]);
                    r_msg[12*8 +: 8] <= return_ascii_from_hex(trace_val[35:32]);
                    r_msg[13*8 +: 8] <= " ";
                    r_msg[14*8 +: 8] <= "O";
                    r_msg[15*8 +: 8] <= "P";
                    r_msg[16*8 +: 8] <= "=";
                    r_msg[17*8 +: 8] <= return_ascii_from_hex(trace_val[31:28]);
                    r_msg[18*8 +: 8] <= return_ascii_from_hex(trace_val[27:24]);
                    r_msg[19*8 +: 8] <= return_ascii_from_hex(trace_val[23:20]);
                    r_msg[20*8 +: 8] <= return_ascii_from_hex(trace_val[19:16]);
                    r_msg[21*8 +: 8] <= return_ascii_from_hex(trace_val[15:12]);
                    r_msg[22*8 +: 8] <= return_ascii_from_hex(trace_val[11: 8]);
                    r_msg[23*8 +: 8] <= return_ascii_from_hex(trace_val[ 7: 4]);
                    r_msg[24*8 +: 8] <= return_ascii_from_hex(trace_val[ 3: 0]);
                    r_msg[25*8 +: 8] <= 8'h0D;
                    r_msg[26*8 +: 8] <= 8'h0A;
                    r_msg_length     <= 8'd27;
                end
            end
        endcase
    end
endtask

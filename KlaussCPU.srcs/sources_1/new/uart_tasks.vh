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
//   6           "SM=xxxxxxxxx"              (FSM state, 33-bit one-hot)
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
    reg   [3:0]  k;          // family index (which reg / stack / trace entry)
    reg   [63:0] data;       // 64-bit data word for family lines
    reg   [31:0] half;       // 32-bit V1H/OPCM half-word
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
                    5'd4:  f_dump_byte = return_ascii_from_hex(r_error_code[7:4]);
                    5'd5:  f_dump_byte = return_ascii_from_hex(r_error_code[3:0]);
                    5'd6:  f_dump_byte = " ";
                    5'd7:  f_dump_byte = "P";
                    5'd8:  f_dump_byte = "C";
                    5'd9:  f_dump_byte = "=";
                    5'd10: f_dump_byte = return_ascii_from_hex(r_PC[31:28]);
                    5'd11: f_dump_byte = return_ascii_from_hex(r_PC[27:24]);
                    5'd12: f_dump_byte = return_ascii_from_hex(r_PC[23:20]);
                    5'd13: f_dump_byte = return_ascii_from_hex(r_PC[19:16]);
                    5'd14: f_dump_byte = return_ascii_from_hex(r_PC[15:12]);
                    5'd15: f_dump_byte = return_ascii_from_hex(r_PC[11: 8]);
                    5'd16: f_dump_byte = return_ascii_from_hex(r_PC[ 7: 4]);
                    5'd17: f_dump_byte = return_ascii_from_hex(r_PC[ 3: 0]);
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
                    5'd16: f_dump_byte = return_ascii_from_hex(r_SP[31:28]);
                    5'd17: f_dump_byte = return_ascii_from_hex(r_SP[27:24]);
                    5'd18: f_dump_byte = return_ascii_from_hex(r_SP[23:20]);
                    5'd19: f_dump_byte = return_ascii_from_hex(r_SP[19:16]);
                    5'd20: f_dump_byte = return_ascii_from_hex(r_SP[15:12]);
                    5'd21: f_dump_byte = return_ascii_from_hex(r_SP[11: 8]);
                    5'd22: f_dump_byte = return_ascii_from_hex(r_SP[ 7: 4]);
                    5'd23: f_dump_byte = return_ascii_from_hex(r_SP[ 3: 0]);
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
                    5'd16: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[31:28]);
                    5'd17: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[27:24]);
                    5'd18: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[23:20]);
                    5'd19: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[19:16]);
                    5'd20: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[15:12]);
                    5'd21: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[11: 8]);
                    5'd22: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[ 7: 4]);
                    5'd23: f_dump_byte = return_ascii_from_hex(r_idx_base_addr[ 3: 0]);
                    5'd24: f_dump_byte = 8'h0D;
                    5'd25: f_dump_byte = 8'h0A;
                    default: f_dump_byte = 8'h00;
                endcase
            end

            DUMP_V1H: begin
                // "V1H=xxxxxxxx\r\n"  (hi32 picked by r_PC[2])
                half = r_PC[2] ? r_hcf_stack_data[63:32] : r_hcf_stack_data[31:0];
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
                half = r_PC[2] ? r_hcf_stack_data[63:32] : r_hcf_stack_data[31:0];
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
                // "SM=xxxxxxxxx\r\n"  (33-bit one-hot FSM)
                case (pos)
                    5'd0:  f_dump_byte = "S";
                    5'd1:  f_dump_byte = "M";
                    5'd2:  f_dump_byte = "=";
                    5'd3:  f_dump_byte = return_ascii_from_hex({3'b0, r_fault_sm[32]});
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
                    5'd6:  f_dump_byte = r_zero_flag     ? "1" : "0";
                    5'd7:  f_dump_byte = " ";
                    5'd8:  f_dump_byte = "E";
                    5'd9:  f_dump_byte = "=";
                    5'd10: f_dump_byte = r_equal_flag    ? "1" : "0";
                    5'd11: f_dump_byte = " ";
                    5'd12: f_dump_byte = "C";
                    5'd13: f_dump_byte = "=";
                    5'd14: f_dump_byte = r_carry_flag    ? "1" : "0";
                    5'd15: f_dump_byte = " ";
                    5'd16: f_dump_byte = "V";
                    5'd17: f_dump_byte = "=";
                    5'd18: f_dump_byte = r_overflow_flag ? "1" : "0";
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
                    5'd6:  f_dump_byte = r_sign_flag ? "1" : "0";
                    5'd7:  f_dump_byte = " ";
                    5'd8:  f_dump_byte = "L";
                    5'd9:  f_dump_byte = "=";
                    5'd10: f_dump_byte = r_less_flag ? "1" : "0";
                    5'd11: f_dump_byte = " ";
                    5'd12: f_dump_byte = "U";
                    5'd13: f_dump_byte = "=";
                    5'd14: f_dump_byte = r_ult_flag  ? "1" : "0";
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
                    // Register dump R0..RF
                    k    = phase[3:0] - DUMP_REG_BASE[3:0];
                    data = r_register[k];
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


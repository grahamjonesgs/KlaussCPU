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

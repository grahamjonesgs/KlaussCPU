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

// Print to serial character from memory location
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_tx_string_at_mem;
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
                // Little-endian: send 8 bytes in address order (lowest byte first).
                // addr+0 → bits[7:0], addr+1 → bits[15:8], … addr+7 → bits[63:56].
                r_msg[7:0]   <= w_mem_read_data[7:0];
                r_msg[15:8]  <= w_mem_read_data[15:8];
                r_msg[23:16] <= w_mem_read_data[23:16];
                r_msg[31:24] <= w_mem_read_data[31:24];
                r_msg[39:32] <= w_mem_read_data[39:32];
                r_msg[47:40] <= w_mem_read_data[47:40];
                r_msg[55:48] <= w_mem_read_data[55:48];
                r_msg[63:56] <= w_mem_read_data[63:56];
                r_msg_length <= 8'h8;
                r_msg_send_DV <= 1'b1;
                r_SM <= UART_DELAY;
                r_mem_read_DV <= 1'b0;
                r_PC <= r_PC + 8;
            end  // if ready asserted, else will loop until ready
        end  // if subsequent loop
    end
endtask

// Print to serial character from memory location given by register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_tx_string_at_reg;
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
                // Little-endian: send 8 bytes in address order (lowest byte first).
                // addr+0 → bits[7:0], addr+1 → bits[15:8], … addr+7 → bits[63:56].
                r_msg[7:0]   <= w_mem_read_data[7:0];
                r_msg[15:8]  <= w_mem_read_data[15:8];
                r_msg[23:16] <= w_mem_read_data[23:16];
                r_msg[31:24] <= w_mem_read_data[31:24];
                r_msg[39:32] <= w_mem_read_data[39:32];
                r_msg[47:40] <= w_mem_read_data[47:40];
                r_msg[55:48] <= w_mem_read_data[55:48];
                r_msg[63:56] <= w_mem_read_data[63:56];
                r_msg_length <= 8'h8;
                r_msg_send_DV <= 1'b1;
                r_SM <= UART_DELAY;
                r_mem_read_DV <= 1'b0;
                r_PC <= r_PC + 4;
            end  // if ready asserted, else will loop until ready
        end  // if subsequent loop
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

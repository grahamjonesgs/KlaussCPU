// Set mem location given in value to contents of register
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_set_mem_from_value_reg;
   input [31:0] i_location;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr <= i_location[31:0];
         r_mem_write_data <= r_reg_port_b;
         r_mem_write_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end // if first loop
        else
        begin
         if (w_mem_ready) begin
            r_SM <= OPCODE_REQUEST;
            r_PC <= r_PC + 8;
            r_mem_write_DV <= 1'b0;
         end  // if ready asserted, else will loop until ready
      end  // if subsequent loop
   end
endtask

// Set mem location given in register to contents of register (first in order is value, second is location)
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_set_mem_from_reg_reg;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr <= r_reg_port_b[31:0];
         r_mem_write_data <= r_reg_port_a;
         r_mem_write_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end // if first loop
        else
        begin
         if (w_mem_ready) begin
            r_SM <= OPCODE_REQUEST;
            r_PC <= r_PC + 4;
            r_mem_write_DV <= 1'b0;
         end  // if ready asserted, else will loop until ready
      end  // if subsequent loop
   end
endtask

// Set contents of register to location given in value
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_set_reg_from_mem_value;
   input [31:0] i_location;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr <= i_location[31:0];
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end // if first loop
        else
        begin
         if (w_mem_ready) begin
            r_writeback_value <= w_mem_read_data;
            r_writeback_reg <= r_reg_2;
            r_SM <= WRITEBACK;
            r_mem_read_DV <= 1'b0;
            r_PC <= r_PC + 8;
         end  // if ready asserted, else will loop until ready
      end  // if subsequent loop
   end
endtask

// Set contents of register to location given in register (first in order is reg to be set, second is location)
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_set_reg_from_mem_reg;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr <= r_reg_port_b[31:0];
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end // if first loop
        else
        begin
         if (w_mem_ready) begin
            r_writeback_value <= w_mem_read_data;
            r_writeback_reg <= r_reg_1;
            r_SM <= WRITEBACK;
            r_mem_read_DV <= 1'b0;
            if (r_mem_read_DV) begin
               r_PC <= r_PC + 4;
            end
         end  // if ready asserted, else will loop until ready
      end  // if subsequent loop
   end
endtask

// MEMSET8 - Write one byte to byte address in register
// Little-endian 64-bit bus: byte_addr[2:0]=0 → bits[7:0] (LSB), 7→bits[63:56] (MSB)
// 1-word instruction (PC+4)
task t_memset8;
   reg [2:0] byte_lane;
   begin
      if (r_extra_clock == 0) begin
         byte_lane        = r_reg_port_b[2:0];
         r_mem_addr       <= r_reg_port_b[31:0];
         r_mem_write_data <= {8{r_reg_port_a[7:0]}};  // replicated; only enabled lane used
         // Little-endian byte enables: lane 0 = bit[0] (LSB), lane 7 = bit[7] (MSB)
         r_mem_byte_en    <= 8'b0000_0001 << byte_lane;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_write_DV <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
            r_PC           <= r_PC + 4;
         end
      end
   end
endtask

// MEMGET8 - Read one byte from byte address in register, zero-extended into dest register
// Little-endian 64-bit bus: byte_addr[2:0]=0 → bits[7:0] (LSB), 7→bits[63:56] (MSB)
// 1-word instruction (PC+4)
task t_memget8;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr    <= r_reg_port_b[31:0];
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_read_DV <= 1'b0;
            case (r_reg_port_b[2:0])
               3'b000: r_writeback_value <= {56'b0, w_mem_read_data[7:0]};
               3'b001: r_writeback_value <= {56'b0, w_mem_read_data[15:8]};
               3'b010: r_writeback_value <= {56'b0, w_mem_read_data[23:16]};
               3'b011: r_writeback_value <= {56'b0, w_mem_read_data[31:24]};
               3'b100: r_writeback_value <= {56'b0, w_mem_read_data[39:32]};
               3'b101: r_writeback_value <= {56'b0, w_mem_read_data[47:40]};
               3'b110: r_writeback_value <= {56'b0, w_mem_read_data[55:48]};
               3'b111: r_writeback_value <= {56'b0, w_mem_read_data[63:56]};
            endcase
            r_writeback_reg <= r_reg_1;
            r_SM            <= WRITEBACK;
            r_PC            <= r_PC + 4;
         end
      end
   end
endtask

// MEMSET16 - Write 16-bit halfword to byte address in register
// Little-endian 64-bit bus, 2-byte aligned
task t_memset16;
   reg [2:0] byte_lane;
   begin
      if (r_extra_clock == 0) begin
         byte_lane        = {r_reg_port_b[2:1], 1'b0};  // aligned to 2-byte boundary
         r_mem_addr       <= {r_reg_port_b[31:1], 1'b0};
         r_mem_write_data <= {4{r_reg_port_a[15:0]}};    // replicated; only enabled lanes used
         // Little-endian: lane 0 = bits[15:0] (LSH), lane 6 = bits[63:48] (MSH)
         r_mem_byte_en    <= 8'b0000_0011 << byte_lane;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_write_DV <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
            r_PC           <= r_PC + 4;
         end
      end
   end
endtask

// MEMGET16 - Read 16-bit halfword, zero-extended into dest register
// Little-endian 64-bit bus, 2-byte aligned
task t_memget16;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr    <= {r_reg_port_b[31:1], 1'b0};
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_read_DV <= 1'b0;
            case (r_reg_port_b[2:1])
               2'b00: r_writeback_value <= {48'b0, w_mem_read_data[15:0]};
               2'b01: r_writeback_value <= {48'b0, w_mem_read_data[31:16]};
               2'b10: r_writeback_value <= {48'b0, w_mem_read_data[47:32]};
               2'b11: r_writeback_value <= {48'b0, w_mem_read_data[63:48]};
            endcase
            r_writeback_reg <= r_reg_1;
            r_SM            <= WRITEBACK;
            r_PC            <= r_PC + 4;
         end
      end
   end
endtask

// MEMSET32 - Write 32-bit word to byte-aligned address
// Little-endian: addr[2]=0 → LOW_HALF bits[31:0], addr[2]=1 → HIGH_HALF bits[63:32]
task t_memset32;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr       <= {r_reg_port_b[31:2], 2'b00};
         r_mem_write_data <= {r_reg_port_a[31:0], r_reg_port_a[31:0]};
         r_mem_byte_en    <= r_reg_port_b[2] ? 8'b1111_0000 : 8'b0000_1111;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_write_DV <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
            r_PC           <= r_PC + 4;
         end
      end
   end
endtask

// MEMGET32 - Read 32-bit word, zero-extended into dest register
// Little-endian: addr[2]=0 → LOW_HALF bits[31:0], addr[2]=1 → HIGH_HALF bits[63:32]
task t_memget32;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr    <= {r_reg_port_b[31:2], 2'b00};
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_read_DV <= 1'b0;
            r_writeback_value <= r_reg_port_b[2] ?
               {32'b0, w_mem_read_data[63:32]} :
               {32'b0, w_mem_read_data[31:0]};
            r_writeback_reg <= r_reg_1;
            r_SM            <= WRITEBACK;
            r_PC            <= r_PC + 4;
         end
      end
   end
endtask

// MEMSET64 - Write 64-bit doubleword to 8-byte aligned address
task t_memset64;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr       <= {r_reg_port_b[31:3], 3'b000};
         r_mem_write_data <= r_reg_port_a;
         r_mem_byte_en    <= 8'b1111_1111;
         r_mem_write_DV   <= 1'b1;
         r_extra_clock    <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_write_DV <= 1'b0;
            r_SM           <= OPCODE_REQUEST;
            r_PC           <= r_PC + 4;
         end
      end
   end
endtask

// MEMGET64 - Read 64-bit doubleword from 8-byte aligned address
task t_memget64;
   begin
      if (r_extra_clock == 0) begin
         r_mem_addr    <= {r_reg_port_b[31:3], 3'b000};
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 1'b1;
      end else begin
         if (w_mem_ready) begin
            r_mem_read_DV     <= 1'b0;
            r_writeback_value <= w_mem_read_data;
            r_writeback_reg   <= r_reg_1;
            r_SM              <= WRITEBACK;
            r_PC              <= r_PC + 4;
         end
      end
   end
endtask


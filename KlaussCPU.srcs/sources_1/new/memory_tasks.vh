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

// MEMGET32 - Read 32-bit word at byte address in r_reg_port_b, zero-extended
// into dest register. Supports any byte alignment of the source address.
//
// The cache returns 8-byte aligned doublewords plus a "next" doubleword for
// reads at the lower half of a cache line. There are four cases depending on
// the byte offset O = addr[2:0] and addr[3]:
//
//   offset 0..4               → all 4 bytes in the first returned doubleword.
//   offset 5..7, addr[3]=0    → spans within same cache line; byte source for
//                                the high bytes is o_mem_read_data_next (cache
//                                lookahead), available in the same cycle.
//   offset 5..7, addr[3]=1    → spans into the NEXT cache line; the cache does
//                                not pre-fetch this. Issue a second read at
//                                (addr & ~7) + 8 to get those bytes.
//
// State machine (r_extra_clock):
//   0: issue first read at addr (cache aligns to 8-byte boundary internally).
//   1: wait first ready. Extract bytes; either writeback (single-read case) or
//      stash dw0 in r_writeback_value and re-assert DV for the second read.
//   2: bubble — lets the cache's stale o_mem_ready (from the first read's
//      READ_CACHE2 → PRE_WAIT path) clear before phase 3 starts polling. The
//      cache's WAIT body in this cycle drives ready low and latches the new
//      DV; without this bubble phase 3 would race-fire on the stale ready=1.
//   3: wait second ready. Combine the saved high bytes of dw0 with the low
//      bytes of dw1, then writeback.
//
// Stash mechanism: r_writeback_value is used as scratch storage for dw0 in
// phases 1-3. Safe because WRITEBACK only fires once we set r_SM<=WRITEBACK
// in the final cycle of phase 1 or phase 3. PC += 4 (1-word RR instruction).
task t_memget32;
   reg [2:0]  offset;
   reg [31:0] result;
   begin
      offset = r_reg_port_b[2:0];

      if (r_extra_clock == 2'd0) begin
         r_mem_addr    <= r_reg_port_b[31:0];
         r_mem_read_DV <= 1'b1;
         r_extra_clock <= 2'd1;
      end else if (r_extra_clock == 2'd1) begin
         if (w_mem_ready) begin
            r_mem_read_DV <= 1'b0;
            if (offset <= 3'd4) begin
               case (offset)
                  3'd0:    result = w_mem_read_data[31:0];
                  3'd1:    result = w_mem_read_data[39:8];
                  3'd2:    result = w_mem_read_data[47:16];
                  3'd3:    result = w_mem_read_data[55:24];
                  3'd4:    result = w_mem_read_data[63:32];
                  default: result = 32'b0;
               endcase
               r_writeback_value <= {32'b0, result};
               r_writeback_reg   <= r_reg_1;
               r_SM              <= WRITEBACK;
               r_PC              <= r_PC + 4;
            end else if (w_mem_next_valid) begin
               // Span within same cache line — use cache lookahead.
               case (offset)
                  3'd5:    result = {w_mem_read_data_next[7:0],  w_mem_read_data[63:40]};
                  3'd6:    result = {w_mem_read_data_next[15:0], w_mem_read_data[63:48]};
                  3'd7:    result = {w_mem_read_data_next[23:0], w_mem_read_data[63:56]};
                  default: result = 32'b0;
               endcase
               r_writeback_value <= {32'b0, result};
               r_writeback_reg   <= r_reg_1;
               r_SM              <= WRITEBACK;
               r_PC              <= r_PC + 4;
            end else begin
               // Cross-cache-line span: stash dw0, issue second read at next dw.
               r_writeback_value <= w_mem_read_data;
               r_mem_addr        <= {r_reg_port_b[31:3], 3'b000} + 32'd8;
               r_mem_read_DV     <= 1'b1;
               r_extra_clock     <= 2'd2;
            end
         end
      end else if (r_extra_clock == 2'd2) begin
         r_extra_clock <= 2'd3;
      end else begin  // r_extra_clock == 2'd3
         if (w_mem_ready) begin
            r_mem_read_DV <= 1'b0;
            // dw1 = w_mem_read_data; dw0 = r_writeback_value (stashed in phase 1).
            case (offset)
               3'd5:    result = {w_mem_read_data[7:0],  r_writeback_value[63:40]};
               3'd6:    result = {w_mem_read_data[15:0], r_writeback_value[63:48]};
               3'd7:    result = {w_mem_read_data[23:0], r_writeback_value[63:56]};
               default: result = 32'b0;
            endcase
            r_writeback_value <= {32'b0, result};
            r_writeback_reg   <= r_reg_1;
            r_SM              <= WRITEBACK;
            r_PC              <= r_PC + 4;
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


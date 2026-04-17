`timescale 1ns / 1ps
// uart_rx_fifo.v — 16-deep, 8-bit synchronous circular FIFO for UART RX buffering.
//
// o_Peek_Byte is combinatorial: it always reflects mem[r_tail] so the CPU can
// capture the byte in the same clock that it asserts i_Read_En (lookahead FIFO).
// The tail pointer advances on the following clock edge.

module uart_rx_fifo #(
    parameter DEPTH = 16
) (
    input        i_Clk,
    input        i_Reset,       // active-high synchronous reset
    input        i_Write_En,    // push: ignored when full
    input  [7:0] i_Write_Byte,
    input        i_Read_En,     // pop:  ignored when empty
    output [7:0] o_Peek_Byte,   // combinatorial head byte (valid when !o_Empty)
    output       o_Empty,
    output       o_Full,
    output [3:0] o_Count
);

    reg [7:0] r_mem  [0:DEPTH-1];
    reg [3:0] r_head;   // write pointer
    reg [3:0] r_tail;   // read pointer
    reg [4:0] r_count;  // 0..DEPTH (5 bits to hold value == DEPTH)

    assign o_Empty     = (r_count == 5'd0);
    assign o_Full      = (r_count == DEPTH[4:0]);
    assign o_Count     = r_count[3:0];
    assign o_Peek_Byte = r_mem[r_tail];  // combinatorial lookahead

    always @(posedge i_Clk) begin
        if (i_Reset) begin
            r_head  <= 4'd0;
            r_tail  <= 4'd0;
            r_count <= 5'd0;
        end else begin
            case ({(i_Write_En & ~o_Full), (i_Read_En & ~o_Empty)})
                2'b10: begin  // write only
                    r_mem[r_head] <= i_Write_Byte;
                    r_head        <= r_head + 1'b1;
                    r_count       <= r_count + 1'b1;
                end
                2'b01: begin  // read only
                    r_tail  <= r_tail + 1'b1;
                    r_count <= r_count - 1'b1;
                end
                2'b11: begin  // simultaneous read + write — count unchanged
                    r_mem[r_head] <= i_Write_Byte;
                    r_head        <= r_head + 1'b1;
                    r_tail        <= r_tail + 1'b1;
                end
                default: ;   // 2'b00: idle
            endcase
        end
    end

endmodule

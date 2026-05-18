`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// ghash — GF(2^128) multiplier for AES-GCM (NIST SP 800-38D §6.3).
//
// Computes Z = X • H in the Galois field GF(2^128) with the irreducible
// polynomial x^128 + x^7 + x^2 + x + 1.
//
// Bit ordering convention
// -----------------------
// NIST GHASH treats a 128-bit block as a polynomial where bit 0 ("leftmost"
// in the byte stream, i.e. the most significant bit of byte 0) is the
// coefficient of x^0.  In this module we represent the value in a Verilog
// 128-bit reg with the NIST/polynomial bit-ordering inversion already
// applied — i.e. bit [127] holds the coefficient of x^0 (MSB of byte 0),
// bit [0] holds the coefficient of x^127 (LSB of byte 15).
//
// The MMIO wrapper (crypto_aes.v) handles the byteswap between the
// little-endian software view and the network-order bit-string view that
// this module operates on, so the contract here is "everything is already
// in network bit order; just multiply."
//
// Reduction polynomial
// --------------------
// R = 11100001 || 0^120 in NIST bit order = `128'hE100..00` in Verilog
// with the high byte holding the coefficients of x^0..x^7.
//
// Latency
// -------
// 128 cycles RUN + 1 cycle DONE = 129 cycles per multiply.  At 100 MHz that's
// ~1.3 µs/block — adequate for 100 Mbps SSH (~12 MB/s peak = ~750k blk/s ≈
// 96M cyc/s of GHASH = 96% CPU on GHASH alone, so this *is* the bottleneck
// of the GCM path; a 4-bit or 8-bit-per-cycle Karatsuba upgrade would be the
// first thing to optimise if profiling justifies it).
//////////////////////////////////////////////////////////////////////////////////

module ghash (
    input              i_clk,
    input              i_rst,
    input              i_start,      // 1-cycle pulse: latch X/H, begin multiply
    input      [127:0] i_X,          // X operand (already in NIST bit order)
    input      [127:0] i_H,          // H operand (already in NIST bit order)
    output reg [127:0] o_Z,          // result Z = X • H
    output reg         o_busy,
    output reg         o_done
);

    // Reduction constant — x^128 + x^7 + x^2 + x + 1, with the implicit
    // x^128 omitted.  In our bit ordering this is the high byte = 0xE1.
    localparam [127:0] R_POLY = 128'h E1000000_00000000_00000000_00000000;

    // Internal state
    reg [127:0] r_Z;          // accumulator (Z[i] in NIST notation)
    reg [127:0] r_V;          // shifted operand (V[i] in NIST notation)
    reg [127:0] r_X_shift;    // X with the current bit-to-process at MSB
    reg [7:0]   r_count;      // 0..127

    localparam ST_IDLE = 2'd0;
    localparam ST_RUN  = 2'd1;
    localparam ST_DONE = 2'd2;
    reg [1:0] r_state;

    always @(posedge i_clk) begin
        if (i_rst) begin
            r_state   <= ST_IDLE;
            r_Z       <= 128'h0;
            r_V       <= 128'h0;
            r_X_shift <= 128'h0;
            r_count   <= 8'h0;
            o_busy    <= 1'b0;
            o_done    <= 1'b0;
            o_Z       <= 128'h0;
        end else begin
            case (r_state)
                ST_IDLE: begin
                    o_busy <= 1'b0;
                    if (i_start) begin
                        r_Z       <= 128'h0;
                        r_V       <= i_H;
                        r_X_shift <= i_X;
                        r_count   <= 8'd0;
                        r_state   <= ST_RUN;
                        o_busy    <= 1'b1;
                        o_done    <= 1'b0;
                    end
                end

                ST_RUN: begin
                    // Process the current X bit (always positioned at MSB of
                    // r_X_shift; equivalent to NIST X[r_count]).
                    if (r_X_shift[127])
                        r_Z <= r_Z ^ r_V;
                    // Bring the next X bit into position 127.
                    r_X_shift <= {r_X_shift[126:0], 1'b0};
                    // Shift V toward LSB; on underflow, fold reduction polynomial in.
                    if (r_V[0])
                        r_V <= (r_V >> 1) ^ R_POLY;
                    else
                        r_V <= r_V >> 1;

                    if (r_count == 8'd127)
                        r_state <= ST_DONE;
                    else
                        r_count <= r_count + 8'd1;
                end

                ST_DONE: begin
                    o_Z     <= r_Z;
                    o_busy  <= 1'b0;
                    o_done  <= 1'b1;
                    r_state <= ST_IDLE;
                end

                default: r_state <= ST_IDLE;
            endcase
        end
    end

endmodule

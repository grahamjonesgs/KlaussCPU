`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// aes_core — AES-128 encrypt/decrypt engine, one round per clock.
//
// Architecture
// ------------
//   - 128-bit data path.
//   - Key schedule expanded on KEY_LOAD into 11 × 128-bit round-key registers,
//     stored in r_round_key[0..10].  Encryption walks 0→10; decryption walks
//     10→0 with the equivalent-inverse-cipher form (inverse MixColumns is
//     applied to round keys 1..9 during use, not at schedule time, which keeps
//     the key schedule identical for encrypt and decrypt).
//   - State is byte-organised in column-major (FIPS-197) order:
//         s[r,c] = state[ 32*c + 8*r + 7 : 32*c + 8*r ]
//     i.e. byte 0 of the 128-bit word is s[0,0], byte 1 is s[1,0], ..., byte 4
//     is s[0,1].  The MMIO-facing data is treated as a flat 128-bit word —
//     bit ordering only matters internally to ShiftRows / MixColumns.
//
// Latency
// -------
//   - KEY_LOAD : ~11 cycles (one expansion step per cycle).
//   - encrypt  : 10 cycles after GO.
//   - decrypt  : 10 cycles after GO.
//
// External handshake
// ------------------
//   - i_go_enc / i_go_dec / i_key_load are 1-cycle pulses from the wrapper.
//   - o_busy goes high the cycle after the pulse and low when the result is
//     latched into o_data_out.  o_done is registered, set in the same cycle
//     that o_busy clears.
//
// Reference: FIPS-197 §5 (encrypt/decrypt rounds) and §5.2 (key expansion).
//////////////////////////////////////////////////////////////////////////////////

module aes_core (
    input  wire         i_clk,
    input  wire         i_rst,

    input  wire         i_key_load,    // 1-cycle pulse: expand i_key into round keys
    input  wire         i_go_enc,      // 1-cycle pulse: encrypt i_data_in
    input  wire         i_go_dec,      // 1-cycle pulse: decrypt i_data_in
    input  wire [127:0] i_key,         // 128-bit key (consumed on i_key_load)
    input  wire [127:0] i_data_in,     // 128-bit plaintext / ciphertext

    output reg          o_busy,
    output reg          o_done,
    output reg  [127:0] o_data_out
);

    // -------------------------------------------------------------------------
    // Round-key storage: 11 × 128-bit (round 0 = original key).
    // -------------------------------------------------------------------------
    reg [127:0] r_round_key [0:10];

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    localparam ST_IDLE  = 3'd0;
    localparam ST_KEXP  = 3'd1;   // key expansion in progress
    localparam ST_ENC   = 3'd2;   // encryption in progress
    localparam ST_DEC   = 3'd3;   // decryption in progress
    localparam ST_DONE  = 3'd4;

    reg [2:0] r_state;
    reg [3:0] r_round;        // 0..10 (encrypt counts up, decrypt counts down)
    reg [127:0] r_state_data; // current 128-bit state

    // -------------------------------------------------------------------------
    // Round-constants for key schedule (Rcon[i] for i = 1..10).  Each entry
    // is the high-byte value placed in the leftmost byte of the round-constant
    // word; the other three bytes are zero.
    // -------------------------------------------------------------------------
    function [7:0] rcon_byte;
        input [3:0] idx;   // 1..10
        case (idx)
            4'd1:  rcon_byte = 8'h01;  4'd2:  rcon_byte = 8'h02;
            4'd3:  rcon_byte = 8'h04;  4'd4:  rcon_byte = 8'h08;
            4'd5:  rcon_byte = 8'h10;  4'd6:  rcon_byte = 8'h20;
            4'd7:  rcon_byte = 8'h40;  4'd8:  rcon_byte = 8'h80;
            4'd9:  rcon_byte = 8'h1b;  4'd10: rcon_byte = 8'h36;
            default: rcon_byte = 8'h00;
        endcase
    endfunction

    // -------------------------------------------------------------------------
    // 16 forward S-boxes (parallel byte lookups for SubBytes during rounds).
    // 4 forward S-boxes (used by the key-schedule SubWord — re-used; the
    // 16 round-S-boxes are unused during ST_KEXP and vice versa, so the
    // synthesiser may share them — we just instantiate both groups for clarity).
    // -------------------------------------------------------------------------
    wire [7:0] w_sbox_in  [0:15];
    wire [7:0] w_sbox_out [0:15];
    wire [7:0] w_isbox_in [0:15];
    wire [7:0] w_isbox_out[0:15];

    genvar gi;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_sboxes
            aes_sbox  u_sb  (.i_byte(w_sbox_in[gi]),  .o_byte(w_sbox_out[gi]));
            aes_isbox u_isb (.i_byte(w_isbox_in[gi]), .o_byte(w_isbox_out[gi]));
        end
    endgenerate

    // Wire current state bytes into the forward S-boxes.
    // Byte layout: state[8*n+7:8*n] = byte n.
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_sb_inputs
            assign w_sbox_in [gi] = r_state_data[8*gi +: 8];
            assign w_isbox_in[gi] = r_state_data[8*gi +: 8];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // SubBytes — apply S-box to every byte (uses w_sbox_out lookups above).
    // -------------------------------------------------------------------------
    wire [127:0] w_after_sub;
    wire [127:0] w_after_isub;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_subbytes
            assign w_after_sub [8*gi +: 8] = w_sbox_out [gi];
            assign w_after_isub[8*gi +: 8] = w_isbox_out[gi];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // ShiftRows — rotate row r left by r positions (column-major state).
    // Row r contains bytes at indices {r, r+4, r+8, r+12}.
    //
    // For forward ShiftRows, byte at (r,c) comes from (r, (c+r) mod 4).
    // -------------------------------------------------------------------------
    function [127:0] shift_rows;
        input [127:0] s;
        integer r, c;
        reg [127:0] out;
        begin
            for (r = 0; r < 4; r = r + 1)
                for (c = 0; c < 4; c = c + 1)
                    out[8*(4*c + r) +: 8] = s[8*(4*((c + r) % 4) + r) +: 8];
            shift_rows = out;
        end
    endfunction

    function [127:0] inv_shift_rows;
        input [127:0] s;
        integer r, c;
        reg [127:0] out;
        begin
            for (r = 0; r < 4; r = r + 1)
                for (c = 0; c < 4; c = c + 1)
                    out[8*(4*c + r) +: 8] = s[8*(4*((c - r + 4) % 4) + r) +: 8];
            inv_shift_rows = out;
        end
    endfunction

    // -------------------------------------------------------------------------
    // MixColumns / InvMixColumns — GF(2^8) maths on each 32-bit column.
    //
    // Forward:   b0 = 2a0 ^ 3a1 ^  a2 ^  a3
    //            b1 =  a0 ^ 2a1 ^ 3a2 ^  a3
    //            b2 =  a0 ^  a1 ^ 2a2 ^ 3a3
    //            b3 = 3a0 ^  a1 ^  a2 ^ 2a3
    // Multiplication by 2 is "xtime"; ×3 = ×2 ⊕ ×1.
    //
    // Inverse:   uses constants {0e, 0b, 0d, 09} per FIPS-197 §5.3.3.
    // -------------------------------------------------------------------------
    function [7:0] xtime;
        input [7:0] b;
        begin
            xtime = b[7] ? ((b << 1) ^ 8'h1b) : (b << 1);
        end
    endfunction
    function [7:0] mul2; input [7:0] b; begin mul2 = xtime(b); end endfunction
    function [7:0] mul3; input [7:0] b; begin mul3 = xtime(b) ^ b; end endfunction
    function [7:0] mul4; input [7:0] b; begin mul4 = xtime(xtime(b)); end endfunction
    function [7:0] mul8; input [7:0] b; begin mul8 = xtime(xtime(xtime(b))); end endfunction
    function [7:0] mul9; input [7:0] b; begin mul9 = mul8(b) ^ b; end endfunction
    function [7:0] mulB; input [7:0] b; begin mulB = mul8(b) ^ mul2(b) ^ b; end endfunction
    function [7:0] mulD; input [7:0] b; begin mulD = mul8(b) ^ mul4(b) ^ b; end endfunction
    function [7:0] mulE; input [7:0] b; begin mulE = mul8(b) ^ mul4(b) ^ mul2(b); end endfunction

    function [31:0] mix_col;
        input [31:0] col;
        reg [7:0] a0, a1, a2, a3;
        begin
            a0 = col[7:0]; a1 = col[15:8]; a2 = col[23:16]; a3 = col[31:24];
            mix_col = {
                mul3(a0) ^ a1 ^ a2 ^ mul2(a3),
                a0 ^ a1 ^ mul2(a2) ^ mul3(a3),
                a0 ^ mul2(a1) ^ mul3(a2) ^ a3,
                mul2(a0) ^ mul3(a1) ^ a2 ^ a3
            };
        end
    endfunction

    function [31:0] inv_mix_col;
        input [31:0] col;
        reg [7:0] a0, a1, a2, a3;
        begin
            a0 = col[7:0]; a1 = col[15:8]; a2 = col[23:16]; a3 = col[31:24];
            inv_mix_col = {
                mulB(a0) ^ mulD(a1) ^ mul9(a2) ^ mulE(a3),
                mulD(a0) ^ mul9(a1) ^ mulE(a2) ^ mulB(a3),
                mul9(a0) ^ mulE(a1) ^ mulB(a2) ^ mulD(a3),
                mulE(a0) ^ mulB(a1) ^ mulD(a2) ^ mul9(a3)
            };
        end
    endfunction

    function [127:0] mix_columns;
        input [127:0] s;
        begin
            mix_columns = { mix_col(s[127:96]), mix_col(s[95:64]),
                            mix_col(s[63:32]),  mix_col(s[31:0]) };
        end
    endfunction
    function [127:0] inv_mix_columns;
        input [127:0] s;
        begin
            inv_mix_columns = { inv_mix_col(s[127:96]), inv_mix_col(s[95:64]),
                                inv_mix_col(s[63:32]),  inv_mix_col(s[31:0]) };
        end
    endfunction

    // -------------------------------------------------------------------------
    // SubWord — apply S-box to all 4 bytes of a 32-bit word (key schedule use).
    // RotWord — rotate the 4 bytes left by one.
    // -------------------------------------------------------------------------
    wire [7:0] w_ks_sb_in  [0:3];
    wire [7:0] w_ks_sb_out [0:3];
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_ks_sboxes
            aes_sbox u_ksb (.i_byte(w_ks_sb_in[gi]), .o_byte(w_ks_sb_out[gi]));
        end
    endgenerate

    reg [31:0] r_ks_word;       // word being processed by key schedule
    reg [3:0]  r_ks_idx;        // round-key index being computed (1..10)
    assign w_ks_sb_in[0] = r_ks_word[15:8];   // RotWord: lo byte was byte[1]
    assign w_ks_sb_in[1] = r_ks_word[23:16];
    assign w_ks_sb_in[2] = r_ks_word[31:24];
    assign w_ks_sb_in[3] = r_ks_word[7:0];

    wire [31:0] w_ks_subrot = { w_ks_sb_out[3], w_ks_sb_out[2],
                                w_ks_sb_out[1], w_ks_sb_out[0] };
    wire [31:0] w_ks_rcon   = { 24'h0, rcon_byte(r_ks_idx) };
    wire [31:0] w_ks_transformed = w_ks_subrot ^ w_ks_rcon;

    // Previous round key, broken into 4 words.
    wire [31:0] w_prev_w0 = r_round_key[r_ks_idx - 1][31:0];
    wire [31:0] w_prev_w1 = r_round_key[r_ks_idx - 1][63:32];
    wire [31:0] w_prev_w2 = r_round_key[r_ks_idx - 1][95:64];
    wire [31:0] w_prev_w3 = r_round_key[r_ks_idx - 1][127:96];

    wire [31:0] w_new_w0 = w_prev_w0 ^ w_ks_transformed;
    wire [31:0] w_new_w1 = w_prev_w1 ^ w_new_w0;
    wire [31:0] w_new_w2 = w_prev_w2 ^ w_new_w1;
    wire [31:0] w_new_w3 = w_prev_w3 ^ w_new_w2;

    // -------------------------------------------------------------------------
    // Round-data computation.
    // -------------------------------------------------------------------------
    wire [127:0] w_enc_full   = mix_columns(shift_rows(w_after_sub)) ^ r_round_key[r_round + 1];
    wire [127:0] w_enc_final  = shift_rows(w_after_sub) ^ r_round_key[10];
    wire [127:0] w_dec_full   = inv_mix_columns(inv_shift_rows(w_after_isub) ^ r_round_key[r_round - 1]);
    wire [127:0] w_dec_final  = inv_shift_rows(w_after_isub) ^ r_round_key[0];

    // -------------------------------------------------------------------------
    // Main state machine
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge i_clk) begin
        if (i_rst) begin
            r_state    <= ST_IDLE;
            r_round    <= 4'd0;
            r_ks_idx   <= 4'd0;
            r_ks_word  <= 32'h0;
            o_busy     <= 1'b0;
            o_done     <= 1'b0;
            o_data_out <= 128'h0;
            r_state_data <= 128'h0;
            for (i = 0; i < 11; i = i + 1)
                r_round_key[i] <= 128'h0;
        end else begin
            case (r_state)
                ST_IDLE: begin
                    o_busy <= 1'b0;
                    if (i_key_load) begin
                        // Round key 0 = the raw key.
                        r_round_key[0] <= i_key;
                        r_ks_idx       <= 4'd1;
                        r_ks_word      <= i_key[127:96];   // w3 of prev key
                        r_state        <= ST_KEXP;
                        o_busy         <= 1'b1;
                        o_done         <= 1'b0;
                    end else if (i_go_enc) begin
                        r_state_data <= i_data_in ^ r_round_key[0];   // initial AddRoundKey
                        r_round      <= 4'd0;
                        r_state      <= ST_ENC;
                        o_busy       <= 1'b1;
                        o_done       <= 1'b0;
                    end else if (i_go_dec) begin
                        r_state_data <= i_data_in ^ r_round_key[10];  // initial AddRoundKey (final round key)
                        r_round      <= 4'd10;
                        r_state      <= ST_DEC;
                        o_busy       <= 1'b1;
                        o_done       <= 1'b0;
                    end
                end

                ST_KEXP: begin
                    r_round_key[r_ks_idx] <= { w_new_w3, w_new_w2, w_new_w1, w_new_w0 };
                    if (r_ks_idx == 4'd10) begin
                        r_state <= ST_DONE;
                    end else begin
                        r_ks_idx  <= r_ks_idx + 4'd1;
                        r_ks_word <= w_new_w3;   // new w3 becomes input to next iteration's SubWord
                    end
                end

                ST_ENC: begin
                    if (r_round == 4'd9) begin
                        // Final round: no MixColumns.
                        r_state_data <= w_enc_final;
                        r_state      <= ST_DONE;
                    end else begin
                        r_state_data <= w_enc_full;
                        r_round      <= r_round + 4'd1;
                    end
                end

                ST_DEC: begin
                    if (r_round == 4'd1) begin
                        // Final round: no inv MixColumns.
                        r_state_data <= w_dec_final;
                        r_state      <= ST_DONE;
                    end else begin
                        r_state_data <= w_dec_full;
                        r_round      <= r_round - 4'd1;
                    end
                end

                ST_DONE: begin
                    o_data_out <= r_state_data;
                    o_busy     <= 1'b0;
                    o_done     <= 1'b1;
                    r_state    <= ST_IDLE;
                end

                default: r_state <= ST_IDLE;
            endcase
        end
    end

endmodule

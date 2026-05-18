`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// sha256_core — SHA-256 compression engine, one round per clock.
//
// Architecture
// ------------
//   - 64 rounds per 512-bit message block (1 cycle each = 64 cycles per block).
//   - 16-deep sliding window of W values (the message schedule).  At round t
//     the window holds W[t..t+15]; window[0] is consumed this round and the
//     newly-computed W[t+16] is shifted in at window[15].  The same recurrence
//     applies uniformly for all rounds 0..63 (rounds 0..15 happen to load
//     directly from the message, but the SAME recurrence still produces a
//     useful value at window[15] for round t+16).
//   - 8 working variables a..h initialised from H[0..7] at the start of each
//     block and added back into H at the end (FIPS 180-4 §6.2.2).
//
// External interface
// ------------------
//   i_init  — 1-cycle pulse: reset H to the standard IV (FIPS 180-4 §5.3.3).
//   i_start — 1-cycle pulse: compress one 512-bit block (i_block).  Caller
//             must do padding + length encoding in software per FIPS 180-4
//             §5.1.1 — the engine just does the compression.
//   i_block — 512-bit message block.  Each 32-bit word M[i] is laid out at
//             bits[32*i + 31 : 32*i] in big-endian-message order (the MMIO
//             wrapper handles the byteswap from CPU little-endian).
//   o_digest — 256-bit current hash: { H7, H6, H5, H4, H3, H2, H1, H0 }
//              with H0 at bits[31:0].  The wrapper presents this as four
//              64-bit MMIO read ports, byteswapped to match standard SHA-256
//              byte order.
//
// Reference: FIPS 180-4 §6.2 (SHA-256 algorithm).
//////////////////////////////////////////////////////////////////////////////////

module sha256_core (
    input              i_clk,
    input              i_rst,

    input              i_init,
    input              i_start,
    input      [511:0] i_block,

    // HMAC-support: load H[0..7] directly from external state.
    // Used by the HMAC wrapper to restore precomputed inner/outer midstates
    // before continuing a streamed hash.  Honoured only in ST_IDLE.
    input              i_h_load,
    input      [255:0] i_h_in,        // {H7, H6, H5, H4, H3, H2, H1, H0}

    output reg         o_busy,
    output reg         o_done,
    output     [255:0] o_digest
);

    // -------------------------------------------------------------------------
    // FIPS 180-4 §5.3.3 — initial hash value.
    // -------------------------------------------------------------------------
    localparam [31:0] IV0 = 32'h6a09e667;
    localparam [31:0] IV1 = 32'hbb67ae85;
    localparam [31:0] IV2 = 32'h3c6ef372;
    localparam [31:0] IV3 = 32'ha54ff53a;
    localparam [31:0] IV4 = 32'h510e527f;
    localparam [31:0] IV5 = 32'h9b05688c;
    localparam [31:0] IV6 = 32'h1f83d9ab;
    localparam [31:0] IV7 = 32'h5be0cd19;

    // -------------------------------------------------------------------------
    // FIPS 180-4 §4.2.2 — round constants K[0..63].
    // -------------------------------------------------------------------------
    function [31:0] K_const;
        input [5:0] idx;
        case (idx)
            6'd00: K_const = 32'h428a2f98;  6'd01: K_const = 32'h71374491;
            6'd02: K_const = 32'hb5c0fbcf;  6'd03: K_const = 32'he9b5dba5;
            6'd04: K_const = 32'h3956c25b;  6'd05: K_const = 32'h59f111f1;
            6'd06: K_const = 32'h923f82a4;  6'd07: K_const = 32'hab1c5ed5;
            6'd08: K_const = 32'hd807aa98;  6'd09: K_const = 32'h12835b01;
            6'd10: K_const = 32'h243185be;  6'd11: K_const = 32'h550c7dc3;
            6'd12: K_const = 32'h72be5d74;  6'd13: K_const = 32'h80deb1fe;
            6'd14: K_const = 32'h9bdc06a7;  6'd15: K_const = 32'hc19bf174;
            6'd16: K_const = 32'he49b69c1;  6'd17: K_const = 32'hefbe4786;
            6'd18: K_const = 32'h0fc19dc6;  6'd19: K_const = 32'h240ca1cc;
            6'd20: K_const = 32'h2de92c6f;  6'd21: K_const = 32'h4a7484aa;
            6'd22: K_const = 32'h5cb0a9dc;  6'd23: K_const = 32'h76f988da;
            6'd24: K_const = 32'h983e5152;  6'd25: K_const = 32'ha831c66d;
            6'd26: K_const = 32'hb00327c8;  6'd27: K_const = 32'hbf597fc7;
            6'd28: K_const = 32'hc6e00bf3;  6'd29: K_const = 32'hd5a79147;
            6'd30: K_const = 32'h06ca6351;  6'd31: K_const = 32'h14292967;
            6'd32: K_const = 32'h27b70a85;  6'd33: K_const = 32'h2e1b2138;
            6'd34: K_const = 32'h4d2c6dfc;  6'd35: K_const = 32'h53380d13;
            6'd36: K_const = 32'h650a7354;  6'd37: K_const = 32'h766a0abb;
            6'd38: K_const = 32'h81c2c92e;  6'd39: K_const = 32'h92722c85;
            6'd40: K_const = 32'ha2bfe8a1;  6'd41: K_const = 32'ha81a664b;
            6'd42: K_const = 32'hc24b8b70;  6'd43: K_const = 32'hc76c51a3;
            6'd44: K_const = 32'hd192e819;  6'd45: K_const = 32'hd6990624;
            6'd46: K_const = 32'hf40e3585;  6'd47: K_const = 32'h106aa070;
            6'd48: K_const = 32'h19a4c116;  6'd49: K_const = 32'h1e376c08;
            6'd50: K_const = 32'h2748774c;  6'd51: K_const = 32'h34b0bcb5;
            6'd52: K_const = 32'h391c0cb3;  6'd53: K_const = 32'h4ed8aa4a;
            6'd54: K_const = 32'h5b9cca4f;  6'd55: K_const = 32'h682e6ff3;
            6'd56: K_const = 32'h748f82ee;  6'd57: K_const = 32'h78a5636f;
            6'd58: K_const = 32'h84c87814;  6'd59: K_const = 32'h8cc70208;
            6'd60: K_const = 32'h90befffa;  6'd61: K_const = 32'ha4506ceb;
            6'd62: K_const = 32'hbef9a3f7;  6'd63: K_const = 32'hc67178f2;
        endcase
    endfunction

    // -------------------------------------------------------------------------
    // FIPS 180-4 §4.1.2 — logical functions (Ch, Maj, Σ0/1, σ0/1).
    // -------------------------------------------------------------------------
    function [31:0] CH;       // Ch(x,y,z) = (x AND y) XOR (NOT x AND z)
        input [31:0] x, y, z;
        CH = (x & y) ^ ((~x) & z);
    endfunction

    function [31:0] MAJ;      // Maj(x,y,z) = (x AND y) XOR (x AND z) XOR (y AND z)
        input [31:0] x, y, z;
        MAJ = (x & y) ^ (x & z) ^ (y & z);
    endfunction

    function [31:0] BSIG0;    // Σ0(x) = ROTR(x,2) ^ ROTR(x,13) ^ ROTR(x,22)
        input [31:0] x;
        BSIG0 = {x[1:0],  x[31:2]}
              ^ {x[12:0], x[31:13]}
              ^ {x[21:0], x[31:22]};
    endfunction

    function [31:0] BSIG1;    // Σ1(x) = ROTR(x,6) ^ ROTR(x,11) ^ ROTR(x,25)
        input [31:0] x;
        BSIG1 = {x[5:0],  x[31:6]}
              ^ {x[10:0], x[31:11]}
              ^ {x[24:0], x[31:25]};
    endfunction

    function [31:0] SSIG0;    // σ0(x) = ROTR(x,7) ^ ROTR(x,18) ^ SHR(x,3)
        input [31:0] x;
        SSIG0 = {x[6:0],  x[31:7]}
              ^ {x[17:0], x[31:18]}
              ^ (x >> 3);
    endfunction

    function [31:0] SSIG1;    // σ1(x) = ROTR(x,17) ^ ROTR(x,19) ^ SHR(x,10)
        input [31:0] x;
        SSIG1 = {x[16:0], x[31:17]}
              ^ {x[18:0], x[31:19]}
              ^ (x >> 10);
    endfunction

    // -------------------------------------------------------------------------
    // State storage
    // -------------------------------------------------------------------------
    reg [31:0] H [0:7];        // running hash state
    reg [31:0] a, b, c, d, e, f, g, h;   // working vars during a block
    reg [31:0] W [0:15];       // sliding message-schedule window
    reg [6:0]  r_round;        // 0..63 during compression
    reg [1:0]  r_state;

    localparam ST_IDLE  = 2'd0;
    localparam ST_RUN   = 2'd1;
    localparam ST_FINAL = 2'd2;

    // -------------------------------------------------------------------------
    // Per-round combinational paths
    //   T1 = h + Σ1(e) + Ch(e,f,g) + K[t] + W[t]
    //   T2 = Σ0(a) + Maj(a,b,c)
    //   new W = σ1(W[14]) + W[9] + σ0(W[1]) + W[0]
    // window[0] holds W[t] for this round; window[15] will hold W[t+16] next.
    // -------------------------------------------------------------------------
    wire [31:0] w_t1   = h + BSIG1(e) + CH(e, f, g) + K_const(r_round[5:0]) + W[0];
    wire [31:0] w_t2   = BSIG0(a) + MAJ(a, b, c);
    wire [31:0] w_newW = SSIG1(W[14]) + W[9] + SSIG0(W[1]) + W[0];

    assign o_digest = { H[7], H[6], H[5], H[4], H[3], H[2], H[1], H[0] };

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge i_clk) begin
        if (i_rst) begin
            r_state <= ST_IDLE;
            r_round <= 7'd0;
            o_busy  <= 1'b0;
            o_done  <= 1'b0;
            H[0] <= IV0; H[1] <= IV1; H[2] <= IV2; H[3] <= IV3;
            H[4] <= IV4; H[5] <= IV5; H[6] <= IV6; H[7] <= IV7;
            a <= 32'h0; b <= 32'h0; c <= 32'h0; d <= 32'h0;
            e <= 32'h0; f <= 32'h0; g <= 32'h0; h <= 32'h0;
            for (i = 0; i < 16; i = i + 1) W[i] <= 32'h0;
        end else begin
            case (r_state)
                ST_IDLE: begin
                    o_busy <= 1'b0;
                    if (i_init) begin
                        H[0] <= IV0; H[1] <= IV1; H[2] <= IV2; H[3] <= IV3;
                        H[4] <= IV4; H[5] <= IV5; H[6] <= IV6; H[7] <= IV7;
                        o_done <= 1'b0;
                    end else if (i_h_load) begin
                        // External H restore — used by HMAC wrapper to seed
                        // the running hash with a precomputed midstate.
                        for (i = 0; i < 8; i = i + 1)
                            H[i] <= i_h_in[32*i +: 32];
                        o_done <= 1'b0;
                    end else if (i_start) begin
                        // Load working variables from running hash state.
                        a <= H[0]; b <= H[1]; c <= H[2]; d <= H[3];
                        e <= H[4]; f <= H[5]; g <= H[6]; h <= H[7];
                        // Load message schedule window from input block.
                        // M[0] is at i_block[31:0], M[15] is at i_block[511:480].
                        for (i = 0; i < 16; i = i + 1)
                            W[i] <= i_block[32*i +: 32];
                        r_round <= 7'd0;
                        r_state <= ST_RUN;
                        o_busy  <= 1'b1;
                        o_done  <= 1'b0;
                    end
                end

                ST_RUN: begin
                    // Working-variable rotation (FIPS 180-4 §6.2.2 step 3).
                    h <= g;
                    g <= f;
                    f <= e;
                    e <= d + w_t1;
                    d <= c;
                    c <= b;
                    b <= a;
                    a <= w_t1 + w_t2;
                    // Shift message-schedule window: W[0..14] <= W[1..15];
                    // W[15] <= computed σ1(W[14]) + W[9] + σ0(W[1]) + W[0].
                    for (i = 0; i < 15; i = i + 1) W[i] <= W[i + 1];
                    W[15] <= w_newW;
                    if (r_round == 7'd63) begin
                        r_state <= ST_FINAL;
                    end else begin
                        r_round <= r_round + 7'd1;
                    end
                end

                ST_FINAL: begin
                    // Add working variables back into running hash state.
                    H[0] <= H[0] + a;
                    H[1] <= H[1] + b;
                    H[2] <= H[2] + c;
                    H[3] <= H[3] + d;
                    H[4] <= H[4] + e;
                    H[5] <= H[5] + f;
                    H[6] <= H[6] + g;
                    H[7] <= H[7] + h;
                    r_state <= ST_IDLE;
                    o_busy  <= 1'b0;
                    o_done  <= 1'b1;
                end

                default: r_state <= ST_IDLE;
            endcase
        end
    end

endmodule

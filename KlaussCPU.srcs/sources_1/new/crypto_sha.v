`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// crypto_sha — MMIO wrapper around sha256_core.
//
// Device base 0xF00B_0000 (MMIO device id 0x00B). See CRYPTO_PLAN.md §6 for
// the design rationale.  Implements the *bare* SHA-256 compression engine —
// padding and length encoding are done in software.  The HMAC wrapper (item 5
// in CRYPTO_PLAN.md) will reuse this device window starting at offset 0x080.
//
// Register layout (offsets within the device window):
//   0x000  SHA_CTRL     W  [0] INIT  (reset H to FIPS-180-4 IV, self-clearing)
//                          [1] START (compress current block, self-clearing)
//   0x008  SHA_STATUS   R  [0] BUSY  (compression in progress)
//                          [1] DONE  (sticky; cleared by next INIT/START)
//   0x010..0x048  SHA_BLOCK0..7  RW  Eight 64-bit slots holding the 512-bit
//                                     message block.  Each slot is two 32-bit
//                                     message words (low half = M[2i], high
//                                     half = M[2i+1]), each byteswapped to
//                                     match SHA-256's big-endian word order.
//   0x050..0x068  SHA_DIGEST0..3 R   Four 64-bit slots holding the 256-bit
//                                     digest H[0..7].  Layout matches
//                                     BLOCK (low = H[2i], high = H[2i+1],
//                                     each byteswapped).
//
// Byte-order rationale
// --------------------
// SHA-256 is defined on a big-endian byte stream (FIPS 180-4 §3.1.2).  The CPU
// is little-endian.  The wrapper applies a per-32-bit-word byteswap on every
// MMIO access so the contract for software is "do `((uint64_t*)dst)[i] =
// REG_SHA_BLOCK_i` to feed message bytes in their natural memory order, and
// `((uint64_t*)out)[i] = REG_SHA_DIGEST_i` to read the digest in its natural
// byte order."  No software byteswapping required.
//
// Throughput
// ----------
// Bare compression: 64 cycles/block + ~3 cycles FSM overhead ≈ 67 cycles/block.
// Software overhead per block: 8 × 64-bit MMIO writes (8 cycles) + 2 status
// reads (2 cycles each) ≈ ~12 cycles.  Total ≈ ~80 cycles per 64 B of message
// = 1.25 cycle/byte = ~80 MB/s at 100 MHz.  Comfortable headroom for SSH at
// 100 Mbps line rate.
//////////////////////////////////////////////////////////////////////////////////

module crypto_sha (
    input             i_Clk,
    input             i_Rst_L,

    // MMIO interface (offsets within device window, addr[15:0]).
    input             i_mmio_write_DV,
    input             i_mmio_read_DV,
    input      [15:0] i_mmio_addr,
    input      [63:0] i_mmio_write_data,
    input      [ 7:0] i_mmio_byte_en,
    output reg [63:0] o_mmio_read_data,
    output            o_mmio_ready
);

    // -------------------------------------------------------------------------
    // Register offsets — bare SHA-256
    // -------------------------------------------------------------------------
    localparam OFF_CTRL    = 16'h0000;
    localparam OFF_STATUS  = 16'h0008;
    localparam OFF_BLOCK0  = 16'h0010;
    localparam OFF_BLOCK7  = 16'h0048;
    localparam OFF_DIGEST0 = 16'h0050;
    localparam OFF_DIGEST3 = 16'h0068;

    // -------------------------------------------------------------------------
    // Register offsets — HMAC wrapper (see CRYPTO_PLAN.md §8)
    //
    // HMAC-SHA-256 caches two midstates derived from the key:
    //   inner_state = SHA256_compress(IV, K' ⊕ ipad)
    //   outer_state = SHA256_compress(IV, K' ⊕ opad)
    // where K' = K (32 bytes) || 0 (32 bytes), ipad = 0x36×64, opad = 0x5c×64.
    //
    // Per-MAC flow:
    //   1. HMAC_CTRL.START  — H := inner_state (single i_h_load pulse to core)
    //   2. Software streams message blocks normally via SHA_BLOCK + START.
    //      Padding must account for 64 prefixed ipad bytes in the length field.
    //   3. Software reads SHA_DIGEST as the inner digest.
    //   4. HMAC_CTRL.FINAL  — H := outer_state
    //   5. Software writes (inner_digest ‖ padding) into BLOCK; one SHA_START
    //      produces the HMAC tag in SHA_DIGEST.
    // -------------------------------------------------------------------------
    localparam OFF_HMAC_CTRL   = 16'h0080;
    localparam OFF_HMAC_STATUS = 16'h0088;
    localparam OFF_HMAC_KEY0   = 16'h0090;
    localparam OFF_HMAC_KEY1   = 16'h0098;
    localparam OFF_HMAC_KEY2   = 16'h00A0;
    localparam OFF_HMAC_KEY3   = 16'h00A8;

    assign o_mmio_ready = 1'b1;
    wire byte_en_unused = |i_mmio_byte_en;  // software always uses 64-bit MMIO accesses

    // -------------------------------------------------------------------------
    // Block-input storage (raw MMIO little-endian view).
    // BLOCK[i] at offset (0x10 + 8*i), i in 0..7.
    // -------------------------------------------------------------------------
    reg [63:0] r_block_mmio [0:7];

    // Per-32-bit-word byteswap helper.
    function [31:0] bswap32;
        input [31:0] v;
        bswap32 = { v[7:0], v[15:8], v[23:16], v[31:24] };
    endfunction

    // Build the 512-bit input to the core: low 32 bits of BLOCKi → M[2i],
    // high 32 bits → M[2i+1], both byteswapped to big-endian word order.
    wire [511:0] w_block_core;
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : g_block_bswap
            assign w_block_core[64*gi      +: 32] = bswap32(r_block_mmio[gi][31:0]);
            assign w_block_core[64*gi + 32 +: 32] = bswap32(r_block_mmio[gi][63:32]);
        end
    endgenerate

    // -------------------------------------------------------------------------
    // HMAC state cache + FSM declarations
    // -------------------------------------------------------------------------
    reg [255:0] r_hmac_key;
    reg [255:0] r_inner_state;       // owned by HMAC FSM block (driver-1)
    reg [255:0] r_outer_state;       // owned by HMAC FSM block (driver-1)
    reg         r_hmac_key_valid;    // owned by HMAC FSM block (driver-1)
    reg [2:0]   r_hmac_fsm;
    reg         r_hmac_use_outer_block;
    reg         r_hmac_init_pulse;
    reg         r_hmac_start_pulse;
    reg         r_hmac_kl_trigger;       // 1-cycle pulse: software wrote HMAC_CTRL.KEY_LOAD
    reg         r_hmac_keyzero_pulse;    // 1-cycle pulse: software wrote HMAC_CTRL.KEY_ZERO
    reg         r_h_load_pulse;          // 1-cycle pulse to core's i_h_load
    reg         r_h_load_sel;            // 0 = inner_state, 1 = outer_state

    localparam HMAC_IDLE    = 3'd0;
    localparam HMAC_START_I = 3'd1;  // INIT sent last cycle → drive START with inner block
    localparam HMAC_WAIT_I  = 3'd2;  // SHA running inner pass; wait for o_done
    localparam HMAC_INIT_O  = 3'd3;  // re-INIT before outer pass
    localparam HMAC_START_O = 3'd4;
    localparam HMAC_WAIT_O  = 3'd5;

    wire w_hmac_busy = (r_hmac_fsm != HMAC_IDLE);

    // Build K' ⊕ ipad / opad as 512-bit blocks (each 64 bytes; first 32 bytes
    // are key XOR pad, next 32 are pad alone, since the key is 32 bytes <
    // block size).  bswap32 puts the key bytes into M[i]'s big-endian
    // expected layout so the SHA core sees the standard FIPS-180-4 message
    // word ordering after the wrapper's existing input byteswap.
    function [511:0] hmac_block;
        input [255:0] key;
        input [7:0]   pad;
        reg [31:0] pad32;
        integer kk;
        begin
            pad32 = {4{pad}};
            for (kk = 0; kk < 8; kk = kk + 1)
                hmac_block[32*kk +: 32] = bswap32(key[32*kk +: 32]) ^ pad32;
            for (kk = 8; kk < 16; kk = kk + 1)
                hmac_block[32*kk +: 32] = pad32;
        end
    endfunction

    wire [511:0] w_inner_block_hmac = hmac_block(r_hmac_key, 8'h36);
    wire [511:0] w_outer_block_hmac = hmac_block(r_hmac_key, 8'h5c);

    // -------------------------------------------------------------------------
    // Core instantiation — i_init/i_start/i_block are muxed between the
    // software MMIO path and the HMAC FSM path; i_h_load is HMAC-FSM-only.
    // -------------------------------------------------------------------------
    reg          r_init_pulse;
    reg          r_start_pulse;
    wire         w_core_busy;
    wire         w_core_done;
    wire [255:0] w_digest_core;

    wire         w_sha_init      = r_init_pulse  | r_hmac_init_pulse;
    wire         w_sha_start     = r_start_pulse | r_hmac_start_pulse;
    wire [511:0] w_sha_block_pre = w_hmac_busy
                                ? (r_hmac_use_outer_block ? w_outer_block_hmac
                                                          : w_inner_block_hmac)
                                : w_block_core;
    wire [255:0] w_sha_h_in      = r_h_load_sel ? r_outer_state : r_inner_state;

    // 512-bit pipeline register on the SHA block input (CRYPTO_PLAN.md §10a).
    // The combinational network selecting between three 512-bit sources
    // (software-written block via byteswap, inner-key-XOR-ipad block, outer-
    // key-XOR-opad block) fans out to 512 destinations inside u_sha.  This
    // was one of the wide-combinational hotspots the placer struggled with.
    //
    // Software / HMAC FSM contracts unchanged: i_start always arrives at
    // least one cycle after the block input has settled (the HMAC FSM goes
    // INIT → START with one transition in between; software writes blocks
    // then writes CTRL.START on a separate MMIO cycle).  The pipeline reg
    // has the correct value latched in time.
    reg [511:0] r_sha_block_pipe;
    always @(posedge i_Clk) r_sha_block_pipe <= w_sha_block_pre;

    (* KEEP_HIERARCHY = "yes" *)
    sha256_core u_sha (
        .i_clk    (i_Clk),
        .i_rst    (~i_Rst_L),
        .i_init   (w_sha_init),
        .i_start  (w_sha_start),
        .i_block  (r_sha_block_pipe),
        .i_h_load (r_h_load_pulse),
        .i_h_in   (w_sha_h_in),
        .o_busy   (w_core_busy),
        .o_done   (w_core_done),
        .o_digest (w_digest_core)
    );

    // Sticky DONE flag — set when core asserts done, cleared on next INIT/START.
    // Gated by !w_hmac_busy so HMAC-FSM-driven SHA passes don't visibly mutate
    // the software-facing DONE bit.
    reg r_done_latch;

    // -------------------------------------------------------------------------
    // Write path — software MMIO interface to SHA registers AND HMAC registers.
    //
    // HMAC_CTRL writes set 1-cycle pulses (KL_trigger / h_load_pulse) that are
    // consumed by the HMAC FSM (separate always block below).
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_init_pulse         <= 1'b0;
            r_start_pulse        <= 1'b0;
            r_done_latch         <= 1'b0;
            r_hmac_kl_trigger    <= 1'b0;
            r_hmac_keyzero_pulse <= 1'b0;
            r_h_load_pulse       <= 1'b0;
            r_h_load_sel         <= 1'b0;
            r_hmac_key           <= 256'h0;
            for (i = 0; i < 8; i = i + 1) r_block_mmio[i] <= 64'h0;
        end else begin
            // Default: deassert single-cycle pulses each cycle.
            r_init_pulse         <= 1'b0;
            r_start_pulse        <= 1'b0;
            r_hmac_kl_trigger    <= 1'b0;
            r_hmac_keyzero_pulse <= 1'b0;
            r_h_load_pulse       <= 1'b0;

            // Sticky DONE — only reflect software-driven SHA ops, not the
            // HMAC FSM's internal midstate-computation passes.
            if (w_core_done && !w_hmac_busy)
                r_done_latch <= 1'b1;

            if (i_mmio_write_DV) begin
                case (i_mmio_addr)
                    OFF_CTRL: begin
                        if (i_mmio_write_data[0]) begin
                            r_init_pulse <= 1'b1;
                            r_done_latch <= 1'b0;
                        end
                        if (i_mmio_write_data[1]) begin
                            r_start_pulse <= 1'b1;
                            r_done_latch  <= 1'b0;
                        end
                    end
                    OFF_BLOCK0 + 16'h00: r_block_mmio[0] <= i_mmio_write_data;
                    OFF_BLOCK0 + 16'h08: r_block_mmio[1] <= i_mmio_write_data;
                    OFF_BLOCK0 + 16'h10: r_block_mmio[2] <= i_mmio_write_data;
                    OFF_BLOCK0 + 16'h18: r_block_mmio[3] <= i_mmio_write_data;
                    OFF_BLOCK0 + 16'h20: r_block_mmio[4] <= i_mmio_write_data;
                    OFF_BLOCK0 + 16'h28: r_block_mmio[5] <= i_mmio_write_data;
                    OFF_BLOCK0 + 16'h30: r_block_mmio[6] <= i_mmio_write_data;
                    OFF_BLOCK0 + 16'h38: r_block_mmio[7] <= i_mmio_write_data;
                    OFF_HMAC_CTRL: begin
                        // [0] KEY_LOAD: kick the HMAC FSM to compute midstates.
                        //               Ignored if the FSM is already running.
                        // [1] START   : H ← inner_state (single i_h_load pulse).
                        // [2] FINAL   : H ← outer_state (single i_h_load pulse).
                        // [3] KEY_ZERO: wipe key here; route a pulse to the
                        //               HMAC FSM block so it clears the
                        //               midstates + valid flag (it owns them).
                        if (i_mmio_write_data[3]) begin
                            r_hmac_key           <= 256'h0;
                            r_hmac_keyzero_pulse <= 1'b1;
                        end
                        if (i_mmio_write_data[0] && !w_hmac_busy)
                            r_hmac_kl_trigger <= 1'b1;
                        if (i_mmio_write_data[1] && !w_hmac_busy && !w_core_busy) begin
                            r_h_load_pulse <= 1'b1;
                            r_h_load_sel   <= 1'b0;   // inner
                        end
                        if (i_mmio_write_data[2] && !w_hmac_busy && !w_core_busy) begin
                            r_h_load_pulse <= 1'b1;
                            r_h_load_sel   <= 1'b1;   // outer
                        end
                    end
                    OFF_HMAC_KEY0: r_hmac_key[63:0]    <= i_mmio_write_data;
                    OFF_HMAC_KEY1: r_hmac_key[127:64]  <= i_mmio_write_data;
                    OFF_HMAC_KEY2: r_hmac_key[191:128] <= i_mmio_write_data;
                    OFF_HMAC_KEY3: r_hmac_key[255:192] <= i_mmio_write_data;
                    default: ;  // ignored
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // HMAC FSM — orchestrates the two SHA passes that compute inner/outer
    // midstates whenever software triggers HMAC_CTRL.KEY_LOAD.  See the
    // localparam list above for the state names.
    //
    // The FSM drives r_hmac_init_pulse and r_hmac_start_pulse, which OR into
    // the SHA core's i_init/i_start; it does NOT touch software's pulses or
    // BLOCK/DIGEST registers directly.
    // -------------------------------------------------------------------------
    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_hmac_fsm             <= HMAC_IDLE;
            r_inner_state          <= 256'h0;
            r_outer_state          <= 256'h0;
            r_hmac_key_valid       <= 1'b0;
            r_hmac_init_pulse      <= 1'b0;
            r_hmac_start_pulse     <= 1'b0;
            r_hmac_use_outer_block <= 1'b0;
        end else begin
            // Default: deassert pulses.
            r_hmac_init_pulse  <= 1'b0;
            r_hmac_start_pulse <= 1'b0;

            // KEY_ZERO from the MMIO block — owned here so the multi-driver
            // DRC stays clean.  Software should only assert KEY_ZERO when the
            // FSM is idle; if it lands mid-computation, the captured midstate
            // is overwritten and the in-flight pass becomes a no-op (the
            // resulting state isn't valid until the next KEY_LOAD completes).
            if (r_hmac_keyzero_pulse) begin
                r_inner_state    <= 256'h0;
                r_outer_state    <= 256'h0;
                r_hmac_key_valid <= 1'b0;
            end

            case (r_hmac_fsm)
                HMAC_IDLE: begin
                    if (r_hmac_kl_trigger) begin
                        r_hmac_init_pulse       <= 1'b1;   // INIT inner pass
                        r_hmac_use_outer_block  <= 1'b0;
                        r_hmac_key_valid        <= 1'b0;
                        r_hmac_fsm              <= HMAC_START_I;
                    end
                end
                HMAC_START_I: begin
                    // SHA core just consumed INIT — fire START with inner block.
                    r_hmac_start_pulse <= 1'b1;
                    r_hmac_fsm         <= HMAC_WAIT_I;
                end
                HMAC_WAIT_I: begin
                    if (w_core_done) begin
                        r_inner_state          <= w_digest_core;
                        r_hmac_init_pulse      <= 1'b1;    // INIT outer pass
                        r_hmac_use_outer_block <= 1'b1;
                        r_hmac_fsm             <= HMAC_START_O;
                    end
                end
                HMAC_START_O: begin
                    r_hmac_start_pulse <= 1'b1;
                    r_hmac_fsm         <= HMAC_WAIT_O;
                end
                HMAC_WAIT_O: begin
                    if (w_core_done) begin
                        r_outer_state          <= w_digest_core;
                        r_hmac_key_valid       <= 1'b1;
                        r_hmac_use_outer_block <= 1'b0;
                        r_hmac_fsm             <= HMAC_IDLE;
                    end
                end
                default: r_hmac_fsm <= HMAC_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read path — combinational mux, registered by bus_splitter.
    // DIGEST is packed two-32-bit-words-per-slot with per-word byteswap so
    // software reads SHA-256 bytes in their natural big-endian-on-the-wire
    // order via a little-endian uint64_t load.
    // -------------------------------------------------------------------------
    function [63:0] digest_pack;
        input [1:0] idx;       // 0..3
        reg [31:0] lo, hi;
        begin
            lo = w_digest_core[(2*idx + 0) * 32 +: 32];
            hi = w_digest_core[(2*idx + 1) * 32 +: 32];
            digest_pack = { bswap32(hi), bswap32(lo) };
        end
    endfunction

    always @* begin
        o_mmio_read_data = 64'h0;
        case (i_mmio_addr)
            OFF_STATUS:  o_mmio_read_data = { 62'h0, r_done_latch, w_core_busy };
            OFF_BLOCK0 + 16'h00: o_mmio_read_data = r_block_mmio[0];
            OFF_BLOCK0 + 16'h08: o_mmio_read_data = r_block_mmio[1];
            OFF_BLOCK0 + 16'h10: o_mmio_read_data = r_block_mmio[2];
            OFF_BLOCK0 + 16'h18: o_mmio_read_data = r_block_mmio[3];
            OFF_BLOCK0 + 16'h20: o_mmio_read_data = r_block_mmio[4];
            OFF_BLOCK0 + 16'h28: o_mmio_read_data = r_block_mmio[5];
            OFF_BLOCK0 + 16'h30: o_mmio_read_data = r_block_mmio[6];
            OFF_BLOCK0 + 16'h38: o_mmio_read_data = r_block_mmio[7];
            OFF_DIGEST0 + 16'h00: o_mmio_read_data = digest_pack(2'd0);
            OFF_DIGEST0 + 16'h08: o_mmio_read_data = digest_pack(2'd1);
            OFF_DIGEST0 + 16'h10: o_mmio_read_data = digest_pack(2'd2);
            OFF_DIGEST0 + 16'h18: o_mmio_read_data = digest_pack(2'd3);
            // HMAC registers
            OFF_HMAC_STATUS: o_mmio_read_data = { 62'h0, r_hmac_key_valid, w_hmac_busy };
            OFF_HMAC_KEY0:   o_mmio_read_data = r_hmac_key[63:0];
            OFF_HMAC_KEY1:   o_mmio_read_data = r_hmac_key[127:64];
            OFF_HMAC_KEY2:   o_mmio_read_data = r_hmac_key[191:128];
            OFF_HMAC_KEY3:   o_mmio_read_data = r_hmac_key[255:192];
            default:     o_mmio_read_data = 64'h0;
        endcase
    end

endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// crypto_aes — MMIO wrapper around aes_core.
//
// Device base 0xF00A_0000 (MMIO device id 0x00A). See CRYPTO_PLAN.md §4 for
// the high-level design and MMIO_MAP.md for the register table.
//
// Register layout (offsets within the device window):
//   0x000  AES_CTRL    W  [0] GO (encrypt/decrypt one block, self-clearing)
//                         [1] ENC (1=encrypt, 0=decrypt; sampled on GO)
//                         [2] KEY_LOAD (kick key-schedule expansion, self-clearing)
//                         [3] KEY_ZERO (wipe key regs, self-clearing)
//   0x008  AES_STATUS  R  [0] BUSY (any operation in progress)
//                         [1] DONE (last operation completed; sticky until next GO)
//   0x010  AES_KEY0    RW key[63:0]
//   0x018  AES_KEY1    RW key[127:64]
//   0x040  AES_IN0     RW data_in[63:0]
//   0x048  AES_IN1     RW data_in[127:64]
//   0x050  AES_OUT0    R  data_out[63:0]
//   0x058  AES_OUT1    R  data_out[127:64]
//
// All registers are 64-bit; the MMIO read mux always returns 64 bits and the
// caller selects the relevant half (sub-64-bit MMIO accesses are not used
// for this device — software uses MEMSET64/MEMGET64 throughout).
//
// MMIO timing: o_mmio_ready is asserted combinationally for any access. The
// CPU side absorbs the 1-cycle pipeline FF on the bus_splitter return path,
// so reads complete on the cycle after the strobe (same as every other MMIO
// device on this CPU).
//////////////////////////////////////////////////////////////////////////////////

module crypto_aes (
    input             i_Clk,
    input             i_Rst_L,

    // MMIO interface (offsets within device window, addr[15:0]).
    input             i_mmio_write_DV,
    input             i_mmio_read_DV,
    input      [15:0] i_mmio_addr,
    input      [63:0] i_mmio_write_data,
    input      [ 7:0] i_mmio_byte_en,   // unused — software always uses 64-bit accesses
    output reg [63:0] o_mmio_read_data,
    output            o_mmio_ready
);

    // -------------------------------------------------------------------------
    // Register offsets — AES-128 core
    // -------------------------------------------------------------------------
    localparam OFF_CTRL    = 16'h0000;
    localparam OFF_STATUS  = 16'h0008;
    localparam OFF_KEY0    = 16'h0010;
    localparam OFF_KEY1    = 16'h0018;
    localparam OFF_IN0     = 16'h0040;
    localparam OFF_IN1     = 16'h0048;
    localparam OFF_OUT0    = 16'h0050;
    localparam OFF_OUT1    = 16'h0058;

    // -------------------------------------------------------------------------
    // Register offsets — GCM / GHASH (CRYPTO_PLAN.md §5).
    //
    // Implements a single accumulating operation: on GCM_CTRL.GO, the
    // hardware computes  tag := (tag XOR X) • H  in GF(2^128).  Software
    // sequences this with AES-CTR encryption to build full AES-GCM:
    //   1. Encrypt 0^128 with AES → write result to GCM_H0/H1 (sets H).
    //   2. Encrypt J0 (96-bit IV || 0x00000001) with AES → save in SW
    //      as J0_KS (used to mask the final tag).
    //   3. For each AAD/CT block: write to GCM_X0/X1, pulse GO, wait.
    //   4. Build lengths block (lenA(64) || lenC(64), big-endian), GO, wait.
    //   5. Final tag = GCM_TAG XOR J0_KS.
    //
    // Byte ordering: all GCM regs use software's natural little-endian
    // uint64_t view (byte 0 at bits[7:0]).  Hardware byteswaps internally to
    // GHASH's network-bit-order convention.
    // -------------------------------------------------------------------------
    localparam OFF_GCM_CTRL   = 16'h0080;
    localparam OFF_GCM_STATUS = 16'h0088;
    localparam OFF_GCM_H0     = 16'h0090;
    localparam OFF_GCM_H1     = 16'h0098;
    localparam OFF_GCM_X0     = 16'h00A0;
    localparam OFF_GCM_X1     = 16'h00A8;
    localparam OFF_GCM_TAG0   = 16'h00B0;
    localparam OFF_GCM_TAG1   = 16'h00B8;

    assign o_mmio_ready = 1'b1;

    wire byte_en_unused = |i_mmio_byte_en;  // keep tools from pruning the port

    // -------------------------------------------------------------------------
    // Data registers
    // -------------------------------------------------------------------------
    reg [127:0] r_key;
    reg [127:0] r_data_in;
    wire [127:0] w_data_out;

    // Pulses to aes_core — 1-cycle wide.
    reg r_key_load_pulse;
    reg r_go_enc_pulse;
    reg r_go_dec_pulse;

    wire w_core_busy;
    wire w_core_done;

    (* KEEP_HIERARCHY = "yes" *)
    aes_core u_aes (
        .i_clk      (i_Clk),
        .i_rst      (~i_Rst_L),
        .i_key_load (r_key_load_pulse),
        .i_go_enc   (r_go_enc_pulse),
        .i_go_dec   (r_go_dec_pulse),
        .i_key      (r_key),
        .i_data_in  (r_data_in),
        .o_busy     (w_core_busy),
        .o_done     (w_core_done),
        .o_data_out (w_data_out)
    );

    // DONE flag — sticky, cleared on next GO.
    reg r_done_latch;

    // -------------------------------------------------------------------------
    // GCM/GHASH state and instance
    //
    // - r_gcm_H, r_gcm_X: assigned only in the MMIO write block (software view).
    // - r_gcm_tag, r_gcm_done_latch, r_gcm_start_pulse, r_gcm_fsm: assigned
    //   only in the GCM FSM block.
    // - r_gcm_go_pulse / r_gcm_reset_pulse: 1-cycle handoff from MMIO block
    //   to FSM block (same pattern as the HMAC keyzero pulse in crypto_sha.v).
    // -------------------------------------------------------------------------
    reg [127:0] r_gcm_H;          // MMIO-owned
    reg [127:0] r_gcm_X;          // MMIO-owned
    reg [127:0] r_gcm_tag;        // FSM-owned
    reg         r_gcm_done_latch; // FSM-owned
    reg         r_gcm_go_pulse;       // MMIO → FSM
    reg         r_gcm_reset_pulse;    // MMIO → FSM
    reg         r_gcm_start_pulse;    // FSM → ghash
    reg [1:0]   r_gcm_fsm;

    // 3 states (still fits in 2 bits).
    //   GCM_IDLE     — no operation in flight.
    //   GCM_STARTING — pulse delivered to ghash; wait 1 cycle so ghash can
    //                  see i_start, clear its sticky o_done, and assert
    //                  o_busy.  Without this, on the SECOND multiply the
    //                  GCM FSM would re-sample the *previous* multiply's
    //                  o_done (which stays high while ghash sits in
    //                  ST_IDLE between operations) and capture the stale
    //                  o_Z as the new tag.
    //   GCM_WAIT     — multiply in progress; wait for fresh o_done.
    localparam GCM_IDLE     = 2'd0;
    localparam GCM_STARTING = 2'd1;
    localparam GCM_WAIT     = 2'd2;

    wire w_gcm_busy = (r_gcm_fsm != GCM_IDLE);

    // Per-byte swap of a 128-bit value: input byte 0 (at bits[7:0]) becomes
    // output byte 15 (at bits[127:120]).  Used to translate software's
    // little-endian uint64_t view into the network-bit-order representation
    // that ghash.v operates on, and vice-versa for the result.
    function [127:0] bswap128;
        input [127:0] v;
        integer k;
        reg [127:0] o;
        begin
            for (k = 0; k < 16; k = k + 1)
                o[8*k +: 8] = v[8*(15-k) +: 8];
            bswap128 = o;
        end
    endfunction

    // GHASH inputs: byteswap r_gcm_H and (r_gcm_tag XOR r_gcm_X) into the
    // network-bit-order view ghash.v expects.
    //
    // Pipeline register inserted between the combinational XOR/bswap network
    // and the ghash module's i_X / i_H ports (CRYPTO_PLAN.md §10a).  The 128-
    // bit XOR + byte-reversal feeds all 128 destinations in ghash's r_X_shift
    // / r_V — wide fanout that was the largest wide-combinational placement
    // hotspot in the previous build.  Adds 1 cycle of latency before GHASH
    // starts; total multiply cost goes from 130 cyc → 131 cyc per block —
    // invisible against the 100+ cycle inner loop.
    //
    // Software contract is unchanged: write H, X, then trigger GO and poll
    // GCM_STATUS.BUSY.  The GCM_STARTING state below also acts as the
    // settle cycle for these pipeline regs, so no further FSM tweaks
    // needed.
    wire [127:0] w_ghash_X_in_pre = bswap128(r_gcm_tag ^ r_gcm_X);
    wire [127:0] w_ghash_H_in_pre = bswap128(r_gcm_H);
    reg  [127:0] r_ghash_X_pipe;
    reg  [127:0] r_ghash_H_pipe;
    always @(posedge i_Clk) begin
        r_ghash_X_pipe <= w_ghash_X_in_pre;
        r_ghash_H_pipe <= w_ghash_H_in_pre;
    end

    wire [127:0] w_ghash_Z_out;
    wire         w_ghash_busy;
    wire         w_ghash_done;

    (* KEEP_HIERARCHY = "yes" *)
    ghash u_ghash (
        .i_clk   (i_Clk),
        .i_rst   (~i_Rst_L),
        .i_start (r_gcm_start_pulse),
        .i_X     (r_ghash_X_pipe),
        .i_H     (r_ghash_H_pipe),
        .o_Z     (w_ghash_Z_out),
        .o_busy  (w_ghash_busy),
        .o_done  (w_ghash_done)
    );

    // -------------------------------------------------------------------------
    // Write path — software MMIO interface
    // -------------------------------------------------------------------------
    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_key             <= 128'h0;
            r_data_in         <= 128'h0;
            r_key_load_pulse  <= 1'b0;
            r_go_enc_pulse    <= 1'b0;
            r_go_dec_pulse    <= 1'b0;
            r_done_latch      <= 1'b0;
            r_gcm_H           <= 128'h0;
            r_gcm_X           <= 128'h0;
            r_gcm_go_pulse    <= 1'b0;
            r_gcm_reset_pulse <= 1'b0;
        end else begin
            // Default: deassert pulses each cycle (single-cycle wide).
            r_key_load_pulse  <= 1'b0;
            r_go_enc_pulse    <= 1'b0;
            r_go_dec_pulse    <= 1'b0;
            r_gcm_go_pulse    <= 1'b0;
            r_gcm_reset_pulse <= 1'b0;

            // Sticky DONE: set when core asserts done, cleared on the next GO.
            if (w_core_done)
                r_done_latch <= 1'b1;

            if (i_mmio_write_DV) begin
                case (i_mmio_addr)
                    OFF_CTRL: begin
                        if (i_mmio_write_data[3]) begin
                            // KEY_ZERO — wipe key (and round-key state inside core
                            // is left stale; software should follow with KEY_LOAD
                            // before reusing the engine).
                            r_key <= 128'h0;
                        end
                        if (i_mmio_write_data[2]) begin
                            r_key_load_pulse <= 1'b1;
                            r_done_latch     <= 1'b0;
                        end
                        if (i_mmio_write_data[0]) begin
                            // GO — encrypt or decrypt depending on ENC bit.
                            if (i_mmio_write_data[1])
                                r_go_enc_pulse <= 1'b1;
                            else
                                r_go_dec_pulse <= 1'b1;
                            r_done_latch <= 1'b0;
                        end
                    end
                    OFF_KEY0: r_key[63:0]    <= i_mmio_write_data;
                    OFF_KEY1: r_key[127:64]  <= i_mmio_write_data;
                    OFF_IN0:  r_data_in[63:0]   <= i_mmio_write_data;
                    OFF_IN1:  r_data_in[127:64] <= i_mmio_write_data;
                    OFF_GCM_CTRL: begin
                        // [0] GO    : kick (tag XOR X) • H accumulation
                        // [1] RESET : tag <= 0  (handed to FSM via pulse)
                        if (i_mmio_write_data[0] && !w_gcm_busy)
                            r_gcm_go_pulse <= 1'b1;
                        if (i_mmio_write_data[1])
                            r_gcm_reset_pulse <= 1'b1;
                    end
                    OFF_GCM_H0: r_gcm_H[63:0]    <= i_mmio_write_data;
                    OFF_GCM_H1: r_gcm_H[127:64]  <= i_mmio_write_data;
                    OFF_GCM_X0: r_gcm_X[63:0]    <= i_mmio_write_data;
                    OFF_GCM_X1: r_gcm_X[127:64]  <= i_mmio_write_data;
                    default: ;  // ignored
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // GCM FSM — owns r_gcm_tag, r_gcm_done_latch, r_gcm_start_pulse, r_gcm_fsm.
    // Watches the 1-cycle pulses from the MMIO block above; never assigns the
    // same regs (single-driver rule).
    // -------------------------------------------------------------------------
    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_gcm_fsm         <= GCM_IDLE;
            r_gcm_tag         <= 128'h0;
            r_gcm_done_latch  <= 1'b0;
            r_gcm_start_pulse <= 1'b0;
        end else begin
            r_gcm_start_pulse <= 1'b0;     // default deassert

            // Reset request from software always honoured immediately —
            // even if a multiply is in flight (the in-flight result is
            // captured but the tag was just reset so the new tag becomes
            // the captured value; software is expected to RESET only when
            // idle).
            if (r_gcm_reset_pulse)
                r_gcm_tag <= 128'h0;

            case (r_gcm_fsm)
                GCM_IDLE: begin
                    if (r_gcm_go_pulse) begin
                        r_gcm_start_pulse <= 1'b1;
                        r_gcm_done_latch  <= 1'b0;
                        r_gcm_fsm         <= GCM_STARTING;
                    end
                end
                GCM_STARTING: begin
                    // Drain cycle.  By the time we land in GCM_WAIT, ghash
                    // has consumed i_start and its o_done is freshly 0.
                    r_gcm_fsm <= GCM_WAIT;
                end
                GCM_WAIT: begin
                    if (w_ghash_done) begin
                        // Capture multiplier output (byteswap back to SW view).
                        r_gcm_tag        <= bswap128(w_ghash_Z_out);
                        r_gcm_done_latch <= 1'b1;
                        r_gcm_fsm        <= GCM_IDLE;
                    end
                end
                default: r_gcm_fsm <= GCM_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read path — combinational mux into o_mmio_read_data.  The mux output is
    // not registered here; the bus_splitter pipelines the return path so the
    // CPU samples the value one cycle after the read strobe.
    // -------------------------------------------------------------------------
    always @* begin
        o_mmio_read_data = 64'h0;
        case (i_mmio_addr)
            OFF_STATUS: o_mmio_read_data = {62'h0, r_done_latch, w_core_busy};
            OFF_KEY0:   o_mmio_read_data = r_key[63:0];
            OFF_KEY1:   o_mmio_read_data = r_key[127:64];
            OFF_IN0:    o_mmio_read_data = r_data_in[63:0];
            OFF_IN1:    o_mmio_read_data = r_data_in[127:64];
            OFF_OUT0:   o_mmio_read_data = w_data_out[63:0];
            OFF_OUT1:   o_mmio_read_data = w_data_out[127:64];
            // GCM / GHASH
            OFF_GCM_STATUS: o_mmio_read_data = {62'h0, r_gcm_done_latch, w_gcm_busy};
            OFF_GCM_H0:     o_mmio_read_data = r_gcm_H[63:0];
            OFF_GCM_H1:     o_mmio_read_data = r_gcm_H[127:64];
            OFF_GCM_X0:     o_mmio_read_data = r_gcm_X[63:0];
            OFF_GCM_X1:     o_mmio_read_data = r_gcm_X[127:64];
            OFF_GCM_TAG0:   o_mmio_read_data = r_gcm_tag[63:0];
            OFF_GCM_TAG1:   o_mmio_read_data = r_gcm_tag[127:64];
            default:        o_mmio_read_data = 64'h0;
        endcase
    end

endmodule

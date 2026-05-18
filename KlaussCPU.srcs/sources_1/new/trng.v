`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// trng — true random number generator (MMIO device id 0x00C, base 0xF00C_0000).
//
// Pipeline
// --------
//   16 ring oscillators ──► metastability FFs ──► XOR-fold (1 raw bit/cycle)
//                                                    │
//                                                    ├──► Repetition-Count Test
//                                                    │    (NIST SP 800-90B §4.4.1)
//                                                    │
//                                                    └──► Von Neumann debias
//                                                         (consumes pairs,
//                                                          emits ~0.25 bit/cyc
//                                                          on average)
//                                                            │
//                                                            ▼
//                                                64-bit shift accumulator
//                                                            │
//                                                            ▼
//                                                    2-deep FIFO
//                                                            │
//                                                            ▼
//                                                    MMIO TRNG_DATA reads
//                                                    (read side-effects: pop)
//
// At ~25% Von Neumann yield on a balanced source, a fresh 64-bit word lands in
// the FIFO every ~256 cycles ≈ 2.6 µs at 100 MHz.  That's ~400 k words/s —
// far more than any reasonable CSPRNG reseed cadence needs.
//
// Conditioning policy
// -------------------
// CRYPTO_PLAN.md §7 originally proposed AES-CBC-MAC conditioning of the raw
// bitstream.  The as-built version instead relies on:
//   1. 16-way XOR distillation (raises per-bit min-entropy close to 1).
//   2. Von Neumann debiasing (removes any residual single-bit bias).
//   3. Software conditioning: SSH seeds HMAC-DRBG with TRNG output (per
//      CRYPTO_PLAN.md §9.4), which is itself a NIST-compliant conditioner.
// This keeps the hardware simple (~200 LUTs) without sacrificing the security
// posture, since the cryptographic conditioning step is preserved in software.
//
// Register layout (offsets within the device window):
//   0x000  TRNG_CTRL    W  [0] ENABLE  (1 = sample ROs and produce output)
//                           [1] RESEED (self-clearing — drains FIFO,
//                                       resets accumulator, restarts the RCT)
//   0x008  TRNG_STATUS  R  [0] READY     (≥1 conditioned word in FIFO)
//                           [1] HEALTH_OK (RCT has not tripped)
//   0x010  TRNG_DATA    R  64-bit conditioned word; a READ consumes one
//                          entry from the FIFO.
//////////////////////////////////////////////////////////////////////////////////

module trng (
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
    // Register offsets
    // -------------------------------------------------------------------------
    localparam OFF_CTRL   = 16'h0000;
    localparam OFF_STATUS = 16'h0008;
    localparam OFF_DATA   = 16'h0010;

    assign o_mmio_ready = 1'b1;
    wire byte_en_unused = |i_mmio_byte_en;

    // -------------------------------------------------------------------------
    // Control register
    // -------------------------------------------------------------------------
    reg r_enable;
    reg r_reseed_pulse;

    // -------------------------------------------------------------------------
    // 16 ring oscillators + double-FF synchroniser per channel.
    //
    // First FF samples the async RO output (this is where the metastability
    // exists — that's the whole point); second FF lets the value settle out of
    // metastable region before downstream logic sees it.  This is the
    // standard CDC-from-truly-async-source pattern.  Crucially, we are NOT
    // trying to *eliminate* the metastability — the residual unpredictability
    // is the entropy source.  The second FF just bounds the worst-case
    // settling time.
    // -------------------------------------------------------------------------
    wire [15:0] w_osc;
    (* DONT_TOUCH = "true" *) reg [15:0] r_osc_meta;   // first sample
    reg [15:0] r_osc_sync;                              // second sample

    genvar gi;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_oscs
            (* DONT_TOUCH = "true" *) (* KEEP_HIERARCHY = "yes" *)
            ring_osc u_ro (.o_bit(w_osc[gi]));
        end
    endgenerate

    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_osc_meta <= 16'h0;
            r_osc_sync <= 16'h0;
        end else if (r_enable) begin
            r_osc_meta <= w_osc;
            r_osc_sync <= r_osc_meta;
        end
    end

    // XOR-fold all 16 oscillator samples → 1 raw entropy bit per cycle.
    // Combining many independent ROs raises the per-bit min-entropy of the
    // result close to 1 even if any single RO is heavily biased.
    wire w_raw_bit   = ^r_osc_sync;
    wire w_raw_valid = r_enable;

    // -------------------------------------------------------------------------
    // Repetition Count Test (NIST SP 800-90B §4.4.1)
    //
    // Track the length of the current run of identical raw bits.  If the run
    // exceeds RCT_LIMIT, latch a fault that drops HEALTH_OK until RESEED.
    // For a balanced source the probability of 32 consecutive identical bits
    // is 2^-31 — that's the chosen false-positive bound.
    // -------------------------------------------------------------------------
    localparam [5:0] RCT_LIMIT = 6'd32;

    reg       r_rct_value;
    reg [5:0] r_rct_count;
    reg       r_rct_fail;

    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_rct_value <= 1'b0;
            r_rct_count <= 6'd1;
            r_rct_fail  <= 1'b0;
        end else if (r_reseed_pulse) begin
            r_rct_value <= w_raw_bit;
            r_rct_count <= 6'd1;
            r_rct_fail  <= 1'b0;
        end else if (w_raw_valid) begin
            if (w_raw_bit == r_rct_value) begin
                if (r_rct_count >= RCT_LIMIT)
                    r_rct_fail <= 1'b1;
                else
                    r_rct_count <= r_rct_count + 6'd1;
            end else begin
                r_rct_value <= w_raw_bit;
                r_rct_count <= 6'd1;
            end
        end
    end

    wire w_health_ok = ~r_rct_fail;

    // -------------------------------------------------------------------------
    // Von Neumann debiasing
    //
    // Consume the raw stream in pairs.  Output the first bit of a pair only
    // when the two bits differ; discard otherwise.  This removes any
    // single-bit bias as long as consecutive raw bits are independent — which
    // they are at one-cycle granularity here, because the 16 ROs are sampled
    // afresh each cycle.
    // -------------------------------------------------------------------------
    reg       r_pair_state;     // 0 = waiting for first of pair, 1 = waiting for second
    reg       r_first_bit;
    reg       r_vn_bit;
    reg       r_vn_valid;

    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_pair_state <= 1'b0;
            r_first_bit  <= 1'b0;
            r_vn_bit     <= 1'b0;
            r_vn_valid   <= 1'b0;
        end else begin
            r_vn_valid <= 1'b0;     // default: no debiased bit produced this cycle
            if (r_reseed_pulse) begin
                r_pair_state <= 1'b0;
            end else if (w_raw_valid) begin
                case (r_pair_state)
                    1'b0: begin
                        r_first_bit  <= w_raw_bit;
                        r_pair_state <= 1'b1;
                    end
                    1'b1: begin
                        if (r_first_bit != w_raw_bit) begin
                            r_vn_bit   <= r_first_bit;
                            r_vn_valid <= 1'b1;
                        end
                        r_pair_state <= 1'b0;
                    end
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // 64-bit accumulator — shift in one Von Neumann bit at a time.
    // -------------------------------------------------------------------------
    reg [63:0] r_accum;
    reg [5:0]  r_accum_count;
    reg        r_accum_full_pulse;

    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_accum             <= 64'h0;
            r_accum_count       <= 6'd0;
            r_accum_full_pulse  <= 1'b0;
        end else begin
            r_accum_full_pulse <= 1'b0;
            if (r_reseed_pulse) begin
                r_accum       <= 64'h0;
                r_accum_count <= 6'd0;
            end else if (r_vn_valid) begin
                r_accum <= {r_accum[62:0], r_vn_bit};
                if (r_accum_count == 6'd63) begin
                    r_accum_count      <= 6'd0;
                    r_accum_full_pulse <= 1'b1;
                end else begin
                    r_accum_count <= r_accum_count + 6'd1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 2-deep FIFO of conditioned 64-bit words.
    // Push fires on r_accum_full_pulse; pop fires on an MMIO read of TRNG_DATA.
    // If the FIFO is full when a push fires, the new word is dropped — this is
    // by design: software reads are the limiting factor, never the source.
    //
    // Edge-detect the read strobe: the CPU keeps i_mmio_read_DV asserted for
    // two cycles (one to drive the device, one waiting for the registered
    // ready from bus_splitter).  A level-sensitive pop would consume two FIFO
    // entries per software read.
    // -------------------------------------------------------------------------
    reg [63:0] r_fifo [0:1];
    reg [1:0]  r_fifo_count;   // 0, 1, or 2
    reg        r_fifo_head;    // pop index
    reg        r_fifo_tail;    // push index

    reg  r_read_dv_prev;
    always @(posedge i_Clk) begin
        if (~i_Rst_L) r_read_dv_prev <= 1'b0;
        else          r_read_dv_prev <= i_mmio_read_DV;
    end
    wire w_read_rising = i_mmio_read_DV && ~r_read_dv_prev;
    wire w_pop = w_read_rising && (i_mmio_addr == OFF_DATA) && (r_fifo_count != 2'd0);

    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_fifo[0]    <= 64'h0;
            r_fifo[1]    <= 64'h0;
            r_fifo_count <= 2'd0;
            r_fifo_head  <= 1'b0;
            r_fifo_tail  <= 1'b0;
        end else if (r_reseed_pulse) begin
            r_fifo_count <= 2'd0;
            r_fifo_head  <= 1'b0;
            r_fifo_tail  <= 1'b0;
        end else begin
            case ({r_accum_full_pulse, w_pop})
                2'b10: begin    // push only
                    if (r_fifo_count != 2'd2) begin
                        r_fifo[r_fifo_tail] <= r_accum;
                        r_fifo_tail         <= ~r_fifo_tail;
                        r_fifo_count        <= r_fifo_count + 2'd1;
                    end
                    // else dropped
                end
                2'b01: begin    // pop only
                    r_fifo_head  <= ~r_fifo_head;
                    r_fifo_count <= r_fifo_count - 2'd1;
                end
                2'b11: begin    // push + pop simultaneously
                    r_fifo[r_fifo_tail] <= r_accum;
                    r_fifo_tail         <= ~r_fifo_tail;
                    r_fifo_head         <= ~r_fifo_head;
                    // count unchanged
                end
                default: ;
            endcase
        end
    end

    wire w_ready = (r_fifo_count != 2'd0);

    // -------------------------------------------------------------------------
    // MMIO write path
    // -------------------------------------------------------------------------
    always @(posedge i_Clk) begin
        if (~i_Rst_L) begin
            r_enable       <= 1'b0;
            r_reseed_pulse <= 1'b0;
        end else begin
            r_reseed_pulse <= 1'b0;
            if (i_mmio_write_DV && i_mmio_addr == OFF_CTRL) begin
                r_enable       <= i_mmio_write_data[0];
                if (i_mmio_write_data[1])
                    r_reseed_pulse <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // MMIO read path
    // -------------------------------------------------------------------------
    always @* begin
        o_mmio_read_data = 64'h0;
        case (i_mmio_addr)
            OFF_CTRL:   o_mmio_read_data = {63'h0, r_enable};
            OFF_STATUS: o_mmio_read_data = {62'h0, w_health_ok, w_ready};
            OFF_DATA:   o_mmio_read_data = r_fifo[r_fifo_head];
            default:    o_mmio_read_data = 64'h0;
        endcase
    end

endmodule

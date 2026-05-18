`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_hmac_sha256 — exercises the full HMAC-SHA-256 construction by driving
// sha256_core through the steps an HMAC implementation must perform.
//
// What this testbench validates:
//   - sha256_core.i_h_load + i_h_in actually load H[0..7] correctly
//   - The HMAC midstate caching pattern is sound (compute inner/outer
//     midstates, restore them, finish the hash, get the right tag)
//   - Block construction with the byteswap conventions used by the wrapper
//
// What it does NOT validate (covered by the on-FPGA C self-test):
//   - The crypto_sha MMIO interface routing
//   - The HMAC FSM's auto-sequencing of the two midstate-computation passes
//
// Reference vector: RFC 4231 Test Case 1
//   Key  = 0x0b × 20 (zero-padded to 64 bytes inside HMAC)
//   Data = "Hi There" (8 bytes ASCII)
//   HMAC-SHA-256 = b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7
//////////////////////////////////////////////////////////////////////////////////

module tb_hmac_sha256;

    reg          clk;
    reg          rst;
    reg          init;
    reg          start;
    reg  [511:0] block;
    reg          h_load;
    reg  [255:0] h_in;
    wire         busy;
    wire         done;
    wire [255:0] digest;

    integer fails = 0;
    integer passes = 0;

    sha256_core dut (
        .i_clk    (clk),
        .i_rst    (rst),
        .i_init   (init),
        .i_start  (start),
        .i_block  (block),
        .i_h_load (h_load),
        .i_h_in   (h_in),
        .o_busy   (busy),
        .o_done   (done),
        .o_digest (digest)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    task wait_done;
        begin
            @(posedge clk);
            while (busy) @(posedge clk);
            @(posedge clk);
        end
    endtask

    task feed_block;
        input [511:0] m_concat;   // {M[0], M[1], ..., M[15]} with M[0] at high bits
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1)
                block[32*i +: 32] = m_concat[32*(15-i) +: 32];
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            wait_done();
        end
    endtask

    task pulse_init;
        begin
            init = 1'b1;
            @(posedge clk);
            init = 1'b0;
            @(posedge clk);
        end
    endtask

    task pulse_hload;
        input [255:0] state;
        begin
            // i_h_in is { H7, H6, ..., H0 } with H0 at bits [31:0].
            h_in   = state;
            h_load = 1'b1;
            @(posedge clk);
            h_load = 1'b0;
            @(posedge clk);
        end
    endtask

    reg [255:0] inner_state;
    reg [255:0] outer_state;
    reg [255:0] inner_digest;
    reg [255:0] final_tag;

    // RFC 4231 TC1 expected HMAC tag.  Canonical byte-stream form is
    // b0344c61_..._2e32cff7.  o_digest is {H[7],...,H[0]}, so the literal
    // has H[7] at the high bits — i.e. the canonical hex with the 32-bit
    // word ORDER reversed (each word's bytes stay in the same order):
    localparam [255:0] EXPECTED_TAG =
        256'h 2e32cff7_26e9376c_c9833da7_881dc200_af0bf12b_5ca8afce_d8db3853_b0344c61;

    initial begin
        $display("=== tb_hmac_sha256: starting RFC 4231 TC1 run ===");
        rst    = 1'b1;
        init   = 1'b0;
        start  = 1'b0;
        h_load = 1'b0;
        block  = 512'h0;
        h_in   = 256'h0;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // ---- Step 1: compute inner_state = SHA-256-compress(IV, K' ⊕ ipad) ----
        // Key K = 0x0b × 20.  K' = K || 0 × 44 (zero-extended to 64 bytes).
        // K' ⊕ ipad: bytes 0..19 = 0x0b ⊕ 0x36 = 0x3d; bytes 20..63 = 0x36.
        //
        // As 32-bit big-endian words:
        //   M[0..4]  = 0x3d3d3d3d (5 words = 20 bytes of 0x3d)
        //   M[5..15] = 0x36363636 (11 words of pure ipad)
        pulse_init();
        feed_block({
            32'h 3d3d3d3d, 32'h 3d3d3d3d, 32'h 3d3d3d3d, 32'h 3d3d3d3d,
            32'h 3d3d3d3d, 32'h 36363636, 32'h 36363636, 32'h 36363636,
            32'h 36363636, 32'h 36363636, 32'h 36363636, 32'h 36363636,
            32'h 36363636, 32'h 36363636, 32'h 36363636, 32'h 36363636
        });
        inner_state = digest;
        $display("       inner_state  = %064h", inner_state);

        // ---- Step 2: compute outer_state = SHA-256-compress(IV, K' ⊕ opad) ----
        // K' ⊕ opad: bytes 0..19 = 0x0b ⊕ 0x5c = 0x57; bytes 20..63 = 0x5c.
        pulse_init();
        feed_block({
            32'h 57575757, 32'h 57575757, 32'h 57575757, 32'h 57575757,
            32'h 57575757, 32'h 5c5c5c5c, 32'h 5c5c5c5c, 32'h 5c5c5c5c,
            32'h 5c5c5c5c, 32'h 5c5c5c5c, 32'h 5c5c5c5c, 32'h 5c5c5c5c,
            32'h 5c5c5c5c, 32'h 5c5c5c5c, 32'h 5c5c5c5c, 32'h 5c5c5c5c
        });
        outer_state = digest;
        $display("       outer_state  = %064h", outer_state);

        // ---- Step 3: inner hash — H ← inner_state, process data + padding ----
        // Data: "Hi There" = 48 69 20 54 68 65 72 65 (8 bytes)
        // Virtual prefix: 64 bytes of K' ⊕ ipad already absorbed → bit count = 64*8 + 8*8 = 576
        // Block: data (8B) || 0x80 (1B) || zeros (47B) || 64-bit-BE(576) (8B)
        //   M[0] = "Hi T" = 0x48692054
        //   M[1] = "here" = 0x68657265
        //   M[2] = 0x80000000
        //   M[3..13] = 0
        //   M[14] = 0x00000000 (length-hi)
        //   M[15] = 0x00000240 (length-lo, 576 = 0x240)
        pulse_hload(inner_state);
        feed_block({
            32'h 48692054, 32'h 68657265, 32'h 80000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000240
        });
        inner_digest = digest;
        $display("       inner_digest = %064h", inner_digest);

        // ---- Step 4: outer hash — H ← outer_state, process inner_digest + padding ----
        // Virtual prefix: 64 bytes of K' ⊕ opad → total bits = 64*8 + 32*8 = 768 = 0x300
        // Block: inner_digest (32B) || 0x80 (1B) || zeros (23B) || 64-bit-BE(768) (8B)
        //   M[0..7] = inner_digest as 8 big-endian 32-bit words.
        //   M[8] = 0x80000000
        //   M[9..13] = 0
        //   M[14] = 0
        //   M[15] = 0x00000300
        pulse_hload(outer_state);
        feed_block({
            inner_digest[255:224], inner_digest[223:192],
            inner_digest[191:160], inner_digest[159:128],
            inner_digest[127:96],  inner_digest[95:64],
            inner_digest[63:32],   inner_digest[31:0],
            32'h 80000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000300
        });
        final_tag = digest;

        // ---- Step 5: verify ----
        if (final_tag === EXPECTED_TAG) begin
            $display("[PASS] HMAC-SHA-256 RFC 4231 TC1: tag = %064h", final_tag);
            passes = passes + 1;
        end else begin
            $display("[FAIL] HMAC-SHA-256 RFC 4231 TC1:");
            $display("       got      = %064h", final_tag);
            $display("       expected = %064h", EXPECTED_TAG);
            fails = fails + 1;
        end

        if (fails == 0)
            $display("=== tb_hmac_sha256: ALL %0d TESTS PASSED ===", passes);
        else
            $display("=== tb_hmac_sha256: %0d FAIL(s), %0d pass(es) ===", fails, passes);
        $finish;
    end

endmodule

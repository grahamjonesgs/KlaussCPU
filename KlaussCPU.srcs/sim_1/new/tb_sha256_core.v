`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_sha256_core — Verilog testbench for sha256_core.v against FIPS-180-4 KATs.
//
// Vectors (all from FIPS-180-4 Appendix B / RFC 6234):
//   1. Empty message "":
//        block: 0x80 || 0x00 × 55 || (64-bit BE length = 0)
//        digest: e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855
//
//   2. "abc" (3 bytes):
//        block: "abc" || 0x80 || 0x00 × 52 || (64-bit BE length = 24)
//        digest: ba7816bf8f01cfea414140de5dae2223b00361a3396177a9cb410ff61f20015a
//
//   3. 56-byte "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
//        block 1: message bytes || 0x80 || 0x00 × 7
//        block 2: 0x00 × 56 || (64-bit BE length = 448)
//        digest: 248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1
//
// The third vector exercises multi-block streaming — the engine must carry the
// intermediate H state forward from block 1 to block 2.
//
// Byte layout into i_block: word M[i] lands at i_block[32*i +: 32].
// M[i] is a 32-bit big-endian view of message bytes (4i, 4i+1, 4i+2, 4i+3).
//
// o_digest convention: o_digest = {H[7], H[6], ..., H[0]} — H[7] at high bits,
// H[0] at bits [31:0].  So expected literals are written H[7] first, which is
// the *reverse* of the canonical SHA-256 byte-stream hex.
//////////////////////////////////////////////////////////////////////////////////

module tb_sha256_core;

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

    // Build a 512-bit block from 16 M[i] words.  M[0]..M[15] are passed as
    // a single concatenated 512-bit value with M[0] at the high bits.
    task feed_block_and_run;
        input [511:0] m_concat;   // {M[0], M[1], ..., M[15]} with M[0] at high bits
        integer i;
        begin
            // Convert from "M[0] at high bits" to i_block's "M[0] at low bits"
            // (sha256_core indexes i_block[32*i +: 32] = M[i]).
            for (i = 0; i < 16; i = i + 1)
                block[32*i +: 32] = m_concat[32*(15-i) +: 32];
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            wait_done();
        end
    endtask

    task check_digest;
        input integer vec_num;
        input [255:0] expected;
        begin
            if (digest === expected) begin
                $display("[PASS] SHA-256 vector %0d: %064h", vec_num, digest);
                passes = passes + 1;
            end else begin
                $display("[FAIL] SHA-256 vector %0d: got %064h, expected %064h",
                         vec_num, digest, expected);
                fails = fails + 1;
            end
        end
    endtask

    task reset_init;
        begin
            init = 1'b1;
            @(posedge clk);
            init = 1'b0;
            @(posedge clk);
        end
    endtask

    initial begin
        $display("=== tb_sha256_core: starting FIPS-180-4 KAT run ===");
        rst    = 1'b1;
        init   = 1'b0;
        start  = 1'b0;
        block  = 512'h0;
        h_load = 1'b0;
        h_in   = 256'h0;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // ---- Vector 1: empty message "" ----
        reset_init();
        // M[0] = 0x80000000 (padding marker only — no message bytes)
        // M[1..14] = 0
        // M[15] = 0 (length 0 bits)
        feed_block_and_run({
            32'h 80000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000
        });
        // Canonical SHA256("") = e3b0c442_98fc1c14_..._7852b855 (byte stream).
        // Reversed for {H[7],...,H[0]} layout:
        check_digest(1,
            256'h 7852b855_a495991b_649b934c_27ae41e4_996fb924_9afbf4c8_98fc1c14_e3b0c442);

        // ---- Vector 2: "abc" ----
        reset_init();
        // M[0] = 0x61_62_63_80  (a,b,c,padding)
        // M[1..14] = 0
        // M[15] = 0x00000018 (length 24 bits)
        feed_block_and_run({
            32'h 61626380, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000018
        });
        // Canonical SHA256("abc") = ba7816bf_..._1f20015a (byte stream).
        check_digest(2,
            256'h 1f20015a_cb410ff6_396177a9_b00361a3_5dae2223_414140de_8f01cfea_ba7816bf);

        // ---- Vector 3: 56-byte "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq" ----
        // Block 1: message bytes (M[0..13]) + 0x80 padding at byte 56 (M[14] hi byte)
        reset_init();
        feed_block_and_run({
            32'h 61626364, 32'h 62636465, 32'h 63646566, 32'h 64656667,
            32'h 65666768, 32'h 66676869, 32'h 6768696a, 32'h 68696a6b,
            32'h 696a6b6c, 32'h 6a6b6c6d, 32'h 6b6c6d6e, 32'h 6c6d6e6f,
            32'h 6d6e6f70, 32'h 6e6f7071, 32'h 80000000, 32'h 00000000
        });
        // Block 2: zero-pad + length 448 bits BE in M[15] (low half) = 0x000001c0
        feed_block_and_run({
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 00000000,
            32'h 00000000, 32'h 00000000, 32'h 00000000, 32'h 000001c0
        });
        // Canonical SHA256("abcdbcdecde…q") = 248d6a61_..._19db06c1 (byte stream).
        check_digest(3,
            256'h 19db06c1_f6ecedd4_64ff2167_a33ce459_0c3e6039_e5c02693_d20638b8_248d6a61);

        if (fails == 0)
            $display("=== tb_sha256_core: ALL %0d TESTS PASSED ===", passes);
        else
            $display("=== tb_sha256_core: %0d FAIL(s), %0d pass(es) ===", fails, passes);
        $finish;
    end

endmodule

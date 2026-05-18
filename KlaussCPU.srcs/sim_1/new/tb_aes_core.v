`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_aes_core — Verilog testbench for aes_core.v against FIPS-197 vectors.
//
// Test vectors (all from FIPS-197 Appendix B / NIST CAVP AES-128):
//   Vector 1: FIPS-197 §B "Cipher Example"
//     K  = 2b7e151628aed2a6abf7158809cf4f3c
//     PT = 3243f6a8885a308d313198a2e0370734
//     CT = 3925841d02dc09fbdc118597196a0b32
//
//   Vector 2: FIPS-197 Appendix C.1 "AES-128"
//     K  = 000102030405060708090a0b0c0d0e0f
//     PT = 00112233445566778899aabbccddeeff
//     CT = 69c4e0d86a7b0430d8cdb78070b4c55a
//
//   Vector 3: NIST SP 800-38A AES-CTR test — single-block, all zero key
//     K  = 00000000000000000000000000000000
//     PT = 00000000000000000000000000000000
//     CT = 66e94bd4ef8a2c3b884cfa59ca342b2e   (= H constant from GHASH TC1)
//
// Each vector is encrypted, then decrypted, and both directions checked.
// Result: $display PASS / FAIL lines per test; final summary line at end.
//
// Byte ordering: the testbench passes 128-bit values to aes_core such that
// PT bytes 0..15 are at bits [127:120] down to [7:0] — i.e. byte 0 is the
// MSB of the 128-bit reg.  This matches the AES specification.  The MMIO
// wrapper handles the conversion from the CPU's little-endian uint64 layout.
//
// Run from Vivado simulator (xsim) — no waveforms required; everything is
// printed.
//////////////////////////////////////////////////////////////////////////////////

module tb_aes_core;

    reg          clk;
    reg          rst;
    reg          key_load;
    reg          go_enc;
    reg          go_dec;
    reg  [127:0] key;
    reg  [127:0] data_in;
    wire         busy;
    wire         done;
    wire [127:0] data_out;

    integer fails = 0;
    integer passes = 0;

    aes_core dut (
        .i_clk      (clk),
        .i_rst      (rst),
        .i_key_load (key_load),
        .i_go_enc   (go_enc),
        .i_go_dec   (go_dec),
        .i_key      (key),
        .i_data_in  (data_in),
        .o_busy     (busy),
        .o_done     (done),
        .o_data_out (data_out)
    );

    // 100 MHz clock
    initial clk = 0;
    always #5 clk = ~clk;

    // Wait until the core de-asserts busy after a one-cycle pulse.
    task wait_done;
        begin
            @(posedge clk);
            while (busy) @(posedge clk);
            // One extra cycle so data_out latches.
            @(posedge clk);
        end
    endtask

    // Encrypt-and-check, then decrypt-and-check.
    task run_vector;
        input [127:0] k_in;
        input [127:0] pt_in;
        input [127:0] ct_expected;
        input integer vec_num;
        reg   [127:0] enc_out;
        reg   [127:0] dec_out;
        begin
            key      = k_in;
            key_load = 1'b1;
            @(posedge clk);
            key_load = 1'b0;
            wait_done();

            // Encrypt
            data_in = pt_in;
            go_enc  = 1'b1;
            @(posedge clk);
            go_enc  = 1'b0;
            wait_done();
            enc_out = data_out;

            if (enc_out === ct_expected) begin
                $display("[PASS] AES vector %0d ENCRYPT: ct = %032h", vec_num, enc_out);
                passes = passes + 1;
            end else begin
                $display("[FAIL] AES vector %0d ENCRYPT: got %032h, expected %032h",
                         vec_num, enc_out, ct_expected);
                fails = fails + 1;
            end

            // Decrypt the result, should round-trip back to plaintext.
            data_in = ct_expected;
            go_dec  = 1'b1;
            @(posedge clk);
            go_dec  = 1'b0;
            wait_done();
            dec_out = data_out;

            if (dec_out === pt_in) begin
                $display("[PASS] AES vector %0d DECRYPT: pt = %032h", vec_num, dec_out);
                passes = passes + 1;
            end else begin
                $display("[FAIL] AES vector %0d DECRYPT: got %032h, expected %032h",
                         vec_num, dec_out, pt_in);
                fails = fails + 1;
            end
        end
    endtask

    initial begin
        $display("=== tb_aes_core: starting FIPS-197 KAT run ===");
        rst      = 1'b1;
        key_load = 1'b0;
        go_enc   = 1'b0;
        go_dec   = 1'b0;
        key      = 128'h0;
        data_in  = 128'h0;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // Vector 1 — FIPS-197 §B
        run_vector(
            128'h 2b7e1516_28aed2a6_abf71588_09cf4f3c,
            128'h 3243f6a8_885a308d_313198a2_e0370734,
            128'h 3925841d_02dc09fb_dc118597_196a0b32,
            1
        );

        // Vector 2 — FIPS-197 Appendix C.1
        run_vector(
            128'h 00010203_04050607_08090a0b_0c0d0e0f,
            128'h 00112233_44556677_8899aabb_ccddeeff,
            128'h 69c4e0d8_6a7b0430_d8cdb780_70b4c55a,
            2
        );

        // Vector 3 — encrypt of zero with zero key (also the GHASH-TC1 H value)
        run_vector(
            128'h 00000000_00000000_00000000_00000000,
            128'h 00000000_00000000_00000000_00000000,
            128'h 66e94bd4_ef8a2c3b_884cfa59_ca342b2e,
            3
        );

        if (fails == 0)
            $display("=== tb_aes_core: ALL %0d TESTS PASSED ===", passes);
        else
            $display("=== tb_aes_core: %0d FAIL(s), %0d pass(es) ===", fails, passes);
        $finish;
    end

endmodule

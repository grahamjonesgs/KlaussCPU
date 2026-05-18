`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_ghash — Verilog testbench for ghash.v, the GF(2^128) multiplier core.
//
// Vectors derived from NIST SP 800-38D Annex B (GCM/GMAC test cases).
//
// Bit ordering note: ghash.v takes its operands already in NIST bit order
// (byte 0 of the message at Verilog bits [127:120], MSB-first within each
// byte).  Hex literals like 128'h66e94bd4... place 0x66 in [127:120], which
// is exactly NIST byte 0 — so we write the test vectors as published.
//
// Tests:
//   1. gf_mult(0, H) = 0                         — trivial identity
//   2. gf_mult(X, 0) = 0                         — symmetric identity
//   3. NIST GCM Test Case 2 — full GHASH computation chain:
//        H        = AES_K=0(0^128) = 0x66e94bd4_ef8a2c3b_884cfa59_ca342b2e
//        CT       = 0x0388dace_60b6a392_f328c2b9_71b2fe78
//        lengths  = 0^64 || (128 bits as 64-bit BE) = 0x...0000_00000080
//        Z1       = CT • H
//        Z2       = (Z1 XOR lengths) • H        = GHASH(empty AAD, CT)
//      Expected GHASH = T XOR AES_K(J0):
//        T          = 0xab6e47d4_2cec13bd_f53a67b2_1257bddf
//        AES_K(J0)  = 0x58e2fcce_fa7e3061_367f1d57_a4e7455a
//        GHASH      = 0xf38cbb1a_d69223dc_c3457ae5_b6b0f885
//      (Z1 itself is not checked separately — if the end-to-end value
//      matches, the math is consistent.)
//////////////////////////////////////////////////////////////////////////////////

module tb_ghash;

    reg          clk;
    reg          rst;
    reg          start;
    reg  [127:0] X;
    reg  [127:0] H;
    wire [127:0] Z;
    wire         busy;
    wire         done;

    integer fails  = 0;
    integer passes = 0;

    ghash dut (
        .i_clk   (clk),
        .i_rst   (rst),
        .i_start (start),
        .i_X     (X),
        .i_H     (H),
        .o_Z     (Z),
        .o_busy  (busy),
        .o_done  (done)
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

    task run_mult;
        input [127:0] x_in;
        input [127:0] h_in;
        begin
            X     = x_in;
            H     = h_in;
            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            wait_done();
        end
    endtask

    task check;
        input integer vec_num;
        input [127:0] expected;
        begin
            if (Z === expected) begin
                $display("[PASS] GHASH vector %0d: Z = %032h", vec_num, Z);
                passes = passes + 1;
            end else begin
                $display("[FAIL] GHASH vector %0d: got %032h, expected %032h",
                         vec_num, Z, expected);
                fails = fails + 1;
            end
        end
    endtask

    reg [127:0] Z1;
    localparam [127:0] H_TC2   = 128'h 66e94bd4_ef8a2c3b_884cfa59_ca342b2e;
    localparam [127:0] CT_TC2  = 128'h 0388dace_60b6a392_f328c2b9_71b2fe78;
    localparam [127:0] LEN_TC2 = 128'h 00000000_00000000_00000000_00000080;
    localparam [127:0] EXP_TC2 = 128'h f38cbb1a_d69223dc_c3457ae5_b6b0f885;

    initial begin
        $display("=== tb_ghash: starting GHASH KAT run ===");
        rst   = 1'b1;
        start = 1'b0;
        X     = 128'h0;
        H     = 128'h0;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // --- Vector 1: 0 • H = 0 ---
        run_mult(128'h0, H_TC2);
        check(1, 128'h0);

        // --- Vector 2: X • 0 = 0 ---
        run_mult(CT_TC2, 128'h0);
        check(2, 128'h0);

        // --- Vector 3: NIST GCM TC2 — chained two-step GHASH ---
        // Z1 = CT • H
        run_mult(CT_TC2, H_TC2);
        Z1 = Z;
        $display("       GHASH TC2 intermediate Z1 = %032h", Z1);

        // Z2 = (Z1 XOR lengths) • H
        run_mult(Z1 ^ LEN_TC2, H_TC2);
        check(3, EXP_TC2);

        if (fails == 0)
            $display("=== tb_ghash: ALL %0d TESTS PASSED ===", passes);
        else
            $display("=== tb_ghash: %0d FAIL(s), %0d pass(es) ===", fails, passes);
        $finish;
    end

endmodule

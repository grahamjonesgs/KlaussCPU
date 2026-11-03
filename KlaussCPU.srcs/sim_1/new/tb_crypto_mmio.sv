`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_crypto_mmio — MMIO-level test of crypto_aes + crypto_sha.
//
// A generic sequencer: tools/vsim/crypto_kat.py writes a list of MMIO ops
// (with expected values from Python hashlib / hmac / cryptography) to
// +ops=<file>; this bench replays them against the two wrappers.
//
// Op line: "<op> <dev> <addr> <data> <mask>" (hex; dev 0 = AES, 1 = SHA)
//   1 W      write data to addr
//   2 R      read addr, check (read & mask) == data
//   3 POLL   read addr until (read & mask) == data (timeout = failure)
//   5 SAVE   read addr into scratch[data]
//   6 REPLAY write scratch[data] to addr
//   7 MARK   print test marker number data
//
// The write strobe is held 3 cycles, like the real MMIO write (DV held
// until the 2-cycle-delayed ack), so level-sensitive side effects are
// exercised the way the SoC drives them.
//////////////////////////////////////////////////////////////////////////////////

module tb_crypto_mmio;
    logic clk = 0, rst_l = 0;
    always #5 clk = ~clk;

    mmio_if aes_bus();
    mmio_if sha_bus();

    crypto_aes u_aes (.i_Clk(clk), .i_Rst_L(rst_l), .mmio(aes_bus.slave));
    crypto_sha u_sha (.i_Clk(clk), .i_Rst_L(rst_l), .mmio(sha_bus.slave));

    logic [63:0] scratch [0:63];
    integer fails = 0, checks = 0, mark = 0;

    task automatic idle_bus;
        aes_bus.write_DV = 0; aes_bus.read_DV = 0; aes_bus.addr = 0;
        aes_bus.write_data = 0; aes_bus.byte_en = 8'hFF;
        sha_bus.write_DV = 0; sha_bus.read_DV = 0; sha_bus.addr = 0;
        sha_bus.write_data = 0; sha_bus.byte_en = 8'hFF;
    endtask

    task automatic wr(input int dev, input [15:0] a, input [63:0] d);
        if (dev == 0) begin aes_bus.addr = {16'hF00A, a}; aes_bus.write_data = d; aes_bus.write_DV = 1; end
        else          begin sha_bus.addr = {16'hF00B, a}; sha_bus.write_data = d; sha_bus.write_DV = 1; end
        repeat (3) @(posedge clk);
        #1 idle_bus();
        @(posedge clk); #1;
    endtask

    task automatic rd(input int dev, input [15:0] a, output [63:0] d);
        if (dev == 0) begin aes_bus.addr = {16'hF00A, a}; aes_bus.read_DV = 1; end
        else          begin sha_bus.addr = {16'hF00B, a}; sha_bus.read_DV = 1; end
        @(posedge clk); #1;
        d = (dev == 0) ? aes_bus.read_data : sha_bus.read_data;
        idle_bus();
        @(posedge clk); #1;
    endtask

    string opsfile;
    integer fd, n, op, dev, polls;
    logic [63:0] a64, d, m, r;

    initial begin
        idle_bus();
        if (!$value$plusargs("ops=%s", opsfile)) begin
            $display("FAIL: no +ops=<file>"); $finish;
        end
        fd = $fopen(opsfile, "r");
        if (fd == 0) begin $display("FAIL: cannot open %s", opsfile); $finish; end
        repeat (4) @(posedge clk);
        #1 rst_l = 1;
        repeat (2) @(posedge clk); #1;
        while (!$feof(fd)) begin
            n = $fscanf(fd, "%h %h %h %h %h\n", op, dev, a64, d, m);
            if (n != 5) continue;
            case (op)
                1: wr(dev, a64[15:0], d);
                2: begin
                    rd(dev, a64[15:0], r);
                    checks++;
                    if ((r & m) !== d) begin
                        fails++;
                        $display("FAIL test %0d: dev %0d @%h read %h & %h, want %h",
                                 mark, dev, a64[15:0], r, m, d);
                    end
                end
                3: begin
                    polls = 0;
                    rd(dev, a64[15:0], r);
                    while ((r & m) !== d && polls < 2000) begin rd(dev, a64[15:0], r); polls++; end
                    if (polls >= 2000) begin
                        fails++;
                        $display("FAIL test %0d: poll timeout dev %0d @%h (last %h)", mark, dev, a64[15:0], r);
                    end
                end
                5: rd(dev, a64[15:0], scratch[d[5:0]]);
                6: wr(dev, a64[15:0], scratch[d[5:0]]);
                7: mark = d;
                default: begin $display("FAIL: bad op %0d", op); fails++; end
            endcase
        end
        $display("%s: %0d checks, %0d failures", fails == 0 ? "PASS" : "FAIL", checks, fails);
        $finish;
    end

    initial begin #50_000_000; $display("FAIL: global timeout"); $finish; end
endmodule

`timescale 1ns / 1ps
// tb_qspi_ctrl — unit test for qspi_ctrl.sv against a behavioural S25FL128S
// (SPI mode 0, single-bit).  Self-checking; prints "TB_QSPI PASS" or
// "TB_QSPI FAIL n".  No waveform dump.
//
//   xvlog -sv mmio_if.sv qspi_ctrl.sv tb_qspi_ctrl.sv && xelab tb_qspi_ctrl && xsim tb_qspi_ctrl -R
//   (or: iverilog -g2012 / verilator --binary)

// ---------------------------------------------------------------------------
// Flash model
// ---------------------------------------------------------------------------
module s25fl_model (
    input        sck,
    input        cs_n,
    input        mosi,
    output logic miso
);
    byte unsigned mem [int];           // sparse; absent = 0xFF
    logic        wel = 0;
    logic        wip = 0;
    int          nbits;
    logic [7:0]  sh_in;
    logic [7:0]  sh_out;
    logic [7:0]  op;
    int          nbytes;
    logic [23:0] addr;
    logic [7:0]  pp_buf [0:255];
    bit          pp_used [0:255];
    int          ops_seen = 0;         // complete opcodes received
    int          low_writes = 0;       // program/erase below 0xF00000
    int          protocol_err = 0;

    function automatic byte unsigned rd(int a);
        return mem.exists(a) ? mem[a] : 8'hFF;
    endfunction

    initial miso = 1'bz;

    always @(negedge cs_n) begin
        nbits = 0; nbytes = 0; sh_out = 8'hFF;
        for (int i = 0; i < 256; i++) pp_used[i] = 0;
    end

    always @(posedge sck) if (!cs_n) begin
        sh_in = {sh_in[6:0], mosi};
        nbits++;
        if (nbits % 8 == 0) begin
            // Addressed commands must arrive in their 4-byte forms
            // (13/12/DC/21); the 3-byte ones would follow BAR.EXTADD.
            if (nbytes == 0) begin
                op = sh_in; ops_seen++;
                if (op == 8'h03 || op == 8'h02 || op == 8'hD8 || op == 8'h20) begin
                    protocol_err++;
                    $display("MODEL: 3-byte-address opcode %02x on the wire", op);
                end
            end
            else if (nbytes == 1 && (op == 8'h13 || op == 8'h12 || op == 8'hDC || op == 8'h21)) begin if (sh_in != 8'h00) protocol_err++; end
            else if (nbytes == 2) addr[23:16] = sh_in;
            else if (nbytes == 3) addr[15:8]  = sh_in;
            else if (nbytes == 4) addr[7:0]   = sh_in;
            else if (op == 8'h12 && wel && !wip) begin
                pp_buf[(addr[7:0] + (nbytes - 5)) & 8'hFF] = sh_in;
                pp_used[(addr[7:0] + (nbytes - 5)) & 8'hFF] = 1;
            end
            nbytes++;
        end
    end

    // Next output byte, decided after each complete byte (falling edge).
    always @(negedge sck) if (!cs_n) begin
        if (nbits % 8 == 0 && nbits > 0) begin
            automatic int k;
            k = nbytes - 1;            // bytes already completed minus opcode
            case (op)
                8'h9F: begin
                    automatic byte unsigned id [6] = '{8'h01, 8'h20, 8'h18, 8'h4D, 8'h01, 8'h80};
                    sh_out = (k < 6) ? id[k] : 8'hFF;
                end
                8'h05: sh_out = {6'b0, wel, wip};
                8'h07, 8'h35, 8'h16: sh_out = 8'h00;
                8'h13: sh_out = (nbytes >= 5) ? rd(addr + nbytes - 5) : 8'hFF;
                default: sh_out = 8'hFF;
            endcase
        end else begin
            sh_out = {sh_out[6:0], 1'b1};
        end
        miso <= #6 sh_out[7];
    end

    always @(posedge cs_n) begin
        miso <= #6 1'bz;
        if (nbits % 8 != 0) begin
            protocol_err++;
            $display("MODEL: CS# rose after %0d bits (not a byte multiple)", nbits);
        end else if (nbits > 0) begin
            case (op)
                8'h06: wel = 1;
                8'h04: wel = 0;
                8'h12: if (nbytes >= 5 && wel && !wip) begin
                    if (addr < 24'hF0_0000) low_writes++;
                    for (int i = 0; i < 256; i++)
                        if (pp_used[i]) mem[{addr[23:8], 8'(i)}] = rd({addr[23:8], 8'(i)}) & pp_buf[i];
                    wip = 1; wel = 0;
                    fork begin #3000; wip = 0; end join_none
                end
                8'hDC, 8'h21: if (nbytes == 5 && wel && !wip) begin
                    automatic int base, size;
                    size = (op == 8'hDC) ? 32'h1_0000 : 32'h1000;
                    base = addr & ~(size - 1);
                    if (base < 24'hF0_0000) low_writes++;
                    for (int a = base; a < base + size; a++)
                        if (mem.exists(a)) mem.delete(a);
                    wip = 1; wel = 0;
                    fork begin #20000; wip = 0; end join_none
                end
                default: ;
            endcase
        end
    end
endmodule

// ---------------------------------------------------------------------------
// Testbench
// ---------------------------------------------------------------------------
module tb_qspi_ctrl;
    logic clk = 0, rst_l = 0, eos = 0;
    always #5 clk = ~clk;

    mmio_if bus();
    wire sck, cs_n, mosi, miso, wp_n, hold_n;
    tri1 miso_pu;
    assign miso_pu = miso;

    qspi_ctrl dut (
        .i_Clk(clk), .i_Rst_L(rst_l), .i_eos(eos), .mmio(bus),
        .o_sck(sck), .o_cs_n(cs_n), .o_mosi(mosi), .i_miso(miso_pu),
        .o_wp_n(wp_n), .o_hold_n(hold_n)
    );
    s25fl_model flash (.sck(sck), .cs_n(cs_n), .mosi(mosi), .miso(miso));

    localparam logic [31:0] BASE = 32'hF012_0000;
    int errors = 0;

    // Like the CPU: write_DV held for HOLD cycles (until the MMIO ready).
    int hold = 3;
    task automatic mw(input logic [15:0] off, input logic [63:0] d, input logic [7:0] be = 8'hFF);
        @(negedge clk);
        bus.addr = BASE | off; bus.write_data = d; bus.byte_en = be; bus.write_DV = 1;
        repeat (hold) @(negedge clk);
        bus.write_DV = 0;
    endtask

    // Two stores back to back with write_DV held high across both (allowed
    // when the addresses differ).
    task automatic mw2(input logic [15:0] o1, input logic [63:0] d1,
                       input logic [15:0] o2, input logic [63:0] d2);
        @(negedge clk);
        bus.byte_en = 8'hFF; bus.write_DV = 1;
        bus.addr = BASE | o1; bus.write_data = d1;
        repeat (hold) @(negedge clk);
        bus.addr = BASE | o2; bus.write_data = d2;
        repeat (hold) @(negedge clk);
        bus.write_DV = 0;
    endtask

    task automatic mr(input logic [15:0] off, output logic [63:0] d);
        @(negedge clk);
        bus.addr = BASE | off; bus.read_DV = 1;
        #1 d = bus.read_data;
        @(negedge clk);
        bus.read_DV = 0;
    endtask

    task automatic check(input string what, input logic [63:0] got, input logic [63:0] exp);
        if (got !== exp) begin
            errors++;
            $display("FAIL %s: got %h exp %h", what, got, exp);
        end
    endtask

    task automatic wait_idle();
        logic [63:0] s;
        int n = 0;
        do begin mr(16'h0010, s); n++; end while (s[0] && n < 100000);
        if (s[0]) begin errors++; $display("FAIL: engine stuck busy"); end
    endtask

    task automatic cmd(input logic [7:0] op, input int len, input logic [23:0] a = 0);
        mw(16'h0008, {40'b0, a});
        mw(16'h0000, {32'b0, 16'(len), 8'h00, op});
        wait_idle();
    endtask

    function automatic logic [7:0] pat(input int i);
        logic [31:0] v;
        v = i ^ 32'h5A;
        return v[7:0];
    endfunction

    function automatic logic [7:0] bufbyte(input logic [63:0] dw[32], input int i);
        return dw[i/8][(i%8)*8 +: 8];
    endfunction

    task automatic read_buf(output logic [63:0] dw[32]);
        for (int i = 0; i < 32; i++) mr(16'h0100 + 16'(i*8), dw[i]);
    endtask

    task automatic wait_wip();
        logic [63:0] dw[32];
        int n = 0;
        do begin cmd(8'h05, 1); read_buf(dw); n++; end while (dw[0][0] && n < 10000);
    endtask

    task automatic expect_reject(input string what, input logic [7:0] op, input int len, input logic [23:0] a);
        logic [63:0] s;
        int n0 = flash.ops_seen;
        cmd(op, len, a);
        mr(16'h0010, s);
        check({what, " rejected"}, s[1], 1'b1);
        check({what, " no flash traffic"}, flash.ops_seen, n0);
    endtask

    logic [63:0] d, dw[32];
    logic [7:0]  e;

    initial begin
        bus.write_DV = 0; bus.read_DV = 0; bus.addr = 0; bus.write_data = 0; bus.byte_en = 0;
        // Preload a "bitstream" and some data.
        for (int i = 0; i < 64; i++) flash.mem[i] = pat(i);
        flash.mem[24'hF0_0010] = 8'h00;
        repeat (5) @(posedge clk);
        rst_l = 1;

        // Before EOS: not ready, commands refused.
        mr(16'h0010, d); check("not ready before EOS", d[2], 0);
        mr(16'h0028, d); check("ID", d, 64'h5153_5049_0000_0001);
        mr(16'h0020, d); check("FLOOR", d, 64'hF0_0000);
        mw(16'h0000, {39'b0, 9'd3, 8'h00, 8'h9F});
        mr(16'h0010, d); check("refused before ready", d[1:0], 2'b10);
        check("no traffic before ready", flash.ops_seen, 0);
        eos = 1;
        repeat (200) @(posedge clk);
        mr(16'h0010, d); check("ready after dummies", d[2], 1);
        check("dummy clocks: no command", flash.ops_seen, 0);

        // RDID
        cmd(8'h9F, 3); mr(16'h0010, d); check("RDID accepted", d[1], 0);
        mr(16'h0100, d); check("RDID", d[23:0], 24'h18_20_01);

        // READ from the bitstream area (allowed)
        cmd(8'h03, 40, 24'h000004);
        read_buf(dw);
        for (int i = 0; i < 40; i++) begin e = pat(i + 4); check($sformatf("READ low byte %0d", i), bufbyte(dw, i), e); end

        // Guard and whitelist
        cmd(8'h06, 0);
        expect_reject("PP below floor",  8'h02, 4, 24'h00_0100);
        expect_reject("SE below floor",  8'hD8, 0, 24'hEF_FFFF);
        expect_reject("P4E below floor", 8'h20, 0, 24'h00_0000);
        expect_reject("bulk erase 60",   8'h60, 0, 24'hF0_0000);
        expect_reject("bulk erase C7",   8'hC7, 0, 24'hF0_0000);
        expect_reject("WRR",             8'h01, 2, 24'hF0_0000);
        expect_reject("BRWR",            8'h17, 1, 24'hF0_0000);
        expect_reject("4PP",             8'h12, 4, 24'hF0_0000);
        expect_reject("4SE",             8'hDC, 0, 24'hF0_0000);
        expect_reject("len 257",         8'h03, 257, 24'hF0_0000);
        expect_reject("len 512",         8'h03, 512, 24'hF0_0000);
        expect_reject("len 513",         8'h03, 513, 24'hF0_0000);
        cmd(8'h04, 0);

        // Erase + program + read back at the floor (256-byte page)
        cmd(8'h06, 0); cmd(8'hD8, 0, 24'hF0_0000);
        mr(16'h0010, d); check("SE accepted", d[1], 0);
        wait_wip();
        cmd(8'h03, 32, 24'hF0_0000); read_buf(dw);
        for (int i = 0; i < 32; i++) check($sformatf("erased %0d", i), bufbyte(dw, i), 8'hFF);
        for (int i = 0; i < 32; i++) mw(16'h0100 + 16'(i*8), {8{8'(i)}} ^ 64'h0706050403020100);
        cmd(8'h06, 0); cmd(8'h02, 256, 24'hF0_0000);
        mr(16'h0010, d); check("PP accepted", d[1], 0);
        wait_wip();
        for (int i = 0; i < 32; i++) mw(16'h0100 + 16'(i*8), 64'h0);
        cmd(8'h03, 256, 24'hF0_0000); read_buf(dw);
        for (int i = 0; i < 256; i++) begin e = pat((i/8) ^ (i%8) ^ 32'h5A); check($sformatf("PP readback %0d", i), bufbyte(dw, i), e); end

        // Partial page at the top of flash, byte-enable buffer writes
        cmd(8'h06, 0); cmd(8'hD8, 0, 24'hFF_0000); wait_wip();
        mw(16'h0100, 64'hAABB_CCDD_EEFF_1122, 8'b0001_1111);   // after wait_wip (RDSR lands in byte 0)
        cmd(8'h06, 0); cmd(8'h02, 5, 24'hFF_FFF0); wait_wip();
        cmd(8'h03, 8, 24'hFF_FFF0); read_buf(dw);
        check("top PP", dw[0], 64'hFFFF_FFDD_EEFF_1122);

        // 4 KB parameter erase above the floor
        cmd(8'h06, 0); cmd(8'h20, 0, 24'hF0_0080); wait_wip();
        cmd(8'h03, 4, 24'hF0_0000); read_buf(dw);
        check("P4E erased", dw[0][31:0], 32'hFFFF_FFFF);

        // CLKDIV 0 is stored as 1 (no 50 MHz); 3 → 12.5 MHz still reads
        mw(16'h0018, 64'h0);
        mr(16'h0018, d); check("CLKDIV 0 clamps to 1", d, 64'h1);
        mw(16'h0018, 64'h3);
        cmd(8'h03, 8, 24'h000000); read_buf(dw);
        for (int i = 0; i < 8; i++) begin e = pat(i); check($sformatf("12.5MHz READ %0d", i), bufbyte(dw, i), e); end
        mw(16'h0018, 64'h1);

        // CMD and ADDR writes while busy are ignored
        mw(16'h0008, 64'h0);
        mw(16'h0000, {39'b0, 9'd200, 8'h00, 8'h03});
        mw(16'h0008, 64'hF0_0000);
        mw(16'h0000, {39'b0, 9'd0, 8'h00, 8'hD8});
        mr(16'h0008, d); check("ADDR write ignored while busy", d, 64'h0);
        wait_idle();
        mr(16'h0000, d); check("CMD while busy ignored", d[7:0], 8'h03);
        mr(16'h0010, d); check("dropped flag set", d[3], 1'b1);
        begin
            int n0 = flash.ops_seen;
            mw(16'h0000, {32'b0, 16'd1, 8'h00, 8'h05});
            mr(16'h0010, d); check("CMD after a dropped write refused", d[3:0], 4'b0110);
            check("refused CMD: no flash traffic", flash.ops_seen, n0);
        end
        cmd(8'h05, 1); mr(16'h0010, d); check("next CMD accepted", d[3:0], 4'b0100);
        // continuous DV: CMD accepted, then an ADDR store under the same DV
        mw(16'h0008, 64'h0);
        mw2(16'h0000, {32'b0, 16'd64, 8'h00, 8'h03}, 16'h0008, 64'hF0_0000);
        wait_idle();
        mr(16'h0008, d); check("held-DV ADDR while busy dropped", d, 64'h0);
        mr(16'h0010, d); check("held-DV dropped flagged", d[3], 1'b1);
        cmd(8'h05, 1); mr(16'h0010, d); check("refused after held-DV drop", d[1], 1'b1);
        // continuous DV: ADDR then CMD (both land; the CMD sees the new ADDR)
        mw2(16'h0008, 64'h000004, 16'h0000, {32'b0, 16'd4, 8'h00, 8'h03});
        wait_idle();
        mr(16'h0010, d); check("held-DV ADDR+CMD accepted", d[3:0], 4'b0100);
        read_buf(dw); e = pat(4); check("held-DV ADDR+CMD read", bufbyte(dw, 0), e);
        // a dropped buffer write also arms the flag
        mw(16'h0000, {32'b0, 16'd64, 8'h00, 8'h03});
        mw(16'h0100, 64'h1234);
        wait_idle();
        mr(16'h0010, d); check("buffer write while busy flagged", d[3], 1'b1);
        cmd(8'h05, 1); mr(16'h0010, d); check("refused after buffer drop", d[1], 1'b1);

        // The bitstream area must be untouched.
        check("no program/erase below floor", flash.low_writes, 0);
        check("protocol errors", flash.protocol_err, 0);
        for (int i = 0; i < 64; i++)
            begin e = pat(i); if (flash.rd(i) != e) begin errors++; $display("FAIL low byte %0d changed", i); end end

        if (errors == 0) $display("TB_QSPI PASS");
        else             $display("TB_QSPI FAIL %0d", errors);
        $finish;
    end

    initial begin #50_000_000; $display("TB_QSPI FAIL timeout"); $finish; end
endmodule

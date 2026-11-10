`timescale 1ns / 1ps
// tb_sd_spi_compat — SPI mode (SD_MODE = 0) of the new sd_spi must be
// bit-for-bit the old SPI-only controller (sd_spi_ref = sd_spi.sv as of
// d9cf6c2).  Both get the same random MMIO traffic (SD_CTRL with [18] = 0,
// SD_DATA kicks, SD_BUF writes with random byte enables, reads anywhere,
// writes into the native window 0x100..0x1F8 — SDM_CMD, CLK_ON, FIFO pushes
// — and program-load pulses), random MISO and card-detect, and the strobe
// held 1..4 cycles, sometimes across two addresses.  Every cycle compares
// SCK, CMD (= MOSI, always driven), DAT3 (= CS_n), DAT1/DAT2 (driven high),
// DAT0 (an input), SD_RESET, and the read data for every offset outside the
// native window (which read 0 on the old design).  Prints "TB_SDSPI PASS".
module tb_sd_spi_compat;
    logic clk = 0, rst_l = 0, load_rst = 0;
    always #5 clk = ~clk;

    mmio_if bus_n();
    mmio_if bus_r();
    logic       miso = 1, cd = 0;
    wire        sck_n, reset_n_n, irq_n, cmd_o, cmd_oe;
    wire [3:0]  dat_o, dat_oe;
    wire        sck_r, reset_n_r, mosi_r, cs_n_r, dat1_r, dat2_r;

    sd_spi dut (
        .i_Clk(clk), .i_Rst_L(rst_l), .i_load_rst(load_rst), .mmio(bus_n), .o_irq(irq_n),
        .i_sd_cd(cd), .o_sd_reset_n(reset_n_n), .o_sd_sck(sck_n),
        .o_sd_cmd(cmd_o), .o_sd_cmd_oe(cmd_oe), .i_sd_cmd(cmd_o),
        .o_sd_dat(dat_o), .o_sd_dat_oe(dat_oe), .i_sd_dat({dat_o[3:1], miso})
    );
    sd_spi_ref ref_i (
        .i_Clk(clk), .i_Rst_L(rst_l), .mmio(bus_r),
        .i_sd_cd(cd), .o_sd_reset_n(reset_n_r), .o_sd_sck(sck_r), .o_sd_mosi(mosi_r),
        .i_sd_miso(miso), .o_sd_cs_n(cs_n_r), .o_sd_dat1(dat1_r), .o_sd_dat2(dat2_r)
    );

    // one stimulus, both buses
    logic        s_wr = 0, s_rd = 0;
    logic [31:0] s_addr = 0;
    logic [63:0] s_wdata = 0;
    logic [7:0]  s_be = 0;
    always_comb begin
        bus_n.write_DV = s_wr; bus_n.read_DV = s_rd; bus_n.addr = s_addr; bus_n.write_data = s_wdata; bus_n.byte_en = s_be;
        bus_r.write_DV = s_wr; bus_r.read_DV = s_rd; bus_r.addr = s_addr; bus_r.write_data = s_wdata; bus_r.byte_en = s_be;
    end

    int errors = 0, cycles = 0, reads_cmp = 0, kicks = 0, sck_edges = 0;
    wire native_win = (s_addr[15:9] == 7'd0) && s_addr[8];
    always @(negedge clk) if (rst_l) begin
        cycles++;
        if ({sck_n, cmd_o, cmd_oe, dat_o[3], dat_oe[3], dat_o[2:1], dat_oe[2:1], dat_oe[0], reset_n_n}
            !== {sck_r, mosi_r, 1'b1, cs_n_r, 1'b1, dat2_r, dat1_r, 2'b11, 1'b0, reset_n_r}) begin
            errors++;
            if (errors < 10)
                $display("[%0t] PIN MISMATCH new sck %b cmd %b/%b dat %b/%b rst %b | ref sck %b mosi %b cs %b d1 %b d2 %b rst %b",
                         $time, sck_n, cmd_o, cmd_oe, dat_o, dat_oe, reset_n_n, sck_r, mosi_r, cs_n_r, dat1_r, dat2_r, reset_n_r);
        end
        if (!native_win) begin
            reads_cmp++;
            if (bus_n.read_data !== bus_r.read_data) begin
                errors++;
                if (errors < 10) $display("[%0t] READ MISMATCH @%h new %h ref %h", $time, s_addr, bus_n.read_data, bus_r.read_data);
            end
        end
    end
    always @(posedge sck_r) sck_edges++;

    function automatic logic [15:0] rand_off();
        int r = $urandom_range(0, 99);
        if (r < 25) return 16'h0000;
        if (r < 45) return 16'h0008;
        if (r < 60) return 16'h0010;
        if (r < 75) return 16'h0200 + 16'($urandom_range(0, 63) * 8);
        if (r < 88) return 16'h0100 + 16'($urandom_range(0, 31) * 8);       // native window
        return 16'($urandom_range(0, 16'hFFFF)) & 16'hFFF8;                  // anywhere
    endfunction
    function automatic logic [63:0] rand_wdata(input logic [15:0] off);
        logic [63:0] v = {$urandom, $urandom};
        if (off == 16'h0000) begin
            v[18] = 1'b0;                                   // SPI mode only
            if ($urandom_range(0, 3) != 0) v[15:0] = 16'($urandom_range(0, 3));   // fast SCK, mostly
        end
        return v;
    endfunction

    initial begin
        repeat (5) @(posedge clk);
        rst_l = 1;
        for (int n = 0; n < 40000; n++) begin
            logic [15:0] off;
            int hold;
            @(negedge clk);
            miso = $urandom; if ($urandom_range(0, 50) == 0) cd = ~cd;
            if ($urandom_range(0, 400) == 0) begin load_rst = 1; @(negedge clk); load_rst = 0; end
            off  = rand_off();
            hold = $urandom_range(1, 4);
            s_addr = 32'hF000_0000 | off; s_be = 8'($urandom); if ($urandom_range(0, 1)) s_be = 8'hFF;
            if ($urandom_range(0, 1)) begin
                s_wdata = rand_wdata(off); s_wr = 1;
                if (off == 16'h0008) kicks++;
                repeat (hold) @(negedge clk);
                if ($urandom_range(0, 4) == 0) begin           // DV held into a second address
                    off = rand_off(); s_addr = 32'hF000_0000 | off; s_wdata = rand_wdata(off);
                    repeat (hold) @(negedge clk);
                end
                s_wr = 0;
            end else begin
                s_rd = 1;
                repeat (hold) @(negedge clk);
                s_rd = 0;
            end
            if ($urandom_range(0, 3) == 0) repeat ($urandom_range(1, 40)) @(negedge clk);   // let bytes shift
        end
        $display("compared %0d cycles, %0d read samples, %0d SD_DATA writes, %0d SCK edges",
                 cycles, reads_cmp, kicks, sck_edges);
        if (errors == 0 && sck_edges > 1000) $display("TB_SDSPI PASS");
        else $display("TB_SDSPI FAIL %0d", errors);
        $finish;
    end
endmodule

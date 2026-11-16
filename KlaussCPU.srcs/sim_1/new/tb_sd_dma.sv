`timescale 1ns / 1ps
// tb_sd_dma — unit test for the SD DMA engine (sd_native, SD DMA spec v1.2)
// against sd_card_model and a behavioural DDR port with random grant and
// ready latencies (+SEED=<n>).  Self-checking; prints "TB_SDDMA PASS" or
// "TB_SDDMA FAIL n".  No waveform dump.  tools/sdsim/run_sd_tb.sh dma.
//
// The DDR model plays mem_read_write's arbiter for master E: the grant
// after a random delay while req is up, one transaction per tenure (DV
// only while granted, the address held), ready after a random latency
// (a write lands in `ddr` on its ready: accepted), release on done.  It
// checks the contract (no DV without the grant, req held until done, no
// address outside DDR) and counts accesses.
//
// (m1) is checked on every completion and abort path the same way
// (m1_check): once an MMIO read shows DMA_BUSY = 0, the DDR port must have
// nothing in flight, no access may follow until the next DMA command, and
// DDR must already hold every word the transfer moved (the caller's
// reference).  A monitor also checks every cycle that DMA_BUSY = 0 implies
// an idle port.
module tb_sd_dma;
    logic clk = 0, rst_l = 0, load_rst = 0;
    always #5 clk = ~clk;

    mmio_if bus();
    wire        sck, reset_n, irq;
    wire        cmd_o, cmd_oe;
    wire [3:0]  dat_o, dat_oe;
    wire        sd_cmd;
    wire [3:0]  sd_dat;
    logic       cd = 1'b0;

    assign sd_cmd = cmd_oe ? cmd_o : 1'bz;
    for (genvar i = 0; i < 4; i++) begin : g_d
        assign sd_dat[i] = dat_oe[i] ? dat_o[i] : 1'bz;
    end
    pullup (sd_cmd);
    pullup (sd_dat[0]); pullup (sd_dat[1]); pullup (sd_dat[2]); pullup (sd_dat[3]);

    // DDR port
    wire         d_req, d_done, d_wdv, d_rdv;
    wire [31:0]  d_addr;
    wire [127:0] d_wdata;
    wire [15:0]  d_mask;
    logic [127:0] d_rdata = '0;
    logic        d_ready = 1'b0, d_grant = 1'b0;

    sd_spi dut (
        .i_Clk(clk), .i_Rst_L(rst_l), .i_load_rst(load_rst), .mmio(bus), .o_irq(irq),
        .i_sd_cd(cd), .o_sd_reset_n(reset_n), .o_sd_sck(sck),
        .o_sd_cmd(cmd_o), .o_sd_cmd_oe(cmd_oe), .i_sd_cmd(sd_cmd),
        .o_sd_dat(dat_o), .o_sd_dat_oe(dat_oe), .i_sd_dat(sd_dat),
        .o_dma_req(d_req), .o_dma_done(d_done), .o_dma_write_DV(d_wdv), .o_dma_read_DV(d_rdv),
        .o_dma_addr(d_addr), .o_dma_write_data(d_wdata), .o_dma_wdf_mask(d_mask),
        .i_dma_read_data(d_rdata), .i_dma_ready(d_ready), .i_dma_grant(d_grant)
    );
    sd_card_model card (.clk(sck), .pwr(!reset_n), .cmd(sd_cmd), .dat(sd_dat));

    localparam logic [31:0] BASE = 32'hF000_0000;
    localparam logic [15:0] SD_CTRL = 16'h000, SDM_CMD = 16'h100, SDM_ARG = 16'h108,
        SDM_RESP0 = 16'h110, SDM_BLK = 16'h120, SDM_STATUS = 16'h128,
        SDM_IRQ_EN = 16'h130, SDM_CFG = 16'h140, SDM_FIFO = 16'h148,
        SDM_DMA = 16'h150, SDM_BLKDONE = 16'h158, SDM_DATA = 16'h180, SDM_CAPS = 16'h1F8;
    localparam int B_CMD_BUSY = 0, B_DATA_BUSY = 1, B_THRESH = 4, B_DMA_BUSY = 5,
        B_CMD_DONE = 8, B_DATA_DONE = 9, B_DAT_CRC = 21, B_WR_CRC = 23, B_FIFO_ERR = 26,
        B_DMA_ERR = 27, B_ERR_ANY = 31;
    localparam logic [63:0] ERR_MASK = 64'h0FFF_0000;
    localparam int RT_NONE = 0, RT_R1 = 1, RT_R1B = 2, RT_R2 = 3, RT_R3 = 4;
    localparam longint DDR_END = 64'h0800_0000;
    localparam logic [63:0] CFG_W4 = 64'h1, CFG_DMA = 64'h1_0000;

    int errors = 0, checks = 0;
    bit verbose = 0;
    longint ncyc = 0;
    always @(posedge clk) ncyc++;

    // ======================================================== DDR model
    logic [63:0] ddr [longint];          // dword at byte address a (a % 8 == 0)
    function automatic logic [63:0] dget(input longint a);
        return ddr.exists(a) ? ddr[a] : {32'hDEAD_0000 | 32'(a >> 3), 32'(a)};
    endfunction
    int gmax = 6, lmax = 12;             // grant delay / ready latency, 0..max
    longint n_wr = 0, n_rd = 0, last_acc = 0;
    longint wr_lo = -1, wr_hi = -1;      // DDR byte range written (this test)
    int     xfer_open = 0;               // granted transactions not yet released
    logic   [31:0] a_q;
    bit     dv_q = 0;
    int     gcnt = 0, lcnt = 0;
    int     dprot = 0;                   // protocol violations
    task automatic dbad(input string s);
        dprot++; errors++;
        if (dprot < 6) $display("[%0t] FAIL DDR port: %s", $time, s);
    endtask
    always @(posedge clk) begin
        d_ready <= 1'b0;
        // grant
        if (!d_grant) begin
            if (d_req) begin
                if (gcnt == 0) begin d_grant <= 1'b1; gcnt <= $urandom_range(0, gmax); end
                else gcnt <= gcnt - 1;
            end
        end else if (d_done) begin
            d_grant <= 1'b0;
            if (d_req) dbad("req still up with done");
        end else if (!d_req) dbad("req dropped without done");
        if ((d_wdv || d_rdv) && !d_grant) dbad("DV without the grant");
        if (d_wdv && d_rdv) dbad("both DVs");
        if (d_done && (d_wdv || d_rdv)) dbad("done with DV up");
        // a transaction
        if ((d_wdv || d_rdv) && d_grant && !d_ready) begin
            if (dv_q && d_addr !== a_q) dbad("address changed under DV");
            if (d_addr[3:0] != 4'h0) dbad("unaligned address");
            if (longint'(d_addr) >= DDR_END) dbad($sformatf("address %08x outside DDR", d_addr));
            if (lcnt == 0) begin
                d_ready <= 1'b1;
                lcnt <= $urandom_range(0, lmax);
                last_acc = ncyc;
                if (d_wdv) begin
                    logic [63:0] hi, lo;
                    hi = dget(d_addr); lo = dget(longint'(d_addr) + 8);
                    for (int i = 0; i < 8; i++) begin
                        if (!d_mask[8 + i]) hi[i*8 +: 8] = d_wdata[64 + i*8 +: 8];
                        if (!d_mask[i])     lo[i*8 +: 8] = d_wdata[i*8 +: 8];
                    end
                    if (d_mask[15:8] != 8'hFF) begin
                        ddr[longint'(d_addr)] = hi;
                        if (wr_lo < 0 || d_addr < wr_lo) wr_lo = d_addr;
                        if (longint'(d_addr) + 8 > wr_hi) wr_hi = longint'(d_addr) + 8;
                    end
                    if (d_mask[7:0] != 8'hFF) begin
                        ddr[longint'(d_addr) + 8] = lo;
                        if (wr_lo < 0 || longint'(d_addr) + 8 < wr_lo) wr_lo = longint'(d_addr) + 8;
                        if (longint'(d_addr) + 16 > wr_hi) wr_hi = longint'(d_addr) + 16;
                    end
                    if (!((d_mask == 16'h0) || (d_mask == 16'h00FF) || (d_mask == 16'hFF00)))
                        dbad($sformatf("odd mask %h", d_mask));
                    n_wr++;
                end else begin
                    d_rdata <= {dget(d_addr), dget(longint'(d_addr) + 8)};
                    n_rd++;
                end
            end else lcnt <= lcnt - 1;
        end
        dv_q <= (d_wdv || d_rdv) && !d_ready;
        a_q  <= d_addr;
    end
    // (m1), every cycle: DMA_BUSY = 0 means nothing in flight on the port
    always @(posedge clk)
        if (rst_l && !dut.native_i.w_dma_busy && (d_req || d_wdv || d_rdv)) dbad("DMA_BUSY = 0 with the port busy");

    // the 4 KiB read-ahead limit of a write DMA: words fetched into the FIFO
    // and not yet sent to the card (the data engine's pops) never exceed 512
    longint fetched = 0, sent = 0, ahead_max = 0;
    always @(posedge clk) begin
        if (dut.native_i.w_dma_push) fetched++;
        if (dut.native_i.w_eng_pop) sent++;
        if (fetched - sent > ahead_max) ahead_max = fetched - sent;
        if (dut.native_i.w_fifo_rs) begin fetched = 0; sent = 0; end
    end

    // ======================================================== MMIO
    task automatic mw(input logic [15:0] off, input logic [63:0] d);
        @(negedge clk);
        bus.addr = BASE | off; bus.write_data = d; bus.byte_en = 8'hFF; bus.write_DV = 1;
        repeat (3) @(negedge clk);
        bus.write_DV = 0;
    endtask
    task automatic mr(input logic [15:0] off, output logic [63:0] d);
        @(negedge clk);
        bus.addr = BASE | off; bus.read_DV = 1;
        @(negedge clk);
        d = bus.read_data;
        repeat (2) @(negedge clk);
        bus.read_DV = 0;
    endtask
    task automatic check(input string what, input logic [63:0] got, input logic [63:0] exp);
        checks++;
        if (got !== exp) begin errors++; $display("[%0t] FAIL %s: got %h exp %h", $time, what, got, exp); end
    endtask
    task automatic check_true(input string what, input bit c);
        checks++;
        if (!c) begin errors++; $display("[%0t] FAIL %s", $time, what); end
    endtask

    logic [63:0] st, d, r0;
    logic [15:0] rca;

    task automatic status(output logic [63:0] s); mr(SDM_STATUS, s); endtask
    task automatic clr_status(); mw(SDM_STATUS, 64'hFFFF_FFFF); endtask
    task automatic cmd_start(input int idx, input logic [31:0] arg, input int rt,
                             input bit data = 0, input bit wr = 0, input bit stop = 0);
        mw(SDM_ARG, {32'h0, arg});
        mw(SDM_CMD, {50'h0, stop, wr, data, 3'(rt), 2'b00, 6'(idx)});
    endtask
    task automatic wait_cmd(output logic [63:0] s);
        longint t0 = ncyc;
        do status(s); while (s[B_CMD_BUSY] && ncyc - t0 < 2_000_000);
        if (s[B_CMD_BUSY]) begin errors++; $display("[%0t] FAIL command stuck busy", $time); end
    endtask
    task automatic do_cmd(input string what, input int idx, input logic [31:0] arg, input int rt,
                          output logic [63:0] s);
        clr_status();
        cmd_start(idx, arg, rt);
        wait_cmd(s);
        mr(SDM_RESP0, r0);
        check({what, " CMD_DONE"}, s[B_CMD_DONE], 1);
        check({what, " no error"}, s & ERR_MASK, 0);
    endtask
    task automatic set_blk(input int bytes, input int count);
        mw(SDM_BLK, {32'h0, 16'(count), 4'h0, 12'(bytes)});
    endtask
    task automatic fifo_reset(); mw(SDM_FIFO, 64'h8000_0000); endtask
    task automatic wait_card_idle();
        longint t0 = ncyc;
        while ((card.rd_active || card.wr_active || card.cmd_oe || card.dat_oe != 0) && ncyc - t0 < 4_000_000) @(negedge clk);
        repeat (20) @(negedge clk);
    endtask
    task automatic dump();
        $display("  d_state=%0d c_state=%0d m_st=%0d dm_ph=%b dm_on=%b dm_wr=%b nblk=%0d a_st=%0d level=%0d used=%0d dd_pend=%b blkdone=%0d ahead_max=%0d card.st=%0d wr_active=%b",
                 dut.native_i.r_d_state, dut.native_i.r_c_state, dut.native_i.r_m_st, dut.native_i.r_dm_ph,
                 dut.native_i.r_dm_on, dut.native_i.r_dm_wr, dut.native_i.r_dm_nblk, dut.native_i.r_a_st,
                 dut.native_i.w_level, dut.native_i.w_used, dut.native_i.r_dd_pend, dut.native_i.r_blkdone,
                 ahead_max, card.st, card.wr_active);
    endtask
    // wait for a status bit to read `v`
    task automatic wait_bit(input string what, input int b, input bit v, output logic [63:0] s,
                            input longint limit = 4_000_000);
        longint t0 = ncyc;
        do status(s); while (s[b] != v && ncyc - t0 < limit);
        if (s[b] != v) begin
            errors++; $display("[%0t] FAIL %s: status bit %0d stuck at %b (%h)", $time, what, b, !v, s);
            dump();
        end
    endtask

    function automatic logic [63:0] card_word(input longint byteaddr);
        logic [63:0] w;
        for (int k = 0; k < 8; k++) w[8*k +: 8] = card.rd_byte(byteaddr + k);
        return w;
    endfunction
    function automatic logic [63:0] wpat(input longint a, input int seed);
        return {32'(a * 2654435761 + seed), 32'(32'h5A5A_0000 ^ a ^ (seed << 16))};
    endfunction

    // DDR: fill [a, a + n) with the pattern (as the CPU's data before a write DMA)
    task automatic ddr_fill(input longint a, input int bytes, input int seed);
        for (longint x = a; x < a + bytes; x += 8) ddr[x] = wpat(x, seed);
    endtask
    task automatic ddr_clear_marks(); wr_lo = -1; wr_hi = -1; endtask
    // DDR [a, a + n blocks of bsz) must hold the card's blocks blk.. (card
    // block stride 512); bytes outside it untouched (wr_lo / wr_hi)
    task automatic check_ddr_read(input string what, input longint a, input longint blk, input int n, input int bsz);
        int bad = 0;
        for (int b = 0; b < n; b++)
            for (int k = 0; k < bsz / 8; k++)
                if (dget(a + b * bsz + k * 8) !== card_word((blk + b) * 512 + k * 8)) begin
                    bad++;
                    if (bad < 4) $display("FAIL %s: block %0d word %0d ddr %h card %h", what, b, k,
                                          dget(a + b * bsz + k * 8), card_word((blk + b) * 512 + k * 8));
                end
        checks++; if (bad) errors++;
        if (n > 0) check_true({what, ": DDR written only inside the transfer"},
                              wr_lo >= a && wr_hi <= a + n * bsz);
        else check_true({what, ": DDR not written"}, wr_lo < 0);
    endtask
    task automatic check_card_written(input string what, input longint a, input longint blk, input int n,
                                      input int bsz, input int seed);
        int bad = 0;
        for (int b = 0; b < n; b++)
            for (int k = 0; k < bsz / 8; k++)
                if (card_word((blk + b) * 512 + k * 8) !== wpat(a + b * bsz + k * 8, seed)) begin
                    bad++;
                    if (bad < 4) $display("FAIL %s: block %0d word %0d card %h exp %h", what, b, k,
                                          card_word((blk + b) * 512 + k * 8), wpat(a + b * bsz + k * 8, seed));
                end
        checks++; if (bad) errors++;
    endtask

    // (m1): DMA_BUSY reads 0 -> the port is idle now and stays idle, and
    // the expected DDR data is already there (checked by the caller right
    // after this returns, before any further cycle could change it)
    task automatic m1_check(input string what);
        logic [63:0] s;
        longint t;
        wait_bit({what, " DMA_BUSY drops"}, B_DMA_BUSY, 0, s);
        t = ncyc;
        check_true({what, " (m1): port idle at DMA_BUSY = 0"}, !d_req && !d_wdv && !d_rdv && !d_grant);
        repeat (400) @(negedge clk);
        check_true({what, " (m1): no access after DMA_BUSY = 0"}, last_acc <= t);
    endtask

    // CMD12 with [13] (stop): wait until it is done (R1b busy included)
    task automatic stop_cmd(output logic [63:0] s);
        clr_status();
        cmd_start(12, 0, RT_R1B, 0, 0, 1);
        wait_cmd(s);
    endtask
    // a DMA data command: DMA_ADDR, size, count, then the command
    task automatic dma_cmd(input int idx, input longint blk, input bit wr, input longint a,
                           input int bsz, input int cnt, input bit w4 = 1);
        fifo_reset();
        clr_status();
        mw(SDM_DMA, a);
        set_blk(bsz, cnt);
        mw(SDM_CFG, (w4 ? CFG_W4 : 0) | CFG_DMA);
        ddr_clear_marks();
        if (verbose) $display("[%0t] DMA CMD%0d blk %0d %s %h %0d x %0d (host_err %0d)", $time, idx, blk,
                              wr ? "from" : "to", a, cnt, bsz, card.host_err);
        cmd_start(idx, 32'(blk), RT_R1, 1, wr);
    endtask

    int seed = 1;
    longint nw0, nr0, t0;
    int     n0, bw0;

    initial begin
        void'($value$plusargs("SEED=%d", seed));
        verbose = $test$plusargs("VERBOSE");
        card.verbose = $test$plusargs("CARDV");
        process::self().srandom(seed);
        bus.write_DV = 0; bus.read_DV = 0; bus.addr = 0; bus.write_data = 0; bus.byte_en = 0;
        repeat (5) @(posedge clk);
        rst_l = 1;
        repeat (3) @(posedge clk);

        // ============================================== registers (idle)
        mr(SDM_CAPS, d);    check("SDM_CAPS [10] DMA", d, 64'h0008_0501);
        mr(SDM_DMA, d);     check("SDM_DMA_ADDR reset", d, 0);
        mw(SDM_DMA, 64'hFFFF_FFFF_FFFF_FFFF); mr(SDM_DMA, d); check("SDM_DMA_ADDR 64-bit, [2:0] read 0", d, 64'hFFFF_FFFF_FFFF_FFF8);
        mw(SDM_DMA, 64'h0000_0001_2345_678F); mr(SDM_DMA, d); check("SDM_DMA_ADDR", d, 64'h0000_0001_2345_6788);
        mw(SDM_CFG, 64'hFFFF_FFFF_FFFF_FFFF); mr(SDM_CFG, d); check("SDM_CFG [16] DMA_EN (and [1], [7:4] read 0)", d, 64'h1_0005);
        mw(SDM_CFG, 0);
        mw(SDM_IRQ_EN, 64'hFFFF_FFFF_FFFF_FFFF); mr(SDM_IRQ_EN, d); check("SDM_IRQ_EN [27]", d, 64'h0FFF_0310);
        mw(SDM_IRQ_EN, 0);
        status(st); check("idle: DMA_BUSY 0, DMA_ERR 0", st & 64'h0800_0020, 0);

        // ============================================== card: identification, 4-bit, 25 MHz
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd9);
        repeat (100) @(posedge clk);
        mw(SDM_CFG, 64'h4); repeat (2000) @(posedge clk); mw(SDM_CFG, 0);
        repeat (200) @(posedge clk);
        do_cmd("CMD0", 0, 0, RT_NONE, st);
        do_cmd("CMD8", 8, 32'h1AA, RT_R1, st);
        for (int i = 0; i < 6; i++) begin
            do_cmd("CMD55", 55, 0, RT_R1, st);
            do_cmd("ACMD41", 41, 32'h40FF_8000, RT_R3, st);
            if (r0[31]) break;
        end
        do_cmd("CMD2", 2, 0, RT_R2, st);
        do_cmd("CMD3", 3, 0, RT_R1, st);
        rca = r0[31:16];
        do_cmd("CMD7", 7, {rca, 16'h0}, RT_R1B, st);
        do_cmd("CMD55", 55, {rca, 16'h0}, RT_R1, st);
        do_cmd("ACMD6", 6, 2, RT_R1, st);
        mw(SDM_CFG, CFG_W4);
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd1);
        check("card in tran, 4-bit", {card.st, card.w4}, {32'd4, 1'b1});

        // ============================================== counted read, CMD17 / CMD18, both lanes
        for (int lane = 0; lane < 2; lane++) begin
            automatic longint a = 64'h0010_0000 + lane * 8;
            automatic string w = $sformatf("CMD18 x4 to DDR @%h", a);
            dma_cmd(18, 1000, 0, a, 512, 4);
            wait_bit({w, " DATA_DONE"}, B_DATA_DONE, 1, st);
            // DATA_DONE only once the last block is in DDR
            check_ddr_read({w, " at DATA_DONE"}, a, 1000, 4, 512);
            check({w, " DATA_BUSY with DATA_DONE"}, st[B_DATA_BUSY], 0);
            check({w, " no error"}, st & ERR_MASK, 0);
            m1_check(w);
            mr(SDM_DMA, d);     check({w, " DMA_ADDR advanced"}, d, a + 2048);
            mr(SDM_BLKDONE, d); check({w, " BLKDONE"}, d, 4);
            mr(SDM_FIFO, d);    check({w, " FIFO empty"}, d, 0);
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end
        dma_cmd(17, 77, 0, 64'h0020_0008, 512, 1);
        wait_bit("CMD17 DATA_DONE", B_DATA_DONE, 1, st);
        check_ddr_read("CMD17 at DATA_DONE", 64'h0020_0008, 77, 1, 512);
        m1_check("CMD17");
        wait_card_idle();

        // ============================================== counted write, CMD24 / CMD25
        for (int lane = 0; lane < 2; lane++) begin
            automatic longint a = 64'h0030_0000 + lane * 8;
            automatic string w = $sformatf("CMD25 x5 from DDR @%h", a);
            ddr_fill(a, 5 * 512, 7 + lane);
            nw0 = n_wr;
            dma_cmd(25, 2000 + lane * 10, 1, a, 512, 5);
            wait_bit({w, " DATA_DONE"}, B_DATA_DONE, 1, st);
            check({w, " DMA_BUSY drops with DATA_DONE"}, st[B_DMA_BUSY], 0);
            check({w, " no error"}, st & ERR_MASK, 0);
            m1_check(w);
            check_card_written(w, a, 2000 + lane * 10, 5, 512, 7 + lane);
            check({w, " no DDR write"}, n_wr - nw0, 0);
            mr(SDM_DMA, d);     check({w, " DMA_ADDR advanced"}, d, a + 5 * 512);
            mr(SDM_BLKDONE, d); check({w, " BLKDONE"}, d, 5);
            check_true({w, " fetched exactly size x count"}, fetched == 5 * 64);
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end
        ddr_fill(64'h0031_0000, 512, 3);
        dma_cmd(24, 2100, 1, 64'h0031_0000, 512, 1);
        wait_bit("CMD24 DATA_DONE", B_DATA_DONE, 1, st);
        m1_check("CMD24");
        check_card_written("CMD24", 64'h0031_0000, 2100, 1, 512, 3);
        wait_card_idle();

        // ============================================== short blocks (CMD16), 8 and 64 bytes
        for (int i = 0; i < 2; i++) begin
            automatic int bs = i ? 64 : 8;
            automatic string w = $sformatf("CMD18 %0d-byte blocks x6", bs);
            do_cmd("CMD16", 16, bs, RT_R1, st);
            dma_cmd(18, 3000, 0, 64'h0040_0008, bs, 6);
            wait_bit({w, " DATA_DONE"}, B_DATA_DONE, 1, st);
            m1_check(w);
            for (int b = 0; b < 6; b++)
                for (int k = 0; k < bs / 8; k++)
                    check({w, " data"}, dget(64'h0040_0008 + b * bs + k * 8), card_word((3000 + b) * 512 + k * 8));
            check_true({w, " DDR range"}, wr_lo == 64'h0040_0008 && wr_hi == 64'h0040_0008 + 6 * bs);
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
            ddr_fill(64'h0041_0000, 6 * bs, 20 + i);
            dma_cmd(25, 3100, 1, 64'h0041_0000, bs, 6);
            wait_bit({w, " write DATA_DONE"}, B_DATA_DONE, 1, st);
            m1_check({w, " write"});
            for (int b = 0; b < 6; b++)
                for (int k = 0; k < bs / 8; k++)
                    check({w, " write data"}, card_word((3100 + b) * 512 + k * 8), wpat(64'h0041_0000 + b * bs + k * 8, 20 + i));
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end
        do_cmd("CMD16", 16, 512, RT_R1, st);

        // ============================================== up-front refusals: DMA_ERR, nothing sent
        begin
            automatic string w;
            // a block size not a multiple of 8
            n0 = card.n_cmds; nw0 = n_wr; nr0 = n_rd;
            fifo_reset(); clr_status();
            mw(SDM_DMA, 64'h0010_0000); set_blk(516, 1); mw(SDM_CFG, CFG_W4 | CFG_DMA);
            cmd_start(17, 5, RT_R1, 1, 0);
            repeat (2000) @(negedge clk);
            status(st);
            check("size 516: DMA_ERR, ERR_ANY, no CMD_DONE, not busy", st & 64'h8800_0323, 64'h8800_0000);
            check("size 516: no card command", card.n_cmds - n0, 0);
            check("size 516: no DDR access", (n_wr - nw0) + (n_rd - nr0), 0);
            mw(SDM_STATUS, 64'h0800_0000); status(st); check("DMA_ERR W1C", st[B_DMA_ERR], 0);
            check("ERR_ANY clears", st[B_ERR_ANY], 0);
            // the same size by PIO is fine (DMA_EN = 0)
            // ranges: end past DDR, end exactly at DDR's end, ADDR beyond 4 GiB
            for (int c = 0; c < 4; c++) begin
                automatic longint a = c == 0 ? DDR_END - 4 * 512 + 8 : c == 1 ? DDR_END - 4 * 512 :
                                      c == 2 ? 64'h1_0000_0000 : DDR_END;
                automatic bit ok = (c == 1);
                w = $sformatf("range %h + 4 x 512", a);
                n0 = card.n_cmds; nw0 = n_wr; nr0 = n_rd;
                fifo_reset(); clr_status();
                mw(SDM_DMA, a); set_blk(512, 4); mw(SDM_CFG, CFG_W4 | CFG_DMA);
                ddr_clear_marks();
                cmd_start(18, 4000, RT_R1, 1, 0);
                if (ok) begin
                    wait_bit({w, " DATA_DONE"}, B_DATA_DONE, 1, st);
                    m1_check(w);
                    check_ddr_read(w, a, 4000, 4, 512);
                    do_cmd("CMD12", 12, 0, RT_R1B, st);
                    wait_card_idle();
                end else begin
                    repeat (2000) @(negedge clk);
                    status(st);
                    check({w, ": refused"}, st & 64'h0800_0323, 64'h0800_0000);
                    check({w, ": no card command"}, card.n_cmds - n0, 0);
                    check({w, ": no DDR access"}, (n_wr - nw0) + (n_rd - nr0), 0);
                    // a refused write the same
                    clr_status(); n0 = card.n_cmds;
                    cmd_start(25, 4000, RT_R1, 1, 1);
                    repeat (2000) @(negedge clk);
                    status(st);
                    check({w, ": write refused"}, st & 64'h0800_0323, 64'h0800_0000);
                    check({w, ": write: no card command"}, card.n_cmds - n0, 0);
                    check({w, ": write: no DDR access"}, (n_wr - nw0) + (n_rd - nr0), 0);
                end
            end
            // DMA_ADDR, size and count written just before SDM_CMD (back to
            // back, the range check's pipeline): refused / accepted correctly
            fifo_reset(); clr_status(); n0 = card.n_cmds;
            mw(SDM_CFG, CFG_W4 | CFG_DMA); mw(SDM_DMA, 64'h0010_0000); set_blk(512, 1);
            mw(SDM_BLK, {32'h0, 16'd2, 4'h0, 12'd512});          // count 2 ...
            mw(SDM_DMA, DDR_END - 512);                             // ... then the address: refused
            cmd_start(18, 4100, RT_R1, 1, 0);
            repeat (500) @(negedge clk);
            status(st);
            check("back-to-back registers: refused", st & 64'h0800_0300, 64'h0800_0000);
            check("back-to-back registers: no command", card.n_cmds - n0, 0);
        end

        // ============================================== open-ended read: per-block range check
        begin
            automatic longint a = DDR_END - 3 * 512;       // blocks 0..2 fit, block 3 does not
            automatic string w = "open-ended read into DDR's end";
            n0 = card.n_cmds;
            dma_cmd(18, 5000, 0, a, 512, 0);
            wait_bit({w, " DMA_ERR"}, B_DMA_ERR, 1, st);
            m1_check(w);
            check_ddr_read(w, a, 5000, 3, 512);
            mr(SDM_DMA, d); check({w, " DMA_ADDR = DDR end"}, d, DDR_END);
            // the card side goes on through the FIFO (PIO): block 3 is there
            t0 = ncyc;
            do mr(SDM_FIFO, d); while (d < 64 && ncyc - t0 < 100000);
            check_true({w, ": the next block reaches the FIFO"}, d >= 64);
            status(st); check({w, ": FIFO_THRESH (PIO again)"}, st[B_THRESH], 1);
            for (int k = 0; k < 64; k++) begin
                mr(SDM_DATA, d);
                if (k < 3) check({w, ": PIO pop of block 3"}, d, card_word(5003 * 512 + k * 8));
            end
            stop_cmd(st);
            check({w, " stop DATA_DONE"}, st[B_DATA_DONE], 1);
            wait_card_idle();
        end

        // ============================================== open-ended write: 4 KiB ahead at most, CMD12
        begin
            automatic longint a = 64'h0050_0000;
            automatic string w = "open-ended write";
            ddr_fill(a, 40 * 512, 31);
            ahead_max = 0;
            card.wr_busy = 200;                            // a slow card: the FIFO fills
            dma_cmd(25, 6000, 1, a, 512, 0);
            bw0 = card.blocks_written; t0 = ncyc;
            while (card.blocks_written - bw0 < 12 && ncyc - t0 < 3_000_000) @(negedge clk);
            check_true($sformatf("%s: at most 4 KiB ahead of the card (%0d words)", w, ahead_max), ahead_max <= 512 && ahead_max >= 448);
            dump();
            stop_cmd(st);
            check({w, " stop DATA_DONE"}, st[B_DATA_DONE], 1);
            m1_check(w);
            mr(SDM_BLKDONE, n0);
            mr(SDM_DMA, d);
            check_true({w, ": DMA_ADDR = what was fetched"}, d == a + fetched * 8 && fetched % 64 == 0);
            check_card_written(w, a, 6000, n0, 512, 31);
            card.wr_busy = 6;
            fifo_reset();
            wait_card_idle();
            // open-ended write that runs into DDR's end: DMA_ERR, nothing fetched past it
            a = DDR_END - 2 * 512;
            ddr_fill(a, 2 * 512, 32);
            nr0 = n_rd; bw0 = card.blocks_written;
            dma_cmd(25, 6100, 1, a, 512, 0);
            wait_bit("open-ended write into DDR's end: DMA_ERR", B_DMA_ERR, 1, st);
            m1_check("open-ended write into DDR's end");
            check("open-ended write into DDR's end: fetched 2 blocks", n_rd - nr0, 2 * 32);
            mr(SDM_DMA, d); check("open-ended write: DMA_ADDR = DDR end", d, DDR_END);
            t0 = ncyc; while (card.blocks_written - bw0 < 2 && ncyc - t0 < 1_000_000) @(negedge clk);
            repeat (3000) @(negedge clk);
            status(st); check("open-ended write: card waits for PIO (DATA_BUSY, THRESH)", st & 64'h12, 64'h12);
            stop_cmd(st);
            check("open-ended write stop", st[B_DATA_DONE], 1);
            check_card_written("open-ended write into DDR's end", a, 6100, 2, 512, 32);
            fifo_reset();
            wait_card_idle();
        end

        // ============================================== CRC failure: the bad block never reaches DDR
        for (int k = 0; k < 2; k++) begin
            automatic longint a = 64'h0060_0000;
            automatic string w = $sformatf("read CRC error at block %0d", 2 + k);
            card.inj_rd_crc_blk = 2 + k;
            dma_cmd(18, 7000, 0, a, 512, 6);
            wait_bit({w, " DATA_CRC_ERR"}, B_DAT_CRC, 1, st);
            m1_check(w);
            status(st);
            check({w, ": no DATA_DONE, not busy"}, st & 64'h0222, 64'h0);
            check_ddr_read(w, a, 7000, 2 + k, 512);     // (only the good blocks; nothing of the bad one)
            mr(SDM_BLKDONE, d); check({w, " BLKDONE = good blocks"}, d, 2 + k);
            mr(SDM_DMA, d);     check({w, " DMA_ADDR"}, d, a + (2 + k) * 512);
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end
        // write: a bad CRC token ends the DMA
        begin
            automatic longint a = 64'h0061_0000;
            ddr_fill(a, 6 * 512, 41);
            card.inj_wr_badtok_blk = 2;
            dma_cmd(25, 7100, 1, a, 512, 6);
            wait_bit("write bad token: WRITE_CRC_ERR", B_WR_CRC, 1, st);
            m1_check("write bad token");
            mr(SDM_BLKDONE, d); check("write bad token: BLKDONE", d, 2);
            stop_cmd(st);
            fifo_reset();
            wait_card_idle();
        end

        // ============================================== register behaviour during a DMA phase
        begin
            automatic longint a = 64'h0070_0000;
            automatic string w = "during a DMA";
            card.n_gap = 400;                             // slow blocks: time to poke
            dma_cmd(18, 8000, 0, a, 512, 8);
            repeat (300) @(negedge clk);
            status(st); check({w, ": DMA_BUSY, DATA_BUSY"}, st & 64'h22, 64'h22);
            mw(SDM_DMA, 64'h0012_3400);
            mr(SDM_DMA, d); check_true({w, ": DMA_ADDR write ignored (live)"}, d >= a && d <= a + 8 * 512);
            t0 = ncyc;
            do mr(SDM_FIFO, d); while (d == 0 && ncyc - t0 < 200000);
            n0 = d;
            mr(SDM_DATA, d);  check({w, ": SDM_DATA load reads 0"}, d, 0);
            mw(SDM_DATA, 64'h1234);
            status(st); check({w, ": SDM_DATA store sets FIFO_ERR"}, st[B_FIFO_ERR], 1);
            check({w, ": FIFO_THRESH reads 0"}, st[B_THRESH], 0);
            // (slow DDR: a whole block waits in the FIFO, as for PIO's
            // FIFO_THRESH; it still reads 0, and does not interrupt)
            lmax = 400; mw(SDM_IRQ_EN, 64'h10);
            t0 = ncyc; while (dut.native_i.w_level < 128 && ncyc - t0 < 400000) @(negedge clk);
            status(st);
            check_true({w, ": whole blocks in the FIFO"}, dut.native_i.w_level >= 64);
            check({w, ": FIFO_THRESH 0 with a block in the FIFO"}, st[B_THRESH], 0);
            check({w, ": no FIFO_THRESH interrupt"}, irq, 0);
            mw(SDM_IRQ_EN, 0); lmax = 12;
            // a data command while DMA_BUSY is ignored (no command, no change)
            n0 = card.n_cmds;
            mw(SDM_CMD, {50'h0, 3'b001, 3'(RT_R1), 2'b00, 6'd17});
            repeat (100) @(negedge clk);
            check({w, ": data command ignored"}, card.n_cmds - n0, 0);
            // a non-data command goes out (CMD13)
            n0 = card.n_cmds;
            mw(SDM_ARG, {rca, 16'h0}); mw(SDM_CMD, {50'h0, 3'b000, 3'(RT_R1), 2'b00, 6'd13});
            wait_cmd(st);
            check({w, ": CMD13 sent"}, card.n_cmds - n0, 1);
            wait_bit({w, " DATA_DONE"}, B_DATA_DONE, 1, st);
            m1_check(w);
            check_ddr_read(w, a, 8000, 8, 512);           // the store did not get into the data
            card.n_gap = 2;
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end

        // ============================================== aborts
        // DMA_EN = 0 during a read: the block in progress finishes, the rest
        // stays for PIO; DMA_ADDR block-granular
        begin
            automatic longint a = 64'h0080_0000;
            automatic string w = "read: DMA_EN = 0";
            dma_cmd(18, 9000, 0, a, 512, 6);
            t0 = ncyc;
            while (dut.native_i.r_dm_nblk < 2 && ncyc - t0 < 1_000_000) @(negedge clk);
            mw(SDM_CFG, CFG_W4);
            m1_check(w);
            mr(SDM_DMA, d);
            check_true({w, ": DMA_ADDR block-granular"}, (d - a) % 512 == 0 && d - a >= 2 * 512);
            n0 = (d - a) / 512;
            check_ddr_read(w, a, 9000, n0, 512);
            // the card side completes as PIO
            for (int b = n0; b < 6; b++) begin
                wait_bit({w, " PIO THRESH"}, B_THRESH, 1, st);
                for (int k = 0; k < 64; k++) begin
                    mr(SDM_DATA, d);
                    if (k == 0 || k == 63) check({w, ": PIO data"}, d, card_word((9000 + b) * 512 + k * 8));
                end
            end
            wait_bit({w, " DATA_DONE"}, B_DATA_DONE, 1, st);
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end
        // DMA_EN = 0 during a write
        begin
            automatic longint a = 64'h0081_0000;
            automatic string w = "write: DMA_EN = 0";
            // (20 blocks: the FIFO holds 8, so fetching is left when it stops)
            ddr_fill(a, 20 * 512, 51);
            dma_cmd(25, 9100, 1, a, 512, 20);
            bw0 = card.blocks_written; t0 = ncyc; while (card.blocks_written - bw0 < 1 && ncyc - t0 < 1_000_000) @(negedge clk);
            mw(SDM_CFG, CFG_W4);
            m1_check(w);
            mr(SDM_DMA, d);
            check_true({w, ": DMA_ADDR block-granular, fetching stopped"}, (d - a) % 512 == 0 && d > a && d < a + 20 * 512);
            n0 = (d - a) / 512;
            // the card side goes on as PIO with the fetched blocks, then waits
            // for more (clock parked between blocks): stop it there (the card
            // model can't take a CMD12 during its write busy)
            t0 = ncyc; while (!(dut.native_i.r_d_state == 8 && dut.native_i.w_level == 0) && ncyc - t0 < 2_000_000) @(negedge clk);
            mr(SDM_BLKDONE, d); check({w, ": the fetched blocks reached the card"}, d, n0);
            check_card_written(w, a, 9100, n0, 512, 51);
            stop_cmd(st);
            fifo_reset();
            wait_card_idle();
        end
        // FIFO reset during a read / a write (at once)
        for (int wr = 0; wr < 2; wr++) begin
            automatic longint a = 64'h0082_0000;
            automatic string w = wr ? "write: FIFO reset" : "read: FIFO reset";
            if (wr) ddr_fill(a, 6 * 512, 61);
            dma_cmd(wr ? 25 : 18, 9200, wr, a, 512, 6);
            t0 = ncyc;
            while (!(d_wdv || d_rdv) && ncyc - t0 < 1_000_000) @(negedge clk);
            repeat ($urandom_range(0, 40)) @(negedge clk);
            fifo_reset();
            m1_check(w);
            mr(SDM_FIFO, d); check({w, ": FIFO empty after"}, d, 0);
            stop_cmd(st);
            fifo_reset();
            wait_card_idle();
        end
        // CMD12 during an open-ended read: the good blocks received are finished
        begin
            automatic longint a = 64'h0083_0000;
            automatic string w = "open-ended read: CMD12";
            // (slow DDR: received blocks wait in the FIFO when CMD12 comes)
            lmax = 300;
            dma_cmd(18, 9300, 0, a, 512, 0);
            t0 = ncyc; while (dut.native_i.r_blkdone < 4 && ncyc - t0 < 2_000_000) @(negedge clk);
            stop_cmd(st);
            check({w, " DATA_DONE"}, st[B_DATA_DONE], 1);
            check({w, ": blocks still to move at the stop"}, st[B_DMA_BUSY] && dut.native_i.w_level >= 64, 1);
            // a data command now (DMA_BUSY, not DATA_BUSY) is ignored
            check({w, ": DATA_BUSY 0"}, st[B_DATA_BUSY], 0);
            n0 = card.n_cmds;
            mw(SDM_CMD, {50'h0, 3'b001, 3'(RT_R1), 2'b00, 6'd17});
            repeat (100) @(negedge clk);
            check({w, ": data command during DMA_BUSY ignored"}, card.n_cmds - n0, 0);
            m1_check(w);
            lmax = 12;
            mr(SDM_BLKDONE, n0);
            check_ddr_read(w, a, 9300, n0, 512);
            mr(SDM_DMA, d); check({w, ": DMA_ADDR = BLKDONE x 512"}, d, a + n0 * 512);
            mr(SDM_FIFO, d); check({w, ": FIFO empty"}, d, 0);
            wait_card_idle();
        end
        // CMD12 during a counted read (early): no DATA_DONE, received blocks moved
        begin
            automatic longint a = 64'h0084_0000;
            automatic string w = "counted read: early CMD12";
            dma_cmd(18, 9400, 0, a, 512, 8);
            t0 = ncyc; while (dut.native_i.r_blkdone < 2 && ncyc - t0 < 1_000_000) @(negedge clk);
            stop_cmd(st);
            m1_check(w);
            status(st); check({w, ": no DATA_DONE"}, st[B_DATA_DONE], 0);
            mr(SDM_BLKDONE, n0);
            check_ddr_read(w, a, 9400, n0, 512);
            wait_card_idle();
        end
        // CMD12 during an open-ended write
        begin
            automatic longint a = 64'h0085_0000;
            ddr_fill(a, 16 * 512, 71);
            dma_cmd(25, 9500, 1, a, 512, 0);
            bw0 = card.blocks_written; t0 = ncyc; while (card.blocks_written - bw0 < 2 + 0 && ncyc - t0 < 1_000_000) @(negedge clk);
            stop_cmd(st);
            check("open-ended write: CMD12 DATA_DONE", st[B_DATA_DONE], 1);
            m1_check("open-ended write: CMD12");
            fifo_reset();
            wait_card_idle();
        end
        // clearing SD_MODE during a read / a write
        for (int wr = 0; wr < 2; wr++) begin
            automatic longint a = 64'h0086_0000;
            automatic string w = wr ? "write: SD_MODE = 0" : "read: SD_MODE = 0";
            if (wr) ddr_fill(a, 6 * 512, 81);
            dma_cmd(wr ? 25 : 18, 9600, wr, a, 512, 6);
            t0 = ncyc;
            // (after the command's response: clearing SD_MODE in the middle
            // of a CMD-line frame leaves the card short of N_RC — a PIO matter)
            while (!(dut.native_i.r_dm_nblk >= 1 && dut.native_i.r_c_state == 0 && dut.native_i.r_trail == 0) && ncyc - t0 < 1_000_000) @(negedge clk);
            repeat ($urandom_range(0, 200)) @(negedge clk);
            mw(SD_CTRL, (64'h1 << 17) | 64'd1);
            m1_check(w);
            status(st); check({w, ": no DATA_DONE, not DATA_BUSY"}, st & 64'h0202, 0);
            mr(SDM_DMA, d);
            check_true({w, ": DMA_ADDR block-granular"}, (d - a) % 512 == 0);
            if (!wr) check_ddr_read(w, a, 9600, (d - a) / 512, 512);
            mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd1);
            // the card stays in data / rcv: CMD12 brings it back
            stop_cmd(st);
            fifo_reset();
            wait_card_idle();
        end
        // the native reset (program load with SD_MODE = 1) during a write
        // DMA's DDR read: the adapter completes its transaction, drops it
        begin
            automatic longint a = 64'h0087_0000;
            ddr_fill(a, 6 * 512, 91);
            // (slow DDR: the card waits for the first block, its clock parked,
            // while a DDR read is in flight — the card model can't take a
            // power cut in the middle of a frame)
            gmax = 30; lmax = 2000;
            dma_cmd(25, 9700, 1, a, 512, 6);
            t0 = ncyc; while (!(d_rdv && dut.native_i.r_d_state == 8 && dut.native_i.r_c_state == 0 && !card.cmd_oe) && ncyc - t0 < 1_000_000) @(negedge clk);
            @(negedge clk); load_rst = 1; @(negedge clk); load_rst = 0;
            m1_check("native reset");
            mr(SDM_FIFO, d); check("native reset: FIFO empty, read data dropped", d, 0);
            mr(SD_CTRL, d); check("native reset: SD_MODE / power off", d[18:17], 0);
            gmax = 6; lmax = 12;
            // the card was powered off: identify again (short)
            mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd9);
            mw(SDM_CFG, 64'h4); repeat (2000) @(posedge clk); mw(SDM_CFG, 0);
            repeat (200) @(posedge clk);
            do_cmd("CMD0", 0, 0, RT_NONE, st);
            do_cmd("CMD8", 8, 32'h1AA, RT_R1, st);
            for (int i = 0; i < 6; i++) begin
                do_cmd("CMD55", 55, 0, RT_R1, st);
                do_cmd("ACMD41", 41, 32'h40FF_8000, RT_R3, st);
                if (r0[31]) break;
            end
            do_cmd("CMD2", 2, 0, RT_R2, st);
            do_cmd("CMD3", 3, 0, RT_R1, st);
            rca = r0[31:16];
            do_cmd("CMD7", 7, {rca, 16'h0}, RT_R1B, st);
            do_cmd("CMD55", 55, {rca, 16'h0}, RT_R1, st);
            do_cmd("ACMD6", 6, 2, RT_R1, st);
            mw(SDM_CFG, CFG_W4);
            mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd1);
        end

        // ============================================== interrupts
        begin
            // DMA_ERR
            mw(SDM_IRQ_EN, 64'h0800_0000);
            fifo_reset(); clr_status();
            mw(SDM_DMA, DDR_END); set_blk(512, 1); mw(SDM_CFG, CFG_W4 | CFG_DMA);
            check("irq low before", irq, 0);
            cmd_start(17, 1, RT_R1, 1, 0);
            repeat (10) @(negedge clk);
            check("DMA_ERR raises the interrupt", irq, 1);
            mw(SDM_STATUS, 64'h0800_0000);
            repeat (3) @(negedge clk);
            check("W1C DMA_ERR drops it", irq, 0);
            // DATA_DONE of a DMA read: only once the data is in DDR
            mw(SDM_IRQ_EN, 64'h0200);
            dma_cmd(18, 9800, 0, 64'h0090_0000, 512, 3);
            t0 = ncyc; while (!irq && ncyc - t0 < 1_000_000) @(negedge clk);
            check("DMA read DATA_DONE interrupt", irq, 1);
            check_ddr_read("DMA read at its DATA_DONE interrupt", 64'h0090_0000, 9800, 3, 512);
            m1_check("DMA read irq");
            mw(SDM_IRQ_EN, 0);
            do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end

        // ============================================== random: sizes, counts, lanes, latencies
        for (int it = 0; it < 24; it++) begin
            automatic int bs  = 8 * $urandom_range(1, 64);
            automatic int cnt = $urandom_range(1, 9);
            automatic bit wr  = $urandom_range(0, 1);
            automatic longint a = 64'h00A0_0000 + 8 * $urandom_range(0, 1023);
            automatic longint blk = 10000 + it * 16;
            automatic string w = $sformatf("random %0d: %s %0d x %0d @%h", it, wr ? "write" : "read", cnt, bs, a);
            gmax = $urandom_range(0, 20); lmax = $urandom_range(0, 40);
            do_cmd("CMD16", 16, bs, RT_R1, st);
            if (wr) ddr_fill(a, cnt * bs, 100 + it);
            dma_cmd(cnt == 1 ? (wr ? 24 : 17) : (wr ? 25 : 18), blk, wr, a, bs, cnt);
            wait_bit({w, " DATA_DONE"}, B_DATA_DONE, 1, st);
            check({w, " no error"}, st & ERR_MASK, 0);
            if (!wr) begin
                int bad = 0;
                for (int b = 0; b < cnt; b++)
                    for (int k = 0; k < bs / 8; k++)
                        if (dget(a + b * bs + k * 8) !== card_word((blk + b) * 512 + k * 8)) bad++;
                check({w, " data at DATA_DONE"}, bad, 0);
                check_true({w, " DDR range"}, wr_lo == a && wr_hi == a + cnt * bs);
            end
            m1_check(w);
            if (wr) begin
                int bad = 0;
                for (int b = 0; b < cnt; b++)
                    for (int k = 0; k < bs / 8; k++)
                        if (card_word((blk + b) * 512 + k * 8) !== wpat(a + b * bs + k * 8, 100 + it)) bad++;
                check({w, " card data"}, bad, 0);
            end
            mr(SDM_DMA, d); check({w, " DMA_ADDR"}, d, a + cnt * bs);
            if (cnt > 1) do_cmd("CMD12", 12, 0, RT_R1B, st);
            wait_card_idle();
        end
        gmax = 6; lmax = 12;

        check("no host-protocol errors at the card", card.host_err, 0);
        if (errors == 0) $display("TB_SDDMA PASS (%0d checks, %0d DDR writes, %0d reads)", checks, n_wr, n_rd);
        else             $display("TB_SDDMA FAIL %0d (of %0d checks)", errors, checks);
        $finish;
    end

    initial begin
        #400ms;
        $display("TB_SDDMA FAIL timeout");
        $finish;
    end
endmodule

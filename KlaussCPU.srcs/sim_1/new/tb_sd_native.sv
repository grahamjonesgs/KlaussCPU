`timescale 1ns / 1ps
// tb_sd_native — unit test for the native SD controller (sd_spi + sd_native)
// against sd_card_model.  Self-checking; prints "TB_SD PASS" or
// "TB_SD FAIL n".  No waveform dump.
//
//   xvlog -sv mmio_if.sv sd_native.sv sd_spi.sv sd_card_model.sv tb_sd_native.sv
//   xelab tb_sd_native && xsim tb_sd_native -R
//   (tools/sdsim/run_sd_tb.sh runs this and tb_sd_spi_compat)
//
// MMIO accesses are driven the way the SoC does them: the strobe is held for
// three cycles, the address is held throughout, read data is sampled in the
// access's second cycle (KlaussCPU registers the decode select, then the
// data), and there is at least one strobe-low cycle between accesses.
module tb_sd_native;
    logic clk = 0, rst_l = 0, load_rst = 0;
    always #5 clk = ~clk;

    mmio_if bus();
    wire        sck, reset_n, irq;
    wire        cmd_o, cmd_oe;
    wire [3:0]  dat_o, dat_oe;
    wire        sd_cmd;
    wire [3:0]  sd_dat;
    logic       cd = 1'b1;

    assign sd_cmd = cmd_oe ? cmd_o : 1'bz;
    for (genvar i = 0; i < 4; i++) begin : g_d
        assign sd_dat[i] = dat_oe[i] ? dat_o[i] : 1'bz;
    end
    pullup (sd_cmd);
    pullup (sd_dat[0]); pullup (sd_dat[1]); pullup (sd_dat[2]); pullup (sd_dat[3]);

    sd_spi dut (
        .i_Clk(clk), .i_Rst_L(rst_l), .i_load_rst(load_rst), .mmio(bus), .o_irq(irq),
        .i_sd_cd(cd), .o_sd_reset_n(reset_n), .o_sd_sck(sck),
        .o_sd_cmd(cmd_o), .o_sd_cmd_oe(cmd_oe), .i_sd_cmd(sd_cmd),
        .o_sd_dat(dat_o), .o_sd_dat_oe(dat_oe), .i_sd_dat(sd_dat)
    );
    sd_card_model card (.clk(sck), .pwr(!reset_n), .cmd(sd_cmd), .dat(sd_dat));

    localparam logic [31:0] BASE = 32'hF000_0000;
    localparam logic [15:0] SD_CTRL = 16'h000, SDM_CMD = 16'h100, SDM_ARG = 16'h108,
        SDM_RESP0 = 16'h110, SDM_RESP1 = 16'h118, SDM_BLK = 16'h120, SDM_STATUS = 16'h128,
        SDM_IRQ_EN = 16'h130, SDM_TIMEOUT = 16'h138, SDM_CFG = 16'h140, SDM_FIFO = 16'h148,
        SDM_DMA = 16'h150, SDM_BLKDONE = 16'h158, SDM_DATA = 16'h180, SDM_CAPS = 16'h1F8;
    // SDM_STATUS bits
    localparam int B_CMD_BUSY = 0, B_DATA_BUSY = 1, B_DAT0 = 2, B_CP = 3, B_THRESH = 4,
        B_CMD_DONE = 8, B_DATA_DONE = 9, B_CMD_TMO = 16, B_CMD_CRC = 17, B_CMD_END = 18,
        B_CMD_IDX = 19, B_DAT_TMO = 20, B_DAT_CRC = 21, B_DAT_END = 22, B_WR_CRC = 23,
        B_WR_NOTOK = 24, B_BUSY_TMO = 25, B_FIFO_ERR = 26, B_ERR_ANY = 31;
    localparam logic [63:0] ERR_MASK = 64'h07FF_0000;
    // response types
    localparam int RT_NONE = 0, RT_R1 = 1, RT_R1B = 2, RT_R2 = 3, RT_R3 = 4;

    int errors = 0;
    int checks = 0;
    int contention = 0;
    longint ncyc = 0;
    always @(posedge clk) ncyc++;

    // ---------------- contention / pin monitors ----------------
    always @(posedge clk) begin
        if (cmd_oe && card.cmd_oe) begin contention++; if (contention < 5) $display("[%0t] CONTENTION on CMD", $time); end
        for (int i = 0; i < 4; i++)
            if (dat_oe[i] && card.dat_oe[i]) begin contention++; if (contention < 5) $display("[%0t] CONTENTION on DAT%0d", $time, i); end
    end
    // SD CLK activity: cycles since the last edge
    longint last_sck_edge = 0;
    always @(sck) last_sck_edge = ncyc;
    int sck_rises = 0;
    always @(posedge sck) sck_rises++;

    // ---------------- MMIO ----------------
    int hold = 3;
    task automatic mw(input logic [15:0] off, input logic [63:0] d);
        @(negedge clk);
        bus.addr = BASE | off; bus.write_data = d; bus.byte_en = 8'hFF; bus.write_DV = 1;
        repeat (hold) @(negedge clk);
        bus.write_DV = 0;
    endtask
    task automatic mr(input logic [15:0] off, output logic [63:0] d);
        @(negedge clk);
        bus.addr = BASE | off; bus.read_DV = 1;
        @(negedge clk);
        d = bus.read_data;              // second cycle of the access
        repeat (hold - 1) @(negedge clk);
        bus.read_DV = 0;
    endtask
    function automatic logic [63:0] rd(input logic [15:0] off);
        // (not used: reads need time)
        return 64'h0;
    endfunction

    task automatic check(input string what, input logic [63:0] got, input logic [63:0] exp);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("[%0t] FAIL %s: got %h exp %h", $time, what, got, exp);
        end
    endtask
    task automatic check_true(input string what, input bit c);
        checks++;
        if (!c) begin errors++; $display("[%0t] FAIL %s", $time, what); end
    endtask

    logic [63:0] st, d, r0, r1;

    task automatic status(output logic [63:0] s);
        mr(SDM_STATUS, s);
    endtask
    task automatic clr_status();
        mw(SDM_STATUS, 64'hFFFF_FFFF);
    endtask

    // start a command
    task automatic cmd_start(input int idx, input logic [31:0] arg, input int rt,
                             input bit data = 0, input bit wr = 0, input bit stop = 0);
        mw(SDM_ARG, {32'h0, arg});
        mw(SDM_CMD, {50'h0, stop, wr, data, 3'(rt), 2'b00, 6'(idx)});
    endtask
    task automatic wait_cmd(output logic [63:0] s, input int limit = 2_000_000);
        longint t0 = ncyc;
        do status(s); while (s[B_CMD_BUSY] && ncyc - t0 < limit);
        if (s[B_CMD_BUSY]) begin errors++; $display("[%0t] FAIL command stuck busy", $time); end
    endtask
    task automatic wait_data(output logic [63:0] s, input int limit = 4_000_000);
        longint t0 = ncyc;
        do status(s); while (s[B_DATA_BUSY] && ncyc - t0 < limit);
        if (s[B_DATA_BUSY]) begin errors++; $display("[%0t] FAIL data stuck busy", $time); end
    endtask
    // command with response; checks CMD_DONE and no error unless told otherwise
    task automatic do_cmd(input string what, input int idx, input logic [31:0] arg, input int rt,
                          output logic [63:0] s, input bit expect_ok = 1);
        clr_status();
        cmd_start(idx, arg, rt);
        wait_cmd(s);
        mr(SDM_RESP0, r0);
        mr(SDM_RESP1, r1);
        if (expect_ok) begin
            check({what, " CMD_DONE"}, s[B_CMD_DONE], 1);
            check({what, " no error"}, s & ERR_MASK, 0);
        end
    endtask

    task automatic set_blk(input int bytes, input int count);
        mw(SDM_BLK, {32'h0, 16'(count), 4'h0, 12'(bytes)});
    endtask
    task automatic fifo_reset();
        mw(SDM_FIFO, 64'h8000_0000);
    endtask
    task automatic pop(output logic [63:0] w);
        mr(SDM_DATA, w);
    endtask
    task automatic push(input logic [63:0] w);
        mw(SDM_DATA, w);
    endtask

    function automatic logic [63:0] card_word(input longint byteaddr);
        logic [63:0] w;
        for (int k = 0; k < 8; k++) w[8*k +: 8] = card.rd_byte(byteaddr + k);
        return w;
    endfunction
    function automatic logic [63:0] wpat(input longint blk, input int w, input int seed);
        return {32'(blk * 1000 + w + seed), 32'(32'hA5C3_0000 ^ (w * 7919) ^ (seed << 20))};
    endfunction

    // drain n blocks of `words` words on FIFO_THRESH, compare with the card
    task automatic drain_check(input string what, input longint blk, input int n, input int words);
        logic [63:0] s, w;
        int bad = 0;
        for (int b = 0; b < n; b++) begin
            longint t0 = ncyc;
            do status(s); while (!s[B_THRESH] && ncyc - t0 < 2_000_000);
            if (!s[B_THRESH]) begin errors++; $display("FAIL %s: no FIFO_THRESH for block %0d", what, b); return; end
            for (int k = 0; k < words; k++) begin
                pop(w);
                if (w !== card_word((blk + b) * 512 + k * 8)) begin
                    bad++;
                    if (bad < 4) $display("FAIL %s: block %0d word %0d got %h exp %h", what, b, k, w, card_word((blk + b) * 512 + k * 8));
                end
            end
        end
        checks++;
        if (bad) errors++;
    endtask

    // push n blocks of 64 words on FIFO_THRESH (write phase running)
    task automatic push_blocks(input longint blk, input int first, input int n, input int seed, input bit on_thresh);
        logic [63:0] s;
        for (int b = first; b < first + n; b++) begin
            if (on_thresh) begin
                longint t0 = ncyc;
                do status(s); while (!s[B_THRESH] && ncyc - t0 < 2_000_000);
                if (!s[B_THRESH]) begin errors++; $display("FAIL push: no FIFO_THRESH for block %0d", b); return; end
            end
            for (int k = 0; k < 64; k++) push(wpat(blk + b, k, seed));
        end
    endtask
    task automatic check_written(input string what, input longint blk, input int n, input int seed);
        int bad = 0;
        for (int b = 0; b < n; b++)
            for (int k = 0; k < 64; k++)
                if (card_word((blk + b) * 512 + k * 8) !== wpat(blk + b, k, seed)) begin
                    bad++;
                    if (bad < 4) $display("FAIL %s: block %0d word %0d card %h exp %h", what, b, k,
                                          card_word((blk + b) * 512 + k * 8), wpat(blk + b, k, seed));
                end
        checks++;
        if (bad) errors++;
    endtask

    task automatic wait_card_idle();
        longint t0 = ncyc;
        while ((card.rd_active || card.wr_active || card.cmd_oe || card.dat_oe != 0) && ncyc - t0 < 4_000_000) @(negedge clk);
        repeat (20) @(negedge clk);
    endtask

    // data read with an expected number of blocks and DATA_DONE
    task automatic read_cmd(input string what, input int idx, input longint blk, input int nblk_cmd, input int ndrain);
        logic [63:0] s;
        fifo_reset();
        clr_status();
        set_blk(512, nblk_cmd);
        cmd_start(idx, 32'(blk), RT_R1, 1, 0);
        drain_check(what, blk, ndrain, 64);
        wait_data(s);
        check({what, " DATA_DONE"}, s[B_DATA_DONE], 1);
        check({what, " no error"}, s & ERR_MASK, 0);
        mr(SDM_BLKDONE, d); check({what, " BLKDONE"}, d, ndrain);
        mr(SDM_FIFO, d);    check({what, " FIFO empty"}, d, 0);
    endtask

    logic [15:0] rca;
    int    n0;
    longint t0;

    initial begin
        bus.write_DV = 0; bus.read_DV = 0; bus.addr = 0; bus.write_data = 0; bus.byte_en = 0;
        repeat (5) @(posedge clk);
        rst_l = 1;
        repeat (3) @(posedge clk);

        // =========================================================== reset values
        mr(SD_CTRL, d);      check("SD_CTRL reset", d, 64'h0001_00F9);
        mr(SDM_CAPS, d);     check("SDM_CAPS", d, 64'h0008_0101);
        mr(SDM_TIMEOUT, d);  check("SDM_TIMEOUT reset", d, {32'd12_500_000, 32'd2_500_000});
        mr(SDM_CFG, d);      check("SDM_CFG reset", d, 0);
        mr(SDM_BLK, d);      check("SDM_BLK reset", d, 64'h0001_0200);
        mr(SDM_IRQ_EN, d);   check("SDM_IRQ_EN reset", d, 0);
        mr(SDM_FIFO, d);     check("SDM_FIFO reset", d, 0);
        mr(SDM_BLKDONE, d);  check("SDM_BLKDONE reset", d, 0);
        mr(SDM_DMA, d);      check("SDM_DMA_ADDR", d, 0);
        status(st);          check("SDM_STATUS reset (DAT0 pulled up, card present)", st, 64'h0C);
        cd = 0; repeat (4) @(posedge clk);
        status(st);          check("CARD_PRESENT follows CD", st[B_CP], 0);
        cd = 1;
        check("irq low", irq, 0);

        // =========================================================== register R/W
        mw(SDM_ARG, 64'hFFFF_FFFF_1234_5678);    mr(SDM_ARG, d);  check("SDM_ARG", d, 64'h1234_5678);
        mw(SDM_BLK, 64'hFFFF_FFFF_FFFF_FFFF);    mr(SDM_BLK, d);  check("SDM_BLK [2:0]/[15:12] read 0", d, 64'hFFFF_0FF8);
        mw(SDM_IRQ_EN, 64'hFFFF_FFFF_FFFF_FFFF); mr(SDM_IRQ_EN, d); check("SDM_IRQ_EN mask", d, 64'h07FF_0310);
        mw(SDM_IRQ_EN, 0);
        mw(SDM_CFG, 64'hFFFF_FFFF_FFFF_FFFF);    mr(SDM_CFG, d);  check("SDM_CFG reserved bits read 0", d, 64'h5);
        mw(SDM_CFG, 0);
        mw(SDM_TIMEOUT, 64'h0000_1111_0000_2222); mr(SDM_TIMEOUT, d); check("SDM_TIMEOUT", d, 64'h0000_1111_0000_2222);
        mw(SDM_TIMEOUT, {32'd12_500_000, 32'd2_500_000});
        mw(SDM_DMA, 64'hFFFF_FFFF);              mr(SDM_DMA, d);  check("SDM_DMA_ADDR reads 0", d, 0);
        mw(SDM_CAPS, 0);                          mr(SDM_CAPS, d); check("SDM_CAPS read-only", d, 64'h0008_0101);
        mr(SDM_CMD, d);                           check("SDM_CMD reads 0", d, 0);
        mw(SDM_STATUS, 64'hFFFF_FFFF);            status(st);      check("SDM_STATUS W1C on clean", st, 64'h0C);

        // =========================================================== SDM_CMD ignored in SPI mode
        n0 = sck_rises;
        cmd_start(0, 0, RT_NONE);
        repeat (50) @(posedge clk);
        status(st); check("SPI mode: SDM_CMD ignored (no busy, no done)", st[9:0] & 10'h303, 0);
        check("SPI mode: no SD clock", sck_rises, n0);
        mw(SDM_CFG, 64'h4);                       // CLK_ON in SPI mode: no clock either
        repeat (100) @(posedge clk);
        check("SPI mode: CLK_ON has no effect", sck_rises, n0);
        mw(SDM_CFG, 0);

        // =========================================================== power-up, 74 clocks
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd124);   // SD mode, power, 400 kHz
        mr(SD_CTRL, d); check("SD_CTRL SD_MODE readback", d, 64'h0006_007C);
        check("pins released in SD mode (CMD)", cmd_oe, 0);
        check("pins released in SD mode (DAT)", dat_oe, 0);
        repeat (100) @(posedge clk);
        n0 = sck_rises;
        mw(SDM_CFG, 64'h4);                       // CLK_ON
        t0 = ncyc;
        while (sck_rises - n0 < 80 && ncyc - t0 < 100000) @(posedge clk);
        mw(SDM_CFG, 0);
        check_true("CLK_ON gives >= 80 clocks", sck_rises - n0 >= 80);
        repeat (1000) @(posedge clk);
        check("CLK parks low when idle", sck, 0);
        n0 = sck_rises; repeat (2000) @(posedge clk);
        check("CLK stopped when idle", sck_rises, n0);

        // =========================================================== identification (400 kHz)
        do_cmd("CMD0", 0, 0, RT_NONE, st);
        check("CMD0 no response bits", r0, 0);
        do_cmd("CMD8", 8, 32'h1AA, RT_R1, st);
        check("CMD8 R7", r0, 64'h08_0000_01AA);
        check("RESP1 0 after a 48-bit response", r1, 0);
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd9);    // 5 MHz for the rest of identification (sim time)
        for (int i = 0; i < 6; i++) begin
            do_cmd("CMD55", 55, 0, RT_R1, st);
            do_cmd("ACMD41", 41, 32'h40FF_8000, RT_R3, st);
            if (r0[31]) break;
        end
        check("ACMD41 R3 (OCR ready, CCS, idx field 3F)", r0, 64'h3F_C0FF_8000);
        do_cmd("CMD2", 2, 0, RT_R2, st);
        check("CMD2 R2 RESP0 (CRC7 + end bit in [7:0])", r0, card.cid[63:0]);
        check("CMD2 R2 RESP1", r1, card.cid[127:64]);
        do_cmd("CMD3", 3, 0, RT_R1, st);
        rca = r0[31:16];
        check("CMD3 R6 RCA", rca, card.NEW_RCA);
        check("CMD3 R6 index", r0[37:32], 3);
        do_cmd("CMD9", 9, {rca, 16'h0}, RT_R2, st);
        check("CMD9 CSD", {r1, r0}, card.csd);
        t0 = ncyc;
        do_cmd("CMD7 (R1b)", 7, {rca, 16'h0}, RT_R1B, st);
        check("card in tran", card.st, 4);
        do_cmd("CMD13", 13, {rca, 16'h0}, RT_R1, st);
        check("CMD13 state tran", r0[12:9], 4);

        // =========================================================== ACMD51 (SCR, 8 B) in 1-bit: byte order
        fifo_reset(); set_blk(8, 1);
        do_cmd("CMD55", 55, {rca, 16'h0}, RT_R1, st);
        clr_status();
        cmd_start(51, 0, RT_R1, 1, 0);
        wait_data(st);
        check("ACMD51 DATA_DONE", st[B_DATA_DONE], 1);
        check("ACMD51 no error", st & ERR_MASK, 0);
        mr(SDM_FIFO, d); check("SCR: 1 word", d, 1);
        mr(SDM_BLKDONE, d); check("SCR BLKDONE", d, 1);
        pop(d); check("SCR little-endian (byte 0 = 0x02 in [7:0])", d, 64'h0000_0000_0380_3502);
        mr(SDM_FIFO, d); check("SCR popped", d, 0);
        pop(d); check("pop from empty FIFO returns 0", d, 0);
        mr(SDM_FIFO, d); check("pop from empty FIFO leaves level 0", d, 0);
        wait_card_idle();

        // =========================================================== 25 MHz, 1-bit single block read/write
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd1);
        read_cmd("CMD17 1-bit", 17, 100, 1, 1);
        wait_card_idle();
        fifo_reset();
        push_blocks(200, 0, 1, 11, 0);                 // pre-fill before the command
        mr(SDM_FIFO, d); check("pre-filled 64 words", d, 64);
        clr_status(); set_blk(512, 1);
        cmd_start(24, 200, RT_R1, 1, 1);
        wait_data(st);
        check("CMD24 1-bit DATA_DONE", st[B_DATA_DONE], 1);
        check("CMD24 1-bit no error", st & ERR_MASK, 0);
        mr(SDM_BLKDONE, d); check("CMD24 BLKDONE", d, 1);
        check_written("CMD24 1-bit", 200, 1, 11);
        wait_card_idle();

        // =========================================================== 4-bit
        do_cmd("CMD55", 55, {rca, 16'h0}, RT_R1, st);
        do_cmd("ACMD6", 6, 2, RT_R1, st);
        mw(SDM_CFG, 64'h1);
        check("card 4-bit", card.w4, 1);

        // CMD6 switch status (64 B)
        fifo_reset(); set_blk(64, 1); clr_status();
        cmd_start(6, 32'h00FF_FFF1, RT_R1, 1, 0);
        wait_data(st);
        check("CMD6 DATA_DONE", st[B_DATA_DONE], 1);
        check("CMD6 no error", st & ERR_MASK, 0);
        begin
            automatic int bad = 0;
            for (int k = 0; k < 8; k++) begin
                logic [63:0] e;
                for (int j = 0; j < 8; j++) e[8*j +: 8] = card.sw_status[k*8 + j];
                pop(d);
                if (d !== e) begin bad++; $display("FAIL CMD6 word %0d %h exp %h", k, d, e); end
            end
            check("CMD6 64-byte status", bad, 0);
        end
        wait_card_idle();
        // ACMD13 (SD status, 64 B)
        fifo_reset(); set_blk(64, 1);
        do_cmd("CMD55", 55, {rca, 16'h0}, RT_R1, st);
        clr_status(); cmd_start(13, 0, RT_R1, 1, 0); wait_data(st);
        check("ACMD13 DATA_DONE", st[B_DATA_DONE], 1);
        pop(d); check("ACMD13 first word", d, {8'(255-7), 8'(255-6), 8'(255-5), 8'(255-4), 8'(255-3), 8'(255-2), 8'(255-1), 8'(255)});
        wait_card_idle();

        read_cmd("CMD17 4-bit", 17, 300, 1, 1);
        wait_card_idle();
        read_cmd("CMD18 4-bit x4", 18, 400, 4, 4);
        do_cmd("CMD12 after counted read", 12, 0, RT_R1B, st);   // (no [13]: data already done)
        wait_card_idle();
        mr(SDM_BLKDONE, d); check("BLKDONE not reset by a non-data command", d, 4);

        // ---- data before the response has finished (N_AC = 2), 8-byte blocks in 4-bit
        card.n_ac = 2;
        fifo_reset(); set_blk(8, 1);
        do_cmd("CMD55", 55, {rca, 16'h0}, RT_R1, st);
        clr_status(); cmd_start(51, 0, RT_R1, 1, 0); wait_data(st); wait_cmd(st);
        check("early data: DATA_DONE", st[B_DATA_DONE], 1);
        check("early data: CMD_DONE", st[B_CMD_DONE], 1);
        check("early data: no error", st & ERR_MASK, 0);
        pop(d); check("early data: SCR", d, 64'h0000_0000_0380_3502);
        card.n_ac = 8;
        wait_card_idle();

        // =========================================================== read flow control: 20 blocks, CPU slow
        fifo_reset(); clr_status(); set_blk(512, 20);
        cmd_start(18, 1000, RT_R1, 1, 0);
        t0 = ncyc;
        do mr(SDM_FIFO, d); while (d < 512 && ncyc - t0 < 1_000_000);
        check("FIFO fills to 8 blocks", d, 512);
        repeat (200) @(posedge clk);
        n0 = sck_rises;
        repeat (5000) @(posedge clk);
        check("read: CLK stopped while the FIFO is full", sck_rises, n0);
        check("read: CLK parked low", sck, 0);
        status(st); check("read: DATA_BUSY while stopped", st[B_DATA_BUSY], 1);
        mr(SDM_BLKDONE, d); check("read: BLKDONE = 8 while stopped", d, 8);
        drain_check("CMD18 x20 flow control", 1000, 20, 64);
        wait_data(st);
        check("flow read DATA_DONE", st[B_DATA_DONE], 1);
        check("flow read no error", st & ERR_MASK, 0);
        mr(SDM_BLKDONE, d); check("flow read BLKDONE", d, 20);
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        wait_card_idle();

        // =========================================================== FIFO_THRESH (read)
        fifo_reset(); clr_status(); set_blk(512, 2);
        cmd_start(18, 2000, RT_R1, 1, 0);
        wait_data(st);
        check("THRESH after DATA_DONE with data left", st[B_THRESH], 1);
        for (int k = 0; k < 64; k++) pop(d);
        status(st); check("THRESH still set with one block left", st[B_THRESH], 1);
        set_blk(8, 1);              // latched size (§4.8): no effect until the next data command
        for (int k = 0; k < 63; k++) pop(d);
        status(st); check("THRESH drops below one block", st[B_THRESH], 0);
        pop(d); check("last word", d, card_word(2001 * 512 + 63 * 8));
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        wait_card_idle();

        // =========================================================== open-ended read + CMD12 [13]
        fifo_reset(); clr_status(); set_blk(512, 0);
        cmd_start(18, 3000, RT_R1, 1, 0);
        drain_check("open-ended read", 3000, 3, 64);
        clr_status();
        cmd_start(12, 0, RT_R1B, 0, 0, 1);
        wait_cmd(st);
        check("stop: CMD_DONE", st[B_CMD_DONE], 1);
        check("stop: DATA_DONE (open-ended ends cleanly)", st[B_DATA_DONE], 1);
        check("stop: DATA_BUSY clear", st[B_DATA_BUSY], 0);
        check("stop: no error", st & ERR_MASK, 0);
        mr(SDM_BLKDONE, d);
        check_true("stop: BLKDONE >= 3", d >= 3);
        mr(SDM_FIFO, r0); check("stop: FIFO holds the good blocks beyond 3", r0, (d - 3) * 64);
        wait_card_idle();

        // =========================================================== counted read stopped early by [13]
        fifo_reset(); clr_status(); set_blk(512, 30);
        cmd_start(18, 3500, RT_R1, 1, 0);
        drain_check("counted read stopped early", 3500, 2, 64);
        clr_status();
        cmd_start(12, 0, RT_R1B, 0, 0, 1);
        wait_cmd(st);
        check("early stop: CMD_DONE", st[B_CMD_DONE], 1);
        check("early stop: no DATA_DONE (counted)", st[B_DATA_DONE], 0);
        check("early stop: DATA_BUSY clear", st[B_DATA_BUSY], 0);
        check("early stop: no error", st & ERR_MASK, 0);
        wait_card_idle();
        // stop with no data phase: an ordinary R1b, DATA_* unchanged
        clr_status();
        cmd_start(12, 0, RT_R1B, 0, 0, 1);
        wait_cmd(st);
        check("stop, no data phase: only CMD_DONE", st[15:0] & 16'h0303, 16'h0100);

        // =========================================================== clearing SD_MODE abandons the engines
        fifo_reset(); clr_status(); set_blk(512, 20);
        cmd_start(18, 3600, RT_R1, 1, 0);
        t0 = ncyc;
        do mr(SDM_BLKDONE, d); while (d < 2 && ncyc - t0 < 1_000_000);
        clr_status();
        // The pins go back to the SPI mapping while the card is mid-block
        // and still driving DAT: that contention is why switching modes
        // needs a power cycle (a software rule, §4.1).  Not counted here.
        n0 = contention;
        mw(SD_CTRL, (64'h1 << 17) | 64'd1);                 // SD_MODE 0, power kept
        status(st);
        check("SD_MODE clear: busy bits clear", st[1:0], 0);
        check("SD_MODE clear: no DONE", st[9:8], 0);
        check("SD_MODE clear: THRESH (DIR kept, data kept)", st[B_THRESH], 1);
        mr(SDM_FIFO, r0);
        check_true("SD_MODE clear: FIFO kept (whole blocks)", r0 >= 128 && r0[5:0] == 0);
        check("SD_MODE clear: SPI pins", {cmd_oe, dat_oe}, 5'b1_1110);
        pop(d); check("SD_MODE clear: pop works in SPI mode", d, card_word(3600 * 512));
        fifo_reset();
        status(st); check("FIFO reset: DIR NONE, no THRESH", st[B_THRESH], 0);
        mw(SDM_ARG, 64'hAA); mr(SDM_ARG, d); check("SD_MODE 0: registers writable", d, 64'hAA);
        // back to SD mode: the card was left mid-transfer; CMD12 recovers it
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd1);
        do_cmd("CMD12 after abandon", 12, 0, RT_R1B, st, 0);
        wait_card_idle();
        contention = n0;
        do_cmd("CMD13 after abandon", 13, {rca, 16'h0}, RT_R1, st);
        check("card back in tran", r0[12:9], 4);

        // =========================================================== multi-block write, pre-fill + THRESH
        fifo_reset();
        status(st); check("write THRESH not set while idle (empty FIFO)", st[B_THRESH], 0);
        push_blocks(5000, 0, 2, 22, 0);
        clr_status(); set_blk(512, 5);
        status(st); check("pre-fill does not raise THRESH (DIR = NONE)", st[B_THRESH], 0);
        cmd_start(25, 5000, RT_R1, 1, 1);
        push_blocks(5000, 2, 3, 22, 1);
        status(st); check("write THRESH clear once count blocks pushed", st[B_THRESH], 0);
        wait_data(st);
        check("CMD25 DATA_DONE", st[B_DATA_DONE], 1);
        check("CMD25 no error", st & ERR_MASK, 0);
        mr(SDM_BLKDONE, d); check("CMD25 BLKDONE", d, 5);
        do_cmd("CMD12 after write", 12, 0, RT_R1B, st);
        check_written("CMD25 x5", 5000, 5, 22);
        wait_card_idle();

        // =========================================================== write flow control
        fifo_reset(); clr_status(); set_blk(512, 3);
        push_blocks(6000, 0, 1, 33, 0);
        cmd_start(25, 6000, RT_R1, 1, 1);
        t0 = ncyc;
        do mr(SDM_BLKDONE, d); while (d < 1 && ncyc - t0 < 1_000_000);
        repeat (300) @(posedge clk);
        n0 = sck_rises;
        repeat (5000) @(posedge clk);
        check("write: CLK stopped waiting for a block", sck_rises, n0);
        status(st); check("write: DATA_BUSY while stopped", st[B_DATA_BUSY], 1);
        check("write: THRESH while waiting", st[B_THRESH], 1);
        n0 = card.n_cmds;
        cmd_start(17, 6500, RT_R1, 1, 0);                    // data command while DATA_BUSY: ignored
        repeat (2000) @(posedge clk);
        status(st);
        check("data SDM_CMD while DATA_BUSY: not sent", card.n_cmds, n0);
        check("data SDM_CMD while DATA_BUSY: no CMD_BUSY", st[B_CMD_BUSY], 0);
        mr(SDM_BLKDONE, d); check("data SDM_CMD while DATA_BUSY: BLKDONE kept", d, 1);
        check("data SDM_CMD while DATA_BUSY: DIR kept (write THRESH)", st[B_THRESH], 1);
        do_cmd("CMD13 during a write phase", 13, {rca, 16'h0}, RT_R1, st);
        check("CMD13 went out", card.n_cmds, n0 + 1);
        check("CMD13 leaves the data phase running", st[B_DATA_BUSY], 1);
        push_blocks(6000, 1, 2, 33, 1);
        wait_data(st);
        check("flow write DATA_DONE", st[B_DATA_DONE], 1);
        check("flow write no error", st & ERR_MASK, 0);
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        check_written("flow write", 6000, 3, 33);
        wait_card_idle();

        // =========================================================== open-ended write + stop
        fifo_reset(); clr_status(); set_blk(512, 0);
        push_blocks(7000, 0, 2, 44, 0);
        cmd_start(25, 7000, RT_R1, 1, 1);
        t0 = ncyc;
        do mr(SDM_BLKDONE, d); while (d < 2 && ncyc - t0 < 1_000_000);
        status(st);
        check("open write: still DATA_BUSY", st[B_DATA_BUSY], 1);
        check("open write: THRESH (open-ended, room)", st[B_THRESH], 1);
        clr_status();
        cmd_start(12, 0, RT_R1B, 0, 0, 1);
        wait_cmd(st);
        check("open write stop: DATA_DONE", st[B_DATA_DONE], 1);
        check("open write stop: DATA_BUSY clear", st[B_DATA_BUSY], 0);
        check("open write stop: no error", st & ERR_MASK, 0);
        check_written("open-ended write", 7000, 2, 44);
        wait_card_idle();

        // =========================================================== SDM_CMD ignored while CMD_BUSY
        n0 = card.n_cmds;
        clr_status();
        cmd_start(13, {rca, 16'h0}, RT_R1);
        mw(SDM_CMD, {50'h0, 3'b000, 3'(RT_R1), 2'b00, 6'd55});   // while busy: ignored
        wait_cmd(st);
        repeat (400) @(posedge clk);
        check("SDM_CMD while CMD_BUSY ignored", card.n_cmds, n0 + 1);
        mr(SDM_RESP0, d); check("response is CMD13's", d[37:32], 13);

        // =========================================================== command errors
        card.inj_no_resp = 13;
        do_cmd("timeout", 13, {rca, 16'h0}, RT_R1, st, 0);
        check("CMD_TIMEOUT", st & ERR_MASK, 64'h1 << B_CMD_TMO);
        check("CMD_TIMEOUT sets CMD_DONE", st[B_CMD_DONE], 1);
        check("ERR_ANY", st[B_ERR_ANY], 1);
        mw(SDM_STATUS, 64'h1 << B_CMD_TMO); status(st);
        check("W1C clears CMD_TIMEOUT only", st & (ERR_MASK | 64'h8000_0300), 64'h100);
        card.inj_rsp_crc = 13;
        do_cmd("crc", 13, {rca, 16'h0}, RT_R1, st, 0);
        check("CMD_CRC_ERR", st & ERR_MASK, 64'h1 << B_CMD_CRC);
        card.inj_rsp_end = 13;
        do_cmd("end", 13, {rca, 16'h0}, RT_R1, st, 0);
        check("CMD_END_ERR", st & ERR_MASK, 64'h1 << B_CMD_END);
        card.inj_rsp_idx = 13;
        do_cmd("idx", 13, {rca, 16'h0}, RT_R1, st, 0);
        check_true("CMD_IDX_ERR", st[B_CMD_IDX]);
        // R2 CRC error
        card.inj_rsp_crc = 9;
        do_cmd("CMD7 deselect", 7, 0, RT_NONE, st);
        do_cmd("R2 crc", 9, {rca, 16'h0}, RT_R2, st, 0);
        check("R2 CMD_CRC_ERR", st & ERR_MASK, 64'h1 << B_CMD_CRC);
        do_cmd("CMD7 reselect", 7, {rca, 16'h0}, RT_R1B, st);
        // response error on a data command aborts its data phase
        card.inj_rsp_crc = 17;
        fifo_reset(); clr_status(); set_blk(512, 1);
        cmd_start(17, 50, RT_R1, 1, 0);
        wait_cmd(st);
        repeat (20) @(posedge clk);
        status(st);
        check("data cmd response error: CMD_CRC_ERR", st & ERR_MASK, 64'h1 << B_CMD_CRC);
        check("data cmd response error: DATA_BUSY clear", st[B_DATA_BUSY], 0);
        check("data cmd response error: no DATA_DONE", st[B_DATA_DONE], 0);
        do_cmd("CMD12 after response error", 12, 0, RT_R1B, st, 0);   // the card is mid-block
        wait_card_idle();
        mr(SDM_FIFO, d); check("data cmd response error: nothing in the FIFO", d, 0);
        mr(SDM_BLKDONE, d); check("data cmd response error: BLKDONE 0", d, 0);

        // =========================================================== read data errors
        card.inj_rd_crc_blk = 2;
        fifo_reset(); clr_status(); set_blk(512, 4);
        cmd_start(18, 8000, RT_R1, 1, 0);
        wait_data(st);
        check("read CRC error: DATA_CRC_ERR", st & ERR_MASK, 64'h1 << B_DAT_CRC);
        check("read CRC error: no DATA_DONE", st[B_DATA_DONE], 0);
        mr(SDM_BLKDONE, d); check("read CRC error: BLKDONE = good blocks", d, 2);
        mr(SDM_FIFO, d);    check("read CRC error: bad block rolled back", d, 128);
        do_cmd("CMD12 after read error", 12, 0, RT_R1B, st, 0);
        check("CMD12 after error: CMD_DONE", st[B_CMD_DONE], 1);
        drain_check("good blocks before the CRC error", 8000, 2, 64);
        wait_card_idle();

        card.inj_rd_end_blk = 0;
        fifo_reset(); clr_status(); set_blk(512, 2);
        cmd_start(18, 8100, RT_R1, 1, 0);
        wait_data(st);
        check("read end error: DATA_END_ERR", st & ERR_MASK, 64'h1 << B_DAT_END);
        mr(SDM_BLKDONE, d); check("read end error: BLKDONE 0", d, 0);
        mr(SDM_FIFO, d);    check("read end error: FIFO empty", d, 0);
        do_cmd("CMD12", 12, 0, RT_R1B, st, 0);
        wait_card_idle();

        card.inj_rd_nodata = 1;
        mw(SDM_TIMEOUT, {32'd12_500_000, 32'd300});
        fifo_reset(); clr_status(); set_blk(512, 1);
        cmd_start(17, 8200, RT_R1, 1, 0);
        wait_data(st);
        check("DATA_TIMEOUT", st & ERR_MASK, 64'h1 << B_DAT_TMO);
        mw(SDM_TIMEOUT, {32'd12_500_000, 32'd2_500_000});
        wait_card_idle();

        // =========================================================== write errors
        card.inj_wr_badtok_blk = 1;
        fifo_reset(); clr_status(); set_blk(512, 3);
        push_blocks(9000, 0, 3, 55, 0);
        cmd_start(25, 9000, RT_R1, 1, 1);
        wait_data(st);
        check("bad token: WRITE_CRC_ERR", st & ERR_MASK, 64'h1 << B_WR_CRC);
        check("bad token: no DATA_DONE", st[B_DATA_DONE], 0);
        mr(SDM_BLKDONE, d); check("bad token: BLKDONE", d, 1);
        do_cmd("CMD12 after write error", 12, 0, RT_R1B, st, 0);
        fifo_reset();
        wait_card_idle();

        card.inj_wr_notok_blk = 0;
        fifo_reset(); clr_status(); set_blk(512, 1);
        push_blocks(9100, 0, 1, 66, 0);
        cmd_start(24, 9100, RT_R1, 1, 1);
        wait_data(st);
        check("no token: WRITE_NO_TOKEN", st & ERR_MASK, 64'h1 << B_WR_NOTOK);
        mr(SDM_BLKDONE, d); check("no token: BLKDONE 0", d, 0);
        do_cmd("CMD12", 12, 0, RT_R1B, st, 0);
        wait_card_idle();

        card.inj_wr_longbusy_blk = 0; card.long_busy = 3000;
        mw(SDM_TIMEOUT, {32'd500, 32'd2_500_000});
        fifo_reset(); clr_status(); set_blk(512, 1);
        push_blocks(9200, 0, 1, 77, 0);
        cmd_start(24, 9200, RT_R1, 1, 1);
        wait_data(st);
        check("long write busy: BUSY_TIMEOUT", st & ERR_MASK, 64'h1 << B_BUSY_TMO);
        mr(SDM_BLKDONE, d); check("long write busy: BLKDONE 0", d, 0);
        mw(SDM_CFG, 64'h5);              // the model's busy counts clocks: let it finish
        wait_card_idle();
        mw(SDM_CFG, 64'h1);
        card.inj_r1b_long = 1;
        do_cmd("R1b long busy", 12, 0, RT_R1B, st, 0);
        check("R1b: BUSY_TIMEOUT", st & ERR_MASK, 64'h1 << B_BUSY_TMO);
        check("R1b: CMD_DONE", st[B_CMD_DONE], 1);
        mw(SDM_TIMEOUT, {32'd12_500_000, 32'd2_500_000});
        mw(SDM_CFG, 64'h5);
        wait_card_idle();
        mw(SDM_CFG, 64'h1);

        // =========================================================== FIFO overflow / ERR
        fifo_reset(); clr_status();
        for (int k = 0; k < 513; k++) push(k);
        mr(SDM_FIFO, d); check("FIFO full at 512", d, 512);
        status(st); check("FIFO_ERR on push to full", st & ERR_MASK, 64'h1 << B_FIFO_ERR);
        pop(d); check("FIFO head word 0", d, 0);
        pop(d); check("FIFO word 1", d, 1);
        fifo_reset();
        mr(SDM_FIFO, d); check("FIFO reset", d, 0);
        // each sticky bit clears with its own W1C
        mw(SDM_STATUS, 64'h1 << B_FIFO_ERR); status(st);
        check("FIFO_ERR W1C", st[B_FIFO_ERR], 0);
        check("ERR_ANY clear", st[B_ERR_ANY], 0);

        // =========================================================== IRQ
        clr_status();
        mw(SDM_IRQ_EN, 64'h1 << B_CMD_DONE);
        check("irq low before", irq, 0);
        cmd_start(13, {rca, 16'h0}, RT_R1); wait_cmd(st);
        repeat (2) @(posedge clk);
        check("irq on CMD_DONE", irq, 1);
        mw(SDM_STATUS, 64'h1 << B_CMD_DONE); repeat (2) @(posedge clk);
        check("irq drops on W1C", irq, 0);
        mw(SDM_IRQ_EN, 64'h1 << B_THRESH);
        fifo_reset(); clr_status(); set_blk(512, 1);
        cmd_start(17, 77, RT_R1, 1, 0); wait_data(st);
        repeat (2) @(posedge clk);
        check("irq on FIFO_THRESH", irq, 1);
        for (int k = 0; k < 64; k++) pop(d);
        repeat (2) @(posedge clk);
        check("irq drops when drained", irq, 0);
        wait_card_idle();
        mw(SDM_IRQ_EN, 64'h1 << B_DATA_DONE);
        repeat (2) @(posedge clk);
        check("irq on DATA_DONE (still set)", irq, 1);
        clr_status(); repeat (2) @(posedge clk);
        check("irq low after clearing", irq, 0);
        mw(SDM_IRQ_EN, 64'h1 << B_CMD_TMO);
        card.inj_no_resp = 13;
        cmd_start(13, {rca, 16'h0}, RT_R1); wait_cmd(st);
        repeat (2) @(posedge clk);
        check("irq on CMD_TIMEOUT", irq, 1);
        mw(SDM_IRQ_EN, 0); repeat (2) @(posedge clk);
        check("irq follows IRQ_EN", irq, 0);
        clr_status();

        // =========================================================== 50 MHz (clk_div = 0) does not break
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd0);
        read_cmd("CMD18 at clk_div 0", 18, 11000, 3, 3);
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        wait_card_idle();
        fifo_reset(); clr_status(); set_blk(512, 2);
        push_blocks(11100, 0, 2, 88, 0);
        cmd_start(25, 11100, RT_R1, 1, 1);
        wait_data(st);
        check("CMD25 at clk_div 0 DATA_DONE", st[B_DATA_DONE], 1);
        check("CMD25 at clk_div 0 no error", st & ERR_MASK, 0);
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        check_written("CMD25 at clk_div 0", 11100, 2, 88);
        wait_card_idle();
        // and a read with flow control at clk_div 0
        fifo_reset(); clr_status(); set_blk(512, 10);
        cmd_start(18, 11200, RT_R1, 1, 0);
        t0 = ncyc;
        do mr(SDM_FIFO, d); while (d < 512 && ncyc - t0 < 1_000_000);
        repeat (3000) @(posedge clk);
        drain_check("clk_div 0 flow control", 11200, 10, 64);
        wait_data(st);
        check("clk_div 0 flow read no error", st & ERR_MASK, 0);
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        wait_card_idle();

        // =========================================================== 1-bit multi-block at 25 MHz
        mw(SD_CTRL, (64'h1 << 18) | (64'h1 << 17) | 64'd1);
        do_cmd("CMD55", 55, {rca, 16'h0}, RT_R1, st);
        do_cmd("ACMD6 1-bit", 6, 0, RT_R1, st);
        mw(SDM_CFG, 64'h0);
        read_cmd("CMD18 1-bit x3", 18, 12000, 3, 3);
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        wait_card_idle();
        fifo_reset(); clr_status(); set_blk(512, 2);
        push_blocks(12100, 0, 1, 99, 0);
        cmd_start(25, 12100, RT_R1, 1, 1);
        push_blocks(12100, 1, 1, 99, 1);
        wait_data(st);
        check("CMD25 1-bit DATA_DONE", st[B_DATA_DONE], 1);
        check("CMD25 1-bit no error", st & ERR_MASK, 0);
        do_cmd("CMD12", 12, 0, RT_R1B, st);
        check_written("CMD25 1-bit x2", 12100, 2, 99);
        wait_card_idle();

        // =========================================================== program-load reset
        // SD_MODE = 1, mid-transfer: the controller resets as on CPU_RESETN
        mw(SDM_IRQ_EN, 64'h1 << B_THRESH);
        fifo_reset(); clr_status(); set_blk(512, 20);
        cmd_start(18, 13000, RT_R1, 1, 0);
        t0 = ncyc;
        do mr(SDM_FIFO, d); while (d < 128 && ncyc - t0 < 1_000_000);
        @(negedge clk); load_rst = 1; @(negedge clk); load_rst = 0;
        repeat (3) @(posedge clk);
        mr(SD_CTRL, d);     check("load reset (SD mode): SD_CTRL", d, 64'h0001_00F9);
        check("load reset: slot power off", reset_n, 1);
        check("load reset: SPI pins (CMD driven high)", {cmd_oe, cmd_o}, 2'b11);
        check("load reset: SPI pins (DAT3 = CS_n = 1, DAT1/2 high, DAT0 input)", {dat_oe, dat_o[3:1]}, 7'b1110_111);
        check("load reset: SCK low", sck, 0);
        mr(SDM_FIFO, d);    check("load reset: FIFO empty", d, 0);
        mr(SDM_BLK, d);     check("load reset: SDM_BLK", d, 64'h0001_0200);
        mr(SDM_CFG, d);     check("load reset: SDM_CFG", d, 0);
        mr(SDM_IRQ_EN, d);  check("load reset: SDM_IRQ_EN", d, 0);
        mr(SDM_BLKDONE, d); check("load reset: BLKDONE", d, 0);
        status(st);         check("load reset: SDM_STATUS idle", st & 64'hFFFF_FF13, 0);
        check("load reset: irq low", irq, 0);
        // SD_MODE = 0: nothing changes
        mw(SD_CTRL, (64'h1 << 17) | 64'd5);
        mw(SDM_ARG, 64'h55);
        @(negedge clk); load_rst = 1; @(negedge clk); load_rst = 0;
        repeat (3) @(posedge clk);
        mr(SD_CTRL, d);     check("load reset (SPI mode): SD_CTRL unchanged", d, 64'h0002_0005);
        mr(SDM_ARG, d);     check("load reset (SPI mode): SDM regs unchanged", d, 64'h55);
        check("load reset (SPI mode): slot still powered", reset_n, 0);

        // =========================================================== summary
        repeat (100) @(posedge clk);
        check("card saw no host protocol errors", card.host_err, 0);
        check("no line contention", contention, 0);
        $display("card: %0d commands, %0d blocks read, %0d written, %0d rejected",
                 card.n_cmds, card.blocks_read, card.blocks_written, card.blocks_rejected);
        if (errors == 0) $display("TB_SD PASS (%0d checks)", checks);
        else             $display("TB_SD FAIL %0d (of %0d checks)", errors, checks);
        $finish;
    end

    initial begin
        #400ms;
        $display("TB_SD FAIL timeout");
        $finish;
    end
endmodule

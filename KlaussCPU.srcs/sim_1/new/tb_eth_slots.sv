`timescale 1ns / 1ps
// tb_eth_slots — unit test for the LiteEth RX slots (HW_PERF_PLAN.md §3):
// eth_mmio_bridge + the real generated liteeth_core (Verilator, with the
// behavioural Xilinx primitives in tools/ethsim/eth_prims.sv), with frames
// injected on the RMII RX pins and TX frames captured from the RMII TX pins.
// Self-checking; prints "TB_ETH PASS" or "TB_ETH FAIL n".  No waveform dump.
// tools/ethsim/run_eth_tb.sh runs it.
//
// Phase A (2 slots, the reset value) uses only what the old 2-slot core had
// and prints every observation as an "OBS" line.  The runner also builds this
// bench with +define+OLD_CORE against the previous liteeth_core.v and
// eth_mmio_bridge.sv (git) and requires the two OBS transcripts to be
// identical: slot sequence, RX_SLOT / RX_LENGTH / RX_ERRORS values, slot
// addresses and contents, TX frame on the wire, the unused TX1 hole.
//
// The remaining phases (new core only) check ETH_RX_SLOTS (clamp, [11:8] = 8,
// program-load reset), 8 back-to-back frames landing in 8 distinct slots at
// the remapped addresses with the 9th counted in RX_ERRORS, a 3-slot wrap,
// a program load with frames pending and in the middle of a frame, and TX
// with 8 slots.
//
// MMIO accesses are driven the way the SoC does them: address / data / byte
// enables held with the strobe until ready, and the strobe still high for
// one more cycle after it (bus_splitter registers ready).
module tb_eth_slots;
    logic clk = 0, clk50 = 0, rst = 1, load_rst = 0;
    always #5  clk   = ~clk;     // 100 MHz sys
    always #10 clk50 = ~clk50;   // 50 MHz RMII REF_CLK

    mmio_if bus();
    wire [29:0] wb_adr;  wire [31:0] wb_dat_w, wb_dat_r;  wire [3:0] wb_sel;
    wire wb_we, wb_cyc, wb_stb, wb_ack, wb_err;  wire [1:0] wb_bte;  wire [2:0] wb_cti;
    wire irq, mdc, mdio, rst_n, tx_en;  wire [1:0] tx_data;
    logic crs_dv = 0;  logic [1:0] rx_data = 0;

`ifdef OLD_CORE
    eth_mmio_bridge bridge (.i_clk(clk), .i_rst(rst), .mmio(bus),
        .o_wb_adr(wb_adr), .o_wb_dat_w(wb_dat_w), .o_wb_sel(wb_sel), .o_wb_we(wb_we),
        .o_wb_cyc(wb_cyc), .o_wb_stb(wb_stb), .o_wb_bte(wb_bte), .o_wb_cti(wb_cti),
        .i_wb_dat_r(wb_dat_r), .i_wb_ack(wb_ack), .i_wb_err(wb_err));
    liteeth_core core (.sys_clock(clk), .sys_reset(rst), .interrupt(irq),
`else
    wire [3:0] nslots;  wire slot_rst;
    eth_mmio_bridge bridge (.i_clk(clk), .i_rst(rst), .mmio(bus),
        .o_wb_adr(wb_adr), .o_wb_dat_w(wb_dat_w), .o_wb_sel(wb_sel), .o_wb_we(wb_we),
        .o_wb_cyc(wb_cyc), .o_wb_stb(wb_stb), .o_wb_bte(wb_bte), .o_wb_cti(wb_cti),
        .i_wb_dat_r(wb_dat_r), .i_wb_ack(wb_ack), .i_wb_err(wb_err),
        .i_load_rst(load_rst), .o_rx_nslots(nslots), .o_rx_slot_reset(slot_rst));
    liteeth_core core (.sys_clock(clk), .sys_reset(rst), .interrupt(irq),
        .rx_nslots(nslots), .rx_slot_reset(slot_rst),
`endif
        .rmii_clocks_ref_clk(clk50), .rmii_crs_dv(crs_dv), .rmii_mdc(mdc), .rmii_mdio(mdio),
        .rmii_rst_n(rst_n), .rmii_rx_data(rx_data), .rmii_tx_data(tx_data), .rmii_tx_en(tx_en),
        .wishbone_adr(wb_adr), .wishbone_dat_w(wb_dat_w), .wishbone_dat_r(wb_dat_r),
        .wishbone_sel(wb_sel), .wishbone_we(wb_we), .wishbone_cyc(wb_cyc), .wishbone_stb(wb_stb),
        .wishbone_bte(wb_bte), .wishbone_cti(wb_cti), .wishbone_ack(wb_ack), .wishbone_err(wb_err));

    // ---------------- registers ----------------
    localparam logic [31:0] CSR = 32'hF006_0000, SRAM = 32'hF008_0000, RXS = 32'hF007_0000;
    localparam logic [15:0] CTRL_RESET = 16'h0000, CTRL_SCRATCH = 16'h0004,
        RX_SLOT = 16'h1000, RX_LENGTH = 16'h1004, RX_ERRORS = 16'h1008,
        RX_EV_STATUS = 16'h100C, RX_EV_PENDING = 16'h1010,
        TX_START = 16'h1018, TX_READY = 16'h101C, TX_SLOT = 16'h1024, TX_LENGTH = 16'h1028,
        TX_EV_PENDING = 16'h1030;

    int errors = 0, checks = 0;
    task automatic check(input string what, input longint got, input longint exp);
        checks++;
        if (got !== exp) begin
            errors++;
            if (errors < 40) $display("[%0t] MISMATCH %s: got %0h expected %0h", $time, what, got, exp);
        end
    endtask

    // ---------------- MMIO ----------------
    initial begin
        bus.write_DV = 0; bus.read_DV = 0; bus.addr = 0; bus.write_data = 0; bus.byte_en = 0;
    end
    task automatic acc(input bit wr, input logic [31:0] a, input logic [63:0] d,
                       input logic [7:0] be, output logic [63:0] q);
        @(negedge clk);
        bus.addr = a; bus.write_data = d; bus.byte_en = be;
        bus.write_DV = wr; bus.read_DV = !wr;
        do @(posedge clk); while (!bus.ready);
        q = bus.read_data;
        @(negedge clk);                    // the lingering strobe cycle
        @(negedge clk);
        bus.write_DV = 0; bus.read_DV = 0;
        @(negedge clk);
    endtask
    // 32-bit access at a 4-aligned byte address (the half selected by byte_en)
    task automatic wr32(input logic [31:0] a, input logic [31:0] d);
        logic [63:0] q;
        acc(1, a, a[2] ? {d, 32'h0} : {32'h0, d}, a[2] ? 8'hF0 : 8'h0F, q);
    endtask
    task automatic rd32(input logic [31:0] a, output logic [31:0] d);
        logic [63:0] q;
        acc(0, a, 64'h0, a[2] ? 8'hF0 : 8'h0F, q);
        d = a[2] ? q[63:32] : q[31:0];
    endtask
    function automatic logic [31:0] csr(input logic [15:0] o); return CSR | o; endfunction

    // ---------------- frames ----------------
    // Frame f (an index chosen by the test): length 60 + 7*(f % 23) bytes
    // (header + payload, no CRC), content a function of (f, byte).
    function automatic int flen(input int f); return 60 + 7 * (f % 23); endfunction
    function automatic logic [7:0] fbyte(input int f, input int j);
        if (j == 0) return 8'h02;            // a unicast dst MAC
        if (j == 12) return 8'h08;           // ethertype 0x0800
        if (j == 13) return 8'h00;
        return 8'((f * 37 + j * 11 + (j >> 8) * 5 + 3) & 8'hFF);
    endfunction
    function automatic logic [31:0] crc32_upd(input logic [31:0] c, input logic [7:0] b);
        c = c ^ {24'h0, b};
        for (int k = 0; k < 8; k++) c = c[0] ? (c >> 1) ^ 32'hEDB8_8320 : (c >> 1);
        return c;
    endfunction
    task automatic rmii_byte(input logic [7:0] b);
        for (int k = 0; k < 4; k++) begin
            @(posedge clk50); #2;
            crs_dv = 1; rx_data = b[2*k +: 2];
        end
    endtask
    // Send frame f on the RMII RX pins (preamble, SFD, frame, FCS), then the
    // 12-byte inter-frame gap.
    task automatic rmii_send(input int f);
        logic [31:0] c = 32'hFFFF_FFFF;
        for (int j = 0; j < 7; j++) rmii_byte(8'h55);
        rmii_byte(8'hD5);
        for (int j = 0; j < flen(f); j++) begin
            rmii_byte(fbyte(f, j));
            c = crc32_upd(c, fbyte(f, j));
        end
        c = ~c;
        for (int j = 0; j < 4; j++) rmii_byte(c[8*j +: 8]);
        @(posedge clk50); #2; crs_dv = 0; rx_data = 0;
        repeat (48) @(posedge clk50);
    endtask

    // RX slot k's CPU address: 0xF008_0000 + (k < 2 ? k*0x800 : 0x1000 + k*0x800)
    function automatic logic [31:0] rx_addr(input int k);
        return SRAM + ((k < 2) ? k * 32'h800 : 32'h1000 + k * 32'h800);
    endfunction

    // Check that slot k holds frame f (every byte, read as 32-bit words).
    task automatic check_slot(input int k, input int f, input bit obs);
        logic [31:0] w;
        int n = flen(f), bad = 0;
        for (int i = 0; i < (n + 3) / 4; i++) begin
            rd32(rx_addr(k) + 4 * i, w);
            for (int b = 0; b < 4 && 4 * i + b < n; b++)
                if (w[8*b +: 8] !== fbyte(f, 4 * i + b)) bad++;
            if (obs && (i < 4 || i == (n + 3) / 4 - 1)) $display("OBS slot %0d word %0d = %h", k, i, w);
        end
        check($sformatf("slot %0d holds frame %0d (bad bytes)", k, f), bad, 0);
    endtask

    // Wait for an RX event, check slot / length / content, then W1C it.
    task automatic rx_one(input int exp_slot, input int f, input bit obs);
        logic [31:0] p, s, l;
        int t = 0;
        do begin rd32(csr(RX_EV_PENDING), p); t++; end while (!p[0] && t < 2000);
        check("RX_EV_PENDING", p[0], 1);
        rd32(csr(RX_SLOT), s);
        rd32(csr(RX_LENGTH), l);
        if (obs) $display("OBS rx pending=%0d slot=%0d len=%0d", p[0], s, l);
        check("RX_SLOT", s, exp_slot);
        check("RX_LENGTH", l, flen(f));
        check_slot(s, f, obs);
        wr32(csr(RX_EV_PENDING), 1);
    endtask

    task automatic rd_obs(input string name, input logic [31:0] a, output logic [31:0] d);
        rd32(a, d);
        $display("OBS %s = %h", name, d);
    endtask

    task automatic expect_no_rx(input string what);
        logic [31:0] p;
        rd32(csr(RX_EV_PENDING), p);
        check({what, ": no RX pending"}, p[0], 0);
    endtask

    // ---------------- TX capture ----------------
    // Collects the bytes of the last frame sent on the RMII TX pins (after
    // the SFD, including the FCS).  Sampled on the REF_CLK rising edge.
    logic [7:0] txq[$];
    int tx_frames = 0;
    initial begin : tx_mon
        logic [7:0] sh;  int nd;  bit sfd;
        forever begin
            @(posedge clk50);
            if (tx_en === 1'b1) begin
                txq.delete(); sh = 0; nd = 0; sfd = 0;
                while (tx_en === 1'b1) begin
                    sh = {tx_data, sh[7:2]}; nd++;
                    if (!sfd) begin
                        if (sh == 8'hD5 && nd >= 4) begin sfd = 1; nd = 0; end
                    end else if (nd == 4) begin txq.push_back(sh); nd = 0; end
                    @(posedge clk50);
                end
                tx_frames++;
            end
        end
    end

    task automatic tx_one(input int f, input bit obs);
        logic [31:0] w, r;
        int n = flen(f), t = 0, bad = 0, nf;
        logic [31:0] c = 32'hFFFF_FFFF, fcs;
        do begin rd32(csr(TX_READY), r); t++; end while (!r[0] && t < 2000);
        for (int i = 0; i < (n + 3) / 4; i++) begin
            for (int b = 0; b < 4; b++) w[8*b +: 8] = (4 * i + b < n) ? fbyte(f, 4 * i + b) : 8'h00;
            wr32(SRAM + 32'h1000 + 4 * i, w);
        end
        rd32(SRAM + 32'h1000, w);
        check("TX0 readback", w, {fbyte(f, 3), fbyte(f, 2), fbyte(f, 1), fbyte(f, 0)});
        nf = tx_frames;
        wr32(csr(TX_SLOT), 0);
        wr32(csr(TX_LENGTH), n);
        wr32(csr(TX_START), 1);
        t = 0;
        while (tx_frames == nf && t < 200000) begin @(posedge clk); t++; end
        repeat (20) @(posedge clk);
        check("TX frame seen", tx_frames, nf + 1);
        check("TX bytes on the wire", txq.size(), n + 4);
        for (int j = 0; j < n && j < txq.size(); j++) begin
            if (txq[j] !== fbyte(f, j)) bad++;
            c = crc32_upd(c, fbyte(f, j));
        end
        c = ~c;
        if (txq.size() >= n + 4) begin
            fcs = {txq[n + 3], txq[n + 2], txq[n + 1], txq[n]};
            check("TX FCS", fcs, c);
        end
        check("TX payload bytes", bad, 0);
        if (obs) $display("OBS tx frame %0d: %0d bytes on the wire, bad=%0d", f, txq.size(), bad);
        wr32(csr(TX_EV_PENDING), 1);
    endtask

    // ---------------- test ----------------
    logic [31:0] d, err0, err1;
    int s0;
    logic [63:0] q;
    logic [31:0] wvals[12] = '{0, 1, 2, 3, 7, 8, 9, 15, 16, 32'hFFFF_FFFF, 32'h8000_0004, 32'h0000_0F06};
    initial begin
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (600) @(posedge clk);       // LiteEth's PHY reset counter (256) and CDC
        rd32(csr(CTRL_SCRATCH), d);
        check("CTRL_SCRATCH", d, 32'h1234_5678);

        // ===== Phase A: 2 slots (reset value), old/new transcript =====
        $display("OBS phase A");
        // Warm-up: the first frame after reset is not always received
        // intact (LiteEth's RMII PHY starts at 10 Mbit/s and detects 100
        // during that frame's preamble).  In this bench a 60-byte first
        // frame followed back-to-back by another vanished and left one
        // stray byte in front of the next frame, identically with the old
        // core.  So the transcript starts after one warm-up frame.
        rmii_send(999);
        repeat (200) @(posedge clk);
        s0 = 0;
        for (int i = 0; i < 4; i++) begin
            rd32(csr(RX_EV_PENDING), d);
            if (d[0]) begin
                rd32(csr(RX_SLOT), err0); rd32(csr(RX_LENGTH), err1);
                $display("OBS warm-up drained slot=%0d len=%0d", err0, err1);
                wr32(csr(RX_EV_PENDING), 1);
                s0 = s0 ^ 1;
            end
        end
        rd_obs("RX_ERRORS0", csr(RX_ERRORS), err0);
        rd_obs("RX_SLOT0", csr(RX_SLOT), d);
        // three back-to-back frames with nothing drained: 2 land, 1 dropped
        rmii_send(0); rmii_send(1); rmii_send(2);
        repeat (200) @(posedge clk);
        rd_obs("RX_ERRORS after 3", csr(RX_ERRORS), err1);
        check("2 slots: 3rd frame dropped", err1 - err0, 1);
        rx_one(s0, 0, 1);
        rx_one(s0 ^ 1, 1, 1);
        expect_no_rx("2 slots drained");
        rd_obs("RX_SLOT idle", csr(RX_SLOT), d);
        // the slot counter keeps alternating
        rmii_send(3); rx_one(s0, 3, 1);
        rmii_send(4); rx_one(s0 ^ 1, 4, 1);
        rmii_send(5); rmii_send(6);
        rx_one(s0, 5, 1);
        rx_one(s0 ^ 1, 6, 1);
        rd_obs("RX_ERRORS end A", csr(RX_ERRORS), d);
        check("2 slots: no more drops", d - err0, 1);
        // TX at 0xF008_1000; the unused TX1 hole reads as before
        tx_one(7, 1);
        rd_obs("hole 0xF008_1800", SRAM + 32'h1800, d);
        rd_obs("RX_SLOT end A", csr(RX_SLOT), d);
        $display("OBS end phase A");

`ifndef OLD_CORE
        // ===== Phase B: ETH_RX_SLOTS register =====
        rd32(RXS, d);              check("ETH_RX_SLOTS reset", d, 32'h0000_0802);
        acc(0, RXS, 0, 8'hFF, q);  check("ETH_RX_SLOTS 64-bit read", q, 64'h0000_0000_0000_0802);
        rd32(RXS + 8, d);          check("0xF007_0008 reads 0", d, 0);
        rd32(RXS + 32'h8000, d);   check("0xF007_8000 reads 0", d, 0);
        wr32(RXS + 8, 5);          rd32(RXS, d); check("gap write dropped", d, 32'h802);
        foreach (wvals[i]) begin
            automatic logic [31:0] v = wvals[i];
            automatic logic [3:0] e = (v < 2) ? 4'd2 : (v > 8) ? 4'd8 : v[3:0];
            wr32(RXS, v);
            rd32(RXS, d);
            check($sformatf("ETH_RX_SLOTS write %h", v), d, {20'h0, 4'd8, 4'h0, e});
        end
        acc(1, RXS, 64'h0000_0004_0000_0000, 8'hF0, q);   // high half only: no change
        rd32(RXS, d); check("high-half write ignored", d, 32'h808);
        acc(1, RXS, 64'h0000_0000_0000_0300, 8'h02, q);   // byte 1 only: no change
        rd32(RXS, d); check("byte-1 write ignored", d, 32'h808);
        acc(1, RXS, 64'h0000_0000_0000_0005, 8'h01, q);   // byte 0: 5
        rd32(RXS, d); check("byte-0 write", d, 32'h805);
        wr32(RXS, 3);
        @(negedge clk); load_rst = 1; @(negedge clk); load_rst = 0;
        rd32(RXS, d); check("program load resets ETH_RX_SLOTS", d, 32'h802);

        // ETH_RX_SLOTS writes flush the RX slot state: a frame pending at
        // 2 slots is dropped and the counter restarts at 0.
        rmii_send(50);
        repeat (200) @(posedge clk);
        rd32(csr(RX_SLOT), d); check("pending before ETH_RX_SLOTS write: RX_SLOT", d, 0);
        rd32(csr(RX_EV_PENDING), d); check("pending before ETH_RX_SLOTS write", d[0], 1);

        // ===== Phase C: 8 slots, 9 back-to-back frames =====
        wr32(RXS, 8);
        wr32(csr(CTRL_RESET), 1);          // the documented change sequence (a no-op here)
        repeat (600) @(posedge clk);
        rd32(RXS, d); check("ETH_RX_SLOTS survives CTRL_RESET", d, 32'h808);
        expect_no_rx("ETH_RX_SLOTS write flushed the pending frame");
        rd32(csr(RX_ERRORS), err0);
        for (int f = 0; f < 9; f++) rmii_send(100 + f);
        repeat (200) @(posedge clk);
        rd32(csr(RX_ERRORS), err1);
        check("8 slots: only the 9th dropped", err1 - err0, 1);
        // every slot still holds its own frame (no overwrite while pending)
        for (int k = 0; k < 8; k++) check_slot(k, 100 + k, 0);
        for (int k = 0; k < 8; k++) rx_one(k, 100 + k, 0);
        expect_no_rx("8 slots drained");
        // wrap 7 -> 0 and keep going
        for (int f = 0; f < 10; f++) begin rmii_send(200 + f); rx_one(f % 8, 200 + f, 0); end
        rd32(csr(RX_ERRORS), d); check("8 slots: no further drops", d - err0, 1);
        // TX still at 0xF008_1000 and does not disturb RX slots
        tx_one(9, 0);
        rmii_send(300); rx_one(2, 300, 0);
        check_slot(3, 203, 0);             // an older frame, untouched by TX staging

        // ===== Phase D: 3 slots (non-power-of-two wrap) =====
        wr32(RXS, 3);
        wr32(csr(CTRL_RESET), 1);
        repeat (600) @(posedge clk);
        rd32(csr(RX_ERRORS), err0);
        for (int f = 0; f < 4; f++) rmii_send(400 + f);
        repeat (200) @(posedge clk);
        rd32(csr(RX_ERRORS), err1);
        check("3 slots: 4th dropped", err1 - err0, 1);
        for (int k = 0; k < 3; k++) rx_one(k, 400 + k, 0);
        expect_no_rx("3 slots drained");
        for (int f = 0; f < 4; f++) begin rmii_send(410 + f); rx_one(f % 3, 410 + f, 0); end

        // ===== Phase E: program load with frames pending =====
        wr32(RXS, 8);
        wr32(csr(CTRL_RESET), 1);
        repeat (600) @(posedge clk);
        for (int f = 0; f < 5; f++) rmii_send(500 + f);
        repeat (200) @(posedge clk);
        rd32(csr(RX_SLOT), d); check("before load: RX_SLOT", d, 0);
        @(negedge clk); load_rst = 1; @(negedge clk); load_rst = 0;
        rd32(RXS, d); check("after load: ETH_RX_SLOTS", d, 32'h802);
        expect_no_rx("after load: pending frames flushed");
        rd32(csr(RX_ERRORS), err0);
        rmii_send(510); rmii_send(511); rmii_send(512);
        repeat (200) @(posedge clk);
        rd32(csr(RX_ERRORS), err1);
        check("after load: 2 slots again, 3rd dropped", err1 - err0, 1);
        rx_one(0, 510, 0);                  // the slot counter restarted at 0
        rx_one(1, 511, 0);
        expect_no_rx("after load drained");

        // ===== Phase F: program load in the middle of a frame =====
        fork
            rmii_send(600);
            begin repeat (150) @(posedge clk50); @(negedge clk); load_rst = 1; @(negedge clk); load_rst = 0; end
        join
        repeat (200) @(posedge clk);
        expect_no_rx("mid-frame load: frame discarded");
        rmii_send(601); rx_one(0, 601, 0);
        rmii_send(602); rx_one(1, 602, 0);
        rmii_send(603); rx_one(0, 603, 0);
`endif

        repeat (50) @(posedge clk);
        $display("TB_ETH checks=%0d errors=%0d", checks, errors);
        if (errors == 0) $display("TB_ETH PASS"); else $display("TB_ETH FAIL %0d", errors);
        $finish;
    end

    initial begin
        #20ms;
        $display("TB_ETH FAIL timeout");
        $finish;
    end
endmodule

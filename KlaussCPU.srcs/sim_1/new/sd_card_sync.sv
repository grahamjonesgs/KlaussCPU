`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// sd_card_sync — a small SD card on the native bus for tb_soc (Verilator):
// synchronous to the system clock, it sees CLK's edges a cycle late (the
// host drives on CLK's fall and samples on the fall after; the card samples
// on the rise and drives after the fall, as sd_card_model).  Simulation only.
//
// It starts in the transfer state (no identification), 4-bit (+SDC_W1: 1-bit),
// block length 512 (CMD16 sets it; a multiple of 8).  Commands: CMD12 (R1b),
// CMD13, CMD16, CMD17 / CMD18 (read), CMD24 / CMD25 (write); any other
// gets an R1.  Data: dword k of block b reads as P(b, k) = {b, k ^ A5A5_0000}
// (little-endian bytes, block byte 0 first on the bus) unless a write stored
// it.  A written dword is compared with P(b, k) too (wr_bad): programs write
// that pattern.  Host-protocol checks (host_err): command CRC7 and end bit,
// N_CC (8 clocks between CMD frames), write CRC16 and end bit, N_WR, the
// write start bit on every line.
//
// Plusargs: +SDC_RDCRC=<n> the n-th block read (from 0, over the run) has a
// bad CRC on DAT0; +SDC_WRTOK=<n> the n-th block written gets CRC status
// 101 (not stored); +SDC_BUSY=<clocks> write busy (8); +SDC_NAC=<clocks>
// command end to the first read block (8); +SDC_GAP=<clocks> between read
// blocks (2); +SDC_V verbose.  At the end of the run the tb prints
// "SDC: ..." with the counters.
// ---------------------------------------------------------------------------
module sd_card_sync (
    input  logic       clk,
    input  logic       sck,
    input  logic       pwr,
    inout  wire        cmd,
    inout  wire  [3:0] dat
);
    logic       cmd_o = 1'b1, cmd_oe = 1'b0;
    logic [3:0] dat_o = 4'hF, dat_oe = 4'h0;
    assign cmd = cmd_oe ? cmd_o : 1'bz;
    for (genvar i = 0; i < 4; i++) begin : g_d
        assign dat[i] = dat_oe[i] ? dat_o[i] : 1'bz;
    end

    // configuration
    int  rd_crc_n = -1, wr_tok_n = -1, wr_busy = 8, n_ac = 8, n_gap = 2;
    bit  w4 = 1'b1, verbose = 1'b0;
    int  blen = 512;
    // counters
    int  n_cmds = 0, blocks_read = 0, blocks_written = 0, wr_bad = 0, host_err = 0, rd_bad_sent = 0;
    initial begin
        void'($value$plusargs("SDC_RDCRC=%d", rd_crc_n));
        void'($value$plusargs("SDC_WRTOK=%d", wr_tok_n));
        void'($value$plusargs("SDC_BUSY=%d", wr_busy));
        void'($value$plusargs("SDC_NAC=%d", n_ac));
        void'($value$plusargs("SDC_GAP=%d", n_gap));
        if ($test$plusargs("SDC_W1")) w4 = 1'b0;
        verbose = $test$plusargs("SDC_V");
    end

    logic [63:0] wmem [longint];          // written dwords, by card byte address
    function automatic logic [63:0] f_p(input longint b, input int k);
        return {32'(b), 32'(k) ^ 32'hA5A5_0000};
    endfunction
    function automatic logic [63:0] f_rd(input longint b, input int k);
        longint a = b * 512 + k * 8;
        return wmem.exists(a) ? wmem[a] : f_p(b, k);
    endfunction
    function automatic logic [6:0] f_crc7(input logic [6:0] c, input logic b);
        logic fb; fb = b ^ c[6];
        return {c[5:3], c[2] ^ fb, c[1:0], fb};
    endfunction
    function automatic logic [15:0] f_crc16(input logic [15:0] c, input logic b);
        logic fb; fb = b ^ c[15];
        return {c[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0);
    endfunction
    task automatic herr(input string s);
        host_err++;
        if (host_err <= 10) $display("SDC: host protocol error: %s", s);
    endtask

    // CLK edges (a cycle late)
    logic sck_q = 1'b0;
    always @(posedge clk) sck_q <= sck;
    wire rise = sck && !sck_q;
    wire fall = !sck && sck_q;
    longint nclk = 0;                     // rising edges

    // ---------------- command receiver / response sender ----------------
    int          c_rx = -1;               // bits received, -1 = idle
    logic [47:0] c_f;
    longint      line_end = -100;         // the last CMD-line frame's end (rise count)
    logic [47:0] r_f;                     // response being sent
    int          r_n = 0, r_wait = 0;     // bits left; rises to wait before the first
    int          r1b_busy = 0;            // DAT0 busy clocks after the response
    bit          r1b_act = 1'b0;          // ... running
    bit          d_rel = 1'b0;            // release DAT (a transfer stopped)
    // ---------------- data engine ----------------
    typedef enum int {D_IDLE, D_RWAIT, D_RSEND, D_RGAP, D_WWAIT, D_WRECV, D_WTOK, D_WBUSY} dst_e;
    dst_e        d_st = D_IDLE;
    bit          d_multi;
    longint      d_blk;                   // the block in progress
    int          d_cnt;                   // clocks / positions within the state
    int          d_bits;                  // data units in a block (nibbles or bits)
    logic [15:0] d_crc [4];
    logic [7:0]  d_buf [0:2047];
    int          d_rdn = 0, d_wrn = 0;    // blocks read / written (for injection)
    bit          d_badcrc;
    logic [2:0]  d_tok;
    longint      d_wref;                  // rise count of the response end / the last busy

    function automatic logic [7:0] blk_byte(input longint b, input int j);
        logic [63:0] w = f_rd(b, j / 8);
        return w[8 * (j % 8) +: 8];
    endfunction

    task automatic respond(input logic [5:0] idx, input logic [31:0] arg);
        logic [6:0] c = 7'h0;
        logic [47:0] f;
        f = {2'b00, idx, arg, 8'h01};
        for (int i = 47; i >= 8; i--) c = f_crc7(c, f[i]);
        f[7:1] = c;
        r_f = f; r_n = 48; r_wait = 2;
    endtask

    function automatic logic [31:0] st32();
        logic [31:0] s = 32'h0;
        s[12:9] = (d_st == D_IDLE) ? 4'd4 : (d_st <= D_RGAP) ? 4'd5 : 4'd6;
        s[8] = (d_st == D_IDLE);
        return s;
    endfunction

    task automatic handle(input logic [47:0] f, input longint end_clk);
        logic [5:0]  idx = f[45:40];
        logic [31:0] arg = f[39:8];
        logic [6:0]  c = 7'h0;
        n_cmds++;
        for (int i = 47; i >= 8; i--) c = f_crc7(c, f[i]);
        if (f[46] !== 1'b1) herr("command transmission bit");
        if (f[0] !== 1'b1) herr("command end bit");
        if (c !== f[7:1]) herr($sformatf("CMD%0d CRC7", idx));
        if (end_clk - 47 - line_end - 1 < 8) herr($sformatf("CMD%0d: %0d clocks since the last CMD frame", idx, end_clk - 47 - line_end - 1));
        line_end = end_clk;
        if (verbose) $display("SDC: CMD%0d arg %h (data state %0d)", idx, arg, d_st);
        case (idx)
            6'd12: begin
                respond(idx, st32());
                // a write's programming busy carries on as the R1b busy
                r1b_busy = (d_st == D_WBUSY) ? d_cnt + 2 : 4;
                if (d_st != D_IDLE && d_st != D_WBUSY) d_rel = 1'b1;
                d_st = D_IDLE;
            end
            6'd16: begin blen = int'(arg); respond(idx, st32()); end
            6'd17, 6'd18: begin
                respond(idx, st32());
                d_st = D_RWAIT; d_cnt = n_ac; d_blk = arg; d_multi = (idx == 6'd18);
            end
            6'd24, 6'd25: begin
                respond(idx, st32());
                d_st = D_WWAIT; d_blk = arg; d_multi = (idx == 6'd25); d_wref = -1;
            end
            default: respond(idx, st32());
        endcase
    endtask

    always @(posedge clk) begin
        if (!pwr) begin
            c_rx = -1; r_n = 0; r1b_busy = 0; r1b_act = 1'b0; d_rel = 1'b0; d_st = D_IDLE;
            cmd_oe <= 1'b0; dat_oe <= 4'h0; cmd_o <= 1'b1; dat_o <= 4'hF;
        end else begin
            // ---------------- rising edge: sample ----------------
            if (rise) begin
                nclk++;
                if (c_rx < 0) begin
                    if (!cmd_oe && cmd === 1'b0) begin c_rx = 1; c_f = '0; c_f[47] = 1'b0; end
                end else begin
                    c_f[47 - c_rx] = cmd;
                    c_rx++;
                    if (c_rx == 48) begin c_rx = -1; handle(c_f, nclk); end
                end
                if (r_n > 0 && r_wait > 0) r_wait--;
                // data: write side samples
                case (d_st)
                    D_WWAIT: if (d_wref >= 0 || r_n == 0) begin
                        if (d_wref < 0) d_wref = line_end;
                        if (dat[0] === 1'b0) begin
                            if (w4 && dat !== 4'h0) herr($sformatf("write start bit not on all lines (%b)", dat));
                            if (nclk - d_wref - 1 < 2) herr("N_WR < 2");
                            d_st = D_WRECV; d_cnt = 0; d_bits = w4 ? 2 * blen : 8 * blen;
                            for (int i = 0; i < 4; i++) d_crc[i] = 16'h0;
                        end
                    end
                    D_WRECV: begin
                        if (d_cnt < d_bits) begin
                            if (w4) begin
                                if (d_cnt % 2 == 0) d_buf[d_cnt / 2][7:4] = dat;
                                else                d_buf[d_cnt / 2][3:0] = dat;
                                for (int i = 0; i < 4; i++) d_crc[i] = f_crc16(d_crc[i], dat[i]);
                            end else begin
                                d_buf[d_cnt / 8][7 - d_cnt % 8] = dat[0];
                                d_crc[0] = f_crc16(d_crc[0], dat[0]);
                            end
                        end else if (d_cnt < d_bits + 16) begin
                            for (int i = 0; i < 4; i++)
                                if (w4 || i == 0) d_crc[i] = f_crc16(d_crc[i], dat[i]);
                        end else begin
                            // the end bit
                            bit bad = 1'b0;
                            for (int i = 0; i < 4; i++)
                                if ((w4 || i == 0) && d_crc[i] != 16'h0) bad = 1'b1;
                            if (bad) herr($sformatf("write block %0d CRC16", d_blk));
                            if ((dat & (w4 ? 4'hF : 4'h1)) !== (w4 ? 4'hF : 4'h1)) herr("write end bit");
                            d_tok = (d_wrn == wr_tok_n) ? 3'b101 : 3'b010;
                            if (d_tok == 3'b010) begin
                                for (int k = 0; k < blen / 8; k++) begin
                                    logic [63:0] w;
                                    for (int j = 0; j < 8; j++) w[8*j +: 8] = d_buf[8*k + j];
                                    wmem[d_blk * 512 + k * 8] = w;
                                    if (w !== f_p(d_blk, k)) begin
                                        wr_bad++;
                                        if (wr_bad <= 5) $display("SDC: written block %0d dword %0d = %h, expected %h", d_blk, k, w, f_p(d_blk, k));
                                    end
                                end
                                blocks_written++;
                            end
                            d_wrn++;
                            d_st = D_WTOK; d_cnt = 0;
                        end
                        d_cnt++;
                    end
                    default: ;
                endcase
            end
            // ---------------- falling edge: drive ----------------
            if (fall) begin
                if (d_rel) begin dat_oe <= 4'h0; dat_o <= 4'hF; d_rel = 1'b0; end
                // the response, then an R1b's busy on DAT0
                if (r_n > 0 && r_wait == 0) begin
                    cmd_oe <= 1'b1; cmd_o <= r_f[r_n - 1];
                    r_n--;
                    if (r_n == 0) line_end = nclk + 1;
                end else if (cmd_oe) begin
                    cmd_oe <= 1'b0; cmd_o <= 1'b1;
                    if (r1b_busy > 0) begin r1b_act = 1'b1; dat_oe[0] <= 1'b1; dat_o[0] <= 1'b0; end
                end else if (r1b_act) begin
                    if (r1b_busy > 0) r1b_busy--;
                    if (r1b_busy == 0) begin r1b_act = 1'b0; dat_oe[0] <= 1'b0; dat_o[0] <= 1'b1; end
                end
                // the data
                case (d_st)
                    D_RWAIT, D_RGAP: begin
                        dat_oe <= 4'h0; dat_o <= 4'hF;
                        if (d_cnt > 0) d_cnt--;
                        else begin
                            // start bit
                            dat_oe <= w4 ? 4'hF : 4'h1; dat_o <= w4 ? 4'h0 : 4'hE;
                            for (int i = 0; i < 4; i++) d_crc[i] = 16'h0;
                            d_badcrc = (d_rdn == rd_crc_n);
                            d_st = D_RSEND; d_cnt = 0; d_bits = w4 ? 2 * blen : 8 * blen;
                        end
                    end
                    D_RSEND: begin
                        if (d_cnt < d_bits) begin
                            logic [3:0] u;
                            logic [7:0] b = blk_byte(d_blk, w4 ? d_cnt / 2 : d_cnt / 8);
                            u = w4 ? ((d_cnt % 2 == 0) ? b[7:4] : b[3:0]) : {3'b111, b[7 - d_cnt % 8]};
                            dat_o <= u;
                            for (int i = 0; i < 4; i++) d_crc[i] = f_crc16(d_crc[i], u[i]);
                        end else if (d_cnt < d_bits + 16) begin
                            logic [3:0] u;
                            for (int i = 0; i < 4; i++) u[i] = (w4 || i == 0) ? d_crc[i][15] : 1'b1;
                            if (d_badcrc && d_cnt == d_bits) u[0] = ~u[0];
                            dat_o <= u;
                            for (int i = 0; i < 4; i++) d_crc[i] = {d_crc[i][14:0], 1'b0};
                        end else begin
                            dat_o <= 4'hF;                // end bit
                            if (d_badcrc) rd_bad_sent++;
                            blocks_read++; d_rdn++;
                            d_blk++;
                            d_st = d_multi ? D_RGAP : D_IDLE;
                            if (!d_multi) d_rel = 1'b1;
                            d_cnt = n_gap + 1;
                        end
                        d_cnt++;
                    end
                    D_WTOK: begin                         // 2 clocks, then 0 tok[2:0] 1
                        case (d_cnt)
                            0, 1: ;
                            2: begin dat_oe[0] <= 1'b1; dat_o[0] <= 1'b0; end
                            3, 4, 5: dat_o[0] <= d_tok[5 - d_cnt];
                            6: dat_o[0] <= 1'b1;
                            default: begin
                                if (d_tok == 3'b010) begin dat_o[0] <= 1'b0; d_st = D_WBUSY; d_cnt = wr_busy; end
                                else begin dat_oe[0] <= 1'b0; d_st = d_multi ? D_WWAIT : D_IDLE; d_wref = nclk; end
                            end
                        endcase
                        if (d_st == D_WTOK) d_cnt++;
                    end
                    D_WBUSY: begin
                        if (d_cnt > 0) d_cnt--;
                        else begin
                            dat_oe[0] <= 1'b0; dat_o[0] <= 1'b1;
                            d_blk++;
                            d_st = d_multi ? D_WWAIT : D_IDLE; d_wref = nclk;
                        end
                    end
                    default: ;
                endcase
            end
        end
    end
endmodule

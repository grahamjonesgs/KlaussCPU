`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// sd_card_model — behavioural SDHC card on the native SD bus, for simulation
// only (tb_sd_native).  Not synthesisable.
//
// Commands: CMD0/2/3/6/7/8/9/12/13/16/17/18/24/25/55, ACMD6/13/22/41/51.
// CMD16 sets the length of CMD17/18/24/25 blocks (1..512; block addresses
// keep a 512-byte stride), so short blocks can be tested; ACMD22 returns the
// count of blocks written (4 bytes, big-endian).
// Responses R1/R1b/R2/R3/R6/R7 with correct CRC7 (unless injected), 1-bit and
// 4-bit data with one CRC16 per line, write CRC status token + busy, a
// sparse byte backing store (absent bytes read as f_pat(addr)).
//
// Timing: the card samples CMD/DAT on the rising edge of CLK and drives its
// outputs T_ODLY after the falling edge (default-speed card timing).
//
// Host-protocol checks (host_err counts them, each with a message): command
// start/transmission/end bits and CRC7, >= 74 clocks with CMD high after
// power-up before the first command, >= 8 clocks (N_CC/N_RC) between the
// end of one CMD-line frame and the next command, write data CRC16 per
// line and end bits, N_WR >= 2 clocks before a write start bit, DAT3 high
// at CMD0 (else the card would enter SPI mode), the start bit on all active
// lines.  Line contention (host and card driving together) is checked by the
// testbench, which sees both output enables.
//
// Error injection (testbench writes the variables; -1 = off; block numbers
// count from 0 within a data command):
//   inj_no_resp / inj_rsp_crc / inj_rsp_end / inj_rsp_idx : command index
//       answered with no response / a bad CRC7 / end bit 0 / a wrong echoed
//       index (one shot each)
//   inj_rd_crc_blk / inj_rd_end_blk : corrupt that read block's CRC (DAT0) /
//       end bit (DAT0)  (one shot)
//   inj_rd_nodata : the next read command sends no data (one shot)
//   inj_wr_badtok_blk / inj_wr_notok_blk : CRC status 101 / no token (and no
//       busy) for that write block (one shot)
//   inj_wr_longbusy_blk : busy for long_busy clocks after that block
//   inj_r1b_long : the next R1b response is followed by long_busy clocks
// ---------------------------------------------------------------------------
module sd_card_model #(parameter real T_ODLY = 6.0) (
    input  wire       clk,
    input  wire       pwr,
    inout  wire       cmd,
    inout  wire [3:0] dat
);
    logic       cmd_o = 1'b1, cmd_oe = 1'b0;
    logic [3:0] dat_o = 4'hF, dat_oe = 4'h0;
    assign cmd = cmd_oe ? cmd_o : 1'bz;
    for (genvar i = 0; i < 4; i++) begin : g_dat
        assign dat[i] = dat_oe[i] ? dat_o[i] : 1'bz;
    end

    // ---------------- configuration / injection ----------------
    int n_cr        = 2;      // clocks between command end bit and response start bit
    int n_ac        = 8;      // clocks between read command end bit and first block
    int n_gap       = 2;      // clocks between read blocks
    int wr_busy     = 6;      // write busy clocks
    int r1b_busy    = 4;      // R1b busy clocks (CMD7, CMD12 after a write)
    int long_busy   = 4000;
    int acmd41_busy_n = 2;    // ACMD41 reports busy this many times first
    int inj_no_resp = -1, inj_rsp_crc = -1, inj_rsp_end = -1, inj_rsp_idx = -1;
    int inj_rd_crc_blk = -1, inj_rd_end_blk = -1, inj_rd_nodata = 0;
    int inj_wr_badtok_blk = -1, inj_wr_notok_blk = -1, inj_wr_longbusy_blk = -1;
    int inj_r1b_long = 0;
    bit verbose     = 0;

    // ---------------- observable state ----------------
    int  host_err = 0;
    int  n_cmds = 0, blocks_read = 0, blocks_written = 0, blocks_rejected = 0;
    int  last_idx = -1;
    logic [31:0] last_arg;
    bit  w4 = 0;              // bus width
    int  blen = 512;          // CMD16 block length for memory blocks
    logic [15:0] rca = 16'h0;
    localparam logic [15:0] NEW_RCA = 16'hB37A;

    typedef enum int {S_IDLE=0, S_READY=1, S_IDENT=2, S_STBY=3, S_TRAN=4, S_DATA=5,
                      S_RCV=6, S_PRG=7, S_DIS=8} st_e;
    st_e st = S_IDLE;
    bit  app_next = 0;
    int  acmd41_n = 0;
    bit  ocr_ready = 0;

    // backing store
    logic [7:0] mem [longint];
    function automatic logic [7:0] f_pat(input longint a);
        return 8'((a * 37) ^ (a >> 9) ^ 8'h5A);
    endfunction
    function automatic logic [7:0] rd_byte(input longint a);
        return mem.exists(a) ? mem[a] : f_pat(a);
    endfunction

    // registers
    logic [127:0] cid, csd;
    logic [63:0]  scr = 64'h0235_8003_0000_0000;   // SD3.0, 1+4 bit, CMD23 (byte 0 = 0x02 first on the bus)
    logic [7:0]   sw_status [64];
    logic [7:0]   sd_status [64];

    // ---------------- CRC helpers ----------------
    function automatic logic [6:0] crc7_step(input logic [6:0] c, input logic b);
        logic fb; fb = b ^ c[6];
        return {c[5:3], c[2] ^ fb, c[1:0], fb};
    endfunction
    function automatic logic [15:0] crc16_step(input logic [15:0] c, input logic b);
        logic fb; fb = b ^ c[15];
        return {c[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0);
    endfunction
    function automatic logic [6:0] crc7_40(input logic [39:0] v);
        logic [6:0] c = 0;
        for (int i = 39; i >= 0; i--) c = crc7_step(c, v[i]);
        return c;
    endfunction
    function automatic logic [6:0] crc7_120(input logic [119:0] v);
        logic [6:0] c = 0;
        for (int i = 119; i >= 0; i--) c = crc7_step(c, v[i]);
        return c;
    endfunction

    initial begin
        cid[127:8] = 120'h03_5344_5355_3332_4780_1234_5678_0164;
        cid[7:1] = crc7_120(cid[127:8]); cid[0] = 1'b1;
        csd[127:8] = 120'h40_0E00_325B_5900_00ED_C87F_800A_4040;
        csd[7:1] = crc7_120(csd[127:8]); csd[0] = 1'b1;
        for (int i = 0; i < 64; i++) begin
            sw_status[i] = 8'(i * 3 + 1);
            sd_status[i] = 8'(255 - i);
        end
        sw_status[0] = 8'h00; sw_status[1] = 8'h64;  // max current
    end

    // ---------------- clock bookkeeping ----------------
    longint clk_n = 0;          // rising edges while powered
    longint pwr_clk = 0;        // clocks since power-up with CMD high, before the first command
    bit     first_cmd = 1;
    longint line_end = -100;    // clk_n of the last CMD-line frame's end bit
    always @(posedge clk) if (pwr) begin
        clk_n++;
        if (first_cmd && cmd === 1'b1) pwr_clk++;
    end

    task automatic herr(input string s);
        host_err++;
        $display("[%0t] SDCARD host protocol error: %s", $time, s);
    endtask

    // ---------------- line drivers (T_ODLY after the falling edge) ----------------
    task automatic drv_cmd(input logic v);
        cmd_o <= #(T_ODLY) v; cmd_oe <= #(T_ODLY) 1'b1;
    endtask
    task automatic rel_cmd();
        cmd_oe <= #(T_ODLY) 1'b0; cmd_o <= #(T_ODLY) 1'b1;
    endtask
    task automatic drv_dat(input logic [3:0] v, input logic [3:0] m);
        for (int i = 0; i < 4; i++) if (m[i]) begin
            dat_o[i] <= #(T_ODLY) v[i]; dat_oe[i] <= #(T_ODLY) 1'b1;
        end
    endtask
    task automatic rel_dat(input logic [3:0] m);
        for (int i = 0; i < 4; i++) if (m[i]) begin
            dat_oe[i] <= #(T_ODLY) 1'b0; dat_o[i] <= #(T_ODLY) 1'b1;
        end
    endtask

    // ---------------- power ----------------
    always @(pwr) begin
        st = S_IDLE; app_next = 0; acmd41_n = 0; ocr_ready = 0; w4 = 0; rca = 0;
        first_cmd = 1; pwr_clk = 0;
        cmd_oe = 0; dat_oe = 0;
    end

    // ---------------- responses ----------------
    function automatic logic [31:0] status32();
        logic [31:0] s = 0;
        s[12:9] = 4'(st);
        s[8]    = (st == S_TRAN);
        s[5]    = app_next;
        return s;
    endfunction

    // send n bits of v[n-1:0], MSB first, N_CR clocks after the command
    task automatic send_bits(input logic [135:0] v, input int n);
        repeat (n_cr) @(posedge clk);
        for (int i = n - 1; i >= 0; i--) begin
            @(negedge clk); drv_cmd(v[i]);
        end
        @(posedge clk); line_end = clk_n;
        @(negedge clk); rel_cmd();
    endtask

    task automatic send_r48(input logic [5:0] idx, input logic [31:0] arg, input bit nocrc = 0);
        logic [47:0] f;
        f[47:46] = 2'b00;
        f[45:40] = (inj_rsp_idx == idx) ? idx ^ 6'h01 : idx;
        f[39:8]  = arg;
        f[7:1]   = nocrc ? 7'h7F : crc7_40(f[47:8]);
        if (nocrc) f[45:40] = 6'h3F;
        if (inj_rsp_crc == idx) begin f[1] = ~f[1]; inj_rsp_crc = -1; end
        f[0] = (inj_rsp_end == idx) ? 1'b0 : 1'b1;
        if (inj_rsp_end == idx) inj_rsp_end = -1;
        if (inj_rsp_idx == idx) inj_rsp_idx = -1;
        send_bits({88'h0, f}, 48);
    endtask

    task automatic send_r2(input logic [5:0] idx, input logic [127:0] r);
        logic [135:0] f;
        f[135:134] = 2'b00; f[133:128] = 6'h3F; f[127:0] = r;
        if (inj_rsp_crc == idx) begin f[1] = ~f[1]; inj_rsp_crc = -1; end
        if (inj_rsp_end == idx) begin f[0] = 1'b0; inj_rsp_end = -1; end
        send_bits(f, 136);
    endtask

    task automatic r1b_busy_after(input int n);
        if (n <= 0) return;
        @(negedge clk);                   // busy from the second clock after the end bit
        @(negedge clk); drv_dat(4'h0, 4'h1);
        repeat (n) @(negedge clk);
        rel_dat(4'h1);
    endtask

    // ---------------- data: read ----------------
    bit rd_active = 0, wr_active = 0, wr_busy_phase = 0, stop_req = 0;

    task automatic send_block(input logic [7:0] b[], input int len, input bit bad_crc, input bit bad_end);
        logic [15:0] c [4];
        logic [3:0]  lines;
        lines = w4 ? 4'hF : 4'h1;
        for (int i = 0; i < 4; i++) c[i] = 0;
        @(negedge clk); drv_dat(4'h0, lines);                  // start bit
        for (int k = 0; k < len; k++) begin
            if (w4) begin
                for (int h = 1; h >= 0; h--) begin
                    logic [3:0] nib; nib = h ? b[k][7:4] : b[k][3:0];
                    @(negedge clk); drv_dat(nib, 4'hF);
                    for (int i = 0; i < 4; i++) c[i] = crc16_step(c[i], nib[i]);
                end
            end else begin
                for (int j = 7; j >= 0; j--) begin
                    @(negedge clk); drv_dat({3'b111, b[k][j]}, 4'h1);
                    c[0] = crc16_step(c[0], b[k][j]);
                end
            end
        end
        if (bad_crc) c[0][3] = ~c[0][3];
        for (int j = 15; j >= 0; j--) begin
            @(negedge clk); drv_dat({c[3][j], c[2][j], c[1][j], c[0][j]}, lines);
        end
        @(negedge clk); drv_dat(bad_end ? 4'hE : 4'hF, lines);   // end bit
        @(negedge clk); rel_dat(lines);
    endtask

    task automatic rd_sender(input longint blk, input bit multi, input int kind);
        // kind 0: memory blocks, 1: SCR (8 B), 2: CMD6 status (64 B), 3: SD status (64 B),
        // 4: ACMD22 blocks written (4 B)
        int nb = 0;
        int nwr = blocks_written;
        repeat (n_ac) @(posedge clk);
        forever begin
            logic [7:0] b[];
            int len;
            len = (kind == 0) ? blen : (kind == 1) ? 8 : (kind == 4) ? 4 : 64;
            b = new[len];
            for (int k = 0; k < len; k++)
                case (kind)
                    0: b[k] = rd_byte((blk + nb) * 512 + k);
                    1: b[k] = scr[63 - 8*k -: 8];
                    2: b[k] = sw_status[k];
                    4: b[k] = 8'(nwr >> (24 - 8 * k));
                    default: b[k] = sd_status[k];
                endcase
            send_block(b, len, nb == inj_rd_crc_blk, nb == inj_rd_end_blk);
            if (nb == inj_rd_crc_blk) inj_rd_crc_blk = -1;
            if (nb == inj_rd_end_blk) inj_rd_end_blk = -1;
            nb++; blocks_read++;
            if (!multi) break;
            repeat (n_gap) @(posedge clk);
        end
    endtask

    task automatic start_read(input longint blk, input bit multi, input int kind);
        stop_req = 0;
        if (inj_rd_nodata) begin inj_rd_nodata = 0; st = S_TRAN; return; end
        fork begin
            rd_active = 1;
            fork
                rd_sender(blk, multi, kind);
                begin wait (stop_req); repeat (2) @(posedge clk); end
            join_any
            disable fork;
            @(negedge clk); rel_dat(4'hF);
            rd_active = 0;
            if (st == S_DATA) st = S_TRAN;
        end join_none
    endtask

    // ---------------- data: write ----------------
    longint wr_ref;     // clk_n of the end of the response / the last busy

    task automatic wr_receiver(input longint blk, input bit multi);
        int nb = 0;
        forever begin
            logic [7:0]  b [512];
            logic [15:0] c [4], rc [4];
            logic [3:0]  lines;
            logic [2:0]  tok;
            int busy;
            lines = w4 ? 4'hF : 4'h1;
            do @(posedge clk); while (dat[0] !== 1'b0);
            if (w4 && dat !== 4'h0) herr($sformatf("write start bit not on all lines (%b)", dat));
            if (clk_n - wr_ref - 1 < 2) herr($sformatf("N_WR %0d < 2", clk_n - wr_ref - 1));
            for (int i = 0; i < 4; i++) begin c[i] = 0; rc[i] = 0; end
            for (int k = 0; k < blen; k++) begin
                if (w4) begin
                    logic [3:0] hi, lo;
                    @(posedge clk); hi = dat; for (int i = 0; i < 4; i++) c[i] = crc16_step(c[i], hi[i]);
                    @(posedge clk); lo = dat; for (int i = 0; i < 4; i++) c[i] = crc16_step(c[i], lo[i]);
                    b[k] = {hi, lo};
                end else begin
                    for (int j = 7; j >= 0; j--) begin
                        @(posedge clk); b[k][j] = dat[0]; c[0] = crc16_step(c[0], dat[0]);
                    end
                end
            end
            for (int j = 15; j >= 0; j--) begin
                @(posedge clk); for (int i = 0; i < 4; i++) rc[i][j] = dat[i];
            end
            for (int i = 0; i < 4; i++)
                if (lines[i] && rc[i] !== c[i]) herr($sformatf("write block %0d DAT%0d CRC16 %h, expected %h", nb, i, rc[i], c[i]));
            @(posedge clk);
            if ((dat & lines) !== lines) herr($sformatf("write block %0d end bit %b", nb, dat));
            // CRC status token, two clocks after the end bit
            if (nb == inj_wr_notok_blk) begin
                inj_wr_notok_blk = -1;
                blocks_rejected++;
                wr_ref = clk_n;
            end else begin
                wr_busy_phase = 1;
                tok = (nb == inj_wr_badtok_blk) ? 3'b101 : 3'b010;
                repeat (2) @(posedge clk);
                @(negedge clk); drv_dat(4'h0, 4'h1);
                for (int j = 2; j >= 0; j--) begin @(negedge clk); drv_dat({3'b111, tok[j]}, 4'h1); end
                @(negedge clk); drv_dat(4'hF, 4'h1);
                if (tok == 3'b010) begin
                    for (int k = 0; k < blen; k++) mem[(blk + nb) * 512 + k] = b[k];
                    blocks_written++;
                    busy = (nb == inj_wr_longbusy_blk) ? long_busy : wr_busy;
                    if (nb == inj_wr_longbusy_blk) inj_wr_longbusy_blk = -1;
                    @(negedge clk); drv_dat(4'h0, 4'h1);
                    repeat (busy) @(negedge clk);
                end else begin
                    inj_wr_badtok_blk = -1;
                    blocks_rejected++;
                    @(negedge clk);
                end
                rel_dat(4'h1);
                @(posedge clk); wr_ref = clk_n;
                wr_busy_phase = 0;
            end
            nb++;
            if (!multi) break;
        end
    endtask

    task automatic start_write(input longint blk, input bit multi);
        stop_req = 0;
        wr_ref = line_end;
        fork begin
            wr_active = 1;
            fork
                wr_receiver(blk, multi);
                begin wait (stop_req && !wr_busy_phase); end
            join_any
            disable fork;
            wr_busy_phase = 0;
            wr_active = 0;
            if (st == S_RCV) st = S_TRAN;
        end join_none
    endtask

    // ---------------- command receiver ----------------
    initial begin
        forever begin
            @(posedge clk);
            if (pwr && !cmd_oe && cmd === 1'b0) begin
                logic [47:0] f;
                f[47] = 1'b0;
                for (int i = 46; i >= 0; i--) begin @(posedge clk); f[i] = cmd; end
                handle(f);
            end
        end
    end

    task automatic handle(input logic [47:0] f);
        logic [5:0]  idx;
        logic [31:0] arg;
        bit app;
        longint gap;
        idx = f[45:40]; arg = f[39:8];
        n_cmds++;
        gap = (clk_n - 47) - line_end - 1;
        if (f[46] !== 1'b1)  herr("command transmission bit");
        if (f[0] !== 1'b1)   herr("command end bit");
        if (crc7_40(f[47:8]) !== f[7:1]) herr($sformatf("CMD%0d CRC7 %h, expected %h", idx, f[7:1], crc7_40(f[47:8])));
        if (first_cmd && pwr_clk < 74) herr($sformatf("only %0d clocks before the first command", pwr_clk));
        if (!first_cmd && gap < 8) herr($sformatf("CMD%0d: %0d clocks since the last CMD frame (N_CC/N_RC >= 8)", idx, gap));
        first_cmd = 0;
        line_end = clk_n;
        last_idx = idx; last_arg = arg;
        app = app_next; app_next = 0;
        if (verbose) $display("[%0t] SDCARD %sCMD%0d arg %h state %0d", $time, app ? "A" : "", idx, arg, st);
        if (inj_no_resp == idx) begin inj_no_resp = -1; return; end

        if (app) begin
            case (idx)
                6'd6: if (st == S_TRAN) begin
                    w4 = (arg[1:0] == 2'b10);
                    app_next = 0; send_r48(idx, status32() | 32'h20); return;
                end
                6'd13: if (st == S_TRAN) begin
                    logic [31:0] s; s = status32() | 32'h20;
                    st = S_DATA; start_read(0, 0, 3); send_r48(idx, s); return;
                end
                6'd22: if (st == S_TRAN) begin
                    logic [31:0] s; s = status32() | 32'h20;
                    st = S_DATA; start_read(0, 0, 4); send_r48(idx, s); return;
                end
                6'd41: if (st == S_IDLE || st == S_READY) begin
                    acmd41_n++;
                    ocr_ready = (acmd41_n > acmd41_busy_n);
                    if (ocr_ready) st = S_READY;
                    send_r48(6'h3F, {ocr_ready, 1'b1, 6'b0, 24'hFF8000}, 1);
                    return;
                end
                6'd51: if (st == S_TRAN) begin
                    logic [31:0] s; s = status32() | 32'h20;
                    st = S_DATA; start_read(0, 0, 1); send_r48(idx, s); return;
                end
                default: ;
            endcase
            // not an app command here: fall through as a normal command
        end

        case (idx)
            6'd0: begin
                if (dat[3] !== 1'b1) herr("CMD0 with DAT3 low (SPI mode)");
                st = S_IDLE; acmd41_n = 0; ocr_ready = 0; w4 = 0; rca = 0;
            end
            6'd2: if (st == S_READY) begin send_r2(idx, cid); st = S_IDENT; end
            6'd3: if (st == S_IDENT || st == S_STBY) begin
                rca = NEW_RCA;
                st = S_STBY;
                send_r48(idx, {rca, 3'b000, 13'(status32())});
            end
            6'd6: if (st == S_TRAN) begin
                logic [31:0] s; s = status32();
                st = S_DATA; start_read(0, 0, 2); send_r48(idx, s);
            end
            6'd7: begin
                if (arg[31:16] == rca && rca != 0) begin
                    send_r48(idx, status32());
                    st = S_TRAN;
                    r1b_busy_after(inj_r1b_long ? long_busy : r1b_busy);
                    inj_r1b_long = 0;
                end else st = S_STBY;              // deselect: no response
            end
            6'd8: if (st == S_IDLE) send_r48(idx, {20'h0, arg[11:0]});
            6'd9: if (st == S_STBY && arg[31:16] == rca) send_r2(idx, csd);
            6'd12: begin
                bit was_wr;
                was_wr = wr_active;
                if (rd_active || wr_active) stop_req = 1;
                send_r48(idx, status32());
                st = S_TRAN;
                if (was_wr || inj_r1b_long) r1b_busy_after(inj_r1b_long ? long_busy : r1b_busy);
                inj_r1b_long = 0;
            end
            6'd13: if (arg[31:16] == rca) send_r48(idx, status32());
            6'd16: if (st == S_TRAN) begin
                if (arg >= 1 && arg <= 512) blen = arg;
                send_r48(idx, status32());
            end
            6'd17, 6'd18: if (st == S_TRAN) begin
                // data is timed from the command's end bit (N_AC), so with
                // a small n_ac it starts before the response has finished
                logic [31:0] s; s = status32();
                st = S_DATA; start_read(arg, idx == 6'd18, 0); send_r48(idx, s);
            end
            6'd24, 6'd25: if (st == S_TRAN) begin
                send_r48(idx, status32()); st = S_RCV;
                start_write(arg, idx == 6'd25);
            end
            6'd55: begin
                app_next = 1;
                send_r48(idx, status32());
            end
            default: if (verbose) $display("SDCARD: CMD%0d ignored", idx);
        endcase
    endtask
endmodule

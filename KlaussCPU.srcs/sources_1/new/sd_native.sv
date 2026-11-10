`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// sd_native — native SD-bus host engine (1-bit / 4-bit, default speed), the
// SD_MODE = 1 half of the SD controller.  Instantiated by sd_spi, which owns
// SD_CTRL (clk_div, pwr_en, SD_MODE), the SPI shifter, SD_BUF and the pin
// mux.  Register map, sequencing and FIFO rules: SD_NATIVE_PLAN.md §4
// (frozen v1) and MMIO_MAP.md "SD card".
//
// Everything runs on i_Clk; there is no new clock domain.
//
//   Clock      CLK = i_Clk / (2·(clk_div+1)), generated as a register
//              (r_clk).  It runs only while it is needed (CLK_ON, a
//              command, a data phase, a busy wait, 8 trailing clocks) and
//              parks LOW.  A data phase that cannot go on (read: no room
//              for another whole block; write: no whole block in the FIFO)
//              parks it between blocks — flow control.
//   Drive      outputs change on the fabric edge that drives CLK low (the
//              card samples on the rising edge, half a period later).
//   Sample     inputs go through one (IOB) flip-flop on every edge; the
//              value captured on the edge that drives CLK low — the last
//              moment before the card's next output change, which in
//              default speed follows the falling edge (tODLY <= 14 ns) —
//              is used the cycle after (ev_samp).  At 25 MHz that leaves
//              ~20 ns of setup and the output path delay as hold.
//
//   FIFO       512 × 64 bits (one RAMB36), first-word-fall-through head
//              register.  Read blocks are written behind a shadow pointer
//              (r_wp) and become visible (r_wc) only when the block's CRC
//              and end bits check good; a bad or aborted block is rolled
//              back.  Little-endian: byte 0 of the block in [7:0] of the
//              first word.
//////////////////////////////////////////////////////////////////////////////////

module sd_native (
    input  logic        i_Clk,
    input  logic        i_rst,           // synchronous, active high
    input  logic        i_sd_mode,       // SD_CTRL[18]
    input  logic [15:0] i_clk_div,       // SD_CTRL[15:0]

    // MMIO (the whole SD device's strobes; the window is decoded here)
    input  logic        i_wr_dv,
    input  logic        i_rd_dv,
    input  logic [15:0] i_addr,
    input  logic [63:0] i_wdata,
    output logic [63:0] o_rdata,         // for the address of the previous cycle; 0 outside 0x100..0x1F8
    output logic        o_irq,           // registered level

    // pins (the parent muxes them with the SPI path)
    output logic        o_clk,
    output logic        o_cmd,
    output logic        o_cmd_oe,
    output logic [3:0]  o_dat,
    output logic [3:0]  o_dat_oe,
    input  logic        i_cmd,
    input  logic [3:0]  i_dat,
    input  logic        i_cd
);

    // -----------------------------------------------------------------------
    // Register offsets (0x100..0x1F8; addr[15:9] = 0, addr[8] = 1)
    // -----------------------------------------------------------------------
    localparam logic [7:0] O_CMD     = 8'h00, O_ARG    = 8'h08, O_RESP0 = 8'h10,
                           O_RESP1   = 8'h18, O_BLK    = 8'h20, O_STATUS = 8'h28,
                           O_IRQEN   = 8'h30, O_TMO    = 8'h38, O_CFG    = 8'h40,
                           O_FIFO    = 8'h48, O_DMA    = 8'h50, O_BLKDONE = 8'h58,
                           O_DATA    = 8'h80, O_CAPS   = 8'hF8;

    localparam logic [63:0] CAPS = 64'h0000_0000_0008_0101;   // v1, 4-bit, no HS, 8 blocks

    localparam int NCR_MAX   = 66;   // response start bit within N_CR (64) clocks, plus slack
    localparam int TOK_WIN   = 8;    // write CRC token start bit within 8 clocks of the end bit
    localparam int BUSY_SKIP = 4;    // samples ignored before testing DAT0 for the end of busy

    // sticky error bits: SDM_STATUS[26:16] = r_err[10:0]
    localparam int E_CMD_TMO = 0, E_CMD_CRC = 1, E_CMD_END = 2, E_CMD_IDX = 3,
                   E_DAT_TMO = 4, E_DAT_CRC = 5, E_DAT_END = 6, E_WR_CRC  = 7,
                   E_WR_NOTOK = 8, E_BUSY_TMO = 9, E_FIFO = 10;

    // -----------------------------------------------------------------------
    // Functions
    // -----------------------------------------------------------------------
    function automatic logic [6:0] f_crc7(input logic [6:0] c, input logic b);
        logic fb;
        fb = b ^ c[6];
        return {c[5:3], c[2] ^ fb, c[1:0], fb};
    endfunction

    function automatic logic [15:0] f_crc16(input logic [15:0] c, input logic b);
        logic fb;
        fb = b ^ c[15];
        return {c[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0000);
    endfunction

    function automatic logic [63:0] f_bswap(input logic [63:0] x);
        return {x[7:0], x[15:8], x[23:16], x[31:24], x[39:32], x[47:40], x[55:48], x[63:56]};
    endfunction

    // -----------------------------------------------------------------------
    // Pin inputs: one flip-flop each (IOB), sampled every edge.  CD gets a
    // two-flop synchroniser (asynchronous mechanical switch).
    // -----------------------------------------------------------------------
    (* IOB = "TRUE" *) logic       r_cmd_in;
    (* IOB = "TRUE" *) logic [3:0] r_dat_in;
    (* ASYNC_REG = "TRUE" *) logic [1:0] r_cd_sync;
    always_ff @(posedge i_Clk) begin
        r_cmd_in  <= i_cmd;
        r_dat_in  <= i_dat;
        r_cd_sync <= {r_cd_sync[0], i_cd};
    end

    // -----------------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------------
    logic [31:0] r_arg;
    logic [63:0] r_resp0, r_resp1;
    logic [11:2] r_blksz;            // block size in 4-byte units (a multiple of 4, §4.3)
    logic [15:0] r_blkcnt;           // 0 = open-ended
    logic        r_cmd_done, r_data_done;
    logic [10:0] r_err;
    logic [26:0] r_irq_en;           // only [4], [9:8], [26:16] are kept
    logic [31:0] r_tmo_data, r_tmo_busy;
    logic        r_cfg_w4, r_cfg_clkon;
    logic [15:0] r_blkdone;

    // -----------------------------------------------------------------------
    // MMIO inputs, registered first: the raw address comes through the
    // SoC's st.SM address mux, which must not reach the command / FIFO
    // decode (it was a failing path).  An access holds its strobe and
    // address for at least three cycles, so acting one cycle later is
    // invisible; read data for an access is valid in its second cycle,
    // which is when the SoC samples it (combinational from r_in_addr).
    // -----------------------------------------------------------------------
    logic        r_in_wr, r_in_rd;
    logic [15:0] r_in_addr;
    logic [63:0] r_in_wdata;
    always_ff @(posedge i_Clk) begin
        if (i_rst) begin
            r_in_wr <= 1'b0;
            r_in_rd <= 1'b0;
        end else begin
            r_in_wr <= i_wr_dv;
            r_in_rd <= i_rd_dv;
        end
        r_in_addr  <= i_addr;
        r_in_wdata <= i_wdata;
    end

    // -----------------------------------------------------------------------
    // MMIO strobes.  A store holds write_DV for several cycles, and stores
    // to different addresses may keep it high throughout: side effects with
    // history act on the first cycle of each access only (as qspi_ctrl).
    // -----------------------------------------------------------------------
    wire       w_win    = (r_in_addr[15:9] == 7'd0) && r_in_addr[8];
    wire [7:0] w_off    = r_in_addr[7:0];
    logic        r_wr_q, r_rd_q;
    logic [15:0] r_waddr_q, r_raddr_q;
    wire w_new_wr = r_in_wr && (!r_wr_q || (r_in_addr != r_waddr_q));
    wire w_new_rd = r_in_rd && (!r_rd_q || (r_in_addr != r_raddr_q));
    wire w_wr     = r_in_wr && w_win;                 // level: plain register writes
    wire w_wr1    = w_new_wr && w_win;                 // first cycle: side effects

    wire w_cmd_wr  = w_wr1 && (w_off == O_CMD);
    wire w_push    = w_wr1 && (w_off == O_DATA);
    wire w_fifo_rs = w_wr1 && (w_off == O_FIFO) && r_in_wdata[31];
    wire w_w1c     = w_wr1 && (w_off == O_STATUS);

    // -----------------------------------------------------------------------
    // Clock generator
    // -----------------------------------------------------------------------
    logic        r_clk;
    logic [15:0] r_div_cnt;
    logic        r_samp;            // ev_fall delayed one cycle: r_*_in holds the sample
    logic [3:0]  r_trail;           // trailing clocks (N_RC / N_CC)
    logic        w_run;
    wire w_tick  = (r_div_cnt >= i_clk_div);
    wire ev_fall = r_clk && w_tick;
    assign o_clk = r_clk;

    // -----------------------------------------------------------------------
    // Command engine state
    // -----------------------------------------------------------------------
    typedef enum logic [2:0] {C_IDLE, C_WAIT, C_SEND, C_RWAIT, C_RECV, C_BUSY} cstate_t;
    cstate_t     r_c_state;
    logic [39:0] r_c_tx;
    logic [6:0]  r_c_crc;
    logic [7:0]  r_c_cnt;
    logic [5:0]  r_c_idx;
    logic [2:0]  r_c_rtype;         // 0 none, 1 R1, 2 R1b, 3 R2, 4 R3
    logic [127:0] r_c_sh;
    logic [31:0] r_c_busy;
    logic        r_c_ok_p, r_c_err_p;    // response finished OK / with an error (one-cycle pulses)

    // -----------------------------------------------------------------------
    // Data engine state
    // -----------------------------------------------------------------------
    typedef enum logic [3:0] {D_IDLE, D_RD_WCMD, D_RD_START, D_RD_DATA, D_RD_CRC, D_RD_HOLD,
                              D_WR_WRSP, D_WR_GAP, D_WR_WAIT, D_WR_DATA, D_WR_CRC, D_WR_END,
                              D_WR_REL, D_WR_TOK, D_WR_TST, D_WR_BUSY} dstate_t;
    dstate_t     r_d_state;
    logic        r_d_w4;
    logic [9:0]  r_d_blkw;          // FIFO words per block, ceil(size / 8)
    logic [9:0]  r_d_sz4;           // block size in 4-byte units (latched)
    logic [5:0]  r_d_wpos;          // unit position within the current word
    logic [15:0] r_d_cnt;           // block count, 0 = open-ended
    logic [14:0] r_d_units;         // bits (1-bit) or nibbles (4-bit) left in the block
    logic [4:0]  r_d_aux;
    logic [3:0]  r_d_tok;
    logic [15:0] r_d_crc [4];
    logic [63:0] r_d_sh;            // receive / transmit word shifter
    logic [31:0] r_d_tmo;

    // -----------------------------------------------------------------------
    // FIFO
    // -----------------------------------------------------------------------
    (* ram_style = "block" *) logic [63:0] r_mem [0:511];
    logic [63:0] r_mem_q;
    logic [9:0]  r_wp;              // write pointer (shadow, includes an unchecked read block)
    logic [9:0]  r_wc;              // committed write pointer
    logic [9:0]  r_rp;              // read pointer (words popped)
    logic [9:0]  r_mrp;             // memory fetch pointer
    logic [63:0] r_head;
    logic        r_head_v, r_fetch;
    logic [25:0] r_push_words;      // words pushed since the FIFO reset (write FIFO_THRESH)
    logic [25:0] r_d_target;        // block count × latched block size, in words
    logic        r_d_tgt_stale;     // r_d_target is one cycle behind the latch
    logic        r_rd_room;         // room for one more whole read block (registered)
    logic [1:0]  r_dir;             // direction latch (§4.5): NONE / READ / WRITE
    localparam logic [1:0] DIR_NONE = 2'd0, DIR_READ = 2'd1, DIR_WRITE = 2'd2;

    wire [9:0]  w_level = r_wc - r_rp;
    wire [9:0]  w_used  = r_wp - r_rp;
    // a size field of 0 (or 1..3 written) acts as 8 bytes; a block takes
    // ceil(size / 8) words, the last one half full when size mod 8 = 4
    wire [9:0]  w_sz4   = (r_blksz == 10'd0) ? 10'd2 : r_blksz;
    wire [9:0]  w_blkw  = 10'((11'(w_sz4) + 11'd1) >> 1);
    wire        w_full  = (w_used == 10'd512);

    // -----------------------------------------------------------------------
    // Engine helper terms
    // -----------------------------------------------------------------------
    wire w_c_active = (r_c_state != C_IDLE);
    wire w_d_active = (r_d_state != D_IDLE);
    wire w_d_read   = (r_d_state == D_RD_WCMD) || (r_d_state == D_RD_START) || (r_d_state == D_RD_DATA)
                   || (r_d_state == D_RD_CRC)  || (r_d_state == D_RD_HOLD);
    wire w_d_write  = w_d_active && !w_d_read;

    // a command accepted this cycle (§4.8: ignored while CMD_BUSY, while
    // SD_MODE = 0, and — with the data flag — while DATA_BUSY)
    wire       w_cmd_acc  = w_cmd_wr && i_sd_mode && !w_c_active && !(r_in_wdata[11] && w_d_active);
    wire [2:0] w_rtype_in = (r_in_wdata[10:8] > 3'd4) ? 3'd0 : r_in_wdata[10:8];
    wire       w_acc_data = w_cmd_acc && r_in_wdata[11];
    wire       w_acc_stop = w_cmd_acc && r_in_wdata[13];

    // the last unit of a FIFO word: a full word, or the block's last unit
    // (which ends a half word when size mod 8 = 4)
    wire w_rx_wordb = (r_d_wpos == (r_d_w4 ? 6'd15 : 6'd63)) || (r_d_units == 15'd1);
    wire w_tx_wordb = w_rx_wordb;
    wire w_rx_half  = (r_d_units == 15'd1) && r_d_sz4[0];

    // receive: next shifter value and the end-of-block checks
    wire [63:0] w_rx_next = r_d_w4 ? {r_d_sh[59:0], r_dat_in} : {r_d_sh[62:0], r_dat_in[0]};
    wire w_blk_end_ok = r_d_w4 ? (r_dat_in == 4'hF) : r_dat_in[0];
    wire w_blk_crc_ok = r_d_w4 ? ((r_d_crc[0] | r_d_crc[1] | r_d_crc[2] | r_d_crc[3]) == 16'h0)
                               : (r_d_crc[0] == 16'h0);
    wire w_rd_last_blk = (r_d_cnt != 16'd0) && (r_blkdone + 16'd1 == r_d_cnt);

    // write: a whole block in the FIFO and the head word ready
    wire w_blk_ready = (w_level >= r_d_blkw) && r_head_v;

    // FIFO side effects
    wire w_eng_we  = r_samp && (r_d_state == D_RD_DATA) && w_rx_wordb;
    wire w_eng_pop = ev_fall && (((r_d_state == D_WR_WAIT) && w_blk_ready)
                              || ((r_d_state == D_WR_DATA) && (r_d_units != 15'd1) && w_tx_wordb));
    wire w_cpu_pop = w_new_rd && w_win && (w_off == O_DATA) && r_head_v;
    wire w_pop     = w_cpu_pop || w_eng_pop;
    wire w_fetch   = (r_mrp != r_wc) && !r_fetch && (!r_head_v || w_pop);
    wire w_push_ok = w_push && !w_full && !w_eng_we;

    // Flow control: park CLK between blocks.  The read decision is also
    // taken combinationally on the end bit's sample, so that even at
    // clk_div = 0 no clock follows the end bit.
    wire w_rd_hold_now = r_samp && (r_d_state == D_RD_CRC) && (r_d_aux == 5'd16)
                         && w_blk_end_ok && w_blk_crc_ok && !w_rd_last_blk && !r_rd_room;
    wire w_hold = (r_d_state == D_RD_HOLD) || ((r_d_state == D_WR_WAIT) && !w_blk_ready) || w_rd_hold_now;
    assign w_run = i_sd_mode && (w_c_active || (!w_hold && (r_cfg_clkon || w_d_active || (r_trail != 4'd0))));

    // live status.  FIFO_THRESH (§4.5) follows the direction latch: 0 for
    // NONE (after a FIFO reset, so a write pre-fill never raises it).
    // The block size (and count) are those latched by the data command
    // (§4.8): SDM_BLK writes take effect at the next data command.
    wire w_thresh_rd = (r_dir == DIR_READ) && (w_level >= r_d_blkw);
    wire w_thresh_wr = (r_dir == DIR_WRITE) && w_d_write
                       && ((11'd512 - {1'b0, w_level}) >= {1'b0, r_d_blkw})
                       && !r_d_tgt_stale && ((r_d_cnt == 16'd0) || (r_push_words < r_d_target));
    wire w_thresh    = w_thresh_rd || w_thresh_wr;
    wire w_err_any   = |r_err;
    wire [31:0] w_status = {w_err_any, 4'b0, r_err, 6'b0, r_data_done, r_cmd_done,
                            3'b0, w_thresh, ~r_cd_sync[1], r_dat_in[0], w_d_active, w_c_active};
    // CARD_PRESENT = NOT the CD pin: the Nexys A7 slot's card-detect switch
    // pulls SD_CD low with a card inserted (measured on the board, 2026-11-10;
    // SD_STATUS[1] keeps reporting the raw pin for compatibility).
    localparam logic [31:0] IRQ_MASK = 32'h07FF_0310;

    // -----------------------------------------------------------------------
    // FIFO memory (simple dual port, synchronous read)
    // -----------------------------------------------------------------------
    wire        w_mem_we    = (w_eng_we && !w_full) || w_push_ok;
    // a half last word: its 4 bytes in [31:0], [63:32] = 0
    wire [63:0] w_rx_word   = w_rx_half ? {32'h0, w_rx_next[7:0], w_rx_next[15:8], w_rx_next[23:16], w_rx_next[31:24]}
                                        : f_bswap(w_rx_next);
    wire [63:0] w_mem_wdata = w_eng_we ? w_rx_word : r_in_wdata;
    always_ff @(posedge i_Clk) begin
        if (w_mem_we) r_mem[r_wp[8:0]] <= w_mem_wdata;
        if (w_fetch)  r_mem_q <= r_mem[r_mrp[8:0]];
    end

    // -----------------------------------------------------------------------
    // Clock generator
    // -----------------------------------------------------------------------
    always_ff @(posedge i_Clk) begin
        if (i_rst || !i_sd_mode) begin
            r_clk     <= 1'b0;
            r_div_cnt <= 16'd0;
            r_samp    <= 1'b0;
        end else begin
            r_samp <= ev_fall;
            if (!r_clk && !w_run) begin
                r_div_cnt <= 16'd0;                 // parked low
            end else if (w_tick) begin
                r_div_cnt <= 16'd0;
                r_clk     <= ~r_clk;
            end else begin
                r_div_cnt <= r_div_cnt + 16'd1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Registers, status, FIFO pointers and both engines
    // -----------------------------------------------------------------------
    always_ff @(posedge i_Clk) begin
        if (i_rst) begin
            r_wr_q      <= 1'b0;  r_rd_q <= 1'b0;
            r_waddr_q   <= 16'h0; r_raddr_q <= 16'h0;
            r_arg       <= 32'h0;
            r_resp0     <= 64'h0; r_resp1 <= 64'h0;
            r_blksz     <= 10'd128; r_blkcnt <= 16'd1;     // 0x0001_0200
            r_dir       <= DIR_NONE;
            r_cmd_done  <= 1'b0;  r_data_done <= 1'b0;
            r_err       <= 11'h0;
            r_irq_en    <= 27'h0;
            r_tmo_data  <= 32'd2_500_000;
            r_tmo_busy  <= 32'd12_500_000;
            r_cfg_w4    <= 1'b0;  r_cfg_clkon <= 1'b0;
            r_blkdone   <= 16'h0;
            r_trail     <= 4'd0;
            r_c_state   <= C_IDLE;
            r_c_ok_p    <= 1'b0;  r_c_err_p <= 1'b0;
            r_c_tx      <= 40'h0; r_c_crc <= 7'h0; r_c_cnt <= 8'h0;
            r_c_idx     <= 6'h0;  r_c_rtype <= 3'd0;
            r_c_sh      <= 128'h0;
            r_c_busy    <= 32'h0;
            r_d_state   <= D_IDLE;
            r_d_w4      <= 1'b0;  r_d_blkw <= 10'd64; r_d_sz4 <= 10'd128; r_d_cnt <= 16'd1;
            r_d_wpos    <= 6'd0;
            r_d_target  <= 26'd64;
            r_d_tgt_stale <= 1'b0;
            r_d_units   <= 15'd0; r_d_aux <= 5'd0;  r_d_tok <= 4'd0;
            for (int i = 0; i < 4; i++) r_d_crc[i] <= 16'h0;
            r_d_sh      <= 64'h0;
            r_d_tmo     <= 32'h0;
            o_cmd       <= 1'b1;  o_cmd_oe <= 1'b0;
            o_dat       <= 4'hF;  o_dat_oe <= 4'h0;
            r_wp        <= 10'd0; r_wc <= 10'd0; r_rp <= 10'd0; r_mrp <= 10'd0;
            r_head      <= 64'h0; r_head_v <= 1'b0; r_fetch <= 1'b0;
            r_push_words <= 26'd0;
            r_rd_room   <= 1'b0;
            o_irq       <= 1'b0;
        end else begin
            r_wr_q    <= r_in_wr;  r_waddr_q <= r_in_addr;
            r_rd_q    <= r_in_rd;  r_raddr_q <= r_in_addr;
            r_c_ok_p  <= 1'b0;
            r_c_err_p <= 1'b0;

            // ---------------- plain register writes ----------------
            if (w_wr && w_off == O_ARG)   r_arg <= r_in_wdata[31:0];
            if (w_wr && w_off == O_BLK)   begin r_blksz <= r_in_wdata[11:2]; r_blkcnt <= r_in_wdata[31:16]; end
            if (w_wr && w_off == O_IRQEN) r_irq_en <= r_in_wdata[26:0] & IRQ_MASK[26:0];
            if (w_wr && w_off == O_TMO)   begin r_tmo_data <= r_in_wdata[31:0]; r_tmo_busy <= r_in_wdata[63:32]; end
            if (w_wr && w_off == O_CFG)   begin r_cfg_w4 <= r_in_wdata[0]; r_cfg_clkon <= r_in_wdata[2]; end

            // ---------------- sticky status: W1C first, engine sets below win ----------------
            if (w_w1c) begin
                if (r_in_wdata[8]) r_cmd_done  <= 1'b0;
                if (r_in_wdata[9]) r_data_done <= 1'b0;
                r_err <= r_err & ~r_in_wdata[26:16];
            end

            // ---------------- trailing clocks ----------------
            if (ev_fall && r_trail != 4'd0) r_trail <= r_trail - 4'd1;

            // FIFO: CPU pop (pop-on-read): w_cpu_pop commits at the end of
            // the access's second cycle, the cycle the SoC samples the head
            // word, and only if that word was valid.

            // ---------------- FIFO: head / fetch ----------------
            r_fetch <= w_fetch;
            if (w_fetch) r_mrp <= r_mrp + 10'd1;
            if (r_fetch) begin
                r_head   <= r_mem_q;
                r_head_v <= 1'b1;
            end else if (w_pop) begin
                r_head_v <= 1'b0;
            end
            if (w_pop) r_rp <= r_rp + 10'd1;

            // ---------------- FIFO: CPU push ----------------
            if (w_push) begin
                if (w_push_ok) begin
                    r_wp <= r_wp + 10'd1;
                    r_wc <= r_wc + 10'd1;
                    r_push_words <= r_push_words + 26'd1;
                end else begin
                    r_err[E_FIFO] <= 1'b1;
                end
            end
            r_rd_room <= ({1'b0, w_used} + {1'b0, r_d_blkw}) <= 11'd512;
            r_d_target    <= r_d_cnt * r_d_blkw;      // from the latched values (DSP), a cycle later
            r_d_tgt_stale <= w_acc_data;

            // =================================================================
            // Engines (only in SD mode; otherwise idle and off the pins)
            // =================================================================
            if (!i_sd_mode) begin
                r_c_state <= C_IDLE;
                if (w_d_read) r_wp <= r_wc;
                r_d_state <= D_IDLE;
                r_trail   <= 4'd0;
                o_cmd <= 1'b1; o_cmd_oe <= 1'b0;
                o_dat <= 4'hF; o_dat_oe <= 4'h0;
            end else begin
                // ---------------- command engine ----------------
                case (r_c_state)
                    C_IDLE: ;
                    C_WAIT: begin       // N_CC / N_RC: wait out the trailing clocks
                        if (ev_fall && r_trail == 4'd0) begin
                            o_cmd    <= r_c_tx[39];      // start bit
                            o_cmd_oe <= 1'b1;
                            r_c_crc  <= f_crc7(7'h0, r_c_tx[39]);
                            r_c_tx   <= {r_c_tx[38:0], 1'b0};
                            r_c_cnt  <= 8'd1;
                            r_c_state <= C_SEND;
                        end
                    end
                    C_SEND: begin
                        if (ev_fall) begin
                            r_c_cnt <= r_c_cnt + 8'd1;
                            if (r_c_cnt < 8'd40) begin
                                o_cmd   <= r_c_tx[39];
                                r_c_crc <= f_crc7(r_c_crc, r_c_tx[39]);
                                r_c_tx  <= {r_c_tx[38:0], 1'b0};
                            end else if (r_c_cnt < 8'd47) begin
                                o_cmd   <= r_c_crc[6];
                                r_c_crc <= {r_c_crc[5:0], 1'b0};
                            end else if (r_c_cnt == 8'd47) begin
                                o_cmd   <= 1'b1;            // end bit
                            end else begin                  // 48: release CMD
                                o_cmd    <= 1'b1;
                                o_cmd_oe <= 1'b0;
                                r_c_cnt  <= 8'd0;
                                if (r_c_rtype == 3'd0) begin
                                    r_cmd_done <= 1'b1;
                                    r_c_ok_p   <= 1'b1;
                                    r_trail    <= 4'd8;
                                    r_c_state  <= C_IDLE;
                                end else begin
                                    r_c_state  <= C_RWAIT;
                                end
                            end
                        end
                    end
                    C_RWAIT: begin
                        if (r_samp) begin
                            if (!r_cmd_in) begin
                                r_c_cnt   <= 8'd0;
                                r_c_crc   <= 7'h0;
                                r_c_state <= C_RECV;
                            end else if (r_c_cnt == 8'(NCR_MAX)) begin
                                r_err[E_CMD_TMO] <= 1'b1;
                                r_cmd_done <= 1'b1;
                                r_c_err_p  <= 1'b1;
                                r_trail    <= 4'd8;
                                r_c_state  <= C_IDLE;
                            end else begin
                                r_c_cnt <= r_c_cnt + 8'd1;
                            end
                        end
                    end
                    C_RECV: begin
                        if (r_samp) begin
                            r_c_cnt <= r_c_cnt + 8'd1;
                            r_c_sh  <= {r_c_sh[126:0], r_cmd_in};
                            // CRC7 over everything it covers, then the 7 CRC
                            // bits themselves: a good response leaves 0.
                            if (r_c_rtype == 3'd3) begin
                                if (r_c_cnt >= 8'd7 && r_c_cnt <= 8'd133) r_c_crc <= f_crc7(r_c_crc, r_cmd_in);
                            end else begin
                                if (r_c_cnt <= 8'd45) r_c_crc <= f_crc7(r_c_crc, r_cmd_in);
                            end
                            if ((r_c_rtype == 3'd3) ? (r_c_cnt == 8'd134) : (r_c_cnt == 8'd46)) begin
                                // r_cmd_in is the end bit
                                logic e_end, e_crc, e_idx;
                                e_end = !r_cmd_in;
                                e_crc = (r_c_rtype != 3'd4) && (r_c_crc != 7'h0);
                                e_idx = ((r_c_rtype == 3'd1) || (r_c_rtype == 3'd2)) && (r_c_sh[44:39] != r_c_idx);
                                if (r_c_rtype == 3'd3) begin
                                    r_resp1 <= r_c_sh[126:63];
                                    r_resp0 <= {r_c_sh[62:0], r_cmd_in};
                                end else begin
                                    r_resp1 <= 64'h0;
                                    r_resp0 <= {26'h0, r_c_sh[44:7]};
                                end
                                r_c_cnt <= 8'd0;
                                if (e_end) r_err[E_CMD_END] <= 1'b1;
                                if (e_crc) r_err[E_CMD_CRC] <= 1'b1;
                                if (e_idx) r_err[E_CMD_IDX] <= 1'b1;
                                if (e_end || e_crc || e_idx) begin
                                    r_cmd_done <= 1'b1;
                                    r_c_err_p  <= 1'b1;
                                    r_trail    <= 4'd8;
                                    r_c_state  <= C_IDLE;
                                end else if (r_c_rtype == 3'd2) begin
                                    r_c_busy  <= 32'h0;
                                    r_c_state <= C_BUSY;
                                end else begin
                                    r_cmd_done <= 1'b1;
                                    r_c_ok_p   <= 1'b1;
                                    r_trail    <= 4'd8;
                                    r_c_state  <= C_IDLE;
                                end
                            end
                        end
                    end
                    C_BUSY: begin       // R1b: DAT0 low while the card is busy
                        if (r_samp) begin
                            if (r_c_cnt < 8'(BUSY_SKIP)) r_c_cnt <= r_c_cnt + 8'd1;
                            if (r_c_cnt >= 8'(BUSY_SKIP) && r_dat_in[0]) begin
                                r_cmd_done <= 1'b1;
                                r_c_ok_p   <= 1'b1;
                                r_trail    <= 4'd8;
                                r_c_state  <= C_IDLE;
                            end else if (r_c_busy >= r_tmo_busy) begin
                                r_err[E_BUSY_TMO] <= 1'b1;
                                r_cmd_done <= 1'b1;
                                r_c_err_p  <= 1'b1;
                                r_trail    <= 4'd8;
                                r_c_state  <= C_IDLE;
                            end else begin
                                r_c_busy <= r_c_busy + 32'd1;
                            end
                        end
                    end
                    default: r_c_state <= C_IDLE;
                endcase

                // a new command (only from C_IDLE)
                if (w_cmd_acc) begin
                    r_c_tx    <= {1'b0, 1'b1, r_in_wdata[5:0], r_arg};
                    r_c_idx   <= r_in_wdata[5:0];
                    r_c_rtype <= w_rtype_in;
                    r_c_cnt   <= 8'd0;
                    r_c_state <= C_WAIT;
                end

                // ---------------- data engine ----------------
                case (r_d_state)
                    D_IDLE: ;

                    // ---- read ----
                    D_RD_WCMD: begin    // look for data once the command's end bit is out
                        if (ev_fall && r_c_state == C_SEND && r_c_cnt == 8'd48) begin
                            r_d_tmo   <= 32'h0;
                            r_d_state <= D_RD_START;
                        end
                    end
                    D_RD_START: begin
                        if (r_c_err_p) begin
                            r_d_state <= D_IDLE;            // response error: abort, no DATA_DONE
                        end else if (r_samp) begin
                            if (!r_dat_in[0]) begin
                                r_d_units <= r_d_w4 ? {2'b0, r_d_sz4, 3'b0} : {r_d_sz4, 5'b0};
                                r_d_wpos  <= 6'd0;
                                for (int i = 0; i < 4; i++) r_d_crc[i] <= 16'h0;
                                r_d_state <= D_RD_DATA;
                            end else if (r_d_tmo >= r_tmo_data) begin
                                r_err[E_DAT_TMO] <= 1'b1;
                                r_trail   <= 4'd8;
                                r_d_state <= D_IDLE;
                            end else begin
                                r_d_tmo <= r_d_tmo + 32'd1;
                            end
                        end
                    end
                    D_RD_DATA: begin
                        if (r_c_err_p) begin
                            r_wp      <= r_wc;
                            r_d_state <= D_IDLE;
                        end else if (r_samp) begin
                            r_d_sh <= w_rx_next;
                            for (int i = 0; i < 4; i++)
                                if (r_d_w4 || i == 0) r_d_crc[i] <= f_crc16(r_d_crc[i], r_dat_in[i]);
                            if (w_eng_we) begin
                                if (w_full) r_err[E_FIFO] <= 1'b1;   // only if the FIFO was not reset first
                                else        r_wp <= r_wp + 10'd1;
                            end
                            r_d_units <= r_d_units - 15'd1;
                            r_d_wpos  <= w_rx_wordb ? 6'd0 : r_d_wpos + 6'd1;
                            if (r_d_units == 15'd1) begin
                                r_d_aux   <= 5'd0;
                                r_d_state <= D_RD_CRC;
                            end
                        end
                    end
                    D_RD_CRC: begin     // 16 CRC bits per line, then the end bit
                        if (r_c_err_p) begin
                            r_wp      <= r_wc;
                            r_d_state <= D_IDLE;
                        end else if (r_samp) begin
                            r_d_aux <= r_d_aux + 5'd1;
                            if (r_d_aux != 5'd16) begin
                                for (int i = 0; i < 4; i++)
                                    if (r_d_w4 || i == 0) r_d_crc[i] <= f_crc16(r_d_crc[i], r_dat_in[i]);
                            end else begin
                                if (w_blk_end_ok && w_blk_crc_ok) begin
                                    r_wc      <= r_wp;              // the block becomes visible
                                    r_blkdone <= r_blkdone + 16'd1;
                                    if (w_rd_last_blk) begin
                                        r_data_done <= 1'b1;
                                        r_trail     <= 4'd8;
                                        r_d_state   <= D_IDLE;
                                    end else begin
                                        r_d_tmo   <= 32'h0;
                                        r_d_state <= r_rd_room ? D_RD_START : D_RD_HOLD;
                                    end
                                end else begin
                                    r_wp <= r_wc;                   // roll the bad block back
                                    if (!w_blk_end_ok) r_err[E_DAT_END] <= 1'b1;
                                    if (!w_blk_crc_ok) r_err[E_DAT_CRC] <= 1'b1;
                                    r_trail   <= 4'd8;
                                    r_d_state <= D_IDLE;
                                end
                            end
                        end
                    end
                    D_RD_HOLD: begin    // CLK parked until the FIFO can take a whole block
                        if (r_c_err_p) r_d_state <= D_IDLE;
                        else if (r_rd_room) begin
                            r_d_tmo   <= 32'h0;
                            r_d_state <= D_RD_START;
                        end
                    end

                    // ---- write ----
                    D_WR_WRSP: begin    // the data phase follows a good response
                        if (r_c_err_p) r_d_state <= D_IDLE;
                        else if (r_c_ok_p) begin
                            r_d_aux   <= 5'd2;
                            r_d_state <= D_WR_GAP;
                        end
                    end
                    D_WR_GAP: begin     // N_WR: >= 2 clocks before the start bit
                        if (ev_fall) begin
                            if (r_d_aux == 5'd0) r_d_state <= D_WR_WAIT;
                            else                 r_d_aux   <= r_d_aux - 5'd1;
                        end
                    end
                    D_WR_WAIT: begin    // CLK parked until a whole block is in the FIFO
                        if (ev_fall && w_blk_ready) begin
                            o_dat     <= r_d_w4 ? 4'h0 : 4'hE;   // start bit
                            o_dat_oe  <= r_d_w4 ? 4'hF : 4'h1;
                            r_d_sh    <= f_bswap(r_head);
                            r_d_units <= r_d_w4 ? {2'b0, r_d_sz4, 3'b0} : {r_d_sz4, 5'b0};
                            r_d_wpos  <= 6'd0;
                            for (int i = 0; i < 4; i++) r_d_crc[i] <= 16'h0;
                            r_d_state <= D_WR_DATA;
                        end
                    end
                    D_WR_DATA: begin
                        if (ev_fall) begin
                            logic [3:0] u;
                            u = r_d_w4 ? r_d_sh[63:60] : {3'b111, r_d_sh[63]};
                            o_dat <= u;
                            for (int i = 0; i < 4; i++)
                                if (r_d_w4 || i == 0) r_d_crc[i] <= f_crc16(r_d_crc[i], u[i]);
                            r_d_units <= r_d_units - 15'd1;
                            r_d_wpos  <= w_tx_wordb ? 6'd0 : r_d_wpos + 6'd1;
                            if (r_d_units == 15'd1) begin
                                r_d_aux   <= 5'd0;
                                r_d_state <= D_WR_CRC;
                            end else if (w_tx_wordb) begin
                                r_d_sh <= f_bswap(r_head);          // next word (popped)
                            end else begin
                                r_d_sh <= r_d_w4 ? {r_d_sh[59:0], 4'h0} : {r_d_sh[62:0], 1'b0};
                            end
                        end
                    end
                    D_WR_CRC: begin
                        if (ev_fall) begin
                            for (int i = 0; i < 4; i++) begin
                                o_dat[i]   <= (r_d_w4 || i == 0) ? r_d_crc[i][15] : 1'b1;
                                r_d_crc[i] <= {r_d_crc[i][14:0], 1'b0};
                            end
                            r_d_aux <= r_d_aux + 5'd1;
                            if (r_d_aux == 5'd15) r_d_state <= D_WR_END;
                        end
                    end
                    D_WR_END: begin
                        if (ev_fall) begin
                            o_dat     <= 4'hF;                       // end bit
                            r_d_state <= D_WR_REL;
                        end
                    end
                    D_WR_REL: begin
                        if (ev_fall) begin
                            o_dat_oe  <= 4'h0;
                            r_d_aux   <= 5'd0;
                            r_d_state <= D_WR_TOK;
                        end
                    end
                    D_WR_TOK: begin     // CRC status token: start bit within 8 clocks
                        if (r_samp) begin
                            if (!r_dat_in[0]) begin
                                r_d_aux   <= 5'd0;
                                r_d_state <= D_WR_TST;
                            end else if (r_d_aux == 5'(TOK_WIN)) begin
                                r_err[E_WR_NOTOK] <= 1'b1;
                                r_trail   <= 4'd8;
                                r_d_state <= D_IDLE;
                            end else begin
                                r_d_aux <= r_d_aux + 5'd1;
                            end
                        end
                    end
                    D_WR_TST: begin     // 3 status bits, then the token's end bit
                        if (r_samp) begin
                            r_d_tok <= {r_d_tok[2:0], r_dat_in[0]};
                            r_d_aux <= r_d_aux + 5'd1;
                            if (r_d_aux == 5'd3) begin
                                if (r_d_tok[2:0] == 3'b010 && r_dat_in[0]) begin
                                    r_d_aux   <= 5'd0;
                                    r_d_tmo   <= 32'h0;
                                    r_d_state <= D_WR_BUSY;
                                end else begin
                                    r_err[E_WR_CRC] <= 1'b1;
                                    r_trail   <= 4'd8;
                                    r_d_state <= D_IDLE;
                                end
                            end
                        end
                    end
                    D_WR_BUSY: begin
                        if (r_samp) begin
                            if (r_d_aux < 5'(BUSY_SKIP)) r_d_aux <= r_d_aux + 5'd1;
                            if (r_d_aux >= 5'(BUSY_SKIP) && r_dat_in[0]) begin
                                r_blkdone <= r_blkdone + 16'd1;
                                if (w_rd_last_blk) begin
                                    r_data_done <= 1'b1;
                                    r_trail     <= 4'd8;
                                    r_d_state   <= D_IDLE;
                                end else begin
                                    r_d_aux   <= 5'd2;
                                    r_d_state <= D_WR_GAP;
                                end
                            end else if (r_d_tmo >= r_tmo_busy) begin
                                r_err[E_BUSY_TMO] <= 1'b1;
                                r_trail   <= 4'd8;
                                r_d_state <= D_IDLE;
                            end else begin
                                r_d_tmo <= r_d_tmo + 32'd1;
                            end
                        end
                    end
                    default: r_d_state <= D_IDLE;
                endcase

                // a stop (CMD12, SDM_CMD[13]) ends a running data phase now;
                // a partly received read block is rolled back.  An
                // open-ended phase stopped this way ended cleanly: DATA_DONE;
                // a counted one stopped early does not set it (§4.8).  (A
                // data command is only accepted with the engine idle.)
                if (w_acc_stop || w_acc_data) begin
                    if (w_d_read) r_wp <= r_wc;
                    o_dat    <= 4'hF;
                    o_dat_oe <= 4'h0;
                    r_d_state <= D_IDLE;
                    if (w_acc_stop && w_d_active && r_d_cnt == 16'd0) r_data_done <= 1'b1;
                end
                if (w_acc_data) begin
                    r_dir     <= r_in_wdata[12] ? DIR_WRITE : DIR_READ;
                    r_blkdone <= 16'h0;
                    r_d_w4    <= r_cfg_w4;
                    r_d_blkw  <= w_blkw;
                    r_d_sz4   <= w_sz4;
                    r_d_cnt   <= r_blkcnt;
                    r_d_state <= r_in_wdata[12] ? D_WR_WRSP : D_RD_WCMD;
                end
            end

            // ---------------- FIFO reset (highest priority on the pointers) ----------------
            if (w_fifo_rs) begin
                r_wp <= 10'd0; r_wc <= 10'd0; r_rp <= 10'd0; r_mrp <= 10'd0;
                r_head_v <= 1'b0; r_fetch <= 1'b0;
                r_push_words <= 26'd0;
                r_dir    <= DIR_NONE;
            end

            o_irq <= |(w_status & {5'h0, r_irq_en} & IRQ_MASK);
        end
    end

    // -----------------------------------------------------------------------
    // Read mux: combinational from the registered address (valid in the
    // access's second cycle); 0 outside the window, so the parent ORs it in.
    // -----------------------------------------------------------------------
    always_comb begin
        o_rdata = 64'h0;
        if (w_win) case (w_off)
            O_ARG:     o_rdata = {32'h0, r_arg};
            O_RESP0:   o_rdata = r_resp0;
            O_RESP1:   o_rdata = r_resp1;
            O_BLK:     o_rdata = {32'h0, r_blkcnt, 4'h0, r_blksz, 2'b00};
            O_STATUS:  o_rdata = {32'h0, w_status};
            O_IRQEN:   o_rdata = {37'h0, r_irq_en};
            O_TMO:     o_rdata = {r_tmo_busy, r_tmo_data};
            O_CFG:     o_rdata = {61'h0, r_cfg_clkon, 1'b0, r_cfg_w4};
            O_FIFO:    o_rdata = {54'h0, w_level};
            O_BLKDONE: o_rdata = {48'h0, r_blkdone};
            O_DATA:    o_rdata = r_head_v ? r_head : 64'h0;
            O_CAPS:    o_rdata = CAPS;
            default:   o_rdata = 64'h0;           // SDM_CMD, SDM_DMA_ADDR, reserved
        endcase
    end

endmodule

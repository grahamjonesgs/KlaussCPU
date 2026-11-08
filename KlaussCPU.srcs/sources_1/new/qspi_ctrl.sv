`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// qspi_ctrl — CPU access to the Nexys A7 configuration flash (Spansion
// S25FL128S, 16 MiB), exposed as MMIO device 0x012 (base 0xF012_0000, see
// MMIO_MAP.md "QSPI flash").
//
// The FPGA boots from this flash: the bitstream lives at offset 0 (~3.1 MB
// compressed).  After configuration the flash pins are ordinary user I/O,
// except CCLK, which the fabric can only drive through STARTUPE2.USRCCLKO
// (instantiated in KlaussCPU.sv; this module just produces o_sck).
//
// A command engine, not a byte-banging port: software loads an opcode, a
// 24-bit address, a length and (for page program) up to 256 bytes into the
// buffer, then writes QSPI_CMD; the hardware runs the whole CS-low
// transaction.  Because the hardware sees the opcode and the address it can
// guard the bitstream:
//
//   * Only whitelisted opcodes run (table below).  Anything else — status /
//     configuration register writes (WRR 0x01), bulk erase (0x60/0xC7), bank
//     register writes, OTP program, … — is rejected without touching the
//     flash and sets STATUS.rejected.
//   * Addressed commands go on the wire as their 4-byte-address forms (4READ
//     0x13, 4PP 0x12, 4SE 0xDC, 4P4E 0x21) with a 0x00 top byte.  Those ignore
//     the flash's volatile BAR.EXTADD bit, so a flash left in 4-byte mode by
//     some other design cannot turn {ADDR, data byte} into a low address.
//   * Page program and sector erase are rejected unless ADDR >= PROT_FLOOR
//     (0xF0_0000 by default).  The floor is aligned to 256 KiB, the largest
//     S25FL128S erase sector, so no erase or program that passes the check can
//     reach below it.  The floor is a synthesis parameter — software cannot
//     move it.
//
// Whitelist (software opcode; kind: R = read len bytes into the buffer,
// W = send len bytes from the buffer, N = no data phase; A = addressed, sent
// as the 4-byte form shown in brackets):
//   0x9F RDID   R      0x05 RDSR1  R      0x07 RDSR2  R      0x35 RDCR   R
//   0x16 BRRD   R      0x03 READ   R A [13]               0x06 WREN   N
//   0x04 WRDI   N      0x30 CLSR   N      0x02 PP     W A g [12]
//   0xD8 SE     N A g [DC]                0x20 P4E    N A g [21]
//   (g = guarded by PROT_FLOOR; len is forced to 0 for N commands)
//
// Register layout (offsets within the device window, all 64-bit aligned):
//   0x000  QSPI_CMD    W   [7:0] opcode, [31:16] len (0..256 bytes; more is
//                          refused).  Starts the transaction.  Refused (and
//                          STATUS.rejected set) if not ready, not whitelisted,
//                          guarded and below the floor, len > 256, or if any
//                          write was dropped since the previous CMD (below).
//                      R   last command word written
//   0x008  QSPI_ADDR   RW  [23:0] flash byte address
//   0x010  QSPI_STATUS R   [0] busy, [1] rejected (last CMD refused; sticky
//                          until the next CMD write), [2] ready (start-up
//                          dummy clocks done; CMD writes before this are
//                          refused), [3] dropped: a CMD/ADDR/CLKDIV/buffer
//                          write arrived while busy and was discarded.  The
//                          next CMD write is then refused and clears it, so a
//                          driver that forgets to poll busy fails loudly
//                          instead of programming the wrong address or data.
//   0x018  QSPI_CLKDIV RW  [7:0] SCK half-period in i_Clk cycles minus 1;
//                          reset 1 → 25 MHz at i_Clk = 100 MHz.  Writing 0
//                          stores 1: 50 MHz leaves no MISO margin through
//                          STARTUPE2 (CCLK delay + flash tV ≈ 20 ns).
//   0x020  QSPI_FLOOR  R   [23:0] PROT_FLOOR
//   0x028  QSPI_ID     R   0x5153_5049_0000_0001 ("QSPI", version 1).  Older
//                          bitstreams read 0 here — drivers probe this.
//   0x100..0x1F8       RW  256-byte data buffer (32 × 64-bit dwords, little-
//                          endian: buffer byte i is dword i/8, bits
//                          (i%8)*8+:8).  READ: buffer byte i = flash
//                          ADDR+i.  PP: buffer byte i goes to ADDR+i within
//                          the 256-byte page (the flash wraps at the page
//                          end).  Any R command (e.g. an RDSR poll) overwrites
//                          bytes 0..len-1, so fill the buffer after polling.
//
// SPI mode 0, MSB first, single-bit (DQ0 = MOSI, DQ1 = MISO; WP#/DQ2 and
// HOLD#/DQ3 are held high).  MISO is sampled at the end of each SCK-high half,
// one full SCK period after the flash launched it on the previous falling edge.
// CS# is held high for >= 8 i_Clk cycles (80 ns) between transactions
// (tCSH >= 50 ns after program/erase commands).
//
// UG470: the first three USRCCLKO cycles after end-of-startup only switch the
// CCLK source and never reach the pin, so after reset (and once i_eos is high)
// the engine clocks 8 dummy cycles with CS# high before setting ready.
//////////////////////////////////////////////////////////////////////////////////

module qspi_ctrl #(
    parameter logic [23:0] PROT_FLOOR = 24'hF0_0000
) (
    input              i_Clk,
    input              i_Rst_L,
    input              i_eos,          // STARTUPE2.EOS (1 after configuration)

    mmio_if.slave      mmio,

    output logic       o_sck,          // to STARTUPE2.USRCCLKO
    output logic       o_cs_n,
    output logic       o_mosi,         // DQ0
    input              i_miso,         // DQ1
    output             o_wp_n,         // DQ2
    output             o_hold_n        // DQ3
);

    localparam logic [15:0] OFF_CMD    = 16'h0000;
    localparam logic [15:0] OFF_ADDR   = 16'h0008;
    localparam logic [15:0] OFF_STATUS = 16'h0010;
    localparam logic [15:0] OFF_CLKDIV = 16'h0018;
    localparam logic [15:0] OFF_FLOOR  = 16'h0020;
    localparam logic [15:0] OFF_ID     = 16'h0028;
    localparam logic [63:0] ID_VALUE   = 64'h5153_5049_0000_0001;

    assign o_wp_n   = 1'b1;
    assign o_hold_n = 1'b1;
    assign mmio.ready = 1'b1;

    wire       is_buf  = (mmio.addr[15:8] == 8'h01);
    wire [4:0] buf_idx = mmio.addr[7:3];

    // -----------------------------------------------------------------------
    // Opcode whitelist decode
    // -----------------------------------------------------------------------
    localparam logic [1:0] K_NONE = 2'd0, K_READ = 2'd1, K_WRITE = 2'd2;

    function automatic logic [3:0] f_decode(input logic [7:0] op);
        // returns {allowed, has_addr, kind[1:0]}
        case (op)
            8'h9F, 8'h05, 8'h07, 8'h35, 8'h16: f_decode = {1'b1, 1'b0, K_READ};
            8'h03:                             f_decode = {1'b1, 1'b1, K_READ};  // 4READ
            8'h06, 8'h04, 8'h30:               f_decode = {1'b1, 1'b0, K_NONE};
            8'h02:                             f_decode = {1'b1, 1'b1, K_WRITE};
            8'hD8, 8'h20:                      f_decode = {1'b1, 1'b1, K_NONE};
            default:                           f_decode = 4'b0;
        endcase
    endfunction

    // Opcode on the wire: addressed commands use the 4-byte-address forms.
    function automatic logic [7:0] f_wire_op(input logic [7:0] op);
        case (op)
            8'h03:   f_wire_op = 8'h13;   // 4READ
            8'h02:   f_wire_op = 8'h12;   // 4PP
            8'hD8:   f_wire_op = 8'hDC;   // 4SE
            8'h20:   f_wire_op = 8'h21;   // 4P4E
            default: f_wire_op = op;
        endcase
    endfunction

    function automatic logic f_guarded(input logic [7:0] op);
        f_guarded = (op == 8'h02) || (op == 8'hD8) || (op == 8'h20);
    endfunction

    // -----------------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------------
    logic [23:0] r_addr;
    logic [ 7:0] r_clk_div;
    logic [31:0] r_cmd_word;
    logic        r_rejected;
    logic        r_ready;
    logic        r_busy;
    logic        r_dropped;

    wire        w_cmd_wr  = mmio.write_DV && (mmio.addr[15:0] == OFF_CMD);
    wire [7:0]  w_op      = mmio.write_data[7:0];
    wire [15:0] w_len_in  = mmio.write_data[31:16];
    wire [3:0]  w_dec     = f_decode(w_op);
    wire [7:0]  w_wire_op = f_wire_op(w_op);
    wire        w_allowed = w_dec[3];
    wire        w_guard_ok = !f_guarded(w_op) || (r_addr >= PROT_FLOOR);
    wire        w_len_ok  = (w_len_in <= 16'd256);
    wire        w_accept  = w_cmd_wr && !r_busy && r_ready && !r_dropped
                            && w_allowed && w_guard_ok && w_len_ok;
    // Any write into the device window while busy is discarded and flagged.
    wire        w_dev_wr  = mmio.write_DV
                            && ((mmio.addr[15:0] == OFF_CMD) || (mmio.addr[15:0] == OFF_ADDR)
                                || (mmio.addr[15:0] == OFF_CLKDIV) || is_buf);
    wire        w_reject  = w_cmd_wr && !r_busy && !w_accept;

    always_ff @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_addr     <= 24'h0;
            r_clk_div  <= 8'd1;
            r_cmd_word <= 32'h0;
            r_rejected <= 1'b0;
            r_dropped  <= 1'b0;
        end else begin
            if (mmio.write_DV && (mmio.addr[15:0] == OFF_ADDR) && !r_busy)
                r_addr <= mmio.write_data[23:0];
            if (mmio.write_DV && (mmio.addr[15:0] == OFF_CLKDIV) && !r_busy)
                r_clk_div <= (mmio.write_data[7:0] == 8'd0) ? 8'd1 : mmio.write_data[7:0];
            if (w_cmd_wr && !r_busy) begin
                r_cmd_word <= mmio.write_data[31:0];
                r_rejected <= w_reject;
                r_dropped  <= 1'b0;
            end else if (w_dev_wr && r_busy) begin
                r_dropped  <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Data buffer — 32 × 64-bit LUTRAM, one write port (MMIO when idle, the
    // engine when busy), two combinational read ports (MMIO dword, engine byte).
    // -----------------------------------------------------------------------
    logic [63:0] r_buf [0:31];

    logic        w_eng_buf_we;
    logic [ 7:0] w_eng_buf_idx;     // byte index 0..255
    logic [ 7:0] w_eng_buf_wdata;

    integer bi;
    always_ff @(posedge i_Clk) begin
        if (r_busy) begin
            if (w_eng_buf_we)
                r_buf[w_eng_buf_idx[7:3]][w_eng_buf_idx[2:0]*8 +: 8] <= w_eng_buf_wdata;
        end else if (mmio.write_DV && is_buf) begin
            for (bi = 0; bi < 8; bi = bi + 1)
                if (mmio.byte_en[bi])
                    r_buf[buf_idx][bi*8 +: 8] <= mmio.write_data[bi*8 +: 8];
        end
    end

    // -----------------------------------------------------------------------
    // Transaction engine
    // -----------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_WAIT_EOS, S_DUMMY, S_IDLE, S_CS_SETUP, S_XFER, S_CS_HOLD
    } state_t;
    state_t      r_state;

    logic [ 7:0] r_div_cnt;
    logic        r_sck_phase;       // 0 = SCK low half, 1 = SCK high half
    logic [ 2:0] r_bit;             // bit index within the byte, 7..0
    logic [ 9:0] r_byte;            // byte index in the transaction
    logic [ 9:0] r_nbytes;          // total bytes: 1 + (4) + len
    logic [ 2:0] r_hdr;             // header bytes: 1, or 5 (opcode + 4 address)
    logic [ 1:0] r_kind;
    logic [ 7:0] r_opcode;
    logic [23:0] r_xaddr;
    logic [ 7:0] r_tx;
    logic [ 7:0] r_rx;
    logic [ 3:0] r_dummy;

    wire w_tick = (r_div_cnt >= r_clk_div);

    // Byte to transmit at transaction byte index i.
    wire [9:0] w_next_byte = r_byte + 10'd1;
    wire [9:0] w_data_idx_next = w_next_byte - {7'b0, r_hdr};
    function automatic logic [7:0] f_tx_byte(input logic [9:0] i,
                                             input logic [2:0] hdr,
                                             input logic [1:0] kind,
                                             input logic [7:0] op,
                                             input logic [23:0] a,
                                             input logic [7:0] bufb);
        if (i == 10'd0)                    f_tx_byte = op;
        else if (hdr == 3'd5 && i == 10'd1) f_tx_byte = 8'h00;     // A[31:24]
        else if (hdr == 3'd5 && i == 10'd2) f_tx_byte = a[23:16];
        else if (hdr == 3'd5 && i == 10'd3) f_tx_byte = a[15:8];
        else if (hdr == 3'd5 && i == 10'd4) f_tx_byte = a[7:0];
        else if (kind == K_WRITE)          f_tx_byte = bufb;
        else                               f_tx_byte = 8'hFF;
    endfunction

    // Engine buffer port: the byte for the *next* tx byte (write commands) or
    // the byte just received (read commands).
    wire [9:0] w_cur_data_idx = r_byte - {7'b0, r_hdr};
    wire       w_byte_done    = (r_state == S_XFER) && w_tick && r_sck_phase
                                && (r_bit == 3'd0);
    wire [7:0] w_rx_full      = {r_rx[6:0], i_miso};

    always_comb begin
        w_eng_buf_we    = w_byte_done && (r_kind == K_READ)
                          && (r_byte >= {7'b0, r_hdr});
        w_eng_buf_idx   = w_eng_buf_we ? w_cur_data_idx[7:0] : w_data_idx_next[7:0];
        w_eng_buf_wdata = w_rx_full;
    end
    wire [7:0] w_eng_buf_rbyte = r_buf[w_eng_buf_idx[7:3]][w_eng_buf_idx[2:0]*8 +: 8];
    wire [7:0] w_nb = f_tx_byte(w_next_byte, r_hdr, r_kind, r_opcode,
                                r_xaddr, w_eng_buf_rbyte);

    always_ff @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_state     <= S_WAIT_EOS;
            r_ready     <= 1'b0;
            r_busy      <= 1'b0;
            r_div_cnt   <= 8'd0;
            r_sck_phase <= 1'b0;
            r_bit       <= 3'd7;
            r_byte      <= 10'd0;
            r_nbytes    <= 10'd0;
            r_hdr       <= 3'd1;
            r_kind      <= K_NONE;
            r_opcode    <= 8'h00;
            r_xaddr     <= 24'h0;
            r_tx        <= 8'hFF;
            r_rx        <= 8'h00;
            r_dummy     <= 4'd0;
            o_sck       <= 1'b0;
            o_cs_n      <= 1'b1;
            o_mosi      <= 1'b1;
        end else begin
            case (r_state)
                // Wait for configuration to finish (EOS), then clock dummy
                // cycles with CS# high so CCLK is really on USRCCLKO.
                S_WAIT_EOS: begin
                    if (i_eos) begin
                        r_state   <= S_DUMMY;
                        r_dummy   <= 4'd0;
                        r_div_cnt <= 8'd0;
                    end
                end

                S_DUMMY: begin
                    if (w_tick) begin
                        r_div_cnt <= 8'd0;
                        o_sck     <= ~o_sck;
                        if (o_sck) begin           // falling edge completes a cycle
                            r_dummy <= r_dummy + 4'd1;
                            if (r_dummy == 4'd7) begin
                                r_state <= S_IDLE;
                                r_ready <= 1'b1;
                            end
                        end
                    end else begin
                        r_div_cnt <= r_div_cnt + 8'd1;
                    end
                end

                S_IDLE: begin
                    o_sck  <= 1'b0;
                    o_cs_n <= 1'b1;
                    r_busy <= 1'b0;
                    if (w_accept) begin
                        r_busy      <= 1'b1;
                        r_opcode    <= w_wire_op;
                        r_xaddr     <= r_addr;
                        r_kind      <= w_dec[1:0];
                        r_hdr       <= w_dec[2] ? 3'd5 : 3'd1;
                        r_nbytes    <= (w_dec[2] ? 10'd5 : 10'd1)
                                       + ((w_dec[1:0] == K_NONE) ? 10'd0 : w_len_in[9:0]);
                        r_byte      <= 10'd0;
                        r_bit       <= 3'd7;
                        r_tx        <= w_wire_op;
                        o_mosi      <= w_wire_op[7];
                        r_div_cnt   <= 8'd0;
                        r_sck_phase <= 1'b0;
                        o_cs_n      <= 1'b0;
                        r_state     <= S_CS_SETUP;
                    end
                end

                // CS# low, SCK low, MOSI = opcode MSB for one half-period
                // (this is the first low half of bit 7).
                S_CS_SETUP: begin
                    if (w_tick) begin
                        r_div_cnt   <= 8'd0;
                        o_sck       <= 1'b1;        // rising edge: flash samples bit 7
                        r_sck_phase <= 1'b1;
                        r_state     <= S_XFER;
                    end else begin
                        r_div_cnt <= r_div_cnt + 8'd1;
                    end
                end

                S_XFER: begin
                    if (w_tick) begin
                        r_div_cnt <= 8'd0;
                        if (!r_sck_phase) begin
                            o_sck       <= 1'b1;
                            r_sck_phase <= 1'b1;
                        end else begin
                            // end of high half: sample MISO, falling edge
                            r_rx        <= w_rx_full;
                            o_sck       <= 1'b0;
                            r_sck_phase <= 1'b0;
                            if (r_bit != 3'd0) begin
                                r_bit  <= r_bit - 3'd1;
                                o_mosi <= r_tx[r_bit - 3'd1];
                            end else if (w_next_byte == r_nbytes) begin
                                r_state <= S_CS_HOLD;
                                o_mosi  <= 1'b1;
                            end else begin
                                r_byte <= w_next_byte;
                                r_bit  <= 3'd7;
                                r_tx   <= w_nb;
                                o_mosi <= w_nb[7];
                            end
                        end
                    end else begin
                        r_div_cnt <= r_div_cnt + 8'd1;
                    end
                end

                // Hold SCK low for half a period, then raise CS# and keep it
                // high for >= 8 cycles before accepting the next command.
                S_CS_HOLD: begin
                    if (!o_cs_n) begin
                        if (w_tick) begin
                            o_cs_n  <= 1'b1;
                            r_dummy <= 4'd0;
                        end else begin
                            r_div_cnt <= r_div_cnt + 8'd1;
                        end
                    end else begin
                        r_dummy <= r_dummy + 4'd1;
                        if (r_dummy == 4'd7) begin
                            r_state <= S_IDLE;
                            r_busy  <= 1'b0;
                        end
                    end
                end

                default: r_state <= S_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Read mux — combinational (registered in KlaussCPU's MMIO read path).
    // -----------------------------------------------------------------------
    always_comb begin
        mmio.read_data = 64'h0;
        if (is_buf) begin
            mmio.read_data = r_buf[buf_idx];
        end else begin
            case (mmio.addr[15:0])
                OFF_CMD:    mmio.read_data = {32'b0, r_cmd_word};
                OFF_ADDR:   mmio.read_data = {40'b0, r_addr};
                OFF_STATUS: mmio.read_data = {60'b0, r_dropped, r_ready, r_rejected, r_busy};
                OFF_CLKDIV: mmio.read_data = {56'b0, r_clk_div};
                OFF_FLOOR:  mmio.read_data = {40'b0, PROT_FLOOR};
                OFF_ID:     mmio.read_data = ID_VALUE;
                default:    mmio.read_data = 64'h0;
            endcase
        end
    end

endmodule

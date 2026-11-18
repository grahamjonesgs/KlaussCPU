`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// sd_spi — bare-metal SPI controller for the Nexys A7 microSD slot, exposed as
// MMIO (base address 0xF000_0000, see MMIO_MAP.md).
//
// Design philosophy: hardware does the smallest useful primitive — single-byte
// SPI transfer, CS/power/clock control, and a 512-byte sector buffer. The full
// SD protocol (CMD0, ACMD41, CMD17, CMD24, CRC, …) lives in software because
// it is easier to debug, more flexible (multi-block, error recovery), and
// dramatically smaller in fabric than a full hardware FSM.
//
// SPI mode 0 (CPOL=0, CPHA=0), MSB first — what every SD card expects in SPI
// mode. Clock divisor is software-programmable so the same hardware can do the
// SD-spec ≤400 kHz init phase and ~25 MHz post-init operation.
//
// Register layout (offsets within the SD device window):
//   0x000  SD_CTRL    RW  [15:0] clk_div, [16] cs_n, [17] pwr_en
//   0x008  SD_DATA    W   [7:0]  byte to send (kicks off transfer)
//                      R   [7:0]  last byte received
//   0x010  SD_STATUS  R   [0] busy, [1] card_present
//   0x200..0x3F8       RW  512-byte sector buffer (64 × 64-bit doublewords)
//
// Timing: with i_Clk = 100 MHz, SCK period = 2 × (clk_div + 1) × 10 ns.
//   clk_div = 124 → ~400 kHz (use during init)
//   clk_div = 1   → ~25 MHz  (max for SD spec)
//
// SD_RESET on the Nexys A7 is an active-low power-gate to the slot; we
// expose it as pwr_en (1 = power on). Slot is OFF at reset — software must
// set pwr_en before doing anything else.
//
// SD_DAT[1] and SD_DAT[2] are unused in SPI mode but should not float; we
// drive them high.
//
// Native SD mode (SD_CTRL[18] = SD_MODE, SD_NATIVE_PLAN.md): the sd_native
// engine (1-bit / 4-bit SD bus, command and data engines, a 4 KiB FIFO,
// registers 0x100..0x1F8) takes over the pins.  With SD_MODE = 0 the block
// is the SPI controller above, unchanged: same registers, pin values and
// timing (CMD = MOSI, DAT3 = CS_n, DAT1/DAT2 driven high, DAT0 = MISO).
// Reset: CPU_RESETN resets everything; a program load (i_load_rst) resets
// the whole controller as CPU_RESETN does iff SD_MODE = 1 (clearing SD_MODE
// and pwr_en), and changes nothing if SD_MODE = 0.
//
// Pads (SD_NATIVE_PLAN.md §9, phase 3: high speed).  Every SD output leaves
// through an IOB register: CLK through an ODDR (a forwarded clock), CMD and
// DAT[3:0] through the OLOGIC output and tristate flip-flops.  They take the
// NEXT value of what the pins carried before phase 3 (the SD_CTRL fields,
// the SPI shifter's SCK / MOSI, sd_native's o_*_nx), so every pin changes
// on the same edge as before, now with a fixed clock-to-pad delay (checked
// every cycle in simulation against the old combinational mux).  CMD and
// DAT[3:0] come in through an IDELAYE2 (VAR_LOAD; SDM_CFG[12:8], 78 ps a
// tap) into an IDDR (SAME_EDGE: both edges, 5 ns apart), for sd_native's
// high-speed sample phases; Q1 is the old IOB input flip-flop.  The SPI
// shifter's MISO still takes the pin itself (the IBUF), as before.
// IDELAYCTRL (bank 35's clock region, X1Y2; the MIG's is in X1Y1) runs on
// the 200 MHz clk_wiz output that is also the MIG's reference.
//////////////////////////////////////////////////////////////////////////////////

module sd_spi (
    input             i_Clk,
    input             i_clk_ref,       // 200 MHz IDELAYCTRL reference (clk_wiz_0; the MIG's)
    input             i_Rst_L,
    input             i_load_rst,      // program load (LOAD_COMPLETE) pulse

    // ----- MMIO peripheral bus (slave). Offsets use mmio.addr[15:0]. -----
    mmio_if.slave     mmio,
    output            o_irq,           // native-mode interrupt (level, registered)

    // ----- SD card pins (Nexys A7 microSD slot) -----
    input             i_sd_cd,         // card detect (board-dependent polarity)
    output            o_sd_reset_n,    // active-low slot power gate (0 = powered)
    output            o_sd_sck,        // SCK / CLK (the ODDR)
    output            o_sd_cmd,        // CMD (SPI: MOSI)
    output            o_sd_cmd_oe,     // (= ~o_sd_cmd_t; for testbenches)
    output            o_sd_cmd_t,      // CMD tristate (1 = released): the OBUFT's T, IOB register
    input             i_sd_cmd,        // the pins (IBUF outputs)
    output     [3:0]  o_sd_dat,        // DAT[3:0] (SPI: DAT3 = CS_n, DAT1/2 high)
    output     [3:0]  o_sd_dat_oe,
    output     [3:0]  o_sd_dat_t,
    input      [3:0]  i_sd_dat,        // (SPI: DAT0 = MISO)

    // ----- SD DMA: DDR master E on mem_read_write's arbiter (sd_native) -----
    output            o_dma_req,
    output            o_dma_done,
    output            o_dma_write_DV,
    output            o_dma_read_DV,
    output     [31:0] o_dma_addr,
    output    [127:0] o_dma_write_data,
    output     [15:0] o_dma_wdf_mask,
    input     [127:0] i_dma_read_data,
    input             i_dma_ready,
    input             i_dma_grant
);

    // -----------------------------------------------------------------------
    // Address constants (offsets within sd_spi's device window).
    // -----------------------------------------------------------------------
    localparam OFF_CTRL   = 16'h0000;
    localparam OFF_DATA   = 16'h0008;
    localparam OFF_STATUS = 16'h0010;
    // Sector buffer: 0x200..0x3F8 (64 × 8-byte doublewords).
    // addr[9] selects buffer vs registers; addr[8:3] is the doubleword index.

    wire is_buf = mmio.addr[9];
    wire [5:0] buf_idx = mmio.addr[8:3];

    // MMIO ready: combinational always-1. Buffer is distributed RAM with
    // single-cycle read; register reads are also combinational.
    assign mmio.ready = 1'b1;

    // -----------------------------------------------------------------------
    // Control register
    // -----------------------------------------------------------------------
    logic [15:0] r_clk_div;     // half-period of SCK, in i_Clk cycles minus 1
    logic        r_pwr_en;      // 1 → slot powered (drives SD_RESET low)
    logic        r_sd_mode;     // SD_CTRL[18]: 1 = native SD bus (sd_native)
    logic        o_sd_cs_n;     // SPI chip select, written directly from SD_CTRL[16]
    logic        o_sd_sck_spi;  // SPI shifter's SCK and MOSI
    logic        o_sd_mosi;
    wire         i_sd_miso = i_sd_dat[0];

    assign o_sd_reset_n = ~r_pwr_en;

    // Program-load reset: only when the last program used native mode.
    wire w_soft_rst = i_load_rst && r_sd_mode;

    // SD_CTRL's next values (the register block below; the pads take them)
    wire w_ctrl_wr  = mmio.write_DV && (mmio.addr[15:0] == OFF_CTRL);
    wire nx_pwr_en  = w_soft_rst ? 1'b0 : w_ctrl_wr ? mmio.write_data[17] : r_pwr_en;
    wire nx_sd_mode = w_soft_rst ? 1'b0 : w_ctrl_wr ? mmio.write_data[18] : r_sd_mode;
    wire nx_cs_n    = w_soft_rst ? 1'b1 : w_ctrl_wr ? mmio.write_data[16] : o_sd_cs_n;

    // -----------------------------------------------------------------------
    // Byte transfer FSM (SPI mode 0, MSB first).
    //   For each of 8 bits:
    //     low half:  SCK=0, present current bit on MOSI
    //     high half: SCK=1, sample MISO into shift register
    //   After 8 bits: latch rx_shift into r_rx_byte, return to idle.
    // -----------------------------------------------------------------------
    localparam BYTE_IDLE = 1'b0, BYTE_SHIFT = 1'b1;

    logic        r_byte_sm;
    logic [ 2:0] r_bit_idx;       // 7 → 0
    logic        r_sck_phase;     // 0 = SCK low half, 1 = SCK high half
    logic [15:0] r_phase_count;
    logic [ 7:0] r_tx_shift;
    logic [ 7:0] r_rx_shift;
    logic [ 7:0] r_rx_byte;
    logic        r_byte_busy;

    wire w_kick_byte = mmio.write_DV
                       && (mmio.addr[15:0] == OFF_DATA)
                       && !r_byte_busy;

    // SCK's and MOSI's next values: exactly the FSM's assignments below
    // (it registers them; the pads take them)
    logic nx_sck_spi, nx_mosi;
    always_comb begin
        nx_sck_spi = o_sd_sck_spi;
        nx_mosi    = o_sd_mosi;
        if (w_soft_rst) begin
            nx_sck_spi = 1'b0;
            nx_mosi    = 1'b1;
        end else if (r_byte_sm == BYTE_IDLE) begin
            nx_sck_spi = 1'b0;
            if (w_kick_byte) nx_mosi = mmio.write_data[7];
        end else if (r_phase_count >= r_clk_div) begin
            if (r_sck_phase == 1'b0) begin
                nx_sck_spi = 1'b1;
            end else begin
                nx_sck_spi = 1'b0;
                nx_mosi    = (r_bit_idx == 3'd0) ? 1'b1 : r_tx_shift[6];
            end
        end
    end

    always_ff @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_byte_sm     <= BYTE_IDLE;
            r_bit_idx     <= 3'd7;
            r_sck_phase   <= 1'b0;
            r_phase_count <= 16'd0;
            r_tx_shift    <= 8'hFF;
            r_rx_shift    <= 8'h00;
            r_rx_byte     <= 8'hFF;
            r_byte_busy   <= 1'b0;
            o_sd_sck_spi  <= 1'b0;
            o_sd_mosi     <= 1'b1;
        end else if (w_soft_rst) begin
            r_byte_sm     <= BYTE_IDLE;
            r_bit_idx     <= 3'd7;
            r_sck_phase   <= 1'b0;
            r_phase_count <= 16'd0;
            r_tx_shift    <= 8'hFF;
            r_rx_shift    <= 8'h00;
            r_rx_byte     <= 8'hFF;
            r_byte_busy   <= 1'b0;
            o_sd_sck_spi  <= 1'b0;
            o_sd_mosi     <= 1'b1;
        end else begin
            case (r_byte_sm)
                BYTE_IDLE: begin
                    o_sd_sck_spi  <= 1'b0;
                    r_byte_busy   <= 1'b0;
                    r_phase_count <= 16'd0;
                    r_sck_phase   <= 1'b0;
                    if (w_kick_byte) begin
                        r_tx_shift  <= mmio.write_data[7:0];
                        o_sd_mosi   <= mmio.write_data[7];   // present MSB before first SCK edge
                        r_bit_idx   <= 3'd7;
                        r_byte_busy <= 1'b1;
                        r_byte_sm   <= BYTE_SHIFT;
                    end
                end

                BYTE_SHIFT: begin
                    if (r_phase_count >= r_clk_div) begin
                        r_phase_count <= 16'd0;
                        if (r_sck_phase == 1'b0) begin
                            // End of SCK-low half — drive SCK high; slave samples MOSI on this edge.
                            o_sd_sck_spi <= 1'b1;
                            r_sck_phase <= 1'b1;
                        end else begin
                            // End of SCK-high half — sample MISO, then either advance bit or finish.
                            r_rx_shift  <= {r_rx_shift[6:0], i_sd_miso};
                            o_sd_sck_spi <= 1'b0;
                            r_sck_phase <= 1'b0;
                            if (r_bit_idx == 3'd0) begin
                                r_rx_byte   <= {r_rx_shift[6:0], i_sd_miso};
                                o_sd_mosi   <= 1'b1;
                                r_byte_busy <= 1'b0;
                                r_byte_sm   <= BYTE_IDLE;
                            end else begin
                                r_bit_idx  <= r_bit_idx - 3'd1;
                                r_tx_shift <= {r_tx_shift[6:0], 1'b0};
                                o_sd_mosi  <= r_tx_shift[6];   // next bit (MSB-1)
                            end
                        end
                    end else begin
                        r_phase_count <= r_phase_count + 16'd1;
                    end
                end
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Sector buffer — 64 × 64-bit = 512 bytes, distributed RAM (combinational
    // read, byte-enabled write). Small enough to keep in LUTRAM and avoids
    // adding a wait-state to mmio.ready.
    // -----------------------------------------------------------------------
    logic [63:0] r_sector_buf [0:63];

    integer bi;
    always_ff @(posedge i_Clk) begin
        if (mmio.write_DV && is_buf) begin
            for (bi = 0; bi < 8; bi = bi + 1) begin
                if (mmio.byte_en[bi])
                    r_sector_buf[buf_idx][bi*8 +: 8] <= mmio.write_data[bi*8 +: 8];
            end
        end
    end

    // -----------------------------------------------------------------------
    // Control register write
    // -----------------------------------------------------------------------
    always_ff @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_clk_div <= 16'd249;   // ~200 kHz init speed @ 100 MHz i_Clk
            o_sd_cs_n <= 1'b1;       // CS deasserted
            r_pwr_en  <= 1'b0;       // slot off
            r_sd_mode <= 1'b0;       // SPI mode
        end else if (w_soft_rst) begin
            r_clk_div <= 16'd249;
            o_sd_cs_n <= 1'b1;
            r_pwr_en  <= 1'b0;
            r_sd_mode <= 1'b0;
        end else if (mmio.write_DV && (mmio.addr[15:0] == OFF_CTRL)) begin
            r_clk_div <= mmio.write_data[15:0];
            o_sd_cs_n <= mmio.write_data[16];
            r_pwr_en  <= mmio.write_data[17];
            r_sd_mode <= mmio.write_data[18];
        end
    end
`ifndef SYNTHESIS
    // the next values above are the registers' next values
    logic       r_nx_chk = 1'b0;
    logic [4:0] r_nx_q;
    always_ff @(posedge i_Clk) begin
        r_nx_chk <= i_Rst_L;
        r_nx_q   <= {nx_pwr_en, nx_sd_mode, nx_cs_n, nx_sck_spi, nx_mosi};
        if (r_nx_chk && i_Rst_L && {r_pwr_en, r_sd_mode, o_sd_cs_n, o_sd_sck_spi, o_sd_mosi} !== r_nx_q)
            $fatal(1, "sd_spi: SD_CTRL / SPI next-value mismatch: %b vs %b",
                   {r_pwr_en, r_sd_mode, o_sd_cs_n, o_sd_sck_spi, o_sd_mosi}, r_nx_q);
    end
`endif

    // -----------------------------------------------------------------------
    // Native SD engine (SD_MODE = 1) and the pin mux
    // -----------------------------------------------------------------------
    // Its own synchronous reset: CPU_RESETN (i_Rst_L comes from the
    // synchronised w_reset_H), or the program-load reset.
    wire         w_nat_rst = ~i_Rst_L | w_soft_rst;

    wire [63:0]  w_nat_rdata;
    wire         w_nat_clk, w_nat_cmd, w_nat_cmd_oe;
    wire [3:0]   w_nat_dat, w_nat_dat_oe;
    wire         w_nat_clk_nx, w_nat_cmd_nx, w_nat_cmd_oe_nx;
    wire [3:0]   w_nat_dat_nx, w_nat_dat_oe_nx;
    wire         w_cmd_q1, w_cmd_q2;
    wire [3:0]   w_dat_q1, w_dat_q2;
    wire [4:0]   w_idly_tap;
    wire         w_idly_ld;
    logic        r_idc_rdy_s;

    sd_native native_i (
        .i_Clk     (i_Clk),
        .i_rst     (w_nat_rst),
        .i_sd_mode (r_sd_mode),
        .i_clk_div (r_clk_div),
        .i_wr_dv   (mmio.write_DV),
        .i_rd_dv   (mmio.read_DV),
        .i_addr    (mmio.addr[15:0]),
        .i_wdata   (mmio.write_data),
        .o_rdata   (w_nat_rdata),
        .o_irq     (o_irq),
        .o_clk     (w_nat_clk),
        .o_cmd     (w_nat_cmd),
        .o_cmd_oe  (w_nat_cmd_oe),
        .o_dat     (w_nat_dat),
        .o_dat_oe  (w_nat_dat_oe),
        .o_clk_nx    (w_nat_clk_nx),
        .o_cmd_nx    (w_nat_cmd_nx),
        .o_cmd_oe_nx (w_nat_cmd_oe_nx),
        .o_dat_nx    (w_nat_dat_nx),
        .o_dat_oe_nx (w_nat_dat_oe_nx),
        .i_cmd_q1  (w_cmd_q1),
        .i_cmd_q2  (w_cmd_q2),
        .i_dat_q1  (w_dat_q1),
        .i_dat_q2  (w_dat_q2),
        .i_cd      (i_sd_cd),
        .o_idly_tap (w_idly_tap),
        .o_idly_ld  (w_idly_ld),
        .i_idly_rdy (r_idc_rdy_s),
        .o_dma_req        (o_dma_req),
        .o_dma_done       (o_dma_done),
        .o_dma_write_DV   (o_dma_write_DV),
        .o_dma_read_DV    (o_dma_read_DV),
        .o_dma_addr       (o_dma_addr),
        .o_dma_write_data (o_dma_write_data),
        .o_dma_wdf_mask   (o_dma_wdf_mask),
        .i_dma_read_data  (i_dma_read_data),
        .i_dma_ready      (i_dma_ready),
        .i_dma_grant      (i_dma_grant)
    );

    // Slot powered (pwr_en = 1): SPI mode keeps every pin's old direction and
    // value; SD mode hands the pins to the native engine.
    // Slot unpowered (pwr_en = 0), either mode: CLK, CMD and DAT[3:0] are all
    // driven LOW.  Driving them high (SPI's MOSI/CS/DAT1/DAT2 idle level) or
    // leaving them on the PULLUPs back-powers the card through its I/O pins,
    // so cutting the slot's power never resets it: on the board, a card left
    // in SD mode stayed there across a 1 s power-off (and vice versa), which
    // breaks the SPI <-> native mode switch (SD_NATIVE_PLAN.md §4.1).
    //
    // The pins are IOB registers loaded with the next value of that mux
    // (see the header): the same values on the same edges as the mux of the
    // registers (w_ref_*, phases 1-2's pins; compared in simulation).
    wire       w_on = r_pwr_en;
    wire       w_ref_sck    = w_on & (r_sd_mode ? w_nat_clk : o_sd_sck_spi);
    wire       w_ref_cmd    = w_on & (r_sd_mode ? w_nat_cmd : o_sd_mosi);
    wire       w_ref_cmd_oe = ~w_on | (r_sd_mode ? w_nat_cmd_oe : 1'b1);
    wire [3:0] w_ref_dat    = {4{w_on}} & (r_sd_mode ? w_nat_dat : {o_sd_cs_n, 1'b1, 1'b1, 1'b1});
    wire [3:0] w_ref_dat_oe = {4{~w_on}} | (r_sd_mode ? w_nat_dat_oe : 4'b1110);

    wire       nx_on        = nx_pwr_en;
    wire       nx_sck       = nx_on & (nx_sd_mode ? w_nat_clk_nx : nx_sck_spi);
    wire       nx_cmd       = nx_on & (nx_sd_mode ? w_nat_cmd_nx : nx_mosi);
    wire       nx_cmd_oe    = ~nx_on | (nx_sd_mode ? w_nat_cmd_oe_nx : 1'b1);
    wire [3:0] nx_dat       = {4{nx_on}} & (nx_sd_mode ? w_nat_dat_nx : {nx_cs_n, 1'b1, 1'b1, 1'b1});
    wire [3:0] nx_dat_oe    = {4{~nx_on}} | (nx_sd_mode ? w_nat_dat_oe_nx : 4'b1110);

    // CMD / DAT: OLOGIC output and tristate flip-flops (T = 1: released).
    // Reset (CPU_RESETN, asynchronous as SD_CTRL): every line driven low, as
    // pwr_en = 0 drives them.
    // (Vivado replicates a register with IOB = TRUE that synthesis would
    // otherwise share between pins: DAT[3:1]'s tristate controls are equal)
    (* IOB = "TRUE" *) logic       r_cmd_o;
    (* IOB = "TRUE" *) logic       r_cmd_t;
    (* IOB = "TRUE" *) logic [3:0] r_dat_o;
    (* IOB = "TRUE" *) logic [3:0] r_dat_t;
    always_ff @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_cmd_o <= 1'b0;  r_cmd_t <= 1'b0;
            r_dat_o <= 4'h0;  r_dat_t <= 4'h0;
        end else begin
            r_cmd_o <= nx_cmd;  r_cmd_t <= ~nx_cmd_oe;
            r_dat_o <= nx_dat;  r_dat_t <= ~nx_dat_oe;
        end
    end
    assign o_sd_cmd    = r_cmd_o;
    assign o_sd_cmd_t  = r_cmd_t;
    assign o_sd_cmd_oe = ~r_cmd_t;
    assign o_sd_dat    = r_dat_o;
    assign o_sd_dat_t  = r_dat_t;
    assign o_sd_dat_oe = ~r_dat_t;

    // CLK: a forwarded clock through an ODDR with D1 = D2 (the pin changes
    // only on the rising edge, as the other outputs)
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) oddr_sck (
        .Q(o_sd_sck), .C(i_Clk), .CE(1'b1), .D1(nx_sck), .D2(nx_sck), .R(~i_Rst_L), .S(1'b0));

    // CMD / DAT[3:0] in: IDELAYE2 (the tap: SDM_CFG[12:8]) -> IDDR
    wire [4:0] w_pin_in = {i_sd_dat, i_sd_cmd};
    wire [4:0] w_pin_dly, w_q1, w_q2;
    for (genvar gi = 0; gi < 5; gi++) begin : g_in
        (* IODELAY_GROUP = "SD_IODELAY" *)
        IDELAYE2 #(.IDELAY_TYPE("VAR_LOAD"), .DELAY_SRC("IDATAIN"), .IDELAY_VALUE(0),
                   .HIGH_PERFORMANCE_MODE("TRUE"), .SIGNAL_PATTERN("DATA"),
                   .REFCLK_FREQUENCY(200.0), .CINVCTRL_SEL("FALSE"), .PIPE_SEL("FALSE"))
        idly (.IDATAIN(w_pin_in[gi]), .DATAOUT(w_pin_dly[gi]), .C(i_Clk),
              .LD(w_idly_ld), .CNTVALUEIN(w_idly_tap), .CNTVALUEOUT(),
              .CE(1'b0), .INC(1'b0), .REGRST(1'b0), .CINVCTRL(1'b0), .LDPIPEEN(1'b0),
              .DATAIN(1'b0));
        IDDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT_Q1(1'b1), .INIT_Q2(1'b1), .SRTYPE("SYNC"))
        iddr (.Q1(w_q1[gi]), .Q2(w_q2[gi]), .C(i_Clk), .CE(1'b1), .D(w_pin_dly[gi]),
              .R(1'b0), .S(1'b0));
    end
    assign {w_dat_q1, w_cmd_q1} = w_q1;
    assign {w_dat_q2, w_cmd_q2} = w_q2;

    // IDELAYCTRL: reset for 64 cycles (>= 60 ns) after CPU_RESETN (the
    // 200 MHz reference runs before ui_clk does: the MIG's PLL / MMCM lock
    // on it); RDY (an asynchronous level) through two flip-flops
    logic [6:0] r_idc_cnt = 7'd0;
    always_ff @(posedge i_Clk)
        if (~i_Rst_L)             r_idc_cnt <= 7'd0;
        else if (!r_idc_cnt[6])   r_idc_cnt <= r_idc_cnt + 7'd1;
    wire w_idc_rdy;
    (* IODELAY_GROUP = "SD_IODELAY" *)
    IDELAYCTRL idelayctrl_sd (.REFCLK(i_clk_ref), .RST(!r_idc_cnt[6]), .RDY(w_idc_rdy));
    (* ASYNC_REG = "TRUE" *) logic [1:0] r_idc_rdy_sync = 2'b00;
    always_ff @(posedge i_Clk) r_idc_rdy_sync <= {r_idc_rdy_sync[0], w_idc_rdy};
    assign r_idc_rdy_s = r_idc_rdy_sync[1];

`ifndef SYNTHESIS
    // the pads carry exactly what the pre-phase-3 mux of the registers did
    // (compared mid-cycle, after the ODDR's output has settled)
    always @(negedge i_Clk)
        if ({o_sd_sck, o_sd_cmd, o_sd_cmd_oe, o_sd_dat, o_sd_dat_oe} !==
            {w_ref_sck, w_ref_cmd, w_ref_cmd_oe, w_ref_dat, w_ref_dat_oe})
            $fatal(1, "sd_spi: pads %b differ from the register mux %b",
                   {o_sd_sck, o_sd_cmd, o_sd_cmd_oe, o_sd_dat, o_sd_dat_oe},
                   {w_ref_sck, w_ref_cmd, w_ref_cmd_oe, w_ref_dat, w_ref_dat_oe});
`endif

    // -----------------------------------------------------------------------
    // Read mux — combinational. Returns 0 for undefined offsets (including
    // 0x100..0x1F8 here).  The native registers are ORed in from sd_native,
    // which decodes the address of the previous cycle (registered) and
    // gives 0 outside its window: an MMIO read holds its address and the
    // SoC samples the data in the second cycle, so the raw address never
    // reaches the native decode.
    // -----------------------------------------------------------------------
    always_comb begin
        mmio.read_data = 64'h0;
        if (is_buf) begin
            mmio.read_data = r_sector_buf[buf_idx];
        end else begin
            case (mmio.addr[15:0])
                OFF_CTRL:   mmio.read_data = {45'b0, r_sd_mode, r_pwr_en, o_sd_cs_n, r_clk_div};
                OFF_DATA:   mmio.read_data = {56'b0, r_rx_byte};
                OFF_STATUS: mmio.read_data = {62'b0, i_sd_cd, r_byte_busy};
                default:    mmio.read_data = 64'h0;
            endcase
        end
        mmio.read_data = mmio.read_data | w_nat_rdata;
    end

endmodule

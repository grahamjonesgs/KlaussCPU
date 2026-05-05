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
//////////////////////////////////////////////////////////////////////////////////

module sd_spi (
    input             i_Clk,
    input             i_Rst_L,

    // ----- MMIO interface (offsets within device window, addr[15:0]) -----
    input             i_mmio_write_DV,
    input             i_mmio_read_DV,
    input      [15:0] i_mmio_addr,
    input      [63:0] i_mmio_write_data,
    input      [ 7:0] i_mmio_byte_en,
    output reg [63:0] o_mmio_read_data,
    output            o_mmio_ready,

    // ----- SD card pins (Nexys A7 microSD slot) -----
    input             i_sd_cd,         // card detect (board-dependent polarity)
    output            o_sd_reset_n,    // active-low slot power gate (0 = powered)
    output reg        o_sd_sck,
    output reg        o_sd_mosi,
    input             i_sd_miso,
    output reg        o_sd_cs_n,
    output            o_sd_dat1,       // unused in SPI mode — drive high
    output            o_sd_dat2        // unused in SPI mode — drive high
);

    // -----------------------------------------------------------------------
    // Address constants (offsets within sd_spi's device window).
    // -----------------------------------------------------------------------
    localparam OFF_CTRL   = 16'h0000;
    localparam OFF_DATA   = 16'h0008;
    localparam OFF_STATUS = 16'h0010;
    // Sector buffer: 0x200..0x3F8 (64 × 8-byte doublewords).
    // addr[9] selects buffer vs registers; addr[8:3] is the doubleword index.

    wire is_buf = i_mmio_addr[9];
    wire [5:0] buf_idx = i_mmio_addr[8:3];

    // MMIO ready: combinational always-1. Buffer is distributed RAM with
    // single-cycle read; register reads are also combinational.
    assign o_mmio_ready = 1'b1;

    // -----------------------------------------------------------------------
    // Control register
    // -----------------------------------------------------------------------
    reg [15:0] r_clk_div;     // half-period of SCK, in i_Clk cycles minus 1
    reg        r_pwr_en;      // 1 → slot powered (drives SD_RESET low)
    // o_sd_cs_n is a reg, written directly from SD_CTRL[16] writes.

    assign o_sd_reset_n = ~r_pwr_en;
    assign o_sd_dat1    = 1'b1;
    assign o_sd_dat2    = 1'b1;

    // -----------------------------------------------------------------------
    // Byte transfer FSM (SPI mode 0, MSB first).
    //   For each of 8 bits:
    //     low half:  SCK=0, present current bit on MOSI
    //     high half: SCK=1, sample MISO into shift register
    //   After 8 bits: latch rx_shift into r_rx_byte, return to idle.
    // -----------------------------------------------------------------------
    localparam BYTE_IDLE = 1'b0, BYTE_SHIFT = 1'b1;

    reg        r_byte_sm;
    reg [ 2:0] r_bit_idx;       // 7 → 0
    reg        r_sck_phase;     // 0 = SCK low half, 1 = SCK high half
    reg [15:0] r_phase_count;
    reg [ 7:0] r_tx_shift;
    reg [ 7:0] r_rx_shift;
    reg [ 7:0] r_rx_byte;
    reg        r_byte_busy;

    wire w_kick_byte = i_mmio_write_DV
                       && (i_mmio_addr == OFF_DATA)
                       && !r_byte_busy;

    always @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_byte_sm     <= BYTE_IDLE;
            r_bit_idx     <= 3'd7;
            r_sck_phase   <= 1'b0;
            r_phase_count <= 16'd0;
            r_tx_shift    <= 8'hFF;
            r_rx_shift    <= 8'h00;
            r_rx_byte     <= 8'hFF;
            r_byte_busy   <= 1'b0;
            o_sd_sck      <= 1'b0;
            o_sd_mosi     <= 1'b1;
        end else begin
            case (r_byte_sm)
                BYTE_IDLE: begin
                    o_sd_sck      <= 1'b0;
                    r_byte_busy   <= 1'b0;
                    r_phase_count <= 16'd0;
                    r_sck_phase   <= 1'b0;
                    if (w_kick_byte) begin
                        r_tx_shift  <= i_mmio_write_data[7:0];
                        o_sd_mosi   <= i_mmio_write_data[7];   // present MSB before first SCK edge
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
                            o_sd_sck    <= 1'b1;
                            r_sck_phase <= 1'b1;
                        end else begin
                            // End of SCK-high half — sample MISO, then either advance bit or finish.
                            r_rx_shift  <= {r_rx_shift[6:0], i_sd_miso};
                            o_sd_sck    <= 1'b0;
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
    // adding a wait-state to o_mmio_ready.
    // -----------------------------------------------------------------------
    reg [63:0] r_sector_buf [0:63];

    integer bi;
    always @(posedge i_Clk) begin
        if (i_mmio_write_DV && is_buf) begin
            for (bi = 0; bi < 8; bi = bi + 1) begin
                if (i_mmio_byte_en[bi])
                    r_sector_buf[buf_idx][bi*8 +: 8] <= i_mmio_write_data[bi*8 +: 8];
            end
        end
    end

    // -----------------------------------------------------------------------
    // Control register write
    // -----------------------------------------------------------------------
    always @(posedge i_Clk or negedge i_Rst_L) begin
        if (~i_Rst_L) begin
            r_clk_div <= 16'd249;   // ~200 kHz init speed @ 100 MHz i_Clk
            o_sd_cs_n <= 1'b1;       // CS deasserted
            r_pwr_en  <= 1'b0;       // slot off
        end else if (i_mmio_write_DV && (i_mmio_addr == OFF_CTRL)) begin
            r_clk_div <= i_mmio_write_data[15:0];
            o_sd_cs_n <= i_mmio_write_data[16];
            r_pwr_en  <= i_mmio_write_data[17];
        end
    end

    // -----------------------------------------------------------------------
    // Read mux — combinational. Returns 0 for undefined offsets.
    // -----------------------------------------------------------------------
    always @* begin
        o_mmio_read_data = 64'h0;
        if (is_buf) begin
            o_mmio_read_data = r_sector_buf[buf_idx];
        end else begin
            case (i_mmio_addr)
                OFF_CTRL:   o_mmio_read_data = {46'b0, r_pwr_en, o_sd_cs_n, r_clk_div};
                OFF_DATA:   o_mmio_read_data = {56'b0, r_rx_byte};
                OFF_STATUS: o_mmio_read_data = {62'b0, i_sd_cd, r_byte_busy};
                default:    o_mmio_read_data = 64'h0;
            endcase
        end
    end

endmodule

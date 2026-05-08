`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// eth_mmio_bridge — bridges the CPU's 64-bit MMIO bus to LiteEth's 32-bit
// classic Wishbone slave.
//
// Address translation (byte-for-byte across the 192 KiB Eth window):
//   eth_byte_addr  = i_mmio_addr - 32'hF006_0000
//   wishbone_adr   = eth_byte_addr[19:2]      // word address; LiteEth ignores upper bits
//
// The CPU bus is 64-bit; LiteEth is 32-bit.  The 8 byte-enables tell us which
// 4-byte half(s) of the doubleword are live:
//     byte_en[3:0] != 0 → low  32-bit half is in play (addr + 0)
//     byte_en[7:4] != 0 → high 32-bit half is in play (addr + 4)
// 64-bit accesses (both halves active) issue two Wishbone cycles back-to-back
// with a one-cycle STB-low gap between them so the LiteEth slave sees them as
// two distinct classic transactions (not a B4 burst).
//
// Wishbone signals always driven for single-transfer mode:
//   o_wb_bte = 2'b00, o_wb_cti = 3'b000
//
// CPU holds i_mmio_addr / i_mmio_write_data / i_mmio_byte_en stable until
// o_mmio_ready, so we don't have to latch the request — the WB outputs read
// directly from the inputs each cycle.
//////////////////////////////////////////////////////////////////////////////////

module eth_mmio_bridge (
    input             i_clk,
    input             i_rst,             // active-high

    // -------- CPU MMIO side (matches bus_splitter MMIO output) --------
    input             i_mmio_write_DV,
    input             i_mmio_read_DV,
    input      [31:0] i_mmio_addr,
    input      [63:0] i_mmio_write_data,
    input      [ 7:0] i_mmio_byte_en,
    output reg [63:0] o_mmio_read_data,
    output reg        o_mmio_ready,

    // -------- LiteEth Wishbone classic master --------
    output reg [29:0] o_wb_adr,
    output reg [31:0] o_wb_dat_w,
    output reg [ 3:0] o_wb_sel,
    output reg        o_wb_we,
    output reg        o_wb_cyc,
    output reg        o_wb_stb,
    output     [ 1:0] o_wb_bte,
    output     [ 2:0] o_wb_cti,
    input      [31:0] i_wb_dat_r,
    input             i_wb_ack,
    input             i_wb_err           // unused for now; tie at top if needed
);

    // ---- Wishbone single-transfer (B4 classic) ----
    assign o_wb_bte = 2'b00;
    assign o_wb_cti = 3'b000;

    // ---- Address derivation (combinational from inputs) ----
    //
    // The CPU may send i_mmio_addr either:
    //   (a) doubleword-aligned (low 3 bits = 0), with byte_en selecting
    //       which half is in play, OR
    //   (b) at the actual byte address (bit 2 reflecting which half)
    //
    // The cache (mem_read_write.v) ignores bits [2:0] in favour of
    // byte_en, so the convention is "byte_en is authoritative."  We
    // align the address to 8 bytes here and derive the WB word index
    // purely from byte_en.  Without the alignment step, a CPU that
    // sends addr=0xF006_0004 with byte_en=0xF0 hits eth_word_addr=1,
    // and the +1 offset for the high half pushes the access to word 2
    // (ctrl_bus_errors) instead of word 1 (ctrl_scratch).
    wire [31:0] eth_byte_aligned = {i_mmio_addr[31:3], 3'b000};
    wire [31:0] eth_byte_offset  = eth_byte_aligned - 32'hF006_0000;
    wire [29:0] eth_word_lo      = eth_byte_offset[31:2];        // bit 0 = 0
    wire [29:0] eth_word_hi      = eth_word_lo | 30'd1;          // bit 0 = 1 (+4 bytes)
    wire        low_active       = |i_mmio_byte_en[3:0];
    wire        high_active      = |i_mmio_byte_en[7:4];
    wire        is_write         = i_mmio_write_DV;
    wire        is_read          = i_mmio_read_DV;

    // ---- FSM ----
    localparam S_IDLE   = 3'd0;
    localparam S_LO     = 3'd1;          // driving low-half WB cycle
    localparam S_LO_GAP = 3'd2;          // STB low between LO and HI (only if both halves)
    localparam S_HI     = 3'd3;          // driving high-half WB cycle
    localparam S_DONE   = 3'd4;          // pulse o_mmio_ready, return to IDLE

    reg [2:0] state;

    always @(posedge i_clk) begin
        if (i_rst) begin
            state            <= S_IDLE;
            o_wb_adr         <= 30'b0;
            o_wb_dat_w       <= 32'b0;
            o_wb_sel         <= 4'b0;
            o_wb_we          <= 1'b0;
            o_wb_cyc         <= 1'b0;
            o_wb_stb         <= 1'b0;
            o_mmio_ready     <= 1'b0;
            o_mmio_read_data <= 64'b0;
        end else begin
            // Default: ready is a 1-cycle pulse, fired in S_DONE.
            o_mmio_ready <= 1'b0;

            case (state)
                // ----------------------------------------------------------------
                S_IDLE: begin
                    o_wb_cyc <= 1'b0;
                    o_wb_stb <= 1'b0;
                    if (is_write || is_read) begin
                        if (low_active) begin
                            // Low-half cycle first (always, when low is active).
                            o_wb_adr   <= eth_word_lo;
                            o_wb_dat_w <= i_mmio_write_data[31:0];
                            o_wb_sel   <= i_mmio_byte_en[3:0];
                            o_wb_we    <= is_write;
                            o_wb_cyc   <= 1'b1;
                            o_wb_stb   <= 1'b1;
                            state      <= S_LO;
                        end else if (high_active) begin
                            // Only high half active — skip straight to high cycle.
                            o_wb_adr   <= eth_word_hi;
                            o_wb_dat_w <= i_mmio_write_data[63:32];
                            o_wb_sel   <= i_mmio_byte_en[7:4];
                            o_wb_we    <= is_write;
                            o_wb_cyc   <= 1'b1;
                            o_wb_stb   <= 1'b1;
                            state      <= S_HI;
                        end else begin
                            // Degenerate access (no byte enables).  Just ack.
                            o_mmio_read_data <= 64'b0;
                            state            <= S_DONE;
                        end
                    end
                end

                // ----------------------------------------------------------------
                S_LO: begin
                    if (i_wb_ack) begin
                        o_mmio_read_data[31:0] <= i_wb_dat_r;
                        if (high_active) begin
                            // Drop STB for one cycle, hold CYC, then issue HI.
                            o_wb_stb <= 1'b0;
                            state    <= S_LO_GAP;
                        end else begin
                            o_wb_cyc <= 1'b0;
                            o_wb_stb <= 1'b0;
                            state    <= S_DONE;
                        end
                    end
                end

                // ----------------------------------------------------------------
                S_LO_GAP: begin
                    // One-cycle STB-low gap.  Set up high-half params and re-assert.
                    o_wb_adr   <= eth_word_hi;
                    o_wb_dat_w <= i_mmio_write_data[63:32];
                    o_wb_sel   <= i_mmio_byte_en[7:4];
                    // o_wb_we stays the same as the write/read direction
                    o_wb_stb   <= 1'b1;
                    state      <= S_HI;
                end

                // ----------------------------------------------------------------
                S_HI: begin
                    if (i_wb_ack) begin
                        o_mmio_read_data[63:32] <= i_wb_dat_r;
                        o_wb_cyc <= 1'b0;
                        o_wb_stb <= 1'b0;
                        state    <= S_DONE;
                    end
                end

                // ----------------------------------------------------------------
                S_DONE: begin
                    // One-cycle ack to the CPU side; rdata is already valid.
                    o_mmio_ready <= 1'b1;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

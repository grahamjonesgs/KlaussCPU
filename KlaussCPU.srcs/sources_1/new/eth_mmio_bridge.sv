`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// eth_mmio_bridge — bridges the CPU's 64-bit MMIO bus to LiteEth's 32-bit
// classic Wishbone slave.
//
// Address translation (byte-for-byte across the 192 KiB Eth window, except
// the slot-window remap and the bridge-owned 0xF007 window described below):
//   eth_byte_addr  = mmio.addr - 32'hF006_0000
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
// CPU holds mmio.addr / mmio.write_data / mmio.byte_en stable until
// mmio.ready, so we don't have to latch the request — the WB outputs read
// directly from the inputs each cycle.
//
// RX slots (HW_PERF_PLAN.md §3).  LiteEth is generated with 8 RX slots and
// 1 TX slot, which it places at WB bytes 0x20000 + k*0x800 (RX k) and
// 0x24000 (TX0).  The slot window is remapped so the 2-slot addresses do
// not move (offsets from 0xF008_0000; only address bits [15:12] change):
//     CPU 0x0000-0x0FFF  RX0, RX1          -> LiteEth 0x0000-0x0FFF
//     CPU 0x1000-0x17FF  TX0               -> LiteEth 0x4000-0x47FF
//     CPU 0x1800-0x1FFF  unused (was TX1)  -> LiteEth 0x4800-0x4FFF (unmapped)
//     CPU 0x2000-0x4FFF  RX2 .. RX7        -> LiteEth 0x1000-0x3FFF
//     CPU 0x5000-0xFFFF  unmapped          -> unchanged (unmapped)
// i.e. RX k is at 0xF008_0000 + (k < 2 ? k*0x800 : 0x1000 + k*0x800).
//
// The 0xF007_xxxx window (no LiteEth slave there) is owned by this bridge:
//   0xF007_0000  ETH_RX_SLOTS  [3:0] RX slots the MAC cycles through, RW,
//                reset 2 (CPU reset and program load); a write (byte 0
//                enabled) of 0/1 gives 2 and of more than 8 gives 8.  Program
//                load and every write also flush the MAC's RX slot state.
//                [11:8] = 8 read-only (the maximum).  Other bits read 0.
//   anything else in 0xF007_xxxx reads 0, writes are dropped.
//////////////////////////////////////////////////////////////////////////////////

module eth_mmio_bridge (
    input             i_clk,
    input             i_rst,             // active-high

    // -------- CPU MMIO side (slave; full 32-bit addr via mmio.addr) --------
    mmio_if.slave     mmio,

    // -------- LiteEth Wishbone classic master --------
    output logic [29:0] o_wb_adr,
    output logic [31:0] o_wb_dat_w,
    output logic [ 3:0] o_wb_sel,
    output logic        o_wb_we,
    output logic        o_wb_cyc,
    output logic        o_wb_stb,
    output     [ 1:0] o_wb_bte,
    output     [ 2:0] o_wb_cti,
    input      [31:0] i_wb_dat_r,
    input             i_wb_ack,
    input             i_wb_err,          // error termination — handled like ack

    // -------- ETH_RX_SLOTS --------
    input             i_load_rst,        // program load (LOAD_COMPLETE): back to 2 slots
    output     [ 3:0] o_rx_nslots,       // -> liteeth_core rx_nslots
    output            o_rx_slot_reset    // -> liteeth_core rx_slot_reset
);

    // ---- Wishbone single-transfer (B4 classic) ----
    assign o_wb_bte = 2'b00;
    assign o_wb_cti = 3'b000;

    // ---- Address derivation (combinational from inputs) ----
    //
    // The CPU may send mmio.addr either:
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
    wire [31:0] eth_byte_aligned = {mmio.addr[31:3], 3'b000};
    wire [31:0] eth_byte_raw     = eth_byte_aligned - 32'hF006_0000;
    // Slot-window remap (see header): bits [15:12] of the 0xF008 window.
    wire        in_slot_win      = (mmio.addr[27:16] == 12'h008);
    logic [3:0] slot_nib;
    always_comb begin
        slot_nib = eth_byte_raw[15:12];
        if (in_slot_win)
            case (eth_byte_raw[15:12])
                4'h1:    slot_nib = 4'h4;   // TX0 (+ the unused TX1 half)
                4'h2:    slot_nib = 4'h1;   // RX2, RX3
                4'h3:    slot_nib = 4'h2;   // RX4, RX5
                4'h4:    slot_nib = 4'h3;   // RX6, RX7
                default: ;
            endcase
    end
    wire [31:0] eth_byte_offset  = {eth_byte_raw[31:16], slot_nib, eth_byte_raw[11:0]};
    wire [29:0] eth_word_lo      = eth_byte_offset[31:2];        // bit 0 = 0
    wire [29:0] eth_word_hi      = eth_word_lo | 30'd1;          // bit 0 = 1 (+4 bytes)
    wire        low_active       = |mmio.byte_en[3:0];
    wire        high_active      = |mmio.byte_en[7:4];
    wire        is_write         = mmio.write_DV;
    wire        is_read          = mmio.read_DV;

    // ---- ETH_RX_SLOTS (bridge-owned 0xF007_xxxx window) ----
    wire        in_gap_win       = (mmio.addr[27:16] == 12'h007);
    wire        is_rx_slots_reg  = in_gap_win && (mmio.addr[15:3] == 13'd0);
    wire [31:0] rx_slots_wval    = mmio.write_data[31:0] & {{8{mmio.byte_en[3]}}, {8{mmio.byte_en[2]}},
                                                            {8{mmio.byte_en[1]}}, {8{mmio.byte_en[0]}}};
    logic [3:0] r_rx_nslots;
    logic       r_rx_slots_wr;       // 1-cycle pulse after an ETH_RX_SLOTS write
    assign o_rx_nslots     = r_rx_nslots;
    // Flush the MAC's RX slot state (slot counter -> 0, pending frames
    // dropped) on program load and after every ETH_RX_SLOTS write.  The
    // write flush is what makes a slot-count change safe: LiteEth's
    // CTRL_RESET soc_rst bit is not wired to anything in the standalone core.
    assign o_rx_slot_reset = i_load_rst | r_rx_slots_wr;

    // ---- FSM ----
    localparam S_IDLE   = 3'd0;
    localparam S_LO     = 3'd1;          // driving low-half WB cycle
    localparam S_LO_GAP = 3'd2;          // STB low between LO and HI (only if both halves)
    localparam S_HI     = 3'd3;          // driving high-half WB cycle
    localparam S_DONE   = 3'd4;          // pulse mmio.ready, then cool down
    localparam S_COOL   = 3'd5;          // hold (not accepting) until strobe drops

    logic [2:0] state;

    // A Wishbone slave may terminate a cycle with ERR instead of ACK, and an
    // address nothing decodes may never terminate at all. Either way the CPU
    // is spinning on mmio.ready — treat ERR as a completed cycle (reads
    // return 32'hFFFF_FFFF) and back it with a watchdog so no access can
    // wedge the whole SoC. 256 cycles is orders of magnitude beyond any real
    // LiteEth CSR/buffer response.
    logic [8:0] r_wb_tmo;
    wire        w_wb_end  = i_wb_ack || i_wb_err || r_wb_tmo[8];
    wire [31:0] w_wb_rdat = (i_wb_err || r_wb_tmo[8]) ? 32'hFFFF_FFFF : i_wb_dat_r;

    always_ff @(posedge i_clk) begin
        if (i_rst) begin
            state            <= S_IDLE;
            r_wb_tmo         <= 9'd0;
            o_wb_adr         <= 30'b0;
            o_wb_dat_w       <= 32'b0;
            o_wb_sel         <= 4'b0;
            o_wb_we          <= 1'b0;
            o_wb_cyc         <= 1'b0;
            o_wb_stb         <= 1'b0;
            mmio.ready     <= 1'b0;
            mmio.read_data <= 64'b0;
            r_rx_nslots    <= 4'd2;
            r_rx_slots_wr  <= 1'b0;
        end else begin
            // Default: ready is a 1-cycle pulse, fired in S_DONE.
            mmio.ready <= 1'b0;
            r_rx_slots_wr <= 1'b0;

            case (state)
                // ----------------------------------------------------------------
                S_IDLE: begin
                    o_wb_cyc <= 1'b0;
                    o_wb_stb <= 1'b0;
                    if ((is_write || is_read) && in_gap_win) begin
                        // Bridge-owned window: no Wishbone cycle.
                        mmio.read_data <= is_rx_slots_reg ? {52'b0, 4'd8, 4'b0, r_rx_nslots} : 64'b0;
                        if (is_write && is_rx_slots_reg && mmio.byte_en[0]) begin
                            r_rx_nslots   <= (rx_slots_wval < 32'd2) ? 4'd2 :
                                             (rx_slots_wval > 32'd8) ? 4'd8 : rx_slots_wval[3:0];
                            r_rx_slots_wr <= 1'b1;
                        end
                        state <= S_DONE;
                    end else if (is_write || is_read) begin
                        if (low_active) begin
                            // Low-half cycle first (always, when low is active).
                            r_wb_tmo   <= 9'd0;
                            o_wb_adr   <= eth_word_lo;
                            o_wb_dat_w <= mmio.write_data[31:0];
                            o_wb_sel   <= mmio.byte_en[3:0];
                            o_wb_we    <= is_write;
                            o_wb_cyc   <= 1'b1;
                            o_wb_stb   <= 1'b1;
                            state      <= S_LO;
                        end else if (high_active) begin
                            // Only high half active — skip straight to high cycle.
                            r_wb_tmo   <= 9'd0;
                            o_wb_adr   <= eth_word_hi;
                            o_wb_dat_w <= mmio.write_data[63:32];
                            o_wb_sel   <= mmio.byte_en[7:4];
                            o_wb_we    <= is_write;
                            o_wb_cyc   <= 1'b1;
                            o_wb_stb   <= 1'b1;
                            state      <= S_HI;
                        end else begin
                            // Degenerate access (no byte enables).  Just ack.
                            mmio.read_data <= 64'b0;
                            state            <= S_DONE;
                        end
                    end
                end

                // ----------------------------------------------------------------
                S_LO: begin
                    r_wb_tmo <= r_wb_tmo + 9'd1;
                    if (w_wb_end) begin
                        mmio.read_data[31:0] <= w_wb_rdat;
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
                    r_wb_tmo   <= 9'd0;
                    o_wb_adr   <= eth_word_hi;
                    o_wb_dat_w <= mmio.write_data[63:32];
                    o_wb_sel   <= mmio.byte_en[7:4];
                    // o_wb_we stays the same as the write/read direction
                    o_wb_stb   <= 1'b1;
                    state      <= S_HI;
                end

                // ----------------------------------------------------------------
                S_HI: begin
                    r_wb_tmo <= r_wb_tmo + 9'd1;
                    if (w_wb_end) begin
                        mmio.read_data[63:32] <= w_wb_rdat;
                        o_wb_cyc <= 1'b0;
                        o_wb_stb <= 1'b0;
                        state    <= S_DONE;
                    end
                end

                // ----------------------------------------------------------------
                S_DONE: begin
                    // One-cycle ack to the CPU side; rdata is already valid.
                    mmio.ready <= 1'b1;
                    state        <= S_COOL;
                end

                // ----------------------------------------------------------------
                // S_COOL: hold WITHOUT accepting a new request until the CPU has
                // dropped its strobe.  bus_splitter registers mmio.ready, so the
                // CPU sees the S_DONE ack one cycle late and keeps i_mmio_*_DV
                // asserted for an extra cycle.  Returning straight to S_IDLE would
                // re-fire the entire Wishbone transaction again off that lingering
                // strobe (the same re-latch hazard mem_read_write.v guards with its
                // COOL_DOWN state).  That phantom duplicate eth access, left in
                // flight after the store "completed", raced the faster post-"fetch
                // improvements" opcode fetch and latched 0 into r_opcode_mem — see
                // fetch_repro T4 / ssh_shell MDIO bring-up crash (ERR=01, OPC=0).
                S_COOL: begin
                    o_wb_cyc <= 1'b0;
                    o_wb_stb <= 1'b0;
                    if (!is_write && !is_read)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // Program load: back to the 2-slot behaviour (the core's
            // sram_writer is flushed by o_rx_slot_reset in the same cycle).
            if (i_load_rst)
                r_rx_nslots <= 4'd2;
        end
    end

endmodule

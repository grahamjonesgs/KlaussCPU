`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 2-way set-associative write-back cache — all arrays in BRAM, zero LUTRAM.
//
// Flow: WAIT detects DV, issues all BRAM reads simultaneously, latches the
// request, and advances to CHECK. CHECK uses the registered BRAM outputs to
// determine hit/miss and branches to the appropriate path. This costs +1 cycle
// on every access vs a LUTRAM tag design, but the hit path is still only 3
// cycles and misses are DDR-dominated (~20-50 cycles).
//
// 64-bit data bus: 128-bit cache line holds 2 × 64-bit doublewords.
// Doubleword offset within line: i_mem_addr[3] (0=upper [127:64], 1=lower [63:0])
// Cache index: addr[3+INDEX_BITS:4], Tag: addr[31:4+INDEX_BITS]
//
// External interface: i_mem_write_DV, i_mem_read_DV, i_mem_addr[31:0],
// i_mem_write_data[63:0], i_mem_byte_en[7:0], o_mem_read_data[63:0],
// o_mem_ready.
//////////////////////////////////////////////////////////////////////////////////

module mem_read_write (
    input         i_Clk,
    inout  [15:0] ddr2_dq,
    inout  [ 1:0] ddr2_dqs_n,
    inout  [ 1:0] ddr2_dqs_p,
    output [12:0] ddr2_addr,
    output [ 2:0] ddr2_ba,
    output        ddr2_ras_n,
    output        ddr2_cas_n,
    output        ddr2_we_n,
    output [ 0:0] ddr2_ck_p,
    output [ 0:0] ddr2_ck_n,
    output [ 0:0] ddr2_cke,
    output [ 0:0] ddr2_cs_n,
    output [ 1:0] ddr2_dm,
    output [ 0:0] ddr2_odt,

    input             i_mem_write_DV,
    input             i_mem_read_DV,
    input      [31:0] i_mem_addr,       // byte address (doubleword-aligned for 64-bit ops)
    input      [63:0] i_mem_write_data,
    input      [ 7:0] i_mem_byte_en,    // byte enables for writes (8'hFF = full doubleword)
    output reg [63:0] o_mem_read_data,
    output reg [63:0] o_mem_read_data_next, // next consecutive doubleword in same cache line
    output reg        o_mem_next_valid,      // 1 when o_mem_read_data_next is valid (offset == 0)
    output reg        o_mem_ready
);

    parameter  CACHE_SIZE = 2_048;              // number of sets — 2 ways × 2048 sets = 4096 total lines = 64 KB
    localparam INDEX_BITS = $clog2(CACHE_SIZE); // 11
    // 32-bit byte address, bottom 4 bits = byte offset within 16-byte line
    // addr[31:4] is the line address; tag = upper bits, index = lower INDEX_BITS
    localparam TAG_BITS   = 28 - INDEX_BITS;    // 17 (addr[31:4] = 28 bits, lower INDEX_BITS used for index)

    // -------------------------------------------------------------------------
    // DDR2 interface
    // -------------------------------------------------------------------------
    wire        sys_clk_i;
    reg  [ 9:0] por_counter = 32;
    wire        resetn = (por_counter == 0);

    reg          o_ddr_mem_write_DV;
    reg          o_ddr_mem_read_DV;
    reg  [ 31:0] o_ddr_mem_addr;
    reg  [127:0] o_ddr_mem_write_data;
    wire [127:0] i_ddr_mem_read_data;
    wire         i_ddr_mem_ready;
    wire [ 15:0] w_app_wdf_mask;
    reg  [ 15:0] r_app_wdf_mask;

    assign w_app_wdf_mask = r_app_wdf_mask;

    // -------------------------------------------------------------------------
    // Address decode — combinational from i_mem_addr (32-bit byte address).
    // CPU holds i_mem_addr stable until o_mem_ready, so these are stable
    // throughout any multi-cycle operation.
    //
    // DDR2 MIG address is in 16-bit half-word units (BL8 = 128-bit burst).
    // byte_addr → DDR half-word addr = byte_addr >> 1
    // Burst-aligned (3 DDR addr bits = 16 bytes = 8×2B): clear bottom 3 DDR bits
    //   → clear bottom 4 byte bits: {byte_addr[31:4], 4'b0000}
    // Doubleword offset within 128-bit cache line: byte_addr[3]  (2 doublewords per line)
    //   0 = upper doubleword [127:64], 1 = lower doubleword [63:0]
    // Cache index: addr[4+INDEX_BITS-1:4], Tag: addr[31:4+INDEX_BITS]
    // -------------------------------------------------------------------------
    wire [31:0]           w_computed_ddr_addr = {i_mem_addr[31:4], 4'b0000};
    wire [INDEX_BITS-1:0] w_cache_index       = i_mem_addr[4+INDEX_BITS-1:4];
    wire [TAG_BITS-1:0]   w_cache_tag         = i_mem_addr[31:4+INDEX_BITS];
    wire                  w_byte_offset       = i_mem_addr[3];    // doubleword within cache line

    // -------------------------------------------------------------------------
    // Cache arrays — ALL in BRAM (dirty bits in distributed RAM — narrow 1-bit
    // arrays fit efficiently in LUTRAM, and the separate always-block write
    // pattern is required for correct DRAM inference).
    //
    // Write address during normal operation: r_cache_index (latched in WAIT).
    // -------------------------------------------------------------------------

    // Tag + valid bit (bit [TAG_BITS] = valid, bits [TAG_BITS-1:0] = tag)
    (* ram_style = "block" *)
    reg [TAG_BITS:0] cache_val_addr_way0 [CACHE_SIZE-1:0];

    (* ram_style = "block" *)
    reg [TAG_BITS:0] cache_val_addr_way1 [CACHE_SIZE-1:0];

    // 128-bit cache line data
    (* ram_style = "block" *)
    reg [127:0] cache_val_data_way0 [CACHE_SIZE-1:0];

    (* ram_style = "block" *)
    reg [127:0] cache_val_data_way1 [CACHE_SIZE-1:0];

    // Dirty bits — 1-bit wide, distributed RAM. Written via dedicated always
    // blocks below (separate from the FSM) so Vivado can infer DRAM cleanly.
    (* ram_style = "distributed" *)
    reg cache_dirty_way0 [CACHE_SIZE-1:0];

    (* ram_style = "distributed" *)
    reg cache_dirty_way1 [CACHE_SIZE-1:0];

    // LRU bit: 0 = way1 most-recently-used (evict way0)
    //          1 = way0 most-recently-used (evict way1)
    (* ram_style = "block" *)
    reg cache_lru [CACHE_SIZE-1:0];

    // -------------------------------------------------------------------------
    // Pipeline registers — filled in WAIT from BRAM reads, used in CHECK
    // -------------------------------------------------------------------------
    reg [TAG_BITS:0]     r_tag_way0;
    reg [TAG_BITS:0]     r_tag_way1;
    reg                  r_dirty_way0;
    reg                  r_dirty_way1;
    reg                  r_lru;
    reg [127:0]          r_data_way0;
    reg [127:0]          r_data_way1;

    // Latched request (stable while CPU waits, but latching makes CDC clear)
    reg [INDEX_BITS-1:0] r_cache_index;
    reg [TAG_BITS-1:0]   r_cache_tag;
    reg                  r_byte_offset;   // which doubleword in 128-bit cache line (0=upper, 1=lower)
    reg [7:0]            r_byte_en;       // byte enables (8'hFF = full doubleword write)
    reg [63:0]           r_write_data;
    reg                  r_is_write;
    reg [31:0]           r_computed_ddr_addr;

    // -------------------------------------------------------------------------
    // Hit / evict decode — combinational from the registered BRAM outputs.
    // Only meaningful after WAIT has issued reads (i.e. in CHECK and beyond).
    // Cache is always enabled (switch no longer controls this).
    // -------------------------------------------------------------------------
    wire r_hit_way0      = r_tag_way0[TAG_BITS] &&
                           (r_tag_way0[TAG_BITS-1:0] == r_cache_tag);
    wire r_hit_way1      = r_tag_way1[TAG_BITS] &&
                           (r_tag_way1[TAG_BITS-1:0] == r_cache_tag);
    wire r_cache_hit     = r_hit_way0 || r_hit_way1;
    wire r_evict_way_sel = r_lru;   // 0 = evict way0, 1 = evict way1
    wire r_evict_dirty   = r_evict_way_sel ? r_dirty_way1 : r_dirty_way0;
    wire [TAG_BITS-1:0] r_evict_tag = r_evict_way_sel
                                      ? r_tag_way1[TAG_BITS-1:0]
                                      : r_tag_way0[TAG_BITS-1:0];

    // -------------------------------------------------------------------------
    // Miss-path pipeline registers
    // -------------------------------------------------------------------------
    reg [127:0] r_cache_val_data_hold; // cache line for hit merge / presentation
    reg [127:0] r_evict_data_hold;     // dirty line being written back to DDR
    reg [31:0]  r_evict_ddr_addr_r;   // DDR address of the dirty eviction
    reg [31:0]  r_fetch_ddr_addr;     // DDR address for the refill fetch
    reg         r_hit_way;            // which way matched (write-hit path)
    reg         r_evict_way;          // which way to replace (miss paths)

    // -------------------------------------------------------------------------
    // State machine — one-hot 16-bit
    // -------------------------------------------------------------------------
    localparam PRE_WAIT              = 16'd1;
    localparam WAIT                  = 16'd2;     // idle: wait for DV, issue BRAM reads
    localparam CHECK                 = 16'd4;     // check registered BRAM results
    localparam WRITE_HIT             = 16'd8;     // merge word into line, set dirty, done
    localparam WRITE_MISS_EVICT      = 16'd16;    // dirty write-miss: start writeback
    localparam WRITE_EVICT_DONE      = 16'd32;    // wait for writeback, then fetch
    localparam WRITE_EVICT_GAP       = 16'd64;    // CDC gap before read DV
    localparam WRITE_FETCH           = 16'd128;   // wait for fetch, merge, store (write-back: line installed dirty)
    localparam READ_CACHE2           = 16'd256;   // read hit: present word from latched line
    localparam READ_EVICT            = 16'd512;   // dirty read-miss: start writeback
    localparam READ_EVICT_DONE       = 16'd1024;  // wait for writeback, then fetch
    localparam READ_EVICT_GAP        = 16'd2048;  // CDC gap before read DV
    localparam READ_WAIT             = 16'd4096;  // wait for DDR fetch

    reg [15:0] state = WAIT;

    // -------------------------------------------------------------------------
    // Power-on reset counter
    // -------------------------------------------------------------------------
    always @(posedge i_Clk) begin
        if (por_counter > 0)
            por_counter <= por_counter - 1;
    end

    // -------------------------------------------------------------------------
    // Initialise metadata arrays to all-invalid at simulation time 0.
    // Vivado compiles this into BRAM/DRAM init vectors.
    // Data arrays need no init (never read without a valid tag match).
    // -------------------------------------------------------------------------
    integer init_i;
    initial begin
        for (init_i = 0; init_i < CACHE_SIZE; init_i = init_i + 1) begin
            cache_val_addr_way0[init_i] = 0;
            cache_val_addr_way1[init_i] = 0;
            cache_dirty_way0[init_i]    = 0;
            cache_dirty_way1[init_i]    = 0;
            cache_lru[init_i]           = 0;
        end
    end

    // -------------------------------------------------------------------------
    // Dirty bit write controls — combinatorial, derived from FSM state.
    // Separate always blocks give Vivado a single clear write port per array,
    // allowing correct distributed-RAM inference and eliminating the large
    // mux trees that result from register-based implementation.
    //
    // dirty_din = 1 on write-hit or write-fetch (line becomes dirty), 0 on read refill (line clean).
    // -------------------------------------------------------------------------
    wire dirty0_wen = (state == WRITE_HIT   &&                        r_hit_way == 1'b0) ||
                      (state == WRITE_FETCH  && i_ddr_mem_ready && r_evict_way == 1'b0) ||
                      (state == READ_WAIT    && i_ddr_mem_ready && r_evict_way == 1'b0);
    // WRITE_FETCH installs as DIRTY (write-back on miss): avoids DDR write-through
    // after a miss, which would race against MIG's internal write pipeline and
    // could clobber bytes committed by a prior buffered write to the same line.
    // The dirty line will be written back to DDR by the normal WRITE_MISS_EVICT path.
    wire dirty0_din = (state == WRITE_HIT) || (state == WRITE_FETCH && i_ddr_mem_ready);

    wire dirty1_wen = (state == WRITE_HIT   &&                        r_hit_way == 1'b1) ||
                      (state == WRITE_FETCH  && i_ddr_mem_ready && r_evict_way == 1'b1) ||
                      (state == READ_WAIT    && i_ddr_mem_ready && r_evict_way == 1'b1);
    wire dirty1_din = (state == WRITE_HIT) || (state == WRITE_FETCH && i_ddr_mem_ready);

    always @(posedge i_Clk) begin
        if (dirty0_wen) cache_dirty_way0[r_cache_index] <= dirty0_din;
    end

    always @(posedge i_Clk) begin
        if (dirty1_wen) cache_dirty_way1[r_cache_index] <= dirty1_din;
    end

    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always @(posedge i_Clk) begin : fsm

        reg [127:0] merged; // procedural variable for cache line merge

        case (state)

            // ------------------------------------------------------------------
            PRE_WAIT: begin
                state <= WAIT;
            end

            // ------------------------------------------------------------------
            // WAIT: idle state. When the CPU asserts a DV, issue all BRAM reads
            // for tags, dirty bits, LRU, and both data ways simultaneously.
            // All results are available next cycle in CHECK.
            // ------------------------------------------------------------------
            WAIT: begin
                o_mem_ready      <= 0;
                o_mem_next_valid <= 0;
                if (i_mem_write_DV || i_mem_read_DV) begin
                    // Issue all BRAM reads in parallel
                    r_tag_way0          <= cache_val_addr_way0[w_cache_index];
                    r_tag_way1          <= cache_val_addr_way1[w_cache_index];
                    r_dirty_way0        <= cache_dirty_way0[w_cache_index];
                    r_dirty_way1        <= cache_dirty_way1[w_cache_index];
                    r_lru               <= cache_lru[w_cache_index];
                    r_data_way0         <= cache_val_data_way0[w_cache_index];
                    r_data_way1         <= cache_val_data_way1[w_cache_index];
                    // Latch request
                    r_cache_index       <= w_cache_index;
                    r_cache_tag         <= w_cache_tag;
                    r_byte_offset       <= w_byte_offset;
                    r_byte_en           <= i_mem_byte_en;
                    r_write_data        <= i_mem_write_data;
                    r_is_write          <= i_mem_write_DV;
                    r_computed_ddr_addr <= w_computed_ddr_addr;
                    state               <= CHECK;
                end
            end

            // ------------------------------------------------------------------
            // CHECK: BRAM results now in r_tag/dirty/lru/data registers.
            // Decode hit/miss, select eviction way, branch to correct path.
            // ------------------------------------------------------------------
            CHECK: begin
                if (r_is_write) begin

                    if (r_cache_hit) begin
                        // Write hit — data already latched, proceed to merge
                        r_hit_way             <= r_hit_way1 ? 1'b1 : 1'b0;
                        r_cache_val_data_hold <= r_hit_way0 ? r_data_way0 : r_data_way1;
                        state                 <= WRITE_HIT;

                    end else begin
                        // Write miss
                        r_evict_way        <= r_evict_way_sel;
                        r_evict_ddr_addr_r <= {r_evict_tag, r_cache_index, 4'b0000};
                        r_fetch_ddr_addr   <= r_computed_ddr_addr;
                        r_evict_data_hold  <= r_evict_way_sel ? r_data_way1 : r_data_way0;
                        if (r_evict_dirty) begin
                            state <= WRITE_MISS_EVICT;
                        end else begin
                            o_ddr_mem_addr    <= r_computed_ddr_addr;
                            o_ddr_mem_read_DV <= 1;
                            state             <= WRITE_FETCH;
                        end
                    end

                end else begin // read

                    if (r_cache_hit) begin
                        // Read hit — select line, update LRU
                        r_cache_val_data_hold <= r_hit_way0 ? r_data_way0 : r_data_way1;
                        cache_lru[r_cache_index] <= r_hit_way0 ? 1'b1 : 1'b0;
                        state <= READ_CACHE2;

                    end else begin
                        // Read miss
                        r_evict_way        <= r_evict_way_sel;
                        r_evict_ddr_addr_r <= {r_evict_tag, r_cache_index, 4'b0000};
                        r_fetch_ddr_addr   <= r_computed_ddr_addr;
                        r_evict_data_hold  <= r_evict_way_sel ? r_data_way1 : r_data_way0;
                        if (r_evict_dirty) begin
                            state <= READ_EVICT;
                        end else begin
                            o_ddr_mem_addr    <= r_computed_ddr_addr;
                            o_ddr_mem_read_DV <= 1;
                            state             <= READ_WAIT;
                        end
                    end

                end
            end // CHECK

            // ------------------------------------------------------------------
            // WRITE HIT: r_cache_val_data_hold contains the current line.
            // Byte-merge write data into old word, then word-merge into line.
            // Dirty bit is set via the combinatorial dirty0/1_wen wires above.
            // ------------------------------------------------------------------
            WRITE_HIT: begin
                begin : write_hit_merge
                    reg [63:0] old_dw;
                    reg [63:0] new_dw;
                    // Extract old doubleword at the target offset
                    // r_byte_offset 0 = upper [127:64], 1 = lower [63:0]
                    old_dw = r_byte_offset ? r_cache_val_data_hold[63:0]
                                           : r_cache_val_data_hold[127:64];
                    // Apply byte enables: r_byte_en[0]=LSByte bits[7:0], r_byte_en[7]=MSByte bits[63:56]
                    new_dw[63:56] = r_byte_en[7] ? r_write_data[63:56] : old_dw[63:56];
                    new_dw[55:48] = r_byte_en[6] ? r_write_data[55:48] : old_dw[55:48];
                    new_dw[47:40] = r_byte_en[5] ? r_write_data[47:40] : old_dw[47:40];
                    new_dw[39:32] = r_byte_en[4] ? r_write_data[39:32] : old_dw[39:32];
                    new_dw[31:24] = r_byte_en[3] ? r_write_data[31:24] : old_dw[31:24];
                    new_dw[23:16] = r_byte_en[2] ? r_write_data[23:16] : old_dw[23:16];
                    new_dw[15:8]  = r_byte_en[1] ? r_write_data[15:8]  : old_dw[15:8];
                    new_dw[7:0]   = r_byte_en[0] ? r_write_data[7:0]   : old_dw[7:0];
                    // Merge new doubleword into cache line
                    if (r_byte_offset)
                        merged = {r_cache_val_data_hold[127:64], new_dw};
                    else
                        merged = {new_dw, r_cache_val_data_hold[63:0]};
                end

                // Separate write-enable per way — one write per array per cycle, BRAM-friendly.
                if (r_hit_way == 1'b0)
                    cache_val_data_way0[r_cache_index] <= merged;
                if (r_hit_way == 1'b1)
                    cache_val_data_way1[r_cache_index] <= merged;
                cache_lru[r_cache_index] <= (r_hit_way == 1'b0) ? 1'b1 : 1'b0;

                o_mem_ready <= 1;
                state       <= PRE_WAIT;
            end

            // ------------------------------------------------------------------
            // WRITE MISS — dirty eviction path
            // ------------------------------------------------------------------
            WRITE_MISS_EVICT: begin
                o_ddr_mem_addr       <= r_evict_ddr_addr_r;
                o_ddr_mem_write_data <= r_evict_data_hold;
                r_app_wdf_mask       <= 16'b0000_0000_0000_0000; // all bytes valid
                o_ddr_mem_write_DV   <= 1;
                state                <= WRITE_EVICT_DONE;
            end

            WRITE_EVICT_DONE: begin
                if (i_ddr_mem_ready) begin
                    o_ddr_mem_write_DV <= 0;
                    o_ddr_mem_addr     <= r_fetch_ddr_addr;
                    state              <= WRITE_EVICT_GAP;
                end
            end

            WRITE_EVICT_GAP: begin
                // One-cycle gap: ddr2_control must see write DV deasserted
                // before read DV rises (CDC safety across ui_clk boundary).
                o_ddr_mem_read_DV <= 1;
                state             <= WRITE_FETCH;
            end

            WRITE_FETCH: begin
                if (i_ddr_mem_ready) begin
                    o_ddr_mem_read_DV <= 0;

                    begin : write_fetch_merge
                        reg [63:0] old_dw;
                        reg [63:0] new_dw;
                        // r_byte_offset 0 = upper [127:64], 1 = lower [63:0]
                        old_dw = r_byte_offset ? i_ddr_mem_read_data[63:0]
                                               : i_ddr_mem_read_data[127:64];
                        new_dw[63:56] = r_byte_en[7] ? r_write_data[63:56] : old_dw[63:56];
                        new_dw[55:48] = r_byte_en[6] ? r_write_data[55:48] : old_dw[55:48];
                        new_dw[47:40] = r_byte_en[5] ? r_write_data[47:40] : old_dw[47:40];
                        new_dw[39:32] = r_byte_en[4] ? r_write_data[39:32] : old_dw[39:32];
                        new_dw[31:24] = r_byte_en[3] ? r_write_data[31:24] : old_dw[31:24];
                        new_dw[23:16] = r_byte_en[2] ? r_write_data[23:16] : old_dw[23:16];
                        new_dw[15:8]  = r_byte_en[1] ? r_write_data[15:8]  : old_dw[15:8];
                        new_dw[7:0]   = r_byte_en[0] ? r_write_data[7:0]   : old_dw[7:0];
                        if (r_byte_offset)
                            merged = {i_ddr_mem_read_data[127:64], new_dw};
                        else
                            merged = {new_dw, i_ddr_mem_read_data[63:0]};
                    end

                    // Install merged line in cache as DIRTY (write-back policy on miss).
                    // Dirty bit set via dirty0/1_din combinatorial wires above.
                    if (r_evict_way == 1'b0) begin
                        cache_val_data_way0[r_cache_index] <= merged;
                        cache_val_addr_way0[r_cache_index] <= {1'b1, r_cache_tag};
                    end
                    if (r_evict_way == 1'b1) begin
                        cache_val_data_way1[r_cache_index] <= merged;
                        cache_val_addr_way1[r_cache_index] <= {1'b1, r_cache_tag};
                    end
                    cache_lru[r_cache_index] <= (r_evict_way == 1'b0) ? 1'b1 : 1'b0;

                    // Line installed as DIRTY — will be written back to DDR on eviction.
                    // Dirty bit set via dirty0/1_din combinatorial wires above.
                    o_mem_ready <= 1;
                    state       <= PRE_WAIT;
                end
            end

            // ------------------------------------------------------------------
            // READ HIT: present the correct doubleword from the latched line.
            // r_byte_offset 0 = upper [127:64], 1 = lower [63:0]
            // o_mem_read_data_next is valid only when offset == 0 (upper dw,
            // next is lower dw in same line); offset == 1 has no next in line.
            // ------------------------------------------------------------------
            READ_CACHE2: begin
                if (r_byte_offset == 1'b0) begin
                    o_mem_read_data      <= r_cache_val_data_hold[127:64];
                    o_mem_read_data_next <= r_cache_val_data_hold[63:0];
                    o_mem_next_valid     <= 1'b1;
                end else begin
                    o_mem_read_data      <= r_cache_val_data_hold[63:0];
                    o_mem_read_data_next <= 64'h0;
                    o_mem_next_valid     <= 1'b0;
                end
                o_mem_ready <= 1;
                state       <= PRE_WAIT;
            end

            // ------------------------------------------------------------------
            // READ MISS — dirty eviction path
            // ------------------------------------------------------------------
            READ_EVICT: begin
                o_ddr_mem_addr       <= r_evict_ddr_addr_r;
                o_ddr_mem_write_data <= r_evict_data_hold;
                r_app_wdf_mask       <= 16'b0000_0000_0000_0000;
                o_ddr_mem_write_DV   <= 1;
                state                <= READ_EVICT_DONE;
            end

            READ_EVICT_DONE: begin
                if (i_ddr_mem_ready) begin
                    o_ddr_mem_write_DV <= 0;
                    o_ddr_mem_addr     <= r_fetch_ddr_addr;
                    state              <= READ_EVICT_GAP;
                end
            end

            READ_EVICT_GAP: begin
                o_ddr_mem_read_DV <= 1;
                state             <= READ_WAIT;
            end

            // ------------------------------------------------------------------
            // READ WAIT: DDR fetch complete — install line, return word
            // ------------------------------------------------------------------
            READ_WAIT: begin
                if (i_ddr_mem_ready) begin
                    o_ddr_mem_read_DV <= 0;

                    // Dirty bit cleared via dirty0/1_wen combinatorial wires above.
                    if (r_evict_way == 1'b0) begin
                        cache_val_data_way0[r_cache_index] <= i_ddr_mem_read_data;
                        cache_val_addr_way0[r_cache_index] <= {1'b1, r_cache_tag};
                    end
                    if (r_evict_way == 1'b1) begin
                        cache_val_data_way1[r_cache_index] <= i_ddr_mem_read_data;
                        cache_val_addr_way1[r_cache_index] <= {1'b1, r_cache_tag};
                    end
                    cache_lru[r_cache_index] <= (r_evict_way == 1'b0) ? 1'b1 : 1'b0;

                    // r_byte_offset 0 = upper [127:64], 1 = lower [63:0]
                    if (r_byte_offset == 1'b0) begin
                        o_mem_read_data      <= i_ddr_mem_read_data[127:64];
                        o_mem_read_data_next <= i_ddr_mem_read_data[63:0];
                        o_mem_next_valid     <= 1'b1;
                    end else begin
                        o_mem_read_data      <= i_ddr_mem_read_data[63:0];
                        o_mem_read_data_next <= 64'h0;
                        o_mem_next_valid     <= 1'b0;
                    end

                    o_mem_ready <= 1;
                    state       <= PRE_WAIT;
                end
            end

            default: state <= WAIT;

        endcase
    end // fsm

    // -------------------------------------------------------------------------
    // Clock wizard and DDR2 controller
    // -------------------------------------------------------------------------
    clk_wiz_0 clk_wiz_0 (
        .i_Clk  (i_Clk),
        .clk_200(sys_clk_i),
        .resetn (resetn)
    );

    ddr2_control ddr2_control (
        .ddr2_dq          (ddr2_dq),
        .ddr2_dqs_n       (ddr2_dqs_n),
        .ddr2_dqs_p       (ddr2_dqs_p),
        .ddr2_addr        (ddr2_addr),
        .ddr2_ba          (ddr2_ba),
        .ddr2_ras_n       (ddr2_ras_n),
        .ddr2_cas_n       (ddr2_cas_n),
        .ddr2_we_n        (ddr2_we_n),
        .ddr2_ck_p        (ddr2_ck_p),
        .ddr2_ck_n        (ddr2_ck_n),
        .ddr2_cke         (ddr2_cke),
        .ddr2_cs_n        (ddr2_cs_n),
        .ddr2_dm          (ddr2_dm),
        .ddr2_odt         (ddr2_odt),
        .resetn           (resetn),
        .sys_clk_i        (sys_clk_i),
        .i_mem_write_DV   (o_ddr_mem_write_DV),
        .i_mem_read_DV    (o_ddr_mem_read_DV),
        .i_mem_addr       (o_ddr_mem_addr),
        .i_mem_write_data (o_ddr_mem_write_data),
        .i_app_wdf_mask   (w_app_wdf_mask),
        .o_mem_read_data  (i_ddr_mem_read_data),
        .o_mem_ready      (i_ddr_mem_ready)
    );

endmodule

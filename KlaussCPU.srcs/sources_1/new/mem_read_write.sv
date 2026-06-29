`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 2-way set-associative write-back cache — all arrays in BRAM, zero LUTRAM.
//
// Flow: WAIT detects DV, issues all BRAM reads simultaneously, latches the
// request, and advances to CHECK. CHECK uses the registered BRAM outputs to
// determine hit/miss; HITS (read and write) complete inside CHECK itself —
// way select, byte merge, and offset select all run from values registered in
// WAIT. The BRAM tag design costs +1 cycle on every access vs LUTRAM tags,
// but the hit path is only 2 cycles and misses are DDR-dominated
// (~20-50 cycles).
//
// 64-bit data bus: 128-bit cache line holds 2 × 64-bit doublewords.
// Doubleword offset within line: cpu.addr[3] (0=upper [127:64], 1=lower [63:0])
// Cache index: addr[3+INDEX_BITS:4], Tag: addr[31:4+INDEX_BITS]
//
// External interface: cpu.write_DV, cpu.read_DV, cpu.addr[31:0],
// cpu.write_data[63:0], cpu.byte_en[7:0], cpu.read_data[63:0],
// cpu.ready.
//////////////////////////////////////////////////////////////////////////////////

module mem_read_write (
    input         i_Clk_board,   // 100 MHz board oscillator — feeds clk_wiz (MIG 200MHz ref)
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

    // CPU-side memory bus (slave): request in, read_data/next/ready out.
    membus_if.slave   cpu,

    // Performance counters (RISC-V Zihpm-style cache events) and control.
    // i_stat_clear: 1-cycle pulse — zeros all counters on the next edge.
    // o_cache_info: read-only geometry register, see MMIO_MAP.md.
    input             i_stat_clear,
    output     [63:0] o_cache_info,
    output logic [63:0] o_cnt_read_hits,
    output logic [63:0] o_cnt_read_misses,
    output logic [63:0] o_cnt_write_hits,
    output logic [63:0] o_cnt_write_misses,
    output logic [63:0] o_cnt_writebacks,
    output logic [63:0] o_cnt_stall_cycles,

    // 50 MHz clock for Ethernet RMII REF_CLK — produced by clk_wiz_0
    // alongside the existing 200 MHz DDR reference clock.  Plumbed up to
    // the top level so KlaussCPU.v can drive both LiteEth's sys-clock
    // input and the PHY's ETH_REFCLK pin (via ODDR).
    output            clk_50,

    // MIG ui_clk (100 MHz at 2:1) exposed to the top so the WHOLE CPU runs on it,
    // synchronous with the cache↔MIG interface (no async CDC needed).
    output            o_ui_clk,

    // DDR2 calibration complete — passed up so the boot-ROM copy (KlaussCPU.v)
    // waits for DDR before writing the resident netboot image into it.
    output            o_calib_done,

    // -------------------------------------------------------------------------
    // DMA master port (master B) — second DDR2 requestor, used by the blitter.
    // 128-bit, burst-aligned, same protocol the cache↔DDR path uses internally.
    // The on-chip arbiter (below) grants the shared ddr2_control to either the
    // cache (master A, priority) or this port. Contract for the blitter:
    //   * Raise i_dma_req to ask for the bus (level; may stay high across ops).
    //   * Wait for o_dma_grant, then drive ONE transaction:
    //       - write: i_dma_write_DV + i_dma_addr/i_dma_write_data/i_dma_wdf_mask,
    //                then hold address stable through the CDC settle (see the
    //                WRITE_EVICT_DONE comment) before releasing.
    //       - read : i_dma_read_DV + i_dma_addr; latch o_dma_read_data when
    //                o_dma_ready pulses.
    //   * Pulse i_dma_done for one cycle to release the bus (re-arbitration
    //     point — keeps a long blit from starving the CPU).
    // -------------------------------------------------------------------------
    input             i_dma_req,
    input             i_dma_done,
    input             i_dma_write_DV,
    input             i_dma_read_DV,
    input      [31:0] i_dma_addr,
    input      [127:0] i_dma_write_data,
    input      [15:0] i_dma_wdf_mask,
    output     [127:0] o_dma_read_data,
    output            o_dma_ready,
    output            o_dma_grant,

    // -------------------------------------------------------------------------
    // Cache-maintenance control (MMIO 0xF005). Full-cache operations that walk
    // every set/way. Pulses (1 cycle) trigger; o_mnt_busy is high while a walk
    // runs. The CPU's next cached access transparently stalls until the walk
    // completes (the FSM only services maintenance from the idle WAIT state),
    // so software may treat a flush/invalidate as synchronous; o_mnt_busy is
    // also exposed for explicit polling.
    //   FLUSH      : write back every dirty line, mark it clean (valid kept).
    //   INVALIDATE : flush dirty lines, then clear valid on every line.
    // Used for DMA/blitter coherency: flush src+dst before a blit, invalidate
    // dst after.
    // -------------------------------------------------------------------------
    input             i_flush_go,
    input             i_inval_go,
    output            o_mnt_busy
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
    // The cache FSM (and the whole CPU above) runs on the MIG ui_clk so it is
    // SYNCHRONOUS with the cache↔MIG interface. `i_Clk` is kept as the name every
    // `always_ff @(posedge i_Clk)` already uses — it's now an internal net driven by
    // the MIG ui_clk (w_ui_clk), not the board oscillator (i_Clk_board feeds the
    // MIG's 200 MHz reference via clk_wiz_0).
    wire        w_ui_clk;
    wire        i_Clk = w_ui_clk;
    assign      o_ui_clk = w_ui_clk;
    logic  [ 9:0] por_counter = 32;
    wire        resetn = (por_counter == 0);

    // Power-up values are explicit so simulation matches hardware (FPGA regs
    // init to 0); without these the DDR-side DVs are X until first assigned.
    logic          o_ddr_mem_write_DV = 1'b0;
    logic          o_ddr_mem_read_DV  = 1'b0;
    logic  [ 31:0] o_ddr_mem_addr;
    logic  [127:0] o_ddr_mem_write_data;
    wire [127:0] i_ddr_mem_read_data;
    wire         i_ddr_mem_ready;
    wire [ 15:0] w_app_wdf_mask;
    logic  [ 15:0] r_app_wdf_mask;

    assign w_app_wdf_mask = r_app_wdf_mask;

    // -------------------------------------------------------------------------
    // DMA arbiter — two masters share the single ddr2_control instance:
    //   master A = cache FSM (the o_ddr_mem_* registers below), priority
    //   master B = blitter DMA port (i_dma_*)
    // r_grant_blit selects which master drives the controller. The grant is
    // registered (no combinational glitches on app_addr) and only switches to
    // the blitter when the cache is idle, then back when the blitter pulses
    // i_dma_done — see the grant always-block near the FSM below.
    // -------------------------------------------------------------------------
    logic          r_grant_blit = 1'b0;     // 0 = cache owns DDR, 1 = blitter owns

    wire         mig_write_DV   = r_grant_blit ? i_dma_write_DV   : o_ddr_mem_write_DV;
    wire         mig_read_DV    = r_grant_blit ? i_dma_read_DV    : o_ddr_mem_read_DV;
    wire [ 31:0] mig_addr       = r_grant_blit ? i_dma_addr       : o_ddr_mem_addr;
    wire [127:0] mig_write_data = r_grant_blit ? i_dma_write_data : o_ddr_mem_write_data;
    wire [ 15:0] mig_wdf_mask   = r_grant_blit ? i_dma_wdf_mask   : w_app_wdf_mask;

    assign o_dma_read_data = i_ddr_mem_read_data;   // shared; blitter latches on o_dma_ready
    assign o_dma_grant     = r_grant_blit;

    // -------------------------------------------------------------------------
    // Cache-maintenance request capture. A flush/invalidate pulse sets a sticky
    // pending flag (+ mode); the main FSM consumes it from WAIT, raising
    // r_mnt_active for the duration of the walk. New pulses while a walk is
    // already active are ignored (the walk is idempotent anyway).
    //   r_mnt_mode: 0 = FLUSH (write back dirty), 1 = INVALIDATE (flush + clear valid)
    // -------------------------------------------------------------------------
    logic r_mnt_pending = 1'b0;
    logic r_mnt_mode    = 1'b0;
    logic r_mnt_active  = 1'b0;     // set by the main FSM while a walk runs

    assign o_mnt_busy = r_mnt_active | r_mnt_pending;

    always_ff @(posedge i_Clk) begin
        if (r_mnt_active) begin
            r_mnt_pending <= 1'b0;             // being serviced — clear request
        end else if (i_flush_go || i_inval_go) begin
            r_mnt_pending <= 1'b1;
            r_mnt_mode    <= i_inval_go;       // INVALIDATE wins if both asserted
        end
    end

    // -------------------------------------------------------------------------
    // ddr2_control's ready (i_ddr_mem_ready) is in the SAME ui_clk domain as the
    // cache FSM (the whole CPU runs synchronous to the 100 MHz MIG ui_clk), so
    // there is no clock crossing here and no 2-FF ready synchroniser — ready is
    // sampled directly.
    // -------------------------------------------------------------------------
    // ddr2_control's ready belongs to whichever master is currently granted.
    // Gate it so the cache only acts on its own DDR completions and the blitter
    // only on its own — neither can mistake the other's ready for its own.
    wire w_cache_ddr_ready = i_ddr_mem_ready & ~r_grant_blit;
    wire w_blit_ddr_ready  = i_ddr_mem_ready &  r_grant_blit;

    assign o_dma_ready = w_blit_ddr_ready;

    // -------------------------------------------------------------------------
    // Address decode — combinational from cpu.addr (32-bit byte address).
    // CPU holds cpu.addr stable until cpu.ready, so these are stable
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
    wire [31:0]           w_computed_ddr_addr = {cpu.addr[31:4], 4'b0000};
    wire [INDEX_BITS-1:0] w_cache_index       = cpu.addr[4+INDEX_BITS-1:4];
    wire [TAG_BITS-1:0]   w_cache_tag         = cpu.addr[31:4+INDEX_BITS];
    wire                  w_byte_offset       = cpu.addr[3];    // doubleword within cache line

    // -------------------------------------------------------------------------
    // Cache arrays — ALL in BRAM (dirty bits in distributed RAM — narrow 1-bit
    // arrays fit efficiently in LUTRAM, and the separate always-block write
    // pattern is required for correct DRAM inference).
    //
    // Write address during normal operation: r_cache_index (latched in WAIT).
    // -------------------------------------------------------------------------

    // Tag + valid bit (bit [TAG_BITS] = valid, bits [TAG_BITS-1:0] = tag)
    (* ram_style = "block" *)
    logic [TAG_BITS:0] cache_val_addr_way0 [CACHE_SIZE-1:0];

    (* ram_style = "block" *)
    logic [TAG_BITS:0] cache_val_addr_way1 [CACHE_SIZE-1:0];

    // 128-bit cache line data
    (* ram_style = "block" *)
    logic [127:0] cache_val_data_way0 [CACHE_SIZE-1:0];

    (* ram_style = "block" *)
    logic [127:0] cache_val_data_way1 [CACHE_SIZE-1:0];

    // Dirty bits — 1-bit wide, distributed RAM. Written via dedicated always
    // blocks below (separate from the FSM) so Vivado can infer DRAM cleanly.
    (* ram_style = "distributed" *)
    logic cache_dirty_way0 [CACHE_SIZE-1:0];

    (* ram_style = "distributed" *)
    logic cache_dirty_way1 [CACHE_SIZE-1:0];

    // LRU bit: 0 = way1 most-recently-used (evict way0)
    //          1 = way0 most-recently-used (evict way1)
    (* ram_style = "block" *)
    logic cache_lru [CACHE_SIZE-1:0];

    // -------------------------------------------------------------------------
    // Pipeline registers — filled in WAIT from BRAM reads, used in CHECK
    // -------------------------------------------------------------------------
    logic [TAG_BITS:0]     r_tag_way0;
    logic [TAG_BITS:0]     r_tag_way1;
    logic                  r_dirty_way0;
    logic                  r_dirty_way1;
    logic                  r_lru;
    logic [127:0]          r_data_way0;
    logic [127:0]          r_data_way1;

    // Latched request (stable while CPU waits, but latching makes CDC clear)
    logic [INDEX_BITS-1:0] r_cache_index;
    logic [TAG_BITS-1:0]   r_cache_tag;
    logic                  r_byte_offset;   // which doubleword in 128-bit cache line (0=upper, 1=lower)
    logic [7:0]            r_byte_en;       // byte enables (8'hFF = full doubleword write)
    logic [63:0]           r_write_data;
    logic                  r_is_write;
    logic [31:0]           r_computed_ddr_addr;

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
    logic [127:0] r_evict_data_hold;     // dirty line being written back to DDR
    logic [31:0]  r_evict_ddr_addr_r;   // DDR address of the dirty eviction
    logic [31:0]  r_fetch_ddr_addr;     // DDR address for the refill fetch
    logic         r_evict_way;          // which way to replace (miss paths)
    logic [3:0]   r_gap_count;          // gap between writeback and refill. Single
                                       // domain, so it only needs to let
                                       // ddr2_control go IDLE->WAIT (both DVs low)
                                       // before the refill DV. (WRITE/READ_EVICT_GAP)

    // -------------------------------------------------------------------------
    // State machine — one-hot 16-bit
    // -------------------------------------------------------------------------
    localparam PRE_WAIT              = 16'd1;
    localparam WAIT                  = 16'd2;     // idle: wait for DV, issue BRAM reads
    localparam CHECK                 = 16'd4;     // check registered BRAM results; hits complete here
    // 16'd8 (WRITE_HIT) and 16'd256 (READ_CACHE2) retired — both hit paths
    // now complete inside CHECK, saving one cycle per hit.
    localparam WRITE_MISS_EVICT      = 16'd16;    // dirty write-miss: start writeback
    localparam WRITE_EVICT_DONE      = 16'd32;    // wait for writeback, then fetch
    localparam WRITE_EVICT_GAP       = 16'd64;    // CDC gap before read DV
    localparam WRITE_FETCH           = 16'd128;   // wait for fetch, merge, store (write-back: line installed dirty)
    localparam READ_EVICT            = 16'd512;   // dirty read-miss: start writeback
    localparam READ_EVICT_DONE       = 16'd1024;  // wait for writeback, then fetch
    localparam READ_EVICT_GAP        = 16'd2048;  // CDC gap before read DV
    localparam READ_WAIT             = 16'd4096;  // wait for DDR fetch
    localparam COOL_DOWN             = 16'd8192;  // dead cycle after PRE_WAIT — covers the
                                                  // 1-cycle return delay added by bus_splitter's
                                                  // registered output stage. Without it, WAIT
                                                  // would re-enter while CPU's DV is still high
                                                  // (CPU sees splitter.ready 1 cycle after cache
                                                  // asserts it, and only drops DV at the next edge),
                                                  // causing a spurious re-latch of the same request.
                                                  // The phantom ready pulses produced by those
                                                  // re-latches were observed to drive opcode
                                                  // fetches off stale cpu.read_data — see
                                                  // CRASH_DUMP.md ERR=01 trace at PC=0x78.
    localparam MAINT                 = 16'd16384; // cache flush/invalidate walk (sub-FSM below)

    logic [15:0] state = WAIT;

    // -------------------------------------------------------------------------
    // Maintenance sub-FSM (inside the MAINT state). Walks r_cache_index 0..N-1,
    // writing back dirty lines (both ways) and, for INVALIDATE, clearing valid.
    // Reuses the r_tag/r_dirty/r_data pipeline regs (idle during maintenance),
    // r_cache_index as the walk counter, and r_gap_count for the writeback CDC
    // settle — the CPU FSM is idle (in WAIT) whenever maintenance runs.
    // -------------------------------------------------------------------------
    localparam MS_READ    = 4'd0;   // issue BRAM reads for the current set
    localparam MS_W0      = 4'd1;   // way0: writeback if valid&dirty
    localparam MS_W0_WAIT = 4'd2;
    localparam MS_W0_GAP  = 4'd3;
    localparam MS_W1      = 4'd4;   // way1: writeback if valid&dirty
    localparam MS_W1_WAIT = 4'd5;
    localparam MS_W1_GAP  = 4'd6;
    localparam MS_CLR     = 4'd7;   // clear dirty (both modes) / valid (INVALIDATE)
    localparam MS_DONE    = 4'd8;
    logic [3:0] r_mnt_sub = MS_READ;

    // Unified cache-array READ index. The CPU (WAIT) and the maintenance walk
    // (MAINT/MS_READ) read the tag/dirty/data/lru arrays through THIS single
    // address, so every array has exactly one read port. Without this, the
    // maintenance walk's read at r_cache_index is a second, independent read
    // address — and a LUT-as-distributed-RAM cell has only one async read port,
    // so Vivado would replicate the (distributed) dirty arrays and spill the
    // extra read port of the wider arrays into LUT-as-DRAM (DRC UTLZ-1
    // over-utilization). WAIT and MAINT are mutually exclusive states, so the
    // mux is free of conflict.
    wire [INDEX_BITS-1:0] w_rd_index = (state == MAINT) ? r_cache_index
                                                        : w_cache_index;

    // -------------------------------------------------------------------------
    // Power-on reset counter — MUST run on the always-on board oscillator, NOT
    // i_Clk (= ui_clk). resetn (= por_counter==0) resets clk_wiz AND the MIG,
    // which PRODUCE ui_clk; clocking this on ui_clk deadlocks at power-on
    // (ui_clk can't start until resetn releases, resetn can't release without
    // ui_clk). The board clock is free-running, so it breaks the loop.
    // -------------------------------------------------------------------------
    always_ff @(posedge i_Clk_board) begin
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
    // Write hits complete inside CHECK (merge + BRAM write in the decode
    // cycle), so the dirty-set condition keys off the combinational hit
    // decode rather than a latched r_hit_way.
    wire w_write_hit_now = (state == CHECK) && r_is_write && r_cache_hit;

    wire dirty0_wen = (w_write_hit_now &&                       r_hit_way0) ||
                      (state == WRITE_FETCH  && w_cache_ddr_ready && r_evict_way == 1'b0) ||
                      (state == READ_WAIT    && w_cache_ddr_ready && r_evict_way == 1'b0) ||
                      (state == MAINT && r_mnt_sub == MS_CLR);  // flush/inval: clear dirty
    // WRITE_FETCH installs as DIRTY (write-back on miss): avoids DDR write-through
    // after a miss, which would race against MIG's internal write pipeline and
    // could clobber bytes committed by a prior buffered write to the same line.
    // The dirty line will be written back to DDR by the normal WRITE_MISS_EVICT path.
    wire dirty0_din = w_write_hit_now || (state == WRITE_FETCH && w_cache_ddr_ready);

    wire dirty1_wen = (w_write_hit_now &&                       r_hit_way1) ||
                      (state == WRITE_FETCH  && w_cache_ddr_ready && r_evict_way == 1'b1) ||
                      (state == READ_WAIT    && w_cache_ddr_ready && r_evict_way == 1'b1) ||
                      (state == MAINT && r_mnt_sub == MS_CLR);  // flush/inval: clear dirty
    wire dirty1_din = w_write_hit_now || (state == WRITE_FETCH && w_cache_ddr_ready);

    always_ff @(posedge i_Clk) begin
        if (dirty0_wen) cache_dirty_way0[r_cache_index] <= dirty0_din;
    end

    always_ff @(posedge i_Clk) begin
        if (dirty1_wen) cache_dirty_way1[r_cache_index] <= dirty1_din;
    end

    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always_ff @(posedge i_Clk) begin : fsm

        logic [127:0] merged; // procedural variable for cache line merge

        case (state)

            // ------------------------------------------------------------------
            // PRE_WAIT runs the cycle after a transaction completes (each
            // completion path sets cpu.ready=1 and state<=PRE_WAIT).  The
            // ready pulse must be visible to the CPU for exactly one cycle:
            // longer than that and a CPU FSM that issues a back-to-back
            // request (e.g. OPCODE_REQUEST → OPCODE_FETCH on the timer-
            // interrupt push→fetch path) will see the *previous* transaction's
            // stale ready in its new state and latch garbage from
            // cpu.read_data.  Clearing ready here — not in WAIT — gives a
            // clean low edge before the next request can be picked up.
            //
            // COOL_DOWN is inserted after PRE_WAIT so that WAIT re-entry is
            // delayed one extra cycle.  Required because bus_splitter now
            // registers cpu.ready: the CPU sees ready one cycle after this
            // module asserts it, and so it drops r_mem_*_DV one cycle later
            // than the legacy combinational-splitter path expected.  Without
            // the dead cycle, WAIT runs while the CPU's DV is still high and
            // re-latches the same request — see CRASH_DUMP.md notes.
            PRE_WAIT: begin
                cpu.ready      <= 0;
                cpu.next_valid <= 0;
                state            <= COOL_DOWN;
            end

            // ------------------------------------------------------------------
            // COOL_DOWN: one-cycle dead state.  Does NOT inspect i_mem_*_DV, so
            // any lingering CPU DV (still high while the CPU finishes observing
            // the registered ready pulse from the splitter) is harmlessly
            // ignored.  Always advances to WAIT.
            // ------------------------------------------------------------------
            COOL_DOWN: begin
                state <= WAIT;
            end

            // ------------------------------------------------------------------
            // WAIT: idle state. When the CPU asserts a DV, issue all BRAM reads
            // for tags, dirty bits, LRU, and both data ways simultaneously.
            // All results are available next cycle in CHECK.
            // ------------------------------------------------------------------
            WAIT: begin
                cpu.ready      <= 0;
                cpu.next_valid <= 0;
                if (r_mnt_pending) begin
                    // Cache maintenance has priority over CPU requests: a pending
                    // flush/invalidate is consumed here, while the CPU stalls on
                    // its (un-served) request until the walk completes.
                    r_mnt_active  <= 1'b1;
                    r_cache_index <= {INDEX_BITS{1'b0}};
                    r_mnt_sub     <= MS_READ;
                    state         <= MAINT;
                end else if (cpu.write_DV || cpu.read_DV) begin
                    // Issue all BRAM reads in parallel (single read port: w_rd_index).
                    r_tag_way0          <= cache_val_addr_way0[w_rd_index];
                    r_tag_way1          <= cache_val_addr_way1[w_rd_index];
                    r_dirty_way0        <= cache_dirty_way0[w_rd_index];
                    r_dirty_way1        <= cache_dirty_way1[w_rd_index];
                    r_lru               <= cache_lru[w_rd_index];
                    r_data_way0         <= cache_val_data_way0[w_rd_index];
                    r_data_way1         <= cache_val_data_way1[w_rd_index];
                    // Latch request
                    r_cache_index       <= w_cache_index;
                    r_cache_tag         <= w_cache_tag;
                    r_byte_offset       <= w_byte_offset;
                    r_byte_en           <= cpu.byte_en;
                    r_write_data        <= cpu.write_data;
                    r_is_write          <= cpu.write_DV;
                    r_computed_ddr_addr <= w_computed_ddr_addr;
                    state               <= CHECK;
                end
            end

            // ------------------------------------------------------------------
            // CHECK: BRAM results now in r_tag/dirty/lru/data registers.
            // Decode hit/miss, select eviction way, branch to correct path.
            //
            // Both HIT paths complete here in one cycle (formerly the separate
            // WRITE_HIT / READ_CACHE2 states): every input they need — tags,
            // both data ways, byte enables, write data, offset — was registered
            // in WAIT, so the way-select mux, byte merge, and offset mux fit
            // comfortably register-to-register inside this module. Saves one
            // cycle on every cache hit. Miss paths are unchanged.
            // ------------------------------------------------------------------
            CHECK: begin
                if (r_is_write) begin

                    if (r_cache_hit) begin
                        // Write hit — byte-merge into the hit way's line and
                        // write the BRAM now.  Dirty bit set via the
                        // combinatorial dirty0/1_wen wires (w_write_hit_now).
                        begin : write_hit_merge
                            logic [127:0] hitline;
                            logic [63:0]  old_dw;
                            logic [63:0]  new_dw;
                            hitline = r_hit_way0 ? r_data_way0 : r_data_way1;
                            // Extract old doubleword at the target offset
                            // r_byte_offset 0 = upper [127:64], 1 = lower [63:0]
                            old_dw = r_byte_offset ? hitline[63:0]
                                                   : hitline[127:64];
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
                                merged = {hitline[127:64], new_dw};
                            else
                                merged = {new_dw, hitline[63:0]};
                        end

                        // Separate write-enable per way — one write per array per cycle, BRAM-friendly.
                        if (r_hit_way0)
                            cache_val_data_way0[r_cache_index] <= merged;
                        if (r_hit_way1)
                            cache_val_data_way1[r_cache_index] <= merged;
                        cache_lru[r_cache_index] <= r_hit_way0 ? 1'b1 : 1'b0;

                        cpu.ready <= 1;
                        state       <= PRE_WAIT;

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
                        // Read hit — present the requested doubleword now.
                        // r_byte_offset 0 = upper [127:64], 1 = lower [63:0];
                        // the "next" doubleword is only valid at offset 0
                        // (companion is the lower dw of the same line).
                        begin : read_hit_present
                            logic [127:0] hitline;
                            hitline = r_hit_way0 ? r_data_way0 : r_data_way1;
                            if (r_byte_offset == 1'b0) begin
                                cpu.read_data      <= hitline[127:64];
                                cpu.read_data_next <= hitline[63:0];
                                cpu.next_valid     <= 1'b1;
                            end else begin
                                cpu.read_data      <= hitline[63:0];
                                cpu.read_data_next <= 64'h0;
                                cpu.next_valid     <= 1'b0;
                            end
                        end
                        cache_lru[r_cache_index] <= r_hit_way0 ? 1'b1 : 1'b0;

                        cpu.ready <= 1;
                        state       <= PRE_WAIT;

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
                if (w_cache_ddr_ready) begin
                    o_ddr_mem_write_DV <= 0;
                    // CRITICAL: do NOT change o_ddr_mem_addr here. ddr2_control
                    // samples cpu.write_DV directly (same ui_clk domain), but
                    // its FSM only returns WRITE_DONE->IDLE->WAIT once BOTH DVs
                    // read low (the IDLE gate there). For a cycle or two after we
                    // drop write_DV it can still be draining its write, and if it
                    // re-enters WAIT->WRITE while write_DV still reads high it
                    // latches app_addr from the live cpu.addr. If we have already
                    // changed addr to the refill target, that spurious write
                    // commits the eviction line's data to the refill address —
                    // corrupting DRAM.  Hold the eviction address through the
                    // gap below so any spurious write is idempotent.
                    r_gap_count        <= 4'd2;
                    state              <= WRITE_EVICT_GAP;
                end
            end

            WRITE_EVICT_GAP: begin
                // Settle gap: hold the eviction address (and keep both DVs low)
                // long enough for ddr2_control to drain its write and return to
                // its DV=0-gated IDLE/WAIT.  Only after that is it safe to change
                // addr to the refill target and raise read DV.  r_gap_count=2 (a
                // few i_Clk cycles) covers it in the single ui_clk domain.
                if (r_gap_count == 4'd0) begin
                    o_ddr_mem_addr    <= r_fetch_ddr_addr;
                    o_ddr_mem_read_DV <= 1;
                    state             <= WRITE_FETCH;
                end else begin
                    r_gap_count <= r_gap_count - 4'd1;
                end
            end

            WRITE_FETCH: begin
                if (w_cache_ddr_ready) begin
                    o_ddr_mem_read_DV <= 0;

                    begin : write_fetch_merge
                        logic [63:0] old_dw;
                        logic [63:0] new_dw;
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
                    cpu.ready <= 1;
                    state       <= PRE_WAIT;
                end
            end

            // READ_CACHE2 / WRITE_HIT removed — both hit paths now complete
            // inside CHECK (see the CHECK comment above).

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
                if (w_cache_ddr_ready) begin
                    o_ddr_mem_write_DV <= 0;
                    // Hold eviction address through the settle gap — see the long
                    // comment in WRITE_EVICT_DONE for the race this prevents.
                    r_gap_count        <= 4'd2;
                    state              <= READ_EVICT_GAP;
                end
            end

            READ_EVICT_GAP: begin
                if (r_gap_count == 4'd0) begin
                    o_ddr_mem_addr    <= r_fetch_ddr_addr;
                    o_ddr_mem_read_DV <= 1;
                    state             <= READ_WAIT;
                end else begin
                    r_gap_count <= r_gap_count - 4'd1;
                end
            end

            // ------------------------------------------------------------------
            // READ WAIT: DDR fetch complete — install line, return word
            // ------------------------------------------------------------------
            READ_WAIT: begin
                if (w_cache_ddr_ready) begin
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
                        cpu.read_data      <= i_ddr_mem_read_data[127:64];
                        cpu.read_data_next <= i_ddr_mem_read_data[63:0];
                        cpu.next_valid     <= 1'b1;
                    end else begin
                        cpu.read_data      <= i_ddr_mem_read_data[63:0];
                        cpu.read_data_next <= 64'h0;
                        cpu.next_valid     <= 1'b0;
                    end

                    cpu.ready <= 1;
                    state       <= PRE_WAIT;
                end
            end

            // ------------------------------------------------------------------
            // MAINT: full-cache flush / invalidate walk. Sub-FSM over
            // r_cache_index (0 .. CACHE_SIZE-1), writing back dirty lines in
            // both ways and (for INVALIDATE) clearing valid. Writebacks reuse
            // the master-A DDR path; the arbiter holds the blitter off while
            // r_mnt_active (so writebacks are never interrupted mid-CDC-gap).
            // ------------------------------------------------------------------
            MAINT: begin
                case (r_mnt_sub)

                    MS_READ: begin
                        // Issue BRAM reads for the current set (both ways), via the
                        // SAME read port the CPU uses (w_rd_index = r_cache_index here).
                        r_tag_way0   <= cache_val_addr_way0[w_rd_index];
                        r_tag_way1   <= cache_val_addr_way1[w_rd_index];
                        r_dirty_way0 <= cache_dirty_way0[w_rd_index];
                        r_dirty_way1 <= cache_dirty_way1[w_rd_index];
                        r_data_way0  <= cache_val_data_way0[w_rd_index];
                        r_data_way1  <= cache_val_data_way1[w_rd_index];
                        r_mnt_sub    <= MS_W0;
                    end

                    MS_W0: begin
                        // r_tag_wayX[TAG_BITS] = valid bit.
                        if (r_tag_way0[TAG_BITS] && r_dirty_way0) begin
                            o_ddr_mem_addr       <= {r_tag_way0[TAG_BITS-1:0], r_cache_index, 4'b0000};
                            o_ddr_mem_write_data <= r_data_way0;
                            r_app_wdf_mask       <= 16'h0000;   // write all 16 bytes
                            o_ddr_mem_write_DV   <= 1'b1;
                            r_mnt_sub            <= MS_W0_WAIT;
                        end else begin
                            r_mnt_sub <= MS_W1;
                        end
                    end

                    MS_W0_WAIT: begin
                        if (w_cache_ddr_ready) begin
                            o_ddr_mem_write_DV <= 1'b0;
                            r_gap_count        <= 4'd2;   // CDC settle, hold addr
                            r_mnt_sub          <= MS_W0_GAP;
                        end
                    end

                    MS_W0_GAP: begin
                        if (r_gap_count == 4'd0) r_mnt_sub <= MS_W1;
                        else                     r_gap_count <= r_gap_count - 4'd1;
                    end

                    MS_W1: begin
                        if (r_tag_way1[TAG_BITS] && r_dirty_way1) begin
                            o_ddr_mem_addr       <= {r_tag_way1[TAG_BITS-1:0], r_cache_index, 4'b0000};
                            o_ddr_mem_write_data <= r_data_way1;
                            r_app_wdf_mask       <= 16'h0000;
                            o_ddr_mem_write_DV   <= 1'b1;
                            r_mnt_sub            <= MS_W1_WAIT;
                        end else begin
                            r_mnt_sub <= MS_CLR;
                        end
                    end

                    MS_W1_WAIT: begin
                        if (w_cache_ddr_ready) begin
                            o_ddr_mem_write_DV <= 1'b0;
                            r_gap_count        <= 4'd2;
                            r_mnt_sub          <= MS_W1_GAP;
                        end
                    end

                    MS_W1_GAP: begin
                        if (r_gap_count == 4'd0) r_mnt_sub <= MS_CLR;
                        else                     r_gap_count <= r_gap_count - 4'd1;
                    end

                    MS_CLR: begin
                        // Dirty bits for both ways are cleared via the dirty0/1_wen
                        // wires (which include the MS_CLR term). For INVALIDATE,
                        // also clear the valid bit on both ways.
                        if (r_mnt_mode) begin   // INVALIDATE
                            cache_val_addr_way0[r_cache_index] <= {(TAG_BITS+1){1'b0}};
                            cache_val_addr_way1[r_cache_index] <= {(TAG_BITS+1){1'b0}};
                        end
                        if (r_cache_index == CACHE_SIZE - 1) begin
                            r_mnt_sub <= MS_DONE;
                        end else begin
                            r_cache_index <= r_cache_index + 1'b1;
                            r_mnt_sub     <= MS_READ;
                        end
                    end

                    MS_DONE: begin
                        r_mnt_active <= 1'b0;
                        state        <= WAIT;
                    end

                    default: r_mnt_sub <= MS_DONE;
                endcase
            end

            default: state <= WAIT;

        endcase
    end // fsm

    // -------------------------------------------------------------------------
    // Performance counters — RISC-V Zihpm-style cache events.
    //
    // CHECK is the unique decision cycle: r_cache_hit, r_is_write, r_evict_dirty
    // are all valid here, the state runs for exactly one i_Clk cycle per access,
    // and exactly one transition out of CHECK fires per request, so each event
    // counter increments at most once per memory access.
    //
    // The "miss path" stall counter ticks every cycle the FSM is anywhere in
    // the writeback / DDR-fetch / CDC-gap chain — i.e., whenever the CPU is
    // waiting on us beyond a single-cycle hit.
    //
    // i_stat_clear pulses for one cycle and zeros every counter; CACHE_INFO is
    // a static geometry descriptor decoded by the MMIO read mux.
    // -------------------------------------------------------------------------
    // Sized intermediates so each field of the concat has the correct width.
    // CACHE_SIZE is an unsized integer parameter, so naked use inside {} would
    // pad/widen unpredictably — bind to explicit-width localparams first.
    localparam [31:0] LP_CACHE_TOTAL_BYTES = CACHE_SIZE * 2 * 16;  // ways × sets × line_bytes
    localparam [15:0] LP_CACHE_NUM_SETS    = CACHE_SIZE;
    localparam [ 7:0] LP_CACHE_LINE_BYTES  = 8'd16;
    localparam [ 7:0] LP_CACHE_NUM_WAYS    = 8'd2;
    localparam [63:0] LP_CACHE_INFO = {
        LP_CACHE_TOTAL_BYTES,   // [63:32]
        LP_CACHE_LINE_BYTES,    // [31:24]
        LP_CACHE_NUM_SETS,      // [23: 8]
        LP_CACHE_NUM_WAYS       // [ 7: 0]
    };
    assign o_cache_info = LP_CACHE_INFO;

    wire is_miss_path = (state == WRITE_MISS_EVICT) ||
                        (state == WRITE_EVICT_DONE) ||
                        (state == WRITE_EVICT_GAP)  ||
                        (state == WRITE_FETCH)      ||
                        (state == READ_EVICT)       ||
                        (state == READ_EVICT_DONE)  ||
                        (state == READ_EVICT_GAP)   ||
                        (state == READ_WAIT);

    // -------------------------------------------------------------------------
    // DMA grant arbitration (see the muxed mig_* wires and the DMA master port).
    // Cache is the priority master; the blitter is handed the bus only while the
    // cache is NOT mid-transaction (is_miss_path low), so an in-flight cache
    // miss — including its CDC writeback gaps — is never interrupted. The
    // blitter performs one 128-bit transaction per grant and pulses i_dma_done
    // to release, so even a full-frame blit yields the bus back to the CPU
    // between every transfer (bounded CPU stall, no starvation).
    //
    // The blitter is also held off while a cache-maintenance walk is active
    // (r_mnt_active): the walk's writebacks use the master-A path and must not
    // be interrupted mid-CDC-gap. In the intended coherency sequence the blit
    // and the flush/invalidate never overlap anyway.
    // -------------------------------------------------------------------------
    always_ff @(posedge i_Clk) begin
        if (!r_grant_blit) begin
            if (!is_miss_path && !r_mnt_active && i_dma_req)
                r_grant_blit <= 1'b1;
        end else begin
            if (i_dma_done)
                r_grant_blit <= 1'b0;
        end
    end

    initial begin
        o_cnt_read_hits    = 64'd0;
        o_cnt_read_misses  = 64'd0;
        o_cnt_write_hits   = 64'd0;
        o_cnt_write_misses = 64'd0;
        o_cnt_writebacks   = 64'd0;
        o_cnt_stall_cycles = 64'd0;
    end

    always_ff @(posedge i_Clk) begin
        if (i_stat_clear) begin
            o_cnt_read_hits    <= 64'd0;
            o_cnt_read_misses  <= 64'd0;
            o_cnt_write_hits   <= 64'd0;
            o_cnt_write_misses <= 64'd0;
            o_cnt_writebacks   <= 64'd0;
            o_cnt_stall_cycles <= 64'd0;
        end else begin
            if (state == CHECK) begin
                if (!r_is_write &&  r_cache_hit)              o_cnt_read_hits    <= o_cnt_read_hits    + 64'd1;
                if (!r_is_write && !r_cache_hit)              o_cnt_read_misses  <= o_cnt_read_misses  + 64'd1;
                if ( r_is_write &&  r_cache_hit)              o_cnt_write_hits   <= o_cnt_write_hits   + 64'd1;
                if ( r_is_write && !r_cache_hit)              o_cnt_write_misses <= o_cnt_write_misses + 64'd1;
                if (!r_cache_hit && r_evict_dirty)            o_cnt_writebacks   <= o_cnt_writebacks   + 64'd1;
            end
            if (is_miss_path)                                 o_cnt_stall_cycles <= o_cnt_stall_cycles + 64'd1;
        end
    end

    // -------------------------------------------------------------------------
    // Clock wizard and DDR2 controller
    // -------------------------------------------------------------------------
    clk_wiz_0 clk_wiz_0 (
        .i_Clk  (i_Clk_board),     // board oscillator (NOT i_Clk=ui_clk — that would be circular)
        .clk_200(sys_clk_i),
        .clk_50 (clk_50),
        .locked (),                // intentionally unused
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
        .i_mem_write_DV   (mig_write_DV),
        .i_mem_read_DV    (mig_read_DV),
        .i_mem_addr       (mig_addr),
        .i_mem_write_data (mig_write_data),
        .i_app_wdf_mask   (mig_wdf_mask),
        .o_mem_read_data  (i_ddr_mem_read_data),
        .o_mem_ready      (i_ddr_mem_ready),
        .o_calib_done     (o_calib_done),
        .o_ui_clk         (w_ui_clk)
    );

endmodule

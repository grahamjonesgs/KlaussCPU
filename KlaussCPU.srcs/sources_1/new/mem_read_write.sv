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

module mem_read_write #(
    // HW_PERF_PLAN.md §6: the unified L2 (l2_cache) in place of ddr2_control,
    // below the DDR arbiter.  0 = ddr2_control, the design before the L2.
    parameter bit L2_EN   = 1'b0,
    parameter int L2_WAYS = 4
) (
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
    output logic [63:0] o_cnt_zero_lines,   // DCBZ operations (HW_PERF_PLAN.md §1)

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
    //   * Wait for o_dma_grant, then drive one transaction at a time:
    //       - write: i_dma_write_DV + i_dma_addr/i_dma_write_data/i_dma_wdf_mask,
    //                then hold address stable through the settle gap (see the
    //                EVICT_SHADOW_WAIT comment) before the next transaction.
    //       - read : i_dma_read_DV + i_dma_addr; latch o_dma_read_data when
    //                o_dma_ready pulses.
    //   * The grant may be HELD across multiple transactions (burst tenure).
    //     Pulse i_dma_done to release the bus — the re-arbitration point. The
    //     blitter yields every CHUNK (8) transactions and at blit end, which
    //     bounds the CPU's added stall to one chunk; i_dma_req held high
    //     through a chunk release re-queues it for the next idle window.
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
    // DDR master C — AMP core 2 (same contract as the blitter master B above:
    // req level, grant, single 128 b burst DV/ready, done releases).  Lowest
    // priority: cache first, blitter second, core 2 last.  Core 2 releases
    // per-transaction, so core 1 never waits behind more than one 16 B burst
    // (AMP_CORE2_PLAN.md §5 non-intrusion).
    // -------------------------------------------------------------------------
    input             i_c2_req,
    input             i_c2_done,
    input             i_c2_write_DV,
    input             i_c2_read_DV,
    input      [31:0] i_c2_addr,
    input      [127:0] i_c2_write_data,
    input      [15:0] i_c2_wdf_mask,
    output     [127:0] o_c2_read_data,
    output            o_c2_ready,
    output            o_c2_grant,

    // -------------------------------------------------------------------------
    // DDR master D — VGA scanout (vga_scanout.sv, VGA_PLAN.md Phase 2).
    // READ-ONLY, and its reads are WIDE: one transaction = one 32 B line
    // (the cache's two pipelined BL8 bursts, requested dword 0), returned on
    // o_vga_read_data in the cache-line layout (dword at byte offset 8k in
    // bits [255-64k -: 64]).  Same req/grant/DV/ready/done contract as B/C;
    // priority among the DMA masters: VGA > blitter > core 2 (the display
    // has a hard deadline).  Unlike B/C it may also be granted at the
    // maintenance walk's between-sets point (MS_READ), so a long flush does
    // not starve the display.  i_vga_addr[4:0] must be 0.
    // -------------------------------------------------------------------------
    input             i_vga_req,
    input             i_vga_done,
    input             i_vga_read_DV,
    input      [31:0] i_vga_addr,
    output     [255:0] o_vga_read_data,
    output            o_vga_ready,
    output            o_vga_grant,
    output            o_vga_yield,      // a cache miss is waiting for DDR: VGA should end its tenure

    // -------------------------------------------------------------------------
    // DDR master E — the SD DMA engine (sd_native, SD DMA spec v1.2).  The
    // B / C contract: req level, grant, one 128 b transaction (DV until
    // ready, wdf_mask 1 = byte not written), done releases; it releases after
    // every transaction.  Priority: VGA > SD > blitter > core 2, gated like the
    // blitter (no cache miss in flight, no maintenance walk pending or
    // running: a data command written after a CACHE_CTRL op cannot reach DDR
    // before the walk's writebacks have).  A DMA master to the L2 (i_dma).
    // -------------------------------------------------------------------------
    input             i_sd_req,
    input             i_sd_done,
    input             i_sd_write_DV,
    input             i_sd_read_DV,
    input      [31:0] i_sd_addr,
    input      [127:0] i_sd_write_data,
    input      [15:0] i_sd_wdf_mask,
    output     [127:0] o_sd_read_data,
    output            o_sd_ready,
    output            o_sd_grant,

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
    output            o_mnt_busy,
    // D-cache range operations (CACHE_CTRL[5:4] RANGE_CLEAN / RANGE_INV over
    // CACHE_OP_ADDR / CACHE_OP_LEN, SD DMA spec v1.2): a one-cycle pulse with
    // the op's lines, copied when the op is accepted.  i_rng_n = ceil(LEN/32)
    // (>= 1: LEN = 0 is a no-op the SoC drops).  RANGE_INV writes a dirty
    // line back before it drops it.  Queued like FLUSH (see the capture).
    input             i_rng_go,
    input             i_rng_inv,
    input      [26:0] i_rng_l0,         // first line: PA[31:5]
    input      [27:0] i_rng_n,

    // 1a stage 3 (STAGE3_DCACHE_PLAN.md): the load-hit pipe.  i_h_addr is the
    // core's AGU result; its shadow tag/data arrays are read EVERY cycle at
    // that address (side-effect free).  o_h_ok / o_h_dword are combinational
    // from those registered reads, the cycle after: the doubleword at the
    // address presented one cycle earlier, valid only when o_h_ok.
    input      [31:0] i_h_addr,
    // (MMU_PLAN §11.9 timing) i_h_addr's page [31:16] for the TLB's two carry
    // candidates and the carry (i_h_addr's own page is the selected one):
    // both are registered and the carry picks after the tag read
    input      [15:0] i_h_pg0,
    input      [15:0] i_h_pg1,
    input             i_h_c1,
    output            o_h_ok,
    output     [63:0] o_h_dword,
    // (tmg timing) each way's dword at the offset, and the way select
    output     [63:0] o_h_dw0,
    output     [63:0] o_h_dw1,
    output            o_h_sel0,

    // HW_PERF_PLAN.md §5: the whole 32 B line of every CPU read this cache
    // answers, for the core's 32 B I-cache lines (one cycle, o_cpu_line_v):
    // with the read's ready on a hit (CHECK), and on a miss when the line
    // has arrived from DDR (READ_WAIT's install — after the early-restart
    // ready, or with the fallback ready).  Exactly one per read, never
    // before its ready and before any later request's; same layout as the
    // arrays (dword k at [255-64k -: 64]).  Writes give none.
    output logic [255:0] o_cpu_line = '0,
    output logic         o_cpu_line_v = 1'b0,

    // HW_PERF_PLAN.md §6 L2 (L2_EN): L2_CTRL (0xF005_0028) write strobe and
    // data ([0] EN, [1] WBINV), the CPU-reset / program-load enable, its
    // state, L2_INFO, and the perf event strobes (MMIO_MAP 0xF00D_0118..)
    input             i_l2_ctrl_wr,
    input      [ 1:0] i_l2_ctrl_wdata,
    input             i_l2_ctrl_en,
    output            o_l2_en,
    output            o_l2_busy,
    output     [63:0] o_l2_info,
    output     [10:0] o_l2_ev
);

    // DV-low settle before a shadow / walk writeback: ddr2_control's IDLE
    // needs two; the L2 accepts after one low cycle
    localparam logic [3:0] WB_GAP = L2_EN ? 4'd0 : 4'd2;

    parameter  CACHE_SIZE = 1_024;              // number of sets — 2 ways × 1024 sets = 2048 total lines × 32 B = 64 KB
    localparam INDEX_BITS = $clog2(CACHE_SIZE); // 10
    // 32 B (256-bit) lines: bottom 5 bits = byte offset within the line.
    // addr[31:5] is the line address (27 bits); tag = upper bits, index = lower INDEX_BITS.
    localparam TAG_BITS   = 27 - INDEX_BITS;    // 17 (addr[31:5] = 27 bits, lower INDEX_BITS used for index)

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
    logic  [255:0] o_ddr_mem_write_data;   // 32 B line (cache path); blitter uses low 128 b
    wire   [255:0] i_ddr_mem_read_data;    // 32 B line (cache path); blitter uses low 128 b
    wire         i_ddr_mem_ready;
    // M10a CWF early channel from ddr2_control (wide reads): the requested
    // dword (+ companion for even offsets) arrives 3+ cycles before the full
    // line — see the early-restart arms in READ_WAIT / WRITE_FETCH.
    wire [ 63:0] w_ddr_rd_dw;
    wire [ 63:0] w_ddr_rd_dw_next;
    wire         w_ddr_dw_next_ok;
    wire         w_ddr_dw_ready;
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
    logic          r_grant_blit = 1'b0;     // blitter owns DDR (master B)
    logic          r_grant_c2   = 1'b0;     // AMP core 2 owns DDR (master C)
    logic          r_grant_vga  = 1'b0;     // VGA scanout owns DDR (master D, read-only)
    logic          r_grant_sd   = 1'b0;     // the SD DMA engine owns DDR (master E)
    // Mutually exclusive by construction (one grant block below); the cache
    // (master A) owns the controller whenever all are low.

    wire         mig_write_DV   = r_grant_vga  ? 1'b0
                                : r_grant_sd   ? i_sd_write_DV
                                : r_grant_blit ? i_dma_write_DV
                                : r_grant_c2   ? i_c2_write_DV   : o_ddr_mem_write_DV;
    wire         mig_read_DV    = r_grant_vga  ? i_vga_read_DV
                                : r_grant_sd   ? i_sd_read_DV
                                : r_grant_blit ? i_dma_read_DV
                                : r_grant_c2   ? i_c2_read_DV    : o_ddr_mem_read_DV;
    wire [ 31:0] mig_addr       = r_grant_vga  ? i_vga_addr
                                : r_grant_sd   ? i_sd_addr
                                : r_grant_blit ? i_dma_addr
                                : r_grant_c2   ? i_c2_addr       : o_ddr_mem_addr;
    wire [255:0] mig_write_data = r_grant_sd   ? {128'b0, i_sd_write_data}
                                : r_grant_blit ? {128'b0, i_dma_write_data}
                                : r_grant_c2   ? {128'b0, i_c2_write_data}
                                               : o_ddr_mem_write_data;
    wire [ 15:0] mig_wdf_mask   = r_grant_sd   ? i_sd_wdf_mask
                                : r_grant_blit ? i_dma_wdf_mask
                                : r_grant_c2   ? i_c2_wdf_mask   : w_app_wdf_mask;
    // Cache and VGA fill a full 32 B line (two pipelined BL8 bursts); the
    // blitter and core 2 do single 128 b bursts. `mig_wide` tells ddr2_control
    // which.  VGA always asks for dword 0 (mig_dw_off, at the ddr2_control
    // instance: in-order bursts, no CWF).
    wire         mig_wide       = ~(r_grant_blit | r_grant_c2 | r_grant_sd);

    assign o_dma_read_data = i_ddr_mem_read_data[127:0]; // blitter uses low 128 b; latches on o_dma_ready
    assign o_dma_grant     = r_grant_blit;
    assign o_c2_read_data  = i_ddr_mem_read_data[127:0];
    assign o_c2_grant      = r_grant_c2;
    assign o_vga_read_data = i_ddr_mem_read_data;
    assign o_vga_grant     = r_grant_vga;
    assign o_sd_read_data  = i_ddr_mem_read_data[127:0];
    assign o_sd_grant      = r_grant_sd;

    // -------------------------------------------------------------------------
    // Cache-maintenance request capture. A flush/invalidate pulse sets a sticky
    // pending flag (+ mode); the main FSM consumes it from WAIT, raising
    // r_mnt_active for the duration of the walk. A pulse that arrives while a
    // walk is ALREADY RUNNING is captured too (it re-arms pending, so a fresh
    // walk runs after this one): the running walk may have passed lines that
    // were dirtied after it started, so dropping the new request would leave
    // software believing data was flushed that never reached DDR.
    //   r_mnt_mode: 0 = FLUSH (write back dirty), 1 = INVALIDATE (flush + clear valid)
    // The running walk uses its own r_mnt_walk_mode copy (latched at the
    // WAIT->MAINT handoff), so a new pulse mid-walk can never retarget the
    // walk already in progress.
    // -------------------------------------------------------------------------
    logic r_mnt_pending   = 1'b0;
    logic r_mnt_mode      = 1'b0;
    logic r_mnt_active    = 1'b0;   // set by the main FSM while a walk runs
    logic r_mnt_walk_mode = 1'b0;   // mode of the RUNNING walk (owned by the FSM)
    logic r_mnt_rng       = 1'b0;   // the RUNNING walk is a range walk (owned by the FSM)

    // Range operations (SD DMA spec v1.2).  One range op can wait here with
    // its own lines (r_rng_*, copied when it is accepted), behind a running
    // walk of either kind.  Anything that would need a second queue entry is
    // turned into a whole-cache walk, a superset that is always correct
    // (RANGE_INV writes back, so dropping more lines loses nothing):
    //   - a range op of more than 2 x 1024 lines (64 KiB, the cache) — the
    //     whole walk is no slower;
    //   - a range op while a whole-cache walk is pending (it merges in: an
    //     INVALIDATE if either invalidates), or while another range op is
    //     pending (both become one whole-cache walk);
    //   - a whole-cache request while a range op is pending (it absorbs it).
    // Every go is a one-cycle pulse (the CACHE_CTRL write's first cycle; a
    // level for the store's whole strobe re-armed the request after its walk
    // had started, so every whole-cache op walked twice — review 1).
    // r_mnt_pending clears on the FSM's own start condition, in the start
    // cycle, unless a go arrives in that same cycle: the walk starting then
    // froze the mode before it, so that go needs a walk of its own (review 1
    // MAJOR 2: it was cleared a cycle later, and the op lost).
    localparam int RNG_MAX = 2 * CACHE_SIZE;
    logic        r_rng_pend = 1'b0;
    logic        r_rng_inv  = 1'b0;
    logic [26:0] r_rng_l0;
    logic [11:0] r_rng_n;           // 1 .. RNG_MAX
    wire         w_rng_start;       // the FSM starts the pending range walk on this edge
    wire         w_rng_pq  = r_rng_pend && !w_rng_start;   // ...still pending after it
    wire         w_rng_cvt = i_rng_go && (i_rng_n > 28'(RNG_MAX) || w_rng_pq || r_mnt_pending);
    wire         w_wgo     = i_flush_go || i_inval_go || w_rng_cvt;
    wire         w_wgo_inv = i_inval_go || (w_rng_cvt && i_rng_inv) || (w_wgo && w_rng_pq && r_rng_inv);

    assign o_mnt_busy = r_mnt_active | r_mnt_pending | r_rng_pend;

    wire         w_whole_start;     // the FSM starts the pending whole walk on this edge
    always_ff @(posedge i_Clk) begin
        if (w_wgo) begin
            r_mnt_pending <= 1'b1;
            // INVALIDATE wins over FLUSH; a still-pending INVALIDATE is never
            // downgraded by a later FLUSH pulse (the walk does the superset).
            // (A go in the start cycle merges with the starting walk's mode
            // into the next walk: a superset.)
            r_mnt_mode    <= w_wgo_inv | (r_mnt_pending & r_mnt_mode);
        end else if (w_whole_start) begin
            // the walk starting on this edge owns the request
            r_mnt_pending <= 1'b0;
        end
        // the range slot
        if (w_rng_start) r_rng_pend <= 1'b0;
        if (w_wgo && w_rng_pq) r_rng_pend <= 1'b0;          // absorbed by the whole walk
        if (i_rng_go && !w_rng_cvt) begin
            r_rng_pend <= 1'b1;
            r_rng_inv  <= i_rng_inv;
            r_rng_l0   <= i_rng_l0;
            r_rng_n    <= i_rng_n[11:0];
        end
    end

    // -------------------------------------------------------------------------
    // ddr2_control's ready (i_ddr_mem_ready) is in the SAME ui_clk domain as the
    // cache FSM (the whole CPU runs synchronous to the 100 MHz MIG ui_clk), so
    // there is no clock crossing here and no 2-FF ready synchroniser — ready is
    // sampled directly.
    // -------------------------------------------------------------------------
    // ddr2_control's ready belongs to whichever master is currently granted.
    // Gate it so each master only acts on its own DDR completions — none can
    // mistake another's ready for its own.
    wire w_cache_ddr_ready = i_ddr_mem_ready & ~r_grant_blit & ~r_grant_c2 & ~r_grant_vga & ~r_grant_sd;
    wire w_blit_ddr_ready  = i_ddr_mem_ready &  r_grant_blit;
    wire w_c2_ddr_ready    = i_ddr_mem_ready &  r_grant_c2;
    wire w_vga_ddr_ready   = i_ddr_mem_ready &  r_grant_vga;
    wire w_sd_ddr_ready    = i_ddr_mem_ready &  r_grant_sd;
    // CWF early pulse is cache-only.  VGA's wide reads DO emit it (they are
    // the cache's read shape), so the VGA term here is load-bearing.
    wire w_cache_dw_ready  = w_ddr_dw_ready  & ~r_grant_blit & ~r_grant_c2 & ~r_grant_vga & ~r_grant_sd;

    assign o_dma_ready = w_blit_ddr_ready;
    assign o_c2_ready  = w_c2_ddr_ready;
    assign o_vga_ready = w_vga_ddr_ready;
    assign o_sd_ready  = w_sd_ddr_ready;

    // -------------------------------------------------------------------------
    // Address decode — combinational from cpu.addr (32-bit byte address).
    // CPU holds cpu.addr stable until cpu.ready, so these are stable
    // throughout any multi-cycle operation.
    //
    // DDR2 MIG address is in 16-bit half-word units (BL8 = 128-bit burst).
    // byte_addr → DDR half-word addr = byte_addr >> 1
    // 32 B line base = clear bottom 5 byte bits: {byte_addr[31:5], 5'b0}.
    // Doubleword offset within 256-bit line: byte_addr[4:3] (1-of-4 dwords).
    //   00=dw0 [255:192], 01=dw1 [191:128], 10=dw2 [127:64], 11=dw3 [63:0]
    // Cache index: addr[5+INDEX_BITS-1:5], Tag: addr[31:5+INDEX_BITS]
    // -------------------------------------------------------------------------
    wire [31:0]           w_computed_ddr_addr = {cpu.addr[31:5], 5'b00000};   // 32 B-aligned line base
    wire [INDEX_BITS-1:0] w_cache_index       = cpu.addr[5+INDEX_BITS-1:5];
    wire [TAG_BITS-1:0]   w_cache_tag         = cpu.addr[31:5+INDEX_BITS];
    wire [1:0]            w_dw_offset         = cpu.addr[4:3];   // 1-of-4 doubleword within the 256-bit line

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

    // 256-bit (32 B) cache line data
    (* ram_style = "block" *)
    logic [255:0] cache_val_data_way0 [CACHE_SIZE-1:0];

    (* ram_style = "block" *)
    logic [255:0] cache_val_data_way1 [CACHE_SIZE-1:0];

    // Load-hit pipe shadow copies of the tag and data arrays.  Written by
    // exactly the statements (and on exactly the edges) that write the arrays
    // above, read only by the hit pipe on their own port — so the FSM's
    // single read port and its timing are untouched.  (+8 RAMB36.)
    (* ram_style = "block" *)
    logic [TAG_BITS:0] h_tag_way0 [CACHE_SIZE-1:0];
    (* ram_style = "block" *)
    logic [TAG_BITS:0] h_tag_way1 [CACHE_SIZE-1:0];
    (* ram_style = "block" *)
    logic [255:0] h_data_way0 [CACHE_SIZE-1:0];
    (* ram_style = "block" *)
    logic [255:0] h_data_way1 [CACHE_SIZE-1:0];

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
    logic [255:0]          r_data_way0;
    logic [255:0]          r_data_way1;

    // Latched request (stable while CPU waits, but latching makes CDC clear)
    logic [INDEX_BITS-1:0] r_cache_index;
    logic [TAG_BITS-1:0]   r_cache_tag;
    logic [1:0]            r_dw_offset;     // 1-of-4 doubleword within the 256-bit line (00=dw0 … 11=dw3)
    logic [7:0]            r_byte_en;       // byte enables (8'hFF = full doubleword write)
    logic [63:0]           r_write_data;
    logic                  r_is_write;
    logic                  r_is_ifetch = 1'b0;   // an I-cache fill (L2 counters)
    // HW_PERF_PLAN.md §1 DCBZ: the latched write zeroes the whole line —
    // a hit overwrites it with zeros, a miss installs a zero line, both
    // dirty, with no DDR read (a dirty victim still goes out through the
    // shadow writeback).  Acked at the latch like every write (M10b).
    logic                  r_zero_line = 1'b0;
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
    logic [255:0] r_evict_data_hold;     // dirty 32 B line being written back to DDR
    logic [31:0]  r_evict_ddr_addr_r;   // DDR address of the dirty eviction
    logic [31:0]  r_fetch_ddr_addr;     // DDR address for the refill fetch
    logic         r_evict_way;          // which way to replace (miss paths)
    logic [3:0]   r_gap_count;          // DV-low settle gap between the refill and the
                                       // shadow writeback. Single domain, so it only
                                       // needs to let ddr2_control go IDLE->WAIT (both
                                       // DVs low) before the write DV. (EVICT_SHADOW_GAP)
    // M10a early restart bookkeeping. r_miss_served: the CPU has already been
    // released for the current miss (early cpu.ready pulsed on the CWF dword)
    // — suppresses the second present/ready at line-install time and stops the
    // stall counter. r_do_evict: the victim line was dirty; its writeback runs
    // AFTER the refill, in the shadow of the restarted CPU (fetch-first). The
    // victim's data/address live in r_evict_data_hold / r_evict_ddr_addr_r,
    // and the blitter arbiter is held off (is_miss_path) until it completes —
    // nothing can observe DDR's stale copy of the victim line in the window.
    logic         r_miss_served = 1'b0;
    logic         r_do_evict    = 1'b0;

    // -------------------------------------------------------------------------
    // M10b served-DV guard — replaces the PRE_WAIT/COOL_DOWN dead states.
    // After a ready pulse the CPU's DV stays high for ~2 more cycles (registered
    // splitter + consume edge); WAIT must not re-latch that same request. The
    // guard: r_dv_served is set at every ready-pulse edge (inline in the FSM,
    // so it is visible on the very first WAIT cycle after a completion) and
    // cleared the moment DV reads low — in ANY state, because the drop can
    // happen while a miss/shadow is still in flight. The ld32 spanning-load
    // path HOLDS DV and re-addresses atomically at its consume edge, so the
    // guard also releases on an address change (r_served_addr mismatch).
    // -------------------------------------------------------------------------
    logic        r_dv_served   = 1'b0;
    logic [31:0] r_served_addr = 32'b0;
    wire w_dv_new = (cpu.write_DV || cpu.read_DV) &&
                    !(r_dv_served && (cpu.addr == r_served_addr));

    // -------------------------------------------------------------------------
    // State machine — one-hot 16-bit
    // -------------------------------------------------------------------------
    localparam WAIT                  = 16'd2;     // idle: wait for DV, issue BRAM reads
    localparam CHECK                 = 16'd4;     // check registered BRAM results; hits complete here
    // 16'd8 (WRITE_HIT) and 16'd256 (READ_CACHE2) retired — both hit paths
    // now complete inside CHECK, saving one cycle per hit.
    // M10a fetch-first: 16'd64 (WRITE_EVICT_GAP), 16'd512/1024/2048
    // (READ_EVICT/_DONE/_GAP) retired — dirty misses now fetch FIRST (the CPU
    // restarts on the CWF dword) and write the victim back afterwards in the
    // shadow (EVICT_SHADOW_*, reusing the 16'd16/32 encodings).
    // M10b: 16'd1 (PRE_WAIT) and 16'd8192 (COOL_DOWN) retired — every
    // completion returns straight to WAIT; the served-DV guard (r_dv_served /
    // r_served_addr, declared above) absorbs the CPU's late DV drop that
    // those two dead states existed to cover.
    localparam EVICT_SHADOW_GAP      = 16'd16;    // DV-low settle before the shadow writeback
    localparam EVICT_SHADOW_WAIT     = 16'd32;    // wait for the shadow writeback to complete
    localparam WRITE_FETCH           = 16'd128;   // wait for fetch, merge, store (write-back: line installed dirty)
    localparam READ_WAIT             = 16'd4096;  // wait for DDR fetch
    // (History: the retired COOL_DOWN covered the 1-cycle return delay of
    // bus_splitter's registered output stage — WAIT re-entering while the
    // CPU's DV was still high re-latched the same request, and the phantom
    // ready pulses drove opcode fetches off stale cpu.read_data — see
    // CRASH_DUMP.md ERR=01 trace at PC=0x78. The served-DV guard covers this
    // by construction: same-address DV cannot re-latch until it drops.)
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
    localparam MS_RCHK    = 4'd9;   // range walk: which ways hold a line of the range
    logic [3:0] r_mnt_sub = MS_READ;

    // Range walk (r_mnt_rng).  It visits min(n, 1024) sets from the first
    // line's index, two cycles a set (MS_READ, MS_RCHK) when nothing there
    // needs writing back.  At visit j (the line L0 + j) a way holds a line
    // of the range iff its tag is T_j = (L0 + j)[26:10], or (n > 1024 and
    // j < n - 1024: the line L0 + j + 1024 at the same index) T_j + 1.
    // The selected ways (r_rw_s0 / r_rw_s1) are written back if dirty
    // (the whole walk's MS_W0 / MS_W1, qualified) and, for RANGE_INV,
    // dropped; the other ways are not touched.
    logic [TAG_BITS-1:0] r_rw_tag, r_rw_tag1;   // T_j, T_j + 1
    logic [10:0]         r_rw_left;             // sets still to visit (this one included)
    logic [10:0]         r_rw_two;              // visits left that hold a second range line
    logic                r_rw_inv;              // RANGE_INV (else RANGE_CLEAN)
    logic                r_rw_s0 = 1'b0, r_rw_s1 = 1'b0;
    wire w_rw_sel0 = r_tag_way0[TAG_BITS] && (r_tag_way0[TAG_BITS-1:0] == r_rw_tag ||
                                              (r_rw_two != 11'd0 && r_tag_way0[TAG_BITS-1:0] == r_rw_tag1));
    wire w_rw_sel1 = r_tag_way1[TAG_BITS] && (r_tag_way1[TAG_BITS-1:0] == r_rw_tag ||
                                              (r_rw_two != 11'd0 && r_tag_way1[TAG_BITS-1:0] == r_rw_tag1));
    // a whole walk selects every way (r_mnt_rng = 0)
    wire w_ms_sel0 = !r_mnt_rng || r_rw_s0;
    wire w_ms_sel1 = !r_mnt_rng || r_rw_s1;

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
            h_tag_way0[init_i]          = 0;
            h_tag_way1[init_i]          = 0;
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
    // WRITE_FETCH installs: on the refill's ready, or (DCBZ: no refill was
    // issued) on its first cycle — one install write for every array
    wire w_wf_fill       = w_cache_ddr_ready || r_zero_line;

    wire dirty0_wen = (w_write_hit_now &&                       r_hit_way0) ||
                      (state == WRITE_FETCH  && w_wf_fill         && r_evict_way == 1'b0) ||
                      (state == READ_WAIT    && w_cache_ddr_ready && r_evict_way == 1'b0) ||
                      (state == MAINT && r_mnt_sub == MS_CLR && w_ms_sel0);  // flush/inval: clear dirty
    // WRITE_FETCH installs as DIRTY (write-back on miss): avoids DDR write-through
    // after a miss, which would race against MIG's internal write pipeline and
    // could clobber bytes committed by a prior buffered write to the same line.
    // The dirty line will be written back to DDR when it is itself evicted
    // (EVICT_SHADOW_* on that later miss) or by a maintenance flush.
    wire dirty0_din = w_write_hit_now || (state == WRITE_FETCH && w_wf_fill);

    wire dirty1_wen = (w_write_hit_now &&                       r_hit_way1) ||
                      (state == WRITE_FETCH  && w_wf_fill         && r_evict_way == 1'b1) ||
                      (state == READ_WAIT    && w_cache_ddr_ready && r_evict_way == 1'b1) ||
                      (state == MAINT && r_mnt_sub == MS_CLR && w_ms_sel1);  // flush/inval: clear dirty
    wire dirty1_din = w_write_hit_now || (state == WRITE_FETCH && w_wf_fill);

    always_ff @(posedge i_Clk) begin
        if (dirty0_wen) cache_dirty_way0[r_cache_index] <= dirty0_din;
    end

    always_ff @(posedge i_Clk) begin
        if (dirty1_wen) cache_dirty_way1[r_cache_index] <= dirty1_din;
    end

    // -------------------------------------------------------------------------
    // Data array writes (tmg timing): byte write enables, from registers.
    // A write hit used to read-modify-write the whole line in CHECK — the
    // hit way's line (r_data_wayN, the BRAM's unregistered output) merged
    // with the store's dword and written back to both arrays: BRAM -> merge
    // -> BRAM, the build's worst path (cache_val_data -> h_data DIADI).  It
    // now writes only the store's bytes, from r_write_data (registered in
    // WAIT), with the hit decided on the write enables; the other bytes of
    // the line keep their contents, which is exactly the old merge (the
    // FSM writes nothing between the WAIT read and CHECK).  A DCBZ hit
    // writes all 32 bytes with zeros.  The installs (WRITE_FETCH's merged
    // fetch line, READ_WAIT's line) write every byte of the victim way.
    // Every array write is still at r_cache_index, on the same edges, so
    // the hit pipe's conflict flag (r_h_conf) is unchanged.  (Both ways
    // hitting — never, the tags of a set are distinct — would now leave
    // each way its own other bytes, where the merge copied way 0's.)
    // -------------------------------------------------------------------------
    logic [255:0] w_wf_line;   // WRITE_FETCH's install: the fetched line, the store merged in
    always_comb begin
        logic [63:0] old_dw, new_dw;
        // fetched 256 b line: dw0[255:192] dw1[191:128] dw2[127:64] dw3[63:0]
        case (r_dw_offset)
            2'b00:   old_dw = i_ddr_mem_read_data[255:192];
            2'b01:   old_dw = i_ddr_mem_read_data[191:128];
            2'b10:   old_dw = i_ddr_mem_read_data[127:64];
            default: old_dw = i_ddr_mem_read_data[63:0];
        endcase
        for (int b = 0; b < 8; b++)
            new_dw[8*b +: 8] = r_byte_en[b] ? r_write_data[8*b +: 8] : old_dw[8*b +: 8];
        w_wf_line = i_ddr_mem_read_data;
        case (r_dw_offset)
            2'b00:   w_wf_line[255:192] = new_dw;
            2'b01:   w_wf_line[191:128] = new_dw;
            2'b10:   w_wf_line[127:64]  = new_dw;
            default: w_wf_line[63:0]    = new_dw;
        endcase
        if (r_zero_line) w_wf_line = '0;   // DCBZ miss: a zero line, no fetch
    end
    // the install edges (one write per array, the victim way)
    wire         w_dfill   = (state == WRITE_FETCH && w_wf_fill) ||
                             (state == READ_WAIT   && w_cache_ddr_ready);
    // a write hit's bytes: dword r_dw_offset (dw k at bytes 31-8k .. 24-8k)
    wire [31:0]  w_hit_be  = r_zero_line ? 32'hFFFF_FFFF
                                         : (32'(r_byte_en) << {~r_dw_offset, 3'b000});
    wire [255:0] w_dwdata  = (state == CHECK)       ? (r_zero_line ? 256'b0 : {4{r_write_data}})
                           : (state == WRITE_FETCH) ? w_wf_line
                           :                          i_ddr_mem_read_data;
    wire [31:0]  w_dwe0    = (w_write_hit_now && r_hit_way0) ? w_hit_be
                                                             : {32{w_dfill && r_evict_way == 1'b0}};
    wire [31:0]  w_dwe1    = (w_write_hit_now && r_hit_way1) ? w_hit_be
                                                             : {32{w_dfill && r_evict_way == 1'b1}};
    always_ff @(posedge i_Clk)
        for (int b = 0; b < 32; b++)
            if (w_dwe0[b]) cache_val_data_way0[r_cache_index][8*b +: 8] <= w_dwdata[8*b +: 8];
    always_ff @(posedge i_Clk)
        for (int b = 0; b < 32; b++)
            if (w_dwe0[b]) h_data_way0[r_cache_index][8*b +: 8] <= w_dwdata[8*b +: 8];
    always_ff @(posedge i_Clk)
        for (int b = 0; b < 32; b++)
            if (w_dwe1[b]) cache_val_data_way1[r_cache_index][8*b +: 8] <= w_dwdata[8*b +: 8];
    always_ff @(posedge i_Clk)
        for (int b = 0; b < 32; b++)
            if (w_dwe1[b]) h_data_way1[r_cache_index][8*b +: 8] <= w_dwdata[8*b +: 8];

    // -------------------------------------------------------------------------
    // 1a stage 3 load-hit pipe.  H1 (this edge): read both shadow ways at
    // i_h_addr.  H2 (next cycle, comb): compare, select, qualify.  Every array
    // write is at r_cache_index in a non-WAIT state, so a read in the same
    // edge as a write to its set (read-first: the old line) or during a
    // maintenance walk is flagged and the core takes its slow path.  A store
    // still on the port (latching, not yet written) is the core's check.
    // -------------------------------------------------------------------------
    wire [INDEX_BITS-1:0] w_h_idx = i_h_addr[5+INDEX_BITS-1:5];
    logic [TAG_BITS:0]    r_h_tq0, r_h_tq1;
    logic [255:0]         r_h_dq0, r_h_dq1;
    logic [TAG_BITS-1:0]  r_h_tag, r_h_tag1;
    logic                 r_h_c1;
    wire  [31:0]          w_h_a0 = {i_h_pg0, i_h_addr[15:0]};
    wire  [31:0]          w_h_a1 = {i_h_pg1, i_h_addr[15:0]};
    logic [1:0]           r_h_off;
    logic                 r_h_conf = 1'b1;
    always_ff @(posedge i_Clk) begin
        r_h_tq0  <= h_tag_way0[w_h_idx];
        r_h_tq1  <= h_tag_way1[w_h_idx];
        r_h_dq0  <= h_data_way0[w_h_idx];
        r_h_dq1  <= h_data_way1[w_h_idx];
        r_h_tag  <= w_h_a0[31:5+INDEX_BITS];
        r_h_tag1 <= w_h_a1[31:5+INDEX_BITS];
        r_h_c1   <= i_h_c1;
        r_h_off  <= i_h_addr[4:3];
        // (and while an invalidating walk waits to start — RANGE_INV, or a
        // whole INVALIDATE, a range op converted into one included — the
        // cache still busy with an earlier miss: a load must not hit a line
        // it is about to drop, so the invalidates are synchronous for loads
        // as for the port; review 1 MAJOR 1)
        r_h_conf <= (state == MAINT) || (state != WAIT && r_cache_index == w_h_idx) ||
                    (r_rng_pend && r_rng_inv) || (r_mnt_pending && r_mnt_mode);
    end
    // (both candidates compared in parallel; the carry selects the result)
    wire w_h_hit0 = r_h_tq0[TAG_BITS] && (r_h_c1 ? (r_h_tq0[TAG_BITS-1:0] == r_h_tag1)
                                                 : (r_h_tq0[TAG_BITS-1:0] == r_h_tag));
    wire w_h_hit1 = r_h_tq1[TAG_BITS] && (r_h_c1 ? (r_h_tq1[TAG_BITS-1:0] == r_h_tag1)
                                                 : (r_h_tq1[TAG_BITS-1:0] == r_h_tag));
`ifndef SYNTHESIS
    // the selected candidate is i_h_addr's tag
    logic [TAG_BITS-1:0] r_h_tagx;
    always_ff @(posedge i_Clk) r_h_tagx <= i_h_addr[31:5+INDEX_BITS];
    // (4-state sims: skipped while any of them is X — the first cycle, or a
    // bench that leaves the hit-pipe inputs unconnected)
    always_ff @(posedge i_Clk) if (!$isunknown({r_h_tag, r_h_tag1, r_h_c1, r_h_tagx}))
        assert ((r_h_c1 ? r_h_tag1 : r_h_tag) == r_h_tagx)
            else $fatal(1, "hit-pipe tag candidates %h / %h (c1 %b) != %h", r_h_tag, r_h_tag1, r_h_c1, r_h_tagx);
`endif
    assign o_h_ok = (w_h_hit0 || w_h_hit1) && !r_h_conf;
`ifndef SYNTHESIS
    // +HCOV: cycles whose hit-pipe read would have hit but overlapped an FSM
    // write to its set (flagged, the core's slow path) — hazard coverage
    int hc_conf = 0;
    bit hc_on = $test$plusargs("HCOV");
    always_ff @(posedge i_Clk) if (hc_on && r_h_conf && (w_h_hit0 || w_h_hit1) && state != MAINT) hc_conf++;
    final if (hc_on) $display("HCOV cache conflict_hits=%0d", hc_conf);
`endif
    // (tmg timing) each way's dword first, then the way: the core extracts
    // a load's value from both ways' dwords in parallel with the tag
    // compare, and w_h_hit0 picks at its register (o_h_dw0 / o_h_dw1 /
    // o_h_sel0); o_h_dword = o_h_sel0 ? o_h_dw0 : o_h_dw1, as before
    function automatic logic [63:0] f_hdw(logic [255:0] l, logic [1:0] off);
        return (off == 2'b00) ? l[255:192] : (off == 2'b01) ? l[191:128]
             : (off == 2'b10) ? l[127:64]  :                  l[63:0];
    endfunction
    assign o_h_dw0   = f_hdw(r_h_dq0, r_h_off);
    assign o_h_dw1   = f_hdw(r_h_dq1, r_h_off);
    assign o_h_sel0  = w_h_hit0;
    assign o_h_dword = w_h_hit0 ? o_h_dw0 : o_h_dw1;

    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always_ff @(posedge i_Clk) begin : fsm

        // Served-DV guard clear — runs in every state (see the declaration
        // comment). The case arms' set-assignments below override when a ready
        // pulses this edge (DV is always high at a ready-set edge).
        if (!(cpu.write_DV || cpu.read_DV))
            r_dv_served <= 1'b0;
        o_cpu_line_v <= 1'b0;   // (HW_PERF_PLAN §5: one-cycle line strobe)

        case (state)

            // ------------------------------------------------------------------
            // WAIT: idle state. When the CPU asserts a NEW DV (served-DV guard:
            // a same-address DV that was already given its ready pulse is the
            // tail of the previous transaction, not a request), issue all BRAM
            // reads for tags, dirty bits, LRU, and both data ways
            // simultaneously. All results are available next cycle in CHECK.
            //
            // M10b write-ack-at-latch: a WRITE is acknowledged HERE, at the
            // latch edge — one cycle before CHECK even knows hit/miss. The
            // store is architecturally committed the moment it is latched
            // (this FSM guarantees completion, subsequent accesses are ordered
            // behind it in WAIT, MMIO never routes through the cache, and the
            // I-cache store snoop runs at ISSUE time in pipeline_core). A
            // write MISS therefore starts with r_miss_served already set: the
            // CWF early-ack and install-time fallback in WRITE_FETCH are
            // pre-suppressed and the whole miss runs in the shadow.
            // ------------------------------------------------------------------
            WAIT: begin
                cpu.ready      <= 0;
                cpu.next_valid <= 0;
                if (r_mnt_pending) begin
                    // Cache maintenance has priority over CPU requests: a pending
                    // flush/invalidate is consumed here, while the CPU stalls on
                    // its (un-served) request until the walk completes.
                    r_mnt_active    <= 1'b1;
                    r_mnt_walk_mode <= r_mnt_mode;   // freeze the mode for this walk
                    r_mnt_rng       <= 1'b0;
                    r_cache_index   <= {INDEX_BITS{1'b0}};
                    r_mnt_sub       <= MS_READ;
                    state           <= MAINT;
                end else if (r_rng_pend) begin
                    // a range walk (w_rng_start): from the op's copied lines
                    r_mnt_active    <= 1'b1;
                    r_mnt_rng       <= 1'b1;
                    r_rw_inv        <= r_rng_inv;
                    r_cache_index   <= r_rng_l0[INDEX_BITS-1:0];
                    r_rw_tag        <= r_rng_l0[26:INDEX_BITS];
                    r_rw_tag1       <= r_rng_l0[26:INDEX_BITS] + 1'b1;
                    r_rw_left       <= (r_rng_n > 12'(CACHE_SIZE)) ? 11'(CACHE_SIZE) : r_rng_n[10:0];
                    r_rw_two        <= (r_rng_n > 12'(CACHE_SIZE)) ? 11'(r_rng_n - 12'(CACHE_SIZE)) : 11'd0;
                    r_mnt_sub       <= MS_READ;
                    state           <= MAINT;
                end else if (w_dv_new) begin
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
                    r_dw_offset         <= w_dw_offset;
                    r_byte_en           <= cpu.byte_en;
                    r_write_data        <= cpu.write_data;
                    r_is_write          <= cpu.write_DV;
                    r_is_ifetch         <= cpu.read_DV && cpu.ifetch;
                    r_zero_line         <= cpu.write_DV && cpu.zero_line;
                    r_computed_ddr_addr <= w_computed_ddr_addr;
                    r_miss_served       <= cpu.write_DV;  // writes: served right here
                    if (cpu.write_DV) begin
                        cpu.ready     <= 1;               // write-ack at latch
                        r_dv_served   <= 1;
                        r_served_addr <= cpu.addr;
                    end
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
                cpu.ready <= 0;   // clears the 1-cycle write-ack pulse from WAIT
                if (r_is_write) begin

                    if (r_cache_hit) begin
                        // Write hit — the hit way's data arrays take the
                        // store's bytes (or, DCBZ, the whole line of zeros)
                        // through their byte write enables (w_dwe0 / w_dwe1,
                        // below the FSM).  Dirty bit set via the
                        // combinatorial dirty0/1_wen wires (w_write_hit_now).
                        cache_lru[r_cache_index] <= r_hit_way0 ? 1'b1 : 1'b0;

                        // Already acked at the WAIT latch (write-ack-at-latch).
                        state       <= WAIT;

                    end else begin
                        // Write miss — M10a fetch-first: issue the refill NOW
                        // (dirty or not); a dirty victim's writeback runs in
                        // the shadow after the CPU restarts (r_do_evict).
                        // DCBZ (r_zero_line): no refill is issued; WRITE_FETCH
                        // installs the zero line on its first cycle through
                        // the same install write (one write port per array).
                        r_evict_way        <= r_evict_way_sel;
                        r_evict_ddr_addr_r <= {r_evict_tag, r_cache_index, 5'b00000};
                        r_fetch_ddr_addr   <= r_computed_ddr_addr;
                        r_evict_data_hold  <= r_evict_way_sel ? r_data_way1 : r_data_way0;
                        r_do_evict         <= r_evict_dirty;
                        o_ddr_mem_addr     <= r_computed_ddr_addr;
                        o_ddr_mem_read_DV  <= !r_zero_line;
                        state              <= WRITE_FETCH;
                    end

                end else begin // read

                    if (r_cache_hit) begin
                        // Read hit — present the requested doubleword now.
                        // r_byte_offset 0 = upper [127:64], 1 = lower [63:0];
                        // the "next" doubleword is only valid at offset 0
                        // (companion is the lower dw of the same line).
                        begin : read_hit_present
                            logic [255:0] hitline;
                            hitline = r_hit_way0 ? r_data_way0 : r_data_way1;
                            // Present the requested doubleword at r_dw_offset, plus the
                            // NEXT sequential doubleword within the 32 B line (offset+1)
                            // for the IFB lookahead. next_valid whenever offset != dw3.
                            case (r_dw_offset)
                                2'b00: begin cpu.read_data <= hitline[255:192]; cpu.read_data_next <= hitline[191:128]; end
                                2'b01: begin cpu.read_data <= hitline[191:128]; cpu.read_data_next <= hitline[127:64];  end
                                2'b10: begin cpu.read_data <= hitline[127:64];  cpu.read_data_next <= hitline[63:0];    end
                                default: begin cpu.read_data <= hitline[63:0];  cpu.read_data_next <= 64'h0;            end
                            endcase
                            cpu.next_valid <= (r_dw_offset != 2'b11);
                        end
                        cache_lru[r_cache_index] <= r_hit_way0 ? 1'b1 : 1'b0;
                        o_cpu_line    <= r_hit_way0 ? r_data_way0 : r_data_way1;
                        o_cpu_line_v  <= 1'b1;

                        cpu.ready     <= 1;
                        r_dv_served   <= 1;              // served-DV guard
                        r_served_addr <= cpu.addr;       // (CPU holds addr until ready)
                        state         <= WAIT;

                    end else begin
                        // Read miss — M10a fetch-first (see the write-miss
                        // comment above).
                        r_evict_way        <= r_evict_way_sel;
                        r_evict_ddr_addr_r <= {r_evict_tag, r_cache_index, 5'b00000};
                        r_fetch_ddr_addr   <= r_computed_ddr_addr;
                        r_evict_data_hold  <= r_evict_way_sel ? r_data_way1 : r_data_way0;
                        r_do_evict         <= r_evict_dirty;
                        o_ddr_mem_addr     <= r_computed_ddr_addr;
                        o_ddr_mem_read_DV  <= 1;
                        state              <= READ_WAIT;
                    end

                end
            end // CHECK

            // ------------------------------------------------------------------
            // WRITE MISS — refill in flight (fetch-first; any dirty victim
            // writeback follows in EVICT_SHADOW_* after the CPU restarts)
            // ------------------------------------------------------------------
            WRITE_FETCH: begin
                cpu.ready <= 0;
                // M10b: writes are acked at the WAIT latch (r_miss_served
                // arrives here already set), so this early arm and the
                // fallback below are vestigial write-path safety nets.
                if (w_cache_dw_ready && !r_miss_served) begin
                    cpu.ready     <= 1;
                    r_dv_served   <= 1;
                    r_served_addr <= cpu.addr;
                    r_miss_served <= 1;
                end
                if (w_wf_fill) begin
                    o_ddr_mem_read_DV <= 0;

                    // (the merged line goes into the data arrays through
                    // w_dwdata / w_dwe0 / w_dwe1, below the FSM)

                    // Install merged line in cache as DIRTY (write-back policy on miss).
                    // Dirty bit set via dirty0/1_din combinatorial wires above.
                    if (r_evict_way == 1'b0) begin
                        cache_val_addr_way0[r_cache_index] <= {1'b1, r_cache_tag};
                        h_tag_way0[r_cache_index]          <= {1'b1, r_cache_tag};
                    end
                    if (r_evict_way == 1'b1) begin
                        cache_val_addr_way1[r_cache_index] <= {1'b1, r_cache_tag};
                        h_tag_way1[r_cache_index]          <= {1'b1, r_cache_tag};
                    end
                    cache_lru[r_cache_index] <= (r_evict_way == 1'b0) ? 1'b1 : 1'b0;

                    // Line installed as DIRTY — will be written back to DDR on eviction.
                    // Dirty bit set via dirty0/1_din combinatorial wires above.
                    if (!r_miss_served) begin
                        cpu.ready     <= 1;   // vestigial fallback (see above)
                        r_dv_served   <= 1;
                        r_served_addr <= cpu.addr;
                    end
                    if (r_do_evict && r_zero_line) begin
                        // DCBZ: no refill was issued, so ddr2_control has
                        // been in its DV-gated WAIT since this request's own
                        // WAIT → CHECK (it passes the gate one edge after
                        // any earlier transaction's DV drop) — the DV-low
                        // settle gap has nothing to settle.  Raise the
                        // writeback now, exactly as CHECK raises a refill's
                        // read DV.  (3 cycles off every dirty-victim DCBZ.)
                        o_ddr_mem_addr       <= r_evict_ddr_addr_r;
                        o_ddr_mem_write_data <= r_evict_data_hold;
                        r_app_wdf_mask       <= 16'b0000_0000_0000_0000; // all bytes valid
                        o_ddr_mem_write_DV   <= 1;
                        state                <= EVICT_SHADOW_WAIT;
                    end else if (r_do_evict) begin
                        r_gap_count <= WB_GAP;
                        state       <= EVICT_SHADOW_GAP;
                    end else begin
                        state       <= WAIT;   // served-DV guard absorbs the DV tail
                    end
                end
            end

            // READ_CACHE2 / WRITE_HIT removed — both hit paths now complete
            // inside CHECK (see the CHECK comment above).

            // ------------------------------------------------------------------
            // READ WAIT: refill in flight. M10a early restart: the CPU gets the
            // requested dword the beat it arrives (CWF: always beat 0/1); the
            // full-line install — and any dirty victim writeback — complete in
            // the shadow while the CPU is already running again.
            // ------------------------------------------------------------------
            READ_WAIT: begin
                cpu.ready      <= 0;   // the early ready below is a 1-cycle pulse
                cpu.next_valid <= 0;
                if (w_cache_dw_ready && !r_miss_served) begin
                    // Early restart: present the CWF dword (+ companion for
                    // even offsets — exactly the cases the IFB lookahead and
                    // the ld32 span fast path can use; odd offsets return
                    // next_valid=0 and those consumers fall back correctly).
                    cpu.read_data      <= w_ddr_rd_dw;
                    cpu.read_data_next <= w_ddr_rd_dw_next;
                    cpu.next_valid     <= w_ddr_dw_next_ok;
                    cpu.ready          <= 1;
                    r_dv_served        <= 1;
                    r_served_addr      <= cpu.addr;
                    r_miss_served      <= 1;
                end
                if (w_cache_ddr_ready) begin
                    o_ddr_mem_read_DV <= 0;
                    o_cpu_line        <= i_ddr_mem_read_data;
                    o_cpu_line_v      <= 1'b1;

                    // Dirty bit cleared via dirty0/1_wen combinatorial wires above.
                    // (the line goes into the data arrays through w_dwe0 /
                    // w_dwe1, below the FSM)
                    if (r_evict_way == 1'b0) begin
                        cache_val_addr_way0[r_cache_index] <= {1'b1, r_cache_tag};
                        h_tag_way0[r_cache_index]          <= {1'b1, r_cache_tag};
                    end
                    if (r_evict_way == 1'b1) begin
                        cache_val_addr_way1[r_cache_index] <= {1'b1, r_cache_tag};
                        h_tag_way1[r_cache_index]          <= {1'b1, r_cache_tag};
                    end
                    cache_lru[r_cache_index] <= (r_evict_way == 1'b0) ? 1'b1 : 1'b0;

                    if (!r_miss_served) begin
                        // Fallback (no early pulse this miss): present requested
                        // dw + next sequential dw (IFB lookahead) from the line.
                        case (r_dw_offset)
                            2'b00: begin cpu.read_data <= i_ddr_mem_read_data[255:192]; cpu.read_data_next <= i_ddr_mem_read_data[191:128]; end
                            2'b01: begin cpu.read_data <= i_ddr_mem_read_data[191:128]; cpu.read_data_next <= i_ddr_mem_read_data[127:64];  end
                            2'b10: begin cpu.read_data <= i_ddr_mem_read_data[127:64];  cpu.read_data_next <= i_ddr_mem_read_data[63:0];    end
                            default: begin cpu.read_data <= i_ddr_mem_read_data[63:0];  cpu.read_data_next <= 64'h0;                        end
                        endcase
                        cpu.next_valid <= (r_dw_offset != 2'b11);
                        cpu.ready      <= 1;
                        r_dv_served    <= 1;
                        r_served_addr  <= cpu.addr;
                    end

                    if (r_do_evict) begin
                        r_gap_count <= WB_GAP;
                        state       <= EVICT_SHADOW_GAP;
                    end else begin
                        state       <= WAIT;   // served-DV guard absorbs the DV tail
                    end
                end
            end

            // ------------------------------------------------------------------
            // EVICT SHADOW: dirty victim writeback, AFTER the refill (fetch-
            // first) and after the CPU has been released. The victim exists
            // only in r_evict_data_hold/r_evict_ddr_addr_r here (its tags were
            // overwritten at install) — is_miss_path keeps the blitter off the
            // bus for the whole window, so DDR's stale copy is unobservable.
            // ------------------------------------------------------------------
            EVICT_SHADOW_GAP: begin
                // Clear any fallback ready pulse and let ddr2_control pass its
                // DV-low IDLE gate before the write DV rises — same settle
                // discipline as the old WRITE_EVICT_GAP, direction reversed.
                cpu.ready      <= 0;
                cpu.next_valid <= 0;
                if (r_gap_count == 4'd0) begin
                    o_ddr_mem_addr       <= r_evict_ddr_addr_r;
                    o_ddr_mem_write_data <= r_evict_data_hold;
                    r_app_wdf_mask       <= 16'b0000_0000_0000_0000; // all bytes valid
                    o_ddr_mem_write_DV   <= 1;
                    state                <= EVICT_SHADOW_WAIT;
                end else begin
                    r_gap_count <= r_gap_count - 4'd1;
                end
            end

            EVICT_SHADOW_WAIT: begin
                if (w_cache_ddr_ready) begin
                    o_ddr_mem_write_DV <= 0;
                    // Hold the victim address while ddr2_control drains and
                    // passes its DV gate. A new miss cannot change
                    // o_ddr_mem_addr before WAIT → CHECK (2 cycles), by which
                    // point ddr2_control has re-entered its DV-gated WAIT; a
                    // spurious re-sampled write would hit the victim address
                    // again: idempotent.
                    state <= WAIT;
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
                        r_mnt_sub    <= r_mnt_rng ? MS_RCHK : MS_W0;
                    end

                    MS_RCHK: begin
                        // range walk: select the ways holding range lines;
                        // with nothing dirty among them, drop them now
                        // (RANGE_INV) and go to the next set — else the
                        // whole walk's writeback steps, then MS_CLR
                        r_rw_s0 <= w_rw_sel0;
                        r_rw_s1 <= w_rw_sel1;
                        if ((w_rw_sel0 && r_dirty_way0) || (w_rw_sel1 && r_dirty_way1)) begin
                            r_mnt_sub <= MS_W0;
                        end else begin
                            if (r_rw_inv && w_rw_sel0) begin
                                cache_val_addr_way0[r_cache_index] <= {(TAG_BITS+1){1'b0}};
                                h_tag_way0[r_cache_index]          <= {(TAG_BITS+1){1'b0}};
                            end
                            if (r_rw_inv && w_rw_sel1) begin
                                cache_val_addr_way1[r_cache_index] <= {(TAG_BITS+1){1'b0}};
                                h_tag_way1[r_cache_index]          <= {(TAG_BITS+1){1'b0}};
                            end
                            if (r_rw_left == 11'd1) begin
                                r_mnt_sub <= MS_DONE;
                            end else begin
                                r_cache_index <= r_cache_index + 1'b1;
                                if (r_cache_index == INDEX_BITS'(CACHE_SIZE - 1)) begin
                                    r_rw_tag  <= r_rw_tag + 1'b1;
                                    r_rw_tag1 <= r_rw_tag1 + 1'b1;
                                end
                                r_rw_left <= r_rw_left - 11'd1;
                                if (r_rw_two != 11'd0) r_rw_two <= r_rw_two - 11'd1;
                                r_mnt_sub <= MS_READ;
                            end
                        end
                    end

                    MS_W0: begin
                        // r_tag_wayX[TAG_BITS] = valid bit. Line offset is 5 bits
                        // (32 B lines) — same construction as r_evict_ddr_addr_r.
                        // A 4-bit offset here (16 B-line era fossil) made the
                        // 31-bit concat zero-extend and flushed every dirty line
                        // to HALF its address — the LVGL flush=blitter code
                        // corruption (0xA600A600 at PC 0xD69A4 = fb line
                        // 0x1AD340 mis-flushed to 0xD69A0).
                        if (r_tag_way0[TAG_BITS] && r_dirty_way0 && w_ms_sel0) begin
                            o_ddr_mem_addr       <= {r_tag_way0[TAG_BITS-1:0], r_cache_index, 5'b00000};
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
                            r_gap_count        <= WB_GAP;   // CDC settle, hold addr
                            r_mnt_sub          <= MS_W0_GAP;
                        end
                    end

                    MS_W0_GAP: begin
                        if (r_gap_count == 4'd0) r_mnt_sub <= MS_W1;
                        else                     r_gap_count <= r_gap_count - 4'd1;
                    end

                    MS_W1: begin
                        if (r_tag_way1[TAG_BITS] && r_dirty_way1 && w_ms_sel1) begin
                            o_ddr_mem_addr       <= {r_tag_way1[TAG_BITS-1:0], r_cache_index, 5'b00000};
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
                            r_gap_count        <= WB_GAP;
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
                        if (r_mnt_rng ? r_rw_inv : r_mnt_walk_mode) begin   // INVALIDATE / RANGE_INV
                            if (w_ms_sel0) begin
                                cache_val_addr_way0[r_cache_index] <= {(TAG_BITS+1){1'b0}};
                                h_tag_way0[r_cache_index]          <= {(TAG_BITS+1){1'b0}};
                            end
                            if (w_ms_sel1) begin
                                cache_val_addr_way1[r_cache_index] <= {(TAG_BITS+1){1'b0}};
                                h_tag_way1[r_cache_index]          <= {(TAG_BITS+1){1'b0}};
                            end
                        end
                        if (r_mnt_rng) begin
                            if (r_rw_left == 11'd1) begin
                                r_mnt_sub <= MS_DONE;
                            end else begin
                                r_cache_index <= r_cache_index + 1'b1;
                                if (r_cache_index == INDEX_BITS'(CACHE_SIZE - 1)) begin
                                    r_rw_tag  <= r_rw_tag + 1'b1;
                                    r_rw_tag1 <= r_rw_tag1 + 1'b1;
                                end
                                r_rw_left <= r_rw_left - 11'd1;
                                if (r_rw_two != 11'd0) r_rw_two <= r_rw_two - 11'd1;
                                r_mnt_sub <= MS_READ;
                            end
                        end else if (r_cache_index == CACHE_SIZE - 1) begin
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
    localparam [31:0] LP_CACHE_TOTAL_BYTES = CACHE_SIZE * 2 * 32;  // ways × sets × line_bytes
    localparam [15:0] LP_CACHE_NUM_SETS    = CACHE_SIZE;
    localparam [ 7:0] LP_CACHE_LINE_BYTES  = 8'd32;
    localparam [ 7:0] LP_CACHE_NUM_WAYS    = 8'd2;
    localparam [63:0] LP_CACHE_INFO = {
        LP_CACHE_TOTAL_BYTES,   // [63:32]
        LP_CACHE_LINE_BYTES,    // [31:24]
        LP_CACHE_NUM_SETS,      // [23: 8]
        LP_CACHE_NUM_WAYS       // [ 7: 0]
    };
    assign o_cache_info = LP_CACHE_INFO;

    // M10a: the shadow-writeback states are part of the miss path — the
    // blitter arbiter must stay locked out until the victim line is safely in
    // DDR (see the EVICT_SHADOW comment in the FSM).
    // the FSM takes the pending range op on this edge (WAIT, no whole walk first)
    assign w_rng_start = (state == WAIT) && !r_mnt_pending && r_rng_pend;
    assign w_whole_start = (state == WAIT) && r_mnt_pending;

    wire is_miss_path = (state == WRITE_FETCH)      ||
                        (state == READ_WAIT)        ||
                        (state == EVICT_SHADOW_GAP) ||
                        (state == EVICT_SHADOW_WAIT);

    // -------------------------------------------------------------------------
    // DMA grant arbitration (see the muxed mig_* wires and the DMA master port).
    // Cache is the priority master; the blitter is handed the bus only while the
    // cache is NOT mid-transaction (is_miss_path low), so an in-flight cache
    // miss — including its CDC writeback gaps — is never interrupted. The
    // blitter holds the grant for a CHUNK of transactions (8) and pulses
    // i_dma_done at chunk boundaries and blit end, so even a full-frame blit
    // yields the bus back to the CPU every few hundred cycles (bounded CPU
    // stall, no starvation); a cache request that arrives mid-chunk parks in
    // its miss state and is served at the next chunk boundary.
    //
    // The blitter is also held off while a cache-maintenance walk is active
    // (r_mnt_active): the walk's writebacks use the master-A path and must not
    // be interrupted mid-CDC-gap. In the intended coherency sequence the blit
    // and the flush/invalidate never overlap anyway.
    // -------------------------------------------------------------------------
    // Three masters, one bus: cache (A, priority — owns it whenever no grant
    // is out), blitter (B), AMP core 2 (C, lowest).  B and C are granted only
    // from the no-grant state, so they are mutually exclusive; each releases
    // via its own done pulse.  Core 2 releases per-transaction, so a cache or
    // blitter request never waits behind more than one 16 B burst.
    // Orphaned-grant guard: both DMA masters hold their req HIGH for the whole
    // tenure and only drop it in the same cycle they pulse done (blit end /
    // core-2 transaction end).  A granted master showing req=0 with no done
    // pulse can therefore only mean it was RESET mid-tenure (CPU_RESETN during
    // a blit); without this guard the grant is held forever and every later
    // cache miss hangs on the parked bus.  Drain a few quiet cycles first so
    // any DDR transaction the dying master left in flight completes before
    // the cache can see a stale ready.
    //
    // Master D (VGA scanout) is granted FIRST among the DMA masters, and its
    // gate is looser: it is read-only (no coherency point to protect), so a
    // pending/running maintenance walk does not block it — only a cache miss
    // in flight, or a walk that is mid-writeback.  MS_READ is the walk's
    // between-sets point: no write DV is up and the previous set's CDC gap has
    // drained, so the walk can park there (its next writeback waits in
    // MS_Wx_WAIT on the gated w_cache_ddr_ready) exactly like a cache miss
    // parks behind a blitter chunk.  Without this a full-cache flush (up to
    // ~1.2 ms with every line dirty) would underflow ~38 display lines.
    // (HW_PERF_PLAN §8 item 3d: the quiet cycles were not enough — a read
    // the dead master left below answers when DDR does, ~40+ cycles with
    // the L2's miss path, and that ready went to the D-cache, which takes a
    // request held up by the grant, as its own line: an orphaned grant is
    // released only once the controller's front end is idle as well
    // (w_ddr_fe_idle: no accepted request unanswered).  The count saturates.)
    wire        w_ddr_fe_idle;
    logic [3:0] r_grant_orphan_cnt = 4'd0;
    wire w_orph_rel = r_grant_orphan_cnt == 4'd15 && w_ddr_fe_idle;
    wire w_grant_orphaned = (r_grant_blit && !i_dma_req && !i_dma_done) ||
                            (r_grant_c2   && !i_c2_req  && !i_c2_done)  ||
                            (r_grant_vga  && !i_vga_req && !i_vga_done) ||
                            (r_grant_sd   && !i_sd_req  && !i_sd_done);
    wire w_no_grant = !r_grant_blit && !r_grant_c2 && !r_grant_vga && !r_grant_sd;
    wire w_vga_gate = !is_miss_path && (state != MAINT || r_mnt_sub == MS_READ);
    assign o_vga_yield = is_miss_path;

    always_ff @(posedge i_Clk) begin
        r_grant_orphan_cnt <= !w_grant_orphaned ? 4'd0
                            : (r_grant_orphan_cnt == 4'd15) ? 4'd15 : r_grant_orphan_cnt + 4'd1;
        if (w_no_grant) begin
            // o_mnt_busy (not just r_mnt_active): a REQUESTED flush/invalidate
            // must win over a new blit/core-2 tenure even before its walk has
            // started, or the walk's coherency point drifts behind a chunk.
            if (i_vga_req && w_vga_gate)
                r_grant_vga  <= 1'b1;
            else if (!is_miss_path && !o_mnt_busy) begin
                if (i_sd_req)      r_grant_sd   <= 1'b1;
                else if (i_dma_req) r_grant_blit <= 1'b1;
                else if (i_c2_req) r_grant_c2   <= 1'b1;
            end
        end else if (r_grant_vga) begin
            if (i_vga_done || (w_grant_orphaned && w_orph_rel))
                r_grant_vga <= 1'b0;
        end else if (r_grant_sd) begin
            if (i_sd_done || (w_grant_orphaned && w_orph_rel))
                r_grant_sd <= 1'b0;
        end else if (r_grant_blit) begin
            if (i_dma_done || (w_grant_orphaned && w_orph_rel))
                r_grant_blit <= 1'b0;
        end else begin
            if (i_c2_done || (w_grant_orphaned && w_orph_rel))
                r_grant_c2 <= 1'b0;
        end
    end

    initial begin
        o_cnt_read_hits    = 64'd0;
        o_cnt_read_misses  = 64'd0;
        o_cnt_write_hits   = 64'd0;
        o_cnt_write_misses = 64'd0;
        o_cnt_writebacks   = 64'd0;
        o_cnt_stall_cycles = 64'd0;
        o_cnt_zero_lines   = 64'd0;
    end

    always_ff @(posedge i_Clk) begin
        if (i_stat_clear) begin
            o_cnt_read_hits    <= 64'd0;
            o_cnt_read_misses  <= 64'd0;
            o_cnt_write_hits   <= 64'd0;
            o_cnt_write_misses <= 64'd0;
            o_cnt_writebacks   <= 64'd0;
            o_cnt_stall_cycles <= 64'd0;
            o_cnt_zero_lines   <= 64'd0;
        end else begin
            if (state == CHECK) begin
                if (!r_is_write &&  r_cache_hit)              o_cnt_read_hits    <= o_cnt_read_hits    + 64'd1;
                if (!r_is_write && !r_cache_hit)              o_cnt_read_misses  <= o_cnt_read_misses  + 64'd1;
                if ( r_is_write &&  r_cache_hit)              o_cnt_write_hits   <= o_cnt_write_hits   + 64'd1;
                if ( r_is_write && !r_cache_hit)              o_cnt_write_misses <= o_cnt_write_misses + 64'd1;
                if (!r_cache_hit && r_evict_dirty)            o_cnt_writebacks   <= o_cnt_writebacks   + 64'd1;
                if ( r_is_write &&  r_zero_line)              o_cnt_zero_lines   <= o_cnt_zero_lines   + 64'd1;
            end
            // M10a: count only CPU-VISIBLE miss stall — once the early restart
            // has released the CPU (r_miss_served), the remaining install /
            // shadow-writeback cycles overlap execution and are not stall.
            if (is_miss_path && !r_miss_served)               o_cnt_stall_cycles <= o_cnt_stall_cycles + 64'd1;
        end
    end

    // -------------------------------------------------------------------------
    // Clock wizard and DDR2 controller
    // -------------------------------------------------------------------------
    wire [1:0] mig_dw_off = r_grant_vga ? 2'b00 : r_dw_offset;   // VGA: dword 0, in-order

    clk_wiz_0 clk_wiz_0 (
        .i_Clk  (i_Clk_board),     // board oscillator (NOT i_Clk=ui_clk — that would be circular)
        .clk_200(sys_clk_i),
        .clk_50 (clk_50),
        .locked (),                // intentionally unused
        .resetn (resetn)
    );

    generate if (L2_EN) begin : g_l2
    l2_cache #(.WAYS(L2_WAYS)) l2_cache (
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
        .i_mem_wide       (mig_wide),        // 1 = cache 32 B (2 pipelined bursts); 0 = blitter 128 b
        .i_mem_dw_off     (mig_dw_off),      // CWF: requested dword (wide reads; VGA = 0)
        .i_mem_write_data (mig_write_data),
        .i_app_wdf_mask   (mig_wdf_mask),
        .o_mem_read_data  (i_ddr_mem_read_data),
        .o_mem_ready      (i_ddr_mem_ready),
        .o_mem_rd_dw      (w_ddr_rd_dw),
        .o_mem_rd_dw_next (w_ddr_rd_dw_next),
        .o_mem_dw_next_ok (w_ddr_dw_next_ok),
        .o_mem_dw_ready   (w_ddr_dw_ready),
        .o_calib_done     (o_calib_done),
        .o_ui_clk         (w_ui_clk),
        .i_dma            (r_grant_blit | r_grant_c2 | r_grant_vga | r_grant_sd),
        .i_ifetch         (r_is_ifetch),
        .i_rt_req         (i_vga_req),
        // the speculative lookup: this cache's request in its CHECK cycle
        .i_spec_v         (state == CHECK),
        .i_spec_addr      (r_computed_ddr_addr),
        .i_ctrl_wr        (i_l2_ctrl_wr),
        .i_ctrl_wdata     (i_l2_ctrl_wdata),
        .i_ctrl_en        (i_l2_ctrl_en),
        .o_en             (o_l2_en),
        .o_busy           (o_l2_busy),
        .o_ev             (o_l2_ev),
        .o_fe_idle        (w_ddr_fe_idle)
    );
    // L2_INFO, CACHE_INFO's format: 128 KiB, 64 B lines, sets, ways
    assign o_l2_info = {32'(128 * 1024), 8'd64, 16'(128 * 1024 / 64 / L2_WAYS), 8'(L2_WAYS)};
    end else begin : g_ddr
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
        .i_mem_wide       (mig_wide),        // 1 = cache 32 B (2 pipelined bursts); 0 = blitter 128 b
        .i_mem_dw_off     (mig_dw_off),      // CWF: requested dword (wide reads; VGA = 0)
        .i_mem_write_data (mig_write_data),
        .i_app_wdf_mask   (mig_wdf_mask),
        .o_mem_read_data  (i_ddr_mem_read_data),
        .o_mem_ready      (i_ddr_mem_ready),
        .o_mem_rd_dw      (w_ddr_rd_dw),
        .o_mem_rd_dw_next (w_ddr_rd_dw_next),
        .o_mem_dw_next_ok (w_ddr_dw_next_ok),
        .o_mem_dw_ready   (w_ddr_dw_ready),
        .o_calib_done     (o_calib_done),
        .o_ui_clk         (w_ui_clk),
        .o_fe_idle        (w_ddr_fe_idle)
    );
    assign o_l2_en   = 1'b0;
    assign o_l2_busy = 1'b0;
    assign o_l2_info = 64'h0;
    assign o_l2_ev   = '0;
    end endgenerate

endmodule

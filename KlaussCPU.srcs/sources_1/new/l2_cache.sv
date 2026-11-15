`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// l2_cache — unified L2 between mem_read_write's DDR arbiter and the MIG UI
// (HW_PERF_PLAN.md §6).
//
// It replaces ddr2_control as the arbiter's target: the same front port
// (DV levels, a ready pulse, the CWF early-dword channel), plus which master
// owns the request (i_dma: blitter / core 2 / VGA; else the D-cache, with
// I-fills marked by i_ifetch).  Every DDR master reaches DDR only through
// here, so the L2 + DDR behave as one memory for all of them (DMA coherence:
// DMA is routed through the L2, non-allocating).
//
// Geometry: L2_KIB KiB, 64 B lines of two 32 B sectors (valid and dirty bit
// per sector), WAYS ways (2 / 4 / 8), tree pseudo-LRU, physically indexed and
// tagged on PA[26:6] (128 MiB of DDR), write-back,
// non-inclusive.  Arrays:
//   data : one memory per way, (SETS*2) x 256 b, a 32 B sector per word in
//          the D-cache's line layout (dword j at [255-64j -: 64]); port A
//          reads all ways at {set, sector} (BRAM latch + output register),
//          port B writes one way with byte enables;
//   tags : one word per set {plru, way[WAYS-1] .. way[0]}, way = {tag, V[1:0],
//          D[1:0]}, tag = PA[26:6+SETW] (the whole 128 MiB of DDR; an access
//          with PA[31:27] != 0 bypasses the L2); port A read (no output
//          register), port B write;
//   write buffer: 2 entries of 64 B + a 64-bit byte mask (LUTRAM data).
//
// Front end (one request at a time; every write is posted — acknowledged at
// its accept edge): a lookup reads port A; while idle the read address is
// the D-cache's request in its CHECK cycle (i_spec_*), so a D-cache miss
// finds its tags already read when its DV arrives: tags compared that
// cycle, the sector muxed out of the data output register the next, and
// o_mem_ready + the 32 B sector on the edge after (3 cycles from the
// D-cache's miss decision to the core).
//
// DDR engine (one transaction at a time): a read of 1, 2 or 4 BL8 bursts,
// critical burst first (the requester's answer when its sector — or burst —
// is in; installs into the data array when allocating), or the drain of
// the write buffer's oldest entry.  A demand read starts only when no
// write-buffer entry holds its line (drained first: Strict MIG ordering
// then makes the read see it) and, for an allocation with a dirty victim,
// when an entry is free.
//////////////////////////////////////////////////////////////////////////////

module l2_cache #(
    parameter int WAYS   = 4,
    parameter int L2_KIB = 128
) (
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
    input         resetn,
    input         sys_clk_i,

    // front port — ddr2_control's contract
    input             i_mem_write_DV,
    input             i_mem_read_DV,
    input      [31:0] i_mem_addr,
    input             i_mem_wide,       // 1 = 32 B (D-cache, VGA); 0 = 16 B (blitter, core 2)
    input      [ 1:0] i_mem_dw_off,     // CWF: the requested dword (wide reads)
    input     [255:0] i_mem_write_data,
    input      [15:0] i_app_wdf_mask,   // narrow writes: 1 = byte not written
    output logic [255:0] o_mem_read_data,
    output logic      o_mem_ready,
    output logic [63:0] o_mem_rd_dw,
    output logic [63:0] o_mem_rd_dw_next,
    output logic      o_mem_dw_next_ok,
    output logic      o_mem_dw_ready,
    output            o_calib_done,
    output            o_ui_clk,

    input             i_dma,            // the request is a DMA master's (not the D-cache's)
    input             i_ifetch,         // a D-cache read for an I-cache fill
    // speculative lookup: the D-cache's request in its CHECK cycle
    input             i_spec_v,
    input      [31:0] i_spec_addr,
    // control (L2_CTRL): a write, or the CPU reset / program-load enable
    input             i_ctrl_wr,
    input      [ 1:0] i_ctrl_wdata,     // [0] EN, [1] WBINV
    input             i_ctrl_en,
    output            o_en,             // L2_CTRL[0]
    output            o_busy,           // L2_CTRL[8]
    // perf events (one-cycle strobes): see the assigns at the end
    output logic [10:0] o_ev
);

    // ---------------------------------------------------------------------
    // geometry
    // ---------------------------------------------------------------------
    localparam int SETS  = L2_KIB * 1024 / 64 / WAYS;
    localparam int SETW  = $clog2(SETS);
    localparam int TAGW  = 21 - SETW;           // line address PA[26:6] = {tag, set}
    localparam int WW    = TAGW + 4;            // per way: {tag, V[1:0], D[1:0]}
    localparam int PB    = WAYS - 1;            // PLRU bits (tree nodes 1 .. WAYS-1)
    localparam int TWW   = WAYS * WW + PB;
    localparam int WAYW  = (WAYS > 1) ? $clog2(WAYS) : 1;

    // ---------------------------------------------------------------------
    // MIG
    // ---------------------------------------------------------------------
    wire        calib_done;
    wire        ui_clk;
    wire        ui_rst;
    assign o_calib_done = calib_done;
    assign o_ui_clk     = ui_clk;
    wire        clk = ui_clk;

    logic [26:0] app_addr = '0;
    logic [ 2:0] app_cmd  = '0;
    logic        app_en   = 1'b0;
    wire         app_rdy;
    logic [63:0] app_wdf_data = '0;
    logic        app_wdf_end  = 1'b0;
    logic        app_wdf_wren = 1'b0;
    logic [ 7:0] app_wdf_mask = '0;
    wire         app_wdf_rdy;
    wire  [63:0] app_rd_data;
    wire         app_rd_data_end;
    wire         app_rd_data_valid;
    wire         app_sr_active, app_ref_ack, app_zq_ack;

    mig_7series_0 mig_7series_0 (
        .ddr2_addr (ddr2_addr), .ddr2_ba (ddr2_ba), .ddr2_cas_n (ddr2_cas_n),
        .ddr2_ck_n (ddr2_ck_n), .ddr2_ck_p (ddr2_ck_p), .ddr2_cke (ddr2_cke),
        .ddr2_ras_n (ddr2_ras_n), .ddr2_we_n (ddr2_we_n), .ddr2_dq (ddr2_dq),
        .ddr2_dqs_n (ddr2_dqs_n), .ddr2_dqs_p (ddr2_dqs_p), .ddr2_cs_n (ddr2_cs_n),
        .ddr2_dm (ddr2_dm), .ddr2_odt (ddr2_odt),
        .init_calib_complete (calib_done),
        .app_addr (app_addr), .app_cmd (app_cmd), .app_en (app_en),
        .app_wdf_data (app_wdf_data), .app_wdf_end (app_wdf_end), .app_wdf_wren (app_wdf_wren),
        .app_rd_data (app_rd_data), .app_rd_data_end (app_rd_data_end),
        .app_rd_data_valid (app_rd_data_valid), .app_rdy (app_rdy), .app_wdf_rdy (app_wdf_rdy),
        .app_sr_req (1'b0), .app_ref_req (1'b0), .app_zq_req (1'b0),
        .app_sr_active (app_sr_active), .app_ref_ack (app_ref_ack), .app_zq_ack (app_zq_ack),
        .ui_clk (ui_clk), .ui_clk_sync_rst (ui_rst), .app_wdf_mask (app_wdf_mask),
        .sys_clk_i (sys_clk_i), .sys_rst (resetn)
    );

    localparam logic [2:0] CMD_WRITE = 3'b000;
    localparam logic [2:0] CMD_READ  = 3'b001;

    // ---------------------------------------------------------------------
    // PLRU (tree; node i's bit set = its LRU side is the right half)
    // ---------------------------------------------------------------------
    function automatic logic [PB-1:0] f_touch(input logic [PB-1:0] b, input int w);
        logic [WAYS-1:0] t;   // t[node], nodes 1 .. WAYS-1
        int node, lo, span;
        t = '0;
        for (int i = 1; i < WAYS; i++) t[i] = b[i-1];
        node = 1; lo = 0; span = WAYS;
        for (int lv = 0; lv < WAYW; lv++) begin
            span = span / 2;
            if (w >= lo + span) begin t[node] = 1'b0; node = 2 * node + 1; lo = lo + span; end
            else                begin t[node] = 1'b1; node = 2 * node; end
        end
        for (int i = 1; i < WAYS; i++) f_touch[i-1] = t[i];
    endfunction

    function automatic int f_plru_vic(input logic [PB-1:0] b);
        logic [WAYS-1:0] t;
        int node, lo, span;
        t = '0;
        for (int i = 1; i < WAYS; i++) t[i] = b[i-1];
        node = 1; lo = 0; span = WAYS;
        for (int lv = 0; lv < WAYW; lv++) begin
            span = span / 2;
            if (t[node]) begin node = 2 * node + 1; lo = lo + span; end
            else           node = 2 * node;
        end
        return lo;
    endfunction

    // ---------------------------------------------------------------------
    // arrays
    // ---------------------------------------------------------------------
    (* ram_style = "block" *) logic [TWW-1:0] tag_ram [SETS];
    logic [SETW-1:0] ta_addr;          // port A address (comb)
    logic [TWW-1:0]  ta_q;             // port A read (BRAM latch)
    logic            tb_we;
    logic [SETW-1:0] tb_addr;
    logic [TWW-1:0]  tb_din;

    initial for (int i = 0; i < SETS; i++) tag_ram[i] = '0;
    always_ff @(posedge clk) ta_q <= tag_ram[ta_addr];
    always_ff @(posedge clk) if (tb_we) tag_ram[tb_addr] <= tb_din;

    logic [SETW:0]    da_addr;         // port A {set, sector} (comb)
    logic [255:0]     da_q [WAYS];     // port A output register (DO_REG)
    logic [WAYS-1:0]  db_we;           // port B: one way
    logic [SETW:0]    db_addr;
    logic [255:0]     db_din;
    logic [31:0]      db_be;

    genvar gw;
    generate for (gw = 0; gw < WAYS; gw++) begin : g_way
        (* ram_style = "block" *) logic [255:0] dram [SETS*2];
        logic [255:0] lat;
        always_ff @(posedge clk) begin
            lat       <= dram[da_addr];
            da_q[gw]  <= lat;
        end
        always_ff @(posedge clk) begin
            if (db_we[gw])
                for (int i = 0; i < 32; i++)
                    if (db_be[i]) dram[db_addr][i*8 +: 8] <= db_din[i*8 +: 8];
        end
    end endgenerate

    // write buffer data: entry e, sector s at {e, s}
    (* ram_style = "distributed" *) logic [255:0] wbd [4];
    logic         wbd_we;
    logic [1:0]   wbd_wa;
    logic [255:0] wbd_din;
    logic [1:0]   wbd_ra;
    always_ff @(posedge clk) if (wbd_we) wbd[wbd_wa] <= wbd_din;
    wire  [255:0] wbd_q = wbd[wbd_ra];

    // ---------------------------------------------------------------------
    // tag word decode (port A read of the address in ra_addr)
    // ---------------------------------------------------------------------
    logic [31:0] ra_addr = '0;         // the address read on the last edge
    logic        ra_ok   = 1'b0;       // ... while idle (no tag write since)
    wire  [TAGW-1:0] ra_tag = ra_addr[26:6+SETW];
    wire             ra_sec = ra_addr[5];

    logic [TAGW-1:0] t_tag [WAYS];
    logic [1:0]      t_v   [WAYS];
    logic [1:0]      t_d   [WAYS];
    wire  [PB-1:0]   t_plru = ta_q[TWW-1 -: PB];
    logic [WAYS-1:0] t_match, t_inv;
    always_comb begin
        for (int w = 0; w < WAYS; w++) begin
            {t_tag[w], t_v[w], t_d[w]} = ta_q[w*WW +: WW];
            t_match[w] = (t_v[w] != 2'b00) && (t_tag[w] == ra_tag);
            t_inv[w]   = (t_v[w] == 2'b00);
        end
    end
    // victim: the first invalid way, else the PLRU's
    logic [WAYW-1:0] t_vic;
    always_comb begin
        t_vic = WAYW'(f_plru_vic(t_plru));
        for (int w = WAYS - 1; w >= 0; w--) if (t_inv[w]) t_vic = WAYW'(w);
    end
    logic [WAYW-1:0] t_mway;
    always_comb begin
        t_mway = '0;
        for (int w = 0; w < WAYS; w++) if (t_match[w]) t_mway = WAYW'(w);
    end

    // ---------------------------------------------------------------------
    // control state
    // ---------------------------------------------------------------------
    logic r_en       = 1'b1;     // allocate / look up (0 = bypass)
    logic r_en_tgt   = 1'b1;     // L2_CTRL[0] as written
    logic r_walking  = 1'b0;     // a write-back-and-invalidate walk (incl. its final drain)
    logic r_walk_end = 1'b0;     // every set done; waiting for the buffer to drain
    logic r_dis_pend = 1'b0;     // the walk ends in bypass (EN written 0)
    logic [SETW-1:0] r_walk_set = '0;
    assign o_en   = r_en_tgt;
    assign o_busy = r_walking;
    wire   w_alloc = r_en && !r_walking;

    // ---------------------------------------------------------------------
    // request
    // ---------------------------------------------------------------------
    logic         q_wr, q_wide, q_dma, q_if;
    logic         q_lk;                 // looked up: enabled and inside DDR (PA[31:27] = 0)
    logic [31:0]  q_addr;
    logic [1:0]   q_dwoff;
    logic [255:0] q_wdata;
    logic [31:0]  q_be;                 // byte enables of q_wdata in the sector word
    wire  [SETW-1:0] q_set  = q_addr[6 +: SETW];
    wire  [TAGW-1:0] q_tag  = q_addr[26:6+SETW];
    wire             q_sec  = q_addr[5];
    wire  [21:0]     q_line = q_addr[27:6];      // the DDR line (write buffer, MIG address)
    function automatic logic f_in_ddr(input logic [31:0] a);
        return a[31:27] == 5'd0;
    endfunction

    logic r_served = 1'b0;              // ready given; ignore DV until it drops
    wire  w_dv     = i_mem_read_DV | i_mem_write_DV;

    // decode registers (F_DEC)
    logic [TWW-1:0]  r_tw;
    logic            r_hit;             // the sector is valid in a matching way
    logic            r_many;            // the line is present (some sector)
    logic [WAYW-1:0] r_mway, r_vic;
    logic [WAYS-1:0] r_msel;            // data output mux (one-hot)

    // ---------------------------------------------------------------------
    // write buffer state
    // ---------------------------------------------------------------------
    logic [1:0]  wb_v   = '0;           // allocated
    logic [1:0]  wb_rdy = '0;           // data complete (may drain)
    logic [21:0] wb_line [2];
    logic [63:0] wb_m    [2];           // byte mask {sector1[31:0], sector0[31:0]}
    logic        wb_h    = 1'b0;        // head (oldest)
    wire  [1:0]  wb_n    = {1'b0, wb_v[0]} + {1'b0, wb_v[1]};
    wire         wb_full = (wb_v == 2'b11);
    wire         wb_tail = wb_v[wb_h] ? !wb_h : wb_h;
    function automatic logic f_wbm(input logic [21:0] ln);
        return (wb_v[0] && wb_line[0] == ln) || (wb_v[1] && wb_line[1] == ln);
    endfunction

    // ---------------------------------------------------------------------
    // DDR engine state
    // ---------------------------------------------------------------------
    localparam logic [1:0] E_IDLE = 2'd0, E_RD = 2'd1, E_WR = 2'd2;
    logic [1:0]  e_st = E_IDLE;
    logic [21:0] e_line;
    logic [1:0]  e_c;                   // first burst (pair index)
    logic [2:0]  e_n;                   // bursts: 1, 2, 4
    logic [2:0]  e_k;                   // commands issued
    logic [2:0]  e_bc;                  // beats received
    logic        e_alloc;
    logic [WAYW-1:0] e_way;
    logic        e_wide, e_early, e_rsp;   // e_rsp: the requester still waits for its answer
    logic [1:0]  e_dwoff;
    logic [255:0] e_buf;                // sector being assembled
    // drain
    logic [3:0]  e_bm;                  // bursts still to write
    logic [1:0]  e_p;                   // burst being written
    logic [1:0]  e_wph;                 // 0: present cmd + beat 0, 1: beat 1, 2: wait both accepted
    wire  [SETW-1:0] e_set = e_line[SETW-1:0];
    wire  e_filling = (e_st == E_RD) && e_alloc;

    // read beat
    wire  [1:0] rb_k = e_bc[2:1];
    wire  [1:0] rb_p = e_c ^ rb_k;
    wire        rb_h = rb_p[0];
    logic [255:0] rb_buf;               // e_buf with this beat merged
    always_comb begin
        rb_buf = e_buf;
        if (!e_bc[0]) rb_buf[191 - 128*rb_h -: 64] = app_rd_data;   // beat 0: the dword at +8
        else          rb_buf[255 - 128*rb_h -: 64] = app_rd_data;   // beat 1: the dword at +0
    end
    wire rb_sec_done = app_rd_data_valid && e_bc[0] && (e_n == 3'd1 || rb_k[0]);
    // the fill writes port B on this edge (has priority over the front end)
    wire eng_bwr = (e_st == E_RD) && e_alloc && rb_sec_done;

    // drain beat data
    wire [1:0]  dr_s  = {wb_h, e_p[1]};
    wire [127:0] dr_chunk = wbd_q[255 - 128*e_p[0] -: 128];
    wire [31:0] dr_m  = wb_m[wb_h][32*e_p[1] +: 32];
    wire [7:0]  dr_m0 = dr_m[16 - 16*e_p[0] +: 8];
    wire [7:0]  dr_m1 = dr_m[24 - 16*e_p[0] +: 8];
    function automatic logic [3:0] f_bursts(input logic [63:0] m);
        for (int p = 0; p < 4; p++) f_bursts[p] = |m[32*p[1] + 16 - 16*p[0] +: 16];
    endfunction
    wire [3:0] wb_hb = f_bursts(wb_m[wb_h]);
    function automatic logic [1:0] f_first(input logic [3:0] bm);
        f_first = bm[0] ? 2'd0 : bm[1] ? 2'd1 : bm[2] ? 2'd2 : 2'd3;
    endfunction

    // ---------------------------------------------------------------------
    // front-end FSM
    // ---------------------------------------------------------------------
    typedef enum logic [3:0] {
        F_IDLE, F_LOOK, F_DEC, F_VC0, F_VC1, F_VC2, F_WDAT, F_WRSP,
        F_WK0, F_WK1, F_WK2
    } fst_t;
    fst_t f_st = F_IDLE;
    fst_t f_ret;                         // after a victim / walk copy
    logic [WAYW-1:0] c_way;              // copy: the way
    logic [1:0]      c_sec;              // copy: its dirty sectors
    logic            c_e;                // copy: the write-buffer entry
    logic [SETW-1:0] c_set;

    // accept: a new request (not one already served), not to the set a fill
    // is writing
    wire w_acc_ok = calib_done && w_dv && !r_served &&
                    !(e_filling && i_mem_addr[6 +: SETW] == e_set);
    // the tags read on the last edge are this request's
    wire w_imm = ra_ok && (ra_addr[31:5] == i_mem_addr[31:5]);

    // port A address
    always_comb begin
        ta_addr = i_spec_addr[6 +: SETW];
        da_addr = {i_spec_addr[6 +: SETW], i_spec_addr[5]};
        case (f_st)
            F_IDLE: if (w_dv || !i_spec_v) begin
                        ta_addr = i_mem_addr[6 +: SETW];
                        da_addr = {i_mem_addr[6 +: SETW], i_mem_addr[5]};
                    end
            F_DEC, F_VC0, F_VC1, F_VC2: begin
                        ta_addr = c_set;
                        da_addr = {c_set, (f_st == F_DEC) ? 1'b0 : 1'b1};
                    end
            F_WK0, F_WK1, F_WK2: begin
                        ta_addr = r_walk_set;
                        da_addr = {r_walk_set, (f_st == F_WK2) ? 1'b0 : 1'b1};
                    end
            default: ;
        endcase
    end
    wire [31:0] ra_next = (f_st == F_IDLE && (w_dv || !i_spec_v)) ? i_mem_addr : i_spec_addr;

    // data output mux
    logic [255:0] d_mux;
    always_comb begin
        d_mux = '0;
        for (int w = 0; w < WAYS; w++) if (r_msel[w]) d_mux = d_mux | da_q[w];
    end

    // decoded r_tw fields
    logic [TAGW-1:0] r_ttag [WAYS];
    logic [1:0]      r_tv   [WAYS];
    logic [1:0]      r_td   [WAYS];
    always_comb for (int w = 0; w < WAYS; w++) {r_ttag[w], r_tv[w], r_td[w]} = r_tw[w*WW +: WW];
    wire [PB-1:0] r_plru = r_tw[TWW-1 -: PB];
    function automatic logic [TWW-1:0] f_setway(input logic [TWW-1:0] tw, input int w,
                                                input logic [TAGW-1:0] tg, input logic [1:0] v,
                                                input logic [1:0] d);
        f_setway = tw;
        f_setway[w*WW +: WW] = {tg, v, d};
    endfunction
    function automatic logic [TWW-1:0] f_plru_tw(input logic [TWW-1:0] tw, input int w);
        f_plru_tw = tw;
        f_plru_tw[TWW-1 -: PB] = f_touch(tw[TWW-1 -: PB], w);
    endfunction
    // walk: the first way of r_tw with a dirty sector
    logic            r_wk_any;
    logic [WAYW-1:0] r_wk_way;
    always_comb begin
        r_wk_any = 1'b0; r_wk_way = '0;
        for (int w = WAYS - 1; w >= 0; w--)
            if (r_td[w] != 2'b00) begin r_wk_any = 1'b1; r_wk_way = WAYW'(w); end
    end

    // the victim of an allocation (registered decode): its dirty sectors
    wire [1:0] r_vd = r_td[r_vic];
    wire       w_rd_req  = !q_wr;
    wire       w_rd_hit  = q_lk && r_hit;
    // a read miss that allocates: a cache read, looked up, allocation on
    wire       w_rd_alloc = w_alloc && q_lk && !q_dma;
    wire       w_rd_newline = w_rd_alloc && !r_many;
    wire       w_rd_needslot = w_rd_newline && (r_vd != 2'b00);
    wire       w_rd_can = (e_st == E_IDLE) && !f_wbm(q_line) && !(w_rd_needslot && wb_full);
    // writes
    wire       w_wr_cache_hit = q_lk && !q_dma && r_many;               // line present: write in place
    wire       w_wr_cache_alloc = w_alloc && q_lk && !q_dma && !r_many; // allocate the sector
    wire       w_wr_dma_hit = q_lk && q_dma && r_hit;
    wire       w_wr_post = !(w_wr_cache_hit || w_wr_cache_alloc || w_wr_dma_hit);

    // the first burst: the one holding the requested dword (a 32 B request
    // carries a line-aligned address and the dword in q_dwoff)
    wire [1:0] w_cburst = q_wide ? {q_addr[5], q_dwoff[1]} : q_addr[5:4];

    // events
    logic ev_i_hit, ev_i_miss, ev_d_hit, ev_d_miss, ev_w_hit, ev_w_miss,
          ev_rd_beat, ev_wr_beat, ev_wbfull, ev_dma_hit, ev_dma_miss;
    assign o_ev = {ev_dma_miss, ev_dma_hit, ev_wbfull, ev_wr_beat, ev_rd_beat,
                   ev_w_miss, ev_w_hit, ev_d_miss, ev_d_hit, ev_i_miss, ev_i_hit};

    // ---------------------------------------------------------------------
    // the FSMs (front end, write buffer, DDR engine)
    // ---------------------------------------------------------------------
    // Port B / write-buffer / tag-write controls are computed combinationally
    // from the state (below) so the memories keep one write site each.
    always_comb begin
        tb_we = 1'b0; tb_addr = q_set; tb_din = r_tw;
        db_we = '0; db_addr = {q_set, q_sec}; db_din = q_wdata; db_be = q_be;
        wbd_we = 1'b0; wbd_wa = {wb_tail, q_sec}; wbd_din = q_wdata;
        if (eng_bwr) begin
            db_we = WAYS'(1) << e_way;
            db_addr = {e_set, rb_p[1]};
            db_din = rb_buf;
            db_be = '1;
        end
        case (f_st)
            F_DEC: begin
                if (!q_wr) begin
                    if (w_rd_hit) begin
                        if (!q_dma) begin
                            tb_we = 1'b1; tb_din = f_plru_tw(r_tw, r_mway);
                        end
                    end else if (w_rd_alloc && w_rd_can) begin
                        tb_we = 1'b1;
                        tb_din = r_many ? f_plru_tw(f_setway(r_tw, r_mway, q_tag,
                                                           r_tv[r_mway] | (q_sec ? 2'b10 : 2'b01),
                                                           r_td[r_mway]), r_mway)
                                        : f_plru_tw(f_setway(r_tw, r_vic, q_tag, 2'b11, 2'b00), r_vic);
                    end
                end else if (!eng_bwr) begin
                    if (w_wr_cache_hit) begin
                        tb_we = 1'b1;
                        tb_din = f_plru_tw(f_setway(r_tw, r_mway, q_tag,
                                                    r_tv[r_mway] | (q_sec ? 2'b10 : 2'b01),
                                                    r_td[r_mway] | (q_sec ? 2'b10 : 2'b01)), r_mway);
                        db_we = WAYS'(1) << r_mway;
                    end else if (w_wr_cache_alloc && !(r_vd != 2'b00 && wb_full)) begin
                        tb_we = 1'b1;
                        tb_din = f_plru_tw(f_setway(r_tw, r_vic, q_tag, q_sec ? 2'b10 : 2'b01,
                                                    q_sec ? 2'b10 : 2'b01), r_vic);
                        if (r_vd == 2'b00) db_we = WAYS'(1) << r_vic;    // else after the copy (F_WDAT)
                    end else if (w_wr_dma_hit) begin
                        tb_we = 1'b1;
                        tb_din = f_setway(r_tw, r_mway, q_tag, r_tv[r_mway],
                                          r_td[r_mway] | (q_sec ? 2'b10 : 2'b01));
                        db_we = WAYS'(1) << r_mway;
                    end else if (w_wr_post && !wb_full) begin
                        wbd_we = 1'b1;
                    end
                end
            end
            F_WDAT: if (!eng_bwr) db_we = WAYS'(1) << r_vic;
            F_VC1: begin wbd_we = 1'b1; wbd_wa = {c_e, 1'b0}; wbd_din = d_mux; end
            F_VC2: begin wbd_we = 1'b1; wbd_wa = {c_e, 1'b1}; wbd_din = d_mux; end
            F_WK2: if (!r_wk_any) begin tb_we = 1'b1; tb_addr = r_walk_set; tb_din = '0; end
            default: ;
        endcase
        wbd_ra = (e_st == E_WR) ? dr_s : {wb_h, 1'b0};
    end

    always_ff @(posedge clk) begin
        o_mem_ready    <= 1'b0;
        o_mem_dw_ready <= 1'b0;
        {ev_i_hit, ev_i_miss, ev_d_hit, ev_d_miss, ev_w_hit, ev_w_miss,
         ev_rd_beat, ev_wr_beat, ev_wbfull, ev_dma_hit, ev_dma_miss} <= '0;
        if (!w_dv) r_served <= 1'b0;
        ra_addr <= ra_next;
        ra_ok   <= (f_st == F_IDLE) && !eng_bwr && f_in_ddr(ra_next) && r_en;

        // ---------------- control ----------------
        // L2_CTRL writes are ignored while a walk runs (BUSY); the CPU reset
        // / program-load enable is not: it cancels a pending disable (the
        // walk then ends as a plain write-back-and-invalidate).
        if (i_ctrl_en) begin
            r_en_tgt   <= 1'b1;
            r_dis_pend <= 1'b0;
            if (!r_walking) r_en <= 1'b1;
        end else if (i_ctrl_wr && !r_walking) begin
            r_en_tgt <= i_ctrl_wdata[0];
            if (!i_ctrl_wdata[0] || i_ctrl_wdata[1]) begin
                // 0: write back, invalidate, then bypass; 3: write back and
                // invalidate, stay enabled.  A disabled L2 holds no line.
                if (r_en) begin
                    r_walking  <= 1'b1;
                    r_walk_end <= 1'b0;
                    r_walk_set <= '0;
                    r_dis_pend <= !i_ctrl_wdata[0];
                end else if (i_ctrl_wdata[0]) r_en <= 1'b1;
            end else r_en <= 1'b1;   // 1: enable (instant)
        end

        // ---------------- front end ----------------
        case (f_st)
            F_IDLE: begin
                if (w_acc_ok) begin
                    q_wr    <= i_mem_write_DV;
                    q_addr  <= i_mem_addr;
                    q_wide  <= i_mem_wide;
                    q_dma   <= i_dma;
                    q_if    <= i_ifetch && !i_dma;
                    q_dwoff <= i_mem_dw_off;
                    q_wdata <= i_mem_wide ? i_mem_write_data
                                          : {i_mem_write_data[127:0], i_mem_write_data[127:0]};
                    q_be    <= i_mem_wide ? 32'hFFFF_FFFF
                                          : (i_mem_addr[4] ? {16'h0, ~i_app_wdf_mask}
                                                           : {~i_app_wdf_mask, 16'h0});
                    c_set   <= i_mem_addr[6 +: SETW];
                    q_lk    <= r_en && f_in_ddr(i_mem_addr);
                    if (i_mem_write_DV) begin
                        o_mem_ready <= 1'b1;          // posted
                        r_served    <= 1'b1;
                    end
                    if (!r_en || !f_in_ddr(i_mem_addr)) begin
                        // bypass (disabled, or outside DDR): no lookup
                        r_hit <= 1'b0; r_many <= 1'b0;
                        f_st  <= F_DEC;
                    end else if (w_imm) begin
                        r_tw   <= ta_q;
                        r_hit  <= |t_match && t_v[t_mway][ra_sec];
                        r_many <= |t_match;
                        r_mway <= t_mway;
                        r_vic  <= t_vic;
                        r_msel <= t_match;
                        f_st   <= F_DEC;
                    end else begin
                        f_st   <= F_LOOK;
                    end
                end else if (r_walking && !r_walk_end && !e_filling) begin
                    f_st <= F_WK0;
                end else if (r_walk_end && wb_v == 2'b00 && e_st == E_IDLE) begin
                    r_walking  <= 1'b0;
                    r_walk_end <= 1'b0;
                    if (r_dis_pend) begin r_en <= 1'b0; r_dis_pend <= 1'b0; end
                end
            end

            F_LOOK: begin
                r_tw   <= ta_q;
                r_hit  <= |t_match && t_v[t_mway][ra_sec];
                r_many <= |t_match;
                r_mway <= t_mway;
                r_vic  <= t_vic;
                r_msel <= t_match;
                f_st   <= F_DEC;
            end

            F_DEC: begin
                if (!q_wr) begin
                    if (w_rd_hit) begin
                        // read hit: the sector from the data output register
                        o_mem_read_data <= q_wide ? d_mux
                                                  : {2{d_mux[255 - 128*q_addr[4] -: 128]}};
                        o_mem_ready <= 1'b1;
                        r_served    <= 1'b1;
                        if (q_dma) ev_dma_hit <= 1'b1;
                        else if (q_if) ev_i_hit <= 1'b1;
                        else ev_d_hit <= 1'b1;
                        f_st <= F_IDLE;
                    end else if (w_rd_can) begin
                        if (q_lk) begin
                            if (q_dma) ev_dma_miss <= 1'b1;
                            else if (q_if) ev_i_miss <= 1'b1;
                            else ev_d_miss <= 1'b1;
                        end
                        // the DDR read (started below, in the engine section)
                        if (w_rd_needslot) begin
                            // copy the victim's dirty sectors to the buffer
                            c_way <= r_vic; c_sec <= r_vd; c_e <= wb_tail;
                            r_msel <= WAYS'(1) << r_vic;
                            wb_v[wb_tail]    <= 1'b1;
                            wb_rdy[wb_tail]  <= 1'b0;
                            wb_line[wb_tail] <= {1'b0, r_ttag[r_vic], q_set};
                            wb_m[wb_tail]    <= {{32{r_vd[1]}}, {32{r_vd[0]}}};
                            f_ret <= F_WRSP;
                            f_st  <= F_VC0;
                        end else begin
                            f_st <= F_WRSP;
                        end
                    end else if (w_rd_needslot && wb_full) begin
                        ev_wbfull <= 1'b1;
                    end
                end else if (!eng_bwr) begin
                    if (w_wr_cache_hit) begin
                        ev_w_hit  <= r_hit;
                        ev_w_miss <= !r_hit;
                        f_st <= F_IDLE;
                    end else if (w_wr_cache_alloc) begin
                        if (r_vd == 2'b00) begin
                            ev_w_miss <= 1'b1;
                            f_st <= F_IDLE;
                        end else if (!wb_full) begin
                            ev_w_miss <= 1'b1;
                            c_way <= r_vic; c_sec <= r_vd; c_e <= wb_tail;
                            r_msel <= WAYS'(1) << r_vic;
                            wb_v[wb_tail]    <= 1'b1;
                            wb_rdy[wb_tail]  <= 1'b0;
                            wb_line[wb_tail] <= {1'b0, r_ttag[r_vic], q_set};
                            wb_m[wb_tail]    <= {{32{r_vd[1]}}, {32{r_vd[0]}}};
                            f_ret <= F_WDAT;
                            f_st  <= F_VC0;
                        end else ev_wbfull <= 1'b1;
                    end else if (w_wr_dma_hit) begin
                        ev_dma_hit <= 1'b1;
                        f_st <= F_IDLE;
                    end else if (!wb_full) begin
                        // post to DDR through the write buffer
                        if (q_lk) begin
                            if (q_dma) ev_dma_miss <= 1'b1;
                            else       ev_w_miss   <= 1'b1;
                        end
                        wb_v[wb_tail]    <= 1'b1;
                        wb_rdy[wb_tail]  <= 1'b1;
                        wb_line[wb_tail] <= q_line;
                        wb_m[wb_tail]    <= q_sec ? {q_be, 32'h0} : {32'h0, q_be};
                        f_st <= F_IDLE;
                    end else ev_wbfull <= 1'b1;
                end
            end

            // victim / walk copy: the port A reads of sectors 0 and 1 were
            // issued on the edges into F_VC0 and F_VC1
            F_VC0: f_st <= F_VC1;
            F_VC1: f_st <= F_VC2;
            F_VC2: begin
                wb_rdy[c_e] <= 1'b1;
                f_st <= f_ret;
            end

            F_WDAT: if (!eng_bwr) f_st <= F_IDLE;   // the allocated sector's data

            F_WRSP: if (!e_rsp) f_st <= F_IDLE;

            // ---------------- write-back-and-invalidate walk ----------------
            F_WK0: f_st <= F_WK1;                   // reads r_walk_set's tags on its edge
            F_WK1: begin
                r_tw <= ta_q;
                f_st <= F_WK2;
            end
            F_WK2: begin
                if (r_wk_any) begin
                    if (!wb_full) begin
                        c_way <= r_wk_way; c_sec <= r_td[r_wk_way]; c_e <= wb_tail;
                        c_set <= r_walk_set;
                        r_msel <= WAYS'(1) << r_wk_way;
                        wb_v[wb_tail]    <= 1'b1;
                        wb_rdy[wb_tail]  <= 1'b0;
                        wb_line[wb_tail] <= {1'b0, r_ttag[r_wk_way], r_walk_set};
                        wb_m[wb_tail]    <= {{32{r_td[r_wk_way][1]}}, {32{r_td[r_wk_way][0]}}};
                        r_tw[r_wk_way*WW +: 2] <= 2'b00;   // its dirty bits: copied
                        f_ret <= F_WK2;
                        f_st  <= F_VC0;
                    end else begin
                        ev_wbfull <= 1'b1;
                        // yield to a request while the buffer drains; the
                        // set is walked again (a way copied already is
                        // copied again: a later entry, so drained after)
                        if (w_acc_ok) f_st <= F_IDLE;
                    end
                end else begin
                    // (tag word written 0 by the comb block)
                    if (r_walk_set == SETW'(SETS - 1)) r_walk_end <= 1'b1;
                    r_walk_set <= r_walk_set + 1'b1;
                    f_st <= F_IDLE;
                end
            end
            default: f_st <= F_IDLE;
        endcase

        // ---------------- DDR engine ----------------
        if (app_en && app_rdy) begin
            if (e_st == E_RD) begin
                if (e_k + 3'd1 < e_n) begin
                    e_k      <= e_k + 3'd1;
                    app_addr <= {e_line, e_c ^ 2'(e_k + 3'd1), 3'b000};
                end else app_en <= 1'b0;
            end else app_en <= 1'b0;
        end

        case (e_st)
            E_IDLE: begin
                if (f_st == F_DEC && !q_wr && !w_rd_hit && w_rd_can) begin
                    // start the read: the first burst holds the requested dword
                    e_st    <= E_RD;
                    e_line  <= q_line;
                    e_c     <= w_cburst;
                    e_alloc <= w_rd_alloc;
                    e_way   <= r_many ? r_mway : r_vic;
                    e_n     <= !q_wide ? 3'd1 : (w_rd_newline ? 3'd4 : 3'd2);
                    e_k     <= 3'd0;
                    e_bc    <= 3'd0;
                    e_wide  <= q_wide;
                    e_early <= q_wide && !q_dma;
                    e_dwoff <= q_dwoff;
                    e_rsp   <= 1'b1;
                    app_en   <= 1'b1;
                    app_cmd  <= CMD_READ;
                    app_addr <= {q_line, w_cburst, 3'b000};
                end else if (wb_v[wb_h] && wb_rdy[wb_h] && !(f_st == F_DEC && !q_wr && !w_rd_hit && w_rd_can)) begin
                    e_st  <= E_WR;
                    e_bm  <= wb_hb;
                    e_p   <= f_first(wb_hb);
                    e_wph <= 2'd0;
                end
            end

            E_RD: if (app_rd_data_valid) begin
                ev_rd_beat <= 1'b1;
                e_bc  <= e_bc + 3'd1;
                e_buf <= rb_buf;
                if (e_early && rb_k == 2'd0) begin
                    if (e_bc[0] == !e_dwoff[0]) begin
                        o_mem_rd_dw    <= app_rd_data;
                        o_mem_dw_ready <= 1'b1;
                    end
                    if (!e_bc[0] && !e_dwoff[0]) o_mem_rd_dw_next <= app_rd_data;
                    o_mem_dw_next_ok <= !e_dwoff[0];
                end
                if (rb_sec_done && e_rsp) begin
                    // the requester's sector (or burst) is in
                    o_mem_read_data <= e_wide ? rb_buf : {2{rb_buf[255 - 128*rb_h -: 128]}};
                    o_mem_ready <= 1'b1;
                    r_served    <= 1'b1;
                    e_rsp       <= 1'b0;
                end
                if (e_bc + 3'd1 == {e_n, 1'b0}) e_st <= E_IDLE;
            end

            E_WR: begin
                // per burst: command + beat 0 together, then beat 1
                case (e_wph)
                    2'd0: if (app_rdy && app_wdf_rdy && !app_en && !app_wdf_wren) begin
                        app_en       <= 1'b1;
                        app_cmd      <= CMD_WRITE;
                        app_addr     <= {wb_line[wb_h], e_p, 3'b000};
                        app_wdf_data <= dr_chunk[63:0];
                        app_wdf_mask <= ~dr_m0;
                        app_wdf_wren <= 1'b1;
                        app_wdf_end  <= 1'b0;
                        e_wph        <= 2'd1;
                    end
                    2'd1: if (app_wdf_rdy) begin            // beat 0 accepted: beat 1
                        ev_wr_beat   <= 1'b1;
                        app_wdf_data <= dr_chunk[127:64];
                        app_wdf_mask <= ~dr_m1;
                        app_wdf_end  <= 1'b1;
                        e_wph        <= 2'd2;
                    end
                    default: begin
                        if (app_wdf_wren && app_wdf_rdy) begin
                            ev_wr_beat   <= 1'b1;
                            app_wdf_wren <= 1'b0;
                            app_wdf_end  <= 1'b0;
                        end
                        if (!(app_wdf_wren && !app_wdf_rdy) && !(app_en && !app_rdy)) begin
                            // both accepted (or accepted on this edge)
                            if ((e_bm & ~(4'b1 << e_p)) == 4'b0) begin
                                wb_v[wb_h]   <= 1'b0;
                                wb_rdy[wb_h] <= 1'b0;
                                wb_h         <= !wb_h;
                                e_st         <= E_IDLE;
                            end else begin
                                e_bm  <= e_bm & ~(4'b1 << e_p);
                                e_p   <= f_first(e_bm & ~(4'b1 << e_p));
                                e_wph <= 2'd0;
                            end
                        end
                    end
                endcase
            end
            default: e_st <= E_IDLE;
        endcase

        if (ui_rst) begin
            f_st <= F_IDLE; e_st <= E_IDLE;
            app_en <= 1'b0; app_wdf_wren <= 1'b0; app_wdf_end <= 1'b0;
            wb_v <= '0; wb_rdy <= '0; e_rsp <= 1'b0;
        end
    end

`ifndef SYNTHESIS
    // a fill must not overwrite a victim sector before it was copied
    always_ff @(posedge clk)
        if (eng_bwr && (f_st == F_VC0 || f_st == F_VC1) && c_set == e_set && c_way == e_way)
            $fatal(1, "l2_cache: fill write overtook the victim copy (set %0d way %0d)", e_set, e_way);
    // the D-cache always writes whole sectors
    always_ff @(posedge clk)
        if (f_st == F_IDLE && w_acc_ok && i_mem_write_DV && i_mem_wide && i_app_wdf_mask != 16'h0)
            $fatal(1, "l2_cache: masked wide write");
`endif
endmodule

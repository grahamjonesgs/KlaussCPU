// ============================================================================
// core2_subsys — AMP core 2: a second pipeline_core at effective 50 MHz.
//
// P1 scope (AMP_CORE2_PLAN.md): the core + a 64 KB local BRAM (code+data for
// now; P2 re-partitions when DDR arrives) + a log FIFO, controlled by core 1
// through MMIO device 0x010.  No DDR access, no interrupts, no LiteEth yet.
//
// Clocking: everything core-2-side advances on r_ce (a /2 toggle), making the
// subsystem a logically-50 MHz synchronous system on the 100 MHz clock — the
// M12 Stage-C SHA pattern.  The XDC gives the pipeline_core instance a blanket
// 2-cycle multicycle budget.  The local RAM and log FIFO run full-rate (their
// inputs from the CE side are stable across each 2-cycle window; pushes are
// qualified with r_ce so they fire exactly once per core cycle).
//
// Rules honoured (plan §3): every core-2-facing handshake is level-held —
// m2_ready is generated ON the CE grid (one core cycle), never a bare 100 MHz
// pulse; start is held until consumed on a CE edge.
//
// Core-2 address map (P1):
//   0x0000_0000 +128K  local RAM (klausscc images link at 0 — runs unmodified)
//   0xE000_0000 +128K  the same RAM at its final planned alias
//   0xF001_xxxx        UART-COMPATIBLE console, backed by the log FIFO:
//                      0x0000 TX_DATA (W: push byte), 0x0008 RX_DATA (reads
//                      0), 0x0010 STATUS ([0]=tx_busy=fifo full, [1]=rx_empty
//                      =1, [2]=rx_full=0).  Any baremetal program's console
//                      output (printf included) works on core 2 unmodified.
//   all else           reads 0, writes acked+dropped (never hangs the bus)
//
// Core-1 mailbox registers (MMIO device 0x010, 64-bit spaced):
//   0x0000 C2_CTRL     RW  [0]=RUN (0 holds core-2 reset).  Read adds
//                          [1]=parked, [4:2]=park_kind, [8]=start pending.
//   0x0008 C2_START_PC RW  core-2 start PC (used on RUN 0→1; typically 0x20)
//   0x0010 C2_WIN_ADDR RW  window address (core-2 space).  Writing triggers a
//                          background prefetch of ram[addr] into C2_WIN_DATA.
//   0x0018 C2_WIN_DATA RW  W: ram[WIN_ADDR] <= data, WIN_ADDR += 8 (the load
//                          path).  R: the prefetched dword for the last
//                          WIN_ADDR write (readback/verify path).
//                          Window ops are only legal while RUN=0.
//   0x0020 C2_LOG      R   [8]=valid, [7:0]=head byte (NON-destructive).
//                      W   pop one byte.
//   0x0028 C2_LOG_CNT  R   bytes in the log FIFO.
//   0x0030 C2_PARK_PC  R   core-2 park PC (valid while parked).
//   0x0038 C2_ETH_OWNER RW [0]=1: core 2 owns the LiteEth windows (0xF006/7/8);
//                          reset 0 = core 1 (boot-ROM netboot untouched).
//                          The non-owner's eth accesses read 0 / drop writes.
// Core-2 view additions (P3): LiteEth windows 0xF006/7/8 when owner;
// 0xF00F_0040 = core 1's clock_ms (read-only mirror, for lwIP sys_now).
// ============================================================================

module core2_subsys
   import klauss_pkg::*;
(
   input  logic i_Clk,
   input  logic i_Rst_L,
   mmio_if.slave mmio,

   // DDR master C (to mem_read_write's arbiter — blitter-shape contract:
   // req level → grant → one 128 b DV/ready burst → done releases).
   output logic         o_ddr_req,
   output logic         o_ddr_done,
   output logic         o_ddr_write_DV,
   output logic         o_ddr_read_DV,
   output logic [31:0]  o_ddr_addr,
   output logic [127:0] o_ddr_write_data,
   output logic [15:0]  o_ddr_wdf_mask,
   input        [127:0] i_ddr_read_data,
   input                i_ddr_ready,
   input                i_ddr_grant,

   // LiteEth window master (P3): eth-bridge-shape request; the top muxes
   // it against core 1's bus_splitter eth port under o_eth_owner.
   output logic         o_eth_owner,       // C2_ETH_OWNER[0]: 1 = core 2 owns
   output logic         o_eth_write_DV,
   output logic         o_eth_read_DV,
   output logic [31:0]  o_eth_addr,
   output logic [63:0]  o_eth_write_data,
   output logic [7:0]   o_eth_byte_en,
   input        [63:0]  i_eth_read_data,
   input                i_eth_ready,

   // Core 1's millisecond clock (0xF00F_0040), mirrored read-only into the
   // core-2 MMIO view so lwIP's sys_now() works unmodified.
   input        [63:0]  i_clock_ms
);
   localparam int RAM_ROWS = 8192;              // 8192 x 128b = 128 KB (P4)
   localparam int LOG_DEPTH = 512;

   assign mmio.ready = 1'b1;   // top-level dv_d2 protocol provides pacing

   // ------------------------------------------------------------------ ce/2
   logic r_ce;
   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L) r_ce <= 1'b0;
      else          r_ce <= ~r_ce;
   end

   // -------------------------------------------------- mailbox control regs
   logic        r_run;
   logic [31:0] r_start_pc;
   logic        r_start_pend;          // level until consumed on a CE edge
   logic [1:0]  r_start_dly;           // CE ticks after RUN rise before start

   // MMIO write strobes are level-held for several cycles: edge-detect once.
   logic r_wr_d;
   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L) r_wr_d <= 1'b0;
      else          r_wr_d <= mmio.write_DV;
   end
   wire w_wr_edge = mmio.write_DV && !r_wr_d;

   // --------------------------------------------------------- core-2 wires
   logic [31:0] c2_addr;
   logic        c2_read_DV, c2_write_DV;
   logic [63:0] c2_wdata;
   logic [7:0]  c2_be;
   logic [63:0] m2_rdata, m2_rdata_next;
   logic        m2_next_valid, m2_ready;
   wire         c2_parked;
   wire [2:0]   c2_park_kind;
   wire [31:0]  c2_park_pc, c2_park_op;
   logic [63:0] c2_dbg_r [0:15];
   logic [31:0] c2_dbg_sp;
   flags_t      c2_dbg_flags;

   pipeline_core #(.SP_RESET(32'h0002_0000)) c2_core_i (   // SP = top of local BRAM
      .clk        (i_Clk),
      .ce         (r_ce),
      .rst        (!i_Rst_L || !r_run),
      .start      (r_start_pend),
      .start_pc   (r_start_pc),
      .m_addr     (c2_addr),
      .m_read_DV  (c2_read_DV),
      .m_write_DV (c2_write_DV),
      .m_wdata    (c2_wdata),
      .m_be       (c2_be),
      .m_rdata    (m2_rdata),
      .m_rdata_next (m2_rdata_next),
      .m_next_valid (m2_next_valid),
      .m_ready    (m2_ready),
      .irq_ready  (1'b0),               // P1: no interrupts on core 2
      .irq_sel    (2'd0),
      .irq_vector (32'h0),
      .irq_ack    (),
      .irq_ack_sel(),
      .int_mask_o (),
      .mask_wr    (1'b0),
      .mask_wdata (4'h0),
      .lcd_byte   (),
      .lcd_dc     (),
      .lcd_dv     (),
      .lcd_rst_n  (),
      .lcd_rst_wr (),
      .lcd_ready  (1'b1),
      .bus_idle   (),
      .perf_stall (),
      .perf_br    (),
      .perf_br_taken (),
      .ret_valid  (),
      .ret_pc     (),
      .ret_op     (),
      .ret_wr     (),
      .ret_wr_addr(),
      .ret_wr_be  (),
      .ret_wr_raw (),
      .parked     (c2_parked),
      .park_kind  (c2_park_kind),
      .park_pc    (c2_park_pc),
      .park_op    (c2_park_op),
      .dbg_r      (c2_dbg_r),
      .dbg_sp     (c2_dbg_sp),
      .dbg_flags  (c2_dbg_flags)
   );

   // Start sequencing (CE grid): RUN 0→1 releases reset, waits 2 core cycles,
   // then holds start until the core has sampled it on one enabled edge.
   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L || !r_run) begin
         r_start_pend <= 1'b0;
         r_start_dly  <= 2'd0;
      end else if (r_ce) begin
         if (r_start_dly != 2'd2) begin
            r_start_dly <= r_start_dly + 2'd1;
            r_start_pend <= (r_start_dly == 2'd1);   // assert entering tick 2
         end else if (r_start_pend) begin
            r_start_pend <= 1'b0;                    // consumed exactly once
         end
      end
   end

   // ------------------------------------------------------------- local RAM
   // One physical port, muxed: the core-2 bus owns it while RUN, the mailbox
   // window while held in reset.  128-bit rows so a fetch can serve the next
   // sequential dword (m_rdata_next) from the same row.
   logic [127:0] ram [0:RAM_ROWS-1];
   logic [127:0] r_ram_q;
   logic [12:0]  ram_idx;
   logic         ram_we;
   logic [127:0] ram_wd;
   logic [15:0]  ram_ben;

   always_ff @(posedge i_Clk) begin
      r_ram_q <= ram[ram_idx];
      if (ram_we) begin
         for (int i = 0; i < 16; i++) begin
            if (ram_ben[i]) ram[ram_idx][i*8 +: 8] <= ram_wd[i*8 +: 8];
         end
      end
   end

   // ------------------------------------------------- core-2 local bus (CE)
   // Mirrors the tb_pipeline_isa membus model in CORE cycles: accept when
   // !ready (the ready-observed-low guard), 1 wait state, ready for exactly
   // one core cycle with the data.
   typedef enum logic [2:0] { P_IDLE, P_RESP, P_DDR, P_CLOOK, P_ETH } c2bus_ph_t;
   c2bus_ph_t   ph;
   logic [31:0] q_addr;
   logic        q_wr;
   logic [63:0] q_wdata;
   logic [7:0]  q_be;
   logic        r_dgo;                  // level: DDR adapter go (CE side)
   logic [127:0] r_drd;                 // adapter result, logical dword order
   logic         r_ddone;               // level: adapter done (until !r_dgo)
   logic         r_ego;                 // level: eth adapter go (CE side)
   logic         r_edone;               // level: eth adapter done (until !r_ego)
   logic [63:0]  r_erd;                 // eth read data
   logic         r_eth_owner;           // C2_ETH_OWNER register

   // ------------------------------------------------- text read cache (P2b)
   // Direct-mapped, 512 x 16 B lines (8 KB), covering ONLY the 1 MB core-2
   // text window at 0x07E0_0000 (immutable after load+flush by contract).
   // A miss is the ordinary uncached burst plus an install; hits serve in
   // the same 2-core-cycle shape as local BRAM.  Valid bits live in FFs so
   // every RUN 0→1 invalidates the whole cache in one cycle; a (contract-
   // violating) write into the window clears its line's valid as a backstop.
   localparam int CT_LINES = 512;
   localparam logic [11:0] CT_WIN = 12'h07E;        // addr[31:20] match
   logic [127:0] ct_data [0:CT_LINES-1];            // 2 RAMB36
   logic [6:0]   ct_tag  [0:CT_LINES-1];            // addr[19:13], LUTRAM
   logic [CT_LINES-1:0] ct_valid;
   logic [127:0] ct_data_q;
   logic [6:0]   ct_tag_q;
   logic         ct_valid_q;
   logic [8:0]   ct_idx;                            // registered read index
   wire  [8:0]   w_ct_look_idx;                     // driven below (comb)
   wire          w_ct_install;                      // install strobe (comb)
   wire          w_ct_wclear;                       // write-backstop clear

   always_ff @(posedge i_Clk) begin
      ct_idx    <= w_ct_look_idx;
      ct_data_q <= ct_data[w_ct_look_idx];
      ct_tag_q  <= ct_tag [w_ct_look_idx];
      if (w_ct_install) begin
         ct_data[q_addr[12:4]] <= r_drd;
         ct_tag [q_addr[12:4]] <= q_addr[19:13];
      end
   end
   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L || !r_run)  ct_valid <= '0;      // invalidate-all each start
      else if (w_ct_wclear)    ct_valid[q_addr[12:4]] <= 1'b0;
      else if (w_ct_install)   ct_valid[q_addr[12:4]] <= 1'b1;
      if (!i_Rst_L || !r_run)  ct_valid_q <= 1'b0;
      else                     ct_valid_q <= ct_valid[w_ct_look_idx];
   end

   wire        q_is_ram = (q_addr[31:17] == 15'h0000) ||       // 128 KB @0
                          (q_addr[31:17] == 15'h7000);         // and @0xE000_0000
   wire        q_is_con = (q_addr[31:16] == 16'hF001);   // UART-shape console
   wire [63:0] q_ram_dw = q_addr[3] ? r_ram_q[127:64] : r_ram_q[63:0];

   // Accept-time decode (c2_addr, before the q_ latch): DDR = anything in the
   // 128 MB window that isn't shadowed by the local BRAM at 0.  Core 2 cannot
   // see DDR's first 128 KB — shared buffers live above it by contract.
   wire        w_acc_is_ram = (c2_addr[31:17] == 15'h0000) ||
                              (c2_addr[31:17] == 15'h7000);
   wire        w_acc_is_ddr = (c2_addr < 32'h0800_0000) && !w_acc_is_ram;
   wire        w_acc_is_ct  = (c2_addr[31:20] == CT_WIN);
   wire        w_acc_is_eth = (c2_addr[31:28] == 4'hF) &&
                              (c2_addr[27:16] == 12'h006 ||
                               c2_addr[27:16] == 12'h007 ||
                               c2_addr[27:16] == 12'h008);
   wire        q_is_tmr     = (q_addr[31:16] == 16'hF00F);   // clock_ms mirror
   wire        q_is_ctext   = (q_addr[31:20] == CT_WIN);

   // Cache lookup index: incoming address during accept, held address after.
   assign w_ct_look_idx = (ph == P_IDLE) ? c2_addr[12:4] : q_addr[12:4];
   assign w_ct_install  = (ph == P_DDR) && r_ddone && !q_wr && q_is_ctext;
   assign w_ct_wclear   = (ph == P_DDR) && r_ddone &&  q_wr && q_is_ctext;

   logic       r_log_push;             // one core cycle wide; FIFO pops it &ce
   logic [7:0] r_log_pushb;
   wire        w_log_full;
   wire [9:0]  w_log_cnt;

   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L || !r_run) begin
         ph         <= P_IDLE;
         m2_ready   <= 1'b0;
         r_log_push <= 1'b0;
         r_dgo      <= 1'b0;
         r_ego      <= 1'b0;
      end else if (r_ce) begin
         m2_ready   <= 1'b0;
         r_log_push <= 1'b0;
         case (ph)
            P_IDLE: if (!m2_ready && (c2_read_DV || c2_write_DV)) begin
               q_addr  <= c2_addr;
               q_wr    <= c2_write_DV;
               q_wdata <= c2_wdata;
               q_be    <= c2_be;
               if (w_acc_is_eth) begin
                  r_ego <= 1'b1;                 // LiteEth window
                  ph    <= P_ETH;
               end else if (w_acc_is_ddr) begin
                  if (!c2_write_DV && w_acc_is_ct) begin
                     ph <= P_CLOOK;              // text window: try the cache
                  end else begin
                     r_dgo <= 1'b1;              // hand to the DDR adapter
                     ph    <= P_DDR;
                  end
               end else begin
                  ph    <= P_RESP;
               end
            end
            P_CLOOK: begin
               if (ct_valid_q && ct_tag_q == q_addr[19:13]) begin
                  m2_rdata      <= q_addr[3] ? ct_data_q[127:64]
                                             : ct_data_q[63:0];
                  m2_rdata_next <= ct_data_q[127:64];
                  m2_next_valid <= !q_addr[3];
                  m2_ready      <= 1'b1;
                  ph            <= P_IDLE;
               end else begin
                  r_dgo <= 1'b1;                 // miss: fetch + install
                  ph    <= P_DDR;
               end
            end
            P_RESP: begin
               if (!q_wr) begin
                  // Console STATUS mirrors the real UART's bit layout:
                  // [0]=tx_busy (fifo full), [1]=rx_empty=1, [2]=rx_full=0.
                  m2_rdata      <= q_is_ram ? q_ram_dw
                                : (q_is_con && q_addr[15:0] == 16'h0010)
                                    ? {61'b0, 1'b0, 1'b1, w_log_full}
                                : (q_is_tmr && q_addr[15:0] == 16'h0040)
                                    ? i_clock_ms : 64'h0;
                  m2_rdata_next <= r_ram_q[127:64];
                  m2_next_valid <= q_is_ram && !q_addr[3];
               end else if (q_is_con && q_addr[15:0] == 16'h0000) begin
                  r_log_push  <= 1'b1;
                  r_log_pushb <= q_wdata[7:0];
               end
               // RAM writes are applied by the port mux below (held for the
               // whole P_RESP core cycle — two identical 100 MHz writes,
               // idempotent by construction).
               m2_ready <= 1'b1;
               ph       <= P_IDLE;
            end
            P_ETH: if (r_edone) begin           // level from the eth adapter
               if (!q_wr) begin
                  m2_rdata      <= r_erd;
                  m2_next_valid <= 1'b0;
               end
               r_ego    <= 1'b0;
               m2_ready <= 1'b1;
               ph       <= P_IDLE;
            end
            P_DDR: if (r_ddone) begin           // level from the adapter
               if (!q_wr) begin
                  m2_rdata      <= q_addr[3] ? r_drd[127:64] : r_drd[63:0];
                  m2_rdata_next <= r_drd[127:64];
                  m2_next_valid <= !q_addr[3];  // pair dword, same 16 B burst
               end
               r_dgo    <= 1'b0;
               m2_ready <= 1'b1;
               ph       <= P_IDLE;
            end
         endcase
      end
   end

   // ------------------------------------------------ DDR adapter (full rate)
   // Runs at 100 MHz against the arbiter's pulse-based grant/ready; presents
   // only LEVELS to the CE side (r_ddone holds until r_dgo drops — the plan's
   // §3 handshake rule).  One transaction per grant (non-intrusion).
   // Data order: ddr2_control returns the addressed dword in the HIGH half
   // (the blitter-corruption convention) — swap to logical on read
   // ([63:0]=dw@base, [127:64]=dw@base+8), swap back on write.
   typedef enum logic [1:0] { D_IDLE, D_REQ, D_DRIVE, D_GAP } dfsm_t;
   dfsm_t        dph;
   logic [1:0]   r_dgap;

   wire [127:0] w_dwr_wire  = q_addr[3] ? {64'h0, q_wdata} : {q_wdata, 64'h0};
   wire [15:0]  w_dmask_wire= q_addr[3] ? {8'hFF, ~q_be}   : {~q_be, 8'hFF};

   // NB: reset on board reset ONLY, not on !r_run — if core 1 stops core 2
   // mid-transaction, the in-flight burst must complete and pulse done, or
   // the arbiter would hold grant C forever and starve the cache.  The CE
   // side drops r_dgo under !r_run, so the adapter simply drains and idles.
   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L) begin
         dph            <= D_IDLE;
         o_ddr_req      <= 1'b0;
         o_ddr_done     <= 1'b0;
         o_ddr_write_DV <= 1'b0;
         o_ddr_read_DV  <= 1'b0;
         r_ddone        <= 1'b0;
      end else begin
         o_ddr_done <= 1'b0;
         case (dph)
            D_IDLE: begin
               if (!r_dgo)          r_ddone <= 1'b0;
               else if (!r_ddone) begin
                  o_ddr_req <= 1'b1;
                  dph       <= D_REQ;
               end
            end
            D_REQ: if (i_ddr_grant) begin
               o_ddr_addr <= {q_addr[31:4], 4'b0};
               if (q_wr) begin
                  o_ddr_write_DV   <= 1'b1;
                  o_ddr_write_data <= w_dwr_wire;
                  o_ddr_wdf_mask   <= w_dmask_wire;
               end else begin
                  o_ddr_read_DV <= 1'b1;
               end
               dph <= D_DRIVE;
            end
            D_DRIVE: if (i_ddr_ready) begin
               if (q_wr) begin
                  o_ddr_write_DV <= 1'b0;
                  r_dgap         <= 2'd2;   // blitter's same-domain settle
                  dph            <= D_GAP;
               end else begin
                  r_drd         <= {i_ddr_read_data[63:0],
                                    i_ddr_read_data[127:64]};   // swap
                  o_ddr_read_DV <= 1'b0;
                  o_ddr_done    <= 1'b1;
                  o_ddr_req     <= 1'b0;
                  r_ddone       <= 1'b1;
                  dph           <= D_IDLE;
               end
            end
            D_GAP: begin
               if (r_dgap == 2'd0) begin
                  o_ddr_done <= 1'b1;
                  o_ddr_req  <= 1'b0;
                  r_ddone    <= 1'b1;
                  dph        <= D_IDLE;
               end else begin
                  r_dgap <= r_dgap - 2'd1;
               end
            end
         endcase
      end
   end

   // ------------------------------------------------ eth adapter (full rate)
   // Drives the bridge-shape request (held until the bridge's 1-cycle ready
   // pulse, data registered there), presents r_edone as a LEVEL to the CE
   // side, drops the strobe when r_ego drops (the bridge's S_COOL waits for
   // exactly that).  When core 2 is NOT the eth owner the top never routes
   // the request, so the adapter self-completes (reads 0, writes dropped)
   // rather than hanging the core-2 bus.
   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L) begin
         o_eth_write_DV <= 1'b0;
         o_eth_read_DV  <= 1'b0;
         r_edone        <= 1'b0;
      end else begin
         if (!r_ego) begin
            o_eth_write_DV <= 1'b0;
            o_eth_read_DV  <= 1'b0;
            r_edone        <= 1'b0;
         end else if (!r_edone) begin
            if (!r_eth_owner) begin
               r_erd   <= 64'h0;
               r_edone <= 1'b1;
            end else if (!o_eth_write_DV && !o_eth_read_DV) begin
               o_eth_addr       <= q_addr;
               o_eth_write_data <= q_wdata;
               o_eth_byte_en    <= q_be;
               o_eth_write_DV   <= q_wr;
               o_eth_read_DV    <= !q_wr;
            end else if (i_eth_ready) begin
               r_erd          <= i_eth_read_data;
               o_eth_write_DV <= 1'b0;
               o_eth_read_DV  <= 1'b0;
               r_edone        <= 1'b1;
            end
         end
      end
   end
   assign o_eth_owner = r_eth_owner;

   // --------------------------------------------------- mailbox window (P1)
   // Full-rate: only active while the core is held in reset (!r_run).
   logic [31:0] r_win_addr;
   logic [63:0] r_win_data;            // prefetched ram[r_win_addr]
   logic [1:0]  r_win_fetch;           // 2-cycle fetch shift (idx -> q -> reg)

   wire w_ctrl_wr = w_wr_edge && (mmio.addr[15:0] == 16'h0000);
   wire w_spc_wr  = w_wr_edge && (mmio.addr[15:0] == 16'h0008);
   wire w_wadr_wr = w_wr_edge && (mmio.addr[15:0] == 16'h0010);
   wire w_wdat_wr = w_wr_edge && (mmio.addr[15:0] == 16'h0018) && !r_run;
   wire w_lpop_wr = w_wr_edge && (mmio.addr[15:0] == 16'h0020);
   wire w_own_wr  = w_wr_edge && (mmio.addr[15:0] == 16'h0038);

   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L) begin
         r_run       <= 1'b0;
         r_start_pc  <= 32'h20;
         r_win_addr  <= 32'h0;
         r_win_fetch <= 2'b00;
         r_eth_owner <= 1'b0;                     // reset: core 1 owns eth (netboot)
      end else begin
         if (w_own_wr)  r_eth_owner <= mmio.write_data[0];
         r_win_fetch <= {r_win_fetch[0], 1'b0};
         if (r_win_fetch[1]) r_win_data <= r_win_addr[3] ? r_ram_q[127:64]
                                                         : r_ram_q[63:0];
         if (w_ctrl_wr) r_run      <= mmio.write_data[0];
         if (w_spc_wr)  r_start_pc <= mmio.write_data[31:0];
         if (w_wadr_wr) begin
            r_win_addr  <= mmio.write_data[31:0];
            r_win_fetch <= 2'b01;                 // prefetch for readback
         end
         if (w_wdat_wr) r_win_addr <= r_win_addr + 32'd8;
      end
   end

   // RAM port mux: core bus while running, window while in reset.
   wire w_core_ram_wr = r_run && (ph == P_RESP) && q_wr && q_is_ram;
   always_comb begin
      if (r_run) begin
         // P_IDLE presents the incoming request address so r_ram_q is valid
         // by P_RESP; P_RESP holds q_addr for the (idempotent) write.
         ram_idx = (ph == P_IDLE) ? c2_addr[16:4] : q_addr[16:4];
         ram_we  = w_core_ram_wr;
         ram_wd  = {q_wdata, q_wdata};
         ram_ben = q_addr[3] ? {q_be, 8'h00} : {8'h00, q_be};
      end else begin
         ram_idx = r_win_addr[16:4];
         ram_we  = w_wdat_wr;
         ram_wd  = {mmio.write_data, mmio.write_data};
         ram_ben = r_win_addr[3] ? 16'hFF00 : 16'h00FF;
      end
   end

   // -------------------------------------------------------------- log FIFO
   // 512 x 8 distributed RAM.  Write side is the CE-paced core bus (push
   // strobe is one CORE cycle = two clocks; qualified with r_ce it enqueues
   // exactly once).  Read side is core 1: non-destructive head + write-to-pop.
   logic [7:0] fifo_mem [0:LOG_DEPTH-1];
   logic [8:0] r_fifo_wp, r_fifo_rp;
   logic [9:0] r_fifo_cnt;
   wire        w_push = r_log_push && r_ce && (r_fifo_cnt != LOG_DEPTH[9:0]);
   wire        w_pop  = w_lpop_wr && (r_fifo_cnt != 10'd0);

   assign w_log_full = (r_fifo_cnt >= LOG_DEPTH[9:0] - 10'd8);  // early full
   assign w_log_cnt  = r_fifo_cnt;

   always_ff @(posedge i_Clk) begin
      if (!i_Rst_L) begin
         r_fifo_wp  <= '0;
         r_fifo_rp  <= '0;
         r_fifo_cnt <= '0;
      end else begin
         if (w_push) begin
            fifo_mem[r_fifo_wp] <= r_log_pushb;
            r_fifo_wp <= r_fifo_wp + 9'd1;
         end
         if (w_pop) r_fifo_rp <= r_fifo_rp + 9'd1;
         case ({w_push, w_pop})
            2'b10: r_fifo_cnt <= r_fifo_cnt + 10'd1;
            2'b01: r_fifo_cnt <= r_fifo_cnt - 10'd1;
            default: ;
         endcase
      end
   end
   wire [7:0] w_log_head  = fifo_mem[r_fifo_rp];
   wire       w_log_valid = (r_fifo_cnt != 10'd0);

   // --------------------------------------------------------- MMIO read mux
   always_comb begin
      mmio.read_data = 64'h0;
      case (mmio.addr[15:0])
         16'h0000: mmio.read_data = {55'b0, r_start_pend, 3'b0,
                                     c2_park_kind, c2_parked, r_run};
         16'h0008: mmio.read_data = {32'b0, r_start_pc};
         16'h0010: mmio.read_data = {32'b0, r_win_addr};
         16'h0018: mmio.read_data = r_win_data;
         16'h0020: mmio.read_data = {55'b0, w_log_valid, w_log_head};
         16'h0028: mmio.read_data = {54'b0, w_log_cnt};
         16'h0030: mmio.read_data = {32'b0, c2_park_pc};
         16'h0038: mmio.read_data = {63'b0, r_eth_owner};
         default:  mmio.read_data = 64'h0;
      endcase
   end

endmodule : core2_subsys

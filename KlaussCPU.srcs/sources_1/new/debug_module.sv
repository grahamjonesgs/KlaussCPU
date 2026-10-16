//===========================================================================
// debug_module — KlaussCPU debug module (DEBUG_PLAN.md Phase 5, §4).
//
// Three pieces:
//   debug_module  core-clock register file (the DMI) driving pipeline_core's
//                 dbg_i / reading dbg_o and the architectural state
//   dm_dtm        JTAG data register + clock-domain crossing to the DMI
//   dm_jtag       BSCANE2 (USER3) wrapper around dm_dtm (synthesis only)
//
// DMI registers (64-bit, address 8 bits).  The host (klausscc gdbserver
// --board) only needs these; see DEBUG_PLAN.md §4.
//
//   0x00 DMCONTROL  RW [0] haltreq: halt, cleared by the DM once halted
//                      [1] resumereq: W1, ignored unless halted
//                      [2] step: with resumereq, halt after one instruction
//                      [3] attached: TRAP / ILLEGAL / HALT enter a debug halt
//                      [4] halt_on_load: halt at the entry of the next program
//   0x01 DMSTATUS   RO [0] halted   [1] running   [2] busy (memory command)
//                      [3] error (sticky: command while running or busy;
//                          cleared by writing DMSTATUS)
//                      [6:4] cause (klauss_pkg DBG_CAUSE_*)
//                      [7] parked (any park)   [10:8] park_kind
//                      [11] halted since this bit was last read (sticky)
//                      [15:12] version = 1   [19:16] breakpoints = 4
//                      [23:20] watchpoints = 2   [31:24] 0x4B ('K')
//   0x02 REG_CMD    W  [4:0] register (0-15 R, 16 SP, 17 PC, 18 FLAGS
//                      {Z,S,C,V}, 19 INT_MASK), [8] write.  Read: DATA0 =
//                      register; write: register = DATA0.  Halted only.
//   0x03 DATA0      RW data for REG_CMD / MEM_CMD
//   0x04 MEM_ADDR   RW byte address (doubleword-aligned for MEM_CMD)
//   0x05 MEM_CMD    W  [7:0] byte enables, [8] write (data = DATA0, lane-
//                      placed), [9] post-increment MEM_ADDR by 8.  Halted
//                      only; DMSTATUS.busy until done, then reads land in
//                      DATA0.  Goes through the core's memory port, so it is
//                      D-cache coherent and decodes MMIO like a CPU access.
//   0x06 PC         RO the halted core's PC
//   0x10-0x13 BPn   RW [31:0] address, [32] enable
//   0x18-0x19 WPn   RW [31:3] doubleword, [32] enable, [33] on loads,
//                      [34] on stores, [47:40] byte lanes
//   0x1F WATCH_ADDR RO byte address of the access that fired a watchpoint
//===========================================================================

module debug_module
   import klauss_pkg::*;
(
   input  logic         clk,
   input  logic         rst,
   // DMI (from dm_dtm): one request per dmi_req pulse, answered next cycle
   input  logic         dmi_req,
   input  logic [1:0]   dmi_op,      // 1 = read, 2 = write
   input  logic [7:0]   dmi_addr,
   input  logic [63:0]  dmi_wdata,
   output logic         dmi_rsp,
   output logic [63:0]  dmi_rdata,
   // core
   output dbg_in_t      dbg_i,
   input  dbg_out_t     dbg_o,
   input  logic         parked,
   input  logic [2:0]   park_kind,
   input  logic [31:0]  park_pc,
   input  logic [63:0]  core_r [0:15],
   input  logic [31:0]  core_sp,
   input  flags_t       core_flags,
   input  logic [3:0]   core_int_mask,
   input  logic         prog_start    // pulse: the loader started a program
);
   logic        haltreq, attached, halt_on_load, error, halted_seen;
   logic [63:0] data0;
   logic [31:0] mem_addr;
   logic        mem_busy, mem_inc;
   logic [3:0]  bp_en;
   logic [3:0][31:0] bp_addr;
   logic [1:0]  wp_en, wp_rd, wp_wr;
   logic [1:0][28:0] wp_dw;
   logic [1:0][7:0]  wp_be;
   logic        halted_q;
   logic        resume_p, step_p, reg_we_p, mem_req_p, mem_we_q;
   logic [4:0]  reg_idx_q;
   logic [7:0]  mem_be_q;

   wire halted = dbg_o.halted;

   function automatic logic [63:0] f_reg(logic [4:0] idx);
      case (idx)
         5'd16:   return {32'b0, core_sp};
         5'd17:   return {32'b0, park_pc};
         5'd18:   return {60'b0, core_flags.zero, core_flags.sign, core_flags.carry, core_flags.overflow};
         5'd19:   return {60'b0, core_int_mask};
         default: return (idx < 5'd16) ? core_r[idx[3:0]] : 64'b0;
      endcase
   endfunction

   wire [63:0] dmstatus = {32'b0, 8'h4B, 4'd2, 4'd4, 4'd1, halted_seen,
                           park_kind, parked, dbg_o.cause, error, mem_busy,
                           !parked, halted};

   always_comb begin
      dbg_i           = '0;
      dbg_i.halt_req  = haltreq;
      dbg_i.attached  = attached;
      dbg_i.bp_en     = bp_en;
      dbg_i.bp_addr   = bp_addr;
      dbg_i.wp_en     = wp_en;
      dbg_i.wp_rd     = wp_rd;
      dbg_i.wp_wr     = wp_wr;
      dbg_i.wp_dw     = wp_dw;
      dbg_i.wp_be     = wp_be;
      dbg_i.resume    = resume_p;
      dbg_i.step      = step_p;
      dbg_i.reg_we    = reg_we_p;
      dbg_i.reg_idx   = reg_idx_q;
      dbg_i.reg_wdata = data0;
      dbg_i.mem_req   = mem_req_p;
      dbg_i.mem_we    = mem_we_q;
      dbg_i.mem_addr  = {mem_addr[31:3], 3'b000};
      dbg_i.mem_wdata = data0;
      dbg_i.mem_be    = mem_be_q;
   end

   always_ff @(posedge clk) begin
      if (rst) begin
         haltreq <= 1'b0;  attached <= 1'b0;  halt_on_load <= 1'b0;  error <= 1'b0;
         halted_seen <= 1'b0;  halted_q <= 1'b0;
         data0 <= '0;  mem_addr <= '0;  mem_busy <= 1'b0;  mem_inc <= 1'b0;
         bp_en <= '0;  bp_addr <= '0;
         wp_en <= '0;  wp_rd <= '0;  wp_wr <= '0;  wp_dw <= '0;  wp_be <= '0;
         resume_p <= 1'b0;  step_p <= 1'b0;  reg_we_p <= 1'b0;  mem_req_p <= 1'b0;
         mem_we_q <= 1'b0;  reg_idx_q <= '0;  mem_be_q <= '0;
         dmi_rsp <= 1'b0;  dmi_rdata <= '0;
      end else begin
         resume_p <= 1'b0;  step_p <= 1'b0;  reg_we_p <= 1'b0;  mem_req_p <= 1'b0;
         dmi_rsp <= 1'b0;

         halted_q <= halted;
         if (halted && !halted_q) begin
            haltreq     <= 1'b0;   // done: a later resume must not re-halt
            halted_seen <= 1'b1;
         end
         if (prog_start && halt_on_load) haltreq <= 1'b1;   // halt at the entry

         if (mem_busy && dbg_o.mem_done) begin
            mem_busy <= 1'b0;
            if (!mem_we_q) data0 <= dbg_o.mem_rdata;
            if (mem_inc) mem_addr <= mem_addr + 32'd8;
         end

         if (dmi_req) begin
            dmi_rsp <= 1'b1;
            if (dmi_op == 2'd1) begin
               case (dmi_addr)
                  8'h00: dmi_rdata <= {59'b0, halt_on_load, attached, 2'b00, haltreq};
                  8'h01: begin dmi_rdata <= dmstatus; halted_seen <= 1'b0; end
                  8'h03: dmi_rdata <= data0;
                  8'h04: dmi_rdata <= {32'b0, mem_addr};
                  8'h06: dmi_rdata <= {32'b0, park_pc};
                  8'h10, 8'h11, 8'h12, 8'h13:
                         dmi_rdata <= {31'b0, bp_en[dmi_addr[1:0]], bp_addr[dmi_addr[1:0]]};
                  8'h18, 8'h19:
                         dmi_rdata <= {16'b0, wp_be[dmi_addr[0]], 5'b0, wp_wr[dmi_addr[0]],
                                       wp_rd[dmi_addr[0]], wp_en[dmi_addr[0]],
                                       wp_dw[dmi_addr[0]], 3'b000};
                  8'h1F: dmi_rdata <= {32'b0, dbg_o.watch_addr};
                  default: dmi_rdata <= '0;
               endcase
            end else if (dmi_op == 2'd2) begin
               dmi_rdata <= '0;
               case (dmi_addr)
                  8'h00: begin
                     if (dmi_wdata[0] && !halted) haltreq <= 1'b1;
                     if (!dmi_wdata[0]) haltreq <= 1'b0;
                     attached     <= dmi_wdata[3];
                     halt_on_load <= dmi_wdata[4];
                     if (dmi_wdata[1]) begin
                        if (halted && !mem_busy) begin
                           resume_p <= 1'b1;  step_p <= dmi_wdata[2];
                        end else error <= 1'b1;
                     end
                  end
                  8'h01: error <= 1'b0;
                  8'h02: begin
                     if (!halted || mem_busy) error <= 1'b1;
                     else if (dmi_wdata[8]) begin
                        reg_we_p <= 1'b1;  reg_idx_q <= dmi_wdata[4:0];
                     end else data0 <= f_reg(dmi_wdata[4:0]);
                  end
                  8'h03: data0 <= dmi_wdata;
                  8'h04: mem_addr <= dmi_wdata[31:0];
                  8'h05: begin
                     if (!halted || mem_busy) error <= 1'b1;
                     else begin
                        mem_req_p <= 1'b1;  mem_busy <= 1'b1;
                        mem_we_q  <= dmi_wdata[8];  mem_be_q <= dmi_wdata[7:0];
                        mem_inc   <= dmi_wdata[9];
                     end
                  end
                  8'h10, 8'h11, 8'h12, 8'h13: begin
                     bp_addr[dmi_addr[1:0]] <= dmi_wdata[31:0];
                     bp_en[dmi_addr[1:0]]   <= dmi_wdata[32];
                  end
                  8'h18, 8'h19: begin
                     wp_dw[dmi_addr[0]] <= dmi_wdata[31:3];
                     wp_en[dmi_addr[0]] <= dmi_wdata[32];
                     wp_rd[dmi_addr[0]] <= dmi_wdata[33];
                     wp_wr[dmi_addr[0]] <= dmi_wdata[34];
                     wp_be[dmi_addr[0]] <= dmi_wdata[47:40];
                  end
                  default: ;
               endcase
            end
         end
      end
   end
endmodule

//===========================================================================
// dm_dtm — JTAG data register for the DMI, and the TCK -> clk crossing.
//
// DR (74 bits, shifted LSB first):  in  {op[1:0], addr[7:0], data[63:0]}
//                                   out {status[1:0], addr[7:0], data[63:0]}
// Each scan returns the result of the PREVIOUS request: CAPTURE-DR loads the
// response, UPDATE-DR issues the new request (op 0 = no request).  status:
// 0 = ok, 3 = busy (the previous request had not been answered when this
// scan captured — repeat the scan after a few TCK cycles in Run-Test/Idle).
// The request crosses with a toggle synchronizer; the response is held
// stable before its toggle returns, so TCK-side reads are glitch-free.
//===========================================================================
module dm_dtm (
   // JTAG side (BSCANE2 USER3 or a testbench)
   input  logic        tck,
   input  logic        sel,       // USER3 selected
   input  logic        capture,   // CAPTURE-DR
   input  logic        shift,     // SHIFT-DR
   input  logic        update,    // UPDATE-DR
   input  logic        tdi,
   output logic        tdo,
   // DMI side (clk)
   input  logic        clk,
   input  logic        rst,
   output logic        dmi_req,
   output logic [1:0]  dmi_op,
   output logic [7:0]  dmi_addr,
   output logic [63:0] dmi_wdata,
   input  logic        dmi_rsp,
   input  logic [63:0] dmi_rdata
);
   localparam int DRW = 74;
   // ---- TCK domain ----
   logic [DRW-1:0] dr;
   logic [DRW-1:0] req_hold;    // stable from UPDATE until the next request
   logic           req_tgl = 1'b0;
   (* ASYNC_REG = "TRUE" *) logic [1:0] rsp_tgl_s;
   assign tdo = dr[0];

   // response (clk domain registers, stable whenever rsp_tgl_s matches)
   logic [63:0] rsp_data;
   logic [7:0]  rsp_addr;
   logic        rsp_tgl = 1'b0;

   always_ff @(posedge tck) begin
      rsp_tgl_s <= {rsp_tgl_s[0], rsp_tgl};
      if (sel && capture)
         dr <= {(rsp_tgl_s[1] == req_tgl) ? 2'd0 : 2'd3, rsp_addr, rsp_data};
      else if (sel && shift)
         dr <= {tdi, dr[DRW-1:1]};
      if (sel && update && dr[DRW-1 -: 2] != 2'd0 && rsp_tgl_s[1] == req_tgl) begin
         req_hold <= dr;
         req_tgl  <= ~req_tgl;
      end
   end

   // ---- clk domain ----
   (* ASYNC_REG = "TRUE" *) logic [2:0] req_tgl_s;
   logic req_seen;
   always_ff @(posedge clk) begin
      if (rst) begin
         req_tgl_s <= '0;  req_seen <= 1'b0;  dmi_req <= 1'b0;
      end else begin
         req_tgl_s <= {req_tgl_s[1:0], req_tgl};
         dmi_req   <= 1'b0;
         if (req_tgl_s[2] != req_seen) begin
            req_seen  <= req_tgl_s[2];
            dmi_req   <= 1'b1;
            dmi_op    <= req_hold[DRW-1 -: 2];
            dmi_addr  <= req_hold[71:64];
            dmi_wdata <= req_hold[63:0];
         end
         if (dmi_rsp) begin
            rsp_data <= dmi_rdata;
            rsp_addr <= dmi_addr;
            rsp_tgl  <= req_seen;   // answered: matches req_tgl again
         end
      end
   end
endmodule

//===========================================================================
// dm_jtag — the debug transport on the FPGA's own JTAG port (USER3), so the
// board's USB cable carries the debugger.  Vivado's hardware manager / ILA
// debug hub use USER1 by default; they cannot share the cable at the same
// time, but they coexist in one bitstream.
//===========================================================================
module dm_jtag (
   input  logic        clk,
   input  logic        rst,
   output logic        dmi_req,
   output logic [1:0]  dmi_op,
   output logic [7:0]  dmi_addr,
   output logic [63:0] dmi_wdata,
   input  logic        dmi_rsp,
   input  logic [63:0] dmi_rdata
);
   logic tck_raw, tck, sel, capture, shift, update, tdi, tdo;
`ifndef DM_NO_BSCAN
   BSCANE2 #(.JTAG_CHAIN(3)) bscan_i (
      .CAPTURE(capture), .DRCK(), .RESET(), .RUNTEST(), .SEL(sel),
      .SHIFT(shift), .TCK(tck_raw), .TDI(tdi), .TMS(), .UPDATE(update),
      .TDO(tdo)
   );
   BUFG tck_bufg (.I(tck_raw), .O(tck));
`else
   assign {tck, sel, capture, shift, update, tdi} = '0;
`endif
   dm_dtm dtm_i (.*);
endmodule

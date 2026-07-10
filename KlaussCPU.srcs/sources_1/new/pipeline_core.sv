//===========================================================================
// pipeline_core — Phase B in-order 5-stage pipeline (feat/pipeline).
// M5a: FULL ISA v2 (classes 1-C per ISA_ENCODING_V2_MAP.md), ideal memory.
// See PIPELINE_IMPL.md §8. Interlock-only — no forwarding until M6:
//   - GPR RAW: stall ID until the producer leaves WB.
//   - Flag READERS (cond branches, ADC/SBC, RCL/RCR, GETF) stall on any
//     in-flight flag writer. Flag WRITERS carry {value, mask} down the pipe
//     and masked-merge at WB — writers never read flags (no writer hazard).
//   - SP users serialize on in-flight SP writers; SP commits at WB.
//   - RET/IRET/HALT/WAIT/RESET/TRAP/illegal serialize: ID stops fetching
//     behind them; RET/IRET redirect at MEM, RESET at EX, the rest park.
//   - Branches resolve in EX (predict-not-taken, flush IF/ID on taken).
//   - MUL/DIV/DELAY/LCD occupy EX (busy interlock, M4); MEMGET32 cross-dword
//     occupies MEM one extra cycle (mem busy).
// Single retire point = WB: rf / flags(masked) / SP / int_mask commit there,
// and the retire interface presents the emulator-format trace record
// (klausscc --emulate --trace) for the tb to print and diff.
//
// Memory (M5b): ONE shared membus-style port — the exact cpu_mem signal set
// (KlaussCPU.sv:655-663) so M5d wires it straight to bus_splitter. Registered
// DV/addr, ready handshake (the FSM extra_clock pattern), read lookahead
// (rdata_next/next_valid within a 32 B line). IF and MEM contend: MEM has
// absolute priority (PIPELINE_IMPL.md §5); IF fills a 2-dword IFB-style
// window (any 1/2/3-word instruction spans at most 2 dwords).
//===========================================================================
module pipeline_core
   import klauss_pkg::*;
(
   input  logic         clk,
   input  logic         rst,
   // run control
   input  logic         start,       // pulse: begin fetching at start_pc
   input  logic [31:0]  start_pc,
   // shared memory port (membus_if master shape)
   output logic [31:0]  m_addr,
   output logic         m_read_DV,
   output logic         m_write_DV,
   output logic [63:0]  m_wdata,
   output logic [7:0]   m_be,
   input  logic [63:0]  m_rdata,
   input  logic [63:0]  m_rdata_next,   // next sequential dword, same line
   input  logic         m_next_valid,
   input  logic         m_ready,
   // LCD side port
   output logic [7:0]   lcd_byte,
   output logic         lcd_dc,
   output logic         lcd_dv,
   output logic         lcd_rst_n,
   input  logic         lcd_ready,
   // retire / trace (registered at the WB commit edge; sample at negedge)
   output logic         ret_valid,
   output logic [31:0]  ret_pc,
   output logic [31:0]  ret_op,
   output logic         ret_wr,      // this retire performed a memory/MMIO write
   output logic [31:0]  ret_wr_addr, // as driven on d_addr
   output logic [7:0]   ret_wr_be,
   output logic [63:0]  ret_wr_raw,  // RAW store value (pre lane-replication)
   // park (drained stop)
   output logic         parked,
   output logic [2:0]   park_kind,   // 0=HALT 1=TRAP 2=ILLEGAL 3=WAIT
   output logic [31:0]  park_pc,
   output logic [31:0]  park_op,
   // architectural state (tb reads for the trace line)
   output logic [63:0]  dbg_r [0:15],
   output logic [31:0]  dbg_sp,
   output flags_t       dbg_flags
);

   // ------------------------------------------------------------------ types
   typedef enum logic [5:0] {
      U_ILL, U_ALU, U_BOOL, U_SHIFT, U_ROT1, U_BTSTN, U_UNARY,
      U_LEA, U_MOV, U_MOV64, U_BEXTR, U_BDEP,
      U_LOAD, U_LOAD32, U_STORE,
      U_PUSH, U_POP, U_GETSP, U_SETSP, U_ADDSP, U_RET, U_IRET,
      U_JMP, U_CALL, U_MUL, U_DIV, U_MOD,
      U_NOP, U_HALT, U_WAIT, U_RESET, U_TRAP, U_DELAY, U_LCD, U_LCDRST
   } uop_e;

   typedef struct packed {
      logic       legal;
      uop_e       uop;
      logic [3:0] sub;          // class minor-op (aluop / unary op / shift op)
      logic [3:0] rd, rs1, rs2;
      logic       use_rs1, use_rs2, use_rdd;
      logic       wreg;
      flags_t     fmask;        // which flag fields this op writes
      logic       fread;        // reads committed flags in EX
      logic       sp_rd, sp_wr;
      logic       is_mem;
      logic       serialize;
      logic       sgn;          // imm-ext / load / muldiv sign attr
      logic [1:0] msz;          // load/store size
      logic [1:0] mmode;        // EA mode
      logic       malign;
      logic       shsrc;        // shift count from N (else rs2)
      logic [5:0] shn;
      logic       shf;          // shift F bit
      logic [1:0] unsz;         // sext/zext size
      logic       unf;          // unary F bit
      logic       blink, brel, brind, binv;
      logic [3:0] bcond;
      logic       mdhigh, mduns;
      logic       lcd_is_data;
      logic [1:0] len;          // instruction words
   } dec_t;

   // ----------------------------------------------------- architectural state
   logic [63:0] rf [0:15];
   flags_t      flags;
   logic [31:0] sp;
   logic [3:0]  int_mask;
   logic [31:0] pc;
   logic        running;        // fetch enabled
   logic        fetch_halt;     // serialize op in flight — hold fetch

   genvar gi;
   generate for (gi = 0; gi < 16; gi++) assign dbg_r[gi] = rf[gi]; endgenerate
   assign dbg_sp = sp; assign dbg_flags = flags;

   // ------------------------------------------- IF: 2-dword window (IFB model)
   logic [63:0] ifb_dw [0:1];
   logic [28:0] ifb_base;          // dword address of slot 0
   logic [1:0]  ifb_val;

   wire [28:0] pc_dw = pc[31:3];
   wire        in0   = ifb_val[0] && (pc_dw == ifb_base);
   wire        in1   = ifb_val[1] && (pc_dw == ifb_base + 29'd1);
   wire [127:0] w_win = in0 ? {ifb_dw[1], ifb_dw[0]} : {64'h0, ifb_dw[1]};
   wire [31:0] w_op   = pc[2] ? w_win[63:32]  : w_win[31:0];
   wire [31:0] w_var1 = pc[2] ? w_win[95:64]  : w_win[63:32];
   wire [31:0] w_var2 = pc[2] ? w_win[127:96] : w_win[95:64];
   // does the instruction spill into the next dword?
   wire        w_span = pc[2] ? (w_op[31:30] != 2'b01) : (w_op[31:30] == 2'b11);
   // the second dword is only sourced when the op sits in slot 0
   wire        fetch_ok = (in0 && (!w_span || ifb_val[1])) || (in1 && !w_span);
   // dword to request: slot-1 top-up when the op sits in slot 0 and spills;
   // otherwise rebase the window at the op's own dword (next_valid refills
   // slot 1 for free within a line).
   wire [28:0] miss_dw  = in0 ? pc_dw + 29'd1 : pc_dw;

   // ----------------------------------------------------------- ID latch
   logic        id_valid;
   logic [31:0] id_pc, id_op, id_var1, id_var2;

   // -------------------------------------------------------------- decoder
   function automatic dec_t f_decode(logic [31:0] op);
      dec_t d;
      logic [1:0] len;   logic [3:0] cls, aluop;
      d = '0;
      len   = op[31:30];
      cls   = op[29:26];
      aluop = op[25:22];
      d.len = len;  d.rd = op[11:8];  d.rs1 = op[7:4];  d.rs2 = op[3:0];
      d.uop = U_ILL;
      case (cls)
         4'h1, 4'h2: begin // ALU reg-reg (1w) / reg-imm (2w)
            d.sgn = op[20];
            if ((cls == 4'h1 && len == 2'b01 && op[20] == 1'b0 && op[19:12] == 8'h0) ||
                (cls == 4'h2 && len == 2'b10 && op[19:12] == 8'h0 && op[3:0] == 4'h0)) begin
               d.use_rs1 = 1'b1;  d.use_rs2 = (cls == 4'h1);
               case ({aluop, op[21]})
                  {4'd0,1'b1}, {4'd1,1'b1}, {4'd2,1'b1}, {4'd3,1'b1}: begin
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = aluop; d.wreg = 1'b1;
                     d.fmask = '1;  d.fread = (aluop == 4'd2 || aluop == 4'd3);
                  end
                  {4'd4,1'b0}, {4'd5,1'b0}, {4'd6,1'b0}: begin
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = aluop; d.wreg = 1'b1;
                  end
                  {4'd7,1'b0}, {4'd8,1'b0}, {4'd9,1'b0}, {4'd10,1'b0}: begin
                     if (cls == 4'h1) begin // MIN/MAX are reg-reg only
                        d.legal = 1'b1; d.uop = U_BOOL; d.wreg = 1'b1;
                        d.sub = (aluop == 4'd7) ? 4'(CMP_MIN)  : (aluop == 4'd8) ? 4'(CMP_MAX)
                              : (aluop == 4'd9) ? 4'(CMP_MINU) : 4'(CMP_MAXU);
                     end
                  end
                  {4'd14,1'b0}: if (cls == 4'h2) begin // LEAPC
                     d.legal = 1'b1; d.uop = U_LEA; d.wreg = 1'b1; d.use_rs1 = 1'b0;
                  end
                  {4'd15,1'b0}: if (cls == 4'h2) begin // MOV: SETR / zext-MOV
                     d.legal = 1'b1; d.uop = U_MOV; d.wreg = 1'b1; d.use_rs1 = 1'b0;
                  end
                  default: ;
               endcase
            end else if (cls == 4'h2 && len == 2'b11 && aluop == 4'd15 && op[21:12] == 10'h0
                         && op[7:0] == 8'h00) begin
               d.legal = 1'b1; d.uop = U_MOV64; d.wreg = 1'b1;   // SETR64
            end
         end
         4'h3: begin // compare
            d.sgn = op[20];
            if ((len == 2'b01 && op[20] == 1'b0 && op[19:12] == 8'h0) ||
                (len == 2'b10 && op[19:12] == 8'h0 && op[3:0] == 4'h0)) begin
               d.use_rs1 = 1'b1;  d.use_rs2 = (len == 2'b01);
               if (op[21]) begin        // B=1: boolean rd = 0/1
                  if (op[25:23] <= 3'd4) begin
                     d.legal = 1'b1; d.uop = U_BOOL; d.wreg = 1'b1;
                     d.sub = 4'(f_cmp_op(op[25:23], op[22]));
                  end
               end else begin           // B=0: flag-setting CMP (= SUB, no wb)
                  if (op[25:23] == 3'd0 && !op[22] && op[11:8] == 4'h0) begin
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = 4'd7; // internal CMP code
                     d.fmask = '1;
                  end
               end
            end
         end
         4'h4: begin
            if (len == 2'b01 && op[13:12] == 2'b00) begin // shift/rotate/bit
               d.shsrc = op[21]; d.shn = op[20:15]; d.shf = op[14];
               d.use_rs1 = 1'b1; d.use_rs2 = !op[21];
               if (!op[21] && op[20:15] != 6'd0) begin /* SRC=0 requires N=0 */ end
               else case (aluop)
                  4'd0, 4'd1, 4'd2: begin
                     d.legal = 1'b1; d.uop = U_SHIFT; d.sub = aluop; d.wreg = 1'b1;
                     if (op[14]) d.fmask.zero = 1'b1;
                  end
                  4'd3, 4'd4: begin
                     d.legal = 1'b1; d.wreg = 1'b1;
                     if (op[21] && op[20:15] == 6'd1 && op[14]) begin
                        d.uop = U_ROT1; d.sub = aluop;
                        d.fmask.zero = 1'b1; d.fmask.carry = 1'b1;
                     end else begin
                        d.uop = U_SHIFT; d.sub = aluop;
                        if (op[14]) d.fmask.zero = 1'b1;
                     end
                  end
                  4'd5, 4'd6: begin // RCL/RCR only as SRC=1 N=1 F=1
                     if (op[21] && op[20:15] == 6'd1 && op[14]) begin
                        d.legal = 1'b1; d.uop = U_ROT1; d.sub = aluop; d.wreg = 1'b1;
                        d.fmask.zero = 1'b1; d.fmask.carry = 1'b1; d.fread = 1'b1;
                     end
                  end
                  4'd8, 4'd9, 4'd10: begin
                     d.legal = 1'b1; d.uop = U_SHIFT; d.sub = aluop; d.wreg = 1'b1;
                     if (op[14]) d.fmask.zero = 1'b1;
                  end
                  4'd11: begin
                     if (op[21]) begin
                        if (op[11:8] == 4'h0 && !op[14]) begin
                           d.legal = 1'b1; d.uop = U_BTSTN; d.fmask.zero = 1'b1;
                        end
                     end else begin
                        d.legal = 1'b1; d.uop = U_SHIFT; d.sub = 4'd11; d.wreg = 1'b1;
                        if (op[14]) d.fmask.zero = 1'b1;
                     end
                  end
                  default: ;
               endcase
            end else if (len == 2'b10 && aluop[3:2] == 2'b11 && op[21] == 1'b0
                         && op[20:15] == 6'd0 && op[14] == 1'b0 && op[13:12] == 2'b00) begin
               if (aluop == 4'd12 && op[3:0] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_BEXTR; d.wreg = 1'b1; d.use_rs1 = 1'b1;
                  d.fmask.zero = 1'b1;
               end else if (aluop == 4'd13) begin
                  d.legal = 1'b1; d.uop = U_BDEP; d.wreg = 1'b1;
                  d.use_rs1 = 1'b1; d.use_rs2 = 1'b1;
               end
            end
         end
         4'h5: begin // unary
            d.unsz = op[21:20]; d.unf = op[19];
            if (len == 2'b01 && op[18:12] == 7'h0 && op[3:0] == 4'h0) begin
               d.use_rs1 = 1'b1;
               if (aluop != 4'd4 && aluop != 4'd5 && op[21:20] != 2'd0) begin /* trap */ end
               else case (aluop)
                  4'd0, 4'd1, 4'd2, 4'd6, 4'd7, 4'd8, 4'd9, 4'd10: begin
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = aluop; d.wreg = 1'b1;
                     if (op[19]) d.fmask.zero = 1'b1;
                  end
                  4'd3: if (op[19]) begin  // ABS requires F
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd3; d.wreg = 1'b1;
                     d.fmask.zero = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  4'd4: if (op[21:20] != 2'd3) begin // SEXT
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd4; d.wreg = 1'b1;
                     if (op[19]) begin d.fmask.zero = 1'b1; d.fmask.sign = 1'b1; end
                  end
                  4'd5: if (op[21:20] != 2'd3) begin // ZEXT
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd5; d.wreg = 1'b1;
                     if (op[19]) d.fmask.zero = 1'b1;
                  end
                  4'd12: begin // GETF
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd12; d.wreg = 1'b1;
                     d.use_rs1 = 1'b0; d.fread = 1'b1;
                     if (op[19]) d.fmask.zero = 1'b1;
                  end
                  4'd14, 4'd15: if (op[19]) begin // INC/DEC = ALU +/- 1
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = (aluop == 4'd14) ? 4'd0 : 4'd1;
                     d.wreg = 1'b1; d.fmask = '1;
                  end
                  default: ;
               endcase
            end
         end
         4'h6, 4'h7: begin // loads / stores
            d.msz = op[25:24]; d.sgn = op[23]; d.mmode = op[22:21]; d.malign = op[20];
            if (op[19:12] == 8'h0
                && !(cls == 4'h7 && op[23])                         // stores: SGN reserved
                && !(op[20] && op[25:24] != 2'b11)                  // A only for 64-bit
                && ((len == 2'b01 && (op[22:21] == 2'b00 || op[22:21] == 2'b11)) ||
                    (len == 2'b10 && (op[22:21] == 2'b01 || op[22:21] == 2'b10)))
                && !(op[22:21] == 2'b00 && op[3:0] != 4'h0)         // [rs1]: rs2 = 0
                && !(op[22:21] == 2'b10 && op[7:0] != 8'h00)        // [imm]: rs1/rs2 = 0
                && !(op[22:21] == 2'b01 && op[3:0] != 4'h0)) begin  // rs1+imm: rs2 = 0
               d.use_rs1 = (op[22:21] != 2'b10);
               d.use_rs2 = (op[22:21] == 2'b11);
               d.is_mem  = 1'b1;
               if (cls == 4'h6) begin
                  if (op[25:24] == 2'b10 && op[22:21] == 2'b00) begin
                     if (!op[23]) begin d.legal = 1'b1; d.uop = U_LOAD32; d.wreg = 1'b1; end
                  end else begin
                     d.legal = 1'b1; d.uop = U_LOAD; d.wreg = 1'b1;
                  end
               end else begin
                  d.legal = 1'b1; d.uop = U_STORE; d.use_rdd = 1'b1;
               end
            end
         end
         4'h8: begin // branch / call
            d.blink = op[25]; d.brel = op[24]; d.brind = op[23];
            d.bcond = op[22:19]; d.binv = op[18];
            if (((len == 2'b10 && !op[23] && op[17:16] == 2'b00 && op[15:0] == 16'h0) ||
                 (len == 2'b01 && !op[24] && op[23] && op[17:16] == 2'b00 && op[15:4] == 12'h0))
                && (op[22:19] <= 4'd9) && !(op[22:19] == 4'd0 && op[18])) begin
               d.legal   = 1'b1;
               d.uop     = op[25] ? U_CALL : U_JMP;
               d.use_rs2 = op[23];
               d.fread   = (op[22:19] != 4'd0);
               if (op[25]) begin d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1; end
            end
         end
         4'h9: begin // stack / SP
            if (op[21:20] == 2'b00 && op[19:12] == 8'h0) begin
               if (len == 2'b01 && op[3:0] == 4'h0) begin
                  case (aluop)
                     4'd0: if (op[11:8] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_PUSH; d.use_rs1 = 1'b1;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
                     end
                     4'd2: if (op[7:4] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_POP; d.wreg = 1'b1;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
                     end
                     4'd3: if (op[7:4] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_GETSP; d.wreg = 1'b1; d.sp_rd = 1'b1;
                     end
                     4'd4: if (op[11:8] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_SETSP; d.use_rs1 = 1'b1; d.sp_wr = 1'b1;
                     end
                     4'd6: if (op[11:4] == 8'h00) begin
                        d.legal = 1'b1; d.uop = U_RET;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1; d.serialize = 1'b1;
                     end
                     4'd7: if (op[11:4] == 8'h00) begin
                        d.legal = 1'b1; d.uop = U_IRET; d.fmask = '1;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1; d.serialize = 1'b1;
                     end
                     default: ;
                  endcase
               end else if (len == 2'b10 && op[11:0] == 12'h0) begin
                  case (aluop)
                     4'd1: begin
                        d.legal = 1'b1; d.uop = U_PUSH;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
                     end
                     4'd5: begin
                        d.legal = 1'b1; d.uop = U_ADDSP; d.sp_rd = 1'b1; d.sp_wr = 1'b1;
                     end
                     default: ;
                  endcase
               end else if (len == 2'b11 && aluop == 4'd1 && op[11:0] == 12'h0) begin
                  d.legal = 1'b1; d.uop = U_PUSH;   // PUSHV64
                  d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
               end
            end
         end
         4'hA: begin // mul / div
            d.mduns = !op[23]; d.mdhigh = op[22]; d.sgn = op[23];
            if (op[21:20] == 2'b00 && op[19:12] == 8'h0
                && ((len == 2'b01) || (len == 2'b10 && op[3:0] == 4'h0))) begin
               d.use_rs1 = 1'b1;  d.use_rs2 = (len == 2'b01);  d.wreg = 1'b1;
               case (op[25:24])
                  2'd0: begin
                     d.legal = 1'b1; d.uop = U_MUL;
                     d.fmask.zero = 1'b1; d.fmask.sign = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  2'd1: if (!op[22]) begin
                     d.legal = 1'b1; d.uop = U_DIV;
                     d.fmask.zero = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  2'd2: if (!op[22]) begin
                     d.legal = 1'b1; d.uop = U_MOD;
                     d.fmask.zero = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  default: ;
               endcase
            end
         end
         4'hB: begin // system
            if (op[25:22] == 4'h0 && op[15:8] == 8'h0 && op[3:0] == 4'h0) begin
               if (len == 2'b01) begin
                  case (op[21:16])
                     6'd0: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_NOP; end
                     6'd1: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_HALT;  d.serialize = 1'b1; end
                     6'd2: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_WAIT;  d.serialize = 1'b1; end
                     6'd3: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_RESET; d.serialize = 1'b1; end
                     6'd4: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_TRAP;  d.serialize = 1'b1; end
                     6'd5: begin d.legal = 1'b1; d.uop = U_DELAY; d.use_rs1 = 1'b1; end
                     default: ;
                  endcase
               end else if (len == 2'b10 && op[21:16] == 6'd5 && op[7:4] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_DELAY;   // DELAYV
               end
            end
         end
         4'hC: begin // LCD
            if (op[23:12] == 12'h0 && op[3:0] == 4'h0) begin
               if (len == 2'b01 && op[25:24] <= 2'd1 && op[11:8] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_LCD; d.use_rs1 = 1'b1;
                  d.lcd_is_data = op[24];
               end else if (len == 2'b10 && op[7:4] == 4'h0 && op[11:8] == 4'h0) begin
                  if (op[25:24] <= 2'd1) begin
                     d.legal = 1'b1; d.uop = U_LCD; d.lcd_is_data = op[24];
                  end else if (op[25:24] == 2'd2) begin
                     d.legal = 1'b1; d.uop = U_LCDRST;
                  end
               end
            end
         end
         default: ;
      endcase
      if (!d.legal) begin
         d.uop = U_ILL; d.serialize = 1'b1;
         d.wreg = 1'b0; d.fmask = '0; d.is_mem = 1'b0; d.sp_wr = 1'b0;
         d.use_rs1 = 1'b0; d.use_rs2 = 1'b0; d.use_rdd = 1'b0; d.fread = 1'b0; d.sp_rd = 1'b0;
      end
      return d;
   endfunction

   dec_t dec;
   assign dec = f_decode(id_op);
   wire [31:0] id_pc_next = id_pc + {26'b0, dec.len, 2'b00};

   // --------------------------------------------------------- EX stage latch
   logic        ex_valid;
   logic [31:0] ex_pc, ex_op, ex_var1, ex_var2, ex_pc_next;
   dec_t        ex_d;
   logic [63:0] ex_a, ex_b, ex_c;   // rs1 / rs2 / rd-data values

   // --------------------------------------------------------- MEM stage latch
   logic        mem_valid;
   logic [31:0] mem_pc, mem_op;
   dec_t        mem_d;
   logic [63:0] mem_result;         // non-mem result (or store raw data)
   flags_t      mem_fval, mem_fmask;
   logic [31:0] mem_eaddr;          // RAW effective address (lane math)
   logic [31:0] mem_iaddr;          // ISSUE address (aligned, drives the port)
   logic [63:0] mem_wdata;          // lane-replicated store data
   logic [7:0]  mem_be;
   logic [31:0] mem_sp_new;
   logic        mem_sp_we;
   logic [63:0] mem_dw0;            // MEMGET32 cross-line stash

   // --------------------------------------------------------- WB stage latch
   logic        wb_valid;
   logic [31:0] wb_pc, wb_op;
   logic [3:0]  wb_rd;
   logic        wb_wreg;
   logic [63:0] wb_value;
   flags_t      wb_fval, wb_fmask;
   logic [31:0] wb_sp_new;
   logic        wb_sp_we;
   logic [3:0]  wb_mask_new;
   logic        wb_mask_we;
   logic        wb_park;
   logic [2:0]  wb_park_kind;
   logic        wb_wr;
   logic [31:0] wb_wr_addr;
   logic [7:0]  wb_wr_be;
   logic [63:0] wb_wr_raw;

   // ------------------------------------------------------------- interlocks
   // NB: explicit wire expressions (no helper function) — a function reading
   // module signals would break the assign's sensitivity list in simulation.
   wire b_ex  = ex_valid  && ex_d.wreg;
   wire b_mem = mem_valid && mem_d.wreg;
   wire b_wb  = wb_valid  && wb_wreg;
   wire hit1 = (b_ex && ex_d.rd == dec.rs1) || (b_mem && mem_d.rd == dec.rs1) || (b_wb && wb_rd == dec.rs1);
   wire hit2 = (b_ex && ex_d.rd == dec.rs2) || (b_mem && mem_d.rd == dec.rs2) || (b_wb && wb_rd == dec.rs2);
   wire hitd = (b_ex && ex_d.rd == dec.rd)  || (b_mem && mem_d.rd == dec.rd)  || (b_wb && wb_rd == dec.rd);
   wire flag_busy = (ex_valid && ex_d.fmask != '0) || (mem_valid && mem_fmask != '0)
                 || (wb_valid && wb_fmask != '0);
   wire sp_busy   = (ex_valid && ex_d.sp_wr) || (mem_valid && mem_sp_we)
                 || (wb_valid && wb_sp_we);
   wire raw_stall = id_valid && (
        (dec.use_rs1 && hit1) || (dec.use_rs2 && hit2) || (dec.use_rdd && hitd) ||
        (dec.fread && flag_busy) ||
        ((dec.sp_rd || dec.sp_wr) && sp_busy) );

   // ------------------------------------------------------------- EX units
   // multiply (silicon DSP48 chain model, M4)
   logic [64:0]  mul_a_q, mul_b_q;
   logic [127:0] mul_pipe1, mul_pipe2;
   logic [1:0]   mul_cnt;
   // divide (silicon restoring divider, M4)
   typedef enum logic [1:0] { MD_IDLE, MD_PREP, MD_STEP } md_phase_t;
   md_phase_t  div_ph;
   div_state_t dv;
   // delay spin
   logic        dly_on;
   logic [44:0] dly_cnt, dly_max;

   wire ex_is_md   = ex_valid && (ex_d.uop == U_MUL || ex_d.uop == U_DIV || ex_d.uop == U_MOD);
   wire ex_is_mul  = ex_valid && (ex_d.uop == U_MUL);
   wire ex_is_div  = ex_valid && (ex_d.uop == U_DIV || ex_d.uop == U_MOD);
   wire ex_div_mod = (ex_d.uop == U_MOD);
   wire ex_div_by0 = ex_is_div && (div_ph == MD_IDLE) && (ex_b == 64'd0);
   wire md_done    = (ex_is_mul && mul_cnt == 2'd3) ||
                     (ex_is_div && div_ph == MD_STEP && dv.counter >= 7'd64) || ex_div_by0;
   wire ex_is_dly  = ex_valid && (ex_d.uop == U_DELAY);
   wire dly_done   = dly_on && (dly_cnt >= dly_max);
   wire ex_is_lcd  = ex_valid && (ex_d.uop == U_LCD);
   wire ex_busy    = (ex_is_md && !md_done) || (ex_is_dly && !dly_done)
                  || (ex_is_lcd && !lcd_ready);

   wire [63:0] w_div_abs_a   = (ex_d.sgn && ex_a[63]) ? (~ex_a + 64'd1) : ex_a;
   wire [63:0] w_div_abs_b   = (ex_d.sgn && ex_b[63]) ? (~ex_b + 64'd1) : ex_b;
   wire [63:0] w_div_shifted = {dv.remainder[62:0], dv.dividend[63]};
   wire [64:0] w_div_trial   = {1'b0, w_div_shifted} - {1'b0, dv.divisor};
   wire        w_div_borrow  = w_div_trial[64];

   function automatic logic [6:0] f_clz64(input logic [63:0] val);
      f_clz64 = 7'd64;
      for (int i = 0; i < 64; i++) if (val[i]) f_clz64 = 7'd63 - 7'(i);
   endfunction
   function automatic logic [6:0] f_ctz64(input logic [63:0] val);
      f_ctz64 = 7'd64;
      for (int i = 63; i >= 0; i--) if (val[i]) f_ctz64 = 7'(i);
   endfunction
   function automatic logic [6:0] f_popcnt64(input logic [63:0] val);
      f_popcnt64 = '0;
      for (int i = 0; i < 64; i++) f_popcnt64 = f_popcnt64 + 7'(val[i]);
   endfunction
   function automatic logic [63:0] f_bitrev64(input logic [63:0] val);
      for (int i = 0; i < 64; i++) f_bitrev64[63-i] = val[i];
   endfunction

   // ---------------------------------------------------------- EX combinational
   logic [63:0] exo_result;
   flags_t      exo_fval;
   flags_t      exo_fmask;         // = ex_d.fmask (value side computed here)
   logic        exo_taken;         // EX redirect (branch/call taken, RESET)
   logic [31:0] exo_target;
   logic [31:0] exo_eaddr;
   logic [63:0] exo_wdata;         // lane-replicated store data
   logic [7:0]  exo_be;
   logic [63:0] exo_raw;           // raw store value (trace)
   logic [31:0] exo_sp_new;
   logic        exo_sp_we;
   logic [31:0] exo_iaddr;         // port ISSUE address (aligned per f_ld_idx)

   wire [63:0] ex_imm    = ex_d.sgn ? {{32{ex_var1[31]}}, ex_var1} : {32'b0, ex_var1};
   wire [63:0] ex_alu_b  = (ex_op[29:26] == 4'h5) ? 64'd1              // INC/DEC
                         : (ex_d.len == 2'b01)    ? ex_b : ex_imm;
   wire [5:0]  ex_shcnt  = ex_d.shsrc ? ex_d.shn : ex_b[5:0];
   wire [1:0]  ex_bcvt_t = f_cond_eval(flags, ex_d.bcond, ex_d.binv);

   always_comb begin : ex_logic
      logic [64:0]        sum;
      logic               is_sub, cin;
      logic signed [63:0] sa, sb;
      logic [63:0]        val, q_raw;
      logic               q_neg;
      logic [4:0]         bx_start, bx_len;
      logic [31:0]        bx_mask, bx_res;
      exo_result = '0;  exo_fval = '0;  exo_fmask = ex_d.fmask;
      exo_taken  = 1'b0; exo_target = '0;
      exo_eaddr  = '0;  exo_wdata = '0; exo_be = 8'hFF; exo_raw = '0;
      exo_sp_new = sp;  exo_sp_we = ex_d.sp_wr;
      // effective address (loads/stores)
      case (ex_d.mmode)
         2'b00: exo_eaddr = ex_a[31:0];
         2'b01: exo_eaddr = ex_a[31:0] + ex_var1;
         2'b10: exo_eaddr = ex_var1;
         2'b11: exo_eaddr = ex_a[31:0] + ex_b[31:0];
      endcase
      case (ex_d.uop)
         U_ALU: begin
            is_sub = (ex_d.sub == 4'd1) || (ex_d.sub == 4'd3) || (ex_d.sub == 4'd7);
            cin    = (ex_d.sub == 4'd2 || ex_d.sub == 4'd3) ? flags.carry : 1'b0;
            case (ex_d.sub)
               4'd4: exo_result = ex_a & ex_alu_b;
               4'd5: exo_result = ex_a | ex_alu_b;
               4'd6: exo_result = ex_a ^ ex_alu_b;
               default: begin
                  if (is_sub) sum = {1'b0, ex_a} - {1'b0, ex_alu_b} - {64'b0, cin};
                  else        sum = {1'b0, ex_a} + {1'b0, ex_alu_b} + {64'b0, cin};
                  exo_result        = sum[63:0];
                  exo_fval.carry    = sum[64];
                  exo_fval.zero     = (sum[63:0] == 64'b0);
                  exo_fval.sign     = sum[63];
                  exo_fval.overflow = is_sub ? ((ex_a[63] != ex_alu_b[63]) && (sum[63] != ex_a[63]))
                                             : ((ex_a[63] == ex_alu_b[63]) && (sum[63] != ex_a[63]));
               end
            endcase
         end
         U_BOOL: begin
            sa = ex_a; sb = ex_alu_b;
            case (cmp_op_e'(ex_d.sub))
               CMP_EQ:   exo_result = (ex_a == ex_alu_b) ? 64'b1 : 64'b0;
               CMP_NE:   exo_result = (ex_a != ex_alu_b) ? 64'b1 : 64'b0;
               CMP_LT:   exo_result = (sa < sb)   ? 64'b1 : 64'b0;
               CMP_LE:   exo_result = (sa <= sb)  ? 64'b1 : 64'b0;
               CMP_GT:   exo_result = (sa > sb)   ? 64'b1 : 64'b0;
               CMP_GE:   exo_result = (sa >= sb)  ? 64'b1 : 64'b0;
               CMP_ULT:  exo_result = (ex_a < ex_alu_b)  ? 64'b1 : 64'b0;
               CMP_ULE:  exo_result = (ex_a <= ex_alu_b) ? 64'b1 : 64'b0;
               CMP_UGT:  exo_result = (ex_a > ex_alu_b)  ? 64'b1 : 64'b0;
               CMP_UGE:  exo_result = (ex_a >= ex_alu_b) ? 64'b1 : 64'b0;
               CMP_MIN:  exo_result = (sa < sb) ? ex_a : ex_alu_b;
               CMP_MAX:  exo_result = (sa > sb) ? ex_a : ex_alu_b;
               CMP_MINU: exo_result = (ex_a < ex_alu_b) ? ex_a : ex_alu_b;
               default:  exo_result = (ex_a > ex_alu_b) ? ex_a : ex_alu_b; // MAXU
            endcase
         end
         U_SHIFT: begin
            case (ex_d.sub)
               4'd0:  exo_result = ex_a << ex_shcnt;
               4'd1:  exo_result = ex_a >> ex_shcnt;
               4'd2:  exo_result = $signed(ex_a) >>> ex_shcnt;
               4'd3:  exo_result = (ex_a << ex_shcnt) | (ex_a >> (64 - ex_shcnt));
               4'd4:  exo_result = (ex_a >> ex_shcnt) | (ex_a << (64 - ex_shcnt));
               4'd8:  exo_result = ex_a |  (64'b1 << ex_shcnt);
               4'd9:  exo_result = ex_a & ~(64'b1 << ex_shcnt);
               4'd10: exo_result = ex_a ^  (64'b1 << ex_shcnt);
               default: exo_result = {63'b0, ex_a[ex_b[5:0]]};   // BTSTRR
            endcase
            exo_fval.zero = (exo_result == 64'b0);
         end
         U_ROT1: begin
            case (ex_d.sub)
               4'd3: begin exo_result = {ex_a[62:0], ex_a[63]};     exo_fval.carry = ex_a[63]; end
               4'd4: begin exo_result = {ex_a[0], ex_a[63:1]};      exo_fval.carry = ex_a[0];  end
               4'd5: begin exo_result = {ex_a[62:0], flags.carry};  exo_fval.carry = ex_a[63]; end
               default: begin exo_result = {flags.carry, ex_a[63:1]}; exo_fval.carry = ex_a[0]; end
            endcase
            exo_fval.zero = (exo_result == 64'b0);
         end
         U_BTSTN: exo_fval.zero = ~ex_a[ex_d.shn];
         U_UNARY: begin
            case (ex_d.sub)
               4'd0:  exo_result = ex_a;
               4'd1:  exo_result = ~ex_a + 64'd1;
               4'd2:  exo_result = ~ex_a;
               4'd3:  begin
                  exo_result        = ex_a[63] ? (~ex_a + 64'd1) : ex_a;
                  exo_fval.overflow = (ex_a == 64'h8000000000000000);
               end
               4'd4:  exo_result = (ex_d.unsz == 2'd0) ? {{56{ex_a[7]}},  ex_a[7:0]}
                                 : (ex_d.unsz == 2'd1) ? {{48{ex_a[15]}}, ex_a[15:0]}
                                                       : {{32{ex_a[31]}}, ex_a[31:0]};
               4'd5:  exo_result = (ex_d.unsz == 2'd0) ? {56'b0, ex_a[7:0]}
                                 : (ex_d.unsz == 2'd1) ? {48'b0, ex_a[15:0]}
                                                       : {32'b0, ex_a[31:0]};
               4'd6:  exo_result = {ex_a[7:0], ex_a[15:8], ex_a[23:16], ex_a[31:24],
                                    ex_a[39:32], ex_a[47:40], ex_a[55:48], ex_a[63:56]};
               4'd7:  exo_result = f_bitrev64(ex_a);
               4'd8:  exo_result = {57'b0, f_popcnt64(ex_a)};
               4'd9:  exo_result = {57'b0, f_clz64(ex_a)};
               4'd10: exo_result = {57'b0, f_ctz64(ex_a)};
               default: exo_result = {flags.zero, flags.zero, flags.carry, flags.overflow, 60'b0}; // GETF
            endcase
            exo_fval.zero = (exo_result == 64'b0);
            if (ex_d.sub == 4'd4) exo_fval.sign = exo_result[63];
         end
         U_LEA:   exo_result = {32'b0, ex_pc + ex_var1};
         U_MOV:   exo_result = ex_imm;
         U_MOV64: exo_result = {ex_var2, ex_var1};
         U_BEXTR: begin
            bx_start = ex_var1[4:0];  bx_len = ex_var1[12:8];
            bx_mask  = (32'hFFFFFFFF >> (32 - bx_len));
            bx_res   = (ex_a >> bx_start) & bx_mask;
            exo_result = {32'b0, bx_res};
            exo_fval.zero = (exo_result == 64'b0);
         end
         U_BDEP: begin
            bx_start = ex_var1[4:0];  bx_len = ex_var1[12:8];
            bx_mask  = (32'hFFFFFFFF >> (32 - bx_len)) << bx_start;
            bx_res   = (ex_b[31:0] << bx_start) & bx_mask;
            exo_result = {32'b0, (ex_a[31:0] & ~bx_mask) | bx_res};
         end
         U_STORE: begin
            exo_raw = ex_c;
            case (ex_d.msz)
               2'd0: begin
                  exo_wdata = {8{ex_c[7:0]}};
                  exo_be    = 8'b0000_0001 << exo_eaddr[2:0];
               end
               2'd1: begin
                  exo_eaddr = {exo_eaddr[31:1], 1'b0};
                  exo_wdata = {4{ex_c[15:0]}};
                  exo_be    = 8'b0000_0011 << {exo_eaddr[2:1], 1'b0};
               end
               2'd2: begin
                  exo_eaddr = {exo_eaddr[31:2], 2'b00};
                  exo_wdata = {ex_c[31:0], ex_c[31:0]};
                  exo_be    = exo_eaddr[2] ? 8'b1111_0000 : 8'b0000_1111;
               end
               default: begin
                  if (ex_d.malign) exo_eaddr = {exo_eaddr[31:3], 3'b000};
                  exo_wdata = ex_c;  exo_be = 8'hFF;
               end
            endcase
         end
         U_PUSH: begin
            exo_eaddr  = sp - 32'd8;
            exo_wdata  = (ex_d.len == 2'b01) ? ex_a
                       : (ex_d.len == 2'b10) ? {32'b0, ex_var1} : {ex_var2, ex_var1};
            exo_raw    = exo_wdata;  exo_be = 8'hFF;
            exo_sp_new = sp - 32'd8;
         end
         U_POP, U_RET, U_IRET: begin
            exo_eaddr  = sp;
            exo_sp_new = sp + 32'd8;
         end
         U_GETSP: exo_result = {32'b0, sp};
         U_SETSP: exo_sp_new = ex_a[31:0];
         U_ADDSP: exo_sp_new = sp + ex_var1;   // sext imm32 added to 32-bit SP
         U_JMP: begin
            exo_taken  = ex_bcvt_t[0];
            exo_target = ex_d.brind ? ex_b[31:0]
                       : ex_d.brel  ? (ex_pc + ex_var1) : ex_var1;
         end
         U_CALL: begin
            exo_taken  = ex_bcvt_t[0];
            exo_target = ex_d.brind ? ex_b[31:0]
                       : ex_d.brel  ? (ex_pc + ex_var1) : ex_var1;
            exo_eaddr  = sp - 32'd8;
            exo_wdata  = {32'b0, ex_pc_next};   // return address
            exo_raw    = exo_wdata;  exo_be = 8'hFF;
            exo_sp_new = sp - 32'd8;
            exo_sp_we  = ex_bcvt_t[0];          // only a taken call touches SP/mem
         end
         U_MUL: begin
            exo_result    = ex_d.mdhigh ? mul_pipe2[127:64] : mul_pipe2[63:0];
            exo_fval.zero = (exo_result == 64'd0);
            exo_fval.sign = exo_result[63];
            if (ex_d.mdhigh)      exo_fval.overflow = 1'b0;
            else if (ex_d.mduns)  exo_fval.overflow = (mul_pipe2[127:64] != 64'd0);
            else                  exo_fval.overflow = (mul_pipe2[127:64] != {64{mul_pipe2[63]}});
         end
         U_DIV, U_MOD: begin
            if (ex_div_by0) begin
               exo_result        = ex_div_mod ? ex_a : 64'hFFFFFFFFFFFFFFFF;
               exo_fmask         = '0;
               exo_fmask.overflow = 1'b1;
               exo_fval.overflow = 1'b1;
            end else begin
               q_raw  = ex_div_mod ? dv.remainder : dv.quotient;
               q_neg  = dv.is_signed && (ex_div_mod ? dv.sign_r : dv.sign_q);
               exo_result        = q_neg ? (~q_raw + 64'd1) : q_raw;
               exo_fval.zero     = (q_raw == 64'd0);
               exo_fval.overflow = 1'b0;
            end
         end
         U_RESET: begin exo_taken = 1'b1; exo_target = 32'h4; end
         default: ;   // NOP/HALT/WAIT/TRAP/DELAY/LCD/LOAD/LOAD32/ILL: nothing in EX
      endcase
      // port issue address (loads align per f_ld_idx; stores already aligned)
      exo_iaddr = exo_eaddr;
      if (ex_d.uop == U_LOAD) case (ex_d.msz)
         2'd1:    exo_iaddr = {exo_eaddr[31:1], 1'b0};
         2'd2:    exo_iaddr = {exo_eaddr[31:2], 2'b00};
         2'd3:    exo_iaddr = ex_d.malign ? {exo_eaddr[31:3], 3'b000} : exo_eaddr;
         default: exo_iaddr = exo_eaddr;
      endcase
   end

   // mul/div operand B mux happens at dispatch: ex_b holds rs2 (len1) or
   // the sign/zero-extended imm32 (len2) — see the dispatch below.

   // ------------------------------------------------ MEM / IF port engines
   // mem_xc: 0=idle 1=wait-ready 2=settle(2nd MEMGET32 read, mirrors
   // f_memget32's dead cycle) 3=wait-ready-2. if_xc: fill in flight.
   logic [1:0] mem_xc;
   logic       if_xc;
   logic [28:0] if_req_dw;
   logic        if_rebase;

   wire mem_is_ld32  = mem_valid && (mem_d.uop == U_LOAD32);
   wire mem_port_rd  = mem_valid && (mem_d.uop == U_LOAD || mem_d.uop == U_POP ||
                                     mem_d.uop == U_RET  || mem_d.uop == U_IRET ||
                                     mem_is_ld32);
   wire mem_port_wr  = mem_valid && (mem_d.uop == U_STORE || mem_d.uop == U_PUSH ||
                                     (mem_d.uop == U_CALL && mem_sp_we));
   wire mem_port_op  = mem_port_rd || mem_port_wr;
   // MEMGET32 spanning a dword: served by the read lookahead when the next
   // dword is in the same line, else a second read (f_memget32 exactly).
   wire ld32_span    = mem_is_ld32 && (mem_eaddr[2:0] > 3'd4);
   wire ld32_need2   = ld32_span && !m_next_valid;
   wire mem_done_now = (mem_xc == 2'd1 && m_ready && !ld32_need2) ||
                       (mem_xc == 2'd3 && m_ready);
   wire mem_busy     = mem_port_op && !mem_done_now;
   wire mem_is_read  = mem_port_rd;
   wire mem_is_write = mem_port_wr;

   // load-value extraction (f_ld_idx / f_memget32 lane math)
   logic [63:0] mem_ldval;
   always_comb begin
      logic [7:0]  b8;
      logic [15:0] h16;
      logic [31:0] w32;
      b8  = m_rdata[(mem_eaddr[2:0] * 8)  +: 8];
      h16 = m_rdata[(mem_eaddr[2:1] * 16) +: 16];
      w32 = mem_eaddr[2] ? m_rdata[63:32] : m_rdata[31:0];
      mem_ldval = m_rdata;
      if (mem_is_ld32) begin
         case (mem_eaddr[2:0])
            3'd0: mem_ldval = {32'b0, m_rdata[31:0]};
            3'd1: mem_ldval = {32'b0, m_rdata[39:8]};
            3'd2: mem_ldval = {32'b0, m_rdata[47:16]};
            3'd3: mem_ldval = {32'b0, m_rdata[55:24]};
            3'd4: mem_ldval = {32'b0, m_rdata[63:32]};
            // spanning: phase-1 lookahead (rdata_next) or phase-3 second read
            // combined with the stashed dw0
            3'd5: mem_ldval = (mem_xc == 2'd3) ? {32'b0, m_rdata[7:0],  mem_dw0[63:40]}
                                               : {32'b0, m_rdata_next[7:0],  m_rdata[63:40]};
            3'd6: mem_ldval = (mem_xc == 2'd3) ? {32'b0, m_rdata[15:0], mem_dw0[63:48]}
                                               : {32'b0, m_rdata_next[15:0], m_rdata[63:48]};
            default: mem_ldval = (mem_xc == 2'd3) ? {32'b0, m_rdata[23:0], mem_dw0[63:56]}
                                                  : {32'b0, m_rdata_next[23:0], m_rdata[63:56]};
         endcase
      end else if (mem_valid && mem_d.uop == U_LOAD) begin
         case (mem_d.msz)
            2'd0: mem_ldval = mem_d.sgn ? {{56{b8[7]}},   b8}  : {56'b0, b8};
            2'd1: mem_ldval = mem_d.sgn ? {{48{h16[15]}}, h16} : {48'b0, h16};
            2'd2: mem_ldval = mem_d.sgn ? {{32{w32[31]}}, w32} : {32'b0, w32};
            default: mem_ldval = m_rdata;
         endcase
      end
   end

   // IRET context split (klauss_pkg f_iret bit layout)
   flags_t iret_fval;
   always_comb begin
      iret_fval          = '0;
      iret_fval.zero     = m_rdata[38];
      iret_fval.carry    = m_rdata[36];
      iret_fval.overflow = m_rdata[35];
      iret_fval.sign     = m_rdata[34];
   end

   // LCD port
   assign lcd_byte = (ex_d.len == 2'b01) ? ex_a[7:0] : ex_var1[7:0];
   assign lcd_dc   = ex_d.lcd_is_data;
   assign lcd_dv   = ex_is_lcd && lcd_ready;

   // ================================================================== clocked
   always_ff @(posedge clk) begin
      if (rst) begin
         pc <= '0; running <= 1'b0; fetch_halt <= 1'b0;
         id_valid <= 1'b0; ex_valid <= 1'b0; mem_valid <= 1'b0; wb_valid <= 1'b0;
         flags <= '0; sp <= 32'h0800_0000; int_mask <= 4'h0;
         mul_cnt <= '0; div_ph <= MD_IDLE; dv <= '0;
         dly_on <= 1'b0; dly_cnt <= '0; dly_max <= '0;
         parked <= 1'b0; park_kind <= '0; park_pc <= '0; park_op <= '0;
         ret_valid <= 1'b0; ret_wr <= 1'b0; lcd_rst_n <= 1'b0;
         mem_xc <= '0; if_xc <= 1'b0; ifb_val <= 2'b00;
         m_read_DV <= 1'b0; m_write_DV <= 1'b0; m_addr <= '0; m_be <= 8'hFF;
         for (int i = 0; i < 16; i++) rf[i] <= 64'b0;
      end else begin
         if (start) begin
            pc <= start_pc; running <= 1'b1; fetch_halt <= 1'b0; parked <= 1'b0;
            id_valid <= 1'b0; ex_valid <= 1'b0; mem_valid <= 1'b0; wb_valid <= 1'b0;
            ifb_val <= 2'b00;
         end

         // ---------------- WB: the single retire point ----------------
         ret_valid <= 1'b0;  ret_wr <= 1'b0;
         if (wb_valid) begin
            if (wb_wreg)    rf[wb_rd] <= wb_value;
            if (wb_fmask != '0)
               flags <= (flags & ~wb_fmask) | (wb_fval & wb_fmask);
            if (wb_sp_we)   sp <= wb_sp_new;
            if (wb_mask_we) int_mask <= wb_mask_new;
            if (wb_park) begin
               parked   <= 1'b1;
               park_kind <= wb_park_kind;
               park_pc  <= wb_pc;  park_op <= wb_op;
               if (wb_park_kind != 3'd2) begin   // illegal: no trace line
                  ret_valid <= 1'b1; ret_pc <= wb_pc; ret_op <= wb_op;
               end
            end else begin
               ret_valid <= 1'b1; ret_pc <= wb_pc; ret_op <= wb_op;
            end
            ret_wr      <= wb_wr;
            ret_wr_addr <= wb_wr_addr;  ret_wr_be <= wb_wr_be;  ret_wr_raw <= wb_wr_raw;
         end

         // ---------------- EX free-running units ----------------
         mul_a_q   <= {(ex_d.mduns ? 1'b0 : ex_a[63]), ex_a};
         mul_b_q   <= {(ex_d.mduns ? 1'b0 : ex_b[63]), ex_b};
         mul_pipe1 <= $signed(mul_a_q) * $signed(mul_b_q);
         mul_pipe2 <= mul_pipe1;

         case (div_ph)
            MD_IDLE: if (ex_is_div && ex_b != 64'd0) begin
               dv.dividend  <= w_div_abs_a;   dv.divisor   <= w_div_abs_b;
               dv.quotient  <= '0;            dv.remainder <= '0;   dv.counter <= '0;
               dv.sign_q    <= ex_a[63] ^ ex_b[63];
               dv.sign_r    <= ex_a[63];
               dv.is_signed <= ex_d.sgn;
               dv.op        <= ex_div_mod ? DIV_OP_MOD : DIV_OP_DIV;
               div_ph       <= MD_PREP;
            end
            MD_PREP: begin : divide_prep
               logic [6:0] prep_clz;
               prep_clz = f_clz64(dv.dividend);
               if (prep_clz[6]) dv.counter <= 7'd64;
               else begin
                  dv.dividend <= dv.dividend << prep_clz[5:0];
                  dv.counter  <= {1'b0, prep_clz[5:0]};
               end
               div_ph <= MD_STEP;
            end
            MD_STEP: begin
               if (dv.counter < 7'd64) begin
                  if (!w_div_borrow) begin
                     dv.remainder <= w_div_trial[63:0];
                     dv.quotient  <= {dv.quotient[62:0], 1'b1};
                  end else begin
                     dv.remainder <= w_div_shifted;
                     dv.quotient  <= {dv.quotient[62:0], 1'b0};
                  end
                  dv.dividend <= {dv.dividend[62:0], 1'b0};
                  dv.counter  <= dv.counter + 7'd1;
               end else if (!mem_busy) begin
                  // finish only when the pipe can actually advance the div out
                  // of EX — else md_done must stay asserted (a MEM-busy freeze
                  // on the finish cycle would otherwise re-issue the divide).
                  dv.op  <= DIV_OP_NONE;
                  div_ph <= MD_IDLE;
               end
            end
            default: div_ph <= MD_IDLE;
         endcase

         if (ex_is_dly) begin
            if (!dly_on) begin
               dly_on  <= 1'b1;  dly_cnt <= '0;
               dly_max <= {(ex_d.len == 2'b01) ? ex_a[31:0] : ex_var1, 13'b0};
            end else if (dly_done) begin
               if (!mem_busy) dly_on <= 1'b0;   // same freeze-on-finish rule
            end else dly_cnt <= dly_cnt + 45'd1;
         end

         if (ex_valid && ex_d.uop == U_LCDRST) lcd_rst_n <= ex_var1[0];

         // ---------------- MEM port engine (extra_clock pattern) ----------------
         if (mem_port_op) begin
            case (mem_xc)
               2'd0: if (!if_xc) begin        // wait for the port (IF may hold it)
                  m_addr     <= mem_iaddr;
                  m_wdata    <= mem_wdata;
                  m_be       <= mem_be;
                  m_read_DV  <= mem_port_rd;
                  m_write_DV <= mem_port_wr;
                  mem_xc     <= 2'd1;
               end
               2'd1: if (m_ready) begin
                  if (ld32_need2) begin        // cross-line MEMGET32: 2nd read
                     mem_dw0 <= m_rdata;
                     m_addr  <= {mem_eaddr[31:3], 3'b000} + 32'd8;
                     mem_xc  <= 2'd2;          // settle cycle (f_memget32 shape)
                  end else begin
                     m_read_DV <= 1'b0;  m_write_DV <= 1'b0;
                     mem_xc    <= 2'd0;
                  end
               end
               2'd2: mem_xc <= 2'd3;
               default: if (m_ready) begin     // 2'd3
                  m_read_DV <= 1'b0;
                  mem_xc    <= 2'd0;
               end
            endcase
         end

         // ---------------- IF fill engine (2-dword IFB window) ----------------
         if (if_xc) begin
            if (m_ready) begin
               if (if_rebase) begin
                  ifb_base  <= if_req_dw;
                  ifb_dw[0] <= m_rdata;       ifb_val[0] <= 1'b1;
                  ifb_dw[1] <= m_rdata_next;  ifb_val[1] <= m_next_valid;
               end else begin
                  ifb_dw[1] <= m_rdata;       ifb_val[1] <= 1'b1;
               end
               m_read_DV <= 1'b0;
               if_xc     <= 1'b0;
            end
         end else if (running && !fetch_halt && !parked && !fetch_ok
                      && !mem_port_op && mem_xc == 2'd0) begin
            m_addr    <= {miss_dw, 3'b000};
            m_read_DV <= 1'b1;
            if_req_dw <= miss_dw;
            if_rebase <= !in0;
            if_xc     <= 1'b1;
         end

         // ---------------- MEM -> WB ----------------
         if (mem_busy) begin
            wb_valid <= 1'b0;
         end else begin
            wb_valid   <= mem_valid;
            wb_pc      <= mem_pc;    wb_op <= mem_op;   wb_rd <= mem_d.rd;
            wb_wreg    <= mem_d.wreg;
            wb_value   <= mem_is_read ? mem_ldval : mem_result;
            wb_fval    <= (mem_valid && mem_d.uop == U_IRET) ? iret_fval : mem_fval;
            wb_fmask   <= mem_fmask;
            wb_sp_new  <= mem_sp_new;  wb_sp_we <= mem_valid && mem_sp_we;
            wb_mask_we <= mem_valid && (mem_d.uop == U_IRET);
            wb_mask_new<= m_rdata[42:39];
            wb_park    <= mem_valid && (mem_d.uop == U_HALT || mem_d.uop == U_TRAP ||
                                        mem_d.uop == U_WAIT || mem_d.uop == U_ILL);
            wb_park_kind <= (mem_d.uop == U_HALT) ? 3'd0 : (mem_d.uop == U_TRAP) ? 3'd1
                          : (mem_d.uop == U_ILL)  ? 3'd2 : 3'd3;
            wb_wr      <= mem_is_write;
            wb_wr_addr <= mem_iaddr;  wb_wr_be <= mem_be;  wb_wr_raw <= mem_result;
            // RET / IRET redirect + fetch resume at MEM completion
            if (mem_valid && (mem_d.uop == U_RET || mem_d.uop == U_IRET)) begin
               pc <= m_rdata[31:0];
               fetch_halt <= 1'b0;
            end

            // ---------------- EX -> MEM ----------------
            if (ex_busy) begin
               mem_valid <= 1'b0;
               if (ex_is_mul) mul_cnt <= mul_cnt + 2'd1;
            end else begin
               mem_valid  <= ex_valid;   // not-taken CALL still retires (trace line)
               mem_pc     <= ex_pc;    mem_op <= ex_op;   mem_d <= ex_d;
               mem_result <= (ex_d.uop == U_STORE || ex_d.uop == U_PUSH ||
                              ex_d.uop == U_CALL) ? exo_raw : exo_result;
               mem_fval   <= exo_fval; mem_fmask <= ex_valid ? exo_fmask : '0;
               mem_eaddr  <= exo_eaddr;  mem_iaddr <= exo_iaddr;
               mem_wdata  <= exo_wdata;  mem_be <= exo_be;
               mem_sp_new <= exo_sp_new; mem_sp_we <= ex_valid && exo_sp_we;

               // EX redirect: taken branch/call, RESET
               if (ex_valid && exo_taken) begin
                  pc         <= exo_target;
                  id_valid   <= 1'b0;
                  ex_valid   <= 1'b0;
                  fetch_halt <= 1'b0;   // RESET resumes here
               end else if (raw_stall) begin
                  ex_valid <= 1'b0;     // bubble; hold ID/pc
               end else begin
                  ex_valid   <= id_valid && !fetch_halt;
                  ex_pc      <= id_pc;   ex_op <= id_op;
                  ex_var1    <= id_var1; ex_var2 <= id_var2;
                  ex_pc_next <= id_pc_next;
                  ex_d       <= dec;
                  ex_a       <= rf[dec.rs1];
                  ex_c       <= rf[dec.rd];
                  // rs2 / imm operand mux for reg-vs-imm op forms (mul/div)
                  ex_b       <= (dec.len != 2'b01 &&
                                 (dec.uop == U_MUL || dec.uop == U_DIV || dec.uop == U_MOD ||
                                  dec.uop == U_BOOL || dec.uop == U_ALU))
                                ? (dec.sgn ? {{32{id_var1[31]}}, id_var1} : {32'b0, id_var1})
                                : rf[dec.rs2];
                  mul_cnt    <= 2'd0;
                  if (id_valid && !fetch_halt && dec.serialize) begin
                     fetch_halt <= 1'b1;   // stop fetching behind a serializer
                     id_valid   <= 1'b0;
                  end else if (running && !fetch_halt && fetch_ok) begin
                     id_valid <= 1'b1;
                     id_op    <= w_op;  id_var1 <= w_var1;  id_var2 <= w_var2;
                     id_pc    <= pc;
                     pc       <= pc + {26'b0, w_op[31:30], 2'b00};  // LEN field
                  end else begin
                     id_valid <= 1'b0;    // IF window miss: bubble, engine fills
                  end
               end
            end
         end
      end
   end

endmodule

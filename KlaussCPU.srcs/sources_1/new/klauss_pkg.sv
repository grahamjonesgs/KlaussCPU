// ============================================================================
// klauss_pkg — shared type and constant definitions for the KlaussCPU core.
//
// Centralises the CPU-wide enums/constants that were previously declared inside
// KlaussCPU.v and reached the task .vh includes by include-scoping.  Modules and
// testbenches `import klauss_pkg::*;` to share one authoritative definition
// instead of re-declaring (e.g. tb_div_prep / tb_predecode no longer need their
// own copies).  Purely organisational — synthesises identically.
// ============================================================================
package klauss_pkg;

   // --- FSM state ----------------------------------------------------------
   // One-hot FSM packed into a 34-bit enum so the named states appear in the
   // waveform and can be reasoned about by name.  Values are the EXACT one-hot
   // bit patterns the design has always used (low 32 states in bits 0..31,
   // ALU_FINISH = bit 32, DIVIDE_PREP = bit 33).  Encoding is left to Vivado's
   // FSM inference, which derives one-hot from these constants exactly as it did
   // from the original localparams, so the synthesized hardware is unchanged.
   // Since the multicycle CPU was removed only the SoC states are reached
   // (NO_PROGRAM boot copy, loader, START_WAIT, PIPE_RUN, HCF_*, HALTED*); the
   // rest are kept so the crash dump's SM= codes stay stable (CRASH_DUMP.md).
   typedef enum logic [33:0] {
      OPCODE_REQUEST = 34'h1, OPCODE_FETCH = 34'h2, OPCODE_FETCH2 = 34'h4,
      VAR1_FETCH = 34'h8, PIPE_RUN = 34'h10, WAITING = 34'h20,  // PIPE_RUN: the 5-stage pipeline_core owns execution (reuses the retired VAR1_FETCH2 bit); WAITING: interruptible core-suspend (WAIT opcode)
      START_WAIT = 34'h40, UART_DELAY = 34'h80, OPCODE_EXECUTE = 34'h100,
      HCF_1 = 34'h200, HCF_2 = 34'h400, HCF_3 = 34'h800, HCF_4 = 34'h1_000,
      NO_PROGRAM = 34'h2_000, LOAD_START = 34'h4_000, LOADING_BYTE = 34'h8_000,
      LOAD_COMPLETE = 34'h10_000, LOAD_WAIT = 34'h20_000,
      DEBUG_DATA = 34'h40_000, DEBUG_DATA2 = 34'h80_000, DEBUG_DATA3 = 34'h100_000,
      DEBUG_WAIT = 34'h200_000,
      MULTIPLY_CALC      = 34'h0040_0000,  // DSP pipeline stage 2 (MREG)
      MULTIPLY_PIPE      = 34'h0100_0000,  // DSP pipeline stage 3 (PREG)
      MULTIPLY_WRITEBACK = 34'h0080_0000,  // Write result
      MULTIPLY_SETUP     = 34'h4000_0000,  // Setup operands for multiply
      WRITEBACK          = 34'h0200_0000,  // Register file writeback stage
      HALTED             = 34'h0400_0000,  // CPU halted, waiting for reset
      DIVIDE_STEP        = 34'h0800_0000,  // Division iteration state
      HALTED_BREAK       = 34'h1000_0000,  // Sending UART break before halt
      MULTIPLY_BREG      = 34'h2000_0000,  // DSP pipeline stage 1 (AREG/BREG)
      HCF_DUMP           = 34'h8000_0000,  // Crash dump UART emission (sub-state inside r_hcf_dump_phase / r_hcf_dump_sub)
      // ALU_FINISH (bit 32) — pipeline register for the 64-bit ALU compute path.
      // Arithmetic / compare tasks register their result + flags into r_alu_pipe_*
      // (one cycle), then ALU_FINISH copies the intermediates out to the
      // architectural flag regs and r_writeback_value (next cycle). Splits the long
      //   r_reg_port_b → 16 CARRY4 → 7 LUT6 → r_carry_flag
      // path into two shorter stages for timing closure.
      ALU_FINISH         = 34'h1_0000_0000,
      // DIVIDE_PREP (bit 33) — divide normalization: one cycle between the div task
      // and DIVIDE_STEP that pre-shifts the dividend by its leading-zero count, so
      // the iteration loop runs (64 - clz) steps instead of a fixed 64. Bit-identical
      // result — the skipped iterations shift zeros into the remainder and emit 0
      // quotient bits. Its own state keeps the CLZ + 64-bit shifter off both the
      // OPCODE_EXECUTE decode region and DIVIDE_STEP's trial-subtract carry chain
      // (the documented critical paths).
      DIVIDE_PREP        = 34'h2_0000_0000
   } e_sm_t;

   // --- Crash-dump error codes (r_error_code) ------------------------------
   localparam ERR_INV_OPCODE = 8'h1, ERR_INV_FSM_STATE = 8'h2, ERR_STACK = 8'h3;
   localparam ERR_DATA_LOAD = 8'h4, ERR_CHECKSUM_LOAD = 8'h5, ERR_OVERFLOW = 8'h6;
   localparam ERR_SEG_WRITE_TO_CODE = 'h7, ERR_SEG_EXEC_DATA = 'h8;
   localparam ERR_TRAP = 8'h9;        // Explicit software trap (TRAP opcode)

   // --- Crash dump phase boundaries (r_hcf_dump_phase) ---------------------
   // Each "phase" emits one UART line. Kept as localparams (not an enum) because
   // r_hcf_dump_phase is an arithmetic counter walking 0..47 (base + offset).
   localparam DUMP_HEADER     = 7'd0;
   localparam DUMP_ERR_PC     = 7'd1;
   localparam DUMP_OPC_SP     = 7'd2;
   localparam DUMP_V1_V2      = 7'd3;
   localparam DUMP_V1H        = 7'd4;   // V1H=xxxxxxxx — hi32 of 64-bit immediate (DRAM read at PC+8)
   localparam DUMP_OPCM       = 7'd5;   // OPCM=xxxxxxxx — DRAM-side re-read at PC; differ from OPC ⇒ cache mismatch
   localparam DUMP_SM         = 7'd6;   // SM=xxxxxxxxx — FSM state (34-bit one-hot, 9 hex digits)
   localparam DUMP_IV0        = 7'd7;   // IV0=xxxxxxxx — timer ISR vector (r_interrupt_table[0])
   localparam DUMP_FLAGS_A    = 7'd8;   // Z E C V
   localparam DUMP_FLAGS_B    = 7'd9;   // S L U
   localparam DUMP_INSTR      = 7'd10;  // INSTR=NNNNNNNN — instructions committed since program load
   localparam DUMP_REG_BASE   = 7'd11;  // R0..RF → phases 11..26
   localparam DUMP_STACK_BASE = 7'd27;  // S0..S3 → phases 27..30 (each preceded by a DDR2 read)
   localparam DUMP_TRACE_BASE = 7'd31;  // T0..TF → phases 31..46 (newest-first)
   localparam DUMP_FOOTER     = 7'd47;  // last phase; on completion → HCF_2

   // --- Architectural condition flags (r_flags) ----------------------------
   // The 7 CPU condition flags as one packed struct.  Fields are written/read
   // individually (incl. inside the interrupt-context save concat and GETFLAGS),
   // so the field order is not load-bearing; it mirrors the crash-dump grouping
   // (Z E C V / S L U) for readability.
   typedef struct packed {
      logic zero;       // Z: last result == 0
      logic carry;      // C: carry / borrow out — x86 BORROW convention: SUB/CMP set C iff a<b (unsigned)
      logic overflow;   // V: signed overflow
      logic sign;       // S: sign of last result (bit 63)
      // E/L/U retired (flag-unification): the equal/less/ult conditions are now
      // DERIVED from Z/S/C/V — EQ=Z, signed LT=S^V, unsigned ULT=C (borrow) —
      // in f_cond_eval and on the ISA-visible flag word (IRQ save / GETF). Never
      // stored, so CMP and arithmetic share one 4-bit flags register.
   } flags_t;

   // --- Deferred-writeback bundle (r_wb) -----------------------------------
   // Deferred register writeback carried in cpu_state_t: the fetch/execute
   // overlap commits it in OPCODE_REQUEST (parallel with dispatch), with a
   // read-port forward mux covering the RAW hazard. Field is `rd` (not `reg`,
   // which is a keyword).
   typedef struct packed {
      logic [63:0] value;     // result to write back
      logic [3:0]  rd;        // destination register index
      logic        set_zero;  // set the zero flag from `value` at writeback
      logic        pending;   // a writeback is deferred/in-flight (RAW-forward source)
   } wb_t;

   // --- Pure helper functions (were functions.vh) --------------------------
   // No module-scope coupling — args in, value out — so they lift cleanly into
   // the package.

   // ASCII hex digit ('0'-'9','A'-'F') -> 4-bit nibble; any non-hex input -> 0x0
   function automatic [3:0] return_hex_from_ascii;
      input [7:0] ascii;
      begin
         case (ascii)
            8'h30:   return_hex_from_ascii = 4'h0;
            8'h31:   return_hex_from_ascii = 4'h1;
            8'h32:   return_hex_from_ascii = 4'h2;
            8'h33:   return_hex_from_ascii = 4'h3;
            8'h34:   return_hex_from_ascii = 4'h4;
            8'h35:   return_hex_from_ascii = 4'h5;
            8'h36:   return_hex_from_ascii = 4'h6;
            8'h37:   return_hex_from_ascii = 4'h7;
            8'h38:   return_hex_from_ascii = 4'h8;
            8'h39:   return_hex_from_ascii = 4'h9;
            8'h41:   return_hex_from_ascii = 4'hA;
            8'h42:   return_hex_from_ascii = 4'hB;
            8'h43:   return_hex_from_ascii = 4'hC;
            8'h44:   return_hex_from_ascii = 4'hD;
            8'h45:   return_hex_from_ascii = 4'hE;
            8'h46:   return_hex_from_ascii = 4'hF;
            default: return_hex_from_ascii = 4'h0;
         endcase
      end
   endfunction

   // 4-bit nibble -> uppercase ASCII hex digit ('0'-'9','A'-'F'); out-of-range -> '?' (0x3F)
   function automatic [7:0] return_ascii_from_hex;
      input [3:0] hex;
      begin
         case (hex)
            4'h0: return_ascii_from_hex = 8'h30;
            4'h1: return_ascii_from_hex = 8'h31;
            4'h2: return_ascii_from_hex = 8'h32;
            4'h3: return_ascii_from_hex = 8'h33;
            4'h4: return_ascii_from_hex = 8'h34;
            4'h5: return_ascii_from_hex = 8'h35;
            4'h6: return_ascii_from_hex = 8'h36;
            4'h7: return_ascii_from_hex = 8'h37;
            4'h8: return_ascii_from_hex = 8'h38;
            4'h9: return_ascii_from_hex = 8'h39;
            4'hA: return_ascii_from_hex = 8'h41;
            4'hB: return_ascii_from_hex = 8'h42;
            4'hC: return_ascii_from_hex = 8'h43;
            4'hD: return_ascii_from_hex = 8'h44;
            4'hE: return_ascii_from_hex = 8'h45;
            4'hF: return_ascii_from_hex = 8'h46;
            default: return_ascii_from_hex = 8'h3F;
         endcase
      end
   endfunction

   // f_predecode_len — encoded instruction length in BYTES (4/8/12) from the
   // 32-bit opcode alone, for the fetch pipeline. ISA encoding v2 carries the
   // word count in the LEN field (bits [31:30]: 01=1, 10=2, 11=3 words), so the
   // length is a field read — no enumerated table to keep in lock-step with the
   // dispatch. LEN=00 is architecturally illegal (any pre-v2 binary word);
   // return 4 so the fetch completes and the dispatch default traps it.
   function automatic [3:0] f_predecode_len;
      input [31:0] opcode;
      begin
         case (opcode[31:30])
            2'b10:   f_predecode_len = 4'd8;
            2'b11:   f_predecode_len = 4'd12;
            default: f_predecode_len = 4'd4;   // 01 = 1 word; 00 = illegal (traps at dispatch)
         endcase
      end
   endfunction


   // ========================================================================
   // CPU datapath state type + the unified next-state functions (moved out of the
   // KlaussCPU module so tb/tools can share them). Pure: (cpu_state_t s, args)
   // -> cpu_state_t. Order: types/enums first, then the f_* functions.
   // ========================================================================
   typedef enum logic [1:0] { DIV_OP_NONE, DIV_OP_DIV, DIV_OP_MOD } e_div_op_t;
   typedef struct packed {
      logic [63:0] dividend;
      logic [63:0] divisor;
      logic [63:0] quotient;
      logic [63:0] remainder;
      logic [6:0]  counter;
      logic        busy;
      logic        sign_q;      // sign of quotient
      logic        sign_r;      // sign of remainder
      logic        is_signed;
      e_div_op_t   op;          // none / div / mod
      logic [3:0]  dest_reg;    // destination register for the result
      logic        pc_inc;      // 0=PC+1, 1=PC+2
   } div_state_t;

   // ========================================================================
   // cpu_state_t — the CPU core datapath state bundled into one struct so the
   // execute tasks operate on it explicitly.  The tasks are `include'd and read/
   // write this module-scope `st` directly with non-blocking assignments — the
   // working, board-verified arrangement.
   //
   // WARNING — do NOT try to pass `st` into the tasks by `ref`: it was attempted
   // and PROVEN NON-VIABLE.  A non-blocking assignment through a `ref` argument is
   // illegal SystemVerilog (IEEE 1800 sec 10.3: no NBA to automatic vars; sec 12.4.2:
   // a ref arg is forbidden where automatic vars are) AND Vivado synthesizes it
   // silently into INCORRECT hardware (every program hung on its first memory write).
   // The supported way to give a task an explicit, package-able interface is the
   // next-state-FUNCTION form: function automatic cpu_state_t t_x(cpu_state_t s, ...)
   // that blocking-writes a local copy and returns it, applied via a single
   // st <= t_x(st, ...) NBA.  (ref post-mortem lives on branch sv-vh-task-lift.)
   // ========================================================================
   typedef struct packed {
      e_sm_t       SM;
      logic [31:0] PC;
      logic [31:0] SP;
      flags_t      flags;
      wb_t         wb;
      div_state_t  div;
      logic [63:0] alu_pipe_value;
      logic        alu_pipe_carry;
      logic        alu_pipe_overflow;
      logic        alu_pipe_mode;
      logic [31:0] mem_addr;
      logic        mem_read_DV;
      logic        mem_write_DV;
      logic [63:0] mem_write_data;
      logic [ 7:0] mem_byte_en;
      logic        mem_was_ready;
      logic [ 1:0] extra_clock;
      logic [63:0] mul_operand_a;
      logic [63:0] mul_operand_b;
      logic [ 3:0] mul_dest_reg;
      logic        mul_is_high;
      logic        mul_is_unsigned;
      logic        mul_is_immediate;
      logic [ 3:0] reg_1;
      logic [ 3:0] reg_2;
      logic [ 3:0] reg_dst;
      logic [ 7:0] error_code;
      logic [ 3:0] int_mask;
      logic [31:0] idx_base_addr;
      logic [31:0] seven_seg_value1;
      logic [31:0] seven_seg_value2;
      logic [15:0] led;
      logic [11:0] RGB_LED_1;
      logic [11:0] RGB_LED_2;
      logic [44:0] timeout_counter;
      logic [44:0] timeout_max;
      logic        timing_start;
      logic        rx_fifo_read;
   } cpu_state_t;

   // ========================================================================
   // f_alu — unified next-state ALU. ONE function for the whole
   // ALU op-class (RRR + reg-immediate + compares), selected by `op`. The caller
   // passes the two operands (extending the immediate sign/zero as the opcode
   // requires), the destination register index `rd`, and the next PC. The
   // general add/sub overflow formulas below reduce to each opcode's simplified
   // form when the immediate is zero-extended (b[63]==0), so no precision is lost.
   // Discipline (proven by the f_addr3 board test): seed n=s, reproduce the
   // rx_fifo_read default, read only args + snapshot s, blocking writes, return n.
   // Subsumes t_addr3/subr3/andr3/orr3/xorr3/addc3/subc3/cmprr3 +
   // add_value/minus_value/addi/and_reg_value/or_reg_value/xor_reg_value/
   // compare_reg_value/inc_reg/dec_reg.
   // ========================================================================
   typedef enum logic [2:0] {
      ALU_ADD, ALU_SUB, ALU_ADC, ALU_SBC, ALU_AND, ALU_OR, ALU_XOR, ALU_CMP
   } alu_op_e;

   function automatic cpu_state_t f_alu(cpu_state_t s, logic [63:0] a, logic [63:0] b,
                                        alu_op_e op, logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t  n;
      logic [64:0] sum;
      logic        cin;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.PC           = pc_next;
      case (op)
         ALU_AND, ALU_OR, ALU_XOR: begin
            // bitwise: direct register writeback, no flags, single-cycle
            n.wb.value   = (op == ALU_AND) ? (a & b) : (op == ALU_OR) ? (a | b) : (a ^ b);
            n.wb.rd      = rd;
            n.wb.pending = 1'b1;
            n.SM         = OPCODE_REQUEST;
         end
         ALU_CMP: begin
            // compare = SUB without register writeback: set Z/S/C/V (committed in
            // ALU_FINISH). C = sum[64] is the x86 borrow (1 iff a<b unsigned); V is
            // the signed-subtract overflow. EQ/LT/ULT are derived from Z/S/C/V in
            // f_cond_eval — no separate equal/less/ult storage.
            sum = {1'b0, a} - {1'b0, b};
            n.alu_pipe_value    = sum[63:0];
            n.alu_pipe_carry    = sum[64];
            n.alu_pipe_overflow = (a[63] != b[63]) && (sum[63] != a[63]) ? 1'b1 : 1'b0;
            n.alu_pipe_mode     = 1'b1;   // CMP: commit flags, no rd write
            n.SM                = ALU_FINISH;
         end
         default: begin
            // ADD / SUB / ADC / SBC : alu_pipe + carry/overflow flags via ALU_FINISH
            cin = (op == ALU_ADC || op == ALU_SBC) ? s.flags.carry : 1'b0;
            if (op == ALU_ADD || op == ALU_ADC)
               sum = {1'b0, a} + {1'b0, b} + {64'b0, cin};
            else
               sum = {1'b0, a} - {1'b0, b} - {64'b0, cin};
            n.alu_pipe_value = sum[63:0];
            n.alu_pipe_carry = sum[64];
            if (op == ALU_ADD || op == ALU_ADC)
               n.alu_pipe_overflow = (a[63] == b[63]) && (sum[63] != a[63]) ? 1'b1 : 1'b0;
            else
               n.alu_pipe_overflow = (a[63] != b[63]) && (sum[63] != a[63]) ? 1'b1 : 1'b0;
            n.alu_pipe_mode = 1'b0;   // ARITH
            n.wb.set_zero   = 1'b1;
            n.wb.rd         = rd;
            n.SM            = ALU_FINISH;
         end
      endcase
      return n;
   endfunction

   // Class-3 boolean compare / min-max selector (pipeline_core d.sub).
   typedef enum logic [3:0] {
      CMP_EQ, CMP_NE, CMP_LT, CMP_LE, CMP_GT, CMP_GE,
      CMP_ULT, CMP_ULE, CMP_UGT, CMP_UGE,
      CMP_MIN, CMP_MAX, CMP_MINU, CMP_MAXU
   } cmp_op_e;

   // ------------------------------------------------------------------------
   // ISA v2 field-decode helpers (pure).
   // ------------------------------------------------------------------------

   // f_cmp_op — class-3 boolean compare: map (PRED, INV) onto cmp_op_e.
   // PRED: 0=EQ 1=LT 2=LE 3=ULT 4=ULE; INV gives NE/GE/GT/UGE/UGT.
   function automatic cmp_op_e f_cmp_op(logic [2:0] pred, logic inv);
      case ({pred, inv})
         {3'd0, 1'b0}: f_cmp_op = CMP_EQ;
         {3'd0, 1'b1}: f_cmp_op = CMP_NE;
         {3'd1, 1'b0}: f_cmp_op = CMP_LT;
         {3'd1, 1'b1}: f_cmp_op = CMP_GE;
         {3'd2, 1'b0}: f_cmp_op = CMP_LE;
         {3'd2, 1'b1}: f_cmp_op = CMP_GT;
         {3'd3, 1'b0}: f_cmp_op = CMP_ULT;
         {3'd3, 1'b1}: f_cmp_op = CMP_UGE;
         {3'd4, 1'b0}: f_cmp_op = CMP_ULE;
         {3'd4, 1'b1}: f_cmp_op = CMP_UGT;
         default:      f_cmp_op = CMP_EQ;   // PRED 5-7 rejected by the caller
      endcase
   endfunction

   // f_cond_eval — class-8 branch condition: one flag mux + INV, over the unified
   // Z/S/C/V register. Returns {valid, taken}. COND: 0=always 1=Z 2=C 3=V 4=S
   // 5=LT 6=LE 7=ULT 8=ULE 9=E; 10-15 invalid (trap). INV on COND=0 reserved.
   // Relations are DERIVED (no E/L/U storage): EQ=Z, signed LT=S^V, LE=Z|(S^V);
   // unsigned uses the x86 borrow — ULT=C (a<b), ULE=C|Z; E aliases Z. These are
   // bit-identical to the retired equal/less/ult bits for any operand pair.
   function automatic logic [1:0] f_cond_eval(flags_t f, logic [3:0] cond, logic inv);
      logic t, v;
      v = 1'b1;
      case (cond)
         4'd0: t = 1'b1;                             // always
         4'd1: t = f.zero;                           // Z   (EQ after CMP)
         4'd2: t = f.carry;                          // C   (raw carry / borrow)
         4'd3: t = f.overflow;                       // V
         4'd4: t = f.sign;                           // S
         4'd5: t = f.sign ^ f.overflow;              // LT  signed   (S^V)
         4'd6: t = f.zero | (f.sign ^ f.overflow);   // LE  signed   (Z | S^V)
         4'd7: t = f.carry;                          // ULT unsigned (borrow: a<b)
         4'd8: t = f.carry | f.zero;                 // ULE unsigned (C | Z)
         4'd9: t = f.zero;                           // E == Z (alias of EQ)
         default: begin t = 1'b0; v = 1'b0; end
      endcase
      if (cond == 4'd0 && inv) v = 1'b0;
      return {v, inv ? ~t : t};
   endfunction

endpackage : klauss_pkg

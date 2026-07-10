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
   typedef enum logic [33:0] {
      OPCODE_REQUEST = 34'h1, OPCODE_FETCH = 34'h2, OPCODE_FETCH2 = 34'h4,
      VAR1_FETCH = 34'h8, VAR1_FETCH2 = 34'h10, WAITING = 34'h20,  // WAITING: interruptible core-suspend (WAIT opcode); uses the state bit above VAR1_FETCH2
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
   // Pipeline stage-latch bundles (Phase B — feat/pipeline). See PIPELINE_IMPL.md.
   // The whole-struct cpu_state_t above stays the reference model on master; the
   // pipeline decomposes it into these per-stage bundles, each with its own valid
   // bit and always_ff. Architectural state (r_register/flags/SP/int_mask/caches)
   // stays central and is committed only at WB (the single retire point). These
   // are first-cut widths and will firm up as each stage's logic lands (M1..M5).
   // ========================================================================
   typedef struct packed {
      logic        valid;
      logic [31:0] pc;
      logic [95:0] words;      // up to 3 x 32-bit instruction words (opcode + imm64)
      logic [1:0]  len;        // word count from LEN[31:30] (1/2/3)
      logic        fault;      // fetch-side fault (illegal / bus)
   } if_t;

   typedef struct packed {
      logic        valid;
      logic [31:0] pc;
      logic [31:0] opcode;
      logic [3:0]  cls;        // CLASS[29:26] (note: `class` is reserved)
      logic [3:0]  rd;
      logic [3:0]  rs1;
      logic [3:0]  rs2;
      logic [63:0] imm;        // sign/zero-extended immediate
      logic [1:0]  len;
      logic        is_branch;
      logic        is_mem;
      logic        is_store;
      logic        is_mul;
      logic        is_div;
      logic        writes_reg;
      logic        writes_flags;
   } id_t;

   typedef struct packed {
      logic        valid;
      logic [31:0] pc;
      logic [3:0]  rd;
      logic [63:0] result;
      flags_t      flags_out;
      logic        writes_reg;
      logic        writes_flags;
      logic        is_mem;
      logic        is_store;
      logic [31:0] mem_addr;
      logic [63:0] store_data;
      logic        taken;      // branch taken (resolved in EX)
      logic [31:0] target;     // redirect target
   } ex_t;

   typedef struct packed {
      logic        valid;
      logic [31:0] pc;
      logic [3:0]  rd;
      logic [63:0] value;
      flags_t      flags_out;
      logic        writes_reg;
      logic        writes_flags;
   } mem_t;

   // Pure combinational EX-stage ALU (mirrors f_alu's arithmetic; no FSM
   // sequencing). aluop = opcode[25:22] (0=ADD 1=SUB 2=ADC 3=SBC 4=AND 5=OR
   // 6=XOR); cin = carry-in for ADC/SBC. Used by the pipeline EX stage.
   typedef struct packed { flags_t flags; logic [63:0] result; } alu_res_t;

   function automatic alu_res_t f_alu_ex(logic [63:0] a, logic [63:0] b,
                                         logic [3:0] aluop, logic cin);
      alu_res_t    r;
      logic [64:0] sum;
      logic        is_sub;
      r      = '0;
      is_sub = (aluop == 4'd1) || (aluop == 4'd3);   // SUB / SBC
      case (aluop)
         4'd4: begin r.result = a & b; r.flags.zero = ((a & b) == 64'b0); end   // AND
         4'd5: begin r.result = a | b; r.flags.zero = ((a | b) == 64'b0); end   // OR
         4'd6: begin r.result = a ^ b; r.flags.zero = ((a ^ b) == 64'b0); end   // XOR
         default: begin                                                          // ADD/SUB/ADC/SBC
            if (is_sub) sum = {1'b0, a} - {1'b0, b} - {64'b0, (aluop == 4'd3) ? cin : 1'b0};
            else        sum = {1'b0, a} + {1'b0, b} + {64'b0, (aluop == 4'd2) ? cin : 1'b0};
            r.result         = sum[63:0];
            r.flags.carry    = sum[64];
            r.flags.zero     = (sum[63:0] == 64'b0);
            r.flags.sign     = sum[63];
            r.flags.overflow = is_sub ? ((a[63] != b[63]) && (sum[63] != a[63]))
                                      : ((a[63] == b[63]) && (sum[63] != a[63]));
         end
      endcase
      return r;
   endfunction

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

   // ========================================================================
   // f_cmpr — boolean compares (rd = (a REL b) ? 1 : 0) + min/max select.
   // All 14 ops are single-cycle: compute a value from (a,b), write wb.value,
   // OPCODE_REQUEST, PC+4. Same discipline as f_alu (seed n=s, reproduce the
   // rx_fifo_read default, read only args + snapshot s, blocking, return n).
   // Collapses t_cmpeqr..t_cmpuger (10) + t_min/max/minu/maxu_regs (4).
   // ========================================================================
   typedef enum logic [3:0] {
      CMP_EQ, CMP_NE, CMP_LT, CMP_LE, CMP_GT, CMP_GE,
      CMP_ULT, CMP_ULE, CMP_UGT, CMP_UGE,
      CMP_MIN, CMP_MAX, CMP_MINU, CMP_MAXU
   } cmp_op_e;

   function automatic cpu_state_t f_cmpr(cpu_state_t s, logic [63:0] a, logic [63:0] b, cmp_op_e op,
                                         logic [31:0] pc_next);
      cpu_state_t         n;
      logic signed [63:0] sa, sb;
      logic [63:0]        val;
      n = s;
      n.rx_fifo_read = 1'b0;
      sa = a; sb = b;
      case (op)
         CMP_EQ:   val = (a  == b ) ? 64'b1 : 64'b0;
         CMP_NE:   val = (a  != b ) ? 64'b1 : 64'b0;
         CMP_LT:   val = (sa <  sb) ? 64'b1 : 64'b0;
         CMP_LE:   val = (sa <= sb) ? 64'b1 : 64'b0;
         CMP_GT:   val = (sa >  sb) ? 64'b1 : 64'b0;
         CMP_GE:   val = (sa >= sb) ? 64'b1 : 64'b0;
         CMP_ULT:  val = (a  <  b ) ? 64'b1 : 64'b0;
         CMP_ULE:  val = (a  <= b ) ? 64'b1 : 64'b0;
         CMP_UGT:  val = (a  >  b ) ? 64'b1 : 64'b0;
         CMP_UGE:  val = (a  >= b ) ? 64'b1 : 64'b0;
         CMP_MIN:  val = (sa <  sb) ? a : b;
         CMP_MAX:  val = (sa >  sb) ? a : b;
         CMP_MINU: val = (a  <  b ) ? a : b;
         CMP_MAXU: val = (a  >  b ) ? a : b;
         default:  val = 64'b0;
      endcase
      n.wb.value   = val;
      n.wb.rd      = s.reg_dst;
      n.SM         = OPCODE_REQUEST;
      n.wb.pending = 1'b1;
      n.PC         = pc_next;
      return n;
   endfunction

   // ========================================================================
   // f_wb — generic single-cycle register writeback. The caller precomputes the
   // value (and the destination reg + next PC); `set_zero` gates the zero-flag
   // update — ops that don't set it leave st.wb.set_zero as-is (matching the
   // originals, which simply didn't assign it). Collapses COPY/SETR/LEAPC/SETFR/
   // NEGR/NOTR/SHLR1/SHRR1/SHLAR/SHRAR/SEXTW/ZEXTW.
   // ========================================================================
   function automatic cpu_state_t f_wb(cpu_state_t s, logic [63:0] value, logic [3:0] rd,
                                       logic set_zero, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.wb.value   = value;
      n.wb.rd      = rd;
      if (set_zero) n.wb.set_zero = 1'b1;
      n.SM         = OPCODE_REQUEST;
      n.wb.pending = 1'b1;
      n.PC         = pc_next;
      return n;
   endfunction

   // f_rot1 — rotate-by-1 / rotate-through-carry: like f_wb but also sets
   // flags.carry (+ set_zero). Caller precomputes the rotated value + carry_out.
   // Collapses ROLR1/RORR1/ROLCR/RORCR.
   function automatic cpu_state_t f_rot1(cpu_state_t s, logic [63:0] value, logic carry_out,
                                         logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.wb.value    = value;
      n.wb.rd       = rd;
      n.flags.carry = carry_out;
      n.wb.set_zero = 1'b1;
      n.SM          = OPCODE_REQUEST;
      n.wb.pending  = 1'b1;
      n.PC          = pc_next;
      return n;
   endfunction

   // f_btst — BTST (bit test): flag-only, no register writeback. zero flag = NOT
   // the tested bit (zero if the bit is 0).
   function automatic cpu_state_t f_btst(cpu_state_t s, logic [63:0] src, logic [5:0] pos,
                                         logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.flags.zero = ~src[pos];
      n.SM         = OPCODE_REQUEST;
      n.PC         = pc_next;
      return n;
   endfunction

   // ========================================================================
   // Tier 2 — setup / simple-state ops.
   // ========================================================================

   // f_delay — DELAYV/DELAYR spin-wait (multi-cycle: st.timing_start/
   // timeout_counter/timeout_max). fraction<<13 ticks; re-enters OPCODE_EXECUTE.
   function automatic cpu_state_t f_delay(cpu_state_t s, logic [31:0] fraction, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.timing_start == 1'b0) begin
         n.timeout_max     = fraction << 13;
         n.timeout_counter = 0;
         n.timing_start    = 1'b1;   // SM stays OPCODE_EXECUTE (n=s) -> re-enter
      end else if (s.timeout_counter >= s.timeout_max) begin
         n.timeout_counter = 0;
         n.timing_start    = 1'b0;
         n.SM              = OPCODE_REQUEST;
         n.PC              = pc_next;
      end else begin
         n.timeout_counter = s.timeout_counter + 1;
         n.SM              = OPCODE_EXECUTE;
      end
      return n;
   endfunction

   // f_sp_set — SETSP/ADDSP (single-cycle SP write). Caller computes new_sp.
   function automatic cpu_state_t f_sp_set(cpu_state_t s, logic [31:0] new_sp, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.SP = new_sp;
      n.SM = OPCODE_REQUEST;
      n.PC = pc_next;
      return n;
   endfunction

   // f_mul_setup — seed the multiply pipeline (MULR/MULUR/MULHR/MULHUR/MULV).
   // Single NBA into MULTIPLY_SETUP; the pipeline (FSM) does the rest incl. the
   // PC increment (via mul_is_immediate). No PC write here.
   function automatic cpu_state_t f_mul_setup(cpu_state_t s, logic [63:0] op_a, logic [63:0] op_b,
                                              logic [3:0] dest, logic is_high, logic is_unsigned,
                                              logic is_immediate);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read      = 1'b0;
      n.mul_operand_a     = op_a;
      n.mul_operand_b     = op_b;
      n.mul_dest_reg      = dest;
      n.mul_is_high       = is_high;
      n.mul_is_unsigned   = is_unsigned;
      n.mul_is_immediate  = is_immediate;
      n.SM                = MULTIPLY_SETUP;
      return n;
   endfunction

   // f_div_setup — seed the divide pipeline (DIVR/DIVUR/MODR/MODUR/DIVV/MODV),
   // CENTRALISING the divide-by-zero guard (was a per-op copy incl. the MODV bug).
   // by-zero: DIV->all-ones, MOD->dividend, overflow flag, single-cycle writeback.
   // else: abs operands (signed) into div_state, SM=DIVIDE_PREP; the iterative
   // divider (FSM) finishes + increments PC per div.pc_inc (= is_immediate). The
   // divisor==0 test covers both reg (rs2==0) and immediate (sext(imm)==0) forms.
   function automatic cpu_state_t f_div_setup(cpu_state_t s, logic [63:0] dividend, logic [63:0] divisor,
                                              logic is_signed, logic is_mod, logic is_immediate,
                                              logic [3:0] dest);
      cpu_state_t  n;
      logic [63:0] abs_dividend, abs_divisor;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (divisor == 64'b0) begin
         n.wb.value       = is_mod ? dividend : 64'hFFFFFFFFFFFFFFFF;
         n.wb.rd          = dest;
         n.flags.overflow = 1'b1;
         n.SM             = OPCODE_REQUEST;
         n.wb.pending     = 1'b1;
         n.PC             = is_immediate ? (s.PC + 32'd8) : (s.PC + 32'd4);
      end else begin
         abs_dividend = (is_signed && dividend[63]) ? (~dividend + 64'd1) : dividend;
         abs_divisor  = (is_signed && divisor[63])  ? (~divisor  + 64'd1) : divisor;
         n.div.dividend  = abs_dividend;
         n.div.divisor   = abs_divisor;
         n.div.quotient  = 64'b0;
         n.div.remainder = 64'b0;
         n.div.counter   = 7'd0;
         n.div.sign_q    = dividend[63] ^ divisor[63];
         n.div.sign_r    = dividend[63];
         n.div.is_signed = is_signed;
         n.div.op        = is_mod ? DIV_OP_MOD : DIV_OP_DIV;
         n.div.dest_reg  = dest;
         n.div.pc_inc    = is_immediate;
         n.SM            = DIVIDE_PREP;
      end
      return n;
   endfunction

   // ========================================================================
   // Tier 3 — multi-cycle DDR2 handshakes (extra_clock pattern). The mem bus
   // (st.mem_*) is written via the returned st; w_mem_ready / w_mem_read_data are
   // passed as READ-ONLY args. The FSM clears extra_clock on the next opcode
   // (unchanged). Standalone (not converted): POP/GETSP/PUSHV64, MEMGET32.
   // ========================================================================

   // f_push — DDR2 stack push (PUSHR/PUSHV/CALLR). Caller passes pushed data + final PC.
   function automatic cpu_state_t f_push(cpu_state_t s, logic [63:0] data, logic [31:0] pc_final,
                                         logic mem_ready);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.SP             = s.SP - 8;
         n.mem_addr       = s.SP - 32'd8;
         n.mem_write_data = data;
         n.mem_byte_en    = 8'hFF;
         n.mem_write_DV   = 1'b1;
         n.extra_clock    = 1'b1;
      end else if (mem_ready) begin
         n.mem_write_DV = 1'b0;
         n.SM           = OPCODE_REQUEST;
         n.PC           = pc_final;
      end
      return n;
   endfunction

   // f_mem_store — DDR2 store (MEMSET*/MEMSETR). Caller passes addr/data/byte_en/PC.
   // byte_en explicit -> fixes the latent stale-be the full-width RR/RV stores relied on.
   function automatic cpu_state_t f_mem_store(cpu_state_t s, logic mem_ready, logic [31:0] addr,
                                              logic [63:0] data, logic [7:0] be, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.mem_addr       = addr;
         n.mem_write_data = data;
         n.mem_byte_en    = be;
         n.mem_write_DV   = 1'b1;
         n.extra_clock    = 1'b1;
      end else if (mem_ready) begin
         n.mem_write_DV = 1'b0;
         n.SM           = OPCODE_REQUEST;
         n.PC           = pc_next;
      end
      return n;
   endfunction

   // f_mem_load — DDR2 load -> WRITEBACK (MEMREAD*/MEMGET8/16/64). Caller precomputes
   // the zero-extended value from w_mem_read_data (only consumed in the ready cycle).
   function automatic cpu_state_t f_mem_load(cpu_state_t s, logic mem_ready, logic [31:0] addr,
                                             logic [63:0] value_ext, logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.mem_addr    = addr;
         n.mem_read_DV = 1'b1;
         n.extra_clock = 1'b1;
      end else if (mem_ready) begin
         n.mem_read_DV = 1'b0;
         n.wb.value    = value_ext;
         n.wb.rd       = rd;
         n.SM          = WRITEBACK;
         n.PC          = pc_next;
      end
      return n;
   endfunction

   // ------------------------------------------------------------------------
   // Tier 3b/3c — indexed load/store (LDIDX*/STIDX*). effective_addr = rs2+imm
   // (or rs2+reg for the *R forms). f_ld_idx/f_st_idx take the RAW effective
   // address + raw data and do all alignment / lane-select / byte-enable
   // internally (mirrors t_load_indexed* / t_store_indexed* exactly). The *reg
   // forms are 3-stage: they redirect read-port B via reg_2 to fetch the offset
   // register, so portb is re-sampled each cycle from the returned st.
   // ------------------------------------------------------------------------
   typedef enum logic [1:0] { MSZ_8 = 2'd0, MSZ_16 = 2'd1, MSZ_32 = 2'd2, MSZ_64 = 2'd3 } mem_sz_e;

   // f_ld_idx — indexed load of 8/16/32/64 bits, zero- or sign-extended.
   // force_align only affects MSZ_64 (LDIDX64 raw vs LDIDX64A 8-byte aligned);
   // sub-word sizes always align to their natural boundary.
   function automatic cpu_state_t f_ld_idx(cpu_state_t s, logic mem_ready,
         logic [31:0] eaddr, logic [63:0] rdata, mem_sz_e sz, logic is_signed,
         logic force_align, logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t n;
      logic [63:0] val;
      logic [7:0]  b8;
      logic [15:0] h16;
      logic [31:0] w32;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         case (sz)
            MSZ_8:   n.mem_addr = eaddr;
            MSZ_16:  n.mem_addr = {eaddr[31:1], 1'b0};
            MSZ_32:  n.mem_addr = {eaddr[31:2], 2'b00};
            default: n.mem_addr = force_align ? {eaddr[31:3], 3'b000} : eaddr; // MSZ_64
         endcase
         n.mem_read_DV = 1'b1;
         n.extra_clock = 1'b1;
      end else if (mem_ready) begin
         n.mem_read_DV = 1'b0;
         b8  = rdata[(eaddr[2:0] * 8)  +: 8];
         h16 = rdata[(eaddr[2:1] * 16) +: 16];
         w32 = eaddr[2] ? rdata[63:32] : rdata[31:0];
         case (sz)
            MSZ_8:   val = is_signed ? {{56{b8[7]}},   b8}  : {56'b0, b8};
            MSZ_16:  val = is_signed ? {{48{h16[15]}}, h16} : {48'b0, h16};
            MSZ_32:  val = is_signed ? {{32{w32[31]}}, w32} : {32'b0, w32};
            default: val = rdata; // MSZ_64
         endcase
         n.wb.value = val;
         n.wb.rd    = rd;
         n.SM       = WRITEBACK;
         n.PC       = pc_next;
      end
      return n;
   endfunction

   // f_st_idx — indexed store of 8/16/32/64 bits. sdata = raw rs1 (replicated
   // internally). MSZ_64 writes byte_en=0xFF explicitly (Tier-3a convention:
   // the old STIDX64 task left byte_en stale — a full-doubleword store must
   // enable all 8 lanes). force_align only affects MSZ_64.
   function automatic cpu_state_t f_st_idx(cpu_state_t s, logic mem_ready,
         logic [31:0] eaddr, logic [63:0] sdata, mem_sz_e sz, logic force_align,
         logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         case (sz)
            MSZ_8: begin
               n.mem_addr       = eaddr;
               n.mem_write_data = {8{sdata[7:0]}};
               n.mem_byte_en    = 8'b0000_0001 << eaddr[2:0];
            end
            MSZ_16: begin
               n.mem_addr       = {eaddr[31:1], 1'b0};
               n.mem_write_data = {4{sdata[15:0]}};
               n.mem_byte_en    = 8'b0000_0011 << {eaddr[2:1], 1'b0};
            end
            MSZ_32: begin
               n.mem_addr       = {eaddr[31:2], 2'b00};
               n.mem_write_data = {sdata[31:0], sdata[31:0]};
               n.mem_byte_en    = eaddr[2] ? 8'b1111_0000 : 8'b0000_1111;
            end
            default: begin // MSZ_64
               n.mem_addr       = force_align ? {eaddr[31:3], 3'b000} : eaddr;
               n.mem_write_data = sdata;
               n.mem_byte_en    = 8'hFF;
            end
         endcase
         n.mem_write_DV = 1'b1;
         n.extra_clock  = 1'b1;
      end else if (mem_ready) begin
         n.mem_write_DV = 1'b0;
         n.SM           = OPCODE_REQUEST;
         n.PC           = pc_next;
      end
      return n;
   endfunction

   // (f_ld_idxreg / f_st_idxreg retired with ISA v2: the reg+reg indexed forms
   // carry the offset register in rs2, which is already on read port B, so
   // LDIDX64R/STIDX64R dispatch through f_ld_idx/f_st_idx with eaddr=rs1+rs2 —
   // no port-redirect dance, 1 word, standard 2-cycle DDR2 handshake.)

   // ------------------------------------------------------------------------
   // Tier 3d — single-cycle stragglers + non-branch control. Fills the gaps
   // Tier 1 missed (SEXTB/SEXTH set the sign flag; ABSR sets overflow — neither
   // fits the plain f_wb) plus the simple control ops (NOP/HALT/WAIT/RESET/TRAP)
   // and the DDR2-read handshakes RET/POP/IRET (mirror f_mem_load + SP bump).
   // ------------------------------------------------------------------------

   // f_wb_ns — writeback that also sets the SIGN flag from value[63] (SEXTB/SEXTH;
   // the sign-extended result's MSB equals the source sign bit). Zero via set_zero.
   function automatic cpu_state_t f_wb_ns(cpu_state_t s, logic [63:0] value, logic [3:0] rd,
                                          logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.wb.value   = value;
      n.wb.rd      = rd;
      n.wb.set_zero = 1'b1;
      n.flags.sign = value[63];
      n.SM         = OPCODE_REQUEST;
      n.wb.pending = 1'b1;
      n.PC         = pc_next;
      return n;
   endfunction

   // f_abs — ABSR: rd = |rs| (signed). Sets zero (set_zero) + overflow (only on
   // |INT64_MIN|, which can't be represented).
   function automatic cpu_state_t f_abs(cpu_state_t s, logic [63:0] rs, logic [3:0] rd,
                                        logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (rs[63]) begin
         n.wb.value       = ~rs + 64'd1;
         n.flags.overflow = (rs == 64'h8000000000000000) ? 1'b1 : 1'b0;
      end else begin
         n.wb.value       = rs;
         n.flags.overflow = 1'b0;
      end
      n.wb.rd      = rd;
      n.wb.set_zero = 1'b1;
      n.SM         = OPCODE_REQUEST;
      n.wb.pending = 1'b1;
      n.PC         = pc_next;
      return n;
   endfunction

   // f_bextr — BEXTR: rd = zero_ext((rs >> start) & ((1<<len)-1)). 32-bit locals
   // reproduce the exact width behaviour (result truncated to [31:0], zero-ext).
   function automatic cpu_state_t f_bextr(cpu_state_t s, logic [63:0] rs, logic [31:0] params,
                                          logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t n;
      logic [4:0]  start_pos;
      logic [4:0]  length;
      logic [31:0] mask;
      logic [31:0] result;
      n = s;
      n.rx_fifo_read = 1'b0;
      start_pos = params[4:0];
      length    = params[12:8];
      mask      = (32'hFFFFFFFF >> (32 - length));
      result    = (rs >> start_pos) & mask;
      n.wb.value   = result;          // zero-extended to 64
      n.wb.rd      = rd;
      n.wb.set_zero = 1'b1;
      n.SM         = OPCODE_REQUEST;
      n.wb.pending = 1'b1;
      n.PC         = pc_next;
      return n;
   endfunction

   // f_bdep — BDEP: deposit len bits of src at start into base. 32-bit low half
   // only; result zero-extended above bit 31 (matches t_deposit_bits widths).
   // v2 operand order: base = rs1 (port A), src = rs2 (port B).
   function automatic cpu_state_t f_bdep(cpu_state_t s, logic [63:0] base, logic [63:0] src,
                                         logic [31:0] params, logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t n;
      logic [4:0]  start_pos;
      logic [4:0]  length;
      logic [31:0] mask;
      logic [31:0] insert_val;
      n = s;
      n.rx_fifo_read = 1'b0;
      start_pos  = params[4:0];
      length     = params[12:8];
      mask       = (32'hFFFFFFFF >> (32 - length)) << start_pos;
      insert_val = (src << start_pos) & mask;
      n.wb.value   = (base & ~mask) | insert_val;
      n.wb.rd      = rd;
      n.SM         = OPCODE_REQUEST;
      n.wb.pending = 1'b1;
      n.PC         = pc_next;
      return n;
   endfunction

   // f_setr64 — SETR64: rd = {hi32, lo32}. 3-word instr; hi32 self-fetched from
   // PC+8 via a DDR2 read (only 2 words are pre-fetched). lane-select on PC[2].
   function automatic cpu_state_t f_setr64(cpu_state_t s, logic mem_ready, logic [63:0] rdata,
                                           logic [31:0] lo, logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.mem_addr    = s.PC + 8;
         n.mem_read_DV = 1'b1;
         n.extra_clock = 1'b1;
      end else if (mem_ready) begin
         n.mem_read_DV = 1'b0;
         n.wb.value    = {(s.PC[2] ? rdata[63:32] : rdata[31:0]), lo};
         n.wb.rd       = rd;
         n.SM          = OPCODE_REQUEST;
         n.wb.pending  = 1'b1;
         n.PC          = pc_next;
      end
      return n;
   endfunction

   // f_ctrl — set SM + PC only (NOP/WAIT/RESET/HALT). HALT passes PC unchanged.
   function automatic cpu_state_t f_ctrl(cpu_state_t s, e_sm_t new_sm, logic [31:0] new_pc);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.SM = new_sm;
      n.PC = new_pc;
      return n;
   endfunction

   // f_trap — TRAP: crash-dump via HCF_1 with ERR_TRAP.
   function automatic cpu_state_t f_trap(cpu_state_t s);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.error_code = ERR_TRAP;
      n.SM         = HCF_1;
      return n;
   endfunction

   // f_ret — RET: pop return addr from DDR2 stack -> PC; SP+=8. (Like f_mem_load
   // but targets PC + bumps SP instead of a register writeback.)
   function automatic cpu_state_t f_ret(cpu_state_t s, logic mem_ready, logic [63:0] rdata);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.mem_addr    = s.SP;
         n.mem_read_DV = 1'b1;
         n.extra_clock = 1'b1;
      end else if (mem_ready) begin
         n.PC          = rdata[31:0];
         n.SP          = s.SP + 8;
         n.mem_read_DV = 1'b0;
         n.SM          = OPCODE_REQUEST;
      end
      return n;
   endfunction

   // f_pop — POP: pop 64-bit from stack into rd (via WRITEBACK state); SP+=8.
   function automatic cpu_state_t f_pop(cpu_state_t s, logic mem_ready, logic [63:0] rdata,
                                        logic [3:0] rd, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.mem_addr    = s.SP;
         n.mem_read_DV = 1'b1;
         n.extra_clock = 1'b1;
      end else if (mem_ready) begin
         n.wb.value    = rdata;
         n.wb.rd       = rd;
         n.SP          = s.SP + 8;
         n.mem_read_DV = 1'b0;
         n.SM          = WRITEBACK;
         n.PC          = pc_next;
      end
      return n;
   endfunction

   // f_iret — IRET: pop the saved context slot -> PC + flags[38:32] + int_mask[42:39]; SP+=8.
   function automatic cpu_state_t f_iret(cpu_state_t s, logic mem_ready, logic [63:0] rdata);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.mem_addr    = s.SP;
         n.mem_read_DV = 1'b1;
         n.extra_clock = 1'b1;
      end else if (mem_ready) begin
         n.PC              = rdata[31:0];
         n.flags.zero      = rdata[38];  // [37]=E, [33]=L, [32]=U were derived on
         n.flags.carry     = rdata[36];  // save (E=Z, L=S^V, U=C); not restored —
         n.flags.overflow  = rdata[35];  // they regenerate from Z/S/C/V.
         n.flags.sign      = rdata[34];
         n.int_mask        = rdata[42:39];
         n.SP              = s.SP + 8;
         n.mem_read_DV     = 1'b0;
         n.SM              = OPCODE_REQUEST;
      end
      return n;
   endfunction

   // ------------------------------------------------------------------------
   // Tier 4 — branch family. f_cond_jump / f_cond_call handle both absolute and
   // PC-relative forms (is_rel). Taken: jump (calls first push return PC+8 via
   // the DDR2 extra_clock handshake). Not-taken: fall through (PC+8) and pre-settle
   // reg_1/2 for the successor (ps_* computed in the arm). The perf-branch strobe
   // (jumps only) and r_ir_presettled are driven by the ARM's own NBAs (they are
   // module-scope 1-cycle strobes, so they stay out of st).
   // ------------------------------------------------------------------------
   // f_cond_jump / f_cond_call — ISA v2 field-driven forms. `target` is the
   // resolved target value (imm32 or rs2 register for RIND); `pc_fall` is the
   // fall-through PC (w_pc_next — PC+4 for 1-word register forms, PC+8 for
   // 2-word immediate forms), which is also the pushed return address for
   // calls. The not-taken path pre-settles reg_1/2/dst for the successor.
   function automatic cpu_state_t f_cond_jump(cpu_state_t s, logic cond, logic [31:0] target,
         logic is_rel, logic [31:0] pc_fall,
         logic ps_ok, logic [3:0] ps_r1, logic [3:0] ps_r2, logic [3:0] ps_rd);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      n.SM = OPCODE_REQUEST;
      if (cond) begin
         n.PC = is_rel ? (s.PC + target) : target;
      end else begin
         n.PC = pc_fall;
         if (ps_ok) begin
            n.reg_1   = ps_r1;
            n.reg_2   = ps_r2;
            // reg_dst deliberately NOT pre-settled here — see f_ps note. Stores
            // are excluded from the fast path (port C settles in FETCH2), so the
            // successor's reg_dst comes from the fetch path, matching phase-1.
         end
      end
      return n;
   endfunction

   function automatic cpu_state_t f_cond_call(cpu_state_t s, logic mem_ready, logic cond,
         logic [31:0] target, logic is_rel, logic [31:0] pc_fall,
         logic ps_ok, logic [3:0] ps_r1, logic [3:0] ps_r2, logic [3:0] ps_rd);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (cond) begin
         if (s.extra_clock == 0) begin
            n.SP             = s.SP - 8;
            n.mem_addr       = s.SP - 32'd8;
            n.mem_write_data = {32'b0, pc_fall};   // return address = fall-through PC
            n.mem_byte_en    = 8'hFF;
            n.mem_write_DV   = 1'b1;
            n.extra_clock    = 1'b1;
         end else if (mem_ready) begin
            n.mem_write_DV = 1'b0;
            n.SM           = OPCODE_REQUEST;
            n.PC           = is_rel ? (s.PC + target) : target;
         end
      end else begin
         n.SM = OPCODE_REQUEST;
         n.PC = pc_fall;
         if (ps_ok) begin
            n.reg_1   = ps_r1;
            n.reg_2   = ps_r2;
            // reg_dst deliberately NOT pre-settled (see f_ps note).
         end
      end
      return n;
   endfunction

   // f_ps — apply the fall-through pre-settle to a COMPUTED next-state:
   // pre-load reg_1/2/dst for the sequential successor (its opcode's register
   // fields, read from the IFB at r_FPC via w_ps_r1/w_ps_r2/w_ps_rd) so the
   // fast-path dispatch can skip the FETCH2 bubble after a single-cycle op.
   // reg_dst is pre-settled too so read port C (v2 store data, rd field) is
   // valid in EXECUTE — this is what lets stores use the fast path. Pure
   // function composition — wrap a 1-cycle op's result:
   //   st <= f_ps(f_wb(...), w_ps_ok, w_ps_r1, w_ps_r2, w_ps_rd);
   // the arm arms r_ir_presettled when ps_ok. One whole-struct NBA, so no
   // partial-override clobber.
   function automatic cpu_state_t f_ps(cpu_state_t n, logic ps_ok,
                                       logic [3:0] ps_r1, logic [3:0] ps_r2, logic [3:0] ps_rd);
      if (ps_ok) begin
         n.reg_1   = ps_r1;
         n.reg_2   = ps_r2;
         // reg_dst is NOT pre-settled: port C (v2 store data) would then latch
         // reg[w_ps_rd] from the possibly-shifted IFB while a fast-pathed store
         // writes using reg_dst=r_ir_reg_dst, and the two can diverge. Instead,
         // stores stay off the fast path (see the r_ir_opcode[29:26]!=7 guard),
         // so reg_dst always comes from the settled fetch path — matching the
         // proven phase-1 pipeline. ps_rd is retained in the signature for a
         // future coordinated fix that pre-settles rd from the IR latch.
      end
      return n;
   endfunction

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

   // ------------------------------------------------------------------------
   // Tier 5 — the two bespoke multi-cycle opcodes (one function each; not shared
   // with anything, but the same next-state form as the rest — no old tasks left).
   // ------------------------------------------------------------------------

   // f_pushv64 — PUSHV64: push a 64-bit immediate {hi32,lo32}. 3-word instr, so
   // hi32 is self-fetched from PC+8 (read), then the doubleword is pushed (write).
   function automatic cpu_state_t f_pushv64(cpu_state_t s, logic mem_ready, logic [63:0] rdata,
                                            logic [31:0] lo);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (s.extra_clock == 0) begin
         n.mem_addr    = s.PC + 8;          // fetch hi32 from PC+8
         n.mem_read_DV = 1'b1;
         n.extra_clock = 1'b1;
      end else if (s.extra_clock == 1) begin
         if (mem_ready) begin
            n.mem_read_DV    = 1'b0;
            n.SP             = s.SP - 8;
            n.mem_addr       = s.SP - 32'd8;
            n.mem_write_data = {(s.PC[2] ? rdata[63:32] : rdata[31:0]), lo};
            n.mem_byte_en    = 8'hFF;
            n.mem_write_DV   = 1'b1;
            n.extra_clock    = 2'd2;
         end
      end else begin // extra_clock == 2
         if (mem_ready) begin
            n.mem_write_DV = 1'b0;
            n.SM           = OPCODE_REQUEST;
            n.PC           = s.PC + 12;
         end
      end
      return n;
   endfunction

   // f_memget32 — MEMGET32: unaligned 32-bit load. offset<=4 -> one dword;
   // offset 5-7 same cache line -> cache lookahead (rdata_next/next_valid);
   // else cross-line span -> second read (dw0 stashed in wb.value between reads).
   // `addr` = the base register value (v2: rs1 / port A); `rd` = destination.
   function automatic cpu_state_t f_memget32(cpu_state_t s, logic mem_ready, logic [63:0] rdata,
         logic [63:0] rdata_next, logic next_valid, logic [31:0] addr, logic [3:0] rd);
      cpu_state_t n;
      logic [2:0]  offset;
      logic [31:0] result;
      n = s;
      n.rx_fifo_read = 1'b0;
      offset = addr[2:0];
      result = 32'b0;
      if (s.extra_clock == 2'd0) begin
         n.mem_addr    = addr;
         n.mem_read_DV = 1'b1;
         n.extra_clock = 2'd1;
      end else if (s.extra_clock == 2'd1) begin
         if (mem_ready) begin
            n.mem_read_DV = 1'b0;
            if (offset <= 3'd4) begin
               case (offset)
                  3'd0:    result = rdata[31:0];
                  3'd1:    result = rdata[39:8];
                  3'd2:    result = rdata[47:16];
                  3'd3:    result = rdata[55:24];
                  3'd4:    result = rdata[63:32];
                  default: result = 32'b0;
               endcase
               n.wb.value = {32'b0, result};
               n.wb.rd    = rd;
               n.SM       = WRITEBACK;
               n.PC       = s.PC + 4;
            end else if (next_valid) begin
               // Span within same cache line — use cache lookahead.
               case (offset)
                  3'd5:    result = {rdata_next[7:0],  rdata[63:40]};
                  3'd6:    result = {rdata_next[15:0], rdata[63:48]};
                  3'd7:    result = {rdata_next[23:0], rdata[63:56]};
                  default: result = 32'b0;
               endcase
               n.wb.value = {32'b0, result};
               n.wb.rd    = rd;
               n.SM       = WRITEBACK;
               n.PC       = s.PC + 4;
            end else begin
               // Cross-cache-line span: stash dw0, issue second read at next dw.
               n.wb.value    = rdata;
               n.mem_addr    = {addr[31:3], 3'b000} + 32'd8;
               n.mem_read_DV = 1'b1;
               n.extra_clock = 2'd2;
            end
         end
      end else if (s.extra_clock == 2'd2) begin
         n.extra_clock = 2'd3;
      end else begin // extra_clock == 2'd3
         if (mem_ready) begin
            n.mem_read_DV = 1'b0;
            // dw1 = rdata; dw0 = s.wb.value (stashed in phase 1).
            case (offset)
               3'd5:    result = {rdata[7:0],  s.wb.value[63:40]};
               3'd6:    result = {rdata[15:0], s.wb.value[63:48]};
               3'd7:    result = {rdata[23:0], s.wb.value[63:56]};
               default: result = 32'b0;
            endcase
            n.wb.value = {32'b0, result};
            n.wb.rd    = rd;
            n.SM       = WRITEBACK;
            n.PC       = s.PC + 4;
         end
      end
      return n;
   endfunction

   // ------------------------------------------------------------------------
   // Tier 6b — LCD-SPI. LCD writes module output ports (o_TX_LCD_*/o_LCD_*),
   // which are NOT in st, so f_lcd handles only the st part and the arm drives
   // the ports via its own NBAs (the r_perf_br pattern). (UART-RX was also
   // converted here but is now RETIRED — UART is MMIO at 0xF001.)
   // ------------------------------------------------------------------------

   // f_lcd — st part of the SPI-DC LCD writes: on i_TX_LCD_Ready, clear the
   // timeout and advance; otherwise hold (stay in OPCODE_EXECUTE, retry). The
   // arm drives o_TX_LCD_Byte / o_LCD_DC / o_TX_LCD_DV alongside this.
   function automatic cpu_state_t f_lcd(cpu_state_t s, logic ready, logic [31:0] pc_next);
      cpu_state_t n;
      n = s;
      n.rx_fifo_read = 1'b0;
      if (ready) begin
         n.timeout_counter = 0;
         n.SM              = OPCODE_REQUEST;
         n.PC              = pc_next;
      end
      return n;
   endfunction

endpackage : klauss_pkg

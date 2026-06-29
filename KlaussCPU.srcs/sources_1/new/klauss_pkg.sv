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
      logic equal;      // E: last compare equal
      logic carry;      // C: carry / borrow out
      logic overflow;   // V: signed overflow
      logic sign;       // S: sign of last result (bit 63)
      logic less;       // L: signed less-than comparison
      logic ult;        // U: unsigned less-than comparison
   } flags_t;

   // --- Deferred-writeback bundle (r_wb) -----------------------------------
   // The P4.1 RAW-forwarding/writeback state that moves together through the
   // pipeline.  Field is `rd` (not `reg`, which is a keyword).
   typedef struct packed {
      logic [63:0] value;     // result to write back
      logic [3:0]  rd;        // destination register index
      logic        set_zero;  // set the zero flag from `value` at writeback
      logic        pending;   // a writeback is deferred/in-flight (RAW-forward source)
   } wb_t;

endpackage : klauss_pkg

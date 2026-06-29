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

   // --- Pure helper functions (were functions.vh) --------------------------
   // No module-scope coupling — args in, value out — so they lift cleanly into
   // the package. Bodies are unchanged; f_predecode_len's casez MUST stay in
   // lock-step with opcode_select.vh's dispatch.

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
   // 32-bit opcode alone, for the fetch pipeline. MUST be kept in lock-step with
   // opcode_select.vh (see that file). Independently enumerated.
   function automatic [3:0] f_predecode_len;
      input [31:0] opcode;
      begin
         // 3-register ALU: upper 16 bits non-zero (NNNN_0???) — always 1 word.
         if (opcode[31:16] != 16'h0000) begin
            f_predecode_len = 4'd4;
         end else begin
            casez (opcode[15:0])
               // ---- 3-word V64 (12 B) — the only two; MUST precede broader arms ----
               16'h0FE?: f_predecode_len = 4'd12;   // SETR64
               16'h4060: f_predecode_len = 4'd12;   // PUSHV64
               // ---- 2-word ops (8 B): RV / RRV / V ----
               16'h02??: f_predecode_len = 4'd8;    // ADDI  RRV
               16'h080?: f_predecode_len = 4'd8;    // SETR  RV
               16'h081?: f_predecode_len = 4'd8;    // ADDV  RV
               16'h082?: f_predecode_len = 4'd8;    // MINUSV RV
               16'h083?: f_predecode_len = 4'd8;    // CMPRV RV
               16'h086?: f_predecode_len = 4'd8;    // ANDV  RV
               16'h087?: f_predecode_len = 4'd8;    // ORV   RV
               16'h088?: f_predecode_len = 4'd8;    // XORV  RV
               16'h091?: f_predecode_len = 4'd8;    // SHLV  RV
               16'h092?: f_predecode_len = 4'd8;    // SHRV  RV
               16'h093?: f_predecode_len = 4'd8;    // SHRAV RV
               16'h099?: f_predecode_len = 4'd8;    // LEAPC RV
               16'h0A0?: f_predecode_len = 4'd8;    // BSET  RV
               16'h0A1?: f_predecode_len = 4'd8;    // BCLR  RV
               16'h0A2?: f_predecode_len = 4'd8;    // BTGL  RV
               16'h0A3?: f_predecode_len = 4'd8;    // BTST  RV
               16'h0AC?: f_predecode_len = 4'd8;    // BEXTR RV
               16'h0AD?: f_predecode_len = 4'd8;    // BDEP  RV
               16'h0B8?: f_predecode_len = 4'd8;    // MULV  RV
               16'h0B9?: f_predecode_len = 4'd8;    // DIVV  RV
               16'h0BA?: f_predecode_len = 4'd8;    // MODV  RV
               16'h0C??: f_predecode_len = 4'd8;    // LDIDX64  RRV
               16'h0D??: f_predecode_len = 4'd8;    // STIDX64  RRV
               16'h0E??: f_predecode_len = 4'd8;    // LDIDX64R RRV
               16'h0FC?: f_predecode_len = 4'd8;    // ROLV  RV
               16'h0FD?: f_predecode_len = 4'd8;    // RORV  RV
               16'hC0??: f_predecode_len = 4'd8;    // LDIDX32   RRV
               16'hC1??: f_predecode_len = 4'd8;    // STIDX32   RRV
               16'hC2??: f_predecode_len = 4'd8;    // LDIDX16   RRV
               16'hC3??: f_predecode_len = 4'd8;    // STIDX16   RRV
               16'hC4??: f_predecode_len = 4'd8;    // LDIDX8    RRV
               16'hC5??: f_predecode_len = 4'd8;    // STIDX8    RRV
               16'hC6??: f_predecode_len = 4'd8;    // LDIDX8_S  RRV
               16'hC7??: f_predecode_len = 4'd8;    // LDIDX16_S RRV
               // Flow control absolute V (1000-1011, 1013-101C) — 2 word.
               16'h1000,16'h1001,16'h1002,16'h1003,16'h1004,16'h1005,
               16'h1006,16'h1007,16'h1008,16'h1009,16'h100A,16'h100B,
               16'h100C,16'h100D,16'h100E,16'h100F,16'h1010,16'h1011:
                         f_predecode_len = 4'd8;
               16'h1013,16'h1014,16'h1015,16'h1016,16'h1017,16'h1018,
               16'h1019,16'h101A,16'h101B,16'h101C:
                         f_predecode_len = 4'd8;
               // PC-relative flow control 1030-1041 — all 2-word V.
               16'h1030,16'h1031,16'h1032,16'h1033,16'h1034,16'h1035,
               16'h1036,16'h1037,16'h1038,16'h1039,16'h103A,16'h103B,
               16'h103C,16'h103D,16'h103E,16'h103F,16'h1040,16'h1041:
                         f_predecode_len = 4'd8;
               // LCD / 7seg / LED / stack / UART / mem immediate (V) forms
               16'h2021: f_predecode_len = 4'd8;    // LCDCMDV  V
               16'h2022: f_predecode_len = 4'd8;    // LCDDATAV V
               16'h2023: f_predecode_len = 4'd8;    // LCD      V
               16'h3070: f_predecode_len = 4'd8;    // LEDV     V
               16'h3071: f_predecode_len = 4'd8;    // 7SEG1V   V
               16'h3072: f_predecode_len = 4'd8;    // 7SEG2V   V
               16'h3074: f_predecode_len = 4'd8;    // RGB1V    V
               16'h3075: f_predecode_len = 4'd8;    // RGB2V    V
               16'h4020: f_predecode_len = 4'd8;    // PUSHV    V
               16'h4050: f_predecode_len = 4'd8;    // ADDSP    V
               16'h5002: f_predecode_len = 4'd8;    // TXMEM    V
               16'h5003: f_predecode_len = 4'd8;    // TXSTRMEM V
               16'h720?: f_predecode_len = 4'd8;    // MEMSETR  RV
               16'h721?: f_predecode_len = 4'd8;    // MEMREADR RV
               16'h73??: f_predecode_len = 4'd8;    // STIDX64R RRV
               16'hFC??: f_predecode_len = 4'd8;    // LDIDX64A RRV
               16'hFD??: f_predecode_len = 4'd8;    // STIDX64A RRV
               // ---- everything else 1-word (4 B) ----
               default:  f_predecode_len = 4'd4;
            endcase
         end
      end
   endfunction

endpackage : klauss_pkg

// ASCII hex digit ('0'-'9','A'-'F') -> 4-bit nibble; any non-hex input -> 0x0
function [3:0] return_hex_from_ascii;
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
function [7:0] return_ascii_from_hex;
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

// f_predecode_len — encoded instruction length in BYTES (4/8/12) from the 32-bit
// opcode alone, for the fetch pipeline. Mirrors the casez in
// opcode_select.vh and the r_PC += {4,8,12} advance in each execute task:
//   4  = 1-word ops (incl. jumps/halts; redirect handled separately)
//   8  = ops that consume a PC+4 immediate (RV / RRV / V forms)
//   12 = the only two V64 ops (SETR64, PUSHV64)
// MUST be kept in lock-step with opcode_select.vh. Independently enumerated
// (NOT derived from f_perf_class, which lumps len-8 and len-4 into one class).
function [3:0] f_predecode_len;
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
            // 1012 (RET) and 102? (JMPR) are 1-word; handled by default=4.
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

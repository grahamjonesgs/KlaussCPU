`timescale 1ns/1ps
// Local unit test for f_predecode_len. ISA v2: length = LEN field (word0
// [31:30]: 01=4B, 10=8B, 11=12B), so this now locks the field extraction and
// the illegal-LEN=00 fallback rather than an enumerated table. Representative
// opcodes from ISA_ENCODING_V2_MAP.md, one or more per class per length.
// f_predecode_len lives in klauss_pkg (compile klauss_pkg.sv alongside this tb).
import klauss_pkg::*;
module tb_predecode;

  integer errors = 0;
  task check;
    input [31:0] op; input [3:0] exp;
    begin
      if (f_predecode_len(op) !== exp) begin
        $display("FAIL: opcode %h -> %0d, expected %0d", op, f_predecode_len(op), exp);
        errors = errors + 1;
      end
    end
  endtask

  initial begin
    // ---- LEN=11 (3 words, 12 B) ----
    check(32'hCBC0_0500, 12);  // SETR64 R5
    check(32'hE440_0000, 12);  // PUSHV64
    // ---- LEN=10 (2 words, 8 B) ----
    check(32'h8830_0310, 8);   // ADDI  (class 2)
    check(32'h8BD0_0200, 8);   // SETR
    check(32'h8C10_0010, 8);   // CMPRV (class 3)
    check(32'h9300_0450, 8);   // BEXTR (class 4)
    check(32'h9B20_0420, 8);   // LDIDX64 (class 6)
    check(32'h9F40_0300, 8);   // MEMSETR (class 7)
    check(32'hA048_0000, 8);   // JMPE (class 8)
    check(32'hA200_0000, 8);   // CALL
    check(32'hA440_0000, 8);   // PUSHV (class 9)
    check(32'hA880_0120, 8);   // MULV (class A)
    check(32'hAC05_0000, 8);   // DELAYV (class B)
    check(32'hB000_0000, 8);   // LCDCMDV (class C)
    // ---- LEN=01 (1 word, 4 B) ----
    check(32'h4420_0312, 4);   // ADDR (class 1)
    check(32'h4C00_0012, 4);   // CMPRR (class 3)
    check(32'h5020_8010, 4);   // SHLR1 (class 4, embedded N)
    check(32'h5020_C310, 4);   // SHLV #1 (class 4, F=1)
    check(32'h5400_0120, 4);   // COPY (class 5)
    check(32'h5B00_0210, 4);   // MEMREADRR (class 6)
    check(32'h5B60_0432, 4);   // LDIDX64R (mode 11 — was 2 words in v1)
    check(32'h5F60_0432, 4);   // STIDX64R (mode 11 — was 2 words in v1)
    check(32'h6080_0003, 4);   // JMPR (class 8, RIND)
    check(32'h6480_0200, 4);   // POP (class 9)
    check(32'h6580_0000, 4);   // RET
    check(32'h6880_0312, 4);   // MULR (class A)
    check(32'h6C00_0000, 4);   // NOP (class B)
    check(32'h7000_0010, 4);   // LCDCMDR (class C)
    // ---- LEN=00 (illegal: any v1 binary word / zeroed DDR2) -> 4, traps at dispatch ----
    check(32'h0000_0000, 4);
    check(32'h0001_0312, 4);   // v1 ADDR — must read as illegal-length 4
    check(32'h3FFF_FFFF, 4);

    if (errors == 0) $display("PREDECODE UNIT TEST: ALL CHECKS PASS");
    else $display("PREDECODE UNIT TEST: %0d FAILURES", errors);
    $finish;
  end
endmodule

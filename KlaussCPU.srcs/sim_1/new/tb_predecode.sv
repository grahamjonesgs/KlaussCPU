`timescale 1ns/1ps
// Local unit test for f_predecode_len (P4.1a). Catches transcription/casez-order
// errors fast, before the synth cycle. Ground truth for the ISA lengths is the
// board-empirical mismatch counter (separate); this locks the function behavior.
// f_predecode_len now lives in klauss_pkg (compile klauss_pkg.sv alongside this tb).
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
    // len-12 — the only two (most fragile arm; must beat broader patterns)
    check(32'h0000_0FE0, 12); check(32'h0000_0FEF, 12);   // SETR64
    check(32'h0000_4060, 12);                              // PUSHV64
    // 08xx high-risk split: len-8 RV (immediate) vs len-4 R
    check(32'h0000_0800, 8); check(32'h0000_0810, 8); check(32'h0000_0820, 8);
    check(32'h0000_0830, 8); check(32'h0000_0860, 8); check(32'h0000_0870, 8);
    check(32'h0000_0880, 8);
    check(32'h0000_0840, 4); check(32'h0000_0850, 4); check(32'h0000_0890, 4);
    check(32'h0000_08A0, 4); check(32'h0000_08F0, 4);
    // 09xx / 0Axx / 0Bxx len-8 RV
    check(32'h0000_0910, 8); check(32'h0000_0920, 8); check(32'h0000_0930, 8);
    check(32'h0000_0990, 8); check(32'h0000_0A00, 8); check(32'h0000_0A30, 8);
    check(32'h0000_0AC0, 8); check(32'h0000_0AD0, 8);
    check(32'h0000_0B80, 8); check(32'h0000_0B90, 8); check(32'h0000_0BA0, 8);
    // 09xx / 0Axx / 0Bxx / 0Fxx len-4 R (default)
    check(32'h0000_0900, 4); check(32'h0000_0940, 4); check(32'h0000_09F0, 4);
    check(32'h0000_0A40, 4); check(32'h0000_0AE0, 4); check(32'h0000_0B00, 4);
    check(32'h0000_0F00, 4); check(32'h0000_0F10, 4); check(32'h0000_0F80, 4);
    check(32'h0000_0FC0, 8); check(32'h0000_0FD0, 8);     // ROLV/RORV len-8
    // 3-reg ALU (upper16 != 0) -> 4
    check(32'h0001_0000, 4); check(32'h0007_0123, 4); check(32'h0050_0FFF, 4);
    // loads/stores len-8 (RRV / V indexed-absolute)
    check(32'h0000_0C00, 8); check(32'h0000_0D55, 8); check(32'h0000_0EFF, 8);
    check(32'h0000_C000, 8); check(32'h0000_C700, 8); check(32'h0000_C300, 8);
    check(32'h0000_FC00, 8); check(32'h0000_FD00, 8);
    check(32'h0000_7200, 8); check(32'h0000_7210, 8); check(32'h0000_7300, 8);
    // loads/stores len-4 (register-indirect forms)
    check(32'h0000_7000, 4); check(32'h0000_7100, 4); check(32'h0000_7500, 4);
    check(32'h0000_7900, 4); check(32'h0000_4000, 4); check(32'h0000_4010, 4);
    // flow control len-8 (absolute V + PC-rel V)
    check(32'h0000_1000, 8); check(32'h0000_1001, 8); check(32'h0000_1011, 8);
    check(32'h0000_101C, 8); check(32'h0000_1030, 8); check(32'h0000_1041, 8);
    // flow control len-4 (RET / JMPR / IRET / CALLR)
    check(32'h0000_1012, 4); check(32'h0000_1020, 4); check(32'h0000_102F, 4);
    check(32'h0000_6011, 4); check(32'h0000_4070, 4); check(32'h0000_407F, 4);
    // V-form io / stack len-8
    check(32'h0000_3070, 8); check(32'h0000_3072, 8); check(32'h0000_3075, 8);
    check(32'h0000_2021, 8); check(32'h0000_2023, 8); check(32'h0000_4020, 8);
    check(32'h0000_4050, 8); check(32'h0000_5002, 8); check(32'h0000_5003, 8);
    // single-word forms (default 4)
    check(32'h0000_0100, 4); check(32'h0000_0500, 4);     // COPY / CMPRR
    check(32'h0000_0200, 8);                              // ADDI RRV len-8
    check(32'h0000_3000, 4); check(32'h0000_3060, 4);     // LED/7seg reg forms
    check(32'h0000_F011, 4); check(32'h0000_F012, 4);     // HALT / WAIT
    check(32'h0000_0000, 4);                              // NOP

    if (errors == 0) $display("PREDECODE UNIT TEST: ALL %0d CHECKS PASS", 0);
    else $display("PREDECODE UNIT TEST: %0d FAILURES", errors);
    $finish;
  end
endmodule

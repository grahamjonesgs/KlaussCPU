//===========================================================================
// tb_flags — flag-unification correctness (Z/S/C/V + derived EQ/LT/LE/ULT/ULE).
//
// Drives CMP through the real f_alu(ALU_CMP) path, mirrors the ALU_FINISH CMP
// commit into a flags_t (Z/S/C/V), then checks f_cond_eval for every condition
// against an independent golden reference over corner cases + a random sweep.
// Pins the x86 borrow polarity (ULT = C). Part of the flag-model cleanup —
// runs on the golden-model side (no board), per the plan's sim-first discipline.
//===========================================================================
module tb_flags;
   import klauss_pkg::*;

   int errors = 0;
   int checks = 0;
   logic [63:0] ra, rb;

   // Check one condition vs a golden expected value, both non-inverted and inverted.
   task automatic check_cond(flags_t f, logic [3:0] cond, logic exp,
                             logic [63:0] a, logic [63:0] b, string nm);
      logic [1:0] r;
      r = f_cond_eval(f, cond, 1'b0);
      checks++;
      if (r[0] !== exp) begin
         errors++;
         $display("FAIL %-14s a=%h b=%h  got=%b exp=%b", nm, a, b, r[0], exp);
      end
      r = f_cond_eval(f, cond, 1'b1);
      checks++;
      if (r[0] !== ~exp) begin
         errors++;
         $display("FAIL %-10s(INV) a=%h b=%h  got=%b exp=%b", nm, a, b, r[0], ~exp);
      end
   endtask

   // Run a,b through CMP and check every derived condition.
   task automatic check_pair(logic [63:0] a, logic [63:0] b);
      cpu_state_t s0, s1;
      flags_t f;
      s0 = '0;
      s1 = f_alu(s0, a, b, ALU_CMP, 4'd0, 32'd0);   // real CMP flag producer
      f = '0;
      f.zero     = (s1.alu_pipe_value == 64'b0);     // mirror ALU_FINISH CMP commit
      f.carry    = s1.alu_pipe_carry;
      f.overflow = s1.alu_pipe_overflow;
      f.sign     = s1.alu_pipe_value[63];
      check_cond(f, 4'd1, (a == b),                   a, b, "EQ(Z)");
      check_cond(f, 4'd9, (a == b),                   a, b, "E");
      check_cond(f, 4'd5, ($signed(a) <  $signed(b)), a, b, "LT");
      check_cond(f, 4'd6, ($signed(a) <= $signed(b)), a, b, "LE");
      check_cond(f, 4'd7, (a <  b),                   a, b, "ULT");
      check_cond(f, 4'd8, (a <= b),                   a, b, "ULE");
      check_cond(f, 4'd4, ((a - b) >> 63) & 1'b1,     a, b, "S");
      check_cond(f, 4'd2, (a <  b),                   a, b, "C(=ULT borrow)");
   endtask

   initial begin
      // deterministic corner cases (overflow, signed/unsigned boundaries)
      check_pair(64'd0, 64'd0);
      check_pair(64'd5, 64'd5);
      check_pair(64'd1, 64'd2);
      check_pair(64'd2, 64'd1);
      check_pair(64'h8000000000000000, 64'h7FFFFFFFFFFFFFFF); // min vs max: sub overflows
      check_pair(64'h7FFFFFFFFFFFFFFF, 64'h8000000000000000);
      check_pair(64'hFFFFFFFFFFFFFFFF, 64'd0);                // -1 vs 0 / MAX vs 0
      check_pair(64'd0, 64'hFFFFFFFFFFFFFFFF);
      check_pair(64'h8000000000000000, 64'd0);                // min vs 0
      check_pair(64'hFFFFFFFFFFFFFFFF, 64'h8000000000000000);
      check_pair(64'h7FFFFFFFFFFFFFFF, 64'h7FFFFFFFFFFFFFFE);
      check_pair(64'h8000000000000001, 64'h8000000000000000);

      // pseudo-random sweep + near-equal neighbours
      for (int i = 0; i < 4000; i++) begin
         ra = {$random, $random};
         rb = {$random, $random};
         check_pair(ra, rb);
         check_pair(ra, ra);
         check_pair(ra, ra + 64'd1);
         check_pair(ra + 64'd1, ra);
      end

      $display("tb_flags: %0d checks, %0d errors", checks, errors);
      if (errors == 0) $display("TB_FLAGS: PASS");
      else             $display("TB_FLAGS: FAIL (%0d)", errors);
      $finish;
   end
endmodule

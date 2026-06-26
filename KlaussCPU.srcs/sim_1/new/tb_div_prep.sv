// Equivalence check for the DIVIDE_PREP clz-skip against reference division.
// Mirrors the RTL datapath expressions from KlaussCPU.v exactly:
//   w_div_shifted / w_div_trial / w_div_borrow, DIVIDE_PREP normalize,
//   DIVIDE_STEP restoring iteration.
`timescale 1ns/1ps
module tb_div_prep;

   logic  [63:0] r_div_dividend, r_div_divisor, r_div_quotient, r_div_remainder;
   logic  [6:0]  r_div_counter;

   // exact copies of the RTL wires
   wire [63:0] w_div_shifted = {r_div_remainder[62:0], r_div_dividend[63]};
   wire [64:0] w_div_trial   = {1'b0, w_div_shifted} - {1'b0, r_div_divisor};
   wire        w_div_borrow  = w_div_trial[64];

   // exact copy of count_leading_zeros from KlaussCPU.v
   function [6:0] count_leading_zeros;
      input [63:0] val;
      integer clz_i;
      logic [6:0] clz_result;
      begin
         clz_result = 7'd64;
         for (clz_i = 0; clz_i < 64; clz_i = clz_i + 1) begin
            if (val[clz_i])
               clz_result = 7'd63 - clz_i[6:0];
         end
         count_leading_zeros = clz_result;
      end
   endfunction

   integer errors = 0;
   integer cycles;

   task run_divide;            // models DIVIDE_PREP + DIVIDE_STEP cycle-by-cycle
      input [63:0] dividend;
      input [63:0] divisor;
      output [63:0] quotient;
      output [63:0] remainder;
      output integer iter_cycles;
      logic [6:0] prep_clz;
      begin
         r_div_dividend  = dividend;
         r_div_divisor   = divisor;
         r_div_quotient  = 64'b0;
         r_div_remainder = 64'b0;

         // ---- DIVIDE_PREP (verbatim logic) ----
         prep_clz = count_leading_zeros(r_div_dividend);
         if (prep_clz[6]) begin
            r_div_counter = 7'd64;
         end else begin
            r_div_dividend = r_div_dividend << prep_clz[5:0];
            r_div_counter  = {1'b0, prep_clz[5:0]};
         end

         // ---- DIVIDE_STEP iterations (verbatim logic) ----
         iter_cycles = 1;       // the PREP cycle
         while (r_div_counter < 7'd64) begin
            #1;                 // settle the wires
            if (!w_div_borrow) begin
               r_div_remainder = w_div_trial[63:0];
               r_div_quotient  = {r_div_quotient[62:0], 1'b1};
            end else begin
               r_div_remainder = w_div_shifted;
               r_div_quotient  = {r_div_quotient[62:0], 1'b0};
            end
            r_div_dividend = {r_div_dividend[62:0], 1'b0};
            r_div_counter  = r_div_counter + 1;
            iter_cycles    = iter_cycles + 1;
         end
         quotient  = r_div_quotient;
         remainder = r_div_remainder;
      end
   endtask

   task check;
      input [63:0] dividend;
      input [63:0] divisor;
      logic [63:0] q, r;
      integer c;
      begin
         if (divisor != 0) begin
            run_divide(dividend, divisor, q, r, c);
            if (q !== dividend / divisor || r !== dividend % divisor) begin
               errors = errors + 1;
               $display("FAIL %h / %h : got q=%h r=%h want q=%h r=%h",
                        dividend, divisor, q, r, dividend/divisor, dividend%divisor);
            end
         end
      end
   endtask

   integer i;
   logic [63:0] a, b, q, r;
   integer c, total_c;
   initial begin
      // edge cases
      check(64'd0, 64'd1);
      check(64'd0, 64'hFFFF_FFFF_FFFF_FFFF);
      check(64'd1, 64'd1);
      check(64'd1, 64'd2);                                  // divisor > dividend
      check(64'hFFFF_FFFF_FFFF_FFFF, 64'd1);                // full-width, 64 iters
      check(64'hFFFF_FFFF_FFFF_FFFF, 64'hFFFF_FFFF_FFFF_FFFF);
      check(64'h8000_0000_0000_0000, 64'd1);                // INT64_MIN magnitude
      check(64'h8000_0000_0000_0000, 64'h8000_0000_0000_0000);
      check(64'd100, 64'd7);
      check(64'd7, 64'd100);

      // random sweep across magnitude ranges
      for (i = 0; i < 20000; i = i + 1) begin
         a = {$random, $random};
         b = {$random, $random};
         case (i % 4)
            0: begin a = a & 64'hFF;        b = (b & 64'hF) | 1; end       // tiny
            1: begin a = a & 64'hFFFF_FFFF; b = (b & 64'hFFFF) | 1; end    // 32-bit-ish
            2: begin b = (b & 64'hFFFF_FFFF) | 1; end                      // full / 32
            3: begin b = b | 64'h1; end                                    // full / full
         endcase
         check(a, b);
      end

      // cycle-count spot checks
      run_divide(64'd9, 64'd2, q, r, c);          $display("9/2: %0d iter cycles (was 64)", c);
      run_divide(64'h89AB_CDEF, 64'd10, q, r, c); $display("32-bit/10: %0d iter cycles (was 64)", c);
      run_divide(64'hFFFF_FFFF_FFFF_FFFF, 64'd3, q, r, c); $display("full/3: %0d iter cycles (was 64)", c);

      if (errors == 0) $display("PASS: all divide equivalence checks passed");
      else             $display("FAILED: %0d errors", errors);
      $finish;
   end
endmodule

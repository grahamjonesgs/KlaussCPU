//===========================================================================
// tb_pipeline — M1 ALU + M2 loads/stores + M3 branches + M4 mul/div
// (interlock-only pipeline).
// M3 adds a countdown loop (backward conditional branch) that exercises branch
// resolution in EX, the flag interlock, and flush/redirect on taken.
// M4 adds mul (lo + signed-high), div/mod (unsigned + signed), both
// divide-by-zero paths (DIV -> all-ones, MOD -> dividend: the MODV-by-zero
// regression), a RAW dependent right behind a busy mul, and a Z-flag-from-div
// conditional branch (flag write + interlock through the iterative unit).
//===========================================================================
module tb_pipeline;
   logic clk = 0, rst = 1;
   logic [31:0] imem_addr, imem_data;
   logic [31:0] dmem_addr;  logic [63:0] dmem_wdata; logic dmem_we; logic [63:0] dmem_rdata;
   logic [63:0] dbg_r [0:15];
   logic [31:0] prog [0:63];
   logic [63:0] dmem [0:1023];
   integer i, errors = 0;

   pipeline_core dut (.clk(clk), .rst(rst), .imem_addr(imem_addr), .imem_data(imem_data),
                      .dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata), .dmem_we(dmem_we),
                      .dmem_rdata(dmem_rdata), .dbg_r(dbg_r));

   assign imem_data  = prog[imem_addr[31:2]];
   assign dmem_rdata = dmem[dmem_addr[31:3]];
   always_ff @(posedge clk) if (dmem_we) dmem[dmem_addr[31:3]] <= dmem_wdata;
   always #5 clk = ~clk;

   function [31:0] enc_alu(input [3:0] aluop, input f, input [3:0] rd, input [3:0] rs1, input [3:0] rs2);
      enc_alu = {2'b01, 4'b0001, aluop, f, 1'b0, 8'b0, rd, rs1, rs2};
   endfunction
   function [31:0] enc_ls(input [3:0] cls, input [3:0] rd, input [3:0] rs1, input [3:0] rs2);
      enc_ls = {2'b01, cls, 14'b0, rd, rs1, rs2};
   endfunction
   function [31:0] enc_br(input [3:0] cond, input inv, input signed [15:0] off);
      enc_br = {2'b01, 4'b1000, cond, inv, 5'b0, off};
   endfunction
   function [31:0] enc_mul(input is_high, input is_uns, input [3:0] rd, input [3:0] rs1, input [3:0] rs2);
      enc_mul = {2'b01, 4'b0010, 2'b00, is_high, is_uns, 10'b0, rd, rs1, rs2};
   endfunction
   function [31:0] enc_div(input is_mod, input is_sgn, input [3:0] rd, input [3:0] rs1, input [3:0] rs2);
      enc_div = {2'b01, 4'b0011, 2'b00, is_mod, is_sgn, 10'b0, rd, rs1, rs2};
   endfunction

   task automatic chk(input string nm, input [63:0] got, input [63:0] exp);
      if (got !== exp) begin errors++; $display("FAIL %-9s = %016h exp %016h", nm, got, exp); end
      else                   $display("ok   %-9s = %016h", nm, got);
   endtask

   initial begin
      for (i = 0; i < 64; i++)   prog[i] = 32'h0;
      for (i = 0; i < 1024; i++) dmem[i] = 64'h0;
      // M1/M2
      prog[0] = enc_alu(4'd0, 1'b1, 4'd4, 4'd1, 4'd2);  // r4 = r1+r2 = 8
      prog[1] = enc_alu(4'd1, 1'b1, 4'd5, 4'd1, 4'd2);  // r5 = r1-r2 = 2
      prog[2] = enc_alu(4'd0, 1'b1, 4'd6, 4'd4, 4'd5);  // r6 = r4+r5 = 10 (RAW)
      prog[3] = enc_ls (4'd7, 4'd4, 4'd9, 4'd0);        // store mem[64] = r4 = 8
      prog[4] = enc_ls (4'd6, 4'd10,4'd9, 4'd0);        // load  r10 = mem[64] = 8
      prog[5] = enc_alu(4'd0, 1'b1, 4'd11,4'd10,4'd1);  // r11 = r10+r1 = 13 (load-use)
      // M3 countdown loop at [6..9]: r12 down from 3, r14 counts iterations
      prog[6] = enc_alu(4'd0, 1'b1, 4'd14,4'd14,4'd13); // LOOP: r14 += r13   (accum)
      prog[7] = enc_alu(4'd1, 1'b1, 4'd12,4'd12,4'd13); //       r12 -= r13   (sets Z at 0)
      prog[8] = enc_br (4'd1, 1'b1, -16'sd2);           //       JMPNZ LOOP (cond=Z,inv -> not-zero)
      prog[9] = enc_alu(4'd0, 1'b1, 4'd15,4'd14,4'd0);  //       r15 = r14 (marker after loop)
      // M4 mul/div at [10..21] (r4/r5 reusable: their M1 values are checked
      // via r6 and mem[64]; r1/r2 clobbered only after their last use)
      prog[10] = enc_mul(1'b0, 1'b0, 4'd4, 4'd1, 4'd2);   // r4  = 5*3 = 15 (MUL lo)
      prog[11] = enc_alu(4'd0, 1'b1, 4'd5, 4'd4, 4'd13);  // r5  = r4+1 = 16 (RAW behind busy mul)
      prog[12] = enc_mul(1'b1, 1'b0, 4'd7, 4'd3, 4'd2);   // r7  = mulh_s(-5,3) = -1 (high of -15)
      prog[13] = enc_div(1'b0, 1'b0, 4'd8, 4'd9, 4'd2);   // r8  = 64/3 = 21 (DIVU)
      prog[14] = enc_div(1'b1, 1'b0, 4'd10,4'd9, 4'd2);   // r10 = 64%3 = 1  (MODU)
      prog[15] = enc_div(1'b0, 1'b1, 4'd4, 4'd3, 4'd2);   // r4  = -5/3 = -1 (DIV signed, trunc)
      prog[16] = enc_div(1'b1, 1'b1, 4'd3, 4'd3, 4'd2);   // r3  = -5%3 = -2 (MOD signed)
      prog[17] = enc_div(1'b0, 1'b0, 4'd1, 4'd9, 4'd0);   // r1  = 64/0 = all-ones (by-zero, V=1)
      prog[18] = enc_div(1'b1, 1'b0, 4'd2, 4'd9, 4'd0);   // r2  = 64%0 = 64 (MODV-by-zero -> dividend)
      prog[19] = enc_div(1'b0, 1'b0, 4'd13,4'd13,4'd2);   // r13 = 1/64 = 0 -> Z=1 (r2 now 64)
      prog[20] = enc_br (4'd1, 1'b0, 16'sd2);             // JMPZ +2 -> skip the poison op
      prog[21] = enc_alu(4'd0, 1'b1, 4'd5, 4'd5, 4'd9);   // POISON r5 += 64 (must be skipped)

      for (i = 0; i < 16; i++) dut.rf[i] = 64'h0;
      dut.rf[1]=64'd5; dut.rf[2]=64'd3; dut.rf[9]=64'd64;
      dut.rf[12]=64'd3; dut.rf[13]=64'd1;   // loop counter + step
      dut.rf[3]=64'hFFFFFFFFFFFFFFFB;       // -5 (signed mul/div operand)

      rst = 1; @(posedge clk); @(posedge clk); rst = 0;
      repeat (220) @(posedge clk);

      chk("r6",        dbg_r[6],  64'd10);
      chk("mem[64]",   dmem[8],   64'd8);
      chk("r11(luse)", dbg_r[11], 64'd13);
      chk("r12(cnt)",  dbg_r[12], 64'd0);   // loop ran to zero
      chk("r14(iter)", dbg_r[14], 64'd3);   // exactly 3 iterations
      chk("r15(post)", dbg_r[15], 64'd3);   // fell through after loop
      // M4
      chk("r5(raw)",   dbg_r[5],  64'd16);                  // mul lo via RAW; poison skipped
      chk("r7(mulh)",  dbg_r[7],  64'hFFFFFFFFFFFFFFFF);    // high word of -15
      chk("r8(divu)",  dbg_r[8],  64'd21);
      chk("r10(modu)", dbg_r[10], 64'd1);
      chk("r4(divs)",  dbg_r[4],  64'hFFFFFFFFFFFFFFFF);    // -5/3 = -1
      chk("r3(mods)",  dbg_r[3],  64'hFFFFFFFFFFFFFFFE);    // -5%3 = -2
      chk("r1(div0)",  dbg_r[1],  64'hFFFFFFFFFFFFFFFF);    // /0 -> all-ones
      chk("r2(mod0)",  dbg_r[2],  64'd64);                  // %0 -> dividend
      chk("r13(z)",    dbg_r[13], 64'd0);                   // 1/64 = 0 (set Z for the branch)
      if (errors == 0) $display("TB_PIPELINE M4: PASS");
      else             $display("TB_PIPELINE M4: FAIL (%0d)", errors);
      $finish;
   end
endmodule

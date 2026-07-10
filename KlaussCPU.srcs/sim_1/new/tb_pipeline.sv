//===========================================================================
// tb_pipeline — M1: verify the interlock-only ALU pipeline (pipeline_core).
// Runs a Class-1 ALU program with back-to-back RAW dependencies (which the M1
// interlock must stall on) and checks the register file against hand-computed
// results. No forwarding yet — correctness under interlocks is the M1 gate.
//===========================================================================
module tb_pipeline;
   logic clk = 0, rst = 1;
   logic [31:0] imem_addr, imem_data;
   logic [63:0] dbg_r [0:15];
   logic [31:0] prog [0:63];
   integer i, errors = 0;

   pipeline_core dut (.clk(clk), .rst(rst), .imem_addr(imem_addr),
                      .imem_data(imem_data), .dbg_r(dbg_r));

   assign imem_data = prog[imem_addr[31:2]];   // combinational instruction memory

   always #5 clk = ~clk;

   // ISA v2 Class-1 ALU encoding: {LEN=01, class=0001, aluop, F, 0, 8'b0, rd, rs1, rs2}
   function [31:0] enc_alu(input [3:0] aluop, input f,
                           input [3:0] rd, input [3:0] rs1, input [3:0] rs2);
      enc_alu = {2'b01, 4'b0001, aluop, f, 1'b0, 8'b0, rd, rs1, rs2};
   endfunction

   task automatic chk(input [3:0] r, input [63:0] exp);
      if (dbg_r[r] !== exp) begin errors++; $display("FAIL r%0d=%016h exp %016h", r, dbg_r[r], exp); end
      else                        $display("ok   r%0d=%016h", r, dbg_r[r]);
   endtask

   initial begin
      for (i = 0; i < 64; i++) prog[i] = 32'h0;   // rest = NOP/bubble
      prog[0] = enc_alu(4'd0, 1'b1, 4'd4, 4'd1, 4'd2);  // ADDR r4 = r1 + r2  = 8
      prog[1] = enc_alu(4'd1, 1'b1, 4'd5, 4'd1, 4'd2);  // SUBR r5 = r1 - r2  = 2
      prog[2] = enc_alu(4'd0, 1'b1, 4'd6, 4'd4, 4'd5);  // ADDR r6 = r4 + r5  = 10  (RAW r4,r5)
      prog[3] = enc_alu(4'd4, 1'b0, 4'd7, 4'd3, 4'd1);  // ANDR r7 = r3 & r1  = 4
      prog[4] = enc_alu(4'd0, 1'b1, 4'd8, 4'd6, 4'd7);  // ADDR r8 = r6 + r7  = 14  (RAW r6,r7)

      // seed the register file (M1 has no immediate-load ops yet)
      for (i = 0; i < 16; i++) dut.rf[i] = 64'h0;
      dut.rf[1] = 64'd5; dut.rf[2] = 64'd3; dut.rf[3] = 64'd100;

      rst = 1; @(posedge clk); @(posedge clk); rst = 0;
      repeat (40) @(posedge clk);

      chk(4'd4, 64'd8);
      chk(4'd5, 64'd2);
      chk(4'd6, 64'd10);
      chk(4'd7, 64'd4);
      chk(4'd8, 64'd14);
      if (errors == 0) $display("TB_PIPELINE M1: PASS");
      else             $display("TB_PIPELINE M1: FAIL (%0d)", errors);
      $finish;
   end
endmodule

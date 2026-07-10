//===========================================================================
// tb_pipeline — M1 (ALU + interlock) + M2 (loads/stores + load-use interlock).
// Verifies the interlock-only pipeline: ALU RAW deps, a store->load to the same
// address (memory ordering), and a load-use dependency (must stall until the
// load's WB). No forwarding yet.
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
   assign dmem_rdata = dmem[dmem_addr[31:3]];               // 64-bit word (byte addr / 8)
   always_ff @(posedge clk) if (dmem_we) dmem[dmem_addr[31:3]] <= dmem_wdata;

   always #5 clk = ~clk;

   function [31:0] enc_alu(input [3:0] aluop, input f, input [3:0] rd, input [3:0] rs1, input [3:0] rs2);
      enc_alu = {2'b01, 4'b0001, aluop, f, 1'b0, 8'b0, rd, rs1, rs2};
   endfunction
   function [31:0] enc_ls(input [3:0] cls, input [3:0] rd, input [3:0] rs1, input [3:0] rs2);
      enc_ls = {2'b01, cls, 14'b0, rd, rs1, rs2};
   endfunction

   task automatic chk(input string nm, input [63:0] got, input [63:0] exp);
      if (got !== exp) begin errors++; $display("FAIL %s = %016h exp %016h", nm, got, exp); end
      else                   $display("ok   %s = %016h", nm, got);
   endtask

   initial begin
      for (i = 0; i < 64; i++)   prog[i] = 32'h0;
      for (i = 0; i < 1024; i++) dmem[i] = 64'h0;
      // M1 ALU (RAW-dependent chain)
      prog[0] = enc_alu(4'd0, 1'b1, 4'd4, 4'd1, 4'd2);  // r4 = r1+r2 = 8
      prog[1] = enc_alu(4'd1, 1'b1, 4'd5, 4'd1, 4'd2);  // r5 = r1-r2 = 2
      prog[2] = enc_alu(4'd0, 1'b1, 4'd6, 4'd4, 4'd5);  // r6 = r4+r5 = 10 (RAW)
      // M2 store -> load (same address) -> use
      prog[3] = enc_ls (4'd7, 4'd4, 4'd9, 4'd0);        // store mem[r9+r0]=r4  -> mem[64]=8
      prog[4] = enc_ls (4'd6, 4'd10,4'd9, 4'd0);        // load  r10=mem[r9+r0] -> 8
      prog[5] = enc_alu(4'd0, 1'b1, 4'd11,4'd10,4'd1);  // r11 = r10+r1 = 13   (load-use, RAW on r10)

      for (i = 0; i < 16; i++) dut.rf[i] = 64'h0;
      dut.rf[1] = 64'd5; dut.rf[2] = 64'd3; dut.rf[9] = 64'd64;   // r9 = base byte addr 64

      rst = 1; @(posedge clk); @(posedge clk); rst = 0;
      repeat (50) @(posedge clk);

      chk("r4",       dbg_r[4],  64'd8);
      chk("r6",       dbg_r[6],  64'd10);
      chk("mem[64]",  dmem[8],   64'd8);   // store landed
      chk("r10(load)",dbg_r[10], 64'd8);   // load read it back
      chk("r11(use)", dbg_r[11], 64'd13);  // load-use stalled correctly
      if (errors == 0) $display("TB_PIPELINE M2: PASS");
      else             $display("TB_PIPELINE M2: FAIL (%0d)", errors);
      $finish;
   end
endmodule

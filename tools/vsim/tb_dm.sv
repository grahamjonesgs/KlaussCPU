//===========================================================================
// tb_dm — self-checking bench for debug_module's MMU stage 1 additions
// (run.py dmtest): registers 20 (SP_ALT) and 21 ({DI, UM}: UM in bit 0,
// DI in bit 1), DMCONTROL[5] EXC_PASS (read-back, dbg_i.exc_pass, cleared
// by a plain program load), and DMSTATUS version 2.
//===========================================================================
`timescale 1ns/1ps
module tb_dm;
   import klauss_pkg::*;
   logic clk = 0, rst = 1;
   always #5 clk = ~clk;
   logic        dmi_req = 0, dmi_rsp, prog_start = 0;
   logic [1:0]  dmi_op = 0;
   logic [7:0]  dmi_addr = 0;
   logic [63:0] dmi_wdata = 0, dmi_rdata;
   dbg_in_t     dbg_i;
   dbg_out_t    dbg_o = '0;
   logic [63:0] core_r [0:15];
   flags_t      core_flags = '0;
   int          fails = 0;
   initial for (int i = 0; i < 16; i++) core_r[i] = 64'(i);

   debug_module dm (.clk, .rst, .dmi_req, .dmi_op, .dmi_addr, .dmi_wdata, .dmi_rsp, .dmi_rdata,
                    .dbg_i, .dbg_o, .parked(1'b1), .park_kind(3'd4), .park_pc(32'h100),
                    .core_r, .core_sp(32'h0800_0000), .core_flags, .core_int_mask(4'h0),
                    .prog_start);

   task automatic dmi(input logic [1:0] op, input logic [7:0] addr, input logic [63:0] wd,
                      output logic [63:0] rd);
      @(posedge clk); dmi_req <= 1; dmi_op <= op; dmi_addr <= addr; dmi_wdata <= wd;
      @(posedge clk); dmi_req <= 0;
      @(posedge clk); rd = dmi_rdata;
   endtask
   task automatic chk(input string what, input logic [63:0] got, input logic [63:0] want);
      if (got !== want) begin fails++; $display("FAIL %s: got %h want %h", what, got, want); end
      else $display("pass %s", what);
   endtask

   initial begin
      logic [63:0] r;
      dbg_o.halted = 1'b1;
      repeat (3) @(posedge clk); rst <= 0;
      dbg_o.sp_alt = 32'h07F8_0000; dbg_o.um = 1'b1; dbg_o.dis = 1'b0;
      dmi(2, 8'h02, 64'd21, r); dmi(1, 8'h03, 0, r); chk("reg21 UM=1 DI=0", r, 64'h1);
      dbg_o.um = 1'b0; dbg_o.dis = 1'b1;
      dmi(2, 8'h02, 64'd21, r); dmi(1, 8'h03, 0, r); chk("reg21 UM=0 DI=1", r, 64'h2);
      dmi(2, 8'h02, 64'd20, r); dmi(1, 8'h03, 0, r); chk("reg20 SP_ALT", r, 64'h07F8_0000);
      dmi(1, 8'h01, 0, r);                       chk("DMSTATUS version", 64'(r[15:12]), 64'd2);
      dmi(2, 8'h00, 64'h28, r);                  // attached + exc_pass
      dmi(1, 8'h00, 0, r);                       chk("DMCONTROL[5] read-back", 64'(r[5]), 64'd1);
      chk("dbg_i.exc_pass", 64'(dbg_i.exc_pass), 64'd1);
      chk("dbg_i.attached", 64'(dbg_i.attached), 64'd1);
      @(posedge clk); prog_start <= 1; @(posedge clk); prog_start <= 0; @(posedge clk);
      chk("exc_pass cleared by a plain load", 64'(dbg_i.exc_pass), 64'd0);
      $display("%0d failing", fails);
      $finish;
   end
endmodule

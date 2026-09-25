//===========================================================================
// tb_trng — directed unit test for the TRNG's MMIO FIFO pop timing.
//
// The ring oscillators don't oscillate in simulation, so entropy is injected
// by forcing the accumulator-full pulse. The read side replicates the SoC's
// MMIO pipeline exactly (KlaussCPU.sv): the device's combinational read_data
// is sampled into r_mmio_read_data one cycle AFTER the strobe rises (the
// r_mmio_addr_q/r_mmio_read_data_comb stage), which is the value the CPU
// ultimately receives. The pop must therefore commit no earlier than the end
// of that second cycle — the bug this test pins down returned the freshly
// EMPTIED slot (0) on the first read and ran one entry behind afterwards.
//===========================================================================
`timescale 1ns / 1ps
module tb_trng;
   logic clk = 0, rst_l = 0;
   always #5 clk = ~clk;

   mmio_if bus();
   assign bus.byte_en = 8'hFF;

   trng dut (.i_Clk(clk), .i_Rst_L(rst_l), .mmio(bus.slave));

   int errors = 0;

   // SoC-accurate MMIO read: strobe high until the (2-cycle-delayed) ready
   // would fire; the value delivered to the CPU is the device's comb
   // read_data during the SECOND strobe cycle (sampled at its end).
   task automatic mmio_read(input logic [15:0] off, output logic [63:0] d);
      begin
         @(negedge clk);
         bus.addr    = {16'hF00C, off};
         bus.read_DV = 1'b1;
         @(negedge clk);          // cycle 1 (strobe rose at its start)
         @(posedge clk);          // end of cycle 2: SoC registers comb data
         d = bus.read_data;
         @(negedge clk);
         bus.read_DV = 1'b0;
         repeat (2) @(negedge clk);   // ready tail / re-arm gap
      end
   endtask

   // Inject one conditioned word: force the accumulator + full pulse for one
   // clock, exactly as the shift path would present them. (push_val is a
   // static copy — force cannot reference an automatic variable.)
   logic [63:0] push_val;
   task automatic push_word(input logic [63:0] v);
      begin
         push_val = v;
         @(negedge clk);
         force dut.r_accum            = push_val;
         force dut.r_accum_full_pulse = 1'b1;
         @(negedge clk);
         // A released register RETAINS the forced 1 until its process next
         // writes it — force it back to 0 across one more edge so exactly
         // one push fires, then release.
         force dut.r_accum_full_pulse = 1'b0;
         @(negedge clk);
         release dut.r_accum;
         release dut.r_accum_full_pulse;
      end
   endtask

   task automatic expect_data(input logic [63:0] want, input string what);
      logic [63:0] d;
      begin
         mmio_read(16'h0010, d);
         if (d !== want) begin
            errors++;
            $display("FAIL %s: got %016x want %016x", what, d, want);
         end else $display("PASS %s (%016x)", what, d);
      end
   endtask

   logic [63:0] d;
   initial begin
      bus.read_DV = 0; bus.write_DV = 0; bus.addr = '0; bus.write_data = '0;
      repeat (4) @(negedge clk);
      rst_l = 1;
      repeat (4) @(negedge clk);

      // Single-entry FIFO: the first read must return the pushed word, not
      // the empty slot (the original bug's signature was a first-read 0).
      push_word(64'hA11C_E5ED_0000_0001);
      repeat (2) @(negedge clk);
      expect_data(64'hA11C_E5ED_0000_0001, "single-push first read");

      // Two entries: reads must return them in order, no entry twice.
      push_word(64'hB0B0_0000_0000_0002);
      push_word(64'hC0C0_0000_0000_0003);
      repeat (2) @(negedge clk);
      expect_data(64'hB0B0_0000_0000_0002, "fifo order [0]");
      expect_data(64'hC0C0_0000_0000_0003, "fifo order [1]");

      // Refill after empty: no one-behind lag.
      push_word(64'hD0D0_0000_0000_0004);
      repeat (2) @(negedge clk);
      expect_data(64'hD0D0_0000_0000_0004, "refill read");

      // STATUS.READY must be 0 when drained, 1 when a word is queued.
      mmio_read(16'h0008, d);
      if (d[0] !== 1'b0) begin errors++; $display("FAIL ready-drained: %x", d); end
      else $display("PASS ready-drained");
      push_word(64'hEEEE_0000_0000_0005);
      repeat (2) @(negedge clk);
      mmio_read(16'h0008, d);
      if (d[0] !== 1'b1) begin errors++; $display("FAIL ready-queued: %x", d); end
      else $display("PASS ready-queued");

      if (errors == 0) $display("TB_TRNG PASS: pop timing matches the SoC MMIO read stage");
      else             $display("TB_TRNG FAILED: %0d errors", errors);
      $finish;
   end
endmodule

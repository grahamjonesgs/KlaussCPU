// Behavioural models of the Xilinx primitives liteeth_core.v instantiates,
// for tb_eth_slots under Verilator (xsim cannot run the generated core: it
// reports a zero-delay oscillation at time 0).  Only the modes LiteEth uses.
module IDDR #(parameter DDR_CLK_EDGE = "SAME_EDGE", parameter INIT_Q1 = 1'b0,
              parameter INIT_Q2 = 1'b0, parameter SRTYPE = "SYNC")
   (output logic Q1, output logic Q2, input C, input CE, input D, input R, input S);
   // SAME_EDGE: both outputs change on the rising edge (Q1 = D at that edge).
   logic fall = 1'b0;
   initial begin Q1 = INIT_Q1; Q2 = INIT_Q2; end
   always @(negedge C) if (CE) fall <= D;
   always @(posedge C) if (CE) begin Q1 <= D; Q2 <= fall; end
endmodule

module ODDR #(parameter DDR_CLK_EDGE = "OPPOSITE_EDGE", parameter INIT = 1'b0,
              parameter SRTYPE = "SYNC")
   (output logic Q, input C, input CE, input D1, input D2, input R, input S);
   // LiteEth drives D1 = D2, so Q is D registered on the rising edge.
   initial Q = INIT;
   always @(posedge C) if (CE) Q <= D1;
endmodule

module FDPE #(parameter INIT = 1'b1)
   (output logic Q, input C, input CE, input D, input PRE);
   initial Q = INIT;
   always @(posedge C or posedge PRE)
      if (PRE) Q <= 1'b1;
      else if (CE) Q <= D;
endmodule

module IOBUF (output O, inout IO, input I, input T);
   // The MDIO pin is not exercised: read back 1 (an idle, pulled-up bus).
   assign O = T ? 1'b1 : I;
endmodule

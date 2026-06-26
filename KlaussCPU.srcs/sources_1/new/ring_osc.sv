`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// ring_osc — single 5-stage inverter-chain ring oscillator.
//
// A combinational loop with an odd number of inverters self-oscillates at a
// frequency determined by routing/cell delay (typically a few hundred MHz on
// Artix-7 for a 5-stage chain).  We sample its output asynchronously with the
// system clock (in trng.v) to produce a metastable bit per sample.
//
// Synthesis attributes
// --------------------
//   - DONT_TOUCH on the chain wire prevents Vivado from collapsing
//     ~~chain[i] back into chain[i] and optimising the whole loop away.
//   - ALLOW_COMBINATORIAL_LOOPS on the wire suppresses the elaboration error
//     that fires by default for combinational rings.
//
// Simulation
// ----------
// `#1` propagation delay on the first stage gives the simulator something to
// schedule, so the ring oscillates at one toggle per simulation time-unit
// instead of trapping the kernel in a zero-delay loop.  Synthesis discards
// the delay (it's specify-block-free) so real hardware uses the routing
// delay, not 1 ns.
//
// One ring per TRNG entropy bit; trng.v instantiates 16 of them.
//////////////////////////////////////////////////////////////////////////////////

module ring_osc (
    output wire o_bit
);

    (* DONT_TOUCH = "true" *) (* ALLOW_COMBINATORIAL_LOOPS = "TRUE" *)
    wire [4:0] chain;

    // 5-stage inverter ring.  The `#1` on the first stage breaks the
    // zero-delay simulation loop without affecting synthesis.
    assign #1 chain[0] = ~chain[4];
    assign     chain[1] = ~chain[0];
    assign     chain[2] = ~chain[1];
    assign     chain[3] = ~chain[2];
    assign     chain[4] = ~chain[3];

    assign o_bit = chain[4];

endmodule

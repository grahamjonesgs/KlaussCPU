`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// bus_splitter — routes the CPU memory bus to one of three destinations:
//
//   addr[31:28] == 4'hF                              ─┬─ MMIO region
//     addr[27:16] in {12'h006, 12'h007, 12'h008}      │   ── Ethernet (LiteEth bridge)
//     else                                            │   ── existing peripherals
//   else                                              ── DRAM (cache + DDR2)
//
// MMIO accesses must NEVER reach the cache (would serve stale device state).
// The Ethernet block is split out from the rest of MMIO because LiteEth's
// Wishbone slave needs a dedicated bridge and consumes 192 KiB of address
// space (CSRs + slot SRAM with a gap) — see ETHERNET_PLAN.md.
//
// CPU-facing handshake is identical to the original mem_read_write interface,
// so the CPU FSM and memory_tasks.vh need no changes.
//
// All request paths are combinational — the CPU holds the request stable
// until ready, so the routing decision can be re-decoded each cycle from
// cpu.addr.  Responses: see COMB_DRAM_RET.
//////////////////////////////////////////////////////////////////////////////////

module bus_splitter #(
    // 1: the DRAM response (cache ready / data, both registered at the
    // cache) reaches the CPU combinationally, one cycle earlier on every
    // DRAM access; MMIO / Ethernet responses stay registered here.  0:
    // every response is registered (the original, FSM-era shape).
    parameter bit COMB_DRAM_RET = 1'b1
) (
    input             i_clk,           // clock for output pipeline (see below)

    // ----- CPU side (membus request/response from the CPU FSM) -----
    membus_if.slave   cpu,

    // ----- DRAM side (membus request/response to mem_read_write) -----
    membus_if.master  dram,

    // ----- MMIO side (existing peripherals: SD, RGB, 7seg, LEDs, cache, timers) -----
    output            o_mmio_write_DV,
    output            o_mmio_read_DV,
    output     [31:0] o_mmio_addr,
    output     [63:0] o_mmio_write_data,
    output     [ 7:0] o_mmio_byte_en,
    input      [63:0] i_mmio_read_data,
    input             i_mmio_ready,

    // ----- Ethernet side (eth_mmio_bridge → LiteEth) -----
    output            o_eth_write_DV,
    output            o_eth_read_DV,
    output     [31:0] o_eth_addr,
    output     [63:0] o_eth_write_data,
    output     [ 7:0] o_eth_byte_en,
    input      [63:0] i_eth_read_data,
    input             i_eth_ready,
    // the registered MMIO / Ethernet ready (r_ready below): cpu.ready for a
    // request in the MMIO region.  The top rebuilds the pipeline's ready
    // from it, dram.ready and the core's own registered address, off the
    // ownership mux on cpu.addr (timing; KlaussCPU.sv w_pip_ready).
    output            o_ready_q
);

    wire is_mmio = (cpu.addr[31:28] == 4'hF);
    wire is_eth  = is_mmio && (
                       (cpu.addr[27:16] == 12'h006) ||
                       (cpu.addr[27:16] == 12'h007) ||
                       (cpu.addr[27:16] == 12'h008)
                   );
    wire is_periph = is_mmio && !is_eth;

    // Strobes only fire on the selected destination.
    assign dram.write_DV = cpu.write_DV & ~is_mmio;
    assign dram.read_DV  = cpu.read_DV  & ~is_mmio;
    assign o_mmio_write_DV = cpu.write_DV &  is_periph;
    assign o_mmio_read_DV  = cpu.read_DV  &  is_periph;
    assign o_eth_write_DV  = cpu.write_DV &  is_eth;
    assign o_eth_read_DV   = cpu.read_DV  &  is_eth;

    // Address/data/be go to all sides — only the strobed side acts.
    assign dram.addr       = cpu.addr;
    assign dram.write_data = cpu.write_data;
    assign dram.byte_en    = cpu.byte_en;
    // DCBZ (HW_PERF_PLAN.md §1): the core faults a DCBZ to MMIO, so the
    // flag only ever accompanies a DRAM write
    assign dram.zero_line  = cpu.zero_line;
    assign o_mmio_addr       = cpu.addr;
    assign o_mmio_write_data = cpu.write_data;
    assign o_mmio_byte_en    = cpu.byte_en;
    assign o_eth_addr        = cpu.addr;
    assign o_eth_write_data  = cpu.write_data;
    assign o_eth_byte_en     = cpu.byte_en;

    // Return path: 3-way mux from currently-targeted side.  REGISTERED to
    // break a long combinational chain — the previous comb-only mux fed
    // a 13-LUT path through the CPU FSM dispatch logic and missed timing
    // (WNS ~ -0.5 ns) once the 3-way Eth split was added.  Adding one
    // pipeline stage here costs 1 cycle of read latency, which the CPU
    // already absorbs via the w_mem_ready handshake.
    wire [63:0] mem_read_data_comb = is_eth    ? i_eth_read_data
                                   : is_periph ? i_mmio_read_data
                                               : dram.read_data;
    wire        mem_ready_comb     = is_eth    ? i_eth_ready
                                   : is_periph ? i_mmio_ready
                                               : dram.ready;
    wire [63:0] mem_read_data_next_comb = dram.read_data_next;
    wire        mem_next_valid_comb     = is_mmio ? 1'b0 : dram.next_valid;

    // Registered response.  With COMB_DRAM_RET the register carries only
    // the MMIO / Ethernet side (never a stale DRAM ready across a change of
    // destination).
    logic [63:0] r_read_data = 64'b0, r_read_data_next = 64'b0;
    logic        r_next_valid = 1'b0, r_ready = 1'b0;
    always_ff @(posedge i_clk) begin
        r_read_data      <= mem_read_data_comb;
        r_read_data_next <= mem_read_data_next_comb;
        r_next_valid     <= mem_next_valid_comb;
        r_ready          <= COMB_DRAM_RET ? (is_mmio && mem_ready_comb) : mem_ready_comb;
    end
    // Response DATA select (timing): from the request address REGISTERED,
    // not live.  Data is only consumed together with a ready, and every
    // responder's ready is a flop set at least one cycle after its request
    // appears (the cache's cpu.ready from its FSM; MMIO ready is
    // r_mmio_*_dv_d2), with the address stable from request to response —
    // so r_is_mmio equals is_mmio whenever the data is used.  This takes
    // the cpu.addr -> decode -> select cone off the read-data paths into
    // the core (the RET target / RAS check -> pc, a forwarded load -> ex_a).
    // READY keeps the live select: the MMIO ready (a delayed copy of the
    // request strobe) stays high for cycles after an access, and a
    // registered select would show it for one cycle against the next,
    // non-MMIO request.
    logic r_is_mmio = 1'b0;
    always_ff @(posedge i_clk) r_is_mmio <= is_mmio;
    always_comb begin
        if (COMB_DRAM_RET && !r_is_mmio) begin
            cpu.read_data      = dram.read_data;
            cpu.read_data_next = dram.read_data_next;
            cpu.next_valid     = dram.next_valid;
        end else begin
            cpu.read_data      = r_read_data;
            cpu.read_data_next = r_read_data_next;
            cpu.next_valid     = r_next_valid;
        end
        cpu.ready = (COMB_DRAM_RET && !is_mmio) ? dram.ready : r_ready;
    end
    assign o_ready_q = r_ready;
endmodule

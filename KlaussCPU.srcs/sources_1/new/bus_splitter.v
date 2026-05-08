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
// All paths are combinational — the CPU holds the request stable until ready,
// so the routing decision can be re-decoded each cycle from i_mem_addr.
//////////////////////////////////////////////////////////////////////////////////

module bus_splitter (
    input             i_clk,           // clock for output pipeline (see below)

    // ----- CPU side (matches mem_read_write external interface) -----
    input             i_mem_write_DV,
    input             i_mem_read_DV,
    input      [31:0] i_mem_addr,
    input      [63:0] i_mem_write_data,
    input      [ 7:0] i_mem_byte_en,
    output reg [63:0] o_mem_read_data,
    output reg [63:0] o_mem_read_data_next,
    output reg        o_mem_next_valid,
    output reg        o_mem_ready,

    // ----- DRAM side (to mem_read_write) -----
    output            o_dram_write_DV,
    output            o_dram_read_DV,
    output     [31:0] o_dram_addr,
    output     [63:0] o_dram_write_data,
    output     [ 7:0] o_dram_byte_en,
    input      [63:0] i_dram_read_data,
    input      [63:0] i_dram_read_data_next,
    input             i_dram_next_valid,
    input             i_dram_ready,

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
    input             i_eth_ready
);

    wire is_mmio = (i_mem_addr[31:28] == 4'hF);
    wire is_eth  = is_mmio && (
                       (i_mem_addr[27:16] == 12'h006) ||
                       (i_mem_addr[27:16] == 12'h007) ||
                       (i_mem_addr[27:16] == 12'h008)
                   );
    wire is_periph = is_mmio && !is_eth;

    // Strobes only fire on the selected destination.
    assign o_dram_write_DV = i_mem_write_DV & ~is_mmio;
    assign o_dram_read_DV  = i_mem_read_DV  & ~is_mmio;
    assign o_mmio_write_DV = i_mem_write_DV &  is_periph;
    assign o_mmio_read_DV  = i_mem_read_DV  &  is_periph;
    assign o_eth_write_DV  = i_mem_write_DV &  is_eth;
    assign o_eth_read_DV   = i_mem_read_DV  &  is_eth;

    // Address/data/be go to all sides — only the strobed side acts.
    assign o_dram_addr       = i_mem_addr;
    assign o_dram_write_data = i_mem_write_data;
    assign o_dram_byte_en    = i_mem_byte_en;
    assign o_mmio_addr       = i_mem_addr;
    assign o_mmio_write_data = i_mem_write_data;
    assign o_mmio_byte_en    = i_mem_byte_en;
    assign o_eth_addr        = i_mem_addr;
    assign o_eth_write_data  = i_mem_write_data;
    assign o_eth_byte_en     = i_mem_byte_en;

    // Return path: 3-way mux from currently-targeted side.  REGISTERED to
    // break a long combinational chain — the previous comb-only mux fed
    // a 13-LUT path through the CPU FSM dispatch logic and missed timing
    // (WNS ~ -0.5 ns) once the 3-way Eth split was added.  Adding one
    // pipeline stage here costs 1 cycle of read latency, which the CPU
    // already absorbs via the w_mem_ready handshake.
    wire [63:0] mem_read_data_comb = is_eth    ? i_eth_read_data
                                   : is_periph ? i_mmio_read_data
                                               : i_dram_read_data;
    wire        mem_ready_comb     = is_eth    ? i_eth_ready
                                   : is_periph ? i_mmio_ready
                                               : i_dram_ready;
    wire [63:0] mem_read_data_next_comb = i_dram_read_data_next;
    wire        mem_next_valid_comb     = is_mmio ? 1'b0 : i_dram_next_valid;

    initial begin
        o_mem_read_data      = 64'b0;
        o_mem_read_data_next = 64'b0;
        o_mem_next_valid     = 1'b0;
        o_mem_ready          = 1'b0;
    end

    always @(posedge i_clk) begin
        o_mem_read_data      <= mem_read_data_comb;
        o_mem_read_data_next <= mem_read_data_next_comb;
        o_mem_next_valid     <= mem_next_valid_comb;
        o_mem_ready          <= mem_ready_comb;
    end

endmodule

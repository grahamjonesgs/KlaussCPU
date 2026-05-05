`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// bus_splitter — routes the CPU memory bus to either the DRAM cache controller
// or the MMIO peripheral bus, based on the top nibble of the address:
//
//   addr[31:28] == 4'hF  → MMIO   (0xF000_0000 – 0xFFFF_FFFF)
//   else                 → DRAM   (cache + DDR2)
//
// MMIO accesses must NEVER reach the cache (would serve stale device state).
// CPU-facing handshake is identical to the original mem_read_write interface,
// so the CPU FSM and memory_tasks.vh need no changes.
//
// All paths are combinational — the CPU holds the request stable until ready,
// so the routing decision can be re-decoded each cycle from i_mem_addr.
//////////////////////////////////////////////////////////////////////////////////

module bus_splitter (
    // ----- CPU side (matches mem_read_write external interface) -----
    input             i_mem_write_DV,
    input             i_mem_read_DV,
    input      [31:0] i_mem_addr,
    input      [63:0] i_mem_write_data,
    input      [ 7:0] i_mem_byte_en,
    output     [63:0] o_mem_read_data,
    output     [63:0] o_mem_read_data_next,
    output            o_mem_next_valid,
    output            o_mem_ready,

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

    // ----- MMIO side (to peripheral logic in CPU module) -----
    output            o_mmio_write_DV,
    output            o_mmio_read_DV,
    output     [31:0] o_mmio_addr,
    output     [63:0] o_mmio_write_data,
    output     [ 7:0] o_mmio_byte_en,
    input      [63:0] i_mmio_read_data,
    input             i_mmio_ready
);

    wire is_mmio = (i_mem_addr[31:28] == 4'hF);

    // Strobes only fire on the selected destination.
    assign o_dram_write_DV = i_mem_write_DV & ~is_mmio;
    assign o_dram_read_DV  = i_mem_read_DV  & ~is_mmio;
    assign o_mmio_write_DV = i_mem_write_DV &  is_mmio;
    assign o_mmio_read_DV  = i_mem_read_DV  &  is_mmio;

    // Address/data/be go to both sides — only the strobed side acts.
    assign o_dram_addr       = i_mem_addr;
    assign o_dram_write_data = i_mem_write_data;
    assign o_dram_byte_en    = i_mem_byte_en;
    assign o_mmio_addr       = i_mem_addr;
    assign o_mmio_write_data = i_mem_write_data;
    assign o_mmio_byte_en    = i_mem_byte_en;

    // Return path: mux from currently-targeted side.
    assign o_mem_read_data      = is_mmio ? i_mmio_read_data : i_dram_read_data;
    assign o_mem_ready          = is_mmio ? i_mmio_ready     : i_dram_ready;

    // Cache-prefetch outputs are DRAM-only; force 0 for MMIO so the CPU does
    // not pick up garbage as a "free" prefetched doubleword.
    assign o_mem_read_data_next = i_dram_read_data_next;
    assign o_mem_next_valid     = is_mmio ? 1'b0 : i_dram_next_valid;

endmodule

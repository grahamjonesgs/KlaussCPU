// ============================================================================
// membus_if — the CPU 64-bit memory bus as a SystemVerilog interface.
//
// The 9-signal bus that the CPU FSM drives, bus_splitter routes, and
// mem_read_write serves.  Same shape appears on THREE faces:
//   * KlaussCPU FSM  -> bus_splitter   (CPU request / response)
//   * bus_splitter   -> mem_read_write (DRAM request / response)
// so one interface collapses ~18 ports on bus_splitter and ~9 on
// mem_read_write into typed ports.
//
// Request:  write_DV, read_DV, addr, write_data, byte_en
// Response: read_data, read_data_next (next consecutive doubleword in the same
//           cache line), next_valid (read_data_next valid), ready.
//
// modports:
//   master — the requester (CPU FSM, or bus_splitter's DRAM face).
//   slave  — the responder (bus_splitter's CPU face, or mem_read_write).
// ============================================================================
interface membus_if;
   logic        write_DV;        // write strobe
   logic        read_DV;         // read strobe
   logic [31:0] addr;            // byte address (doubleword-aligned for 64-bit ops)
   logic [63:0] write_data;      // write data
   logic [ 7:0] byte_en;         // byte enables (8'hFF = full doubleword)
   logic [63:0] read_data;       // read data
   logic [63:0] read_data_next;  // next consecutive doubleword in the same cache line
   logic        next_valid;      // 1 when read_data_next is valid (offset == 0)
   logic        ready;           // transaction complete

   modport master (output write_DV, read_DV, addr, write_data, byte_en,
                   input  read_data, read_data_next, next_valid, ready);
   modport slave  (input  write_DV, read_DV, addr, write_data, byte_en,
                   output read_data, read_data_next, next_valid, ready);
endinterface : membus_if

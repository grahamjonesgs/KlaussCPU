// ============================================================================
// mmio_if — the 64-bit MMIO peripheral bus as a SystemVerilog interface.
//
// Replaces the 7 hand-replicated MMIO ports on each peripheral
// (crypto_aes, crypto_sha, sd_spi, blitter_dma, trng, eth_mmio_bridge) with a
// single typed port.  Topology (unchanged): bus_splitter broadcasts the request
// (addr/write_data/byte_en) to all peripherals; the per-peripheral write_DV /
// read_DV strobes are gated by address decode in KlaussCPU; each peripheral
// drives read_data/ready, which KlaussCPU muxes back to the splitter.
//
//   addr is 32-bit: the 16-bit-address peripherals use addr[15:0]; the Ethernet
//   bridge uses the full width.
//
// modports:
//   slave  — a peripheral: consumes the request, drives read_data/ready.
//   master — the CPU/decode side: drives the request, consumes read_data/ready.
// ============================================================================
interface mmio_if;
   logic        write_DV;     // write strobe (per-peripheral, address-decoded)
   logic        read_DV;      // read  strobe (per-peripheral, address-decoded)
   logic [31:0] addr;         // request address (broadcast; 16-bit slaves use [15:0])
   logic [63:0] write_data;   // write data (broadcast)
   logic [ 7:0] byte_en;      // byte enables (broadcast)
   logic [63:0] read_data;    // read data (driven by the addressed slave)
   logic        ready;        // transaction-complete (driven by the slave)

   modport slave  (input  write_DV, read_DV, addr, write_data, byte_en,
                   output read_data, ready);
   modport master (output write_DV, read_DV, addr, write_data, byte_en,
                   input  read_data, ready);
endinterface : mmio_if

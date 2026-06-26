`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// boot_rom — resident bootloader image (see NETBOOT_PLAN.md).
//
// An initialized block RAM holding the `netboot` program's DDR image, built
// exactly as the working UART/kbt version (linked at 0x20) — NOT relinked.  At
// power-on a small copy FSM in KlaussCPU.v reads this ROM doubleword-by-double-
// word and writes it into DDR starting at byte 0, then jumps to entry 0x20 via
// the existing LOAD_COMPLETE/run path.  So netboot runs from DDR and behaves
// identically to a UART load — the ROM just replaces the UART as the source.
//
// Image layout = build_ddr_image() output: a 4×64-bit heap header (word 0 lo32
// = heap_start = total image byte length) followed by netboot's flat program.
// The copy FSM reads word 0 to learn the length, so the ROM may be larger than
// the image (trailing doublewords are unused).
//
// INIT_FILE is a $readmemh image: one 64-bit doubleword per line, 16 hex chars,
// little-endian value of bytes [8*i .. 8*i+7] — produced by `klausscc --mem-out`.
//
// Read is synchronous (1-cycle latency): present i_dw_addr, o_dword valid next
// clock.  Read-only — netboot's runtime data lives in DDR after the copy.
//////////////////////////////////////////////////////////////////////////////////

module boot_rom #(
    parameter DEPTH_DW  = 32768,           // 64-bit words → 256 KiB capacity
    parameter ADDR_W    = 15,              // = $clog2(DEPTH_DW)
    parameter INIT_FILE = "netboot.mem"
) (
    input                   i_clk,
    input      [ADDR_W-1:0] i_dw_addr,      // doubleword index
    output logic [63:0]       o_dword
);
    (* ram_style = "block" *) reg [63:0] mem [0:DEPTH_DW-1];

    initial begin
        $readmemh(INIT_FILE, mem);
    end

    always_ff @(posedge i_clk) begin
        o_dword <= mem[i_dw_addr];
    end

endmodule

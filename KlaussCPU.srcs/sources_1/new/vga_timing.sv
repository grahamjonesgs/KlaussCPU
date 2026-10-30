`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// vga_timing — 640×480 @ 60 Hz VESA DMT raster counters (VGA_PLAN.md).
//
// Runs in the CPU clock domain (i_Clk = MIG ui_clk, 100 MHz). The 25 MHz pixel
// rate is a CLOCK ENABLE (o_pix_ce, one cycle in CE_DIV) rather than a second
// clock, so the display path needs no CDC and no new clock constraints.
// 25.000 MHz vs the nominal 25.175 MHz gives a 59.5 Hz frame — well inside what
// monitors and VGA→HDMI converters accept.
//
// Raster (pixels / lines, both syncs ACTIVE LOW for this mode):
//
//   h:  0 ........ 639 | 640 .. 655 | 656 .. 751 | 752 .. 799
//       active (640)     front (16)   sync (96)    back (48)     total 800
//   v:  0 ........ 479 | 480 .. 489 | 490 .. 491 | 492 .. 524
//       active (480)     front (10)   sync (2)     back (33)     total 525
//
// o_x/o_y hold the position of the pixel CURRENTLY being produced for the whole
// CE period; they advance at the edge where o_pix_ce is high. Everything below
// is combinational from those two registers, so a consumer that registers its
// outputs on o_pix_ce sees all of them with the same one-pixel delay (keep it
// that way — a per-signal delay mismatch shifts the picture against the syncs).
//
// Strobes (one i_Clk cycle, coincident with o_pix_ce):
//   o_line_start   — last CE of the pixel at h == 0 (every line, incl. blanking)
//   o_frame_start  — the same, on line 0
//   o_vblank_start — the same, on line V_ACTIVE (first blank line); the point
//                    where later phases latch FB_BASE and raise the vsync IRQ
//////////////////////////////////////////////////////////////////////////////////

module vga_timing #(
    parameter int H_ACTIVE = 640,
    parameter int H_FP     = 16,
    parameter int H_SYNC   = 96,
    parameter int H_BP     = 48,
    parameter int V_ACTIVE = 480,
    parameter int V_FP     = 10,
    parameter int V_SYNC   = 2,
    parameter int V_BP     = 33,
    parameter int CE_DIV   = 4      // i_Clk cycles per pixel (100 MHz / 4 = 25 MHz)
) (
    input              i_Clk,
    input              i_Rst_L,

    output logic       o_pix_ce,
    output logic [9:0] o_x,
    output logic [9:0] o_y,
    output logic       o_active,
    output logic       o_hsync_n,
    output logic       o_vsync_n,
    output logic       o_line_start,
    output logic       o_frame_start,
    output logic       o_vblank_start
);

    localparam int H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;   // 800
    localparam int V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;   // 525
    localparam int HS_ON   = H_ACTIVE + H_FP;                   // 656
    localparam int VS_ON   = V_ACTIVE + V_FP;                   // 490

    logic [$clog2(CE_DIV)-1:0] r_div = '0;
    logic [9:0]                r_h   = '0;
    logic [9:0]                r_v   = '0;

    assign o_pix_ce = (r_div == CE_DIV - 1);

    always_ff @(posedge i_Clk) begin
        if (!i_Rst_L) begin
            r_div <= '0;
            r_h   <= '0;
            r_v   <= '0;
        end else begin
            r_div <= o_pix_ce ? '0 : r_div + 1'b1;
            if (o_pix_ce) begin
                if (r_h == H_TOTAL - 1) begin
                    r_h <= '0;
                    r_v <= (r_v == V_TOTAL - 1) ? '0 : r_v + 1'b1;
                end else begin
                    r_h <= r_h + 1'b1;
                end
            end
        end
    end

    assign o_x            = r_h;
    assign o_y            = r_v;
    assign o_active       = (r_h < H_ACTIVE) && (r_v < V_ACTIVE);
    assign o_hsync_n      = !((r_h >= HS_ON) && (r_h < HS_ON + H_SYNC));
    assign o_vsync_n      = !((r_v >= VS_ON) && (r_v < VS_ON + V_SYNC));
    assign o_line_start   = o_pix_ce && (r_h == 0);
    assign o_frame_start  = o_line_start && (r_v == 0);
    assign o_vblank_start = o_line_start && (r_v == V_ACTIVE);

endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// vga_ctrl — VGA output for the Nexys A7 VGA connector (VGA_PLAN.md).
//
// Phase 0: free-running 640×480 @ 60 Hz TEST PATTERN, no CPU involvement. It
// exists to prove the pins, the 12-bit resistor DAC, and that the VGA→HDMI
// converter locks to the timing before any DDR scanout work. Later phases add
// the 0xF011 MMIO block and replace the pattern with the scanout line buffer;
// the timing generator and the output register stage stay as they are.
//
// Test pattern (active area, x 0..639, y 0..479):
//   border      1-pixel white frame on all four edges — proves the porches put
//               the image exactly inside the converter's capture window
//   y   0..359  8 colour bars, 80 px each: white yellow cyan green magenta red
//               blue black
//   y 360..479  four 30-line bands — R, G, B, grey ramps — 16 steps of 40 px,
//               level 0..15: proves every DAC bit and the pin order
//   square      32×32 orange square bouncing inside the bar area, moved once
//               per frame — proves the image is live, not a frozen frame
//
// Every output (RGB and both syncs) goes through ONE register stage loaded on
// the pixel CE, so they stay aligned with each other; the FFs are IOB-packed so
// the pin timing doesn't depend on placement.
//////////////////////////////////////////////////////////////////////////////////

module vga_ctrl (
    input              i_Clk,       // CPU clock (ui_clk, 100 MHz)
    input              i_Rst_L,

    output logic [3:0] o_vga_r,
    output logic [3:0] o_vga_g,
    output logic [3:0] o_vga_b,
    output logic       o_vga_hs,
    output logic       o_vga_vs
);

    localparam int SQ_SIZE  = 32;
    localparam int SQ_X_MIN = 8,  SQ_X_MAX = 600;   // top-left corner bounds:
    localparam int SQ_Y_MIN = 8,  SQ_Y_MAX = 300;   // stays inside the bar area

    // -------------------------------------------------------------------------
    // Raster timing
    // -------------------------------------------------------------------------
    logic       w_pix_ce, w_active, w_hsync_n, w_vsync_n;
    logic       w_line_start, w_frame_start, w_vblank_start;
    logic [9:0] w_x, w_y;

    vga_timing timing (
        .i_Clk          (i_Clk),
        .i_Rst_L        (i_Rst_L),
        .o_pix_ce       (w_pix_ce),
        .o_x            (w_x),
        .o_y            (w_y),
        .o_active       (w_active),
        .o_hsync_n      (w_hsync_n),
        .o_vsync_n      (w_vsync_n),
        .o_line_start   (w_line_start),
        .o_frame_start  (w_frame_start),
        .o_vblank_start (w_vblank_start)
    );

    // -------------------------------------------------------------------------
    // Bouncing square: moves 2 px right/left and 1 px down/up per frame, at the
    // start of vblank so it never changes mid-picture.
    // -------------------------------------------------------------------------
    logic [9:0] r_sq_x = 10'd100, r_sq_y = 10'd100;
    logic       r_sq_left = 1'b0, r_sq_up = 1'b0;

    always_ff @(posedge i_Clk) begin
        if (!i_Rst_L) begin
            r_sq_x    <= 10'd100;
            r_sq_y    <= 10'd100;
            r_sq_left <= 1'b0;
            r_sq_up   <= 1'b0;
        end else if (w_vblank_start) begin
            if (r_sq_left) begin
                r_sq_x <= r_sq_x - 10'd2;
                if (r_sq_x <= SQ_X_MIN + 2) r_sq_left <= 1'b0;
            end else begin
                r_sq_x <= r_sq_x + 10'd2;
                if (r_sq_x >= SQ_X_MAX - 2) r_sq_left <= 1'b1;
            end
            if (r_sq_up) begin
                r_sq_y <= r_sq_y - 10'd1;
                if (r_sq_y <= SQ_Y_MIN + 1) r_sq_up <= 1'b0;
            end else begin
                r_sq_y <= r_sq_y + 10'd1;
                if (r_sq_y >= SQ_Y_MAX - 1) r_sq_up <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Test pattern (combinational from the raster position)
    // -------------------------------------------------------------------------
    // Bar index = x / 80 by comparison (no divider).
    logic [2:0] w_bar;
    always_comb begin
        if      (w_x < 10'd80)  w_bar = 3'd0;
        else if (w_x < 10'd160) w_bar = 3'd1;
        else if (w_x < 10'd240) w_bar = 3'd2;
        else if (w_x < 10'd320) w_bar = 3'd3;
        else if (w_x < 10'd400) w_bar = 3'd4;
        else if (w_x < 10'd480) w_bar = 3'd5;
        else if (w_x < 10'd560) w_bar = 3'd6;
        else                    w_bar = 3'd7;
    end

    // Ramp level = x / 40 for x < 640, as (x * 205) >> 13 — exact over 0..639
    // (205/8192 is just above 1/40, and the error stays below one step).
    logic [17:0] w_ramp_prod;
    logic [3:0]  w_level;
    assign w_ramp_prod = w_x * 18'd205;
    assign w_level     = w_ramp_prod[16:13];

    wire w_border = (w_x == 10'd0) || (w_x == 10'd639) ||
                    (w_y == 10'd0) || (w_y == 10'd479);
    wire w_square = (w_x >= r_sq_x) && (w_x < r_sq_x + SQ_SIZE) &&
                    (w_y >= r_sq_y) && (w_y < r_sq_y + SQ_SIZE);

    logic [11:0] w_pattern;   // {R, G, B}
    always_comb begin
        if (!w_active)
            w_pattern = 12'h000;          // RGB must be black during blanking
        else if (w_border)
            w_pattern = 12'hFFF;
        else if (w_square)
            w_pattern = 12'hF80;
        else if (w_y < 10'd360)           // colour bars: W Y C G M R B K
            w_pattern = {{4{~w_bar[1]}}, {4{~w_bar[2]}}, {4{~w_bar[0]}}};
        else if (w_y < 10'd390)
            w_pattern = {w_level, 4'h0, 4'h0};
        else if (w_y < 10'd420)
            w_pattern = {4'h0, w_level, 4'h0};
        else if (w_y < 10'd450)
            w_pattern = {4'h0, 4'h0, w_level};
        else
            w_pattern = {w_level, w_level, w_level};
    end

    // -------------------------------------------------------------------------
    // Output register stage — one stage for every signal, IOB-packed.
    // Reset: black, syncs at their inactive (high) level.
    // -------------------------------------------------------------------------
    (* IOB = "TRUE" *) logic [3:0] r_vga_r = 4'h0, r_vga_g = 4'h0, r_vga_b = 4'h0;
    (* IOB = "TRUE" *) logic       r_vga_hs = 1'b1, r_vga_vs = 1'b1;

    always_ff @(posedge i_Clk) begin
        if (!i_Rst_L) begin
            r_vga_r  <= 4'h0;
            r_vga_g  <= 4'h0;
            r_vga_b  <= 4'h0;
            r_vga_hs <= 1'b1;
            r_vga_vs <= 1'b1;
        end else if (w_pix_ce) begin
            {r_vga_r, r_vga_g, r_vga_b} <= w_pattern;
            r_vga_hs <= w_hsync_n;
            r_vga_vs <= w_vsync_n;
        end
    end

    assign o_vga_r  = r_vga_r;
    assign o_vga_g  = r_vga_g;
    assign o_vga_b  = r_vga_b;
    assign o_vga_hs = r_vga_hs;
    assign o_vga_vs = r_vga_vs;

endmodule

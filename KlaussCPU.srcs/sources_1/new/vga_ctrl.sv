`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// vga_ctrl — VGA output for the Nexys A7 VGA connector (VGA_PLAN.md).
// MMIO device id 0x011, base 0xF011_0000.
//
// 640×480 @ 60 Hz from vga_timing (25 MHz pixel CE on the 100 MHz CPU clock).
// Register file, 256-entry palette, vsync IRQ, and DDR framebuffer scanout
// (vga_scanout.sv — DDR master D; RGB565 or 8-bit palette, optional 2x
// pixel/line doubling).
//
// Pixel source, highest priority first:
//   CTRL.TEST_PATTERN  colour bars / ramps / border / bouncing square (Phase 0)
//   CTRL.PALETTE_VIEW  16×16 grid of 40×30 cells, cell (row, col) shows
//                      palette[row*16 + col] — checks the palette path
//   CTRL.SCANOUT_EN    the DDR framebuffer, inside its image window
//   otherwise          VGA_BORDER colour (also outside the image window, and
//                      on a line whose fetch did not finish — an underflow)
// Reset state is TEST_PATTERN, so a fresh bitstream shows a picture with no
// software running.
//
// Register layout (offsets within the device window, 64-bit accesses):
//   0x000 VGA_CTRL      RW [0] SCANOUT_EN   [1] DOUBLE (320 → 640 px, 2x lines)
//                          [2] BPP8 (palette indices; else RGB565)
//                          [3] TEST_PATTERN (reset 1)
//                          [4] VSYNC_IRQ_EN   [5] PALETTE_VIEW
//   0x008 VGA_FB_BASE   RW framebuffer byte address [31:5] (32 B aligned).
//                          Pending value; copied to FB_ACTIVE at the start of
//                          vblank, so a write is a tear-free page flip.
//   0x010 VGA_STRIDE    RW bytes per source line [15:5] (32 B aligned; reset 640)
//   0x018 VGA_STATUS    R  [0] IN_VBLANK  [1] VSYNC_PENDING  [31:16] FRAME
//                          count  [41:32] current raster line  [63:48]
//                          UNDERFLOW count (starved display lines, saturating)
//                       W  [1]=1 clears VSYNC_PENDING, [2]=1 clears UNDERFLOW
//   0x020 VGA_BORDER    RW [11:0] colour {R,G,B} outside the source image
//   0x028 VGA_VSTART    RW [9:0] first display line of the image
//   0x030 VGA_FB_ACTIVE R  the FB_BASE latched at the last vblank
//   0x038 VGA_HEIGHT    RW [9:0] source lines (reset 240)
//   0x800..0xFF8 VGA_PALETTE RW 256 entries, 8 B apart, [11:0] = {R,G,B}
//
// Mode fields (DOUBLE, BPP8, STRIDE, HEIGHT, VSTART, FB_ACTIVE) take effect
// at the next frame (see vga_scanout.sv).  The CPU draws through the
// write-back cache, so software must flush the framebuffer before flipping.
//
// Vsync IRQ: VSYNC_PENDING sets at the start of vblank (line 480) every frame;
// o_irq = VSYNC_PENDING & VSYNC_IRQ_EN, a level the ISR must clear with a
// STATUS write of bit 1 before IRET (the blitter's DONE pattern). Vblank lasts
// 45 lines ≈ 1.4 ms — the window for flips and palette updates.
//
// Pixel pipeline (all registered on the pixel CE, so every signal — RGB, HS,
// VS — sees the same two-stage delay and stays aligned):
//   stage 1  source select from (x, y): direct colour, palette index, or the
//            scanout line-buffer dword (read the cycle after x changes)
//   stage 2  pixel extract + palette lookup (LUTRAM) / RGB565→444 + blanking
//            → IOB output FFs
//////////////////////////////////////////////////////////////////////////////////

module vga_ctrl (
    input              i_Clk,       // CPU clock (ui_clk, 100 MHz)
    input              i_Rst_L,

    mmio_if.slave      mmio,        // offsets use mmio.addr[15:0]
    output logic       o_irq,       // vsync, level (see header)

    // DDR master D (scanout) — to mem_read_write
    output logic        o_dma_req,
    output logic        o_dma_done,
    output logic        o_dma_read_DV,
    output logic [31:0] o_dma_addr,
    input       [255:0] i_dma_read_data,
    input               i_dma_ready,
    input               i_dma_grant,

    output logic [3:0] o_vga_r,
    output logic [3:0] o_vga_g,
    output logic [3:0] o_vga_b,
    output logic       o_vga_hs,
    output logic       o_vga_vs
);

    localparam logic [15:0] OFF_CTRL      = 16'h0000;
    localparam logic [15:0] OFF_FB_BASE   = 16'h0008;
    localparam logic [15:0] OFF_STRIDE    = 16'h0010;
    localparam logic [15:0] OFF_STATUS    = 16'h0018;
    localparam logic [15:0] OFF_BORDER    = 16'h0020;
    localparam logic [15:0] OFF_VSTART    = 16'h0028;
    localparam logic [15:0] OFF_FB_ACTIVE = 16'h0030;
    localparam logic [15:0] OFF_HEIGHT    = 16'h0038;

    localparam logic [5:0]  CTRL_RESET    = 6'b00_1000;   // TEST_PATTERN

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
    // Registers
    // -------------------------------------------------------------------------
    logic [5:0]  r_ctrl       = CTRL_RESET;
    logic [31:5] r_fb_base    = '0;
    logic [31:5] r_fb_active  = '0;
    logic [15:5] r_stride     = 11'd20;        // 640 B = 320 px of RGB565
    logic [11:0] r_border     = 12'h000;
    logic [9:0]  r_vstart     = 10'd0;
    logic [9:0]  r_height     = 10'd240;
    logic        r_vs_pending = 1'b0;
    logic [15:0] r_frame      = '0;
    logic [15:0] r_underflow  = '0;
    logic        w_underflow;                  // pulse from vga_scanout

    wire w_scan_en      = r_ctrl[0];
    wire w_double       = r_ctrl[1];
    wire w_bpp8         = r_ctrl[2];
    wire w_test_pattern = r_ctrl[3];
    wire w_irq_en       = r_ctrl[4];
    wire w_palette_view = r_ctrl[5];

    wire w_pal_sel = (mmio.addr[15:11] == 5'b00001);   // 0x0800..0x0FFF
    wire [7:0] w_pal_cpu_idx = mmio.addr[10:3];

    assign mmio.ready = 1'b1;
    wire byte_en_unused = |mmio.byte_en;

    always_ff @(posedge i_Clk) begin
        if (!i_Rst_L) begin
            r_ctrl       <= CTRL_RESET;
            r_fb_base    <= '0;
            r_fb_active  <= '0;
            r_stride     <= 11'd20;
            r_border     <= 12'h000;
            r_vstart     <= 10'd0;
            r_height     <= 10'd240;
            r_vs_pending <= 1'b0;
            r_frame      <= '0;
            r_underflow  <= '0;
        end else begin
            if (w_vblank_start) begin
                r_fb_active  <= r_fb_base;
                r_vs_pending <= 1'b1;
                r_frame      <= r_frame + 16'd1;
            end
            if (w_underflow && r_underflow != 16'hFFFF) r_underflow <= r_underflow + 16'd1;
            // A W1C in the same cycle as a new vblank loses to the set, so a
            // frame boundary can never be dropped.
            if (mmio.write_DV) begin
                case (mmio.addr[15:0])
                    OFF_CTRL:    r_ctrl    <= mmio.write_data[5:0];
                    OFF_FB_BASE: r_fb_base <= mmio.write_data[31:5];
                    OFF_STRIDE:  r_stride  <= mmio.write_data[15:5];
                    OFF_STATUS: begin
                        if (mmio.write_data[1] && !w_vblank_start) r_vs_pending <= 1'b0;
                        if (mmio.write_data[2])                    r_underflow  <= '0;
                    end
                    OFF_BORDER:  r_border  <= mmio.write_data[11:0];
                    OFF_VSTART:  r_vstart  <= mmio.write_data[9:0];
                    OFF_HEIGHT:  r_height  <= mmio.write_data[9:0];
                    default: ;
                endcase
            end
        end
    end

    assign o_irq = r_vs_pending & w_irq_en;

    // -------------------------------------------------------------------------
    // Palette: 256 × 12-bit LUTRAM. Port A = CPU write + readback, port B =
    // display read (stage 2). Not reset — software loads it.
    // -------------------------------------------------------------------------
    (* ram_style = "distributed" *) logic [11:0] r_palette [0:255];
    initial for (int i = 0; i < 256; i++) r_palette[i] = 12'h000;

    always_ff @(posedge i_Clk)
        if (mmio.write_DV && w_pal_sel) r_palette[w_pal_cpu_idx] <= mmio.write_data[11:0];

    // -------------------------------------------------------------------------
    // MMIO readback (combinational; the SoC registers it)
    // -------------------------------------------------------------------------
    wire w_in_vblank = (w_y >= 10'd480);

    always_comb begin
        mmio.read_data = 64'h0;
        if (w_pal_sel)
            mmio.read_data = {52'b0, r_palette[w_pal_cpu_idx]};
        else
            case (mmio.addr[15:0])
                OFF_CTRL:      mmio.read_data = {58'b0, r_ctrl};
                OFF_FB_BASE:   mmio.read_data = {32'b0, r_fb_base, 5'b0};
                OFF_STRIDE:    mmio.read_data = {48'b0, r_stride, 5'b0};
                OFF_STATUS:    mmio.read_data = {r_underflow, 6'b0, w_y, r_frame,
                                                 14'b0, r_vs_pending, w_in_vblank};
                OFF_BORDER:    mmio.read_data = {52'b0, r_border};
                OFF_VSTART:    mmio.read_data = {54'b0, r_vstart};
                OFF_FB_ACTIVE: mmio.read_data = {32'b0, r_fb_active, 5'b0};
                OFF_HEIGHT:    mmio.read_data = {54'b0, r_height};
                default:       mmio.read_data = 64'h0;
            endcase
    end

    // -------------------------------------------------------------------------
    // DDR scanout (line fetch + line buffer)
    // -------------------------------------------------------------------------
    logic        w_use_img, w_img_bpp8;
    logic [63:0] w_img_dword;
    logic [2:0]  w_img_sub;

    vga_scanout scanout (
        .i_Clk          (i_Clk),
        .i_Rst_L        (i_Rst_L),
        .i_en           (w_scan_en),
        .i_double       (w_double),
        .i_bpp8         (w_bpp8),
        .i_fb_base      ({r_fb_active, 5'b0}),
        .i_stride       ({r_stride, 5'b0}),
        .i_height       (r_height),
        .i_vstart       (r_vstart),
        .i_line_start   (w_line_start),
        .i_x            (w_x),
        .i_y            (w_y),
        .o_use_img      (w_use_img),
        .o_dword        (w_img_dword),
        .o_sub          (w_img_sub),
        .o_bpp8         (w_img_bpp8),
        .o_underflow    (w_underflow),
        .o_dma_req      (o_dma_req),
        .o_dma_done     (o_dma_done),
        .o_dma_read_DV  (o_dma_read_DV),
        .o_dma_addr     (o_dma_addr),
        .i_dma_read_data(i_dma_read_data),
        .i_dma_ready    (i_dma_ready),
        .i_dma_grant    (i_dma_grant)
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
    // Stage 1 (combinational from the raster position): test pattern, palette
    // grid index, or border.
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

    // x / 40 as (x * 205) >> 13 — exact over 0..639 (ramp step and grid
    // column). y / 30 as (y * 1093) >> 15 — exact over 0..479 (grid row).
    logic [17:0] w_xdiv_prod;
    logic [19:0] w_ydiv_prod;
    logic [3:0]  w_x40, w_y30;
    assign w_xdiv_prod = w_x * 18'd205;
    assign w_ydiv_prod = w_y * 20'd1093;
    assign w_x40       = w_xdiv_prod[16:13];
    assign w_y30       = w_ydiv_prod[18:15];

    wire w_border = (w_x == 10'd0) || (w_x == 10'd639) ||
                    (w_y == 10'd0) || (w_y == 10'd479);
    wire w_square = (w_x >= r_sq_x) && (w_x < r_sq_x + SQ_SIZE) &&
                    (w_y >= r_sq_y) && (w_y < r_sq_y + SQ_SIZE);

    logic [11:0] w_pattern;   // {R, G, B}
    always_comb begin
        if (w_border)
            w_pattern = 12'hFFF;
        else if (w_square)
            w_pattern = 12'hF80;
        else if (w_y < 10'd360)           // colour bars: W Y C G M R B K
            w_pattern = {{4{~w_bar[1]}}, {4{~w_bar[2]}}, {4{~w_bar[0]}}};
        else if (w_y < 10'd390)
            w_pattern = {w_x40, 4'h0, 4'h0};
        else if (w_y < 10'd420)
            w_pattern = {4'h0, w_x40, 4'h0};
        else if (w_y < 10'd450)
            w_pattern = {4'h0, 4'h0, w_x40};
        else
            w_pattern = {w_x40, w_x40, w_x40};
    end

    localparam logic [1:0] K_DIRECT = 2'd0, K_PALVIEW = 2'd1, K_IMG = 2'd2;

    logic [1:0]  r_s1_kind   = K_DIRECT;
    logic [11:0] r_s1_rgb    = '0;     // direct colour
    logic [7:0]  r_s1_idx    = '0;     // palette-view index
    logic [63:0] r_s1_dword  = '0;     // scanout line-buffer dword
    logic [2:0]  r_s1_sub    = '0;
    logic        r_s1_bpp8   = 1'b0;
    logic        r_s1_active = 1'b0;
    logic        r_s1_hs_n   = 1'b1, r_s1_vs_n = 1'b1;

    always_ff @(posedge i_Clk) begin
        if (!i_Rst_L) begin
            r_s1_active <= 1'b0;
            r_s1_hs_n   <= 1'b1;
            r_s1_vs_n   <= 1'b1;
            r_s1_kind   <= K_DIRECT;
        end else if (w_pix_ce) begin
            r_s1_active <= w_active;
            r_s1_hs_n   <= w_hsync_n;
            r_s1_vs_n   <= w_vsync_n;
            r_s1_kind   <= w_test_pattern ? K_DIRECT
                         : w_palette_view ? K_PALVIEW
                         : w_use_img      ? K_IMG
                         :                  K_DIRECT;
            r_s1_rgb    <= w_test_pattern ? w_pattern : r_border;
            r_s1_idx    <= {w_y30, w_x40};
            r_s1_dword  <= w_img_dword;
            r_s1_sub    <= w_img_sub;
            r_s1_bpp8   <= w_img_bpp8;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: pixel extract, palette lookup / RGB565→444, blanking → output
    // FFs (IOB-packed).  Pixels are little-endian in the dword: byte j at
    // [8j+7:8j], RGB565 halfword i at [16i+15:16i].
    // Reset: black, syncs at their inactive (high) level.
    // -------------------------------------------------------------------------
    wire [7:0]  w_img_byte = r_s1_dword[{r_s1_sub, 3'b000} +: 8];
    wire [15:0] w_img_565  = r_s1_dword[{r_s1_sub[2:1], 4'b0000} +: 16];
    wire [11:0] w_img_444  = {w_img_565[15:12], w_img_565[10:7], w_img_565[4:1]};
    wire [7:0]  w_pal_idx  = (r_s1_kind == K_PALVIEW) ? r_s1_idx : w_img_byte;
    wire [11:0] w_pal_rgb  = r_palette[w_pal_idx];

    logic [11:0] w_s2_rgb;
    always_comb begin
        if (!r_s1_active)                  w_s2_rgb = 12'h000;   // black in blanking
        else case (r_s1_kind)
            K_PALVIEW:                     w_s2_rgb = w_pal_rgb;
            K_IMG:                         w_s2_rgb = r_s1_bpp8 ? w_pal_rgb : w_img_444;
            default:                       w_s2_rgb = r_s1_rgb;
        endcase
    end

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
            {r_vga_r, r_vga_g, r_vga_b} <= w_s2_rgb;
            r_vga_hs <= r_s1_hs_n;
            r_vga_vs <= r_s1_vs_n;
        end
    end

    assign o_vga_r  = r_vga_r;
    assign o_vga_g  = r_vga_g;
    assign o_vga_b  = r_vga_b;
    assign o_vga_hs = r_vga_hs;
    assign o_vga_vs = r_vga_vs;

endmodule

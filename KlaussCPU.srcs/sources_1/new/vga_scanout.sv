`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// vga_scanout — DDR framebuffer scanout for vga_ctrl (VGA_PLAN.md Phase 2).
//
// DDR master D on mem_read_write's arbiter: read-only, WIDE reads (one 32 B
// line per transaction, returned in the cache-line layout — dword at byte
// offset 8k in bits [255-64k -: 64]).  Lines land in a ping-pong line buffer
// (one RAMB36, 512 x 64: half 0 = words 0..255, half 1 = 256..511, 2 KB each),
// and the display side reads pixels out of it.
//
// Source image: HEIGHT lines of 320 (DOUBLE) or 640 pixels, RGB565 or 8-bit
// palette indices (BPP8), STRIDE bytes apart from FB_BASE.  DOUBLE repeats
// every pixel and every line, so 320x240 fills 640x480.  The image occupies
// display lines VSTART .. VSTART + HEIGHT*(DOUBLE?2:1) - 1 (clipped at 480);
// lines outside it show the border colour.  All mode fields, SCANOUT_EN
// included, are latched at the start of vblank (line 480) — the same moment
// vga_ctrl copies FB_BASE to FB_ACTIVE — so anything software writes during a
// frame applies, consistently, from the next one, and nothing can tear the
// picture.  FB_BASE and STRIDE are used 32 B aligned.
//
// Fetch schedule (source line s lives in half s[0]):
//   * line (VSTART-1) mod 525        — prefetch source line 0 (frame start;
//                                      once per frame, after the line-480
//                                      latch)
//   * first display line of source s — fetch s+1 into the other half
// so each fetch has one source line of display time (1 display line native,
// 2 doubled) and never writes the half on screen.  A fetch is
// line_bytes/32 wide reads: 10 (320 x 8 bpp), 20 (320 x 16 / 640 x 8) or
// 40 (640 x 16), in tenures of CHUNK reads so a cache miss waits at most one
// chunk.  If a fetch is still running when the next one is due (it lost the
// race), it is abandoned after its in-flight read and the new line starts —
// the late line was already counted as an underflow.
//
// Display side: a pixel shows image data only if its line's half is VALID
// (fully fetched); otherwise o_use_img = 0 (vga_ctrl shows the border) and
// o_underflow pulses once at the start of that display line.
//////////////////////////////////////////////////////////////////////////////////

module vga_scanout #(
    parameter int CHUNK = 4              // wide reads per DDR tenure
) (
    input               i_Clk,
    input               i_Rst_L,

    // configuration (vga_ctrl registers)
    input               i_en,            // CTRL.SCANOUT_EN
    input               i_double,        // CTRL.DOUBLE
    input               i_bpp8,          // CTRL.BPP8
    input        [31:0] i_fb_base,       // FB_ACTIVE (vblank-latched FB_BASE)
    input        [15:0] i_stride,
    input        [9:0]  i_height,        // source lines
    input        [9:0]  i_vstart,

    // raster (vga_timing)
    input               i_line_start,
    input        [9:0]  i_x,
    input        [9:0]  i_y,

    // to the vga_ctrl pixel pipeline (valid for the current (x, y); the
    // dword is the registered BRAM read of that pixel's address)
    output logic        o_use_img,
    output logic [63:0] o_dword,
    output logic [2:0]  o_sub,           // byte (BPP8) / halfword ([2:1]) in the dword
    output logic        o_bpp8,
    output logic        o_underflow,     // 1-cycle pulse per starved display line

    // DDR master D (mem_read_write)
    output logic        o_dma_req,
    output logic        o_dma_done,
    output logic        o_dma_read_DV,
    output logic [31:0] o_dma_addr,
    input       [255:0] i_dma_read_data,
    input               i_dma_ready,
    input               i_dma_grant
);

    // -------------------------------------------------------------------------
    // Frame-latched configuration
    // -------------------------------------------------------------------------
    logic        r_armed  = 1'b0;        // scanning out this frame
    logic        r_ftrig_ok = 1'b0;      // frame trigger allowed (set at line 480)
    logic        r_f_en   = 1'b0;
    logic        r_f_dbl  = 1'b0, r_f_bpp8 = 1'b0;
    logic [9:0]  r_f_height = '0, r_f_vstart = '0;
    logic [31:5] r_f_stride = '0;
    logic [31:5] r_line_addr = '0;       // DDR address of the line being queued

    // Display-geometry helpers (from the frame-latched values)
    wire [9:0]  w_rel_y  = i_y - r_f_vstart;
    wire [10:0] w_img_h  = r_f_dbl ? {r_f_height, 1'b0} : {1'b0, r_f_height};
    wire        w_in_img_y = r_armed && (i_y < 10'd480) && (i_y >= r_f_vstart) &&
                             ({1'b0, w_rel_y} < w_img_h);
    wire [9:0]  w_src_line = r_f_dbl ? {1'b0, w_rel_y[9:1]} : w_rel_y;
    wire        w_first_disp = !r_f_dbl || !w_rel_y[0];   // first display line of its source line

    // Reads per line: (320 or 640 px) x (1 or 2 B) / 32 B
    wire [5:0]  w_line_reads = r_f_dbl ? (r_f_bpp8 ? 6'd10 : 6'd20)
                                       : (r_f_bpp8 ? 6'd20 : 6'd40);

    // -------------------------------------------------------------------------
    // Line buffer: 512 x 64 simple dual-port BRAM
    // -------------------------------------------------------------------------
    (* ram_style = "block" *) logic [63:0] r_lbuf [0:511];
    logic [8:0]  r_lb_waddr;
    logic [63:0] r_lb_wdata;
    logic        r_lb_we = 1'b0;
    logic [8:0]  w_lb_raddr;
    logic [63:0] r_lb_rdata;

    always_ff @(posedge i_Clk) begin
        if (r_lb_we) r_lbuf[r_lb_waddr] <= r_lb_wdata;
        r_lb_rdata <= r_lbuf[w_lb_raddr];
    end

    logic [1:0] r_valid = 2'b00;         // per-half: fully fetched

    // -------------------------------------------------------------------------
    // Display side (combinational from the raster position)
    // -------------------------------------------------------------------------
    wire [9:0]  w_px     = r_f_dbl ? {1'b0, i_x[9:1]} : i_x;
    wire [10:0] w_pbyte  = r_f_bpp8 ? {1'b0, w_px} : {w_px, 1'b0};
    wire        w_half   = w_src_line[0];
    assign w_lb_raddr    = {w_half, w_pbyte[10:3]};

    assign o_use_img = w_in_img_y && (i_x < 10'd640) && r_valid[w_half];
    assign o_dword   = r_lb_rdata;
    assign o_sub     = w_pbyte[2:0];
    assign o_bpp8    = r_f_bpp8;

    // -------------------------------------------------------------------------
    // Fetch triggers (at each display line start)
    // -------------------------------------------------------------------------
    wire [9:0] w_pre_line = (r_f_vstart == 10'd0) ? 10'd524 : r_f_vstart - 10'd1;
    wire       w_trig_frame = i_line_start && r_ftrig_ok && (i_y == w_pre_line);
    wire       w_trig_next  = i_line_start && !w_trig_frame && w_in_img_y && w_first_disp &&
                              ({1'b0, w_src_line} + 11'd1 < {1'b0, r_f_height});

    logic        r_pend = 1'b0;          // a line fetch is queued
    logic        r_pend_half;
    logic [31:5] r_pend_addr;

    always_ff @(posedge i_Clk) begin
        o_underflow <= 1'b0;
        if (!i_Rst_L) begin
            r_armed    <= 1'b0;
            r_ftrig_ok <= 1'b0;
        end else begin
            // Vblank start ends this frame's image (it is clipped at 480
            // anyway), latches the next frame's settings and arms its
            // trigger — so a frame whose VSTART moved down shows border above
            // its image, not a replay of the previous frame.
            if (i_line_start && i_y == 10'd480) begin
                r_ftrig_ok <= 1'b1;
                r_armed    <= 1'b0;
                r_f_en     <= i_en;
                r_f_dbl    <= i_double;
                r_f_bpp8   <= i_bpp8;
                r_f_height <= i_height;
                r_f_vstart <= i_vstart;
                r_f_stride <= {16'b0, i_stride[15:5]};
            end
            if (w_trig_frame) begin
                r_ftrig_ok  <= 1'b0;
                r_armed     <= r_f_en;
                r_line_addr <= i_fb_base[31:5];   // FB_ACTIVE, latched at 480 too
            end else if (w_trig_next) begin
                r_line_addr <= r_line_addr + r_f_stride;
            end
            // a starved line: in the image, but its half never completed
            if (i_line_start && w_in_img_y && !r_valid[w_half])
                o_underflow <= 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Fetch engine (DDR master D)
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] { F_IDLE, F_REQ, F_RD, F_GAP, F_REL, F_DONE, F_WAITREL } f_state_t;
    f_state_t    r_fs = F_IDLE;

    logic        r_half;                 // half being filled
    logic [31:5] r_addr;                 // next read address
    logic [5:0]  r_left;                 // reads left in this line
    logic [7:0]  r_widx;                 // next line-buffer word (within the half)
    logic [2:0]  r_chunk;                // reads done in this tenure
    logic [1:0]  r_gap;
    logic [255:0] r_wline;               // captured read, drained 1 dword/cycle
    logic [8:0]  r_wbase;
    logic [2:0]  r_wcnt = 3'd0;          // dwords left to write
    logic        r_wlast = 1'b0;         // this drain completes the line

    assign o_dma_addr = {r_addr, 5'b0};

    always_ff @(posedge i_Clk) begin
        r_lb_we    <= 1'b0;
        o_dma_done <= 1'b0;
        if (!i_Rst_L) begin
            r_fs          <= F_IDLE;
            o_dma_req     <= 1'b0;
            o_dma_read_DV <= 1'b0;
            r_pend        <= 1'b0;
            r_valid       <= 2'b00;
            r_wcnt        <= 3'd0;
            r_wlast       <= 1'b0;
        end else begin
            // ---- queue a line (a newer request replaces an unstarted one) ----
            if ((w_trig_frame && r_f_en) || w_trig_next) begin
                r_pend      <= 1'b1;
                r_pend_half <= w_trig_frame ? 1'b0 : ~w_half;
                r_pend_addr <= w_trig_frame ? i_fb_base[31:5] : r_line_addr + r_f_stride;
            end

            // ---- drain a captured read into the line buffer ----
            if (r_wcnt != 3'd0) begin
                r_lb_we    <= 1'b1;
                r_lb_waddr <= r_wbase;
                r_lb_wdata <= r_wline[255:192];
                r_wline    <= {r_wline[191:0], 64'b0};
                r_wbase    <= r_wbase + 9'd1;
                r_wcnt     <= r_wcnt - 3'd1;
                if (r_wcnt == 3'd1 && r_wlast) begin
                    r_valid[r_wbase[8]] <= 1'b1;
                    r_wlast <= 1'b0;
                end
            end

            case (r_fs)
                F_IDLE: begin
                    if (r_pend) begin
                        r_pend         <= 1'b0;
                        r_half         <= r_pend_half;
                        r_addr         <= r_pend_addr;
                        r_left         <= w_line_reads;
                        r_widx         <= 8'd0;
                        r_chunk        <= 3'd0;
                        r_valid[r_pend_half] <= 1'b0;
                        o_dma_req      <= 1'b1;
                        r_fs           <= F_REQ;
                    end
                end

                F_REQ: begin                     // wait for the bus
                    if (i_dma_grant) begin
                        o_dma_read_DV <= 1'b1;
                        r_fs          <= F_RD;
                    end
                end

                F_RD: begin                      // one wide read in flight
                    if (i_dma_ready) begin
                        o_dma_read_DV <= 1'b0;
                        r_wline       <= i_dma_read_data;
                        r_wbase       <= {r_half, r_widx};
                        r_wcnt        <= 3'd4;
                        r_wlast       <= (r_left == 6'd1) && !r_pend;
                        r_widx        <= r_widx + 8'd4;
                        r_addr        <= r_addr + 27'd1;
                        r_left        <= r_left - 6'd1;
                        r_chunk       <= r_chunk + 3'd1;
                        r_gap         <= 2'd1;
                        r_fs          <= F_GAP;
                    end
                end

                F_GAP: begin                     // DV low >= 1 cycle between reads
                    if (r_gap != 2'd0) begin
                        r_gap <= r_gap - 2'd1;
                    end else if (r_pend || r_left == 6'd0 || r_chunk == CHUNK) begin
                        r_gap <= 2'd1;
                        r_fs  <= F_REL;          // end of line / abandon / chunk
                    end else begin
                        o_dma_read_DV <= 1'b1;
                        r_fs          <= F_RD;
                    end
                end

                F_REL: begin                     // >= 2 cycles DV low, then done
                    if (r_gap != 2'd0) begin
                        r_gap <= r_gap - 2'd1;
                    end else begin
                        o_dma_done <= 1'b1;
                        // keep req up across a chunk release (re-queue);
                        // drop it with done when the line is finished
                        if (r_pend || r_left == 6'd0) o_dma_req <= 1'b0;
                        r_fs <= F_WAITREL;
                    end
                end

                F_WAITREL: begin                 // grant drops the cycle after done
                    if (!i_dma_grant) begin
                        r_chunk <= 3'd0;
                        r_fs    <= (r_pend || r_left == 6'd0) ? F_IDLE : F_REQ;
                    end
                end

                default: r_fs <= F_IDLE;
            endcase
        end
    end

endmodule

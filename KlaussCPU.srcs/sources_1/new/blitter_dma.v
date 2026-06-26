`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// blitter_dma — 2D DMA blitter (RGB565 framebuffer accelerator).
//
// Device base 0xF00E_0000 (MMIO device id 0x00E). See BLITTER_IMPLEMENTATION_PLAN.md
// and blitter-fpga-handoff.md. This module is BOTH:
//   * an MMIO slave (operand registers + START/STATUS), and
//   * a DDR bus master (master B on mem_read_write's arbiter).
//
// Pixels are RGB565, 16 bpp, little-endian. A 128-bit DDR word = 8 pixels.
// Rects may start at any pixel (any even byte) and src/dst strides may differ,
// so source and destination can have different alignment within their 128-bit
// words. The engine streams one pixel per step through:
//   * a 128-bit source read-buffer  (r_rbuf, COPY/COPY_BLEND),
//   * a 128-bit A8 mask read-buffer  (r_mbuf, MASK_BLEND), and
//   * a 128-bit destination word accumulator (r_wbuf).
//
// Two destination-word modes:
//   * Opaque (FILL, COPY): the accumulator is written with a per-byte write mask
//     so partial edge words touch only in-rect bytes — no read-modify-write.
//   * Blend (FILL_BLEND, COPY_BLEND, MASK_BLEND): the destination word is READ
//     into the accumulator first (it is the blend background), each in-rect pixel
//     is blended in place, and the whole word is written back (mask = 0).
//
// Blend math, per RGB565 channel (handoff spec, matches LVGL's >>8 form):
//     out = (fg*alpha + bg*(255-alpha) + 128) >> 8
//   foreground:  FILL_BLEND / MASK_BLEND -> COLOR,   COPY_BLEND -> src pixel
//   alpha:       FILL_BLEND / COPY_BLEND -> ALPHA reg, MASK_BLEND -> A8 mask[x,y]
//   NOTE: MASK_BLEND uses COLOR as the foreground (anti-aliased text/glyph case).
//
// Register layout (offsets within the device window; all 64-bit, 8-byte spaced
// to match the AES/SHA convention — software uses 64-bit MMIO accesses):
//   0x00 CTRL    W  [0] START (self-clearing), [3:1] OP, [4] IRQ_EN
//   0x08 STATUS  R  [0] BUSY, [1] DONE (write-1-to-clear)
//   0x10 DST_ADDR    RW dst top-left byte address
//   0x18 DST_STRIDE  RW dst row bytes
//   0x20 SRC_ADDR    RW src top-left byte address
//   0x28 SRC_STRIDE  RW src row bytes
//   0x30 WIDTH       RW pixels per row
//   0x38 HEIGHT      RW rows
//   0x40 COLOR       RW RGB565 in [15:0]
//   0x48 ALPHA       RW global alpha 0..255
//   0x50 MASK_ADDR   RW A8 mask top-left byte address (MASK_BLEND)
//   0x58 MASK_STRIDE RW A8 mask row bytes
//   0x60 CYCLES   R  cycle count of the last completed blit
//
// OP: 0 FILL, 1 COPY, 2 FILL_BLEND, 3 COPY_BLEND, 4 MASK_BLEND.
//
// MMIO timing matches the other devices: o_mmio_ready is combinational (the
// bus_splitter return FF gives the CPU its 1-cycle read latency).
//////////////////////////////////////////////////////////////////////////////////

module blitter_dma (
    input             i_Clk,
    input             i_Rst_L,

    // -------- MMIO slave (offsets within device window, addr[15:0]) --------
    input             i_mmio_write_DV,
    input             i_mmio_read_DV,
    input      [15:0] i_mmio_addr,
    input      [63:0] i_mmio_write_data,
    input      [ 7:0] i_mmio_byte_en,    // unused — software uses 64-bit accesses
    output reg [63:0] o_mmio_read_data,
    output            o_mmio_ready,

    // -------- DDR master (master B on mem_read_write's arbiter) --------
    output reg        o_dma_req,
    output reg        o_dma_done,
    output reg        o_dma_write_DV,
    output reg        o_dma_read_DV,
    output reg [31:0] o_dma_addr,
    output reg [127:0] o_dma_write_data,
    output reg [15:0] o_dma_wdf_mask,
    input      [127:0] i_dma_read_data,
    input             i_dma_ready,
    input             i_dma_grant,

    // -------- DONE interrupt (o_irq, wired into the CPU) --------
    output            o_irq
);

    // -------------------------------------------------------------------------
    // Register offsets
    // -------------------------------------------------------------------------
    localparam OFF_CTRL        = 16'h0000;
    localparam OFF_STATUS      = 16'h0008;
    localparam OFF_DST_ADDR    = 16'h0010;
    localparam OFF_DST_STRIDE  = 16'h0018;
    localparam OFF_SRC_ADDR    = 16'h0020;
    localparam OFF_SRC_STRIDE  = 16'h0028;
    localparam OFF_WIDTH       = 16'h0030;
    localparam OFF_HEIGHT      = 16'h0038;
    localparam OFF_COLOR       = 16'h0040;
    localparam OFF_ALPHA       = 16'h0048;
    localparam OFF_MASK_ADDR   = 16'h0050;
    localparam OFF_MASK_STRIDE = 16'h0058;
    localparam OFF_CYCLES      = 16'h0060;

    // Operation codes
    localparam OP_FILL       = 3'd0;
    localparam OP_COPY       = 3'd1;
    localparam OP_FILL_BLEND = 3'd2;
    localparam OP_COPY_BLEND = 3'd3;
    localparam OP_MASK_BLEND = 3'd4;

    assign o_mmio_ready = 1'b1;
    wire byte_en_unused = |i_mmio_byte_en;  // keep the port from being pruned

    // -------------------------------------------------------------------------
    // RGB565 blend: out = (fg*a + bg*(255-a) + 128) >> 8, per channel.
    // 5/6/5-bit channels blended directly; a+ia=255 keeps each result in range.
    // -------------------------------------------------------------------------
    function [15:0] blend565;
        input [15:0] fg;
        input [15:0] bg;
        input [ 7:0] a;
        reg   [ 7:0] ia;
        reg   [15:0] tr, tg, tb;    // 6b*8b + 6b*8b + 128 < 2^15, fits in 16b
        begin
            ia = 8'd255 - a;
            tr = fg[15:11]*a + bg[15:11]*ia + 16'd128;  // R5
            tg = fg[10:5] *a + bg[10:5] *ia + 16'd128;  // G6
            tb = fg[4:0]  *a + bg[4:0]  *ia + 16'd128;  // B5
            blend565 = { tr[12:8], tg[13:8], tb[12:8] };
        end
    endfunction

    // -------------------------------------------------------------------------
    // Operand registers (software-written via MMIO)
    // -------------------------------------------------------------------------
    reg [31:0] r_dst_addr;
    reg [31:0] r_dst_stride;
    reg [31:0] r_src_addr;
    reg [31:0] r_src_stride;
    reg [15:0] r_width;       // pixels (max 65535)
    reg [15:0] r_height;      // rows   (max 65535)
    reg [15:0] r_color;       // RGB565
    reg [ 7:0] r_alpha;       // global alpha
    reg [31:0] r_mask_addr;   // A8 mask base
    reg [31:0] r_mask_stride;
    reg [ 2:0] r_op;
    reg        r_irq_en;

    // -------------------------------------------------------------------------
    // Status / control handoff between the MMIO block and the engine FSM.
    // Single-owner discipline: the MMIO block produces 1-cycle pulses; the FSM
    // owns r_busy / r_done.
    // -------------------------------------------------------------------------
    reg  r_start_pulse;       // MMIO → FSM: begin a blit
    reg  r_done_clear_pulse;  // MMIO → FSM: clear DONE (W1C)
    reg  r_busy;              // FSM-owned
    reg  r_done;              // FSM-owned, sticky until W1C
    reg [31:0] r_cycles;      // running cycle count during a blit
    reg [31:0] r_cycles_last; // latched at completion (CYCLES register)

    assign o_irq = r_done & r_irq_en;

    // =========================================================================
    // MMIO write / read
    // =========================================================================
    always @(posedge i_Clk) begin
        if (!i_Rst_L) begin
            r_dst_addr        <= 32'h0;
            r_dst_stride      <= 32'h0;
            r_src_addr        <= 32'h0;
            r_src_stride      <= 32'h0;
            r_width           <= 16'h0;
            r_height          <= 16'h0;
            r_color           <= 16'h0;
            r_alpha           <= 8'h0;
            r_mask_addr       <= 32'h0;
            r_mask_stride     <= 32'h0;
            r_op              <= 3'h0;
            r_irq_en          <= 1'b0;
            r_start_pulse     <= 1'b0;
            r_done_clear_pulse<= 1'b0;
        end else begin
            r_start_pulse      <= 1'b0;   // pulses are 1 cycle
            r_done_clear_pulse <= 1'b0;
            if (i_mmio_write_DV) begin
                case (i_mmio_addr)
                    OFF_CTRL: begin
                        if (i_mmio_write_data[0] && !r_busy) begin
                            r_op          <= i_mmio_write_data[3:1];
                            r_irq_en      <= i_mmio_write_data[4];
                            r_start_pulse <= 1'b1;
                        end
                    end
                    OFF_STATUS:      r_done_clear_pulse <= i_mmio_write_data[1];
                    OFF_DST_ADDR:    r_dst_addr    <= i_mmio_write_data[31:0];
                    OFF_DST_STRIDE:  r_dst_stride  <= i_mmio_write_data[31:0];
                    OFF_SRC_ADDR:    r_src_addr    <= i_mmio_write_data[31:0];
                    OFF_SRC_STRIDE:  r_src_stride  <= i_mmio_write_data[31:0];
                    OFF_WIDTH:       r_width       <= i_mmio_write_data[15:0];
                    OFF_HEIGHT:      r_height      <= i_mmio_write_data[15:0];
                    OFF_COLOR:       r_color       <= i_mmio_write_data[15:0];
                    OFF_ALPHA:       r_alpha       <= i_mmio_write_data[7:0];
                    OFF_MASK_ADDR:   r_mask_addr   <= i_mmio_write_data[31:0];
                    OFF_MASK_STRIDE: r_mask_stride <= i_mmio_write_data[31:0];
                    default: ;
                endcase
            end
        end
    end

    always @* begin
        case (i_mmio_addr)
            OFF_STATUS:      o_mmio_read_data = {62'b0, r_done, r_busy};
            OFF_DST_ADDR:    o_mmio_read_data = {32'b0, r_dst_addr};
            OFF_DST_STRIDE:  o_mmio_read_data = {32'b0, r_dst_stride};
            OFF_SRC_ADDR:    o_mmio_read_data = {32'b0, r_src_addr};
            OFF_SRC_STRIDE:  o_mmio_read_data = {32'b0, r_src_stride};
            OFF_WIDTH:       o_mmio_read_data = {48'b0, r_width};
            OFF_HEIGHT:      o_mmio_read_data = {48'b0, r_height};
            OFF_COLOR:       o_mmio_read_data = {48'b0, r_color};
            OFF_ALPHA:       o_mmio_read_data = {56'b0, r_alpha};
            OFF_MASK_ADDR:   o_mmio_read_data = {32'b0, r_mask_addr};
            OFF_MASK_STRIDE: o_mmio_read_data = {32'b0, r_mask_stride};
            OFF_CYCLES:      o_mmio_read_data = {32'b0, r_cycles_last};
            default:         o_mmio_read_data = 64'h0;
        endcase
    end

    // =========================================================================
    // Engine FSM
    // =========================================================================
    localparam S_IDLE     = 5'd0;
    localparam S_PIX      = 5'd1;   // evaluate current pixel
    localparam S_PLACE    = 5'd2;   // write/blend pixel into the accumulator
    localparam S_WF_REQ   = 5'd3;   // write-flush: acquire bus
    localparam S_WF_DRIVE = 5'd4;   // write-flush: drive write, wait ready
    localparam S_WF_GAP   = 5'd5;   // write-flush: CDC settle
    localparam S_WF_REL   = 5'd6;   // write-flush: release bus, continue
    localparam S_RD_REQ   = 5'd7;   // generic read: acquire bus
    localparam S_RD_DRIVE = 5'd8;   // generic read: drive read, wait ready
    localparam S_RD_REL   = 5'd9;   // generic read: release bus, continue
    localparam S_END      = 5'd10;  // final flush dispatch
    localparam S_FINISH   = 5'd11;  // mark done
    localparam S_BLEND    = 5'd12;  // blend pipeline stage 1: latch operands (blend ops only)
    localparam S_BLEND2   = 5'd13;  // blend pipeline stage 2: register the 6 products (multiplies)

    // Read targets for the generic read sub-FSM.
    localparam RDT_SRC  = 2'd0;
    localparam RDT_DST  = 2'd1;
    localparam RDT_MASK = 2'd2;

    reg [4:0]  state;
    // Blend pipeline registers (cut the long mask-addr -> alpha -> blend -> wbuf
    // path in two: S_BLEND latches these operands, S_PLACE does the blend math).
    reg [15:0] r_fg_q;          // foreground pixel (src or COLOR)
    reg [15:0] r_bg_q;          // background pixel (current dst)
    reg [ 7:0] r_alpha_q;       // alpha (mask A8 or global)
    // Stage-2 blend products (fg_ch*a, bg_ch*ia per channel) registered in
    // S_BLEND2 so S_PLACE only does add+pack — splits the blend565 chain so it
    // closes on the default impl strategy (was a 13-level path at the 10ns edge).
    reg [15:0] r_mul_fg_r, r_mul_bg_r;   // R5 products
    reg [15:0] r_mul_fg_g, r_mul_bg_g;   // G6 products
    reg [15:0] r_mul_fg_b, r_mul_bg_b;   // B5 products
    reg [15:0] r_x;             // current pixel column
    reg [15:0] r_row;           // current row
    reg [31:0] r_dst_row_base;  // byte address of current dst row start
    reg [31:0] r_src_row_base;  // byte address of current src row start
    reg [31:0] r_mask_row_base; // byte address of current mask row start

    reg [127:0] r_wbuf;         // destination accumulator
    reg [31:0]  r_wbuf_addr;    // 16-byte-aligned address of r_wbuf
    reg [15:0]  r_wbuf_mask;    // per-byte write mask (1 = don't write)
    reg         r_wbuf_open;    // a destination word is currently adopted
    reg         r_wbuf_dirty;   // >=1 pixel written into the current word

    reg [127:0] r_rbuf;         // source read buffer
    reg [31:0]  r_rbuf_addr;
    reg         r_rbuf_valid;

    reg [127:0] r_mbuf;         // A8 mask read buffer
    reg [31:0]  r_mbuf_addr;
    reg         r_mbuf_valid;

    reg [1:0]   r_rd_target;    // which buffer the in-flight read fills
    reg [31:0]  r_rd_addr;
    reg [3:0]   r_gap;          // CDC settle countdown
    reg         r_flush_final;  // 1 = the in-flight flush is the final one

    // Op classification.
    wire w_is_blend = (r_op == OP_FILL_BLEND) ||
                      (r_op == OP_COPY_BLEND) ||
                      (r_op == OP_MASK_BLEND);
    wire w_use_src  = (r_op == OP_COPY) || (r_op == OP_COPY_BLEND);
    wire w_use_mask = (r_op == OP_MASK_BLEND);

    // Combinational pixel-address decode from the current cursors.
    wire [31:0] w_dst_byte  = r_dst_row_base + {15'b0, r_x, 1'b0};   // + r_x*2
    wire [31:0] w_dst_word  = {w_dst_byte[31:4], 4'b0};
    wire [2:0]  w_dst_off   = w_dst_byte[3:1];
    wire [31:0] w_src_byte  = r_src_row_base + {15'b0, r_x, 1'b0};
    wire [31:0] w_src_word  = {w_src_byte[31:4], 4'b0};
    wire [2:0]  w_src_off   = w_src_byte[3:1];
    wire [31:0] w_mask_byte = r_mask_row_base + {16'b0, r_x};        // A8: 1 B/px
    wire [31:0] w_mask_word = {w_mask_byte[31:4], 4'b0};
    wire [3:0]  w_mask_off  = w_mask_byte[3:0];

    // Per-pixel data path. For opaque ops, w_fg is the pixel written directly in
    // S_PLACE. For blend ops, w_fg/w_bg/w_alpha are latched in S_BLEND and the
    // blend math runs in S_PLACE from the registered operands (timing pipeline).
    wire [15:0] w_src_pixel  = r_rbuf[w_src_off*16 +: 16];
    wire [15:0] w_fg         = w_use_src ? w_src_pixel : r_color;
    wire [15:0] w_bg         = r_wbuf[w_dst_off*16 +: 16];
    wire [ 7:0] w_alpha      = w_use_mask ? r_mbuf[w_mask_off*8 +: 8] : r_alpha;

    // Fetch needs for the current pixel.
    wire w_need_wflush  = r_wbuf_open && r_wbuf_dirty && (w_dst_word != r_wbuf_addr);
    wire w_need_srcfill = w_use_src  && (!r_rbuf_valid || (w_src_word  != r_rbuf_addr));
    wire w_need_maskfill= w_use_mask && (!r_mbuf_valid || (w_mask_word != r_mbuf_addr));
    wire w_last_pixel   = (r_x == r_width - 16'd1);

    // BYTE-ORDER FIX: the cache (mem_read_write) maps CPU doubleword addr[3]=0 to
    // the UPPER 64 bits [127:64] of a 128-bit line, the opposite of the raw-line /
    // ddr2_control convention ([63:0]=beat0=bytes 0-7) that this blitter's DMA uses.
    // CPU-written src is therefore stored with its two 64-bit halves swapped vs our
    // view (invisible to memcpy + blitter-only sim; only the cross-path CPU-writes /
    // blitter-reads case exposes it -> the dst+8 beat-slip). Swap the two halves on
    // every DMA read so r_rbuf/r_mbuf/r_wbuf are in our logical order, and swap back
    // on write (data + byte-mask). FILL/aligned copies are unaffected (the swap
    // cancels for verbatim data); unaligned re-packing is fixed.
    wire [127:0] w_dma_rd_swap = {i_dma_read_data[63:0], i_dma_read_data[127:64]};

    always @(posedge i_Clk) begin
        if (!i_Rst_L) begin
            state          <= S_IDLE;
            r_busy         <= 1'b0;
            r_done         <= 1'b0;
            r_cycles       <= 32'h0;
            r_cycles_last  <= 32'h0;
            o_dma_req      <= 1'b0;
            o_dma_done     <= 1'b0;
            o_dma_write_DV <= 1'b0;
            o_dma_read_DV  <= 1'b0;
            o_dma_addr     <= 32'h0;
            o_dma_write_data <= 128'h0;
            o_dma_wdf_mask <= 16'hFFFF;
            r_wbuf_open    <= 1'b0;
            r_wbuf_dirty   <= 1'b0;
            r_rbuf_valid   <= 1'b0;
            r_mbuf_valid   <= 1'b0;
        end else begin
            if (r_done_clear_pulse) r_done <= 1'b0;   // DONE is W1C
            if (r_busy) r_cycles <= r_cycles + 32'd1;  // blit cycle counter
            o_dma_done <= 1'b0;                         // done is a 1-cycle pulse

            case (state)

                // ----------------------------------------------------------
                S_IDLE: begin
                    if (r_start_pulse) begin
                        r_busy         <= 1'b1;
                        r_done         <= 1'b0;
                        r_cycles       <= 32'h0;
                        r_x            <= 16'h0;
                        r_row          <= 16'h0;
                        r_dst_row_base <= r_dst_addr;
                        r_src_row_base <= r_src_addr;
                        r_mask_row_base<= r_mask_addr;
                        r_wbuf_open    <= 1'b0;
                        r_wbuf_dirty   <= 1'b0;
                        r_rbuf_valid   <= 1'b0;
                        r_mbuf_valid   <= 1'b0;
                        if (r_width == 16'h0 || r_height == 16'h0) begin
                            // Degenerate (empty) blit: no pixels, but still report
                            // a nonzero CYCLES so software polling the count never
                            // mistakes a completed empty blit for "never ran".
                            r_cycles <= 32'h1;
                            state    <= S_FINISH;
                        end else begin
                            state <= S_PIX;
                        end
                    end
                end

                // ----------------------------------------------------------
                // Evaluate the current pixel. Priority:
                //   1. all rows done            -> S_END
                //   2. dst word changed (dirty) -> flush the accumulator
                //   3. no word open             -> open one (blend: read dst)
                //   4. need src / mask word     -> read it
                //   5. otherwise                -> place the pixel
                // ----------------------------------------------------------
                S_PIX: begin
                    if (r_row == r_height) begin
                        state <= S_END;
                    end else if (w_need_wflush) begin
                        r_flush_final <= 1'b0;
                        state <= S_WF_REQ;
                    end else if (!r_wbuf_open) begin
                        if (w_is_blend) begin
                            // Read the destination word (blend background).
                            r_rd_target <= RDT_DST;
                            r_rd_addr   <= w_dst_word;
                            state       <= S_RD_REQ;
                        end else begin
                            // Opaque: adopt an empty masked accumulator.
                            r_wbuf_addr  <= w_dst_word;
                            r_wbuf_mask  <= 16'hFFFF;
                            r_wbuf_open  <= 1'b1;
                            r_wbuf_dirty <= 1'b0;
                            state        <= S_PIX;
                        end
                    end else if (w_need_srcfill) begin
                        r_rd_target <= RDT_SRC;
                        r_rd_addr   <= w_src_word;
                        state       <= S_RD_REQ;
                    end else if (w_need_maskfill) begin
                        r_rd_target <= RDT_MASK;
                        r_rd_addr   <= w_mask_word;
                        state       <= S_RD_REQ;
                    end else begin
                        // Blend ops take the S_BLEND pipeline stage first (latch
                        // operands), then S_PLACE; opaque ops go straight to S_PLACE.
                        state <= w_is_blend ? S_BLEND : S_PLACE;
                    end
                end

                // ----------------------------------------------------------
                // Blend pipeline stage 1 — latch operands (cursor stable, so the
                // S_PLACE write position w_dst_off is unchanged). This breaks the
                // mask-addr -> alpha-mux -> blend -> wbuf path across two cycles.
                S_BLEND: begin
                    r_fg_q    <= w_fg;
                    r_bg_q    <= w_bg;
                    r_alpha_q <= w_alpha;
                    state     <= S_BLEND2;
                end

                // ----------------------------------------------------------
                // Blend pipeline stage 2 — register the 6 products (the heavy
                // multiplies). S_PLACE then does the add+128+pack. This splits
                // the old 13-level blend565 chain (~10ns at the edge) so it
                // closes comfortably on the DEFAULT impl strategy. Result is
                // bit-identical to blend565(); blended pixels just cost +1 cycle
                // (negligible — the blitter is DDR-bound). Cursor is stable.
                S_BLEND2: begin : blend_mul
                    reg [7:0] ia;
                    ia = 8'd255 - r_alpha_q;
                    r_mul_fg_r <= r_fg_q[15:11] * r_alpha_q;
                    r_mul_bg_r <= r_bg_q[15:11] * ia;
                    r_mul_fg_g <= r_fg_q[10:5]  * r_alpha_q;
                    r_mul_bg_g <= r_bg_q[10:5]  * ia;
                    r_mul_fg_b <= r_fg_q[4:0]   * r_alpha_q;
                    r_mul_bg_b <= r_bg_q[4:0]   * ia;
                    state      <= S_PLACE;
                end

                // ----------------------------------------------------------
                S_PLACE: begin
                    // Opaque: write w_fg directly (short path). Blend: add the
                    // registered products + 128 and pack (the light second half).
                    if (w_is_blend) begin : blend_add
                        reg [15:0] tr, tg, tb;
                        tr = r_mul_fg_r + r_mul_bg_r + 16'd128;  // R5
                        tg = r_mul_fg_g + r_mul_bg_g + 16'd128;  // G6
                        tb = r_mul_fg_b + r_mul_bg_b + 16'd128;  // B5
                        r_wbuf[w_dst_off*16 +: 16] <= {tr[12:8], tg[13:8], tb[12:8]};
                    end else begin
                        r_wbuf[w_dst_off*16 +: 16] <= w_fg;
                        r_wbuf_mask[w_dst_off*2 +: 2] <= 2'b00;  // mark 2 bytes valid
                    end
                    r_wbuf_dirty <= 1'b1;
                    // Advance cursor.
                    if (w_last_pixel) begin
                        r_x             <= 16'h0;
                        r_row           <= r_row + 16'd1;
                        r_dst_row_base  <= r_dst_row_base  + r_dst_stride;
                        r_src_row_base  <= r_src_row_base  + r_src_stride;
                        r_mask_row_base <= r_mask_row_base + r_mask_stride;
                    end else begin
                        r_x <= r_x + 16'd1;
                    end
                    state <= S_PIX;
                end

                // ----------------------------------------------------------
                // Write-flush: push r_wbuf to DDR with its byte mask.
                // ----------------------------------------------------------
                S_WF_REQ: begin
                    o_dma_req <= 1'b1;
                    if (i_dma_grant) begin
                        o_dma_addr       <= r_wbuf_addr;
                        o_dma_write_data <= {r_wbuf[63:0], r_wbuf[127:64]};        // swap back (see w_dma_rd_swap)
                        o_dma_wdf_mask   <= {r_wbuf_mask[7:0], r_wbuf_mask[15:8]};  // mask follows the data halves
                        o_dma_write_DV   <= 1'b1;
                        state            <= S_WF_DRIVE;
                    end
                end

                S_WF_DRIVE: begin
                    if (i_dma_ready) begin
                        o_dma_write_DV <= 1'b0;
                        r_gap          <= 4'd7;     // CDC settle
                        state          <= S_WF_GAP;
                    end
                end

                S_WF_GAP: begin
                    if (r_gap == 4'd0) begin
                        o_dma_done <= 1'b1;        // release the bus
                        state      <= S_WF_REL;
                    end else begin
                        r_gap <= r_gap - 4'd1;
                    end
                end

                S_WF_REL: begin
                    o_dma_req    <= 1'b0;
                    r_wbuf_open  <= 1'b0;          // accumulator retired
                    r_wbuf_dirty <= 1'b0;
                    state        <= r_flush_final ? S_FINISH : S_PIX;
                end

                // ----------------------------------------------------------
                // Generic read: load r_rd_addr into the buffer named by
                // r_rd_target (src / dst-background / mask).
                // ----------------------------------------------------------
                S_RD_REQ: begin
                    o_dma_req <= 1'b1;
                    if (i_dma_grant) begin
                        o_dma_addr    <= r_rd_addr;
                        o_dma_read_DV <= 1'b1;
                        state         <= S_RD_DRIVE;
                    end
                end

                S_RD_DRIVE: begin
                    if (i_dma_ready) begin
                        case (r_rd_target)
                            RDT_SRC: begin
                                r_rbuf       <= w_dma_rd_swap;
                                r_rbuf_addr  <= o_dma_addr;
                                r_rbuf_valid <= 1'b1;
                            end
                            RDT_MASK: begin
                                r_mbuf       <= w_dma_rd_swap;
                                r_mbuf_addr  <= o_dma_addr;
                                r_mbuf_valid <= 1'b1;
                            end
                            default: begin // RDT_DST — open the accumulator
                                r_wbuf       <= w_dma_rd_swap;
                                r_wbuf_addr  <= o_dma_addr;
                                r_wbuf_mask  <= 16'h0000;  // write whole word back
                                r_wbuf_open  <= 1'b1;
                                r_wbuf_dirty <= 1'b0;
                            end
                        endcase
                        o_dma_read_DV <= 1'b0;
                        o_dma_done    <= 1'b1;    // reads are idempotent — no gap
                        state         <= S_RD_REL;
                    end
                end

                S_RD_REL: begin
                    o_dma_req <= 1'b0;
                    state     <= S_PIX;
                end

                // ----------------------------------------------------------
                S_END: begin
                    if (r_wbuf_open && r_wbuf_dirty) begin
                        r_flush_final <= 1'b1;
                        state <= S_WF_REQ;
                    end else begin
                        state <= S_FINISH;
                    end
                end

                S_FINISH: begin
                    r_busy        <= 1'b0;
                    r_done        <= 1'b1;
                    r_cycles_last <= r_cycles;
                    state         <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

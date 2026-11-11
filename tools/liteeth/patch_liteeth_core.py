#!/usr/bin/env python3
"""Apply the KlaussCPU project-level hand edits to a freshly generated
liteeth_core.v (see REGENERATE.md, "What we changed at the project level").

    python3 patch_liteeth_core.py generated/gateware/liteeth_core.v \
        ../../KlaussCPU.srcs/sources_1/new/liteeth_core.v

Every edit is an exact text replacement that must match exactly once, so a
LiteEth/LiteX version that changes the generated text fails loudly here
instead of silently producing an unpatched core.

Edits:
  1. Remove (* dont_touch *) from the rmii_clocks_ref_clk port (Synth 8-5396).
  2. ETH_RX_SLOTS (HW_PERF_PLAN.md §3): the core is generated with
     nrxslots: 8; two new inputs make the sram_writer use only the first
     rx_nslots (2..8) of them, and flush its state on program load:
       - rx_nslots      the slot counter and the stat FIFO pointers wrap
                        at rx_nslots, and the stat FIFO (one entry per
                        filled slot) is "full" at rx_nslots entries, so a
                        frame that finds every slot in use is dropped and
                        counted in RX_ERRORS, exactly as with a core
                        generated with nrxslots = rx_nslots.
       - rx_slot_reset  one-cycle pulse: slot counter -> 0, stat FIFO
                        emptied (no RX event pending), writer FSM to WRITE
                        (or DISCARD-ALL if a frame is arriving, so its tail
                        is not stored as a truncated frame).  RX_ERRORS and
                        everything else are untouched.
"""
import sys

W = "wishbone_interface_writer_"

EDITS = [
    # 1. dont_touch on the RMII reference clock port.
    (
        "    (* dont_touch = \"true\" *)\n"
        "    input  wire          rmii_clocks_ref_clk,\n",
        "    // dont_touch removed from rmii_clocks_ref_clk: the attribute on a clock\n"
        "    // input port causes Synth 8-5396 (extra LUT logic on the WCLK net of the\n"
        "    // distributed-RAM CRC/stat FIFOs).  The actual CDC sync FFs in this\n"
        "    // module already carry their own async_reg + dont_touch attributes, so\n"
        "    // there is nothing protected at port level that would be lost.\n"
        "    input  wire          rmii_clocks_ref_clk,\n",
    ),
    # 2a. New ports.
    (
        "    input  wire          rmii_crs_dv,\n",
        "    input  wire          rmii_crs_dv,\n"
        "    // KlaussCPU patch (ETH_RX_SLOTS, tools/liteeth/patch_liteeth_core.py):\n"
        "    // RX slots in use (2..8, from the bridge's ETH_RX_SLOTS) and a\n"
        "    // one-cycle sram_writer flush (program load).  sys_clock domain.\n"
        "    input  wire    [3:0] rx_nslots,\n"
        "    input  wire          rx_slot_reset,\n",
    ),
    # 2b. Stat FIFO full at rx_nslots entries (was: != nrxslots).
    (
        f"assign {W}stat_fifo_syncfifo_writable = ({W}stat_fifo_level != 4'd8);\n",
        f"assign {W}stat_fifo_syncfifo_writable = ({W}stat_fifo_level < rx_nslots);  // KlaussCPU patch\n",
    ),
    # 2c. Slot counter wraps at rx_nslots (was: 3-bit wrap at 8).
    (
        f"            {W}slot_liteethmac_liteethmacsramwriter_next_value <= ({W}slot + 1'd1);\n",
        f"            {W}slot_liteethmac_liteethmacsramwriter_next_value <= "
        f"((({{1'd0, {W}slot}} + 4'd1) >= rx_nslots) ? 3'd0 : ({W}slot + 1'd1));  // KlaussCPU patch\n",
    ),
    # 2d. Stat FIFO pointers wrap at rx_nslots too (was: 3-bit wrap at 8), so
    #     the produce pointer always equals the slot counter and, at 2 slots,
    #     the FIFO behaves exactly like the depth-2 FIFO of an nrxslots: 2
    #     core (including what RX_SLOT reads while nothing is pending).
    (
        f"        {W}stat_fifo_produce <= ({W}stat_fifo_produce + 1'd1);\n",
        f"        {W}stat_fifo_produce <= "
        f"((({{1'd0, {W}stat_fifo_produce}} + 4'd1) >= rx_nslots) ? 3'd0 : ({W}stat_fifo_produce + 1'd1));  // KlaussCPU patch\n",
    ),
    (
        f"        {W}stat_fifo_consume <= ({W}stat_fifo_consume + 1'd1);\n",
        f"        {W}stat_fifo_consume <= "
        f"((({{1'd0, {W}stat_fifo_consume}} + 4'd1) >= rx_nslots) ? 3'd0 : ({W}stat_fifo_consume + 1'd1));  // KlaussCPU patch\n",
    ),
    # 2e. Flush on rx_slot_reset (after the normal updates, before sys_rst).
    (
        f"    if ({W}slot_liteethmac_liteethmacsramwriter_next_value_ce) begin\n"
        f"        {W}slot <= {W}slot_liteethmac_liteethmacsramwriter_next_value;\n"
        f"    end\n",
        f"    if ({W}slot_liteethmac_liteethmacsramwriter_next_value_ce) begin\n"
        f"        {W}slot <= {W}slot_liteethmac_liteethmacsramwriter_next_value;\n"
        f"    end\n"
        f"    // KlaussCPU patch: rx_slot_reset flushes the sram_writer.  A frame\n"
        f"    // that is arriving (WRITE with data written or a beat now, or one of\n"
        f"    // the DISCARD-REMAINING / DISCARD-ALL states) finishes in DISCARD-ALL.\n"
        f"    if (rx_slot_reset) begin\n"
        f"        {W}slot <= 3'd0;\n"
        f"        {W}length <= 11'd0;\n"
        f"        {W}stat_fifo_level <= 4'd0;\n"
        f"        {W}stat_fifo_produce <= 3'd0;\n"
        f"        {W}stat_fifo_consume <= 3'd0;\n"
        f"        if ((((liteethmacsramwriter_state == 3'd0) & (({W}length != 11'd0) | {W}sink_sink_valid))"
        f" | (liteethmacsramwriter_state == 3'd1) | (liteethmacsramwriter_state == 3'd2))"
        f" & (~({W}sink_sink_valid & {W}sink_sink_last))) begin\n"
        f"            liteethmacsramwriter_state <= 3'd2;\n"
        f"        end else begin\n"
        f"            liteethmacsramwriter_state <= 3'd0;\n"
        f"        end\n"
        f"    end\n",
    ),
]


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src = open(sys.argv[1]).read()
    for old, new in EDITS:
        n = src.count(old)
        if n != 1:
            sys.exit(f"patch_liteeth_core: expected 1 match, found {n}:\n{old}")
        src = src.replace(old, new)
    open(sys.argv[2], "w").write(src)


if __name__ == "__main__":
    main()

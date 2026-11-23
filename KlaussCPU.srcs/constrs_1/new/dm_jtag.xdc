# On-chip debugger (DEBUG_EN=1): the BSCANE2 USER3 TCK clock of dm_jtag.
# Host JTAG runs at <= 30 MHz.  TCK is asynchronous to every core clock; the
# DMI crossing in dm_dtm uses toggle synchronizers (ASYNC_REG) with stable
# data registers, so all paths between the domains are false paths.
create_clock -period 33.333 -name dm_tck [get_pins -hierarchical -filter {NAME =~ *dm_jtag_i/bscan_i/TCK}]
set_clock_groups -asynchronous -group [get_clocks dm_tck]

# The debug module (~1.5k LUTs) apart from the core, in the top-right
# corner (build dbg_aa8505a: unconstrained it sat at X54-80 Y116-172, at
# the core's edge, beside the TLB's t1_pahi / mem_pa2hi paths, WNS -0.052).
# Its links to the core are registered on the core side (pipeline_core
# dq_bp_* / dq_wp_*) or are register reads while halted.
create_pblock pb_dm
add_cells_to_pblock [get_pblocks pb_dm] [get_cells -quiet {g_debug.debug_module_i g_debug.dm_jtag_i}]
resize_pblock [get_pblocks pb_dm] -add {SLICE_X50Y170:SLICE_X79Y199}

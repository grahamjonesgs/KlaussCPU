# floorplan.xdc — HW_PERF_PLAN §10 (timing hardening): implementation-only
# placement constraints.
#
# The TLB (mmu_tlb: 32 entries x {VPN, ASID, PFN, XWR}, the data and fetch
# CAMs; ~2.6k LUTs, 1.8k FFs) is held in one compact block in the middle of
# the core's region.  Its data-CAM paths (the key copies q_a / q_b -> the
# per-entry page match -> the PFN OR-tree -> t1_pahi / mem_pa2hi / mem_iaddr
# / the hit pipe's tag) are 7-8 LUT levels with ~1 ns nets; unconstrained,
# the placer pushed the TLB into a corner of the core in some runs (the
# family then led the list at +0.007).  The block is ~26 x 38 slices,
# ~70 % full.
create_pblock pb_tlb
add_cells_to_pblock [get_pblocks pb_tlb] [get_cells -hier -filter {NAME =~ pipeline_core_i/g_tlb.tlb_i}]
resize_pblock [get_pblocks pb_tlb] -add {SLICE_X20Y130:SLICE_X45Y167}

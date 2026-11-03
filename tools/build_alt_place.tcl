# build_alt_place.tcl — re-place/route from build_fast.tcl's post-opt checkpoint
# with a different placer directive, when tier 1 + tier 2 just miss.
#
#   vivado -mode batch -source tools/build_alt_place.tcl \
#          -tclargs <place_directive> <out_dir> [<opt_dcp>]
#
# <opt_dcp> defaults to KlaussCPU.runs/impl_1/KlaussCPU_opt.dcp next to this
# script.  Flow: place_design -directive <d> -> phys_opt AggressiveExplore ->
# route AggressiveExplore -> the build_fast tier-2 phys_opt sequence while
# not met.  Writes <out_dir>/final.dcp, timing.rpt, and KlaussCPU.bit only
# when setup and hold are met.  Used for crypto fb6bf4e (2026-11-03):
# Performance_Explore -0.009 -> ExtraPostPlacementOpt +0.050 (ExtraTimingOpt
# +0.012).  Two variants fit in parallel in 27 GB.
set pd [lindex $argv 0]
set out [lindex $argv 1]
proc wns {} { return [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] }
proc whs {} { return [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]] }
proc met {} { return [expr {[wns] >= 0.0 && [whs] >= 0.0}] }
set_param general.maxThreads 6
set opt_dcp [expr {[llength $argv] > 2 ? [lindex $argv 2] : [file normalize [file join [file dirname [info script]] .. KlaussCPU.runs impl_1 KlaussCPU_opt.dcp]]}]
open_checkpoint $opt_dcp
place_design -directive $pd
phys_opt_design -directive AggressiveExplore
route_design -directive AggressiveExplore
puts [format "ALT %s routed: WNS %+.3f WHS %+.3f" $pd [wns] [whs]]
foreach d {AggressiveExplore AlternateFlowWithRetiming AggressiveExplore} {
  if {[met]} break
  phys_opt_design -directive $d
  puts [format "ALT %s physopt %s: WNS %+.3f WHS %+.3f" $pd $d [wns] [whs]]
}
write_checkpoint -force $out/final.dcp
report_timing_summary -max_paths 10 -file $out/timing.rpt
if {[met]} { write_bitstream -force $out/KlaussCPU.bit; puts "ALT $pd MET" } else { puts "ALT $pd NOT MET" }

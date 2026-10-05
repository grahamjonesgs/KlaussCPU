# build_fast.tcl — M12 Stage A two-tier build flow (PIPELINE_M12_TIMING.md)
#
# Tier 1: synth_1 -> impl_1 (Performance_Explore) up to route_design, with
#         post-route phys_opt DISABLED and incremental implementation OFF.
# Tier 2: only on a timing miss, run phys_opt on the routed checkpoint
#         in-session: AggressiveExplore -> AlternateFlowWithRetiming ->
#         AggressiveExplore, stopping as soon as setup AND hold are met.
# The bitstream is written only when timing is met (unless -force_bit).
#
# INCREMENTAL PLACEMENT IS CONDEMNED on this design / Vivado 2025.2 (placer
# errors, silently poisoned QoR, a 9-hour wedge) — this script turns it off.
#
# Usage (from anywhere):
#   vivado -mode batch -source tools/build_fast.tcl [-tclargs <options>]
# Options:
#   -jobs N       parallel runs for launch_runs (default 4; lower on 8 GB RAM)
#   -threads N    general.maxThreads for synth/place/route (default 8;
#                 Vivado's Windows default is 2)
#   -resynth      force synth_1 to re-run even if it is up to date
#   -tier2_only   skip tier 1; run tier 2 on the existing routed checkpoint
#   -force_bit    write the bitstream even if timing is not met
#
# Outputs (in KlaussCPU.runs/impl_1/):
#   KlaussCPU.bit                         — only when met (or -force_bit)
#   KlaussCPU_postroute_physopt.dcp       — when tier 2 ran
#   build_fast_timing.rpt                 — final timing summary
# Exit code: 0 = met, 1 = timing not met, 2 = run failure.

set t_start [clock seconds]
set proj_dir [file normalize [file join [file dirname [info script]] ..]]
set proj_xpr [file join $proj_dir KlaussCPU.xpr]

set opt_jobs 4
set opt_threads 8
set opt_resynth 0
set opt_tier2_only 0
set opt_force_bit 0
for {set i 0} {$i < [llength $argv]} {incr i} {
   switch -- [lindex $argv $i] {
      -jobs       { incr i; set opt_jobs [lindex $argv $i] }
      -threads    { incr i; set opt_threads [lindex $argv $i] }
      -resynth    { set opt_resynth 1 }
      -tier2_only { set opt_tier2_only 1 }
      -force_bit  { set opt_force_bit 1 }
      default     { puts "build_fast: unknown option [lindex $argv $i]"; exit 2 }
   }
}

set TIER2_DIRECTIVES {AggressiveExplore AlternateFlowWithRetiming AggressiveExplore}

proc elapsed {t0} {
   set s [expr {[clock seconds] - $t0}]
   return [format "%dm%02ds" [expr {$s / 60}] [expr {$s % 60}]]
}

proc bf_log {msg} { puts "\n#### build_fast: $msg\n" }

proc run_ok {run} {
   return [expr {[get_property PROGRESS [get_runs $run]] eq "100%" &&
                 ![string match "*ERROR*" [get_property STATUS [get_runs $run]]]}]
}

# Worst setup / hold slack of the currently open design.
proc wns {} { return [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] }
proc whs {} { return [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]] }
proc met {} { return [expr {[wns] >= 0.0 && [whs] >= 0.0}] }

set_param general.maxThreads $opt_threads
open_project $proj_xpr
set top      [get_property top [current_fileset]]
set impl_dir [get_property DIRECTORY [get_runs impl_1]]
set routed   [file join $impl_dir ${top}_routed.dcp]

# ---------------------------------------------------------------- tier 1
if {!$opt_tier2_only} {
   # Pin the tier-1 configuration so a GUI tweak can't silently change it.
   set impl [get_runs impl_1]
   set_property strategy Performance_Explore $impl
   set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED false $impl
   set_property AUTO_INCREMENTAL_CHECKPOINT 0 $impl
   set_property INCREMENTAL_CHECKPOINT "" $impl

   # IP output products are not tracked in git (they differ per OS — see
   # .gitignore), so a fresh checkout has only the IP sources.  Generate
   # whatever is missing or stale; a no-op when everything is up to date.
   set t0 [clock seconds]
   generate_target all [get_ips]
   bf_log "IP output products checked/generated in [elapsed $t0]"

   # An IP synthesised out-of-context (MIG) also needs the checkpoint + stub
   # its own synth run writes next to the .xci.  After a pull deletes them the
   # run can still look complete, and synth_1 then fails with "module not
   # found" — so re-run that IP's synth run whenever its .dcp is missing.
   # (Core-container IP, e.g. clk_wiz_0_1.xcix, keeps its products inside the
   # .xcix; its .xci path isn't a file on disk, so it's skipped.)
   foreach ip [get_ips] {
      set xci [get_property IP_FILE $ip]
      if {![file exists $xci]} continue
      if {![get_property GENERATE_SYNTH_CHECKPOINT [get_files $xci]]} continue
      if {[file exists "[file rootname $xci].dcp"]} continue
      set run [get_runs -quiet ${ip}_synth_1]
      if {$run eq ""} {
         bf_log "IP $ip has no checkpoint and no ${ip}_synth_1 run"
         exit 2
      }
      set t0 [clock seconds]
      bf_log "IP $ip: checkpoint missing, re-running $run"
      reset_run $run
      launch_runs $run -jobs $opt_jobs
      wait_on_run $run
      if {![run_ok $run]} {
         bf_log "$run FAILED: [get_property STATUS $run]"
         exit 2
      }
      bf_log "$run done in [elapsed $t0]"
   }

   set synth [get_runs synth_1]
   if {$opt_resynth || [get_property NEEDS_REFRESH $synth] || ![run_ok synth_1]} {
      set t0 [clock seconds]
      bf_log "synth_1 (jobs=$opt_jobs)"
      reset_run synth_1
      launch_runs synth_1 -jobs $opt_jobs
      wait_on_run synth_1
      if {![run_ok synth_1]} {
         bf_log "synth_1 FAILED: [get_property STATUS $synth]"
         exit 2
      }
      bf_log "synth_1 done in [elapsed $t0]"
   } else {
      bf_log "synth_1 up to date, skipping"
   }

   set t0 [clock seconds]
   bf_log "impl_1 tier 1 -> route_design"
   reset_run impl_1
   launch_runs impl_1 -to_step route_design -jobs $opt_jobs
   wait_on_run impl_1
   if {![file exists $routed]} {
      bf_log "impl_1 FAILED: [get_property STATUS [get_runs impl_1]]"
      exit 2
   }
   bf_log "impl_1 tier 1 done in [elapsed $t0]"
}

if {![file exists $routed]} {
   bf_log "no routed checkpoint at $routed — run without -tier2_only first"
   exit 2
}

open_checkpoint $routed
set tier1_wns [wns]
set tier1_whs [whs]
bf_log [format "tier 1: WNS %+.3f  WHS %+.3f" $tier1_wns $tier1_whs]

# ---------------------------------------------------------------- tier 2
set tier2_ran 0
if {![met]} {
   foreach d $TIER2_DIRECTIVES {
      set t0 [clock seconds]
      set before [wns]
      phys_opt_design -directive $d
      set tier2_ran 1
      bf_log [format "tier 2 %s: WNS %+.3f -> %+.3f  WHS %+.3f  (%s)" \
                 $d $before [wns] [whs] [elapsed $t0]]
      if {[met]} break
   }
   write_checkpoint -force [file join $impl_dir ${top}_postroute_physopt.dcp]
}

report_timing_summary -max_paths 10 -file [file join $impl_dir build_fast_timing.rpt]
set final_wns [wns]
set final_whs [whs]
set is_met [met]

if {$is_met || $opt_force_bit} {
   write_bitstream -force [file join $impl_dir ${top}.bit]
}

set summary [format "tier 1 WNS %+.3f / WHS %+.3f" $tier1_wns $tier1_whs]
if {$tier2_ran} {
   append summary [format ", tier 2 WNS %+.3f / WHS %+.3f" $final_wns $final_whs]
}
if {$is_met} {
   bf_log "MET ($summary). Bitstream: [file join $impl_dir ${top}.bit]. Total [elapsed $t_start]"
   exit 0
} elseif {$opt_force_bit} {
   bf_log "NOT MET ($summary). Bitstream written anyway (-force_bit). Total [elapsed $t_start]"
   exit 1
} else {
   bf_log "NOT MET ($summary). No bitstream written. Total [elapsed $t_start]"
   exit 1
}

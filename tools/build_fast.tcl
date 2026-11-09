# build_fast.tcl — M12 Stage A two-tier build flow (PIPELINE_M12_TIMING.md)
#
# Tier 1: synth_1 -> impl_1 (Performance_Explore) up to route_design, with
#         post-route phys_opt DISABLED and incremental implementation OFF.
# Tier 2: only on a timing miss, run phys_opt on the routed checkpoint
#         in-session: AggressiveExplore -> AlternateFlowWithRetiming ->
#         AggressiveExplore, stopping as soon as setup AND hold are met.
# The bitstream is written only when timing is met (unless -force_bit).
#
# INCREMENTAL PLACEMENT was CONDEMNED on this design / Vivado 2025.2 (placer
# errors, silently poisoned QoR, a 9-hour wedge): it is OFF by default.
# -incr <routed.dcp> opts in for a quick turnaround on a small RTL change
# (2026-11-09, Graham: fast builds), with a wall-clock cap; its bitstream is
# for testing, and a release still comes from a full (non-incremental) run.
#
# Run builds ONE AT A TIME (each wants most of the machine); on a narrow
# miss, tools/build_alt_place.tcl re-places from the opt checkpoint —
# sequentially, after the build, not in parallel with another build.
#
# OOC IP checkpoints (the MIG) are not in git, so every fresh git-archive
# copy used to re-synthesise them (~1 min).  They are now cached across
# copies in ~/.cache/kbuild/ipcache/<sha1 of the .xci + .prj + Vivado
# version> (KBUILD_IPCACHE overrides the location; -no_ipcache disables).
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
#   -incr DCP     incremental synthesis + place and route from a reference
#                 build: DCP is its routed checkpoint (<dir>/KlaussCPU.runs/
#                 impl_1/KlaussCPU_routed.dcp); its synth checkpoint next to
#                 it (../synth_1/KlaussCPU.dcp) seeds incremental synthesis,
#                 which keeps the netlist names stable so placement can be
#                 reused.  Capped at -incr_cap minutes (default 30); on a
#                 failure or the cap it falls back to the full flow.  OFF by
#                 default.
#   -incr_cap M   the incremental run's wall-clock cap in minutes
#   -no_ipcache   do not use / fill the OOC IP checkpoint cache
#   -stop_after_synth   stop once synthesis (and the IP cache) is done
#   -quick        TEST build: impl_1 with the default strategy (Vivado
#                 Implementation Defaults, much faster than
#                 Performance_Explore), no tier 2, bitstream always written
#                 (implies -force_bit).  For board bring-up only — finish with
#                 a normal run before committing results that depend on timing.
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
set opt_quick 0
set opt_incr ""
set opt_incr_cap 30
set opt_ipcache 1
set opt_stop_synth 0
for {set i 0} {$i < [llength $argv]} {incr i} {
   switch -- [lindex $argv $i] {
      -jobs       { incr i; set opt_jobs [lindex $argv $i] }
      -threads    { incr i; set opt_threads [lindex $argv $i] }
      -resynth    { set opt_resynth 1 }
      -tier2_only { set opt_tier2_only 1 }
      -force_bit  { set opt_force_bit 1 }
      -quick      { set opt_quick 1; set opt_force_bit 1 }
      -incr       { incr i; set opt_incr [file normalize [lindex $argv $i]] }
      -incr_cap   { incr i; set opt_incr_cap [lindex $argv $i] }
      -no_ipcache { set opt_ipcache 0 }
      -stop_after_synth { set opt_stop_synth 1 }
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

# ---- the OOC IP cache (before anything else touches the project)
set ipcache [expr {[info exists ::env(KBUILD_IPCACHE)] ? $::env(KBUILD_IPCACHE)
                                                        : [file normalize ~/.cache/kbuild/ipcache]}]
set runs_dir [file join $proj_dir KlaussCPU.runs]
# The cache entry of an OOC IP is its state after its synth run: the run
# directory (Vivado judges the run from it — without it launch_runs
# synth_1 re-runs the IP) and the IP's own files (ip.tar: the .xci
# directory, whose regenerated .xml lists the checkpoint / stubs — a fresh
# generate_target leaves them out and synth_1 cannot find the module; or
# a core container's .xcix, which the run rewrites, and its directory).
# Keyed by the IP's source file(s) as checked out and the Vivado version.
proc ip_key {ip_file} {
   set files [list $ip_file]
   if {[file extension $ip_file] eq ".xci"} {
      set files [concat $files [glob -nocomplain -directory [file dirname $ip_file] *.prj]]
   }
   return [lindex [exec sh -c "{ cat [join [lsort $files] { }]; echo [version -short]; } | sha1sum"] 0]
}
array set ip_cdir {}
set ip_restored 0
foreach ip [get_ips] {
   set ipf [get_property IP_FILE $ip]
   set run [get_runs -quiet ${ip}_synth_1]
   if {$run eq ""} continue
   if {![file exists $ipf]} {
      # a core-container IP (clk_wiz_0 in clk_wiz_0_1.xcix): key by the .xcix
      set ipf [lindex [glob -nocomplain -directory [file join $proj_dir KlaussCPU.srcs sources_1 ip] ${ip}*.xcix] 0]
      if {$ipf eq ""} continue
   }
   set cdir [file join $ipcache ${ip}_[ip_key $ipf]]
   set ip_cdir($ip) $cdir
   set rdir [file join $runs_dir ${ip}_synth_1]
   if {$opt_ipcache && [file exists [file join $cdir run.tar]] && [file exists [file join $cdir ip.tar]]
       && ![file exists $rdir]} {
      file mkdir $runs_dir
      exec tar -xf [file join $cdir run.tar] -C $runs_dir
      exec tar -xf [file join $cdir ip.tar] -C [file join $proj_dir KlaussCPU.srcs sources_1 ip]
      bf_log "IP $ip: run and IP products restored from the cache ($cdir)"
      set ip_restored 1
   }
}
# reopen so the restored run directories are seen
if {$ip_restored} {
   close_project
   open_project $proj_xpr
}
set top      [get_property top [current_fileset]]
set impl_dir [get_property DIRECTORY [get_runs impl_1]]
set routed   [file join $impl_dir ${top}_routed.dcp]

# ---------------------------------------------------------------- tier 1
if {!$opt_tier2_only} {
   # Pin the tier-1 configuration so a GUI tweak can't silently change it.
   set impl [get_runs impl_1]
   set_property strategy [expr {$opt_quick ? "Vivado Implementation Defaults"
                                           : "Performance_Explore"}] $impl
   set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED false $impl
   set_property AUTO_INCREMENTAL_CHECKPOINT 0 $impl
   set synth_run [get_runs synth_1]
   set_property AUTO_INCREMENTAL_CHECKPOINT 0 $synth_run
   if {$opt_incr ne ""} {
      if {![file exists $opt_incr]} { bf_log "-incr: no checkpoint $opt_incr"; exit 2 }
      set incr_synth [file normalize [file join [file dirname $opt_incr] .. synth_1 ${top}.dcp]]
      set_property INCREMENTAL_CHECKPOINT $opt_incr $impl
      set_property INCREMENTAL_CHECKPOINT.DIRECTIVE RuntimeOptimized $impl
      if {[file exists $incr_synth]} {
         set_property INCREMENTAL_CHECKPOINT $incr_synth $synth_run
      } else {
         set incr_synth "(none)"
      }
      bf_log "INCREMENTAL from $opt_incr, synthesis from $incr_synth (RuntimeOptimized, cap ${opt_incr_cap} min)"
   } else {
      set_property INCREMENTAL_CHECKPOINT "" $impl
      set_property INCREMENTAL_CHECKPOINT "" $synth_run
   }

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
      if {[file extension $xci] ne ".xci"} continue
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
   # NEEDS_REFRESH misses sources edited outside Vivado (a batch session
   # opens the project fresh), so also re-synthesise when any design source
   # or constraint file is newer than the synth checkpoint.
   set synth_dcp [file join [get_property DIRECTORY $synth] ${top}.dcp]
   set stale_src ""
   if {[file exists $synth_dcp]} {
      set dcp_time [file mtime $synth_dcp]
      foreach f [concat [get_files -quiet -of_objects [get_filesets sources_1]] \
                        [get_files -quiet -of_objects [get_filesets constrs_1]]] {
         if {[file exists $f] && [file mtime $f] > $dcp_time} { set stale_src $f; break }
      }
   }
   if {$stale_src ne ""} { bf_log "synth_1 stale: $stale_src is newer than the checkpoint" }
   if {$opt_resynth || $stale_src ne "" || [get_property NEEDS_REFRESH $synth] || ![run_ok synth_1]} {
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

   if {$opt_ipcache} {
      foreach ip [array names ip_cdir] {
         set cdir $ip_cdir($ip)
         set rdir [file join $runs_dir ${ip}_synth_1]
         if {[file exists [file join $cdir run.tar]] || ![file isdirectory $rdir]} continue
         if {![run_ok ${ip}_synth_1]} continue
         file mkdir $cdir
         exec tar -cf [file join $cdir run.tar] -C $runs_dir ${ip}_synth_1
         # the IP's own files, relative to sources_1/ip
         set ipf [get_property IP_FILE [get_ips $ip]]
         set iproot [file join $proj_dir KlaussCPU.srcs sources_1 ip]
         if {[file exists $ipf]} {
            set members [list [file tail [file dirname $ipf]]]
         } else {
            set xcix [lindex [glob -nocomplain -directory $iproot ${ip}*.xcix] 0]
            set members [list [file tail $xcix]]
            set xdir [file rootname $xcix]
            if {[file isdirectory $xdir]} { lappend members [file tail $xdir] }
         }
         exec tar -cf [file join $cdir ip.tar] -C $iproot {*}$members
         bf_log "IP $ip: run and IP products cached ($cdir)"
      }
   }
   if {$opt_stop_synth} { bf_log "stopping after synthesis (-stop_after_synth). Total [elapsed $t_start]"; exit 0 }

   set t0 [clock seconds]
   bf_log "impl_1 tier 1 -> route_design"
   reset_run impl_1
   launch_runs impl_1 -to_step route_design -jobs $opt_jobs
   if {$opt_incr ne ""} {
      # (the cap: incremental placement once wedged for 9 hours here; and
      # the incremental placer can simply fail — then run the full flow)
      set incr_err [catch {wait_on_run -timeout $opt_incr_cap impl_1} incr_msg]
      if {$incr_err || ![file exists $routed]} {
         bf_log "INCREMENTAL impl failed or exceeded ${opt_incr_cap} min after [elapsed $t0]: falling back to the full flow"
         reset_run impl_1
         set_property INCREMENTAL_CHECKPOINT "" [get_runs impl_1]
         set t0 [clock seconds]
         launch_runs impl_1 -to_step route_design -jobs $opt_jobs
         wait_on_run impl_1
      }
   } else {
      wait_on_run impl_1
   }
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
if {$opt_incr ne "" && !$opt_tier2_only} {
   # how much of the reference was reused (cells / nets)
   report_incremental_reuse -file [file join $impl_dir build_fast_incr_reuse.rpt]
   bf_log "incremental reuse report: [file join $impl_dir build_fast_incr_reuse.rpt]"
}
set tier1_wns [wns]
set tier1_whs [whs]
bf_log [format "tier 1: WNS %+.3f  WHS %+.3f" $tier1_wns $tier1_whs]

# ---------------------------------------------------------------- tier 2
set tier2_ran 0
if {![met] && !$opt_quick} {
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
if {$opt_quick} { set summary "QUICK build, $summary" }
if {$tier2_ran} {
   append summary [format ", tier 2 WNS %+.3f / WHS %+.3f" $final_wns $final_whs]
}
if {$is_met} {
   bf_log "MET ($summary). Bitstream: [file join $impl_dir ${top}.bit]. Total [elapsed $t_start]"
   exit 0
} elseif {$opt_force_bit} {
   bf_log "NOT MET ($summary). Bitstream written anyway ([expr {$opt_quick ? "-quick" : "-force_bit"}]). Total [elapsed $t_start]"
   exit 1
} else {
   bf_log "NOT MET ($summary). No bitstream written. Total [elapsed $t_start]"
   exit 1
}

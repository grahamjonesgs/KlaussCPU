# sd_io_timing.tcl — the SD pads' I/O delays from a routed checkpoint, and
# the high-speed (50 MHz) sample window they predict for each sample phase
# (SD_NATIVE_PLAN.md §9.4).
#
#   vivado -mode batch -source tools/sd_io_timing.tcl -tclargs <routed.dcp> [board_ns]
#
# The SD pins keep their false paths (the sample point is calibrated at run
# time), so report_timing still gives every path's delays, marked "False
# Path".  All pad registers (the CLK ODDR, the CMD / DAT OLOGIC flip-flops,
# the IDDRs) are clocked by ui_clk in the same clock region (X1Y2, bank 35),
# so the delays below are taken relative to the clock at those cells:
#   out(pin)  = clock-to-pad: OLOGIC / ODDR C->Q + OBUF(T)  (slow max, fast min)
#   in(pin)   = pad to the IDDR's D: IBUF + IDELAYE2 at tap 0 (slow max, fast min)
#   skew      = each pad cell's clock arrival minus the CLK ODDR's
# With board_ns (default 0.3) for each direction of the traces, the bit the
# card launches on CLK's rising edge (fabric edge R at the ODDR) reaches the
# IDDR between
#   R + out(CLK) + board + tODLY + board + in(DAT)          (valid from)
#   R + 20 + out(CLK) + board + tOH + board + in(DAT)       (valid until)
# with tODLY <= 14 ns and tOH >= 2.5 ns (SD high speed), each at one corner
# (slow, fast: a chip's delays move together), less the skew of the IDDRs'
# clock against the ODDR's; phase p samples it at R + 10 + 5p ns (minus
# 78 ps per IDELAY tap), modulo the 20 ns clock.

set dcp [lindex $argv 0]
set board [expr {[llength $argv] > 1 ? [lindex $argv 1] : 0.3}]
open_checkpoint $dcp

proc pdelay {from to corner} {
    # data path delay of the worst path from -> to at a corner (max: slow, min: fast)
    set p [get_timing_paths -from $from -to $to -delay_type $corner -max_paths 1 -nworst 1]
    if {$p eq ""} { return "" }
    return [list [get_property DATAPATH_DELAY $p] [get_property ENDPOINT_CLOCK_DELAY $p] [get_property STARTPOINT_CLOCK_DELAY $p]]
}

set oddr [get_cells -hier -filter {REF_NAME == ODDR && NAME =~ sd_spi_i/*}]
set sdports [get_ports {o_SD_SCK io_SD_CMD io_SD_DAT[*]}]
# the outputs: every pad register (ODDR, OLOGIC output / tristate flip-flop)
set worst_out {}
foreach r [get_cells -hier -filter {NAME =~ sd_spi_i/* && (REF_NAME == ODDR || PRIMITIVE_GROUP == FLOP_LATCH)}] {
    if {![string match "OLOGIC*" [get_property LOC $r]]} continue
    set mx [get_timing_paths -from $r -to $sdports -delay_type max -max_paths 1]
    set mn [get_timing_paths -from $r -to $sdports -delay_type min -max_paths 1]
    if {$mx eq ""} continue
    puts [format "SDIO out %-28s %-12s %-14s  slow %.3f  fast %.3f ns" $r [get_property ENDPOINT_PIN $mx] \
          [get_property LOC $r] [get_property DATAPATH_DELAY $mx] [get_property DATAPATH_DELAY $mn]]
}
set cmx [get_timing_paths -from $oddr -to [get_ports o_SD_SCK] -delay_type max -max_paths 1]
set cmn [get_timing_paths -from $oddr -to [get_ports o_SD_SCK] -delay_type min -max_paths 1]
set out_s [get_property DATAPATH_DELAY $cmx]; set out_f [get_property DATAPATH_DELAY $cmn]
# the inputs: pad -> IDDR D (IBUF + IDELAYE2 at tap 0)
set in_s 0; set in_f 99
foreach d [get_cells -hier -filter {REF_NAME == IDDR && NAME =~ sd_spi_i/*}] {
    set mx [get_timing_paths -from $sdports -to $d -delay_type max -max_paths 1]
    set mn [get_timing_paths -from $sdports -to $d -delay_type min -max_paths 1]
    set a [get_property DATAPATH_DELAY $mx]; set b [get_property DATAPATH_DELAY $mn]
    puts [format "SDIO in  %-12s -> %-28s %-14s  slow %.3f  fast %.3f ns" [get_property STARTPOINT_PIN $mx] $d \
          [get_property LOC $d] $a $b]
    if {$a > $in_s} { set in_s $a }
    if {$b < $in_f} { set in_f $b }
}
# the clock at the pad cells: the ODDR (X1Y132) vs the IDDRs (X1Y113..121),
# both as the capture clock of an internal path into them (slow corner)
set ck_o [get_property ENDPOINT_CLOCK_DELAY [get_timing_paths -to [get_pins $oddr/D1] -delay_type max -max_paths 1]]
set ck_i [get_property ENDPOINT_CLOCK_DELAY [get_timing_paths -from $sdports -to [get_cells -hier -filter {REF_NAME == IDDR && NAME =~ sd_spi_i/*}] -delay_type max -max_paths 1]]
set skew [expr {$ck_i - $ck_o}]
puts [format "SDIO clock at the ODDR %.3f, at the IDDRs %.3f ns (slow): the IDDRs sample %.3f ns after the ODDR's edge" $ck_o $ck_i $skew]
puts [format "SDIO CLK clock-to-pad slow %.3f fast %.3f; CMD/DAT pad-to-IDDR (tap 0) slow %.3f fast %.3f; board %.2f ns each way" \
      $out_s $out_f $in_s $in_f $board]
# the window at one corner (a chip is at one corner: all its delays move together)
foreach {nm odly oh} {spec 14.0 2.5 typical 7.0 2.5} {
    foreach {corner o i} [list slow $out_s $in_s fast $out_f $in_f] {
        set lo [expr {$o + $board + $odly + $board + $i - $skew}]
        set hi [expr {20.0 + $o + $board + $oh + $board + $i - $skew}]
        set line [format "SDIO %-7s card (tODLY %4.1f, tOH %.1f), %s corner: valid %5.2f .. %5.2f ns after R; phases:" $nm $odly $oh $corner $lo $hi]
        for {set p 0} {$p < 7} {incr p} {
            set t [expr {10.0 + 5.0 * $p}]
            while {$t >= $lo + 20.0} { set t [expr {$t - 20.0}] }
            while {$t < $lo} { set t [expr {$t + 20.0}] }
            if {$t < $hi} { append line [format "  %d:P(%.1f)" $p [expr {min($t - $lo, $hi - $t)}]] } else { append line [format "  %d:." $p] }
        }
        puts $line
    }
}

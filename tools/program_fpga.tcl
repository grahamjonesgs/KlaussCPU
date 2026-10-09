# program_fpga.tcl — JTAG-program the Nexys A7 (volatile; lost on power cycle).
#
# Usage:
#   vivado -mode batch -nojournal -source tools/program_fpga.tcl [-tclargs <file.bit>]
# Default bitstream: KlaussCPU.runs/impl_1/KlaussCPU.bit
# Prints "JTAG_PROGRAM: DONE" on success; exits 1 on any failure.

set proj_dir [file normalize [file join [file dirname [info script]] ..]]
set bit [file join $proj_dir KlaussCPU.runs impl_1 KlaussCPU.bit]
if {[llength $argv] > 0} { set bit [file normalize [lindex $argv 0]] }
if {![file exists $bit]} { puts "JTAG_PROGRAM: no bitstream at $bit"; exit 1 }

open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets -quiet]
if {[llength $targets] == 0} {
   puts "JTAG_PROGRAM: no JTAG target found — is the board's USB attached to this host?"
   exit 1
}
current_hw_target [lindex $targets 0]
open_hw_target
set dev [lindex [get_hw_devices -quiet xc7a100t*] 0]
if {$dev eq ""} { puts "JTAG_PROGRAM: no xc7a100t on [current_hw_target]"; exit 1 }
current_hw_device $dev
set_property PROGRAM.FILE $bit $dev
puts "JTAG_PROGRAM: programming $dev with $bit"
program_hw_devices $dev
puts "JTAG_PROGRAM: DONE"
close_hw_target
disconnect_hw_server
close_hw_manager
exit 0

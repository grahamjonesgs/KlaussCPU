#!/usr/bin/env bash
# Build the DEBUG_EN=1 (on-chip debugger) bitstream, see DEBUG_PLAN.md.
#
# Works on a copy of the project, so the checked-in KlaussCPU.xpr (which
# does not list debug_module.sv / dm_jtag.xdc) and its runs stay untouched:
# rsync the working tree to OUT/klaussCPU, add the debug sources, set the
# generic DEBUG_EN=1, then run tools/build_fast.tcl there.
#
#   tools/build_debug.sh [OUT]       (default ~/.cache/klausscpu-debug-build)
#
# Bitstream: OUT/klaussCPU/KlaussCPU.runs/impl_1/KlaussCPU.bit.  Program it
# (volatile; a power cycle returns to the flash image) with e.g.
#   openFPGALoader -b nexys_a7_100 <bit>
# Extra arguments for build_fast.tcl: BUILD_ARGS (default
# "-jobs 4 -threads 12 -resynth").  Needs vivado on PATH (or VIVADO_BIN).
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
OUT=${1:-$HOME/.cache/klausscpu-debug-build}
VIVADO=${VIVADO_BIN:-vivado}
mkdir -p "$OUT"
rsync -a --delete --exclude .git --exclude '*.runs' --exclude '*.cache' --exclude '*.hw' \
  --exclude '*.sim' --exclude '*.gen' --exclude perf "$REPO/" "$OUT/klaussCPU/"
cat >"$OUT/prep.tcl" <<'TCL'
open_project [file join [file dirname [info script]] klaussCPU KlaussCPU.xpr]
set src [file join [file dirname [info script]] klaussCPU KlaussCPU.srcs]
add_files -fileset sources_1 [file join $src sources_1 new debug_module.sv]
add_files -fileset constrs_1 [file join $src constrs_1 new dm_jtag.xdc]
set_property generic {DEBUG_EN=1} [get_filesets sources_1]
puts "prep: top=[get_property top [current_fileset]] generic=[get_property generic [get_filesets sources_1]]"
close_project
TCL
cd "$OUT"
"$VIVADO" -mode batch -source prep.tcl -nojournal -log prep.log >/dev/null
# shellcheck disable=SC2086
"$VIVADO" -mode batch -source klaussCPU/tools/build_fast.tcl -nojournal -log build.log \
  -tclargs ${BUILD_ARGS:--jobs 4 -threads 12 -resynth}
echo "bitstream: $OUT/klaussCPU/KlaussCPU.runs/impl_1/KlaussCPU.bit"

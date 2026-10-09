#!/usr/bin/env bash
# VGA_PLAN.md Phase 0/1 gate: vga_ctrl vs VESA 640x480@60 (tb_vga.sv).
# Checks HS/VS periods, widths and polarity, the active window between the
# porches, the test-pattern / border / palette-view content, the MMIO
# registers, palette readback, the vsync IRQ and the FB_BASE vblank latch
# (~7 frames, ~120 ms sim).
#   ./run_vga.sh
set -euo pipefail
cd "$(dirname "$0")"
SRC=$(cd ../../KlaussCPU.srcs && pwd)
export PATH=$PATH:/opt/Xilinx/2025.2/Vivado/bin:/c/AMDDesignTools/2025.2/Vivado/bin
mkdir -p out vga_run
cd vga_run

xvlog -sv \
  "$SRC/sources_1/new/mmio_if.sv" \
  "$SRC/sources_1/new/vga_timing.sv" \
  "$SRC/sources_1/new/vga_ctrl.sv" \
  "$SRC/sim_1/new/tb_vga.sv" > xvlog.log 2>&1 \
  || { tail -30 xvlog.log; exit 1; }
xelab tb_vga -s tvga --relax > xelab.log 2>&1 \
  || { tail -30 xelab.log; exit 1; }
xsim tvga -runall > xsim.log 2>&1 || { tail -30 xsim.log; exit 1; }

grep -E "frame|vsync|FAIL|PASS|TIMEOUT" xsim.log
grep -q "TB_VGA PASS" xsim.log && echo "VGA-P1: PASS" || { echo "VGA-P1: FAIL"; exit 1; }

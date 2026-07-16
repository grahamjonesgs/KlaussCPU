#!/usr/bin/env bash
# M10a cache unit gate: mem_read_write + the REAL ddr2_control on a beat-level
# fake MIG (tb_cache.sv). True sim coverage of critical-word-first command
# reorder, the early dword channel / early restart, fetch-first shadow
# writebacks, DMA-grant lockout, and maintenance walks.
#   ./run_m10_cache.sh
set -euo pipefail
cd "$(dirname "$0")"
SRC=$(cd ../../KlaussCPU.srcs && pwd)
export PATH=$PATH:/opt/Xilinx/2025.2/Vivado/bin
mkdir -p out cache_run
cd cache_run

xvlog -sv \
  "$SRC/sources_1/new/membus_if.sv" \
  "$SRC/sources_1/new/ddr2_control.sv" \
  "$SRC/sources_1/new/mem_read_write.sv" \
  "$SRC/sim_1/new/tb_cache.sv" > xvlog.log 2>&1 \
  || { tail -30 xvlog.log; exit 1; }
xelab tb_cache -s tcache --relax > xelab.log 2>&1 \
  || { tail -30 xelab.log; exit 1; }
xsim tcache -runall > xsim.log 2>&1 || { tail -30 xsim.log; exit 1; }

grep -E "Phase|PASS|FAIL|TIMEOUT|latency|miss @" xsim.log
grep -q "TB_CACHE PASS" xsim.log && echo "M10-CACHE: PASS" || { echo "M10-CACHE: FAIL"; exit 1; }

#!/bin/bash
# Run the LiteEth RX-slots unit testbench (sim_1/new/tb_eth_slots.sv) under
# Verilator (>= 5.020, --timing) with the real generated liteeth_core.v and
# the behavioural Xilinx primitives in eth_prims.sv.  (xsim reports a
# zero-delay oscillation at time 0 in the generated core.)
#   new   the working tree's eth_mmio_bridge.sv + liteeth_core.v
#   old   +define+OLD_CORE against the 2-slot core and bridge from git
#         ($ETH_OLD_REV, default 8926cdd, before ETH_RX_SLOTS)
#   all   both; then the phase-A "OBS" transcripts must be identical
# Work dir: $ETHSIM_WORK (default ~/.cache/ethsim).  No waveforms are dumped.
# Usage: tools/ethsim/run_eth_tb.sh [new|old|all]
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
S=$ROOT/KlaussCPU.srcs
WORK=${ETHSIM_WORK:-$HOME/.cache/ethsim}
OLD_REV=${ETH_OLD_REV:-8926cdd}
which=${1:-all}
rc=0
run() {   # $1 = name, $2 = extra verilator flags, rest = sources
    local name=$1 flags=$2; shift 2
    local d=$WORK/$name
    rm -rf "$d"; mkdir -p "$d"; cd "$d"
    verilator --binary --timing -j 0 -Wno-fatal -Wno-lint -Wno-style -Wno-MULTIDRIVEN \
        -Wno-UNOPTFLAT -Wno-BLKANDNBLK --timescale 1ns/1ps --top-module tb_eth_slots \
        --Mdir obj $flags $ROOT/tools/ethsim/eth_prims.sv "$@" $S/sim_1/new/tb_eth_slots.sv \
        > build.log 2>&1 || { grep -E "%Error" build.log | head -20; return 1; }
    ./obj/Vtb_eth_slots > sim.log 2>&1 || true
    grep -E "^TB_ETH|MISMATCH" sim.log | head -40
    sed -n '/^OBS phase A/,/^OBS end phase A/p' sim.log > obs.txt
    grep -qE "^TB_ETH PASS" sim.log
}
if [ "$which" = new ] || [ "$which" = all ]; then
    echo "== new (working tree)"
    run new "" $S/sources_1/new/mmio_if.sv $S/sources_1/new/eth_mmio_bridge.sv \
        $S/sources_1/new/liteeth_core.v || rc=1
fi
if [ "$which" = old ] || [ "$which" = all ]; then
    echo "== old ($OLD_REV, 2 RX slots)"
    o=$WORK/old_src; rm -rf "$o"; mkdir -p "$o"
    git -C "$ROOT" show "$OLD_REV:KlaussCPU.srcs/sources_1/new/eth_mmio_bridge.sv" > "$o/eth_mmio_bridge.sv"
    git -C "$ROOT" show "$OLD_REV:KlaussCPU.srcs/sources_1/new/liteeth_core.v" > "$o/liteeth_core.v"
    run old "+define+OLD_CORE" $S/sources_1/new/mmio_if.sv "$o/eth_mmio_bridge.sv" "$o/liteeth_core.v" || rc=1
fi
if [ "$which" = all ]; then
    if [ -s "$WORK/new/obs.txt" ] && cmp -s "$WORK/new/obs.txt" "$WORK/old/obs.txt"; then
        echo "TB_ETH 2-slot transcript identical to $OLD_REV ($(wc -l < "$WORK/new/obs.txt") OBS lines)"
    else
        echo "TB_ETH 2-slot transcript DIFFERS from $OLD_REV"
        diff "$WORK/old/obs.txt" "$WORK/new/obs.txt" | head -20
        rc=1
    fi
fi
exit $rc

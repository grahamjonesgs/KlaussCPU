#!/bin/bash
# Run the SD controller unit testbenches under xsim (Vivado 2025.2):
#   tb_sd_native      native SD mode against sd_card_model
#   tb_sd_spi_compat  SPI mode against the old sd_spi (sd_spi_ref)
# Work dir: $SDSIM_WORK (default ~/.cache/sdsim).  No waveforms are dumped.
# Usage: tools/sdsim/run_sd_tb.sh [native|compat|all]
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
S=$ROOT/KlaussCPU.srcs
WORK=${SDSIM_WORK:-$HOME/.cache/sdsim}
export PATH=/opt/xilinx/2025.2/Vivado/bin:$PATH
which=${1:-all}
rc=0
run() {   # $1 = top, rest = sources
    local top=$1; shift
    local d=$WORK/$top
    rm -rf "$d"; mkdir -p "$d"; cd "$d"
    xvlog -sv "$@" > xvlog.log 2>&1 || { tail -20 xvlog.log; return 1; }
    xelab -debug off "$top" > xelab.log 2>&1 || { grep -i error xelab.log | head; return 1; }
    xsim "$top" -R > xsim.log 2>&1 || true
    grep -E "^TB_|FAIL|compared|card:" xsim.log | head -40
    grep -qE "^TB_[A-Z]+ PASS" xsim.log
}
# mmio_if.sv has no `timescale: compile it after a file that sets one
if [ "$which" = native ] || [ "$which" = all ]; then
    run tb_sd_native $S/sources_1/new/sd_native.sv $S/sources_1/new/mmio_if.sv $S/sources_1/new/sd_spi.sv \
        $S/sim_1/new/sd_card_model.sv $S/sim_1/new/tb_sd_native.sv || rc=1
fi
if [ "$which" = compat ] || [ "$which" = all ]; then
    run tb_sd_spi_compat $S/sources_1/new/sd_native.sv $S/sources_1/new/mmio_if.sv $S/sources_1/new/sd_spi.sv \
        $S/sim_1/new/sd_spi_ref.sv $S/sim_1/new/tb_sd_spi_compat.sv || rc=1
fi
exit $rc

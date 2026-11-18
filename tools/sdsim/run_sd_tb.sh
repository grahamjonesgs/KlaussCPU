#!/bin/bash
# Run the SD controller unit testbenches under xsim (Vivado 2025.2):
#   tb_sd_native      native SD mode against sd_card_model
#   tb_sd_spi_compat  SPI mode against the old sd_spi (sd_spi_ref)
#   tb_sd_dma         the SD DMA engine against sd_card_model and a DDR port
#                     model (SDDMA_SEED=<n>: its random latencies)
# tb_sd_native and tb_sd_dma also run a lock-step reference (SD_M0): sd_spi /
# sd_native as of $SD_M0_REV (default 6dcbcfb, the release before phase 3),
# renamed *_m0, on the same MMIO and pins; until the high-speed sampling is
# first enabled every pin and read must match it cycle for cycle.
# The pads use the Xilinx primitives (ODDR, IDDR, IDELAYE2, IDELAYCTRL):
# the unisims_ver library and glbl.
# tb_sd_native also runs tools/board/sd_hs_test.c's pick() (DPI-C, built by xsc with
# -DHOST_TEST) on its high-speed sweeps.
# Work dir: $SDSIM_WORK (default ~/.cache/sdsim).  No waveforms are dumped.
# Usage: tools/sdsim/run_sd_tb.sh [native|compat|dma|all]
set -e -o pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
S=$ROOT/KlaussCPU.srcs
WORK=${SDSIM_WORK:-$HOME/.cache/sdsim}
export PATH=/opt/xilinx/2025.2/Vivado/bin:$PATH
GLBL=/opt/xilinx/2025.2/Vivado/data/verilog/src/glbl.v
which=${1:-all}
rc=0
# the lock-step reference (SD_M0), from the git repository $SD_M0_GIT
# (default: this tree's; a copy outside git, e.g. a mutant, names its origin)
M0_REV=${SD_M0_REV:-6dcbcfb}
M0_GIT=${SD_M0_GIT:-$ROOT}
mkdir -p "$WORK"
git -C "$M0_GIT" show "$M0_REV:KlaussCPU.srcs/sources_1/new/sd_native.sv" \
    | sed -e 's/\bsd_native\b/sd_native_m0/g' > "$WORK/sd_native_m0.sv"
git -C "$M0_GIT" show "$M0_REV:KlaussCPU.srcs/sources_1/new/sd_spi.sv" \
    | sed -e 's/\bsd_spi\b/sd_spi_m0/g' -e 's/\bsd_native\b/sd_native_m0/g' > "$WORK/sd_spi_m0.sv"
M0="$WORK/sd_native_m0.sv $WORK/sd_spi_m0.sv"
run() {   # $1 = top, rest = sources; $DPI_C: C sources of DPI-C imports (built -DHOST_TEST)
    local top=$1; shift
    local d=$WORK/$top svlib=
    rm -rf "$d"; mkdir -p "$d"; cd "$d"
    if [ -n "${DPI_C:-}" ]; then
        xsc --gcc_compile_options -DHOST_TEST $DPI_C > xsc.log 2>&1 || { tail -20 xsc.log; return 1; }
        svlib="-sv_lib dpi"
    fi
    xvlog -sv $XVLOG_DEFS "$@" $GLBL > xvlog.log 2>&1 || { tail -20 xvlog.log; return 1; }
    xelab -debug off -L unisims_ver $svlib -s "${top}_sim" "work.$top" work.glbl > xelab.log 2>&1 || { grep -i error xelab.log | head; return 1; }
    xsim "${top}_sim" -R $XSIM_ARGS > xsim.log 2>&1 || true
    grep -E "^TB_|FAIL|compared|card:|^HS |^M0 " xsim.log | head -60
    grep -qE "^TB_[A-Z]+ PASS" xsim.log
}
# mmio_if.sv has no `timescale: compile it after a file that sets one
if [ "$which" = native ] || [ "$which" = all ]; then
    # (DPI-C: tools/board/sd_hs_test.c's pick(), run on the HS sweeps)
    XVLOG_DEFS="-d SD_M0" DPI_C=$ROOT/tools/board/sd_hs_test.c \
    run tb_sd_native $S/sources_1/new/sd_native.sv $S/sources_1/new/mmio_if.sv $S/sources_1/new/sd_spi.sv $M0 \
        $S/sim_1/new/sd_card_model.sv $S/sim_1/new/tb_sd_native.sv || rc=1
fi
if [ "$which" = compat ] || [ "$which" = all ]; then
    run tb_sd_spi_compat $S/sources_1/new/sd_native.sv $S/sources_1/new/mmio_if.sv $S/sources_1/new/sd_spi.sv \
        $S/sim_1/new/sd_spi_ref.sv $S/sim_1/new/tb_sd_spi_compat.sv || rc=1
fi
if [ "$which" = dma ] || [ "$which" = all ]; then
    XSIM_ARGS="-testplusarg SEED=${SDDMA_SEED:-1} ${SDDMA_PLUS:-}" XVLOG_DEFS="-d SD_M0" \
    run tb_sd_dma $S/sources_1/new/sd_native.sv $S/sources_1/new/mmio_if.sv $S/sources_1/new/sd_spi.sv $M0 \
        $S/sim_1/new/sd_card_model.sv $S/sim_1/new/tb_sd_dma.sv || rc=1
fi
exit $rc

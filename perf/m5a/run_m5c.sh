#!/usr/bin/env bash
# M5c verification: precise-interrupt / WAIT / SMC tests on the pipeline core.
#   ./run_m5c.sh            — directed WAIT + SMC tests, then IRQ storms on
#                             bst (fixed latency) and test_64bit (random
#                             latency), diffing the program-only trace
#                             (handler lines at 0x100xxx stripped, i= and f=
#                             fields excluded) against the no-IRQ goldens.
# Assumes run_m5a.sh has generated bst/test_64bit goldens + the tpisa snapshot.
set -euo pipefail
cd "$(dirname "$0")"
export PATH=$PATH:/opt/Xilinx/2025.2/Vivado/bin
SRC=$(cd ../../KlaussCPU.srcs && pwd)
mkdir -p out

xvlog -sv "$SRC/sources_1/new/klauss_pkg.sv" \
          "$SRC/sources_1/new/pipeline_core.sv" \
          "$SRC/sim_1/new/tb_pipeline_isa.sv" > out/xvlog.log 2>&1 \
  || { tail -30 out/xvlog.log; exit 1; }
xelab tb_pipeline_isa -s tpisa > out/xelab.log 2>&1 \
  || { tail -30 out/xelab.log; exit 1; }

RES=0
xsim tpisa -runall -testplusarg "TEST=wait" -testplusarg "TRACE=out/wait.trace" \
  -testplusarg "UARTF=out/wait.uart" | grep "TB_M5C" | tee /dev/stderr | grep -q PASS || RES=1
xsim tpisa -runall -testplusarg "TEST=smc" -testplusarg "TRACE=out/smc.trace" \
  -testplusarg "UARTF=out/smc.uart" | grep "TB_M5C" | tee /dev/stderr | grep -q PASS || RES=1

storm () {  # prog period extra_args golden_uart
  local P=$1 PER=$2; shift 2
  xsim tpisa -runall -testplusarg "IMAGE=$P.mem" -testplusarg "TRACE=out/$P.storm.trace" \
    -testplusarg "UARTF=out/$P.storm.uart" -testplusarg "IRQ_PERIOD=$PER" "$@" \
    | grep "TB_M5A: [0-9s]" >&2
  awk '$0 !~ / pc=001000/' "out/$P.storm.trace" | sed -e 's/^i=[0-9]* //' -e 's/ f=[01]*//' > "out/$P.storm.nf"
  sed -e 's/^i=[0-9]* //' -e 's/ f=[01]*//' "$P.trace" > "out/$P.gold.ni"
  if cmp -s "out/$P.storm.nf" "out/$P.gold.ni"; then
    echo "STORM $P: program trace identical under IRQ storm"
  else
    echo "STORM $P: TRACE DIVERGED"; diff "out/$P.gold.ni" "out/$P.storm.nf" | head -4; RES=1
  fi
}
storm bst 997
storm test_64bit 463 -testplusarg "LAT=1" -testplusarg "RAND=9"

[ $RES = 0 ] && echo "M5C: PASS" || echo "M5C: FAIL"
exit $RES

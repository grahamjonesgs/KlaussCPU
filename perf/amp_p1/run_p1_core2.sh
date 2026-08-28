#!/usr/bin/env bash
# AMP P1 sim gate: assemble hello_core2.kla, run it on core2_subsys in xsim,
# check the log FIFO output byte-for-byte.  See AMP_CORE2_PLAN.md P1.
set -euo pipefail
cd "$(dirname "$0")"
K=/home/graham/Documents/src/klausscc/target/release/klausscc
SRC=$(cd ../../KlaussCPU.srcs && pwd)
export PATH=$PATH:/opt/Xilinx/2025.2/Vivado/bin
mkdir -p out

$K --input hello_core2.kla -b hello_core2.bin -o hello_core2.txt > out/asm.log 2>&1
python3 code2mem.py hello_core2.txt.code hello_core2.mem > /dev/null
NDW=$(wc -l < hello_core2.mem)
$K --input ddr_sum.kla -b ddr_sum.bin -o ddr_sum.txt > out/asm2.log 2>&1
python3 code2mem.py ddr_sum.txt.code ddr_sum.mem > /dev/null
NDW2=$(wc -l < ddr_sum.mem)
python3 gen_p2b.py 2000 > out/genp2b_sim.log 2>&1
$K --input loop_ddr.kla -o loop_ddr.txt > out/asm3.log 2>&1
python3 code2mem.py loop_ddr.txt.code out/sim_loop_ddr.mem > /dev/null
NDW3=$(wc -l < out/sim_loop_ddr.mem)
$K --input eth_probe.kla -o eth_probe.txt > out/asm4.log 2>&1
python3 code2mem.py eth_probe.txt.code eth_probe.mem > /dev/null
NDW4=$(wc -l < eth_probe.mem)

xvlog -sv "$SRC/sources_1/new/klauss_pkg.sv" \
          "$SRC/sources_1/new/mmio_if.sv" \
          "$SRC/sources_1/new/pipeline_core.sv" \
          "$SRC/sources_1/new/core2_subsys.sv" \
          "$SRC/sim_1/new/tb_core2.sv" > out/xvlog.log 2>&1 \
  || { tail -30 out/xvlog.log; exit 1; }
xelab tb_core2 -s tc2 > out/xelab.log 2>&1 \
  || { tail -30 out/xelab.log; exit 1; }
xsim tc2 -runall \
  -testplusarg "IMAGE=$PWD/hello_core2.mem" \
  -testplusarg "NDW=$NDW" \
  -testplusarg "IMAGE2=$PWD/ddr_sum.mem" \
  -testplusarg "NDW2=$NDW2" \
  -testplusarg "IMAGE3=$PWD/out/sim_loop_ddr.mem" \
  -testplusarg "NDW3=$NDW3" \
  -testplusarg "IMAGE4=$PWD/eth_probe.mem" \
  -testplusarg "NDW4=$NDW4" \
  2>&1 | tee out/xsim.log | grep -E "TB_CORE2|Fatal|Error"
grep -q "ALL PHASES PASS" out/xsim.log && echo "AMP P1+P2a SIM: PASS" || { echo "AMP P1+P2a SIM: FAIL"; exit 1; }

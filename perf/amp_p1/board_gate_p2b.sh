#!/usr/bin/env bash
# AMP P2b board gate: regenerate the 1M-iteration fetch-rate probes, program
# the FPGA, re-run the P2a coherency gate (regression), then measure the
# core-2 loop wall time: local BRAM (L=) vs DDR text window through the read
# cache (D=).  Target: D within ~2x of L (uncached would be ~10-15x).
set -euo pipefail
cd "$(dirname "$0")"
K=/home/graham/Documents/src/klausscc/target/release/klausscc
mkdir -p out

python3 gen_p2b.py 1000000 > out/genp2b.log 2>&1
$K --input loop_local.kla -o loop_local.txt > out/asmL.log 2>&1
python3 code2mem.py loop_local.txt.code loop_local.mem > /dev/null
$K --input loop_ddr.kla -o loop_ddr.txt > out/asmD.log 2>&1
python3 code2mem.py loop_ddr.txt.code loop_ddr.mem > /dev/null
python3 fill_p2b.py
$K --input core1_p2btest.kla -o core1_p2btest.txt > out/asmT.log 2>&1

/opt/Xilinx/2025.2/Vivado/bin/vivado -mode batch \
  -source /home/graham/.klausscpu_scratch/prog_p1_core2.tcl \
  -journal out/prog3.jou -log out/prog3.log > /dev/null 2>&1
grep -q "JTAG_PROGRAM: DONE" out/prog3.log && echo "FPGA programmed (volatile)"
sleep 3

echo "=== P2a coherency regression ==="
timeout 120 $K --input core1_p2test.kla --serial /dev/ttyUSB1 --monitor \
  > out/p2regr.log 2>&1 || true
grep -aq "P2 PASS" out/p2regr.log && echo "P2a regression: PASS" \
  || { echo "P2a regression: FAIL"; tail -20 out/p2regr.log; exit 1; }

echo "=== fetch-rate probe (1M-iteration loop) ==="
timeout 300 $K --input core1_p2btest.kla --serial /dev/ttyUSB1 --monitor \
  > out/p2b.log 2>&1 || true
grep -aE "^L=|^D=|P2B DONE" out/p2b.log || { tail -20 out/p2b.log; exit 1; }
echo "AMP P2b BOARD: DONE (L=local BRAM ms, D=cached DDR ms, hex)"

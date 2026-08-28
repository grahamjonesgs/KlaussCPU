#!/usr/bin/env bash
# AMP P1 board gate: JTAG-program the new bitstream, run the core-1 loader,
# expect core-2's "hello from core 2" forwarded over the real UART + P1 PASS.
set -euo pipefail
cd "$(dirname "$0")"
K=/home/graham/Documents/src/klausscc/target/release/klausscc
mkdir -p out

/opt/Xilinx/2025.2/Vivado/bin/vivado -mode batch \
  -source /home/graham/.klausscpu_scratch/prog_p1_core2.tcl \
  -journal out/prog.jou -log out/prog.log > /dev/null 2>&1
grep -q "JTAG_PROGRAM: DONE" out/prog.log && echo "FPGA programmed (volatile)"
sleep 3

timeout 90 $K --input core1_p1test.kla --serial /dev/ttyUSB1 --monitor \
  > out/board.log 2>&1 || true
if grep -aq "hello from core 2" out/board.log && grep -aq "P1 PASS" out/board.log; then
  echo "AMP P1 BOARD: PASS"
  grep -a "hello from core 2\|P1 PASS" out/board.log
else
  echo "AMP P1 BOARD: FAIL — last UART output:"
  tail -25 out/board.log
  exit 1
fi

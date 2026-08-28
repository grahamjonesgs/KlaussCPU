#!/usr/bin/env bash
# AMP P2a board gate:
#  1. JTAG-program the new bitstream.
#  2. core1_p2test: hello image -> DDR (cached stores + FLUSH) -> core 2
#     executes it from DDR; data buffer -> FLUSH -> core 2 sums + writes the
#     result to DDR -> core 1 INVALIDATEs + reads back + compares.
#  3. CPI non-intrusion A/B: perf_haz with core 2 idle, then with core 2
#     spinning on uncached DDR reads (worst-case master-C pressure).
set -euo pipefail
cd "$(dirname "$0")"
K=/home/graham/Documents/src/klausscc/target/release/klausscc
HAZ=/media/psf/src/klausscpu-runtime/baremetal/perf_haz.elf
mkdir -p out

/opt/Xilinx/2025.2/Vivado/bin/vivado -mode batch \
  -source /home/graham/.klausscpu_scratch/prog_p1_core2.tcl \
  -journal out/prog2.jou -log out/prog2.log > /dev/null 2>&1
grep -q "JTAG_PROGRAM: DONE" out/prog2.log && echo "FPGA programmed (volatile)"
sleep 3

echo "=== coherency gate (core1_p2test) ==="
timeout 120 $K --input core1_p2test.kla --serial /dev/ttyUSB1 --monitor \
  > out/p2board.log 2>&1 || true
if grep -aq "hello from core 2" out/p2board.log && \
   grep -aq "^sum" out/p2board.log && \
   grep -aq "P2 PASS" out/p2board.log; then
  echo "AMP P2a COHERENCY: PASS"
else
  echo "AMP P2a COHERENCY: FAIL — last UART:"
  tail -25 out/p2board.log
  exit 1
fi

echo "=== CPI baseline (core 2 idle) ==="
timeout 300 $K --input "$HAZ" --serial /dev/ttyUSB1 --monitor \
  > out/haz_idle.log 2>&1 || true
grep -a "^HAZ" out/haz_idle.log | tee out/haz_idle.csv || true

echo "=== start core-2 DDR spinner ==="
timeout 60 $K --input core1_spinload.kla --serial /dev/ttyUSB1 --monitor \
  > out/spinload.log 2>&1 || true
grep -aq "core2 spinning" out/spinload.log && echo "core 2 spinning on DDR"

echo "=== CPI with core 2 spinning ==="
timeout 300 $K --input "$HAZ" --serial /dev/ttyUSB1 --monitor \
  > out/haz_spin.log 2>&1 || true
grep -a "^HAZ" out/haz_spin.log | tee out/haz_spin.csv || true

echo "=== A/B (idle vs spinning) ==="
paste -d'  ' out/haz_idle.csv out/haz_spin.csv || true
echo "AMP P2a BOARD: DONE"

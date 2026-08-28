#!/usr/bin/env bash
# AMP P4a board gate: VNC served BY CORE 2.
#  1. JTAG-program the P4 bitstream (128 KB core-2 BRAM).
#  2. Serial-load core1_amp_vnc.elf: core 1 boots core 2 (lwIP + VNC),
#     animates a 320x200 framebuffer in shared DDR, posts frames.
#  3. Wait for "core2 vnc: listening", then vnc_probe.py from this VM:
#     hextile+raw offered, then raw-only — fps / bytes-per-update, which is
#     the A/B against the 2026-08-24 single-core numbers (doom ~3 fps).
set -uo pipefail
cd "$(dirname "$0")"
K=/home/graham/Documents/src/klausscc/target/release/klausscc
# Serial node can re-enumerate after a USB drop: take the last ttyUSB present.
SER=$(ls /dev/ttyUSB* 2>/dev/null | tail -1); [ -n "$SER" ] || { echo "no /dev/ttyUSB* — board USB not attached"; exit 1; }
BM=/media/psf/src/klausscpu-runtime/baremetal
PROBE=$PWD/vnc_probe.py
mkdir -p out

/opt/Xilinx/2025.2/Vivado/bin/vivado -mode batch \
  -source /home/graham/.klausscpu_scratch/prog_p1_core2.tcl \
  -journal out/prog5.jou -log out/prog5.log > /dev/null 2>&1
grep -q "JTAG_PROGRAM: DONE" out/prog5.log && echo "FPGA programmed (volatile)"
sleep 5

timeout 400 $K --input "$BM/core1_amp_vnc.elf" --serial "$SER" --monitor \
  > out/p4_host.log 2>&1 &
HOST=$!
for i in $(seq 1 90); do
  grep -aq "core2 vnc: listening" out/p4_host.log && break
  sleep 1
done
IP=$(grep -a "core2: IP" out/p4_host.log | head -1 | sed 's/.*IP //' | tr -d '\r')
if [ -z "$IP" ] || ! grep -aq "core2 vnc: listening" out/p4_host.log; then
  echo "core 2 VNC did not come up — UART so far:"; tail -25 out/p4_host.log
  kill $HOST 2>/dev/null; exit 1
fi
echo "core 2 VNC listening at $IP"
sleep 2
python3 "$PROBE" "$IP" --seconds 30 --label c2-hextile 2>&1 | tail -3
python3 "$PROBE" "$IP" --raw-only --seconds 30 --label c2-raw 2>&1 | tail -3
sleep 6
kill $HOST 2>/dev/null; wait $HOST 2>/dev/null
echo "--- board console (last 25 lines) ---"; tail -25 out/p4_host.log
echo "AMP P4a BOARD: DONE"

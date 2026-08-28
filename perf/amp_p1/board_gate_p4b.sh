#!/usr/bin/env bash
# AMP P4b board gate: DOOM with VNC served by core 2.
#   serial-load the Zephyr doom AMP image (build_doom_amp), wait for core 2's
#   "core2 vnc: listening" (forwarded as [core2] lines), then probe from here:
#   hextile+raw and raw-only, 30 s each, while capturing doom's own
#   "doom: drawfps=.. tick=..ms" profile lines — the A/B against 2026-08-24
#   (single core: ~3 fps at the client, tick 175→323 ms under send preemption).
set -uo pipefail
cd "$(dirname "$0")"
exec > >(tee out/p4b_gate.stdout) 2>&1   # keep the gate's own output for later analysis
K=/home/graham/Documents/src/klausscc/target/release/klausscc
# Serial node can re-enumerate after a USB drop: take the last ttyUSB present.
SER=$(ls /dev/ttyUSB* 2>/dev/null | tail -1); [ -n "$SER" ] || { echo "no /dev/ttyUSB* — board USB not attached"; exit 1; }
ELF=${1:-/media/psf/src/klausscpu-runtime/zephyr-ws/build_doom_amp/zephyr/zephyr.elf}
mkdir -p out
[ -f "$ELF" ] || { echo "no $ELF"; exit 1; }

# Clean slate: JTAG-reprogram first.  A handover from a RUNNING core 2 (the
# previous session still streaming) can leave LiteEth in a state eth_init
# doesn't recover — core 2 then hangs in DHCP.  Reprogramming resets both
# cores, the mailbox and the MAC.
/opt/Xilinx/2025.2/Vivado/bin/vivado -mode batch \
  -source /home/graham/.klausscpu_scratch/prog_p1_core2.tcl \
  -journal out/prog6.jou -log out/prog6.log > /dev/null 2>&1
grep -q "JTAG_PROGRAM: DONE" out/prog6.log && echo "FPGA programmed (clean slate)"
sleep 5

timeout 600 $K --input "$ELF" --serial "$SER" --monitor > out/p4b_doom.log 2>&1 &
HOST=$!
# Anchor on THIS session's core-2 boot: the log FIFO can hold a stale
# backlog from the previous program (incl. an old "listening" line), so wait
# for the fresh "lwIP bring-up" first, then "listening" AFTER it.
for i in $(seq 1 150); do
  B=$(grep -an "lwIP bring-up" out/p4b_doom.log | tail -1 | cut -d: -f1)
  if [ -n "$B" ] && tail -n +$B out/p4b_doom.log | grep -aq "core2 vnc: listening"; then break; fi
  sleep 1
done
B=$(grep -an "lwIP bring-up" out/p4b_doom.log | tail -1 | cut -d: -f1)
{ [ -n "$B" ] && tail -n +$B out/p4b_doom.log | grep -aq "core2 vnc: listening"; } \
  || { echo "core 2 VNC not up — UART:"; tail -30 out/p4b_doom.log; kill $HOST; exit 1; }
IP=$(grep -a "core2: IP" out/p4b_doom.log | head -1 | sed 's/.*IP //' | tr -d '\r')
echo "core 2 VNC at $IP; letting doom reach steady state (title -> demo)..."
sleep 25
echo "=== doom tick WITHOUT a client (baseline for this build) ==="
grep -a "doom: drawfps" out/p4b_doom.log | tail -2 | tr -d '\r'
python3 vnc_probe.py "$IP" --seconds 30 --label doom-c2-hextile 2>&1 | tail -1
echo "=== doom tick DURING hextile streaming ==="
grep -a "doom: drawfps" out/p4b_doom.log | tail -3 | tr -d '\r'
python3 vnc_probe.py "$IP" --raw-only --seconds 30 --label doom-c2-raw 2>&1 | tail -1
echo "=== doom tick DURING raw streaming ==="
grep -a "doom: drawfps" out/p4b_doom.log | tail -3 | tr -d '\r'
sleep 3
echo "--- core-2 lines ---"; grep -a "\[core2\]" out/p4b_doom.log | tail -6 | tr -d '\r'
kill $HOST 2>/dev/null; wait $HOST 2>/dev/null
echo "AMP P4b BOARD: DONE"

#!/usr/bin/env bash
# run_probe_soc.sh — assemble the M7 hazard probe (.kla), boot it through the REAL
# boot-ROM -> DDR -> loader -> PIPE_RUN chain in tb_soc (behavioral DDR), and print
# the per-kernel hazard-counter dump.  This is a PLUMBING check (does the pipeline
# read 0xF00D_00B0..E8 correctly + halt clean) — magnitudes are the behavioral-DDR
# model's (fixed 8-cycle latency), NOT silicon; real numbers come from the board.
#
#   ./run_probe_soc.sh [profile]     profile = sim (default) | board
set -euo pipefail
cd "$(dirname "$0")"
PROF=${1:-sim}
KLA=haz_probe_${PROF}.kla
K=/home/graham/Documents/src/klausscc/target/release/klausscc
SRC=$(cd ../../KlaussCPU.srcs && pwd)
export PATH=$PATH:/opt/Xilinx/2025.2/Vivado/bin
mkdir -p out soc_run

# --- assemble .kla -> .kbt -> netboot.mem -----------------------------------
python3 gen_probe.py "$PROF" > "$KLA"
$K --input "$KLA" -b "haz_probe_${PROF}.kbt" >/dev/null 2>&1
# klausscc appends .kbt
python3 kbt2mem.py "haz_probe_${PROF}.kbt.kbt" "soc_run/netboot.mem"
echo "netboot.mem: $(wc -l < soc_run/netboot.mem) doublewords"

cd soc_run
xvlog -sv \
  "$SRC/sources_1/new/klauss_pkg.sv" \
  "$SRC/sources_1/new/membus_if.sv" \
  "$SRC/sources_1/new/mmio_if.sv" \
  "$SRC/sources_1/new/pipeline_core.sv" \
  "$SRC/sources_1/new/bus_splitter.sv" \
  "$SRC/sources_1/new/mem_read_write.sv" \
  "$SRC/sources_1/new/boot_rom.sv" \
  "$SRC/sources_1/new/uart_send_msg.sv" \
  "$SRC/sources_1/new/uart_rx.sv" \
  "$SRC/sources_1/new/uart_rx_fifo.sv" \
  "$SRC/sources_1/new/sd_spi.sv" \
  "$SRC/sources_1/new/crypto_aes.sv" \
  "$SRC/sources_1/new/crypto_sha.sv" \
  "$SRC/sources_1/new/aes_core.sv" \
  "$SRC/sources_1/new/aes_sbox.sv" \
  "$SRC/sources_1/new/ghash.sv" \
  "$SRC/sources_1/new/sha256_core.sv" \
  "$SRC/sources_1/new/trng.sv" \
  "$SRC/sources_1/new/blitter_dma.sv" \
  "$SRC/sources_1/new/eth_mmio_bridge.sv" \
  "$SRC/sources_1/new/Seven_seg_LED_Display_Controller.sv" \
  "$SRC/sources_1/new/SPI_Master.sv" \
  "$SRC/sources_1/new/SPI_Master_With_Single_CS.sv" \
  "$SRC/sources_1/new/RGB_LED.sv" \
  "$SRC/sources_1/new/uart_tx.sv" \
  "$SRC/sources_1/new/KlaussCPU.sv" \
  "$SRC/sim_1/new/tb_soc.sv" \
  -i "$SRC/sources_1/new" > xvlog.log 2>&1 || { grep -i ERROR xvlog.log | head; exit 1; }
xvlog "/opt/Xilinx/2025.2/Vivado/data/verilog/src/glbl.v" >> xvlog.log 2>&1 || true
xelab tb_soc glbl -s tsoc -L unisims_ver --relax > xelab.log 2>&1 \
  || { grep -iE "ERROR" xelab.log | head; exit 1; }
xsim tsoc -runall \
  -testplusarg "TRACE=../out/probe_${PROF}.soc.trace" \
  -testplusarg "UARTF=../out/probe_${PROF}.soc.uart" \
  | tee "../out/probe_${PROF}.soc.log" | grep -E "TB_SOC" || true
cd ..

echo "==================== UART capture ===================="
cat "out/probe_${PROF}.soc.uart" || true
echo "==================== parsed ==========================="
python3 parse_haz.py "out/probe_${PROF}.soc.uart" --csv "out/probe_${PROF}.soc.csv" || true

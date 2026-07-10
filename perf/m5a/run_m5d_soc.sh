#!/usr/bin/env bash
# M5d full-SoC smoke: boot <prog> through the REAL boot-ROM -> DDR copy ->
# loader -> PIPE_RUN chain (behavioral DDR + clk_wiz only), then diff the
# retire trace and UART against the emulator goldens.
#   ./run_m5d_soc.sh <prog> [maxi]
# Trace normalization vs the standalone gate: f= excluded (mul/div flag
# model), i= kept, and the DATA field of sub-word (be != ff) stores is
# stripped on both sides (tb_soc emits raw, golden shows merged; addr+be
# still compared — full data compared on 64-bit + MMIO stores).
set -euo pipefail
cd "$(dirname "$0")"
PROG=${1:?usage: run_m5d_soc.sh <prog> [maxi]}
MAXI=${2:-0}
K=/home/graham/Documents/src/klausscc/target/release/klausscc
E=/media/psf/src/klausscpu-runtime/baremetal
SRC=$(cd ../../KlaussCPU.srcs && pwd)
export PATH=$PATH:/opt/Xilinx/2025.2/Vivado/bin
mkdir -p out soc_run
[ -f "$PROG.mem" ] || $K --mem-out "$E/$PROG.elf" --mem-file "$PROG.mem" | grep mem-out
[ -f "$PROG.trace" ] || $K --input "$E/$PROG.elf" --emulate --trace "$PROG.trace" > "$PROG.emu.out" 2>&1
[ -f "$PROG.uart.golden" ] || awk '/--- Captured UART output ---/{f=1;next} /--- end UART ---/{f=0} f' "$PROG.emu.out" > "$PROG.uart.golden"

cd soc_run
cp "../$PROG.mem" netboot.mem

# everything the top needs, minus the real ddr2_control/clk_wiz (tb shadows them)
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
  -i "$SRC/sources_1/new" > xvlog.log 2>&1 || { grep ERROR xvlog.log | head -10; exit 1; }
# liteeth_core + ring_osc are STUBBED inside tb_soc.sv (sim-hostile / unused)
xvlog "/opt/Xilinx/2025.2/Vivado/data/verilog/src/glbl.v" >> xvlog.log 2>&1 \
  || { grep ERROR xvlog.log | head -5; exit 1; }
xelab tb_soc glbl -s tsoc -L unisims_ver --relax > xelab.log 2>&1 \
  || { grep -E "ERROR" xelab.log | head -10; exit 1; }
xsim tsoc -runall \
  -testplusarg "TRACE=../out/$PROG.soc.trace" \
  -testplusarg "UARTF=../out/$PROG.soc.uart" \
  -testplusarg "MAXI=$MAXI" \
  | tee "../out/$PROG.soc.log" | grep "TB_SOC"
cd ..

# --- diff -------------------------------------------------------------------
norm () {  # strip f= field; blank the data of sub-word (be != ff) stores
  sed -E -e 's/ f=[01]*//' \
         -e 's#(wr=[0-9a-f]{8}/(f[0-9a-e]|[0-9a-e][0-9a-f])/)[0-9a-f]{16}#\1xx#' "$1"
}
RES=0
norm "$PROG.trace"            > "out/$PROG.soc.gold.n"
norm "out/$PROG.soc.trace"    > "out/$PROG.soc.rtl.n"
if [ "$MAXI" != 0 ]; then head -n "$MAXI" "out/$PROG.soc.gold.n" > "out/$PROG.soc.gold.nc" \
  && mv "out/$PROG.soc.gold.nc" "out/$PROG.soc.gold.n"; fi
if cmp -s "out/$PROG.soc.gold.n" "out/$PROG.soc.rtl.n"; then
  echo "SOC TRACE: IDENTICAL ($(wc -l < "out/$PROG.soc.rtl.n") lines)"
else
  echo "SOC TRACE: DIFFERS — first divergence:"
  diff "out/$PROG.soc.gold.n" "out/$PROG.soc.rtl.n" | head -6
  RES=1
fi
if [ "$MAXI" = 0 ]; then
  iconv -f UTF-8 -t LATIN1 "$PROG.uart.golden" > "out/$PROG.soc.gold.uart" 2>/dev/null \
    || cp "$PROG.uart.golden" "out/$PROG.soc.gold.uart"
  if grep -qF "$(head -c 64 "out/$PROG.soc.gold.uart")" "out/$PROG.soc.uart" \
     && [ "$(tail -c $(wc -c < "out/$PROG.soc.gold.uart") "out/$PROG.soc.uart" | cksum)" = "$(cksum < "out/$PROG.soc.gold.uart")" ]; then
    echo "SOC UART: program byte stream present and identical (after loader banner)"
  else
    echo "SOC UART: DIFFERS (see out/$PROG.soc.uart)"
    RES=1
  fi
fi
[ $RES = 0 ] && echo "M5D SOC $PROG: PASS" || echo "M5D SOC $PROG: FAIL"
exit $RES

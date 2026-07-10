#!/usr/bin/env bash
# M5a golden-trace runner: xsim the full-ISA pipeline_core on a real compiled
# ELF image and diff the RTL trace + UART byte stream against the klausscc
# emulator golden. See PIPELINE_IMPL.md §8.
#
# usage: ./run_m5a.sh <prog> [maxi]
#   prog: hello | bst | expr | test_64bit | queens | <any baremetal elf>
#   maxi: cap instructions (0 = run to halt); golden trace is capped to match
set -euo pipefail
cd "$(dirname "$0")"

PROG=${1:?usage: run_m5a.sh <prog> [maxi]}
MAXI=${2:-0}
K=/home/graham/Documents/src/klausscc/target/release/klausscc
E=/media/psf/src/klausscpu-runtime/baremetal
SRC=$(cd ../../KlaussCPU.srcs && pwd)
export PATH=$PATH:/opt/Xilinx/2025.2/Vivado/bin
mkdir -p out

# --- goldens (regenerate if missing) ---------------------------------------
if [ ! -f "$PROG.mem" ]; then
  $K --mem-out "$E/$PROG.elf" --mem-file "$PROG.mem" | grep mem-out
fi
if [ "$MAXI" = 0 ]; then
  GTRACE=$PROG.trace
  if [ ! -f "$GTRACE" ]; then
    $K --input "$E/$PROG.elf" --emulate --trace "$GTRACE" > "$PROG.emu.out" 2>&1
  fi
else
  GTRACE=${PROG}_cap.trace
  if [ ! -f "$GTRACE" ]; then
    $K --input "$E/$PROG.elf" --emulate --trace "$GTRACE" --max-instructions "$MAXI" > "${PROG}_cap.emu.out" 2>&1
  fi
fi
if [ ! -f "$PROG.uart.golden" ]; then
  [ -f "$PROG.emu.out" ] || $K --input "$E/$PROG.elf" --emulate > "$PROG.emu.out" 2>&1
  awk '/--- Captured UART output ---/{f=1;next} /--- end UART ---/{f=0} f' "$PROG.emu.out" > "$PROG.uart.golden"
fi

# --- compile + run ----------------------------------------------------------
xvlog -sv "$SRC/sources_1/new/klauss_pkg.sv" \
          "$SRC/sources_1/new/pipeline_core.sv" \
          "$SRC/sim_1/new/tb_pipeline_isa.sv" > out/xvlog.log 2>&1 \
  || { tail -30 out/xvlog.log; exit 1; }
xelab tb_pipeline_isa -s tpisa > out/xelab.log 2>&1 \
  || { tail -30 out/xelab.log; exit 1; }
xsim tpisa -runall \
  -testplusarg "IMAGE=$PROG.mem" \
  -testplusarg "TRACE=out/$PROG.rtl.trace" \
  -testplusarg "UARTF=out/$PROG.rtl.uart" \
  -testplusarg "MAXI=$MAXI" \
  | tee "out/$PROG.xsim.log" | grep "TB_M5A"

# --- diff -------------------------------------------------------------------
# The f= field is stripped before comparing: the emulator does not model the
# silicon's mul/div flag writes (FSM: mul Z/S/V, div Z/V; emulator: none), so
# the flag field diverges after mul/div by design. The pipeline implements the
# SILICON semantics (the board A/B vs master is the real oracle). Any
# program-visible flag effect still shows up as a pc/reg divergence.
RES=0
sed 's/ f=[01]*//' "$GTRACE"              > "out/$PROG.gold.nf"
sed 's/ f=[01]*//' "out/$PROG.rtl.trace"  > "out/$PROG.rtl.nf"
if cmp -s "out/$PROG.gold.nf" "out/$PROG.rtl.nf"; then
  echo "TRACE: IDENTICAL ($(wc -l < "$GTRACE") lines, f= field excluded)"
  if cmp -s "$GTRACE" "out/$PROG.rtl.trace"; then
    echo "TRACE: (f= field also identical)"
  else
    echo "TRACE: (f= field differs on $(diff "$GTRACE" "out/$PROG.rtl.trace" | grep -c '^<') lines — expected after mul/div)"
  fi
else
  echo "TRACE: DIFFERS — first divergence (f= excluded):"
  diff "out/$PROG.gold.nf" "out/$PROG.rtl.nf" | head -8
  RES=1
fi
if [ "$MAXI" = 0 ]; then
  # The emulator captures UART via `String::push(b as char)` — bytes >= 0x80
  # are Latin-1->UTF-8 expanded in the golden. Undo that for the byte compare.
  iconv -f UTF-8 -t LATIN1 "$PROG.uart.golden" > "out/$PROG.uart.gold.raw" 2>/dev/null \
    || cp "$PROG.uart.golden" "out/$PROG.uart.gold.raw"
  if cmp -s "out/$PROG.uart.gold.raw" "out/$PROG.rtl.uart"; then
    echo "UART:  IDENTICAL ($(wc -c < "out/$PROG.rtl.uart") bytes)"
  else
    echo "UART:  DIFFERS"
    diff <(xxd "out/$PROG.uart.gold.raw") <(xxd "out/$PROG.rtl.uart") | head -8
    RES=1
  fi
fi
[ $RES = 0 ] && echo "M5A $PROG: PASS" || echo "M5A $PROG: FAIL"
exit $RES

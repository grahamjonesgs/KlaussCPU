#!/usr/bin/env bash
# Debug-hook regression (DEBUG_PLAN.md Phase 5): xsim pipeline_core with
# DEBUG_EN=1 under a scripted debugger (tb_pipeline_isa +DBG=<mode>) and check
# the program's trace + UART against the klausscc emulator golden, i.e. that
# halting, stepping, breakpoints and watchpoints are invisible to the program.
#
# usage: ./run_m5_dbg.sh <prog> <mode> [plusargs...]
#   mode: idle | halts | steps | bp | wp | regmem | attached | irqsteps | jtag
#   idle  = DEBUG_EN=1 with the debug port tied off (no +DBG)
#   jtag  = tb_dm_jtag: JTAG scans -> dm_dtm -> debug_module -> core (entry
#           halt, registers, memory, breakpoint at the golden's 1000th pc,
#           200 steps, run to HALT)
#   bp    = breakpoint at the pc of the golden's 1000th instruction (or +BP=)
#   wp    = watchpoint on the golden's 50th store address (or +WP= / +WPK=)
#   irqsteps needs +IRQ_PERIOD=n; its trace is not compared (IRQ timing)
set -euo pipefail
cd "$(dirname "$0")"

PROG=${1:?usage: run_m5_dbg.sh <prog> <mode> [plusargs]}
MODE=${2:?mode}
shift 2
K=${KLAUSSCC:-/home/graham/Documents/src/klausscc/target/release/klausscc}
E=${E:-$HOME/Documents/src/klausscpu-runtime/baremetal}
SRC=$(cd ../../KlaussCPU.srcs && pwd)
export PATH=$PATH:/opt/xilinx/2025.2/Vivado/bin:/opt/Xilinx/2025.2/Vivado/bin
mkdir -p out

[ -f "$PROG.mem" ] || $K --mem-out "$E/$PROG.elf" --mem-file "$PROG.mem" 2>&1 | grep mem-out
[ -f "$PROG.trace" ] || $K --input "$E/$PROG.elf" --emulate --trace "$PROG.trace" > "$PROG.emu.out" 2>&1
if [ ! -f "$PROG.uart.golden" ]; then
  awk '/--- Captured UART output ---/{f=1;next} /--- end UART ---/{f=0} f' "$PROG.emu.out" > "$PROG.uart.golden"
fi

if [ ! -f out/dbg.stamp ] || [ "$SRC/sources_1/new/pipeline_core.sv" -nt out/dbg.stamp ] \
   || [ "$SRC/sim_1/new/tb_pipeline_isa.sv" -nt out/dbg.stamp ]; then
  xvlog -sv "$SRC/sources_1/new/klauss_pkg.sv" "$SRC/sources_1/new/pipeline_core.sv" \
            "$SRC/sim_1/new/tb_pipeline_isa.sv" > out/xvlog_dbg.log 2>&1 \
    || { tail -30 out/xvlog_dbg.log; exit 1; }
  xelab tb_pipeline_isa -generic_top "DEBUG_EN=1" -s tpisa_dbg > out/xelab_dbg.log 2>&1 \
    || { tail -30 out/xelab_dbg.log; exit 1; }
  touch out/dbg.stamp
fi

if [ "$MODE" = jtag ]; then
  if [ ! -f out/jtag.stamp ] || [ "$SRC/sources_1/new/debug_module.sv" -nt out/jtag.stamp ] \
     || [ "$SRC/sources_1/new/pipeline_core.sv" -nt out/jtag.stamp ] \
     || [ "$SRC/sim_1/new/tb_dm_jtag.sv" -nt out/jtag.stamp ]; then
    xvlog -sv -d DM_NO_BSCAN "$SRC/sources_1/new/klauss_pkg.sv" "$SRC/sources_1/new/pipeline_core.sv" \
              "$SRC/sources_1/new/debug_module.sv" "$SRC/sim_1/new/tb_dm_jtag.sv" > out/xvlog_jtag.log 2>&1 \
      || { tail -30 out/xvlog_jtag.log; exit 1; }
    xelab tb_dm_jtag -s tdmj > out/xelab_jtag.log 2>&1 || { tail -30 out/xelab_jtag.log; exit 1; }
    touch out/jtag.stamp
  fi
fi

EXTRA=()
case "$MODE" in
  idle) ;;
  jtag)
    BP=$(sed -n '1000s/.* pc=\([0-9a-f]*\) .*/\1/p' "$PROG.trace")
    EXTRA=(-testplusarg "BP=$BP") ;;
  bp)
    BP=$(sed -n '1000s/.* pc=\([0-9a-f]*\) .*/\1/p' "$PROG.trace")
    EXTRA=(-testplusarg "DBG=bp" -testplusarg "BP=$BP") ;;
  wp)
    WP=$(grep -o 'wr=[0-9a-f]*' "$PROG.trace" | grep -v 'wr=f' | sed -n '50s/wr=//p')
    EXTRA=(-testplusarg "DBG=wp" -testplusarg "WP=$WP") ;;
  *) EXTRA=(-testplusarg "DBG=$MODE") ;;
esac
for a in "$@"; do EXTRA+=(-testplusarg "$a"); done

TAG="$PROG.$MODE"
SNAP=tpisa_dbg; [ "$MODE" = jtag ] && SNAP=tdmj
xsim $SNAP -runall -testplusarg "IMAGE=$PROG.mem" \
  -testplusarg "TRACE=out/$TAG.rtl.trace" -testplusarg "UARTF=out/$TAG.rtl.uart" \
  "${EXTRA[@]}" | tee "out/$TAG.xsim.log" | grep -E "TB_M5A: stop|TB_DBG" || true

RES=0
grep -q 'TB_DBG: ERROR' "out/$TAG.xsim.log" && RES=1
if grep -q 'TB_DBG: mode=.* errors=[1-9]' "out/$TAG.xsim.log"; then RES=1; fi
if [ "$MODE" != irqsteps ]; then
  if cmp -s <(sed 's/ f=[01]*//' "$PROG.trace") <(sed 's/ f=[01]*//' "out/$TAG.rtl.trace"); then
    echo "TRACE: IDENTICAL ($(wc -l < "$PROG.trace") lines)"
  else
    echo "TRACE: DIFFERS:"; diff <(sed 's/ f=[01]*//' "$PROG.trace") <(sed 's/ f=[01]*//' "out/$TAG.rtl.trace") | head -6
    RES=1
  fi
  iconv -f UTF-8 -t LATIN1 "$PROG.uart.golden" > "out/$TAG.uart.gold" 2>/dev/null || cp "$PROG.uart.golden" "out/$TAG.uart.gold"
  cmp -s "out/$TAG.uart.gold" "out/$TAG.rtl.uart" && echo "UART:  IDENTICAL" || { echo "UART:  DIFFERS"; RES=1; }
fi
case "$MODE" in
  bp)
    WANT=$(grep -c " pc=$BP " "$PROG.trace")
    GOT=$(sed -n 's/.*bp hits=\([0-9]*\).*/\1/p' "out/$TAG.xsim.log")
    [ "$GOT" = "$WANT" ] && echo "BP:    $GOT hits at $BP (= executions in the golden)" \
                         || { echo "BP:    $GOT hits at $BP, golden executes it $WANT times"; RES=1; } ;;
  wp)
    DW=$(printf '%08x' $(( 0x$WP & ~7 )))
    WANT=$(grep -c "wr=$DW" "$PROG.trace" || true)
    GOT=$(sed -n 's/.*wp hits=\([0-9]*\).*/\1/p' "out/$TAG.xsim.log")
    echo "WP:    $GOT halts for $WANT golden stores to $DW (two within one drain merge)"
    { [ "$GOT" -ge 1 ] && [ "$GOT" -le "$WANT" ]; } || RES=1 ;;
esac
[ $RES = 0 ] && echo "M5_DBG $PROG $MODE: PASS" || echo "M5_DBG $PROG $MODE: FAIL"
exit $RES

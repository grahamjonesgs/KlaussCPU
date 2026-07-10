#!/usr/bin/env bash
# M5e on-silicon regression: run each program on the board over UART and
# compare the program's UART byte stream against the emulator golden.
#   ./run_m5e_board.sh [prog ...]     (default: the full regression set)
# Assumes the M5d bitstream is already programmed (prog.tcl).
set -uo pipefail
cd "$(dirname "$0")"
K=/home/graham/Documents/src/klausscc/target/release/klausscc
E=/media/psf/src/klausscpu-runtime/baremetal
PROGS=${@:-"hello bst expr test_64bit queens crypto dhrystone"}
mkdir -p out board
PASS=0; FAIL=0
for P in $PROGS; do
  # golden (program bytes only)
  if [ ! -f "$P.uart.golden" ]; then
    $K --input "$E/$P.elf" --emulate > "$P.emu.out" 2>&1
    awk '/--- Captured UART output ---/{f=1;next} /--- end UART ---/{f=0} f' "$P.emu.out" > "$P.uart.golden"
  fi
  iconv -f UTF-8 -t LATIN1 "$P.uart.golden" > "board/$P.gold.raw" 2>/dev/null || cp "$P.uart.golden" "board/$P.gold.raw"
  # run on the board (monitor exits on the halt break; belt-and-braces
  # timeout). klausscc's monitor writes the received UART stream to STDERR.
  timeout 600 $K --input "$E/$P.elf" --serial /dev/ttyUSB1 --monitor \
      > /dev/null 2> "board/$P.board.raw"
  # program bytes = between the loader's "Load Complete OK" banner and the
  # monitor's "CPU halted." status line
  perl -0777 -ne 'if (/Load Complete OK[\r\n]+(.*?)\r?\n?CPU halted\./s) { print $1 }' \
      "board/$P.board.raw" > "board/$P.board.prog"
  GB=$(wc -c < "board/$P.gold.raw")
  head -c "$GB" "board/$P.board.prog" > "board/$P.board.cut"
  if cmp -s "board/$P.board.cut" "board/$P.gold.raw"; then
    echo "BOARD $P: UART IDENTICAL ($GB bytes)"
    PASS=$((PASS+1))
  else
    echo "BOARD $P: UART DIFFERS"
    diff <(xxd "board/$P.gold.raw" | head -6) <(xxd "board/$P.board.cut" | head -6) | head -10
    FAIL=$((FAIL+1))
  fi
done
echo "M5E BOARD: $PASS pass, $FAIL fail"
[ $FAIL = 0 ]

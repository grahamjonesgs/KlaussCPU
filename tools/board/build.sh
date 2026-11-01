#!/usr/bin/env bash
# Build a tools/board/*.c test against the klausscpu-runtime objects.
#   tools/board/build.sh <name> [OPT]     -> tools/board/out/<name>.elf
# (SRC=<file stem> DEFS=-D... build a variant: SRC=hwprop_test DEFS=-DKERN_ONLY tools/board/build.sh hwprop_kern)
# Needs KLAUSSCPU_LLVM_BIN (default ~/Documents/src/klausscpu-llvm/build/bin)
# and RT (default ~/Documents/src/klausscpu-runtime) with its baremetal
# objects built (make -C $RT/baremetal hello.elf).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
N=${1:?name}; OPT=${2:--O1}; SRC=${SRC:-$N}; DEFS=${DEFS:-}
B=${KLAUSSCPU_LLVM_BIN:-$HOME/Documents/src/klausscpu-llvm/build/bin}
RT=${RT:-$HOME/Documents/src/klausscpu-runtime}
BM=$RT/baremetal
mkdir -p "$HERE/out"
python3 "$HERE/gen_hwprop.py" > /dev/null
"$B/clang" -target klausscpu-unknown-elf $OPT -isystem "$RT/picolibc-install/include" -I"$RT" -I"$HERE" \
  -nostdlib -ffunction-sections -fdata-sections -fPIC -D__IEEE_LITTLE_ENDIAN -D_LDBL_EQ_DBL \
  $DEFS -c -o "$HERE/out/$N.o" "$HERE/$SRC.c"
"$B/ld.lld" -T "$RT/klausscpu.ld" --gc-sections -o "$HERE/out/$N.elf" \
  "$BM/crt0.o" "$BM/uart_stubs.o" "$BM/syscalls.o" "$BM/stdio_handles.o" "$BM/setjmp.o" "$BM/atomic.o" \
  "$HERE/out/$N.o" "$RT/picolibc-install/lib/libc.a"
echo "$HERE/out/$N.elf"

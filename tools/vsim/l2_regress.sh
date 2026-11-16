#!/bin/bash
# HW_PERF_PLAN.md §6 (review 1): the L2 regression.
#
#   tools/vsim/l2_regress.sh [section ...]     (default: every section)
#
# Sections (each prints "PASS <name>" / "FAIL <name>" per command; logs in
# $VSIM_WORK/l2reg/<name>.log; sections can run in parallel with separate
# VSIM_WORK directories):
#   tbl2     tb_l2 at 2 / 4 / 8 ways under four MIG timings, every directed
#            test and 12 random seeds x 3,000 transactions
#   mut      l2mut.py: the coherence mutations (each must be killed)
#   soc36    the SoC tests at the default MIG latency (+MIGLAT=36)
#   soc8     the same at +MIGLAT=8 (the old tb_soc's 8-cycle DDR: its race
#            windows), the pre-L2 tests only
#   socvar   the SoC tests with the L2 off, with 8 ways, and under jittered /
#            slow MIG timings
#   eq       the L2_EN = 0 equivalence: VSIM_L2=0 against $BASE under the
#            same tb_soc (identical traces and PIPE_RUN cycles)
#   core     the core regression (5 configurations), diff / excdiff /
#            mpudiff / tlbdiff / tlbpage / dcbzdiff / dmtest, btfuzz
# $BASE: the master revision the L2 branch is compared against (d7f3994).
set -u
cd "$(dirname "$0")/../.."
BASE=${BASE:-d7f3994}
W=${VSIM_WORK:-/tmp/vsim}
L=$W/l2reg
mkdir -p "$L"
R="python3 tools/vsim/run.py"
NFAIL=0

t() {   # t <name> <env...> -- <command...>
   local name=$1; shift
   local envs=()
   while [ "$1" != "--" ]; do envs+=("$1"); shift; done
   shift
   if env "${envs[@]}" "$@" > "$L/$name.log" 2>&1; then
      echo "PASS $name   $(grep -E 'differing|failing|killed|SWEEP|pass |Consistency' "$L/$name.log" | tail -n 2 | tr '\n' ' ' | cut -c1-200)"
   else
      echo "FAIL $name   (see $L/$name.log) $(tail -n 3 "$L/$name.log" | tr '\n' ' ' | cut -c1-200)"
      NFAIL=$((NFAIL + 1))
   fi
}

soc_set() {   # soc_set <tag> <env...>: the pre-L2 SoC tests
   local tag=$1; shift
   t socdiff_$tag "$@" -- $R socdiff --base "$BASE" --seeds 30
   t socdiff64_$tag "$@" -- $R socdiff --base "$BASE" --seeds 20 --span 64
   t socmmio_$tag "$@" -- $R socmmio --base "$BASE"
   t socwalk_$tag "$@" -- $R socwalk
   t socx_$tag "$@" -- $R socx
   t socdcbz_$tag "$@" -- $R socdcbz
}

sec_tbl2() { t tbl2_sweep VSIM_JOBS=${VSIM_JOBS:-12} -- python3 tools/vsim/l2mut.py --sweep --seeds 12 --n 3000; }
sec_mut()  { t l2mut VSIM_JOBS=${VSIM_JOBS:-12} -- python3 tools/vsim/l2mut.py; }
sec_soc36() {
   soc_set m36 X=1
   t socl2_m36 X=1 -- $R socl2
   t socsd_m36 X=1 -- $R socsd
   t soceth_m36 X=1 -- $R soceth
   t socvga_m36 X=1 -- $R socvga
   t socframe_m36 X=1 -- $R socframe          # HW_PERF_PLAN §8 item 1
   t socorphan_m36 X=1 -- $R socorphan        # §8 item 3d
}
sec_soc8() {
   soc_set m8 VSIM_SOC_PLUS=+MIGLAT=8
   t socl2_m8 VSIM_SOC_PLUS=+MIGLAT=8 -- $R socl2
}
sec_socvar() {
   soc_set nol2 VSIM_L2=0
   t socl2_nol2 VSIM_L2=0 -- $R socl2
   t socorphan_nol2 VSIM_L2=0 -- $R socorphan
   soc_set w8 VSIM_L2_WAYS=8
   t socl2_w8 VSIM_L2_WAYS=8 -- $R socl2
   soc_set jit "VSIM_SOC_PLUS=+MIGLAT=8 +MIGJIT=20 +MIGRDY=30"
   t socl2_jit "VSIM_SOC_PLUS=+MIGLAT=8 +MIGJIT=20 +MIGRDY=30" -- $R socl2
   soc_set slow "VSIM_SOC_PLUS=+MIGLAT=60 +MIGREF=500"
   t socl2_slow "VSIM_SOC_PLUS=+MIGLAT=60 +MIGREF=500" -- $R socl2
}
sec_eq() {
   t eq_l2off VSIM_L2=0 -- $R socdiff --base "$BASE" --base-tb-wt --seeds 30 -v
   t eq_l2off64 VSIM_L2=0 -- $R socdiff --base "$BASE" --base-tb-wt --seeds 20 --span 64 -v
}
sec_core() {
   t test_default X=1 -- $R test
   t test_dbg VSIM_DEBUG_EN=1 -- $R test
   t test_mpu VSIM_MPU_EN=1 -- $R test
   t test_tlb16 VSIM_TLB_N=16 -- $R test
   t test_nowalk VSIM_TLB_WALK=0 -- $R test
   t diff X=1 -- $R diff --base "$BASE" --seeds 40 --lats 1 3 7 --plus +TRACECYC
   t diff_irq X=1 -- $R diff --base "$BASE" --seeds 40 --irq
   t excdiff X=1 -- $R excdiff
   t mpudiff VSIM_MPU_EN=1 -- $R mpudiff
   t tlbdiff X=1 -- $R tlbdiff
   t tlbpage X=1 -- $R tlbpage
   t dcbzdiff X=1 -- $R dcbzdiff --seeds 200
   t dmtest X=1 -- $R dmtest
   t btfuzz X=1 -- $R btfuzz --seeds 300
}

secs=("$@")
[ ${#secs[@]} -eq 0 ] && secs=(tbl2 mut soc36 soc8 socvar eq core)
for s in "${secs[@]}"; do "sec_$s"; done
echo "l2_regress: $NFAIL failing ($*)"
exit $((NFAIL > 0))

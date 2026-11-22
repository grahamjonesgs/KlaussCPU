#!/usr/bin/env python3
"""Mutation checks of the MMU_PLAN.md §13 user single step and watchpoints.

Each mutation is one or more textual changes to an RTL file.  The mutant's
sources (KlaussCPU.srcs/sources_1/new, sim_1/new and tools/) are copied to
$VSIM_WORK/ustepmut/<name> and its suite runs there; a suite entry is
run.py's arguments, optionally prefixed by an environment dict.  A mutation
is killed when its suite fails (a check, an assertion, a deadlock or a
build error).  Every mutation must be killed.

    python3 tools/vsim/ustepmut.py [--only NAME ...] [-j 6] [--list]
"""
import argparse, concurrent.futures as cf, os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'ustepmut')
PC, TOP = 'sources_1/new/pipeline_core.sv', 'sources_1/new/KlaussCPU.sv'

DBG = {'VSIM_DEBUG_EN': '1'}
STEP = [['test', 'ud_*'], ['udbgfuzz', '--seeds', '6', '--mode', 'step', '--irq']]
WATCH = [['test', 'uw_*', 'ud_te_walk'], ['udbgfuzz', '--seeds', '8', '--mode', 'watch']]
BOTH = [['test', 'ud_*', 'uw_*'], ['udbgfuzz', '--seeds', '8']]
DM = [(DBG, ['test', 'ud_debugger'])]

# (name, what it breaks, file, suite, old, new [, old2, new2 ...])
MUTS = [
    # ---------------- arming ----------------
    ('arm_pm', 'an IRET to supervisor mode arms (PM ignored)', PC, STEP,
     "m_rdata[45] &&\n                          m_rdata[44] && ud_evnz;", "m_rdata[45] &&\n                          ud_evnz;"),
    ('arm_evec', 'arming ignores EVEC = 0', PC, STEP,
     "m_rdata[44] && ud_evnz;", "m_rdata[44];"),
    ('arm_ss', 'every IRET to user mode arms (SS ignored)', PC, WATCH,
     "!mem_xsq && m_rdata[45] &&\n", "!mem_xsq &&\n"),
    ('frame_ss', 'the frame never carries SS (no re-arm)', PC, STEP,
     "{18'b0, EN_UDBG && us_arm,", "{18'b0, 1'b0,"),
    ('frame_ss_pos', 'SS pushed at bit 46 instead of 45', PC, STEP,
     "{18'b0, EN_UDBG && us_arm, EN_EXC && umode,", "{17'b0, EN_UDBG && us_arm, 1'b0, EN_EXC && umode,"),
    ('disarm', 'entry does not disarm', PC, STEP,
     "               if (EN_UDBG) begin\n                  us_arm <= 1'b0;  us_lat <= 1'b0;  us_ret <= 1'b0;  us_ent <= 1'b0;  wp_ent <= 1'b0;\n               end\n               ev_pcz              = 1'b1;",
     "               if (EN_UDBG) begin\n                  us_ent <= 1'b0;  wp_ent <= 1'b0;\n               end\n               ev_pcz              = 1'b1;"),
    ('disarm_lat', 'entry disarms but leaves the latched count', PC, STEP,
     "                  us_arm <= 1'b0;  us_lat <= 1'b0;  us_ret <= 1'b0;  us_ent <= 1'b0;  wp_ent <= 1'b0;\n               end\n               ev_pcz",
     "                  us_arm <= 1'b0;  us_ret <= 1'b0;  us_ent <= 1'b0;  wp_ent <= 1'b0;\n               end\n               ev_pcz"),
    # ---------------- the step boundary ----------------
    ('bnd_irq_first', 'an interrupt may be taken between the stepped instruction and STEP', PC, STEP,
     "end else if (EN_UDBG && us_lat && running && !fetch_halt && !irq_active",
     "end else if (EN_UDBG && us_lat && !irq_take && running && !fetch_halt && !irq_active"),
    ('bnd_none', 'no STEP boundary (the step never fires)', PC, STEP,
     "end else if (EN_UDBG && us_lat && running", "end else if (1'b0 && us_lat && running"),
    ('latch_count', 'the latch does not count the stepped instruction', PC, STEP,
     "if (EN_UDBG && (us_arm || (wb_valid && wb_ussa))) us_lat <= 1'b1;", "if (1'b0) us_lat <= 1'b1;"),
    ('abandon', 'a STEP / WATCH boundary is abandoned like an IRQ without a request', PC, BOTH,
     "if (!(EN_EXC && exc_ent) && !(EN_UDBG && (us_ent || wp_ent)) &&", "if (!(EN_EXC && exc_ent) &&"),
    ('xcancel', 'an access fault of the stepped op does not cancel the STEP boundary', PC, STEP,
     "            irq_active <= 1'b0;\n            if (EN_UDBG) us_ent <= 1'b0;   // (§13: a stepped op's fault comes instead)",
     "            irq_active <= 1'b0;"),
    ('mcancel', 'a misaligned stepped op does not cancel the STEP boundary', PC, STEP,
     "                  if (EN_UDBG) us_ent <= 1'b0;   // (§13: a stepped op's fault comes instead)\n", ""),
    ('wk_lat', 'a successful walk keeps the step count', PC, [['test', 'ud_te_walk']],
     "               if (EN_UDBG) us_lat <= 1'b0;\n", ""),
    ('resume0', 'a debugger resume forgets that the stepped op retired', PC, DM,
     "if (EN_UDBG && !start) us_lat <= us_ret;", "if (EN_UDBG && !start) us_lat <= 1'b0;"),
    ('resume1', 'a debugger resume counts the stepped op as done', PC, DM,
     "if (EN_UDBG && !start) us_lat <= us_ret;", "if (EN_UDBG && !start) us_lat <= 1'b1;"),
    ('ret_flag', 'the retirement flag is never set', PC, DM,
     "if (EN_UDBG && us_lat) us_ret <= 1'b1;", ""),
    ('step_eaddr', 'STEP EADDR is the next PC, not the stepped one', PC, STEP,
     "exc_addr_q  <= wp_ent ? wp_addr : id_pc;", "exc_addr_q  <= wp_ent ? wp_addr : pc;"),
    ('step_vec', 'STEP / WATCH vector to TLBVEC / a stale vector', PC, STEP,
     "                        exc_vec_q   <= evec;\n", ""),
    ('sstep_bit', 'ECAUSE[12] SSTEP not set', PC, BOTH,
     "exc_ux_q    <= {us_ent, wp_ent && wp_st,", "exc_ux_q    <= {1'b0, wp_ent && wp_st,"),
    ('cause_prio', 'step + watch reports STEP', PC, [['test', 'uw_cases'], ['udbgfuzz', '--seeds', '8', '--mode', 'watch']],
     "exc_cause_q <= wp_ent ? X_WATCH : X_STEP;", "exc_cause_q <= us_ent ? X_STEP : X_WATCH;"),
    # ---------------- the watch compare (EX) ----------------
    ('w_c3', 'the carry into EA bit 3 ignored', PC, WATCH,
     "ex_tlo3[3], {14'b0, wp_k[n][1:0]}, 2);", "1'b0, {14'b0, wp_k[n][1:0]}, 2);"),
    ('w_c5', 'the carry into EA bit 5 ignored', PC, WATCH,
     "hi   = f_cf27(ex_ta[31:5], ex_tb[31:5], ex_tlo5[5], wp_k[n][28:2]);",
     "hi   = f_cf27(ex_ta[31:5], ex_tb[31:5], 1'b0, wp_k[n][28:2]);"),
    ('w_cf', 'the carry-free compare drops the generate term', PC, WATCH,
     "((a[i-1] & b[i-1]) | ((a[i-1] | b[i-1]) & ~k[i-1])));\n      return ok;\n   endfunction\n   wire [2:0] ex_wea",
     "((a[i-1] | b[i-1]) & ~k[i-1]));\n      return ok;\n   endfunction\n   wire [2:0] ex_wea"),
    ('w_lo', 'EA bits [4:3] not compared', PC, WATCH,
     "(ex_wzl || lo)", "(1'b1 || lo)"),
    ('w_zl', 'DCBZ compares one doubleword only', PC, WATCH,
     "(ex_wzl || lo)", "(lo)"),
    ('w_d1', 'the 2nd doubleword compared against K, not K - 1', PC, WATCH,
     "d1   = ex_ta[31:3] == wp_km1[n];", "d1   = ex_ta[31:3] == wp_k[n];"),
    ('w_km1', 'K - 1 computed as K + 1', PC, WATCH,
     "wp_km1[wb_op[13]] <= wb_value[31:3] - 29'd1;", "wp_km1[wb_op[13]] <= wb_value[31:3] + 29'd1;"),
    ('w_m32', 'MEMGET32 bytes as a 3-byte access', PC, WATCH,
     "m32 = 16'h000F << ex_wea;", "m32 = 16'h0007 << ex_wea;"),
    # (8'h03 << EA[2:0] would be equivalent: a 16-bit access is 2-aligned or traps)
    ('w_m16', 'a 16-bit access compares only its low byte', PC, WATCH,
     "2'd1:    ex_wm0 = 8'h03 << {ex_wea[2:1], 1'b0};", "2'd1:    ex_wm0 = 8'h01 << {ex_wea[2:1], 1'b0};"),
    ('w_m32w', '32-bit access bytes at the wrong half', PC, WATCH,
     "2'd2:    ex_wm0 = ex_wea[2] ? 8'hF0 : 8'h0F;\n            default: ex_wm0 = 8'hFF;\n         endcase\n      end else if (ex_d.uop == U_LR",
     "2'd2:    ex_wm0 = ex_wea[2] ? 8'h0F : 8'hF0;\n            default: ex_wm0 = 8'hFF;\n         endcase\n      end else if (ex_d.uop == U_LR"),
    ('w_lvr', 'LEAVERET compares only its first doubleword', PC, WATCH,
     "         ex_wm1 = 8'hFF;\n", ""),
    ('w_kind', 'R and W swapped', PC, WATCH,
     "kind = (ex_wld && wp_r[n]) || (ex_wst && wp_w[n]);", "kind = (ex_wld && wp_w[n]) || (ex_wst && wp_r[n]);"),
    ('w_call', 'a not-taken CALLcc counts as a store', PC, WATCH,
     "(ex_d.uop == U_CALL && ex_bcvt_t[0]);", "ex_d.uop == U_CALL;"),
    ('w_iret', 'IRET\'s frame read is watched', PC, [['test', 'uw_supervisor', 'uw_cases'], ['udbgfuzz', '--seeds', '8', '--mode', 'watch']],
     "ex_d.uop == U_LEAVE || ex_d.uop == U_LEAVERET || ex_d.uop == U_LR;     // (not IRET)",
     "ex_d.uop == U_LEAVE || ex_d.uop == U_LEAVERET || ex_d.uop == U_LR || ex_d.uop == U_IRET;"),
    ('w_mode', 'the mode rule ignored (every access qualifies)', PC, WATCH,
     "mode = umode || (!wp_u[n] && !ex_dis_eff);", "mode = 1'b1;"),
    ('w_u', 'U ignored', PC, WATCH,
     "mode = umode || (!wp_u[n] && !ex_dis_eff);", "mode = umode || !ex_dis_eff;"),
    ('w_di', 'the committed DI instead of the program-order one', PC, WATCH,
     "mode = umode || (!wp_u[n] && !ex_dis_eff);", "mode = umode || (!wp_u[n] && !irq_dis);"),
    ('w_sc', 'a failing SC hits', PC, WATCH,
     "wire ex_whit = (ex_wh != 2'b00) && !(ex_d.uop == U_SC && !ex_sc_ok);", "wire ex_whit = (ex_wh != 2'b00);"),
    ('w_misal', 'a misaligned access hits', PC, WATCH,
     "wire ex_wgate = EN_UDBG && ex_valid && ud_go && !ex_misal &&", "wire ex_wgate = EN_UDBG && ex_valid && ud_go &&"),
    ('w_go', 'hits with EVEC = 0 / the debugger attached', PC, [['test', 'uw_supervisor']] + DM,
     "ud_go   <= EN_UDBG && evec != 32'b0 && !(DEBUG_EN && dbg_i.attached);", "ud_go   <= EN_UDBG;"),
    ('w_att', 'hits with the debugger attached', PC, DM,
     "ud_go   <= EN_UDBG && evec != 32'b0 && !(DEBUG_EN && dbg_i.attached);", "ud_go   <= EN_UDBG && evec != 32'b0;"),
    # ---------------- MEM: the squash and the boundary ----------------
    ('m_xsq', 'a faulting op also takes WATCH', PC, WATCH,
     "wire mem_wfire    = EN_UDBG && mem_valid && mem_wph && !mem_xsq && !sq_spec && !wb_wait;",
     "wire mem_wfire    = EN_UDBG && mem_valid && mem_wph && !sq_spec && !wb_wait;"),
    ('m_pc_succ', 'the successor is the fall-through (a CALL\'s target lost)', PC, WATCH,
     "pc      <= (mem_retq || mem_lvrq) ? mem_pred_tgt : mem_succ;", "pc      <= (mem_retq || mem_lvrq) ? mem_pred_tgt : mem_pc_next;"),
    # (a predicted return CAN fire behind younger ops: udbgfuzz seed 54, a
    # LEAVERET at +LAT=1, found when an assertion claimed otherwise)
    ('m_pc_ret', 'a predicted return does not point fetch at its target', PC,
     WATCH + [['udbgfuzz', '--seed0', '54', '--seeds', '1', '--mode', 'watch']],
     "if (!(mem_retq || mem_lvrq) || mem_pred) begin\n               ev_pcz  = 1'b1;",
     "if (!(mem_retq || mem_lvrq)) begin\n               ev_pcz  = 1'b1;"),
    # (not mutated: 'start clears the WPs' — core 1 is reset through every
    # program load, so start never comes alone)
    ('m_memv', 'the EX op entering MEM on the fire edge survives', PC, WATCH,
     "            if (!mem_busy) mem_valid <= 1'b0;   // (the EX op entering MEM)\n", ""),
    ('m_exv', 'the EX op survives the fire', PC, WATCH,
     "            if (!mem_busy) mem_valid <= 1'b0;   // (the EX op entering MEM)\n            ex_valid   <= 1'b0;\n",
     "            if (!mem_busy) mem_valid <= 1'b0;   // (the EX op entering MEM)\n"),
    ('m_irq', 'no boundary taken at the fire (fetch continues)', PC, WATCH,
     "            irq_active <= 1'b1;\n            wp_ent     <= 1'b1;", "            wp_ent     <= 1'b1;"),
    ('m_lcd', 'the LCD strobe not gated by the watch squash', PC, [['test', 'uw_supervisor']],
     "                     !mem_ysq_go && !sq_spec;\n", "                     !mem_xsq_go && !sq_spec;\n"),
    ('m_eaddr2', 'the 2nd-doubleword EADDR is the raw EA', PC, WATCH,
     "wp_addr    <= mem_wp2 ? {mem_ea_raw[31:3] + 29'd1, 3'b000} : mem_ea_raw;", "wp_addr    <= mem_ea_raw;"),
    ('m_wp2', 'the 2nd-doubleword-only test inverted', PC, WATCH,
     "mem_wp2    <= ex_wh0 == 2'b00;", "mem_wp2    <= ex_wh0 != 2'b00;"),
    ('m_bits', 'the WP bits swapped', PC, WATCH,
     "wp_hits    <= mem_wpm;", "wp_hits    <= {mem_wpm[0], mem_wpm[1]};"),
    ('m_st', 'the WRITE bit wrong', PC, WATCH,
     "mem_wps    <= ex_wst;", "mem_wps    <= ex_wld;"),
    # ---------------- SRs, ECAUSE, CPU_FEATURES ----------------
    ('sr_legal', 'MFSR of 0x40-0x43 ILLEGAL', PC, [['test', 'ud_sr']],
     "(EN_UDBG && op[19:14] == 6'b0100_00))) begin   // (§13: 0x40-0x43)\n                  d.legal = 1'b1; d.wreg = 1'b1;",
     "1'b0)) begin   // (§13: 0x40-0x43)\n                  d.legal = 1'b1; d.wreg = 1'b1;"),
    ('sr_rd', 'WPn_CTRL reads U and W swapped', PC, [['test', 'ud_sr']],
     "{53'b0, wp_u[id_op[13]], wp_w[id_op[13]], wp_r[id_op[13]], wp_be[id_op[13]]}",
     "{53'b0, wp_w[id_op[13]], wp_u[id_op[13]], wp_r[id_op[13]], wp_be[id_op[13]]}"),
    ('sr_mux', 'the MFSR leg for 0x40-0x43 missing', PC, [['test', 'ud_sr']],
     "   assign id_srv = (EN_UDBG && id_op[19:16] == 4'h4) ? wp_rdata\n                 : (EN_TLB",
     "   assign id_srv = (EN_TLB"),
    ('ecause_w', 'MTSR ECAUSE keeps 9 bits', PC, [['test', 'ud_sr']],
     "4'd2: ecause <= mtsr_wdata[12:0];", "4'd2: ecause <= {4'b0, mtsr_wdata[8:0]};"),
    ('feat', 'CPU_FEATURES[29:28] not reported', TOP, [['socmmio', '--base', 'master']],
     "| {34'b0, {2{CORE1_EXC_EN && CORE1_UDBG_EN}}, 28'b0};", "| {34'b0, 2'b00, 28'b0};"),
    # ---------------- review of 1813b9c (2026-11-23) ----------------
    # MAJOR: a stepped unpredicted RET / LEAVERET (an empty RAS serializes,
    # fetch_halt holds the STEP boundary off) fires in its first MEM cycle:
    # the entry lost SSTEP (a double step).  Killed by the directed test and,
    # separately, by udbgfuzz (its injected unbalanced returns) alone.
    ('m_sstep', 'a watch fire before the STEP boundary drops SSTEP', PC, [['test', 'uw_ret_unpred']],
     "            if (us_lat) us_ent <= 1'b1;\n", ""),
    ('m_sstep_fz', 'the same, against udbgfuzz alone', PC, [['udbgfuzz', '--seeds', '12', '--mode', 'watch']],
     "            if (us_lat) us_ent <= 1'b1;\n", ""),
    ('dm_um', 'a debugger write of UM = 0 leaves the step armed', PC, [(DBG, ['test', 'ud_dm_um'])],
     "                           if (EN_UDBG && !dbg_i.reg_wdata[0]) begin us_arm <= 1'b0;  us_lat <= 1'b0;  us_ret <= 1'b0; end\n", ""),
    ('m_sqspec', 'mem_wfire ignores the EN_ST1 squash (sq_spec)', PC, [['test', 'uw_sq_spec']],
     "wire mem_wfire    = EN_UDBG && mem_valid && mem_wph && !mem_xsq && !sq_spec && !wb_wait;",
     "wire mem_wfire    = EN_UDBG && mem_valid && mem_wph && !mem_xsq && !wb_wait;"),
    ('dbl_disarm', 'a DOUBLE_FAULT park keeps the step armed', PC, [(DBG, ['test', 'ud_dbl_dm'])],
     "                     if (EN_UDBG) begin\n                        us_arm <= 1'b0;  us_lat <= 1'b0;  us_ret <= 1'b0;  us_ent <= 1'b0;  wp_ent <= 1'b0;\n                     end\n                     park_op    <= 32'b0;",
     "                     park_op    <= 32'b0;"),
    ('m_sup_di', 'a supervisor access hits whatever DI says (udbgfuzz --mode sup)', PC,
     [['udbgfuzz', '--seeds', '8', '--mode', 'sup']],
     "mode = umode || (!wp_u[n] && !ex_dis_eff);", "mode = umode || !wp_u[n];"),
    ('m_sup_u', 'a supervisor access hits a U = 1 watchpoint (udbgfuzz --mode sup)', PC,
     [['udbgfuzz', '--seeds', '8', '--mode', 'sup']],
     "mode = umode || (!wp_u[n] && !ex_dis_eff);", "mode = umode || !ex_dis_eff;"),
    # (the reviewer's further mutations that the suites kill)
    ('x_uxq', 'ECAUSE[12:9] not cleared at a WB exception record', PC, BOTH,
     "               exc_cause_q <= wb_xcause;\n               exc_ux_q    <= '0;\n", "               exc_cause_q <= wb_xcause;\n"),
    ('x_busy', 'the fire kills MEM even when it is busy', PC, WATCH,
     "            if (!mem_busy) mem_valid <= 1'b0;   // (the EX op entering MEM)\n",
     "            mem_valid <= 1'b0;   // (the EX op entering MEM)\n"),
    ('x_idv', 'the fire keeps the ID op', PC, WATCH,
     "            ex_valid   <= 1'b0;\n            id_valid   <= 1'b0;\n            dly_on     <= 1'b0;\n            div_ph     <= MD_IDLE;\n            if (!(mem_retq",
     "            ex_valid   <= 1'b0;\n            dly_on     <= 1'b0;\n            div_ph     <= MD_IDLE;\n            if (!(mem_retq"),
]


def stage(name):
    d = os.path.join(WORK, name)
    shutil.rmtree(d, ignore_errors=True)
    for sub in ('KlaussCPU.srcs/sources_1/new', 'KlaussCPU.srcs/sim_1/new', 'tools'):
        shutil.copytree(os.path.join(ROOT, sub), os.path.join(d, sub), ignore=shutil.ignore_patterns('__pycache__'))
    return d


def check_patterns():
    bad = []
    for m in MUTS:
        t = open(os.path.join(ROOT, 'KlaussCPU.srcs', m[2])).read()
        for old in m[4::2]:
            if t.count(old) != 1:
                bad.append(f'{m[0]}: pattern found {t.count(old)} times')
    return bad


def run_mut(m, keep=False):
    name, what, f, suite = m[:4]
    d = stage(name)
    p = os.path.join(d, 'KlaussCPU.srcs', f)
    t = open(p).read()
    for old, new in zip(m[4::2], m[5::2]):
        if t.count(old) != 1:
            return name, 'BAD', f'pattern found {t.count(old)} times'
        t = t.replace(old, new)
    open(p, 'w').write(t)
    killed, first = False, '?'
    for cmd in suite:
        env = dict(os.environ)
        env.update(VSIM_WORK=os.path.join(d, 'vw'), VSIM_GIT=ROOT, PYTHONDONTWRITEBYTECODE='1')
        if isinstance(cmd, tuple):
            env.update(cmd[0]); cmd = cmd[1]
        try:
            r = subprocess.run([sys.executable, os.path.join(d, 'tools/vsim/run.py')] + cmd, env=env,
                               capture_output=True, text=True, timeout=7200)
            out, rc = r.stdout + r.stderr, r.returncode
        except subprocess.TimeoutExpired:
            out, rc = 'timeout', 1
        if rc != 0:
            killed = True
            first = ' '.join(cmd[:2]) + ': ' + next((l for l in out.splitlines() if l.startswith('FAIL') or 'failing' in l
                                                     or 'failed' in l or 'atal' in l or 'timeout' in l), out[-200:])
            break
    if not keep:
        shutil.rmtree(d, ignore_errors=True)
    return name, 'killed' if killed else 'SURVIVED', first[:220]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('-j', type=int, default=6)
    ap.add_argument('--keep', action='store_true')
    ap.add_argument('--list', action='store_true', help='check every pattern applies, then stop')
    a = ap.parse_args()
    bad = check_patterns()
    if bad or a.list:
        print('\n'.join(bad) or f'{len(MUTS)} mutations, every pattern found once')
        return 1 if bad else 0
    sel = [m for m in MUTS if not a.only or m[0] in a.only]
    os.makedirs(WORK, exist_ok=True)
    res = []
    with cf.ThreadPoolExecutor(a.j) as ex:
        for name, st, first in ex.map(lambda m: run_mut(m, a.keep), sel):
            what = next(m[1] for m in MUTS if m[0] == name)
            print(f'{st:9s} {name:14s} {what}\n          {first}', flush=True)
            res.append((name, st))
    k = sum(st == 'killed' for _, st in res)
    print(f'{k} of {len(res)} mutations killed')
    return 0 if k == len(res) else 1


if __name__ == '__main__':
    sys.exit(main())

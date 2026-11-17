#!/usr/bin/env python3
"""Mutation checks of the HW_PERF_PLAN §10 timing restructures.

Each restructure is a new form of an old function with a simulation
assertion that the two are equal.  Each mutation breaks the new form; a
"_na" variant breaks it with that assertion removed, so the existing tests
must see it through behaviour.  Same runner as fablemut.py: the mutant's
sources are copied to $VSIM_WORK/tmg2mut/<name>, its suite (run.py
commands) runs there, and it is killed when a command fails.

    python3 tools/vsim/tmg2mut.py [--only NAME ...] [-j 6]
"""
import argparse, concurrent.futures as cf, os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'tmg2mut')
TOP, PC, L2, BS = ('sources_1/new/KlaussCPU.sv', 'sources_1/new/pipeline_core.sv',
                   'sources_1/new/l2_cache.sv', 'sources_1/new/bus_splitter.sv')
BASE = os.environ.get('TMG2MUT_BASE', 'e298fc6')   # origin/master at the start of §10

CORE = [['test'], ['diff', '--base', BASE, '--seeds', '12', '--lats', '1', '3']]
FWD = [['test'], ['fwdfuzz', '--base', BASE, '--seeds', '8'], ['diff', '--base', BASE, '--seeds', '12', '--lats', '1', '3']]
SOC = [['socdiff', '--base', BASE, '--seeds', '6'], ['socmmio', '--base', BASE]]
# (the L2's DDR read path: socdiff's small programs hit in the L2 after the boot copy wrote them)
L2S = [['socl2']]
TLB = [['test', 'tx_*', 'tlb_*', 'wk_*'], ['tlbpage'], ['tlbdiff']]

# (name, what it breaks, file, suite (list of run.py argument lists), old, new[, old2, new2 ...])
MUTS = [
    # ---------------- L2: the read beat's lanes by case ----------------
    ('l2_lane', 'beat 0 merged into the wrong half of the sector', L2, L2S,
     "            2'b00: rb_buf[191:128] = app_rd_data;\n            2'b01: rb_buf[ 63:  0] = app_rd_data;",
     "            2'b00: rb_buf[ 63:  0] = app_rd_data;\n            2'b01: rb_buf[191:128] = app_rd_data;"),
    ('l2_lane_na', 'the same, without its equivalence assertion', L2, L2S,
     "            2'b00: rb_buf[191:128] = app_rd_data;\n            2'b01: rb_buf[ 63:  0] = app_rd_data;",
     "            2'b00: rb_buf[ 63:  0] = app_rd_data;\n            2'b01: rb_buf[191:128] = app_rd_data;",
     "        assert (rb_buf == rb_buf_ref) else", "        assert (1'b1) else"),
    ('l2_half_na', "a narrow answer's half swapped (no assertion)", L2, L2S,
     "wire [127:0] rb_half = rb_h ? rb_buf[127:0] : rb_buf[255:128];",
     "wire [127:0] rb_half = rb_h ? rb_buf[255:128] : rb_buf[127:0];",
     "        assert (rb_half == rb_buf_ref[255 - 128*rb_h -: 128]) else", "        assert (1'b1) else"),
    # ---------------- WB forward: the load / MUL value late ----------------
    ('wbl_mul', "a direct MUL's WB value taken from the port extraction", PC, FWD,
     "      wb_ldval = (EN_MUL2 && wb_mdir) ? mul_r2", "      wb_ldval = (EN_MUL2 && wb_mdir && 1'b0) ? mul_r2"),
    ('wbl_ext_na', 'the port value never extracted (no assertion)', PC, FWD,
     "      if (wb_ld_full) wb_ldval_p = m_rdata_dram;", "      if (1'b1) wb_ldval_p = m_rdata_dram;",
     "      assert (wb_ldval == ((EN_DC_H && wb_h && wb_hok && !(EN_MUL2 && wb_mdir)) ? wb_hext", "      assert (1'b1 || wb_ldval == ((EN_DC_H && wb_h && wb_hok && !(EN_MUL2 && wb_mdir)) ? wb_hext"),
    ('wl_a', "the WB load forward of rs1 wins over a younger MEM producer", PC, FWD,
     "wire wl_a = b_wb && wb_ldf && wb_rd == id_rs1  && !(mem_fwd_ok && mem_d.rd == id_rs1);",
     "wire wl_a = b_wb && wb_ldf && wb_rd == id_rs1;"),
    ('wl_a_na', 'the same, without the forward-form assertion', PC, FWD,
     "wire wl_a = b_wb && wb_ldf && wb_rd == id_rs1  && !(mem_fwd_ok && mem_d.rd == id_rs1);",
     "wire wl_a = b_wb && wb_ldf && wb_rd == id_rs1;",
     '         else $fatal(1, "shift-last forward form (pc %h)", id_pc);', '         else ;'),
    ('wl_b_na', 'an immediate operand replaced by a WB load whose rd matches the rs2 field (no assertion)', PC, FWD,
     "(wl_b && !id_b_one && !id_b_imm) ? f_wsh(id_w, wb_ldval)", "(wl_b && !id_b_one) ? f_wsh(id_w, wb_ldval)",
     '         else $fatal(1, "shift-last forward form (pc %h)", id_pc);', '         else ;'),
    ('wl_c_na', "ex_c's WB load forward ignores the MEM producer (no assertion)", PC, [['test', 'fwd_wl_prio']] + FWD,
     "wire wl_c = b_wb && wb_ldf && wb_rd == id_rd   && !(mem_fwd_ok && mem_d.rd == id_rd);",
     "wire wl_c = b_wb && wb_ldf && wb_rd == id_rd;",
     '         else $fatal(1, "shift-last forward form (pc %h)", id_pc);', '         else ;'),
    ('wl_ta_na', "the TLB key's WB load forward taken for a stack op (no assertion)", PC, TLB + [['test', 'tlb_*']],
     "                      : (wl_a && !t_sp && !(t_mem && t_md == 2'b10)) ? wb_ldval[31:0] : t_ta_n;",
     "                      : (wl_a && !(t_mem && t_md == 2'b10)) ? wb_ldval[31:0] : t_ta_n;",
     '         else $fatal(1, "shift-last forward form (pc %h)", id_pc);', '         else ;'),
    # ---------------- ID: the candidate predecode ----------------
    ('pd_sel', "the slot-0 / slot-1 predecode swapped", PC, [['test']],
     "wire [2:0] w_pd = pc[2] ? (in0 ? w_pd1 : w_pd3) : (in0 ? w_pd0 : w_pd2);",
     "wire [2:0] w_pd = pc[2] ? (in0 ? w_pd1 : w_pd3) : (in0 ? w_pd2 : w_pd0);"),
    ('pd_sel_na', 'the same, without its assertion', PC, CORE,
     "wire [2:0] w_pd = pc[2] ? (in0 ? w_pd1 : w_pd3) : (in0 ? w_pd0 : w_pd2);",
     "wire [2:0] w_pd = pc[2] ? (in0 ? w_pd1 : w_pd3) : (in0 ? w_pd2 : w_pd0);",
     "      assert (w_pd_ser == w_decf.serialize && w_pd_priv == f_priv(w_dec, w_op) && w_pd_legal == w_dec.legal)",
     "      assert (1'b1)"),
    ('lcy', "the last word's carry for LEN 11 from pc[26:2]", PC, [['test']],
     "return (len == 2'b11 && &p[26:3]) || (len == 2'b10 && &p[26:2]);",
     "return (len == 2'b11 && &p[26:2]) || (len == 2'b10 && &p[26:2]);"),
    ('pkseg', "the frame's kseg test without the SP - 8 borrow", PC, TLB,
     "return (v[26:3] == 24'd0) ? (v[31:27] == 5'b10001) : (v[31:27] == 5'b10000);",
     "return v[31:27] == 5'b10000;"),
    # ---------------- MEM: the registered port classes ----------------
    ('cls_prd_na', 'IRET is not a port read (no assertion)', PC, CORE,
     "               mem_prdq   <= (ex_d.uop == U_LOAD || ex_d.uop == U_POP || ex_d.uop == U_RET ||\n                              ex_d.uop == U_IRET ||",
     "               mem_prdq   <= (ex_d.uop == U_LOAD || ex_d.uop == U_POP || ex_d.uop == U_RET ||\n                              1'b0 ||",
     '            else $fatal(1, "§10 MEM port-class drift: pc=%h uop=%0d", mem_pc, mem_d.uop);', '            else ;'),
    ('cls_trap', "a TRAP conversion keeps the store class", PC, [['test']],
     "                  mem_prdq     <= 1'b0;  mem_pwrq <= 1'b0;", "                  mem_prdq     <= 1'b0;"),
    ('cls_sh_na', 'the MEM shift class from ALU ops (no assertion)', PC, CORE,
     "               mem_shq    <= ex_d.uop == U_SHIFT;", "               mem_shq    <= ex_d.uop == U_ALU;",
     '            else $fatal(1, "§10 MEM port-class drift: pc=%h uop=%0d", mem_pc, mem_d.uop);', '            else ;'),
    # ---------------- fetch: the NLB compare-then-select ----------------
    ('nlb_sel', "the NLB match's in0 select swapped", PC, [['test']],
     "wire        nlb0_dw_ok     = in0 ? nlb0_ok_b1 : nlb0_ok_pc;", "wire        nlb0_dw_ok     = in0 ? nlb0_ok_pc : nlb0_ok_b1;"),
    ('nlb_sel_na', 'the same, without its assertion', PC, CORE,
     "wire        nlb0_dw_ok     = in0 ? nlb0_ok_b1 : nlb0_ok_pc;", "wire        nlb0_dw_ok     = in0 ? nlb0_ok_pc : nlb0_ok_b1;",
     '         else $fatal(1, "nlb*_dw_ok != the miss_dw form (pc %h)", pc);', '         else ;'),
    # ---------------- SoC: the bus region predecode ----------------
    ('bus_eth', 'the LiteEth region without 0xF007 (the RX slots)', TOP, [['soceth'], ['socmmio', '--base', BASE]],
     "return a[27:16] == 12'h006 || a[27:16] == 12'h007 || a[27:16] == 12'h008;",
     "return a[27:16] == 12'h006 || a[27:16] == 12'h008;"),
    ('bus_pip', "the pipeline's accesses never MMIO", TOP, [['socmmio', '--base', BASE]],
     "wire        w_bus_io   = r_pipe_owns_dec ? pip_m_io : w_st_io;", "wire        w_bus_io   = r_pipe_owns_dec ? 1'b0 : w_st_io;"),
    # ---------------- round 2 ----------------
    ('pdx_sel', "the field predecode's slot-0 / slot-1 candidates swapped", PC, [['test']],
     "wire [14:0] w_px = pc[2] ? (in0 ? w_pdx[1] : w_pdx[3]) : (in0 ? w_pdx[0] : w_pdx[2]);",
     "wire [14:0] w_px = pc[2] ? (in0 ? w_pdx[1] : w_pdx[3]) : (in0 ? w_pdx[2] : w_pdx[0]);"),
    ('pdx_rs1_na', "id_rs1 latched from the predecoded rd field (no assertion sees it)", PC, CORE,
     "id_rs1    <= w_px[14:11];", "id_rs1    <= w_px[10:7];"),
    ('pdx_use_na', "id_use_rs2 latched from use_rdd (no assertion sees it)", PC, CORE,
     "id_use_rs2 <= w_px[5];", "id_use_rs2 <= w_px[4];"),
    ('maddr_sd_na', "SD's address copy always the FSM's", TOP, [['socsd']],
     "assign sd_bus.addr       = w_maddr[0];", "assign sd_bus.addr       = st.mem_addr;"),
    ('maddr_sel', "a device address copy's ownership select inverted", TOP, [['socmmio', '--base', BASE]],
     "      assign w_maddr[k] = r_owns_dev[k] ? pip_m_addr : st.mem_addr;",
     "      assign w_maddr[k] = r_owns_dev[k] ? st.mem_addr : pip_m_addr;"),
    ('wcnt', "TLB_WCNT counts by 2", PC, [['test', 'wk_*'], ['socwalk']],
     "wire [47:0] wcnt_p1 = wcnt + 48'd1;", "wire [47:0] wcnt_p1 = wcnt + 48'd2;"),
    ('hmmio', "the hit pipe's MMIO test on the wrong nibble value", PC, [['test']],
     "wire        ex_hmmio   = ex_tpg[15:12] == 4'hF;", "wire        ex_hmmio   = ex_tpg[15:12] == 4'hE;"),
    ('hmmio_na', "the same, without its assertion", PC, [['socmmio', '--base', BASE], ['test', 'mmio_*']],
     "wire        ex_hmmio   = ex_tpg[15:12] == 4'hF;", "wire        ex_hmmio   = ex_tpg[15:12] == 4'hE;",
     "         assert (!ex_mph && ex_hmmio == (exo_iaddr[31:28] == 4'hF))", "         assert (1'b1)"),
]

def stage(name):
    d = os.path.join(WORK, name)
    shutil.rmtree(d, ignore_errors=True)
    for sub in ('KlaussCPU.srcs/sources_1/new', 'KlaussCPU.srcs/sim_1/new', 'tools'):
        shutil.copytree(os.path.join(ROOT, sub), os.path.join(d, sub), ignore=shutil.ignore_patterns('__pycache__'))
    return d


def run_mut(m, keep=False):
    name, what, f, suite = m[:4]
    d = stage(name)
    p = os.path.join(d, 'KlaussCPU.srcs', f)
    t = open(p).read()
    for old, new in zip(m[4::2], m[5::2]):     # (one or more textual changes)
        if t.count(old) != 1:
            return name, 'BAD', f'pattern found {t.count(old)} times'
        t = t.replace(old, new)
    open(p, 'w').write(t)
    env = dict(os.environ)
    env.update(VSIM_WORK=os.path.join(d, 'vw'), VSIM_GIT=ROOT, PYTHONDONTWRITEBYTECODE='1')
    killed, first = False, '?'
    for cmd in suite:
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
    a = ap.parse_args()
    sel = [m for m in MUTS if not a.only or m[0] in a.only]
    os.makedirs(WORK, exist_ok=True)
    res = []
    with cf.ThreadPoolExecutor(a.j) as ex:
        for name, st, first in ex.map(lambda m: run_mut(m, a.keep), sel):
            what = next(m[1] for m in MUTS if m[0] == name)
            print(f'{st:9s} {name:12s} {what}\n          {first}', flush=True)
            res.append((name, st))
    k = sum(st == 'killed' for _, st in res)
    print(f'{k} of {len(res)} mutations killed')
    return 0 if k == len(res) else 1


if __name__ == '__main__':
    sys.exit(main())

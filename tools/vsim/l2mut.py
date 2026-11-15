#!/usr/bin/env python3
"""HW_PERF_PLAN.md §6: mutation checks of l2_cache's coherence logic.

Each mutation is a single textual change to l2_cache.sv; the mutant is built
into the tb_l2 unit bench (Verilator) and run on every directed test and on
random seeds under two MIG timings.  A mutation is killed when any run fails
(a reference-memory mismatch, a DDR mismatch after a write-back, a counter
expectation, or an assertion).  Usage:

    python3 tools/vsim/l2mut.py [--only NAME ...] [--seeds N] [--ways 4]
    python3 tools/vsim/l2mut.py --sweep [--seeds 12] [--n 3000]   (no mutation:
        every test at 2 / 4 / 8 ways under four MIG timings)

Build products go to $VSIM_WORK/l2mut (default /tmp/vsim/l2mut) and are
deleted per mutant.
"""
import argparse, os, re, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
SRC = os.path.join(ROOT, 'KlaussCPU.srcs')
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'l2mut')

TESTS = ['hit', 'sector', 'evict', 'wbhit', 'plru', 'dma', 'wbinv', 'toggle', 'alias', 'wbinvq', 'wkturn', 'rtdrain',
         'vgastream']
TIMINGS = [['+MIGLAT=20'], ['+MIGLAT=8', '+MIGJIT=20', '+MIGRDY=30']]

# (name, what it breaks, old text, new text)
MUTS = [
    ('wbm', 'a DDR read does not wait for a buffer entry of its line',
     "wire       w_rd_can = (e_st == E_IDLE) && !f_wbm(q_line) &&",
     "wire       w_rd_can = (e_st == E_IDLE) &&"),
    ('dma_rd_hit', 'DMA reads ignore the L2 (always DDR)',
     "wire       w_rd_hit  = q_lk && r_hit;",
     "wire       w_rd_hit  = q_lk && r_hit && !q_dma;"),
    ('dma_wr_hit', 'a DMA write to a valid sector goes to DDR (the L2 copy stays stale)',
     "wire       w_wr_dma_hit = q_lk && q_dma && r_hit;",
     "wire       w_wr_dma_hit = 1'b0;"),
    ('dma_wr_clean', 'a DMA write merged into the L2 leaves the sector clean',
     "                                          r_td[r_mway] | (q_sec ? 2'b10 : 2'b01));\n                        db_we = WAYS'(1) << r_mway;\n                    end else if (w_wr_post",
     "                                          r_td[r_mway]);\n                        db_we = WAYS'(1) << r_mway;\n                    end else if (w_wr_post"),
    ('fill_set', 'a request to the set a fill is writing is accepted',
     "!(e_filling && i_mem_addr[6 +: SETW] == e_set) &&",
     "1'b1 &&"),
    ('victim_drop', "an allocation's dirty victim is not written back",
     "wire [1:0] r_vd = r_td[r_vic];",
     "wire [1:0] r_vd = 2'b00;"),
    ('walk_nocopy', 'the walk invalidates dirty ways without writing them back',
     "            if (r_td[w] != 2'b00) begin r_wk_any = 1'b1; r_wk_way = WAYW'(w); end",
     "            if (1'b0) begin r_wk_any = 1'b1; r_wk_way = WAYW'(w); end"),
    ('walk_nodrain', 'BUSY drops before the write buffer has drained',
     "if (r_walking && r_walk_end && r_wk_snap && r_wk_fl == 2'b00) begin",
     "if (r_walking && r_walk_end && r_wk_snap) begin"),
    ('walk_yield_skip', 'a walk that yields mid-set moves on to the next set',
     "                        if (w_acc_ok) f_st <= F_IDLE;",
     "                        if (w_acc_ok) begin f_st <= F_IDLE; r_walk_set <= r_walk_set + 1'b1; end"),
    ('dis_nowalk', 'a write of 0 disables at once (no write-back)',
     "                r_walk_end <= !r_en;",
     "                r_walk_end <= 1'b1;"),
    ('alloc_both', 'a sector allocation marks the other sector valid',
     "tb_din = f_plru_tw(f_setway(r_tw, r_vic, q_tag, q_sec ? 2'b10 : 2'b01,",
     "tb_din = f_plru_tw(f_setway(r_tw, r_vic, q_tag, 2'b11,"),
    ('refetch_present', "a sector miss refetches the whole line (over the other sector's data)",
     "e_n     <= !q_wide ? 3'd1 : (w_rd_both ? 3'd4 : 3'd2);",
     "e_n     <= !q_wide ? 3'd1 : (w_rd_alloc && !i_rt_req ? 3'd4 : 3'd2);"),
    ('rt_valid', 'a sector-only fill (VGA fetching) marks both sectors valid',
     "w_rd_both ? 2'b11 : (q_sec ? 2'b10 : 2'b01),",
     "2'b11,"),
    ('wb_order', 'the write buffer drains entry 0 first, not the oldest',
     "    wire         wb_dh   = wb_h;",
     "    wire         wb_dh   = !wb_v[0];"),
    ('spec_noaddr', 'the speculative lookup is used for any address',
     "wire w_imm = ra_ok && (ra_addr[31:5] == i_mem_addr[31:5]);",
     "wire w_imm = ra_ok;"),
    ('spec_wrcoll', 'a speculative read next to a data-array write is used',
     "                   !(|db_we   && db_addr[SETW:1]   == ra_next[6 +: SETW]) &&\n                   !(|r_db_we && r_db_addr[SETW:1] == ra_next[6 +: SETW]);",
     "                   1'b1;"),
    ('acc_wrcoll', 'a request is accepted while a write to its set is pending',
     "                    !(|r_db_we && i_mem_addr[6 +: SETW] == r_db_addr[SETW:1]);",
     "                    1'b1;"),
    ('early_beat', 'the early critical dword is taken from the wrong beat',
     "if (e_bc[0] == !e_dwoff[0]) begin",
     "if (e_bc[0] == e_dwoff[0]) begin"),
    # review 1
    ('alias_bypass', 'PA[31:27] != 0 bypasses the L2 (the pre-review rule; the MIG aliases PA[27])',
     "                    q_lk    <= r_en;",
     "                    q_lk    <= r_en && i_mem_addr[31:27] == 5'd0;"),
    ('walk_recopy', "the walk clears a copied way's dirty bits in r_tw only (M1: a yield re-copies it)",
     "tb_we = 1'b1; tb_din[r_wk_way*WW +: 2] = 2'b00;",
     "tb_din[r_wk_way*WW +: 2] = 2'b00;"),
    ('rt_drain', 'drains start during VGA / DMA whenever the front end is busy (M2: the pre-review guard)',
     "(!(i_dma || i_rt_req) || w_fe_need)) begin",
     "(!(i_dma || i_rt_req) || f_st != F_IDLE)) begin"),
    ('rt_need_wbm', "a read of a buffered line does not count as waiting on the buffer (VGA: no drain, it waits)",
     ": !w_rd_hit && (f_wbm(q_line) || (w_rd_needslot && wb_full)));",
     ": !w_rd_hit && (w_rd_needslot && wb_full));"),
    ('wk_noturn', 'the walk does not take turns with requests (minor 2)',
     "wire w_acc_ok = calib_done && w_dv && !r_served && !(r_wk_turn && w_wk_on) &&",
     "wire w_acc_ok = calib_done && w_dv && !r_served &&"),
    ('strm_snoop', 'a write to the line does not drop the stream buffer',
     "                        if (i_mem_addr[26:6] == s_sec[21:1]) s_v <= 1'b0;",
     "                        if (1'b0) s_v <= 1'b0;"),
    ('strm_fill_snoop', 'a write during a stream fill does not stop its capture',
     "                        if (i_mem_addr[26:6] == e_line) e_strm_ok <= 1'b0;",
     "                        if (1'b0) e_strm_ok <= 1'b0;"),
    ('strm_any', 'a stream fill also for a line the L2 holds (its sector 1 may be dirty there)',
     "wire       w_strm_new = q_dma && q_wide && !q_sec && !r_many;",
     "wire       w_strm_new = q_dma && q_wide && !q_sec;"),
    ('served', "a DV still high after its ready is taken as a new request (the r_served guard)",
     "wire w_acc_ok = calib_done && w_dv && !r_served && !(r_wk_turn && w_wk_on) &&",
     "wire w_acc_ok = calib_done && w_dv && !(r_wk_turn && w_wk_on) &&"),
    ('busy_write', 'L2_CTRL writes are taken while BUSY',
     "end else if (i_ctrl_wr && !r_walking) begin",
     "end else if (i_ctrl_wr) begin"),
]


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def build(name, l2src, ways):
    d = os.path.join(WORK, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    p = os.path.join(d, 'l2_cache.sv')
    open(p, 'w').write(l2src)
    r = run(['verilator', '--binary', '--timing', '-j', '8', '-Wno-fatal', '-Wno-lint', '-Wno-style',
             '-Wno-MULTIDRIVEN', '-Wno-BLKANDNBLK', '-Wno-UNOPTFLAT', '--assert', '--top-module', 'tb_l2',
             f'+define+L2_WAYS={ways}', '-Mdir', os.path.join(d, 'obj'), p,
             os.path.join(SRC, 'sim_1', 'new', 'mig_model.sv'), os.path.join(SRC, 'sim_1', 'new', 'tb_l2.sv')])
    exe = os.path.join(d, 'obj', 'Vtb_l2')
    if r.returncode != 0 or not os.path.exists(exe):
        print(r.stdout[-2000:], r.stderr[-2000:])
        raise SystemExit(f'build failed: {name}')
    return d, exe


def _one(exe, t, tm):
    args = [exe] + [f'+TEST={t.split()[0]}'] + t.split()[1:] + tm
    try:
        r = run(args, timeout=1800)
        out = r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        out = 'TIMEOUT'
    if 'TB_L2 PASS' in out:
        return None
    m = re.search(r'TB_L2 FAIL.*|Assertion.*|TIMEOUT|fatal.*', out)
    return f'{t} {" ".join(tm)}: {m.group(0)[:120] if m else out[-120:]}'


def runs(exe, seeds, stop_first, timings=None, n=2000):
    """the directed tests and `seeds` random seeds under each MIG timing, in
    parallel (VSIM_JOBS, default 12); stop_first: cancel the rest at the
    first failing run"""
    import concurrent.futures as cf
    timings = timings or TIMINGS
    jobs = [(t, tm) for tm in timings for t in TESTS] + \
           [(f'random +SEED={s} +N={n}', tm) for tm in timings for s in range(1, seeds + 1)]
    fails = []
    with cf.ThreadPoolExecutor(int(os.environ.get('VSIM_JOBS', '12'))) as ex:
        fut = {ex.submit(_one, exe, t, tm): (t, tm) for t, tm in jobs}
        for f in cf.as_completed(fut):
            r = f.result() if not f.cancelled() else None
            if r:
                fails.append(r)
                if stop_first:
                    for g in fut:
                        g.cancel()
    return fails, len(jobs)


# the tb_l2 sweep (HW_PERF_PLAN §6): every directed test and random seeds at
# 2, 4 and 8 ways under four MIG timings
SWEEP_TIMINGS = [['+MIGLAT=4'], ['+MIGLAT=36'], ['+MIGLAT=8', '+MIGJIT=20', '+MIGRDY=30'],
                 ['+MIGLAT=60', '+MIGREF=200', '+MIGRDY=10']]


def sweep(seeds, n):
    base = open(os.path.join(SRC, 'sources_1', 'new', 'l2_cache.sv')).read()
    bad = 0
    for ways in (2, 4, 8):
        d, exe = build(f'sweep{ways}', base, ways)
        f, k = runs(exe, seeds, False, SWEEP_TIMINGS, n)
        shutil.rmtree(d, ignore_errors=True)
        print(f'tb_l2 {ways} ways: {k - len(f)} of {k} runs pass {f[:3]}', flush=True)
        bad += len(f)
    print('SWEEP PASS' if not bad else f'SWEEP FAIL ({bad} runs)')
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--seeds', type=int, default=4)
    ap.add_argument('--ways', type=int, default=4)
    ap.add_argument('--all', action='store_true', help='run every job (default: stop at the first kill)')
    ap.add_argument('--sweep', action='store_true', help='no mutations: the tb_l2 sweep (ways x MIG timings)')
    ap.add_argument('--n', type=int, default=3000, help='--sweep: transactions per random seed')
    a = ap.parse_args()
    if a.sweep:
        return sweep(a.seeds, a.n)
    base = open(os.path.join(SRC, 'sources_1', 'new', 'l2_cache.sv')).read()
    sel = [m for m in MUTS if not a.only or m[0] in a.only]
    if not a.only or 'none' in a.only:
        d, exe = build('none', base, a.ways)
        f, n = runs(exe, a.seeds, False)
        shutil.rmtree(d, ignore_errors=True)
        print(f'unmutated: {len(f)} of {n} runs fail {f[:3]}', flush=True)
        if f:
            return 1
    survived = []
    for name, what, old, new in sel:
        if base.count(old) != 1:
            print(f'{name}: pattern found {base.count(old)} times', flush=True)
            survived.append(name)
            continue
        d, exe = build(name, base.replace(old, new), a.ways)
        f, n = runs(exe, a.seeds, not a.all)
        shutil.rmtree(d, ignore_errors=True)
        print(f'{"killed  " if f else "SURVIVED"} {name:16s} ({what}): {len(f)} failing{" (first)" if f and not a.all else ""}'
              f'{": " + f[0] if f else ""}', flush=True)
        if not f:
            survived.append(name)
    print(f'{len(sel) - len(survived)} of {len(sel)} killed; survived: {survived}')
    return 1 if survived else 0


if __name__ == '__main__':
    sys.exit(main())

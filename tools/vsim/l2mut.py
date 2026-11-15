#!/usr/bin/env python3
"""HW_PERF_PLAN.md §6: mutation checks of l2_cache's coherence logic.

Each mutation is a single textual change to l2_cache.sv; the mutant is built
into the tb_l2 unit bench (Verilator) and run on every directed test and on
random seeds under two MIG timings.  A mutation is killed when any run fails
(a reference-memory mismatch, a DDR mismatch after a write-back, a counter
expectation, or an assertion).  Usage:

    python3 tools/vsim/l2mut.py [--only NAME ...] [--seeds N] [--ways 4]

Build products go to $VSIM_WORK/l2mut (default /tmp/vsim/l2mut) and are
deleted per mutant.
"""
import argparse, os, re, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
SRC = os.path.join(ROOT, 'KlaussCPU.srcs')
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'l2mut')

TESTS = ['hit', 'sector', 'evict', 'wbhit', 'plru', 'dma', 'wbinv', 'toggle', 'range']
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
     "end else if (r_walk_end && wb_v == 2'b00 && e_st == E_IDLE) begin",
     "end else if (r_walk_end) begin"),
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
    ('range', 'an access outside DDR is looked up (aliases a DDR line)',
     "        return a[31:27] == 5'd0;",
     "        return 1'b1;"),
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


def runs(exe, seeds, stop_first):
    fails = []
    jobs = [(t, tm) for tm in TIMINGS for t in TESTS] + \
           [(f'random +SEED={s} +N=2000', tm) for tm in TIMINGS for s in range(1, seeds + 1)]
    for t, tm in jobs:
        args = [exe] + [f'+TEST={t.split()[0]}'] + t.split()[1:] + tm
        try:
            r = run(args, timeout=900)
            out = r.stdout + r.stderr
        except subprocess.TimeoutExpired:
            out = 'TIMEOUT'
        if 'TB_L2 PASS' not in out:
            m = re.search(r'TB_L2 FAIL.*|Assertion.*|TIMEOUT|fatal.*', out)
            fails.append(f'{t} {" ".join(tm)}: {m.group(0)[:120] if m else out[-120:]}')
            if stop_first:
                break
    return fails, len(jobs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--seeds', type=int, default=4)
    ap.add_argument('--ways', type=int, default=4)
    ap.add_argument('--all', action='store_true', help='run every job (default: stop at the first kill)')
    a = ap.parse_args()
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

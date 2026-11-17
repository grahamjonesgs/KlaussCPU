#!/usr/bin/env python3
"""Mutation checks of the HW_PERF_PLAN §9 fixes (the Fable review).

Each mutation is one textual change to an RTL file.  The mutant's sources
(KlaussCPU.srcs/sources_1/new, sim_1/new and tools/) are copied to
$VSIM_WORK/fablemut/<name> and its suite (run.py commands) runs there.
A mutation is killed when its suite fails (a check, an assertion, a
deadlock or a build error).  Mutants run in parallel (-j); their
directories are deleted afterwards.

    python3 tools/vsim/fablemut.py [--only NAME ...] [-j 6]
"""
import argparse, concurrent.futures as cf, os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'fablemut')
TOP, PC, BL = 'sources_1/new/KlaussCPU.sv', 'sources_1/new/pipeline_core.sv', 'sources_1/new/blitter_dma.sv'

EI_SUITE = [['test', 'ei_enable_store'], ['socei', '--srcs', 'mask', 'vec0', 'int2en', '--no-loop', '-j', '4']]

# (name, what it breaks, file, suite (list of run.py argument lists), old, new)
MUTS = [
    # ---------------- item 1: CPU_RESETN ----------------
    ('rst_dv', 'the reset branch keeps the FSM\'s DVs (item 1b)', TOP, [['socreset']],
     "         st.mem_write_DV  <= 1'b0;\n         st.mem_read_DV   <= 1'b0;\n         r_s_rst          <= 1'b0;",
     "         r_s_rst          <= 1'b0;"),
    ('boot_idle', 'boot phase 0 does not wait for the D-cache to be idle (item 1a)', TOP, [['socreset']],
     "if (w_calib_done && w_dc_idle) begin", "if (w_calib_done) begin"),
    # ---------------- item 2: break / command ----------------
    ('brk_preempt', 'a break / command cycle skips the state arms again (item 2)', TOP, [['socbreak']],
     "         case (st.SM)\n            NO_PROGRAM: begin",
     "         if (!(w_uart_break || (w_uart_rx_DV && r_break_received))) case (st.SM)\n            NO_PROGRAM: begin"),
    ('brk_data', 'LOADING_BYTE takes a command byte as data (item 2)', TOP, [['socbreak', 'load_split.*']],
     "if (w_uart_rx_DV && !r_break_received) begin   // a data byte (§9 item 2)", "if (w_uart_rx_DV) begin"),
    # ---------------- item 3: S stops the DMA masters ----------------
    ('s_sd', 'S does not reset the SD controller (item 3)', TOP, [['socsload', 'sd']],
     ".i_load_rst(r_load_pulse || r_s_rst),", ".i_load_rst(r_load_pulse),"),
    ('s_blit', 'S does not reset the blitter (item 3)', TOP, [['socsload', 'blit']],
     ".i_soft_rst(r_s_rst),", ".i_soft_rst(1'b0),"),
    # ---------------- item 4: precise EI ----------------
    ('ei_none', 'no conversion in EX (item 4)', PC, EI_SUITE,
     "wire ex_eicv = EN_DIEI && ex_valid", "wire ex_eicv = 1'b0 && ex_valid"),
    ('ei_disp', 'a converted EI leaves EX without the ID op squashed / fetch halted', PC, EI_SUITE,
     "               end else if (ex_eicv) begin\n", "               end else if (1'b0) begin\n"),
    ('ei_rfch', 'a converted EI leaving EX is not carried to wb_rfch', PC, EI_SUITE,
     "mem_eiser  <= ex_eiser || ex_eicv;", "mem_eiser  <= ex_eiser;"),
    ('ei_regok', 'the conversion takes the combinational irq_ok\'s previous value from a stale register', PC,
     EI_SUITE, "always_ff @(posedge clk) irq_ok_q <= !rst && irq_ok;", "always_ff @(posedge clk) irq_ok_q <= 1'b0;"),
    # (ID conversion: id_eicv only saves the refetch of the op behind the
    # EI — the EX conversion covers it; expected to survive, see §9)
    ('ei_id', 'no conversion in ID (performance only: the EX conversion covers it)', PC, EI_SUITE,
     "wire id_eicv = EN_DIEI && id_valid", "wire id_eicv = 1'b0 && id_valid"),
    # ---------------- item 6: the loader ----------------
    ('ld_mixed', "the loader's nibbles as blocking writes next to the 'X' branch's NBA again (item 6)", TOP,
     [['socbreak', 'load_split5']],
     "o_ram_write_value <= v;", "o_ram_write_value = v;"),
]


def stage(name):
    d = os.path.join(WORK, name)
    shutil.rmtree(d, ignore_errors=True)
    for sub in ('KlaussCPU.srcs/sources_1/new', 'KlaussCPU.srcs/sim_1/new', 'tools'):
        shutil.copytree(os.path.join(ROOT, sub), os.path.join(d, sub), ignore=shutil.ignore_patterns('__pycache__'))
    return d


def run_mut(m, keep=False):
    name, what, f, suite, old, new = m
    d = stage(name)
    p = os.path.join(d, 'KlaussCPU.srcs', f)
    t = open(p).read()
    if t.count(old) != 1:
        return name, 'BAD', f'pattern found {t.count(old)} times'
    open(p, 'w').write(t.replace(old, new))
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
    return 0 if all(st == 'killed' or n == 'ei_id' for n, st in res) else 1


if __name__ == '__main__':
    sys.exit(main())

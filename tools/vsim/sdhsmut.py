#!/usr/bin/env python3
"""Mutation checks of SD high speed (SD_NATIVE_PLAN.md §9, phase 3): the HS
sample phases, the IDDR / IDELAY input path, the strobe skip after a line
release, the IOB pads and their next-value logic, and the default-speed
equivalence (the lock-step against the phase-2 controller, 6dcbcfb).

Each mutation is one textual change to an RTL file (or to the board test's
pick(), which tb_sd_native runs through DPI-C).  The mutant's sources
(KlaussCPU.srcs/sources_1/new, sim_1/new and tools/) are copied to
$VSIM_WORK/sdhsmut/<name> and its suite runs there:
  'hs'     tb_sd_native, high-speed tests only (+HS_ONLY: the phase sweeps
           against the timing model, HS multi-block read / write with CLK
           parks, CMD6 switch and back, resets in HS);
  'native' tb_sd_native, all of it (default speed in lock-step with
           6dcbcfb, then the HS tests);
  'dma'    tb_sd_dma, seed 1 (lock-step; DMA at 50 MHz at the end);
  'compat' tb_sd_spi_compat (SPI mode against the d9cf6c2 controller);
  a list   the named socsddma tests under Verilator (tb_soc).
A mutation is killed when its suite fails (a check, an assertion, or a
build error).  Mutants run in parallel (-j); their directories are deleted
afterwards.

    python3 tools/vsim/sdhsmut.py [--only NAME ...] [-j 4]
"""
import argparse, concurrent.futures as cf, os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'sdhsmut')
SN, SS, TOP = 'sources_1/new/sd_native.sv', 'sources_1/new/sd_spi.sv', 'sources_1/new/KlaussCPU.sv'
SB = '../tools/board/sd_hs_test.c'      # (relative to KlaussCPU.srcs, as the RTL files)

# (name, what it breaks, file, suite, old, new)
MUTS = [
    # ---------------- the HS sample instant (tb_sd_native +HS_ONLY) ----------------
    ('ph_off1', 'every phase samples 5 ns later than SDM_CFG[7:4] says', SN, 'hs',
     "        case (3'd6 - w_ph)", "        case (3'd5 - w_ph)"),
    ('q2_is_q1', 'the falling-edge sample (IDDR Q2) is the rising-edge one', SN, 'hs',
     "            3'd1:    w_hs_cand = w_q2;", "            3'd1:    w_hs_cand = w_q1;"),
    ('hist_short', 'the history skips a stage (the oldest phase is 10 ns too late)', SN, 'hs',
     "r_h1[3] <= r_h1[2];", "r_h1[3] <= r_h1[1];"),
    ('strobe_d1', 'the HS strobe one cycle early (r_samp_d): every phase off by 10 ns', SN, 'hs',
     "    wire       w_samp = r_cfg_hs ? r_samp_dd : r_samp;", "    wire       w_samp = r_cfg_hs ? r_samp_d : r_samp;"),
    ('hs_noop', 'SDM_CFG[1] changes nothing (the default sampling at 50 MHz)', SN, 'hs',
     "    wire [4:0] w_smp  = r_cfg_hs ? r_hs_s    : w_q1;", "    wire [4:0] w_smp  = w_q1;"),
    ('no_clamp', 'phases 7..15 are not clamped to 6', SN, 'hs',
     "    wire  [2:0] w_ph = (r_cfg_phase > 4'd6) ? 3'd6 : r_cfg_phase[2:0];",
     "    wire  [2:0] w_ph = r_cfg_phase[2:0];"),
    ('skip_cmd', 'no HS skip after CMD is released (a stale host bit taken as the response start)', SN, 'hs',
     "                                    r_c_skip   <= r_cfg_hs ? 2'd2 : 2'd0;",
     "                                    r_c_skip   <= 2'd0;"),
    ('skip_cmd1', 'the HS skip after CMD is released is one strobe, not two', SN, 'hs',
     "                                    r_c_skip   <= r_cfg_hs ? 2'd2 : 2'd0;",
     "                                    r_c_skip   <= r_cfg_hs ? 2'd1 : 2'd0;"),
    ('skip_dat', 'no HS skip after a write block (a stale host CRC bit taken as the token start)', SN, 'hs',
     "                            r_d_skip  <= r_cfg_hs ? 2'd2 : 2'd0;", "                            r_d_skip  <= 2'd0;"),
    ('tap_noload', 'the IDELAY tap is never loaded (SDM_CFG[12:8] does nothing)', SN, 'hs',
     "    always_ff @(posedge i_Clk) r_idly_ld <= i_rst || (w_wr && w_off == O_CFG);",
     "    always_ff @(posedge i_Clk) r_idly_ld <= i_rst;"),
    ('tap_norst', 'a reset does not reload the IDELAY tap (0)', SN, 'hs',
     "    always_ff @(posedge i_Clk) r_idly_ld <= i_rst || (w_wr && w_off == O_CFG);",
     "    always_ff @(posedge i_Clk) r_idly_ld <= (w_wr && w_off == O_CFG);"),
    ('q2_swap', 'the IDDR outputs swapped in sd_spi (Q1 <-> Q2)', SS, 'hs',
     "    assign {w_dat_q2, w_cmd_q2} = w_q2;", "    assign {w_dat_q2, w_cmd_q2} = w_q1;"),
    ('cfg_rd', 'SDM_CFG reads the tap back as 0', SN, 'hs',
     "{47'h0, r_cfg_dma, 3'h0, r_cfg_tap, r_cfg_phase, 1'b0,", "{47'h0, r_cfg_dma, 3'h0, 5'h0, r_cfg_phase, 1'b0,"),
    # ---------------- the pads and default speed (tb_sd_native, lock-step) ----------------
    ('ds_hs_sample', 'default speed takes the HS sample (r_hs_s)', SN, 'native',
     "    wire [4:0] w_smp  = r_cfg_hs ? r_hs_s    : w_q1;", "    wire [4:0] w_smp  = r_hs_s;"),
    ('ds_hs_strobe', 'default speed takes the HS strobe (r_samp_dd)', SN, 'native',
     "    wire       w_samp = r_cfg_hs ? r_samp_dd : r_samp;", "    wire       w_samp = r_samp_dd;"),
    ('ds_dat_skip', 'the skip after a write block also in default speed, 3 strobes (eats the CRC token start)', SN, 'native',
     "                            r_d_skip  <= r_cfg_hs ? 2'd2 : 2'd0;", "                            r_d_skip  <= r_cfg_hs ? 2'd2 : 2'd3;"),
    ('pad_late', "the CLK pad takes the current value, not the next (a cycle late)", SS, 'native',
     ".D1(nx_sck), .D2(nx_sck)", ".D1(w_ref_sck), .D2(w_ref_sck)"),
    ('pad_d2', "the CLK ODDR's falling-edge half inverted", SS, 'native',
     ".D1(nx_sck), .D2(nx_sck)", ".D1(nx_sck), .D2(~nx_sck)"),
    ('nx_start', "the next-value logic misses the command start bit", SN, 'native',
     "                    nx_cmd = r_c_tx[39];  nx_cmd_oe = 1'b1;", "                    nx_cmd = r_c_tx[39];"),
    ('nx_wdata', "the next-value logic sends the write data's next nibble", SN, 'native',
     "                    D_WR_DATA: nx_dat = r_d_w4 ? r_d_sh[63:60] : {3'b111, r_d_sh[63]};",
     "                    D_WR_DATA: nx_dat = r_d_w4 ? r_d_sh[59:56] : {3'b111, r_d_sh[62]};"),
    ('nx_spi', "the SPI MOSI next value ignores a kick", SS, 'compat',
     "            if (w_kick_byte) nx_mosi = mmio.write_data[7];", ""),
    ('caps_hs', 'SDM_CAPS[9] (high speed) reads 0', SN, 'native',
     "64'h0000_0000_0008_0B01", "64'h0000_0000_0008_0901"),
    ('st_rdy', 'SDM_STATUS[6] (IDELAY_RDY) reads 0', SN, 'native',
     "1'b0, r_idly_rdy, w_dma_busy, w_thresh,", "1'b0, 1'b0, w_dma_busy, w_thresh,"),
    # ---------------- DMA at 50 MHz (tb_sd_dma) ----------------
    ('dma_ds_data', 'the data lines take the default sample (Q1) in HS (DMA at 50 MHz)', SN, 'dma',
     "    wire [3:0] w_din  = w_smp[4:1];", "    wire [3:0] w_din  = w_q1[4:1];"),
    ('hs_strobe2', 'the HS strobe two cycles early (r_samp): the sample 20 ns early with it', SN, 'hs',
     "    wire       w_samp = r_cfg_hs ? r_samp_dd : r_samp;", "    wire       w_samp = r_samp;"),
    # ---------------- the board test's pick() (tb_sd_native +HS_ONLY, DPI-C) ----------------
    ('pick_alias', "sd_hs_test.c's pick() keeps a phase p <= 2 (may take bit j - 1) instead of p + 4", SB, 'hs',
     "    if (bi >= 0 && pp[bi] <= 2 && ok[bi + 4]) *best_p = pp[bi] + 4;", ""),
    # ---------------- the SoC (tb_soc, sd_card_sync +SDC_HS) ----------------
    ('top_tristate', "the top's CMD tristate inverted (the card can never answer)", TOP, ['hs_rd_p2'],
     "   assign io_SD_CMD = w_sd_cmd_t ? 1'bz : w_sd_cmd_o;", "   assign io_SD_CMD = w_sd_cmd_t ? w_sd_cmd_o : 1'bz;"),
    ('soc_wr_skip', 'the DAT0 skip after a write block is 0 in HS (tb_soc write at 50 MHz)', SN, ['hs_wr_p*'],
     "                            r_d_skip  <= r_cfg_hs ? 2'd2 : 2'd0;", "                            r_d_skip  <= 2'd3;"),
]


def stage(name):
    d = os.path.join(WORK, name)
    shutil.rmtree(d, ignore_errors=True)
    for sub in ('KlaussCPU.srcs/sources_1/new', 'KlaussCPU.srcs/sim_1/new', 'tools'):
        shutil.copytree(os.path.join(ROOT, sub), os.path.join(d, sub), ignore=shutil.ignore_patterns('__pycache__', 'out'))
    return d


def run_sd(d, which, env, args=''):
    env = dict(env, SDSIM_WORK=os.path.join(d, 'sdsim'), SD_M0_GIT=ROOT, XSIM_ARGS=args)
    r = subprocess.run(['bash', os.path.join(d, 'tools/sdsim/run_sd_tb.sh'), which], env=env,
                       capture_output=True, text=True, timeout=5400)
    tag = {'native': 'TB_SD PASS', 'dma': 'TB_SDDMA PASS', 'compat': 'TB_SDSPI PASS'}[which]
    top = {'native': 'tb_sd_native', 'dma': 'tb_sd_dma', 'compat': 'tb_sd_spi_compat'}[which]
    log = os.path.join(d, 'sdsim', top, 'xsim.log')
    fat = ''
    if os.path.exists(log):
        fat = '\n'.join(l for l in open(log, errors='replace').read().splitlines() if 'Fatal' in l or 'FATAL' in l)
    return fat + '\n' + r.stdout + r.stderr, r.returncode != 0 or tag not in r.stdout


def run_mut(m, keep=False):
    name, what, f, suite, old, new = m
    d = stage(name)
    p = os.path.join(d, 'KlaussCPU.srcs', f)
    t = open(p).read()
    if t.count(old) != 1:
        return name, 'BAD', f'pattern found {t.count(old)} times'
    open(p, 'w').write(t.replace(old, new))
    env = dict(os.environ)
    if suite == 'hs':
        out, killed = run_sd(d, 'native', env, '-testplusarg HS_ONLY')
    elif suite in ('native', 'dma', 'compat'):
        out, killed = run_sd(d, suite, env, '-testplusarg SEED=1' if suite == 'dma' else '')
    else:
        env.update(VSIM_WORK=os.path.join(d, 'vw'))
        r = subprocess.run([sys.executable, os.path.join(d, 'tools/vsim/run.py'), 'socsddma'] + suite, env=env,
                           capture_output=True, text=True, timeout=7200)
        out = r.stdout + r.stderr
        killed = r.returncode != 0
    first = next((l for l in out.splitlines()
                  if 'FAIL' in l or 'atal' in l or 'ERROR' in l or 'MISMATCH' in l or 'failed' in l), '?')
    if not keep:
        shutil.rmtree(d, ignore_errors=True)
    return name, 'killed' if killed else 'SURVIVED', first[:220]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('-j', type=int, default=4)
    ap.add_argument('--keep', action='store_true')
    a = ap.parse_args()
    sel = [m for m in MUTS if not a.only or m[0] in a.only]
    os.makedirs(WORK, exist_ok=True)
    res = []
    with cf.ThreadPoolExecutor(a.j) as ex:
        for name, st, first in ex.map(lambda m: run_mut(m, a.keep), sel):
            what = next(m[1] for m in MUTS if m[0] == name)
            print(f'{st:9s} {name:14s} {what}\n          {first}', flush=True)
            res.append(st)
    k = res.count('killed')
    print(f'{k} of {len(res)} mutations killed')
    return 0 if k == len(res) else 1


if __name__ == '__main__':
    sys.exit(main())

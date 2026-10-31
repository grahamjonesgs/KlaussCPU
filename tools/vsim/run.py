#!/usr/bin/env python3
"""tools/vsim/run.py — open-source simulation of pipeline_core (Verilator).

  build:  python3 tools/vsim/run.py build [--rev REV]   (default: working tree)
  test:   python3 tools/vsim/run.py test [names...] [--rev REV]
  diff:   python3 tools/vsim/run.py diff --base REV [--seeds N]
          random programs on REV and on the working tree; the retire traces
          must be identical (architecture-neutral changes) — cycles reported.

Directed tests live in tools/vsim/tests.py (each returns an Asm plus the
expected final-state fields).  Build products go to $VSIM_WORK (default
/tmp/vsim).  Verilator 5.020 cannot take a default value on a module input,
so the core is copied and `dbg_i = '0` is stripped from the copy only.
"""
import argparse, hashlib, os, re, subprocess, sys, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
SRC = os.path.join(ROOT, 'KlaussCPU.srcs', 'sources_1', 'new')
WORK = os.environ.get('VSIM_WORK', '/tmp/vsim')
sys.path.insert(0, HERE)


def git_show(rev, path):
    rel = os.path.relpath(path, ROOT)
    return subprocess.check_output(['git', '-C', ROOT, 'show', f'{rev}:{rel}'], text=True)


def build(rev=None, quiet=True):
    dbg = os.environ.get('VSIM_DEBUG_EN') == '1'     # elaborate DEBUG_EN=1
    tag = (rev or 'wt') + ('_dbg' if dbg else '')
    d = os.path.join(WORK, 'build_' + re.sub(r'[^A-Za-z0-9_.-]', '_', tag))
    os.makedirs(d, exist_ok=True)
    srcs = {}
    for name in ('klauss_pkg.sv', 'pipeline_core.sv'):
        p = os.path.join(SRC, name)
        srcs[name] = git_show(rev, p) if rev else open(p).read()
    core = srcs['pipeline_core.sv'].replace("dbg_in_t      dbg_i = '0,", 'dbg_in_t      dbg_i,')
    # Verilator 5.020: no NBA to an unpacked array inside a for loop — the
    # invalidate-all loops become (equivalent) whole-array assignments.
    core = re.sub(r"for \(int i = 0; i < IC_LINES; i\+\+\) begin ic_vhi\[i\] <= 1'b0; ic_vlo\[i\] <= 1'b0; end",
                  "begin ic_vhi <= '{default:1'b0}; ic_vlo <= '{default:1'b0}; end", core)
    core = core.replace("for (int i = 0; i < 16; i++) rf[i] <= 64'b0;", "rf <= '{default:64'b0};")
    srcs['pipeline_core.sv'] = core
    tb = open(os.path.join(HERE, 'tb_vsim.sv')).read()
    h = hashlib.sha1((srcs['klauss_pkg.sv'] + core + tb + str(dbg)).encode()).hexdigest()
    exe = os.path.join(d, 'obj', 'Vtb_vsim')
    stamp = os.path.join(d, 'stamp')
    if os.path.exists(exe) and os.path.exists(stamp) and open(stamp).read() == h:
        return exe
    for n, s in srcs.items():
        open(os.path.join(d, n), 'w').write(s)
    defs = ['-GDEBUG_EN=1'] if dbg else []
    if 'st_post_ok' in core:
        defs.append('+define+HAS_POST')
    if 'output logic [1:0]   perf_ic' in core:
        defs.append('+define+HAS_PERF_IC')
    cmd = ['verilator', '--binary', '--timing', '-j', '8', '-O2', '-Wno-fatal', '-Wno-lint',
           '-Wno-style', '-Wno-MULTIDRIVEN', '-Wno-UNOPTFLAT', '--assert', '--top-module', 'tb_vsim',
           '-Mdir', os.path.join(d, 'obj'), *defs,
           os.path.join(d, 'klauss_pkg.sv'), os.path.join(d, 'pipeline_core.sv'),
           os.path.join(HERE, 'tb_vsim.sv')]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-4000:], r.stderr[-4000:])
        raise SystemExit(f'verilator build failed ({tag})')
    open(stamp, 'w').write(h)
    return exe


SOC_FILES = ['klauss_pkg.sv', 'membus_if.sv', 'mmio_if.sv', 'pipeline_core.sv', 'core2_subsys.sv',
             'bus_splitter.sv', 'mem_read_write.sv', 'boot_rom.sv', 'uart_send_msg.sv', 'uart_rx.sv',
             'uart_rx_fifo.sv', 'sd_spi.sv', 'crypto_aes.sv', 'crypto_sha.sv', 'aes_core.sv',
             'aes_sbox.sv', 'ghash.sv', 'sha256_core.sv', 'trng.sv', 'blitter_dma.sv',
             'eth_mmio_bridge.sv', 'Seven_seg_LED_Display_Controller.sv', 'SPI_Master.sv',
             'SPI_Master_With_Single_CS.sv', 'RGB_LED.sv', 'vga_timing.sv', 'vga_scanout.sv',
             'vga_ctrl.sv', 'uart_tx.sv', 'KlaussCPU.sv']
ODDR_STUB = '''module ODDR #(parameter DDR_CLK_EDGE = "SAME_EDGE", parameter INIT = 1'b0, parameter SRTYPE = "SYNC")
   (output reg Q, input C, input CE, input D1, input D2, input R, input S);
   initial Q = INIT;
   always @(posedge C) Q <= D1;
endmodule
'''


def patch_core(core):
    core = core.replace("dbg_in_t      dbg_i = '0,", 'dbg_in_t      dbg_i,')
    core = re.sub(r"for \(int i = 0; i < IC_LINES; i\+\+\) begin ic_vhi\[i\] <= 1'b0; ic_vlo\[i\] <= 1'b0; end",
                  "begin ic_vhi <= '{default:1'b0}; ic_vlo <= '{default:1'b0}; end", core)
    return core.replace("for (int i = 0; i < 16; i++) rf[i] <= 64'b0;", "rf <= '{default:64'b0};")


def build_soc(rev=None):
    """full SoC (sim_1/new/tb_soc.sv: behavioural DDR, real cache / bus /
    MMIO / HCF) under Verilator.  tb_soc reads netboot.mem from its cwd."""
    tag = rev or 'wt'
    d = os.path.join(WORK, 'soc_' + re.sub(r'[^A-Za-z0-9_.-]', '_', tag))
    src = os.path.join(d, 'src')
    os.makedirs(src, exist_ok=True)
    allsrc = ''
    names = SOC_FILES + ['uart_tasks.vh']
    for n in names:
        p = os.path.join(SRC, n)
        t = git_show(rev, p) if rev else open(p).read()
        if n == 'pipeline_core.sv':
            t = patch_core(t)
        if n == 'uart_send_msg.sv':
            # mixed blocking / NBA on i_msg_count mis-simulates in Verilator
            # 5.020 (BLKANDNBLK waived); the IDLE reset is equivalent blocking
            t = t.replace('i_msg_count <= 0;', 'i_msg_count = 0;')
        open(os.path.join(src, n), 'w').write(t)
        allsrc += t
    tbp = os.path.join(ROOT, 'KlaussCPU.srcs', 'sim_1', 'new', 'tb_soc.sv')
    tb = git_show(rev, tbp) if rev else open(tbp).read()
    # report the cycle count at HALTED (for real-cache A/B timing)
    tb = tb.replace('$display("TB_SOC: HALTED after %0d retired instructions, %0d UART bytes", icount, uart_n);',
                    '$display("TB_SOC: HALTED after %0d retired instructions, %0d UART bytes, t=%0d", icount, uart_n, runcyc);')
    tb = tb.replace('   longint last_i = -1;', '   longint last_i = -1;\n   longint runcyc = 0;\n'
                    '   always @(posedge dut.i_Clk) if (dut.st.SM == PIPE_RUN) runcyc++;')
    open(os.path.join(src, 'tb_soc.sv'), 'w').write(tb)
    open(os.path.join(src, 'unisim_stubs.v'), 'w').write(ODDR_STUB)
    h = hashlib.sha1((allsrc + tb).encode()).hexdigest()
    exe = os.path.join(d, 'obj', 'Vtb_soc')
    stamp = os.path.join(d, 'stamp')
    if os.path.exists(exe) and os.path.exists(stamp) and open(stamp).read() == h:
        return exe
    cmd = ['verilator', '--binary', '--timing', '-j', '8', '-Wno-fatal', '-Wno-lint', '-Wno-style',
           '-Wno-MULTIDRIVEN', '-Wno-BLKANDNBLK', '-Wno-UNOPTFLAT', '--top-module', 'tb_soc',
           '-I' + src, '-Mdir', os.path.join(d, 'obj')] + \
          [os.path.join(src, n) for n in SOC_FILES] + \
          [os.path.join(src, 'tb_soc.sv'), os.path.join(src, 'unisim_stubs.v')]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-4000:], r.stderr[-4000:])
        raise SystemExit(f'verilator SoC build failed ({tag})')
    open(stamp, 'w').write(h)
    return exe


def run_soc(exe, image, name, maxi=0, timeout=1800):
    """image: an Asm (assembled into a boot image) or a .mem path"""
    d = os.path.join(WORK, 'socruns', name)
    os.makedirs(d, exist_ok=True)
    mem = os.path.join(d, 'netboot.mem')
    if isinstance(image, str):
        shutil.copy(image, mem)
    else:
        image.write_boot_mem(mem)
    args = [exe, f'+TRACE={d}/soc.trace', f'+UARTF={d}/soc.uart'] + ([f'+MAXI={maxi}'] if maxi else [])
    r = subprocess.run(args, capture_output=True, text=True, cwd=d, timeout=timeout)
    return {'log': r.stdout + r.stderr, 'trace': os.path.join(d, 'soc.trace'),
            'uart': open(os.path.join(d, 'soc.uart'), 'rb').read().decode('latin-1')}


def parse_out(path):
    res = {}
    for line in open(path):
        for tok in line.split():
            if '=' in tok:
                k, v = tok.split('=', 1)
                res[k] = v
    return res


def run(exe, asm, name, plus=(), trace=False):
    d = os.path.join(WORK, 'runs')
    os.makedirs(d, exist_ok=True)
    mem = os.path.join(d, name + '.mem')
    asm.write_mem(mem)
    out = os.path.join(d, name + '.out')
    args = [exe, f'+IMAGE={mem}', f'+OUT={out}', *plus]
    tr = None
    if trace:
        tr = os.path.join(d, name + '.trace')
        args.append(f'+TRACE={tr}')
    if os.path.exists(out):
        os.remove(out)
    r = subprocess.run(args, capture_output=True, text=True, timeout=600)
    if not os.path.exists(out):
        print(r.stdout[-3000:], r.stderr[-3000:])
        return {'end': 'crash', 'log': r.stdout + r.stderr}, tr
    st = parse_out(out)
    st['log'] = r.stdout + r.stderr
    return st, tr


def check(st, exp):
    bad = []
    for k, v in exp.items():
        got = st.get(k)
        if isinstance(v, int):
            if k.startswith('r') or k.startswith('m'):
                want = f'{v & 0xFFFFFFFFFFFFFFFF:016x}'
            elif k in ('pc', 'sp', 'op'):
                want = f'{v:08x}'
            else:
                want = str(v)
        else:
            want = v
        if want == 'nonzero':
            if got is None or int(got, 16) == 0:
                bad.append(f'{k}: got {got} want nonzero')
            continue
        if got != want:
            bad.append(f'{k}: got {got} want {want}')
    return bad


def cmd_test(a):
    import tests
    exe = build(a.rev)
    names = a.names or [n for n in tests.ALL]
    fails = 0
    for n in names:
        fn = tests.ALL[n]
        variants = fn()
        if not isinstance(variants, list):
            variants = [variants]
        for i, (asm, exp, plus) in enumerate(variants):
            tn = n if len(variants) == 1 else f'{n}.{i}'
            st, _ = run(exe, asm, tn, plus)
            bad = check(st, exp)
            if 'fatal' in st.get('log', '').lower() or 'assert' in st.get('log', '').lower():
                bad.append('simulator assertion: ' + st['log'][-500:])
            if bad:
                fails += 1
                print(f'FAIL {tn}: ' + '; '.join(bad))
            else:
                print(f'pass {tn}  (cycles={st.get("cycles")} instrs={st.get("instrs")})')
    print(f'{fails} failing')
    return 1 if fails else 0


def cmd_diff(a):
    import randprog
    base = build(a.base)
    new = build(None)
    fails = 0
    tot = {'base': 0, 'new': 0}
    for s in range(a.seed0, a.seed0 + a.seeds):
        for lat in a.lats:
            asm = randprog.gen(s, **({'irq': True} if a.irq else {}))
            plus = [f'+LAT={lat}'] + ([f'+IRQ_PERIOD={97 + s % 50}'] if a.irq else [])
            sb, tb = run(base, asm, f'rb{s}_{lat}', plus, trace=True)
            sn, tn = run(new, asm, f'rn{s}_{lat}', plus, trace=True)
            same = sb.get('end') == 'park' and sb.get('kind') == '0' and open(tb).read() == open(tn).read()
            if a.irq:
                # IRQ timing legitimately shifts with cycle counts: compare
                # the final state of the main program instead of the trace
                keys = [f'r{i}' for i in range(14)] + ['end', 'kind', 'pc'] + \
                       [k for k in sb if k.startswith('m') and k != 'm00180000']
                same = sb.get('end') == 'park' and all(sb.get(k) == sn.get(k) for k in keys) \
                       and all(sb.get(k) == sn.get(k) for k in sn if k.startswith('m') and k != 'm00180000')
            if 'fatal' in (sb['log'] + sn['log']).lower():
                same = False
            tot['base'] += int(sb.get('cycles', 0)); tot['new'] += int(sn.get('cycles', 0))
            tag = 'same' if same else 'DIFF'
            if not same:
                fails += 1
            if not same or a.verbose:
                print(f'{tag} seed={s} lat={lat} cycles {sb.get("cycles")} -> {sn.get("cycles")} '
                      f'instrs {sb.get("instrs")}/{sn.get("instrs")} end {sb.get("end")}/{sn.get("end")}')
    print(f'{fails} differing; total cycles base={tot["base"]} new={tot["new"]} '
          f'({100.0 * (tot["new"] - tot["base"]) / max(1, tot["base"]):+.2f}%)')
    return 1 if fails else 0


def cmd_socdiff(a):
    """random programs through the full SoC (real cache / bus splitter):
    base rev vs working tree, retire traces must be identical"""
    import randprog
    base = build_soc(a.base)
    new = build_soc(None)
    fails = 0
    tb = tn = 0
    for sd in range(a.seed0, a.seed0 + a.seeds):
        asm = randprog.gen(sd)
        rb = run_soc(base, asm, f'sd_b{sd}')
        rn = run_soc(new, asm, f'sd_n{sd}')
        okb = 'HALTED' in rb['log']
        same = okb and open(rb['trace']).read() == open(rn['trace']).read()
        t = [int(re.search(r't=(\d+)', r['log']).group(1)) if 't=' in r['log'] else 0 for r in (rb, rn)]
        tb += t[0]; tn += t[1]
        if not same:
            fails += 1
        if not same or a.verbose:
            print(f'{"same" if same else "DIFF"} seed={sd} cycles {t[0]} -> {t[1]}  {rb["log"].strip().splitlines()[-2][:60]}')
    print(f'{fails} differing; PIPE_RUN cycles base={tb} new={tn} ({100.0 * (tn - tb) / max(1, tb):+.2f}%)')
    return 1 if fails else 0


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build'); b.add_argument('--rev')
    t = sub.add_parser('test'); t.add_argument('names', nargs='*'); t.add_argument('--rev')
    d = sub.add_parser('diff'); d.add_argument('--base', required=True)
    d.add_argument('--seeds', type=int, default=20); d.add_argument('--seed0', type=int, default=1)
    d.add_argument('--lats', type=int, nargs='+', default=[3])
    d.add_argument('--irq', action='store_true'); d.add_argument('-v', '--verbose', action='store_true')
    sd = sub.add_parser('socdiff'); sd.add_argument('--base', required=True)
    sd.add_argument('--seeds', type=int, default=10); sd.add_argument('--seed0', type=int, default=1)
    sd.add_argument('-v', '--verbose', action='store_true')
    a = p.parse_args()
    if a.cmd == 'build':
        print(build(a.rev))
    elif a.cmd == 'test':
        sys.exit(cmd_test(a))
    elif a.cmd == 'socdiff':
        sys.exit(cmd_socdiff(a))
    else:
        sys.exit(cmd_diff(a))


if __name__ == '__main__':
    main()

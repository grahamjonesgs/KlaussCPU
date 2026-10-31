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
    tag = rev or 'wt'
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
    h = hashlib.sha1((srcs['klauss_pkg.sv'] + core + tb).encode()).hexdigest()
    exe = os.path.join(d, 'obj', 'Vtb_vsim')
    stamp = os.path.join(d, 'stamp')
    if os.path.exists(exe) and os.path.exists(stamp) and open(stamp).read() == h:
        return exe
    for n, s in srcs.items():
        open(os.path.join(d, n), 'w').write(s)
    defs = []
    if 'perf_ic_miss' in core:
        defs.append('+define+HAS_IC_MISS')
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


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build'); b.add_argument('--rev')
    t = sub.add_parser('test'); t.add_argument('names', nargs='*'); t.add_argument('--rev')
    d = sub.add_parser('diff'); d.add_argument('--base', required=True)
    d.add_argument('--seeds', type=int, default=20); d.add_argument('--seed0', type=int, default=1)
    d.add_argument('--lats', type=int, nargs='+', default=[3])
    d.add_argument('--irq', action='store_true'); d.add_argument('-v', '--verbose', action='store_true')
    a = p.parse_args()
    if a.cmd == 'build':
        print(build(a.rev))
    elif a.cmd == 'test':
        sys.exit(cmd_test(a))
    else:
        sys.exit(cmd_diff(a))


if __name__ == '__main__':
    main()

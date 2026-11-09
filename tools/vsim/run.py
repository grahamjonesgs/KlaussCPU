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
    noexc = os.environ.get('VSIM_EXC_EN') == '0'     # elaborate EXC_EN=0 (core 2's MMU-less core)
    tag = (rev or 'wt') + ('_dbg' if dbg else '') + ('_noexc' if noexc else '')
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
    h = hashlib.sha1((srcs['klauss_pkg.sv'] + core + tb + str(dbg) + str(noexc)).encode()).hexdigest()
    exe = os.path.join(d, 'obj', 'Vtb_vsim')
    stamp = os.path.join(d, 'stamp')
    if os.path.exists(exe) and os.path.exists(stamp) and open(stamp).read() == h:
        return exe
    for n, s in srcs.items():
        open(os.path.join(d, n), 'w').write(s)
    defs = ['-GDEBUG_EN=1'] if dbg else []
    if 'parameter bit EXC_EN' in core:
        defs.append('+define+HAS_EXC')
        if noexc:
            defs.append('-GEXC_EN=0')
    if 'm_rdata_dram' in core:
        defs.append('+define+HAS_RDATA_DRAM')
    if 'input  logic [63:0]  h_dword' in core:
        defs.append('+define+HAS_DCH')
    if 'ras_miss' in core:
        defs.append('+define+HAS_RAS')
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
             'uart_rx_fifo.sv', 'sd_spi.sv', 'qspi_ctrl.sv', 'crypto_aes.sv', 'crypto_sha.sv', 'aes_core.sv',
             'aes_sbox.sv', 'ghash.sv', 'sha256_core.sv', 'trng.sv', 'blitter_dma.sv',
             'eth_mmio_bridge.sv', 'Seven_seg_LED_Display_Controller.sv', 'SPI_Master.sv',
             'SPI_Master_With_Single_CS.sv', 'RGB_LED.sv', 'vga_timing.sv', 'vga_scanout.sv',
             'vga_ctrl.sv', 'uart_tx.sv', 'KlaussCPU.sv']
ODDR_STUB = '''module STARTUPE2 #(parameter PROG_USR = "FALSE", parameter real SIM_CCLK_FREQ = 0.0)
   (output CFGCLK, output CFGMCLK, output EOS, output PREQ, input CLK, input GSR, input GTS,
    input KEYCLEARB, input PACK, input USRCCLKO, input USRCCLKTS, input USRDONEO, input USRDONETS);
   assign EOS = 1'b1; assign CFGCLK = 1'b0; assign CFGMCLK = 1'b0; assign PREQ = 1'b0;
endmodule
module ODDR #(parameter DDR_CLK_EDGE = "SAME_EDGE", parameter INIT = 1'b0, parameter SRTYPE = "SYNC")
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
    present = []
    for n in names:
        p = os.path.join(SRC, n)
        try:
            t = git_show(rev, p) if rev else open(p).read()
        except subprocess.CalledProcessError:
            continue        # newer than `rev` (e.g. qspi_ctrl.sv on an old base)
        present.append(n)
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
           '-Wno-MULTIDRIVEN', '-Wno-BLKANDNBLK', '-Wno-UNOPTFLAT', '--assert', '--top-module', 'tb_soc',
           '-I' + src, '-Mdir', os.path.join(d, 'obj')] + \
          [os.path.join(src, n) for n in SOC_FILES if n in present] + \
          [os.path.join(src, 'tb_soc.sv'), os.path.join(src, 'unisim_stubs.v')]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-4000:], r.stderr[-4000:])
        raise SystemExit(f'verilator SoC build failed ({tag})')
    open(stamp, 'w').write(h)
    return exe


def run_soc(exe, image, name, maxi=0, timeout=1800, trace=False):
    """image: an Asm (assembled into a boot image) or a .mem path"""
    d = os.path.join(WORK, 'socruns', name)
    os.makedirs(d, exist_ok=True)
    mem = os.path.join(d, 'netboot.mem')
    if isinstance(image, str):
        shutil.copy(image, mem)
    else:
        image.write_boot_mem(mem)
    # The retire trace is ~400 B per instruction: only when asked for (a
    # long program's trace once filled the disk).
    tr = f'{d}/soc.trace' if trace else '/dev/null'
    args = [exe, f'+TRACE={tr}', f'+UARTF={d}/soc.uart'] + ([f'+MAXI={maxi}'] if maxi else [])
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
        if got is None and k.startswith('m') and int(want, 16) == 0:
            continue                      # the TB dumps only non-zero dwords
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
            st, _ = run(exe, asm, tn, list(plus) + a.plus)
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
            plus = [f'+LAT={lat}'] + ([f'+IRQ_PERIOD={97 + s % 50}'] if a.irq else []) + a.plus
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


def _exc_variant(seed, user, p_inj, irq=False, **kw):
    """MMU stage 1: the random program `seed` with exception ops injected
    before random instructions (SYSCALL, TRAP, an illegal word, and in user
    mode privileged ops), run from a user-mode (or supervisor) start, with a
    handler that returns past each one.  Its final state must equal the plain
    program's.  Returns (asm, number injected)."""
    import random, randprog
    from kasm import Asm, u32
    from tests import _xsetup, _to_user, HALT_W
    rnd = random.Random(seed * 7919 + 1)
    words = [0x6C080000, 0x6C040000, 0x7C000000] + \
            ([0x6C060000, 0x6C070000, 0x6C401000, 0x6C011000, 0x65C00000, 0x6C020000,
              0x6C030000, 0x70000010] if user else [])
    # (0x6C011000 is not HALT: a reserved-field HALT = ILLEGAL; HALT itself
    # ends the program)

    class InjAsm(Asm):
        def __init__(self, *aa, **kk):
            super().__init__(*aa, **kk)
            self.inj = False
            self.n_inj = 0
            _xsetup(self)
            if user:
                # (with IRQs: the timer is unmasked by the IRET frame; the
                # program's own INT_MASK store faults in user mode and is
                # stepped over)
                _to_user(self, 'u', mask=1 if irq else 0)
            self.label('u')
            self.inj = True

        def emit(self, *ws):
            if self.inj and rnd.random() < p_inj:
                self.n_inj += 1
                super().emit(rnd.choice(words))
            return super().emit(*ws)

    old = randprog.Asm
    randprog.Asm = InjAsm
    try:
        asm = randprog.gen(seed, irq=irq, **kw)
    finally:
        randprog.Asm = old
    asm.inj = False
    # the handler: no memory writes outside the kernel stack; a user HALT
    # (PRIVILEGED, EADDR = the HALT word) halts here
    asm.label('h')
    asm.push(13); asm.push(14)
    asm.mfsr(13, 'eaddr'); asm.setr(14, HALT_W)
    asm.fbr('ne', 13, 14, 'h_nh'); asm.halt()
    asm.label('h_nh')
    asm.mfsr(13, 'ecause')
    asm.fbr('eq', 13, 4, 'h_r', imm=True)
    asm.getsp(13); asm.ld(64, 14, 13, 16); asm.addi_s(14, 14, 4); asm.st(64, 14, 13, 16)
    asm.label('h_r')
    asm.pop(14); asm.pop(13)
    asm.iret()
    return asm, asm.n_inj


def cmd_excdiff(a):
    """MMU stage 1: random programs with injected exceptions (user mode and
    supervisor) vs the same programs without: r1-r12, r15 and the data
    window must match"""
    import randprog
    exe = build(None)
    fails = n_tot = 0
    for s in range(a.seed0, a.seed0 + a.seeds):
        for lat in a.lats:
            plus = [f'+LAT={lat}'] + ([f'+IRQ_PERIOD={151 + s % 50}'] if a.irq else []) + a.plus
            sa, _ = run(exe, randprog.gen(s, **({'irq': True} if a.irq else {})), f'xa{s}_{lat}', plus)
            for user in (True, False):
                asm, n = _exc_variant(s, user, a.p, irq=a.irq)
                n_tot += n
                sb, _ = run(exe, asm, f'xb{s}_{lat}_{int(user)}', plus)
                keys = [f'r{i}' for i in list(range(1, 13)) + [15]] + \
                       sorted({k for k in list(sa) + list(sb)
                               if re.fullmatch(r'm[0-9a-f]{8}', k) and k != 'm00180000'})
                bad = [k for k in keys if sa.get(k) != sb.get(k)]
                ok = sa.get('end') == 'park' and sb.get('end') == 'park' and sb.get('kind') == '0' and not bad
                if 'fatal' in (sa['log'] + sb['log']).lower():
                    ok = False
                if not ok:
                    fails += 1
                if not ok or a.verbose:
                    print(f'{"same" if ok else "DIFF"} seed={s} lat={lat} user={user} injected={n} '
                          f'end {sa.get("end")}/{sb.get("end")} kind {sb.get("kind")} {bad[:4]}')
    print(f'{fails} differing; {n_tot} exception ops injected')
    return 1 if fails else 0


def cmd_dmtest(a):
    """tb_dm: the debug module's MMU stage 1 registers (self-checking)"""
    d = os.path.join(WORK, 'dm')
    os.makedirs(d, exist_ok=True)
    cmd = ['verilator', '--binary', '--timing', '-Wno-fatal', '-Wno-lint', '-Wno-style', '--top-module', 'tb_dm',
           '-Mdir', os.path.join(d, 'obj'), os.path.join(SRC, 'klauss_pkg.sv'), os.path.join(SRC, 'debug_module.sv'),
           os.path.join(HERE, 'tb_dm.sv')]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-3000:])
        return 1
    r = subprocess.run([os.path.join(d, 'obj', 'Vtb_dm')], capture_output=True, text=True)
    print(r.stdout)
    return 0 if '0 failing' in r.stdout else 1


def _mmio_prog(seed):
    """MMIO reads/writes (INT_VEC0..3, read back) interleaved with DRAM
    loads, stores and push/pop: back-to-back MMIO <-> DRAM responses"""
    import random
    from kasm import Asm
    rnd = random.Random(seed)
    a = Asm()
    a.setr(12, 0x10000); a.setr(11, 0xF00F0010)
    for i in range(4):
        a.setr(1, 0x1000 + 0x40 * i + seed); a.st(32, 1, 11, 8 * i)
    for i in range(8):
        a.setr64(2, rnd.getrandbits(64)); a.st(64, 2, 12, 8 * i)
    for _ in range(120):
        c = rnd.random()
        r = rnd.randrange(1, 10)
        if c < 0.35:
            a.ld(32, r, 11, 8 * rnd.randrange(4))
        elif c < 0.65:
            a.ld(64, r, 12, 8 * rnd.randrange(8))
        elif c < 0.75:
            a.st(64, r, 12, 8 * rnd.randrange(8))
        elif c < 0.85:
            a.setr(13, 0x1000 + rnd.randrange(64)); a.st(32, 13, 11, 8 * rnd.randrange(4))
        elif c < 0.92:
            a.push(r); a.pop(rnd.randrange(1, 10))
        else:
            a.alu('add', r, r, rnd.randrange(1, 10))
    for i in range(4):
        a.ld(32, 1 + i, 11, 8 * i)
    a.halt()
    return a


def _mpu_variant(seed, p_sys=0.05, irq=False):
    """MMU stage 2: randprog `seed` in user mode under random MPU regions,
    with SYSCALLs injected.  The handler (no memory access: SCRATCH0/1 hold
    r13/r14) turns the MPU off on an ACCESS fault and retries the faulting
    instruction, and turns it on again on a SYSCALL.  So every instruction
    runs exactly once with its full effect, after faults at arbitrary
    points: the final state must equal the plain program's."""
    import random, randprog
    from kasm import Asm, MPU_EN, MPU_R, MPU_W, MPU_X, MPU_G
    from tests import _xsetup, _to_user, _mpu_reg, _mpu_on, HALT_W
    rnd = random.Random(seed * 104729 + 7)
    areas = [(0x0, 0x4000, False), (0x10000, 0x10800, True), (0x07F7F000, 0x07F80000, True)]

    class MpuAsm(Asm):
        def __init__(self, *aa, **kk):
            super().__init__(*aa, **kk)
            self.inj = False
            self.n_inj = 0
            _xsetup(self)
            for n in range(16):
                if rnd.random() < 0.75:
                    lo, hi, guard_ok = rnd.choice(areas)
                    b = rnd.randrange(lo, hi, 32)
                    l = min(hi - 1, b + rnd.choice([0, 32, 64, 256, 1024, 4096]) + rnd.randrange(32))
                    at = MPU_EN | (rnd.random() < 0.7) * MPU_R | (rnd.random() < 0.6) * MPU_W | \
                         (rnd.random() < 0.6) * MPU_X
                    if guard_ok and rnd.random() < 0.15:
                        at |= MPU_G
                    _mpu_reg(self, n, b, l, at)
            _mpu_on(self)
            _to_user(self, 'u', mask=1 if irq else 0)
            self.label('u')
            self.inj = True

        def emit(self, *ws):
            if self.inj and rnd.random() < p_sys:
                self.n_inj += 1
                super().emit(0x6C080000)                  # SYSCALL
            return super().emit(*ws)

    old = randprog.Asm
    randprog.Asm = MpuAsm
    try:
        asm = randprog.gen(seed, irq=irq)
    finally:
        randprog.Asm = old
    asm.inj = False
    asm.label('h')
    asm.mtsr('scratch0', 13); asm.mtsr('scratch1', 14)
    asm.mfsr(13, 'eaddr'); asm.setr(14, HALT_W)
    asm.fbr('ne', 13, 14, 'h_nh'); asm.halt()
    asm.label('h_nh')
    asm.mfsr(13, 'ecause'); asm.alui('and', 13, 13, 0xFF, sgn=False)
    asm.setr(14, 0x100)
    asm.fbr('eq', 13, 4, 'h_set', imm=True)               # SYSCALL: MPU on
    # ACCESS_* on MMIO is the stage 1 user rule (with IRQs, randprog's
    # INT_MASK store): step over it; any other ACCESS_*: MPU off, retry
    asm.mfsr(13, 'eaddr'); asm.shift_n('shr', 13, 13, 28); asm.setr(14, 15)
    asm.fbr('ne', 13, 14, 'h_off')
    asm.getsp(13); asm.ld(64, 14, 13); asm.addi_s(14, 14, 4); asm.st(64, 14, 13)
    asm.jmp('h_ret')
    asm.label('h_off')
    asm.setr(14, 0)
    asm.label('h_set')
    asm.mtsr('status', 14)
    asm.label('h_ret')
    asm.mfsr(13, 'scratch0'); asm.mfsr(14, 'scratch1')
    asm.iret()
    return asm, asm.n_inj


def cmd_mpudiff(a):
    import randprog
    exe = build(None)
    fails = n_exc = 0
    for s in range(a.seed0, a.seed0 + a.seeds):
        for lat in a.lats:
            plus = [f'+LAT={lat}'] + ([f'+IRQ_PERIOD={151 + s % 50}'] if a.irq else []) + a.plus
            sa, _ = run(exe, randprog.gen(s, irq=a.irq), f'pa{s}_{lat}', plus)
            asm, n = _mpu_variant(s, irq=a.irq)
            sb, _ = run(exe, asm, f'pb{s}_{lat}', plus)
            n_exc += int(sb.get('exc_n', 0))
            keys = [f'r{i}' for i in list(range(1, 13)) + [15]] + \
                   sorted({k for k in list(sa) + list(sb) if re.fullmatch(r'm[0-9a-f]{8}', k) and k != 'm00180000'})
            bad = [k for k in keys if sa.get(k) != sb.get(k)]
            ok = sa.get('end') == 'park' and sb.get('end') == 'park' and sb.get('kind') == '0' and not bad
            if 'fatal' in (sa['log'] + sb['log']).lower():
                ok = False
            if not ok:
                fails += 1
            if not ok or a.verbose:
                print(f'{"same" if ok else "DIFF"} seed={s} lat={lat} exc={sb.get("exc_n")} '
                      f'end {sa.get("end")}/{sb.get("end")} kind {sb.get("kind")} {bad[:4]}')
    print(f'{fails} differing; {n_exc} exception entries')
    return 1 if fails else 0


def cmd_socmmio(a):
    """full SoC, base rev vs working tree: MMIO / DRAM interleavings must
    give identical retire traces (the bus_splitter response paths)"""
    base = build_soc(a.base)
    new = build_soc(None)
    fails = 0
    for sd in range(a.seed0, a.seed0 + a.seeds):
        asm = _mmio_prog(sd)
        rb = run_soc(base, asm, f'mm_b{sd}', trace=True)
        rn = run_soc(new, asm, f'mm_n{sd}', trace=True)
        tb_ = open(rb['trace']).read()
        same = 'HALTED' in rb['log'] and 'HALTED' in rn['log'] and tb_ == open(rn['trace']).read()
        if not same:
            fails += 1
            print(f'DIFF seed={sd} lines {len(tb_.splitlines())}/{len(open(rn["trace"]).read().splitlines())}')
    # CPU_FEATURES (0xF00F_0048) on the working tree: 0xFB = stage 1 + LR/SC
    # + the 16-region MPU
    from kasm import Asm
    f = Asm(); f.setr(1, 0xF00F0048); f.ld(64, 2, 1); f.halt()
    rf = run_soc(new, f, 'cpufeat', trace=True)
    last = open(rf['trace']).read().strip().splitlines()[-1]
    if 'r2=00000000000000fb' not in last:
        fails += 1
        print('CPU_FEATURES: ' + last[:200])
    print(f'{fails} differing')
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
        asm = randprog.gen(sd, span=a.span)
        rb = run_soc(base, asm, f'sd_b{sd}', trace=True)
        rn = run_soc(new, asm, f'sd_n{sd}', trace=True)
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
    # extra plusargs for every run, e.g. --plus +HIT=70 (tb_vsim's hit-pipe model)
    for sp in (t,):
        sp.add_argument('--plus', nargs='*', default=[])
    d = sub.add_parser('diff'); d.add_argument('--base', required=True)
    d.add_argument('--seeds', type=int, default=20); d.add_argument('--seed0', type=int, default=1)
    d.add_argument('--lats', type=int, nargs='+', default=[3])
    d.add_argument('--irq', action='store_true'); d.add_argument('-v', '--verbose', action='store_true')
    d.add_argument('--plus', nargs='*', default=[])
    sub.add_parser('dmtest')
    mp = sub.add_parser('mpudiff'); mp.add_argument('--seeds', type=int, default=20)
    mp.add_argument('--seed0', type=int, default=1); mp.add_argument('--lats', type=int, nargs='+', default=[3])
    mp.add_argument('--irq', action='store_true'); mp.add_argument('-v', '--verbose', action='store_true')
    mp.add_argument('--plus', nargs='*', default=[])
    sm = sub.add_parser('socmmio'); sm.add_argument('--base', required=True)
    sm.add_argument('--seeds', type=int, default=20); sm.add_argument('--seed0', type=int, default=1)
    x = sub.add_parser('excdiff'); x.add_argument('--seeds', type=int, default=20)
    x.add_argument('--seed0', type=int, default=1); x.add_argument('--lats', type=int, nargs='+', default=[3])
    x.add_argument('--p', type=float, default=0.08); x.add_argument('--irq', action='store_true')
    x.add_argument('-v', '--verbose', action='store_true')
    x.add_argument('--plus', nargs='*', default=[])
    sd = sub.add_parser('socdiff'); sd.add_argument('--base', required=True)
    sd.add_argument('--seeds', type=int, default=10); sd.add_argument('--seed0', type=int, default=1)
    sd.add_argument('-v', '--verbose', action='store_true')
    sd.add_argument('--span', type=lambda v: int(v, 0), default=0x800,
                    help='data footprint of the random loads/stores (e.g. 64: same-line hazards)')
    a = p.parse_args()
    if a.cmd == 'build':
        print(build(a.rev))
    elif a.cmd == 'test':
        sys.exit(cmd_test(a))
    elif a.cmd == 'socdiff':
        sys.exit(cmd_socdiff(a))
    elif a.cmd == 'excdiff':
        sys.exit(cmd_excdiff(a))
    elif a.cmd == 'dmtest':
        sys.exit(cmd_dmtest(a))
    elif a.cmd == 'mpudiff':
        sys.exit(cmd_mpudiff(a))
    elif a.cmd == 'socmmio':
        sys.exit(cmd_socmmio(a))
    else:
        sys.exit(cmd_diff(a))


if __name__ == '__main__':
    main()

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
import argparse, fnmatch, hashlib, os, re, subprocess, sys, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
SRC = os.path.join(ROOT, 'KlaussCPU.srcs', 'sources_1', 'new')
WORK = os.environ.get('VSIM_WORK', '/tmp/vsim')
sys.path.insert(0, HERE)


def git_show(rev, path):
    rel = os.path.relpath(path, ROOT)
    return subprocess.check_output(['git', '-C', ROOT, 'show', f'{rev}:{rel}'], text=True,
                                   stderr=subprocess.DEVNULL)


def ic_geom():
    """VSIM_IC=<lines>x<line bytes> (HW_PERF_PLAN §5): the I-cache to
    elaborate, e.g. 1024x16 (the 16 KiB I-cache before §5), 1024x32 (32
    KiB), 2048x32 (64 KiB, the core's default); None = the default"""
    v = os.environ.get('VSIM_IC', '')
    if not v:
        return None
    n, b = v.lower().split('x')
    return int(n), int(b)


def build(rev=None, quiet=True):
    dbg = os.environ.get('VSIM_DEBUG_EN') == '1'     # elaborate DEBUG_EN=1
    noexc = os.environ.get('VSIM_EXC_EN') == '0'     # elaborate EXC_EN=0 (core 2's MMU-less core)
    mpu = os.environ.get('VSIM_MPU_EN') == '1'       # elaborate MPU_EN=1 (off on the A7)
    tlbn = os.environ.get('VSIM_TLB_N', '')         # elaborate TLB_N=<n> (default: the core's, 32)
    nowalk = os.environ.get('VSIM_TLB_WALK') == '0'  # elaborate TLB_EN=1, TLB_WALK=0 (MMU_PLAN §11.8: no walker)
    ic = ic_geom()                                   # VSIM_IC=<lines>x<bytes>, e.g. 1024x16 (HW_PERF_PLAN §5)
    nobtic = os.environ.get('VSIM_BTIC_EN') == '0'   # elaborate BTIC_EN=0 (btfuzz's reference)
    tag = (rev or 'wt') + ('_dbg' if dbg else '') + ('_noexc' if noexc else '') + ('_mpu' if mpu else '') + \
          (f'_tlb{tlbn}' if tlbn else '') + ('_nowalk' if nowalk else '') + (f'_ic{ic[0]}x{ic[1]}' if ic else '') + \
          ('_nobtic' if nobtic else '')
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
    h = hashlib.sha1((srcs['klauss_pkg.sv'] + core + tb + str(dbg) + str(noexc) + str(mpu) + tlbn + str(nowalk) +
                      str(ic) + str(nobtic)).encode()).hexdigest()
    exe = os.path.join(d, 'obj', 'Vtb_vsim')
    stamp = os.path.join(d, 'stamp')
    if os.path.exists(exe) and os.path.exists(stamp) and open(stamp).read() == h:
        return exe
    for n, s in srcs.items():
        open(os.path.join(d, n), 'w').write(s)
    defs = ['-GDEBUG_EN=1'] if dbg else []
    if nobtic:
        defs.append('+define+HAS_NOBTIC')
    if 'parameter bit EXC_EN' in core:
        defs.append('+define+HAS_EXC')
        if noexc:
            defs.append('-GEXC_EN=0')
    if mpu and 'parameter bit MPU_EN' in core:
        defs += ['+define+HAS_MPU', '-GMPU_EN=1']
    if tlbn and 'parameter int TLB_N' in core:
        defs += ['+define+HAS_TLBN', f'-GTLB_N={int(tlbn)}']
    if nowalk and 'parameter bit TLB_WALK' in core:
        defs.append('-GTLB_WALK=0')
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
    if 'output logic         m_zl' in core:
        defs.append('+define+HAS_DCBZ')
    if 'parameter bit TLB_WALK' in core and not noexc:
        defs.append('+define+HAS_WALK')
    if 'output logic         perf_itlb' in core:
        defs.append('+define+HAS_PERF_ITLB')
    if 'parameter int IC_LB' in core:
        defs.append('+define+HAS_ICLB')
        if ic:
            defs += [f'-GIC_LINES={ic[0]}', f'-GIC_LB={ic[1]}']
    elif ic and ic != (1024, 16):
        raise SystemExit(f'VSIM_IC={ic[0]}x{ic[1]}: this revision has only the 16 KiB / 16 B I-cache')
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
             'uart_rx_fifo.sv', 'sd_native.sv', 'sd_spi.sv', 'qspi_ctrl.sv', 'crypto_aes.sv', 'crypto_sha.sv', 'aes_core.sv',
             'aes_sbox.sv', 'ghash.sv', 'sha256_core.sv', 'trng.sv', 'blitter_dma.sv',
             'eth_mmio_bridge.sv', 'Seven_seg_LED_Display_Controller.sv', 'SPI_Master.sv',
             'SPI_Master_With_Single_CS.sv', 'RGB_LED.sv', 'vga_timing.sv', 'vga_scanout.sv',
             'vga_ctrl.sv', 'uart_tx.sv', 'ddr2_control.sv', 'l2_cache.sv', 'KlaussCPU.sv']
# HW_PERF_PLAN §6: tb_soc's DDR is the behavioural MIG behind the real
# ddr2_control / l2_cache (older revisions' tb_soc has a behavioural
# ddr2_control instead: then neither ddr2_control.sv nor the MIG model)
SIM_MIG = os.path.join(ROOT, 'KlaussCPU.srcs', 'sim_1', 'new', 'mig_model.sv')
SIM_SDC = os.path.join(ROOT, 'KlaussCPU.srcs', 'sim_1', 'new', 'sd_card_sync.sv')
# a run whose tb_soc consistency checker saw a wrong read answer
CONS_FATAL = 'FATAL consistency'


def _l2bit():
    """CPU_FEATURES[26] (the L2) of the tb_soc build, and [27] (the cache
    range ops, SD DMA spec v1.2: every build of this revision)"""
    return ((1 << 26) if l2_cfg()[0] else 0) | (1 << 27)


def l2_cfg():
    """VSIM_L2=0: build tb_soc without the L2 (CORE1_L2_EN = 0, ddr2_control);
    VSIM_L2_WAYS=<n>: the L2's ways (HW_PERF_PLAN §6)"""
    return os.environ.get('VSIM_L2', '1') != '0', int(os.environ.get('VSIM_L2_WAYS', '4'))
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



# tb_soc probe (sim copy only): what the walker's two reads meet in the
# D-cache (mem_read_write), sampled when each read's DV rises.  One WKEV line
# per read: ph (0 = D, 1 = T), addr, hit (a line of that address is cached),
# dirty (that line is dirty), vdirty (a miss whose LRU victim is dirty),
# shadow (the cache is in an EVICT_SHADOW_* writeback), same (that writeback
# is this very line)
WK_PROBE = r"""
   // ---- walker / D-cache probe (tools/vsim/run.py) ----
   logic wkp_q = 1'b0;
   always @(posedge dut.i_Clk) begin
      wkp_q <= dut.pipeline_core_i.wk && dut.pipeline_core_i.irq_xc && dut.pipeline_core_i.m_read_DV;
      if (dut.pipeline_core_i.wk && dut.pipeline_core_i.irq_xc && dut.pipeline_core_i.m_read_DV && !wkp_q) begin
         automatic logic [31:0] ad = dut.pipeline_core_i.m_addr;
         automatic int ix = ad[14:5];
         automatic logic h0 = dut.mem_read_write.cache_val_addr_way0[ix] == {1'b1, ad[31:15]};
         automatic logic h1 = dut.mem_read_write.cache_val_addr_way1[ix] == {1'b1, ad[31:15]};
         automatic logic lru = dut.mem_read_write.cache_lru[ix];
         automatic logic vd = lru ? dut.mem_read_write.cache_dirty_way1[ix] : dut.mem_read_write.cache_dirty_way0[ix];
         automatic logic sh = (dut.mem_read_write.state == 16'd16) || (dut.mem_read_write.state == 16'd32);
         // (a dirty victim is "in flight" from its miss: the refill, then the writeback)
         automatic logic ev = dut.mem_read_write.r_do_evict && (sh || dut.mem_read_write.state == 16'd4096);
         $display("WKEV ph=%0d addr=%08x hit=%b dirty=%b vdirty=%b shadow=%b same=%b", dut.pipeline_core_i.wk_ph, ad,
                  h0 | h1, (h0 & dut.mem_read_write.cache_dirty_way0[ix]) | (h1 & dut.mem_read_write.cache_dirty_way1[ix]),
                  !(h0 | h1) & vd, sh, ev && dut.mem_read_write.r_evict_ddr_addr_r[31:5] == ad[31:5]);
      end
   end
"""


# tb_soc probe (sim copy only): tb_vsim's BTIC checks (HW_PERF_PLAN §5
# review M1 / review 3) on core 1 — every valid BTIC entry's target line and
# its branch's lines (first and last word: the last word's dword from the
# arm that wrote the entry) are in the I-cache tag shadow (16 B lines: the
# pair banks) or the pending install, at an index the entry's kills compare; TE = 1 through the micro-ITLB as the
# entry is written; a BTIC hit's window is at the ID op's target.  A
# violation prints a FATAL line at the end of the run, which run_soc turns
# into a failure of every command; BTPROBE gives the counts checked.
BTIC_PROBE = r"""
   // ---- BTIC check probe (tools/vsim/run.py) ----
   localparam int BTP_LW = %(lw)d;
   localparam int BTP_TS = %(ts)d;
   longint btp_unc = 0, btp_pau = 0, btp_chk = 0, btp_hits = 0, btp_stale = 0, btp_noarm = 0;
   logic [28:0] btp_pt [0:63], btp_pb [0:63], btp_pl [0:63];
   logic [28:0] btp_vt [0:63], btp_vb [0:63], btp_vl [0:63];
   logic [63:0] btp_pk = '0, btp_mv = '0;
   logic [28:0] btp_mb [0:63];
   logic [29:0] btp_mt [0:63];
   logic [29:0] btp_at [0:1];
   logic [28:0] btp_al [0:1];
   logic [1:0]  btp_av = '0;
   function automatic logic [29:0] btp_pa(input logic [28:0] vdw, output logic ok);
      ok = 1'b1;
      if (!%(c)s.te) return {1'b0, vdw};
      for (int u = 0; u < $bits(%(c)s.ut_v); u++)
         if (%(c)s.ut_v[u] && %(c)s.ut_vpn[u] == vdw[28:13]) return {1'b0, %(c)s.ut_pa[u], vdw[12:0]};
      ok = 1'b0;
      return '0;
   endfunction
   function automatic logic btp_cov(input logic [28:0] p, input logic [28:0] v, input int k);
      automatic int ix = int'(v[BTP_TS-1:BTP_LW]);
      automatic logic [28:0] sh = (BTP_LW == 2) ? 29'(%(c)s.ic_tagsh[ix])
                                : ix[0] ? 29'(%(c)s.ic_tagff_o[ix >> 1]) : 29'(%(c)s.ic_tagff_e[ix >> 1]);
      return (sh == 29'(p[28:BTP_TS]) || (%(c)s.lp_v && %(c)s.lp_pa == p[28:2])) &&
             (ix == int'(%(c)s.bt_lidx[k]) || ix == int'(%(c)s.bt_blidx[k]));
   endfunction
   always @(posedge %(c)s.clk) if (!%(c)s.rst) begin
      for (int k = 0; k < 64; k++) begin
         if (%(c)s.bt_v[k] && (!btp_mv[k] || %(c)s.bt_base[k] != btp_mb[k] || %(c)s.bt_tag[k] != btp_mt[k])) begin
            automatic logic ok_t, ok_b, ok_l;
            automatic logic [28:0] vl = %(c)s.bt_tag[k][29:1];
            automatic logic [29:0] pt, pb, pl;
            if (btp_av[0] && btp_at[0] == %(c)s.bt_tag[k])      vl = btp_al[0];
            else if (btp_av[1] && btp_at[1] == %(c)s.bt_tag[k]) vl = btp_al[1];
            else btp_noarm++;
            pt = btp_pa(%(c)s.bt_base[k], ok_t);
            pb = btp_pa(%(c)s.bt_tag[k][29:1], ok_b);
            pl = btp_pa(vl, ok_l);
            btp_pt[k] = pt[28:0];  btp_pb[k] = pb[28:0];  btp_pl[k] = pl[28:0];
            btp_vt[k] = %(c)s.bt_base[k];  btp_vb[k] = %(c)s.bt_tag[k][29:1];  btp_vl[k] = vl;
            btp_pk[k] = ok_t && ok_b && ok_l;
            if (btp_pk[k]) btp_chk++; else btp_pau++;
         end
         btp_mv[k] = %(c)s.bt_v[k];  btp_mb[k] = %(c)s.bt_base[k];  btp_mt[k] = %(c)s.bt_tag[k];
         if (%(c)s.bt_v[k] && btp_pk[k] &&
             !(btp_cov(btp_pt[k], btp_vt[k], k) && btp_cov(btp_pb[k], btp_vb[k], k) && btp_cov(btp_pl[k], btp_vl[k], k)))
            btp_unc++;
      end
      if (%(c)s.id_valid && %(c)s.id_bp_tk && %(c)s.bt_hit && %(c)s.id_bt_base != %(c)s.id_bp_tgt[31:3]) btp_stale++;
      if (%(c)s.id_early_rd && !%(c)s.start && %(c)s.bt_hit) btp_hits++;
      if (%(c)s.id_early_rd && !%(c)s.start && !%(c)s.bt_hit) begin
         btp_at[1] = btp_at[0];  btp_al[1] = btp_al[0];  btp_av[1] = btp_av[0];
         btp_at[0] = %(c)s.id_pc[31:2];  btp_al[0] = %(c)s.id_last_dw;  btp_av[0] = 1'b1;
      end
   end
   final begin
      $display("BTPROBE btic_chk=%%0d btic_hits=%%0d btic_pau=%%0d", btp_chk, btp_hits, btp_pau);
      if (btp_unc != 0) $display("TB: FATAL BTIC coverage: btic_unc=%%0d (a valid BTIC entry's line not in the tag shadow)", btp_unc);
      if (btp_stale != 0) $display("TB: FATAL BTIC currency: btic_stale=%%0d (a BTIC hit's window not at the branch's target)", btp_stale);
      if (btp_noarm != 0) $display("TB: FATAL BTIC check: btic_noarm=%%0d (an entry written without its arm)", btp_noarm);
   end
"""

BTIC_FAILS = []      # runs whose BTIC coverage check failed (main() exits non-zero)
BTIC_STATS = {'runs': 0, 'chk': 0, 'hits': 0, 'pau': 0}   # what the checks saw (printed at exit)
# a run fails the BTIC gate on: the coverage count, the currency count, an
# entry without its arm (the TB's FATAL lines), or the core's own BTIC
# assertion (it stops the run: no result file)
CONS_FAILS = []
CONS_STATS = {'runs': 0, 'n': 0, 'rt_need': 0}
SOC_BEH = set()      # tb_soc executables of an older tb (its own behavioural DDR: no consistency checker)
SOC_PROBED = set()   # tb_soc executables built with the BTIC probe (a run of one must print BTPROBE)
SOC_L2P = set()      # tb_soc executables built with the L2 drain probe (a run of one must print its line)
BTIC_FATAL = ('FATAL BTIC', 'BTIC base != target', 'pos0 unsound')


def _btic_note(name, st, log, gate=True):
    """HW_PERF_PLAN §5 review M2 / review 3: the BTIC checks of one run.
    gate=False: a base revision's run in an A/B (only the revision under
    test is gated).  Returns True if the run failed them."""
    bad = any(st.get(k, '0') not in ('0', None) for k in ('btic_unc', 'btic_stale', 'btic_noarm')) or \
        any(f in log for f in BTIC_FATAL)
    if gate:
        BTIC_STATS['runs'] += 1
        BTIC_STATS['chk'] += int(st.get('btic_chk', 0))
        BTIC_STATS['hits'] += int(st.get('btic_hits', 0))
        BTIC_STATS['pau'] += int(st.get('btic_pau', 0))
        if bad:
            BTIC_FAILS.append(name)
    return bad and gate


def build_soc(rev=None, crypto_full=False, nowalk=False, tb_wt=False):
    """full SoC (sim_1/new/tb_soc.sv: behavioural DDR, real cache / bus /
    MMIO / HCF) under Verilator.  tb_soc reads netboot.mem from its cwd.
    crypto_full: KlaussCPU with every optional crypto block (the default
    build is the lean A7 one)."""
    ic = ic_geom()
    l2, l2w = l2_cfg()
    tag = (rev or 'wt') + ('_cfull' if crypto_full else '') + ('_nowalk' if nowalk else '') + \
          (f'_ic{ic[0]}x{ic[1]}' if ic else '') + ('' if l2 else '_nol2') + (f'_l2w{l2w}' if l2w != 4 else '') + \
          ('_tbwt' if tb_wt else '') + \
          ''.join(f'_{k}{os.environ[k]}' for k in ('VSIM_L2_SPEC', 'VSIM_L2_XTRA') if k in os.environ)
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
    # tb_wt: the working tree's tb_soc (and MIG model) around rev's RTL — an
    # older revision then runs on the same DDR model as the working tree
    tb = git_show(rev, tbp) if rev and not tb_wt else open(tbp).read()
    beh_ddr = 'module ddr2_control' in tb          # an older tb_soc: its own DDR model
    if beh_ddr:
        present = [n for n in present if n not in ('ddr2_control.sv', 'l2_cache.sv')]
    else:
        mig = git_show(rev, SIM_MIG) if rev and not tb_wt else open(SIM_MIG).read()
        open(os.path.join(src, 'mig_model.sv'), 'w').write(mig)
        allsrc += mig
    # the SD card model (tb_soc instantiates it since the SD DMA work)
    sdc = None
    if 'sd_card_sync' in tb:
        sdc = git_show(rev, SIM_SDC) if rev and not tb_wt else open(SIM_SDC).read()
        open(os.path.join(src, 'sd_card_sync.sv'), 'w').write(sdc)
        allsrc += sdc
    # report the cycle count at HALTED (for real-cache A/B timing)
    tb = tb.replace('$display("TB_SOC: HALTED after %0d retired instructions, %0d UART bytes", icount, uart_n);',
                    '$display("TB_SOC: HALTED after %0d retired instructions, %0d UART bytes, t=%0d", icount, uart_n, runcyc);')
    tb = tb.replace('   longint last_i = -1;', '   longint last_i = -1;\n   longint runcyc = 0;\n'
                    '   always @(posedge dut.i_Clk) if (dut.st.SM == PIPE_RUN) runcyc++;')
    ov = []
    if crypto_full:
        ov.append(".CRYPTO_AES_EN(1'b1), .CRYPTO_GCM_EN(1'b1), .CRYPTO_SHA_EN(1'b1), .CRYPTO_HMAC_EN(1'b1)")
    if nowalk:       # TLB_EN = 1, TLB_WALK = 0 on core 1 (CPU_FEATURES[25] = 0)
        ov.append(".CORE1_TLB_WALK(1'b0)")
    ic = ic_geom()   # VSIM_IC (HW_PERF_PLAN §5): core 1's I-cache geometry
    if ic and 'CORE1_IC_LB' in allsrc:
        ov.append(f'.CORE1_IC_LINES({ic[0]}), .CORE1_IC_LB({ic[1]})')
    if 'CORE1_L2_EN' in allsrc:
        ov.append(f".CORE1_L2_EN(1'b{int(l2)}), .CORE1_L2_WAYS({l2w})")
    elif not l2 and not beh_ddr and not tb_wt:
        raise SystemExit('VSIM_L2=0 on a revision without CORE1_L2_EN')
    elif ic and ic != (1024, 16):
        raise SystemExit(f'VSIM_IC={ic[0]}x{ic[1]}: this revision has only the 16 KiB / 16 B I-cache')
    if ov:
        assert tb.count('KlaussCPU dut (') == 1
        tb = tb.replace('KlaussCPU dut (', 'KlaussCPU #(' + ', '.join(ov) + ') dut (')
    if (rev is None or tb_wt) and 'wk_fin' in allsrc and 'WKEV' not in tb:
        k = tb.rindex('endmodule')
        tb = tb[:k] + WK_PROBE + tb[k:]
    probed = False
    lines, lb = ic if ic else (2048, 32)
    if 'lp_pa' in allsrc and 'ic_tagsh' in allsrc:      # (review 3: both line sizes)
        k = tb.rindex('endmodule')
        lw = 2 if lb == 32 else 1
        tb = tb[:k] + BTIC_PROBE % {'c': 'dut.pipeline_core_i', 'lw': lw, 'ts': (lines - 1).bit_length() + lw} + tb[k:]
        probed = True
    elif rev is None and 'bt_base' in allsrc:
        # the current revision has a BTIC but the signal-name test failed
        # (a renamed signal): a silent skip would make every BTIC check vacuous
        raise SystemExit('tb_soc BTIC probe skipped: lp_pa / ic_tagsh not found in the current sources')
    open(os.path.join(src, 'tb_soc.sv'), 'w').write(tb)
    open(os.path.join(src, 'unisim_stubs.v'), 'w').write(ODDR_STUB)
    h = hashlib.sha1((allsrc + tb).encode()).hexdigest()
    exe = os.path.join(d, 'obj', 'Vtb_soc')
    stamp = os.path.join(d, 'stamp')
    if probed:
        SOC_PROBED.add(exe)
    if beh_ddr:
        SOC_BEH.add(exe)
    if l2 and not beh_ddr and 'CORE1_L2_EN' in allsrc:
        SOC_L2P.add(exe)
    if os.path.exists(exe) and os.path.exists(stamp) and open(stamp).read() == h:
        return exe
    # measurement knobs (HW_PERF_PLAN §6): VSIM_L2_SPEC=0 (4-cycle hits),
    # VSIM_L2_XTRA=<n> (n more cycles a hit)
    kn = [f'+define+L2_SPEC_DEF={os.environ["VSIM_L2_SPEC"]}'] if 'VSIM_L2_SPEC' in os.environ else []
    kn += [f'+define+L2_XTRA_DEF={os.environ["VSIM_L2_XTRA"]}'] if 'VSIM_L2_XTRA' in os.environ else []
    if beh_ddr:
        defs = []
    elif 'CORE1_L2_EN' not in allsrc:
        defs = ['+define+DDRPATH=ddr2_control']          # a pre-L2 revision under the current tb_soc
    else:
        # (L2_PROBE: tb_soc's count of drains in VGA's / DMA's window, review 1 M2)
        defs = ['+define+L2_PROBE'] if l2 else ['+define+DDRPATH=g_ddr.ddr2_control']
        if l2 and 'r_wk_wb' in allsrc:
            defs.append('+define+L2_PROBE_WK')        # (review 2: the walk's drain terms)
    if 'r_grant_sd' in allsrc:
        defs.append('+define+SD_DMA_PROBE')           # (master E: tb_soc counts its transactions)
    cmd = ['verilator', '--binary', '--timing', '-j', '8', '-Wno-fatal', '-Wno-lint', '-Wno-style',
           '-Wno-MULTIDRIVEN', '-Wno-BLKANDNBLK', '-Wno-UNOPTFLAT', '--assert', '--top-module', 'tb_soc',
           '-I' + src, '-Mdir', os.path.join(d, 'obj')] + defs + \
          kn + [os.path.join(src, n) for n in SOC_FILES if n in present] + \
          ([] if beh_ddr else [os.path.join(src, 'mig_model.sv')]) + \
          ([os.path.join(src, 'sd_card_sync.sv')] if sdc else []) + \
          [os.path.join(src, 'tb_soc.sv'), os.path.join(src, 'unisim_stubs.v')]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-4000:], r.stderr[-4000:])
        raise SystemExit(f'verilator SoC build failed ({tag})')
    open(stamp, 'w').write(h)
    return exe


def run_soc(exe, image, name, maxi=0, timeout=1800, trace=False, gate=True, plus=()):
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
    args = [exe, f'+TRACE={tr}', f'+UARTF={d}/soc.uart'] + ([f'+MAXI={maxi}'] if maxi else []) + \
        list(plus or ()) + [x for x in os.environ.get('VSIM_SOC_PLUS', '').split() if x]
    r = subprocess.run(args, capture_output=True, text=True, cwd=d, timeout=timeout)
    st = {}
    m = re.search(r'BTPROBE btic_chk=(\d+) btic_hits=(\d+) btic_pau=(\d+)', r.stdout)
    if m:
        st = {'btic_chk': m.group(1), 'btic_hits': m.group(2), 'btic_pau': m.group(3)}
    elif exe in SOC_PROBED and gate and r.returncode == 0 and 'HALTED' in r.stdout:
        # the probe was built in but printed nothing: its checks did not run
        BTIC_FAILS.append(name + ' (no BTPROBE line)')
    if _btic_note(name, st, r.stdout + r.stderr, gate):
        print(f'FAIL {name}: BTIC check (tb_soc probe)')
    if gate and CONS_FATAL in r.stdout + r.stderr:
        # HW_PERF_PLAN §6: the memory-consistency checker (L2 + DDR = memory)
        CONS_FAILS.append(name)
        print(f'FAIL {name}: memory consistency (tb_soc checker)')
    m = re.search(r'consistency checks=(\d+)', r.stdout)
    if gate and exe not in SOC_BEH and not (m and int(m.group(1)) > 0):
        # review 1: a gated run whose checker checked nothing (no HALTED
        # report, or zero answers) is not a pass
        CONS_FAILS.append(name + (' (0 checks)' if m else ' (no consistency line)'))
        print(f'FAIL {name}: consistency checker {"checked nothing" if m else "did not report"}')
    if m and gate:
        CONS_STATS['runs'] += 1
        CONS_STATS['n'] += int(m.group(1))
    m = re.search(r'L2 rt drains bad=(\d+) need=(\d+)', r.stdout)
    if gate and exe in SOC_L2P and not m:
        # review 2: an L2-on gated run whose drain probe did not report
        CONS_FAILS.append(name + ' (no L2 rt drains line)')
        print(f'FAIL {name}: L2 drain probe did not report')
    if m and gate:
        CONS_STATS['rt_need'] += int(m.group(2))
        if int(m.group(1)):
            # review 1 M2: a drain started in VGA's / DMA's window with no
            # request waiting on the buffer
            CONS_FAILS.append(name + f' ({m.group(1)} drains in the VGA/DMA window)')
            print(f'FAIL {name}: {m.group(1)} L2 drains started in the VGA / DMA window')
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


def run(exe, asm, name, plus=(), trace=False, gate=True):
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
        _btic_note(name, {}, r.stdout + r.stderr, gate)
        return {'end': 'crash', 'log': r.stdout + r.stderr}, tr
    st = parse_out(out)
    st['log'] = r.stdout + r.stderr
    _btic_note(name, st, st['log'], gate)
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
        if isinstance(v, tuple):          # (lo, hi): an inclusive range (rN / mN: of the hex value)
            if got is None or not (v[0] <= int(got, 16 if k[0] in 'rm' else 10) <= v[1]):
                bad.append(f'{k}: got {got} want {v[0]}..{v[1]}')
            continue
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
    import fnmatch
    names = [n for n in tests.ALL if not a.names or any(fnmatch.fnmatch(n, g) for g in a.names)]
    if a.names and not names:
        raise SystemExit('no test matches ' + ' '.join(a.names))
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
    hits = {'base': 0, 'new': 0}      # BTIC hits (tb_vsim's count; a base without it: 0)
    for s in range(a.seed0, a.seed0 + a.seeds):
        for lat in a.lats:
            asm = randprog.gen(s, **({'irq': True} if a.irq else {}))
            plus = [f'+LAT={lat}'] + ([f'+IRQ_PERIOD={97 + s % 50}'] if a.irq else []) + a.plus
            sb, tb = run(base, asm, f'rb{s}_{lat}', plus, trace=True, gate=False)
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
            hits['base'] += int(sb.get('btic_hits', 0)); hits['new'] += int(sn.get('btic_hits', 0))
            tag = 'same' if same else 'DIFF'
            if not same:
                fails += 1
            if not same or a.verbose:
                print(f'{tag} seed={s} lat={lat} cycles {sb.get("cycles")} -> {sn.get("cycles")} '
                      f'instrs {sb.get("instrs")}/{sn.get("instrs")} end {sb.get("end")}/{sn.get("end")}')
    print(f'{fails} differing; total cycles base={tot["base"]} new={tot["new"]} '
          f'({100.0 * (tot["new"] - tot["base"]) / max(1, tot["base"]):+.2f}%); '
          f'BTIC hits base={hits["base"]} new={hits["new"]}')
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


WK_PTB = 0x300000                  # tlbdiff --walk: the directory (PA) ...
WK_TABS = (0x302000, 0x304000)     # ... and the PTE tables for D[0] / D[1]


def _tlb_variant(seed, p_sys=0.04, irq=False, k_force=0, walk=False, p_absent=0.0, span=0x800):
    """MMU stage 3: randprog `seed` in user mode with TE = 1, its code run
    through a non-identity VA (page CVP -> frame 0), starting from an empty
    TLB.  A kseg refill handler at TLBVEC maps CVP -> 0 and every other
    page to itself, round-robin over K entries (K = 2 thrashes; K = 1
    would livelock, as any load needs its code page and its data page).  Injected
    SYSCALLs bump ASID (the non-global entries stop matching) and TLBINV on
    its wrap.  The handler runs a TLBP first, which must miss.  With IRQs, the TB's handler at 0x0010_0000 runs translated
    in supervisor mode and is refilled the same way.  Every instruction
    runs once with its full effect, so the final state must equal the
    plain physical run's.

    walk (MMU_PLAN §11.8): page tables in memory (CVP -> frame 0, every
    other page below 0x0800_0000 to itself, a fraction p_absent of them
    not PRESENT) and the hardware walker on; the software handler stays
    for the walker's fallbacks (absent PTEs)."""
    import random, randprog
    from kasm import Asm
    from tests import _tlb_set, HALT_W, KSP, USP, TLB_RWX, TLB_N
    rnd = random.Random(seed * 7919 + 3)
    # (with IRQs every ~150-200 cycles, K = 2 starves: the handler's two
    # pages evict the interrupted instruction's two before it can retire)
    K = rnd.choice([4, 16] if irq else [2, 4, 16])
    if k_force:                      # --k: every seed over K entries (up to TLB_N)
        K = k_force
    CVP = rnd.choice([0x0040, 0x0123, 0x0400, 0x07F0])
    CNT = 0x80200000                                       # kseg: the round-robin index

    class TlbAsm(Asm):
        def __init__(self, *aa, **kk):
            super().__init__(*aa, **kk)
            self.inj = False
            self.n_inj = 0
            self.leapc(1, 'h'); self.setr(2, 0x80000000); self.alu('or', 1, 1, 2)
            self.mtsr('evec', 1); self.mtsr('tlbvec', 1)
            self.setr(1, KSP); self.setsp(1)
            _tlb_set(self, 0, 0x0000, 0x000, TLB_RWX, g=1)
            if walk:
                self.setr(1, WK_PTB); self.mtsr('tlb_ptb', 1)
                self.setr(1, 1); self.mtsr('tlb_walk', 1)  # EN, RR = 0
            self.setr(1, 0x200); self.mtsr('status', 1)  # TE (through the identity entry)
            self.leapc(1, 'kz'); self.setr(2, 0x80000000); self.alu('or', 1, 1, 2); self.jmpr(1)
            self.label('kz')                                # now in kseg
            self.tlbinv()
            self.setr(1, USP); self.mtsr('usp', 1)
            self.leapc(2, 'u'); self.alui('and', 2, 2, 0xFFFF, sgn=False)
            self.setr(3, CVP << 16); self.alu('or', 2, 2, 3)
            self.setr64(3, (1 << 44) | ((1 if irq else 0) << 39)); self.alu('or', 2, 2, 3)
            self.push(2); self.iret()
            self.label('u')
            self.inj = True

        def emit(self, *ws):
            if self.inj and rnd.random() < p_sys:
                self.n_inj += 1
                super().emit(0x6C080000)                  # SYSCALL
            return super().emit(*ws)

    old = randprog.Asm
    randprog.Asm = TlbAsm
    try:
        asm = randprog.gen(seed, irq=irq, span=span)
    finally:
        randprog.Asm = old
    asm.inj = False
    asm.label('h')                                      # runs at its kseg alias
    asm.mtsr('scratch0', 13); asm.mtsr('scratch1', 14)
    asm.mfsr(13, 'eaddr'); asm.setr(14, HALT_W)
    asm.fbr('ne', 13, 14, 'h_nh'); asm.halt()
    asm.label('h_nh')
    asm.mfsr(13, 'ecause'); asm.alui('and', 13, 13, 0xFF, sgn=False)
    asm.fbr('eq', 13, 4, 'h_sys', imm=True)
    asm.addi_s(14, 13, -11); asm.fbr('ule', 14, 2, 'h_miss', imm=True)
    # ACCESS_* on MMIO (randprog's INT_MASK store, with IRQs): step over;
    # anything else is a bug: stop here (the state then differs)
    asm.mfsr(13, 'eaddr'); asm.shift_n('shr', 13, 13, 28); asm.setr(14, 15)
    asm.fbr('ne', 13, 14, 'h_bad')
    asm.getsp(13); asm.ld(64, 14, 13); asm.addi_s(14, 14, 4); asm.st(64, 14, 13)
    asm.jmp('h_ret')
    asm.label('h_bad')
    asm.halt()
    asm.label('h_miss')                                 # TLB_HI = {VPN, G 0, ASID}
    asm.tlbp(); asm.mfsr(13, 'tlb_index')               # (a TLBP must miss: P = 1)
    asm.shift_n('shr', 13, 13, 31); asm.fbr('ne', 13, 1, 'h_bad', imm=True)
    asm.mfsr(13, 'tlb_hi'); asm.shift_n('shr', 13, 13, 16)
    asm.setr(14, CVP); asm.fbr('ne', 13, 14, 'h_id')
    asm.setr(13, 0)
    asm.label('h_id')
    asm.shift_n('shl', 13, 13, 16); asm.alui('or', 13, 13, TLB_RWX, sgn=False)
    asm.mtsr('tlb_lo', 13)
    asm.setr(13, CNT); asm.ld(64, 14, 13); asm.inc(14)
    asm.alui('and', 14, 14, K - 1, sgn=False); asm.st(64, 14, 13)
    asm.mtsr('tlb_index', 14); asm.tlbwi()
    asm.jmp('h_ret')
    asm.label('h_sys')                                  # ASID + 1 (mod 4); TLBINV on the wrap
    asm.mfsr(13, 'asid'); asm.inc(13); asm.alui('and', 13, 13, 3, sgn=False)
    asm.mtsr('asid', 13)
    asm.fbr('ne', 13, 0, 'h_ret', imm=True)
    asm.tlbinv()
    asm.label('h_ret')
    asm.mfsr(13, 'scratch0'); asm.mfsr(14, 'scratch1')
    asm.iret()
    n_abs = 0
    if walk:
        for d, t in enumerate(WK_TABS):
            asm.dword(WK_PTB + 8 * d, 0xA5A50000F8001FFE | t | 1)
        for vp in range(0x800):
            absent = rnd.random() < p_absent
            n_abs += absent
            pte = 0x5A5A0000F800FFE0 | ((0 if vp == CVP else vp) << 16) | (0 if absent else 0x10) | TLB_RWX
            asm.dword(WK_TABS[vp >> 10] + 8 * (vp & 0x3FF), pte)
    return asm, (K, CVP, asm.n_inj) + ((n_abs,) if walk else ())


def cmd_tlbdiff(a):
    import randprog
    exe = build(None)
    fails = n_exc = n_wk = 0
    for s in range(a.seed0, a.seed0 + a.seeds):
        for lat in a.lats:
            plus = [f'+LAT={lat}'] + ([f'+IRQ_PERIOD={151 + s % 50}'] if a.irq else []) + a.plus
            sa, _ = run(exe, randprog.gen(s, irq=a.irq, span=a.span), f'ta{s}_{lat}', plus)
            asm, info = _tlb_variant(s, irq=a.irq, k_force=a.k, walk=a.walk, p_absent=a.absent, span=a.span)
            sb, _ = run(exe, asm, f'tb{s}_{lat}', plus)
            n_exc += int(sb.get('exc_n', 0))
            n_wk += int(sb.get('wk_n', 0))
            keys = [f'r{i}' for i in list(range(1, 13)) + [15]] + \
                   sorted({k for k in list(sa) + list(sb) if re.fullmatch(r'm[0-9a-f]{8}', k) and k != 'm00180000'})
            bad = [k for k in keys if sa.get(k) != sb.get(k)]
            ok = sa.get('end') == 'park' and sb.get('end') == 'park' and sb.get('kind') == '0' and not bad
            if 'fatal' in (sa['log'] + sb['log']).lower():
                ok = False
            if not ok:
                fails += 1
            if not ok or a.verbose:
                print(f'{"same" if ok else "DIFF"} seed={s} lat={lat} K,CVP,sys[,absent]={info} '
                      f'exc={sb.get("exc_n")} walks={sb.get("wk_n")}/{sb.get("wk_s")} '
                      f'cycles {sa.get("cycles")}/{sb.get("cycles")} end {sa.get("end")}/{sb.get("end")} '
                      f'kind {sb.get("kind")} {bad[:4]}')
    print(f'{fails} differing; {n_exc} exception entries; {n_wk} walker refills')
    return 1 if fails else 0


def cmd_socmmio(a):
    """full SoC, base rev vs working tree: MMIO / DRAM interleavings must
    give identical retire traces (the bus_splitter response paths)"""
    base = build_soc(a.base)
    new = build_soc(None)
    fails = 0
    for sd in range(a.seed0, a.seed0 + a.seeds):
        asm = _mmio_prog(sd)
        rb = run_soc(base, asm, f'mm_b{sd}', trace=True, gate=False)
        rn = run_soc(new, asm, f'mm_n{sd}', trace=True)
        tb_ = open(rb['trace']).read()
        same = 'HALTED' in rb['log'] and 'HALTED' in rn['log'] and tb_ == open(rn['trace']).read()
        if not same:
            fails += 1
            print(f'DIFF seed={sd} lines {len(tb_.splitlines())}/{len(open(rn["trace"]).read().splitlines())}')
    # CPU_FEATURES (0xF00F_0048) on the working tree: 0x03F0_200D = stage 1
    # + LR/SC + the 32-entry TLB, no MPU, lean crypto (TRNG only), native SD,
    # DCBZ, STATUS.MAL64, ETH_RX_SLOTS, the TLB walker ([25]): the A7
    from kasm import Asm
    f = Asm(); f.setr(1, 0xF00F0048); f.ld(64, 2, 1); f.halt()
    rf = run_soc(new, f, 'cpufeat', trace=True)
    last = open(rf['trace']).read().strip().splitlines()[-1]
    l2, l2w = l2_cfg()
    if f'r2=00000000{0x0FF0200D if l2 else 0x0BF0200D:08x}' not in last:
        fails += 1
        print('CPU_FEATURES: ' + last[:200])
    # CACHE_INFO (0xF005_0008) and ICACHE_INFO (0xF005_0018, HW_PERF_PLAN
    # §5: 1 way, 2048 sets, 32 B lines, 64 KiB — or VSIM_IC's geometry);
    # L2_INFO 0xF005_0020 and L2_CTRL 0xF005_0028 (§6; 0 without the L2)
    f = Asm(); f.setr(1, 0xF0050000)
    f.ld(64, 2, 1, 8); f.ld(64, 3, 1, 0x18); f.ld(64, 4, 1, 0x20); f.ld(64, 5, 1, 0x28); f.halt()
    rf = run_soc(new, f, 'icinfo', trace=True)
    last = open(rf['trace']).read().strip().splitlines()[-1]
    n, b = ic_geom() or (2048, 32)
    ici = f'{n * b:08x}{b:02x}{n:04x}01'
    l2i = f'0002000040{128 * 1024 // 64 // l2w:04x}{l2w:02x}' if l2 else '0' * 16
    for want in ('r2=0001000020040002', f'r3={ici}', f'r4={l2i}', f'r5={int(l2):016x}'):
        if want not in last:
            fails += 1
            print(f'CACHE_INFO / ICACHE_INFO: want {want}: ' + last[:200])
    print(f'{fails} differing')
    return 1 if fails else 0


def _crypto_prog():
    """the optional crypto windows: AES CTR0 and GCM_H0 read back what was
    written, AES_CAP / SHA_CAP, HMAC_STATUS after a KEY_LOAD (KEY_VALID),
    CPU_FEATURES; an absent block reads 0 and ignores writes, without
    stalling the bus (the program must reach its HALT)"""
    from kasm import Asm
    a = Asm()
    a.setr(10, 0xF00A0000); a.setr(11, 0xF00B0000)
    a.setr64(1, 0x1122334455667788); a.st(64, 1, 10, 0xC0); a.ld(64, 1, 10, 0xC0)   # AES CTR0
    a.setr(2, 0x99); a.st(64, 2, 10, 0x90); a.ld(64, 2, 10, 0x90)                   # GCM_H0
    a.ld(64, 3, 10, 0xF8)                                                            # AES_CAP
    a.ld(64, 4, 11, 0xF8)                                                            # SHA_CAP
    a.setr(5, 0x5A); a.st(64, 5, 11, 0x90)                                           # HMAC_KEY0
    a.setr(5, 1); a.st(64, 5, 11, 0x80)                                              # KEY_LOAD
    a.setr(7, 400)
    a.label('w'); a.ld(64, 5, 11, 0x88); a.dec(7); a.fbr('ne', 7, 0, 'w', imm=True)  # HMAC_STATUS
    a.setr(12, 0xF00F0048); a.ld(64, 6, 12)                                          # CPU_FEATURES
    for i in range(16):                                                              # more traffic
        a.st(64, 1, 10 + (i & 1), 8 * i); a.ld(64, 8, 10 + (i & 1), 8 * i)
    a.halt()
    return a


def cmd_soccrypto(a):
    """lean (the A7 default) and full crypto configurations: absent windows
    read 0 and ignore writes, present ones work, CPU_FEATURES[20:16]"""
    fails = 0
    for full in (False, True):
        exe = build_soc(None, crypto_full=full)
        r = run_soc(exe, _crypto_prog(), 'crypto_' + ('full' if full else 'lean'), trace=True)
        last = open(r['trace']).read().strip().splitlines()[-1]
        regs = dict(re.findall(r'(r\d+)=([0-9a-f]{16})', last))
        got = {k: int(regs.get(k, '0'), 16) for k in ('r1', 'r2', 'r3', 'r4', 'r5', 'r6')}
        if full:
            want = {'r1': 0x1122334455667788, 'r2': 0x99, 'r3': 3, 'r4': 3, 'r6': 0x03FF200D | _l2bit()}
            ok = all(got[k] == v for k, v in want.items()) and (got['r5'] & 2)
        else:
            want = {'r1': 0, 'r2': 0, 'r3': 0, 'r4': 0, 'r5': 0, 'r6': 0x03F0200D | _l2bit()}
            ok = all(got[k] == v for k, v in want.items())
        ok = ok and 'HALTED' in r['log']
        fails += not ok
        print(f'{"pass" if ok else "FAIL"} {"full" if full else "lean"}: ' +
              ' '.join(f'{k}={v:x}' for k, v in got.items()))
    print(f'{fails} failing')
    return 1 if fails else 0


def _socsd_progs():
    """the native SD controller's MMIO through the real SoC bus (sd_native.sv;
    SD_NATIVE_PLAN.md): reset values, CPU_FEATURES[21], INT2_EN / INT2_PENDING,
    FIFO push and pop-on-read with back-to-back loads, and the source-2
    cascade.  No card: the controller stays in SPI mode, where every SDM_*
    register but SDM_CMD works (§4.8)."""
    from kasm import Asm
    SD, IRQ = 0xF0000000, 0xF00F0000
    progs = []
    a = Asm(); a.setr(15, SD); a.setr(14, IRQ)
    a.ld(64, 1, 14, 0x48); a.ld(64, 2, 14, 0x50); a.ld(64, 3, 14, 0x58)
    a.ld(64, 4, 15, 0x1F8); a.ld(64, 5, 15, 0x120); a.ld(64, 6, 15, 0x138)
    a.ld(64, 7, 15, 0x000); a.ld(64, 8, 15, 0x128); a.ld(64, 9, 15, 0x148)   # r8 SDM_STATUS: DAT0 driven low (slot off at reset), CARD_PRESENT 1 (tb_soc ties SD_CD = 0 = card in)
    a.halt()
    progs.append(('regs', a, {'r1': 0x03F0200D | _l2bit(), 'r2': 1, 'r3': 0, 'r4': 0x00080501, 'r5': 0x00010200,
                              'r6': (12500000 << 32) | 2500000, 'r7': 0x000100F9, 'r8': 0x8, 'r9': 0}))
    a = Asm(); a.setr(15, SD)
    a.setr(9, 0x80000000); a.st(64, 9, 15, 0x148)                      # FIFO reset
    for k in range(8):
        a.setr64(13, 0x1111111111111111 * (k + 1)); a.st(64, 13, 15, 0x180)
    a.ld(64, 12, 15, 0x148)                                            # level 8
    for k in range(8):
        a.ld(64, k + 1, 15, 0x180)                                     # back-to-back pops
    a.ld(64, 9, 15, 0x148)                                             # level 0
    a.ld(64, 10, 15, 0x180)                                            # pop from empty: 0
    a.halt()
    exp = {f'r{k + 1}': 0x1111111111111111 * (k + 1) for k in range(8)}
    exp.update({'r12': 8, 'r9': 0, 'r10': 0})
    progs.append(('fifo', a, exp))
    a = Asm(); a.setr(15, SD); a.setr(14, IRQ)
    a.setr(1, 1 << 26); a.st(64, 1, 15, 0x130)                         # SDM_IRQ_EN = FIFO_ERR
    a.setr(1, 0x80000000); a.st(64, 1, 15, 0x148)
    a.setr(7, 513)
    a.label('p'); a.st(64, 7, 15, 0x180); a.dec(7); a.fbr('ne', 7, 0, 'p', imm=True)
    a.ld(64, 10, 15, 0x148)                                            # 512
    a.ld(64, 2, 14, 0x58)                                              # INT2_PENDING = 0b10
    a.ld(64, 3, 14, 0x08)                                              # INT_PENDING[2] = 0 (INT2_EN = 01)
    a.setr(4, 2); a.st(64, 4, 14, 0x50)                                # INT2_EN = SD
    a.nop(); a.nop(); a.nop()
    a.ld(64, 5, 14, 0x08)                                              # INT_PENDING[2] = 1
    a.ld(64, 6, 14, 0x50)                                              # 2
    a.st(64, 1, 15, 0x148)                                             # FIFO reset (not the error)
    a.setr(7, 1 << 26); a.st(64, 7, 15, 0x128)                         # W1C FIFO_ERR
    a.nop(); a.nop(); a.nop()
    a.ld(64, 8, 14, 0x58)                                              # 0
    a.ld(64, 9, 14, 0x08)                                              # INT_PENDING[2] = 0
    a.halt()
    progs.append(('cascade', a, {'r10': 512, 'r2': 2, 'r3': ('bit2', 0), 'r5': ('bit2', 1), 'r6': 2,
                                 'r8': 0, 'r9': ('bit2', 0)}))
    return progs


def _soc_final(log):
    """tb_soc's HALTED report: final registers, the card's counters, the
    D-cache probes, the maintenance walks / RANGE_IINV sweeps"""
    m = re.search(r'TB_SOC: REGS(.*)', log)
    regs = {k: int(v, 16) for k, v in re.findall(r'(r\d+)=([0-9a-f]{16})', m.group(1))} if m else None
    sdc = dict((k, int(v)) for k, v in re.findall(r'(\w+)=(\d+)', (re.search(r'TB_SOC: SDC (.*)', log) or [''])[0]))
    dcp = {int(k): (int(v), int(d)) for k, v, d in re.findall(r'DCP(\d) [0-9a-f]+ valid=(\d) dirty=(\d)', log)}
    walks = re.findall(r'TB_SOC: WALK (\w+) sets=(\d+) dirty_wb=(\d+) cycles=(\d+)', log)
    sweeps = re.findall(r'TB_SOC: RKSWEEP lines=(\d+) cycles=(\d+)', log)
    return regs, sdc, dcp, walks, sweeps


def cmd_socsddma(a):
    """SD DMA spec v1.2 through the SoC (tools/vsim/sddma.py): DMA reads and
    writes through mem_read_write's arbiter and the L2 against tb_soc's card,
    the (m3) ordering without the MNT_BUSY poll, the range operations'
    coherence (partial lines, straddling lines, LEN = 0, the queue, the
    combinations), and the walk timings.  Every run is also under tb_soc's
    memory-consistency checker (DMA transactions included)."""
    import sddma
    exe = build_soc(None)
    fails = 0
    for name, asm, exp, plus, chk in sddma.tests():
        if a.only and not any(fnmatch.fnmatch(name, g) for g in a.only):
            continue
        r = run_soc(exe, asm, 'sddma_' + name, plus=plus + a.plus, timeout=7200)
        regs, sdc, dcp, walks, sweeps = _soc_final(r['log'])
        bad = []
        if regs is None:
            bad.append('no HALTED')
        else:
            for k, v in exp.items():
                if regs.get(k) != v:
                    bad.append(f'{k}={regs.get(k) if regs.get(k) is None else hex(regs[k])} want {v:#x}')
        for k, v in chk.items():
            if k == 'dcp':
                for i, want in v.items():
                    if dcp.get(i) != want:
                        bad.append(f'DCP{i} (valid, dirty)={dcp.get(i)} want {want}')
            elif k == 'walks':
                got = [w[0] for w in walks]
                if got != v:
                    bad.append(f'walks {got} want {v}')
            elif sdc.get(k) != v:
                bad.append(f'card {k}={sdc.get(k)} want {v}')
        fails += bool(bad)
        tim = ' '.join(f'{w[0]}({w[1]} sets, {w[2]} wb)={w[3]}c' for w in walks[:6])
        print(f'{"pass" if not bad else "FAIL"} {name} {" ".join(bad)}  '
              f'[dma wr={sdc.get("sd_dma_wr")} rd={sdc.get("sd_dma_rd")} card rd={sdc.get("rd_blocks")} '
              f'wr={sdc.get("wr_blocks")}] {tim} '
              + ' '.join(f'iinv({x[0]} lines)={x[1]}c' for x in sweeps))
        if not bad and not a.keep:
            shutil.rmtree(os.path.join(WORK, 'socruns', 'sddma_' + name), ignore_errors=True)
    print(f'{fails} failing')
    return 1 if fails else 0


def cmd_socsd(a):
    exe = build_soc(None)
    fails = 0
    for name, asm, exp in _socsd_progs():
        r = run_soc(exe, asm, 'socsd_' + name, trace=True)
        last = open(r['trace']).read().strip().splitlines()[-1] if 'HALTED' in r['log'] else ''
        regs = {k: int(v, 16) for k, v in re.findall(r'(r\d+)=([0-9a-f]{16})', last)}
        bad = []
        for k, v in exp.items():
            got = regs.get(k)
            ok = got is not None and (((got >> 2) & 1) == v[1] if isinstance(v, tuple) else got == v)
            if not ok:
                bad.append(f'{k}={got if got is None else hex(got)} want {v if isinstance(v, tuple) else hex(v)}')
        fails += bool(bad) or not last
        print(f'{"pass" if not bad and last else "FAIL"} {name} ' + ' '.join(bad))
    print(f'{fails} failing')
    return 1 if fails else 0


# --------------------------------------------- HW_PERF_PLAN.md §1: DCBZ
def _dcbz_variant(seed, use_dcbz, p=0.08, span=0x100, csum=False):
    """randprog `seed` (data span `span`) with line-zeroing groups injected
    before random instructions.  use_dcbz: `r14 = line; r13 = 0; r13 = EA;
    DCBZ [r13]`; else the reference `r14 = line; r13 = 0; 4 x st r13 ->
    [r14 + 8i]; r13 = EA`.  Both leave the same registers and memory, so
    the final states must match.  csum: before the HALT, fold the data
    window into r13 (for runs that only report registers: tb_soc)."""
    import random, randprog
    from kasm import Asm
    rnd = random.Random(seed * 31337 + 11)

    class DzAsm(Asm):
        def __init__(self, *aa, **kk):
            super().__init__(*aa, **kk)
            self.inj = True
            self.n_inj = 0

        def emit(self, *ws):
            if self.inj and rnd.random() < p:
                self.inj = False
                line = randprog.DATA + rnd.randrange(0, span, 32)
                if rnd.random() < 0.3:      # same D-cache set, other tags: evictions
                    line += 0x8000 * rnd.randrange(1, 4)
                ea = line + rnd.randrange(32)
                self.setr(0, 0)
                if use_dcbz:
                    self.setr(0, ea); self.dcbz(0)
                else:
                    for i in range(4):
                        self.st_abs(64, 0, line + 8 * i)
                    self.setr(0, ea)
                # often a load of the line right behind it (into a data
                # register, so a stale value spreads into the final state)
                if rnd.random() < 0.6:
                    sz = rnd.choice([8, 16, 32, 64])
                    self.ld_abs(sz, rnd.randrange(1, 11), line + rnd.randrange(0, 32 // (sz // 8)) * (sz // 8))
                self.n_inj += 1
                self.inj = True
            return super().emit(*ws)

        def halt(self):
            if csum:
                self.inj = False
                self.setr(13, 0)
                for o in range(0, 0x800, 8):
                    self.ld(64, 14, 12, o, mode='long')
                    self.alu('xor', 13, 13, 14)
                    self.shift_n('rol', 13, 13, 7)
            return super().halt()

    old = randprog.Asm
    randprog.Asm = DzAsm
    try:
        asm = randprog.gen(seed, span=span)
    finally:
        randprog.Asm = old
    return asm, asm.n_inj


def cmd_dcbzdiff(a):
    """core TB: DCBZ programs vs their 4-store references (same final
    registers, flags and data window), over latencies and hit-pipe rates"""
    exe = build(None)
    fails = n_inj = n_dyn = 0
    plus_sets = [['+LAT=1'], ['+LAT=3', '+HIT=70'], ['+LAT=7', '+HIT=100'], ['+LAT=2', '+LATR=6', '+HIT=40']]
    for sd in range(a.seed0, a.seed0 + a.seeds):
        for pi, plus in enumerate(plus_sets):
            span = (0x40, 0x100, 0x800)[(sd + pi) % 3]
            ra, n = _dcbz_variant(sd, True, span=span)
            rb, _ = _dcbz_variant(sd, False, span=span)
            n_inj += n
            sa, _ = run(exe, ra, f'za{sd}_{pi}', plus)
            sb, _ = run(exe, rb, f'zb{sd}_{pi}', plus)
            keys = [f'r{i}' for i in range(16)] + ['sp', 'flags', 'end', 'kind'] + \
                   sorted({k for k in list(sa) + list(sb) if re.fullmatch(r'm[0-9a-f]{8}', k)})
            bad = [k for k in keys if sa.get(k) != sb.get(k)]
            # (zero_lines counts executed DCBZs: loops repeat / skip groups)
            ok = sa.get('end') == 'park' and sa.get('kind') in ('0', '5') and not bad and \
                 (n == 0 or int(sa.get('zero_lines', 0)) > 0) and sb.get('zero_lines') == '0' and \
                 'fatal' not in (sa['log'] + sb['log']).lower()
            n_dyn += int(sa.get('zero_lines', 0))
            fails += not ok
            if not ok or a.verbose:
                print(f'{"same" if ok else "DIFF"} seed={sd} {" ".join(plus)} span={span:#x} dcbz={n} '
                      f'zl={sa.get("zero_lines")} cycles {sa.get("cycles")}/{sb.get("cycles")} {bad[:4]}')
    print(f'{fails} differing; {n_inj} DCBZs injected, {n_dyn} executed')
    return 1 if fails else 0


def _btfuzz_prog(seed):
    """HW_PERF_PLAN §5 review M1 / review 3: random code over I-cache
    aliases.  4-8 small functions in slots 64 KiB apart (and 128 B apart
    within a region), so many share an index; bodies mix a signature add
    to r1 (the patch target, the function's first word), tight loops inside
    one window, a load (it waits behind a pending line answer), calls to
    higher-numbered functions, and snoop-coherent stores rewriting a
    function's signature.  Review 3: branch SITES — a forward branch over
    1-4 signature adds, 1-word (JMP, or an FBR on r5 = DATA_BASE: always /
    never taken) or 2-word (JMP abs, or a 2-word FBR whose word 1 is the
    same address), placed at random, at a 32 B or a 16 B line's last word
    (a 2-word site then crosses into the next line) or in a tight loop —
    and stores that patch them: a site's target (its offset, or the JMP
    abs's address word) and its kind (unconditional / conditional, taken /
    not), from anywhere or right before the site runs (the site, fetched
    and predicted before the store, is squashed and refetched).  A 2-word
    site's address word names a label in its skip or a far trampoline
    (ZBASE: another line, at the functions' indices) back to its end.
    Deterministic: its result must not depend on timing or on the
    BTIC."""
    import random
    from kasm import Asm
    from tests import _word
    rnd = random.Random(seed)
    add = lambda c: _word(lambda t: t.alui_s('add', 1, 1, c))
    # 1-word site words (position-independent: the offset is in words)
    def w1(kind, k):             # k: the target is the site + 4 k (1 <= k <= n + 1)
        t = Asm(base=0)
        if kind == 'jmp':
            t.jmp(4 * k)
        else:
            t.fbr('ne' if kind == 'fbr_t' else 'eq', 5, 0, 4 * k, imm=True)
        return t.assemble()[0]
    def w0_2(kind, k):           # word 0 of a 2-word site (target the site + 4 k, 2 <= k <= n + 2)
        t = Asm(base=0)
        if kind == 'jabs':
            t.jmp_abs(0)
        else:
            t.fbr_imm32('ne' if kind == 'fbr32_t' else 'eq', 5, 0, 4 * k)
        return t.assemble()[0]
    while True:
        slots = [0x21000 + j * 0x10000 + o * 0x80 for j in range(3) for o in range(4)]
        rnd.shuffle(slots)
        nf = rnd.randint(4, 8)
        fa = slots[:nf]                            # function k at fa[k]
        body = {}
        sites = []                                 # (func, n, words, skip)
        for k in range(nf):
            ops = [('sig', rnd.randint(1, 9))]
            for _ in range(rnd.randint(1, 5)):
                r = rnd.random()
                if r < 0.15:
                    ls = None
                    if rnd.random() < 0.4:             # a site in the loop
                        ls = (len(sites), rnd.choice((1, 2)), rnd.randint(1, 3))
                        sites.append((k, ls[1], ls[2]))
                    ops.append(('loop', rnd.randint(1, 12), ls))
                elif r < 0.3 and k + 1 < nf:
                    ops.append(('call', rnd.randrange(k + 1, nf)))
                elif r < 0.4:
                    ops.append(('ld',))
                elif r < 0.55:
                    ops.append(('patch', rnd.randrange(nf), rnd.randint(1, 9)))
                elif r < 0.75:
                    sw = rnd.choice((1, 2))
                    sk = rnd.randint(1, 4)
                    al = rnd.choice((None, 28, 12, 24, 4))
                    if rnd.random() < 0.35:            # patched right before it runs (an SMC squash)
                        ops.append(('bpatch', len(sites)))
                    ops.append(('site', len(sites), sw, sk, al))
                    sites.append((k, sw, sk))
                elif r < 0.9:
                    ops.append(('bpatch',))            # (resolved below: needs the site list)
                else:
                    ops.append(('add', rnd.randint(1, 9)))
            body[k] = ops

        def tgt2(n, sk):
            """a 2-word site's address word: a label in its skip, or a far
            trampoline (Z<n>_<m>: another line, the straddle class)"""
            if rnd.random() < 0.5:
                return f'Z{n}_{rnd.randint(0, 1)}'
            return f'S{n}_t{rnd.randint(2, sk + 2)}'

        def bpatch(a, n=None):
            """a store (or two) rewriting a site (n, else a random one): target and / or kind"""
            if not sites:
                return
            if n is None:
                n = rnd.randrange(len(sites))
            _, sw, sk = sites[n]
            if sw == 1:
                kind = rnd.choice(('jmp', 'fbr_t', 'fbr_n'))
                a.setr(3, w1(kind, rnd.randint(1, sk + 1))); a.leapc(4, f'S{n}'); a.st(32, 3, 4)
            else:
                what = rnd.random()
                if what < 0.6:                     # the address word (JMP abs's target)
                    a.leapc(3, tgt2(n, sk)); a.leapc(4, f'S{n}'); a.st(32, 3, 4, 4)
                if what >= 0.3:                    # the kind (and an FBR's offset)
                    kind = rnd.choice(('jabs', 'fbr32_t', 'fbr32_n'))
                    a.setr(3, w0_2(kind, rnd.randint(2, sk + 2))); a.leapc(4, f'S{n}'); a.st(32, 3, 4)

        def site(a, n, sw, sk, al):
            if al is not None:
                while a.pc % 32 != al:
                    a.nop()
            a.label(f'S{n}')
            if sw == 1:
                a.raw(w1(rnd.choice(('jmp', 'fbr_t', 'fbr_n')), rnd.randint(1, sk + 1)))
                for j in range(sk):
                    a.label(f'S{n}_t{j + 1}')
                    a.alui_s('add', 1, 1, 1 << j)
                a.label(f'S{n}_t{sk + 1}')
            else:
                kind = rnd.choice(('jabs', 'fbr32_t', 'fbr32_n'))
                a.raw(w0_2(kind, rnd.randint(2, sk + 2)))
                tl = tgt2(n, sk)
                a.emit(lambda pc, tl=tl: a.labels[tl])
                for j in range(sk):
                    a.label(f'S{n}_t{j + 2}')
                    a.alui_s('add', 1, 1, 1 << j)
                a.label(f'S{n}_t{sk + 2}')

        a = Asm()
        a.setr(1, 0); a.setr(5, DATA_BASE)
        for _ in range(rnd.randint(3, 6)):
            for _ in range(rnd.randint(2, 5)):
                a.call(f'F{rnd.randrange(nf)}')
            if rnd.random() < 0.5:
                a.setr(3, add(rnd.randint(1, 9))); a.setr(4, fa[rnd.randrange(nf)]); a.st(32, 3, 4)
            if rnd.random() < 0.5:
                bpatch(a)
        a.halt()
        fit = True
        for k in sorted(range(nf), key=lambda k: fa[k]):
            if a.pc > fa[k]:
                fit = False                        # the previous body ran into this slot: again
                break
            while a.pc < fa[k]:
                a.nop()
            a.label(f'F{k}')
            for n, op in enumerate(body[k]):
                if op[0] in ('sig', 'add'):
                    a.alui_s('add', 1, 1, op[1])
                elif op[0] == 'ld':
                    a.ld(64, 6, 5); a.alui_s('add', 6, 6, 1)
                elif op[0] == 'call':
                    a.call(f'F{op[1]}')
                elif op[0] == 'patch':
                    a.setr(3, add(op[2])); a.setr(4, fa[op[1]]); a.st(32, 3, 4)
                elif op[0] == 'bpatch':
                    bpatch(a, *op[1:])
                elif op[0] == 'site':
                    site(a, *op[1:])
                elif op[0] == 'loop':
                    a.setr(2, op[1])
                    while a.pc % 8:
                        a.nop()
                    lb = f'F{k}_l{n}'
                    a.label(lb)
                    a.alui_s('add', 1, 1, 1)
                    if op[2]:
                        site(a, *op[2], None)
                    a.addi_s(2, 2, -1)
                    a.fbr('ne', 2, 0, lb, imm=True)
            a.ret()
        # the 2-word sites' far targets: back to the end of the site's skip
        if fit and a.pc <= ZBASE:
            while a.pc < ZBASE:
                a.nop()
            for n, (_, sw, sk) in enumerate(sites):
                if sw == 2:
                    for m in range(2):
                        a.label(f'Z{n}_{m}')
                        a.alui_s('add', 1, 1, 16 << m)
                        a.jmp(f'S{n}_t{sk + 2}')
        if fit and all(f'S{n}' in a.labels for n in range(len(sites))):
            return a, fa


DATA_BASE = 0x10000
ZBASE = 0x51040          # btfuzz: the 2-word sites' far targets


def cmd_btfuzz(a):
    """HW_PERF_PLAN §5 review M1: _btfuzz_prog seeds under several line /
    memory timings, against the same program on a BTIC_EN = 0 build (same
    final registers and data window), with tb_vsim's BTIC coverage check
    (btic_unc = 0, the global gate)"""
    old = os.environ.get('VSIM_BTIC_EN')
    os.environ['VSIM_BTIC_EN'] = '0'
    try:
        ref = build(None)
    finally:
        if old is None:
            del os.environ['VSIM_BTIC_EN']
        else:
            os.environ['VSIM_BTIC_EN'] = old
    exe = build(a.rev)
    plus_sets = [['+LAT=1'], ['+LAT=3', '+LDLY=3'], ['+LAT=2', '+LDLY=8'], ['+LAT=1', '+LDLY=1', '+LDLYR=12'],
                 ['+LAT=4', '+LDLYR=20'], ['+LAT=2', '+LDLY=5', '+HIT=50'], ['+LAT=1', '+LDLY=16']]
    fails = n_unc = 0
    for sd in range(a.seed0, a.seed0 + a.seeds):
        prog, _ = _btfuzz_prog(sd)
        sr, _ = run(ref, prog, 'bfr', ['+LAT=1'])        # (fixed names: ~1 MB images)
        keys = [f'r{i}' for i in range(16)] + ['end', 'kind'] + \
               sorted(k for k in sr if re.fullmatch(r'm[0-9a-f]{8}', k))
        for pi, plus in enumerate(plus_sets):
            st, _ = run(exe, prog, f'bf_{pi}', plus)
            bad = [k for k in keys if st.get(k) != sr.get(k)]
            unc = int(st.get('btic_unc', 0))
            n_unc += unc > 0
            ok = sr.get('end') == 'park' and sr.get('kind') == '0' and not bad and unc == 0 and \
                'fatal' not in (st['log'] + sr['log']).lower()
            fails += not ok
            if not ok or a.verbose:
                print(f'{"same" if ok else "DIFF"} seed={sd} {" ".join(plus)} r1={st.get("r1")}/{sr.get("r1")} '
                      f'btic_unc={unc} cycles={st.get("cycles")} {bad[:4]}')
    print(f'{fails} failing ({n_unc} with btic_unc > 0)')
    return 1 if fails else 0


def _soc_regs(r):
    """registers of the last retire line (the HALT) of a tb_soc run"""
    if 'HALTED' not in r['log']:
        return None
    last = open(r['trace']).read().strip().splitlines()[-1]
    return {k: int(v, 16) for k, v in re.findall(r'(r\d+)=([0-9a-f]{16})', last)}


def _soc_ddr(r):
    m = re.search(r'DDR reads=(\d+) writes=(\d+)', r['log'])
    return (int(m.group(1)), int(m.group(2))) if m else None


def _dz_cnt_prog(dz=True, tail=True):
    """the SoC DCBZ program: set 0 (index 0) holds A (dirty) and B (dirty,
    MRU); DCBZ C (miss: A is the victim, written back, no fetch); loads of
    C read 0; DCBZ C again (hit); the counter deltas; A read back from DDR.
    dz=False: each DCBZ is a NOP (same code layout); tail=False: HALT right
    after the delay that follows the first DCBZ."""
    from kasm import Asm
    A, B, C = 0x200000, 0x208000, 0x210000
    a = Asm()
    a.setr(15, 0xF0050000)
    a.setr(10, A); a.setr(11, B); a.setr(12, C + 13)
    a.setr64(1, 0x1111222233334444); a.st(64, 1, 10, 8)       # write miss: A dirty
    a.setr64(2, 0x5555666677778888); a.st(64, 2, 11, 16)      # write miss: B dirty, MRU
    a.ld(64, 3, 10, 8)                                        # (hit pipe: no LRU change)
    def delay(n):
        a.setr(9, n); a.label(f'd{len(a.labels)}'); lb = f'd{len(a.labels) - 1}'
        a.dec(9); a.fbr('ne', 9, 0, lb, imm=True)
    delay(40)
    for i, off in enumerate((0x48, 0x58, 0x60, 0x70, 0x50)):   # r4..r8 = counters before
        a.ld(64, 4 + i, 15, off)
    dzat = a.dcbz(12) if dz else a.nop()
    if not tail:
        delay(60)
        a.halt()
        return a, dzat
    a.ld(64, 9, 12, 24 - 13, mode='long')                     # C+24 right behind it
    a.ld(64, 13, 12, -13, mode='long')                        # C+0
    a.alu('or', 9, 9, 13)
    a.setr(12, C + 1)
    a.dcbz(12) if dz else a.nop()                             # hit
    delay(60)
    for i, off in enumerate((0x48, 0x58, 0x60, 0x70, 0x50)):  # deltas
        a.ld(64, 13, 15, off); a.alu('sub', 4 + i, 13, 4 + i)
    a.ld(64, 14, 10, 8)                                       # A: a miss, from DDR
    a.ld(64, 13, 11, 16)                                      # B: still cached
    a.halt()
    return a, dzat


def cmd_socdcbz(a):
    """DCBZ through the full SoC (real cache, bus splitter, behavioural DDR)"""
    from kasm import Asm
    exe = build_soc(None)
    fails = 0

    def report(name, ok, msg=''):
        nonlocal fails
        fails += not ok
        print(f'{"pass" if ok else "FAIL"} {name} {msg}')

    # counters, zeros, the victim through DDR, the retire trace line
    prog, dzat = _dz_cnt_prog()
    r = run_soc(exe, prog, 'dz_cnt', trace=True)
    g = _soc_regs(r)
    want = {'r9': 0, 'r4': 0, 'r5': 1, 'r6': 1, 'r7': 2, 'r8': 1,
            'r14': 0x1111222233334444, 'r13': 0x5555666677778888}
    bad = {k: hex(g.get(k, -1)) for k in want if g is None or g.get(k) != want[k]} if g else {'halt': 0}
    tl = [l for l in open(r['trace']) if f'pc={dzat:08x}' in l]
    tr_ok = len(tl) == 1 and ' wr=00210000/ff/0000000000000000' in tl[0]
    report('counters', not bad and tr_ok, f'{bad} trace={tl[0].split(" wr=")[-1].strip() if tl else None}')
    # DDR traffic: the DCBZ miss (dirty victim) adds one line write and no
    # read, against the same program with the DCBZ a NOP
    r0 = run_soc(exe, _dz_cnt_prog(False, False)[0], 'dz_ref')
    r1 = run_soc(exe, _dz_cnt_prog(True, False)[0], 'dz_one')
    d0, d1 = _soc_ddr(r0), _soc_ddr(r1)
    report('ddr', d0 is not None and d1 is not None and d1[0] == d0[0] and d1[1] == d0[1] + 1,
           f'ref={d0} dcbz={d1}')
    # a code line zeroed and patched through the D-cache: the patch runs
    # (the I-cache copies of both halves were snooped)
    for half in (0, 16):
        pa = Asm()
        new_w = Asm(); new_w.inc(2); new_w.ret()
        w = new_w.assemble()
        pa.setr(1, 0); pa.setr(2, 0)
        pa.leapc(3, 'F')
        pa.call('F'); pa.call('F')
        pa.dcbz(3)
        pa.setr64(4, (w[1] << 32) | w[0]); pa.st(64, 4, 3)
        pa.call('F')
        pa.halt()
        pa.nop()
        while (pa.pc & 31) != 0:
            pa.nop()
        for _ in range(half // 4):
            pa.nop()
        pa.label('F')
        pa.inc(1); pa.ret()
        rr = run_soc(exe, pa, f'dz_smc{half}', trace=True)
        g = _soc_regs(rr)
        report(f'smc{half}', g is not None and g.get('r1') == 2 and g.get('r2') == 1,
               f'r1={g and g.get("r1")} r2={g and g.get("r2")}')
    # random DCBZ programs vs their 4-store references (checksummed window)
    for sd in range(a.seed0, a.seed0 + a.seeds):
        span = (0x40, 0x100, 0x800)[sd % 3]
        ra, n = _dcbz_variant(sd, True, span=span, csum=True)
        rb, _ = _dcbz_variant(sd, False, span=span, csum=True)
        ga = _soc_regs(run_soc(exe, ra, f'dz_ra{sd}', trace=True))
        gb = _soc_regs(run_soc(exe, rb, f'dz_rb{sd}', trace=True))
        report(f'random seed={sd} span={span:#x} dcbz={n}', ga is not None and ga == gb,
               '' if ga == gb else str({k: (hex(ga[k]), hex(gb[k])) for k in ga if ga[k] != gb.get(k)}))
    print(f'{fails} failing')
    return 1 if fails else 0



# ---- the walker through the real D-cache (tb_soc): MMU_PLAN §11.8 ----------
SW_PTB, SW_TA, SW_TOLD = 0x224000, 0x232000, 0x234000      # directory, new table, stale table
SW_NEW, SW_OLD = 0x28, 0x29                                # frames (PA 0x28_0000 / 0x29_0000)
SW_K = 0x80000000


def _sw_prog(kind, pad=0):
    """one SoC walker program.  Tables: D[2] and D[3] -> the stale table TOLD
    (PTE 0 / 5 -> frame OLD, whose data is stale), the image's TA table has
    PTE 5 -> NEW (kinds s3t) or is absent.  Registers at HALT:
      dirty : D[3] -> TA and the PTEs rewritten by kseg stores (D and T lines
              dirty in the D-cache when walked; the image holds only the
              stale ones): r4 = 0x600D (NEW), r5 = 0x7777
      vict  : the sets of D's and T's lines hold two dirty lines each (A / B):
              the walk reads miss with a dirty victim; the victims are then
              read back from DDR (r6..r9)
      s3t   : the PTE line is dirty (PTE rewritten to NEW), evicted by two
              other lines of its set, then walked at once: its read meets the
              eviction writeback in flight
      s3d   : the same for the directory line (D[3] rewritten to TA)"""
    from kasm import Asm, TLB_V, TLB_R, TLB_W
    import tests
    from tests import _te_on, _wk_on, _wk_pte, WK_JD
    pte = lambda fr: _wk_pte(fr, TLB_V | TLB_R | TLB_W)
    ptb = 0x242010 if kind == 's3t' else SW_PTB      # s3t: D[3] shares a cache set with the PTE line
    a = Asm()
    # the image can hold 256 KiB only: the page tables and data are stored
    # (physical, TE = 0) and written back to DDR (CACHE_CTRL INVALIDATE: the
    # D-cache is then empty and clean), then the program starts
    def init(addr, val):
        a.setr(8, addr); a.setr64(3, val); a.st(64, 3, 8)
    dirs = {2: WK_JD | SW_TOLD | 1, 3: WK_JD | SW_TOLD | 1}
    if kind != 'dirty':
        dirs[3] = WK_JD | SW_TA | 1
    for i, v in dirs.items():
        init(ptb + 8 * i, v)
    init(SW_TOLD + 8 * 5, pte(SW_OLD)); init(SW_TOLD + 8 * 0, pte(SW_OLD))
    init(SW_TA + 8 * 5, pte(SW_NEW if kind in ('vict', 's3d') else SW_OLD))
    init((SW_NEW << 16) | 8, 0x600D); init((SW_OLD << 16) | 8, 0xDEAD)
    init((SW_NEW << 16) | 0x10, 0x7777); init((SW_OLD << 16) | 0x10, 0xBAD2)
    a.setr(8, 0xF0050000); a.setr(3, 4); a.st(64, 3, 8)              # INVALIDATE
    a.label('pl'); a.ld(64, 3, 8, 0x10); a.fbr('ne', 3, 0, 'pl', imm=True)    # CACHE_STATUS.MNT_BUSY
    _te_on(a)
    _wk_on(a, 8, 1, ptb=ptb)
    a.setr(1, 0); a.mtsr('tlb_wcnt', 1)
    def st64(addr, val, ra=8, rv=3):
        a.setr(ra, SW_K | addr); a.setr64(rv, val); a.st(64, rv, ra)
    def ldk(addr, rd, ra=11):
        a.setr(ra, SW_K | addr); a.ld(64, rd, ra)
    if kind == 'dirty':
        st64(SW_PTB + 8 * 3, WK_JD | SW_TA | 1)
        st64(SW_TA + 8 * 5, pte(SW_NEW))
        st64(SW_TOLD + 8 * 0, pte(SW_NEW))
        a.setr(9, 0x0C050008); a.ld(64, 4, 9)
        a.setr(9, 0x08000010); a.ld(64, 5, 9)
    elif kind == 'vict':
        vals = [0x1111000000000001, 0x2222000000000002, 0x3333000000000003, 0x4444000000000004]
        for ad, v in zip((0x204018, 0x20C018, 0x202028, 0x20A028), vals):
            st64(ad, v)
        a.setr(9, 0x0C050008); a.ld(64, 4, 9)
        for rd, ad in zip((6, 7, 8, 9), (0x204018, 0x20C018, 0x202028, 0x20A028)):
            ldk(ad, rd)
    elif kind in ('s3t', 's3d'):
        # sub: two loads (r11, r12: the evicting misses), pad NOPs, then the
        # access that walks (r9).  Called once beforehand with harmless
        # operands so that its code is in the I-cache (no fetch fills in the
        # window), then for real.
        a.setr(11, SW_K | 0x300040); a.setr(12, SW_K | 0x308040); a.setr(9, 0x00000008)    # (a mapped page: no walk)
        a.call('sub')
        if kind == 's3t':
            # the PTE line P (set 0x101) dirty and LRU behind one more line X
            # of its set; D[3] is in that set too: the walk's D read misses,
            # evicts P (refill first, writeback in the shadow), and the T read
            # of P comes right behind it
            st64(SW_TA + 8 * 5, pte(SW_NEW))
            a.setr(11, SW_K | 0x20A028); a.setr(12, SW_K | 0x20A028)
        else:
            st64(ptb + 8 * 3, WK_JD | SW_TA | 1)  # the directory line dirty
            a.setr(11, SW_K | 0x20C018); a.setr(12, SW_K | 0x214018)
        a.setr(9, 0x0C050008)
        a.call('sub')
        a.jmp('end')
        a.label('sub')
        a.ld(64, 6, 11); a.ld(64, 7, 12)
        for _ in range(pad):
            a.nop()
        a.ld(64, 4, 9)
        a.ret()
        a.label('end')
    a.mfsr(10, 'tlb_wcnt')
    a.halt()
    return a, vals if kind == 'vict' else None


def _sw_events(log):
    return [dict(kv.split('=') for kv in l.split()[1:]) for l in log.splitlines() if l.startswith('WKEV ')]


def cmd_socwalk(a):
    """the hardware walker through the real D-cache / bus / behavioural DDR
    (tb_soc), and CPU_FEATURES[25] with TLB_WALK = 0.  Each program's result
    registers, and the walker's events (the tb_soc probe, WKEV): D / T lines
    dirty in the cache, the walk reads missing with a dirty victim, and the
    read meeting the eviction writeback of its own line"""
    exe = build_soc(None)
    fails = 0

    def report(name, ok, msg=''):
        nonlocal fails
        fails += not ok
        print(f'{"pass" if ok else "FAIL"} {name} {msg}')

    # 1. D and T lines dirty in the D-cache
    prog, _ = _sw_prog('dirty')
    r = run_soc(exe, prog, 'sw_dirty', trace=True)
    g = _soc_regs(r); ev = _sw_events(r['log'])
    nd = {ph: sum(1 for e in ev if e['ph'] == ph and e['dirty'] == '1') for ph in '01'}
    ok = bool(g) and g.get('r4') == 0x600D and g.get('r5') == 0x7777 and g.get('r10') == 2 and nd['0'] and nd['1']
    report('dirty D / T lines', ok, f'regs={g and {k: hex(g[k]) for k in ("r4", "r5", "r10")}} dirty reads D/T={nd["0"]}/{nd["1"]}')
    # 2. a walk read missing with a dirty victim, and the victims' data in DDR
    prog, vals = _sw_prog('vict')
    r = run_soc(exe, prog, 'sw_vict', trace=True)
    g = _soc_regs(r); ev = _sw_events(r['log'])
    nv = {ph: sum(1 for e in ev if e['ph'] == ph and e['vdirty'] == '1') for ph in '01'}
    ok = bool(g) and g.get('r4') == 0x600D and g.get('r10') == 1 and nv['0'] and nv['1'] and \
        all(g.get(f'r{6 + i}') == vals[i] for i in range(4))
    report('dirty victim', ok, f'regs={g and {k: hex(g[k]) for k in ("r4", "r6", "r7", "r8", "r9", "r10")}} dirty-victim reads D/T={nv["0"]}/{nv["1"]}')
    # 3. the PTE / directory line in its eviction writeback when the walk reads it
    for kind, ph in (('s3t', '1'), ('s3d', '0')):
        seen = []
        allok = True
        for pad in (0, 1, 2, 3, 4, 6, 8, 12):
            prog, _ = _sw_prog(kind, pad)
            r = run_soc(exe, prog, f'sw_{kind}_{pad}', trace=True)
            g = _soc_regs(r); ev = _sw_events(r['log'])
            good = bool(g) and g.get('r4') == 0x600D and g.get('r10') == 1
            allok &= good
            if any(e['ph'] == ph and e['same'] == '1' for e in ev):
                seen.append(pad)
            if not good or a.verbose:
                print(f'  {kind} pad={pad}: r4={g and hex(g.get("r4", -1))} r10={g and g.get("r10")} events={ev}')
        report(f'evict shadow {kind}', allok and bool(seen), f'read met its own line in the writeback at pad={seen}')
    # 4. CPU_FEATURES[25] with TLB_WALK = 0 (and 1 for reference)
    from kasm import Asm
    for nw, want in ((False, 0x03F0200D | _l2bit()), (True, 0x01F0200D | _l2bit())):
        f = Asm(); f.setr(1, 0xF00F0048); f.ld(64, 2, 1); f.halt()
        g = _soc_regs(run_soc(build_soc(None, nowalk=nw), f, f'sw_feat{int(nw)}', trace=True))
        report(f'CPU_FEATURES {"TLB_WALK=0" if nw else "default"}', bool(g) and g.get('r2') == want,
               f'{g and hex(g.get("r2"))} want {want:#x}')
    print(f'{fails} failing')
    return 1 if fails else 0


def cmd_socx(a):
    """MMU_PLAN §11.9 through the real D-cache (tb_soc): tests.py's
    tx_battery programs (page-boundary loads / stores through bases in other
    pages, TE = 1, supervisor and user) with the windows stored first (TE =
    0, so the translated loads hit those lines in the cache: the hit pipe's
    two carry-candidate tags)"""
    import tests
    exe = build_soc(None)
    fails = 0
    for seed in range(a.seed0, a.seed0 + a.seeds):
        for user in (False, True):
            prog, e, _ = tests._tx_battery(seed, user, [], soc=True)
            r = run_soc(exe, prog, f'sx{seed}_{int(user)}', trace=True)
            g = _soc_regs(r)
            ok = bool(g) and g.get('r5') == e['r5'] and g.get('r6') == e['r6'] and 'fatal' not in r['log'].lower()
            fails += not ok
            if not ok or a.verbose:
                print(f'{"pass" if ok else "FAIL"} seed={seed} user={user} r5={g and hex(g.get("r5", 0))} '
                      f'want {e["r5"]:#x} r6={g and hex(g.get("r6", 0))} want {e["r6"]:#x}')
            try:
                os.remove(r['trace'])
            except OSError:
                pass
    print(f'{fails} failing')
    return 1 if fails else 0


def _vga_prog(lines, wbinv_every=1500, flush_every=0, span=0x80000, blit=False):
    """HW_PERF_PLAN §6: 640 x 480 RGB565 scanout of a framebuffer at 4 MiB
    under heavy CPU traffic: a strided walk over `span` bytes (D-cache and L2
    misses, dirty victims on both levels, so the L2's write buffer drains),
    and every `wbinv_every` iterations an L2 write-back-and-invalidate (L2_CTRL =
    3, BUSY polled; skipped without the L2); runs `lines` x ~3200 cycles.
    blit (review 1): the blitter also FILLs a 640 x 64 rect at 5 MiB
    (80 KiB of 16 B DMA writes) back to back (restarted from the loop as
    soon as it is idle).  r1 = VGA_STATUS at the end (underflows [63:48])."""
    from kasm import Asm
    a = Asm()
    a.setr(15, 0xF0110000)
    a.setr(1, 0x400000); a.st(64, 1, 15, 0x08)          # FB_BASE
    a.setr(1, 1280); a.st(64, 1, 15, 0x10)              # STRIDE
    a.setr(1, 480); a.st(64, 1, 15, 0x38)               # HEIGHT
    a.setr(1, 0); a.st(64, 1, 15, 0x28)                 # VSTART
    a.setr(1, 4); a.st(64, 1, 15, 0x18)                 # clear the underflow count
    a.setr(1, 1); a.st(64, 1, 15, 0x00)                 # SCANOUT_EN, RGB565, native
    a.setr(14, 0xF0050000)
    a.setr(10, 0x100000)                                # data base
    a.setr(9, (lines * 3200) // 40)                     # iterations (~40 cycles each at best)
    a.setr(8, 0)                                        # offset
    a.setr(7, wbinv_every)
    if blit:
        a.setr(13, 0xF00E0000)
        for off, v in ((0x10, 0x500000), (0x18, 1280), (0x30, 640), (0x38, 64), (0x40, 0x1234)):
            a.setr(1, v); a.st(64, 1, 13, off)
    a.label('top')
    a.alu('add', 11, 10, 8)
    a.ld(64, 2, 11, 0); a.addi_s(2, 2, 1); a.st(64, 2, 11, 0)
    a.ld(64, 3, 11, 0x1000); a.st(64, 3, 11, 0x1008)
    if blit:
        a.ld(64, 4, 13, 0x08); a.setr(12, 1); a.alu('and', 4, 4, 12)     # STATUS.BUSY
        a.fbr('ne', 4, 0, 'noblit', imm=True)
        a.setr(4, 2); a.st(64, 4, 13, 0x08)                              # DONE: clear
        a.setr(4, 1); a.st(64, 4, 13, 0x00)                              # START, FILL
        a.label('noblit')
    a.setr(12, 37 * 64); a.alu('add', 8, 8, 12)
    a.setr(12, span - 1); a.alu('and', 8, 8, 12)
    a.dec(7)
    a.fbr('ne', 7, 0, 'nowrap', imm=True)
    a.setr(7, wbinv_every)
    a.setr(1, 3); a.st(64, 1, 14, 0x28)                 # L2_CTRL: WBINV (ignored without the L2)
    a.label('poll')
    a.ld(64, 1, 14, 0x28); a.setr(12, 0x100); a.alu('and', 1, 1, 12)
    a.fbr('ne', 1, 0, 'poll', imm=True)
    a.label('nowrap')
    a.dec(9)
    a.fbr('ne', 9, 0, 'top', imm=True)
    a.ld(64, 1, 15, 0x18)                               # VGA_STATUS
    a.halt()
    return a


def cmd_socvga(a):
    """HW_PERF_PLAN §6: VGA scanout keeps its deadline through the L2 (and
    without it) under CPU miss / writeback traffic and L2 WBINV walks, at
    the given MIG timing (--plus).  Fails on any starved display line or on
    a fetch that did not run."""
    exe = build_soc(None)
    prog = _vga_prog(a.lines, a.wbinv_every, span=a.span, blit=a.blit)
    tag = f'vga_{os.environ.get("VSIM_L2", "1")}_' + re.sub(r'[^A-Za-z0-9]', '', ''.join(a.plus)) + \
        f'_{a.span:x}_{a.wbinv_every}{"_b" if a.blit else ""}'
    r = run_soc(exe, prog, tag, timeout=14400, plus=['+VGA_V0=478'] + a.plus)
    m = re.search(r'VGA lines=(\d+) max_fetch=(\d+) underflows=(\d+)', r['log'])
    l2 = re.findall(r'TB_SOC: (MIG bursts rd=\d+ wr=\d+)', r['log'])
    rt = re.findall(r'TB_SOC: L2 rt (drains bad=\d+ need=\d+)', r['log'])
    vr = re.search(r'VGA reads=(\d+) read_cycles=(\d+) grant_cyc=(\d+)', r['log'])
    if vr:
        n = max(1, int(vr.group(1)))
        rt.append(f'vga_rd_lat={int(vr.group(2)) / n:.1f} gnt_wait/read={int(vr.group(3)) / n:.1f}')
    t = re.search(r'\bt=(\d+)', r['log'])
    ok = bool(m) and int(m.group(1)) >= 20 and int(m.group(3)) == 0 and 'HALTED' in r['log']   # (the scanout ran)
    print(f'{"pass" if ok else "FAIL"} socvga lines={m and m.group(1)} max_fetch={m and m.group(2)} '
          f'(of 3200) underflows={m and m.group(3)} {l2[0] if l2 else ""} {" ".join(rt)} '
          f'cycles={t and t.group(1)} {"blit " if a.blit else ""}{" ".join(a.plus)}')
    shutil.rmtree(os.path.join(WORK, 'socruns', tag), ignore_errors=True)
    return 0 if ok else 1


M64 = (1 << 64) - 1


def _l2_loop(a, label, n_reg, n):
    a.setr(n_reg, n); a.label(label)


def _l2_ctrl_prog(K=400):
    """HW_PERF_PLAN §6, L2_CTRL semantics through the SoC (MMIO_MAP):
    96 KiB of stores (the D-cache's victims make the L2 dirty), then a write
    of 0: BUSY must read 1 at once (r2 = 0x100); a write of 1 during the walk
    is ignored; K stores (+ loads) spread over every L2 set run during the
    walk (walked and not-yet-walked sets); BUSY polled to 0; L2_CTRL then
    reads 0 (r4: disabled), a write of 1 enables (r5 = 1); r11 = the sum of
    the region's dwords, read back through the D-cache (bypass L2), r12 =
    the same after re-enabling and a WBINV (3)."""
    from kasm import Asm
    R, SZ, K1, K2, ST = 0x200000, 96 * 1024, 0x9E3779B97F4A7C15, 0xC2B2AE3D27D4EB4F, 0x2348
    mem = {}
    a = Asm()
    a.setr(15, 0xF0050000)
    a.setr(10, R); a.setr(8, 0); a.setr64(7, K1); a.setr(12, 32)
    _l2_loop(a, 'f1', 9, SZ // 32)
    a.st(64, 8, 10, 0); a.alu('add', 8, 8, 7); a.alu('add', 10, 10, 12)
    a.dec(9); a.fbr('ne', 9, 0, 'f1', imm=True)
    v = 0
    for i in range(SZ // 32):
        mem[R + 32 * i] = v; v = (v + K1) & M64
    a.setr(1, 0); a.st(64, 1, 15, 0x28)          # disable: write back, invalidate, bypass
    a.ld(64, 2, 15, 0x28)                         # BUSY at once
    a.setr(1, 1); a.st(64, 1, 15, 0x28)           # ignored (BUSY)
    a.setr(10, R); a.setr(8, 0); a.setr64(6, 5); a.setr64(7, K2); a.setr(13, SZ); a.setr(12, ST)
    _l2_loop(a, 'w1', 9, K)
    a.alu('add', 11, 10, 8); a.st(64, 6, 11, 0); a.ld(64, 3, 11, 8)
    a.alu('add', 6, 6, 7); a.alu('add', 8, 8, 12)
    a.fbr('ult', 8, 13, 'w1n'); a.alu('sub', 8, 8, 13)
    a.label('w1n')
    a.dec(9); a.fbr('ne', 9, 0, 'w1', imm=True)
    off, v = 0, 5
    for j in range(K):
        mem[R + off] = v; v = (v + K2) & M64; off += ST
        if off >= SZ:
            off -= SZ
    a.label('poll'); a.ld(64, 4, 15, 0x28); a.setr(1, 0x100); a.alu('and', 4, 4, 1)
    a.fbr('ne', 4, 0, 'poll', imm=True)
    a.ld(64, 4, 15, 0x28)                         # 0: disabled, idle
    a.setr(1, 1); a.st(64, 1, 15, 0x28); a.ld(64, 5, 15, 0x28)   # 1
    for k, lab in ((11, 's1'), (12, 's2')):
        if k == 12:
            a.setr(1, 3); a.st(64, 1, 15, 0x28)
            a.label('poll2'); a.ld(64, 1, 15, 0x28); a.setr(3, 0x100); a.alu('and', 1, 1, 3)
            a.fbr('ne', 1, 0, 'poll2', imm=True)
        a.setr(10, R); a.setr(k, 0); a.setr(13, 8)
        _l2_loop(a, lab, 9, SZ // 8)
        a.ld(64, 3, 10, 0); a.alu('add', k, k, 3); a.alu('add', 10, 10, 13)
        a.dec(9); a.fbr('ne', 9, 0, lab, imm=True)
    a.halt()
    csum = sum(mem.values()) & M64
    return a, {'r2': 0x100, 'r4': 0, 'r5': 1, 'r11': csum, 'r12': csum}


def _l2_blit_prog():
    """HW_PERF_PLAN §6, DMA ordering through the SoC: the CPU writes 8 rows
    of 64 B, FLUSH (its writeback reaches the L2 / write buffer), the
    blitter FILLs bytes 8..23 of each row (the same bytes), INVALIDATE, the
    CPU reads.  Three rounds: L2 on (the writeback allocates, the blitter's
    write merges), L2 off (both posted to the write buffer: order kept), L2
    on again with the reverse order (blitter first, then CPU stores, FLUSH,
    then a blitter COPY of the rows reads the L2's dirty data).  r11..r13 =
    per-round sums of the rows (r13: of the COPY's destination)."""
    from kasm import Asm
    B, C = 0x300000, 0x340000
    a = Asm()
    a.setr(15, 0xF0050000); a.setr(14, 0xF00E0000)

    def wait_flush():
        lb = f'wf{len(a.labels)}'
        a.label(lb); a.ld(64, 1, 15, 0x10); a.fbr('ne', 1, 0, lb, imm=True)

    def blit(op, dst, src=0):
        a.setr(1, dst); a.st(64, 1, 14, 0x10); a.setr(1, 64); a.st(64, 1, 14, 0x18)
        a.setr(1, src); a.st(64, 1, 14, 0x20); a.setr(1, 64); a.st(64, 1, 14, 0x28)
        a.setr(1, 8 if op == 0 else 32); a.st(64, 1, 14, 0x30); a.setr(1, 8); a.st(64, 1, 14, 0x38)
        a.setr(1, 0x1234); a.st(64, 1, 14, 0x40)
        a.setr(1, 1 | op << 1); a.st(64, 1, 14, 0x00)
        lb = f'bw{len(a.labels)}'
        a.label(lb); a.ld(64, 1, 14, 0x08); a.setr(2, 2); a.alu('and', 1, 1, 2)
        a.fbr('eq', 1, 0, lb, imm=True)
        a.setr(1, 2); a.st(64, 1, 14, 0x08)

    def cpu_rows(seed):
        for r in range(8):
            for k in range(8):
                a.setr64(1, (seed * 0x1000193 + r * 8 + k + 1) * 0x9E3779B97F4A7C15 & M64)
                a.st(64, 1, 10, r * 64 + 8 * k, mode='long')

    def sum_rows(base, reg):
        a.setr(10, base); a.setr(reg, 0)
        for r in range(8):
            for k in range(8):
                a.ld(64, 1, 10, r * 64 + 8 * k, mode='long'); a.alu('add', reg, reg, 1)

    def exp_rows(seed, fill_after):
        t = 0
        for r in range(8):
            for k in range(8):
                v = (seed * 0x1000193 + r * 8 + k + 1) * 0x9E3779B97F4A7C15 & M64
                if fill_after and k in (1, 2):
                    v = 0x1234123412341234
                t += v
        return t & M64

    exp = {}
    for rnd, reg in ((0, 11), (1, 12)):
        if rnd == 1:
            a.setr(1, 0); a.st(64, 1, 15, 0x28)
            lb = 'dis'; a.label(lb); a.ld(64, 1, 15, 0x28); a.setr(2, 0x100); a.alu('and', 1, 1, 2)
            a.fbr('ne', 1, 0, lb, imm=True)
        a.setr(10, B + 0x1000 * rnd)
        cpu_rows(rnd + 1)
        a.setr(1, 2); a.st(64, 1, 15, 0x00); wait_flush()          # FLUSH
        blit(0, B + 0x1000 * rnd + 8)                                 # FILL bytes 8..23
        a.setr(1, 4); a.st(64, 1, 15, 0x00); wait_flush()          # INVALIDATE
        sum_rows(B + 0x1000 * rnd, reg)
        exp[f'r{reg}'] = exp_rows(rnd + 1, True)
    # round 3: L2 on; blitter fill, then CPU rows over it, FLUSH, COPY to C
    a.setr(1, 1); a.st(64, 1, 15, 0x28)
    blit(0, B + 0x2000 + 8)
    a.setr(10, B + 0x2000); cpu_rows(3)
    a.setr(1, 2); a.st(64, 1, 15, 0x00); wait_flush()
    blit(1, C, B + 0x2000)
    a.setr(1, 4); a.st(64, 1, 15, 0x00); wait_flush()
    sum_rows(C, 13)
    exp['r13'] = exp_rows(3, False)
    a.halt()
    return a, exp


def _l2_cnt_prog():
    """HW_PERF_PLAN §6: the L2 counters (0xF00D_0118 ..) over two read
    passes of a cold 96 KiB region (32 B stride): pass 1 misses once per
    64 B line (the other sector arrives with it), pass 2 (the D-cache
    thrashes, the L2 holds it all) hits; r1..r11 = the counter deltas,
    r12 / r13 = PERF_CYCLES of pass 1 / pass 2."""
    from kasm import Asm
    R, SZ = 0x200000, 96 * 1024
    a = Asm()
    a.setr(15, 0xF00D0000)
    a.setr(1, 1); a.st(64, 1, 15, 0x00)
    for p, reg in ((0, 12), (1, 13)):
        a.ld(64, 14, 15, 0x08)
        a.setr(10, R); a.setr(12 if p == 0 else 13, 0); a.setr(11, 32)
        _l2_loop(a, f'p{p}', 9, SZ // 32)
        a.ld(64, 2, 10, 0); a.alu('add', 10, 10, 11)
        a.dec(9); a.fbr('ne', 9, 0, f'p{p}', imm=True)
        a.ld(64, reg, 15, 0x08); a.alu('sub', reg, reg, 14)
    for k in range(11):
        a.ld(64, 1 + k, 15, 0x118 + 8 * k)
    a.halt()
    return a


def cmd_socl2(a):
    """HW_PERF_PLAN §6: the L2 through the SoC — L2_CTRL semantics with
    stores during the walk, DMA (blitter) ordering against D-cache
    writebacks, the counters.  Every run also under the consistency
    checker."""
    exe = build_soc(None)
    l2 = l2_cfg()[0]
    fails = 0

    def report(name, ok, msg=''):
        nonlocal fails
        fails += not ok
        print(f'{"pass" if ok else "FAIL"} {name} {msg}')

    if l2:
        prog, exp = _l2_ctrl_prog()
        g = _soc_regs(run_soc(exe, prog, 'l2_ctrl', trace=True, plus=a.plus))
        bad = {k: hex(g.get(k, -1)) for k in exp if g.get(k) != exp[k]} if g else {'halt': 0}
        report('ctrl', not bad, str(bad))
    prog, exp = _l2_blit_prog()
    g = _soc_regs(run_soc(exe, prog, 'l2_blit', trace=True, plus=a.plus))
    bad = {k: hex(g.get(k, -1)) for k in exp if g.get(k) != exp[k]} if g else {'halt': 0}
    report('blit', not bad, str(bad))
    if l2:
        g = _soc_regs(run_soc(exe, _l2_cnt_prog(), 'l2_cnt', trace=True, plus=a.plus))
        if g is None:
            report('counters', False, 'no halt')
        else:
            c = [g[f'r{1 + k}'] for k in range(11)]
            names = ['i_hit', 'i_miss', 'd_hit', 'd_miss', 'w_hit', 'w_miss', 'rd_beat', 'wr_beat',
                     'wbfull', 'dma_hit', 'dma_miss']
            # (w_hit / w_miss: dirty lines of the boot image leaving the D-cache)
            ok = (c[3] == 1536 and 4600 <= c[2] <= 4620 and c[4] + c[5] <= 64 and c[9] == c[10] == 0 and
                  c[6] == 8 * (c[1] + c[3]) and c[7] == 0)
            report('counters', ok, ' '.join(f'{n}={v}' for n, v in zip(names, c)) +
                   f' cycles pass1={g["r12"]} pass2={g["r13"]} ({g["r12"] / 3072:.1f} / {g["r13"] / 3072:.1f} a load)')
    print(f'{fails} failing')
    return 1 if fails else 0


def _bench_progs():
    """HW_PERF_PLAN §6: footprints between the L1s (64 KiB each) and the
    L2 (128 KiB): (name, Asm, check register) — each ends with r1 = a
    checksum that must not depend on the memory system"""
    import random
    from kasm import Asm
    out = []
    # data: 4 passes of load + add + store over 96 KiB (dwords at 16 B stride)
    a = Asm(); R, SZ = 0x200000, 96 * 1024
    a.setr(1, 0); a.setr(12, 16); a.setr(8, 4)
    a.label('dp')
    a.setr(10, R); a.setr(9, SZ // 16)
    a.label('dl'); a.ld(64, 2, 10, 0); a.alu('add', 1, 1, 2); a.addi_s(2, 2, 3); a.st(64, 2, 10, 0)
    a.alu('add', 10, 10, 12); a.dec(9); a.fbr('ne', 9, 0, 'dl', imm=True)
    a.dec(8); a.fbr('ne', 8, 0, 'dp', imm=True)
    a.halt()
    out.append(('data96k', a))
    # code: 96 functions of ~1 KiB, called in turn, 3 passes
    a = Asm(); rnd = random.Random(7)
    a.setr(1, 0); a.setr(8, 3)
    a.label('cp')
    for f in range(96):
        a.call(f'f{f}')
    a.dec(8); a.fbr('ne', 8, 0, 'cp', imm=True)
    a.halt()
    for f in range(96):
        while a.pc % 32:
            a.nop()
        a.label(f'f{f}')
        for _ in range(250):
            r = rnd.randrange(2, 8)
            a.alu(rnd.choice(['add', 'xor']), 1, 1, r) if rnd.random() < 0.5 else a.addi_s(r, r, rnd.randrange(1, 9))
        a.ret()
    out.append(('code96k', a))
    # pointer chase: a random cycle through 112 KiB of 64 B nodes, 4 laps
    a = Asm(); R, N = 0x280000, 112 * 1024 // 64
    perm = list(range(N)); rnd.shuffle(perm)
    a.setr(10, R + 64 * perm[0])
    nxt = {perm[i]: perm[(i + 1) % N] for i in range(N)}
    # build the list with stores (node k -> next), then chase
    a.setr(12, R)
    for k in range(N):
        a.setr(2, R + 64 * nxt[k]); a.setr(11, R + 64 * k); a.st(64, 2, 11, 0)
    a.setr(1, 0); a.setr(9, 4 * N)
    a.label('pc'); a.ld(64, 10, 10, 0); a.alu('add', 1, 1, 10); a.dec(9); a.fbr('ne', 9, 0, 'pc', imm=True)
    a.halt()
    out.append(('chase112k', a))
    return out


def cmd_socbench(a):
    """HW_PERF_PLAN §6: PIPE_RUN cycles of the large-footprint programs with
    and without the L2 (VSIM_L2=0 build), same MIG model; registers equal"""
    with _env(['VSIM_L2=0']):
        off = build_soc(None)
    on = build_soc(None)
    # --base REV: REV's RTL (L2 on) under the working tree's tb_soc / MIG
    # model as a third column (review 1: cycle deltas against fba47c0)
    cols = [('off', off), ('on', on)] + ([('base', build_soc(a.base, tb_wt=True))] if a.base else [])
    fails = 0
    for name, prog in _bench_progs():
        res = []
        for tag, exe in cols:
            r = run_soc(exe, prog, f'bench_{name}_{tag}', trace=True, plus=a.plus, timeout=7200)
            t = int(re.search(r'\bt=(\d+)', r['log']).group(1)) if 't=' in r['log'] else 0
            res.append((t, _soc_regs(r)))
            try:
                os.remove(r['trace'])
            except OSError:
                pass
        same = res[0][1] is not None and all(r[1] == res[0][1] for r in res)
        fails += not same
        b = f'; {a.base}: {res[2][0]} ({100.0 * (res[1][0] - res[2][0]) / max(1, res[2][0]):+.2f}% now)' if a.base else ''
        print(f'{"same" if same else "DIFF"} {name}: cycles {res[0][0]} -> {res[1][0]} '
              f'({100.0 * (res[1][0] - res[0][0]) / max(1, res[0][0]):+.1f}%){b} {" ".join(a.plus)}')
    return 1 if fails else 0


def cmd_soceth(a):
    """ETH_RX_SLOTS (0xF007_0000, HW_PERF_PLAN.md §3) through the real SoC bus
    (bus_splitter -> eth_mmio_bridge; tb_soc stubs liteeth_core, the RX slot
    behaviour itself is tools/ethsim/run_eth_tb.sh): reset value 0x802 after
    the program load, write clamping, the [11:8] = 8 field, the rest of the
    0xF007 window reading 0, CPU_FEATURES[24]."""
    from kasm import Asm
    exe = build_soc(None)
    a = Asm(); a.setr(15, 0xF0070000); a.setr(14, 0xF00F0000)
    a.ld(64, 1, 14, 0x48)                                              # CPU_FEATURES
    a.ld(64, 2, 15, 0x00)                                              # 0x802 (reset / program load)
    a.setr(3, 5); a.st(64, 3, 15, 0x00); a.ld(64, 4, 15, 0x00)         # 0x805
    a.setr(3, 0); a.st(64, 3, 15, 0x00); a.ld(64, 5, 15, 0x00)         # 0 -> 2
    a.setr(3, 100); a.st(64, 3, 15, 0x00); a.ld(64, 6, 15, 0x00)       # 100 -> 8
    a.ld(64, 7, 15, 0x08)                                              # 0
    a.setr(3, 1); a.st(64, 3, 15, 0x00); a.ld(64, 8, 15, 0x00)         # 1 -> 2
    a.halt()
    exp = {'r1': 0x03F0200D | _l2bit(), 'r2': 0x802, 'r4': 0x805, 'r5': 0x802, 'r6': 0x808, 'r7': 0, 'r8': 0x802}
    r = run_soc(exe, a, 'soceth', trace=True)
    last = open(r['trace']).read().strip().splitlines()[-1] if 'HALTED' in r['log'] else ''
    regs = {k: int(v, 16) for k, v in re.findall(r'(r\d+)=([0-9a-f]{16})', last)}
    bad = [f'{k}={regs.get(k) if regs.get(k) is None else hex(regs[k])} want {hex(v)}'
           for k, v in exp.items() if regs.get(k) != v]
    print(f'{"pass" if last and not bad else "FAIL"} soceth ' + ' '.join(bad))
    return 1 if bad or not last else 0


class _env:
    """temporarily set environment variables (KEY=VAL strings)"""
    def __init__(self, kvs):
        self.kv = dict(x.split('=', 1) for x in (kvs or []))
    def __enter__(self):
        self.old = {k: os.environ.get(k) for k in self.kv}
        os.environ.update(self.kv)
    def __exit__(self, *e):
        for k, v in self.old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def cmd_socdiff(a):
    """random programs through the full SoC (real cache / bus splitter):
    base rev vs working tree, retire traces must be identical.  --base wt:
    the working tree as the base (with --base-env, e.g. VSIM_L2=0)"""
    import randprog
    with _env(a.base_env):
        base = build_soc(None if a.base == 'wt' else a.base, tb_wt=a.base_tb_wt)
    new = build_soc(None)
    fails = 0
    tb = tn = 0
    hits = {'base': 0, 'new': 0}      # BTIC hits (the probe's count)
    for sd in range(a.seed0, a.seed0 + a.seeds):
        asm = randprog.gen(sd, span=a.span)
        rb = run_soc(base, asm, f'sd_b{sd}', trace=True, gate=a.base == 'wt' or a.base_tb_wt, plus=a.plus)
        rn = run_soc(new, asm, f'sd_n{sd}', trace=True, plus=a.plus)
        okb = 'HALTED' in rb['log']
        same = okb and open(rb['trace']).read() == open(rn['trace']).read()
        t = [int(re.search(r'\bt=(\d+)', r['log']).group(1)) if 't=' in r['log'] else 0 for r in (rb, rn)]
        tb += t[0]; tn += t[1]
        for k, r in (('base', rb), ('new', rn)):
            m = re.search(r'BTPROBE btic_chk=\d+ btic_hits=(\d+)', r['log'])
            hits[k] += int(m.group(1)) if m else 0
        if not same:
            fails += 1
        if not same or a.verbose:
            print(f'{"same" if same else "DIFF"} seed={sd} cycles {t[0]} -> {t[1]}  {rb["log"].strip().splitlines()[-2][:60]}')
    print(f'{fails} differing; PIPE_RUN cycles base={tb} new={tn} ({100.0 * (tn - tb) / max(1, tb):+.2f}%); '
          f'BTIC hits base={hits["base"]} new={hits["new"]}')
    return 1 if fails else 0


def _tecost_prog(seed, te, user):
    """MMU_PLAN §11.9: randprog `seed` behind a prologue that maps every page
    it touches (code, data, stack) with global identity entries and then
    writes STATUS with TE = 1 or 0 -- the only difference between the two
    runs.  user: IRET into user mode first (a user HALT traps to a
    supervisor HALT)."""
    import randprog
    from kasm import Asm
    from tests import _tlb_set, _to_user, TLB_RWX, USP, KSP

    class CostAsm(Asm):
        def __init__(self, *aa, **kk):
            super().__init__(*aa, **kk)
            self.leapc(1, 'h_cost'); self.mtsr('evec', 1)
            for i, vp in enumerate((0x0000, 0x0001, USP >> 16, (USP >> 16) - 1)):
                _tlb_set(self, i, vp, vp, TLB_RWX, g=1)
            self.setr(1, USP); self.setsp(1)
            self.setr(1, 0x200 if te else 0); self.mtsr('status', 1)
            if user:                      # (the kernel SP of the final trap: kseg with TE)
                self.setr(1, KSP if te else USP - 0x8000); self.setsp(1)
                _to_user(self, 'u_cost')
                self.label('u_cost')

    old = randprog.Asm
    randprog.Asm = CostAsm
    try:
        asm = randprog.gen(seed)
    finally:
        randprog.Asm = old
    asm.label('h_cost')
    asm.halt()
    return asm


def cmd_tecost(a):
    """MMU_PLAN §11.9: the cycle cost of TE = 1 on randprog programs (every
    page mapped; STATUS the only difference), supervisor and user mode"""
    exe = build(a.rev)
    rc = 0
    for user in (False, True):
        tot = [0, 0]
        for s in range(a.seed0, a.seed0 + a.seeds):
            st = []
            for te in (0, 1):
                r, _ = run(exe, _tecost_prog(s, te, user), f'tc{s}_{te}{int(user)}', a.plus)
                if r.get('end') != 'park' or r.get('kind') != '0' or 'fatal' in r['log'].lower():
                    print(f'FAIL seed={s} te={te} user={user}: end={r.get("end")} kind={r.get("kind")}')
                    rc = 1
                st.append(r)
            if [st[0].get(k) for k in ('r1', 'r2', 'r3', 'r10', 'm00010000')] != \
               [st[1].get(k) for k in ('r1', 'r2', 'r3', 'r10', 'm00010000')]:
                print(f'DIFF seed={s} user={user}')
                rc = 1
            for i in (0, 1):
                tot[i] += int(st[i].get('cycles', 0))
        print(f'{"user" if user else "supervisor"}: TE=0 {tot[0]}  TE=1 {tot[1]}  '
              f'({100.0 * (tot[1] - tot[0]) / max(1, tot[0]):+.2f}%)  {a.plus}')
    return rc


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
    tl = sub.add_parser('tlbdiff'); tl.add_argument('--seeds', type=int, default=20)
    tl.add_argument('--seed0', type=int, default=1); tl.add_argument('--lats', type=int, nargs='+', default=[3])
    tl.add_argument('--irq', action='store_true'); tl.add_argument('-v', '--verbose', action='store_true')
    tl.add_argument('--plus', nargs='*', default=[])
    tl.add_argument('--k', type=int, default=0, help='refill over K entries (a power of two <= TLB_N; default: random 2/4/16)')
    tl.add_argument('--walk', action='store_true', help='page tables + the hardware walker (MMU_PLAN §11.8)')
    tl.add_argument('--absent', type=float, default=0.0, help='--walk: fraction of PTEs not PRESENT (software refill)')
    tl.add_argument('--span', type=lambda v: int(v, 0), default=0x800,
                    help='data footprint (e.g. 0x80000: 8 pages, for evictions with a small VSIM_TLB_N)')
    tp = sub.add_parser('tlbpage'); tp.add_argument('--seeds', type=int, default=30)
    tp.add_argument('--seed0', type=int, default=1); tp.add_argument('--lats', type=int, nargs='+', default=[3])
    tp.add_argument('--irq', action='store_true'); tp.add_argument('--hit', type=int)
    tp.add_argument('--plus', nargs='*', default=[])
    sub.add_parser('soccrypto')
    dz = sub.add_parser('dcbzdiff'); dz.add_argument('--seeds', type=int, default=20)
    dz.add_argument('--seed0', type=int, default=1); dz.add_argument('-v', '--verbose', action='store_true')
    sz = sub.add_parser('socdcbz'); sz.add_argument('--seeds', type=int, default=6)
    sz.add_argument('--seed0', type=int, default=1)
    sub.add_parser('socsd')
    sdd = sub.add_parser('socsddma'); sdd.add_argument('only', nargs='*')
    sdd.add_argument('--plus', action='append', default=[]); sdd.add_argument('--keep', action='store_true')
    sw = sub.add_parser('socwalk'); sw.add_argument('-v', '--verbose', action='store_true')
    sub.add_parser('soceth')
    bp = sub.add_parser('socbench'); bp.add_argument('--plus', action='append', default=[])
    bp.add_argument('--base', help='also REV (L2 on) under the current tb_soc / MIG model')
    l2p = sub.add_parser('socl2'); l2p.add_argument('--plus', action='append', default=[])
    vg = sub.add_parser('socvga'); vg.add_argument('--lines', type=int, default=120)
    vg.add_argument('--wbinv-every', type=int, default=1500)
    vg.add_argument('--span', type=lambda v: int(v, 0), default=0x80000)
    vg.add_argument('--plus', action='append', default=[])
    vg.add_argument('--blit', action='store_true', help='the blitter FILLs back to back too (review 1)')
    sx = sub.add_parser('socx'); sx.add_argument('--seeds', type=int, default=4)
    sx.add_argument('--seed0', type=int, default=0); sx.add_argument('-v', '--verbose', action='store_true')
    sm = sub.add_parser('socmmio'); sm.add_argument('--base', required=True)
    sm.add_argument('--seeds', type=int, default=20); sm.add_argument('--seed0', type=int, default=1)
    x = sub.add_parser('excdiff'); x.add_argument('--seeds', type=int, default=20)
    x.add_argument('--seed0', type=int, default=1); x.add_argument('--lats', type=int, nargs='+', default=[3])
    x.add_argument('--p', type=float, default=0.08); x.add_argument('--irq', action='store_true')
    x.add_argument('-v', '--verbose', action='store_true')
    x.add_argument('--plus', nargs='*', default=[])
    bf = sub.add_parser('btfuzz'); bf.add_argument('--seeds', type=int, default=200)
    bf.add_argument('--seed0', type=int, default=1); bf.add_argument('--rev')
    bf.add_argument('-v', '--verbose', action='store_true')
    tc = sub.add_parser('tecost'); tc.add_argument('--seeds', type=int, default=60)
    tc.add_argument('--seed0', type=int, default=1); tc.add_argument('--rev')
    tc.add_argument('--plus', nargs='*', default=[])
    sd = sub.add_parser('socdiff'); sd.add_argument('--base', required=True)
    sd.add_argument('--seeds', type=int, default=10); sd.add_argument('--seed0', type=int, default=1)
    sd.add_argument('-v', '--verbose', action='store_true')
    sd.add_argument('--base-env', action='append', help='KEY=VAL for the base build (with --base wt)')
    sd.add_argument('--base-tb-wt', action='store_true',
                    help="the base revision's RTL under the working tree's tb_soc / MIG model")
    sd.add_argument('--plus', action='append', default=[], help='tb_soc plusarg for both runs (e.g. +MIGLAT=20)')
    sd.add_argument('--span', type=lambda v: int(v, 0), default=0x800,
                    help='data footprint of the random loads/stores (e.g. 64: same-line hazards)')
    a = p.parse_args()
    import atexit

    def _btic_gate():
        # HW_PERF_PLAN §5 review M2: the BTIC coverage check is a default
        # expectation of every run with 32 B lines (tb_vsim and tb_soc)
        if BTIC_STATS['runs']:
            print(f'BTIC checks: {BTIC_STATS["chk"]} entries checked, {BTIC_STATS["hits"]} hits checked '
                  f'in {BTIC_STATS["runs"]} gated run(s); {BTIC_STATS["pau"]} entries skipped (TE=1, PA unknown)')
        # Vacuity rule: BTIC hits happened but the coverage monitor checked no
        # entry at all (and none was skipped for an unknown PA, which is the one
        # legitimate reason).  Runs with no BTIC activity (BTIC_EN=0, tiny
        # tests: hits == 0) are not affected.  Aggregated over the command, as
        # a single short run may hit without a checkable entry.
        if BTIC_STATS['hits'] > 0 and BTIC_STATS['chk'] == 0 and BTIC_STATS['pau'] == 0:
            print('BTIC gate: hits but zero entries checked (vacuous coverage check)')
            sys.stdout.flush(); os._exit(1)
        if CONS_STATS['runs']:
            print(f'Consistency checks: {CONS_STATS["n"]} answers checked in {CONS_STATS["runs"]} tb_soc run(s); '
                  f'{CONS_STATS["rt_need"]} L2 drains in the VGA/DMA window, each for a request waiting on the buffer')
        if CONS_FAILS:
            print(f'memory consistency check failed in {len(CONS_FAILS)} run(s): {" ".join(CONS_FAILS[:10])}')
            sys.stdout.flush(); os._exit(1)
        if BTIC_FAILS:
            print(f'BTIC coverage check failed in {len(BTIC_FAILS)} run(s): {" ".join(BTIC_FAILS[:10])}')
            sys.stdout.flush(); os._exit(1)
    atexit.register(_btic_gate)
    # tests.py's (and tlbpage's) `import run` must get THIS module, not a second copy run as __main__
    # that would bypass the BTIC gate state (BTIC_FAILS / BTIC_STATS)
    sys.modules.setdefault('run', sys.modules[__name__])
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
    elif a.cmd == 'tlbpage':
        sys.modules['run'] = sys.modules[__name__]   # (tlbpage's `import run`: this module, its BTIC gate)
        import tlbpage
        sys.exit(tlbpage.main(a.seed0, a.seeds, a.lats, a.hit, a.irq, a.plus))
    elif a.cmd == 'soccrypto':
        sys.exit(cmd_soccrypto(a))
    elif a.cmd == 'dcbzdiff':
        sys.exit(cmd_dcbzdiff(a))
    elif a.cmd == 'socdcbz':
        sys.exit(cmd_socdcbz(a))
    elif a.cmd == 'socwalk':
        sys.exit(cmd_socwalk(a))
    elif a.cmd == 'socsddma':
        sys.exit(cmd_socsddma(a))
    elif a.cmd == 'socsd':
        sys.exit(cmd_socsd(a))
    elif a.cmd == 'socx':
        sys.exit(cmd_socx(a))
    elif a.cmd == 'soceth':
        sys.exit(cmd_soceth(a))
    elif a.cmd == 'socbench':
        sys.exit(cmd_socbench(a))
    elif a.cmd == 'socl2':
        sys.exit(cmd_socl2(a))
    elif a.cmd == 'socvga':
        sys.exit(cmd_socvga(a))
    elif a.cmd == 'tlbdiff':
        sys.exit(cmd_tlbdiff(a))
    elif a.cmd == 'socmmio':
        sys.exit(cmd_socmmio(a))
    elif a.cmd == 'tecost':
        sys.exit(cmd_tecost(a))
    elif a.cmd == 'btfuzz':
        sys.exit(cmd_btfuzz(a))
    else:
        sys.exit(cmd_diff(a))


if __name__ == '__main__':
    main()

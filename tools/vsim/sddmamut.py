#!/usr/bin/env python3
"""Mutation checks of the SD DMA engine, the cache range operations and
RANGE_IINV (SD DMA spec v1.2; HW_PERF_PLAN.md §7).

Each mutation is one textual change to an RTL file.  The mutant's sources
(KlaussCPU.srcs/sources_1/new, sim_1/new and tools/) are copied to
$VSIM_WORK/sddmamut/<name> and its suite runs there: 'unit' = tb_sd_dma under
xsim (tools/sdsim/run_sd_tb.sh dma, seeds 1 and 2), 'soc' = the named
socsddma tests under Verilator (tb_soc).  A mutation is killed when its
suite fails (a check, an assertion, or a build error).  Mutants run in
parallel (-j); their directories are deleted afterwards.

    python3 tools/vsim/sddmamut.py [--only NAME ...] [-j 6]
"""
import argparse, concurrent.futures as cf, os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
WORK = os.path.join(os.environ.get('VSIM_WORK', '/tmp/vsim'), 'sddmamut')
SN, MR, PC = 'sources_1/new/sd_native.sv', 'sources_1/new/mem_read_write.sv', 'sources_1/new/pipeline_core.sv'
TOP = 'sources_1/new/KlaussCPU.sv'

# (name, what it breaks, file, suite (unit | list of socsddma test globs), old, new)
MUTS = [
    # ---------------- the engine (tb_sd_dma) ----------------
    ('dd_early', "a DMA read's DATA_DONE when its last block is received, not in DDR", SN, 'unit',
     "if (r_dm_on && !r_dm_wr) r_dd_pend   <= 1'b1;\n                                        else                     r_data_done <= 1'b1;",
     "r_data_done <= 1'b1;"),
    ('m1_adapter', 'DMA_BUSY ignores the transaction in flight (m1)', SN, 'unit',
     "wire       w_dma_busy = w_a_busy || (r_dm_ph && !w_dm_wend);",
     "wire       w_dma_busy = (r_dm_ph && !w_dm_wend);"),
    ('refuse_size', 'a block size not a multiple of 8 is not refused', SN, 'unit',
     "(r_blksz[2] || (r_blkcnt != 16'd0 && !r_rng_ok));", "(r_blkcnt != 16'd0 && !r_rng_ok);"),
    ('refuse_range', 'a counted range leaving DDR is not refused', SN, 'unit',
     "(r_blksz[2] || (r_blkcnt != 16'd0 && !r_rng_ok));", "r_blksz[2];"),
    ('range_hi', 'the up-front check ignores SDM_DMA_ADDR[63:27]', SN, 'unit',
     "        r_rng_ok   <= (r_dma_addr[63:DDR_LG] == '0) &&", "        r_rng_ok   <= 1'b1 &&"),
    ('blk_check', 'no per-block check (an open-ended DMA runs off DDR)', SN, 'unit',
     "    wire w_blk_ok = (r_dma_addr[63:DDR_LG] == '0) &&", "    wire w_blk_ok = 1'b1 ||"),
    ('fetch_room', 'a write fetches a block without room for it in the FIFO', SN, 'unit',
     "(r_dm_wr ? ((11'd512 - {1'b0, w_used}) >= {1'b0, r_d_blkw})", "(r_dm_wr ? 1'b1"),
    ('fetch_count', 'a counted write fetches past size x count', SN, 'unit',
     "wire w_cnt_left = (r_d_cnt == 16'd0) || (r_dm_nblk != r_d_cnt);", "wire w_cnt_left = 1'b1;"),
    ('kill_keep', 'a FIFO reset leaves the engine waiting for its transaction (its read is pushed after)', SN, 'unit',
     "                r_x_own <= 1'b0;\n                r_m_st  <= M_IDLE;\n            end\n        end\n    end",
     "            end\n        end\n    end"),
    ('abort_ignored', 'DMA_EN = 0 / SD_MODE = 0 does not stop the engine', SN, 'unit',
     "            if (w_dm_abort) r_dm_on <= 1'b0;", ""),
    ('pio_pop', 'an SDM_DATA load during a DMA pops the FIFO', SN, 'unit',
     "wire w_cpu_pop = w_new_rd && w_win && (w_off == O_DATA) && r_head_v && !w_dma_busy;",
     "wire w_cpu_pop = w_new_rd && w_win && (w_off == O_DATA) && r_head_v;"),
    ('pio_push', 'an SDM_DATA store during a DMA is pushed', SN, 'unit',
     "wire w_push    = w_push_rq && !w_dma_busy;", "wire w_push    = w_push_rq;"),
    ('thresh', 'FIFO_THRESH is not 0 during a DMA', SN, 'unit',
     "wire w_thresh    = (w_thresh_rd || w_thresh_wr) && !w_dma_busy;", "wire w_thresh    = (w_thresh_rd || w_thresh_wr);"),
    ('addr_write', 'an SDM_DMA_ADDR write during a DMA takes', SN, 'unit',
     "w_off == O_DMA && !w_dma_busy) r_dma_addr", "w_off == O_DMA) r_dma_addr"),
    ('lane_mask', "a single word's mask is the other lane's", SN, 'unit',
     "16'h0000 : lane ? 16'hFF00 : 16'h00FF;", "16'h0000 : lane ? 16'h00FF : 16'hFF00;"),
    ('stop_drop', 'after CMD12 / an error the blocks still in the FIFO are not moved', SN, 'unit',
     "(!w_d_active && !r_dd_pend && w_level < r_d_blkw));", "(!w_d_active && !r_dd_pend));"),
    ('adapter_reset', 'the native reset drops a DDR transaction in flight', SN, 'unit',
     "        r_a_cpl    <= 1'b0;\n        case (r_a_st)",
     "        r_a_cpl    <= 1'b0;\n        if (i_rst) begin r_a_st <= A_IDLE; r_o_req <= 1'b0; r_o_wdv <= 1'b0; r_o_rdv <= 1'b0; end else\n        case (r_a_st)"),
    ('irq27', 'DMA_ERR does not interrupt', SN, 'unit',
     "localparam logic [31:0] IRQ_MASK = DMA_EN ? 32'h0FFF_0310", "localparam logic [31:0] IRQ_MASK = DMA_EN ? 32'h07FF_0310"),
    ('busy_cmd', 'a data command is accepted while DMA_BUSY', SN, 'unit',
     "!(r_in_wdata[11] && (w_dbusy || w_dma_busy));", "!(r_in_wdata[11] && w_dbusy);"),
    # ---------------- the arbiter, the range walk (tb_soc) ----------------
    ('m3_gate', 'the SD DMA is granted during a maintenance walk (m3)', MR, ['wr_m3_*', 'rd_m3_*'],
     "            if (i_vga_req && w_vga_gate)\n                r_grant_vga  <= 1'b1;",
     "            if (i_vga_req && w_vga_gate)\n                r_grant_vga  <= 1'b1;\n            else if (i_sd_req && !is_miss_path)\n                r_grant_sd   <= 1'b1;"),
    ('m3_pending', 'a pending range op is not MNT_BUSY (nor holds the DMA off)', MR, ['wr_m3_queued', 'rd_m3_queued', 'range_queue_*'],
     "assign o_mnt_busy = r_mnt_active | r_mnt_pending | r_rng_pend;",
     "assign o_mnt_busy = r_mnt_active | r_mnt_pending;"),
    ('rw_two', "the range walk misses a set's second range line (T + 1)", MR, ['range_two'],
     "(r_rw_two != 11'd0 && r_tag_way0[TAG_BITS-1:0] == r_rw_tag1));\n    wire w_rw_sel1 = r_tag_way1[TAG_BITS] && (r_tag_way1[TAG_BITS-1:0] == r_rw_tag ||\n                                              (r_rw_two != 11'd0 && r_tag_way1[TAG_BITS-1:0] == r_rw_tag1));",
     "1'b0);\n    wire w_rw_sel1 = r_tag_way1[TAG_BITS] && (r_tag_way1[TAG_BITS-1:0] == r_rw_tag ||\n                                              1'b0);"),
    ('rw_any', 'the range walk takes every valid way of a set it visits (not only the range)', MR, ['range_prec'],
     "    wire w_rw_sel0 = r_tag_way0[TAG_BITS] && (r_tag_way0[TAG_BITS-1:0] == r_rw_tag ||\n                                              (r_rw_two != 11'd0 && r_tag_way0[TAG_BITS-1:0] == r_rw_tag1));\n    wire w_rw_sel1 = r_tag_way1[TAG_BITS] && (r_tag_way1[TAG_BITS-1:0] == r_rw_tag ||",
     "    wire w_rw_sel0 = r_tag_way0[TAG_BITS] && (1'b1 ||\n                                              (r_rw_two != 11'd0 && r_tag_way0[TAG_BITS-1:0] == r_rw_tag1));\n    wire w_rw_sel1 = r_tag_way1[TAG_BITS] && (1'b1 ||"),
    ('rinv_nowb', 'RANGE_INV drops a dirty line without writing it back (the emulator / kernel reading)', MR, ['range_inv'],
     "if ((w_rw_sel0 && r_dirty_way0) || (w_rw_sel1 && r_dirty_way1)) begin\n                            r_mnt_sub <= MS_W0;",
     "if (!r_rw_inv && ((w_rw_sel0 && r_dirty_way0) || (w_rw_sel1 && r_dirty_way1))) begin\n                            r_mnt_sub <= MS_W0;"),
    ('rw_lastset', 'the range walk stops one set early', MR, ['range_partial', 'rd_m3_*', 'wr_m3_*'],
     "                            if (r_rw_left == 11'd1) begin\n                                r_mnt_sub <= MS_DONE;\n                            end else begin\n                                r_cache_index <= r_cache_index + 1'b1;\n                                if (r_cache_index == INDEX_BITS'(CACHE_SIZE - 1)) begin\n                                    r_rw_tag  <= r_rw_tag + 1'b1;\n                                    r_rw_tag1 <= r_rw_tag1 + 1'b1;\n                                end\n                                r_rw_left <= r_rw_left - 11'd1;\n                                if (r_rw_two != 11'd0) r_rw_two <= r_rw_two - 11'd1;\n                                r_mnt_sub <= MS_READ;\n                            end\n                        end\n                    end",
     "                            if (r_rw_left == 11'd2) begin\n                                r_mnt_sub <= MS_DONE;\n                            end else begin\n                                r_cache_index <= r_cache_index + 1'b1;\n                                if (r_cache_index == INDEX_BITS'(CACHE_SIZE - 1)) begin\n                                    r_rw_tag  <= r_rw_tag + 1'b1;\n                                    r_rw_tag1 <= r_rw_tag1 + 1'b1;\n                                end\n                                r_rw_left <= r_rw_left - 11'd1;\n                                if (r_rw_two != 11'd0) r_rw_two <= r_rw_two - 11'd1;\n                                r_mnt_sub <= MS_READ;\n                            end\n                        end\n                    end"),
    ('rng_live', 'a queued range op uses the live CACHE_OP_ADDR / LEN, not its copy', MR, ['range_queue_copy'],
     "        if (i_rng_go && !w_rng_cvt) begin\n            r_rng_pend <= 1'b1;\n            r_rng_inv  <= i_rng_inv;\n            r_rng_l0   <= i_rng_l0;\n            r_rng_n    <= i_rng_n[11:0];\n        end",
     "        if (i_rng_go && !w_rng_cvt) begin\n            r_rng_pend <= 1'b1;\n            r_rng_inv  <= i_rng_inv;\n        end\n        r_rng_l0   <= i_rng_l0;\n        r_rng_n    <= i_rng_n[11:0];"),
    ('rng_noqueue', 'a second range op overwrites the pending one', MR, ['range_queue_full'],
     "wire         w_rng_cvt = i_rng_go && (i_rng_n > 28'(RNG_MAX) || w_rng_pq || r_mnt_pending);",
     "wire         w_rng_cvt = i_rng_go && (i_rng_n > 28'(RNG_MAX) || r_mnt_pending);"),
    ('rng_big', 'a range op over 64 KiB walks only 1024 sets of it', MR, ['range_combo'],
     "wire         w_rng_cvt = i_rng_go && (i_rng_n > 28'(RNG_MAX) || w_rng_pq || r_mnt_pending);",
     "wire         w_rng_cvt = i_rng_go && (w_rng_pq || r_mnt_pending);"),
    ('whole_wins', 'range bits with FLUSH / INVALIDATE also queue a range op', TOP, ['range_combo'],
     "(w_mmio_write_data[5:4] != 2'b00) &&\n                                 !w_mmio_write_data[1] && !w_mmio_write_data[2] &&",
     "(w_mmio_write_data[5:4] != 2'b00) &&"),
    ('hit_pend', 'a load hits a line a pending RANGE_INV is about to drop (the hit pipe not held off)', MR, ['inv_hit_range'],
     "r_h_conf <= (state == MAINT) || (state != WAIT && r_cache_index == w_h_idx) ||\n                    (r_rng_pend && r_rng_inv);",
     "r_h_conf <= (state == MAINT) || (state != WAIT && r_cache_index == w_h_idx);"),
    # ---------------- RANGE_IINV (tb_soc) ----------------
    ('iinv_nomask', 'the range sweep clears nothing', PC, ['iinv', 'iinv_wide'],
     "         wire we = swst ? (!r_sw_rng || r_sw_mask[b])", "         wire we = swst ? (!r_sw_rng)"),
    ('iinv_off1', "the range sweep's bank mask is one bank off", PC, ['iinv', 'iinv_other'],
     "         blo = int'(s >> VB_AW) + ((a < s[VB_AW-1:0]) ? 1 : 0);\n         bhi = int'(e >> VB_AW) - ((a > e[VB_AW-1:0]) ? 1 : 0);",
     "         blo = int'(s >> VB_AW) + ((a < s[VB_AW-1:0]) ? 1 : 0) + 1;\n         bhi = int'(e >> VB_AW) - ((a > e[VB_AW-1:0]) ? 1 : 0) + 1;"),
    ('iinv_snoop', "a store's snoop clear during a range sweep is lost", PC, ['iinv_snoop'],
     "      wire               swst  = ic_sw && !(r_sw_rng && snoop_q);   // a sweep step this cycle",
     "      wire               swst  = ic_sw;"),
    # (not here: a range sweep's end clearing vl_any.  Every fetch during a
    # sweep misses and refills, and the deferred install sets vl_any again
    # as it ends, so a later ICACHE_INV is never skipped in practice — an
    # equivalent mutant; the RTL keeps vl_any anyway)
    ('iinv_ignored', 'RANGE_IINV is not decoded', PC, ['iinv'],
     "   wire        icinv      = icinv_f || icinv_r;", "   wire        icinv      = icinv_f;"),
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
    if suite == 'unit':
        out, killed = '', False
        for seed in (1, 2):
            env.update(SDSIM_WORK=os.path.join(d, 'sdsim'), SDDMA_SEED=str(seed))
            r = subprocess.run(['bash', os.path.join(d, 'tools/sdsim/run_sd_tb.sh'), 'dma'], env=env,
                               capture_output=True, text=True, timeout=3600)
            out += r.stdout + r.stderr
            if r.returncode != 0 or 'TB_SDDMA PASS' not in r.stdout:
                killed = True
                break
        first = next((l for l in out.splitlines() if 'FAIL' in l or 'atal' in l or 'ERROR' in l or 'FATAL' in l), '?')
    else:
        env.update(VSIM_WORK=os.path.join(d, 'vw'))
        r = subprocess.run([sys.executable, os.path.join(d, 'tools/vsim/run.py'), 'socsddma'] + suite, env=env,
                           capture_output=True, text=True, timeout=7200)
        out = r.stdout + r.stderr
        killed = r.returncode != 0
        first = next((l for l in out.splitlines() if l.startswith('FAIL') or 'failed' in l or 'atal' in l), '?')
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
            print(f'{st:9s} {name:14s} {what}\n          {first}', flush=True)
            res.append(st)
    k = res.count('killed')
    print(f'{k} of {len(res)} mutations killed')
    return 0 if k == len(res) else 1


if __name__ == '__main__':
    sys.exit(main())

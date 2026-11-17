"""HW_PERF_PLAN §9 (Fable review fixes): the SoC control FSM's memory
handshake under CPU_RESETN pulses, UART breaks / commands and UART S loads.

Run through tools/vsim/run.py (socreset, socbreak, socsload).  A probe is
appended to the sim copy of tb_soc (a separate build); tb_soc's UART RX
line (uart_rx_tb) is driven by it.  Adapted from the reviewers'
confirm.py (review-tests/fable-memsys).  No traces.

The probe's checks:
- boot-copy ack pairing: every phase-4 ack the FSM takes (seen as phase 4
  left without a reset) must come 1-3 cycles after a D-cache WAIT write
  latch of st.mem_addr (FR BAD-ACK otherwise: a stale ready taken as the
  ack, the boot dword lost);
- hang detection: PIPE_RUN (or, in the HCF cases, the dump's progress)
  within a bound after the stimulus;
- the retire count of the FSM's PIPE_RUN arm (r_instr_count) against the
  pipeline's retires (a break / command cycle used to skip that arm);
- SD DMA / blitter DDR grants while the UART loader runs (LOADING_BYTE),
  after a grace period for the orphaned-grant release.
"""
import os, re, shutil

PROBE = r"""
   // ===== SoC FSM probe (tools/vsim/socfsm.py, HW_PERF_PLAN §9) =====
   longint fr_t = 0;
   always @(posedge dut.i_Clk) fr_t++;
   // --- boot-copy ack pairing (the ack is seen as phase 4 left, no reset)
   longint fr_lat_t = -100; logic [31:0] fr_lat_a = 0;
   int fr_acks = 0, fr_bad = 0;
   bit fr_p4_q = 0; logic [31:0] fr_p4_a = 0; logic [14:0] fr_p4_dw = 0; logic [15:0] fr_p4_ds = 0;
   always @(posedge dut.i_Clk) begin
      if (fr_p4_q && !dut.w_reset_H && dut.st.SM != LOADING_BYTE &&
          !(dut.st.SM == NO_PROGRAM && dut.r_boot_active && dut.r_boot_phase == 3'd4)) begin
         fr_acks++;
         if (!(fr_t - 1 - fr_lat_t <= 3 && fr_lat_a == fr_p4_a)) begin
            fr_bad++;
            $display("FR t=%0d BAD-ACK boot dw=%0d st.mem_addr=%08x dcache_state=%0d last_latch=%08x@%0d",
                     fr_t - 1, fr_p4_dw, fr_p4_a, fr_p4_ds, fr_lat_a, fr_lat_t);
         end
      end
      fr_p4_q  = dut.st.SM == NO_PROGRAM && dut.r_boot_active && dut.r_boot_phase == 3'd4 && dut.w_mem_ready &&
                 !dut.w_reset_H;
      fr_p4_a  = dut.st.mem_addr;  fr_p4_dw = dut.r_boot_dw_addr;  fr_p4_ds = dut.mem_read_write.state;
      if (dut.mem_read_write.state == 16'd2 && dut.mem_read_write.w_dv_new && dut.dram_mem.write_DV &&
          !dut.mem_read_write.r_mnt_pending && !dut.mem_read_write.r_rng_pend) begin
         fr_lat_t = fr_t; fr_lat_a = dut.dram_mem.addr;
      end
      if (dut.w_uart_break)
         $display("FR t=%0d break pulse SM=%0h boot_phase=%0d dw=%0d hcf_sub=%0d w_mem_ready=%b",
                  fr_t, dut.st.SM, dut.r_boot_phase, dut.r_boot_dw_addr, dut.r_hcf_dump_sub, dut.w_mem_ready);
   end
   // --- the FSM's retire count against the pipeline's
   longint fr_ret = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.w_reset_H || dut.st.SM == LOAD_COMPLETE) fr_ret = 0;
      else if (dut.st.SM == PIPE_RUN && dut.pip_ret_valid) fr_ret++;
   end
   // --- HCF memory reads (first few, for the two-pass break sweep)
   int fr_hcf_n = 0; bit fr_hcf_q = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.st.SM == HCF_DUMP && dut.st.mem_read_DV && !fr_hcf_q && fr_hcf_n < 3) begin
         fr_hcf_n++;
         $display("FR t=%0d hcf_read phase=%0d addr=%08x", fr_t, dut.r_hcf_dump_phase, dut.st.mem_addr);
      end
      fr_hcf_q = dut.st.SM == HCF_DUMP && dut.st.mem_read_DV;
   end
   // --- SD DMA / blitter grants while the UART loader runs (after a grace)
   int fr_ld_n = 0, fr_sd_ld = 0, fr_bl_ld = 0, fr_sdw_ld = 0;
   always @(posedge dut.i_Clk) if (dut.st.SM == LOADING_BYTE) begin
      fr_ld_n++;
      if (fr_ld_n > 200) begin
`ifdef SD_DMA_PROBE
         if (dut.mem_read_write.r_grant_sd) fr_sd_ld++;
         if (dut.mem_read_write.r_grant_sd && dut.mem_read_write.mig_write_DV && dut.mem_read_write.i_ddr_mem_ready)
            fr_sdw_ld++;
`endif
         if (dut.mem_read_write.r_grant_blit) fr_bl_ld++;
      end
   end else fr_ld_n = 0;
   // --- the grants held when the S command takes effect, and the arbiter's
   // orphaned-grant releases (§8 item 3d) since
   int fr_orph = 0; bit fr_ld_q = 0; logic [2:0] fr_g_s = 0;
   always @(posedge dut.i_Clk) begin
      if (dut.st.SM == LOADING_BYTE && !fr_ld_q)
         fr_g_s = {
`ifdef SD_DMA_PROBE
                   dut.mem_read_write.r_grant_sd,
`else
                   1'b0,
`endif
                   dut.mem_read_write.r_grant_blit, dut.mem_read_write.r_grant_vga};
      fr_ld_q = dut.st.SM == LOADING_BYTE;
      if (dut.st.SM == LOADING_BYTE && dut.mem_read_write.w_grant_orphaned && dut.mem_read_write.w_orph_rel) begin
         fr_orph++;
         $display("FR t=%0d orphaned grant released (loader, %0d cycles in)", fr_t, fr_ld_n);
      end
   end
   final $display("FR at S grants sd/blit/vga=%b orphan releases=%0d", fr_g_s, fr_orph);
   // grant rises of the SD DMA engine / the blitter while a program runs
   // (the first 60 each: the two-pass placement of an S on a tenure)
   int fr_gr_n = 0; logic [1:0] fr_gr_q = 0;
   int fr_gr_max = 120;
   initial void'($value$plusargs("FR_GRMAX=%d", fr_gr_max));
   always @(posedge dut.i_Clk) if (dut.st.SM == PIPE_RUN) begin
      automatic logic [1:0] g = {
`ifdef SD_DMA_PROBE
         dut.mem_read_write.r_grant_sd,
`else
         1'b0,
`endif
         dut.mem_read_write.r_grant_blit};
      if (g[1] && !fr_gr_q[1] && fr_gr_n < fr_gr_max) begin fr_gr_n++; $display("FR t=%0d grant rise sd", fr_t); end
      if (g[0] && !fr_gr_q[0] && fr_gr_n < fr_gr_max) begin fr_gr_n++; $display("FR t=%0d grant rise blit", fr_t); end
      fr_gr_q = g;
   end
   always @(posedge dut.i_Clk) if (dut.st.SM == LOADING_BYTE && !fr_ld_q) $display("FR t=%0d S takes effect", fr_t);
   // --- §9 review MINOR 2: CTRL START stores reaching the idle blitter
   // (their DV rises, while a program runs: the placement of an S on
   // one), one in the cycle of the S's reset pulse (coverage: the blitter
   // idle, so the store would set its start pulse), and blits the engine
   // starts while the loader runs (each one is a failure)
   int fr_bs_n = 0, fr_bl_st_ld = 0; bit fr_bs_q = 0, fr_bb_q = 0, fr_bs_s = 0;
   always @(posedge dut.i_Clk) begin
      automatic bit c = dut.w_blit_write_DV && dut.w_mmio_addr[15:0] == 16'h0000 && dut.w_mmio_write_data[0] &&
                        !dut.blitter_dma_i.r_busy;
      if (dut.st.SM == PIPE_RUN && c && !fr_bs_q && fr_bs_n < fr_gr_max) begin
         fr_bs_n++; $display("FR t=%0d blit start store", fr_t);
      end
      fr_bs_q = c;
      if (dut.r_s_rst && c) fr_bs_s = 1;
      // (one started in the cycle of the reset pulse dies with it)
      if (dut.st.SM == LOADING_BYTE && dut.blitter_dma_i.r_busy && !fr_bb_q && !dut.r_s_rst) begin
         fr_bl_st_ld++; $display("FR t=%0d blit started while the loader runs", fr_t);
      end
      fr_bb_q = dut.blitter_dma_i.r_busy;
   end
   final $display("FR blit starts while loading=%0d start store at S=%0d", fr_bl_st_ld, fr_bs_s);
   final $display("FR final retired fsm=%0d tb=%0d acks=%0d bad_acks=%0d ld_sd_grant=%0d ld_sd_wr=%0d ld_blit_grant=%0d grants vga=%0d blit=%0d sd=%0d",
                  dut.r_instr_count, fr_ret, fr_acks, fr_bad, fr_sd_ld, fr_sdw_ld, fr_bl_ld,
                  dut.mem_read_write.r_grant_vga, dut.mem_read_write.r_grant_blit,
`ifdef SD_DMA_PROBE
                  dut.mem_read_write.r_grant_sd);
`else
                  1'b0);
`endif
   // --- UART driver (CLKS_PER_BIT = 33)
   task automatic fr_bit(input logic v); uart_rx_tb = v; repeat (33) @(posedge dut.i_Clk); endtask
   task automatic fr_byte(input logic [7:0] b);
      fr_bit(0); for (int i = 0; i < 8; i++) fr_bit(b[i]); fr_bit(1); fr_bit(1);
   endtask
   task automatic fr_break(input int n);
      uart_rx_tb = 0; repeat (n) @(posedge dut.i_Clk); uart_rx_tb = 1; repeat (200) @(posedge dut.i_Clk);
   endtask
   // send a file; after `split` characters (0: none) a break and 'A' (an
   // ignored command) are inserted
   task automatic fr_send_file(input string fn, input int split);
      int fd, c, n; fd = $fopen(fn, "r");
      if (fd == 0) begin $display("FR cannot open %s", fn); $finish; end
      c = $fgetc(fd); n = 0;
      while (c >= 0) begin
         fr_byte(8'(c)); n++;
         if (split != 0 && n == split) begin fr_break(400); fr_byte(8'h41); end
         c = $fgetc(fd);
      end
      $fclose(fd);
      $display("FR t=%0d kbt sent", fr_t);
   endtask
   // a CPU_RESETN press of exactly n board cycles (driven on the falling edge)
   task automatic fr_press(input int n);
      @(negedge clk100); cpu_resetn = 1'b0; repeat (n) @(negedge clk100); cpu_resetn = 1'b1;
   endtask
   task automatic fr_state(input string tag);
      $display("FR %s t=%0d SM=%0h boot_active=%b phase=%0d dw=%0d st.mem_write_DV=%b st.mem_read_DV=%b st.mem_addr=%08x dstate=%0d dv_served=%b served_addr=%08x hcf_phase=%0d hcf_sub=%0d acks=%0d bad_acks=%0d",
               tag, fr_t, dut.st.SM, dut.r_boot_active, dut.r_boot_phase, dut.r_boot_dw_addr, dut.st.mem_write_DV,
               dut.st.mem_read_DV, dut.st.mem_addr, dut.mem_read_write.state, dut.mem_read_write.r_dv_served,
               dut.mem_read_write.r_served_addr, dut.r_hcf_dump_phase, dut.r_hcf_dump_sub, fr_acks, fr_bad);
   endtask
   task automatic fr_wait_run(input int n, output bit ok);
      ok = 0;
      for (int i = 0; i < n && !ok; i++) begin @(posedge dut.i_Clk); ok = dut.st.SM == PIPE_RUN; end
   endtask

   int fr_mode = 0, fr_k = 1, fr_l1 = 3, fr_h1 = 0, fr_l2 = 0, fr_dw = 0, fr_off = 0, fr_brk = 400, fr_nb = 0;
   int fr_split = 0, fr_what = 0;
   longint fr_at = 0;
   string fr_kbt = "";
   bit fr_ok, fr_fin;
   int fr_seen = 0; bit fr_cq = 0, fr_go = 0;
   // mode 1 trigger: the k-th start of a D-cache read miss in flight and not yet answered, in PIPE_RUN
   always @(posedge dut.i_Clk) if (fr_mode == 1 && !fr_go && dut.st.SM == PIPE_RUN) begin
      automatic bit c = dut.mem_read_write.state == 16'd4096 && !dut.mem_read_write.r_miss_served;
      if (c && !fr_cq) begin fr_seen++; if (fr_seen == fr_k) fr_go = 1; end
      fr_cq = c;
   end
   initial begin
      void'($value$plusargs("FR_MODE=%d", fr_mode));
      void'($value$plusargs("FR_K=%d", fr_k));
      void'($value$plusargs("FR_L1=%d", fr_l1));
      void'($value$plusargs("FR_H1=%d", fr_h1));
      void'($value$plusargs("FR_L2=%d", fr_l2));
      void'($value$plusargs("FR_DW=%d", fr_dw));
      void'($value$plusargs("FR_OFF=%d", fr_off));
      void'($value$plusargs("FR_BRK=%d", fr_brk));
      void'($value$plusargs("FR_NB=%d", fr_nb));
      void'($value$plusargs("FR_AT=%d", fr_at));
      void'($value$plusargs("FR_SPLIT=%d", fr_split));
      void'($value$plusargs("FR_WHAT=%d", fr_what));
      void'($value$plusargs("FR_KBT=%s", fr_kbt));
      fr_fin = $test$plusargs("FR_FIN");
      if (fr_mode inside {[1:5]}) begin
         if (fr_mode == 1) begin
            wait (fr_go);
         end else if (fr_mode == 2 || fr_mode == 4) begin
            // the power-up boot copy: dword fr_dw entering phase 3, + fr_off cycles
            wait (dut.st.SM == NO_PROGRAM && dut.r_boot_active && dut.r_boot_phase == 3'd3 && dut.r_boot_dw_addr == 15'(fr_dw));
            repeat (fr_off) @(posedge dut.i_Clk);
         end else if (fr_mode == 3) begin
            // the boot copy with a boot write miss in the D-cache's shadow (WRITE_FETCH) at dword >= fr_dw
            wait (dut.st.SM == NO_PROGRAM && dut.r_boot_dw_addr >= 15'(fr_dw) && dut.mem_read_write.state == 16'd128);
            repeat (fr_off) @(posedge dut.i_Clk);
         end else if (fr_mode == 5) begin
            // the crash dump: its first DDR read (st.mem_read_DV), + fr_off cycles
            wait (dut.st.SM == HCF_DUMP && dut.st.mem_read_DV);
            repeat (fr_off) @(posedge dut.i_Clk);
         end
         if (fr_mode == 4) begin
            $display("FR t=%0d break starts (%0d cycles low, off %0d)", fr_t, fr_brk, fr_off);
            fr_break(fr_brk);
         end else begin
            fr_state("before-press");
            $display("FR t=%0d press: low %0d, high %0d, low %0d", fr_t, fr_l1, fr_h1, fr_l2);
            fr_press(fr_l1);
            if (fr_l2 != 0) begin repeat (fr_h1 - 1) @(negedge clk100); fr_state("before-press-2"); fr_press(fr_l2); end
         end
         fr_wait_run(400000, fr_ok);
         fr_state(fr_ok ? "RAN" : "HUNG");
         if (!fr_ok) begin $display("FR RESULT hang"); $finish; end
         else if (fr_fin) begin $display("FR RESULT ok"); $finish; end
         // otherwise the program runs to HALTED (tb_soc prints REGS)
      end else if (fr_mode == 6) begin
         // a UART break starting at cycle fr_at (0: none) — the crash dump
         // must progress past the phase it was in when the pulse came
         automatic int ph;
         wait (dut.st.SM == HCF_DUMP);
         if (fr_at != 0) begin
            wait (fr_t >= fr_at);
            fr_break(fr_brk);
         end else
            wait (fr_hcf_n > 0);
         ph = dut.r_hcf_dump_phase;
         for (int i = 0; i < 200000 && dut.r_hcf_dump_phase <= ph + 1; i++) @(posedge dut.i_Clk);
         if (dut.r_hcf_dump_phase > ph + 1) begin fr_state("DUMP-PROGRESS"); $display("FR RESULT ok"); end
         else begin fr_state("DUMP-HUNG"); $display("FR RESULT hang"); end
         $finish;
      end else if (fr_mode == 7) begin
         // fr_nb breaks, each followed by 'A' (an ignored command), while the program runs
         wait (dut.st.SM == PIPE_RUN);
         repeat (2000) @(posedge dut.i_Clk);
         for (int i = 0; i < fr_nb; i++) begin
            fr_break(fr_brk + 37 * i);
            fr_byte(8'h41);
            repeat (100 + 13 * i) @(posedge dut.i_Clk);
         end
         $display("FR t=%0d breaks sent", fr_t);
      end else if (fr_mode == 8) begin
         // break + S + .kbt (with fr_split: a break and 'A' inside the stream)
         // once the running program has an SD DMA (fr_what = 1) or blitter
         // (2) grant, or at once (0)
         wait (dut.st.SM == PIPE_RUN);
`ifdef SD_DMA_PROBE
         if (fr_what == 1) wait (dut.mem_read_write.r_grant_sd);
`endif
         if (fr_what == 2) wait (dut.mem_read_write.r_grant_blit);
         repeat (2000 + fr_off) @(posedge dut.i_Clk);
         if ($test$plusargs("FR_NOS")) begin
            // (no load: the DMA's grant pattern from here on, for the
            // two-pass placement of an S on a tenure)
            repeat (12000) @(posedge dut.i_Clk);
            $display("FR t=%0d no-S run ends", fr_t);
            $finish;
         end
         $display("FR t=%0d break + load (what=%0d)", fr_t, fr_what);
         fr_break(2000);
         fr_send_file(fr_kbt, fr_split);
      end
   end
"""

MARK, MAGIC = 0x300000, 0x13579BDF
# socsload blits: the S placed this many cycles from a START store's DV rise
BLITS_D = tuple(int(x) for x in os.environ.get('BLITS_D', '-2,-1,0,1,2').split(','))
FR = ('fr', PROBE)


def _run():
    import run
    return run


def prog_rd_loop():
    """1st boot: store MARK, corrupt dword 0 with 0x5A5A5A5A, then loop on
    DRAM read misses for ever.  A boot that finds MARK halts with r5 = the
    dword 0 it booted with (must equal the image length = ROM dword 0)."""
    from kasm import Asm
    a = Asm()
    a.setr(11, 0); a.setr(7, MARK); a.setr(8, MAGIC)
    a.ld(64, 5, 11, 0)
    a.ld(64, 6, 7, 0)
    a.fbr('eq', 6, 8, 'done')
    a.st(64, 8, 7, 0)
    a.setr(1, 0x5A5A5A5A); a.st(64, 1, 11, 0)
    a.setr(10, 0x100000); a.setr(9, 0)
    a.label('top')
    a.alu('add', 12, 10, 9)
    a.ld(64, 2, 12, 0); a.ld(64, 3, 12, 0x1000)
    a.setr(13, 37 * 64); a.alu('add', 9, 9, 13)
    a.setr(13, 0x80000 - 1); a.alu('and', 9, 9, 13)
    a.jmp('top')
    a.label('done')
    a.halt()
    return a


def image_len(a, path):
    a.write_boot_mem(path)
    return int(open(path).readline(), 16)


def kbt_words(words, data, path):
    """klausscc .kbt wire string (to_kbt in klausscc src/output/image.rs):
    'S', header + program words (+ dword data), entry, 'Z', checksum, 'X'"""
    mem = {0x20 + 4 * i: w for i, w in enumerate(words)}
    for ad, v in data.items():
        mem[ad] = v & 0xFFFFFFFF; mem[ad + 4] = v >> 32
    end = max(mem) + 4
    heap = (end + 7) & ~7
    stream = [heap] + [0] * 7 + [mem.get(x, 0) for x in range(0x20, end, 4)] + [0x20]
    s = 'S' + ''.join(w.to_bytes(4, 'little').hex().upper() for w in stream)
    chk = (sum(((w >> 16) + (w & 0xFFFF)) for w in stream) + 2 * len(stream)) & 0xFFFF
    s += 'Z' + chk.to_bytes(4, 'little').hex().upper() + 'X'
    open(path, 'w').write(s)
    return path


def kbt(a, path):
    return kbt_words(a.assemble(), a.data, path)


def tiny_loaded():
    from kasm import Asm
    a = Asm(); a.setr(5, 0xC0FFEE); a.halt(); return a


def regs(log):
    m = re.search(r'TB_SOC: REGS(.*)', log)
    return {k: int(v, 16) for k, v in re.findall(r'(r\d+)=([0-9a-f]{16})', m.group(1))} if m else None


def final(log):
    m = re.search(r'FR final retired fsm=(\d+) tb=(\d+) acks=(\d+) bad_acks=(\d+) ld_sd_grant=(\d+) ld_sd_wr=(\d+) '
                  r'ld_blit_grant=(\d+) grants vga=(\d) blit=(\d) sd=(\d)', log)
    if not m:
        return None
    k = ('fsm', 'tb', 'acks', 'bad', 'ld_sd', 'ld_sdw', 'ld_bl', 'g_vga', 'g_blit', 'g_sd')
    return dict(zip(k, (int(x) for x in m.groups())))


def go(exe, img, tag, plus, keep=False):
    run = _run()
    r = run.run_soc(exe, img, tag, timeout=3600, gate=False,
                    plus=list(plus) + ['+MAXI=3000000', '+IDLEMAX=60000000'])
    if not keep:
        shutil.rmtree(os.path.join(run.WORK, 'socruns', tag), ignore_errors=True)
    return r


def _report(tag, bad, r, verbose):
    if bad or verbose:
        print(f'{"FAIL" if bad else "pass"} {tag}: ' + '; '.join(bad))
        if bad or verbose:
            for l in r['log'].splitlines():
                if l.startswith('FR ') and ('BAD-ACK' in l or 'RESULT' in l or 'HUNG' in l or 'RAN' in l
                                            or 'final' in l or 'PROGRESS' in l) or 'FATAL' in l or 'DEADLOCK' in l:
                    print('     ', l[:240])


def _common_bad(r, f, need_halt):
    bad = []
    if f is None:
        bad.append('no FR final line')
    elif f['bad']:
        bad.append(f'{f["bad"]} boot acks not paired with a D-cache write latch (stale ready: dword lost)')
    if 'FR RESULT hang' in r['log']:
        bad.append('hang')
    if 'FATAL' in r['log']:
        bad.append('FATAL: ' + ' '.join(l for l in r['log'].splitlines() if 'FATAL' in l)[:160])
    if 'DEADLOCK' in r['log']:
        bad.append('DEADLOCK')
    if need_halt and 'HALTED' not in r['log']:
        bad.append('no HALTED')
    return bad


def cmd_socreset(a):
    """§9 item 1: CPU_RESETN pulse shapes against the boot FSM and the
    never-reset D-cache (finding memsys-1).  A press of 1-30 cycles while a
    read miss is in flight (the dead read's ready must not be taken as boot
    dword 0's ack); a short press, a 6-15-cycle release, a long press (a DV
    surviving the reset at address 0 used to hang for good); release
    bounce during the power-up copy; two glitches while a boot write miss
    is in the D-cache's shadow; a press during the crash dump's DDR reads.
    Every run must reach PIPE_RUN, pair every boot ack with a D-cache write
    latch, and (read-miss cases) boot with the ROM's dword 0."""
    run = _run()
    exe = run.build_soc(a.rev, tb_wt=bool(a.rev), probe=FR)
    os.makedirs(run.WORK, exist_ok=True)
    tmp = os.path.join(run.WORK, 'img_fr.mem')
    small = prog_rd_loop()
    big = prog_rd_loop(); big.dword(0x4000, 0x1234)       # a copy longer than 100 dwords
    want_small, want_big = image_len(small, tmp), image_len(big, tmp)
    from kasm import Asm
    hcf = Asm()
    for r in range(1, 5):
        hcf.setr(r, 0x1111 * r); hcf.push(r)
    hcf.trap()
    cases = []
    for n in (1, 2, 3, 5, 10, 20, 30):
        cases.append((f'lone_{n}', small, ['+FR_MODE=1', '+FR_K=5', f'+FR_L1={n}'], True))
    for n in (40, 200):
        cases.append((f'ctl_{n}', small, ['+FR_MODE=1', '+FR_K=5', f'+FR_L1={n}'], True))
    for h in (2, 4, 6, 10, 15, 30, 100):
        cases.append((f'b_3_{h}_200', small, ['+FR_MODE=1', '+FR_K=5', '+FR_L1=3', f'+FR_H1={h}', '+FR_L2=200'], True))
    cases.append(('b_3_15_1', small, ['+FR_MODE=1', '+FR_K=5', '+FR_L1=3', '+FR_H1=15', '+FR_L2=1'], True))
    cases.append(('b_1_8_1', small, ['+FR_MODE=1', '+FR_K=7', '+FR_L1=1', '+FR_H1=8', '+FR_L2=1'], True))
    for h in range(2, 9):
        cases.append((f'rel_dw40_h{h}', big, ['+FR_MODE=2', '+FR_DW=40', '+FR_L1=1', f'+FR_H1={h}', '+FR_L2=1', '+FR_FIN'], False))
    for off in range(0, 4):
        cases.append((f'p2_dw40_o{off}', big, ['+FR_MODE=2', '+FR_DW=40', f'+FR_OFF={off}', '+FR_L1=40', f'+FR_H1={2 + off}',
                                              '+FR_L2=1', '+FR_FIN'], False))
    for h in (2, 3, 4, 6, 8, 16, 24, 40):
        cases.append((f'wf_h{h}', big, ['+FR_MODE=3', '+FR_DW=100', '+FR_L1=1', f'+FR_H1={h}', '+FR_L2=1', '+FR_FIN'], False))
    for off in (0, 1, 2, 3, 5):
        for n in (1, 3):
            cases.append((f'hcf_o{off}_{n}', hcf, ['+FR_MODE=5', f'+FR_OFF={off}', f'+FR_L1={n}', '+FR_FIN'], False))
    fails = n = 0
    for tag, img, plus, halt in cases:
        if a.only and not any(re.fullmatch(p, tag) for p in a.only):
            continue
        r = go(exe, img, 'fr_' + tag, plus)
        f = final(r['log'])
        bad = _common_bad(r, f, halt)
        if halt:
            rg = regs(r['log'])
            want = want_big if img is big else want_small
            if rg is None or rg['r5'] != want:
                bad.append(f'booted with dword 0 = {rg and hex(rg["r5"])}, the ROM has {want:#x} (boot dword 0 lost)')
        elif 'FR RESULT ok' not in r['log']:
            bad.append('no RESULT ok')
        n += 1
        fails += bool(bad)
        _report(tag, bad, r, a.verbose)
    print(f'socreset: {fails} failing of {n}')
    return 1 if fails else 0


def _hcf_prog():
    from kasm import Asm
    a = Asm()
    for r in range(1, 5):
        a.setr(r, 0x1111 * r); a.push(r)
    a.trap()
    return a


def cmd_socbreak(a):
    """§9 item 2: a UART break or command byte in a one-cycle ack cycle
    (finding memsys-2).  The break swept over 48 cycle offsets through the
    power-up boot copy (the copy must finish, and the program run); over
    the crash dump's DDR reads (the dump must progress; two passes: the
    first finds the read's cycle); breaks + 'A' while a program runs (the
    FSM's retire count must equal the pipeline's: the break cycle used to
    skip the PIPE_RUN arm); a break + 'A' inside a UART load's stream (the
    command byte must not be loaded as data).  Coverage: some break pulse
    must land on a boot ack and on an HCF read ack."""
    run = _run()
    exe = run.build_soc(a.rev, tb_wt=bool(a.rev), probe=FR)
    from kasm import Asm
    fails = n = 0
    cov_boot = cov_hcf = 0
    boot = Asm(); boot.setr(5, 0x77); boot.halt()
    boot.dword(0x4000, 0x1234)            # a 16 KiB image: ~2k boot-copy dwords

    def sel(tag):
        return not a.only or any(re.fullmatch(p, tag) for p in a.only)

    def note(tag, bad, r):
        nonlocal fails, n
        n += 1
        fails += bool(bad)
        _report(tag, bad, r, a.verbose)

    for off in range(0, 48):
        tag = f'boot_o{off}'
        if not sel(tag):
            continue
        r = go(exe, boot, 'fr_' + tag, ['+FR_MODE=4', '+FR_DW=40', f'+FR_OFF={off}'])
        f = final(r['log'])
        bad = _common_bad(r, f, True)
        rg = regs(r['log'])
        if rg is None or rg['r5'] != 0x77:
            bad.append(f'r5 = {rg and hex(rg["r5"])} (want 0x77)')
        if re.search(r'break pulse SM=\S+ boot_phase=4 dw=\d+ hcf_sub=\d+ w_mem_ready=1', r['log']):
            cov_boot += 1
        note(tag, bad, r)
    # the crash dump: pass 1 finds the first HCF DDR read's cycle
    hcf = _hcf_prog()
    if sel('hcf_o0') or not a.only or any(t.startswith('hcf') for t in a.only):
        r = go(exe, hcf, 'fr_hcf_pass1', ['+FR_MODE=6'])
        m = re.search(r'FR t=(\d+) hcf_read', r['log'])
        if not m or 'FR RESULT ok' not in r['log']:
            note('hcf_pass1', ['no HCF read / the dump did not progress without a break'], r)
        else:
            t_r = int(m.group(1))
            for off in range(0, 40):
                tag = f'hcf_o{off}'
                if not sel(tag):
                    continue
                r = go(exe, hcf, 'fr_' + tag, ['+FR_MODE=6', f'+FR_AT={t_r - 340 + off}'])
                bad = _common_bad(r, final(r['log']), False)
                if 'FR RESULT ok' not in r['log']:
                    bad.append('the crash dump did not progress')
                if re.search(r'break pulse SM=\S+ boot_phase=\d+ dw=\d+ hcf_sub=3 w_mem_ready=1', r['log']):
                    cov_hcf += 1
                note(tag, bad, r)
    # breaks + 'A' while a program runs: the FSM's retire count
    run_p = Asm()
    run_p.setr(9, 60000); run_p.setr(5, 0)
    run_p.label('l'); run_p.inc(5); run_p.dec(9); run_p.fbr('ne', 9, 0, 'l', imm=True)
    run_p.halt()
    for nb in (4, 9):
        tag = f'run_nb{nb}'
        if not sel(tag):
            continue
        r = go(exe, run_p, 'fr_' + tag, ['+FR_MODE=7', f'+FR_NB={nb}'])
        f = final(r['log'])
        bad = _common_bad(r, f, True)
        rg = regs(r['log'])
        if rg is None or rg['r5'] != 60000:
            bad.append(f'r5 = {rg and rg["r5"]} (want 60000)')
        if f and f['fsm'] != f['tb']:
            bad.append(f'FSM retire count {f["fsm"]} != the pipeline\'s {f["tb"]} (a break / command cycle skipped PIPE_RUN)')
        note(tag, bad, r)
    # a break + 'A' inside a UART load's stream: ignored, the load succeeds
    os.makedirs(run.WORK, exist_ok=True)
    kp = kbt(tiny_loaded(), os.path.join(run.WORK, 'fr_tiny.kbt'))
    spin = Asm(); spin.label('s'); spin.jmp('s')
    for split in (5, 30, 60):
        tag = f'load_split{split}'
        if not sel(tag):
            continue
        r = go(exe, spin, 'fr_' + tag, ['+FR_MODE=8', f'+FR_KBT={kp}', f'+FR_SPLIT={split}'])
        bad = _common_bad(r, final(r['log']), True)
        rg = regs(r['log'])
        if rg is None or rg['r5'] != 0xC0FFEE:
            bad.append(f'the loaded program did not run: r5 = {rg and hex(rg["r5"])}')
        note(tag, bad, r)
    if not a.only:
        if not cov_boot:
            print('FAIL socbreak coverage: no break pulse landed on a boot-copy ack')
            fails += 1
        if not cov_hcf:
            print('FAIL socbreak coverage: no break pulse landed on an HCF read ack')
            fails += 1
    print(f'socbreak: {fails} failing of {n} (break pulses on a boot ack: {cov_boot}, on an HCF read ack: {cov_hcf})')
    return 1 if fails else 0


def _sload_image():
    """the loaded program: a 0x800-byte image whose data at 0x400.. is a
    known pattern; r12 = the dwords at 0x400..0x800 that differ from it;
    r6 = the blitter's STATUS (BUSY = bit 0)"""
    from kasm import Asm
    b = Asm()
    b.setr(12, 0); b.setr(10, 0x400); b.setr(9, 0x80)
    b.label('l'); b.ld(64, 2, 10, 0)
    b.fbr('eq', 2, 10, 'ok'); b.inc(12)
    b.label('ok'); b.addi_s(10, 10, 8); b.dec(9); b.fbr('ne', 9, 0, 'l', imm=True)
    b.setr(13, 0xF00E0000); b.ld(64, 6, 13, 0x08)
    b.halt()
    for ad in range(0x400, 0x800, 8):
        b.dword(ad, ad)
    return b


def cmd_socsload(a):
    """§9 item 3: a UART S load stops a running SD DMA and blitter
    (finding memsys-3).  The running program has a 512-block SD DMA read
    into 0x400 (inside the new image), or back-to-back blitter FILLs; then
    break + S + a .kbt load.  No SD / blitter DDR grant may be held while
    the loader runs (after a 200-cycle grace for the orphaned-grant
    release), no grant may be held at the end, the loaded image must be
    intact, and the loaded program must run (the blitter idle)."""
    run = _run()
    import sddma
    from kasm import Asm
    exe = run.build_soc(a.rev, tb_wt=bool(a.rev), probe=FR)
    os.makedirs(run.WORK, exist_ok=True)
    kp = kbt(_sload_image(), os.path.join(run.WORK, 'fr_sload.kbt'))
    s = sddma.SdAsm()
    s.prologue()
    s.dma(18, 0, 0x400, 512, 512, 0)
    s.label('spin'); s.jmp('spin')
    bl = Asm()
    bl.setr(13, 0xF00E0000)
    for off, v in ((0x10, 0x500000), (0x18, 1280), (0x30, 640), (0x38, 480), (0x40, 0x1234)):
        bl.setr(1, v); bl.st(64, 1, 13, off)
    bl.label('top')
    bl.ld(64, 4, 13, 0x08); bl.setr(12, 1); bl.alu('and', 4, 4, 12)   # STATUS.BUSY
    bl.fbr('ne', 4, 0, 'top', imm=True)
    bl.setr(4, 2); bl.st(64, 4, 13, 0x08)                             # DONE: clear
    bl.setr(4, 0x11); bl.st(64, 4, 13, 0x00)                          # START, FILL, IRQ_EN
    bl.jmp('top')
    # §9 review MINOR 2: back-to-back 16-pixel FILLs (no IRQ), a START store
    # every few dozen cycles; the S is placed on their DVs (cases blits_*)
    bs = Asm()
    bs.setr(13, 0xF00E0000)
    for off, v in ((0x10, 0x500000), (0x18, 1280), (0x30, 16), (0x38, 1), (0x40, 0x1234)):
        bs.setr(1, v); bs.st(64, 1, 13, off)
    bs.setr(12, 1); bs.setr(11, 2)
    bs.label('top')
    bs.ld(64, 4, 13, 0x08); bs.alu('and', 4, 4, 12)                    # STATUS.BUSY
    bs.fbr('ne', 4, 0, 'top', imm=True)
    bs.st(64, 11, 13, 0x08)                                           # DONE: clear
    bs.st(64, 12, 13, 0x00)                                           # START, FILL
    bs.jmp('top')
    kt = kbt(tiny_loaded(), os.path.join(run.WORK, 'fr_tiny.kbt'))
    fails = n = 0
    cov = {'sd': 0, 'blit': 0, 'blits': 0}
    cases = [('sd', s, 1, 0, kp), ('blit', bl, 2, 0, kp)]
    # the S placed 2 / 5 / 9 cycles after later grant rises (a short load):
    # the run is deterministic up to the S, so pass 1 (offset 0) gives the
    # grant rises and the S's own cycle
    for tag, img, what in (('sd', s, 1), ('blit', bl, 2)):
        if a.only and tag not in a.only:
            continue
        r = go(exe, img, f'fr_sload_{tag}_p1', ['+FR_MODE=8', f'+FR_WHAT={what}', f'+FR_KBT={kt}'])
        ts = re.search(r'FR t=(\d+) S takes effect', r['log'])
        r = go(exe, img, f'fr_sload_{tag}_p1n', ['+FR_MODE=8', f'+FR_WHAT={what}', '+FR_NOS', '+FR_GRMAX=100000'])
        rises = [int(x) for x in re.findall(rf'FR t=(\d+) grant rise {tag}\b', r['log'])]
        later = [g for g in rises if ts and g > int(ts.group(1)) + 50][:3] if ts else []
        if len(later) < 3:
            print(f'FAIL socsload {tag}: pass 1 found no grant rises after the S ({len(rises)} rises)')
            fails += 1
        for g, d in zip(later, (2, 5, 9)):
            cases.append((tag, img, what, g + d - int(ts.group(1)), kt))
    # the S's reset pulse placed around the first cycles of later START
    # stores' DVs (before the §9 review fix, the one in the pulse's cycle
    # started a blit while the loader ran)
    if not a.only or 'blits' in a.only:
        r = go(exe, bs, 'fr_sload_blits_p1', ['+FR_MODE=8', '+FR_WHAT=2', f'+FR_KBT={kt}'])
        ts = re.search(r'FR t=(\d+) S takes effect', r['log'])
        r = go(exe, bs, 'fr_sload_blits_p1n', ['+FR_MODE=8', '+FR_WHAT=2', '+FR_NOS', '+FR_GRMAX=100000'])
        sts = [int(x) for x in re.findall(r'FR t=(\d+) blit start store', r['log'])]
        later = [g for g in sts if ts and g > int(ts.group(1)) + 50][:2] if ts else []
        if len(later) < 2:
            print(f'FAIL socsload blits: pass 1 found no START stores after the S ({len(sts)} stores)')
            fails += 1
        for g in later:
            for d in BLITS_D:
                cases.append(('blits', bs, 2, g + d - int(ts.group(1)), kt))
    for tag, img, what, off, kf in cases:
        if a.only and tag not in a.only:
            continue
        r = go(exe, img, f'fr_sload_{tag}_{off}', ['+FR_MODE=8', f'+FR_WHAT={what}', f'+FR_KBT={kf}', f'+FR_OFF={off}'])
        f = final(r['log'])
        bad = _common_bad(r, f, True)
        if f:
            if f['ld_sd'] or f['ld_bl']:
                bad.append(f'DMA grants while the loader ran: sd={f["ld_sd"]} cycles ({f["ld_sdw"]} DDR writes), '
                           f'blit={f["ld_bl"]} cycles')
            if f['g_vga'] or f['g_blit'] or f['g_sd']:
                bad.append(f'a grant held at the end: vga={f["g_vga"]} blit={f["g_blit"]} sd={f["g_sd"]}')
        mb = re.search(r'FR blit starts while loading=(\d+) start store at S=(\d)', r['log'])
        if not mb:
            bad.append('no blit-starts line')
        else:
            if int(mb.group(1)):
                bad.append(f'{mb.group(1)} blits started while the loader ran')
            if tag == 'blits':
                cov['blits'] += mb.group(2) == '1'
        m = re.search(r'FR at S grants sd/blit/vga=([01]{3}) orphan releases=(\d+)', r['log'])
        if not m:
            bad.append('no grant-at-S line')
        else:
            held = m.group(1)[what - 1] == '1'
            if tag != 'blits':
                cov[tag] += held
            # (the blitter's reset drops its req mid-tenure: the arbiter's
            # orphaned-grant guard must release the grant.  The SD engine's
            # DDR adapter is not reset — it finishes its one transaction and
            # releases with its done — so SD leaves no orphan; either way
            # the grant-free check above covers the release.)
            if held and what == 2 and int(m.group(2)) == 0:
                bad.append('the blitter grant held at S was not released by the orphaned-grant guard')
        rg = regs(r['log'])
        if rg is None:
            bad.append('the loaded program did not halt')
        elif kf is kp:
            if rg['r12']:
                bad.append(f'{rg["r12"]} dwords of the loaded image overwritten')
            if rg['r6'] & 1:
                bad.append(f'the blitter still busy in the loaded program (STATUS={rg["r6"]:#x})')
        elif rg['r5'] != 0xC0FFEE:
            bad.append(f'the loaded program did not run: r5 = {rg["r5"]:#x}')
        n += 1
        fails += bool(bad)
        _report(f'{tag}_{off}', bad, r, a.verbose)
    for k, v in cov.items():
        if not a.only or k in a.only:
            if not v:
                print(f'FAIL socsload coverage: ' + ('no S pulse found a START store' if k == 'blits'
                                                     else f'no S found the {k} grant held'))
                fails += 1
    print(f'socsload: {fails} failing of {n} (runs whose S found the grant held: sd {cov["sd"]}, blit {cov["blit"]}; '
          f'whose S pulse found a START store: blits {cov["blits"]})')
    return 1 if fails else 0

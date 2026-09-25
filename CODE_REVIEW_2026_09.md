# Code Review 2026-09 — Findings, Fixes, Deferrals

External review pass over the pipeline core, memory system, and SoC
(2026-09-25). This file records what was found, what was fixed in this pass,
how each fix was verified, and what was deliberately deferred with reasons.
Everything below is sim-verified; board verification (m5e suite + a Zephyr
soak for the IRQ fix) is still owed before the next QSPI flash.

## Fixed in this pass

### 1. Interrupt could dispatch inside `irq_lock` (RTOS-affecting)
`pipeline_core.sv`, IRQ entry sequencer. The core decided to take an
interrupt at a dispatch boundary, then drained the pipeline before pushing
the frame. An older in-flight `INT_MASK` store (Zephyr `irq_lock`) landed
during the drain — and the frame was pushed anyway with the mask already 0,
running the handler inside the critical section. The same window dispatched
a spurious blitter IRQ when an older store acked the level-triggered DONE.
**Fix:** re-check `irq_ready` at the drain boundary; abandon the entry
(fetch resumes at `pc`) if the cause is gone, re-latch `irq_sel`/vector if
not (the drained stores may have retargeted the priority source).
**Verified:** new directed `TEST=maskrace` in `tb_pipeline_isa.sv`
(irq_lock/unlock loop under prime-period storms, oracle = ack must not fire
with `irq_ready` low at the push). Pre-fix core: FAIL (3 and 57 in-lock
dispatches on the two sweeps). Fixed core: PASS. Existing IRQ-storm trace
identity (bst, test_64bit) still passes. `FPGA_HANDOFF_IRQ_RESPONSE.md`
updated — its "zero-window" claim was FSM-era and holds again only via this
re-check.

### 2. Blitter reset mid-blit wedged DDR forever
`mem_read_write.sv` arbiter. The grant only released on the master's `done`
pulse; CPU_RESETN mid-blit reset the blitter (clearing req/done) with the
grant still out — every later cache miss parked forever. Core 2's DDR
adapter had the same exposure (its "board reset ONLY" comment guards the
`!r_run` soft-stop, not the button — `i_Rst_L` IS CPU_RESETN).
**Fix:** orphaned-grant guard. Both masters hold `req` high for a whole
tenure and drop it only in the same cycle as `done`, so granted + `req`=0 +
no `done` can only mean the master was reset; after a 15-cycle drain (lets
any in-flight DDR burst finish) the grant is force-released.

### 3. Cache-maintenance races
`mem_read_write.sv`. (a) A FLUSH/INVALIDATE pulse arriving while a walk was
already running was silently dropped — lines dirtied after the walk started
were never written back, though software believed they were. (b) The
blitter could be granted a whole chunk tenure after a flush was requested
but before its walk started (`r_mnt_active` checked instead of the full
busy condition).
**Fix:** the pending flag is now cleared only at the WAIT→MAINT consume
edge, so a mid-walk pulse re-arms it and a fresh walk follows; the running
walk latches its own `r_mnt_walk_mode` copy (a mid-walk pulse can no longer
retarget it — the live `r_mnt_mode` was read in MS_CLR); B/C grants gate on
`o_mnt_busy` (active | pending). A pending INVALIDATE is never downgraded
by a later FLUSH pulse.

### 4. TRNG returned the wrong FIFO entry
`trng.sv`. The FIFO popped on the read strobe's rising edge, but the SoC's
MMIO read pipeline samples the device's combinational data one cycle later
— the first read returned the empty slot (0), later reads ran one entry
behind, and an entry could be returned twice.
**Fix:** pop off the delayed strobe (the pattern the UART RX pop already
uses). Also: RCT cutoff 32→64 (32 false-tripped every ~21 s at 10⁸
windows/s), and a tripped health check now BLOCKS output (pushes are
suppressed while `r_rct_fail` is latched; READY drops once the FIFO
drains). MMIO_MAP.md updated.

### 5. CPU_RESETN was raw and unsynchronized
`KlaussCPU.sv` + XDC. The raw button drove the huge synchronous-reset
fanout; `set_false_path` hid the hazard that the release edge could land in
different FFs' setup windows on different cycles. **Fix:** 2-FF synchronizer
(`r_rstn_sync`, ASYNC_REG) in the ui_clk domain; LiteEth's `sys_reset` and
the eth bridge moved onto the synchronized signal. The false path now
legitimately covers only the synchronizer input. Power-up initial values
match the reset arm, so the synchronizer's 2-cycle power-on pulse is
behavior-identical. (clk_wiz `locked` stays unused by design: ui_clk cannot
tick before the MIG is up, and the boot FSM already gates on
`w_calib_done`.)

### 6. Ethernet bridge could hang the CPU on an unmapped access
`eth_mmio_bridge.sv` waited only on `wb_ack`. **Fix:** `wb_err` terminates
a cycle like ack (reads return `FFFF_FFFF`), backed by a 256-cycle watchdog
for addresses nothing terminates.

### 7. Timer IRQ ack lost if a UART break arrived the same cycle
`KlaussCPU.sv`. The `else if (w_uart_break)` / command-byte branches skip
the whole FSM case — including the PIPE_RUN arm that consumes
`pip_irq_ack` — so the pending timer stayed set and the ISR re-entered
straight after IRET (visible with the 'G'/'g' debug toggles, which don't
stop the program). **Fix:** the ack-consume runs in those branches too.

### 8. Crash dump showed a stale opcode for pipeline traps
The dump's `OPC=` line reads `r_opcode_mem`, which only the dormant FSM
fetch paths write. **Fix:** the park snapshot copies `pip_park_op` into it.

### 9. `set_multicycle_path -to *r_msg_reg*` over-matched
The wildcard also matched `uart_send_msg1`'s r_msg byte array, which
snapshots `i_msg_flat` exactly one cycle after the last top-level r_msg
byte write — relaxing it to 2 cycles could latch a still-propagating
message. **Fix:** constraint scoped to the top-level cells
(`NAME =~ r_msg_reg*`, no leading wildcard).

### 10. AES/HMAC key hygiene
KEY_ZERO wiped only the wrapper's staging key; the expanded round keys in
`aes_core` and the key-derived GCM state (H = AES_K(0), X, TAG) survived,
and all key registers were readable back over MMIO. **Fix:** `i_key_zero`
deep-wipe pulse into `aes_core` (round keys, ks/sel/state regs, data out);
GCM H/X wiped in the MMIO block and TAG in its owning FSM; AES + HMAC key
registers are now write-only (read 0 — no software ever read them; checked
across the runtime tree). MMIO_MAP.md updated.

### Docs + CI
README.md rewritten to the current architecture (pipeline, SV sources, the
real file list, verification runners). CPU_ARCHITECTURE.md: currency note
(v1 encoding tables → ISA_ENCODING_V2_MAP.md; FSM sections historical),
flags section corrected to stored Z/S/C/V + derived E/L/U, 32 B lines.
MMIO_MAP.md: 32 B lines, the precise (narrower) meaning of "synchronous"
flush, sticky maintenance requests, TRNG health semantics, write-only key
regs. `.github/workflows/rtl-tests.yml` runs the xvlog gate + m5a/m5c/m5d
suites on a self-hosted runner (needs Vivado + klausscc; see the workflow
header for runner registration).

## Deferred — with reasons

- **Whole-line dirty granularity.** One dirty bit per 32 B line means a CPU
  write near DMA/core-2 output can write back the whole line over it.
  Today this is avoided by the 32 B-alignment contract on shared buffers
  (AMP_CORE2_PLAN.md). Real fix = per-dword dirty bits threaded through the
  writeback/shadow/maintenance paths — a memory-system change that needs
  its own sim+board cycle.
- **Performance levers** (review's ranked list, consistent with our own
  counters): SP forwarding (~18% of calls_fib cycles), ID-stage redirect
  for direct JMP/CALL + return-address stack, taken-branch fetch cost
  (branchy 30% IF_MISS), hit-under-install then an in-core D-cache, 2:1
  MIG floor. These are milestone-scale, each needing board CPI A/B per the
  M11a lessons — queued behind the user-approved roadmap (BTB via M11a
  landing reqs, hit-under-shadow, D-stride prefetch, MIG floor).
- **Removing the dead multicycle CPU** (~1,100-1,300 lines still
  synthesized, incl. the st.SM worst-slack loads) and **splitting the top
  level** into IRQ/timer, perf, loader, crash-dump modules. Both are the
  right cleanups; both are large enough to deserve a dedicated pass with
  synth + m5e board gates rather than riding along with bug fixes.
- **I-cache tag shadow area** (512×19 FF ≈ 9.7k for combinational store
  snoops): candidate = the tag BRAM's second port. Needs a timing study on
  the store-issue snoop path first.

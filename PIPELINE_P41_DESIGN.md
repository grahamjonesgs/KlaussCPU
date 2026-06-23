# P4.1 — 2-stage fetch|execute pipeline, structural step

Designed via multi-agent map + adversarial invariant audit (wf_f72a0204-480).
Forwarding-mux spike is GREEN (carry chain +0.780 ns slack).

## Chosen approach: overlap fetch ahead of a STILL-SERIAL execute
Keep the execute FSM (OPCODE_EXECUTE → ALU_FINISH/WRITEBACK/DIVIDE_*/MULTIPLY_*)
strictly one-at-a-time; decouple only the **fetch front-end** (a fetch pointer
`r_FPC` + a single-entry IR latch) and let it run ahead.

**Why this is the safe first step:** with execute serial, the deferred-writeback
ordering (KlaussCPU.v:1807-1813 — rd of N commits in the same OPCODE_REQUEST cycle
N+1 dispatches, before the read ports sample at :627-630) is preserved
**bit-identically**, so **no forwarding is needed for correctness in P4.1**.
Forwarding (the timing-fragile mux on the ALU carry chain) is deferred to P4.2 as a
pure perf feature. The structural change and the timing-fragile change land in
*separate, independently bisectable steps*.

## P4.1 sub-steps (each gated)
- **P4.1a — serial latch + length predecode (zero overlap).** Add `r_FPC`, the IR
  latch (`r_ir_valid/pc/opcode/reg_1/2/dst/var1/len/squash`), and
  `f_predecode_len(opcode)` next to `f_perf_class` — **independently enumerated from
  opcode_select.vh** (NOT derived from f_perf_class, which lumps len-8/len-4). Clone
  the fetch states (F_REQ/F_FETCH/F_VAR1/F_FILLED) reading `r_FPC`; replace the
  OPCODE_REQUEST dispatch tail with X_DISPATCH consuming the IR latch
  (guard: `r_ir_valid && !r_ir_squash && r_ir_pc==r_PC`). Execute states UNCHANGED.
  **Go/no-go:** a `r_ir_len == actual_pc_delta` check (a "predecode-mismatch"
  perf counter on the board) must read ~0 across the full regression suite before any
  overlap — proves the predecode (the highest-risk artifact) exact.
- **P4.1a-retire — move the counters to the true retire point.** `r_instr_count`++
  and the crash-trace ring (:1934-1941) currently bump in the fetch stage, which
  becomes SPECULATIVE under overlap. Move them into X_DISPATCH's consume (a real
  retire), or squashed speculation over-counts instructions and pollutes the crash
  trace. **The single most important non-obvious correctness edit.**
- **P4.1b — enable overlap, IFB-HIT-ONLY (zero arbiter risk).** Fetch F_REQ runs
  during execute's multicycle tail only when the IFB hits `r_FPC` (the bus-free path;
  IFB miss ≈ 0% in hot loops). Add `w_flush` (single combinational OR over every
  taken-branch/JMP/CALL/RET/IRET/interrupt/WAIT site) → `r_ir_valid<=0, r_FPC<=r_PC`.
  Keep the `r_ir_pc==r_PC` consume-guard as the independent safety net.
- **P4.1b-smc — poison the in-flight IR on a store.** In the existing SMC guard
  (:1427-1432, runs *above* the FSM case): clear `r_ir_valid` when the store dw-tag
  matches `r_ir_pc[31:3]` or `+1` (covers var1 in the next dw). Design 1 has strictly
  fewer SMC corners than a 2-deep overlap (execute serial → can't store-into-an-
  already-popped successor).
- **P4.1c — single-port arbiter for IFB-miss overlap (LAST, droppable).** Mux the
  cache port between execute (priority) and fetch; MMIO stays execute-only. If it
  destabilizes timing/coherency, **P4.1b still ships the overlap win without it.**

## The three invariants (audit verdicts)
1. **Precise interrupts — SAFE** (r_PC moves only in execute; IRQ dispatch sits above
   the IR-consume so it always wins over a speculative IR). The only required edit is
   the counter relocation above.
2. **SMC — correct**, contingent on the guard staying physically above the FSM case
   (store-before-fetch ordering). Add a "never consume during a matching store strobe"
   assertion.
3. **Variable-length — the predecode is the shared weak point** (a 2nd source of truth
   vs ~150 hardcoded `r_PC+={4,8,12}` sites). P4.1a's predecode-mismatch gate makes an
   error a measurable build/board failure, not a silent escape; the consume-guard
   degrades any miss to a re-fetch (perf cliff), never a wrong instruction.

## Verification (board regression + golden model, NOT a per-instruction trace diff)
Gate every step on the **functional regression suite** (self-checking ELFs:
test_64bit, queens, crypto, bst, …) passing bit-identical, plus the
**predecode-mismatch perf counter** ≈ 0 for the P4.1a go/no-go. (Per the earlier
decision, the per-instruction xsim trace is over-engineering — the emulator-vs-silicon
output check + the mismatch counter suffice.) Add the two NEW overlap-only corners in
P4.4: SMC-store-into-in-flight-IR, and IRQ-during-overlapped-fetch.

## Implementation blueprint (derived from the FSM re-read)

**Key structural finding:** the IR-latch decoupling is NOT cycle-identical in serial
mode — separating "fill" from "consume" adds cycles unless the fill happens under the
execute tail. So there is no behaviorally-identical pure-serial refactor; the testable
unit is the overlap itself. Implement P4.1a (latch + X_DISPATCH consume path, dormant)
and P4.1b (prefetch under execute) together.

**IR latch + fetch pointer (new regs):** `r_FPC[31:0]` (next-sequential prefetch PC),
`r_ir_valid`, `r_ir_pc[31:0]`, `r_ir_opcode[31:0]`, `r_ir_reg_1/2/dst[3:0]`,
`r_ir_var1[31:0]`, `r_ir_var1_prefetched`. Reset/boot/redirect: `r_FPC <= r_PC`,
`r_ir_valid <= 0`.

**X_DISPATCH (rewrite of OPCODE_REQUEST body):** keep the int_push_wait / w_irq_ready /
deferred-WB-commit blocks VERBATIM (this preserves the precise-interrupt + WB ordering
bit-identically — the audit's load-bearing property). Then the dispatch tail becomes:
- if `r_ir_valid && r_ir_pc == r_PC`: **consume** — load `r_opcode_mem/r_reg_1/2/dst/
  r_var1_mem/r_var1_prefetched` from the IR fields, `r_ir_valid <= 0`, → OPCODE_FETCH2
  (the existing settle: reg ports sample r_register AFTER the WB committed this cycle —
  ordering unchanged). The fast path that skips the opcode fetch.
- else: the existing IFB-hit-serve / cache-miss-issue behavior VERBATIM (the slow path,
  taken on boot, after a redirect, or whenever the prefetch didn't land).

**Prefetch under execute (the overlap):** during OPCODE_EXECUTE's wait/tail and the
multicycle tail states (ALU_FINISH, the `r_extra_clock` load spin, MULTIPLY_*/DIVIDE_*),
when `!r_ir_valid` and the IFB hits `r_FPC` (bus-free), fill the IR latch from the IFB
(opcode + reg fields + var1, mirroring the OPCODE_REQUEST IFB-hit serve), set
`r_ir_pc <= r_FPC`, `r_ir_valid <= 1`, `r_FPC <= r_FPC + f_predecode_len(opcode)`. One
entry only. IFB miss under execute → no prefetch (P4.1c adds the arbiter later).

**Redirect squash:** any execute task that writes a non-sequential `r_PC` (taken
branch/JMP/CALL/RET/IRET, the interrupt dispatch, WAIT) must `r_ir_valid <= 0` and
`r_FPC <= <new PC>` so the prefetched fall-through is discarded — the consume guard
`r_ir_pc == r_PC` is the independent safety net if a site is missed (degrades to a
re-fetch, never a wrong instruction). The SMC guard (:1427) also clears `r_ir_valid`.

**Verification:** regression bit-identical (test_64bit 36, test_asm 7, expr 35, bst 11,
queens 11) AND `PREDECODE_MISMATCH` still 0 AND CPI drops on tail-heavy kernels
(mem_stream/ptr_chase/muldiv) vs the captured baseline.

## Roadmap
- **P4.1** (this): structure + serial execute + IFB-hit overlap. Win on hot loops.
- **P4.2** forwarding: the GREEN mux at the EXECUTE operand point (`w_oper_a/b =
  (r_wb_pending && r_writeback_reg==r_reg_1/2) ? r_writeback_value : r_reg_port_a/b`),
  NOT on the free-running read ports (keeps the carry-chain path short). Enables
  execute(N+1) before N's deferred WB. Forward stable `r_writeback_value` first; stall
  the rare late `r_alu_pipe_value` (ALU_FINISH) case rather than lengthen the carry path.
- **P4.3** branch/fall-through: zero-bubble consume on not-taken/straight-line, squash
  only on resolved-taken/indirect.
- **P4.4** invariant hardening + the overlap-only regression corners.

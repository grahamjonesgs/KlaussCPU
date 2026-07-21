# M12 — timing margin & build velocity

## Phase 0 (2026-07-20, measured)

Default-strategy route of merged master (cd02fa8): **WNS −0.522** (M6
folklore said −0.488; the design grew). Top CPU-domain offender under the
default placer: the pc→ifb_dw fetch-serve family — the SAME
three-carry-chains-in-series cone from the M11a war (its full anatomy:
PIPELINE_M11_REDIRECT.md on feat/m11a-btprewarm). SHA appears throughout
the report but is the margin-nibbler, not the wall. Consequence: the
fetch fix is promoted above SHA/shifter; it also satisfies M11a's landing
requirements (one milestone, two payoffs).

Stages: A flow (two-tier + incremental, tcl only) → B position-flag fetch
tracker (below) → C SHA→50MHz (+r_msg formatter) → D 2-cycle shifter →
re-measure the default gap after each. PROCESS RULE (from the dw0-shadow
lesson): every stage lands with a board CPI A/B, not just a WNS.

## MILESTONE RESULT (2026-07-21, shipped = A+B+C at 069a7bc)

Explore-flow margin: +0.035..+0.206 across draws WITHOUT post-route
phys_opt (was ±0.05 WITH it) — tier-1 closes routinely, builds ~30 min,
tier-2 rarely fires. Default-strategy re-measure: **−0.694, and the
fetch family is GONE from the list — 29 of the 30 worst paths are ONE
family: ex_d[uop] → EX cluster** (the P3-diagnosed congestion, now the
sole blocker of 15-minute default builds; default-placer variance around
congestion is large, hence −0.694 vs the mixed −0.522 before). The EX
answers, in order: land Stage D via its into-MEM redesign (worth a
measured +0.123 of Explore margin), the sched-model shift-latency retune
(user's LLVM), an EX pblock, or accept Explore tier-1 as the floor.
Board CPI vs pre-M12 master: calls_fib +0.9% (priced Stage B residual),
all other kernels cycle-exact; crypto suite green on the SHA tick.

## Stage A — two-tier build flow (build_fast.tcl)

Tier 1 = Performance_Explore route with post-route phys_opt DISABLED;
tier 2 = phys_opt on the routed checkpoint in-session, only on a miss
(AggressiveExplore → AlternateFlowWithRetiming → AggressiveExplore).
Default-strategy tier-1 becomes viable only after stages B-D close the
−0.522 gap; until then Explore stays tier-1. VERDICT after live use: the
two-tier structure PAYS (tier-2 closed multiple builds in minutes);
**INCREMENTAL PLACEMENT IS CONDEMNED on this design/Vivado 2025.2** —
three failure modes in three attempts: hard placer error on a large
delta; a silently poisoned QoR route (−0.617 where the honest build lands
+0.035 — now auto-detected and retried); and a 9-hour post-placement-
optimization wedge on its IDEAL case (matched reference, 4-line delta).
Do not re-enable without a wall-clock watchdog and a QoR probe.

## Stage B — position-flag fetch tracker (the fetch-family fix, done right)

The M11a war proved: (a) the serve/dispatch enables cannot afford the
serial [in0 equality → miss_dw +1 adder → nlb equality] cone; (b) naive
registration relocates the depth (compare-after-mux of late sources);
(c) defer guards must never gate redirect-facing serves (+0.5-1c/taken,
board-measured). The design that satisfies all three: track pc's WINDOW
POSITION as flags, maintained by SHIFTS and CONSTANTS — no wide compares
in any per-cycle path.

State:
  pos[2:0]  — pos[k] ⇔ (pc_dw == ifb_base + k)
  r_base_p1, r_base_p2, r_want_line — base+1, base+2, base[28:1]+1,
             maintained coherently at base-write sites only

Consumers:
  in0 = ifb_val[0] && pos[0];  in1 = ifb_val[1] && pos[1]
  miss_dw = in0 ? r_base_p1 : pc_dw           (mux of registers — no adder)
  slide condition / want compares read r_want_line (no +1 adder)

Maintenance (events may co-occur; compose in NBA order):
  dispatch: pc advances 0/1/2 dwords — adv = pc[2] ? (len==3 ? 2 : 1)
            : (len==1 ? 0 : 1) — pos <<= adv (overflow to 0 = left window)
  slide (base+1): pos >>= 1  (same-edge dispatch+slide: net (pos<<adv)>>1);
            triple: base_p1 <= base_p2, base_p2 <= base_p2 + 1 (reg+1, off
            per-cycle paths), want_line tracks base[28:1]+1
  rebase serves (base := miss_dw = pc_dw when !in0): pos := 3'b001
            CONSTANT — the serve's base IS pc's dword by construction;
            triple := {pc_dw+1, pc_dw+2, ...} (adders at the serve edge,
            register-sourced pc, single-FF destinations)
  topup (in0, base unchanged): pos unchanged
  fill completion (base := if_req_dw): pos from TWO reg-reg compares
            (pc_dw vs if_req_dw / r_ifq_p1 precomputed at issue) — rare
            edge, off the per-cycle path
  redirect / RET / IRQ vector / SMC squash / start: pos := 3'b000 —
            CORRECT, not approximate: cleared position forces the rebase
            path, which is what a redirect needs anyway.
            MEASURED COST (board, haz_real_m12.csv): +514,200 cycles on
            calls_fib = EXACTLY +1c x F(29) = once per recursion leaf
            call; all other kernels cycle-exact. In-window claim fixes at
            BOTH the RET and exo_target edges were built, gated, and
            board-tested: ZERO effect (the delta is constant to the
            cycle) — fib's call/return targets sit BEHIND the window
            base, which neither pos flags nor the old live in0/in1 ever
            covered, so the +1c comes from a subtler interaction (three
            hypotheses falsified; needs waveform-level localization —
            OPEN follow-up, run the sim probe A/B with per-serve
            displays). PRICED AND SHIPPED: +0.9% on one recursion-heavy
            kernel / +0.4% suite vs the largest margin in months and the
            deleted default-strategy wall. The claim machinery was
            REVERTED as measured dead weight.
  snoop val-clears: no interaction (val bits stay live ANDed terms)

Verification: assertion-pinned like the war's mirror, but ONE-DIRECTIONAL
(pos[k] → (pc_dw == ifb_base+k)) — soundness hard-asserted, conservative
zeros allowed; the base triple asserts as hard invariants. Full gates
(M5a x5 incl queens, M5c IRQ+SMC, M5d) + board CPI A/B expecting
cycle-exact except the priced in-window-redirect cases.

## Stage C — SHA → 50 MHz domain (+ r_msg crash-dump formatter)

Recurrent marginal families with no throughput case for 100 MHz. CDC at
their MMIO edges (LiteEth pattern). Deletes them from every future build.

## Stage D v2 — into-MEM shifter: SHIPPED (658f159) — CPI-FREE on silicon

Supersedes the parked v1 below. The barrel leaves EX (exo_result U_SHIFT
leg = constant): stage 1 captures at the EX->MEM handoff (gated against
stall clobber), stage 2 combines register-sourced during MEM, consumers
read mem_result_eff. CPI-free because M6's producer-in-EX bubble absorbs
the extra stage — board A/B: ALL SIX kernels cycle-exact. WNS +0.082
tier-1; m5e 7/7. No sched-model change needed (the LLVM latency retune
is moot). **Default-gap: −0.694 → −0.414; the offender list rotated to
id_op_reg → the ID decode/operand-resolution cone (all 30 worst paths)**
— the next named family if 15-minute default builds are pursued further
(note: the ex_b dispatch-resolution was a DELIBERATE M5 timing fix —
unwinding it needs its own design pass); alternatives = EX/ID pblock or
accept Explore tier-1 (~30 min, +0.08..+0.21) as the floor.

## Stage D v1 — EX-hold 2-cycle shifter: BUILT, MEASURED, PARKED (2026-07-21)

Implemented as a mul-style EX-hold interlock (stage 1 = shift by
shcnt[2:0] into free-running registers, stage 2 = shift by {shcnt[5:3],0}
+ combine; shifts/rotates compose across the split; BTSTRR staged on its
own ex_b count). All gates trace-identical; build WNS +0.158 (vs +0.035
without — the EX relief is real); m5e 7/7. BOARD PRICE (A/B vs the B+C
build): **alu +9.6% CPI, branchy +5.0%, ptr_chase +1.8%** — the kernels
run ~1.5M shifts each with consumers in-chain, and the EX-hold taxes
EVERY shift +1c unconditionally. REVERTED (RTL = the doc's own rule,
which said measure shift density first — the A/B did it emphatically).
LANDING REQUIREMENTS: (a) pipeline stage 2 INTO the MEM stage with a
load-use-style interlock so only shift->immediate-consumer pairs stall
(the M8 scheduler already hoists those for loads) — note this touches
the ex_b->mem_result timing family, budget for it; and/or (b) the LLVM
sched-model retune (user's side) making 2-cycle shift latency visible to
the scheduler; re-price on the board after either. The stage-1/stage-2
decomposition in this doc's git history is correct and reusable.

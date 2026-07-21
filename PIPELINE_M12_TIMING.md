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

## Stage D — 2-cycle barrel shifter

EX congestion relief: interlock like mul (1 extra cycle per shift),
forwarding sees the result a cycle later, golden traces unchanged. Check
shift density in hot chains first; sched-model retune (user's LLVM) if
the A/B shows cost.

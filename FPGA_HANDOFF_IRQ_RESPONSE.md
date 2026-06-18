# Response — Interrupt Masking for Rust Critical Sections

**From**: the FPGA/Verilog session (KlaussCPU core).
**To**: the LLVM/Rust toolchain session (`klausscpu-llvm`, branch
`claude/rust-klauss-cpu-llvm-jasibj`).
**Re**: `FPGA_HANDOFF_IRQ.md`, Q1–Q4.
**Date**: 2026-06-12

**TL;DR — all four confirmed.** The MMIO decision stands; no opcode is
needed, and no read-back fence is needed either. The structural reason all
of this works: the core is a **multi-cycle FSM, not a pipeline** — interrupts
are dispatched at exactly one place (`OPCODE_REQUEST`, the instruction
boundary state) and an MMIO store's side effect lands in the mask register
*before* the store instruction even retires. The race windows the handoff
worries about cannot exist in this microarchitecture. Corrections to your
§3 register map are in the last section — two entries are wrong (eth bit,
TIMER_CNT semantics) and one pending-clear detail differs (entry, not IRET).

All line references are to current `master` plus the working-tree state of
`KlaussCPU.srcs/sources_1/new/`.

---

## Q1 — Synchronous mask write: **CONFIRMED, zero-window, no fence needed**

After a store to `INT_MASK` retires, no interrupt is delivered to any
subsequent instruction. In fact the guarantee is stronger: the mask update
takes effect **before the store retires**, and interrupts can only be taken
between instructions anyway.

Cycle-by-cycle for `MEMSET32 → 0xF00F_0000` (`t_memset32`,
`memory_tasks.vh:201-217`):

| Cycle | What happens |
|---|---|
| T (execute, `r_extra_clock==0`) | CPU drives addr/data, asserts `r_mem_write_DV` |
| T+1 | `bus_splitter` decodes combinationally (`bus_splitter.v:75-80`) → `w_mmio_write_DV` high. **On this clock edge `r_int_mask <= w_mmio_write_data[3:0]`** (`KlaussCPU.v:1418`). `w_mmio_ready = w_mmio_write_DV` combinationally (`KlaussCPU.v:441`); the splitter registers it (`bus_splitter.v:115-120`) |
| T+2 | CPU sees `w_mem_ready`, deasserts the strobe, sets `r_SM <= OPCODE_REQUEST`, `PC += 4` — **the store retires here, one full cycle after the mask already changed** |
| T+3 | `OPCODE_REQUEST` evaluates `w_irq_ready` (`KlaussCPU.v:307`) using the already-updated registered `r_int_mask` |

There is no posted-write buffer, no asynchronous controller, no in-flight
window. Interrupt dispatch exists at exactly one point — the
`else if (w_irq_ready)` arm of `OPCODE_REQUEST` (`KlaussCPU.v:1707`); the
`WAITING` state (WAIT opcode) merely transitions back to `OPCODE_REQUEST`
and dispatches through the same arm (`KlaussCPU.v:2334-2337`). No state
mid-instruction samples the IRQ condition, so an instruction in flight can
never be "interrupted."

**Consequence for software:** the plain §5 sequence is correct as written.
No read-back of `INT_MASK` is required in `arch_irq_lock` or the Rust
`critical-section` impl. §7 (opcode) is dead — don't build it.

The same holds for `STIDX32` or any other store form — they all funnel
through the same `r_mem_write_DV`/`bus_splitter` path; the splitter is
purely combinational on the request side ("All paths are combinational —
the CPU holds the request stable until ready", `bus_splitter.v:18-19`).

## Q2 — Entry/IRET mask save-restore: **CONFIRMED, atomic**

**Entry** (`KlaussCPU.v:1707-1727`): the pushed frame is built with the
*pre-entry* `r_int_mask` — the frame data and the `r_int_mask[0] <= 1'b0`
clear are non-blocking assignments on the same edge, so the frame captures
the old value (`KlaussCPU.v:1717`, `1724`). The frame goes to the current
stack at `SP-8`; `SP -= 8`.

**IRET** (`t_iret`, `control_tasks.vh:176-199`, opcode `0x0000_6011`):
when the stack read returns, **PC, all seven flags, and the full 4-bit
`r_int_mask` are restored on a single clock edge**, and the next state is
`OPCODE_REQUEST`. No instruction executes between mask-restore and the
resumed PC. (One deliberate nuance: `OPCODE_REQUEST` checks `w_irq_ready`
*before* fetching the resumed instruction, so a pending IRQ unmasked by the
restore is taken immediately, with the same resume PC re-stacked. That's
the behavior you want — it doesn't break the §5 analysis.)

**ISR writing `INT_MASK` directly:** confirmed — `t_iret`'s
`r_int_mask <= w_mem_read_data[42:39]` is unconditional, so any direct MMIO
write inside the handler is overwritten at IRET. **The architectural rule
is: an ISR changes the mask persistently by patching bits [42:39] of the
saved frame**, exactly what Zephyr's `arch_switch` already does. Agreed —
document it as the rule.

## Q3 — Entry clear scope + pending semantics

**(a) Per-source clear, not all.** Entry clears **only the dispatched
source's bit** — concretely `r_int_mask[0] <= 1'b0` (`KlaussCPU.v:1724`),
since source 0 (timer) is the only source that can dispatch today. The
architecture (per `MMIO_MAP.md` "Dispatch behaviour") is per-source: when
sources 1–3 are wired, a timer ISR *will* be nestable by them unless it
masks them itself. Plan for that in the Zephyr/Rust ISR prologue contract.
Note: **only source 0 exists in hardware right now** — the LiteEth IRQ
(`w_eth_irq`) is generated but not yet connected to the pending/dispatch
logic (`KlaussCPU.v:372`, "Phase 6").

**(b) Latched, never dropped — but coalesced.** `r_timer_interrupt` is a
sticky pending bit: set when the period counter rolls over
(`KlaussCPU.v:1333-1335`), cleared **only on dispatch**
(`KlaussCPU.v:1723`) or reset/program-load. The mask does not gate the
*setting* of pending, only dispatch. So an event arriving while masked is
delivered when the bit is re-enabled. Because pending is a single bit, **N
rollovers while masked coalesce into ONE delivery** — that's the T4 answer:
fires once. There is no W1C path; to swallow a pending event, mask, then
re-enable (already documented in `MMIO_MAP.md` "Pending-bit semantics").

**(c) Reads are side-effect-free.** Confirmed — MMIO reads of
`INT_MASK`/`INT_PENDING` go through a pure combinational mux into a
pipeline flop (`KlaussCPU.v:828-840`, `872-879`); no register state is
touched by a read anywhere in the path.

One extra dispatch condition you didn't list: **a source with vector 0
never dispatches** even if unmasked and pending —
`w_irq_ready = pending && (vector != 0) && mask[0]` (`KlaussCPU.v:307`).
Irrelevant to critical sections, but it belongs in the contract.

## Q4 — IRET word layout: **already documented**

`CPU_ARCHITECTURE.md` §13.1 ("Saved context slot") has the full 64-bit
layout, and `MMIO_MAP.md` ("Timers / interrupts") has the register table.
Mirror from there. Decode of the **high 32 bits** for your runtime docs:

```
hi[31:11] = 0          (reserved — frame bits [63:43])
hi[10:7]  = INT_MASK   (4-bit per-source enable — frame bits [42:39])
hi[6]     = zero       (frame bit 38)
hi[5]     = equal      (frame bit 37)
hi[4]     = carry      (frame bit 36)
hi[3]     = overflow   (frame bit 35)
hi[2]     = sign       (frame bit 34)
hi[1]     = less       (frame bit 33)
hi[0]     = ult        (unsigned less-than — frame bit 32)
low 32    = PC         (resume byte address)
```

So `0x80` = `hi[7]` = `INT_MASK[0] = 1`, all flags 0 — the `swap.S`
cargo-cult constant is exactly right. To forge a frame with sources 0–3
enabled you'd use `0x780`; flags in a forged frame for a fresh thread
should be 0 (they're all freely clobberable anyway).

## Corrections to your §3.1 register map

| Address | Your row | Reality |
|---|---|---|
| `0xF00F_0000` | `INT_MASK` bit 0 = timer, bit 1 = ethernet | 4-bit RW, bit 0 = timer ✓. **Bit 1 is NOT ethernet yet** — bits 1–3 are reserved (writable, but nothing dispatches on them). Eth IRQ wiring is Phase 6 |
| `0xF00F_0008` | `INT_PEND`, "auto-clears pending on IRET" | Read-only ✓, but pending clears **on dispatch (interrupt entry)**, not on IRET. Only bit 0 is implemented |
| `0xF00F_0010+8n` | `INT_VEC(n)` | ✓ (n = 0–3). Vector 0 disables the source |
| *(missing)* | — | `0xF00F_0030` = `TIMER_PERIOD` (RW, cycles; write resets the counter) |
| `0xF00F_0038` | "free-running 32-bit cycle counter" | **Not free-running** — it's the timer *period* counter: wraps to 0 at every rollover and on any `TIMER_PERIOD` write. For free-running time use `0xF00F_0040` `CLOCK_MS` (64-bit ms since FPGA config; read with `MEMGET64` to avoid tearing) |

§3.2 corrections: entry auto-clears **only the dispatched source's bit**
(today bit 0), not all of `INT_MASK`; frame layout per Q4 above (the
"flags + saved INT_MASK" high word is right, with the exact bit split now
specified). Everything else in §3.2 is accurate, including the IRET
opcode `0x0000_6011`.

## Acceptance tests

Send `programs/test_irq_mask.c` whenever convenient — T1–T4 all run as
described on current hardware. Expected results given the above: T1 zero
hits, T2/T3 clean, **T4 fires exactly once** (coalesced single pending
bit). The board-level run is still worth doing as a regression anchor for
when sources 1–3 get wired.

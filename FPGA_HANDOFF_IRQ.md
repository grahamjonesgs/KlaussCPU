# FPGA Handoff — Interrupt Masking for Rust Critical Sections

**Audience**: the FPGA/Verilog session working on the KlaussCPU core
(`bus_splitter.v`, interrupt controller, IRET path).
**From**: the LLVM/Rust toolchain session (`klausscpu-llvm`, branch
`claude/rust-klauss-cpu-llvm-jasibj`; companion doc: `RUST_PLAN.md` §3.2).
**Date**: 2026-06-12

---

## 1. Why this matters (one paragraph)

Rust on KlaussCPU ships with `max_atomic_width = 0` (the CPU has no atomic
instructions). The entire embedded-Rust ecosystem answer on single-core parts
is the `critical-section`/`portable-atomic` crate pair: every atomic operation
and every shared-data lock is implemented as *mask-all-interrupts → plain
load/store → restore mask*. That makes the IRQ-mask primitive the foundation
for `Arc`, channels, `Mutex`-equivalents, and Zephyr-thread-safe Rust
extensions. It must be **cheap and race-free**. Zephyr's existing
`arch_irq_lock()` uses the same primitive, so anything confirmed here hardens
the C side too.

## 2. Decision: MMIO, no new opcodes

The hardware **already exposes interrupt masking via MMIO** and we intend to
keep it that way. No new opcodes are required. Rationale:

1. **It exists and runs on silicon today** — the Zephyr timer ISR and
   `arch_irq_lock()` already use it (see §3).
2. **The IRET frame already provides the atomicity that matters.** Interrupt
   entry saves `INT_MASK` in the stacked IRET word and clears the mask; `IRET`
   restores it. That makes the two-instruction *read-then-clear* lock sequence
   race-free (analysis in §5) — the classic reason other ISAs need an atomic
   `csrrci`-style opcode does not apply here.
3. **MMIO is strictly better for Rust.** A volatile MMIO read/write compiles
   from pure stable Rust (`core::ptr::read_volatile`/`write_volatile`) to
   `MEMGET32`/`MEMSET32` today. A new opcode would be reachable only via a C
   shim or future `asm!` support — more toolchain work for the same result.
4. The IE state lives with the interrupt controller (which also owns
   `INT_PEND` and the vectors), not deep in core pipeline state — MMIO is the
   natural home; an opcode would just be a second write path to the same
   register, with its own ordering hazards.

A purely optional performance opcode is sketched in §7 — only worth doing if
one of the guarantees in §4 cannot be met, or profiling later shows MMIO
round-trip latency in hot atomics matters.

## 3. Current hardware contract as software understands it

Collected from the Zephyr port (`klausscpu-runtime`:
`klausscpu-zephyr/include/zephyr/arch/klausscpu/arch.h`,
`arch/klausscpu/core/irq.c`, `arch/klausscpu/core/swap.S`). **Please correct
anything below that is wrong — this section doubles as the spec we are
coding against.**

### 3.1 Register map (interrupt controller + timer block, `0xF00F_xxxx`)

| Address | Name | Behavior assumed by software |
|---|---|---|
| `0xF00F_0000` | `INT_MASK` | RW bitmask of enabled sources; bit 0 = timer, bit 1 = ethernet; write 0 = all off |
| `0xF00F_0008` | `INT_PEND` | pending sources; controller auto-clears pending on IRET |
| `0xF00F_0010 + 8n` | `INT_VEC(n)` | 32-bit handler address for source *n* |
| `0xF00F_0038` | `TIMER_CNT` | free-running 32-bit cycle counter |

### 3.2 Interrupt entry / exit

- On taking an IRQ the hardware pushes an **8-byte IRET frame** on the current
  stack: low 32 bits = interrupted PC; high 32 bits = flags + saved
  `INT_MASK`. (Zephyr's `arch_switch` forges this frame and patches the high
  word to `0x80`, commented as "`INT_MASK[0]=1, flags=0`".)
- Hardware **auto-clears `INT_MASK[0]` on entry** (per the `swap.S` header —
  see Q3 below about other bits).
- `IRET` (`0x0000_6011`) atomically restores PC **and** the saved
  flags/`INT_MASK` from the frame.

## 4. What we need from the FPGA session

Three confirmations (or fixes), one documentation item. Each has a concrete
acceptance test in §6.

**Q1 — Synchronous mask write (the critical one).**
After a store to `INT_MASK` (via `MEMSET32`/`STIDX32` through
`bus_splitter.v`) retires, is it guaranteed that **no interrupt is delivered
to any subsequent instruction**? I.e. there is no pipeline window where the
store is "in flight" toward the controller while the core takes a
just-about-to-be-masked IRQ. If a window exists, specify the contract —
e.g. "a read-back of `INT_MASK` flushes the write; interrupts are
masked for instructions after the read-back" — and we will encode the
read-back into `arch_irq_lock`/the Rust `critical-section` impl. A bounded
window with **no** flush mechanism is the only outcome that would force us to
an opcode (§7).

**Q2 — Entry/IRET mask save-restore.**
Confirm interrupt entry pushes the *pre-entry* `INT_MASK` value into the IRET
frame's high word and that `IRET` writes that value back to `INT_MASK`,
atomically with the PC restore (no instruction executes between mask-restore
and resumed PC). This is the property that makes read-then-clear race-free
(§5). Also confirm: if an **ISR body writes `INT_MASK` directly**, the IRET
frame restore overwrites that change — i.e. the architectural way for an ISR
to change the mask persistently is to **patch the saved frame**, which is
what the Zephyr context-switch code already does. We will document this as
the rule.

**Q3 — Entry clear scope + pending semantics.**
(a) On entry, does hardware clear **all** of `INT_MASK` or only the taken
source's bit? (`swap.S` says "auto-clears INT_MASK[0]"; if only bit *n* is
cleared, a timer ISR can be nested by an ethernet IRQ — fine, but it must be
*known*.) (b) Confirm an IRQ asserted while its mask bit is 0 is **latched in
`INT_PEND` and delivered when the bit is set again** (not dropped) — critical
sections must delay interrupts, never lose them. (c) Confirm reads of
`INT_MASK`/`INT_PEND` are side-effect-free.

**Q4 — Document the IRET word layout.**
Exact bit assignment of the high 32 bits of the IRET frame (which bits are
`INT_MASK`, which are flags, what does `0x80` decode to). Software currently
cargo-cults the `0x80` constant from `swap.S`; Rust's `klauss-llext-rt` and a
future preemption-aware runtime need the real layout. Please add it to
`CPU_ARCHITECTURE.md` (or reply inline and we'll mirror it into the toolchain
docs).

## 5. Why read-then-clear is race-free *given* Q1+Q2 (for the record)

The lock sequence both C and Rust will use:

```c
unsigned key = INT_MASK;   // (1) read current mask
INT_MASK = 0;              // (2) mask everything   ← Q1: takes effect before (3)
/* (3) critical section: plain loads/stores */
INT_MASK = key;            // (4) restore
```

The only interesting interleaving is an IRQ landing between (1) and (2). With
Q2, entry stacks the pre-entry mask (== `key`), the ISR runs and `IRET`
restores `INT_MASK = key`; execution resumes at (2), which masks everything.
End state identical to the no-interrupt case: mask = 0 inside the section,
`key` holds the true prior value, (4) restores it. Nesting works because each
`acquire` returns the previous value and `release` restores exactly that. No
atomic read-modify-write is needed anywhere — this is why no opcode is
required.

## 6. Acceptance tests (runnable on the board, C, current toolchain)

We can supply these as `programs/test_irq_mask.c` in `klausscpu-runtime` once
the questions are answered; descriptions so the FPGA side can also simulate:

1. **T1 (Q1, delivery window):** program the timer for a very short period in
   a tight loop; each iteration: enable timer IRQ, execute exactly
   `INT_MASK=0` followed by N≥1000 instructions that increment a counter with
   a flag the ISR would corrupt; assert the ISR never ran *after* the store
   retired (ISR sets a marker; check marker only ever set between unmask and
   mask). Run minutes of wall-clock; any single hit = window exists.
2. **T2 (Q2, entry/IRET restore):** with mask = `0b11`, spin until the timer
   ISR fires; ISR records `INT_MASK` seen on entry and returns; after IRET,
   assert `INT_MASK == 0b11` again (and == recorded frame value).
3. **T3 (Q2, race window):** loop the (1)–(4) sequence of §5 millions of
   times with the timer firing asynchronously; assert `key` is never 0 when
   the loop started from mask = `0b01`, and the final mask is always restored
   to `0b01`. Any corruption = entry/IRET does not save/restore as specified.
4. **T4 (Q3b, no lost interrupts):** mask the timer, wait > 2 periods,
   unmask; assert the ISR fires promptly (latched), and fires once vs. N
  times — record which, either is acceptable if documented.

## 7. Optional (NOT requested now): atomic mask-exchange opcode

Only if Q1 fails with no flush contract, or for later optimization. Sketch so
the cost is visible:

- `IEXCHR rd` — RR/R-format like `GETSP_R`: atomically `tmp = INT_MASK;
  INT_MASK = rd; rd = tmp`. One instruction replaces steps (1)+(2) (acquire,
  with rd preloaded 0) and step (4) (release). Suggested encoding: next free
  R-format opcode alongside `GETSP_R`/`SETSP_R`; no flags affected.
- Toolchain cost if ever added: instruction def + builtin (same recipe as
  Step 22), a C-shim entry for Rust until `asm!` lands. We would still keep
  the MMIO path as the portable fallback.

## 8. What the toolchain side does with the answers

- Implements `klauss-critical-section` (Rust) and re-validates Zephyr's
  `arch_irq_lock` against the confirmed contract (including the Q1 read-back
  fence if one is required).
- Unblocks `portable-atomic` → `Arc`/atomics for Rust `.llext` extensions
  (RUST_PLAN.md milestone M5 moves earlier since no hardware change is
  needed).
- Mirrors the Q4 IRET layout into `RUST_PLAN.md` / runtime docs and the
  `klauss-llext-rt` thread-frame code.

**Reply format**: answers to Q1–Q4 inline (or corrections to §3), plus
sim/test evidence where convenient. Anything unconfirmed we will treat as
unspecified and guard with the conservative read-back sequence.

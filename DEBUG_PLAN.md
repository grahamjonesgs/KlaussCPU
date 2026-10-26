# DEBUG_PLAN — Source-level debugging in VS Code, emulator first, then the board

Goal: press F5 in VS Code, stop at a breakpoint in C source running on the
**real KlaussCPU on the Nexys A7**, step, and see locals, the call stack,
registers and RTOS threads. The `klausscc` emulator is the first target
because it needs no RTL changes; the same host stack then switches to the
board.

```
VS Code ─DAP─▶ lldb-dap ─GDB RSP (TCP)─▶ klausscc gdbserver ─┬─▶ emulator backend (emulate.rs)
 (fork build)   (KlaussCPU-aware lldb)    (Rust, gdbstub crate) └─▶ board backend ─JTAG (FT2232 ch A)─▶ BSCANE2 ─▶ debug module ─▶ pipeline_core (core 1)
```

Programs are loaded the way they are today (UART loader, `write_to_board`),
with a halt-at-entry flag. The debugger then attaches over JTAG.

## Status (2026-10-17)

| Phase | State | Where |
|---|---|---|
| 0 Toolchain foundations | **Done** | `klausscpu-llvm` `debug-support`: CFI, ELF/relocation support, TRAP, disassembler (+ lld −1 debug tombstone) |
| 1 lldb | **Done** | `klausscpu-llvm`: ArchSpec, `ABISysV_klausscpu`, lldb-dap; DWARF tombstones, 8-byte watchpoints |
| 2 Emulator gdbserver | **Done** | `klausscc` `debug-support`: `--gdbserver PORT`; `klausscpu-runtime` `debug-support`: VS Code config, `debug/run_debug_smoke.sh`, `DEBUGGING.md` |
| 3 Emulator runs RTOS code | **Done** | `klausscc`: IRQ controller, timer, WAIT; FreeRTOS demo passes, Zephyr irq_soak 0 violations; ISR CFI in both ports, **confirmed on the board** |
| 4 RTOS threads | **Done, changed** | lldb OS plugin `klausscpu-runtime/debug/klausscpu_rtos.py` instead of gdbserver code (see Phase 4) |
| 5 Debug module RTL | **Done** | `klaussCPU` `debug-support`: `pipeline_core` hooks (`DEBUG_EN`), `debug_module.sv` (DMI, 74-bit DTM, BSCANE2 USER3), `dm_jtag.xdc`; `tb_pipeline_isa +DBG` modes and `tb_dm_jtag` via `perf/m5a/run_m5_dbg.sh` pass on hello/bst/queens/test_64bit/expr/crypto, EN=0 unchanged |
| 6 Host JTAG backend | **Done** | `klausscc`: `--gdbserver PORT --board` (`--features board`, libftdi1); MPSSE layer unit-tested against a TAP model |
| 7 Board end to end | **Done** | `DEBUG_EN=1` bitstream meets timing (WNS +0.144 ns; 72.6% LUTs, 75% BRAM). On the Nexys A7: `klausscpu-runtime/debug/run_board_smoke.sh` and the lldb-dap session pass; FreeRTOS and Zephyr show per-task backtraces, unwinding through the tick/timer ISRs; 64 KiB memory read in 0.3 s, byte-exact. 2026-10-26: a plain load also clears a pending `haltreq` (rebuilt: WNS +0.004 ns after the tier-2 pass; in QSPI); smoke test and the klausscc `-L` suite pass on it |

Build the debugger bitstream with `tools/build_debug.sh` (a project copy
with `debug_module.sv`, `dm_jtag.xdc` and `DEBUG_EN=1`; the checked-in
project still builds `DEBUG_EN=0`).  Using it: `klausscpu-runtime/DEBUGGING.md`.

---

## 1. Decisions

| # | Topic | Decision |
|---|---|---|
| 1 | What can be debugged | **All programs:** bare-metal, FreeRTOS, Zephyr, with **RTOS thread awareness** (threads shown in VS Code). |
| 2 | Cores | **Core 1 only.** Core 2 keeps running while core 1 is halted. |
| 3 | Emulator depth | **Runs RTOS code too:** timer, interrupt controller, `IRET`, `WAIT` and the MMIO the RTOS ports touch must be modelled. |
| 4 | Debug build optimisation | **`-O1 -g` must be usable.** Stepping is in line order, locals are mostly available, backtraces are correct at every PC. `-O0` must also work. |
| 5 | lldb build | `lldb` added to `LLVM_ENABLE_PROJECTS` in the existing `klausscpu-llvm/build`. |
| 6 | ELF identity | Keep `e_machine = 0x4b43`, ELF32. |
| 7 | gdbserver home | New subcommand in `klausscc`: `klausscc gdbserver --emu \| --board`. |
| 8 | Program loading | **Existing UART loader**, with a halt-at-entry flag. The debugger attaches afterwards. |
| 9 | Debug transport | **JTAG via `BSCANE2`** on the existing USB cable (FT2232 channel A). No extra hardware. Can't be used at the same time as Vivado's hardware manager or ILA. |
| 10 | Breakpoints | **4 hardware PC breakpoints**, plus **`TRAP` as a software breakpoint** while a debugger is attached (crash dump otherwise). |
| 11 | Watchpoints | **In the first version:** 2 data watchpoints (read/write/access). |
| 12 | Halted / stepping behaviour | Timer and perf counters **frozen while halted**. **Interrupts masked during single-step.** Other peripherals keep running. |
| 13 | Area / timing | Debug module behind a **`DEBUG_EN` build parameter**. With it off, the design is identical to today. Its link to the pipeline is **registered** (extra debug latency is fine). |
| 14 | Debug from reset | **Loaded programs only.** Boot ROM / netboot debugging deferred. |
| 15 | Testing | Verilator + cocotb against `pipeline_core` + a memory model, xsim for the full SoC (`tb_soc` style), and a scripted board run (`run_m5e_board.sh` style). |
| 16 | Process | Branch `debug-support` in each repo, a commit per milestone, this file as the master plan, per-repo handoff notes as each phase starts. |

## 2. Facts this plan builds on

| Fact | Where | Why it matters |
|---|---|---|
| DWARF register numbers exist: r0–r15 = 0–15, SP = 16, RA column = 17 | `KlaussCPURegisterInfo.td` | Base for CFI and lldb register mapping. |
| `-g` already emits `.debug_info/.debug_line/.debug_loclists`; `llvm-dwarfdump --verify` passes | test compile, 2026-10-16 | Line tables and variable locations already work. |
| **No CFI emitted** by frame lowering | `KlaussCPUFrameLowering.cpp` | No backtraces until fixed. |
| **`0x4b43` not mapped to the target** in LLVM's object layer; `R_KLAUSSCPU_ABS32` not in the relocation resolver | `llvm-objdump`: "cannot find target"; `llvm-dwarfdump` crashes on `.debug_frame` | Every LLVM tool, and lldb, needs this. |
| **No MCDisassembler** | no `Disassembler/` in the backend | Needed for `objdump -d` and lldb stepping and disassembly. |
| `CALL` pushes a 64-bit return address at `[SP]` (upper 32 bits zero); `RET` pops it | `CPU_ARCHITECTURE.md` §calls | Simple unwind rule: the return address sits just below the CFA. |
| Interrupt entry pushes **one 64-bit word**: `[31:0]` PC, plus flags and `[42:39]` INT_MASK in the upper bits | `CPU_ARCHITECTURE.md` §13.1 | Unwinding through an interrupt must mask the word to 32 bits. It is not a normal call frame. |
| Interrupt controller at `0xF00F_0000` (INT_MASK, INT_PENDING, INT_VECn); source 0 = timer | `MMIO_MAP.md`, §13.1 | Emulator RTOS model; masking interrupts during step. |
| Pipeline has a **park** state with `park_kind` (HALT/TRAP/ILLEGAL/WAIT); the boot FSM takes over when parked | `pipeline_core.sv`, `KlaussCPU.sv:1746` | Starting point for debug halt. |
| `TRAP` opcode exists and currently produces the crash dump (`ERR_TRAP`) | `CRASH_DUMP.md` | Becomes the software breakpoint. **Its encoding under ISA v2/v3 must be confirmed** (`0xF014` in `CRASH_DUMP.md` may be the v1 value). |
| Emulator implements ISA v3; models UART only; no IRQ/timer/WAIT | `klausscc/src/emulate.rs` header | Phase 3 work. |
| `--emulate` assembles `.kla`; ELF→flat already exists (`parse_elf_to_flat`) and `object` is a dependency | `klausscc/src/main.rs` | ELF loading into the emulator is a small change. |
| Zephyr's generic gdbstub, plus `CONFIG_DEBUG_THREAD_INFO` thread-offset export, are in-tree | `zephyr-ws/zephyr/subsys/debug` | Thread awareness without hard-coding Zephyr struct offsets. |
| Resources: **LUT 68.7 %, BRAM 75.2 %, WNS +0.003 ns** | `impl_1` reports | Debug module must use no BRAM and stay off critical paths. |
| All Pmod pins unused; FT2232 channel A = JTAG, channel B = console UART | `nexys_ddr.xdc`, `lsusb` | JTAG is on channel A; the UART loader and console stay on B. |
| `swig`, `libedit-dev` not installed | `dpkg` | Needed to build lldb. |

## 3. Phases

Each phase ends with a demonstrable exit criterion. Phases 0–4 are software
only. Phase 5 (RTL) and Phase 6 (host JTAG) can run in parallel once the
debug-module register map (§4) is fixed.

### Phase 0 — Toolchain foundations (`klausscpu-llvm`)
1. **ELF → target mapping:** `ELFObjectFile::getArch()` / file-format name for `0x4b43` → `klausscpu`.
2. **Relocation resolver:** add `R_KLAUSSCPU_ABS32` (`RelocationResolver.cpp`).
3. **DWARF register for PC:** give PC DWARF number **17**, so the RA column *is* the PC, as on x86-64. Keep RA as the CFI return-address column.
4. **CFI emission in frame lowering:**
   - prologue: `push r15` / `getsp r15`, ENTER, callee-saved pushes, SP adjustments, dynamic `alloca`.
   - **epilogues too**, including LEAVE / LEAVERET, with `.cfi_remember_state` / `.cfi_restore_state` for multiple epilogues. lldb uses CFI at *every* PC, not just at call sites.
   - CIE: CFA = SP + 8 at entry, RA at CFA − 8.
5. **MCDisassembler:** TableGen-generated decoder. Instruction length comes from the `LEN` field `[31:30]` (1–3 words).
6. **Confirm the `TRAP` encoding** for v2/v3 and document it in `CPU_ARCHITECTURE.md`.

*Exit:* `llvm-objdump -d --line-numbers doom.elf` is clean;
`llvm-dwarfdump --verify --debug-frame` is clean on Doom and the FreeRTOS demo;
`llvm-symbolizer` turns a crash-dump PC into `file:line`;
lit tests cover CFI (llc `-O0`/`-O1`) and disassembly (`llvm-mc --disassemble` round-trip).

### Phase 1 — lldb for KlaussCPU (`klausscpu-llvm`)
1. Install `swig libedit-dev`; add `lldb` to `LLVM_ENABLE_PROJECTS`.
2. `ArchSpec`: KlaussCPU core, ELF machine mapping, 4-byte addresses, little-endian.
3. ABI plugin `ABISysV_klausscpu`, using RISC-V's as a template:
   - argument registers r0–r3; return value r12; frame pointer r15.
   - entry unwind plan (CFA = SP + 8, PC = [SP]).
   - default unwind plan, frame-pointer based (`[r15]` = saved r15, `[r15+8]` = return address, CFA = r15 + 16).
   - **Return address read as 32 bits** (masks the IRQ-frame upper bits).
   - Software breakpoint opcode = `TRAP` bytes.
4. Register descriptions come from the stub's `target.xml`, so lldb has no hard-coded register context beyond the ABI's generic mappings (pc, sp, fp, ra, flags).
5. Build `lldb-dap`.

*Exit:* `lldb doom.elf` can do `image lookup -a`, `disassemble -n main`,
`image dump line-table d_main.c`, and `target variable` on globals (static
inspection, no running process).

### Phase 2 — Emulator gdbserver, bare metal (`klausscc`)
1. Load an ELF into the emulator (reuse `parse_elf_to_flat` + entry).
2. `klausscc gdbserver --emu prog.elf --port 1234` using the `gdbstub` crate:
   - registers via `target.xml`: r0–r15 (64-bit), sp, pc (32-bit), flags (Z S C V), int_mask.
   - memory read/write; **MMIO region `0xF000_0000+` marked as I/O in the memory map** and refused unless forced, because lldb reads memory freely and some MMIO reads have side effects (e.g. UART RX pops).
   - continue, single-step, Ctrl-C interrupt.
   - software breakpoints (TRAP patching), 4 emulated hardware breakpoints, 2 watchpoints (reported *after* the access).
3. VS Code `launch.json` (lldb-dap, `attachCommands`: `target create` + `gdb-remote`), and `lldb-dap.executable-path` → fork build.

*Exit (first VS Code milestone):* F5 on a bare-metal C program built at `-O1 -g`:
breakpoints by line, step in/over/out, correct backtrace at every stop,
locals and globals, register view, watchpoint stop.

### Phase 3 — Emulator runs RTOS code (`klausscc`)
1. Model the interrupt controller (INT_MASK / INT_PENDING / INT_VECn), the timer, interrupt entry (the 64-bit context word), `IRET`, `WAIT`, with the hardware auto-clear of the dispatched source.
2. Model, or stub with a once-per-address warning, the other MMIO the FreeRTOS and Zephyr ports touch at boot (cache control, perf counters, LEDs/7-seg, UART RX). An inventory comes first.
3. **Unwinding through interrupts:** give the interrupt entry stubs in the FreeRTOS port and Zephyr `swap.S`/ISR code explicit CFI (`.cfi_signal_frame`; return address = low 32 bits of `[SP]`).
4. Timer timing is cycle-agnostic in the emulator (ticks every N instructions). Interrupt *timing* won't match the RTL. This is accepted; it is not a golden-trace mode.

*Exit:* FreeRTOS `demo.elf` and Zephyr hello/shell boot and run under the
emulator; a breakpoint in a task stops with a full backtrace, including through
an interrupt frame.

### Phase 4 — RTOS thread awareness (works on both backends)

**As built:** an lldb Python OS plugin (`klausscpu-runtime/debug/klausscpu_rtos.py`)
instead of code in the gdbserver.  It walks the kernel lists through lldb with
the program's own DWARF, so items 2–3 below need no offset tables and no
`gimli`, and it works unchanged on the board because it only reads memory.
Zephyr uses `_kernel.threads` (`CONFIG_THREAD_MONITOR`) rather than
`CONFIG_DEBUG_THREAD_INFO`.  An lldb fix was needed so plug-in threads refresh
their name and core at each stop.  The original design, for reference:

1. Thread list over RSP (`qfThreadInfo`, `qThreadExtraInfo`, `Hg`, per-thread `g`).
2. **Zephyr:** use `CONFIG_DEBUG_THREAD_INFO` exported offsets (`_kernel_thread_info_offsets`), as OpenOCD does.
3. **FreeRTOS:** `pxCurrentTCB`, ready/delayed/suspended/terminated lists; struct offsets read from the ELF's DWARF (`gimli`) rather than hard-coded.
4. Per-port **saved-context layout** (how each port's context switch stacks registers) documented in each port and decoded by the gdbserver.

*Exit:* pausing a FreeRTOS or Zephyr program in VS Code lists every task, with
a correct backtrace for each.

### Phase 5 — Debug module RTL (`klaussCPU`, `DEBUG_EN`)
1. **JTAG front end:** `BSCANE2` on **USER3** (clear of Vivado's debug hub on USER1). The DMI-style data register is in the TCK domain, crossing to 100 MHz via a req/ack handshake. TCK gets `create_clock` + `set_clock_groups -asynchronous`.
2. **Halt / resume / step:**
   - halt at an instruction boundary through the park mechanism, with a new park kind DEBUG.
   - step = run exactly one retired instruction with interrupts masked, then halt again.
   - halt cause register: request, step, hardware breakpoint n, watchpoint n, TRAP, ILLEGAL, HCF.
3. **When a debugger is attached** (DM `ATTACHED` bit): TRAP, ILLEGAL and HCF enter debug halt **instead of the crash dump**, so crashes become debug sessions. Detached, behaviour is unchanged.
4. **Register access** while halted: r0–r15, PC, SP, flags, INT_MASK, through a mux on the pipeline-owned state.
5. **Memory access** while halted: 64-bit and byte-enable reads and writes **through the CPU memory port / D-cache** (coherent). Stores into code are already snooped into the I-cache; use the `ICACHE_INV` path if that doesn't cover DM writes. MMIO allowed (the host decides).
6. **4 PC comparators**: halt *before* executing the matching instruction. **2 watchpoints**: address + size-mask, R/W/RW; halt *after* the access retires, with the address reported.
7. Freeze the timer and perf counters while halted (`dbg_halted` gate).
8. **Halt-at-entry:** the UART loader path checks a DM `HALT_ON_LOAD` bit before handing off to `PIPE_RUN`.
9. Core 2 is unaffected.

*Tests:*
- cocotb + Verilator on `pipeline_core` + DM + memory model, driving the DMI directly (BSCANE2 wrapped and stubbed): halt during a cache miss, halt with an IRQ pending, step over CALL/RET/branches/IRET, step never enters an ISR, breakpoint on a 2- or 3-word instruction, watchpoint on a byte store inside a 64-bit word, TRAP while attached vs detached, register and memory round-trips, self-modifying-code breakpoint insertion.
- **`tb_pipeline_isa` golden traces unchanged** with `DEBUG_EN=0` and with `DEBUG_EN=1` idle.
- `tb_soc` in xsim for the full SoC with the DM.

*Exit:* all of the above pass; implementation meets timing (WNS ≥ 0) with
`DEBUG_EN=1`; LUT delta recorded; no BRAM used.

### Phase 6 — Host JTAG backend (`klausscc`)
1. FT2232 channel A via MPSSE (libftdi1), detaching the kernel driver from channel A only; the console UART on channel B stays usable. Digilent cable GPIO/init values are taken from `openFPGALoader`'s Digilent cable definition.
2. TAP control, `USER3` IR, DMI scans, batched for throughput.
3. `klausscc gdbserver --board`: the same `gdbstub` Target as the emulator, behind a backend trait.
4. Detects Vivado `hw_server` holding the cable and fails with a clear message.
5. Install `openFPGALoader` so the board can be programmed without opening Vivado's hardware manager.

*Exit:* on the board from the command line: attach, halt, read and write
registers and memory, step, hardware breakpoint, resume; memory read throughput
recorded.

### Phase 7 — Board end to end
1. VS Code task: build → UART load with `HALT_ON_LOAD` → `klausscc gdbserver --board` → lldb-dap attach. One F5.
2. Doom, FreeRTOS and Zephyr sessions with thread view.
3. `perf/debug/run_debug_board.sh`: scripted `lldb -b` session (break, step, bt, frame variable, watchpoint) with expected output, for regression.

*Done when:* F5 in VS Code stops at a breakpoint in Doom's renderer **on the
FPGA**, steps through C source at `-O1`, shows locals, the call stack and
registers, lists RTOS threads in a FreeRTOS/Zephyr build, and a crash under an
attached debugger stops at the faulting instruction instead of producing a
crash dump.

## 4. Debug-module register map (to be fixed at the start of Phase 5)

A DMI-style interface: one JTAG data register of `{addr[7:0], data[63:0], op[1:0]}`.
Draft registers, final layout to be settled before RTL and host work split:

| Addr | Name | Purpose |
|---|---|---|
| 0x00 | `DMCONTROL` | haltreq, resumereq, step, attached, halt_on_load, ndmreset |
| 0x01 | `DMSTATUS` | halted, running, cause[3:0], busy, error, version |
| 0x02 | `REG_SEL` | register number (r0–r15, PC, SP, FLAGS, INT_MASK) + read/write |
| 0x03 | `DATA0` | 64-bit data for register / memory access |
| 0x04 | `MEM_ADDR` | address; auto-increment flag |
| 0x05 | `MEM_CTRL` | size / byte-enables, read/write, go |
| 0x10–0x13 | `BP0–3` | PC + enable |
| 0x18–0x19 | `WP0–1_ADDR` | address |
| 0x1A–0x1B | `WP0–1_CTRL` | size mask, R/W/RW, enable |
| 0x1F | `HIT_ADDR` | address of the last watchpoint hit |

## 5. Risks

| Risk | Mitigation |
|---|---|
| **Timing (WNS +0.003 ns)** | Registered DM interfaces, comparators off the fetch critical path (compare one stage late and flush), `DEBUG_EN=0` fallback. |
| lldb port effort is open-ended | Use a stub-supplied `target.xml` and a minimal ABI; follow RISC-V / LoongArch plugins. |
| `-O1` variable quality is reduced by custom MI passes (`KlaussCPUFlagReuse`, `KlaussCPUFuseCmpBr`) | Check with `llvm-dwarfdump --statistics` before and after; MIR debugify on those passes. |
| Unwinding through interrupt frames | Explicit CFI in the ISR stubs; the ABI masks the return address to 32 bits. |
| Vivado `hw_server` / ILA share the JTAG cable | Detect and report; program with `openFPGALoader`; ILA and debugging are used one at a time. |
| Digilent FT2232 channel-A specifics | Copy `openFPGALoader`'s known-good cable init. |
| Debugger reads of MMIO with side effects | gdbserver memory map marks MMIO as I/O; refused unless forced. |
| Software breakpoints in relocated/copied code (Zephyr LLEXT) | Hardware breakpoints as fallback; LLEXT load hook later. |
| Emulator IRQ semantics drift from the RTL | Emulator IRQ model mirrors §13.1 and is checked against the `run_m5c` directed tests' architectural (not timing) effects. |

## 6. Repos and branches

| Repo | Branch | Phases |
|---|---|---|
| `klausscpu-llvm` | `debug-support` | 0, 1 |
| `klausscc` | `debug-support` | 2, 3, 4, 6 |
| `klausscpu-runtime` | `debug-support` | 3 (ISR CFI, MMIO inventory), 4 (thread info config), 7 (VS Code configs, scripts) |
| `klaussCPU` | `debug-support` | 5, 7 (board script), this plan |

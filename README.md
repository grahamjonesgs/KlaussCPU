# KlaussCPU

A custom 64-bit CPU on FPGA (Xilinx Artix-7, Nexys A7 100T) with its own
Verilog implementation, LLVM backend, and on-board peripherals (DDR2 cache,
UART, SD over SPI, 7-segment + RGB LEDs, timer interrupts, 10/100 Ethernet
via LiteEth).

## Documentation map

| Doc | What's in it |
|-----|--------------|
| [CPU_ARCHITECTURE.md](CPU_ARCHITECTURE.md) | Instruction set, register file, opcode encoding, pipeline stages, interrupt model. The reference for anyone writing assembly or extending the ISA. |
| [MMIO_MAP.md](MMIO_MAP.md) | Memory-mapped I/O peripherals (`0xF000_0000+`). Register tables for SD, UART, RGB, 7-seg, LEDs/switches, cache controller, timer/IRQ, **Ethernet (LiteEth)**. Includes the `mmio.h` C header and driver sketches. |
| [CRASH_DUMP.md](CRASH_DUMP.md) | Format of the UART crash dump emitted on HCF (halt-and-catch-fire). Field-by-field decode of the dump bytes — used by the human reading them after a fault. |
| [ETHERNET_PLAN.md](ETHERNET_PLAN.md) | Phased plan + decision log + risk log for the Ethernet integration. Captures the full debug journey: bridge alignment fix, MDIO TA quirk, RMII clock-skew fix via ODDR inversion. |
| [llvm_backend_plan.md](llvm_backend_plan.md) | LLVM target plan for the KlaussCPU ISA (instruction selection, calling convention, lowering). |
| [LLVM_NEW_INSTRUCTIONS.md](LLVM_NEW_INSTRUCTIONS.md) | New ISA additions and how they're wired into the LLVM backend. |
| [tools/liteeth/REGENERATE.md](tools/liteeth/REGENERATE.md) | How to rebuild [liteeth_core.v](KlaussCPU.srcs/sources_1/new/liteeth_core.v) from [liteeth_nexys_a7.yml](tools/liteeth/liteeth_nexys_a7.yml) — toolchain setup, regen command, sanity checks, project-specific tweaks that don't live in the YAML. |

## Repository layout

```
KlaussCPU/
├── KlaussCPU.srcs/
│   ├── sources_1/new/        HDL sources (the actual CPU)
│   │   ├── KlaussCPU.v       Top-level — port list and module instantiations
│   │   ├── opcode_select.vh  Opcode dispatch (the case statement that drives the FSM)
│   │   ├── control_tasks.vh  Tasks for flow control / interrupts / TRAP / IRET
│   │   ├── register_tasks.vh Tasks for register ops (SETR, SETR64, etc.)
│   │   ├── alu_extended_tasks.vh  ALU / arithmetic / load-store tasks
│   │   ├── stack_tasks.vh    PUSH/POP/PUSHV/PUSHV64/CALL/RET
│   │   ├── memory_tasks.vh   Wide MMIO load/store tasks
│   │   ├── uart_tasks.vh     UART helpers + crash dump emitter
│   │   ├── led_tasks.vh      LED + RGB LED + switch tasks
│   │   ├── seven_seg.vh      7-segment display tasks
│   │   ├── timing_tasks.vh   Timer-related opcodes
│   │   ├── mem_read_write.v  L1 cache controller + clk_wiz instantiation
│   │   ├── ddr2_control.v    MIG wrapper, ddr2 user interface
│   │   ├── bus_splitter.v    Routes CPU memory bus → DRAM / MMIO / Eth
│   │   ├── eth_mmio_bridge.v MMIO ↔ LiteEth Wishbone translation
│   │   ├── liteeth_core.v    GENERATED — see tools/liteeth/REGENERATE.md
│   │   ├── uart_rx.v, uart_tx.v, uart_send_msg.v, uart_rx_fifo.v
│   │   ├── SPI_Master*.v, sd_spi.v
│   │   ├── stack.v, ram_sp.v, bus_splitter.v
│   │   ├── Seven_seg_LED_Display_Controller.v, RGB_LED.v
│   │   └── functions.vh
│   ├── sources_1/ip/         Xilinx IP — clk_wiz_0 (clocks), mig_7series_0 (DDR2), ila_0 (debug)
│   └── constrs_1/imports/new/
│       ├── nexys_ddr.xdc     Pin/timing constraints
│       └── liteeth_core.xdc  Emitted by LiteEth — kept for reference; not used
├── tools/
│   └── liteeth/              LiteEth regeneration assets (YAML + instructions)
└── *.md                      Architecture documentation (this dir)
```

## First-time setup pointers

- **Building the FPGA design** — open `KlaussCPU.xpr` in Vivado 2024.1+. Run synthesis, then implementation, then generate bitstream. Programmable target: Nexys A7 100T over JTAG.
- **Writing programs for the CPU** — use the LLVM target in `~/Documents/src/llvm-project/llvm/lib/Target/KlaussCPU/`. Sample programs in `…/Target/KlaussCPU/runtime/`.
- **Regenerating `liteeth_core.v`** — only needed if changing LiteEth config. See [tools/liteeth/REGENERATE.md](tools/liteeth/REGENERATE.md).
- **Crash dumps** — emitted over UART on HCF. Decode per [CRASH_DUMP.md](CRASH_DUMP.md).

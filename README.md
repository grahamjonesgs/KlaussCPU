# KlaussCPU

A custom 64-bit CPU on FPGA (Xilinx Artix-7, Nexys A7 100T) with its own
SystemVerilog implementation, LLVM backend, and on-board peripherals (DDR2
cache, UART, SD over SPI, 7-segment + RGB LEDs, timer interrupts, 10/100
Ethernet via LiteEth, AES-128/GCM + SHA-256/HMAC + TRNG crypto engines, a 2D
DMA blitter, and a second AMP core running the network stack).

Execution is a 5-stage in-order pipeline (`pipeline_core.sv`) at 100 MHz,
synchronous with the DDR2 controller's ui_clk (2:1 MIG). The original
multicycle FSM in `KlaussCPU.sv` still handles boot, program load, and the
crash dump, then hands the memory port to the pipeline (`PIPE_RUN`).

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
│   │   ├── KlaussCPU.sv      Top level — boot/loader/crash-dump FSM, MMIO hub,
│   │   │                     IRQ controller + timer, perf counters
│   │   ├── pipeline_core.sv  5-stage pipeline (owns rf/flags/SP/PC/int_mask;
│   │   │                     I-cache + next-line buffer + sliding fetch window)
│   │   ├── klauss_pkg.sv     Shared package — flags_t, decode structs, enums
│   │   ├── membus_if.sv, mmio_if.sv  Bus interfaces (memory port, MMIO slaves)
│   │   ├── uart_tasks.vh     Crash-dump formatter + UART TX message task
│   │   ├── mem_read_write.sv L1 D-cache (2-way, 32 B lines) + DDR arbiter
│   │   │                     (cache / blitter / core-2) + maintenance walks
│   │   ├── ddr2_control.sv   MIG wrapper (2:1, ui_clk = 100 MHz CPU clock)
│   │   ├── bus_splitter.sv   Routes CPU memory bus → DRAM / MMIO / Eth
│   │   ├── boot_rom.sv       Resident netboot image, copied to DDR at boot
│   │   ├── blitter_dma.sv    2D DMA blitter (MMIO 0xF00E_xxxx)
│   │   ├── core2_subsys.sv   AMP core 2 — second pipeline_core at ce/2 with
│   │   │                     local BRAM, DDR master C, LiteEth owner mux
│   │   ├── crypto_aes.sv, aes_core.sv, aes_sbox.sv, ghash.sv   AES-128 + GCM
│   │   ├── crypto_sha.sv, sha256_core.sv                       SHA-256 + HMAC
│   │   ├── trng.sv, ring_osc.sv                                TRNG (16 ROs)
│   │   ├── eth_mmio_bridge.sv MMIO ↔ LiteEth Wishbone translation
│   │   ├── liteeth_core.v    GENERATED — see tools/liteeth/REGENERATE.md
│   │   ├── uart_rx.sv, uart_tx.sv, uart_send_msg.sv, uart_rx_fifo.sv
│   │   ├── SPI_Master*.sv, sd_spi.sv
│   │   └── Seven_seg_LED_Display_Controller.sv, RGB_LED.sv
│   ├── sim_1/new/            Testbenches — tb_pipeline_isa (golden-trace +
│   │                         IRQ storm + SMC + maskrace), tb_soc (full SoC
│   │                         boot→run vs emulator), tb_cache, tb_blitter, …
│   ├── sources_1/ip/         Xilinx IP — clk_wiz_0 (clocks), mig_7series_0 (DDR2), ila_0 (debug)
│   └── constrs_1/imports/new/
│       ├── nexys_ddr.xdc     Pin/timing constraints
│       └── liteeth_core.xdc  Emitted by LiteEth — kept for reference; not used
├── perf/m5a/                 Verification runners: run_m5a.sh (core vs emulator
│                             golden trace), run_m5c.sh (IRQ/WAIT/SMC/maskrace),
│                             run_m5d_soc.sh (full-SoC boot + UART identity),
│                             run_m5e_board.sh (on-silicon suite), + captures
├── tools/
│   ├── liteeth/              LiteEth regeneration assets (YAML + instructions)
│   └── netboot/              Netboot tooling
└── *.md                      Architecture documentation (this dir)
```

## First-time setup pointers

- **Building the FPGA design** — open `KlaussCPU.xpr` in Vivado 2025.2+. Run synthesis, then implementation, then generate bitstream. Programmable target: Nexys A7 100T over JTAG.
- **Running the verification suites** — `perf/m5a/run_m5a.sh <prog>` diffs the RTL retire trace against the `klausscc` emulator golden; `run_m5c.sh` runs the directed IRQ/WAIT/SMC/mask-race tests; `run_m5d_soc.sh <prog>` boots the full SoC in xsim and requires UART byte-stream identity; `run_m5e_board.sh` runs the suite on silicon.
- **Writing programs for the CPU** — use the LLVM target in `~/Documents/src/llvm-project/llvm/lib/Target/KlaussCPU/`. Sample programs in `…/Target/KlaussCPU/runtime/`.
- **Regenerating `liteeth_core.v`** — only needed if changing LiteEth config. See [tools/liteeth/REGENERATE.md](tools/liteeth/REGENERATE.md).
- **Crash dumps** — emitted over UART on HCF. Decode per [CRASH_DUMP.md](CRASH_DUMP.md).

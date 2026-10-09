
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 09/24/2020 01:15:33 PM
// Design Name:
// Module Name: SPI_top
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
module KlaussCPU (
    input             CPU_RESETN,        // CPU reset button
    input             i_Clk_board,       // 100 MHz board oscillator (feeds the MIG 200MHz ref)
    input             i_uart_rx,
    input             i_load_H,          // Load button
    output            o_uart_tx,
    output     [15:0] o_led,
    output            o_SPI_LCD_Clk,
    input             i_SPI_LCD_MISO,
    output            o_SPI_LCD_MOSI,
    output            o_SPI_LCD_CS_n,
    output logic        o_LCD_DC,
    output logic        o_LCD_reset_n,
    output     [ 7:0] o_Anode_Activate,  // anode signals of the 7-segment LED display
    output     [ 7:0] o_LED_cathode,     // cathode patterns of the 7-segment LED display
    input      [15:0] i_switch,
    output     [ 2:0] o_LED_RGB_1,
    output     [ 2:0] o_LED_RGB_2,

    // microSD slot (Nexys A7, SPI mode)
    output            o_SD_RESET,    // active-low slot power gate
    input             i_SD_CD,       // card detect
    output            o_SD_SCK,
    output            o_SD_MOSI,
    input             i_SD_MISO,
    output            o_SD_CS_n,     // SD_DAT[3] used as CS in SPI mode
    output            o_SD_DAT1,     // unused in SPI mode — driven high
    output            o_SD_DAT2,     // unused in SPI mode — driven high

    // Ethernet (Nexys A7 SMSC LAN8720A PHY, RMII)
    output            ETH_MDC,
    inout             ETH_MDIO,
    output            ETH_RSTN,
    input             ETH_CRSDV,
    input             ETH_RXERR,           // not used by LiteEth's RMII PHY; declared for pin completeness
    input      [ 1:0] ETH_RXD,
    output            ETH_TXEN,
    output     [ 1:0] ETH_TXD,
    output            ETH_REFCLK,          // 50 MHz to PHY (driven via ODDR from clk_50)

    // VGA connector (12-bit resistor DAC; VGA_PLAN.md)
    output     [ 3:0] VGA_R,
    output     [ 3:0] VGA_G,
    output     [ 3:0] VGA_B,
    output            VGA_HS,
    output            VGA_VS,

    // DDR2 Physical Interface Signals
    //Inouts
    inout [15:0] ddr2_dq,
    inout [1:0] ddr2_dqs_n,
    inout [1:0] ddr2_dqs_p,
    // Outputs
    output [12:0] ddr2_addr,
    output [2:0] ddr2_ba,
    output ddr2_ras_n,
    output ddr2_cas_n,
    output ddr2_we_n,
    //output ddr2_reset_n,
    output [0:0] ddr2_ck_p,
    output [0:0] ddr2_ck_n,
    output [0:0] ddr2_cke,
    output [0:0] ddr2_cs_n,
    output [1:0] ddr2_dm,
    output [0:0] ddr2_odt
);

   // Synchronous clocking: the WHOLE CPU runs on the MIG ui_clk (100 MHz at
   // 2:1), exposed by mem_read_write below as o_ui_clk. `i_Clk` is kept as the
   // name every `always_ff @(posedge i_Clk)` and `.i_Clk(i_Clk)` already uses — it is
   // now an internal net driven by ui_clk, not the board oscillator. The board
   // oscillator (i_Clk_board) feeds only the MIG's 200 MHz reference (via clk_wiz
   // inside mem_read_write). This makes the cache↔MIG crossing a single domain.
   wire w_ui_clk;
   wire i_Clk = w_ui_clk;

   localparam STACK_TOP = 32'h800_0000;  // one doubleword (8 bytes) above top of 128 MiB byte address space

   // CPU-wide types and constants — FSM state enum e_sm_t, crash-dump error
   // codes, and crash-dump phase boundaries — live in klauss_pkg so modules and
   // testbenches share one authoritative definition.  This wildcard import is a
   // module-scope declaration, so the names are visible to the `include task
   // files pulled in further down.  r_fault_sm (the crash-dump snapshot) stays a
   // plain logic [33:0] so its nibble slicing in uart_tasks.vh keeps working.
   import klauss_pkg::*;

   // (Error codes ERR_* and crash-dump phase boundaries DUMP_* now live in
   //  klauss_pkg, imported above.)

   // UART receive control
   wire [7:0] w_uart_rx_value;  // Received value
   wire w_uart_rx_DV;  // receive flag
   wire w_uart_break;  // Break condition detected
   logic  r_break_received;  // Set after break, cleared after command byte

   // UART RX FIFO — buffers bytes for the CPU to read via RXRB / RXRNB opcodes
   wire       w_rx_fifo_empty;
   wire       w_rx_fifo_full;
   wire [7:0] w_rx_fifo_byte;   // combinatorial peek at FIFO head

   // LCD control
   logic [3:0] o_TX_LCD_Count;  // # bytes per CS low
   logic [7:0] o_TX_LCD_Byte;  // Byte to transmit on MOSI
   logic o_TX_LCD_DV;  // Data Valid Pulse with i_TX_Byte
   wire i_TX_LCD_Ready;  // Transmit Ready for next byte

   // RX (MISO) Signals
   wire [3:0] i_RX_LCD_Count;  // Index RX byte
   wire i_RX_LCD_DV;  // Data Valid pulse (1 clock cycle)
   wire [7:0] i_RX_LCD_Byte;  // Byte received on MISO


   // Machine control

   // Invariant: the FSM is one-hot every cycle (guards the manual one-hot state
   // allocation the design depends on).  Concurrent SVA — Vivado synthesis
   // ignores it (netlist unaffected); it is checked under xsim/formal.  Gated on
   // !$isunknown so it skips the power-up window before the initial block runs.
   a_sm_onehot: assert property (@(posedge i_Clk)
                                 (!$isunknown(st.SM)) |-> $onehot(st.SM))
      else $error("st.SM not one-hot: %h", st.SM);
   // Snapshot of st.SM at fault time.  st.SM gets overwritten by the HCF chain
   // (HCF_1 → HCF_DUMP → ...) before the dump emits, so dumping st.SM directly
   // is useless.  We continuously copy st.SM into r_fault_sm while the FSM is
   // in any *non-HCF* state; the moment the trap fires, r_fault_sm freezes
   // at the state that was running immediately before the transition into
   // HCF_1, which is what we actually want in the crash dump.
   logic [33:0] r_fault_sm = 34'b0;
   logic [31:0] r_mem_read_addr;
   wire [31:0] w_opcode;
   wire [31:0] w_var1;
   wire [31:0] w_var2;
   wire [31:0] w_mem;
   logic r_hcf_message_sent;
   logic [31:0] r_start_wait_counter;

   // Crash-dump trace ring buffer — captures {PC, opcode} of every instruction
   // the pipeline retires (pip_ret_*, in PIPE_RUN).  On HCF entry the most recent 16 entries (newest at
   // r_trace_idx-1) are flushed over UART so a crash log shows the branch-history
   // leading up to the failing instruction, not just the failing instruction itself.
   logic [63:0] r_trace_buf [0:15];
   logic [3:0]  r_trace_idx;          // next-write index, wraps freely
   // Free-running count of instructions committed since program load. 32-bit
   // wraps at ~4.3e9 — plenty for crash diagnostics. Reset on initial,
   // CPU_RESETN, and successful program load (LOAD_COMPLETE).
   logic [31:0] r_instr_count;
   logic        r_trace_full;         // 1 once the ring has wrapped at least once

   // Crash dump UART state machine.  Lives entirely inside HCF_DUMP; the sub-state
   // r_hcf_dump_sub walks each line through the UART handshake used elsewhere
   // (PREP → BYTE_BUILD → ACK → DONE_WAIT), with an extra STACK_FETCH branch
   // for lines that need a DDR2 read first.
   //
   // BYTE_BUILD streams r_msg one byte per cycle from f_dump_byte (see
   // uart_tasks.vh).  The legacy single-cycle build collapsed to a 207-input
   // 256-bit mux in synth; the byte-streamed form is an 8-bit mux instead.
   logic [6:0]  r_hcf_dump_phase;     // which dump line to emit (see DUMP_* localparams)
   logic [2:0]  r_hcf_dump_sub;       // 000=PREP, 001=ACK, 010=DONE_WAIT, 011=STACK_FETCH, 100=BREAK, 101=BYTE_BUILD, 110=PRE_DISPLAY
   logic [4:0]  r_hcf_dump_byte_pos;  // current byte index during BYTE_BUILD (0..length-1)
   logic [63:0] r_hcf_stack_data;    // captured stack doubleword for the active stack phase
   logic        r_hcf_stack_loaded;   // 1 when r_hcf_stack_data is valid for current phase

   //load control
   //reg          o_ram_write_DV;
   logic [31:0] o_ram_write_value;
   logic [31:0] o_ram_write_addr;
   logic [31:0] r_ram_next_write_addr;
   logic [2:0] r_load_byte_counter;
   logic [15:0] r_checksum;
   logic [15:0] r_old_checksum;
   logic [15:0] r_calc_checksum;
   logic [15:0] r_rec_checksum;
   logic [31:0] r_PC_requested;

   // Register control
   logic [63:0] r_register[15:0];

   // Display value
   logic r_error_display_type;

   // Stack control — stack now lives in DDR2 RAM, top of 128 MiB, growing down
   // SP = 32'h800_0000 means empty; PUSH: SP-=8, mem[SP]=val; POP: val=mem[SP], SP+=8
   // R15 is the frame pointer by convention (software convention only, no hardware enforcement)

   // UART send message
   logic [255:0] r_msg;  // 32 bytes — longest message is 21 bytes (case 2 of t_tx_message)
   logic [7:0] r_msg_length;
   logic r_msg_send_DV;
   wire i_msg_sent_DV;
   wire w_sending_msg;

   // String transmission state machines (for TXSTRMEM and TXSTRMEMR)
   // (r_tx_str_* string-streaming state removed — TXSTRMEM/TXSTRMEMR retired; UART is MMIO)

   // temp vars for timing

   // Interrupt handler
   logic [31:0] r_interrupt_table[3:0];
   logic r_timer_interrupt;
   logic [31:0] r_timer_interrupt_counter;
   logic [63:0] r_timer_interrupt_counter_sec;
   // Per-source interrupt enable. Bit N = source N enabled (1) / masked (0).
   // Software-controlled via INTMASKR/INTMASKV. Hardware auto-clears the
   // dispatched source's bit on entry; IRET restores the 4-bit mask from the
   // stack slot (bits [42:39] of the saved context word).
   // Timer-interrupt period in raw clock cycles. Software-controlled via
   // TIMERSETR/TIMERSETV. Counter rolls over (and asserts r_timer_interrupt)
   // when r_timer_interrupt_counter > r_timer_period.
   logic [31:0] r_timer_period;

   // Interrupt request to the pipeline — an unmasked, vectored source is
   // pending (pipeline_core dispatches it, and wakes a WAIT, on w_irq_ready).
   //   source 0 = timer (r_timer_interrupt, hardware-cleared on dispatch)
   //   source 1 = blitter DONE (w_blit_irq = blitter r_done & IRQ_EN; level —
   //              the ISR must ack by writing STATUS.DONE (W1C) before IRET)
   // A source fires only when it has a non-zero vector AND its mask bit is set.
   //   source 2 = LiteEth RX/TX event (r_eth_irq = registered LiteEth
   //              `interrupt` = OR of (EV_PENDING & EV_ENABLE); level — the ISR
   //              must W1C RX_/TX_EV_PENDING before IRET). Only while core 1
   //              owns the MAC (C2_ETH_OWNER clear).
   //   source 3 = VGA vsync (w_vga_irq = VSYNC_PENDING & VSYNC_IRQ_EN, set at
   //              the start of vblank; level — the ISR must W1C VGA_STATUS[1]
   //              before IRET).
   // w_irq_sel picks the source to dispatch: timer (0) > blitter (1) > eth (2)
   // > vga (3).
   // M5d: the pipeline core owns int_mask; the FSM's st.int_mask is retired
   // from the gating (kept in st only as a dormant field).
   wire [3:0] pip_int_mask;
   wire w_blit_irq;   // blitter DONE — declared here (used below, driven at the blitter instance)
   wire w_irq_src0  = r_timer_interrupt && (r_interrupt_table[0] != 32'h0) && pip_int_mask[0];
   wire w_irq_src1  = w_blit_irq        && (r_interrupt_table[1] != 32'h0) && pip_int_mask[1];
   logic r_eth_irq;   // registered LiteEth interrupt (driven after the LiteEth instance)
   wire w_irq_src2  = r_eth_irq         && (r_interrupt_table[2] != 32'h0) && pip_int_mask[2];
   wire w_vga_irq;    // VGA vsync — driven at the vga_ctrl instance
   wire w_irq_src3  = w_vga_irq         && (r_interrupt_table[3] != 32'h0) && pip_int_mask[3];
   wire w_irq_ready = w_irq_src0 || w_irq_src1 || w_irq_src2 || w_irq_src3;
   wire [1:0] w_irq_sel = w_irq_src0 ? 2'd0 : w_irq_src1 ? 2'd1 : w_irq_src2 ? 2'd2 : 2'd3;

   // Free-running millisecond counter (since LOAD_COMPLETE). 64-bit so it
   // takes ~5.8e8 years to wrap at 100 MHz. Read-only via MMIO 0xF00F_0040.
   // r_clock_ms_div counts 0..99_999 (one ms at 100 MHz) before incrementing
   // r_clock_ms — 17 bits hold up to 131071 so 99_999 fits comfortably.
   logic [63:0] r_clock_ms;
   logic [16:0] r_clock_ms_div;

   // Memory
   wire [63:0] w_mem_read_data;
   wire [63:0] w_mem_read_data_next; // next doubleword in same cache line
   wire        w_mem_next_valid;     // 1 when w_mem_read_data_next is valid

   // -------------------------------------------------------------------------
   // Bus splitter outputs — DRAM side (to mem_read_write) and MMIO side
   // (to peripheral logic below). Splitter routes on i_mem_addr[31:28]:
   //   4'hF → MMIO, else DRAM. CPU FSM sees only the original r_mem_*/w_mem_*
   //   signals; routing is invisible above this line.
   // -------------------------------------------------------------------------
   // (The bus_splitter<->mem_read_write DRAM bus is now the dram_mem interface
   //  instance, declared near the bus_splitter instantiation below.)

   wire        w_mmio_write_DV;
   wire        w_mmio_read_DV;
   wire [31:0] w_mmio_addr;
   wire [63:0] w_mmio_write_data;
   wire [ 7:0] w_mmio_byte_en;

   // Ethernet block — bus_splitter routes 0xF006/F007/F008 to the bridge.
   wire        w_eth_write_DV;
   wire        w_eth_read_DV;
   wire [31:0] w_eth_addr;
   wire [63:0] w_eth_write_data;
   wire [ 7:0] w_eth_byte_en;
   wire [63:0] w_eth_read_data;
   wire        w_eth_ready;

   // 50 MHz Ethernet REF_CLK domain (generated by clk_wiz_0 inside mem_read_write).
   wire        clk_50;

   // LiteEth Wishbone master interface (driven by eth_mmio_bridge).
   wire [29:0] w_eth_wb_adr;
   wire [31:0] w_eth_wb_dat_w;
   wire [31:0] w_eth_wb_dat_r;
   wire [ 3:0] w_eth_wb_sel;
   wire        w_eth_wb_we;
   wire        w_eth_wb_cyc;
   wire        w_eth_wb_stb;
   wire [ 1:0] w_eth_wb_bte;
   wire [ 2:0] w_eth_wb_cti;
   wire        w_eth_wb_ack;
   wire        w_eth_wb_err;
   wire        w_eth_irq;       // LiteEth combined RX/TX event -> interrupt source 2 (r_eth_irq)

   // Cache performance counters / control (driven by mem_read_write below;
   // exposed via MMIO 0xF005_xxxx — see MMIO_MAP.md "Cache controller").
   wire [63:0] w_cache_info;
   wire [63:0] w_cnt_read_hits;
   wire [63:0] w_cnt_read_misses;
   wire [63:0] w_cnt_write_hits;
   wire [63:0] w_cnt_write_misses;
   wire [63:0] w_cnt_writebacks;
   wire [63:0] w_cnt_stall_cycles;
   // CACHE_CTRL bit 0 self-clearing: a 1-cycle pulse on the cycle the CPU
   // writes 0xF005_0000 with bit 0 set, fed straight into mem_read_write's
   // i_stat_clear (which zeros all counters on the next edge).
   wire        w_cache_stat_clear = w_mmio_write_DV
                                  && (w_mmio_addr[27:16] == 12'h005)
                                  && (w_mmio_addr[15:0]  == 16'h0000)
                                  && w_mmio_write_data[0];

   // CACHE_CTRL (0xF005_0000) bit1 = FLUSH, bit2 = INVALIDATE — self-clearing
   // 1-cycle pulses into mem_read_write's maintenance FSM (full-cache walk).
   // o_mnt_busy (CACHE_STATUS 0xF005_0010 bit0) reads high during the walk.
   wire        w_cache_flush_go = w_mmio_write_DV
                                && (w_mmio_addr[27:16] == 12'h005)
                                && (w_mmio_addr[15:0]  == 16'h0000)
                                && w_mmio_write_data[1];
   wire        w_cache_inval_go = w_mmio_write_DV
                                && (w_mmio_addr[27:16] == 12'h005)
                                && (w_mmio_addr[15:0]  == 16'h0000)
                                && w_mmio_write_data[2];
   wire        w_cache_mnt_busy;

   // -------------------------------------------------------------------------
   // Performance counters (Tier 0/1/2) — exposed via MMIO 0xF00D_xxxx (see
   // MMIO_MAP.md "Performance counters"). Free-running; cleared by writing
   // PERF_CTRL (0xF00D_0000) bit 0. A dedicated always block (search
   // "Performance-counter block") drives these off existing FSM state and the
   // decoded opcode, so the main CPU datapath / critical path is untouched.
   // -------------------------------------------------------------------------
   // Tier 0 — denominators
   logic [47:0] r_perf_cycles;        // every i_Clk cycle
   logic [47:0] r_perf_instr;         // retired (committed) instructions
   // Tier 1 — cycle accounting. FETCH/MUL/DIV/INT cycle buckets, MUL/DIV_OPS
   // and FASTPATH counted multicycle-CPU states and read 0 since it was
   // removed; pipeline stalls are attributed by the M6 STALL_* counters.
   logic [47:0] r_perf_exec_cycles;   // PIPE_RUN cycles
   logic [47:0] r_perf_idle_cycles;   // HALTED / HALTED_BREAK
   logic [47:0] r_perf_int_ops;       // interrupt dispatches (pip_irq_ack)
   // Tier 2 — instruction mix + branch behaviour
   logic [47:0] r_perf_cnt_alu;
   logic [47:0] r_perf_cnt_load;
   logic [47:0] r_perf_cnt_store;
   logic [47:0] r_perf_cnt_branch;        // conditional branches (cond jumps)
   logic [47:0] r_perf_cnt_branch_taken;  // of those, the taken subset
   logic [47:0] r_perf_cnt_jump;          // unconditional direct jumps (JMP/JMPREL)
   logic [47:0] r_perf_cnt_call;          // direct + conditional calls
   logic [47:0] r_perf_cnt_indirect;      // JMPR / RET / IRET / CALLR (reg/stack target)
   logic [47:0] r_perf_cnt_other;         // mul/div/system/io/nop/etc.
   // M6: pipeline per-hazard attribution (MMIO 0xB0-0xE8; strobes from
   // pipeline_core.perf_stall — see its port comment for the bit map)
   logic [47:0] r_perf_stall_data;     // 0xB0 GPR RAW stall cycles
   logic [47:0] r_perf_stall_loaduse;  // 0xB8 load-use subset
   logic [47:0] r_perf_stall_flags;    // 0xC0 flag-reader stall cycles
   logic [47:0] r_perf_stall_sp;       // 0xC8 SP-serialization stall cycles
   logic [47:0] r_perf_stall_muldiv;   // 0xD0 EX-busy cycles (mul/div/delay/lcd)
   logic [47:0] r_perf_branch_flush;   // 0xD8 taken-redirect events
   logic [47:0] r_perf_if_miss;        // 0xE0 fetch-window miss cycles
   logic [47:0] r_perf_mem_wait;       // 0xE8 MEM port wait cycles
   // Bus-wedge flight recorder (0xF00D_0100/0108, RO; cleared by PERF_CTRL
   // bit 0 like the counters, so a snapshot SURVIVES a program reload). If a
   // CPU bus transaction holds its DV for 2^20 cycles (~10.5 ms — no legal
   // transaction is remotely that long) the full handshake state is latched
   // once: which DV, the address, every ready in the chain, splitter decode
   // inputs, FSM state and IRQ context. Diagnose a hang by reloading with a
   // dump program and reading two regs. snap0: [31:0] addr, [39:32] reserved,
   // [40] read_DV, [41] write_DV, [42] cpu_mem.ready, [43] w_mmio_read_DV,
   // [44] r_mmio_read_dv_d, [45] w_mmio_ready, [46] w_eth_ready,
   // [47] dram_mem.ready, [48] w_pipe_owns, [49] r_timer_interrupt,
   // [50] w_irq_ready, [51] mem_busy (pip_perf_stall[7]), [52] if_miss
   // (pip_perf_stall[6]), [53] pip_bus_idle, [63] snapshot-valid.
   // snap1: [31:0] r_perf_cycles at latch, [35:32] pip_int_mask,
   // [36] w_irq_src0, [37] w_irq_src1, [63:38] r_timer_interrupt_counter[25:0].
   logic [20:0] r_wedge_cnt;
   logic        r_wedge_latched;
   logic [63:0] r_wedge_snap0, r_wedge_snap1;
   // Instruction-class codes (return value of f_perf_class)
   localparam PC_OTHER=3'd0, PC_ALU=3'd1, PC_LOAD=3'd2, PC_STORE=3'd3,
              PC_BRANCH=3'd4, PC_JUMP=3'd5, PC_CALL=3'd6, PC_INDIRECT=3'd7;
   // PERF_CTRL bit 0 self-clearing — same pattern as w_cache_stat_clear.
   wire        w_perf_stat_clear = w_mmio_write_DV
                                 && (w_mmio_addr[27:16] == 12'h00D)
                                 && (w_mmio_addr[15:0]  == 16'h0000)
                                 && w_mmio_write_data[0];

   logic  [63:0] r_mmio_read_data_comb;  // combinational decode (driven by always_comb)
   logic  [63:0] r_mmio_read_data;       // registered version delivered to bus_splitter (breaks long timing path)
   // The read decode's SELECT is also registered (r_mmio_addr_q): the raw
   // w_mmio_addr arrives through the w_pipe_owns (st.SM) address mux, and
   // st.SM -> addr-mux -> giant read decode -> r_mmio_read_data was the
   // recurring WNS family (met at +0.02..0.03 on good seeds, -0.5 on bad).
   // Decoding from an FF gives the whole mux a clean full cycle. MMIO reads
   // are therefore 2-stage: DV at T, addr FF at T+1 (decode), data FF at T+2,
   // ready (dv_d2) at T+2 — the bus_splitter FF delivers data+ready together
   // at T+3. Costs +1 cycle per MMIO read (poll loops only).
   logic  [31:0] r_mmio_addr_q;
   logic         r_mmio_read_dv_d;       // read strobe delayed 1 (addr FF valid)
   logic         r_mmio_read_dv_d2;      // read strobe delayed 2 → ready
   // UART MMIO (0xF001) RX pop-on-read: sel is based on the REGISTERED
   // addr/strobe so it tracks the decode stage; the pop pulse fires the cycle
   // the (pre-pop) head byte is being decoded, and the FIFO advances the edge
   // after — the data FF captures the old head. r_uart_rx_rd_d keeps it a
   // single pulse per read handshake.
   wire          w_uart_rx_rd_sel = r_mmio_read_dv_d & (r_mmio_addr_q[27:16] == 12'h001)
                                                     & (r_mmio_addr_q[15:0]  == 16'h0008);
   logic         r_uart_rx_rd_d;
   wire          w_uart_rx_pop    = w_uart_rx_rd_sel & ~r_uart_rx_rd_d;
   // WRITE acks use the same 2-cycle delayed-DV (edge-tracking) shape as the
   // read ready. A combinational write ack (the original
   // `w_mmio_write_DV | ...`) deadlocks the pipeline's ready-EDGE protocol:
   // after an MMIO READ completes, its ready tail (read dv delay chain + the
   // bus_splitter output FF) stays high for a few cycles; an MMIO STORE
   // issued 1 cycle behind (consecutive load;store — e.g. Zephyr's irq_lock =
   // read int_mask; write int_mask) sees stale-high ready at issue (rdy_armed
   // stays 0), and its own comb ack then bridges the tail so ready NEVER
   // falls -> the core never arms, the write never completes, the whole bus
   // wedges (this was the LVGL boot hang; flight-recorder capture:
   // write_DV=1 / ready=1 forever at 0xF00F_0000). With BOTH acks delayed 2
   // cycles from their own DVs, any back-to-back MMIO pair (read;write,
   // write;write, ...) presents at least one low-ready cycle between the
   // outgoing tail and the incoming ack, so rdy_armed always re-arms:
   // outgoing tail ends at completion+2, the next DV rises at completion+2
   // (earliest, k=1) and its ack at completion+4 — ready is low at
   // completion+3. Write side effects are level-based off the (multi-cycle)
   // DV as before; the extra held cycle changes nothing.
   logic       r_mmio_write_dv_d;      // write strobe delayed 1
   logic       r_mmio_write_dv_d2;     // write strobe delayed 2 → ack
   wire        w_mmio_ready = r_mmio_write_dv_d2 | r_mmio_read_dv_d2;
   wire w_mem_ready;
   logic [31:0] r_opcode_mem;
   logic [31:0] r_var1_mem;
   logic [31:0] r_var2_mem;

   wire w_reset_H;
   logic r_boot_flash;
   
   //=========================================================================
   // Flags: unified Z/S/C/V register (flags_t). The old separate equal/less/ult
   // compare flags are retired — EQ/LT/ULT are derived in f_cond_eval (klauss_pkg).
   //=========================================================================

   
  // Free-running millisecond clock since FPGA boot. Lives in its own always
  // block so it ticks every cycle, independent of the main FSM, UART
  // break/command handling, and reset logic. Initial values come from the
  // top-level initial block (r_clock_ms = 0, r_clock_ms_div = 0). 64-bit
  // counter wraps in ~5.8e8 years at 100 MHz; the divider wraps every 1 ms.
  always_ff @(posedge i_Clk) begin
     if (r_clock_ms_div >= 17'd99_999) begin
        r_clock_ms_div <= 17'd0;
        r_clock_ms     <= r_clock_ms + 64'd1;
     end else begin
        r_clock_ms_div <= r_clock_ms_div + 17'd1;
     end
  end


   // SoC control state (loader / boot copy / PIPE_RUN glue / crash dump /
   // peripheral registers). cpu_state_t lives in klauss_pkg.sv. Instruction
   // execution belongs entirely to pipeline_core (the pre-pipeline multicycle
   // CPU that shared this state machine was removed in M13).
   cpu_state_t st;

   // Track the last non-HCF FSM state so the crash dump can show what was
   // executing at the moment of the trap.  All five HCF states are excluded
   // — HCF_1 fires first, then the chain walks through HCF_DUMP / HCF_2..4
   // and would otherwise overwrite the snapshot before the dump emits.
   always_ff @(posedge i_Clk) begin
      if ((st.SM != HCF_1) && (st.SM != HCF_DUMP) &&
          (st.SM != HCF_2) && (st.SM != HCF_3) && (st.SM != HCF_4)) begin
         r_fault_sm <= st.SM;
      end
   end

   // UART TX break generation — holds o_uart_tx low to signal program end
   wire       w_uart_tx_serial;   // internal serial output from uart_send_msg
   logic        r_break_active;     // when 1, overrides TX line to low (break)
   logic [11:0] r_break_counter;    // countdown: 2500 clocks ≈ 7.5 frames at CLKS_PER_BIT=33

   // Mux: break takes priority over normal TX; idle state is line-high
   assign o_uart_tx  = r_break_active ? 1'b0 : w_uart_tx_serial;

   // CPU_RESETN is a raw button pin: synchronize it into the ui_clk domain
   // (2-FF) before it fans out to the design's thousands of synchronous-reset
   // loads. The whole design is synchronous, so assertion through the
   // synchronizer is fine; what must be clean is the RELEASE edge — raw, it
   // can land inside different FFs' setup windows on different cycles and
   // release parts of the SoC one cycle apart (the XDC's set_false_path was
   // hiding exactly that hazard). The 2'b00 power-up value also yields a
   // clean 2-cycle domain reset once the MIG's ui_clk starts (DDR calibration
   // itself is separately gated by w_calib_done in the boot FSM).
   (* ASYNC_REG = "TRUE" *) logic [1:0] r_rstn_sync = 2'b00;
   always_ff @(posedge i_Clk) r_rstn_sync <= {r_rstn_sync[0], CPU_RESETN};
   assign w_reset_H  = !r_rstn_sync[1];

   // KEEP_HIERARCHY prevents Vivado from flattening these modules' logic into
   // surrounding CPU slices. Without it the placer can scatter sd_spi/splitter
   // cells across the CPU's 64-bit ALU carry chain, lengthening route delay on
   // an already-tight critical path (r_reg_port_b → st.flags.carry, ~25 levels).
   // CPU memory bus (FSM <-> bus_splitter) and DRAM bus (bus_splitter <-> cache),
   // both via membus_if.  The FSM still drives r_mem_* / observes w_mem_*; these
   // assigns bridge those wires to the interface (request out, response back).
   membus_if cpu_mem();
   membus_if dram_mem();
   // ------------------------------------------------------------------------
   // M5d: in PIPE_RUN the 5-stage pipeline_core owns the memory port; in every
   // other state (boot copy / loader / HCF stack reads) the FSM's st.mem_*
   // drive it as before. The handoff points guarantee the loser's port is
   // idle: the pipeline parks only after draining (pip_bus_idle), and the FSM
   // is idle in PIPE_RUN.
   // ------------------------------------------------------------------------
   wire        w_pipe_owns = (st.SM == PIPE_RUN);
   wire [31:0] pip_m_addr;
   wire        pip_m_read_DV, pip_m_write_DV;
   wire [63:0] pip_m_wdata;
   wire [7:0]  pip_m_be;
   assign cpu_mem.write_DV      = w_pipe_owns ? pip_m_write_DV : st.mem_write_DV;
   assign cpu_mem.read_DV       = w_pipe_owns ? pip_m_read_DV  : st.mem_read_DV;
   assign cpu_mem.addr          = w_pipe_owns ? pip_m_addr     : st.mem_addr;
   assign cpu_mem.write_data    = w_pipe_owns ? pip_m_wdata    : st.mem_write_data;
   assign cpu_mem.byte_en       = w_pipe_owns ? pip_m_be       : st.mem_byte_en;
   assign w_mem_read_data       = cpu_mem.read_data;
   assign w_mem_read_data_next  = cpu_mem.read_data_next;
   assign w_mem_next_valid      = cpu_mem.next_valid;
   assign w_mem_ready           = cpu_mem.ready;

   // Pipeline core — the execution engine (owns rf / flags / SP / PC /
   // int_mask; single retire at WB; see pipeline_core.sv + PIPELINE_IMPL.md).
   // Held in reset through the boot/loader states (a fresh program starts from
   // architectural zero, like the emulator); NOT reset in HCF/HALTED so the
   // crash dump can snapshot its state.
   wire w_pip_hold_rst = (st.SM == NO_PROGRAM) || (st.SM == LOADING_BYTE) ||
                         (st.SM == LOAD_COMPLETE) || (st.SM == START_WAIT);
   logic        r_pip_start;
   logic [31:0] r_pip_start_pc;
   wire         pip_ret_valid, pip_ret_wr;
   wire [31:0]  pip_ret_pc, pip_ret_op, pip_ret_wr_addr;
   wire [7:0]   pip_ret_wr_be;
   wire [63:0]  pip_ret_wr_raw;
   wire         pip_parked;
   wire [2:0]   pip_park_kind;
   wire [31:0]  pip_park_pc, pip_park_op;
   wire [63:0]  pip_dbg_r [0:15];
   wire [31:0]  pip_dbg_sp;
   flags_t      pip_dbg_flags;
   wire         pip_irq_ack;
   wire [1:0]   pip_irq_ack_sel;
   wire [7:0]   pip_lcd_byte;
   wire         pip_lcd_dc, pip_lcd_dv, pip_lcd_rst_n, pip_lcd_rst_wr;
   wire         pip_bus_idle;   // gates the park -> FSM bus handoff
   wire [7:0]   pip_perf_stall;
   wire         pip_perf_br, pip_perf_br_taken, pip_perf_fbr_taken;
   // int_mask MMIO write (0xF00F_0000) mirrored into the core — the
   // "MMIO store wins" ordering is preserved inside pipeline_core.
   wire         w_pip_mask_wr = w_mmio_write_DV && (w_mmio_addr[27:16] == 12'h00F)
                                                && (w_mmio_addr[15:0]  == 16'h0000);

   pipeline_core pipeline_core_i (
      .clk        (i_Clk),
      .ce         (1'b1),           // core 1 runs full-rate; ce exists for core 2
      .rst        (w_reset_H || w_pip_hold_rst),
      .start      (r_pip_start),
      .start_pc   (r_pip_start_pc),
      .m_addr     (pip_m_addr),
      .m_read_DV  (pip_m_read_DV),
      .m_write_DV (pip_m_write_DV),
      .m_wdata    (pip_m_wdata),
      .m_be       (pip_m_be),
      .m_rdata    (w_mem_read_data),
      .m_rdata_next (w_mem_read_data_next),
      .m_next_valid (w_mem_next_valid),
      .m_ready    (w_mem_ready && w_pipe_owns),
      .irq_ready  (w_irq_ready),
      .irq_sel    (w_irq_sel),
      .irq_vector (r_interrupt_table[w_irq_sel]),
      .irq_ack    (pip_irq_ack),
      .irq_ack_sel(pip_irq_ack_sel),
      .int_mask_o (pip_int_mask),
      .mask_wr    (w_pip_mask_wr),
      .mask_wdata (w_mmio_write_data[3:0]),
      .lcd_byte   (pip_lcd_byte),
      .lcd_dc     (pip_lcd_dc),
      .lcd_dv     (pip_lcd_dv),
      .lcd_rst_n  (pip_lcd_rst_n),
      .lcd_rst_wr (pip_lcd_rst_wr),
      .lcd_ready  (i_TX_LCD_Ready),
      .bus_idle   (pip_bus_idle),
      .perf_stall (pip_perf_stall),
      .perf_br    (pip_perf_br),
      .perf_br_taken (pip_perf_br_taken),
      .perf_fbr_taken (pip_perf_fbr_taken),
      .ret_valid  (pip_ret_valid),
      .ret_pc     (pip_ret_pc),
      .ret_op     (pip_ret_op),
      .ret_wr     (pip_ret_wr),
      .ret_wr_addr(pip_ret_wr_addr),
      .ret_wr_be  (pip_ret_wr_be),
      .ret_wr_raw (pip_ret_wr_raw),
      .parked     (pip_parked),
      .park_kind  (pip_park_kind),
      .park_pc    (pip_park_pc),
      .park_op    (pip_park_op),
      .dbg_r      (pip_dbg_r),
      .dbg_sp     (pip_dbg_sp),
      .dbg_flags  (pip_dbg_flags)
   );

   (* KEEP_HIERARCHY = "yes" *)
   bus_splitter bus_splitter_i (
       .i_clk(i_Clk),
       .cpu(cpu_mem),     // CPU side  (FSM request / response)
       .dram(dram_mem),   // DRAM side (to mem_read_write)
       // MMIO side (existing peripherals)
       .o_mmio_write_DV(w_mmio_write_DV),
       .o_mmio_read_DV(w_mmio_read_DV),
       .o_mmio_addr(w_mmio_addr),
       .o_mmio_write_data(w_mmio_write_data),
       .o_mmio_byte_en(w_mmio_byte_en),
       .i_mmio_read_data(r_mmio_read_data),
       .i_mmio_ready(w_mmio_ready),
       // Ethernet side (bridge → LiteEth)
       .o_eth_write_DV(w_eth_write_DV),
       .o_eth_read_DV(w_eth_read_DV),
       .o_eth_addr(w_eth_addr),
       .o_eth_write_data(w_eth_write_data),
       .o_eth_byte_en(w_eth_byte_en),
       .i_eth_read_data(w_eth_read_data),
       .i_eth_ready(w_eth_ready)
   );

   // -------------------------------------------------------------------------
   // Per-device chip-selects for MMIO. addr[27:16] picks the device; we gate
   // the write/read strobes so that each peripheral module only sees strobes
   // intended for it. addr[15:0] is the offset within the device's window.
   // -------------------------------------------------------------------------
   wire        w_sd_sel       = (w_mmio_addr[27:16] == 12'h000);
   wire        w_sd_write_DV  = w_mmio_write_DV & w_sd_sel;
   wire        w_sd_read_DV   = w_mmio_read_DV  & w_sd_sel;
   wire [63:0] w_sd_read_data;

   mmio_if sd_bus();
   assign sd_bus.write_DV   = w_sd_write_DV;
   assign sd_bus.read_DV    = w_sd_read_DV;
   assign sd_bus.addr       = w_mmio_addr;
   assign sd_bus.write_data = w_mmio_write_data;
   assign sd_bus.byte_en    = w_mmio_byte_en;
   (* KEEP_HIERARCHY = "yes" *)
   sd_spi sd_spi_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .mmio(sd_bus),
       .i_sd_cd(i_SD_CD),
       .o_sd_reset_n(o_SD_RESET),
       .o_sd_sck(o_SD_SCK),
       .o_sd_mosi(o_SD_MOSI),
       .i_sd_miso(i_SD_MISO),
       .o_sd_cs_n(o_SD_CS_n),
       .o_sd_dat1(o_SD_DAT1),
       .o_sd_dat2(o_SD_DAT2)
   );
   assign w_sd_read_data = sd_bus.read_data;

   // -------------------------------------------------------------------------
   // Crypto AES — device id 0x00A. See CRYPTO_PLAN.md §4 and MMIO_MAP.md.
   // -------------------------------------------------------------------------
   wire        w_aes_sel      = (w_mmio_addr[27:16] == 12'h00A);
   wire        w_aes_write_DV = w_mmio_write_DV & w_aes_sel;
   wire        w_aes_read_DV  = w_mmio_read_DV  & w_aes_sel;
   wire [63:0] w_aes_read_data;

   mmio_if aes_bus();
   assign aes_bus.write_DV   = w_aes_write_DV;
   assign aes_bus.read_DV    = w_aes_read_DV;
   assign aes_bus.addr       = w_mmio_addr;
   assign aes_bus.write_data = w_mmio_write_data;
   assign aes_bus.byte_en    = w_mmio_byte_en;
   (* KEEP_HIERARCHY = "yes" *)
   crypto_aes crypto_aes_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .mmio(aes_bus)
   );
   assign w_aes_read_data = aes_bus.read_data;

   // -------------------------------------------------------------------------
   // Crypto SHA-256 — device id 0x00B. See CRYPTO_PLAN.md §6 and MMIO_MAP.md.
   // -------------------------------------------------------------------------
   wire        w_sha_sel      = (w_mmio_addr[27:16] == 12'h00B);
   wire        w_sha_write_DV = w_mmio_write_DV & w_sha_sel;
   wire        w_sha_read_DV  = w_mmio_read_DV  & w_sha_sel;
   wire [63:0] w_sha_read_data;

   mmio_if sha_bus();
   assign sha_bus.write_DV   = w_sha_write_DV;
   assign sha_bus.read_DV    = w_sha_read_DV;
   assign sha_bus.addr       = w_mmio_addr;
   assign sha_bus.write_data = w_mmio_write_data;
   assign sha_bus.byte_en    = w_mmio_byte_en;
   (* KEEP_HIERARCHY = "yes" *)
   crypto_sha crypto_sha_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .mmio(sha_bus)
   );
   assign w_sha_read_data = sha_bus.read_data;

   // -------------------------------------------------------------------------
   // Crypto TRNG — device id 0x00C. See CRYPTO_PLAN.md §7 and MMIO_MAP.md.
   // -------------------------------------------------------------------------
   wire        w_trng_sel      = (w_mmio_addr[27:16] == 12'h00C);
   wire        w_trng_write_DV = w_mmio_write_DV & w_trng_sel;
   wire        w_trng_read_DV  = w_mmio_read_DV  & w_trng_sel;
   wire [63:0] w_trng_read_data;

   // MMIO bus to the TRNG slave (pilot of the mmio_if interface refactor).
   // Broadcast request driven in; per-peripheral decoded strobes driven in;
   // read_data/ready read back into the existing MMIO mux wires.
   mmio_if trng_bus();
   assign trng_bus.write_DV   = w_trng_write_DV;
   assign trng_bus.read_DV    = w_trng_read_DV;
   assign trng_bus.addr       = w_mmio_addr;        // slave uses mmio.addr[15:0]
   assign trng_bus.write_data = w_mmio_write_data;
   assign trng_bus.byte_en    = w_mmio_byte_en;
   (* KEEP_HIERARCHY = "yes" *)
   trng trng_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .mmio(trng_bus)
   );
   assign w_trng_read_data = trng_bus.read_data;

   // -------------------------------------------------------------------------
   // 2D DMA blitter — device id 0x00E. MMIO slave for operands + START/STATUS;
   // a second DDR master (master B) on mem_read_write's arbiter. See
   // BLITTER_IMPLEMENTATION_PLAN.md and blitter-fpga-handoff.md.
   // -------------------------------------------------------------------------
   wire        w_blit_sel      = (w_mmio_addr[27:16] == 12'h00E);
   wire        w_blit_write_DV = w_mmio_write_DV & w_blit_sel;
   wire        w_blit_read_DV  = w_mmio_read_DV  & w_blit_sel;
   wire [63:0] w_blit_read_data;

   // DMA master wires to mem_read_write (instantiated above).
   wire         w_blit_dma_req;
   wire         w_blit_dma_done;
   wire         w_blit_dma_write_DV;
   wire         w_blit_dma_read_DV;
   wire [31:0]  w_blit_dma_addr;
   wire [127:0] w_blit_dma_write_data;
   wire [15:0]  w_blit_dma_wdf_mask;
   wire [127:0] w_blit_dma_read_data;
   wire         w_blit_dma_ready;
   wire         w_blit_dma_grant;
   // (w_blit_irq declared near the IRQ source gating at the top of the module)

   mmio_if blit_bus();
   assign blit_bus.write_DV   = w_blit_write_DV;
   assign blit_bus.read_DV    = w_blit_read_DV;
   assign blit_bus.addr       = w_mmio_addr;
   assign blit_bus.write_data = w_mmio_write_data;
   assign blit_bus.byte_en    = w_mmio_byte_en;
   (* KEEP_HIERARCHY = "yes" *)
   blitter_dma blitter_dma_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .mmio(blit_bus),
       .o_dma_req(w_blit_dma_req),
       .o_dma_done(w_blit_dma_done),
       .o_dma_write_DV(w_blit_dma_write_DV),
       .o_dma_read_DV(w_blit_dma_read_DV),
       .o_dma_addr(w_blit_dma_addr),
       .o_dma_write_data(w_blit_dma_write_data),
       .o_dma_wdf_mask(w_blit_dma_wdf_mask),
       .i_dma_read_data(w_blit_dma_read_data),
       .i_dma_ready(w_blit_dma_ready),
       .i_dma_grant(w_blit_dma_grant),
       .o_irq(w_blit_irq)
   );
   assign w_blit_read_data = blit_bus.read_data;

   // -------------------------------------------------------------------------
   // VGA output — device id 0x011. 640x480@60 on the ui_clk domain (25 MHz
   // pixel CE). Registers, palette and the vsync IRQ (source 3); see
   // vga_ctrl.sv and VGA_PLAN.md.
   // -------------------------------------------------------------------------
   wire        w_vga_sel      = (w_mmio_addr[27:16] == 12'h011);
   wire        w_vga_write_DV = w_mmio_write_DV & w_vga_sel;
   wire        w_vga_read_DV  = w_mmio_read_DV  & w_vga_sel;
   wire [63:0] w_vga_read_data;
   // (w_vga_irq declared near the IRQ source gating at the top of the module)

   mmio_if vga_bus();
   assign vga_bus.write_DV   = w_vga_write_DV;
   assign vga_bus.read_DV    = w_vga_read_DV;
   assign vga_bus.addr       = w_mmio_addr;
   assign vga_bus.write_data = w_mmio_write_data;
   assign vga_bus.byte_en    = w_mmio_byte_en;
   // DDR master D (scanout) wires to mem_read_write (instantiated below).
   wire         w_vga_dma_req;
   wire         w_vga_dma_done;
   wire         w_vga_dma_read_DV;
   wire [31:0]  w_vga_dma_addr;
   wire [255:0] w_vga_dma_read_data;
   wire         w_vga_dma_ready;
   wire         w_vga_dma_grant;

   (* KEEP_HIERARCHY = "yes" *)
   vga_ctrl vga_ctrl_i (
       .i_Clk   (i_Clk),
       .i_Rst_L (~w_reset_H),
       .mmio    (vga_bus),
       .o_irq   (w_vga_irq),
       .o_dma_req      (w_vga_dma_req),
       .o_dma_done     (w_vga_dma_done),
       .o_dma_read_DV  (w_vga_dma_read_DV),
       .o_dma_addr     (w_vga_dma_addr),
       .i_dma_read_data(w_vga_dma_read_data),
       .i_dma_ready    (w_vga_dma_ready),
       .i_dma_grant    (w_vga_dma_grant),
       .o_vga_r (VGA_R),
       .o_vga_g (VGA_G),
       .o_vga_b (VGA_B),
       .o_vga_hs(VGA_HS),
       .o_vga_vs(VGA_VS)
   );
   assign w_vga_read_data = vga_bus.read_data;

   // -------------------------------------------------------------------------
   // AMP core 2 — device id 0x010.  A second pipeline_core at effective
   // 50 MHz (ce/2 + blanket multicycle, the M12 Stage-C pattern) with a 64 KB
   // local BRAM and a log FIFO, controlled by core 1 through this window.
   // P1 scope: no DDR access, no interrupts, no LiteEth.  See
   // AMP_CORE2_PLAN.md and core2_subsys.sv for the register map.
   // -------------------------------------------------------------------------
   wire        w_c2_sel      = (w_mmio_addr[27:16] == 12'h010);
   wire        w_c2_write_DV = w_mmio_write_DV & w_c2_sel;
   wire        w_c2_read_DV  = w_mmio_read_DV  & w_c2_sel;
   wire [63:0] w_c2_read_data;

   // DDR master-C wires (to mem_read_write's arbiter, instantiated below).
   wire         w_c2_ddr_req, w_c2_ddr_done;
   wire         w_c2_ddr_write_DV, w_c2_ddr_read_DV;
   wire [31:0]  w_c2_ddr_addr;
   wire [127:0] w_c2_ddr_write_data;
   wire [15:0]  w_c2_ddr_wdf_mask;
   wire [127:0] w_c2_ddr_read_data;
   wire         w_c2_ddr_ready, w_c2_ddr_grant;
   // LiteEth OWNER mux wires (core-2 eth master + owner flag).
   wire         w_c2_eth_owner;
   wire         w_c2e_write_DV, w_c2e_read_DV;
   wire [31:0]  w_c2e_addr;
   wire [63:0]  w_c2e_write_data;
   wire [7:0]   w_c2e_byte_en;
   wire [63:0]  w_c2e_read_data;
   wire         w_c2e_ready;

   mmio_if c2_bus();
   assign c2_bus.write_DV   = w_c2_write_DV;
   assign c2_bus.read_DV    = w_c2_read_DV;
   assign c2_bus.addr       = w_mmio_addr;
   assign c2_bus.write_data = w_mmio_write_data;
   assign c2_bus.byte_en    = w_mmio_byte_en;
   (* KEEP_HIERARCHY = "yes" *)
   core2_subsys core2_subsys_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .mmio(c2_bus),
       .o_ddr_req(w_c2_ddr_req),
       .o_ddr_done(w_c2_ddr_done),
       .o_ddr_write_DV(w_c2_ddr_write_DV),
       .o_ddr_read_DV(w_c2_ddr_read_DV),
       .o_ddr_addr(w_c2_ddr_addr),
       .o_ddr_write_data(w_c2_ddr_write_data),
       .o_ddr_wdf_mask(w_c2_ddr_wdf_mask),
       .i_ddr_read_data(w_c2_ddr_read_data),
       .i_ddr_ready(w_c2_ddr_ready),
       .i_ddr_grant(w_c2_ddr_grant),
       .o_eth_owner(w_c2_eth_owner),
       .o_eth_write_DV(w_c2e_write_DV),
       .o_eth_read_DV(w_c2e_read_DV),
       .o_eth_addr(w_c2e_addr),
       .o_eth_write_data(w_c2e_write_data),
       .o_eth_byte_en(w_c2e_byte_en),
       .i_eth_read_data(w_c2e_read_data),
       .i_eth_ready(w_c2e_ready),
       .i_clock_ms(r_clock_ms)
   );
   assign w_c2_read_data = c2_bus.read_data;

   // -------------------------------------------------------------------------
   // MMIO read mux — combinational decode into r_mmio_read_data_comb.
   // The result is registered into r_mmio_read_data (FF) below to break the
   // long combinational path from peripheral state regs through bus_splitter
   // and into the UART helpers' FSMs (which gate the main CPU st.SM/st.PC CE).
   // Reads therefore take 2 cycles: address valid in cycle N → comb decode in
   // cycle N → registered into r_mmio_read_data at edge N+1 → ready pulses in
   // cycle N+1 so the CPU FSM samples it.
   // Returns zero for undefined offsets (treat as scratch / write-only).
   // See MMIO_MAP.md for the full memory map.
   // -------------------------------------------------------------------------

   /* Internal LED register.  The top-level `o_led` port was converted from
      `output logic` to `output wire` so we could once tap an ETH_TXEN
      diagnostic into bit 0.  The diagnostic was removed (an ODDR driving
      a pin can't have its Q net read by fabric — REQP-1884), but the
      wire-port + internal reg + continuous assign pattern remains as a
      neutral pass-through.  Same behaviour as the original output logic. */
   assign o_led = st.led;

   always_comb begin
      r_mmio_read_data_comb = 64'h0;
      case (r_mmio_addr_q[27:16])
         12'h000: r_mmio_read_data_comb = w_sd_read_data;  // SD card
         12'h001: begin  // UART
            case (r_mmio_addr_q[15:0])
               16'h0008: r_mmio_read_data_comb = {56'b0, w_rx_fifo_byte};  // RX_DATA (peek; FIFO pops on read)
               // STATUS: bit0 = TX busy, bit1 = RX empty, bit2 = RX full
               16'h0010: r_mmio_read_data_comb = {61'b0, w_rx_fifo_full, w_rx_fifo_empty, w_sending_msg};
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h002: begin  // RGB LEDs
            case (r_mmio_addr_q[15:0])
               16'h0000: r_mmio_read_data_comb = {52'b0, st.RGB_LED_1};
               16'h0008: r_mmio_read_data_comb = {52'b0, st.RGB_LED_2};
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h003: begin  // 7-segment display (raw padded values)
            case (r_mmio_addr_q[15:0])
               16'h0000: r_mmio_read_data_comb = {32'b0, st.seven_seg_value2};
               16'h0008: r_mmio_read_data_comb = {32'b0, st.seven_seg_value1};
               16'h0010: r_mmio_read_data_comb = {st.seven_seg_value1, st.seven_seg_value2};
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h004: begin  // LEDs (RW) and switches (RO)
            case (r_mmio_addr_q[15:0])
               16'h0000: r_mmio_read_data_comb = {48'b0, st.led};
               16'h0008: r_mmio_read_data_comb = {48'b0, i_switch};
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h005: begin  // Cache controller — counters and config (RO)
            case (r_mmio_addr_q[15:0])
               // 0x0000 CACHE_CTRL is self-clearing — reads as 0
               16'h0000: r_mmio_read_data_comb = 64'h0;
               16'h0008: r_mmio_read_data_comb = w_cache_info;
               16'h0010: r_mmio_read_data_comb = {63'b0, w_cache_mnt_busy}; // CACHE_STATUS
               16'h0040: r_mmio_read_data_comb = w_cnt_read_hits;
               16'h0048: r_mmio_read_data_comb = w_cnt_read_misses;
               16'h0050: r_mmio_read_data_comb = w_cnt_write_hits;
               16'h0058: r_mmio_read_data_comb = w_cnt_write_misses;
               16'h0060: r_mmio_read_data_comb = w_cnt_writebacks;
               16'h0068: r_mmio_read_data_comb = w_cnt_stall_cycles;
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h00A: r_mmio_read_data_comb = w_aes_read_data;  // Crypto: AES
         12'h00B: r_mmio_read_data_comb = w_sha_read_data;  // Crypto: SHA-256
         12'h00C: r_mmio_read_data_comb = w_trng_read_data; // Crypto: TRNG
         12'h00E: r_mmio_read_data_comb = w_blit_read_data; // 2D DMA blitter
         12'h010: r_mmio_read_data_comb = w_c2_read_data;   // AMP core 2 mailbox
         12'h011: r_mmio_read_data_comb = w_vga_read_data;  // VGA output
         12'h00F: begin  // Interrupt controller / timer
            case (r_mmio_addr_q[15:0])
               16'h0000: r_mmio_read_data_comb = {60'b0, pip_int_mask};  // M5d: live mask is the pipeline's
               16'h0008: r_mmio_read_data_comb = {60'b0, w_vga_irq, r_eth_irq, w_blit_irq, r_timer_interrupt}; // INT_PENDING: [0]=timer, [1]=blitter, [2]=eth, [3]=vga vsync
               16'h0010: r_mmio_read_data_comb = {32'b0, r_interrupt_table[0]};
               16'h0018: r_mmio_read_data_comb = {32'b0, r_interrupt_table[1]};
               16'h0020: r_mmio_read_data_comb = {32'b0, r_interrupt_table[2]};
               16'h0028: r_mmio_read_data_comb = {32'b0, r_interrupt_table[3]};
               16'h0030: r_mmio_read_data_comb = {32'b0, r_timer_period};
               16'h0038: r_mmio_read_data_comb = {32'b0, r_timer_interrupt_counter};
               16'h0040: r_mmio_read_data_comb = r_clock_ms;
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h00D: begin  // Performance counters (RO; PERF_CTRL self-clearing reads 0)
            case (r_mmio_addr_q[15:0])
               16'h0000: r_mmio_read_data_comb = 64'h0;            // PERF_CTRL
               16'h0008: r_mmio_read_data_comb = r_perf_cycles;
               16'h0010: r_mmio_read_data_comb = r_perf_instr;
               16'h0018: r_mmio_read_data_comb = 64'h0;            // FETCH_CYCLES (multicycle CPU; retired)
               16'h0020: r_mmio_read_data_comb = r_perf_exec_cycles;
               16'h0028: r_mmio_read_data_comb = 64'h0;            // MUL_CYCLES (multicycle CPU; retired)
               16'h0030: r_mmio_read_data_comb = 64'h0;            // DIV_CYCLES (multicycle CPU; retired)
               16'h0038: r_mmio_read_data_comb = 64'h0;            // INT_CYCLES (multicycle CPU; retired)
               16'h0040: r_mmio_read_data_comb = r_perf_idle_cycles;
               16'h0048: r_mmio_read_data_comb = 64'h0;            // MUL_OPS (multicycle CPU; retired)
               16'h0050: r_mmio_read_data_comb = 64'h0;            // DIV_OPS (multicycle CPU; retired)
               16'h0058: r_mmio_read_data_comb = r_perf_int_ops;
               16'h0060: r_mmio_read_data_comb = r_perf_cnt_alu;
               16'h0068: r_mmio_read_data_comb = r_perf_cnt_load;
               16'h0070: r_mmio_read_data_comb = r_perf_cnt_store;
               16'h0078: r_mmio_read_data_comb = r_perf_cnt_branch;
               16'h0080: r_mmio_read_data_comb = r_perf_cnt_branch_taken;
               16'h0088: r_mmio_read_data_comb = r_perf_cnt_jump;
               16'h0090: r_mmio_read_data_comb = r_perf_cnt_call;
               16'h0098: r_mmio_read_data_comb = r_perf_cnt_indirect;
               16'h00A0: r_mmio_read_data_comb = r_perf_cnt_other;
               16'h00A8: r_mmio_read_data_comb = 64'h0;            // FASTPATH (multicycle CPU; retired)
               // M6 pipeline hazard attribution (cycles unless noted)
               16'h00B0: r_mmio_read_data_comb = {16'b0, r_perf_stall_data};
               16'h00B8: r_mmio_read_data_comb = {16'b0, r_perf_stall_loaduse};
               16'h00C0: r_mmio_read_data_comb = {16'b0, r_perf_stall_flags};
               16'h00C8: r_mmio_read_data_comb = {16'b0, r_perf_stall_sp};
               16'h00D0: r_mmio_read_data_comb = {16'b0, r_perf_stall_muldiv};
               16'h00D8: r_mmio_read_data_comb = {16'b0, r_perf_branch_flush}; // events
               16'h00E0: r_mmio_read_data_comb = {16'b0, r_perf_if_miss};
               16'h00E8: r_mmio_read_data_comb = {16'b0, r_perf_mem_wait};
               // bus-wedge flight recorder (bit map at the r_wedge_* decl)
               16'h0100: r_mmio_read_data_comb = r_wedge_snap0;
               16'h0108: r_mmio_read_data_comb = r_wedge_snap1;
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         default: r_mmio_read_data_comb = 64'h0;
      endcase
   end

   // Pipeline FF on the MMIO read return path. The FF on r_mmio_read_data
   // breaks the path from peripheral RAMs/regs (notably the SD sector buffer)
   // through bus_splitter and into uart_send_msg/uart_rx, which gate the
   // main CPU st.SM/st.PC clock enables. r_mmio_read_dv_d generates the
   // 1-cycle-delayed ready pulse so the CPU samples r_mmio_read_data on the
   // cycle after the read strobe.
   always_ff @(posedge i_Clk) begin
      r_mmio_addr_q      <= w_mmio_addr;
      r_mmio_read_data   <= r_mmio_read_data_comb;
      r_mmio_read_dv_d   <= w_mmio_read_DV;
      r_mmio_read_dv_d2  <= r_mmio_read_dv_d;
      r_mmio_write_dv_d  <= w_mmio_write_DV;
      r_mmio_write_dv_d2 <= r_mmio_write_dv_d;
      r_uart_rx_rd_d     <= w_uart_rx_rd_sel;
   end

   // Declared here (ahead of its use in the .o_calib_done port below) so strict
   // SystemVerilog (xvlog -sv) doesn't implicitly net it at first use and then
   // flag a redeclaration; the boot-ROM block below consumes it.
   wire        w_calib_done;

   mem_read_write mem_read_write (
       .i_Clk_board(i_Clk_board),   // board oscillator -> clk_wiz -> MIG 200MHz ref
       .o_ui_clk(w_ui_clk),         // MIG ui_clk (100MHz) -> drives i_Clk for the whole CPU
       .ddr2_dq(ddr2_dq),
       .ddr2_dqs_n(ddr2_dqs_n),
       .ddr2_dqs_p(ddr2_dqs_p),
       // Outputs
       .ddr2_addr(ddr2_addr),
       .ddr2_ba(ddr2_ba),
       .ddr2_ras_n(ddr2_ras_n),
       .ddr2_cas_n(ddr2_cas_n),
       .ddr2_we_n(ddr2_we_n),
       .ddr2_ck_p(ddr2_ck_p),
       .ddr2_ck_n(ddr2_ck_n),
       .ddr2_cke(ddr2_cke),
       .ddr2_cs_n(ddr2_cs_n),
       .ddr2_dm(ddr2_dm),
       .ddr2_odt(ddr2_odt),

       .cpu(dram_mem),   // DRAM-side membus from bus_splitter

       // Cache performance counters and clear pulse.
       .i_stat_clear(w_cache_stat_clear),
       .o_cache_info(w_cache_info),
       .o_cnt_read_hits(w_cnt_read_hits),
       .o_cnt_read_misses(w_cnt_read_misses),
       .o_cnt_write_hits(w_cnt_write_hits),
       .o_cnt_write_misses(w_cnt_write_misses),
       .o_cnt_writebacks(w_cnt_writebacks),
       .o_cnt_stall_cycles(w_cnt_stall_cycles),

       // 50 MHz output for Ethernet PHY REF_CLK (drives liteeth_core and ODDR).
       .clk_50(clk_50),

       // DDR2 ready — gates the resident boot-ROM copy below.
       .o_calib_done(w_calib_done),

       // DMA master port — driven by the 2D blitter (device 0x00E).
       .i_dma_req(w_blit_dma_req),
       .i_dma_done(w_blit_dma_done),
       .i_dma_write_DV(w_blit_dma_write_DV),
       .i_dma_read_DV(w_blit_dma_read_DV),
       .i_dma_addr(w_blit_dma_addr),
       .i_dma_write_data(w_blit_dma_write_data),
       .i_dma_wdf_mask(w_blit_dma_wdf_mask),
       .o_dma_read_data(w_blit_dma_read_data),
       .o_dma_ready(w_blit_dma_ready),
       .o_dma_grant(w_blit_dma_grant),

       // DDR master C — AMP core 2's uncached window (lowest priority).
       .i_c2_req(w_c2_ddr_req),
       .i_c2_done(w_c2_ddr_done),
       .i_c2_write_DV(w_c2_ddr_write_DV),
       .i_c2_read_DV(w_c2_ddr_read_DV),
       .i_c2_addr(w_c2_ddr_addr),
       .i_c2_write_data(w_c2_ddr_write_data),
       .i_c2_wdf_mask(w_c2_ddr_wdf_mask),
       .o_c2_read_data(w_c2_ddr_read_data),
       .o_c2_ready(w_c2_ddr_ready),
       .o_c2_grant(w_c2_ddr_grant),

       // DDR master D — VGA scanout (read-only wide reads; first among DMA masters).
       .i_vga_req(w_vga_dma_req),
       .i_vga_done(w_vga_dma_done),
       .i_vga_read_DV(w_vga_dma_read_DV),
       .i_vga_addr(w_vga_dma_addr),
       .o_vga_read_data(w_vga_dma_read_data),
       .o_vga_ready(w_vga_dma_ready),
       .o_vga_grant(w_vga_dma_grant),

       // Cache-maintenance control (MMIO 0xF005) — flush/invalidate for DMA coherency.
       .i_flush_go(w_cache_flush_go),
       .i_inval_go(w_cache_inval_go),
       .o_mnt_busy(w_cache_mnt_busy)
   );

   // ==========================================================================
   // Resident boot ROM (see NETBOOT_PLAN.md).  Holds the netboot image
   // ($readmemh "netboot.mem", built as today and converted with
   // `klausscc --mem-out`).  At reset the NO_PROGRAM boot sub-FSM reads word 0
   // (heap_start = image byte length), copies the image into DDR from byte 0,
   // and hands off to LOAD_COMPLETE — so netboot runs exactly as a UART load,
   // with no UART bootstrap.  A UART break+'S' still preempts to reflash.
   // ==========================================================================
   // (w_calib_done declared above the mem_read_write instantiation — see note there)
   wire [63:0] w_boot_dword;        // boot_rom read data (1-cycle latency)
   logic  [14:0] r_boot_dw_addr;      // doubleword index into the ROM / DDR
   logic  [15:0] r_boot_len_dw;       // image length in doublewords (from word 0)
   logic  [ 2:0] r_boot_phase;        // sub-state within NO_PROGRAM:
                                    //   0=wait DDR calib, 1=read word 0 (image length),
                                    //   2=ROM read latency, 3=issue DDR write, 4=ack write / advance
   logic         r_boot_active;       // 1 = boot copy pending/in-progress

   boot_rom #(
       .DEPTH_DW (32768),           // 256 KiB capacity — sized for netboot
       .ADDR_W   (15),
       .INIT_FILE("netboot.mem")
   ) boot_rom_i (
       .i_clk    (i_Clk),
       .i_dw_addr(r_boot_dw_addr),
       .o_dword  (w_boot_dword)
   );


   // ==========================================================================
   // Ethernet — eth_mmio_bridge (CPU MMIO ↔ Wishbone) + liteeth_core (MAC + PHY)
   // See ETHERNET_PLAN.md for the full address map and timing rationale.
   // CPU window 0xF006_0000..0xF008_FFFF, byte-for-byte translated to LiteEth.
   // ==========================================================================

   // CPU MMIO side (from bus_splitter Eth port; full 32-bit addr)
   mmio_if eth_bus();
   // AMP P3 — LiteEth OWNER mux.  Exactly one core drives the bridge:
   // core 1 (bus_splitter eth port) by default / at reset so the boot-ROM
   // netboot path is untouched; core 2 once software sets C2_ETH_OWNER.
   // The non-owner never reaches the bridge: core 1 gets a 1-cycle ack of
   // its own strobe (reads 0) so its bus never hangs; core 2's adapter
   // self-completes on its side (core2_subsys.sv).
   wire w_eth_own2 = w_c2_eth_owner;
   assign eth_bus.write_DV   = w_eth_own2 ? w_c2e_write_DV   : w_eth_write_DV;
   assign eth_bus.read_DV    = w_eth_own2 ? w_c2e_read_DV    : w_eth_read_DV;
   assign eth_bus.addr       = w_eth_own2 ? w_c2e_addr       : w_eth_addr;
   assign eth_bus.write_data = w_eth_own2 ? w_c2e_write_data : w_eth_write_data;
   assign eth_bus.byte_en    = w_eth_own2 ? w_c2e_byte_en    : w_eth_byte_en;
   logic r_eth_nown_dv_d;
   always_ff @(posedge i_Clk) begin
      r_eth_nown_dv_d <= (w_eth_write_DV | w_eth_read_DV) & w_eth_own2;
   end
   wire w_eth_nown_ack = (w_eth_write_DV | w_eth_read_DV) & w_eth_own2 & ~r_eth_nown_dv_d;
   eth_mmio_bridge eth_mmio_bridge_i (
       .i_clk             (i_Clk),
       .i_rst             (w_reset_H),      // synchronized (see r_rstn_sync)
       .mmio              (eth_bus),
       // LiteEth Wishbone master
       .o_wb_adr          (w_eth_wb_adr),
       .o_wb_dat_w        (w_eth_wb_dat_w),
       .o_wb_sel          (w_eth_wb_sel),
       .o_wb_we           (w_eth_wb_we),
       .o_wb_cyc          (w_eth_wb_cyc),
       .o_wb_stb          (w_eth_wb_stb),
       .o_wb_bte          (w_eth_wb_bte),
       .o_wb_cti          (w_eth_wb_cti),
       .i_wb_dat_r        (w_eth_wb_dat_r),
       .i_wb_ack          (w_eth_wb_ack),
       .i_wb_err          (w_eth_wb_err)
   );
   // Interrupt source 2: LiteEth's combined event, registered (same i_Clk
   // domain) and masked off while core 2 owns the MAC.
   always_ff @(posedge i_Clk) r_eth_irq <= w_eth_irq && !w_eth_own2;
   assign w_eth_read_data = w_eth_own2 ? 64'h0          : eth_bus.read_data;
   assign w_eth_ready     = w_eth_own2 ? w_eth_nown_ack : eth_bus.ready;
   assign w_c2e_read_data = eth_bus.read_data;
   assign w_c2e_ready     = w_eth_own2 ? eth_bus.ready  : 1'b0;

   liteeth_core liteeth_core_i (
       .sys_clock          (i_Clk),
       .sys_reset          (w_reset_H),     // synchronized (see r_rstn_sync)
       .interrupt          (w_eth_irq),

       // RMII connection to PHY pins
       .rmii_clocks_ref_clk(clk_50),
       .rmii_crs_dv        (ETH_CRSDV),
       .rmii_mdc           (ETH_MDC),
       .rmii_mdio          (ETH_MDIO),
       .rmii_rst_n         (ETH_RSTN),
       .rmii_rx_data       (ETH_RXD),
       .rmii_tx_data       (ETH_TXD),
       .rmii_tx_en         (ETH_TXEN),

       // Wishbone slave (driven by eth_mmio_bridge)
       .wishbone_adr       (w_eth_wb_adr),
       .wishbone_dat_w     (w_eth_wb_dat_w),
       .wishbone_dat_r     (w_eth_wb_dat_r),
       .wishbone_sel       (w_eth_wb_sel),
       .wishbone_we        (w_eth_wb_we),
       .wishbone_cyc       (w_eth_wb_cyc),
       .wishbone_stb       (w_eth_wb_stb),
       .wishbone_bte       (w_eth_wb_bte),
       .wishbone_cti       (w_eth_wb_cti),
       .wishbone_ack       (w_eth_wb_ack),
       .wishbone_err       (w_eth_wb_err)
   );

   // ETH_REFCLK (output to PHY) — clock-mode output via ODDR is the proper
   // way to forward an internal clock to a pin on 7-series.  Driving it from
   // a logic register would not meet the PHY's clock-input spec.
   //
   // D1/D2 are swapped (D1=0, D2=1) to invert the output relative to clk_50.
   // At 50 MHz this is equivalent to a 10 ns / 180° phase shift, which
   // centres the PHY's REFCLK rising edge in the middle of the data-valid
   // window at the PHY pin (data is launched from clk_50's rising edge and
   // propagates through ODDR+OBUF to the pin; without this shift the PHY's
   // sample point lands ~0.6 ns into the new data — below LAN8720A's 4 ns
   // tSU and causing transmitted frames to be rejected silently, with the
   // Pi seeing zero RX bytes despite our MAC reporting TX complete).
   ODDR #(
       .DDR_CLK_EDGE("OPPOSITE_EDGE"),
       .INIT(1'b0),
       .SRTYPE("SYNC")
   ) ODDR_eth_refclk (
       .Q (ETH_REFCLK),
       .C (clk_50),
       .CE(1'b1),
       .D1(1'b0),
       .D2(1'b1),
       .R (1'b0),
       .S (1'b0)
   );

   // ETH_RXERR is on the connector but unused by LiteEth's RMII PHY — input
   // declared at the top with a PACKAGE_PIN constraint, intentionally left
   // unread.  Vivado will warn; that's expected.


   uart_send_msg uart_send_msg1 (
       .i_Clk(i_Clk),
       .i_msg_flat(r_msg),
       .i_msg_length(r_msg_length),
       .i_msg_send_DV(r_msg_send_DV),
       .o_Tx_Serial(w_uart_tx_serial),
       .o_msg_sent_DV(i_msg_sent_DV),
       .o_sending_msg(w_sending_msg)
   );


   uart_rx uart_rx1 (
       .i_Clock(i_Clk),
       .i_Rx_Serial(i_uart_rx),
       .o_Rx_DV(w_uart_rx_DV),
       .o_Rx_Byte(w_uart_rx_value),
       .o_Break(w_uart_break)
   );

   // Write to FIFO only when the byte is not consumed by the break/command
   // handler (!r_break_received) or the program loader (st.SM != LOADING_BYTE).
   uart_rx_fifo uart_rx_fifo1 (
       .i_Clk        (i_Clk),
       .i_Reset      (w_reset_H),
       .i_Write_En   (w_uart_rx_DV & !r_break_received & (st.SM != LOADING_BYTE)),
       .i_Write_Byte (w_uart_rx_value),
       .i_Read_En    (st.rx_fifo_read | w_uart_rx_pop),
       .o_Peek_Byte  (w_rx_fifo_byte),
       .o_Empty      (w_rx_fifo_empty),
       .o_Full       (w_rx_fifo_full),
       .o_Count      ()
   );


   Seven_seg_LED_Display_Controller Seven_seg_LED_Display_Controller1 (
       .i_sysclk(i_Clk),
       .i_reset(w_reset_H),
       .i_displayed_number1(st.seven_seg_value1),  // Number to display
       .i_displayed_number2(st.seven_seg_value2),  // Number to display
       .o_Anode_Activate(o_Anode_Activate),
       .o_LED_cathode(o_LED_cathode)
   );

   SPI_Master_With_Single_CS SPI_Master_With_Single_CS_inst (
       .i_Rst_L   (~w_reset_H),
       .i_Clk     (i_Clk),
       // TX (MOSI) Signals
       .i_TX_Count(o_TX_LCD_Count),  // # bytes per CS low
       .i_TX_Byte (o_TX_LCD_Byte),   // Byte to transmit on MOSI
       .i_TX_DV   (o_TX_LCD_DV),     // Data Valid Pulse with i_TX_Byte
       .o_TX_Ready(i_TX_LCD_Ready),  // Transmit Ready for next byte
       // RX (MISO) Signals
       .o_RX_Count(i_RX_LCD_Count),  // Index RX byte
       .o_RX_DV   (i_RX_LCD_DV),     // Data Valid pulse (1 clock cycle)
       .o_RX_Byte (i_RX_LCD_Byte),   // Byte received on MISO
       // SPI Interface
       .o_SPI_Clk (o_SPI_LCD_Clk),
       .i_SPI_MISO(i_SPI_LCD_MISO),
       .o_SPI_MOSI(o_SPI_LCD_MOSI),
       .o_SPI_CS_n(o_SPI_LCD_CS_n)
   );
   /*
rams_sp_nc rams_sp_nc1 (
               .i_clk(i_Clk),
               .i_opcode_read_addr(st.PC),
               .i_mem_read_addr(r_mem_read_addr),
               .o_dout_opcode(w_opcode),
               .o_dout_mem(w_mem),
               .o_dout_var1(w_var1),
               .o_dout_var2(w_var2),
               .i_write_addr(o_ram_write_addr),
               .i_write_value(o_ram_write_value),
               .i_write_en(o_ram_write_DV)
                );
 */
   integer i;
   initial begin
      st.flags.sign <= 0;
       for (i = 0; i < 16; i = i + 1)
       r_register[i] = 64'b0;
   end
   
   assign w_opcode = r_opcode_mem;

   assign w_var1   = r_var1_mem;
   assign w_var2   = r_var2_mem;
   

   // Stack module removed — stack now uses DDR2 RAM via st.SP register

   RGB_LED RGB_LED (
       .i_sysclk(i_Clk),
       .LED1(st.RGB_LED_1),
       .LED2(st.RGB_LED_2),
       .o_LED_RGB_1(o_LED_RGB_1),
       .o_LED_RGB_2(o_LED_RGB_2)
   );


   /*ila_0  myila(.clk(i_Clk),
             .probe0(w_opcode),
             .probe1(0),
             .probe2(st.PC),
             .probe3(st.SM),
             .probe4(r_var1_mem),
             .probe5(0),
             .probe6(0),
             .probe7(0),
             .probe8(st.mem_read_DV),
             .probe9(st.mem_addr),
             .probe10(w_mem_ready),
             .probe11(w_var1),
             .probe12(w_mem_read_data),
             .probe13(w_temp_cache_hit),
             .probe14(w_temp_cache_value),
             .probe15(0)

            ); */

   // All opcode-handler tasks + the t_opcode_select dispatcher were converted to
   // next-state functions (f_*) or retired (UART->MMIO); only uart_tasks.vh
   // remains, holding live FSM-state helpers (t_tx_message/t_debug_message loader
   // & debug messages) + the HCF crash-dump formatter (f_dump_*).
   `include "uart_tasks.vh"

   initial begin
      o_TX_LCD_Count = 4'd1;
      o_TX_LCD_Byte = 8'b0;
      st.SM = NO_PROGRAM;
      r_boot_active = 1'b1;
      r_boot_phase = 3'd0;
      r_boot_dw_addr = 15'd0;
      st.timeout_counter = 0;
      o_LCD_reset_n = 1'b0;
      st.PC = 32'h0;
      st.flags.zero = 0;
      st.flags.carry = 0;
      st.flags.overflow = 0;
      st.error_code = 8'h0;
      st.timeout_counter = 32'b0;
      st.seven_seg_value1 = 32'h20_10_00_07;
      st.seven_seg_value2 = 32'h21_21_21_21;
      st.led <= 16'h0;
      o_ram_write_addr = 32'h0;
      r_ram_next_write_addr = 32'h0;
      st.SP = 32'h800_0000;          // empty-descending stack, top of 128 MiB byte space
      r_msg_send_DV <= 1'b0;
      r_hcf_message_sent <= 1'b0;
      st.RGB_LED_1 = 12'h000;
      st.RGB_LED_2 = 12'h000;
      st.timing_start <= 0;
      r_timer_interrupt_counter <= 0;
      r_timer_interrupt_counter_sec <= 0;
      st.int_mask <= 4'h0;            // all sources masked at power-up
      r_timer_period <= 32'h000F_FFFF;  // default ~10.5 ms @ 100 MHz
      r_clock_ms <= 64'h0;
      r_clock_ms_div <= 17'h0;
      st.mem_write_DV <= 0;
      st.mem_read_DV <= 0;
      st.mem_byte_en <= 8'hFF;
      r_msg = 256'b0;
      r_boot_flash = 0;
      r_break_received = 0;
      st.wb.set_zero = 0;
      st.wb.pending        = 0;
      st.rx_fifo_read  = 0;
      r_break_active  = 0;
      r_break_counter = 0;
      r_pip_start = 1'b0;
      r_pip_start_pc = 32'h20;
      r_trace_idx = 4'h0;
      r_trace_full = 1'b0;
      r_instr_count = 32'h0;
      r_perf_cycles = 64'd0;       r_perf_instr = 64'd0;
      r_perf_exec_cycles = 64'd0;  r_perf_idle_cycles = 64'd0;
      r_perf_int_ops = 64'd0;
      r_perf_cnt_alu = 64'd0;      r_perf_cnt_load = 64'd0;
      r_perf_cnt_store = 64'd0;    r_perf_cnt_branch = 64'd0;
      r_perf_cnt_branch_taken = 64'd0; r_perf_cnt_jump = 64'd0;
      r_perf_cnt_call = 64'd0;     r_perf_cnt_indirect = 64'd0;
      r_perf_cnt_other = 64'd0;
      r_hcf_dump_phase = 7'd0;
      r_hcf_dump_sub = 3'b000;
      r_hcf_dump_byte_pos = 5'd0;
      r_hcf_stack_loaded = 1'b0;
      r_hcf_stack_data = 64'b0;
      for (i = 0; i < 16; i = i + 1)
         r_trace_buf[i] = 64'b0;
   end

   always_ff @(posedge i_Clk) begin
      if (w_reset_H) begin

         st.SM <= NO_PROGRAM;
         st.SP <= 32'h800_0000;
         r_boot_active   <= 1'b1;   // arm the resident boot-ROM copy
         r_boot_phase    <= 3'd0;
         r_boot_dw_addr  <= 15'd0;
         st.wb.pending    <= 1'b0;
         r_break_received <= 1'b0;
         for (i = 0; i < 16; i = i + 1)
            r_register[i] <= 64'b0;
         r_trace_idx <= 4'h0;
         r_trace_full <= 1'b0;
         r_instr_count <= 32'h0;
         r_hcf_dump_phase <= 7'd0;
         r_hcf_dump_sub <= 3'b000;
         r_hcf_dump_byte_pos <= 5'd0;
         r_hcf_stack_loaded <= 1'b0;
         for (i = 0; i < 16; i = i + 1)
            r_trace_buf[i] <= 64'b0;

      end // if (w_reset_H)
      // Break received: arm the flag so next byte is treated as a command
      else if (w_uart_break) begin
         r_break_received <= 1'b1;
         // The pipeline's 1-cycle dispatch ack must be consumed even on a
         // branch that skips the PIPE_RUN arm, or the timer source stays
         // pending and the ISR re-enters straight after its IRET.
         if (pip_irq_ack && pip_irq_ack_sel == 2'd0)
            r_timer_interrupt <= 1'b0;
      end
      // Command characters are only accepted after a break
      else if (w_uart_rx_DV & r_break_received) begin
         r_break_received <= 1'b0;  // consume the break — one command per break
         if (pip_irq_ack && pip_irq_ack_sel == 2'd0)
            r_timer_interrupt <= 1'b0;   // same ack-consume as the break branch
         case (w_uart_rx_value)
            8'h53: begin // 'S' — load start
               st.SM <= LOADING_BYTE;
               r_load_byte_counter <= 0;
               o_ram_write_addr <= 32'h0;
               r_ram_next_write_addr <= 32'h0;
               r_checksum <= 16'h0;
               r_old_checksum <= 16'h0;
               st.RGB_LED_1 <= 12'h0;
               st.RGB_LED_2 <= 12'h0;
               st.led <= 16'h0;
               st.mem_write_DV <= 1'b0;
               st.mem_read_DV <= 1'b0;
            end
            // Any other byte after break: silently ignored
         endcase
      end else begin
         r_msg_send_DV  <= 1'b0;
         st.rx_fifo_read <= 1'b0;

         if (r_timer_interrupt_counter > r_timer_period) begin
            r_timer_interrupt_counter <= 0;
            r_timer_interrupt <= 1;
         end else begin
            r_timer_interrupt_counter <= r_timer_interrupt_counter + 1;
         end

         if (r_timer_interrupt_counter_sec > 100_000_000) begin
            r_timer_interrupt_counter_sec <= 0;
         end else begin
            r_timer_interrupt_counter_sec <= r_timer_interrupt_counter_sec + 1;
         end

         // NOTE: the MMIO write handler now runs AFTER this case(st.SM) — see the
         // block just past the endcase. It was moved there so a converted store's
         // whole-struct `st <= f_x(st)` NBA in OPCODE_EXECUTE cannot clobber the
         // peripheral st fields (esp. st.int_mask) it writes on the same cycle.

         case (st.SM)
            NO_PROGRAM: begin
               st.seven_seg_value1 <= 32'h22222222;
               st.seven_seg_value2 <= 32'h22222222;

               if (r_boot_active) begin
                  // ---- Resident boot-ROM → DDR copy (sub-FSM in r_boot_phase) ----
                  // Marker on the upper display so a stuck copy is distinguishable
                  // from a blank idle board ("b00t").
                  st.seven_seg_value1 <= 32'h0B000704;
                  case (r_boot_phase)
                     3'd0: begin
                        // Wait for DDR calibration, then present word 0.
                        if (w_calib_done) begin
                           r_boot_dw_addr <= 15'd0;
                           r_boot_phase   <= 3'd1;
                        end
                     end
                     3'd1: begin
                        // word 0 valid: heap_start (lo32) = image byte length.
                        // >>3 → doubleword count.  Zero ⇒ no valid image: idle.
                        if (w_boot_dword[18:3] == 16'd0) begin
                           r_boot_active <= 1'b0;   // unprogrammed ROM → normal idle
                        end else begin
                           r_boot_len_dw  <= w_boot_dword[18:3];
                           r_boot_dw_addr <= 15'd0; // restart at dword 0 to copy it
                           r_boot_phase   <= 3'd2;
                        end
                     end
                     3'd2: begin
                        // ROM read latency: o_dword settles for r_boot_dw_addr.
                        r_boot_phase <= 3'd3;
                     end
                     3'd3: begin
                        // Issue the full 64-bit DDR write of this doubleword.
                        st.mem_addr       <= {14'd0, r_boot_dw_addr, 3'b0};
                        st.mem_write_data <= w_boot_dword;
                        st.mem_byte_en    <= 8'hFF;
                        st.mem_write_DV   <= 1'b1;
                        r_boot_phase     <= 3'd4;
                     end
                     3'd4: begin
                        if (w_mem_ready) begin
                           st.mem_write_DV <= 1'b0;
                           if (r_boot_dw_addr == r_boot_len_dw - 1'b1) begin
                              // Hand off to the normal run path.  Equal checksums
                              // bypass LOAD_COMPLETE's verify; entry = 0x20.
                              r_boot_active   <= 1'b0;
                              r_PC_requested  <= 32'h0000_0020;
                              r_calc_checksum <= 16'h0;
                              r_rec_checksum  <= 16'h0;
                              st.SP            <= 32'h800_0000;
                              st.SM            <= LOAD_COMPLETE;
                           end else begin
                              r_boot_dw_addr <= r_boot_dw_addr + 1'b1;
                              r_boot_phase   <= 3'd2;  // ROM latency before next
                           end
                        end
                     end
                     default: r_boot_phase <= 3'd0;
                  endcase
               end else if (r_timer_interrupt_counter_sec == 0) begin
                  // Idle heartbeat: no resident image present, waiting for a UART
                  // load — alternate the two RGB LEDs once per second.
                  case (r_boot_flash)
                     0: begin
                        st.RGB_LED_1  <= 12'h010;
                        st.RGB_LED_2  <= 12'h100;
                        //o_led[0]<=1;
                        r_boot_flash <= 1;
                     end
                     default: begin
                        st.RGB_LED_1  <= 12'h100;
                        st.RGB_LED_2  <= 12'h010;
                        //o_led[0]<=0;
                        r_boot_flash <= 0;
                     end
                  endcase
               end
            end

            LOADING_BYTE: begin

               if (w_mem_ready) begin
                  st.mem_write_DV <= 1'b0;
               end
               st.SP <= 32'h800_0000;  // reset stack pointer during program load

               st.seven_seg_value1 <= {
                  8'h24,
                  4'h0,
                  r_ram_next_write_addr[27:24],
                  4'h0,
                  r_ram_next_write_addr[23:20],
                  4'h0,
                  r_ram_next_write_addr[19:16]
               };
               st.seven_seg_value2 <= {
                  4'h0,
                  r_ram_next_write_addr[15:12],
                  4'h0,
                  r_ram_next_write_addr[11:8],
                  4'h0,
                  r_ram_next_write_addr[7:4],
                  4'h0,
                  r_ram_next_write_addr[3:0]
               };

               if (w_uart_rx_DV) begin


                  case (w_uart_rx_value)
                     8'h58: // End char X
                        begin
                        if (r_load_byte_counter == 0) begin
                           st.SM <= LOAD_COMPLETE;
                           r_calc_checksum<=r_old_checksum+o_ram_write_addr[17:2]*2+o_ram_write_value[31:16]; //adding number of words to checksum (addr>>2=word count)
                           r_rec_checksum <= o_ram_write_value[15:0];
                           o_ram_write_value <= 32'h0;

                        end // (r_load_byte_counter==0)
                            else
                            begin
                           st.SM <= HCF_1;  // Halt and catch fire error
                           st.error_code <= ERR_DATA_LOAD;
                        end  // else (r_load_byte_counter==7)
                     end  // case 8'h58
                     8'h5A: // Start data flag Z
                        begin
                        r_PC_requested <= o_ram_write_value[31:0];
                     end
                     8'h0a: ;  // ignore LF
                     8'h0d: ;  // ignore CR
                     default: begin
                        // Pack hex pairs into o_ram_write_value in little-endian byte order:
                        // first hex pair (stream byte 0) → bits[7:0], last (byte 3) → bits[31:24].
                        // This makes mem_byte[N] land at bits[8(N%4)+7:8(N%4)] of the 32-bit
                        // word, matching the CPU's LE byte-lane mapping in MEMGET8 / STIDX8 etc.
                        case (r_load_byte_counter)
                           0: o_ram_write_value[7:4]   = return_hex_from_ascii(w_uart_rx_value);
                           1: o_ram_write_value[3:0]   = return_hex_from_ascii(w_uart_rx_value);
                           2: o_ram_write_value[15:12] = return_hex_from_ascii(w_uart_rx_value);
                           3: o_ram_write_value[11:8]  = return_hex_from_ascii(w_uart_rx_value);
                           4: o_ram_write_value[23:20] = return_hex_from_ascii(w_uart_rx_value);
                           5: o_ram_write_value[19:16] = return_hex_from_ascii(w_uart_rx_value);
                           6: o_ram_write_value[31:28] = return_hex_from_ascii(w_uart_rx_value);
                           7: o_ram_write_value[27:24] = return_hex_from_ascii(w_uart_rx_value);
                           default: ;
                        endcase  //r_load_byte_counter
                        if (r_load_byte_counter == 7) begin
                           r_load_byte_counter <= 0;
                           case (st.RGB_LED_1)
                              12'h050: st.RGB_LED_1 <= 12'h005;
                              default: st.RGB_LED_1 <= 12'h050;
                           endcase
                           o_ram_write_addr <= r_ram_next_write_addr;
                           r_ram_next_write_addr <= r_ram_next_write_addr + 4;  // byte addr: 4 bytes per word
                           if (r_ram_next_write_addr>32'h7FF_FFFC) // Nexys has 128 MiB DDR2, last valid word at byte addr 0x7FF_FFFC
                                begin
                              st.SM <= HCF_1;  // Halt and catch fire error
                              st.error_code <= ERR_OVERFLOW;
                           end
                           st.mem_addr <= r_ram_next_write_addr;
                           // Place 32-bit word in the correct half of the 64-bit doubleword.
                           // Little-endian layout: addr[2]==0 → LOW half [31:0]; addr[2]==1 → HIGH half [63:32].
                           if (r_ram_next_write_addr[2] == 1'b0) begin
                              st.mem_write_data <= {32'b0, o_ram_write_value};
                              st.mem_byte_en    <= 8'h0F;
                           end else begin
                              st.mem_write_data <= {o_ram_write_value, 32'b0};
                              st.mem_byte_en    <= 8'hF0;
                           end
                           st.mem_write_DV <= 1'b1;

                           r_old_checksum <= r_checksum;
                           r_checksum <= r_checksum + o_ram_write_value[31:16] + o_ram_write_value[15:0];
                        end // if (r_load_byte_counter==7)
                            else
                            begin
                           r_load_byte_counter <= r_load_byte_counter + 1;
                        end  // else if (r_load_byte_counter==7)
                     end  // case default
                  endcase  // w_uart_rx_value
               end
            end

            LOAD_COMPLETE: begin
               st.seven_seg_value1 <= 32'h22222222;  // Blank 7 seg
               if (r_calc_checksum==r_rec_checksum) // Last value received should be checksum
                begin  // Reset all flags and jump to first instruction
                  o_LCD_reset_n <= 1'b0;
                  st.led <= 16'h0;
                  o_ram_write_addr <= 32'h0;
                  o_TX_LCD_Byte <= 8'b0;
                  o_TX_LCD_Count <= 4'd1;
                  st.flags.carry <= 1'b0;
                  st.error_code <= 8'h0;
                  r_hcf_message_sent <= 1'b0;
                  r_interrupt_table[0] <= 32'h0;  // clear all 4 handler vectors;
                  r_interrupt_table[1] <= 32'h0;  // a 0 vector disables that source
                  r_interrupt_table[2] <= 32'h0;
                  r_interrupt_table[3] <= 32'h0;
                  r_msg_send_DV <= 1'b0;
                  st.flags.overflow <= 1'b0;
                  st.PC <= r_PC_requested;
                  st.mem_byte_en <= 8'hFF;  // restore full-doubleword default after loader partial writes
                  r_ram_next_write_addr <= 32'h0;
                  st.RGB_LED_1 <= 12'h000;
                  st.RGB_LED_2 <= 12'h000;
                  st.seven_seg_value1 <= 32'h22_22_22_22;
                  st.seven_seg_value2 <= 32'h22_22_22_22;
                  st.SM <= START_WAIT;
                  st.timeout_counter <= 0;
                  r_timer_interrupt <= 0;
                  r_timer_interrupt_counter <= 0;
                  st.int_mask <= 4'h0;            // all sources masked until program enables
                  r_timer_period <= 32'h000F_FFFF;  // default ~10.5 ms @ 100 MHz
                  r_instr_count <= 32'h0;        // reset committed-instruction counter for the new run
                  st.timing_start <= 0;
                  st.flags.zero <= 0;
                  t_tx_message(8'd1);  // Load OK message
               end else begin
                  st.SM <= HCF_1;  // Halt and catch fire error
                  st.error_code <= ERR_CHECKSUM_LOAD;
                  t_tx_message(8'd2);  // Load error message
               end
            end

            // Delay to enable load message to be sent before starting
            START_WAIT: begin
               r_msg_send_DV <= 1'b0;
               if (r_start_wait_counter == 0) begin
                  // M5d: execution belongs to the pipeline core.
                  r_pip_start    <= 1'b1;
                  r_pip_start_pc <= st.PC;
                  st.SM <= PIPE_RUN;
                  st.seven_seg_value1 <= 32'h22_22_22_22;
                  st.seven_seg_value2 <= 32'h22_22_22_22;
               end else begin
                  r_start_wait_counter <= r_start_wait_counter - 1;
                  st.seven_seg_value1 <= 32'h21_21_21_21;
                  st.seven_seg_value2 <= 32'h21_21_21_21;
               end
            end

            // ─────────────────────────────────────────────────────────────
            // PIPE_RUN — the 5-stage pipeline_core owns execution and the
            // memory port. This arm only mirrors retire bookkeeping (crash
            // trace ring, instruction count), consumes IRQ dispatch acks,
            // relays the LCD side-port, and handles the drained park
            // (HALT / TRAP / illegal) by snapshotting the pipeline's
            // architectural state into the FSM's copies so the existing
            // HALTED_BREAK / HCF crash-dump machinery works unchanged.
            // ─────────────────────────────────────────────────────────────
            PIPE_RUN: begin
               r_pip_start   <= 1'b0;
               r_msg_send_DV <= 1'b0;   // MMIO UART TX (after-case) re-arms it
               if (pip_ret_valid) begin
                  r_trace_buf[r_trace_idx] <= {pip_ret_pc, pip_ret_op};
                  r_trace_idx              <= r_trace_idx + 4'd1;
                  if (r_trace_idx == 4'd15) r_trace_full <= 1'b1;
                  r_instr_count <= r_instr_count + 32'd1;
               end
               if (pip_irq_ack && pip_irq_ack_sel == 2'd0)
                  r_timer_interrupt <= 1'b0;   // dispatch consumed the timer
               if (pip_lcd_dv) begin
                  o_TX_LCD_Byte <= pip_lcd_byte;
                  o_LCD_DC      <= pip_lcd_dc;
                  o_TX_LCD_DV   <= 1'b1;
               end else begin
                  o_TX_LCD_DV   <= 1'b0;
               end
               if (pip_lcd_rst_wr) o_LCD_reset_n <= pip_lcd_rst_n;
               if (pip_parked && pip_bus_idle) begin
                  for (i = 0; i < 16; i = i + 1) r_register[i] <= pip_dbg_r[i];
                  st.PC    <= pip_park_pc;
                  st.SP    <= pip_dbg_sp;
                  st.flags <= pip_dbg_flags;
                  // The crash dump's OPC= line reads w_opcode = r_opcode_mem,
                  // which only the (dormant) FSM fetch paths write — without
                  // this copy a pipeline trap dumps a stale FSM-era opcode.
                  r_opcode_mem <= pip_park_op;
                  case (pip_park_kind)
                     3'd0: st.SM <= HALTED_BREAK;                                  // HALT
                     3'd1: begin st.SM <= HCF_1; st.error_code <= ERR_TRAP; end    // TRAP
                     3'd2: begin st.SM <= HCF_1; st.error_code <= ERR_INV_OPCODE; end
                     default: ;   // WAIT parks wake inside the pipeline
                  endcase
               end
            end

            HCF_1: begin
               // First entry only: kick the crash-dump UART emitter.  HCF_4 loops
               // back here periodically to drive the 7-seg, but we don't want to
               // re-spam the dump each loop, so r_hcf_message_sent gates it.
               if (!r_hcf_message_sent) begin
                  r_hcf_message_sent <= 1'b1;
                  r_hcf_dump_phase   <= 7'd0;
                  r_hcf_dump_sub     <= 3'b000;
                  r_hcf_stack_loaded <= 1'b0;
                  r_break_counter    <= 12'd0;  // clean start for the post-dump UART break
                  st.SM               <= HCF_DUMP;
               end else begin
                  st.timeout_counter <= 0;
                  st.SM              <= HCF_2;
               end
            end

            // Crash dump: walk r_hcf_dump_phase through the dump-line sequence,
            // emitting each line over UART using the canonical 4-step handshake
            // (PREP → ACK → DONE_WAIT).  Stack phases insert an extra
            // STACK_FETCH state to read the doubleword from DDR2 first.
            HCF_DUMP: begin
               case (r_hcf_dump_sub)
                  // PREP — fill r_msg for the current phase, pulse DV.
                  // For stack phases, kick a DDR2 read first; the response
                  // goes through STACK_FETCH and re-enters PREP with
                  // r_hcf_stack_loaded=1 so the line emit can proceed.
                  3'b000: begin
                     if (!w_sending_msg) begin
                        if ((r_hcf_dump_phase >= DUMP_STACK_BASE)
                         && (r_hcf_dump_phase <  DUMP_STACK_BASE + 7'd4)
                         && !r_hcf_stack_loaded) begin
                           // Skip DDR2 reads past the top of the stack region
                           // (st.SP+offset >= STACK_TOP).  Substitute an FFs
                           // sentinel and mark loaded so the next PREP emits
                           // the line directly.  Prevents OOB DDR2 access when
                           // the stack is empty (st.SP at initial 0x0800_0000).
                           if ((st.SP + ({25'b0, r_hcf_dump_phase - DUMP_STACK_BASE} << 3))
                                 >= STACK_TOP) begin
                              r_hcf_stack_data   <= 64'hFFFF_FFFF_FFFF_FFFF;
                              r_hcf_stack_loaded <= 1'b1;
                           end else begin
                              st.mem_addr     <= st.SP +
                                 ({25'b0, r_hcf_dump_phase - DUMP_STACK_BASE} << 3);
                              st.mem_read_DV  <= 1'b1;
                              r_hcf_dump_sub <= 3'b011;
                           end
                        end else if (r_hcf_dump_phase == DUMP_V1H && !r_hcf_stack_loaded) begin
                           // Read DRAM at PC+8 to recover the hi32 of a 64-bit
                           // immediate (V64 encoding: lo32 at PC+4, hi32 at PC+8).
                           st.mem_addr     <= st.PC + 32'd8;
                           st.mem_read_DV  <= 1'b1;
                           r_hcf_dump_sub <= 3'b011;
                        end else if (r_hcf_dump_phase == DUMP_OPCM && !r_hcf_stack_loaded) begin
                           // Re-read DRAM at PC.  If this disagrees with OPC,
                           // the opcode cache is incoherent with DRAM.
                           st.mem_addr     <= st.PC;
                           st.mem_read_DV  <= 1'b1;
                           r_hcf_dump_sub <= 3'b011;
                        end else if (r_hcf_dump_phase >= DUMP_REG_BASE
                                  && r_hcf_dump_phase <  DUMP_STACK_BASE
                                  && !r_hcf_stack_loaded) begin
                           // Register phase pre-fetch (R0..RF).  Same trick as
                           // the trace branch below: f_dump_byte's register
                           // branch used to read r_register[k] with a runtime
                           // index, inferring a 16:1 64-bit mux hung off the
                           // live register file and dragging all 16 regs into
                           // the dump corner — the routing-congestion source
                           // behind the crash-dump rip-up/reroute.  Latch the
                           // selected register here one cycle before BYTE_BUILD
                           // so f_dump_byte reads the flat r_hcf_stack_data
                           // latch instead (see uart_tasks.vh).
                           r_hcf_stack_data   <= r_register[r_hcf_dump_phase[3:0]
                                                 - DUMP_REG_BASE[3:0]];
                           r_hcf_stack_loaded <= 1'b1;
                        end else if (r_hcf_dump_phase >= DUMP_TRACE_BASE && !r_hcf_stack_loaded) begin
                           // Trace phase pre-fetch.  Lift the r_trace_buf read out
                           // of f_dump_byte's combinational path: without this,
                           // the path r_trace_idx → trace_pos → r_trace_buf[pos]
                           // → return_ascii_from_hex → r_msg byte write was the
                           // failing endpoint at WNS −0.020 ns post-route.
                           // Latching the trace entry one cycle before BYTE_BUILD
                           // removes the BRAM read from the byte-mux and lets
                           // f_dump_byte's trace branch read from a flat register.
                           r_hcf_stack_data   <= r_trace_buf[r_trace_idx - 4'd1
                                                 - (r_hcf_dump_phase[3:0] - DUMP_TRACE_BASE[3:0])];
                           r_hcf_stack_loaded <= 1'b1;
                           // Stay in PREP — next cycle the final else branch
                           // launches the byte build with the latched data.
                        end else begin
                           // Snapshot the line length and start the byte-by-byte
                           // build.  BYTE_BUILD writes r_msg one byte per cycle
                           // and pulses send_DV on the last byte.
                           r_msg_length        <= f_dump_length(r_hcf_dump_phase);
                           r_hcf_dump_byte_pos <= 5'd0;
                           r_hcf_dump_sub      <= 3'b101;
                        end
                     end
                  end

                  // ACK — wait for the UART to latch our DV / start sending.
                  // Once w_sending_msg goes high we know the bytes have been
                  // captured; the default top-of-case clears DV automatically.
                  3'b001: begin
                     if (w_sending_msg) begin
                        r_hcf_dump_sub <= 3'b010;
                     end
                  end

                  // DONE_WAIT — wait for the UART completion pulse, then
                  // advance to the next phase, or fall through to HCF_2 once
                  // the footer line has been sent.
                  3'b010: begin
                     if (i_msg_sent_DV) begin
                        r_hcf_dump_sub <= 3'b000;
                        // Leaving a pre-fetched phase invalidates the cached
                        // value so the next phase loads fresh data.  Stack/V1H/
                        // OPCM are DDR2-fetched; trace phases are pre-fetched
                        // from r_trace_buf (BRAM) for timing reasons (see PREP).
                        if (((r_hcf_dump_phase >= DUMP_REG_BASE)
                          && (r_hcf_dump_phase <  DUMP_STACK_BASE))
                         || ((r_hcf_dump_phase >= DUMP_STACK_BASE)
                          && (r_hcf_dump_phase <  DUMP_STACK_BASE + 7'd4))
                         || (r_hcf_dump_phase == DUMP_V1H)
                         || (r_hcf_dump_phase == DUMP_OPCM)
                         || (r_hcf_dump_phase >= DUMP_TRACE_BASE)) begin
                           r_hcf_stack_loaded <= 1'b0;
                        end
                        if (r_hcf_dump_phase == DUMP_FOOTER) begin
                           // After the footer line drains, drop into the BREAK
                           // sub-state to assert a UART break (line low for ~2.5
                           // frames) so a host parser sees an unambiguous
                           // end-of-dump marker — same pattern as HALTED_BREAK.
                           r_hcf_dump_phase <= 7'd0;
                           r_hcf_dump_sub   <= 3'b100;  // override default 000
                        end else begin
                           r_hcf_dump_phase <= r_hcf_dump_phase + 7'd1;
                        end
                     end
                  end

                  // STACK_FETCH — wait for DDR2 read to complete, latch the
                  // doubleword, then return to PREP which now sees
                  // r_hcf_stack_loaded=1 and emits the line.
                  3'b011: begin
                     if (w_mem_ready) begin
                        r_hcf_stack_data   <= w_mem_read_data;
                        r_hcf_stack_loaded <= 1'b1;
                        st.mem_read_DV      <= 1'b0;
                        r_hcf_dump_sub     <= 3'b000;
                     end
                  end

                  // BYTE_BUILD — write one byte of r_msg per cycle from
                  // f_dump_byte(phase, pos).  On the last byte we pulse
                  // r_msg_send_DV so uart_send_msg snapshots r_msg the next
                  // cycle (NBAs schedule the final byte and DV for the same
                  // edge).  Replaces the legacy 256-bit-wide single-cycle
                  // assignment that became a 207-input mux in synth.
                  3'b101: begin
                     r_msg[r_hcf_dump_byte_pos*8 +: 8] <= f_dump_byte(r_hcf_dump_phase, r_hcf_dump_byte_pos);
                     if (r_hcf_dump_byte_pos == r_msg_length[4:0] - 5'd1) begin
                        r_msg_send_DV  <= 1'b1;
                        r_hcf_dump_sub <= 3'b001;  // ACK
                     end else begin
                        r_hcf_dump_byte_pos <= r_hcf_dump_byte_pos + 5'd1;
                     end
                  end

                  // BREAK — wait for the footer's UART transmission to fully
                  // drain, then hold the TX line low for 2500 clocks (~7.5 frame
                  // times at CLKS_PER_BIT=33) as a UART break.  This mirrors
                  // HALTED_BREAK so a host parser can treat the break as the
                  // unambiguous end-of-dump marker, the same way it does for
                  // a clean HALT.  Once the break completes we move to the
                  // PRE_DISPLAY hold (3'b110) before HCF_2 takes over the 7-seg.
                  3'b100: begin
                     if (r_break_counter == 0) begin
                        if (!w_sending_msg) begin
                           r_break_active  <= 1'b1;
                           r_break_counter <= 12'd2500;
                        end
                     end else begin
                        r_break_counter <= r_break_counter - 1;
                        if (r_break_counter == 12'd1) begin
                           r_break_active    <= 1'b0;
                           st.timeout_counter <= 0;
                           r_hcf_dump_sub    <= 3'b110;
                        end
                     end
                  end

                  // PRE_DISPLAY — hold for 3 seconds (300M cycles @ 100 MHz)
                  // after the UART crash dump completes so the operator can
                  // read whatever was on the 7-seg at the moment of the fault
                  // before HCF_2 overwrites it with the error display.
                  3'b110: begin
                     if (st.timeout_counter >= 45'd300_000_000) begin
                        st.timeout_counter <= 0;
                        st.SM              <= HCF_2;
                     end else begin
                        st.timeout_counter <= st.timeout_counter + 1;
                     end
                  end

                  default: r_hcf_dump_sub <= 3'b000;
               endcase
            end

            HCF_2: begin
               st.seven_seg_value1[31:8] <= 24'h230C0F;
               st.seven_seg_value1[7:0] <= st.error_code;
               st.seven_seg_value2 <= 32'h22_22_22_22;
               st.timeout_max <= 32'd100_000_000;
               if (st.timeout_counter >= st.timeout_max) begin
                  st.timeout_counter <= 0;
                  st.SM <= HCF_3;
               end  // if(st.timeout_counter>=DELAY_TIME)
                else
                begin
                  st.timeout_counter <= st.timeout_counter + 1;
               end  // else if(st.timeout_counter>=DELAY_TIME)
            end
            HCF_3: begin
               st.timeout_counter <= 0;
               st.SM <= HCF_4;
               r_error_display_type <= ~r_error_display_type;
            end
            HCF_4: begin
               if (r_error_display_type) begin
                  // Error codes 0x1..0x9 — see ERR_* localparams above (ERR_INV_OPCODE .. ERR_TRAP).

                  case (st.error_code)
                     ERR_CHECKSUM_LOAD:
                     // incoming checksum
                     st.seven_seg_value1 <= {
                        4'h0,
                        r_rec_checksum[15:12],
                        4'h0,
                        r_rec_checksum[11:8],
                        4'h0,
                        r_rec_checksum[7:4],
                        4'h0,
                        r_rec_checksum[3:0]
                     };
                     ERR_DATA_LOAD:  // Load counter
              begin
                        st.seven_seg_value1 <= {
                           8'h24,
                           8'h24,
                           4'h0,
                           r_ram_next_write_addr[23:20],
                           4'h0,
                           r_ram_next_write_addr[19:16]
                        };
                        st.seven_seg_value2 <= {
                           4'h0,
                           r_ram_next_write_addr[15:12],
                           4'h0,
                           r_ram_next_write_addr[11:8],
                           4'h0,
                           r_ram_next_write_addr[7:4],
                           4'h0,
                           r_ram_next_write_addr[3:0]
                        };
                     end
                     default: // Also for opcode 1
                            // Blank then Program counter
                     begin
                        st.seven_seg_value1 <= {8'h22, 8'h22, 4'h0, st.PC[23:20], 4'h0, st.PC[19:16]};
                        st.seven_seg_value2 <= {
                           4'h0, st.PC[15:12], 4'h0, st.PC[11:8], 4'h0, st.PC[7:4], 4'h0, st.PC[3:0]
                        };
                     end


                  endcase
               end   // if (r_error_display_type)
                else
                begin

                  case (st.error_code)
                     ERR_CHECKSUM_LOAD:
                     // Calculated checksum
                     st.seven_seg_value1 <= {
                        4'h0,
                        r_calc_checksum[15:12],
                        4'h0,
                        r_calc_checksum[11:8],
                        4'h0,
                        r_calc_checksum[7:4],
                        4'h0,
                        r_calc_checksum[3:0]
                     };

                     ERR_DATA_LOAD: begin
                        // Three blanks then loading byte counter
                        st.seven_seg_value1 <= 32'h22_22_22_22;
                        st.seven_seg_value2 <= {8'h22, 8'h22, 8'h22, 6'h0, r_load_byte_counter[1:0]};
                     end
                     default // Also for opcode 1
                        // Show the full 32-bit opcode across both displays:
                        //   7seg1 = w_opcode[31:16] (upper 4 hex digits)
                        //   7seg2 = w_opcode[15:0]  (lower 4 hex digits)
                     begin
                        st.seven_seg_value1 <= {
                           4'h0,
                           w_opcode[31:28],
                           4'h0,
                           w_opcode[27:24],
                           4'h0,
                           w_opcode[23:20],
                           4'h0,
                           w_opcode[19:16]
                        };
                        st.seven_seg_value2 <= {
                           4'h0,
                           w_opcode[15:12],
                           4'h0,
                           w_opcode[11:8],
                           4'h0,
                           w_opcode[7:4],
                           4'h0,
                           w_opcode[3:0]
                        };
                     end

                  endcase

               end  // else if (r_error_display_type)

               st.timeout_max <= 32'd100_000_000;
               if (st.timeout_counter >= st.timeout_max) begin
                  st.timeout_counter <= 0;
                  st.SM <= HCF_1;
               end  // if(st.timeout_counter>=DELAY_TIME)
                else
                begin
                  st.timeout_counter <= st.timeout_counter + 1;
               end  // else if(st.timeout_counter>=DELAY_TIME)

            end

            HALTED_BREAK: begin
               // Wait for any in-flight TX to finish, then hold line low for
               // ~7.5 frames (2500 clocks at CLKS_PER_BIT=33) as a UART break,
               // then transition to HALTED.
               if (r_break_counter == 0) begin
                  if (!w_sending_msg) begin
                     r_break_active  <= 1'b1;
                     r_break_counter <= 12'd2500;
                  end
               end else begin
                  r_break_counter <= r_break_counter - 1;
                  if (r_break_counter == 12'd1) begin
                     r_break_active <= 1'b0;
                     st.SM           <= HALTED;
                  end
               end
            end

            HALTED: begin
               // CPU halted - do nothing until reset
            end

            default: st.SM <= HCF_1;  // loop in error
         endcase  // case(st.SM)

         // MMIO write handler — MOVED to AFTER case(st.SM): a converted store's
         // whole-struct `st <= f_x(st)` NBA in OPCODE_EXECUTE would otherwise clobber
         // the peripheral st fields (esp. st.int_mask) that this handler writes in the
         // same cycle. Running after the case makes the MMIO write win (correct: a
         // memory-mapped store must take effect). Boot/HCF display-override states issue
         // no MMIO stores, so nothing legitimate is overridden.
         if (w_mmio_write_DV) begin
            case (w_mmio_addr[27:16])
               12'h001: begin  // UART TX
                  case (w_mmio_addr[15:0])
                     // TX_DATA: write low byte -> transmit one byte via the message
                     // sender (length=1). SW must poll STATUS.TX-busy (bit0) first.
                     16'h0000: begin
                        r_msg[7:0]    <= w_mmio_write_data[7:0];
                        r_msg_length  <= 8'd1;
                        r_msg_send_DV <= 1'b1;
                     end
                     default: ;
                  endcase
               end
               12'h002: begin  // RGB LEDs
                  case (w_mmio_addr[15:0])
                     16'h0000: st.RGB_LED_1 <= w_mmio_write_data[11:0];
                     16'h0008: st.RGB_LED_2 <= w_mmio_write_data[11:0];
                     default: ;
                  endcase
               end
               12'h003: begin  // 7-segment display
                  case (w_mmio_addr[15:0])
                     // SEG_LOW: 4 hex digits → lower display (value2)
                     16'h0000: st.seven_seg_value2 <= {
                        4'h0, w_mmio_write_data[15:12],
                        4'h0, w_mmio_write_data[11:8],
                        4'h0, w_mmio_write_data[7:4],
                        4'h0, w_mmio_write_data[3:0]
                     };
                     // SEG_HIGH: 4 hex digits → upper display (value1)
                     16'h0008: st.seven_seg_value1 <= {
                        4'h0, w_mmio_write_data[15:12],
                        4'h0, w_mmio_write_data[11:8],
                        4'h0, w_mmio_write_data[7:4],
                        4'h0, w_mmio_write_data[3:0]
                     };
                     // SEG_ALL: 8 hex digits across both displays
                     16'h0010: begin
                        st.seven_seg_value1 <= {
                           4'h0, w_mmio_write_data[31:28],
                           4'h0, w_mmio_write_data[27:24],
                           4'h0, w_mmio_write_data[23:20],
                           4'h0, w_mmio_write_data[19:16]
                        };
                        st.seven_seg_value2 <= {
                           4'h0, w_mmio_write_data[15:12],
                           4'h0, w_mmio_write_data[11:8],
                           4'h0, w_mmio_write_data[7:4],
                           4'h0, w_mmio_write_data[3:0]
                        };
                     end
                     // SEG_BLANK: any write blanks both displays
                     16'h0018: begin
                        st.seven_seg_value1 <= 32'h22222222;
                        st.seven_seg_value2 <= 32'h22222222;
                     end
                     default: ;
                  endcase
               end
               12'h004: begin  // 16-bit LED bar
                  case (w_mmio_addr[15:0])
                     16'h0000: st.led <= w_mmio_write_data[15:0];
                     default: ;
                  endcase
               end
               12'h00F: begin  // Interrupt controller / timer
                  case (w_mmio_addr[15:0])
                     16'h0000: st.int_mask           <= w_mmio_write_data[3:0];
                     // 16'h0008 (INT_PENDING) is read-only; writes ignored
                     16'h0010: r_interrupt_table[0] <= w_mmio_write_data[31:0];
                     16'h0018: r_interrupt_table[1] <= w_mmio_write_data[31:0];
                     16'h0020: r_interrupt_table[2] <= w_mmio_write_data[31:0];
                     16'h0028: r_interrupt_table[3] <= w_mmio_write_data[31:0];
                     16'h0030: begin
                        r_timer_period            <= w_mmio_write_data[31:0];
                        r_timer_interrupt_counter <= 32'h0;  // restart with new period
                     end
                     // 16'h0038 (TIMER_COUNT) is read-only; writes ignored
                     default: ;
                  endcase
               end
               default: ;
            endcase
         end
      end  // else if (w_reset_H)
   end  // always_ff @(posedge i_Clk)

   //=========================================================================
   // Instruction classifier for the performance counters. Returns one
   // mutually-exclusive class per opcode, so the Tier-2 mix counters sum to
   // the retired-instruction count. mul/div/system/io/nop fall into PC_OTHER
   // (mul/div are broken out separately by the Tier-1 *_OPS counters).
   // ISA v2: buckets follow the CLASS field (word0[29:26]); control-flow
   // sub-buckets (jump vs branch vs call vs indirect) use exact templates.
   //=========================================================================
   function [2:0] f_perf_class;
      input [31:0] op;
      begin
         // ISA v2: the CLASS field IS the bucket; only class 8/9 need
         // sub-classification (jump vs branch vs call vs indirect; PUSH/POP
         // vs SP ops). Applies to every legal encoding by construction.
         case (op[29:26])
            4'h1, 4'h2, 4'h3, 4'h4, 4'h5:                   // ALU / compare / shift / unary
               f_perf_class = PC_ALU;
            4'h6: f_perf_class = PC_LOAD;
            4'h7: f_perf_class = PC_STORE;
            4'h8: begin                                     // branch family
               if (op[23])                                  // RIND: JMPR / CALLR
                  f_perf_class = PC_INDIRECT;
               else if (op[25])                             // LINK: CALL / CALLcc / CALLREL
                  f_perf_class = PC_CALL;
               else if (op[22:19] == 4'd0)                  // COND=always: JMP / JMPREL
                  f_perf_class = PC_JUMP;
               else
                  f_perf_class = PC_BRANCH;
            end
            4'hD: f_perf_class = PC_BRANCH;                 // v3 B fused compare-and-branch
            4'h9: begin                                     // stack: PUSH*/POP move data
               case (op[25:22])
                  4'd0, 4'd1, 4'd8: f_perf_class = PC_STORE; // PUSH / PUSHI / ENTER (pushes R15)
                  4'd2, 4'd9: f_perf_class = PC_LOAD;       // POP / LEAVE (pops R15)
                  4'd6, 4'd7, 4'd10: f_perf_class = PC_INDIRECT; // RET / IRET / LEAVERET
                  default:    f_perf_class = PC_OTHER;      // GETSP / SETSP / ADDSP
               endcase
            end
            default:                                        // mul/div, system, I/O, illegal
               f_perf_class = PC_OTHER;
         endcase
      end
   endfunction


   //=========================================================================
   // Performance-counter block (Tier 0/1/2). Its own always block: reads the
   // pipeline's perf strobes (pip_perf_*, pip_ret_*, pip_irq_ack) and st.SM.
   // Writes only the r_perf_* counters, so it adds no logic to the main CPU
   // critical path. Free-running; PERF_CTRL bit 0 (or hard reset) clears all.
   //=========================================================================
   always_ff @(posedge i_Clk) begin
      if (w_reset_H || w_perf_stat_clear) begin
         r_perf_cycles           <= 64'd0;  r_perf_instr            <= 64'd0;
         r_perf_exec_cycles      <= 64'd0;  r_perf_idle_cycles      <= 64'd0;
         r_perf_int_ops          <= 64'd0;
         r_perf_cnt_alu          <= 64'd0;  r_perf_cnt_load         <= 64'd0;
         r_perf_cnt_store        <= 64'd0;  r_perf_cnt_branch       <= 64'd0;
         r_perf_cnt_branch_taken <= 64'd0;  r_perf_cnt_jump         <= 64'd0;
         r_perf_cnt_call         <= 64'd0;  r_perf_cnt_indirect     <= 64'd0;
         r_perf_cnt_other        <= 64'd0;
         r_perf_stall_data <= 48'd0;  r_perf_stall_loaduse <= 48'd0;
         r_perf_stall_flags <= 48'd0; r_perf_stall_sp <= 48'd0;
         r_perf_stall_muldiv <= 48'd0; r_perf_branch_flush <= 48'd0;
         r_perf_if_miss <= 48'd0;     r_perf_mem_wait <= 48'd0;
      end else begin
         // M6 pipeline hazard attribution
         if (pip_perf_stall[0]) r_perf_stall_data    <= r_perf_stall_data    + 48'd1;
         if (pip_perf_stall[1]) r_perf_stall_loaduse <= r_perf_stall_loaduse + 48'd1;
         if (pip_perf_stall[2]) r_perf_stall_flags   <= r_perf_stall_flags   + 48'd1;
         if (pip_perf_stall[3]) r_perf_stall_sp      <= r_perf_stall_sp      + 48'd1;
         if (pip_perf_stall[4]) r_perf_stall_muldiv  <= r_perf_stall_muldiv  + 48'd1;
         if (pip_perf_stall[5]) r_perf_branch_flush  <= r_perf_branch_flush  + 48'd1;
         if (pip_perf_stall[6]) r_perf_if_miss       <= r_perf_if_miss       + 48'd1;
         if (pip_perf_stall[7]) r_perf_mem_wait      <= r_perf_mem_wait      + 48'd1;
         // EX-resolved conditional jumps + MEM-resolved fused branches (v3 B);
         // both can land on the same cycle.
         if ((pip_perf_br && pip_perf_br_taken) || pip_perf_fbr_taken)
            r_perf_cnt_branch_taken <= r_perf_cnt_branch_taken
                                     + ((pip_perf_br && pip_perf_br_taken && pip_perf_fbr_taken) ? 64'd2 : 64'd1);
         // Tier 0 — total cycles and retired instructions.
         r_perf_cycles <= r_perf_cycles + 64'd1;
         if (pip_ret_valid)
            r_perf_instr <= r_perf_instr + 64'd1;

         // Tier 1 — cycle buckets + interrupt dispatches.
         if (st.SM == HALTED || st.SM == HALTED_BREAK)
            r_perf_idle_cycles <= r_perf_idle_cycles + 64'd1;
         else if (st.SM == PIPE_RUN)
            r_perf_exec_cycles <= r_perf_exec_cycles + 64'd1;
         if (pip_irq_ack)
            r_perf_int_ops <= r_perf_int_ops + 64'd1;

         // Tier 2 — instruction mix: one class per committed instruction.
         // M5d: pipeline retires classify by the retired opcode.
         if (pip_ret_valid) begin
            case (f_perf_class(pip_ret_op))
               PC_ALU:      r_perf_cnt_alu      <= r_perf_cnt_alu      + 64'd1;
               PC_LOAD:     r_perf_cnt_load     <= r_perf_cnt_load     + 64'd1;
               PC_STORE:    r_perf_cnt_store    <= r_perf_cnt_store    + 64'd1;
               PC_BRANCH:   r_perf_cnt_branch   <= r_perf_cnt_branch   + 64'd1;
               PC_JUMP:     r_perf_cnt_jump     <= r_perf_cnt_jump     + 64'd1;
               PC_CALL:     r_perf_cnt_call     <= r_perf_cnt_call     + 64'd1;
               PC_INDIRECT: r_perf_cnt_indirect <= r_perf_cnt_indirect + 64'd1;
               default:     r_perf_cnt_other    <= r_perf_cnt_other    + 64'd1;
            endcase
         end

      end
   end

   //=========================================================================
   // Bus-wedge flight recorder (see the r_wedge_* declaration for the bit
   // map). Pure observers off the bus/handshake FFs — no functional fan-out.
   //=========================================================================
   // The clear is REGISTERED before use: the raw w_perf_stat_clear cone
   // (st.SM -> w_pipe_owns -> write_DV mux -> MMIO decode) missed timing as a
   // direct reset fan-in to these 150-odd FFs (WNS -0.45). One cycle of clear
   // latency is irrelevant here (clears and hangs are milliseconds apart).
   logic r_wedge_clear;
   always_ff @(posedge i_Clk) begin
      r_wedge_clear <= w_reset_H || w_perf_stat_clear;
      if (r_wedge_clear) begin
         r_wedge_cnt     <= '0;
         r_wedge_latched <= 1'b0;
         r_wedge_snap0   <= 64'd0;
         r_wedge_snap1   <= 64'd0;
      end else begin
         if (cpu_mem.read_DV || cpu_mem.write_DV)
            r_wedge_cnt <= r_wedge_cnt + 21'd1;   // DV held: transaction in flight
         else
            r_wedge_cnt <= '0;                    // completed/idle: not stuck
         if (!r_wedge_latched && r_wedge_cnt[20]) begin
            r_wedge_latched <= 1'b1;
            r_wedge_snap0 <= {1'b1, 9'b0,
                              pip_bus_idle,             // [53]
                              pip_perf_stall[6],        // [52] if_miss
                              pip_perf_stall[7],        // [51] mem_busy
                              w_irq_ready,              // [50]
                              r_timer_interrupt,        // [49]
                              w_pipe_owns,              // [48]
                              dram_mem.ready,           // [47]
                              w_eth_ready,              // [46]
                              w_mmio_ready,             // [45]
                              r_mmio_read_dv_d,         // [44]
                              w_mmio_read_DV,           // [43]
                              cpu_mem.ready,            // [42]
                              cpu_mem.write_DV,         // [41]
                              cpu_mem.read_DV,          // [40]
                              // [39:32] reserved (was st.SM — sampling the
                              // one-hot state vector forced a one-hot->binary
                              // encoder that anchored st.SM replication and
                              // cost WNS; w_pipe_owns [48] carries the only
                              // state question that matters here)
                              8'b0,
                              cpu_mem.addr};            // [31:0]
            r_wedge_snap1 <= {r_timer_interrupt_counter[25:0],  // [63:38]
                              w_irq_src1,               // [37]
                              w_irq_src0,               // [36]
                              pip_int_mask,             // [35:32]
                              r_perf_cycles[31:0]};     // [31:0]
         end
      end
   end

   //=========================================================================
   // Bit manipulation helper functions (active during single cycles)
   //=========================================================================
   // Population count - count number of 1 bits
   function [6:0] popcount;
      input [63:0] val;
      integer i;
      begin
         popcount = 0;
         for (i = 0; i < 64; i = i + 1) begin
            popcount = popcount + val[i];
         end
      end
   endfunction
   
   // Count leading zeros (64-bit)
   // Iterate low->high so the last overwrite is the highest set bit.
   function [6:0] count_leading_zeros;
      input [63:0] val;
      integer clz_i;
      logic [6:0] clz_result;
      begin
         clz_result = 7'd64;
         for (clz_i = 0; clz_i < 64; clz_i = clz_i + 1) begin
            if (val[clz_i])
               clz_result = 7'd63 - clz_i[6:0];
         end
         count_leading_zeros = clz_result;
      end
   endfunction

   // Count trailing zeros (64-bit)
   // Iterate high->low so the last overwrite is the lowest set bit.
   function [6:0] count_trailing_zeros;
      input [63:0] val;
      integer ctz_i;
      logic [6:0] ctz_result;
      begin
         ctz_result = 7'd64;
         for (ctz_i = 63; ctz_i >= 0; ctz_i = ctz_i - 1) begin
            if (val[ctz_i])
               ctz_result = ctz_i[6:0];
         end
         count_trailing_zeros = ctz_result;
      end
   endfunction

   // Bit reverse (64-bit)
   function [63:0] bit_reverse;
      input [63:0] val;
      integer br_i;
      begin
         for (br_i = 0; br_i < 64; br_i = br_i + 1) begin
            bit_reverse[63-br_i] = val[br_i];
         end
      end
   endfunction

endmodule

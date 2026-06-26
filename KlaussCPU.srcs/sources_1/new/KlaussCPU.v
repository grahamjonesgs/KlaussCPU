
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
    output reg        o_LCD_DC,
    output reg        o_LCD_reset_n,
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
   // name every `always @(posedge i_Clk)` and `.i_Clk(i_Clk)` already uses — it is
   // now an internal net driven by ui_clk, not the board oscillator. The board
   // oscillator (i_Clk_board) feeds only the MIG's 200 MHz reference (via clk_wiz
   // inside mem_read_write). This makes the cache↔MIG crossing a single domain.
   wire i_Clk = w_ui_clk;
   wire w_ui_clk;

   localparam STACK_TOP = 32'h800_0000;  // one doubleword (8 bytes) above top of 128 MiB byte address space

   // State Machine Code
   localparam OPCODE_REQUEST = 32'h1, OPCODE_FETCH = 32'h2, OPCODE_FETCH2 = 32'h4;
   localparam VAR1_FETCH = 32'h8, VAR1_FETCH2 = 32'h10, WAITING = 32'h20;  // WAITING: interruptible core-suspend (WAIT opcode); uses the state bit above VAR1_FETCH2
   localparam START_WAIT = 32'h40, UART_DELAY = 32'h80, OPCODE_EXECUTE = 32'h100;
   localparam HCF_1 = 32'h200, HCF_2 = 32'h400, HCF_3 = 32'h800, HCF_4 = 32'h1_000;
   localparam NO_PROGRAM = 32'h2_000, LOAD_START = 32'h4_000, LOADING_BYTE = 32'h8_000;
   localparam LOAD_COMPLETE = 32'h10_000, LOAD_WAIT = 32'h20_000;
   localparam DEBUG_DATA = 32'h40_000, DEBUG_DATA2 = 32'h80_000, DEBUG_DATA3 = 32'h100_000;
   localparam DEBUG_WAIT = 32'h200_000;
   localparam MULTIPLY_CALC      = 32'h0040_0000;  // DSP pipeline stage 2 (MREG)
   localparam MULTIPLY_PIPE      = 32'h0100_0000;  // DSP pipeline stage 3 (PREG)
   localparam MULTIPLY_WRITEBACK = 32'h0080_0000;  // Write result
   localparam MULTIPLY_SETUP     = 32'h4000_0000;  // Setup operands for multiply
   localparam WRITEBACK          = 32'h0200_0000;  // Register file writeback stage
   localparam HALTED             = 32'h0400_0000;  // CPU halted, waiting for reset
   localparam DIVIDE_STEP        = 32'h0800_0000;  // Division iteration state
   localparam HALTED_BREAK       = 32'h1000_0000;  // Sending UART break before halt
   localparam MULTIPLY_BREG      = 32'h2000_0000;  // DSP pipeline stage 1 (AREG/BREG)
   localparam HCF_DUMP           = 32'h8000_0000;  // Crash dump UART emission (sub-state inside r_hcf_dump_phase / r_hcf_dump_sub)
   // Pipeline register for the 64-bit ALU compute path. Arithmetic / compare
   // tasks register their result + flags into r_alu_pipe_* (one cycle), then
   // ALU_FINISH copies the intermediates out to the architectural flag regs
   // and r_writeback_value (next cycle). Splits the long
   //   r_reg_port_b → 16 CARRY4 → 7 LUT6 → r_carry_flag
   // path into two shorter stages for timing closure.
   localparam ALU_FINISH         = 33'h1_0000_0000;
   // Divide normalization: one cycle between the div task and DIVIDE_STEP that
   // pre-shifts the dividend by its leading-zero count, so the iteration loop
   // runs (64 - clz) steps instead of a fixed 64. Bit-identical result — the
   // skipped iterations shift zeros into the remainder and emit 0 quotient
   // bits. Its own state keeps the CLZ + 64-bit shifter off both the
   // OPCODE_EXECUTE decode region and DIVIDE_STEP's trial-subtract carry chain
   // (the documented critical paths).
   localparam DIVIDE_PREP        = 34'h2_0000_0000;

   // Error Codes
   localparam ERR_INV_OPCODE = 8'h1, ERR_INV_FSM_STATE = 8'h2, ERR_STACK = 8'h3;
   localparam ERR_DATA_LOAD = 8'h4, ERR_CHECKSUM_LOAD = 8'h5, ERR_OVERFLOW = 8'h6;
   localparam ERR_SEG_WRITE_TO_CODE = 'h7, ERR_SEG_EXEC_DATA = 'h8;
   localparam ERR_TRAP = 8'h9;        // Explicit software trap (TRAP opcode)

   // Crash dump phase boundaries (r_hcf_dump_phase). Each "phase" emits one UART line.
   localparam DUMP_HEADER     = 7'd0;
   localparam DUMP_ERR_PC     = 7'd1;
   localparam DUMP_OPC_SP     = 7'd2;
   localparam DUMP_V1_V2      = 7'd3;
   localparam DUMP_V1H        = 7'd4;   // V1H=xxxxxxxx — hi32 of 64-bit immediate (DRAM read at PC+8)
   localparam DUMP_OPCM       = 7'd5;   // OPCM=xxxxxxxx — DRAM-side re-read at PC; differ from OPC ⇒ cache mismatch
   localparam DUMP_SM         = 7'd6;   // SM=xxxxxxxxx — FSM state (34-bit one-hot, 9 hex digits)
   localparam DUMP_IV0        = 7'd7;   // IV0=xxxxxxxx — timer ISR vector (r_interrupt_table[0])
   localparam DUMP_FLAGS_A    = 7'd8;   // Z E C V
   localparam DUMP_FLAGS_B    = 7'd9;   // S L U
   localparam DUMP_INSTR      = 7'd10;  // INSTR=NNNNNNNN — instructions committed since program load
   localparam DUMP_REG_BASE   = 7'd11;  // R0..RF → phases 11..26
   localparam DUMP_STACK_BASE = 7'd27;  // S0..S3 → phases 27..30 (each preceded by a DDR2 read)
   localparam DUMP_TRACE_BASE = 7'd31;  // T0..TF → phases 31..46 (newest-first)
   localparam DUMP_FOOTER     = 7'd47;  // last phase; on completion → HCF_2

   // UART receive control
   wire [7:0] w_uart_rx_value;  // Received value
   wire w_uart_rx_DV;  // receive flag
   wire w_uart_break;  // Break condition detected
   reg  r_break_received;  // Set after break, cleared after command byte

   // UART RX FIFO — buffers bytes for the CPU to read via RXRB / RXRNB opcodes
   wire       w_rx_fifo_empty;
   wire       w_rx_fifo_full;
   wire [7:0] w_rx_fifo_byte;   // combinatorial peek at FIFO head
   reg        r_rx_fifo_read;   // 1-cycle strobe: pop one byte from FIFO

   // LCD control
   reg [3:0] o_TX_LCD_Count;  // # bytes per CS low
   reg [7:0] o_TX_LCD_Byte;  // Byte to transmit on MOSI
   reg o_TX_LCD_DV;  // Data Valid Pulse with i_TX_Byte
   wire i_TX_LCD_Ready;  // Transmit Ready for next byte

   // RX (MISO) Signals
   wire [3:0] i_RX_LCD_Count;  // Index RX byte
   wire i_RX_LCD_DV;  // Data Valid pulse (1 clock cycle)
   wire [7:0] i_RX_LCD_Byte;  // Byte received on MISO

   reg [44:0] r_timeout_counter;  // Room for 32 bits plus the 13 left shift in the timing task
   reg [44:0] r_timeout_max;

   // Machine control
   reg [33:0] r_SM;   // 34 bits: bit 32 is ALU_FINISH, bit 33 is DIVIDE_PREP
   // Snapshot of r_SM at fault time.  r_SM gets overwritten by the HCF chain
   // (HCF_1 → HCF_DUMP → ...) before the dump emits, so dumping r_SM directly
   // is useless.  We continuously copy r_SM into r_fault_sm while the FSM is
   // in any *non-HCF* state; the moment the trap fires, r_fault_sm freezes
   // at the state that was running immediately before the transition into
   // HCF_1, which is what we actually want in the crash dump.
   reg [33:0] r_fault_sm = 34'b0;
   reg [31:0] r_PC;           // byte address, always word-aligned (bits [1:0] = 0)
   reg [31:0] r_mem_read_addr;
   wire [31:0] w_opcode;
   wire [31:0] w_var1;
   wire [31:0] w_var2;
   wire [31:0] w_mem;
   reg [3:0] r_reg_1;
   reg [3:0] r_reg_2;
   reg [3:0] r_reg_dst;
   reg [1:0] r_extra_clock;
   reg [31:0] r_idx_base_addr;  // Saved base address for indexed register ops (byte addr) — driven/consumed in alu_extended_tasks.vh / uart_tasks.vh
   reg r_hcf_message_sent;
   reg [31:0] r_start_wait_counter;

   // Crash-dump trace ring buffer — captures {PC, opcode} of every dispatched
   // fetch in OPCODE_FETCH2.  On HCF entry the most recent 16 entries (newest at
   // r_trace_idx-1) are flushed over UART so a crash log shows the branch-history
   // leading up to the failing instruction, not just the failing instruction itself.
   reg [63:0] r_trace_buf [0:15];
   reg [3:0]  r_trace_idx;          // next-write index, wraps freely
   // Free-running count of instructions committed since program load. 32-bit
   // wraps at ~4.3e9 — plenty for crash diagnostics. Reset on initial,
   // CPU_RESETN, and successful program load (LOAD_COMPLETE).
   reg [31:0] r_instr_count;
   reg        r_trace_full;         // 1 once the ring has wrapped at least once

   // Crash dump UART state machine.  Lives entirely inside HCF_DUMP; the sub-state
   // r_hcf_dump_sub walks each line through the UART handshake used elsewhere
   // (PREP → BYTE_BUILD → ACK → DONE_WAIT), with an extra STACK_FETCH branch
   // for lines that need a DDR2 read first.
   //
   // BYTE_BUILD streams r_msg one byte per cycle from f_dump_byte (see
   // uart_tasks.vh).  The legacy single-cycle build collapsed to a 207-input
   // 256-bit mux in synth; the byte-streamed form is an 8-bit mux instead.
   reg [6:0]  r_hcf_dump_phase;     // which dump line to emit (see DUMP_* localparams)
   reg [2:0]  r_hcf_dump_sub;       // 000=PREP, 001=ACK, 010=DONE_WAIT, 011=STACK_FETCH, 100=BREAK, 101=BYTE_BUILD, 110=PRE_DISPLAY
   reg [4:0]  r_hcf_dump_byte_pos;  // current byte index during BYTE_BUILD (0..length-1)
   reg [63:0] r_hcf_stack_data;    // captured stack doubleword for the active stack phase
   reg        r_hcf_stack_loaded;   // 1 when r_hcf_stack_data is valid for current phase

   //load control
   //reg          o_ram_write_DV;
   reg [31:0] o_ram_write_value;
   reg [31:0] o_ram_write_addr;
   reg [31:0] r_ram_next_write_addr;
   reg [7:0] rx_count;
   reg [2:0] r_load_byte_counter;
   reg [15:0] r_checksum;
   reg [15:0] r_old_checksum;
   reg [15:0] r_calc_checksum;
   reg [15:0] r_rec_checksum;
   reg [31:0] r_PC_requested;

   // Register control
   reg [63:0] r_register[15:0];
   reg r_zero_flag;
   reg r_equal_flag;
   reg r_carry_flag;
   reg r_overflow_flag;
   reg [7:0] r_error_code;

   // -----------------------------------------------------------------------
   // ALU pipeline registers — written by arithmetic / compare tasks during
   // OPCODE_EXECUTE; consumed in ALU_FINISH (next cycle) to drive the
   // architectural flags + r_writeback_value. Adds +1 cycle to ADD/SUB/CMP
   // ops in exchange for breaking the 64-bit subtractor → carry-flag path.
   // r_alu_pipe_mode picks between ARITH (0) and CMP (1) finish behavior.
   // -----------------------------------------------------------------------
   reg [63:0] r_alu_pipe_value;     // ARITH+CMP: subtract/add result
   reg        r_alu_pipe_carry;     // ARITH only
   reg        r_alu_pipe_overflow;  // ARITH only
   reg        r_alu_pipe_equal;     // CMP only
   reg        r_alu_pipe_less;      // CMP only (signed less-than)
   reg        r_alu_pipe_ult;       // CMP only (unsigned less-than)
   reg        r_alu_pipe_mode;      // 0 = ARITH (carry/overflow/sign/value), 1 = CMP (equal/less/ult/sign)

   // Display value
   reg [31:0] r_seven_seg_value1;
   reg [31:0] r_seven_seg_value2;
   reg r_error_display_type;
   reg [11:0] r_RGB_LED_1;
   reg [11:0] r_RGB_LED_2;

   // Stack control — stack now lives in DDR2 RAM, top of 128 MiB, growing down
   // SP = 32'h800_0000 means empty; PUSH: SP-=8, mem[SP]=val; POP: val=mem[SP], SP+=8
   // R15 is the frame pointer by convention (software convention only, no hardware enforcement)
   reg [31:0] r_SP;           // byte address, always doubleword-aligned (bits [2:0] = 0)
   reg        r_int_push_wait;  // set while waiting for DDR2 to complete interrupt PC-push

   // UART send message
   reg [255:0] r_msg;  // 32 bytes — longest message is 21 bytes (case 2 of t_tx_message)
   reg [7:0] r_msg_length;
   reg r_msg_send_DV;
   reg r_mem_was_ready;  // driven/consumed in uart_tasks.vh
   wire i_msg_sent_DV;
   wire w_sending_msg;

   // String transmission state machines (for TXSTRMEM and TXSTRMEMR)
   reg [2:0] r_tx_str_state_mem;   // State machine for TXSTRMEM (imm32 address)
   reg [2:0] r_tx_str_state_reg;   // State machine for TXSTRMEMR (register address)
   reg [26:0] r_tx_str_addr_mem;   // Current address for TXSTRMEM
   reg [26:0] r_tx_str_addr_reg;   // Current address for TXSTRMEMR
   reg        r_tx_str_done_mem;   // Persists has_null across UART handshake states (TXSTRMEM)
   reg        r_tx_str_done_reg;   // Persists has_null across UART handshake states (TXSTRMEMR)

   // temp vars for timing
   reg r_timing_start;

   // Interrupt handler
   reg [31:0] r_interrupt_table[3:0];
   reg r_timer_interrupt;
   reg [31:0] r_timer_interrupt_counter;
   reg [63:0] r_timer_interrupt_counter_sec;
   // Per-source interrupt enable. Bit N = source N enabled (1) / masked (0).
   // Software-controlled via INTMASKR/INTMASKV. Hardware auto-clears the
   // dispatched source's bit on entry; IRET restores the 4-bit mask from the
   // stack slot (bits [42:39] of the saved context word).
   reg [ 3:0] r_int_mask;
   // Timer-interrupt period in raw clock cycles. Software-controlled via
   // TIMERSETR/TIMERSETV. Counter rolls over (and asserts r_timer_interrupt)
   // when r_timer_interrupt_counter > r_timer_period.
   reg [31:0] r_timer_period;

   // Interrupt-wake condition — an unmasked, vectored source is pending. This is
   // the SAME expression OPCODE_REQUEST uses to dispatch, factored out so the
   // WAITING (WAIT opcode) suspend exits on exactly the dispatch condition (no
   // separate edge logic → no lost wakeups).
   //   source 0 = timer (r_timer_interrupt, hardware-cleared on dispatch)
   //   source 1 = blitter DONE (w_blit_irq = blitter r_done & IRQ_EN; level —
   //              the ISR must ack by writing STATUS.DONE (W1C) before IRET)
   // A source fires only when it has a non-zero vector AND its mask bit is set.
   // w_irq_sel picks the source to dispatch; timer (0) has priority over blitter
   // (1). Sources 2..3 remain free for future devices — OR them in here.
   wire w_irq_src0  = r_timer_interrupt && (r_interrupt_table[0] != 32'h0) && r_int_mask[0];
   wire w_irq_src1  = w_blit_irq        && (r_interrupt_table[1] != 32'h0) && r_int_mask[1];
   wire w_irq_ready = w_irq_src0 || w_irq_src1;
   wire [1:0] w_irq_sel = w_irq_src0 ? 2'd0 : 2'd1;

   // Free-running millisecond counter (since LOAD_COMPLETE). 64-bit so it
   // takes ~5.8e8 years to wrap at 100 MHz. Read-only via MMIO 0xF00F_0040.
   // r_clock_ms_div counts 0..99_999 (one ms at 100 MHz) before incrementing
   // r_clock_ms — 17 bits hold up to 131071 so 99_999 fits comfortably.
   reg [63:0] r_clock_ms;
   reg [16:0] r_clock_ms_div;

   // Memory
   reg r_mem_write_DV;
   reg r_mem_read_DV;
   reg [31:0] r_mem_addr;      // byte address
   reg [63:0] r_mem_write_data;
   reg [ 7:0] r_mem_byte_en;   // byte enables: 8'hFF=full doubleword, else partial op
   wire [63:0] w_mem_read_data;
   wire [63:0] w_mem_read_data_next; // next doubleword in same cache line
   wire        w_mem_next_valid;     // 1 when w_mem_read_data_next is valid

   // -------------------------------------------------------------------------
   // Bus splitter outputs — DRAM side (to mem_read_write) and MMIO side
   // (to peripheral logic below). Splitter routes on i_mem_addr[31:28]:
   //   4'hF → MMIO, else DRAM. CPU FSM sees only the original r_mem_*/w_mem_*
   //   signals; routing is invisible above this line.
   // -------------------------------------------------------------------------
   wire        w_dram_write_DV;
   wire        w_dram_read_DV;
   wire [31:0] w_dram_addr;
   wire [63:0] w_dram_write_data;
   wire [ 7:0] w_dram_byte_en;
   wire [63:0] w_dram_read_data;
   wire [63:0] w_dram_read_data_next;
   wire        w_dram_next_valid;
   wire        w_dram_ready;

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
   wire        w_eth_irq;       // LiteEth combined RX/TX event (not yet wired to INT_PENDING)

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
   reg [47:0] r_perf_cycles;        // every i_Clk cycle
   reg [47:0] r_perf_instr;         // retired (committed) instructions
   // Tier 1 — cycle accounting (buckets are disjoint; sum ≈ cycles in steady run)
   reg [47:0] r_perf_fetch_cycles;  // instruction fetch / decode states
   reg [47:0] r_perf_exec_cycles;   // execute / ALU-finish / writeback (incl. mem stalls)
   reg [47:0] r_perf_mul_cycles;    // multiply pipeline states
   reg [47:0] r_perf_div_cycles;    // iterative divide state
   reg [47:0] r_perf_int_cycles;    // interrupt context-push wait
   reg [47:0] r_perf_idle_cycles;   // HALTED / HALTED_BREAK
   reg [47:0] r_perf_mul_ops;       // multiply instructions
   reg [47:0] r_perf_div_ops;       // divide / mod instructions
   reg [47:0] r_perf_int_ops;       // interrupt dispatches
   // Tier 2 — instruction mix + branch behaviour
   reg [47:0] r_perf_cnt_alu;
   reg [47:0] r_perf_cnt_load;
   reg [47:0] r_perf_cnt_store;
   reg [47:0] r_perf_cnt_branch;        // conditional branches (cond jumps)
   reg [47:0] r_perf_cnt_branch_taken;  // of those, the taken subset
   reg [47:0] r_perf_cnt_jump;          // unconditional direct jumps (JMP/JMPREL)
   reg [47:0] r_perf_cnt_call;          // direct + conditional calls
   reg [47:0] r_perf_cnt_indirect;      // JMPR / RET / IRET / CALLR (reg/stack target)
   reg [47:0] r_perf_cnt_other;         // mul/div/system/io/nop/etc.
   // Fast-path-dispatch fire count (MMIO 0xA8): instructions that skipped the
   // FETCH2 bubble. A healthy 15-53% rate is itself live proof f_predecode_len is
   // exact — a wrong length fails the r_ir_pc==r_PC consume guard and collapses
   // the rate. (This slot replaced a passive predecode-mismatch validator, now
   // covered by the consume guard and the functional regression.)
   reg [47:0] r_perf_fastpath;       // fast-path dispatches (skipped FETCH2)
   reg        r_fastpath_fired;      // 1-cycle strobe asserted when the fast-path consumes
   reg        r_int_push_wait_d;        // edge-detect for interrupt dispatch
   // Branch-outcome strobe — set for 1 cycle by t_cond_jump / t_cond_jump_rel
   // (conditional encodings only), consumed by the perf block below.
   reg        r_perf_br_valid;
   reg        r_perf_br_taken;
   // Instruction-class codes (return value of f_perf_class)
   localparam PC_OTHER=3'd0, PC_ALU=3'd1, PC_LOAD=3'd2, PC_STORE=3'd3,
              PC_BRANCH=3'd4, PC_JUMP=3'd5, PC_CALL=3'd6, PC_INDIRECT=3'd7;
   // PERF_CTRL bit 0 self-clearing — same pattern as w_cache_stat_clear.
   wire        w_perf_stat_clear = w_mmio_write_DV
                                 && (w_mmio_addr[27:16] == 12'h00D)
                                 && (w_mmio_addr[15:0]  == 16'h0000)
                                 && w_mmio_write_data[0];

   reg  [63:0] r_mmio_read_data_comb;  // combinational decode (driven by always @*)
   reg  [63:0] r_mmio_read_data;       // registered version delivered to bus_splitter (breaks long timing path)
   reg         r_mmio_read_dv_d;       // delayed read strobe → ready pulse one cycle later
   // MMIO writes complete in 1 cycle (write-side has no read-data path).
   // MMIO reads now take 2 cycles: cycle 1 captures decode into the FF,
   // cycle 2 asserts ready so the CPU FSM samples r_mmio_read_data.
   wire        w_mmio_ready = w_mmio_write_DV | r_mmio_read_dv_d;
   wire w_mem_ready;
   reg [31:0] r_opcode_mem;
   reg [31:0] r_var1_mem;
   reg [31:0] r_var2_mem;
   reg r_var1_prefetched; // 1 when r_var1_mem was populated from the opcode cache line

   // -------------------------------------------------------------------------
   // 2-stage fetch|execute overlap. r_FPC is a prefetch pointer running ahead of
   // r_PC; a single-entry instruction-register (IR) latch holds one decoded
   // instruction. The latch is filled from the instruction buffer under a
   // bus-idle execute tail (the prefetch, below) and consumed by the fast-path
   // dispatch in OPCODE_REQUEST. The consume guard r_ir_pc == r_PC is the
   // independent safety net: a mismatch degrades to a normal re-fetch, never a
   // wrong instruction. The sequential length comes from f_predecode_len (proven
   // exact on hardware — a wrong length fails the guard and collapses the
   // fast-path rate). Execute stays serial; only the fetch front-end overlaps it.
   // -------------------------------------------------------------------------
   reg [31:0] r_FPC;              // next-sequential prefetch PC (advanced by f_predecode_len)
   reg        r_ir_valid;         // IR latch holds a valid prefetched instruction
   reg [31:0] r_ir_pc;            // PC of the latched instruction (must == r_PC to consume)
   reg [31:0] r_ir_opcode;        // its 32-bit opcode
   reg [3:0]  r_ir_reg_1;
   reg [3:0]  r_ir_reg_2;
   reg [3:0]  r_ir_reg_dst;
   reg [31:0] r_ir_var1;          // prefetched var1 (PC+4) when available in the IFB
   reg        r_ir_var1_prefetched;
   reg        r_ir_presettled;    // the execute tail already loaded r_reg_1/2 from
                                  // this latch, so the registered read ports settle
                                  // during the tail and the fast-path dispatch can
                                  // skip the FETCH2 settle bubble (saves 1 cyc/instr).

   // -------------------------------------------------------------------------
   // Instruction fetch buffer (IFB) — a one-line (two-doubleword) capture of
   // the last cache line read for opcodes. Sequential fetches that land in the
   // buffered line skip OPCODE_FETCH's ~5-cycle cache round-trip (a cache *hit*
   // still costs ~5 cycles through the cache pipeline + bus_splitter register).
   // Per-doubleword granularity so serving reuses the exact r_PC[2] half-select
   // the cache path uses — no 128-bit repacking. Filled in OPCODE_FETCH on a
   // miss; checked in OPCODE_REQUEST; invalidated when a store writes a buffered
   // line (keeps self-modifying code / the Zephyr LLEXT loader coherent — the
   // unified cache is coherent today and the IFB must not reintroduce a stale
   // copy). Branch redirects need no flush: a changed PC simply misses the tag.
   //
   // NB: held at one line (S=2) deliberately. An S=4 (two-line) variant was
   // measured and captured *zero* additional hits — every hot loop in the
   // workload exceeds two lines, and a round-robin line buffer gets no
   // cross-line reuse below whole-loop size (a cliff, not a gradient). The
   // loops already sit in L1 at ~0% miss; the IFB's only job is intra-line
   // sequential reuse, which one line fully delivers. Hiding the remaining
   // per-line cache-access latency needs overlap (prefetch / pipeline), not
   // more buffer capacity.
   // -------------------------------------------------------------------------
   reg [63:0] r_ifb_dw     [0:1];   // the two doublewords of one 16-byte line
   reg [28:0] r_ifb_dwaddr [0:1];   // dw-aligned tag = addr[31:3] per slot
   reg [ 1:0] r_ifb_dwval;          // per-slot valid

   wire [28:0] w_ifb_pc_dw  = r_PC[31:3];         // dw holding the opcode at PC
   wire [28:0] w_ifb_pc1_dw = r_PC[31:3] + 1'b1;  // dw holding PC+4 (when PC[2]==1)
   wire w_ifb_op_hit0 = r_ifb_dwval[0] && (r_ifb_dwaddr[0] == w_ifb_pc_dw);
   wire w_ifb_op_hit1 = r_ifb_dwval[1] && (r_ifb_dwaddr[1] == w_ifb_pc_dw);
   wire w_ifb_op_hit  = w_ifb_op_hit0 | w_ifb_op_hit1;
   wire [63:0] w_ifb_op_dw = w_ifb_op_hit0 ? r_ifb_dw[0] : r_ifb_dw[1];
   wire w_ifb_v1_hit0 = r_ifb_dwval[0] && (r_ifb_dwaddr[0] == w_ifb_pc1_dw);
   wire w_ifb_v1_hit1 = r_ifb_dwval[1] && (r_ifb_dwaddr[1] == w_ifb_pc1_dw);
   wire w_ifb_v1_hit  = w_ifb_v1_hit0 | w_ifb_v1_hit1;
   wire [63:0] w_ifb_v1_dw = w_ifb_v1_hit0 ? r_ifb_dw[0] : r_ifb_dw[1];

   // Instruction-buffer lookup mirrored on the prefetch pointer r_FPC (the
   // prefetch reads the buffered line at r_FPC; no cache request, never the bus).
   wire [28:0] w_ifb_fpc_dw   = r_FPC[31:3];
   wire [28:0] w_ifb_fpc1_dw  = r_FPC[31:3] + 1'b1;
   wire w_ifb_fpc_hit0  = r_ifb_dwval[0] && (r_ifb_dwaddr[0] == w_ifb_fpc_dw);
   wire w_ifb_fpc_hit1  = r_ifb_dwval[1] && (r_ifb_dwaddr[1] == w_ifb_fpc_dw);
   wire w_ifb_fpc_hit   = w_ifb_fpc_hit0 | w_ifb_fpc_hit1;
   wire [63:0] w_ifb_fpc_data = w_ifb_fpc_hit0 ? r_ifb_dw[0] : r_ifb_dw[1];
   wire w_ifb_fpcv1_hit0 = r_ifb_dwval[0] && (r_ifb_dwaddr[0] == w_ifb_fpc1_dw);
   wire w_ifb_fpcv1_hit1 = r_ifb_dwval[1] && (r_ifb_dwaddr[1] == w_ifb_fpc1_dw);
   wire w_ifb_fpcv1_hit  = w_ifb_fpcv1_hit0 | w_ifb_fpcv1_hit1;
   wire [63:0] w_ifb_fpcv1_data = w_ifb_fpcv1_hit0 ? r_ifb_dw[0] : r_ifb_dw[1];
   // Execute-tail states where the memory bus is idle and a one-entry IFB-hit
   // prefetch may fill the IR latch without contending for the cache port.
   wire w_exec_tail = (r_SM == OPCODE_EXECUTE) || (r_SM == ALU_FINISH)    ||
                      (r_SM == MULTIPLY_SETUP) || (r_SM == MULTIPLY_BREG) ||
                      (r_SM == MULTIPLY_CALC)  || (r_SM == MULTIPLY_PIPE) ||
                      (r_SM == DIVIDE_PREP)    || (r_SM == DIVIDE_STEP);

   // Debug
   reg r_debug_flag;
   reg r_debug_step_flag;
   reg r_debug_step_run;

   wire w_reset_H;
   reg r_boot_flash;
   
   //=========================================================================
   // Additional flags for expanded comparisons
   //=========================================================================
   reg r_sign_flag;      // Sign of last result (bit 63)
   reg r_less_flag;      // Result of signed less-than comparison
   reg r_ult_flag;       // Result of unsigned less-than comparison

   reg r_mul_is_immediate;  // If true, increment PC by 2 instead of 1
   
   //=============================================================================
  // PIPELINED MULTIPLY REGISTERS
  //=============================================================================

  // Pipeline stage registers (active during s_multiply state)
  reg [127:0] r_mul_pipe1;        // Stage 1: multiply result (maps to DSP48 MREG)
  reg [127:0] r_mul_pipe2;        // Stage 2: registered output (maps to DSP48 PREG)
  wire [63:0] r_mul_result_lo;
  wire [63:0] r_mul_result_hi;
  assign r_mul_result_lo = r_mul_pipe2[63:0];
  assign r_mul_result_hi = r_mul_pipe2[127:64];

  reg        r_mul_is_high;      // Are we capturing high word?
  reg        r_mul_is_unsigned;  // Unsigned operation?
  reg [3:0]  r_mul_dest_reg;     // Destination register
  // Dedicated multiply operand capture (breaks path from register file)
  reg [63:0] r_mul_operand_a;
  reg [63:0] r_mul_operand_b;

  // Sign-extended (65-bit) operand latches. Doing the signed/unsigned mux
  // *here* (before the DSP) keeps the LUT2 off the operand_q -> DSP-cascade
  // path so Vivado can absorb operand_q into DSP48E1 AREG/BREG cleanly.
  // For unsigned ops we zero-extend; for signed we sign-extend. A single
  // 65x65 signed multiply then gives the correct lower-128-bit result for
  // both cases.
  reg [64:0] r_mul_operand_a_q;
  reg [64:0] r_mul_operand_b_q;

  // Free-running 3-stage multiply pipeline (Vivado: DSP48 AREG/BREG + MREG + PREG)
  always @(posedge i_Clk) begin
     // Stage 1: sign-extend & latch operands (absorbed into DSP AREG/BREG)
     r_mul_operand_a_q <= {(r_mul_is_unsigned ? 1'b0 : r_mul_operand_a[63]), r_mul_operand_a};
     r_mul_operand_b_q <= {(r_mul_is_unsigned ? 1'b0 : r_mul_operand_b[63]), r_mul_operand_b};
     // Stage 2: multiply (MREG) - lower 128 bits of 130-bit signed product
     r_mul_pipe1 <= $signed(r_mul_operand_a_q) * $signed(r_mul_operand_b_q);
     // Stage 3: register (PREG)
     r_mul_pipe2 <= r_mul_pipe1;
  end

  // Free-running millisecond clock since FPGA boot. Lives in its own always
  // block so it ticks every cycle, independent of the main FSM, UART
  // break/command handling, and reset logic. Initial values come from the
  // top-level initial block (r_clock_ms = 0, r_clock_ms_div = 0). 64-bit
  // counter wraps in ~5.8e8 years at 100 MHz; the divider wraps every 1 ms.
  always @(posedge i_Clk) begin
     if (r_clock_ms_div >= 17'd99_999) begin
        r_clock_ms_div <= 17'd0;
        r_clock_ms     <= r_clock_ms + 64'd1;
     end else begin
        r_clock_ms_div <= r_clock_ms_div + 17'd1;
     end
  end


   //=========================================================================
   // Hardware divide using iterative but optimized state machine
   // For true single-cycle, you'd need a pipelined divider IP
   //=========================================================================
   reg [63:0] r_div_dividend;
   reg [63:0] r_div_divisor;
   reg [63:0] r_div_quotient;
   reg [63:0] r_div_remainder;
   reg [6:0]  r_div_counter;
   reg        r_div_busy;
   reg        r_div_sign_q;      // Sign of quotient
   reg        r_div_sign_r;      // Sign of remainder
   reg        r_div_is_signed;
   reg [1:0]  r_div_op;          // 0=none, 1=div, 2=mod
   reg [3:0]  r_div_dest_reg;    // Destination register for division result
   reg        r_div_pc_inc;      // 0=PC+1, 1=PC+2
   
   localparam DIV_OP_NONE = 2'd0;
   localparam DIV_OP_DIV  = 2'd1;
   localparam DIV_OP_MOD  = 2'd2;

   // Restoring-division step, factored so synthesis sees ONE 65-bit subtract
   // instead of a separate 64-bit comparator + 64-bit subtractor in series.
   // In restoring division the ">=" test and the subtract are the same
   // operation: the borrow-out of {rem,bit} - divisor IS the comparison
   // result (no borrow => result >= 0 => quotient bit 1, take the difference;
   // borrow => quotient bit 0, keep the shifted remainder).  Driving the
   // quotient-bit mux from w_div_borrow halves the logic on this path.
   wire [63:0] w_div_shifted = {r_div_remainder[62:0], r_div_dividend[63]};
   wire [64:0] w_div_trial   = {1'b0, w_div_shifted} - {1'b0, r_div_divisor};
   wire        w_div_borrow  = w_div_trial[64];
   
   // Dedicated read ports - registered every cycle
   reg [63:0] r_reg_port_a;
   reg [63:0] r_reg_port_b;

   // Writeback pipeline registers
   reg [63:0] r_writeback_value;
   reg [3:0]  r_writeback_reg;
   reg        r_writeback_set_zero_flag;  // Set zero flag from writeback value in WRITEBACK stage
   reg        r_wb_pending;               // writeback deferred into OPCODE_REQUEST dispatch (eliminates the WRITEBACK cycle)

    always @(posedge i_Clk) begin
       // RAW forward: the execute-tail pre-settle moves the port read into the
       // deferred-writeback commit cycle, so forward a still-pending writeback
       // whose dest matches the operand (the value r_register WILL hold next
       // cycle). This mux is on the port INPUT (reg-file read), not the
       // r_reg_port_b->carry-flag OUTPUT path, so the tight carry chain is
       // untouched. On the slow (FETCH2) path r_wb_pending is already 0 when the
       // port reads, so it is a no-op there.
       r_reg_port_a <= (r_wb_pending && (r_writeback_reg == r_reg_1)) ? r_writeback_value : r_register[r_reg_1];
       r_reg_port_b <= (r_wb_pending && (r_writeback_reg == r_reg_2)) ? r_writeback_value : r_register[r_reg_2];
   end

   // Track the last non-HCF FSM state so the crash dump can show what was
   // executing at the moment of the trap.  All five HCF states are excluded
   // — HCF_1 fires first, then the chain walks through HCF_DUMP / HCF_2..4
   // and would otherwise overwrite the snapshot before the dump emits.
   always @(posedge i_Clk) begin
      if ((r_SM != HCF_1) && (r_SM != HCF_DUMP) &&
          (r_SM != HCF_2) && (r_SM != HCF_3) && (r_SM != HCF_4)) begin
         r_fault_sm <= r_SM;
      end
   end

   // UART TX break generation — holds o_uart_tx low to signal program end
   wire       w_uart_tx_serial;   // internal serial output from uart_send_msg
   reg        r_break_active;     // when 1, overrides TX line to low (break)
   reg [11:0] r_break_counter;    // countdown: 2500 clocks ≈ 7.5 frames at CLKS_PER_BIT=33

   // Mux: break takes priority over normal TX; idle state is line-high
   assign o_uart_tx  = r_break_active ? 1'b0 : w_uart_tx_serial;
   assign w_reset_H  = !CPU_RESETN;

   // KEEP_HIERARCHY prevents Vivado from flattening these modules' logic into
   // surrounding CPU slices. Without it the placer can scatter sd_spi/splitter
   // cells across the CPU's 64-bit ALU carry chain, lengthening route delay on
   // an already-tight critical path (r_reg_port_b → r_carry_flag, ~25 levels).
   (* KEEP_HIERARCHY = "yes" *)
   bus_splitter bus_splitter_i (
       .i_clk(i_Clk),
       // CPU side — same names the FSM has always driven/observed
       .i_mem_write_DV(r_mem_write_DV),
       .i_mem_read_DV(r_mem_read_DV),
       .i_mem_addr(r_mem_addr),
       .i_mem_write_data(r_mem_write_data),
       .i_mem_byte_en(r_mem_byte_en),
       .o_mem_read_data(w_mem_read_data),
       .o_mem_read_data_next(w_mem_read_data_next),
       .o_mem_next_valid(w_mem_next_valid),
       .o_mem_ready(w_mem_ready),
       // DRAM side
       .o_dram_write_DV(w_dram_write_DV),
       .o_dram_read_DV(w_dram_read_DV),
       .o_dram_addr(w_dram_addr),
       .o_dram_write_data(w_dram_write_data),
       .o_dram_byte_en(w_dram_byte_en),
       .i_dram_read_data(w_dram_read_data),
       .i_dram_read_data_next(w_dram_read_data_next),
       .i_dram_next_valid(w_dram_next_valid),
       .i_dram_ready(w_dram_ready),
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
   wire        w_sd_ready;

   (* KEEP_HIERARCHY = "yes" *)
   sd_spi sd_spi_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .i_mmio_write_DV(w_sd_write_DV),
       .i_mmio_read_DV(w_sd_read_DV),
       .i_mmio_addr(w_mmio_addr[15:0]),
       .i_mmio_write_data(w_mmio_write_data),
       .i_mmio_byte_en(w_mmio_byte_en),
       .o_mmio_read_data(w_sd_read_data),
       .o_mmio_ready(w_sd_ready),
       .i_sd_cd(i_SD_CD),
       .o_sd_reset_n(o_SD_RESET),
       .o_sd_sck(o_SD_SCK),
       .o_sd_mosi(o_SD_MOSI),
       .i_sd_miso(i_SD_MISO),
       .o_sd_cs_n(o_SD_CS_n),
       .o_sd_dat1(o_SD_DAT1),
       .o_sd_dat2(o_SD_DAT2)
   );

   // -------------------------------------------------------------------------
   // Crypto AES — device id 0x00A. See CRYPTO_PLAN.md §4 and MMIO_MAP.md.
   // -------------------------------------------------------------------------
   wire        w_aes_sel      = (w_mmio_addr[27:16] == 12'h00A);
   wire        w_aes_write_DV = w_mmio_write_DV & w_aes_sel;
   wire        w_aes_read_DV  = w_mmio_read_DV  & w_aes_sel;
   wire [63:0] w_aes_read_data;
   wire        w_aes_ready;

   (* KEEP_HIERARCHY = "yes" *)
   crypto_aes crypto_aes_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .i_mmio_write_DV(w_aes_write_DV),
       .i_mmio_read_DV(w_aes_read_DV),
       .i_mmio_addr(w_mmio_addr[15:0]),
       .i_mmio_write_data(w_mmio_write_data),
       .i_mmio_byte_en(w_mmio_byte_en),
       .o_mmio_read_data(w_aes_read_data),
       .o_mmio_ready(w_aes_ready)
   );

   // -------------------------------------------------------------------------
   // Crypto SHA-256 — device id 0x00B. See CRYPTO_PLAN.md §6 and MMIO_MAP.md.
   // -------------------------------------------------------------------------
   wire        w_sha_sel      = (w_mmio_addr[27:16] == 12'h00B);
   wire        w_sha_write_DV = w_mmio_write_DV & w_sha_sel;
   wire        w_sha_read_DV  = w_mmio_read_DV  & w_sha_sel;
   wire [63:0] w_sha_read_data;
   wire        w_sha_ready;

   (* KEEP_HIERARCHY = "yes" *)
   crypto_sha crypto_sha_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .i_mmio_write_DV(w_sha_write_DV),
       .i_mmio_read_DV(w_sha_read_DV),
       .i_mmio_addr(w_mmio_addr[15:0]),
       .i_mmio_write_data(w_mmio_write_data),
       .i_mmio_byte_en(w_mmio_byte_en),
       .o_mmio_read_data(w_sha_read_data),
       .o_mmio_ready(w_sha_ready)
   );

   // -------------------------------------------------------------------------
   // Crypto TRNG — device id 0x00C. See CRYPTO_PLAN.md §7 and MMIO_MAP.md.
   // -------------------------------------------------------------------------
   wire        w_trng_sel      = (w_mmio_addr[27:16] == 12'h00C);
   wire        w_trng_write_DV = w_mmio_write_DV & w_trng_sel;
   wire        w_trng_read_DV  = w_mmio_read_DV  & w_trng_sel;
   wire [63:0] w_trng_read_data;
   wire        w_trng_ready;

   (* KEEP_HIERARCHY = "yes" *)
   trng trng_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .i_mmio_write_DV(w_trng_write_DV),
       .i_mmio_read_DV(w_trng_read_DV),
       .i_mmio_addr(w_mmio_addr[15:0]),
       .i_mmio_write_data(w_mmio_write_data),
       .i_mmio_byte_en(w_mmio_byte_en),
       .o_mmio_read_data(w_trng_read_data),
       .o_mmio_ready(w_trng_ready)
   );

   // -------------------------------------------------------------------------
   // 2D DMA blitter — device id 0x00E. MMIO slave for operands + START/STATUS;
   // a second DDR master (master B) on mem_read_write's arbiter. See
   // BLITTER_IMPLEMENTATION_PLAN.md and blitter-fpga-handoff.md.
   // -------------------------------------------------------------------------
   wire        w_blit_sel      = (w_mmio_addr[27:16] == 12'h00E);
   wire        w_blit_write_DV = w_mmio_write_DV & w_blit_sel;
   wire        w_blit_read_DV  = w_mmio_read_DV  & w_blit_sel;
   wire [63:0] w_blit_read_data;
   wire        w_blit_ready;

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
   wire         w_blit_irq;          // DONE interrupt — wired to the CPU interrupt controller (source 1)

   (* KEEP_HIERARCHY = "yes" *)
   blitter_dma blitter_dma_i (
       .i_Clk(i_Clk),
       .i_Rst_L(~w_reset_H),
       .i_mmio_write_DV(w_blit_write_DV),
       .i_mmio_read_DV(w_blit_read_DV),
       .i_mmio_addr(w_mmio_addr[15:0]),
       .i_mmio_write_data(w_mmio_write_data),
       .i_mmio_byte_en(w_mmio_byte_en),
       .o_mmio_read_data(w_blit_read_data),
       .o_mmio_ready(w_blit_ready),
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

   // -------------------------------------------------------------------------
   // MMIO read mux — combinational decode into r_mmio_read_data_comb.
   // The result is registered into r_mmio_read_data (FF) below to break the
   // long combinational path from peripheral state regs through bus_splitter
   // and into the UART helpers' FSMs (which gate the main CPU r_SM/r_PC CE).
   // Reads therefore take 2 cycles: address valid in cycle N → comb decode in
   // cycle N → registered into r_mmio_read_data at edge N+1 → ready pulses in
   // cycle N+1 so the CPU FSM samples it.
   // Returns zero for undefined offsets (treat as scratch / write-only).
   // See MMIO_MAP.md for the full memory map.
   // -------------------------------------------------------------------------

   /* Internal LED register.  The top-level `o_led` port was converted from
      `output reg` to `output wire` so we could once tap an ETH_TXEN
      diagnostic into bit 0.  The diagnostic was removed (an ODDR driving
      a pin can't have its Q net read by fabric — REQP-1884), but the
      wire-port + internal reg + continuous assign pattern remains as a
      neutral pass-through.  Same behaviour as the original output reg. */
   reg [15:0] r_led;
   assign o_led = r_led;

   always @* begin
      r_mmio_read_data_comb = 64'h0;
      case (w_mmio_addr[27:16])
         12'h000: r_mmio_read_data_comb = w_sd_read_data;  // SD card
         12'h002: begin  // RGB LEDs
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data_comb = {52'b0, r_RGB_LED_1};
               16'h0008: r_mmio_read_data_comb = {52'b0, r_RGB_LED_2};
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h003: begin  // 7-segment display (raw padded values)
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data_comb = {32'b0, r_seven_seg_value2};
               16'h0008: r_mmio_read_data_comb = {32'b0, r_seven_seg_value1};
               16'h0010: r_mmio_read_data_comb = {r_seven_seg_value1, r_seven_seg_value2};
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h004: begin  // LEDs (RW) and switches (RO)
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data_comb = {48'b0, r_led};
               16'h0008: r_mmio_read_data_comb = {48'b0, i_switch};
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         12'h005: begin  // Cache controller — counters and config (RO)
            case (w_mmio_addr[15:0])
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
         12'h00F: begin  // Interrupt controller / timer
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data_comb = {60'b0, r_int_mask};
               16'h0008: r_mmio_read_data_comb = {62'b0, w_blit_irq, r_timer_interrupt}; // INT_PENDING: [0]=timer, [1]=blitter
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
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data_comb = 64'h0;            // PERF_CTRL
               16'h0008: r_mmio_read_data_comb = r_perf_cycles;
               16'h0010: r_mmio_read_data_comb = r_perf_instr;
               16'h0018: r_mmio_read_data_comb = r_perf_fetch_cycles;
               16'h0020: r_mmio_read_data_comb = r_perf_exec_cycles;
               16'h0028: r_mmio_read_data_comb = r_perf_mul_cycles;
               16'h0030: r_mmio_read_data_comb = r_perf_div_cycles;
               16'h0038: r_mmio_read_data_comb = r_perf_int_cycles;
               16'h0040: r_mmio_read_data_comb = r_perf_idle_cycles;
               16'h0048: r_mmio_read_data_comb = r_perf_mul_ops;
               16'h0050: r_mmio_read_data_comb = r_perf_div_ops;
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
               16'h00A8: r_mmio_read_data_comb = r_perf_fastpath;  // fast-path-dispatch fire count (this slot was the retired predecode validator)
               default:  r_mmio_read_data_comb = 64'h0;
            endcase
         end
         default: r_mmio_read_data_comb = 64'h0;
      endcase
   end

   // Pipeline FF on the MMIO read return path. The FF on r_mmio_read_data
   // breaks the path from peripheral RAMs/regs (notably the SD sector buffer)
   // through bus_splitter and into uart_send_msg/uart_rx, which gate the
   // main CPU r_SM/r_PC clock enables. r_mmio_read_dv_d generates the
   // 1-cycle-delayed ready pulse so the CPU samples r_mmio_read_data on the
   // cycle after the read strobe.
   always @(posedge i_Clk) begin
      r_mmio_read_data <= r_mmio_read_data_comb;
      r_mmio_read_dv_d <= w_mmio_read_DV;
   end

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

       .i_mem_write_DV(w_dram_write_DV),
       .i_mem_read_DV(w_dram_read_DV),
       .i_mem_addr(w_dram_addr),
       .i_mem_write_data(w_dram_write_data),
       .i_mem_byte_en(w_dram_byte_en),
       .o_mem_read_data(w_dram_read_data),
       .o_mem_read_data_next(w_dram_read_data_next),
       .o_mem_next_valid(w_dram_next_valid),
       .o_mem_ready(w_dram_ready),

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
   wire        w_calib_done;
   wire [63:0] w_boot_dword;        // boot_rom read data (1-cycle latency)
   reg  [14:0] r_boot_dw_addr;      // doubleword index into the ROM / DDR
   reg  [15:0] r_boot_len_dw;       // image length in doublewords (from word 0)
   reg  [ 2:0] r_boot_phase;        // sub-state within NO_PROGRAM:
                                    //   0=wait DDR calib, 1=read word 0 (image length),
                                    //   2=ROM read latency, 3=issue DDR write, 4=ack write / advance
   reg         r_boot_active;       // 1 = boot copy pending/in-progress

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

   eth_mmio_bridge eth_mmio_bridge_i (
       .i_clk             (i_Clk),
       .i_rst             (~CPU_RESETN),
       // CPU MMIO side (from bus_splitter Eth port)
       .i_mmio_write_DV   (w_eth_write_DV),
       .i_mmio_read_DV    (w_eth_read_DV),
       .i_mmio_addr       (w_eth_addr),
       .i_mmio_write_data (w_eth_write_data),
       .i_mmio_byte_en    (w_eth_byte_en),
       .o_mmio_read_data  (w_eth_read_data),
       .o_mmio_ready      (w_eth_ready),
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

   liteeth_core liteeth_core_i (
       .sys_clock          (i_Clk),
       .sys_reset          (~CPU_RESETN),
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
   // handler (!r_break_received) or the program loader (r_SM != LOADING_BYTE).
   uart_rx_fifo uart_rx_fifo1 (
       .i_Clk        (i_Clk),
       .i_Reset      (w_reset_H),
       .i_Write_En   (w_uart_rx_DV & !r_break_received & (r_SM != LOADING_BYTE)),
       .i_Write_Byte (w_uart_rx_value),
       .i_Read_En    (r_rx_fifo_read),
       .o_Peek_Byte  (w_rx_fifo_byte),
       .o_Empty      (w_rx_fifo_empty),
       .o_Full       (w_rx_fifo_full),
       .o_Count      ()
   );


   Seven_seg_LED_Display_Controller Seven_seg_LED_Display_Controller1 (
       .i_sysclk(i_Clk),
       .i_reset(w_reset_H),
       .i_displayed_number1(r_seven_seg_value1),  // Number to display
       .i_displayed_number2(r_seven_seg_value2),  // Number to display
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
               .i_opcode_read_addr(r_PC),
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
      r_sign_flag <= 0;
      r_less_flag <= 0;
      r_ult_flag <= 0;
      r_div_busy <= 0;
      r_div_op <= DIV_OP_NONE;
      r_div_counter <= 0;
       for (i = 0; i < 16; i = i + 1)
       r_register[i] = 64'b0;
      r_mul_is_immediate = 0;
   end
   
   assign w_opcode = r_opcode_mem;
   assign w_var1   = r_var1_mem;
   assign w_var2   = r_var2_mem;
   

   // Stack module removed — stack now uses DDR2 RAM via r_SP register

   RGB_LED RGB_LED (
       .i_sysclk(i_Clk),
       .LED1(r_RGB_LED_1),
       .LED2(r_RGB_LED_2),
       .o_LED_RGB_1(o_LED_RGB_1),
       .o_LED_RGB_2(o_LED_RGB_2)
   );


   /*ila_0  myila(.clk(i_Clk),
             .probe0(w_opcode),
             .probe1(0),
             .probe2(r_PC),
             .probe3(r_SM),
             .probe4(r_var1_mem),
             .probe5(0),
             .probe6(0),
             .probe7(0),
             .probe8(r_mem_read_DV),
             .probe9(r_mem_addr),
             .probe10(w_mem_ready),
             .probe11(w_var1),
             .probe12(w_mem_read_data),
             .probe13(w_temp_cache_hit),
             .probe14(w_temp_cache_value),
             .probe15(0)

            ); */

   `include "timing_tasks.vh"
   `include "LCD_tasks.vh"
   `include "led_tasks.vh"
   `include "register_tasks.vh"
   `include "control_tasks.vh"
   `include "stack_tasks.vh"
   `include "functions.vh"
   `include "seven_seg.vh"
   `include "opcode_select.vh"
   `include "uart_tasks.vh"
   `include "memory_tasks.vh"
   `include "alu_extended_tasks.vh"    

   initial begin
      o_TX_LCD_Count = 4'd1;
      o_TX_LCD_Byte = 8'b0;
      r_SM = NO_PROGRAM;
      r_boot_active = 1'b1;
      r_boot_phase = 3'd0;
      r_boot_dw_addr = 15'd0;
      r_timeout_counter = 0;
      o_LCD_reset_n = 1'b0;
      r_PC = 32'h0;
      r_zero_flag = 0;
      r_equal_flag = 0;
      r_carry_flag = 0;
      r_overflow_flag = 0;
      r_error_code = 8'h0;
      r_timeout_counter = 32'b0;
      r_seven_seg_value1 = 32'h20_10_00_07;
      r_seven_seg_value2 = 32'h21_21_21_21;
      r_led <= 16'h0;
      rx_count = 8'b0;
      o_ram_write_addr = 32'h0;
      r_ram_next_write_addr = 32'h0;
      r_SP = 32'h800_0000;          // empty-descending stack, top of 128 MiB byte space
      r_int_push_wait = 1'b0;
      r_msg_send_DV <= 1'b0;
      r_hcf_message_sent <= 1'b0;
      r_RGB_LED_1 = 12'h000;
      r_RGB_LED_2 = 12'h000;
      r_timing_start <= 0;
      r_timer_interrupt_counter <= 0;
      r_timer_interrupt_counter_sec <= 0;
      r_int_mask <= 4'h0;            // all sources masked at power-up
      r_timer_period <= 32'h000F_FFFF;  // default ~10.5 ms @ 100 MHz
      r_clock_ms <= 64'h0;
      r_clock_ms_div <= 17'h0;
      r_mem_write_DV <= 0;
      r_mem_read_DV <= 0;
      r_mem_byte_en <= 8'hFF;
      r_msg = 256'b0;
      r_boot_flash = 0;
      r_debug_flag = 0;
      r_debug_step_flag = 0;
      r_debug_step_run = 0;
      r_break_received = 0;
      r_writeback_set_zero_flag = 0;
      r_wb_pending        = 0;
      r_alu_pipe_value    = 64'b0;
      r_alu_pipe_carry    = 1'b0;
      r_alu_pipe_overflow = 1'b0;
      r_alu_pipe_equal    = 1'b0;
      r_alu_pipe_less     = 1'b0;
      r_alu_pipe_ult      = 1'b0;
      r_alu_pipe_mode     = 1'b0;
      r_rx_fifo_read  = 0;
      r_break_active  = 0;
      r_break_counter = 0;
      r_var1_prefetched = 0;
      r_ir_valid = 1'b0;
      r_ir_presettled = 1'b0;
      r_FPC = 32'h0;
      r_trace_idx = 4'h0;
      r_trace_full = 1'b0;
      r_instr_count = 32'h0;
      r_ifb_dwval = 2'b0;
      r_perf_br_valid = 1'b0;
      r_perf_br_taken = 1'b0;
      r_int_push_wait_d = 1'b0;
      r_perf_cycles = 64'd0;       r_perf_instr = 64'd0;
      r_perf_fetch_cycles = 64'd0; r_perf_exec_cycles = 64'd0;
      r_perf_mul_cycles = 64'd0;   r_perf_div_cycles = 64'd0;
      r_perf_int_cycles = 64'd0;   r_perf_idle_cycles = 64'd0;
      r_perf_mul_ops = 64'd0;      r_perf_div_ops = 64'd0;
      r_perf_int_ops = 64'd0;
      r_perf_cnt_alu = 64'd0;      r_perf_cnt_load = 64'd0;
      r_perf_cnt_store = 64'd0;    r_perf_cnt_branch = 64'd0;
      r_perf_cnt_branch_taken = 64'd0; r_perf_cnt_jump = 64'd0;
      r_perf_cnt_call = 64'd0;     r_perf_cnt_indirect = 64'd0;
      r_perf_cnt_other = 64'd0;
      r_perf_fastpath = 64'd0;
      r_fastpath_fired = 1'b0;
      r_hcf_dump_phase = 7'd0;
      r_hcf_dump_sub = 3'b000;
      r_hcf_dump_byte_pos = 5'd0;
      r_hcf_stack_loaded = 1'b0;
      r_hcf_stack_data = 64'b0;
      for (i = 0; i < 16; i = i + 1)
         r_trace_buf[i] = 64'b0;
   end

   always @(posedge i_Clk) begin
      if (w_reset_H) begin

         r_SM <= NO_PROGRAM;
         r_SP <= 32'h800_0000;
         r_boot_active   <= 1'b1;   // arm the resident boot-ROM copy
         r_boot_phase    <= 3'd0;
         r_boot_dw_addr  <= 15'd0;
         r_int_push_wait <= 1'b0;
         r_wb_pending    <= 1'b0;
         r_break_received <= 1'b0;
         for (i = 0; i < 16; i = i + 1)
            r_register[i] <= 64'b0;
         r_trace_idx <= 4'h0;
         r_trace_full <= 1'b0;
         r_instr_count <= 32'h0;
         r_ifb_dwval <= 2'b0;
         r_ir_valid <= 1'b0;
         r_ir_presettled <= 1'b0;
         r_FPC <= 32'h0;
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
      end
      // Command characters are only accepted after a break
      else if (w_uart_rx_DV & r_break_received) begin
         r_break_received <= 1'b0;  // consume the break — one command per break
         case (w_uart_rx_value)
            8'h53: begin // 'S' — load start
               r_SM <= LOADING_BYTE;
               r_load_byte_counter <= 0;
               o_ram_write_addr <= 32'h0;
               r_ram_next_write_addr <= 32'h0;
               r_checksum <= 16'h0;
               r_old_checksum <= 16'h0;
               r_RGB_LED_1 <= 12'h0;
               r_RGB_LED_2 <= 12'h0;
               r_led <= 16'h0;
               r_mem_write_DV <= 1'b0;
               r_mem_read_DV <= 1'b0;
            end
            8'h47: r_debug_flag      <= 1;  // 'G' — debug on
            8'h67: r_debug_flag      <= 0;  // 'g' — debug off
            8'h57: r_debug_step_flag <= 1;  // 'W' — step on
            8'h77: r_debug_step_flag <= 0;  // 'w' — step off
            8'h6E: r_debug_step_run  <= 1;  // 'n' — next step
            // Any other byte after break: silently ignored
         endcase
      end else begin
         r_msg_send_DV  <= 1'b0;
         r_rx_fifo_read <= 1'b0;
         r_perf_br_valid <= 1'b0;  // 1-cycle strobe; jump tasks re-assert it
         r_fastpath_fired <= 1'b0; // default; the fast-path dispatch below asserts it

         // Prefetch under execute: during a bus-idle execute tail, fill the
         // one-entry IR latch from the instruction buffer at r_FPC (this
         // instruction's sequential successor, set at the dispatch below /
         // OPCODE_FETCH2). Buffer-hit-only — no cache request — so it never
         // contends for the memory port; the !r_mem_*_DV guards exclude load/store
         // and store cycles. The execute-tail handoff then pre-loads r_reg_1/2 from
         // this latch so the fast-path dispatch can skip the FETCH2 bubble.
         if (w_exec_tail && !r_ir_valid && !r_mem_read_DV && !r_mem_write_DV
             && w_ifb_fpc_hit) begin
            r_ir_pc      <= r_FPC;
            r_ir_opcode  <= r_FPC[2] ? w_ifb_fpc_data[63:32] : w_ifb_fpc_data[31:0];
            r_ir_reg_1   <= r_FPC[2] ? w_ifb_fpc_data[39:36] : w_ifb_fpc_data[7:4];
            r_ir_reg_2   <= r_FPC[2] ? w_ifb_fpc_data[35:32] : w_ifb_fpc_data[3:0];
            r_ir_reg_dst <= r_FPC[2] ? w_ifb_fpc_data[43:40] : w_ifb_fpc_data[11:8];
            if (r_FPC[2] == 1'b0) begin
               r_ir_var1            <= w_ifb_fpc_data[63:32];  // var1 shares this dw
               r_ir_var1_prefetched <= 1'b1;
            end else if (w_ifb_fpcv1_hit) begin
               r_ir_var1            <= w_ifb_fpcv1_data[31:0]; // var1 in the next dw
               r_ir_var1_prefetched <= 1'b1;
            end else begin
               r_ir_var1_prefetched <= 1'b0;
            end
            r_ir_valid   <= 1'b1;
         end

         // Instruction-buffer coherence: a DRAM store into a buffered line
         // drops it so the next fetch re-reads the modified bytes (self-
         // modifying code / Zephyr LLEXT loader). MMIO stores (top nibble F)
         // can't alias code, so they're excluded.
         if (r_mem_write_DV && (r_mem_addr[31:28] != 4'hF)) begin
            if (r_ifb_dwval[0] && (r_ifb_dwaddr[0] == r_mem_addr[31:3]))
               r_ifb_dwval[0] <= 1'b0;
            if (r_ifb_dwval[1] && (r_ifb_dwaddr[1] == r_mem_addr[31:3]))
               r_ifb_dwval[1] <= 1'b0;
            // Self-modifying-code poison: a store into the latched instruction's
            // dword (or the next dword, where a spilled var1 lives) invalidates the
            // prefetched IR so stale code is never consumed (same discipline as the
            // instruction buffer above).
            if (r_ir_valid && ((r_ir_pc[31:3] == r_mem_addr[31:3]) ||
                               (r_ir_pc[31:3] + 1'b1 == r_mem_addr[31:3]))) begin
               r_ir_valid      <= 1'b0;
               r_ir_presettled <= 1'b0;
            end
         end

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

         //=====================================================================
         // MMIO write handler — fires when bus_splitter routes a CPU store
         // (LD/ST opcode → r_mem_write_DV) to an MMIO address (top nibble 'F).
         // Peripheral state regs are touched here AND by the legacy opcode
         // tasks (e.g. t_led_rgb1_value, t_7_seg1_reg). Both paths converge
         // on the same registers — no double-driver conflict because they
         // never fire on the same cycle (legacy opcodes run during
         // OPCODE_EXECUTE; MMIO writes complete during memory-task states
         // that do not otherwise touch these regs).
         //
         // Write handler runs BEFORE the FSM case statement, so any state
         // that assigns these regs explicitly (NO_PROGRAM boot animation,
         // LOADING_BYTE display, HCF blanking) overrides the MMIO write.
         // Address decode: addr[27:16]=device, addr[15:0]=register offset.
         // See doc/MMIO_MAP.md for the full memory map.
         //=====================================================================
         if (w_mmio_write_DV) begin
            case (w_mmio_addr[27:16])
               12'h002: begin  // RGB LEDs
                  case (w_mmio_addr[15:0])
                     16'h0000: r_RGB_LED_1 <= w_mmio_write_data[11:0];
                     16'h0008: r_RGB_LED_2 <= w_mmio_write_data[11:0];
                     default: ;
                  endcase
               end
               12'h003: begin  // 7-segment display
                  case (w_mmio_addr[15:0])
                     // SEG_LOW: 4 hex digits → lower display (value2)
                     16'h0000: r_seven_seg_value2 <= {
                        4'h0, w_mmio_write_data[15:12],
                        4'h0, w_mmio_write_data[11:8],
                        4'h0, w_mmio_write_data[7:4],
                        4'h0, w_mmio_write_data[3:0]
                     };
                     // SEG_HIGH: 4 hex digits → upper display (value1)
                     16'h0008: r_seven_seg_value1 <= {
                        4'h0, w_mmio_write_data[15:12],
                        4'h0, w_mmio_write_data[11:8],
                        4'h0, w_mmio_write_data[7:4],
                        4'h0, w_mmio_write_data[3:0]
                     };
                     // SEG_ALL: 8 hex digits across both displays
                     16'h0010: begin
                        r_seven_seg_value1 <= {
                           4'h0, w_mmio_write_data[31:28],
                           4'h0, w_mmio_write_data[27:24],
                           4'h0, w_mmio_write_data[23:20],
                           4'h0, w_mmio_write_data[19:16]
                        };
                        r_seven_seg_value2 <= {
                           4'h0, w_mmio_write_data[15:12],
                           4'h0, w_mmio_write_data[11:8],
                           4'h0, w_mmio_write_data[7:4],
                           4'h0, w_mmio_write_data[3:0]
                        };
                     end
                     // SEG_BLANK: any write blanks both displays
                     16'h0018: begin
                        r_seven_seg_value1 <= 32'h22222222;
                        r_seven_seg_value2 <= 32'h22222222;
                     end
                     default: ;
                  endcase
               end
               12'h004: begin  // 16-bit LED bar
                  case (w_mmio_addr[15:0])
                     16'h0000: r_led <= w_mmio_write_data[15:0];
                     default: ;
                  endcase
               end
               12'h00F: begin  // Interrupt controller / timer
                  case (w_mmio_addr[15:0])
                     16'h0000: r_int_mask           <= w_mmio_write_data[3:0];
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

         case (r_SM)
            NO_PROGRAM: begin
               r_seven_seg_value1 <= 32'h22222222;
               r_seven_seg_value2 <= 32'h22222222;

               if (r_boot_active) begin
                  // ---- Resident boot-ROM → DDR copy (sub-FSM in r_boot_phase) ----
                  // Marker on the upper display so a stuck copy is distinguishable
                  // from a blank idle board ("b00t").
                  r_seven_seg_value1 <= 32'h0B000704;
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
                        r_mem_addr       <= {14'd0, r_boot_dw_addr, 3'b0};
                        r_mem_write_data <= w_boot_dword;
                        r_mem_byte_en    <= 8'hFF;
                        r_mem_write_DV   <= 1'b1;
                        r_boot_phase     <= 3'd4;
                     end
                     3'd4: begin
                        if (w_mem_ready) begin
                           r_mem_write_DV <= 1'b0;
                           if (r_boot_dw_addr == r_boot_len_dw - 1'b1) begin
                              // Hand off to the normal run path.  Equal checksums
                              // bypass LOAD_COMPLETE's verify; entry = 0x20.
                              r_boot_active   <= 1'b0;
                              r_PC_requested  <= 32'h0000_0020;
                              r_calc_checksum <= 16'h0;
                              r_rec_checksum  <= 16'h0;
                              r_SP            <= 32'h800_0000;
                              r_SM            <= LOAD_COMPLETE;
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
                        r_RGB_LED_1  <= 12'h010;
                        r_RGB_LED_2  <= 12'h100;
                        //o_led[0]<=1;
                        r_boot_flash <= 1;
                     end
                     default: begin
                        r_RGB_LED_1  <= 12'h100;
                        r_RGB_LED_2  <= 12'h010;
                        //o_led[0]<=0;
                        r_boot_flash <= 0;
                     end
                  endcase
               end
            end

            LOADING_BYTE: begin

               if (w_mem_ready) begin
                  r_mem_write_DV <= 1'b0;
               end
               r_SP <= 32'h800_0000;  // reset stack pointer during program load
               r_int_push_wait <= 1'b0;

               r_seven_seg_value1 <= {
                  8'h24,
                  4'h0,
                  r_ram_next_write_addr[27:24],
                  4'h0,
                  r_ram_next_write_addr[23:20],
                  4'h0,
                  r_ram_next_write_addr[19:16]
               };
               r_seven_seg_value2 <= {
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
                           r_SM <= LOAD_COMPLETE;
                           r_calc_checksum<=r_old_checksum+o_ram_write_addr[17:2]*2+o_ram_write_value[31:16]; //adding number of words to checksum (addr>>2=word count)
                           r_rec_checksum <= o_ram_write_value[15:0];
                           o_ram_write_value <= 32'h0;

                        end // (r_load_byte_counter==0)
                            else
                            begin
                           r_SM <= HCF_1;  // Halt and catch fire error
                           r_error_code <= ERR_DATA_LOAD;
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
                           case (r_RGB_LED_1)
                              12'h050: r_RGB_LED_1 <= 12'h005;
                              default: r_RGB_LED_1 <= 12'h050;
                           endcase
                           o_ram_write_addr <= r_ram_next_write_addr;
                           r_ram_next_write_addr <= r_ram_next_write_addr + 4;  // byte addr: 4 bytes per word
                           if (r_ram_next_write_addr>32'h7FF_FFFC) // Nexys has 128 MiB DDR2, last valid word at byte addr 0x7FF_FFFC
                                begin
                              r_SM <= HCF_1;  // Halt and catch fire error
                              r_error_code <= ERR_OVERFLOW;
                           end
                           r_mem_addr <= r_ram_next_write_addr;
                           // Place 32-bit word in the correct half of the 64-bit doubleword.
                           // Little-endian layout: addr[2]==0 → LOW half [31:0]; addr[2]==1 → HIGH half [63:32].
                           if (r_ram_next_write_addr[2] == 1'b0) begin
                              r_mem_write_data <= {32'b0, o_ram_write_value};
                              r_mem_byte_en    <= 8'h0F;
                           end else begin
                              r_mem_write_data <= {o_ram_write_value, 32'b0};
                              r_mem_byte_en    <= 8'hF0;
                           end
                           r_mem_write_DV <= 1'b1;

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
               r_seven_seg_value1 <= 32'h22222222;  // Blank 7 seg
               if (r_calc_checksum==r_rec_checksum) // Last value received should be checksum
                begin  // Reset all flags and jump to first instruction
                  o_LCD_reset_n <= 1'b0;
                  r_led <= 16'h0;
                  o_ram_write_addr <= 32'h0;
                  o_TX_LCD_Byte <= 8'b0;
                  o_TX_LCD_Count <= 4'd1;
                  r_carry_flag <= 1'b0;
                  r_debug_flag <= 1'b0;
                  r_debug_step_flag <= 1'b0;
                  r_debug_step_run <= 1'b0;
                  r_equal_flag <= 1'b0;
                  r_error_code <= 8'h0;
                  r_hcf_message_sent <= 1'b0;
                  r_interrupt_table[0] <= 32'h0;  // clear all 4 handler vectors;
                  r_interrupt_table[1] <= 32'h0;  // a 0 vector disables that source
                  r_interrupt_table[2] <= 32'h0;
                  r_interrupt_table[3] <= 32'h0;
                  r_msg_send_DV <= 1'b0;
                  r_overflow_flag <= 1'b0;
                  r_PC <= r_PC_requested;
                  r_mem_byte_en <= 8'hFF;  // restore full-doubleword default after loader partial writes
                  r_ram_next_write_addr <= 32'h0;
                  r_RGB_LED_1 <= 12'h000;
                  r_RGB_LED_2 <= 12'h000;
                  r_seven_seg_value1 <= 32'h22_22_22_22;
                  r_seven_seg_value2 <= 32'h22_22_22_22;
                  r_SM <= START_WAIT;
                  r_timeout_counter <= 0;
                  r_timer_interrupt <= 0;
                  r_timer_interrupt_counter <= 0;
                  r_int_mask <= 4'h0;            // all sources masked until program enables
                  r_timer_period <= 32'h000F_FFFF;  // default ~10.5 ms @ 100 MHz
                  r_instr_count <= 32'h0;        // reset committed-instruction counter for the new run
                  r_ifb_dwval <= 2'b0;           // drop any buffered code from the previous program
                  r_FPC <= r_PC_requested;       // prefetch pointer starts at the entry point
                  r_ir_valid <= 1'b0;            // no prefetched instruction yet
                  r_ir_presettled <= 1'b0;
                  r_timing_start <= 0;
                  r_zero_flag <= 0;
                  t_tx_message(8'd1);  // Load OK message
               end else begin
                  r_SM <= HCF_1;  // Halt and catch fire error
                  r_error_code <= ERR_CHECKSUM_LOAD;
                  t_tx_message(8'd2);  // Load error message
               end
            end

            // Delay to enable load message to be sent before starting
            START_WAIT: begin
               r_msg_send_DV <= 1'b0;
               if (r_start_wait_counter == 0) begin
                  r_SM <= OPCODE_REQUEST;
                  r_seven_seg_value1 <= 32'h22_22_22_22;
                  r_seven_seg_value2 <= 32'h22_22_22_22;
               end else begin
                  r_start_wait_counter <= r_start_wait_counter - 1;
                  r_seven_seg_value1 <= 32'h21_21_21_21;
                  r_seven_seg_value2 <= 32'h21_21_21_21;
               end
            end

            // Delay to enable load message to be sent before starting
            UART_DELAY: begin
               r_msg_send_DV <= 1'b0;
               if (!w_sending_msg) begin
                  r_SM <= OPCODE_REQUEST;
               end

            end

            OPCODE_REQUEST: begin
               r_msg_send_DV <= 1'b0;
               r_extra_clock <= 2'b0;  // always reset — all instructions rely on this
               r_tx_str_state_mem <= 3'b0;  // reset string transmission state machine (TXSTRMEM)
               r_tx_str_state_reg <= 3'b0;  // reset string transmission state machine (TXSTRMEMR)
               r_mem_byte_en <= 8'hFF;  // default full-word; byte ops override this

               // Execute-occupancy fusion: commit the previous
               // instruction's register writeback here, in parallel with the
               // fetch dispatch below. r_register and the dispatch's
               // r_mem_addr/r_PC are disjoint resources, and the register read
               // ports are sampled at OPCODE_FETCH2 (after this write), so the
               // next instruction observes the result with no RAW hazard. This
               // removes the dedicated WRITEBACK cycle (~ -1 cyc/instr on the
               // writeback-producing op classes).
               if (r_wb_pending) begin
                  r_register[r_writeback_reg] <= r_writeback_value;
                  if (r_writeback_set_zero_flag)
                     r_zero_flag <= (r_writeback_value == 64'b0);
                  r_writeback_set_zero_flag <= 1'b0;
                  r_wb_pending <= 1'b0;
               end

               if (r_int_push_wait) begin
                  // Waiting for DDR2 to finish the timer-interrupt PC push
                  if (w_mem_ready) begin
                     r_mem_write_DV  <= 1'b0;
                     r_int_push_wait <= 1'b0;
                     r_mem_addr      <= r_PC;  // r_PC already set to interrupt target
                     r_mem_read_DV   <= 1'b1;
                     r_SM            <= OPCODE_FETCH;
                  end
               end else if (w_irq_ready) begin
                  // Start pushing current PC + flags + mask onto DDR2 stack before jumping to handler.
                  // Slot layout (64-bit doubleword):
                  //   [63:43] = 0
                  //   [42:39] = r_int_mask (per-source enables, restored by IRET)
                  //   [38]    = zero,    [37] = equal,  [36] = carry,
                  //   [35]    = overflow,[34] = sign,   [33] = less, [32] = ult
                  //   [31:0]  = PC (resume address)
                  r_SP             <= r_SP - 8;
                  r_mem_addr       <= r_SP - 32'd8;
                  // Zero flag pushed as it will be AFTER any deferred writeback
                  // committing this same cycle (r_wb_pending block above), so the
                  // saved interrupt context stays precise.
                  r_mem_write_data <= {21'b0, r_int_mask,
                                       ((r_wb_pending && r_writeback_set_zero_flag) ? (r_writeback_value == 64'b0) : r_zero_flag),
                                       r_equal_flag, r_carry_flag,
                                       r_overflow_flag, r_sign_flag, r_less_flag, r_ult_flag,
                                       r_PC};
                  r_mem_byte_en    <= 8'hFF;
                  r_mem_write_DV   <= 1'b1;
                  // Source-selected dispatch (timer=0 priority, blitter=1). The
                  // pushed mask above is the pre-dispatch r_int_mask, so IRET
                  // re-enables this source. Timer pending is hardware-cleared
                  // here; the blitter is level/sticky and acked by the ISR
                  // (write STATUS.DONE W1C) before IRET.
                  if (w_irq_sel == 2'd0) r_timer_interrupt <= 1'b0;
                  r_int_mask[w_irq_sel] <= 1'b0;  // mask this source while handler runs; IRET restores
                  r_PC             <= r_interrupt_table[w_irq_sel];
                  r_int_push_wait  <= 1'b1;
                  r_ir_valid       <= 1'b0;   // discard the speculative fall-through
                  r_ir_presettled  <= 1'b0;   // (IRQ redirect; precise interrupts preserved)
                  // stay in OPCODE_REQUEST until push completes
               end else if (r_ir_presettled && r_ir_valid && r_ir_pc == r_PC) begin
                  // Fast-path dispatch: the prefetched IR was pre-settled into
                  // r_reg_1/2 during the previous instruction's execute tail, so the
                  // reg-file read ports are already valid — skip the OPCODE_FETCH2
                  // settle bubble and go straight to EXECUTE (saves 1 cyc/instr).
                  // Ordered AFTER w_irq_ready so a pending interrupt always wins.
                  r_opcode_mem      <= r_ir_opcode;
                  r_reg_dst         <= r_ir_reg_dst;
                  r_var1_mem        <= r_ir_var1;
                  r_var1_prefetched <= r_ir_var1_prefetched;
                  // Crash trace + retire counter: FETCH2 (their usual home) is skipped
                  // on this path, so the unique commit gate moves here.
                  r_trace_buf[r_trace_idx] <= {r_PC, r_ir_opcode};
                  r_trace_idx              <= r_trace_idx + 4'd1;
                  if (r_trace_idx == 4'd15) r_trace_full <= 1'b1;
                  r_instr_count <= r_instr_count + 32'd1;
                  r_FPC <= r_PC + f_predecode_len(r_ir_opcode);  // next sequential prefetch
                  r_fastpath_fired <= 1'b1;  // a retired instruction that skips FETCH2 (counts in r_perf_instr + r_perf_fastpath)
                  r_ir_valid      <= 1'b0;
                  r_ir_presettled <= 1'b0;
                  if (r_ir_var1_prefetched) begin
                     if (r_debug_flag && r_ir_opcode[31:12] != 20'h0000F)
                        r_SM <= DEBUG_DATA;
                     else
                        r_SM <= OPCODE_EXECUTE;
                  end else begin
                     r_SM          <= VAR1_FETCH;   // var1 not buffered — fetch it (also settles)
                     r_mem_addr    <= (r_PC + 4);
                     r_mem_read_DV <= 1'b1;
                  end
               end else if (w_ifb_op_hit) begin
                  // Instruction-buffer hit: serve the opcode now and skip the
                  // OPCODE_FETCH cache round-trip. var1 (PC+4) is prefetched
                  // from the buffer when available; otherwise OPCODE_FETCH2
                  // falls through to VAR1_FETCH exactly as on a non-prefetch.
                  r_opcode_mem <= r_PC[2] ? w_ifb_op_dw[63:32] : w_ifb_op_dw[31:0];
                  // Latch register fields a cycle early — the opcode is available
                  // combinationally on an IFB hit, so the registered reg-file
                  // reads settle during OPCODE_FETCH2 and the VAR1_FETCH2 bubble
                  // is no longer needed (fetch sequencing 3 cycles -> 2).
                  r_reg_1   <= r_PC[2] ? w_ifb_op_dw[39:36] : w_ifb_op_dw[7:4];
                  r_reg_2   <= r_PC[2] ? w_ifb_op_dw[35:32] : w_ifb_op_dw[3:0];
                  r_reg_dst <= r_PC[2] ? w_ifb_op_dw[43:40] : w_ifb_op_dw[11:8];
                  if (r_PC[2] == 1'b0) begin
                     r_var1_mem        <= w_ifb_op_dw[63:32];  // var1 shares this dw
                     r_var1_prefetched <= 1'b1;
                  end else if (w_ifb_v1_hit) begin
                     r_var1_mem        <= w_ifb_v1_dw[31:0];   // var1 in the next buffered dw
                     r_var1_prefetched <= 1'b1;
                  end else begin
                     r_var1_prefetched <= 1'b0;
                  end
                  r_ir_valid      <= 1'b0;   // drop the prefetch (served from the buffer, not pre-settled)
                  r_ir_presettled <= 1'b0;
                  r_SM <= OPCODE_FETCH2;
               end else begin
                  r_ir_valid      <= 1'b0;   // drop any stale prefetch on a cache-miss fetch
                  r_ir_presettled <= 1'b0;
                  r_mem_addr    <= r_PC;
                  r_mem_read_DV <= 1'b1;
                  r_SM          <= OPCODE_FETCH;
               end
            end

            OPCODE_FETCH: begin
               if (w_mem_ready) begin
                  // PC[2] selects which 32-bit half of the 64-bit doubleword holds the opcode.
                  // Little-endian layout:
                  //   [31:0]  = bytes at the doubleword-aligned base address  (PC[2]==0)
                  //   [63:32] = bytes at base+4                               (PC[2]==1)
                  r_opcode_mem  <= r_PC[2] ? w_mem_read_data[63:32]
                                           : w_mem_read_data[31:0];
                  // Latch register fields here (a cycle earlier than the old
                  // OPCODE_FETCH2) so the registered reg reads settle by the time
                  // OPCODE_FETCH2 hands off to OPCODE_EXECUTE — no VAR1_FETCH2.
                  r_reg_1   <= r_PC[2] ? w_mem_read_data[39:36] : w_mem_read_data[7:4];
                  r_reg_2   <= r_PC[2] ? w_mem_read_data[35:32] : w_mem_read_data[3:0];
                  r_reg_dst <= r_PC[2] ? w_mem_read_data[43:40] : w_mem_read_data[11:8];
                  r_mem_read_DV <= 1'b0;
                  if (r_PC[2] == 0) begin
                     // var1 (at PC+4) is in the HIGH half of the same doubleword — always here.
                     r_var1_mem        <= w_mem_read_data[63:32];
                     r_var1_prefetched <= 1'b1;
                  end else if (w_mem_next_valid) begin
                     // var1 (at PC+4) is in the next doubleword's low half.
                     r_var1_mem        <= w_mem_read_data_next[31:0];
                     r_var1_prefetched <= 1'b1;
                  end else begin
                     r_var1_prefetched <= 1'b0;
                  end
                  // Refill the instruction buffer from the returned line so the
                  // following sequential fetches hit it. The cache exposes the
                  // requested doubleword (w_mem_read_data) and, when valid, the
                  // line companion (w_mem_read_data_next, addr bit 3 toggled).
                  r_ifb_dw[0]     <= w_mem_read_data;
                  r_ifb_dwaddr[0] <= r_mem_addr[31:3];
                  r_ifb_dwval[0]  <= 1'b1;
                  if (w_mem_next_valid) begin
                     r_ifb_dw[1]     <= w_mem_read_data_next;
                     r_ifb_dwaddr[1] <= {r_mem_addr[31:4], ~r_mem_addr[3]};
                     r_ifb_dwval[1]  <= 1'b1;
                  end else begin
                     r_ifb_dwval[1]  <= 1'b0;
                  end
                  r_SM <= OPCODE_FETCH2;
               end  // if ready asserted, else will loop until ready
            end

            OPCODE_FETCH2: begin
               // Capture {PC, opcode} into the crash-dump trace ring exactly once
               // per dispatched fetch.  OPCODE_FETCH2 is the unique "instruction
               // committed for execution" gate (it precedes every path into
               // OPCODE_EXECUTE, including the debug-step and interrupt-handler
               // paths), so this gives one entry per executed instruction.
               r_trace_buf[r_trace_idx] <= {r_PC, w_opcode};
               r_trace_idx              <= r_trace_idx + 4'd1;
               if (r_trace_idx == 4'd15)
                  r_trace_full <= 1'b1;
               // Bump the committed-instruction counter once per fetch (this is
               // the unique commit gate). 32-bit wrap is ~4.3e9 — irrelevant
               // for crash diagnostics.
               r_instr_count <= r_instr_count + 32'd1;
               // Advance the prefetch pointer to this instruction's sequential
               // successor (the fast path does this in OPCODE_REQUEST instead). The
               // predecode is proven exact, so r_FPC tracks the fall-through; a
               // control-flow redirect just makes the next dispatch recompute it.
               r_FPC <= r_PC + f_predecode_len(w_opcode);
               // Register fields (r_reg_1/2/dst) were latched a cycle earlier in
               // OPCODE_REQUEST (IFB hit) or OPCODE_FETCH (miss), so r_reg_port_a/b
               // are already settling — the old VAR1_FETCH2 bubble is gone.
               if (r_var1_prefetched) begin
                  // var1 already in r_var1_mem — go straight to execute.
                  if (r_debug_flag && w_opcode[31:12] != 20'h0000F) begin
                     r_SM <= DEBUG_DATA;
                  end else begin
                     r_SM <= OPCODE_EXECUTE;
                  end
               end else begin
                  r_SM          <= VAR1_FETCH;
                  r_mem_addr    <= (r_PC + 4);
                  r_mem_read_DV <= 1'b1;
               end
            end

            // VAR1_FETCH2 removed: register fields are now latched a cycle
            // earlier (OPCODE_REQUEST / OPCODE_FETCH), so the registered reg
            // reads no longer need a dedicated settle bubble before EXECUTE.

            VAR1_FETCH: begin
               if (w_mem_ready) begin
                  r_var1_mem<=w_mem_read_data[31:0]; // lower 32 bits = instruction word at this address (little-endian, PC[2]==0)
                  // Refill the instruction buffer from this returned line, same
                  // as OPCODE_FETCH does.  VAR1_FETCH only runs when the
                  // immediate at PC+4 fell outside the buffered/prefetched
                  // doubleword (PC at the last word of a line), so the line
                  // returned here is the NEXT code line — exactly what the
                  // following fetch at PC+8 needs.  Without this refill that
                  // fetch pays a second full cache round-trip for data the CPU
                  // just had in its hands.  r_mem_addr still holds PC+4 (set in
                  // OPCODE_FETCH2), so the tagging matches OPCODE_FETCH's
                  // refill verbatim.
                  r_ifb_dw[0]     <= w_mem_read_data;
                  r_ifb_dwaddr[0] <= r_mem_addr[31:3];
                  r_ifb_dwval[0]  <= 1'b1;
                  if (w_mem_next_valid) begin
                     r_ifb_dw[1]     <= w_mem_read_data_next;
                     r_ifb_dwaddr[1] <= {r_mem_addr[31:4], ~r_mem_addr[3]};
                     r_ifb_dwval[1]  <= 1'b1;
                  end else begin
                     r_ifb_dwval[1]  <= 1'b0;
                  end
                  if (r_debug_flag&&w_opcode[31:12]!=20'h0000F) begin  // Ignore delay/NOP opcodes (0x0000_F???)
                     r_SM <= DEBUG_DATA;
                  end else begin
                     r_SM <= OPCODE_EXECUTE;
                  end
                  r_mem_read_DV <= 1'b0;

               end  // if ready asserted, else will loop until ready
            end


            DEBUG_DATA: begin
               t_debug_message;
               r_SM <= DEBUG_DATA2;
            end

            DEBUG_DATA2: begin
               r_msg_send_DV <= 1'b0;
               r_SM <= DEBUG_DATA3;
            end

            DEBUG_DATA3: begin
               if (!w_sending_msg) begin
                  r_SM <= OPCODE_EXECUTE;
                  if (r_debug_step_flag == 1'b1) begin
                     r_SM <= DEBUG_WAIT;
                  end else begin
                     r_SM <= OPCODE_EXECUTE;
                  end
               end
            end

            DEBUG_WAIT: begin
               if (r_debug_step_run == 1'b1) begin
                  r_debug_step_run <= 1'b0;
                  r_SM <= OPCODE_EXECUTE;
               end
            end

            OPCODE_EXECUTE: begin
               t_opcode_select;
            end  // case OPCODE_EXECUTE

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
                  r_SM               <= HCF_DUMP;
               end else begin
                  r_timeout_counter <= 0;
                  r_SM              <= HCF_2;
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
                           // (r_SP+offset >= STACK_TOP).  Substitute an FFs
                           // sentinel and mark loaded so the next PREP emits
                           // the line directly.  Prevents OOB DDR2 access when
                           // the stack is empty (r_SP at initial 0x0800_0000).
                           if ((r_SP + ({25'b0, r_hcf_dump_phase - DUMP_STACK_BASE} << 3))
                                 >= STACK_TOP) begin
                              r_hcf_stack_data   <= 64'hFFFF_FFFF_FFFF_FFFF;
                              r_hcf_stack_loaded <= 1'b1;
                           end else begin
                              r_mem_addr     <= r_SP +
                                 ({25'b0, r_hcf_dump_phase - DUMP_STACK_BASE} << 3);
                              r_mem_read_DV  <= 1'b1;
                              r_hcf_dump_sub <= 3'b011;
                           end
                        end else if (r_hcf_dump_phase == DUMP_V1H && !r_hcf_stack_loaded) begin
                           // Read DRAM at PC+8 to recover the hi32 of a 64-bit
                           // immediate (V64 encoding: lo32 at PC+4, hi32 at PC+8).
                           r_mem_addr     <= r_PC + 32'd8;
                           r_mem_read_DV  <= 1'b1;
                           r_hcf_dump_sub <= 3'b011;
                        end else if (r_hcf_dump_phase == DUMP_OPCM && !r_hcf_stack_loaded) begin
                           // Re-read DRAM at PC.  If this disagrees with OPC,
                           // the opcode cache is incoherent with DRAM.
                           r_mem_addr     <= r_PC;
                           r_mem_read_DV  <= 1'b1;
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
                        r_mem_read_DV      <= 1'b0;
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
                           r_timeout_counter <= 0;
                           r_hcf_dump_sub    <= 3'b110;
                        end
                     end
                  end

                  // PRE_DISPLAY — hold for 3 seconds (300M cycles @ 100 MHz)
                  // after the UART crash dump completes so the operator can
                  // read whatever was on the 7-seg at the moment of the fault
                  // before HCF_2 overwrites it with the error display.
                  3'b110: begin
                     if (r_timeout_counter >= 45'd300_000_000) begin
                        r_timeout_counter <= 0;
                        r_SM              <= HCF_2;
                     end else begin
                        r_timeout_counter <= r_timeout_counter + 1;
                     end
                  end

                  default: r_hcf_dump_sub <= 3'b000;
               endcase
            end

            HCF_2: begin
               r_seven_seg_value1[31:8] <= 24'h230C0F;
               r_seven_seg_value1[7:0] <= r_error_code;
               r_seven_seg_value2 <= 32'h22_22_22_22;
               r_timeout_max <= 32'd100_000_000;
               if (r_timeout_counter >= r_timeout_max) begin
                  r_timeout_counter <= 0;
                  r_SM <= HCF_3;
               end  // if(r_timeout_counter>=DELAY_TIME)
                else
                begin
                  r_timeout_counter <= r_timeout_counter + 1;
               end  // else if(r_timeout_counter>=DELAY_TIME)
            end
            HCF_3: begin
               r_timeout_counter <= 0;
               r_SM <= HCF_4;
               r_error_display_type <= ~r_error_display_type;
            end
            HCF_4: begin
               if (r_error_display_type) begin
                  // Error codes 0x1..0x9 — see ERR_* localparams above (ERR_INV_OPCODE .. ERR_TRAP).

                  case (r_error_code)
                     ERR_CHECKSUM_LOAD:
                     // incoming checksum
                     r_seven_seg_value1 <= {
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
                        r_seven_seg_value1 <= {
                           8'h24,
                           8'h24,
                           4'h0,
                           r_ram_next_write_addr[23:20],
                           4'h0,
                           r_ram_next_write_addr[19:16]
                        };
                        r_seven_seg_value2 <= {
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
                        r_seven_seg_value1 <= {8'h22, 8'h22, 4'h0, r_PC[23:20], 4'h0, r_PC[19:16]};
                        r_seven_seg_value2 <= {
                           4'h0, r_PC[15:12], 4'h0, r_PC[11:8], 4'h0, r_PC[7:4], 4'h0, r_PC[3:0]
                        };
                     end


                  endcase
               end   // if (r_error_display_type)
                else
                begin

                  case (r_error_code)
                     ERR_CHECKSUM_LOAD:
                     // Calculated checksum
                     r_seven_seg_value1 <= {
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
                        r_seven_seg_value1 <= 32'h22_22_22_22;
                        r_seven_seg_value2 <= {8'h22, 8'h22, 8'h22, 6'h0, r_load_byte_counter[1:0]};
                     end
                     default // Also for opcode 1
                        // Show the full 32-bit opcode across both displays:
                        //   7seg1 = w_opcode[31:16] (upper 4 hex digits)
                        //   7seg2 = w_opcode[15:0]  (lower 4 hex digits)
                     begin
                        r_seven_seg_value1 <= {
                           4'h0,
                           w_opcode[31:28],
                           4'h0,
                           w_opcode[27:24],
                           4'h0,
                           w_opcode[23:20],
                           4'h0,
                           w_opcode[19:16]
                        };
                        r_seven_seg_value2 <= {
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

               r_timeout_max <= 32'd100_000_000;
               if (r_timeout_counter >= r_timeout_max) begin
                  r_timeout_counter <= 0;
                  r_SM <= HCF_1;
               end  // if(r_timeout_counter>=DELAY_TIME)
                else
                begin
                  r_timeout_counter <= r_timeout_counter + 1;
               end  // else if(r_timeout_counter>=DELAY_TIME)

            end
            
             MULTIPLY_SETUP: begin
    // Operands now valid in r_mul_operand_a/b; this cycle they propagate
    // into r_mul_operand_a_q/b_q (absorbed into DSP48E1 AREG/BREG).
    r_SM <= MULTIPLY_BREG;
end

MULTIPLY_BREG: begin
    // Wait for DSP input registers (AREG/BREG) - multiply now starting
    r_SM <= MULTIPLY_CALC;
end

MULTIPLY_CALC: begin
    // Wait for pipeline stage 2 (MREG) - multiply is computed
    // by the free-running pipeline from r_mul_operand_a_q/b_q
    r_SM <= MULTIPLY_PIPE;
end

MULTIPLY_PIPE: begin
    // Wait for pipeline stage 3 (PREG) - result now in r_mul_result_hi/lo
    r_SM <= MULTIPLY_WRITEBACK;
end

MULTIPLY_WRITEBACK: begin
    // Stage result into writeback pipeline
    if (r_mul_is_high)
        r_writeback_value <= r_mul_result_hi;
    else
        r_writeback_value <= r_mul_result_lo;
    r_writeback_reg <= r_mul_dest_reg;

    // Flags from registered values
    if (r_mul_is_high) begin
        r_zero_flag     <= (r_mul_result_hi == 64'b0);
        r_sign_flag     <= r_mul_result_hi[63];
        r_overflow_flag <= 1'b0;
    end else begin
        r_zero_flag     <= (r_mul_result_lo == 64'b0);
        r_sign_flag     <= r_mul_result_lo[63];
        if (r_mul_is_unsigned)
            r_overflow_flag <= (r_mul_result_hi != 64'b0);
        else
            r_overflow_flag <= (r_mul_result_hi != {64{r_mul_result_lo[63]}});
    end

    // PC increment depends on instruction type
    if (r_mul_is_immediate)
        r_PC <= r_PC + 8;
    else
        r_PC <= r_PC + 4;

    // Execute-tail handoff: pre-load r_reg_1/2 from the prefetched latch (the
    // multiply used r_mul_operand_*_q, not r_reg_1/2) so the next dispatch skips
    // FETCH2. r_PC becomes the successor this cycle; OPCODE_REQUEST's r_ir_pc==r_PC
    // guard confirms the match next cycle.
    if (r_ir_valid) begin
        r_reg_1         <= r_ir_reg_1;
        r_reg_2         <= r_ir_reg_2;
        r_ir_presettled <= 1'b1;
    end
    r_SM <= OPCODE_REQUEST; r_wb_pending <= 1'b1;
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
                     r_SM           <= HALTED;
                  end
               end
            end

            HALTED: begin
               // CPU halted - do nothing until reset
            end

            // Interruptible core-suspend (WAIT opcode). The core parks here
            // issuing no instruction fetches and no bus/MMIO activity (only the
            // free-running timer / clock_ms / perf counters keep ticking), until
            // an unmasked, vectored interrupt becomes pending. The wake is
            // LEVEL-sensitive on w_irq_ready and re-evaluated every cycle, so an
            // interrupt arriving on or after the WAIT cycle is never lost (this
            // is what makes the software idiom "enable-interrupts; WAIT" safe).
            // t_wait already advanced r_PC to the instruction after WAIT, so the
            // normal dispatch in OPCODE_REQUEST saves that PC and IRET resumes
            // past the WAIT. If an interrupt is already pending when WAIT runs,
            // this exits next cycle — it never sleeps with work pending.
            // CPU_RESETN (handled above) and UART load/command paths force-exit
            // like any other state.
            WAITING: begin
               if (w_irq_ready)
                  r_SM <= OPCODE_REQUEST;
            end

            // DIVIDE_PREP — skip the leading-zero iterations of restoring
            // division.  The div tasks land here (dividend already abs'd,
            // divisor checked non-zero); pre-shifting the dividend by clz and
            // starting the counter there is bit-identical to running those
            // iterations: each would shift a 0 into the remainder, fail the
            // trial subtract, and emit a 0 quotient bit.  Typical 32-bit
            // operands now iterate ~32 times instead of a fixed 64; small loop
            // counters take a handful.  DIVIDE_STEP's iteration datapath is
            // untouched (its trial-subtract carry chain is a known critical
            // path), and the CLZ + shifter here run register-to-register in
            // their own cycle.
            DIVIDE_PREP: begin : divide_prep
               reg [6:0] prep_clz;
               prep_clz = count_leading_zeros(r_div_dividend);
               if (prep_clz[6]) begin
                  // dividend == 0 (clz = 64): quotient 0, remainder 0 —
                  // go straight to DIVIDE_STEP's finish branch.
                  r_div_counter <= 7'd64;
               end else begin
                  r_div_dividend <= r_div_dividend << prep_clz[5:0];
                  r_div_counter  <= {1'b0, prep_clz[5:0]};
               end
               r_SM <= DIVIDE_STEP;
            end

            DIVIDE_STEP: begin
               // Shared division iteration - avoids re-evaluating opcode casez each cycle
               if (r_div_counter < 7'd64) begin
                  // Restoring division step — single subtract (w_div_trial),
                  // borrow-out (w_div_borrow) selects quotient bit and remainder.
                  if (!w_div_borrow) begin
                     r_div_remainder <= w_div_trial[63:0];
                     r_div_quotient  <= {r_div_quotient[62:0], 1'b1};
                  end
                  else begin
                     r_div_remainder <= w_div_shifted;
                     r_div_quotient  <= {r_div_quotient[62:0], 1'b0};
                  end
                  r_div_dividend <= {r_div_dividend[62:0], 1'b0};
                  r_div_counter <= r_div_counter + 1;
               end
               else begin
                  // Division complete - write result based on op type
                  if (r_div_op == DIV_OP_DIV) begin
                     if (r_div_is_signed && r_div_sign_q)
                        r_writeback_value <= ~r_div_quotient + 1;
                     else
                        r_writeback_value <= r_div_quotient;
                     r_zero_flag <= (r_div_quotient == 0) ? 1'b1 : 1'b0;
                  end
                  else begin  // DIV_OP_MOD
                     if (r_div_is_signed && r_div_sign_r)
                        r_writeback_value <= ~r_div_remainder + 1;
                     else
                        r_writeback_value <= r_div_remainder;
                     r_zero_flag <= (r_div_remainder == 0) ? 1'b1 : 1'b0;
                  end
                  r_writeback_reg <= r_div_dest_reg;
                  r_overflow_flag <= 1'b0;
                  r_div_op <= DIV_OP_NONE;
                  r_PC <= r_PC + (r_div_pc_inc ? 8 : 4);
                  // Execute-tail handoff: pre-load r_reg_1/2 from the prefetched
                  // latch (the divide used r_div_* regs, not r_reg_1/2) so the next
                  // dispatch skips FETCH2.
                  if (r_ir_valid) begin
                     r_reg_1         <= r_ir_reg_1;
                     r_reg_2         <= r_ir_reg_2;
                     r_ir_presettled <= 1'b1;
                  end
                  r_SM <= OPCODE_REQUEST; r_wb_pending <= 1'b1;
               end
            end

            WRITEBACK: begin
               r_register[r_writeback_reg] <= r_writeback_value;
               if (r_writeback_set_zero_flag)
                  r_zero_flag <= (r_writeback_value == 64'b0);
               r_writeback_set_zero_flag <= 1'b0;
               r_SM <= OPCODE_REQUEST;
            end

            //==================================================================
            // ALU_FINISH — second pipeline stage for arithmetic / compare ops.
            // Cycle 1 (the task in OPCODE_EXECUTE) registered the 64-bit
            // subtract / compare result + flags into r_alu_pipe_*. This stage
            // copies the intermediates to architectural state, then either
            // proceeds to WRITEBACK (ARITH ops, write rd) or directly back to
            // OPCODE_REQUEST (CMP ops, no rd). Mode bit selects.
            //==================================================================
            ALU_FINISH: begin
               if (r_alu_pipe_mode == 1'b0) begin       // ARITH
                  r_writeback_value <= r_alu_pipe_value;
                  r_carry_flag      <= r_alu_pipe_carry;
                  r_overflow_flag   <= r_alu_pipe_overflow;
                  r_sign_flag       <= r_alu_pipe_value[63];
                  r_SM              <= OPCODE_REQUEST; r_wb_pending <= 1'b1;
               end else begin                           // CMP
                  r_equal_flag <= r_alu_pipe_equal;
                  r_less_flag  <= r_alu_pipe_less;
                  r_ult_flag   <= r_alu_pipe_ult;
                  r_sign_flag  <= r_alu_pipe_value[63];
                  r_SM         <= OPCODE_REQUEST;
               end
               // Execute-tail handoff: the ALU already consumed its operands and
               // r_PC is the successor, so pre-load r_reg_1/2 from the prefetched
               // latch — the read ports settle now and the fast-path dispatch skips FETCH2.
               if (r_ir_valid) begin
                  r_reg_1         <= r_ir_reg_1;
                  r_reg_2         <= r_ir_reg_2;
                  r_ir_presettled <= 1'b1;
               end
            end

            default: r_SM <= HCF_1;  // loop in error
         endcase  // case(r_SM)
      end  // else if (w_reset_H)
   end  // always @(posedge i_Clk)

   //=========================================================================
   // Instruction classifier for the performance counters. Returns one
   // mutually-exclusive class per opcode, so the Tier-2 mix counters sum to
   // the retired-instruction count. mul/div/system/io/nop fall into PC_OTHER
   // (mul/div are broken out separately by the Tier-1 *_OPS counters). The
   // casez patterns mirror t_opcode_select (opcode_select.vh) exactly.
   //=========================================================================
   function [2:0] f_perf_class;
      input [31:0] op;
      begin
         casez (op)
            // Conditional branches (conditional jumps, absolute + PC-relative)
            32'h0000_1001, 32'h0000_1002, 32'h0000_1003, 32'h0000_1004,
            32'h0000_1005, 32'h0000_1006, 32'h0000_1007, 32'h0000_1008,
            32'h0000_1013, 32'h0000_1014, 32'h0000_1015, 32'h0000_1016,
            32'h0000_1017, 32'h0000_1018, 32'h0000_1019, 32'h0000_101A,
            32'h0000_101B, 32'h0000_101C,
            32'h0000_1031, 32'h0000_1032, 32'h0000_1033, 32'h0000_1034,
            32'h0000_1035, 32'h0000_1036, 32'h0000_1037, 32'h0000_1038,
            32'h0000_1039, 32'h0000_103A, 32'h0000_103B, 32'h0000_103C,
            32'h0000_103D, 32'h0000_103E, 32'h0000_103F, 32'h0000_1040:
               f_perf_class = PC_BRANCH;
            // Unconditional direct jumps
            32'h0000_1000, 32'h0000_1030:
               f_perf_class = PC_JUMP;
            // Direct + conditional calls
            32'h0000_1009, 32'h0000_100A, 32'h0000_100B, 32'h0000_100C,
            32'h0000_100D, 32'h0000_100E, 32'h0000_100F, 32'h0000_1010,
            32'h0000_1011, 32'h0000_1041:
               f_perf_class = PC_CALL;
            // Indirect / stack-target control transfers
            32'h0000_1012,                                   // RET
            32'h0000_102?,                                   // JMPR
            32'h0000_6011,                                   // IRET
            32'h0000_407?:                                   // CALLR
               f_perf_class = PC_INDIRECT;
            // Loads (incl. POP)
            32'h0000_0C??, 32'h0000_0E??,                    // LDIDX64 / LDIDX64R
            32'h0000_C0??, 32'h0000_C2??, 32'h0000_C4??,
            32'h0000_C6??, 32'h0000_C7??,                    // LDIDX32/16/8/8s/16s
            32'h0000_71??, 32'h0000_721?,                    // MEMREADRR / MEMREADR
            32'h0000_75??, 32'h0000_77??, 32'h0000_79??,
            32'h0000_7B??,                                   // MEMGET8/16/32/64
            32'h0000_FC??,                                   // LDIDX64A
            32'h0000_401?:                                   // POP
               f_perf_class = PC_LOAD;
            // Stores (incl. PUSH / PUSHV / PUSHV64)
            32'h0000_0D??,                                   // STIDX64
            32'h0000_C1??, 32'h0000_C3??, 32'h0000_C5??,     // STIDX32/16/8
            32'h0000_70??, 32'h0000_720?, 32'h0000_73??,     // MEMSET64RR / MEMSETR / STIDX64R
            32'h0000_74??, 32'h0000_76??, 32'h0000_78??,
            32'h0000_7A??,                                   // MEMSET8/16/32/64
            32'h0000_FD??,                                   // STIDX64A
            32'h0000_400?, 32'h0000_4020, 32'h0000_4060:     // PUSH / PUSHV / PUSHV64
               f_perf_class = PC_STORE;
            // ALU: arithmetic / logic / shift / rotate / compare / bit / moves
            32'h0001_0???, 32'h0002_0???, 32'h0003_0???, 32'h0004_0???,
            32'h0005_0???, 32'h0006_0???, 32'h0007_0???,     // basic arith (NOT 0010-0017 mul/div)
            32'h0020_0???, 32'h0021_0???, 32'h0022_0???,
            32'h0023_0???, 32'h0024_0???,                    // shift / rotate
            32'h0030_0???, 32'h0031_0???, 32'h0032_0???, 32'h0033_0???,
            32'h0034_0???, 32'h0035_0???, 32'h0036_0???, 32'h0037_0???,
            32'h0038_0???, 32'h0039_0???,                    // boolean compare
            32'h0040_0???, 32'h0041_0???, 32'h0042_0???, 32'h0043_0???, // min / max
            32'h0050_0???, 32'h0051_0???, 32'h0052_0???, 32'h0053_0???, // bit
            32'h0000_01??, 32'h0000_02??, 32'h0000_05??,     // COPY / ADDI / CMPRR
            32'h0000_08??,                                   // SETR..SHRAR block
            32'h0000_090?, 32'h0000_091?, 32'h0000_092?, 32'h0000_093?,
            32'h0000_094?, 32'h0000_095?, 32'h0000_096?, 32'h0000_097?,
            32'h0000_098?, 32'h0000_099?,                    // shifts / ext / bswap / not / leapc
            32'h0000_0A??,                                   // bit ops / popcnt / clz / ctz / bextr / bdep
            32'h0000_0F0?, 32'h0000_0F1?,                    // SEXTW / ZEXTW
            32'h0000_0F8?, 32'h0000_0F9?, 32'h0000_0FA?,
            32'h0000_0FB?, 32'h0000_0FC?, 32'h0000_0FD?,     // rotates
            32'h0000_0FE?:                                   // SETR64
               f_perf_class = PC_ALU;
            default:
               f_perf_class = PC_OTHER;
         endcase
      end
   endfunction


   //=========================================================================
   // Performance-counter block (Tier 0/1/2). Its own always block: reads
   // existing FSM state (r_SM, r_div_counter, r_int_push_wait), the decoded
   // opcode (w_opcode) and the branch-outcome strobe set by the jump tasks.
   // Writes only the r_perf_* counters, so it adds no logic to the main CPU
   // critical path. Free-running; PERF_CTRL bit 0 (or hard reset) clears all.
   //=========================================================================
   always @(posedge i_Clk) begin
      r_int_push_wait_d <= r_int_push_wait;
      if (w_reset_H || w_perf_stat_clear) begin
         r_perf_cycles           <= 64'd0;  r_perf_instr            <= 64'd0;
         r_perf_fetch_cycles     <= 64'd0;  r_perf_exec_cycles      <= 64'd0;
         r_perf_mul_cycles       <= 64'd0;  r_perf_div_cycles       <= 64'd0;
         r_perf_int_cycles       <= 64'd0;  r_perf_idle_cycles      <= 64'd0;
         r_perf_mul_ops          <= 64'd0;  r_perf_div_ops          <= 64'd0;
         r_perf_int_ops          <= 64'd0;
         r_perf_cnt_alu          <= 64'd0;  r_perf_cnt_load         <= 64'd0;
         r_perf_cnt_store        <= 64'd0;  r_perf_cnt_branch       <= 64'd0;
         r_perf_cnt_branch_taken <= 64'd0;  r_perf_cnt_jump         <= 64'd0;
         r_perf_cnt_call         <= 64'd0;  r_perf_cnt_indirect     <= 64'd0;
         r_perf_cnt_other        <= 64'd0;
         r_perf_fastpath           <= 64'd0;
      end else begin
         // Tier 0 — total cycles and retired instructions.
         // OPCODE_FETCH2 is the unique 1-cycle commit gate (mirrors r_instr_count).
         r_perf_cycles <= r_perf_cycles + 64'd1;
         if (r_SM == OPCODE_FETCH2 || r_fastpath_fired) r_perf_instr <= r_perf_instr + 64'd1;
         if (r_fastpath_fired) r_perf_fastpath <= r_perf_fastpath + 64'd1;  // fast-path-dispatch fire count (MMIO 0xA8)

         // Tier 1 — disjoint cycle buckets. The interrupt context-push happens
         // inside OPCODE_REQUEST (r_int_push_wait==1), so it is split out first
         // to keep the fetch bucket clean.
         if (r_int_push_wait)
            r_perf_int_cycles <= r_perf_int_cycles + 64'd1;
         else if (r_SM == OPCODE_REQUEST || r_SM == OPCODE_FETCH  ||
                  r_SM == OPCODE_FETCH2  || r_SM == VAR1_FETCH)
            r_perf_fetch_cycles <= r_perf_fetch_cycles + 64'd1;
         else if (r_SM == OPCODE_EXECUTE || r_SM == ALU_FINISH ||
                  r_SM == WRITEBACK)
            r_perf_exec_cycles <= r_perf_exec_cycles + 64'd1;
         else if (r_SM == MULTIPLY_SETUP || r_SM == MULTIPLY_BREG ||
                  r_SM == MULTIPLY_CALC  || r_SM == MULTIPLY_PIPE ||
                  r_SM == MULTIPLY_WRITEBACK)
            r_perf_mul_cycles <= r_perf_mul_cycles + 64'd1;
         else if (r_SM == DIVIDE_STEP || r_SM == DIVIDE_PREP)
            r_perf_div_cycles <= r_perf_div_cycles + 64'd1;
         else if (r_SM == HALTED || r_SM == HALTED_BREAK || r_SM == WAITING)
            r_perf_idle_cycles <= r_perf_idle_cycles + 64'd1;

         // Tier 1 — event counts. MULTIPLY_SETUP is a 1-cycle entry state (one
         // per multiply); DIVIDE_STEP with counter==0 is the first divide cycle.
         if (r_SM == MULTIPLY_SETUP)
            r_perf_mul_ops <= r_perf_mul_ops + 64'd1;
         // DIVIDE_PREP is the unique 1-cycle entry state per divide (the old
         // counter==0 test no longer fires once per op — prep starts the
         // counter at clz, and a zero dividend skips the iterations entirely).
         if (r_SM == DIVIDE_PREP)
            r_perf_div_ops <= r_perf_div_ops + 64'd1;
         if (r_int_push_wait && !r_int_push_wait_d)
            r_perf_int_ops <= r_perf_int_ops + 64'd1;

         // Tier 2 — conditional-branch taken outcome (strobe from jump tasks).
         // The matching BRANCH total is incremented from the classifier below,
         // so taken ≤ branch always holds.
         if (r_perf_br_valid && r_perf_br_taken)
            r_perf_cnt_branch_taken <= r_perf_cnt_branch_taken + 64'd1;

         // Tier 2 — instruction mix: one class per committed instruction.
         if (r_SM == OPCODE_FETCH2) begin
            case (f_perf_class(w_opcode))
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

         // (the passive predecode-mismatch validator was retired — superseded by the
         // r_ir_pc==r_PC consume guard, the functional regression, and the
         // fast-path fire-rate counter which collapses on any predecode error.)
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
      reg [6:0] clz_result;
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
      reg [6:0] ctz_result;
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

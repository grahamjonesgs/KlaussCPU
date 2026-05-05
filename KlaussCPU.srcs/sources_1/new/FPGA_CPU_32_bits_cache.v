
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
module FPGA_CPU_32_bits_cache (
    input             CPU_RESETN,        // CPU reset button
    input             i_Clk,             // FPGA Clock
    input             i_uart_rx,
    input             i_load_H,          // Load button
    output            o_uart_tx,
    output reg [15:0] o_led,
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

   localparam STACK_TOP = 32'h800_0000;  // one doubleword (8 bytes) above top of 128 MiB byte address space

   // State Machine Code
   localparam OPCODE_REQUEST = 32'h1, OPCODE_FETCH = 32'h2, OPCODE_FETCH2 = 32'h4;
   localparam VAR1_FETCH = 32'h8, VAR1_FETCH2 = 32'h10, VAR1_FETCH3 = 32'h20;
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

   // Error Codes
   localparam ERR_INV_OPCODE = 8'h1, ERR_INV_FSM_STATE = 8'h2, ERR_STACK = 8'h3;
   localparam ERR_DATA_LOAD = 8'h4, ERR_CHECKSUM_LOAD = 8'h5, ERR_OVERFLOW = 8'h6;
   localparam ERR_SEG_WRITE_TO_CODE = 'h7, ERR_SEG_EXEC_DATA = 'h8;
   localparam ERR_TRAP = 8'h9;        // Explicit software trap (TRAP opcode)

   // Crash dump phase boundaries (r_hcf_dump_phase). Each "phase" emits one UART line.
   localparam DUMP_HEADER     = 6'd0;
   localparam DUMP_ERR_PC     = 6'd1;
   localparam DUMP_OPC_SP     = 6'd2;
   localparam DUMP_V1_V2      = 6'd3;
   localparam DUMP_FLAGS_A    = 6'd4;   // Z E C V
   localparam DUMP_FLAGS_B    = 6'd5;   // S L U
   localparam DUMP_REG_BASE   = 6'd6;   // R0..RF → phases  6..21
   localparam DUMP_STACK_BASE = 6'd22;  // S0..S3 → phases 22..25 (each preceded by a DDR2 read)
   localparam DUMP_TRACE_BASE = 6'd26;  // T0..TF → phases 26..41 (newest-first)
   localparam DUMP_FOOTER     = 6'd42;  // last phase; on completion → HCF_2

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
   reg [32:0] r_SM;   // 33 bits: bit 32 is ALU_FINISH (added for 64-bit ALU pipeline)
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
   reg [31:0] r_idx_base_addr;  // Saved base address for indexed register ops (byte addr)
   reg r_hcf_message_sent;
   reg [31:0] r_start_wait_counter;

   // Crash-dump trace ring buffer — captures {PC, opcode} of every dispatched
   // fetch in OPCODE_FETCH2.  On HCF entry the most recent 16 entries (newest at
   // r_trace_idx-1) are flushed over UART so a crash log shows the branch-history
   // leading up to the failing instruction, not just the failing instruction itself.
   reg [63:0] r_trace_buf [0:15];
   reg [3:0]  r_trace_idx;          // next-write index, wraps freely
   reg        r_trace_full;         // 1 once the ring has wrapped at least once

   // Crash dump UART state machine.  Lives entirely inside HCF_DUMP; the sub-state
   // r_hcf_dump_sub walks each line through the canonical 4-step UART handshake
   // (PREP → ACK → DONE_WAIT) used elsewhere in this codebase, with an extra
   // STACK_FETCH branch for stack lines that need a DDR2 read first.
   reg [5:0]  r_hcf_dump_phase;     // which dump line to emit (see DUMP_* localparams)
   reg [2:0]  r_hcf_dump_sub;       // 000=PREP, 001=ACK, 010=DONE_WAIT, 011=STACK_FETCH, 100=BREAK
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
   reg r_mem_was_ready;
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
   reg  [63:0] r_mmio_read_data;
   wire        w_mmio_ready = 1'b1;   // simple regs ready instantly
   wire w_mem_ready;
   reg [31:0] r_opcode_mem;
   reg [31:0] r_var1_mem;
   reg [31:0] r_var2_mem;
   reg r_var1_prefetched; // 1 when r_var1_mem was populated from the opcode cache line

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
   
   // Dedicated read ports - registered every cycle
   reg [63:0] r_reg_port_a;
   reg [63:0] r_reg_port_b;

   // Writeback pipeline registers
   reg [63:0] r_writeback_value;
   reg [3:0]  r_writeback_reg;
   reg        r_writeback_set_zero_flag;  // Set zero flag from writeback value in WRITEBACK stage

    always @(posedge i_Clk) begin
       r_reg_port_a <= r_register[r_reg_1];
       r_reg_port_b <= r_register[r_reg_2];
   end

   // UART TX break generation — holds o_uart_tx low to signal program end
   wire       w_uart_tx_serial;   // internal serial output from uart_send_msg
   reg        r_break_active;     // when 1, overrides TX line to low (break)
   reg [11:0] r_break_counter;    // countdown: 2500 clocks ≈ 2.5 frames

   // Mux: break takes priority over normal TX; idle state is line-high
   assign o_uart_tx  = r_break_active ? 1'b0 : w_uart_tx_serial;
   assign w_reset_H  = !CPU_RESETN;

   // KEEP_HIERARCHY prevents Vivado from flattening these modules' logic into
   // surrounding CPU slices. Without it the placer can scatter sd_spi/splitter
   // cells across the CPU's 64-bit ALU carry chain, lengthening route delay on
   // an already-tight critical path (r_reg_port_b → r_carry_flag, ~25 levels).
   (* KEEP_HIERARCHY = "yes" *)
   bus_splitter bus_splitter_i (
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
       // MMIO side
       .o_mmio_write_DV(w_mmio_write_DV),
       .o_mmio_read_DV(w_mmio_read_DV),
       .o_mmio_addr(w_mmio_addr),
       .o_mmio_write_data(w_mmio_write_data),
       .o_mmio_byte_en(w_mmio_byte_en),
       .i_mmio_read_data(r_mmio_read_data),
       .i_mmio_ready(w_mmio_ready)
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
   // MMIO read mux — combinational; reads return current peripheral state.
   // Returns zero for undefined offsets (treat as scratch / write-only).
   // See MMIO_MAP.md for the full memory map.
   // -------------------------------------------------------------------------
   always @* begin
      r_mmio_read_data = 64'h0;
      case (w_mmio_addr[27:16])
         12'h000: r_mmio_read_data = w_sd_read_data;  // SD card
         12'h002: begin  // RGB LEDs
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data = {52'b0, r_RGB_LED_1};
               16'h0008: r_mmio_read_data = {52'b0, r_RGB_LED_2};
               default:  r_mmio_read_data = 64'h0;
            endcase
         end
         12'h003: begin  // 7-segment display (raw padded values)
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data = {32'b0, r_seven_seg_value2};
               16'h0008: r_mmio_read_data = {32'b0, r_seven_seg_value1};
               16'h0010: r_mmio_read_data = {r_seven_seg_value1, r_seven_seg_value2};
               default:  r_mmio_read_data = 64'h0;
            endcase
         end
         12'h004: begin  // LEDs (RW) and switches (RO)
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data = {48'b0, o_led};
               16'h0008: r_mmio_read_data = {48'b0, i_switch};
               default:  r_mmio_read_data = 64'h0;
            endcase
         end
         12'h00F: begin  // Interrupt controller / timer
            case (w_mmio_addr[15:0])
               16'h0000: r_mmio_read_data = {60'b0, r_int_mask};
               16'h0008: r_mmio_read_data = {63'b0, r_timer_interrupt};
               16'h0010: r_mmio_read_data = {32'b0, r_interrupt_table[0]};
               16'h0018: r_mmio_read_data = {32'b0, r_interrupt_table[1]};
               16'h0020: r_mmio_read_data = {32'b0, r_interrupt_table[2]};
               16'h0028: r_mmio_read_data = {32'b0, r_interrupt_table[3]};
               16'h0030: r_mmio_read_data = {32'b0, r_timer_period};
               16'h0038: r_mmio_read_data = {32'b0, r_timer_interrupt_counter};
               16'h0040: r_mmio_read_data = r_clock_ms;
               default:  r_mmio_read_data = 64'h0;
            endcase
         end
         default: r_mmio_read_data = 64'h0;
      endcase
   end

   mem_read_write mem_read_write (
       .i_Clk(i_Clk),
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
       .o_mem_ready(w_dram_ready)
   );


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
      o_led <= 16'h0;
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
      r_trace_idx = 4'h0;
      r_trace_full = 1'b0;
      r_hcf_dump_phase = 6'd0;
      r_hcf_dump_sub = 3'b000;
      r_hcf_stack_loaded = 1'b0;
      r_hcf_stack_data = 64'b0;
      for (i = 0; i < 16; i = i + 1)
         r_trace_buf[i] = 64'b0;
   end

   always @(posedge i_Clk) begin
      if (w_reset_H) begin

         r_SM <= NO_PROGRAM;
         r_SP <= 32'h800_0000;
         r_int_push_wait <= 1'b0;
         r_break_received <= 1'b0;
         for (i = 0; i < 16; i = i + 1)
            r_register[i] <= 64'b0;
         r_trace_idx <= 4'h0;
         r_trace_full <= 1'b0;
         r_hcf_dump_phase <= 6'd0;
         r_hcf_dump_sub <= 3'b000;
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
               o_led <= 16'h0;
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

         // Free-running millisecond clock — increments every 100_000 cycles
         // (1 ms at 100 MHz). Exposed read-only at MMIO 0xF00F_0040.
         if (r_clock_ms_div >= 17'd99_999) begin
            r_clock_ms_div <= 17'd0;
            r_clock_ms     <= r_clock_ms + 64'd1;
         end else begin
            r_clock_ms_div <= r_clock_ms_div + 17'd1;
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
                     16'h0000: o_led <= w_mmio_write_data[15:0];
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

               if (r_timer_interrupt_counter_sec == 0) begin
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
                  8'h22,
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
                        end  // else (r_load_byte_counter==3)
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
                        end // if (r_load_byte_counter==3)
                            else
                            begin
                           r_load_byte_counter <= r_load_byte_counter + 1;
                        end  // else if (r_load_byte_counter==3)
                     end  // case default
                  endcase  // w_uart_rx_value
               end
            end

            LOAD_COMPLETE: begin
               r_seven_seg_value1 <= 32'h22222222;  // Blank 7 seg
               if (r_calc_checksum==r_rec_checksum) // Last value received should be checksum
                begin  // Reset all flags and jump to first instruction
                  o_LCD_reset_n <= 1'b0;
                  o_led <= 16'h0;
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
                  r_clock_ms <= 64'h0;            // millisecond clock starts at 0 per program run
                  r_clock_ms_div <= 17'h0;
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

               if (r_int_push_wait) begin
                  // Waiting for DDR2 to finish the timer-interrupt PC push
                  if (w_mem_ready) begin
                     r_mem_write_DV  <= 1'b0;
                     r_int_push_wait <= 1'b0;
                     r_mem_addr      <= r_PC;  // r_PC already set to interrupt target
                     r_mem_read_DV   <= 1'b1;
                     r_SM            <= OPCODE_FETCH;
                  end
               end else if (r_timer_interrupt && r_interrupt_table[0] != 32'h0 && r_int_mask[0]) begin
                  // Start pushing current PC + flags + mask onto DDR2 stack before jumping to handler.
                  // Slot layout (64-bit doubleword):
                  //   [63:43] = 0
                  //   [42:39] = r_int_mask (per-source enables, restored by IRET)
                  //   [38]    = zero,    [37] = equal,  [36] = carry,
                  //   [35]    = overflow,[34] = sign,   [33] = less, [32] = ult
                  //   [31:0]  = PC (resume address)
                  r_SP             <= r_SP - 8;
                  r_mem_addr       <= r_SP - 32'd8;
                  r_mem_write_data <= {21'b0, r_int_mask,
                                       r_zero_flag, r_equal_flag, r_carry_flag,
                                       r_overflow_flag, r_sign_flag, r_less_flag, r_ult_flag,
                                       r_PC};
                  r_mem_byte_en    <= 8'hFF;
                  r_mem_write_DV   <= 1'b1;
                  r_timer_interrupt <= 1'b0;
                  r_int_mask[0]    <= 1'b0;       // mask source 0 while handler runs; IRET restores
                  r_PC             <= r_interrupt_table[0];
                  r_int_push_wait  <= 1'b1;
                  // stay in OPCODE_REQUEST until push completes
               end else begin
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
               r_reg_1   <= w_opcode[7:4];
               r_reg_2   <= w_opcode[3:0];
               r_reg_dst <= w_opcode[11:8];
               if (r_var1_prefetched) begin
                  // var1 already in r_var1_mem — skip memory fetch.
                  // VAR1_FETCH2 is a 1-cycle bubble so r_reg_port_a/b
                  // (registered reads) update before OPCODE_EXECUTE uses them.
                  r_SM <= VAR1_FETCH2;
               end else begin
                  r_SM          <= VAR1_FETCH;
                  r_mem_addr    <= (r_PC + 4);
                  r_mem_read_DV <= 1'b1;
               end
            end

            VAR1_FETCH2: begin
               // Pipeline bubble only — r_reg_port_a/b now valid.
               if (r_debug_flag && w_opcode[31:12] != 20'h0000F) begin
                  r_SM <= DEBUG_DATA;
               end else begin
                  r_SM <= OPCODE_EXECUTE;
               end
            end


            VAR1_FETCH: begin
               if (w_mem_ready) begin
                  r_var1_mem<=w_mem_read_data[31:0]; // lower 32 bits = instruction word at this address (little-endian, PC[2]==0)
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
                  r_hcf_dump_phase   <= 6'd0;
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
                         && (r_hcf_dump_phase <  DUMP_STACK_BASE + 6'd4)
                         && !r_hcf_stack_loaded) begin
                           // Skip DDR2 reads past the top of the stack region
                           // (r_SP+offset >= STACK_TOP).  Substitute an FFs
                           // sentinel and mark loaded so the next PREP emits
                           // the line directly.  Prevents OOB DDR2 access when
                           // the stack is empty (r_SP at initial 0x0800_0000).
                           if ((r_SP + ({26'b0, r_hcf_dump_phase - DUMP_STACK_BASE} << 3))
                                 >= STACK_TOP) begin
                              r_hcf_stack_data   <= 64'hFFFF_FFFF_FFFF_FFFF;
                              r_hcf_stack_loaded <= 1'b1;
                           end else begin
                              r_mem_addr     <= r_SP +
                                 ({26'b0, r_hcf_dump_phase - DUMP_STACK_BASE} << 3);
                              r_mem_read_DV  <= 1'b1;
                              r_hcf_dump_sub <= 3'b011;
                           end
                        end else begin
                           t_hcf_dump_build_line;
                           r_msg_send_DV  <= 1'b1;
                           r_hcf_dump_sub <= 3'b001;
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
                        // Leaving a stack phase invalidates the cached read so
                        // the next stack phase fetches fresh data.
                        if ((r_hcf_dump_phase >= DUMP_STACK_BASE)
                         && (r_hcf_dump_phase <  DUMP_STACK_BASE + 6'd4)) begin
                           r_hcf_stack_loaded <= 1'b0;
                        end
                        if (r_hcf_dump_phase == DUMP_FOOTER) begin
                           // After the footer line drains, drop into the BREAK
                           // sub-state to assert a UART break (line low for ~2.5
                           // frames) so a host parser sees an unambiguous
                           // end-of-dump marker — same pattern as HALTED_BREAK.
                           r_hcf_dump_phase <= 6'd0;
                           r_hcf_dump_sub   <= 3'b100;  // override default 000
                        end else begin
                           r_hcf_dump_phase <= r_hcf_dump_phase + 6'd1;
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

                  // BREAK — wait for the footer's UART transmission to fully
                  // drain, then hold the TX line low for 2500 clocks (~2.5 frame
                  // times at CLKS_PER_BIT=100) as a UART break.  This mirrors
                  // HALTED_BREAK so a host parser can treat the break as the
                  // unambiguous end-of-dump marker, the same way it does for
                  // a clean HALT.  Once the break completes we fall through to
                  // HCF_2 for the existing 7-seg error display loop.
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
                           r_SM              <= HCF_2;
                        end
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
                  // ERR_INV_OPCODE=8'h1, ERR_INV_FSM_STATE=8'h2, ERR_STACK=8'h3, ERR_DATA_LOAD=8'h4, ERR_CHECKSUM_LOAD=8'h5;

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

    r_SM <= WRITEBACK;
end

            HALTED_BREAK: begin
               // Wait for any in-flight TX to finish, then hold line low for
               // ~2.5 frames (2500 clocks at CLKS_PER_BIT=100) as a UART break,
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

            DIVIDE_STEP: begin
               // Shared division iteration - avoids re-evaluating opcode casez each cycle
               if (r_div_counter < 7'd64) begin
                  // Restoring division step
                  if ({r_div_remainder[62:0], r_div_dividend[63]} >= r_div_divisor) begin
                     r_div_remainder <= {r_div_remainder[62:0], r_div_dividend[63]} - r_div_divisor;
                     r_div_quotient <= {r_div_quotient[62:0], 1'b1};
                  end
                  else begin
                     r_div_remainder <= {r_div_remainder[62:0], r_div_dividend[63]};
                     r_div_quotient <= {r_div_quotient[62:0], 1'b0};
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
                  r_SM <= WRITEBACK;
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
                  r_SM              <= WRITEBACK;
               end else begin                           // CMP
                  r_equal_flag <= r_alu_pipe_equal;
                  r_less_flag  <= r_alu_pipe_less;
                  r_ult_flag   <= r_alu_pipe_ult;
                  r_sign_flag  <= r_alu_pipe_value[63];
                  r_SM         <= OPCODE_REQUEST;
               end
            end

            default: r_SM <= HCF_1;  // loop in error
         endcase  // case(r_SM)
      end  // else if (w_reset_H)
   end  // always @(posedge i_Clk)
   
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

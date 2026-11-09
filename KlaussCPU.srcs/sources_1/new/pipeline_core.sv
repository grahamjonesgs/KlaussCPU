//===========================================================================
// pipeline_core — in-order 5-stage pipeline (IF ID EX MEM WB).
// Full ISA v2 + v3 (classes 1-D per ISA_ENCODING_V2_MAP.md). See
// PIPELINE_IMPL.md §8. Hazards:
//   - GPR RAW: register-sourced forwarding at the ID->EX operand latch
//     (M6): MEM/WB producers forward; a producer still in EX costs one
//     bubble, a load (in MEM) another.
//   - Flag READERS (cond branches, ADC/SBC, RCL/RCR, GETF) stall on a flag
//     writer in EX; one in MEM forwards via flags_eff (M6b). Flag WRITERS
//     carry {value, mask} down the pipe and masked-merge at WB — writers
//     never read flags (no writer hazard).
//   - SP users serialize on in-flight SP writers; SP commits at WB.
//   - RET/IRET/HALT/WAIT/RESET/TRAP/illegal serialize: ID stops fetching
//     behind them; RET/IRET redirect at MEM, RESET at EX, the rest park.
//   - Branches resolve in EX (predict-not-taken, flush IF/ID on taken);
//     fused compare-and-branch (class D) resolves in MEM, squashing EX/ID.
//   - MUL/DIV/DELAY/LCD occupy EX (busy interlock, M4); MEMGET32 cross-dword
//     occupies MEM one extra cycle (mem busy).
// Single retire point = WB: rf / flags(masked) / SP / int_mask commit there,
// and the retire interface presents the emulator-format trace record
// (klausscc --emulate --trace) for the tb to print and diff.
//
// Memory (M5b): ONE shared membus-style port — the exact cpu_mem signal set
// (KlaussCPU.sv:655-663) so M5d wires it straight to bus_splitter. Registered
// DV/addr, ready handshake (the FSM extra_clock pattern), read lookahead
// (rdata_next/next_valid within a 32 B line). IF and MEM contend: MEM has
// absolute priority (PIPELINE_IMPL.md §5); IF fills a 2-dword IFB-style
// window (any 1/2/3-word instruction spans at most 2 dwords).
//===========================================================================
module pipeline_core
   import klauss_pkg::*;
#(
   // Architectural SP after reset.  Core 1 = top of DDR; the AMP core-2
   // instance points it at the top of its local BRAM (AMP_CORE2_PLAN.md).
   parameter logic [31:0] SP_RESET = 32'h0800_0000,
   // Debug hooks for debug_module.sv (DEBUG_PLAN.md Phase 5). 0 = none of the
   // logic below exists and dbg_i is ignored (core 2, existing builds).
   parameter bit DEBUG_EN = 1'b0,
   // 2a branch-target instruction cache (see EN_BTIC).  Core 2 turns it off:
   // it is ~2k LUTs + a BRAM per instance and core 2 does not need it.
   parameter bit BTIC_EN = 1'b1,
   // MMU_PLAN.md stage 1: precise exceptions, user mode, SYSCALL, MFSR /
   // MTSR, LR/SC.  0 = the new encodings trap as illegal and nothing else
   // changes (core 2 builds it off).
   parameter bit EXC_EN = 1'b1,
   // MMU_PLAN.md stage 2: the 16-region MPU (needs EXC_EN).  0 = its SRs
   // are illegal and nothing else changes (core 2 builds it off).
   parameter bit MPU_EN = 1'b1,
   // MMU_PLAN.md stage 3: the 32-entry software-refilled TLB, TE (needs
   // EXC_EN).  0 = its SRs and instructions are illegal (core 2: off).
   parameter bit TLB_EN = 1'b1
)
(
   input  logic         clk,
   input  logic         rst,
   // Clock enable: every sequential block advances only when ce=1.  Core 1
   // ties it high (bit-identical to the pre-ce core); the AMP core-2 wrapper
   // drives a /2 toggle so the whole core closes timing under a blanket
   // 2-cycle multicycle constraint (AMP_CORE2_PLAN.md §3).  All latencies and
   // handshakes are in CORE cycles (ce ticks), including the mul/shifter
   // pipes — nothing free-runs past ce.
   input  logic         ce,
   // run control
   input  logic         start,       // pulse: begin fetching at start_pc
   input  logic [31:0]  start_pc,
   // shared memory port (membus_if master shape)
   output logic [31:0]  m_addr,
   output logic         m_read_DV,
   output logic         m_write_DV,
   output logic [63:0]  m_wdata,
   output logic [7:0]   m_be,
   input  logic [63:0]  m_rdata,
   input  logic [63:0]  m_rdata_next,   // next sequential dword, same line
   // 1a: the DRAM side's read data (the cache's registered output, before
   // the bus_splitter response select) — a fast load is always DRAM, so its
   // value skips the address decode / select (timing); = m_rdata elsewhere
   input  logic [63:0]  m_rdata_dram,
   // 1a stage 3 load-hit pipe (mem_read_write): the EX op's load address goes
   // out every cycle; the cache answers the cycle after with the doubleword
   // and whether it is a clean hit (STAGE3_DCACHE_PLAN.md)
   output logic [31:0]  h_addr,
   input  logic         h_ok,
   input  logic [63:0]  h_dword,
   input  logic         m_next_valid,
   input  logic         m_ready,
   // LCD side port
   output logic [7:0]   lcd_byte,
   output logic         lcd_dc,
   output logic         lcd_dv,
   output logic         lcd_rst_n,
   output logic         lcd_rst_wr,   // pulse: LCDRST executed (lcd_rst_n valid)
   input  logic         lcd_ready,
   // interrupts (external gating: irq_ready already masked by int_mask_o,
   // exactly like w_irq_ready / w_irq_sel / r_interrupt_table at top level)
   input  logic         irq_ready,
   input  logic [1:0]   irq_sel,
   input  logic [31:0]  irq_vector,
   output logic         irq_ack,       // 1-cycle: source irq_ack_sel dispatched
   output logic [1:0]   irq_ack_sel,
   output logic [3:0]   int_mask_o,
   // int_mask MMIO write-back (the SoC MMIO handler owns 0xF00F_0000)
   input  logic         mask_wr,
   input  logic [3:0]   mask_wdata,
   // retire / trace (registered at the WB commit edge; sample at negedge)
   output logic         ret_valid,
   output logic [31:0]  ret_pc,
   output logic [31:0]  ret_op,
   output logic         ret_wr,      // this retire performed a memory/MMIO write
   output logic [31:0]  ret_wr_addr, // as driven on d_addr
   output logic [7:0]   ret_wr_be,
   output logic [63:0]  ret_wr_raw,  // RAW store value (pre lane-replication)
   // port fully idle (no transaction in flight) — gates external bus handoff
   output logic         bus_idle,
   // per-cycle hazard-attribution strobes (accumulated by the SoC perf block):
   // [0] GPR RAW stall  [1] load-use subset  [2] flag stall  [3] SP stall
   // [4] mul/div/EX-busy [5] taken-branch flush (event)  [6] IF window miss
   // [7] MEM port wait
   output logic [7:0]   perf_stall,
   output logic         perf_id_empty,  // ID holds no op (for exclusive attribution)
   output logic         perf_br,        // conditional branch resolved (event)
   output logic         perf_br_taken,
   output logic         perf_fbr_taken, // ISA v3 B: taken fused branch (resolves in MEM)
   // I-cache (HW proposals §2.6): [0] demand-lookup miss that issues a
   // shared-port fill (event)  [1] fill in flight (cycle) — splits IF_MISS
   // into I-cache refill vs window bubbles
   output logic [1:0]   perf_ic,
   // park (drained stop)
   output logic         parked,
   output logic [2:0]   park_kind,   // 0=HALT 1=TRAP 2=ILLEGAL 3=WAIT 4=debug 5=MISALIGNED
   output logic [31:0]  park_pc,
   output logic [31:0]  park_op,
   // architectural state (tb reads for the trace line)
   output logic [63:0]  dbg_r [0:15],
   output logic [31:0]  dbg_sp,
   output flags_t       dbg_flags,
   // debugger (DEBUG_EN): halt/resume/step, breakpoints, watchpoints, and
   // register / memory access while debug-halted
   input  dbg_in_t      dbg_i = '0,
   output dbg_out_t     dbg_o
);

   // ------------------------------------------------------- build switches
   // (declared first: xvlog / Vivado reject a localparam used before it)
   // ISA v3 B (fused compare-and-branch) build switch. It adds a MEM-stage
   // pc source; the M13 builds measured CPU pc-path margin +0.03 ns with it
   // vs ~+0.2 ns without (WNS then set by SoC paths). 0 = class D traps and
   // the redirect logic synthesizes away (compile with
   // -mllvm -klausscpu-fuse-cmp-br=false).
   localparam bit EN_FBR = 1'b1;
   // HW proposal 6 (HARDWARE_PROPOSALS.md §2.6): FBR PRED 5-7 = EQW/LTW/ULTW
   // and the 2-word FBR with a sext(imm32) right-hand side.  0 = both trap
   // (the pre-proposal decode).
   localparam bit EN_FBR_W = 1'b1;
   // HW proposal 7a (§2.7): DI rd / EI rs1, class B system OP 6 / 7.
   // 0 = both trap and irq_dis stays 0.
   localparam bit EN_DIEI = 1'b1;
   // HW proposal 8 / §3.3: a 16- or 32-bit load/store whose address is not
   // naturally aligned parks (park_kind 5 -> crash dump ERR_MISALIGN)
   // instead of silently aligning down.  MEMGET32 ([rs1] 32-bit load, the
   // unaligned-tolerant form), byte and 64-bit accesses and the stack ops
   // keep their defined behaviour.  0 = the old silent align-down.
   localparam bit EN_MISAL_TRAP = 1'b1;
   // HW proposal 9 (§2.9): the missing short forms — CMPRVW simm8 (class 3
   // short with INV=1), MULV simm8 (class A OP=3, LEN=01, SGN [23]) and
   // LEAPC simm16 (class 2 LEN=01 OP=14 F=1, simm16 = {[19:12],[7:0]}).
   // All three trap today; 0 keeps it that way.  They add legs to
   // f_short_imm on the w_op -> id_var1 latch path.
   localparam bit EN_SHORT9 = 1'b1;
   // HW proposal 1c (§2.4): posted DRAM stores — a 1-entry store buffer on
   // the core side of the port.  A store that touches no fetch-side code
   // copy leaves MEM one cycle after issue; the port stays busy (st_post)
   // until its ready, and every other port user waits for that drain.  0 =
   // every store holds MEM until its ready (the pre-proposal behaviour).
   localparam bit EN_POST_ST = 1'b1;
   // HW proposal 3 (§2.6): start the NLB prefetch of the line after next on
   // the slide that empties the NLB (see the prefetch engine).  0 = the M9
   // start condition only.
   localparam bit EN_PF_EARLY = 1'b1;
   // HW proposal 2b-lite (§2.5): an EQ/NE fused branch (incl. EQW/NEW)
   // redirects from EX like JMPcc, one cycle earlier than from MEM.  Only
   // the equality forms: a 64-bit XOR-reduce from the ex_a/ex_b registers,
   // in parallel with (and no deeper than) the 64-bit wb_value==0 reduce
   // already on the JMPcc flags_eff -> exo_taken path.  The ordered compares
   // (carry chains) stay in MEM — ISA_V3_IMPL.md: a 64-bit magnitude compare
   // on the EX redirect path would not close.  TIMING-UNVERIFIED: default 0
   // until a build shows the exo_taken -> pc family still meets.
   localparam bit EN_FBR_EQ_EX = 1'b0;
   // HW proposal 2a, stage (a): return-address stack.  A taken CALL pushes
   // its return address as it leaves EX; a RET / LEAVERET dispatching from
   // ID with a non-empty stack redirects fetch to the top entry instead of
   // serializing, and is verified when its stack read completes in MEM — a
   // mismatch redirects there and squashes EX/ID exactly like a taken FBR.
   // IRET stays serialized.  0 = every return serializes (the old path).
   localparam bit EN_RAS = 1'b1;
   localparam int RAS_DEPTH = 4;           // entries (power of two)
   localparam int RAS_PW    = $clog2(RAS_DEPTH);
   // HW proposal 2a, stage (b): ID-stage branch prediction.  A 2-bit
   // counter table (LUTRAM, indexed by pc word bits, read at the IF->ID
   // latch) predicts conditional JMPcc and FBR; direct unconditional
   // JMP/CALL are always predicted taken.  A predicted-taken branch
   // redirects fetch when it dispatches from ID (target = registered id_pc
   // + displacement); it still resolves where it always did (JMPcc/CALL in
   // EX, FBR in MEM), which now redirects only on a wrong prediction — to
   // the taken target or the carried fall-through.  Incompatible with
   // EN_FBR_EQ_EX (that leg is ignored when EN_BP=1).  0 = no prediction.
   localparam bit EN_BP = 1'b1;
   localparam int BP_IDXW = 9;             // 512 counters
   // Predict taken only at counter 3 ("strongly taken") instead of >= 2:
   // a data-dependent ~50/50 branch then mostly stays predicted not-taken
   // (a wrong taken prediction costs a full redirect; a correct not-taken
   // one is free), while a loop branch reaches 3 within two iterations.
   localparam bit BP_STRONG = 1'b1;
   // HW proposal 1 step (1a/1b): early load issue.  A read (load, POP,
   // LEAVE, LEAVERET, RET, IRET) moving from EX into MEM while the port is
   // idle puts DV and its address on the port on that same edge (from
   // exo_iaddr, which already feeds mem_iaddr), so the cache sees it in
   // MEM's first cycle: one MEM cycle less per such load.  Not when the
   // MEM op then leaving is itself a port op (its DV must drop first), nor
   // for stores (their post / snoop checks run at the issue cycle).
   localparam bit EN_EARLY_LD = 1'b1;
   // HW proposal 1a (first stage): a plain DRAM load (LOAD, POP, LEAVE)
   // leaves MEM as soon as its request is on the port; WB waits for the
   // data and commits it straight from the response (the depth of the old
   // MEM-completion path).  The next instruction enters MEM a cycle earlier
   // and independent work behind the load overlaps its latency.  A load's
   // consumer waits until it has committed.  The next port request chains
   // onto the completion edge when its address differs.  MEMGET32,
   // RET/IRET/LEAVERET, MMIO and non-posted stores keep the blocking path.
   localparam bit EN_LD_WB = 1'b1;
   // 1a stage 2: a consumer dispatching on the edge a fast load's response
   // arrives takes the extracted value directly (wb_ldval into the operand
   // latches) instead of waiting for the register-file write: one cycle off
   // every load-use.  A deeper operand-latch input — timing switch.
   localparam bit EN_LD_FWD = 1'b1;
   // 1a stage 3: a DRAM LOAD/POP/LEAVE looks itself up in the cache's
   // load-hit pipe (h_addr at its EX->MEM edge, the answer in its first MEM
   // cycle) instead of going on the port.  A hit leaves MEM after one cycle
   // with its data, so a load holds one issue slot and its consumer issues 3
   // after it; back-to-back hits need no port.  A miss or conflict issues the
   // ordinary port request from WB (wb_hmiss) and waits there like a 1a fast
   // load.  0 = 1a stage 2 (the port for every load).
   localparam bit EN_DC_H = 1'b1;
   // One-slot stores: a DRAM store whose port is completely idle leaves MEM
   // on its issue edge, posted at once (st_post) — before st_post_ok (the
   // registered "no fetch-side code copy" check) is known.  The cycle after,
   // a store that should not have posted (it overlaps code somewhere on the
   // fetch side: SMC) squashes everything younger (MEM/EX/ID) and refetches
   // from its fall-through; the refetch goes through the port behind it and
   // the I-cache snoop has cleared the line by then.  0 = a store holds MEM
   // for its issue cycle plus the post cycle (1c).
   localparam bit EN_ST1 = 1'b1;
   // Front end: start the next-line prefetch on a BTIC-hit redirect edge
   // (the window is already loaded; no demand lookup needs the BRAM port).
   localparam bit EN_BTPF = 1'b1;
   // Front end: a 2-entry next-line buffer.  Either entry serves slides and
   // line crosses; the prefetch fills want_line, then (pipelined, no idle
   // cycle) want_line+1, so straight code runs a line ahead: 2-word code
   // fetches at ~1.2 cycles/op instead of 2.0.  (A "demand waits for the
   // in-flight prefetch" rule was tried and dropped: it interacted badly with
   // EN_BTPF on crc32.)  0 = the M9 single NLB.
   localparam bit EN_NLB2 = 1'b1;
   // SP forwarding: the stack pointer an EX op sees (ex_sp) is captured at
   // dispatch from the youngest older SP writer in MEM / WB (or sp), so a
   // RET / POP / PUSH behind a CALL / PUSH no longer waits for that SP to
   // commit in WB.  0 = SP readers wait for every in-flight SP writer.
   localparam bit EN_SPFWD = 1'b1;
   // Store -> load forwarding into the hit pipe (see h_fwd).
   localparam bit EN_STLF = 1'b1;
   // RET through the hit pipe / store forwarding: its stack read is answered
   // in MEM (forwarded from the CALL's push: 1 cycle; cache hit: 2), not on
   // the port behind the push.  A miss falls back to the port in MEM.
   localparam bit EN_HRET = 1'b1;
   // MUL holds EX 2 cycles, not 4: it travels on through MEM and WB commits
   // its product straight from the DSP's last stage (mul_pipe2 holds it on
   // the MUL's 2nd and 3rd cycle after leaving EX); a MUL still in MEM then
   // captures it (mem_mres).  Consumers forward from WB (latency 4, was 5);
   // flag readers wait for the MUL to commit (its flags stay off flags_eff).
   localparam bit EN_MUL2 = 1'b1;
   // Load -> ALU/BOOL/FBR bypass: such a consumer may dispatch while the
   // fast load (LOAD/POP/LEAVE) that writes its rs1/rs2 is in MEM, provided
   // EX is empty (the load is the youngest writer).  The load's extracted
   // value is registered at its MEM -> WB edge (wb_hext) and feeds the
   // consumer's ALU / compare operands in EX; a load that missed holds the
   // consumer in EX until its response commits, which is captured into the
   // operand register.  Load-use distance 3 -> 2 for these consumers.
   localparam bit EN_LDBYP = 1'b1;
   // A second store entry: a DRAM store that finds the port busy with a
   // posted store leaves MEM into sq_* (with the same issue-time snoop / post
   // check / squash as EN_ST1) and goes on the port when the posted one
   // drains.  Every other port user waits behind it.  0 = a second store
   // waits in MEM for the drain.
   localparam bit EN_SQ = 1'b1;
   localparam bit EN_PFYIELD = 1'b0;   // (timing: -1.1 ns on the IF cone)
   localparam bit EN_NLB2SRV = 1'b1;   // the 2nd entry also serves line-cross rebases
   // HW proposal 2a (front end): branch-target instruction cache.  16
   // entries indexed by branch pc[5:2], each holding the fetch window seen
   // at that branch's target (base dword + 2 dwords).  A predicted-taken
   // branch whose entry hits loads the IFB on its ID-dispatch edge, so the
   // target instruction latches on the next edge instead of after an
   // I-cache lookup + serve (2 cycles).  Entries are filled from the IFB
   // right after a predicted-taken redirect is served, and die with their
   // I-cache line (install / store snoop at that index), on ICACHE_INV and
   // at start — so every entry is a copy of a resident, snooped I-cache line
   // and the SMC completion squash covers the same window as for a serve.
   localparam bit EN_BTIC = BTIC_EN;
   // MMU_PLAN.md stage 1 (see EXC_EN).  With EVEC = 0 (reset / start) every
   // exception parks exactly as before, so existing images are unaffected.
   localparam bit EN_EXC = EXC_EN;
   localparam bit EN_MPU = EXC_EN && MPU_EN;   // MMU stage 2 (MPU)
   localparam bit EN_TLB = EXC_EN && TLB_EN;   // MMU stage 3 (TLB, TE)
   // Exception causes (ECAUSE, MMU_PLAN.md §4.1)
   localparam logic [3:0] X_ILL = 4'd1, X_PRIV = 4'd2, X_TRAP = 4'd3, X_SYSCALL = 4'd4,
                          X_MISAL_LD = 4'd5, X_MISAL_ST = 4'd6, X_ACC_LD = 4'd7,
                          X_ACC_ST = 4'd8, X_ACC_FE = 4'd9, X_MISAL_FE = 4'd10,
                          X_TLB_LD = 4'd11, X_TLB_ST = 4'd12, X_TLB_FE = 4'd13;
   // PRED 5/6/7 compare as EQ/LT/ULT on the 32-bit-shifted operands.  A
   // function of the same 4 bits f_cmp_op already reads (no extra level).
   function automatic logic [2:0] f_fbr_pred(logic [2:0] p);
      case (p)
         3'd5:    return 3'd0;   // EQW
         3'd6:    return 3'd1;   // LTW
         3'd7:    return 3'd3;   // ULTW
         default: return p;
      endcase
   endfunction

   // ------------------------------------------------------------------ types
   typedef enum logic [5:0] {
      U_ILL, U_ALU, U_BOOL, U_SHIFT, U_ROT1, U_BTSTN, U_UNARY,
      U_LEA, U_MOV, U_MOV64, U_BEXTR, U_BDEP,
      U_LOAD, U_LOAD32, U_STORE,
      U_PUSH, U_POP, U_GETSP, U_SETSP, U_ADDSP, U_RET, U_IRET,
      U_ENTER, U_LEAVE, U_LEAVERET,     // ISA v3 C
      U_FBR,                            // ISA v3 B fused compare-and-branch
      U_JMP, U_CALL, U_MUL, U_DIV, U_MOD,
      U_NOP, U_HALT, U_WAIT, U_RESET, U_TRAP, U_DELAY, U_LCD, U_LCDRST,
      U_DI, U_EI,                       // HW proposal 7a: interrupt disable/enable
      // MMU_PLAN.md stage 1: U_EXC = an exception op (cause in .sub) that
      // never commits; U_GETST = MFSR STATUS; LR / SC.  (MFSR of any other
      // SR executes as U_MOV64 with its value loaded at dispatch; MTSR as a
      // flag-less COPY with .mtsr, committed at WB.)
      U_EXC, U_GETST, U_LR, U_SC
   } uop_e;

   typedef struct packed {
      logic       legal;
      uop_e       uop;
      logic [3:0] sub;          // class minor-op (aluop / unary op / shift op)
      logic [3:0] rd, rs1, rs2;
      logic       use_rs1, use_rs2, use_rdd;
      logic       wreg;
      flags_t     fmask;        // which flag fields this op writes
      logic       fread;        // reads committed flags in EX
      logic       sp_rd, sp_wr;
      logic       is_mem;
      logic       serialize;
      logic       sgn;          // imm-ext / load / muldiv sign attr
      logic [1:0] msz;          // load/store size
      logic [1:0] mmode;        // EA mode
      logic       malign;
      logic       shsrc;        // shift count from N (else rs2)
      logic [5:0] shn;
      logic       shf;          // shift F bit
      logic [1:0] unsz;         // sext/zext size
      logic       unf;          // unary F bit
      logic       blink, brel, brind, binv;
      logic [3:0] bcond;
      logic       mdhigh, mduns;
      logic       lcd_is_data;
      logic [1:0] len;          // instruction words
      logic       w32;          // ISA v3 D3: result = sext of its low 32 bits (in MEM)
      logic       mtsr;         // MMU stage 1: MTSR (SR written at WB, then refetch)
      logic       fx2, fxg;     // a fetch fault: on the instruction's 2nd granule only / a GUARD denial
      logic       fxl;          // (stage 3) a TLB-related fetch fault (miss / X): TLB_HI auto-load
      logic       tlbi;         // MMU stage 3: TLBWI / TLBR / TLBP / TLBINV (at WB, then refetch)
   } dec_t;

   // ----------------------------------------------------- architectural state
   logic [63:0] rf [0:15];
   flags_t      flags;
   logic [31:0] sp;
   logic [3:0]  int_mask;
   // HW proposal 7a: core-internal interrupt DISABLE bit (DI sets it, EI
   // writes it from rs1[0]).  1 = no IRQ is taken at a dispatch boundary.
   // Saved at frame bit [43] on IRQ entry and restored by IRET; the
   // DISABLE sense keeps software-built frames (bit 43 = 0) enabled.
   logic        irq_dis;
   // MMU_PLAN.md stage 1 state.  umode = UM (1 = user).  sp_alt = the
   // inactive stack pointer (the user SP in supervisor mode, the kernel SP
   // in user mode; SR USP).  System registers EVEC / ECAUSE / EADDR /
   // SCRATCH0-1 / TP / CYCLE.  res_* = the LR/SC reservation.
   logic        umode;
   // fetch-side user mode: UM, or an IRET returning to user mode sitting in
   // WB (UM commits there a cycle after the IRET redirected fetch from MEM;
   // a lookup pending from before the IRET can issue in that cycle).
   // Registers only.  Used by the fill engine and the IF->ID latch checks.
   logic        fe_um;
   logic [31:0] sp_alt;
   // EVEC / ECAUSE / EADDR / SCRATCH0-1 / TP / CYCLE live in mmu_sr (a
   // kept hierarchy, timing); these are its outputs (names kept for tb_vsim)
   logic [31:0] evec, eaddr;
   logic [8:0]  ecause;          // [8] GUARD (stage 2)
   // MMU stage 2: mmu_mpu's outputs (see its instance at the end)
   logic        mpu_en;          // STATUS[8]
   // MMU stage 3: mmu_tlb's outputs (see its instance at the end)
   logic        te;              // STATUS[9]
   logic [7:0]  tlb_asid;
   logic [31:0] tlb_vec;         // TLBVEC
   logic [31:0] tlb_rdata;       // MFSR of a TLB SR
   logic        tlb_hi_load;     // exception entry: TLB_HI <- {VPN(EADDR), 0, ASID}
   logic [15:0] tlb_hi_vpn;
   logic [15:0] tlb_d_vpn, tlb_f_vpn;   // lookup ports: data / fetch (+ TLBP)
   logic        tlb_d_hit, tlb_f_hit;
   logic [10:0] tlb_d_pfn, tlb_f_pfn;
   logic [2:0]  tlb_d_xwr, tlb_f_xwr;   // {X, W, R}
   logic        mtsr_commit, tlbi_commit, tlb_discard, sr_ent_we;   // (assigned at the end)
   wire         tlb_psel;        // a TLBP in MEM or WB: the fetch port is the probe's
   wire         tlb_pload;       // a TLBP in MEM: load its key into it_vpn / it_asid
   logic [15:0] tlb_hi_vpn_o;
   logic [7:0]  tlb_hi_asid_o;
   logic [28:0] wb_va_dw;        // the WB op's VA doubleword (LR/SC reservation)
   logic [31:0] mpu_rdata;       // MFSR of an MPU SR
   logic        mpu_d1, mpu_dg1, mpu_d2, mpu_dg2;   // data: deny / GUARD, 1st and 2nd granule
   logic        mpu_f1, mpu_fg1, mpu_f2, mpu_fg2;   // fetch: the same at pc's granule and the next
   logic        res_valid;
   logic [28:0] res_dw;
   // exception entry: the IRQ entry sequencer runs with exc_ent set
   logic        exc_ent;
   logic [3:0]  exc_cause_q;
   logic [31:0] exc_vec_q;       // (stage 3, timing) the vector, chosen as the exception is latched
   logic        exc_guard_q;     // ECAUSE[8]
   logic        exc_tlb_q;       // (stage 3) TLB-related: TLB_HI auto-load
   // the frame push address of an entry (the kernel SP after any swap)
   wire [31:0]  push_va = ((EN_EXC && umode) ? sp_alt : sp) - 32'd8;
   logic [31:0] exc_addr_q, exc_epc_q;
   logic [31:0] pc;
   logic        running;        // fetch enabled
   logic        fetch_halt;     // serialize op in flight — hold fetch

   genvar gi;
   generate for (gi = 0; gi < 16; gi++) assign dbg_r[gi] = rf[gi]; endgenerate
   assign dbg_sp = sp; assign dbg_flags = flags;
   assign int_mask_o = int_mask;

   // IRQ entry sequencer (the OPCODE_REQUEST frame-push, pipelined form):
   // taken at a dispatch boundary INSTEAD of a fetch; the pipe drains, the
   // frame {mask, 7-bit flag word, resume PC} is pushed via the port, the
   // source is auto-masked, PC redirects to the vector.
   logic       irq_active;
   logic       irq_xc;
   logic [1:0] irq_sel_q;
   logic [31:0] irq_vec_q;

   // ------------------------------------------------------------- debug hooks
   // A halt (request, step completion, watchpoint) is taken at a dispatch
   // boundary like an IRQ: fetch stops, the pipe drains, and the core parks
   // with park_kind 4 at the next instruction's PC.  A breakpoint squashes
   // the matching instruction in ID before it executes and parks at its PC.
   // TRAP / ILLEGAL / HALT park as debug halts while a debugger is attached.
   // Resume restarts fetch at park_pc through the start path (which also
   // invalidates the I-cache, so code patched while halted is refetched).
   logic        dbg_active;      // halt taken at a boundary; pipe draining
   logic [2:0]  dbg_cause_q;     // cause for that coming park
   logic [2:0]  dbg_cause;       // cause of the current debug halt
   logic        dbg_stepping;    // resumed with step
   logic        dbg_step_done;   // ... and its one instruction has dispatched
   logic        dbg_skip;        // first dispatch after resume: no breakpoint
   logic        id_bp;           // ID instruction matched a breakpoint
   logic        dbg_watch_req;   // a watchpoint fired: halt at the next boundary
   logic [31:0] dbg_watch_addr;
   logic        dbg_mx;          // debugger memory access in flight
   logic        dbg_mem_done;
   logic [63:0] dbg_mem_rdata;
   wire dbg_halted = DEBUG_EN && parked && park_kind == 3'd4;
   wire dbg_stop   = DEBUG_EN && (dbg_i.halt_req || dbg_watch_req || dbg_step_done);
   // single-steps never take interrupts (and do not wake a WAIT)
   wire irq_ok     = irq_ready && !(DEBUG_EN && dbg_stepping);
   // HW proposal 7a: an IRQ is TAKEN only while not disabled (committed
   // bit; a DI still in flight is caught by the drain-time re-check). WAIT
   // still wakes on irq_ok, WFI-style, so "DI; test; WAIT" cannot deadlock.
   wire irq_take   = irq_ok && !(EN_DIEI && irq_dis);
   wire dbg_bp_hit = DEBUG_EN && !dbg_skip &&
                     |(dbg_i.bp_en & {dbg_i.bp_addr[3] == pc, dbg_i.bp_addr[2] == pc,
                                      dbg_i.bp_addr[1] == pc, dbg_i.bp_addr[0] == pc});
   assign dbg_o.halted     = dbg_halted;
   assign dbg_o.cause      = dbg_cause;
   assign dbg_o.watch_addr = dbg_watch_addr;
   assign dbg_o.mem_done   = dbg_mem_done;
   assign dbg_o.mem_rdata  = dbg_mem_rdata;
   assign dbg_o.sp_alt     = sp_alt;
   assign dbg_o.um         = umode;
   assign dbg_o.dis        = irq_dis;
   // TRAP / ILLEGAL / HALT -> debug-halt cause
   function automatic logic [2:0] f_dbg_park_cause(logic [2:0] kind);
      return (kind == 3'd1 || kind == 3'd5 || kind == 3'd6) ? DBG_CAUSE_TRAP
           : (kind == 3'd2) ? DBG_CAUSE_ILL : DBG_CAUSE_HALT;
   endfunction

   // ------------------------------------------- IF: 2-dword window (IFB model)
   logic [63:0] ifb_dw [0:1];
   logic [28:0] ifb_base;          // dword address of slot 0
   logic [1:0]  ifb_val;
   // MMU stage 3: per-slot fetch-fault tags.  A slot served by a fake fill
   // for a page the micro-ITLB refused (TE = 1) holds zeros and ft = 1;
   // fm = the refusal was a TLB miss.  The IF->ID latch turns them into
   // TLB_MISS_FETCH / ACCESS_FETCH.  Every real serve writes 0.
   logic [1:0]  ifb_ft, ifb_fm;

   wire [28:0] pc_dw = pc[31:3];
   // M12 position-flag fetch tracker (PIPELINE_M12_TIMING.md): pc's window
   // position is FLAGS maintained by shifts (dispatch), constants (serves)
   // and zeroing (redirects — correct, it just forces the rebase path a
   // redirect needs anyway), never live wide compares — the serial
   // [in0 equality -> miss_dw +1 adder -> nlb equality] cone was the
   // measured 16-level default-strategy wall. The base+K registers are
   // maintained coherently at base-write sites (rare, off per-cycle paths).
   // Real write sites set blocking EVENT FLAGS; one composer at the end of
   // the clocked block folds them — no mirrored guards, no drift.
   logic [2:0]  pos;               // pos[k] <=> (pc_dw == ifb_base + k)
   logic [28:0] r_base_p1;         // ifb_base + 1
   logic [28:0] r_base_p2;         // ifb_base + 2
   logic [27:0] r_want_line;       // ifb_base[28:1] + 1
   wire        in0   = ifb_val[0] && pos[0];
   wire        in1   = ifb_val[1] && pos[1];
   wire [127:0] w_win = in0 ? {ifb_dw[1], ifb_dw[0]} : {64'h0, ifb_dw[1]};
   wire [31:0] w_op   = pc[2] ? w_win[63:32]  : w_win[31:0];
   wire [31:0] w_var1 = pc[2] ? w_win[95:64]  : w_win[63:32];
   wire [31:0] w_var2 = pc[2] ? w_win[127:96] : w_win[95:64];
   // does the instruction spill into the next dword?
   wire        w_span = pc[2] ? (w_op[31:30] != 2'b01) : (w_op[31:30] == 2'b11);
   // the second dword is only sourced when the op sits in slot 0
   wire        fetch_ok = (in0 && (!w_span || ifb_val[1])) || (in1 && !w_span);
   // dword to request: slot-1 top-up when the op sits in slot 0 and spills;
   // otherwise rebase the window at the op's own dword (next_valid refills
   // slot 1 for free within a line).
   wire [28:0] miss_dw  = in0 ? r_base_p1 : pc_dw;   // mux of registers (M12)
   // Dispatch pc-advance, precomputed (timing keeper from the parked M11a
   // war, PIPELINE_M11_REDIRECT.md): the +4/+8/+12 adders run in parallel
   // with the window muxes and w_op's LEN bits only steer a narrow select —
   // the serial (window mux -> 32b add) chain was the depth of the pc->pc
   // zero-margin family. LEN==00 (illegal op) keeps the old pc+0 behavior.
   wire [31:0] w_pc_p4  = pc + 32'd4;
   wire [31:0] w_pc_p8  = pc + 32'd8;
   wire [31:0] w_pc_p12 = pc + 32'd12;
   wire [31:0] w_pc_adv = (w_op[31:30] == 2'b01) ? w_pc_p4
                        : (w_op[31:30] == 2'b10) ? w_pc_p8
                        : (w_op[31:30] == 2'b11) ? w_pc_p12 : pc;

   // ======================= M7a: fetch I-cache (fill-through) ==================
   // Direct-mapped, IC_LINES x 16 B (2 dwords), per-line tag + PER-DWORD valid.
   // Transparent accelerator on the IFB fill path: a hit returns the SAME
   // (rdata, rdata_next, next_valid) triple the shared-port line-read gives for a
   // given miss_dw, so the fill-engine capture logic is unchanged — a hit just
   // skips the shared m_* port (the measured IF_MISS + IF/MEM-contention win;
   // perf/m7/RESULTS.md). Coherence: invalidate-all on rst/start; per-store
   // index-invalidate snoop (conservative — clears the indexed line without a tag
   // read; SMC-correct, M7b refines to tag-checked). Miss path is unchanged (fetch
   // at miss_dw, install what returns; per-dword valid tolerates partial fills).
   //   dword addr = pc[31:3] (29b); line = dword[28:1]; dword-in-line = dword[0]=addr[3]
   localparam int IC_LINES = 512;
   localparam int IC_IDXW  = 9;                 // $clog2(IC_LINES)
   (* ram_style = "block" *) logic [127:0] ic_data [0:IC_LINES-1];  // {addr3=0 dw [127:64], addr3=1 dw [63:0]}
   (* ram_style = "block" *) logic [18:0]  ic_tag  [0:IC_LINES-1];
   logic ic_vhi [0:IC_LINES-1];                 // addr3=0 dword present
   logic ic_vlo [0:IC_LINES-1];                 // addr3=1 dword present
   // M7b: FF shadow of the tags, read COMBINATIONALLY by the store-snoop so it
   // can invalidate only a line the store actually overwrites (tag-checked),
   // not every line that index-aliases the store (index-only was catastrophic
   // under data-write-heavy workloads: a store's index collides with a cached
   // CODE line's index and evicts it though the code is untouched — LVGL's
   // framebuffer/.bss writes thrashed the I-cache to ~51% IF_MISS / ~286 CPI).
   logic [18:0] ic_tagff [0:IC_LINES-1];
   // Power-up value (the FPGA's register/LUTRAM init is 0): the 1c post
   // check reads it for every store, so it must not be X in 4-state sims.
   // A stale 0 tag only makes posting conservative for addresses below 8 KB.
   initial for (int i = 0; i < IC_LINES; i++) ic_tagff[i] = '0;
   // M12 snoop split: the tag compare runs at store issue and is registered
   // here; the ic_v* clear lands one cycle later (the one-cycle compare +
   // clear was the tier-1 worst family: tagff LUTRAM read -> 19b compare ->
   // 1024-FF index decode). The registered read below bypasses a pending
   // clear so a lookup in the gap cycle cannot hit the stale line.
   logic               snoop_q;
   logic [IC_IDXW-1:0] snoop_idx_q;

   // ======================= M9: next-line buffer (NLB) ========================
   // One prefetched 16 B line + a background I-cache lookup that runs while the
   // pipe executes out of the IFB, so a sequential line cross is served from a
   // register in 1 cycle instead of the 2-3 cycle if_look — at ANY alignment.
   // Prefetch fills on I-cache HIT only (a miss never touches the shared port:
   // no MEM contention, no pollution; cold lines take the demand path as
   // before). See PIPELINE_M9_FETCH_PREFETCH.md.
   logic [127:0] nlb_data;
   logic [27:0]  nlb_line;            // line address = dword[28:1] = byte[31:4]
   logic         nlb_vhi, nlb_vlo;    // per-dword valid (addr3=0 / addr3=1)
   // EN_NLB2: the second entry (same shape; either entry may hold any line)
   logic [127:0] nlb1_data;
   logic [27:0]  nlb1_line;
   logic         nlb1_vhi, nlb1_vlo;
   logic         pf_look;             // prefetch lookup owns the BRAM read port
   logic [27:0]  pf_line;
   logic         if_look;             // demand I-cache lookup phase (no port) —
                                      // declared here for the pf_sel mux below
   // The line after the one the window ends in ((ifb_base+2)>>1 for both
   // parities of ifb_base — for an odd base this "next line" also completes
   // the current straddle line's missing low dword). M12: registered
   // (r_want_line, maintained with the base+K triple) — no live +1 adder.
   wire  [27:0]  want_line    = r_want_line;
   wire          nlb0_has_want = (nlb_vhi || nlb_vlo) && (nlb_line == want_line);
   wire          nlb1_has_want = EN_NLB2 && (nlb1_vhi || nlb1_vlo) && (nlb1_line == want_line);
   wire          nlb_has_want  = nlb0_has_want || nlb1_has_want;
   // EN_NLB2: the line after want (the prefetch's second target)
   wire  [27:0]  want_p1       = r_want_line + 28'd1;
   wire          nlb0_has_wp1  = (nlb_vhi || nlb_vlo) && (nlb_line == want_p1);
   wire          nlb1_has_wp1  = EN_NLB2 && (nlb1_vhi || nlb1_vlo) && (nlb1_line == want_p1);
   wire          nlb_has_wp1   = nlb0_has_wp1 || nlb1_has_wp1;

   // Demand lookups own the BRAM read port whenever if_look is (or is about to
   // go) active; the prefetch leg uses it only in the gaps. Same-index reads
   // return the same row, so a mux switch can never serve stale data.
   // (EN_NLB2: the prefetch runs most of the time, so it also yields the
   // port on a cycle the demand side will start a lookup — the window cannot
   // supply the next op and neither NLB entry nor a slide can: the demand's
   // free first-cycle read of miss_dw stays free; the prefetch resumes.)
   logic              pf_yield;
   wire               pf_sel      = pf_look && !if_look && !(EN_NLB2 && EN_PFYIELD && pf_yield);
   wire [IC_IDXW-1:0] ic_look_idx = pf_sel ? pf_line[IC_IDXW-1:0]
                                           : miss_dw[IC_IDXW:1];
   wire [18:0]        ic_look_tag = miss_dw[28:IC_IDXW+1];
   // MMU stage 3 (TE = 1): VIPT.  The index (VA[12:4]) is untranslated;
   // the tag is PA[31:13] = {PA[31:16], VA[15:13]}.  A one-entry micro-ITLB
   // (ut_*) holds one page's translation and fetch verdict (for the fetch
   // mode ut_um); it is read alongside the BRAM, into r_look_*, so the tag
   // compare reads registers only.  A lookup whose page it does not cover
   // stalls while it_ref refills it from the main TLB's fetch port (one
   // cycle); a refused page is served as a fake fill with a fault tag, so
   // no untranslated or faulting address ever reaches the port.
   logic        ut_v, ut_um, ut_ok, ut_miss;
   logic [15:0] ut_vpn, ut_pa;
   logic        it_ref;
   logic [15:0] it_vpn;
   logic [7:0]  it_asid;          // the fetch-port CAM's ASID (ASID, or TLB_HI's for a TLBP)
   wire  [15:0] look_vpn = pf_sel ? pf_line[27:12] : miss_dw[28:13];
   logic [15:0] r_look_vpn = '0, r_look_pa = '0;
   logic        r_look_um = 1'b0, r_look_hit = 1'b0, r_look_ok = 1'b0, r_look_miss = 1'b0;
   // registered read of the line for the current miss_dw (1-cycle BRAM latency)
   logic [IC_IDXW-1:0] ic_rd_idx;
   logic [127:0]       ic_rd_data;
   logic [18:0]        ic_rd_tag;
   logic               ic_rd_vhi, ic_rd_vlo;
   // pending snoop clear (lands this edge) — read it as already invalid
   wire snoop_kill = snoop_q && (snoop_idx_q == ic_look_idx);
   always_ff @(posedge clk) if (ce) begin
      ic_rd_idx  <= ic_look_idx;
      ic_rd_data <= ic_data[ic_look_idx];
      ic_rd_tag  <= ic_tag [ic_look_idx];
      ic_rd_vhi  <= ic_vhi [ic_look_idx] && !snoop_kill;
      ic_rd_vlo  <= ic_vlo [ic_look_idx] && !snoop_kill;
      if (EN_TLB) begin
         r_look_vpn  <= look_vpn;
         r_look_um   <= fe_um;
         r_look_hit  <= ut_v && ut_vpn == look_vpn && ut_um == fe_um;
         r_look_pa   <= ut_pa;
         r_look_ok   <= ut_ok;
         r_look_miss <= ut_miss;
      end
   end
   // hit once the registered read has caught up to the (stable, stalled) miss_dw
   wire ic_rd_current = (ic_rd_idx == ic_look_idx);
   // (stage 3: the registered look is for this page and fetch mode)
   wire lk_cov        = r_look_hit && r_look_vpn == miss_dw[28:13] && r_look_um == fe_um;
   // (timing: the TE mux is on the reference — registers — so the BRAM's
   // tag output feeds one compare, as before)
   wire [18:0] ic_ref_tag = (EN_TLB && te) ? {r_look_pa, miss_dw[12:10]} : ic_look_tag;
   wire ic_tag_match  = ic_rd_current && (ic_rd_tag == ic_ref_tag) &&
                        (!(EN_TLB && te) || (lk_cov && r_look_ok));
   wire ic_dw_valid   = miss_dw[0] ? ic_rd_vlo : ic_rd_vhi;
   wire ic_hit        = ic_tag_match && ic_dw_valid;
   // (A registered dword-select shadow with a defer guard was tried here and
   // REVERTED: the defer fires after every pc move, i.e. on EVERY taken
   // branch's target serve — board-measured +0.5-1c per taken branch
   // (haz_real_m9b_keepers.csv). Redirect-facing serves must never be
   // defer-guarded; see PIPELINE_M11_REDIRECT.md.)
   // triple mirroring the m_* line-read for this miss_dw (see mem_read_write:
   // addr3=0 -> line[127:64], addr3=1 -> line[63:0]; next = the low dword)
   wire [63:0] ifr_rdata      = miss_dw[0] ? ic_rd_data[63:0] : ic_rd_data[127:64];
   wire [63:0] ifr_rdata_next = ic_rd_data[63:0];
   wire        ifr_next_valid = !miss_dw[0] && ic_rd_vlo;
   // (if_look is declared in the M9 NLB block above, ahead of the pf_sel mux)

   // M9: prefetch-side decode of the registered read (meaningful under pf_sel)
   wire pf_rd_current = pf_sel && (ic_rd_idx == pf_line[IC_IDXW-1:0]);
   wire [18:0] pf_ref_tag = (EN_TLB && te) ? {r_look_pa, pf_line[11:9]} : pf_line[27:IC_IDXW];
   wire pf_tag_match  = pf_rd_current && (ic_rd_tag == pf_ref_tag) &&
                        (!(EN_TLB && te) || (r_look_hit && r_look_vpn == pf_line[27:12] &&
                                             r_look_um == fe_um && r_look_ok));
   // M9: NLB serve triple for the demand window (mirrors ifr_* / m_* shapes)
   wire        nlb0_dw_ok     = (nlb_line == miss_dw[28:1]) &&
                                (miss_dw[0] ? nlb_vlo : nlb_vhi);
   wire        nlb1_dw_ok     = EN_NLB2 && EN_NLB2SRV && (nlb1_line == miss_dw[28:1]) &&
                                (miss_dw[0] ? nlb1_vlo : nlb1_vhi);
   wire        nlb_dw_ok      = nlb0_dw_ok || nlb1_dw_ok;
   wire [127:0] nlb_srv_data  = nlb0_dw_ok ? nlb_data : nlb1_data;
   wire        nlb_srv_vlo    = nlb0_dw_ok ? nlb_vlo  : nlb1_vlo;
   wire [63:0] nlb_rdata      = miss_dw[0] ? nlb_srv_data[63:0] : nlb_srv_data[127:64];
   wire [63:0] nlb_rdata_next = nlb_srv_data[63:0];
   wire        nlb_next_valid = !miss_dw[0] && nlb_srv_vlo;

   // M9+ 0-CYCLE serve: sliding window. Whenever pc is running from IFB slot 1
   // (in1) and the NLB holds the dword after the window (ifb_base+2 — its line
   // is want_line for BOTH base parities; its dword-in-line bit is ifb_base[0]),
   // shift slot1->slot0 and refill slot1 from the NLB at the same edge. pc then
   // never exits the window on a sequential path: the line cross that cost a
   // 1-cycle rebase serve costs 0, and a slot-1 straddle that cost an if_look
   // (~3c: odd-read rebase + top-up) costs at most 1. Register moves only —
   // nothing new lands on the w_op/dispatch cone. (w_slide, which needs the
   // MEM-stage store-completion signals, is declared after mem_done_now below.)
   wire        slide0_ok   = (nlb_line == want_line) &&
                             (ifb_base[0] ? nlb_vlo : nlb_vhi);
   wire        slide1_ok   = EN_NLB2 && (nlb1_line == want_line) &&
                             (ifb_base[0] ? nlb1_vlo : nlb1_vhi);
   wire        slide_dw_ok = slide0_ok || slide1_ok;
   wire [127:0] slide_line = slide0_ok ? nlb_data : nlb1_data;
   wire [63:0] slide_data  = ifb_base[0] ? slide_line[63:0] : slide_line[127:64];

   // ----------------------------------------------------------- ID latch
   logic        id_valid;
   logic [31:0] id_pc, id_op, id_var1, id_var2;
   // M13: latch-time precomputes (off the id_op->decode cones). id_pc_nx =
   // id_pc + 4*LEN (the fetch side's w_pc_adv); id_last_dw = dword of the
   // op's LAST word (pc + 4*(LEN-1)) for the SMC squash.
   logic [31:0] id_pc_nx;
   logic [28:0] id_last_dw;

   // -------------------------------------------------------------- decoder
   // f_decode_raw: field/legality decode WITHOUT the final illegal-op clear
   // (the M13 hazard predecode reads its use flags raw — see w_dec below).
   function automatic dec_t f_decode_raw(logic [31:0] op);
      dec_t d;
      logic [1:0] len;   logic [3:0] cls, aluop;
      d = '0;
      len   = op[31:30];
      cls   = op[29:26];
      aluop = op[25:22];
      d.len = len;  d.rd = op[11:8];  d.rs1 = op[7:4];  d.rs2 = op[3:0];
      d.uop = U_ILL;
      case (cls)
         4'h1, 4'h2: begin // ALU reg-reg (1w) / reg-imm (2w)
            d.sgn = op[20];
            // v3 D3: W [19] on class 1 ADD/SUB and the class 2 (2-word) ADD
            d.w32 = op[19] && len != 2'b01 || (cls == 4'h1 && op[19]);
            if ((cls == 4'h1 && len == 2'b01 && op[20] == 1'b0 && op[18:12] == 7'h0 &&
                 (!op[19] || aluop <= 4'd1)) ||
                (cls == 4'h2 && len == 2'b10 && op[18:12] == 7'h0 && op[3:0] == 4'h0 &&
                 (!op[19] || aluop == 4'd0)) ||
                (cls == 4'h2 && len == 2'b01 && op[3:0] == 4'h0) ||   // v3 A5 short: imm8 [19:12]
                (EN_SHORT9 && cls == 4'h2 && len == 2'b01 && aluop == 4'd14 && op[21:20] == 2'b10)) begin
               d.use_rs1 = 1'b1;  d.use_rs2 = (cls == 4'h1);
               case ({aluop, op[21]})
                  {4'd0,1'b1}, {4'd1,1'b1}, {4'd2,1'b1}, {4'd3,1'b1}: begin
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = aluop; d.wreg = 1'b1;
                     d.fmask = '1;  d.fread = (aluop == 4'd2 || aluop == 4'd3);
                  end
                  {4'd4,1'b0}, {4'd5,1'b0}, {4'd6,1'b0}: begin
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = aluop; d.wreg = 1'b1;
                  end
                  {4'd7,1'b0}, {4'd8,1'b0}, {4'd9,1'b0}, {4'd10,1'b0}: begin
                     if (cls == 4'h1) begin // MIN/MAX are reg-reg only
                        d.legal = 1'b1; d.uop = U_BOOL; d.wreg = 1'b1;
                        d.sub = (aluop == 4'd7) ? 4'(CMP_MIN)  : (aluop == 4'd8) ? 4'(CMP_MAX)
                              : (aluop == 4'd9) ? 4'(CMP_MINU) : 4'(CMP_MAXU);
                     end
                  end
                  {4'd14,1'b0}: if (cls == 4'h2) begin // LEAPC
                     d.legal = 1'b1; d.uop = U_LEA; d.wreg = 1'b1; d.use_rs1 = 1'b0;
                  end
                  {4'd14,1'b1}: if (EN_SHORT9 && cls == 4'h2 && len == 2'b01) begin // P9 LEAPC simm16
                     d.legal = 1'b1; d.uop = U_LEA; d.wreg = 1'b1; d.use_rs1 = 1'b0;
                  end
                  {4'd15,1'b0}: if (cls == 4'h2) begin // MOV: SETR / zext-MOV
                     d.legal = 1'b1; d.uop = U_MOV; d.wreg = 1'b1; d.use_rs1 = 1'b0;
                  end
                  default: ;
               endcase
            end else if (cls == 4'h2 && len == 2'b11 && aluop == 4'd15 && op[21:12] == 10'h0
                         && op[7:0] == 8'h00) begin
               d.legal = 1'b1; d.uop = U_MOV64; d.wreg = 1'b1;   // SETR64
            end
         end
         4'h3: begin // compare
            d.sgn = op[20];
            // v3 D2: W [19] = 32-bit compare (reg form and 2-word form only;
            // the short form's [19:12] is its immediate)
            if ((len == 2'b01 && op[20] == 1'b0 && op[18:12] == 7'h0) ||
                (len == 2'b10 && op[18:12] == 7'h0 && op[3:0] == 4'h0) ||
                (len == 2'b01 && op[20] && !op[21] && op[3:0] == 4'h0)) begin // v3 A3 short CMP
               d.use_rs1 = 1'b1;  d.use_rs2 = (len == 2'b01) && !op[20];
               if (op[21]) begin        // B=1: boolean rd = 0/1
                  if (op[25:23] <= 3'd4) begin
                     d.legal = 1'b1; d.uop = U_BOOL; d.wreg = 1'b1;
                     d.sub = 4'(f_cmp_op(op[25:23], op[22]));
                  end
               end else begin           // B=0: flag-setting CMP (= SUB, no wb)
                  // (P9: INV=1 on the SHORT form = the W compare, CMPRVW simm8)
                  if (op[25:23] == 3'd0 && op[11:8] == 4'h0 &&
                      (!op[22] || (EN_SHORT9 && len == 2'b01 && op[20]))) begin
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = 4'd7; // internal CMP code
                     d.fmask = '1;
                  end
               end
            end
         end
         4'h4: begin
            if (len == 2'b01 && op[13:12] == 2'b00) begin // shift/rotate/bit
               d.shsrc = op[21]; d.shn = op[20:15]; d.shf = op[14];
               d.use_rs1 = 1'b1; d.use_rs2 = !op[21];
               if (!op[21] && op[20:15] != 6'd0) begin /* SRC=0 requires N=0 */ end
               else case (aluop)
                  4'd0, 4'd1, 4'd2: begin
                     d.legal = 1'b1; d.uop = U_SHIFT; d.sub = aluop; d.wreg = 1'b1;
                     if (op[14]) d.fmask.zero = 1'b1;
                  end
                  4'd3, 4'd4: begin
                     d.legal = 1'b1; d.wreg = 1'b1;
                     if (op[21] && op[20:15] == 6'd1 && op[14]) begin
                        d.uop = U_ROT1; d.sub = aluop;
                        d.fmask.zero = 1'b1; d.fmask.carry = 1'b1;
                     end else begin
                        d.uop = U_SHIFT; d.sub = aluop;
                        if (op[14]) d.fmask.zero = 1'b1;
                     end
                  end
                  4'd5, 4'd6: begin // RCL/RCR only as SRC=1 N=1 F=1
                     if (op[21] && op[20:15] == 6'd1 && op[14]) begin
                        d.legal = 1'b1; d.uop = U_ROT1; d.sub = aluop; d.wreg = 1'b1;
                        d.fmask.zero = 1'b1; d.fmask.carry = 1'b1; d.fread = 1'b1;
                     end
                  end
                  4'd8, 4'd9, 4'd10: begin
                     d.legal = 1'b1; d.uop = U_SHIFT; d.sub = aluop; d.wreg = 1'b1;
                     if (op[14]) d.fmask.zero = 1'b1;
                  end
                  4'd11: begin
                     if (op[21]) begin
                        if (op[11:8] == 4'h0 && !op[14]) begin
                           d.legal = 1'b1; d.uop = U_BTSTN; d.fmask.zero = 1'b1;
                        end
                     end else begin
                        d.legal = 1'b1; d.uop = U_SHIFT; d.sub = 4'd11; d.wreg = 1'b1;
                        if (op[14]) d.fmask.zero = 1'b1;
                     end
                  end
                  default: ;
               endcase
            end else if (len == 2'b10 && aluop[3:2] == 2'b11 && op[21] == 1'b0
                         && op[20:15] == 6'd0 && op[14] == 1'b0 && op[13:12] == 2'b00) begin
               if (aluop == 4'd12 && op[3:0] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_BEXTR; d.wreg = 1'b1; d.use_rs1 = 1'b1;
                  d.fmask.zero = 1'b1;
               end else if (aluop == 4'd13) begin
                  d.legal = 1'b1; d.uop = U_BDEP; d.wreg = 1'b1;
                  d.use_rs1 = 1'b1; d.use_rs2 = 1'b1;
               end
            end
         end
         4'h5: begin // unary
            d.unsz = op[21:20]; d.unf = op[19];
            if (len == 2'b01 && op[18:12] == 7'h0 && op[3:0] == 4'h0) begin
               d.use_rs1 = 1'b1;
               if (aluop != 4'd4 && aluop != 4'd5 && op[21:20] != 2'd0) begin /* trap */ end
               else case (aluop)
                  4'd0, 4'd1, 4'd2, 4'd6, 4'd7, 4'd8, 4'd9, 4'd10: begin
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = aluop; d.wreg = 1'b1;
                     if (op[19]) d.fmask.zero = 1'b1;
                  end
                  4'd3: if (op[19]) begin  // ABS requires F
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd3; d.wreg = 1'b1;
                     d.fmask.zero = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  4'd4: if (op[21:20] != 2'd3) begin // SEXT
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd4; d.wreg = 1'b1;
                     if (op[19]) begin d.fmask.zero = 1'b1; d.fmask.sign = 1'b1; end
                  end
                  4'd5: if (op[21:20] != 2'd3) begin // ZEXT
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd5; d.wreg = 1'b1;
                     if (op[19]) d.fmask.zero = 1'b1;
                  end
                  4'd12: begin // GETF
                     d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd12; d.wreg = 1'b1;
                     d.use_rs1 = 1'b0; d.fread = 1'b1;
                     if (op[19]) d.fmask.zero = 1'b1;
                  end
                  4'd14, 4'd15: if (op[19]) begin // INC/DEC = ALU +/- 1
                     d.legal = 1'b1; d.uop = U_ALU; d.sub = (aluop == 4'd14) ? 4'd0 : 4'd1;
                     d.wreg = 1'b1; d.fmask = '1;
                  end
                  default: ;
               endcase
            end
         end
         4'h6, 4'h7: begin // loads / stores
            d.msz = op[25:24]; d.sgn = op[23]; d.mmode = op[22:21]; d.malign = op[20];
            if ((op[19:12] == 8'h0 || (len == 2'b01 && op[22:21] == 2'b01)) // v3 A4 imm8
                && !(cls == 4'h7 && op[23])                         // stores: SGN reserved
                && !(op[20] && op[25:24] != 2'b11)                  // A only for 64-bit
                && ((len == 2'b01 && (op[22:21] != 2'b10)) ||      // v3: +MODE=01 short
                    (len == 2'b10 && (op[22:21] == 2'b01 || op[22:21] == 2'b10)))
                && !(op[22:21] == 2'b00 && op[3:0] != 4'h0)         // [rs1]: rs2 = 0
                && !(op[22:21] == 2'b10 && op[7:0] != 8'h00)        // [imm]: rs1/rs2 = 0
                && !(op[22:21] == 2'b01 && op[3:0] != 4'h0)) begin  // rs1+imm: rs2 = 0
               d.use_rs1 = (op[22:21] != 2'b10);
               d.use_rs2 = (op[22:21] == 2'b11);
               d.is_mem  = 1'b1;
               if (cls == 4'h6) begin
                  if (op[25:24] == 2'b10 && op[22:21] == 2'b00) begin
                     if (!op[23]) begin d.legal = 1'b1; d.uop = U_LOAD32; d.wreg = 1'b1; end
                  end else begin
                     d.legal = 1'b1; d.uop = U_LOAD; d.wreg = 1'b1;
                  end
               end else begin
                  d.legal = 1'b1; d.uop = U_STORE; d.use_rdd = 1'b1;
               end
            end else if (EN_EXC && len == 2'b01 && op[22:21] == 2'b00 && op[19:13] == 7'h0 &&
                         op[12] && !op[20] && op[25]) begin
               // MMU stage 1 LR/SC: MODE=00 with [12]=1 (illegal before);
               // 32 / 64-bit.  LR.D: SGN = 0; LR.W: SGN = sign-extend.  SC:
               // rd = 0/1 result, rs2 = data, SGN = 0.
               d.is_mem = 1'b1;  d.use_rs1 = 1'b1;  d.wreg = 1'b1;
               if (cls == 4'h6 && op[3:0] == 4'h0 && !(op[24] && op[23])) begin
                  d.legal = 1'b1; d.uop = U_LR;
               end else if (cls == 4'h7 && !op[23]) begin
                  d.legal = 1'b1; d.uop = U_SC; d.use_rs2 = 1'b1;
               end
            end
         end
         4'h8: begin // branch / call
            d.blink = op[25]; d.brel = op[24]; d.brind = op[23];
            d.bcond = op[22:19]; d.binv = op[18];
            if (((len == 2'b10 && !op[23] && op[17:16] == 2'b00 && op[15:0] == 16'h0) ||
                 (len == 2'b01 && !op[24] && op[23] && op[17:16] == 2'b00 && op[15:4] == 12'h0) ||
                 (len == 2'b01 && op[24] && !op[23]))               // v3 A1: disp18 [17:0]
                && (op[22:19] <= 4'd9) && !(op[22:19] == 4'd0 && op[18])) begin
               d.legal   = 1'b1;
               d.uop     = op[25] ? U_CALL : U_JMP;
               d.use_rs2 = op[23];
               d.fread   = (op[22:19] != 4'd0);
               if (op[25]) begin d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1; end
            end
         end
         4'h9: begin // stack / SP
            // ISA v3 C: ENTER N (op 8, N = [21:0]) = push R15; R15 = SP;
            // SP -= 8*N.  LEAVE (op 9) = SP = R15; pop R15.  Both name R15
            // implicitly (the register fields are N / zero).
            if (len == 2'b01 && aluop == 4'd8) begin
               d.legal = 1'b1; d.uop = U_ENTER; d.wreg = 1'b1;
               d.rd = 4'd15; d.rs1 = 4'd15; d.use_rs1 = 1'b1;
               d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
            end else if (len == 2'b01 && aluop == 4'd9 && op[21:0] == 22'h0) begin
               d.legal = 1'b1; d.uop = U_LEAVE; d.wreg = 1'b1;
               d.rd = 4'd15; d.rs1 = 4'd15; d.use_rs1 = 1'b1;
               d.sp_wr = 1'b1; d.is_mem = 1'b1;
            end else if (len == 2'b01 && aluop == 4'd10 && op[21:0] == 22'h0) begin
               // LEAVERET (op 10) = LEAVE; RET: R15 = [R15], PC = [R15+8],
               // SP = R15 + 16. Two dwords: one read when [R15+8] is in the
               // same 16 B line (m_rdata_next), else the MEMGET32 2nd read.
               d.legal = 1'b1; d.uop = U_LEAVERET; d.wreg = 1'b1;
               d.rd = 4'd15; d.rs1 = 4'd15; d.use_rs1 = 1'b1;
               d.sp_wr = 1'b1; d.is_mem = 1'b1; d.serialize = 1'b1;
            end else if (op[21:20] == 2'b00 && op[19:12] == 8'h0) begin
               if (len == 2'b01 && op[3:0] == 4'h0) begin
                  case (aluop)
                     4'd0: if (op[11:8] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_PUSH; d.use_rs1 = 1'b1;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
                     end
                     4'd2: if (op[7:4] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_POP; d.wreg = 1'b1;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
                     end
                     4'd3: if (op[7:4] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_GETSP; d.wreg = 1'b1; d.sp_rd = 1'b1;
                     end
                     4'd4: if (op[11:8] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_SETSP; d.use_rs1 = 1'b1; d.sp_wr = 1'b1;
                     end
                     4'd6: if (op[11:4] == 8'h00) begin
                        d.legal = 1'b1; d.uop = U_RET;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1; d.serialize = 1'b1;
                     end
                     4'd7: if (op[11:4] == 8'h00) begin
                        d.legal = 1'b1; d.uop = U_IRET; d.fmask = '1;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1; d.serialize = 1'b1;
                     end
                     default: ;
                  endcase
               end else if (len == 2'b10 && op[11:0] == 12'h0) begin
                  case (aluop)
                     4'd1: begin
                        d.legal = 1'b1; d.uop = U_PUSH;
                        d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
                     end
                     4'd5: begin
                        d.legal = 1'b1; d.uop = U_ADDSP; d.sp_rd = 1'b1; d.sp_wr = 1'b1;
                     end
                     default: ;
                  endcase
               end else if (len == 2'b11 && aluop == 4'd1 && op[11:0] == 12'h0) begin
                  d.legal = 1'b1; d.uop = U_PUSH;   // PUSHV64
                  d.sp_rd = 1'b1; d.sp_wr = 1'b1; d.is_mem = 1'b1;
               end
            end
         end
         4'hA: begin // mul / div
            d.mduns = !op[23]; d.mdhigh = op[22]; d.sgn = op[23];
            // v3 D3: MULW = W [19] on the low-half MUL
            d.w32 = op[19];
            if (EN_SHORT9 && len == 2'b01 && op[25:24] == 2'd3 && op[22:20] == 3'b000 &&
                op[3:0] == 4'h0) begin
               // P9 MULV simm8: rd = rs1 * ext(imm8 [19:12]), SGN [23] (as MULV)
               d.use_rs1 = 1'b1; d.wreg = 1'b1; d.w32 = 1'b0;
               d.legal = 1'b1; d.uop = U_MUL;
               d.fmask.zero = 1'b1; d.fmask.sign = 1'b1; d.fmask.overflow = 1'b1;
            end else if (op[21:20] == 2'b00 && op[18:12] == 7'h0 &&
                (!op[19] || (op[25:24] == 2'd0 && !op[22]))
                && ((len == 2'b01) || (len == 2'b10 && op[3:0] == 4'h0))) begin
               d.use_rs1 = 1'b1;  d.use_rs2 = (len == 2'b01);  d.wreg = 1'b1;
               case (op[25:24])
                  2'd0: begin
                     d.legal = 1'b1; d.uop = U_MUL;
                     d.fmask.zero = 1'b1; d.fmask.sign = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  2'd1: if (!op[22]) begin
                     d.legal = 1'b1; d.uop = U_DIV;
                     d.fmask.zero = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  2'd2: if (!op[22]) begin
                     d.legal = 1'b1; d.uop = U_MOD;
                     d.fmask.zero = 1'b1; d.fmask.overflow = 1'b1;
                  end
                  default: ;
               endcase
            end
         end
         4'hB: begin // system
            if (EN_DIEI && len == 2'b01 && op[25:22] == 4'h0 && op[21:17] == 5'd3 &&
                         op[15:12] == 4'h0 && op[3:0] == 4'h0) begin
               // HW proposal 7a: OP 6 = DI rd (rd = old ENABLE bit, 0/1; then
               // disable), OP 7 = EI rs1 (enable = rs1[0]), so DI r ; ... ;
               // EI r restores the previous state.
               if (!op[16] && op[7:4] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_DI; d.wreg = 1'b1;
               end else if (op[16] && op[11:8] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_EI; d.use_rs1 = 1'b1;
               end
            end else if (op[25:22] == 4'h0 && op[15:8] == 8'h0 && op[3:0] == 4'h0) begin
               if (len == 2'b01) begin
                  case (op[21:16])
                     6'd0: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_NOP; end
                     6'd1: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_HALT;  d.serialize = 1'b1; end
                     6'd2: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_WAIT;  d.serialize = 1'b1; end
                     6'd3: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_RESET; d.serialize = 1'b1; end
                     6'd4: if (op[7:4] == 4'h0) begin d.legal = 1'b1; d.uop = U_TRAP;  d.serialize = 1'b1; end
                     6'd5: begin d.legal = 1'b1; d.uop = U_DELAY; d.use_rs1 = 1'b1; end
                     // MMU stage 1: SYSCALL (an exception op from decode)
                     // MMU stage 3: TLBWI (9) / TLBR (10) / TLBP (11) / TLBINV (12)
                     6'd9, 6'd10, 6'd11, 6'd12: if (EN_TLB && op[7:4] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_NOP; d.tlbi = 1'b1; d.serialize = 1'b1;
                     end
                     6'd8: if (EN_EXC && op[7:4] == 4'h0) begin
                        d.legal = 1'b1; d.uop = U_EXC; d.sub = X_SYSCALL; d.serialize = 1'b1;
                     end
                     default: ;
                  endcase
               end else if (len == 2'b10 && op[21:16] == 6'd5 && op[7:4] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_DELAY;   // DELAYV
               end
            end else if (EN_EXC && len == 2'b01 && op[25:22] == 4'h1 && op[20] == 1'b0 &&
                         op[3:0] == 4'h0) begin
               // MMU stage 1: MFSR rd, sr ([21]=0) / MTSR sr, rs1 ([21]=1),
               // sr = [19:12].  SRs 0-8 exist; MTSR to CYCLE (8) is illegal.
               // MFSR STATUS is U_GETST (its DI bit is forwarded in EX); any
               // other MFSR is a MOV64 whose value is loaded at dispatch
               // (id_mfsr).  MTSR is a flag-less COPY of rs1, committed to
               // the SR at WB, serializing.
               if (!op[21] && op[7:4] == 4'h0 &&
                   (op[19:12] <= 8'd8 || (EN_MPU && op[19:12] >= 8'h10 && op[19:12] <= 8'h2F) ||
                    (EN_TLB && op[19:12] >= 8'h30 && op[19:12] <= 8'h34))) begin
                  d.legal = 1'b1; d.wreg = 1'b1;
                  d.uop = (op[19:12] == 8'd0) ? U_GETST : U_MOV64;
               end else if (op[21] && op[11:8] == 4'h0 &&
                            (op[19:12] <= 8'd7 || (EN_MPU && op[19:12] >= 8'h10 && op[19:12] <= 8'h2F) ||
                             (EN_TLB && op[19:12] >= 8'h30 && op[19:12] <= 8'h34))) begin
                  d.legal = 1'b1; d.uop = U_UNARY; d.sub = 4'd0; d.use_rs1 = 1'b1;
                  d.mtsr = 1'b1; d.serialize = 1'b1;
               end
            end
         end
         4'hD: begin // ISA v3 B: fused compare-and-branch (no flags)
            // PRED [25:23] (as class 3), INV [22], IMM [21] (rhs = simm4
            // [3:0]), simm13 word displacement [20:8]. The compare runs on
            // the EX boolean path; a taken branch redirects from MEM.
            // HW proposal 6 (EN_FBR_W): PRED 5/6/7 = EQW/LTW/ULTW, the 32-bit
            // compare done by D2's dispatch shift (id_w); LEN=10 with IMM=1
            // and [3:0]=0 = rhs sext(imm32) from word 1 (id_var1).
            if (EN_FBR && (len == 2'b01 || (EN_FBR_W && len == 2'b10 && op[21] && op[3:0] == 4'h0))
                && (EN_FBR_W || op[25:23] <= 3'd4)) begin
               d.legal = 1'b1; d.uop = U_FBR; d.sgn = 1'b1;
               d.use_rs1 = 1'b1; d.use_rs2 = !op[21];
               d.sub = 4'(f_cmp_op(f_fbr_pred(op[25:23]), op[22]));
            end
         end
         4'hC: begin // LCD
            if (op[23:12] == 12'h0 && op[3:0] == 4'h0) begin
               if (len == 2'b01 && op[25:24] <= 2'd1 && op[11:8] == 4'h0) begin
                  d.legal = 1'b1; d.uop = U_LCD; d.use_rs1 = 1'b1;
                  d.lcd_is_data = op[24];
               end else if (len == 2'b10 && op[7:4] == 4'h0 && op[11:8] == 4'h0) begin
                  if (op[25:24] <= 2'd1) begin
                     d.legal = 1'b1; d.uop = U_LCD; d.lcd_is_data = op[24];
                  end else if (op[25:24] == 2'd2) begin
                     d.legal = 1'b1; d.uop = U_LCDRST;
                  end
               end
            end
         end
         default: ;
      endcase
      return d;
   endfunction

   // MMU stage 1: an exception op (no effects; serializes like ILLEGAL).
   // .legal = 0 keeps it out of the latch-predecode assertions (its raw use
   // flags may stall conservatively, as an illegal op's do).
   function automatic dec_t f_mkx(dec_t din, logic [3:0] cause);
      dec_t d;
      d = din;
      d.legal = 1'b0; d.uop = U_EXC; d.sub = cause; d.serialize = 1'b1; d.mtsr = 1'b0; d.tlbi = 1'b0;
      d.wreg = 1'b0; d.fmask = '0; d.is_mem = 1'b0; d.sp_wr = 1'b0;
      d.use_rs1 = 1'b0; d.use_rs2 = 1'b0; d.use_rdd = 1'b0; d.fread = 1'b0; d.sp_rd = 1'b0;
      return d;
   endfunction
   // privileged in user mode (MMU_PLAN.md §3.5): MFSR other than STATUS /
   // TP / CYCLE, MTSR other than TP
   function automatic logic f_priv(dec_t d, logic [31:0] op);
      case (d.uop)
         U_DI, U_EI, U_IRET, U_HALT, U_WAIT, U_RESET, U_DELAY, U_LCD, U_LCDRST: return 1'b1;
         U_MOV64: return op[29:26] == 4'hB && op[19:12] != 8'd7 && op[19:12] != 8'd8;
         default: return (d.mtsr && op[19:12] != 8'd7) || d.tlbi;
      endcase
   endfunction
   // um = user mode; fx / fxc = a fetch fault on this instruction (MMU
   // stage 1: user fetch from 0xF... (0) or a misaligned user PC (1)),
   // which wins over ILLEGAL and PRIVILEGED
   function automatic dec_t f_decode(logic [31:0] op, logic um, logic fx, logic fxc,
                                     logic fx2 = 1'b0, logic fxg = 1'b0,
                                     logic fxm = 1'b0, logic fxl = 1'b0);
      dec_t d;
      d = f_decode_raw(op);
      if (!d.legal) begin
         d.uop = U_ILL; d.serialize = 1'b1; d.mtsr = 1'b0; d.tlbi = 1'b0;
         d.wreg = 1'b0; d.fmask = '0; d.is_mem = 1'b0; d.sp_wr = 1'b0;
         d.use_rs1 = 1'b0; d.use_rs2 = 1'b0; d.use_rdd = 1'b0; d.fread = 1'b0; d.sp_rd = 1'b0;
      end else if (EN_EXC && um && f_priv(d, op)) begin
         d = f_mkx(d, X_PRIV);
      end
      if (EN_EXC && fx) begin
         d = f_mkx(d, fxc ? X_MISAL_FE : (EN_TLB && fxm) ? X_TLB_FE : X_ACC_FE);
         d.fx2 = fx2 && !fxc;  d.fxg = fxg && !fxc;  d.fxl = EN_TLB && fxl && !fxc;
      end
      return d;
   endfunction

   // MMU stage 1: the ID op's fetch fault, registered at the IF->ID latch;
   // id_mfsr = it is an MFSR (its SR value, id_srv, loads ex_var1/2)
   logic id_fx, id_fxc, id_mfsr, id_fx2, id_fxg, id_fxm, id_fxl;
   logic [63:0] id_srv;           // mmu_sr's read port, at id_op[15:12]
   dec_t dec;
   assign dec = f_decode(id_op, umode, id_fx, id_fxc, id_fx2, id_fxg, id_fxm, id_fxl);
   // M12 hazard predecode: the operand-use flags the interlock needs are
   // decoded from w_op at the IF->ID latch and registered, so raw_stall (and
   // through it the pc/dispatch hold) starts from registers instead of the
   // full f_decode(id_op) cone (the id_op->pc family). Read from the RAW
   // decode (M13): the flags skip the legality clear, so an illegal op may
   // stall conservatively (it traps anyway) but the full legality cone stays
   // off this path. Equal to dec.* for every legal op — asserted below.
   dec_t w_dec;
   assign w_dec = f_decode_raw(w_op);
   // M13: the dispatch's serializer test also reads a latch-time bit
   // (id_ser = full decode, so illegal ops still serialize) instead of
   // f_decode(id_op) — it gated every ID-latch load (the id_op->id_* family).
   dec_t w_decf;
   // MMU stage 1 fetch checks (user mode only): a PC with [1:0] != 0, or
   // any word of the instruction at 0xF000_0000+, makes it an exception op
   // (MISALIGNED_FETCH / ACCESS_FETCH) — decided here at the IF->ID latch
   // from pc and the window's LEN bits, registered into id_fx / id_fxc.
   wire [31:0] w_last_pc = (w_op[31:30] == 2'b11) ? w_pc_p8 : (w_op[31:30] == 2'b10) ? w_pc_p4 : pc;
   wire        w_fxc = EN_EXC && fe_um && (pc[1:0] != 2'b00);
   // (stage 2) the MPU's fetch decision at pc's 32-byte granule (p1) and,
   // for an instruction crossing into the next granule, at that one (p2);
   // the stage 1 user 0xF... rule applies first in each.  Computed from pc
   // and the window's LEN bits, registered into id_fx / id_fx2 / id_fxg.
   wire        w_cross = (w_op[31:30] == 2'b10 && pc[4:2] == 3'd7) ||
                         (w_op[31:30] == 2'b11 && pc[4:3] == 2'b11);
   wire        w_s1a = fe_um && pc[31:28] == 4'hF;
   wire        w_s1b = fe_um && w_last_pc[31:28] == 4'hF;
   // (stage 3, TE = 1: the MPU is off; the segment rule of §11.2 — user at
   // 0x8000_0000+, supervisor in the 0x88-0xBF hole — from pc (ts*), then
   // the window slot's fault tag (tf*, tm* = a TLB miss).  An instruction
   // whose second dword faults spans into the next page, so its EADDR is
   // that page's start = the next granule's start, as for the MPU.)
   wire        w_te  = EN_TLB && te;
   wire        w_tsa = w_te && (fe_um ? pc[31] : (pc[31:30] == 2'b10 && pc[31:27] != 5'b10000));
   wire        w_tsb = w_te && (fe_um ? w_last_pc[31]
                                      : (w_last_pc[31:30] == 2'b10 && w_last_pc[31:27] != 5'b10000));
   wire        w_tf1 = w_te && (in0 ? ifb_ft[0] : ifb_ft[1]);
   wire        w_tm1 = w_te && (in0 ? ifb_fm[0] : ifb_fm[1]);
   wire        w_tf2 = w_te && in0 && w_span && ifb_ft[1];
   wire        w_tm2 = w_te && in0 && w_span && ifb_fm[1];
   wire        w_mpu = EN_MPU && !w_te;
   wire        w_p1  = w_s1a || (w_mpu && mpu_f1) || w_tsa || w_tf1;
   wire        w_p2  = w_s1b || (w_mpu && w_cross && mpu_f2) || w_tsb || w_tf2;
   wire        w_fx  = EN_EXC && (w_fxc || w_p1 || w_p2);
   wire        w_fx2 = !w_p1 && w_p2;                       // EADDR: the next granule's start
   wire        w_fxg = w_mpu && (w_p1 ? (!w_s1a && mpu_fg1) : (!w_s1b && w_cross && mpu_fg2));
   wire        w_fxl = w_p1 ? (!w_s1a && !w_tsa && w_tf1) : (!w_s1b && !w_tsb && w_tf2);
   wire        w_fxm = w_p1 ? (!w_s1a && !w_tsa && w_tm1) : (!w_s1b && !w_tsb && w_tm2);
   assign w_decf = f_decode(w_op, 1'b0, 1'b0, 1'b0);
   // (timing: the latch-time predecode — id_ser, the branch / return
   // predictor bits — reads the decode WITHOUT the MMU overrides; a fetch
   // fault gates the predictor bits at their uses (id_fx), and the
   // overrides' serialize is ORed in as a parallel term, w_xser.  A
   // privileged op is never a branch or return.)
   wire w_xser = EN_EXC && (w_fx || (fe_um && f_priv(w_dec, w_op)));
   logic id_ser;
   logic id_use_rs1, id_use_rs2, id_use_rdd, id_fread, id_sp_rw;

   // ISA v3 short 1-word forms (ISA_V3_PROPOSAL.md §3): the immediate lives in
   // word 0 and is formed HERE, at the IF->ID latch, into id_var1 — so every
   // EX consumer (ex_b imm leg, EA, branch target, MOV) is unchanged.
   //   A5 class 2  LEN=01          imm8 [19:12], SGN [20] picks sext/zext
   //   A3 class 3  LEN=01 SGN=1 B=0 simm8 [19:12]
   //   A4 class 6/7 LEN=01 MODE=01  simm8 [19:12] << SIZE [25:24]
   //   A1 class 8  LEN=01 RIND=0    simm18 [17:0] << 2 (PC-relative)
   function automatic logic [32:0] f_short_imm(logic [31:0] op);
      logic [7:0] i8;
      logic [31:0] s8;
      i8 = op[19:12];
      s8 = {{24{i8[7]}}, i8};
      if (op[31:30] != 2'b01) return {1'b0, 32'b0};
      case (op[29:26])
         4'h2: return {1'b1, (EN_SHORT9 && op[25:21] == 5'b11101)            // P9 LEAPC simm16
                             ? {{16{op[19]}}, op[19:12], op[7:0]}
                             : op[20] ? s8 : {24'b0, i8}};
         4'hA: return {EN_SHORT9 && op[25:24] == 2'b11, op[23] ? s8 : {24'b0, i8}}; // P9 MULV
         4'h3: return {op[20] && !op[21], s8};
         4'h6, 4'h7: return {op[22:21] == 2'b01, s8 << op[25:24]};
         4'h8: return {!op[23], {{12{op[17]}}, op[17:0], 2'b00}};
         // B: fused branch compare immediate simm4 (IMM form)
         4'hD: return {op[21], {{28{op[3]}}, op[3:0]}};
         // C: ENTER's SP delta -(8 + 8N), so EX reuses the ADDSP adder
         4'h9: return {op[25:22] == 4'd8, 32'd0 - ({7'b0, op[21:0], 3'b000} + 32'd8)};
         default: return {1'b0, 32'b0};
      endcase
   endfunction
   wire [32:0] w_short = f_short_imm(w_op);
   function automatic logic [63:0] f_wsh(logic w, logic [63:0] v);
      return w ? {v[31:0], 32'b0} : v;
   endfunction
   // ex_b operand select, predecoded at the latch (M13): constant 1 (INC/DEC),
   // the extended immediate, or a register. Replaces a dec.len/dec.uop test
   // on the id_op->ex_b cone; asserted equal to the old decode below.
   logic id_b_one, id_b_imm, id_b_sext;
   // rs1 / rd register indices, decoded at the latch: = op[7:4] / op[11:8]
   // except ENTER / LEAVE / LEAVERET (R15).  Keeps the f_decode levels off
   // the id_op -> rf[] -> ex_a operand mux (fanout 129).
   logic [3:0] id_rs1, id_rd;
   // ISA v3 D2: 32-bit compare (class 3 W [19], not the short form). Both
   // operands are shifted left 32 at the dispatch mux: a 64-bit compare of
   // {a[31:0],0} vs {b[31:0],0} yields exactly the 32-bit Z/S/C/V (and the
   // 32-bit signed/unsigned order for the boolean forms) — EX is unchanged.
   logic id_w;
   wire [31:0] id_pc_next = id_pc_nx;

   // --------------------------------------------------------- EX stage latch
   logic        ex_valid;
   logic [31:0] ex_pc, ex_op, ex_var1, ex_var2, ex_pc_next;
   logic [28:0] ex_last_dw;         // M13: id_last_dw carried (SMC squash)
   dec_t        ex_d;
   logic [63:0] ex_a, ex_b, ex_c;   // rs1 / rs2 / rd-data values
   logic [31:0] ex_sp;              // EN_SPFWD: the SP this EX op reads
   // M6: GPR forwarding happens at the ID->EX register INPUT (the proven P41
   // shape): a producer completing EX forwards its combinational exo_result
   // straight into the consumer's operand latch on the dispatch edge — no
   // muxes inside EX, no use-site changes. Flags forward differently (M6b):
   // ex_fwd_f marks a consumer whose flags live in the mem_* registers.
   logic        ex_fwd_f;
   // 2a RAS: circular stack (ras_ptr = next push slot, ras_cnt = valid
   // entries, saturating — an overflowing push overwrites the oldest).
   // id_ras_ret: the ID op is a RET / LEAVERET (latch-time predecode).
   // ex_/mem_pred: this return was predicted, with its predicted target.
   logic [31:0]        ras [0:RAS_DEPTH-1];
   logic [RAS_PW-1:0]  ras_ptr;
   logic [RAS_PW:0]    ras_cnt;
   logic               id_ras_ret;
   logic               ex_pred, mem_pred;
   logic [31:0]        ex_pred_tgt, mem_pred_tgt;
   wire  [31:0]        ras_top = ras[ras_ptr - 1'b1];
   // 2b predictor state.  id_bp_unc: direct unconditional JMP/CALL;
   // id_bp_cnd: direct conditional JMPcc; id_bp_fbr: fused branch;
   // id_bp_abs: absolute (non-REL) target.  ex_/mem_ptk: predicted taken.
   (* ram_style = "distributed" *) logic [1:0] pht [0:(1<<BP_IDXW)-1];
   initial for (int i = 0; i < (1<<BP_IDXW); i++) pht[i] = 2'b01;   // weakly not-taken
   logic [1:0]         id_bp_ctr, ex_bp_ctr, mem_bp_ctr;
   logic [BP_IDXW-1:0] id_bp_idx, ex_bp_idx, mem_bp_idx;
   logic               id_bp_unc, id_bp_cnd, id_bp_fbr, id_bp_abs;
   logic               ex_bp_cnd, ex_ptk, mem_bp_fbr, mem_ptk;
   logic [31:0]        mem_pc_next;
   wire  [31:0]        id_bp_off = id_bp_fbr ? id_var2 : id_var1;
   wire  [31:0]        id_bp_tgt = id_bp_abs ? id_var1 : id_pc + id_bp_off;
   // 2a BTIC (see EN_BTIC): 64 entries, index = pc[7:2] ^ pc[13:8].  Tag,
   // base and data are read only at the index (LUTRAM-shaped); the line
   // index is a separate FF copy for the all-entry invalidation compare.
   localparam int BT_N = 64;
   logic [BT_N-1:0]    bt_v;
   logic [29:0]        bt_tag  [0:BT_N-1];   // branch pc[31:2]
   logic [28:0]        bt_base [0:BT_N-1];   // target dword (= IFB base)
   logic [63:0]        bt_d0   [0:BT_N-1], bt_d1 [0:BT_N-1];
   logic [BT_N-1:0]    bt_v1;                // slot-1 dword valid (same line only)
   logic [IC_IDXW-1:0] bt_lidx [0:BT_N-1];   // I-cache line index of the target window
   logic [IC_IDXW-1:0] bt_blidx[0:BT_N-1];   // I-cache line index of the BRANCH itself
   logic [28:0]        bt_bp1  [0:BT_N-1];   // base + 1, base + 2, base[28:1] + 1, kept
   logic [28:0]        bt_bp2  [0:BT_N-1];   //   with the entry (no adders after a hit)
   logic [27:0]        bt_want [0:BT_N-1];
   // ID-stage snapshot of the entry looked up at the IF->ID latch by the
   // branch's own pc (so the dispatch edge only muxes registers)
   logic               id_bt_hit;
   logic               id_rpend;         // the ID op's early redirect (2a) has not been done
   // ...which waits while EX holds an op that can redirect from EX, so the
   // two never meet on one edge (keeps EX's exo_taken off the pos claim)
   wire                ex_may_redir = ex_valid && (ex_d.uop == U_JMP || ex_d.uop == U_CALL ||
                                                    ex_d.uop == U_RESET);
   logic [5:0]         id_bt_idx;
   logic [63:0]        id_bt_d0, id_bt_d1;
   logic               id_bt_v1;
   logic [28:0]        id_bt_base, id_bt_bp1, id_bt_bp2;
   logic [27:0]        id_bt_want;
   logic               r_bt_pend;            // a missed entry waits for its serve
   logic [5:0]         r_bt_widx;
   logic [29:0]        r_bt_wtag;
   logic [28:0]        r_bt_wbase;
   logic [IC_IDXW-1:0] r_bt_wblidx;
   logic               ev_btic;          // blocking event: IFB loaded from the BTIC
   logic               ev_ovr;           // blocking event: an older squash moved pc after dispatch
   logic               ev_icinst;        // blocking event: an I-cache line install
   logic [8:0]         ev_icinst_idx;
   wire  [5:0]         bt_lidx_pc = pc[7:2] ^ pc[13:8];   // looked up at the IF->ID latch
   wire                id_bp_tk  = EN_BP && !(EN_EXC && id_fx) &&
                                   (id_bp_unc || ((id_bp_cnd || id_bp_fbr) &&
                                   (BP_STRONG ? (id_bp_ctr == 2'b11) : id_bp_ctr[1])));
   // 2a early redirect (see id_rpend)
   wire                id_early_rd  = EN_BP && id_valid && id_rpend && id_bp_tk && !fetch_halt &&
                                      !ex_may_redir;
   // A tag match implies the same branch instruction, hence the same target:
   // an entry dies with its branch's line as well as its target's.
   // (timing: no bt_v[id_bt_idx] re-read — a 64:1 mux on the redirect cone.
   // The snapshot carries its entry's line indices (id_bt_li / id_bt_bli);
   // an invalidation that matches them clears id_bt_hit, and one landing on
   // the very edge a snapshot was taken is caught here from its registered
   // index (r_btk_*).)
   logic [IC_IDXW-1:0] id_bt_li, id_bt_bli, r_btk_ic, r_btk_sn;
   logic               r_btk_icv, r_btk_snv, r_btk_all;
   wire                bt_hit    = EN_BTIC && id_bt_hit && !r_btk_all &&
                                   !(r_btk_icv && (id_bt_li == r_btk_ic || id_bt_bli == r_btk_ic)) &&
                                   !(r_btk_snv && (id_bt_li == r_btk_sn || id_bt_bli == r_btk_sn));
   function automatic logic [1:0] f_sat2(logic [1:0] c, logic t);
      return t ? ((c == 2'b11) ? c : c + 2'b01) : ((c == 2'b00) ? c : c - 2'b01);
   endfunction
   // Training: ONE write port (LUTRAM).  An FBR trains as it leaves MEM; a
   // conditional JMPcc as it leaves EX unsquashed.  On a same-edge clash
   // the (older) FBR wins and the JMPcc update is dropped.
   logic                pht_we;
   logic [BP_IDXW-1:0]  pht_wa;
   logic [1:0]          pht_wd;
   always_ff @(posedge clk) if (ce && pht_we) pht[pht_wa] <= pht_wd;

   // --------------------------------------------------------- MEM stage latch
   logic        mem_valid;
   logic [31:0] mem_pc, mem_op;
   dec_t        mem_d;
   logic [63:0] mem_result;         // non-mem result (or store raw data)
   flags_t      mem_fval, mem_fmask;
   logic        mem_zval;           // Z = (value==0), evaluated at WB (set_zero)
   logic [31:0] mem_eaddr;          // RAW effective address (lane math)
   logic [31:0] mem_iaddr;          // ISSUE address (aligned, drives the port)
   logic [63:0] mem_wdata;          // lane-replicated store data
   logic [7:0]  mem_be;
   logic [31:0] mem_sp_new;
   logic        mem_sp_we;
   logic [63:0] mem_dw0;            // MEMGET32 cross-line stash

   // --------------------------------------------------------- WB stage latch
   logic        wb_valid;
   // 1a: WB holds a fast load (its value comes from the port response at
   // commit) and the fields to extract it
   logic        wb_ld, wb_ld_full, wb_ld_sgn;
   logic [1:0]  wb_ld_msz;
   logic [2:0]  wb_ld_ea;
   // 1a stage 3: mem_h = the MEM op is a hit-pipe load; mem_h1 = its first
   // MEM cycle (the cache's answer is live); mem_hok/mem_hdw = that answer
   // captured, for a load held in MEM.  wb_h/wb_hok/wb_hdw follow it into
   // WB, with its address and byte enables for a slow-path request.
   logic        mem_h, mem_h1, mem_hok, wb_h, wb_hok;
   logic [63:0] mem_hdw_r, wb_hdw;
   // EN_STLF (timing): a forwarded load's data is selected AFTER the
   // registers — both candidate store dwords are captured every advance,
   // and only the 1-bit select / valid come from the dword compare
   logic [63:0] mem_fwa, mem_fwb, mem_fwc;
   logic        mem_fq, mem_fsel, mem_fselc;
   wire  [63:0] mem_hdw = mem_fq ? (mem_fsel ? mem_fwa : mem_fselc ? mem_fwc : mem_fwb) : mem_hdw_r;
   logic [31:0] wb_iaddr;
   logic [7:0]  wb_be;
   // EN_MUL2: mem_mage = cycles the MUL has spent in MEM; mem_mres / msgn /
   // movf = its product (value, sign, overflow) captured on its 2nd MEM
   // cycle; wb_mdir = the MUL in WB reads its product from mul_pipe2 now
   logic [1:0]  mem_mage;
   logic [63:0] mem_mres;
   logic        mem_msgn, mem_movf;
   logic        wb_mul, wb_mdir, wb_mhi, wb_muns, wb_mw32;
   // registered MEM / WB op class for the hazard cone: mem_lkq = a load or
   // (EN_MUL2) MUL (no MEM forward), mem_mulq = MUL, mem_mulf / wb_mulf = a
   // flag-setting MUL
   logic        mem_lkq, mem_mulq, mem_mulf, wb_mulf;
   // EN_LDBYP: mem_fastq = the MEM op is a fast load (registered at EX ->
   // MEM); id_ldc = the ID op may consume through the bypass; ex_lb_a/b =
   // the EX op reads rs1 / rs2 from the bypass; ex_w = its W-compare shift;
   // wb_hext = the WB load's extracted value
   logic        mem_fastq, id_ldc, ex_lb_a, ex_lb_b, ex_w;
   logic [63:0] wb_hext;
   // 1a: ld_out = a fast load's read is outstanding on the port (the load
   // itself is in WB, flagged wb_ld, waiting for the data).
   logic       ld_out;
   // (ready-edge arming, see the MEM / IF port engines; declared here for
   // the 1a hazard terms)
   logic rdy_armed;
   wire  w_mrdy = m_ready && rdy_armed;
   logic [31:0] wb_pc, wb_op;
   logic [3:0]  wb_rd;
   logic        wb_wreg;
   logic [63:0] wb_value;
   flags_t      wb_fval, wb_fmask;
   logic        wb_zval;
   logic        wb_zero;            // (wb_value == 0), reduced at the MEM->WB latch
   logic [31:0] wb_sp_new;
   logic        wb_sp_we;
   logic [3:0]  wb_mask_new;
   logic        wb_mask_we;
   logic        wb_park;
   logic [2:0]  wb_park_kind;
   logic        wb_wr;
   logic [31:0] wb_wr_addr;
   logic [7:0]  wb_wr_be;
   logic [63:0] wb_wr_raw;

   // ------------------------------------------------------------- interlocks
   // NB: explicit wire expressions (no helper function) — a function reading
   // module signals would break the assign's sensitivity list in simulation.
   wire b_ex  = ex_valid  && ex_d.wreg;
   wire b_mem = mem_valid && mem_d.wreg;
   wire b_wb  = wb_valid  && wb_wreg;
   // M6 v2 — REGISTER-SOURCED forwarding (the shape P3 actually verified: the
   // FSM forwards from the wb.value REGISTER, never from an ALU output; the
   // zero-bubble exo_result->operand-register feedback was WNS -0.750):
   //  - producer in MEM at dispatch -> forward mem_result (non-memory-read)
   //  - producer in WB  at dispatch -> forward wb_value (loads INCLUDED: the
   //    loaded value is in wb_value; rf only updates on this same edge)
   //  - producer in EX  at dispatch -> ONE bubble (next cycle it forwards)
   // Every forward mux sits at an ID->EX register D-input, fed by registers.
   wire mem_loadlk  = mem_valid && (mem_d.uop == U_LOAD || mem_d.uop == U_LOAD32 ||
                                    mem_d.uop == U_POP || mem_d.uop == U_LEAVE ||
                                    mem_d.uop == U_LEAVERET || mem_d.uop == U_LR);
   // EN_MUL2: a MUL in MEM has no value yet (its product is in the DSP)
   // (timing: the MEM op's class is registered at EX -> MEM — mem_lkq /
   // mem_mulq — so the RAW-hazard cone ANDs a flop with mem_valid instead of
   // decoding mem_d.uop)
   wire mem_mul     = EN_MUL2 && mem_valid && mem_mulq;
   wire mem_lk      = mem_valid && mem_lkq;
   wire mem_fwd_ok  = b_mem && !mem_lk;
   // Flag producers: in EX at dispatch -> 1 bubble; in MEM at dispatch -> the
   // consumer's EX cycle reads them from the WB registers (flags_eff below);
   // in WB at dispatch -> they commit on this edge, committed flags are right.
   wire ex_f_fwdable = mem_valid && (mem_fmask != '0);
   // (register fields: id_rs1 / id_rd predecoded at the latch; dec.rs2 = op[3:0])
   // 1a: a fast load in WB has no forwardable value until it commits
   wire wb_ldlk = EN_LD_WB && wb_valid && wb_ld && wb_wreg && !(EN_LD_FWD && ld_out && w_mrdy) &&
                  !(EN_DC_H && wb_h && wb_hok);
   // EN_LDBYP: a fast load in MEM, an ALU/BOOL/FBR consumer: no hazard on
   // that operand (it reads the load from WB through opa / opb; a younger
   // writer of the same register in EX is still the b_ex hazard)
   wire lb_ok = EN_LDBYP && id_ldc && mem_valid && mem_fastq && mem_d.wreg;
   wire lb1   = lb_ok && mem_d.rd == id_rs1;
   wire lb2   = lb_ok && mem_d.rd == dec.rs2 && !id_b_imm && !id_b_one;
   wire hit1 = (b_ex && ex_d.rd == id_rs1) || (mem_lk && mem_d.wreg && mem_d.rd == id_rs1 && !lb1) ||
               (wb_ldlk && wb_rd == id_rs1);
   wire hit2 = (b_ex && ex_d.rd == dec.rs2) || (mem_lk && mem_d.wreg && mem_d.rd == dec.rs2 && !lb2) ||
               (wb_ldlk && wb_rd == dec.rs2);
   wire hitd = (b_ex && ex_d.rd == id_rd)  || (mem_lk && mem_d.wreg && mem_d.rd == id_rd) ||
               (wb_ldlk && wb_rd == id_rd);
   wire wb_mdflag = EN_MUL2 && wb_valid && wb_mulf;
   wire flag_busy = (ex_valid && ex_d.fmask != '0) ||
                    (EN_MUL2 && mem_valid && mem_mulf) || wb_mdflag;   // EN_MUL2
   // EN_SPFWD: an SP writer in MEM / WB forwards its new SP into ex_sp at
   // dispatch, so only one still in EX holds an SP reader back
   wire sp_busy   = (ex_valid && ex_d.sp_wr) ||
                    (!EN_SPFWD && ((mem_valid && mem_sp_we) || (wb_valid && wb_sp_we)));
   wire raw_stall = id_valid && (
        (id_use_rs1 && hit1) || (id_use_rs2 && hit2) || (id_use_rdd && hitd) ||
        (id_fread && flag_busy) ||
        (id_sp_rw && sp_busy) );

   // hazard-attribution strobes (events/cycles for the SoC perf counters)
   assign perf_stall[0] = id_valid && ((id_use_rs1 && hit1) || (id_use_rs2 && hit2)
                                       || (id_use_rdd && hitd));
   // load-use: the RAW stall is on a load — still in EX, in MEM, or a fast
   // load waiting in WB (not only the MEM case: a consumer right behind a
   // load first waits while the load is in EX).
   function automatic logic f_id_reads(logic [3:0] r);
      return (id_use_rs1 && r == id_rs1) || (id_use_rs2 && r == dec.rs2) ||
             (id_use_rdd && r == id_rd);
   endfunction
   wire ex_loadp = ex_valid && ex_d.wreg &&
                   (ex_d.uop == U_LOAD || ex_d.uop == U_LOAD32 || ex_d.uop == U_POP ||
                    ex_d.uop == U_LEAVE || ex_d.uop == U_LEAVERET || ex_d.uop == U_LR);
   assign perf_stall[1] = id_valid && ((mem_loadlk && mem_d.wreg && f_id_reads(mem_d.rd)) ||
                                       (ex_loadp && f_id_reads(ex_d.rd)) ||
                                       (wb_ldlk && f_id_reads(wb_rd)));
   assign perf_stall[2] = id_valid && id_fread && flag_busy;
   assign perf_id_empty = !id_valid;
   assign perf_stall[3] = id_valid && id_sp_rw && sp_busy;

   // ------------------------------------------------------------- EX units
   // multiply — the silicon DSP48 chain, in its OWN always block exactly like
   // KlaussCPU.sv:558: no reset and no data-dependent enables, so the operand
   // regs absorb into DSP AREG/BREG.  The only enable is the plain ce net
   // (constant 1 for core 1; the /2 toggle for core 2), which keeps mul
   // latency in CORE cycles for the mul_cnt interlock.
   // TIMING HISTORY (AMP builds, 2026-08): this ce-gated shape closes (P1/P2a
   // met; P2b tier-1 -0.057 on id_op->DSP CEA1, tier-2 closed).  The
   // enable-free variant was tried in P3 and was WORSE (-0.266): phys_opt's
   // DSP register optimization then pulls ex_a into the DSP's 2nd A register,
   // exposing the decode->operand-mux cone as a full-cycle path into the DSP
   // (M5d's family).  Real fix = register the mul operands at DISPATCH (the
   // M5 ALU b-operand pattern) or dont_touch on mul_a_q/mul_b_q — its own
   // design pass (AMP_CORE2_PLAN.md §8).
   logic [64:0]  mul_a_q, mul_b_q;
   logic [127:0] mul_pipe1, mul_pipe2;
   logic [1:0]   mul_cnt;
   always_ff @(posedge clk) if (ce) begin
      mul_a_q   <= {(ex_d.mduns ? 1'b0 : ex_a[63]), ex_a};
      mul_b_q   <= {(ex_d.mduns ? 1'b0 : ex_b[63]), ex_b};
      mul_pipe1 <= $signed(mul_a_q) * $signed(mul_b_q);
      mul_pipe2 <= mul_pipe1;
   end
   // EN_MUL2: {overflow, sign, value} of a MUL from its 128-bit product (the
   // EX-stage U_MUL arm's rules; W forms sign-extend the low 32 as MEM does)
   function automatic logic [65:0] f_mulres(logic [127:0] pr, logic hi, logic uns, logic w32);
      logic [63:0] r64, v;
      logic        ov;
      r64 = hi ? pr[127:64] : pr[63:0];
      v   = w32 ? {{32{r64[31]}}, r64[31:0]} : r64;
      ov  = hi ? 1'b0 : uns ? (pr[127:64] != 64'd0) : (pr[127:64] != {64{pr[63]}});
      return {ov, r64[63], v};
   endfunction
   wire [65:0] mul_m = f_mulres(mul_pipe2, mem_d.mdhigh, mem_d.mduns, mem_d.w32);
   wire [65:0] mul_w = f_mulres(mul_pipe2, wb_mhi, wb_muns, wb_mw32);
   // divide (silicon restoring divider, M4)
   typedef enum logic [1:0] { MD_IDLE, MD_PREP, MD_STEP } md_phase_t;
   md_phase_t  div_ph;
   div_state_t dv;
   // delay spin
   logic        dly_on;
   logic [44:0] dly_cnt, dly_max;

   wire ex_is_md   = ex_valid && (ex_d.uop == U_MUL || ex_d.uop == U_DIV || ex_d.uop == U_MOD);
   wire ex_is_mul  = ex_valid && (ex_d.uop == U_MUL);
   wire ex_is_div  = ex_valid && (ex_d.uop == U_DIV || ex_d.uop == U_MOD);
   wire ex_div_mod = (ex_d.uop == U_MOD);
   wire ex_div_by0 = ex_is_div && (div_ph == MD_IDLE) && (ex_b == 64'd0);
   wire md_done    = (ex_is_mul && mul_cnt == (EN_MUL2 ? 2'd1 : 2'd3)) ||
                     (ex_is_div && div_ph == MD_STEP && dv.counter >= 7'd64) || ex_div_by0;
   wire ex_is_dly  = ex_valid && (ex_d.uop == U_DELAY);
   wire dly_done   = dly_on && (dly_cnt >= dly_max);
   wire ex_is_lcd  = ex_valid && (ex_d.uop == U_LCD);
   // EN_LDBYP: a bypass consumer whose load is not a hit-pipe answer in WB
   // (it missed) waits in EX for the load's response (captured into ex_a/b)
   wire ex_lbhold  = EN_LDBYP && ex_valid && (ex_lb_a || ex_lb_b) && !(wb_valid && wb_h && wb_hok);
   // MMU stage 2 (timing): with the MPU on, a memory op spends one extra
   // cycle in EX — its EA is registered (ex_ea_q) in the first, and the
   // region compares run from that register in the second, so the EA adder
   // and the compares are never in series.  Registered terms only.
   logic        ex_mph;           // the op in EX has had its MPU cycle
   logic        ex_mph2;          // ... and (TE, two-access op crossing a page) its 2nd-page cycle
   // MMU stage 3: a spanning MEMGET32 / LEAVERET (EA = ex_a) whose 2nd
   // doubleword is in the next 64 KiB page
   wire ex_xpg = (ex_d.uop == U_LEAVERET || (ex_d.uop == U_LOAD32 && ex_a[2:0] > 3'd4)) &&
                 ex_a[15:3] == 13'h1FFF;
   logic [31:0] ex_ea_q;          // exo_eaddr, registered every cycle
   // (stage 3: the same EX cycle carries the TLB lookup while TE = 1 — the
   // CAM reads the registered EA; a page-crossing two-access op takes a
   // third cycle for its 2nd page)
   // (timing: xlt_q / te_q are MPU_EN || TE and TE registered a cycle late
   // — safe: MTSR STATUS serializes and refetches, so no memory op is in EX
   // within a cycle of its commit; start empties the pipe)
   logic xlt_q = 1'b0, te_q = 1'b0;
   always_ff @(posedge clk) if (ce) begin
      xlt_q <= (EN_MPU && mpu_en) || (EN_TLB && te);
      te_q  <= EN_TLB && te;
   end
   wire ex_mpu_hold = xlt_q && ex_valid && ex_d.is_mem &&
                      (!ex_mph || (te_q && ex_xpg && !ex_mph2));
   wire ex_busy    = (ex_is_md && !md_done) || (ex_is_dly && !dly_done)
                  || (ex_is_lcd && !lcd_ready) || ex_lbhold || ex_mpu_hold;
   assign perf_stall[4] = ex_busy;

   wire [63:0] w_div_abs_a   = (ex_d.sgn && ex_a[63]) ? (~ex_a + 64'd1) : ex_a;
   wire [63:0] w_div_abs_b   = (ex_d.sgn && ex_b[63]) ? (~ex_b + 64'd1) : ex_b;
   wire [63:0] w_div_shifted = {dv.remainder[62:0], dv.dividend[63]};
   wire [64:0] w_div_trial   = {1'b0, w_div_shifted} - {1'b0, dv.divisor};
   wire        w_div_borrow  = w_div_trial[64];

   function automatic logic [6:0] f_clz64(input logic [63:0] val);
      f_clz64 = 7'd64;
      for (int i = 0; i < 64; i++) if (val[i]) f_clz64 = 7'd63 - 7'(i);
   endfunction
   function automatic logic [6:0] f_ctz64(input logic [63:0] val);
      f_ctz64 = 7'd64;
      for (int i = 63; i >= 0; i--) if (val[i]) f_ctz64 = 7'(i);
   endfunction
   function automatic logic [6:0] f_popcnt64(input logic [63:0] val);
      f_popcnt64 = '0;
      for (int i = 0; i < 64; i++) f_popcnt64 = f_popcnt64 + 7'(val[i]);
   endfunction
   function automatic logic [63:0] f_bitrev64(input logic [63:0] val);
      for (int i = 0; i < 64; i++) f_bitrev64[63-i] = val[i];
   endfunction

   // ---------------------------------------------------------- EX combinational
   logic [63:0] exo_result;
   flags_t      exo_fval;
   flags_t      exo_fmask;         // = ex_d.fmask (value side computed here)
   logic        exo_taken;         // EX redirect (branch/call taken, RESET)
   logic [31:0] exo_target;
   logic [31:0] exo_eaddr;
   logic [63:0] exo_wdata;         // lane-replicated store data
   logic [7:0]  exo_be;
   logic [63:0] exo_raw;           // raw store value (trace)
   logic [31:0] exo_sp_new;
   logic        exo_sp_we;
   logic [31:0] exo_iaddr;         // port ISSUE address (aligned per f_ld_idx)

   wire [63:0] ex_imm    = ex_d.sgn ? {{32{ex_var1[31]}}, ex_var1} : {32'b0, ex_var1};
   // The ALU/compare b-operand is fully resolved at DISPATCH into ex_b
   // (reg / sign-or-zero-extended imm32 / constant 1 for INC-DEC): the 64-bit
   // adder and comparators read pure registers — no decode mux in front of
   // the carry chain (the WNS -0.332 ex_op -> mem_result path).
   wire [5:0]  ex_shcnt  = ex_d.shsrc ? ex_d.shn : ex_b[5:0];
   // M6b: effective flags for EX readers. A flag producer that was in MEM at
   // the consumer's dispatch sits in the WB registers during the consumer's
   // EX cycle (it commits at the end of that cycle): merge its masked fields
   // over the still-uncommitted flags register. Deferred-Z producers
   // contribute wb_zero = (wb_value == 0), reduced a cycle early at the
   // MEM->WB latch so the forwarded Z is a single register here.
   flags_t flags_eff;
   always_comb begin
      flags_eff = flags;
      if (ex_fwd_f) begin
         if (wb_fmask.zero)     flags_eff.zero     = wb_zval ? wb_zero
                                                             : wb_fval.zero;
         if (wb_fmask.carry)    flags_eff.carry    = wb_fval.carry;
         if (wb_fmask.overflow) flags_eff.overflow = wb_fval.overflow;
         if (wb_fmask.sign)     flags_eff.sign     = wb_fval.sign;
      end
   end
   wire [1:0]  ex_bcvt_t = f_cond_eval(flags_eff, ex_d.bcond, ex_d.binv);

   // HW proposal 7a: the disable bit as seen by the instruction in EX — the
   // youngest older writer in MEM (DI/EI) or WB (DI/EI/IRET), else the
   // committed bit.  All register-sourced.
   logic        mem_dis_we, mem_dis_new;   // EX->MEM (DI/EI only)
   logic        wb_dis_we,  wb_dis_new;    // MEM->WB (DI/EI/IRET)
   wire ex_dis_eff = (mem_valid && mem_dis_we) ? mem_dis_new
                   : (wb_valid  && wb_dis_we)  ? wb_dis_new : irq_dis;

   // MMU stage 1: does the SC in EX succeed?  The reservation as it will be
   // after the older LR / SC in WB, then in MEM, commit (both always do: a
   // faulting one is no longer an LR / SC uop, and a squash of either kills
   // this SC too).  SC is [rs1] only, so its address is ex_a (no adder).
   logic wb_lr, wb_sc;
   wire ex_sc_ok = (mem_valid && mem_d.uop == U_LR) ? (mem_eaddr[31:3] == ex_a[31:3])
                 : (mem_valid && mem_d.uop == U_SC) ? 1'b0
                 : (wb_valid && wb_lr)              ? (wb_va_dw == ex_a[31:3])
                 : (wb_valid && wb_sc)              ? 1'b0
                 :                                    (res_valid && res_dw == ex_a[31:3]);
   wire [31:0] ex_sp_r = EN_SPFWD ? ex_sp : sp;
   // EN_LDBYP: the ALU / BOOL / FBR operands (a bypassed load's value from
   // WB, with the W-compare shift applied as at dispatch)
   wire [63:0] opa = (EN_LDBYP && ex_lb_a) ? (ex_w ? {wb_hext[31:0], 32'b0} : wb_hext) : ex_a;
   wire [63:0] opb = (EN_LDBYP && ex_lb_b) ? (ex_w ? {wb_hext[31:0], 32'b0} : wb_hext) : ex_b;
   // MIN/MAX in EX: only which operand wins (registered as mem_mmpick); the
   // select is in MEM (see mem_result_eff)
   wire ex_is_mm  = (ex_d.uop == U_BOOL) &&
                    (cmp_op_e'(ex_d.sub) == CMP_MIN || cmp_op_e'(ex_d.sub) == CMP_MAX ||
                     cmp_op_e'(ex_d.sub) == CMP_MINU || cmp_op_e'(ex_d.sub) == CMP_MAXU);
   wire ex_mmpick = (cmp_op_e'(ex_d.sub) == CMP_MIN)  ? ($signed(opa) < $signed(opb))
                  : (cmp_op_e'(ex_d.sub) == CMP_MAX)  ? ($signed(opa) > $signed(opb))
                  : (cmp_op_e'(ex_d.sub) == CMP_MINU) ? (opa < opb)
                  :                                     (opa > opb);      // MAXU
   always_comb begin : ex_logic
      logic [64:0]        sum;
      logic               is_sub, cin;
      logic signed [63:0] sa, sb;
      logic [63:0]        val, q_raw;
      logic               q_neg;
      logic [4:0]         bx_start, bx_len;
      logic [31:0]        bx_mask, bx_res;
      exo_result = '0;  exo_fval = '0;  exo_fmask = ex_d.fmask;
      exo_taken  = 1'b0; exo_target = '0;
      exo_eaddr  = '0;  exo_wdata = '0; exo_be = 8'hFF; exo_raw = '0;
      exo_sp_new = ex_sp_r;  exo_sp_we = ex_d.sp_wr;
      // effective address (loads/stores)
      case (ex_d.mmode)
         2'b00: exo_eaddr = ex_a[31:0];
         2'b01: exo_eaddr = ex_a[31:0] + ex_var1;
         2'b10: exo_eaddr = ex_var1;
         2'b11: exo_eaddr = ex_a[31:0] + ex_b[31:0];
      endcase
      case (ex_d.uop)
         U_ALU: begin
            is_sub = (ex_d.sub == 4'd1) || (ex_d.sub == 4'd3) || (ex_d.sub == 4'd7);
            cin    = (ex_d.sub == 4'd2 || ex_d.sub == 4'd3) ? flags_eff.carry : 1'b0;
            case (ex_d.sub)
               4'd4: exo_result = opa & opb;
               4'd5: exo_result = opa | opb;
               4'd6: exo_result = opa ^ opb;
               default: begin
                  if (is_sub) sum = {1'b0, opa} - {1'b0, opb} - {64'b0, cin};
                  else        sum = {1'b0, opa} + {1'b0, opb} + {64'b0, cin};
                  exo_result        = sum[63:0];
                  exo_fval.carry    = sum[64];
                  exo_fval.sign     = sum[63];
                  exo_fval.overflow = is_sub ? ((opa[63] != opb[63]) && (sum[63] != opa[63]))
                                             : ((opa[63] == opb[63]) && (sum[63] != opa[63]));
               end
            endcase
         end
         U_BOOL, U_FBR: begin
            // U_FBR: the taken bit lands in mem_result[0]; its target
            // (PC + 4*simm13, from ex_var2) rides in the EA register.
            if (ex_d.uop == U_FBR) exo_eaddr = ex_pc + ex_var2;
            sa = opa; sb = opb;
            case (cmp_op_e'(ex_d.sub))
               CMP_EQ:   exo_result = (opa == opb) ? 64'b1 : 64'b0;
               CMP_NE:   exo_result = (opa != opb) ? 64'b1 : 64'b0;
               CMP_LT:   exo_result = (sa < sb)   ? 64'b1 : 64'b0;
               CMP_LE:   exo_result = (sa <= sb)  ? 64'b1 : 64'b0;
               CMP_GT:   exo_result = (sa > sb)   ? 64'b1 : 64'b0;
               CMP_GE:   exo_result = (sa >= sb)  ? 64'b1 : 64'b0;
               CMP_ULT:  exo_result = (opa < opb)  ? 64'b1 : 64'b0;
               CMP_ULE:  exo_result = (opa <= opb) ? 64'b1 : 64'b0;
               CMP_UGT:  exo_result = (opa > opb)  ? 64'b1 : 64'b0;
               CMP_UGE:  exo_result = (opa >= opb) ? 64'b1 : 64'b0;
               // MIN/MAX/MINU/MAXU (timing): the 64-bit select moved to MEM
               // (mem_result_eff, M12 Stage D style).  EX hands on opa (as
               // the result), opb (in the store-data register, unused by ALU
               // ops) and the 1-bit pick (ex_mmpick, registered).
               default:  begin exo_result = opa; exo_wdata = opb; end
            endcase
            // 2b-lite: an equality FBR redirects from EX (exo_taken), and
            // goes down the pipe as not-taken so MEM does not redirect again
            if (EN_FBR_EQ_EX && !EN_BP && ex_d.uop == U_FBR &&
                (cmp_op_e'(ex_d.sub) == CMP_EQ || cmp_op_e'(ex_d.sub) == CMP_NE)) begin
               exo_taken  = exo_result[0];
               exo_target = exo_eaddr;          // ex_pc + ex_var2
               exo_result = 64'b0;
            end
         end
         U_SHIFT: exo_result = 64'b0;   // M12 Stage D: the result is produced
                                        // in MEM (w_sh2 via mem_result_eff);
                                        // the barrel left the EX cluster.
         U_ROT1: begin
            case (ex_d.sub)
               4'd3: begin exo_result = {ex_a[62:0], ex_a[63]};     exo_fval.carry = ex_a[63]; end
               4'd4: begin exo_result = {ex_a[0], ex_a[63:1]};      exo_fval.carry = ex_a[0];  end
               4'd5: begin exo_result = {ex_a[62:0], flags_eff.carry};  exo_fval.carry = ex_a[63]; end
               default: begin exo_result = {flags_eff.carry, ex_a[63:1]}; exo_fval.carry = ex_a[0]; end
            endcase
         end
         U_BTSTN: exo_fval.zero = ~ex_a[ex_d.shn];
         U_UNARY: begin
            case (ex_d.sub)
               4'd0:  exo_result = ex_a;
               4'd1:  exo_result = ~ex_a + 64'd1;
               4'd2:  exo_result = ~ex_a;
               4'd3:  begin
                  exo_result        = ex_a[63] ? (~ex_a + 64'd1) : ex_a;
                  exo_fval.overflow = (ex_a == 64'h8000000000000000);
               end
               4'd4:  exo_result = (ex_d.unsz == 2'd0) ? {{56{ex_a[7]}},  ex_a[7:0]}
                                 : (ex_d.unsz == 2'd1) ? {{48{ex_a[15]}}, ex_a[15:0]}
                                                       : {{32{ex_a[31]}}, ex_a[31:0]};
               4'd5:  exo_result = (ex_d.unsz == 2'd0) ? {56'b0, ex_a[7:0]}
                                 : (ex_d.unsz == 2'd1) ? {48'b0, ex_a[15:0]}
                                                       : {32'b0, ex_a[31:0]};
               4'd6:  exo_result = {ex_a[7:0], ex_a[15:8], ex_a[23:16], ex_a[31:24],
                                    ex_a[39:32], ex_a[47:40], ex_a[55:48], ex_a[63:56]};
               4'd7:  exo_result = f_bitrev64(ex_a);
               4'd8:  exo_result = {57'b0, f_popcnt64(ex_a)};
               4'd9:  exo_result = {57'b0, f_clz64(ex_a)};
               4'd10: exo_result = {57'b0, f_ctz64(ex_a)};
               default: exo_result = {flags_eff.zero, flags_eff.zero, flags_eff.carry, flags_eff.overflow, 60'b0}; // GETF
            endcase
            if (ex_d.sub == 4'd4) exo_fval.sign = exo_result[63];
         end
         U_LEA:   exo_result = {32'b0, ex_pc + ex_var1};
         U_MOV:   exo_result = ex_imm;
         U_MOV64: exo_result = {ex_var2, ex_var1};
         U_BEXTR: begin
            bx_start = ex_var1[4:0];  bx_len = ex_var1[12:8];
            bx_mask  = (32'hFFFFFFFF >> (32 - bx_len));
            bx_res   = (ex_a >> bx_start) & bx_mask;
            exo_result = {32'b0, bx_res};
         end
         U_BDEP: begin
            bx_start = ex_var1[4:0];  bx_len = ex_var1[12:8];
            bx_mask  = (32'hFFFFFFFF >> (32 - bx_len)) << bx_start;
            bx_res   = (ex_b[31:0] << bx_start) & bx_mask;
            exo_result = {32'b0, (ex_a[31:0] & ~bx_mask) | bx_res};
         end
         U_STORE: begin
            exo_raw = ex_c;
            case (ex_d.msz)
               2'd0: begin
                  exo_wdata = {8{ex_c[7:0]}};
                  exo_be    = 8'b0000_0001 << exo_eaddr[2:0];
               end
               2'd1: begin
                  exo_eaddr = {exo_eaddr[31:1], 1'b0};
                  exo_wdata = {4{ex_c[15:0]}};
                  exo_be    = 8'b0000_0011 << {exo_eaddr[2:1], 1'b0};
               end
               2'd2: begin
                  exo_eaddr = {exo_eaddr[31:2], 2'b00};
                  exo_wdata = {ex_c[31:0], ex_c[31:0]};
                  exo_be    = exo_eaddr[2] ? 8'b1111_0000 : 8'b0000_1111;
               end
               default: begin
                  if (ex_d.malign) exo_eaddr = {exo_eaddr[31:3], 3'b000};
                  exo_wdata = ex_c;  exo_be = 8'hFF;
               end
            endcase
         end
         U_PUSH: begin
            exo_eaddr  = ex_sp_r - 32'd8;
            exo_wdata  = (ex_d.len == 2'b01) ? ex_a
                       : (ex_d.len == 2'b10) ? {32'b0, ex_var1} : {ex_var2, ex_var1};
            exo_raw    = exo_wdata;  exo_be = 8'hFF;
            exo_sp_new = ex_sp_r - 32'd8;
         end
         U_SC: begin                             // MMU stage 1: data = rs2
            exo_raw = ex_b;
            if (ex_d.msz == 2'd2) begin
               exo_wdata = {ex_b[31:0], ex_b[31:0]};
               exo_be    = exo_eaddr[2] ? 8'b1111_0000 : 8'b0000_1111;
            end else begin
               exo_wdata = ex_b;  exo_be = 8'hFF;
            end
            exo_result = {63'b0, !ex_sc_ok};     // 0 = stored, 1 = failed
         end
         U_POP, U_RET, U_IRET: begin
            exo_eaddr  = ex_sp_r;
            exo_sp_new = ex_sp_r + 32'd8;
         end
         U_ENTER: begin                          // push R15; R15 = SP; SP -= 8N
            exo_eaddr  = ex_sp_r - 32'd8;
            exo_wdata  = ex_a;  exo_raw = ex_a;  exo_be = 8'hFF;
            exo_result = {32'b0, ex_sp_r - 32'd8};    // new R15 (forwards from MEM)
            exo_sp_new = ex_sp_r + ex_var1;           // ex_var1 = -(8 + 8N)
         end
         U_LEAVE, U_LEAVERET: begin              // SP = R15; pop R15 (; pop PC)
            exo_eaddr  = ex_a[31:0];
            exo_sp_new = ex_a[31:0] + ((ex_d.uop == U_LEAVERET) ? 32'd16 : 32'd8);
         end
         U_GETSP: exo_result = {32'b0, ex_sp_r};
         U_SETSP: exo_sp_new = ex_a[31:0];
         U_ADDSP: exo_sp_new = ex_sp_r + ex_var1;   // sext imm32 added to 32-bit SP
         U_JMP: begin
            exo_taken  = ex_bcvt_t[0];
            exo_target = ex_d.brind ? ex_b[31:0]
                       : ex_d.brel  ? (ex_pc + ex_var1) : ex_var1;
         end
         U_CALL: begin
            exo_taken  = ex_bcvt_t[0];
            exo_target = ex_d.brind ? ex_b[31:0]
                       : ex_d.brel  ? (ex_pc + ex_var1) : ex_var1;
            exo_eaddr  = ex_sp_r - 32'd8;
            exo_wdata  = {32'b0, ex_pc_next};   // return address
            exo_raw    = exo_wdata;  exo_be = 8'hFF;
            exo_sp_new = ex_sp_r - 32'd8;
            exo_sp_we  = ex_bcvt_t[0];          // only a taken call touches SP/mem
         end
         U_MUL: begin
            exo_result    = ex_d.mdhigh ? mul_pipe2[127:64] : mul_pipe2[63:0];
            exo_fval.sign = exo_result[63];
            if (ex_d.mdhigh)      exo_fval.overflow = 1'b0;
            else if (ex_d.mduns)  exo_fval.overflow = (mul_pipe2[127:64] != 64'd0);
            else                  exo_fval.overflow = (mul_pipe2[127:64] != {64{mul_pipe2[63]}});
         end
         U_DIV, U_MOD: begin
            if (ex_div_by0) begin
               exo_result        = ex_div_mod ? ex_a : 64'hFFFFFFFFFFFFFFFF;
               exo_fmask         = '0;
               exo_fmask.overflow = 1'b1;
               exo_fval.overflow = 1'b1;
            end else begin
               q_raw  = ex_div_mod ? dv.remainder : dv.quotient;
               q_neg  = dv.is_signed && (ex_div_mod ? dv.sign_r : dv.sign_q);
               exo_result        = q_neg ? (~q_raw + 64'd1) : q_raw;
               exo_fval.overflow = 1'b0;
            end
         end
         U_RESET: begin exo_taken = 1'b1; exo_target = 32'h4; end
         U_DI:    exo_result = {63'b0, !ex_dis_eff};   // old enable bit
         // MMU stage 1: MFSR STATUS = {DI, UM}; DI forwarded from an older
         // DI / EI in MEM / WB (program order), UM is stable (it only
         // changes with the pipe drained or behind a serializing IRET)
         U_GETST: exo_result = {54'b0, EN_TLB && te, EN_MPU && mpu_en, 6'b0, ex_dis_eff, umode};
         default: ;   // NOP/HALT/WAIT/TRAP/DELAY/LCD/LOAD/LOAD32/ILL: nothing in EX
      endcase
      // port issue address (loads align per f_ld_idx; stores already aligned)
      exo_iaddr = exo_eaddr;
      if (ex_d.uop == U_LOAD) case (ex_d.msz)
         2'd1:    exo_iaddr = {exo_eaddr[31:1], 1'b0};
         2'd2:    exo_iaddr = {exo_eaddr[31:2], 2'b00};
         2'd3:    exo_iaddr = ex_d.malign ? {exo_eaddr[31:3], 3'b000} : exo_eaddr;
         default: exo_iaddr = exo_eaddr;
      endcase
   end

   // HW proposal 8 / §3.3 misaligned trap detect.  The low two EA bits come
   // from their own 2-bit adds (off the 32-bit EA carry chain); the uop /
   // size terms are register-sourced.  U_LOAD32 (MEMGET32) is exempt.
   wire [1:0] ex_ea_lo = (ex_d.mmode == 2'b00) ? ex_a[1:0]
                       : (ex_d.mmode == 2'b01) ? ex_a[1:0] + ex_var1[1:0]
                       : (ex_d.mmode == 2'b10) ? ex_var1[1:0]
                       :                         ex_a[1:0] + ex_b[1:0];
   wire ex_misal = EN_MISAL_TRAP && ex_valid &&
                   (((ex_d.uop == U_LOAD || ex_d.uop == U_STORE) &&
                     ((ex_d.msz == 2'd1 && ex_ea_lo[0]) ||
                      (ex_d.msz == 2'd2 && ex_ea_lo != 2'b00))) ||
                    // MMU stage 1: LR/SC ([rs1] only) must be naturally aligned
                    (EN_EXC && (ex_d.uop == U_LR || ex_d.uop == U_SC) &&
                     ((ex_d.msz == 2'd2 && ex_a[1:0] != 2'b00) ||
                      (ex_d.msz == 2'd3 && ex_a[2:0] != 3'b000))));
   logic mem_misal;      // the MEM op is a converted misaligned access (parks 5)
   logic ev_misal;       // blocking event: a misaligned op entered MEM this edge
   // MMU stage 1: the raw effective address (a sub-word store's exo_eaddr
   // is aligned down) for EADDR; registered at EX -> MEM (D-path only)
   wire [31:0] ex_ea_raw = (ex_d.uop != U_STORE) ? exo_eaddr
                         : (ex_d.mmode == 2'b00) ? ex_a[31:0]
                         : (ex_d.mmode == 2'b01) ? ex_a[31:0] + ex_var1
                         : (ex_d.mmode == 2'b10) ? ex_var1
                         :                         ex_a[31:0] + ex_b[31:0];
   logic        mem_misal_st;   // the converted misaligned op was a store
   logic [31:0] mem_ea_raw;
   // MMU stage 1 data-side access fault (MMU_PLAN.md §7.1): a user-mode
   // access to MMIO (0xF000_0000+), including the second doubleword of a
   // LEAVERET / spanning MEMGET32, or an LR/SC to MMIO in either mode.
   // Computed in EX from the EA and only REGISTERED (mem_xsq): the op
   // enters MEM converted to an exception op (no port access), and in its
   // first MEM cycle squashes EX / ID like a taken fused branch.
   wire ex_memop  = (ex_d.uop == U_LOAD || ex_d.uop == U_LOAD32 || ex_d.uop == U_STORE ||
                     ex_d.uop == U_PUSH || ex_d.uop == U_POP || ex_d.uop == U_RET ||
                     ex_d.uop == U_IRET || ex_d.uop == U_ENTER || ex_d.uop == U_LEAVE ||
                     ex_d.uop == U_LEAVERET || ex_d.uop == U_LR || ex_d.uop == U_SC ||
                     (ex_d.uop == U_CALL && ex_bcvt_t[0]));
   wire ex_mst    = (ex_d.uop == U_STORE || ex_d.uop == U_PUSH || ex_d.uop == U_CALL ||
                     ex_d.uop == U_ENTER || ex_d.uop == U_SC);
   wire ex_lrsc   = (ex_d.uop == U_LR || ex_d.uop == U_SC);
   wire ex_ea_f   = exo_eaddr[31:28] == 4'hF;
   wire ex_ea_xf  = (ex_d.uop == U_LEAVERET || (ex_d.uop == U_LOAD32 && exo_eaddr[2:0] > 3'd4)) &&
                    exo_eaddr[31:3] == 29'h1DFF_FFFF;     // the 2nd dword is 0xF000_0000
   // per doubleword: the stage 1 rule (u1 / u2) first, then the MPU (stage
   // 2) at the EA's granule and — for a two-access op whose 2nd dword is in
   // the next granule (EA = ex_a for both) — at that granule
   wire ex_s1    = EN_EXC && ex_valid && ex_memop && !ex_misal && (umode || ex_lrsc);
   wire ex_u1    = ex_s1 && ex_ea_f;
   wire ex_u2    = ex_s1 && ex_ea_xf;
   wire ex_mchk  = EN_MPU && !(EN_TLB && te) && ex_valid && ex_memop && !ex_misal;
   wire ex_x2c   = (ex_d.uop == U_LEAVERET || (ex_d.uop == U_LOAD32 && ex_a[2:0] > 3'd4)) &&
                   ex_a[4:3] == 2'b11;
   // MMU stage 3 (TE = 1): translate ex_ea_q — segment (§11.2), then the
   // TLB's data port.  Per access: segment ACCESS > TLB_MISS > ACCESS (TLB
   // permission).  The MPU is ignored while TE = 1.
   wire [31:0] tva    = ex_ea_q;
   wire        t_useg = !tva[31];
   wire        t_kseg = tva[31:27] == 5'b10000;                 // 0x8000_0000-0x87FF_FFFF
   wire        t_kmap = tva[31:30] == 2'b11 && tva[31:28] != 4'hF;
   wire        t_tlbs = t_useg || t_kmap;
   wire        t_seg  = (tva[31:30] == 2'b10 && !t_kseg) || (umode && !t_useg);
   wire        t_miss = t_tlbs && !tlb_d_hit;
   wire        t_perm = t_tlbs && tlb_d_hit && !(ex_mst ? tlb_d_xwr[1] : tlb_d_xwr[0]);
   wire        t_dn   = t_seg || t_miss || t_perm;
   wire [15:0] t_pahi = t_tlbs ? {5'b0, tlb_d_pfn} : t_kseg ? {5'b0, tva[26:16]} : tva[31:16];
   assign tlb_d_vpn = tva[31:16];
   logic        t1_dn, t1_seg, t1_miss, t1_tlb;   // the 1st page, for the 2nd-page cycle
   logic [15:0] t1_pahi;
   wire ex_tchk  = EN_TLB && te && ex_valid && ex_memop && !ex_misal;
   wire ex_x3    = EN_TLB && te && ex_xpg;                      // leaving in its 3rd cycle
   wire d1_dn    = ex_x3 ? t1_dn   : t_dn;
   wire d1_seg   = ex_x3 ? t1_seg  : t_seg;
   wire d1_miss  = ex_x3 ? t1_miss : t_miss;
   wire d1_tlb   = ex_x3 ? t1_tlb  : (t_miss || t_perm);
   wire [15:0] d1_pahi = ex_x3 ? t1_pahi : t_pahi;
   wire ex_dn1   = ex_u1 || (ex_tchk ? d1_dn : (ex_mchk && mpu_d1));
   wire ex_dn2   = ex_u2 || (ex_tchk ? (ex_x3 && t_dn) : (ex_mchk && ex_x2c && mpu_d2));
   wire ex_ufault = ex_dn1 || ex_dn2;
   wire ex_xg    = !ex_tchk &&
                   (ex_dn1 ? (!ex_u1 && ex_mchk && mpu_dg1) : (!ex_u2 && ex_mchk && ex_x2c && mpu_dg2));
   // the reported access's kind: a TLB miss (cause 11/12), and whether it
   // is TLB-related (miss or TLB permission: TLB_HI is auto-loaded)
   wire ex_xtm   = ex_tchk && (ex_dn1 ? (!ex_u1 && !d1_seg && d1_miss) : (!ex_u2 && !t_seg && t_miss));
   wire ex_xtlb  = ex_tchk && (ex_dn1 ? (!ex_u1 && !d1_seg && d1_tlb)
                                       : (!ex_u2 && !t_seg && (t_miss || t_perm)));
   // (for the early-issue ops — MEMGET32 [rs1], LEAVERET: EA = ex_a — the
   // same test from the register alone, off the EA adder)
   wire ex_uf_early = EN_EXC && umode && (ex_a[31:28] == 4'hF || ex_a[31:3] == 29'h1DFF_FFFF);
   logic mem_xsq;       // the MEM op is an access fault (no access; squash EX / ID)
   logic mem_xs_st, mem_xs2, mem_xs_g;   // ... a store / 2nd dword only / GUARD
   logic mem_xs_tm, mem_xs_tlb;          // ... a TLB miss / TLB-related (stage 3)
   logic [15:0] mem_pa2hi;               // (stage 3) the 2nd page's PA[31:16]
   logic        mem_xpg;                 // ... the op crosses a page
   // MEM -> WB exception record: wb_xc = the WB op is an exception op
   // (ILLEGAL / TRAP / misaligned / U_EXC), with its cause and EADDR
   logic        wb_xc;
   logic [3:0]  wb_xcause;
   logic [31:0] wb_xaddr, wb_pc_next;
   logic        wb_xguard;            // ECAUSE[8]: a GUARD denial (stage 2)
   logic        wb_mtsr;
   logic        wb_tlbi;              // a TLB instruction (MMU stage 3)
   logic        wb_xtlb;              // a TLB-related fault: TLB_HI auto-load
   logic        wb_um_we, wb_um_new;     // IRET: UM restore (+ stack swap)
   logic [31:0] wb_spalt_new;
   assign fe_um = umode || (wb_valid && wb_um_we && wb_um_new);
   // the WB exception op enters the handler (else it parks / debug-halts as
   // before): EVEC set, and not a debugger-attached ILLEGAL / TRAP /
   // MISALIGNED unless the debugger passes them on (EXC_PASS)
   wire wb_exc_go = EN_EXC && wb_xc && (evec != 32'b0) &&
                    !(DEBUG_EN && dbg_i.attached && !dbg_i.exc_pass &&
                      (wb_xcause == X_ILL || wb_xcause == X_TRAP ||
                       wb_xcause == X_MISAL_LD || wb_xcause == X_MISAL_ST));

   // mul/div operand B mux happens at dispatch: ex_b holds rs2 (len1) or
   // the sign/zero-extended imm32 (len2) — see the dispatch below.

   // ------------------------------------------------ MEM / IF port engines
   // mem_xc: 0=idle 1=wait-ready 2=settle(2nd MEMGET32 read, mirrors
   // f_memget32's dead cycle) 3=wait-ready-2. if_xc: fill in flight.
   logic [1:0] mem_xc;
   logic       if_xc;
   // 1c posted store: st_post = a retired store still draining on the port
   // (its DV held until ready).  st_post_ok = the store issued last edge may
   // post: DRAM, its line's tag not in the I-cache tag shadow, and no IFB /
   // NLB / ID / EX code overlap — all compared at issue from registers into
   // this flop, so the post decision (mem_done_now) reads registers only.
   // A code-touching store keeps the completion-time SMC squash path.
   logic       st_post, st_post_ok;
   // EN_SQ: the queued store (address, lane-replicated data, byte enables)
   logic        sq_v;
   logic [31:0] sq_addr;
   logic [63:0] sq_wdata;
   logic [7:0]  sq_be;
   // (stage 3) the VA page of the store on the port / in sq_*, for STLF with TE = 1
   logic [15:0] m_vahi, sq_vahi;
   logic        m_vav, sq_vav;
   // EN_ST1: st_spec = the store that left MEM last edge posted
   // speculatively (st_post_ok is its verdict this cycle); st_spec_nx = its
   // fall-through, the refetch pc if it was not postable
   logic        st_spec;
   logic [31:0] st_spec_nx;
   logic [28:0] if_req_dw;
   logic        if_rebase;
   // MMU stage 1: the fill in flight is a fake one (user fetch from 0xF...)
   logic        if_fake;
   logic        if_drop;      // MMU stage 3: the fill in flight must not reach the window
   logic        if_fflt, if_fmiss;   // (stage 3) the fake fill is a refused page / a TLB miss
   logic [15:0] if_tghi;             // (stage 3) the fill's I-cache tag [18:3] (PA[31:16] with TE)
   // (also, review 2 #1, in supervisor mode with the MPU on or TE = 1: no
   // fetch fill ever reads a device.  A GUARD over the address then
   // raises ACCESS_FETCH|GUARD at ID; otherwise the zeros are ILLEGAL.
   // With both off a supervisor fetch from MMIO reads it, as before.)
   wire         w_ufz = EN_EXC && (fe_um || (EN_MPU && mpu_en) || (EN_TLB && te)) &&
                        miss_dw[28:25] == 4'hF;
   assign bus_idle = (mem_xc == 2'd0) && !if_xc && !irq_xc && !st_post && !sq_v && !ld_out
                     && !m_read_DV && !m_write_DV;

   // Ready-EDGE arming: the real cache HOLDS ready high until its 2-FF sync
   // sees DV drop (then COOL_DOWN). A back-to-back request issued while the
   // previous transaction's ready is still high would otherwise "complete"
   // instantly with stale data. Every issue clears the arm (unless ready is
   // already low); the arm sets once ready has been observed low; completions
   // require ready AND armed. One flag suffices — one transaction in flight.

   wire mem_is_ld32  = mem_valid && (mem_d.uop == U_LOAD32);
   // 1a: a load that may leave MEM once issued (see EN_LD_WB)
   wire mem_fastld   = EN_LD_WB && mem_valid && !mem_misal && !mem_xsq && mem_iaddr[31:28] != 4'hF &&
                       (mem_d.uop == U_LOAD || mem_d.uop == U_POP || mem_d.uop == U_LEAVE);
   // ISA v3 B: taken fused branch in MEM (compare bit registered in EX)
   // fbr_tk = the fused branch in MEM is taken; fbr_taken = it REDIRECTS
   // (squashing EX/ID), i.e. it disagrees with the ID prediction (2b).
   // 1a: WB holds a fast load whose data has not arrived: nothing advances
   // (ready is earliest 2 cycles after DV, the load reaches WB after 1)
   wire wb_wait      = EN_LD_WB && wb_valid && wb_ld && !(ld_out && w_mrdy) &&
                       !(EN_DC_H && wb_h && wb_hok);
   // MMU stage 1: an access-fault op in MEM squashes EX / ID on the edge it
   // leaves MEM (as fbr_taken: only !wb_wait holds it there)
   wire mem_xsq_go   = EN_EXC && mem_valid && mem_xsq && !wb_wait;
   // 1a stage 3: a hit-pipe load that missed (or conflicted) waits in WB for
   // its port request — every younger port user holds off until it issues.
   wire wb_hmiss     = EN_DC_H && wb_valid && wb_h && !wb_hok && !ld_out;
   // ...or, faster, on the edge it leaves MEM: its first MEM cycle knows the
   // miss (h_ok), and with the port free nothing else issues that edge (the
   // MEM op is this load, early issue waits on mem_h, and an I-cache fill
   // only issues out of a lookup — so not while if_look: then the WB path).
   // The load then reaches WB with its request already out (ld_out).
   // (!ld_out && !wb_hmiss imply WB is not waiting, so it does leave MEM.)
   wire h_miss_now   = EN_DC_H && mem_valid && mem_h && mem_h1 && !h_ok && mem_d.uop != U_RET && !mem_xsq &&
                       !wb_hmiss && !if_xc && !if_look && !st_post && !sq_v && !ld_out && mem_xc == 2'd0 &&
                       !irq_xc && !(DEBUG_EN && dbg_mx);
   // (1a: a non-port op may now wait in MEM behind a fast load in WB —
   // act only on the edge it leaves MEM)
   wire fbr_tk       = EN_FBR && mem_valid && mem_d.uop == U_FBR && mem_result[0] && !wb_wait;
   wire fbr_taken    = EN_FBR && mem_valid && mem_d.uop == U_FBR && (mem_result[0] != (EN_BP && mem_ptk))
                       && !wb_wait;
   wire [31:0] fbr_tgt = mem_result[0] ? mem_eaddr : mem_pc_next;
   // EX redirect: a taken JMP/CALL/RESET, except a JMP/CALL already taken
   // by the ID prediction; a predicted-taken one that falls through
   // redirects to its fall-through.
   wire ex_bj        = (ex_d.uop == U_JMP || ex_d.uop == U_CALL);
   wire ex_redir     = exo_taken ^ (EN_BP && ex_ptk && ex_bj);
   wire [31:0] ex_redir_tgt = exo_taken ? exo_target : ex_pc_next;
   // 2a: a predicted return's actual target (as the RET redirect below takes
   // it) and the mispredict, valid on its completion edge (!mem_busy).
   // (EN_HRET: a RET answered by the hit pipe / a forwarded store has its
   // return address in the captured doubleword)
   wire        mem_ret_h  = EN_HRET && mem_valid && mem_d.uop == U_RET && mem_h && !mem_h1 && mem_hok;
   wire [31:0] ret_actual = mem_ret_h ? mem_hdw[31:0]
                          : (mem_d.uop != U_LEAVERET) ? m_rdata[31:0]
                          : (mem_xc == 2'd3) ? m_rdata[31:0] : m_rdata_next[31:0];

   // (EN_HRET: a hit-pipe RET is a port read only once its answer came back
   // a miss — the cycle after its first, with nothing captured)
   // (MMU: an access-faulted op — mem_xsq, registered in EX — makes no
   // port access, and the WB / redirect side ignores it: §7.1)
   wire mem_port_rd  = mem_valid && !mem_xsq && !(EN_DC_H && mem_h && !(mem_d.uop == U_RET && !mem_h1 && !mem_hok)) &&
                                    (mem_d.uop == U_LOAD || mem_d.uop == U_POP ||
                                     mem_d.uop == U_RET  || mem_d.uop == U_IRET ||
                                     mem_d.uop == U_LEAVE || mem_d.uop == U_LEAVERET ||
                                     mem_d.uop == U_LR || mem_is_ld32);
   wire mem_port_wr  = mem_valid && !mem_xsq && (mem_d.uop == U_STORE || mem_d.uop == U_PUSH ||
                                     mem_d.uop == U_ENTER ||
                                     (mem_d.uop == U_CALL && mem_sp_we) ||
                                     // MMU stage 1: an SC stores only on success
                                     // (its result, registered in EX, is mem_result[0] = 0)
                                     (mem_d.uop == U_SC && !mem_result[0]));
   wire mem_port_op  = mem_port_rd || mem_port_wr;
   // MEMGET32 spanning a dword: served by the read lookahead when the next
   // dword is in the same line, else a second read (f_memget32 exactly).
   wire ld32_span    = mem_is_ld32 && (mem_eaddr[2:0] > 3'd4);
   wire ld32_need2   = ld32_span && !m_next_valid;
   // LEAVERET: [R15+8] not in the same line -> second read (same path)
   wire lr_need2     = mem_valid && mem_d.uop == U_LEAVERET && !m_next_valid;
   wire mem_need2    = ld32_need2 || lr_need2;
   // EN_ST1: the store issues this edge onto a completely idle port (no
   // chaining — registers only) and leaves MEM with it
   wire st_spec_now  = EN_ST1 && EN_POST_ST && mem_valid && mem_port_wr && !mem_misal &&
                       mem_iaddr[31:28] != 4'hF && mem_xc == 2'd0 &&
                       !if_xc && !wb_hmiss && !ld_out && !st_post && !sq_v && !(DEBUG_EN && dbg_mx);
   // EN_SQ: ...or leaves MEM behind the posted store (into sq_*, or straight
   // onto the port if that one drains this edge)
   // (the EN_ST1 store that left MEM last edge turns out, this cycle, to
   // overlap fetch-side code: everything younger is squashed — see sq_spec
   // below the dispatch)
   wire sq_spec      = EN_ST1 && st_spec && !st_post_ok;
   // (not on the sq_spec edge: the MEM store is younger than the store that
   // squashes, and the squash never clears sq_v — it would still write)
   wire sq_enq_now   = EN_SQ && EN_ST1 && EN_POST_ST && mem_valid && mem_port_wr && !mem_misal &&
                       mem_iaddr[31:28] != 4'hF && mem_xc == 2'd0 &&
                       st_post && !sq_v && !if_xc && !wb_hmiss && !ld_out && !(DEBUG_EN && dbg_mx) &&
                       !sq_spec;
   wire mem_done_now = st_spec_now || sq_enq_now ||
                       (mem_xc == 2'd1 && w_mrdy && !mem_need2) ||
                       (mem_xc == 2'd3 && w_mrdy) ||
                       (mem_xc == 2'd1 && st_post_ok) ||  // 1c: posted (registered)
                       (mem_xc == 2'd1 && mem_fastld);    // 1a: issued; WB collects the data
   wire mem_port_wait = mem_port_op && !mem_done_now;
   // EN_HRET: a hit-pipe RET holds MEM for its first cycle (its return
   // address is checked from the captured answer, a register, the cycle after)
   // (not for an access-faulted op: nothing may hold MEM for it — its
   // squash, mem_xsq_go, acts on the edge it leaves MEM)
   wire ret_h_wait   = EN_HRET && mem_valid && mem_d.uop == U_RET && mem_h && mem_h1 && !mem_xsq;
   wire mem_busy     = mem_port_wait || wb_wait || ret_h_wait;
   wire ras_miss     = EN_RAS && mem_valid && mem_pred && !mem_busy && !mem_xsq && (ret_actual != mem_pred_tgt);
   wire ex_rd_op     = (ex_d.uop == U_LOAD || ex_d.uop == U_LOAD32 || ex_d.uop == U_POP ||
                        ex_d.uop == U_LEAVE || ex_d.uop == U_LEAVERET ||
                        ex_d.uop == U_RET || ex_d.uop == U_IRET);
   // (ex_misal is declared below; the early issue excludes it in the block)
   // 1a stage 3: the EX op enters MEM through the hit pipe (on its EX->MEM
   // edge).  Not while a store to the same 32 B line is on the port (latching
   // or in its CHECK write: the line write has not landed yet).
   // (an ex_hld load never early-issues, even when it cannot use the pipe —
   // MMIO, misaligned, a same-line store on the port: it issues from MEM a
   // cycle later.  Keeps h_elig's address compare out of the early-issue /
   // IF-issue cones, the build's worst paths.)
   wire ex_hld       = (ex_d.uop == U_LOAD || ex_d.uop == U_POP || ex_d.uop == U_LEAVE ||
                        (EN_HRET && ex_d.uop == U_RET));
   // EN_STLF: store -> load forwarding.  The youngest older store whose
   // dword is the load's — the one leaving MEM on this edge (sf_a), else the
   // one on the port (sf_b) — supplies the load's data when its byte enables
   // cover the load's bytes (its data is lane-replicated, so the covered lanes
   // are exactly the load's).  Such a load enters MEM as an answered hit-pipe
   // load.  A store to the SAME dword that does not cover keeps the load on
   // the slow path; a store to another dword of the line no longer does (it
   // changes only its own dword; the cache flags its line-write cycle).
   wire [7:0]  ex_ldmask  = (ex_d.uop != U_LOAD || ex_d.msz == 2'd3) ? 8'hFF
                          : (ex_d.msz == 2'd2) ? (exo_iaddr[2] ? 8'hF0 : 8'h0F)
                          : (ex_d.msz == 2'd1) ? (8'h03 << {exo_iaddr[2:1], 1'b0})
                          :                      (8'h01 << exo_iaddr[2:0]);
   // (stage 3, TE = 1: a store's port address is a PA and the load's a VA.
   // The block compares the page offset only — conservative for aliases.
   // A forward needs the youngest such store's VA page to equal the
   // load's: the MEM store's from mem_eaddr; the port's / queued one's from
   // m_vahi / sq_vahi, valid (m_vav / sq_vav) only for stores issued from
   // MEM and dropped by a TLB / ASID / TE change, which may remap the VA.)
   wire        sf_te      = EN_TLB && te;
   wire        sf_hia     = sf_te ? (mem_eaddr[31:16] == exo_iaddr[31:16])
                                  : (mem_iaddr[31:16] == exo_iaddr[31:16]);
   wire        sf_hib     = sf_te ? (m_vav && m_vahi == exo_iaddr[31:16])
                                  : (m_addr[31:16] == exo_iaddr[31:16]);
   wire        sf_hic     = sf_te ? (sq_vav && sq_vahi == exo_iaddr[31:16])
                                  : (sq_addr[31:16] == exo_iaddr[31:16]);
   wire        sf_a       = mem_valid && mem_port_wr && mem_iaddr[15:3] == exo_iaddr[15:3] &&
                            (sf_te || sf_hia);
   wire        sf_b       = m_write_DV && m_addr[15:3] == exo_iaddr[15:3] &&
                            (sf_te || sf_hib);
   // EN_SQ: the queued store (younger than the posted one on the port)
   wire        sf_c       = EN_SQ && sq_v && sq_addr[15:3] == exo_iaddr[15:3] &&
                            (sf_te || sf_hic);
   wire        sf_vok     = !sf_te || (sf_a ? sf_hia : sf_c ? sf_hic : sf_hib);
   wire        sf_cov     = sf_a ? ((mem_be & ex_ldmask) == ex_ldmask)
                          : sf_c ? ((sq_be  & ex_ldmask) == ex_ldmask)
                                 : ((m_be & ex_ldmask) == ex_ldmask);
   wire        h_line_blk = EN_STLF ? (sf_a || sf_b || sf_c)
                                    : ((m_write_DV && m_addr[31:5] == exo_iaddr[31:5]) ||
                                       (mem_valid && mem_port_wr && mem_iaddr[31:5] == exo_iaddr[31:5]));
   wire        h_base     = EN_DC_H && ex_valid && ex_hld && !ex_misal && exo_iaddr[31:28] != 4'hF &&
                            !(DEBUG_EN && dbg_mx);
   wire        h_fwd      = EN_STLF && h_base && (sf_a || sf_b || sf_c) && sf_cov && sf_vok;
   wire h_elig       = h_base && (!h_line_blk || h_fwd);
   // (stage 3: with TE = 1 the tag bits [31:16] are the PA — the cache
   // indexes with the VA bits [14:5] it always used: VIPT, 64 KiB pages)
   assign h_addr     = (EN_TLB && te) ? {t_pahi, exo_iaddr[15:0]} : exo_iaddr;
   wire w_early_ld   = EN_EARLY_LD && ex_valid && !ex_busy && !mem_busy && ex_rd_op && !(EN_DC_H && ex_hld) && !wb_hmiss &&
                       !(EN_DC_H && mem_valid && mem_h) &&   // its miss is not known yet
                       !mem_port_op && mem_xc == 2'd0 && !if_xc && !st_post && !sq_v && !irq_xc &&
                       !ld_out && !fbr_taken && !mem_xsq_go && !(DEBUG_EN && dbg_mx) &&
                       // (stage 2: no early issue with the MPU on — its
                       // decision must not reach the EX->MEM enables)
                       !xlt_q;
   wire mem_is_read  = mem_port_rd;
   wire mem_is_write = mem_port_wr;

   // ---- completing-store squashes (evaluated on the store's completion edge,
   // the same edge the EX op would advance into MEM) ----
   // SMC: a DRAM store overlapping the code of the younger op in EX or ID.
   wire        smc_st     = mem_done_now && mem_is_write && mem_iaddr[31:28] != 4'hF &&
                            !st_spec_now && !sq_enq_now;   // (EN_ST1 / EN_SQ: checked the cycle after, sq_spec)
   wire [28:0] smc_dw     = mem_eaddr[31:3];     // the store's VA (= mem_iaddr with TE = 0)
   wire        smc_hit_id = smc_st && id_valid &&
                            (id_pc[31:3] == smc_dw || id_last_dw == smc_dw);
   wire        smc_hit_ex = smc_st && ex_valid &&
                            (ex_pc[31:3] == smc_dw || ex_last_dw == smc_dw);
   // ICACHE_INV (CACHE_CTRL 0xF005_0000 bit 3, a fence.i): the SoC ignores
   // bit 3; the core decodes its own completing store. Invalidates every
   // fetch-side code copy and refetches everything younger — for code written
   // by non-snooping writers (DMA, core 2) and as an explicit loader fence.
   wire        icinv      = mem_done_now && mem_is_write && mem_iaddr == 32'hF005_0000 &&
                            mem_be[0] && mem_wdata[3];
   // Squash the EX op (it must not enter MEM: it was fetched before the
   // store) / anything younger than the store.
   wire        sq_ex      = smc_hit_ex || (icinv && ex_valid);
   wire        sq_any     = sq_ex || smc_hit_id || (icinv && id_valid);
   // The op being LATCHED into ID on this edge (from the IFB slot the store
   // invalidates on this same edge) is a stale copy too: smc_hit_id only
   // sees the op already in ID.  Its dword span, as id_last_dw computes it.
   wire [28:0] w_last_dw  = (w_op[31:30] == 2'b11) ? w_pc_p8[31:3]
                          : (w_op[31:30] == 2'b10) ? w_pc_p4[31:3] : pc_dw;
   wire        smc_hit_lat = smc_st && (pc_dw == smc_dw || w_last_dw == smc_dw);
   // 2b predictor training write (declared after mem_busy / ras_miss / sq_ex)
   wire pht_we_mem   = EN_BP && mem_valid && mem_bp_fbr && !wb_wait;   // as it leaves MEM
   wire pht_we_ex    = EN_BP && ex_valid && ex_bp_cnd && !ex_busy && !mem_busy &&
                       !fbr_taken && !sq_ex && !mem_xsq_go;   // (no ras_miss term: training only)
   assign pht_we = pht_we_mem || pht_we_ex;
   assign pht_wa = pht_we_mem ? mem_bp_idx : ex_bp_idx;
   assign pht_wd = pht_we_mem ? f_sat2(mem_bp_ctr, mem_result[0]) : f_sat2(ex_bp_ctr, ex_bcvt_t[0]);

   // M9+ slide enable (see slide_dw_ok above). Suppressed on a DRAM-store
   // COMPLETION edge: the SMC squash (end of the clocked block) indexes IFB
   // slots by the current base, and a same-edge slide would move the store's
   // dword out from under its invalidate — the slide just retries next cycle.
   // Issue-time store races need no gate: the issue-edge NLB kill + ic_v*
   // clear stop any re-capture of the stored line, and a pre-store copy slid
   // into the window is still at base/base+1 (or squashes via hit_id/hit_ex)
   // when the completion snoop lands — the same window the 1-cycle serve has.
   // (EN_ST1: a speculatively posted store has no completion-edge squash —
   // sq_spec, a cycle later, clears the whole IFB — so it does not gate.)
   wire w_slide = in1 && slide_dw_ok &&
                  !(mem_done_now && mem_is_write && mem_iaddr[31:28] != 4'hF && !st_spec_now && !sq_enq_now);
   assign pf_yield = !fetch_ok && !nlb_dw_ok && !w_slide;

   // branch / flush / fetch / port attribution (events and wait cycles)
   // An EX op killed this edge by an older fused branch or a store squash
   // never retires, so it is not counted.
   // JMP / CALL take exo_taken = ex_bcvt_t[0]; the counters read that
   // directly, keeping the counter adders off the exo_taken mux (which,
   // with EN_FBR_EQ_EX, also carries the 64-bit FBR equality reduce).  An
   // EX-resolved FBR is counted one cycle late from a register.
   logic r_perf_fbr_ex, r_perf_ras_miss, r_perf_ex_redir;
   always_ff @(posedge clk)
      if (rst) begin
         r_perf_fbr_ex <= 1'b0;  r_perf_ras_miss <= 1'b0;  r_perf_ex_redir <= 1'b0;
      end else if (ce) begin
         r_perf_fbr_ex <= EN_FBR_EQ_EX && ex_valid && !ex_busy && !mem_busy && exo_taken &&
                          ex_d.uop == U_FBR && !fbr_taken && !sq_ex;
         r_perf_ras_miss <= ras_miss;   // 2a: a mispredicted return redirects
         r_perf_ex_redir <= EN_BP && ex_valid && !ex_busy && !mem_busy && ex_redir && ex_bj &&
                            !fbr_taken && !ras_miss && !sq_ex;
      end
   // (with EN_BP, a redirect is a misprediction: the strobe registers it)
   assign perf_stall[5] = (!EN_BP && ex_valid && !ex_busy && !mem_busy && ex_bcvt_t[0] && !fbr_taken && !sq_ex
                           && ex_bj) || r_perf_ex_redir || fbr_taken || r_perf_fbr_ex
                           || r_perf_ras_miss;
   assign perf_stall[6] = running && !fetch_halt && !parked && !irq_active && !fetch_ok;
   assign perf_stall[7] = mem_busy;
   assign perf_br       = ex_valid && !ex_busy && !mem_busy && !fbr_taken && !sq_ex
                          && (ex_d.uop == U_JMP) && (ex_d.bcond != 4'd0);
   assign perf_br_taken = ex_bcvt_t[0];   // = exo_taken whenever perf_br (a JMP)
   assign perf_fbr_taken = fbr_tk ||      // MEM ops always commit: counted when taken
                           r_perf_fbr_ex; // 2b-lite: EX-resolved (registered)
   // (mirrors the if_look miss-issue condition in the fill engine)
   assign perf_ic[0] = !if_xc && if_look && ic_rd_current && !ic_hit &&
                       !mem_port_op && mem_xc == 2'd0 && !st_post && !sq_v && !ld_out && !irq_active && !w_early_ld &&
                       !wb_hmiss && !(DEBUG_EN && (dbg_active || dbg_halted));
   assign perf_ic[1] = if_xc;

   // ================= M12 Stage D: into-MEM 2-cycle shifter ==================
   // The 64-bit barrel LEAVES the EX cluster (exo_result's U_SHIFT leg is a
   // constant — the deepest cone and its fanout gone from the congested
   // ex_d[uop]->EX family, the sole default-strategy blocker). Stage 1
   // shifts by shcnt[2:0] into registers AT THE EX->MEM HANDOFF (gated, so
   // a shift stalled in MEM can never be clobbered by the next); stage 2
   // combines by the captured high bits DURING the MEM stage, purely
   // register-sourced; consumers read mem_result_eff. CPI-FREE BY
   // CONSTRUCTION: M6 forwarding already gives every producer-in-EX
   // consumer a 1-bubble gap, and the combine completes inside it — no
   // interlock, no sched-model change needed.
   wire mem_is_sh     = mem_valid && (mem_d.uop == U_SHIFT);
   wire ex_is_sh_hand = ex_valid && (ex_d.uop == U_SHIFT) && !ex_busy && !mem_busy;
   logic [63:0] r_shl1, r_shr1, r_sar1, r_rol1, r_ror1, r_oh1, r_btr1, r_sha_op;
   logic [2:0]  r_shhi, r_bthi;
   logic [3:0]  r_shsub;
   always_ff @(posedge clk) if (ce) begin
      if (ex_is_sh_hand) begin
         r_shl1  <= ex_a << ex_shcnt[2:0];
         r_shr1  <= ex_a >> ex_shcnt[2:0];
         r_sar1  <= $signed(ex_a) >>> ex_shcnt[2:0];
         r_rol1  <= (ex_a << ex_shcnt[2:0]) | (ex_a >> (7'd64 - {4'b0, ex_shcnt[2:0]}));
         r_ror1  <= (ex_a >> ex_shcnt[2:0]) | (ex_a << (7'd64 - {4'b0, ex_shcnt[2:0]}));
         r_oh1   <= 64'b1 << ex_shcnt[2:0];
         r_btr1  <= (ex_a >> ex_b[2:0]) | (ex_a << (7'd64 - {4'b0, ex_b[2:0]}));
         r_shhi  <= ex_shcnt[5:3];
         r_bthi  <= ex_b[5:3];
         r_shsub <= ex_d.sub;
         r_sha_op<= ex_a;
      end
   end
   logic [63:0] w_sh2;
   always_comb begin : sh_stage2
      logic [6:0]  hi8, bhi8;
      logic [63:0] btr_full;
      hi8  = {1'b0, r_shhi, 3'b000};
      bhi8 = {1'b0, r_bthi, 3'b000};
      btr_full = (r_btr1 >> bhi8) | (r_btr1 << (7'd64 - bhi8));
      case (r_shsub)
         4'd0:  w_sh2 = r_shl1 << hi8;
         4'd1:  w_sh2 = r_shr1 >> hi8;
         4'd2:  w_sh2 = $signed(r_sar1) >>> hi8;
         4'd3:  w_sh2 = (r_rol1 << hi8) | (r_rol1 >> (7'd64 - hi8));
         4'd4:  w_sh2 = (r_ror1 >> hi8) | (r_ror1 << (7'd64 - hi8));
         4'd8:  w_sh2 = r_sha_op |  (r_oh1 << hi8);
         4'd9:  w_sh2 = r_sha_op & ~(r_oh1 << hi8);
         4'd10: w_sh2 = r_sha_op ^  (r_oh1 << hi8);
         default: w_sh2 = {63'b0, btr_full[0]};   // BTSTRR
      endcase
   end
   // v3 D3: a W op's result is its low 32 bits sign-extended — applied here,
   // on the registered MEM value both forwarding and WB read (only the
   // upper-half select changes; EX is untouched).
   // (timing: MIN/MAX's select is here — opa in mem_result, opb in
   // mem_wdata, the pick registered in EX; CPI-free like the shifter's 2nd
   // stage: a producer in EX already gives its consumer one bubble)
   logic mem_mmq, mem_mmpick;
   wire [63:0] mem_result_eff = mem_is_sh ? w_sh2
                              : mem_mmq ? (mem_mmpick ? mem_result : mem_wdata)
                              : mem_d.w32 ? {{32{mem_result[31]}}, mem_result[31:0]}
                              : mem_result;

   // load-value extraction (f_ld_idx / f_memget32 lane math)
   logic [63:0] mem_ldval;
   always_comb begin
      logic [7:0]  b8;
      logic [15:0] h16;
      logic [31:0] w32;
      b8  = m_rdata[(mem_eaddr[2:0] * 8)  +: 8];
      h16 = m_rdata[(mem_eaddr[2:1] * 16) +: 16];
      w32 = mem_eaddr[2] ? m_rdata[63:32] : m_rdata[31:0];
      mem_ldval = m_rdata;
      if (mem_is_ld32) begin
         case (mem_eaddr[2:0])
            3'd0: mem_ldval = {32'b0, m_rdata[31:0]};
            3'd1: mem_ldval = {32'b0, m_rdata[39:8]};
            3'd2: mem_ldval = {32'b0, m_rdata[47:16]};
            3'd3: mem_ldval = {32'b0, m_rdata[55:24]};
            3'd4: mem_ldval = {32'b0, m_rdata[63:32]};
            // spanning: phase-1 lookahead (rdata_next) or phase-3 second read
            // combined with the stashed dw0
            3'd5: mem_ldval = (mem_xc == 2'd3) ? {32'b0, m_rdata[7:0],  mem_dw0[63:40]}
                                               : {32'b0, m_rdata_next[7:0],  m_rdata[63:40]};
            3'd6: mem_ldval = (mem_xc == 2'd3) ? {32'b0, m_rdata[15:0], mem_dw0[63:48]}
                                               : {32'b0, m_rdata_next[15:0], m_rdata[63:48]};
            default: mem_ldval = (mem_xc == 2'd3) ? {32'b0, m_rdata[23:0], mem_dw0[63:56]}
                                                  : {32'b0, m_rdata_next[23:0], m_rdata[63:56]};
         endcase
      end else if (mem_valid && mem_d.uop == U_LEAVERET) begin
         mem_ldval = (mem_xc == 2'd3) ? mem_dw0 : m_rdata;   // saved R15
      end else if (mem_valid && (mem_d.uop == U_LOAD || mem_d.uop == U_LR)) begin
         case (mem_d.msz)
            2'd0: mem_ldval = mem_d.sgn ? {{56{b8[7]}},   b8}  : {56'b0, b8};
            2'd1: mem_ldval = mem_d.sgn ? {{48{h16[15]}}, h16} : {48'b0, h16};
            2'd2: mem_ldval = mem_d.sgn ? {{32{w32[31]}}, w32} : {32'b0, w32};
            default: mem_ldval = m_rdata;
         endcase
      end
   end

   // 1a: a fast load's value at its WB commit (the response this cycle)
   function automatic logic [63:0] f_ldext(logic [63:0] dw, logic [2:0] ea, logic [1:0] msz,
                                           logic sgn, logic full);
      logic [7:0] b8; logic [15:0] h16; logic [31:0] w32;
      b8  = dw[(ea * 8) +: 8];
      h16 = dw[(ea[2:1] * 16) +: 16];
      w32 = ea[2] ? dw[63:32] : dw[31:0];
      if (full) return dw;
      case (msz)
         2'd0:    return sgn ? {{56{b8[7]}},   b8}  : {56'b0, b8};
         2'd1:    return sgn ? {{48{h16[15]}}, h16} : {48'b0, h16};
         2'd2:    return sgn ? {{32{w32[31]}}, w32} : {32'b0, w32};
         default: return dw;
      endcase
   endfunction
   // (stage 3: a hit-pipe hit's doubleword, else the port response)
   logic [63:0] wb_ldval;
   // (EN_MUL2: a MUL in WB taking its product from the DSP joins here, the
   // first select of the load-forward path, as a full-dword "load" — no
   // extra mux level at the end of the forward cone)
   wire  [63:0] wb_lddw = (EN_MUL2 && wb_mdir) ? mul_w[63:0]
                        : (EN_DC_H && wb_h && wb_hok) ? wb_hdw : m_rdata_dram;
   always_comb begin
      logic [7:0]  b8;
      logic [15:0] h16;
      logic [31:0] w32;
      b8  = wb_lddw[(wb_ld_ea * 8) +: 8];
      h16 = wb_lddw[(wb_ld_ea[2:1] * 16) +: 16];
      w32 = wb_ld_ea[2] ? wb_lddw[63:32] : wb_lddw[31:0];
      if (wb_ld_full || (EN_MUL2 && wb_mdir)) wb_ldval = wb_lddw;
      else case (wb_ld_msz)
         2'd0:    wb_ldval = wb_ld_sgn ? {{56{b8[7]}},   b8}  : {56'b0, b8};
         2'd1:    wb_ldval = wb_ld_sgn ? {{48{h16[15]}}, h16} : {48'b0, h16};
         2'd2:    wb_ldval = wb_ld_sgn ? {{32{w32[31]}}, w32} : {32'b0, w32};
         default: wb_ldval = wb_lddw;
      endcase
   end

   // the WB forwarding source: a fast load's response value (1a stage 2)
   wire [63:0] wb_fwd_v = ((EN_LD_FWD && wb_ld) || (EN_MUL2 && wb_mdir)) ? wb_ldval : wb_value;
   flags_t wb_fval_eff;
   always_comb begin
      wb_fval_eff = wb_fval;
      if (EN_MUL2 && wb_mdir) begin
         wb_fval_eff.sign     = mul_w[64];
         wb_fval_eff.overflow = mul_w[65];
      end
   end

   // IRET context split (klauss_pkg f_iret bit layout)
   flags_t iret_fval;
   always_comb begin
      iret_fval          = '0;
      iret_fval.zero     = m_rdata[38];
      iret_fval.carry    = m_rdata[36];
      iret_fval.overflow = m_rdata[35];
      iret_fval.sign     = m_rdata[34];
   end

   // LCD port
   assign lcd_byte = (ex_d.len == 2'b01) ? ex_a[7:0] : ex_var1[7:0];
   assign lcd_dc   = ex_d.lcd_is_data;
   // Strobe only on the edge the LCD op leaves EX, as LCDRST does: while MEM
   // is busy EX holds, and a strobe then (with lcd_ready dropping behind it,
   // so ex_busy keeps the op in EX) sent the byte twice; a taken fused branch
   // or an EX squash kills the op, so it must not send.  (Nor the EN_ST1
   // squash: the op is refetched and would send again.)
   assign lcd_dv   = ex_is_lcd && lcd_ready && !mem_busy && !fbr_taken && !ras_miss && !sq_ex &&
                     !mem_xsq_go && !sq_spec;
   // A load's EX-side watchpoint compares (hit pipe, early issue) fire only
   // for a load that really leaves EX: not one an older op kills this edge,
   // nor one with an access fault (it never reads)
   wire ex_wp_ok   = !fbr_taken && !ras_miss && !sq_ex && !mem_xsq_go && !sq_spec && !ex_ufault;

   // M12 pos-tracker plumbing: event flags are BLOCKING-assigned inside the
   // clocked block (pure combinational locals to synthesis — never FFs);
   // r_ifq_p1 registers if_req_dw+1 at fill issue for the completion update.
   logic       ev_disp, ev_slide, ev_rebase, ev_fill, ev_pcz;
   logic       ev_bp;   // a breakpoint was taken in ID this edge (debug)
   logic [1:0] ev_adv;
   logic [28:0] r_ifq_p1;

   // ================================================================== clocked
   always_ff @(posedge clk) begin
      if (rst) begin
         pc <= '0; running <= 1'b0; fetch_halt <= 1'b0;
         id_valid <= 1'b0; ex_valid <= 1'b0; mem_valid <= 1'b0; wb_valid <= 1'b0;
         flags <= '0; sp <= SP_RESET; int_mask <= 4'h0; irq_dis <= 1'b0;
         umode <= 1'b0; sp_alt <= '0;
         res_valid <= 1'b0; res_dw <= '0; exc_ent <= 1'b0; mem_xsq <= 1'b0;
         id_fx <= 1'b0; id_fxc <= 1'b0; id_mfsr <= 1'b0; wb_xc <= 1'b0; wb_mtsr <= 1'b0;
         wb_um_we <= 1'b0; wb_lr <= 1'b0; wb_sc <= 1'b0; mem_mmq <= 1'b0;
         m_vav <= 1'b0; sq_vav <= 1'b0; m_vahi <= '0; sq_vahi <= '0;
         ut_v <= 1'b0; it_ref <= 1'b0; it_vpn <= '0; it_asid <= '0; ut_vpn <= '0; ut_pa <= '0; ut_um <= 1'b0; ut_ok <= 1'b0; ut_miss <= 1'b0;
         ifb_ft <= 2'b00; ifb_fm <= 2'b00; if_fflt <= 1'b0; if_fmiss <= 1'b0;
         id_fx2 <= 1'b0; id_fxg <= 1'b0; id_fxm <= 1'b0; id_fxl <= 1'b0; wb_xguard <= 1'b0; exc_guard_q <= 1'b0; ex_mph <= 1'b0;
         ex_mph2 <= 1'b0; exc_tlb_q <= 1'b0; wb_xtlb <= 1'b0;
         mem_dis_we <= 1'b0; wb_dis_we <= 1'b0; mem_misal <= 1'b0;
         st_post <= 1'b0; st_post_ok <= 1'b0; ld_out <= 1'b0; wb_ld <= 1'b0; st_spec <= 1'b0; sq_v <= 1'b0;
         mem_h <= 1'b0; mem_h1 <= 1'b0; wb_h <= 1'b0; wb_hok <= 1'b0; mem_hok <= 1'b0; mem_fq <= 1'b0;
         wb_mul <= 1'b0; wb_mdir <= 1'b0; mem_mage <= 2'd0;
         mem_lkq <= 1'b0; mem_mulq <= 1'b0; mem_mulf <= 1'b0; wb_mulf <= 1'b0;
         mem_fastq <= 1'b0; ex_lb_a <= 1'b0; ex_lb_b <= 1'b0; id_ldc <= 1'b0;
         ras_ptr <= '0; ras_cnt <= '0; id_ras_ret <= 1'b0; ex_pred <= 1'b0; mem_pred <= 1'b0;
         id_bp_unc <= 1'b0; id_bp_cnd <= 1'b0; id_bp_fbr <= 1'b0; ex_ptk <= 1'b0; mem_ptk <= 1'b0;
         bt_v <= '0; r_bt_pend <= 1'b0; id_bt_hit <= 1'b0; id_rpend <= 1'b0;
         r_btk_icv <= 1'b0; r_btk_snv <= 1'b0; r_btk_all <= 1'b0;
         ex_bp_cnd <= 1'b0; mem_bp_fbr <= 1'b0;
         mul_cnt <= '0; div_ph <= MD_IDLE; dv <= '0;
         dly_on <= 1'b0; dly_cnt <= '0; dly_max <= '0;
         parked <= 1'b0; park_kind <= '0; park_pc <= '0; park_op <= '0;
         ret_valid <= 1'b0; ret_wr <= 1'b0; lcd_rst_n <= 1'b0;
         mem_xc <= '0; if_xc <= 1'b0; if_look <= 1'b0; ifb_val <= 2'b00; if_fake <= 1'b0; if_drop <= 1'b0;
         nlb_vhi <= 1'b0; nlb_vlo <= 1'b0; pf_look <= 1'b0; nlb1_vhi <= 1'b0; nlb1_vlo <= 1'b0;
         pos <= 3'b000; r_base_p1 <= 29'd1; r_base_p2 <= 29'd2; r_want_line <= 28'd1;
         m_read_DV <= 1'b0; m_write_DV <= 1'b0; m_addr <= '0; m_be <= 8'hFF;
         irq_active <= 1'b0; irq_xc <= 1'b0; irq_ack <= 1'b0;
         rdy_armed <= 1'b0; ex_fwd_f <= 1'b0; snoop_q <= 1'b0;
         dbg_active <= 1'b0; dbg_cause_q <= '0; dbg_cause <= '0;
         dbg_stepping <= 1'b0; dbg_step_done <= 1'b0; dbg_skip <= 1'b0; id_bp <= 1'b0;
         dbg_watch_req <= 1'b0; dbg_watch_addr <= '0;
         dbg_mx <= 1'b0; dbg_mem_done <= 1'b0; dbg_mem_rdata <= '0;
         id_use_rs1 <= 1'b0; id_use_rs2 <= 1'b0; id_use_rdd <= 1'b0;
         id_fread <= 1'b0; id_sp_rw <= 1'b0;
         id_b_one <= 1'b0; id_b_imm <= 1'b0; id_b_sext <= 1'b0; id_w <= 1'b0; id_ser <= 1'b0;
         id_rs1 <= 4'd0; id_rd <= 4'd0;
         for (int i = 0; i < 16; i++) rf[i] <= 64'b0;
         for (int i = 0; i < IC_LINES; i++) begin ic_vhi[i] <= 1'b0; ic_vlo[i] <= 1'b0; end
      end else if (ce) begin
         // M12 pos-tracker event flags: cleared here, BLOCKING-set by the
         // real write sites below, folded by the pos_track composer at the
         // end of this block. Same-process sampling — no mirrors, no drift.
         ev_disp = 1'b0; ev_slide = 1'b0; ev_rebase = 1'b0;
         ev_fill = 1'b0; ev_pcz = 1'b0; ev_adv = 2'd0; ev_bp = 1'b0; ev_misal = 1'b0;
         ev_btic = 1'b0; ev_ovr = 1'b0; ev_icinst = 1'b0; ev_icinst_idx = '0;

         // MMU stage 1: a program load starts in supervisor mode with no
         // vector (MMU_PLAN.md §3.1); a debugger resume keeps them
         if (EN_EXC && start) begin
            umode <= 1'b0;                 // (EVEC / ECAUSE / EADDR: mmu_sr)
            res_valid <= 1'b0; exc_ent <= 1'b0;
         end
         if (start || (dbg_halted && dbg_i.resume)) begin
            ev_pcz = 1'b1;
            pc <= (DEBUG_EN && !start) ? park_pc : start_pc;
            running <= 1'b1; fetch_halt <= 1'b0; parked <= 1'b0;
            id_valid <= 1'b0; ex_valid <= 1'b0; mem_valid <= 1'b0; wb_valid <= 1'b0;
            ifb_val <= 2'b00; if_look <= 1'b0;
            nlb_vhi <= 1'b0; nlb_vlo <= 1'b0; pf_look <= 1'b0; nlb1_vhi <= 1'b0; nlb1_vlo <= 1'b0;
            bt_v <= '0;  r_bt_pend <= 1'b0;   // 2a: the BTIC is a code copy too
            ut_v <= 1'b0;  it_ref <= 1'b0;    // (stage 3: the micro-ITLB)
            // invalidate-all: a fresh program image may reuse code addresses
            for (int i = 0; i < IC_LINES; i++) begin ic_vhi[i] <= 1'b0; ic_vlo[i] <= 1'b0; end
            if (DEBUG_EN) begin
               dbg_stepping  <= !start && dbg_i.step;
               dbg_step_done <= 1'b0;
               dbg_skip      <= !start;   // don't re-break on the resume instruction
               dbg_watch_req <= 1'b0;
               dbg_active    <= 1'b0;
            end
         end

         irq_ack <= 1'b0;
         dbg_mem_done <= 1'b0;

         // ready-edge arming (issues below re-clear it; NBA last-write wins)
         if (!m_ready) rdy_armed <= 1'b1;

         // WAIT wake: a parked WAIT resumes on a pending IRQ; the next dispatch
         // boundary takes the interrupt (resume PC = WAIT+4, as the FSM's
         // WAITING -> OPCODE_REQUEST path does).
         if (parked && park_kind == 3'd3 && irq_ok) begin
            parked     <= 1'b0;
            fetch_halt <= 1'b0;
         end
         if (DEBUG_EN && parked && park_kind == 3'd3) begin
            // A halt request stops a WAIT in place (resume re-executes it);
            // a step completes it as if woken (the interrupt stays pending).
            if (dbg_stepping) begin
               park_kind <= 3'd4;  dbg_cause <= DBG_CAUSE_STEP;  running <= 1'b0;
               park_pc   <= park_pc + 32'd4;
               dbg_stepping <= 1'b0;  dbg_step_done <= 1'b0;
            end else if (dbg_i.halt_req) begin
               park_kind <= 3'd4;  dbg_cause <= DBG_CAUSE_REQ;  running <= 1'b0;
            end
         end

         // Debug halt: the boundary was taken (dbg_active); park once drained.
         // A TRAP/HALT/ILLEGAL that parked first (at WB) wins.
         if (DEBUG_EN && dbg_active) begin
            if (parked) begin
               dbg_active <= 1'b0;
            end else if (!id_valid && !ex_valid && !mem_valid && !wb_valid
                         && mem_xc == 2'd0 && !if_xc && !st_post && !sq_v && !ld_out) begin
               dbg_active    <= 1'b0;
               parked        <= 1'b1;
               running       <= 1'b0;   // no dispatch / fetch until resume
               park_kind     <= 3'd4;
               park_pc       <= pc;
               park_op       <= '0;
               dbg_cause     <= dbg_cause_q;
               dbg_stepping  <= 1'b0;
               dbg_step_done <= 1'b0;
               dbg_watch_req <= 1'b0;
            end
         end

         // ---------------- IRQ entry sequencer ----------------
         if (irq_active) begin
            if (!irq_xc) begin
               // wait for full drain + a free port, then push the frame
               if (!id_valid && !ex_valid && !mem_valid && !wb_valid
                   && mem_xc == 2'd0 && !if_xc && !st_post && !sq_v && !ld_out) begin
                  // RE-CHECK at the drain boundary: an OLDER in-flight store
                  // may have completed during the shadow and killed the cause
                  // — an INT_MASK write (Zephyr irq_lock) or a level-IRQ ack
                  // (blitter DONE W1C). Entering the handler then would run it
                  // inside the critical section / dispatch a spurious IRQ.
                  // All older stores have retired here (pipeline empty,
                  // mem_xc==0), so irq_ready is now final for this boundary.
                  // (MMU stage 1: an exception entry, exc_ent, never abandons)
                  if (!(EN_EXC && exc_ent) && (!irq_ready || (EN_DIEI && irq_dis))) begin
                     // (7a: a DI that committed during the drain also abandons)
                     irq_active <= 1'b0;   // abandon: fetch resumes at pc
                  end else if (EN_TLB && te && push_va[31:27] != 5'b10000) begin
                     // MMU stage 3: with TE = 1 the frame push needs SP - 8 in
                     // kseg — otherwise DOUBLE_FAULT: park 6 (debug halt when
                     // attached), writing none of ECAUSE / EADDR / TLB_HI
                     irq_active <= 1'b0;
                     exc_ent    <= 1'b0;
                     parked     <= 1'b1;
                     park_kind  <= 3'd6;
                     park_pc    <= exc_ent ? exc_epc_q : pc;
                     park_op    <= 32'b0;
                     if (DEBUG_EN && dbg_i.attached) begin
                        park_kind <= 3'd4;  running <= 1'b0;  dbg_cause <= DBG_CAUSE_TRAP;
                        dbg_stepping <= 1'b0;  dbg_step_done <= 1'b0;
                     end
                  end else begin
                     // Re-latch source+vector too: the drained stores may have
                     // retargeted the highest-priority pending source.
                     irq_sel_q  <= irq_sel;
                     irq_vec_q  <= irq_vector;
                     // MMU stage 1: from user mode the frame goes on the
                     // kernel SP (sp_alt); frame bit 44 = previous mode;
                     // an exception's frame PC is its EPC
                     // (stage 3: with TE = 1 the push is not translated: PA =
                     // SP - 8 - 0x8000_0000, SP - 8 in kseg — checked above)
                     m_addr     <= (EN_TLB && te) ? {5'b0, push_va[26:0]} : push_va;
                     m_wdata    <= {19'b0, EN_EXC && umode, EN_DIEI && irq_dis, int_mask,
                                    flags.zero, flags.zero, flags.carry,
                                    flags.overflow, flags.sign,
                                    (flags.sign ^ flags.overflow), flags.carry,
                                    (EN_EXC && exc_ent) ? exc_epc_q : pc};
                     m_be       <= 8'hFF;
                     m_write_DV <= 1'b1;
                     m_vav      <= 1'b0;
                     rdy_armed  <= !m_ready;
                     irq_xc     <= 1'b1;
                  end
               end
            end else if (w_mrdy) begin
               m_write_DV          <= 1'b0;
               // MMU stage 1: entry from user mode swaps the stack pointers
               // (sp = the kernel SP below the frame, sp_alt = the user SP)
               if (EN_EXC && umode) begin
                  sp     <= sp_alt - 32'd8;
                  sp_alt <= sp;
                  umode  <= 1'b0;
               end else
                  sp     <= sp - 32'd8;
               if (EN_EXC) res_valid <= 1'b0;
               ev_pcz              = 1'b1;
               if (EN_EXC && exc_ent) begin
                  // exception: interrupts off (the old DI is in the frame)
                  // (stage 3: a TLB miss goes to TLBVEC when that is set)
                  // (stage 3: chosen into exc_vec_q when the exception
                  // was latched — a register, so this pc leg is as it was)
                  pc          <= EN_TLB ? exc_vec_q : evec;
                  irq_dis     <= 1'b1;
                  // (ECAUSE / EADDR written by mmu_sr: sr_ent_we)
                  fetch_halt  <= 1'b0;
                  exc_ent     <= 1'b0;
               end else begin
                  int_mask[irq_sel_q] <= 1'b0;   // masked while handler runs; IRET restores
                  pc                  <= irq_vec_q;
                  irq_ack             <= 1'b1;
                  irq_ack_sel         <= irq_sel_q;
               end
               irq_xc              <= 1'b0;
               irq_active          <= 1'b0;
            end
         end

         // EN_LDBYP: a bypass consumer that does not leave EX on the edge its
         // load commits out of WB (a miss held it, or the pipe is frozen)
         // captures the load's value then and runs from ex_a / ex_b
         if (EN_LDBYP && ex_valid && (ex_lb_a || ex_lb_b) && wb_valid && !wb_wait &&
             (ex_busy || mem_busy)) begin
            if (ex_lb_a) ex_a <= ex_w ? {wb_ldval[31:0], 32'b0} : wb_ldval;
            if (ex_lb_b) ex_b <= ex_w ? {wb_ldval[31:0], 32'b0} : wb_ldval;
            ex_lb_a <= 1'b0;  ex_lb_b <= 1'b0;
         end

         // ---------------- WB: the single retire point ----------------
         ret_valid <= 1'b0;  ret_wr <= 1'b0;
         if (wb_valid && !wb_wait) begin
            // (1a: a fast load commits its value straight from the response)
            if (wb_wreg)    rf[wb_rd] <= ((EN_LD_WB && wb_ld) || (EN_MUL2 && wb_mdir)) ? wb_ldval : wb_value;
            if (wb_fmask != '0) begin
               flags <= (flags & ~wb_fmask) | (wb_fval_eff & wb_fmask);
               // Deferred Z (the FSM's set_zero): the 64-bit ==0 reduction
               // runs at the MEM->WB latch (wb_zero), not in EX behind the
               // shifter/adder (the WNS -0.351 path). Every Z-writer's zero is
               // (value==0) except BTSTN (~bit) and IRET (frame restore),
               // which keep wb_fval.zero. DIV negation preserves zero.
               if (wb_fmask.zero && wb_zval)
                  flags.zero <= (EN_MUL2 && wb_mdir) ? (mul_w[63:0] == 64'b0) : wb_zero;
            end
            if (wb_sp_we)   sp <= wb_sp_new;
            if (wb_mask_we) int_mask <= wb_mask_new;
            if (EN_DIEI && wb_dis_we) irq_dis <= wb_dis_new;
            // MMU stage 1: the LR/SC reservation (LR sets it, any SC clears it)
            if (EN_EXC && wb_lr) begin res_valid <= 1'b1; res_dw <= wb_va_dw; end
            if (EN_EXC && wb_sc) res_valid <= 1'b0;
            if (EN_EXC && wb_um_we) begin          // IRET (MMU stage 1)
               umode     <= wb_um_new;
               if (wb_um_new) sp_alt <= wb_spalt_new;
               res_valid <= 1'b0;
            end
            if (wb_park && wb_exc_go) begin
               // MMU stage 1: the exception op commits nothing and retires
               // no trace line; everything younger is already gone (it
               // serialized or squashed), fetch is halted.  The IRQ entry
               // sequencer pushes the frame and vectors to EVEC.
               irq_active  <= 1'b1;
               exc_ent     <= 1'b1;
               exc_cause_q <= wb_xcause;
               // (an older MTSR to EVEC / TLBVEC has committed: both are final)
               exc_vec_q   <= (EN_TLB && wb_xcause >= X_TLB_LD && wb_xcause <= X_TLB_FE &&
                               tlb_vec != 32'b0) ? tlb_vec : evec;
               exc_guard_q <= wb_xguard;
               exc_tlb_q   <= wb_xtlb;
               exc_addr_q  <= wb_xaddr;
               exc_epc_q   <= (wb_xcause == X_SYSCALL) ? wb_pc_next : wb_pc;
            end else if (wb_park) begin
               parked   <= 1'b1;
               park_kind <= wb_park_kind;
               park_pc  <= wb_pc;  park_op <= wb_op;
               if (DEBUG_EN && dbg_i.attached && wb_park_kind != 3'd3) begin
                  park_kind <= 3'd4;
                  running   <= 1'b0;
                  dbg_cause <= f_dbg_park_cause(wb_park_kind);
                  dbg_stepping <= 1'b0;  dbg_step_done <= 1'b0;
               end
               if (wb_park_kind != 3'd2 && wb_park_kind != 3'd5) begin   // illegal / misaligned: not executed, no trace line
                  ret_valid <= 1'b1; ret_pc <= wb_pc; ret_op <= wb_op;
               end
            end else begin
               ret_valid <= 1'b1; ret_pc <= wb_pc; ret_op <= wb_op;
               // MMU stage 1: MTSR writes its SR, then fetch restarts behind it
               if (EN_EXC && (wb_mtsr || wb_tlbi)) begin
                  // (USP here; every other SR is written by mmu_sr / mmu_mpu /
                  // mmu_tlb; a TLB instruction runs in mmu_tlb)
                  if (wb_mtsr && wb_op[19:12] == 8'd4) sp_alt <= wb_value[31:0];
                  if (tlb_discard) res_valid <= 1'b0;   // (stage 3: TLB / ASID / TE change)
                  ev_pcz     = 1'b1;
                  pc         <= wb_pc + 32'd4;
                  fetch_halt <= 1'b0;
               end
            end
            ret_wr      <= wb_wr;
            ret_wr_addr <= wb_wr_addr;  ret_wr_be <= wb_wr_be;  ret_wr_raw <= wb_wr_raw;
         end

         // ---------------- EX free-running units ----------------
         // (MMU stage 2 / 3: the MPU's and the TLB's data input — the EA,
         // captured in the op's first EX cycle; for a page-crossing two-
         // access op, the next page in its 3rd)
         if (!ex_mph)                                  ex_ea_q <= exo_eaddr;
         else if (ex_mpu_hold && EN_TLB && te && ex_xpg) ex_ea_q <= {ex_ea_q[31:16] + 16'd1, 16'h0000};
         // (multiply chain lives in its own always block above — timing)

         case (div_ph)
            MD_IDLE: if (ex_is_div && ex_b != 64'd0) begin
               dv.dividend  <= w_div_abs_a;   dv.divisor   <= w_div_abs_b;
               dv.quotient  <= '0;            dv.remainder <= '0;   dv.counter <= '0;
               dv.sign_q    <= ex_a[63] ^ ex_b[63];
               dv.sign_r    <= ex_a[63];
               dv.is_signed <= ex_d.sgn;
               dv.op        <= ex_div_mod ? DIV_OP_MOD : DIV_OP_DIV;
               div_ph       <= MD_PREP;
            end
            MD_PREP: begin : divide_prep
               logic [6:0] prep_clz;
               prep_clz = f_clz64(dv.dividend);
               if (prep_clz[6]) dv.counter <= 7'd64;
               else begin
                  dv.dividend <= dv.dividend << prep_clz[5:0];
                  dv.counter  <= {1'b0, prep_clz[5:0]};
               end
               div_ph <= MD_STEP;
            end
            MD_STEP: begin
               if (dv.counter < 7'd64) begin
                  if (!w_div_borrow) begin
                     dv.remainder <= w_div_trial[63:0];
                     dv.quotient  <= {dv.quotient[62:0], 1'b1};
                  end else begin
                     dv.remainder <= w_div_shifted;
                     dv.quotient  <= {dv.quotient[62:0], 1'b0};
                  end
                  dv.dividend <= {dv.dividend[62:0], 1'b0};
                  dv.counter  <= dv.counter + 7'd1;
               end else if (!mem_busy) begin
                  // finish only when the pipe can actually advance the div out
                  // of EX — else md_done must stay asserted (a MEM-busy freeze
                  // on the finish cycle would otherwise re-issue the divide).
                  dv.op  <= DIV_OP_NONE;
                  div_ph <= MD_IDLE;
               end
            end
            default: div_ph <= MD_IDLE;
         endcase

         if (ex_is_dly) begin
            if (!dly_on) begin
               dly_on  <= 1'b1;  dly_cnt <= '0;
               dly_max <= {(ex_d.len == 2'b01) ? ex_a[31:0] : ex_var1, 13'b0};
            end else if (dly_done) begin
               if (!mem_busy) dly_on <= 1'b0;   // same freeze-on-finish rule
            end else dly_cnt <= dly_cnt + 45'd1;
         end

         lcd_rst_wr <= 1'b0;
         if (ex_valid && ex_d.uop == U_LCDRST && !mem_busy && !fbr_taken && !ras_miss && !sq_ex &&
             !mem_xsq_go && !sq_spec) begin
            lcd_rst_n  <= ex_var1[0];
            lcd_rst_wr <= 1'b1;
         end

         // M12 snoop stage 2: the registered tag-match clears the line (the
         // issue below re-arms snoop_q; NBA last-write wins).
         snoop_q <= 1'b0;
         if (snoop_q) begin
            ic_vhi[snoop_idx_q] <= 1'b0;
            ic_vlo[snoop_idx_q] <= 1'b0;
         end

         // 1c: posted-store drain.  Nothing else can be on the port (every
         // issue point waits for !st_post), so this ready is the store's.
         if (st_post && w_mrdy) begin
            m_write_DV <= 1'b0;
            st_post    <= 1'b0;
            // EN_SQ: the queued store follows on the same edge (the cache
            // needs a DV-low cycle between two requests to one address)
            if (sq_v && sq_addr != m_addr) begin
               m_addr     <= sq_addr;
               m_wdata    <= sq_wdata;
               m_be       <= sq_be;
               m_write_DV <= 1'b1;
               m_vahi     <= sq_vahi;  m_vav <= sq_vav;
               rdy_armed  <= !m_ready;
               st_post    <= 1'b1;
               sq_v       <= 1'b0;
            end
         end
         // ...or on an idle port (the same-address case, a cycle later)
         if (EN_SQ && sq_v && !st_post && !if_xc && !ld_out && mem_xc == 2'd0) begin
            m_addr     <= sq_addr;
            m_wdata    <= sq_wdata;
            m_be       <= sq_be;
            m_write_DV <= 1'b1;
            m_vahi     <= sq_vahi;  m_vav <= sq_vav;
            rdy_armed  <= !m_ready;
            st_post    <= 1'b1;
            sq_v       <= 1'b0;
         end

         // 1a: the fast load's response arrives (its WB commit is this edge)
         if (ld_out && w_mrdy) begin
            m_read_DV <= 1'b0;
            ld_out    <= 1'b0;
         end

         // 1a stage 3: a hit-pipe load that missed issues its port request
         // from WB once the port is free; from here it is a 1a fast load
         // (ld_out, WB collects the response).  Younger port users wait on
         // wb_hmiss, so it is the oldest request.
         if (wb_hmiss && !if_xc && !st_post && !sq_v && mem_xc == 2'd0 && !irq_xc &&
             !(DEBUG_EN && dbg_mx)) begin
            m_addr    <= wb_iaddr;
            m_be      <= wb_be;
            m_read_DV <= 1'b1;
            rdy_armed <= !m_ready;
            ld_out    <= 1'b1;
         end
         if (h_miss_now) begin
            m_addr    <= mem_iaddr;
            m_be      <= mem_be;
            m_read_DV <= 1'b1;
            rdy_armed <= !m_ready;
            ld_out    <= 1'b1;
         end

         // EN_ST1: the speculative post's verdict (st_post_ok) is consumed
         // this cycle (sq_spec, below); a new issue below re-sets both
         st_spec <= 1'b0;
         if (st_spec) st_post_ok <= 1'b0;

         // ---------------- MEM port engine (extra_clock pattern) ----------------
         // (nothing issues for a MEM op the EN_ST1 squash kills this edge: a
         // store chained onto the draining posted store would write, and any
         // issue would leave mem_xc busy with no MEM op behind it)
         if (mem_port_op && !sq_spec) begin
            case (mem_xc)
               // wait for the port (IF / a posted store / a fast load's read);
               // 1a: chain onto a fast load's completion edge when the address
               // differs (DV may stay high — the cache's served-DV guard then
               // sees a new request; the same address needs a DV-low cycle)
               // (stage 2: also onto a posted store's drain edge)
               2'd0: if ((!if_xc && !wb_hmiss && !sq_v &&
                          (!(ld_out || st_post) || (w_mrdy && mem_iaddr != m_addr))) || sq_enq_now) begin
                  // (EN_SQ: a store queued behind the posted one goes on the
                  // port now only if that one drains this edge to another
                  // address; else it waits in sq_*)
                  if (!sq_enq_now || (w_mrdy && mem_iaddr != m_addr)) begin
                     m_addr     <= mem_iaddr;
                     m_wdata    <= mem_wdata;
                     m_be       <= mem_be;
                     m_read_DV  <= mem_port_rd;
                     m_write_DV <= mem_port_wr;
                     m_vahi     <= mem_eaddr[31:16];  m_vav <= 1'b1;
                     rdy_armed  <= !m_ready;
                  end else begin
                     sq_v       <= 1'b1;
                     sq_addr    <= mem_iaddr;
                     sq_vahi    <= mem_eaddr[31:16];  sq_vav <= 1'b1;
                     sq_wdata   <= mem_wdata;
                     sq_be      <= mem_be;
                  end
                  mem_xc     <= 2'd1;
                  // M7b store-snoop: TAG-CHECKED. Invalidate only if the store's
                  // tag matches the cached line's tag (the store actually writes
                  // that code line) — a store that merely index-aliases a cached
                  // line no longer evicts it. SMC-correct (a real code-modifying
                  // store has a matching tag) and still backed by the M5c
                  // squash-and-refetch. mem_iaddr is byte-aligned: index [12:4],
                  // tag [31:13] (= the install's if_req_dw[28:IC_IDXW+1]).
                  // M12: compare here, clear next cycle (snoop_q, below the
                  // case). No install can land in the gap: this issue needs
                  // !if_xc and the store then holds the port (mem_port_op
                  // blocks fill issue), and the store's data is not in memory
                  // before mem_xc==1 completes anyway.
                  snoop_q     <= mem_port_wr &&
                                 ic_tagff[mem_iaddr[IC_IDXW+3:4]] == mem_iaddr[31:IC_IDXW+4];
                  snoop_idx_q <= mem_iaddr[IC_IDXW+3:4];
                  // 1c: may this store post next cycle? (no code copy of its
                  // dword anywhere on the fetch side; MMIO never posts)
                  // (the I-cache holds a copy only if the line is VALID: the
                  // snoop clears ic_v* but not the tag shadow, so a tag-only
                  // compare kept refusing every later store to a once-code
                  // line — on the board, lines left by earlier programs /
                  // the boot ROM: stores stopped posting (EN_ST1: squashed))
                  st_post_ok  <= EN_POST_ST && mem_port_wr && mem_iaddr[31:28] != 4'hF &&
                                 !(ic_tagff[mem_iaddr[IC_IDXW+3:4]] == mem_iaddr[31:IC_IDXW+4] &&
                                   (ic_vhi[mem_iaddr[IC_IDXW+3:4]] || ic_vlo[mem_iaddr[IC_IDXW+3:4]])) &&
                                 !(ifb_val[0] && ifb_base  == smc_dw) &&
                                 !(ifb_val[1] && r_base_p1 == smc_dw) &&
                                 !((nlb_vhi || nlb_vlo) && nlb_line == smc_dw[28:1]) &&
                                 !((nlb1_vhi || nlb1_vlo) && nlb1_line == smc_dw[28:1]) &&
                                 !(id_valid && (id_pc[31:3] == smc_dw || id_last_dw == smc_dw)) &&
                                 !(ex_valid && (ex_pc[31:3] == smc_dw || ex_last_dw == smc_dw));
                  // Watchpoints: compare this access at issue; the halt is
                  // taken at the next dispatch boundary (the instruction
                  // completes; at most the two younger ones in EX/ID too).
                  if (DEBUG_EN) begin
                     for (int w = 0; w < 2; w++)
                        if (dbg_i.wp_en[w] && mem_iaddr[31:3] == dbg_i.wp_dw[w] &&
                            (mem_be & dbg_i.wp_be[w]) != 8'h00 &&
                            ((dbg_i.wp_rd[w] && mem_port_rd) || (dbg_i.wp_wr[w] && mem_port_wr))) begin
                           dbg_watch_req  <= 1'b1;
                           dbg_watch_addr <= mem_iaddr;
                        end
                  end
                  // M9: issue-time NLB snoop + in-flight prefetch abort. Closes
                  // the 1-cycle race where a prefetch captures a BRAM read that
                  // predates this store's ic_v* clear (the mem_done_now
                  // store-match below is the belt-and-braces line-granular kill).
                  if (mem_port_wr && (nlb_vhi || nlb_vlo)
                      && nlb_line == mem_eaddr[31:4]) begin
                     nlb_vhi <= 1'b0;  nlb_vlo <= 1'b0;
                  end
                  if (mem_port_wr && (nlb1_vhi || nlb1_vlo)
                      && nlb1_line == mem_eaddr[31:4]) begin
                     nlb1_vhi <= 1'b0;  nlb1_vlo <= 1'b0;
                  end
                  if (mem_port_wr && pf_look && pf_line == mem_eaddr[31:4])
                     pf_look <= 1'b0;
                  // EN_ST1: the store leaves MEM now, already posted
                  // (EN_SQ: or queued behind the posted store)
                  if (st_spec_now || sq_enq_now) begin
                     mem_xc     <= 2'd0;
                     // (a queued store is not on the port: st_post stays the
                     // posted store's — unless it chained on as that drained)
                     if (st_spec_now || (w_mrdy && mem_iaddr != m_addr)) st_post <= 1'b1;
                     st_spec    <= 1'b1;
                     st_spec_nx <= mem_pc_next;
                  end
               end
               2'd1: if (w_mrdy) begin
                  if (mem_need2) begin         // cross-line MEMGET32 / LEAVERET: 2nd read
                     mem_dw0   <= m_rdata;
                     // (stage 3: the 2nd dword's PA — next to the 1st, or
                     // the start of the next page's frame)
                     m_addr    <= mem_xpg ? {mem_pa2hi, 16'h0000} : {mem_iaddr[31:3], 3'b000} + 32'd8;
                     rdy_armed <= 1'b0;        // new request (DV held, addr changed)
                     mem_xc    <= 2'd2;        // settle cycle (f_memget32 shape)
                  end else begin
                     m_read_DV <= 1'b0;  m_write_DV <= 1'b0;
                     mem_xc    <= 2'd0;
                  end
                  st_post_ok <= 1'b0;
               end else if (st_post_ok) begin
                  // 1c: post — the store retires from MEM now (mem_done_now);
                  // its DV stays up and st_post owns the port until ready.
                  st_post    <= 1'b1;
                  st_post_ok <= 1'b0;
                  mem_xc     <= 2'd0;
               end else if (mem_fastld) begin
                  // 1a: the read is on the port; the load moves on to WB,
                  // which collects the response (ld_out owns the port)
                  ld_out <= 1'b1;
                  mem_xc <= 2'd0;
               end
               2'd2: mem_xc <= 2'd3;
               default: if (w_mrdy) begin     // 2'd3
                  m_read_DV <= 1'b0;
                  mem_xc    <= 2'd0;
               end
            endcase
         end

         // ---------------- debugger memory access (debug-halted only) ----------
         // The parked core's port is idle; one access at a time through the
         // same port (D-cache coherent, MMIO decoded by the SoC as usual).
         if (DEBUG_EN && dbg_halted) begin
            if (!dbg_mx) begin
               if (dbg_i.mem_req && !if_xc && mem_xc == 2'd0 && !st_post && !sq_v && !ld_out) begin
                  m_addr     <= dbg_i.mem_addr;
                  m_wdata    <= dbg_i.mem_wdata;
                  m_be       <= dbg_i.mem_be;
                  m_read_DV  <= !dbg_i.mem_we;
                  m_write_DV <= dbg_i.mem_we;
                  m_vav      <= 1'b0;
                  rdy_armed  <= !m_ready;
                  dbg_mx     <= 1'b1;
               end
            end else if (w_mrdy) begin
               m_read_DV     <= 1'b0;  m_write_DV <= 1'b0;
               dbg_mem_rdata <= m_rdata;
               dbg_mem_done  <= 1'b1;
               dbg_mx        <= 1'b0;
            end
         end

         // ------------- IF fill engine: I-cache lookup -> hit | miss-fill -------
         // Three phases: (idle) on a fetch-window miss, start an I-cache LOOKUP
         // (no port); on the lookup result, either serve the IFB from the cache
         // (hit, 1 cyc, no port — the win) or issue the m_* miss-fill; on fill
         // completion, capture into the IFB AND install the line. The IFB capture
         // is byte-identical between the hit and miss paths (ifr_* mirrors m_*).
         if (if_xc) begin
            if (EN_EXC && if_fake) begin
               // MMU stage 1 fake fill (see the miss issue): zeros, no install
               if (EN_TLB && if_drop) begin
                  // (a TLB / ASID / TE change discarded the window meanwhile)
               end else if (if_rebase) begin
                  ev_fill   = 1'b1;
                  ifb_base  <= if_req_dw;
                  ifb_dw[0] <= 64'b0;  ifb_val[0] <= 1'b1;
                  ifb_val[1] <= 1'b0;
                  ifb_ft[0] <= if_fflt;  ifb_fm[0] <= if_fmiss;
               end else begin
                  ifb_dw[1] <= 64'b0;  ifb_val[1] <= 1'b1;
                  ifb_ft[1] <= if_fflt;  ifb_fm[1] <= if_fmiss;
               end
               if_xc   <= 1'b0;
               if_fake <= 1'b0;
               if_drop <= 1'b0;
            end else if (w_mrdy) begin
               if_drop <= 1'b0;
               if (EN_TLB && if_drop) begin
                  // (window discarded meanwhile: install only, physical tags)
               end else if (if_rebase) begin
                  ev_fill   = 1'b1;
                  ifb_base  <= if_req_dw;
                  ifb_dw[0] <= m_rdata;       ifb_val[0] <= 1'b1;
                  ifb_dw[1] <= m_rdata_next;  ifb_val[1] <= m_next_valid;
                  ifb_ft    <= 2'b00;  ifb_fm <= 2'b00;
               end else begin
                  ifb_dw[1] <= m_rdata;       ifb_val[1] <= 1'b1;
                  ifb_ft[1] <= 1'b0;   ifb_fm[1] <= 1'b0;
               end
               m_read_DV <= 1'b0;
               if_xc     <= 1'b0;
               // --- install the fetched dword(s): if_req_dw[0]=addr3 (0->hi dword,
               //     + lo when m_next_valid; 1->lo dword). Per-dword valid tolerates
               //     the partial (odd-address) fill.
               begin : ic_install
                  // Deterministic install (no old-state read → robust to a
                  // redirect during the fill). An even fetch returns BOTH dwords
                  // (m_next_valid always set for addr3=0) → full-line install; an
                  // odd fetch returns only the low dword → write it, retag, clear
                  // hi. (An odd fill only occurs on an empty/other-tag line — a
                  // cached line's odd dword would have hit — so no cached data is
                  // lost.)  {m_rdata, m_rdata_next} = {addr3=0 dword, addr3=1 dword}.
                  logic [IC_IDXW-1:0] wi;
                  logic [18:0]        wt;
                  wi = if_req_dw[IC_IDXW:1];
                  wt = {if_tghi, if_req_dw[12:IC_IDXW+1]};   // (stage 3: PA tag)
                  ev_icinst = 1'b1;  ev_icinst_idx = wi;   // 2a: BTIC entries of this line die
                  ic_tag[wi]   <= wt;
                  ic_tagff[wi] <= wt;   // M7b snoop shadow
                  if (if_req_dw[0] == 1'b0) begin        // even fill: full line
                     ic_data[wi] <= {m_rdata, m_rdata_next};
                     ic_vhi[wi]  <= 1'b1;
                     ic_vlo[wi]  <= m_next_valid;
                  end else begin                         // odd fill: low dword only
                     ic_data[wi][63:0] <= m_rdata;
                     ic_vhi[wi]  <= 1'b0;
                     ic_vlo[wi]  <= 1'b1;
                  end
               end
            end
         end else if (if_look) begin
            if (ic_rd_current) begin           // registered read now valid for miss_dw
               if (ic_hit) begin               // serve from the I-cache (no port)
                  // Use CURRENT in0/miss_dw (not latched): a branch redirect can
                  // change miss_dw mid-lookup, and ic_rd_current ties ic_rd_data to
                  // the current request — so base and data stay self-consistent
                  // (a stale mix would corrupt the window; the m_* path is immune
                  // because it captures old base + old data together).
                  if (!in0) begin
                     ifb_base  <= miss_dw;  ev_rebase = 1'b1;
                     ifb_dw[0] <= ifr_rdata;       ifb_val[0] <= 1'b1;
                     ifb_dw[1] <= ifr_rdata_next;  ifb_val[1] <= ifr_next_valid;
                     ifb_ft    <= 2'b00;  ifb_fm <= 2'b00;
                  end else begin
                     ifb_dw[1] <= ifr_rdata;       ifb_val[1] <= 1'b1;
                     ifb_ft[1] <= 1'b0;   ifb_fm[1] <= 1'b0;
                  end
                  if_look <= 1'b0;
               end else if (EN_TLB && te && !lk_cov) begin
                  // MMU stage 3: the micro-ITLB does not cover this page (in
                  // this fetch mode): refill it, then look again
                  if (!it_ref && !tlb_psel) begin
                     it_ref <= 1'b1;  it_vpn <= miss_dw[28:13];  it_asid <= tlb_asid;
                  end
               end else if (!mem_port_op && mem_xc == 2'd0 && !st_post && !sq_v && !ld_out && !irq_active && !w_early_ld
                            && !wb_hmiss && !(DEBUG_EN && (dbg_active || dbg_halted))) begin
                  // !irq_active: a lookup that was live when the IRQ was taken must
                  // NOT issue its miss-fill during IRQ entry — the sequencer's frame
                  // push only checks !if_xc, so a same-cycle (or mid-push) fill issue
                  // would clobber m_addr/m_read_DV on top of the in-flight write and
                  // the frame lands on the fetch address (= code corruption at the
                  // return pc). Pre-M7a the single fill-issue point checked
                  // !irq_active; the M7a lookup/issue split lost it on this path.
                  // Hit-serves stay allowed during entry (no port; warms the window
                  // the IRET returns to). irq_xc ⊆ irq_active, so this covers the
                  // whole entry window; if_look simply holds and re-evaluates.
                  // (stage 3, TE = 1: the PA from the micro-ITLB; a refused
                  // page — segment, TLB miss, no X — is a fake fill with a
                  // fault tag, nothing on the port)
                  m_addr    <= (EN_TLB && te) ? {r_look_pa, miss_dw[12:0], 3'b000}
                                              : {miss_dw, 3'b000};   // miss: issue the shared-port fill
                  if_tghi   <= (EN_TLB && te) ? r_look_pa : miss_dw[28:13];
                  if_fflt   <= EN_TLB && te && !r_look_ok;
                  if_fmiss  <= EN_TLB && te && r_look_miss;
                  // MMU stage 1: in user mode a 0xF... dword is a FAKE fill —
                  // no DV on the port; it completes next cycle with zeros and
                  // no I-cache install, and the ID fetch check makes the op
                  // ACCESS_FETCH.  (Steers only these D inputs: off the
                  // ic_hit serve chain.)
                  m_read_DV <= !(w_ufz || (EN_TLB && te && !r_look_ok));
                  if_fake   <= w_ufz || (EN_TLB && te && !r_look_ok);
                  rdy_armed <= !m_ready;
                  if_req_dw <= miss_dw;             // old base+data captured together
                  r_ifq_p1  <= miss_dw + 29'd1;     // M12: for the fill pos update
                  if_rebase <= !in0;
                  if_xc     <= 1'b1;
                  if_look   <= 1'b0;
               end
               // else: port busy — hold in if_look (re-evaluates in0/miss_dw each cycle)
            end
         end else if (running && !fetch_halt && !parked && !irq_active) begin
            if (!fetch_ok) begin
               // M9: serve a sequential line cross from the NLB in ONE cycle (no
               // BRAM, no port). Same current-in0/miss_dw discipline as the
               // I-cache hit serve — base and data are registered together.
               if (nlb_dw_ok) begin
                  if (!in0) begin
                     ifb_base  <= miss_dw;  ev_rebase = 1'b1;
                     ifb_dw[0] <= nlb_rdata;       ifb_val[0] <= 1'b1;
                     ifb_dw[1] <= nlb_rdata_next;  ifb_val[1] <= nlb_next_valid;
                     ifb_ft    <= 2'b00;  ifb_fm <= 2'b00;
                  end else begin
                     ifb_dw[1] <= nlb_rdata;       ifb_val[1] <= 1'b1;
                     ifb_ft[1] <= 1'b0;   ifb_fm[1] <= 1'b0;
                  end
               end else if (w_slide) begin
                  // M9+: slot-1 straddle (in1 && w_span at an even base): the op
                  // needs ifb_base+2, which nlb_dw_ok can't see (miss_dw is still
                  // in the window's own line). Slide instead of if_look: 1 cycle,
                  // no BRAM, no port.
                  ifb_base  <= ifb_base + 29'd1;  ev_slide = 1'b1;
                  ifb_dw[0] <= ifb_dw[1];       ifb_val[0] <= 1'b1;
                  ifb_dw[1] <= slide_data;      ifb_val[1] <= 1'b1;
                  ifb_ft    <= {1'b0, ifb_ft[1]};  ifb_fm <= {1'b0, ifb_fm[1]};
               end else begin
                  if_look   <= 1'b1;              // begin lookup (no port access)
               end
            end else if (w_slide) begin
               // M9+ 0-cycle serve: pc is dispatching out of slot 1 — slide the
               // window forward NOW, so the coming sequential line cross finds
               // its dword already in the window and fetch never stalls.
               ifb_base  <= ifb_base + 29'd1;  ev_slide = 1'b1;
               ifb_dw[0] <= ifb_dw[1];       ifb_val[0] <= 1'b1;
               ifb_dw[1] <= slide_data;      ifb_val[1] <= 1'b1;
               ifb_ft    <= {1'b0, ifb_ft[1]};  ifb_fm <= {1'b0, ifb_fm[1]};
            end
         end

         // ---------------- M9: NLB prefetch engine (background) ----------------
         // Runs the normal 1-cycle BRAM lookup at want_line through the shared
         // read port whenever the demand side is idle; captures on hit, gives
         // up silently on miss (demand path handles cold lines). A demand
         // lookup starting mid-prefetch simply steals the mux (pf_sel drops);
         // the prefetch retries when the port frees up.
         if (pf_look) begin
            if (if_look) begin
               pf_look <= 1'b0;                // demand owns the port — retry later
            end else if (pf_rd_current) begin
               if (pf_tag_match && (ic_rd_vhi || ic_rd_vlo)) begin
                  // EN_NLB2: into the entry that does not hold a line still
                  // ahead of the window (want / want+1)
                  if (EN_NLB2 && (nlb0_has_want || nlb0_has_wp1)) begin
                     nlb1_data <= ic_rd_data;
                     nlb1_line <= pf_line;
                     nlb1_vhi  <= ic_rd_vhi;
                     nlb1_vlo  <= ic_rd_vlo;
                  end else begin
                     nlb_data <= ic_rd_data;
                     nlb_line <= pf_line;
                     nlb_vhi  <= ic_rd_vhi;
                     nlb_vlo  <= ic_rd_vlo;
                  end
               end
               // EN_NLB2: pipelined — having captured the window's next line,
               // go straight on to the one after it (no idle cycle between
               // lookups: a line per 2 cycles, the rate of 2-word code)
               if (EN_NLB2 && pf_tag_match && pf_line == want_line && !nlb_has_wp1)
                  pf_line <= want_p1;
               else
                  pf_look <= 1'b0;             // hit captured / miss abandoned
            end
         end else if (EN_PF_EARLY && running && !fetch_halt && !parked && !irq_active
                      && w_slide && ifb_base[0] && !if_look && !if_xc &&
                      !(EN_NLB2 && nlb_has_wp1)) begin
            // HW proposal 3 (§2.6): a slide from an ODD base consumes the
            // NLB's last dword, and want_line moves to want_line+1 on this
            // edge.  Start that line's lookup NOW instead of next cycle: the
            // 3-cycle BRAM prefetch then lands as the window's last word
            // dispatches, closing the one-bubble-per-8-words gap of
            // straight-line 1-word code.  The NLB's old line is dead after
            // this edge, so the earlier capture cannot clobber a live line.
            // (Paired with the redirect abort in the pos_track composer: a
            // prefetch must never hold the BRAM port into a redirect.)
            pf_look <= 1'b1;
            pf_line <= r_want_line + 28'd1;
         end else if (running && !fetch_halt && !parked && !irq_active
                      && fetch_ok && ifb_val[0] && (!nlb_has_want || (EN_NLB2 && !nlb_has_wp1))
                      && !if_look && !if_xc) begin
            // the window's next line first; then (EN_NLB2) the one after it
            pf_look <= 1'b1;
            pf_line <= nlb_has_want ? want_p1 : want_line;
         end

         // ---------------- 2a: predicted-taken redirect at ID entry ----------
         // On a predicted-taken branch's FIRST cycle in ID, fetch moves to
         // its target (and loads the BTIC window) whether or not the branch
         // can dispatch this cycle — so these enables hang off registers, not
         // off the dispatch / stall decision, and a branch waiting in ID for
         // its operands finds its target already fetched.  Anything older
         // that moves pc this edge (EX redirect, squashes) comes later in
         // this block and wins (ev_ovr drops the BTIC position claim); it
         // sits after the IF engine so its window load wins over a same-edge
         // slide / serve.
         if (id_early_rd && !start) begin
            ev_pcz = 1'b1;
            pc     <= id_bp_tgt;
            id_rpend <= 1'b0;
            if (bt_hit) begin
               ev_btic    = 1'b1;
               ifb_base   <= id_bt_base;
               ifb_dw[0]  <= id_bt_d0;
               ifb_dw[1]  <= id_bt_d1;
               ifb_val    <= {id_bt_v1, 1'b1};
               ifb_ft     <= 2'b00;  ifb_fm <= 2'b00;
               if_look    <= 1'b0;
            end else if (EN_BTIC) begin
               r_bt_pend   <= 1'b1;
               r_bt_widx   <= id_pc[7:2] ^ id_pc[13:8];
               r_bt_wtag   <= id_pc[31:2];
               r_bt_wbase  <= id_bp_tgt[31:3];
               r_bt_wblidx <= id_pc[IC_IDXW+3:4];
            end
         end

         // 1a stage 3: the hit pipe answers in the load's first MEM cycle;
         // keep the answer for a load that stays in MEM (EX -> MEM below
         // re-arms mem_h1 for the next load: last write wins)
         if (mem_mul) begin
            if (mem_mage != 2'd3) mem_mage <= mem_mage + 2'd1;
            if (mem_mage == 2'd1) begin
               mem_mres <= mul_m[63:0];  mem_msgn <= mul_m[64];  mem_movf <= mul_m[65];
            end
         end
         if (mem_h1) begin
            mem_hok <= h_ok;
            mem_hdw_r <= h_dword;
         end
         mem_h1 <= 1'b0;

         // ---------------- MEM -> WB ----------------
         if (mem_busy) begin
            if (!wb_wait) wb_valid <= 1'b0;   // (1a: a waiting fast load stays in WB)
         end else begin
            wb_valid   <= mem_valid && !sq_spec;
            wb_ld      <= mem_fastld;
            wb_ld_full <= (mem_d.uop != U_LOAD);   // POP / LEAVE: the whole dword
            wb_ld_msz  <= mem_d.msz;
            wb_ld_sgn  <= mem_d.sgn;
            wb_ld_ea   <= mem_eaddr[2:0];
            wb_h       <= mem_valid && mem_h && mem_d.uop != U_RET && !mem_xsq;   // (a RET resolves in MEM)
            wb_hok     <= mem_h1 ? h_ok : mem_hok;
            wb_hdw     <= mem_h1 ? h_dword : mem_hdw;
            wb_iaddr   <= mem_iaddr;  wb_be <= mem_be;
            wb_pc      <= mem_pc;    wb_op <= mem_op;   wb_rd <= mem_d.rd;
            wb_wreg    <= mem_d.wreg && !mem_xsq;
            wb_value   <= mem_is_read ? mem_ldval : mem_result_eff;
            // Z producers are never port reads (IRET's flags come from
            // iret_fval with wb_zval 0), so the reduce sees mem_result_eff
            // only and stays off the m_rdata path.
            wb_zero    <= (mem_result_eff == 64'b0);
            wb_fval    <= (mem_valid && mem_d.uop == U_IRET) ? iret_fval : mem_fval;
            wb_zval    <= mem_zval && !(mem_valid && mem_d.uop == U_IRET);
            // EN_MUL2: the MUL's product: from WB itself (age 0), the DSP now
            // (age 1) or the capture (age 2+)
            wb_mul     <= mem_mul;
            // EN_LDBYP: the hit-pipe / forwarded answer, extracted (a missed
            // load's value comes from the port; its consumer captures that)
            wb_hext    <= f_ldext(mem_h1 ? h_dword : mem_hdw, mem_eaddr[2:0], mem_d.msz, mem_d.sgn,
                                  mem_d.uop != U_LOAD);
            wb_mulf    <= mem_mul && mem_mulf;
            wb_mdir    <= mem_mul && mem_mage == 2'd0;
            wb_mhi     <= mem_d.mdhigh;  wb_muns <= mem_d.mduns;  wb_mw32 <= mem_d.w32;
            if (mem_mul && mem_mage != 2'd0) begin
               wb_value         <= (mem_mage == 2'd1) ? mul_m[63:0] : mem_mres;
               wb_zero          <= ((mem_mage == 2'd1) ? mul_m[63:0] : mem_mres) == 64'b0;
               wb_fval.sign     <= (mem_mage == 2'd1) ? mul_m[64] : mem_msgn;
               wb_fval.overflow <= (mem_mage == 2'd1) ? mul_m[65] : mem_movf;
            end
            wb_fmask   <= mem_xsq ? '0 : mem_fmask;
            wb_sp_new  <= mem_sp_new;  wb_sp_we <= mem_valid && mem_sp_we && !mem_xsq;
            // MMU stage 1: IRET restores UM from frame bit 44; returning to
            // user mode it swaps the stacks (sp = the user SP, sp_alt = the
            // kernel SP above the popped frame).  Computed here, so SP
            // forwarding to a younger op carries the right value.
            wb_um_we   <= EN_EXC && mem_valid && mem_d.uop == U_IRET && !mem_xsq;
            wb_um_new  <= m_rdata[44];
            wb_spalt_new <= mem_sp_new;
            if (EN_EXC && mem_valid && mem_d.uop == U_IRET && m_rdata[44])
               wb_sp_new <= sp_alt;
            wb_mask_we <= mem_valid && (mem_d.uop == U_IRET) && !mem_xsq;
            wb_mask_new<= m_rdata[42:39];
            wb_dis_we  <= mem_valid && (mem_dis_we || mem_d.uop == U_IRET) && !mem_xsq;
            wb_dis_new <= (mem_d.uop == U_IRET) ? m_rdata[43] : mem_dis_new;
            wb_park    <= mem_valid && (mem_d.uop == U_HALT || mem_d.uop == U_TRAP ||
                                        mem_d.uop == U_WAIT || mem_d.uop == U_ILL ||
                                        (EN_EXC && (mem_d.uop == U_EXC || mem_xsq)));
            // (MMU stage 1: an exception op parks as ILLEGAL when EVEC = 0)
            wb_park_kind <= mem_misal ? 3'd5 : (EN_EXC && mem_xsq) ? 3'd2
                          : (mem_d.uop == U_HALT) ? 3'd0 : (mem_d.uop == U_TRAP) ? 3'd1
                          : (mem_d.uop == U_ILL || mem_d.uop == U_EXC) ? 3'd2 : 3'd3;
            // MMU stage 1: the exception record (cause, EADDR, next PC)
            wb_xc      <= EN_EXC && mem_valid &&
                          (mem_d.uop == U_TRAP || mem_d.uop == U_ILL || mem_d.uop == U_EXC || mem_xsq);
            wb_xcause  <= mem_misal ? (mem_misal_st ? X_MISAL_ST : X_MISAL_LD)
                        : mem_xsq ? (mem_xs_tm ? (mem_xs_st ? X_TLB_ST : X_TLB_LD)
                                             : (mem_xs_st ? X_ACC_ST : X_ACC_LD))
                        : (mem_d.uop == U_TRAP) ? X_TRAP
                        : (mem_d.uop == U_ILL)  ? X_ILL : mem_d.sub;
            wb_xaddr   <= mem_misal ? mem_ea_raw
                        // (an access fault on the 2nd dword only: its start)
                        : mem_xsq ? (mem_xs2 ? {mem_ea_raw[31:3] + 29'd1, 3'b000} : mem_ea_raw)
                        : (mem_d.uop == U_EXC && mem_d.sub == X_SYSCALL) ? 32'b0
                        : (mem_d.uop == U_EXC && mem_d.sub == X_MISAL_FE) ? mem_pc
                        : (mem_d.uop == U_EXC && (mem_d.sub == X_ACC_FE || (EN_TLB && mem_d.sub == X_TLB_FE)))
                          ? (mem_d.fx2 ? {mem_pc[31:5] + 27'd1, 5'b0} : mem_pc)
                        : mem_op;
            wb_pc_next <= mem_pc_next;
            wb_mtsr    <= EN_EXC && mem_valid && mem_d.mtsr;
            wb_tlbi    <= EN_TLB && mem_valid && mem_d.tlbi;
            wb_va_dw   <= mem_eaddr[31:3];        // (the reservation holds a VA)
            wb_xtlb    <= EN_TLB && (mem_xsq ? mem_xs_tlb : (mem_valid && mem_d.uop == U_EXC && mem_d.fxl));
            wb_xguard  <= EN_MPU && (mem_xsq ? mem_xs_g
                                     : (mem_d.uop == U_EXC && mem_d.sub == X_ACC_FE && mem_d.fxg));
            wb_lr      <= EN_EXC && mem_valid && mem_d.uop == U_LR && !mem_xsq;
            wb_sc      <= EN_EXC && mem_valid && mem_d.uop == U_SC && !mem_xsq;
            wb_wr      <= mem_is_write;
            wb_wr_addr <= mem_iaddr;  wb_wr_be <= mem_be;  wb_wr_raw <= (mem_d.uop == U_ENTER || mem_d.uop == U_SC) ? mem_wdata : mem_result; // ENTER: stored R15; SC: its data
            // RET / IRET redirect + fetch resume at MEM completion
            // (2a: a PREDICTED return does nothing here — fetch already
            // runs at its target, and a mismatch is the ras_miss squash.)
            if (mem_valid && (mem_d.uop == U_RET || mem_d.uop == U_IRET ||
                              mem_d.uop == U_LEAVERET) && !(EN_RAS && mem_pred) && !mem_xsq) begin
               ev_pcz = 1'b1;
               // LEAVERET's return address is the SECOND dword: the line
               // lookahead, or the 2nd read (mem_xc 3) with dw0 stashed.
               pc <= ret_actual;
               fetch_halt <= 1'b0;
            end

            // ---------------- EX -> MEM ----------------
            if (ex_busy) begin
               mem_valid <= 1'b0;
               if (ex_is_mul) mul_cnt <= mul_cnt + 2'd1;
               if (ex_mpu_hold) begin
                  if (!ex_mph) ex_mph <= 1'b1;
                  else         ex_mph2 <= 1'b1;
               end
               // (stage 3) the 1st page's translation, kept for the 2nd-page cycle
               if (ex_mpu_hold && ex_mph) begin
                  t1_dn <= t_dn;  t1_seg <= t_seg;  t1_miss <= t_miss;  t1_tlb <= t_miss || t_perm;
                  t1_pahi <= t_pahi;
               end
            end else begin
               ex_mph     <= 1'b0;          // the op leaves EX (or EX was empty)
               ex_mph2    <= 1'b0;
               mem_valid  <= ex_valid;   // not-taken CALL still retires (trace line)
               // early load issue (see EN_EARLY_LD): the MEM engine then
               // starts in its wait-ready state
               if (w_early_ld && !ex_misal && !ex_uf_early) begin
                  m_addr     <= exo_iaddr;
                  m_be       <= exo_be;
                  m_read_DV  <= 1'b1;
                  rdy_armed  <= !m_ready;
                  mem_xc     <= 2'd1;
                  if (DEBUG_EN && ex_wp_ok) begin
                     for (int w = 0; w < 2; w++)
                        if (dbg_i.wp_en[w] && exo_iaddr[31:3] == dbg_i.wp_dw[w] &&
                            (exo_be & dbg_i.wp_be[w]) != 8'h00 && dbg_i.wp_rd[w]) begin
                           dbg_watch_req  <= 1'b1;
                           dbg_watch_addr <= exo_iaddr;
                        end
                  end
               end
               mem_mage   <= 2'd0;
               mem_lkq    <= (ex_d.uop == U_LOAD || ex_d.uop == U_LOAD32 || ex_d.uop == U_POP ||
                              ex_d.uop == U_LEAVE || ex_d.uop == U_LEAVERET || ex_d.uop == U_LR ||
                              (EN_MUL2 && ex_d.uop == U_MUL));
               mem_mulq   <= EN_MUL2 && ex_d.uop == U_MUL;
               mem_fastq  <= (ex_d.uop == U_LOAD || ex_d.uop == U_POP || ex_d.uop == U_LEAVE) &&
                             !ex_misal && exo_iaddr[31:28] != 4'hF;
               mem_mulf   <= EN_MUL2 && ex_d.uop == U_MUL && exo_fmask != '0;
               mem_h      <= h_elig;
               mem_h1     <= h_elig && !h_fwd;     // (a forwarded load needs no cache answer)
               mem_fq   <= h_fwd;
               mem_fsel <= sf_a;
               mem_fselc <= sf_c;
               mem_fwa  <= mem_wdata;
               mem_fwb  <= m_wdata;
               mem_fwc  <= sq_wdata;
               if (h_fwd) mem_hok <= 1'b1;
               // (watchpoints compare physical addresses, as the MEM-issue
               // compare on mem_iaddr: h_addr is the PA with TE = 1)
               if (DEBUG_EN && h_elig && ex_wp_ok) begin
                  for (int w = 0; w < 2; w++)
                     if (dbg_i.wp_en[w] && h_addr[31:3] == dbg_i.wp_dw[w] &&
                         (exo_be & dbg_i.wp_be[w]) != 8'h00 && dbg_i.wp_rd[w]) begin
                        dbg_watch_req  <= 1'b1;
                        dbg_watch_addr <= h_addr;
                     end
               end
               mem_pc     <= ex_pc;    mem_op <= ex_op;   mem_d <= ex_d;
               mem_pred   <= ex_valid && ex_pred;  mem_pred_tgt <= ex_pred_tgt;
               mem_ptk    <= ex_ptk;  mem_pc_next <= ex_pc_next;
               mem_bp_idx <= ex_bp_idx;  mem_bp_ctr <= ex_bp_ctr;
               mem_bp_fbr <= ex_valid && ex_d.uop == U_FBR;
               // 2a: a taken CALL pushes its return address as it leaves EX
               // (not when an older squash kills it this edge)
               // (not gated by ras_miss: a wrong-path CALL pushed on the edge a
               // mispredicted RET redirects costs only a later RAS miss, and the
               // m_rdata compare would sit on the RAS write enable)
               if (EN_RAS && ex_valid && ex_d.uop == U_CALL && exo_taken &&
                   !fbr_taken && !sq_ex && !sq_spec && !mem_xsq_go) begin
                  ras[ras_ptr] <= ex_pc_next;
                  ras_ptr      <= ras_ptr + 1'b1;
                  if (ras_cnt != (RAS_PW+1)'(RAS_DEPTH)) ras_cnt <= ras_cnt + 1'b1;
               end
               mem_result <= (ex_d.uop == U_STORE || ex_d.uop == U_PUSH ||
                              ex_d.uop == U_CALL) ? exo_raw : exo_result;
               mem_fval   <= exo_fval; mem_fmask <= ex_valid ? exo_fmask : '0;
               mem_zval   <= (ex_d.uop != U_BTSTN);   // Z from (value==0) at WB
               mem_eaddr  <= exo_eaddr;
               // (stage 3: with TE = 1 the port address is the PA; the page
               // offset is the VA's)
               mem_iaddr  <= (EN_TLB && te) ? {d1_pahi, exo_iaddr[15:0]} : exo_iaddr;
               mem_pa2hi  <= t_pahi;                // a page-crossing two-access op's 2nd page
               mem_xpg    <= ex_x3;
               mem_wdata  <= exo_wdata;  mem_be <= exo_be;
               mem_mmq    <= ex_is_mm;   mem_mmpick <= ex_mmpick;
               mem_sp_new <= exo_sp_new; mem_sp_we <= ex_valid && exo_sp_we;
               mem_misal  <= ex_misal;
               mem_misal_st <= (ex_d.uop == U_STORE || ex_d.uop == U_SC);
               mem_ea_raw <= ex_ea_raw;
               // MMU: an access fault is ONE registered bit (timing: nothing
               // else at EX -> MEM depends on it).  The op enters MEM as it
               // is; MEM gates its port access, WB its commit, and in its
               // first MEM cycle it squashes EX / ID (mem_xsq_go).
               mem_xsq    <= ex_ufault;
               mem_xs_st  <= ex_mst;               // direction (cause)
               mem_xs2    <= !ex_dn1;              // only the 2nd dword faulted
               mem_xs_g   <= ex_xg;                // GUARD denial (stage 2, ECAUSE[8])
               mem_xs_tm  <= ex_xtm;               // TLB miss (stage 3)
               mem_xs_tlb <= ex_xtlb;              // TLB-related (TLB_HI auto-load)
               mem_dis_we <= ex_valid && (ex_d.uop == U_DI || ex_d.uop == U_EI);
               mem_dis_new<= (ex_d.uop == U_DI) ? 1'b1 : !ex_a[0];

               // Operand registers load on every non-busy cycle, also under an
               // EX redirect or a RAW bubble (ex_valid <= 0 then, so the values
               // are dead): their enable is !ex_busy alone, not the EX
               // flags -> taken cone.
               // M6 register-sourced forwarding at the operand-register
               // inputs: youngest writer wins (MEM over WB over rf).
               ex_a       <= f_wsh(id_w,
                             (mem_fwd_ok && mem_d.rd == id_rs1) ? mem_result_eff
                           : (b_wb       && wb_rd    == id_rs1) ? wb_fwd_v
                           : rf[id_rs1]);
               // EN_SPFWD: the youngest older SP writer's value (one in EX
               // holds this op back via sp_busy)
               ex_lb_a    <= id_use_rs1 && lb1;
               ex_lb_b    <= id_use_rs2 && lb2;
               ex_w       <= id_w;
               ex_sp      <= (mem_valid && mem_sp_we) ? mem_sp_new
                           : (wb_valid && wb_sp_we)   ? wb_sp_new : sp;
               ex_c       <= (mem_fwd_ok && mem_d.rd == id_rd)  ? mem_result_eff
                           : (b_wb       && wb_rd    == id_rd)  ? wb_fwd_v
                           : rf[id_rd];
               // The b-operand fully resolves HERE (reg / extended imm32 /
               // constant 1 for INC-DEC / forwarded result) so EX's adder
               // and comparators read pure registers.
               ex_b       <= f_wsh(id_w,
                             id_b_one ? 64'd1
                           : id_b_imm
                             ? (id_b_sext ? {{32{id_var1[31]}}, id_var1} : {32'b0, id_var1})
                           : (mem_fwd_ok && mem_d.rd == dec.rs2) ? mem_result_eff
                           : (b_wb       && wb_rd    == dec.rs2) ? wb_fwd_v
                           : rf[dec.rs2]);

               // EX redirect: taken branch/call, RESET
               if (ex_valid && ex_redir) begin
                  ev_pcz     = 1'b1;
                  pc         <= ex_redir_tgt;
                  id_valid   <= 1'b0;
                  ex_valid   <= 1'b0;
                  fetch_halt <= 1'b0;   // RESET resumes here
               end else if (raw_stall) begin
                  ex_valid <= 1'b0;     // bubble; hold ID/pc
               end else begin
                  ex_valid   <= id_valid && !fetch_halt && !(DEBUG_EN && id_bp);
                  ex_pc      <= id_pc;   ex_op <= id_op;
                  // MMU stage 1: MFSR (not STATUS) is a MOV64 of its SR value
                  ex_var1    <= (EN_EXC && id_mfsr) ? id_srv[31:0]  : id_var1;
                  ex_var2    <= (EN_EXC && id_mfsr) ? id_srv[63:32] : id_var2;
                  ex_pc_next <= id_pc_next;
                  ex_pred    <= 1'b0;
                  ex_ptk     <= EN_BP && id_bp_tk;
                  ex_bp_idx  <= id_bp_idx;  ex_bp_ctr <= id_bp_ctr;
                  ex_bp_cnd  <= id_bp_cnd;
                  ex_last_dw <= id_last_dw;
                  ex_d       <= dec;
                  ex_fwd_f   <= dec.fread && ex_f_fwdable;   // M6b flag forward
                  mul_cnt    <= 2'd0;
                  if (DEBUG_EN && id_valid && id_bp) begin
                     // breakpoint: the ID op does not execute; park at its PC
                     // once the older instructions drain
                     ev_bp       = 1'b1;
                     ev_pcz      = 1'b1;
                     pc          <= id_pc;
                     id_valid    <= 1'b0;
                     id_bp       <= 1'b0;
                     dbg_active  <= 1'b1;
                     dbg_cause_q <= DBG_CAUSE_BP;
                  end else if (EN_RAS && id_valid && !fetch_halt && id_ras_ret && !(EN_EXC && id_fx) &&
                               ras_cnt != '0) begin
                     // 2a: predicted return — fetch continues at the RAS top
                     // (no serialize); verified when the RET completes MEM
                     ev_pcz      = 1'b1;
                     pc          <= ras_top;
                     id_valid    <= 1'b0;
                     ex_pred     <= 1'b1;
                     ex_pred_tgt <= ras_top;

                     ras_ptr     <= ras_ptr - 1'b1;
                     ras_cnt     <= ras_cnt - 1'b1;
                  end else if (EN_BP && id_valid && !fetch_halt && id_bp_tk && id_rpend) begin
                     // 2b: a predicted-taken branch dispatching before (or on
                     // the edge of) its early redirect: nothing is latched
                     // behind it.  If the early redirect was held off (EX
                     // could redirect) it is done here, without the BTIC.
                     // (After a stall it dispatches normally and the target
                     // op latches behind it.)
                     id_valid    <= 1'b0;
                     if (!id_early_rd) begin
                        ev_pcz = 1'b1;
                        pc     <= id_bp_tgt;
                     end
                  end else if (id_valid && !fetch_halt && id_ser) begin
                     fetch_halt <= 1'b1;   // stop fetching behind a serializer
                     id_valid   <= 1'b0;
                  end else if (DEBUG_EN && running && !fetch_halt && !irq_active
                               && !dbg_active && dbg_stop) begin
                     // debug halt at this boundary (like an IRQ, but parks)
                     dbg_active  <= 1'b1;
                     dbg_cause_q <= dbg_watch_req ? DBG_CAUSE_WATCH
                                  : dbg_step_done ? DBG_CAUSE_STEP : DBG_CAUSE_REQ;
                     id_valid    <= 1'b0;
                  end else if (running && !fetch_halt && !irq_active
                               && !(DEBUG_EN && dbg_active) && irq_take) begin
                     // take the IRQ at this boundary instead of fetching; the
                     // instruction in ID (older) dispatched above and drains.
                     // (irq_sel_q / irq_vec_q latch at the drain boundary in
                     // the IRQ sequencer; not here, off the EX-redirect cone)
                     irq_active <= 1'b1;
                     id_valid   <= 1'b0;
                  end else if (running && !fetch_halt && !irq_active
                               && !(DEBUG_EN && dbg_active) && fetch_ok) begin
                     id_valid <= 1'b1;
                     if (DEBUG_EN) begin
                        id_bp         <= dbg_bp_hit;
                        dbg_skip      <= 1'b0;
                        dbg_step_done <= dbg_stepping;   // its one instruction
                     end
                     id_op    <= w_op;  id_var1 <= w_var1;  id_var2 <= w_var2;
                     if (w_short[32]) id_var1 <= w_short[31:0];
                     // B: the fused branch's displacement (4*simm13) rides in
                     // id_var2 (a 1-word op has no second immediate word)
                     // (also the 2-word FBR: its word 1 is the imm32 rhs)
                     if (w_op[31] != w_op[30] && w_op[29:26] == 4'hD)
                        id_var2 <= {{17{w_op[20]}}, w_op[20:8], 2'b00};
                     id_b_one  <= (w_dec.uop == U_ALU && w_op[29:26] == 4'h5);
                     id_rs1    <= w_dec.rs1;
                     id_ldc    <= (w_dec.uop == U_ALU || w_dec.uop == U_BOOL || w_dec.uop == U_FBR);
                     id_rd     <= w_dec.rd;
                     id_b_imm  <= (w_op[31:30] != 2'b01 || w_op[29:26] == 4'h2 ||
                                   (w_op[29:26] == 4'h3 && w_op[20]) ||
                                   (w_op[29:26] == 4'hD && w_op[21]) ||
                                   (EN_SHORT9 && w_op[29:26] == 4'hA && w_op[25:24] == 2'b11)) &&
                                  (w_dec.uop == U_MUL || w_dec.uop == U_DIV || w_dec.uop == U_MOD ||
                                   w_dec.uop == U_BOOL || w_dec.uop == U_ALU || w_dec.uop == U_FBR);
                     id_b_sext <= w_dec.sgn;
                     id_ser    <= w_decf.serialize || w_xser;
                     id_ras_ret <= EN_RAS && (w_decf.uop == U_RET || w_decf.uop == U_LEAVERET);
                     id_bp_ctr  <= pht[pc[BP_IDXW+1:2]];
                     id_rpend   <= 1'b1;
                     // 2a: BTIC lookup by this instruction's own pc
                     id_bt_hit  <= EN_BTIC && bt_v[bt_lidx_pc] && bt_tag[bt_lidx_pc] == pc[31:2];
                     id_bt_idx  <= bt_lidx_pc;
                     id_bt_li   <= bt_lidx[bt_lidx_pc];  id_bt_bli <= bt_blidx[bt_lidx_pc];
                     id_bt_d0   <= bt_d0[bt_lidx_pc];   id_bt_d1  <= bt_d1[bt_lidx_pc];
                     id_bt_v1   <= bt_v1[bt_lidx_pc];   id_bt_base <= bt_base[bt_lidx_pc];
                     id_bt_bp1  <= bt_bp1[bt_lidx_pc];  id_bt_bp2 <= bt_bp2[bt_lidx_pc];
                     id_bt_want <= bt_want[bt_lidx_pc];
                     id_bp_idx  <= pc[BP_IDXW+1:2];
                     id_bp_unc  <= EN_BP && (w_decf.uop == U_JMP || w_decf.uop == U_CALL) &&
                                   !w_decf.brind && w_decf.bcond == 4'd0;
                     id_bp_cnd  <= EN_BP && (w_decf.uop == U_JMP || w_decf.uop == U_CALL) &&
                                   !w_decf.brind && w_decf.bcond != 4'd0;
                     id_bp_fbr  <= EN_BP && w_decf.uop == U_FBR;
                     id_bp_abs  <= !w_decf.brel && w_decf.uop != U_FBR;
                     id_w      <= ((w_op[29:26] == 4'h3) && w_op[19] &&
                                   !(w_op[31:30] == 2'b01 && w_op[20])) ||
                                  // P9: short CMPRVW (class 3 LEN=01 SGN=1 B=0 INV=1)
                                  (EN_SHORT9 && w_op[29:26] == 4'h3 && w_op[31:30] == 2'b01 &&
                                   w_op[20] && !w_op[21] && w_op[22]) ||
                                  // HW proposal 6: FBR PRED 5-7 (EQW/LTW/ULTW)
                                  (EN_FBR_W && w_op[29:26] == 4'hD && w_op[25] &&
                                   (w_op[24] || w_op[23]));
                     id_fx      <= w_fx;  id_fxc <= w_fxc;  id_fx2 <= w_fx2;  id_fxg <= w_fxg;
                     id_fxm     <= w_fxm; id_fxl <= w_fxl;
                     id_mfsr    <= EN_EXC && w_op[31:26] == 6'b01_1011 && w_op[25:21] == 5'b00010;
                     id_use_rs1 <= w_dec.use_rs1;  id_use_rs2 <= w_dec.use_rs2;
                     id_use_rdd <= w_dec.use_rdd;  id_fread   <= w_dec.fread;
                     id_sp_rw   <= w_dec.sp_rd || w_dec.sp_wr;
                     id_pc    <= pc;
                     id_pc_nx <= w_pc_adv;
                     id_last_dw <= (w_op[31:30] == 2'b11) ? w_pc_p8[31:3]
                                 : (w_op[31:30] == 2'b10) ? w_pc_p4[31:3] : pc[31:3];
                     pc       <= w_pc_adv;   // precomputed pc+LEN (see IF section)
                     ev_disp  = 1'b1;        // M12: dword advance for pos shift
                     ev_adv   = (w_op[31:30] == 2'b00) ? 2'd0
                              : pc[2] ? ((w_op[31:30] == 2'b11) ? 2'd2 : 2'd1)
                                      : ((w_op[31:30] == 2'b01) ? 2'd0 : 2'd1);
                  end else begin
                     id_valid <= 1'b0;    // IF window miss: bubble, engine fills
                  end
               end

               // §3.3 misaligned access: it enters MEM as a non-memory parking
               // op (no port access, no writeback; parks kind 5 at WB), and
               // everything younger is dropped as behind a serializer.  An
               // IRQ / debug boundary taken on this same edge is cancelled
               // (the core parks first).  Older FBR / SMC squashes below
               // still win (NBA order) and kill it as wrong-path.
               if (ex_misal) begin
                  ev_misal     = 1'b1;
                  mem_d.uop    <= U_TRAP;
                  mem_d.wreg   <= 1'b0;
                  mem_d.is_mem <= 1'b0;
                  mem_lkq      <= 1'b0;  mem_mulq <= 1'b0;  mem_mulf <= 1'b0;  mem_fastq <= 1'b0;
                  ev_pcz       = 1'b1;
                  id_valid     <= 1'b0;
                  ex_valid     <= 1'b0;
                  fetch_halt   <= 1'b1;
                  irq_active   <= 1'b0;
                  if (DEBUG_EN) dbg_active <= 1'b0;
               end
            end
         end

         // ---------------- ISA v3 B: taken fused branch (MEM redirect) ------
         // Older than everything in EX/ID, so it overrides this edge's EX
         // redirect and dispatch writes (NBA last-write wins): squash the
         // two younger instructions and redirect. It never waits on the port
         // (not a memory op), so it always leaves MEM this edge. An IRQ
         // taken meanwhile drains first and then pushes this corrected pc.
         // 2a: a mispredicted return squashes the same way, redirecting to
         // its loaded target (the RET itself retires to WB this edge).
         // MMU stage 1: an access-fault op leaving MEM squashes everything
         // younger (EX / ID) and halts fetch; at WB it takes the exception.
         // (Before the SMC / EN_ST1 squashes below, which are older and win.)
         if (mem_xsq_go) begin
            if (DEBUG_EN) begin dbg_active <= 1'b0;  id_bp <= 1'b0; end
            ev_ovr     = 1'b1;
            mem_valid  <= 1'b0;
            ex_valid   <= 1'b0;
            id_valid   <= 1'b0;
            dly_on     <= 1'b0;
            div_ph     <= MD_IDLE;
            fetch_halt <= 1'b1;
            irq_active <= 1'b0;
         end
         if (fbr_taken || ras_miss) begin
            if (DEBUG_EN && ev_bp) dbg_active <= 1'b0;   // the ID op was wrong-path
            ev_pcz     = 1'b1;
            ev_ovr     = 1'b1;
            pc         <= fbr_taken ? fbr_tgt : ret_actual;
            mem_valid  <= 1'b0;
            ex_valid   <= 1'b0;
            id_valid   <= 1'b0;
            dly_on     <= 1'b0;
            div_ph     <= MD_IDLE;   // a wrong-path DIV in EX may have started
            fetch_halt <= 1'b0;
         end

         // ---------------- SMC squash (store into in-flight code) ----------
         // Runs LAST in the block: overrides this edge's dispatch/redirect
         // writes (the same ordering rule as the SoC MMIO-store-wins handler).
         // A completing DRAM store invalidates matching IFB slots and squashes
         // any YOUNGER in-flight instruction (IF window / ID / EX) whose word
         // span overlaps the stored dword; refetch from the oldest squashed.
         // Safe because side effects only exist from MEM onward.
         if (smc_st) begin
            if (ifb_val[0] && ifb_base == smc_dw)          ifb_val[0] <= 1'b0;
            if (ifb_val[1] && ifb_base + 29'd1 == smc_dw)  ifb_val[1] <= 1'b0;
            // M9: the NLB is a code copy too — same store-match rule (line-granular)
            if ((nlb_vhi || nlb_vlo) && nlb_line == smc_dw[28:1]) begin
               nlb_vhi <= 1'b0;  nlb_vlo <= 1'b0;
            end
            if ((nlb1_vhi || nlb1_vlo) && nlb1_line == smc_dw[28:1]) begin
               nlb1_vhi <= 1'b0;  nlb1_vlo <= 1'b0;
            end
         end
         // MMU stage 3: the micro-ITLB refill, through the main TLB's fetch
         // port (a TLBP in WB owns that port: tlb_psel, from registers).  The verdict
         // is for the fetch mode now (ut_um): user at 0x8000_0000+ and the
         // supervisor hole are segment faults; useg / kmap go through the
         // TLB and need X; kseg is direct; MMIO is identity.
         if (EN_TLB && it_ref && !tlb_psel) begin : ut_refill
            logic f_useg, f_kseg, f_kmap, f_seg, f_tlbs;
            f_useg = !it_vpn[15];
            f_kseg = it_vpn[15:11] == 5'b10000;
            f_kmap = it_vpn[15:14] == 2'b11 && it_vpn[15:12] != 4'hF;
            f_seg  = fe_um ? it_vpn[15] : (it_vpn[15:14] == 2'b10 && !f_kseg);
            f_tlbs = f_useg || f_kmap;
            ut_v    <= 1'b1;
            ut_vpn  <= it_vpn;
            ut_um   <= fe_um;
            ut_ok   <= !f_seg && (!f_tlbs || (tlb_f_hit && tlb_f_xwr[2]));
            ut_miss <= !f_seg && f_tlbs && !tlb_f_hit;
            ut_pa   <= f_tlbs ? {5'b0, tlb_f_pfn} : f_kseg ? {5'b0, it_vpn[10:0]} : it_vpn;
            it_ref  <= 1'b0;
         end
         // a TLBP takes the fetch port: its key (TLB_HI, final — TLBP
         // serializes) is loaded as it sits in MEM; a pending refill is
         // dropped and the lookup asks again afterwards
         if (tlb_pload) begin
            it_vpn <= tlb_hi_vpn_o;  it_asid <= tlb_hi_asid_o;  it_ref <= 1'b0;
         end
         // MMU stage 3: a TLB / ASID / TE change discards the fetch side's
         // virtual copies (the I-cache, physically tagged, is kept)
         if (tlb_discard) begin
            ut_v <= 1'b0;  it_ref <= 1'b0;
            m_vav <= 1'b0;  sq_vav <= 1'b0;   // (STLF: their VAs may now map elsewhere)
            nlb_vhi <= 1'b0;  nlb_vlo <= 1'b0;  pf_look <= 1'b0;  nlb1_vhi <= 1'b0;  nlb1_vlo <= 1'b0;
            if_look <= 1'b0;
            ifb_val <= 2'b00; ev_pcz = 1'b1; ev_ovr = 1'b1;
            if (if_xc) if_drop <= 1'b1;
         end
         if (icinv) begin
            for (int i = 0; i < IC_LINES; i++) begin ic_vhi[i] <= 1'b0; ic_vlo[i] <= 1'b0; end
            nlb_vhi <= 1'b0;  nlb_vlo <= 1'b0;  pf_look <= 1'b0;  nlb1_vhi <= 1'b0;  nlb1_vlo <= 1'b0;
            if_look <= 1'b0;  snoop_q <= 1'b0;
            ifb_val <= 2'b00; ev_pcz = 1'b1; ev_ovr = 1'b1;
         end
         if (sq_any) begin
            ev_ovr = 1'b1;
            if (DEBUG_EN && ev_bp) dbg_active <= 1'b0;   // squashed; it re-dispatches
            // M13 fix: the EX op is fetched-before-the-store too, so it must
            // not commit (it used to advance into MEM here and ALSO be
            // refetched: stale execution, then a deadlock when the ID op
            // dispatched this edge was a serializer that set fetch_halt).
            if (sq_ex) mem_valid <= 1'b0;
            // Only the ID op hit (!sq_ex): the EX op is older than it and
            // stays.  Normally it advanced into MEM on this edge, so EX holds
            // the squashed ID op and is cleared; but a busy multicycle op
            // (DIV/MUL, DELAY, LCD) did not advance and must be kept — clearing
            // it here dropped it, and the refetch starts after it (at id_pc).
            if (sq_ex || !ex_busy) ex_valid <= 1'b0;
            if (sq_ex) begin
               dly_on <= 1'b0;
               div_ph <= MD_IDLE;   // the squashed DIV may have started
            end
            id_valid   <= 1'b0;
            ifb_val    <= 2'b00;
            ev_pcz     = 1'b1;
            pc         <= sq_ex ? ex_pc : id_pc;
            // Every in-flight serializer is younger than the store, so it is
            // squashed here and re-arms fetch_halt when it re-dispatches.
            // (§3.3: a misaligned op that entered MEM this edge is older
            // than the squashed ID op — it parks, so fetch stays halted.)
            fetch_halt <= ev_misal && !sq_ex;
         end

         // SMC: an op latched into ID this edge from a now-stale IFB slot is
         // dropped and refetched from its own pc (ev_disp = it was latched;
         // a full sq_any above already refetches from an older pc).
         if (smc_hit_lat && ev_disp && !sq_any) begin
            ev_ovr   = 1'b1;
            if (DEBUG_EN) id_bp <= 1'b0;
            id_valid <= 1'b0;
            ifb_val  <= 2'b00;
            ev_pcz   = 1'b1;
            pc       <= pc;
         end

         // EN_ST1: the store that posted speculatively last edge overlaps
         // fetch-side code.  Everything younger (MEM, EX, ID, the IFB) was
         // fetched before its write: squash it all and refetch from the
         // store's fall-through.  Last in the block: wins over every other
         // redirect / dispatch this edge.  (MEM -> WB and the RAS push are
         // gated by sq_spec where they are written.)
         if (sq_spec) begin
            ev_ovr = 1'b1;
            if (DEBUG_EN && ev_bp) dbg_active <= 1'b0;
            if (DEBUG_EN) id_bp <= 1'b0;
            mem_valid  <= 1'b0;
            ex_valid   <= 1'b0;
            dly_on     <= 1'b0;
            div_ph     <= MD_IDLE;
            id_valid   <= 1'b0;
            ifb_val    <= 2'b00;
            // one cycle after the issue an I-cache lookup / prefetch started
            // before the snoop (clearing the line on this edge) may still be
            // in flight: drop them and the NLB, as ICACHE_INV does
            nlb_vhi    <= 1'b0;  nlb_vlo <= 1'b0;  nlb1_vhi <= 1'b0;  nlb1_vlo <= 1'b0;
            pf_look    <= 1'b0;  if_look <= 1'b0;
            ev_pcz     = 1'b1;
            pc         <= st_spec_nx;
            fetch_halt <= 1'b0;
         end

         // debugger register write (debug-halted: no WB activity competes)
         if (dbg_halted && dbg_i.reg_we) begin
            case (dbg_i.reg_idx)
               5'd16:   sp      <= dbg_i.reg_wdata[31:0];
               5'd17:   park_pc <= dbg_i.reg_wdata[31:0];   // resume PC
               5'd18:   flags   <= '{zero: dbg_i.reg_wdata[3], sign: dbg_i.reg_wdata[2],
                                     carry: dbg_i.reg_wdata[1], overflow: dbg_i.reg_wdata[0]};
               5'd19:   int_mask <= dbg_i.reg_wdata[3:0];
               // MMU stage 1: SP_ALT, {DI, UM}
               5'd20:   if (EN_EXC) sp_alt <= dbg_i.reg_wdata[31:0];
               5'd21:   if (EN_EXC) begin umode <= dbg_i.reg_wdata[0]; irq_dis <= dbg_i.reg_wdata[1]; end
               default: if (dbg_i.reg_idx < 5'd16) rf[dbg_i.reg_idx[3:0]] <= dbg_i.reg_wdata;
            endcase
         end

         // int_mask MMIO write-back (SoC handler owns 0xF00F_0000) — last, so
         // a memory-mapped store wins over a same-edge IRET restore.
         if (mask_wr) int_mask <= mask_wdata;

         // ---------------- 2a: BTIC capture and invalidation ----------------
         // Capture: the redirect of a missed entry has been served — the
         // window sits at the target with pc still on it.  (Slot 1 only when
         // it is in the same I-cache line, so one line index covers it.)
         // Not on an edge whose store snoop covers the entry's lines: the
         // invalidation below compares the entries' OLD line indices, so a
         // capture landing on the snoop edge would survive it holding the
         // pre-store window.  Nor on an EN_ST1 squash edge (the window may
         // predate the store).  The capture stays pending; a later, fresh
         // window at the target installs it.
         // (MMU stage 1: never a window at 0xF... — in user mode that is a
         // zero-served window, not a copy of an I-cache line)
         if (EN_BTIC && r_bt_pend && ifb_val[0] && pos[0] && ifb_base == r_bt_wbase &&
             !(EN_EXC && r_bt_wbase[28:25] == 4'hF) &&
             !(EN_TLB && (ifb_ft[0] || (ifb_val[1] && ifb_ft[1]))) &&   // (stage 3: a refused page)
             pc_dw == r_bt_wbase && !sq_spec &&
             !(snoop_q && (r_bt_wbase[IC_IDXW:1] == snoop_idx_q || r_bt_wblidx == snoop_idx_q))) begin
            bt_v[r_bt_widx]    <= 1'b1;
            bt_tag[r_bt_widx]  <= r_bt_wtag;
            bt_base[r_bt_widx] <= r_bt_wbase;
            bt_d0[r_bt_widx]   <= ifb_dw[0];
            bt_d1[r_bt_widx]   <= ifb_dw[1];
            bt_v1[r_bt_widx]   <= ifb_val[1] && !r_bt_wbase[0];
            bt_lidx[r_bt_widx] <= r_bt_wbase[IC_IDXW:1];
            bt_blidx[r_bt_widx] <= r_bt_wblidx;
            bt_bp1[r_bt_widx]  <= r_base_p1;    // = ifb_base + 1 (ifb_base == wbase)
            bt_bp2[r_bt_widx]  <= r_base_p2;
            bt_want[r_bt_widx] <= r_want_line;
            if (id_bt_idx == r_bt_widx) id_bt_hit <= 1'b0;   // its snapshot is stale
            r_bt_pend          <= 1'b0;
         end
         // Invalidation (after the capture: NBA last wins): the entry's
         // I-cache line is re-installed or store-snooped; ICACHE_INV.
         if (EN_BTIC) begin
            for (int k = 0; k < BT_N; k++) begin
               if (ev_icinst && (bt_lidx[k] == ev_icinst_idx || bt_blidx[k] == ev_icinst_idx)) bt_v[k] <= 1'b0;
               if (snoop_q   && (bt_lidx[k] == snoop_idx_q   || bt_blidx[k] == snoop_idx_q))   bt_v[k] <= 1'b0;
            end
            if (icinv || tlb_discard) bt_v <= '0;
            // the ID snapshot is a copy of one entry: the same kills apply to
            // it (an older snapshot via its own indices; one taken on this
            // edge via r_btk_* next cycle, in bt_hit)
            if (ev_icinst && (id_bt_li == ev_icinst_idx || id_bt_bli == ev_icinst_idx)) id_bt_hit <= 1'b0;
            if (snoop_q   && (id_bt_li == snoop_idx_q   || id_bt_bli == snoop_idx_q))   id_bt_hit <= 1'b0;
            if (icinv || tlb_discard) id_bt_hit <= 1'b0;
            r_btk_icv <= ev_icinst;  r_btk_ic <= ev_icinst_idx;
            r_btk_snv <= snoop_q;    r_btk_sn <= snoop_idx_q;
            r_btk_all <= icinv || tlb_discard;
         end

         // ---------------- M12 pos-tracker composer (runs LAST) ----------------
         // Folds this edge's events (blocking flags set by the real write
         // sites above) into the position flags and the base+K triple. Fold
         // order matters: dispatch shift, then slide shift-down (they
         // co-occur on eager slides), then the exclusive strong writers;
         // ev_pcz (redirect/RET/IRQ/squash/start) folds LAST and wins.
         begin : pos_track
            logic [2:0] p;
            p = pos;
            if (ev_disp)   p = p << ev_adv;
            if (ev_slide)  p = {1'b0, p[2:1]};
            if (ev_rebase) p = 3'b001;              // rebase base := pc's dword
            if (ev_fill)   p = {1'b0,
                                (pc_dw == r_ifq_p1),
                                (pc_dw == if_req_dw)};
            // (In-window redirect claims at the RET/exo edges were tried and
            // REVERTED: board-measured ZERO effect on the calls_fib +514k —
            // fib's call/return targets sit BEHIND the window base, which
            // neither the flags nor the old live compares ever covered — and
            // the claims added a compare cone plus an assertion-caught
            // stale-base corner. The +1c/leaf-call residual is priced in
            // PIPELINE_M12_TIMING.md.)
            if (ev_pcz)    p = 3'b000;              // conservative: forces rebase
            // 2a: window loaded at pc's dword — unless an older squash moved
            // pc after the dispatch (the base below still follows the load)
            if (ev_btic && !ev_ovr) p = 3'b001;
            pos <= p;
            // HW proposal 3: a redirect aborts any background prefetch (it was
            // for the old sequential path), so the BRAM read port reads the
            // new miss_dw on the very next cycle — the free first-cycle read
            // the demand lookup relies on.  (NBA here wins over the engine.)
            if (EN_PF_EARLY && ev_pcz) pf_look <= 1'b0;
            // A BTIC-hit redirect loaded the window itself (no demand lookup
            // follows), so its next line's prefetch starts on the same edge
            // instead of after the window is seen valid (~3 cycles later):
            // short target runs no longer drain the window before the NLB.
            // (not the full !ev_ovr guard: it put every squash — the RAS-miss
            // m_rdata -> ret_actual cone — into the pf_line enable.  Only the
            // common squash, a mispredicted fused branch in MEM (registers),
            // is excluded: a prefetch started on that edge would hold the
            // BRAM port the redirect's demand lookup needs next cycle.)
            if (EN_BTPF && ev_btic && !fbr_taken) begin
               pf_look <= 1'b1;
               pf_line <= id_bt_want;
            end
            // base+K triple: track this edge's base writer (sites are
            // mutually exclusive; pcz does not move base)
            if (ev_btic) begin                      // 2a: base' = the loaded base
               r_base_p1   <= id_bt_bp1;
               r_base_p2   <= id_bt_bp2;
               r_want_line <= id_bt_want;
            end else if (ev_rebase) begin
               r_base_p1   <= pc_dw + 29'd1;        // base' = miss_dw = pc_dw (!in0)
               r_base_p2   <= pc_dw + 29'd2;
               r_want_line <= pc_dw[28:1] + 28'd1;
            end else if (ev_fill) begin
               r_base_p1   <= r_ifq_p1;             // base' = if_req_dw
               r_base_p2   <= r_ifq_p1 + 29'd1;
               r_want_line <= if_req_dw[28:1] + 28'd1;
            end else if (ev_slide) begin
               r_base_p1   <= r_base_p2;            // base' = base + 1
               r_base_p2   <= r_base_p2 + 29'd1;
               r_want_line <= r_base_p1[28:1] + 28'd1;
            end
         end
      end
   end

   // ------------------------------------------------ MMU stage 1: mmu_sr
   // The system-register file, in a kept hierarchy so synthesis does not
   // fold its read mux / write decode into the dispatch and WB cones.
   // Written by MTSR as it commits at WB and by exception entry as it
   // completes; read at dispatch (MFSR, id_srv).
   assign mtsr_commit = EN_EXC && wb_valid && !wb_wait && wb_mtsr && !wb_park;
   assign tlbi_commit = EN_TLB && wb_valid && !wb_wait && wb_tlbi && !wb_park;
   // MMU stage 3: TLBWI, TLBINV, MTSR ASID and MTSR STATUS discard the fetch
   // side's virtual copies (window, NLB, BTIC, the micro-ITLB) and clear the
   // LR/SC reservation (MMU_PLAN.md §11.3)
   assign tlb_discard = EN_TLB && ((tlbi_commit && (wb_op[21:16] == 6'd9 || wb_op[21:16] == 6'd12)) ||
                                 (mtsr_commit && (wb_op[19:12] == 8'h33 || wb_op[19:12] == 8'h00)));
   wire sr_mtsr_we  = mtsr_commit && wb_op[19:16] == 4'h0;     // SRs 0-8 (mmu_sr)
   assign sr_ent_we = EN_EXC && irq_active && irq_xc && w_mrdy && exc_ent;
   logic [63:0] sr_rdata;
   // MFSR: SRs 0x10-0x2F are the MPU's (an SR number's low nibble alone
   // would alias 0-8)
   assign id_srv = (EN_TLB && id_op[19:16] == 4'h3) ? {32'b0, tlb_rdata}
                 : (EN_MPU && id_op[19:16] != 4'h0) ? {32'b0, mpu_rdata} : sr_rdata;
   generate if (EN_EXC) begin : g_sr
      (* keep_hierarchy = "yes" *)
      mmu_sr sr_i (
         .clk, .rst, .ce,
         .clr        (start),
         .mtsr_we    (sr_mtsr_we),
         .mtsr_idx   (wb_op[15:12]),
         .mtsr_wdata (wb_value),
         .ent_we     (sr_ent_we),
         .ent_cause  ({exc_guard_q, 4'b0, exc_cause_q}),
         .ent_addr   (exc_addr_q),
         .ridx       (id_op[15:12]),
         .sp_alt,
         .rdata      (sr_rdata),
         .evec, .ecause, .eaddr);
   end else begin : g_nosr
      assign sr_rdata = '0;  assign evec = '0;  assign ecause = '0;  assign eaddr = '0;
   end endgenerate

   // ----------------------------------------------- MMU stage 2: mmu_mpu
   // 16 regions (MMU_PLAN.md §10).  Written by an MTSR to 0x10-0x2F / STATUS
   // committing at WB; read at dispatch (MFSR).  Its data decision is on the
   // EA's granule (and the next one for a two-access op), its fetch
   // decision on pc's granule (and the next one): both only REGISTERED
   // (mem_xsq / id_fx), never on an EX->MEM or dispatch enable.
   generate if (EN_MPU) begin : g_mpu
      mmu_mpu mpu_i (
         .clk, .rst, .ce,
         .clr      (start),
         .we       (mtsr_commit && (wb_op[19:16] == 4'h1 || wb_op[19:16] == 4'h2)),  // (0x10-0x2F)
         .widx     (wb_op[16:12] - 5'h10),
         .wdata    (wb_value[31:0]),
         .en_we    (mtsr_commit && wb_op[19:12] == 8'd0),
         .en_wd    (wb_value[8]),
         .ridx     (id_op[16:12] - 5'h10),
         .rdata    (mpu_rdata),
         .mpu_en,
         .d_g      (ex_ea_q[31:5]),
         .d_gx     (ex_a[31:5]),
         .d_st     (ex_mst),
         .d_um     (umode),
         .d_deny1  (mpu_d1), .d_guard1 (mpu_dg1), .d_deny2 (mpu_d2), .d_guard2 (mpu_dg2),
         .f_g      (pc[31:5]),
         .f_um     (fe_um),
         .f_deny1  (mpu_f1), .f_guard1 (mpu_fg1), .f_deny2 (mpu_f2), .f_guard2 (mpu_fg2));
   end else begin : g_nompu
      assign mpu_en = 1'b0;  assign mpu_rdata = '0;
      assign mpu_d1 = 1'b0;  assign mpu_dg1 = 1'b0;  assign mpu_d2 = 1'b0;  assign mpu_dg2 = 1'b0;
      assign mpu_f1 = 1'b0;  assign mpu_fg1 = 1'b0;  assign mpu_f2 = 1'b0;  assign mpu_fg2 = 1'b0;
   end endgenerate

   // ----------------------------------------------- MMU stage 3: mmu_tlb
   // 32 entries + TLB_HI / TLB_LO / TLB_INDEX / ASID / TLBVEC (0x30-0x34)
   // and TE (STATUS[9]).  SR writes and TLB instructions act as they commit
   // at WB (serializing); MFSR reads at dispatch.
   // (data / fetch translation is wired in the following steps)
   assign tlb_hi_load = sr_ent_we && exc_tlb_q;
   assign tlb_hi_vpn  = exc_addr_q[31:16];
   assign tlb_f_vpn   = it_vpn;   // the micro-ITLB refill
   // (timing: the probe select is from registers, not the commit, which
   // waits on the bus; the commit only enables TLB_INDEX's write)
   assign tlb_psel    = EN_TLB && ((wb_valid && wb_tlbi && wb_op[21:16] == 6'd11) || tlb_pload);
   assign tlb_pload   = EN_TLB && mem_valid && mem_d.tlbi && mem_op[21:16] == 6'd11;
   generate if (EN_TLB) begin : g_tlb
      (* keep_hierarchy = "yes" *)
      mmu_tlb tlb_i (
         .clk, .rst, .ce,
         .clr      (start),
         .sr_we    (mtsr_commit && wb_op[19:16] == 4'h3),
         .sr_widx  (wb_op[14:12]),
         .sr_wdata (wb_value[31:0]),
         .te_we    (mtsr_commit && wb_op[19:12] == 8'h00),
         .te_wd    (wb_value[9]),
         .sr_ridx  (id_op[14:12]),
         .sr_rdata (tlb_rdata),
         .te, .asid (tlb_asid), .tlbvec (tlb_vec),
         .op_wi    (tlbi_commit && wb_op[21:16] == 6'd9),
         .op_r     (tlbi_commit && wb_op[21:16] == 6'd10),
         .op_p     (tlbi_commit && wb_op[21:16] == 6'd11),
         .op_inv   (tlbi_commit && wb_op[21:16] == 6'd12),
         .hi_load  (tlb_hi_load), .hi_vpn (tlb_hi_vpn),
         .d_vpn    (tlb_d_vpn), .d_hit (tlb_d_hit), .d_pfn (tlb_d_pfn), .d_xwr (tlb_d_xwr),
         .f_vpn    (tlb_f_vpn), .f_asid (it_asid), .hi_vpn_o (tlb_hi_vpn_o), .hi_asid_o (tlb_hi_asid_o),
         .f_hit (tlb_f_hit), .f_pfn (tlb_f_pfn), .f_xwr (tlb_f_xwr));
   end else begin : g_notlb
      assign te = 1'b0;  assign tlb_asid = '0;  assign tlb_vec = '0;  assign tlb_rdata = '0;
      assign tlb_d_hit = 1'b0;  assign tlb_d_pfn = '0;  assign tlb_d_xwr = '0;
      assign tlb_f_hit = 1'b0;  assign tlb_f_pfn = '0;  assign tlb_f_xwr = '0;
      assign tlb_hi_vpn_o = '0;  assign tlb_hi_asid_o = '0;
   end endgenerate

`ifndef SYNTHESIS
   // Load-hit pipe coverage (sim only, +HCOV): loads that left MEM through
   // the hit pipe as hits / misses, and loads kept off it by a same-line
   // store still on the port.
   int hc_hit = 0, hc_miss = 0, hc_stblk = 0;
   bit hc_on = $test$plusargs("HCOV");
   always_ff @(posedge clk) if (hc_on && !rst && running && !mem_busy) begin
      if (mem_valid && mem_h) begin
         if (mem_h1 ? h_ok : mem_hok) hc_hit++; else hc_miss++;
      end
      if (!ex_busy && ex_valid && ex_hld && !ex_misal && exo_iaddr[31:28] != 4'hF &&
          m_write_DV && m_addr[31:5] == exo_iaddr[31:5]) hc_stblk++;
   end
   final if (hc_on) $display("HCOV core hit=%0d miss=%0d store_blocked=%0d", hc_hit, hc_miss, hc_stblk);
   // +HCOV: cycles a store spends in MEM after its issue cycle (its 2nd
   // slot: posting, or waiting for ready), and the total cycles
   int sc_st2 = 0, sc_cyc = 0;
   always_ff @(posedge clk) if (hc_on && !rst && running) begin
      sc_cyc++;
      if (mem_valid && mem_is_write && mem_xc != 2'd0) sc_st2++;
   end
   final if (hc_on) $display("HCOV store2 %0d of %0d cycles", sc_st2, sc_cyc);

   // M12 pos-tracker self-checks. Position flags are ONE-DIRECTIONAL by
   // design (a set flag must be TRUE; a clear flag may be conservative —
   // post-redirect zeros force the rebase path, which is correct). The
   // base+K triple is a hard invariant whenever the window is armed.
   always_ff @(posedge clk) if (!rst && wb_valid && wb_fmask.zero && wb_zval)
      assert (wb_zero == (wb_value == 64'b0))
         else $fatal(1, "wb_zero stale: pc=%h value=%h", wb_pc, wb_value);
   always_ff @(posedge clk) if (!rst && running) begin
      if (ifb_val[0] && pos[0]) assert (pc_dw == ifb_base)
         else $fatal(1, "pos0 unsound: pc=%h base=%h", pc, ifb_base);
      if (ifb_val[1] && pos[1]) assert (pc_dw == ifb_base + 29'd1)
         else $fatal(1, "pos1 unsound: pc=%h base=%h", pc, ifb_base);
      if ((ifb_val != 2'b00) && pos[2]) assert (pc_dw == ifb_base + 29'd2)
         else $fatal(1, "pos2 unsound: pc=%h base=%h", pc, ifb_base);
      if (ifb_val != 2'b00) begin
         assert (r_base_p1 == ifb_base + 29'd1)
            else $fatal(1, "base_p1 drift: base=%h p1=%h", ifb_base, r_base_p1);
         assert (r_base_p2 == ifb_base + 29'd2)
            else $fatal(1, "base_p2 drift: base=%h p2=%h", ifb_base, r_base_p2);
         assert (r_want_line == ifb_base[28:1] + 28'd1)
            else $fatal(1, "want_line drift: base=%h want=%h", ifb_base, r_want_line);
      end
      // M12 hazard predecode must track the full decode of the latched op
      if (id_valid && dec.legal)
         assert (id_use_rs1 == dec.use_rs1 && id_use_rs2 == dec.use_rs2 &&
                 id_use_rdd == dec.use_rdd && id_fread == dec.fread &&
                 id_sp_rw == (dec.sp_rd || dec.sp_wr))
            else $fatal(1, "id hazard predecode drift: op=%h", id_op);
      if (mem_valid)
         assert ((mem_fastq && !mem_xsq) == (mem_fastld && EN_LD_WB))
            else $fatal(1, "mem_fastq drift: pc=%h", mem_pc);
      if (ex_valid && (ex_lb_a || ex_lb_b) && wb_valid && wb_h && wb_hok)
         assert (wb_ld && (!ex_lb_a || wb_rd == ex_d.rs1) && (!ex_lb_b || wb_rd == ex_d.rs2) &&
                 wb_hext == wb_ldval)
            else $fatal(1, "load bypass: producer not in WB: pc=%h", ex_pc);
      if (mem_valid)
         assert (mem_lkq == (mem_loadlk || (EN_MUL2 && mem_d.uop == U_MUL)) && mem_mulq == (EN_MUL2 && mem_d.uop == U_MUL))
            else $fatal(1, "mem_lkq / mem_mulq drift: pc=%h", mem_pc);
      if (id_valid)
         assert (id_rs1 == dec.rs1 && id_rd == dec.rd)
            else $fatal(1, "id_rs1/id_rd predecode drift: op=%h", id_op);
      if (id_valid && dec.legal)
         assert (id_b_one == (dec.uop == U_ALU && id_op[29:26] == 4'h5) &&
                 id_b_imm == ((dec.len != 2'b01 || id_op[29:26] == 4'h2 ||
                               (id_op[29:26] == 4'h3 && id_op[20]) ||
                               (id_op[29:26] == 4'hD && id_op[21]) ||
                               (EN_SHORT9 && id_op[29:26] == 4'hA && id_op[25:24] == 2'b11)) &&
                              (dec.uop == U_MUL || dec.uop == U_DIV || dec.uop == U_MOD ||
                               dec.uop == U_BOOL || dec.uop == U_ALU || dec.uop == U_FBR)) &&
                 id_b_sext == dec.sgn)
            else $fatal(1, "id ex_b-select predecode drift: op=%h", id_op);
      // 1a invariants: a fast load hands off only when WB can take it, and
      // its response arrives only while it is waiting in WB
      if (EN_LD_WB && mem_xc == 2'd1 && mem_fastld && !w_mrdy && !st_post_ok)
         assert (!wb_wait) else $fatal(1, "1a: fast load handed off while WB waits");
      if (EN_LD_WB && ld_out && w_mrdy)
         assert (wb_valid && wb_ld) else $fatal(1, "1a: fast-load response without the load in WB");
      // 2a: an early redirect never meets an EX redirect on one edge
      if (id_early_rd) assert (!(ex_valid && ex_redir)) else $fatal(1, "2a: early redirect met an EX redirect");
      // 2a: a BTIC hit's window is at the predicted target
      if (EN_BTIC && id_valid && id_bp_tk && bt_hit)
         assert (id_bt_base == id_bp_tgt[31:3]) else $fatal(1, "BTIC base != target: pc=%h", id_pc);
      // MMU stage 1: nothing younger than an IRET can have been decoded
      // under the old UM (IRET serializes; fetch rebases after its MEM edge)
      if (EN_EXC && wb_valid && wb_um_we)
         assert (!id_valid && !ex_valid && !mem_valid)
            else $fatal(1, "MMU: op in flight behind a committing IRET: pc=%h", wb_pc);
      // a sq_spec edge always has the posted store still on the port (it
      // posted last edge), so a younger store cannot post straight onto an
      // idle port on that edge (st_spec_now needs !st_post)
      if (sq_spec)
         assert (st_post || sq_v) else $fatal(1, "sq_spec with the port idle: pc=%h", mem_pc);
      if (id_valid)
         assert (id_ser == dec.serialize)
            else $fatal(1, "id serialize predecode drift: op=%h", id_op);
   end
`endif

endmodule


//===========================================================================
// mmu_sr — MMU stage 1 system registers (MMU_PLAN.md §6.1) and the MFSR
// read mux.  EADDR, SCRATCH0/1 and TP are a 4 x 64 distributed RAM (one
// write port: an MTSR committing at WB, or exception entry writing EADDR;
// read asynchronously at dispatch).  LUTRAM has no reset, so each entry has
// a valid flop, cleared by reset (and, for EADDR, by a program load), and
// reads as 0 until written — §3.1's "0 after reset".  EVEC (read every
// cycle: the vector and its != 0 test), ECAUSE (written with EADDR at
// entry; one RAM write port) and the CYCLE counter stay in flops.  USP /
// SP_ALT stays in the core (the stack swaps write it).
//===========================================================================
module mmu_sr (
   input  logic        clk,
   input  logic        rst,
   input  logic        ce,
   input  logic        clr,          // program load: EVEC / ECAUSE / EADDR <- 0
   input  logic        mtsr_we,      // an MTSR commits (WB)
   input  logic [3:0]  mtsr_idx,
   input  logic [63:0] mtsr_wdata,
   input  logic        ent_we,       // an exception entry completes
   input  logic [8:0]  ent_cause,    // {GUARD, 4'b0, cause}
   input  logic [31:0] ent_addr,
   input  logic [3:0]  ridx,         // MFSR SR number (low 4 bits; 0-8 decode)
   input  logic [31:0] sp_alt,
   output logic [63:0] rdata,
   output logic [31:0] evec,
   output logic [8:0]  ecause,
   output logic [31:0] eaddr
);
   // SR -> RAM slot: EADDR (3) 0, SCRATCH0 (5) 1, SCRATCH1 (6) 2, TP (7) 3
   function automatic logic [2:0] f_slot(logic [3:0] sr);   // {in RAM, slot}
      case (sr)
         4'd3:    return 3'b100;
         4'd5:    return 3'b101;
         4'd6:    return 3'b110;
         4'd7:    return 3'b111;
         default: return 3'b000;
      endcase
   endfunction
   (* ram_style = "distributed" *) logic [63:0] sram [0:3];
   logic [3:0]  sv;                        // slot written since reset
   logic [63:0] cycle;
   wire  [2:0]  ws  = f_slot(mtsr_idx);
   wire         rwe = ce && (ent_we || (mtsr_we && ws[2]));
   wire  [1:0]  wa  = ent_we ? 2'd0 : ws[1:0];
   wire  [63:0] wd  = ent_we ? {32'b0, ent_addr}
                    : (mtsr_idx == 4'd3) ? {32'b0, mtsr_wdata[31:0]} : mtsr_wdata;
   always_ff @(posedge clk) if (rwe) sram[wa] <= wd;
   always_ff @(posedge clk) begin
      if (rst) begin
         evec <= '0;  ecause <= '0;  cycle <= '0;  sv <= '0;
      end else if (ce) begin
         cycle <= cycle + 64'd1;          // every core clock from reset
         if (rwe) sv[wa] <= 1'b1;
         if (mtsr_we) case (mtsr_idx)
            4'd1: evec   <= {mtsr_wdata[31:2], 2'b00};
            4'd2: ecause <= mtsr_wdata[8:0];
            default: ;                    // STATUS: no writable bits; USP: core
         endcase
         if (ent_we) ecause <= ent_cause;
         if (clr) begin
            evec <= '0;  ecause <= '0;  sv[0] <= 1'b0;
         end
      end
   end
   wire [2:0]  rs = f_slot(ridx);
   wire [63:0] rram = sv[rs[1:0]] ? sram[rs[1:0]] : 64'b0;
   always_comb begin
      case (ridx)
         4'd1:    rdata = {32'b0, evec};
         4'd2:    rdata = {55'b0, ecause};
         4'd4:    rdata = {32'b0, sp_alt};
         4'd8:    rdata = cycle;
         default: rdata = rs[2] ? rram : 64'b0;   // EADDR / SCRATCH0-1 / TP
      endcase
   end
   assign eaddr = sv[0] ? sram[0][31:0] : 32'b0;   // (tb_vsim's view)
endmodule


//===========================================================================
// mmu_mpu — MMU stage 2 MPU (MMU_PLAN.md §10): 16 base / limit / permission
// regions at 32-byte granularity, MPU_EN, and the decision for one data
// access and one instruction fetch per cycle.
//
// A region covers granule g when EN and BASE <= g <= LIMIT.  Decision (with
// MPU_EN): any covering GUARD region denies (either mode); else supervisor
// is allowed; else user needs a covering non-GUARD region with the
// permission (data: R or W by direction; fetch: X).
//
// The 2nd granule (g + 1, for a two-access op or an instruction crossing a
// granule edge) reuses the first granule's magnitude compares:
//   BASE <= g+1  <=>  BASE <= g  or  BASE == g+1
//   g+1 <= LIMIT <=>  g <= LIMIT and g != LIMIT
// with g+1 from an incrementer on a register (d_gx = ex_a, f_g = pc), not
// on the EA adder.  (For the two-access ops d_g == d_gx: their EA is ex_a.)
//===========================================================================
module mmu_mpu (
   input  logic        clk,
   input  logic        rst,
   input  logic        ce,
   input  logic        clr,           // program load: MPU_EN <- 0
   input  logic        we,            // MTSR to an MPU SR commits
   input  logic [4:0]  widx,          // SR - 0x10: [4:1] region, [0] 0 = BASE / 1 = LIMIT
   input  logic [31:0] wdata,
   input  logic        en_we,         // MTSR STATUS commits
   input  logic        en_wd,         //   its [8]
   input  logic [4:0]  ridx,          // MFSR SR - 0x10
   output logic [31:0] rdata,
   output logic        mpu_en,
   input  logic [26:0] d_g,           // the EA's granule (EA[31:5])
   input  logic [26:0] d_gx,          // ex_a[31:5] (= d_g for two-access ops)
   input  logic        d_st,          // a store (needs W), else a load (R)
   input  logic        d_um,          // user mode
   output logic        d_deny1, d_guard1, d_deny2, d_guard2,
   input  logic [26:0] f_g,           // pc[31:5]
   input  logic        f_um,
   output logic        f_deny1, f_guard1, f_deny2, f_guard2
);
   localparam int N = 16;
   logic [26:0] base  [0:N-1];
   logic [26:0] limit [0:N-1];
   logic [4:0]  attr  [0:N-1];        // {GUARD, X, W, R, EN}
   always_ff @(posedge clk) begin
      if (rst) begin
         mpu_en <= 1'b0;
         for (int n = 0; n < N; n++) begin base[n] <= '0; limit[n] <= '0; attr[n] <= '0; end
      end else if (ce) begin
         if (we) begin
            if (!widx[0]) begin base[widx[4:1]] <= wdata[31:5];  attr[widx[4:1]] <= wdata[4:0]; end
            else                limit[widx[4:1]] <= wdata[31:5];
         end
         if (en_we) mpu_en <= en_wd;
         if (clr)   mpu_en <= 1'b0;
      end
   end
   // MFSR (timing / area): a LUTRAM copy written alongside the flops, with
   // a valid flop per SR (LUTRAM has no reset; reset reads 0) — not a 32:1
   // mux of the flops
   (* ram_style = "distributed" *) logic [31:0] shadow [0:2*N-1];
   logic [2*N-1:0] sv;
   always_ff @(posedge clk) if (ce && we) shadow[widx] <= widx[0] ? {wdata[31:5], 5'b0} : wdata;
   always_ff @(posedge clk) if (rst) sv <= '0; else if (ce && we) sv[widx] <= 1'b1;
   assign rdata = sv[ridx] ? shadow[ridx] : 32'b0;

   wire [26:0] d_g1 = d_gx + 27'd1;
   wire [26:0] f_g1 = f_g + 27'd1;
   logic [N-1:0] dc1, dc2, fc1, fc2, gd, pr, pw, px;
   always_comb begin
      for (int n = 0; n < N; n++) begin
         logic dge, dle, fge, fle;
         dge = d_g >= base[n];  dle = d_g <= limit[n];
         fge = f_g >= base[n];  fle = f_g <= limit[n];
         dc1[n] = attr[n][0] && dge && dle;
         dc2[n] = attr[n][0] && (dge || d_g1 == base[n]) && dle && (d_gx != limit[n]);
         fc1[n] = attr[n][0] && fge && fle;
         fc2[n] = attr[n][0] && (fge || f_g1 == base[n]) && fle && (f_g != limit[n]);
         gd[n]  = attr[n][4];
         pr[n]  = attr[n][1];  pw[n] = attr[n][2];  px[n] = attr[n][3];
      end
   end
   wire [N-1:0] pd = d_st ? pw : pr;
   assign d_guard1 = mpu_en && |(dc1 & gd);
   assign d_guard2 = mpu_en && |(dc2 & gd);
   assign d_deny1  = d_guard1 || (mpu_en && d_um && !(|(dc1 & ~gd & pd)));
   assign d_deny2  = d_guard2 || (mpu_en && d_um && !(|(dc2 & ~gd & pd)));
   assign f_guard1 = mpu_en && |(fc1 & gd);
   assign f_guard2 = mpu_en && |(fc2 & gd);
   assign f_deny1  = f_guard1 || (mpu_en && f_um && !(|(fc1 & ~gd & px)));
   assign f_deny2  = f_guard2 || (mpu_en && f_um && !(|(fc2 & ~gd & px)));
endmodule


//===========================================================================
// mmu_tlb — MMU stage 3 TLB (MMU_PLAN.md §11.3): 32 fully associative
// entries {VPN[15:0], ASID[7:0], G, V, PFN[10:0], X/W/R}, the TLB SRs
// (TLB_HI 0x30, TLB_LO 0x31, TLB_INDEX 0x32, ASID 0x33, TLBVEC 0x34), TE
// (STATUS[9]) and the TLB instructions.  Two lookup ports with the current
// ASID: d_* (data) and f_* (fetch; with op_p it is TLBP's probe of
// TLB_HI's VPN / ASID).  An entry matches when V, VPN equal, and G or its
// ASID equals the lookup's.  TLBR reads a LUTRAM copy of the entry fields
// (V from the flops); the CAMs read the flops.
//===========================================================================
module mmu_tlb (
   input  logic        clk,
   input  logic        rst,
   input  logic        ce,
   input  logic        clr,           // program load: as reset
   input  logic        sr_we,         // MTSR to 0x30-0x34 commits
   input  logic [2:0]  sr_widx,
   input  logic [31:0] sr_wdata,
   input  logic        te_we,         // MTSR STATUS commits
   input  logic        te_wd,         //   its [9]
   input  logic [2:0]  sr_ridx,
   output logic [31:0] sr_rdata,
   output logic        te,
   output logic [7:0]  asid,
   output logic [31:0] tlbvec,
   input  logic        op_wi, op_r, op_p, op_inv,
   input  logic        hi_load,       // exception entry: TLB_HI <- {hi_vpn, G 0, ASID}
   input  logic [15:0] hi_vpn,
   input  logic [15:0] d_vpn,
   output logic        d_hit,
   output logic [10:0] d_pfn,
   output logic [2:0]  d_xwr,
   input  logic [15:0] f_vpn,         // registers: the micro-ITLB refill, or TLB_HI for a TLBP
   input  logic [7:0]  f_asid,
   output logic [15:0] hi_vpn_o,      // TLB_HI's VPN / ASID (the TLBP's search key)
   output logic [7:0]  hi_asid_o,
   output logic        f_hit,
   output logic [10:0] f_pfn,
   output logic [2:0]  f_xwr
);
   localparam int N = 32;
   logic [N-1:0]  e_v, e_g;
   logic [15:0]   e_vpn  [0:N-1];
   logic [7:0]    e_asid [0:N-1];
   logic [10:0]   e_pfn  [0:N-1];
   logic [2:0]    e_xwr  [0:N-1];
   // TLBR's copy: {VPN, G, ASID, PFN, XWR}
   (* ram_style = "distributed" *) logic [38:0] e_copy [0:N-1];
   logic [15:0] hi_v;  logic hi_g;  logic [7:0] hi_a;      // TLB_HI
   logic [10:0] lo_pfn; logic [2:0] lo_xwr; logic lo_v;   // TLB_LO
   logic [4:0]  idx;  logic idx_p;                         // TLB_INDEX
   logic [29:0] vec;

   function automatic logic [N-1:0] f_match(logic [15:0] vpn, logic [7:0] a);
      logic [N-1:0] m;
      for (int n = 0; n < N; n++) m[n] = e_v[n] && e_vpn[n] == vpn && (e_g[n] || e_asid[n] == a);
      return m;
   endfunction
   function automatic logic [4:0] f_enc(logic [N-1:0] m);
      logic [4:0] r;
      r = '0;
      for (int n = 0; n < N; n++) r = r | (m[n] ? 5'(n) : 5'd0);
      return r;
   endfunction
   wire [N-1:0] dm = f_match(d_vpn, asid);
   // one fetch-port CAM: TLBP's key is loaded into f_vpn / f_asid while
   // the TLBP is in MEM (MMU stage 3 timing: no probe mux, all registers)
   wire [N-1:0] fm = f_match(f_vpn, f_asid);
   assign hi_vpn_o = hi_v;  assign hi_asid_o = hi_a;
   always_comb begin
      d_pfn = '0;  d_xwr = '0;  f_pfn = '0;  f_xwr = '0;
      for (int n = 0; n < N; n++) begin
         d_pfn |= dm[n] ? e_pfn[n] : 11'd0;  d_xwr |= dm[n] ? e_xwr[n] : 3'd0;
         f_pfn |= fm[n] ? e_pfn[n] : 11'd0;  f_xwr |= fm[n] ? e_xwr[n] : 3'd0;
      end
   end
   assign d_hit = |dm;
   assign f_hit = |fm;

   always_ff @(posedge clk) if (ce && op_wi) e_copy[idx] <= {hi_v, hi_g, hi_a, lo_pfn, lo_xwr};
   wire [38:0] rd = e_copy[idx];

   always_ff @(posedge clk) begin
      if (rst || (ce && clr)) begin
         te <= 1'b0;  asid <= '0;  vec <= '0;  e_v <= '0;
         if (rst) begin
            hi_v <= '0; hi_g <= 1'b0; hi_a <= '0; lo_pfn <= '0; lo_xwr <= '0; lo_v <= 1'b0;
            idx <= '0; idx_p <= 1'b0;
         end
      end else if (ce) begin
         if (sr_we) case (sr_widx)
            3'd0: begin hi_v <= sr_wdata[31:16]; hi_g <= sr_wdata[8]; hi_a <= sr_wdata[7:0]; end
            3'd1: begin lo_pfn <= sr_wdata[26:16]; lo_xwr <= sr_wdata[3:1]; lo_v <= sr_wdata[0]; end
            3'd2: begin idx <= sr_wdata[4:0]; idx_p <= sr_wdata[31]; end
            3'd3: asid <= sr_wdata[7:0];
            3'd4: vec <= sr_wdata[31:2];
            default: ;
         endcase
         if (te_we) te <= te_wd;
         if (hi_load) begin hi_v <= hi_vpn; hi_g <= 1'b0; hi_a <= asid; end
         if (op_wi) begin
            e_v[idx] <= lo_v;  e_vpn[idx] <= hi_v;  e_g[idx] <= hi_g;  e_asid[idx] <= hi_a;
            e_pfn[idx] <= lo_pfn;  e_xwr[idx] <= lo_xwr;
         end
         if (op_r) begin
            {hi_v, hi_g, hi_a, lo_pfn, lo_xwr} <= rd;
            lo_v <= e_v[idx];
         end
         if (op_p) begin
            if (f_hit) begin idx <= f_enc(fm); idx_p <= 1'b0; end
            else       idx_p <= 1'b1;                    // [4:0] unchanged on a miss
         end
         if (op_inv) e_v <= '0;
      end
   end
   always_comb begin
      case (sr_ridx)
         3'd0:    sr_rdata = {hi_v, 7'b0, hi_g, hi_a};
         3'd1:    sr_rdata = {5'b0, lo_pfn, 12'b0, lo_xwr, lo_v};
         3'd2:    sr_rdata = {idx_p, 26'b0, idx};
         3'd3:    sr_rdata = {24'b0, asid};
         3'd4:    sr_rdata = {vec, 2'b00};
         default: sr_rdata = 32'b0;
      endcase
   end
   assign tlbvec = {vec, 2'b00};
endmodule

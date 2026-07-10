//===========================================================================
// pipeline_core — Phase B in-order pipeline (feat/pipeline). See PIPELINE_IMPL.md.
//   M1 ALU (IF/ID/EX/WB) · M2 MEM stage + loads/stores · M3 branches ·
//   M4 mul/div busy interlock.
// Interlock-only (no forwarding until M6): RAW deps stall ID until the producer
// leaves WB; flag-consuming branches stall until the flag producer leaves WB.
//
// Instruction subset (ISA v2, 1 word):
//   Class 1 ALU  rd = rs1 OP rs2 (aluop=op[25:22], F=op[21]).
//   Class 2 MUL  rd = rs1*rs2 (op[23]=is_high, op[22]=is_unsigned). Models the
//           silicon DSP48 chain (AREG/BREG -> MREG -> PREG): issue + 3 hold
//           cycles in EX = the FSM's MULTIPLY_SETUP/BREG/CALC/PIPE; flags
//           Z/S/V per MULTIPLY_WRITEBACK (C preserved).
//   Class 3 DIV  rd = rs1 /|% rs2 (op[23]=is_mod, op[22]=is_signed). Models the
//           silicon iterative unit: setup (abs/signs, = f_div_setup) ->
//           DIVIDE_PREP (CLZ skip) -> DIVIDE_STEP 1 bit/cycle -> finish; flags
//           Z/V (S/C preserved). Divide-by-zero resolves in 1 EX cycle:
//           DIV -> all-ones, MOD -> dividend, V=1 (Z/S/C preserved).
//   Class 6 load  rd = mem[rs1+rs2].
//   Class 7 store mem[rs1+rs2] = rd.
//   Class 8 branch (M3 model): cond=op[25:22] (f_cond_eval), signed word offset =
//           op[15:0]; resolves in EX; taken -> PC = branch_pc + offset*4, flush.
//   rd=op[11:8], rs1=op[7:4], rs2=op[3:0]. op==0 = NOP/bubble.
//
// M4 busy interlock: while a mul/div occupies EX, IF/ID freeze (PC + ID latch
// hold) and MEM/WB drain on bubbles; on the completion cycle the result+flags
// enter the EX->MEM latch and the front end resumes. Because mul/div write only
// SOME flag fields, the preserved fields are read from the committed flags
// register at completion — so ID holds a mul/div while any older flag-writer
// is still in flight (flag_busy), the same discipline branches use.
//===========================================================================
module pipeline_core
   import klauss_pkg::*;
(
   input  logic        clk,
   input  logic        rst,
   output logic [31:0] imem_addr,
   input  logic [31:0] imem_data,
   output logic [31:0] dmem_addr,
   output logic [63:0] dmem_wdata,
   output logic        dmem_we,
   input  logic [63:0] dmem_rdata,
   output logic [63:0] dbg_r [0:15]
);

   logic [63:0] rf [0:15];
   flags_t      flags;
   logic [31:0] pc;

   logic        id_valid;
   logic [31:0] id_op, id_pc;

   logic        ex_valid, ex_is_load, ex_is_store, ex_is_branch, ex_is_mul, ex_is_div;
   logic        ex_wreg, ex_wflags;
   logic [3:0]  ex_aluop, ex_rd, ex_cond;
   logic        ex_inv;
   logic [63:0] ex_a, ex_b, ex_stdata;
   logic [31:0] ex_target;
   logic        ex_cin;

   logic        mem_valid, mem_is_load, mem_is_store, mem_wreg, mem_wflags;
   logic [3:0]  mem_rd;
   logic [63:0] mem_alu, mem_addr_r, mem_stdata;
   flags_t      mem_flags;

   logic        wb_valid, wb_wreg, wb_wflags;
   logic [3:0]  wb_rd;
   logic [63:0] wb_value;
   flags_t      wb_flags;

   // ---- M4 multiply unit — the silicon free-running DSP48 pipeline
   //      (r_mul_operand_*_q = AREG/BREG, mul_pipe1 = MREG, mul_pipe2 = PREG).
   logic [64:0]  mul_a_q, mul_b_q;
   logic [127:0] mul_pipe1, mul_pipe2;
   logic [1:0]   mul_cnt;    // cycles the mul has held EX; PREG valid at 3

   // ---- M4 divide unit — the silicon restoring divider (klauss_pkg div_state_t)
   typedef enum logic [1:0] { MD_IDLE, MD_PREP, MD_STEP } md_phase_t;
   md_phase_t  div_ph;
   div_state_t dv;

   // ---- ID decode ----
   wire [3:0] d_class = id_op[29:26];
   wire [3:0] d_aluop = id_op[25:22];
   wire [3:0] d_cond  = id_op[25:22];
   wire       d_inv   = id_op[21];
   wire [3:0] d_rd    = id_op[11:8];
   wire [3:0] d_rs1   = id_op[7:4];
   wire [3:0] d_rs2   = id_op[3:0];
   wire signed [15:0] d_boff = id_op[15:0];
   wire       d_is_alu    = id_valid && (id_op != 32'h0) && (d_class == 4'd1);
   wire       d_is_mul    = id_valid && (d_class == 4'd2);
   wire       d_is_div    = id_valid && (d_class == 4'd3);
   wire       d_is_load   = id_valid && (d_class == 4'd6);
   wire       d_is_store  = id_valid && (d_class == 4'd7);
   wire       d_is_branch = id_valid && (d_class == 4'd8);
   wire       d_valid_op  = d_is_alu || d_is_mul || d_is_div || d_is_load || d_is_store || d_is_branch;
   wire       d_wreg      = d_is_alu || d_is_mul || d_is_div || d_is_load;
   wire       d_wflags    = (d_is_alu && (d_aluop <= 4'd3)) || d_is_mul || d_is_div;

   // ---- Interlock: source reg busy if pending dest in EX/MEM/WB ----
   wire b_ex  = ex_valid  && ex_wreg;
   wire b_mem = mem_valid && mem_wreg;
   wire b_wb  = wb_valid  && wb_wreg;
   wire hit1 = (b_ex && ex_rd == d_rs1) || (b_mem && mem_rd == d_rs1) || (b_wb && wb_rd == d_rs1);
   wire hit2 = (b_ex && ex_rd == d_rs2) || (b_mem && mem_rd == d_rs2) || (b_wb && wb_rd == d_rs2);
   wire hitd = (b_ex && ex_rd == d_rd ) || (b_mem && mem_rd == d_rd ) || (b_wb && wb_rd == d_rd );
   // ---- Flag interlock: branches READ flags; mul/div PARTIALLY write them
   //      (preserved fields come from the committed register at completion) —
   //      both wait for any older flag producer to leave WB.
   wire flag_busy = (ex_valid && ex_wflags) || (mem_valid && mem_wflags) || (wb_valid && wb_wflags);
   wire raw_stall = d_valid_op && ( hit1 || hit2 || (d_is_store && hitd) ||
                                    ((d_is_branch || d_is_mul || d_is_div) && flag_busy) );

   assign imem_addr = pc;
   genvar gi;
   generate for (gi = 0; gi < 16; gi++) assign dbg_r[gi] = rf[gi]; endgenerate

   alu_res_t    ex_res;
   assign ex_res = f_alu_ex(ex_a, ex_b, ex_aluop, ex_cin);
   wire [63:0]  ex_eaddr = ex_a + ex_b;

   // Branch resolves in EX against the (committed) flags register.
   wire [1:0] ex_cev   = f_cond_eval(flags, ex_cond, ex_inv);
   wire       ex_taken = ex_valid && ex_is_branch && ex_cev[0];

   // ---- M4: mul/div occupancy of EX. Mode bits ride the ex_aluop latch
   //      (= op[25:22]): mul {high,unsigned}, div {mod,signed} in bits [1:0].
   wire ex_mul_high   = ex_aluop[1];
   wire ex_mul_uns    = ex_aluop[0];
   wire ex_div_mod    = ex_aluop[1];
   wire ex_div_signed = ex_aluop[0];

   wire ex_div_by0 = ex_valid && ex_is_div && (div_ph == MD_IDLE) && (ex_b == 64'd0);
   wire md_done    = (ex_valid && ex_is_mul && mul_cnt == 2'd3) ||
                     (ex_valid && ex_is_div && div_ph == MD_STEP && dv.counter >= 7'd64) ||
                     ex_div_by0;
   wire ex_is_md   = ex_valid && (ex_is_mul || ex_is_div);
   wire md_busy    = ex_is_md && !md_done;

   // Divider datapath wires (same shapes as KlaussCPU.sv w_div_*).
   wire [63:0] w_div_abs_a   = (ex_div_signed && ex_a[63]) ? (~ex_a + 64'd1) : ex_a;
   wire [63:0] w_div_abs_b   = (ex_div_signed && ex_b[63]) ? (~ex_b + 64'd1) : ex_b;
   wire [63:0] w_div_shifted = {dv.remainder[62:0], dv.dividend[63]};
   wire [64:0] w_div_trial   = {1'b0, w_div_shifted} - {1'b0, dv.divisor};
   wire        w_div_borrow  = w_div_trial[64];

   // Local copy of KlaussCPU.sv count_leading_zeros (bit 6 set <=> input == 0).
   function automatic logic [6:0] f_clz64(input logic [63:0] val);
      f_clz64 = 7'd64;
      for (int i = 0; i < 64; i++) if (val[i]) f_clz64 = 7'd63 - 7'(i);
   endfunction

   // Completion value + flags, sampled only on the md_done cycle. Preserved
   // flag fields read the committed register — final, because ID held the
   // mul/div until older flag-writers drained (flag_busy term in raw_stall).
   logic [63:0] md_value;
   flags_t      md_flags;
   always_comb begin : md_mux
      logic [63:0] q_raw;
      logic        q_neg;
      md_flags = flags;
      md_value = 'x;
      q_raw    = ex_div_mod ? dv.remainder : dv.quotient;
      q_neg    = dv.is_signed && (ex_div_mod ? dv.sign_r : dv.sign_q);
      if (ex_is_mul) begin                    // = MULTIPLY_WRITEBACK
         md_value      = ex_mul_high ? mul_pipe2[127:64] : mul_pipe2[63:0];
         md_flags.zero = (md_value == 64'd0);
         md_flags.sign = md_value[63];
         if (ex_mul_high)
            md_flags.overflow = 1'b0;
         else if (ex_mul_uns)
            md_flags.overflow = (mul_pipe2[127:64] != 64'd0);
         else
            md_flags.overflow = (mul_pipe2[127:64] != {64{mul_pipe2[63]}});
      end else if (ex_div_by0) begin          // = f_div_setup by-zero guard
         md_value          = ex_div_mod ? ex_a : 64'hFFFFFFFFFFFFFFFF;
         md_flags.overflow = 1'b1;
      end else begin                          // = DIVIDE_STEP finish branch
         md_value          = q_neg ? (~q_raw + 64'd1) : q_raw;
         md_flags.zero     = (q_raw == 64'd0);
         md_flags.overflow = 1'b0;
      end
   end

   assign dmem_addr  = mem_addr_r[31:0];
   assign dmem_wdata = mem_stdata;
   assign dmem_we    = mem_valid && mem_is_store;
   wire [63:0] mem_value = mem_is_load ? dmem_rdata : mem_alu;

   always_ff @(posedge clk) begin
      if (rst) begin
         pc <= 0; id_valid <= 0; ex_valid <= 0; mem_valid <= 0; wb_valid <= 0; flags <= '0;
         mul_cnt <= '0; div_ph <= MD_IDLE; dv <= '0;
      end else begin
         if (wb_valid && wb_wreg)   rf[wb_rd] <= wb_value;
         if (wb_valid && wb_wflags) flags     <= wb_flags;

         wb_valid  <= mem_valid;   wb_rd <= mem_rd;   wb_value <= mem_value;
         wb_flags  <= mem_flags;   wb_wreg <= mem_wreg;   wb_wflags <= mem_wflags;

         // Free-running multiply chain, as on silicon: latches whatever EX
         // holds every cycle; only the PREG-valid (mul_cnt==3) sample is used.
         mul_a_q   <= {(ex_mul_uns ? 1'b0 : ex_a[63]), ex_a};
         mul_b_q   <= {(ex_mul_uns ? 1'b0 : ex_b[63]), ex_b};
         mul_pipe1 <= $signed(mul_a_q) * $signed(mul_b_q);
         mul_pipe2 <= mul_pipe1;

         // Iterative divider: issue -> MD_PREP (CLZ skip) -> MD_STEP (1 bit/cycle).
         case (div_ph)
            MD_IDLE: if (ex_valid && ex_is_div && ex_b != 64'd0) begin
               dv.dividend  <= w_div_abs_a;   dv.divisor   <= w_div_abs_b;
               dv.quotient  <= '0;            dv.remainder <= '0;   dv.counter <= '0;
               dv.sign_q    <= ex_a[63] ^ ex_b[63];
               dv.sign_r    <= ex_a[63];
               dv.is_signed <= ex_div_signed;
               dv.op        <= ex_div_mod ? DIV_OP_MOD : DIV_OP_DIV;
               div_ph       <= MD_PREP;
            end
            MD_PREP: begin : divide_prep
               logic [6:0] prep_clz;
               prep_clz = f_clz64(dv.dividend);
               if (prep_clz[6]) begin          // dividend == 0: straight to finish
                  dv.counter <= 7'd64;
               end else begin
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
               end else begin                  // finish cycle: md_done, EX advances
                  dv.op  <= DIV_OP_NONE;
                  div_ph <= MD_IDLE;
               end
            end
            default: div_ph <= MD_IDLE;
         endcase

         if (md_busy) begin
            // M4 busy interlock: mul/div holds EX; IF/ID freeze (pc, id_* keep
            // their values), MEM/WB drain on a bubble.
            mem_valid <= 1'b0;
            if (ex_is_mul) mul_cnt <= mul_cnt + 2'd1;
         end else begin
            mem_valid   <= ex_valid;   mem_is_load <= ex_is_load;  mem_is_store <= ex_is_store;
            mem_rd      <= ex_rd;
            mem_alu     <= ex_is_md ? md_value : ex_res.result;
            mem_addr_r  <= ex_eaddr;
            mem_stdata  <= ex_stdata;
            mem_flags   <= ex_is_md ? md_flags : ex_res.flags;
            mem_wreg    <= ex_wreg;    mem_wflags <= ex_wflags;

            if (ex_taken) begin
               // Taken branch: redirect and flush the wrong-path instr in ID (and the
               // in-flight IF fetch, discarded by moving PC before it latches).
               pc       <= ex_target;
               id_valid <= 1'b0;
               ex_valid <= 1'b0;
            end else if (raw_stall) begin
               ex_valid <= 1'b0;      // bubble; hold ID/pc
            end else begin
               ex_valid    <= d_valid_op;
               ex_is_load  <= d_is_load;   ex_is_store <= d_is_store;  ex_is_branch <= d_is_branch;
               ex_is_mul   <= d_is_mul;    ex_is_div   <= d_is_div;    mul_cnt <= 2'd0;
               ex_a        <= rf[d_rs1];   ex_b <= rf[d_rs2];          ex_stdata <= rf[d_rd];
               ex_aluop    <= d_aluop;     ex_cond <= d_cond;  ex_inv <= d_inv;  ex_rd <= d_rd;
               ex_cin      <= flags.carry;
               ex_wreg     <= d_wreg;      ex_wflags <= d_wflags;
               ex_target   <= id_pc + (32'(d_boff) << 2);   // branch target (PC-relative words)

               id_valid <= 1'b1;  id_op <= imem_data;  id_pc <= pc;
               pc       <= pc + 32'd4;
            end
         end
      end
   end

endmodule

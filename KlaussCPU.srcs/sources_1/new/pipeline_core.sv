//===========================================================================
// pipeline_core — Phase B in-order pipeline (feat/pipeline). See PIPELINE_IMPL.md.
//
// MILESTONE M1: IF / ID / EX / WB for Class-1 ALU reg-reg ops, INTERLOCK-ONLY
// (no forwarding — a RAW dependency stalls ID until the producer leaves WB).
// Shared instruction port (imem_addr/imem_data) backed by the testbench. This is
// the correctness skeleton; loads/branches/mul-div/exceptions land in M2..M5, and
// forwarding (EX->EX 2-input) in M6.
//
// Instruction subset (ISA v2 Class 1, LEN=01, 1 word):
//   op[29:26]=1 (class), op[25:22]=aluop (0=ADD 1=SUB 2=ADC 3=SBC 4=AND 5=OR
//   6=XOR), op[21]=F, rd=op[11:8], rs1=op[7:4], rs2=op[3:0].
// Sentinel: op == 32'h0 is treated as a bubble/NOP (no writes, no stall source).
//===========================================================================
module pipeline_core
   import klauss_pkg::*;
(
   input  logic        clk,
   input  logic        rst,
   output logic [31:0] imem_addr,   // byte address of the instruction to fetch
   input  logic [31:0] imem_data,   // instruction word at imem_addr (comb from tb)
   output logic [63:0] dbg_r [0:15] // register-file view for the testbench
);

   // ---- Architectural state (central; RF/flags written at WB only) ----
   logic [63:0] rf [0:15];
   flags_t      flags;
   logic [31:0] pc;

   // ---- Stage registers ----
   // IF is implicit (pc + imem). ID holds the fetched word; EX holds operands;
   // WB holds the result to commit.
   logic        id_valid;
   logic [31:0] id_op;

   logic        ex_valid;
   logic [63:0] ex_a, ex_b;
   logic [3:0]  ex_aluop;
   logic [3:0]  ex_rd;
   logic        ex_cin;
   logic        ex_wreg, ex_wflags;

   logic        wb_valid;
   logic [3:0]  wb_rd;
   logic [63:0] wb_value;
   flags_t      wb_flags;
   logic        wb_wreg, wb_wflags;

   // ---- ID decode (combinational on id_op) ----
   wire [3:0] d_class = id_op[29:26];
   wire [3:0] d_aluop = id_op[25:22];
   wire [3:0] d_rd    = id_op[11:8];
   wire [3:0] d_rs1   = id_op[7:4];
   wire [3:0] d_rs2   = id_op[3:0];
   wire       d_is_alu  = id_valid && (id_op != 32'h0) && (d_class == 4'd1);
   wire       d_wreg    = d_is_alu;                       // every M1 ALU op writes rd
   wire       d_wflags  = d_is_alu && (d_aluop <= 4'd3);  // ADD/SUB/ADC/SBC set flags

   // ---- Interlock (no forwarding): stall ID while a source reg is still being
   // written by an instruction in EX or WB (RF not yet updated). ----
   wire raw_hit_ex = ex_valid && ex_wreg && ((ex_rd == d_rs1) || (ex_rd == d_rs2));
   wire raw_hit_wb = wb_valid && wb_wreg && ((wb_rd == d_rs1) || (wb_rd == d_rs2));
   wire raw_stall  = d_is_alu && (raw_hit_ex || raw_hit_wb);

   assign imem_addr = pc;
   genvar gi;
   generate for (gi = 0; gi < 16; gi++) assign dbg_r[gi] = rf[gi]; endgenerate

   // ---- EX combinational compute ----
   alu_res_t ex_res;
   assign ex_res = f_alu_ex(ex_a, ex_b, ex_aluop, ex_cin);

   integer i;
   always_ff @(posedge clk) begin
      if (rst) begin
         pc       <= 32'h0;
         id_valid <= 1'b0;
         ex_valid <= 1'b0;
         wb_valid <= 1'b0;
         flags    <= '0;
         // rf intentionally not reset — the testbench seeds it.
      end else begin
         // -- WB: commit to architectural state --
         if (wb_valid && wb_wreg)   rf[wb_rd] <= wb_value;
         if (wb_valid && wb_wflags) flags     <= wb_flags;

         // -- EX -> WB (always advances; EX bubble propagates as wb_valid=0) --
         wb_valid  <= ex_valid;
         wb_rd     <= ex_rd;
         wb_value  <= ex_res.result;
         wb_flags  <= ex_res.flags;
         wb_wreg   <= ex_wreg;
         wb_wflags <= ex_wflags;

         // -- ID -> EX and IF -> ID (frozen on a stall) --
         if (raw_stall) begin
            ex_valid <= 1'b0;   // insert a bubble into EX
            // hold ID (id_valid/id_op) and pc — refetch/re-decode next cycle
         end else begin
            ex_valid  <= d_is_alu;
            ex_a      <= rf[d_rs1];
            ex_b      <= rf[d_rs2];
            ex_aluop  <= d_aluop;
            ex_rd     <= d_rd;
            ex_cin    <= flags.carry;
            ex_wreg   <= d_wreg;
            ex_wflags <= d_wflags;

            id_valid  <= 1'b1;
            id_op     <= imem_data;
            pc        <= pc + 32'd4;
         end
      end
   end

endmodule

//===========================================================================
// pipeline_core — Phase B in-order pipeline (feat/pipeline). See PIPELINE_IMPL.md.
//
// M1: IF/ID/EX/WB Class-1 ALU, interlock-only.
// M2: + MEM stage (IF/ID/EX/MEM/WB) with 64-bit indexed loads/stores and the
//     load-use interlock. Still interlock-only (no forwarding): a RAW dependency
//     stalls ID until the producer leaves WB; because a load's result isn't
//     committed until its (now one-later) WB, load-use is covered by the same
//     scoreboard. Forwarding (EX->EX 2-input) arrives in M6.
//
// Instruction subset (ISA v2, LEN=01, 1 word):
//   Class 1 ALU rd = rs1 OP rs2 (aluop=op[25:22], F=op[21]).
//   Class 6 load  rd = mem[rs1 + rs2]  (M2 EA model = rs1+rs2).
//   Class 7 store mem[rs1 + rs2] = rd  (store data from rd; rd is a SOURCE here).
//   rd=op[11:8], rs1=op[7:4], rs2=op[3:0]. op==0 is a NOP/bubble.
//===========================================================================
module pipeline_core
   import klauss_pkg::*;
(
   input  logic        clk,
   input  logic        rst,
   output logic [31:0] imem_addr,
   input  logic [31:0] imem_data,
   // data memory (64-bit, word addressed by dmem_addr[.:3]; backed by the tb)
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
   logic [31:0] id_op;

   // EX bundle
   logic        ex_valid, ex_is_load, ex_is_store, ex_wreg, ex_wflags;
   logic [3:0]  ex_aluop, ex_rd;
   logic [63:0] ex_a, ex_b, ex_stdata;
   logic        ex_cin;

   // MEM bundle
   logic        mem_valid, mem_is_load, mem_is_store, mem_wreg, mem_wflags;
   logic [3:0]  mem_rd;
   logic [63:0] mem_alu, mem_addr_r, mem_stdata;
   flags_t      mem_flags;

   // WB bundle
   logic        wb_valid, wb_wreg, wb_wflags;
   logic [3:0]  wb_rd;
   logic [63:0] wb_value;
   flags_t      wb_flags;

   // ---- ID decode ----
   wire [3:0] d_class = id_op[29:26];
   wire [3:0] d_aluop = id_op[25:22];
   wire [3:0] d_rd    = id_op[11:8];
   wire [3:0] d_rs1   = id_op[7:4];
   wire [3:0] d_rs2   = id_op[3:0];
   wire       d_is_alu   = id_valid && (id_op != 32'h0) && (d_class == 4'd1);
   wire       d_is_load  = id_valid && (d_class == 4'd6);
   wire       d_is_store = id_valid && (d_class == 4'd7);
   wire       d_valid_op = d_is_alu || d_is_load || d_is_store;
   wire       d_wreg     = d_is_alu || d_is_load;
   wire       d_wflags   = d_is_alu && (d_aluop <= 4'd3);

   // ---- Interlock (no forwarding): a source reg is busy while it is the pending
   // destination of an instruction in EX / MEM / WB. Stores read rd as data, so
   // rd is a source for stores. ----
   // Inlined (not a function reading module signals — that breaks continuous-
   // assign sensitivity). A source reg is busy if it is a pending destination in
   // EX/MEM/WB. Stores read rd as data, so rd is a source for stores.
   wire b_ex  = ex_valid  && ex_wreg;
   wire b_mem = mem_valid && mem_wreg;
   wire b_wb  = wb_valid  && wb_wreg;
   wire hit1 = (b_ex && ex_rd == d_rs1) || (b_mem && mem_rd == d_rs1) || (b_wb && wb_rd == d_rs1);
   wire hit2 = (b_ex && ex_rd == d_rs2) || (b_mem && mem_rd == d_rs2) || (b_wb && wb_rd == d_rs2);
   wire hitd = (b_ex && ex_rd == d_rd ) || (b_mem && mem_rd == d_rd ) || (b_wb && wb_rd == d_rd );
   wire raw_stall = d_valid_op && (hit1 || hit2 || (d_is_store && hitd));

   assign imem_addr = pc;
   genvar gi;
   generate for (gi = 0; gi < 16; gi++) assign dbg_r[gi] = rf[gi]; endgenerate

   // EX combinational
   alu_res_t    ex_res;
   assign ex_res = f_alu_ex(ex_a, ex_b, ex_aluop, ex_cin);
   wire [63:0]  ex_eaddr = ex_a + ex_b;   // M2 effective-address model

   // MEM: drive the data port
   assign dmem_addr  = mem_addr_r[31:0];
   assign dmem_wdata = mem_stdata;
   assign dmem_we    = mem_valid && mem_is_store;
   wire [63:0] mem_value = mem_is_load ? dmem_rdata : mem_alu;

   always_ff @(posedge clk) begin
      if (rst) begin
         pc <= 0; id_valid <= 0; ex_valid <= 0; mem_valid <= 0; wb_valid <= 0;
         flags <= '0;
      end else begin
         // WB commit
         if (wb_valid && wb_wreg)   rf[wb_rd] <= wb_value;
         if (wb_valid && wb_wflags) flags     <= wb_flags;

         // MEM -> WB
         wb_valid  <= mem_valid;
         wb_rd     <= mem_rd;
         wb_value  <= mem_value;
         wb_flags  <= mem_flags;
         wb_wreg   <= mem_wreg;
         wb_wflags <= mem_wflags;

         // EX -> MEM
         mem_valid   <= ex_valid;
         mem_is_load <= ex_is_load;
         mem_is_store<= ex_is_store;
         mem_rd      <= ex_rd;
         mem_alu     <= ex_res.result;
         mem_addr_r  <= ex_eaddr;
         mem_stdata  <= ex_stdata;
         mem_flags   <= ex_res.flags;
         mem_wreg    <= ex_wreg;
         mem_wflags  <= ex_wflags;

         // ID -> EX / IF -> ID (frozen on stall)
         if (raw_stall) begin
            ex_valid <= 1'b0;    // bubble
         end else begin
            ex_valid    <= d_valid_op;
            ex_is_load  <= d_is_load;
            ex_is_store <= d_is_store;
            ex_a        <= rf[d_rs1];
            ex_b        <= rf[d_rs2];
            ex_stdata   <= rf[d_rd];     // store data (also harmless for others)
            ex_aluop    <= d_aluop;
            ex_rd       <= d_rd;
            ex_cin      <= flags.carry;
            ex_wreg     <= d_wreg;
            ex_wflags   <= d_wflags;

            id_valid <= 1'b1;
            id_op    <= imem_data;
            pc       <= pc + 32'd4;
         end
      end
   end

endmodule

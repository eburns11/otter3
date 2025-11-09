`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 03/02/2023 04:17:51 PM
// Module Name: OTTER
//////////////////////////////////////////////////////////////////////////////////

typedef enum logic [6:0] {
    LUI      = 7'b0110111,
    AUIPC    = 7'b0010111,
    JAL      = 7'b1101111,
    JALR     = 7'b1100111,
    BRANCH   = 7'b1100011,
    LOAD     = 7'b0000011,
    STORE    = 7'b0100011,
    OP_IMM   = 7'b0010011,
    OP       = 7'b0110011,
    SYSTEM   = 7'b1110011
} opcode_t;

typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic [31:0] rs1_data;
    logic [31:0] rs2_data;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic flush;
    logic [1:0] rf_wr_sel;
    logic [2:0] pc_src;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] ir;
    logic [31:0] pc;
    logic [31:0] pc_inc;
    logic ALU_src_a;
    logic [1:0] ALU_src_b;
    logic [31:0] immediate;
    logic [31:0] alu_result;
    logic [31:0] mem_rdata; //data read from mem
} instr_t;


module OTTER(
    input logic RST,
    input logic [31:0] IOBUS_IN,
    input logic CLK,
    output logic IOBUS_WR,
    output logic [31:0] IOBUS_OUT,
    output logic [31:0] IOBUS_ADDR
    );

    instr_t if_pipe_reg; // P1
    instr_t de_pipe_reg; // P2
    instr_t ex_pipe_reg; // P3
    instr_t mem_pipe_reg; // P4

    initial begin
        if_pipe_reg <= '0; // P1
        de_pipe_reg <= '0; // P2
        ex_pipe_reg <= '0; // P3
        mem_pipe_reg <= '0; // P4
    end

    logic [31:0] wb_data;
    logic mem_rden2;
    logic mem_rden1;
    logic pc_rst;
    logic [31:0] ir;
    logic [2:0] pc_source;
    logic [31:0] pc_out, pc_out_inc, jalr, branch, jal;
    logic [31:0] rs1;
    logic [4:0] rd_addr, rs1_addr, rs2_addr;
    logic [2:0] funct;
    logic [6:0] opcode;
    logic ir30;
    logic [1:0] rf_wr_sel;
    //logic [4:0] reg_wa;
    logic alu_src_a;
    logic [1:0] alu_src_b;
    logic [3:0] alu_fun;
    logic [31:0] srcA, srcB;
    logic br_eq, br_lt, br_ltu;
    logic [31:0] Utype, Itype, Stype, Btype, Jtype;
    logic stall;
    logic [31:0] dout2;

    assign mem_rden2 = 1; //always read cus mux? or turn it off if not accessing? ... OK for now. 
    
    //NOTE ABOUT METHODOLOGY FOR CREATING TOP-LEVEL MODULE:
    //I decided to look at the OTTER diagram and create logic (connecting wires)
    //from left to right. This way, I was able to methodically move through the diagram,
    //connecting PC to Memory, then Memory to Reg File, Reg File Mux, and Immediate 
    //Generator, then connecting ALU, Branch Address Generator, then Branch Condition Generator,
    //and finally connecting the Control Unit consisting of the FSM and Decoder. It made
    //the process simpler, and allowed for me to create a "flow" in the SystemVerilog code.
    //I did my best to keep the name of my lgoic consistent to the names that are in the OTTER
    //diagram, and made all interconnecting wires lowercase so as to not confuse them with I/O.
            //this was already here right? delete it?

///////////////////////////////////////////////////
// Instruction Fetch

    //Create logic for PC; connecting wires to Memory module and RegFile Mux

    assign mem_rden1 = 1; //always read an instruction?!?
    assign pc_rst = RST;

    always_ff @(posedge CLK) begin
        if (RST) begin
            if_pipe_reg <= 0;
        end
        else begin
            if (!stall) begin
                if_pipe_reg.pc <= pc_out;
                if_pipe_reg.pc_inc <= pc_out_inc;  //store the pc
                if_pipe_reg.ir <= ir;
                // checking pc_source global here since it is assigned to de on the 
                // same negedge 
                if_pipe_reg.flush <= pc_source != 3'b000;
            end
        end
    end
    
    //Instantiate the PC and connect relevant I/O
    PC OTTER_PC(.CLK(CLK), .RST(pc_rst), .PC_WRITE(!stall), .PC_SOURCE(pc_source),
        .JALR(jalr), .JAL(jal), .BRANCH(branch), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc_out), .PC_OUT_INC(pc_out_inc));


///////////////////////////////////////////////////
// Instruction Decode


//Create logic for FSM and Decoder
    
    assign ir30 = if_pipe_reg.ir[30];
    assign opcode = if_pipe_reg.ir[6:0];
    assign funct = if_pipe_reg.ir[14:12];

    assign rd_addr = if_pipe_reg.ir[11:7];
    assign rs1_addr = if_pipe_reg.ir[19:15];
    assign rs2_addr = if_pipe_reg.ir[24:20];

    logic[2:0] pc_source_unstable; // can be modified by flushed instructions

    //assign reg_adr1 = if_pipe_reg.ir[19:15];
    //assign reg_adr2 = if_pipe_reg.ir[24:20];
    //assign reg_wa = mem_pipe_reg.ir[11:7];
    //assign imgen_ir = if_pipe_reg.ir[31:7];

    always_ff @(posedge CLK) begin
        if (RST) begin
            de_pipe_reg <= 0;
        end
        else begin
            if (!stall) begin
                de_pipe_reg <= if_pipe_reg;
                de_pipe_reg.flush <= (pc_source != 3'b000) | if_pipe_reg.flush;
                de_pipe_reg.opcode <= opcode_t'(opcode);  //store the opcode
                de_pipe_reg.rs1_data <= rs1;  //store register 1 data
                de_pipe_reg.rs2_data <= IOBUS_OUT;  // IOBUS_OUT used for rs2
                de_pipe_reg.rd_addr <= rd_addr;  //store dest register addr
                de_pipe_reg.rs1_addr <= rs1_addr;  //store reg 1 addr
                de_pipe_reg.rs2_addr <= rs2_addr;  //store reg 2 addr
                de_pipe_reg.alu_fun <= alu_fun;  //store alu fun
                de_pipe_reg.ALU_src_a <= alu_src_a;  //store alu source a
                de_pipe_reg.ALU_src_b <= alu_src_b;  //store alu source b
                de_pipe_reg.rf_wr_sel <= rf_wr_sel;  //store the reg file write source
                de_pipe_reg.mem_type <= if_pipe_reg.ir[14:12];
                de_pipe_reg.pc_src <= pc_source_unstable;

                case(opcode_t'(opcode))  //store the immediate value
                    LUI : begin 
                        de_pipe_reg.immediate <= Utype;
                    end
                    AUIPC : begin
                        de_pipe_reg.immediate <= Utype;
                    end
                    JAL : begin
                        de_pipe_reg.immediate <= Jtype;
                    end
                    JALR : begin
                        de_pipe_reg.immediate <= Itype;
                    end
                    BRANCH : begin
                        de_pipe_reg.immediate <= Btype;
                    end
                    LOAD : begin
                        de_pipe_reg.immediate <= Itype;
                    end
                    STORE : begin
                        de_pipe_reg.immediate <= Stype;
                    end
                    OP_IMM : begin
                        de_pipe_reg.immediate <= Itype;
                    end
                    OP : begin
                        de_pipe_reg.immediate <= 0;
                    end
                    default : begin
                        de_pipe_reg.immediate <= 0;
                    end
                endcase
            end
        end
    end

    //Create logic for Branch Condition Generator
        
    
    //Instantiate Branch Condition Generator, connect all 
    //relevant I/O
    BCG OTTER_BCG(.RS1(rs1), .RS2(IOBUS_OUT), .BR_EQ(br_eq), .BR_LT(br_lt), .BR_LTU(br_ltu));
    
    //Instantiate Decoder, connect all relevant I/O
    CU_DCDR OTTER_DCDR(.IR_30(ir30), .IR_OPCODE(opcode), .IR_FUNCT(funct), .BR_EQ(br_eq), .BR_LT(br_lt),
     .BR_LTU(br_ltu), .ALU_FUN(alu_fun), .ALU_SRCA(alu_src_a), .ALU_SRCB(alu_src_b), .PC_SOURCE(pc_source_unstable),
      .RF_WR_SEL(rf_wr_sel));  //needs to be modified for forwarding only I think
      

//Create logic for the RegFile, Immediate Generator, Branch Addresss 
    //Generator, and ALU MUXes     
    
    //Instantiate RegFile, connect all relevant I/O      //doesn't need to stall because it is changed from mem stage, not anything being stalled
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(mem_pipe_reg.regWrite), .ADR1(if_pipe_reg.ir[19:15]), .ADR2(if_pipe_reg.ir[24:20]), .WA(mem_pipe_reg.rd_addr), 
        .WD(wb_data), .RS1(rs1), .RS2(IOBUS_OUT));

    //Create logic for Immediate Generator outputs and BAG and ALU MUX inputs    
 
    
    //Instantiate Immediate Generator, connect all relevant I/O
    ImmediateGenerator OTTER_IMGEN(.IR(if_pipe_reg.ir[31:7]), .U_TYPE(Utype), .I_TYPE(Itype), .S_TYPE(Stype),
        .B_TYPE(Btype), .J_TYPE(Jtype));



//////////////////////////////////////////////////
// Execute


//Create logic for ALU

    logic [31:0] rs1_protected;
    logic [31:0] rs2_protected;
    logic [1:0] ex_det_fwd;
    logic [1:0] mem_det_fwd;

    logic regWrite;

    always_comb begin
        case(opcode_t'(de_pipe_reg.opcode))  //store the immediate value
            LUI : begin 
                regWrite <= !de_pipe_reg.flush;
            end
            AUIPC : begin
                regWrite <= !de_pipe_reg.flush;
            end
            JAL : begin
                regWrite <= !de_pipe_reg.flush;
            end
            JALR : begin
                regWrite <= !de_pipe_reg.flush;
            end
            BRANCH : begin
                regWrite <= 0;
            end
            LOAD : begin
                regWrite <= !de_pipe_reg.flush;
            end
            STORE : begin
                regWrite <= 0;
            end
            OP_IMM : begin
                regWrite <= !de_pipe_reg.flush;
            end
            OP : begin
                regWrite <= !de_pipe_reg.flush;
            end
            default : begin
                regWrite <= 0;
            end
        endcase
    end

    always_ff @(posedge CLK) begin
        if (RST || stall) begin
            ex_pipe_reg <= 0;
        end
        else if (!stall) begin
            ex_pipe_reg <= de_pipe_reg;
            ex_pipe_reg.alu_result <= IOBUS_ADDR; // output from alu
            if (de_pipe_reg.opcode == STORE) begin
                ex_pipe_reg.memWrite <= !de_pipe_reg.flush;
            end
            else begin
                ex_pipe_reg.memWrite <= 0;
            end
            ex_pipe_reg.regWrite <= regWrite;
        end
    end

    RAWDetector EX_DETECTOR(.DEST_REG(ex_pipe_reg.rd_addr), .READ_REG_1(de_pipe_reg.rs1_addr), .READ_REG_2(de_pipe_reg.rs2_addr), .RF_WE(ex_pipe_reg.regWrite), .FWD(ex_det_fwd));
    RAWDetector MEM_DETECTOR(.DEST_REG(mem_pipe_reg.rd_addr), .READ_REG_1(de_pipe_reg.rs1_addr), .READ_REG_2(de_pipe_reg.rs2_addr), .RF_WE(mem_pipe_reg.regWrite), .FWD(mem_det_fwd));

    //rs1_protected mux
    always_comb begin
        if (ex_det_fwd[0]) begin  //if raw to next intruction, forward from ex stage
            rs1_protected = ex_pipe_reg.alu_result;  //rs1 becomes alu output
        end
        else if (mem_det_fwd[0]) begin  //if raw to next next instruction, forward from mem stage
            if (mem_pipe_reg.opcode == LOAD) begin  //if it was a load, pull from memory
                rs1_protected = mem_pipe_reg.mem_rdata;
            end
            else begin  //else use alu result
                rs1_protected = mem_pipe_reg.alu_result;
            end
        end
        else begin  //else normal behavior, no forward
            rs1_protected = de_pipe_reg.rs1_data;
        end
    end

    //rs2_protected mux
    always_comb begin
        if (ex_det_fwd[1]) begin
            rs2_protected = ex_pipe_reg.alu_result;
        end
        else if (mem_det_fwd[1]) begin
            if (mem_pipe_reg.opcode == LOAD) begin
                rs2_protected = mem_pipe_reg.mem_rdata;
            end
            else begin
                rs2_protected = mem_pipe_reg.alu_result;
            end
        end
        else begin
            rs2_protected = de_pipe_reg.rs2_data;
        end
    end

    always_ff @(posedge CLK) begin
        if ((ex_det_fwd != 2'b00) && (ex_pipe_reg.opcode == LOAD) && ex_pipe_reg.regWrite && !ex_pipe_reg.flush) begin  //if raw from ex stage and it is a load, stall the pipeline
            stall = 1'b1;
        end
        else begin
            stall = 1'b0;
        end
    end
    
    //Instantiate ALU two-to-one Mux, ALU four-to-one MUX,
    //and ALU; connect all relevant I/O
    TwoMux OTTER_ALU_MUXA(.ALU_SRC_A(de_pipe_reg.ALU_src_a), .RS1(rs1_protected), .U_TYPE(de_pipe_reg.immediate), .SRC_A(srcA));
    FourMux OTTER_ALU_MUXB(.SEL(de_pipe_reg.ALU_src_b), .ZERO(rs2_protected), .ONE(de_pipe_reg.immediate), .TWO(de_pipe_reg.immediate), .THREE(de_pipe_reg.pc), .OUT(srcB));
    ALU OTTER_ALU(.SRC_A(srcA), .SRC_B(srcB), .ALU_FUN(de_pipe_reg.alu_fun), .RESULT(IOBUS_ADDR));

    //Instantiate Branch Address Generator, connect all relevant I/O    
    //BAG OTTER_BAG(.RS1(de_pipe_reg.rs1_data), .I_TYPE(de_pipe_reg.immediate), .J_TYPE(de_pipe_reg.immediate), .B_TYPE(de_pipe_reg.immediate), .FROM_PC(de_pipe_reg.pc_inc),
    //     .JAL(jal), .JALR(jalr), .BRANCH(branch));

    //ImmediateGenerator OTTER_IMGEN(.IR(if_pipe_reg.ir[31:7]), .U_TYPE(Utype), .I_TYPE(Itype), .S_TYPE(Stype),
    //    .B_TYPE(Btype), .J_TYPE(Jtype));
    //getting data late before

    always_comb begin
        if (!de_pipe_reg.flush) begin
            pc_source = de_pipe_reg.pc_src;
        end else begin
            pc_source = '0;
        end
    end

    BAG OTTER_BAG(.RS1(rs1_protected), .I_TYPE(de_pipe_reg.immediate), .J_TYPE(de_pipe_reg.immediate), .B_TYPE(de_pipe_reg.immediate), .FROM_PC(de_pipe_reg.pc),
         .JAL(jal), .JALR(jalr), .BRANCH(branch));


//////////////////////////////////////////////////
// Memory



//Create logic for Memory module; conecting wires to RegFile
    //Immediate Generator, and RegFile Mux    
    logic [13:0] addr1;
    assign addr1 = pc_out[15:2];

    logic sign;
    assign sign = ex_pipe_reg.mem_type[2];
    logic [1:0] size;
    assign size = ex_pipe_reg.mem_type[1:0];
    
    // the Memory module and connect relevant I/O    
    Memory OTTER_MEMORY(.MEM_CLK(CLK), .MEM_RDEN1(mem_rden1), .MEM_RDEN2(mem_rden2), 
        .MEM_WE2(ex_pipe_reg.memWrite && !ex_pipe_reg.flush), .MEM_ADDR1(addr1), .MEM_ADDR2(ex_pipe_reg.alu_result), .MEM_DIN2(ex_pipe_reg.rs2_data), .MEM_SIZE(size),
         .MEM_SIGN(sign), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR), .MEM_DOUT1(ir), .MEM_DOUT2(dout2));

    always_ff @(posedge CLK) begin
        if (RST) begin
            mem_pipe_reg <= 0;
        end
        else begin
            mem_pipe_reg <= ex_pipe_reg;
            mem_pipe_reg.mem_rdata <= dout2;
        end
    end

//////////////////////////////////////////////////
// Writeback


    //wb mux
    always_comb begin 
        case (mem_pipe_reg.rf_wr_sel)
            0: wb_data = mem_pipe_reg.pc_inc; // JAL: PC+4
            1: wb_data = 0;
            2: wb_data = mem_pipe_reg.mem_rdata; //LOAD
            3: wb_data = mem_pipe_reg.alu_result; //ALU
            default: wb_data = 0;
        endcase
    end
    
//Instantiate RegFile Mux, connect all relevant I/O
//    FourMux OTTER_REG_MUX(.SEL(rf_wr_sel), .ZERO(pc_out_inc), .ONE(32'b0), .TWO(dout2), .THREE(IOBUS_ADDR),
//        .OUT(wd));
    
    
    
endmodule
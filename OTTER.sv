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
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] ir;
    logic [31:0] pc_inc;
    logic [2:0] pc_src;
    logic [31:0] ALU_Op_A;
    logic [31:0] ALU_Op_B;
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



    logic [31:0] wb_data;
    logic reg_wr;
    logic mem_we2;
    logic mem_rden2;

    assign reg_wr = mem_pipe_reg.regWrite;
    assign mem_we2 = 0; //if store
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
    logic pc_rst, pc_write;   //wired
    logic [31:0] ir;
    logic [2:0] pc_source;
    logic [31:0] pc_out, pc_out_inc, jalr, branch, jal;   //pc_out wired


    assign pc_write = 1; //always go to the next instruction
    assign mem_rden1 = 1; //always read an instruction?!?
    assign pc_rst = RST;

    always_ff @(posedge CLK) begin
        if_pipe_reg.ir <= ir;  //always store the next instruction into the IF pc register
        if_pipe_reg.pc_inc <= pc_out_inc;  //store the pc
    end
    
    //Instantiate the PC and connect relevant I/O
    PC OTTER_PC(.CLK(CLK), .RST(pc_rst), .PC_WRITE(pc_write), .PC_SOURCE(pc_source),
        .JALR(jalr), .JAL(jal), .BRANCH(branch), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc_out), .PC_OUT_INC(pc_out_inc));


///////////////////////////////////////////////////
// Instruction Decode


//Create logic for FSM and Decoder
    logic ir30;
    assign ir30 = if_pipe_reg.ir[30];
    logic [6:0] opcode;
    assign opcode = if_pipe_reg.ir[6:0];
    logic [2:0] funct;
    assign funct = if_pipe_reg.ir[14:12];


    always_ff @(posedge CLK) begin
        de_pipe_reg.pc_inc <= if_pipe_reg.pc_inc;  //move the pc along
        de_pipe_reg.opcode <= opcode_t'(opcode);  //store the opcode
        de_pipe_reg.rs1_data <= rs1;  //store register 1 data
        de_pipe_reg.rs2_data <= IOBUS_OUT;  // IOBUS_OUT used for rs2
        de_pipe_reg.rd_addr <= reg_wa;  //store dest register addr
        de_pipe_reg.rs1_addr <= reg_adr1;  //store reg 1 addr
        de_pipe_reg.rs2_addr <= reg_adr2;  //store reg 2 addr
        de_pipe_reg.alu_fun <= alu_fun;  //store alu fun
        de_pipe_reg.ALU_Op_A <= alu_src_a;  //store alu source a
        de_pipe_reg.ALU_Op_B <= alu_src_b;  //store alu source b
        de_pipe_reg.pc_src <= pc_source;  //store the pc source
        de_pipe_reg.rf_wr_sel <= rf_wr_sel;  //store the reg file write source

        case(opcode_t'(opcode))  //store the immediate value
            LUI : de_pipe_reg.immediate <= Utype;
            AUIPC : de_pipe_reg.immediate <= Utype;
            JAL : de_pipe_reg.immediate <= Jtype;
            JALR : de_pipe_reg.immediate <= Jtype;
            BRANCH : de_pipe_reg.immediate <= Btype;
            LOAD : de_pipe_reg.immediate <= Itype;
            STORE : de_pipe_reg.immediate <= Stype;
            OP_IMM : de_pipe_reg.immediate <= Itype;
            default : de_pipe_reg.immediate <= 0;
        endcase
    end

    //Create logic for Branch Condition Generator
    logic br_eq, br_lt, br_ltu;    
    
    //Instantiate Branch Condition Generator, connect all 
    //relevant I/O
    BCG OTTER_BCG(.RS1(rs1), .RS2(IOBUS_OUT), .BR_EQ(br_eq), .BR_LT(br_lt), .BR_LTU(br_ltu));
    
    //Instantiate Decoder, connect all relevant I/O
    CU_DCDR OTTER_DCDR(.IR_30(ir30), .IR_OPCODE(opcode), .IR_FUNCT(funct), .BR_EQ(br_eq), .BR_LT(br_lt),
     .BR_LTU(br_ltu), .ALU_FUN(alu_fun), .ALU_SRCA(alu_src_a), .ALU_SRCB(alu_src_b), .PC_SOURCE(pc_source),
      .RF_WR_SEL(rf_wr_sel));  //needs to be modified for forwarding only I think

//Create logic for the RegFile, Immediate Generator, Branch Addresss 
    //Generator, and ALU MUXes     
    logic reg_wr;
    logic [1:0] rf_wr_sel;
    logic [4:0] reg_adr1;
    assign reg_adr1 = if_pipe_reg.ir[19:15];
    logic [4:0] reg_adr2;
    assign reg_adr2 = if_pipe_reg.ir[24:20];
    logic [4:0] reg_wa;
    assign reg_wa = if_pipe_reg.ir[11:7];
    logic [24:0] imgen_ir;
    assign imgen_ir = if_pipe_reg.ir[31:7];
    logic [31:0] wd, rs1;
    
    //Instantiate RegFile, connect all relevant I/O    
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(reg_wr), .ADR1(reg_adr1), .ADR2(reg_adr2), .WA(reg_wa), 
        .WD(wb_data), .RS1(rs1), .RS2(IOBUS_OUT));

    //Create logic for Immediate Generator outputs and BAG and ALU MUX inputs    
    logic [31:0] Utype, Itype, Stype, Btype, Jtype;
    
    //Instantiate Immediate Generator, connect all relevant I/O
    ImmediateGenerator OTTER_IMGEN(.IR(imgen_ir), .U_TYPE(Utype), .I_TYPE(Itype), .S_TYPE(Stype),
        .B_TYPE(Btype), .J_TYPE(Jtype));


//////////////////////////////////////////////////
// Execute


//Create logic for ALU
    logic alu_src_a;
    logic [1:0] alu_src_b;
    logic [3:0] alu_fun;
    logic [31:0] srcA, srcB;


    always_ff @(posedge CLK) begin
        ex_pipe_reg<= de_pipe_reg;
        ex_pipe_reg.alu_result <= IOBUS_ADDR; // output from alu
    end
    
    //Instantiate ALU two-to-one Mux, ALU four-to-one MUX,
    //and ALU; connect all relevant I/O
    TwoMux OTTER_ALU_MUXA(.ALU_SRC_A(de_pipe_reg.ALU_Op_A), .RS1(de_pipe_reg.rs1_data), .U_TYPE(de_pipe_reg.immediate), .SRC_A(srcA));
    FourMux OTTER_ALU_MUXB(.SEL(de_pipe_reg.ALU_Op_B), .ZERO(IOBUS_OUT), .ONE(de_pipe_reg.immediate), .TWO(de_pipe_reg.immediate), .THREE(de_pipe_reg.pc_inc), .OUT(srcB));
    ALU OTTER_ALU(.SRC_A(srcA), .SRC_B(srcB), .ALU_FUN(de_pipe_reg.alu_fun), .RESULT(IOBUS_ADDR));


//Instantiate Branch Address Generator, connect all relevant I/O    
    BAG OTTER_BAG(.RS1(de_pipe_reg.rs1_data), .I_TYPE(de_pipe_reg.immediate), .J_TYPE(de_pipe_reg.immediate), .B_TYPE(de_pipe_reg.immediate), .FROM_PC(de_pipe_reg.pc_inc),
         .JAL(jal), .JALR(jalr), .BRANCH(branch));


//////////////////////////////////////////////////
// Memory



//Create logic for Memory module; conecting wires to RegFile
    //Immediate Generator, and RegFile Mux    
    logic [13:0] addr1;
    assign addr1 = pc_out[15:2];
    logic mem_rden1, mem_rden2, mem_we2;
    logic [31:0] dout2;
    logic sign;
    assign sign = ir[14];
    logic [1:0] size;
    assign size = ir[13:12];
    
    //Instantiate the Memory module and connect relevant I/O    
    Memory OTTER_MEMORY(.MEM_CLK(CLK), .MEM_RDEN1(mem_rden1), .MEM_RDEN2(mem_rden2), 
        .MEM_WE2(mem_we2), .MEM_ADDR1(addr1), .MEM_ADDR2(IOBUS_ADDR), .MEM_DIN2(IOBUS_OUT), .MEM_SIZE(size),
         .MEM_SIGN(sign), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR), .MEM_DOUT1(ir), .MEM_DOUT2(dout2));

    always_ff @(posedge CLK) begin 
        mem_pipe_reg <= ex_pipe_reg;
        mem_pipe_reg.mem_rdata <= dout2;
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
    FourMux OTTER_REG_MUX(.SEL(rf_wr_sel), .ZERO(pc_out_inc), .ONE(32'b0), .TWO(dout2), .THREE(IOBUS_ADDR),
        .OUT(wd));
    
    
    
endmodule

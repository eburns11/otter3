`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: J. Callenes, P. Hummel
//
// Create Date: 01/27/2019 08:37:11 AM
// Module Name: OTTER_mem
// Project Name: Memory for OTTER RV32I RISC-V
// Tool Versions: Xilinx Vivado 2019.2
// Description: 64k Memory, dual access read single access write. Designed to
//              purposely utilize BRAM which requires synchronous reads and write
//              ADDR1 used for Program Memory Instruction. Word addressable so it
//              must be adapted from byte addresses in connection from PC
//              ADDR2 used for data access, both internal and external memory
//              mapped IO. ADDR2 is byte addressable.
//              RDEN1 and EDEN2 are read enables for ADDR1 and ADDR2. These are
//              needed due to synchronous reading
//              MEM_SIZE used to specify reads as byte (0), half (1), or word (2)
//              MEM_SIGN used to specify unsigned (1) vs signed (0) extension
//              IO_IN is data from external IO and synchronously buffered
//
// Memory OTTER_MEMORY (
//    .MEM_CLK   (),
//    .MEM_RDEN1 (),
//    .MEM_RDEN2 (),
//    .MEM_WE2   (),
//    .MEM_ADDR1 (),
//    .MEM_ADDR2 (),
//    .MEM_DIN2  (),
//    .MEM_SIZE  (),
//    .MEM_SIGN  (),
//    .IO_IN     (),
//    .IO_WR     (),
//    .MEM_DOUT1 (),
//    .MEM_DOUT2 ()  );
//
// Revision:
// Revision 0.01 - Original by J. Callenes
// Revision 1.02 - Rewrite to simplify logic by P. Hummel
// Revision 1.03 - changed signal names, added instantiation template
// Revision 1.04 - added defualt to write case statement
// Revision 1.05 - changed MEM_WD to MEM_DIN2, changed default to save nothing
// Revision 1.06 - removed typo in instantiation template
// Revision 1.07 - remove unused wordAddr1 signal
//
//////////////////////////////////////////////////////////////////////////////////
                                                                                                                            
module Memory (
  input MEM_CLK,
  input MEM_WE2,          // write enable.
  input [31:0] MEM_ADDR1, // Instruction Memory word Addr (Connect to PC[15:2])
  input [31:0] MEM_ADDR2, // Data Memory Addr
  input [31:0] MEM_DIN2,  // Data to save
  input [1:0] MEM_SIZE,   // 0-Byte, 1-Half, 2-Word
  input MEM_SIGN,         // 1-unsigned 0-signed
  input [31:0] IO_IN,     // Data from IO
  input RST,
  output logic IO_WR,     // IO 1-write 0-read
  output logic [31:0] MEM_DOUT1,  // Instruction
  output logic [31:0] MEM_DOUT2, // Data
  output logic PC_STALL);
  
  logic [31:0] memReadWord, ioBuffer, memReadSized;
  logic [1:0] byteOffset;
  assign byteOffset = MEM_ADDR2[1:0];     // byte offset of memory address
  logic weAddrValid;      // active when saving (WE) to valid memory address

  logic [31:0] icache_words [8];
  imem INSTR_MEMORY(.a(MEM_ADDR1), .words(icache_words));
  logic icache_hit, icache_miss, icache_update, icache_pc_stall;

  CacheFSM IMEM_FSM(.hit(icache_hit), .miss(icache_miss), .CLK(MEM_CLK), .RST(RST), .update(icache_update), .pc_stall(icache_pc_stall));
  DM_Cache ICACHE(.PC(MEM_ADDR1), .CLK(MEM_CLK), .update(icache_update), .words(icache_words),
                  .rd(MEM_DOUT1), .hit(icache_hit), .miss(icache_miss));

  logic [31:0] dcache_words [4];
  logic [31:0] dcache_wb_words [4];
  logic dcache_we;
  dmem DATA_MEMORY(.CLK(MEM_CLK), .a(MEM_ADDR2), .we(dcache_we), .wb_words(dcache_wb_words), .words(dcache_words));
  logic dcache_hit, dcache_miss, dcache_update, dcache_pc_stall;
  logic [31:0] dcache_out;

  //we need to find a stall method other than pc, this won't work. Do we need to stall the entire pipeline?
  CacheFSM DMEM_FSM(.hit(dcache_hit), .miss(dcache_miss), .CLK(MEM_CLK), .RST(RST), .update(dcache_update), .pc_stall(dcache_pc_stall));
  SA_Cache DCACHE(.CLK(MEM_CLK), .update(dcache_update), .addr(MEM_ADDR2), .words(dcache_words), .cache_we(MEM_WE2), .mem_din(MEM_DIN2), .mem_size(MEM_SIZE), .mem_byte_offset(byteOffset),
                  .rd(dcache_out), .mem_wb(dcache_we), .wb_words(dcache_wb_words), .hit(dcache_hit), .miss(dcache_miss));
  
  assign PC_STALL = icache_pc_stall | dcache_pc_stall;

  
          
  // buffer the IO input for reading
  always_ff @(negedge MEM_CLK) begin
    if(MEM_RDEN2)
      ioBuffer <= IO_IN;
  end

  
  // BRAM requires all reads and writes to occur synchronously
  always_ff @(negedge MEM_CLK) begin
    if (MEM_RDEN2)                       // Read word from memory
      memReadWord <= memory[wordAddr2];
      
  end
      
  // Change the data word into sized bytes and sign extend
  always_comb begin
    case({MEM_SIGN,MEM_SIZE,byteOffset})
      5'b00011: memReadSized = {{24{memReadWord[31]}},memReadWord[31:24]};  // signed byte
      5'b00010: memReadSized = {{24{memReadWord[23]}},memReadWord[23:16]};
      5'b00001: memReadSized = {{24{memReadWord[15]}},memReadWord[15:8]};
      5'b00000: memReadSized = {{24{memReadWord[7]}},memReadWord[7:0]};
                                  
      5'b00110: memReadSized = {{16{memReadWord[31]}},memReadWord[31:16]};  // signed half
      5'b00101: memReadSized = {{16{memReadWord[23]}},memReadWord[23:8]};
      5'b00100: memReadSized = {{16{memReadWord[15]}},memReadWord[15:0]};
          
      5'b01000: memReadSized = memReadWord;                   // word
              
      5'b10011: memReadSized = {24'd0,memReadWord[31:24]};    // unsigned byte
      5'b10010: memReadSized = {24'd0,memReadWord[23:16]};
      5'b10001: memReadSized = {24'd0,memReadWord[15:8]};
      5'b10000: memReadSized = {24'd0,memReadWord[7:0]};
              
      5'b10110: memReadSized = {16'd0,memReadWord[31:16]};    // unsigned half
      5'b10101: memReadSized = {16'd0,memReadWord[23:8]};
      5'b10100: memReadSized = {16'd0,memReadWord[15:0]};
          
      default:  memReadSized = 32'b0;     // unsupported size, byte offset combination
    endcase
  end

  //lowk wanna just forget mmio because I dont think its an expected use case
  // Memory Mapped IO 
  always_comb begin
    if(MEM_ADDR2 >= 32'h00010000) begin  // external address range
      IO_WR = MEM_WE2;                 // IO Write
      MEM_DOUT2 = ioBuffer;            // IO read from buffer
      weAddrValid = 0;                 // address beyond memory range
    end
    else begin
      IO_WR = 0;                  // not MMIO
      MEM_DOUT2 = memReadSized;   // output sized and sign extended data
      weAddrValid = MEM_WE2;      // address in valid memory range
    end
  end
      
endmodule
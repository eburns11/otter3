`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 02/23/2023 09:39:49 AM
// Module Name: CU_DCDR
//////////////////////////////////////////////////////////////////////////////////

module PC_SRC_DCDR(
    input logic [6:0] IR_OPCODE,
    input logic [2:0] IR_FUNCT,
    input logic BR_EQ,
    input logic BR_LT,
    input logic BR_LTU,
    output logic [2:0] PC_SOURCE
    );
    
    always_comb begin
        PC_SOURCE = 3'b000;
        
        //Case statement depending on the opcode for the 
        //instruction, or the last seven bits of each instruction
        case (IR_OPCODE)
            7'b1101111: begin // JAL
                PC_SOURCE = 3'b011;
            end
            7'b1100111: begin // JALR
                PC_SOURCE = 3'b001;
            end
            7'b1100011: begin // B-Type
                //nested case statement dependent on the
                //function three bits.
                //Because there are six real branch instructions, there
                //are six pairs of if-else statements in each of six cases
                //for the branch instructions.
                case(IR_FUNCT)
                    3'b000: begin
                        if (BR_EQ == 1'b1)
                            PC_SOURCE = 3'b010;
                        else
                            PC_SOURCE = 3'b000; 
                    end
                    3'b001: begin 
                        if (BR_EQ == 1'b0)
                            PC_SOURCE = 3'b010;
                        else
                            PC_SOURCE = 3'b000; 
                    end
                    3'b100: begin 
                        if (BR_LT == 1'b1)
                            PC_SOURCE = 3'b010;
                        else
                            PC_SOURCE = 3'b000;
                    end
                    3'b101: begin 
                        if (BR_LT == 1'b0)
                            PC_SOURCE = 3'b010;
                        else
                            PC_SOURCE = 3'b000;
                    end
                    3'b110: begin 
                        if (BR_LTU == 1'b1)
                            PC_SOURCE = 3'b010;
                        else
                            PC_SOURCE = 3'b000;
                    end
                    3'b111: begin 
                        if (BR_LTU == 1'b0)
                            PC_SOURCE = 3'b010;
                        else
                            PC_SOURCE = 3'b000;
                    end
                endcase
            end
            default: begin end
        endcase
    end
    
endmodule

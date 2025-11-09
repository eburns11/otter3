`timescale 1ns / 1ps

module RAWDetector(
    input logic [4:0] DEST_REG,
    input logic [4:0] READ_REG_1,
    input logic [4:0] READ_REG_2,
    input logic RF_WE,
    output logic [1:0] FWD
);

    always_comb begin
        FWD = 2'b00;
        if (RF_WE) begin
            if ((DEST_REG == READ_REG_1) && (READ_REG_1 != 0)) begin
                FWD[0] = 1'b1;
            end
            if ((DEST_REG == READ_REG_2) && (READ_REG_2 != 0)) begin
                FWD[1] = 1'b1;
            end
        end
    end


endmodule
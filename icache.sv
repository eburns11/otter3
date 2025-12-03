module imem(
    input logic [31:0] a,
    output logic [31:0] words [8]
    );

    logic [31:0] ram[0:4079];
    logic [31:0] addr;
    assign addr = {a[31:5], 3'b000};
    initial $readmemh("../hdl/performance.mem", ram, 0, 4079);
    //changed memory so it does output 8 words
    assign words[0] = ram[addr];
    assign words[1] = ram[addr+1];
    assign words[2] = ram[addr+2];
    assign words[3] = ram[addr+3];
    assign words[4] = ram[addr+4];
    assign words[5] = ram[addr+5];
    assign words[6] = ram[addr+6];
    assign words[7] = ram[addr+7];

endmodule


module CacheFSM(input hit, input miss, output logic update);
    assign update = miss;
endmodule


module DM_Cache(
    input [31:0] PC,input CLK,
    input update,
    input logic [31:0] words [8],
    output logic [31:0] rd,
    output logic hit,
    output logic miss
    );

    parameter NUM_BLOCKS = 16;
    parameter BLOCK_SIZE = 8;
    parameter INDEX_SIZE = 4;
    parameter WORD_OFFSET_SIZE = 3;
    parameter BYTE_OFFSET = 0;
    parameter TAG_SIZE = 32 - INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET;

    logic [31:0] data[NUM_BLOCKS-1:0][BLOCK_SIZE-1:0];
    logic [TAG_SIZE-1:0] tags[NUM_BLOCKS-1:0];
    logic valid_bits[NUM_BLOCKS-1:0];
    logic [3:0] index;
    logic [TAG_SIZE-1:0]cache_tag, pc_tag;
    logic [2:0] pc_offset;
    initial begin
        int i;int j;
        for(i = 0; i < NUM_BLOCKS; i = i + 1) begin //initializing RAM to 0
            for(j=0; j < BLOCK_SIZE; j = j + 1)
            data[i][j] = 32'b0;
            tags[i] = 32'b0;
            valid_bits[i] = 1'b0;
        end
    end
    assign index = PC[8:5];
    assign validity = valid_bits[index];
    assign cache_tag = tags[index];
    assign pc_offset = PC[4:2];
    assign pc_tag = PC[31:9];
    assign hit = (validity && (cache_tag == pc_tag));
    assign miss = !hit;
    always_comb begin
        rd = 32'h00000013; //nop
        if(hit) rd = data[index][pc_offset];
    end
    always_ff @(negedge CLK) begin
        if(update) begin
            tags[index] <= pc_tag;
            data[index][0] <= words[0];
            data[index][1] <= words[1];
            data[index][2] <= words[2];
            data[index][3] <= words[3];
            data[index][4] <= words[4];
            data[index][5] <= words[5];
            data[index][6] <= words[6];
            data[index][7] <= words[7];
            valid_bits[index] <= 1'b1;
        end
    end
endmodule

module imem(
    input logic [31:0] a,
    output logic [31:0] w0,
    output logic [31:0] w1,
    output logic [31:0] w2,
    output logic [31:0] w3,
    output logic [31:0] w4,
    output logic [31:0] w5,
    output logic [31:0] w6,
    output logic [31:0] w7
    );

    logic [31:0] ram[0:16383];
    initial $readmemh("performance.mem", ram, 0, 4079);
    //changed memory so it does output 8 words
    assign w0 = ram[a[31:2]];
    assign w1 = ram[a[31:2]+1];
    assign w2 = ram[a[31:2]+2];
    assign w3 = ram[a[31:2]+3];
    assign w4 = ram[a[31:2]+4];
    assign w5 = ram[a[31:2]+5];
    assign w6 = ram[a[31:2]+6];
    assign w7 = ram[a[31:2]+7];

endmodule


module CacheFSM(input hit, input miss, input CLK, input RST, output logic update, output logic pc_stall);
    typedef enum{
        ST_READ_CACHE,
        ST_READ_MEM
    } state_type;
    state_type PS, NS;
    always_ff @(posedge CLK) begin
    if(RST == 1)
        PS <= ST_READ_MEM;
    else
        PS <= NS;
    end
    always_comb begin
        update = 1'b1
        pc_stall = 0;
        case (PS)
        ST_READ_CACHE: begin
            update = 1'b0;
            if(hit) begin
                NS = ST_READ_CACHE;
            end
            else if(miss) begin
                pc_stall = 1'b1;
                NS = ST_READ_MEM;
            end
            else NS = ST_READ_CACHE;
            end
        ST_READ_MEM: begin
            pc_stall = 1'b1;
            NS = ST_READ_CACHE;
        end
        default: NS = ST_READ_CACHE;
        endcase
    end
endmodule


module Cache(
    input [31:0] PC,input CLK,input update,
    input logic [31:0] w0,input logic [31:0] w1,
    input logic [31:0] w2, input logic [31:0] w3,
    input logic [31:0] w4, input logic [31:0] w5,
    input logic [31:0] w6, input logic [31:0] w7,
    output logic [31:0] rd,output logic hit, output logic miss
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
            data[index][0] <= w0;
            data[index][1] <= w1;
            data[index][2] <= w2;
            data[index][3] <= w3;
            data[index][4] <= w4;
            data[index][5] <= w5;
            data[index][6] <= w6;
            data[index][7] <= w7;
            valid_bits[index] <= 1'b1;
        end
    end
endmodule

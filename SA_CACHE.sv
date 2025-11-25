module sa_dmem(
    input logic [31:0] a,
    output logic [31:0] w0,
    output logic [31:0] w1,
    output logic [31:0] w2,
    output logic [31:0] w3,
    //output logic [31:0] w4,
    //output logic [31:0] w5,
    //output logic [31:0] w6,
    //output logic [31:0] w7
    );

    logic [31:0] ram[0:4079];
    logic [31:0] addr;
    assign addr = {a[31:5], 3'b000};
    initial $readmemh("../hdl/performance.mem", ram, 0, 4079);
    //changed memory so it does output 8 words
    assign w0 = ram[addr];
    assign w1 = ram[addr+1];
    assign w2 = ram[addr+2];
    assign w3 = ram[addr+3];
    //assign w4 = ram[addr+4];
    //assign w5 = ram[addr+5];
    //assign w6 = ram[addr+6];
    //assign w7 = ram[addr+7];

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
        update = 1'b1;
        pc_stall = 1'b0;
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
            NS = ST_READ_CACHE;
        end
        default: NS = ST_READ_CACHE;
        endcase
    end
endmodule


module SA_Cache(
    input [31:0] PC,
    input CLK,
    input update,
    input logic [31:0] w0,
    input logic [31:0] w1,
    input logic [31:0] w2,
    input logic [31:0] w3,
    output logic [31:0] rd,
    output logic hit,
    output logic miss
    );


    //parameter BLOCK_SIZE = 4; // 4 words
    parameter INDEX_SIZE = 2; // 4 sets 
    parameter WORD_OFFSET_SIZE = 2; // 4 words per block
    parameter BYTE_OFFSET = 2; // output 32 bits
    parameter TAG_SIZE = 32 - INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET; 
    parameter NUM_SETS = 4;
    parameter NUM_WAYS = 4;
    parameter BLOCK_WORDS = 4;

    typedef struct packed {
        logic [31:0] block[BLOCK_WORDS];
        logic [TAG_SIZE-1:0] tag;
        logic valid;
        logic dirty;
    } cache_way_t;

    cache_way_t cache [NUM_SETS][NUM_WAYS];

    initial begin
        cache <= 0;
    end
    logic [1:0] byte_offset;
    logic [1:0] word_offset;
    logic [1:0] index;
    logic [TAG_SIZE-1:0] tag;

    assign byte_offset = PC[1:0];
    assign word_offset = PC[3:2];
    assign index = PC[5:4];
    assign tag = PC[31:6];


    logic way0_hit, way1_hit, way2_hit, way3_hit;
    logic [1:0] hit_way; // which way hit?

    assign way0_hit = cache[index][0].valid && cache[index][0].tag == tag;
    assign way1_hit = cache[index][1].valid && cache[index][1].tag == tag;
    assign way2_hit = cache[index][2].valid && cache[index][2].tag == tag;
    assign way3_hit = cache[index][3].valid && cache[index][3].tag == tag;

    assign hit = way0_hit | way1_hit | way2_hit | way3_hit;
    assign miss = !hit;

    always_comb begin
        if (way0_hit)      hit_way = 2'd0;
        else if (way1_hit) hit_way = 2'd1;
        else if (way2_hit) hit_way = 2'd2;
        else if (way3_hit) hit_way = 2'd3;
        else               hit_way = 2'd0; //dflt
    end


    always_comb begin
        rd = 32'h00000013; //nop
        //if(hit) rd= cache[].data[]//data[index][pc_offset];
        if (hit) rd = cache[index][hit_way].block[word_offset];
    end

    always_ff @(negedge CLK) begin
        if(update) begin
            // FIXME: alwats loading into way 0 (impl w/ wb)
            cache[index][0].valid <= 1;
            cache[index][0].tag <= tag;
            cache[index][0].block[0] <= w0;
            cache[index][0].block[1] <= w1;
            cache[index][0].block[2] <= w2;
            cache[index][0].block[3] <= w3;
            //cache[index][0].block[4] <= w4;
            //cache[index][0].block[5] <= w5;
            //cache[index][0].block[6] <= w6;
            //cache[index][0].block[7] <= w7;
        end
    end
endmodule

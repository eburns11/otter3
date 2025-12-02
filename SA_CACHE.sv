module dmem(
    input logic CLK,
    input logic [31:0] a,
    input logic we,
    input logic [31:0] wb_words [4],
    output logic [31:0] words [4]
    );

    logic [31:0] ram[0:4079];
    logic [31:0] addr;
    assign addr = {2'b00, a[31:4], 2'b00}; //fixed for 4 words at a time, not 8
    initial $readmemh("../hdl/performance.mem", ram, 0, 4079);

    always_ff @(negedge CLK) begin
        if (we) begin
            ram[addr] <= wb_words[0];
            ram[addr+1] <= wb_words[1];
            ram[addr+2] <= wb_words[2];
            ram[addr+3] <= wb_words[3];

            words[0] <= '0;
            words[1] <= '0;
            words[2] <= '0;
            words[3] <= '0;
        end else begin
            words[0] <= ram[addr];
            words[1] <= ram[addr+1];
            words[2] <= ram[addr+2];
            words[3] <= ram[addr+3];
        end
    end

endmodule

module SA_Cache(
    input CLK,
    input MEM_RDEN,
    input update,
    input logic [31:0] addr,
    input logic [31:0] words [4],
    input logic cache_we,
    input logic [31:0] mem_din,
    input logic [1:0] mem_size,
    input logic [1:0] mem_byte_offset,
    output logic [31:0] rd,
    output logic mem_wb,
    output logic [31:0] wb_words [4],
    output logic [31:0] wb_addr,
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

    //16 blocks x 4 words = 64 total words
    //each set is 4 blocks

    typedef struct packed{
        logic [BLOCK_WORDS-1:0][31:0] block;
        logic [TAG_SIZE-1:0] tag;
        logic valid;
        logic dirty;
    } cache_way_t;

    logic [1:0] byte_offset;
    logic [1:0] word_offset;
    logic [1:0] index;
    logic [TAG_SIZE-1:0] tag;
    
    logic way0_hit, way1_hit, way2_hit, way3_hit;
    logic [1:0] hit_way; // which way hit?

    cache_way_t cache [NUM_SETS][NUM_WAYS];

    logic queue_we[NUM_SETS];
    logic [1:0] queue_lru[NUM_SETS];
    PRIO_Q queues[NUM_SETS](.CLK(CLK), .we(queue_we), .in(hit_way), .lru(queue_lru));

    assign queue_we[0] = (hit) ? index == 0 : 0;
    assign queue_we[1] = (hit) ? index == 1 : 0;
    assign queue_we[2] = (hit) ? index == 2 : 0;
    assign queue_we[3] = (hit) ? index == 3 : 0;

    initial begin
        for (int i = 0; i < NUM_SETS; i++) begin
            for (int j = 0; j < NUM_WAYS; j++) begin
                cache[i][j] = '0;
            end
        end
    end

    assign byte_offset = addr[1:0];
    assign word_offset = addr[3:2];
    assign index = addr[5:4];
    assign tag = addr[31:6];


    assign way0_hit = cache[index][0].valid && cache[index][0].tag == tag;
    assign way1_hit = cache[index][1].valid && cache[index][1].tag == tag;
    assign way2_hit = cache[index][2].valid && cache[index][2].tag == tag;
    assign way3_hit = cache[index][3].valid && cache[index][3].tag == tag;

    assign hit = way0_hit | way1_hit | way2_hit | way3_hit;
    assign miss = !hit && MEM_RDEN;

    always_comb begin
        if (way0_hit)      hit_way = 2'd0;
        else if (way1_hit) hit_way = 2'd1;
        else if (way2_hit) hit_way = 2'd2;
        else if (way3_hit) hit_way = 2'd3;
        else               hit_way = 2'd0; //dflt
    end

    always_comb begin
        rd = '0; //blank data

        if (hit) rd = cache[index][hit_way].block[word_offset];
    end

    assign mem_wb = update && cache[index][queue_lru[index]].dirty;
    assign wb_addr = {cache[index][queue_lru[index]].tag, index, 4'b0000};
    assign wb_words[0] = cache[index][queue_lru[index]].block[0];
    assign wb_words[1] = cache[index][queue_lru[index]].block[1];
    assign wb_words[2] = cache[index][queue_lru[index]].block[2];
    assign wb_words[3] = cache[index][queue_lru[index]].block[3];

    always_ff @(negedge CLK) begin
        if(update) begin
            cache[index][queue_lru[index]].valid <= 1;
            cache[index][queue_lru[index]].dirty <= 0;
            cache[index][queue_lru[index]].tag <= tag;
            cache[index][queue_lru[index]].block[0] <= words[0];
            cache[index][queue_lru[index]].block[1] <= words[1];
            cache[index][queue_lru[index]].block[2] <= words[2];
            cache[index][queue_lru[index]].block[3] <= words[3];
        end else if (cache_we && hit) begin
            case({mem_size,mem_byte_offset})
                4'b0000: cache[index][hit_way].block[word_offset][7:0]   <= mem_din[7:0];     // sb at byte offsets
                4'b0001: cache[index][hit_way].block[word_offset][15:8]  <= mem_din[7:0];
                4'b0010: cache[index][hit_way].block[word_offset][23:16] <= mem_din[7:0];
                4'b0011: cache[index][hit_way].block[word_offset][31:24] <= mem_din[7:0];
                4'b0100: cache[index][hit_way].block[word_offset][15:0]  <= mem_din[15:0];     // sh at byte offsets
                4'b0101: cache[index][hit_way].block[word_offset][23:8]  <= mem_din[15:0];
                4'b0110: cache[index][hit_way].block[word_offset][31:16] <= mem_din[15:0];
                4'b1000: cache[index][hit_way].block[word_offset] <= mem_din;                  // sw
            endcase

            cache[index][hit_way].dirty <= 1;
        end
    end
endmodule

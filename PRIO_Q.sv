//2 bit priority queue for the SA Cache
//2 bits for 4 sets

module PRIO_Q(
    input logic CLK,
    input logic we,
    input logic [1:0] in,
    output logic [1:0] lru
);

    reg [1:0] slots [4];
    logic slots_eq [3];
    logic slots_we [4];

    assign slots_eq[0] = in == slots[0];
    assign slots_eq[1] = in == slots[1];
    assign slots_eq[2] = in == slots[2];

    assign slots_we[0] = slots_eq[0];
    assign slots_we[1] = slots_eq[0] | slots_eq[1];
    assign slots_we[2] = slots_eq[0] | slots_eq[1] | slots_eq[2];
    assign slots_we[3] = 1'b1;

    assign lru = slots[0];

    initial begin
        slots = {0,1,2,3};
    end

    always_ff @(negedge CLK) begin
        if (we) begin
            if (slots_we[3]) begin
                slots[3] <= in;
            end
            if (slots_we[2]) begin
                slots[2] <= slots[3];
            end
            if (slots_we[1]) begin
                slots[1] <= slots[2];
            end
            if (slots_we[0]) begin
                slots[0] <= slots[1];
            end
        end
    end

endmodule
module tag_array (
    input  logic        clk,
    input  logic        we,
    input  logic [5:0]  index,
    input  logic [21:0] tag_in,
    output logic [21:0] tag_out
);

    // Hardcoded Constants
    // NUM_SETS = 64
    // TAG_BITS = 22
    // INDEX_BITS = 6

    logic [21:0] mem [0:63];
    logic [21:0] tag_out_r;



    assign tag_out = tag_out_r;

    always_comb begin
        tag_out_r = mem[index];
    end

    always_ff @(posedge clk) begin
        if (we) begin
            mem[index] <= tag_in;
        end
    end

endmodule
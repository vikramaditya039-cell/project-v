module data_array (
    input  logic        clk,
    input  logic        we,
    input  logic [5:0]  index,
    input  logic [1:0]  word_sel,
    input  logic [31:0] wdata,
    output logic [31:0] rdata
);

    // Hardcoded Constants
    // NUM_SETS = 64
    // LINE_BYTES = 16 -> WORDS_PER_LINE = 4
    // WORD_SEL_BITS = 2

    logic [31:0] mem [0:63][0:3];
    logic [31:0] rdata_r;
    


    assign rdata = rdata_r;

    always_comb begin
        rdata_r = mem[index][word_sel];
    end

    always_ff @(posedge clk) begin
        if (we) begin
            mem[index][word_sel] <= wdata;
        end
    end

endmodule
module pseudo_lru (
    input  logic       clk,
    input  logic       rst_n,
    
    input  logic       access_en,
    input  logic [5:0] access_index,
    input  logic [1:0] access_way,
    
    output logic [1:0] victim_way
);

    logic [2:0] plru_state [64];
    logic [1:0] victim_way_next;
    
    always_comb begin
        logic [2:0] current_state;
        current_state = plru_state[access_index];
        
        if (!current_state[0]) begin
            victim_way_next = current_state[1] ? 2'd1 : 2'd0;
        end else begin
            victim_way_next = current_state[2] ? 2'd3 : 2'd2;
        end
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 64; i++) begin
                plru_state[i] <= 3'b0;
            end
            victim_way <= 2'b0;
        end else begin
            victim_way <= victim_way_next;
            
            if (access_en) begin
                case (access_way)
                    2'd0: begin
                        plru_state[access_index][0] <= 1'b1;
                        plru_state[access_index][1] <= 1'b1;
                    end
                    2'd1: begin
                        plru_state[access_index][0] <= 1'b1;
                        plru_state[access_index][1] <= 1'b0;
                    end
                    2'd2: begin
                        plru_state[access_index][0] <= 1'b0;
                        plru_state[access_index][2] <= 1'b1;
                    end
                    2'd3: begin
                        plru_state[access_index][0] <= 1'b0;
                        plru_state[access_index][2] <= 1'b0;
                    end
                    default: begin
                        plru_state[access_index] <= 3'b0;
                    end
                endcase
            end
        end
    end
endmodule

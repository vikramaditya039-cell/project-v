// FIXED VERSION - Handles refill data correctly
// Key changes marked with // FIX comments

module l1_cache_core (
    input  logic        clk,
    input  logic        rst_n,

    // CPU Request Interface
    input  logic        req_valid,
    input  logic        req_we,
    input  logic [31:0] req_addr,
    input  logic [31:0] req_wdata,
    input  logic [3:0]  req_wstrb,
    output logic        resp_valid,
    output logic [31:0] resp_rdata,
    output logic        resp_stall,

    // Memory Interface
    output logic        mem_req_valid,
    output logic        mem_req_we,
    output logic [31:0] mem_req_addr,
    output logic [31:0] mem_req_wdata,
    input  logic [31:0] mem_resp_rdata,
    input  logic        mem_resp_valid
);

        // State Machine States
        typedef enum logic [2:0] {
        S_IDLE            = 3'd0,
        S_LOOKUP          = 3'd1,
        S_MISS_SELECT     = 3'd2,
        S_WRITEBACK_REQ   = 3'd3,
        S_WRITEBACK_WAIT  = 3'd4,
        S_REFILL_REQ      = 3'd5,
        S_REFILL_WAIT     = 3'd6
    } state_t;

    state_t state; // (reg) Current FSM state register, tracks cache controller state

    // Current Request Registers
    logic [31:0] cur_req_addr;  // (reg) Latched CPU request address
    logic        cur_req_we;    // (reg) Latched CPU write enable (1=write, 0=read)
    logic [31:0] cur_req_wdata; // (reg) Latched CPU write data
    logic [3:0]  cur_req_wstrb; // (reg) Latched CPU byte-level write strobe

    // FIX: Add register to store refilled words temporarily
    logic [31:0] refill_buffer [0:3]; // (reg) Temporary buffer to hold 4 words (one cache line) during refill from memory

    // Address Decomposition
    logic [5:0]  cur_index;    // (wire) Set index extracted from address bits [9:4], selects 1 of 64 sets
    logic [21:0] cur_tag;      // (wire) Tag extracted from address bits [31:10], used for tag comparison
    logic [1:0]  cur_word_sel; // (wire) Word select from address bits [3:2], picks 1 of 4 words in a cache line
    
    assign cur_index = cur_req_addr[9:4];
    assign cur_tag = cur_req_addr[31:10];
    assign cur_word_sel = cur_req_addr[3:2];

    // Cache Metadata
    logic valid_bits [64][4]; // (reg) Valid bit array: valid_bits[set][way], indicates if a cache line contains valid data
    logic dirty_bits [64][4]; // (reg) Dirty bit array: dirty_bits[set][way], indicates if a cache line has been modified (needs writeback)

    // Tag and Data Array Interfaces
    logic [21:0] tag_rd [4];  // (wire) Tag read data from each of the 4 ways (combinational output from tag arrays)
    logic [31:0] data_rd [4]; // (wire) Data read output from each of the 4 ways (combinational output from data arrays)

    logic        tag_we;      // (reg) Tag array write enable
    logic [5:0]  tag_index;   // (reg) Tag array set index for read/write operations
    logic [1:0]  tag_way;     // (reg) Tag array way select for write operations
    logic [21:0] tag_data;    // (reg) Tag data to be written into the tag array

    logic        data_we;     // (reg) Data array write enable
    logic [5:0]  data_index;  // (reg) Data array set index for read/write operations
    logic [1:0]  data_way;    // (reg) Data array way select for write operations
    logic [31:0] data_wdata;  // (reg) Data to be written into the data array

    logic [1:0]  array_word_sel; // (reg) Word select input to data arrays, picks 1 of 4 words within a cache line

    // Generate Tag and Data Arrays
    generate
        for (genvar w = 0; w < 4; w++) begin : gen_arrays
            tag_array tags (
                .clk(clk),
                .we(tag_we && (tag_way == w[1:0])),
                .index(tag_index),
                .tag_in(tag_data),
                .tag_out(tag_rd[w])
            );

            data_array data (
                .clk(clk),
                .we(data_we && (data_way == w[1:0])),
                .index(data_index),
                .word_sel(array_word_sel),
                .wdata(data_wdata),
                .rdata(data_rd[w])
            );
        end
    endgenerate

    // Pseudo-LRU Replacement Policy
    logic       plru_update_en;  // (reg) Enable signal to update PLRU tree on cache access
    logic [1:0] plru_access_way; // (reg) Way number of the most recently accessed way (input to PLRU)
    logic [1:0] plru_victim_way; // (wire) Victim way selected by PLRU algorithm for eviction (output from PLRU)

    pseudo_lru plru_inst (
        .clk(clk),
        .rst_n(rst_n),
        .access_en(plru_update_en),
        .access_index(cur_index),
        .access_way(plru_access_way),
        .victim_way(plru_victim_way)
    );

    // Miss/Refill Tracking
    logic [1:0]  victim_way;    // (reg) Latched way index of the eviction victim during miss handling
    logic [31:0] victim_addr;   // (reg) Reconstructed full address of the victim line for writeback
    logic [1:0]  transfer_cnt;  // (reg) Word counter (0-3) tracking progress during writeback/refill transfers

    // Response Assignment
    assign resp_stall = (state != S_IDLE);

    // Utility Function - Byte Strobe to Mask
    function automatic logic [31:0] mask_from_strb(input logic [3:0] strb);
        return {{8{strb[3]}}, {8{strb[2]}}, {8{strb[1]}}, {8{strb[0]}}};
    endfunction

    // Combinational hit detection logic
    logic        hit_found; // (wire) Combinational flag: asserted when any way has a tag match (cache hit)
    logic [1:0]  hit_way;   // (wire) Combinational: way index of the matching way on a cache hit
    logic [3:0]  way_hits;  // (wire) Per-way hit vector: way_hits[i]=1 if way i is valid and tag matches
    
    always_comb begin
        for (int h = 0; h < 4; h++) begin
            way_hits[h] = valid_bits[cur_index][h] && (tag_rd[h] == cur_tag);
        end
        hit_found = |way_hits;
        hit_way = '0;
        for (int h = 0; h < 4; h++) begin
            if (way_hits[h]) begin
                hit_way = h[1:0];
            end
        end
    end

    // Combinational victim selection
    logic [1:0] selected_victim_way; // (wire) Combinational: chosen victim way (prefers invalid way, falls back to PLRU)
    logic       found_invalid_way;   // (wire) Combinational flag: asserted if an invalid (empty) way was found for allocation
    
    always_comb begin
        selected_victim_way = plru_victim_way;
        found_invalid_way = '0;
        for (int inv_idx = 0; inv_idx < 4; inv_idx++) begin
            if (!valid_bits[cur_index][inv_idx] && !found_invalid_way) begin
                selected_victim_way = inv_idx[1:0];
                found_invalid_way = 1'b1;
            end
        end
    end
    
    
    // Main State Machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            cur_req_addr <= '0;
            cur_req_we <= '0;
            cur_req_wdata <= '0;
            cur_req_wstrb <= '0;
            array_word_sel <= '0;
            resp_valid <= '0;
            resp_rdata <= '0;
            tag_we <= '0;
            data_we <= '0;
            mem_req_valid <= '0;
            mem_req_we <= '0;
            mem_req_addr <= '0;
            mem_req_wdata <= '0;
            plru_update_en <= '0;
            transfer_cnt <= '0;
            victim_way <= '0;
            victim_addr <= '0;
            data_index <= '0;
            tag_index <= '0;
            plru_access_way <= '0;
            data_way <= '0;
            data_wdata <= '0;
            tag_way <= '0;
            tag_data <= '0;
            
            // FIX: Initialize refill buffer
            for (int i = 0; i < 4; i++) begin
                refill_buffer[i] <= '0;
            end
            
            for (int si = 0; si < 64; si++) begin
                for (int wi = 0; wi < 4; wi++) begin
                    valid_bits[si][wi] <= '0;
                    dirty_bits[si][wi] <= '0;
                end
            end
        end else begin
            resp_valid <= '0;
            tag_we <= '0;
            data_we <= '0;
            mem_req_valid <= '0;
            plru_update_en <= '0;
            
            unique case (state)
                S_IDLE: begin
                    if (req_valid) begin
                        cur_req_addr <= req_addr;
                        cur_req_we <= req_we;
                        cur_req_wdata <= req_wdata;
                        cur_req_wstrb <= req_wstrb;
                        array_word_sel <= req_addr[3:2];
                        state <= S_LOOKUP;
                    end
                end

                S_LOOKUP: begin
                    if (hit_found) begin
                        plru_update_en <= 1'b1;
                        plru_access_way <= hit_way;

                        if (cur_req_we) begin
                            data_index <= cur_index;
                            data_way <= hit_way;
                            array_word_sel <= cur_word_sel;
                            data_wdata <= (data_rd[hit_way] & ~mask_from_strb(cur_req_wstrb)) |
                                          (cur_req_wdata & mask_from_strb(cur_req_wstrb));
                            data_we <= 1'b1;
                            dirty_bits[cur_index][hit_way] <= 1'b1;
                            resp_rdata <= '0;
                        end else begin
                            resp_rdata <= data_rd[hit_way];
                        end
                        
                        resp_valid <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        state <= S_MISS_SELECT;
                    end
                end

                S_MISS_SELECT: begin
                    victim_way <= selected_victim_way;
                    victim_addr <= {tag_rd[selected_victim_way], cur_index, 4'b0};
                    transfer_cnt <= '0;
                    data_index <= cur_index;  // FIX: Set data_index for reading during writeback
                    data_way <= selected_victim_way;

                    if (valid_bits[cur_index][selected_victim_way] && 
                        dirty_bits[cur_index][selected_victim_way]) begin
                        array_word_sel <= '0;
                        state <= S_WRITEBACK_REQ; 
                    end else begin
                        array_word_sel <= '0;
                        state <= S_REFILL_REQ;
                    end
                end
                
                
                S_WRITEBACK_REQ: begin
                    //array_word_sel <= transfer_cnt[1:0];
                    mem_req_valid <= 1'b1;
                    mem_req_we <= 1'b1;
                    mem_req_addr <= victim_addr + (transfer_cnt << 2);
                    mem_req_wdata <= data_rd[victim_way];
                    state <= S_WRITEBACK_WAIT;
                end

                S_WRITEBACK_WAIT: begin
                    if (mem_resp_valid) begin
                        if (transfer_cnt == 3) begin
                            transfer_cnt <= '0;
                            state <= S_REFILL_REQ;
                        end else begin
                            transfer_cnt <= transfer_cnt + 1'b1;
                            array_word_sel <= transfer_cnt + 1'b1;  // Set for next iteration
                            state <= S_WRITEBACK_REQ;  // Go back to prep
                        end
                    end
                end

                S_REFILL_REQ: begin
                    array_word_sel <= transfer_cnt[1:0];
                    mem_req_valid <= 1'b1;
                    mem_req_we <= '0;
                    mem_req_addr <= {cur_req_addr[31:4], 4'b0} + (transfer_cnt << 2);
                    state <= S_REFILL_WAIT;
                end

                S_REFILL_WAIT: begin
                    if (mem_resp_valid) begin
                        // FIX: Store in buffer AND write to cache
                        refill_buffer[transfer_cnt] <= mem_resp_rdata;
                        
                        // Write refilled data to cache
                        data_index <= cur_index;
                        data_way <= victim_way;
                        array_word_sel <= transfer_cnt[1:0];
                        data_wdata <= mem_resp_rdata;
                        data_we <= 1'b1;

                        if (transfer_cnt == 3) begin
                            // Last word - complete refill
                            transfer_cnt <= '0;
                            
                            // Update tag
                            tag_index <= cur_index;
                            tag_way <= victim_way;
                            tag_data <= cur_tag;
                            tag_we <= 1'b1;
                            
                            // Mark valid
                            valid_bits[cur_index][victim_way] <= 1'b1;
                            
                            // Set dirty based on operation
                            if (cur_req_we) begin
                                dirty_bits[cur_index][victim_way] <= 1'b1;
                            end else begin
                                dirty_bits[cur_index][victim_way] <= 1'b0;
                            end
                            
                            // Update LRU
                            plru_update_en <= 1'b1;
                            plru_access_way <= victim_way;
                            
                            if (!cur_req_we) begin
                                // FIX: Read after refill - use buffer directly!
                                resp_rdata <= refill_buffer[cur_word_sel];
                                resp_valid <= 1'b1;
                            end else begin
                                // FIX: Write after refill - use buffer for read-modify-write
                                data_index <= cur_index;
                                data_way <= victim_way;
                                array_word_sel <= cur_word_sel;
                                data_wdata <= (refill_buffer[cur_word_sel] & ~mask_from_strb(cur_req_wstrb)) |
                                              (cur_req_wdata & mask_from_strb(cur_req_wstrb));
                                data_we <= 1'b1;
                                resp_rdata <= '0;
                                resp_valid <= 1'b1;
                            end
                            
                            state <= S_IDLE;
                            
                        end else begin
                            // More words to fetch
                            transfer_cnt <= transfer_cnt + 1'b1;
                            state <= S_REFILL_REQ;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
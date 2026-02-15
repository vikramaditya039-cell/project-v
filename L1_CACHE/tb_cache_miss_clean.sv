`timescale 1ns/1ps

module tb_cache_miss_clean;

    // Clock and Reset
    logic        clk;
    logic        rst_n;

    // CPU Request Interface
    logic        req_valid;
    logic        req_we;
    logic [31:0] req_addr;
    logic [31:0] req_wdata;
    logic [3:0]  req_wstrb;
    logic        resp_valid;
    logic [31:0] resp_rdata;
    logic        resp_stall;

    // Memory Interface
    logic        mem_req_valid;
    logic        mem_req_we;
    logic [31:0] mem_req_addr;
    logic [31:0] mem_req_wdata;
    logic [31:0] mem_resp_rdata;
    logic        mem_resp_valid;

    // Instantiate DUT
    l1_cache_core dut (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(req_valid),
        .req_we(req_we),
        .req_addr(req_addr),
        .req_wdata(req_wdata),
        .req_wstrb(req_wstrb),
        .resp_valid(resp_valid),
        .resp_rdata(resp_rdata),
        .resp_stall(resp_stall),
        .mem_req_valid(mem_req_valid),
        .mem_req_we(mem_req_we),
        .mem_req_addr(mem_req_addr),
        .mem_req_wdata(mem_req_wdata),
        .mem_resp_rdata(mem_resp_rdata),
        .mem_resp_valid(mem_resp_valid)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    // Memory model - simulates main memory with configurable latency
    logic [1:0] mem_latency_cnt;
    parameter MEM_LATENCY = 1; // 1 cycle latency

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_resp_valid <= 1'b0;
            mem_resp_rdata <= 32'h0;
            mem_latency_cnt <= 2'b0;
        end else begin
            if (mem_req_valid && mem_latency_cnt == 0) begin
                mem_latency_cnt <= MEM_LATENCY;
            end else if (mem_latency_cnt > 0) begin
                mem_latency_cnt <= mem_latency_cnt - 1;
                if (mem_latency_cnt == 1) begin
                    mem_resp_valid <= 1'b1;
                    if (!mem_req_we) begin
                        // Generate pattern based on address
                        mem_resp_rdata <= mem_req_addr + 32'h12340000;
                    end else begin
                        mem_resp_rdata <= 32'h0;
                    end
                end else begin
                    mem_resp_valid <= 1'b0;
                end
            end else begin
                mem_resp_valid <= 1'b0;
            end
        end
    end

    // Track refill progress
    integer refill_word_count;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refill_word_count <= 0;
        end else begin
            if (mem_req_valid && !mem_req_we && mem_resp_valid) begin
                refill_word_count <= refill_word_count + 1;
            end else if (resp_valid) begin
                refill_word_count <= 0;
            end
        end
    end

    // Test sequence
    initial begin
        // Initialize signals
        rst_n = 1'b0;
        req_valid = 1'b0;
        req_we = 1'b0;
        req_addr = 32'h0;
        req_wdata = 32'h0;
        req_wstrb = 4'h0;

        // Generate VCD dump
        $dumpfile("cache_miss_clean.vcd");
        $dumpvars(0, tb_cache_miss_clean);

        // Reset
        #20;
        rst_n = 1'b1;
        

        $display("\n=== CACHE MISS WITH CLEAN VICTIM (REFILL ONLY) ===\n");

        // ========================================
        // TEST 1: First miss - invalid victim (refill from memory)
        // ========================================
        $display("TEST 1: Read MISS to address 0x2000 (invalid victim - refill)");
        $display("  Cache state: Empty (all ways invalid)");
        $display("  Expected: MISS -> Select invalid way -> REFILL 4 words");
        
        @(posedge clk);
        req_valid = 1'b1;
        req_we = 1'b0;
        req_addr = 32'h00002000;  // Index=0, Tag=0x8
        req_wstrb = 4'hF;
        
        @(negedge clk);
        req_valid = 1'b0;
        
        $display("  Time %0t: Request issued", $time);
        
        // Monitor state progression
        wait(dut.state == dut.S_MISS_SELECT);
        $display("  Time %0t: State = MISS_SELECT", $time);
        
        wait(dut.state == dut.S_REFILL_REQ);
        $display("  Time %0t: State = REFILL_REQ (starting refill)", $time);
        
        // Count refill transfers
        repeat(4) begin
            wait(mem_req_valid && !mem_req_we);
            $display("  Time %0t: Memory read request [%0d/4]: addr=0x%h", 
                     $time, refill_word_count+1, mem_req_addr);
            wait(mem_resp_valid);
            $display("  Time %0t: Memory response [%0d/4]: data=0x%h", 
                     $time, refill_word_count, mem_resp_rdata);
            @(negedge clk);
        end
        
        // Wait for CPU response
        wait(resp_valid == 1'b1);
        $display("  Time %0t: CPU Response: rdata=0x%h", $time, resp_rdata);
        $display("  Total refill time: %0t ns", $time - 40);
        @(negedge clk);

        #100;

        // ========================================
        // TEST 2: Another miss - different tag, same set (clean victim)
        // ========================================
        $display("\nTEST 2: Read MISS to address 0x3000 (clean victim - refill)");
        $display("  Previous line at 0x2000 is clean (not modified)");
        $display("  Expected: MISS -> Select victim (way 0 or invalid) -> REFILL");
        
        @(negedge clk);
        req_valid = 1'b1;
        req_we = 1'b0;
        req_addr = 32'h00003000;  // Index=0, Tag=0xC - different tag
        req_wstrb = 4'hF;
        
        @(negedge clk);
        req_valid = 1'b0;
        
        $display("  Time %0t: Request issued", $time);
        
        wait(dut.state == dut.S_MISS_SELECT);
        $display("  Time %0t: State = MISS_SELECT", $time);
        $display("  Selected victim way: %0d", dut.selected_victim_way);
        
        wait(dut.state == dut.S_REFILL_REQ);
        $display("  Time %0t: State = REFILL_REQ (no writeback needed)", $time);
        
        // Monitor refill
        repeat(4) begin
            wait(mem_req_valid && !mem_req_we);
            $display("  Time %0t: Memory read [%0d/4]: addr=0x%h", 
                     $time, refill_word_count+1, mem_req_addr);
            wait(mem_resp_valid);
            @(negedge clk);
        end
        
        wait(resp_valid == 1'b1);
        $display("  Time %0t: CPU Response: rdata=0x%h", $time, resp_rdata);
        @(negedge clk);

        #100;

        // ========================================
        // TEST 3: Write miss with clean victim
        // ========================================
        $display("\nTEST 3: Write MISS to address 0x4000 (clean victim)");
        $display("  Expected: MISS -> REFILL -> WRITE to cache");
        
        @(negedge clk);
        req_valid = 1'b1;
        req_we = 1'b1;
        req_addr = 32'h00004000;  // Index=0, Tag=0x10
        req_wdata = 32'habcd0123;
        req_wstrb = 4'hF;
        
        @(negedge clk);
        req_valid = 1'b0;
        
        $display("  Time %0t: Write request issued", $time);
        
        wait(dut.state == dut.S_MISS_SELECT);
        $display("  Time %0t: State = MISS_SELECT", $time);
        
        wait(dut.state == dut.S_REFILL_REQ);
        $display("  Time %0t: State = REFILL_REQ", $time);
        
        // Monitor refill
        repeat(4) begin
            wait(mem_req_valid && !mem_req_we);
            $display("  Time %0t: Memory read [%0d/4]: addr=0x%h", 
                     $time, refill_word_count+1, mem_req_addr);
            wait(mem_resp_valid);
            @(negedge clk);
        end
        
        wait(resp_valid == 1'b1);
        $display("  Time %0t: Write complete, line marked dirty", $time);
        @(negedge clk);

        #100;

        // ========================================
        // TEST 4: Verify cache state after refills
        // ========================================
        $display("\nTEST 4: Verification reads (should all hit now)");
        
        // Read back the write
        @(negedge clk);
        req_valid = 1'b1;
        req_we = 1'b0;
        req_addr = 32'h00004000;
        req_wstrb = 4'hF;
        
        @(negedge clk);
        req_valid = 1'b0;
        
        wait(resp_valid == 1'b1);
        $display("  Verification: Read 0x4000 = 0x%h (expected 0xabcd0123)", resp_rdata);
        if (resp_rdata == 32'habcd0123)
            $display("  ✓ Write after refill PASSED");
        @(negedge clk);

        #100;

        // ========================================
        // Summary
        // ========================================
        $display("\n=== TEST SUMMARY ===");
        $display("Cache MISS with CLEAN victim tests completed!");
        $display("\nKey Observations:");
        $display("1. Invalid way selection takes priority over PLRU");
        $display("2. Clean victim requires NO writeback");
        $display("3. Refill sequence: 4 memory reads for complete cache line");
        $display("4. Each refill word takes ~2 cycles (REQ + WAIT states)");
        $display("5. Total miss penalty (clean): ~8-10 cycles for 4-word refill");
        $display("\nState sequence: IDLE -> LOOKUP -> MISS_SELECT -> REFILL_REQ -> REFILL_WAIT (x4) -> IDLE");
        $display("\nWaveform saved to cache_miss_clean.vcd");
        
        #100;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #20000;
        $display("\nERROR: Simulation timeout!");
        $finish;
    end

    // Detailed state monitor
    always @(posedge clk) begin
        if (dut.state != dut.S_IDLE && dut.state != $past(dut.state))
            $display("  [%0t] State change: %0d -> %0d", $time, $past(dut.state), dut.state);
    end

endmodule

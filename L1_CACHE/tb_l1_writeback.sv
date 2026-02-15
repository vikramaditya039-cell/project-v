`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Design Name: L1 Cache Writeback Testbench
// Module Name: tb_l1_writeback
// Description: Verifies the Dirty-Victim Eviction -> Refill scenario.
//////////////////////////////////////////////////////////////////////////////////

module tb_l1_writeback();

    // -------------------------------------------------------------------------
    // Signal Declarations
    // -------------------------------------------------------------------------
    logic        clk;
    logic        rst_n;

    // CPU Interface
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

    // -------------------------------------------------------------------------
    // DUT Instantiation
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Memory Model Instantiation
    // -------------------------------------------------------------------------
    memory_model #(
        .MEM_WORDS(65536) // 64K words
    ) mem_inst (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(mem_req_valid),
        .req_we(mem_req_we),
        .req_addr(mem_req_addr),
        .req_wdata(mem_req_wdata),
        .resp_rdata(mem_resp_rdata),
        .resp_valid(mem_resp_valid)
    );

    // -------------------------------------------------------------------------
    // Clock Generation
    // -------------------------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period
    end

    // -------------------------------------------------------------------------
    // Helper Tasks
    // -------------------------------------------------------------------------
    task cpu_access(
        input logic        we,
        input logic [31:0] addr,
        input logic [31:0] data
    );
        begin
            // Wait for stall to deassert before sending new request
            wait(!resp_stall);
            
            @(posedge clk);
            req_valid <= 1'b1;
            req_we    <= we;
            req_addr  <= addr;
            req_wdata <= data;
            req_wstrb <= 4'b1111; // Full word write
            
            @(posedge clk);
            req_valid <= 1'b0;
            
            // Wait for response
            wait(resp_valid);
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Monitoring Processes
    // -------------------------------------------------------------------------
    // Track memory transactions to verify sequence
    logic saw_writeback;
    logic saw_refill;

    always @(posedge clk) begin
        if (rst_n && mem_req_valid) begin
            if (mem_req_we) begin
                $display("[Time %0t] MEMORY WRITE: Addr=0x%h Data=0x%h", $time, mem_req_addr, mem_req_wdata);
                saw_writeback <= 1'b1;
            end else begin
                $display("[Time %0t] MEMORY READ:  Addr=0x%h", $time, mem_req_addr);
                // Only count as checking the "refill" phase if we have already seen a writeback
                // or if this is part of the final check
                if (req_addr == 32'h00005000) begin // The specific address for the conflict
                     saw_refill <= 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Main Stimulus
    // -------------------------------------------------------------------------
    initial begin
        // Init
        rst_n = 0;
        req_valid = 0;
        req_we = 0;
        req_addr = 0;
        req_wdata = 0;
        req_wstrb = 0;
        saw_writeback = 0;
        saw_refill = 0;

        $display("---------------------------------------");
        $display("Starting L1 Writeback Testbench");
        $display("---------------------------------------");

        #10 rst_n = 1;
        #10;

        // -------------------------------------------------------
        // Step 1: Fill Set 0 with Dirty Lines
        // -------------------------------------------------------
        // Cache Params: 
        // Index [9:4]: 6 bits -> 64 sets
        // Tag [31:10]: 22 bits
        // WordSel [3:2]: 2 bits -> 4 words/line
        // Address Format: [Tag][Index][Word][Byte]
        // Set 0 Index = 0x00
        
        // We will write to 4 different tags mapping to Set 0.
        // Addresses below map to Index 0.
        // To avoid 'X's in writeback, we must write to ALL words in the line.
        // Line size = 16 bytes (4 words).
        
        $display("STEP 1: Fills Set 0 with 4 Dirty Lines (Writing full lines)");
        
        // Way 0 (Tag 1: 0x00001000)
        cpu_access(1, 32'h00001000, 32'h11111111); 
        cpu_access(1, 32'h00001004, 32'h11111111);
        cpu_access(1, 32'h00001008, 32'h11111111);
        cpu_access(1, 32'h0000100C, 32'h11111111);

        // Way 1 (Tag 2: 0x00002000)
        cpu_access(1, 32'h00002000, 32'h22222222);
        cpu_access(1, 32'h00002004, 32'h22222222);
        cpu_access(1, 32'h00002008, 32'h22222222);
        cpu_access(1, 32'h0000200C, 32'h22222222);

        // Way 2 (Tag 3: 0x00003000)
        cpu_access(1, 32'h00003000, 32'h33333333);
        cpu_access(1, 32'h00003004, 32'h33333333);
        cpu_access(1, 32'h00003008, 32'h33333333);
        cpu_access(1, 32'h0000300C, 32'h33333333);

        // Way 3 (Tag 4: 0x00004000)
        cpu_access(1, 32'h00004000, 32'h44444444);
        cpu_access(1, 32'h00004004, 32'h44444444);
        cpu_access(1, 32'h00004008, 32'h44444444);
        cpu_access(1, 32'h0000400C, 32'h44444444);
        
        $display("Current State: Set 0 is FULL of DIRTY lines.");
        
        // -------------------------------------------------------
        // Step 2: Trigger Conflict Miss
        // -------------------------------------------------------
        // Access a 5th Tag mapping to Set 0.
        // Tag 5: 0x00005000
        // This MUST cause an eviction. 
        // Since we touched them 1,2,3,4, the PLRU state typically protects the MRU ones.
        // But regardless of which is victim, ALL are dirty.
        // So we MUST see a Writeback transaction followed by a Refill.
        
        $display("STEP 2: Requesting 5th Address (0x00005000) - Should Trigger WB + Refill");
        
        // Reset monitors for this phase
        saw_writeback = 0;
        saw_refill = 0;
        
        // Issue Read Request for new address
        cpu_access(0, 32'h00005000, 32'h0);
        
        // -------------------------------------------------------
        // Verify Results
        // -------------------------------------------------------
        if (saw_writeback && saw_refill) begin
            $display("SUCCESS: Observed both Writeback and Refill transactions.");
        end else begin
            $display("FAILURE: Missing expected transactions.");
            if (!saw_writeback) $display(" - Did NOT observe Writeback.");
            if (!saw_refill)    $display(" - Did NOT observe Refill.");
        end

        #100;
        $finish;
    end

endmodule

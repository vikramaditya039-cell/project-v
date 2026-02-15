`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/29/2026
// Design Name: PLRU Testbench
// Module Name: tb_plru
// Project Name: Titan Cache
// Target Devices: 
// Tool Versions: 
// Description: Testbench for Pseudo-LRU replacement policy
// 
// Dependencies: pseudo_lru.sv
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tb_plru();

    // -------------------------------------------------------------------------
    // Signal Declarations
    // -------------------------------------------------------------------------
    logic       clk;
    logic       rst_n;
    
    logic       access_en;
    logic [5:0] access_index;
    logic [1:0] access_way;
    
    logic [1:0] victim_way;

    // -------------------------------------------------------------------------
    // DUT Instantiation
    // -------------------------------------------------------------------------
    pseudo_lru dut (
        .clk(clk),
        .rst_n(rst_n),
        .access_en(access_en),
        .access_index(access_index),
        .access_way(access_way),
        .victim_way(victim_way)
    );

    // -------------------------------------------------------------------------
    // Clock Generation
    // -------------------------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period -> 100MHz
    end

    // -------------------------------------------------------------------------
    // Test Tasks
    // -------------------------------------------------------------------------
    
    // Task to perform an access and wait for the result to propagate
    task do_access(input logic [5:0] idx, input logic [1:0] way);
        begin
            @(posedge clk);
            access_index = idx;
            access_en = 1'b1;
            access_way = way;
            
            @(posedge clk);
            access_en = 1'b0;
            
            // Wait one more cycle for the victim_way output to register the new state
            @(posedge clk);
        end
    endtask

    // Task to check the current victim output
    task check_victim(input logic [5:0] idx, input logic [1:0] expected_victim);
        begin
            // Set index and wait for registered output
            access_index = idx;
            access_en = 1'b0; // Ensure we aren't writing
            
            // Wait for pipeline delay:
            // T0: index set
            // T0->T1: next logic settles
            // T1 edge: valid output registered
            repeat(2) @(posedge clk); 
            
            if (victim_way !== expected_victim) begin
                $error("[Time %0t] Error: Index %0d, Expected Owner %0d, Got %0d", 
                        $time, idx, expected_victim, victim_way);
            end else begin
                $display("[Time %0t] Pass: Index %0d, Victim %0d", 
                         $time, idx, victim_way);
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main Test Stimulus
    // -------------------------------------------------------------------------
    initial begin
        // Initialize
        rst_n = 0;
        access_en = 0;
        access_index = 0;
        access_way = 0;

        $display("---------------------------------------");
        $display("Starting PLRU Testbench");
        $display("---------------------------------------");

        // Apply Reset
        #20 rst_n = 1;
        #10;

        // ---------------------------------------------
        // Test Case 1: Reset Check
        // ---------------------------------------------
        $display("Test Case 1: Checking Reset State (Victim should be 0)");
        check_victim(0, 0);

        // ---------------------------------------------
        // Test Case 2: Verify PLRU logic on Set 0
        // Path: 0 -> 2 -> 1 -> 3 -> 0
        // ---------------------------------------------
        $display("\nTest Case 2: Walking through PLRU states on Set 0");

        // 1. Access Way 0. State should update to point AWAY from 0.
        // Initial State: 000 (points to 0).
        // Access 0 -> State becomes x11 (points to Right, Way 2).
        do_access(0, 0); 
        check_victim(0, 2);

        // 2. Access Way 2.
        // State x11 -> Access 2 -> x10 (Points Left, Way 1? No)
        // Way 2 Access: Bit0<=0 (Left), Bit2<=1 (Points to 3).
        // Wait, Code:
        // Way 2: state[0]<=0, state[2]<=1.
        // Current: x11 (Bit 0=1, Bit 1=1).
        // New: 011? No.
        // Original: 000
        // Access 0 -> 011 (Bit0=1, Bit1=1).
        // Access 2 -> 110 (Bit0=0, Bit2=1). Bit1 stays 1.
        // Logic check: Bit0=0(Left). Left child is Bit1. Bit1=1 -> Victim 1.
        do_access(0, 2);
        check_victim(0, 1);

        // 3. Access Way 1.
        // State 110.
        // Access 1: state[0]<=1, state[1]<=0.
        // New: 101 (Bit0=1, Bit2=1, Bit1=0).
        // Logic check: Bit0=1(Right). Right child is Bit2. Bit2=1 -> Victim 3.
        do_access(0, 1);
        check_victim(0, 3);

        // 4. Access Way 3.
        // State 101.
        // Access 3: state[0]<=0, state[2]<=0.
        // New: 000 (Bit0=0, Bit2=0, Bit1=0).
        // Logic check: Bit0=0(Left). Left child is Bit1. Bit1=0 -> Victim 0.
        do_access(0, 3);
        check_victim(0, 0);

        // ---------------------------------------------
        // Test Case 3: Independence of Sets
        // ---------------------------------------------
        $display("\nTest Case 3: Checking Independence of Sets");
        
        // Modify Set 1, ensure Set 0 stays at 0 (from previous step)
        // Access Way 0 on Set 1 -> Victim should become 2 for Set 1.
        do_access(1, 0);
        
        // Check Set 1
        check_victim(1, 2);
        
        // Check Set 0 (Should still be 0)
        check_victim(0, 0);

        // ---------------------------------------------
        // Completion
        // ---------------------------------------------
        $display("\n---------------------------------------");
        $display("Testbench Complete");
        $display("---------------------------------------");
        $finish;
    end

endmodule

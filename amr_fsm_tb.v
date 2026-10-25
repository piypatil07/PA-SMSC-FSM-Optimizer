`timescale 1ns/1ps

module amr_fsm_tb;
    reg clk;
    reg rst;
    reg [12:1] x;
    wire [12:1] y;

    // Instantiate the DUT (Device Under Test)
    amr_pa_smsc_fsm uut (
        .clk(clk),
        .rst(rst),
        .x(x),
        .y(y)
    );

    // Clock generation: 10ns period (100 MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test Scenarios
    initial begin
        $display("========================================");
        $display("Starting AMR FSM Simulation");
        $display("========================================");

        // Initialize
        rst = 1; x = 12'b0;
        #20;
        rst = 0;
        #10;

        // SCENARIO 1: Normal Navigation (a1 -> a2)
        $display("[TIME %0t] Scenario 1: Normal Navigation Start", $time);
        x[8] = 1; x[9] = 1; // x8x9 active
        #20;
        if (uut.cs == 7'b1000000) $display("SUCCESS: Transitioned to a2");
        else $display("FAIL: Expected a2, got %b", uut.cs);
        
        // SCENARIO 2: Emergency Stop (a2 -> a12 via a3)
        $display("[TIME %0t] Scenario 2: Emergency Stop", $time);
        x[8] = 0; x[9] = 0; x[1] = 0; x[3] = 1; // Trigger a3 then a12
        #20;
        x[3] = 1; // Hold x3 for a12
        #20;
        if (uut.cs == 7'b0110010) $display("SUCCESS: Transitioned to a12 (E-Stop)");
        else $display("FAIL: Expected a12, got %b", uut.cs);

        // SCENARIO 3: Low Battery (a12 -> a16)
        $display("[TIME %0t] Scenario 3: Low Battery Detection", $time);
        x[3] = 0; x[6] = 1; // x6 (Battery Critical)
        #20;
        if (uut.cs == 7'b0101001) $display("SUCCESS: Transitioned to a16 (Battery Critical)");
        else $display("FAIL: Expected a16, got %b", uut.cs);

        // Finish
        #50;
        $display("========================================");
        $display("Simulation Complete");
        $display("========================================");
        $finish;
    end

    // Monitor outputs
    initial begin
        $monitor("TIME=%0t | STATE=%b (Decimal: %0d) | INPUTS=%b | OUTPUTS=%b", 
                 $time, uut.cs, uut.cs, x, y);
    end
endmodule
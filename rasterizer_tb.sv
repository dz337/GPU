`timescale 1ns / 1ps

module rasterizer_tb;
    
    // Clock and reset
    logic clk = 0;
    logic rst_n = 0;
    always #5 clk = ~clk;  // 100MHz clock
    
    // Rasterizer signals
    logic i_start;
    logic signed [9:0] i_v0_x, i_v0_y, i_v1_x, i_v1_y, i_v2_x, i_v2_y;
    logic o_fragment_valid;
    logic signed [9:0] o_fragment_x, o_fragment_y;
    logic signed [20:0] o_lambda0, o_lambda1, o_lambda2;
    logic o_done;
    
    // Instantiate rasterizer
    rasterizer #(.CORD_WIDTH(10)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_start(i_start),
        .i_v0_x(i_v0_x), .i_v0_y(i_v0_y),
        .i_v1_x(i_v1_x), .i_v1_y(i_v1_y),
        .i_v2_x(i_v2_x), .i_v2_y(i_v2_y),
        .o_fragment_valid(o_fragment_valid),
        .o_fragment_x(o_fragment_x),
        .o_fragment_y(o_fragment_y),
        .o_lambda0(o_lambda0),
        .o_lambda1(o_lambda1),
        .o_lambda2(o_lambda2),
        .o_done(o_done)
    );
    
    // Fragment counter
    int fragment_count = 0;
    
    always @(posedge clk) begin
        if (o_fragment_valid) begin
            fragment_count++;
            $display("Fragment %0d: (%0d,%0d) edge=(% 0d,% 0d,% 0d)", 
                     fragment_count, o_fragment_x, o_fragment_y,
                     o_lambda0, o_lambda1, o_lambda2);
        end
    end
    
    initial begin
        $display("=== Rasterizer Testbench ===");
        
        // Reset
        i_start = 0;
        #20 rst_n = 1;
        #20;
        
        // Test triangle: (20,20), (30,20), (25,30)
        $display("\nTesting triangle (20,20)-(30,20)-(25,30)");
        i_v0_x = 20; i_v0_y = 20;
        i_v1_x = 30; i_v1_y = 20;
        i_v2_x = 25; i_v2_y = 30;
        i_start = 1;
        #10 i_start = 0;
        
        // Wait for completion
        wait(o_done);
        #20;
        
        $display("\n=== Results ===");
        $display("Total fragments generated: %0d", fragment_count);
        $display("Expected: ~55 fragments for this triangle");
        
        if (fragment_count == 0)
            $display("ERROR: No fragments generated!");
        
        $finish;
    end
    
    // Timeout
    initial begin
        #50000;
        $display("TIMEOUT - rasterizer did not complete");
        $finish;
    end
    
endmodule

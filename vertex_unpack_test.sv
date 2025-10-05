`timescale 1ns / 1ps

module vertex_unpack_test;
    logic clk = 0;
    always #5 clk = ~clk;
    
    // Simulate vertex_fetch output
    logic [32*15-1:0] vertex_data_full;
    
    // Unpacked outputs
    logic signed [9:0] v0x, v0y, v1x, v1y, v2x, v2y;
    
    // Same unpacking as gpu_top
    assign v0x = vertex_data_full[0*32 +: 10];
    assign v0y = vertex_data_full[1*32 +: 10];
    assign v1x = vertex_data_full[2*32 +: 10];
    assign v1y = vertex_data_full[3*32 +: 10];
    assign v2x = vertex_data_full[4*32 +: 10];
    assign v2y = vertex_data_full[5*32 +: 10];
    
    initial begin
        $display("=== Vertex Unpacking Test ===");
        
        // Set raw vertex data like it comes from vertex_fetch
        vertex_data_full[0*32 +: 32] = 32'd20;  // v0_x
        vertex_data_full[1*32 +: 32] = 32'd20;  // v0_y
        vertex_data_full[2*32 +: 32] = 32'd30;  // v1_x
        vertex_data_full[3*32 +: 32] = 32'd20;  // v1_y
        vertex_data_full[4*32 +: 32] = 32'd25;  // v2_x
        vertex_data_full[5*32 +: 32] = 32'd30;  // v2_y
        
        #10;
        
        $display("After unpacking:");
        $display("  v0x = %d (expected 20)", v0x);
        $display("  v0y = %d (expected 20)", v0y);
        $display("  v1x = %d (expected 30)", v1x);
        $display("  v1y = %d (expected 20)", v1y);
        $display("  v2x = %d (expected 25)", v2x);
        $display("  v2y = %d (expected 30)", v2y);
        
        if (v0x == 20 && v1x == 30 && v2x == 25)
            $display("\nSUCCESS: Vertex unpacking works correctly!");
        else
            $display("\nERROR: Vertex unpacking is broken!");
        
        $finish;
    end
endmodule

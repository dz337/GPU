`timescale 1ns / 1ps

module gpu_top_vertex_tb;
    
    logic clk = 0;
    logic rst_n = 0;
    always #5 clk = ~clk;
    
    // GPU control signals
    logic i_bus_we;
    logic [31:0] i_bus_addr, i_bus_wdata, o_bus_rdata;
    
    // DDR interface (fake memory)
    logic o_dram_we;
    logic [31:0] o_dram_addr, o_dram_wdata, i_dram_rdata;
    
    // Fake DDR memory - enlarged for framebuffer
    logic [31:0] fake_ddr[0:327679];  // ~1.3MB for 640x480 framebuffer + vertex data
    
    // DDR read/write logic - use bit slicing directly
    always @(posedge clk) begin
        if (!o_dram_we && o_dram_addr[31:28] == 4'h1) begin
            i_dram_rdata <= fake_ddr[(o_dram_addr - 32'h10000000) >> 2];
        end else begin
            i_dram_rdata <= 32'h00000000;
        end
    end
    
    // DDR write logic - compute index inline
    always @(posedge clk) begin
        if (o_dram_we && o_dram_addr[31:28] == 4'h1) begin
            fake_ddr[(o_dram_addr - 32'h10000000) >> 2] <= o_dram_wdata;
            $display("Writing to fake_ddr[%0d] = 0x%h", (o_dram_addr - 32'h10000000) >> 2, o_dram_wdata);
        end
    end
    
    // Instantiate GPU
    gpu_top #(
        .DATA_WIDTH(32),
        .CORD_WIDTH(10)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_bus_we(i_bus_we),
        .i_bus_addr(i_bus_addr),
        .i_bus_wdata(i_bus_wdata),
        .o_bus_rdata(o_bus_rdata),
        .o_dram_we(o_dram_we),
        .o_dram_addr(o_dram_addr),
        .o_dram_wdata(o_dram_wdata),
        .i_dram_rdata(i_dram_rdata)
    );
    
    // Monitor rasterizer fragments
    int frag_count = 0;
    always @(posedge clk) begin
        if (dut.rast_frag_valid) begin
            frag_count++;
            if (frag_count <= 10 || frag_count > 51) begin
                $display("Fragment %0d: (%0d,%0d)", frag_count, 
                         dut.frag_x, dut.frag_y);
            end
        end
    end
    
    initial begin
        $display("=== GPU Top Vertex Fetch Test ===");
        
        // Initialize all bus signals
        i_bus_we = 0;
        i_bus_addr = 0;
        i_bus_wdata = 0;
        
        // Initialize fake DDR with triangle data
        // Vertex 0: (20,20)
        fake_ddr[0] = 32'd20;  // v0_x
        fake_ddr[1] = 32'd20;  // v0_y
        // Vertex 1: (30,20)
        fake_ddr[2] = 32'd30;  // v1_x
        fake_ddr[3] = 32'd20;  // v1_y
        // Vertex 2: (25,30)
        fake_ddr[4] = 32'd25;  // v2_x
        fake_ddr[5] = 32'd30;  // v2_y
        // Colors
        fake_ddr[6] = 32'hFFFF0000;  // red
        fake_ddr[7] = 32'hFF00FF00;  // green
        fake_ddr[8] = 32'hFF0000FF;  // blue
        // UVs (zeros)
        for (int i = 9; i < 15; i++) fake_ddr[i] = 0;
        
        // Reset
        i_bus_we = 0;
        #20 rst_n = 1;
        #20;
        
        // Configure GPU
        $display("Configuring GPU...");
        write_reg(32'h00000008, 32'h10000000);  // VERTEX_BASE
        write_reg(32'h0000000C, 32'd3);          // VERTEX_COUNT
        write_reg(32'h00000014, 32'h10010000);  // FB_BASE - within testbench memory
        
        // Start GPU
        $display("Starting GPU pipeline...");
        write_reg(32'h00000000, 32'd1);  // CONTROL
        
        // Wait for completion
        repeat(5000) @(posedge clk);
        
        $display("\n=== Results ===");
        $display("Total fragments generated: %0d", frag_count);
        if (frag_count == 0)
            $display("ERROR: No fragments - vertex data not reaching rasterizer!");
        else if (frag_count < 50)
            $display("WARNING: Only %0d fragments (expected ~61)", frag_count);
        else
            $display("SUCCESS: GPU rendered triangle!");
        
        $finish;
    end
    
    task write_reg(input [31:0] addr, input [31:0] data);
        #1;  // Small delay before clock edge
        i_bus_addr = addr;
        i_bus_wdata = data;
        i_bus_we = 1;
        @(posedge clk);
        #1;
        i_bus_we = 0;
        @(posedge clk);
    endtask
    
endmodule











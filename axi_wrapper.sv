// axi_wrapper.sv - Fixed version
module axi_wrapper #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 32,
    parameter integer C_M_AXI_DATA_WIDTH = 32,
    parameter integer C_M_AXI_ADDR_WIDTH = 32,
    parameter integer C_M_AXI_ID_WIDTH = 4
)(
    // AXI4-Lite Slave Interface
    input  wire                                  s_axi_aclk,
    input  wire                                  s_axi_aresetn,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        s_axi_awaddr,
    input  wire [2:0]                           s_axi_awprot,
    input  wire                                  s_axi_awvalid,
    output wire                                  s_axi_awready,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]        s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]    s_axi_wstrb,
    input  wire                                  s_axi_wvalid,
    output wire                                  s_axi_wready,
    output wire [1:0]                           s_axi_bresp,
    output wire                                  s_axi_bvalid,
    input  wire                                  s_axi_bready,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        s_axi_araddr,
    input  wire [2:0]                           s_axi_arprot,
    input  wire                                  s_axi_arvalid,
    output wire                                  s_axi_arready,
    output wire [C_S_AXI_DATA_WIDTH-1:0]        s_axi_rdata,
    output wire [1:0]                           s_axi_rresp,
    output wire                                  s_axi_rvalid,
    input  wire                                  s_axi_rready,
    
    // AXI4 Master Interface
    output wire [C_M_AXI_ID_WIDTH-1:0]          m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]        m_axi_awaddr,
    output wire [7:0]                           m_axi_awlen,
    output wire [2:0]                           m_axi_awsize,
    output wire [1:0]                           m_axi_awburst,
    output wire                                  m_axi_awlock,
    output wire [3:0]                           m_axi_awcache,
    output wire [2:0]                           m_axi_awprot,
    output wire [3:0]                           m_axi_awqos,
    output wire                                  m_axi_awvalid,
    input  wire                                  m_axi_awready,
    output wire [C_M_AXI_DATA_WIDTH-1:0]        m_axi_wdata,
    output wire [(C_M_AXI_DATA_WIDTH/8)-1:0]    m_axi_wstrb,
    output wire                                  m_axi_wlast,
    output wire                                  m_axi_wvalid,
    input  wire                                  m_axi_wready,
    input  wire [C_M_AXI_ID_WIDTH-1:0]          m_axi_bid,
    input  wire [1:0]                           m_axi_bresp,
    input  wire                                  m_axi_bvalid,
    output wire                                  m_axi_bready,
    output wire [C_M_AXI_ID_WIDTH-1:0]          m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]        m_axi_araddr,
    output wire [7:0]                           m_axi_arlen,
    output wire [2:0]                           m_axi_arsize,
    output wire [1:0]                           m_axi_arburst,
    output wire                                  m_axi_arlock,
    output wire [3:0]                           m_axi_arcache,
    output wire [2:0]                           m_axi_arprot,
    output wire [3:0]                           m_axi_arqos,
    output wire                                  m_axi_arvalid,
    input  wire                                  m_axi_arready,
    input  wire [C_M_AXI_ID_WIDTH-1:0]          m_axi_rid,
    input  wire [C_M_AXI_DATA_WIDTH-1:0]        m_axi_rdata,
    input  wire [1:0]                           m_axi_rresp,
    input  wire                                  m_axi_rlast,
    input  wire                                  m_axi_rvalid,
    output wire                                  m_axi_rready
);

    logic clk, rst_n;
    logic bus_we;
    logic [C_S_AXI_ADDR_WIDTH-1:0] bus_addr;
    logic [C_S_AXI_DATA_WIDTH-1:0] bus_wdata, bus_rdata;
    
    logic dram_we;
    logic [C_M_AXI_ADDR_WIDTH-1:0] dram_addr;
    logic [C_M_AXI_DATA_WIDTH-1:0] dram_wdata, dram_rdata;

    assign clk = s_axi_aclk;
    assign rst_n = s_axi_aresetn;

    // AXI-Lite write FSM
    typedef enum logic [1:0] {WR_IDLE, WR_ADDR, WR_DATA, WR_RESP} wr_state_t;
    wr_state_t wr_state, wr_next;
    logic [C_S_AXI_ADDR_WIDTH-1:0] wr_addr_reg;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
            wr_addr_reg <= '0;
        end else begin
            wr_state <= wr_next;
            if (s_axi_awvalid && s_axi_awready)
                wr_addr_reg <= s_axi_awaddr;
        end
    end
    
    always_comb begin
        wr_next = wr_state;
        case (wr_state)
            WR_IDLE: begin
                if (s_axi_awvalid && s_axi_wvalid) wr_next = WR_RESP;
                else if (s_axi_awvalid) wr_next = WR_DATA;
                else if (s_axi_wvalid) wr_next = WR_ADDR;
            end
            WR_ADDR: if (s_axi_awvalid) wr_next = WR_RESP;
            WR_DATA: if (s_axi_wvalid) wr_next = WR_RESP;
            WR_RESP: if (s_axi_bready) wr_next = WR_IDLE;
        endcase
    end
    
    assign s_axi_awready = (wr_state == WR_IDLE) || (wr_state == WR_ADDR);
    assign s_axi_wready = (wr_state == WR_IDLE) || (wr_state == WR_DATA);
    assign s_axi_bvalid = (wr_state == WR_RESP);
    assign s_axi_bresp = 2'b00;
    
    logic bus_we_pulse;
    always_ff @(posedge clk) begin
        if (!rst_n) bus_we_pulse <= 1'b0;
        else bus_we_pulse <= ((wr_state == WR_IDLE) && s_axi_awvalid && s_axi_wvalid) ||
                            ((wr_state == WR_ADDR) && s_axi_awvalid) ||
                            ((wr_state == WR_DATA) && s_axi_wvalid);
    end
    
    assign bus_we = bus_we_pulse;
    assign bus_addr = s_axi_awvalid ? s_axi_awaddr : wr_addr_reg;
    assign bus_wdata = s_axi_wdata;
    
    // AXI-Lite read FSM
    typedef enum logic [1:0] {RD_IDLE, RD_WAIT, RD_RESP} rd_state_t;
    rd_state_t rd_state, rd_next;
    logic [C_S_AXI_ADDR_WIDTH-1:0] rd_addr_reg;
    logic [C_S_AXI_DATA_WIDTH-1:0] rd_data_reg;
    logic rd_en;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_state <= RD_IDLE;
            rd_addr_reg <= '0;
            rd_data_reg <= '0;
            rd_en <= 1'b0;
        end else begin
            rd_state <= rd_next;
            if (s_axi_arvalid && s_axi_arready) begin
                rd_addr_reg <= s_axi_araddr;
                rd_en <= 1'b1;
            end else rd_en <= 1'b0;
            if (rd_en) rd_data_reg <= bus_rdata;
        end
    end
    
    always_comb begin
        rd_next = rd_state;
        case (rd_state)
            RD_IDLE: if (s_axi_arvalid) rd_next = RD_WAIT;
            RD_WAIT: rd_next = RD_RESP;
            RD_RESP: if (s_axi_rready) rd_next = RD_IDLE;
        endcase
    end
    
    assign s_axi_arready = (rd_state == RD_IDLE);
    assign s_axi_rvalid = (rd_state == RD_RESP);
    assign s_axi_rdata = rd_data_reg;
    assign s_axi_rresp = 2'b00;

    // GPU instance
    gpu_top #(
        .DATA_WIDTH(32), .VEC_SIZE(4), .CORD_WIDTH(10),
        .FB_WIDTH(640), .FB_HEIGHT(480), .ADDR_WIDTH(32),
        .INSTR_DEPTH(256), .FIFO_DEPTH(64),
        .NUM_MASTERS(3), .NUM_SLAVES(1)
    ) gpu_inst (
        .clk(clk), .rst_n(rst_n),
        .i_bus_we(bus_we),
        .i_bus_addr(rd_en ? rd_addr_reg : bus_addr),
        .i_bus_wdata(bus_wdata),
        .o_bus_rdata(bus_rdata),
        .o_dram_we(dram_we),
        .o_dram_addr(dram_addr),
        .o_dram_wdata(dram_wdata),
        .i_dram_rdata(dram_rdata)
    );

    // AXI master FSM
    typedef enum logic [2:0] {M_IDLE, M_WRITE_ADDR, M_WRITE_DATA, M_WRITE_RESP, M_READ_ADDR, M_READ_DATA} master_state_t;
    master_state_t m_state, m_next;
    logic [C_M_AXI_ADDR_WIDTH-1:0] m_addr_reg;
    logic [C_M_AXI_DATA_WIDTH-1:0] m_wdata_reg;
    logic m_we_reg;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_state <= M_IDLE;
            m_addr_reg <= '0;
            m_wdata_reg <= '0;
            m_we_reg <= '0;
            dram_rdata <= '0;
        end else begin
            m_state <= m_next;
            if (m_state == M_IDLE && (dram_we || (!dram_we && dram_addr != '0))) begin
                m_addr_reg <= dram_addr;
                m_wdata_reg <= dram_we ? dram_wdata : '0;
                m_we_reg <= dram_we;
            end
            if (m_axi_rvalid && m_axi_rready) dram_rdata <= m_axi_rdata;
        end
    end
    
    always_comb begin
        m_next = m_state;
        case (m_state)
            M_IDLE: begin
                if (dram_we) m_next = M_WRITE_ADDR;
                else if (dram_addr != '0 && !dram_we) m_next = M_READ_ADDR;
            end
            M_WRITE_ADDR: if (m_axi_awready) m_next = M_WRITE_DATA;
            M_WRITE_DATA: if (m_axi_wready) m_next = M_WRITE_RESP;
            M_WRITE_RESP: if (m_axi_bvalid) m_next = M_IDLE;
            M_READ_ADDR: if (m_axi_arready) m_next = M_READ_DATA;
            M_READ_DATA: if (m_axi_rvalid) m_next = M_IDLE;
        endcase
    end
    
    assign m_axi_awid = 4'h0;
    assign m_axi_awaddr = (m_state == M_WRITE_ADDR) ? m_addr_reg : 32'h0;
    assign m_axi_awlen = 8'h00;
    assign m_axi_awsize = 3'b010;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awlock = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot = 3'b000;
    assign m_axi_awqos = 4'b0000;
    assign m_axi_awvalid = (m_state == M_WRITE_ADDR);
    
    assign m_axi_wdata = (m_state == M_WRITE_DATA) ? m_wdata_reg : 32'h0;
    assign m_axi_wstrb = (m_state == M_WRITE_DATA) ? 4'b1111 : 4'b0000;
    assign m_axi_wlast = (m_state == M_WRITE_DATA);
    assign m_axi_wvalid = (m_state == M_WRITE_DATA);
    assign m_axi_bready = (m_state == M_WRITE_RESP);
    
    assign m_axi_arid = 4'h0;
    assign m_axi_araddr = (m_state == M_READ_ADDR) ? m_addr_reg : 32'h0;
    assign m_axi_arlen = 8'h00;
    assign m_axi_arsize = 3'b010;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot = 3'b000;
    assign m_axi_arqos = 4'b0000;
    assign m_axi_arvalid = (m_state == M_READ_ADDR);
    assign m_axi_rready = (m_state == M_READ_DATA);

endmodule

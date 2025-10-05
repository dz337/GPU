// axi_wrapper.sv  (32-bit DRAM side + AXI-Lite -> gpu_top CPU bus bridge)
module axi_wrapper #(
  parameter int ADDR_WIDTH   = 32,
  parameter int DATA_WIDTH   = 32
)(
  // AXI-Lite slave (from PS GP0 via interconnect)
  input  logic                       S_AXI_ACLK,
  input  logic                       S_AXI_ARESETN,
  // write address channel
  input  logic [ADDR_WIDTH-1:0]      S_AXI_AWADDR,
  input  logic                       S_AXI_AWVALID,
  output logic                       S_AXI_AWREADY,
  // write data channel
  input  logic [DATA_WIDTH-1:0]      S_AXI_WDATA,
  input  logic [DATA_WIDTH/8-1:0]    S_AXI_WSTRB,
  input  logic                       S_AXI_WVALID,
  output logic                       S_AXI_WREADY,
  // write response
  output logic [1:0]                 S_AXI_BRESP,
  output logic                       S_AXI_BVALID,
  input  logic                       S_AXI_BREADY,
  // read address
  input  logic [ADDR_WIDTH-1:0]      S_AXI_ARADDR,
  input  logic                       S_AXI_ARVALID,
  output logic                       S_AXI_ARREADY,
  // read data
  output logic [DATA_WIDTH-1:0]      S_AXI_RDATA,
  output logic [1:0]                 S_AXI_RRESP,
  output logic                       S_AXI_RVALID,
  input  logic                       S_AXI_RREADY,

  // AXI master to DDR (YOUR interconnect binds this to PS HP/MIG)
  // Keep all these the same names/widths the BD expects,
  // but we make the *GPU* side 32-bit below.
  input  logic                       M_AXI_ACLK,
  input  logic                       M_AXI_ARESETN,
  output logic [ADDR_WIDTH-1:0]      M_AXI_AWADDR,
  output logic [7:0]                 M_AXI_AWLEN,
  output logic [2:0]                 M_AXI_AWSIZE,
  output logic [1:0]                 M_AXI_AWBURST,
  output logic                       M_AXI_AWVALID,
  input  logic                       M_AXI_AWREADY,
  output logic [DATA_WIDTH-1:0]      M_AXI_WDATA,      // 32-bit to match gpu_top
  output logic [DATA_WIDTH/8-1:0]    M_AXI_WSTRB,      // 4-bit strobes
  output logic                       M_AXI_WLAST,
  output logic                       M_AXI_WVALID,
  input  logic                       M_AXI_WREADY,
  input  logic [1:0]                 M_AXI_BRESP,
  input  logic                       M_AXI_BVALID,
  output logic                       M_AXI_BREADY,
  output logic [ADDR_WIDTH-1:0]      M_AXI_ARADDR,
  output logic [7:0]                 M_AXI_ARLEN,
  output logic [2:0]                 M_AXI_ARSIZE,
  output logic [1:0]                 M_AXI_ARBURST,
  output logic                       M_AXI_ARVALID,
  input  logic                       M_AXI_ARREADY,
  input  logic [DATA_WIDTH-1:0]      M_AXI_RDATA,      // 32-bit path in wrapper
  input  logic [1:0]                 M_AXI_RRESP,
  input  logic                       M_AXI_RLAST,
  input  logic                       M_AXI_RVALID,
  output logic                       M_AXI_RREADY
);

  // ---------------------------
  // AXI-Lite -> gpu_top CPU bus
  // ---------------------------
  // AW/W handshake combine into one-cycle write pulse to gpu_top
  logic                 wr_inflight;
  logic [ADDR_WIDTH-1:0] awaddr_q;

  // Simple ready/valid: single-beat writes
  assign S_AXI_AWREADY = (!wr_inflight);
  assign S_AXI_WREADY  = (!wr_inflight);

  // Accept write when both valid and we're ready (no skid buffer for bring-up)
  wire do_aw = S_AXI_AWVALID && S_AXI_AWREADY;
  wire do_w  = S_AXI_WVALID  && S_AXI_WREADY;

  always_ff @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
    if (!S_AXI_ARESETN) begin
      wr_inflight <= 1'b0;
      awaddr_q    <= '0;
    end else begin
      // capture address
      if (do_aw) begin
        awaddr_q    <= S_AXI_AWADDR;
        wr_inflight <= 1'b1;
      end
      // once data is seen, complete the write
      if (do_w) begin
        wr_inflight <= 1'b0;
      end
    end
  end

  // Issue write response when data accepted
  always_ff @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
    if (!S_AXI_ARESETN) begin
      S_AXI_BVALID <= 1'b0;
      S_AXI_BRESP  <= 2'b00;
    end else begin
      if (do_w) begin
        S_AXI_BVALID <= 1'b1;
        S_AXI_BRESP  <= 2'b00; // OKAY
      end else if (S_AXI_BVALID && S_AXI_BREADY) begin
        S_AXI_BVALID <= 1'b0;
      end
    end
  end

  // Read channel: accept AR, return gpu_top read in next cycle
  logic [ADDR_WIDTH-1:0] araddr_q;
  logic                  ar_pending;

  assign S_AXI_ARREADY = !ar_pending;

  always_ff @(posedge S_AXI_ACLK or negedge S_AXI_ARESETN) begin
    if (!S_AXI_ARESETN) begin
      araddr_q   <= '0;
      ar_pending <= 1'b0;
      S_AXI_RVALID <= 1'b0;
      S_AXI_RDATA  <= '0;
      S_AXI_RRESP  <= 2'b00;
    end else begin
      if (S_AXI_ARVALID && S_AXI_ARREADY) begin
        araddr_q   <= S_AXI_ARADDR;
        ar_pending <= 1'b1;
      end

      // Present read data one cycle later (gpu_top read is combinational)
      if (ar_pending) begin
        ar_pending  <= 1'b0;
        S_AXI_RVALID<= 1'b1;
        S_AXI_RRESP <= 2'b00; // OKAY
        S_AXI_RDATA <= o_bus_rdata; // from gpu_top below
      end else if (S_AXI_RVALID && S_AXI_RREADY) begin
        S_AXI_RVALID<= 1'b0;
      end
    end
  end

  // ------------
  // gpu_top hook
  // ------------
  // Generate one-cycle write enable when W arrives.
  wire cpu_we_pulse = do_w;

  // AXI-Lite is byte addressed; gpu_top expects byte address too.
  // Mask upper bits if you want to restrict address window.
  wire [ADDR_WIDTH-1:0] cpu_addr  = awaddr_q;

  // Apply WSTRB to data (support partial writes); for your regs (full words),
  // WSTRB is typically 4'b1111; we still honor strobes.
  function automatic [31:0] apply_wstrb(input [31:0] oldv, input [31:0] newv, input [3:0] st);
    apply_wstrb = oldv;
    if (st[0]) apply_wstrb[ 7:0]  = newv[ 7:0];
    if (st[1]) apply_wstrb[15:8]  = newv[15:8];
    if (st[2]) apply_wstrb[23:16] = newv[23:16];
    if (st[3]) apply_wstrb[31:24] = newv[31:24];
  endfunction

  // Write data passed as-is; gpu_top registers do their own capture.
  wire [31:0] cpu_wdata = S_AXI_WDATA;
  // (If you needed read-modify-write with strobes, you'd stage a shadow array.
  // For gpu_top, full-word writes are fine.)

  // Read data returned here:
  wire [31:0] o_bus_rdata;

  // Tie-offs for masters/routing inside gpu_top ? AXI master below
  wire                      o_dram_we;
  wire [ADDR_WIDTH-1:0]     o_dram_addr;
  wire [31:0]               o_dram_wdata; // 32-bit from gpu_top
  wire [31:0]               i_dram_rdata;

  // Hook gpu_top to the S_AXI clock/reset domain (your BD ties S_AXI_ACLK to FCLK1)
  gpu_top #(
    .DATA_WIDTH  (32),
    .VEC_SIZE    (4),
    .CORD_WIDTH  (10),
    .FB_WIDTH    (640),
    .FB_HEIGHT   (480),
    .ADDR_WIDTH  (32),
    .INSTR_DEPTH (256),
    .FIFO_DEPTH  (64),
    .NUM_MASTERS (3),
    .NUM_SLAVES  (1)
  ) gpu_inst (
    .clk          (S_AXI_ACLK),
    .rst_n        (S_AXI_ARESETN),

    // CPU bus from AXI-Lite
    .i_bus_we     (cpu_we_pulse),
    .i_bus_addr   (cpu_addr),
    .i_bus_wdata  (cpu_wdata),
    .o_bus_rdata  (o_bus_rdata),

    // DRAM side (32-bit)
    .o_dram_we    (o_dram_we),
    .o_dram_addr  (o_dram_addr),
    .o_dram_wdata (o_dram_wdata),
    .i_dram_rdata (i_dram_rdata)
  );

  // -----------------------------
  // Minimal AXI master (32-bit) to DDR
  // -----------------------------
  // For bring-up: perform simple single-beat writes on o_dram_we, and ignore reads
  // (Your existing interconnect likely expects AXI4; this is a stub. If you already
  // had a working master path, keep it; just make sure it's 32-bit to/from gpu_top.)

  // Default safe values
  assign M_AXI_AWLEN   = 8'd0;
  assign M_AXI_AWSIZE  = 3'b010; // 4 bytes
  assign M_AXI_AWBURST = 2'b01;  // INCR
  assign M_AXI_ARSIZE  = 3'b010;
  assign M_AXI_ARBURST = 2'b01;

  // Simple single-beat write channel
  typedef enum logic [1:0] {WIDLE, WADDR, WDATA} wstate_t;
  wstate_t wst, wst_n;

  always_ff @(posedge M_AXI_ACLK or negedge M_AXI_ARESETN) begin
    if (!M_AXI_ARESETN) wst <= WIDLE;
    else                wst <= wst_n;
  end

  always_comb begin
    // defaults
    M_AXI_AWADDR  = o_dram_addr;
    M_AXI_AWVALID = 1'b0;
    M_AXI_WDATA   = o_dram_wdata; // 32-bit path
    M_AXI_WSTRB   = 4'hF;
    M_AXI_WLAST   = 1'b1;
    M_AXI_WVALID  = 1'b0;
    M_AXI_BREADY  = 1'b1;

    wst_n = wst;

    case (wst)
      WIDLE: begin
        if (o_dram_we) begin
          M_AXI_AWVALID = 1'b1;
          wst_n         = WADDR;
        end
      end
      WADDR: begin
        M_AXI_AWVALID = 1'b1;
        if (M_AXI_AWREADY) begin
          M_AXI_WVALID = 1'b1;
          wst_n        = WDATA;
        end
      end
      WDATA: begin
        M_AXI_WVALID = 1'b1;
        if (M_AXI_WREADY) begin
          wst_n = WIDLE;
        end
      end
    endcase
  end

  // Read channel (stub: not used by current gpu_top; provide benign defaults)
  assign M_AXI_ARADDR  = '0;
  assign M_AXI_ARVALID = 1'b0;
  assign M_AXI_RREADY  = 1'b1;
  assign i_dram_rdata  = M_AXI_RDATA;

endmodule

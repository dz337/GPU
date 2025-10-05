module gpu_top #(
  parameter int DATA_WIDTH   = 32,
  parameter int VEC_SIZE     = 4,
  parameter int CORD_WIDTH   = 10,
  parameter int FB_WIDTH     = 640,
  parameter int FB_HEIGHT    = 480,
  parameter int ADDR_WIDTH   = 32,
  parameter int INSTR_DEPTH  = 256,
  parameter int FIFO_DEPTH   = 64,
  parameter int NUM_MASTERS  = 3, // 0: vertex_fetch, 1: shader_core, 2: framebuffer
  parameter int NUM_SLAVES   = 1  // 0: main memory (DRAM)
)(
  input  logic                     clk,
  input  logic                     rst_n,

  // CPU bus (AXI-lite-ish) for control + shader loading
  input  logic                     i_bus_we,
  input  logic [ADDR_WIDTH-1:0]    i_bus_addr,
  input  logic [DATA_WIDTH-1:0]    i_bus_wdata,
  output logic [DATA_WIDTH-1:0]    o_bus_rdata,

  // DRAM interface (single slave)
  output logic                     o_dram_we,
  output logic [ADDR_WIDTH-1:0]    o_dram_addr,
  output logic [DATA_WIDTH-1:0]    o_dram_wdata,
  input  logic [DATA_WIDTH-1:0]    i_dram_rdata
);

  localparam int VERTEX_POS_BITS    = CORD_WIDTH * 2 * 3;
  localparam int VERTEX_COLOR_BITS  = DATA_WIDTH * 3;
  localparam int VERTEX_UV_BITS     = DATA_WIDTH * 2 * 3;
  localparam int VERTEX_TOTAL_BITS  = VERTEX_POS_BITS + VERTEX_COLOR_BITS + VERTEX_UV_BITS;
  localparam int FIFO_DATA_WIDTH    = (2 * CORD_WIDTH) + (3 * (2 * CORD_WIDTH + 1));
  localparam int INSTR_WIDTH        = 32;

  // CPU map
  localparam ADDR_CONTROL      = 32'h00000000;  // W: bit0=start
  localparam ADDR_STATUS       = 32'h00000004;  // R: {30'b0, irq, busy}
  localparam ADDR_VERTEX_BASE  = 32'h00000008;  // R/W
  localparam ADDR_VERTEX_COUNT = 32'h0000000C;  // R/W (low 16b)
  localparam ADDR_PC           = 32'h00000010;  // R/W
  localparam ADDR_FB_BASE      = 32'h00000014;  // R/W

  // Debug counters (R/O)
  localparam ADDR_FRAG_IN_CNT  = 32'h00000018;
  localparam ADDR_FIFO_POP_CNT = 32'h0000001C;
  localparam ADDR_PX_CNT       = 32'h00000020;
  localparam ADDR_FB_WR_CNT    = 32'h00000024;

  // Debug latched triangle/bbox/area (R/O)
  localparam ADDR_DBG_V0       = 32'h00000028; // {v0y(16), v0x(16)}
  localparam ADDR_DBG_V1       = 32'h0000002C; // {v1y, v1x}
  localparam ADDR_DBG_V2       = 32'h00000030; // {v2y, v2x}
  localparam ADDR_DBG_BBOX_MIN = 32'h00000034; // {min_y, min_x}
  localparam ADDR_DBG_BBOX_MAX = 32'h00000038; // {max_y, max_x}
  localparam ADDR_DBG_AREA     = 32'h0000003C; // signed area

  localparam ADDR_SHADER_BASE  = 32'h00001000; // shader mem window

  // ---------- helpers for safe packing (avoid out-of-range selects) ----------
  function automatic logic [15:0] pack16 (input logic signed [CORD_WIDTH-1:0] v);
    pack16 = { {(16-CORD_WIDTH){v[CORD_WIDTH-1]}}, v };
  endfunction
  localparam int AREA_WIDTH = (2*CORD_WIDTH+1);
  function automatic logic [31:0] pack_area32 (input logic signed [AREA_WIDTH-1:0] a);
    pack_area32 = { {(32-AREA_WIDTH){a[AREA_WIDTH-1]}}, a };
  endfunction
  // ---------------------------------------------------------------------------

  // Pipeline states
  typedef enum logic [2:0] {
    PIPE_IDLE,
    PIPE_FETCH_VERTEX,
    PIPE_EXECUTE_SHADER,
    PIPE_RASTERIZE,
    PIPE_FRAGMENT,
    PIPE_DONE
  } pipeline_state_t;

  // Reset sync
  logic glbl_rst_n;
  reset_sync reset_sync_inst (
    .clk    (clk),
    .arst_n (rst_n),
    .srst_n (glbl_rst_n)
  );

  // Control regs
  logic [ADDR_WIDTH-1:0] vertex_base_addr;
  logic [15:0]           vertex_count;
  logic [15:0]           current_vertex;
  logic [31:0]           shader_pc;
  logic [ADDR_WIDTH-1:0] fb_base_addr;

  // Pipeline control
  pipeline_state_t pipe_state, next_pipe_state;
  logic pipeline_busy;
  logic vertex_fetch_done;
  logic rasterizer_done;

  // Controller
  logic controller_start, controller_irq, pipeline_start; // pipeline_start unused but kept
  logic start_request;

  // Interconnect
  logic [NUM_MASTERS-1:0]                   master_req;
  logic [NUM_MASTERS-1:0]                   master_we;
  logic [NUM_MASTERS-1:0][ADDR_WIDTH-1:0]   master_addr;
  logic [NUM_MASTERS-1:0][DATA_WIDTH-1:0]   master_wdata;
  logic [NUM_MASTERS-1:0][DATA_WIDTH-1:0]   master_rdata;

  logic [NUM_SLAVES-1:0]                    slave_req;
  logic [NUM_SLAVES-1:0]                    slave_we;
  logic [NUM_SLAVES-1:0][ADDR_WIDTH-1:0]    slave_addr;
  logic [NUM_SLAVES-1:0][DATA_WIDTH-1:0]    slave_wdata;
  logic [NUM_SLAVES-1:0][DATA_WIDTH-1:0]    slave_rdata;

  // Shader
  logic [INSTR_WIDTH-1:0] shader_instruction, shader_host_rdata;
  logic shader_exec_en;
  logic [4:0] shader_opcode;
  logic [3:0] shader_rd, shader_rs1, shader_rs2;
  logic [11:0] shader_imm;

  // Vertex data
  logic [32*15-1:0]       vertex_data_full;
  logic [VERTEX_TOTAL_BITS-1:0] vertex_data;
  logic signed [CORD_WIDTH-1:0] v0x, v0y, v1x, v1y, v2x, v2y;
  logic [DATA_WIDTH-1:0]       v0_color, v1_color, v2_color;
  logic [DATA_WIDTH-1:0]       v0_u, v0_v, v1_u, v1_v, v2_u, v2_v;

  // Rasterizer I/F
  logic rast_start;
  logic rast_frag_valid;
  logic signed [CORD_WIDTH-1:0]   frag_x, frag_y;
  logic signed [(CORD_WIDTH*2):0] lambda0, lambda1, lambda2;

  // FIFO
  logic fifo_wr, fifo_rd, fifo_full, fifo_empty;
  logic [FIFO_DATA_WIDTH-1:0] fifo_wdata, fifo_rdata;

  // Fragment stage
  logic fs_in_valid, fs_ready;
  logic signed [CORD_WIDTH-1:0]    fs_in_x, fs_in_y;
  logic signed [(CORD_WIDTH*2):0]  fs_in_lambda0, fs_in_lambda1, fs_in_lambda2;
  logic [DATA_WIDTH-1:0]           interpolated_color, interpolated_u, interpolated_v;

  // Texture
  logic tex_req_valid, texel_valid;
  logic [DATA_WIDTH-1:0] tex_u, tex_v, texel_color_single;
  logic [VEC_SIZE-1:0][DATA_WIDTH-1:0] texel_color_vec;

  // Framebuffer
  logic pixel_we;
  logic signed [CORD_WIDTH-1:0] pixel_x, pixel_y;
  logic [VEC_SIZE-1:0][DATA_WIDTH-1:0] pixel_color;

  // ---- write regs ----------------------------------------------------------
  always_ff @(posedge clk or negedge glbl_rst_n) begin
    if (!glbl_rst_n) begin
      vertex_base_addr <= '0;
      vertex_count     <= '0;
      shader_pc        <= '0;
      fb_base_addr     <= '0;
      controller_start <= 1'b0;
      start_request    <= 1'b0;
    end else begin
      controller_start <= 1'b0; // pulse
      start_request    <= 1'b0;
      if (i_bus_we) begin
        unique case (i_bus_addr)
          ADDR_CONTROL:      if (i_bus_wdata[0]) begin controller_start <= 1'b1; start_request <= 1'b1; end
          ADDR_VERTEX_BASE:  vertex_base_addr <= i_bus_wdata;
          ADDR_VERTEX_COUNT: vertex_count     <= i_bus_wdata[15:0];
          ADDR_PC:           shader_pc        <= i_bus_wdata;
          ADDR_FB_BASE:      fb_base_addr     <= i_bus_wdata;
        endcase
      end
    end
  end

  // ---- CPU read mux (aka "CPU mux") ---------------------------------------
  // debug probe regs declared ahead so we can read them here
  logic signed [CORD_WIDTH-1:0] dbg_v0x, dbg_v0y, dbg_v1x, dbg_v1y, dbg_v2x, dbg_v2y;
  logic signed [CORD_WIDTH-1:0] dbg_bbox_min_x, dbg_bbox_min_y, dbg_bbox_max_x, dbg_bbox_max_y;
  logic signed [(CORD_WIDTH*2):0] dbg_tri_area;

  always_comb begin
    if (i_bus_addr >= ADDR_SHADER_BASE) begin
      o_bus_rdata = shader_host_rdata;
    end else begin
      o_bus_rdata = '0;
      unique case (i_bus_addr)
        ADDR_STATUS:       o_bus_rdata = {30'b0, controller_irq, pipeline_busy};
        ADDR_CONTROL:      o_bus_rdata = {31'b0, controller_start};
        ADDR_VERTEX_BASE:  o_bus_rdata = vertex_base_addr;
        ADDR_VERTEX_COUNT: o_bus_rdata = {16'b0, vertex_count};
        ADDR_PC:           o_bus_rdata = shader_pc;
        ADDR_FB_BASE:      o_bus_rdata = fb_base_addr;

        ADDR_FRAG_IN_CNT:  o_bus_rdata = frag_in_cnt;
        ADDR_FIFO_POP_CNT: o_bus_rdata = fifo_pop_cnt;
        ADDR_PX_CNT:       o_bus_rdata = px_cnt;
        ADDR_FB_WR_CNT:    o_bus_rdata = fb_wr_cnt;

        ADDR_DBG_V0:       o_bus_rdata = { pack16(dbg_v0y),        pack16(dbg_v0x)        };
        ADDR_DBG_V1:       o_bus_rdata = { pack16(dbg_v1y),        pack16(dbg_v1x)        };
        ADDR_DBG_V2:       o_bus_rdata = { pack16(dbg_v2y),        pack16(dbg_v2x)        };
        ADDR_DBG_BBOX_MIN: o_bus_rdata = { pack16(dbg_bbox_min_y), pack16(dbg_bbox_min_x) };
        ADDR_DBG_BBOX_MAX: o_bus_rdata = { pack16(dbg_bbox_max_y), pack16(dbg_bbox_max_x) };
        ADDR_DBG_AREA:     o_bus_rdata = pack_area32(dbg_tri_area);

        default: /* 0 */;
      endcase
    end
  end

  assign pipeline_busy = (pipe_state != PIPE_IDLE);

  // ---- pipeline FSM --------------------------------------------------------
  always_ff @(posedge clk or negedge glbl_rst_n) begin
    if (!glbl_rst_n) begin
      pipe_state     <= PIPE_IDLE;
      current_vertex <= '0;
    end else begin
      pipe_state <= next_pipe_state;
      if (pipe_state == PIPE_IDLE && controller_start) begin
        current_vertex <= '0;
      end else if (pipe_state == PIPE_FETCH_VERTEX && vertex_fetch_done) begin
        current_vertex <= current_vertex + 3;
      end
    end
  end

  always_comb begin
    next_pipe_state = pipe_state;
    unique case (pipe_state)
      PIPE_IDLE:           if (controller_start || start_request) next_pipe_state = PIPE_FETCH_VERTEX;
      PIPE_FETCH_VERTEX:   if (vertex_fetch_done)                  next_pipe_state = PIPE_EXECUTE_SHADER;
      PIPE_EXECUTE_SHADER:                                         next_pipe_state = PIPE_RASTERIZE;
      PIPE_RASTERIZE: begin
        if (rasterizer_done) begin
          if (current_vertex >= vertex_count) next_pipe_state = PIPE_DONE;
          else                                next_pipe_state = PIPE_FETCH_VERTEX;
        end
      end
      PIPE_DONE:           next_pipe_state = PIPE_IDLE;
      default:             next_pipe_state = PIPE_IDLE;
    endcase
  end

  // ---- controller / IRQ ----------------------------------------------------
  controller #(.QUEUE_DEPTH(16)) ctrl_inst (
    .clk              (clk),
    .rst_n            (glbl_rst_n),
    .i_bus_addr       (i_bus_addr),
    .i_bus_wdata      (i_bus_wdata),
    .i_bus_we         (i_bus_we && (i_bus_addr == ADDR_CONTROL || i_bus_addr == ADDR_STATUS)),
    .o_bus_rdata      (),
    .i_pipeline_busy  (pipeline_busy),
    .o_start_pipeline (pipeline_start),
    .o_irq            (controller_irq)
  );

  // ---- shader loader / decoder (kept for arch compatibility) ---------------
  shader_loader #(.INSTR_WIDTH(32), .INSTR_DEPTH(INSTR_DEPTH)) shader_loader_inst (
    .clk          (clk),
    .rst_n        (glbl_rst_n),
    .i_host_we    (i_bus_we && i_bus_addr >= ADDR_SHADER_BASE),
    .i_host_addr  (i_bus_addr[$clog2(INSTR_DEPTH)-1+2:2]),
    .i_host_wdata (i_bus_wdata),
    .o_host_rdata (shader_host_rdata),
    .i_gpu_addr   (shader_pc[$clog2(INSTR_DEPTH)-1:0]),
    .o_gpu_instr  (shader_instruction)
  );

  instruction_decoder decoder_inst (
    .i_instruction_word (shader_instruction),
    .o_opcode           (shader_opcode),
    .o_rd_addr          (shader_rd),
    .o_rs1_addr         (shader_rs1),
    .o_rs2_addr         (shader_rs2),
    .o_imm              (shader_imm)
  );

  assign shader_exec_en = (pipe_state == PIPE_EXECUTE_SHADER);

  shader_core #(.DATA_WIDTH(DATA_WIDTH), .VEC_SIZE(VEC_SIZE), .NUM_REGS(16)) core0 (
    .clk         (clk),
    .rst_n       (glbl_rst_n),
    .i_exec_en   (shader_exec_en),
    .i_opcode    (shader_opcode),
    .i_rd_addr   (shader_rd),
    .i_rs1_addr  (shader_rs1),
    .i_rs2_addr  (shader_rs2),
    .o_mem_req   (master_req[1]),
    .i_mem_ready (1'b1)
  );
  assign master_addr[1]  = '0;
  assign master_wdata[1] = '0;
  assign master_we[1]    = 1'b0;

  // ---- interconnect --------------------------------------------------------
  gpu_interconnect #(
    .NUM_MASTERS (NUM_MASTERS),
    .NUM_SLAVES  (NUM_SLAVES),
    .ADDR_WIDTH  (ADDR_WIDTH),
    .DATA_WIDTH  (DATA_WIDTH)
  ) interconnect_inst (
    .clk            (clk),
    .rst_n          (glbl_rst_n),
    .i_master_req   (master_req),
    .i_master_we    (master_we),
    .i_master_addr  (master_addr),
    .i_master_wdata (master_wdata),
    .o_master_rdata (master_rdata),
    .o_slave_req    (slave_req),
    .o_slave_we     (slave_we),
    .o_slave_addr   (slave_addr),
    .o_slave_wdata  (slave_wdata),
    .i_slave_rdata  (slave_rdata)
  );

  // DRAM hookup
  assign o_dram_we      = slave_we[0];
  assign o_dram_addr    = slave_addr[0];
  assign o_dram_wdata   = slave_wdata[0];
  assign slave_rdata[0] = i_dram_rdata;

  // ---- vertex fetch --------------------------------------------------------
  vertex_fetch #(
    .ATTR_WIDTH       (32),
    .ATTRS_PER_VERTEX (15),
    .ADDR_WIDTH       (ADDR_WIDTH)
  ) vf_inst (
    .clk            (clk),
    .rst_n          (glbl_rst_n),
    .i_start_fetch  (pipe_state == PIPE_FETCH_VERTEX),
    .i_base_addr    (vertex_base_addr),
    .i_vertex_index (current_vertex),
    .o_fetch_done   (vertex_fetch_done),
    .o_mem_req      (master_req[0]),
    .o_mem_addr     (master_addr[0]),
    .i_mem_ready    (1'b1),
    .i_mem_rdata    (master_rdata[0][31:0]), // single 32b word feed
    .o_vertex_data  (vertex_data_full)
  );
  assign master_wdata[0] = '0;
  assign master_we[0]    = 1'b0;

  // unpack layout (matches your smoke test)
  assign v0x      = vertex_data_full[0*32 +: 10];
  assign v0y      = vertex_data_full[1*32 +: 10];
  assign v1x      = vertex_data_full[2*32 +: 10];
  assign v1y      = vertex_data_full[3*32 +: 10];
  assign v2x      = vertex_data_full[4*32 +: 10];
  assign v2y      = vertex_data_full[5*32 +: 10];
  assign v0_color = vertex_data_full[6*32 +: 32];
  assign v1_color = vertex_data_full[7*32 +: 32];
  assign v2_color = vertex_data_full[8*32 +: 32];
  assign v0_u     = vertex_data_full[9*32 +: 32];
  assign v0_v     = vertex_data_full[10*32 +: 32];
  assign v1_u     = vertex_data_full[11*32 +: 32];
  assign v1_v     = vertex_data_full[12*32 +: 32];
  assign v2_u     = vertex_data_full[13*32 +: 32];
  assign v2_v     = vertex_data_full[14*32 +: 32];

  // ---- latch coords before rasterization ----------------------------------
  (* dont_touch = "true" *) logic signed [CORD_WIDTH-1:0]
    v0x_latched, v0y_latched, v1x_latched, v1y_latched, v2x_latched, v2y_latched;

  always_ff @(posedge clk) begin
    if (pipe_state == PIPE_EXECUTE_SHADER) begin
      v0x_latched <= v0x; v0y_latched <= v0y;
      v1x_latched <= v1x; v1y_latched <= v1y;
      v2x_latched <= v2x; v2y_latched <= v2y;
    end
  end

  // expose debug triangle/bbox/area during rasterization
  always_ff @(posedge clk or negedge glbl_rst_n) begin
    if (!glbl_rst_n) begin
      dbg_v0x <= '0; dbg_v0y <= '0; dbg_v1x <= '0; dbg_v1y <= '0; dbg_v2x <= '0; dbg_v2y <= '0;
      dbg_bbox_min_x <= '0; dbg_bbox_min_y <= '0; dbg_bbox_max_x <= '0; dbg_bbox_max_y <= '0;
      dbg_tri_area <= '0;
    end else if (pipe_state == PIPE_RASTERIZE) begin
      dbg_v0x <= v0x_latched; dbg_v0y <= v0y_latched;
      dbg_v1x <= v1x_latched; dbg_v1y <= v1y_latched;
      dbg_v2x <= v2x_latched; dbg_v2y <= v2y_latched;

      dbg_bbox_min_x <= (v0x_latched <= v1x_latched && v0x_latched <= v2x_latched) ? v0x_latched
                      : (v1x_latched <= v2x_latched) ? v1x_latched : v2x_latched;
      dbg_bbox_max_x <= (v0x_latched >= v1x_latched && v0x_latched >= v2x_latched) ? v0x_latched
                      : (v1x_latched >= v2x_latched) ? v1x_latched : v2x_latched;
      dbg_bbox_min_y <= (v0y_latched <= v1y_latched && v0y_latched <= v2y_latched) ? v0y_latched
                      : (v1y_latched <= v2y_latched) ? v1y_latched : v2y_latched;
      dbg_bbox_max_y <= (v0y_latched >= v1y_latched && v0y_latched >= v2y_latched) ? v0y_latched
                      : (v1y_latched >= v2y_latched) ? v1y_latched : v2y_latched;

      dbg_tri_area <= (v1x_latched - v0x_latched)*(v2y_latched - v0y_latched)
                    - (v1y_latched - v0y_latched)*(v2x_latched - v0x_latched);
    end
  end

  // ---- rasterizer ----------------------------------------------------------
  assign rast_start = (pipe_state == PIPE_RASTERIZE);

  rasterizer #(.CORD_WIDTH(CORD_WIDTH)) rast_inst (
    .clk              (clk),
    .rst_n            (glbl_rst_n),
    .i_start          (rast_start),
    .i_v0_x           (v0x_latched), .i_v0_y (v0y_latched),
    .i_v1_x           (v1x_latched), .i_v1_y (v1y_latched),
    .i_v2_x           (v2x_latched), .i_v2_y (v2y_latched),
    .o_fragment_valid (rast_frag_valid),
    .o_fragment_x     (frag_x),
    .o_fragment_y     (frag_y),
    .o_lambda0        (lambda0),
    .o_lambda1        (lambda1),
    .o_lambda2        (lambda2),
    .o_done           (rasterizer_done)
  );

  // ---- FIFO to fragment stage ---------------------------------------------
  assign fifo_wr    = rast_frag_valid;
  assign fifo_wdata = {frag_x, frag_y, lambda0, lambda1, lambda2};

  fifo #(.DATA_WIDTH(FIFO_DATA_WIDTH), .DEPTH(FIFO_DEPTH)) frag_fifo (
    .clk      (clk),
    .rst_n    (glbl_rst_n),
    .i_wr_en  (fifo_wr),
    .i_w_data (fifo_wdata),
    .i_rd_en  (fifo_rd),
    .o_r_data (fifo_rdata),
    .o_full   (fifo_full),
    .o_empty  (fifo_empty)
  );

  // Proper ready/valid: valid independent of ready; pop when both valid & ready
  assign fs_in_valid = !fifo_empty;
  assign fifo_rd     = fs_in_valid && fs_ready;

  assign {fs_in_x, fs_in_y, fs_in_lambda0, fs_in_lambda1, fs_in_lambda2} = fifo_rdata;

  // ---- interpolators + fragment shader + texture ---------------------------
  attribute_interpolator #(.ATTR_WIDTH(DATA_WIDTH), .WEIGHT_WIDTH(2*CORD_WIDTH+1)) color_interp (
    .i_attr0(v0_color), .i_attr1(v1_color), .i_attr2(v2_color),
    .i_lambda0(fs_in_lambda0), .i_lambda1(fs_in_lambda1), .i_lambda2(fs_in_lambda2),
    .o_interpolated_attr(interpolated_color)
  );

  attribute_interpolator #(.ATTR_WIDTH(DATA_WIDTH), .WEIGHT_WIDTH(2*CORD_WIDTH+1)) u_interp (
    .i_attr0(v0_u), .i_attr1(v1_u), .i_attr2(v2_u),
    .i_lambda0(fs_in_lambda0), .i_lambda1(fs_in_lambda1), .i_lambda2(fs_in_lambda2),
    .o_interpolated_attr(interpolated_u)
  );

  attribute_interpolator #(.ATTR_WIDTH(DATA_WIDTH), .WEIGHT_WIDTH(2*CORD_WIDTH+1)) v_interp (
    .i_attr0(v0_v), .i_attr1(v1_v), .i_attr2(v2_v),
    .i_lambda0(fs_in_lambda0), .i_lambda1(fs_in_lambda1), .i_lambda2(fs_in_lambda2),
    .o_interpolated_attr(interpolated_v)
  );

  fragment_shader #(.DATA_WIDTH(DATA_WIDTH), .VEC_SIZE(VEC_SIZE), .CORD_WIDTH(CORD_WIDTH)) fs_inst (
    .clk             (clk),
    .rst_n           (glbl_rst_n),
    .i_frag_valid    (fs_in_valid),
    .o_ready         (fs_ready),
    .i_frag_x        (fs_in_x),
    .i_frag_y        (fs_in_y),
    .i_frag_color    ({VEC_SIZE{interpolated_color}}),
    .i_frag_tex_coord({interpolated_v, interpolated_u}),
    .o_tex_req_valid (tex_req_valid),
    .o_tex_u_coord   (tex_u),
    .o_tex_v_coord   (tex_v),
    .i_texel_valid   (texel_valid),
    .i_texel_color   (texel_color_vec),
    .o_pixel_valid   (pixel_we),
    .o_pixel_x       (pixel_x),
    .o_pixel_y       (pixel_y),
    .o_pixel_color   (pixel_color)
  );

  texture_unit #(.TEX_WIDTH(256), .TEX_HEIGHT(256), .CORD_WIDTH(16), .DATA_WIDTH(DATA_WIDTH)) tex_unit (
    .clk          (clk),
    .rst_n        (glbl_rst_n),
    .i_req_valid  (tex_req_valid),
    .i_u_coord    (tex_u[15:0]),
    .i_v_coord    (tex_v[15:0]),
    .o_data_valid (texel_valid),
    .o_texel_color(texel_color_single)
  );

  assign texel_color_vec = {VEC_SIZE{texel_color_single}};

  // ---- framebuffer writer + boot test pixel --------------------------------
  logic [7:0] test_counter;
  logic       test_pixel_we;
  logic signed [CORD_WIDTH-1:0] pixel_x_mux, pixel_y_mux;
  logic [DATA_WIDTH-1:0]        pixel_color_mux;

  always_ff @(posedge clk or negedge glbl_rst_n) begin
    if (!glbl_rst_n) test_counter <= '0;
    else if (test_counter < 100) test_counter <= test_counter + 1;
  end
  assign test_pixel_we   = (test_counter == 50);
  assign pixel_x_mux     = test_pixel_we ? 10'd100 : pixel_x;
  assign pixel_y_mux     = test_pixel_we ? 10'd100 : pixel_y;
  assign pixel_color_mux = test_pixel_we ? 32'hFFFFFFFF : pixel_color[0];

  framebuffer #(
    .SCREEN_WIDTH (FB_WIDTH),
    .SCREEN_HEIGHT(FB_HEIGHT),
    .COLOR_WIDTH  (DATA_WIDTH),
    .ADDR_WIDTH   (ADDR_WIDTH)
  ) fb_inst (
    .clk           (clk),
    .rst_n         (glbl_rst_n),
    .i_fb_base_addr(fb_base_addr),
    .i_pixel_we    (test_pixel_we | pixel_we),
    .i_pixel_x     (pixel_x_mux),
    .i_pixel_y     (pixel_y_mux),
    .i_pixel_color (pixel_color_mux),
    .o_mem_req     (master_req[2]),
    .o_mem_addr    (master_addr[2]),
    .o_mem_wdata   (master_wdata[2])
  );

  assign master_we[2] = master_req[2]; // FB writes when requesting

  // ---- debug counters ------------------------------------------------------
  logic [31:0] frag_in_cnt, fifo_pop_cnt, px_cnt, fb_wr_cnt;
  always_ff @(posedge clk or negedge glbl_rst_n) begin
    if (!glbl_rst_n) begin
      frag_in_cnt <= '0; fifo_pop_cnt <= '0; px_cnt <= '0; fb_wr_cnt <= '0;
    end else begin
      if (rast_frag_valid) frag_in_cnt <= frag_in_cnt + 1;
      if (fifo_rd)         fifo_pop_cnt <= fifo_pop_cnt + 1;
      if (pixel_we)        px_cnt <= px_cnt + 1;
      if (master_req[2])   fb_wr_cnt <= fb_wr_cnt + 1;
    end
  end

endmodule

module vertex_fetch #(
    parameter int ATTR_WIDTH = 32,
    parameter int ATTRS_PER_VERTEX = 15,
    parameter int ADDR_WIDTH = 32
) (
    input logic clk,
    input logic rst_n,
    input logic i_start_fetch,
    input logic [ADDR_WIDTH-1:0] i_base_addr,
    input logic [15:0] i_vertex_index,
    output logic o_fetch_done,
    output logic o_mem_req,
    output logic [ADDR_WIDTH-1:0] o_mem_addr,
    input logic i_mem_ready,
    input logic [ATTR_WIDTH-1:0] i_mem_rdata,
    output logic [ATTR_WIDTH*ATTRS_PER_VERTEX-1:0] o_vertex_data
);

  typedef enum logic [1:0] {IDLE, READING, DONE} state_t;
  state_t state;
  logic [ATTR_WIDTH*ATTRS_PER_VERTEX-1:0] data_reg;
  logic [4:0] word_count;
  logic [ADDR_WIDTH-1:0] base_addr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= IDLE;
      data_reg <= '0;
      word_count <= 0;
      base_addr <= '0;
    end else begin
      case (state)
        IDLE: begin
          if (i_start_fetch) begin
            state <= READING;
            word_count <= 0;
            data_reg <= '0;
            base_addr <= i_base_addr + (i_vertex_index * ATTRS_PER_VERTEX * 4);
          end
        end
        
        READING: begin
          if (i_mem_ready) begin
            data_reg[word_count*32 +: 32] <= i_mem_rdata;
            // FIXED: Check against ATTRS_PER_VERTEX-1 before incrementing
            if (word_count == ATTRS_PER_VERTEX-1) begin
              state <= DONE;
            end
            word_count <= word_count + 1;
          end
        end
        
        DONE: begin
          state <= IDLE;
        end
      endcase
    end
  end

  assign o_mem_req = (state == READING);
  assign o_mem_addr = base_addr + (word_count * 4);
  assign o_vertex_data = data_reg;
  assign o_fetch_done = (state == DONE);

endmodule

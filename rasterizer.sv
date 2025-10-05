module rasterizer #(
    parameter int CORD_WIDTH = 10
) (
    input logic clk,
    input logic rst_n,
    input logic i_start,
    input logic [CORD_WIDTH-1:0] i_v0_x,  // FIXED: unsigned
    input logic [CORD_WIDTH-1:0] i_v0_y,
    input logic [CORD_WIDTH-1:0] i_v1_x,
    input logic [CORD_WIDTH-1:0] i_v1_y,
    input logic [CORD_WIDTH-1:0] i_v2_x,
    input logic [CORD_WIDTH-1:0] i_v2_y,
    output logic o_fragment_valid,
    output logic [CORD_WIDTH-1:0] o_fragment_x,
    output logic [CORD_WIDTH-1:0] o_fragment_y,
    output logic signed [(CORD_WIDTH*2):0] o_lambda0,
    output logic signed [(CORD_WIDTH*2):0] o_lambda1,
    output logic signed [(CORD_WIDTH*2):0] o_lambda2,
    output logic o_done
);

  logic is_running;
  logic [CORD_WIDTH-1:0] v0_x_reg, v0_y_reg, v1_x_reg, v1_y_reg, v2_x_reg, v2_y_reg;
  logic [CORD_WIDTH-1:0] x_reg, y_reg;
  logic [CORD_WIDTH-1:0] bbox_min_x_reg, bbox_min_y_reg, bbox_max_x_reg, bbox_max_y_reg;

  // FIXED: Edge math in wider signed domain with zero-extension
  logic signed [(CORD_WIDTH+1):0] sx, sy, sv0x, sv0y, sv1x, sv1y, sv2x, sv2y;
  assign sx = $signed({1'b0, x_reg});
  assign sy = $signed({1'b0, y_reg});
  assign sv0x = $signed({1'b0, v0_x_reg});
  assign sv0y = $signed({1'b0, v0_y_reg});
  assign sv1x = $signed({1'b0, v1_x_reg});
  assign sv1y = $signed({1'b0, v1_y_reg});
  assign sv2x = $signed({1'b0, v2_x_reg});
  assign sv2y = $signed({1'b0, v2_y_reg});

  logic signed [(CORD_WIDTH*2+2):0] edge0, edge1, edge2;
  assign edge0 = (sx - sv1x) * (sv2y - sv1y) - (sy - sv1y) * (sv2x - sv1x);
  assign edge1 = (sx - sv2x) * (sv0y - sv2y) - (sy - sv2y) * (sv0x - sv2x);
  assign edge2 = (sx - sv0x) * (sv1y - sv0y) - (sy - sv0y) * (sv1x - sv0x);

  logic is_inside;
  assign is_inside = ((edge0 >= 0) && (edge1 >= 0) && (edge2 >= 0)) ||
                     ((edge0 <= 0) && (edge1 <= 0) && (edge2 <= 0));

  assign o_lambda0 = edge0;
  assign o_lambda1 = edge1;
  assign o_lambda2 = edge2;

  // FIXED: Only trigger on i_start rising edge
  logic was_idle;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      was_idle <= 1'b1;
    end else begin
      was_idle <= !is_running;
    end
  end
  
  wire start_trigger = i_start && was_idle;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      is_running <= 1'b0;
      x_reg <= '0; y_reg <= '0;
      v0_x_reg <= '0; v0_y_reg <= '0;
      v1_x_reg <= '0; v1_y_reg <= '0;
      v2_x_reg <= '0; v2_y_reg <= '0;
      bbox_min_x_reg <= '0; bbox_min_y_reg <= '0;
      bbox_max_x_reg <= '0; bbox_max_y_reg <= '0;
    end else begin
      if (start_trigger) begin
        is_running <= 1'b1;
        v0_x_reg <= i_v0_x; v0_y_reg <= i_v0_y;
        v1_x_reg <= i_v1_x; v1_y_reg <= i_v1_y;
        v2_x_reg <= i_v2_x; v2_y_reg <= i_v2_y;

        if (i_v0_x <= i_v1_x && i_v0_x <= i_v2_x) begin
          bbox_min_x_reg <= i_v0_x; x_reg <= i_v0_x;
        end else if (i_v1_x <= i_v2_x) begin
          bbox_min_x_reg <= i_v1_x; x_reg <= i_v1_x;
        end else begin
          bbox_min_x_reg <= i_v2_x; x_reg <= i_v2_x;
        end
        
        if (i_v0_x >= i_v1_x && i_v0_x >= i_v2_x) bbox_max_x_reg <= i_v0_x;
        else if (i_v1_x >= i_v2_x) bbox_max_x_reg <= i_v1_x;
        else bbox_max_x_reg <= i_v2_x;
        
        if (i_v0_y <= i_v1_y && i_v0_y <= i_v2_y) begin
          bbox_min_y_reg <= i_v0_y; y_reg <= i_v0_y;
        end else if (i_v1_y <= i_v2_y) begin
          bbox_min_y_reg <= i_v1_y; y_reg <= i_v1_y;
        end else begin
          bbox_min_y_reg <= i_v2_y; y_reg <= i_v2_y;
        end
        
        if (i_v0_y >= i_v1_y && i_v0_y >= i_v2_y) bbox_max_y_reg <= i_v0_y;
        else if (i_v1_y >= i_v2_y) bbox_max_y_reg <= i_v1_y;
        else bbox_max_y_reg <= i_v2_y;

      end else if (is_running) begin
        if (x_reg >= bbox_max_x_reg) begin
          x_reg <= bbox_min_x_reg;
          if (y_reg >= bbox_max_y_reg) begin
            is_running <= 1'b0;
          end else begin
            y_reg <= y_reg + 1;
          end
        end else begin
          x_reg <= x_reg + 1;
        end
      end
    end
  end

  assign o_fragment_valid = is_running && is_inside;
  assign o_fragment_x = x_reg;
  assign o_fragment_y = y_reg;
  assign o_done = !is_running;

endmodule

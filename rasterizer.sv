module rasterizer #(
    parameter int CORD_WIDTH = 10
) (
    input  logic clk,
    input  logic rst_n,

    // One-shot start
    input  logic i_start,

    // Triangle vertices (signed pixel coords)
    input  logic signed [CORD_WIDTH-1:0] i_v0_x,
    input  logic signed [CORD_WIDTH-1:0] i_v0_y,
    input  logic signed [CORD_WIDTH-1:0] i_v1_x,
    input  logic signed [CORD_WIDTH-1:0] i_v1_y,
    input  logic signed [CORD_WIDTH-1:0] i_v2_x,
    input  logic signed [CORD_WIDTH-1:0] i_v2_y,

    // Covered pixel stream
    output logic                         o_fragment_valid,
    output logic signed [CORD_WIDTH-1:0] o_fragment_x,
    output logic signed [CORD_WIDTH-1:0] o_fragment_y,

    // Barycentric numerators (edge-function values)
    output logic signed [(CORD_WIDTH*2):0] o_lambda0,
    output logic signed [(CORD_WIDTH*2):0] o_lambda1,
    output logic signed [(CORD_WIDTH*2):0] o_lambda2,

    output logic o_done
);

  // --- State/latches -------------------------------------------------------
  logic is_running;

  logic signed [CORD_WIDTH-1:0] v0_x_reg, v0_y_reg;
  logic signed [CORD_WIDTH-1:0] v1_x_reg, v1_y_reg;
  logic signed [CORD_WIDTH-1:0] v2_x_reg, v2_y_reg;

  logic signed [CORD_WIDTH-1:0] x_reg, y_reg;
  logic signed [CORD_WIDTH-1:0] bbox_min_x_reg, bbox_min_y_reg;
  logic signed [CORD_WIDTH-1:0] bbox_max_x_reg, bbox_max_y_reg;

  // Signed area to reject degenerate triangles
  logic signed [(CORD_WIDTH*2):0] tri_area;

  // --- Edge functions (combinational) --------------------------------------
  logic signed [(CORD_WIDTH*2):0] edge0, edge1, edge2;

  assign edge0 = (x_reg - v1_x_reg) * (v2_y_reg - v1_y_reg)
               - (y_reg - v1_y_reg) * (v2_x_reg - v1_x_reg);

  assign edge1 = (x_reg - v2_x_reg) * (v0_y_reg - v2_y_reg)
               - (y_reg - v2_y_reg) * (v0_x_reg - v2_x_reg);

  assign edge2 = (x_reg - v0_x_reg) * (v1_y_reg - v0_y_reg)
               - (y_reg - v0_y_reg) * (v1_x_reg - v0_x_reg);

  // Accept either winding: inside if all >=0 or all <=0
  wire is_inside = ((edge0 >= 0) && (edge1 >= 0) && (edge2 >= 0)) ||
                   ((edge0 <= 0) && (edge1 <= 0) && (edge2 <= 0));

  // Bbox guard (kept explicit so tools don't "optimize" scan edges away)
  wire in_bbox = (x_reg >= bbox_min_x_reg) && (x_reg <= bbox_max_x_reg) &&
                 (y_reg >= bbox_min_y_reg) && (y_reg <= bbox_max_y_reg);

  // Expose edge numerators as "barycentric" outputs
  assign o_lambda0 = edge0;
  assign o_lambda1 = edge1;
  assign o_lambda2 = edge2;

  // --- Row-major scan FSM ---------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      is_running       <= 1'b0;
      x_reg            <= '0;
      y_reg            <= '0;
      v0_x_reg         <= '0; v0_y_reg <= '0;
      v1_x_reg         <= '0; v1_y_reg <= '0;
      v2_x_reg         <= '0; v2_y_reg <= '0;
      bbox_min_x_reg   <= '0; bbox_min_y_reg <= '0;
      bbox_max_x_reg   <= '0; bbox_max_y_reg <= '0;
      tri_area         <= '0;
    end else begin
      if (i_start && !is_running) begin
        is_running <= 1'b1;

        // latch verts
        v0_x_reg <= i_v0_x; v0_y_reg <= i_v0_y;
        v1_x_reg <= i_v1_x; v1_y_reg <= i_v1_y;
        v2_x_reg <= i_v2_x; v2_y_reg <= i_v2_y;

        // bbox
        bbox_min_x_reg <= (i_v0_x <= i_v1_x && i_v0_x <= i_v2_x) ? i_v0_x
                          : (i_v1_x <= i_v2_x) ? i_v1_x : i_v2_x;
        bbox_max_x_reg <= (i_v0_x >= i_v1_x && i_v0_x >= i_v2_x) ? i_v0_x
                          : (i_v1_x >= i_v2_x) ? i_v1_x : i_v2_x;

        bbox_min_y_reg <= (i_v0_y <= i_v1_y && i_v0_y <= i_v2_y) ? i_v0_y
                          : (i_v1_y <= i_v2_y) ? i_v1_y : i_v2_y;
        bbox_max_y_reg <= (i_v0_y >= i_v1_y && i_v0_y >= i_v2_y) ? i_v0_y
                          : (i_v1_y >= i_v2_y) ? i_v1_y : i_v2_y;

        // start at bbox min corner
        x_reg <= (i_v0_x <= i_v1_x && i_v0_x <= i_v2_x) ? i_v0_x
               : (i_v1_x <= i_v2_x) ? i_v1_x : i_v2_x;
        y_reg <= (i_v0_y <= i_v1_y && i_v0_y <= i_v2_y) ? i_v0_y
               : (i_v1_y <= i_v2_y) ? i_v1_y : i_v2_y;

        tri_area <= (i_v1_x - i_v0_x)*(i_v2_y - i_v0_y)
                  - (i_v1_y - i_v0_y)*(i_v2_x - i_v0_x);

      end else if (is_running) begin
        if (tri_area == 0) begin
          is_running <= 1'b0; // degenerate triangle
        end else begin
          // inclusive scan of bbox
          if (x_reg >= bbox_max_x_reg) begin
            x_reg <= bbox_min_x_reg;
            if (y_reg >= bbox_max_y_reg) begin
              is_running <= 1'b0; // done last pixel
            end else begin
              y_reg <= y_reg + 1;
            end
          end else begin
            x_reg <= x_reg + 1;
          end
        end
      end
    end
  end

  // outputs
  assign o_fragment_valid = is_running && in_bbox && is_inside;
  assign o_fragment_x     = x_reg;
  assign o_fragment_y     = y_reg;
  assign o_done           = !is_running;

endmodule

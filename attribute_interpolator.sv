// attribute_interpolator.sv - FIXED barycentric interpolation
module attribute_interpolator #(
    parameter int ATTR_WIDTH   = 32,
    parameter int WEIGHT_WIDTH = 21
) (
    input logic signed [ATTR_WIDTH-1:0] i_attr0,
    input logic signed [ATTR_WIDTH-1:0] i_attr1,
    input logic signed [ATTR_WIDTH-1:0] i_attr2,
    input logic signed [WEIGHT_WIDTH-1:0] i_lambda0,
    input logic signed [WEIGHT_WIDTH-1:0] i_lambda1,
    input logic signed [WEIGHT_WIDTH-1:0] i_lambda2,
    output logic signed [ATTR_WIDTH-1:0] o_interpolated_attr
);

    // Proper barycentric interpolation: (attr0*0 + attr1*1 + attr2*2) / (0+1+2)
    logic signed [ATTR_WIDTH+WEIGHT_WIDTH-1:0] term0, term1, term2;
    logic signed [ATTR_WIDTH+WEIGHT_WIDTH+1:0] numerator;
    logic signed [WEIGHT_WIDTH+1:0] denominator;
    
    assign term0 = i_attr0 * i_lambda0;
    assign term1 = i_attr1 * i_lambda1;
    assign term2 = i_attr2 * i_lambda2;
    assign numerator = term0 + term1 + term2;
    assign denominator = i_lambda0 + i_lambda1 + i_lambda2;
    
    // Proper division instead of random bit shift
    assign o_interpolated_attr = (denominator != 0) ? (numerator / denominator) : '0;

endmodule

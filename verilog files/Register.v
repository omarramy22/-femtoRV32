module Register #(parameter n = 8)(
    input clk,
    input load,
    input rst,
    input [n-1:0] D,
    output [n-1:0] Q
);
    wire [n-1:0] ans;
    genvar i;

    generate
        for (i = 0; i < n; i = i + 1) begin
            assign ans[i] = (load == 0) ? Q[i] : D[i];
            DFlipFlop df(.clk(clk), .rst(rst), .D(ans[i]), .Q(Q[i]));
        end
    endgenerate

endmodule

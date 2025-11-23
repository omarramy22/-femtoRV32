module Branch_Unit (
    input  wire [2:0] funct3,
    input  wire        Zero,
    input  wire        LessThan,
    input  wire        LessThanUnsigned,
    output reg         branch_taken
);

always @(*) begin
    case (funct3)
        3'b000: branch_taken = Zero;
        3'b001: branch_taken = ~Zero;
        3'b100: branch_taken = LessThan;
        3'b101: branch_taken = ~LessThan;
        3'b110: branch_taken = LessThanUnsigned;
        3'b111: branch_taken = ~LessThanUnsigned;
        default: branch_taken = 1'b0;
    endcase
end

endmodule

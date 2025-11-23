module NextPC (
    input  wire [31:0] pc_plus_4,
    input  wire [31:0] branch_target,
    input  wire [31:0] jal_target,
    input  wire [31:0] jalr_target,
    input  wire        branch_taken,
    input  wire        Jump,
    input  wire        JumpR,
    output reg  [31:0] next_pc
);

always @(*) begin
    if (JumpR)
        next_pc = jalr_target;
    else if (Jump)
        next_pc = jal_target;
    else if (branch_taken)
        next_pc = branch_target;
    else
        next_pc = pc_plus_4;
end

endmodule

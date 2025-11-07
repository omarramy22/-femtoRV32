`timescale 1ns / 1ps
`include "defines.v"
//////////////////////////////////////////////////////////////////////////////////
// Module: N_bit_ALU
// Description: Behavioral ALU for RV32I using macros for operation codes
//////////////////////////////////////////////////////////////////////////////////

module N_bit_ALU #(parameter N = 32) (
    input  wire [N-1:0] A,
    input  wire [N-1:0] B,
    input  wire [3:0]   Sel,      // operation selector (from defines.vh)
    output reg  [N-1:0] Out,
    output wire Zero,
    output wire LessThan,
    output wire LessThanUnsigned
);

    wire signed [N-1:0] sA = A;
    wire signed [N-1:0] sB = B;

    wire [N-1:0] and_res  = A & B;
    wire [N-1:0] or_res   = A | B;
    wire [N-1:0] xor_res  = A ^ B;
    wire [N-1:0] add_res  = A + B;
    wire [N-1:0] sub_res  = A - B;
    wire [N-1:0] sll_res  = A << B[4:0];
    wire [N-1:0] srl_res  = A >> B[4:0];
    wire [N-1:0] sra_res  = $signed(A) >>> B[4:0];
    wire [N-1:0] slt_res  = (sA < sB) ? 32'd1 : 32'd0;
    wire [N-1:0] sltu_res = (A < B)   ? 32'd1 : 32'd0;

    always @(*) begin
        case (Sel)
            `ALU_ADD:   Out = add_res;          // ADD
            `ALU_SUB:   Out = sub_res;          // SUB
            `ALU_PASS:  Out = A;                // PASS (forward A)
            `ALU_OR:    Out = or_res;           // OR
            `ALU_AND:   Out = and_res;          // AND
            `ALU_XOR:   Out = xor_res;          // XOR
            `ALU_SRL:   Out = srl_res;          // SRL
            `ALU_SRA:   Out = sra_res;          // SRA
            `ALU_SLL:   Out = sll_res;          // SLL
            `ALU_SLT:   Out = slt_res;          // SLT (signed)
            `ALU_SLTU:  Out = sltu_res;         // SLTU (unsigned)
            default:    Out = {N{1'b0}};
        endcase
    end

    assign Zero = (Out == {N{1'b0}});
    assign LessThan = (sA < sB);
    assign LessThanUnsigned = (A < B);

endmodule

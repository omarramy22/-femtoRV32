`timescale 1ns / 1ps
`include "defines.v"

module N_bit_ALU #(parameter N = 32) (
    input  wire [N-1:0] A,
    input  wire [N-1:0] B,
    input  wire [3:0]   Sel,
    output reg  [N-1:0] Out,
    output wire Zero,
    output wire LessThan,
    output wire LessThanUnsigned
);

    wire signed [N-1:0] sA = A;
    wire signed [N-1:0] sB = B;

    always @(*) begin
        case (Sel)

            `ALU_ADD:   Out = A + B;
            `ALU_SUB:   Out = A - B;
            `ALU_AND:   Out = A & B;
            `ALU_OR:    Out = A | B;
            `ALU_XOR:   Out = A ^ B;

            `ALU_SLL:   Out = A <<  B[4:0];
            `ALU_SRL:   Out = A >>  B[4:0];
            `ALU_SRA:   Out = sA >>> B[4:0];

            `ALU_SLT:   Out = (sA < sB) ? 32'd1 : 32'd0;
            `ALU_SLTU:  Out = (A  < B ) ? 32'd1 : 32'd0;

            `ALU_PASS:  Out = A;

            default:    Out = {N{1'b0}};

        endcase
    end

    assign Zero              = (Out == 0);
    assign LessThan          = (sA < sB);
    assign LessThanUnsigned  = (A  < B);

endmodule

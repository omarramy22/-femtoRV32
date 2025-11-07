`timescale 1ns / 1ps
`include "defines.v"

module ALU_Control (
    input  wire [1:0] ALUOp,       // from main control unit
    input  wire [2:0] funct3,      // instruction[14:12]
    input  wire [6:0] funct7,      // instruction[31:25]
    output reg  [3:0] ALU_Sel      // to ALU Sel input
);

always @(*) begin
    case (ALUOp)
        2'b00: ALU_Sel = `ALU_ADD; // For loads, stores, and AUIPC (always ADD)
        2'b01: ALU_Sel = `ALU_SUB; // For branch comparisons (BEQ, etc.)
        2'b10: begin                // For R-type and I-type arithmetic
            case (funct3)
                3'b000: ALU_Sel = (funct7[5]) ? `ALU_SUB : `ALU_ADD; // ADD/SUB
                3'b111: ALU_Sel = `ALU_AND;  // AND
                3'b110: ALU_Sel = `ALU_OR;   // OR
                3'b100: ALU_Sel = `ALU_XOR;  // XOR
                3'b001: ALU_Sel = `ALU_SLL;  // SLL
                3'b101: ALU_Sel = (funct7[5]) ? `ALU_SRA : `ALU_SRL; // SRA/SRL
                3'b010: ALU_Sel = `ALU_SLT;  // SLT
                3'b011: ALU_Sel = `ALU_SLTU; // SLTU
                default: ALU_Sel = `ALU_PASS;
            endcase
        end
        default: ALU_Sel = `ALU_PASS;
    endcase
end

endmodule

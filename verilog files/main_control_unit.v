`timescale 1ns / 1ps
`include "defines.v"

module Main_Control (
    input  wire [4:0] opcode,    
    output reg        Branch,
    output reg        MemRead,
    output reg        MemToReg,
    output reg  [1:0] ALUOp,
    output reg        MemWrite,
    output reg        ALUSrc,
    output reg        RegWrite,
    output reg        Jump
);

always @(*) begin
    Branch    = 0;
    MemRead   = 0;
    MemToReg  = 0;
    ALUOp     = 2'b00;
    MemWrite  = 0;
    ALUSrc    = 0;
    RegWrite  = 0;
    Jump      = 0;

    case (opcode)
        `OPCODE_Arith_R: begin
            RegWrite = 1;
            ALUOp    = 2'b10;
        end

        `OPCODE_Arith_I: begin
            RegWrite = 1;
            ALUSrc   = 1;
            ALUOp    = 2'b10;
        end

        `OPCODE_Load: begin
            RegWrite = 1;
            ALUSrc   = 1;
            MemToReg = 1;
            MemRead  = 1;
            ALUOp    = 2'b00;
        end

        `OPCODE_Store: begin
            ALUSrc   = 1;
            MemWrite = 1;
            ALUOp    = 2'b00;
        end

        `OPCODE_Branch: begin
            Branch   = 1;
            ALUOp    = 2'b01;  
        end

        `OPCODE_LUI: begin
            RegWrite = 1;
            ALUSrc   = 1;
            ALUOp    = 2'b00;
        end

        `OPCODE_AUIPC: begin
            RegWrite = 1;
            ALUSrc   = 1;
            ALUOp    = 2'b00;
        end

        `OPCODE_JAL: begin
            RegWrite = 1;
            Jump     = 1;
            ALUSrc   = 1;
        end

        `OPCODE_JALR: begin
            RegWrite = 1;
            Jump     = 1;
            ALUSrc   = 1;
        end

        default: begin
        end
    endcase
end

endmodule

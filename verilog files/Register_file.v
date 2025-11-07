`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/07/2025 06:01:59 PM
// Design Name: 
// Module Name: Register_file
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module register_file #(parameter N = 32) (
    input clk,
    input rst,
    input RegWrite,
    input [4:0] ReadReg1,
    input [4:0] ReadReg2,
    input [4:0] WriteReg,
    input [N-1:0] WriteData,
    output [N-1:0] ReadData1,
    output [N-1:0] ReadData2
);
    reg [N-1:0] regs [31:0];
    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) regs[i] <= {N{1'b0}};
        end else begin
            if (RegWrite && (WriteReg != 5'b00000)) regs[WriteReg] <= WriteData;
        end
    end
    assign ReadData1 = (ReadReg1 == 5'b00000) ? {N{1'b0}} : regs[ReadReg1];
    assign ReadData2 = (ReadReg2 == 5'b00000) ? {N{1'b0}} : regs[ReadReg2];
endmodule


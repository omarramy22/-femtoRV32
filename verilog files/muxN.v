`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/07/2025 05:41:54 PM
// Design Name: 
// Module Name: muxN
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


module muxN #(parameter N=32)(input [N-1:0] a, input [N-1:0] b, input sel, output [N-1:0] y);
    genvar i;
        generate
            for(i=0;i<N;i=i+1) 
                begin: loop
                    mux_2_1 m(a[i],b[i],sel,y[i]);
                end
        endgenerate
endmodule
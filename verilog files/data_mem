`timescale 1ns / 1ps

module DataMem (
    input  wire        clk,
    input  wire        MemRead,
    input  wire        MemWrite,
    input  wire [31:0] addr,       
    input  wire [31:0] data_in,    
    input  wire [2:0]  funct3,     
    output reg  [31:0] data_out    
);

    reg [7:0] mem [0:4095];
    integer i;

    
    always @(posedge clk) begin
        if (MemWrite) begin
            case (funct3)
                3'b000: begin 
                    mem[addr] <= data_in[7:0];
                end
                3'b001: begin 
                    mem[addr]     <= data_in[7:0];
                    mem[addr + 1] <= data_in[15:8];
                end
                3'b010: begin 
                    mem[addr]     <= data_in[7:0];
                    mem[addr + 1] <= data_in[15:8];
                    mem[addr + 2] <= data_in[23:16];
                    mem[addr + 3] <= data_in[31:24];
                end
                default: ; // ignore others
            endcase
        end
    end

    always @(*) begin
        if (MemRead) begin
            case (funct3)
                3'b000: 
                    data_out = {{24{mem[addr][7]}}, mem[addr]};
                3'b100: 
                    data_out = {24'b0, mem[addr]};
                3'b001: 
                    data_out = {{16{mem[addr+1][7]}}, mem[addr+1], mem[addr]};
                3'b101: 
                    data_out = {16'b0, mem[addr+1], mem[addr]};
                3'b010: 
                    data_out = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
                default:
                    data_out = 32'b0;
            endcase
        end else begin
            data_out = 32'b0;
        end
    end

endmodule

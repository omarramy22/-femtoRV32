module mem #(parameter MEMFILE = "default.mem")
(
    input  wire        clk,
    input  wire        MemRead,
    input  wire        MemWrite,
    input  wire [31:0] addr,
    input  wire [31:0] write_data,
    input  wire [2:0]  funct3,
    output reg  [31:0] read_data
);

    reg [7:0] mem_array [0:4095];
    reg [31:0] instr_words [0:1023];
    integer i;

    initial begin
        for (i = 0; i < 4096; i = i + 1)
            mem_array[i] = 8'h00;

            $readmemh(MEMFILE, mem);        
            for (i = 0; i < 1024; i = i + 1) begin
            mem_array[i*4 + 0] = instr_words[i][7:0];
            mem_array[i*4 + 1] = instr_words[i][15:8];
            mem_array[i*4 + 2] = instr_words[i][23:16];
            mem_array[i*4 + 3] = instr_words[i][31:24];
        end
    end

    always @(posedge clk) begin
        if (MemWrite) begin
            case (funct3)
                3'b000: mem_array[addr] <= write_data[7:0];
                3'b001: begin
                    mem_array[addr]     <= write_data[7:0];
                    mem_array[addr + 1] <= write_data[15:8];
                end
                3'b010: begin
                    mem_array[addr]     <= write_data[7:0];
                    mem_array[addr + 1] <= write_data[15:8];
                    mem_array[addr + 2] <= write_data[23:16];
                    mem_array[addr + 3] <= write_data[31:24];
                end
            endcase
        end

        if (MemRead) begin
            case (funct3)
                3'b000: read_data <= {{24{mem_array[addr][7]}}, mem_array[addr]};
                3'b100: read_data <= {24'b0, mem_array[addr]};
                3'b001: read_data <= {{16{mem_array[addr+1][7]}}, mem_array[addr+1], mem_array[addr]};
                3'b101: read_data <= {16'b0, mem_array[addr+1], mem_array[addr]};
                3'b010: read_data <= {mem_array[addr+3], mem_array[addr+2], mem_array[addr+1], mem_array[addr]};
                default: read_data <= 32'b0;
            endcase
        end
        else begin
            read_data <= 32'b0;
        end
    end

endmodule

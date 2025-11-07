module instr_mem (
    input  wire        clk,
    input  wire [31:0] addr,    // byte address (from PC)
    output reg  [31:0] instr
);

    reg [31:0] mem [0:1023];

    initial begin
        // Load instructions from external hex file
        $readmemh("test/programs/basic_tests.mem", mem);
    end

    always @(posedge clk) begin
        instr <= mem[addr[31:2]];   
    end

endmodule
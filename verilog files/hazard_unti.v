module HazardUnit (
    input  wire        ID_EX_MemRead,
    input  wire [4:0]  ID_EX_rd,
    input  wire [4:0]  IF_ID_rs1,
    input  wire [4:0]  IF_ID_rs2,
    input  wire        EX_branch_taken,
    output reg         PCWrite,
    output reg         IF_ID_Write,
    output reg         ID_EX_Flush
);

    always @(*) begin
        PCWrite     = 1'b1;
        IF_ID_Write = 1'b1;
        ID_EX_Flush = 1'b0;

        if (ID_EX_MemRead &&
           (ID_EX_rd != 5'b0) &&
           ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2))) begin
            PCWrite     = 1'b0;
            IF_ID_Write = 1'b0;
            ID_EX_Flush = 1'b1;
        end

        if (EX_branch_taken) begin
            PCWrite     = 1'b1;
            IF_ID_Write = 1'b0;
            ID_EX_Flush = 1'b1;
        end
    end

endmodule

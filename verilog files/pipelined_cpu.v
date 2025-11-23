`timescale 1ns / 1ps
`include "defines.v"

module cpu_top #(parameter MEMFILE = "default.mem")(
    input  wire        clk,
    input  wire        rst,
    input  wire [1:0]  ledsel,
    input  wire [3:0]  ssdSel,
    input  wire        ssdClk,
    output reg  [15:0] leds,
    output reg  [3:0]  Anode,
    output reg  [6:0]  LED_out
);

    reg phase;
    always @(posedge clk or posedge rst) begin
        if (rst) phase <= 1'b0;
        else phase <= ~phase;
    end

    wire [31:0] pc_next;
    wire [31:0] pc_out;
    wire pc_en;
    pc pc_reg(.clk(clk), .rst(rst), .en(pc_en), .next_pc(pc_next), .pc_out(pc_out));

    wire MemRead_if;
    assign MemRead_if = (phase == 1'b0); // IF uses phase C0
    wire MemWrite_if = 1'b0;
    wire [2:0] funct3_if = 3'b010;
    wire [31:0] mem_addr_if = pc_out;

    wire [31:0] mem_read_data;
    wire MemRead_mem;
    wire MemWrite_mem;
    wire [2:0] funct3_mem;
    wire [31:0] mem_addr_mem;
    wire [31:0] mem_write_data_mem;

    wire mem_MemRead = (phase == 1'b0) ? MemRead_if : MemRead_mem;
    wire mem_MemWrite = (phase == 1'b0) ? MemWrite_if : MemWrite_mem;
    wire [31:0] mem_addr  = (phase == 1'b0) ? mem_addr_if  : mem_addr_mem;
    wire [31:0] mem_wdata = (phase == 1'b0) ? 32'b0         : mem_write_data_mem;
    wire [2:0]  mem_f3   = (phase == 1'b0) ? funct3_if     : funct3_mem;

    mem #(.MEMFILE(MEMFILE)) u_mem(
        .clk(clk),
        .MemRead(mem_MemRead),
        .MemWrite(mem_MemWrite),
        .addr(mem_addr),
        .write_data(mem_wdata),
        .funct3(mem_f3),
        .read_data(mem_read_data)
    );

    wire [63:0] IF_ID_in;
    wire [63:0] IF_ID_out;
    assign IF_ID_in = {pc_out, mem_read_data};

    wire IF_ID_Write;
    Register #(64) IF_ID_reg(.clk(clk), .load(IF_ID_Write), .rst(rst), .D(IF_ID_in), .Q(IF_ID_out));
    wire [31:0] ID_PC   = IF_ID_out[63:32];
    wire [31:0] ID_Inst = IF_ID_out[31:0];

    wire [4:0] opcode5 = ID_Inst[6:2];
    wire Branch_c, MemRead_c, MemToReg_c, MemWrite_c, ALUSrc_c, RegWrite_c, Jump_c, JumpR_c;
    wire [1:0] ALUOp_c;
    Main_Control ctrl(.opcode(opcode5),
                      .Branch(Branch_c),
                      .MemRead(MemRead_c),
                      .MemToReg(MemToReg_c),
                      .ALUOp(ALUOp_c),
                      .MemWrite(MemWrite_c),
                      .ALUSrc(ALUSrc_c),
                      .RegWrite(RegWrite_c),
                      .Jump(Jump_c),
                      .JumpR(JumpR_c));

    wire [31:0] ReadData1, ReadData2;
    wire [31:0] WB_WriteData;
    wire WB_RegWrite;
    wire [4:0] WB_Rd;
    wire WB_MemToReg;
    wire [31:0] WB_ALUOut;
    wire [31:0] WB_MemData;
    assign WB_WriteData = (WB_MemToReg) ? WB_MemData : WB_ALUOut;
    register_file rf(.clk(clk), .rst(rst), .RegWrite(WB_RegWrite),
                     .ReadReg1(ID_Inst[19:15]), .ReadReg2(ID_Inst[24:20]),
                     .WriteReg(WB_Rd), .WriteData(WB_WriteData),
                     .ReadData1(ReadData1), .ReadData2(ReadData2));

    wire [31:0] Imm;
    imm_gen immgen(.inst(ID_Inst), .imm(Imm));

    wire [9:0] ctrl_bits;
    assign ctrl_bits = {RegWrite_c, MemToReg_c, MemRead_c, MemWrite_c, Branch_c, ALUOp_c, ALUSrc_c, Jump_c, JumpR_c};

    wire [162:0] ID_EX_in;
    assign ID_EX_in = {ctrl_bits, ID_PC, ReadData1, ReadData2, Imm, ID_Inst[14:12], ID_Inst[31:25], ID_Inst[19:15], ID_Inst[24:20], ID_Inst[11:7]};

    wire [9:0]  ID_EX_ctrl;
    wire [31:0] ID_EX_PC;
    wire [31:0] ID_EX_RegR1;
    wire [31:0] ID_EX_RegR2;
    wire [31:0] ID_EX_Imm;
    wire [2:0]  ID_EX_funct3;
    wire [6:0]  ID_EX_funct7;
    wire [4:0]  ID_EX_Rs1;
    wire [4:0]  ID_EX_Rs2;
    wire [4:0]  ID_EX_Rd;

    assign ID_EX_ctrl    = ID_EX_in[162:153];
    assign ID_EX_PC      = ID_EX_in[152:121];
    assign ID_EX_RegR1   = ID_EX_in[120:89];
    assign ID_EX_RegR2   = ID_EX_in[88:57];
    assign ID_EX_Imm     = ID_EX_in[56:25];
    assign ID_EX_funct3  = ID_EX_in[24:22];
    assign ID_EX_funct7  = ID_EX_in[21:15];
    assign ID_EX_Rs1     = ID_EX_in[14:10];
    assign ID_EX_Rs2     = ID_EX_in[9:5];
    assign ID_EX_Rd      = ID_EX_in[4:0];

    wire PCWrite, ID_EX_Flush;
    HazardUnit hazard0 (
        .ID_EX_MemRead(ID_EX_ctrl[7]),
        .ID_EX_rd(ID_EX_Rd),
        .IF_ID_rs1(ID_Inst[19:15]),
        .IF_ID_rs2(ID_Inst[24:20]),
        .EX_branch_taken(1'b0),
        .PCWrite(PCWrite),
        .IF_ID_Write(IF_ID_Write),
        .ID_EX_Flush(ID_EX_Flush)
    );

    wire [9:0] ctrl_bits_after_flush = (ID_EX_Flush) ? 10'b0 : ctrl_bits;
    wire [162:0] ID_EX_in2;
    assign ID_EX_in2 = {ctrl_bits_after_flush, ID_PC, ReadData1, ReadData2, Imm, ID_Inst[14:12], ID_Inst[31:25], ID_Inst[19:15], ID_Inst[24:20], ID_Inst[11:7]};

    Register #(163) ID_EX_reg(.clk(clk), .load(1'b1), .rst(rst), .D(ID_EX_in2), .Q(ID_EX_in));

    wire [1:0] ForwardA, ForwardB;
    wire [4:0] EXMEM_rd;
    wire [4:0] MEMWB_rd;
    wire EXMEM_RegWrite, MEMWB_RegWrite;
    ForwardingUnit fwd0(
        .ID_EX_rs1(ID_EX_Rs1),
        .ID_EX_rs2(ID_EX_Rs2),
        .EX_MEM_rd(EXMEM_rd),
        .MEM_WB_rd(MEMWB_rd),
        .EX_MEM_CTRL_regwrite(EXMEM_RegWrite),
        .MEM_WB_CTRL_regwrite(MEMWB_RegWrite),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );

    wire [1:0] ALUOp_ex = ID_EX_ctrl[4:3];
    wire ALUSrc_ex      = ID_EX_ctrl[2];

    wire [31:0] fwdA_from_EXMEM;
    wire [31:0] fwdA_from_MEMWB;
    wire [31:0] fwdB_from_EXMEM;
    wire [31:0] fwdB_from_MEMWB;

    wire [31:0] ALU_inA_pre = ID_EX_RegR1;
    wire [31:0] ALU_inB_pre = ID_EX_RegR2;

    wire [31:0] ALU_inA = (ForwardA == 2'b10) ? fwdA_from_EXMEM :
                          (ForwardA == 2'b01) ? fwdA_from_MEMWB :
                          ALU_inA_pre;

    wire [31:0] ALU_inB_pre2 = (ForwardB == 2'b10) ? fwdB_from_EXMEM :
                               (ForwardB == 2'b01) ? fwdB_from_MEMWB :
                               ALU_inB_pre;

    wire [31:0] ALU_inB = (ALUSrc_ex) ? ID_EX_Imm : ALU_inB_pre2;

    wire [3:0] ALU_Sel;
    ALU_Control aluctrl(.ALUOp(ALUOp_ex), .funct3(ID_EX_funct3), .funct7(ID_EX_funct7), .ALU_Sel(ALU_Sel));

    wire [31:0] ALU_Out;
    wire Zero, LessThan, LessThanUnsigned;
    N_bit_ALU alu(.A(ALU_inA), .B(ALU_inB), .Sel(ALU_Sel), .Out(ALU_Out), .Zero(Zero), .LessThan(LessThan), .LessThanUnsigned(LessThanUnsigned));

    wire EX_branch_taken;
    Branch_Unit branch_unit(.funct3(ID_EX_funct3), .Zero(Zero), .LessThan(LessThan), .LessThanUnsigned(LessThanUnsigned), .branch_taken(EX_branch_taken));

    wire [31:0] branch_target = ID_EX_PC + ID_EX_Imm;

    wire [3:0] EX_MEM_ctrl;
    assign EX_MEM_ctrl = {ID_EX_ctrl[9], ID_EX_ctrl[8], ID_EX_ctrl[7], ID_EX_ctrl[6]}; // {RegWrite, MemToReg, MemRead, MemWrite}

    wire [110:0] EX_MEM_in;
    assign EX_MEM_in = {EX_MEM_ctrl, ALU_Out, ALU_inB_pre, branch_target, ID_EX_funct3, EX_branch_taken, ID_EX_ctrl[1], ID_EX_ctrl[0], ID_EX_Rd};
    Register #(111) EX_MEM_reg(.clk(clk), .load(1'b1), .rst(rst), .D(EX_MEM_in), .Q(EX_MEM_in));

    wire [3:0] MEM_EX_ctrl_out = EX_MEM_in[110:107];
    wire [31:0] MEM_ALUOut      = EX_MEM_in[106:75];
    wire [31:0] MEM_WriteData   = EX_MEM_in[74:43];
    wire [31:0] MEM_BranchTarget= EX_MEM_in[42:11];
    wire [2:0]  MEM_ex_funct3   = EX_MEM_in[10:8];
    wire MEM_branch_taken      = EX_MEM_in[7];
    wire MEM_Jump              = EX_MEM_in[6];
    wire MEM_JumpR             = EX_MEM_in[5];
    wire [4:0] MEM_Rd          = EX_MEM_in[4:0];

    assign MemRead_mem = MEM_EX_ctrl_out[1];
    assign MemWrite_mem = MEM_EX_ctrl_out[0];
    assign funct3_mem = MEM_ex_funct3;
    assign mem_addr_mem = MEM_ALUOut;
    assign mem_write_data_mem = MEM_WriteData;

    wire [70:0] MEM_WB_in;
    assign MEM_WB_in = {MEM_EX_ctrl_out[3], MEM_EX_ctrl_out[2], MEM_ALUOut, mem_read_data, MEM_Rd};
    Register #(71) MEM_WB_reg(.clk(clk), .load(1'b1), .rst(rst), .D(MEM_WB_in), .Q(MEM_WB_in));

    assign WB_RegWrite = MEM_WB_in[70];
    assign WB_MemToReg = MEM_WB_in[69];
    assign WB_ALUOut    = MEM_WB_in[68:37];
    assign WB_MemData   = MEM_WB_in[36:5];
    assign WB_Rd        = MEM_WB_in[4:0];

    assign fwdA_from_EXMEM = MEM_ALUOut;
    assign fwdB_from_EXMEM = MEM_ALUOut;
    assign fwdA_from_MEMWB = WB_WriteData;
    assign fwdB_from_MEMWB = WB_WriteData;

    assign EXMEM_rd = MEM_Rd;
    assign MEMWB_rd = WB_Rd;
    assign EXMEM_RegWrite = MEM_EX_ctrl_out[3];
    assign MEMWB_RegWrite = WB_RegWrite;

    wire [31:0] pc_plus_4 = pc_out + 32'd4;
    wire [31:0] next_pc_wire;
    assign next_pc_wire = (MEM_JumpR) ? MEM_ALUOut :
                          (MEM_Jump)  ? MEM_BranchTarget :
                          (MEM_branch_taken) ? MEM_BranchTarget :
                          pc_plus_4;

    assign pc_next = next_pc_wire;
    assign pc_en = PCWrite;

    // SSD/display logic
    reg [13:0] ssd;
    reg [19:0] refreshC = 0;
    wire [1:0] LEDAcounter;
    reg [31:0] in_PC_r;
    always @(posedge clk or posedge rst) begin
        if (rst) in_PC_r <= 32'd0;
        else in_PC_r <= pc_out;
    end

    wire [31:0] new_PC = pc_next;
    wire [31:0] GenOut = ID_EX_Imm;
    wire [31:0] ALUI2 = ALU_inB;
    wire [31:0] ReadD1_w = ReadData1;
    wire [31:0] ReadD2_w = ReadData2;
    wire [31:0] WritData_w = WB_WriteData;
    wire [31:0] dataOut_w = mem_read_data;
    wire [31:0] ALU_out_w = ALU_Out;

    always @(*) begin
        case (ssdSel)
            4'b0000: ssd <= pc_out[13:0];
            4'b0001: ssd <= new_PC[13:0];
            4'b0010: ssd <= ID_EX_Imm[13:0];
            4'b0011: ssd <= in_PC_r[13:0];
            4'b0100: ssd <= ReadD1_w[13:0];
            4'b0101: ssd <= ReadD2_w[13:0];
            4'b0110: ssd <= WritData_w[13:0];
            4'b0111: ssd <= GenOut[13:0];
            4'b1000: ssd <= {GenOut[30:18], 1'b0};
            4'b1001: ssd <= ALUI2[13:0];
            4'b1010: ssd <= ALU_out_w[13:0];
            4'b1011: ssd <= dataOut_w[13:0];
            default: ssd <= 14'd0;
        endcase
    end

    always @(posedge ssdClk) refreshC <= refreshC + 1;
    assign LEDAcounter = refreshC[19:18];

    reg [3:0] LEDbcd;
    always @(*) begin
        case (LEDAcounter)
            2'b00: begin Anode = 4'b0111; LEDbcd = ssd / 1000; end
            2'b01: begin Anode = 4'b1011; LEDbcd = (ssd % 1000) / 100; end
            2'b10: begin Anode = 4'b1101; LEDbcd = ((ssd % 1000) % 100) / 10; end
            2'b11: begin Anode = 4'b1110; LEDbcd = ((ssd % 1000) % 100) % 10; end
        endcase
    end

    always @(*) begin
        case(LEDbcd)
            4'b0000: LED_out = 7'b0000001;
            4'b0001: LED_out = 7'b1001111;
            4'b0010: LED_out = 7'b0010010;
            4'b0011: LED_out = 7'b0000110;
            4'b0100: LED_out = 7'b1001100;
            4'b0101: LED_out = 7'b0100100;
            4'b0110: LED_out = 7'b0100000;
            4'b0111: LED_out = 7'b0001111;
            4'b1000: LED_out = 7'b0000000;
            4'b1001: LED_out = 7'b0000100;
            default: LED_out = 7'b0000001;
        endcase
    end

    wire [15:0] Signs;
    assign Signs = {2'b00, ALUOp_ex, ALU_Sel, Zero, EX_branch_taken, Branch_c, MemRead_c, MemToReg_c, MemWrite_c, ALUSrc_c, RegWrite_c};

    always @(*) begin
        case (ledsel)
            2'b00: leds  mem_read_data[15:0];
            2'b01: leds = mem_read_data[31:16];
            2'b10: leds = Signs;
            default: leds = 16'd0;
        endcase
    end

endmodule

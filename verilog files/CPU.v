`timescale 1ns / 1ps
`include "defines.v"

module rv32i_single_cpu (
    input  wire        clk,
    input  wire        rst,
    output wire [31:0] pc_out,          // current PC
    output wire [31:0] pc_next_out,     // NEXT PC
    output wire [31:0] instr_out,       // current instruction
    output wire [31:0] alu_out,         // ALU result
    output wire [31:0] mem_out,         // Data memory output
    output wire [31:0] reg_wdata_out,   // Writeback data
    output wire        branch_taken_out,// Branch decision
    output wire        Jump_out,        // Jump flag
    output wire        RegWrite_out,     // Register write enable
    output wire        halted_out        // halt result    
);

wire is_system_opcode = (instr[6:2] == 5'b11100); // ECALL/EBREAK/PAUSE
wire is_fence_opcode  = (instr[6:2] == 5'b00011); // FENCE / FENCE.TSO


// single halt condition
wire halt_inst = is_system_opcode | is_fence_opcode;

reg halted;
always @(posedge clk or posedge rst) begin
    if (rst)
        halted <= 1'b0;
    else if (halt_inst)
        halted <= 1'b1;
end

// current PC
wire [31:0] pc_current;

// next PC computed by top module
wire [31:0] pc_next;

// instantiate program counter
pc pc_reg (
    .clk(clk),
    .rst(rst),
    .en(~halted),
    .next_pc(pc_next),
    .pc_out(pc_current)
);

// instruction fetch
wire [31:0] instr;
instr_mem instr_mem_inst (
    .clk(clk),
    .addr(pc_current),
    .instr(instr)
);


wire [4:0] opcode5 = instr[6:2];
wire [2:0] funct3  = instr[14:12];
wire [6:0] funct7  = instr[31:25];
wire [4:0] rs1_idx = instr[19:15];
wire [4:0] rs2_idx = instr[24:20];
wire [4:0] rd_idx  = instr[11:7];

// control outputs from main control
wire Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite, Jump;
wire [1:0] ALUOp;
Main_Control main_ctrl (
    .opcode(opcode5),
    .Branch(Branch),
    .MemRead(MemRead),
    .MemToReg(MemToReg),
    .ALUOp(ALUOp),
    .MemWrite(MemWrite),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .Jump(Jump)
);

// read data from register file
wire [31:0] rs1_data;
wire [31:0] rs2_data;

// data to write back to register file
wire [31:0] regfile_wdata;

register_file regfile_inst (
    .clk(clk),
    .rst(rst),
    .RegWrite(RegWrite),      
    .ReadReg1(rs1_idx),
    .ReadReg2(rs2_idx),
    .WriteReg(rd_idx),
    .WriteData(regfile_wdata),
    .ReadData1(rs1_data),
    .ReadData2(rs2_data)
);


wire [31:0] imm;
rv32_ImmGen immgen (
    .IR(instr),
    .Imm(imm)
);


wire [3:0] ALU_Sel;
ALU_Control alu_ctrl (
    .ALUOp(ALUOp),
    .funct3(funct3),
    .funct7(funct7),
    .ALU_Sel(ALU_Sel)
);


// choose ALU B input: either rs2 or imm
wire [31:0] alu_b;
wire [31:0] alu_a = rs1_data;
assign alu_b = (ALUSrc) ? imm : rs2_data;

// ALU outputs (Zero, LessThan, LessThanUnsigned) used for branching
wire [31:0] alu_result;
wire alu_zero, alu_lt, alu_ltu;
N_bit_ALU #(.N(32)) alu_inst (
    .A(alu_a),
    .B(alu_b),
    .Sel(ALU_Sel),
    .Out(alu_result),
    .Zero(alu_zero),
    .LessThan(alu_lt),
    .LessThanUnsigned(alu_ltu)
);


wire [31:0] mem_read_data;
DataMem data_mem (
    .clk(clk),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .addr(alu_result),       // address from ALU 
    .data_in(rs2_data),      // store value comes from rs2
    .funct3(funct3),
    .data_out(mem_read_data)
);


// We must select between:
//  - For LUI: imm (U-type)
//  - For AUIPC: pc + imm
//  - For JAL/JALR: write PC+4 to rd
//  - For Load: data from memory
//  - Otherwise: ALU result

wire [31:0] pc_plus_4 = pc_current + 32'd4;
wire [31:0] branch_target = pc_current + imm;
wire [31:0] jalr_target = (rs1_data + imm) & ~32'd1; // JALR target

// select writeback from mem or ALU
wire [31:0] wb_alu_or_mem;
muxN #(.N(32)) wb_mem_alu_mux (
    .a(alu_result),
    .b(mem_read_data),
    .sel(MemToReg),
    .y(wb_alu_or_mem)
);

// compute final writeback (regfile_wdata)
reg [31:0] final_wb;
always @(*) begin
    // default
    final_wb = wb_alu_or_mem;

    // LUI: write imm << 12 (imm_gen already returns U-type with lower 12 bits 0)
    if (opcode5 == `OPCODE_LUI) begin
        final_wb = imm;
    end
    // AUIPC: write pc + imm
    else if (opcode5 == `OPCODE_AUIPC) begin
        final_wb = pc_current + imm;
    end
    // JAL and JALR: write PC+4 to rd
    else if ((opcode5 == `OPCODE_JAL) || (opcode5 == `OPCODE_JALR)) begin
        final_wb = pc_plus_4;
    end
end

assign regfile_wdata = final_wb;


reg branch_taken;
always @(*) begin
    branch_taken = 1'b0;
    if (Branch) begin
        case (funct3)
            3'b000: branch_taken = alu_zero;           // BEQ
            3'b001: branch_taken = ~alu_zero;          // BNE
            3'b100: branch_taken = alu_lt;             // BLT (signed)
            3'b101: branch_taken = ~alu_lt;            // BGE (signed)
            3'b110: branch_taken = alu_ltu;            // BLTU (unsigned)
            3'b111: branch_taken = ~alu_ltu;           // BGEU (unsigned)
            default: branch_taken = 1'b0;
        endcase
    end
end


reg [31:0] computed_next_pc;
always @(*) begin
    if (opcode5 == `OPCODE_JALR) begin
        // JALR uses rs1 + imm (clear LSB)
        computed_next_pc = jalr_target;
    end
    else if (opcode5 == `OPCODE_JAL) begin
        computed_next_pc = branch_target; // JAL uses pc + imm
    end
    else if (branch_taken) begin
        computed_next_pc = branch_target;
    end
    else begin
        computed_next_pc = pc_plus_4;
    end
end

assign pc_next = computed_next_pc;

assign pc_out          = pc_current;
assign pc_next_out     = pc_next;
assign instr_out       = instr;
assign alu_out         = alu_result;
assign mem_out         = mem_read_data;
assign reg_wdata_out   = regfile_wdata;
assign branch_taken_out = branch_taken;
assign Jump_out        = Jump;
assign RegWrite_out    = RegWrite;
assign halted_out = halted;

endmodule

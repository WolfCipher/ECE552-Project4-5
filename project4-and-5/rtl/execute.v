// alu, branch/jump update
// DOES choose between immediate and register 2
module execute(
    input wire i_clk,
    // ALU inputs
    input wire [31:0] reg1,
    input wire [31:0] reg2,
    input wire [31:0] imm,
    input wire [2:0] funct3,
    input wire [2:0] i_opsel,
    input wire i_sub,
    input wire i_unsigned,
    input wire i_arith,
    // signals related to PC, branch, and ALU
    input wire [31:0] i_PC,
    input wire [31:0] i_PC4,
    output reg [31:0] o_result,
    output reg o_eq,
    output reg o_slt,
    output reg [31:0] target_addr,
    output reg [31:0] o_PC,
    output reg [31:0] o_PC4,
    // signals for proper memory access
    output reg o_unsigned, // for memory
    output wire [3:0] o_mask, // for memory
    output wire [31:0] mem_addr, // for memory; different from o_result if not working with a word
    output wire [31:0] mem_wdata,
    output reg [31:0] o_reg2,
    // input mux signals
    input wire i_ALUSrc,
    input wire i_isJALR,
    input wire i_Jump,
    input wire i_BranchEqual,
    input wire i_BranchLT,
    input wire i_Branch,
    input wire i_MemRead,
    input wire i_MemtoReg,
    input wire i_MemWrite,
    input wire [4:0] i_rd_waddr,
    input wire i_RegWrite,
    input wire i_UpperType,
    input wire i_IsUInstruct,
    // output mux signals
    output reg o_isJALR,
    output reg o_Jump,
    output reg o_BranchEqual,
    output reg o_BranchLT,
    output reg o_Branch,
    output reg o_MemRead,
    output reg o_MemtoReg,
    output reg o_MemWrite,
    output reg [4:0] o_rd_waddr,
    output reg o_RegWrite,
    output reg o_IsUInstruct,
    // U type result
    output reg [31:0] o_uimm,
    // Input Retire Instructions
    input wire i_halt,
    input wire [31:0] i_inst,
    input wire i_trapD,
    input wire [31:0] i_reg1,
    input wire [31:0] i_reg2,
    input wire [4:0] i_rs1,
    input wire [4:0] i_rs2,
    // Output Retire Instructions
    output reg o_halt,
    output reg [31:0] o_inst,
    output reg o_trapD,
    output reg [31:0] o_reg1,
    output reg [31:0] o_reg2_retire,
    output reg [4:0] o_rs1,
    output reg [4:0] o_rs2,
    output reg o_trapX
);

    // ALU
    wire [31:0] i_op1, i_op2;
    assign i_op1 = reg1;
    assign i_op2 = i_ALUSrc ? imm : reg2;
    alu op (i_opsel, i_sub, i_unsigned, i_arith, i_op1, i_op2, o_result, o_eq, o_slt);
    

    // branch or jump target address
    assign target_addr = i_isJALR ? (reg1 + imm) : (i_PC + imm);

    // U-type immediate
    assign o_uimm = i_UpperType ? target_addr : imm; // target_addr = i_PC + imm

    // mask decoder
    always @(posedge i_clk) begin
        o_unsigned <= funct3[2]; // LB/LBU, LH/LHU, LW/LWU
    end
    assign o_mask = (funct3[1:0] == 2'b00) ? (
                        (o_result[1:0] == 2'b00) ? 4'b0001 :
                        (o_result[1:0] == 2'b01) ? 4'b0010 :
                        (o_result[1:0] == 2'b10) ? 4'b0100 :
                        4'b1000
                    ) : // BYTE
                    (funct3[1:0] == 2'b01) ? (
                        (o_result[1:0] == 2'b00) ? 4'b0011 :
                        //(o_result[1:0] == 2'b01) ? 4'b0110 :
                        (o_result[1:0] == 2'b10) ? 4'b1100 :
                        //(o_result[1:0] == 2'b11) ? 4'b1001 :
                        4'b0000 // DON'T CARE
                    ) : // HALFWORD
                    4'b1111; // WORD
    assign mem_addr = {o_result[31:2], 2'b00};

    // data to send to memory
    wire [31:0] shift_data;

    wire bit0 = (o_mask == 4'b0001) || (o_mask == 4'b0011) || (o_mask == 4'b0101) || (o_mask == 4'b0111) || (o_mask == 4'b1001) || (o_mask == 4'b1011) || (o_mask == 4'b1101) || (o_mask == 4'b1111);
    wire bit1 = (o_mask == 4'b0010) || (o_mask == 4'b0110) || (o_mask == 4'b1010) || (o_mask == 4'b1110);
    wire bit2 = (o_mask == 4'b0100) || (o_mask == 4'b1100);

    assign shift_data = (bit0) ? reg2 :
                        (bit1) ? {reg2[23:0], 8'd0} :
                        (bit2) ? {reg2[15:0], 16'd0} :
                        {reg2[7:0], 24'd0}; // (o_mask == 4'b1000)

    assign mem_wdata = shift_data;

    // trap check
    wire mem_misalign = (funct3[1:0] == 2'b00) ? 1'b0 : // BYTE can be at any address
                        (funct3[1:0] == 2'b01) ? (o_result[0] != 1'b0) : // HALFWORD must be halfword-aligned
                        (o_result[1:0] != 2'b00); // WORD must be word-aligned

    always @(posedge i_clk) begin
        o_trapX <= (mem_misalign && (i_MemWrite || i_MemRead)) || ((target_addr[1:0] != 2'b00) && (i_Jump || i_Branch));
    end

    // pass through stage
    always @(posedge i_clk) begin
        o_PC <= i_PC;
        o_PC4 <= i_PC4;
        o_isJALR <= i_isJALR;
        o_Jump <= i_Jump;
        o_BranchEqual <= i_BranchEqual;
        o_BranchLT <= i_BranchLT;
        o_Branch <= i_Branch;
        o_MemRead <= i_MemRead;
        o_MemtoReg <= i_MemtoReg;
        o_MemWrite <= i_MemWrite;
        o_rd_waddr <= i_rd_waddr;
        o_RegWrite <= i_RegWrite;
        o_IsUInstruct <= i_IsUInstruct;
        o_reg2 <= reg2;

        // Retire instructions
        o_halt <= i_halt;
        o_inst <= i_inst;
        o_trapD <= i_trapD;
        o_reg1 <= i_reg1;
        o_reg2_retire <= i_reg2;
        o_rs1 <= i_rs1;
        o_rs2 <= i_rs2;
    end

endmodule

`default_nettype wire

// alu, branch/jump update
// DOES choose between immediate and register 2
module execute(
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
    output wire [31:0] o_result,
    output wire o_eq,
    output wire o_slt,
    output wire [31:0] target_addr,
    output wire [31:0] o_PC,
    output wire [31:0] o_PC4,
    // signals for proper memory access
    output wire o_unsigned, // for memory
    output wire [3:0] o_mask, // for memory
    output wire [31:0] mem_addr, // for memory; different from o_result if not working with a word
    output wire [31:0] mem_wdata,
    output wire [31:0] o_reg2,
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
    output wire o_isJALR,
    output wire o_Jump,
    output wire o_BranchEqual,
    output wire o_BranchLT,
    output wire o_Branch,
    output wire o_MemRead,
    output wire o_MemtoReg,
    output wire o_MemWrite,
    output wire [4:0] o_rd_waddr,
    output wire o_RegWrite,
    output wire o_IsUInstruct,
    // U type result
    output wire [31:0] o_uimm,
    // Trap Check
    output wire o_trap
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
    assign o_unsigned = funct3[2]; // LB/LBU, LH/LHU, LW/LWU
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

    assign o_trap = (mem_misalign && (i_MemWrite || i_MemRead)) || ((target_addr[1:0] != 2'b00) && (i_Jump || i_Branch));

    // pass through stage
    assign o_PC = i_PC;
    assign o_PC4 = i_PC4;
    assign o_isJALR = i_isJALR;
    assign o_Jump = i_Jump;
    assign o_BranchEqual = i_BranchEqual;
    assign o_BranchLT = i_BranchLT;
    assign o_Branch = i_Branch;
    assign o_MemRead = i_MemRead;
    assign o_MemtoReg = i_MemtoReg;
    assign o_MemWrite = i_MemWrite;
    assign o_rd_waddr = i_rd_waddr;
    assign o_RegWrite = i_RegWrite;
    assign o_IsUInstruct = i_IsUInstruct;
    assign o_reg2 = reg2;

endmodule

`default_nettype wire

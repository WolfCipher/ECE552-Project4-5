// alu, branch/jump update
// DOES choose between immediate and register 2
module execute(
    input wire i_clk,
    // ALU inputs
    input reg [31:0] reg1,
    input reg [31:0] reg2,
    input reg [31:0] imm,
    input reg [2:0] funct3,
    input reg [2:0] i_opsel,
    input reg i_sub,
    input reg i_unsigned,
    input reg i_arith,
    // signals related to PC, branch, and ALU
    input reg [31:0] i_PC,
    input reg [31:0] i_PC4,
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
    input reg i_ALUSrc,
    input reg i_isJALR,
    input reg i_Jump,
    input reg i_BranchEqual,
    input reg i_BranchLT,
    input reg i_Branch,
    input reg i_MemRead,
    input reg i_MemtoReg,
    input reg i_MemWrite,
    input reg [4:0] i_rd_waddr,
    input reg i_RegWrite,
    input reg i_UpperType,
    input reg i_IsUInstruct,
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

    assign o_trap = (mem_misalign && (i_MemWrite || i_MemRead)) || ((target_addr[1:0] != 2'b00) && (i_Jump || i_Branch));

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
    end

endmodule

`default_nettype wire

// data memory
module memory(
    input wire i_clk,
    // signals sent to data memory
    input wire [3:0] i_mask,
    input wire i_unsigned,
    input wire [31:0] i_mem_addr,
    input wire [31:0] i_reg2,
    // ALU signal
    input wire [31:0] i_result,
    // Branch and PC signals
    input wire i_eq,
    input wire i_slt,
    input wire [31:0] target_addr,
    input wire [31:0] i_PC,
    input wire [31:0] i_PC4,
    output reg [31:0] o_PC,
    output reg [31:0] next_PC,
    // Results to choose between in WB stage
    output reg [31:0] read_data,
    output reg [31:0] read_alu,
    output reg [31:0] o_uimm,
    // input Mux signals
    input wire i_isJALR,
    input wire i_Jump,
    input wire i_BranchEqual,
    input wire i_BranchLT,
    input wire i_Branch,
    input wire i_MemRead,
    input wire i_MemtoReg,
    //input wire i_MemWrite, must ask one cycle ahead
    input wire [4:0] i_rd_waddr,
    input wire i_RegWrite,
    input wire i_IsUInstruct,
    input wire [31:0] i_uimm,
    // output Mux signals
    output reg o_Jump,
    output reg o_MemtoReg,
    output reg [4:0] o_rd_waddr,
    output reg o_RegWrite,
    output reg o_IsUInstruct,
    // dmem
    input wire [31:0] i_dmem_rdata,
    output wire o_dmem_ren,
    // Input Retire Instructions
    input wire i_halt,
    input wire [31:0] i_inst,
    input wire i_trapD,
    input wire [31:0] i_reg1,
    input wire [31:0] i_reg2_retire,
    input wire [4:0] i_rs1,
    input wire [4:0] i_rs2,
    input wire i_trapX,
    // Output Retire Instructions
    output reg o_halt,
    output reg [31:0] o_inst,
    output reg o_trapD,
    output reg [31:0] o_reg1,
    output reg [31:0] o_reg2,
    output reg [4:0] o_rs1,
    output reg [4:0] o_rs2,
    output reg o_trapX
);

    // determine PC
    wire [31:0] muxed_target;
    assign muxed_target = i_isJALR ? {target_addr[31:1], 1'b0} : target_addr;

    wire branch_taken;
    assign branch_taken = ((i_BranchEqual & i_eq) | // beq
                            (i_BranchLT & i_slt) |  // blt(u)
                            (~i_BranchLT & ~i_slt & ~i_BranchEqual) | // bge(u)
                            (~i_BranchEqual & ~i_eq & ~i_BranchLT)) // bne
                        & i_Branch;

    always @(posedge i_clk) begin
        next_PC <= (branch_taken || i_Jump) ? muxed_target : i_PC4;
    end

    // dmem
    assign o_dmem_ren = i_MemRead;

    // ****** READ *******
    // only read if read-enabled
    // select bytes using the mask
    wire [31:0] data, masked_data;
    assign data = i_MemRead ? i_dmem_rdata : 32'd0;
    assign masked_data[31:24] = data[31:24] & {8{i_mask[3]}};
    assign masked_data[23:16] = data[23:16] & {8{i_mask[2]}};
    assign masked_data[15:8] = data[15:8] & {8{i_mask[1]}};
    assign masked_data[7:0] = data[7:0] & {8{i_mask[0]}};

    // handle any needed shifts
    wire sign_bit;

    wire bit3 = (i_mask == 4'b1000) || (i_mask == 4'b1100) || (i_mask == 4'b1010) || (i_mask == 4'b1001) || (i_mask == 4'b1110) || (i_mask == 4'b1101) || (i_mask == 4'b1011) || (i_mask == 4'b1111);
    wire bit2 = (i_mask == 4'b0100) || (i_mask == 4'b0110) || (i_mask == 4'b0101) || (i_mask == 4'b0111);
    wire bit1 = (i_mask == 4'b0010) || (i_mask == 4'b0011);

    assign sign_bit = i_unsigned ? 1'd0 : (
                      (bit3) ? masked_data[31] :
                      (bit2) ? masked_data[23] :
                      (bit1) ? masked_data[15] :
                      masked_data[7]
                      );

    wire [31:0] o_data;
    assign o_data = (i_mask == 4'b1111) ? masked_data :
                    (i_mask == 4'b0011) ? {{16{sign_bit}}, masked_data[15:0]} :
                    (i_mask == 4'b0001) ? {{24{sign_bit}}, masked_data[7:0]} :
                    (i_mask == 4'b0110) ? {{16{sign_bit}}, masked_data[23:8]} :
                    (i_mask == 4'b1100) ? {{16{sign_bit}}, masked_data[31:16]} :
                    (i_mask == 4'b1000) ? {{24{sign_bit}}, masked_data[31:24]} :
                    (i_mask == 4'b0100) ? {{24{sign_bit}}, masked_data[23:16]} :
                    (i_mask == 4'b0010) ? {{24{sign_bit}}, masked_data[15:8]} :
                    32'd0; // default case, should never occur

    // pass through stage
    always @(posedge i_clk) begin
        o_PC <= i_PC;
        read_alu <= i_result;
        o_Jump <= i_Jump;
        o_MemtoReg <= i_MemtoReg;
        o_rd_waddr <= i_rd_waddr;
        o_RegWrite <= i_RegWrite;
        o_IsUInstruct <= i_IsUInstruct;
        o_uimm <= i_uimm;
        read_data <= o_data;

        // Retire instructions
        o_halt <= i_halt;
        o_inst <= i_inst;
        o_trapD <= i_trapD;
        o_rs1 <= i_rs1;
        o_rs2 <= i_rs2;
        o_reg1 <= i_reg1;
        o_reg2 <= i_reg2_retire;
        o_trapX <= i_trapX;
    end

endmodule

`default_nettype wire

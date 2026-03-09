module writeback(
    input wire i_clk,
    input wire [31:0] i_PC,
    input wire [31:0] i_next_PC,
    // results to choose between
    input wire [31:0] read_data,
    input wire [31:0] read_alu,
    input wire [31:0] i_uimm,
    output wire [31:0] dest_result,
    output wire [31:0] o_PC,
    output wire [31:0] o_next_PC,
    // input mux signals
    input wire i_Jump,
    input wire i_MemtoReg,
    input wire [4:0] i_rd_waddr,
    input wire i_RegWrite,
    input wire i_IsUInstruct,
    // output signals
    output wire o_RegWrite,
    output wire [4:0] o_rd_waddr,
    // Input Retire Instructions
    input wire i_halt,
    input wire [31:0] i_inst,
    input wire i_trapD,
    input wire [31:0] i_reg1,
    input wire [31:0] i_reg2,
    input wire [4:0] i_rs1,
    input wire [4:0] i_rs2,
    input wire i_trapX,
    input wire [3:0] i_dmem_mask,
    input wire [31:0] i_dmem_addr,
    input wire [31:0] i_dmem_wdata,
    input wire i_dmem_ren,
    input wire i_dmem_wen,
    input wire [31:0] i_dmem_rdata,
    // Output Retire Instructions
    output wire o_halt,
    output wire [31:0] o_inst,
    output wire o_trapD,
    output wire [31:0] o_reg1,
    output wire [31:0] o_reg2,
    output wire [4:0] o_rs1,
    output wire [4:0] o_rs2,
    output wire o_trapX,
    output wire [3:0] o_dmem_mask,
    output wire [31:0] o_dmem_addr,
    output wire [31:0] o_dmem_wdata,
    output wire o_dmem_ren,
    output wire o_dmem_wen,
    output wire [31:0] o_dmem_rdata

    //added 
    input  wire        i_valid,
    output wire        o_valid
);
    // determine value to write back
    assign dest_result = i_Jump ? (i_PC + 32'd4) : (i_IsUInstruct ? i_uimm : (i_MemtoReg ? read_data : read_alu));

    // pass through stage - write back is handled via connections in hart
    // remember, writing occurs before reading in decode cycle
    assign o_next_PC = i_next_PC;

    // retire instructions
    assign o_PC = i_PC;
    assign o_RegWrite = i_RegWrite;
    assign o_rd_waddr = i_rd_waddr;
    assign o_halt = i_halt;
    assign o_inst = i_inst;
    assign o_trapD = i_trapD;
    assign o_reg1 = i_reg1;
    assign o_reg2 = i_reg2;
    assign o_rs1 = i_rs1;
    assign o_rs2 = i_rs2;
    assign o_trapX = i_trapX;
    assign o_dmem_mask = i_dmem_mask;
    assign o_dmem_addr = i_dmem_addr;
    assign o_dmem_wdata = i_dmem_wdata;
    assign o_dmem_ren = i_dmem_ren;
    assign o_dmem_wen = i_dmem_wen;
    assign o_dmem_rdata = i_dmem_rdata;
    assign o_valid = i_valid;


endmodule

`default_nettype wire

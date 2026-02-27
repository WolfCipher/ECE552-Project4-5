// data memory
module memory(
    // signals sent to data memory
    input wire i_clk,
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
    output wire [31:0] o_PC,
    output wire [31:0] next_PC,
    // Results to choose between in WB stage
    output wire [31:0] read_data,
    output wire [31:0] read_alu,
    output wire [31:0] o_uimm,
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
    output wire o_Jump,
    output wire o_MemtoReg,
    output wire [4:0] o_rd_waddr,
    output wire o_RegWrite,
    output wire o_IsUInstruct,
    // dmem
    input wire [31:0] i_dmem_rdata,
    output wire o_dmem_ren
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

    assign next_PC = (branch_taken || i_Jump) ? muxed_target : i_PC4;

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


    // read and write data TODO
    //data_memory dmem (i_clk, i_mask, i_unsigned, i_MemRead, i_MemWrite, i_mem_addr, i_reg2, i_MemtoR)

    // pass through stage
    assign o_PC = i_PC;
    assign read_alu = i_result;
    assign o_Jump = i_Jump;
    assign o_MemtoReg = i_MemtoReg;
    assign o_rd_waddr = i_rd_waddr;
    assign o_RegWrite = i_RegWrite;
    assign o_IsUInstruct = i_IsUInstruct;
    assign o_uimm = i_uimm;
    assign read_data = o_data;

endmodule

// module data_memory(
//     input wire i_clk,
//     input wire [3:0] mask,
//     input wire i_unsigned,
//     //input wire i_rst,
//     input wire i_MemRead,
//     input wire i_MemWrite,
//     input wire [31:0] i_addr,
//     input wire [31:0] i_data,
//     output wire [31:0] o_data
// );

//     reg [31:0] d_mem [31:0];

//     // ****** READ *******
//     // only read if read-enabled
//     // select bytes using the mask
//     wire [31:0] data, masked_data;
//     assign data = i_MemRead ? d_mem[i_addr] : 32'hxxxxxxxx;
//     assign masked_data[31:24] = data[31:24] & {8{mask[3]}};
//     assign masked_data[23:16] = data[23:16] & {8{mask[2]}};
//     assign masked_data[15:8] = data[15:8] & {8{mask[1]}};
//     assign masked_data[7:0] = data[7:0] & {8{mask[0]}};

//     // handle any needed shifts
//     wire sign_bit;

//     assign sign_bit = i_unsigned ? (
//                       (mask == 4'1xxx) ? masked_data[31] :
//                       (mask == 4'01xx) ? masked_data[23] :
//                       (mask == 4'001x) ? masked_data[15] :
//                       masked_data[7]
//                       ) : 0;

//     assign o_data = (mask == 4'b1111) ? masked_data :
//                     (mask == 4'b0011) ? {{16{sign_bit}}, masked_data[15:0]} :
//                     (mask == 4'b0001) ? {{24{sign_bit}}, masked_data[7:0]} :
//                     (mask == 4'b0110) ? {{16{sign_bit}}, masked_data[23:8]} :
//                     (mask == 4'b1100) ? {{16{sign_bit}}, masked_data[31:16]} :
//                     (mask == 4'b1000) ? {{24{sign_bit}}, masked_data[31:24]} :

//     // ****** WRITE *******
//     wire [31:0] shift_data;
//     assign shift_data = (mask == 4'bxxx1) ? i_data :
//                         (mask == 4'bxx10) ? {i_data[23:0], 8'd0} :
//                         (mask == 4'bx100) ? {i_data[15:0], 16'd0} :
//                         (mask == 4'b1000) ? {i_data[7:0], 24'd0} :

//     always @(posedge i_clk) begin
//         // handle active-high reset
//         // if (i_rst == 1) begin
//         //     for (i = 0; i < 32; i = i + 1)
//         //         reg_file[i] <= 32'd0;
//         // end

//         // only write if write enabled
//         if (i_MemWrite == 1)
//             if (mask[3] == 1)
//                 d_mem[i_addr][31:24] <= shift_data[31:24]
//             if (mask[2] == 1)
//                 d_mem[i_addr][23:16] <= shift_data[23:16]
//             if (mask[1] == 1)
//                 d_mem[i_addr][15:8] <= shift_data[15:8]
//             if (mask[0] == 1)
//                 d_mem[i_addr][7:0] <= shift_data[7:0]

//     end

// endmodule

`default_nettype wire

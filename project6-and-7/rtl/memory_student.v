`default_nettype none

// data memory
module memory_student(
    input wire i_clk,
    output wire stall,
    input wire i_dmem_ready, //inserted here
    input wire i_dmem_valid, //inserted here
    // signals sent to data memory
    input wire [3:0] i_mask,
    input wire i_unsigned,
    input wire [31:0] i_mem_addr,
    input wire [31:0] i_reg2,
    // ALU signal
    input wire [31:0] i_result,
    // Branch and PC signals
    //input wire i_eq,
    //input wire i_slt,
    input wire [31:0] target_addr,
    input wire [31:0] i_PC,
    input wire [31:0] i_PC4,
    output wire [31:0] o_PC,
    //output wire [31:0] next_PC,
    // Results to choose between in WB stage
    output wire [31:0] read_data,
    output wire [31:0] read_alu,
    output wire [31:0] o_uimm,
    // input Mux signals
    input wire i_isJALR,
    input wire i_Jump,
    // input wire i_BranchEqual,
    // input wire i_BranchLT,
    // input wire i_Branch,
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
    input wire [31:0] i_dmem_wdata,
    input wire i_dmem_wen,
    input wire i_valid,
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
    output wire o_dmem_ren_retire,
    output wire o_dmem_wen,
    output wire [31:0] o_dmem_rdata,
    output wire o_valid,
    // forwarding signal
    output wire [31:0] forward_result
);

    reg read_req_sent_r = 1'b0; // Tracks whether a load request has been accepted.
    reg read_resp_seen_r = 1'b0; // Tracks whether the accepted load has completed.
    reg [31:0] read_data_r = 32'd0;

    // handle forwarding
    assign forward_result = i_Jump ? (i_PC + 32'd4) : (i_IsUInstruct ? i_uimm : i_result);

    // dmem read request:
    // send exactly once when memory is ready and a load is present,
    // then keep the stage stalled until the corresponding i_dmem_valid arrives.
    wire load_active;
    assign load_active = i_MemRead & i_valid;
    assign o_dmem_ren = load_active & ~read_req_sent_r & i_dmem_ready;
    assign stall = load_active & ~read_resp_seen_r;

    always @(posedge i_clk) begin
        if (!load_active) begin
            read_req_sent_r <= 1'b0;
            read_resp_seen_r <= 1'b0;
        end else begin
            if (o_dmem_ren) begin
                read_req_sent_r <= 1'b1;
            end
            if (i_dmem_valid && read_req_sent_r) begin
                read_data_r <= i_dmem_rdata;
                read_resp_seen_r <= 1'b1;
            end
        end
    end

    // ****** READ *******
    // only read if read-enabled
    // select bytes using the mask
    wire [31:0] data, masked_data;
    assign data = i_MemRead ? (i_dmem_valid ? i_dmem_rdata : read_data_r) : 32'd0; //inserted here
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
                    32'd0;

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

    // Retire instructions
    assign o_halt = i_halt;
    assign o_inst = i_inst;
    assign o_trapD = i_trapD;
    assign o_rs1 = i_rs1;
    assign o_rs2 = i_rs2;
    assign o_reg1 = i_reg1;
    assign o_reg2 = i_reg2_retire;
    assign o_trapX = i_trapX;
    assign o_valid = i_valid;

    assign o_dmem_wdata = i_dmem_wdata;
    assign o_dmem_wen = i_dmem_wen;
    assign o_dmem_mask = i_mask;
    assign o_dmem_addr = i_mem_addr;
    assign o_dmem_rdata = i_dmem_valid ? i_dmem_rdata : read_data_r; //inserted here
    assign o_dmem_ren_retire = i_MemRead & i_valid;

endmodule

`default_nettype wire
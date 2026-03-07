module writeback(
    input wire i_clk,
    input reg [31:0] i_PC,
    input reg [31:0] i_next_PC,
    // results to choose between
    input reg [31:0] read_data,
    input reg [31:0] read_alu,
    input reg [31:0] i_uimm,
    output wire [31:0] dest_result,
    output wire [31:0] o_PC,
    output reg [31:0] o_next_PC,
    // input mux signals
    input reg i_Jump,
    input reg i_MemtoReg,
    input reg [4:0] i_rd_waddr,
    input reg i_RegWrite,
    input reg i_IsUInstruct,
    // output signals
    output wire o_RegWrite,
    output wire [4:0] o_rd_waddr
);
    // determine value to write back
    assign dest_result = i_Jump ? (i_PC + 32'd4) : (i_IsUInstruct ? i_uimm : (i_MemtoReg ? read_data : read_alu));

    // pass through stage - write back is handled via connections in hart
    // remember, writing occurs before reading in decode cycle
    always(@posedge i_clk) begin
        o_next_PC <= i_next_PC;
    end

    assign o_PC = i_PC;
    assign o_RegWrite = i_RegWrite;
    assign o_rd_waddr = i_rd_waddr;

endmodule

`default_nettype wire

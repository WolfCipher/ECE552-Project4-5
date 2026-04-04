module exExForwarding (
    input wire [4:0] rs1_addr,
    input wire [4:0] rs2_addr,
    input wire [4:0] rd_addr_M,
    input wire RegWrite_M,
    input wire valid_M, // only forward from valid instructions
    input wire MemRead_M,
    output wire reg1_forward,
    output wire reg2_forward
);

    // Do not forward load results from M-stage (data not available until W stage)
    assign reg1_forward = (rs1_addr == rd_addr_M) & (rd_addr_M != 5'd0) & RegWrite_M & valid_M & ~MemRead_M;
    assign reg2_forward = (rs2_addr == rd_addr_M) & (rd_addr_M != 5'd0) & RegWrite_M & valid_M & ~MemRead_M;

endmodule

`default_nettype wire
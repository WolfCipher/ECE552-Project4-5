module memExForwarding (
    input wire [4:0] rs1_addr,
    input wire [4:0] rs2_addr,
    input wire [4:0] rd_addr_W,
    input wire RegWrite_W,
    output wire reg1_forward,
    output wire reg2_forward
);

    assign reg1_forward = (rs1_addr == rd_addr_W) & (rd_addr_W != 5'd0) & RegWrite_W;
    assign reg2_forward = (rs2_addr == rd_addr_W) & (rd_addr_W != 5'd0) & RegWrite_W;

endmodule

`default_nettype wire
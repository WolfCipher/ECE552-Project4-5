  
module fetch #(
    parameter RESET_ADDR = 32'h00000000
) (
    input i_rst, 
	input i_clk,
	input wire [31:0] i_imem_rdata, // data from mem
	input wire [31:0] next_pc,
	output wire [31:0] o_imem_raddr,
	output wire [31:0] pc_to_decode,
	output wire [31:0] instruction
);

	reg [31:0] pc;

	assign instruction = i_imem_rdata; // what the instruction says
	assign o_imem_raddr = pc;
	assign pc_to_decode = pc;

    always @(posedge i_clk) begin
        if (i_rst) pc <= RESET_ADDR; // if we need to rset set to base addr
        else pc <= next_pc; //otherwise just take the next instruciton that we found
    end


endmodule

`default_nettype wire


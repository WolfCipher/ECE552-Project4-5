`default_nettype none

module fetch #(
    parameter RESET_ADDR = 32'h00000000
) (
    input  wire        i_rst,
    input  wire        i_clk,
    input  wire        i_imem_ready,   //inserted here
    input  wire        i_imem_valid,   //inserted here
    input  wire [31:0] i_imem_rdata,
    input  wire [31:0] next_pc,
    input  wire        i_stall_F,      //inserted here
    input  wire        branch_taken,

    output wire [31:0] o_imem_raddr,
    output wire        o_imem_ren,     //inserted here
    output wire        o_fetch_wait,   //inserted here
    output wire [31:0] pc_to_decode,
    output wire [31:0] instruction
);

    reg [31:0] pc;
    reg [31:0] instruction_r;          //inserted here
    reg [31:0] pc_to_decode_r;         //inserted here
    reg        inst_valid_r;           //inserted here
    reg        req_outstanding_r;      //inserted here
    reg [31:0] redirect_pc_r;
    reg        redirect_pending_r;

    assign o_imem_raddr = pc;

    assign o_imem_ren =
        i_imem_ready &
        ~(req_outstanding_r & ~i_imem_valid) &
        ~inst_valid_r &
        ~i_stall_F;
        
    // A newly returned instruction can be forwarded immediately to decode,
    // except for wrong-path responses being squashed by redirect.
    wire squash_resp;
    wire resp_available;
    assign squash_resp = branch_taken | redirect_pending_r;
    assign resp_available = (i_imem_valid & ~squash_resp) | inst_valid_r;

    assign o_fetch_wait = ~resp_available; //inserted here

    assign instruction  = i_imem_valid ? i_imem_rdata : instruction_r; //inserted here
    assign pc_to_decode = i_imem_valid ? pc : pc_to_decode_r; //inserted here

    always @(posedge i_clk) begin
        if (i_rst) begin
            pc <= RESET_ADDR;
            instruction_r <= 32'h00000013;   //inserted here
            pc_to_decode_r <= RESET_ADDR;    //inserted here
            inst_valid_r <= 1'b0;            //inserted here
            req_outstanding_r <= 1'b0;       //inserted here
            redirect_pc_r <= RESET_ADDR;
            redirect_pending_r <= 1'b0;
        end else begin
            if (branch_taken) begin
                // Hold redirect target until fetch can actually advance PC.
                redirect_pc_r <= next_pc;
                redirect_pending_r <= 1'b1;
            end

            if (o_imem_ren) begin
                req_outstanding_r <= 1'b1;   //inserted here
            end

            if (i_imem_valid) begin
                req_outstanding_r <= 1'b0;       //inserted here
                if (squash_resp) begin
                    // Drop the wrong-path response and redirect fetch PC.
                    pc <= branch_taken ? next_pc : redirect_pc_r;
                    redirect_pending_r <= 1'b0;
                    inst_valid_r <= 1'b0;
                end else if (i_stall_F) begin
                    instruction_r <= i_imem_rdata;   //inserted here
                    pc_to_decode_r <= pc;            //inserted here
                    inst_valid_r <= 1'b1;            //inserted here
                end else begin
                    pc <= next_pc;
                    inst_valid_r <= 1'b0;           //inserted here
                end
            end else if (inst_valid_r && !i_stall_F) begin
                pc <= branch_taken ? next_pc : (redirect_pending_r ? redirect_pc_r : next_pc);
                if (branch_taken || redirect_pending_r) begin
                    redirect_pending_r <= 1'b0;
                end
                inst_valid_r <= 1'b0;            //inserted here
            end
        end
    end

endmodule

`default_nettype wire
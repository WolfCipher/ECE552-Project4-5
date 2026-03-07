
module decode (
    input wire i_clk,
    input wire [31:0] instruction,
    // output mux values
    output reg jump,
    output reg branch_eq,
    output reg branch_lt,
    output reg branch,
    output reg mem_read,
    output reg mem_write,
    output reg mem_to_reg,
    output reg alu_src, //1 if reg 0 if imm
    output reg reg_write,
    output reg is_u_instruct,
    output reg u_type,
    output reg is_jalr,
    // register and immediate values
    output reg [31:0] reg_data1,
    output reg [31:0] reg_data2,
    output reg [31:0] o_immediate,
    // ALU values
    output reg [2:0] funct3,
    output reg [2:0] i_opsel,
    output reg i_sub,
    output reg i_unsigned,
    output reg i_arith,
    // register addresses
    output wire [4:0] rs1_raddr,
    output wire [4:0] rs2_raddr,
    output reg [4:0] o_rd_waddr,
    // retire instruction handling
    output reg halt, // asserted if EBREAK
    output reg [31:0] o_retire_instruction,
    output reg trap,
    output reg [31:0] rs1_rdata,
    output reg [31:0] rs2_rdata,
    // output wire [4:0] rd_waddr,
    // output wire [31:0] rd_wdata,
    // PC values
    input wire [31:0] i_PC,
    output reg [31:0] o_PC,
    output reg [31:0] o_PC4
);

//control file
//opcode[6:0] is instruction[6:0]
wire [6:0] opcode;
assign opcode = instruction[6:0];
//funct3[2:0] is instruction[14:12]
always @(posedge i_clk) begin
    funct3 <= instruction[14:12];
end
//funct7[6:0] is instruction[31:25]
wire [6:0] funct7;
assign funct7 = instruction[31:25];

// Instruction format, determined by the instruction decoder based on the
    // opcode. This is one-hot encoded according to the following format:
    // [0] R-type
    // [1] I-type
    // [2] S-type
    // [3] B-type
    // [4] U-type
    // [5] J-type
wire [5:0] format;

// instruction type
wire is_r, is_i, is_s, is_b, is_u, is_j;
wire is_i_regular, is_i_jalr, is_i_load;

assign is_i_regular = (opcode == 7'b0010011);
assign is_i_jalr = (opcode == 7'b1100111);
assign is_i_load = (opcode == 7'b0000011);

assign is_r = (opcode == 7'b0110011);
assign is_i = is_i_regular || is_i_jalr || is_i_load;
assign is_s = (opcode == 7'b0100011);
assign is_b = (opcode == 7'b1100011);
assign is_u = (opcode == 7'b0010111) || (opcode == 7'b0110111);
assign is_j = (opcode == 7'b1101111);

wire r_valid, i_valid, s_valid, b_valid, u_valid, j_valid, ebreak_valid;
wire reg_i_valid, load_valid, jalr_valid;
wire valid;

assign reg_i_valid = is_i_regular && ((funct3 == 3'b001) ? (funct7 == 7'd0) :
                                    (funct3 == 3'b101) ? ((funct7 == 7'b0100000) || funct7 == 7'b0000000) : 1'b1);
assign load_valid = is_i_load && (funct3 != 3'b011) && (funct3 != 3'b110) && (funct3 != 3'b111);
assign jalr_valid = is_i_jalr && (funct3 == 3'b000);

assign r_valid = is_r && (funct7 == 7'b0000000 || (funct7 == 7'b0100000 && (funct3 == 3'b000 || funct3 == 3'b101)));
assign i_valid = reg_i_valid || jalr_valid || load_valid;
assign s_valid = is_s && (funct3 == 3'b000 || funct3 == 3'b001 || funct3 == 3'b010);
assign b_valid = is_b && (funct3 == 3'b000 || funct3 == 3'b001 || funct3 == 3'b100 || funct3 == 3'b101 || funct3 == 3'b110 || funct3 == 3'b111);
assign u_valid = is_u;
assign j_valid = is_j;
assign ebreak_valid = (instruction == 32'h00100073); // EBREAK instruction

assign valid = r_valid || i_valid || s_valid || b_valid || u_valid || j_valid || ebreak_valid;

assign format = is_r ? 6'b000001 : // R-Type
                (is_i) ? 6'b000010 : // I-Type
                (is_s) ? 6'b000100 : // S-Type
                (is_b) ? 6'b001000 : // B-Type
                (is_u) ? 6'b010000 : // U-Type
                (is_j) ? 6'b100000 : // J-Type
                6'b000000; // invalid instruction

assign o_PC = i_PC;
assign o_PC4 = i_PC + 31'd4;
assign rs1_raddr = instruction[19:15];
assign rs2_raddr = instruction[24:20];

// handle retire values
always @(posedge i_clk) begin
    halt <= ebreak_valid;
    o_retire_instruction <= instruction;
    rs1_rdata <= (rs1_raddr == 5'd0) ? 32'd0 : reg_data1;
    rs2_rdata <= (rs2_raddr == 5'd0) ? 32'd0 : reg_data2;
    trap <= ~valid;
end

// mux control signals
always @(posedge i_clk) begin
    // TODO: HANDLE FLUSHING HERE
    is_jalr <= jalr_valid;
    jump <= j_valid | jalr_valid; // J-Type or jalr
    branch_eq <= (funct3 == 3'b000) | (funct3 == 3'b101)  | (funct3 == 3'b111); // beq, bge(u)
    branch_lt <= (funct3 == 3'b110) | (funct3 == 3'b100) | (funct3 == 3'b001); // blt(u), bne
    branch <= b_valid; // is B-Type
    mem_read <= load_valid; // is load
    mem_write <= s_valid; // is S-Type
    mem_to_reg <= load_valid; // is load
    alu_src <= !(is_r || is_b); // choose imm unless R-Type or B-Type
    reg_write <= r_valid || i_valid || u_valid || j_valid; // write unless S-Type or B-Type
    i_opsel <= (r_valid || reg_i_valid) ? instruction[14:12] : 3'b000; // for R-Type and regular I-Type, use funct3; for other types, use ADD
    i_sub <= (is_r && funct7[5]) | is_b; // sub for R-type, subtract compare for branch
    i_arith <= funct7[5];
    i_unsigned <= (funct3 == 3'b110) | (funct3 == 3'b011) | (funct3 == 3'b111);
    is_u_instruct <= u_valid;
    u_type <= ~instruction[5];
end

// generate immediate
wire [31:0] imm;
imm i (instruction, format, imm);
always @(posedge i_clk) begin
    o_immediate <= imm;
end

// handle future register write
always @(posedge i_clk) begin
    o_rd_waddr <= instruction[11:7];
end

endmodule

`default_nettype wire

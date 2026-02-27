`default_nettype none

// The arithmetic logic unit (ALU) is responsible for performing the core
// calculations of the processor. It takes two 32-bit operands and outputs
// a 32 bit result based on the selection operation - addition, comparison,
// shift, or logical operation. This ALU is a purely combinational block, so
// you should not attempt to add any registers or pipeline it.
module alu (
    // NOTE: Both 3'b010 and 3'b011 are used for set less than operations and
    // your implementation should output the same result for both codes. The
    // reason for this will become clear in project 3.
    //
    // Major operation selection.
    // 3'b000: addition/subtraction if `i_sub` asserted
    // 3'b001: shift left logical
    // 3'b010,
    // 3'b011: set less than/unsigned if `i_unsigned` asserted
    // 3'b100: exclusive or
    // 3'b101: shift right logical/arithmetic if `i_arith` asserted
    // 3'b110: or
    // 3'b111: and
    input  wire [ 2:0] i_opsel,
    // When asserted, addition operations should subtract instead.
    // This is only used for `i_opsel == 3'b000` (addition/subtraction).
    input  wire        i_sub,
    // When asserted, comparison operations should be treated as unsigned.
    // This is used for branch comparisons and set less than unsigned. For
    // branch operations, the ALU result is not used, only the comparison
    // results.
    input  wire        i_unsigned,
    // When asserted, right shifts should be treated as arithmetic instead of
    // logical. This is only used for `i_opsel == 3'b101` (shift right).
    input  wire        i_arith,
    // First 32-bit input operand.
    input  wire [31:0] i_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_op2,
    // 32-bit output result. Any carry out should be ignored.
    output wire [31:0] o_result,
    // Equality result. This is used externally to determine if a branch
    // should be taken.
    output wire        o_eq,
    // Set less than result. This is used externally to determine if a branch
    // should be taken.
    output wire        o_slt
);

    wire [31:0] w_add, w_sll, w_slt, w_xor, w_sr, w_or, w_and, w_eq;

    adder sum (i_op1, i_op2, i_sub, w_add);
    sll shift (i_op1, i_op2[4:0], w_sll);
    slt set (i_op1, i_op2, i_unsigned, w_slt);
    xor32 xor1 (i_op1, i_op2, w_xor);
    sr shiftR (i_op1, i_op2[4:0], i_arith, w_sr);
    or32 or1 (i_op1, i_op2, w_or);
    and32 and1 (i_op1, i_op2, w_and);
    adder is_eq (i_op1, i_op2, 1'b1, w_eq);

    assign o_result = (i_opsel == 3'b000) ? w_add :
                      (i_opsel == 3'b001) ? w_sll :
                      (i_opsel == 3'b010) ? w_slt :
                      (i_opsel == 3'b011) ? w_slt :
                      (i_opsel == 3'b100) ? w_xor :
                      (i_opsel == 3'b101) ? w_sr :
                      (i_opsel == 3'b110) ? w_or :
                      (i_opsel == 3'b111) ? w_and :
                      32'b0;
    assign o_eq =     w_eq == 0;
    assign o_slt =    w_slt != 0;

endmodule

module adder(
    input wire [31:0] i_a,
    input wire [31:0] i_b,
    input wire i_carry,
    output wire [31:0] o_sum
);
    wire [31:0] flip_b;
    assign flip_b = i_b ^ {32{i_carry}};
    assign o_sum = i_a + flip_b + {31'd0, i_carry};
endmodule

module sll(
    input wire [31:0] i_a,
    input wire [4:0] i_b,
    output wire [31:0] result
);
    assign result = (i_b == 5'b00000) ? i_a :
                    (i_b == 5'b00001) ? {i_a[30:0], {1{1'b0}}} :
                    (i_b == 5'b00010) ? {i_a[29:0], {2{1'b0}}} :
                    (i_b == 5'b00011) ? {i_a[28:0], {3{1'b0}}} :
                    (i_b == 5'b00100) ? {i_a[27:0], {4{1'b0}}} :
                    (i_b == 5'b00101) ? {i_a[26:0], {5{1'b0}}} :
                    (i_b == 5'b00110) ? {i_a[25:0], {6{1'b0}}} :
                    (i_b == 5'b00111) ? {i_a[24:0], {7{1'b0}}} :
                    (i_b == 5'b01000) ? {i_a[23:0], {8{1'b0}}} :
                    (i_b == 5'b01001) ? {i_a[22:0], {9{1'b0}}} :
                    (i_b == 5'b01010) ? {i_a[21:0], {10{1'b0}}} :
                    (i_b == 5'b01011) ? {i_a[20:0], {11{1'b0}}} :
                    (i_b == 5'b01100) ? {i_a[19:0], {12{1'b0}}} :
                    (i_b == 5'b01101) ? {i_a[18:0], {13{1'b0}}} :
                    (i_b == 5'b01110) ? {i_a[17:0], {14{1'b0}}} :
                    (i_b == 5'b01111) ? {i_a[16:0], {15{1'b0}}} :
                    (i_b == 5'b10000) ? {i_a[15:0], {16{1'b0}}} :
                    (i_b == 5'b10001) ? {i_a[14:0], {17{1'b0}}} :
                    (i_b == 5'b10010) ? {i_a[13:0], {18{1'b0}}} :
                    (i_b == 5'b10011) ? {i_a[12:0], {19{1'b0}}} :
                    (i_b == 5'b10100) ? {i_a[11:0], {20{1'b0}}} :
                    (i_b == 5'b10101) ? {i_a[10:0], {21{1'b0}}} :
                    (i_b == 5'b10110) ? {i_a[9:0], {22{1'b0}}} :
                    (i_b == 5'b10111) ? {i_a[8:0], {23{1'b0}}} :
                    (i_b == 5'b11000) ? {i_a[7:0], {24{1'b0}}} :
                    (i_b == 5'b11001) ? {i_a[6:0], {25{1'b0}}} :
                    (i_b == 5'b11010) ? {i_a[5:0], {26{1'b0}}} :
                    (i_b == 5'b11011) ? {i_a[4:0], {27{1'b0}}} :
                    (i_b == 5'b11100) ? {i_a[3:0], {28{1'b0}}} :
                    (i_b == 5'b11101) ? {i_a[2:0], {29{1'b0}}} :
                    (i_b == 5'b11110) ? {i_a[1:0], {30{1'b0}}} :
                    {i_a[0:0], {31{1'b0}}};
endmodule

module slt(
    input wire [31:0] i_a,
    input wire [31:0] i_b,
    input wire i_unsigned,
    output wire [31:0] result
);
    wire a_sign, b_sign;
    assign a_sign = i_a[31];
    assign b_sign = i_b[31];

    wire signed_less_than;
    assign signed_less_than = (a_sign == b_sign) ?
                                i_a < i_b :         // compare magnitude
                                a_sign & ~b_sign;   // compare sign bits

    wire intermediate;
    assign intermediate = i_unsigned ? i_a < i_b : signed_less_than;

    assign result = {31'd0, intermediate};
endmodule

module xor32(
    input wire [31:0] i_a,
    input wire [31:0] i_b,
    output wire [31:0] result
);
    assign result = i_a ^ i_b;
endmodule

module sr(
    input wire [31:0] i_a,
    input wire [4:0] i_b,
    input wire i_arith,
    output wire [31:0] result
);
    wire sign;
    assign sign = i_arith & i_a[31];
    //assign result = {{i_b{sign}}, i_a[31:5'b0+i_b]};
    assign result = (i_b == 5'b00000) ? i_a :
                    (i_b == 5'b00001) ? {{1{sign}}, i_a[31:1]} :
                    (i_b == 5'b00010) ? {{2{sign}}, i_a[31:2]} :
                    (i_b == 5'b00011) ? {{3{sign}}, i_a[31:3]} :
                    (i_b == 5'b00100) ? {{4{sign}}, i_a[31:4]} :
                    (i_b == 5'b00101) ? {{5{sign}}, i_a[31:5]} :
                    (i_b == 5'b00110) ? {{6{sign}}, i_a[31:6]} :
                    (i_b == 5'b00111) ? {{7{sign}}, i_a[31:7]} :
                    (i_b == 5'b01000) ? {{8{sign}}, i_a[31:8]} :
                    (i_b == 5'b01001) ? {{9{sign}}, i_a[31:9]} :
                    (i_b == 5'b01010) ? {{10{sign}}, i_a[31:10]} :
                    (i_b == 5'b01011) ? {{11{sign}}, i_a[31:11]} :
                    (i_b == 5'b01100) ? {{12{sign}}, i_a[31:12]} :
                    (i_b == 5'b01101) ? {{13{sign}}, i_a[31:13]} :
                    (i_b == 5'b01110) ? {{14{sign}}, i_a[31:14]} :
                    (i_b == 5'b01111) ? {{15{sign}}, i_a[31:15]} :
                    (i_b == 5'b10000) ? {{16{sign}}, i_a[31:16]} :
                    (i_b == 5'b10001) ? {{17{sign}}, i_a[31:17]} :
                    (i_b == 5'b10010) ? {{18{sign}}, i_a[31:18]} :
                    (i_b == 5'b10011) ? {{19{sign}}, i_a[31:19]} :
                    (i_b == 5'b10100) ? {{20{sign}}, i_a[31:20]} :
                    (i_b == 5'b10101) ? {{21{sign}}, i_a[31:21]} :
                    (i_b == 5'b10110) ? {{22{sign}}, i_a[31:22]} :
                    (i_b == 5'b10111) ? {{23{sign}}, i_a[31:23]} :
                    (i_b == 5'b11000) ? {{24{sign}}, i_a[31:24]} :
                    (i_b == 5'b11001) ? {{25{sign}}, i_a[31:25]} :
                    (i_b == 5'b11010) ? {{26{sign}}, i_a[31:26]} :
                    (i_b == 5'b11011) ? {{27{sign}}, i_a[31:27]} :
                    (i_b == 5'b11100) ? {{28{sign}}, i_a[31:28]} :
                    (i_b == 5'b11101) ? {{29{sign}}, i_a[31:29]} :
                    (i_b == 5'b11110) ? {{30{sign}}, i_a[31:30]} :
                    {{31{sign}}, i_a[31:31]};
endmodule

module or32(
    input wire [31:0] i_a,
    input wire [31:0] i_b,
    output wire [31:0] result
);
    assign result = i_a | i_b;
endmodule

module and32(
    input wire [31:0] i_a,
    input wire [31:0] i_b,
    output wire [31:0] result
);
    assign result = i_a & i_b;
endmodule

`default_nettype wire

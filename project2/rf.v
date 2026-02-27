`default_nettype none

// The register file is effectively a single cycle memory with 32-bit words
// and depth 32. It has two asynchronous read ports, allowing two independent
// registers to be read at the same time combinationally, and one synchronous
// write port, allowing a register to be written to on the next clock edge.
// The register `x0` is hardwired to zero, and writes to it are ignored.
module rf #(
    // When this parameter is set to 1, "RF bypass" mode is enabled. This
    // allows data at the write port to be observed at the read ports
    // immediately without having to wait for the next clock edge. This is
    // a common forwarding optimization in a pipelined core (project 5), but
    // will cause a single-cycle processor to behave incorrectly.
    //
    // You are required to implement and test both modes. In project 3 and 4,
    // you will set this to 0, before enabling it in project 5.
    parameter BYPASS_EN = 0
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Both read register ports are asynchronous (zero-cycle). That is, read
    // data is visible combinationally without having to wait for a clock.
    //
    // Register read port 1, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs1_raddr,
    output wire [31:0] o_rs1_rdata,
    // Register read port 2, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs2_raddr,
    output wire [31:0] o_rs2_rdata,
    // The register write port is synchronous. When write is enabled, the
    // write data is visible after the next clock edge.
    //
    // Write register enable, address [0, 31] and input data.
    input  wire        i_rd_wen,
    input  wire [ 4:0] i_rd_waddr,
    input  wire [31:0] i_rd_wdata
);
    // TODO: Fill in your implementation here.
    // register file
    reg [31:0] reg_file [31:0];

    // ASYNCHRONOUS
    // allow a bypass mode allowing data at the write port to be observed at the read port immediately
    wire bypass1, bypass2;
    assign bypass1 = BYPASS_EN ? (i_rs1_raddr == i_rd_waddr) && (i_rd_waddr != 0) : 0;
    assign bypass2 = BYPASS_EN ? (i_rs2_raddr == i_rd_waddr) && (i_rd_waddr != 0) : 0;

    // always read registers
    assign o_rs1_rdata = bypass1 ? i_rd_wdata : reg_file[i_rs1_raddr];
    assign o_rs2_rdata = bypass2 ? i_rd_wdata : reg_file[i_rs2_raddr];

    integer i;

    // SYNCHRONOUS
    always @(posedge i_clk) begin
        // handle active-high reset
        if (i_rst == 1) begin
            reg_file[0] <= 32'd0;
            reg_file[1] <= 32'd0;
            reg_file[2] <= 32'd0;
            reg_file[3] <= 32'd0;
            reg_file[4] <= 32'd0;
            reg_file[5] <= 32'd0;
            reg_file[6] <= 32'd0;
            reg_file[7] <= 32'd0;
            reg_file[8] <= 32'd0;
            reg_file[9] <= 32'd0;
            reg_file[10] <= 32'd0;
            reg_file[11] <= 32'd0;
            reg_file[12] <= 32'd0;
            reg_file[13] <= 32'd0;
            reg_file[14] <= 32'd0;
            reg_file[15] <= 32'd0;
            reg_file[16] <= 32'd0;
            reg_file[17] <= 32'd0;
            reg_file[18] <= 32'd0;
            reg_file[19] <= 32'd0;
            reg_file[20] <= 32'd0;
            reg_file[21] <= 32'd0;
            reg_file[22] <= 32'd0;
            reg_file[23] <= 32'd0;
            reg_file[24] <= 32'd0;
            reg_file[25] <= 32'd0;
            reg_file[26] <= 32'd0; 
            reg_file[27] <= 32'd0;
            reg_file[28] <= 32'd0;
            reg_file[29] <= 32'd0;
            reg_file[30] <= 32'd0;
            reg_file[31] <= 32'd0;
        end

        // only write if write enabled
        // ignore writes to register zero
        if (i_rd_wen == 1)
            if(i_rd_waddr != 0)
                reg_file[i_rd_waddr] <= i_rd_wdata;

    end

endmodule

`default_nettype wire

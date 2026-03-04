module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available on the same cycle.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002000`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
	// The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);

    // PC signals
    wire [31:0] PC_F_D, PC_D_X, PC_X_M, PC_M_W; // before adding 4
    wire [31:0] PC4_D_X, PC4_X_M, PC4_M_W; // after adding 4
    wire [31:0] target_addr_X_M; // PC + target_addr
    wire [31:0] next_PC_M_W, next_PC_W_F; // output of branch/jump logic

    // Mux Signals
    wire isJALR_D_X, isJALR_X_M;
    wire Jump_D_X, Jump_X_M, Jump_M_W;
    wire BranchEqual_D_X, BranchEqual_X_M;
    wire BranchLT_D_X, BranchLT_X_M;
    wire Branch_D_X, Branch_X_M;
    wire MemRead_D_X, MemRead_X_M; // TODO: replace last signal with o_dmem_ren
    wire MemtoReg_D_X, MemtoReg_X_M, MemtoReg_M_W;
    wire MemWrite_D_X; //MemWrite_X_M; // TODO: replace last signal with o_dmem_wen
    wire RegWrite_D_X, RegWrite_X_M, RegWrite_M_W;
    wire UpperType_D_X;
    wire IsUInstruct_D_X, IsUInstruct_X_M, IsUInstruct_M_W;
    wire ALUSrc_D_X;

    // Destination Address
    wire [4:0] rd_waddr_D_X, rd_waddr_X_M, rd_waddr_M_W;

    // register access signals
    wire i_reg_write_en, wb_en;
    wire [4:0] i_reg_write_addr, wb_addr;
    wire [31:0] i_reg_write_data, wb_data;
    wire [4:0] rs1_raddr, rs2_raddr;

    assign i_reg_write_en = wb_en;
    assign i_reg_write_addr = wb_addr;
    assign i_reg_write_data = wb_data;

    // ALU result, U type result, memory result
    wire [31:0] ALU_X_M, ALU_M_W;
    wire [31:0] uimm_X_M, uimm_M_W;
    wire [31:0] mem_read_M_W; // TODO replace with i_dem_rdata

    // Signals just between decode and execute stages
    wire [31:0] reg1, reg2, imm;
    wire [2:0] funct3, i_opsel;
    wire i_sub, i_unsigned, i_arith;

    // Signals just between execute and memory
    wire eq, slt, mem_unsigned;
    wire [31:0] mem_addr, reg2_X_M; // TODO replace with o_dem_addr, o_dem_wdata
    assign mem_addr = o_dmem_addr;

    // **** HANDLE RETIRE *******
    // for single-cycle implementations, o_retire_valid will always be 1
    assign o_retire_valid = 1;
    // check for traps in stages where we can find a bad instruction and bad addresses
    wire trap_D, trap_X;
    assign o_retire_trap = trap_D | trap_X;
    // retired instruction pc
    assign o_retire_next_pc = next_PC_W_F;
    // register addresses
    assign o_retire_rs1_raddr = rs1_raddr;
    assign o_retire_rs2_raddr = rs2_raddr;
    //assign o_retire_rd_waddr = i_reg_write_addr;
    //assign o_retire_rd_wdata = i_reg_write_data;
    assign o_retire_rd_waddr = wb_en ? wb_addr : 5'd0;
    assign o_retire_rd_wdata =
        (wb_en && wb_addr != 5'd0)
            ? wb_data
            : 32'd0;


    // TODO take action if the retired instruction is valid
    // wire next_pc;
    // assign next_pc = o_retire_valid ? o_retire_next_pc : 32'd0; // TODO: what should default value be?

    // Additional wires
    wire [31:0] instruction;    // never declared

    rf #(0) reg_file (
        i_clk, i_rst,
        // Register read port 1, with input address [0, 31] and output data.
        rs1_raddr, reg1,
        // Register read port 2, with input address [0, 31] and output data.
        rs2_raddr, reg2,
        // Write register enable, address [0, 31] and input data.
        i_reg_write_en, i_reg_write_addr, i_reg_write_data
    );

    // ***** BUILD CONNECTIONS *****
    fetch #(RESET_ADDR) fetch_inst (
        i_rst,
        i_clk,
        i_imem_rdata,
        next_PC_W_F,
        o_imem_raddr,
        PC_F_D,
        instruction
    );


    decode d (
        i_imem_rdata,
        // output mux values
        Jump_D_X, BranchEqual_D_X, BranchLT_D_X, Branch_D_X,
        MemRead_D_X, MemWrite_D_X, MemtoReg_D_X,
        ALUSrc_D_X, //1 if reg 0 if imm
        RegWrite_D_X, IsUInstruct_D_X, UpperType_D_X, isJALR_D_X,
        // register and immediate values
        reg1, reg2, imm,
        // ALU values
        funct3, i_opsel, i_sub, i_unsigned, i_arith,
        // Register accesses
        rs1_raddr, rs2_raddr, rd_waddr_D_X,
        // Retire instructions
        o_retire_halt, o_retire_inst, trap_D,
        o_retire_rs1_rdata, o_retire_rs2_rdata,
        //o_retire_rd_waddr, o_retire_rd_wdata,
        // PC
        PC_F_D, PC_D_X, PC4_D_X
    );

    execute x (
        // ALU inputs
        reg1, reg2, imm, funct3, i_opsel, i_sub, i_unsigned, i_arith,
        // signals related to PC, branch, and ALU
        PC_D_X, PC4_D_X, ALU_X_M, eq, slt, target_addr_X_M, PC_X_M, PC4_X_M,
        // signals for proper memory access
        mem_unsigned, o_dmem_mask, o_dmem_addr, o_dmem_wdata, reg2_X_M,
        // input mux signals
        ALUSrc_D_X, isJALR_D_X, Jump_D_X, BranchEqual_D_X, BranchLT_D_X, Branch_D_X,
        MemRead_D_X, MemtoReg_D_X, MemWrite_D_X, rd_waddr_D_X,
        RegWrite_D_X, UpperType_D_X, IsUInstruct_D_X,
        // output mux signals
        isJALR_X_M, Jump_X_M, BranchEqual_X_M, BranchLT_X_M, Branch_X_M,
        MemRead_X_M, MemtoReg_X_M, o_dmem_wen,
        rd_waddr_X_M, RegWrite_X_M, IsUInstruct_X_M,
        // U type result
        uimm_X_M,
        // trap check
        trap_X
    );

    memory m (
        // signals sent to data memory
        i_clk, o_dmem_mask, mem_unsigned, mem_addr, reg2_X_M,
        // ALU signal
        ALU_X_M,
        // Branch and PC signals
        eq, slt, target_addr_X_M, PC_X_M, PC4_X_M, PC_M_W, next_PC_M_W,
        // Results to choose between in WB stage
        mem_read_M_W, ALU_M_W, uimm_M_W,
        // input Mux signals
        isJALR_X_M, Jump_X_M, BranchEqual_X_M, BranchLT_X_M, Branch_X_M,
        MemRead_X_M, MemtoReg_X_M, //MemWrite_X_M,
        rd_waddr_X_M, RegWrite_X_M, IsUInstruct_X_M,
        uimm_X_M,
        // output Mux signals
        Jump_M_W, MemtoReg_M_W, rd_waddr_M_W, RegWrite_M_W, IsUInstruct_M_W,
        // dmem
        i_dmem_rdata, o_dmem_ren
    );


    writeback w (
        PC_M_W,
        next_PC_M_W,
        // results to choose between
        mem_read_M_W, ALU_M_W, uimm_M_W,
        wb_data,
        o_retire_pc,
        next_PC_W_F,
        // input mux signals
        Jump_M_W, MemtoReg_M_W, rd_waddr_M_W,
        RegWrite_M_W, IsUInstruct_M_W,
        // output signals
        wb_en,
        wb_addr
    );

endmodule

`default_nettype wire

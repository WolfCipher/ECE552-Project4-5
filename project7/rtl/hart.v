`default_nettype none

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
    // per cycle and sequentially returns a 32-bit instruction word. For
    // projects 6 and 7, this memory has been updated to be more realistic
    // - reads are no longer combinational, and both read and write accesses
    // take multiple cycles to complete.
    //
    // The testbench memory models a fixed, multi cycle memory with partial
    // pipelining. The memory will accept a new request every N cycles by
    // asserting `mem_ready`, and if a request is made, the memory perform
    // the request (read or write) after M cycles, asserting mem_valid to
    // indicate the read data is ready (or the write is complete). Requests
    // are completed in order. The values of N and M are deterministic, but
    // may change between test cases - you must design your CPU to work
    // correctly by looking at `mem_ready` and `mem_valid` rather than
    // hardcoding a latency assumption.
    //
    // Indicates that the memory is ready to accept a new read request.
    input  wire        i_imem_ready,
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Issue a read request to the memory on this cycle. This should not be
    // asserted if `i_imem_ready` is not asserted.
    output wire        o_imem_ren,
    // Indicates that a valid instruction word is being returned from memory.
    input  wire        i_imem_valid,
    // Instruction word fetched from memory, available sequentially some
    // M cycles after a request (imem_ren) is issued.
    input  wire [31:0] i_imem_rdata,

    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle.
    //
    // The timing of the dmem interface is the same as the imem interface. See
    // the documentation above.
    //
    // Indicates that the memory is ready to accept a new read or write request.
    input  wire        i_dmem_ready,
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
    // address. It is illegal to assert this and `o_dmem_ren` on the same
    // cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned word at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // the processor supports byte and half-word loads and stores at unaligned
    // and 16-bit aligned addresses, respectively. To support this, the access
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
    // Indicates that a valid data word is being returned from memory.
    input  wire        i_dmem_valid,
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
    output wire [31:0] o_retire_dmem_addr,
    output wire [ 3:0] o_retire_dmem_mask,
    output wire        o_retire_dmem_ren,
    output wire        o_retire_dmem_wen,
    output wire [31:0] o_retire_dmem_rdata,
    output wire [31:0] o_retire_dmem_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc
);
    
    // PC signals
    reg [31:0] PC_F_D_r, PC_D_X_r, PC_X_M_r, PC_M_W_r; // before adding 4
    reg [31:0] PC4_D_X_r, PC4_X_M_r, PC4_M_W_r; // after adding 4
    reg [31:0] target_addr_D_X_r; // PC + target_addr
    reg [31:0] next_PC_M_W_r; // output of branch/jump logic
    
    wire [31:0] PC_F_D_w, PC_D_X_w, PC_X_M_w, PC_M_W_w; // before adding 4
    wire [31:0] PC4_D_X_w, PC4_X_M_w, PC4_M_W_w; // after adding 4
    wire [31:0] target_addr_D_X_w; // PC + target_addr
    wire [31:0] next_PC_M_W_w, next_PC_W_F; // output of branch/jump logic

    // Mux Signals
    reg isJALR_D_X_r, isJALR_X_M_r;
    reg Jump_D_X_r, Jump_X_M_r, Jump_M_W_r;
    reg BranchEqual_D_X_r, BranchEqual_X_M_r;
    reg BranchLT_D_X_r, BranchLT_X_M_r;
    reg Branch_D_X_r, Branch_X_M_r;
    reg MemRead_D_X_r, MemRead_X_M_r;
    reg MemtoReg_D_X_r, MemtoReg_X_M_r, MemtoReg_M_W_r;
    reg MemWrite_D_X_r;
    reg RegWrite_D_X_r, RegWrite_X_M_r, RegWrite_M_W_r;
    reg UpperType_D_X_r;
    reg IsUInstruct_D_X_r, IsUInstruct_X_M_r, IsUInstruct_M_W_r;
    reg ALUSrc_D_X_r;

    wire isJALR_D_X_w, isJALR_X_M_w;
    wire Jump_D_X_w, Jump_X_M_w, Jump_M_W_w;
    wire BranchEqual_D_X_w;
    wire BranchLT_D_X_w;
    wire Branch_D_X_w;
    wire MemRead_D_X_w, MemRead_X_M_w;
    wire MemtoReg_D_X_w, MemtoReg_X_M_w, MemtoReg_M_W_w;
    wire MemWrite_D_X_w;
    wire RegWrite_D_X_w, RegWrite_X_M_w, RegWrite_M_W_w;
    wire UpperType_D_X_w;
    wire IsUInstruct_D_X_w, IsUInstruct_X_M_w, IsUInstruct_M_W_w;
    wire ALUSrc_D_X_w;

    // Destination Address
    reg [4:0] rd_waddr_D_X_r, rd_waddr_X_M_r, rd_waddr_M_W_r;
    wire [4:0] rd_waddr_D_X_w, rd_waddr_X_M_w, rd_waddr_M_W_w;

    // register access signals
    wire i_reg_write_en, wb_en;
    wire [4:0] i_reg_write_addr, wb_addr;
    wire [31:0] i_reg_write_data, wb_data;

    assign i_reg_write_en = wb_en;
    assign i_reg_write_addr = wb_addr;
    assign i_reg_write_data = wb_data;

    // ALU result, U type result, memory result
    reg [31:0] ALU_X_M_r, ALU_M_W_r;
    reg [31:0] uimm_X_M_r, uimm_M_W_r;
    reg [31:0] mem_read_M_W_r;

    wire [31:0] ALU_X_M_w, ALU_M_W_w;
    wire [31:0] uimm_X_M_w, uimm_M_W_w;
    wire [31:0] mem_read_M_W_w;

    // Signals just between decode and execute stages
    reg [31:0] reg1_r, reg2_r, imm_r;
    reg [2:0] funct3_r, i_opsel_r;
    reg i_sub_r, i_unsigned_r, i_arith_r;

    wire [31:0] reg1_w, reg2_w, imm_w;
    wire [2:0] funct3_w, i_opsel_w;
    wire i_sub_w, i_unsigned_w, i_arith_w;
    
    // Signals just between execute and memory
    reg eq_r, slt_r, mem_unsigned_r;
    reg [31:0] dmem_addr_r, reg2_X_M_r;
    reg [3:0] dmem_mask_r;

    wire eq_w, slt_w, mem_unsigned_w;
    wire [31:0] dmem_addr_w, reg2_X_M_w;
    wire [3:0] dmem_mask_w;

    wire [31:0] dmem_wdata_ex_w;
    wire        dmem_wen_ex_w;
    reg [31:0] dmem_wdata_ex_r;
    reg        dmem_wen_ex_r;

    //inserted here
    wire fetch_wait;

    // Drive the external dmem interface from the MEM stage (registered) signals.
    assign o_dmem_addr  = dmem_addr_r;
    assign o_dmem_mask  = dmem_mask_r;
    assign o_dmem_wdata = dmem_wdata_ex_r;
    assign o_dmem_wen   = dmem_wen_ex_r;

    wire stall_F, stall_D, stall_X, stall_M;
    wire stall_M_X, stall_M_X_D, stall_M_X_D_F;

    assign stall_M_X = stall_M | stall_X;
    assign stall_M_X_D = stall_M_X | stall_D;
    assign stall_M_X_D_F = stall_M_X_D | stall_F; //inserted here

    wire stall_D_from_F; //inserted here
    assign stall_D_from_F = stall_M_X | stall_F; //inserted here

    // **** FORWARDING SIGNALS *****
    wire reg1_ex_forward;
    wire reg2_ex_forward;
    wire [31:0] exExForwardingResult;
    wire reg1_mem_forward;
    wire reg2_mem_forward;
    
    // **** HANDLE RETIRE *******
    reg  valid_F_D_r;
    reg  valid_D_X_r, valid_X_M_r, valid_M_W_r;
    wire valid_F_D_w;
    wire valid_D_X_w, valid_X_M_w, valid_M_W_w;
    wire valid_W_F;

    //assign stall_X = valid_D_X_r & MemWrite_D_X_r & ~i_dmem_ready; //inserted here
    //assign stall_M = valid_X_M_r & MemRead_X_M_r & ~i_dmem_valid; //inserted here
    assign stall_F = fetch_wait; //inserted here

    assign valid_F_D_w = ~fetch_wait;
    // Allow the current decoded instruction to advance even when fetch is waiting.
    assign valid_D_X_w = valid_F_D_r & ~stall_D_from_F;
    // valid_X_M_w and valid_M_W_w are driven by execute and memory_student module outputs
    assign o_retire_valid = valid_W_F;

    // TRAP
    reg trapD_D_X_r, trapD_X_M_r, trapD_M_W_r;
    reg trapX_X_M_r, trapX_M_W_r;

    wire trapD_D_X_w, trapD_X_M_w, trapD_M_W_w;
    wire trapX_X_M_w, trapX_M_W_w;

    wire trapD_W_F, trapX_W_F;

    assign o_retire_trap = valid_W_F & (trapD_W_F | trapX_W_F);

    // PC
    assign o_retire_next_pc = next_PC_W_F;

    // Register Source
    reg [4:0] rs1_raddr_D_X_r, rs1_raddr_X_M_r, rs1_raddr_M_W_r;
    reg [4:0] rs2_raddr_D_X_r, rs2_raddr_X_M_r, rs2_raddr_M_W_r;
    reg [31:0] rs1_rdata_D_X_r, rs1_rdata_X_M_r, rs1_rdata_M_W_r;
    reg [31:0] rs2_rdata_D_X_r, rs2_rdata_X_M_r, rs2_rdata_M_W_r;

    wire [4:0] rs1_raddr_D_X_w, rs1_raddr_X_M_w, rs1_raddr_M_W_w;
    wire [4:0] rs2_raddr_D_X_w, rs2_raddr_X_M_w, rs2_raddr_M_W_w;
    wire [31:0] rs1_rdata_D_X_w, rs1_rdata_X_M_w, rs1_rdata_M_W_w;
    wire [31:0] rs2_rdata_D_X_w, rs2_rdata_X_M_w, rs2_rdata_M_W_w;

    wire [4:0] rs1_raddr_W_F, rs2_raddr_W_F;
    wire [31:0] rs1_rdata_W_F, rs2_rdata_W_F;

    assign o_retire_rs1_raddr = rs1_raddr_W_F;
    assign o_retire_rs2_raddr = rs2_raddr_W_F;
    assign o_retire_rs1_rdata = rs1_rdata_W_F;
    assign o_retire_rs2_rdata = rs2_rdata_W_F;

    // Register Destination
    assign o_retire_rd_waddr = wb_en ? wb_addr : 5'd0;
    assign o_retire_rd_wdata =
        (wb_en && wb_addr != 5'd0)
            ? wb_data
            : 32'd0;

    // DMEM
    reg [31:0] dmem_addr_X_M_r, dmem_addr_M_W_r;
    reg [31:0] dmem_rdata_X_M_r, dmem_rdata_M_W_r;
    reg dmem_ren_X_M_r, dmem_ren_M_W_r;
    reg dmem_wen_X_M_r, dmem_wen_M_W_r;
    reg [3:0] dmem_mask_X_M_r, dmem_mask_M_W_r;
    reg [31:0] dmem_wdata_X_M_r, dmem_wdata_M_W_r;

    wire [31:0] dmem_addr_X_M_w, dmem_addr_M_W_w;
    wire [31:0] dmem_rdata_X_M_w, dmem_rdata_M_W_w;
    wire dmem_ren_X_M_w, dmem_ren_M_W_w;
    wire dmem_wen_X_M_w, dmem_wen_M_W_w;
    wire [3:0] dmem_mask_X_M_w, dmem_mask_M_W_w;
    wire [31:0] dmem_wdata_X_M_w, dmem_wdata_M_W_w;

    wire [31:0] dmem_addr_W_F;
    wire [3:0] dmem_mask_W_F;
    wire [31:0] dmem_rdata_W_F, dmem_wdata_W_F;
    wire dmem_ren_W_F, dmem_wen_W_F;

    assign o_retire_dmem_addr = dmem_addr_W_F;
    assign o_retire_dmem_ren = dmem_ren_W_F;
    assign o_retire_dmem_wen = dmem_wen_W_F;
    assign o_retire_dmem_mask = dmem_mask_W_F;
    assign o_retire_dmem_wdata = dmem_wdata_W_F;
    assign o_retire_dmem_rdata = dmem_rdata_W_F;

    // HALT
    reg halt_D_X_r, halt_X_M_r, halt_M_W_r;
    wire halt_D_X_w, halt_X_M_w, halt_M_W_w;
    wire halt_W_F;

    assign o_retire_halt = valid_W_F & halt_W_F;

    // INSTRUCTION
    reg [31:0] inst_D_X_r, inst_X_M_r, inst_M_W_r;
    wire [31:0] inst_D_X_w, inst_X_M_w, inst_M_W_w;
    wire [31:0] inst_W_F;
    assign o_retire_inst = inst_W_F;

    // Additional wires
    reg [31:0] instruction_r;
    wire [31:0] instruction_w;

    wire [31:0] next_PC_to_fetch;

    wire branch_taken;

    wire redirect_X;
    assign redirect_X = valid_D_X_r & (Jump_D_X_r | branch_taken);

    wire flush;
    assign flush = redirect_X;

    wire [31:0] redirect_target_X;
    assign redirect_target_X = isJALR_D_X_r ? {target_addr_D_X_w[31:1], 1'b0} : target_addr_D_X_w;

    assign next_PC_to_fetch = redirect_X ? redirect_target_X : (o_imem_raddr + 32'd4);
    always @(posedge i_clk) begin
        if (i_rst) begin
            PC_F_D_r <= 32'd0;
            PC_D_X_r <= 32'd0;
            PC_X_M_r <= 32'd0;
            PC_M_W_r <= 32'd0;
            PC4_D_X_r <= 32'd0;
            PC4_X_M_r <= 32'd0;
            PC4_M_W_r <= 32'd0;
            target_addr_D_X_r <= 32'd0;
            next_PC_M_W_r <= RESET_ADDR;

            valid_F_D_r <= 1'b0;
            valid_D_X_r <= 1'b0;
            valid_X_M_r <= 1'b0;
            valid_M_W_r <= 1'b0;

            isJALR_D_X_r <= 1'b0;
            isJALR_X_M_r <= 1'b0;
            Jump_D_X_r <= 1'b0;
            Jump_X_M_r <= 1'b0;
            Jump_M_W_r <= 1'b0;
            BranchEqual_D_X_r <= 1'b0;
            BranchEqual_X_M_r <= 1'b0;
            BranchLT_D_X_r <= 1'b0;
            BranchLT_X_M_r <= 1'b0;
            Branch_D_X_r <= 1'b0;
            Branch_X_M_r <= 1'b0;
            MemRead_D_X_r <= 1'b0;
            MemRead_X_M_r <= 1'b0;
            MemtoReg_D_X_r <= 1'b0;
            MemtoReg_X_M_r <= 1'b0;
            MemtoReg_M_W_r <= 1'b0;
            MemWrite_D_X_r <= 1'b0;
            RegWrite_D_X_r <= 1'b0;
            RegWrite_X_M_r <= 1'b0;
            RegWrite_M_W_r <= 1'b0;
            UpperType_D_X_r <= 1'b0;
            IsUInstruct_D_X_r <= 1'b0;
            IsUInstruct_X_M_r <= 1'b0;
            IsUInstruct_M_W_r <= 1'b0;
            ALUSrc_D_X_r <= 1'b0;

            rd_waddr_D_X_r <= 5'd0;
            rd_waddr_X_M_r <= 5'd0;
            rd_waddr_M_W_r <= 5'd0;

            ALU_X_M_r <= 32'd0;
            ALU_M_W_r <= 32'd0;
            uimm_X_M_r <= 32'd0;
            uimm_M_W_r <= 32'd0;
            mem_read_M_W_r <= 32'd0;

            reg1_r <= 32'd0;
            reg2_r <= 32'd0;
            imm_r <= 32'd0;
            funct3_r <= 3'd0;
            i_opsel_r <= 3'd0;
            i_sub_r <= 1'b0;
            i_unsigned_r <= 1'b0;
            i_arith_r <= 1'b0;

            eq_r <= 1'b0;
            slt_r <= 1'b0;
            mem_unsigned_r <= 1'b0;
            dmem_addr_r <= 32'd0;
            dmem_mask_r <= 4'd0;
            reg2_X_M_r <= 32'd0;

            trapD_D_X_r <= 1'b0;
            trapD_X_M_r <= 1'b0;
            trapD_M_W_r <= 1'b0;
            trapX_X_M_r <= 1'b0;
            trapX_M_W_r <= 1'b0;

            rs1_raddr_D_X_r <= 5'd0;
            rs1_raddr_X_M_r <= 5'd0;
            rs1_raddr_M_W_r <= 5'd0;
            rs2_raddr_D_X_r <= 5'd0;
            rs2_raddr_X_M_r <= 5'd0;
            rs2_raddr_M_W_r <= 5'd0;
            rs1_rdata_D_X_r <= 32'd0;
            rs1_rdata_X_M_r <= 32'd0;
            rs1_rdata_M_W_r <= 32'd0;
            rs2_rdata_D_X_r <= 32'd0;
            rs2_rdata_X_M_r <= 32'd0;
            rs2_rdata_M_W_r <= 32'd0;

            dmem_addr_X_M_r <= 32'd0;
            dmem_addr_M_W_r <= 32'd0;
            dmem_rdata_X_M_r <= 32'd0;
            dmem_rdata_M_W_r <= 32'd0;
            dmem_ren_X_M_r <= 1'b0;
            dmem_ren_M_W_r <= 1'b0;
            dmem_wen_X_M_r <= 1'b0;
            dmem_wen_M_W_r <= 1'b0;
            dmem_mask_X_M_r <= 4'd0;
            dmem_mask_M_W_r <= 4'd0;
            dmem_wdata_X_M_r <= 32'd0;
            dmem_wdata_M_W_r <= 32'd0;
            dmem_wdata_ex_r <= 32'd0;
            dmem_wen_ex_r <= 1'b0;

            halt_D_X_r <= 1'b0;
            halt_X_M_r <= 1'b0;
            halt_M_W_r <= 1'b0;
            inst_D_X_r <= 32'd0;
            inst_X_M_r <= 32'd0;
            inst_M_W_r <= 32'd0;
            instruction_r <= 32'h00000013;

        end else begin
            // F/D
            valid_F_D_r   <= flush ? 1'b0 : (stall_M_X_D_F ? valid_F_D_r : valid_F_D_w); //inserted here
            PC_F_D_r      <= flush ? 32'd0 : (stall_M_X_D_F ? PC_F_D_r : PC_F_D_w); //inserted here
            instruction_r <= flush ? 32'h00000013 : (stall_M_X_D_F ? instruction_r : instruction_w); //inserted here

            // D/X : hold on stall_M_X, bubble on stall_D, clear on flush
            RegWrite_D_X_r    <= flush ? 1'b0 : (stall_M_X ? RegWrite_D_X_r    : (stall_D_from_F ? 1'b0  : RegWrite_D_X_w)); //inserted here
            MemRead_D_X_r     <= flush ? 1'b0 : (stall_M_X ? MemRead_D_X_r     : (stall_D_from_F ? 1'b0  : MemRead_D_X_w)); //inserted here
            MemWrite_D_X_r    <= flush ? 1'b0 : (stall_M_X ? MemWrite_D_X_r    : (stall_D_from_F ? 1'b0  : MemWrite_D_X_w)); //inserted here
            Jump_D_X_r        <= flush ? 1'b0 : (stall_M_X ? Jump_D_X_r        : (stall_D_from_F ? 1'b0  : Jump_D_X_w)); //inserted here
            Branch_D_X_r      <= flush ? 1'b0 : (stall_M_X ? Branch_D_X_r      : (stall_D_from_F ? 1'b0  : Branch_D_X_w)); //inserted here
            BranchEqual_D_X_r <= flush ? 1'b0 : (stall_M_X ? BranchEqual_D_X_r : (stall_D_from_F ? 1'b0  : BranchEqual_D_X_w)); //inserted here
            BranchLT_D_X_r    <= flush ? 1'b0 : (stall_M_X ? BranchLT_D_X_r    : (stall_D_from_F ? 1'b0  : BranchLT_D_X_w)); //inserted here
            MemtoReg_D_X_r    <= flush ? 1'b0 : (stall_M_X ? MemtoReg_D_X_r    : (stall_D_from_F ? 1'b0  : MemtoReg_D_X_w)); //inserted here
            isJALR_D_X_r      <= flush ? 1'b0 : (stall_M_X ? isJALR_D_X_r      : (stall_D_from_F ? 1'b0  : isJALR_D_X_w)); //inserted here
            rd_waddr_D_X_r    <= flush ? 5'd0 : (stall_M_X ? rd_waddr_D_X_r    : (stall_D_from_F ? 5'd0  : rd_waddr_D_X_w)); //inserted here
            halt_D_X_r        <= flush ? 1'b0 : (stall_M_X ? halt_D_X_r        : (stall_D_from_F ? 1'b0  : halt_D_X_w)); //inserted here
            trapD_D_X_r       <= flush ? 1'b0 : (stall_M_X ? trapD_D_X_r       : (stall_D_from_F ? 1'b0  : trapD_D_X_w)); //inserted here
            IsUInstruct_D_X_r <= flush ? 1'b0 : (stall_M_X ? IsUInstruct_D_X_r : (stall_D_from_F ? 1'b0  : IsUInstruct_D_X_w)); //inserted here
            UpperType_D_X_r   <= flush ? 1'b0 : (stall_M_X ? UpperType_D_X_r   : (stall_D_from_F ? 1'b0  : UpperType_D_X_w)); //inserted here
            ALUSrc_D_X_r      <= flush ? 1'b0 : (stall_M_X ? ALUSrc_D_X_r      : (stall_D_from_F ? 1'b0  : ALUSrc_D_X_w)); //inserted here

            PC_D_X_r          <= flush ? 32'd0 : (stall_M_X ? PC_D_X_r         : (stall_D_from_F ? 32'd0 : PC_D_X_w)); //inserted here
            PC4_D_X_r         <= flush ? 32'd0 : (stall_M_X ? PC4_D_X_r        : (stall_D_from_F ? 32'd0 : PC4_D_X_w)); //inserted here

            reg1_r            <= flush ? 32'd0 : (stall_M_X ? reg1_r         : reg1_w); //inserted here
            reg2_r            <= flush ? 32'd0 : (stall_M_X ? reg2_r         : reg2_w); //inserted here
            imm_r             <= flush ? 32'd0 : (stall_M_X ? imm_r          : imm_w); //inserted here
            funct3_r          <= flush ? 3'd0  : (stall_M_X ? funct3_r       : funct3_w); //inserted here
            i_opsel_r         <= flush ? 3'd0  : (stall_M_X ? i_opsel_r      : i_opsel_w); //inserted here
            i_sub_r           <= flush ? 1'b0  : (stall_M_X ? i_sub_r        : i_sub_w); //inserted here
            i_unsigned_r      <= flush ? 1'b0  : (stall_M_X ? i_unsigned_r   : i_unsigned_w); //inserted here
            i_arith_r         <= flush ? 1'b0  : (stall_M_X ? i_arith_r      : i_arith_w); //inserted here

            rs1_raddr_D_X_r   <= flush ? 5'd0  : (stall_M_X ? rs1_raddr_D_X_r  : (stall_D_from_F ? 5'd0 : rs1_raddr_D_X_w)); //inserted here
            rs2_raddr_D_X_r   <= flush ? 5'd0  : (stall_M_X ? rs2_raddr_D_X_r  : (stall_D_from_F ? 5'd0 : rs2_raddr_D_X_w)); //inserted here
            rs1_rdata_D_X_r   <= flush ? 32'd0 : (stall_M_X ? rs1_rdata_D_X_r  : (stall_D_from_F ? 32'd0 : rs1_rdata_D_X_w)); //inserted here
            rs2_rdata_D_X_r   <= flush ? 32'd0 : (stall_M_X ? rs2_rdata_D_X_r  : (stall_D_from_F ? 32'd0 : rs2_rdata_D_X_w)); //inserted here

            inst_D_X_r        <= flush ? 32'd0 : (stall_M_X ? inst_D_X_r       : (stall_D_from_F ? 32'd0 : inst_D_X_w)); //inserted here
            valid_D_X_r       <= flush ? 1'b0  : (stall_M_X ? valid_D_X_r      : (stall_D_from_F ? 1'b0  : valid_D_X_w)); //inserted here

            // X/M : hold on stall_M_X
            target_addr_D_X_r <= stall_M_X ? target_addr_D_X_r : target_addr_D_X_w;
            isJALR_X_M_r      <= stall_M_X ? isJALR_X_M_r      : isJALR_X_M_w;
            Jump_X_M_r        <= stall_M_X ? Jump_X_M_r        : Jump_X_M_w;
            MemRead_X_M_r     <= stall_M_X ? MemRead_X_M_r     : MemRead_X_M_w;
            MemtoReg_X_M_r    <= stall_M_X ? MemtoReg_X_M_r    : MemtoReg_X_M_w;
            RegWrite_X_M_r    <= stall_M_X ? RegWrite_X_M_r    : RegWrite_X_M_w;
            IsUInstruct_X_M_r <= stall_M_X ? IsUInstruct_X_M_r : IsUInstruct_X_M_w;

            PC_X_M_r          <= stall_M_X ? PC_X_M_r          : PC_X_M_w;
            PC4_X_M_r         <= stall_M_X ? PC4_X_M_r         : PC4_X_M_w;

            rd_waddr_X_M_r    <= stall_M_X ? rd_waddr_X_M_r    : rd_waddr_X_M_w;
            ALU_X_M_r         <= stall_M_X ? ALU_X_M_r         : ALU_X_M_w;
            uimm_X_M_r        <= stall_M_X ? uimm_X_M_r        : uimm_X_M_w;

            eq_r              <= stall_M_X ? eq_r              : eq_w;
            slt_r             <= stall_M_X ? slt_r             : slt_w;
            mem_unsigned_r    <= stall_M_X ? mem_unsigned_r    : mem_unsigned_w;
            dmem_addr_r       <= stall_M_X ? dmem_addr_r       : dmem_addr_w;
            dmem_mask_r       <= stall_M_X ? dmem_mask_r       : dmem_mask_w;
            reg2_X_M_r        <= stall_M_X ? reg2_X_M_r        : reg2_X_M_w;

            trapD_X_M_r       <= stall_M_X ? trapD_X_M_r       : trapD_X_M_w;
            trapX_X_M_r       <= stall_M_X ? trapX_X_M_r       : trapX_X_M_w;

            rs1_raddr_X_M_r   <= stall_M_X ? rs1_raddr_X_M_r   : rs1_raddr_X_M_w;
            rs2_raddr_X_M_r   <= stall_M_X ? rs2_raddr_X_M_r   : rs2_raddr_X_M_w;
            rs1_rdata_X_M_r   <= stall_M_X ? rs1_rdata_X_M_r   : rs1_rdata_X_M_w;
            rs2_rdata_X_M_r   <= stall_M_X ? rs2_rdata_X_M_r   : rs2_rdata_X_M_w;

            dmem_addr_X_M_r   <= stall_M_X ? dmem_addr_X_M_r   : dmem_addr_X_M_w;
            dmem_rdata_X_M_r  <= stall_M_X ? dmem_rdata_X_M_r  : dmem_rdata_X_M_w;
            dmem_ren_X_M_r    <= stall_M_X ? dmem_ren_X_M_r    : dmem_ren_X_M_w;
            dmem_wen_X_M_r    <= stall_M_X ? dmem_wen_X_M_r    : dmem_wen_X_M_w;
            dmem_mask_X_M_r   <= stall_M_X ? dmem_mask_X_M_r   : dmem_mask_X_M_w;
            dmem_wdata_X_M_r  <= stall_M_X ? dmem_wdata_X_M_r  : dmem_wdata_X_M_w;
            dmem_wdata_ex_r   <= stall_M_X ? dmem_wdata_ex_r   : dmem_wdata_ex_w;
            dmem_wen_ex_r     <= stall_M_X ? dmem_wen_ex_r     : dmem_wen_ex_w;

            halt_X_M_r        <= stall_M_X ? halt_X_M_r        : halt_X_M_w;
            inst_X_M_r        <= stall_M_X ? inst_X_M_r        : inst_X_M_w;
            valid_X_M_r       <= stall_M_X ? valid_X_M_r       : valid_X_M_w;
            // M/W : hold on stall_M
            next_PC_M_W_r     <= stall_M ? next_PC_M_W_r : next_PC_M_W_w; //inserted here
            Jump_M_W_r        <= stall_M ? Jump_M_W_r : Jump_M_W_w; //inserted here
            MemtoReg_M_W_r    <= stall_M ? MemtoReg_M_W_r : MemtoReg_M_W_w; //inserted here
            RegWrite_M_W_r    <= stall_M ? RegWrite_M_W_r : RegWrite_M_W_w; //inserted here
            IsUInstruct_M_W_r <= stall_M ? IsUInstruct_M_W_r : IsUInstruct_M_W_w; //inserted here

            PC_M_W_r          <= stall_M ? PC_M_W_r : PC_M_W_w; //inserted here
            PC4_M_W_r         <= stall_M ? PC4_M_W_r : PC4_M_W_w; //inserted here

            rd_waddr_M_W_r    <= stall_M ? rd_waddr_M_W_r : rd_waddr_M_W_w; //inserted here
            ALU_M_W_r         <= stall_M ? ALU_M_W_r : ALU_M_W_w; //inserted here
            uimm_M_W_r        <= stall_M ? uimm_M_W_r : uimm_M_W_w; //inserted here
            mem_read_M_W_r    <= stall_M ? mem_read_M_W_r : mem_read_M_W_w; //inserted here

            trapD_M_W_r       <= stall_M ? trapD_M_W_r : trapD_M_W_w; //inserted here
            trapX_M_W_r       <= stall_M ? trapX_M_W_r : trapX_M_W_w; //inserted here

            rs1_raddr_M_W_r   <= stall_M ? rs1_raddr_M_W_r : rs1_raddr_M_W_w; //inserted here
            rs2_raddr_M_W_r   <= stall_M ? rs2_raddr_M_W_r : rs2_raddr_M_W_w; //inserted here
            rs1_rdata_M_W_r   <= stall_M ? rs1_rdata_M_W_r : rs1_rdata_M_W_w; //inserted here
            rs2_rdata_M_W_r   <= stall_M ? rs2_rdata_M_W_r : rs2_rdata_M_W_w; //inserted here

            dmem_addr_M_W_r   <= stall_M ? dmem_addr_M_W_r : dmem_addr_M_W_w; //inserted here
            dmem_rdata_M_W_r  <= stall_M ? dmem_rdata_M_W_r : dmem_rdata_M_W_w; //inserted here
            dmem_ren_M_W_r    <= stall_M ? dmem_ren_M_W_r : dmem_ren_M_W_w; //inserted here
            dmem_wen_M_W_r    <= stall_M ? dmem_wen_M_W_r : dmem_wen_M_W_w; //inserted here
            dmem_mask_M_W_r   <= stall_M ? dmem_mask_M_W_r : dmem_mask_M_W_w; //inserted here
            dmem_wdata_M_W_r  <= stall_M ? dmem_wdata_M_W_r : dmem_wdata_M_W_w; //inserted here

            halt_M_W_r        <= stall_M ? halt_M_W_r : halt_M_W_w; //inserted here
            inst_M_W_r        <= stall_M ? inst_M_W_r : inst_M_W_w; //inserted here
            // Advance M/W when not stalled, else hold
            valid_M_W_r       <= stall_M ? valid_M_W_r : valid_M_W_w; //inserted here
        end
    end

    rf #(1) reg_file (
        i_clk, i_rst,
        rs1_raddr_D_X_w, reg1_w,
        rs2_raddr_D_X_w, reg2_w,
        i_reg_write_en, i_reg_write_addr, i_reg_write_data
    );

    // cache stuff
    wire [31:0] imem_raddr;
    wire        imem_ren;
    wire        icache_busy;
    wire [31:0] icache_rdata;

    cache icache (
        .i_clk       (i_clk),
        .i_rst       (i_rst),
        // backing memory side
        .i_mem_ready (i_mem_ready),
        .i_mem_rdata (i_mem_rdata),
        .i_mem_valid (i_mem_valid),
        .o_mem_addr  (o_mem_addr),
        .o_mem_ren   (o_mem_ren),
        .o_mem_wen   (),
        .o_mem_wdata (),
        // fetch side
        .i_req_addr  (imem_raddr),
        .i_req_ren   (imem_ren),
        .i_req_wen   (1'b0),
        .i_req_mask  (4'b1111),
        .i_req_wdata (32'b0),
        .o_busy      (icache_busy),
        .o_res_rdata (icache_rdata)
    );

    fetch #(RESET_ADDR) fetch_inst (
        i_rst,
        i_clk,
        ~icache_busy,    // ready when cache not busy
        ~icache_busy,    // valid when cache not busy
        icache_rdata, 
        next_PC_to_fetch,
        stall_M_X_D,     //inserted here
        redirect_X,
        imem_raddr,      // these feed INTO the cache
        imem_ren, 
        fetch_wait,      //inserted here
        PC_F_D_w,
        instruction_w
    );

    decode d (
        i_clk,
        instruction_r,
        Jump_D_X_w, BranchEqual_D_X_w, BranchLT_D_X_w, Branch_D_X_w,
        MemRead_D_X_w, MemWrite_D_X_w, MemtoReg_D_X_w,
        ALUSrc_D_X_w,
        RegWrite_D_X_w, IsUInstruct_D_X_w, UpperType_D_X_w, isJALR_D_X_w,
        reg1_w, reg2_w, imm_w,
        funct3_w, i_opsel_w, i_sub_w, i_unsigned_w, i_arith_w,
        rs1_raddr_D_X_w, rs2_raddr_D_X_w, rd_waddr_D_X_w,
        halt_D_X_w, inst_D_X_w, trapD_D_X_w,
        rs1_rdata_D_X_w, rs2_rdata_D_X_w,
        PC_F_D_r, PC_D_X_w, PC4_D_X_w,
        RegWrite_D_X_r, MemRead_D_X_r, rd_waddr_D_X_r,
        RegWrite_X_M_r, rd_waddr_X_M_r,
        RegWrite_M_W_r, rd_waddr_M_W_r,
        stall_D,
        branch_taken
    );

    execute x (
        i_clk, stall_X, i_dmem_ready,
        reg1_r, reg2_r, imm_r, funct3_r, i_opsel_r, i_sub_r, i_unsigned_r, i_arith_r,
        PC_D_X_r, PC4_D_X_r, ALU_X_M_w, eq_w, slt_w, target_addr_D_X_w, PC_X_M_w, PC4_X_M_w,
        mem_unsigned_w, dmem_mask_w, dmem_addr_w, dmem_wdata_ex_w, reg2_X_M_w,
        ALUSrc_D_X_r, isJALR_D_X_r, Jump_D_X_r, BranchEqual_D_X_r, BranchLT_D_X_r, Branch_D_X_r,
        MemRead_D_X_r, MemtoReg_D_X_r, MemWrite_D_X_r, rd_waddr_D_X_r,
        RegWrite_D_X_r, UpperType_D_X_r, IsUInstruct_D_X_r,
        isJALR_X_M_w, Jump_X_M_w,
        MemRead_X_M_w, MemtoReg_X_M_w, dmem_wen_ex_w,
        rd_waddr_X_M_w, RegWrite_X_M_w, IsUInstruct_X_M_w,
        uimm_X_M_w,
        halt_D_X_r, inst_D_X_r, trapD_D_X_r,
        rs1_rdata_D_X_r, rs2_rdata_D_X_r,
        rs1_raddr_D_X_r, rs2_raddr_D_X_r,
        valid_D_X_r,
        halt_X_M_w, inst_X_M_w, trapD_X_M_w,
        rs1_rdata_X_M_w, rs2_rdata_X_M_w,
        rs1_raddr_X_M_w, rs2_raddr_X_M_w,
        trapX_X_M_w,
        dmem_wdata_X_M_w, dmem_wen_X_M_w,
        valid_X_M_w,
        branch_taken,
        exExForwardingResult, wb_data,
        reg1_ex_forward, reg2_ex_forward,
        reg1_mem_forward, reg2_mem_forward
    );

    memory_student m (
        i_clk, stall_M,
        i_dmem_ready, //inserted here
        i_dmem_valid, //inserted here
        dmem_mask_r, mem_unsigned_r, dmem_addr_r, reg2_X_M_r,
        ALU_X_M_r,
        target_addr_D_X_r, PC_X_M_r, PC4_X_M_r, PC_M_W_w,
        mem_read_M_W_w, ALU_M_W_w, uimm_M_W_w,
        isJALR_X_M_r, Jump_X_M_r,
        MemRead_X_M_r, MemtoReg_X_M_r,
        rd_waddr_X_M_r, RegWrite_X_M_r, IsUInstruct_X_M_r,
        uimm_X_M_r,
        Jump_M_W_w, MemtoReg_M_W_w, rd_waddr_M_W_w, RegWrite_M_W_w, IsUInstruct_M_W_w,
        i_dmem_rdata, o_dmem_ren,
        halt_X_M_r, inst_X_M_r, trapD_X_M_r,
        rs1_rdata_X_M_r, rs2_rdata_X_M_r,
        rs1_raddr_X_M_r, rs2_raddr_X_M_r,
        trapX_X_M_r,
        dmem_wdata_X_M_r, dmem_wen_X_M_r,
        valid_X_M_r,
        halt_M_W_w, inst_M_W_w, trapD_M_W_w,
        rs1_rdata_M_W_w, rs2_rdata_M_W_w,
        rs1_raddr_M_W_w, rs2_raddr_M_W_w,
        trapX_M_W_w,
        dmem_mask_M_W_w, dmem_addr_M_W_w, dmem_wdata_M_W_w,
        dmem_ren_M_W_w, dmem_wen_M_W_w, dmem_rdata_M_W_w,
        valid_M_W_w,
        exExForwardingResult
    );

    writeback w (
        i_clk,
        PC_M_W_r,
        next_PC_M_W_r,
        mem_read_M_W_r, ALU_M_W_r, uimm_M_W_r,
        wb_data,
        o_retire_pc,
        next_PC_W_F,
        Jump_M_W_r, MemtoReg_M_W_r, rd_waddr_M_W_r,
        RegWrite_M_W_r, IsUInstruct_M_W_r,
        wb_en,
        wb_addr,
        halt_M_W_r, inst_M_W_r, trapD_M_W_r,
        rs1_rdata_M_W_r, rs2_rdata_M_W_r,
        rs1_raddr_M_W_r, rs2_raddr_M_W_r,
        trapX_M_W_r,
        dmem_mask_M_W_r, dmem_addr_M_W_r, dmem_wdata_M_W_r,
        dmem_ren_M_W_r, dmem_wen_M_W_r, dmem_rdata_M_W_r,
        halt_W_F, inst_W_F, trapD_W_F,
        rs1_rdata_W_F, rs2_rdata_W_F,
        rs1_raddr_W_F, rs2_raddr_W_F,
        trapX_W_F,
        dmem_mask_W_F, dmem_addr_W_F, dmem_wdata_W_F,
        dmem_ren_W_F, dmem_wen_W_F, dmem_rdata_W_F,
        valid_M_W_r,
        valid_W_F
    );

    exExForwarding eef (
        rs1_raddr_D_X_r, rs2_raddr_D_X_r,
        rd_waddr_X_M_r,
        RegWrite_X_M_r,
        valid_X_M_r,
        MemRead_X_M_r,
        reg1_ex_forward,
        reg2_ex_forward
    );

    memExForwarding mef (
        rs1_raddr_D_X_r, rs2_raddr_D_X_r,
        rd_waddr_M_W_r,
        RegWrite_M_W_r,
        valid_M_W_r,
        reg1_mem_forward,
        reg2_mem_forward
    );

endmodule

`default_nettype wire
`default_nettype none

module cache (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // External memory interface. See hart interface for details. This
    // interface is nearly identical to the phase 5 memory interface, with the
    // exception that the byte mask (`o_mem_mask`) has been removed. This is
    // no longer needed as the cache will only access the memory at word
    // granularity, and implement masking internally.
    input  wire        i_mem_ready,
    output wire [31:0] o_mem_addr,
    output wire        o_mem_ren,
    output wire        o_mem_wen,
    output wire [31:0] o_mem_wdata,
    input  wire [31:0] i_mem_rdata,
    input  wire        i_mem_valid,
    // Interface to CPU hart. This is nearly identical to the phase 5 hart memory
    // interface, but includes a stall signal (`o_busy`), and the input/output
    // polarities are swapped for obvious reasons.
    //
    // The CPU should use this as a stall signal for both instruction fetch
    // (IF) and memory (MEM) stages, from the instruction or data cache
    // respectively. If a memory request is made (`i_req_ren` for instruction
    // cache, or either `i_req_ren` or `i_req_wen` for data cache), this
    // should be asserted *combinationally* if the request results in a cache
    // miss.
    //
    // In case of a cache miss, the CPU must stall the respective pipeline
    // stage and deassert ren/wen on subsequent cycles, until the cache
    // deasserts `o_busy` to indicate it has serviced the cache miss. However,
    // the CPU must keep the other request lines constant. For example, the
    // CPU should not change the request address while stalling.
    output wire        o_busy,
    // 32-bit read/write address to access from the cache. This should be
    // 32-bit aligned (i.e. the two LSBs should be zero). See `i_req_mask` for
    // how to perform half-word and byte accesses to unaligned addresses.
    input  wire [31:0] i_req_addr,
    // When asserted, the cache should perform a read at the aligned address
    // specified by `i_req_addr` and return the 32-bit word at that address,
    // either immediately (i.e. combinationally) on a cache hit, or
    // synchronously on a cache miss. It is illegal to assert this and
    // `i_dmem_wen` on the same cycle.
    input  wire        i_req_ren,
    // When asserted, the cache should perform a write at the aligned address
    // specified by `i_req_addr` with the 32-bit word provided in
    // `o_req_wdata` (specified by the mask). This is necessarily synchronous,
    // but may either happen on the next clock edge (on a cache hit) or after
    // multiple cycles of latency (cache miss). As the cache is write-through
    // and write-allocate, writes must be applied to both the cache and
    // underlying memory.
    // It is illegal to assert this and `i_dmem_ren` on the same cycle.
    input  wire        i_req_wen,
    // The memory interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    input  wire [ 3:0] i_req_mask,
    // The 32-bit word to write to memory, if the request is a write
    // (i_req_wen is asserted). Only the bytes corresponding to set bits in
    // the mask should be written into the cache (and to backing memory).
    input  wire [31:0] i_req_wdata,
    // THe 32-bit data word read from memory on a read request.
    output wire [31:0] o_res_rdata
);
    // These parameters are equivalent to those provided in the project
    // 6 specification. Feel free to use them, but hardcoding these numbers
    // rather than using the localparams is also permitted, as long as the
    // same values are used (and consistent with the project specification).
    //
    // 32 sets * 2 ways per set * 16 bytes per way = 1K cache
    localparam O = 4;            // 4 bit offset => 16 byte cache line
    localparam S = 5;            // 5 bit set index => 32 sets
    localparam DEPTH = 32;   // 32 sets
    localparam W = 2;            // 2 way set associative, NMRU
    localparam T = 32 - O - S;   // 23 bit tag
    localparam D = 16 / 4;   // 16 bytes per line / 4 bytes per word = 4 words per line

    // The following memory arrays model the cache structure. As this is
    // an internal implementation detail, you are *free* to modify these
    // arrays as you please.

    // Backing memory, modeled as two separate ways.
    reg [   31:0] datas0 [DEPTH - 1:0][D - 1:0];
    reg [   31:0] datas1 [DEPTH - 1:0][D - 1:0];
    reg [T - 1:0] tags0  [DEPTH - 1:0];
    reg [T - 1:0] tags1  [DEPTH - 1:0];
    reg [1:0] valid [DEPTH - 1:0];
    reg       lru   [DEPTH - 1:0];

    // Fill in your implementation here.

    // Break address up
    // split into parts
    wire [T-1:0] req_tag  = i_req_addr[31:9];  
    wire [S-1:0] req_set  = i_req_addr[8:4];        
    wire [1:0]   req_word = i_req_addr[3:2];  

    wire hit0 = valid[req_set][0] && (tags0[req_set] == req_tag);
    wire hit1 = valid[req_set][1] && (tags1[req_set] == req_tag);
    wire hit  = hit0 || hit1;
    
    // select corresponding hit data
    wire [31:0] hit_data = hit0 ? datas0[req_set][req_word]
                            : datas1[req_set][req_word];

    assign o_res_rdata = hit ? hit_data : (i_mem_valid && i_mem_ren) ? i_mem_rdata : 32'd0; // set result data
    //assign o_busy      = (i_req_ren || i_req_wen) && !hit; //if there is a r/w request and we didn't get a hit assert
    //assign o_mem_ren = i_req_ren && !hit;
    //assign o_mem_wen = i_req_wen && !hit;

    // break down the address into tag, set index, and block offset
    

    always @(posedge i_clk) begin
        if ((i_req_ren || i_req_wen) && (hit || i_mem_valid)) begin

            // update lru[set_index] --- 0 for way 0 is LRU, 1 for way 1 is LRU
            // upon a hit, the LRU should be updated to be the other way, as the hit way is now the MRU
            // upon a miss, the LRU should change value, since the LRU way will be replaced with the new block, making it the MRU, and the other way the LRU
            lru[req_set] <= hit0 ? 1'b1 : (hit1 ? 1'b0 : ~lru[req_set]);

            // update valid[set_index]
            assign valid[req_set][lru[req_set]] = 1'b1;

        end
   

    always @(posedge i_clk) begin
        if (i_req_wen && hit) begin
            if (hit0) begin
                if (i_req_mask[0]) datas0[req_set][req_word][7:0]   <= i_req_wdata[7:0];
                if (i_req_mask[1]) datas0[req_set][req_word][15:8]  <= i_req_wdata[15:8];
                if (i_req_mask[2]) datas0[req_set][req_word][23:16] <= i_req_wdata[23:16];
                if (i_req_mask[3]) datas0[req_set][req_word][31:24] <= i_req_wdata[31:24];
                //lru[req_set] <= 1'b1; // way 0 used so evict one next
            end 
            else begin // hit1
                if (i_req_mask[0]) datas1[req_set][req_word][7:0]   <= i_req_wdata[7:0];
                if (i_req_mask[1]) datas1[req_set][req_word][15:8]  <= i_req_wdata[15:8];
                if (i_req_mask[2]) datas1[req_set][req_word][23:16] <= i_req_wdata[23:16];
                if (i_req_mask[3]) datas1[req_set][req_word][31:24] <= i_req_wdata[31:24];
                //lru[req_set] <= 1'b0; // swap to evict 0 next
            end
        end
    end

    // WRITE-THROUGH, WRITE-ALLOCATE
    //get new data
    wire [31:0] mem_wdata;
    assign mem_wdata[7:0]   = i_req_mask[0] ? i_req_wdata[7:0]   : hit_data[7:0];
    assign mem_wdata[15:8]  = i_req_mask[1] ? i_req_wdata[15:8]  : hit_data[15:8];
    assign mem_wdata[23:16] = i_req_mask[2] ? i_req_wdata[23:16] : hit_data[23:16];
    assign mem_wdata[31:24] = i_req_mask[3] ? i_req_wdata[31:24] : hit_data[31:24];

    // wire [31:0] shift_data, mem_wdata;

    // wire bit0 = (i_req_mask == 4'b0001) || (i_req_mask == 4'b0011) || (i_req_mask == 4'b0101) || (i_req_mask == 4'b0111) || (i_req_mask == 4'b1001) || (i_req_mask == 4'b1011) || (i_req_mask == 4'b1101) || (i_req_mask == 4'b1111);
    // wire bit1 = (i_req_mask == 4'b0010) || (i_req_mask == 4'b0110) || (i_req_mask == 4'b1010) || (i_req_mask == 4'b1110);
    // wire bit2 = (i_req_mask == 4'b0100) || (i_req_mask == 4'b1100);

    // assign shift_data = (bit0) ? i_req_wdata :
    //                     (bit1) ? {i_req_wdata[23:0], 8'd0} :
    //                     (bit2) ? {i_req_wdata[15:0], 16'd0} :
    //                     {i_req_wdata[7:0], 24'd0}; // (i_req_mask == 4'b1000)

    // assign mem_wdata = shift_data;

    // miss handling
    reg req_sent_r = 1'b0; // Tracks whether a load request has been accepted.
    reg resp_seen_r = 1'b0; // Tracks whether the accepted load has completed.
    //reg [31:0] read_data_r = 32'd0;

    // mem read request:
    // send exactly once when memory is ready and a load is present,
    // then keep the stage stalled until the corresponding i_mem_valid arrives.
    wire load_active;
    assign load_active = (i_req_ren || i_req_wen) && !hit;
    assign o_mem_ren = i_req_ren & ~req_sent_r & i_mem_ready & !hit;
    assign o_mem_wen = i_req_wen & ~req_sent_r & i_mem_ready & !hit;
    assign o_busy = load_active & ~resp_seen_r;

    always @(posedge i_clk) begin
        if (!load_active) begin
            req_sent_r <= 1'b0;
            resp_seen_r <= 1'b0;
        end else begin
            if (o_mem_ren || o_mem_wen) begin
                req_sent_r <= 1'b1;
            end
            if (i_mem_valid && req_sent_r) begin
                if (i_req_ren) begin
                    read_data_r <= i_mem_rdata;
                end
                resp_seen_r <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire

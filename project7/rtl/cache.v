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
    localparam D = 4;   // 16 bytes per line / 4 bytes per word = 4 words per line

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

    // miss handling variables
    reg req_sent_r = 1'b0; // Tracks whether a load request has been accepted.
    reg resp_seen_r = 1'b0; // Tracks whether the accepted load has completed.
    reg busy = 1'b0; // Tracks whether we are currently in a load request (between accepting a request and seeing the response).
    reg ren = 1'b0;
    reg wen = 1'b0;
    reg [31:0] addr_r = 32'b0;
    reg [31:0] wdata_r = 32'b0;
    reg [S-1:0] miss_set_r = {S{1'b0}};
    reg [T-1:0] miss_tag_r = {T{1'b0}};
    reg         miss_way_r = 1'b0;
    reg [1:0]   miss_word_r = 2'b00;
    reg [2:0]   send_word_r = 3'd0;
    reg [2:0]   recv_word_r = 3'd0;

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

    assign o_res_rdata = hit ? hit_data : i_mem_rdata; // set result data
    //assign o_busy      = (i_req_ren || i_req_wen) && !hit; //if there is a r/w request and we didn't get a hit assert
    //assign o_mem_ren = i_req_ren && !hit;
    //assign o_mem_wen = i_req_wen && !hit;


    // VALID, LRU, and TAG updates
    wire new_lru = hit0 ? 1'b1 : (hit1 ? 1'b0 : ~lru[req_set]);
    // On a miss, replace the old LRU way and write req_tag into that way.
    wire [T-1:0] new_tag0 = (!hit && (lru[req_set] == 1'b0)) ? req_tag : tags0[req_set];
    wire [T-1:0] new_tag1 = (!hit && (lru[req_set] == 1'b1)) ? req_tag : tags1[req_set];
    
    always @(posedge i_clk) begin
        if ((i_req_ren || i_req_wen) && (hit)) begin

            // update lru[set_index] --- 0 for way 0 is LRU, 1 for way 1 is LRU
            // upon a hit, the LRU should be updated to be the other way, as the hit way is now the MRU
            // upon a miss, the LRU should change value, since the LRU way will be replaced with the new block, making it the MRU, and the other way the LRU
            lru[req_set] <= new_lru;

            // update valid[set_index]
            valid[req_set][~new_lru] <= 1'b1;

            // update tag[set_index][way]
            tags0[req_set] <= new_tag0;
            tags1[req_set] <= new_tag1;

        end
        if (i_rst) begin
            valid[0][0] <= 1'b0;
            valid[0][1] <= 1'b0;
            valid[1][0] <= 1'b0;
            valid[1][1] <= 1'b0;
            valid[2][0] <= 1'b0;
            valid[2][1] <= 1'b0;
            valid[3][0] <= 1'b0;
            valid[3][1] <= 1'b0;
            valid[4][0] <= 1'b0;
            valid[4][1] <= 1'b0;
            valid[5][0] <= 1'b0;
            valid[5][1] <= 1'b0;
            valid[6][0] <= 1'b0;
            valid[6][1] <= 1'b0;
            valid[7][0] <= 1'b0;
            valid[7][1] <= 1'b0;
            valid[8][0] <= 1'b0;
            valid[8][1] <= 1'b0;
            valid[9][0] <= 1'b0;
            valid[9][1] <= 1'b0;
            valid[10][0] <= 1'b0;
            valid[10][1] <= 1'b0;
            valid[11][0] <= 1'b0;
            valid[11][1] <= 1'b0;
            valid[12][0] <= 1'b0;
            valid[12][1] <= 1'b0;
            valid[13][0] <= 1'b0;
            valid[13][1] <= 1'b0;
            valid[14][0] <= 1'b0;
            valid[14][1] <= 1'b0;
            valid[15][0] <= 1'b0;
            valid[15][1] <= 1'b0;
            valid[16][0] <= 1'b0;
            valid[16][1] <= 1'b0;
            valid[17][0] <= 1'b0;
            valid[17][1] <= 1'b0;
            valid[18][0] <= 1'b0;
            valid[18][1] <= 1'b0;
            valid[19][0] <= 1'b0;
            valid[19][1] <= 1'b0;
            valid[20][0] <= 1'b0;
            valid[20][1] <= 1'b0;
            valid[21][0] <= 1'b0;
            valid[21][1] <= 1'b0;
            valid[22][0] <= 1'b0;
            valid[22][1] <= 1'b0;
            valid[23][0] <= 1'b0;
            valid[23][1] <= 1'b0;
            valid[24][0] <= 1'b0;
            valid[24][1] <= 1'b0;
            valid[25][0] <= 1'b0;
            valid[25][1] <= 1'b0;
            valid[26][0] <= 1'b0;
            valid[26][1] <= 1'b0;
            valid[27][0] <= 1'b0;
            valid[27][1] <= 1'b0;
            valid[28][0] <= 1'b0;
            valid[28][1] <= 1'b0;
            valid[29][0] <= 1'b0;
            valid[29][1] <= 1'b0;
            valid[30][0] <= 1'b0;
            valid[30][1] <= 1'b0;
            valid[31][0] <= 1'b0;
            valid[31][1] <= 1'b0;
            lru[0] <= 1'b0;
            lru[1] <= 1'b0;
            lru[2] <= 1'b0;
            lru[3] <= 1'b0;
            lru[4] <= 1'b0;
            lru[5] <= 1'b0;
            lru[6] <= 1'b0;
            lru[7] <= 1'b0;
            lru[8] <= 1'b0;
            lru[9] <= 1'b0;
            lru[10] <= 1'b0;
            lru[11] <= 1'b0;
            lru[12] <= 1'b0;
            lru[13] <= 1'b0;
            lru[14] <= 1'b0;
            lru[15] <= 1'b0;
            lru[16] <= 1'b0;
            lru[17] <= 1'b0;
            lru[18] <= 1'b0;
            lru[19] <= 1'b0;
            lru[20] <= 1'b0;
            lru[21] <= 1'b0;
            lru[22] <= 1'b0;
            lru[23] <= 1'b0;
            lru[24] <= 1'b0;
            lru[25] <= 1'b0;
            lru[26] <= 1'b0;
            lru[27] <= 1'b0;
            lru[28] <= 1'b0;
            lru[29] <= 1'b0;
            lru[30] <= 1'b0;
            lru[31] <= 1'b0;
            busy <= 1'b0;
            req_sent_r <= 1'b0;
            resp_seen_r <= 1'b0;
            ren <= 1'b0;
            wen <= 1'b0;
            send_word_r <= 3'd0;
            recv_word_r <= 3'd0;
        end
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

    // mem read request:
    // send exactly once when memory is ready and a load is present,
    // then keep the stage stalled until the corresponding i_mem_valid arrives.
    wire load_active;
    assign load_active = (i_req_ren && !hit) || (i_req_wen);
    assign o_mem_addr = ren ? {addr_r[31:4], send_word_r[1:0], 2'b00} : addr_r; // line fill on read miss
    assign o_mem_wdata = wdata_r;
    assign o_mem_ren = ren & (send_word_r < 3'd4) & i_mem_ready;
    assign o_mem_wen = wen & ~req_sent_r & i_mem_ready;
    // Assert busy immediately on a new miss and hold it while servicing.
    assign o_busy = busy | ((i_req_ren || i_req_wen) && !hit);
    wire [31:0] miss_data_0 = (new_lru == 1'b1) ? (ren ? i_mem_rdata : wdata_r) : datas0[req_set][req_word]; // data to update cache with on a miss, either from memory for a load, or the new data for a store
    wire [31:0] miss_data_1 = (new_lru == 1'b0) ? (ren ? i_mem_rdata : wdata_r) : datas1[req_set][req_word];

    always @(posedge i_clk) begin
        if (!load_active && !busy) begin
            req_sent_r <= 1'b0;
            resp_seen_r <= 1'b0;
            ren <= 1'b0;
            wen <= 1'b0;
        end else begin
            if (load_active) begin
                if (!busy) begin
                    // Start of a new request: clear previous transaction markers.
                    req_sent_r <= 1'b0;
                    resp_seen_r <= 1'b0;
                    send_word_r <= 3'd0;
                    recv_word_r <= 3'd0;
                end
                if (i_req_ren && !hit && !busy) begin
                    miss_set_r <= req_set;
                    miss_tag_r <= req_tag;
                    miss_way_r <= lru[req_set];
                    miss_word_r <= req_word;
                    addr_r <= {i_req_addr[31:4], 4'b0000};
                end else if (i_req_wen && !hit && !busy) begin
                    miss_set_r <= req_set;
                    miss_tag_r <= req_tag;
                    miss_way_r <= lru[req_set];
                    miss_word_r <= req_word;
                    addr_r <= {i_req_addr[31:2], 2'b00};
                end else begin
                    addr_r <= {i_req_addr[31:2], 2'b00};
                end
                wdata_r <= mem_wdata;

                if (!hit) begin
                    busy <= 1'b1;
                end
            end
            if (o_mem_ren || o_mem_wen) begin
                req_sent_r <= 1'b1;
                if (o_mem_ren) begin
                    send_word_r <= send_word_r + 3'd1;
                end
            end
            if (ren && i_mem_valid) begin
                if (miss_way_r == 1'b0) begin
                    datas0[miss_set_r][recv_word_r[1:0]] <= i_mem_rdata;
                end else begin
                    datas1[miss_set_r][recv_word_r[1:0]] <= i_mem_rdata;
                end

                if (recv_word_r == 3'd3) begin
                    resp_seen_r <= 1'b1;
                    busy <= 1'b0;
                    ren <= 1'b0;
                    req_sent_r <= 1'b0;

                    // LRU, VALID, TAG
                    lru[miss_set_r] <= ~miss_way_r;
                    valid[miss_set_r][miss_way_r] <= 1'b1;
                    if (miss_way_r == 1'b0) begin
                        tags0[miss_set_r] <= miss_tag_r;
                    end else begin
                        tags1[miss_set_r] <= miss_tag_r;
                    end
                end else begin
                    recv_word_r <= recv_word_r + 3'd1;
                end
            end
            if (busy && req_sent_r && wen && !ren) begin
                resp_seen_r <= 1'b1;
                busy <= 1'b0;
                wen <= 1'b0;
                req_sent_r <= 1'b0;

                // LRU, VALID, TAG
                lru[miss_set_r] <= ~miss_way_r;
                valid[miss_set_r][miss_way_r] <= 1'b1;
                if (miss_way_r == 1'b0) begin
                    tags0[miss_set_r] <= miss_tag_r;
                    datas0[miss_set_r][miss_word_r] <= wdata_r;
                end else begin
                    tags1[miss_set_r] <= miss_tag_r;
                    datas1[miss_set_r][miss_word_r] <= wdata_r;
                end
            end
        end
        if (i_req_ren && !hit) begin
            ren <= 1'b1;
        end
        if (i_req_wen) begin
            wen <= 1'b1;
        end
    end

endmodule

`default_nettype wire

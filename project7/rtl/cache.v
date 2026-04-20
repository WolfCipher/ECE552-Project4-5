`default_nettype none

module cache (
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire        i_mem_ready,
    output wire [31:0] o_mem_addr,
    output wire        o_mem_ren,
    output wire        o_mem_wen,
    output wire [31:0] o_mem_wdata,
    input  wire [31:0] i_mem_rdata,
    input  wire        i_mem_valid,
    output wire        o_busy,
    input  wire [31:0] i_req_addr,
    input  wire        i_req_ren,
    input  wire        i_req_wen,
    input  wire [ 3:0] i_req_mask,
    input  wire [31:0] i_req_wdata,
    output wire [31:0] o_res_rdata
);

    localparam O     = 4;
    localparam S     = 5;
    localparam DEPTH = 32;
    localparam T     = 32 - O - S;  // 23
    localparam D     = 4;

    // State encoding
    localparam IDLE        = 2'd0;
    localparam FILL        = 2'd1;
    localparam WRITE_CACHE = 2'd2;
    localparam WRITE_MEM   = 2'd3;

    reg [1:0] state;

    // Cache storage
    reg [31:0]   datas0 [DEPTH-1:0][D-1:0];
    reg [31:0]   datas1 [DEPTH-1:0][D-1:0];
    reg [T-1:0]  tags0  [DEPTH-1:0];
    reg [T-1:0]  tags1  [DEPTH-1:0];
    reg [1:0]    valid  [DEPTH-1:0];
    reg          lru    [DEPTH-1:0];

    // Miss-handling registers
    reg [S-1:0]  miss_set_r;
    reg [T-1:0]  miss_tag_r;
    reg          miss_way_r;
    reg [1:0]    miss_word_r;
    reg [1:0]    fill_word_r;
    reg [31:0]   miss_addr_r;   // cache-line-aligned address for fill
    reg [31:0]   wdata_r;       // masked write data
    reg [3:0]    mask_r;
    reg          req_sent_r;
    reg          was_write_r;   // was this miss caused by a write?
    reg write_hit_pending_r;


    // Address decomposition (from live request)
    wire [T-1:0] req_tag  = i_req_addr[31:9];
    wire [S-1:0] req_set  = i_req_addr[8:4];
    wire [1:0]   req_word = i_req_addr[3:2];

    // Hit detection (always on live request address)
    wire hit0 = valid[req_set][0] && (tags0[req_set] == req_tag);
    wire hit1 = valid[req_set][1] && (tags1[req_set] == req_tag);
    wire hit  = hit0 || hit1;

    wire [31:0] hit_data = hit0 ? datas0[req_set][req_word]
                                : datas1[req_set][req_word];

    // Busy: assert combinationally on first cycle of a miss
    assign o_busy = (state == FILL) ||
                    (state == WRITE_MEM) ||
                    (i_req_ren && !hit) ||
                    (i_req_wen && !hit);

    // Read data: on a hit return cache data, during fill return nothing useful
    // (CPU is stalled on miss anyway)
    assign o_res_rdata = hit ? hit_data : 32'd0;

    // Memory interface
    // During FILL: read successive words of the cache line
    // During WRITE_MEM: write the masked word through to memory
    assign o_mem_addr  = (state == FILL)      ? {miss_addr_r[31:4], fill_word_r, 2'b00}
                       : (state == WRITE_MEM)  ? {miss_addr_r[31:2], 2'b00}
                       : 32'd0;

    assign o_mem_wdata = wdata_r;

    assign o_mem_ren = (state == FILL)      && !req_sent_r && i_mem_ready;
    assign o_mem_wen = (state == WRITE_MEM) && !req_sent_r && i_mem_ready;

    // Masked write data for a write request
    wire [31:0] masked_wdata;
    assign masked_wdata[7:0]   = i_req_mask[0] ? i_req_wdata[7:0]   : hit_data[7:0];
    assign masked_wdata[15:8]  = i_req_mask[1] ? i_req_wdata[15:8]  : hit_data[15:8];
    assign masked_wdata[23:16] = i_req_mask[2] ? i_req_wdata[23:16] : hit_data[23:16];
    assign masked_wdata[31:24] = i_req_mask[3] ? i_req_wdata[31:24] : hit_data[31:24];

    integer i;
    always @(posedge i_clk) begin
        if (i_rst) begin
            state      <= IDLE;
            req_sent_r <= 1'b0;
            fill_word_r <= 2'b00;
            was_write_r <= 1'b0;
            miss_set_r  <= {S{1'b0}};
            miss_tag_r  <= {T{1'b0}};
            miss_way_r  <= 1'b0;
            miss_word_r <= 2'b00;
            miss_addr_r <= 32'd0;
            wdata_r     <= 32'd0;
            mask_r      <= 4'd0;
            write_hit_pending_r <= 1'b0;
            for (i = 0; i < DEPTH; i = i + 1) begin
                valid[i]  <= 2'b00;
                lru[i]    <= 1'b0;
                tags0[i]  <= {T{1'b0}};
                tags1[i]  <= {T{1'b0}};
            end
        end else begin
            case (state)

                // ----------------------------------------------------------------
                // IDLE: check for hits and misses
                // ----------------------------------------------------------------
                IDLE: begin
                    if (i_req_ren || i_req_wen) begin
                        if (hit) begin
                            // ----- HIT -----
                            // Update LRU
                            lru[req_set] <= hit0 ? 1'b1 : 1'b0;

                            if (i_req_wen) begin
                                // Write hit: update cache
                                if (hit0) begin
                                    if (i_req_mask[0]) datas0[req_set][req_word][7:0]   <= i_req_wdata[7:0];
                                    if (i_req_mask[1]) datas0[req_set][req_word][15:8]  <= i_req_wdata[15:8];
                                    if (i_req_mask[2]) datas0[req_set][req_word][23:16] <= i_req_wdata[23:16];
                                    if (i_req_mask[3]) datas0[req_set][req_word][31:24] <= i_req_wdata[31:24];
                                end else begin
                                    if (i_req_mask[0]) datas1[req_set][req_word][7:0]   <= i_req_wdata[7:0];
                                    if (i_req_mask[1]) datas1[req_set][req_word][15:8]  <= i_req_wdata[15:8];
                                    if (i_req_mask[2]) datas1[req_set][req_word][23:16] <= i_req_wdata[23:16];
                                    if (i_req_mask[3]) datas1[req_set][req_word][31:24] <= i_req_wdata[31:24];
                                end
                                // Write-through: go write to memory
                                // Capture write data (already masked above, but
                                // we need full word for memory)
                                wdata_r             <= masked_wdata;
                                mask_r              <= i_req_mask;
                                miss_addr_r         <= {i_req_addr[31:2], 2'b00};
                                req_sent_r          <= 1'b0;
                                write_hit_pending_r <= 1'b1;
                                state               <= WRITE_MEM;
                            end
                            // read hit: nothing else to do, stays IDLE

                        end else begin
                            // ----- MISS -----
                            // Latch miss info
                            miss_set_r  <= req_set;
                            miss_tag_r  <= req_tag;
                            miss_way_r  <= lru[req_set];
                            miss_word_r <= req_word;
                            miss_addr_r <= {i_req_addr[31:4], 4'b0000};
                            fill_word_r <= 2'b00;
                            req_sent_r  <= 1'b0;
                            was_write_r <= i_req_wen;
                            // Capture write data using current hit_data for
                            // masked merge (hit_data is 0 on miss but we
                            // re-apply mask after fill)
                            wdata_r     <= i_req_wdata;
                            mask_r      <= i_req_mask;
                            state       <= FILL;
                        end
                    end
                end

                // ----------------------------------------------------------------
                // FILL: load the full cache line from memory word by word
                // ----------------------------------------------------------------
                FILL: begin
                    // Fire a read request when memory is ready and we haven't
                    // sent this word's request yet
                    if (!req_sent_r && i_mem_ready) begin
                        req_sent_r <= 1'b1;
                    end

                    if (req_sent_r && i_mem_valid) begin
                        // Store the returned word into the correct way
                        if (miss_way_r == 1'b0)
                            datas0[miss_set_r][fill_word_r] <= i_mem_rdata;
                        else
                            datas1[miss_set_r][fill_word_r] <= i_mem_rdata;

                        req_sent_r <= 1'b0;

                        if (fill_word_r == 2'b11) begin
                            // Line fill complete: update tag/valid/lru
                            if (miss_way_r == 1'b0)
                                tags0[miss_set_r] <= miss_tag_r;
                            else
                                tags1[miss_set_r] <= miss_tag_r;
                            valid[miss_set_r][miss_way_r] <= 1'b1;
                            lru[miss_set_r]               <= ~miss_way_r;

                            if (was_write_r) begin
                                // Apply masked write to the newly filled line
                                if (miss_way_r == 1'b0) begin
                                    if (mask_r[0]) datas0[miss_set_r][miss_word_r][7:0]   <= wdata_r[7:0];
                                    if (mask_r[1]) datas0[miss_set_r][miss_word_r][15:8]  <= wdata_r[15:8];
                                    if (mask_r[2]) datas0[miss_set_r][miss_word_r][23:16] <= wdata_r[23:16];
                                    if (mask_r[3]) datas0[miss_set_r][miss_word_r][31:24] <= wdata_r[31:24];
                                end else begin
                                    if (mask_r[0]) datas1[miss_set_r][miss_word_r][7:0]   <= wdata_r[7:0];
                                    if (mask_r[1]) datas1[miss_set_r][miss_word_r][15:8]  <= wdata_r[15:8];
                                    if (mask_r[2]) datas1[miss_set_r][miss_word_r][23:16] <= wdata_r[23:16];
                                    if (mask_r[3]) datas1[miss_set_r][miss_word_r][31:24] <= wdata_r[31:24];
                                end
                                // Now write through to memory
                                miss_addr_r <= {miss_addr_r[31:4], miss_word_r, 2'b00};
                                req_sent_r  <= 1'b0;
                                write_hit_pending_r <= 1'b0;
                                state       <= WRITE_MEM;
                            end else begin
                                // Read miss done
                                state <= IDLE;
                            end
                        end else begin
                            fill_word_r <= fill_word_r + 2'b01;
                        end
                    end
                end

                // ----------------------------------------------------------------
                // WRITE_CACHE: (unused now, merged into FILL completion)
                // ----------------------------------------------------------------
                WRITE_CACHE: begin
                    state <= IDLE;
                end

                // ----------------------------------------------------------------
                // WRITE_MEM: write one word through to backing memory
                // ----------------------------------------------------------------
                WRITE_MEM: begin
                    if (!req_sent_r && i_mem_ready) begin
                        req_sent_r <= 1'b1;
                    end
                    if (req_sent_r && i_mem_valid) begin
                        req_sent_r <= 1'b0;
                        write_hit_pending_r <= 1'b0;
                        state      <= IDLE;
                    end
                end

            endcase
        end
    end

endmodule

`default_nettype wire
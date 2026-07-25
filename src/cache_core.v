module cache_core #(
    parameter ADDR_WIDTH = 32, 
    parameter LINE_BYTES = 16, 
    parameter NUM_SETS = 64, 
    parameter NUM_WAYS = 2  
) (
    // cpu / cache line req
    input wire clk, rst, req_valid, req_write, 
    input wire [ADDR_WIDTH - 1 : 0] req_addr, 
    input wire [LINE_BYTES * 8 - 1 : 0] req_wline, 
    input wire [LINE_BYTES - 1 : 0] req_wmask,

    // Lower mem line resp
    input wire [LINE_BYTES * 8 - 1 : 0] mem_rline, 
    input wire mem_ready, 

    // cpu / cache line resp
    output reg [LINE_BYTES * 8 - 1 : 0] resp_rline, 
    output reg resp_ready, 
    output wire hit, 

    // lower mem line req
    output reg mem_req_valid, mem_req_write,
    output reg [ADDR_WIDTH - 1 : 0] mem_req_addr, 
    output reg [LINE_BYTES * 8 - 1 : 0] mem_wline
); 

    localparam OFFSET_BITS = $clog2(LINE_BYTES); 
    localparam INDEX_BITS = $clog2(NUM_SETS);
    localparam TAG_BITS = ADDR_WIDTH - OFFSET_BITS - INDEX_BITS; 
    localparam LINE_BITS = LINE_BYTES * 8;

    localparam IDLE       = 3'd0;
    localparam LOOKUP     = 3'd1;
    localparam WRITEBACK  = 3'd2;
    localparam REFILL_REQ = 3'd3;
    localparam RESPOND    = 3'd4;

    wire [OFFSET_BITS - 1 : 0] req_offset;
    wire [INDEX_BITS - 1 : 0] req_index; 
    wire [TAG_BITS - 1 : 0] req_tag;

    assign req_offset = req_addr[OFFSET_BITS - 1: 0];
    assign req_index = req_addr[OFFSET_BITS + INDEX_BITS - 1 : OFFSET_BITS];
    assign req_tag = req_addr[ADDR_WIDTH - 1 : OFFSET_BITS + INDEX_BITS];

    reg [LINE_BITS - 1 : 0] data_array [0 : NUM_WAYS - 1][0 : NUM_SETS - 1];
    reg [TAG_BITS - 1 : 0] tag_array [0 : NUM_WAYS - 1][0 : NUM_SETS - 1];
    reg valid_array [0 : NUM_WAYS - 1][0 : NUM_SETS - 1];
    reg dirty_array [0 : NUM_WAYS - 1][0 : NUM_SETS -1];
    reg lru_array[0 : NUM_SETS - 1]; 

    wire hit_way0 = valid_array[0][req_index] && (tag_array[0][req_index] == req_tag); 
    wire hit_way1 = valid_array[1][req_index] && (tag_array[1][req_index] == req_tag);

    wire is_hit = hit_way0 || hit_way1;
    (* ram_style = "block" *) reg [LINE_BITS-1:0] data_way0 [0:NUM_SETS-1];
    (* ram_style = "block" *) reg [LINE_BITS-1:0] data_way1 [0:NUM_SETS-1];

    reg [LINE_BITS-1:0] way0_line_q;
    reg [LINE_BITS-1:0] way1_line_q;
    // wire [LINE_BITS - 1 : 0] hit_line = hit_way0 ? data_array[0][req_index] : 
    //                                     hit_way1 ? data_array[1][req_index] : 
    //                                                 {LINE_BITS{1'b0}}; 
    wire victim_way = !valid_array[0][req_index] ? 1'b0 : 
                      !valid_array[1][req_index] ? 1'b1 : 
                                lru_array[req_index];
    wire victim_valid = valid_array[victim_way][req_index];
    wire vicitm_dirty = dirty_array[victim_way][req_index];
    wire [TAG_BITS - 1 : 0] victim_tag = tag_array[victim_way][req_index];
    wire [LINE_BITS - 1 : 0] victim_line = data_array[victim_way][req_index];

    reg [2:0] state, next_state;
    reg saved_write, saved_victim_way, saved_victim_dirty;
    reg [ADDR_WIDTH - 1 : 0] saved_addr;
    reg [INDEX_BITS - 1 : 0] saved_index;
    reg [TAG_BITS - 1 : 0] saved_tag, saved_victim_tag;
    reg [LINE_BITS - 1 : 0] saved_wline, saved_victim_line, saved_resp_line;
    reg [LINE_BYTES - 1 : 0] saved_wmask; 

    wire saved_hit_way0 = valid_array[0][saved_index] &&
                        (tag_array[0][saved_index] == saved_tag);

    wire saved_hit_way1 = valid_array[1][saved_index] &&
                        (tag_array[1][saved_index] == saved_tag);

    wire saved_is_hit = saved_hit_way0 || saved_hit_way1;

    wire [LINE_BITS-1:0] saved_hit_line =
        saved_hit_way0 ? way0_line_q :
        saved_hit_way1 ? way1_line_q :
                        {LINE_BITS{1'b0}};

    wire saved_victim_way =
        !valid_array[0][saved_index] ? 1'b0 :
        !valid_array[1][saved_index] ? 1'b1 :
                                    lru_array[saved_index];

    wire saved_victim_valid = valid_array[saved_victim_way][saved_index];
    wire saved_victim_dirty = dirty_array[saved_victim_way][saved_index];
    wire [TAG_BITS-1:0] saved_victim_tag_live =
        tag_array[saved_victim_way][saved_index];

    wire [LINE_BITS-1:0] saved_victim_line_live =
        saved_victim_way ? way1_line_q : way0_line_q;

    integer i;

    function [LINE_BITS - 1 : 0] merge_line; 
        input [LINE_BITS - 1 : 0] old_line;
        input [LINE_BITS - 1 : 0] new_line; 
        input [LINE_BYTES - 1 : 0] byte_mask; 
        integer j; 
        begin
            merge_line = old_line; 
            for (j = 0; j < LINE_BYTES; j = j + 1) begin
                if (byte_mask[j]) begin
                    merge_line[j*8+:8] = new_line[j*8+:8];
                end 
            end 
        end 
    endfunction

    wire [LINE_BITS - 1 : 0] refill_merged_line = saved_write ? merge_line(mem_rline, saved_wline, saved_wmask) : mem_rline;
    assign hit = (state == IDLE) && req_valid && is_hit;

    always @(*) begin
        next_state = state;
        resp_ready = 1'b0; 
        mem_req_valid = 1'b0;
        mem_req_write = 1'b0;
        mem_req_addr = {ADDR_WIDTH{1'b0}};
        mem_wline = {LINE_BITS{1'b0}};
        resp_rline = {LINE_BITS{1'b0}};
        case(state) 
            // IDLE: 
            //     begin 
            //         if (req_valid) begin
            //             if (is_hit) begin
            //                 resp_ready = 1'b1;
            //                 resp_rline = req_write ? merge_line(hit_line, req_wline, req_wmask) : hit_line; 
            //             end else begin
            //                 if (victim_valid && vicitm_dirty) begin
            //                     next_state = WRITEBACK; 
            //                 end else begin
            //                     next_state = REFILL_REQ; 
            //                 end 
            //             end 
            //         end
            //     end 
            IDLE: 
                begin
                    if (req_valid) begin
                        next_state = LOOKUP;
                    end
                end
            LOOKUP: 
                begin
                    if (saved_is_hit) begin
                        resp_ready = 1'b1;
                        resp_rline = saved_write ?
                            merge_line(saved_hit_line, saved_wline, saved_wmask) :
                            saved_hit_line;

                        next_state = IDLE;
                    end else begin
                        if (saved_victim_valid && saved_victim_dirty) begin
                            next_state = WRITEBACK;
                        end else begin
                            next_state = REFILL_REQ;
                        end
                    end
                end
            WRITEBACK:
                begin
                    mem_req_valid = 1'b1; 
                    mem_req_write = 1'b1;
                    mem_req_addr = {saved_victim_tag, saved_index, {OFFSET_BITS{1'b0}}};
                    mem_wline = saved_victim_line;
                    if (mem_ready) begin
                        next_state = REFILL_REQ; 
                    end 
                end 
            REFILL_REQ: 
                begin
                    mem_req_valid = 1'b1; 
                    mem_req_write = 1'b0;
                    mem_req_addr = {saved_tag, saved_index, {OFFSET_BITS{1'b0}}};
                    // mem_req_addr = {saved_tag, saved_index, {4{1'b0}}};
                    if (mem_ready) begin
                        next_state = RESPOND;
                    end 
                end 
            RESPOND: 
                begin
                    resp_ready = 1'b1;
                    resp_rline = saved_resp_line;
                    next_state = IDLE;
                end 
            default: next_state = IDLE;
        endcase
    end 

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            saved_write <= 1'b0; 
            saved_addr <= {ADDR_WIDTH{1'b0}}; 
            saved_index <= {INDEX_BITS{1'b0}};
            saved_tag <= {TAG_BITS{1'b0}}; 
            saved_wline <= {LINE_BITS{1'b0}}; 
            saved_wmask <= {LINE_BYTES{1'b0}}; 
            saved_victim_way <= 1'b0;
            saved_victim_dirty <= 1'b0; 
            saved_victim_tag <= {TAG_BITS{1'b0}}; 
            saved_victim_line <= {LINE_BITS{1'b0}}; 
            saved_resp_line <= {LINE_BITS{1'b0}}; 

            for (i = 0; i < NUM_SETS; i = i + 1) begin
                valid_array[0][i] <= 1'b0;
                valid_array[1][i] <= 1'b0;
                dirty_array[0][i] <= 1'b0;
                dirty_array[1][i] <= 1'b0;
                lru_array[i] <= 1'b0;
            end 
        end else begin
            state <= next_state; 
            case (state) 
                // IDLE: 
                //     begin
                //         if (req_valid) begin
                //             if (is_hit) begin
                //                 lru_array[req_index] <= hit_way0 ? 1'b1 : 1'b0;
                //                 if (req_write) begin
                //                    if (hit_way0) begin
                //                         data_array[0][req_index] <= merge_line(data_array[0][req_index], req_wline, req_wmask);
                //                         dirty_array[0][req_index] <= 1'b1;
                //                     end else begin
                //                         data_array[1][req_index] <= merge_line(data_array[1][req_index], req_wline, req_wmask);
                //                         dirty_array[1][req_index] <= 1'b1;
                //                     end 
                //                 end 
                //             end else begin
                //                 saved_write <= req_write; 
                //                 saved_addr <= req_addr; 
                //                 saved_index <= req_index; 
                //                 saved_tag <= req_tag; 
                //                 saved_wline <= req_wline; 
                //                 saved_wmask <= req_wmask; 
                //                 saved_victim_way <= victim_way; 
                //                 saved_victim_dirty <= vicitm_dirty; 
                //                 saved_victim_tag <= victim_tag; 
                //                 saved_victim_line <= victim_line; 
                //             end 
                //         end 
                //     end
                IDLE: 
                    begin
                        if (req_valid) begin
                            saved_write <= req_write;
                            saved_addr  <= req_addr;
                            saved_index <= req_index;
                            saved_tag   <= req_tag;
                            saved_wline <= req_wline;
                            saved_wmask <= req_wmask;

                            // synchronous BRAM reads
                            way0_line_q <= data_way0[req_index];
                            way1_line_q <= data_way1[req_index];
                        end
                    end
                LOOKUP: 
                    begin
                        if (saved_is_hit) begin
                            lru_array[saved_index] <= saved_hit_way0 ? 1'b1 : 1'b0;

                            if (saved_write) begin
                                if (saved_hit_way0) begin
                                    data_way0[saved_index] <= merge_line(way0_line_q, saved_wline, saved_wmask);
                                    dirty_array[0][saved_index] <= 1'b1;
                                end else begin
                                    data_way1[saved_index] <= merge_line(way1_line_q, saved_wline, saved_wmask);
                                    dirty_array[1][saved_index] <= 1'b1;
                                end
                            end
                        end else begin
                            saved_victim_way   <= saved_victim_way;
                            saved_victim_dirty <= saved_victim_dirty;
                            saved_victim_tag   <= saved_victim_tag_live;
                            saved_victim_line  <= saved_victim_line_live;
                        end
                    end
                REFILL_REQ: 
                    begin
                        if (mem_ready) begin
                            // data_array[saved_victim_way][saved_index] <= refill_merged_line; 
                            if (saved_victim_way == 1'b0) begin
                                data_way0[saved_index] <= refill_merged_line;
                            end else begin
                                data_way1[saved_index] <= refill_merged_line;
                            end
                            tag_array[saved_victim_way][saved_index] <= saved_tag; 
                            valid_array[saved_victim_way][saved_index] <= 1'b1;
                            dirty_array[saved_victim_way][saved_index] <= saved_write; 
                            saved_resp_line <= refill_merged_line; 
                            lru_array[saved_index] <= (saved_victim_way == 1'b0) ? 1'b1 : 1'b0;
                        end 
                    end 
            endcase
        end 
    end 

endmodule
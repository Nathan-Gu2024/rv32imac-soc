module direct_mapped_cache 
    (
        input wire clk, rst,
        input wire [31:0] cpu_req_addr, cpu_write_data,
        input wire [3:0] mem_write_mask,
        input wire cpu_read_req, cpu_write_req, mem_ready,
        input wire [127:0] mem_read_data,
        output reg [31:0] cpu_read_data, mem_req_addr,
        output reg cpu_ready,
        output wire mem_req_valid
    );
    wire [3:0] offset = cpu_req_addr[3:0];
    wire [5:0] index = cpu_req_addr[9:4];
    wire [21:0] tag = cpu_req_addr [31:10];

    reg [127:0] data_array [0:63];
    reg [21:0] tag_array [0:63];
    reg [63:0] valid_array;

    wire is_hit = valid_array[index] && (tag_array[index] == tag);

    localparam IDLE = 1'b0;
    localparam FETCH = 1'b1;
    reg state, next_state;

    assign mem_req_valid = (state == FETCH);

    always @(posedge clk) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end 

    always @(*) begin
        next_state = state;
        cpu_ready = 1'b0;
        mem_req_addr = {tag, index, 4'b0};

        case (offset[3:2]) 
            2'b00: cpu_read_data = data_array[index][31:0];
            2'b01: cpu_read_data = data_array[index][63:32];
            2'b10: cpu_read_data = data_array[index][95:64];
            2'b11: cpu_read_data = data_array[index][127:96];
        endcase

        case (state) 
            IDLE:
                begin
                    if (cpu_read_req) begin
                        if (is_hit) begin
                            cpu_ready = 1'b1;
                        end else begin
                            next_state = FETCH;
                        end 
                    end else if (cpu_write_req) begin
                        cpu_ready = 1'b1;
                    end 
                end 
            FETCH:
                begin
                    if (mem_ready) begin
                        next_state = IDLE;
                    end 
                end 
        endcase
    end 

    always @(posedge clk) begin
        if (rst) begin
            integer i;
            for (i = 0; i < 64; i = i + 1)
                valid_array[i] <= 1'b0;
        end else if (state == FETCH && mem_ready) begin
            data_array[index] <= mem_read_data;
            tag_array[index]  <= tag;
            valid_array[index] <= 1'b1;
        end else if (state == IDLE && cpu_write_req) begin
            if (is_hit) begin
                case (offset[3:2])
                    2'b00: begin
                        if (mem_write_mask[0]) 
                            data_array[index][7:0] <= cpu_write_data[7:0];
                        if (mem_write_mask[1]) 
                            data_array[index][15:8] <= cpu_write_data[15:8];
                        if (mem_write_mask[2]) 
                            data_array[index][23:16] <= cpu_write_data[23:16];
                        if (mem_write_mask[3]) 
                            data_array[index][31:24] <= cpu_write_data[31:24];
                    end
                    2'b01: begin
                        if (mem_write_mask[0]) 
                            data_array[index][39:32] <= cpu_write_data[7:0];
                        if (mem_write_mask[1]) 
                            data_array[index][47:40] <= cpu_write_data[15:8];
                        if (mem_write_mask[2]) 
                            data_array[index][55:48] <= cpu_write_data[23:16];
                        if (mem_write_mask[3]) 
                            data_array[index][63:56] <= cpu_write_data[31:24];
                    end
                    2'b10: begin
                        if (mem_write_mask[0]) 
                            data_array[index][71:64] <= cpu_write_data[7:0];
                        if (mem_write_mask[1]) 
                            data_array[index][79:72] <= cpu_write_data[15:8];
                        if (mem_write_mask[2]) 
                            data_array[index][87:80] <= cpu_write_data[23:16];
                        if (mem_write_mask[3]) 
                            data_array[index][95:88] <= cpu_write_data[31:24];
                    end
                    2'b11: begin
                        if (mem_write_mask[0]) 
                            data_array[index][103:96] <= cpu_write_data[7:0];
                        if (mem_write_mask[1]) 
                            data_array[index][111:104] <= cpu_write_data[15:8];
                        if (mem_write_mask[2]) 
                            data_array[index][119:112] <= cpu_write_data[23:16];
                        if (mem_write_mask[3]) 
                            data_array[index][127:120] <= cpu_write_data[31:24];
                    end
                endcase
            end
        end 
    end 

endmodule
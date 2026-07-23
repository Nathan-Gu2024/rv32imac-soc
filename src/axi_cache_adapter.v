module axi_cache_adapter #(
    parameter ADDR_WIDTH = 32,
    parameter LINE_BITS = 128,
    parameter AXI_DATA_WIDTH = 64
)(
    input wire clk,
    input wire rst,

    // Cache-line side: one blocking 128-bit line transaction.
    input wire mem_req_valid,
    input wire mem_req_write,
    input wire [ADDR_WIDTH-1:0] mem_req_addr,
    input wire [LINE_BITS-1:0] mem_wline,
    output reg [LINE_BITS-1:0] mem_rline,
    output reg mem_ready,

    // AXI read address channel
    output reg [ADDR_WIDTH-1:0] m_axi_araddr,
    output reg [7:0] m_axi_arlen,
    output reg [2:0] m_axi_arsize,
    output reg [1:0] m_axi_arburst,
    output reg m_axi_arvalid,
    input wire m_axi_arready,

    // AXI read data channel
    input wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input wire m_axi_rvalid,
    input wire m_axi_rlast,
    output reg m_axi_rready,

    // AXI write address channel
    output reg [ADDR_WIDTH-1:0] m_axi_awaddr,
    output reg [7:0] m_axi_awlen,
    output reg [2:0] m_axi_awsize,
    output reg [1:0] m_axi_awburst,
    output reg m_axi_awvalid,
    input wire m_axi_awready,

    // AXI write data channel
    output reg [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output reg [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg m_axi_wlast,
    output reg m_axi_wvalid,
    input wire m_axi_wready,

    // AXI write response channel
    input wire [1:0] m_axi_bresp,
    input wire m_axi_bvalid,
    output reg m_axi_bready
);

    // This adapter assumes the cache line is an integer multiple of the AXI data width.
    localparam integer LINE_BYTES = LINE_BITS / 8;
    localparam integer BEATS_PER_LINE = LINE_BITS / AXI_DATA_WIDTH;
    localparam integer OFFSET_BITS = $clog2(LINE_BYTES);

    localparam [7:0] AXI_LEN = BEATS_PER_LINE - 1;

    // AXI SIZE is log2(bytes per beat).
    localparam [2:0] AXI_SIZE =
        (AXI_DATA_WIDTH == 32) ? 3'b010 :
        (AXI_DATA_WIDTH == 64) ? 3'b011 :
        (AXI_DATA_WIDTH == 128) ? 3'b100 :
                                  3'b000;

    localparam [2:0] IDLE = 3'd0;
    localparam [2:0] READ_ADDR = 3'd1;
    localparam [2:0] READ_DATA = 3'd2;
    localparam [2:0] WRITE_ADDR = 3'd3;
    localparam [2:0] WRITE_DATA = 3'd4;
    localparam [2:0] WRITE_RESP = 3'd5;
    localparam [2:0] DONE = 3'd6;

    reg [2:0] state, next_state;
    reg [7:0] beat_count;

    reg [ADDR_WIDTH-1:0] saved_addr;
    reg [LINE_BITS-1:0] saved_wline;
    reg saved_write;

    wire [ADDR_WIDTH-1:0] saved_line_addr =
        {saved_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            beat_count <= 8'd0;
            saved_addr <= {ADDR_WIDTH{1'b0}};
            saved_wline <= {LINE_BITS{1'b0}};
            saved_write <= 1'b0;
            mem_rline <= {LINE_BITS{1'b0}};
        end else begin
            state <= next_state;

            // Latch the line transaction. After this point the upstream request
            // may drop or change without corrupting the AXI transaction.
            if (state == IDLE && mem_req_valid) begin
                saved_addr <= mem_req_addr;
                saved_wline <= mem_wline;
                saved_write <= mem_req_write;
                beat_count <= 8'd0;
            end

            // Pack AXI read beats into the 128-bit cache line.
            if (state == READ_DATA && m_axi_rvalid && m_axi_rready) begin
                mem_rline[beat_count*AXI_DATA_WIDTH +: AXI_DATA_WIDTH] <= m_axi_rdata;
                beat_count <= beat_count + 8'd1;
            end

            // Advance write beat counter only when a W beat is accepted.
            if (state == WRITE_DATA && m_axi_wvalid && m_axi_wready) begin
                beat_count <= beat_count + 8'd1;
            end

            if (state == DONE) begin
                beat_count <= 8'd0;
            end
        end
    end

    always @(*) begin
        next_state = state;

        case (state)
            IDLE: 
                begin
                    if (mem_req_valid) begin
                        if (mem_req_write)
                            next_state = WRITE_ADDR;
                        else
                            next_state = READ_ADDR;
                    end
                end

            READ_ADDR: 
                begin
                    if (m_axi_arvalid && m_axi_arready)
                        next_state = READ_DATA;
                end

            READ_DATA:
                begin
                    if (m_axi_rvalid && m_axi_rready && m_axi_rlast)
                        next_state = DONE;
                end

            WRITE_ADDR: 
                begin
                    if (m_axi_awvalid && m_axi_awready)
                        next_state = WRITE_DATA;
                end

            WRITE_DATA:
                begin
                    if (m_axi_wvalid && m_axi_wready && m_axi_wlast)
                        next_state = WRITE_RESP;
                end

            WRITE_RESP: 
                begin
                    if (m_axi_bvalid && m_axi_bready)
                        next_state = DONE;
                end

            DONE: 
                begin
                    next_state = IDLE;
                end
            default: next_state = IDLE;
        endcase
    end

    always @(*) begin
        m_axi_araddr = saved_line_addr;
        m_axi_arlen = AXI_LEN;
        m_axi_arsize = AXI_SIZE;
        m_axi_arburst = 2'b01; // INCR
        m_axi_arvalid = 1'b0;

        m_axi_rready = 1'b0;

        m_axi_awaddr = saved_line_addr;
        m_axi_awlen = AXI_LEN;
        m_axi_awsize = AXI_SIZE;
        m_axi_awburst = 2'b01; // INCR
        m_axi_awvalid = 1'b0;

        m_axi_wdata = {AXI_DATA_WIDTH{1'b0}};
        m_axi_wstrb = {(AXI_DATA_WIDTH/8){1'b1}}; // full-line dirty eviction
        m_axi_wlast = 1'b0;
        m_axi_wvalid = 1'b0;

        m_axi_bready = 1'b0;

        mem_ready = 1'b0;
        case (state)
            READ_ADDR: 
                begin
                    m_axi_arvalid = 1'b1;
                end

            READ_DATA: 
                begin
                    m_axi_rready = 1'b1;
                end

            WRITE_ADDR: 
                begin
                    m_axi_awvalid = 1'b1;
                end
            WRITE_DATA: 
                begin
                    m_axi_wvalid = 1'b1;
                    m_axi_wdata = saved_wline[beat_count*AXI_DATA_WIDTH +: AXI_DATA_WIDTH];
                    m_axi_wlast = (beat_count == AXI_LEN);
                end
            WRITE_RESP: 
                begin
                    m_axi_bready = 1'b1;
                end
            DONE: 
                begin
                    // Pulse complete only after full read line is received or write response completes
                    mem_ready = 1'b1;
                end
        endcase
    end

endmodule

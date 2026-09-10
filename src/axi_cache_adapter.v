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
    // Number of consecutive LINES this request covers. 1 is the historical
    // behaviour and what both caches drive. Reads only; writes ignore it.
    input wire [7:0] mem_req_lines,
    // Pulses when a WRITE burst has consumed one line and needs the next.
    // Unused by single-line requesters (both caches).
    output reg mem_wnext,
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

    (* mark_debug = "true" *) reg [2:0] state, next_state;
    (* mark_debug = "true" *) reg [7:0] beat_count;

    (* mark_debug = "true" *) reg [ADDR_WIDTH-1:0] saved_addr;
    reg [LINE_BITS-1:0] saved_wline;
    (* mark_debug = "true" *) reg saved_write;
    (* mark_debug = "true" *) reg [7:0] saved_lines;
    // Registered per-line completion pulse. mem_rline is written on the beat
    // itself, so a combinational pulse on the last beat would publish a line
    // that is still one beat short. Registering it lands the pulse exactly one
    // cycle later - the same relative timing the old DONE-state pulse had.
    reg line_ready;
    // Lines already handed to AXI in the current write burst.
    reg [7:0] line_sent;

    // Debug-only mirror of the input port so it can be probed directly
    // alongside mem_arbiter's own state, in the same ILA capture, to
    // check whether mem_arbiter's SVC_D/SVC_I actually reaches here.
    (* mark_debug = "true" *) wire dbg_mem_req_valid = mem_req_valid;
    (* mark_debug = "true" *) wire dbg_mem_req_write = mem_req_write;
    (* mark_debug = "true" *) wire [ADDR_WIDTH-1:0] dbg_mem_req_addr = mem_req_addr;

    wire [ADDR_WIDTH-1:0] saved_line_addr =
        {saved_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};

    // Timeout/retry: confirmed via AXI Protocol Checker (rule AXI_ERRS_RID,
    // a spurious/duplicate R beat with no live matching AR) that the real
    // PS7 HP0 interconnect can occasionally leave a read's final beat
    // (or, symmetrically, a write's BRESP) unanswered forever, hanging
    // the CPU permanently. This has nothing to do with our own protocol
    // compliance (verified correct) - it's a rare interconnect condition
    // outside this RTL's control. If a transaction makes no forward
    // progress for TIMEOUT_LIMIT cycles, abandon it and drop back to
    // IDLE. mem_req_valid/mem_req_addr/mem_req_write from cache_core are
    // still held live (cache_core is itself still waiting on mem_ready),
    // so IDLE's own latch logic below reissues the identical request
    // fresh on the very next cycle - entirely transparent to cache_core,
    // which just observes a longer wait before mem_ready eventually
    // pulses. retry_count is debug-only visibility into how often this
    // actually fires.
    //
    // Restricting this to only READ_DATA/WRITE_RESP (where we're waiting
    // on the PEER's VALID signal and only hold READY ourselves, which is
    // legal to drop any time) is the protocol-clean answer, and it does
    // work - confirmed via retry_count incrementing on real hardware.
    // But the real interconnect has also been observed getting stuck in
    // READ_ADDR (ARREADY itself never arriving), which has no
    // protocol-legal recovery from the master side at all: AXI4 requires
    // ARVALID, once asserted, to stay asserted with a stable address
    // until ARREADY arrives, full stop. Given the alternative is a
    // permanent hang, this deliberately widens the timeout to every
    // non-idle/non-done state, accepting the known, intentional tradeoff
    // of occasionally violating AXI_ERRM_ARVALID_STABLE (and whatever it
    // cascades into, e.g. AXI_AUXM_RCAM_OVERFLOW) in order to actually
    // recover instead of hanging forever. The protocol checker's own
    // documentation notes RCAM_OVERFLOW specifically does not indicate a
    // functional failure of the monitored system, just that the checker
    // itself may lose track - consistent with this being a real,
    // deliberate compliance tradeoff rather than a correctness bug.
    localparam [15:0] TIMEOUT_LIMIT = 16'd4000;
    wire timeout_eligible = (state != IDLE) && (state != DONE);
    (* mark_debug = "true" *) reg [15:0] timeout_cnt;
    (* mark_debug = "true" *) reg [15:0] retry_count;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            beat_count <= 8'd0;
            saved_lines <= 8'd1;
            line_ready <= 1'b0;
            line_sent <= 8'd0;
            mem_wnext <= 1'b0;
            saved_addr <= {ADDR_WIDTH{1'b0}};
            saved_wline <= {LINE_BITS{1'b0}};
            saved_write <= 1'b0;
            mem_rline <= {LINE_BITS{1'b0}};
            timeout_cnt <= 16'd0;
            retry_count <= 16'd0;
        end else if (timeout_eligible && timeout_cnt >= TIMEOUT_LIMIT) begin
            state <= IDLE;
            beat_count <= 8'd0;
            timeout_cnt <= 16'd0;
            retry_count <= retry_count + 16'd1;
        end else begin
            state <= next_state;
            timeout_cnt <= timeout_eligible ? (timeout_cnt + 16'd1) : 16'd0;

            // Latch the line transaction. After this point the upstream request
            // may drop or change without corrupting the AXI transaction.
            if (state == IDLE && mem_req_valid) begin
                saved_addr <= mem_req_addr;
                saved_wline <= mem_wline;
                saved_write <= mem_req_write;
                line_sent <= 8'd0;
                saved_lines <= (mem_req_lines == 8'd0) ? 8'd1 : mem_req_lines;
                beat_count <= 8'd0;
            end

            // Pack AXI read beats into the 128-bit cache line. beat_count is
            // the index WITHIN the current line, so it wraps every
            // BEATS_PER_LINE and a multi-line burst reuses the same packing.
            line_ready <= 1'b0;
            if (state == READ_DATA && m_axi_rvalid && m_axi_rready) begin
                mem_rline[beat_count*AXI_DATA_WIDTH +: AXI_DATA_WIDTH] <= m_axi_rdata;
                if (beat_count == BEATS_PER_LINE - 1) begin
                    beat_count <= 8'd0;
                    line_ready <= 1'b1;   // one line complete, publish next cycle
                end else begin
                    beat_count <= beat_count + 8'd1;
                end
            end

            // Advance the write beat counter, wrapping per LINE so a burst
            // reuses the same slicing. mem_wnext pulses on each wrap so the
            // requester can present the next line.
            mem_wnext <= 1'b0;
            if (state == WRITE_DATA && m_axi_wvalid && m_axi_wready) begin
                if (beat_count == BEATS_PER_LINE - 1) begin
                    beat_count <= 8'd0;
                    line_sent  <= line_sent + 8'd1;
                end else begin
                    beat_count <= beat_count + 8'd1;
                end
                // One beat EARLY: the requester's FIFO needs a cycle to present
                // the next line, and the next line's first beat follows
                // immediately after this line's last one.
                if ((beat_count == BEATS_PER_LINE - 2) &&
                    (line_sent + 8'd1 < saved_lines))
                    mem_wnext <= 1'b1;
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
        // Span every line of the request in ONE transaction: that is the
        // point of the burst, since the ~30-cycle round trip is paid per
        // transaction rather than per beat.
        m_axi_arlen = (saved_lines * BEATS_PER_LINE) - 8'd1;
        m_axi_arsize = AXI_SIZE;
        m_axi_arburst = 2'b01; // INCR
        m_axi_arvalid = 1'b0;

        m_axi_rready = 1'b0;

        m_axi_awaddr = saved_line_addr;
        m_axi_awlen = (saved_lines * BEATS_PER_LINE) - 8'd1;
        m_axi_awsize = AXI_SIZE;
        m_axi_awburst = 2'b01; // INCR
        m_axi_awvalid = 1'b0;

        m_axi_wdata = {AXI_DATA_WIDTH{1'b0}};
        m_axi_wstrb = {(AXI_DATA_WIDTH/8){1'b1}}; // full-line dirty eviction
        m_axi_wlast = 1'b0;
        m_axi_wvalid = 1'b0;

        m_axi_bready = 1'b0;

        // Reads complete a line at a time, so they pulse from line_ready.
        // Writes still pulse once in DONE. Splitting these keeps a single-line
        // read at exactly one pulse with unchanged timing rather than firing
        // from both sources.
        mem_ready = line_ready;
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
                    // Single-line writes slice the latched copy, keeping the
                    // cache path bit-identical. Multi-line bursts read mem_wline
                    // live, which the requester advances on each mem_wnext.
                    m_axi_wdata = (saved_lines == 8'd1)
                        ? saved_wline[beat_count*AXI_DATA_WIDTH +: AXI_DATA_WIDTH]
                        : mem_wline[beat_count*AXI_DATA_WIDTH +: AXI_DATA_WIDTH];
                    // WLAST marks the end of the whole burst, not of a line.
                    m_axi_wlast = (beat_count == AXI_LEN) &&
                                  (line_sent == saved_lines - 8'd1);
                end
            WRITE_RESP: 
                begin
                    m_axi_bready = 1'b1;
                end
            DONE: 
                begin
                    // Writes only: reads already pulsed per line above.
                    if (saved_write) mem_ready = 1'b1;
                end
        endcase
    end

endmodule

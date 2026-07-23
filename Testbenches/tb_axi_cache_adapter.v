`timescale 1ns/1ps
`include "../src/axi_cache_adapter.v"

module tb_axi_cache_adapter;

    parameter AXI_DATA_WIDTH = 64;
    parameter ADDR_WIDTH     = 32;
    parameter LINE_BITS      = 128;

    localparam integer BEATS = LINE_BITS / AXI_DATA_WIDTH;

    reg clk;
    reg rst;

    // Cache-line side
    reg  mem_req_valid;
    reg  mem_req_write;
    reg  [ADDR_WIDTH-1:0] mem_req_addr;
    reg  [LINE_BITS-1:0]  mem_wline;
    wire [LINE_BITS-1:0]  mem_rline;
    wire mem_ready;

    // AXI read address
    wire [ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0] m_axi_arlen;
    wire [2:0] m_axi_arsize;
    wire [1:0] m_axi_arburst;
    wire m_axi_arvalid;
    reg  m_axi_arready;

    // AXI read data
    reg  [AXI_DATA_WIDTH-1:0] m_axi_rdata;
    reg  m_axi_rvalid;
    reg  m_axi_rlast;
    wire m_axi_rready;

    // AXI write address
    wire [ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0] m_axi_awlen;
    wire [2:0] m_axi_awsize;
    wire [1:0] m_axi_awburst;
    wire m_axi_awvalid;
    reg  m_axi_awready;

    // AXI write data
    wire [AXI_DATA_WIDTH-1:0]   m_axi_wdata;
    wire [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb;
    wire m_axi_wlast;
    wire m_axi_wvalid;
    reg  m_axi_wready;

    // AXI write response
    reg [1:0] m_axi_bresp;
    reg       m_axi_bvalid;
    wire      m_axi_bready;

    axi_cache_adapter #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BITS(LINE_BITS),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) uut (
        .clk(clk),
        .rst(rst),

        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_wline(mem_wline),
        .mem_rline(mem_rline),
        .mem_ready(mem_ready),

        .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),

        .m_axi_rdata(m_axi_rdata),
        .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rlast(m_axi_rlast),
        .m_axi_rready(m_axi_rready),

        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),

        .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),

        .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready)
    );

    always #5 clk = ~clk;

    task reset_inputs;
        begin
            mem_req_valid = 1'b0;
            mem_req_write = 1'b0;
            mem_req_addr  = {ADDR_WIDTH{1'b0}};
            mem_wline     = {LINE_BITS{1'b0}};

            m_axi_arready = 1'b0;
            m_axi_rdata   = {AXI_DATA_WIDTH{1'b0}};
            m_axi_rvalid  = 1'b0;
            m_axi_rlast   = 1'b0;

            m_axi_awready = 1'b0;
            m_axi_wready  = 1'b0;
            m_axi_bresp   = 2'b00;
            m_axi_bvalid  = 1'b0;
        end
    endtask

    task accept_read_address;
        begin
            wait(m_axi_arvalid === 1'b1);

            @(negedge clk);
            m_axi_arready = 1'b1;
            #1;
            if (m_axi_arvalid !== 1'b1) begin
                $display("FAIL: ARVALID dropped before ARREADY handshake");
                $finish;
            end

            @(posedge clk); // AR handshake
            @(negedge clk);
            m_axi_arready = 1'b0;
        end
    endtask

    task send_read_line;
        input [LINE_BITS-1:0] line;
        integer i;
        begin
            accept_read_address();

            for (i = 0; i < BEATS; i = i + 1) begin
                // Drive each R beat on the negedge and hold it stable
                // through the following posedge where the adapter samples it.
                @(negedge clk);
                m_axi_rvalid = 1'b1;
                m_axi_rdata  = line[i*AXI_DATA_WIDTH +: AXI_DATA_WIDTH];
                m_axi_rlast  = (i == BEATS-1);

                #1;
                while (m_axi_rready !== 1'b1) begin
                    @(negedge clk);
                    #1;
                end

                @(posedge clk); // R handshake
            end

            @(negedge clk);
            m_axi_rvalid = 1'b0;
            m_axi_rlast  = 1'b0;
            m_axi_rdata  = {AXI_DATA_WIDTH{1'b0}};
        end
    endtask

    task accept_write_address;
        begin
            wait(m_axi_awvalid === 1'b1);

            @(negedge clk);
            m_axi_awready = 1'b1;
            #1;
            if (m_axi_awvalid !== 1'b1) begin
                $display("FAIL: AWVALID dropped before AWREADY handshake");
                $finish;
            end

            @(posedge clk); // AW handshake
            @(negedge clk);
            m_axi_awready = 1'b0;
        end
    endtask

    task accept_write_line;
        input [LINE_BITS-1:0] expected_line;
        integer i;
        reg [AXI_DATA_WIDTH-1:0] expected_beat;
        reg expected_last;
        begin
            accept_write_address();

            for (i = 0; i < BEATS; i = i + 1) begin
                @(negedge clk);
                m_axi_wready = 1'b1;
                #1;

                while (m_axi_wvalid !== 1'b1) begin
                    @(negedge clk);
                    #1;
                end

                expected_beat = expected_line[i*AXI_DATA_WIDTH +: AXI_DATA_WIDTH];
                expected_last = (i == BEATS-1);

                if (m_axi_wdata !== expected_beat) begin
                    $display("FAIL: W beat %0d data mismatch. Got %h expected %h",
                             i, m_axi_wdata, expected_beat);
                    $finish;
                end

                if (m_axi_wlast !== expected_last) begin
                    $display("FAIL: W beat %0d WLAST mismatch. Got %b expected %b",
                             i, m_axi_wlast, expected_last);
                    $finish;
                end

                if (m_axi_wstrb !== {(AXI_DATA_WIDTH/8){1'b1}}) begin
                    $display("FAIL: WSTRB mismatch. Got %h", m_axi_wstrb);
                    $finish;
                end

                $display("   [AXI Slave] Accepted W beat %0d: Data = %h, WLAST = %b",
                         i, m_axi_wdata, m_axi_wlast);

                @(posedge clk); // W handshake
            end

            @(negedge clk);
            m_axi_wready = 1'b0;

            @(negedge clk);
            m_axi_bresp  = 2'b00;
            m_axi_bvalid = 1'b1;
            #1;
            while (m_axi_bready !== 1'b1) begin
                @(negedge clk);
                #1;
            end

            @(posedge clk); // B handshake
            @(negedge clk);
            m_axi_bvalid = 1'b0;
        end
    endtask

    task issue_cache_read;
        input [ADDR_WIDTH-1:0] addr;
        begin
            @(negedge clk);
            mem_req_valid = 1'b1;
            mem_req_write = 1'b0;
            mem_req_addr  = addr;
            mem_wline     = {LINE_BITS{1'b0}};
        end
    endtask

    task issue_cache_write;
        input [ADDR_WIDTH-1:0] addr;
        input [LINE_BITS-1:0]  line;
        begin
            @(negedge clk);
            mem_req_valid = 1'b1;
            mem_req_write = 1'b1;
            mem_req_addr  = addr;
            mem_wline     = line;
        end
    endtask

    task clear_cache_request_after_ready;
        begin
            wait(mem_ready === 1'b1);
            @(negedge clk);
            mem_req_valid = 1'b0;
            mem_req_write = 1'b0;
            mem_req_addr  = {ADDR_WIDTH{1'b0}};
            mem_wline     = {LINE_BITS{1'b0}};
        end
    endtask

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        reset_inputs();

        repeat (4) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;

        $display("Testing AXI_DATA_WIDTH = %0d", AXI_DATA_WIDTH);

        // Test 1: cache-line read
        $display("\n--- Test 1: Cache Read Miss ---");
        issue_cache_read(32'h8000_0018);

        wait(m_axi_arvalid === 1'b1);
        #1;
        if (m_axi_araddr !== 32'h8000_0010)
            $display("FAIL: ARADDR alignment got=%h expected=80000010", m_axi_araddr);
        else
            $display("PASS: ARADDR aligned to %h", m_axi_araddr);

        send_read_line(128'h1111_2222_3333_4444_5555_6666_7777_8888);

        wait(mem_ready === 1'b1);
        #1;
        if (mem_rline !== 128'h1111_2222_3333_4444_5555_6666_7777_8888) begin
            $display("FAIL: Read data mismatch. Got %h", mem_rline);
            $finish;
        end else begin
            $display("PASS: Read data assembled correctly.");
        end
        clear_cache_request_after_ready();

        // Test 2: cache-line writeback
        $display("\n--- Test 2: Cache Dirty Writeback ---");
        issue_cache_write(32'h8000_0024,
                          128'hDEAD_BEEF_CAFE_BAFE_0123_4567_89AB_CDEF);

        wait(m_axi_awvalid === 1'b1);
        #1;
        if (m_axi_awaddr !== 32'h8000_0020)
            $display("FAIL: AWADDR alignment got=%h expected=80000020", m_axi_awaddr);
        else
            $display("PASS: AWADDR aligned to %h", m_axi_awaddr);

        accept_write_line(128'hDEAD_BEEF_CAFE_BAFE_0123_4567_89AB_CDEF);

        wait(mem_ready === 1'b1);
        #1;
        $display("PASS: Write transaction completed after BVALID.");
        clear_cache_request_after_ready();

        $display("\nAll AXI adapter tests finished.");
        $finish;
    end

endmodule

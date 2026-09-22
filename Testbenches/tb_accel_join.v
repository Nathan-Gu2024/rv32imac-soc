`timescale 1ns/1ps
`include "../src/accel_port_join.v"

// accel_port_join against a single-port memory model of the kind the existing
// accelerator benches already use.
//
// What has to be true for this module to be safe to drop in front of eight
// working bench mocks and the legacy mem_arbiter path:
//
//   1. a read alone and a write alone behave exactly as the old single port did,
//      INCLUDING the asymmetric completion - per LINE for a read, once per
//      TRANSACTION for a write;
//   2. with both directions pending it SERIALISES rather than interleaving, and
//      neither direction is starved;
//   3. a completion is routed to the channel that owns the grant and never to
//      the other one - the property that makes the direction ambiguity
//      unreproducible inside this module;
//   4. mem_req_valid drops for at least one cycle between transactions, because
//      several bench mocks accept a new request on `mem_req_valid && !mem_ready`
//      and would otherwise run a duplicate.
//
// Addresses stay below 0x4000: the model indexes mem[addr>>4] into a 1024-entry
// array. Choosing test addresses without checking them against the model's bounds
// has produced three false "RTL bugs" in this repo already.
module tb_accel_join;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    // split side
    reg         rd_valid = 0;  reg [31:0] rd_addr = 0;  reg [7:0] rd_lines = 0;
    wire        rd_ready;      wire [127:0] rline;
    reg         wr_valid = 0;  reg [31:0] wr_addr = 0;  reg [7:0] wr_lines = 0;
    reg  [127:0] wline = 0;
    wire        wnext, wr_ready;

    // single-port side
    wire        m_valid, m_write;
    wire [31:0] m_addr;
    wire [7:0]  m_lines;
    wire [127:0] m_wline;
    reg         m_ready = 0, m_wnext = 0;
    reg  [127:0] m_rline = 0;

    accel_port_join DUT (
        .clk(clk), .rst(rst),
        .accel_rd_req_valid(rd_valid), .accel_rd_req_addr(rd_addr),
        .accel_rd_req_lines(rd_lines), .accel_rd_ready(rd_ready),
        .accel_rline(rline),
        .accel_wr_req_valid(wr_valid), .accel_wr_req_addr(wr_addr),
        .accel_wr_req_lines(wr_lines), .accel_wline(wline),
        .accel_wnext(wnext), .accel_wr_ready(wr_ready),
        .mem_req_valid(m_valid), .mem_req_write(m_write), .mem_req_addr(m_addr),
        .mem_req_lines(m_lines), .mem_wline(m_wline),
        .mem_ready(m_ready), .mem_wnext(m_wnext), .mem_rline(m_rline)
    );

    // ---- single-port model, in the style the accelerator benches use ----
    reg [127:0] mem [0:1023];
    integer mi;
    reg        busy = 0, is_wr = 0;
    reg [31:0] cur_addr;
    reg [7:0]  left;
    reg [3:0]  lat;
    reg        phase = 1'b0;
    integer    txn_count = 0;

    always @(posedge clk) begin
        if (rst) begin
            busy <= 0; m_ready <= 0; m_wnext <= 0; lat <= 0;
        end else begin
            m_ready <= 1'b0;
            m_wnext <= 1'b0;
            // NOT on a cycle this model is signalling completion. The requester
            // still holds valid while it observes ready, so accepting here
            // re-latches the transaction that just finished. That is the defect
            // that made tb_mm_accel_c_test replay a burst and stamp line 0 across
            // a result buffer; it is a property of the MODEL, not of the DUT, and
            // it belongs in every model in this repo.
            if (!busy && m_valid && !m_ready) begin
                busy      <= 1'b1;
                is_wr     <= m_write;
                cur_addr  <= m_addr;
                left      <= (m_lines == 8'd0) ? 8'd1 : m_lines;
                lat       <= 3;
                phase     <= 1'b0;
                txn_count <= txn_count + 1;
            end else if (busy) begin
                if (lat != 0) lat <= lat - 1;
                else if (is_wr) begin
                    // TWO phases per line, which is what gives the requester a
                    // cycle to present the next one after wnext. Consuming a line
                    // on the same cycle wnext is pulsed reads the line the
                    // requester has not replaced yet, and every line after the
                    // first carries line 0's bytes.
                    if (phase == 1'b0) begin
                        mem[cur_addr[13:4]] <= m_wline;
                        if (left > 8'd1) m_wnext <= 1'b1;
                        phase <= 1'b1;
                    end else begin
                        phase    <= 1'b0;
                        cur_addr <= cur_addr + 32'd16;
                        left     <= left - 8'd1;
                        if (left == 8'd1) begin
                            busy    <= 1'b0;
                            m_ready <= 1'b1;             // one pulse, whole txn
                        end
                    end
                end else begin
                    // Read: one completion per LINE.
                    m_rline  <= mem[cur_addr[13:4]];
                    m_ready  <= 1'b1;
                    cur_addr <= cur_addr + 32'd16;
                    left     <= left - 8'd1;
                    if (left == 8'd1) busy <= 1'b0;
                end
            end
        end
    end

    // ---- observers ----
    integer rd_pulses = 0, wr_pulses = 0, misroute = 0;
    integer both_granted = 0, valid_gap_ok = 0;
    reg m_valid_prev = 0;
    always @(posedge clk) begin
        if (!rst) begin
            if (rd_ready) begin
                rd_pulses = rd_pulses + 1;
                if (DUT.state != 2'd1) misroute = misroute + 1;
            end
            if (wr_ready) begin
                wr_pulses = wr_pulses + 1;
                if (DUT.state != 2'd2) misroute = misroute + 1;
            end
            // Strict serialisation: never both channels completing at once.
            if (rd_ready && wr_ready) both_granted = both_granted + 1;
            m_valid_prev <= m_valid;
        end
    end

    integer errors = 0;
    task check(input cond, input [8*76-1:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    // sticky completion captures, so a one-cycle pulse is never raced
    reg rd_done_seen = 0, wr_done_seen = 0, clr = 0;
    always @(posedge clk) begin
        if (clr) begin rd_done_seen <= 0; wr_done_seen <= 0; end
        else if (!rst) begin
            if (rd_ready && rd_pulses >= 3) rd_done_seen <= 1'b1;  // 4th line
            if (wr_ready) wr_done_seen <= 1'b1;
        end
    end

    reg [127:0] RL [0:15];
    integer rn = 0;
    always @(posedge clk) if (!rst && rd_ready) begin RL[rn] = rline; rn = rn + 1; end

    integer i, bad;
    reg [15:0] ln;

    initial begin
        for (mi = 0; mi < 1024; mi = mi + 1) mem[mi] = {4{16'hBEEF, mi[15:0]}};
        repeat (4) @(posedge clk);
        rst = 0;
        @(negedge clk);

        // ---- Test 1: 4-line read alone, one completion PER LINE ----
        $display("--- Test 1: 4-line read, per-line completion ---");
        rn = 0; rd_pulses = 0; txn_count = 0;
        rd_addr = 32'h0000_1000; rd_lines = 8'd4; rd_valid = 1'b1;
        wait (rn == 4);
        @(negedge clk); rd_valid = 1'b0;
        repeat (4) @(posedge clk);
        check(rd_pulses == 4, "read produced exactly 4 line completions");
        check(txn_count == 1, "and used exactly ONE single-port transaction");
        bad = 0;
        for (i = 0; i < 4; i = i + 1)
            if (RL[i] !== mem[(32'h1000 >> 4) + i]) bad = bad + 1;
        check(bad == 0, "read returned the right lines in order");

        // ---- Test 2: 4-line write alone, ONE completion ----
        $display("--- Test 2: 4-line write, one completion per transaction ---");
        @(negedge clk); clr = 1'b1; @(negedge clk); clr = 1'b0;
        wr_pulses = 0; txn_count = 0;
        ln = 0;
        wline = {4{16'h1234, ln}};
        wr_addr = 32'h0000_2000; wr_lines = 8'd4; wr_valid = 1'b1;
        while (!wr_done_seen) begin
            @(posedge clk); #1;
            if (wnext) begin ln = ln + 1; wline = {4{16'h1234, ln}}; end
        end
        @(negedge clk); wr_valid = 1'b0;
        repeat (4) @(posedge clk);
        check(wr_pulses == 1, "write produced exactly ONE completion");
        check(txn_count == 1, "and used exactly ONE single-port transaction");
        bad = 0;
        for (i = 0; i < 4; i = i + 1)
            if (mem[(32'h2000 >> 4) + i] !== {4{16'h1234, i[15:0]}}) bad = bad + 1;
        check(bad == 0, "write placed every line at its own address");

        // ---- Test 3: both pending -> serialised, neither starved ----
        $display("--- Test 3: both directions pending ---");
        @(negedge clk); clr = 1'b1; @(negedge clk); clr = 1'b0;
        rn = 0; rd_pulses = 0; wr_pulses = 0; txn_count = 0;
        misroute = 0; both_granted = 0;
        ln = 0;
        wline = {4{16'h5A5A, ln}};
        rd_addr = 32'h0000_1400; rd_lines = 8'd4;
        wr_addr = 32'h0000_2400; wr_lines = 8'd4;
        rd_valid = 1'b1; wr_valid = 1'b1;
        fork
            begin
                while (!wr_done_seen) begin
                    @(posedge clk); #1;
                    if (wnext) begin ln = ln + 1; wline = {4{16'h5A5A, ln}}; end
                end
                @(negedge clk); wr_valid = 1'b0;
            end
            begin
                while (rd_pulses < 4) @(posedge clk);
                @(negedge clk); rd_valid = 1'b0;
            end
        join
        repeat (6) @(posedge clk);

        check(rd_pulses == 4, "read still got exactly 4 line completions");
        check(wr_pulses == 1, "write still got exactly ONE completion");
        check(misroute == 0, "every completion went to the channel holding the grant");
        check(both_granted == 0, "the two channels never completed on the same cycle");
        check(txn_count == 2, "exactly two single-port transactions, serialised");
        bad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (RL[i] !== mem[(32'h1400 >> 4) + i]) bad = bad + 1;
            if (mem[(32'h2400 >> 4) + i] !== {4{16'h5A5A, i[15:0]}}) bad = bad + 1;
        end
        check(bad == 0, "and both transfers carried correct data");

        $display("");
        if (errors == 0) $display("=== ACCEL JOIN PASSED ===");
        else             $display("=== %0d ACCEL JOIN ERROR(S) ===", errors);
        $finish;
    end

    initial begin
        #200_000;
        $display("TIMEOUT - a transfer never completed");
        $display("  state=%0d rd_valid=%b wr_valid=%b rd_pulses=%0d wr_pulses=%0d",
                 DUT.state, rd_valid, wr_valid, rd_pulses, wr_pulses);
        $finish;
    end
endmodule

`timescale 1ns/1ps

// Phase 1 macro bring-up vehicle: a self-checking march test wrapped around
// a single sky130 SRAM macro.
//
// Its purpose is to prove the OpenLane macro flow end to end - EXTRA_LEFS /
// EXTRA_LIBS / EXTRA_GDS_FILES resolution, manual macro placement, PDN
// hookup over a hard block, and timing through a blackbox with real .lib
// arcs - before any of that risk is taken on inside the cache. It is
// deliberately small; the counter, comparator and FSM around the macro
// exist so the placer has genuine standard cells to work with instead of a
// bare macro wired straight to I/O, which would exercise none of the
// interaction that actually fails first.
//
// The wrapper it instantiates (sram_sky130) is the real Phase 2 building
// block, so whatever this run teaches about the macro carries directly into
// the dcache conversion rather than being thrown away.
module sram_test (
    input  wire clk,
    input  wire rst,
    input  wire start,
    output reg  done,
    output reg  pass
);

    localparam AW = 9;
    localparam LAST = {AW{1'b1}};   // 511

    localparam S_IDLE  = 3'd0;
    localparam S_WRITE = 3'd1;
    localparam S_READ  = 3'd2;
    localparam S_DRAIN = 3'd3;      // catch the final read's data
    localparam S_DONE  = 3'd4;

    reg [2:0]    state;
    reg [AW-1:0] addr;

    // Every data bit depends on the address, so a stuck address line and a
    // swapped data bit both surface as a mismatch rather than aliasing to a
    // value that happens to be correct.
    function [31:0] pattern(input [AW-1:0] a);
        pattern = {7'b0, a, 7'b0, a} ^ 32'hA5A5_5A5A;
    endfunction

    // During the fill phase the read port is pointed away from the address
    // being written. Port 1 is always selected, so leaving it on `addr`
    // would collide with port 0 every single cycle and bury the log in the
    // model's same-address read-during-write warnings - which are real, and
    // worth being able to see when they matter. ~addr can never equal addr.
    wire [AW-1:0] raddr = (state == S_WRITE) ? ~addr : addr;

    wire [31:0] rdata;
    wire        we    = (state == S_WRITE);
    wire [31:0] wdata = pattern(addr);

    sram_sky130 u_sram (
        .clk   (clk),
        .raddr (raddr),
        .rdata (rdata),
        .we    (we),
        .waddr (addr),
        .wdata (wdata),
        .wmask (4'hF)
    );

    // The read is registered: an address presented in cycle N returns data
    // in N+1, so the expected value has to be delayed to match.
    reg [AW-1:0] rd_addr_d;
    reg          rd_valid_d;

    always @(posedge clk) begin
        if (rst) begin
            rd_addr_d  <= {AW{1'b0}};
            rd_valid_d <= 1'b0;
        end else begin
            rd_addr_d  <= addr;
            rd_valid_d <= (state == S_READ);
        end
    end

    // != rather than !== because this has to synthesize; there are no X's on
    // silicon. X-propagation checking belongs to tb_sram_wrapper.v, which
    // compares with !== and can therefore catch a read that returned nothing.
    wire mismatch = rd_valid_d && (rdata != pattern(rd_addr_d));

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            addr  <= {AW{1'b0}};
            done  <= 1'b0;
            pass  <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    pass <= 1'b1;
                    addr <= {AW{1'b0}};
                    if (start) state <= S_WRITE;
                end

                S_WRITE: begin
                    if (addr == LAST) begin
                        addr  <= {AW{1'b0}};
                        state <= S_READ;
                    end else begin
                        addr <= addr + 1'b1;
                    end
                end

                S_READ: begin
                    if (mismatch) pass <= 1'b0;
                    if (addr == LAST) state <= S_DRAIN;
                    else              addr <= addr + 1'b1;
                end

                S_DRAIN: begin
                    // Data for the last address presented in S_READ lands here.
                    if (mismatch) pass <= 1'b0;
                    state <= S_DONE;
                end

                S_DONE: begin
                    done <= 1'b1;
                    if (!start) state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

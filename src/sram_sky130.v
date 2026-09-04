`timescale 1ns/1ps

// Wrapper presenting the sky130 OpenRAM 1RW+1R macro through the
// active-high, always-reading interface the caches in this design are
// written against. One instance is 2KB: 512 words x 32 bits, byte-maskable.
//
// PORT ASSIGNMENT IS LOAD-BEARING, not arbitrary:
//
//     Port 1 (read-only)  -> the CPU read path. Selected every cycle.
//     Port 0 (read/write) -> writes only (store hits and refill).
//
// The reason is a property of the OpenRAM macro that inferred Xilinx BRAM
// does not have. In the behavioral model, dout0 is driven to 32'bx on EVERY
// posedge of clk0, unconditionally:
//
//     always @(posedge clk0) begin
//         csb0_reg = csb0; ...
//         #(T_HOLD) dout0 = 32'bx;        // <- not inside any if
//
// and is only restored by the read block when that cycle happened to be a
// read (!csb0_reg && web0_reg). So a write cycle, or any deselected cycle,
// leaves dout0 at X until the next read. dcache_bram.v assumes its output
// registers HOLD across stalls and across store cycles - an assumption BRAM
// satisfies for free. Routing reads through port 1, which has no write
// block and therefore no X injection, keeps that assumption true instead of
// requiring the cache to be restructured around it.
//
// Cross-port read-during-write at the same address is UNDEFINED here, as it
// was on BRAM; the model prints an explicit warning when it happens. The
// store->load bypass in dcache_bram.v is what makes that safe. It is a
// requirement of this memory, not a leftover from the Xilinx flow, and the
// mutation test that proves it earns its keep still applies unchanged.
//
// Power: vccd1/vssd1 are connected physically by FP_PDN_MACRO_HOOKS rather
// than through RTL ports. That is sufficient for PnR and DRC. Enabling LVS
// later requires threading USE_POWER_PINS ports through this wrapper so the
// netlist carries the connection explicitly.
module sram_sky130 (
    input  wire        clk,

    // Read port - permanently selected, one-cycle registered read.
    input  wire [8:0]  raddr,
    output wire [31:0] rdata,

    // Write port - byte-maskable, active high.
    input  wire        we,
    input  wire [8:0]  waddr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wmask
);

    // dout0 is intentionally left unconnected (see header), so the open-port
    // warning is suppressed below - a deliberate choice here, not an
    // oversight. Note that a comment line whose first word is the linter's
    // own name is parsed as a malformed pragma, which is why this one is
    // worded around it.
    /* verilator lint_off PINCONNECTEMPTY */
    // VERBOSE=0 is not cosmetic. The PDK behavioral model $displays a line on
    // EVERY read and EVERY write of every instance. With five instances in
    // the D-cache that is millions of lines across a CoreMark run, and
    // because the model's $display emits newlines into the middle of the
    // testbench's character-at-a-time UART output, it does not merely add
    // noise - it shreds the program's stdout, so the CoreMark CRC lines can
    // no longer be read at all. The same-address read-during-write WARNING
    // is NOT gated by this and still prints, which is the one message worth
    // keeping. The synthesis blackbox stub declares VERBOSE too, so this
    // override is legal on both paths.
    sky130_sram_2kbyte_1rw1r_32x512_8 #(
        .VERBOSE (0)
    ) u_macro (
        // Port 0: writes only. Both selects are active low.
        .clk0   (clk),
        .csb0   (~we),
        .web0   (~we),
        .wmask0 (wmask),
        .addr0  (waddr),
        .din0   (wdata),
        .dout0  (),          // deliberately unused - see header

        // Port 1: read, held selected so rdata is always the last address read.
        .clk1   (clk),
        .csb1   (1'b0),
        .addr1  (raddr),
        .dout1  (rdata)
    );
    /* verilator lint_on PINCONNECTEMPTY */

endmodule

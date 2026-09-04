/// sta-blackbox
`timescale 1ns/1ps

// The marker on the first line is required by OpenSTA, which reads this file
// as a GATE-LEVEL NETLIST and whose parser does not accept a parameter list -
// the `#(parameter VERBOSE = 0)` below is a syntax error to it:
//
//   Error while reading src/sky130_sram_blackbox.v:
//   Make sure that this a gate-level netlist not an RTL file
//   line 35, syntax error, unexpected '#', expecting ';' or '('
//
// `/// sta-blackbox` tells OpenSTA to skip the file and blackbox the modules
// in it, which is exactly right: timing for this macro comes from its .lib
// via EXTRA_LIBS, never from here.

// Interface-only stub for the sky130 OpenRAM macro, for SYNTHESIS AND LINT.
//
// The PDK ships a behavioral model at
//   $PDK_ROOT/$PDK/libs.ref/sky130_sram_macros/verilog/
// but that file cannot be handed to OpenLane as VERILOG_FILES_BLACKBOX,
// because the linter elaborates it rather than blackboxing it, and the
// model's simulation timing controls (#(T_HOLD), #(DELAY)) then trip the
// flow's "Timing constructs found in the RTL" check and abort at step 0.
//
// So the two consumers get different files, deliberately:
//
//   synthesis / lint  ->  THIS stub (ports only, no body, no delays)
//   simulation        ->  the PDK behavioral model, passed directly to
//                         iverilog by tb_sram_wrapper.v
//
// Timing for STA comes from the macro's .lib via EXTRA_LIBS, and the
// physical view from EXTRA_LEFS / EXTRA_GDS_FILES. Nothing here contributes
// timing or area - an empty body is the entire point.
//
// The port list must stay byte-identical to the real macro's. If it drifts,
// synthesis silently builds against the wrong interface while simulation
// keeps using the right one, and the two only disagree after place-and-route.
// An empty body means every port reads as unused or undriven, which is the
// definition of a blackbox rather than a defect. Suppressed here so the
// warnings do not accumulate - Phase 2 instantiates five of these, and 55
// spurious warnings would hide a real one.
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNDRIVEN */
// VERBOSE exists here only so that sram_sky130.v's `#(.VERBOSE(0))` override
// - which it needs to silence the simulation model's per-access $display -
// is legal against this stub too. It has no meaning for a hard macro and
// affects nothing in synthesis.
module sky130_sram_2kbyte_1rw1r_32x512_8 #(
    parameter VERBOSE = 0
) (
    // Port 0: read/write
    input  wire        clk0,
    input  wire        csb0,
    input  wire        web0,
    input  wire [3:0]  wmask0,
    input  wire [8:0]  addr0,
    input  wire [31:0] din0,
    output wire [31:0] dout0,

    // Port 1: read only
    input  wire        clk1,
    input  wire        csb1,
    input  wire [8:0]  addr1,
    output wire [31:0] dout1
);
endmodule
/* verilator lint_on UNDRIVEN */
/* verilator lint_on UNUSEDSIGNAL */

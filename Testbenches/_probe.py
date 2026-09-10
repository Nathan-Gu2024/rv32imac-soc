#!/usr/bin/env python3
r"""Temporary probe on the result-DMA state, to find where it stalls."""
import sys

P = "tb_mm_accel_c_test.v"
s = open(P).read()

if "dbg_dma" in s:
    print("  probe already present")
    sys.exit(0)

probe = r"""
    // ---- TEMPORARY PROBE: result-DMA stall diagnosis ----
    integer dbg_dma = 0;
    reg dbg_prev = 1'b0;
    always @(posedge clk) begin
        dbg_prev <= DUT.ACCEL.dmaBusy;
        if (DUT.ACCEL.dmaBusy && !dbg_prev)
            $display("[dbg] DMA start t=%0t", $time);
        if (DUT.ACCEL.dmaBusy) begin
            dbg_dma = dbg_dma + 1;
            if (dbg_dma % 400 == 1)
                $display("[dbg] t=%0t busy=%b fillLeft=%0d cnt=%0d deqv=%b reqv=%b lines=%0d rdy=%b wnext=%b sent=%0d",
                         $time, DUT.ACCEL.dmaBusy, DUT.ACCEL.fillLeft,
                         DUT.ACCEL._lineFifo_io_count, DUT.ACCEL._lineFifo_io_deq_valid,
                         accel_mem_req_valid, accel_mem_req_lines,
                         accel_mem_ready, accel_mem_wnext, DUT.ACCEL.dmaSent);
        end
    end
"""

anchor = "    initial clk = 0;"
if anchor not in s:
    sys.exit("ANCHOR MISSING")
s = s.replace(anchor, probe + "\n" + anchor, 1)
open(P, "w").write(s)
print("  probe installed")

addi x1, x0, 100
addi x2, x0, 4      # address 4
sw   x1, 0(x2)      # mem[4] = 100
addi x3, x0, 0      # no dependency on load
lw   x4, 0(x2)      # x4 = 100 — no load-use hazard (x3 between)
add  x5, x4, x1     # x5 = 200
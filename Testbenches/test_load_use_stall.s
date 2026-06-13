# Store a value first so we can load it back
addi x1, x0, 42
addi x2, x0, 0      # base address = 0
sw   x1, 0(x2)      # mem[0] = 42
lw   x3, 0(x2)      # x3 = 42  (load)
add  x4, x3, x0     # depends on x3 — load-use stall must fire, x4 = 42
addi x5, x3, 1      # x5 = 43
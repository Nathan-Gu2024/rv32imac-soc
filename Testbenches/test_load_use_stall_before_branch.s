addi x1, x0, 5
addi x2, x0, 0
sw   x1, 0(x2)
lw   x3, 0(x2)      # load
beq  x3, x1, pass   # load-use stall + branch on loaded value
addi x4, x0, 99     # should NOT execute
pass:
addi x4, x0, 1      # x4 = 1
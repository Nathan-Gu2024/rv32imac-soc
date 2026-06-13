addi x1, x0, 5
addi x2, x0, 3
bne  x1, x2, skip   # branch taken (5 != 3), skip the next
addi x3, x0, 99     # should NOT execute
skip:
addi x3, x0, 1      # x3 = 1
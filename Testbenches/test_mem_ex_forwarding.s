addi x1, x0, 10
addi x2, x0, 5
add  x3, x1, x2    # x3 = 15  (goes through EX, then MEM)
addi x4, x0, 1
add  x5, x3, x4    # x3 comes from MEM/WB → forward, x5 = 16
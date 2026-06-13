addi x1, x0, 7
addi x2, x1, 0      # x2 = 7, depends on x1 (forwarding)
beq  x1, x2, equal  # branch on forwarded value — should be taken
addi x3, x0, 99     # should NOT execute
equal:
addi x3, x0, 1      # x3 = 1
addi x1, x0, 0        # x1 = 0
      jal  x2, target       # jump to target, x2 = address of instruction below
      addi x1, x1, 1        # x1 = 1 + 1 = 2! 
      jal  x0, end          # JUMP over the traps to the end of the program!
      
      addi x1, x0, 99       # TRAP 1: should NOT execute (skipped by jal)
      
target:
      addi x1, x1, 1        # x1 = 0 + 1 = 1
      jalr x3, x2, 0        # jump back to the instruction saved in x2
      
      addi x1, x1, 10       # TRAP 2: should NOT execute (skipped by jalr)
      
end:
      nop                   # Safe landing spot for the program to idle
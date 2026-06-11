# EX to EX
addi x1, x0, 10      
add  x2, x1, x1      # x1 forwarded from EX/MEM

# MEM to EX
addi x3, x0, 5       
nop                  
add  x4, x3, x3      # x3 forwarded from MEM/WB

# Vector Hazard
addi x5, x0, 2
addi x6, x0, 3
add  x7, x5, x6      # Forward x5 from MEM/WB, x6 from EX/MEM
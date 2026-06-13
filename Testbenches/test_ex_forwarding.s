addi x1, x0, 10
add  x2, x1, x1    # depends on x1 — EX→EX forward, x2 = 20
add  x3, x2, x1    # depends on x2 — EX→EX forward, x3 = 30
add  x4, x3, x2    # depends on x3 — EX→EX forward, x4 = 50
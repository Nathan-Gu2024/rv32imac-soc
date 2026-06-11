# --- Test 3A: Branch Taken (Flush) ---
addi x1, x0, 5
addi x2, x0, 5
beq  x1, x2, target   # Branch is TAKEN
addi x3, x0, 99       # SHADOW INSTRUCTION 1 (Must be flushed!)
addi x4, x0, 99       # SHADOW INSTRUCTION 2 (If your branch resolves in EX)

target:
addi x5, x0, 123      # x5 should successfully become 123

# --- Test 3B: Branch Not Taken (No Flush) ---
addi x2, x0, 6
beq  x1, x2, target2  # Branch is NOT TAKEN
addi x6, x0, 7        # Should execute normally!
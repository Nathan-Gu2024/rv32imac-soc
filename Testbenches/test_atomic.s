# Setup: Initialize memory
li t0, 0x100      # t0 = Lock address in memory (e.g., 0x100)
li t1, 0          # t1 = Unlocked state (0)
sw t1, 0(t0)      # Initialize the lock address to 0
li t2, 1          # t2 = Locked state (1)

# Succeed
acquire_success:
    lr.w t3, (t0)     # Load-Reserved from 0x100 into t3. (Monitor Valid = 1)
    bnez t3, acquire_success # If lock is already 1, keep trying (spinlock)
    
    sc.w t4, t2, (t0) # Store-Conditional: Write 1 to 0x100. Write success code to t4.
    bnez t4, acquire_success # If t4 != 0, the SC failed. Try again.

    # If we get here, t4 == 0 and memory 0x100 == 1. We hold the lock!
    sw t1, 0(t0)      # Release the lock: write 0 back to 0x100

# Fail
acquire_fail:
    lr.w t3, (t0)     # Load-Reserved from 0x100 into t3. (Monitor Valid = 1)
    
    # SIMULATING INTERFERENCE: 
    # A standard store to ANY address should invalidate the reservation lock.
    sw t1, 4(t0)      # Store 0 to 0x104. (Monitor Valid MUST become 0 here!)

    sc.w t4, t2, (t0) # Store-Conditional: Attempt to write 1 to 0x100.
    
    # Because of the dummy store above, this SC MUST fail. 
    # It should not write to memory, and it must return a non-zero value to t4 (usually 1).
    beqz t4, hardware_failure # If t4 == 0, your CPU failed the test!

    # Test Passed! Loop infinitely.
success_loop:
    j success_loop

hardware_failure:
    j hardware_failure
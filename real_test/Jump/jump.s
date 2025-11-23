# jump_test.s
    .text
    jal x1, target
    addi x2, x0, 99   # skipped
target:
    addi x2, x0, 7
    # now test JALR (use x3 as target)
    addi x3, x0, 8
    addi x4, x0, 0
    jalr x4, x3, 0    # x4 <- return address, jump to x3 (which is 8)
    addi x5, x0, 1    # skipped if jump

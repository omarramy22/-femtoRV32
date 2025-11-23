# itype_test.s
    .text
    addi x1, x0, 10
    addi x2, x1, 5    # x2 = 15
    ori  x3, x1, 1    # x3 = 11
    xori x4, x1, 2    # x4 = 8  (10 ^ 2 = 8)
    slti x5, x1, 11   # x5 = 1

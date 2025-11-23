# rtype_test.s
    .text
    addi x1, x0, 10
    addi x2, x0, 3
    add  x3, x1, x2    # x3 = 13
    sub  x4, x1, x2    # x4 = 7
    or   x5, x1, x2
    and  x6, x1, x2

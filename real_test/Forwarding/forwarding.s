# forwarding_test.s
    .text
    addi x1, x0, 4
    addi x2, x0, 6
    add  x5, x1, x2     # x5 = 10
    sub  x6, x5, x1     # should forward x5 (EX->EX)

# branch_test.s
    .text
    addi x1, x0, 5
    addi x2, x0, 5
    beq  x1, x2, label
    addi x3, x0, 99   # should be skipped
label:
    addi x3, x0, 7

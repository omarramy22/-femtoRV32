# bigtask_test.s
    .text
    addi x1, x0, 2
    addi x2, x0, 3
    add  x3, x1, x2      # 5
    addi x4, x3, 7       # 12
    sw   x4, 4(x0)       # store 12 at mem[4]
    lw   x5, 4(x0)       # load back -> 12
    add  x6, x5, x1      # 14
    beq  x6, x0, skip
    addi x7, x0, 9
skip:
    addi x8, x0, 11

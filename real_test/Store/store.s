# memory_test.s
    .text
    addi x1, x0, 50
    sw   x1, 0(x0)
    lw   x2, 0(x0)

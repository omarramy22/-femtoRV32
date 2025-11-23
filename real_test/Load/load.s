# loaduse_test.s
    .text
    addi x2, x0, 8      # base
    sw   x2, 0(x0)      # store 8 at mem[0]
    lw   x1, 0(x0)      # load x1 <- 8
    add  x3, x1, x2     # dependent on load; requires stall

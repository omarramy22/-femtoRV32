module tb_rtype;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));
    always #5 clk = ~clk;

    initial begin
        $display("===== R-TYPE ALU TEST =====");

        // program:
        //   addi x1, x0, 10
        //   addi x2, x0, 3
        //   add  x3, x1, x2    # x3 = 13
        //   sub  x4, x1, x2    # x4 = 7
        //   or   x5, x1, x2
        //   and  x6, x1, x2

        #10 rst = 0;

        repeat (25) #10;

        $display("x3 = %0d (expect 13)", dut.rf.regs[3]);
        $display("x4 = %0d (expect 7)", dut.rf.regs[4]);
        $display("x5 = %0d", dut.rf.regs[5]);
        $display("x6 = %0d", dut.rf.regs[6]);

        $finish;
    end
endmodule

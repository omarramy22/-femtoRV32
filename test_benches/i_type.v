module tb_itype;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));
    always #5 clk = ~clk;

    initial begin
        $display("===== I-TYPE ALU TEST =====");

        // program:
        //   addi x1, x0, 10
        //   addi x2, x1, 5   # 15
        //   ori  x3, x1, 1
        //   xori x4, x1, 2
        //   slti x5, x1, 11  # 1

        #10 rst = 0;
        repeat (20) #10;

        $display("x2 = %0d (expect 15)", dut.rf.regs[2]);
        $display("x3 = %0d", dut.rf.regs[3]);
        $display("x4 = %0d", dut.rf.regs[4]);
        $display("x5 = %0d (expect 1)", dut.rf.regs[5]);

        $finish;
    end
endmodule

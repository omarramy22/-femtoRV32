module tb_branch;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));
    always #5 clk = ~clk;

    initial begin
        $display("===== BRANCH TEST =====");

        //   addi x1, x0, 5
        //   addi x2, x0, 5
        //   beq  x1, x2, label   # taken
        //   addi x3, x0, 99      # MUST BE SKIPPED
        // label:
        //   addi x3, x0, 7

        #10 rst = 0;
        repeat (35) #10;

        $display("x3 = %0d (expect 7)", dut.rf.regs[3]);

        $finish;
    end
endmodule

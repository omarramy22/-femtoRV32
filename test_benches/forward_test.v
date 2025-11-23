module tb_forwarding;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));
    always #5 clk = ~clk;

    initial begin
        $display("===== FORWARDING TEST =====");

        //   add x5, x1, x2
        //   sub x6, x5, x3    # forwarded

        #10 rst = 0;

        repeat (30) #10;

        $display("x6 = %0d (expect x1+x2-x3)", dut.rf.regs[6]);

        $finish;
    end
endmodule

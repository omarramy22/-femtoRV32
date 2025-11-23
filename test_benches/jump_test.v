module tb_jump;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));
    always #5 clk = ~clk;

    initial begin
        $display("===== JAL / JALR TEST =====");

        //   jal x1, target
        //   addi x2, x0, 99   # skipped
        // target:
        //   addi x2, x0, 7

        #10 rst = 0;
        repeat (40) #10;

        $display("x1 = link = %0d (PC+4)", dut.rf.regs[1]);
        $display("x2 = %0d (expect 7)", dut.rf.regs[2]);

        $finish;
    end
endmodule

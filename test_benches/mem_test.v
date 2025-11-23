module tb_memory;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));
    always #5 clk = ~clk;

    initial begin
        $display("===== MEMORY TEST =====");

        // Store then load the value back:
        //   addi x1, x0, 50
        //   sw   x1, 0(x0)
        //   lw   x2, 0(x0)

        #10 rst = 0;

        repeat (40) #10;

        $display("x2 = %0d (expect 50)", dut.rf.regs[2]);

        $finish;
    end
endmodule
